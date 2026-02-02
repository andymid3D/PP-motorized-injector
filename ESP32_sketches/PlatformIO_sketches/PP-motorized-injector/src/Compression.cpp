#include "Compression.h"
#include "config.h"
#include "MotorWrapper.h"
#include "GPTimer.h"  // Add GPTimer include
#include "injector_fsm.h"  // For commonInjectParams_t
#include "BroadcastDataStore.h"  // For BDS data access

extern commonInjectParams_t commonParams;  // From main.cpp
extern GPTimer hwTimer;  // Add GPTimer external declaration

namespace Compression {
    // ===== STATIC STATE VARIABLES =====
    static CompressionMode currentMode = MODE_1_TRAVEL;
    static enum {
        PRESSURE_CHECK,      // Check sensor (first ms)
        TRAVEL_DOWN,         // MODE 1: Travel until contact (skip for MODE 2)
        TORQUE_RAMP,         // Apply torque ramp
        DONE
    } step = DONE;
    
    static bool stateEntry = false;
    static uint64_t stateEnterTime = 0;      // Use uint64_t to match GPTimer
    static uint64_t stepTimer = 0;             // Use uint64_t to match GPTimer
    static bool complete = false;
    static bool isErrorFlag = false;
    static bool isTimeoutFlag = false;
    static bool pressureSensorChecked = false;
    static uint64_t lastCommandTime = 0;      // Use uint64_t to match GPTimer
    
    // ===== BEGIN: Initialize with mode selection =====
    void begin(CompressionMode mode) {
        currentMode = mode;
        stateEntry = true;
        stateEnterTime = hwTimer.micros() / 1000;  // CRITICAL: Compression timing - must use GPTimer
        stepTimer = hwTimer.micros() / 1000;  // CRITICAL: Compression timing - must use GPTimer
        complete = false;
        isErrorFlag = false;
        isTimeoutFlag = false;
        pressureSensorChecked = false;
        lastCommandTime = hwTimer.micros() / 1000;  // CRITICAL: Compression timing - must use GPTimer
        
        // Start with pressure check for MODE 1, direct to TORQUE_RAMP for MODE 2
        if (mode == MODE_1_TRAVEL) {
            step = PRESSURE_CHECK;
        } else {
            step = TORQUE_RAMP;  // MODE 2: skip travel, go to torque ramp
        }
    }
    
    // ===== UPDATE: Non-blocking compression logic =====
    bool update(CanBusHandlerV2& motor) {
        uint64_t now = hwTimer.micros() / 1000;  // CRITICAL: Compression timing - must use GPTimer
        uint64_t elapsed = now - stateEnterTime;
        uint64_t stepElapsed = now - stepTimer;
        
        // ===== STEP 0: PRESSURE CHECK (First ms, MODE 1 only) =====
        if (step == PRESSURE_CHECK) {
            if (stateEntry) {
                // TODO: Read HX711 pressure sensor
                // For now: user button press will confirm mould/nozzleblock is in place
                // Pressure sensor check happens here as backup
                pressureSensorChecked = true;
                stateEntry = false;
            }
            
            // Move to next step after minimal delay
            if (stepElapsed > 50) {
                step = TRAVEL_DOWN;
                stepTimer = now;
                stateEntry = true;  // CRITICAL: Reset stateEntry for next step
            }
            return false;
        }
        
        // ===== STEP 1: TRAVEL DOWN until plastic contact (MODE 1 only) =====
        if (step == TRAVEL_DOWN) {
            if (stateEntry) {
                // Queue all commands - ring buffer handles timing
                MotorWrapper::setMotorLimits(motor, COMPRESS_TRAVEL_VEL_LIMIT, REFILL_CURRENT_LIMIT, MODULE_COMPRESSION, "Compress Travel");
                MotorWrapper::setModeAndMove(motor, 1, 6, COMPRESS_TRAVEL_TORQUE, MODULE_COMPRESSION, "Compress Travel Down Torque");
                lastCommandTime = hwTimer.micros() / 1000;  // CRITICAL: Compression timing - must use GPTimer
                stepTimer = hwTimer.micros() / 1000;  // CRITICAL: Compression timing - must use GPTimer
                stateEntry = false;
            }
            
            // Check for plastic contact:
            // - Motor stalls (torque exceeds input_torque limit)
            // - Axis error indicates problem
            // Note: Velocity near-zero is NORMAL for torque mode without resistance
            unsigned long travelElapsed = (hwTimer.micros() / 1000) - stepTimer;  // CRITICAL: Compression timing - must use GPTimer
            BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
            bool stallDetected = broadcast.getAxisError() != 0;
            const TimestampedIq* iqData = broadcast.getLatestIq();
            bool torqueExceeded = travelElapsed > INJECT_STABLE_TIME_MS && fabs(broadcast.getVelocity()) < 0.1f && iqData->iqMeasured > COMPRESS_CONTACT_IQ_THRESHOLD;
            
            if (stallDetected || torqueExceeded) {
                // Queue commands - ring buffer handles timing
                MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_COMPRESSION, "Compress Stop");  // Stay in torque mode
                MotorWrapper::adjustMotorLimits(motor, COMPRESS_CONTACT_CURRENT, MODULE_COMPRESSION, "Contact Detected");
                step = TORQUE_RAMP;
                stepTimer = hwTimer.micros() / 1000;  // CRITICAL: Compression timing - must use GPTimer
                stateEntry = true;
                return false;
            }
            
            // Timeout: no plastic in barrel (reached max distance or endstop)
            if (travelElapsed > COMPRESS_TRAVEL_TIMEOUT_MS) {
                isTimeoutFlag = true;
                complete = true;
                return true;
            }
            return false;
        }
        
        // ===== STEP 2: TORQUE RAMP (MODE 1 & MODE 2) =====
        if (step == TORQUE_RAMP) {
            if (stateEntry) {
                // Queue commands for MODE 2 - ring buffer handles timing
                if (currentMode == MODE_2_MICRO) {
                    MotorWrapper::setMotorLimits(motor, COMPRESS_MICRO_VEL_LIMIT, COMPRESS_MICRO_CURRENT, MODULE_COMPRESSION, "Micro Torque");
                    motor.setControllerModes(ODriveCANProtocol::ControlMode::TORQUE_CONTROL, 
                                           ODriveCANProtocol::InputMode::TORQUE_RAMP);
                }
                // For MODE 1, already in torque mode from TRAVEL_DOWN
                stepTimer = hwTimer.micros() / 1000;  // CRITICAL: Compression timing - must use GPTimer
                stateEntry = false;
            }
            
            // Calculate torque ramp
            float rampDuration = commonParams.compressRampDuration;
            unsigned long rampElapsed = (hwTimer.micros() / 1000) - stepTimer;  // CRITICAL: Compression timing - must use GPTimer
            float elapsedSec = rampElapsed / 1000.0f;
            float targetTorque = (commonParams.compressRampTarget / rampDuration) * elapsedSec;
            if (targetTorque > commonParams.compressRampTarget) {
                targetTorque = commonParams.compressRampTarget;
            }
            
            // Send torque setpoint ONCE with retry system (no more spamming)
            static bool torqueCommandSent = false;
            if (!torqueCommandSent) {
                bool commandQueued = MotorWrapper::setModeAndMoveWithRetry(
                    motor, 1, 6, targetTorque, MODULE_COMPRESSION, 
                    "CompressRamp", MotorWrapper::PRIORITY_HIGH
                );
                if (commandQueued) {
                    torqueCommandSent = true;
                    lastCommandTime = now;
                }
            }
            
            // Completion conditions
            bool reachedTorqueTarget = (targetTorque >= commonParams.compressRampTarget);
            bool stallDetected = rampElapsed > INJECT_STABLE_TIME_MS && motor.getAxisError() != 0;
            bool timeoutOnTorque = rampElapsed > COMPRESS_RAMP_TIMEOUT_MS;
            
            if ((reachedTorqueTarget && rampElapsed > 500) || stallDetected || timeoutOnTorque) {
                MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_COMPRESSION, "Compress Release");
                lastCommandTime = now;
                complete = true;
                return true;
            }
            
            return false;
        }
        
        return complete;
    }
    
    // ===== QUERY STATE =====
    bool isComplete() {
        return complete && !isErrorFlag && !isTimeoutFlag;
    }
    
    bool hasError() {
        return isErrorFlag;
    }
    
    bool isTimeout() {
        return isTimeoutFlag;
    }
    
    // ===== RESET =====
    void reset() {
        stateEntry = true;
        complete = false;
        isErrorFlag = false;
        isTimeoutFlag = false;
        pressureSensorChecked = false;
    }
}
