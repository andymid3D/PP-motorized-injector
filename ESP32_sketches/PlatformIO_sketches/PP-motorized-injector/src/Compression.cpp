#include "Compression.h"
#include "config.h"
#include "MotorWrapper.h"
#include "injector_fsm.h"  // For commonInjectParams_t

extern commonInjectParams_t commonParams;  // From main.cpp

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
    static unsigned long stateEnterTime = 0;
    static unsigned long stepTimer = 0;
    static bool complete = false;
    static bool isErrorFlag = false;
    static bool isTimeoutFlag = false;
    static bool pressureSensorChecked = false;
    static unsigned long lastCommandTime = 0;
    
    // ===== TIMEOUT CALCULATION =====
    // Maximum time to reach plastic contact: distance / speed + margin
    // Travel distance is roughly 5-10cm (50-100 turns), max 40+ cm before bottom
    static constexpr float MAX_TRAVEL_TIME_CONST = 10000;  // 10 seconds for full travel if no plastic
    
    // ===== BEGIN: Initialize with mode selection =====
    void begin(CompressionMode mode) {
        currentMode = mode;
        stateEntry = true;
        stateEnterTime = millis();
        stepTimer = millis();
        complete = false;
        isErrorFlag = false;
        isTimeoutFlag = false;
        pressureSensorChecked = false;
        lastCommandTime = millis();
        
        // Start with pressure check for MODE 1, skip to travel/torque for MODE 2
        if (mode == MODE_1_TRAVEL) {
            step = PRESSURE_CHECK;
        } else {
            step = TORQUE_RAMP;  // MODE 2: go directly to torque
        }
    }
    
    // ===== UPDATE: Non-blocking compression logic =====
    bool update(CanBusHandlerV2& motor) {
        unsigned long now = millis();
        unsigned long elapsed = now - stateEnterTime;
        unsigned long stepElapsed = now - stepTimer;
        
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
                // Set motor limits for torque travel (12.5 rps vel_limit, 15A current from config)
                MotorWrapper::setMotorLimits(motor, COMPRESS_TRAVEL_VEL_LIMIT, REFILL_CURRENT_LIMIT, "Compress Travel");
                
                // Wait for command to be sent
                unsigned long waitStart = millis();
                while (millis() - waitStart < (CAN_COMMAND_GAP_MS + 5)) {
                    motor.loop();
                }
                
                // Send torque command to travel down (10A = tested working value)
                MotorWrapper::setModeAndMove(motor, 1, 6, 10.0f, "Compress Travel Down Torque");  // Mode 1=Torque, Input 6=TORQUE_RAMP, 10A (torque_constant=1)
                lastCommandTime = millis();
                stateEntry = false;
            }
            
            // Check for plastic contact:
            // - Motor stalls (torque exceeds input_torque limit)
            // - Axis error indicates problem
            // Note: Velocity near-zero is NORMAL for torque mode without resistance
            bool stallDetected = motor.getAxisError() != 0;
            bool torqueExceeded = stepElapsed > 500 && fabs(motor.getVelocity()) < 0.1f && motor.getIqReadings().Iq_measured > 8.0f;  // High current + stopped = blocked
            
            if (stallDetected || torqueExceeded) {
                MotorWrapper::setModeAndMove(motor, 2, 2, 0, "Compress Stop");  // Stop travel
                lastCommandTime = now;
                
                // Increase current limit for compression after contact
                MotorWrapper::adjustMotorLimits(motor, COMPRESS_CONTACT_CURRENT, "Contact Detected");
                
                // Wait for command to be sent
                unsigned long waitStart = millis();
                while (millis() - waitStart < (CAN_COMMAND_GAP_MS + 5)) {
                    motor.loop();
                }
                
                step = TORQUE_RAMP;
                stepTimer = now;
                stateEntry = true;  // CRITICAL: Reset stateEntry for next step
                return false;
            }
            
            // Timeout: no plastic in barrel (reached max distance or endstop)
            if (stepElapsed > COMPRESS_TRAVEL_TIMEOUT_MS) {
                isTimeoutFlag = true;
                complete = true;
                // No mode change - let state transition handle cleanup
                return true;
            }
            
            // Safety: bottom endstop hit (should not reach this, but emergency stop)
            if (motor.getVelocity() > 0.1f && elapsed > 500) {
                // Motor is moving down but hasn't hit endstop yet
                // Continue as normal
            }
            
            return false;
        }
        
        // ===== STEP 2: TORQUE RAMP (MODE 1 & MODE 2) =====
        if (step == TORQUE_RAMP) {
            if (stateEntry) {
                // Set motor limits for torque control
                // For MODE 2, set initial compression limits
                if (currentMode == MODE_2_MICRO) {
                    MotorWrapper::setMotorLimits(motor, COMPRESS_MICRO_VEL_LIMIT, COMPRESS_MICRO_CURRENT, "Micro Torque");
                    
                    // Wait for command to be sent
                    unsigned long waitStart = millis();
                    while (millis() - waitStart < (CAN_COMMAND_GAP_MS + 5)) {
                        motor.loop();
                    }
                    
                    // Set mode to torque control (MODE 2 only)
                    motor.setControllerModes(ODriveCANProtocol::ControlMode::TORQUE_CONTROL, 
                                           ODriveCANProtocol::InputMode::TORQUE_RAMP);
                    waitStart = millis();
                    while (millis() - waitStart < (CAN_COMMAND_GAP_MS + 5)) {
                        motor.loop();
                    }
                }
                // For MODE 1, already in torque mode from TRAVEL_DOWN
                stateEntry = false;
            }
            
            // Calculate torque ramp
            float rampDuration = commonParams.compressRampDuration;  // From commonParams (default: 2.0 sec)
            float elapsedSec = stepElapsed / 1000.0f;
            float targetTorque = (commonParams.compressRampTarget / rampDuration) * elapsedSec;
            if (targetTorque > commonParams.compressRampTarget) {
                targetTorque = commonParams.compressRampTarget;
            }
            
            // Send torque setpoint updates (mode already set in stateEntry or TRAVEL_DOWN)
            if (now - lastCommandTime >= CAN_COMMAND_GAP_MS) {
                motor.setInputTorque(targetTorque);  // Only update setpoint, not mode
                lastCommandTime = now;
            }
            
            // Completion conditions
            bool reachedTorqueTarget = (targetTorque >= commonParams.compressRampTarget);
            bool stallDetected = stepElapsed > 500 && motor.getAxisError() != 0;
            bool timeoutOnTorque = stepElapsed > 15000;  // 15 seconds max for torque ramp
            
            if ((reachedTorqueTarget && stepElapsed > 500) || stallDetected || timeoutOnTorque) {
                MotorWrapper::setModeAndMove(motor, 1, 6, 0, "Compress Release");
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
