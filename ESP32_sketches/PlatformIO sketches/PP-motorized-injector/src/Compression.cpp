#include "Compression.h"
#include "config.h"
#include "MotorWrapper.h"

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
    static constexpr float MAX_TRAVEL_TIME_MS = 10000;  // 10 seconds for full travel if no plastic
    
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
            }
            return false;
        }
        
        // ===== STEP 1: TRAVEL DOWN until plastic contact (MODE 1 only) =====
        if (step == TRAVEL_DOWN) {
            if (stateEntry) {
                // Set velocity control for travel
                motor.setControllerModes(ODriveCANProtocol::ControlMode::VELOCITY_CONTROL,
                                        ODriveCANProtocol::InputMode::PASSTHROUGH);
                stateEntry = false;
            }
            
            // Send velocity command to travel down
            if (now - lastCommandTime >= CAN_COMMAND_GAP_MS) {
                motor.setInputVel(SPEED_COMPRESS_INIT);  // Positive = down (inject direction)
                lastCommandTime = now;
            }
            
            // Check for plastic contact:
            // - Velocity drops to near zero (plunger hits resistance)
            // - Or motor stalls (pressure/force too high)
            bool contactDetected = stepElapsed > 500 && fabs(motor.getVelocity()) < 0.5f;
            bool stallDetected = stepElapsed > 500 && motor.getAxisError() != 0;
            
            if (contactDetected || stallDetected) {
                motor.setInputVel(0);  // Stop travel
                lastCommandTime = now;
                step = TORQUE_RAMP;
                stepTimer = now;
                return false;
            }
            
            // Timeout: no plastic in barrel (reached max distance or endstop)
            if (stepElapsed > MAX_TRAVEL_TIME_MS) {
                isTimeoutFlag = true;
                complete = true;
                motor.setInputVel(0);
                lastCommandTime = now;
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
                // Set torque control for compression
                motor.setControllerModes(ODriveCANProtocol::ControlMode::TORQUE_CONTROL,
                                        ODriveCANProtocol::InputMode::PASSTHROUGH);
                stateEntry = false;
            }
            
            // Calculate torque ramp
            float rampDuration = 2.0f;  // Seconds to reach target torque
            float elapsedSec = stepElapsed / 1000.0f;
            float targetTorque = (TORQUE_COMPRESSION_HOLD / rampDuration) * elapsedSec;
            if (targetTorque > TORQUE_COMPRESSION_HOLD) {
                targetTorque = TORQUE_COMPRESSION_HOLD;
            }
            
            // Send command if enough time has elapsed
            if (now - lastCommandTime >= CAN_COMMAND_GAP_MS) {
                motor.setInputTorque(targetTorque);
                lastCommandTime = now;
            }
            
            // Completion conditions
            bool reachedTorqueTarget = (targetTorque >= TORQUE_COMPRESSION_HOLD);
            bool stallDetected = stepElapsed > 500 && motor.getAxisError() != 0;
            bool timeoutOnTorque = stepElapsed > 15000;  // 15 seconds max for torque ramp
            
            if ((reachedTorqueTarget && stepElapsed > 500) || stallDetected || timeoutOnTorque) {
                motor.setInputTorque(0);
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
