#include "ReadyToInject.h"
#include "config.h"
#include "MotorWrapper.h"

namespace ReadyToInject {
    // ===== STATIC STATE VARIABLES =====
    static bool stateEntry = false;
    static unsigned long stateEnterTime = 0;
    static unsigned long lastAutoCompressionTime = 0;
    static bool microCompressing = false;
    static unsigned long compressionStartTime = 0;
    static bool error = false;
    static unsigned long lastCommandTime = 0;
    static const unsigned long MICRO_COMPRESSION_DURATION = 2000;  // ~2 seconds
    
    // ===== BEGIN: Initialize on state entry =====
    void begin() {
        stateEntry = true;
        stateEnterTime = millis();
        lastAutoCompressionTime = millis();  // Reset compression timer on entry
        microCompressing = false;
        error = false;
        lastCommandTime = millis();
    }
    
    // ===== UPDATE: Handle idle waiting + micro-compression timer =====
    bool update(CanBusHandlerV2& motor) {
        unsigned long now = millis();
        unsigned long elapsed = now - stateEnterTime;
        
        // ===== ENTRY: Set velocity control (idle) =====
        if (stateEntry) {
            // Set motor limits for idle state
            MotorWrapper::setMotorLimits(motor, REFILL_CONTROLLER_VEL_LIMIT, REFILL_CURRENT_LIMIT, "ReadyIdle");
            delay(CAN_COMMAND_GAP_MS + 5);
            MotorWrapper::setModeAndMove(motor, 1, 6, 0, "Idle Stop");  // Stop motor, idle
            stateEntry = false;
        }
        
        // ===== MICRO-COMPRESSION TIMER =====
        unsigned long timeSinceLastCompress = now - lastAutoCompressionTime;
        
        if (timeSinceLastCompress >= READY_MICRO_INTERVAL_MS && !microCompressing) {
            // Time to start micro-compression
            microCompressing = true;
            compressionStartTime = now;
            
            // Set motor limits for torque control (micro-compression)
            MotorWrapper::setMotorLimits(motor, COMPRESS_MICRO_VEL_LIMIT, COMPRESS_MICRO_CURRENT, "MicroCompress");
            delay(CAN_COMMAND_GAP_MS + 5);
            lastCommandTime = now;
        }
        
        // ===== MICRO-COMPRESSION EXECUTION =====
        if (microCompressing) {
            unsigned long compressionElapsed = now - compressionStartTime;
            
            // Apply torque ramp over ~2 seconds
            float rampDuration = MICRO_COMPRESSION_DURATION / 1000.0f;  // Convert to seconds
            float elapsedSec = compressionElapsed / 1000.0f;
            
            // Linear torque ramp: 0 → COMPRESS_RAMP_TARGET
            float targetTorque = (COMPRESS_RAMP_TARGET / rampDuration) * elapsedSec;
            if (targetTorque > COMPRESS_RAMP_TARGET) {
                targetTorque = COMPRESS_RAMP_TARGET;
            }
            
            // Send torque setpoint updates (mode already set in entry)
            if (now - lastCommandTime >= CAN_COMMAND_GAP_MS) {
                motor.setInputTorque(targetTorque);  // Only update setpoint, not mode
                lastCommandTime = now;
            }
            
            // Check completion: time elapsed or stall detected
            bool completedByTime = compressionElapsed >= MICRO_COMPRESSION_DURATION;
            bool completedByStall = (compressionElapsed > 500) && (fabs(motor.getVelocity()) < 0.5f);
            
            if (completedByTime || completedByStall) {
                // Compression complete, return to idle
                microCompressing = false;
                lastAutoCompressionTime = now;  // Reset timer for next compression
                
                // Ramp torque back to zero smoothly
                MotorWrapper::setModeAndMove(motor, 1, 6, 0, "MicroCompress Release");
                lastCommandTime = now;
            }
        }
        
        return false;  // This state doesn't auto-complete
    }
    
    // ===== BUTTON HANDLERS =====
    bool handlePurgeButton() {
        // Upper+Lower pressed: user wants to start injection (go to PURGE_ZERO)
        // If micro-compression is running, stop it and proceed
        if (microCompressing) {
            microCompressing = false;
            lastAutoCompressionTime = millis();  // Reset compression timer
        }
        return true;  // Proceed to PURGE_ZERO
    }
    
    bool handleRefillButton() {
        // Center pressed: user wants to abort this cycle and return to REFILL
        // If micro-compression is running, stop it and proceed
        if (microCompressing) {
            microCompressing = false;
            lastAutoCompressionTime = millis();  // Reset compression timer
        }
        return true;  // Proceed to REFILL
    }
    
    // ===== QUERY STATE =====
    bool isComplete() {
        return false;  // This is an idle waiting state
    }
    
    bool isMicroCompressing() {
        return microCompressing;
    }
    
    bool hasError() {
        return error;
    }
    
    // ===== RESET =====
    void reset() {
        stateEntry = true;
        microCompressing = false;
        error = false;
    }
}
