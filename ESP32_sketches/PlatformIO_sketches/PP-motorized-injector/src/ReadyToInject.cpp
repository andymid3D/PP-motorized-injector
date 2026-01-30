#include "ReadyToInject.h"
#include "config.h"
#include "MotorWrapper.h"
#include "injector_fsm.h"  // For commonInjectParams_t

extern commonInjectParams_t commonParams;  // From main.cpp

namespace ReadyToInject {
    // ===== STATIC STATE VARIABLES =====
    static enum {
        IDLE_WAITING,           // Normal idle state
        MICRO_COMPRESSING       // Active micro-compression
    } state = IDLE_WAITING;
    static bool stateEntry = false;
    static unsigned long stateEnterTime = 0;
    static unsigned long lastAutoCompressionTime = 0;
    static unsigned long compressionStartTime = 0;
    static bool error = false;
    static unsigned long lastCommandTime = 0;
    
    // ===== BEGIN: Initialize on state entry =====
    void begin() {
        state = IDLE_WAITING;
        stateEntry = true;
        stateEnterTime = millis();
        lastAutoCompressionTime = millis();
        error = false;
        lastCommandTime = millis();
        
        // Queue idle commands immediately - ring buffer handles timing
        // Note: Commands will be sent when update() is called
    }
    
    // ===== UPDATE: Handle idle waiting + micro-compression timer =====
    bool update(CanBusHandlerV2& motor) {
        unsigned long now = millis();
        unsigned long elapsed = now - stateEnterTime;
        
        // ===== ENTRY: Set idle mode once =====
        if (stateEntry) {
            MotorWrapper::setMotorLimits(motor, REFILL_CONTROLLER_VEL_LIMIT, REFILL_CURRENT_LIMIT, MODULE_READY_TO_INJECT, "ReadyIdle");
            MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_READY_TO_INJECT, "Idle Stop");
            stateEntry = false;
        }
        
        // ===== STATE: Idle waiting + micro-compression timer =====
        if (state == IDLE_WAITING) {
            unsigned long timeSinceLastCompress = now - lastAutoCompressionTime;
            
            if (timeSinceLastCompress >= READY_MICRO_INTERVAL_MS) {
                // Time to start micro-compression - queue commands
                MotorWrapper::setMotorLimits(motor, COMPRESS_MICRO_VEL_LIMIT, COMPRESS_MICRO_CURRENT, MODULE_READY_TO_INJECT, "MicroCompress");
                motor.setControllerModes(ODriveCANProtocol::ControlMode::TORQUE_CONTROL, ODriveCANProtocol::InputMode::TORQUE_RAMP);
                state = MICRO_COMPRESSING;
                compressionStartTime = millis();
                lastCommandTime = millis();
            }
            return false;
        }
        
        // ===== STATE: Execute micro-compression =====
        if (state == MICRO_COMPRESSING) {
            unsigned long compressionElapsed = now - compressionStartTime;
            
            // Apply torque ramp over configured duration
            float rampDuration = READY_MICRO_DURATION_MS / 1000.0f;  // Convert to seconds
            float elapsedSec = compressionElapsed / 1000.0f;
            
            // Linear torque ramp: 0 → commonParams.compressMicroCurrent
            float targetTorque = (commonParams.compressMicroCurrent / rampDuration) * elapsedSec;
            if (targetTorque > commonParams.compressMicroCurrent) {
                targetTorque = commonParams.compressMicroCurrent;
            }
            
            // Send torque setpoint ONCE with retry system (no more spamming)
            static bool torqueCommandSent = false;
            if (!torqueCommandSent) {
                bool commandQueued = MotorWrapper::setModeAndMoveWithRetry(
                    motor, 1, 6, targetTorque, MODULE_READY_TO_INJECT, 
                    "MicroCompress", MotorWrapper::PRIORITY_NORMAL
                );
                if (commandQueued) {
                    torqueCommandSent = true;
                    lastCommandTime = now;
                }
            }
            
            // Check completion: time elapsed or stall detected
            bool completedByTime = compressionElapsed >= READY_MICRO_DURATION_MS;
            bool completedByStall = (compressionElapsed > 500) && (fabs(motor.getVelocity()) < 0.5f);
            
            if (completedByTime || completedByStall) {
                // Compression complete, return to idle
                MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_READY_TO_INJECT, "MicroCompress Release");
                lastAutoCompressionTime = now;
                state = IDLE_WAITING;
                lastCommandTime = now;
            }
            return false;
        }
        
        return false;  // This state doesn't auto-complete
    }
    
    // ===== BUTTON HANDLERS =====
    bool handlePurgeButton() {
        // Upper+Lower pressed: user wants to start injection (go to PURGE_ZERO)
        // If micro-compression is running, stop it and proceed
        if (state == MICRO_COMPRESSING) {
            state = IDLE_WAITING;
            lastAutoCompressionTime = millis();  // Reset compression timer
        }
        return true;  // Proceed to PURGE_ZERO
    }
    
    bool handleRefillButton() {
        // Center pressed: return to REFILL
        // If micro-compression is running, stop it
        if (state == MICRO_COMPRESSING) {
            state = IDLE_WAITING;
            lastAutoCompressionTime = millis();  // Reset compression timer
        }
        return true;  // Proceed to REFILL
    }
    
    // ===== QUERY STATE =====
    bool isComplete() {
        return false;  // This is an idle waiting state
    }
    
    bool isMicroCompressing() {
        return (state == MICRO_COMPRESSING);
    }
    
    bool hasError() {
        return error;
    }
    
    // ===== RESET =====
    void reset() {
        state = IDLE_WAITING;
        stateEntry = true;
        error = false;
    }
}
