#include "Injection.h"
#include "config.h"
#include "MotorWrapper.h"

extern commonInjectParams_t commonParams;  // From main.cpp

namespace Injection {
    // ===== STATIC STATE VARIABLES =====
    static InjectionPhase phase = DONE;
    static bool stateEntry = false;
    static unsigned long stateEnterTime = 0;
    static unsigned long phaseStartTime = 0;
    static bool complete = false;
    static bool error = false;
    static unsigned long lastCommandTime = 0;
    static bool pressureSensorChecked = false;
    
    // Store mould parameters
    static actualMouldParams_t currentMould;
    
    // Position tracking
    static float injectStartPos = 0.0f;
    static float packStartPos = 0.0f;
    static float targetInjectPos = 0.0f;
    static float targetPackPos = 0.0f;
    
    // Helper: Volume to Turns conversion
    static float volToTurns(float cm3) {
        return cm3 * TURNS_PER_CM3_VOL;
    }
    
    // ===== BEGIN: Initialize with mould parameters =====
    void begin(const actualMouldParams_t& mouldParams) {
        currentMould = mouldParams;
        phase = FILLING;
        stateEntry = true;
        stateEnterTime = millis();
        phaseStartTime = millis();
        complete = false;
        error = false;
        lastCommandTime = millis();
        pressureSensorChecked = false;
        
        injectStartPos = 0.0f;
        packStartPos = 0.0f;
    }
    
    // ===== UPDATE: Non-blocking injection logic =====
    bool update(CanBusHandlerV2& motor) {
        unsigned long now = millis();
        unsigned long stateElapsed = now - stateEnterTime;
        unsigned long phaseElapsed = now - phaseStartTime;
        
        // ===== ENTRY: Initialize FILLING phase =====
        if (stateEntry) {
            // Pressure sensor check (first ms)
            pressureSensorChecked = true;
            
            // Capture start position and calculate targets
            injectStartPos = motor.getPosition();
            targetInjectPos = injectStartPos + volToTurns(currentMould.fillVolume);
            
            // Safety: don't exceed mechanical limit
            if (targetInjectPos > POS_BOTTOM_MAX) {
                targetInjectPos = POS_BOTTOM_MAX;
            }
            
            // Log injection details
            char logBuf[128];
            snprintf(logBuf, sizeof(logBuf), "Inject: Start=%.1f Target=%.1f MoveAmt=%.1f turns (%.1fcm3)",
                     injectStartPos, targetInjectPos, 
                     volToTurns(currentMould.fillVolume), currentMould.fillVolume);
            MessageBuffer::getInstance().sendMessage(logBuf);
            
            // Queue all commands - ring buffer handles timing
            MotorWrapper::setMotorLimits(motor, INJECT_FILL_CONTROLLER_VEL_LIMIT, INJECT_FILL_CURRENT, "Inject Fill");
            MotorWrapper::setTrapTrajParams(motor, commonParams.injectFillTrapVelLimit, 
                                           commonParams.injectFillAccel, 
                                           commonParams.injectFillDecel, 
                                           "Fill Traj");
            MotorWrapper::setModeAndMove(motor, 3, 5, targetInjectPos, "Pos Inject");
            lastCommandTime = now;
            
            stateEntry = false;
        }
        
        // ===== FILLING PHASE =====
        if (phase == FILLING) {
            // TRAP_TRAJ mode maintains position target automatically
            // No need to resend commands unless motor appears stuck
            
            // Check for completion: velocity drops to near-zero and stays stable
            bool velocityLow = fabs(motor.getVelocity()) < 0.1f;
            bool timingStable = phaseElapsed > 500;
            bool positionClose = fabs(motor.getPosition() - targetInjectPos) < 1.0f;
            
            if (velocityLow && timingStable && positionClose) {
                // Transition to PACKING phase
                phase = PACKING;
                phaseStartTime = now;
                packStartPos = motor.getPosition();
                targetPackPos = packStartPos + volToTurns(currentMould.packVolume);
                
                // Safety: don't exceed mechanical limit
                if (targetPackPos > POS_BOTTOM_MAX) {
                    targetPackPos = POS_BOTTOM_MAX;
                }
                
                // Set motor limits for packing (higher pressure, controller limit = machine max for TRAP_TRAJ authority)
                MotorWrapper::setMotorLimits(motor, INJECT_PACK_CONTROLLER_VEL_LIMIT, INJECT_PACK_CURRENT, "Inject Pack");
                
                // Configure TRAP_TRAJ with mould-specific accel/decel for pack (trajectory limit - slower, controlled)
                MotorWrapper::setTrapTrajParams(motor, commonParams.injectPackTrapVelLimit,
                                               commonParams.injectPackAccel,
                                               commonParams.injectPackDecel,
                                               "Pack Traj");
                
                // Set lower pressure for packing - send ONCE
                MotorWrapper::setModeAndMove(motor, 3, 5, targetPackPos, "Pos Pack");
                lastCommandTime = now;
                
                return false;  // Still running
            }
            
            // Safety timeout (injection should complete in <10 seconds typically)
            if (phaseElapsed > 30000) {
                error = true;
                complete = true;
                MotorWrapper::setModeAndMove(motor, 2, 1, 0, "Inject Timeout Stop");
                return true;
            }
            
            return false;
        }
        
        // ===== PACKING PHASE =====
        if (phase == PACKING) {
            // TRAP_TRAJ mode maintains position target automatically
            // No need to resend commands
            
            // Packing time in milliseconds
            unsigned long packTimeMs = (unsigned long)(currentMould.packTime * 1000);
            
            // Check for completion: pack time elapsed
            if (phaseElapsed >= packTimeMs) {
                phase = DONE;
                complete = true;
                // No mode change - let state transition handle cleanup
                return true;
            }
            
            return false;
        }
        
        return complete;
    }
    
    // ===== QUERY STATE =====
    bool isComplete() {
        return complete && !error;
    }
    
    bool hasError() {
        return error;
    }
    
    InjectionPhase getPhase() {
        return phase;
    }
    
    // ===== RESET =====
    void reset() {
        stateEntry = true;
        phase = DONE;
        complete = false;
        error = false;
        pressureSensorChecked = false;
    }
}
