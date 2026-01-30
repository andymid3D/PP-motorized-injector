#include "Refill.h"
#include "config.h"
#include "MotorWrapper.h"
#include "injector_fsm.h"  // For commonInjectParams_t

extern commonInjectParams_t commonParams;  // From main.cpp

namespace Refill {
    // ===== STATIC STATE VARIABLES =====
    static enum { 
        MOVING_TO_HOME,      // Position move active
        WAIT_ARRIVE,         // Arrived, waiting for button
        DONE 
    } step = DONE;
    static unsigned long stepTimer = 0;
    static bool stateEntry = false;
    static bool complete = false;
    static bool error = false;
    
    // ===== BEGIN: Initialize on state entry =====
    void begin() {
        step = MOVING_TO_HOME;
        stepTimer = millis();
        stateEntry = true;
        complete = false;
        error = false;
    }
    
    // ===== UPDATE: Non-blocking state machine =====
    bool update(CanBusHandlerV2& motor) {
        unsigned long now = millis();
        unsigned long elapsed = now - stepTimer;
        
        // ===== STEP 0: Move to OFFSET_REFILL_GAP =====
        if (step == MOVING_TO_HOME) {
            if (stateEntry) {
                // Queue all commands - ring buffer handles timing
                MotorWrapper::setMotorLimits(motor, REFILL_CONTROLLER_VEL_LIMIT, REFILL_CURRENT_LIMIT, MODULE_REFILL, "Refill");
                MotorWrapper::setTrapTrajParams(motor, commonParams.refillTrapVelLimit, 
                                                commonParams.refillAccel, commonParams.refillDecel, MODULE_REFILL, "Refill Traj");
                MotorWrapper::setModeAndMove(motor, 3, 5, OFFSET_REFILL_GAP, MODULE_REFILL, "Pos Refill");
                stepTimer = millis();
                stateEntry = false;
            }
            
            // Check if motor arrived (velocity < threshold for sustained time)
            unsigned long moveElapsed = now - stepTimer;
            if (fabs(motor.getVelocity()) < 0.1f && moveElapsed > INJECT_STABLE_TIME_MS) {
                step = WAIT_ARRIVE;
                stepTimer = now;
            }
            
            // Safety timeout (calculated from full barrel length)
            if (moveElapsed > REFILL_TIMEOUT_MS) {
                error = true;
                complete = true;
                return true;
            }
            return false;
        }
        
        // ===== STEP 1: Wait at home position =====
        if (step == WAIT_ARRIVE) {
            // Just idling at position, waiting for button input from main.cpp
            // Main.cpp will call handleCompressButton() to advance state
            // (Don't auto-advance, wait for user button press)
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
    
    // ===== RESET =====
    void reset() {
        step = DONE;
        complete = false;
        error = false;
        stateEntry = true;
    }
    
    // ===== BUTTON HANDLERS =====
    bool handleToggleEndOfDay() {
        // This is handled in main.cpp with direct flag toggle
        // But module can track that it happened
        return true;  // Placeholder for now
    }
    
    bool handleCompressButton() {
        // User pressed Center button to proceed to COMPRESSION
        // Main.cpp will call this; we just confirm we can proceed
        if (step == WAIT_ARRIVE && !error) {
            complete = true;
            return true;
        }
        return false;
    }
}
