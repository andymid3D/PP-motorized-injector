#include "Refill.h"
#include "config.h"
#include "MotorWrapper.h"

namespace Refill {
    // ===== STATIC STATE VARIABLES =====
    static enum { MOVING_TO_HOME, WAIT_ARRIVE, DONE } step = DONE;
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
                // Set motor limits for refill move (controller limit = machine max for TRAP_TRAJ authority)
                MotorWrapper::setMotorLimits(motor, REFILL_CONTROLLER_VEL_LIMIT, REFILL_CURRENT_LIMIT, "Refill");
                
                // Wait for command to be sent via loop()
                unsigned long waitStart = millis();
                while (millis() - waitStart < (CAN_COMMAND_GAP_MS + 5)) {
                    motor.loop();  // Process CAN queue during wait
                }
                
                // Configure TRAP_TRAJ for smooth move (trajectory limit - actual movement speed)
                MotorWrapper::setTrapTrajParams(motor, REFILL_TRAP_VEL_LIMIT, REFILL_ACCEL, REFILL_DECEL, "Refill Traj");
                
                // Wait for commands to be sent
                waitStart = millis();
                while (millis() - waitStart < (CAN_COMMAND_GAP_MS + 5)) {
                    motor.loop();  // Process CAN queue during wait
                }
                
                // Execute position move with TRAP_TRAJ
                MotorWrapper::setModeAndMove(motor, 3, 5, OFFSET_REFILL_GAP, "Pos Refill");
                stateEntry = false;
            }
            
            // Check if motor arrived (velocity < threshold for sustained time)
            if (fabs(motor.getVelocity()) < 0.1f && elapsed > 500) {
                step = WAIT_ARRIVE;
                stepTimer = now;
            }
            
            // Safety timeout (15 seconds should be plenty for this move)
            if (elapsed > 15000) {
                error = true;
                complete = true;
                return true;
            }
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
