#include "Refill.h"
#include "config.h"

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
                // Set position control mode (once at entry)
                motor.setControllerModes(ODriveCANProtocol::ControlMode::POSITION_CONTROL,
                                        ODriveCANProtocol::InputMode::PASSTHROUGH);
                delay(CAN_COMMAND_GAP_MS / 2);  // Small delay to avoid command collision
                motor.setInputPos(OFFSET_REFILL_GAP);
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
