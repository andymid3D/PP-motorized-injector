#include "Refill.h"
#include "config.h"
#include "MotorWrapper.h"
#include "GPTimer.h"  // Add GPTimer include
#include "BroadcastDataStore.h"  // Add include for broadcast data
#include "injector_fsm.h"  // For commonInjectParams_t

extern commonInjectParams_t commonParams;  // From main.cpp
extern GPTimer hwTimer;  // Global timer instance from main.cpp

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
        // Use GPTimer for consistent timing with other system components
        stepTimer = hwTimer.micros() / 1000;  // Convert microseconds to milliseconds
        stateEntry = true;
        complete = false;
        error = false;
    }
    
    // ===== UPDATE: Non-blocking state machine =====
    bool update(CanBusHandlerV2& motor) {
        BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
        // Use GPTimer for consistent timing with other system components
        unsigned long now = hwTimer.micros() / 1000;  // Convert microseconds to milliseconds
        unsigned long elapsed = now - stepTimer;
        
        // ===== STEP 0: Move to OFFSET_REFILL_GAP =====
        if (step == MOVING_TO_HOME) {
            if (stateEntry) {
                // DEBUG: Log Refill state entry (commented out to reduce debug overlap)
                // char dbgBuf[80];
                // snprintf(dbgBuf, sizeof(dbgBuf), "[REFILL_DEBUG] State entry - step=%d stateEntry=%d", step, stateEntry);
                // MessageBuffer::getInstance().sendMessage(dbgBuf);
                
                // Queue all commands - ring buffer handles timing (16 slots now!)
                MotorWrapper::setMotorLimits(motor, REFILL_CONTROLLER_VEL_LIMIT, REFILL_CURRENT_LIMIT, MODULE_REFILL, "Refill");
                MotorWrapper::setTrapTrajParams(motor, commonParams.refillTrapVelLimit, 
                                                commonParams.refillAccel, commonParams.refillDecel, MODULE_REFILL, "Refill Traj");
                MotorWrapper::setModeAndMove(motor, 3, 5, OFFSET_REFILL_GAP, MODULE_REFILL, "Pos Refill");
                stepTimer = hwTimer.micros() / 1000;  // Use GPTimer for consistency
                stateEntry = false;
                
                // DEBUG: Log after commands sent (commented out to reduce debug overlap)
                // char dbgBuf2[60];
                // snprintf(dbgBuf2, sizeof(dbgBuf2), "[REFILL_DEBUG] Commands sent - %lu", stepTimer);
                // MessageBuffer::getInstance().sendMessage(dbgBuf2);
            }
            
            // Check if motor arrived (velocity < threshold for sustained time)
            unsigned long moveElapsed = now - stepTimer;
            
            // DEBUG: Log moveElapsed calculation (helpful for underflow debugging)
            static unsigned long lastElapsedDebug = 0;
            if (now - lastElapsedDebug > 1000) {  // Every 1 second
                char dbgElapsed[60];
                snprintf(dbgElapsed, sizeof(dbgElapsed), "[REFILL_DEBUG] moveElapsed=%lu now=%lu timer=%lu", 
                         moveElapsed, now, stepTimer);
                MessageBuffer::getInstance().sendMessage(dbgElapsed);
                lastElapsedDebug = now;
            }
            
            if (fabs(broadcast.getVelocity()) < 0.1f && moveElapsed > INJECT_STABLE_TIME_MS) {
                step = WAIT_ARRIVE;
                stepTimer = hwTimer.micros() / 1000;  // Use GPTimer for consistency
                
                // DEBUG: Log arrival
                char dbgBuf3[60];
                snprintf(dbgBuf3, sizeof(dbgBuf3), "[REFILL_DEBUG] Motor arrived - %lu %.1f", 
                         moveElapsed, broadcast.getVelocity());
                MessageBuffer::getInstance().sendMessage(dbgBuf3);
            }
            
            // Safety timeout for worst-case: Refill from barrel end (355 turns) back to Refill position (47.75 turns)
            // This handles the scenario where barrel is almost empty and motor must travel full distance
            // Only check timeout after commands have been sent (stateEntry = false)
            // Fix: Check for unsigned underflow (moveElapsed > now means stepTimer > now)
            if (!stateEntry && moveElapsed <= now && moveElapsed > REFILL_FROM_BARREL_END_TIMEOUT_MS) {
                // DEBUG: Log timeout trigger
                char dbgBuf4[60];
                snprintf(dbgBuf4, sizeof(dbgBuf4), "[REFILL_DEBUG] TIMEOUT - %lu > %lu", 
                         moveElapsed, REFILL_FROM_BARREL_END_TIMEOUT_MS);
                MessageBuffer::getInstance().sendMessage(dbgBuf4);
                
                error = true;
                complete = true;
                return true;
            }
            
            // DEBUG: Log periodic status (every 1 second to avoid overlap)
            static unsigned long lastDebugTime = 0;
            if (now - lastDebugTime > 1000) {
                char dbgBuf5[60];
                snprintf(dbgBuf5, sizeof(dbgBuf5), "[REFILL_DEBUG] Status - %lu %d %d %.1f", 
                         moveElapsed, stateEntry, error, broadcast.getVelocity());
                MessageBuffer::getInstance().sendMessage(dbgBuf5);
                lastDebugTime = now;
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
