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
    static uint64_t stepTimer = 0;  // Use uint64_t for GPTimer compatibility
    static bool stateEntry = false;
    static bool complete = false;
    static bool error = false;
    
    // ===== HEARTBEAT STATE TRACKING (for stale trajectory detection) =====
    static uint8_t lastHeartbeatState = 255;  // Track heartbeat state changes
    static uint64_t lastHeartbeatTimestamp = 0;  // Track when heartbeat was received
    static bool newTrajectoryStarted = false;  // Track if new trajectory has begun
    // ===== END HEARTBEAT STATE TRACKING =====
    
    // ===== BEGIN: Initialize on state entry =====
    void begin() {
        step = MOVING_TO_HOME;
        // Use GPTimer for consistent timing with other system components
        stepTimer = hwTimer.micros();  // Store microseconds directly (uint64_t)
        stateEntry = true;
        complete = false;
        error = false;
        
        // ===== RESET HEARTBEAT TRACKING =====
        lastHeartbeatState = 255;  // Force state change detection
        lastHeartbeatTimestamp = 0;
        newTrajectoryStarted = false;  // Will be set when new trajectory detected
        // ===== END HEARTBEAT TRACKING RESET =====
    }
    
    // ===== UPDATE: Non-blocking state machine =====
    bool update(CanBusHandlerV2& motor) {
        BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
        // Use GPTimer for consistent timing with other system components
        uint64_t now = hwTimer.micros();  // Use microseconds directly (uint64_t)
        uint64_t elapsed = now - stepTimer;
        
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
                stepTimer = hwTimer.micros();  // Use GPTimer for consistency (uint64_t)
                stateEntry = false;
                
                // DEBUG: Log after commands sent (commented out to reduce debug overlap)
                // char dbgBuf2[60];
                // snprintf(dbgBuf2, sizeof(dbgBuf2), "[REFILL_DEBUG] Commands sent - %lu", stepTimer);
                // MessageBuffer::getInstance().sendMessage(dbgBuf2);
            }
            
            // ===== HEARTBEAT STATE CHANGE DETECTION =====
            // Detect when ODrive starts new trajectory by monitoring heartbeat state changes
            const TimestampedHeartbeat* latestHb = broadcast.getLatestHeartbeat();  // Now with automatic fresh data guarantee!
            if (latestHb) {
                uint8_t currentHeartbeatState = latestHb->axisState;
                uint64_t currentHeartbeatTimestamp = latestHb->timestamp;
                
                // Detect heartbeat state change (indicates new trajectory activity)
                if (currentHeartbeatState != lastHeartbeatState) {
                    lastHeartbeatState = currentHeartbeatState;
                    lastHeartbeatTimestamp = currentHeartbeatTimestamp;
                    
                    // If we see state change after sending command, new trajectory has started
                    // Use state change detection instead of timestamp comparison for reliability
                    if (!newTrajectoryStarted) {
                        newTrajectoryStarted = true;
                        char dbgHb[60];
                        snprintf(dbgHb, sizeof(dbgHb), "[REFILL] New trajectory detected state=%d", currentHeartbeatState);
                        MessageBuffer::getInstance().sendMessage(dbgHb);
                    }
                }
            }
            // ===== END HEARTBEAT STATE CHANGE DETECTION =====
            
            // Check if motor arrived using ODrive's trajectory completion flag (clean, no timing math)
            bool trajectoryComplete = broadcast.isTrajectoryComplete();
            bool motorStopped = fabs(broadcast.getVelocity()) < 0.1f;
            bool motorActuallyMoving = fabs(broadcast.getVelocity()) > 1.0f;  // Motor has started moving
            
            // Only consider trajectory complete if we've detected a NEW trajectory (not stale data)
            bool freshTrajectoryComplete = newTrajectoryStarted && trajectoryComplete;
            
            // DEBUG: Log moveElapsed calculation (helpful for underflow debugging)
            static uint64_t lastElapsedDebug = 0;
            if (now - lastElapsedDebug > 1000000) {  // Every 1 second (1,000,000 microseconds)
                char dbgElapsed[80];
                snprintf(dbgElapsed, sizeof(dbgElapsed), "[REFILL_DEBUG] trajComplete=%d fresh=%d stopped=%d moving=%d vel=%.1f", 
                         trajectoryComplete, freshTrajectoryComplete, motorStopped, motorActuallyMoving, broadcast.getVelocity());
                MessageBuffer::getInstance().sendMessage(dbgElapsed);
                lastElapsedDebug = now;
            }
            
            // CRITICAL: Only check arrival AFTER new trajectory started AND motor moved AND stopped
            // This prevents false positives from stale trajectory flags
            static bool movementStarted = false;
            if (motorActuallyMoving) movementStarted = true;
            
            if (movementStarted && freshTrajectoryComplete && motorStopped) {
                step = WAIT_ARRIVE;
                stepTimer = hwTimer.micros();  // Use GPTimer for consistency (uint64_t)
                
                // DEBUG: Log arrival
                char dbgBuf3[60];
                snprintf(dbgBuf3, sizeof(dbgBuf3), "[REFILL_DEBUG] Motor arrived via fresh trajectory");
                MessageBuffer::getInstance().sendMessage(dbgBuf3);
                
                // Reset movementStarted for next cycle
                movementStarted = false;
            }
            
            // Safety timeout for worst-case: Refill from barrel end (355 turns) back to Refill position (47.75 turns)
            // This handles the scenario where barrel is almost empty and motor must travel full distance
            // Only check timeout after commands have been sent (stateEntry = false)
            uint64_t moveElapsed = now - stepTimer;  // Calculate for timeout check only (now in microseconds)
            if (!stateEntry && moveElapsed <= now && moveElapsed > (REFILL_FROM_BARREL_END_TIMEOUT_MS * 1000)) {
                // DEBUG: Log timeout trigger
                char dbgBuf4[60];
                snprintf(dbgBuf4, sizeof(dbgBuf4), "[REFILL_DEBUG] TIMEOUT - %lluus > %lums", 
                         moveElapsed, REFILL_FROM_BARREL_END_TIMEOUT_MS);
                MessageBuffer::getInstance().sendMessage(dbgBuf4);
                
                error = true;
                complete = true;
                return true;
            }
            
            // DEBUG: Log periodic status (every 1 second to avoid overlap)
            static uint64_t lastDebugTime = 0;
            if (now - lastDebugTime > 1000000) {  // 1 second in microseconds
                char dbgBuf5[60];
                snprintf(dbgBuf5, sizeof(dbgBuf5), "[REFILL_DEBUG] Status - %d %d %d %d %.1f", 
                         movementStarted, trajectoryComplete, stateEntry, error, broadcast.getVelocity());
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
            return true;  // Motor has arrived, unlock buttons
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
