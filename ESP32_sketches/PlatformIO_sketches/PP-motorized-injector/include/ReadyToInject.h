#ifndef READYTOINJECT_H
#define READYTOINJECT_H

#include <Arduino.h>
#include "CanBusHandlerV2.h"

/*
 * READY_TO_INJECT STATE MACHINE
 * 
 * Purpose: Idle waiting state with automatic micro-compression every 30s
 * 
 * Flow:
 *   1. Enter READY_TO_INJECT after COMPRESSION or after canceling PURGE
 *   2. Idle waiting for user input
 *   3. Every TIME_AUTO_COMPRESS (30s), trigger micro-compression autonomously
 *   4. Micro-compression: Apply torque ramp, ~2 seconds, compress plastic under plunger
 *   5. User buttons:
 *      - Upper+Lower: Go to PURGE_ZERO (start injection prep)
 *      - Center: Return to REFILL (abort this cycle)
 * 
 * Context: CTX_IDLE
 * Motor Control: Velocity (idle) or Torque (during micro-compression)
 * 
 * Note: Micro-compression runs silently in background without changing LED states.
 * Optional: Flash center LED during compression (0.5s duration).
 */

namespace ReadyToInject {
    // Initialize on state entry
    void begin();
    
    // Non-blocking update - handles micro-compression timer and logic
    bool update(CanBusHandlerV2& motor);
    
    // Query state
    bool isComplete();              // Not used for this state (idle waiting)
    bool isMicroCompressing();      // True if currently compressing
    bool hasError();
    
    // Button handlers (called from main.cpp)
    bool handlePurgeButton();       // Upper+Lower → proceed to PURGE_ZERO
    bool handleRefillButton();      // Center → return to REFILL
    
    // Reset for next cycle
    void reset();
};

#endif
