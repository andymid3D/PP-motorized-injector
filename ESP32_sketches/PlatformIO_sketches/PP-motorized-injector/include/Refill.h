#ifndef REFILL_H
#define REFILL_H

#include <Arduino.h>
#include "CanBusHandlerV2.h"

/*
 * REFILL STATE MACHINE
 * 
 * Purpose: Move plunger from COMPRESSION position back to rest position (OFFSET_REFILL_GAP)
 * 
 * Flow:
 *   1. Enter REFILL state
 *   2. Move to OFFSET_REFILL_GAP using position control
 *   3. Wait for arrival
 *   4. Handle user inputs:
 *      - Upper+Lower: Toggle endOfDay flag
 *      - Center: Proceed to COMPRESSION
 *   5. Research: Optional drift check (homing-zero on each refill?)
 * 
 * Context: CTX_IDLE
 * Motor Control: Position Control (Mode 3)
 */

namespace Refill {
    // Initialize on state entry
    void begin();
    
    // Non-blocking update (returns true when ready for next state)
    bool update(CanBusHandlerV2& motor);
    
    // Query state
    bool isComplete();
    bool hasError();
    
    // Reset for next cycle
    void reset();
    
    // Button handlers
    bool handleToggleEndOfDay();
    bool handleCompressButton();
};

#endif
