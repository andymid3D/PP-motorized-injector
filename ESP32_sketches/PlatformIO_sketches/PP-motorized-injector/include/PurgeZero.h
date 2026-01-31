#ifndef PURGEZERO_H
#define PURGEZERO_H

#include <Arduino.h>
#include "CanBusHandlerV2.h"

/*
 * PURGE ZERO STATE MACHINE
 * 
 * Purpose: Manual plunger movement to purge nozzle and set injection zero point
 * 
 * Flow:
 *   1. User presses Upper button → retract (up) at SPEED_PURGE
 *   2. User presses Lower button → push plastic out (down) at SPEED_PURGE
 *   3. Center button press → confirm zero point, proceed to ANTIDRIP
 * 
 * Context: CTX_PURGE (same as original manual purge mode)
 * Motor Control: Velocity Control (Mode 2)
 * 
 * Note: This state allows user to manually remove cold plastic from nozzle
 * and establish a consistent injection starting point.
 */

namespace PurgeZero {
    // Initialize on state entry
    void begin();
    
    // Non-blocking update - handle continuous movement based on button state
    // buttonUp, buttonDown, buttonCenter are raw button states (HIGH=released, LOW=pressed)
    bool update(CanBusHandlerV2& motor, bool buttonUp, bool buttonDown, bool buttonCenter);
    
    // Query state
    bool isComplete();
    bool hasError();
    
    // Getters
    float getPurgeZeroPosition();
    
    // Reset for next cycle
    void reset();
};

#endif
