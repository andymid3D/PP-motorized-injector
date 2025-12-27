#ifndef ANTIDRIP_H
#define ANTIDRIP_H

#include <Arduino.h>
#include "CanBusHandlerV2.h"

/*
 * ANTIDRIP STATE MACHINE
 * 
 * Purpose: Slow upward plunger movement to prevent plastic drip while user places mould
 * 
 * Flow:
 *   1. Enter ANTIDRIP after purge/zero
 *   2. Apply slow upward velocity (-SPEED_ANTIDRIP) to counteract drip
 *   3. User has TIME_ANTIDRIP_TIMEOUT seconds to place mould and confirm
 *   4. Center+Lower button press → proceed to INJECT (normal good flow)
 *   5. Upper button release OR timeout → return to READY_TO_INJECT (failed cycle)
 * 
 * Context: CTX_MOVING_FREE
 * Motor Control: Velocity Control (Mode 2)
 * 
 * Pressure Sensor: Check during first ms to confirm mould is blocked
 * (HX711 reads pressure from underneath rotating platform)
 */

namespace AntiDrip {
    // Initialize on state entry
    void begin();
    
    // Non-blocking update
    // Returns: true if state complete (either proceed to INJECT or return to READY)
    bool update(CanBusHandlerV2& motor);
    
    // Query outcomes
    bool isComplete();      // Completed (ready to proceed to INJECT)
    bool isTimeout();       // Timeout occurred (return to READY_TO_INJECT)
    bool isAborted();       // User pressed abort (return to READY_TO_INJECT)
    bool hasError();
    
    // Reset for next cycle
    void reset();
};

#endif
