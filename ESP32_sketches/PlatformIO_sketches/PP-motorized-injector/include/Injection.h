#ifndef INJECTION_H
#define INJECTION_H

#include <Arduino.h>
#include "CanBusHandlerV2.h"
#include "injector_fsm.h"  // For actualMouldParams_t

/*
 * INJECTION STATE MACHINE
 * 
 * Purpose: Execute injection cycle with automatic transition from INJECT → HOLD
 * 
 * Flow:
 *   INJECT Phase:
 *   1. Capture start position
 *   2. Set target = startPos + fillVolume
 *   3. Apply position control to reach target
 *   4. Auto-transition to HOLD when velocity < threshold and position > 500ms stable
 *   
 *   HOLD Phase (Packing):
 *   1. Capture pack start position
 *   2. Set target = packStartPos + packVolume
 *   3. Apply position control with lower pressure
 *   4. Hold for packTime seconds
 *   5. Auto-transition to RELEASE on timeout
 * 
 * Pressure Sensor: Check during first ms to confirm mould is blocked
 * Context: CTX_BLOCKED (no plunger reversal allowed)
 * Motor Control: Position Control (Mode 3)
 * 
 * Note: Both INJECT and HOLD use actualMouldParams struct which comes from display/library
 * User can abort at any time (Upper button) to proceed to RELEASE
 */

namespace Injection {
    enum InjectionPhase { FILLING, PACKING, DONE };
    
    // Initialize with mould parameters
    void begin(const actualMouldParams_t& mouldParams);
    
    // Non-blocking update
    bool update(CanBusHandlerV2& motor);
    
    // Query state
    bool isComplete();      // Cycle finished successfully
    bool hasError();        // Error occurred
    InjectionPhase getPhase();  // Current phase (FILLING, PACKING, DONE)
    
    // Reset for next cycle
    void reset();
};

#endif
