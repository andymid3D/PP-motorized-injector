#ifndef COMPRESSION_H
#define COMPRESSION_H

#include <Arduino.h>
#include "CanBusHandlerV2.h"

/*
 * COMPRESSION STATE MACHINE
 * 
 * Purpose: Apply torque ramp to compress plastic in barrel
 * Supports two modes: Full travel (Mode 1) or micro-compression (Mode 2)
 * 
 * MODE 1 (Post-Refill):
 *   1. Travel down from OFFSET_REFILL_GAP until contact with plastic (~5-10cm travel)
 *   2. Once contact (velocity drops, pressure detected), switch to torque ramp
 *   3. Apply torque ramp until target TORQUE_COMPRESSION_HOLD reached or timeout
 *   4. On timeout/no plastic: return to REFILL
 *   5. On success: return to READY_TO_INJECT
 * 
 * MODE 2 (Micro-compression in ReadyToInject):
 *   1. Skip travel phase (already in contact)
 *   2. Apply torque ramp directly
 *   3. Complete in ~2 seconds
 * 
 * Pressure Sensor: Check during first ms to confirm mould/nozzleblock is in place
 * Context: CTX_BLOCKED
 * Motor Control: Velocity (Mode 1 travel) → Torque (compression)
 * 
 * Future Enhancement: Implement vel_ramp to medium speed with low torque check
 * (once plastic contact confirmed), then switch to torque mode.
 */

namespace Compression {
    enum CompressionMode { MODE_1_TRAVEL, MODE_2_MICRO };
    
    // Initialize with mode selection
    void begin(CompressionMode mode);
    
    // Non-blocking update
    bool update(CanBusHandlerV2& motor);
    
    // Query state
    bool isComplete();      // Successfully completed
    bool hasError();        // Error occurred
    bool isTimeout();       // Timeout (no plastic detected)
    
    // Reset for next cycle
    void reset();
};

#endif
