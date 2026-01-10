#ifndef TRANSITION_ERROR_HANDLER_H
#define TRANSITION_ERROR_HANDLER_H

#include <Arduino.h>

/**
 * TransitionErrorHandler
 * 
 * Handles critical ODrive state/mode transitions where RTR failure
 * indicates fundamental hardware/communication failure requiring emergency stop.
 * 
 * The 4 Critical Transitions:
 * 1. IDLE → CLOSED_LOOP_CONTROL (State 8) - Homing/movement enable
 * 2. Mode change to POSITION_CONTROL (Mode 3) - Refill/Injection moves
 * 3. Mode change to TORQUE_CONTROL (Mode 1) - Compression/packing
 * 4. Any state → IDLE (State 1) - Emergency stop/release
 * 
 * On RTR failure at any critical transition:
 * - Log detailed error message with timestamp
 * - Set FSM to ERROR_STATE
 * - If motor is moving: Cut DC Contactor immediately (via SafetyManager)
 * - Halt further operation until power cycle
 */
class TransitionErrorHandler {
public:
    enum TransitionType {
        TRANS_IDLE_TO_CLOSED_LOOP,     // Entering closed loop control
        TRANS_MODE_TO_POSITION,         // Switching to position mode
        TRANS_MODE_TO_TORQUE,           // Switching to torque mode
        TRANS_ANY_TO_IDLE               // Emergency stop/idle
    };
    
    /**
     * Handle RTR failure during critical transition
     * @param type Type of transition that failed
     * @param axisState Current axis state from ODrive
     * @param controlMode Current control mode from ODrive
     */
    static void handleTransitionFailure(TransitionType type, 
                                       uint8_t axisState, 
                                       uint8_t controlMode);
    
    /**
     * Get human-readable name for transition type
     */
    static const char* getTransitionName(TransitionType type);
};

#endif // TRANSITION_ERROR_HANDLER_H
