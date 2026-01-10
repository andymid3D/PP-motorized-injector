#ifndef __INJECTOR_FSM_DUAL_H__
#define __INJECTOR_FSM_DUAL_H__

#include <stdint.h>

/**
 * Dual Finite State Machine Architecture
 * 
 * Separates STATE IDENTITY from STATE TRANSITIONS
 * - States enum: What state are we in? (IDLE, HOMING, REFILL, INJECT, etc.)
 * - Transitions struct: How do we move between states? (entry actions, exit conditions, next state)
 * 
 * Benefits:
 * - Can re-enter same state from different paths
 * - Can perform different actions on state entry based on transition source
 * - Cleaner separation of concerns (what vs how)
 * - Easier to refactor later without breaking FSM structure
 * - Can do initial homing once, then refill-only moves without re-homing
 */

// ===== STATE IDENTITY =====
enum class InjectorState : uint8_t {
    // Initialization
    ERROR_STATE             = 0,   // System error - stopped, waiting for reset
    INIT_HEATING            = 1,   // Heating up to setpoint
    INIT_HOMING             = 2,   // Calibration + encoder zeroing
    
    // Main Cycle
    IDLE                    = 3,   // Ready, waiting for user input
    REFILL                  = 4,   // Load plastic into chamber
    COMPRESSION             = 5,   // Apply pressure on plastic
    READY_TO_INJECT         = 6,   // Holding pressure, awaiting signal
    PURGE_MANUAL            = 7,   // Manual jog mode (upper/lower buttons)
    ANTIDRIP                = 8,   // Decompression before injection
    INJECT                  = 9,   // Rapid fill into mould
    HOLD_PACKING            = 10,  // Post-fill pressure (packing)
    RELEASE                 = 11,  // Retract to eject mould
    CONFIRM_MOULD_REMOVAL   = 12,  // Wait for user to remove mould
    
    MAX_STATE               = 13
};

// ===== STATE DATA =====
struct FSMState {
    InjectorState currentState = InjectorState::INIT_HEATING;
    InjectorState previousState = InjectorState::ERROR_STATE;
    uint32_t stateEntryTime = 0;      // millis() when entering current state
    uint32_t stateExitTime = 0;       // millis() when exiting current state
    uint32_t totalTimeInState = 0;    // Total milliseconds spent in this state
    uint32_t stateCount = 0;          // Number of times entered this state
    uint32_t error = 0;               // Error code if currentState == ERROR_STATE
    bool isNewState = false;          // True only on first loop of new state
};

// ===== TRANSITION ACTIONS =====
/**
 * Callbacks executed during state transitions
 * Called ONCE when entering/exiting state
 */
struct TransitionHandlers {
    // Entry: Called once when state is first entered
    void (*onEntry)(void) = nullptr;
    
    // Exit: Called once before leaving state
    void (*onExit)(void) = nullptr;
    
    // Main loop: Called every loop while in state
    void (*onUpdate)(void) = nullptr;
};

// ===== TRANSITION DECISION LOGIC =====
/**
 * Evaluate exit conditions for current state
 * Returns next state, or current state if no transition
 */
typedef InjectorState (*TransitionLogic)(void);

// ===== PUBLIC FSM INTERFACE =====

/**
 * Get human-readable state name
 */
const char* getStateName(InjectorState state);

/**
 * Get transition handlers for a state
 */
const TransitionHandlers& getStateHandlers(InjectorState state);

/**
 * Get transition logic for a state
 */
TransitionLogic getStateTransition(InjectorState state);

/**
 * Execute FSM step: check transitions, call handlers
 * Returns true if state changed
 */
bool fsmStep(FSMState& state);

// ===== STATE CONFIGURATION =====
// Define state handlers in injector_fsm.cpp
// Each state has:
//   - onEntry(): Called once when state is entered
//   - onUpdate(): Called every loop while in state
//   - onExit(): Called once before leaving state
//   - Transition logic: Evaluates exit conditions, returns next state

#endif // __INJECTOR_FSM_DUAL_H__
