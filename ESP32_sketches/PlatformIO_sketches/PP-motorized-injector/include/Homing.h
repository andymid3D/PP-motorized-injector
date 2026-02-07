#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include "CanBusHandlerV2.h"
#include "SafetyManager.h"
#include "BroadcastDataStore.h"
#include "config.h"

/**
 * Homing Module - Non-Blocking State Machine Implementation
 * 
 * CRITICAL PRINCIPLE: Non-blocking architecture using state machines
 * - All methods return quickly (no while loops, no delay())
 * - Call update() each loop() to process one state transition
 * - Uses BroadcastDataStore for axis state/velocity (no repeated CAN queries)
 * - Uses millisDelay for non-blocking timing
 * 
 * States:
 *   IDLE → CLEAR_ERRORS → CALIBRATE → WAIT_CALIBRATE → REQUEST_CL → WAIT_CL
 *   → RETRACT_FAST → DECELERATE → BACKOFF → APPROACH → WAIT_STOP → RESET_ENCODER → DONE
 * 
 * Usage:
 *   In setup():   Homing::begin();
 *   In loop():    if (!Homing::isComplete()) Homing::update(motor, safety);
 *   In config:    Can skip steps via flags (calibrationDone, encoderZeroed)
 */

class Homing {
public:
    // ===== STATE MACHINE =====
    enum class HomingState {
        IDLE,               // Not running
        CLEAR_ERRORS,       // Clear any motor errors
        CALIBRATE,          // Request encoder calibration (State 7)
        WAIT_CALIBRATE,     // Wait for calibration to complete (1→7→1 pattern)
        REQUEST_CL,         // Request Closed Loop (State 8)
        WAIT_CL,            // Wait for Closed Loop to activate
        RETRACT_FAST,       // Fast retract up to top endstop
        DECELERATE,         // Decelerate and wait for motor stop
        BACKOFF,            // Move down slightly to relax endstop pressure
        APPROACH,           // Slow approach back to endstop
        WAIT_STOP,          // Wait for motor to stop completely
        RESET_ENCODER,      // Reset encoder to 0
        DONE,               // Homing complete
        ERROR_STATE         // Something went wrong
    };
    
    // ===== MAIN INTERFACE =====
    
    // Begin homing sequence (call once from setup or state machine)
    static void begin(CanBusHandlerV2& motor, SafetyManager& safety);
    
    // Process one state transition (call each loop() while not complete)
    static void update(CanBusHandlerV2& motor, SafetyManager& safety);
    
    // Query current state
    static HomingState getState();
    static bool isComplete();
    static bool hasError();
    
    // Reset for next homing attempt
    static void reset();
    
    // ===== DRIFT TRACKING (for refill homing) =====
    static const int MAX_DRIFT_HISTORY = 100;
    struct DriftRecord {
        uint64_t timestamp;
        float offsetTurns;
    };
    
    static float checkRefillDrift(CanBusHandlerV2& motor, SafetyManager& safety);
    static const DriftRecord* getDriftHistory(int& count);
    static void clearDriftHistory();
    
    // ===== FLAG MANAGEMENT =====
    static void resetCalibrationFlag();
    static void resetEncoderZeroFlag();
    static const char* getStateString();
    
    // ===== DEBUG: Last sent modes =====
    static uint8_t getLastControlMode();
    static uint8_t getLastInputMode();

private:
    // State machine state
    static HomingState currentState_;
    static HomingState previousState_;
    static uint64_t stateEnteredUs_;  // Timestamp when state entered (microseconds)
    
    // Persistent flags (survive power cycles)
    static bool calibrationDone_;
    static bool encoderZeroed_;
    
    // Drift history
    static DriftRecord driftHistory_[MAX_DRIFT_HISTORY];
    static int driftCount_;
    
    // Saved context (for restoring after homing)
    static SafetyContext savedContext_;
    
    // Helper: Detect state 1→7→1 calibration pattern
    static bool calibrationComplete_;
    static uint8_t lastSeenState_;
    
    // Debug: Track last modes sent
    static uint8_t lastControlModeSent_;
    static uint8_t lastInputModeSent_;
    
    // Critical: Timestamp when mode command was sent (for 50ms gap before velocity command)
    static uint64_t modeCommandSentAtUs_;  // Timestamp in microseconds
    static bool backoffVelCmdSent_;  // Track if velocity command sent during backoff
    static bool retractVelCmdSent_;  // Track if velocity command sent during retract
    static bool approachVelCmdSent_; // Track if velocity command sent during approach

    // Safety context restoration tracking
    static SafetyManager* safetyRef_;
    static bool contextRestored_;
    
    // Transition to next state
    static void nextState(HomingState newState);
    
    // State handlers (process current state)
    static void handleClearErrors(CanBusHandlerV2& motor, SafetyManager& safety);
    static void handleCalibrate(CanBusHandlerV2& motor, SafetyManager& safety);
    static void handleWaitCalibrate(CanBusHandlerV2& motor, SafetyManager& safety);
    static void handleRequestCL(CanBusHandlerV2& motor, SafetyManager& safety);
    static void handleWaitCL(CanBusHandlerV2& motor, SafetyManager& safety);
    static void handleRetractFast(CanBusHandlerV2& motor, SafetyManager& safety);
    static void handleDecelerate(CanBusHandlerV2& motor, SafetyManager& safety);
    static void handleBackoff(CanBusHandlerV2& motor, SafetyManager& safety);
    static void handleApproach(CanBusHandlerV2& motor, SafetyManager& safety);
    static void handleWaitStop(CanBusHandlerV2& motor, SafetyManager& safety);
    static void handleResetEncoder(CanBusHandlerV2& motor, SafetyManager& safety);
    static void handleDone(CanBusHandlerV2& motor, SafetyManager& safety);
    
    // Helper: Record drift measurement
    static void recordDrift(float offset);
};

#endif // HOMING_H
