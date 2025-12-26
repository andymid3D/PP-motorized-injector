#include "Homing.h"
#include "MessageBuffer.h"

// ===== STATIC MEMBER INITIALIZATION =====
Homing::HomingState Homing::currentState_ = Homing::HomingState::IDLE;
Homing::HomingState Homing::previousState_ = Homing::HomingState::IDLE;
uint32_t Homing::stateEnteredMs_ = 0;

bool Homing::calibrationDone_ = false;
bool Homing::encoderZeroed_ = false;
SafetyContext Homing::savedContext_ = CTX_IDLE;

Homing::DriftRecord Homing::driftHistory_[Homing::MAX_DRIFT_HISTORY];
int Homing::driftCount_ = 0;

bool Homing::calibrationComplete_ = false;
uint8_t Homing::lastSeenState_ = 0;

uint8_t Homing::lastControlModeSent_ = 255;
uint8_t Homing::lastInputModeSent_ = 255;
uint32_t Homing::modeCommandSentAtMs_ = 0;


// ===== PUBLIC INTERFACE =====

void Homing::begin(CanBusHandlerV2& motor, SafetyManager& safety) {
    // Save current safety context
    savedContext_ = safety.getContext();
    safety.setContext(CTX_MOVING_FREE);
    
    // Reset state machine
    currentState_ = HomingState::CLEAR_ERRORS;
    previousState_ = HomingState::IDLE;
    stateEnteredMs_ = millis();
    calibrationComplete_ = false;
    lastSeenState_ = 0;
}

void Homing::update(CanBusHandlerV2& motor, SafetyManager& safety) {
    // Dispatch to appropriate state handler (non-blocking)
    switch (currentState_) {
        case HomingState::CLEAR_ERRORS:     handleClearErrors(motor, safety); break;
        case HomingState::CALIBRATE:        handleCalibrate(motor, safety); break;
        case HomingState::WAIT_CALIBRATE:   handleWaitCalibrate(motor, safety); break;
        case HomingState::REQUEST_CL:       handleRequestCL(motor, safety); break;
        case HomingState::WAIT_CL:          handleWaitCL(motor, safety); break;
        case HomingState::RETRACT_FAST:     handleRetractFast(motor, safety); break;
        case HomingState::DECELERATE:       handleDecelerate(motor, safety); break;
        case HomingState::BACKOFF:          handleBackoff(motor, safety); break;
        case HomingState::APPROACH:         handleApproach(motor, safety); break;
        case HomingState::WAIT_STOP:        handleWaitStop(motor, safety); break;
        case HomingState::RESET_ENCODER:    handleResetEncoder(motor, safety); break;
        case HomingState::DONE:             handleDone(motor, safety); break;
        case HomingState::ERROR_STATE:      /* Stay in error */ break;
        default: break;
    }
}

Homing::HomingState Homing::getState() {
    return currentState_;
}

bool Homing::isComplete() {
    return currentState_ == HomingState::DONE;
}

bool Homing::hasError() {
    return currentState_ == HomingState::ERROR_STATE;
}

void Homing::reset() {
    currentState_ = HomingState::IDLE;
    previousState_ = HomingState::IDLE;
    calibrationComplete_ = false;
    lastSeenState_ = 0;
}

const char* Homing::getStateString() {
    switch (currentState_) {
        case HomingState::IDLE:            return "IDLE";
        case HomingState::CLEAR_ERRORS:    return "CLEAR_ERRORS";
        case HomingState::CALIBRATE:       return "CALIBRATE";
        case HomingState::WAIT_CALIBRATE:  return "WAIT_CALIBRATE";
        case HomingState::REQUEST_CL:      return "REQUEST_CL";
        case HomingState::WAIT_CL:         return "WAIT_CL";
        case HomingState::RETRACT_FAST:    return "RETRACT_FAST";
        case HomingState::DECELERATE:      return "DECELERATE";
        case HomingState::BACKOFF:         return "BACKOFF";
        case HomingState::APPROACH:        return "APPROACH";
        case HomingState::WAIT_STOP:       return "WAIT_STOP";
        case HomingState::RESET_ENCODER:   return "RESET_ENCODER";
        case HomingState::DONE:            return "DONE";
        case HomingState::ERROR_STATE:     return "ERROR";
        default:                           return "UNKNOWN";
    }
}

// ===== PRIVATE HELPER =====

void Homing::nextState(HomingState newState) {
    if (newState != currentState_) {
        previousState_ = currentState_;
        currentState_ = newState;
        stateEnteredMs_ = millis();
        modeCommandSentAtMs_ = 0;  // Reset timestamp for new state
        // State changes logged via main.cpp [HOMING_DEBUG] every loop iteration
    }
}

// ===== STATE HANDLERS (Non-Blocking) =====

void Homing::handleClearErrors(CanBusHandlerV2& motor, SafetyManager& safety) {
    // One-time: Clear errors
    if (previousState_ != HomingState::CLEAR_ERRORS) {
        motor.clearErrors();
        // Move to next state immediately
        nextState(calibrationDone_ ? HomingState::REQUEST_CL : HomingState::CALIBRATE);
    }
}

void Homing::handleCalibrate(CanBusHandlerV2& motor, SafetyManager& safety) {
    if (calibrationDone_) {
        // Skip calibration
        nextState(HomingState::REQUEST_CL);
        return;
    }
    
    // One-time: Request calibration
    if (previousState_ != HomingState::CALIBRATE) {
        motor.setAxisState(ODriveCANProtocol::AxisState::ENCODER_OFFSET_CALIBRATION);
        nextState(HomingState::WAIT_CALIBRATE);
    }
}

void Homing::handleWaitCalibrate(CanBusHandlerV2& motor, SafetyManager& safety) {
    BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
    uint8_t state = broadcast.getAxisState();
    
    // Detect state pattern: 1 → 7 → 1 (calibration complete)
    if (state == 7) {
        calibrationComplete_ = true;
    }
    
    if (calibrationComplete_ && state == 1) {
        calibrationDone_ = true;
        nextState(HomingState::REQUEST_CL);
    }
    
    // Timeout protection
    if (millis() - stateEnteredMs_ > 15000) {
        MessageBuffer::getInstance().sendMessage("Calibration timeout (>15s)");
        nextState(HomingState::ERROR_STATE);
    }
}

void Homing::handleRequestCL(CanBusHandlerV2& motor, SafetyManager& safety) {
    // One-time: Request Closed Loop
    if (previousState_ != HomingState::REQUEST_CL) {
        motor.setAxisState(ODriveCANProtocol::AxisState::CLOSED_LOOP_CONTROL);
        nextState(HomingState::WAIT_CL);
    }
}

void Homing::handleWaitCL(CanBusHandlerV2& motor, SafetyManager& safety) {
    BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
    
    if (broadcast.getAxisState() == 8) {
        nextState(HomingState::RETRACT_FAST);
        return;
    }
    
    // Timeout
    if (millis() - stateEnteredMs_ > 3000) {
        MessageBuffer::getInstance().sendMessage("Closed Loop failed (timeout)");
        nextState(HomingState::ERROR_STATE);
    }
}

void Homing::handleRetractFast(CanBusHandlerV2& motor, SafetyManager& safety) {
    BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
    
    // One-time: Send mode change on first entry
    if (previousState_ != HomingState::RETRACT_FAST && modeCommandSentAtMs_ == 0) {
        motor.setControllerModes(ODriveCANProtocol::ControlMode::VELOCITY_CONTROL,
                                 ODriveCANProtocol::InputMode::VEL_RAMP);
        lastControlModeSent_ = (uint8_t)ODriveCANProtocol::ControlMode::VELOCITY_CONTROL;
        lastInputModeSent_ = (uint8_t)ODriveCANProtocol::InputMode::VEL_RAMP;
        modeCommandSentAtMs_ = millis();  // Timestamp mode command sent
    }
    
    // Send velocity only after CAN_COMMAND_GAP_MS delay to allow ODrive to process mode change
    if (modeCommandSentAtMs_ > 0 && millis() - modeCommandSentAtMs_ >= CAN_COMMAND_GAP_MS) {
        motor.setInputVel(-SPEED_HOMING_FAST);  // Negative = up
    }
    
    // Check for endstop or timeout
    bool topHit = safety.getTopEndstop().isPressed();
    if (topHit) {
        motor.setInputVel(0.0f);
        nextState(HomingState::DECELERATE);
        return;
    }
    
    // Timeout (barrel length safety)
    float barrelLength = OFFSET_REFILL_GAP + OFFSET_COLD_ZONE + STROKE_HEATED_ZONE;
    float retractTimeMs = (barrelLength / SPEED_HOMING_FAST) * 1000.0f + 2000;
    if (millis() - stateEnteredMs_ > (uint32_t)retractTimeMs) {
        nextState(HomingState::ERROR_STATE);
    }
}

void Homing::handleDecelerate(CanBusHandlerV2& motor, SafetyManager& safety) {
    BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
    
    // Check if velocity is below threshold
    if (broadcast.isVelocityBelowThreshold(HOMING_VELOCITY_STOP_THRESHOLD)) {
        nextState(HomingState::BACKOFF);
        return;
    }
    
    // Timeout
    if (millis() - stateEnteredMs_ > 3000) {
        nextState(HomingState::BACKOFF);
    }
}

void Homing::handleBackoff(CanBusHandlerV2& motor, SafetyManager& safety) {
    // One-time: Send mode change on first entry
    if (previousState_ != HomingState::BACKOFF && modeCommandSentAtMs_ == 0) {
        motor.setControllerModes(ODriveCANProtocol::ControlMode::VELOCITY_CONTROL,
                                 ODriveCANProtocol::InputMode::VEL_RAMP);
        lastControlModeSent_ = (uint8_t)ODriveCANProtocol::ControlMode::VELOCITY_CONTROL;
        lastInputModeSent_ = (uint8_t)ODriveCANProtocol::InputMode::VEL_RAMP;
        modeCommandSentAtMs_ = millis();  // Timestamp mode command sent
    }
    
    // Send velocity only after CAN_COMMAND_GAP_MS delay to allow ODrive to process mode change
    if (modeCommandSentAtMs_ > 0 && millis() - modeCommandSentAtMs_ >= CAN_COMMAND_GAP_MS) {
        motor.setInputVel(HOMING_BACKOFF_VELOCITY);  // Positive = down/forward
    }
    
    // Move forward for HOMING_BACKOFF_DURATION regardless of endstop state
    // This allows the plunger to relax the endstop pressure
    if (millis() - stateEnteredMs_ >= HOMING_BACKOFF_DURATION) {
        motor.setInputVel(0.0f);
        nextState(HomingState::APPROACH);
        return;
    }
    
    // Timeout (safety net)
    if (millis() - stateEnteredMs_ > (HOMING_BACKOFF_DURATION + 5000)) {
        MessageBuffer::getInstance().sendMessage("Backoff timeout (>duration+5s)");
        nextState(HomingState::ERROR_STATE);
    }
}

void Homing::handleApproach(CanBusHandlerV2& motor, SafetyManager& safety) {
    // One-time: Send mode change on first entry
    if (previousState_ != HomingState::APPROACH && modeCommandSentAtMs_ == 0) {
        motor.setControllerModes(ODriveCANProtocol::ControlMode::VELOCITY_CONTROL,
                                 ODriveCANProtocol::InputMode::VEL_RAMP);
        lastControlModeSent_ = (uint8_t)ODriveCANProtocol::ControlMode::VELOCITY_CONTROL;
        lastInputModeSent_ = (uint8_t)ODriveCANProtocol::InputMode::VEL_RAMP;
        modeCommandSentAtMs_ = millis();  // Timestamp mode command sent
    }
    
    // Send velocity only after CAN_COMMAND_GAP_MS delay to allow ODrive to process mode change
    if (modeCommandSentAtMs_ > 0 && millis() - modeCommandSentAtMs_ >= CAN_COMMAND_GAP_MS) {
        motor.setInputVel(HOMING_APPROACH_VELOCITY);  // Negative = up, slow
    }
    
    // Check for endstop
    bool topHit = safety.getTopEndstop().isPressed();
    if (topHit) {
        motor.setInputVel(0.0f);
        nextState(HomingState::WAIT_STOP);
        return;
    }
    
    // Timeout
    if (millis() - stateEnteredMs_ > 10000) {
        MessageBuffer::getInstance().sendMessage("Approach timeout (>10s)");
        nextState(HomingState::ERROR_STATE);
    }
}

void Homing::handleWaitStop(CanBusHandlerV2& motor, SafetyManager& safety) {
    BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
    
    // Count how long velocity is below threshold
    static uint32_t stoppedSinceMs_ = 0;
    
    if (broadcast.isVelocityBelowThreshold(HOMING_VELOCITY_STOP_THRESHOLD)) {
        if (stoppedSinceMs_ == 0) {
            stoppedSinceMs_ = millis();
        }
        
        // Need 500ms of below-threshold to confirm stopped
        if (millis() - stoppedSinceMs_ > 500) {
            stoppedSinceMs_ = 0;  // Reset for next time
            nextState(HomingState::RESET_ENCODER);
            return;
        }
    } else {
        stoppedSinceMs_ = 0;  // Reset timer if velocity increases
    }
    
    // Timeout
    if (millis() - stateEnteredMs_ > 10000) {
        stoppedSinceMs_ = 0;
        nextState(HomingState::RESET_ENCODER);
    }
}

void Homing::handleResetEncoder(CanBusHandlerV2& motor, SafetyManager& safety) {
    if (encoderZeroed_) {
        nextState(HomingState::DONE);
        return;
    }
    
    // One-time: Reset encoder to 0
    if (previousState_ != HomingState::RESET_ENCODER) {
        motor.setLinearCount(0);
        encoderZeroed_ = true;
        delay(20);  // Small delay for command processing
    }
    
    nextState(HomingState::DONE);
}

void Homing::handleDone(CanBusHandlerV2& motor, SafetyManager& safety) {
    // One-time: Restore safety context
    if (previousState_ != HomingState::DONE) {
        safety.setContext(savedContext_);
    }
}

// ===== REFILL HOMING (Lightweight) =====
float Homing::checkRefillDrift(CanBusHandlerV2& motor, SafetyManager& safety) {
    // TODO: Implement refill drift checking (non-blocking version)
    // For now, return 0.0
    return 0.0f;
}

// ===== DRIFT HISTORY =====

void Homing::recordDrift(float offset) {
    if (driftCount_ < MAX_DRIFT_HISTORY) {
        driftHistory_[driftCount_].timestamp = millis();
        driftHistory_[driftCount_].offsetTurns = offset;
        driftCount_++;
    }
}

const Homing::DriftRecord* Homing::getDriftHistory(int& count) {
    count = driftCount_;
    return driftHistory_;
}

void Homing::clearDriftHistory() {
    driftCount_ = 0;
}

// ===== FLAG MANAGEMENT =====

void Homing::resetCalibrationFlag() {
    calibrationDone_ = false;
}

void Homing::resetEncoderZeroFlag() {
    encoderZeroed_ = false;
}

// ===== DEBUG: GET LAST MODES =====

uint8_t Homing::getLastControlMode() {
    return lastControlModeSent_;
}

uint8_t Homing::getLastInputMode() {
    return lastInputModeSent_;
}

