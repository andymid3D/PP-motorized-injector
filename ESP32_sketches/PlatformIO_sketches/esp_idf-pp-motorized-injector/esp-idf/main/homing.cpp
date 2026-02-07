#include "homing.h"
#include "motor_wrapper.h"
#include "time_utils.h"
#include "esp_log.h"
#include <cmath>

static const char* TAG = "HOMING";

static const char* homingStateName(Homing::HomingState state) {
    switch (state) {
        case Homing::HomingState::IDLE: return "IDLE";
        case Homing::HomingState::CLEAR_ERRORS: return "CLEAR_ERRORS";
        case Homing::HomingState::CALIBRATE: return "CALIBRATE";
        case Homing::HomingState::WAIT_CALIBRATE: return "WAIT_CALIBRATE";
        case Homing::HomingState::REQUEST_CL: return "REQUEST_CL";
        case Homing::HomingState::WAIT_CL: return "WAIT_CL";
        case Homing::HomingState::RETRACT_FAST: return "RETRACT_FAST";
        case Homing::HomingState::DECELERATE: return "DECELERATE";
        case Homing::HomingState::BACKOFF: return "BACKOFF";
        case Homing::HomingState::APPROACH: return "APPROACH";
        case Homing::HomingState::WAIT_STOP: return "WAIT_STOP";
        case Homing::HomingState::RESET_ENCODER: return "RESET_ENCODER";
        case Homing::HomingState::DONE: return "DONE";
        case Homing::HomingState::ERROR_STATE: return "ERROR";
        default: return "UNKNOWN";
    }
}

Homing::HomingState Homing::currentState_ = Homing::HomingState::IDLE;
Homing::HomingState Homing::previousState_ = Homing::HomingState::IDLE;
uint64_t Homing::stateEnteredUs_ = 0;

bool Homing::calibrationDone_ = false;
bool Homing::encoderZeroed_ = false;
SafetyContext Homing::savedContext_ = CTX_IDLE;

bool Homing::calibrationComplete_ = false;
uint8_t Homing::lastSeenState_ = 0;

uint8_t Homing::lastControlModeSent_ = 255;
uint8_t Homing::lastInputModeSent_ = 255;
uint64_t Homing::modeCommandSentAtUs_ = 0;
bool Homing::backoffVelCmdSent_ = false;
bool Homing::retractVelCmdSent_ = false;
bool Homing::approachVelCmdSent_ = false;

SafetyManager* Homing::safetyRef_ = nullptr;
bool Homing::contextRestored_ = false;

void Homing::begin(CanBus& motor, SafetyManager& safety) {
    safetyRef_ = &safety;
    contextRestored_ = false;
    savedContext_ = safety.getContext();
    safety.setContext(CTX_MOVING_FREE);

    currentState_ = HomingState::CLEAR_ERRORS;
    previousState_ = HomingState::IDLE;
    stateEnteredUs_ = time_utils::micros();
    calibrationComplete_ = false;
    lastSeenState_ = 0;
    encoderZeroed_ = false;
    modeCommandSentAtUs_ = 0;
    backoffVelCmdSent_ = false;
    retractVelCmdSent_ = false;
    approachVelCmdSent_ = false;
}

void Homing::update(CanBus& motor, SafetyManager& safety) {
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
        case HomingState::ERROR_STATE:      break;
        default: break;
    }
}

Homing::HomingState Homing::getState() { return currentState_; }

bool Homing::isComplete() { return currentState_ == HomingState::DONE; }

bool Homing::hasError() { return currentState_ == HomingState::ERROR_STATE; }

void Homing::reset() {
    currentState_ = HomingState::IDLE;
    previousState_ = HomingState::IDLE;
    calibrationComplete_ = false;
    calibrationDone_ = false;
    encoderZeroed_ = false;
    modeCommandSentAtUs_ = 0;
    lastSeenState_ = 0;
    backoffVelCmdSent_ = false;
    retractVelCmdSent_ = false;
    approachVelCmdSent_ = false;
}

void Homing::invalidateCalibration() {
    calibrationComplete_ = false;
    calibrationDone_ = false;
    encoderZeroed_ = false;
    lastSeenState_ = 0;
}

const char* Homing::getStateString() {
    switch (currentState_) {
        case HomingState::IDLE: return "IDLE";
        case HomingState::CLEAR_ERRORS: return "CLEAR_ERRORS";
        case HomingState::CALIBRATE: return "CALIBRATE";
        case HomingState::WAIT_CALIBRATE: return "WAIT_CALIBRATE";
        case HomingState::REQUEST_CL: return "REQUEST_CL";
        case HomingState::WAIT_CL: return "WAIT_CL";
        case HomingState::RETRACT_FAST: return "RETRACT_FAST";
        case HomingState::DECELERATE: return "DECELERATE";
        case HomingState::BACKOFF: return "BACKOFF";
        case HomingState::APPROACH: return "APPROACH";
        case HomingState::WAIT_STOP: return "WAIT_STOP";
        case HomingState::RESET_ENCODER: return "RESET_ENCODER";
        case HomingState::DONE: return "DONE";
        case HomingState::ERROR_STATE: return "ERROR";
        default: return "UNKNOWN";
    }
}

void Homing::nextState(HomingState newState) {
    if (newState != currentState_) {
        previousState_ = currentState_;
        currentState_ = newState;
        stateEnteredUs_ = time_utils::micros();
        modeCommandSentAtUs_ = 0;
        backoffVelCmdSent_ = false;
        retractVelCmdSent_ = false;
        approachVelCmdSent_ = false;
#if APP_DEBUG
        ESP_LOGI(TAG, "STATE %s -> %s", homingStateName(previousState_), homingStateName(newState));
#endif
        if (!contextRestored_ && safetyRef_ &&
            (newState == HomingState::DONE || newState == HomingState::ERROR_STATE)) {
            safetyRef_->setContext(savedContext_);
            contextRestored_ = true;
        }
    }
}

void Homing::handleClearErrors(CanBus& motor, SafetyManager& safety) {
    if (previousState_ != HomingState::CLEAR_ERRORS) {
        motor.clearErrors();
        nextState(calibrationDone_ ? HomingState::REQUEST_CL : HomingState::CALIBRATE);
    }
}

void Homing::handleCalibrate(CanBus& motor, SafetyManager& safety) {
    if (calibrationDone_) {
        nextState(HomingState::REQUEST_CL);
        return;
    }
    if (previousState_ != HomingState::CALIBRATE) {
        motor.setAxisState(odrive_can::AxisState::ENCODER_OFFSET_CALIBRATION);
        nextState(HomingState::WAIT_CALIBRATE);
    }
}

void Homing::handleWaitCalibrate(CanBus& motor, SafetyManager& safety) {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    uint8_t state = bds.getAxisState();
    uint32_t encoderErr = bds.getEncoderError();

    if (encoderErr == 0x100) {
        motor.clearErrors();
        calibrationComplete_ = false;
        nextState(HomingState::CALIBRATE);
        return;
    }

    if (state == 7) calibrationComplete_ = true;
    if (calibrationComplete_ && state == 1) {
        calibrationDone_ = true;
        nextState(HomingState::REQUEST_CL);
    }

    if (time_utils::micros() - stateEnteredUs_ > 15000000ULL) {
        nextState(HomingState::ERROR_STATE);
    }
}

void Homing::handleRequestCL(CanBus& motor, SafetyManager& safety) {
    if (previousState_ != HomingState::REQUEST_CL) {
        motor.setAxisState(odrive_can::AxisState::CLOSED_LOOP_CONTROL);
        nextState(HomingState::WAIT_CL);
    }
}

void Homing::handleWaitCL(CanBus& motor, SafetyManager& safety) {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    uint64_t motorErr = bds.getMotorError();

    if (motorErr == 0x40) {
        motor.clearErrors();
        calibrationDone_ = false;
        calibrationComplete_ = false;
        nextState(HomingState::CALIBRATE);
        return;
    }

    uint8_t currentState = bds.getAxisState();
    uint64_t lastUpdate = bds.getLastAxisUpdate();
    uint64_t currentTime = time_utils::micros();
    uint64_t staleness = currentTime - lastUpdate;

    if (staleness > 50000) {
        return;
    }

    if (currentState == 8) {
        nextState(HomingState::RETRACT_FAST);
        return;
    }

    if (currentTime - stateEnteredUs_ > 3000000ULL) {
        nextState(HomingState::ERROR_STATE);
    }
}

void Homing::handleRetractFast(CanBus& motor, SafetyManager& safety) {
    if (previousState_ != HomingState::RETRACT_FAST && modeCommandSentAtUs_ == 0) {
        MotorWrapper::setControllerModes(motor, odrive_can::ControlMode::VELOCITY_CONTROL,
                                         odrive_can::InputMode::VEL_RAMP, MODULE_INIT_HOMING, "RetractFast");
        lastControlModeSent_ = (uint8_t)odrive_can::ControlMode::VELOCITY_CONTROL;
        lastInputModeSent_ = (uint8_t)odrive_can::InputMode::VEL_RAMP;
        modeCommandSentAtUs_ = time_utils::micros();
    }

    if (modeCommandSentAtUs_ > 0 && !retractVelCmdSent_) {
        bool commandQueued = MotorWrapper::setModeAndMoveWithRetry(
            motor, 2, 2, HOMING_FAST_VEL, MODULE_INIT_HOMING,
            "RetractFast", MotorWrapper::PRIORITY_NORMAL
        );
        if (commandQueued) retractVelCmdSent_ = true;
    }

    if (safety.getTopEndstop().isPressed()) {
        motor.setInputVel(0.0f);
        nextState(HomingState::DECELERATE);
        return;
    }

    float barrelLength = OFFSET_REFILL_GAP + OFFSET_COLD_ZONE + STROKE_HEATED_ZONE;
    float retractTimeMs = (barrelLength / std::fabs(HOMING_FAST_VEL)) * 1000.0f + 2000;
    if (time_utils::micros() - stateEnteredUs_ > ((uint32_t)retractTimeMs * 1000ULL)) {
        nextState(HomingState::ERROR_STATE);
    }
}

void Homing::handleDecelerate(CanBus& motor, SafetyManager& safety) {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    if (bds.isVelocityBelowThreshold(HOMING_STOP_THRESHOLD)) {
        nextState(HomingState::BACKOFF);
        return;
    }
    if (time_utils::micros() - stateEnteredUs_ > 3000000ULL) {
        nextState(HomingState::BACKOFF);
    }
}

void Homing::handleBackoff(CanBus& motor, SafetyManager& safety) {
    if (previousState_ != HomingState::BACKOFF && modeCommandSentAtUs_ == 0) {
        MotorWrapper::setControllerModes(motor, odrive_can::ControlMode::VELOCITY_CONTROL,
                                         odrive_can::InputMode::VEL_RAMP, MODULE_INIT_HOMING, "Backoff");
        lastControlModeSent_ = (uint8_t)odrive_can::ControlMode::VELOCITY_CONTROL;
        lastInputModeSent_ = (uint8_t)odrive_can::InputMode::VEL_RAMP;
        modeCommandSentAtUs_ = time_utils::micros();
    }

    if (modeCommandSentAtUs_ > 0 && !backoffVelCmdSent_) {
        bool commandQueued = MotorWrapper::setModeAndMoveWithRetry(
            motor, 2, 2, HOMING_BACKOFF_VEL, MODULE_INIT_HOMING,
            "Backoff", MotorWrapper::PRIORITY_NORMAL
        );
        if (commandQueued) backoffVelCmdSent_ = true;
    }

    uint64_t elapsed = time_utils::micros() - stateEnteredUs_;
    if (elapsed >= (HOMING_BACKOFF_DURATION * 1000ULL)) {
        motor.setInputVel(0.0f);
        nextState(HomingState::APPROACH);
        return;
    }

    if (time_utils::micros() - stateEnteredUs_ > ((HOMING_BACKOFF_DURATION + 5000) * 1000ULL)) {
        nextState(HomingState::ERROR_STATE);
    }
}

void Homing::handleApproach(CanBus& motor, SafetyManager& safety) {
    if (previousState_ != HomingState::APPROACH && modeCommandSentAtUs_ == 0) {
        MotorWrapper::setControllerModes(motor, odrive_can::ControlMode::VELOCITY_CONTROL,
                                         odrive_can::InputMode::VEL_RAMP, MODULE_INIT_HOMING, "Approach");
        lastControlModeSent_ = (uint8_t)odrive_can::ControlMode::VELOCITY_CONTROL;
        lastInputModeSent_ = (uint8_t)odrive_can::InputMode::VEL_RAMP;
        modeCommandSentAtUs_ = time_utils::micros();
    }

    if (modeCommandSentAtUs_ > 0 && !approachVelCmdSent_) {
        bool commandQueued = MotorWrapper::setModeAndMoveWithRetry(
            motor, 2, 2, HOMING_APPROACH_VEL, MODULE_INIT_HOMING,
            "Approach", MotorWrapper::PRIORITY_NORMAL
        );
        if (commandQueued) approachVelCmdSent_ = true;
    }

    if (safety.getTopEndstop().isPressed()) {
        motor.setInputVel(0.0f);
        nextState(HomingState::WAIT_STOP);
        return;
    }

    if (time_utils::micros() - stateEnteredUs_ > 10000000ULL) {
        nextState(HomingState::ERROR_STATE);
    }
}

void Homing::handleWaitStop(CanBus& motor, SafetyManager& safety) {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    static uint64_t stoppedSinceUs_ = 0;

    if (bds.isVelocityBelowThreshold(HOMING_STOP_THRESHOLD)) {
        if (stoppedSinceUs_ == 0) stoppedSinceUs_ = time_utils::micros();
        if (time_utils::micros() - stoppedSinceUs_ > 500000ULL) {
            stoppedSinceUs_ = 0;
            nextState(HomingState::RESET_ENCODER);
            return;
        }
    } else {
        stoppedSinceUs_ = 0;
    }

    if (time_utils::micros() - stateEnteredUs_ > 10000000ULL) {
        stoppedSinceUs_ = 0;
        nextState(HomingState::RESET_ENCODER);
    }
}

void Homing::handleResetEncoder(CanBus& motor, SafetyManager& safety) {
    if (encoderZeroed_) {
        nextState(HomingState::DONE);
        return;
    }
    motor.setLinearCount(0);
    encoderZeroed_ = true;
    nextState(HomingState::DONE);
}

void Homing::handleDone(CanBus& motor, SafetyManager& safety) {
    if (previousState_ != HomingState::DONE && !contextRestored_) {
        safety.setContext(savedContext_);
        contextRestored_ = true;
    }
}
