#ifndef HOMING_H
#define HOMING_H

#include "can_bus.h"
#include "safety_manager.h"
#include "broadcast_data_store.h"
#include "config.h"

class Homing {
public:
    enum class HomingState {
        IDLE,
        CLEAR_ERRORS,
        CALIBRATE,
        WAIT_CALIBRATE,
        REQUEST_CL,
        WAIT_CL,
        RETRACT_FAST,
        DECELERATE,
        BACKOFF,
        APPROACH,
        WAIT_STOP,
        RESET_ENCODER,
        DONE,
        ERROR_STATE
    };

    static void begin(CanBus& motor, SafetyManager& safety);
    static void update(CanBus& motor, SafetyManager& safety);

    static HomingState getState();
    static bool isComplete();
    static bool hasError();
    static void reset();
    static void invalidateCalibration();

    static const char* getStateString();

private:
    static HomingState currentState_;
    static HomingState previousState_;
    static uint64_t stateEnteredUs_;

    static bool calibrationDone_;
    static bool encoderZeroed_;
    static SafetyContext savedContext_;

    static bool calibrationComplete_;
    static uint8_t lastSeenState_;

    static uint8_t lastControlModeSent_;
    static uint8_t lastInputModeSent_;
    static uint64_t modeCommandSentAtUs_;
    static bool backoffVelCmdSent_;
    static bool retractVelCmdSent_;
    static bool approachVelCmdSent_;

    static SafetyManager* safetyRef_;
    static bool contextRestored_;

    static void nextState(HomingState newState);

    static void handleClearErrors(CanBus& motor, SafetyManager& safety);
    static void handleCalibrate(CanBus& motor, SafetyManager& safety);
    static void handleWaitCalibrate(CanBus& motor, SafetyManager& safety);
    static void handleRequestCL(CanBus& motor, SafetyManager& safety);
    static void handleWaitCL(CanBus& motor, SafetyManager& safety);
    static void handleRetractFast(CanBus& motor, SafetyManager& safety);
    static void handleDecelerate(CanBus& motor, SafetyManager& safety);
    static void handleBackoff(CanBus& motor, SafetyManager& safety);
    static void handleApproach(CanBus& motor, SafetyManager& safety);
    static void handleWaitStop(CanBus& motor, SafetyManager& safety);
    static void handleResetEncoder(CanBus& motor, SafetyManager& safety);
    static void handleDone(CanBus& motor, SafetyManager& safety);
};

#endif // HOMING_H
