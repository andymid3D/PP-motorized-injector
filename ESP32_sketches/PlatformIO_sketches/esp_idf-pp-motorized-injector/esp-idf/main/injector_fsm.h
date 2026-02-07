#ifndef INJECTOR_FSM_H
#define INJECTOR_FSM_H

#include <cstdint>
#include "mould.h"

enum InjectorStates : int {
    ERROR_STATE,
    INIT_HEATING,
    INIT_HOT_NOT_HOMED,
    INIT_HOMING,
    INIT_HOMED_ENCODER_ZEROED,
    REFILL,
    COMPRESSION,
    READY_TO_INJECT,
    PURGE_ZERO,
    ANTIDRIP,
    INJECT,
    HOLD_INJECTION,
    RELEASE,
    CONFIRM_MOULD_REMOVAL
};

struct fsm_inputs {
    bool selectButtonPressed;
    bool upButtonPressed;
    bool downButtonPressed;

    bool emergencyStop;
    bool topEndStopActivated;
    bool bottomEndStopActivated;
    bool barrelEndStopActivated;

    int nozzleTemperature;
    int64_t actualENPosition;
    int64_t actualMOTPosition;
    int trackingError;
    bool isRunning;
    bool isHoming;
    bool isCompressing;
};

using fsm_inputs_t = fsm_inputs;

struct commonInjectParams {
    float refillTrapVelLimit;
    float refillAccel;
    float refillDecel;

    float compressRampTarget;
    float compressRampDuration;
    float compressMicroCurrent;

    float injectFillTrapVelLimit;
    float injectFillAccel;
    float injectFillDecel;
    float injectFillCurrent;

    float injectPackTrapVelLimit;
    float injectPackAccel;
    float injectPackDecel;
    float injectPackCurrent;

    float injectVelThreshold;
    float injectPosLolerance;
    uint32_t injectStableTimeMs;
};

using commonInjectParams_t = commonInjectParams;

struct fsm_state {
    InjectorStates currentState;
    uint16_t error;
    actualMouldParams_t mouldParams;
};

using fsm_state_t = fsm_state;

struct fsm_outputs {
    uint32_t currentSelectLEDcolour;
    uint32_t currentUpLEDcolour;
    uint32_t currentDownLEDcolour;
    bool doCommandMotor;
    int motorCommand;
    int motorSpeed;
    int motorDistance;
    int motorAcceleration;
    bool doHeaterControl;
    int heaterTemperature;
    bool setEncoderZero;
};

using fsm_outputs_t = fsm_outputs;

#endif // INJECTOR_FSM_H
