#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>
#include "motor_control.h"
#include "button_control.h"
#include "serial_comms.h"

// Enum for Injector States
enum InjectorStates : int
{
    ERROR_STATE = 0,
    INIT_HEATING,
    INIT_HOT_NOT_HOMED,
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

// Enum for Injector Errors
enum InjectorError : uint16_t
{
    NO_ERROR = 1 << 0,
    ERROR_1 = 1 << 1,
    ERROR_2 = 1 << 2,
    ERROR_3 = 1 << 3,
    ERROR_4 = 1 << 4,
    ERROR_5 = 1 << 5,
    ERROR_6 = 1 << 6
};

// Function declarations
void transitionToState(InjectorStates toState);
void machineState();
int sanityCheck();

extern InjectorStates currentState;
extern uint16_t error;

#endif // STATE_MACHINE_H