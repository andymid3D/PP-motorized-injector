#ifndef INJECTOR_FSM_H
#define INJECTOR_FSM_H
////////////////////////////////
// State machine
////////////////////////////////
#include <Arduino.h>

enum InjectorError : uint16_t {
    NO_ERROR = 1 << 0,
    ERROR_1  = 1 << 1,  // temp below min
    ERROR_2  = 1 << 2,  // barrel endstop activated
    ERROR_3  = 1 << 3,  // top endstop activaded outside of function
    ERROR_4  = 1 << 4,  // bottom endstop (activaded outside of function)
    ERROR_5  = 1 << 5,  // emergency stop pressed
    ERROR_6  = 1 << 6,  // barrel block NOT active in function that requieres
  };
  
  // StatesMachine, states that the machine may be in at any point in time
  
  enum InjectorStates : int{
    ERROR_STATE = 0,                // SOME TYPE OF CHECK TRIGGERED
    INIT_HEATING = 1,               // INITIAL POWER ON STATES
    INIT_HOT_NOT_HOMED = 2,         // INITIAL POWER ON STATES
    INIT_HOMED_ENCODER_ZEROED = 3,  // INITIAL POWER ON STATES
    REFILL,                     // DEFAULT WAITING STATES
    COMPRESSION,                // DEFAULT WAITING STATES
    READY_TO_INJECT,            // DEFAULT WAITING STATES
    PURGE_ZERO,                 // INJECT PROCESS STATES, normally will start here...
    ANTIDRIP,                   // INJECT PROCESS STATES,
    INJECT,                     // INJECT PROCESS STATES,
    HOLD_INJECTION,             // INJECT PROCESS STATES,
    RELEASE,                    // INJECT PROCESS STATES,
    CONFIRM_MOULD_REMOVAL       // INJECT PROCESS STATES, ... and end here before returning 
                                // to Refill or READY_TO_INJECT, depending on EndOfDayFlag
  };

extern boolean readNozzleTemp;
extern int nozzleTemperature;

extern boolean readEmergencyStop;
extern boolean emergencyStop;

extern boolean readSelectButton;
extern boolean selectButtonPressed;

extern boolean readUpButton;
extern boolean upButtonPressed;

extern boolean readDownButton;
extern boolean downButtonPressed;

extern boolean readTopEndStop;
extern boolean topEndStopActivated;

extern boolean readBottomEndStop;
extern boolean bottomEndStopActivated;

extern boolean readBarrelEndStop;
extern boolean barrelEndStopActivated;

extern InjectorStates currentState;
extern uint16_t error;


#endif