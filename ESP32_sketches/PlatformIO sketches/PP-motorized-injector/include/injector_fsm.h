#ifndef INJECTOR_FSM_H
#define INJECTOR_FSM_H
////////////////////////////////
// State machine
////////////////////////////////
#include <Arduino.h>
#include "mould.h"

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
    ERROR_STATE,                // SOME TYPE OF CHECK TRIGGERED
    INIT_HEATING,               // INITIAL POWER ON STATES
    INIT_HOT_NOT_HOMED,         // INITIAL POWER ON STATES
    INIT_HOMING,
    INIT_HOMED_ENCODER_ZEROED,  // INITIAL POWER ON STATES
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

  /** QUESTION: why do we want to abstract the commands already present in the FA motor library..? why not just call them directly each time, why do we want to add 
  * an extra layer of code to do what the library already is prepared for..? for COMPRESS & HOME, is different, as is a separate function with a combination of 
  * motor and algoritmic comparisons to change speeds during the function duration
  */
  enum MotorCommands : int {
    STOP,                    // stop motor
    HOME,                    // move motor until endstop is activated, then set position to 0
    CONTIUOUS_MOVE_UP,       // move motor continuously up
    CONTIUOUS_MOVE_DOWN,     // move motor continuously down
    PROGRAMMED_MOVE,         // move motor a programmed distance FIXME: should this be relative or absolute? or dowe need another command for absolute moves?
    HOME,                    // function to move motor quickly until endstop is activated, back off slowly, comeback slowly, set position to 0, and offset a certain amount
    COMPRESS,                // move motor to compress the plastic in the barrel, assume some step loss might occur if needed
    CLEAR_STEPS              // clear the steps counter (clear to zero or adjust to encoder reading? we may need this after a compression)
  };

  typedef struct fsm_inputs {

    // buttons 
    boolean selectButtonPressed;
    boolean upButtonPressed;
    boolean downButtonPressed;
  
    boolean emergencyStop;
    // endstops
    boolean topEndStopActivated;
    boolean bottomEndStopActivated;
    boolean barrelEndStopActivated;
    // temperature
    int nozzleTemperature;
    // encoder and motor
    int64_t actualENPosition;
    int64_t actualMOTPosition;
    int trackingError;

    bool HomingDone;
  
  } fsm_inputs_t;
  
  typedef struct fsm_state {
    InjectorStates currentState;
    uint16_t error;
    actualMouldParams_t mouldParams;
  } fsm_state_t;
  
  typedef struct fsm_outputs {
  // LEDS
    uint32_t currentSelectLEDcolour;
    uint32_t currentUpLEDcolour;
    uint32_t currentDownLEDcolour;
    // motor
    boolean doCommandMotor;
    MotorCommands motorCommand;
    int motorSpeed;
    int motorDistance;
    int motorAcceleration;
    // heater <--- not sure if this is needed, as HEATER CONTROL is external to the electronics, completely manual... temperature is read only
    //            but dictates whether machine can move or not once reached minTempForAnyMove
    boolean doHeaterControl;
    int heaterTemperature;
    // encoder
    boolean setEncoderZero;
  
  } fsm_outputs_t;
  
extern fsm_inputs_t fsm_inputs;
extern fsm_outputs_t fsm_outputs;
extern fsm_state_t fsm_state;

void stateMachineLoop();

#endif // INJECTOR_FSM_H