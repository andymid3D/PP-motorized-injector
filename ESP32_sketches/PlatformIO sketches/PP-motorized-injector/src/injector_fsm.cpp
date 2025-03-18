#include "config.h"
#include "injector_fsm.h"


fsm_inputs_t fsm_inputs;
fsm_outputs_t fsm_outputs;
fsm_state_t fsm_state;


/**
 *
 */
void buttonLEDsColors(uint32_t newSelectLEDcolour, uint32_t newUpLEDcolour, uint32_t newDownLEDcolour) // could use case/switch instead? but are different types of defining behaviours..?
{
    fsm_outputs.currentSelectLEDcolour = newSelectLEDcolour;
    fsm_outputs.currentUpLEDcolour = newUpLEDcolour;
    fsm_outputs.currentDownLEDcolour = newDownLEDcolour;
    fsm_outputs.doUpdateLEDs = true;
}


////////////////////
/**
 * should run in each loop, or at least every 10ms, and if any endstop or emergency stop is triggered, stop machine
 * and report which endstop has been triggered... possible thereafter only allow moves in opposite direction, or back to refill, or
 * other options..?
 *
 * @returns InjectorError
 */
uint16_t sanityCheck()
{
  uint16_t error = InjectorError::NO_ERROR;

  if (fsm_inputs.nozzleTemperature <= minTempForAnyMove) // and stop machine
  {
    error |= InjectorError::ERROR_1;
  }

  if (fsm_inputs.barrelEndStopActivated)
  {
    error |= InjectorError::ERROR_2;
  }

  if (fsm_inputs.topEndStopActivated)
  {
    error |= InjectorError::ERROR_3;
  }

  if (fsm_inputs.bottomEndStopActivated)
  {
    error |= InjectorError::ERROR_4;
  }

  if (fsm_inputs.emergencyStop)
  {
    error |= InjectorError::ERROR_5;
  }
  // FIXME add error state when function REQUIRES something under barrel (compression, purge or mould fill)
  return error;
}

void doMotorCommand(MotorCommands command, int speed = 0, int distance = 0, int acceleration = defaultAcceletationNema)
{
  if (fsm_outputs.doCommandMotor == true) {
    // this is an error in the fsm logic, a motor command is already set for this iteration
    // should we skip this command or overwrite it? raise an error?
    // after properly testing and debugging the fsm, this should never happen
    // for now, we will overwrite the command
  }
  fsm_outputs.doCommandMotor = true;
  fsm_outputs.motorCommand = command;
  fsm_outputs.motorSpeed = speed;
  fsm_outputs.motorDistance = distance;
  fsm_outputs.motorAcceleration = acceleration;
}

void exitState(InjectorStates state)
{
  switch (state)
  {
  case InjectorStates::PURGE_ZERO:
    doMotorCommand(MotorCommands::CLEAR_STEPS);
    // stepper->setCurrentPosition(0);
    break;
  case InjectorStates::INJECT:
    doMotorCommand(MotorCommands::STOP);
    Serial.println("Motor moved " && fsm_inputs.actualMOTPosition && " steps"); // send via serial the actual steps moved by motor: in the case mould was overfilling, can be useful info for user
    break;
  case InjectorStates::HOLD_INJECTION:
    doMotorCommand(MotorCommands::STOP);
    break;
  default:
    break;
  }
}

void enterState(InjectorStates state)
{
  switch (state)
  {
  case InjectorStates::ERROR_STATE:
    doMotorCommand(MotorCommands::STOP);
    break;
  case InjectorStates::REFILL:
    doMotorCommand(MotorCommands::PROGRAMMED_MOVE, generalFastSpeed, refillOpeningOffsetDistSteps);
    break;
  case InjectorStates::HOLD_INJECTION:
    doMotorCommand(MotorCommands::PROGRAMMED_MOVE, fsm_state.mouldParams.holdMouldMoveSpeed, fsm_state.mouldParams.holdMouldMoveDistSteps);
    break;
  default:
    break;
  }
}

void machineState() //
{
  // static StatesMachine runState;  // useful here, or to make runState a Static variable between loops
  //  maybe can add "static" to above enum declaration...?

  /** button activation/ availability is activted on entering new state..? if motor move is started during transtion, STOPPING move by user
   * can only be done once in new State..?
   */

  switch (fsm_state.currentState)
  {
  case InjectorStates::ERROR_STATE: // function that includes stopMove(), flashes LEDs red (maybe with
    // Serial.println("ERROR!! " + error);

    // state action, runs every loop while the state is active
    buttonLEDsColors(RED_RGB, RED_RGB, RED_RGB); // FIXME ERROR_STATE should be all red, with flashing sequence as per Error to Identify

    if (fsm_inputs.selectButtonPressed) // FIXME ERROR_STATE this is only for testing, SHOULD NOT EXIST IN REAL SKETCH
    {
      // exit ERROR state action
      exitState(fsm_state.currentState);
      // transition action, from ERROR_STATE to INIT_HEATING
    
      // transition to next state
      fsm_state.currentState = InjectorStates::INIT_HEATING;
      // enter INIT_HEATING state action
      enterState(fsm_state.currentState);
    }
    break;

  case InjectorStates::INIT_HEATING: //  FIXME INIT_HEATING this is only for testing, SHOULD NOT EXIST IN REAL SKETCH
    // state action, runs every loop while the state is active
    buttonLEDsColors(RED_RGB, RED_RGB, RED_RGB);

    if (fsm_inputs.upButtonPressed)
    {
      // exit INIT_HEATING state action
      exitState(fsm_state.currentState);
      // transition action, from INIT_HEATING to INIT_HOT_NOT_HOMED

      // transition to next state
      fsm_state.currentState = InjectorStates::INIT_HOT_NOT_HOMED;
      // enter INIT_HOT_NOT_HOMED state action
      enterState(fsm_state.currentState);
    }

    if (fsm_inputs.nozzleTemperature > minTempForAnyMove)
    {
      // exit INIT_HEATING state action
      exitState(fsm_state.currentState);
      // transition action, from INIT_HEATING to INIT_HOT_NOT_HOMED

      // transition to next state
      fsm_state.currentState = InjectorStates::INIT_HOT_NOT_HOMED;
      // enter INIT_HOT_NOT_HOMED state action
      enterState(fsm_state.currentState);
    }

    break;

  case InjectorStates::INIT_HOT_NOT_HOMED:
    // state action, runs every loop while the state is active
    buttonLEDsColors(YELLOW_RGB, YELLOW_RGB, YELLOW_RGB);

    if (fsm_inputs.upButtonPressed)
    {
      // exit INIT_HOT_NOT_HOMED state action
      exitState(fsm_state.currentState);

      // transition action, from INIT_HOT_NOT_HOMED to INIT_HOMED_ENCODER_ZEROED
      doMotorCommand(MotorCommands::CONTIUOUS_MOVE_UP); // FIXME this should be...: ?
      // homeMove(generalFastSpeed, homingSlowSpeed);  // followed by
      // encoder.setCount(0);  // or this last line should go in machineState..? reset has to occur AFTER homeMove finishes, but not PART of homeMove

      // transition to next state
      fsm_state.currentState = InjectorStates::INIT_HOMED_ENCODER_ZEROED;

      // enter INIT_HOMED_ENCODER_ZEROED state action
      enterState(fsm_state.currentState);
    }
    if (fsm_inputs.downButtonPressed) //  FIXME INIT_HOT_NOT_HOME downButtonPressed this is only for testing, SHOULD NOT EXIST IN REAL SKETCH
    {
      // exit INIT_HOT_NOT_HOMED state action
      exitState(fsm_state.currentState);
      // transition action, from INIT_HOT_NOT_HOMED to INIT_HEATING

      // transition to next state
      fsm_state.currentState = InjectorStates::INIT_HEATING;
      // enter INIT_HEATING state action
      enterState(fsm_state.currentState);
    }
    break;

  case InjectorStates::INIT_HOMED_ENCODER_ZEROED:      // FIXME should be only the reset of the encoder, no button presses exist in real sketch
    // state action, runs every loop while the state is active
    buttonLEDsColors(YELLOW_RGB, RED_RGB, YELLOW_RGB); //  FIXME YELLOW_RGB, YELLOW_RGB, YELLOW_RGB

    if (fsm_inputs.downButtonPressed)
    {
      // exit INIT_HOMED_ENCODER_ZEROED state action
      exitState(fsm_state.currentState);
      // transition action, from INIT_HOMED_ENCODER_ZEROED to REFILL
      // transition to next state
      fsm_state.currentState = InjectorStates::REFILL;
      // enter REFILL state action
      enterState(fsm_state.currentState);
    }
    if (fsm_inputs.upButtonPressed)
    {
      // exit INIT_HOMED_ENCODER_ZEROED state action
      exitState(fsm_state.currentState);
      // transition action, from INIT_HOMED_ENCODER_ZEROED to INIT_HOT_NOT_HOMED

      // transition to next state
      fsm_state.currentState = InjectorStates::INIT_HOT_NOT_HOMED;
      // enter INIT_HOT_NOT_HOMED state action
      enterState(fsm_state.currentState);
    }
    break;

  case InjectorStates::REFILL:
    // state action, runs every loop while the state is active
    if (fsm_inputs.upButtonPressed && fsm_inputs.downButtonPressed)
    {
      endOfDayFlag = !endOfDayFlag;
    }
    if (endOfDayFlag == 0)
    {
      buttonLEDsColors(GREEN_RGB, BLACK_RGB, BLACK_RGB);
    }
    else if (endOfDayFlag == 1)
    {
      buttonLEDsColors(GREEN_RGB, BLUE_RGB, BLUE_RGB);
    }

    if (fsm_inputs.selectButtonPressed)
    {
      // exit REFILL state action
      exitState(fsm_state.currentState);
      // transition action, from REFILL to COMPRESSION
      // doMoveMotor = true;
      // compressionFunction();  // FIXME  compression function currently commented out

      // transition to next state
      fsm_state.currentState = InjectorStates::COMPRESSION;
      // enter COMPRESSION state action
      enterState(fsm_state.currentState);
    }

    break;

  case InjectorStates::COMPRESSION:
    // state action, runs every loop while the state is active
    buttonLEDsColors(RED_RGB, BLACK_RGB, RED_RGB);

    if (fsm_inputs.selectButtonPressed)
    {
      // exit COMPRESSION state action
      exitState(fsm_state.currentState);
      // transition action, from COMPRESSION to READY_TO_INJECT

      // transition to next state
      fsm_state.currentState = InjectorStates::READY_TO_INJECT;
      // enter READY_TO_INJECT state action
      enterState(fsm_state.currentState);
    }
    break;

  case InjectorStates::READY_TO_INJECT:
    // state action, runs every loop while the state is active
    buttonLEDsColors(GREEN_RGB, YELLOW_RGB, GREEN_RGB);

    if (fsm_inputs.selectButtonPressed && fsm_inputs.downButtonPressed)
    {
      // exit READY_TO_INJECT state action
      exitState(fsm_state.currentState);
      // transition action, from READY_TO_INJECT to PURGE_ZERO

      // transition to next state
      fsm_state.currentState = InjectorStates::PURGE_ZERO;
      // enter PURGE_ZERO state action
      enterState(fsm_state.currentState);
    }
    else if (fsm_inputs.upButtonPressed)
    {
      // exit READY_TO_INJECT state action
      exitState(fsm_state.currentState);
      // transition action, from READY_TO_INJECT to REFILL

      // transition to next state
      fsm_state.currentState = InjectorStates::REFILL;
      // enter REFILL state action
      enterState(fsm_state.currentState);
    }

    break;

  case InjectorStates::PURGE_ZERO:
    // state action, runs every loop while the state is active
    buttonLEDsColors(GREEN_RGB, YELLOW_RGB, YELLOW_RGB);
    if (fsm_inputs.upButtonPressed)
    {
      doMotorCommand(MotorCommands::CONTIUOUS_MOVE_UP, purgeSpeed);
    }

    if (fsm_inputs.downButtonPressed)
    {
      doMotorCommand(MotorCommands::CONTIUOUS_MOVE_DOWN, purgeSpeed);
    }

    if (fsm_inputs.selectButtonPressed)
    {
      // exit PURGE_ZERO state action
        exitState(fsm_state.currentState);
      // transition action, from PURGE_ZERO to ANTIDRIP

      // transition to next state
      fsm_state.currentState = InjectorStates::ANTIDRIP;
      // enter ANTIDRIP state action
      enterState(fsm_state.currentState);
    }

    break;

  case InjectorStates::ANTIDRIP:
    // state action, runs every loop while the state is active
    buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB);

    if (fsm_inputs.selectButtonPressed)
    {
      // exit ANTIDRIP state action
      exitState(fsm_state.currentState);
      // transition action, from ANTIDRIP to READY_TO_INJECT
      doMotorCommand(MotorCommands::STOP);
      // transition to next state
      fsm_state.currentState = READY_TO_INJECT;
      // enter READY_TO_INJECT state action
      enterState(fsm_state.currentState);
    }

    if (fsm_inputs.upButtonPressed && fsm_inputs.downButtonPressed)
    {
      // exit ANTIDRIP state action
      exitState(fsm_state.currentState);
      // transition action, from ANTIDRIP to INJECT
      doMotorCommand(MotorCommands::PROGRAMMED_MOVE, fsm_state.mouldParams.fillMouldMoveSpeed, fsm_state.mouldParams.fillMouldMoveDistSteps, fsm_state.mouldParams.fillMouldAccel);

      // transition to next state
      fsm_state.currentState = InjectorStates::INJECT;
      // enter INJECT state action
      enterState(fsm_state.currentState);
    }

    break;

  case InjectorStates::INJECT:
    // state action, runs every loop while the state is active
    buttonLEDsColors(RED_RGB, GREEN_RGB, BLACK_RGB); // middle/UP LED GREEN flashing

    if (fsm_inputs.selectButtonPressed)
    {
      // exit INJECT state action
      exitState(fsm_state.currentState);
      // transition action, from INJECT to RELEASE

      // transition to next state
      fsm_state.currentState = InjectorStates::RELEASE;
      // enter RELEASE state action
      enterState(fsm_state.currentState);
    }

    break;

  case InjectorStates::HOLD_INJECTION:
    // state action, runs every loop while the state is active
    buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB); // bottom/DOWN LED GREEN flashing

    if (fsm_inputs.selectButtonPressed)
    {
      // exit HOLD_INJECTION state action
      exitState(fsm_state.currentState);
      // transition action, from HOLD_INJECTION to RELEASE
      doMotorCommand(MotorCommands::PROGRAMMED_MOVE, releaseMouldMoveSpeed, releaseMouldMoveDistSteps); // common machine action, uses default accel

      // transition to next state
      fsm_state.currentState = InjectorStates::RELEASE;
      // enter RELEASE state action
      enterState(fsm_state.currentState);
    }

    break;

  case InjectorStates::RELEASE:
    // state action, runs every loop while the state is active
    buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);
    break;

  case InjectorStates::CONFIRM_MOULD_REMOVAL:
    // state action, runs every loop while the state is active
    buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);

    if (endOfDayFlag == 0)
    {
      if (fsm_inputs.selectButtonPressed || fsm_inputs.upButtonPressed || fsm_inputs.downButtonPressed)
      {
        // exit CONFIRM_MOULD_REMOVAL state action
        exitState(fsm_state.currentState);
        // transition action, from CONFIRM_MOULD_REMOVAL to REFILL

        // transition to next state
        fsm_state.currentState = InjectorStates::REFILL;
        // enter REFILL state action
        enterState(fsm_state.currentState);
      }
    }

    if (endOfDayFlag == 1)
    {
      if (fsm_inputs.selectButtonPressed || fsm_inputs.upButtonPressed || fsm_inputs.downButtonPressed)
      {
        // exit CONFIRM_MOULD_REMOVAL state action
        exitState(fsm_state.currentState);  
        // transition action, from CONFIRM_MOULD_REMOVAL to READY_TO_INJECT

        // transition to next state
        fsm_state.currentState = InjectorStates::READY_TO_INJECT;
        // enter READY_TO_INJECT state action
        enterState(fsm_state.currentState);
      }
    }
    break;
  }
}


void stateMachineLoop() {
  uint16_t error = sanityCheck();
  if (error != InjectorError::NO_ERROR) {
      fsm_state.currentState = InjectorStates::ERROR_STATE;

      Serial.printf("Found error!. Changing to ERROR_STATE to INIT_HEATING with error %d\n", error);
  } 
  machineState();
}
