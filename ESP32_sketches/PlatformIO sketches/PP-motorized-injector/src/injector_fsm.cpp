#include "config.h"
#include "injector_fsm.h"

fsm_inputs_t fsm_inputs;
fsm_outputs_t fsm_outputs;
fsm_state_t fsm_state;


void buttonLEDsColors(uint32_t newSelectLEDcolour, uint32_t newUpLEDcolour, uint32_t newDownLEDcolour)
{
  fsm_outputs.currentSelectLEDcolour = newSelectLEDcolour;
  fsm_outputs.currentUpLEDcolour = newUpLEDcolour;
  fsm_outputs.currentDownLEDcolour = newDownLEDcolour;
}

////////////////////
/**
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
  if (fsm_outputs.doCommandMotor == true)
  {
    // this is an error in the fsm logic, a motor command is already set for this iteration
    // should we skip this command or overwrite it? raise an error?
    // after properly testing and debugging the fsm, this should never happen
    // for now, we will overwrite the command
    if (fsm_outputs.motorCommand == MotorCommands::STOP && (command == CONTIUOUS_MOVE_UP || ......))
    {
      // a stop command can be overwritten by a movement ...
    }

  }
  fsm_outputs.doCommandMotor = true;
  fsm_outputs.motorCommand = command;
  fsm_outputs.motorSpeed = speed;
  fsm_outputs.motorDistance = distance;
  fsm_outputs.motorAcceleration = acceleration;
}

/**  QUESTION:  doMotorCommand (or FA library motor commands) are realised in enterState & exitState
 * or are realised in machineState..? I don't understand why they are in both..?
 *
 * also, how de we distinguish whether a move is entering or exiting a State, for example, once exiting
 * INIT_HOMED_ENCODER_ZEROED, we are entering REFILL, so the doMotorCommand_REFILL should be called in enterState
 * to REFILL or exitState of INIT_HOMED_ENCODER_ZEROED..?
 * CONSIDERATION: enterState COMPRESSION should be most indicated to actually do the motor move, as it is the best
 * place to do so, if we do in the REFILL exitState, and later want to give another exit to REFILL, then it would
 * still do the motor move, which would not be correct..? so better that the State dependent motor moves are done in enterState.?
 */

void exitState(InjectorStates state)
{
  switch (state)
  {
  case InjectorStates::COMPRESSION:
    doMotorCommand(MotorCommands::STOP);
    break;
  case InjectorStates::ANTIDRIP:
    doMotorCommand(MotorCommands::STOP);
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
    doMotorCommand(MotorCommands::HOME); // Homing function consists of various motor moves
    doMotorCommand(MotorCommands::PROGRAMMED_MOVE, generalFastSpeed, refillOpeningOffsetDistSteps);
    break;
  case InjectorStates::COMPRESSION:
    doMotorCommand(MotorCommands::COMPRESS, initialCompressionSpeed, minCompressionSpeed);
    break;
  case InjectorStates::ANTIDRIP:
    doMotorCommand(MotorCommands::PROGRAMMED_MOVE, antiDripReverseSpeed, antiDripReverseDistSteps);
    break;
    /** OBSERVATION: the EXPECTED button presses to leave this PROGRAMMED_MOVE before it finishes can interrupt this motor move..?
     * if not, then we should use a different motor command, that can be interrupted by button press, or by a timer..?
     * also, if NO button press before move is finished, we should go back to READY_TO_INJECT
     * BECAUSE OF THIS, maybe better a CONTINUOUS move, with a check against millis that stops move after ANTI_DRIP_REVERSE_TIME..?
     */
  case InjectorStates::INJECT: // FIXME   should be ABSOLUTE motor move, relative to Purge Zeroed position, after ANTIDRIP has moved plunger backwards a small amount...?
    doMotorCommand(MotorCommands::PROGRAMMED_MOVE, fsm_state.mouldParams.fillMouldMoveSpeed, fsm_state.mouldParams.fillMouldMoveDistSteps);
    break;
  case InjectorStates::HOLD_INJECTION:
    doMotorCommand(MotorCommands::PROGRAMMED_MOVE, fsm_state.mouldParams.holdMouldMoveSpeed, fsm_state.mouldParams.holdMouldMoveDistSteps);
    break;
  case InjectorStates::RELEASE:
    doMotorCommand(MotorCommands::PROGRAMMED_MOVE, releaseMouldMoveSpeed, releaseMouldMoveDistSteps);
    break;
  default:
    break;
  }
}

/**
 * QUESTION: do we need to call to exitState and enterState in machineState when there is no actual action
 * or command to be done..? or is it enough to call them when there is a command to be done..?
 * what are the consequences of calling them when there is no State that correspondes in exit or enter..?
 * 
 *  State template
 * 
 * case InjectorStates::INIT_HEATING: //  FIXME INIT_HEATING this is only for testing, SHOULD NOT EXIST IN REAL SKETCH
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
 */
void machineState() //
{

  /** button activation/ availability is activted on entering new state..? if motor move is started during transtion, STOPPING move by user
   * can only be done once in new State..?
   */

  switch (fsm_state.currentState)
  {
  case InjectorStates::ERROR_STATE: // function that includes stopMove(), flashes LEDs red (maybe with
    // Serial.println("ERROR!! " + error);

    buttonLEDsColors(RED_RGB, RED_RGB, RED_RGB); // FIXME ERROR_STATE should be all red, with flashing sequence as per Error to Identify

  case InjectorStates::INIT_HEATING:
    buttonLEDsColors(RED_RGB, RED_RGB, RED_RGB);

    if (fsm_inputs.nozzleTemperature >= minTempForAnyMove)
    {
      exitState(fsm_state.currentState);
      // no INIT_HEATING exit transtion action

      fsm_state.currentState = InjectorStates::INIT_HOT_NOT_HOMED;

      enterState(fsm_state.currentState);
      // no INIT_HOT_NOT_HOMED enter transtion action
    }

    break;

  case InjectorStates::INIT_HOT_NOT_HOMED:
    buttonLEDsColors(YELLOW_RGB, YELLOW_RGB, YELLOW_RGB);

    if (fsm_inputs.upButtonPressed)
    {
      exitState(fsm_state.currentState);
      doMotorCommand(MotorCommands::HOME);
       
      fsm_state.currentState = InjectorStates::INIT_HOMING;

      enterState(fsm_state.currentState);
      // no INIT_HOMED_ENCODER_ZEROED enter transtion action

    }

    break;

  case InjectorStates::INIT_HOMING: 
    // state action, runs every loop while the state is active
    buttonLEDsColors(YELLOW_RGB, YELLOW_RGB, YELLOW_RGB);

    if (fsm_inputs.HomingDone)
    {
      // exit INIT_HEATING state action
      exitState(fsm_state.currentState);
      // transition action, from INIT_HEATING to INIT_HOT_NOT_HOMED
      fsm_outputs.setEncoderZero = true;
      // transition to next state
      fsm_state.currentState = InjectorStates::REFILL;
      // enter INIT_HOT_NOT_HOMED state action
      enterState(fsm_state.currentState);
    }

  case InjectorStates::REFILL:
    if (endOfDayFlag == 0)
    {
      buttonLEDsColors(GREEN_RGB, BLACK_RGB, BLACK_RGB);
    }
    else if (endOfDayFlag == 1)
    {
      buttonLEDsColors(GREEN_RGB, BLUE_RGB, BLUE_RGB);
    }

    if (fsm_inputs.upButtonPressed && fsm_inputs.downButtonPressed)
    {
      endOfDayFlag = !endOfDayFlag;
    }

    if (fsm_inputs.selectButtonPressed)
    {
      exitState(fsm_state.currentState);
      // no REFILL exit transtion action

      fsm_state.currentState = InjectorStates::COMPRESSION;

      enterState(fsm_state.currentState);
      // do motor command COMPRESSION

    }

    break;

  case InjectorStates::COMPRESSION:
    buttonLEDsColors(RED_RGB, BLACK_RGB, RED_RGB);

    if (fsm_inputs.selectButtonPressed)
    {
      exitState(fsm_state.currentState);
      // do motor command STOP

      fsm_state.currentState = InjectorStates::READY_TO_INJECT;

      enterState(fsm_state.currentState);
      // no READY_TO_INJECT enter transtion action
    }
    break;
    /** QUESTION: will the state machine stay in this state until COMRESION move is complete..?
     * possible have to include state change in COMPRESSION motor move.? that seems like bad logic..
     * otherwise, how to move to next state..?
     */

  case InjectorStates::READY_TO_INJECT:
    buttonLEDsColors(GREEN_RGB, YELLOW_RGB, GREEN_RGB);

    if (fsm_inputs.selectButtonPressed && fsm_inputs.downButtonPressed)
    {
      exitState(fsm_state.currentState);
      // no READY_TO_INJECT exit transtion action

      fsm_state.currentState = InjectorStates::PURGE_ZERO;

      enterState(fsm_state.currentState);
      // no PURGE_ZERO enter transtion action
    }
    else if (fsm_inputs.upButtonPressed)
    {
      exitState(fsm_state.currentState);
      // no READY_TO_INJECT exit transtion action - NOTE: IF THERE WAS, here we have 2 different exit scenes,
      // possible one may not be appropriate for one of the destination states?

      fsm_state.currentState = InjectorStates::REFILL;

      enterState(fsm_state.currentState);
    // do motor command Home & ProgrammedMove to RefillOpeningOffsetDistSteps
  }

    break;

  case InjectorStates::PURGE_ZERO:
    buttonLEDsColors(GREEN_RGB, YELLOW_RGB, YELLOW_RGB);

    if (fsm_inputs.upButtonPressed)
    {
      doMotorCommand(MotorCommands::CONTIUOUS_MOVE_UP, purgeSpeed);
    }

    if (fsm_inputs.downButtonPressed)
    {
      doMotorCommand(MotorCommands::CONTIUOUS_MOVE_DOWN, purgeSpeed);
    }

    if (!fsm_inputs.upButtonPressed && !fsm_inputs.downButtonPressed)
    {
      doMotorCommand(MotorCommands::STOP);
    }

    if (fsm_inputs.selectButtonPressed)
    {
      exitState(fsm_state.currentState);
      // clear motor steps, set position to 0
      doMotorCommand(MotorCommands::CLEAR_STEPS);
      fsm_state.currentState = InjectorStates::ANTIDRIP;

      enterState(fsm_state.currentState);
      // do motor command PROGRAMMED_MOVE to AntiDripReverseDistSteps
    }

    break;

  case InjectorStates::ANTIDRIP:  // see OBSERVATION: in enterState

  buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB);

    if (fsm_inputs.selectButtonPressed)
    {
      exitState(fsm_state.currentState);
      // do motor command STOP

      fsm_state.currentState = READY_TO_INJECT;

      enterState(fsm_state.currentState);
      // no READY_TO_INJECT enter transtion action
    }

    if (fsm_inputs.upButtonPressed && fsm_inputs.downButtonPressed)
    {
      exitState(fsm_state.currentState);
      // do motor command STOP - as mentioned above, here we have 2 different exit scenes,
      // is a stop command appropriate for both destination states..?

      fsm_state.currentState = InjectorStates::INJECT;

      enterState(fsm_state.currentState);
      // do motor command PROGRAMMED_MOVE to fillMouldMoveDistSteps - ABSOLUTE MOVE, to fill from Purge zeroed motor

    }

  //  else if (!stepper->isRunning())   // ANTIDRIP motor move has finished, but no button press to move to next state
    else if (!fsm_outputs.doCommandMotor) // here the function will wait for the transitionToState motor move to complete before moving to the next State?? Copilot suggestion ;-)  
      {
        exitState(fsm_state.currentState);
        // do motor command STOP

        fsm_state.currentState = READY_TO_INJECT;

        enterState(fsm_state.currentState);
        // no READY_TO_INJECT enter transtion action
      } 

    break;

    /** OBSERVATION: for the next 3 states, if they are not interrupted by user cancel, then each state should
     * finish the motor move before moving to next state anyway..? so no need to do motor stop..?
     * or better from error handling point of view, to do motor stop in each state..? or at least a motor move check..?
     */


  case InjectorStates::INJECT:
    buttonLEDsColors(RED_RGB, GREEN_RGB, BLACK_RGB); // middle/UP LED GREEN flashing

    if (fsm_inputs.selectButtonPressed)
    {
      exitState(fsm_state.currentState);
      // do motor command STOP
      doMotorCommand(MotorCommands::STOP);
      fsm_state.currentState = InjectorStates::RELEASE;

      enterState(fsm_state.currentState);
      // do motor command PROGRAMMED_MOVE to releaseMouldMoveDistSteps
    }

    else if (!fsm_outputs.doCommandMotor) // here the function will wait for the transitionToState motor move to complete before moving to the next State?? Copilot suggestion ;-)
                                          // this could be used in all states, to wait for motor move to complete before moving to next state..?
                                          // or at least COMPRESSION state where there is no User action to exit state..?
    {
      exitState(fsm_state.currentState);
      // do motor command STOP

      fsm_state.currentState = InjectorStates::HOLD_INJECTION;

      enterState(fsm_state.currentState);
      // do motor command PROGRAMMED_MOVE to holdMouldMoveDistSteps
    }
    break;

  case InjectorStates::HOLD_INJECTION:
    buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB); // bottom/DOWN LED GREEN flashing

    if (fsm_inputs.selectButtonPressed)
    {
      exitState(fsm_state.currentState);
      // do motor command STOP

      fsm_state.currentState = InjectorStates::RELEASE;

      enterState(fsm_state.currentState);
      // do motor command PROGRAMMED_MOVE to releaseMouldMoveDistSteps
    }

    else if (!fsm_outputs.doCommandMotor) // here the function will wait for the transitionToState motor move to complete before moving to the next State?? Copilot suggestion ;-)
    {
      exitState(fsm_state.currentState);
      // do motor command STOP

      fsm_state.currentState = InjectorStates::RELEASE;

      enterState(fsm_state.currentState);
      // do motor command PROGRAMMED_MOVE to releaseMouldMoveDistSteps
    }

    break;

  case InjectorStates::RELEASE:
    buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);

    /** no exit/ enter strategy here..? there will be no moves when exiting to CONFIRM_MOULD_REMOVAL
     * nor when entering to CONFIRM_MOULD_REMOVAL, so no need to do motor stop..?
     * again, will motor move finishing trigger next state..? or should we have a check that motor has stopped..?
     */
    break;

  case InjectorStates::CONFIRM_MOULD_REMOVAL:
    buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);

    if (endOfDayFlag == 0)
    {
      if (fsm_inputs.selectButtonPressed || fsm_inputs.upButtonPressed || fsm_inputs.downButtonPressed)
      {
        exitState(fsm_state.currentState);
        // no CONFIRM_MOULD_REMOVAL exit transtion action

        fsm_state.currentState = InjectorStates::REFILL;

        enterState(fsm_state.currentState);
        // do motor command Home & ProgrammedMove to RefillOpeningOffsetDistSteps

      }
    }

    if (endOfDayFlag == 1)
    {
      if (fsm_inputs.selectButtonPressed || fsm_inputs.upButtonPressed || fsm_inputs.downButtonPressed)
      {
        exitState(fsm_state.currentState);
        // no CONFIRM_MOULD_REMOVAL exit transtion action

        fsm_state.currentState = InjectorStates::READY_TO_INJECT;

        enterState(fsm_state.currentState);
        // no READY_TO_INJECT enter transtion action
      }
    }
    break;
  }
}

void stateMachineLoop()
{
  uint16_t error = sanityCheck();
  if (error != InjectorError::NO_ERROR)
  {
    fsm_state.currentState = InjectorStates::ERROR_STATE;

    Serial.printf("Found error!. Changing to ERROR_STATE to INIT_HEATING with error %d\n", error); // ?? why INIT_HEATING..?
  }
  machineState();
}
