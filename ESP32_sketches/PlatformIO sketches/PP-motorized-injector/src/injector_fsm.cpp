#include "config.h"
#include "injector_fsm.h"



InjectorStates currentState = InjectorStates::INIT_HEATING;  // declaring variable runState can only have valid values of enum
uint16_t error = InjectorError::NO_ERROR;

fsm_inputs_t fsm_inputs;
fsm_outputs_t fsm_outputs;
fsm_state_t fsm_state;



////////////////////
/**
 * should run in each loop, or at least every 10ms, and if any endstop or emergency stop is triggered, stop machine
 * and report which endstop has been triggered... possible thereafter only allow moves in opposite direction, or back to refill, or
 * other options..?
 *
 * @returns InjectorError
 */
int sanityCheck()
{
  int error = InjectorError::NO_ERROR;

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
/**
 * to transition states would also apply button activation / availablity..? or this only in machine states..?
 * some motor moves user may want to stop before motor move is completed..? INJECT or HOLD, or COMPRESSION
 * how to apply ContinuousMotorMoves..? As in Purge, where State does not change whilst moving motor.. this function must be in Machine State..?
 *
 */
void transitionToState(InjectorStates toState)
{
  if (currentState != toState)
  {
    Serial.printf("Changing from %d to %d\n", currentState, toState);

    if (toState == ERROR_STATE)
    {
      stepper->stopMove();
    }

    else if (toState == INIT_HOMED_ENCODER_ZEROED)
    {
      doMoveMotor = true;
      continuousMotorMoveUp(generalFastSpeed); // FIXME this should be...: ?
      // homeMove(generalFastSpeed, homingSlowSpeed);  // followed by
      // encoder.setCount(0);  // or this last line should go in machineState..? reset has to occur AFTER homeMove finishes, but not PART of homeMove
      // transitionToState(InjectorStates::REFILL);  and finally transition to next State
    }
    else if (toState == REFILL) // once INIT_HOMED_ENCODER_ZEROED done, REFILL is where plunger at homed offset for plastic filling
    {
      doMoveMotor = true;
      programmedMotorMove(generalFastSpeed, refillOpeningOffsetDistSteps /*,defaultAcceletationNema*/);
    }
    else if (toState == COMPRESSION)
    {
      // doMoveMotor = true;
      // compressionFunction();  // FIXME  compression function currently commented out
    }
    else if (toState == INJECT)
    {
      doMoveMotor = true;
      programmedMotorMove(fillMouldMoveSpeed, fillMouldMoveDistSteps, fillMouldAccel);
    }
    else if (toState == HOLD_INJECTION)
    {
      doMoveMotor = true;
      programmedMotorMove(holdMouldMoveSpeed, holdMouldMoveDistSteps /*, fillMouldAccel*/); // leave as default or as same as fillMouldAccel?
    }
    else if (toState == RELEASE)
    {
      doMoveMotor = true;
      programmedMotorMove(releaseMouldMoveSpeed, releaseMouldMoveDistSteps); // common machine action, uses default accel
    }
    else
    {
      switch (currentState)
      {

      case INIT_HOMED_ENCODER_ZEROED:

        break;
      }
    }

    currentState = toState;
  }
}

void machineState() //
{
  // static StatesMachine runState;  // useful here, or to make runState a Static variable between loops
  //  maybe can add "static" to above enum declaration...?

  /** button activation/ availability is activted on entering new state..? if motor move is started during transtion, STOPPING move by user
   * can only be done once in new State..?
   */

  switch (currentState)
  {
  case InjectorStates::ERROR_STATE: // function that includes stopMove(), flashes LEDs red (maybe with
    // Serial.println("ERROR!! " + error);

    buttonLEDsColors(RED_RGB, RED_RGB, RED_RGB); // FIXME ERROR_STATE should be all red, with flashing sequence as per Error to Identify

    if (fsm_inputs.selectButtonPressed) // FIXME ERROR_STATE this is only for testing, SHOULD NOT EXIST IN REAL SKETCH
    {
      transitionToState(InjectorStates::INIT_HEATING);
    }
    break;

  case InjectorStates::INIT_HEATING: //  FIXME INIT_HEATING this is only for testing, SHOULD NOT EXIST IN REAL SKETCH
    buttonLEDsColors(RED_RGB, RED_RGB, RED_RGB);

    if (fsm_inputs.upButtonPressed)
    {
      transitionToState(InjectorStates::INIT_HOT_NOT_HOMED);
    }

    if (fsm_inputs.nozzleTemperature > minTempForAnyMove)
    {
      transitionToState(InjectorStates::INIT_HOT_NOT_HOMED);
    }

    break;

  case InjectorStates::INIT_HOT_NOT_HOMED:
    buttonLEDsColors(YELLOW_RGB, YELLOW_RGB, YELLOW_RGB);

    if (fsm_inputs.upButtonPressed)
    {
      transitionToState(InjectorStates::INIT_HOMED_ENCODER_ZEROED);
    }
    if (fsm_inputs.downButtonPressed) //  FIXME INIT_HOT_NOT_HOME downButtonPressed this is only for testing, SHOULD NOT EXIST IN REAL SKETCH
    {
      transitionToState(InjectorStates::INIT_HEATING);
    }
    break;

  case InjectorStates::INIT_HOMED_ENCODER_ZEROED:      // FIXME should be only the reset of the encoder, no button presses exist in real sketch
    buttonLEDsColors(YELLOW_RGB, RED_RGB, YELLOW_RGB); //  FIXME YELLOW_RGB, YELLOW_RGB, YELLOW_RGB

    if (fsm_inputs.downButtonPressed)
    {
      transitionToState(InjectorStates::REFILL);
    }
    if (fsm_inputs.upButtonPressed)
    {
      transitionToState(InjectorStates::INIT_HOT_NOT_HOMED);
    }
    break;

  case InjectorStates::REFILL:
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
      transitionToState(InjectorStates::COMPRESSION);
    }

    if (fsm_inputs.upButtonPressed && fsm_inputs.downButtonPressed)
    {
      endOfDayFlag = !endOfDayFlag;
    }

    break;

  case InjectorStates::COMPRESSION:
    buttonLEDsColors(RED_RGB, BLACK_RGB, RED_RGB);

    if (fsm_inputs.selectButtonPressed)
    {
      transitionToState(InjectorStates::REFILL);
    }
    break;

  case InjectorStates::READY_TO_INJECT:
    buttonLEDsColors(GREEN_RGB, YELLOW_RGB, GREEN_RGB);

    if (fsm_inputs.selectButtonPressed && fsm_inputs.downButtonPressed)
    {
      transitionToState(InjectorStates::PURGE_ZERO);
    }
    else if (fsm_inputs.upButtonPressed)
    {
      transitionToState(InjectorStates::REFILL);
    }

    break;

  case InjectorStates::PURGE_ZERO:
    buttonLEDsColors(GREEN_RGB, YELLOW_RGB, YELLOW_RGB);
    if (fsm_inputs.selectButtonPressed)
    {
      stepper->setCurrentPosition(0);
      transitionToState(InjectorStates::ANTIDRIP);
    }

    if (fsm_inputs.upButtonPressed)
    {
      doMoveMotor = true;
      continuousMotorMoveUp(purgeSpeed);
    }

    if (fsm_inputs.downButtonPressed)
    {
      doMoveMotor = true;
      continuousMotorMoveDown(purgeSpeed);
    }

    break;

  case InjectorStates::ANTIDRIP:
    buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB);

    if (fsm_inputs.selectButtonPressed)
    {
      stepper->stopMove();
      transitionToState(READY_TO_INJECT);
    }

    if (fsm_inputs.upButtonPressed && fsm_inputs.downButtonPressed)
    {
      transitionToState(InjectorStates::INJECT);
    }

    break;

  case InjectorStates::INJECT:
    buttonLEDsColors(RED_RGB, GREEN_RGB, BLACK_RGB); // middle/UP LED GREEN flashing

    if (fsm_inputs.selectButtonPressed)
    {
      stepper->stopMove();
      Serial.println("Motor moved " && stepper->getCurrentPosition() && " steps"); // send via serial the actual steps moved by motor: in the case mould was overfilling, can be useful info for user
      //
      transitionToState(InjectorStates::RELEASE);
    }

    break;

  case InjectorStates::HOLD_INJECTION:
    buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB); // bottom/DOWN LED GREEN flashing

    if (fsm_inputs.selectButtonPressed)
    {
      stepper->stopMove();
      transitionToState(InjectorStates::RELEASE);
    }

    break;

  case InjectorStates::RELEASE:
    buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);
    break;

  case InjectorStates::CONFIRM_MOULD_REMOVAL:
    buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);

    if (endOfDayFlag == 0)
    {
      if (fsm_inputs.selectButtonPressed || fsm_inputs.upButtonPressed || fsm_inputs.downButtonPressed)
      {
        transitionToState(InjectorStates::REFILL);
      }
    }

    if (endOfDayFlag == 1)
    {
      if (fsm_inputs.selectButtonPressed || fsm_inputs.upButtonPressed || fsm_inputs.downButtonPressed)
      {
        transitionToState(InjectorStates::READY_TO_INJECT);
      }
    }
    break;
  }
}


void stateMachineLoop() {
  error = sanityCheck();
  if (error != InjectorError::NO_ERROR) {
      transitionToState(InjectorStates::ERROR_STATE);

      Serial.printf("Found error!. Changing to ERROR_STATE to INIT_HEATING with error %d\n", error);
  } 
  machineState();
}
