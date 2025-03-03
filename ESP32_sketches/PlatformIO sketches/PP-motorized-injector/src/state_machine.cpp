#include "state_machine.h"

// Global variables
InjectorStates currentState = InjectorStates::INIT_HEATING;
uint16_t error = InjectorError::NO_ERROR;

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
            homeMove(homingFastSpeed, homingSlowSpeed);
        }
        else if (toState == REFILL)
        {
            doMoveMotor = true;
            homeMove(homingFastSpeed, homingSlowSpeed);
            programmedMotorMove(generalFastSpeed, refillOpeningOffsetDistSteps);
        }
        else if (toState == COMPRESSION)
        {
            doMoveMotor = true;
            compressionFunction(initialCompressionSpeed, minCompressionSpeed);
        }
        else if (toState == INJECT)
        {
            doMoveMotor = true;
            programmedMotorMove(fillMouldMoveSpeed, fillMouldMoveDistSteps, fillMouldAccel);
        }
        else if (toState == HOLD_INJECTION)
        {
            doMoveMotor = true;
            programmedMotorMove(holdMouldMoveSpeed, holdMouldMoveDistSteps);
        }
        else if (toState == RELEASE)
        {
            doMoveMotor = true;
            programmedMotorMove(releaseMouldMoveSpeed, releaseMouldMoveDistSteps);
        }

        currentState = toState;
    }
}

void machineState()
{
    switch (currentState)
    {
        case ERROR_STATE:
            buttonLEDsColors(RED_RGB, RED_RGB, RED_RGB);
            break;
        case INIT_HEATING:
            buttonLEDsColors(RED_RGB, RED_RGB, RED_RGB);
            if (nozzleTemperature > minTempForAnyMove)
            {
                currentState = INIT_HOT_NOT_HOMED;
            }
            break;
        case INIT_HOT_NOT_HOMED:
            buttonLEDsColors(YELLOW_RGB, YELLOW_RGB, YELLOW_RGB);
            if (upButtonPressed)
            {
                currentState = INIT_HOMED_ENCODER_ZEROED;
            }
            break;
        case INIT_HOMED_ENCODER_ZEROED:
            buttonLEDsColors(YELLOW_RGB, YELLOW_RGB, YELLOW_RGB);
            encoder.setCount(0);
            currentState = REFILL;
            break;
        case REFILL:
            if (endOfDayFlag == 0)
            {
                buttonLEDsColors(GREEN_RGB, BLACK_RGB, BLACK_RGB);
            }
            else if (endOfDayFlag == 1)
            {
                buttonLEDsColors(GREEN_RGB, BLUE_RGB, BLUE_RGB);
            }
            if (selectButtonPressed)
            {
                currentState = COMPRESSION;
            }
            if (upButtonPressed && downButtonPressed)
            {
                endOfDayFlag = !endOfDayFlag;
            }
            break;
        case COMPRESSION:
            buttonLEDsColors(RED_RGB, BLACK_RGB, RED_RGB);
            break;
        case READY_TO_INJECT:
            buttonLEDsColors(GREEN_RGB, YELLOW_RGB, GREEN_RGB);
            if (selectButtonPressed && downButtonPressed)
            {
                currentState = PURGE_ZERO;
            }
            else if (upButtonPressed)
            {
                currentState = REFILL;
            }
            break;
        case PURGE_ZERO:
            buttonLEDsColors(GREEN_RGB, YELLOW_RGB, YELLOW_RGB);
            if (selectButtonPressed)
            {
                stepper->setCurrentPosition(0);
                currentState = ANTIDRIP;
            }
            if (upButtonPressed)
            {
                doMoveMotor = true;
                continuousMotorMoveUp(purgeSpeed);
            }
            if (downButtonPressed)
            {
                doMoveMotor = true;
                continuousMotorMoveDown(purgeSpeed);
            }
            break;
        case ANTIDRIP:
            buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB);
            if (selectButtonPressed)
            {
                stepper->stopMove();
                currentState = READY_TO_INJECT;
            }
            if (upButtonPressed && downButtonPressed)
            {
                currentState = INJECT;
            }
            break;
        case INJECT:
            buttonLEDsColors(RED_RGB, GREEN_RGB, BLACK_RGB);
            if (selectButtonPressed)
            {
                stepper->stopMove();
                Serial.println("Motor moved " && stepper->getCurrentPosition() && " steps");
                currentState = RELEASE;
            }
            else
            {
                currentState = HOLD_INJECTION;
            }
            break;
        case HOLD_INJECTION:
            buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB);
            if (selectButtonPressed)
            {
                stepper->stopMove();
                currentState = RELEASE;
            }
            else
            {
                currentState = RELEASE;
            }
            break;
        case RELEASE:
            buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);
            currentState = CONFIRM_MOULD_REMOVAL;
            break;
        case CONFIRM_MOULD_REMOVAL:
            buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);
            if (endOfDayFlag == 0)
            {
                if (selectButtonPressed || upButtonPressed || downButtonPressed)
                {
                    currentState = REFILL;
                }
            }
            if (endOfDayFlag == 1)
            {
                if (selectButtonPressed || upButtonPressed || downButtonPressed)
                {
                    currentState = READY_TO_INJECT;
                }
            }
            break;
    }
}

int sanityCheck()
{
    int error = InjectorError::NO_ERROR;
    if (nozzleTemperature <= minTempForAnyMove)
    {
        error |= InjectorError::ERROR_1;
    }
    if (barrelEndStopActivated)
    {
        error |= InjectorError::ERROR_2;
    }
    if (topEndStopActivated)
    {
        error |= InjectorError::ERROR_3;
    }
    if (bottomEndStopActivated)
    {
        error |= InjectorError::ERROR_4;
    }
    if (emergencyStop)
    {
        error |= InjectorError::ERROR_5;
    }
    return error;
}