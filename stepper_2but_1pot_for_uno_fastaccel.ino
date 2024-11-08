/*   
 *   Basic example code for controlling a stepper with the AccelStepper library
 *      
 *   by Dejan, https://howtomechatronics.com
 */
// As in StepperDemo for Motor 1 on AVR
#define dirPinStepper    5
//#define enablePinStepper 6
#define stepPinStepper   9  // OC1A in case of AVR
#define forwardPin 7
#define backwardPin 6
#define pot A0
int runSpeed;
int runMove;
int runState=0;
const int maxSpeedLimit = 10000;
const int moveDist=5000;
int forwardPinState;
int backwardPinState;
int potReading;
int motorSpeed;

#include <FastAccelStepper.h>
#include <elapsedMillis.h>

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

elapsedMillis printTime;

void setup() 
{
  Serial.begin(115200);
  pinMode(forwardPin, INPUT_PULLUP);
  pinMode(backwardPin, INPUT_PULLUP);
  pinMode(pot, INPUT);

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) 
  {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setSpeedInHz(runSpeed);  // the parameter is Hz, steps/s !!!
    stepper->setAcceleration(50000);
    stepper->move(moveDist);
  }
}

void executeMotorCommand(int8_t runState)
{
  switch (runState)
  {
    case 0:
//	  stepper1.setAcceleration(200);
      stepper->stopMove();
	    motorSpeed = 0;
	    stepper->setCurrentPosition(0);
      break;
    case 1:
	    stepper->setSpeedInHz(runSpeed);
	    stepper->runForward();
      break;
    case -1:
	    stepper->setSpeedInHz(runSpeed);
	    stepper->runBackward();
      break;
  }
}


void loop() 
{
  if ((runState!=runState) || (printTime >= 1000))
   {
    Serial.print("F");
    Serial.print(" ");
    Serial.print(forwardPinState);
    Serial.print(" ");
    Serial.print("B");
    Serial.print(" ");
    Serial.print(backwardPinState);
    Serial.print(" ");
    Serial.print("rS");
    Serial.print(" ");
    Serial.print(runState);
    Serial.print(" ");
    Serial.print("S");
    Serial.print(" ");
    Serial.println(runSpeed);
   /* Serial.print(" ");
    Serial.print("D");
    Serial.print(" ");
    Serial.println(stepper1.distanceToGo());*/
    printTime=0;
   }
  forwardPinState = digitalRead(forwardPin);
  backwardPinState = digitalRead(backwardPin);

  if ((forwardPinState == 0 && backwardPinState == 0) || (forwardPinState == 1 && backwardPinState == 1)) 
  {
    runState = 0;
    executeMotorCommand(runState);
  }
  else if (forwardPinState == 0 && backwardPinState == 1)
  {
    runState = 1;
    executeMotorCommand(runState);
  }
  else if (forwardPinState == 1 && backwardPinState == 0)
  {
    runState = -1;
    executeMotorCommand(runState);
  }
  
  potReading = analogRead(pot);
  motorSpeed =  map(potReading, 0, 1023, 5, maxSpeedLimit);
  runSpeed = (motorSpeed); 
}