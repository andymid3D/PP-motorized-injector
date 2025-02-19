/*   
stepper_2but_for_ESP32_fastaccel.ino
 */
#define dirPinStepper    22
//#define enablePinStepper 6
#define stepPinStepper   23  // OC1A in case of AVR
#define downPin 27 
#define upPin 26


int runSpeed;
int runMove;
int runState=0;
const int maxSpeedLimit = 10000;
const int moveDist=5000;
int downPinState;
int upPinState;
int potReading =500;
int motorSpeed;

#include <FastAccelStepper.h>
#include <elapsedMillis.h>

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

elapsedMillis printTime;

void setup() 
{
  Serial.begin(115200);
  pinMode(downPin, INPUT_PULLUP);
  pinMode(upPin, INPUT_PULLUP);


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
  if (/*(runState!=runState) ||*/ (printTime >= 200))
   {
    Serial.print("F");
    Serial.print(" ");
    Serial.print(downPinState);
    Serial.print(" ");
    Serial.print("B");
    Serial.print(" ");
    Serial.print(upPinState);
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
  downPinState = digitalRead(downPin);
  upPinState = digitalRead(upPin);

  if ((downPinState == 0 && upPinState == 0) || (downPinState == 1 && upPinState == 1)) 
  {
    runState = 0;
    executeMotorCommand(runState);
  }
  else if (downPinState == 0 && upPinState == 1)
  {
    runState = 1;
    executeMotorCommand(runState);
  }
  else if (downPinState == 1 && upPinState == 0)
  {
    runState = -1;
    executeMotorCommand(runState);
  }

  motorSpeed =  map(potReading, 0, 1023, 5, maxSpeedLimit);
  runSpeed = (motorSpeed); 
}