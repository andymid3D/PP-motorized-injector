
/* program to control motorized injection machine via ESP32 for direct Nema34 160mm with external DM860H
driver, 3 inductive endstops, 3 lower keypad buttons with Neopixel LEDs behind them to indicate distinct
funtionality, SPI thermocouple, & independant Encoder to Motor comparison, all via a State Machine to 
progress thru different machine states, using either sensor or user input via buttons

sketch should return via Serial data on Encoder and Temp, later this ESP32 will be connected via UART to 
a ESP32 Display, which will hold different mould structs, and therefore be able to load to the motor ESP32
distinct injection parameters as per mould to inject, and also keep track from Encoder data of when the 
injector is Refilled, so each Refill will be graphically represented by a rectangule inside a tube like 
graphic, move down and change color as injection and refills happens, & have a timing tracker to be able 
to see if it has had enough time to melt (avoiding cold injections!) */

#include <Arduino.h>
#include <SPI.h>

#include <FastAccelStepper.h>
#include <Adafruit_MAX31855.h> // temp board, was enclosed with quotes "" in example, not sure why
#include <Adafruit_NeoPixel.h> // ws2812B RGB addressable LEDs
#include <Bounce2.h>           // bounce library allows to time button presses
#include <ESP32Encoder.h>

#include "config.h"
#include "injector_fsm.h"


////////////////////////////////
// Stepper
////////////////////////////////
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

////////////////////////////////
// Encoder
////////////////////////////////
ESP32Encoder encoder;

Adafruit_MAX31855 thermocouple(TEMPNozzleVSPI_SCK_CLK, TEMPNozzleVSPI_Dpin_MOSI_CS, TEMPNozzleVSPI_MISO_DO);

Adafruit_NeoPixel keypadleds = Adafruit_NeoPixel(keypadLedCount, WS2812B_BUTTON_LEDS_PIN);
Adafruit_NeoPixel ringleds = Adafruit_NeoPixel(ringLedCount, WS2812B_RING_LEDS_PIN);

////////////////////////////////
// Input block
////////////////////////////////

Bounce2::Button downButton = Bounce2::Button();
Bounce2::Button upButton = Bounce2::Button();
Bounce2::Button selectButton = Bounce2::Button();
Bounce2::Button topEndstop = Bounce2::Button();
Bounce2::Button bottomEndstop = Bounce2::Button();
Bounce2::Button barrelEndstop = Bounce2::Button();
Bounce2::Button EMERGENCYstop = Bounce2::Button();

boolean readNozzleTemp = true;
boolean readEmergencyStop = true;
boolean readSelectButton = true;
boolean readUpButton = true;
boolean readDownButton = true;
boolean readTopEndStop = true;
boolean readBottomEndStop = true;
boolean readBarrelEndStop = true;

/**
 * 
 */
void bounceButtonsSetup() {
  // IF YOUR BUTTON HAS AN INTERNAL PULL-UP or PULL-DOWN
  downButton.attach(BUTTON_DOWN_PIN, INPUT_PULLUP);
  upButton.attach(BUTTON_UP_PIN, INPUT_PULLUP);  // USE INTERNAL PULL-UP
  selectButton.attach(BUTTON_SELECT_PIN, INPUT_PULLUP);
  topEndstop.attach(ENDSTOP_TOP_PLUNGER_PIN, INPUT_PULLUP);
  bottomEndstop.attach(ENDSTOP_BOTTOM_PLUNGER_PIN, INPUT_PULLUP);    //Top & Bottom NO, 5v
  barrelEndstop.attach(ENDSTOP_BARREL_PLUNGER_PIN, INPUT_PULLDOWN);  //FINDA sensor is NC, 0v
  EMERGENCYstop.attach(EMERGENCY_STOP_PIN, INPUT_PULLUP);    // CHECK THAT IS NO!!

  // DEBOUNCE INTERVAL IN MILLISECONDS
  downButton.interval(debounceInterval);
  upButton.interval(debounceInterval);
  selectButton.interval(debounceInterval);
  topEndstop.interval(debounceInterval);
  bottomEndstop.interval(debounceInterval);
  barrelEndstop.interval(debounceInterval);
  EMERGENCYstop.interval(debounceInterval);

  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  downButton.setPressedState(LOW);
  upButton.setPressedState(LOW);
  selectButton.setPressedState(LOW);
  topEndstop.setPressedState(LOW);  // NO, with pullup, goes LOW when closed
  bottomEndstop.setPressedState(LOW);
  barrelEndstop.setPressedState(HIGH);  //  NC, with pulldown, goes HIGH when active
  EMERGENCYstop.setPressedState(LOW);   //  CHECK THAT IS NO!!
}

////////////////////////////////
// END Input block
////////////////////////////////
void clearLEDs()
{
  for (int i = 0; i < keypadLedCount; i++)
  {
    keypadleds.setPixelColor(i, 0);
  }
}



/**
 *
 */
void outputButtonLEDsColors() // could use case/switch instead? but are different types of defining behaviours..?
{
  if (fsm_inputs.selectButtonPressed || fsm_inputs.upButtonPressed || fsm_inputs.downButtonPressed) // set brightness to about 80% on any button press
  {
    keypadleds.setBrightness(ledHBrightness);
  }
  else
  {
    keypadleds.setBrightness(ledLBrightness);
  }

  keypadleds.setPixelColor(0, fsm_outputs.currentSelectLEDcolour);
  keypadleds.setPixelColor(1, fsm_outputs.currentUpLEDcolour);
  keypadleds.setPixelColor(2, fsm_outputs.currentDownLEDcolour);

  keypadleds.show();

  return;
}

/**
 * 
 */
void getInputs() {
  if (readNozzleTemp) {
    fsm_inputs.nozzleTemperature = thermocouple.readCelsius();
  } 

  if (readEmergencyStop) {
    fsm_inputs.emergencyStop = EMERGENCYstop.pressed();
  }

  if (readSelectButton) {
    fsm_inputs.selectButtonPressed = selectButton.pressed();
  }

  if (readUpButton) {
    fsm_inputs.upButtonPressed = upButton.pressed();
  }

  if (readDownButton) {
    fsm_inputs.downButtonPressed = downButton.pressed();
  }

  if (readTopEndStop) {
    fsm_inputs.topEndStopActivated = topEndstop.isPressed();
  }

  if (readBottomEndStop) {
    fsm_inputs.bottomEndStopActivated = bottomEndstop.isPressed();
  }

  if (readBarrelEndStop) {
    fsm_inputs.barrelEndStopActivated = barrelEndstop.isPressed();
  }

  fsm_inputs.actualENPosition = encoder.getCount() / 2;
}

void setOutputs() {
  outputButtonLEDsColors();
}


void setup() {
    Serial.begin(9600);

    delay(2000); // wait 2 seconds. REQUIERED !!!
    Serial.println("PP_injector_ESP32");

    // encoder setup

// FIXME commented out for testing
    //encoder.attachHalfQuad(ENCODER_A_PIN, ENCODER_B_PIN);  // possible have to reverse..? or rename, for Encoder library "CLK, DT ", and also rename EncoderPins..?
    // encoder.setCount ( 0 );  move this line to INIT_HOMED_ENCODER_ZEROED function?

    // stepper setup
    //#define DRIVER_RMT 1  // type of driver for FAS, to separate any motor interference, not needed as directly used "1" below..?
    engine.init();                                           
    
    // FIXME  Core assignment
    // engine.init(1); 
    // stepper = engine.stepperConnectToPin(STEPPER_STEP_PIN, 1);  // "1" defines DRIVER_RMT, for encoder to use PCNT, not Stepper
    // refere to https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md
    // for ways to CPU_CORE & DRIVER_TYPE, if this does not compile as-is, and 
    // https://valarsystems.com/blogs/val-2000/section-9-dual-core-setup for alternative CPU_CORE method

    stepper = engine.stepperConnectToPin(STEPPER_STEP_PIN);
    stepper->setAbsoluteSpeedLimit(8000);  // 16.000.000 tick/s / 2000steps/s maxSpeedLimit = 8000 ticks/ step
                                                           // 16.000.000 tick/s / 5000steps/s maxSpeedLimit = 3200 ticks/ step

    if (stepper) {
      stepper->setDirectionPin(STEPPER_DIR_PIN);  // possible not needed if moveTo commands are also -ve..?
    }

    // thermocouple setup
    thermocouple.begin();

    // keypadLEDs setup
    keypadleds.begin();  // Call this to start up the LED strip.
    //  clearLEDs();   // This function, defined below, turns all LEDs off...
    keypadleds.show();  // ...but the LEDs don't actually update until you call this.

    // Buttons & Endstops Bounce2 setup is very long, so called from the following function
    bounceButtonsSetup();

    fsm_state.currentState = InjectorStates::INIT_HEATING;
    fsm_state.error = InjectorError::NO_ERROR;

    readNozzleTemp = true;
    readEmergencyStop = true;
}

long now; 
long fastTaskTime = 0;
long mediumTaskTime = 0;
long slowTaskTime = 2;


/** 
 * Comms functions, every 100ms send data via serial
 */
void serialRegular100msMessages() {
  Serial.println("===============");
  //Serial.printf("loops: %d, avg time: %d\n", numLoops, avgLoopTime);
  Serial.printf("state: %d, ER: %d\n",fsm_state.currentState, fsm_state.error);
  Serial.println("selectLed: " + String(fsm_outputs.currentSelectLEDcolour, HEX));
  Serial.println("upLed: " + String(fsm_outputs.currentUpLEDcolour, HEX));
  Serial.println("downLed: " + String(fsm_outputs.currentDownLEDcolour, HEX));

  Serial.printf("temps: NZ=%d\n", fsm_inputs.nozzleTemperature);

  Serial.printf("inputs: ES=%d, SEL=%d, UP=%d, DW=%d, TE=%d, BE=%d, BR=%d\n", fsm_inputs.emergencyStop, fsm_inputs.selectButtonPressed, fsm_inputs.upButtonPressed, fsm_inputs.downButtonPressed, fsm_inputs.topEndStopActivated, fsm_inputs.bottomEndStopActivated, fsm_inputs.barrelEndStopActivated);

// // FIXME commented out for testing
//       //actualENPosition = encoder.getCount() / 2;
//       if (actualENPosition != oldENPosition) /* ... and actualENPosition & oldENPosition are updated by EncoderActualPosition, but if there
//      there has been no change in last 100ms, then do not print new position... possibly this should be divided in loop..? */
//       {
//         Serial.print("Encoder: ");
//         Serial.println(actualENPosition);
//         oldENPosition = actualENPosition;
//       }
//       Serial.print("TempBox: ");
//       Serial.println(thermocouple.readInternal());
//       Serial.print("TempNozzle: ");
//       Serial.println(nozzleTemperature);

}

void loop() {
  // this eventloop runs tasks based on 3 different rates: high speed tasks running every loop interation, fast tasks running every 1ms, medium tasks at 10ms, and slow tasks at 100ms
  
  // high speed tasks
  downButton.update();
  upButton.update();
  selectButton.update();
  topEndstop.update();
  bottomEndstop.update();
  barrelEndstop.update();
  EMERGENCYstop.update();
  

  now = millis();
  if (now - fastTaskTime  >= 1) {
    fastTaskTime = now;
    // fast tasks
  
  }
  if (now - mediumTaskTime >= 10) {
    mediumTaskTime = now;
    // medium tasks
    getInputs();
    stateMachineLoop();
    setOutputs();

  }
  if (now - slowTaskTime >= 100) {
    slowTaskTime = now;
    // slow tasks
    serialRegular100msMessages();
  }

}

