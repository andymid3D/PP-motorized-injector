// injector_ESP32_state_machine.ino

/* sketch to control motorized injection machine via ESP32 for direct Nema34 160mm with external DM860H
driver, 3 inductive endstops, 3 lower keypad buttons with Neopixel LEDs behind them to indicate distinct
funtionality, SPI thermocouple, & independant Encoder to Motor comparison, all via a State Machine to
progress thru different machine states, using either sensor or user input via buttons

sketch should return via Serial data on Encoder and Temp, later this ESP32 will be connected via UART to
a ESP32 Display, which will hold different mould structs, and therefore be able to load to the motor ESP32
distinct injection parameters as per mould to inject, and also keep track from Encoder data of when the
injector is Refilled, so each Refill will be graphically represented by a rectangule inside a tube like
graphic, move down and change color as injection and refills happens, & have a timing tracker to be able
to see if it has had enough time to melt (avoiding cold injections!) */

/* global varible definitions & declaractions, some may be moved to local funtions, but
leave here and comment out where they have gone - QUESTION: can LOCAL variables within functions be changed via
serial message updates..? ex: compressionFunction has local speedReductionFactor & comparisonPercentage, during
initial sketch & machine calibration, it may be necessary to update these values, would be easier via Serial
than flashing...? Although in theory, once checked and sure cannot cause infinite loop, should again be purely
local and never need changing
 */

#include <Arduino.h>
#include "state_machine.h"
#include "motor_control.h"
#include "button_control.h"
#include "serial_comms.h"
#include "config.h"

// Global variables
// ESP32Encoder encoder;
// int nozzleTemperature;

bool SelectPinState;
int ledLBrightness = 50;
int ledHBrightness = 200;
const int keypadLedCount = 3;
const int ringLedCount = 35;
const int debounce_Interval = 5;

boolean readNozzleTemp = true;
int nozzleTemperature;
boolean readEmergencyStop = true;
boolean emergencyStop;
boolean readSelectButton = true;
boolean readUpButton = true;
boolean readDownButton = true;
boolean readTopEndStop = true;
boolean readBottomEndStop = true;
boolean readBarrelEndStop = true;
int actualENPosition;

//////////////////

// global common constants
// revise what is now NOT NEEDED, OR what variables should go inside Functions to make LOCAL

/* global common variables for machine limits and moves to offsets, these, once identified the max values
should NEVER need changing, even from advanced Common panel, as superior values run the risk of machine
performing outside of the decided security limits of movement/accel, distances, etc
revise what is  NOT NEEDED, OR what variables should go inside Functions to make LOCAL*/
const int minTempForAnyMove = 20;              // FIXME              // min temp considered safe for machine move
const int maxSpeedLimit = 2000;                // FIXME, 5000 IRL, 2000 corresponds to 5rps NEMA, about 1/4 rps gear, about 18.85/4 = 4.7125cm linear
                                               // plunger movement, about 25cm3 injected volume, per second MAX speed limited
const int moveContinuosDistSteps = 5000;       // arbitrary large distance for continuous movements, gets reset to 0 each loop anyway
const int defaultAcceletationNema = 50000;     /* check in reality, maybe large moulds will benefit from slower acceleration together with higher speeds..?
                                           to avoid blocking motor/losing steps with high speed requests with too high accelerrations..
                                          or smaller moulds benefit also from slower acceleration/speed so as not to explode, or fill with
                                          less acceleration tapering off... */
const int maxHomingStepsDistSteps = -30000;    // CHECK THIS NUMBER OF STEPS COVERS WHOLE MOVEMENT RANGE, HEATED ZONE + REFILL + TOPENDSTOP + extra
const int totalStrokeStepsDistSteps = 21220;   // calc'd value, check against real, this is ONLY HEATED ZONE, need another OFFSET from Encoder 0
const int refillOpeningOffsetDistSteps = 3800; // ESTIMATED! once homed to top endstop, how far to move to position just above filling hole in barrel
                                               // THIS VALUE HAS TO BE FOUND BY CALIBRATION AND ADJUSTED TO REALITY!!
const int heatedZoneOffsetDistSteps = 7200;    // calc'd value, check with reality
const int buttonShortPress = 500;              // for short press of button, ms
const int buttonLongPress = 2000;              // long press...
const int debounceInterval = 5;

////////////////////

boolean doMoveMotor = false;
// for motor Funtion, to be re-written depenging on State to these variables
int currentMotorSpeed; // example, for INJECT state, motorSpeed=fillMouldMoveSpeed, then call ProgrammedMotorMove
                       // should use this updated value for this next call of ProgrammedMotorMove
int motorSpeed;

int currentAcceleration;  // same as above, requires each State that contemplates using ProgrammedMotorMove
                          // to have all three values, otherwise risk of using older value from previous call!
int currentMotorDistance; // same as above, is a RELATIVE move (not moveTo!)
int MotorDir;             // direction of ANY motor move, or can this be omitted as DistSteps variables can be -ve..?
int MotorPosition;        /* reference position for next ProgrammedMotorMove, set to 0 after Purge is finished... possibly
                          not needed as all moves should be RELATIVE, and resetting motor position does not need a variable */
int stepsMoved = 0;       // Track the number of steps moved, to compare to steps made, and also for encoder comparison
/* steps to cm3 & cml conversion
volumen    80steps = 1cm3,  2000steps = 25cm3  (average 2D mould approx)
linear    424steps = 1cml, 21220steps = 50cml  (total useful heating zone, below Refill offset)
*/
int stepsToCM3 = 80;
int stepsToCML = 424;

/* global common variables, different types of movement parameter storage, to be sent from ESP32 or stored
locally as common constants between all injection/homing/compression/ etc processes..
during initial calibration of machine, will refine these values, probably from advanced access
panel on display, that will later be fixed (or advanced panel will be hidden from user until
certain combo of button presses will reveal, but these common values should not need changing
once initial machien calibration has found ideal values)
FIND IDEAL VALUES OF THESE COMMON PARAMETERS AND INITIALIZE HERE!! Could become constants, but should be changable
under certain (testing & calibration?) purposes
*/

// int readyState=0  // states for non-moving ready states  - N/A
long int purgeSpeed = 80;                  // speed for purge move, continous move until button release  80= 1cm3/s
long int generalFastSpeed = maxSpeedLimit; // how fast to home, continous move until reaching endstop, also for other general moves..? max /2..?
// homing params moved to Homng Function as not needed elsewhere
// int HomingDirection;      // maybe not needed, as maxHomingStepsDistSteps can be -ve..?
// int HomeOffSetDistSteps;       // once first reached HomeEndstop, how much to back off before slower approach
// int HomeOffSetAccel;      // once first reached HomeEndstop, how much Accel to back off before slower approach
// int HomeOffsetSpeed;      // once first reached HomeEndstop, how quic to back off before slower approach
long int homingFastSpeed = generalFastSpeed / 2;
long int homingSlowSpeed = maxSpeedLimit / 10;                             // how slow to home, 2nd approach, continous move until reaching endstop
long int initialCompressionSpeed = generalFastSpeed / 2;                   // could be generalFastSpeed / 2..?
long int minCompressionSpeed = generalFastSpeed / 20;                      // at what speed is compression no longer useful..?
long int comparisonPercentage = 50;                                        /* EXAMPLE VALUE what % difference between sent motor steps and read encoder steps should a change
                           in speed be commanded?
                           higher % will cuase more skipping before changing speed, lower % will cause more speed changes (o vica versa),
                           but also possibly increase probability of infinite loop, if use line comparisonPercentage *= speedReductionFactor; */
long int speedReductionFactor = 50;                                        // EXAMPLE VALUE Percentage to reduce speed when comparisonPercentage limit reached
long int antiDripReverseTime = 15000;                                      // 15s in case if using ConstMotorMove, instead of ProgrammedMotorMove, how many seconds before cancel..?
long int antiDripReverseSpeed = stepsToCML / (antiDripReverseTime / 1000); // 80steps/15, per sec.. after purge, to recuperate and avoid drip until mould is placed - TEST
long int antiDripReverseMillis;                                            //  after purge, start millis timer
// using distance: normally the user SHOULD press INJECT button BEFORE this move is completed, but if is ProgrammedMotorMove,
// can I interrupt this move before it completes..?
long int antiDripFinalOffset;                   // once AntiDrip constMoveBack is interrupted with button press for injection, store actual motor position and sum to 0?
long int releaseMouldMoveSpeed = maxSpeedLimit; // after hold, must raise the barrel a few mm to free mould for removal.. most likely will be a constant
long int releaseMouldMoveDistSteps = -200;      //  same as above, 200 steps is approx 0.47cm, -ve as ProgMotorMove is normally +ve downward
long int constantInjectionParams[] = {
    purgeSpeed,
    generalFastSpeed,
    initialCompressionSpeed,
    minCompressionSpeed,
    comparisonPercentage,
    speedReductionFactor,
    antiDripReverseTime,
    antiDripReverseSpeed,
    releaseMouldMoveSpeed,
    releaseMouldMoveDistSteps};

/* array to hold message with above constant parameters, to be assigned to and returned to ESP32..?
 ={purgeSpeed, generalFastSpeed, initialCompressionSpeed, antiDripReverseSpeed, AntiDripReverseDistSteps, releaseMouldMoveSpeed, releaseMouldMoveDistSteps}
*/

/* actual injection variables, will be adjusted each time for each different mould, user will have access
via panel on display to change these values whilst calibrating a mould, and will then be stored on Display
ESP32 in Flash as array/struct of values particular to each mould
*/

/// @brief struct to hold ActualMouldParams
char mouldName;             // unique string name for stuct of mould parameters for below variables
int fillMouldMoveSpeed;     /* speed of next move, sent from ESP32, test, is possible different for 2D and 3D moulds,
higher speeds will lead to earlier over-current errors */
int fillMouldMoveDistSteps; /* distance to move plunger to fill mould, sent from ESP32 at the start of each injection, as
per mould in use */
int fillMouldAccel;         // possibly not needed, have to discover whether small 2D/2.5D moulds benefit from slower accel on fill..?

int holdMouldMoveSpeed;     /* ... and how fast (will be a low speed, test.. this will have to translate into a timed move,
for example has to last 15s to allow 25g of plastic to cool enough, then distSteps * speed has to take 15s.
make formula that can take time requiered, distance to move, and divide to get speed OR use ContMove for Time..? */
int holdMouldMoveDistSteps; // after mould fill, apply a little more move/pressure to get good surface finish
int holdMouldAcccel;        // as this will be a very slow speed and small distance motor move, either use fillMouldAccel, or leave as default..?
struct actualMouldParams
{
  const char *mouldName;
  int fillMouldMoveSpeed;
  int fillMouldMoveDistSteps;
  int fillMouldAccel;
  int holdMouldMoveSpeed;
  int holdMouldMoveDistSteps;
  int holdMouldAcccel;
};

// FIXME commented out for testing
// struct actualMouldParams mouldNow = { "Posavasos", maxSpeedLimit, motorAcceleration, 2160, (maxSpeedLimit / 10), 80 };
/* struct to hold received message with above variable parameters... can I include in {variable name1, variable name2, etc}..?
={fillMouldMoveSpeed, fillMouldMoveDistSteps, holdMouldMoveSpeed, holdMouldMoveDistSteps}, better practice would be struct including name
of mould and list of valies ofr the variables above, which would then be written to these variables ... include check of
message (5 comma separated values with special starting char?) length */

// comms between Arduino & ESP32
char messageToDisplay;   /* message character to be sent to display, some letters mean errors, some to display text, some as first message
to second message that contain parameter arrays for variables or constants (for confirmation on screen) */
int messageArrayToESP32; // parameter array, can be either variable array or constants array, depending on previous message..?

// encoder & position data from FAS library
// long oldFA0Position = 0;         // encoder postion on motor FAS zeroing - maybe not need, revise ComparisonFunction
long actualFAPosition; // getPosition from zeroed FastAccel library, to compare to Encoder data, and to send to ESP32 for display
// unsigned long oldENPosition;     // for referencing encoder positions..
int64_t actualENPosition; // same for encoder data.. if cannot zero encoder data (so far only zeros on Arduino reset), then also make
                          // oldENPosition, and subract to actualOldENPosition on FA zeroing, before substituting old with actual, should
                          // give same value since last zero of FA

// endstop states, active or not, other checks to relay via Serial. mostly active LOW, with PULLUP (exceptions as per endstop)
bool EMERGENCYStopPinActive = 0;          // is Emergency Stop true, then cannot allow movement - consider wiring also direct to stepper motor wires
                                          // but ONLY if sure that breaking this connection will not damage driver.. otherwise, also wire to driver PSU
bool topPlungerEndstopActive = 0;         // is 1/NC, then can move up, if 0/NC, then triggered
bool bottomPlungerEndstopActive = 0;      // is 1/NC, then can move down, if 0/NC, then triggered
bool barrelClampOKEndstopActive = 0;      // is 0/NO, then can move inject, if 1/NC, then triggered, barrel has moved!!
bool mouldPresentEndstopActive = 0;       // still to see how to implement, to assure that nozzle is blocked by either mould or
bool nozzleBlockPresentEndstopActive = 0; // purge bar/cap, so can move down w/o displacing barrel
bool endOfDayFlag = 0;                    // 0 = refillAfterInject, 1= skip and, possibly, measure steps since last top endstop - could read from encoder.. as FA library
                                          // is contemplated to be reset after each purge, so if ==0, reset encoder, ==1 do not reset encoder
bool initialHomingDone = 0;               // on power on, machine needs homing, then mark this flag to show has been done at least once
int errorReason;

// lower keypad button states, could be bool?  Or just use directly digitalRead of pin in any arguments/functions..?
bool DownPinState;   // var for purge function, read from BUTTON_DOWN_PIN pin
bool UpPinState;     // var for purge function, read from BUTTON_UP_PIN pin
bool SelectPinState; // var for purge function, read from BUTTON_SELECT_PIN pin

//  LED stuff
int ledLBrightness = 50;      // initial brightness for LEDs, to possible increase to 100 during an actual press..?
int ledHBrightness = 200;     // pressed key brightness for LEDs, to increase to 200 (of 255 max) during an actual press..?
const int keypadLedCount = 3; // 3 for just keypad buttons
const int ringLedCount = 35;  // 35 for LED ring on end of barrel
#define GREEN_RGB ((uint32_t)0x008000)
#define RED_RGB ((uint32_t)0xFF0000)
#define YELLOW_RGB ((uint32_t)0xFF8C00)
#define BLUE_RGB ((uint32_t)0x0000FF)
#define BLACK_RGB ((uint32_t)0x000000)
#define WHITE_RGB ((uint32_t)0xFFFFFF)

const int debounce_Interval = 5;

// libraries to include

#include <ESP32Encoder.h>
ESP32Encoder encoder;

#include <FastAccelStepper.h> // *
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// #include <ACS712.h> // current sensor, may not be needed, but added 2 to motor phase B + & - just in case
#include <SPI.h>
#include <Adafruit_MAX31855.h> // temp board, was enclosed with quotes "" in example, not sure why
Adafruit_MAX31855 thermocouple(TEMPNozzleVSPI_SCK_CLK, TEMPNozzleVSPI_Dpin_MOSI_CS, TEMPNozzleVSPI_MISO_DO);

#include <Adafruit_NeoPixel.h> // ws2812B RGB addressable LEDs
Adafruit_NeoPixel keypadleds = Adafruit_NeoPixel(keypadLedCount, WS2812B_BUTTON_LEDS_PIN);
Adafruit_NeoPixel ringleds = Adafruit_NeoPixel(ringLedCount, WS2812B_RING_LEDS_PIN);

#include <Bounce2.h> // bounce library allows to time button presses
Bounce2::Button downButton = Bounce2::Button();
Bounce2::Button upButton = Bounce2::Button();
Bounce2::Button selectButton = Bounce2::Button();
Bounce2::Button topEndstop = Bounce2::Button();
Bounce2::Button bottomEndstop = Bounce2::Button();
Bounce2::Button barrelEndstop = Bounce2::Button();
Bounce2::Button EMERGENCYstop = Bounce2::Button();

////////////////////////////////
// Input block
////////////////////////////////

boolean readNozzleTemp = true;
int nozzleTemperature;

boolean readEmergencyStop = true;
boolean emergencyStop;

boolean readSelectButton = true;
boolean selectButtonPressed;

boolean readUpButton = true;
boolean upButtonPressed;

boolean readDownButton = true;
boolean downButtonPressed;

boolean readTopEndStop = true;
boolean topEndStopActivated;

boolean readBottomEndStop = true;
boolean bottomEndStopActivated;

boolean readBarrelEndStop = true;
boolean barrelEndStopActivated;

/**
 *
 */
void getInputs()
{
  if (readNozzleTemp)
  {
    nozzleTemperature = thermocouple.readCelsius();
  }

  if (readEmergencyStop)
  {
    emergencyStop = EMERGENCYstop.pressed();
  }

  if (readSelectButton)
  {
    selectButton.update();
    selectButtonPressed = selectButton.pressed();
  }

  if (readUpButton)
  {
    upButton.update();
    upButtonPressed = upButton.pressed();
  }

  if (readDownButton)
  {
    downButton.update();
    downButtonPressed = downButton.pressed();
  }

  if (readTopEndStop)
  {
    topEndstop.update();
    topEndStopActivated = topEndstop.isPressed();
  }

  if (readBottomEndStop)
  {
    bottomEndstop.update();
    bottomEndStopActivated = bottomEndstop.isPressed();
  }

  if (readBarrelEndStop)
  {
    barrelEndstop.update();
    barrelEndStopActivated = barrelEndstop.isPressed();
  }

  actualENPosition = encoder.getCount() / 2;
}

////////////////////////////////
// END Input block
////////////////////////////////

////////////////////////////////
// State machine
////////////////////////////////

enum InjectorError : uint16_t
{
  NO_ERROR = 1 << 0,
  ERROR_1 = 1 << 1, // temp below min
  ERROR_2 = 1 << 2, // barrel endstop activated
  ERROR_3 = 1 << 3, // top endstop activaded outside of function
  ERROR_4 = 1 << 4, // bottom endstop (activaded outside of function)
  ERROR_5 = 1 << 5, // emergency stop pressed
  ERROR_6 = 1 << 6, // barrel block NOT active in function that requieres
};

// StatesMachine, states that the machine may be in at any point in time

enum InjectorStates : int
{
  ERROR_STATE = 0,               // SOME TYPE OF CHECK TRIGGERED
  INIT_HEATING,              // INITIAL POWER ON STATES
  INIT_HOT_NOT_HOMED,        // INITIAL POWER ON STATES
  INIT_HOMED_ENCODER_ZEROED, // INITIAL POWER ON STATES
  REFILL,                        // DEFAULT WAITING STATES
  COMPRESSION,                   // DEFAULT WAITING STATES
  READY_TO_INJECT,               // DEFAULT WAITING STATES
  PURGE_ZERO,                    // INJECT PROCESS STATES, normally will start here...
  ANTIDRIP,                      // INJECT PROCESS STATES,
  INJECT,                        // INJECT PROCESS STATES,
  HOLD_INJECTION,                // INJECT PROCESS STATES,
  RELEASE,                       // INJECT PROCESS STATES,
  CONFIRM_MOULD_REMOVAL          // INJECT PROCESS STATES, ... and end here before returning
                                 // to Refill or READY_TO_INJECT, depending on EndOfDayFlag
};

InjectorStates currentState = InjectorStates::INIT_HEATING; // declaring variable runState can only have valid values of enum
uint16_t error = InjectorError::NO_ERROR;

////////////////////////////////
// END State machine
////////////////////////////////

/**
 *
 */
void bounceButtonsSetup()
{
  // IF YOUR BUTTON HAS AN INTERNAL PULL-UP or PULL-DOWN
  downButton.attach(BUTTON_DOWN_PIN, INPUT_PULLUP);
  upButton.attach(BUTTON_UP_PIN, INPUT_PULLUP); // USE INTERNAL PULL-UP
  selectButton.attach(BUTTON_SELECT_PIN, INPUT_PULLUP);
  topEndstop.attach(ENDSTOP_TOP_PLUNGER_PIN, INPUT_PULLUP);
  bottomEndstop.attach(ENDSTOP_BOTTOM_PLUNGER_PIN, INPUT_PULLUP);   // Top & Bottom NO, 5v
  barrelEndstop.attach(ENDSTOP_BARREL_PLUNGER_PIN, INPUT_PULLDOWN); // FINDA sensor is NC, 0v
  EMERGENCYstop.attach(EMERGENCY_STOP_PIN, INPUT_PULLUP);           // CHECK THAT IS NO!!

  // DEBOUNCE INTERVAL IN MILLISECONDS
  downButton.interval(debounce_Interval);
  upButton.interval(debounce_Interval);
  selectButton.interval(debounce_Interval);
  topEndstop.interval(debounce_Interval);
  bottomEndstop.interval(debounce_Interval);
  barrelEndstop.interval(debounce_Interval);
  EMERGENCYstop.interval(debounce_Interval);

  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  downButton.setPressedState(LOW);
  upButton.setPressedState(LOW);
  selectButton.setPressedState(LOW);
  topEndstop.setPressedState(LOW); // NO, with pullup, goes LOW when closed
  bottomEndstop.setPressedState(LOW);
  barrelEndstop.setPressedState(HIGH); //  NC, with pulldown, goes HIGH when active
  EMERGENCYstop.setPressedState(LOW);  //  CHECK THAT IS NO!!
}

/*
  // functions declarations, to try and organize most commonly re-used functions
  long EncoderActualPosition();                                             // reads encoder position and returns REFERENCE variable for use by other functions
  void resetEncoderZero();                                                 // resets Encoder to Zero, ONLY to be called ONCE at power up, toggle initialHomingDone
  void ProgrammedMotorMove(motorSpeed, motorAcceleration, motorMove);      // simply moves motor at set speed, accel and distance indicated
  void ContinuousMotorMoveForward(motorSpeed);                             // simply moves plunger continously (during button press or other condition is 0/1)
  void ContinuousMotorMoveUp(motorSpeed);                            // simply moves plunger continously (during button press or other condition is 0/1)
  void HomeMove(generalFastSpeed, HomingSlowSpeed);                                 // simply sends plunger to TopEndstop - CAN USE ContinuousMotorMoveUp
  void RefillPositionMove(GeneralFastSpeed/2, refillOpeningOffsetDistSteps, motorAcceleration )  // tp becalled after HomeMove, to always move to correct Refill position from Home
  bool compareMotorEncoder();                                              // function to compare Encoder & FastAccel position, to detect when skipping steps,
and therefore compression of plastic or filling of mould, and to reduce speed/distance commands until are below min threshold,
when then returns "complete"/"1" to function where has been called from
  void compressionFunction(initialCompressionSpeed, minCompressionSpeed);  // ChatGPT written compression uses compareMotorEncoder funtion
  void clearLEDs();                                                        // clears LEDs and turns off, before next colour setting
  void ButtonLEDsColors(selectLEDcolour, upLEDcolour, downLEDcolour);      // sets colours of LEDs behind buttons
  void SerialRegular100msMessages();                                       // every 100ms, send via serial the Temp, Encoder position, etc
  void SerialChangeOfState();                                              // serial print when a change of state happens, and new state
  void SerialPrintNextInjectionValues();                                   // serial print the parameters of next injection
  void BounceButtonsSetup();                                               // setup of buttons list to avoid setup() overload
  void BounceButtonsUpdateCall();                                          // same for button update call in loop()
  int CheckMinTempEndstopsEmergencyStopPin();                                // should run in each loop, and if any endstop or emergency stop is triggered, stop machine
and report which endstop has been triggered... possible thereafter only allow moves in opposite direction, or back to refill, or
other options..? */

//////////////////////////////////

// functions fully described

//   /**
//    * Encoder PCNT library use should mean that encoder value is callable at any time (will always give back
//    * current position, so really only needed at start of relative motor moves, to check next move positions)
//    * Encoder function will get called not every loop, but maybe every 10ms, to give time for max distances to trigger other code, and every 100ms
//    * wil be called by SerialRegular100msMessages, where if motor is stopped, actual = old, and no need to print value
//    */
//   long EncoderActualPosition() {

// // FIXME commented out for testing
//     //actualENPosition = encoder.getCount() / 2;  // revise last int, if motor steps = 400 & encoder PPR = 1000, adjust this int
//                                                 // (possible issue of needing to round, this is bad, better maybe to increase driver steps to same as encoder?)
//     oldENPosition = actualENPosition;           // if this does NOT change between Serial.prints, then don't print

//     return (actualENPosition);
//   }

/**
 *
 */
void resetEncoderZero()
{

  // FIXME commented out for testing - uncommented by Andy and changed from clearCount.. why clearCount and not setCount(0)?
  encoder.setCount(0);

  return;
}

/////////////////////////////////////
// motor functions

/**
 * NOTE: When calling this function from different states, it will be called with the arguments particular to that state, ex.
 * INJECT will pass ProgrammedMotorMove(fillMouldMoveSpeed, fillMouldAccel, fillMouldMoveDistSteps)... will this AUTOMATICALLY write those
 * parameters to the functions named paramters, ie motorSpeed, motorAcceleration, motorMove..? Because if I wish to do a later
 * compareMotorEncoder function, this function needs to have the ProgrammedMotorMove NAMED params, not the PASSED params names..?
 *
 * @param motorSpeed The speed...
 * @param motorAcceleration Accelerattion...
 * @param motorMove Move the motor forward or backward
 *
 * NOTE ABOUT using shortened simplier function calls: as expressed below, can call programmedMotorMove with 2 (speed & distance) or 3 (w/ accel as well) parameters...
 * when using 2 parameteres, then the accel parameter will be auto-added with the "default" setting from this function definition, AFAIK! example:
 * programmedMotorMove(1000,2000); speed & steps only, programmedMotorMove (1000,2000,3000); full control including acceleration
 *
 * REVISE ALL CALLS OF programmedMotorMove IN OTHER FUNCTIONS, THIS HAS CHANGED THE ORDER OF THE PARAMETERS, PREVIOUSLY ACCEL WAS SECOND AND NOW IS THIRD!
 * DO ***NOT*** WANT TO MIX UP OLD ACCELERATION PARAMTER WITH NEW DISTANCE PARAMETER
 */
void programmedMotorMove(int motorSpeed, int motorDistance, int motorAcceleration = defaultAcceletationNema) // RELATIVE motor move to new position
{
  if (doMoveMotor)
  {
    doMoveMotor = false;

    currentMotorSpeed = motorSpeed;
    currentAcceleration = motorAcceleration;
    currentMotorDistance = motorDistance;

    stepper->setSpeedInHz(motorSpeed);
    stepper->setAcceleration(motorAcceleration);
    stepper->move(motorDistance); // <- RELATIVE move, motor position should be zeroed, and then move is simply set from function variable,
                                  // or could be set from encoder position and ABSOLUTE move (moveTo) used, but ADDING function variable to actual position
  }
}

/**
 * CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
 */
void continuousMotorMoveDown(int motorSpeed)
{
  currentMotorSpeed = motorSpeed;

  stepper->setSpeedInHz(motorSpeed);
  stepper->runForward();
}

/**
 * CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
 */
void continuousMotorMoveUp(int motorSpeed)
{
  currentMotorSpeed = motorSpeed;

  stepper->setSpeedInHz(motorSpeed);
  stepper->runBackward();
}

int HomingSlowSpeed = maxSpeedLimit / 10; // how slow to home, 2nd approach, continous move until reaching endstop

/**
 * FIXME
 * once called, no user action can stop until complete!
 */
void homeMove(int homingFastSpeed, int homingSlowSpeed)
{
  // int HomingDirection;      // maybe not needed, as using ContMoveBackwards
  int homeOffSetDistSteps = 212;           // once first reached HomeEndstop, how much to back off before slower approach, 212 steps ≈ 5mm
  int homeOffSetAccel = 10000;             // once first reached HomeEndstop, how much Accel to back off before slower approach 10000 = 1/5th normal
  int homeOffsetSpeed = maxSpeedLimit / 2; // once first reached HomeEndstop, how quick to back off before slower approach
  // int homingFastSpeed = generalFastSpeed;  // these 2 variables need to be globals as they are called in this function, but also in other functions
  // int homingSlowSpeed = maxSpeedLimit / 10; // how slow to home, 2nd approach, continous move until reaching endstop

  if (digitalRead(ENDSTOP_TOP_PLUNGER_PIN) == !topPlungerEndstopActive)
  {
    continuousMotorMoveUp(homingFastSpeed); // OR ProgrammedMotorMove with maxHomingStepsDistSteps, if fails, gives error?
  }

  programmedMotorMove(homeOffsetSpeed, homeOffSetDistSteps, homeOffSetAccel); // programmed motor move assumed to be positive DistSteps..?
  // FIXED: ACCEL LAST PARAMETER OK
  if (digitalRead(ENDSTOP_TOP_PLUNGER_PIN) == !topPlungerEndstopActive)
  {
    continuousMotorMoveUp(homingSlowSpeed);
  }
}

// FIXME moved this definitions & function above first declaration below
boolean changeSelectLEDcolour;
boolean changeUpLEDcolour;
boolean changeDownLEDcolour;

uint32_t currentSelectLEDcolour;
uint32_t currentUpLEDcolour;
uint32_t currentDownLEDcolour;

void buttonLEDsColors(uint32_t newSelectLEDcolour, uint32_t newUpLEDcolour, uint32_t newDownLEDcolour) // could use case/switch instead? but are different types of defining behaviours..?
{
  changeSelectLEDcolour = currentSelectLEDcolour != newSelectLEDcolour;
  if (changeSelectLEDcolour)
  {
    currentSelectLEDcolour = newSelectLEDcolour;
  }

  changeUpLEDcolour = currentUpLEDcolour != newUpLEDcolour;
  if (changeUpLEDcolour)
  {
    currentUpLEDcolour = newUpLEDcolour;
  }

  changeDownLEDcolour = currentDownLEDcolour != newDownLEDcolour;
  if (changeDownLEDcolour)
  {
    currentDownLEDcolour = newDownLEDcolour;
  }
}

// void RefillPositionMove(int GeneralFastSpeed, int refillOpeningOffsetDistSteps, int motorAcceleration)
// {
//   programmedMotorMove(GeneralFastSpeed / 2, refillOpeningOffsetDistSteps, motorAcceleration);
// }  // FIXME  not needed if transitionToState==REFILL has same ProgrammedMotorMove??

// function to compare Motor & Encoder positions
bool compareMotorEncoder(int motorMove, int actualENPosition, int comparisonPercentage) // chatGPT written
{
  // Calculate difference between motor position and encoder position
  int motorError = abs(motorMove - actualENPosition);

  // Calculate the acceptable difference threshold
  int threshold = (motorMove * comparisonPercentage) / 100;

  // Return true if the difference is greater than the threshold
  return (motorError >= threshold);
}

/**
 * FIXME
 * motor compression function takes compareMotorEncoder and motor moves to apply pressure up to a threshold and then
 * reduce the speed (increases torque of stepper?) until again threshold past, etc, until minCompressionSpeed reached,
 *  thenwill stop
 */
void compressionFunction(int initialCompressionSpeed, int minCompressionSpeed) // chatGPT written
{
  buttonLEDsColors(RED_RGB, BLACK_RGB, RED_RGB);
  selectButton.update();

  // add ButtonSelect interrupt function on user press, goes to Ready_to_Inject (also EStop..? or EStop better permament in loop
  //  and then goes to ERROR-STATE?)
  if (!selectButton.isPressed())
  {
    // Set initial speed
    stepper->setSpeedInHz(initialCompressionSpeed); // Starting speed in steps per second

    while (stepper->getSpeedInMilliHz() / 1000 > minCompressionSpeed)
    {
      // Move the plunger down
      stepper->runForward();

      // FIXME commented out for testing - Andy uncommented for testing!

      // Call the comparison function to check if motor-encoder difference exceeds threshold
      if (compareMotorEncoder(stepper->targetPos(), encoder.getCount(), comparisonPercentage))
      {
        // Reduce speed when the difference is greater than the allowed threshold
        int newSpeed = stepper->getSpeedInMilliHz() / 1000 * speedReductionFactor / 10;
        stepper->setSpeedInHz(newSpeed);
        buttonLEDsColors(RED_RGB, BLACK_RGB, YELLOW_RGB); // to show compression is acting (has reduced speed at least once) and ongoing..

        // Optionally, reduce comparison threshold on each iteration (for finer control)
        comparisonPercentage *= speedReductionFactor; /* could lead to infinite smaller steps if minCompressionSpeed is too low...?
        for debugging this option, would be good to show actual speed - if later wish to add another compressionDuringHeating function
        where the plastic is already quite compressed, and will start (and end) from/to a much lower speeds, have to make sure this
        does not produce infinite loop as never reaches minCompressionSpeed */
      }
    }

    // Stop motor when speed falls below threshold
    stepper->stopMove();
    buttonLEDsColors(RED_RGB, BLACK_RGB, GREEN_RGB); // to show compression is complete
    delay(500);                                      // ONLY DELAY IN ENTIRE SKETCH, and is only to give time for GREENrgb LED to be shown before

    //  return;  //  FIXME - removed return and added line below... once finished function should return to function that called it, be it compression or inject.. should be break..?
    currentState = InjectorStates::READY_TO_INJECT;
  }
  else
  {
    stepper->stopMove();
    currentState = InjectorStates::READY_TO_INJECT;
  }
}

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

/**
 *
 */
void outputButtonLEDsColors() // could use case/switch instead? but are different types of defining behaviours..?
{
  if (selectButtonPressed || upButtonPressed || downButtonPressed) // set brightness to about 80% on any button press
  {
    keypadleds.setBrightness(ledHBrightness);
  }
  else
  {
    keypadleds.setBrightness(ledLBrightness);
  }

  if (changeSelectLEDcolour)
  {
    keypadleds.setPixelColor(0, currentSelectLEDcolour);
    changeSelectLEDcolour = false;
  }

  if (changeUpLEDcolour)
  {
    keypadleds.setPixelColor(1, currentUpLEDcolour);
    changeUpLEDcolour = false;
  }

  if (changeDownLEDcolour)
  {
    keypadleds.setPixelColor(2, currentDownLEDcolour);
    changeDownLEDcolour = false;
  }

  keypadleds.show();

  return;
}

void serialPrintNextInjectionValues()
{
  Serial.println("Next injection values:");
  // add here struct of variablePerMouldParams

  return;
}

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

  if (nozzleTemperature <= minTempForAnyMove) // and stop machine
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
  // FIXME add error state when function REQUIRES something under barrel (compression, purge or mould fill)
  return error;
}

/**
 * to transition states would also apply button activation / availablity..? or this only in machine states..?
 * some motor moves user may want to stop before motor move is completed..? INJECT or HOLD, or COMPRESSION
 * how to apply ContinuousMotorMoves..? As in Purge, where State does not change whilst moving motor.. this function must be in Machine State..?
 *
 * also, transitionToState motor moves are completely furfilled BEFORE entering new state..? is Compression move finished
 * before entering COMPRESSION, where the Select button press can cancel..? Although a !Select is included in CompressionFunction...
 *
 * Encoder zeroing is performed after move to home in INIT_HOMED_ENCODER_ZEROED?
 * maybe another State in between is needed?
 *
 * note: REFILL State will requiere homeMove before OffsetMove, as offset is RELATIVE from HOME
 *
 * also, how is toState set..?
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
      homeMove(homingFastSpeed, homingSlowSpeed); // followed by
      // encoder.setCount(0);  // or this last line should go in machineState..? reset has to occur AFTER homeMove finishes, but not PART of homeMove
    }
    else if (toState == REFILL) // once INIT_HOMED_ENCODER_ZEROED done, REFILL is where plunger at homed offset for plastic filling
    {
      doMoveMotor = true;
      homeMove(homingFastSpeed, homingSlowSpeed); // to do next programmedMotorMove, has to be from homed move, as next move is RELATIVE MOVE
      programmedMotorMove(generalFastSpeed, refillOpeningOffsetDistSteps /*,defaultAcceletationNema*/);
    }
    else if (toState == COMPRESSION)
    {
      doMoveMotor = true;
      compressionFunction(initialCompressionSpeed, minCompressionSpeed);
    }
    else if (toState == INJECT) // FIXME   should be ABSOLUTE motor move, relative to Purge Zeroed position, after ANTIDRIP has moved plunger backwards a small amount...
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
  // static StatesMachine runState;  // ?? useful here, or to make runState a Static variable between loops
  //  maybe can add "static" to above enum declaration...?

  /** button activation/ availability is activted on entering new state..? if motor move is started during transtion, STOPPING move by user
   * can only be done once in new State..?
   */

  switch (currentState)
  {
  case InjectorStates::ERROR_STATE: // function that includes stopMove(), flashes LEDs red (maybe with
    // Serial.println("ERROR!! " + error);

    buttonLEDsColors(RED_RGB, RED_RGB, RED_RGB); // FIXME ERROR_STATE should be all red, with flashing sequence as per Error to Identify

    break;

  case InjectorStates::INIT_HEATING: //
    buttonLEDsColors(RED_RGB, RED_RGB, RED_RGB);

    if (nozzleTemperature > minTempForAnyMove)
    {
      currentState = InjectorStates::INIT_HOT_NOT_HOMED;
    }

    break;

  case InjectorStates::INIT_HOT_NOT_HOMED:
    buttonLEDsColors(YELLOW_RGB, YELLOW_RGB, YELLOW_RGB);

    if (upButtonPressed)
    {
      currentState = InjectorStates::INIT_HOMED_ENCODER_ZEROED;
    }

    break;

  case InjectorStates::INIT_HOMED_ENCODER_ZEROED:
    buttonLEDsColors(YELLOW_RGB, YELLOW_RGB, YELLOW_RGB);

    //  resetEncoderZero();  // use a separate function taht does the same, or just the command..
    encoder.setCount(0);
    currentState = InjectorStates::REFILL;

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

    if (selectButtonPressed)
    {
      currentState = InjectorStates::COMPRESSION;
    }

    if (upButtonPressed && downButtonPressed)
    {
      endOfDayFlag = !endOfDayFlag;
    }

    break;

  case InjectorStates::COMPRESSION:
    buttonLEDsColors(RED_RGB, BLACK_RGB, RED_RGB);

    // if (selectButtonPressed) //  there is already an "if !Select" in the CompressionFunction that does this same ...
    // {
    //   stepper->stopMove();
    //   currentState = READY_TO_INJECT;
    // }

    //  ... however, the CompressionFunction has a "return" at the end, as was initially contemplated to be able to use in INJECT
    //  as well as COMPRESSION States, in the case that want to assure max pressure filling as well as max pressure Compression...
    //  revise and decide what is most appropiate usecase, as otherwise, ATM, there is no way of getting to READY_TO_INJECT
    break;

  case InjectorStates::READY_TO_INJECT:
    buttonLEDsColors(GREEN_RGB, YELLOW_RGB, GREEN_RGB);

    if (selectButtonPressed && downButtonPressed)
    {
      currentState = InjectorStates::PURGE_ZERO;
    }
    else if (upButtonPressed)
    {
      currentState = InjectorStates::REFILL;
    }

    break;

  case InjectorStates::PURGE_ZERO:
    buttonLEDsColors(GREEN_RGB, YELLOW_RGB, YELLOW_RGB);
    if (selectButtonPressed)
    {
      stepper->setCurrentPosition(0);
      currentState = InjectorStates::ANTIDRIP;
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

  case InjectorStates::ANTIDRIP:
    buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB);

    if (selectButtonPressed)
    {
      stepper->stopMove();
      currentState = InjectorStates::READY_TO_INJECT;
    }

    if (upButtonPressed && downButtonPressed)
    {
      currentState = InjectorStates::INJECT;
    }

    break;

  case InjectorStates::INJECT:
    buttonLEDsColors(RED_RGB, GREEN_RGB, BLACK_RGB); // middle/UP LED GREEN flashing

    if (selectButtonPressed)
    {
      stepper->stopMove();
      Serial.println("Motor moved " && stepper->getCurrentPosition() && " steps"); // send via serial the actual steps moved by motor: in the case mould was overfilling, can be useful info for user
      //
      currentState = InjectorStates::RELEASE;
    }

    else // here the function will wait for the transitionToState motor move to complete before moving to the next State??
    {
      currentState = InjectorStates::HOLD_INJECTION;
    }

    break;

  case InjectorStates::HOLD_INJECTION:
    buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB); // bottom/DOWN LED GREEN flashing

    if (selectButtonPressed)
    {
      stepper->stopMove();
      currentState = InjectorStates::RELEASE;
    }

    else // here the function will wait for the transitionToState motor move to complete before moving to the next State??
    {
      currentState = InjectorStates::RELEASE;
    }

    break;

  case InjectorStates::RELEASE:
    buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);

    currentState = InjectorStates::CONFIRM_MOULD_REMOVAL; // here the function will wait for the transitionToState motor move to complete before moving to the next State??

    break;

  case InjectorStates::CONFIRM_MOULD_REMOVAL:
    buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);

    if (endOfDayFlag == 0)
    {
      if (selectButtonPressed || upButtonPressed || downButtonPressed)
      {
        currentState = InjectorStates::REFILL;
      }
    }

    if (endOfDayFlag == 1)
    {
      if (selectButtonPressed || upButtonPressed || downButtonPressed)
      {
        currentState = InjectorStates::READY_TO_INJECT;
      }
    }
    break;
  }
}

/**
 * Comms functions, every 100ms send data via serial
 */
long avgLoopTime = 0;
long numLoops = 0;
long m = 0;
void serialRegular100msMessages()
{
  // if (millis() - 100 >= printTime)  // only when 100ms has past...
  long newM = micros();
  if (m + 1000000 < newM)
  {
    Serial.println("===============");
    Serial.printf("loops: %d, avg time: %d\n", numLoops, avgLoopTime);
    Serial.printf("state: %d, ER: %d\n", currentState, error);
    Serial.println("selectLed: " + String(currentSelectLEDcolour, HEX));
    Serial.println("upLed: " + String(currentUpLEDcolour, HEX));
    Serial.println("downLed: " + String(currentDownLEDcolour, HEX));

    Serial.printf("temps: NZ=%d\n", nozzleTemperature);

    Serial.printf("inputs: ES=%d, SEL=%d, UP=%d, DW=%d, TE=%d, BE=%d, BR=%d\n", emergencyStop, selectButtonPressed, upButtonPressed, downButtonPressed, topEndStopActivated, bottomEndStopActivated, barrelEndStopActivated);

    // // FIXME commented out for testing
    //  FIXME  use SafeString library for ALL SERIAL PRINTS, UART COMMS, and MILLISDELAYS!! very robust library with many error mensajes, non-blocking
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
    m = newM;
  }

  if (numLoops != 0)
  {
    avgLoopTime = (avgLoopTime / numLoops) + m / (numLoops + 1);
  }
  else
  {
    avgLoopTime = m;
  }
  numLoops++;
}

void loop()
{
  long startTime = millis();

  getInputs();

  error = sanityCheck();
  if (error != InjectorError::NO_ERROR)
  {
    transitionToState(InjectorStates::ERROR_STATE);

    Serial.printf("Found error!. Changing to ERROR_STATE to INIT_HEATING with error %d\n", error);
  }

  machineState();

  // output
  outputButtonLEDsColors();

  // debug
  serialRegular100msMessages();

  long endTime = millis();

  long elapsedTime = endTime - startTime;
  //
}

void setup()
{
  setupSerial();
  setupButtons();
  // Initialize other hardware components here

  delay(2000); // wait 2 seconds. REQUIERED !!!
  Serial.println("injector_ESP32_state_machine");

  // encoder setup

  encoder.attachHalfQuad(ENCODER_A_PIN, ENCODER_B_PIN);
  encoder.setCount(0);

  // stepper setup
  // #define DRIVER_RMT 1  // type of driver for FAS, to separate any motor interference, not needed as directly used "1" below..?
  engine.init();

  // FIXME  Core assignment... NECESSARY?!? Revising webs, comms BETWEEN cores can be a problem... test and check, decide if needed to assign distinct cores
  // useful info..? https://www.codeproject.com/Articles/5295781/Use-both-cores-on-an-ESP32-Easy-synchronization-wi
  // engine.init(1);
  // FIXME  DRIVER type used
  // stepper = engine.stepperConnectToPin(STEPPER_STEP_PIN, 1);  // "1" defines DRIVER_RMT, for encoder to use PCNT, not Stepper
  // refere to https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md
  // for ways to CPU_CORE & DRIVER_TYPE, if this does not compile as-is, and
  // https://valarsystems.com/blogs/val-2000/section-9-dual-core-setup for alternative CPU_CORE method
  // FIXME can also use ENCODER with FA, attachToPulseCounter, see FastAccelStepper.h for all 3 above points
  // see also https://oss.issuehunt.io/r/madhephaestus/ESP32Encoder/issues/43..?
  // and https://github.com/gin66/FastAccelStepper/issues/48

  stepper = engine.stepperConnectToPin(STEPPER_STEP_PIN);
  stepper->setAbsoluteSpeedLimit(3200); // 16.000.000 tick/s / 2000steps/s maxSpeedLimit = 8000 ticks/ step
                                        // 16.000.000 tick/s / 5000steps/s maxSpeedLimit = 3200 ticks/ step

  if (stepper)
  {
    stepper->setDirectionPin(STEPPER_DIR_PIN); // possible not needed if moveTo commands are also -ve..?
  }

  // thermocouple setup
  thermocouple.begin();

  // keypadLEDs setup
  keypadleds.begin(); // Call this to start up the LED strip.
  //  clearLEDs();   // This function, defined below, turns all LEDs off...
  keypadleds.show(); // ...but the LEDs don't actually update until you call this.

  // Buttons & Endstops Bounce2 setup is very long, so called from the following function
  bounceButtonsSetup();

  currentState = InjectorStates::INIT_HEATING;
  error = InjectorError::NO_ERROR;

  readNozzleTemp = true;
  readEmergencyStop = true;
}