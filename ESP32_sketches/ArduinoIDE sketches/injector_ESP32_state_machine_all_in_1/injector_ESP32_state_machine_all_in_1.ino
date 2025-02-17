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

// pin definitions, decided on from "GPIO_Limitations_ESP32_NodeMCU.jpg", and which side may be more convenient

#define UART_tx_ESP32 17                         // * TX2 UART comms with ESP32
#define UART_rx_ESP32 16                         // *
#define ENCODERApin 36                           // * A+ encoder pin, A- not used  "CLK ENCODER "
#define ENCODERBpin 39                           // * B+ encoder pin, B- not used  "DT ENCODER "
#define StepperSTEPPin 32                        // * any GPIO for ESP32
#define StepperDIRPin 33                         // * may not be needed if using -ve moves..?
#define EndstopTOPPlungerPin 22                     // * all buttons/endstops with INPUT_PULLUP to avoid floating values
#define EndstopBOTTOMPlungerPin 21                  // * so all ACTIVE STATES are ==0
#define EndstopBARRELClampOKPin 5                   // *
#define EndstopMOULDPresentPin 12                   // do not yet have a way of detecting, possibly pressure sensor under plastic floor on platform..?
#define EndstopNOZZLEBlockForCompressOrPurgePin 15  // also as above, not decided hardware method yet
#define TEMPNozzleVSPI_MISO_DO MISO              // * 19
#define TEMPNozzleVSPI_Dpin_MOSI_CS MOSI         // * 23 these 3 pins are for daughter PCB thermocouple
#define TEMPNozzleVSPI_SCK_CLK SCK               //  *  18 (possible change to analog component if run out of pins
#define BUTTONDownPin 26                         // * these 3 pins for lower keypad, to facilite Purge and INject Confirm
#define BUTTONUpPin 27                           // *
#define BUTTONSelectPin 14                       // *
#define WS2812B_ButtonAddressableLEDsPin 25         // * 1 pin to address 3 LEDs buttons...
#define WS2812B_RingAddressableLEDsPin  2            // 1 pin for 35 LEDs nozzle ring, to change color as per moment of use..
#define EMERGENCYStopPin 4                           //  *
//#define CURRENTSensor1   34      // * as yet to be decided on how to use motor phase current readng
//#define CURRENTSensor2   35      // * to be useful for detecting motor current (reaching amc compression/fill)

//////////////////////////////////

// global constants
// revise what is now NOT NEEDED, OR what variables should go inside Functions to make LOCAL

/* global common variables for machine limits and moves to offsets, these, once identified the max values
should NEVER need changing, even from advanced Common panel, as superior values run the risk of machine
performing outside of the decided security limits of movement/accel, distances, etc
revise what is  NOT NEEDED, OR what variables should go inside Functions to make LOCAL*/
const int minTempForAnyMove = 170;              // min temp considered safe for machine move
const int maxSpeedLimit = 2000;                 // 2000 corresponds to 5rps NEMA, about 1/4 rps gear, about 18.85/4 = 4.7125cm linear
                                                // plunger movement, about 25cm3 injected volume, per second MAX speed limited
const int moveContinuosDistSteps = 5000;        // arbitrary large distance for continuous movements, gets reset to 0 each loop anyway
const int defaultAcceletationNema = 50000;      /* check in reality, maybe large moulds will benefit from slower acceleration together with higher speeds..?
                                            to avoid blocking motor/losing steps with high speed requests with too high accelerrations..
                                           or smaller moulds benefit also from slower acceleration/speed so as not to explode, or fill with
                                           less acceleration tapering off... */
const int maxHomingStepsDistSteps = 30000;      // CHECK THIS NUMBER OF STEPS COVERS WHOLE MOVEMENT RANGE, HEATED ZONE + REFILL + TOPENDSTOP + extra
const int totalStrokeStepsDistSteps = 21220;    //calc'd value, check against real, this is ONLY HEATED ZONE, need another OFFSET from Encoder 0
const int refillOpeningOffsetDistSteps = 3800;  // ESTIMATED! once homed to top endstop, how far to move to position just above filling hole in barrel
                                                // THIS VALUE HAS TO BE FOUND BY CALIBRATION AND ADJUSTED TO REALITY!!
const int heatedZoneOffsetDistSteps = 7200;     // calc'd value, check with reality
const int buttonShortPress = 500;               // for short press of button, ms
const int buttonLongPress = 2000;               // long press...
const int debounceInterval = 5;


// for motor Funtion, to be re-written depenging on State to these variables
int motorSpeed;         // example, for INJECT state, motorSpeed=FillMouldMoveSpeed, then call ProgrammedMotorMove
                        // should use this updated value for this next call of ProgrammedMotorMove
int motorAcceleration;  // same as above, requires each State that contemplates using ProgrammedMotorMove
                        // to have all three values, otherwise risk of using older value from previous call!
int motorMove;          // same as above, is a RELATIVE move (not moveTo!)
int MotorDir;           // direction of ANY motor move, or can this be omitted as DistSteps variables can be -ve..?
int MotorPosition;      /* reference position for next ProgrammedMotorMove, set to 0 after Purge is finished... possibly 
                        not needed as all moves should be RELATIVE, and resetting motor position does not need a variable */
int stepsMoved = 0;     // Track the number of steps moved, to compare to steps made, and also for encoder comparison
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
char StatesMachine;
char runState;  // states for movement functions (purge, inject, etc), 0 to ensure when no button is pressed, no movement
// int readyState=0  // states for non-moving ready states  - N/A
long int PurgeSpeed = 80;                   // speed for purge move, continous move until button release  80= 1cm3/s
long int GeneralFastSpeed = maxSpeedLimit;  // how fast to home, continous move until reaching endstop, also for other general moves..? max /2..?
// homing params moved to Homng Function as not needed elsewhere
// int HomingDirection;      // maybe not needed, as maxHomingStepsDistSteps can be -ve..?
// int HomeOffSetDistSteps;       // once first reached HomeEndstop, how much to back off before slower approach
// int HomeOffSetAccel;      // once first reached HomeEndstop, how much Accel to back off before slower approach
// int HomeOffsetSpeed;      // once first reached HomeEndstop, how quic to back off before slower approach
// int HomingSlowSpeed;      // how slow to home, 2nd approach, continous move until reaching endstop
long int initialCompressionSpeed = GeneralFastSpeed / 2;                    // could be GeneralFastSpeed / 2..?
long int minCompressionSpeed = GeneralFastSpeed / 20;                       // at what speed is compression no longer useful..?
long int comparisonPercentage = 50;                                         /* EXAMPLE VALUE what % difference between sent motor steps and read encoder steps should a change 
                            in speed be commanded?
                            higher % will cuase more skipping before changing speed, lower % will cause more speed changes (o vica versa),
                            but also possibly increase probability of infinite loop, if use line comparisonPercentage *= speedReductionFactor; */
long int speedReductionFactor = 5;                                          // EXAMPLE VALUE Percentage to reduce speed when comparisonPercentage limit reached
long int AntiDripReverseTime = 15000;                                       // 15s in case if using ConstMotorMove, instead of ProgrammedMotorMove, how many seconds before cancel..?
long int AntiDripReverseSpeed = stepsToCML / (AntiDripReverseTime / 1000);  // 80steps/15, per sec.. after purge, to recuperate and avoid drip until mould is placed - TEST
long int AntiDripReverseMillis;                                             //  after purge, start millis timer
// using distance: normally the user SHOULD press INJECT button BEFORE this move is completed, but if is ProgrammedMotorMove,
// can I interrupt this move before it completes..?
long int AntiDripFinalOffset;                    // once AntiDrip constMoveBack is interrupted with button press for injection, store actual motor position and sum to 0?
long int ReleaseMouldMoveSpeed = maxSpeedLimit;  // after hold, must raise the barrel a few mm to free mould for removal.. most likely will be a constant
long int ReleaseMouldMoveDistSteps = -200;       //  same as above, 200 steps is approx 0.47cm, -ve as ProgMotorMove is normally +ve downward
long int constantInjectionParams[] = {
  PurgeSpeed,
  GeneralFastSpeed,
  initialCompressionSpeed,
  minCompressionSpeed,
  comparisonPercentage,
  speedReductionFactor,
  AntiDripReverseTime,
  AntiDripReverseSpeed,
  ReleaseMouldMoveSpeed,
  ReleaseMouldMoveDistSteps
};

/* array to hold message with above constant parameters, to be assigned to and returned to ESP32..?
 ={PurgeSpeed, GeneralFastSpeed, initialCompressionSpeed, AntiDripReverseSpeed, AntiDripReverseDistSteps, ReleaseMouldMoveSpeed, ReleaseMouldMoveDistSteps}
*/

/* actual injection variables, will be adjusted each time for each different mould, user will have access
via panel on display to change these values whilst calibrating a mould, and will then be stored on Display
ESP32 in Flash as array/struct of values particular to each mould
*/


char MouldName;              // unique string name for stuct of mould parameters for below variables
int FillMouldMoveSpeed;      /* speed of next move, sent from ESP32, test, is possible different for 2D and 3D moulds, 
higher speeds will lead to earlier over-current errors */
int FillMouldAccel;          // possibly not needed, have to discover whether small 2D/2.5D moulds benefit from slower accel on fill..?
int FillMouldMoveDistSteps;  /* distance to move plunger to fill mould, sent from ESP32 at the start of each injection, as 
per mould in use */
int HoldMouldMoveSpeed;      /* ... and how fast (will be a low speed, test.. this will have to translate into a timed move,
for example has to last 15s to allow 25g of plastic to cool enough, then distSteps * speed has to take 15s.
make formula that can take time requiered, distance to move, and divide to get speed OR use ContMove for Time..? */
int HoldMouldMoveDistSteps;  // after mould fill, apply a little more move/pressure to get good surface finish
struct actualMouldParams {
  const char* MouldName;
  int FillMouldMoveSpeed;
  int FillMouldAccel;
  int FillMouldMoveDistSteps;
  int HoldMouldMoveSpeed;
  int HoldMouldMoveDistSteps;
} ;

struct actualMouldParams mouldNow = { "Posavasos", maxSpeedLimit, motorAcceleration, 2160, (maxSpeedLimit / 10), 80 };
/* struct to hold received message with above variable parameters... can I include in {variable name1, variable name2, etc}..?
={FillMouldMoveSpeed, FillMouldMoveDistSteps, HoldMouldMoveSpeed, HoldMouldMoveDistSteps}, better practice would be struct including name
of mould and list of valies ofr the variables above, which would then be written to these variables ... include check of 
message (5 comma separated values with special starting char?) length */



// comms between Arduino & ESP32
char messageToDisplay;    /* message character to be sent to display, some letters mean errors, some to display text, some as first message
to second message that contain parameter arrays for variables or constants (for confirmation on screen) */
int messageArrayToESP32;  // parameter array, can be either variable array or constants array, depending on previous message..?

// encoder & position data from FAS library
long oldFA0Position = 0;         // encoder postion on motor FAS zeroing - maybe not need, revise ComparisonFunction
long actualFAPosition;           // getPosition from zeroed FastAccel library, to compare to Encoder data, and to send to ESP32 for display
unsigned long oldENPosition;     // for referencing encoder positions..
unsigned long actualENPosition;  // same for encoder data.. if cannot zero encoder data (so far only zeros on Arduino reset), then also make
                                 // oldENPosition, and subract to actualOldENPosition on FA zeroing, before substituting old with actual, should
                                 // give same value since last zero of FA



// endstop states, active or not, other checks to relay via Serial. mostly active LOW, with PULLUP (exceptions as per endstop)
int tempNozzleDegrees;                     // var to fill from temp sensor, to compare to a min temp to allow any type of movement
bool EMERGENCYStopPinActive = 0;              // is Emergency Stop true, then cannot allow movement - consider wiring also direct to stepper motor wires
                                           // but ONLY if sure that breaking this connection will not damage driver.. otherwise, also wire to driver PSU
bool topPlungerEndstopActive = 0;          // is 1/NC, then can move up, if 0/NC, then triggered
bool bottomPlungerEndstopActive = 0;       // is 1/NC, then can move down, if 0/NC, then triggered
bool barrelClampOKEndstopActive = 1;       // is 0/NO, then can move inject, if 1/NC, then triggered, barrel has moved!!
bool mouldPresentEndstopActive = 0;        // still to see how to implement, to assure that nozzle is blocked by either mould or
bool nozzleBlockPresentEndstopActive = 0;  // purge bar/cap, so can move down w/o displacing barrel
bool endOfDayFlag = 0;                     // 0 = refillAfterInject, 1= skip and, possibly, measure steps since last top endstop - could read from encoder.. as FA library
                                           // is contemplated to be reset after each purge, so if ==0, reset encoder, ==1 do not reset encoder
bool initialHomingDone = 0;                // on power on, machine needs homing, then mark this flag to show has been done at least once
int errorReason;


// lower keypad button states, could be bool?  Or just use directly digitalRead of pin in any arguments/functions..?
bool DownPinState;    // var for purge function, read from BUTTONDownPin pin
bool UpPinState;      // var for purge function, read from BUTTONUpPin pin
bool SelectPinState;  // var for purge function, read from BUTTONSelectPin pin

//  LED stuff
int ledLBrightness = 50;       // initial brightness for LEDs, to possible increase to 100 during an actual press..?
int ledHBrightness = 200;      // pressed key brightness for LEDs, to increase to 200 (of 255 max) during an actual press..?
const int keypadLedCount = 3;  // 3 for just keypad buttons, +35 for LED ring on end of barrel!
const int ringLedCount = 35;  // 3 for just keypad buttons
#define GREENrgb  0, 128, 0
#define REDrgb  255, 0, 0
#define YELLOWrgb  255, 140, 0
#define BLUErgb  0, 0, 255
#define BLACKrgb  0, 0, 0
char selectLEDcolour;
char upLEDcolour;
char downLEDcolour;
const int DEBOUNCE_INTERVAL = 5;





// libraries to include

#include <ESP32Encoder.h>  // * https://github.com/madhephaestus/ESP32Encoder.git
ESP32Encoder encoder;      // should use PCNT timer on ESP32, so should be callable at any moment with updated encoder value..?

#include <FastAccelStepper.h>  // *
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
stepper->setAbsoluteSpeedLimit(3200);  // 16.000.000 tick/s / 2000steps/s maxSpeedLimit = 8000 ticks/ step
                                                           // 16.000.000 tick/s / 5000steps/s maxSpeedLimit = 3200 ticks/ step

#include <elapsedMillis.h>
elapsedMillis printTime;

//#include <ACS712.h> // current sensor, may not be needed, but added 2 to motor phase B + & - just in case
#include <SPI.h>
#include <Adafruit_MAX31855.h>  // temp board, was enclosed with quotes "" in example, not sure why
Adafruit_MAX31855 thermocouple(TEMPNozzleVSPI_SCK_CLK, TEMPNozzleVSPI_Dpin_MOSI_CS, TEMPNozzleVSPI_MISO_DO);

#include <Adafruit_NeoPixel.h>  // ws2812B RGB addressable LEDs
Adafruit_NeoPixel keypadleds = Adafruit_NeoPixel(keypadLedCount, WS2812B_ButtonAddressableLEDsPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ringleds = Adafruit_NeoPixel(ringLedCount, WS2812B_RingAddressableLEDsPin, NEO_GRB + NEO_KHZ800);

#include <Bounce2.h>  // bounce library allows to time button presses
Bounce2::Button down_button = Bounce2::Button();
Bounce2::Button up_button = Bounce2::Button();
Bounce2::Button select_button = Bounce2::Button();
Bounce2::Button TOP_ENDSTOP = Bounce2::Button();
Bounce2::Button BOTTOM_ENDSTOP = Bounce2::Button();
Bounce2::Button BARREL_ENDSTOP = Bounce2::Button();
Bounce2::Button EMERGENCY_STOP = Bounce2::Button();



  void BounceButtonsSetup() {
    // IF YOUR BUTTON HAS AN INTERNAL PULL-UP or PULL-DOWN
    down_button.attach(BUTTONDownPin, INPUT_PULLUP);
    up_button.attach(BUTTONUpPin, INPUT_PULLUP);  // USE INTERNAL PULL-UP
    select_button.attach(BUTTONSelectPin, INPUT_PULLUP);
    TOP_ENDSTOP.attach(EndstopTOPPlungerPin, INPUT_PULLUP);
    BOTTOM_ENDSTOP.attach(EndstopBOTTOMPlungerPin, INPUT_PULLUP);    //Top & Bottom NO, 5v
    BARREL_ENDSTOP.attach(EndstopBARRELClampOKPin, INPUT_PULLDOWN);  //FINDA sensor is NC, 0v
    EMERGENCY_STOP.attach(EMERGENCYStopPin, INPUT_PULLUP);    // CHECK THAT IS NO!!

    // DEBOUNCE INTERVAL IN MILLISECONDS
    down_button.interval(DEBOUNCE_INTERVAL);
    up_button.interval(DEBOUNCE_INTERVAL);
    select_button.interval(DEBOUNCE_INTERVAL);
    TOP_ENDSTOP.interval(DEBOUNCE_INTERVAL);
    BOTTOM_ENDSTOP.interval(DEBOUNCE_INTERVAL);
    BARREL_ENDSTOP.interval(DEBOUNCE_INTERVAL);
    EMERGENCY_STOP.interval(DEBOUNCE_INTERVAL);

    // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
    down_button.setPressedState(LOW);
    up_button.setPressedState(LOW);
    select_button.setPressedState(LOW);
    TOP_ENDSTOP.setPressedState(LOW);  // NO, with pullup, goes LOW when closed
    BOTTOM_ENDSTOP.setPressedState(LOW);
    BARREL_ENDSTOP.setPressedState(HIGH);  //  NC, with pulldown, goes HIGH when active
    EMERGENCY_STOP.setPressedState(LOW);   //  CHECK THAT IS NO!!
  }



  void setup() {
    Serial.begin(115200);
    Serial.println("injector_ESP32_state_machine.ino");

    // encoder setup
    encoder.attachHalfQuad(ENCODERApin, ENCODERBpin);  // possible have to reverse..? or rename, for Encoder library "CLK, DT ", and also rename EncoderPins..?
    // encoder.setCount ( 0 );  move this line to INIT_HOMED_ENCODER_ZEROED function?

    // stepper setup
    //#define DRIVER_RMT 1  // type of driver for FAS, to separate any motor interference, not needed as directly used "1" below..?
    engine.init(1);                                           //   Core assignment
    stepper = engine.stepperConnectToPin(StepperSTEPPin, 1);  // "1" defines DRIVER_RMT, for encoder to use PCNT, not Stepper
                                                              /*  refere to https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md
 for ways to CPU_CORE & DRIVER_TYPE, if this does not compile as-is, and 
 https://valarsystems.com/blogs/val-2000/section-9-dual-core-setup for alternative CPU_CORE method
 */
    if (stepper) {
      stepper->setDirectionPin(StepperDIRPin);  // possible not needed if moveTo commands are also -ve..?
    }

    // thermocouple setup
    thermocouple.begin();

    // keypadLEDs setup
    keypadleds.begin();  // Call this to start up the LED strip.
    //  clearLEDs();   // This function, defined below, turns all LEDs off...
    keypadleds.show();  // ...but the LEDs don't actually update until you call this.

    // Buttons & Endstops Bounce2 setup is very long, so called from the following function
    BounceButtonsSetup();
  }

  void loop() {
    CheckMinTempEndstopsEmergencyStopPin();
    MachineState(int runState);
  }

/*
  // functions declarations, to try and organize most commonly re-used functions
  long EncoderActualPosition();                                             // reads encoder position and returns REFERENCE variable for use by other functions
  void ResetEncoderZero();                                                 // resets Encoder to Zero, ONLY to be called ONCE at power up, toggle initialHomingDone
  void ProgrammedMotorMove(motorSpeed, motorMove, motorAcceleration);      // simply moves motor at set speed, accel and distance indicated
  void ContinuousMotorMoveForward(motorSpeed);                             // simply moves plunger continously (during button press or other condition is 0/1)
  void ContinuousMotorMoveBackward(motorSpeed);                            // simply moves plunger continously (during button press or other condition is 0/1)
  void HomeMove(GeneralFastSpeed, HomingSlowSpeed);                                 // simply sends plunger to TopEndstop - CAN USE ContinuousMotorMoveBackward
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

  // Encoder function will get called not every loop, but maybe every 10ms, to give time for max distances to trigger other code, and every 100ms
  // wil be called by SerialRegular100msMessages, where if motor is stopped, actual = old, and no need to print value
  long EncoderActualPosition() {
    actualENPosition = encoder.getCount() / 2;  // revise last int, if motor steps = 400 & encoder PPR = 1000, adjust this int
                                                // (possible issue of needing to round, this is bad, better maybe to increase driver steps to same as encoder?)
    oldENPosition = actualENPosition;           // if this does NOT change between Serial.prints, then don't print

    return (actualENPosition);
  }

  void ResetEncoderZero() {
    encoder.clearCount();

    return;
  }

  // motor functions
  /* NOTE: When calling this function from different states, it will be called with the arguments particular to that state, ex.
INJECT will pass ProgrammedMotorMove(FillMouldMoveSpeed, FillMouldAccel, FillMouldMoveDistSteps)... will this AUTOMATICALLY write those 
parameters to the functions named paramters, ie motorSpeed, motorAcceleration, motorMove..? Because if I wish to do a later
compareMotorEncoder function, this function needs to have the ProgrammedMotorMove NAMED params, not the PASSED params names..?
 */
  void ProgrammedMotorMove(motorSpeed, motorAcceleration, motorMove)  // RELATIVE motor move to new position
  {
    stepper->setSpeedInHz(motorSpeed);
    stepper->setAcceleration(motorAcceleration);
    stepper->move(motorMove);
  }


  void ContinuousMotorMoveForward(motorSpeed)  // CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
  {
    stepper->setSpeedInHz(motorSpeed);
    stepper->runForward();
  }

  void ContinuousMotorMoveBackward(motorSpeed)  // CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
  {
    stepper->setSpeedInHz(motorSpeed);
    stepper->runBackward();
  }

  int HomingSlowSpeed = maxSpeedLimit / 10;  // how slow to home, 2nd approach, continous move until reaching endstop
  void HomeMove(GeneralFastSpeed, HomingSlowSpeed)  // once called, no user action can stop until complete!
  {
    //int HomingDirection;      // maybe not needed, as using ContMoveBackwards
    int HomeOffSetDistSteps = 212;             // once first reached HomeEndstop, how much to back off before slower approach, 212 steps ≈ 5mm
    int HomeOffSetAccel = 10000;               // once first reached HomeEndstop, how much Accel to back off before slower approach 10000 = 1/5th normal
    int HomeOffsetSpeed = maxSpeedLimit / 2;   // once first reached HomeEndstop, how quick to back off before slower approach
    int HomingSlowSpeed = maxSpeedLimit / 10;  // how slow to home, 2nd approach, continous move until reaching endstop
    if (digitalRead(EndstopTOPPlungerPin) == !topPlungerEndstopActive) {
      ContinuousMotorMoveBackward(GeneralFastSpeed);  // OR ProgrammedMotorMove with maxHomingStepsDistSteps, if fails, gives error?
    }
    ProgrammedMotorMove(HomeOffsetSpeed, HomeOffSetAccel, HomeOffSetDistSteps);  // programmed motor move assumed to be positive DistSteps..?
    if (digitalRead(EndstopTOPPlungerPin) == !topPlungerEndstopActive) {
      ContinuousMotorMoveBackward(HomingSlowSpeed);
    }
  }

  void RefillPositionMove(GeneralFastSpeed/2, refillOpeningOffsetDistSteps, motorAcceleration )
  {
    ProgrammedMotorMove(GeneralFastSpeed/2, refillOpeningOffsetDistSteps, motorAcceleration );
  }

  // function to compare Motor & Encoder positions
  bool compareMotorEncoder(motorMove, actualENPosition, comparisonPercentage)  // chatGPT written
  {
    // Calculate difference between motor position and encoder position
    int motorError = abs(motorMove - actualENPosition);

    // Calculate the acceptable difference threshold
    int threshold = (motorMove * comparisonPercentage) / 100;

    // Return true if the difference is greater than the threshold
    return (motorError >= threshold);
  }

  /* motor compression function takes compareMotorEncoder and motor moves to apply pressure up to a threshold and then
reduce the speed (increases torque of stepper?) until again threshold past, etc, until minCompressionSpeed reached, thenwill stop */
  void compressionFunction(initialCompressionSpeed, minCompressionSpeed)  // chatGPT written
  {
    ButtonLEDsColors(, BLACK_rgb, );
    select_button.update();

    // add ButtonSelect interrupt function on user press, goes to Ready_to_Inject (also EStop..? or EStop better permament in loop
    //  and then goes to ERROR-STATE?)
    if (!select_button.isPressed) {
      // Set initial speed
      stepper->setSpeedInHz(initialCompressionSpeed);  // Starting speed in steps per second

      while (stepper->getSpeed() > minCompressionSpeed) {
        // Move the plunger down
        stepper->runForward();

        // Call the comparison function to check if motor-encoder difference exceeds threshold
        if (compareMotorEncoder(stepper->getTargetPosition(), encoder.getPosition(), comparisonPercentage)) {
          // Reduce speed when the difference is greater than the allowed threshold
          int newSpeed = stepper->getSpeed() * (speedReductionFactor / 10);
          stepper->setSpeedInHz(newSpeed);
          ButtonLEDsColors(, BLACK_rgb, YELLOW_rgb);  // to show compression is acting (has reduced speed at least once) and ongoing..

          // Optionally, reduce comparison threshold on each iteration (for finer control)
          comparisonPercentage *= speedReductionFactor; /* could lead to infinite smaller steps if minCompressionSpeed is too low...? 
          for debugging this option, would be good to show actual speed - if later wish to add another compressionDuringHeating function
          where the plastic is already quite compressed, and will start (and end) from/to a much lower speeds, have to make sure this 
          does not produce infinite loop as never reaches minCompressionSpeed */
        }
      }

      // Stop motor when speed falls below threshold
      stepper->stopMove();
      ButtonLEDsColors(, BLACK_rgb, GREENrgb);  // to show compression is complete
      delay(500);                           // ONLY DELAY IN ENTIRE SKETCH, and is only to give time for GREENrgb LED to be shown before

      return;  // once finished function should return to function that called it, be it compression or inject.. should be break..?
    } else {
      stepper->stopMove();
      StatesMachine::READY_TO_INJECT;
    }
  }


  void clearLEDs() {
    for (int i = 0; i < keypadLedCount; i++) {
      keypadleds.setPixelColor(i, 0);
    }
  }


  void ButtonLEDsColors(char selectLEDcolour, char upLEDcolour, char downLEDcolour )  //could use case/switch instead? but are different types of defining behaviours..?
  {
    BounceButtonsUpdateCall();
    if (down_button.isPressed() || up_button.isPressed() || select_button.isPressed())  // set brightness to about 80% on any button press
    {
      keypadleds.setBrightness(ledHBrightness);
      keypadleds.setPixelColor(0, selectLEDcolour);
      keypadleds.setPixelColor(1, upLEDcolour);
      keypadleds.setPixelColor(2, downLEDcolour);
      keypadleds.show();
    } else  // set brightness to about 30% when no button pressed
    {
      keypadleds.setBrightness(ledLBrightness);
      keypadleds.setPixelColor(0, selectLEDcolour);
      keypadleds.setPixelColor(1, upLEDcolour);
      keypadleds.setPixelColor(2, downLEDcolour);
      keypadleds.show();
    }

    return;
  }



  // Comms functions, every 100ms send data via serial
  void SerialRegular100msMessages() {
    if (millis() - 100 >= printTime)  // only when 100ms has past...
    {
      actualENPosition = encoder.getCount() / 2;
      if (actualENPosition != oldENPosition) /* ... and actualENPosition & oldENPosition are updated by EncoderActualPosition, but if there
     there has been no change in last 100ms, then do not print new position... possibly this should be divided in loop..? */
      {
        Serial.print("Encoder: ");
        Serial.println(actualENPosition);
        oldENPosition = actualENPosition;
      }
      Serial.print("TempBox: ");
      Serial.println(thermocouple.readInternal());
      Serial.print("TempNozzle: ");
      Serial.println(thermocouple.readCelsius());
      printTime = millis();
    }

    return;
  }


  void SerialPrintNextInjectionValues() {
    Serial.println("Next injection values:");
    // add here struct of variablePerMouldParams

    return;
  }


  void BounceButtonsUpdateCall() {
    down_button.update();
    up_button.update();
    select_button.update();
    BARREL_ENDSTOP.update();
    TOP_ENDSTOP.update();
    BOTTOM_ENDSTOP.update();

    EMERGENCY_STOP.update();
  }

  int CheckMinTempEndstopsEmergencyStopPin() /* should run in each loop, or at least every 10ms, and if any endstop or emergency stop is triggered, stop machine                              
and report which endstop has been triggered... possible thereafter only allow moves in opposite direction, or back to refill, or 
other options..? */
  {
    tempNozzleDegrees = thermocouple.readCelsius();  // digitalRead, maybe every 10ms, state of temo, compare to minTemp, EmergencyStopPin, all ENdstops, etc, and if any are active, give error
    if (tempNozzleDegrees <= minTempForAnyMove)      // and stop machine
    {
      runState = StatesMachine::ERROR_STATE;
      return 1;
    }
    BounceButtonsUpdateCall();
    else if (BARREL_ENDSTOP.isPressed())
    {
      runState = StatesMachine::ERROR_STATE;
      return 2;
    }
    else if (TOP_ENDSTOP.isPressed())
    {
      runState = StatesMachine::ERROR_STATE;
      return 3;
    }
    else if (BOTTOM_ENDSTOP.isPressed())
    {
      runState = StatesMachine::ERROR_STATE;
      return 4;
    }
    else if (EMERGENCY_STOP.isPressed())
    {
      runState = StatesMachine::ERROR_STATE;
      return 5;
    }
    else {
      break;  // or should be "return"..?
    }
  }




// StatesMachine, states that the machine may be in at any point in time

enum StatesMachine {
  ERROR_STATE,                // SOME TYPE OF CHECK TRIGGERED
  INIT_HEATING,               // INITIAL POWER ON STATES
  INIT_HOT_NOT_HOMED,         // INITIAL POWER ON STATES
  INIT_HOMED_ENCODER_ZEROED,  // INITIAL POWER ON STATES
  REFILL,                     // DEFAULT WAITING STATES
  COMPRESSION,                // DEFAULT WAITING STATES
  READY_TO_INJECT,            // DEFAULT WAITING STATES
  PURGE_ZERO,                 // INJECT PROCESS STATES, normally will start here...
  ANTIDRIP,                   // INJECT PROCESS STATES,
  INJECT,                     // INJECT PROCESS STATES,
  HOLD_INJECTION,             // INJECT PROCESS STATES,
  RELEASE,                    // INJECT PROCESS STATES,
  CONFIRM_MOULD_REMOVAL       // INJECT PROCESS STATES, ... and end here before returning to Refill
                              // or READY_TO_INJECT, depending on EndOfDayFlag
};

enum StatesMachine runState;  // delaring variable runState can only have valid values of enum

  /* notes on MachineState...:
1) "return" in called functions will return back to this function EACH LOOP..? or ONLY when a user input 
finishes the called function..? This would cause a jump to next state..?  
1a) or the CASE statement should provide the user input to pass to the next state 

example, Purge State the user  may press the down/up buton varios times, but until presses Confirm, 
the State is the same and will not change 

to revise, that each State has an exit code line that must be completed (by user or parameter value
of sensor) before being able to change State


2) variables (passed on to called functions) that can be adjusted by user input must be global, but 
variables that will never change (except by re-writing code as are fundamentally incorrect) should 
be only declared within the FUNCTION that is later called from this general function,... example, 
ProgrammedMotorMove is called many times, with distStepsint variable values as per State of machine, so 
will require global variables, but HomeMove will ALWAYS use same (common) variables, so should be
declared within this function locally...
some States will requiere Actual values particular to this cycle, other States maybe Common values that
should never change..? However, it would be good practice to have Common variables also globals as 
SOMETIMES there may be a requirement to change them especially during testing and calibration, 
these local variables may be more convenient to have as globals, and write extra functions to 
be able to change them..? Once running alpha machine, should be able to distStepsiguish more variables
that can be made local to functions, and others that, although Common, may require occasional 
adjustment.. QUESTION: can local varibles of a function be updated via Serial writes..? Useful...
 */



void MachineState(int runState)  //
{
  //static StatesMachine runState;  // useful here, or to make runState a Static variable between loops
  // maybe can add "static" to above enum declaration...?

  switch (runState) 
  {
    case StatesMachine::ERROR_STATE:  // function that includes stopMove(), flashes LEDs red (maybe with
      stepper->stopMove();
      switch (errorReason) 
      {
        case errorReason = 1:
          Serial.println("Temp too low for machine move, check heating system");
            ButtonLEDsColors(REDrgb, REDrgb,REDrgb );
          break;

        case errorReason = 2:
          Serial.println("Barrel Endstop indicates Barrel has slipped, please turn off machine, & reposition");
            ButtonLEDsColors(REDrgb, REDrgb, REDrgb);
          delay(500);
          ButtonLEDsColors(REDrgb, 0, REDrgb);
          delay(500);
          break;

        case errorReason = 3:
          Serial.println("Top Endstop triggered, this should not be possibe, check endstop wiring if plungerreally has not reached max height");
            ButtonLEDsColors(REDrgb, REDrgb, REDrgb);
          delay(500);
          ButtonLEDsColors(0, REDrgb, REDrgb);
          delay(500);
          break;

        case errorReason = 4:
          Serial.println("Bottom Endstop triggered, no more plastic left to inject, please Home or Refill");
            ButtonLEDsColors(REDrgb, REDrgb, REDrgb);
          delay(500);
          ButtonLEDsColors(REDrgb, REDrgb, 0);
          delay(500);
          break;

        case errorReason = 5:
          ProgrammedMotorMove(ReleaseMouldMoveSpeed, motorAcceleration, ReleaseMouldMoveDistSteps);
            Serial.println("Emergency Endstop triggered, please revise reason, fix, and de-activate");
              ButtonLEDsColors(REDrgb, REDrgb, REDrgb);
          delay(500);
          ButtonLEDsColors(0, 0, 0);
          delay(500);
          break;
      }

      break;

    case StatesMachine::INIT_HEATING:
      delay(500);  // delay to give MAX31855 time to initialize and stabilize
      tempNozzleDegrees = thermocouple.readCelsius();
      if (tempNozzleDegrees <= minTempForAnyMove) 
      {
        ButtonLEDsColors(, , );
        // send every 500ms the actual temp via serial
      } 
      else 
      {
        runState = StatesMachine::INIT_HOT_NOT_HOMED;
      }
      break;

    case StatesMachine::INIT_HOT_NOT_HOMED:
      ButtonLEDsColors(YELLOW_rgb, YELLOW_rgb, YELLOW_rgb);  // set button LEDs to all YELLOW_rgb, user can now press center/up button to start Homing process, for first time
      up_button.update();
      UpPinState = up_button.read();
      if (UpPinState == LOW && up_button.currentDuration() > buttonShortPress) 
      {

        /*while (digitalRead(BUTTONUpPin)==LOW) // user starts to press Homing button...
        {
          long unsigned int button_millis = millis;
          if ((digitalRead(BUTTONUpPin)==LOW)  && (button_millis >= millis+buttonShortPress))  // ... continous holding for ShortPress time...
          {  */

        HomeMove(GeneralFastSpeed, HomingSlowSpeed);          // ... but once sees Homing sequence start, should let go of button..? whatif doesn't..?
        runState = StatesMachine::INIT_HOMED_ENCODER_ZEROED;  // should only change state when HomeMove has been finished, && encoder set to 0
      }
      //}

      break;

    case StatesMachine::INIT_HOMED_ENCODER_ZEROED:
      ResetEncoderZero();  // should only set Encoder to 0 when HomeMove has done RETURN (finished)
      stepper->setCurrentPosition(0);
      ProgrammedMotorMove(GeneralFastSpeed, motorAcceleration, refillOpeningOffsetDistSteps);
      runState = StatesMachine::REFILL;
      break;

    case StatesMachine::REFILL:
      BounceButtonsUpdateCall();
      if (endOfDayFlag == 0) 
      {
        ButtonLEDsColors(GREENrgb, BLACK_rgb, BLACK_rgb);
      } 
      else (endOfDayFlag == 1); 
      {
        ButtonLEDsColors(GREENrgb, BLUE_rgb, BLUE_rgb);
      }
      while (digitalRead(BUTTONUpPin) == LOW) &&(digitalRead(BUTTONDownPin) == LOW)
      // while ((up_button.isPressed) && (down_button.isPressed))     // better to use Bounce2 library methods..?
      {
        long unsigned int button_millis = millis;
        if ((digitalRead(BUTTONUpPin) == LOW) && (digitalRead(BUTTONDownPin) == LOW) && ((button_millis + buttonShortPress) >= millis))
          // if ((up_button.isPressed) && (down_button.isPressed) && ((button_millis + buttonShortPress) >= millis))  // can combine Bounce2 methods with millis..?
          // would seem shorter than having to .read to a variable, then logic && variable & currentDuration of EACH button..?
        {
          endOfDayFlag = !endOfDayFlag;
        }
      }
      while (digitalRead(BUTTONSelectPin==LOW))
      {
        nozzleBlockPresentEndstopActive = digitalRead(EndstopNOZZLEBlockForCompressOrPurgePin);  // CHECK that EndstopNOZZLEBlockForCompressOrPurgePin is PRESENT before
        if (nozzleBlockPresentEndstopActive = 0)                                           // allowing next State compression, otherwise plastic will be extruded from tip!
        {
          long unsigned int button_millis = millis;
          if (digitalRead(BUTTONSelectPin == LOW) && ((button_millis + buttonShortPress) >= millis)) 
          {
            // relative move from Refill position
            ProgrammedMotorMove((GeneralFastSpeed / 2), (motorAcceleration / 2), (heatedZoneOffsetDistSteps - refillOpeningOffsetDistSteps));
            runState = StatesMachine::COMPRESSION;
          }
        }
        else
        {
          Serial.print("The Nozzle block is not in place, cannot proceed until nozzle block in place");
        }
      }
        break;

      case StatesMachine::COMPRESSION:
        ButtonLEDsColors(, BLACK_rgb, );
        compressionFunction(initialCompressionSpeed, minCompressionSpeed);
        // compare totalStrokeSteps to amountRecordedDuringCompresion, and if less than ex 80%, go back to Refill as barrel is not full - or ignore this (flag)
        runState = StatesMachine::READY_TO_INJECT;

        break;

      case StatesMachine::READY_TO_INJECT:
      ButtonLEDsColors (GREENrgb, YELLOW_rgb, GREENrgb)
      while (digitalRead(BUTTONUpPin==LOW)
      {
            long unsigned int button_millis = millis;
            if (digitalRead(BUTTONUpPin == LOW) && (button_millis + buttonLongPress >= millis)) {
              runState = StatesMachine::REFILL;
            }
      }
      while (digitalRead(BUTTONSelectPin)==LOW) && (digitalRead(BUTTONDownPin)==LOW)
      {
            long unsigned int button_millis = millis;
            if ((digitalRead(BUTTONSelectPin) == LOW) && (digitalRead(BUTTONDownPin) == LOW) && ((button_millis + buttonShortPress) >= millis)) {
              runState = StatesMachine::PURGE_ZERO;
            }
      }

      break;


    case StatesMachine::PURGE_ZERO:
      ButtonLEDsColors (GREENrgb, YELLOW_rgb, YELLOW_rgb);
      down_button.update();
      up_button.update();
      select_button.update();
      DownPinState = down_button.read();
      UpPinState = up_button.read();
      SelectPinState = select_button.read();

      if ((DownPinState == 0 && UpPinState == 0) || (DownPinState == 1 && UpPinState == 1))   // whilst NO button press or BOTH 
      {                                                                           // button press, NO MOVE and set stepper position to 0
        stepper->stopMove();
        stepper->setCurrentPosition(0);
      }
      else if (DownPinState == 0 && UpPinState == 1)       // if DOWN button pressed, move continuous down until button released
      {
        ContinuousMotorMoveForward(PurgeSpeed);
      }
      else if (DownPinState == 1 && UpPinState == 0)    // if UP button pressed, move continuos up until button released
      {
        ContinuousMotorMoveBackward(PurgeSpeed);
      }
      else if (SelectPinState == 0)
      {
        long unsigned int button_millis = millis;
        if ((SelectPinState == 0) && ((button_millis + buttonShortPress) >= millis))  // if SELECT button pressed 1/2s, go to next state
        {                                                                           // as no other buttons are pressed, stepper position is already set to 0
          stepper->setCurrentPosition(0);
          runState = StatesMachine::ANTIDRIP;
        }
      }

      break;

    case StatesMachine::ANTIDRIP:
      ButtonLEDsColors (, GREENrgb, GREENrgb);
      Serial.print("You have ");
      Serial.print(AntiDripReverseTime/1000);
      Serial.println("s to place mould, adjust platform, and press both Green buttons to proceed with Injection")
      /*down_button.update();
      up_button.update();
      select_button.update();
      DownPinState = down_button.read();
      UpPinState = up_button.read();
      SelectPinState = select_button.read();
      
      moved inside "while" loop so that the button presses are looped, if they are above, they will only be updated once, before "while"..?
      */

      AntiDripReverseMillis = millis();
      
      while (millis() - AntiDripReverseMillis <= AntiDripReverseTime)
      {
        ContinuousMotorMoveBackward(AntiDripReverseSpeed);
        down_button.update();
        up_button.update();
        select_button.update();
        DownPinState = down_button.read();
        UpPinState = up_button.read();
        SelectPinState = select_button.read();

        if ((DownPinState == LOW && down_button.currentDuration() >= buttonShortPress) && (UpPinState == LOW && up_button.currentDuration() >= buttonShortPress)) 
        {
          stepper->stopMove();
          AntiDripFinalOffset = stepper->getCurrentPosition;
          runState = StatesMachine::INJECT;  // do I need a "break" or "return" here to escape the "while" and/or "if" loops..?
          break;                             // this in theory will break out of highest "while" loop, therefore stopping motor and going to next State, or will
                                              // only break out of second while, forcing timer to finish before advancing to next State..?
        } 
        else if (SelectPinState == LOW && select_button.currentDuration() >= buttonShortPress) 
        {
          stepper->stopMove();
          runState = StatesMachine::READY_TO_INJECT;
          Serial.println("Injection Process CANCELED by user, please start a new Purge/ Injection cycle");
          break;
        }
        
      } 
      runState=StatesMachine::READY_TO_INJECT;
      Serial.println("Timeout on Injection Process exceeded, please start a new Purge/ Injection cycle");
      break;
    
      /*  simple continuousMotorMoveBackwards, to avoid drip, but with countdown, after which auto-cancel (lack of user input, or
      simply finishing ContinuousMotorMoveBackward within the millis timeout in while loop), next command is goto ReadyToInject..
      as to avoid too much air / time allows plastic to cool in nozzle tip, so returns to previous State...
      if buttons are pressed within "while" loop, then new State is registered.. should add "break" to immediately stop motor move..? */

      

    case StatesMachine::INJECT:
      ButtonLEDsColors (, GREENrgb, 0);
      Serial.print("Injecting ");
      Serial.print(FillMouldMoveDistSteps/stepsToCM3);
      Serial.println("cm3 of plastic");

      ProgrammedMotorMove(FillMouldMoveSpeed, FillMouldAccel, (FillMouldMoveDistSteps += AntiDripFinalOffset));

      runState=StatesMachine::HOLD_INJECTION;
      break;
     /* CHECK that MOULD is PRESENT in previous State before allowing injection, otherwise plastic will be extruded from tip!
     Barrel will also move, etc
    // per-mould ProgrammedMotorMove to inject with required pressure/speed during time requiered 
    REFINEMENTS: change to include compareMotorEncoder function, to check that mould is full... actually, if using contMotorMove
    & comparison, similar to Compression function, but maybe with other Adjustment parameteres, could maybe detect some/all
    types of mould when are full..? At least 3D moulds, and the slow down could also be adjusted to act as a Hold function, so all in 1!
    */
    

    case StatesMachine::HOLD_INJECTION:
      ButtonLEDsColors (, GREENrgb, GREENrgb);
      Serial.print("Holding pressure, adding ");
      Serial.print(HoldMouldMoveDistSteps/stepsToCM3);
      Serial.println("cm3 of plastic");

      ProgrammedMotorMove(HoldMouldMoveSpeed, FillMouldAccel, HoldMouldMoveDistSteps);

      runState=StatesMachine::RELEASE;
      // per-mould ProgrammedMotorMove to maintain pressure during time requiered, see Inject function, could be a constMotorMove with Compare
      // to combine both stages... instead of a "linear" dropoff in compression speed and distance, could be first of all a new much lower HoldSpeed,
      // then continue with much lower parameters and possibly for timed amount
      break;

    case StatesMachine::RELEASE:
      ButtonLEDsColors (GREENrgb, GREENrgb, GREENrgb);
      Serial.print("Releasing, please lower platform and remove ");
      Serial.print(mouldNow.MouldName);
      Serial.println(" mould");

      ProgrammedMotorMove(ReleaseMouldMoveSpeed, FillMouldAccel, ReleaseMouldMoveDistSteps);

      runState=StatesMachine::CONFIRM_MOULD_REMOVAL;
      // common ProgrammedMotorMove to release pressure and allow mould removal
      // if EndOfDayFlag==1, goto ReadyToInject, if ==0, goto Refill
      break;

    case StateMachine::CONFIRM_MOULD_REMOVAL:
      ButtonLEDsColors (GREENrgb, GREENrgb, GREENrgb);
      down_button.update();
      up_button.update();
      select_button.update();
      DownPinState = down_button.read();
      UpPinState = up_button.read();
      SelectPinState = select_button.read();

      if (DownPinState==LOW || UpPinState==LOW || SelectPinState==LOW)
      {
        if (EndOfDayFlag = 0) 
        {
          HomeMove(GeneralFastSpeed / 2, HomingSlowSpeed);
          stepper->setCurrentPosition(0);
          ProgrammedMotorMove(GeneralFastSpeed, motorAcceleration, refillOpeningOffsetDistSteps);
          runState = StatesMachine::REFILL;
        } 
        else if (EndOfDayFlag = 1) 
        {
          Serial.println("Please place nozzle block to avoid plastic drip until next use of the injector, thank you!");
          runState = StatesMachine::READY_TO_INJECT;
        }
      }
      break;
      // press any button to confirm mould removal, 
      // if EndOfDayFlag==1, goto ReadyToInject, if ==0, goto Refill

  }
}



  void SerialChangeOfState() {
    if (runState != runState) {
      Serial.print("State Machine change to: ");
      Serial.println(runState);
    }

    return;
  }