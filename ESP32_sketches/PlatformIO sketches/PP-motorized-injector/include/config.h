#ifndef CONFIG_H
#define CONFIG_H

/* global varible definitions & declaractions, some may be moved to local funtions, but 
leave here and comment out where they have gone - QUESTION: can LOCAL variables within functions be changed via 
serial message updates..? ex: compressionFunction has local speedReductionFactor & comparisonPercentage, during 
initial sketch & machine calibration, it may be necessary to update these values, would be easier via Serial
than flashing...? Although in theory, once checked and sure cannot cause infinite loop, should again be purely 
local and never need changing
 */

#include <Arduino.h>

 // pin definitions, decided on from "GPIO_Limitations_ESP32_NodeMCU.jpg", and which side may be more convenient
 
 #define ENCODER_A_PIN 34                           // * A+ encoder pin, A- not used  "CLK ENCODER "
 #define ENCODER_B_PIN 35                           // * B+ encoder pin, B- not used  "DT ENCODER "
 #define WS2812B_RING_LEDS_PIN  32            // 1 pin for 35 LEDs nozzle ring, to change color as per moment of use..
 #define WS2812B_BUTTON_LEDS_PIN 33         // * 1 pin to address 3 LEDs buttons...
 #define BUTTON_SELECT_PIN 25                            // * these 3 pins for lower keypad, to facilite Purge and INject Confirm
 #define BUTTON_UP_PIN 26                      // *
 #define BUTTON_DOWN_PIN 27                         // *
 #define TEMPNozzleVSPI_SCK_CLK 14               //  *  18 (possible change to analog component if run out of pins
 #define TEMPNozzleVSPI_MISO_DO 12              // * 19
 #define TEMPNozzleVSPI_Dpin_MOSI_CS 13         // * 23 these 3 pins are for daughter PCB thermocouple
 
 //////////////////
 
 #define STEPPER_STEP_PIN 23                        // * any GPIO for ESP32
 #define STEPPER_DIR_PIN 22                         // * may not be needed if using -ve moves..?
 
 #define ENDSTOP_TOP_PLUNGER_PIN 19                     // * all buttons/endstops with INPUT_PULLUP to avoid floating values
 #define ENDSTOP_BOTTOM_PLUNGER_PIN 18                  // * so all ACTIVE STATES are ==0
 #define ENDSTOP_BARREL_PLUNGER_PIN 5                   // *
 #define UART_tx_ESP32 17                         // * TX2 UART comms with ESP32
 #define UART_rx_ESP32 16                         // *
 //#define CURRENTSensor1   4      // * as yet to be decided on how to use motor phase current readng
 #define EMERGENCY_STOP_PIN 0                           //  *
 //#define CURRENTSensor2   2      // * to be useful for detecting motor current (reaching amc compression/fill)
 #define ENDSTOP_NOZZLE_BLOCK_COMPRESS_OR_PURGE_PIN 15  // also as above, not decided hardware method yet
 #define EndstopMOULDPresentPin                    // do not yet have a way of detecting, possibly pressure sensor under plastic floor on platform..?
 
 //////////////////////////////////
 
 // global constants
 // revise what is now NOT NEEDED, OR what variables should go inside Functions to make LOCAL
 
 /* global common variables for machine limits and moves to offsets, these, once identified the max values
 should NEVER need changing, even from advanced Common panel, as superior values run the risk of machine
 performing outside of the decided security limits of movement/accel, distances, etc
 revise what is  NOT NEEDED, OR what variables should go inside Functions to make LOCAL*/
 const int minTempForAnyMove = 20; // FIXME              // min temp considered safe for machine move
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

 ////////////////////

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
// int HomingSlowSpeed;      // how slow to home, 2nd approach, continous move until reaching endstop
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

bool endOfDayFlag = 0;                    // 0 = refillAfterInject, 1= skip and, possibly, measure steps since last top endstop - could read from encoder.. as FA library
                                          // is contemplated to be reset after each purge, so if ==0, reset encoder, ==1 do not reset encoder
bool initialHomingDone = 0;               // on power on, machine needs homing, then mark this flag to show has been done at least once
int errorReason;

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

#endif // CONFIG_H