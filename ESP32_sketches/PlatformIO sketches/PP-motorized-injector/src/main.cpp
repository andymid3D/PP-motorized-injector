#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Bounce2.h>
#include <SafeString.h>

#include "config.h"
#include "injector_fsm.h"
#include "SafetyManager.h"
#include "CanBusHandlerV2.h"
#include "DebugCommands.h"
#include "UI.h"
#include "Homing.h"
#include "BroadcastDataStore.h"
#include "SerialMessaging.h"
#include "MessageBuffer.h"

// --- FSM Global Variables ---
fsm_inputs_t fsm_inputs;
fsm_outputs_t fsm_outputs;
fsm_state_t fsm_state;

// --- Hardware Objects ---
SafetyManager safety;
CanBusHandlerV2 motor;
Adafruit_NeoPixel ledsButtons(LED_COUNT_BUTTONS, PIN_LED_BUTTONS, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledsRing(LED_COUNT_RING, PIN_LED_RING, NEO_GRB + NEO_KHZ800);

// --- Debug Mode ---
// CRITICAL: Only ONE debug mode can be enabled at a time
// Enabling both will cause unpredictable machine behavior
bool debugCommandsEnabled = false;  // Set to false to run normal FSM, true for serial debug commands
bool debugHomingEnabled = true;    // Set to true to debug homing sequence step-by-step

// Runtime check: Ensure mutual exclusion
void validateDebugModes() {
    if (debugCommandsEnabled && debugHomingEnabled) {
        MessageBuffer::getInstance().sendMessage("CRITICAL ERROR: Both debugCommandsEnabled AND debugHomingEnabled are TRUE!");
        MessageBuffer::getInstance().sendMessage("Only ONE debug mode allowed at a time!");
        MessageBuffer::getInstance().sendMessage("Machine will NOT run until this is fixed!");
        while (true) {
            delay(1000);  // Halt machine
        }
    }
}

DebugCommands debugCmds;

// --- Input Debouncers ---
Bounce2::Button btnCenter = Bounce2::Button(); 
Bounce2::Button btnUpper = Bounce2::Button();  
Bounce2::Button btnLower = Bounce2::Button();  

// --- Global Flags & Timers ---
MachineFlags flags = {0, 0, 0, 0}; // Added calibration flag init
unsigned long lastAutoCompress = 0;
unsigned long antiDripTimer = 0;
unsigned long lastDebugTime = 0;
unsigned long lastMotorCmdTime = 0; 
unsigned long bootTime = 0;
unsigned long stateTimer = 0; 
bool tempErrorActive = false; 
bool errorLogged = false; 
bool buttonLock = false; 

// State Management
int lastFsmState = -1;
bool stateEntry = false;
String lastCmdStr = "None";
float injectStartPos = 0.0f; // Track injection start position
float packStartPos = 0.0f;   // Track pack start position
int lastControlMode = -1;    // Track last sent control mode (0=Voltage, 1=Torque, 2=Velocity, 3=Position)
int lastInputMode = -1;      // Track last sent input mode (0=Inactive, 1=Passthrough, 2=VelRamp, 3=PosFilter, 4=TrapTraj, 5=TorqueRamp)

// --- Parameters ---
actualMouldParams_t currentMould = {
    "Default", 
    35.0f,  // Fill Volume (cm3)
    25.0f,  // Fill Speed (RPS)
    20.0f,  // Fill Pressure (Amps)
    5.0f,   // Pack Volume (cm3)
    2.0f,   // Pack Speed (RPS)
    10.0f,  // Pack Pressure (Amps)
    2.0f,   // Pack Time (Sec)
    5.0f    // Cooling Time (Sec)
};

// --- Helper: Log ---
void logMessage(const char* msg) { 
    MessageBuffer::getInstance().sendMessage(msg); 
}

// --- Helper: Volume to Turns ---
float volToTurns(float cm3) {
    return cm3 * TURNS_PER_CM3_VOL;
}

// --- Helper: Read Temp ---
int readThermocouple() {
    static int lastValidTemp = 20; 
    long sum = 0;
    const int samples = 10;
    for(int i=0; i<samples; i++) sum += analogRead(PIN_TEMP_ANALOG);
    float avgAdc = sum / (float)samples;
    float voltage = (avgAdc / 4095.0) * 3.3;
    float currentTemp = voltage / 0.01; 
    
    if (currentTemp < 5.0f) return lastValidTemp;
    static float smoothedTemp = 0;
    if (smoothedTemp == 0) smoothedTemp = currentTemp;
    smoothedTemp = (smoothedTemp * 0.80) + (currentTemp * 0.20);
    lastValidTemp = (int)smoothedTemp;
    return lastValidTemp;
}

// --- Helper: State Names ---
const char* getStateName(int state) {
    switch(state) {
        case ERROR_STATE: return "ERROR_STATE";
        case INIT_HEATING: return "INIT_HEATING";
        case INIT_HOT_NOT_HOMED: return "INIT_HOT_WAIT";
        case INIT_HOMING: return "INIT_HOMING";
        case REFILL: return "REFILL";
        case COMPRESSION: return "COMPRESSION";
        case READY_TO_INJECT: return "READY_TO_INJECT";
        case PURGE_ZERO: return "PURGE_ZERO";
        case ANTIDRIP: return "ANTIDRIP";
        case INJECT: return "INJECT";
        case HOLD_INJECTION: return "HOLD_PACKING";
        case RELEASE: return "RELEASE";
        case CONFIRM_MOULD_REMOVAL: return "CONFIRM_REMOVAL";
        default: return "UNKNOWN";
    }
}

// --- Helper: LEDs ---
void updateLeds() {
    uint32_t colUpper = BLACK_RGB;
    uint32_t colCenter = BLACK_RGB;
    uint32_t colLower = BLACK_RGB;
    uint32_t colRing = BLACK_RGB;
    
    switch (fsm_state.currentState) {
        case ERROR_STATE: if ((millis() / 500) % 2 == 0) { colUpper = RED_RGB; colCenter = RED_RGB; colLower = RED_RGB; colRing = RED_RGB; } break;
        case INIT_HEATING: colUpper = RED_RGB; colCenter = RED_RGB; colLower = RED_RGB; colRing = RED_RGB; break;
        case INIT_HOT_NOT_HOMED: colUpper = YELLOW_RGB; colCenter = YELLOW_RGB; colLower = YELLOW_RGB; colRing = YELLOW_RGB; break;
        case INIT_HOMING: if ((millis() / 500) % 2 == 0) { colUpper = YELLOW_RGB; colRing = YELLOW_RGB; } break;
        case REFILL: colCenter = GREEN_RGB; if (flags.endOfDay) { colUpper = BLUE_RGB; colLower = BLUE_RGB; } else { colUpper = BLACK_RGB; colLower = BLACK_RGB; } colRing = GREEN_RGB; break;
        case COMPRESSION: colUpper = RED_RGB; colCenter = BLACK_RGB; colLower = RED_RGB; colRing = RED_RGB; break; 
        case READY_TO_INJECT: colUpper = GREEN_RGB; colCenter = YELLOW_RGB; colLower = GREEN_RGB; colRing = GREEN_RGB; break; 
        case PURGE_ZERO: colUpper = YELLOW_RGB; colCenter = GREEN_RGB; colLower = YELLOW_RGB; colRing = YELLOW_RGB; break;
        case ANTIDRIP: colUpper = RED_RGB; colCenter = GREEN_RGB; colLower = GREEN_RGB; colRing = RED_RGB; break;
        case INJECT: colUpper = RED_RGB; colCenter = GREEN_RGB; colLower = BLACK_RGB; colRing = RED_RGB; break;
        case HOLD_INJECTION: colUpper = RED_RGB; colCenter = GREEN_RGB; colLower = GREEN_RGB; colRing = RED_RGB; break;
        case RELEASE: colUpper = GREEN_RGB; colCenter = GREEN_RGB; colLower = GREEN_RGB; colRing = GREEN_RGB; break;
        case CONFIRM_MOULD_REMOVAL: colUpper = GREEN_RGB; colCenter = GREEN_RGB; colLower = GREEN_RGB; colRing = GREEN_RGB; break;
    }
    ledsButtons.setPixelColor(0, colUpper); ledsButtons.setPixelColor(1, colCenter); ledsButtons.setPixelColor(2, colLower); ledsButtons.show();
    for(int i=0; i<LED_COUNT_RING; i++) ledsRing.setPixelColor(i, colRing); ledsRing.show();
}

// --- SAFETY WRAPPER ---
void setModeAndMove(int ctrlMode, int inputMode, float value, String cmdName) {
    if ((millis() - lastMotorCmdTime > 20)) {
        motor.setControllerModes((ODriveCANProtocol::ControlMode)ctrlMode, 
                                 (ODriveCANProtocol::InputMode)inputMode);
        if (ctrlMode == 1) motor.setInputTorque(value);      
        else if (ctrlMode == 2) motor.setInputVel(value); 
        else if (ctrlMode == 3) motor.setInputPos(value); 
        lastMotorCmdTime = millis();
        lastCmdStr = cmdName;
        lastControlMode = ctrlMode;  // Track the mode sent
        lastInputMode = inputMode;    // Track the mode sent
    }
}
// NOTE: InputMode mapping for ODrive 0.5.6:
// 0 = INACTIVE (no control)
// 1 = PASSTHROUGH (direct setpoint, no ramp)
// 2 = VEL_RAMP (ODrive handles velocity ramping)
// 3 = POS_FILTER (ODrive handles position filtering)
// 5 = TRAP_TRAJ (trapezoidal trajectory - smooth ramps, PREFERRED for position moves)
// 6 = TORQUE_RAMP (ODrive handles torque ramping)
//
// For position moves: Use InputMode 5 (TRAP_TRAJ) via proper CAN message format

// ===== COMMENTED OUT: Old blocking homing sequence (now using Homing::update() state machine) =====
/*
bool runHomingSequence() {
    static int step = 0;
    static int lastStep = -1;
    static unsigned long stepStartTime = 0;
    
    if (stateEntry) { step = 0; lastStep = -1; } 
    
    bool newStep = (step != lastStep);
    if (newStep) { 
        lastStep = step;
        stepStartTime = millis();
        //retryCount = 0;
        
            // Log step transitions
        switch (step) {
            case 0: logMessage("Home: 0-ClearErr"); break;
            case 1: logMessage("Home: 1-CheckCalib"); break;
            case 2: logMessage("Home: 2-Calibrate(State7)"); break;
            case 3: logMessage("Home: 3-WaitCalib"); break;
            case 4: logMessage("Home: 4-RequestClosedLoop(State8)"); break;
            case 5: logMessage("Home: 5-SetVelocityMode"); break;
            case 6: logMessage("Home: 6-FastRetractUp"); break;
            case 7: logMessage("Home: 7-Decelerate"); break;
            case 8: logMessage("Home: 8-BackoffDown"); break;
            case 9: logMessage("Home: 9-SlowApproach"); break;
            case 10: logMessage("Home: 10-WaitStop"); break;
            case 11: logMessage("Home: 11-ResetEncoder"); break;
            case 12: logMessage("Home: COMPLETE"); return true;
        }
    }
    
    
    uint32_t elapsed = millis() - stepStartTime;

    switch(step) {
        // ===== 0: Clear errors, set safety context =====
        case 0:
            safety.setContext(CTX_MOVING_FREE); 
            motor.clearErrors(); 
            lastCmdStr = "ClearErr";
            if (elapsed > 500) step++;
            break;
            
        // ===== 1: Check if calibration already done =====
        case 1:
            if (flags.calibrationDone) { 
                logMessage("  (Calib skipped, done before)");
                step = 4; 
            } else {
                step++;
            }
            break;
            
        // ===== 2: Request calibration (State 7) =====
        case 2:
            motor.setAxisState(ODriveCANProtocol::AxisState::ENCODER_OFFSET_CALIBRATION);
            lastCmdStr = "State7";
            step++;
            break;
            
        // ===== 3: Wait for calibration to complete (State 1 → 7 → 1) =====
        case 3: {
            uint8_t currentState = motor.getAxisState();
            
            // Track which substep we're in:
            // substep 0 = waiting for State 7 to start
            // substep 1 = waiting for State 1 to indicate completion
            static uint8_t substep = 0;
            static uint8_t lastLoggedState = 255;
            
            if (newStep) { substep = 0; lastLoggedState = 255; }
            
            // Log state changes
            if (currentState != lastLoggedState) {
                lastLoggedState = currentState;
                char buf[64];
                snprintf(buf, sizeof(buf), "  HB: State=%u", currentState);
                logMessage(buf);
            }
            
            if (substep == 0) {
                // Waiting for State 7 (calibration started)
                if (currentState == 7) {
                    logMessage("  (Calibrating, entered State 7)");
                    substep = 1;  // Move to phase 2
                } else if (elapsed > 5000) {
                    logMessage("ERROR: State 7 never reached - aborting");
                    return false;
                }
            } else if (substep == 1) {
                // Waiting for State 1 (calibration finished)
                if (currentState == 1) {
                    flags.calibrationDone = true;
                    logMessage("  (Calib complete, back to State 1)");
                    substep = 0;
                    lastLoggedState = 255;
                    step++;
                } else if (elapsed > 15000) {
                    logMessage("ERROR: Calibration timeout (State 7 never returned to State 1) - aborting");
                    return false;
                }
            }
            break;
        }
            
        // ===== 4: Request Closed Loop (State 8) =====
        case 4: {
            uint8_t currentError = motor.getAxisError();
            uint8_t currentState = motor.getAxisState();
            
            // Log error on change
            static uint8_t lastErrorCode = 0;
            if (currentError != lastErrorCode) {
                lastErrorCode = currentError;
                if (currentError != 0) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "  Err:0x%X", currentError);
                    logMessage(buf);
                }
            }
            
            // Log state on change
            static uint8_t lastState = 255;
            if (currentState != lastState) {
                lastState = currentState;
                char buf[32];
                snprintf(buf, sizeof(buf), "  State=%u", currentState);
                logMessage(buf);
            }
            
            // Request state 8
            if ((elapsed % 200) == 0 || elapsed < 50) {  // Every 200ms, or first time
                motor.clearErrors();
                delay(5);
                motor.setAxisState(ODriveCANProtocol::AxisState::CLOSED_LOOP_CONTROL);
                lastCmdStr = "State8";
            }
            
            // Success: state 8 reached
            if (currentState == 8) {
                lastErrorCode = 0;
                lastState = 255;
                logMessage("  (State 8 achieved)");
                step++;
            }
            // Timeout - log once then fail
            else if (elapsed > 3000) {
                static bool errorLogged = false;
                if (!errorLogged) {
                    logMessage("ERROR: State 8 failed after 3s - aborting");
                    errorLogged = true;
                }
                return false;
            }
            break;
        }
        
        // ===== 5: Fast retract up until top endstop =====
        case 5: {
            static bool modeSwitched = false;
            
            if (newStep) {
                modeSwitched = false;
            }
            
            // First iteration: set control mode to velocity
            if (!modeSwitched) {
                motor.setControllerModes(ODriveCANProtocol::ControlMode::VELOCITY_CONTROL, 
                                        ODriveCANProtocol::InputMode::PASSTHROUGH);
                delay(20);
                modeSwitched = true;
                lastCmdStr = "Mode2";
            } else {
                // Now send velocity commands
                motor.setInputVel(-SPEED_HOMING_FAST);  // UP
                lastCmdStr = "Vel -Fast";
                
                if (safety.isTopEndstopHit()) {
                    motor.setInputVel(0);
                    lastCmdStr = "Stop";
                    modeSwitched = false;
                    step++;
                }
            }
            break;
        }
            
        // ===== 6: Decelerate smoothly =====
        case 6: {
            float elapsed_sec = elapsed / 1000.0f;
            float decelRate = 25.0f;  // 25 turns/sec^2
            float targetVel = -SPEED_HOMING_FAST + (decelRate * elapsed_sec);
            if (targetVel > 0) targetVel = 0;
            
            motor.setInputVel(targetVel);
            lastCmdStr = "Vel Decel";
            
            if (fabs(motor.getVelocity()) < 0.1f || elapsed > 1000) {
                motor.setInputVel(0);
                lastCmdStr = "Stop";
                step++;
            }
            break;
        }
        
        // ===== 7: Backoff down to relax endstop =====
        case 7:
            motor.setInputVel(SPEED_HOMING_SLOW);  // DOWN
            lastCmdStr = "Vel +Slow";
            if (elapsed > 1500) {
                motor.setInputVel(0);
                lastCmdStr = "Stop";
                step++;
            }
            break;
            
        // ===== 8: Slow approach back to top endstop =====
        case 8:
            motor.setInputVel(-SPEED_HOMING_SLOW);  // UP
            lastCmdStr = "Vel -Slow";
            if (safety.isTopEndstopHit()) {
                motor.setInputVel(0);
                lastCmdStr = "Stop";
                step++;
            }
            break;
            
        // ===== 9: Wait for complete stop =====
        case 9:
            setModeAndMove(2, 1, 0, "Vel 0");
            if (fabs(motor.getVelocity()) < 0.05f && elapsed > 500) {
                step++;
            } else if (elapsed > 3000) {
                logMessage("  (Stop timeout)");
                step++;
            }
            break;
            
        // ===== 10: Reset encoder to 0 (confirmed) =====
        case 10:
            if (!flags.initialHomingDone) {
                motor.setLinearCount(0);
                // TODO: Add waitForEncoderReset to V2 later
                delay(50);  // Give motor time to process encoder reset
                if (true) {  // Simplified - skip wait for now
                    logMessage("  (Encoder reset @ 0)");
                    motor.setControllerModes(ODriveCANProtocol::ControlMode::POSITION_CONTROL,
                                            ODriveCANProtocol::InputMode::PASSTHROUGH);
                    delay(5);
                    motor.setInputPos(0.0f);
                    lastCmdStr = "Pos 0";
                    flags.initialHomingDone = true;
                } else {
                    logMessage("  (Encoder reset FAILED)");
                    flags.initialHomingDone = true;
                }
            }
            step++;
            break;
            
        // ===== 11: Done =====
        case 11:
            return true;
    }
    
    return false;
}
*/
// ===== END COMMENTED SECTION =====

// --- Compression Cycle ---
bool runCompressionCycle() {
    static unsigned long compressStart = 0;
    if (stateEntry) {
        logMessage("Compression: Start Torque Ramp");
        safety.setContext(CTX_BLOCKED);
        compressStart = millis();
    }
    float elapsed = (millis() - compressStart) / 1000.0f;
    float targetTorque = (TORQUE_COMPRESSION_HOLD / 2.0f) * elapsed;
    if (targetTorque > TORQUE_COMPRESSION_HOLD) targetTorque = TORQUE_COMPRESSION_HOLD;
    
    setModeAndMove(1, 1, targetTorque, "TorqueMode");

    if (elapsed > 15.0f) { logMessage("Compression: Timeout"); setModeAndMove(2, 1, 0, "Stop"); return true; }
    if (elapsed > 1.0f && abs(motor.getVelocity()) < 0.5f) { logMessage("Compression: Stall Detected"); setModeAndMove(2, 1, 0, "Stop"); return true; }
    return false;
}

// --- Debug Report ---
void printDebugReport() {
    char buf[180];
    long pDisp = safety.getPressure();
    if (pDisp > 999999) pDisp = 999999; if (pDisp < -999999) pDisp = -999999;
    
    // Mode name helpers
    const char* ctrlModeName[] = {"Voltage", "Torque", "Velocity", "Position"};
    const char* inputModeName[] = {"Inactive", "Passthrough", "VelRamp", "PosFilter", "TrapTraj", "TorqueRamp"};
    const char* ctrlStr = (lastControlMode >= 0 && lastControlMode < 4) ? ctrlModeName[lastControlMode] : "None";
    const char* inputStr = (lastInputMode >= 0 && lastInputMode < 6) ? inputModeName[lastInputMode] : "None";
    
    snprintf(buf, sizeof(buf), "[%-16s] T:%-3d P:%-7ld | OD:%d Err:0x%-2X | P:%-5.1f V:%-4.1f | Ctrl:%s Input:%s | Cmd:%s",
        getStateName(fsm_state.currentState), fsm_inputs.nozzleTemperature, pDisp,
        motor.getAxisState(), motor.getAxisError(), motor.getPosition(), motor.getVelocity(), 
        ctrlStr, inputStr, lastCmdStr.c_str());
    MessageBuffer::getInstance().set1HzMessage(buf);
}

void setup() {
    Serial.begin(115200); 
    delay(2000); 
    MessageBuffer::getInstance().sendMessage("SYSTEM START");
    SafeString::setOutput(Serial); 
    SerialMessaging::begin();  // Initialize non-blocking serial messaging
    
    if (debugCommandsEnabled) {
        // DEBUG MODE: Skip FSM initialization, only init DebugCommands
        motor.begin();
        debugCmds.begin(motor);
        MessageBuffer::getInstance().sendMessage("Debug mode activated - FSM disabled");
    } else {
        // NORMAL MODE: Full FSM initialization
        safety.begin(); motor.begin(); pinMode(PIN_TEMP_ANALOG, INPUT);
        ledsButtons.begin(); ledsRing.begin(); ledsButtons.setBrightness(LED_BRIGHT_LOW); ledsRing.setBrightness(LED_BRIGHT_LOW);
    }
    
    // Correct Pin Definitions
    btnUpper.attach(PIN_BTN_UPPER, INPUT_PULLUP); 
    btnCenter.attach(PIN_BTN_CENTER, INPUT_PULLUP); 
    btnLower.attach(PIN_BTN_LOWER, INPUT_PULLUP);
    btnUpper.interval(10); btnCenter.interval(10); btnLower.interval(10);
    
    fsm_state.currentState = InjectorStates::INIT_HEATING;
    bootTime = millis();
}

void loop() {
    motor.loop(); safety.updateInputs(); fsm_inputs.nozzleTemperature = readThermocouple();
    btnCenter.update(); btnUpper.update(); btnLower.update();
    // Bus voltage is received via cyclic broadcast (0x17), no need to poll

    // CRITICAL: Validate that only ONE debug mode is enabled
    validateDebugModes();

    // ===== DEBUG MODE: Bypass FSM entirely =====
    // When debugCommandsEnabled=true, accept serial commands and translate to CAN
    // This allows testing individual motor commands without running full FSM
    if (debugCommandsEnabled) {
        debugCmds.loop();
        return;  // Skip all FSM code when in debug mode
    }
    
    // ===== DEBUG HOMING MODE: Run non-blocking homing state machine =====
    // When debugHomingEnabled=true, this is the ONLY thing the machine does
    if (debugHomingEnabled) {
        // Non-blocking state machine update - processes ONE state per loop
        if (Homing::getState() != Homing::HomingState::IDLE) {
            Homing::update(motor, safety);
        }
        
        // Continuous status output (1Hz) - non-blocking via SerialMessaging
        if (millis() - lastDebugTime >= 1000) {
            lastDebugTime = millis();
            BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
            
            char debugBuf[256];
            snprintf(debugBuf, sizeof(debugBuf), 
                "[HOMING] OD:X=%d | Pos:%.2f Vel:%.2f | ModeSent(C:%d I:%d) | 0x09Rx:%lu | MotPos:%.2f MotVel:%.2f | Temp:%d State:%s",
                broadcast.getAxisState(),
                broadcast.getPosition(), broadcast.getVelocity(),
                Homing::getLastControlMode(), Homing::getLastInputMode(),
                motor.getEncoderEstimatesRxCount(),
                motor.getEncoderEstimates().position, motor.getEncoderEstimates().velocity,
                (int)fsm_inputs.nozzleTemperature,
                Homing::getStateString());
            Serial.println(debugBuf);
        }
        
        // Turn on motor contactor and set ready state LEDs
        safety.enableMotorPower(true);
        
        if (btnUpper.fell()) {  // Upper button pressed
            MessageBuffer::getInstance().sendMessage(">>> UPPER BUTTON PRESSED - Starting homing sequence <<<");
            indicateRunning();  // Yellow LEDs = executing
            
            // Begin state machine (doesn't block)
            Homing::begin(motor, safety);
        }
        
        // Check for completion
        if (Homing::isComplete()) {
            MessageBuffer::getInstance().sendMessage(">>> HOMING COMPLETE <<<");
            indicateReady();  // Green LEDs = success
            MessageBuffer::getInstance().sendMessage("Press upper button to repeat");
            Homing::reset();  // Reset for next attempt
        }
        
        // Check for errors
        if (Homing::hasError()) {
            MessageBuffer::getInstance().sendMessage(">>> HOMING FAILED <<<");
            indicateError();  // Red LEDs = failure
            Homing::reset();
        }
        
        indicateWaiting();  // Blue LEDs = waiting for user input
        return;  // Skip all FSM code when in homing debug mode
    }
    
    // ===== NORMAL FSM OPERATION (when not in debug mode) =====

    if (fsm_state.currentState != lastFsmState) { stateEntry = true; lastFsmState = fsm_state.currentState; stateTimer = millis(); } 
    else { stateEntry = false; }

    // --- BUTTON LOCK LOGIC ---
    if (btnUpper.read() == LOW && btnLower.read() == LOW) buttonLock = true;
    else if (btnCenter.read() == LOW && btnLower.read() == LOW) buttonLock = true;
    if (buttonLock && btnUpper.read() == HIGH && btnLower.read() == HIGH && btnCenter.read() == HIGH) buttonLock = false;

    // --- SAFETY CHECKS ---
    static unsigned long lowTempStart = 0;
    if (fsm_state.currentState != INIT_HEATING && fsm_state.currentState != INIT_HOMING) {
        if (fsm_inputs.nozzleTemperature < TEMP_CRITICAL) {
             if (lowTempStart == 0) lowTempStart = millis();
             if (millis() - lowTempStart > 2000) { if (!tempErrorActive) { safety.triggerHalt(ERR_UNDER_TEMP); fsm_state.currentState = InjectorStates::ERROR_STATE; tempErrorActive = true; } }
        } else { lowTempStart = 0; if (fsm_inputs.nozzleTemperature > (TEMP_CRITICAL + 2)) { tempErrorActive = false; } }
    }

    if (millis() - bootTime > 3000) {
        bool movingDown = motor.getVelocity() > 0.1f;
        if (!safety.check(motor.getVelocity(), movingDown)) { fsm_state.currentState = InjectorStates::ERROR_STATE; fsm_state.error = safety.getLastError(); }
        if (motor.getAxisError() != 0 && fsm_state.currentState != INIT_HOMING && fsm_state.currentState != ERROR_STATE) { fsm_state.currentState = InjectorStates::ERROR_STATE; fsm_state.error = motor.getAxisError(); }
    }

    // --- STATE MACHINE ---
    bool ignoreButtons = (millis() - stateTimer < 500);

    switch (fsm_state.currentState) {
        case InjectorStates::ERROR_STATE:
            setModeAndMove(2, 1, 0, "Stop");
            if (stateEntry) { 
                char errBuf[64];
                snprintf(errBuf, sizeof(errBuf), "ERROR STATE ENTERED: 0x%X", fsm_state.error);
                MessageBuffer::getInstance().sendMessage(errBuf);
                errorLogged = true; 
            }
            if (safety.isEStopPressed() || safety.isBarrelOpen()) safety.enableMotorPower(false); else safety.enableMotorPower(true);
            if (!ignoreButtons && !buttonLock && btnCenter.released()) { 
                if (!safety.isEStopPressed() && !safety.isBarrelOpen()) { logMessage("User Reset."); safety.resetError(); motor.clearErrors(); fsm_state.currentState = InjectorStates::INIT_HEATING; } 
            }
            break;

        case InjectorStates::INIT_HEATING: 
            if (fsm_inputs.nozzleTemperature >= TEMP_CRITICAL) fsm_state.currentState = InjectorStates::INIT_HOT_NOT_HOMED; else safety.enableMotorPower(false);
            break;

        case InjectorStates::INIT_HOT_NOT_HOMED: 
            safety.enableMotorPower(true); 
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                if (fsm_inputs.nozzleTemperature >= TEMP_MIN_MOVE) fsm_state.currentState = InjectorStates::INIT_HOMING; else logMessage("Temp too low!"); 
            }
            break;

        case InjectorStates::INIT_HOMING: {
            // Initialize homing state machine once
            if (stateEntry) {
                Homing::begin(motor, safety);
                stateEntry = false;
            }
            
            // Update non-blocking homing state machine
            Homing::update(motor, safety);
            
            // Check for completion or error
            if (Homing::isComplete()) {
                delay(500);
                setModeAndMove(3, 1, OFFSET_REFILL_GAP, "Pos Refill");
                fsm_state.currentState = InjectorStates::REFILL;
            } else if (Homing::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFF;  // Homing error
            }
            break;
        }

        case InjectorStates::REFILL:
            if (stateEntry) {
                char logBuf[80];
                snprintf(logBuf, sizeof(logBuf), "Refill: Move to %.1f turns", OFFSET_REFILL_GAP);
                logMessage(logBuf);
                setModeAndMove(3, 1, OFFSET_REFILL_GAP, "Pos Refill");
                //motor.setControllerMode(3, 1);     // Mode 3 (Position), InputMode 1 (PASSTHROUGH)
                //motor.setPosition(OFFSET_REFILL_GAP);  // Send target position once
                lastMotorCmdTime = millis();
            }
            
            safety.setContext(CTX_IDLE); 
            if (!ignoreButtons && btnUpper.read() == LOW && btnLower.read() == LOW) { flags.endOfDay = !flags.endOfDay; delay(500); }
            else if (!ignoreButtons && !buttonLock && btnCenter.released()) { fsm_state.currentState = InjectorStates::COMPRESSION; }
            break;

        case InjectorStates::COMPRESSION:
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                logMessage("Compression: User aborted, returning to Refill");
                motor.setInputVel(0);  // CRITICAL: Stop motor immediately
                lastMotorCmdTime = millis();
                fsm_state.currentState = InjectorStates::REFILL; 
            }
            else if (!ignoreButtons && !buttonLock && btnLower.released()) { 
                logMessage("Compression: Complete, ready to inject");
                motor.setInputVel(0);  // CRITICAL: Stop motor immediately
                lastMotorCmdTime = millis();
                fsm_state.currentState = InjectorStates::READY_TO_INJECT; 
                lastAutoCompress = millis(); 
            }
            else if (runCompressionCycle()) { 
                logMessage("Compression: Cycle finished");
                motor.setInputVel(0);  // CRITICAL: Stop motor
                lastMotorCmdTime = millis();
                fsm_state.currentState = InjectorStates::READY_TO_INJECT; 
                lastAutoCompress = millis(); 
            }
            break;

        case InjectorStates::READY_TO_INJECT:
            safety.setContext(CTX_IDLE); 
            if (millis() - lastAutoCompress > TIME_AUTO_COMPRESS) fsm_state.currentState = InjectorStates::COMPRESSION; 
            if (!ignoreButtons && btnUpper.read() == LOW && btnLower.read() == LOW) { fsm_state.currentState = InjectorStates::PURGE_ZERO; }
            else if (!ignoreButtons && !buttonLock && btnCenter.released()) { 
                logMessage("Ready: Returning to Refill");
                motor.setInputVel(0);  // Stop motor
                lastMotorCmdTime = millis();
                fsm_state.currentState = InjectorStates::REFILL; 
            }
            break;

        case InjectorStates::PURGE_ZERO:
            safety.setContext(CTX_PURGE); 
            static bool buttonsReleased = false;
            if (stateEntry) buttonsReleased = false;
            if (!buttonsReleased) { if (btnUpper.read() == HIGH && btnLower.read() == HIGH) { buttonsReleased = true; logMessage("Purge: Buttons Released."); } } 
            else {
                if (btnUpper.read() == LOW) setModeAndMove(2, 1, -SPEED_PURGE, "Purge Up");
                else if (btnLower.read() == LOW) setModeAndMove(2, 1, SPEED_PURGE, "Purge Down");
                else setModeAndMove(2, 1, 0, "Stop");
                if (btnCenter.pressed()) { 
                    logMessage("Purge: Entering AntiDrip");
                    fsm_state.currentState = InjectorStates::ANTIDRIP; 
                    antiDripTimer = millis(); 
                }
            }
            break;

        case InjectorStates::ANTIDRIP:
            {
                safety.setContext(CTX_MOVING_FREE); 
                if (stateEntry) {
                    logMessage("AntiDrip: Decompression (slow retract, 15s timeout)");
                    motor.setControllerModes(ODriveCANProtocol::ControlMode::VELOCITY_CONTROL,
                                            ODriveCANProtocol::InputMode::PASSTHROUGH);
                    lastMotorCmdTime = 0;              // Force immediate command
                }
                // Move UP (negative velocity) to decompress
                setModeAndMove(2, 1, -SPEED_ANTIDRIP, "AntiDrip Vel");
                
                // Check buttons FIRST - allow interrupt at any time
                if (!ignoreButtons && btnCenter.read() == LOW && btnLower.read() == LOW) { 
                    logMessage("AntiDrip: User confirmed, moving to Inject");
                    motor.setInputVel(0);  // Stop
                    lastMotorCmdTime = millis();
                    fsm_state.currentState = InjectorStates::INJECT; 
                }
                else if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                    logMessage("AntiDrip: User abort, returning to Ready");
                    motor.setInputVel(0);  // Stop
                    lastMotorCmdTime = millis();
                    fsm_state.currentState = InjectorStates::READY_TO_INJECT; 
                }
                // Timeout - return to READY_TO_INJECT (don't auto-inject)
                else if (millis() - stateTimer > TIME_ANTIDRIP_TIMEOUT) {
                    logMessage("AntiDrip: Timeout, returning to Ready");
                    motor.setInputVel(0);  // Stop
                    lastMotorCmdTime = millis();
                    fsm_state.currentState = InjectorStates::READY_TO_INJECT; 
                }
            }
            break;

        case InjectorStates::INJECT:
            {
                safety.setContext(CTX_BLOCKED); 
                if (stateEntry) {
                    injectStartPos = motor.getPosition(); // Capture start position
                    motor.setLimits(currentMould.fillSpeed + 5.0f, currentMould.fillPressure);
                    float targetPos = injectStartPos + volToTurns(currentMould.fillVolume);
                    if (targetPos > POS_BOTTOM_MAX) targetPos = POS_BOTTOM_MAX;
                    char logBuf[80];
                    snprintf(logBuf, sizeof(logBuf), "Inject: Start=%.1f Tgt=%.1f Vol=%.1f", injectStartPos, targetPos, currentMould.fillVolume);
                    logMessage(logBuf);
                    motor.setControllerModes(ODriveCANProtocol::ControlMode::POSITION_CONTROL,
                                            ODriveCANProtocol::InputMode::PASSTHROUGH);
                    motor.setInputPos(targetPos);  // Send target position once
                    lastMotorCmdTime = millis();
                }
                if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                    logMessage("Inject: User abort, releasing mould");
                    fsm_state.currentState = InjectorStates::RELEASE; 
                }
                if (millis() - stateTimer > 500 && abs(motor.getVelocity()) < 0.1) { 
                    char logBuf[64];
                    snprintf(logBuf, sizeof(logBuf), "Inject: Done, traveled %.1f turns", motor.getPosition() - injectStartPos);
                    logMessage(logBuf);
                    fsm_state.currentState = InjectorStates::HOLD_INJECTION; 
                }
            }
            break;

        case InjectorStates::HOLD_INJECTION:
            {
                if (stateEntry) {
                    packStartPos = motor.getPosition(); // Capture pack start position
                    motor.setLimits(currentMould.packSpeed + 2.0f, currentMould.packPressure);
                    char logBuf[80];
                    snprintf(logBuf, sizeof(logBuf), "Pack: Start=%.1f Vol=%.1f Time=%.1fs", 
                        packStartPos, currentMould.packVolume, currentMould.packTime);
                    logMessage(logBuf);
                    motor.setControllerModes(ODriveCANProtocol::ControlMode::POSITION_CONTROL,
                                            ODriveCANProtocol::InputMode::PASSTHROUGH);
                    float holdTarget = packStartPos + volToTurns(currentMould.packVolume);
                    if (holdTarget > POS_BOTTOM_MAX) holdTarget = POS_BOTTOM_MAX;
                    motor.setInputPos(holdTarget);  // Send target position once
                    lastMotorCmdTime = millis();
                }
                if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                    logMessage("Pack: User abort, releasing mould");
                    fsm_state.currentState = InjectorStates::RELEASE; 
                }
                if (millis() - stateTimer > (currentMould.packTime * 1000)) {
                    char logBuf[64];
                    snprintf(logBuf, sizeof(logBuf), "Pack: Done, traveled %.1f turns", motor.getPosition() - packStartPos);
                    logMessage(logBuf);
                    fsm_state.currentState = InjectorStates::RELEASE;
                }
            }
            break;

        case InjectorStates::RELEASE: 
            {
                if (stateEntry) {
                    motor.setLimits(VEL_LIMIT_INJECT, 30.0f);
                    motor.setControllerModes(ODriveCANProtocol::ControlMode::POSITION_CONTROL, 
                                            ODriveCANProtocol::InputMode::PASSTHROUGH);
                    float releaseTarget = motor.getPosition() + DIST_RELEASE_MOULD;  // POSITIVE = DOWN, so we ADD to go down and relieve
                    motor.setInputPos(releaseTarget);  // Send target position once
                    lastMotorCmdTime = millis();
                }
                if (millis() - stateTimer > 2000) { 
                    logMessage("Release: Complete, confirming mould removal");
                    motor.setInputVel(0);  // Stop
                    lastMotorCmdTime = millis();
                    fsm_state.currentState = InjectorStates::CONFIRM_MOULD_REMOVAL; 
                }
            }
            break;

        case InjectorStates::CONFIRM_MOULD_REMOVAL:
             if (!ignoreButtons && !buttonLock && (btnCenter.released() || btnUpper.released() || btnLower.released())) { 
                 if(flags.endOfDay) fsm_state.currentState = InjectorStates::READY_TO_INJECT; 
                 else fsm_state.currentState = InjectorStates::REFILL; 
             }
             break;
    }
    updateLeds();
    if (millis() - lastDebugTime > 1000) { 
        lastDebugTime = millis(); 
        printDebugReport();
        // Flush buffered messages: output 1Hz status + accumulated event messages
        const char* output = MessageBuffer::getInstance().getOutput();
        Serial.println(output);
        MessageBuffer::getInstance().clearBuffer();  // Clear events for next cycle
    }
}
