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
#include "MotorWrapper.h"  // Centralized motor control

// ===== MODULAR STATE MACHINES (Phase 1 Integration) =====
#include "Refill.h"
#include "Compression.h"
#include "Injection.h"
#include "AntiDrip.h"
#include "PurgeZero.h"
#include "ReadyToInject.h"
// ===== END MODULE INCLUDES =====

// ===== ERROR MANAGEMENT (Centralized Logging & Classification) =====
#include "ErrorManager.h"
// ===== END ERROR MANAGEMENT =====

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
bool debugHomingEnabled = false;   // Set to true to debug homing sequence step-by-step

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
// Motor control state tracking moved to MotorWrapper namespace
float injectStartPos = 0.0f; // Track injection start position
float packStartPos = 0.0f;   // Track pack start position

// --- Parameters ---
// Common parameters (used across all moulds, Display-writable)
commonInjectParams_t commonParams = {
    REFILL_TRAP_VEL_LIMIT,        // refillTrapVelLimit (15.0 rps)
    REFILL_ACCEL,                 // refillAccel (20.0 rps²)
    REFILL_DECEL,                 // refillDecel (20.0 rps²)
    COMPRESS_RAMP_TARGET,         // compressRampTarget (15.0 A)
    COMPRESS_RAMP_DURATION,       // compressRampDuration (2.0 sec)
    COMPRESS_MICRO_CURRENT,       // compressMicroCurrent (10.0 A)
    INJECT_FILL_TRAP_VEL_LIMIT,   // injectFillTrapVelLimit (20.0 rps)
    INJECT_FILL_ACCEL,            // injectFillAccel (20.0 rps²)
    INJECT_FILL_DECEL,            // injectFillDecel (20.0 rps²)
    INJECT_FILL_CURRENT,          // injectFillCurrent (31.0 A)
    INJECT_PACK_TRAP_VEL_LIMIT,   // injectPackTrapVelLimit (10.0 rps)
    INJECT_PACK_ACCEL,            // injectPackAccel (10.0 rps²)
    INJECT_PACK_DECEL,            // injectPackDecel (10.0 rps²)
    INJECT_PACK_CURRENT,          // injectPackCurrent (30.0 A)
    INJECT_VEL_THRESHOLD,         // injectVelThreshold (0.1 rps)
    INJECT_POS_TOLERANCE,         // injectPosTolerance (1.0 turns)
    INJECT_STABLE_TIME_MS         // injectStableTimeMs (500 ms)
};

// Mould-specific parameters (per-mould settings, Display-writable)
actualMouldParams_t currentMould = {
    "Default", 
    35.0f,  // Fill Volume (cm3)
    25.0f,  // Fill Speed (RPS)
    20.0f,  // Fill Pressure (Amps)
    5.0f,   // Pack Volume (cm3)
    2.0f,   // Pack Speed (RPS)
    10.0f,  // Pack Pressure (Amps)
    2.0f,   // Pack Time (Sec)
    5.0f,   // Cooling Time (Sec)
    REFILL_ACCEL,  // Fill Trap Accel (default)
    REFILL_DECEL,  // Fill Trap Decel (default)
    10.0f,    // Pack Trap Accel (slower, more controlled)
    10.0f     // Pack Trap Decel (slower, more controlled)
};

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
        case CONFIRM_MOULD_REMOVAL: colUpper = GREEN_RGB; colCenter = BLACK_RGB; colLower = GREEN_RGB; colRing = GREEN_RGB; break;  // Center OFF
    }
    
    // Brightness control: HIGH when any button pressed, LOW otherwise
    uint8_t brightness = (btnUpper.read() == LOW || btnCenter.read() == LOW || btnLower.read() == LOW) ? LED_BRIGHT_HIGH : LED_BRIGHT_LOW;
    ledsButtons.setBrightness(brightness);
    ledsRing.setBrightness(brightness);
    
    ledsButtons.setPixelColor(0, colUpper); ledsButtons.setPixelColor(1, colCenter); ledsButtons.setPixelColor(2, colLower); ledsButtons.show();
    for(int i=0; i<LED_COUNT_RING; i++) ledsRing.setPixelColor(i, colRing); ledsRing.show();
}

// --- MOTOR CONTROL WRAPPERS ---
// All motor commands now go through MotorWrapper namespace (see MotorWrapper.h/cpp)
// This section intentionally empty - functions moved to shared MotorWrapper for module access
// NOTE: InputMode mapping for ODrive 0.5.6:
// 0 = INACTIVE (no control)
// 1 = PASSTHROUGH (direct setpoint, no ramp)
// 2 = VEL_RAMP (ODrive handles velocity ramping)
// 3 = POS_FILTER (ODrive handles position filtering)
// 5 = TRAP_TRAJ (trapezoidal trajectory - smooth ramps, PREFERRED for position moves)
// 6 = TORQUE_RAMP (ODrive handles torque ramping)
//
// For position moves: Use InputMode 5 (TRAP_TRAJ) via proper CAN message format


// --- Compression Cycle ---
bool runCompressionCycle() {
    static unsigned long compressStart = 0;
    if (stateEntry) {
        logMessage("Compression: Start Torque Ramp");
        safety.setContext(CTX_BLOCKED);
        compressStart = millis();
    }
    float elapsed = (millis() - compressStart) / 1000.0f;
    float targetTorque = (COMPRESS_RAMP_TARGET / 2.0f) * elapsed;
    if (targetTorque > COMPRESS_RAMP_TARGET) targetTorque = COMPRESS_RAMP_TARGET;
    
    MotorWrapper::setModeAndMove(motor, 1, 1, targetTorque, "TorqueMode");

    if (elapsed > 15.0f) { logMessage("Compression: Timeout"); MotorWrapper::setModeAndMove(motor, 2, 1, 0, "Stop"); return true; }
    if (elapsed > 1.0f && abs(motor.getVelocity()) < 0.5f) { logMessage("Compression: Stall Detected"); MotorWrapper::setModeAndMove(motor, 2, 1, 0, "Stop"); return true; }
    return false;
}

// --- Debug Report ---
void printDebugReport(unsigned long currentLoopTime, unsigned long maxLoopTimeSinceLastReport) {
    char buf[256];
    long pDisp = safety.getPressure();
    // Display cap removed to observe full EMI range
    
    // Get current measurements for contact detection
    const ODriveCANProtocol::CyclicIq& iq_data = motor.getIq();
    float iq_setpoint = iq_data.Iq_setpoint;
    float iq_measured = iq_data.Iq_measured;
    
    // Get numeric mode/input values
    int lastControlMode = MotorWrapper::getLastControlMode();
    int lastInputMode = MotorWrapper::getLastInputMode();
    String lastCmdStr = MotorWrapper::getLastCommand();
    
    // Calculate uptime in seconds
    unsigned long uptimeSeconds = millis() / 1000;
    
    BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
    uint32_t motorErr = broadcast.getMotorError();
    uint32_t encoderErr = broadcast.getEncoderError();
    uint32_t controllerErr = broadcast.getControllerError();
    
    // Get CAN queue depth for debugging (0-8)
    uint8_t queueDepth = motor.getQueueDepth();
    
    snprintf(buf, sizeof(buf), "[%lus c:%lums m:%lums][[%-12s] T:%-3d P:%-7ld Q:%d OD:%d MX:0x%-2X EX:0x%-2X CX:0x%-2X P:%-5.1f V:%-4.1f IqS:%-4.1f IqM:%-4.1f C:%d I:%d Cmd:%s]",
        uptimeSeconds, currentLoopTime, maxLoopTimeSinceLastReport,
        getStateName(fsm_state.currentState), fsm_inputs.nozzleTemperature, pDisp, queueDepth,
        motor.getAxisState(), motorErr, encoderErr, controllerErr, motor.getPosition(), motor.getVelocity(), 
        iq_setpoint, iq_measured,
        lastControlMode, lastInputMode, lastCmdStr.c_str());
    MessageBuffer::getInstance().set1HzMessage(buf);
}

void setup() {
    Serial.begin(115200); 
    delay(2000); 
    MessageBuffer::getInstance().sendMessage("SYSTEM START");
    SafeString::setOutput(Serial); 
    SerialMessaging::begin();  // Initialize non-blocking serial messaging
    MotorWrapper::init();  // Initialize motor wrapper tracking variables
    
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
    // ===== LOOP TIMING INSTRUMENTATION =====
    unsigned long loopStart = millis();
    static unsigned long lastLoopReportTime = 0;
    static unsigned long maxLoopTime = 0;
    static unsigned long loopTime = 0;  // Current loop time, accessible to printDebugReport
    
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

    // --- MOVEMENT LOCK (Position Move Safety) ---
    // Locks button handlers during critical position moves (e.g., returning to Refill)
    // Prevents accidental state changes while motor is moving to target position
    static bool moveLockActive = false;

    // --- SAFETY CHECKS ---
    static unsigned long lowTempStart = 0;
    // --- Temperature Safety Check ---
    // TEMP CHECK DISABLED: EMI causing false readings - will be re-enabled after hardware fix
    /*
    if (fsm_state.currentState != INIT_HEATING && fsm_state.currentState != INIT_HOMING) {
        if (fsm_inputs.nozzleTemperature < TEMP_CRITICAL) {
             if (lowTempStart == 0) lowTempStart = millis();
             if (millis() - lowTempStart > 2000) { if (!tempErrorActive) { safety.triggerHalt(ERR_UNDER_TEMP); fsm_state.currentState = InjectorStates::ERROR_STATE; tempErrorActive = true; } }
        } else { lowTempStart = 0; if (fsm_inputs.nozzleTemperature > (TEMP_CRITICAL + 2)) { tempErrorActive = false; } }
    }
    */

    if (millis() - bootTime > 3000) {
        bool movingDown = motor.getVelocity() > 0.1f;
        if (!safety.check(motor.getVelocity(), movingDown)) { fsm_state.currentState = InjectorStates::ERROR_STATE; fsm_state.error = safety.getLastError(); }
        
        // ===== CENTRALIZED ERROR CHECKING & RECOVERY (New Architecture) =====
        // Check for ODrive errors (any non-zero error code)
        uint32_t axisErr = motor.getAxisError();
        uint32_t motorErr = motor.getMotorErrorDetails().motor_error;
        uint32_t encoderErr = motor.getEncoderErrorDetails().encoder_error;
        uint32_t controllerErr = motor.getControllerErrorDetails().controller_error;
        
        if (hasAnyError(axisErr, motorErr, encoderErr, controllerErr) && 
            fsm_state.currentState != INIT_HOMING && 
            fsm_state.currentState != ERROR_STATE) {
            
            // Log error for diagnostics
            logError(axisErr, motorErr, encoderErr, controllerErr, fsm_state.currentState);
            
            // Classify error severity
            ErrorSeverity severity = classifyError(axisErr, motorErr, encoderErr, controllerErr);
            
            // Recovery strategy based on severity
            switch(severity) {
                case ERR_EXPECTED_TRANSIENT:
                    // Module handles it (e.g., Homing auto-clears 0x100)
                    // Do nothing here, let module-level handling work
                    break;
                    
                case ERR_RECOVERABLE_RETRY:
                    // Clear errors + request CLC, stay in current state
                    MessageBuffer::getInstance().sendMessage("Error: Recoverable (retry) - clearing and requesting State 8");
                    motor.clearErrors();
                    delay(ERROR_CLEAR_DELAY_MS);  // Brief pause for error clear to process
                    motor.setAxisState(ODriveCANProtocol::AxisState::CLOSED_LOOP_CONTROL);
                    // Stay in current state, retry operation
                    break;
                    
                case ERR_RECOVERABLE_HOMING:
                    // Requires recalibration - return to homing
                    MessageBuffer::getInstance().sendMessage("Error: Requires recalibration - returning to homing");
                    fsm_state.currentState = InjectorStates::INIT_HOMING;
                    flags.calibrationDone = false;  // Force recalibration
                    break;
                    
                case ERR_SAFETY_CRITICAL:
                    // Hardware fault - enter ERROR_STATE, require user intervention
                    MessageBuffer::getInstance().sendMessage("Error: SAFETY CRITICAL - user intervention required");
                    safety.triggerHalt(ERR_OVER_TEMP);  // Use existing safety error code
                    fsm_state.currentState = InjectorStates::ERROR_STATE;
                    fsm_state.error = motorErr;  // Store primary error code
                    break;
            }
        }
        // ===== END CENTRALIZED ERROR CHECKING =====
    }


    // --- STATE MACHINE ---
    bool ignoreButtons = (millis() - stateTimer < 500);

    switch (fsm_state.currentState) {
        case InjectorStates::ERROR_STATE:
            if (stateEntry) {
                MotorWrapper::setModeAndMove(motor, 2, 1, 0, "Stop");
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
            
            // Log homing state progress (throttled to 1Hz)
            static unsigned long lastHomingLogTime = 0;
            if (millis() - lastHomingLogTime > 1000) {
                char homingBuf[64];
                snprintf(homingBuf, sizeof(homingBuf), "[HOMING: %s]", Homing::getStateString());
                MessageBuffer::getInstance().sendMessage(homingBuf);
                lastHomingLogTime = millis();
            }
            
            // Check for completion or error
            if (Homing::isComplete()) {
                // Don't send move command here - let Refill module handle it
                fsm_state.currentState = InjectorStates::REFILL;
            } else if (Homing::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFF;  // Homing error
            }
            break;
        }


        case InjectorStates::REFILL:
            /*
            ===== COMMENTED OUT: Old FSM Logic (replaced by modular pattern) =====
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
            ===== END COMMENTED REFILL =====
            */
            // ===== NEW: Modular Refill =====
            if (stateEntry) {
                Refill::begin();
                stateEntry = false;
            }
            if (Refill::update(motor)) {
                // Position reached - unlock buttons
                moveLockActive = false;
            }
            
            // Check if position move complete to unlock buttons
            if (moveLockActive && Refill::isComplete()) {
                moveLockActive = false;
            }
            
            // Button handlers (locked during position move)
            if (!ignoreButtons && !moveLockActive) {
                // Upper+Lower: Toggle end-of-day flag
                static unsigned long togglePressTime = 0;
                static bool toggleProcessed = false;
                
                if (btnUpper.read() == LOW && btnLower.read() == LOW) {
                    if (!toggleProcessed) {
                        if (togglePressTime == 0) {
                            togglePressTime = millis();
                        }
                        if (millis() - togglePressTime >= UI_BUTTON_TOGGLE_DELAY_MS) {
                            flags.endOfDay = !flags.endOfDay;
                            toggleProcessed = true;
                        }
                    }
                } else {
                    // Buttons released - reset for next toggle
                    togglePressTime = 0;
                    toggleProcessed = false;
                }
                // Center: Proceed to Compression
                if (btnCenter.released()) {
                    fsm_state.currentState = InjectorStates::COMPRESSION;
                }
            }
            
            if (Refill::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFE;  // Refill error
            }
            break;

        case InjectorStates::COMPRESSION:
            /*
            ===== COMMENTED OUT: Old FSM Logic (replaced by modular pattern) =====
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
            ===== END COMMENTED COMPRESSION =====
            */
            // ===== NEW: Modular Compression =====
            if (stateEntry) {
                Compression::begin(Compression::MODE_1_TRAVEL);  // Full travel mode post-refill
                stateEntry = false;
            }
            if (Compression::update(motor)) {
                fsm_state.currentState = InjectorStates::READY_TO_INJECT;
                lastAutoCompress = millis();
            }
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                logMessage("Compression: User aborted, returning to Refill");
                fsm_state.currentState = InjectorStates::REFILL;
                moveLockActive = true;  // Lock buttons until Refill position reached
                Compression::reset();
            }
            if (!ignoreButtons && !buttonLock && btnLower.released()) { 
                logMessage("Compression: User confirmed, ready to inject");
                fsm_state.currentState = InjectorStates::READY_TO_INJECT;
                lastAutoCompress = millis();
                Compression::reset();
            }
            if (Compression::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFD;  // Compression error
            }
            break;

        case InjectorStates::READY_TO_INJECT:
            /*
            ===== COMMENTED OUT: Old FSM Logic (replaced by modular pattern) =====
            safety.setContext(CTX_IDLE); 
            if (millis() - lastAutoCompress > TIME_AUTO_COMPRESS) fsm_state.currentState = InjectorStates::COMPRESSION; 
            if (!ignoreButtons && btnUpper.read() == LOW && btnLower.read() == LOW) { fsm_state.currentState = InjectorStates::PURGE_ZERO; }
            else if (!ignoreButtons && !buttonLock && btnCenter.released()) { 
                logMessage("Ready: Returning to Refill");
                motor.setInputVel(0);  // Stop motor
                lastMotorCmdTime = millis();
                fsm_state.currentState = InjectorStates::REFILL; 
            }
            ===== END COMMENTED READY_TO_INJECT =====
            */
            // ===== NEW: Modular ReadyToInject =====
            if (stateEntry) {
                ReadyToInject::begin();
                stateEntry = false;
            }
            if (ReadyToInject::update(motor)) {
                // ReadyToInject runs indefinitely, check for user input to proceed
            }
            if (!ignoreButtons && btnUpper.read() == LOW && btnLower.read() == LOW) { 
                logMessage("Ready: User confirms, moving to Purge");
                fsm_state.currentState = InjectorStates::PURGE_ZERO;
                ReadyToInject::reset();
            }
            else if (!ignoreButtons && !buttonLock && btnCenter.released()) { 
                logMessage("Ready: User abort, returning to Refill");
                fsm_state.currentState = InjectorStates::REFILL;
                moveLockActive = true;  // Lock buttons until Refill position reached
                ReadyToInject::reset();
            }
            if (ReadyToInject::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFC;  // ReadyToInject error
            }
            break;

        case InjectorStates::PURGE_ZERO:
            /*
            ===== COMMENTED OUT: Old FSM Logic (replaced by modular pattern) =====
            safety.setContext(CTX_PURGE); 
            static bool buttonsReleased = false;
            if (stateEntry) buttonsReleased = false;
            if (!buttonsReleased) { if (btnUpper.read() == HIGH && btnLower.read() == HIGH) { buttonsReleased = true; logMessage("Purge: Buttons Released."); } } 
            else {
                if (btnUpper.read() == LOW) MotorWrapper::setModeAndMove(motor, 2, 1, PURGE_VEL_UP, "Purge Up");       // PURGE_VEL_UP is negative (up)
                else if (btnLower.read() == LOW) MotorWrapper::setModeAndMove(motor, 2, 1, PURGE_VEL_DOWN, "Purge Down");  // PURGE_VEL_DOWN is positive (down)
                else MotorWrapper::setModeAndMove(motor, 2, 1, 0, "Stop");
                if (btnCenter.pressed()) { 
                    logMessage("Purge: Entering AntiDrip");
                    fsm_state.currentState = InjectorStates::ANTIDRIP; 
                    antiDripTimer = millis(); 
                }
            }
            ===== END COMMENTED PURGE_ZERO =====
            */
            // ===== NEW: Modular PurgeZero =====
            if (stateEntry) {
                PurgeZero::begin();
                stateEntry = false;
            }
            if (PurgeZero::update(motor, btnUpper.read(), btnLower.read(), btnCenter.released())) {
                logMessage("PurgeZero: Complete, moving to AntiDrip");
                fsm_state.currentState = InjectorStates::ANTIDRIP;
                PurgeZero::reset();
            }
            if (PurgeZero::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFB;  // PurgeZero error
            }
            break;

        case InjectorStates::ANTIDRIP:
            /*
            ===== COMMENTED OUT: Old FSM Logic (replaced by modular pattern) =====
            {
                safety.setContext(CTX_MOVING_FREE); 
                if (stateEntry) {
                    logMessage("AntiDrip: Decompression (slow retract, 15s timeout)");
                    motor.setControllerModes(ODriveCANProtocol::ControlMode::VELOCITY_CONTROL,
                                            ODriveCANProtocol::InputMode::PASSTHROUGH);
                    lastMotorCmdTime = 0;              // Force immediate command
                }
                // Move UP (negative velocity) to decompress
                MotorWrapper::setModeAndMove(motor, 2, 1, ANTIDRIP_VEL, "AntiDrip Vel");  // ANTIDRIP_VEL is negative (up)
                
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
            ===== END COMMENTED ANTIDRIP =====
            */
            // ===== NEW: Modular AntiDrip =====
            if (stateEntry) {
                AntiDrip::begin();
                stateEntry = false;
            }
            if (AntiDrip::update(motor)) {
                // AntiDrip complete, handle user button responses
            }
            if (!ignoreButtons && btnCenter.read() == LOW && btnLower.read() == LOW) { 
                logMessage("AntiDrip: User confirmed, moving to Inject");
                fsm_state.currentState = InjectorStates::INJECT;
                AntiDrip::reset();
            }
            else if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                logMessage("AntiDrip: User abort, returning to Ready");
                fsm_state.currentState = InjectorStates::READY_TO_INJECT;
                AntiDrip::reset();
            }
            if (AntiDrip::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFA;  // AntiDrip error (usually timeout)
            }
            break;

        case InjectorStates::INJECT:
            /*
            ===== COMMENTED OUT: Old FSM Logic (replaced by modular pattern) =====
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
            ===== END COMMENTED INJECT =====
            */
            // ===== NEW: Modular Injection (FILLING phase) =====
            if (stateEntry) {
                Injection::begin(currentMould);
                stateEntry = false;
            }
            if (Injection::update(motor)) {
                // Auto-transition to HOLD_INJECTION (module continues in PACKING phase)
                fsm_state.currentState = InjectorStates::HOLD_INJECTION;
                // DO NOT reset() here - module needs to stay active for PACKING phase!
            }
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                logMessage("Inject: User abort, releasing mould");
                fsm_state.currentState = InjectorStates::RELEASE;
                Injection::reset();
            }
            if (Injection::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xF9;  // Injection error
            }
            break;

        case InjectorStates::HOLD_INJECTION:
            /*
            ===== COMMENTED OUT: Old FSM Logic (replaced by modular pattern) =====
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
            ===== END COMMENTED HOLD_INJECTION =====
            */
            // ===== NEW: Modular Injection (PACKING phase, handled by same module) =====
            Injection::update(motor);  // CRITICAL: Must call update() to check pack timer
            if (Injection::isComplete()) {
                logMessage("Hold: Pack time complete, releasing mould");
                fsm_state.currentState = InjectorStates::RELEASE;
                Injection::reset();  // Reset module when DONE (exiting injection sequence)
            }
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                logMessage("Pack: User abort, releasing mould");
                fsm_state.currentState = InjectorStates::RELEASE;
                Injection::reset();  // Reset module on abort
            }
            break;

        case InjectorStates::RELEASE: 
            /*
            ===== COMMENTED OUT: Old FSM Logic (replaced by modular pattern) =====
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
            ===== END COMMENTED RELEASE =====
            */
            // ===== NEW: Release (simple auto-transition) =====
            if (stateEntry) {
                logMessage("Release: Unloading mould");
                
                // Queue all commands - ring buffer handles timing
                MotorWrapper::setMotorLimits(motor, RELEASE_CONTROLLER_VEL_LIMIT, RELEASE_CURRENT_LIMIT, "RELEASE");
                MotorWrapper::setTrapTrajParams(motor, RELEASE_TRAP_VEL_LIMIT, RELEASE_ACCEL, RELEASE_DECEL, "RELEASE_TRAJ");
                
                float releaseTarget = motor.getPosition() + RELEASE_DIST;  // RELEASE_DIST is negative (up)
                MotorWrapper::setModeAndMove(motor, 3, 5, releaseTarget, "Pos Release");  // Mode 3 (Position), InputMode 5 (TRAP_TRAJ)
                
                stateEntry = false;
            }
            if (millis() - stateTimer > 2000) { 
                logMessage("Release: Complete, confirming mould removal");
                fsm_state.currentState = InjectorStates::CONFIRM_MOULD_REMOVAL; 
            }
            break;

        case InjectorStates::CONFIRM_MOULD_REMOVAL:
             /*
             ===== COMMENTED OUT: Old FSM Logic (replaced by modular pattern) =====
             if (!ignoreButtons && !buttonLock && (btnCenter.released() || btnUpper.released() || btnLower.released())) { 
                 if(flags.endOfDay) fsm_state.currentState = InjectorStates::READY_TO_INJECT; 
                 else fsm_state.currentState = InjectorStates::REFILL; 
             }
             ===== END COMMENTED CONFIRM =====
             */
             // ===== NEW: Confirm (button-driven state return) =====
             static unsigned long confirmButtonTime = 0;
             
             // Only accept UPPER or LOWER buttons (not center to avoid carry-over)
             if (!ignoreButtons && !buttonLock && (btnUpper.released() || btnLower.released())) {
                 if (confirmButtonTime == 0) {
                     confirmButtonTime = millis();
                 }
             }
             
             // After 1 second delay, proceed to next state
             if (confirmButtonTime > 0 && millis() - confirmButtonTime >= 1000) {
                 if(flags.endOfDay) {
                     logMessage("Confirm: Returning to ReadyToInject");
                     fsm_state.currentState = InjectorStates::READY_TO_INJECT; 
                 }
                 else {
                     logMessage("Confirm: Returning to Refill");
                     fsm_state.currentState = InjectorStates::REFILL;
                     moveLockActive = true;  // Lock buttons until Refill position reached
                 }
                 confirmButtonTime = 0;  // Reset for next time
             }
             break;

    }
    updateLeds();
    
    // ===== LOOP TIMING MEASUREMENT =====
    unsigned long loopEnd = millis();
    loopTime = loopEnd - loopStart;
    if (loopTime > maxLoopTime) maxLoopTime = loopTime;
    
    if (millis() - lastDebugTime > 1000) { 
        lastDebugTime = millis(); 
        printDebugReport(loopTime, maxLoopTime);  // Pass timing values as parameters
        maxLoopTime = 0;  // Reset max after reporting (tracks max over ~1 second)
        // Flush buffered messages: output 1Hz status + accumulated event messages
        const char* output = MessageBuffer::getInstance().getOutput();
        Serial.println(output);
        MessageBuffer::getInstance().clearBuffer();  // Clear events for next cycle
    }
}
