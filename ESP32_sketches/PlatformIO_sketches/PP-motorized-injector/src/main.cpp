#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Bounce2.h>
#include <SafeString.h>
#include <BufferedOutput.h>
#include <ESP32-TWAI-CAN.hpp>

#include "config.h"
#include "injector_fsm.h"
#include "SafetyManager.h"
#include "CanBusHandlerV2.h"
#include "DebugCommands.h"
#include "UI.h"
#include "Homing.h"
#include "BroadcastDataStore.h"
#include "MessageBuffer.h"
#include "MotorWrapper.h"
#include "GPTimer.h"
#include "CanRxHandler.h"
#include "RingBuffer.h"
#include "PhaseTests.h"
#include "ProtectedWindowTest.h"
#include "OurLoopTimer.h" // Use our custom dual-core timer

// ===== MODULAR STATE MACHINES (Phase 1 Integration) =====
#include "Refill.h"
#include "Compression.h"
#include "Injection.h"
#include "AntiDrip.h"
#include "PurgeZero.h"
#include "ReadyToInject.h"
// ===== END MODULE INCLUDES =====

// ===== DEBUG MODULES =====
#include "DebugCommands.h"
// #include "RTRDebug.h"  // DEPRECATED - files moved to .old
// ===== END DEBUG MODULES =====

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

createBufferedOutput(serialOutput, 512, DROP_UNTIL_EMPTY);

// --- Debug Mode ---
bool debugCommandsEnabled = false;
bool debugHomingEnabled = false;
bool debugRTREnabled = false;

void validateDebugModes() {
    uint8_t enabledCount = 0;
    if (debugCommandsEnabled) enabledCount++;
    if (debugHomingEnabled) enabledCount++;
    if (debugRTREnabled) enabledCount++;
    
    if (enabledCount > 1) {
        MessageBuffer::getInstance().sendMessage("CRITICAL ERROR: Multiple debug modes enabled!");
        while (true) {
            delay(1000);
        }
    }
}

DebugCommands debugCmds;
ProtectedWindowTest protectedWindow;

// --- Input Debouncers ---
Bounce2::Button btnCenter = Bounce2::Button(); 
Bounce2::Button btnUpper = Bounce2::Button();  
Bounce2::Button btnLower = Bounce2::Button();  

// --- Global Flags & Timers ---
MachineFlags flags = {0, 0, 0, 0};
unsigned long lastAutoCompress = 0;
unsigned long antiDripTimer = 0;
unsigned long lastDebugTime = 0;
unsigned long lastMotorCmdTime = 0; 
unsigned long bootTime = 0;
unsigned long stateTimer = 0; 
bool tempErrorActive = false; 
bool errorLogged = false; 
bool buttonLock = false; 

int lastFsmState = -1;
bool stateEntry = false;
float injectStartPos = 0.0f;
float packStartPos = 0.0f;

commonInjectParams_t commonParams = {
    REFILL_TRAP_VEL_LIMIT, REFILL_ACCEL, REFILL_DECEL,
    COMPRESS_RAMP_TARGET, COMPRESS_RAMP_DURATION, COMPRESS_MICRO_CURRENT,
    INJECT_FILL_TRAP_VEL_LIMIT, INJECT_FILL_ACCEL, INJECT_FILL_DECEL, INJECT_FILL_CURRENT,
    INJECT_PACK_TRAP_VEL_LIMIT, INJECT_PACK_ACCEL, INJECT_PACK_DECEL, INJECT_PACK_CURRENT,
    INJECT_VEL_THRESHOLD, INJECT_POS_TOLERANCE, INJECT_STABLE_TIME_MS
};

actualMouldParams_t currentMould = {
    "Default", 35.0f, 25.0f, 20.0f, 5.0f, 2.0f, 10.0f, 2.0f, 5.0f,
    REFILL_ACCEL, REFILL_DECEL, 10.0f, 10.0f
};

float volToTurns(float cm3) {
    return cm3 * TURNS_PER_CM3_VOL;
}

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

void updateLeds() {
    uint32_t colUpper = BLACK_RGB, colCenter = BLACK_RGB, colLower = BLACK_RGB, colRing = BLACK_RGB;
    
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
        case CONFIRM_MOULD_REMOVAL: colUpper = GREEN_RGB; colCenter = BLACK_RGB; colLower = GREEN_RGB; colRing = GREEN_RGB; break;
    }
    
    uint8_t brightness = (btnUpper.read() == LOW || btnCenter.read() == LOW || btnLower.read() == LOW) ? LED_BRIGHT_HIGH : LED_BRIGHT_LOW;
    ledsButtons.setBrightness(brightness);
    ledsRing.setBrightness(brightness);
    
    ledsButtons.setPixelColor(0, colUpper); ledsButtons.setPixelColor(1, colCenter); ledsButtons.setPixelColor(2, colLower); ledsButtons.show();
    for(int i=0; i<LED_COUNT_RING; i++) ledsRing.setPixelColor(i, colRing); ledsRing.show();
}

void testCanRxISR() {
    // This function is now deprecated as its logic is part of the standard test harness.
}

bool runCompressionCycle() {
    static unsigned long compressStart = 0;
    if (stateEntry) {
        MessageBuffer::getInstance().sendMessage("Compression: Start Torque Ramp");
        safety.setContext(CTX_BLOCKED);
        compressStart = millis();
    }
    float elapsed = (millis() - compressStart) / 1000.0f;
    float targetTorque = (COMPRESS_RAMP_TARGET / 2.0f) * elapsed;
    if (targetTorque > COMPRESS_RAMP_TARGET) targetTorque = COMPRESS_RAMP_TARGET;
    
    MotorWrapper::setModeAndMove(motor, 1, 1, targetTorque, "TorqueMode");

    if (elapsed > 15.0f) { MessageBuffer::getInstance().sendMessage("Compression: Timeout"); MotorWrapper::setModeAndMove(motor, 2, 1, 0, "Stop"); return true; }
    if (elapsed > 1.0f && abs(motor.getVelocity()) < 0.5f) { MessageBuffer::getInstance().sendMessage("Compression: Stall Detected"); MotorWrapper::setModeAndMove(motor, 2, 1, 0, "Stop"); return true; }
    return false;
}

void printDebugReport(unsigned long currentLoopTime, unsigned long maxLoopTimeSinceLastReport) {
    char buf[256];
    long pDisp = safety.getPressure();
    const ODriveCANProtocol::CyclicIq& iq_data = motor.getIq();
    float iq_setpoint = iq_data.Iq_setpoint;
    float iq_measured = iq_data.Iq_measured;
    int lastControlMode = MotorWrapper::getLastControlMode();
    int lastInputMode = MotorWrapper::getLastInputMode();
    String lastCmdStr = MotorWrapper::getLastCommand();
    unsigned long uptimeSeconds = millis() / 1000;
    BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
    uint32_t motorErr = broadcast.getMotorError();
    uint32_t encoderErr = broadcast.getEncoderError();
    uint32_t controllerErr = broadcast.getControllerError();
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
    
    serialOutput.connect(Serial);

    MessageBuffer::getInstance().sendMessage("SYSTEM START");
    SafeString::setOutput(Serial); 
    MotorWrapper::init();
    
    if (!hwTimer.begin()) {
        Serial.println("ERROR: GPTimer init failed");
        while(1) { delay(1000); }
    }
    
#if TEST_MODE_PHASE1
    Serial.println("\n========================================");
    Serial.println("PHASE 1 TEST MODE ACTIVE");
    Serial.println("FSM bypassed - testing new modules only");
    Serial.println("========================================\n");
    
    initLoopTimer(); // Initialize our custom dual-core timer

    safety.begin();
    safety.enableMotorPower(true);
    delay(500);
    
    ESP32Can.setPins(PIN_CAN_TX, PIN_CAN_RX);
    ESP32Can.setSpeed(TWAI_SPEED_250KBPS);
    if (!ESP32Can.begin()) {
        Serial.println("ERROR: CAN bus init failed");
        while(1) { delay(1000); }
    }
    
#if TEST_BDS_INTEGRATION_ENABLED || TEST_PROTECTED_WINDOW_ENABLED
    debugCmds.begin(motor);
#endif
    
#if TEST_PROTECTED_WINDOW_ENABLED
    protectedWindow.begin(motor);
#endif
    
    CanRxHandler& canRx = CanRxHandler::getInstance();
    if (!canRx.begin()) {
        Serial.println("ERROR: CanRxHandler init failed");
        while(1) { delay(1000); }
    }
    
    canRx.setOutput(&serialOutput);
    
    PhaseTests::runSetupTests();
    
    if (!canRx.startCore0Task()) {
        Serial.println("ERROR: Failed to start Core 0 task");
        while(1) { delay(1000); }
    }
    
    return;
#endif
    
    if (debugCommandsEnabled) {
        motor.begin();
        debugCmds.begin(motor);
        MessageBuffer::getInstance().sendMessage("Debug mode activated - FSM disabled");
    } else if (debugRTREnabled) {
        motor.begin();
        MessageBuffer::getInstance().sendMessage("RTR Debug mode DEPRECATED - ignored");
    } else {
        safety.begin(); motor.begin(); pinMode(PIN_TEMP_ANALOG, INPUT);
        ledsButtons.begin(); ledsRing.begin(); ledsButtons.setBrightness(LED_BRIGHT_LOW); ledsRing.setBrightness(LED_BRIGHT_LOW);
    }
    
    btnUpper.attach(PIN_BTN_UPPER, INPUT_PULLUP);
    btnCenter.attach(PIN_BTN_CENTER, INPUT_PULLUP); 
    btnLower.attach(PIN_BTN_LOWER, INPUT_PULLUP);
    btnUpper.interval(10); btnCenter.interval(10); btnLower.interval(10);
    
    fsm_state.currentState = InjectorStates::INIT_HEATING;
    bootTime = millis();
}

void loop() {
    serialOutput.nextByteOut();

#if TEST_MODE_PHASE1
    toggleLoopFlag(); // Mark the start/end of the loop for measurement

#if TEST_PROTECTED_WINDOW_ENABLED
    protectedWindow.loop();
#else
    PhaseTests::runLoopTests(); // No longer needs serialOutput passed
#endif
    return;
#endif
    
    unsigned long loopStart = millis();
    static unsigned long lastLoopReportTime = 0;
    static unsigned long maxLoopTime = 0;
    static unsigned long loopTime = 0;
    
    motor.loop(); safety.updateInputs(); fsm_inputs.nozzleTemperature = readThermocouple();
    btnCenter.update(); btnUpper.update(); btnLower.update();

    validateDebugModes();

    if (debugCommandsEnabled) {
        debugCmds.loop();
        return;
    }
    
    if (debugRTREnabled) {
        return;
    }
    
    if (debugHomingEnabled) {
        if (Homing::getState() != Homing::HomingState::IDLE) {
            Homing::update(motor, safety);
        }
        
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
                motor.getEncoderEstimatesFromBroadcastDataStore().position, motor.getEncoderEstimatesFromBroadcastDataStore().velocity,
                (int)fsm_inputs.nozzleTemperature,
                Homing::getStateString());
            serialOutput.println(debugBuf);
        }
        
        safety.enableMotorPower(true);
        
        if (btnUpper.fell()) {
            MessageBuffer::getInstance().sendMessage(">>> UPPER BUTTON PRESSED - Starting homing sequence <<<");
            indicateRunning();
            Homing::begin(motor, safety);
        }
        
        if (Homing::isComplete()) {
            MessageBuffer::getInstance().sendMessage(">>> HOMING COMPLETE <<<");
            indicateReady();
            MessageBuffer::getInstance().sendMessage("Press upper button to repeat");
            Homing::reset();
        }
        
        if (Homing::hasError()) {
            MessageBuffer::getInstance().sendMessage(">>> HOMING FAILED <<<");
            indicateError();
            Homing::reset();
        }
        
        indicateWaiting();
        return;
    }
    
    if (fsm_state.currentState != lastFsmState) { stateEntry = true; lastFsmState = fsm_state.currentState; stateTimer = millis(); }
    else { stateEntry = false; }

    if (btnUpper.read() == LOW && btnLower.read() == LOW) buttonLock = true;
    else if (btnCenter.read() == LOW && btnLower.read() == LOW) buttonLock = true;
    if (buttonLock && btnUpper.read() == HIGH && btnLower.read() == HIGH && btnCenter.read() == HIGH) buttonLock = false;

    static bool moveLockActive = false;

    if (millis() - bootTime > 3000) {
        bool movingDown = motor.getVelocity() > 0.1f;
        if (!safety.check(motor.getVelocity(), movingDown)) { fsm_state.currentState = InjectorStates::ERROR_STATE; fsm_state.error = safety.getLastError(); }
        
        uint32_t axisErr = motor.getAxisError();
        uint32_t motorErr = motor.getMotorErrorDetails().motor_error;
        uint32_t encoderErr = motor.getEncoderErrorDetails().encoder_error;
        uint32_t controllerErr = motor.getControllerErrorDetails().controller_error;
        
        if (hasAnyError(axisErr, motorErr, encoderErr, controllerErr) && 
            fsm_state.currentState != INIT_HEATING &&
            fsm_state.currentState != INIT_HOT_NOT_HOMED &&
            fsm_state.currentState != INIT_HOMING && 
            fsm_state.currentState != ERROR_STATE) {
            
            logError(axisErr, motorErr, encoderErr, controllerErr, fsm_state.currentState);
            ErrorSeverity severity = classifyError(axisErr, motorErr, encoderErr, controllerErr);
            
            switch(severity) {
                case ERR_EXPECTED_TRANSIENT:
                    break;
                case ERR_RECOVERABLE_RETRY:
                    MessageBuffer::getInstance().sendMessage("Error: Recoverable (retry) - clearing and requesting State 8");
                    motor.clearErrors();
                    delay(ERROR_CLEAR_DELAY_MS);
                    motor.setAxisState(ODriveCANProtocol::AxisState::CLOSED_LOOP_CONTROL);
                    break;
                case ERR_RECOVERABLE_HOMING:
                    MessageBuffer::getInstance().sendMessage("Error: Requires recalibration - returning to homing");
                    fsm_state.currentState = InjectorStates::INIT_HOMING;
                    flags.calibrationDone = false;
                    break;
                case ERR_SAFETY_CRITICAL:
                    MessageBuffer::getInstance().sendMessage("Error: SAFETY CRITICAL - user intervention required");
                    safety.triggerHalt(ERR_OVER_TEMP);
                    fsm_state.currentState = InjectorStates::ERROR_STATE;
                    fsm_state.error = motorErr;
                    break;
            }
        }
    }

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
                if (!safety.isEStopPressed() && !safety.isBarrelOpen()) { MessageBuffer::getInstance().sendMessage("User Reset."); safety.resetError(); motor.clearErrors(); fsm_state.currentState = InjectorStates::INIT_HEATING; } 
            }
            break;

        case InjectorStates::INIT_HEATING: 
            if (stateEntry) {
                MessageBuffer::getInstance().sendMessage("Boot: Clearing ODrive errors");
                motor.clearErrors();
                delay(100);
            }
            if (fsm_inputs.nozzleTemperature >= TEMP_CRITICAL) fsm_state.currentState = InjectorStates::INIT_HOT_NOT_HOMED; else safety.enableMotorPower(false);
            break;

        case InjectorStates::INIT_HOT_NOT_HOMED: 
            safety.enableMotorPower(true); 
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                if (fsm_inputs.nozzleTemperature >= TEMP_MIN_MOVE) fsm_state.currentState = InjectorStates::INIT_HOMING; else MessageBuffer::getInstance().sendMessage("Temp too low!"); 
            }
            break;

        case InjectorStates::INIT_HOMING: {
            if (stateEntry) {
                Homing::begin(motor, safety);
                stateEntry = false;
            }
            Homing::update(motor, safety);
            static unsigned long lastHomingLogTime = 0;
            if (millis() - lastHomingLogTime > 1000) {
                char homingBuf[64];
                snprintf(homingBuf, sizeof(homingBuf), "[HOMING: %s]", Homing::getStateString());
                MessageBuffer::getInstance().sendMessage(homingBuf);
                lastHomingLogTime = millis();
            }
            if (Homing::isComplete()) {
                fsm_state.currentState = InjectorStates::REFILL;
            } else if (Homing::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFF;
            }
            break;
        }

        case InjectorStates::REFILL:
            if (stateEntry) {
                Refill::begin();
                stateEntry = false;
            }
            if (Refill::update(motor)) {
                moveLockActive = false;
            }
            if (moveLockActive && Refill::isComplete()) {
                moveLockActive = false;
            }
            if (!ignoreButtons && !moveLockActive) {
                static unsigned long togglePressTime = 0;
                static bool toggleProcessed = false;
                if (btnUpper.read() == LOW && btnLower.read() == LOW) {
                    if (!toggleProcessed) {
                        if (togglePressTime == 0) togglePressTime = millis();
                        if (millis() - togglePressTime >= UI_BUTTON_TOGGLE_DELAY_MS) {
                            flags.endOfDay = !flags.endOfDay;
                            toggleProcessed = true;
                        }
                    }
                } else {
                    togglePressTime = 0;
                    toggleProcessed = false;
                }
                if (btnCenter.released()) {
                    fsm_state.currentState = InjectorStates::COMPRESSION;
                }
            }
            if (Refill::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFE;
            }
            break;

        case InjectorStates::COMPRESSION:
            if (stateEntry) {
                Compression::begin(Compression::MODE_1_TRAVEL);
                stateEntry = false;
            }
            if (Compression::update(motor)) {
                fsm_state.currentState = InjectorStates::READY_TO_INJECT;
                lastAutoCompress = millis();
            }
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                MessageBuffer::getInstance().sendMessage("Compression: User aborted, returning to Refill");
                fsm_state.currentState = InjectorStates::REFILL;
                moveLockActive = true;
                Compression::reset();
            }
            if (!ignoreButtons && !buttonLock && btnLower.released()) { 
                MessageBuffer::getInstance().sendMessage("Compression: User confirmed, ready to inject");
                fsm_state.currentState = InjectorStates::READY_TO_INJECT;
                lastAutoCompress = millis();
                Compression::reset();
            }
            if (Compression::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFD;
            }
            break;

        case InjectorStates::READY_TO_INJECT:
            if (stateEntry) {
                ReadyToInject::begin();
                stateEntry = false;
            }
            if (ReadyToInject::update(motor)) {}
            if (!ignoreButtons && btnUpper.read() == LOW && btnLower.read() == LOW) {
                MessageBuffer::getInstance().sendMessage("Ready: User confirms, moving to Purge");
                fsm_state.currentState = InjectorStates::PURGE_ZERO;
                ReadyToInject::reset();
            }
            else if (!ignoreButtons && !buttonLock && btnCenter.released()) { 
                MessageBuffer::getInstance().sendMessage("Ready: User abort, returning to Refill");
                fsm_state.currentState = InjectorStates::REFILL;
                moveLockActive = true;
                ReadyToInject::reset();
            }
            if (ReadyToInject::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFC;
            }
            break;

        case InjectorStates::PURGE_ZERO:
            if (stateEntry) {
                PurgeZero::begin();
                stateEntry = false;
            }
            if (PurgeZero::update(motor, btnUpper.read(), btnLower.read(), btnCenter.released())) {
                MessageBuffer::getInstance().sendMessage("PurgeZero: Complete, moving to AntiDrip");
                fsm_state.currentState = InjectorStates::ANTIDRIP;
                PurgeZero::reset();
            }
            if (PurgeZero::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFB;
            }
            break;

        case InjectorStates::ANTIDRIP:
            if (stateEntry) {
                AntiDrip::begin();
                stateEntry = false;
            }
            if (AntiDrip::update(motor)) {}
            if (!ignoreButtons && btnCenter.read() == LOW && btnLower.read() == LOW) {
                MessageBuffer::getInstance().sendMessage("AntiDrip: User confirmed, moving to Inject");
                fsm_state.currentState = InjectorStates::INJECT;
                AntiDrip::reset();
            }
            else if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                MessageBuffer::getInstance().sendMessage("AntiDrip: User abort, returning to Ready");
                fsm_state.currentState = InjectorStates::READY_TO_INJECT;
                AntiDrip::reset();
            }
            if (AntiDrip::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xFA;
            }
            break;

        case InjectorStates::INJECT:
            if (stateEntry) {
                Injection::begin(currentMould);
                stateEntry = false;
            }
            if (Injection::update(motor)) {
                fsm_state.currentState = InjectorStates::HOLD_INJECTION;
            }
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                MessageBuffer::getInstance().sendMessage("Inject: User abort, releasing mould");
                fsm_state.currentState = InjectorStates::RELEASE;
                Injection::reset();
            }
            if (Injection::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xF9;
            }
            break;

        case InjectorStates::HOLD_INJECTION:
            Injection::update(motor);
            if (Injection::isComplete()) {
                MessageBuffer::getInstance().sendMessage("Hold: Pack time complete, releasing mould");
                fsm_state.currentState = InjectorStates::RELEASE;
                Injection::reset();
            }
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                MessageBuffer::getInstance().sendMessage("Pack: User abort, releasing mould");
                fsm_state.currentState = InjectorStates::RELEASE;
                Injection::reset();
            }
            break;

        case InjectorStates::RELEASE: 
            if (stateEntry) {
                MessageBuffer::getInstance().sendMessage("Release: Unloading mould");
                MotorWrapper::setMotorLimits(motor, RELEASE_CONTROLLER_VEL_LIMIT, RELEASE_CURRENT_LIMIT, "RELEASE");
                MotorWrapper::setTrapTrajParams(motor, RELEASE_TRAP_VEL_LIMIT, RELEASE_ACCEL, RELEASE_DECEL, "RELEASE_TRAJ");
                float releaseTarget = motor.getPosition() + RELEASE_DIST;
                MotorWrapper::setModeAndMove(motor, 3, 5, releaseTarget, "Pos Release");
                stateEntry = false;
            }
            if (millis() - stateTimer > 2000) { 
                MessageBuffer::getInstance().sendMessage("Release: Complete, confirming mould removal");
                fsm_state.currentState = InjectorStates::CONFIRM_MOULD_REMOVAL; 
            }
            break;

        case InjectorStates::CONFIRM_MOULD_REMOVAL:
             static unsigned long confirmButtonTime = 0;
             if (!ignoreButtons && !buttonLock && (btnUpper.released() || btnLower.released())) {
                 if (confirmButtonTime == 0) confirmButtonTime = millis();
             }
             if (confirmButtonTime > 0 && millis() - confirmButtonTime >= 1000) {
                 if(flags.endOfDay) {
                     MessageBuffer::getInstance().sendMessage("Confirm: Returning to ReadyToInject");
                     fsm_state.currentState = InjectorStates::READY_TO_INJECT; 
                 }
                 else {
                     MessageBuffer::getInstance().sendMessage("Confirm: Returning to Refill");
                     fsm_state.currentState = InjectorStates::REFILL;
                     moveLockActive = true;
                 }
                 confirmButtonTime = 0;
             }
             break;
    }
    updateLeds();
    
    unsigned long loopEnd = millis();
    loopTime = loopEnd - loopStart;
    if (loopTime > maxLoopTime) maxLoopTime = loopTime;
    
    if (millis() - lastDebugTime > 1000) { 
        lastDebugTime = millis(); 
        printDebugReport(loopTime, maxLoopTime);
        maxLoopTime = 0;
        #if DEBUG_ENABLED
        const char* output = MessageBuffer::getInstance().getOutput();
        serialOutput.println(output);
        MessageBuffer::getInstance().clearBuffer();
        #endif
    }
}
