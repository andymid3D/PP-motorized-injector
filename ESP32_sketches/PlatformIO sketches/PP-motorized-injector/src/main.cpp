#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Bounce2.h>
#include <SafeString.h>

#include "config.h"
#include "injector_fsm.h"
#include "SafetyManager.h"
#include "CanBusHandler.h"

// --- FSM Global Variables ---
fsm_inputs_t fsm_inputs;
fsm_outputs_t fsm_outputs;
fsm_state_t fsm_state;

// --- Hardware Objects ---
SafetyManager safety;
CanBusHandler motor;
Adafruit_NeoPixel ledsButtons(LED_COUNT_BUTTONS, PIN_LED_BUTTONS, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledsRing(LED_COUNT_RING, PIN_LED_RING, NEO_GRB + NEO_KHZ800);

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
void logMessage(const char* msg) { Serial.println(msg); }

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
    if (value == 0.0f || (millis() - lastMotorCmdTime > 20)) {
        motor.setControllerMode(ctrlMode, inputMode);
        if (ctrlMode == 1) motor.setTorque(value);      
        else if (ctrlMode == 2) motor.setVelocity(value); 
        else if (ctrlMode == 3) motor.setPosition(value); 
        lastMotorCmdTime = millis();
        lastCmdStr = cmdName;
    }
}

// --- Homing Sequence (Fixed Inertia Spinout) ---
bool runHomingSequence() {
    static int step = 0;
    static int lastStep = -1;
    static unsigned long waitStart = 0;
    
    if (stateEntry) { step = 0; lastStep = -1; } 
    bool newStep = (step != lastStep);
    if (newStep) lastStep = step;
    bool sendCmd = (millis() - lastMotorCmdTime > 20);
    if (sendCmd) lastMotorCmdTime = millis();

    switch(step) {
        case 0: 
            if (newStep) {
                logMessage("Home: Clear Err");
                safety.setContext(CTX_MOVING_FREE); 
                motor.clearErrors(); 
                lastCmdStr = "ClearErr";
                waitStart = millis(); 
            }
            if (millis() - waitStart > 500) step++; 
            break;
        case 1: 
            // Check if Calibration already done this session
            if (flags.calibrationDone) { 
                logMessage("Home: Calib Skipped"); 
                step = 4; 
            } 
            else step++;
            break;
        case 2: 
            if (newStep) { logMessage("Home: Calib (State 7)"); motor.setAxisState(7); lastCmdStr = "State7"; waitStart = millis(); }
            if (motor.getAxisState() == 7) step++; 
            if (millis() - waitStart > 3000) { if (motor.getAxisState() == 1) step++; } 
            break;
        case 3: 
            if (motor.getAxisState() == 1) {
                flags.calibrationDone = true; // Mark as done
                step++; 
            }
            break;
        case 4: 
            if (newStep) { logMessage("Home: Request Closed Loop"); motor.setAxisState(8); lastCmdStr = "State8"; waitStart = millis(); }
            if (motor.getAxisState() == 8) { if (millis() - waitStart > 200) step++; } 
            else { if (millis() - waitStart > 500) { motor.setAxisState(8); waitStart = millis(); } }
            break;
        case 5: 
            if (newStep) logMessage("Home: Move Up");
            setModeAndMove(2, 1, -SPEED_HOMING_FAST, "Vel -Fast");
            if (safety.isTopEndstopHit()) { motor.stop(); step++; } 
            break;
        case 6: 
            // FIX: Wait for Inertia to settle (Velocity ~ 0)
            if (newStep) { logMessage("Home: Relax (Wait for Stop)"); waitStart = millis(); }
            setModeAndMove(2, 1, 0, "Vel 0");
            
            // Wait until stopped OR timeout 2s
            if (abs(motor.getVelocity()) < 0.1 || (millis() - waitStart > 2000)) {
                step++;
            }
            break;
        case 7: 
            if (newStep) { logMessage("Home: Backoff"); waitStart = millis(); }
            setModeAndMove(2, 1, SPEED_HOMING_SLOW, "Vel +Slow");
            if (millis() - waitStart > 1500) { motor.stop(); step++; }
            break;
        case 8: 
            if (newStep) logMessage("Home: Slow Approach");
            setModeAndMove(2, 1, -SPEED_HOMING_SLOW, "Vel -Slow");
            if (safety.isTopEndstopHit()) { motor.stop(); step++; } 
            break;
        case 9: 
            if (!flags.initialHomingDone) { motor.setPosition(0.0f); lastCmdStr = "SetPos 0"; flags.initialHomingDone = true; } 
            logMessage("Home: Done"); 
            return true; 
    }
    return false;
}

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
    char buf[128];
    long pDisp = safety.getPressure();
    if (pDisp > 999999) pDisp = 999999; if (pDisp < -999999) pDisp = -999999;
    snprintf(buf, sizeof(buf), "[%-16s] T:%-3d P:%-7ld | OD:%d Err:0x%-2X | P:%-5.1f V:%-4.1f | Cmd:%s",
        getStateName(fsm_state.currentState), fsm_inputs.nozzleTemperature, pDisp,
        motor.getAxisState(), motor.getAxisError(), motor.getPosition(), motor.getVelocity(), lastCmdStr.c_str());
    Serial.println(buf);
}

void setup() {
    Serial.begin(115200); delay(2000); Serial.println("\n\n--- SYSTEM START ---");
    SafeString::setOutput(Serial); 
    safety.begin(); motor.begin(); pinMode(PIN_TEMP_ANALOG, INPUT);
    ledsButtons.begin(); ledsRing.begin(); ledsButtons.setBrightness(LED_BRIGHT_LOW); ledsRing.setBrightness(LED_BRIGHT_LOW);
    
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
    static unsigned long lastVbusReq = 0;
    if (millis() - lastVbusReq > 200) { motor.requestVbusVoltage(); lastVbusReq = millis(); }

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
            if (stateEntry) { Serial.printf("!!! ERROR STATE ENTERED: 0x%X !!!\n", fsm_state.error); errorLogged = true; }
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

        case InjectorStates::INIT_HOMING: 
            if (runHomingSequence()) { setModeAndMove(3, 1, OFFSET_REFILL_GAP, "Pos Refill"); fsm_state.currentState = InjectorStates::REFILL; } 
            break;

        case InjectorStates::REFILL:
            safety.setContext(CTX_IDLE); 
            if (!ignoreButtons && btnUpper.read() == LOW && btnLower.read() == LOW) { flags.endOfDay = !flags.endOfDay; delay(500); }
            else if (!ignoreButtons && !buttonLock && btnCenter.released()) { fsm_state.currentState = InjectorStates::COMPRESSION; }
            break;

        case InjectorStates::COMPRESSION:
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { setModeAndMove(2, 1, 0, "Stop"); fsm_state.currentState = InjectorStates::REFILL; }
            else if (!ignoreButtons && !buttonLock && btnLower.released()) { setModeAndMove(2, 1, 0, "Stop"); fsm_state.currentState = InjectorStates::READY_TO_INJECT; lastAutoCompress = millis(); }
            if (runCompressionCycle()) { fsm_state.currentState = InjectorStates::READY_TO_INJECT; lastAutoCompress = millis(); }
            break;

        case InjectorStates::READY_TO_INJECT:
            safety.setContext(CTX_IDLE); 
            if (millis() - lastAutoCompress > TIME_AUTO_COMPRESS) fsm_state.currentState = InjectorStates::COMPRESSION; 
            if (!ignoreButtons && btnUpper.read() == LOW && btnLower.read() == LOW) { fsm_state.currentState = InjectorStates::PURGE_ZERO; }
            else if (!ignoreButtons && !buttonLock && btnCenter.released()) { fsm_state.currentState = InjectorStates::REFILL; }
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
                if (btnCenter.pressed()) { fsm_state.currentState = InjectorStates::ANTIDRIP; antiDripTimer = millis(); setModeAndMove(3, 1, motor.getPosition() + DIST_ANTIDRIP_REV, "AntiDrip"); }
            }
            break;

        case InjectorStates::ANTIDRIP:
            safety.setContext(CTX_MOVING_FREE); 
            if (!ignoreButtons && btnCenter.read() == LOW && btnLower.read() == LOW) { fsm_state.currentState = InjectorStates::INJECT; }
            else if (!ignoreButtons && !buttonLock && btnUpper.released()) { fsm_state.currentState = InjectorStates::READY_TO_INJECT; }
            break;

        case InjectorStates::INJECT:
            safety.setContext(CTX_BLOCKED); 
            if (stateEntry) {
                motor.setLimits(currentMould.fillSpeed + 5.0f, currentMould.fillPressure);
                float targetPos = motor.getPosition() + volToTurns(currentMould.fillVolume);
                if (targetPos > POS_BOTTOM_MAX) targetPos = POS_BOTTOM_MAX;
                setModeAndMove(3, 1, targetPos, "Inject Pos");
            }
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { fsm_state.currentState = InjectorStates::RELEASE; }
            if (millis() - stateTimer > 500 && abs(motor.getVelocity()) < 0.1) { logMessage("Inject Done."); fsm_state.currentState = InjectorStates::HOLD_INJECTION; }
            break;

        case InjectorStates::HOLD_INJECTION:
            if (stateEntry) {
                logMessage("Packing...");
                motor.setLimits(currentMould.packSpeed + 2.0f, currentMould.packPressure);
                float holdTarget = motor.getPosition() + volToTurns(currentMould.packVolume);
                if (holdTarget > POS_BOTTOM_MAX) holdTarget = POS_BOTTOM_MAX;
                setModeAndMove(3, 1, holdTarget, "Pack Pos");
            }
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { fsm_state.currentState = InjectorStates::RELEASE; }
            if (millis() - stateTimer > (currentMould.packTime * 1000)) fsm_state.currentState = InjectorStates::RELEASE;
            break;

        case InjectorStates::RELEASE: 
            if (stateEntry) {
                motor.setLimits(VEL_LIMIT_INJECT, 30.0f); 
                float releaseTarget = motor.getPosition() + DIST_RELEASE_MOULD; 
                setModeAndMove(3, 1, releaseTarget, "Release Pos");
            }
            if (millis() - stateTimer > 2000) { setModeAndMove(2, 1, 0, "Stop"); fsm_state.currentState = InjectorStates::CONFIRM_MOULD_REMOVAL; }
            break;

        case InjectorStates::CONFIRM_MOULD_REMOVAL:
             if (!ignoreButtons && !buttonLock && (btnCenter.released() || btnUpper.released() || btnLower.released())) { 
                 if(flags.endOfDay) fsm_state.currentState = InjectorStates::READY_TO_INJECT; 
                 else fsm_state.currentState = InjectorStates::REFILL; 
             }
             break;
    }
    updateLeds();
    if (millis() - lastDebugTime > 1000) { lastDebugTime = millis(); printDebugReport(); }
}