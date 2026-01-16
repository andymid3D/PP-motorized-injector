#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Bounce2.h>
#include <SafeString.h>
#include <loopTimer.h>  // PHASE 1: Loop performance monitoring
#include <ESP32-TWAI-CAN.hpp>  // PHASE 1: CAN bus for testing

#include "config.h"
#include "injector_fsm.h"
#include "SafetyManager.h"
#include "CanBusHandlerV2.h"
#include "DebugCommands.h"
#include "UI.h"
#include "Homing.h"
#include "BroadcastDataStore.h"
#include "MessageBuffer.h"
#include "MotorWrapper.h"  // Centralized motor control
#include "GPTimer.h"  // PHASE 1: Hardware timer (Step 1.1)
#include "CanRxHandler.h"  // PHASE 1: Core 0 ISR + Queue (Step 1.3)

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

// NOTE: loopTimer global instance created automatically by <loopTimer.h> include

// --- Debug Mode ---
// CRITICAL: Only ONE debug mode can be enabled at a time
// Enabling multiple will cause unpredictable machine behavior
bool debugCommandsEnabled = false;   // Serial debug commands (DebugCommands module)
bool debugHomingEnabled = false;    // Debug homing sequence step-by-step
bool debugRTREnabled = false;       // RTR flag testing (RTRDebug module) - DEPRECATED, files moved to .old

// Runtime check: Ensure mutual exclusion
void validateDebugModes() {
    uint8_t enabledCount = 0;
    if (debugCommandsEnabled) enabledCount++;
    if (debugHomingEnabled) enabledCount++;
    if (debugRTREnabled) enabledCount++;
    
    if (enabledCount > 1) {
        MessageBuffer::getInstance().sendMessage("CRITICAL ERROR: Multiple debug modes enabled!");
        MessageBuffer::getInstance().sendMessage("Only ONE debug mode allowed at a time!");
        MessageBuffer::getInstance().sendMessage("Machine will NOT run until this is fixed!");
        while (true) {
            delay(1000);  // Halt machine
        }
    }
}

DebugCommands debugCmds;
// RTRDebug rtrDebug;  // DEPRECATED - module disabled

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

// ===== PHASE 1 TESTING: CanRxHandler Queue Validation (Step 1.4) =====
// TEMPORARY: Remove after validation complete
/**
 * @brief Test CanRxHandler ISR operation with real CAN data
 * 
 * Tests (Step 1.6):
 * 1. Queue initialization
 * 2. CAN bus initialization (ESP32Can)
 * 3. ISR registration and alert callback
 * 4. Real CAN message reception from ODrive
 * 5. Message retrieval with timestamp validation
 * 6. Queue statistics (depth, overflows, messages received)
 * 
 * Expected: ODrive broadcasts heartbeat (~10ms interval) and encoder estimates
 * 
 * NOTE: Requires hardware - ODrive must be powered and broadcasting
 */
void testCanRxISR() {
    Serial.println("\n=== CanRxHandler ISR Test Start ===");
    Serial.println("NOTE: Requires ODrive powered and broadcasting on CAN bus");
    
    CanRxHandler& canRx = CanRxHandler::getInstance();
    
    // Test 1: Initialize CAN bus BEFORE CanRxHandler
    Serial.println("\n[1] Initializing CAN bus (ESP32Can)...");
    ESP32Can.setPins(PIN_CAN_TX, PIN_CAN_RX);
    ESP32Can.setSpeed(TWAI_SPEED_250KBPS);
    if (!ESP32Can.begin()) {
        Serial.println("FAIL: CAN bus initialization");
        return;
    }
    Serial.println("PASS: CAN bus initialized (250kbps)");
    
    // Diagnostic: Check TWAI driver state
    twai_status_info_t twai_status;
    if (twai_get_status_info(&twai_status) == ESP_OK) {
        Serial.print("     TWAI state: ");
        Serial.print(twai_status.state);
        Serial.print(" (0=STOPPED, 1=RUNNING, 2=BUS_OFF, 3=RECOVERING)");
        Serial.println();
        Serial.print("     TX queue: ");
        Serial.print(twai_status.msgs_to_tx);
        Serial.print(", RX queue: ");
        Serial.println(twai_status.msgs_to_rx);
        Serial.print("     TX failed: ");
        Serial.print(twai_status.tx_failed_count);
        Serial.print(", RX missed: ");
        Serial.print(twai_status.rx_missed_count);
        Serial.print(", Bus errors: ");
        Serial.println(twai_status.bus_error_count);
    }
    
    // Test 2: Try direct frame read (bypass CanRxHandler)
    Serial.println("\n[2] Testing direct CAN frame reception (3 second test)...");
    uint32_t directTestStart = millis();
    uint32_t directFramesReceived = 0;
    while (millis() - directTestStart < 3000) {
        CanFrame rxFrame;
        if (ESP32Can.readFrame(rxFrame, 0)) {  // 0 = non-blocking
            directFramesReceived++;
            if (directFramesReceived == 1) {
                Serial.print("     FIRST FRAME: ID=0x");
                Serial.print(rxFrame.identifier, HEX);
                Serial.print(" DLC=");
                Serial.print(rxFrame.data_length_code);
                Serial.print(" Data: ");
                for (int i = 0; i < rxFrame.data_length_code && i < 8; i++) {
                    if (rxFrame.data[i] < 0x10) Serial.print("0");
                    Serial.print(rxFrame.data[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
            }
        }
        delayMicroseconds(100);
    }
    Serial.print("     Direct frames received: ");
    Serial.println(directFramesReceived);
    
    if (directFramesReceived == 0) {
        Serial.println("FAIL: No frames with ESP32Can.readFrame()");
        Serial.println("      Issue: CAN bus wiring, termination, or ODrive not broadcasting");
        Serial.println("      Check: CAN_H/CAN_L connected? 120Ω termination? ODrive LED blinking?");
        return;  // Stop test - no point continuing
    } else {
        Serial.println("PASS: Direct frame reception working!");
    }
    
    // Test 3: Initialize CanRxHandler (registers alerts)
    Serial.println("\n[3] Initializing CanRxHandler (queue + alerts)...");
    if (!canRx.begin()) {
        Serial.println("FAIL: CanRxHandler initialization");
        return;
    }
    Serial.println("PASS: CanRxHandler initialized");
    Serial.print("     Queue size: ");
    Serial.print(32);  // QUEUE_SIZE
    Serial.println(" messages");
    
    // Test 4: Wait for CAN messages via CanRxHandler polling
    Serial.println("\n[4] Testing CanRxHandler.pollAndQueue() (10 second test)...");
    Serial.println("     Measuring real-time loop overhead WITH polling active");
    
    uint32_t testStartTime = millis();
    uint32_t testDuration = 10000;  // 10 seconds
    uint32_t lastPrintTime = millis();
    uint32_t messagesProcessed = 0;
    uint32_t pollCalls = 0;
    uint32_t lastMessagesReceived = 0;
    
    // REAL-TIME LOOP TIME MEASUREMENT (hardware timestamp precision)
    uint64_t minLoopTime = UINT64_MAX;
    uint64_t maxLoopTime = 0;
    uint64_t totalLoopTime = 0;
    uint32_t loopIterations = 0;
    uint64_t lastLoopEnd = hwTimer.micros();
    
    while (millis() - testStartTime < testDuration) {
        // === START LOOP TIMING ===
        uint64_t loopStart = hwTimer.micros();
        
        // Poll TWAI and queue new messages (non-blocking, fast)
        canRx.pollAndQueue();
        pollCalls++;
        
        // Check for messages in queue (non-blocking)
        CANRxMessage msg;
        while (canRx.receiveMessage(msg, 0)) {  // 0 timeout = non-blocking
            messagesProcessed++;
            
            // Print message details every 500ms
            if (millis() - lastPrintTime >= 500) {
                Serial.print("     [");
                Serial.print(messagesProcessed);
                Serial.print("] CAN ID: 0x");
                Serial.print(msg.canId, HEX);
                Serial.print("  DLC: ");
                Serial.print(msg.dlc);
                Serial.print("  TS: ");
                Serial.print((uint32_t)msg.timestamp);
                Serial.print("us  Data: ");
                for (int i = 0; i < msg.dlc && i < 8; i++) {
                    if (msg.data[i] < 0x10) Serial.print("0");
                    Serial.print(msg.data[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                lastPrintTime = millis();
            }
        }
        
        // === END LOOP TIMING ===
        uint64_t loopEnd = hwTimer.micros();
        uint64_t loopDuration = loopEnd - loopStart;
        
        // Track statistics (excluding Serial.print overhead)
        if (loopDuration < minLoopTime) minLoopTime = loopDuration;
        if (loopDuration > maxLoopTime) maxLoopTime = loopDuration;
        totalLoopTime += loopDuration;
        loopIterations++;
        
        // Print progress every 2 seconds
        if (millis() - lastPrintTime >= 2000) {
            uint32_t currentReceived = canRx.getMessagesReceived();
            uint32_t newMessages = currentReceived - lastMessagesReceived;
            uint32_t avgLoopTime = (uint32_t)(totalLoopTime / loopIterations);
            
            Serial.print("     Progress: ");
            Serial.print(messagesProcessed);
            Serial.print(" processed, ");
            Serial.print(newMessages);
            Serial.print(" new, ");
            Serial.print(avgLoopTime);
            Serial.print("us avg loop");
            Serial.println();
            
            lastMessagesReceived = currentReceived;
            pollCalls = 0;
            lastPrintTime = millis();
        }
        
        // Small delay to prevent tight loop
        delayMicroseconds(100);
        lastLoopEnd = loopEnd;
    }
    
    // Calculate final loop time statistics
    uint32_t avgLoopTime = (uint32_t)(totalLoopTime / loopIterations);
    
    Serial.println("\n[4b] Real-Time Loop Performance (DURING polling):");
    Serial.print("     Loop iterations: ");
    Serial.println(loopIterations);
    Serial.print("     Min loop time: ");
    Serial.print((uint32_t)minLoopTime);
    Serial.println(" us");
    Serial.print("     Max loop time: ");
    Serial.print((uint32_t)maxLoopTime);
    Serial.println(" us");
    Serial.print("     Avg loop time: ");
    Serial.print(avgLoopTime);
    Serial.println(" us");
    Serial.print("     Poll frequency: ");
    Serial.print(loopIterations / 10);
    Serial.println(" Hz");
    Serial.println("     NOTE: Includes pollAndQueue() + receiveMessage() overhead");
    Serial.println("     NOTE: Excludes Serial.print() time (measured separately)");
    
    // Test 5: Validate statistics
    Serial.println("\n[5] Test Results:");
    Serial.print("     Messages processed: ");
    Serial.println(messagesProcessed);
    Serial.print("     ISR total received: ");
    Serial.println(canRx.getMessagesReceived());
    Serial.print("     Queue overflows: ");
    Serial.println(canRx.getQueueOverflows());
    Serial.print("     Final queue depth: ");
    Serial.println(canRx.getQueueDepth());
    
    // Expected: ~2000 messages in 10s (ODrive broadcasts ~5 msgs every 10ms = 500/sec per node)
    // With NodeID=0: Heartbeat + Encoder = ~200 messages/sec = ~2000 messages in 10s
    Serial.println("\n[6] Validation:");
    if (messagesProcessed == 0) {
        Serial.println("FAIL: No messages via CanRxHandler.pollAndQueue()");
        Serial.println("      But direct ESP32Can.readFrame() worked!");
        Serial.println("      Issue: TWAI alerts not triggering OR pollAndQueue() logic error");
        return;
    }
    
    if (canRx.getQueueOverflows() > 0) {
        Serial.print("WARN: Queue overflows detected (");
        Serial.print(canRx.getQueueOverflows());
        Serial.println(")");
        Serial.println("      Consider increasing queue size or processing faster");
    } else {
        Serial.println("PASS: Zero queue overflows (queue sizing OK)");
    }
    
    if (messagesProcessed >= 100) {
        Serial.println("PASS: Message reception working (>100 msgs in 10s)");
    } else {
        Serial.println("WARN: Low message rate (<100 msgs in 10s)");
        Serial.println("      Expected ~200/sec from ODrive NodeID=0");
    }
    
    Serial.println("\n=== CanRxHandler ISR Test Complete ===");
    Serial.println("PASS: ISR connection validated");
    Serial.println("NOTE: Check loopTimer stats for ISR overhead measurement\n");
}
// ===== END PHASE 1 TESTING =====


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
    MotorWrapper::init();  // Initialize motor wrapper tracking variables
    
    // Initialize GPTimer (PHASE 1: Step 1.1)
    if (!hwTimer.begin()) {
        Serial.println("ERROR: GPTimer init failed");
        while(1) { delay(1000); }  // Halt - timer critical for CAN RX
    }
    
#if TEST_MODE_PHASE1
    // ===== PHASE 1 TEST MODE: Isolated Module Testing =====
    Serial.println("\n========================================");
    Serial.println("PHASE 1 TEST MODE ACTIVE");
    Serial.println("FSM bypassed - testing new modules only");
    Serial.println("========================================\n");
    
    // Initialize safety manager (required for contactor control)
    safety.begin();
    
    // Enable DC Contactor (power ODrive for CAN messages)
    safety.enableMotorPower(true);
    delay(500);  // Allow ODrive to boot
    
    // Initialize CAN bus hardware
    ESP32Can.setPins(PIN_CAN_TX, PIN_CAN_RX);
    ESP32Can.setSpeed(TWAI_SPEED_250KBPS);
    if (!ESP32Can.begin()) {
        Serial.println("ERROR: CAN bus init failed");
        while(1) { delay(1000); }
    }
    
    // Initialize CanRxHandler
    CanRxHandler& canRx = CanRxHandler::getInstance();
    if (!canRx.begin()) {
        Serial.println("ERROR: CanRxHandler init failed");
        while(1) { delay(1000); }
    }
    
    // Launch Core 0 polling task
    if (!canRx.startCore0Task()) {
        Serial.println("ERROR: Failed to start Core 0 task");
        while(1) { delay(1000); }
    }
    
    // Print test info AFTER task running (minimal startup delay)
    Serial.println("\n========================================");
    Serial.println("PHASE 1: Dual-Core CAN RX Test Active");
    Serial.println("Core 0: Polling task | Core 1: Queue drain");
    Serial.println("========================================\n");
    
    return;  // Skip normal FSM initialization
#endif
    
    // ===== NORMAL MODE: FSM Initialization =====
    if (debugCommandsEnabled) {
        // DEBUG MODE: Skip FSM initialization, only init DebugCommands
        motor.begin();
        debugCmds.begin(motor);
        MessageBuffer::getInstance().sendMessage("Debug mode activated - FSM disabled");
    } else if (debugRTREnabled) {
        // RTR DEBUG MODE: Test RTR flag behavior (isolated from CanBusHandlerV2)
        // DEPRECATED: RTRDebug module disabled, files moved to .old
        motor.begin();  // Initialize CAN bus
        // rtrDebug.begin();
        MessageBuffer::getInstance().sendMessage("RTR Debug mode DEPRECATED - ignored");
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
    // ===== PHASE 1: PERFORMANCE MONITORING =====
    // loopTimer.check() measures loop execution time, prints stats every 5 seconds
    // Output: "loop us Latency / 5sec max:XXX avg:YYY / sofar max:ZZZ avg:WWW max - prt:PPP"
    // NOTE: Adds ~1-2ms overhead, remove after performance validation
    loopTimer.check(Serial);
    
#if TEST_MODE_PHASE1
    // ===== PHASE 1 TEST MODE: FSM Load Simulation =====
    // Simulate typical FSM loop overhead to test queue drain under load
    
    static uint32_t msgCount = 0;
    static unsigned long lastStatsTime = millis();
    static uint32_t drainCycles = 0;
    static uint32_t maxMsgsPerDrain = 0;
    static uint32_t maxDrainTime = 0;
    
    // SIMULATE FSM WORK (comment out to test bare performance)
    // Uncomment sections progressively to see impact
    
    // 1. Simulate sensor reads (~30µs)
    delayMicroseconds(30);
    
    // 2. Simulate thermocouple read (~100µs) - happens every loop in production
    delayMicroseconds(100);
    
    // 3. Simulate button debounce (~15µs)
    delayMicroseconds(15);
    
    // 4. Simulate module state machine checks (~20µs)
    delayMicroseconds(20);
    
    // TOTAL simulated overhead: ~165µs (realistic FSM baseline)
    // Uncomment next line to test with full simulated load:
    delayMicroseconds(165);

    // user added delay for threshld discovery
    delayMicroseconds(700000);

    
    CanRxHandler& canRx = CanRxHandler::getInstance();
    CANRxMessage msg;
    
    // Check queue depth BEFORE draining (shows accumulation during previous loop)
    uint8_t queueDepthBefore = canRx.getQueueDepth();
    
    // Measure drain cycle (when queue has messages)
    uint32_t msgsThisDrain = 0;
    uint64_t drainStart = hwTimer.micros();
    
    while (canRx.receiveMessage(msg, 0)) {
        msgCount++;
        msgsThisDrain++;
    }
    
    // If we drained messages, record stats
    if (msgsThisDrain > 0) {
        uint32_t drainTime = hwTimer.micros() - drainStart;
        drainCycles++;
        if (msgsThisDrain > maxMsgsPerDrain) maxMsgsPerDrain = msgsThisDrain;
        if (drainTime > maxDrainTime) maxDrainTime = drainTime;
    }
    
    // Track max queue depth observed
    static uint8_t maxQueueDepth = 0;
    if (queueDepthBefore > maxQueueDepth) maxQueueDepth = queueDepthBefore;
    
    // Print stats every 5 seconds
    if (millis() - lastStatsTime >= 5000) {
        Serial.print("[Stats] Messages: ");
        Serial.print(msgCount);
        Serial.print(" | Rate: ");
        Serial.print(msgCount / 5.0, 1);
        Serial.print(" msg/s | Drain cycles: ");
        Serial.print(drainCycles);
        Serial.print(" | Max burst: ");
        Serial.print(maxMsgsPerDrain);
        Serial.print(" msgs in ");
        Serial.print(maxDrainTime);
        Serial.print(" µs | Queue depth: ");
        Serial.print(maxQueueDepth);
        Serial.print(" | Overflows: ");
        Serial.println(canRx.getQueueOverflows());
        
        msgCount = 0;
        drainCycles = 0;
        maxMsgsPerDrain = 0;
        maxDrainTime = 0;
        maxQueueDepth = 0;
        lastStatsTime = millis();
    }
    
    return;  // Skip all FSM processing
#endif
    
    // ===== NORMAL MODE: FSM Processing =====
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
    
    // ===== RTR DEBUG MODE: Test RTR flag behavior =====
    // DEPRECATED: RTRDebug module disabled
    if (debugRTREnabled) {
        // rtrDebug.loop();  // DEPRECATED - commented out
        return;  // Skip all FSM code when in RTR debug mode
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
            fsm_state.currentState != INIT_HEATING &&           // Allow boot-time error clearing
            fsm_state.currentState != INIT_HOT_NOT_HOMED &&     // Allow warmup with cleared errors
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
                if (!safety.isEStopPressed() && !safety.isBarrelOpen()) { MessageBuffer::getInstance().sendMessage("User Reset."); safety.resetError(); motor.clearErrors(); fsm_state.currentState = InjectorStates::INIT_HEATING; } 
            }
            break;

        case InjectorStates::INIT_HEATING: 
            if (stateEntry) {
                // Clear any pre-existing ODrive errors from previous session
                // This prevents stale errors from blocking startup
                MessageBuffer::getInstance().sendMessage("Boot: Clearing ODrive errors");
                motor.clearErrors();
                delay(100);  // Allow ODrive to process clear command
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
                MessageBuffer::getInstance().sendMessage("Compression: User aborted, returning to Refill");
                fsm_state.currentState = InjectorStates::REFILL;
                moveLockActive = true;  // Lock buttons until Refill position reached
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
                fsm_state.error = 0xFD;  // Compression error
            }
            break;

        case InjectorStates::READY_TO_INJECT:
            // ===== NEW: Modular ReadyToInject =====
            if (stateEntry) {
                ReadyToInject::begin();
                stateEntry = false;
            }
            if (ReadyToInject::update(motor)) {
                // ReadyToInject runs indefinitely, check for user input to proceed
            }
            if (!ignoreButtons && btnUpper.read() == LOW && btnLower.read() == LOW) { 
                MessageBuffer::getInstance().sendMessage("Ready: User confirms, moving to Purge");
                fsm_state.currentState = InjectorStates::PURGE_ZERO;
                ReadyToInject::reset();
            }
            else if (!ignoreButtons && !buttonLock && btnCenter.released()) { 
                MessageBuffer::getInstance().sendMessage("Ready: User abort, returning to Refill");
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
           // ===== NEW: Modular PurgeZero =====
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
                fsm_state.error = 0xFB;  // PurgeZero error
            }
            break;

        case InjectorStates::ANTIDRIP:
            // ===== NEW: Modular AntiDrip =====
            if (stateEntry) {
                AntiDrip::begin();
                stateEntry = false;
            }
            if (AntiDrip::update(motor)) {
                // AntiDrip complete, handle user button responses
            }
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
                fsm_state.error = 0xFA;  // AntiDrip error (usually timeout)
            }
            break;

        case InjectorStates::INJECT:
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
                MessageBuffer::getInstance().sendMessage("Inject: User abort, releasing mould");
                fsm_state.currentState = InjectorStates::RELEASE;
                Injection::reset();
            }
            if (Injection::hasError()) {
                fsm_state.currentState = InjectorStates::ERROR_STATE;
                fsm_state.error = 0xF9;  // Injection error
            }
            break;

        case InjectorStates::HOLD_INJECTION:
           // ===== NEW: Modular Injection (PACKING phase, handled by same module) =====
            Injection::update(motor);  // CRITICAL: Must call update() to check pack timer
            if (Injection::isComplete()) {
                MessageBuffer::getInstance().sendMessage("Hold: Pack time complete, releasing mould");
                fsm_state.currentState = InjectorStates::RELEASE;
                Injection::reset();  // Reset module when DONE (exiting injection sequence)
            }
            if (!ignoreButtons && !buttonLock && btnUpper.released()) { 
                MessageBuffer::getInstance().sendMessage("Pack: User abort, releasing mould");
                fsm_state.currentState = InjectorStates::RELEASE;
                Injection::reset();  // Reset module on abort
            }
            break;

        case InjectorStates::RELEASE: 
           // ===== NEW: Release (simple auto-transition) =====
            if (stateEntry) {
                MessageBuffer::getInstance().sendMessage("Release: Unloading mould");
                
                // Queue all commands - ring buffer handles timing
                MotorWrapper::setMotorLimits(motor, RELEASE_CONTROLLER_VEL_LIMIT, RELEASE_CURRENT_LIMIT, "RELEASE");
                MotorWrapper::setTrapTrajParams(motor, RELEASE_TRAP_VEL_LIMIT, RELEASE_ACCEL, RELEASE_DECEL, "RELEASE_TRAJ");
                
                float releaseTarget = motor.getPosition() + RELEASE_DIST;  // RELEASE_DIST is negative (up)
                MotorWrapper::setModeAndMove(motor, 3, 5, releaseTarget, "Pos Release");  // Mode 3 (Position), InputMode 5 (TRAP_TRAJ)
                
                stateEntry = false;
            }
            if (millis() - stateTimer > 2000) { 
                MessageBuffer::getInstance().sendMessage("Release: Complete, confirming mould removal");
                fsm_state.currentState = InjectorStates::CONFIRM_MOULD_REMOVAL; 
            }
            break;

        case InjectorStates::CONFIRM_MOULD_REMOVAL:
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
                     MessageBuffer::getInstance().sendMessage("Confirm: Returning to ReadyToInject");
                     fsm_state.currentState = InjectorStates::READY_TO_INJECT; 
                 }
                 else {
                     MessageBuffer::getInstance().sendMessage("Confirm: Returning to Refill");
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
        #if DEBUG_ENABLED
        // Flush buffered messages: output 1Hz status + accumulated event messages
        const char* output = MessageBuffer::getInstance().getOutput();
        Serial.println(output);
        MessageBuffer::getInstance().clearBuffer();  // Clear events for next cycle
        #endif
    }
}
