#include "TimingSystemTest.h"
#include "BroadcastDataStore.h"
#include "GPTimer.h"
#include "MessageBuffer.h"
#include "config.h"

TimingSystemTest::TimingSystemTest() {
    enabled_ = false;
    testStartTime_ = 0;
    baselineCaptured_ = false;
    baselinePos_ = 0.0f;
    baselineIq_ = 0.0f;
}

void TimingSystemTest::begin() {
    Serial.println("\n=== Timing System Test ===");
    Serial.println("Testing new BroadcastDataStore timing accessors");
    Serial.println("Type 'timing_start' to begin timing analysis");
    Serial.println("Type 'timing_baseline' to capture movement baseline");
    Serial.println("Type 'timing_check' to check movement detection");
    Serial.println("Type 'timing_window' to test command windows");
    Serial.println("========================\n");
}

void TimingSystemTest::handleCommand(const String& command) {
    if (command == "timing_start") {
        startTest();
    }
    else if (command == "timing_baseline") {
        captureBaseline();
    }
    else if (command == "timing_check") {
        checkMovement();
    }
    else if (command == "timing_window") {
        testCommandWindow();
    }
    else if (command == "timing_debug") {
        printDebugData();
    }
    else if (command == "timing_status") {
        showStatus();
    }
}

void TimingSystemTest::startTest() {
    enabled_ = true;
    testStartTime_ = hwTimer.micros();
    baselineCaptured_ = false;
    
    Serial.println("[TIMING] Test started");
    Serial.print("[TIMING] Test start time: ");
    Serial.println(testStartTime_);
    Serial.println("[TIMING] Use 'timing_baseline' to capture baseline");
}

void TimingSystemTest::captureBaseline() {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    
    float iqSetpoint, iqMeasured, position, velocity;
    if (bds.getMovementData(iqSetpoint, iqMeasured, position, velocity)) {
        baselinePos_ = position;
        baselineIq_ = iqMeasured;
        baselineBusCurrent_ = bds.getBusCurrent(); // Get BUS current separately
        baselineCaptured_ = true;
        
        uint64_t encoderTime = bds.getEncoderTimestamp();
        uint64_t iqTime = bds.getIqTimestamp();
        
        Serial.println("[TIMING] Baseline captured:");
        Serial.print("  Position: ");
        Serial.print(position, 6);
        Serial.println(" turns");
        Serial.print("  IQ Measured: ");
        Serial.print(iqMeasured, 3);
        Serial.println(" A");
        Serial.print("  IQ Setpoint: ");
        Serial.print(iqSetpoint, 3);
        Serial.println(" A");
        Serial.print("  BUS Current: ");
        Serial.print(baselineBusCurrent_, 3);
        Serial.println(" A");
        Serial.print("  Velocity: ");
        Serial.print(velocity, 3);
        Serial.println(" turns/s");
        Serial.print("  Encoder timestamp: ");
        Serial.println(encoderTime);
        Serial.print("  IQ timestamp: ");
        Serial.println(iqTime);
        Serial.print("  Current time: ");
        Serial.println(hwTimer.micros());
        
        Serial.println("[TIMING] Baseline ready - move motor and use 'timing_check'");
        Serial.println("[TIMING] Compare: IQ_set (command) vs IQ_measured (actual) vs BUS_current (real-time)");
    } else {
        Serial.println("[TIMING] ERROR: No fresh data available");
    }
}

void TimingSystemTest::checkMovement() {
    if (!baselineCaptured_) {
        Serial.println("[TIMING] ERROR: No baseline captured - use 'timing_baseline' first");
        return;
    }
    
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    
    float iqSetpoint, iqMeasured, position, velocity;
    if (bds.getMovementData(iqSetpoint, iqMeasured, position, velocity)) {
        float currentBusCurrent = bds.getBusCurrent();
        
        Serial.println("[TIMING] Movement check:");
        Serial.print("  Current position: ");
        Serial.print(position, 6);
        Serial.println(" turns");
        Serial.print("  Baseline position: ");
        Serial.print(baselinePos_, 6);
        Serial.println(" turns");
        Serial.print("  Position delta: ");
        Serial.print(fabs(position - baselinePos_), 6);
        Serial.println(" turns");
        
        Serial.println("[TIMING] Current comparison:");
        Serial.print("  IQ_set (command): ");
        Serial.print(iqSetpoint, 3);
        Serial.print(" A (delta: ");
        Serial.print(fabs(iqSetpoint - baselineIq_), 3);
        Serial.println(" A)");
        Serial.print("  IQ_measured (actual): ");
        Serial.print(iqMeasured, 3);
        Serial.print(" A (delta: ");
        Serial.print(fabs(iqMeasured - baselineIq_), 3);
        Serial.println(" A)");
        Serial.print("  BUS_current (real-time): ");
        Serial.print(currentBusCurrent, 3);
        Serial.print(" A (delta: ");
        Serial.print(fabs(currentBusCurrent - baselineBusCurrent_), 3);
        Serial.println(" A)");
        
        // Check thresholds (convert to proper units)
        float posThreshold = (float)MOVEMENT_POS_THRESHOLD_TICKS / 8192.0f;  // Convert ticks to turns
        float iqThreshold = (float)MOVEMENT_IQ_THRESHOLD_MA / 1000.0f;       // Convert mA to A
        
        bool posMoved = fabs(position - baselinePos_) > posThreshold;
        bool iqSetMoved = fabs(iqSetpoint - baselineIq_) > iqThreshold;
        bool iqMeasMoved = fabs(iqMeasured - baselineIq_) > iqThreshold;
        bool busMoved = fabs(currentBusCurrent - baselineBusCurrent_) > iqThreshold;
        
        Serial.println("[TIMING] Detection results:");
        Serial.print("  Position movement: ");
        Serial.println(posMoved ? "DETECTED" : "none");
        Serial.print("  IQ_set movement: ");
        Serial.println(iqSetMoved ? "DETECTED" : "none");
        Serial.print("  IQ_measured movement: ");
        Serial.println(iqMeasMoved ? "DETECTED" : "none");
        Serial.print("  BUS_current movement: ");
        Serial.println(busMoved ? "DETECTED" : "none");
        
        // Show thresholds for debugging
        Serial.println("[TIMING] Thresholds used:");
        Serial.print("  Position threshold: ");
        Serial.print(posThreshold, 6);
        Serial.println(" turns");
        Serial.print("  IQ threshold: ");
        Serial.print(iqThreshold, 3);
        Serial.println(" A");
        
        if (posMoved || iqSetMoved || iqMeasMoved || busMoved) {
            Serial.println("[TIMING] MOVEMENT DETECTED!");
        } else {
            Serial.println("[TIMING] No movement detected");
        }
    } else {
        Serial.println("[TIMING] ERROR: No fresh data available");
    }
}

void TimingSystemTest::printDebugData() {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    
    Serial.println("[TIMING] Debug - Raw data store status:");
    
    // Check struct sizes and alignment
    Serial.println("  Struct alignment checks:");
    Serial.print("    TimestampedEncoder size: ");
    Serial.println(sizeof(TimestampedEncoder));
    Serial.print("    TimestampedIq size: ");
    Serial.println(sizeof(TimestampedIq));
    Serial.print("    float size: ");
    Serial.println(sizeof(float));
    Serial.print("    uint64_t size: ");
    Serial.println(sizeof(uint64_t));
    Serial.print("    bool size: ");
    Serial.println(sizeof(bool));
    
    // Check latest encoder data
    const TimestampedEncoder* encData = bds.getLatestEncoder();
    if (encData) {
        uint64_t currentTime = hwTimer.micros();
        uint64_t age = currentTime - encData->timestamp;
        
        Serial.println("  Encoder data:");
        Serial.print("    Position: ");
        Serial.print(encData->position, 6);
        Serial.println(" turns");
        Serial.print("    Velocity: ");
        Serial.print(encData->velocity, 3);
        Serial.println(" turns/s");
        Serial.print("    Raw position bytes: ");
        uint32_t* posBytes = (uint32_t*)&encData->position;
        Serial.print("0x");
        if (*posBytes < 0x1000) Serial.print("0");
        if (*posBytes < 0x100) Serial.print("0");
        if (*posBytes < 0x10) Serial.print("0");
        Serial.print(*posBytes, HEX);
        Serial.println();
        Serial.print("    Raw velocity bytes: ");
        uint32_t* velBytes = (uint32_t*)&encData->velocity;
        Serial.print("0x");
        if (*velBytes < 0x1000) Serial.print("0");
        if (*velBytes < 0x100) Serial.print("0");
        if (*velBytes < 0x10) Serial.print("0");
        Serial.print(*velBytes, HEX);
        Serial.println();
        Serial.print("    Timestamp: ");
        Serial.println(encData->timestamp);
        Serial.print("    Age: ");
        Serial.print(age);
        Serial.println(" us");
        Serial.print("    Fresh: ");
        Serial.println(age < 50000 ? "YES" : "NO");
    } else {
        Serial.println("  Encoder data: NULL");
    }
    
    // Check latest IQ data
    const TimestampedIq* iqData = bds.getLatestIq();
    if (iqData) {
        uint64_t currentTime = hwTimer.micros();
        uint64_t age = currentTime - iqData->timestamp;
        
        Serial.println("  IQ data:");
        Serial.print("    IQ Setpoint: ");
        Serial.print(iqData->iqSetpoint, 3);
        Serial.println(" A");
        Serial.print("    IQ Measured: ");
        Serial.print(iqData->iqMeasured, 3);
        Serial.println(" A");
        Serial.print("    Raw setpoint bytes: ");
        uint32_t* setBytes = (uint32_t*)&iqData->iqSetpoint;
        Serial.print("0x");
        if (*setBytes < 0x1000) Serial.print("0");
        if (*setBytes < 0x100) Serial.print("0");
        if (*setBytes < 0x10) Serial.print("0");
        Serial.print(*setBytes, HEX);
        Serial.println();
        Serial.print("    Raw measured bytes: ");
        uint32_t* measBytes = (uint32_t*)&iqData->iqMeasured;
        Serial.print("0x");
        if (*measBytes < 0x1000) Serial.print("0");
        if (*measBytes < 0x100) Serial.print("0");
        if (*measBytes < 0x10) Serial.print("0");
        Serial.print(*measBytes, HEX);
        Serial.println();
        Serial.print("    Timestamp: ");
        Serial.println(iqData->timestamp);
        Serial.print("    Age: ");
        Serial.print(age);
        Serial.println(" us");
        Serial.print("    Fresh: ");
        Serial.println(age < 50000 ? "YES" : "NO");
    } else {
        Serial.println("  IQ data: NULL");
    }
    
    // Check BUS data
    Serial.println("  BUS data:");
    Serial.print("    BUS Current: ");
    Serial.print(bds.getBusCurrent(), 3);
    Serial.println(" A");
    Serial.print("    BUS Voltage: ");
    Serial.print(bds.getBusVoltage(), 1);
    Serial.println(" V");
    
    Serial.println("[TIMING] Debug complete");
}

void TimingSystemTest::testCommandWindow() {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    uint64_t currentTime = hwTimer.micros();
    
    Serial.println("[TIMING] Command Window Test:");
    Serial.print("  Current time: ");
    Serial.println(currentTime);
    
    uint64_t encoderTime = bds.getEncoderTimestamp();
    Serial.print("  Last encoder time: ");
    Serial.println(encoderTime);
    
    uint32_t offset = bds.getTimingOffset(currentTime);
    Serial.print("  Timing offset: ");
    Serial.print(offset);
    Serial.println(" us");
    
    bool inWindow = bds.isInCommandWindow(currentTime);
    Serial.print("  In command window: ");
    Serial.println(inWindow ? "YES" : "NO");
    
    // Test custom windows
    Serial.println("  Testing custom windows:");
    for (uint32_t testOffset = 2000; testOffset <= 8000; testOffset += 2000) {
        bool inCustomWindow = bds.isInCommandWindow(currentTime, testOffset, 1000);
        Serial.print("    Offset ");
        Serial.print(testOffset);
        Serial.print("us: ");
        Serial.println(inCustomWindow ? "IN" : "OUT");
    }
    
    Serial.println("[TIMING] Command window test complete");
}

void TimingSystemTest::showStatus() {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    
    Serial.println("[TIMING] System Status:");
    Serial.print("  Test enabled: ");
    Serial.println(enabled_ ? "YES" : "NO");
    Serial.print("  Baseline captured: ");
    Serial.println(baselineCaptured_ ? "YES" : "NO");
    
    bool dataFresh = bds.isDataFresh();
    Serial.print("  Data fresh: ");
    Serial.println(dataFresh ? "YES" : "NO");
    
    uint64_t encoderTime = bds.getEncoderTimestamp();
    uint64_t iqTime = bds.getIqTimestamp();
    uint64_t currentTime = hwTimer.micros();
    
    Serial.print("  Encoder age: ");
    Serial.print(encoderTime > 0 ? (currentTime - encoderTime) : 0);
    Serial.println(" us");
    
    Serial.print("  IQ age: ");
    Serial.print(iqTime > 0 ? (currentTime - iqTime) : 0);
    Serial.println(" us");
    
    Serial.println("[TIMING] Status complete");
}
