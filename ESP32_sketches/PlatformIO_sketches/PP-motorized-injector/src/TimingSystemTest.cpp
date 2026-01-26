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
        baselineCaptured_ = true;
        
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
        Serial.print("  Velocity: ");
        Serial.print(velocity, 3);
        Serial.println(" turns/s");
        
        // Show timestamps
        uint64_t encoderTime = bds.getEncoderTimestamp();
        uint64_t iqTime = bds.getIqTimestamp();
        Serial.print("  Encoder timestamp: ");
        Serial.println(encoderTime);
        Serial.print("  IQ timestamp: ");
        Serial.println(iqTime);
        Serial.print("  Current time: ");
        Serial.println(hwTimer.micros());
        
        Serial.println("[TIMING] Baseline ready - move motor and use 'timing_check'");
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
        
        Serial.print("  Current IQ: ");
        Serial.print(iqMeasured, 3);
        Serial.println(" A");
        Serial.print("  Baseline IQ: ");
        Serial.print(baselineIq_, 3);
        Serial.println(" A");
        Serial.print("  IQ delta: ");
        Serial.print(fabs(iqMeasured - baselineIq_), 3);
        Serial.println(" A");
        
        // Check thresholds
        float posThreshold = (float)MOVEMENT_POS_THRESHOLD_TICKS/8192.0f;
        float iqThreshold = (float)MOVEMENT_IQ_THRESHOLD_MA/1000.0f;
        
        Serial.print("  Position threshold: ");
        Serial.print(posThreshold, 6);
        Serial.println(" turns");
        Serial.print("  IQ threshold: ");
        Serial.print(iqThreshold, 3);
        Serial.println(" A");
        
        bool movementDetected = bds.hasMovementOccurred(baselinePos_, baselineIq_, posThreshold, iqThreshold);
        Serial.print("  Movement detected: ");
        Serial.println(movementDetected ? "YES" : "NO");
        
        // Show data freshness
        bool dataFresh = bds.isDataFresh();
        Serial.print("  Data fresh: ");
        Serial.println(dataFresh ? "YES" : "NO");
        
    } else {
        Serial.println("[TIMING] ERROR: No fresh data available");
    }
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
