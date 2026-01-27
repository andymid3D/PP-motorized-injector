#include "TimingSystemTest.h"
#include "BroadcastDataStore.h"
#include "CanBusHandlerV2.h"
#include "ODriveCANProtocol.h"
#include "GPTimer.h"
#include "OurLoopTimer.h"  // For toggleLoopFlag function
#include "MessageBuffer.h"
#include "config.h"
#include <climits>  // For LLONG_MAX

TimingSystemTest::TimingSystemTest() 
    : enabled_(false), testStartTime_(0), baselineCaptured_(false), 
      baselinePos_(0.0f), baselineIq_(0.0f), can_(nullptr),
      testPhase_(TEST_PHASE_PRE_ROLL), captureStartTime_(0), 
      commandSendTime_(0), phaseStartTime_(0), collectionEndTime_(0),
      testActive_(false) {
}

void TimingSystemTest::begin() {
    Serial.println("\n=== Timing System Test ===");
    Serial.println("Testing new BroadcastDataStore timing accessors");
    Serial.println("Commands: timing_start, timing_baseline, timing_check, timing_debug, timing_status, timing_window, timing_auto, timing_iq_history");
    Serial.println("Features: Position + IQ current movement detection (BUS current disabled)");
    Serial.println("NOTE: Use begin(CanBusHandlerV2) for full command sending capability");
    Serial.println("========================\n");
}

void TimingSystemTest::begin(CanBusHandlerV2& canHandler) {
    can_ = &canHandler;
    begin();
    Serial.println("[TIMING] CAN command sending ENABLED");
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
    else if (command == "timing_debug") {
        printDebugData();
    }
    else if (command == "timing_status") {
        showStatus();
    }
    else if (command == "timing_window") {
        testCommandWindow();
    }
    else if (command == "timing_auto") {
        runAutoCalibration();
    }
    else if (command == "timing_iq_history") {
        showIQHistory(500);
    }
    else {
        Serial.println("[TIMING] Unknown command. Available: timing_start, timing_baseline, timing_check, timing_debug, timing_status, timing_window, timing_auto, timing_iq_history");
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
    bool hasData = bds.getMovementData(iqSetpoint, iqMeasured, position, velocity);
    
    // Try to get latest data even if not fresh
    if (!hasData) {
        Serial.println("[TIMING] No fresh data available, trying latest data...");
        const TimestampedIq* iqData = bds.getLatestIq();
        const TimestampedEncoder* encData = bds.getLatestEncoder();
        
        if (iqData && encData) {
            iqSetpoint = iqData->iqSetpoint;
            iqMeasured = iqData->iqMeasured;
            position = encData->position;
            velocity = encData->velocity;
            hasData = true;
            
            uint64_t currentTime = hwTimer.micros();
            uint64_t iqAge = currentTime - iqData->timestamp;
            uint64_t encAge = currentTime - encData->timestamp;
            
            Serial.printf("[TIMING] Using stale data - IQ age: %llu us, Encoder age: %llu us\n", iqAge, encAge);
        }
    }
    
    if (hasData) {
        baselinePos_ = position;
        baselineIq_ = iqMeasured;
        // BUS current disabled - unreliable sensor
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
        Serial.println("  BUS Current: DISABLED (unreliable)");
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
        Serial.println("[TIMING] ERROR: No data available - check CAN connection");
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
        Serial.println("  BUS_current: DISABLED (unreliable sensor)");
        
        // Check thresholds (convert to proper units)
        float posThreshold = (float)MOVEMENT_POS_THRESHOLD_TICKS / 8192.0f;  // Convert ticks to turns
        float iqThreshold = (float)MOVEMENT_IQ_THRESHOLD_MA / 1000.0f;       // Convert mA to A
        
        bool posMoved = fabs(position - baselinePos_) > posThreshold;
        bool iqSetMoved = fabs(iqSetpoint - baselineIq_) > iqThreshold;
        bool iqMeasMoved = fabs(iqMeasured - baselineIq_) > iqThreshold;
        
        Serial.println("[TIMING] Detection results:");
        Serial.print("  Position movement: ");
        Serial.println(posMoved ? "DETECTED" : "none");
        Serial.print("  IQ_set movement: ");
        Serial.println(iqSetMoved ? "DETECTED" : "none");
        Serial.print("  IQ_measured movement: ");
        Serial.println(iqMeasMoved ? "DETECTED" : "none");
        
        // Show thresholds for debugging
        Serial.println("[TIMING] Thresholds used:");
        Serial.print("  Position threshold: ");
        Serial.print(posThreshold, 6);
        Serial.println(" turns");
        Serial.print("  IQ threshold: ");
        Serial.print(iqThreshold, 3);
        Serial.println(" A");
        
        if (posMoved || iqSetMoved || iqMeasMoved) {
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
        
    Serial.println("[TIMING] Command window test complete");
}

void TimingSystemTest::runAutoCalibration() {
    Serial.println("[TIMING] === MOVEMENT TEST SEQUENCE ===");
    
    if (!can_) {
        Serial.println("[TIMING] ERROR: No CAN handler available - use begin(CanBusHandlerV2)");
        return;
    }
    
    // Step 1: Capture baseline
    Serial.println("[TIMING] Step 1: Capturing baseline...");
    captureBaseline();
    
    // Step 2: Set controller modes for position control
    Serial.println("[TIMING] Step 2: Setting position control mode...");
    if (can_->setControllerModes(ODriveCANProtocol::ControlMode::POSITION_CONTROL, 
                                  ODriveCANProtocol::InputMode::PASSTHROUGH)) {
        Serial.println("[TIMING] ✓ Position control mode set successfully");
    } else {
        Serial.println("[TIMING] ✗ Failed to set position control mode");
        return;
    }
    
    // Step 3: Start buffer collection and wait 1s pre-roll
    Serial.println("[TIMING] Step 3: Starting buffer collection...");
    captureStartTime_ = hwTimer.micros();
    
    // Set pre-roll phase and return immediately
    testPhase_ = TEST_PHASE_PRE_ROLL;
    phaseStartTime_ = captureStartTime_;
    testActive_ = true;
    
    Serial.println("[TIMING] Step 4: Collecting 1000ms pre-roll data...");
    
    // Return immediately - main loop will handle pre-roll timing
    return;
}

void TimingSystemTest::checkTestCompletion() {
    if (!testActive_) return;
    
    uint64_t currentTime = hwTimer.micros();
    
    if (testPhase_ == TEST_PHASE_PRE_ROLL) {
        // Check if pre-roll period is complete (1000ms)
        if (currentTime - phaseStartTime_ >= 1000000) {
            // Send command now
            Serial.println("[TIMING] Pre-roll complete, sending command...");
            Serial.println("[TIMING] Step 5: Sending set_position 1.0...");
            uint64_t commandSendTime = currentTime;
            if (can_->setInputPos(1.0f)) {
                Serial.println("[TIMING] ✓ set_position 1.0 command sent successfully");
                Serial.printf("[TIMING] Command sent at: %llu (T+%llu us from start)\n", 
                             commandSendTime, commandSendTime - captureStartTime_);
                commandSendTime_ = commandSendTime;
            } else {
                Serial.println("[TIMING] ✗ Failed to send set_position 1.0 command");
                testActive_ = false;
                return;
            }
            
            // Move to post-command collection
            testPhase_ = TEST_PHASE_POST_COMMAND;
            Serial.println("[TIMING] Step 6: Continuing collection for 1000ms post-command...");
            collectionEndTime_ = captureStartTime_ + 2000000; // 2s total from start
        }
    } else if (testPhase_ == TEST_PHASE_POST_COMMAND) {
        // Check if full collection period is complete
        if (currentTime >= collectionEndTime_) {
            // Analyze data centered on command time with automatic window adjustment
            Serial.println("[TIMING] Step 7: Analyzing collected data...");
            analyzeCommandCenteredData();
            
            Serial.println("[TIMING] === MOVEMENT TEST COMPLETE ===");
            testActive_ = false;
        }
    }
}

void TimingSystemTest::analyzeCommandCenteredData() {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    
    Serial.println("[TIMING] === PRODUCTION RESPONSE ANALYSIS ===");
    
    // Production parameters
    const uint32_t ANALYSIS_WINDOW_MS = 200;  // 200ms chase window
    const float SPIKE_THRESHOLD_A = 5.0f;       // 5A spike detection threshold
    const uint32_t SEARCH_START_MS = 0;         // Start chase immediately from command
    const bool USE_SETPOINT_FOR_DETECTION = true; // Use setpoint (faster) vs measured
    const uint32_t CONFIRMATION_WINDOW_MS = 50;  // 50ms confirmation after spike
    
    Serial.printf("[TIMING] Command: %llu, Window: %dms, Threshold: %.1fA\n", 
                 commandSendTime_, ANALYSIS_WINDOW_MS, SPIKE_THRESHOLD_A);
    
    // Chase window: command+0ms to command+200ms
    uint64_t chaseStart = commandSendTime_ + (SEARCH_START_MS * 1000);
    uint64_t chaseEnd = commandSendTime_ + (ANALYSIS_WINDOW_MS * 1000);
    
    // Production analysis variables
    float maxIQMeasured = 0.0f;
    float maxIQSetpoint = 0.0f;
    uint64_t spikeTime = 0;
    bool spikeDetected = false;
    int dataPoints = 0;
    float baselineIQ = 0.0f;
    bool baselineFound = false;
    
    Serial.printf("[TIMING] Chasing from %llu to %llu...\n", chaseStart, chaseEnd);
    Serial.println("[TIMING] Command-centered data:");
    
    // Collect all valid data points first, then sort by time
    struct TimeOrderedData {
        uint64_t timestamp;
        float iqSetpoint;
        float iqMeasured;
        int32_t relativeTime;
    };
    
    TimeOrderedData validData[50];  // Max 50 points in 200ms at 10ms rate
    int validCount = 0;
    
    // First pass: collect all data in window
    for (size_t i = 0; i < BDS_IQ_HISTORY_SIZE && validCount < 50; i++) {
        const TimestampedIq* iqData = bds.getHistoryIq(i);
        if (!iqData) continue;
        
        // Only chase data in our window (always positive time from command)
        if (iqData->timestamp < chaseStart || iqData->timestamp > chaseEnd) {
            continue;
        }
        
        // Calculate relative time (always positive - no overflow issues!)
        int32_t relativeTime = (int32_t)((int64_t)(iqData->timestamp - commandSendTime_) / 1000);
        
        // Store for time-ordered processing
        validData[validCount].timestamp = iqData->timestamp;
        validData[validCount].iqSetpoint = iqData->iqSetpoint;
        validData[validCount].iqMeasured = iqData->iqMeasured;
        validData[validCount].relativeTime = relativeTime;
        validCount++;
    }
    
    // Sort by relative time (chronological order)
    for (int i = 0; i < validCount - 1; i++) {
        for (int j = i + 1; j < validCount; j++) {
            if (validData[i].relativeTime > validData[j].relativeTime) {
                TimeOrderedData temp = validData[i];
                validData[i] = validData[j];
                validData[j] = temp;
            }
        }
    }
    
    // Second pass: process in chronological order
    for (int i = 0; i < validCount; i++) {
        // Establish baseline from FIRST chronological data point
        if (!baselineFound) {
            baselineIQ = USE_SETPOINT_FOR_DETECTION ? 
                        fabs(validData[i].iqSetpoint) : 
                        fabs(validData[i].iqMeasured);
            baselineFound = true;
            Serial.printf("[TIMING] Baseline: %.3fA at T+%dms (%s)\n", 
                         baselineIQ, validData[i].relativeTime,
                         USE_SETPOINT_FOR_DETECTION ? "setpoint" : "measured");
        }
        
        // Show ALL data points in chronological order
        Serial.printf("  T+%dms: set=%.1fA, meas=%.1fA (IQts=%llu)\n", 
                     validData[i].relativeTime, validData[i].iqSetpoint, 
                     validData[i].iqMeasured, validData[i].timestamp);
        
        // Spike detection: first significant increase from baseline
        float currentValue = USE_SETPOINT_FOR_DETECTION ? 
                           fabs(validData[i].iqSetpoint) : 
                           fabs(validData[i].iqMeasured);
        
        if (!spikeDetected && currentValue > (baselineIQ + SPIKE_THRESHOLD_A)) {
            spikeDetected = true;
            spikeTime = validData[i].timestamp;
            maxIQMeasured = fabs(validData[i].iqMeasured);
            maxIQSetpoint = fabs(validData[i].iqSetpoint);
            
            Serial.printf("[TIMING] ✓ SPIKE DETECTED at T+%dms: %.1fA (%s, baseline: %.1fA)\n", 
                         validData[i].relativeTime, currentValue,
                         USE_SETPOINT_FOR_DETECTION ? "setpoint" : "measured", 
                         baselineIQ);
                         
            // Production: Could exit early here, but continue to confirm
            Serial.printf("[TIMING] Continuing for %dms confirmation...\n", CONFIRMATION_WINDOW_MS);
        }
        
        // Track maximum values (always track both for reporting)
        if (fabs(validData[i].iqMeasured) > maxIQMeasured) {
            maxIQMeasured = fabs(validData[i].iqMeasured);
        }
        if (fabs(validData[i].iqSetpoint) > maxIQSetpoint) {
            maxIQSetpoint = fabs(validData[i].iqSetpoint);
        }
        
        dataPoints++;
    }
    
    // Production results output
    Serial.println("[TIMING] === ANALYSIS RESULTS ===");
    
    if (spikeDetected) {
        uint32_t responseLatencyMs = (spikeTime - commandSendTime_) / 1000;
        
        Serial.printf("[TIMING] Response: DETECTED\n");
        Serial.printf("[TIMING] Latency: %dms\n", responseLatencyMs);
        Serial.printf("[TIMING] Peak Current: %.1fA\n", maxIQMeasured);
        Serial.printf("[TIMING] Peak Setpoint: %.1fA\n", maxIQSetpoint);
        Serial.printf("[TIMING] Data Points: %d\n", dataPoints);
        Serial.printf("[TIMING] Status: SUCCESS\n");
        
        // Structured data for automation
        Serial.printf("[DATA] LATENCY=%d,PEAK=%.1f,SETPOINT=%.1f,POINTS=%d,STATUS=OK\n",
                     responseLatencyMs, maxIQMeasured, maxIQSetpoint, dataPoints);
        
    } else {
        Serial.printf("[TIMING] Response: NOT DETECTED\n");
        Serial.printf("[TIMING] Baseline: %.1fA\n", baselineIQ);
        Serial.printf("[TIMING] Threshold: %.1fA\n", SPIKE_THRESHOLD_A);
        Serial.printf("[TIMING] Max Found: %.1fA\n", maxIQMeasured);
        Serial.printf("[TIMING] Data Points: %d\n", dataPoints);
        Serial.printf("[TIMING] Status: NO_RESPONSE\n");
        
        // Structured data for automation
        Serial.printf("[DATA] LATENCY=-1,PEAK=%.1f,SETPOINT=%.1f,POINTS=%d,STATUS=NO_RSP\n",
                     maxIQMeasured, maxIQSetpoint, dataPoints);
    }
    
    Serial.println("[TIMING] === END PRODUCTION ANALYSIS ===");
}

void TimingSystemTest::showIQHistoryFromTime(uint64_t startTime, uint32_t durationMs) {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
        
    Serial.print("[TIMING] IQ History (from command send, last ");
    Serial.print(durationMs);
    Serial.println("ms):");
        
    uint64_t currentTime = hwTimer.micros();
    uint64_t endTime = startTime + (durationMs * 1000);
        
    Serial.printf("[TIMING] Capture start time: %llu, Current time: %llu, Window: %llu us\n", 
                 startTime, currentTime, durationMs * 1000);
    Serial.printf("[TIMING] Ring buffer size: %d samples (max 1s at 100ms intervals)\n", BDS_IQ_HISTORY_SIZE);
    
    // Check total available data first
    int totalAvailable = 0;
    for (size_t i = 0; i < BDS_IQ_HISTORY_SIZE; i++) {
        if (bds.getHistoryIq(i)) totalAvailable++;
    }
    Serial.printf("[TIMING] Total IQ samples available: %d\n", totalAvailable);
        
    // Get historical data from ring buffer
    float maxSetpoint = 0.0f;
    float maxMeasured = 0.0f;
    int dataPoints = 0;
    uint64_t latestTimestamp = 0;
    uint64_t oldestTimestamp = UINT64_MAX;
        
    // Iterate through ALL IQ history to find data in our window
    for (size_t i = 0; i < BDS_IQ_HISTORY_SIZE; i++) {  // Use actual ring buffer size
        const TimestampedIq* iqData = bds.getHistoryIq(i);
        if (!iqData) {
            if (i < 20) Serial.printf("[TIMING] No IQ data at index %zu\n", i);
            continue;
        }
        
        // Check if data is in our time window
        if (iqData->timestamp < startTime || iqData->timestamp > endTime) {
            continue;
        }
        
        // Get corresponding encoder data
        const TimestampedEncoder* encData = nullptr;
        uint64_t encTimestamp = 0;
        float position = 0.0f, velocity = 0.0f;
        int64_t smallestDt = LLONG_MAX;
        
        // Find encoder data closest to IQ timestamp
        for (size_t j = 0; j < BDS_ENCODER_HISTORY_SIZE; j++) {
            const TimestampedEncoder* testEnc = bds.getHistoryEncoder(j);
            if (!testEnc) break;
            
            int64_t dt = (int64_t)(testEnc->timestamp - iqData->timestamp);
            if (llabs(dt) < llabs(smallestDt)) {
                smallestDt = dt;
                encData = testEnc;
                encTimestamp = testEnc->timestamp;
                position = testEnc->position;
                velocity = testEnc->velocity;
            }
        }
            
        uint32_t ageFromCommand = (iqData->timestamp - startTime) / 1000;
        if (encData) {
            Serial.printf("  T+%ums: set=%.3fA, meas=%.3fA, pos=%.6f, vel=%.3f (IQts=%llu, ENcts=%llu, dt=%lld)\n", 
                         ageFromCommand, iqData->iqSetpoint, iqData->iqMeasured, 
                         position, velocity, iqData->timestamp, encTimestamp, 
                         (int64_t)(iqData->timestamp - encTimestamp));
        } else {
            Serial.printf("  T+%ums: set=%.3fA, meas=%.3fA, pos=---, vel=--- (IQts=%llu)\n", 
                         ageFromCommand, iqData->iqSetpoint, iqData->iqMeasured, iqData->timestamp);
        }
            
        // Track timing
        if (iqData->timestamp > latestTimestamp) latestTimestamp = iqData->timestamp;
        if (iqData->timestamp < oldestTimestamp) oldestTimestamp = iqData->timestamp;
            
        // Track max values
        if (fabs(iqData->iqSetpoint) > fabs(maxSetpoint)) {
            maxSetpoint = iqData->iqSetpoint;
        }
        if (fabs(iqData->iqMeasured) > fabs(maxMeasured)) {
            maxMeasured = iqData->iqMeasured;
        }
        dataPoints++;
    }
        
    // Show timing analysis
    if (dataPoints > 0) {
        Serial.printf("[TIMING] Timing analysis - Latest: %llu, Oldest: %llu, Span: %llu us\n", 
                     latestTimestamp, oldestTimestamp, latestTimestamp - oldestTimestamp);
    } else {
        Serial.println("[TIMING] No data found in time window!");
    }
    Serial.printf("[TIMING] Max values - Setpoint: %.3fA, Measured: %.3fA\n", maxSetpoint, maxMeasured);
    Serial.printf("[TIMING] Data points: %d\n", dataPoints);
    Serial.println("[TIMING] IQ History complete");
}

void TimingSystemTest::showIQHistory(uint32_t durationMs) {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
        
    Serial.print("[TIMING] IQ History (last ");
    Serial.print(durationMs);
    Serial.println("ms):");
        
    uint64_t currentTime = hwTimer.micros();
    uint64_t startTime = currentTime - (durationMs * 1000);
        
    Serial.printf("[TIMING] Current time: %llu, Start time: %llu\n", currentTime, startTime);
        
    // Get historical data from ring buffer
    float maxSetpoint = 0.0f;
    float maxMeasured = 0.0f;
    int dataPoints = 0;
    uint64_t latestTimestamp = 0;
    uint64_t oldestTimestamp = UINT64_MAX;
        
    // Iterate through recent IQ history
    for (size_t i = 0; i < 50; i++) {  // Check more entries
        const TimestampedIq* iqData = bds.getHistoryIq(i);
        if (!iqData) {
            Serial.printf("[TIMING] No IQ data at index %zu\n", i);
            continue;
        }
        
        if (iqData->timestamp < startTime) {
            Serial.printf("[TIMING] IQ data too old: %llu < %llu\n", iqData->timestamp, startTime);
            continue;
        }
            
        uint32_t ageMs = (currentTime - iqData->timestamp) / 1000;
        Serial.printf("  T+%ums: set=%.3fA, meas=%.3fA (ts=%llu)\n", 
                     ageMs, iqData->iqSetpoint, iqData->iqMeasured, iqData->timestamp);
            
        // Track timing
        if (iqData->timestamp > latestTimestamp) latestTimestamp = iqData->timestamp;
        if (iqData->timestamp < oldestTimestamp) oldestTimestamp = iqData->timestamp;
            
        // Track max values
        if (fabs(iqData->iqSetpoint) > fabs(maxSetpoint)) {
            maxSetpoint = iqData->iqSetpoint;
        }
        if (fabs(iqData->iqMeasured) > fabs(maxMeasured)) {
            maxMeasured = iqData->iqMeasured;
        }
        dataPoints++;
    }
        
    // Show timing analysis
    Serial.printf("[TIMING] Timing analysis - Latest: %llu, Oldest: %llu, Span: %llu us\n", 
                 latestTimestamp, oldestTimestamp, latestTimestamp - oldestTimestamp);
    Serial.printf("[TIMING] Max values - Setpoint: %.3fA, Measured: %.3fA\n", maxSetpoint, maxMeasured);
    Serial.printf("[TIMING] Data points: %d\n", dataPoints);
    Serial.println("[TIMING] IQ History complete");
}

void TimingSystemTest::showStatus() {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    
    Serial.println("[TIMING] System Status:");
    Serial.print("  Test enabled: ");
    Serial.println(enabled_ ? "YES" : "NO");
    Serial.print("  Baseline captured: ");
    Serial.println(baselineCaptured_ ? "YES" : "NO");
    
    if (baselineCaptured_) {
        Serial.print("  Baseline position: ");
        Serial.print(baselinePos_, 6);
        Serial.println(" turns");
        Serial.print("  Baseline IQ: ");
        Serial.print(baselineIq_, 3);
        Serial.println(" A");
    }
    
    // Show data freshness
    uint64_t encoderTime = bds.getEncoderTimestamp();
    uint64_t iqTime = bds.getIqTimestamp();
    uint64_t currentTime = hwTimer.micros();
    
    bool dataFresh = (encoderTime > 0 && (currentTime - encoderTime) < 50000) && 
                     (iqTime > 0 && (currentTime - iqTime) < 50000);
    Serial.print("  Data fresh: ");
    Serial.println(dataFresh ? "YES" : "NO");
    
    Serial.print("  Encoder age: ");
    Serial.print(encoderTime > 0 ? (currentTime - encoderTime) : 0);
    Serial.println(" us");
    
    Serial.print("  IQ age: ");
    Serial.print(iqTime > 0 ? (currentTime - iqTime) : 0);
    Serial.println(" us");
}
