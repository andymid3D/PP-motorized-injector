/**
 * @file PhaseTests.cpp
 * @brief Implementation of Phase 1 test harness
 * 
 * @details
 * Centralizes all Phase 1 module validation tests.
 * Each test checks its enable flag before running.
 * 
 * @author PP-motorized-injector
 * @date January 16, 2026
 */

#include "PhaseTests.h"
#include "RingBuffer.h"
#include "CanRxHandler.h"
#include "GPTimer.h"
#include "BroadcastDataStore.h"  // Phase 1.7: BDS v2 testing
#include "OurLoopTimer.h" // Use our custom dual-core timer
#include <BufferedOutput.h>  // Phase 2: Non-blocking serial output

#if TEST_BDS_INTEGRATION_ENABLED
#include "DebugCommands.h"
extern DebugCommands debugCmds;
#endif

// External globals (declared in main.cpp)
extern GPTimer hwTimer;

// =============================================================================
// PUBLIC INTERFACE
// =============================================================================

void PhaseTests::runSetupTests() {
#if TEST_RINGBUFFER_ENABLED
    testRingBuffer();
#endif

#if TEST_BDS_STORAGE_ENABLED
    testBDSStorage();
#endif

    // Future setup tests here
    // #if TEST_SOME_OTHER_MODULE
    //   testSomeOtherModule();
    // #endif
}

void PhaseTests::runLoopTests() { // Removed BufferedOutput& serialOut parameter
    // ===== Performance Monitoring (Always Active) =====
#if TEST_LOOPTIMER_ENABLED
    static unsigned long lastLoopStatsPrintTime = 0;
    if (millis() - lastLoopStatsPrintTime >= 5000) { // Print stats every 5 seconds
        printLoopStats();
        lastLoopStatsPrintTime = millis();
    }
#endif

    // ===== Active Loop Tests =====
#if TEST_STRESS_QUEUE_ENABLED
    testStressQueue();
#elif TEST_BASELINE_TIMING_ENABLED
    // testBaselineTiming(serialOut); // This test will need to be updated to use Serial.print directly or OurLoopTimer
#elif TEST_BDS_STORAGE_ENABLED
    // testBDSStorage();  // Setup test, not loop test
#elif TEST_BDS_INTEGRATION_ENABLED
    testBDSIntegration();
#elif TEST_PROTECTED_WINDOW_ENABLED
    // Protected window test runs independently (not via PhaseTests)
    // See main.cpp for ProtectedWindowTest initialization
#else
    // No active loop tests - baseline performance measurement
    // Queue drain happens naturally, loopTimer measures actual performance
#endif
}

// =============================================================================
// PHASE 1.7: RingBuffer Validation
// =============================================================================

#if TEST_RINGBUFFER_ENABLED

void PhaseTests::testRingBuffer() {
    Serial.println("=== RingBuffer Unit Test ===");
    
    // Test structure
    struct TestData {
        float value;
        uint64_t timestamp;
    };
    
    RingBuffer<TestData, 5> buffer;  // Small buffer for testing
    
    // Test 1: Empty buffer
    Serial.print("Test 1 - Empty: ");
    if (buffer.isEmpty() && buffer.getCount() == 0 && buffer.getLatest() == nullptr) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 2: Push elements (not full)
    Serial.print("Test 2 - Push 3 elements: ");
    for (int i = 1; i <= 3; i++) {
        buffer.push({(float)i * 10.0f, (uint64_t)i * 1000});
    }
    if (buffer.getCount() == 3 && !buffer.isFull()) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 3: Get latest
    Serial.print("Test 3 - Latest value: ");
    const TestData* latest = buffer.getLatest();
    if (latest && latest->value == 30.0f && latest->timestamp == 3000) {
        Serial.println("PASS (30.0 @ 3000us)");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 4: Get history
    Serial.print("Test 4 - History access: ");
    const TestData* prev = buffer.getHistory(1);  // Previous value
    const TestData* oldest = buffer.getHistory(2);  // Oldest value
    if (prev && prev->value == 20.0f && oldest && oldest->value == 10.0f) {
        Serial.println("PASS (20.0, 10.0)");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 5: Fill buffer (reaches capacity)
    Serial.print("Test 5 - Fill to capacity: ");
    buffer.push({40.0f, 4000});
    buffer.push({50.0f, 5000});
    if (buffer.isFull() && buffer.getCount() == 5) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 6: Wraparound (oldest evicted)
    Serial.print("Test 6 - Wraparound: ");
    buffer.push({60.0f, 6000});  // Should evict 10.0
    buffer.push({70.0f, 7000});  // Should evict 20.0
    latest = buffer.getLatest();
    oldest = buffer.getHistory(4);  // Oldest still in buffer
    if (latest && latest->value == 70.0f && oldest && oldest->value == 30.0f) {
        Serial.println("PASS (latest=70.0, oldest=30.0)");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 7: forEach iteration
    Serial.print("Test 7 - forEach: [");
    int count = 0;
    buffer.forEach([&count](const TestData& item, size_t idx) {
        if (count > 0) Serial.print(", ");
        Serial.print(item.value, 1);
        count++;
        return true;
    });
    Serial.println("] (newest to oldest)");
    
    // Test 8: Clear and reuse
    Serial.print("Test 8 - Clear: ");
    buffer.clear();
    if (buffer.isEmpty() && buffer.getCount() == 0) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    Serial.println("=== RingBuffer Test Complete ===\n");
}

#endif // TEST_RINGBUFFER_ENABLED

// =============================================================================
// PHASE 1.6b: Queue Stress Testing
// =============================================================================
// NOTE: This test includes FULL FSM LOAD SIMULATION (700ms delay)
// For BASELINE testing (no delays), use separate module-specific tests
// Strategy: New modules start baseline → validate → add stress

#if TEST_STRESS_QUEUE_ENABLED

void PhaseTests::testStressQueue() {
    // ===== FSM Load Simulation =====
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

    // user added delay for threshold discovery
    delayMicroseconds(700000);  // 700ms - critical threshold test

    
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
}

#endif // TEST_STRESS_QUEUE_ENABLED

// =============================================================================
// PHASE 1.7: BDS v2 Storage Validation
// =============================================================================
// Tests ring buffer storage, timestamp preservation, staleness detection

#if TEST_BDS_STORAGE_ENABLED

void PhaseTests::testBDSStorage() {
    Serial.println("\n=== BDS v2 Storage Test ===");
    
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    uint64_t baseTime = hwTimer.micros();
    
    // Test 1: Store encoder data
    Serial.print("Test 1 - Store encoder: ");
    bds.storeEncoder(10.5f, 2.3f, baseTime, false);
    const TimestampedEncoder* enc1 = bds.getLatestEncoder();
    if (enc1 != nullptr && enc1->position == 10.5f && enc1->velocity == 2.3f && enc1->timestamp == baseTime) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 2: Store multiple encoder samples
    Serial.print("Test 2 - Multiple samples: ");
    bds.storeEncoder(20.5f, 3.3f, baseTime + 10000, false);
    bds.storeEncoder(30.5f, 4.3f, baseTime + 20000, false);
    const TimestampedEncoder* enc2 = bds.getLatestEncoder();
    if (enc2 != nullptr && enc2->position == 30.5f && enc2->timestamp == baseTime + 20000) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 3: Store heartbeat
    Serial.print("Test 3 - Store heartbeat: ");
    bds.storeHeartbeat(0x00000000, 8, 0, 0, 0, 0, baseTime + 30000, false);  // All flags clear, trajectory not done
    const TimestampedHeartbeat* hb = bds.getLatestHeartbeat();
    if (hb != nullptr && hb->axisState == 8 && hb->axisError == 0) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 4: Store Iq data
    Serial.print("Test 4 - Store Iq: ");
    bds.storeIq(15.0f, 14.8f, baseTime + 40000, false);
    const TimestampedIq* iq = bds.getLatestIq();
    if (iq != nullptr && iq->iqSetpoint == 15.0f && iq->iqMeasured == 14.8f) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 5: Store BusVI data
    Serial.print("Test 5 - Store BusVI: ");
    bds.storeBusVI(24.1f, 5.2f, baseTime + 50000, false);
    const TimestampedBusVI* busvi = bds.getLatestBusVI();
    if (busvi != nullptr && busvi->busVoltage == 24.1f && busvi->busCurrent == 5.2f) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 6: Staleness detection (should NOT be stale - just stored with current time)
    Serial.print("Test 6 - Fresh data not stale: ");
    uint64_t freshTime = hwTimer.micros();
    bds.storeEncoder(50.5f, 6.3f, freshTime, false);  // Store with CURRENT time
    bool stale = bds.isEncoderStale(500000);  // 500ms threshold
    if (!stale) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 7: Age calculation (should be very small - just stored)
    Serial.print("Test 7 - Age calculation: ");
    uint64_t currentTime = hwTimer.micros();
    uint64_t age = bds.getEncoderAgeMicros(currentTime);
    if (age < 100000) {  // Should be < 100ms old (just stored)
        Serial.print("PASS (age=");
        Serial.print((uint32_t)age);
        Serial.println("us)");
    } else {
        Serial.print("FAIL (age=");
        Serial.print((uint32_t)age);
        Serial.println("us)");
    }
    
    // Test 8: TX correlation flag
    Serial.print("Test 8 - TX correlation flag: ");
    bds.storeEncoder(40.5f, 5.3f, baseTime + 60000, true);  // Mark as response
    const TimestampedEncoder* enc3 = bds.getLatestEncoder();
    if (enc3 != nullptr && enc3->isResponse == true) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 9: Backward compatibility (v1 getters still work)
    Serial.print("Test 9 - V1 compatibility: ");
    float pos = bds.getPosition();
    float vel = bds.getVelocity();
    if (pos == 40.5f && vel == 5.3f) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 10: Ring buffer wraparound (fill > history size)
    Serial.print("Test 10 - Ring wraparound: ");
    for (int i = 0; i < 15; i++) {
        bds.storeEncoder(100.0f + i, 1.0f, baseTime + 70000 + (i * 1000), false);
    }
    const TimestampedEncoder* enc4 = bds.getLatestEncoder();
    if (enc4 != nullptr && enc4->position == 114.0f) {  // Should be last value (100 + 14)
        Serial.println("PASS");
    } else {
        Serial.println("FAIL");
    }
    
    Serial.println("=== BDS v2 Storage Test Complete ===");
    Serial.println();
}

#endif // TEST_BDS_STORAGE_ENABLED

// =============================================================================
// PHASE 1.7: BDS v2 Integration with Live ODrive (Continuous Loop Test)
// =============================================================================
// Tests drainAndStore() with real CAN messages from ODrive
// Validates production code path (CanRxHandler -> BDS v2)

#if TEST_BDS_INTEGRATION_ENABLED

void PhaseTests::testBDSIntegration() {
    // Static variables for continuous testing
    static uint32_t lastStatsTime = 0;
    static uint32_t msgCount = 0;
    static uint32_t drainCalls = 0;
    static uint32_t maxMsgsPerDrain = 0;
    static uint32_t burstCount = 0;  // Track >5 msg bursts
    static bool firstRun = true;
    
    if (firstRun) {
        Serial.println("\n=== BDS v2 Integration Test (Live ODrive) ===");
        Serial.println("Calling drainAndStore() every loop...");
        Serial.println("Monitoring: Message rate, BDS updates, loop time");
        Serial.println("\n=== Debug Commands Available ===");
        Serial.println("Type 'help' for command list");
        Serial.println("Example: axis_state 8 (enter closed loop)");
        Serial.println("         controller_modes 2 1 (velocity passthrough)");
        Serial.println("         set_velocity 2.0\n");
        firstRun = false;
        lastStatsTime = millis();
    }
    
    // Handle serial debug commands (non-blocking)
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "help") {
            Serial.println("\n=== Quick Command Reference ===");
            Serial.println("CALIBRATION:");
            Serial.println("  axis_state 7              // Full calibration (motor will move!)");
            Serial.println("SETUP:");
            Serial.println("  axis_state 8              // Enter closed loop");
            Serial.println("  controller_modes 3 1      // Position passthrough");
            Serial.println("  controller_modes 3 5      // Position trap_traj");
            Serial.println("MOVE:");
            Serial.println("  set_position 10.5         // Go to 10.5 turns");
            Serial.println("  set_position 0.0          // Return to zero");
            Serial.println("  set_torque 0.5            // Apply 0.5 Nm (torque mode)");
            Serial.println("LIMITS:");
            Serial.println("  set_limits 25.0 15.0      // 25 rps, 15A");
            Serial.println("CONTROL:");
            Serial.println("  axis_state 1              // Idle (stop)");
            Serial.println("  clear_errors              // Clear axis errors");
            Serial.println("  set_linear_count 0        // Zero encoder\n");
        } else if (cmd.length() > 0) {
            // Forward command to DebugCommands for parsing/execution
            Serial.print("[CMD] ");
            Serial.println(cmd);
            debugCmds.handleCommand(cmd);
        }
    }
    
    // Call production drain method (processes queue -> BDS)
    CanRxHandler& canRx = CanRxHandler::getInstance();
    uint64_t drainStart = hwTimer.micros();
    uint32_t processed = canRx.drainAndStore();
    uint64_t drainDuration = hwTimer.micros() - drainStart;
    
    // Service CAN TX queue (send queued commands with timing enforcement)
    extern CanBusHandlerV2 motor;
    motor.loop();
    
    if (processed > 0) {
        msgCount += processed;
        drainCalls++;
        if (processed > maxMsgsPerDrain) {
            maxMsgsPerDrain = processed;
            // Log significant bursts (>5 messages, likely cause of max loop spike)
            if (processed > 5) {
                burstCount++;
                Serial.print("[BURST] ");
                Serial.print(processed);
                Serial.print(" msgs drained in ");
                Serial.print((uint32_t)drainDuration);
                Serial.println(" us");
            }
        }
    }
    
    // Print stats every 5 seconds
    if (millis() - lastStatsTime >= 5000) {
        BroadcastDataStore& bds = BroadcastDataStore::getInstance();
        
        // Get latest data from BDS
        const TimestampedEncoder* enc = bds.getLatestEncoder();
        const TimestampedHeartbeat* hb = bds.getLatestHeartbeat();
        const TimestampedIq* iq = bds.getLatestIq();
        const TimestampedMotorError* motorErr = bds.getLatestMotorError();
        const TimestampedEncoderError* encErr = bds.getLatestEncoderError();
        const TimestampedControllerError* ctrlErr = bds.getLatestControllerError();
        
        Serial.print("[Integration] Messages: ");
        Serial.print(msgCount);
        Serial.print(" | Rate: ");
        Serial.print(msgCount / 5.0, 1);
        Serial.print(" msg/s | Drain calls: ");
        Serial.print(drainCalls);
        Serial.print(" | Max burst: ");
        Serial.print(maxMsgsPerDrain);
        Serial.print(" msgs");
        if (burstCount > 0) {
            Serial.print(" (>");
            Serial.print(burstCount);
            Serial.print(" bursts >5)");
        }
        Serial.print(" | Queue: ");
        Serial.print(canRx.getQueueDepth());
        Serial.print(" | Overflows: ");
        Serial.println(canRx.getQueueOverflows());
        
        // BDS diagnostic data (commented out for cleaner output - re-enable if needed)
        // if (enc != nullptr) {
        //     Serial.print("[BDS Encoder] Pos: ");
        //     Serial.print(enc->position, 2);
        //     Serial.print(" turns | Vel: ");
        //     Serial.print(enc->velocity, 2);
        //     Serial.print(" rps | Age: ");
        //     uint64_t age = hwTimer.micros() - enc->timestamp;
        //     Serial.print((uint32_t)age);
        //     Serial.println(" us");
        // } else {
        //     Serial.println("[BDS Encoder] No data yet");
        // }
        
        // if (hb != nullptr) {
        //     Serial.print("[BDS Heartbeat] State: ");
        //     Serial.print(hb->axisState);
        //     Serial.print(" | Axis Error: 0x");
        //     Serial.print(hb->axisError, HEX);
        //     Serial.print(" | Age: ");
        //     uint64_t age = hwTimer.micros() - hb->timestamp;
        //     Serial.print((uint32_t)age);
        //     Serial.println(" us");
        // } else {
        //     Serial.println("[BDS Heartbeat] No data yet");
        // }
        
        // Show all 4 error types (ODrive separates them)
        // Serial.print("[BDS Errors] Axis: 0x");
        // Serial.print(hb ? hb->axisError : 0, HEX);
        // Serial.print(" | Motor: 0x");
        // Serial.print(motorErr ? (unsigned long long)motorErr->motorError : 0, HEX);  // 64-bit for ODrive motor errors
        // Serial.print(" | Encoder: 0x");
        // Serial.print(encErr ? encErr->encoderError : 0, HEX);
        // Serial.print(" | Controller: 0x");
        // Serial.println(ctrlErr ? ctrlErr->controllerError : 0, HEX);
        
        // if (iq != nullptr) {
        //     Serial.print("[BDS Iq] Setpoint: ");
        //     Serial.print(iq->iqSetpoint, 2);
        //     Serial.print(" A | Measured: ");
        //     Serial.print(iq->iqMeasured, 2);
        //     Serial.println(" A");
        // } else {
        //     Serial.println("[BDS Iq] No data yet");
        // }
        
        // Check V1 backward compatibility
        // Serial.print("[V1 API] Pos: ");
        // Serial.print(bds.getPosition(), 2);
        // Serial.print(" | Vel: ");
        // Serial.print(bds.getVelocity(), 2);
        // Serial.print(" | State: ");
        // Serial.println(bds.getAxisState());
        
        Serial.println();
        
        msgCount = 0;
        drainCalls = 0;
        maxMsgsPerDrain = 0;
        burstCount = 0;
        lastStatsTime = millis();
    }
}

#endif  // TEST_BDS_INTEGRATION_ENABLED

// =============================================================================
// PHASE 2: BASELINE TIMING TEST (Before SafeString)
// =============================================================================

#if TEST_BASELINE_TIMING_ENABLED

/**
 * @brief Baseline timing test - measure performance WITH Serial.print() blocking
 * 
 * Purpose: Capture "before" metrics to compare against SafeString implementation
 * 
 * Measures:
 * - Loop time with Serial.print() calls
 * - Message capture rate
 * - Queue depth
 * 
 * Expected Results (with Serial blocking):
 * - Loop time: Variable (1-5ms spikes during Serial.print)
 * - Message rate: ~180 msg/s (ODrive broadcast rate)
 * - Queue depth: 0-2 (occasional accumulation during Serial output)
 * 
 * Run for 60 seconds to get stable averages
 */
void PhaseTests::testBaselineTiming(BufferedOutput& serialOut) { // serialOut parameter is now unused
    static bool testStarted = false;
    static unsigned long testStartTime = 0;
    static uint32_t msgCount = 0;
    static uint32_t drainCalls = 0;
    static unsigned long lastStatsTime = 0;
    static uint8_t maxQueueDepth = 0;
    
    const unsigned long TEST_DURATION_MS = 60000;  // 60 second test
    const unsigned long STATS_INTERVAL_MS = 5000;  // Print every 5 seconds
    
    // Initialize test on first call
    if (!testStarted) {
        Serial.println("\n===== BASELINE TIMING TEST (Pre-SafeString) ====="); // Use Serial.println directly
        Serial.println("Purpose: Measure loop performance WITH Serial.print() blocking");
        Serial.println("Duration: 60 seconds");
        Serial.println("Metrics: Loop time, message rate, queue depth");
        Serial.println("\nTest running...\n");
        
        testStartTime = millis();
        lastStatsTime = millis();
        testStarted = true;
    }
    
    // Check if test complete
    unsigned long elapsed = millis() - testStartTime;
    if (elapsed >= TEST_DURATION_MS) {
        Serial.println("\n===== BASELINE TEST COMPLETE =====");
        Serial.print("Total messages: "); Serial.println(msgCount);
        Serial.print("Average rate: "); Serial.print(msgCount / 60.0, 1); Serial.println(" msg/s");
        Serial.print("Max queue depth: "); Serial.println(maxQueueDepth);
        Serial.println("\nNOTE: Save these results for comparison after SafeString integration");
        Serial.println("Expected improvement: Loop time reduction of 1-5ms per Serial.print()");
        Serial.println("\n==============================================\n");
        
        while(1) { delay(1000); }  // Halt for review
    }
    
    // Drain CAN queue and count messages
    CanRxHandler& canRx = CanRxHandler::getInstance();
    CANRxMessage msg;
    
    uint8_t queueDepthBefore = canRx.getQueueDepth();
    if (queueDepthBefore > maxQueueDepth) maxQueueDepth = queueDepthBefore;
    
    uint32_t msgsThisDrain = 0;
    while (canRx.receiveMessage(msg, 0)) {
        msgCount++;
        msgsThisDrain++;
    }
    
    if (msgsThisDrain > 0) drainCalls++;
    
    // Print stats every 5 seconds (THIS IS THE BLOCKING POINT WE'RE MEASURING)
    if (millis() - lastStatsTime >= STATS_INTERVAL_MS) {
        Serial.print("["); Serial.print(elapsed / 1000); Serial.print("s] ");
        Serial.print("Messages: "); Serial.print(msgCount);
        Serial.print(" | Rate: "); Serial.print(msgCount / (float)(elapsed / 1000), 1); Serial.print(" msg/s");
        Serial.print(" | Max queue: "); Serial.print(maxQueueDepth);
        Serial.print(" | Overflows: "); Serial.println(canRx.getQueueOverflows());
        
        lastStatsTime = millis();
    }
}

#endif  // TEST_BASELINE_TIMING_ENABLED
