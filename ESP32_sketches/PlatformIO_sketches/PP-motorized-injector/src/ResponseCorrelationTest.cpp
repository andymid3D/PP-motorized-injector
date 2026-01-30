#include "ResponseCorrelationTest.h"
#include "config.h"
#include "MotorWrapper.h"
#include "TimingSystemTest.h"
#include "CanBusHandlerV2.h"
#include "MessageBuffer.h"

#if RESPONSE_CORRELATION_TEST_ENABLED

namespace ResponseCorrelationTest {
    // ===== TEST STATE =====
    static bool testActive = false;
    static int currentTestStep = 0;
    static unsigned long testStartTime = 0;
    static unsigned long lastDisplayTime = 0;
    static TestResults results[MAX_TESTS];
    
    // ===== TEST CONFIGURATION =====
    struct TestCase {
        String name;
        uint8_t moduleId;
        int ctrlMode;
        int inputMode;
        float value;
        MotorWrapper::RetryPriority priority;
        uint32_t expectedStartLatencyMs;
        bool shouldSucceed;
        String description;
    };
    
    static TestCase testCases[] = {
        {"Refill_Start_Detect", MODULE_REFILL, 3, 5, OFFSET_REFILL_GAP, 
         MotorWrapper::PRIORITY_NORMAL, 100, true, "Position move start detection"},
         
        {"Compression_Start_Torque", MODULE_COMPRESSION, 1, 6, COMPRESS_TRAVEL_TORQUE, 
         MotorWrapper::PRIORITY_HIGH, 75, true, "Torque move start detection"},
         
        {"Inject_Start_Position", MODULE_INJECT, 3, 5, 50.0f, 
         MotorWrapper::PRIORITY_CRITICAL, 50, true, "Critical injection start detection"},
         
        {"AntiDrip_Start_Slow", MODULE_ANTIDRIP, 2, 1, ANTIDRIP_VEL, 
         MotorWrapper::PRIORITY_LOW, 200, true, "Slow vel_ramp start detection"},
         
        {"PurgeZero_Start_Down", MODULE_PURGE_ZERO, 2, 1, PURGE_VEL_DOWN, 
         MotorWrapper::PRIORITY_LOW, 200, true, "Purge down start detection"},
         
        {"Release_Start_Position", MODULE_RELEASE, 3, 5, -5.0f, 
         MotorWrapper::PRIORITY_NORMAL, 100, true, "Release position start detection"},
         
        {"ReadyToInject_Start_Idle", MODULE_READY_TO_INJECT, 1, 6, 0, 
         MotorWrapper::PRIORITY_NORMAL, 75, true, "Ready to inject idle start"},
         
        {"Invalid_Position_Test", MODULE_REFILL, 3, 5, 999.0f, 
         MotorWrapper::PRIORITY_NORMAL, 100, false, "Invalid position should fail gracefully"}
    };
    
    // ===== INITIALIZATION =====
    void init() {
        Serial.println("=== RESPONSE CORRELATION TEST INIT ===");
        Serial.println("Testing Module ID tracking + START response correlation");
        Serial.println("Cyclic rates: Errors=50ms, IQ=10ms, Heartbeat=100ms");
        Serial.println("Test focus: START detection only (not completion)");
        Serial.printf("Test cases: %d\n", sizeof(testCases)/sizeof(TestCase));
        Serial.println("Commands: 'test_response' to start, 'test_reset' to reset");
        Serial.println("=========================================");
        
        resetTest();
    }
    
    // ===== MAIN TEST LOOP =====
    void loop() {
        if (!testActive) return;
        
        runCurrentTestStep();
        checkTestTimeout();
        displayTestProgress();
        
        // Check for manual commands
        handleSerialCommands();
    }
    
    // ===== TEST EXECUTION =====
    void startTest() {
        Serial.println("\n🚀 STARTING RESPONSE CORRELATION TEST");
        Serial.println("Testing START detection for different move types");
        Serial.println("==========================================");
        
        testActive = true;
        currentTestStep = 0;
        testStartTime = millis();
        lastDisplayTime = millis();
        
        // Clear any pending commands in TimingSystemTest
        TimingSystemTest::getInstance().clearPendingCommands();
        
        if (sizeof(testCases)/sizeof(TestCase) > 0) {
            Serial.printf("Test 1/%d: %s\n", sizeof(testCases)/sizeof(TestCase), testCases[0].name.c_str());
            Serial.printf("Description: %s\n", testCases[0].description.c_str());
            executeTestCase(testCases[0]);
        }
    }
    
    void executeTestCase(TestCase& test) {
        Serial.printf("\n📋 Executing: %s\n", test.name.c_str());
        Serial.printf("   Module: M%d, Mode: %d/%d, Value: %.2f\n", 
                     test.moduleId, test.ctrlMode, test.inputMode, test.value);
        Serial.printf("   Priority: %d, Expected start latency: %dms\n", 
                     test.priority, test.expectedStartLatencyMs);
        Serial.printf("   Description: %s\n", test.description.c_str());
        
        // Register command with TimingSystemTest for START detection tracking
        TimingSystemTest::getInstance().registerCommand(
            hwTimer.micros(), test.moduleId, test.ctrlMode, test.name
        );
        
        // Execute command via MotorWrapper with retry logic
        bool commandQueued = MotorWrapper::setModeAndMoveWithRetry(
            motor, test.ctrlMode, test.inputMode, test.value,
            test.moduleId, test.name, test.priority
        );
        
        if (!commandQueued) {
            Serial.printf("❌ FAILED: Command queue full for %s\n", test.name.c_str());
            recordTestResult(test, false, 0, "Command queue failed");
            nextTestStep();
        } else {
            Serial.printf("✅ Command queued successfully, waiting for START detection...\n");
        }
    }
    
    // ===== RESULT HANDLING =====
    void onCommandStarted(uint8_t moduleId, uint32_t latency, String cmdName) {
        if (!testActive) return;
        
        TestCase& currentTest = testCases[currentTestStep];
        
        // Check if this response matches our current test
        if (moduleId == currentTest.moduleId && cmdName == currentTest.name) {
            Serial.printf("\n🎯 START DETECTED: %s\n", cmdName.c_str());
            Serial.printf("   Module: M%d, Start latency: %dms\n", moduleId, latency);
            
            // Check if latency is within expected range (allow 2x tolerance)
            bool latencyOk = (latency <= currentTest.expectedStartLatencyMs * 2);
            String reason = latencyOk ? "Start detected successfully" : "Start latency out of expected range";
            
            if (latencyOk) {
                Serial.printf("✅ SUCCESS: %s (latency: %dms, expected: %dms)\n", 
                             cmdName.c_str(), latency, currentTest.expectedStartLatencyMs);
            } else {
                Serial.printf("⚠️  WARNING: %s (latency: %dms, expected: %dms)\n", 
                             cmdName.c_str(), latency, currentTest.expectedStartLatencyMs);
            }
            
            recordTestResult(currentTest, latencyOk, latency, reason);
            nextTestStep();
        }
    }
    
    void onCommandFailed(uint8_t moduleId, String cmdName, String reason) {
        if (!testActive) return;
        
        TestCase& currentTest = testCases[currentTestStep];
        
        if (moduleId == currentTest.moduleId && cmdName == currentTest.name) {
            Serial.printf("\n❌ FAILED: %s\n", cmdName.c_str());
            Serial.printf("   Module: M%d, Reason: %s\n", moduleId, reason.c_str());
            
            bool expectedFailure = !currentTest.shouldSucceed;
            String resultReason = expectedFailure ? "Expected failure" : "Unexpected failure";
            
            recordTestResult(currentTest, expectedFailure, 0, resultReason);
            nextTestStep();
        }
    }
    
    // ===== TEST MANAGEMENT =====
    void nextTestStep() {
        currentTestStep++;
        
        if (currentTestStep >= sizeof(testCases)/sizeof(TestCase)) {
            completeTest();
        } else {
            // Brief pause between tests
            delay(2000);
            
            Serial.printf("\nTest %d/%d: %s\n", 
                         currentTestStep + 1, sizeof(testCases)/sizeof(TestCase), 
                         testCases[currentTestStep].name.c_str());
            Serial.printf("Description: %s\n", testCases[currentTestStep].description.c_str());
            executeTestCase(testCases[currentTestStep]);
        }
    }
    
    void completeTest() {
        testActive = false;
        
        Serial.println("\n🎯 RESPONSE CORRELATION TEST COMPLETE");
        Serial.println("=====================================");
        
        // Display results summary
        int passed = 0, failed = 0;
        uint32_t totalLatency = 0;
        uint32_t minLatency = UINT32_MAX;
        uint32_t maxLatency = 0;
        
        Serial.println("\n📊 DETAILED RESULTS:");
        for (int i = 0; i < currentTestStep; i++) {
            TestResults& result = results[i];
            
            Serial.printf("Test %2d: %-20s - %s", i + 1, result.testName.c_str(),
                         result.success ? "PASS" : "FAIL");
            
            if (result.success && result.startLatencyMs > 0) {
                Serial.printf(" (latency: %3dms)", result.startLatencyMs);
                totalLatency += result.startLatencyMs;
                if (result.startLatencyMs < minLatency) minLatency = result.startLatencyMs;
                if (result.startLatencyMs > maxLatency) maxLatency = result.startLatencyMs;
            } else {
                Serial.printf(" (latency:   N/A)");
            }
            
            Serial.printf(" [%s]\n", result.reason.c_str());
            
            if (result.success) {
                passed++;
            } else {
                failed++;
            }
        }
        
        Serial.println("\n📈 SUMMARY STATISTICS:");
        Serial.println("=====================================");
        Serial.printf("Results: %d passed, %d failed\n", passed, failed);
        Serial.printf("Success rate: %.1f%%\n", (float)passed / (passed + failed) * 100.0f);
        
        if (passed > 0) {
            Serial.printf("Latency stats: Avg=%dms, Min=%dms, Max=%dms\n", 
                         totalLatency / passed, minLatency, maxLatency);
        }
        
        Serial.printf("Test duration: %lums\n", millis() - testStartTime);
        Serial.println("=====================================");
        
        resetTest();
    }
    
    // ===== UTILITY FUNCTIONS =====
    void resetTest() {
        testActive = false;
        currentTestStep = 0;
        testStartTime = 0;
        lastDisplayTime = 0;
        memset(results, 0, sizeof(results));
    }
    
    void recordTestResult(TestCase& test, bool success, uint32_t latency, String reason) {
        TestResults& result = results[currentTestStep];
        result.testName = test.name;
        result.success = success;
        result.startLatencyMs = latency;
        result.reason = reason;
        result.timestamp = millis();
    }
    
    void checkTestTimeout() {
        if (!testActive) return;
        
        TestCase& currentTest = testCases[currentTestStep];
        uint32_t timeout = MotorWrapper::RETRY_TIMEOUTS[currentTest.priority] * MotorWrapper::MAX_RETRIES * 2; // 2x safety
        
        if ((millis() - testStartTime) > timeout) {
            Serial.printf("\n⏰ TIMEOUT: %s (timeout: %lums)\n", 
                         currentTest.name.c_str(), timeout);
            recordTestResult(currentTest, false, 0, "Test timeout - no START response detected");
            nextTestStep();
        }
    }
    
    void displayTestProgress() {
        if (!testActive) return;
        
        // Display progress every 10 seconds
        if (millis() - lastDisplayTime > 10000) {
            TestCase& currentTest = testCases[currentTestStep];
            Serial.printf("⏳ Test in progress: %s (elapsed: %lums, waiting for START detection)\n",
                         currentTest.name.c_str(), millis() - testStartTime);
            lastDisplayTime = millis();
        }
    }
    
    void handleSerialCommands() {
        // Handle manual commands during test
        if (Serial.available()) {
            String cmd = Serial.readString();
            cmd.trim();
            
            if (cmd == "test_reset") {
                Serial.println("🔄 Test reset manually");
                resetTest();
            } else if (cmd == "test_status") {
                if (testActive) {
                    TestCase& currentTest = testCases[currentTestStep];
                    Serial.printf("📋 Current test: %s (elapsed: %lums)\n", 
                                 currentTest.name.c_str(), millis() - testStartTime);
                } else {
                    Serial.println("ℹ️  No test currently active");
                }
            }
        }
    }
};

#endif // RESPONSE_CORRELATION_TEST_ENABLED
