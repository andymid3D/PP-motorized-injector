#ifndef RESPONSE_CORRELATION_TEST_H
#define RESPONSE_CORRELATION_TEST_H

#include <Arduino.h>

// Test enable flag - set to true to enable testing
#define RESPONSE_CORRELATION_TEST_ENABLED false

namespace ResponseCorrelationTest {
    struct TestResults {
        String testName;
        bool success;
        uint32_t startLatencyMs;
        String reason;
        unsigned long timestamp;
    };
    
    static const int MAX_TESTS = 8;
    
    // Main interface
    void init();
    void loop();
    void startTest();
    void resetTest();
    
    // Callback handlers (called by TimingSystemTest when START response detected)
    void onCommandStarted(uint8_t moduleId, uint32_t latency, String cmdName);
    void onCommandFailed(uint8_t moduleId, String cmdName, String reason);
};

#endif
