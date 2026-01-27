#ifndef TIMING_SYSTEM_TEST_H
#define TIMING_SYSTEM_TEST_H

#include <Arduino.h>
#include "CanBusHandlerV2.h"
#include "ODriveCANProtocol.h"

/**
 * TimingSystemTest - Test harness for new cyclic-based timing system
 * 
 * Tests the BroadcastDataStore timing accessors and movement verification
 * before full integration into CanBusHandler.
 * 
 * Usage:
 *   TimingSystemTest timingTest;
 *   timingTest.begin();
 *   timingTest.handleCommand("timing_start");
 *   timingTest.handleCommand("timing_baseline");
 *   // Move motor...
 *   timingTest.handleCommand("timing_check");
 */
class TimingSystemTest {
public:
    TimingSystemTest();
    
    /**
     * Initialize timing system test
     */
    void begin();
    
    /**
     * Initialize timing system test with CAN bus handler
     * @param canHandler CAN bus handler reference
     */
    void begin(CanBusHandlerV2& canHandler);
    
    /**
     * Handle test commands
     * @param command Command string (timing_start, timing_baseline, timing_check, timing_window, timing_status)
     */
    void handleCommand(const String& command);
    
    /**
     * Check if active test is complete (called from main loop)
     */
    void checkTestCompletion();
    
private:
    bool enabled_;
    uint64_t testStartTime_;
    
    // Baseline data for movement testing
    bool baselineCaptured_;
    float baselinePos_;
    float baselineIq_;
    
    // CAN bus handler for sending commands
    CanBusHandlerV2* can_;
    
    // Test phase state machine
    enum TestPhase {
        TEST_PHASE_PRE_ROLL,
        TEST_PHASE_POST_COMMAND,
        TEST_PHASE_COMPLETE
    };
    TestPhase testPhase_;
    uint64_t captureStartTime_;
    uint64_t commandSendTime_;
    uint64_t phaseStartTime_;
    uint64_t collectionEndTime_;
    bool testActive_;
    
    /**
     * Start timing system test
     */
    void startTest();
    
    /**
     * Capture baseline position and IQ current
     */
    void captureBaseline();
    
    /**
     * Check for movement since baseline
     */
    void checkMovement();
    
    /**
     * Test command window timing
     */
    void testCommandWindow();
    
    // Auto-calibration functionality
    void runAutoCalibration();
    void showIQHistory(uint32_t durationMs);
    void showIQHistoryFromTime(uint64_t startTime, uint32_t durationMs);
    void analyzeCommandCenteredData();
    void showStatus();
    
    /**
     * Print debug data for troubleshooting
     */
    void printDebugData();
};

#endif // TIMING_SYSTEM_TEST_H
