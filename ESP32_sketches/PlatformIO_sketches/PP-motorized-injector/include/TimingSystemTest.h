#ifndef TIMING_SYSTEM_TEST_H
#define TIMING_SYSTEM_TEST_H

#include <Arduino.h>

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
     * Handle test commands
     * @param command Command string (timing_start, timing_baseline, timing_check, timing_window, timing_status)
     */
    void handleCommand(const String& command);
    
private:
    bool enabled_;
    uint64_t testStartTime_;
    
    // Baseline data for movement testing
    bool baselineCaptured_;
    float baselinePos_;
    float baselineIq_;
    float baselineBusCurrent_;
    
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
    
    /**
     * Print debug data for troubleshooting
     */
    void printDebugData();
    
    /**
     * Show system status
     */
    void showStatus();
};

#endif // TIMING_SYSTEM_TEST_H
