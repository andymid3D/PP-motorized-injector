/**
 * @file CanRxHandlerTest.h
 * @brief Non-disruptive Core 0 CanRxHandler diagnostic module
 * 
 * @details
 * This module monitors the health and activity of the CanRxHandler Core 0 task
 * without interfering with production FSM operation. It provides detailed visibility
 * into CAN message reception, Core 0 task status, and ESP32Can.readFrame() performance.
 * 
 * Features:
 * - Core 0 task startup verification
 * - ESP32Can.readFrame() polling statistics  
 * - CAN message reception monitoring
 * - Task health and performance metrics
 * - Non-blocking serial output
 * 
 * @date January 29, 2026
 */

#ifndef CANRXHANDLERTEST_H
#define CANRXHANDLERTEST_H

#include <Arduino.h>

class CanRxHandlerTest {
public:
    /**
     * @brief Initialize the diagnostic module
     * 
     * Sets up static variables and prepares for monitoring.
     * Call once in setup() after CanRxHandler initialization.
     */
    static void begin();
    
    /**
     * @brief Main diagnostic loop
     * 
     * Called every loop() iteration to monitor Core 0 task health.
     * Collects statistics and prints reports every 5 seconds.
     * Non-disruptive to production FSM.
     */
    static void loop();
    
    /**
     * @brief Force immediate status report
     * 
     * Prints current diagnostic status immediately.
     * Useful for manual debugging via serial commands.
     */
    static void printStatus();
    
    /**
     * @brief Enable/disable verbose debugging
     * 
     * @param enabled True for verbose output, false for summary only
     */
    static void setVerbose(bool enabled);
    
private:
    // ===== DIAGNOSTIC STATE =====
    static bool initialized_;
    static bool verbose_;
    static uint32_t lastReportTime_;
    static uint32_t startTime_;
    
    // ===== CORE 0 TASK MONITORING =====
    static uint32_t lastTaskCheckTime_;
    static bool core0TaskActive_;
    static uint32_t core0TaskChecks_;
    
    // ===== CAN RECEPTION STATISTICS =====
    static uint32_t totalPolls_;
    static uint32_t successfulReads_;
    static uint32_t failedReads_;
    static uint32_t messagesReceived_;
    static uint32_t lastMessageTime_;
    
    // ===== PER-REPORT STATISTICS =====
    static uint32_t reportPolls_;
    static uint32_t reportSuccess_;
    static uint32_t reportFailed_;
    static uint32_t lastReportPolls_;
    
    // ===== PERFORMANCE METRICS =====
    static uint32_t maxPollInterval_;
    static uint32_t lastPollTime_;
    static uint32_t pollIntervalSum_;
    static uint32_t pollIntervalCount_;
    
    // ===== INTERNAL METHODS =====
    static void checkCore0TaskHealth();
    static void updateStatistics();
    static void printReport();
    static void printVerboseInfo();
};

#endif // CANRXHANDLERTEST_H
