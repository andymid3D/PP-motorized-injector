// src/CanRxHandlerTest.cpp
#include "CanRxHandlerTest.h"
#include "CanRxHandler.h"
#include <ESP32-TWAI-CAN.hpp>
#include "GPTimer.h"
#include "config.h"

// ===== STATIC MEMBER INITIALIZATION =====
bool CanRxHandlerTest::initialized_ = false;
bool CanRxHandlerTest::verbose_ = false;
uint32_t CanRxHandlerTest::lastReportTime_ = 0;
uint32_t CanRxHandlerTest::startTime_ = 0;

uint32_t CanRxHandlerTest::lastTaskCheckTime_ = 0;
bool CanRxHandlerTest::core0TaskActive_ = false;
uint32_t CanRxHandlerTest::core0TaskChecks_ = 0;

uint32_t CanRxHandlerTest::totalPolls_ = 0;
uint32_t CanRxHandlerTest::successfulReads_ = 0;
uint32_t CanRxHandlerTest::failedReads_ = 0;
uint32_t CanRxHandlerTest::messagesReceived_ = 0;
uint32_t CanRxHandlerTest::lastMessageTime_ = 0;

uint32_t CanRxHandlerTest::reportPolls_ = 0;
uint32_t CanRxHandlerTest::reportSuccess_ = 0;
uint32_t CanRxHandlerTest::reportFailed_ = 0;
uint32_t CanRxHandlerTest::lastReportPolls_ = 0;

uint32_t CanRxHandlerTest::maxPollInterval_ = 0;
uint32_t CanRxHandlerTest::lastPollTime_ = 0;
uint32_t CanRxHandlerTest::pollIntervalSum_ = 0;
uint32_t CanRxHandlerTest::pollIntervalCount_ = 0;

extern GPTimer hwTimer;

// ===== PUBLIC API IMPLEMENTATION =====

void CanRxHandlerTest::begin() {
    if (initialized_) return;
    
    startTime_ = millis();
    lastReportTime_ = startTime_;
    lastTaskCheckTime_ = startTime_;
    lastPollTime_ = startTime_;
    
    initialized_ = true;
    
    DEBUG_PRINTLN("[CanRxTest] Initialized - Monitoring Core 0 CanRxHandler task");
}

void CanRxHandlerTest::loop() {
    if (!initialized_) return;
    
    uint32_t currentTime = millis();
    
    // Check Core 0 task health every 1 second
    if (currentTime - lastTaskCheckTime_ >= 1000) {
        checkCore0TaskHealth();
        lastTaskCheckTime_ = currentTime;
    }
    
    // Update statistics
    updateStatistics();
    
    // Print report every 5 seconds
    if (currentTime - lastReportTime_ >= 5000) {
        printReport();
        lastReportTime_ = currentTime;
    }
}

void CanRxHandlerTest::printStatus() {
    if (!initialized_) {
        DEBUG_PRINTLN("[CanRxTest] ERROR: Not initialized");
        return;
    }
    
    printReport();
}

void CanRxHandlerTest::setVerbose(bool enabled) {
    verbose_ = enabled;
    DEBUG_PRINT(enabled ? "[CanRxTest] Verbose mode enabled" : "[CanRxTest] Verbose mode disabled");
}

// ===== PRIVATE METHODS =====

void CanRxHandlerTest::checkCore0TaskHealth() {
    CanRxHandler& canRx = CanRxHandler::getInstance();
    
    // Check if Core 0 task is responsive by querying statistics
    uint32_t messagesReceived = canRx.getMessagesReceived();
    uint32_t queueDepth = canRx.getQueueDepth();
    uint32_t overflows = canRx.getQueueOverflows();
    
    // Core 0 task is considered active if it's processing messages
    // or if statistics are changing (indicating task is running)
    bool taskActive = (messagesReceived > 0) || (queueDepth > 0) || (overflows > 0);
    
    // Update task activity tracking
    if (taskActive != core0TaskActive_) {
        core0TaskActive_ = taskActive;
        DEBUG_PRINTF("[CanRxTest] Core 0 task status: %s\n", taskActive ? "ACTIVE" : "INACTIVE");
    }
    
    core0TaskChecks_++;
    
    if (verbose_) {
        DEBUG_PRINTF("[CanRxTest] Core0 check #%u | Messages: %u | Queue: %u | Overflows: %u\n",
                    core0TaskChecks_, messagesReceived, queueDepth, overflows);
    }
}

void CanRxHandlerTest::updateStatistics() {
    // Simulate ESP32Can.readFrame() polling to test CAN bus directly
    CanFrame testFrame;
    
    // Record poll timing
    uint32_t currentTime = millis();
    if (lastPollTime_ > 0) {
        uint32_t pollInterval = currentTime - lastPollTime_;
        pollIntervalSum_ += pollInterval;
        pollIntervalCount_++;
        
        if (pollInterval > maxPollInterval_) {
            maxPollInterval_ = pollInterval;
        }
    }
    lastPollTime_ = currentTime;
    
    // Test direct CAN read (non-blocking)
    totalPolls_++;
    reportPolls_++;
    
    bool readResult = ESP32Can.readFrame(testFrame, 0); // 0 = non-blocking
    
    if (readResult) {
        successfulReads_++;
        reportSuccess_++;
        messagesReceived_++;
        lastMessageTime_ = currentTime;
        
        if (verbose_) {
            DEBUG_PRINTF("[CanRxTest] Direct CAN read: ID=0x%03X Len=%u Data=%02X%02X%02X%02X\n",
                        testFrame.identifier, testFrame.data_length_code,
                        testFrame.data[0], testFrame.data[1], testFrame.data[2], testFrame.data[3]);
        }
    } else {
        failedReads_++;
        reportFailed_++;
    }
}

void CanRxHandlerTest::printReport() {
    uint32_t uptime = millis() - startTime_;
    float totalSuccessRate = (totalPolls_ > 0) ? (float)(successfulReads_ * 100) / totalPolls_ : 0.0f;
    float reportSuccessRate = (reportPolls_ > 0) ? (float)(reportSuccess_ * 100) / reportPolls_ : 0.0f;
    float avgPollInterval = (pollIntervalCount_ > 0) ? (float)pollIntervalSum_ / pollIntervalCount_ : 0.0f;
    
    // Main report
    DEBUG_PRINTLN("[CanRxTest] ========== Core 0 Task Diagnostic Report ==========");
    DEBUG_PRINTF("[CanRxTest] Uptime: %us | Core0 checks: %u | Task active: %s\n",
                uptime, core0TaskChecks_, core0TaskActive_ ? "YES" : "NO");
    
    // Per-report statistics (last 5 seconds)
    DEBUG_PRINTF("[CanRxTest] LAST 5s: Polls: %u | Success: %u (%.1f%%) | Failed: %u\n",
                reportPolls_, reportSuccess_, reportSuccessRate, reportFailed_);
    
    // Cumulative statistics
    DEBUG_PRINTF("[CanRxTest] TOTAL: Polls: %u | Success: %u (%.1f%%) | Failed: %u\n",
                totalPolls_, successfulReads_, totalSuccessRate, failedReads_);
    
    DEBUG_PRINTF("[CanRxTest] Messages: %u | Last msg: %us ago | Max poll gap: %ums\n",
                messagesReceived_, 
                (lastMessageTime_ > 0) ? (millis() - lastMessageTime_) / 1000 : 0,
                maxPollInterval_);
    
    DEBUG_PRINTF("[CanRxTest] Avg poll interval: %.1fms | Poll rate: %.1f Hz\n",
                avgPollInterval, (avgPollInterval > 0) ? 1000.0f / avgPollInterval : 0.0f);
    
    // CanRxHandler statistics
    CanRxHandler& canRx = CanRxHandler::getInstance();
    DEBUG_PRINTF("[CanRxTest] CanRxHandler | Total msgs: %u | Queue depth: %u | Overflows: %u\n",
                canRx.getMessagesReceived(), canRx.getQueueDepth(), canRx.getQueueOverflows());
    
    // Verbose details if enabled
    if (verbose_) {
        printVerboseInfo();
    }
    
    DEBUG_PRINTLN("[CanRxTest] =====================================================");
    
    // Reset per-report counters for next interval
    reportPolls_ = 0;
    reportSuccess_ = 0;
    reportFailed_ = 0;
}

void CanRxHandlerTest::printVerboseInfo() {
    DEBUG_PRINTLN("[CanRxTest] --- Verbose Details ---");
    
    // Core 0 task details
    CanRxHandler& canRx = CanRxHandler::getInstance();
    bool hasMessages = canRx.hasMessages();
    
    DEBUG_PRINTF("[CanRxTest] CanRxHandler.hasMessages(): %s\n", hasMessages ? "YES" : "NO");
    DEBUG_PRINTF("[CanRxTest] Core0 task responsiveness: %s\n", 
                (core0TaskChecks_ > 0) ? "CHECKING" : "NOT_STARTED");
    
    // CAN bus status
    DEBUG_PRINTF("[CanRxTest] Direct CAN read success rate: %.1f%%\n",
                (totalPolls_ > 0) ? (float)(successfulReads_ * 100) / totalPolls_ : 0.0f);
    
    if (lastMessageTime_ > 0) {
        DEBUG_PRINTF("[CanRxTest] Time since last successful read: %ums\n", millis() - lastMessageTime_);
    } else {
        DEBUG_PRINTLN("[CanRxTest] No successful CAN reads yet");
    }
    
    DEBUG_PRINTLN("[CanRxTest] --- End Verbose ---");
}
