/**
 * @file CanRxHandler.h
 * @brief Fast-poll CAN message handler with Core 0 FreeRTOS task and direct classification
 * 
 * @details
 * This module is responsible for robust and high-performance CAN message reception.
 * It runs a dedicated FreeRTOS task on Core 0 to continuously poll the CAN bus,
 * timestamp incoming messages with 1µs resolution using GPTimer, and directly
 * classify them into the BroadcastDataStore.
 * 
 * An internal FreeRTOS queue is used to buffer raw messages from the CAN peripheral
 * before they are classified. This queue is primarily for internal management and
 * for testing/debugging purposes (e.g., by ProtectedWindowTest).
 * 
 * Architecture:
 * - Dedicated FreeRTOS task runs on Core 0 (CANRxPoll).
 * - This task continuously calls pollAndProcess() to:
 *   - Poll ESP32Can.readFrame() (non-blocking).
 *   - Timestamp messages with hwTimer.micros().
 *   - Places raw messages into an internal queue.
 *   - (Future: Directly classify messages from this internal queue into BroadcastDataStore).
 * - Core 1 (main loop) consumes processed data from BroadcastDataStore or
 *   (for testing) drains the internal queue via receiveMessage().
 * 
 * @date January 17, 2026 (Updated)
 */

#ifndef CAN_RX_HANDLER_H
#define CAN_RX_HANDLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <ESP32-TWAI-CAN.hpp>  // Use library we trust (not raw ESP-IDF)

#include "GPTimer.h"
#include <esp_task_wdt.h> // For WDT functions
#include <BufferedOutput.h> // For BufferedOutput

/**
 * @brief CAN message structure with hardware timestamp
 * 
 * ⚠️ CRITICAL: timestamp is captured when message is polled (by Core 0 task)
 * This timestamp is PART OF THE MESSAGE DATA - do NOT ignore or overwrite it!
 * 
 * Timestamp from GPTimer ensures accurate timing independent of task delays.
 * All consumers MUST use the existing timestamp field, never call micros()/millis() again.
 */
struct CANRxMessage {
    uint32_t canId;        ///< CAN message ID (ODrive NodeID + Command ID)
    uint8_t data[8];       ///< Message data payload (8 bytes max)
    uint8_t dlc;           ///< Data length code (actual bytes used)
    uint64_t timestamp;    ///< ⚠️ PROTECTED: GPTimer microseconds at reception - DO NOT OVERWRITE
};

/**
 * @brief Singleton CAN RX handler with Core 0 FreeRTOS task and internal queue
 * 
 * Architecture:
 * - Dedicated FreeRTOS task runs on Core 0 (CANRxPoll).
 * - This task continuously polls the CAN bus, timestamps messages, and places them
 *   into an internal FreeRTOS queue.
 * - WDT management for Core 0 is centralized here.
 * 
 * Usage:
 * @code
 * // In setup()
 * CanRxHandler& canRx = CanRxHandler::getInstance();
 * canRx.begin();  // Creates internal queue, initializes WDT for Core 0
 * canRx.startCore0Task();  // Launch polling task on Core 0
 * 
 * // In loop() (on Core 1)
 * CANRxMessage msg;
 * while (canRx.receiveMessage(msg)) { // For testing/debugging internal queue
 *     // Process message
 * }
 * // In production, Core 1 would consume from BroadcastDataStore after CPU0 classifies
 * @endcode
 */
class CanRxHandler {
public:
    /**
     * @brief Get singleton instance
     * @return Reference to singleton CanRxHandler
     */
    static CanRxHandler& getInstance();
    
    /**
     * @brief Initialize internal queue and WDT for Core 0
     * @return true if successful, false on queue creation failure
     * 
     * MUST be called in setup() before startCore0Task().
     */
    bool begin();
    
    /**
     * @brief Check if messages available in internal queue
     * @return true if queue has messages, false if empty
     * 
     * Non-blocking check, safe to call every loop iteration.
     * Primarily for testing/debugging the internal queue.
     */
    bool hasMessages() const;
    
    /**
     * @brief Get next message from internal queue (non-blocking by default)
     * @param msg Reference to CANRxMessage struct to fill
     * @param timeoutMs Timeout in milliseconds (0 = no wait, default)
     * @return true if message retrieved, false if queue empty/timeout
     * 
     * Dequeues oldest message (FIFO order).
     * Safe to call multiple times per loop to drain queue.
     * Primarily for testing/debugging the internal queue.
     */
    bool receiveMessage(CANRxMessage& msg, uint32_t timeoutMs = 0);
    
    /**
     * @brief Get current internal queue depth (messages waiting)
     * @return Number of messages in queue (0-128)
     * 
     * Debug/monitoring function.
     */
    uint32_t getQueueDepth() const;
    
    /**
     * @brief Get total messages received by Core 0 task since begin()
     * @return Total messages processed by Core 0 task.
     * 
     * Debug/monitoring function.
     */
    uint32_t getMessagesReceived() const;
    
    /**
     * @brief Get internal queue overflow count (messages lost)
     * @return Number of messages dropped due to full queue.
     * 
     * Debug/monitoring function - should always be 0 in production.
     */
    uint32_t getQueueOverflows() const;
    
    /**
     * @brief Poll CAN bus and process received messages (on Core 0 task)
     * 
     * This function is called continuously by the Core 0 FreeRTOS task.
     * - Polls ESP32Can.readFrame() (non-blocking).
     * - Timestamps with hwTimer.micros().
     * - Places raw messages into the internal queue.
     * - (Future: Directly classifies messages into BroadcastDataStore).
     * 
     * Simple approach: ~10us per call when no messages.
     * Uses proven ESP32-TWAI-CAN library (same as CanBusHandlerV2).
     */
    void pollAndProcess(); // Renamed from pollAndQueue()
    
    /**
     * @brief Start dedicated Core 0 polling task
     * @return true if task created successfully, false on error.
     * 
     * Creates FreeRTOS task pinned to Core 0 that continuously calls pollAndProcess().
     * Task runs at high priority.
     */
    bool startCore0Task();
    
    /**
     * @brief Stop Core 0 polling task
     * 
     * Stops the FreeRTOS task if running.
     */
    void stopCore0Task();
    
    /**
     * @brief Set BufferedOutput for non-blocking Core 0 logging
     * @param serialOut Pointer to BufferedOutput instance.
     * 
     * MUST be called after begin() and before startCore0Task() if using non-blocking serial.
     * If not called, Core 0 task will use blocking Serial.print() (not recommended).
     */
    void setOutput(BufferedOutput* serialOut); // Renamed parameter back to serialOut
    
    /**
     * @brief Drain internal queue and store messages to BroadcastDataStore
     * 
     * Processes all queued CAN messages and stores to BDS v2 ring buffers.
     * Should be called every loop() iteration in production code (by Core 1).
     * 
     * @return Number of messages processed this call.
     * 
     * Non-blocking - drains entire queue in one call.
     * Parses CAN ID and stores timestamped data to appropriate BDS ring buffer.
     */
    uint32_t drainAndStore();
    
private:
    /**
     * @brief Private constructor (singleton pattern)
     */
    CanRxHandler();
    
    /**
     * @brief Private destructor
     */
    ~CanRxHandler();
    
    static const uint8_t QUEUE_SIZE = 128;  ///< Internal queue depth (128 messages, handles 700ms buffer)
    QueueHandle_t messageQueue_;           ///< FreeRTOS internal queue handle
    TaskHandle_t pollingTaskHandle_;       ///< Core 0 polling task handle
    BufferedOutput* serialOut_;            ///< Non-blocking serial output (for Core 0 logging)
    
    uint32_t messagesReceived_;            ///< Total messages received by Core 0 task
    uint32_t queueOverflows_;              ///< Messages dropped by Core 0 task (full internal queue)
    
    /**
     * @brief Static task function for FreeRTOS (Core 0)
     * @param pvParameters Pointer to CanRxHandler instance
     * 
     * Continuous polling loop with performance monitoring.
     * Runs on Core 0, freeing Core 1 for FSM processing.
     */
    static void pollingTaskCore0(void* pvParameters);
    
    // Prevent copying (singleton)
    CanRxHandler(const CanRxHandler&) = delete;
    CanRxHandler& operator=(const CanRxHandler&) = delete;
};

#endif // CAN_RX_HANDLER_H
