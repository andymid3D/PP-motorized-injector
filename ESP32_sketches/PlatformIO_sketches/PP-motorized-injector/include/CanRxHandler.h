/**
 * @file CanRxHandler.h
 * @brief Fast-poll CAN message handler with FreeRTOS queue
 * 
 * PHASE 1 - Step 1.3: Create isolated RX handler module
 * - Hardware timestamps via GPTimer (1us resolution)
 * - FreeRTOS queue (32 messages deep)
 * - Fast polling with ESP32Can.readFrame() (proven approach)
 * - Non-blocking queue consumer for loop()
 * 
 * Step 1.6: Simple polling via ESP32Can library
 * - Polls ESP32Can.readFrame() every loop iteration (fast, non-blocking)
 * - Reads frames immediately with timestamp
 * - Queues messages for processing
 * 
 * NOTE: Uses ESP32-TWAI-CAN library (same as CanBusHandlerV2)
 *       Simple KISS approach - no complex ESP-IDF TWAI alerts needed
 * 
 * @date January 16, 2026
 */

#ifndef CAN_RX_HANDLER_H
#define CAN_RX_HANDLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <ESP32-TWAI-CAN.hpp>  // Use library we trust (not raw ESP-IDF)
#include "GPTimer.h"

/**
 * @brief CAN message structure with hardware timestamp
 * 
 * Captured in ISR, queued for processing in loop()
 * Timestamp from GPTimer ensures accurate timing independent of ISR/task delays
 */
struct CANRxMessage {
    uint32_t canId;        ///< CAN message ID (ODrive NodeID + Command ID)
    uint8_t data[8];       ///< Message data payload (8 bytes max)
    uint8_t dlc;           ///< Data length code (actual bytes used)
    uint64_t timestamp;    ///< GPTimer microseconds at reception
};

/**
 * @brief Singleton CAN RX handler with Core 0 ISR and queue
 * 
 * Architecture:
 * - ISR runs on Core 0 (TWAI interrupt)
 * - Queue bridges ISR → loop() (Core 1)
 * - Consumer checks queue every loop iteration
 * - No polling - interrupt-driven only
 * 
 * Usage:
 * @code
 * // In setup()
 * CanRxHandler& canRx = CanRxHandler::getInstance();
 * canRx.begin();  // Creates queue, registers ISR
 * 
 * // In loop()
 * CANRxMessage msg;
 * while (canRx.receiveMessage(msg)) {
 *     // Process message
 * }
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
     * @brief Initialize queue and register ISR
     * @return true if successful, false on queue creation failure
     * 
     * MUST be called in setup() before loop() starts
     * ISR connection will be implemented in Step 1.4
     */
    bool begin();
    
    /**
     * @brief Check if messages available in queue
     * @return true if queue has messages, false if empty
     * 
     * Non-blocking check, safe to call every loop iteration
     */
    bool hasMessages() const;
    
    /**
     * @brief Get next message from queue (non-blocking by default)
     * @param msg Reference to CANRxMessage struct to fill
     * @param timeoutMs Timeout in milliseconds (0 = no wait, default)
     * @return true if message retrieved, false if queue empty/timeout
     * 
     * Dequeues oldest message (FIFO order)
     * Safe to call multiple times per loop to drain queue
     */
    bool receiveMessage(CANRxMessage& msg, uint32_t timeoutMs = 0);
    
    /**
     * @brief Get current queue depth (messages waiting)
     * @return Number of messages in queue (0-32)
     * 
     * Debug/monitoring function
     */
    uint32_t getQueueDepth() const;
    
    /**
     * @brief Get total messages received since begin()
     * @return Total messages processed by ISR
     * 
     * Debug/monitoring function
     */
    uint32_t getMessagesReceived() const;
    
    /**
     * @brief Get queue overflow count (messages lost)
     * @return Number of messages dropped due to full queue
     * 
     * Debug/monitoring function - should always be 0 in production
     */
    uint32_t getQueueOverflows() const;
    
    /**
     * @brief Poll CAN bus and queue received messages
     * 
     * MUST be called every loop() iteration for low-latency message capture
     * 
     * - Polls ESP32Can.readFrame() (non-blocking)
     * - Timestamps with hwTimer.micros()
     * - Queues messages for processing
     * 
     * Simple approach: ~10us per call when no messages
     * Uses proven ESP32-TWAI-CAN library (same as CanBusHandlerV2)
     * 
     * Implemented in Step 1.6
     */
    void pollAndQueue();
    
    /**
     * @brief Start dedicated Core 0 polling task (PHASE 1: Step 1.6b)
     * @return true if task created successfully, false on error
     * 
     * Creates FreeRTOS task pinned to Core 0 that continuously polls CAN bus
     * Task runs at high priority with dedicated loopTimer for performance monitoring
     * 
     * Usage:
     * @code
     * canRx.begin();  // Create queue
     * canRx.startCore0Task();  // Launch polling task on Core 0
     * @endcode
     */
    bool startCore0Task();
    
    /**
     * @brief Stop Core 0 polling task
     * 
     * Stops the FreeRTOS task if running
     * Allows switching between task-based and loop-based polling
     */
    void stopCore0Task();
    
private:
    /**
     * @brief Private constructor (singleton pattern)
     */
    CanRxHandler();
    
    /**
     * @brief Private destructor
     */
    ~CanRxHandler();
    
    static const uint8_t QUEUE_SIZE = 128;  ///< Queue depth (128 messages, handles 700ms buffer)
    QueueHandle_t messageQueue_;           ///< FreeRTOS queue handle
    TaskHandle_t pollingTaskHandle_;       ///< Core 0 polling task handle
    
    uint32_t messagesReceived_;            ///< Total messages received
    uint32_t queueOverflows_;              ///< Messages dropped (full queue)
    
    /**
     * @brief Static task function for FreeRTOS (Core 0)
     * @param pvParameters Pointer to CanRxHandler instance
     * 
     * Continuous polling loop with performance monitoring
     * Runs on Core 0, freeing Core 1 for FSM processing
     */
    static void pollingTaskCore0(void* pvParameters);
    
    // Prevent copying (singleton)
    CanRxHandler(const CanRxHandler&) = delete;
    CanRxHandler& operator=(const CanRxHandler&) = delete;
};

#endif // CAN_RX_HANDLER_H
