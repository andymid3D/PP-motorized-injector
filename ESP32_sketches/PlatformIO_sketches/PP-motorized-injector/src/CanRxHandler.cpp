// src/CanRxHandler.cpp
#include "CanRxHandler.h"
#include <ESP32-TWAI-CAN.hpp>
#include "BroadcastDataStore.h" // ADDED: For direct classification
#include "ODriveCANProtocol.h"  // ADDED: For ODriveCANProtocol::parseCyclicHeartbeat

extern GPTimer hwTimer;

CanRxHandler& CanRxHandler::getInstance() {
    static CanRxHandler instance;
    return instance;
}

CanRxHandler::CanRxHandler()
    : messageQueue_(nullptr)
    , messagesReceived_(0)
    , queueOverflows_(0) {
}

CanRxHandler::~CanRxHandler() {
    if (messageQueue_) {
        vQueueDelete(messageQueue_);
    }
}

bool CanRxHandler::begin() {
    // Create internal FreeRTOS queue for raw messages from peripheral
    messageQueue_ = xQueueCreate(QUEUE_SIZE, sizeof(CANRxMessage));
    if (!messageQueue_) {
        Serial.println("ERROR: Failed to create CAN RX internal queue");
        return false;
    }

    // Configure watchdog: remove IDLE0, we'll reset manually
    esp_task_wdt_init(3, true);  // 3 second timeout, panic on timeout
    esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));  // Remove IDLE0
    
    Serial.println("CanRxHandler: Internal queue created (32 slots)");
    return true;
}

// Core 0 task function
void CanRxHandler::pollingTaskCore0(void* parameter) { // Corrected scope
    esp_task_wdt_add(NULL); // This adds the *current* task (pollingTaskCore0) to the WDT

    CanRxHandler* handler = static_cast<CanRxHandler*>(parameter);
    uint32_t pollCount = 0;
    while (true) {
        handler->pollAndProcess();
        if (++pollCount >= 100) { // Reset watchdog periodically
            esp_task_wdt_reset();
            pollCount = 0;
        }
        // Small delay to prevent tight loop if no messages, but keep high frequency
        delayMicroseconds(1);
    }
}

bool CanRxHandler::startCore0Task() {
    TaskHandle_t taskHandle_ = NULL;
    xTaskCreatePinnedToCore(
        pollingTaskCore0,
        "CANRxPoll",
        4096,          // Stack size
        this,
        1,             // Priority (same as Measuring task)
        &taskHandle_,
        0              // Core 0
    );
    return taskHandle_ != NULL;
}

bool CanRxHandler::hasMessages() const {
    if (!messageQueue_) return false;
    return uxQueueMessagesWaiting(messageQueue_) > 0;
}

bool CanRxHandler::receiveMessage(CANRxMessage& msg, uint32_t timeoutMs) {
    if (!messageQueue_) return false;
    
    TickType_t ticks = (timeoutMs == 0) ? 0 : pdMS_TO_TICKS(timeoutMs);
    return xQueueReceive(messageQueue_, &msg, ticks) == pdTRUE;
}

uint32_t CanRxHandler::getQueueDepth() const {
    if (!messageQueue_) return 0;
    return uxQueueMessagesWaiting(messageQueue_);
}

uint32_t CanRxHandler::getMessagesReceived() const {
    return messagesReceived_;
}

uint32_t CanRxHandler::getQueueOverflows() const {
    return queueOverflows_;
}

void CanRxHandler::pollAndProcess() {
    CanFrame frame;
    if (!ESP32Can.readFrame(frame, 0)) return; // 0 = non-blocking

    uint64_t timestamp = hwTimer.micros(); // Timestamp immediately

    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    uint32_t cmdId = frame.identifier & 0x1F; // Mask out Node ID (bits 5-10)

    // Minimal Direct Classification: ONLY Heartbeat for bundle marker
    if (cmdId == ODriveCANProtocol::CYCLIC_HEARTBEAT) {
        // Parse the raw CAN frame data into the Heartbeat struct
        ODriveCANProtocol::CyclicHeartbeat hb_data = ODriveCANProtocol::parseCyclicHeartbeat((const can_Message_t&)frame);
        // Call storeHeartbeat with the parsed components
        // Passing 0 for procedureResult as it's not directly available in CyclicHeartbeat
        bds.storeHeartbeat(hb_data.axis_error, hb_data.axis_state, 0, timestamp, false);
    }
    // All other messages are NOT classified to BDS in this micro-step.

    // Always put the message into the CanRxHandler's internal queue
    // This ensures ProtectedWindowTest can collect ALL messages for its timeline analysis.
    CANRxMessage msg;
    msg.canId = frame.identifier;
    msg.dlc = frame.data_length_code;
    memcpy(msg.data, frame.data, frame.data_length_code);
    msg.timestamp = timestamp;

    if (xQueueSend(messageQueue_, &msg, 0) == pdPASS) {
        messagesReceived_++;
    } else {
        queueOverflows_++;
    }
}

void CanRxHandler::setOutput(BufferedOutput* serialOut) { // Corrected parameter name
    serialOut_ = serialOut; // Corrected member name
}

// Basic implementation for drainAndStore() for this micro-step
uint32_t CanRxHandler::drainAndStore() {
    CANRxMessage msg;
    uint32_t count = 0;
    while(receiveMessage(msg, 0)) {
        // For this micro-step, pollAndProcess() only puts into internal queue (and classifies Heartbeat to BDS).
        // So, this just drains the internal queue.
        count++;
    }
    return count;
}
