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
    // Store encoder data for timing system
    else if (cmdId == ODriveCANProtocol::CYCLIC_ENCODER_ESTIMATES) {
        // Manual float parsing to bypass broken CAN library
        union { float f; uint8_t bytes[4]; } manual_pos, manual_vel;
        manual_pos.bytes[0] = frame.data[0];
        manual_pos.bytes[1] = frame.data[1];
        manual_pos.bytes[2] = frame.data[2];
        manual_pos.bytes[3] = frame.data[3];
        manual_vel.bytes[0] = frame.data[4];
        manual_vel.bytes[1] = frame.data[5];
        manual_vel.bytes[2] = frame.data[6];
        manual_vel.bytes[3] = frame.data[7];
        
        // Store manually parsed values
        bds.storeEncoder(manual_pos.f, manual_vel.f, timestamp, false);
        
        // DEBUG: Commented out to reduce streaming noise
        // Serial.print("[CAN-FIX] Manual: pos=");
        // Serial.print(manual_pos.f, 6);
        // Serial.print(" vel=");
        // Serial.print(manual_vel.f, 6);
        // Serial.print(" RAW: ");
        // for (int i = 0; i < frame.data_length_code; i++) {
        //     if (frame.data[i] < 0x10) Serial.print("0");
        //     Serial.print(frame.data[i], HEX);
        //     Serial.print(" ");
        // }
        // Serial.println();
    }
    // Store IQ data for timing system
    else if (cmdId == ODriveCANProtocol::CYCLIC_IQ) {
        // Manual float parsing to bypass broken CAN library
        union { float f; uint8_t bytes[4]; } manual_setpoint, manual_measured;
        manual_setpoint.bytes[0] = frame.data[0];
        manual_setpoint.bytes[1] = frame.data[1];
        manual_setpoint.bytes[2] = frame.data[2];
        manual_setpoint.bytes[3] = frame.data[3];
        manual_measured.bytes[0] = frame.data[4];
        manual_measured.bytes[1] = frame.data[5];
        manual_measured.bytes[2] = frame.data[6];
        manual_measured.bytes[3] = frame.data[7];
        
        // Store manually parsed values
        bds.storeIq(manual_setpoint.f, manual_measured.f, timestamp, false);
        
        // DEBUG: Commented out to reduce streaming noise
        // Serial.print("[CAN-FIX] IQ: set=");
        // Serial.print(manual_setpoint.f, 3);
        // Serial.print(" meas=");
        // Serial.print(manual_measured.f, 3);
        // Serial.print(" RAW: ");
        // for (int i = 0; i < frame.data_length_code; i++) {
        //     if (frame.data[i] < 0x10) Serial.print("0");
        //     Serial.print(frame.data[i], HEX);
        //     Serial.print(" ");
        // }
        // Serial.println();
    }
    // Store motor error data
    else if (cmdId == ODriveCANProtocol::CYCLIC_MOTOR_ERROR) {
        ODriveCANProtocol::CyclicMotorError motor_data = ODriveCANProtocol::parseCyclicMotorError((const can_Message_t&)frame);
        bds.storeMotorError(motor_data.motor_error, timestamp, false);
    }
    // Store encoder error data
    else if (cmdId == ODriveCANProtocol::CYCLIC_ENCODER_ERROR) {
        ODriveCANProtocol::CyclicEncoderError encoder_data = ODriveCANProtocol::parseCyclicEncoderError((const can_Message_t&)frame);
        bds.storeEncoderError(encoder_data.encoder_error, timestamp, false);
    }
    // Store controller error data
    else if (cmdId == ODriveCANProtocol::CYCLIC_CONTROLLER_ERROR) {
        ODriveCANProtocol::CyclicControllerError controller_data = ODriveCANProtocol::parseCyclicControllerError((const can_Message_t&)frame);
        bds.storeControllerError(controller_data.controller_error, timestamp, false);
    }
    // Store bus voltage/current data for movement detection comparison
    else if (cmdId == ODriveCANProtocol::CYCLIC_BUS_VI) {
        // Manual float parsing to bypass broken CAN library
        union { float f; uint8_t bytes[4]; } manual_voltage, manual_current;
        manual_voltage.bytes[0] = frame.data[0];
        manual_voltage.bytes[1] = frame.data[1];
        manual_voltage.bytes[2] = frame.data[2];
        manual_voltage.bytes[3] = frame.data[3];
        manual_current.bytes[0] = frame.data[4];
        manual_current.bytes[1] = frame.data[5];
        manual_current.bytes[2] = frame.data[6];
        manual_current.bytes[3] = frame.data[7];
        
        // Store manually parsed values
        bds.storeBusVI(manual_voltage.f, manual_current.f, timestamp, false);
        
        // DEBUG: Commented out to reduce streaming noise
        // Serial.print("[CAN-FIX] BUS: V=");
        // Serial.print(manual_voltage.f, 1);
        // Serial.print(" I=");
        // Serial.print(manual_current.f, 3);
        // Serial.print(" RAW: ");
        // for (int i = 0; i < frame.data_length_code; i++) {
        //     if (frame.data[i] < 0x10) Serial.print("0");
        //     Serial.print(frame.data[i], HEX);
        //     Serial.print(" ");
        // }
        // Serial.println();
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
