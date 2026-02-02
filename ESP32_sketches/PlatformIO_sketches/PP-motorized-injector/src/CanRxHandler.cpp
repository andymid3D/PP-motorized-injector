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
    uint32_t lastDebugTime = millis();
    
    // DEBUG: Indicate task started
    Serial.println("[Core0] CanRxHandler task started");
    
    while (true) {
        handler->pollAndProcess();
        if (++pollCount >= 100) { // Reset watchdog periodically
            esp_task_wdt_reset();
            pollCount = 0;
        }
        
        // DEBUG: Report activity every 5 seconds
        uint32_t currentTime = millis();
        if (currentTime - lastDebugTime >= 5000) {
            uint32_t totalMsgs = handler->getMessagesReceived();
            uint32_t queueDepth = handler->getQueueDepth();
            Serial.printf("[Core0] Active - Total msgs: %u | Queue: %u | Polls: 100\n", totalMsgs, queueDepth);
            lastDebugTime = currentTime;
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
    if (!messageQueue_) return;

    // Read CAN frame (non-blocking)
    CanFrame frame;
    if (!ESP32Can.readFrame(frame, 0)) {
        return; // No message available
    }

    // DEBUG: Count reads
    messagesReceived_++;
    
    uint64_t timestamp = hwTimer.micros(); // Timestamp immediately

    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    uint32_t cmdId = frame.identifier & 0x1F; // Mask out Node ID (bits 5-10)

    // Process heartbeat messages (CRITICAL for axis state detection)
    if (cmdId == ODriveCANProtocol::CYCLIC_HEARTBEAT) {
        // Parse heartbeat according to ODrive spec:
        // Byte 0-3: Axis Error (uint32 - full error code)
        // Byte 4:   Axis State (uint8)
        // Byte 5:   Motor Error Flag (uint8 - boolean flag only)
        // Byte 6:   Encoder Error Flag (uint8 - boolean flag only)
        // Byte 7:   Controller Error Flag (uint8 - boolean flag) + Trajectory Done Flag (uint8)
        uint32_t axis_error = 0;
        uint8_t axis_state = 0;
        uint8_t motor_error_flag = 0;
        uint8_t encoder_error_flag = 0;
        uint8_t controller_error_flag = 0;
        uint8_t trajectory_done_flag = 0;
        
        // Parse axis error (uint32, little-endian) - this is the ONLY error code in heartbeat
        axis_error |= ((uint32_t)frame.data[0]) << 0;
        axis_error |= ((uint32_t)frame.data[1]) << 8;
        axis_error |= ((uint32_t)frame.data[2]) << 16;
        axis_error |= ((uint32_t)frame.data[3]) << 24;
        
        // Parse individual flags (boolean indicators, not error codes)
        axis_state = frame.data[4];
        motor_error_flag = frame.data[5];                    // 0=OK, 1=error present
        encoder_error_flag = frame.data[6];                  // 0=OK, 1=error present
        controller_error_flag = frame.data[7] & 0x7F;         // 0=OK, 1=error present (bits 0-6)
        trajectory_done_flag = (frame.data[7] >> 7) & 0x01;  // Bit 7
        
        // Store heartbeat with all indicators
        bds.storeHeartbeat(axis_error, axis_state, motor_error_flag, encoder_error_flag, 
                          controller_error_flag, trajectory_done_flag, timestamp, false);
        
        // NOTE: Don't store flags in error structures - those are for full error codes from dedicated messages
        // Flags are available for debugging but not used in main error reporting
    }
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
        bds.storeEncoder(manual_pos.f, manual_vel.f, timestamp, false);
    }
    else if (cmdId == ODriveCANProtocol::CYCLIC_IQ) {
        // Manual float parsing to bypass broken CAN library
        union { float f; uint8_t bytes[4]; } manual_iq, manual_id;
        manual_iq.bytes[0] = frame.data[0];
        manual_iq.bytes[1] = frame.data[1];
        manual_iq.bytes[2] = frame.data[2];
        manual_iq.bytes[3] = frame.data[3];
        manual_id.bytes[0] = frame.data[4];
        manual_id.bytes[1] = frame.data[5];
        manual_id.bytes[2] = frame.data[6];
        manual_id.bytes[3] = frame.data[7];
        bds.storeIq(manual_iq.f, manual_id.f, timestamp, false);
    }
    // Store motor error data - use manual parsing to avoid can_getSignal endianness issues
    else if (cmdId == ODriveCANProtocol::CYCLIC_MOTOR_ERROR) {
        // Parse 64-bit motor error (little-endian) - ODrive sends 64-bit for motor errors!
        uint64_t motor_error = 0;
        motor_error |= ((uint64_t)frame.data[0]) << 0;
        motor_error |= ((uint64_t)frame.data[1]) << 8;
        motor_error |= ((uint64_t)frame.data[2]) << 16;
        motor_error |= ((uint64_t)frame.data[3]) << 24;
        motor_error |= ((uint64_t)frame.data[4]) << 32;
        motor_error |= ((uint64_t)frame.data[5]) << 40;
        motor_error |= ((uint64_t)frame.data[6]) << 48;
        motor_error |= ((uint64_t)frame.data[7]) << 56;
        
        #if DEBUG_ENABLED
        Serial.print("[CAN_RX] Motor Error: 0x");
        Serial.print((unsigned long long)motor_error, HEX);
        Serial.print(" at ");
        Serial.println(timestamp);
        #endif
        
        bds.storeMotorError(motor_error, timestamp, false);
    }
    // Store encoder error data - use manual parsing to avoid can_getSignal endianness issues
    else if (cmdId == ODriveCANProtocol::CYCLIC_ENCODER_ERROR) {
        // Parse 32-bit encoder error (little-endian)
        uint32_t encoder_error = 0;
        encoder_error |= ((uint32_t)frame.data[0]) << 0;
        encoder_error |= ((uint32_t)frame.data[1]) << 8;
        encoder_error |= ((uint32_t)frame.data[2]) << 16;
        encoder_error |= ((uint32_t)frame.data[3]) << 24;
        bds.storeEncoderError(encoder_error, timestamp, false);
    }
    // Store controller error data - use manual parsing to avoid can_getSignal endianness issues
    else if (cmdId == ODriveCANProtocol::CYCLIC_CONTROLLER_ERROR) {
        // Parse 32-bit controller error (little-endian)
        uint32_t controller_error = 0;
        controller_error |= ((uint32_t)frame.data[0]) << 0;
        controller_error |= ((uint32_t)frame.data[1]) << 8;
        controller_error |= ((uint32_t)frame.data[2]) << 16;
        controller_error |= ((uint32_t)frame.data[3]) << 24;
        
        #if DEBUG_ENABLED
        Serial.print("[CAN_RX] Controller Error: 0x");
        Serial.print(controller_error, HEX);
        Serial.print(" at ");
        Serial.println(timestamp);
        #endif
        
        bds.storeControllerError(controller_error, timestamp, false);
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
