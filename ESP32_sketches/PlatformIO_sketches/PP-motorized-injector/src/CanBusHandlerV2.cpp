#include "CanBusHandlerV2.h"
#include "BroadcastDataStore.h"
#include "MessageBuffer.h"
#include "config.h"
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

/**
 * CAN Bus Handler V2 Implementation
 * Lean wrapper around ODrive CANSimple protocol
 * 
 * Cyclic Message Integration:
 * All incoming cyclic messages are:
 * 1. Parsed via ODriveCANProtocol parsers
 * 2. Stored in CanBusHandlerV2 internal structures (for backward compatibility)
 * 3. Fed to BroadcastDataStore for non-blocking access by state machines
 */

// Helper: Convert ODrive can_Message_t to ESP32 CanFrame (twai_message_t)
static CanFrame messageToFrame(const can_Message_t& msg) {
    CanFrame frame;
    frame.identifier = msg.id;
    frame.extd = msg.isExt ? 1 : 0;
    frame.rtr = msg.rtr ? 1 : 0;
    frame.data_length_code = msg.len;
    std::memcpy(frame.data, msg.buf, 8);
    return frame;
}

// Helper: Convert ESP32 CanFrame (twai_message_t) to ODrive can_Message_t
static can_Message_t frameToMessage(const CanFrame& frame) {
    can_Message_t msg;
    msg.id = frame.identifier;
    msg.isExt = frame.extd ? true : false;
    msg.rtr = frame.rtr ? true : false;
    msg.len = frame.data_length_code;
    std::memcpy(msg.buf, frame.data, 8);
    return msg;
}

void CanBusHandlerV2::begin() {
    // Initialize CAN bus (ESP32 TWAI interface)
    ESP32Can.setPins(PIN_CAN_TX, PIN_CAN_RX);
    ESP32Can.setSpeed(TWAI_SPEED_250KBPS);
    ESP32Can.begin();
}

void CanBusHandlerV2::loop() {
    // REMOVED: All CAN RX processing from CanBusHandlerV2::loop()
    // CPU0 (CanRxHandler) is now SOLELY responsible for reading incoming CAN messages.
    // CanBusHandlerV2 will get its needed RX data from BroadcastDataStore.

    // Send next queued command if CAN_COMMAND_GAP_MS has elapsed since last send
    uint32_t now = millis();
    if (!isQueueEmpty() && (now - lastCommandSentTime_) >= CAN_COMMAND_GAP_MS) {
        // Dequeue command from tail
        can_Message_t cmd = commandQueue_[queueTail_];
        queueTail_ = (queueTail_ + 1) % CMD_QUEUE_SIZE;
        queueFull_ = false;  // No longer full after dequeue
        
        // Send to CAN bus
        CanFrame frame = messageToFrame(cmd);
        ESP32Can.writeFrame(frame);
        lastCommandSentTime_ = now;  // Track when this command was actually SENT
    }
}

bool CanBusHandlerV2::isAlive(uint32_t timeoutMs) const {
    // This method now needs to query BroadcastDataStore for heartbeat freshness
    // For now, it will return true, but this needs to be updated in a later phase
    // when BroadcastDataStore is fully populated by CanRxHandler.
    return true;
}

// ===== BACKWARD COMPATIBILITY GETTERS =====
// These provide the old interface for existing code
// Internally they use the new cyclic message structures

// Note: These are inlined in header, no implementations needed

// ===== PRIVATE HELPER: Queue command with CAN_COMMAND_GAP_MS timing =====

// Helper method to queue a command respecting CAN_COMMAND_GAP_MS
// This ensures adequate time for ODrive to process between successive commands
// Returns false if queue is full (command dropped)
bool CanBusHandlerV2::_queueCommand(const can_Message_t& cmd) {
    // Check for queue overflow
    if (queueFull_) {
        // Queue full - drop new message and log error
        char buf[64];
        snprintf(buf, sizeof(buf), "CAN_QUEUE_OVERFLOW: Dropped cmd 0x%03X", cmd.id);
        MessageBuffer::getInstance().sendMessage(buf);
        return false;  // Command not queued
    }
    
    // Enqueue command at head
    commandQueue_[queueHead_] = cmd;
    queueHead_ = (queueHead_ + 1) % CMD_QUEUE_SIZE;
    
    // Check if queue is now full
    if (queueHead_ == queueTail_) {
        queueFull_ = true;
    }
    
    return true;  // Command queued successfully
}

// Helper method to queue a command WITH RTR flag and wait for response
// BLOCKS for up to RTR_TIMEOUT_MS * RTR_RETRY_COUNT milliseconds
// Returns true if RTR response received, false on timeout/error
bool CanBusHandlerV2::_queueCommandWithRTR(const can_Message_t& msg) {
    // If RTR is disabled (RTR_RETRY_COUNT = 0), send as regular command
    if (RTR_RETRY_COUNT == 0) {
        return _queueCommand(msg);  // Send without RTR flag, no blocking wait
    }
    
    // Create RTR message (copy input, set RTR flag)
    can_Message_t rtrMsg = msg;
    rtrMsg.rtr = true;  // Set RTR flag (request remote transmission)
    
    // Attempt send with retries
    for (uint8_t attempt = 0; attempt < RTR_RETRY_COUNT; attempt++) {
        // Queue command
        if (!_queueCommand(rtrMsg)) {
            // Queue full - fatal error
            MessageBuffer::getInstance().sendMessage("RTR_FAIL: Queue full");
            return false;
        }
        
        // Initialize RTR tracking
        pendingRTR_.canId = msg.id;
        pendingRTR_.sentTime = micros();
        pendingRTR_.waiting = true;
        pendingRTR_.retryCount = attempt;
        
        // BLOCKING WAIT: Poll for RTR response or timeout
        uint32_t startTime = micros();
        while (pendingRTR_.waiting) {
            // Service CAN bus (processes RX and TX)
            loop(); // This will now only service TX
            
            // Check timeout
            if ((micros() - startTime) >= (RTR_TIMEOUT_MS * 1000UL)) {
                break;  // Timeout - try next retry
            }
            
            // Small delay to prevent tight loop (100us between polls)
            delayMicroseconds(100);
        }
        
        // Check if RTR response received
        if (!pendingRTR_.waiting) {
            // Success - RTR response received
            return true;
        }
        
        // Timeout - log and retry
        char buf[64];
        snprintf(buf, sizeof(buf), "RTR_TIMEOUT: ID=0x%03X Attempt=%d", msg.id, attempt + 1);
        MessageBuffer::getInstance().sendMessage(buf);
    }
    
    // All retries exhausted - fatal error
    char buf[64];
    snprintf(buf, sizeof(buf), "RTR_FAIL: ID=0x%03X (no response after %d attempts)", msg.id, RTR_RETRY_COUNT);
    MessageBuffer::getInstance().sendMessage(buf);
    return false;
}

// ===== Command Builders =====

bool CanBusHandlerV2::setAxisState(ODriveCANProtocol::AxisState state) {
    return _queueCommandWithRTR(ODriveCANProtocol::buildSetAxisState(NODE_ID, state));
}

bool CanBusHandlerV2::setControllerModes(ODriveCANProtocol::ControlMode ctrlMode, 
                                         ODriveCANProtocol::InputMode inputMode) {
    return _queueCommandWithRTR(ODriveCANProtocol::buildSetControllerModes(NODE_ID, ctrlMode, inputMode));
}

bool CanBusHandlerV2::setInputPos(float position) {
    return _queueCommandWithRTR(ODriveCANProtocol::buildSetInputPos(NODE_ID, position));
}

bool CanBusHandlerV2::setInputVel(float velocity) {
    return _queueCommandWithRTR(ODriveCANProtocol::buildSetInputVel(NODE_ID, velocity));
}

bool CanBusHandlerV2::setInputTorque(float torque) {
    return _queueCommandWithRTR(ODriveCANProtocol::buildSetInputTorque(NODE_ID, torque));
}

bool CanBusHandlerV2::setLimits(float velLimit, float currentLimit) {
    return _queueCommandWithRTR(ODriveCANProtocol::buildSetLimits(NODE_ID, velLimit, currentLimit));
}

bool CanBusHandlerV2::setLinearCount(int32_t count) {
    return _queueCommand(ODriveCANProtocol::buildSetLinearCount(NODE_ID, count));
}

bool CanBusHandlerV2::clearErrors() {
    return _queueCommandWithRTR(ODriveCANProtocol::buildClearErrors(NODE_ID));
}

// ===== DIAGNOSTIC & CONTROL COMMANDS (RTR) =====

bool CanBusHandlerV2::heartbeatRequest() {
    return _queueCommand(ODriveCANProtocol::buildHeartbeatRequest(NODE_ID));
}

bool CanBusHandlerV2::estop() {
    // E-stop is critical - queue immediately with highest priority
    return _queueCommand(ODriveCANProtocol::buildEstop(NODE_ID));
}

bool CanBusHandlerV2::getMotorError() {
    return _queueCommand(ODriveCANProtocol::buildGetMotorError(NODE_ID));
}

bool CanBusHandlerV2::getEncoderError() {
    return _queueCommand(ODriveCANProtocol::buildGetEncoderError(NODE_ID));
}

bool CanBusHandlerV2::getSensorlessError() {
    return _queueCommand(ODriveCANProtocol::buildGetSensorlessError(NODE_ID));
}

bool CanBusHandlerV2::setAxisNodeId(uint32_t newNodeId) {
    return _queueCommand(ODriveCANProtocol::buildSetAxisNodeId(NODE_ID, newNodeId));
}

bool CanBusHandlerV2::getEncoderCount() {
    return _queueCommand(ODriveCANProtocol::buildGetEncoderCount(NODE_ID));
}

bool CanBusHandlerV2::startAnticogging() {
    return _queueCommand(ODriveCANProtocol::buildStartAnticogging(NODE_ID));
}

bool CanBusHandlerV2::setTrajVelLimit(float trajVelLimit) {
    return _queueCommandWithRTR(ODriveCANProtocol::buildSetTrajVelLimit(NODE_ID, trajVelLimit));
}

bool CanBusHandlerV2::setTrajAccelLimits(float accelLimit, float decelLimit) {
    return _queueCommandWithRTR(ODriveCANProtocol::buildSetTrajAccelLimits(NODE_ID, accelLimit, decelLimit));
}

bool CanBusHandlerV2::setTrajInertia(float inertia) {
    return _queueCommand(ODriveCANProtocol::buildSetTrajInertia(NODE_ID, inertia));
}

void CanBusHandlerV2::reboot() {
    _queueCommand(ODriveCANProtocol::buildReboot(NODE_ID));
}

void CanBusHandlerV2::setPositionGain(float posGain) {
    _queueCommand(ODriveCANProtocol::buildSetPositionGain(NODE_ID, posGain));
}

void CanBusHandlerV2::setVelGains(float velGain, float velIntegratorGain) {
    _queueCommand(ODriveCANProtocol::buildSetVelGains(NODE_ID, velGain, velIntegratorGain));
}

void CanBusHandlerV2::getAdcVoltage() {
    _queueCommand(ODriveCANProtocol::buildGetAdcVoltage(NODE_ID));
}

void CanBusHandlerV2::getControllerError() {
    _queueCommand(ODriveCANProtocol::buildGetControllerError(NODE_ID));
}

// REMOVED: onCanMessageReceived - this is now handled by CanRxHandler on CPU0
// The internal state (heartbeat_, encoder_estimates_, etc.) in CanBusHandlerV2
// will need to be updated by reading from BroadcastDataStore if CanBusHandlerV2
// still needs these values for its own logic. For now, they are effectively stale
// if not updated by other means.


// ===== RAW CAN ACCESS (for RTRDebug module) =====
/*
bool CanBusHandlerV2::sendRawRTR(uint32_t canId) {
    can_Message_t cmd = ODriveCANProtocol::makeEmptyMessage(NODE_ID, canId);
    cmd.rtr = true;  // Set RTR flag
    cmd.len = 0;     // RTR messages must have DLC=0
    return _queueCommand(cmd);
}

bool CanBusHandlerV2::sendRawNoRTR(uint32_t canId) {
    can_Message_t cmd = ODriveCANProtocol::makeEmptyMessage(NODE_ID, canId);
    cmd.rtr = false; // No RTR flag
    cmd.len = 0;     // DLC=0 (ODrive 0.5.5+ responds to DLC=0 without RTR)
    return _queueCommand(cmd);
}

bool CanBusHandlerV2::sendRawData(uint32_t canId, const uint8_t* data, uint8_t dlc) {
    can_Message_t cmd = ODriveCANProtocol::makeEmptyMessage(NODE_ID, canId);
    cmd.rtr = false;
    cmd.len = (dlc > 8) ? 8 : dlc;  // Clamp to max 8 bytes
    std::memcpy(cmd.buf, data, cmd.len);
    return _queueCommand(cmd);
}

// Raw message buffer for RTRDebug (simple circular buffer)
static can_Message_t rawMsgBuffer[16];
static size_t rawMsgWriteIdx = 0;
static size_t rawMsgReadIdx = 0;
static size_t rawMsgCount = 0;

bool CanBusHandlerV2::hasRawMessage() const {
    return (rawMsgCount > 0);
}

bool CanBusHandlerV2::getRawMessage(uint32_t& id, uint8_t* data, uint8_t& dlc, bool& rtr) {
    if (rawMsgCount == 0) return false;
    
    can_Message_t& msg = rawMsgBuffer[rawMsgReadIdx];
    id = msg.id;
    dlc = msg.len;
    rtr = msg.rtr;
    std::memcpy(data, msg.buf, 8);
    
    rawMsgReadIdx = (rawMsgReadIdx + 1) % 16;
    rawMsgCount--;
    
    return true;
}
*/
