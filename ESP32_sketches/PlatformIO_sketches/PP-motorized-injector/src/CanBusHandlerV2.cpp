#include "CanBusHandlerV2.h"
#include "BroadcastDataStore.h"
#include "MessageBuffer.h"
#include "GPTimer.h"
#include "TimingSystemTest.h"
#include "config.h"
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <cmath>
#include <cstdio>

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

    // Handle stop verification
    if (waitingForStop_) {
        uint64_t now = hwTimer.micros();
        
        // Check for timeout
        if ((now - stopCommandTime_) > (STOP_TIMEOUT_MS * 1000)) {
            waitingForStop_ = false;
            char buf[64];
            snprintf(buf, sizeof(buf), "STOP_VERIFY_TIMEOUT: Motor didn't stop in %dms", STOP_TIMEOUT_MS);
            MessageBuffer::getInstance().sendMessage(buf);
            return;
        }
        
        // Check if motor has actually stopped
        if (isMotorStopped()) {
            // Motor stopped - wait for settle time
            if ((now - stopCommandTime_) > (STOP_SETTLE_TIME_MS * 1000)) {
                waitingForStop_ = false;
                char buf[64];
                snprintf(buf, sizeof(buf), "STOP_VERIFIED: Motor stopped and settled");
                MessageBuffer::getInstance().sendMessage(buf);
            }
            return; // Don't send next command yet
        }
        
        // Still waiting for motor to stop
        return; // Don't send next command yet
    }

    // Send next queued command if CAN_COMMAND_GAP_MS has elapsed since last send
    uint64_t now = hwTimer.micros();
    if (!isQueueEmpty() && (now - lastCommandSentTime_) >= (CAN_COMMAND_GAP_MS * 1000)) {
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

bool CanBusHandlerV2::isMotorStopped() const {
    // Get current velocity from BroadcastDataStore
    BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
    float currentVelocity = broadcast.getVelocity();
    
    // Check if velocity is below threshold
    return fabs(currentVelocity) < STOP_VELOCITY_THRESHOLD;
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
    
    // Tx-triggered collection: Notify TimingSystemTest of movement commands
    // Only trigger on commands that cause actual motor movement
    switch (cmd.id) {
        case (0 << 5) | ODriveCANProtocol::MSG_SET_INPUT_POS:
            TimingSystemTest::getInstance().onMovementCommandTx(cmd);
            break;
        case (0 << 5) | ODriveCANProtocol::MSG_SET_INPUT_VEL:
            // Check if this is a stop command (velocity = 0)
            {
                float velocity;
                std::memcpy(&velocity, cmd.buf, sizeof(float));
                if (fabs(velocity) < STOP_VELOCITY_THRESHOLD) {
                    // This is a stop command - start verification
                    waitingForStop_ = true;
                    stopCommandTime_ = hwTimer.micros();
                    char buf[64];
                    snprintf(buf, sizeof(buf), "STOP_CMD_SENT: Starting verification");
                    MessageBuffer::getInstance().sendMessage(buf);
                }
            }
            TimingSystemTest::getInstance().onMovementCommandTx(cmd);
            break;
        case (0 << 5) | ODriveCANProtocol::MSG_SET_INPUT_TORQUE:
            TimingSystemTest::getInstance().onMovementCommandTx(cmd);
            break;
        // Non-movement commands (config, requests, etc.)
        // CAN_ID_SET_AXIS_STATE, CAN_ID_SET_CONTROLLER_MODES, etc. - no trigger
        default:
            break;
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

// ===== Command Builders =====

bool CanBusHandlerV2::setAxisState(ODriveCANProtocol::AxisState state) {
    return _queueCommand(ODriveCANProtocol::buildSetAxisState(NODE_ID, state));
}

bool CanBusHandlerV2::setControllerModes(ODriveCANProtocol::ControlMode ctrlMode, 
                                         ODriveCANProtocol::InputMode inputMode) {
    return _queueCommand(ODriveCANProtocol::buildSetControllerModes(NODE_ID, ctrlMode, inputMode));
}

bool CanBusHandlerV2::setInputPos(float position) {
    return _queueCommand(ODriveCANProtocol::buildSetInputPos(NODE_ID, position));
}

bool CanBusHandlerV2::setInputVel(float velocity) {
    return _queueCommand(ODriveCANProtocol::buildSetInputVel(NODE_ID, velocity, 0.0f));
}

bool CanBusHandlerV2::setInputTorque(float torque) {
    return _queueCommand(ODriveCANProtocol::buildSetInputTorque(NODE_ID, torque));
}

bool CanBusHandlerV2::setLimits(float velLimit, float currentLimit) {
    return _queueCommand(ODriveCANProtocol::buildSetLimits(NODE_ID, velLimit, currentLimit));
}

bool CanBusHandlerV2::setLinearCount(int32_t count) {
    return _queueCommand(ODriveCANProtocol::buildSetLinearCount(NODE_ID, count));
}

bool CanBusHandlerV2::clearErrors() {
    return _queueCommand(ODriveCANProtocol::buildClearErrors(NODE_ID));
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

bool CanBusHandlerV2::getSensorlessError() {
    return _queueCommand(ODriveCANProtocol::buildGetSensorlessError(NODE_ID));
}

bool CanBusHandlerV2::setAxisNodeId(uint32_t newNodeId) {
    return _queueCommand(ODriveCANProtocol::buildSetAxisNodeId(NODE_ID, newNodeId));
}

bool CanBusHandlerV2::startAnticogging() {
    return _queueCommand(ODriveCANProtocol::buildStartAnticogging(NODE_ID));
}

bool CanBusHandlerV2::setTrajVelLimit(float trajVelLimit) {
    return _queueCommand(ODriveCANProtocol::buildSetTrajVelLimit(NODE_ID, trajVelLimit));
}

bool CanBusHandlerV2::setTrajAccelLimits(float accelLimit, float decelLimit) {
    return _queueCommand(ODriveCANProtocol::buildSetTrajAccelLimits(NODE_ID, accelLimit, decelLimit));
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

// ===== DIAGNOSTIC & STATUS METHODS =====

uint8_t CanBusHandlerV2::getQueueCount() const {
    if (queueFull_) return CMD_QUEUE_SIZE;
    if (queueHead_ >= queueTail_) return queueHead_ - queueTail_;
    return CMD_QUEUE_SIZE - (queueTail_ - queueHead_);
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

// ===== QUEUE SIZE MONITORING =====

uint8_t CanBusHandlerV2::getQueueSize() const {
    if (queueFull_) return CMD_QUEUE_SIZE;
    if (queueHead_ >= queueTail_) return queueHead_ - queueTail_;
    return CMD_QUEUE_SIZE - (queueTail_ - queueHead_);
}
