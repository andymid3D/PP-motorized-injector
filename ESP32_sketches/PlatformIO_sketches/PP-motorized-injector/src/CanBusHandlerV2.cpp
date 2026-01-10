#include "CanBusHandlerV2.h"
#include "BroadcastDataStore.h"
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
    // Service CAN RX queue - read all available frames
    CanFrame rxFrame;
    if (ESP32Can.readFrame(rxFrame, 0)) {
        // Extract node ID (bits 5-12) and message ID (bits 0-4)
        uint32_t nodeId = rxFrame.identifier >> 5;
        uint32_t msgId = rxFrame.identifier & 0x1F;
        
        // Only process messages from our ODrive
        if (nodeId != NODE_ID) return;
        
        // Convert to ODrive protocol format for parsing
        can_Message_t msg = frameToMessage(rxFrame);
        
        // Get BroadcastDataStore instance for feeding parsed data
        BroadcastDataStore& broadcastStore = BroadcastDataStore::getInstance();
        
        // Parse cyclic broadcast messages
        switch (msgId) {
            case ODriveCANProtocol::CYCLIC_HEARTBEAT: {
                auto hb = ODriveCANProtocol::parseHeartbeat(msg);
                // Copy from Heartbeat struct to CyclicHeartbeat (same layout)
                heartbeat_.axis_error = hb.axis_error;
                heartbeat_.axis_state = hb.axis_state;
                heartbeat_.motor_error_flag = hb.motor_error_flag;
                heartbeat_.encoder_error_flag = hb.encoder_error_flag;
                heartbeat_.controller_error_flag = hb.controller_error_flag;
                heartbeat_.trajectory_done_flag = hb.trajectory_done_flag;
                lastHeartbeatTime_ = millis();
                
                // Feed to BroadcastDataStore
                broadcastStore.updateAxisState(hb.axis_state);
                broadcastStore.updateAxisError(hb.axis_error);
                break;
            }
                
            case ODriveCANProtocol::CYCLIC_ENCODER_ESTIMATES: {
                auto ee = ODriveCANProtocol::parseEncoderEstimate(msg);
                // Copy from EncoderEstimate struct to CyclicEncoderEstimates (same layout)
                encoder_estimates_.position = ee.position;
                encoder_estimates_.velocity = ee.velocity;
                encoderEstimatesRxCount_++;  // Track that we received this message
                
                // Feed to BroadcastDataStore (position/velocity in turns with decimals)
                broadcastStore.updateEncoderEstimates(ee.position, ee.velocity);
                break;
            }
            
            case ODriveCANProtocol::CYCLIC_IQ: {
                Iq_ = ODriveCANProtocol::parseCyclicIq(msg);
                
                // Feed to BroadcastDataStore
                // Note: updatePowerData requires SafeString for logging, so store locally
                // Iq data available via getIqReadings() from CanBusHandlerV2
                break;
            }
            
            case ODriveCANProtocol::CYCLIC_BUS_VI: {
                bus_vi_ = ODriveCANProtocol::parseCyclicBusVoltageCurrent(msg);
                
                // Feed to BroadcastDataStore
                // Note: updatePowerData requires SafeString for logging, so store locally
                // Bus V/I data available via getBusVoltageCurrentReadings() from CanBusHandlerV2
                break;
            }
            
            case ODriveCANProtocol::CYCLIC_MOTOR_ERROR: {
                auto me = ODriveCANProtocol::parseCyclicMotorError(msg);
                motor_error_ = me;
                
                // Feed to BroadcastDataStore (no logging here - let SerialMessaging handle frequency)
                broadcastStore.updateMotorError(me.motor_error);
                break;
            }
            
            case ODriveCANProtocol::CYCLIC_ENCODER_ERROR: {
                auto ee = ODriveCANProtocol::parseCyclicEncoderError(msg);
                encoder_error_ = ee;
                
                // Feed to BroadcastDataStore
                broadcastStore.updateEncoderError(ee.encoder_error);
                break;
            }
            
            case ODriveCANProtocol::CYCLIC_CONTROLLER_ERROR: {
                auto ce = ODriveCANProtocol::parseCyclicControllerError(msg);
                controller_error_ = ce;
                
                // Feed to BroadcastDataStore
                broadcastStore.updateControllerError(ce.controller_error);
                break;
            }
        }
    }
    
    // Send pending command if CAN_COMMAND_GAP_MS has elapsed since last send
    uint32_t now = millis();
    if (hasPendingCommand_ && (now - lastCommandSentTime_) >= CAN_COMMAND_GAP_MS) {
        CanFrame frame = messageToFrame(pendingCommand_);
        ESP32Can.writeFrame(frame);
        hasPendingCommand_ = false;
        lastCommandSentTime_ = now;  // Track when this command was actually SENT
    }
}

bool CanBusHandlerV2::isAlive(uint32_t timeoutMs) const {
    return (millis() - lastHeartbeatTime_) < timeoutMs;
}

// ===== BACKWARD COMPATIBILITY GETTERS =====
// These provide the old interface for existing code
// Internally they use the new cyclic message structures

// Note: These are inlined in header, no implementations needed

// ===== PRIVATE HELPER: Queue command with CAN_COMMAND_GAP_MS timing =====

// Helper method to queue a command respecting CAN_COMMAND_GAP_MS
// This ensures adequate time for ODrive to process between successive commands
void CanBusHandlerV2::_queueCommand(const can_Message_t& cmd) {
    pendingCommand_ = cmd;
    hasPendingCommand_ = true;
    // Command will be sent by loop() when CAN_COMMAND_GAP_MS has elapsed since last send
    // No need to manipulate lastCommandTime_ here - just queue it
}

// ===== Command Builders =====

void CanBusHandlerV2::setAxisState(ODriveCANProtocol::AxisState state) {
    _queueCommand(ODriveCANProtocol::buildSetAxisState(NODE_ID, state));
}

void CanBusHandlerV2::setControllerModes(ODriveCANProtocol::ControlMode ctrlMode, 
                                         ODriveCANProtocol::InputMode inputMode) {
    _queueCommand(ODriveCANProtocol::buildSetControllerModes(NODE_ID, ctrlMode, inputMode));
}

void CanBusHandlerV2::setInputPos(float position) {
    _queueCommand(ODriveCANProtocol::buildSetInputPos(NODE_ID, position));
}

void CanBusHandlerV2::setInputVel(float velocity) {
    _queueCommand(ODriveCANProtocol::buildSetInputVel(NODE_ID, velocity));
}

void CanBusHandlerV2::setInputTorque(float torque) {
    _queueCommand(ODriveCANProtocol::buildSetInputTorque(NODE_ID, torque));
}

void CanBusHandlerV2::setLimits(float velLimit, float currentLimit) {
    _queueCommand(ODriveCANProtocol::buildSetLimits(NODE_ID, velLimit, currentLimit));
}

void CanBusHandlerV2::setLinearCount(int32_t count) {
    _queueCommand(ODriveCANProtocol::buildSetLinearCount(NODE_ID, count));
}

void CanBusHandlerV2::clearErrors() {
    _queueCommand(ODriveCANProtocol::buildClearErrors(NODE_ID));
}

// ===== DIAGNOSTIC & CONTROL COMMANDS (RTR) =====

void CanBusHandlerV2::heartbeatRequest() {
    _queueCommand(ODriveCANProtocol::buildHeartbeatRequest(NODE_ID));
}

void CanBusHandlerV2::estop() {
    // E-stop doesn't need the gap - send immediately
    pendingCommand_ = ODriveCANProtocol::buildEstop(NODE_ID);
    hasPendingCommand_ = true;
    lastCommandTime_ = millis();  // Force send on next loop
}

void CanBusHandlerV2::getMotorError() {
    _queueCommand(ODriveCANProtocol::buildGetMotorError(NODE_ID));
}

void CanBusHandlerV2::getEncoderError() {
    _queueCommand(ODriveCANProtocol::buildGetEncoderError(NODE_ID));
}

void CanBusHandlerV2::getSensorlessError() {
    _queueCommand(ODriveCANProtocol::buildGetSensorlessError(NODE_ID));
}

void CanBusHandlerV2::setAxisNodeId(uint32_t newNodeId) {
    _queueCommand(ODriveCANProtocol::buildSetAxisNodeId(NODE_ID, newNodeId));
}

void CanBusHandlerV2::getEncoderCount() {
    _queueCommand(ODriveCANProtocol::buildGetEncoderCount(NODE_ID));
}

void CanBusHandlerV2::startAnticogging() {
    _queueCommand(ODriveCANProtocol::buildStartAnticogging(NODE_ID));
}

void CanBusHandlerV2::setTrajVelLimit(float trajVelLimit) {
    _queueCommand(ODriveCANProtocol::buildSetTrajVelLimit(NODE_ID, trajVelLimit));
}

void CanBusHandlerV2::setTrajAccelLimits(float accelLimit, float decelLimit) {
    _queueCommand(ODriveCANProtocol::buildSetTrajAccelLimits(NODE_ID, accelLimit, decelLimit));
}

void CanBusHandlerV2::setTrajInertia(float inertia) {
    _queueCommand(ODriveCANProtocol::buildSetTrajInertia(NODE_ID, inertia));
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

void CanBusHandlerV2::onCanMessageReceived(const can_Message_t& msg) {
    // Decode message based on CAN ID
    uint8_t nodeId = (msg.id >> 5) & 0x3F;
    uint8_t cmdId = msg.id & 0x1F;
    
    // Only process messages for our node
    if (nodeId != NODE_ID) return;
    
    // Parse all cyclic broadcast messages
    switch (cmdId) {
        case ODriveCANProtocol::CyclicMessageID::CYCLIC_HEARTBEAT:
            heartbeat_ = ODriveCANProtocol::parseCyclicHeartbeat(msg);
            lastHeartbeatTime_ = millis();
            break;
            
        case ODriveCANProtocol::CyclicMessageID::CYCLIC_ENCODER_ESTIMATES:
            encoder_estimates_ = ODriveCANProtocol::parseCyclicEncoderEstimates(msg);
            break;
            
        case ODriveCANProtocol::CyclicMessageID::CYCLIC_MOTOR_ERROR:
            motor_error_ = ODriveCANProtocol::parseCyclicMotorError(msg);
            break;
            
        case ODriveCANProtocol::CyclicMessageID::CYCLIC_ENCODER_ERROR:
            encoder_error_ = ODriveCANProtocol::parseCyclicEncoderError(msg);
            break;
            
        case ODriveCANProtocol::CyclicMessageID::CYCLIC_SENSORLESS_ERROR:
            sensorless_error_ = ODriveCANProtocol::parseCyclicSensorlessError(msg);
            break;
            
        case ODriveCANProtocol::CyclicMessageID::CYCLIC_ENCODER_COUNT:
            encoder_count_ = ODriveCANProtocol::parseCyclicEncoderCount(msg);
            break;
            
        case ODriveCANProtocol::CyclicMessageID::CYCLIC_IQ:
            Iq_ = ODriveCANProtocol::parseCyclicIq(msg);
            break;
            
        case ODriveCANProtocol::CyclicMessageID::CYCLIC_SENSORLESS_ESTIMATES:
            sensorless_estimates_ = ODriveCANProtocol::parseCyclicSensorlessEstimates(msg);
            break;
            
        case ODriveCANProtocol::CyclicMessageID::CYCLIC_BUS_VI:
            bus_vi_ = ODriveCANProtocol::parseCyclicBusVoltageCurrent(msg);
            break;
            
        case ODriveCANProtocol::CyclicMessageID::CYCLIC_CONTROLLER_ERROR:
            controller_error_ = ODriveCANProtocol::parseCyclicControllerError(msg);
            break;
            
        default:
            // Unknown cyclic message ID
            break;
    }
}
