#include "CanBusHandler.h"
#include <ESP32-TWAI-CAN.hpp> 

CanBusHandler::CanBusHandler() : _currentPos(0), _currentVel(0), _lastHeartbeat(0), _busVoltage(0), _busCurrent(0) {}

void CanBusHandler::begin() {
    ESP32Can.setPins(PIN_CAN_TX, PIN_CAN_RX);
    ESP32Can.setSpeed(TWAI_SPEED_250KBPS); 
    ESP32Can.begin();
}

void CanBusHandler::loop() {
    CanFrame rxFrame;
    if (ESP32Can.readFrame(rxFrame, 0)) { 
        uint32_t nodeId = rxFrame.identifier >> 5;
        uint32_t cmdId = rxFrame.identifier & 0x01F; 
        
        if (nodeId != ODRIVE_NODE_ID) return; 

        if (cmdId == CMD_HEARTBEAT) {
            _lastHeartbeat = millis();
            memcpy(&_axisError, &rxFrame.data[0], 4);
            memcpy(&_axisState, &rxFrame.data[4], 1); 
        }
        else if (cmdId == CMD_GET_ENCODER) {
            float rawPos, rawVel;
            memcpy(&rawPos, &rxFrame.data[0], 4);
            memcpy(&rawVel, &rxFrame.data[4], 4);
            
            // Handle Inversion on Read
            if (INVERT_MOTOR_DIR) {
                _currentPos = -rawPos;
                _currentVel = -rawVel;
            } else {
                _currentPos = rawPos;
                _currentVel = rawVel;
            }
        }
        else if (cmdId == CMD_GET_TORQUES) {
            memcpy(&_currentTorque, &rxFrame.data[0], 4);
        }
        else if (cmdId == CMD_GET_VBUS_VOLTAGE) { 
            memcpy(&_busVoltage, &rxFrame.data[0], 4);
            memcpy(&_busCurrent, &rxFrame.data[4], 4); // <--- NEW
        }
    }
}

// NEW: Request Voltage explicitly
void CanBusHandler::requestVbusVoltage() {
    CanFrame frame;
    frame.identifier = (ODRIVE_NODE_ID << 5) | CMD_GET_VBUS_VOLTAGE;
    frame.extd = 0;
    frame.rtr = 1; // Remote Transmission Request
    frame.data_length_code = 8;
    ESP32Can.writeFrame(frame);
}

void CanBusHandler::sendFloat(uint32_t id, float value) {
    CanFrame frame;
    frame.identifier = (ODRIVE_NODE_ID << 5) | id;
    frame.extd = 0;
    frame.data_length_code = 8;
    memcpy(&frame.data[0], &value, 4);
    memset(&frame.data[4], 0, 4);
    ESP32Can.writeFrame(frame);
}

void CanBusHandler::sendCommand(uint32_t id, uint32_t value) {
    CanFrame frame;
    frame.identifier = (ODRIVE_NODE_ID << 5) | id;
    frame.extd = 0;
    frame.data_length_code = 8; 
    memcpy(&frame.data[0], &value, 4);
    memset(&frame.data[4], 0, 4);
    ESP32Can.writeFrame(frame);
}

void CanBusHandler::setAxisState(uint32_t state) {
    sendCommand(CMD_SET_AXIS_STATE, state);
}

void CanBusHandler::setControllerMode(uint8_t control_mode, uint8_t input_mode) {
    CanFrame frame;
    frame.identifier = (ODRIVE_NODE_ID << 5) | CMD_SET_CONTROLLER_MODE;
    frame.extd = 0;
    frame.data_length_code = 8;
    memcpy(&frame.data[0], &control_mode, 4);
    memcpy(&frame.data[4], &input_mode, 4);
    ESP32Can.writeFrame(frame);
}

void CanBusHandler::setVelocity(float turns_per_sec) {
    // Handle Inversion on Write
    if (INVERT_MOTOR_DIR) turns_per_sec = -turns_per_sec;

    if (turns_per_sec > VEL_LIMIT_INJECT) turns_per_sec = VEL_LIMIT_INJECT;
    if (turns_per_sec < -VEL_LIMIT_INJECT) turns_per_sec = -VEL_LIMIT_INJECT;
    
    CanFrame frame;
    frame.identifier = (ODRIVE_NODE_ID << 5) | CMD_SET_INPUT_VEL;
    frame.extd = 0;
    frame.data_length_code = 8;
    memcpy(&frame.data[0], &turns_per_sec, 4);
    float torque_ff = 0.0f;
    memcpy(&frame.data[4], &torque_ff, 4);
    ESP32Can.writeFrame(frame);
}

void CanBusHandler::setPosition(float turns) {
    // Handle Inversion on Write
    if (INVERT_MOTOR_DIR) turns = -turns;

    if (turns > POS_BOTTOM_MAX) turns = POS_BOTTOM_MAX;
    if (turns < POS_HOME) turns = POS_HOME;

    CanFrame frame;
    frame.identifier = (ODRIVE_NODE_ID << 5) | CMD_SET_INPUT_POS;
    frame.extd = 0;
    frame.data_length_code = 8;
    memcpy(&frame.data[0], &turns, 4);
    int16_t vel_ff = 0;
    int16_t torque_ff = 0;
    memcpy(&frame.data[4], &vel_ff, 2);
    memcpy(&frame.data[6], &torque_ff, 2);
    ESP32Can.writeFrame(frame);
}

void CanBusHandler::setTorque(float torque_nm) {
    if (INVERT_MOTOR_DIR) torque_nm = -torque_nm;
    
    CanFrame frame;
    frame.identifier = (ODRIVE_NODE_ID << 5) | CMD_SET_INPUT_TORQUE;
    frame.extd = 0;
    frame.data_length_code = 8;
    memcpy(&frame.data[0], &torque_nm, 4);
    ESP32Can.writeFrame(frame);
}

// ODrive 3.6 Protocol: CMD_SET_LIMITS = 0x00F
void CanBusHandler::setLimits(float velocity_limit, float current_limit) {
    CanFrame frame;
    frame.identifier = (ODRIVE_NODE_ID << 5) | 0x00F; // CMD_SET_LIMITS
    frame.extd = 0;
    frame.data_length_code = 8;
    
    // Byte 0-3: Velocity Limit (Float)
    // Byte 4-7: Current Limit (Float)
    memcpy(&frame.data[0], &velocity_limit, 4);
    memcpy(&frame.data[4], &current_limit, 4);
    
    ESP32Can.writeFrame(frame);
}

void CanBusHandler::stop() {
    setVelocity(0.0f);
}

void CanBusHandler::clearErrors() {
    sendCommand(0x018, 0);
}