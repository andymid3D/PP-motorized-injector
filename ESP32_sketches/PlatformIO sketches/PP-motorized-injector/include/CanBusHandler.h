#ifndef CAN_BUS_HANDLER_H
#define CAN_BUS_HANDLER_H

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include "config.h"

// ODrive CAN Command IDs (Node ID 0)
#define CMD_HEARTBEAT           0x001
#define CMD_SET_AXIS_STATE      0x007
#define CMD_GET_ENCODER         0x009
#define CMD_SET_CONTROLLER_MODE 0x00B // <--- NEW
#define CMD_SET_INPUT_POS       0x00C
#define CMD_SET_INPUT_VEL       0x00D
#define CMD_SET_INPUT_TORQUE    0x00E
#define CMD_GET_TORQUES         0x01C
#define CMD_GET_VBUS_VOLTAGE    0x017 // <--- NEW

class CanBusHandler {
public:
    CanBusHandler();
    void begin();
    void loop(); 

    // Commands
    void setAxisState(uint32_t state); 
    void setControllerMode(uint8_t control_mode, uint8_t input_mode); 
    void setVelocity(float turns_per_sec);
    void setPosition(float turns);
    void setTorque(float torque_nm); 
    void stop(); 
    void clearErrors(); 
    void requestVbusVoltage();
    void setLimits(float velocity_limit, float current_limit);



    // Feedback
    float getPosition() { return _currentPos; }
    float getVelocity() { return _currentVel; }
    float getTorque() { return _currentTorque; } 
    float getBusVoltage() { return _busVoltage; } 
    float getBusCurrent() { return _busCurrent; }
    bool isAlive() { return (millis() - _lastHeartbeat) < 1000; }
    uint32_t getAxisError() { return _axisError; }
    uint32_t getAxisState() { return _axisState; }

private:
    float _currentPos;
    float _currentVel;
    float _currentTorque; 
    float _busVoltage; 
    float _busCurrent;
    unsigned long _lastHeartbeat;
    uint32_t _axisError = 0;
    uint32_t _axisState = 0;
    
    void sendFloat(uint32_t id, float value);
    void sendCommand(uint32_t id, uint32_t value);
};

#endif