#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <cstdint>
#include "driver/twai.h"
#include "odrive_can.h"
#include "config.h"

class CanBus {
public:
    void begin();
    void loop();

    // Commands
    bool setAxisState(odrive_can::AxisState state);
    bool setControllerModes(odrive_can::ControlMode ctrl, odrive_can::InputMode input);
    bool setInputPos(float pos);
    bool setInputVel(float vel, float torque_ff = 0.0f);
    bool setInputTorque(float torque);
    bool setLimits(float vel_limit, float current_limit);
    bool setTrajVelLimit(float vel_limit);
    bool setTrajAccelLimits(float accel, float decel);
    bool setLinearCount(int32_t count);
    bool clearErrors();
    bool estop();

    uint8_t getQueueDepth() const;

private:
    bool queueCommand(const twai_message_t& cmd);
    bool isQueueEmpty() const;
    bool isQueueFull() const;

    static constexpr uint8_t CMD_QUEUE_SIZE = 16;
    twai_message_t commandQueue_[CMD_QUEUE_SIZE];
    uint8_t queueHead_ = 0;
    uint8_t queueTail_ = 0;
    bool queueFull_ = false;

    uint64_t lastCommandSentTime_ = 0;

    // Stop verification
    bool waitingForStop_ = false;
    uint64_t stopCommandTime_ = 0;
    static constexpr float STOP_VELOCITY_THRESHOLD = 0.5f;
    static constexpr uint32_t STOP_SETTLE_TIME_MS = 100;
    static constexpr uint32_t STOP_TIMEOUT_MS = 500;

    bool isMotorStopped() const;
};

#endif // CAN_BUS_H
