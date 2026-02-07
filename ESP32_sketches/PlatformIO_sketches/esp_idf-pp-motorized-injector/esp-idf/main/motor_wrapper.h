#ifndef MOTOR_WRAPPER_H
#define MOTOR_WRAPPER_H

#include <cstdint>
#include <string>
#include "can_bus.h"
#include "config.h"

namespace MotorWrapper {

enum RetryPriority {
    PRIORITY_CRITICAL = 0,
    PRIORITY_HIGH = 1,
    PRIORITY_NORMAL = 2,
    PRIORITY_LOW = 3
};

void init();
void updateMotionState(int tempC, float velocity);
bool canStartMove();
bool isTempCritical();

void setMotorLimits(CanBus& motor, float vel_lim, float current_lim, uint8_t moduleId, const char* context);
void setTrapTrajParams(CanBus& motor, float vel_limit, float accel, float decel, uint8_t moduleId, const char* context);
void setControllerModes(CanBus& motor, odrive_can::ControlMode ctrlMode, odrive_can::InputMode inputMode, uint8_t moduleId, const char* context);

bool setModeAndMove(CanBus& motor, int ctrlMode, int inputMode, float value, uint8_t moduleId, const char* cmdName);
bool setModeAndMoveWithRetry(CanBus& motor, int ctrlMode, int inputMode, float value, uint8_t moduleId, const char* cmdName, RetryPriority priority);

void adjustMotorLimits(CanBus& motor, float current_lim, uint8_t moduleId, const char* reason);

void universalStop(CanBus& motor, uint8_t moduleId, const char* reason);
void safeModeChange(CanBus& motor, int ctrlMode, int inputMode, uint8_t moduleId, const char* cmdName);

const char* getLastCommand();
int getLastControlMode();
int getLastInputMode();
float getLastVelLimit();
float getLastCurrentLimit();

uint64_t timeSinceLastCommand();

} // namespace MotorWrapper

#endif // MOTOR_WRAPPER_H
