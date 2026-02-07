#include "motor_wrapper.h"
#include "broadcast_data_store.h"
#include "time_utils.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

static const char* TAG = "MOTOR_WRAPPER";

namespace MotorWrapper {

static uint64_t lastCmdTime = 0;
static std::string lastCmdStr = "None";
static int lastControlMode = -1;
static int lastInputMode = -1;
static float lastVelLimit = 0.0f;
static float lastCurrentLimit = 0.0f;
static uint8_t lastModuleId = 255;

static int currentTempC = 25;
static float currentVelocity = 0.0f;
static bool motionStateValid = false;
static uint64_t lastTempBlockLogUs = 0;
static bool tempCriticalLatched = false;

static const uint32_t RETRY_TIMEOUTS[] = {50, 75, 100, 200};
static const uint8_t MAX_RETRIES = 3;

void init() {
    lastCmdTime = 0;
    lastCmdStr = "None";
    lastControlMode = -1;
    lastInputMode = -1;
    lastVelLimit = 0.0f;
    lastCurrentLimit = 0.0f;
    lastModuleId = 255;
}

void updateMotionState(int tempC, float velocity) {
    currentTempC = tempC;
    currentVelocity = velocity;
    motionStateValid = true;

    if (!tempCriticalLatched) {
        if (currentTempC <= TEMP_CRITICAL) {
            tempCriticalLatched = true;
        }
    } else {
        if (currentTempC >= TEMP_CRITICAL_RECOVER) {
            tempCriticalLatched = false;
        }
    }
}

bool isTempCritical() {
    if (!motionStateValid) return false;
    return tempCriticalLatched;
}

bool canStartMove() {
    if (!motionStateValid) return true;
    if (isTempCritical()) return false;
    if (currentTempC < TEMP_MIN_MOVE && std::fabs(currentVelocity) < 0.1f) {
        return false;
    }
    return true;
}

static bool isStopCommand(int ctrlMode, float value) {
    if (ctrlMode == 2 && std::fabs(value) < 0.01f) return true;
    if (ctrlMode == 1 && std::fabs(value) < 0.01f) return true;
    if (ctrlMode == 3) {
        BroadcastDataStore& bds = BroadcastDataStore::getInstance();
        if (std::fabs(value - bds.getPosition()) < 0.01f) return true;
    }
    return false;
}

static bool tempAllowsCommand(int ctrlMode, float value) {
    if (!motionStateValid) return true;
    if (isStopCommand(ctrlMode, value)) return true;
    if (!canStartMove()) {
        uint64_t now = time_utils::micros();
        if (now - lastTempBlockLogUs > 2000000ULL) {
            ESP_LOGW(TAG, "TEMP_MOVE_BLOCKED: temp=%d vel=%.2f", currentTempC, currentVelocity);
            lastTempBlockLogUs = now;
        }
        return false;
    }
    return true;
}

void setMotorLimits(CanBus& motor, float vel_lim, float current_lim, uint8_t moduleId, const char* context) {
    if (!motor.setLimits(vel_lim, current_lim)) {
        ESP_LOGW(TAG, "CAN_QUEUE_FULL: Limits [M%d:%s]", moduleId, context);
        return;
    }
    lastCmdTime = time_utils::micros();
    lastCmdStr = std::string("Limits:") + context;
    lastVelLimit = vel_lim;
    lastCurrentLimit = current_lim;
    lastModuleId = moduleId;
}

void setControllerModes(CanBus& motor, odrive_can::ControlMode ctrlMode, odrive_can::InputMode inputMode, uint8_t moduleId, const char* context) {
    if (!motor.setControllerModes(ctrlMode, inputMode)) {
        ESP_LOGW(TAG, "CAN_QUEUE_FULL: Controller Modes [M%d:%s]", moduleId, context);
        return;
    }
    lastCmdTime = time_utils::micros();
    lastCmdStr = std::string("Modes:") + context;
    lastControlMode = static_cast<int>(ctrlMode);
    lastInputMode = static_cast<int>(inputMode);
    lastModuleId = moduleId;
}

void setTrapTrajParams(CanBus& motor, float vel_limit, float accel, float decel, uint8_t moduleId, const char* context) {
    if (!motor.setTrajVelLimit(vel_limit)) {
        ESP_LOGW(TAG, "CAN_QUEUE_FULL: TrajVel [M%d:%s]", moduleId, context);
        return;
    }
    if (!motor.setTrajAccelLimits(accel, decel)) {
        ESP_LOGW(TAG, "CAN_QUEUE_FULL: TrajAccel [M%d:%s]", moduleId, context);
        return;
    }
    lastCmdTime = time_utils::micros();
    lastCmdStr = std::string("TrapParams:") + context;
    lastModuleId = moduleId;
}

void adjustMotorLimits(CanBus& motor, float current_lim, uint8_t moduleId, const char* reason) {
    setMotorLimits(motor, lastVelLimit, current_lim, moduleId, reason);
}

void universalStop(CanBus& motor, uint8_t moduleId, const char* reason) {
    motor.setAxisState(odrive_can::AxisState::IDLE);
    ESP_LOGI(TAG, "UNIVERSAL_STOP [M%d:%s]", moduleId, reason);
}

void safeModeChange(CanBus& motor, int ctrlMode, int inputMode, uint8_t moduleId, const char* cmdName) {
    universalStop(motor, moduleId, "PreModeChange");
    vTaskDelay(pdMS_TO_TICKS(20));
    motor.setControllerModes(static_cast<odrive_can::ControlMode>(ctrlMode),
                             static_cast<odrive_can::InputMode>(inputMode));
    vTaskDelay(pdMS_TO_TICKS(20));
    motor.setAxisState(odrive_can::AxisState::CLOSED_LOOP_CONTROL);
    ESP_LOGI(TAG, "SAFE_MODE_CHANGE [M%d:%s]", moduleId, cmdName);
}

bool setModeAndMove(CanBus& motor, int ctrlMode, int inputMode, float value, uint8_t moduleId, const char* cmdName) {
    if (!tempAllowsCommand(ctrlMode, value)) {
        return false;
    }

    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    float currentVelocity = bds.getVelocity();

    bool isDirectionChange = false;
    if (std::fabs(currentVelocity) > 0.5f) {
        if ((ctrlMode == 2 && value * currentVelocity < 0) ||
            (ctrlMode == 3 && std::fabs(value - bds.getPosition()) > 1.0f)) {
            isDirectionChange = true;
        }
    }

    if (isDirectionChange) {
        ESP_LOGW(TAG, "Direction change detected, stopping first");
        motor.setInputVel(0.0f);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    bool stopCmd = isStopCommand(ctrlMode, value);
    bool moduleChanged = (lastModuleId != moduleId);
    if (!stopCmd && (moduleChanged || lastControlMode != ctrlMode || lastInputMode != inputMode)) {
        safeModeChange(motor, ctrlMode, inputMode, moduleId, cmdName);
    }

    bool queued = false;
    if (ctrlMode == 1) queued = motor.setInputTorque(value);
    else if (ctrlMode == 2) queued = motor.setInputVel(value);
    else if (ctrlMode == 3) queued = motor.setInputPos(value);

    if (!queued) {
        ESP_LOGW(TAG, "CAN_QUEUE_FULL: Setpoint [M%d:%s]", moduleId, cmdName);
        return false;
    }

    lastCmdTime = time_utils::micros();
    lastCmdStr = cmdName;
    lastControlMode = ctrlMode;
    lastInputMode = inputMode;
    lastModuleId = moduleId;
    return true;
}

bool setModeAndMoveWithRetry(CanBus& motor, int ctrlMode, int inputMode, float value, uint8_t moduleId, const char* cmdName, RetryPriority priority) {
    uint32_t timeoutMs = RETRY_TIMEOUTS[priority];
    uint8_t retryCount = 0;

    while (retryCount < MAX_RETRIES) {
        if (!setModeAndMove(motor, ctrlMode, inputMode, value, moduleId, cmdName)) {
            return false;
        }

        uint64_t startTime = time_utils::micros();
        bool commandStarted = false;
        while ((time_utils::micros() - startTime) < (timeoutMs * 1000ULL)) {
            if ((time_utils::micros() - startTime) > (25 * 1000ULL)) {
                commandStarted = true;
                break;
            }
        }

        if (commandStarted) {
            return true;
        }

        retryCount++;
        if (retryCount < MAX_RETRIES) {
            static uint64_t retryDelayStart = 0;
            static uint8_t currentRetryCount = 0;
            if (retryDelayStart == 0 || currentRetryCount != retryCount) {
                retryDelayStart = time_utils::micros();
                currentRetryCount = retryCount;
            }
            if ((time_utils::micros() - retryDelayStart) < (50 * retryCount * 1000ULL)) {
                retryCount--;
                return false;
            }
            retryDelayStart = 0;
        }
    }

    ESP_LOGW(TAG, "CMD_FAILED: [M%d:%s]", moduleId, cmdName);
    return false;
}

const char* getLastCommand() { return lastCmdStr.c_str(); }
int getLastControlMode() { return lastControlMode; }
int getLastInputMode() { return lastInputMode; }
float getLastVelLimit() { return lastVelLimit; }
float getLastCurrentLimit() { return lastCurrentLimit; }

uint64_t timeSinceLastCommand() { return time_utils::micros() - lastCmdTime; }

} // namespace MotorWrapper
