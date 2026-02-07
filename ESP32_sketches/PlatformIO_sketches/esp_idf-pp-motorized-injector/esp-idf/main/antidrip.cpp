#include "antidrip.h"
#include "config.h"
#include "motor_wrapper.h"
#include "time_utils.h"
#include "esp_log.h"

namespace AntiDrip {
    static bool stateEntry = false;
    static uint64_t stateEnterTime = 0;
    static bool complete = false;
    static bool isTimeoutFlag = false;
    static bool isAbortedFlag = false;
    static bool error = false;
    static uint64_t lastCommandTime = 0;
    static bool pressureSensorChecked = false;

    void begin() {
        stateEntry = true;
        stateEnterTime = 0;
        complete = false;
        isTimeoutFlag = false;
        isAbortedFlag = false;
        error = false;
        lastCommandTime = time_utils::millis();
        pressureSensorChecked = false;
    }

    bool update(CanBus& motor) {
        uint64_t now = time_utils::millis();
        uint64_t elapsed = now - stateEnterTime;
        static bool loggedNoMove = false;

        if (stateEntry) {
            if (!MotorWrapper::canStartMove()) {
                if (!loggedNoMove) {
#if APP_DEBUG
                    ESP_LOGW("ANTIDRIP", "BLOCKED: temp gate");
#endif
                    loggedNoMove = true;
                }
                return false;
            }
            MotorWrapper::setMotorLimits(motor, ANTIDRIP_VEL_LIMIT, ANTIDRIP_CURRENT_LIMIT, MODULE_ANTIDRIP, "AntiDrip");
            if (!MotorWrapper::setModeAndMove(motor, 2, 1, ANTIDRIP_VEL, MODULE_ANTIDRIP, "AntiDrip Up")) {
                if (!loggedNoMove) {
#if APP_DEBUG
                    ESP_LOGW("ANTIDRIP", "COMMAND FAILED");
#endif
                    loggedNoMove = true;
                }
                return false;
            }
            lastCommandTime = now;
            stateEnterTime = now;
            stateEntry = false;
            loggedNoMove = false;
        }

        if (!pressureSensorChecked) {
            pressureSensorChecked = true;
        }

        if (elapsed > ANTIDRIP_TIMEOUT_MS) {
            isTimeoutFlag = true;
            complete = true;
            MotorWrapper::setModeAndMove(motor, 2, 1, 0, MODULE_ANTIDRIP, "AntiDrip Stop");
            return true;
        }

        return complete;
    }

    bool isComplete() { return complete && !isTimeoutFlag && !isAbortedFlag && !error; }
    bool isTimeout() { return complete && isTimeoutFlag; }
    bool isAborted() { return complete && isAbortedFlag; }
    bool hasError() { return error; }

    void reset() {
        stateEntry = true;
        complete = false;
        isTimeoutFlag = false;
        isAbortedFlag = false;
        error = false;
        pressureSensorChecked = false;
    }
}
