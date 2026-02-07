#include "purge_zero.h"
#include "config.h"
#include "motor_wrapper.h"
#include "time_utils.h"
#include "broadcast_data_store.h"

namespace PurgeZero {
    static bool stateEntry = false;
    static bool complete = false;
    static bool error = false;
    static bool buttonsReleased = false;
    static uint64_t lastCommandTime = 0;
    static int lastButtonState = 0;
    static float purgeZeroPosition = 0.0f;

    void begin() {
        stateEntry = true;
        complete = false;
        error = false;
        buttonsReleased = false;
        lastCommandTime = time_utils::micros();
        lastButtonState = 0;
    }

    bool update(CanBus& motor, bool buttonUp, bool buttonDown, bool buttonCenterReleased) {
        uint64_t now = time_utils::micros();

        if (stateEntry) {
            MotorWrapper::setMotorLimits(motor, PURGE_VEL_LIMIT, PURGE_CURRENT_LIMIT, MODULE_PURGE_ZERO, "PurgeZero");
            lastCommandTime = now;
            stateEntry = false;
        }

        if (!buttonsReleased) {
            if (!buttonUp && !buttonDown) {
                // both released (active low)
                buttonsReleased = true;
            }
            return false;
        }

        int currentButtonState = 0;
        if (buttonUp) {
            currentButtonState = 1;
        } else if (buttonDown) {
            currentButtonState = 2;
        }

        if (currentButtonState != lastButtonState) {
            if (currentButtonState == 1) {
                MotorWrapper::setModeAndMove(motor, 2, 1, PURGE_VEL_UP, MODULE_PURGE_ZERO, "Purge Up");
            } else if (currentButtonState == 2) {
                MotorWrapper::setModeAndMove(motor, 2, 1, PURGE_VEL_DOWN, MODULE_PURGE_ZERO, "Purge Down");
            } else {
                MotorWrapper::setModeAndMove(motor, 2, 1, 0, MODULE_PURGE_ZERO, "Purge Stop");
            }
            lastButtonState = currentButtonState;
            lastCommandTime = now;
        }

        if (buttonCenterReleased) {
            BroadcastDataStore& bds = BroadcastDataStore::getInstance();
            purgeZeroPosition = bds.getPosition();
            MotorWrapper::setModeAndMove(motor, 2, 1, 0, MODULE_PURGE_ZERO, "Purge Confirm");
            lastCommandTime = now;
            complete = true;
            return true;
        }

        return false;
    }

    bool isComplete() { return complete && !error; }
    bool hasError() { return error; }
    float getPurgeZeroPosition() { return purgeZeroPosition; }

    void reset() {
        stateEntry = true;
        complete = false;
        error = false;
        buttonsReleased = false;
    }
}
