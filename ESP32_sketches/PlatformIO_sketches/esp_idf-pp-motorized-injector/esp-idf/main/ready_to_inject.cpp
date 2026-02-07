#include "ready_to_inject.h"
#include "config.h"
#include "motor_wrapper.h"
#include "time_utils.h"
#include "broadcast_data_store.h"
#include "injector_fsm.h"
#include <cmath>

extern commonInjectParams_t commonParams;

namespace ReadyToInject {
    static enum { IDLE_WAITING, MICRO_COMPRESSING } state = IDLE_WAITING;
    static bool stateEntry = false;
    static uint64_t stateEnterTime = 0;
    static uint64_t lastAutoCompressionTime = 0;
    static uint64_t compressionStartTime = 0;
    static bool error = false;
    static uint64_t lastCommandTime = 0;
    static bool torqueCommandSent = false;

    void begin() {
        state = IDLE_WAITING;
        stateEntry = true;
        stateEnterTime = time_utils::micros();
        lastAutoCompressionTime = time_utils::micros();
        error = false;
        lastCommandTime = time_utils::micros();
        torqueCommandSent = false;
    }

    bool update(CanBus& motor) {
        BroadcastDataStore& bds = BroadcastDataStore::getInstance();
        uint64_t now = time_utils::micros();

        if (stateEntry) {
            MotorWrapper::setMotorLimits(motor, REFILL_CONTROLLER_VEL_LIMIT, REFILL_CURRENT_LIMIT, MODULE_READY_TO_INJECT, "ReadyIdle");
            MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_READY_TO_INJECT, "Idle Stop");
            stateEntry = false;
        }

        if (state == IDLE_WAITING) {
            uint64_t timeSinceLastCompress = now - lastAutoCompressionTime;
            if (timeSinceLastCompress >= (READY_MICRO_INTERVAL_MS * 1000ULL)) {
                if (!MotorWrapper::canStartMove()) {
                    return false;
                }
                MotorWrapper::setMotorLimits(motor, COMPRESS_MICRO_VEL_LIMIT, COMPRESS_MICRO_CURRENT, MODULE_READY_TO_INJECT, "MicroCompress");
                motor.setControllerModes(odrive_can::ControlMode::TORQUE_CONTROL, odrive_can::InputMode::TORQUE_RAMP);
                state = MICRO_COMPRESSING;
                compressionStartTime = time_utils::micros();
                lastCommandTime = time_utils::micros();
                torqueCommandSent = false;
            }
            return false;
        }

        if (state == MICRO_COMPRESSING) {
            uint64_t compressionElapsed = now - compressionStartTime;
            float rampDuration = READY_MICRO_DURATION_MS / 1000.0f;
            float elapsedSec = compressionElapsed / 1000000.0f;
            float targetTorque = (commonParams.compressMicroCurrent / rampDuration) * elapsedSec;
            if (targetTorque > commonParams.compressMicroCurrent) targetTorque = commonParams.compressMicroCurrent;

            if (!torqueCommandSent) {
                bool commandQueued = MotorWrapper::setModeAndMoveWithRetry(
                    motor, 1, 6, targetTorque, MODULE_READY_TO_INJECT,
                    "MicroCompress", MotorWrapper::PRIORITY_NORMAL
                );
                if (commandQueued) {
                    torqueCommandSent = true;
                    lastCommandTime = now;
                }
            }

            bool completedByTime = compressionElapsed >= (READY_MICRO_DURATION_MS * 1000ULL);
            bool completedByStall = (compressionElapsed > 500000) && (std::fabs(bds.getVelocity()) < 0.5f);

            if (completedByTime || completedByStall) {
                MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_READY_TO_INJECT, "MicroCompress Release");
                lastAutoCompressionTime = now;
                state = IDLE_WAITING;
                torqueCommandSent = false;
                lastCommandTime = now;
            }
            return false;
        }

        return false;
    }

    bool isComplete() { return false; }
    bool isMicroCompressing() { return state == MICRO_COMPRESSING; }
    bool hasError() { return error; }

    void reset() {
        state = IDLE_WAITING;
        stateEntry = true;
        error = false;
        torqueCommandSent = false;
    }
}
