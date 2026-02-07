#include "compression.h"
#include "config.h"
#include "motor_wrapper.h"
#include "time_utils.h"
#include "injector_fsm.h"
#include "broadcast_data_store.h"
#include <cmath>

extern commonInjectParams_t commonParams;

namespace Compression {
    static CompressionMode currentMode = MODE_1_TRAVEL;
    static enum { PRESSURE_CHECK, TRAVEL_DOWN, TORQUE_RAMP, DONE } step = DONE;

    static bool stateEntry = false;
    static uint64_t stateEnterTime = 0;
    static uint64_t stepTimer = 0;
    static bool complete = false;
    static bool isErrorFlag = false;
    static bool isTimeoutFlag = false;
    static bool pressureSensorChecked = false;
    static uint64_t lastCommandTime = 0;
    static bool torqueCommandSent = false;

    void begin(CompressionMode mode) {
        currentMode = mode;
        stateEntry = true;
        stateEnterTime = time_utils::micros();
        stepTimer = time_utils::micros();
        complete = false;
        isErrorFlag = false;
        isTimeoutFlag = false;
        pressureSensorChecked = false;
        lastCommandTime = time_utils::micros();
        torqueCommandSent = false;

        if (mode == MODE_1_TRAVEL) step = PRESSURE_CHECK;
        else step = TORQUE_RAMP;
    }

    bool update(CanBus& motor) {
        uint64_t now = time_utils::micros();
        uint64_t stepElapsed = now - stepTimer;

        if (step == PRESSURE_CHECK) {
            if (stateEntry) {
                pressureSensorChecked = true;
                stateEntry = false;
            }
            if (stepElapsed > 50) {
                step = TRAVEL_DOWN;
                stepTimer = now;
                stateEntry = true;
            }
            return false;
        }

        if (step == TRAVEL_DOWN) {
            if (stateEntry) {
                if (!MotorWrapper::canStartMove()) {
                    return false;
                }
                MotorWrapper::setMotorLimits(motor, COMPRESS_TRAVEL_VEL_LIMIT, REFILL_CURRENT_LIMIT, MODULE_COMPRESSION, "Compress Travel");
                if (!MotorWrapper::setModeAndMove(motor, 1, 6, COMPRESS_TRAVEL_TORQUE, MODULE_COMPRESSION, "Compress Travel Down Torque")) {
                    return false;
                }
                lastCommandTime = time_utils::micros();
                stepTimer = time_utils::micros();
                stateEntry = false;
            }

            uint64_t travelElapsed = time_utils::micros() - stepTimer;
            BroadcastDataStore& bds = BroadcastDataStore::getInstance();
            bool stallDetected = bds.getAxisError() != 0;
            const TimestampedIq* iqData = bds.getLatestIq();
            bool torqueExceeded = false;
            if (iqData) {
                torqueExceeded = travelElapsed > INJECT_STABLE_TIME_MS && std::fabs(bds.getVelocity()) < 0.1f && iqData->iqMeasured > COMPRESS_CONTACT_IQ_THRESHOLD;
            }

            if (stallDetected || torqueExceeded) {
                MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_COMPRESSION, "Compress Stop");
                MotorWrapper::adjustMotorLimits(motor, COMPRESS_CONTACT_CURRENT, MODULE_COMPRESSION, "Contact Detected");
                step = TORQUE_RAMP;
                stepTimer = time_utils::micros();
                stateEntry = true;
                return false;
            }

            if (travelElapsed > (COMPRESS_TRAVEL_TIMEOUT_MS * 1000ULL)) {
                isTimeoutFlag = true;
                complete = true;
                return true;
            }
            return false;
        }

        if (step == TORQUE_RAMP) {
            if (stateEntry) {
                if (!MotorWrapper::canStartMove()) {
                    return false;
                }
                if (currentMode == MODE_2_MICRO) {
                    MotorWrapper::setMotorLimits(motor, COMPRESS_MICRO_VEL_LIMIT, COMPRESS_MICRO_CURRENT, MODULE_COMPRESSION, "Micro Torque");
                    motor.setControllerModes(odrive_can::ControlMode::TORQUE_CONTROL, odrive_can::InputMode::TORQUE_RAMP);
                }
                stepTimer = time_utils::micros();
                stateEntry = false;
            }

            BroadcastDataStore& bds = BroadcastDataStore::getInstance();
            float rampDuration = commonParams.compressRampDuration; // seconds
            uint64_t rampElapsed = time_utils::micros() - stepTimer;
            float targetTorque = (commonParams.compressRampTarget / rampDuration) * (rampElapsed / 1000000.0f);
            if (targetTorque > commonParams.compressRampTarget) targetTorque = commonParams.compressRampTarget;

            if (!torqueCommandSent) {
                bool commandQueued = MotorWrapper::setModeAndMoveWithRetry(
                    motor, 1, 6, targetTorque, MODULE_COMPRESSION,
                    "CompressRamp", MotorWrapper::PRIORITY_HIGH
                );
                if (!commandQueued) {
                    return false;
                }
                torqueCommandSent = true;
                lastCommandTime = now;
            }

            bool reachedTorqueTarget = (targetTorque >= commonParams.compressRampTarget);
            bool stallDetected = rampElapsed > INJECT_STABLE_TIME_MS && bds.getAxisError() != 0;
            bool timeoutOnTorque = rampElapsed > (COMPRESS_RAMP_TIMEOUT_MS * 1000ULL);

            if ((reachedTorqueTarget && rampElapsed > 500000) || stallDetected || timeoutOnTorque) {
                MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_COMPRESSION, "Compress Release");
                lastCommandTime = now;
                complete = true;
                return true;
            }
            return false;
        }

        return complete;
    }

    bool isComplete() { return complete && !isErrorFlag && !isTimeoutFlag; }
    bool hasError() { return isErrorFlag; }
    bool isTimeout() { return isTimeoutFlag; }

    void reset() {
        stateEntry = true;
        complete = false;
        isErrorFlag = false;
        isTimeoutFlag = false;
        pressureSensorChecked = false;
        torqueCommandSent = false;
    }
}
