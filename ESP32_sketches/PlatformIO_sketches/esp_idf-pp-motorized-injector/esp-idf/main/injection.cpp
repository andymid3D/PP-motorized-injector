#include "injection.h"
#include "config.h"
#include "motor_wrapper.h"
#include "time_utils.h"
#include "broadcast_data_store.h"
#include "purge_zero.h"
#include <cmath>

extern commonInjectParams_t commonParams;

namespace Injection {
    static InjectionPhase phase = DONE;
    static bool stateEntry = false;
    static uint64_t stateEnterTime = 0;
    static uint64_t phaseStartTime = 0;
    static bool complete = false;
    static bool error = false;
    static uint64_t lastCommandTime = 0;
    static bool pressureSensorChecked = false;

    static actualMouldParams_t currentMould;

    static float injectStartPos = 0.0f;
    static float packStartPos = 0.0f;
    static float targetInjectPos = 0.0f;
    static float targetPackPos = 0.0f;

    static float volToTurns(float cm3) {
        return cm3 * TURNS_PER_CM3_VOL;
    }

    static float turnsToVol(float turns) {
        if (TURNS_PER_CM3_VOL == 0.0f) return 0.0f;
        return turns / TURNS_PER_CM3_VOL;
    }

    void begin(const actualMouldParams_t& mouldParams) {
        currentMould = mouldParams;
        phase = FILLING;
        stateEntry = true;
        stateEnterTime = 0;
        phaseStartTime = 0;
        complete = false;
        error = false;
        lastCommandTime = time_utils::micros();
        pressureSensorChecked = false;
        injectStartPos = 0.0f;
        packStartPos = 0.0f;
    }

    bool update(CanBus& motor) {
        uint64_t now = time_utils::micros();
        uint64_t stateElapsed = (now - stateEnterTime) / 1000ULL;
        uint64_t phaseElapsed = (now - phaseStartTime) / 1000ULL;

        if (stateEntry) {
            if (!MotorWrapper::canStartMove()) {
                return false;
            }
            pressureSensorChecked = true;

            injectStartPos = PurgeZero::getPurgeZeroPosition();
            targetInjectPos = injectStartPos + volToTurns(currentMould.fillVolume);
            if (targetInjectPos > POS_BOTTOM_MAX) targetInjectPos = POS_BOTTOM_MAX;

            MotorWrapper::setMotorLimits(motor, INJECT_FILL_CONTROLLER_VEL_LIMIT, INJECT_FILL_CURRENT, MODULE_INJECT, "Inject Fill");
            MotorWrapper::setTrapTrajParams(motor, commonParams.injectFillTrapVelLimit,
                                           commonParams.injectFillAccel,
                                           commonParams.injectFillDecel,
                                           MODULE_INJECT, "Fill Traj");
            if (!MotorWrapper::setModeAndMove(motor, 3, 5, targetInjectPos, MODULE_INJECT, "Pos Inject")) {
                return false;
            }
            lastCommandTime = now;
            stateEnterTime = now;
            phaseStartTime = now;
            stateEntry = false;
            return false;
        }

        if (phase == FILLING) {
            BroadcastDataStore& bds = BroadcastDataStore::getInstance();
            bool velocityLow = std::fabs(bds.getVelocity()) < 0.1f;
            bool timingStable = phaseElapsed > 500;
            bool positionClose = std::fabs(bds.getPosition() - targetInjectPos) < 1.0f;

            if (velocityLow && timingStable && positionClose) {
                phase = PACKING;
                phaseStartTime = now;
                packStartPos = bds.getPosition();
                targetPackPos = packStartPos + volToTurns(currentMould.packVolume);
                if (targetPackPos > POS_BOTTOM_MAX) targetPackPos = POS_BOTTOM_MAX;

                MotorWrapper::setMotorLimits(motor, INJECT_PACK_CONTROLLER_VEL_LIMIT, INJECT_PACK_CURRENT, MODULE_INJECT, "Inject Pack");
                MotorWrapper::setTrapTrajParams(motor, commonParams.injectPackTrapVelLimit,
                                               commonParams.injectPackAccel,
                                               commonParams.injectPackDecel,
                                               MODULE_INJECT, "Pack Traj");
                MotorWrapper::setModeAndMove(motor, 3, 5, targetPackPos, MODULE_INJECT, "Pos Pack");
                lastCommandTime = now;
                return false;
            }

            if (phaseElapsed > INJECT_FILL_TIMEOUT_MS) {
                error = true;
                complete = true;
                MotorWrapper::setModeAndMove(motor, 2, 1, 0, MODULE_INJECT, "Inject Timeout Stop");
                return true;
            }
            return false;
        }

        if (phase == PACKING) {
            uint64_t packTimeMs = static_cast<uint64_t>(currentMould.packTime * 1000.0f);
            if (phaseElapsed >= packTimeMs) {
                phase = DONE;
                complete = true;
                return true;
            }
            return false;
        }

        return complete;
    }

    bool isComplete() { return complete && !error; }
    bool hasError() { return error; }
    InjectionPhase getPhase() { return phase; }

    float getInjectStartPos() { return injectStartPos; }
    float getTargetInjectPos() { return targetInjectPos; }
    float getTargetPackPos() { return targetPackPos; }

    float getInjectVolumeFromPos(float currentPos) {
        return turnsToVol(currentPos - injectStartPos);
    }

    float getTargetInjectVolume() {
        return turnsToVol(targetInjectPos - injectStartPos);
    }

    void reset() {
        stateEntry = true;
        phase = DONE;
        complete = false;
        error = false;
        pressureSensorChecked = false;
    }
}
