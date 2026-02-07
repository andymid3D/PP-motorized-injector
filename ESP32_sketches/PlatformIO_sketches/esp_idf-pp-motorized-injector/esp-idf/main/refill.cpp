#include "refill.h"
#include "config.h"
#include "motor_wrapper.h"
#include "time_utils.h"
#include "broadcast_data_store.h"
#include "injector_fsm.h"
#include <cmath>

extern commonInjectParams_t commonParams;

namespace Refill {
    static enum { MOVING_TO_HOME, WAIT_ARRIVE, DONE } step = DONE;
    static uint64_t stepTimer = 0;
    static bool stateEntry = false;
    static bool complete = false;
    static bool error = false;

    static uint8_t lastHeartbeatState = 255;
    static uint64_t lastHeartbeatTimestamp = 0;
    static bool newTrajectoryStarted = false;
    static bool movementStarted = false;

    void begin() {
        step = MOVING_TO_HOME;
        stepTimer = 0;
        stateEntry = true;
        complete = false;
        error = false;
        lastHeartbeatState = 255;
        lastHeartbeatTimestamp = 0;
        newTrajectoryStarted = false;
        movementStarted = false;
    }

    bool update(CanBus& motor) {
        BroadcastDataStore& bds = BroadcastDataStore::getInstance();
        if (step == MOVING_TO_HOME) {
            if (stateEntry) {
                if (!MotorWrapper::canStartMove()) {
                    return false;
                }
                MotorWrapper::setMotorLimits(motor, REFILL_CONTROLLER_VEL_LIMIT, REFILL_CURRENT_LIMIT, MODULE_REFILL, "Refill");
                MotorWrapper::setTrapTrajParams(motor, commonParams.refillTrapVelLimit,
                                                commonParams.refillAccel, commonParams.refillDecel,
                                                MODULE_REFILL, "Refill Traj");
                if (!MotorWrapper::setModeAndMove(motor, 3, 5, OFFSET_REFILL_GAP, MODULE_REFILL, "Pos Refill")) {
                    return false;
                }
                stepTimer = time_utils::micros();
                stateEntry = false;
            }

            const TimestampedHeartbeat* latestHb = bds.getLatestHeartbeat();
            if (latestHb) {
                uint8_t currentHeartbeatState = latestHb->axisState;
                uint64_t currentHeartbeatTimestamp = latestHb->timestamp;
                if (currentHeartbeatState != lastHeartbeatState) {
                    lastHeartbeatState = currentHeartbeatState;
                    lastHeartbeatTimestamp = currentHeartbeatTimestamp;
                    if (!newTrajectoryStarted) {
                        newTrajectoryStarted = true;
                    }
                }
            }

            bool trajectoryComplete = bds.isTrajectoryComplete();
            bool motorStopped = std::fabs(bds.getVelocity()) < 0.1f;
            bool motorActuallyMoving = std::fabs(bds.getVelocity()) > 1.0f;

            if (motorActuallyMoving) movementStarted = true;

            bool freshTrajectoryComplete = newTrajectoryStarted && trajectoryComplete;
            if (movementStarted && freshTrajectoryComplete && motorStopped) {
                step = WAIT_ARRIVE;
                stepTimer = time_utils::micros();
                movementStarted = false;
            }

            if (stepTimer == 0) {
                return false;
            }
            uint64_t moveElapsed = time_utils::micros() - stepTimer;
            if (!stateEntry && moveElapsed > (REFILL_FROM_BARREL_END_TIMEOUT_MS * 1000ULL)) {
                error = true;
                complete = true;
                return true;
            }
            return false;
        }

        if (step == WAIT_ARRIVE) {
            return true;
        }

        return complete;
    }

    bool isComplete() { return complete && !error; }
    bool hasError() { return error; }

    void reset() {
        step = DONE;
        complete = false;
        error = false;
        stateEntry = true;
    }
}
