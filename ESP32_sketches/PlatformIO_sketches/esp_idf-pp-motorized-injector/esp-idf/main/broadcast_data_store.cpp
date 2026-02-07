#include "broadcast_data_store.h"
#include "time_utils.h"
#include "config.h"
#include <cmath>

BroadcastDataStore& BroadcastDataStore::getInstance() {
    static BroadcastDataStore instance;
    return instance;
}

BroadcastDataStore::BroadcastDataStore()
    : dataMutex_(xSemaphoreCreateMutex()),
      axis_{0, 0.0f, 0.0f, 0},
      power_{0.0f, 0.0f, 0.0f, 0.0f, 0},
      sensors_{0.0f, 0.0f, false, false, false},
      errors_{0, 0, 0, 0, 0, false},
      estimates_{0.0f, 0.0f, 0},
      lastHeartbeatRead_(0) {
}

void BroadcastDataStore::storeHeartbeat(uint32_t axisError, uint8_t axisState, uint8_t motorErrorFlag,
                                        uint8_t encoderErrorFlag, uint8_t controllerErrorFlag,
                                        uint8_t trajectoryDoneFlag, uint64_t timestamp, bool isResponse) {
    TimestampedHeartbeat msg = {axisError, axisState, motorErrorFlag, encoderErrorFlag, controllerErrorFlag, trajectoryDoneFlag, timestamp, isResponse};
    heartbeatHistory_.push(msg);

    axis_.state = axisState;
    axis_.lastUpdateUs = time_utils::micros();

    errors_.axisError = axisError;
    errors_.motorError = motorErrorFlag ? 0xFFFFFFFFFFFFFFFFULL : 0;
    errors_.encoderError = encoderErrorFlag ? 0xFFFFFFFF : 0;
    errors_.controllerError = controllerErrorFlag ? 0xFFFFFFFF : 0;
    errors_.lastUpdateUs = time_utils::micros();
    errors_.hasAnyError = axisError != 0 || motorErrorFlag || encoderErrorFlag || controllerErrorFlag;
}

void BroadcastDataStore::storeEncoder(float position, float velocity, uint64_t timestamp, bool isResponse) {
    TimestampedEncoder msg = {position, velocity, timestamp, isResponse};
    encoderHistory_.push(msg);

    estimates_.position = position;
    estimates_.velocity = velocity;
    estimates_.lastUpdateUs = time_utils::micros();
    axis_.positionTurns = position;
    axis_.velocityTurnsPerSec = velocity;
}

void BroadcastDataStore::storeIq(float iqSetpoint, float iqMeasured, uint64_t timestamp, bool isResponse) {
    TimestampedIq msg = {iqSetpoint, iqMeasured, timestamp, isResponse};
    iqHistory_.push(msg);

    power_.iqS = iqSetpoint;
    power_.iqM = iqMeasured;
    power_.lastUpdateUs = time_utils::micros();
}

void BroadcastDataStore::storeMotorError(uint64_t motorError, uint64_t timestamp, bool isResponse) {
    TimestampedMotorError msg = {motorError, timestamp, isResponse};
    motorErrorHistory_.push(msg);

    if (dataMutex_ && xSemaphoreTake(dataMutex_, portMAX_DELAY) == pdTRUE) {
        errors_.motorError = motorError;
        errors_.lastUpdateUs = time_utils::micros();
        errors_.hasAnyError = (motorError != 0);
        xSemaphoreGive(dataMutex_);
    }
}

void BroadcastDataStore::storeEncoderError(uint32_t encoderError, uint64_t timestamp, bool isResponse) {
    TimestampedEncoderError msg = {encoderError, timestamp, isResponse};
    encoderErrorHistory_.push(msg);

    errors_.encoderError = encoderError;
    errors_.hasAnyError = (encoderError != 0);
}

void BroadcastDataStore::storeControllerError(uint32_t controllerError, uint64_t timestamp, bool isResponse) {
    TimestampedControllerError msg = {controllerError, timestamp, isResponse};
    controllerErrorHistory_.push(msg);

    if (dataMutex_ && xSemaphoreTake(dataMutex_, portMAX_DELAY) == pdTRUE) {
        errors_.controllerError = controllerError;
        errors_.lastUpdateUs = time_utils::micros();
        errors_.hasAnyError = (controllerError != 0);
        xSemaphoreGive(dataMutex_);
    }
}

const TimestampedHeartbeat* BroadcastDataStore::getLatestHeartbeat() const {
    uint64_t now = time_utils::micros();
    if (now - lastHeartbeatRead_ > FRESH_DATA_DELAY_US) {
        vTaskDelay(pdMS_TO_TICKS(15));
        lastHeartbeatRead_ = time_utils::micros();
    }
    return heartbeatHistory_.getLatest();
}

const TimestampedEncoder* BroadcastDataStore::getLatestEncoder() const {
    return encoderHistory_.getLatest();
}

const TimestampedIq* BroadcastDataStore::getLatestIq() const {
    return iqHistory_.getLatest();
}

const TimestampedMotorError* BroadcastDataStore::getLatestMotorError() const {
    return motorErrorHistory_.getLatest();
}

const TimestampedEncoderError* BroadcastDataStore::getLatestEncoderError() const {
    return encoderErrorHistory_.getLatest();
}

const TimestampedControllerError* BroadcastDataStore::getLatestControllerError() const {
    return controllerErrorHistory_.getLatest();
}

bool BroadcastDataStore::isEncoderStale(uint64_t maxAgeMicros) const {
    const TimestampedEncoder* latest = encoderHistory_.getLatest();
    if (!latest) return true;
    uint64_t age = time_utils::micros() - latest->timestamp;
    return age > maxAgeMicros;
}

bool BroadcastDataStore::isHeartbeatStale(uint64_t maxAgeMicros) const {
    const TimestampedHeartbeat* latest = heartbeatHistory_.getLatest();
    if (!latest) return true;
    uint64_t age = time_utils::micros() - latest->timestamp;
    return age > maxAgeMicros;
}

bool BroadcastDataStore::isIqStale(uint64_t maxAgeMicros) const {
    const TimestampedIq* latest = iqHistory_.getLatest();
    if (!latest) return true;
    uint64_t age = time_utils::micros() - latest->timestamp;
    return age > maxAgeMicros;
}

uint64_t BroadcastDataStore::getEncoderAgeMicros(uint64_t currentTime) const {
    const TimestampedEncoder* latest = encoderHistory_.getLatest();
    if (!latest) return UINT64_MAX;
    return currentTime - latest->timestamp;
}

uint64_t BroadcastDataStore::getHeartbeatAgeMicros(uint64_t currentTime) const {
    const TimestampedHeartbeat* latest = heartbeatHistory_.getLatest();
    if (!latest) return UINT64_MAX;
    return currentTime - latest->timestamp;
}

bool BroadcastDataStore::isBroadcastDataStale() const {
    if (estimates_.lastUpdateUs == 0) {
        return false;
    }
    uint64_t now = time_utils::micros();
    return (now - estimates_.lastUpdateUs) > (BROADCAST_STALE_TIMEOUT_MS * 1000ULL);
}

uint8_t BroadcastDataStore::getAxisState() const { return axis_.state; }
float BroadcastDataStore::getPosition() const { return estimates_.position; }
float BroadcastDataStore::getVelocity() const { return estimates_.velocity; }
uint64_t BroadcastDataStore::getLastAxisUpdate() const { return axis_.lastUpdateUs; }

float BroadcastDataStore::getIqSetpoint() const { return power_.iqS; }
float BroadcastDataStore::getIqMeasured() const { return power_.iqM; }
float BroadcastDataStore::getBusCurrent() const { return power_.busCurrent; }
float BroadcastDataStore::getBusVoltage() const { return power_.busVoltage; }
uint64_t BroadcastDataStore::getLastPowerUpdate() const { return power_.lastUpdateUs; }

float BroadcastDataStore::getTemperature() const { return sensors_.temperature; }
float BroadcastDataStore::getPressure() const { return sensors_.pressureTorque; }
bool BroadcastDataStore::isTopEndstopActive() const { return sensors_.topEndstop; }
bool BroadcastDataStore::isBottomEndstopActive() const { return sensors_.bottomEndstop; }
bool BroadcastDataStore::isBarrelEndstopActive() const { return sensors_.barrelEndstop; }

uint32_t BroadcastDataStore::getAxisError() const { return errors_.axisError; }
uint64_t BroadcastDataStore::getMotorError() const {
    uint64_t result = 0;
    if (dataMutex_ && xSemaphoreTake(dataMutex_, portMAX_DELAY) == pdTRUE) {
        result = errors_.motorError;
        xSemaphoreGive(dataMutex_);
    }
    return result;
}

uint32_t BroadcastDataStore::getEncoderError() const { return errors_.encoderError; }

uint32_t BroadcastDataStore::getControllerError() const {
    uint32_t result = 0;
    if (dataMutex_ && xSemaphoreTake(dataMutex_, portMAX_DELAY) == pdTRUE) {
        result = errors_.controllerError;
        xSemaphoreGive(dataMutex_);
    }
    return result;
}

bool BroadcastDataStore::hasAnyError() const { return errors_.hasAnyError; }
uint64_t BroadcastDataStore::getLastErrorUpdate() const { return errors_.lastUpdateUs; }

uint64_t BroadcastDataStore::getMotorErrorSafe() const {
    const TimestampedMotorError* previous = motorErrorHistory_.getHistory(1);
    return previous ? previous->motorError : 0;
}

uint32_t BroadcastDataStore::getControllerErrorSafe() const {
    const TimestampedControllerError* previous = controllerErrorHistory_.getHistory(1);
    return previous ? previous->controllerError : 0;
}

bool BroadcastDataStore::isTrajectoryComplete() const {
    const TimestampedHeartbeat* hb = getLatestHeartbeat();
    if (!hb) return false;
    return hb->trajectoryDoneFlag != 0;
}

bool BroadcastDataStore::isTrajectoryComplete(uint64_t& timestamp) const {
    const TimestampedHeartbeat* hb = getLatestHeartbeat();
    if (!hb) {
        timestamp = 0;
        return false;
    }
    timestamp = hb->timestamp;
    return hb->trajectoryDoneFlag != 0;
}

bool BroadcastDataStore::isVelocityBelowThreshold(float thresholdTurnsPerSec) const {
    return std::fabs(axis_.velocityTurnsPerSec) < thresholdTurnsPerSec;
}

bool BroadcastDataStore::isMoving(float threshold) const {
    return std::fabs(axis_.velocityTurnsPerSec) > threshold;
}
