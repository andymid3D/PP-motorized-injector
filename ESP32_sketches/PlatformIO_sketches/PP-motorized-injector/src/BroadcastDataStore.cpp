#include "BroadcastDataStore.h"
#include "config.h"
#include "GPTimer.h"

BroadcastDataStore& BroadcastDataStore::getInstance() {
    // Meyer's singleton - thread-safe, lazy initialization
    static BroadcastDataStore instance;
    return instance;
}

// =============================================================================
// BDS V2: RING BUFFER STORAGE (Phase 1.7)
// =============================================================================

void BroadcastDataStore::storeHeartbeat(uint32_t axisError, uint8_t axisState, uint8_t procedureResult, uint64_t timestamp, bool isResponse) {
    TimestampedHeartbeat msg = {axisError, axisState, procedureResult, timestamp, isResponse};
    heartbeatHistory_.push(msg);
    
    // Update v1 storage for backward compatibility
    axis_.state = axisState;
    axis_.lastUpdateMs = millis();
    errors_.axisError = axisError;
}

void BroadcastDataStore::storeEncoder(float position, float velocity, uint64_t timestamp, bool isResponse) {
    TimestampedEncoder msg = {position, velocity, timestamp, isResponse};
    encoderHistory_.push(msg);
    
    // Update v1 storage for backward compatibility
    estimates_.position = position;
    estimates_.velocity = velocity;
    estimates_.lastUpdateMs = millis();
    axis_.positionTurns = position;
    axis_.velocityTurnsPerSec = velocity;
}

void BroadcastDataStore::storeIq(float iqSetpoint, float iqMeasured, uint64_t timestamp, bool isResponse) {
    TimestampedIq msg = {iqSetpoint, iqMeasured, timestamp, isResponse};
    iqHistory_.push(msg);
    
    // Update v1 storage for backward compatibility
    power_.iqS = iqSetpoint;
    power_.iqM = iqMeasured;
    power_.lastUpdateMs = millis();
}

void BroadcastDataStore::storeBusVI(float busVoltage, float busCurrent, uint64_t timestamp, bool isResponse) {
    TimestampedBusVI msg = {busVoltage, busCurrent, timestamp, isResponse};
    busVIHistory_.push(msg);
    
    // Update v1 storage for backward compatibility
    power_.busVoltage = busVoltage;
    power_.busCurrent = busCurrent;
}

void BroadcastDataStore::storeMotorError(uint32_t motorError, uint64_t timestamp, bool isResponse) {
    TimestampedMotorError msg = {motorError, timestamp, isResponse};
    motorErrorHistory_.push(msg);
    
    // Update v1 storage for backward compatibility
    errors_.motorError = motorError;
    errors_.lastUpdateMs = millis();
    errors_.hasAnyError = (motorError != 0);
}

void BroadcastDataStore::storeEncoderError(uint32_t encoderError, uint64_t timestamp, bool isResponse) {
    TimestampedEncoderError msg = {encoderError, timestamp, isResponse};
    encoderErrorHistory_.push(msg);
    
    // Update v1 storage for backward compatibility
    errors_.encoderError = encoderError;
    errors_.hasAnyError = (encoderError != 0);
}

void BroadcastDataStore::storeControllerError(uint32_t controllerError, uint64_t timestamp, bool isResponse) {
    TimestampedControllerError msg = {controllerError, timestamp, isResponse};
    controllerErrorHistory_.push(msg);
    
    // Update v1 storage for backward compatibility
    errors_.controllerError = controllerError;
    errors_.hasAnyError = (controllerError != 0);
}

// =============================================================================
// BDS V2: HISTORICAL DATA ACCESS
// =============================================================================

const TimestampedHeartbeat* BroadcastDataStore::getLatestHeartbeat() const {
    return heartbeatHistory_.getLatest();
}

const TimestampedEncoder* BroadcastDataStore::getLatestEncoder() const {
    return encoderHistory_.getLatest();
}

const TimestampedIq* BroadcastDataStore::getLatestIq() const {
    return iqHistory_.getLatest();
}

const TimestampedIq* BroadcastDataStore::getHistoryIq(size_t index) const {
    return iqHistory_.getHistory(index);
}

const TimestampedEncoder* BroadcastDataStore::getHistoryEncoder(size_t index) const {
    return encoderHistory_.getHistory(index);
}

const TimestampedBusVI* BroadcastDataStore::getLatestBusVI() const {
    return busVIHistory_.getLatest();
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

// =============================================================================
// BDS V2: STALENESS DETECTION (microsecond resolution)
// =============================================================================

bool BroadcastDataStore::isEncoderStale(uint64_t maxAgeMicros) const {
    const TimestampedEncoder* latest = encoderHistory_.getLatest();
    if (latest == nullptr) return true;  // No data yet = stale
    
    extern GPTimer hwTimer;
    uint64_t age = hwTimer.micros() - latest->timestamp;
    return age > maxAgeMicros;
}

bool BroadcastDataStore::isHeartbeatStale(uint64_t maxAgeMicros) const {
    const TimestampedHeartbeat* latest = heartbeatHistory_.getLatest();
    if (latest == nullptr) return true;
    
    extern GPTimer hwTimer;
    uint64_t age = hwTimer.micros() - latest->timestamp;
    return age > maxAgeMicros;
}

bool BroadcastDataStore::isIqStale(uint64_t maxAgeMicros) const {
    const TimestampedIq* latest = iqHistory_.getLatest();
    if (latest == nullptr) return true;
    
    extern GPTimer hwTimer;
    uint64_t age = hwTimer.micros() - latest->timestamp;
    return age > maxAgeMicros;
}

uint64_t BroadcastDataStore::getEncoderAgeMicros(uint64_t currentTime) const {
    const TimestampedEncoder* latest = encoderHistory_.getLatest();
    if (latest == nullptr) return UINT64_MAX;  // No data = infinite age
    return currentTime - latest->timestamp;
}

uint64_t BroadcastDataStore::getHeartbeatAgeMicros(uint64_t currentTime) const {
    const TimestampedHeartbeat* latest = heartbeatHistory_.getLatest();
    if (latest == nullptr) return UINT64_MAX;
    return currentTime - latest->timestamp;
}

// =============================================================================
// V1 COMPATIBILITY: STALENESS CHECK (millisecond resolution - legacy)
// =============================================================================
bool BroadcastDataStore::isBroadcastDataStale() const {
    // Check encoder estimates (most frequent broadcast, ~10ms)
    // 100ms timeout = matches heartbeat interval
    // If heartbeat arrives with axis error → existing error handling catches it
    // If NO heartbeat at all → CAN disconnect/ODrive crash/ESP32 CAN failure
    uint32_t now = millis();
    return (now - estimates_.lastUpdateMs) > BROADCAST_STALE_TIMEOUT_MS;
}

// ===== UPDATE METHODS =====
void BroadcastDataStore::updateAxisData(uint8_t state, float pos, float vel, SafeString& debugLog) {
    axis_.state = state;
    axis_.positionTurns = pos;
    axis_.velocityTurnsPerSec = vel;
    axis_.lastUpdateMs = millis();
    
    // Append to debug log (non-blocking serial buffering)
    debugLog += " AxisState:";
    debugLog += state;
    debugLog += " Pos:";
    debugLog += pos;
    debugLog += " Vel:";
    debugLog += vel;
}

void BroadcastDataStore::updatePowerData(float iqS, float iqM, float busCurrent, float busVoltage, SafeString& debugLog) {
    power_.iqS = iqS;
    power_.iqM = iqM;
    power_.busCurrent = busCurrent;
    power_.busVoltage = busVoltage;
    power_.lastUpdateMs = millis();
    
    // Append to debug log
    debugLog += " IqS:";
    debugLog += iqS;
    debugLog += " IqM:";
    debugLog += iqM;
    debugLog += " BusI:";
    debugLog += busCurrent;
    debugLog += " BusV:";
    debugLog += busVoltage;
}

void BroadcastDataStore::updateSensorData(float temp, float pressure, bool top, bool bot, bool barrel) {
    sensors_.temperature = temp;
    sensors_.pressureTorque = pressure;
    sensors_.topEndstop = top;
    sensors_.bottomEndstop = bot;
    sensors_.barrelEndstop = barrel;
}

void BroadcastDataStore::updateAxisState(uint8_t state) {
    axis_.state = state;
    axis_.lastUpdateMs = millis();
}

// ===== ERROR UPDATE METHODS =====
void BroadcastDataStore::updateAxisError(uint32_t axisError) {
    errors_.axisError = axisError;
    errors_.lastUpdateMs = millis();
}

void BroadcastDataStore::updateMotorError(uint32_t motorError, SafeString* debugLog) {
    errors_.motorError = motorError;
    errors_.lastUpdateMs = millis();
    errors_.hasAnyError = (motorError != 0);
    if (motorError != 0 && debugLog != nullptr) {
        *debugLog += " MotorErr:0x";
        char hex[16];
        snprintf(hex, sizeof(hex), "%08lX", motorError);
        *debugLog += hex;
    }
}

void BroadcastDataStore::updateEncoderError(uint32_t encoderError, SafeString* debugLog) {
    errors_.encoderError = encoderError;
    errors_.lastUpdateMs = millis();
    errors_.hasAnyError = (encoderError != 0);
    if (encoderError != 0 && debugLog != nullptr) {
        *debugLog += " EncoderErr:0x";
        char hex[16];
        snprintf(hex, sizeof(hex), "%08lX", encoderError);
        *debugLog += hex;
    }
}

void BroadcastDataStore::updateControllerError(uint32_t controllerError, SafeString* debugLog) {
    errors_.controllerError = controllerError;
    errors_.lastUpdateMs = millis();
    errors_.hasAnyError = (controllerError != 0);
    if (controllerError != 0 && debugLog != nullptr) {
        *debugLog += " CtrlErr:0x";
        char hex[16];
        snprintf(hex, sizeof(hex), "%08lX", controllerError);
        *debugLog += hex;
    }
}

// ===== ENCODER ESTIMATES UPDATE =====
void BroadcastDataStore::updateEncoderEstimates(float position, float velocity) {
    estimates_.position = position;
    estimates_.velocity = velocity;
    estimates_.lastUpdateMs = millis();
}

// ===== QUERY METHODS =====
uint8_t BroadcastDataStore::getAxisState() const {
    return axis_.state;
}

float BroadcastDataStore::getPosition() const {
    return estimates_.position;
}

float BroadcastDataStore::getVelocity() const {
    return estimates_.velocity;
}

uint32_t BroadcastDataStore::getLastAxisUpdate() const {
    return axis_.lastUpdateMs;
}

float BroadcastDataStore::getIqSetpoint() const {
    return power_.iqS;
}

float BroadcastDataStore::getIqMeasured() const {
    return power_.iqM;
}

float BroadcastDataStore::getBusCurrent() const {
    return power_.busCurrent;
}

float BroadcastDataStore::getBusVoltage() const {
    return power_.busVoltage;
}

uint32_t BroadcastDataStore::getLastPowerUpdate() const {
    return power_.lastUpdateMs;
}

float BroadcastDataStore::getTemperature() const {
    return sensors_.temperature;
}

float BroadcastDataStore::getPressure() const {
    return sensors_.pressureTorque;
}

bool BroadcastDataStore::isTopEndstopActive() const {
    return sensors_.topEndstop;
}

bool BroadcastDataStore::isBottomEndstopActive() const {
    return sensors_.bottomEndstop;
}

bool BroadcastDataStore::isBarrelEndstopActive() const {
    return sensors_.barrelEndstop;
}

uint32_t BroadcastDataStore::getAxisError() const {
    return errors_.axisError;
}

uint32_t BroadcastDataStore::getMotorError() const {
    return errors_.motorError;
}

uint32_t BroadcastDataStore::getEncoderError() const {
    return errors_.encoderError;
}

uint32_t BroadcastDataStore::getControllerError() const {
    return errors_.controllerError;
}

bool BroadcastDataStore::hasAnyError() const {
    return errors_.hasAnyError;
}

uint32_t BroadcastDataStore::getLastErrorUpdate() const {
    return errors_.lastUpdateMs;
}

float BroadcastDataStore::getEncoderPosition() const {
    return estimates_.position;
}

float BroadcastDataStore::getEncoderVelocity() const {
    return estimates_.velocity;
}

// ===== TIME-BASED QUERIES =====
bool BroadcastDataStore::isAxisDataStale(uint32_t maxAgeMs) const {
    uint32_t age = millis() - axis_.lastUpdateMs;
    return age > maxAgeMs;
}

bool BroadcastDataStore::isPowerDataStale(uint32_t maxAgeMs) const {
    uint32_t age = millis() - power_.lastUpdateMs;
    return age > maxAgeMs;
}

uint32_t BroadcastDataStore::getAxisDataAgeMsecs() const {
    return millis() - axis_.lastUpdateMs;
}

uint32_t BroadcastDataStore::getPowerDataAgeMsecs() const {
    return millis() - power_.lastUpdateMs;
}

// ===== VELOCITY THRESHOLD CHECKS =====
bool BroadcastDataStore::isVelocityBelowThreshold(float thresholdTurnsPerSec) const {
    return fabs(axis_.velocityTurnsPerSec) < thresholdTurnsPerSec;
}

bool BroadcastDataStore::isMoving(float threshold) const {
    return fabs(axis_.velocityTurnsPerSec) > threshold;
}

// ===== DEBUG / MONITORING =====
void BroadcastDataStore::printStatus(SafeString& output) {
    output.clear();
    output += "State:";
    output += axis_.state;
    output += " Pos:";
    output += axis_.positionTurns;
    output += " Vel:";
    output += axis_.velocityTurnsPerSec;
    output += " Temp:";
    output += sensors_.temperature;
    output += " IqM:";
    output += power_.iqM;
    output += " BusV:";
    output += power_.busVoltage;
}

void BroadcastDataStore::clearErrorFlags() {
    errors_.axisError = 0;
    errors_.motorError = 0;
    errors_.encoderError = 0;
    errors_.controllerError = 0;
    errors_.hasAnyError = false;
}

void BroadcastDataStore::resetAllData() {
    axis_ = {0, 0.0f, 0.0f, 0};
    power_ = {0, 0, 0, 0, 0};
    sensors_ = {};
    errors_ = {0, 0, 0, 0, 0, false};
    estimates_ = {0.0f, 0.0f, 0};
}

// =============================================================================
// TIMING SYSTEM ACCESSORS (NEW)
// =============================================================================

uint64_t BroadcastDataStore::getEncoderTimestamp() const {
    const TimestampedEncoder* latest = getLatestEncoder();
    return latest ? latest->timestamp : 0;
}

uint64_t BroadcastDataStore::getIqTimestamp() const {
    const TimestampedIq* latest = getLatestIq();
    return latest ? latest->timestamp : 0;
}

uint32_t BroadcastDataStore::getTimingOffset(uint64_t currentTime) const {
    uint64_t encoderTime = getEncoderTimestamp();
    if (encoderTime == 0) return UINT32_MAX; // No encoder data
    return (currentTime > encoderTime) ? (uint32_t)(currentTime - encoderTime) : 0;
}

bool BroadcastDataStore::isInCommandWindow(uint64_t currentTime, uint32_t offsetUs, uint32_t durationUs) const {
    uint32_t offset = getTimingOffset(currentTime);
    if (offset == UINT32_MAX) return false; // No encoder data
    
    return (offset >= offsetUs) && (offset < (offsetUs + durationUs));
}

bool BroadcastDataStore::getMovementData(float& iqSetpoint, float& iqMeasured, float& position, float& velocity) const {
    const TimestampedIq* iqData = getLatestIq();
    const TimestampedEncoder* encData = getLatestEncoder();
    
    if (!iqData || !encData) return false;
    
    // Check if data is fresh (within 50ms)
    uint64_t currentTime = hwTimer.micros();
    if ((currentTime - iqData->timestamp) > 50000 || (currentTime - encData->timestamp) > 50000) {
        return false;
    }
    
    iqSetpoint = iqData->iqSetpoint;
    iqMeasured = iqData->iqMeasured;
    position = encData->position;
    velocity = encData->velocity;
    
    return true;
}

bool BroadcastDataStore::hasMovementOccurred(float baselinePos, float baselineIq, float posThreshold, float iqThreshold) const {
    float iqSetpoint, iqMeasured, position, velocity;
    if (!getMovementData(iqSetpoint, iqMeasured, position, velocity)) return false;
    
    // Check position change
    float posDelta = fabs(position - baselinePos);
    if (posDelta >= posThreshold) return true;
    
    // Check IQ current change
    float iqDelta = fabs(iqMeasured - baselineIq);
    if (iqDelta >= iqThreshold) return true;
    
    return false;
}

bool BroadcastDataStore::isDataFresh(uint64_t maxAgeUs) const {
    uint64_t currentTime = hwTimer.micros();
    
    const TimestampedEncoder* encData = getLatestEncoder();
    const TimestampedIq* iqData = getLatestIq();
    
    if (!encData || !iqData) return false;
    
    return ((currentTime - encData->timestamp) <= maxAgeUs) && 
           ((currentTime - iqData->timestamp) <= maxAgeUs);
}
