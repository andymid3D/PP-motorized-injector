#include "BroadcastDataStore.h"

BroadcastDataStore& BroadcastDataStore::getInstance() {
    // Meyer's singleton - thread-safe, lazy initialization
    static BroadcastDataStore instance;
    return instance;
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
