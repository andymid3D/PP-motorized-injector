#ifndef BROADCAST_DATA_STORE_H
#define BROADCAST_DATA_STORE_H

#include <Arduino.h>
#include <SafeString.h>

/**
 * BroadcastDataStore - Central Repository for ODrive Broadcast Data
 * 
 * Purpose:
 *   - Cache all cyclic broadcast data from ODrive (updated by CanBusHandler)
 *   - Provide single source of truth for motor state, position, velocity, etc.
 *   - Eliminate repeated CAN queries (expensive on a tight loop)
 *   - Enable non-blocking decision logic (check state/velocity, don't wait)
 * 
 * Usage:
 *   1. Call update() whenever new cyclic broadcast is received from ODrive
 *   2. Query state/position/velocity/current as needed in loop()
 *   3. Use for timeout logic: check velocity < threshold instead of delay()
 * 
 * NonBlocking Design:
 *   - All updates append to SafeString for serial output buffering
 *   - State machines read current values, make decisions, return (non-blocking)
 *   - millisDelay or similar used for timing, not busy-wait loops
 */

class BroadcastDataStore {
public:
    // ===== AXIS STATE DATA =====
    struct AxisData {
        uint8_t state;           // Current axis state (0=UNDEFINED, 1=IDLE, 7=ENCODER_OFFSET, 8=CLOSED_LOOP, etc)
        float positionTurns;     // Encoder position in turns
        float velocityTurnsPerSec;  // Velocity in turns/second
        uint32_t lastUpdateMs;   // Timestamp of last broadcast (millis())
    };
    
    // ===== CURRENT & VOLTAGE DATA =====
    struct PowerData {
        float iqS;              // Setpoint current (A)
        float iqM;              // Measured current (A)
        float busCurrent;       // Total bus current (A)
        float busVoltage;       // Bus voltage (V)
        uint32_t lastUpdateMs;
    };
    
    // ===== SENSOR DATA (from various sources) =====
    struct SensorData {
        float temperature;      // °C (from thermistor)
        float pressureTorque;   // Nm (from load cell)
        bool topEndstop;        // Top position sensor
        bool bottomEndstop;     // Bottom position sensor
        bool barrelEndstop;     // Barrel position sensor
    };
    
    // ===== ERROR TRACKING (10ms broadcast interval) =====
    struct ErrorData {
        uint32_t axisError;           // Axis state error flags (from heartbeat)
        uint32_t motorError;          // Motor error flags (CYCLIC_MOTOR_ERROR 0x03)
        uint32_t encoderError;        // Encoder error flags (CYCLIC_ENCODER_ERROR 0x04)
        uint32_t controllerError;     // Controller error flags (CYCLIC_CONTROLLER_ERROR 0x1D)
        uint32_t lastUpdateMs;        // Timestamp of last error broadcast
        bool hasAnyError;             // Quick check: any error != 0
    };
    
    // ===== ENCODER ESTIMATES DATA (10ms broadcast interval) =====
    // Encoder estimates: position and velocity in turns (float, with decimals)
    // Based on CYCLIC_ENCODER_ESTIMATES (0x09) - full resolution position/velocity
    // CPR = 8192 counts per turn: counts = turns * 8192
    struct EncoderEstimatesData {
        float position;               // Position in turns (CYCLIC_ENCODER_ESTIMATES 0x09)
        float velocity;               // Velocity in turns/second
        uint32_t lastUpdateMs;
    };
    
    // ===== SINGLETON PATTERN =====
    static BroadcastDataStore& getInstance();
    
    /**
     * Check if broadcast data is stale (no updates in 50ms)
     * Indicates CAN bus issue, ODrive crash, or disconnection
     * @return true if no broadcast received in 50ms
     */
    bool isBroadcastDataStale() const;
    
    // ===== UPDATE METHODS (called when new broadcasts arrive) =====
    // These are called from CanBusHandlerV2 when cyclic messages arrive
    // Append to SafeString for non-blocking serial logging
    void updateAxisData(uint8_t state, float pos, float vel, SafeString& debugLog);
    void updateAxisState(uint8_t state);  // Quick update from heartbeat (non-blocking)
    void updatePowerData(float iqS, float iqM, float busCurrent, float busVoltage, SafeString& debugLog);
    void updateSensorData(float temp, float pressure, bool top, bool bot, bool barrel);
    
    // Error updates (from error cyclic messages at 10ms intervals)
    void updateAxisError(uint32_t axisError);
    void updateMotorError(uint32_t motorError, SafeString* debugLog = nullptr);
    void updateEncoderError(uint32_t encoderError, SafeString* debugLog = nullptr);
    void updateControllerError(uint32_t controllerError, SafeString* debugLog = nullptr);
    
    // Encoder estimates (position/velocity in turns from CYCLIC_ENCODER_ESTIMATES 0x09)
    void updateEncoderEstimates(float position, float velocity);
    
    // ===== QUERY METHODS (read current cached values) =====
    // Non-blocking - just return last known value
    uint8_t getAxisState() const;
    float getPosition() const;           // Returns turns
    float getVelocity() const;           // Returns turns/second
    uint32_t getLastAxisUpdate() const;  // Returns millis() of last update
    
    float getIqSetpoint() const;         // Setpoint current (A)
    float getIqMeasured() const;         // Measured current (A)
    float getBusCurrent() const;         // Bus current (A)
    float getBusVoltage() const;         // Bus voltage (V)
    uint32_t getLastPowerUpdate() const;
    
    float getTemperature() const;
    float getPressure() const;
    bool isTopEndstopActive() const;
    bool isBottomEndstopActive() const;
    bool isBarrelEndstopActive() const;
    
    // Error queries
    uint32_t getAxisError() const;
    uint32_t getMotorError() const;
    uint32_t getEncoderError() const;
    uint32_t getControllerError() const;
    bool hasAnyError() const;
    uint32_t getLastErrorUpdate() const;
    
    // Encoder estimates queries (position/velocity in turns)
    float getEncoderPosition() const;
    float getEncoderVelocity() const;
    
    // ===== TIME-BASED QUERIES (for timeout logic) =====
    // Non-blocking: check if broadcast data is stale
    bool isAxisDataStale(uint32_t maxAgeMs = 500) const;
    bool isPowerDataStale(uint32_t maxAgeMs = 500) const;
    uint32_t getAxisDataAgeMsecs() const;
    uint32_t getPowerDataAgeMsecs() const;
    
    // ===== VELOCITY THRESHOLD CHECKS (for state machine logic) =====
    // These enable non-blocking decision logic without arbitrary timeouts
    bool isVelocityBelowThreshold(float thresholdTurnsPerSec) const;
    bool isMoving(float threshold = 0.05f) const;  // Default: > 0.05 turns/sec = moving
    
    // ===== DEBUG / MONITORING =====
    void printStatus(SafeString& output);
    void clearErrorFlags();
    void resetAllData();
    
    // ===== ACTIVE BROADCAST MESSAGES LOG =====
    // Documents which cyclic messages are active and their broadcast intervals
    static constexpr const char* ACTIVE_BROADCASTS = 
        "CYCLIC_HEARTBEAT (0x01, 100ms): axis_error, axis_state, flags\n"
        "CYCLIC_ENCODER_ESTIMATES (0x09, 10ms): position, velocity\n"
        "CYCLIC_IQ (0x14, 100ms): Iq_setpoint, Iq_measured\n"
        "CYCLIC_BUS_VI (0x17, 100ms): bus_voltage, bus_current\n"
        "CYCLIC_MOTOR_ERROR (0x03, 10ms): motor_error flags\n"
        "CYCLIC_ENCODER_ERROR (0x04, 10ms): encoder_error flags\n"
        "CYCLIC_CONTROLLER_ERROR (0x1D, 10ms): controller_error flags";

private:
    // Private constructor (singleton)
    BroadcastDataStore() : axis_{0, 0.0f, 0.0f, 0}, power_{0, 0, 0, 0, 0}, 
                          errors_{0, 0, 0, 0, 0, false}, estimates_{0.0f, 0.0f, 0} {}
    BroadcastDataStore(const BroadcastDataStore&) = delete;  // No copies
    BroadcastDataStore& operator=(const BroadcastDataStore&) = delete;  // No assignment
    
    AxisData axis_;
    PowerData power_;
    SensorData sensors_;
    ErrorData errors_;
    EncoderEstimatesData estimates_;
};

#endif // BROADCAST_DATA_STORE_H
