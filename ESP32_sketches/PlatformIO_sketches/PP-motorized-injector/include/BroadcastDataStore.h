#ifndef BROADCAST_DATA_STORE_H
#define BROADCAST_DATA_STORE_H

#include <Arduino.h>
#include <SafeString.h>
#include "RingBuffer.h"
#include "GPTimer.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// =============================================================================
// RING BUFFER HISTORY SIZES (Phase 1.7 BDS v2)
// =============================================================================
// Size calculations based on broadcast intervals:
// - Encoder (10ms): 250 samples = 2.5s history
// - Heartbeat (100ms): 10 samples = 1000ms (1s) history
// - Iq/BusVI (10ms): 250 samples = 2.5s history (upgraded from 100ms)
// - Errors (10ms): 10 samples = 100ms history (recent errors only)
// =============================================================================
#define BDS_IQ_HISTORY_SIZE 1000        // 10s at 10ms, 100s at 100ms (continuous collection)
#define BDS_ENCODER_HISTORY_SIZE 1000   // 10s at 10ms, 100s at 100ms (continuous collection)
#define BDS_MOTOR_ERROR_HISTORY_SIZE  250
#define BDS_ENCODER_ERROR_HISTORY_SIZE 250
#define BDS_CONTROLLER_ERROR_HISTORY_SIZE 250
#define BDS_HEARTBEAT_HISTORY_SIZE    10
#define BDS_BUSVI_HISTORY_SIZE        10

// =============================================================================
// TIMESTAMPED MESSAGE STRUCTS (Phase 1.7 BDS v2)
// =============================================================================
// These structs store ODrive broadcast data with high-resolution timestamps
// for staleness detection and TX response correlation.
//
// Design Notes:
// - timestamp: 64-bit microseconds from GPTimer (1µs resolution)
// - isResponse: true if message correlates to recent TX command (for filtering)
// - Backward compatible: Keep existing non-timestamped structs for now
// - Ring buffer storage: RingBuffer<TimestampedXXX, HISTORY_SIZE>
// =============================================================================

/**
 * Heartbeat (CAN 0x001, 100ms interval)
 * Contains axis error, axis state, and all error flags
 */
struct TimestampedHeartbeat {
    uint32_t axisError;             // Axis error flags (bytes 0-3)
    uint8_t axisState;              // Current state (byte 4: IDLE=1, CLOSED_LOOP=8, etc)
    uint8_t motorErrorFlag;         // Motor error flag (byte 5: 0=OK, 1=error present)
    uint8_t encoderErrorFlag;       // Encoder error flag (byte 6: 0=OK, 1=error present)
    uint8_t controllerErrorFlag;    // Controller error flag (byte 7 bits 0-6: 0=OK, 1=error present)
    uint8_t trajectoryDoneFlag;     // Trajectory completion flag (byte 7 bit 7: 0=in progress, 1=done)
    uint64_t timestamp;             // GPTimer microseconds
    bool isResponse;                // True if correlates to recent TX command
};

/**
 * Encoder Estimates (CAN 0x009, 10ms interval)
 * High-frequency position/velocity data
 */
struct TimestampedEncoder {
    float position;             // Position in turns
    float velocity;             // Velocity in turns/second
    uint64_t timestamp;         // GPTimer microseconds
    bool isResponse;            // True if correlates to recent TX command
};

/**
 * Iq Readings (CAN 0x014, 100ms interval)
 * Current setpoint and measured values
 */
struct TimestampedIq {
    float iqSetpoint;           // Setpoint current (A)
    float iqMeasured;           // Measured current (A)
    uint64_t timestamp;         // GPTimer microseconds
    bool isResponse;            // True if correlates to recent TX command
};

/**
 * Bus Voltage/Current (CAN 0x017, 100ms interval)
 * Power supply monitoring
 */
struct TimestampedBusVI {
    float busVoltage;           // Bus voltage (V)
    float busCurrent;           // Bus current (A)
    uint64_t timestamp;         // GPTimer microseconds
    bool isResponse;            // True if correlates to recent TX command
};

/**
 * Motor Error (CAN 0x003, 10ms interval)
 * Motor-specific error flags
 */
struct TimestampedMotorError {
    uint64_t motorError;        // Motor error flags (64-bit for ODrive)
    uint64_t timestamp;         // GPTimer microseconds
    bool isResponse;            // True if correlates to recent TX command
};

/**
 * Encoder Error (CAN 0x004, 10ms interval)
 * Encoder-specific error flags
 */
struct TimestampedEncoderError {
    uint32_t encoderError;      // Encoder error flags
    uint64_t timestamp;         // GPTimer microseconds
    bool isResponse;            // True if correlates to recent TX command
};

/**
 * Controller Error (CAN 0x01D, 10ms interval)
 * Controller-specific error flags
 */
struct TimestampedControllerError {
    uint32_t controllerError;   // Controller error flags
    uint64_t timestamp;         // GPTimer microseconds
    bool isResponse;            // True if correlates to recent TX command
};

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
        uint64_t motorError;          // Motor error flags (64-bit from CYCLIC_MOTOR_ERROR 0x03)
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
    
    // ===== BDS V2: RING BUFFER STORAGE (Phase 1.7) =====
    // Store timestamped message history for staleness detection and TX correlation
    void storeHeartbeat(uint32_t axisError, uint8_t axisState, uint8_t motorErrorFlag, uint8_t encoderErrorFlag, 
                     uint8_t controllerErrorFlag, uint8_t trajectoryDoneFlag, uint64_t timestamp, bool isResponse = false);
    void storeEncoder(float position, float velocity, uint64_t timestamp, bool isResponse = false);
    void storeIq(float iqSetpoint, float iqMeasured, uint64_t timestamp, bool isResponse = false);
    void storeBusVI(float busVoltage, float busCurrent, uint64_t timestamp, bool isResponse = false);
    void storeMotorError(uint64_t motorError, uint64_t timestamp, bool isResponse = false);
    void storeEncoderError(uint32_t encoderError, uint64_t timestamp, bool isResponse = false);
    void storeControllerError(uint32_t controllerError, uint64_t timestamp, bool isResponse = false);
    
    // ===== BDS V2: HISTORICAL DATA ACCESS =====
    const TimestampedHeartbeat* getLatestHeartbeat() const;
    const TimestampedEncoder* getLatestEncoder() const;
    const TimestampedIq* getLatestIq() const;
    const TimestampedBusVI* getLatestBusVI() const;
    
    // History access methods
    const TimestampedIq* getHistoryIq(size_t index) const;
    const TimestampedEncoder* getHistoryEncoder(size_t index) const;
    
    // ===== TIMING SYSTEM ACCESSORS (NEW) =====
    // Accessors for cyclic-based timing window system
    
    /**
     * Get latest encoder bundle timestamp (for timing window calculations)
     * @return Microsecond timestamp of last encoder message (0 if none)
     */
    uint64_t getEncoderTimestamp() const;
    
    /**
     * Get latest IQ current bundle timestamp (for movement verification)
     * @return Microsecond timestamp of last IQ message (0 if none)
     */
    uint64_t getIqTimestamp() const;
    
    /**
     * Calculate timing offset from encoder bundle start
     * @param currentTime Current time in microseconds
     * @return Offset in microseconds from last encoder bundle
     */
    uint32_t getTimingOffset(uint64_t currentTime) const;
    
    /**
     * Check if we're in a valid command timing window
     * @param currentTime Current time in microseconds
     * @param offsetUs Window offset from encoder start (default CMD_WINDOW_OFFSET_US)
     * @param durationUs Window duration (default CMD_WINDOW_DURATION_US)
     * @return True if current time is within command window
     */
    bool isInCommandWindow(uint64_t currentTime, uint32_t offsetUs = CMD_WINDOW_OFFSET_US, uint32_t durationUs = CMD_WINDOW_DURATION_US) const;
    
    /**
     * Get movement verification data (IQ current and position)
     * @param iqSetpoint Output: IQ setpoint current (A)
     * @param iqMeasured Output: IQ measured current (A)
     * @param position Output: Current position (turns)
     * @param velocity Output: Current velocity (turns/sec)
     * @return True if data is available (recent), false if stale
     */
    bool getMovementData(float& iqSetpoint, float& iqMeasured, float& position, float& velocity) const;
    
    /**
     * Check if movement has occurred since baseline
     * @param baselinePos Baseline position (turns)
     * @param baselineIq Baseline IQ current (A)
     * @param posThreshold Position threshold (default MOVEMENT_POS_THRESHOLD_TICKS/8192)
     * @param iqThreshold IQ threshold (default MOVEMENT_IQ_THRESHOLD_MA/1000)
     * @return True if movement detected, false otherwise
     */
    bool hasMovementOccurred(float baselinePos, float baselineIq, 
                            float posThreshold = (float)MOVEMENT_POS_THRESHOLD_TICKS/8192.0f,
                            float iqThreshold = (float)MOVEMENT_IQ_THRESHOLD_MA/1000.0f) const;
    
    /**
     * Get data staleness for timing validation
     * @param maxAgeUs Maximum acceptable age in microseconds
     * @return True if data is fresh, false if stale
     */
    bool isDataFresh(uint64_t maxAgeUs = 50000) const; // Default 50ms
    const TimestampedMotorError* getLatestMotorError() const;
    const TimestampedEncoderError* getLatestEncoderError() const;
    const TimestampedControllerError* getLatestControllerError() const;
    
    // ===== BDS V2: STALENESS DETECTION (µs resolution) =====
    bool isEncoderStale(uint64_t maxAgeMicros = 500000) const;  // Default 500ms
    bool isHeartbeatStale(uint64_t maxAgeMicros = 500000) const;
    bool isIqStale(uint64_t maxAgeMicros = 500000) const;
    uint64_t getEncoderAgeMicros(uint64_t currentTime) const;
    uint64_t getHeartbeatAgeMicros(uint64_t currentTime) const;
    
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
    void updateMotorError(uint64_t motorError, SafeString* debugLog = nullptr);
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
    uint64_t getMotorError() const;  // 64-bit for ODrive motor errors
    uint32_t getEncoderError() const;
    uint32_t getControllerError() const;
    bool hasAnyError() const;
    uint32_t getLastErrorUpdate() const;
    
    // Safe error queries (read previous entry to avoid race conditions)
    uint64_t getMotorErrorSafe() const;  // Reads previous entry, not latest
    uint32_t getControllerErrorSafe() const;
    
    // ===== TRAJECTORY COMPLETION DETECTION =====
    
    // Check if trajectory move is complete (trajectory_done_flag in heartbeat)
    bool isTrajectoryComplete() const;
    
    // Check if trajectory move is complete with GPTimer timestamp
    bool isTrajectoryComplete(uint64_t& timestamp) const;
    
    // ===== HEARTBEAT FLAG ACCESS =====
    
    // Get individual error flags from latest heartbeat
    bool hasMotorErrorFlag() const;
    bool hasEncoderErrorFlag() const;
    bool hasControllerErrorFlag() const;
    
    // Get all heartbeat flags at once
    bool getHeartbeatFlags(uint8_t& motorFlag, uint8_t& encoderFlag, uint8_t& controllerFlag, uint8_t& trajFlag) const;
    
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
                          errors_{0, 0, 0, 0, 0, false}, estimates_{0.0f, 0.0f, 0} {
        // Create mutex for thread safety
        dataMutex_ = xSemaphoreCreateMutex();
    }
    BroadcastDataStore(const BroadcastDataStore&) = delete;  // No copies
    BroadcastDataStore& operator=(const BroadcastDataStore&) = delete;  // No assignment
    
    // Thread safety mutex
    SemaphoreHandle_t dataMutex_;
    
    // ===== V1 STORAGE (backward compatibility, still used by existing code) =====
    AxisData axis_;
    PowerData power_;
    SensorData sensors_;
    ErrorData errors_;
    EncoderEstimatesData estimates_;
    
    // ===== V2 STORAGE (ring buffers with timestamps) =====
    RingBuffer<TimestampedHeartbeat, BDS_HEARTBEAT_HISTORY_SIZE> heartbeatHistory_;
    RingBuffer<TimestampedEncoder, BDS_ENCODER_HISTORY_SIZE> encoderHistory_;
    RingBuffer<TimestampedIq, BDS_IQ_HISTORY_SIZE> iqHistory_;
    RingBuffer<TimestampedBusVI, BDS_BUSVI_HISTORY_SIZE> busVIHistory_;
    RingBuffer<TimestampedMotorError, BDS_MOTOR_ERROR_HISTORY_SIZE> motorErrorHistory_;
    RingBuffer<TimestampedEncoderError, BDS_ENCODER_ERROR_HISTORY_SIZE> encoderErrorHistory_;
    RingBuffer<TimestampedControllerError, BDS_CONTROLLER_ERROR_HISTORY_SIZE> controllerErrorHistory_;
};

#endif // BROADCAST_DATA_STORE_H
