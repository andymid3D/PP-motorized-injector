#ifndef BROADCAST_DATA_STORE_H
#define BROADCAST_DATA_STORE_H

#include <cstdint>
#include "ring_buffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// History sizes
#define BDS_IQ_HISTORY_SIZE           500
#define BDS_ENCODER_HISTORY_SIZE      500
#define BDS_MOTOR_ERROR_HISTORY_SIZE  200
#define BDS_ENCODER_ERROR_HISTORY_SIZE 200
#define BDS_CONTROLLER_ERROR_HISTORY_SIZE 200
#define BDS_HEARTBEAT_HISTORY_SIZE    500

struct TimestampedHeartbeat {
    uint32_t axisError;
    uint8_t axisState;
    uint8_t motorErrorFlag;
    uint8_t encoderErrorFlag;
    uint8_t controllerErrorFlag;
    uint8_t trajectoryDoneFlag;
    uint64_t timestamp;
    bool isResponse;
};

struct TimestampedEncoder {
    float position;
    float velocity;
    uint64_t timestamp;
    bool isResponse;
};

struct TimestampedIq {
    float iqSetpoint;
    float iqMeasured;
    uint64_t timestamp;
    bool isResponse;
};

struct TimestampedMotorError {
    uint64_t motorError;
    uint64_t timestamp;
    bool isResponse;
};

struct TimestampedEncoderError {
    uint32_t encoderError;
    uint64_t timestamp;
    bool isResponse;
};

struct TimestampedControllerError {
    uint32_t controllerError;
    uint64_t timestamp;
    bool isResponse;
};

class BroadcastDataStore {
public:
    struct AxisData {
        uint8_t state;
        float positionTurns;
        float velocityTurnsPerSec;
        uint64_t lastUpdateUs;
    };

    struct PowerData {
        float iqS;
        float iqM;
        float busCurrent;
        float busVoltage;
        uint64_t lastUpdateUs;
    };

    struct SensorData {
        float temperature;
        float pressureTorque;
        bool topEndstop;
        bool bottomEndstop;
        bool barrelEndstop;
    };

    struct ErrorData {
        uint32_t axisError;
        uint64_t motorError;
        uint32_t encoderError;
        uint32_t controllerError;
        uint64_t lastUpdateUs;
        bool hasAnyError;
    };

    struct EncoderEstimatesData {
        float position;
        float velocity;
        uint64_t lastUpdateUs;
    };

    static BroadcastDataStore& getInstance();

    void storeHeartbeat(uint32_t axisError, uint8_t axisState, uint8_t motorErrorFlag, uint8_t encoderErrorFlag,
                       uint8_t controllerErrorFlag, uint8_t trajectoryDoneFlag, uint64_t timestamp, bool isResponse = false);
    void storeEncoder(float position, float velocity, uint64_t timestamp, bool isResponse = false);
    void storeIq(float iqSetpoint, float iqMeasured, uint64_t timestamp, bool isResponse = false);
    void storeMotorError(uint64_t motorError, uint64_t timestamp, bool isResponse = false);
    void storeEncoderError(uint32_t encoderError, uint64_t timestamp, bool isResponse = false);
    void storeControllerError(uint32_t controllerError, uint64_t timestamp, bool isResponse = false);

    const TimestampedHeartbeat* getLatestHeartbeat() const;
    const TimestampedEncoder* getLatestEncoder() const;
    const TimestampedIq* getLatestIq() const;

    const TimestampedMotorError* getLatestMotorError() const;
    const TimestampedEncoderError* getLatestEncoderError() const;
    const TimestampedControllerError* getLatestControllerError() const;

    bool isEncoderStale(uint64_t maxAgeMicros = 500000) const;
    bool isHeartbeatStale(uint64_t maxAgeMicros = 500000) const;
    bool isIqStale(uint64_t maxAgeMicros = 500000) const;
    uint64_t getEncoderAgeMicros(uint64_t currentTime) const;
    uint64_t getHeartbeatAgeMicros(uint64_t currentTime) const;

    bool isBroadcastDataStale() const;

    // Queries
    uint8_t getAxisState() const;
    float getPosition() const;
    float getVelocity() const;
    uint64_t getLastAxisUpdate() const;

    float getIqSetpoint() const;
    float getIqMeasured() const;
    float getBusCurrent() const;
    float getBusVoltage() const;
    uint64_t getLastPowerUpdate() const;

    float getTemperature() const;
    float getPressure() const;
    bool isTopEndstopActive() const;
    bool isBottomEndstopActive() const;
    bool isBarrelEndstopActive() const;

    uint32_t getAxisError() const;
    uint64_t getMotorError() const;
    uint32_t getEncoderError() const;
    uint32_t getControllerError() const;
    bool hasAnyError() const;
    uint64_t getLastErrorUpdate() const;

    uint64_t getMotorErrorSafe() const;
    uint32_t getControllerErrorSafe() const;

    bool isTrajectoryComplete() const;
    bool isTrajectoryComplete(uint64_t& timestamp) const;

    bool isVelocityBelowThreshold(float thresholdTurnsPerSec) const;
    bool isMoving(float threshold = 0.05f) const;

private:
    BroadcastDataStore();
    BroadcastDataStore(const BroadcastDataStore&) = delete;
    BroadcastDataStore& operator=(const BroadcastDataStore&) = delete;

    mutable SemaphoreHandle_t dataMutex_;

    AxisData axis_;
    PowerData power_;
    SensorData sensors_;
    ErrorData errors_;
    EncoderEstimatesData estimates_;

    RingBuffer<TimestampedHeartbeat, BDS_HEARTBEAT_HISTORY_SIZE> heartbeatHistory_;
    RingBuffer<TimestampedEncoder, BDS_ENCODER_HISTORY_SIZE> encoderHistory_;
    RingBuffer<TimestampedIq, BDS_IQ_HISTORY_SIZE> iqHistory_;
    RingBuffer<TimestampedMotorError, BDS_MOTOR_ERROR_HISTORY_SIZE> motorErrorHistory_;
    RingBuffer<TimestampedEncoderError, BDS_ENCODER_ERROR_HISTORY_SIZE> encoderErrorHistory_;
    RingBuffer<TimestampedControllerError, BDS_CONTROLLER_ERROR_HISTORY_SIZE> controllerErrorHistory_;

    mutable uint64_t lastHeartbeatRead_;
    static const uint64_t FRESH_DATA_DELAY_US = 15000;
};

#endif // BROADCAST_DATA_STORE_H
