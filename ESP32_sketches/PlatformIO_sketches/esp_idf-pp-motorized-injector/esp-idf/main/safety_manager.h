#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <cstdint>
#include "debounced_input.h"
#include "hx711.h"
#include "config.h"

enum MachineError {
    ERR_NONE = 0,
    ERR_ESTOP = 1,
    ERR_BARREL_POSITION_LOST = 2,
    ERR_NOZZLE_NOT_BLOCKED = 3,
    ERR_OVER_TEMP = 4,
    ERR_HARD_LIMIT = 5,
    ERR_UNDER_TEMP = 6,
    ERR_BOTTOM_ENDSTOP_COLLISION = 7,
    ERR_TOP_ENDSTOP_COLLISION = 8,
    ERR_BROADCAST_STALE = 9,
    ERR_CAN_RTR_FAILURE = 10
};

enum SafetyContext {
    CTX_IDLE,
    CTX_MOVING_FREE,
    CTX_BLOCKED,
    CTX_PURGE
};

class SafetyManager {
public:
    SafetyManager();
    void begin();

    void updateInputs();
    bool check(float current_velocity, bool is_moving_down);

    void setContext(SafetyContext ctx);
    void enableMotorPower(bool enable);
    void triggerHalt(MachineError err);
    void resetError();

    void forceEmergencyShutdown(const char* reason);
    void flagError(MachineError err);

    static SafetyManager& getInstance();

    bool isEStopPressed();
    bool isBarrelOpen();
    bool isTopEndstopHit();
    bool isBottomEndstopHit();

    MachineError getLastError() { return _lastError; }
    SafetyContext getContext() { return _currentContext; }
    long getPressure() { return _currentPressure; }
    bool isMotorPowerEnabled() const { return _motorPowerEnabled; }

    void retareLoadCell();
    void setLoadCellScale(float scale);
    void setLoadCellOffset(long offset);
    long getLoadCellRaw();
    void setLoadCellAveraging(uint8_t samples);
    void setLoadCellMedianFilter(bool enable);

    DebouncedInput& getTopEndstop() { return dbTop; }
    DebouncedInput& getBottomEndstop() { return dbBot; }

private:
    HX711 _loadCell;
    MachineError _lastError;
    SafetyContext _currentContext;
    bool _powerCutLatched;

    long _currentPressure;
    long _pressureBaseline;
    uint64_t _moveStartTime;
    bool _wasMovingDown;
    float _startPosition;

    float _loadCellScale;
    long _loadCellOffset;
    uint8_t _loadCellAvgSamples;
    long* _loadCellBuffer;
    uint8_t _loadCellBufferIdx;
    bool _loadCellUseMedian;
    long _lastValidReading;

    DebouncedInput dbEStop;
    DebouncedInput dbBarrel;
    DebouncedInput dbTop;
    DebouncedInput dbBot;

    uint8_t _estopCounter;
    uint8_t _barrelCounter;
    uint8_t _topCounter;
    uint8_t _botCounter;

    uint64_t _bootTime;
    bool _motorPowerEnabled;
    uint64_t _lastPowerEnableTimeUs;
    uint64_t _lastPowerDisableTimeUs;
    uint64_t _lastPowerEnableBlockLogUs;

    static const uint8_t CONFIDENCE_THRESHOLD = 50;
};

#endif // SAFETY_MANAGER_H
