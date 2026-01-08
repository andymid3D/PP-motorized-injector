#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <Arduino.h>
#include <Bounce2.h> 
#include "config.h"
#include "HX711.h"

enum MachineError {
    ERR_NONE = 0,
    ERR_ESTOP = 1,
    ERR_BARREL_POSITION_LOST = 2, 
    ERR_NOZZLE_NOT_BLOCKED = 3,   
    ERR_OVER_TEMP = 4,
    ERR_HARD_LIMIT = 5,
    ERR_UNDER_TEMP = 6
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
    
    // Main Loop Functions
    void updateInputs(); 
    bool check(float current_velocity, bool is_moving_down);
    
    // Context & Control
    void setContext(SafetyContext ctx);
    void enableMotorPower(bool enable);
    void triggerHalt(MachineError err);
    void resetError();
    
    // Clean Getters 
    bool isEStopPressed();      
    bool isBarrelOpen();        
    bool isTopEndstopHit();     
    bool isBottomEndstopHit();  
    
    // Bounce2 Accessors (for advanced debouncing control)
    Bounce2::Button& getTopEndstop() { return dbTop; }
    Bounce2::Button& getBottomEndstop() { return dbBot; }
    
    MachineError getLastError() { return _lastError; }
    SafetyContext getContext() { return _currentContext; }
    long getPressure() { return _currentPressure; }
    
    // HX711 Load Cell Configuration Methods
    void retareLoadCell();                    // Re-tare after motor power on (fixes EMI baseline shift)
    void setLoadCellScale(float scale);       // Set calibration factor (counts to engineering units)
    void setLoadCellOffset(long offset);      // Manual baseline adjustment
    long getLoadCellRaw();                    // Get raw ADC value for diagnostics
    void setLoadCellAveraging(uint8_t samples); // Enable moving average filter (1=off, 2-10 recommended)
    void setLoadCellMedianFilter(bool enable); // Use median filter instead of average (better for EMI spikes)

private:
    HX711 _loadCell;
    MachineError _lastError;
    SafetyContext _currentContext; 
    
    long _currentPressure;
    long _pressureBaseline;
    unsigned long _moveStartTime;
    bool _wasMovingDown;
    float _startPosition; 
    
    // HX711 Configuration
    float _loadCellScale;
    long _loadCellOffset;
    uint8_t _loadCellAvgSamples;
    long* _loadCellBuffer;  // Circular buffer for averaging/median
    uint8_t _loadCellBufferIdx;
    bool _loadCellUseMedian; // Use median filter instead of average
    long _lastValidReading;  // Last known good reading (for outlier rejection) 

    // Centralized Debouncers (Physical Contact Bounce)
    Bounce2::Button dbEStop = Bounce2::Button();
    Bounce2::Button dbBarrel = Bounce2::Button();
    Bounce2::Button dbTop = Bounce2::Button();
    Bounce2::Button dbBot = Bounce2::Button();

    // EMI Confidence Counters (Software Glitch Filter)
    uint8_t _estopCounter;
    uint8_t _barrelCounter;
    uint8_t _topCounter;
    uint8_t _botCounter;

    // Threshold: Number of consecutive loops a signal must be active to be trusted
    // Loop runs fast, so 50 counts is roughly 20-50ms of continuous signal
    static const uint8_t CONFIDENCE_THRESHOLD = 50;
};

#endif