#include "AntiDrip.h"
#include "config.h"
#include "MotorWrapper.h"

namespace AntiDrip {
    // ===== STATIC STATE VARIABLES =====
    static bool stateEntry = false;
    static unsigned long stateEnterTime = 0;
    static bool complete = false;
    static bool isTimeoutFlag = false;
    static bool isAbortedFlag = false;
    static bool error = false;
    static unsigned long lastCommandTime = 0;
    static bool pressureSensorChecked = false;  // Check pressure in first ms
    
    // ===== BEGIN: Initialize on state entry =====
    void begin() {
        stateEntry = true;
        stateEnterTime = millis();
        complete = false;
        isTimeoutFlag = false;
        isAbortedFlag = false;
        error = false;
        lastCommandTime = millis();
        pressureSensorChecked = false;
    }
    
    // ===== UPDATE: Apply slow retract, monitor for timeout/user input =====
    bool update(CanBusHandlerV2& motor) {
        unsigned long now = millis();
        unsigned long elapsed = now - stateEnterTime;
        
        // ===== ENTRY: Set velocity control mode =====
        if (stateEntry) {
            MotorWrapper::setMotorLimits(motor, VEL_LIMIT_ANTIDRIP, CURRENT_LIMIT_REFILL, "AntiDrip");
            delay(CAN_COMMAND_GAP_MS + 5);
            lastCommandTime = now;
            stateEntry = false;
        }
        
        // ===== PRESSURE SENSOR CHECK (First ms) =====
        if (!pressureSensorChecked) {
            // Check if mould/NozzleBlock is in place (sensor should detect pressure)
            // This is a backup check; primary is user button confirmation
            // TODO: Implement pressure sensor read from HX711
            // For now, just log that we're waiting for mould placement
            pressureSensorChecked = true;
        }
        
        // ===== CONTINUOUS COMMAND: Send slow upward velocity =====
        if (now - lastCommandTime >= CAN_COMMAND_GAP_MS) {
            MotorWrapper::setModeAndMove(motor, 2, 1, -SPEED_ANTIDRIP, "AntiDrip Up");  // Negative = up, slow retract to prevent drip
            lastCommandTime = now;
        }
        
        // ===== TIMEOUT CHECK =====
        if (elapsed > TIME_ANTIDRIP_TIMEOUT) {
            isTimeoutFlag = true;
            complete = true;
            MotorWrapper::setModeAndMove(motor, 2, 1, 0, "AntiDrip Stop");  // Stop motor
            return true;
        }
        
        return complete;
    }
    
    // ===== QUERY STATE =====
    bool isComplete() {
        return complete && !isTimeoutFlag && !isAbortedFlag && !error;
    }
    
    bool isTimeout() {
        return complete && isTimeoutFlag;
    }
    
    bool isAborted() {
        return complete && isAbortedFlag;
    }
    
    bool hasError() {
        return error;
    }
    
    // ===== RESET =====
    void reset() {
        stateEntry = true;
        complete = false;
        isTimeoutFlag = false;
        isAbortedFlag = false;
        error = false;
        pressureSensorChecked = false;
    }
}
