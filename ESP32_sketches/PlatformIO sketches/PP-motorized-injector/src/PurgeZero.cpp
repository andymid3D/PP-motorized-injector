#include "PurgeZero.h"
#include "config.h"
#include "MotorWrapper.h"

namespace PurgeZero {
    // ===== STATIC STATE VARIABLES =====
    static bool stateEntry = false;
    static bool complete = false;
    static bool error = false;
    static bool buttonsReleased = false;  // Track initial button release debounce
    static unsigned long lastCommandTime = 0;
    
    // ===== BEGIN: Initialize on state entry =====
    void begin() {
        stateEntry = true;
        complete = false;
        error = false;
        buttonsReleased = false;
        lastCommandTime = millis();
    }
    
    // ===== UPDATE: Handle button-controlled movement =====
    bool update(CanBusHandlerV2& motor, bool buttonUp, bool buttonDown, bool buttonCenter) {
        unsigned long now = millis();
        
        // ===== DEBOUNCE: Wait for buttons to release at entry =====
        if (stateEntry) {
            // Set velocity control mode (once at entry)
            motor.setControllerModes(ODriveCANProtocol::ControlMode::VELOCITY_CONTROL,
                                    ODriveCANProtocol::InputMode::PASSTHROUGH);
            lastCommandTime = now;
            stateEntry = false;
        }
        
        // Debounce: wait for upper+lower buttons to be released
        if (!buttonsReleased) {
            if (buttonUp == HIGH && buttonDown == HIGH) {
                buttonsReleased = true;
            }
            return false;  // Don't accept commands until debounced
        }
        
        // ===== MOVEMENT CONTROL =====
        // Only send new velocity commands if minimum gap has elapsed
        if (now - lastCommandTime >= CAN_COMMAND_GAP_MS) {
            if (buttonUp == LOW) {
                // Upper button pressed: retract (up) = negative velocity
                motor.setInputVel(-SPEED_PURGE);
                lastCommandTime = now;
            } else if (buttonDown == LOW) {
                // Lower button pressed: push plastic out (down) = positive velocity
                motor.setInputVel(SPEED_PURGE);
                lastCommandTime = now;
            } else {
                // Neither button pressed: stop
                motor.setInputVel(0);
                lastCommandTime = now;
            }
        }
        
        // ===== CENTER BUTTON: Confirm zero point =====
        if (buttonCenter == LOW) {
            // User pressed center: confirm current position as zero point for injection
            motor.setInputVel(0);  // Stop motor
            lastCommandTime = now;
            complete = true;
            return true;
        }
        
        return false;
    }
    
    // ===== QUERY STATE =====
    bool isComplete() {
        return complete && !error;
    }
    
    bool hasError() {
        return error;
    }
    
    // ===== RESET =====
    void reset() {
        stateEntry = true;
        complete = false;
        error = false;
        buttonsReleased = false;
    }
}
