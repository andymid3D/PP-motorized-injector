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
            // Set motor limits for manual control
            MotorWrapper::setMotorLimits(motor, VEL_LIMIT_PURGE, CURRENT_LIMIT_REFILL, "PurgeZero");
            delay(CAN_COMMAND_GAP_MS + 5);
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
                MotorWrapper::setModeAndMove(motor, 2, 1, -SPEED_PURGE, "Purge Up");
                lastCommandTime = now;
            } else if (buttonDown == LOW) {
                // Lower button pressed: push plastic out (down) = positive velocity
                MotorWrapper::setModeAndMove(motor, 2, 1, SPEED_PURGE, "Purge Down");
                lastCommandTime = now;
            } else {
                // Neither button pressed: stop
                MotorWrapper::setModeAndMove(motor, 2, 1, 0, "Purge Stop");
                lastCommandTime = now;
            }
        }
        
        // ===== CENTER BUTTON: Confirm zero point =====
        if (buttonCenter == LOW) {
            // User pressed center: confirm current position as zero point for injection
            MotorWrapper::setModeAndMove(motor, 2, 1, 0, "Purge Confirm");  // Stop motor
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
