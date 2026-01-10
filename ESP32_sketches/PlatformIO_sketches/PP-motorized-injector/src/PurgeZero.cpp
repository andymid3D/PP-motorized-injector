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
    static int lastButtonState = 0;  // 0=stop, 1=up, 2=down
    
    // ===== BEGIN: Initialize on state entry =====
    void begin() {
        stateEntry = true;
        complete = false;
        error = false;
        buttonsReleased = false;
        lastCommandTime = millis();
        lastButtonState = 0;  // Reset button state
    }
    
    // ===== UPDATE: Handle button-controlled movement =====
    bool update(CanBusHandlerV2& motor, bool buttonUp, bool buttonDown, bool buttonCenter) {
        unsigned long now = millis();
        
        // ===== DEBOUNCE: Wait for buttons to release at entry =====
        if (stateEntry) {
            // Queue limit command - ring buffer handles timing
            MotorWrapper::setMotorLimits(motor, PURGE_VEL_LIMIT, PURGE_CURRENT_LIMIT, "PurgeZero");
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
        // Determine current button state
        int currentButtonState = 0;  // 0=stop, 1=up, 2=down
        if (buttonUp == LOW) {
            currentButtonState = 1;  // Up
        } else if (buttonDown == LOW) {
            currentButtonState = 2;  // Down
        }
        
        // Only send commands if button state changed AND minimum gap elapsed
        if (currentButtonState != lastButtonState && now - lastCommandTime >= CAN_COMMAND_GAP_MS) {
            if (currentButtonState == 1) {
                // Upper button pressed: retract (up) = PURGE_VEL_UP (negative in config)
                MotorWrapper::setModeAndMove(motor, 2, 1, PURGE_VEL_UP, "Purge Up");
            } else if (currentButtonState == 2) {
                // Lower button pressed: push plastic out (down) = PURGE_VEL_DOWN (positive in config)
                MotorWrapper::setModeAndMove(motor, 2, 1, PURGE_VEL_DOWN, "Purge Down");
            } else {
                // Neither button pressed: stop
                MotorWrapper::setModeAndMove(motor, 2, 1, 0, "Purge Stop");
            }
            lastButtonState = currentButtonState;
            lastCommandTime = now;
        }
        
        // ===== CENTER BUTTON: Confirm zero point =====
        if (buttonCenter) {  // buttonCenter is now bool pressed() event from main
            // User released center: confirm current position as zero point for injection
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
