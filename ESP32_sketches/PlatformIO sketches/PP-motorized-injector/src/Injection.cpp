#include "Injection.h"
#include "config.h"

namespace Injection {
    // ===== STATIC STATE VARIABLES =====
    static InjectionPhase phase = DONE;
    static bool stateEntry = false;
    static unsigned long stateEnterTime = 0;
    static unsigned long phaseStartTime = 0;
    static bool complete = false;
    static bool error = false;
    static unsigned long lastCommandTime = 0;
    static bool pressureSensorChecked = false;
    
    // Store mould parameters
    static actualMouldParams_t currentMould;
    
    // Position tracking
    static float injectStartPos = 0.0f;
    static float packStartPos = 0.0f;
    static float targetInjectPos = 0.0f;
    static float targetPackPos = 0.0f;
    
    // Helper: Volume to Turns conversion
    static float volToTurns(float cm3) {
        return cm3 * TURNS_PER_CM3_VOL;
    }
    
    // ===== BEGIN: Initialize with mould parameters =====
    void begin(const actualMouldParams_t& mouldParams) {
        currentMould = mouldParams;
        phase = FILLING;
        stateEntry = true;
        stateEnterTime = millis();
        phaseStartTime = millis();
        complete = false;
        error = false;
        lastCommandTime = millis();
        pressureSensorChecked = false;
        
        injectStartPos = 0.0f;
        packStartPos = 0.0f;
    }
    
    // ===== UPDATE: Non-blocking injection logic =====
    bool update(CanBusHandlerV2& motor) {
        unsigned long now = millis();
        unsigned long stateElapsed = now - stateEnterTime;
        unsigned long phaseElapsed = now - phaseStartTime;
        
        // ===== ENTRY: Initialize FILLING phase =====
        if (stateEntry) {
            // Pressure sensor check (first ms)
            pressureSensorChecked = true;
            
            // Capture start position and calculate targets
            injectStartPos = motor.getPosition();
            targetInjectPos = injectStartPos + volToTurns(currentMould.fillVolume);
            
            // Safety: don't exceed mechanical limit
            if (targetInjectPos > POS_BOTTOM_MAX) {
                targetInjectPos = POS_BOTTOM_MAX;
            }
            
            // Set position control mode
            motor.setControllerModes(ODriveCANProtocol::ControlMode::POSITION_CONTROL,
                                    ODriveCANProtocol::InputMode::PASSTHROUGH);
            motor.setInputPos(targetInjectPos);
            lastCommandTime = now;
            
            stateEntry = false;
        }
        
        // ===== FILLING PHASE =====
        if (phase == FILLING) {
            // Resend position command periodically to ensure motor keeps moving
            if (now - lastCommandTime >= CAN_COMMAND_GAP_MS * 2) {
                motor.setInputPos(targetInjectPos);
                lastCommandTime = now;
            }
            
            // Check for completion: velocity drops to near-zero and stays stable
            bool velocityLow = fabs(motor.getVelocity()) < 0.1f;
            bool timingStable = phaseElapsed > 500;
            bool positionClose = fabs(motor.getPosition() - targetInjectPos) < 1.0f;
            
            if (velocityLow && timingStable && positionClose) {
                // Transition to PACKING phase
                phase = PACKING;
                phaseStartTime = now;
                packStartPos = motor.getPosition();
                targetPackPos = packStartPos + volToTurns(currentMould.packVolume);
                
                // Safety: don't exceed mechanical limit
                if (targetPackPos > POS_BOTTOM_MAX) {
                    targetPackPos = POS_BOTTOM_MAX;
                }
                
                // Set lower pressure for packing
                motor.setInputPos(targetPackPos);
                lastCommandTime = now;
                
                return false;  // Still running
            }
            
            // Safety timeout (injection should complete in <10 seconds typically)
            if (phaseElapsed > 30000) {
                error = true;
                complete = true;
                motor.setInputVel(0);
                return true;
            }
            
            return false;
        }
        
        // ===== PACKING PHASE =====
        if (phase == PACKING) {
            // Resend position command periodically
            if (now - lastCommandTime >= CAN_COMMAND_GAP_MS * 2) {
                motor.setInputPos(targetPackPos);
                lastCommandTime = now;
            }
            
            // Packing time in milliseconds
            unsigned long packTimeMs = (unsigned long)(currentMould.packTime * 1000);
            
            // Check for completion: pack time elapsed
            if (phaseElapsed >= packTimeMs) {
                phase = DONE;
                complete = true;
                motor.setInputVel(0);
                return true;
            }
            
            return false;
        }
        
        return complete;
    }
    
    // ===== QUERY STATE =====
    bool isComplete() {
        return complete && !error;
    }
    
    bool hasError() {
        return error;
    }
    
    InjectionPhase getPhase() {
        return phase;
    }
    
    // ===== RESET =====
    void reset() {
        stateEntry = true;
        phase = DONE;
        complete = false;
        error = false;
        pressureSensorChecked = false;
    }
}
