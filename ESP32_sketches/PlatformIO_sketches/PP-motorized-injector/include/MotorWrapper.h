#ifndef MOTOR_WRAPPER_H
#define MOTOR_WRAPPER_H

#include <Arduino.h>
#include "CanBusHandlerV2.h"
#include "config.h"
#include "MessageBuffer.h"

/*
 * MOTOR WRAPPER - Centralized Motor Control with CAN Timing Enforcement
 * 
 * Purpose: Provide consistent motor control interface with:
 * - CAN command gap enforcement (CAN_COMMAND_GAP_MS from config.h)
 * - Limit setting before moves (vel_limit, current_lim)
 * - TRAP_TRAJ parameter configuration
 * - Command logging for diagnostics
 * - Module ID tracking for response correlation
 * - Retry logic with priority-based timeouts
 * - Single point for all motor commands
 * 
 * All motor movements MUST go through these wrappers to ensure:
 * - ODrive firmware has time to process mode changes
 * - CAN bus isn't overwhelmed
 * - Consistent timing across all states
 * - Module-specific retry policies
 * 
 * Usage Pattern:
 *   1. setMotorLimits(vel, current, moduleId, "context")     // Set limits first
 *   2. setTrapTrajParams(vel, accel, decel, moduleId, "ctx") // If using TRAP_TRAJ
 *   3. setModeAndMove(mode, inputMode, val, moduleId, "cmd") // Execute move
 *   4. adjustMotorLimits(current, moduleId, "reason")        // Dynamic adjustment during move
 * 
 * Retry Priority Levels:
 *   - PRIORITY_CRITICAL (50ms): INJECTION - time-critical plastic flow
 *   - PRIORITY_HIGH (75ms): COMPRESSION - contact detection timing
 *   - PRIORITY_NORMAL (100ms): REFILL, RELEASE - moderate speed moves
 *   - PRIORITY_LOW (200ms): ANTIDRIP, PURGE_ZERO - slow vel_ramp moves
 */

namespace MotorWrapper {
    // ===== RETRY PRIORITY LEVELS =====
    enum RetryPriority {
        PRIORITY_CRITICAL = 0,  // 50ms timeout - INJECTION (time-critical plastic flow)
        PRIORITY_HIGH = 1,      // 75ms timeout - COMPRESSION (contact detection timing)
        PRIORITY_NORMAL = 2,    // 100ms timeout - REFILL, RELEASE (moderate speed moves)
        PRIORITY_LOW = 3         // 200ms timeout - ANTIDRIP, PURGE_ZERO (slow vel_ramp moves)
    };
    
    // ===== INITIALIZATION =====
    void init();
    
    // ===== CORE WRAPPER FUNCTIONS =====
    
    // Set motor velocity and current limits (CAN message 0x00F)
    // Must be called BEFORE setModeAndMove for every move
    void setMotorLimits(CanBusHandlerV2& motor, float vel_lim, float current_lim, uint8_t moduleId, String context);
    
    // Set TRAP_TRAJ parameters (CAN messages 0x011 + 0x012)
    // Call AFTER setMotorLimits, BEFORE setModeAndMove when using TRAP_TRAJ input mode
    void setTrapTrajParams(CanBusHandlerV2& motor, float vel_limit, float accel, float decel, uint8_t moduleId, String context);
    
    // Set controller modes (CAN 0x00B + 0x00C) - for mode changes without movement
    void setControllerModes(CanBusHandlerV2& motor, ODriveCANProtocol::ControlMode ctrlMode, 
                           ODriveCANProtocol::InputMode inputMode, uint8_t moduleId, String context);
    
    // Execute motor move command with mode and input mode
    // Enforces CAN_COMMAND_GAP_MS between commands
    void setModeAndMove(CanBusHandlerV2& motor, int ctrlMode, int inputMode, float value, uint8_t moduleId, String cmdName);
    
    // Execute motor move with retry logic based on priority
    // Returns true if command succeeded, false if all retries failed
    bool setModeAndMoveWithRetry(CanBusHandlerV2& motor, int ctrlMode, int inputMode, float value, 
                                 uint8_t moduleId, String cmdName, RetryPriority priority);
    
    // Dynamic adjustment of limits during move
    // Typically used to increase current_lim after contact detection
    void adjustMotorLimits(CanBusHandlerV2& motor, float current_lim, uint8_t moduleId, String reason);
    
    // ===== SAFETY FUNCTIONS =====
    
    // Universal safe stop using State 1 (IDLE) - works from any mode
    void universalStop(CanBusHandlerV2& motor, uint8_t moduleId, String reason);
    
    // Safe mode change with universal stop before mode transition
    void safeModeChange(CanBusHandlerV2& motor, int ctrlMode, int inputMode, 
                       uint8_t moduleId, String cmdName);
    
    // ===== QUERY FUNCTIONS =====
    
    // Get last sent command name (for debugging)
    String getLastCommand();
    
    // Get last sent control mode (0=Voltage, 1=Torque, 2=Velocity, 3=Position)
    int getLastControlMode();
    
    // Get last sent input mode (0=Inactive, 1=Passthrough, 2=VelRamp, 3=PosFilter, 4=TrapTraj, 5/6=TorqueRamp)
    int getLastInputMode();
    
    // Get last sent velocity limit
    float getLastVelLimit();
    
    // Get last sent current limit
    float getLastCurrentLimit();
    
    // ===== UTILITY =====
    
    // Check if enough time has passed since last command
    bool canSendCommand();
    
    // Get milliseconds since last command
    unsigned long timeSinceLastCommand();
};

#endif
