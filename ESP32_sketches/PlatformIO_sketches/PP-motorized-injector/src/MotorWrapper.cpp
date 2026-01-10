#include "MotorWrapper.h"

namespace MotorWrapper {
    // ===== STATIC VARIABLES (shared across all calls) =====
    static unsigned long lastCmdTime = 0;
    static String lastCmdStr = "None";
    static int lastControlMode = -1;
    static int lastInputMode = -1;
    static float lastVelLimit = 0.0f;
    static float lastCurrentLimit = 0.0f;
    
    // ===== INITIALIZATION =====
    void init() {
        lastCmdTime = 0;
        lastCmdStr = "None";
        lastControlMode = -1;
        lastInputMode = -1;
        lastVelLimit = 0.0f;
        lastCurrentLimit = 0.0f;
    }
    
    // ===== SET MOTOR LIMITS (CAN 0x00F) =====
    void setMotorLimits(CanBusHandlerV2& motor, float vel_lim, float current_lim, String context) {
        unsigned long now = millis();
        
        // Enforce CAN command gap
        if ((now - lastCmdTime) >= CAN_COMMAND_GAP_MS) {
            if (!motor.setLimits(vel_lim, current_lim)) {
                // Queue full - log warning
                char errBuf[64];
                snprintf(errBuf, sizeof(errBuf), "CAN_QUEUE_FULL: Limits [%s]", context.c_str());
                MessageBuffer::getInstance().sendMessage(errBuf);
                return;  // Don't update state if command failed
            }
            
            lastCmdTime = now;
            lastCmdStr = "Limits:" + context;
            lastVelLimit = vel_lim;
            lastCurrentLimit = current_lim;
            
            // Log for diagnostics
            char buf[128];
            snprintf(buf, sizeof(buf), "SET_LIMITS: vel=%.1f rps, current=%.1f A [%s]",
                vel_lim, current_lim, context.c_str());
            MessageBuffer::getInstance().sendMessage(buf);
        }
    }
    
    // ===== SET TRAP_TRAJ PARAMETERS (CAN 0x011 + 0x012) =====
    void setTrapTrajParams(CanBusHandlerV2& motor, float vel_limit, float accel, float decel, String context) {
        // Queue both messages - ring buffer handles timing
        if (!motor.setTrajVelLimit(vel_limit)) {
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "CAN_QUEUE_FULL: TrajVel [%s]", context.c_str());
            MessageBuffer::getInstance().sendMessage(errBuf);
            return;
        }
        if (!motor.setTrajAccelLimits(accel, decel)) {
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "CAN_QUEUE_FULL: TrajAccel [%s]", context.c_str());
            MessageBuffer::getInstance().sendMessage(errBuf);
            return;
        }
        
        lastCmdTime = millis();
        lastCmdStr = "TrapParams:" + context;
            
        // Log for diagnostics
        char buf[128];
        snprintf(buf, sizeof(buf), 
            "SET_TRAP_TRAJ: vel=%.1f rps, accel=%.1f, decel=%.1f [%s]",
            vel_limit, accel, decel, context.c_str());
        MessageBuffer::getInstance().sendMessage(buf);
    }
    
    // ===== DYNAMIC ADJUSTMENT (Resend limits with new current) =====
    void adjustMotorLimits(CanBusHandlerV2& motor, float current_lim, String reason) {
        // Reuse setMotorLimits with last velocity limit
        setMotorLimits(motor, lastVelLimit, current_lim, reason);
    }
    
    // ===== EXECUTE MOTOR MOVE (Unified Wrapper) =====
    void setModeAndMove(CanBusHandlerV2& motor, int ctrlMode, int inputMode, float value, String cmdName) {
        // Queue mode command - ring buffer handles timing
        if (!motor.setControllerModes((ODriveCANProtocol::ControlMode)ctrlMode, 
                                      (ODriveCANProtocol::InputMode)inputMode)) {
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "CAN_QUEUE_FULL: Mode [%s]", cmdName.c_str());
            MessageBuffer::getInstance().sendMessage(errBuf);
            return;
        }
        
        char modeBuf[80];
        snprintf(modeBuf, sizeof(modeBuf), "MODE_CMD: Ctrl=%d Input=%d [%s]",
            ctrlMode, inputMode, cmdName.c_str());
        MessageBuffer::getInstance().sendMessage(modeBuf);
        
        // Queue setpoint command - ring buffer handles timing
        bool queued = false;
        if (ctrlMode == 1) queued = motor.setInputTorque(value);      // Torque mode
        else if (ctrlMode == 2) queued = motor.setInputVel(value);    // Velocity mode
        else if (ctrlMode == 3) queued = motor.setInputPos(value);    // Position mode
        
        if (!queued) {
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "CAN_QUEUE_FULL: Setpoint [%s]", cmdName.c_str());
            MessageBuffer::getInstance().sendMessage(errBuf);
            return;
        }
        
        lastCmdTime = millis();
        lastCmdStr = cmdName;
        lastControlMode = ctrlMode;
        lastInputMode = inputMode;
        
        // Log setpoint command
        char buf[128];
        snprintf(buf, sizeof(buf), "SETPOINT_CMD: Mode=%d InputMode=%d Val=%.2f [%s]",
            ctrlMode, inputMode, value, cmdName.c_str());
        MessageBuffer::getInstance().sendMessage(buf);
    }
    
    // ===== QUERY FUNCTIONS =====
    
    String getLastCommand() {
        return lastCmdStr;
    }
    
    int getLastControlMode() {
        return lastControlMode;
    }
    
    int getLastInputMode() {
        return lastInputMode;
    }
    
    float getLastVelLimit() {
        return lastVelLimit;
    }
    
    float getLastCurrentLimit() {
        return lastCurrentLimit;
    }
    
    // ===== UTILITY =====
    
    bool canSendCommand() {
        return (millis() - lastCmdTime) >= CAN_COMMAND_GAP_MS;
    }
    
    unsigned long timeSinceLastCommand() {
        return millis() - lastCmdTime;
    }
};
