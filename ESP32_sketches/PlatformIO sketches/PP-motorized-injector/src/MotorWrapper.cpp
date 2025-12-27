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
            motor.setLimits(vel_lim, current_lim);
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
        unsigned long now = millis();
        
        // First message: vel_limit (0x011)
        if ((now - lastCmdTime) >= CAN_COMMAND_GAP_MS) {
            motor.setTrajVelLimit(vel_limit);
            lastCmdTime = now;
            
            // Wait for gap before second message
            delay(CAN_COMMAND_GAP_MS + 5);  // Small safety margin
            
            // Second message: accel/decel (0x012)
            motor.setTrajAccelLimits(accel, decel);
            lastCmdTime = millis();
            lastCmdStr = "TrapParams:" + context;
            
            // Log for diagnostics
            char buf[128];
            snprintf(buf, sizeof(buf), 
                "SET_TRAP_TRAJ: vel=%.1f rps, accel=%.1f, decel=%.1f [%s]",
                vel_limit, accel, decel, context.c_str());
            MessageBuffer::getInstance().sendMessage(buf);
        }
    }
    
    // ===== EXECUTE MOTOR MOVE (Unified Wrapper) =====
    void setModeAndMove(CanBusHandlerV2& motor, int ctrlMode, int inputMode, float value, String cmdName) {
        unsigned long now = millis();
        
        // Enforce CAN command gap
        if ((now - lastCmdTime) >= CAN_COMMAND_GAP_MS) {
            // Set control mode and input mode
            motor.setControllerModes((ODriveCANProtocol::ControlMode)ctrlMode, 
                                     (ODriveCANProtocol::InputMode)inputMode);
            
            // Send appropriate setpoint based on control mode
            if (ctrlMode == 1) motor.setInputTorque(value);      // Torque mode
            else if (ctrlMode == 2) motor.setInputVel(value);    // Velocity mode
            else if (ctrlMode == 3) motor.setInputPos(value);    // Position mode
            
            lastCmdTime = now;
            lastCmdStr = cmdName;
            lastControlMode = ctrlMode;
            lastInputMode = inputMode;
            
            // Log for diagnostics
            char buf[128];
            snprintf(buf, sizeof(buf), "MOTOR_CMD: Mode=%d InputMode=%d Val=%.2f [%s]",
                ctrlMode, inputMode, value, cmdName.c_str());
            MessageBuffer::getInstance().sendMessage(buf);
        }
    }
    
    // ===== DYNAMIC ADJUSTMENT (Resend limits with new current) =====
    void adjustMotorLimits(CanBusHandlerV2& motor, float current_lim, String reason) {
        // Reuse setMotorLimits with last velocity limit
        setMotorLimits(motor, lastVelLimit, current_lim, reason);
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
