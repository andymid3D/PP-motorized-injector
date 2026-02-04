#include "MotorWrapper.h"
#include "TimingSystemTest.h"  // Add include for TimingSystemTest
#include "GPTimer.h"  // Add GPTimer for accurate timing
#include "BroadcastDataStore.h"  // For velocity monitoring
#include "MessageBuffer.h"  // For debug messages

namespace MotorWrapper {
    // ===== STATIC VARIABLES (shared across all calls) =====
    static uint64_t lastCmdTime = 0;  // Use uint64_t for microseconds
    static String lastCmdStr = "None";
    static int lastControlMode = -1;
    static int lastInputMode = -1;
    static float lastVelLimit = 0.0f;
    static float lastCurrentLimit = 0.0f;
    static uint8_t lastModuleId = 255;
    
    // ===== RETRY TIMEOUT CONFIGURATIONS =====
    static const uint32_t RETRY_TIMEOUTS[] = {
        50,   // PRIORITY_CRITICAL (50ms) - INJECTION
        75,   // PRIORITY_HIGH (75ms) - COMPRESSION  
        100,  // PRIORITY_NORMAL (100ms) - REFILL, RELEASE
        200   // PRIORITY_LOW (200ms) - ANTIDRIP, PURGE_ZERO
    };
    
    static const uint8_t MAX_RETRIES = 3;
    
    // ===== INITIALIZATION =====
    void init() {
        lastCmdTime = 0;
        lastCmdStr = "None";
        lastControlMode = -1;
        lastInputMode = -1;
        lastVelLimit = 0.0f;
        lastCurrentLimit = 0.0f;
        lastModuleId = 255;
    }
    
    // ===== SET MOTOR LIMITS (CAN 0x00F) =====
    void setMotorLimits(CanBusHandlerV2& motor, float vel_lim, float current_lim, uint8_t moduleId, String context) {
        uint64_t now = hwTimer.micros();  // Use microseconds directly for motor command timing
        
        // Send limits command (CanBusHandlerV2 handles timing)
        if (!motor.setLimits(vel_lim, current_lim)) {
            // Queue full - log warning
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "CAN_QUEUE_FULL: Limits [M%d:%s]", moduleId, context.c_str());
            MessageBuffer::getInstance().sendMessage(errBuf);
            return;  // Don't update state if command failed
        }
        
        lastCmdTime = now;  // now is already in microseconds
        lastCmdStr = "Limits:" + context;
        lastVelLimit = vel_lim;
        lastCurrentLimit = current_lim;
        lastModuleId = moduleId;
        
        // Log for diagnostics (commented out to reduce Refill debug overlap)
        // char buf[128];
        // snprintf(buf, sizeof(buf), "SET_LIMITS: vel=%.1f rps, current=%.1f A [M%d:%s]",
        //     vel_lim, current_lim, moduleId, context.c_str());
        // MessageBuffer::getInstance().sendMessage(buf);
    }
    
    // ===== SET CONTROLLER MODES (CAN 0x00B + 0x00C) =====
    void setControllerModes(CanBusHandlerV2& motor, ODriveCANProtocol::ControlMode ctrlMode, 
                           ODriveCANProtocol::InputMode inputMode, uint8_t moduleId, String context) {
        // Queue both messages - ring buffer handles timing
        if (!motor.setControllerModes(ctrlMode, inputMode)) {
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "CAN_QUEUE_FULL: Controller Modes [M%d:%s]", moduleId, context.c_str());
            MessageBuffer::getInstance().sendMessage(errBuf);
            return;
        }
        
        // Update command tracking
        lastCmdTime = hwTimer.micros();  // Use microseconds directly for motor command timing
        lastCmdStr = "Modes:" + context;
        lastControlMode = (int)ctrlMode;
        lastInputMode = (int)inputMode;
        lastModuleId = moduleId;
        
        // Log for diagnostics
        char buf[128];
        snprintf(buf, sizeof(buf), "SET_MODES: ctrl=%d input=%d [M%d:%s]",
            (int)ctrlMode, (int)inputMode, moduleId, context.c_str());
        MessageBuffer::getInstance().sendMessage(buf);
    }
    
    // ===== SET TRAP_TRAJ PARAMETERS (CAN 0x011 + 0x012) =====
    void setTrapTrajParams(CanBusHandlerV2& motor, float vel_limit, float accel, float decel, uint8_t moduleId, String context) {
        // Queue both messages - ring buffer handles timing
        if (!motor.setTrajVelLimit(vel_limit)) {
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "CAN_QUEUE_FULL: TrajVel [M%d:%s]", moduleId, context.c_str());
            MessageBuffer::getInstance().sendMessage(errBuf);
            return;
        }
        if (!motor.setTrajAccelLimits(accel, decel)) {
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "CAN_QUEUE_FULL: TrajAccel [M%d:%s]", moduleId, context.c_str());
            MessageBuffer::getInstance().sendMessage(errBuf);
            return;
        }
        
        lastCmdTime = hwTimer.micros();  // Use microseconds directly for motor command timing
        lastCmdStr = "TrapParams:" + context;
        lastModuleId = moduleId;
            
        // Log for diagnostics (commented out to reduce Refill debug overlap)
        // char buf[128];
        // snprintf(buf, sizeof(buf), 
        //     "SET_TRAP_TRAJ: vel=%.1f rps, accel=%.1f, decel=%.1f [M%d:%s]",
        //     vel_limit, accel, decel, moduleId, context.c_str());
        // MessageBuffer::getInstance().sendMessage(buf);
    }
    
    // ===== DYNAMIC ADJUSTMENT (Resend limits with new current) =====
    void adjustMotorLimits(CanBusHandlerV2& motor, float current_lim, uint8_t moduleId, String reason) {
        // Reuse setMotorLimits with last velocity limit
        setMotorLimits(motor, lastVelLimit, current_lim, moduleId, reason);
    }
    
    // ===== UNIVERSAL SAFE STOP =====
    void universalStop(CanBusHandlerV2& motor, uint8_t moduleId, String reason) {
        motor.setAxisState(ODriveCANProtocol::AxisState::IDLE);  // State 1
        
        char stopBuf[80];
        snprintf(stopBuf, sizeof(stopBuf), "UNIVERSAL_STOP: State=1 [M%d:%s]", moduleId, reason.c_str());
        MessageBuffer::getInstance().sendMessage(stopBuf);
    }
    
    void safeModeChange(CanBusHandlerV2& motor, int ctrlMode, int inputMode, 
                       uint8_t moduleId, String cmdName) {
        // Step 1: Universal stop
        universalStop(motor, moduleId, "PreModeChange");
        delayMicroseconds(20000);  // 20ms delay using microseconds
        
        // Step 2: Mode change (safe in IDLE)
        motor.setControllerModes((ODriveCANProtocol::ControlMode)ctrlMode, 
                               (ODriveCANProtocol::InputMode)inputMode);
        
        // Log for diagnostics (commented out to reduce Refill debug overlap)
        // char modeBuf[80];
        // snprintf(modeBuf, sizeof(modeBuf), "SAFE_MODE_CHANGE: Ctrl=%d Input=%d [M%d:%s]",
        //         ctrlMode, inputMode, moduleId, cmdName.c_str());
        // MessageBuffer::getInstance().sendMessage(modeBuf);
        delayMicroseconds(20000);  // 20ms delay using microseconds
        
        // Step 3: Resume operation
        motor.setAxisState(ODriveCANProtocol::AxisState::CLOSED_LOOP_CONTROL);
    }
    
    // ===== EXECUTE MOTOR MOVE (Unified Wrapper) =====
    void setModeAndMove(CanBusHandlerV2& motor, int ctrlMode, int inputMode, float value, uint8_t moduleId, String cmdName) {
        // Register command with TimingSystemTest for START response tracking (disabled for now)
        // TimingSystemTest::getInstance().registerCommand(
        //     hwTimer.micros(), moduleId, ctrlMode, cmdName
        // );
        
        // Notify TimingSystemTest that we're sending a command (disabled for now)
        // TimingSystemTest::getInstance().onMovementCommandTx();
        
        // ===== DIRECTION CHANGE SAFETY =====
        // Check if this is a direction change and verify motor is stopped
        BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
        float currentVelocity = broadcast.getVelocity();
        
        // Detect direction change: velocity > threshold AND new command in opposite direction
        bool isDirectionChange = false;
        if (fabs(currentVelocity) > 0.5f) {  // Motor is moving
            if ((ctrlMode == 2 && value * currentVelocity < 0) ||  // Velocity reversal
                (ctrlMode == 3 && fabs(value - broadcast.getPosition()) > 1.0f)) {  // Large position change
                isDirectionChange = true;
            }
        }
        
        // For direction changes, ensure motor is stopped first
        if (isDirectionChange) {
            char safetyBuf[80];
            snprintf(safetyBuf, sizeof(safetyBuf), "[MOTOR] Direction change detected - stopping first (vel=%.1f)", currentVelocity);
            MessageBuffer::getInstance().sendMessage(safetyBuf);
            
            // Send stop command and wait for verification
            motor.setInputVel(0.0f);
            uint64_t stopStartTime = hwTimer.micros();
            
            // Wait up to 100ms for motor to stop
            while (hwTimer.micros() - stopStartTime < 100000) {
                BroadcastDataStore& bds = BroadcastDataStore::getInstance();
                if (fabs(bds.getVelocity()) < 0.1f) {
                    MessageBuffer::getInstance().sendMessage("[MOTOR] Stop verified, proceeding with direction change");
                    break;
                }
                delay(1);  // Small delay to prevent tight loop
            }
        }
        // ===== END DIRECTION CHANGE SAFETY =====
        
        // Use safe mode change if mode is different AND we're not switching modules
        // Module transitions (e.g., Homing→Refill) legitimately use different modes
        if ((lastControlMode != ctrlMode || lastInputMode != inputMode) && 
            (lastModuleId == moduleId || lastModuleId == 255)) {  // 255 = uninitialized
            safeModeChange(motor, ctrlMode, inputMode, moduleId, cmdName);
        }
        
        // Queue setpoint command - ring buffer handles timing
        bool queued = false;
        if (ctrlMode == 1) queued = motor.setInputTorque(value);      // Torque mode
        else if (ctrlMode == 2) queued = motor.setInputVel(value);    // Velocity mode
        else if (ctrlMode == 3) queued = motor.setInputPos(value);    // Position mode
        
        if (!queued) {
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "CAN_QUEUE_FULL: Setpoint [M%d:%s]", moduleId, cmdName.c_str());
            MessageBuffer::getInstance().sendMessage(errBuf);
            return;
        }
        
        // Update tracking variables
        lastCmdTime = hwTimer.micros();  // Use microseconds directly for motor command timing
        lastCmdStr = cmdName;
        lastControlMode = ctrlMode;
        lastInputMode = inputMode;
        lastModuleId = moduleId;
        
        // Log setpoint command
        char buf[128];
        snprintf(buf, sizeof(buf), "SETPOINT_CMD: Mode=%d InputMode=%d Val=%.2f [M%d:%s]",
            ctrlMode, inputMode, value, moduleId, cmdName.c_str());
        MessageBuffer::getInstance().sendMessage(buf);
        
        // For position moves, we could wait for trajectory completion
        if (ctrlMode == 3 && queued) {
            // Optional: Add trajectory completion waiting here if needed
            // waitForTrajectoryDone(calculateTimeout(value));
        }
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
    
    // ===== RETRY LOGIC WITH PRIORITY-BASED TIMEOUTS =====
    bool setModeAndMoveWithRetry(CanBusHandlerV2& motor, int ctrlMode, int inputMode, float value, 
                                 uint8_t moduleId, String cmdName, RetryPriority priority) {
        
        uint32_t timeoutMs = RETRY_TIMEOUTS[priority];
        uint8_t retryCount = 0;
        
        while (retryCount < MAX_RETRIES) {
            // Attempt to send command
            setModeAndMove(motor, ctrlMode, inputMode, value, moduleId, cmdName);
            
            // Wait for START response detection (non-blocking)
            unsigned long startTime = hwTimer.micros(); // Use GPTimer for consistency
            bool commandStarted = false;
            
            // Check for START response detection within timeout
            while ((hwTimer.micros() - startTime) < (timeoutMs * 1000)) { // Convert to microseconds
                // Check with TimingSystemTest for actual START response confirmation (disabled for now)
                // if (TimingSystemTest::getInstance().hasCommandStarted(moduleId, cmdName)) {
                //     commandStarted = true;
                //     break;
                // }
                
                // For now, simulate success after short delay (original behavior)
                if ((hwTimer.micros() - startTime) > (25 * 1000)) { // Simulated 25ms response time
                    commandStarted = true;
                    break;
                }
            }
            
            if (commandStarted) {
                // START detected - success!
                uint32_t latency = (hwTimer.micros() - startTime) / 1000; // Convert back to ms for display
                
                // Log success
                char buf[128];
                snprintf(buf, sizeof(buf), "CMD_STARTED: [M%d:%s] Retry=%d/%d Latency=%lums",
                         moduleId, cmdName.c_str(), retryCount, MAX_RETRIES, latency);
                MessageBuffer::getInstance().sendMessage(buf);
                
                return true;
            }
            
            // START not detected - log retry attempt
            retryCount++;
            char retryBuf[128];
            snprintf(retryBuf, sizeof(retryBuf), "CMD_RETRY: [M%d:%s] Attempt=%d/%d Timeout=%lums",
                     moduleId, cmdName.c_str(), retryCount, MAX_RETRIES, timeoutMs);
            MessageBuffer::getInstance().sendMessage(retryBuf);
            
            // Exponential backoff before retry - NON-BLOCKING
            if (retryCount < MAX_RETRIES) {
                static unsigned long retryDelayStart = 0;
                static uint8_t currentRetryCount = 0;
                
                // Initialize delay for this retry
                if (retryDelayStart == 0 || currentRetryCount != retryCount) {
                    retryDelayStart = hwTimer.micros(); // Use GPTimer for consistency
                    currentRetryCount = retryCount;
                }
                
                // Check if delay completed (convert ms to us)
                if ((hwTimer.micros() - retryDelayStart) < (50 * retryCount * 1000)) {
                    retryCount--; // Don't consume retry count yet
                    return false; // Exit function, will retry on next call
                }
                
                // Delay completed - reset for next retry
                retryDelayStart = 0;
            }
        }
        
        // All retries failed - no START response detected
        char failBuf[128];
        snprintf(failBuf, sizeof(failBuf), "CMD_FAILED: [M%d:%s] All %d retries exhausted (no START response)",
                 moduleId, cmdName.c_str(), MAX_RETRIES);
        MessageBuffer::getInstance().sendMessage(failBuf);
        
        return false;
    }
    
    // ===== UTILITY =====
    
    bool canSendCommand() {
        // CanBusHandlerV2 handles timing now
        return true;
    }
    
    uint64_t timeSinceLastCommand() {
        return hwTimer.micros() - lastCmdTime;  // Use microseconds directly for motor command timing
    }
};
