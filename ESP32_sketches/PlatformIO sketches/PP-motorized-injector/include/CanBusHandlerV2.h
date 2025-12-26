#ifndef __CANBUS_HANDLER_V2_H__
#define __CANBUS_HANDLER_V2_H__

#include "ODriveCANProtocol.h"

/**
 * CAN Bus Abstraction Layer for ODrive
 * 
 * Responsibilities:
 * - Maintain ODrive state (position, velocity, axis state, errors)
 * - Parse incoming CAN messages (heartbeat, encoder estimates)
 * - Queue outgoing CAN commands (minimal sends, rate-limited 20ms)
 * - Verify ODrive is alive via heartbeat watchdog
 * - Handle CAN bus errors gracefully
 * 
 * PARAMETER BYTE LAYOUT REFERENCE:
 * Multi-parameter messages MUST send all parameters together
 * 
 * setControllerModes(ctrlMode, inputMode)
 *   Byte 0: control_mode (ControlMode: 0-3)
 *   Byte 4: input_mode (InputMode: 0, 1, 2, 3, 5, 6)
 *   → BOTH parameters required in single command
 * 
 * setInputPos(position)
 *   Byte 0-3: position (float32, turns)
 * 
 * setInputVel(velocity)
 *   Byte 0-3: velocity (float32, turns/second)
 * 
 * setInputTorque(torque)
 *   Byte 0-3: torque (float32, Nm or amps)
 * 
 * setLimits(velLimit, currentLimit)
 *   Byte 0-3: velocity_limit (float32, turns/second)
 *   Byte 4-7: current_limit (float32, amps)
 *   → BOTH parameters required in single command
 * 
 * setLinearCount(count)
 *   Byte 0-3: count (int32_t, typically 0 for reset)
 * 
 * setAxisState(state)
 *   Byte 0-3: axis_state (AxisState enum: 0-13)
 * 
 * clearErrors()
 *   No parameters (command-only message)
 */

class CanBusHandlerV2 {
public:
    // ===== Initialization =====
    
    void begin();  // Initialize CAN bus interface
    void loop();   // Call every main loop iteration to process RX/TX queue

    // ===== Feedback Data (from heartbeat + encoder broadcasts) =====
    
    float getPosition() const { return encoder_estimates_.position; }
    float getVelocity() const { return encoder_estimates_.velocity; }
    
    uint8_t getAxisState() const { return heartbeat_.axis_state; }
    uint32_t getAxisError() const { return heartbeat_.axis_error; }
    bool hasMotorError() const { return heartbeat_.motor_error_flag != 0; }
    bool hasEncoderError() const { return heartbeat_.encoder_error_flag != 0; }
    bool hasControllerError() const { return heartbeat_.controller_error_flag != 0; }
    bool isTrajectoryDone() const { return heartbeat_.trajectory_done_flag != 0; }
    
    /**
     * Check if ODrive is alive (heartbeat received recently)
     * @param timeoutMs Timeout in milliseconds (default 500ms)
     * @return true if heartbeat received within timeout
     */
    bool isAlive(uint32_t timeoutMs = 500) const;
    
    // Diagnostic: Get count of encoder estimates messages received
    uint32_t getEncoderEstimatesRxCount() const { return encoderEstimatesRxCount_; }
    
    // ===== CYCLIC MESSAGE DATA (all broadcasts received from ODrive) =====
    // These are updated whenever ODrive broadcasts the corresponding cyclic message
    // Most are disabled by default (0ms broadcast rate) - enable via ODrive config
    
    // Heartbeat (0x01, ~100ms): axis state, errors, flags
    const ODriveCANProtocol::CyclicHeartbeat& getHeartbeat() const { return heartbeat_; }
    
    // Encoder estimates (0x09, ~10ms): position + velocity
    const ODriveCANProtocol::CyclicEncoderEstimates& getEncoderEstimates() const { return encoder_estimates_; }
    
    // Motor error details (0x03, disabled by default)
    const ODriveCANProtocol::CyclicMotorError& getMotorErrorDetails() const { return motor_error_; }
    
    // Encoder error details (0x04, disabled by default)
    const ODriveCANProtocol::CyclicEncoderError& getEncoderErrorDetails() const { return encoder_error_; }
    
    // Sensorless error details (0x05, disabled by default)
    const ODriveCANProtocol::CyclicSensorlessError& getSensorlessErrorDetails() const { return sensorless_error_; }
    
    // Encoder count (0x0A, disabled by default)
    const ODriveCANProtocol::CyclicEncoderCount& getEncoderCount() const { return encoder_count_; }
    
    // Motor current Iq setpoint + measured (0x14, disabled by default)
    // Useful for current loop tuning
    const ODriveCANProtocol::CyclicIq& getIq() const { return Iq_; }
    
    // Sensorless position + velocity (0x15, disabled by default)
    // Alternative to encoder estimates if sensorless mode enabled
    const ODriveCANProtocol::CyclicSensorlessEstimates& getSensorlessEstimates() const { return sensorless_estimates_; }
    
    // Bus voltage + current (0x17, disabled by default)
    // Useful for power monitoring and supply diagnostics
    const ODriveCANProtocol::CyclicBusVoltageCurrent& getBusVoltageCurrent() const { return bus_vi_; }
    
    // Controller error details (0x1D, disabled by default)
    const ODriveCANProtocol::CyclicControllerError& getControllerErrorDetails() const { return controller_error_; }
    
    // ===== Commands (queue for transmission) =====
    // All commands queue for transmission and enforce rate limiting (20ms minimum between sends)
    // IMPORTANT: Commands that take multiple parameters MUST send all parameters together
    //            See ODriveCANProtocol.*Params structs for byte layouts
    
    /**
     * Set axis requested state (AxisState enum)
     * Examples: IDLE (1), CALIBRATION (3-6), CLOSED_LOOP_CONTROL (8)
     * @param state Target AxisState
     */
    void setAxisState(ODriveCANProtocol::AxisState state);
    
    /**
     * Set control mode and input mode together
     * CRITICAL: Both parameters MUST be sent in same command
     * @param ctrlMode ControlMode: 0=Voltage, 1=Torque, 2=Velocity, 3=Position
     * @param inputMode InputMode: 0=Inactive, 1=Passthrough, 2=VelRamp, 3=PosFilter, 5=TrapTraj, 6=TorqueRamp
     * 
     * Example: setControllerModes(ControlMode::POSITION_CONTROL, InputMode::TRAP_TRAJ)
     */
    void setControllerModes(ODriveCANProtocol::ControlMode ctrlMode, 
                           ODriveCANProtocol::InputMode inputMode);
    
    /**
     * Set position target
     * Byte 0-3: position (float32)
     * @param position Target position in turns
     * 
     * Example: setInputPos(10.5f) → move to 10.5 turns
     */
    void setInputPos(float position);
    
    /**
     * Set velocity target
     * Byte 0-3: velocity (float32)
     * @param velocity Target velocity in turns/second
     * 
     * Example: setInputVel(5.0f) → move at 5 turns/sec
     */
    void setInputVel(float velocity);
    
    /**
     * Set torque target (current control)
     * Byte 0-3: torque (float32)
     * @param torque Target torque in Nm (or amps if using current control)
     * 
     * Example: setInputTorque(2.5f) → apply 2.5 Nm torque
     */
    void setInputTorque(float torque);
    
    /**
     * Set velocity and current limits
     * CRITICAL: Both parameters MUST be sent in same command
     * Byte 0-3: velocity_limit (float32)
     * Byte 4-7: current_limit (float32)
     * @param velLimit Velocity limit in turns/second
     * @param currentLimit Current limit in amps
     * 
     * Example: setLimits(50.0f, 10.0f) → max 50 turns/sec, max 10 amps
     */
    void setLimits(float velLimit, float currentLimit);
    
    /**
     * Reset encoder to zero (or set to arbitrary count)
     * Sets the encoder count value (ODrive's internal step counter)
     * Byte 0-3: count (int32_t)
     * @param count Encoder count value in steps (typically 0 to reset to zero)
     * 
     * IMPORTANT: This is the encoder COUNT (in ODrive's step units), NOT position in turns.
     * ODrive encoder CPR = 8192 counts per turn.
     * If you need to set position in turns: count = turns * 8192 (rounded to int32_t)
     * 
     * Current usage: setLinearCount(0) to reset encoder to zero after homing
     * 
     * Example: 
     *   setLinearCount(0) → reset to zero turns
     *   setLinearCount(8192) → set to 1.0 turn
     *   setLinearCount(16384) → set to 2.0 turns
     */
    void setLinearCount(int32_t count);
    
    /**
     * Clear all ODrive errors
     * Command-only, no parameters required
     * 
     * Use when recovering from ERROR_STATE after fixing root cause
     */
    void clearErrors();
    
    // ===== DIAGNOSTIC & CONTROL COMMANDS (RTR - Remote Transfer Request) =====
    // These request data from ODrive or perform advanced control
    // Not rate-limited (requests are typically infrequent)
    
    /**
     * Request immediate heartbeat from ODrive
     * Forces ODrive to send heartbeat immediately (not just ~100ms interval)
     */
    void heartbeatRequest();
    
    /**
     * Emergency stop - cuts motor PWM immediately
     * Motor coasts to a stop
     * Clears any pending commands
     */
    void estop();
    
    /**
     * Request motor error flags from ODrive (diagnostic)
     * ODrive will respond with motor error value via CAN
     */
    void getMotorError();
    
    /**
     * Request encoder error flags from ODrive (diagnostic)
     * ODrive will respond with encoder error value via CAN
     */
    void getEncoderError();
    
    /**
     * Request sensorless estimator error from ODrive (diagnostic)
     * ODrive will respond with sensorless error value via CAN
     */
    void getSensorlessError();
    
    /**
     * Change the CAN node ID of this axis
     * WARNING: After calling, axis must be re-discovered at new address
     * @param newNodeId New CAN node ID (0-63)
     */
    void setAxisNodeId(uint32_t newNodeId);
    
    /**
     * Request current encoder count from ODrive (diagnostic)
     * ODrive will respond with encoder count value via CAN
     */
    void getEncoderCount();
    
    /**
     * Start anticogging calibration procedure
     * Motor will move in characteristic pattern to build anticogging map
     * May take several seconds
     */
    void startAnticogging();
    
    /**
     * Set trajectory planner velocity limit
     * Used with setControllerModes(..., InputMode::TRAP_TRAJ)
     * @param trajVelLimit Max trajectory velocity (turns/second)
     * 
     * Example: setTrajVelLimit(20.0f) → limit to 20 turns/sec
     */
    void setTrajVelLimit(float trajVelLimit);
    
    /**
     * Set trajectory planner acceleration and deceleration limits
     * CRITICAL: Both parameters MUST be sent in same command
     * Used with setControllerModes(..., InputMode::TRAP_TRAJ)
     * Byte 0-3: accel_limit (float32, turns/sec²)
     * Byte 4-7: decel_limit (float32, turns/sec²)
     * @param accelLimit Max acceleration (turns/sec²)
     * @param decelLimit Max deceleration (turns/sec²)
     * 
     * Example: setTrajAccelLimits(100.0f, 100.0f) → symmetric accel/decel
     */
    void setTrajAccelLimits(float accelLimit, float decelLimit);
    
    /**
     * Set trajectory planner feedforward inertia
     * Improves trajectory tracking accuracy
     * @param inertia Feedforward inertia value
     * 
     * Example: setTrajInertia(0.0f) → no feedforward
     */
    void setTrajInertia(float inertia);
    
    /**
     * Get current Iq (current) setpoint and measured values
     * Read from cyclic CYCLIC_IQ (0x14) broadcast (~100ms if enabled)
     * Useful for current loop tuning and diagnostics
     */
    const ODriveCANProtocol::CyclicIq& getIqReadings() const { return Iq_; }
    
    /**
     * Get sensorless estimator position and velocity
     * Read from cyclic CYCLIC_SENSORLESS_ESTIMATES (0x15) broadcast (~100ms if enabled)
     * Alternative to encoder estimates if sensorless mode enabled
     */
    const ODriveCANProtocol::CyclicSensorlessEstimates& getSensorlessEstimatesReadings() const { return sensorless_estimates_; }
    
    /**
     * Reboot the entire ODrive controller
     * Motor will stop, all state reset
     * CAN communication will be lost briefly
     */
    void reboot();
    
    /**
     * Request bus voltage and current measurements
     * Read from cyclic CYCLIC_BUS_VI (0x17) broadcast (~100ms if enabled)
     * Useful for power monitoring and supply diagnostics
     */
    const ODriveCANProtocol::CyclicBusVoltageCurrent& getBusVoltageCurrentReadings() const { return bus_vi_; }
    
    /**
     * Set position controller proportional gain
     * Affects responsiveness of position loop
     * Byte 0-3: pos_gain (float32, 1/sec)
     * @param posGain Position loop proportional gain (typical 5-20)
     * 
     * Example: setPositionGain(10.0f) → moderate responsiveness
     */
    void setPositionGain(float posGain);
    
    /**
     * Set velocity controller proportional and integrator gains
     * CRITICAL: Both parameters MUST be sent in same command
     * Used to tune velocity loop response
     * Byte 0-3: vel_gain (float32, Nm·sec/turn)
     * Byte 4-7: vel_integrator_gain (float32, Nm·sec²/turn)
     * @param velGain Velocity loop proportional gain (typical 0.1-1.0)
     * @param velIntegratorGain Velocity loop integrator gain (typical 0.01-0.1)
     * 
     * Example: setVelGains(0.5f, 0.05f) → tuned velocity loop
     */
    void setVelGains(float velGain, float velIntegratorGain);
    
    /**
     * Request ADC voltage measurement (diagnostic)
     * Requires GPIO pin configured for analog input
     * ODrive will respond with adc_voltage (float32)
     * Useful for external sensor monitoring (temperature, pressure, etc.)
     */
    void getAdcVoltage();
    
    /**
     * Request controller error flags from ODrive (diagnostic)
     * ODrive will respond with controller_error value
     * Provides detailed error state for troubleshooting
     */
    void getControllerError();
    
    // ===== CAN Message Processing (called by CAN interrupt handler) =====
    
    /**
     * Call this from CAN RX interrupt when a message arrives
     * @param msg The CAN message received
     */
    void onCanMessageReceived(const can_Message_t& msg);

private:
    static constexpr uint8_t NODE_ID = 0;  // ODrive node ID
    
    // Helper: Queue command respecting CAN_COMMAND_GAP_MS timing
    void _queueCommand(const can_Message_t& cmd);
    
    // Current ODrive state (updated by CAN RX)
    // Core messages (always processed)
    ODriveCANProtocol::CyclicHeartbeat heartbeat_;
    ODriveCANProtocol::CyclicEncoderEstimates encoder_estimates_;
    
    // Diagnostic/error messages (most disabled by default in ODrive config)
    ODriveCANProtocol::CyclicMotorError motor_error_;
    ODriveCANProtocol::CyclicEncoderError encoder_error_;
    ODriveCANProtocol::CyclicSensorlessError sensorless_error_;
    ODriveCANProtocol::CyclicEncoderCount encoder_count_;
    ODriveCANProtocol::CyclicIq Iq_;
    ODriveCANProtocol::CyclicSensorlessEstimates sensorless_estimates_;
    ODriveCANProtocol::CyclicBusVoltageCurrent bus_vi_;
    ODriveCANProtocol::CyclicControllerError controller_error_;
    
    uint32_t lastHeartbeatTime_ = 0;
    
    // Diagnostic counters (for debugging CAN message reception)
    uint32_t encoderEstimatesRxCount_ = 0;  // Count of 0x09 messages received
    
    // Rate limiting for command sends (20ms minimum between sends)
    uint32_t lastCommandTime_ = 0;
    static constexpr uint32_t CMD_RATE_LIMIT_MS = 20;
    
    // Track when last command was SENT (not queued)
    // Used to enforce CAN_COMMAND_GAP_MS between successive sends
    uint32_t lastCommandSentTime_ = 0;
    
    // Command queue (one pending at a time for now)
    can_Message_t pendingCommand_;
    bool hasPendingCommand_ = false;
};

#endif // __CANBUS_HANDLER_V2_H__
