/**
 * ODrive CANSimple Protocol Wrapper (ODrive 0.5.6 Firmware)
 * 
 * SOURCE OF TRUTH:
 * - Message IDs: https://docs.odriverobotics.com/v/0.5.6/can-protocol.html#messages
 * - Cyclic Messages: https://docs.odriverobotics.com/v/0.5.6/can-protocol.html#cyclic-messages  
 * - AxisState enum: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState
 * - ControlMode enum: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode
 * - InputMode enum: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode
 * 
 * NOTE: Cyclic broadcast message IDs have 1 less hex digit than request/response message IDs
 *       e.g., MSG_HEARTBEAT request is 0x001, but cyclic broadcast is 0x01
 *       This is critical for correct CAN frame parsing.
 */

#ifndef __ODRIVE_CAN_PROTOCOL_H__
#define __ODRIVE_CAN_PROTOCOL_H__

#include "can_helpers.hpp"

/**
 * Lean ODrive CANSimple Protocol Wrapper
 * Handles CAN communication with ODrive 3.6 on a single node (Node ID 0)
 * 
 * Key features:
 * - Parse incoming heartbeat messages (axis state, errors every 100ms)
 * - Parse incoming encoder estimate messages (position, velocity every 10ms)
 * - Send move commands (position, velocity, torque)
 * - Send control mode changes
 * - Send limits (velocity, current)
 * - Single node only (Node ID 0 hardcoded)
 * 
 * Source: https://github.com/odriverobotics/ODrive/blob/master/Firmware/communication/can/can_simple.hpp
 */

class ODriveCANProtocol {
public:
    // ===== REQUEST/RESPONSE MESSAGE IDs (from #messages table) =====
    // These are command IDs we CAN SEND to the ODrive (or request responses with RTR bit)
    // NO CORRESPONDENCE to cyclic message IDs - different table entirely
    // Format when sending: (NodeID << 5) | RequestMessageID
    enum RequestMessageID : uint32_t {
        MSG_HEARTBEAT_REQUEST       = 0x001,  // Request heartbeat (RTR)
        MSG_ESTOP                   = 0x002,  // Emergency stop
        MSG_GET_MOTOR_ERROR         = 0x003,  // Request motor error (RTR)
        MSG_GET_ENCODER_ERROR       = 0x004,  // Request encoder error (RTR)
        MSG_GET_SENSORLESS_ERROR    = 0x005,  // Request sensorless error (RTR)
        MSG_SET_AXIS_NODE_ID        = 0x006,  // Set axis CAN node ID
        MSG_SET_AXIS_STATE          = 0x007,  // Set axis requested state
        MSG_SET_AXIS_STARTUP_CONFIG = 0x008,  // Not yet implemented
        MSG_GET_ENCODER_ESTIMATES   = 0x009,  // Request encoder estimates (RTR)
        MSG_GET_ENCODER_COUNT       = 0x00A,  // Request encoder count (RTR)
        MSG_SET_CONTROLLER_MODES    = 0x00B,  // Set control_mode + input_mode
        MSG_SET_INPUT_POS           = 0x00C,  // Set position target
        MSG_SET_INPUT_VEL           = 0x00D,  // Set velocity target
        MSG_SET_INPUT_TORQUE        = 0x00E,  // Set torque target
        MSG_SET_LIMITS              = 0x00F,  // Set velocity limit + current limit
        MSG_START_ANTICOGGING       = 0x010,  // Start anticogging calibration
        MSG_SET_TRAJ_VEL_LIMIT      = 0x011,  // Set trajectory velocity limit
        MSG_SET_TRAJ_ACCEL_LIMITS   = 0x012,  // Set trajectory acceleration + deceleration limits
        MSG_SET_TRAJ_INERTIA        = 0x013,  // Set trajectory inertia
        MSG_GET_IQ                  = 0x014,  // Request Iq setpoint + measured (RTR)
        MSG_GET_SENSORLESS_ESTIMATES= 0x015,  // Request sensorless estimates (RTR)
        MSG_REBOOT                  = 0x016,  // Reboot ODrive
        MSG_GET_BUS_VOLTAGE_CURRENT = 0x017,  // Request bus voltage + current (RTR)
        MSG_CLEAR_ERRORS            = 0x018,  // Clear all errors
        MSG_SET_LINEAR_COUNT        = 0x019,  // Set encoder linear count / reset encoder
        MSG_SET_POSITION_GAIN       = 0x01A,  // Set position loop gain
        MSG_SET_VEL_GAINS           = 0x01B,  // Set velocity loop gains
        MSG_GET_ADC_VOLTAGE         = 0x01C,  // Get ADC voltage (RTR, requires GPIO pin)
        MSG_GET_CONTROLLER_ERROR    = 0x01D,  // Request controller error (RTR)
    };

    // ===== CYCLIC/BROADCAST MESSAGE IDs (from #cyclic-messages table) =====
    // These are command IDs AUTOMATICALLY BROADCAST by ODrive axis on a timer (we RECEIVE only)
    // NO CORRESPONDENCE to request/response message IDs - separate table entirely
    // Format when receiving: (NodeID << 5) | CyclicMessageID
    // NOTE: Cyclic message IDs are SINGLE HEX DIGIT (0x01, 0x03, etc.), NOT 0x001, 0x003
    enum CyclicMessageID : uint32_t {
        CYCLIC_HEARTBEAT            = 0x01,   // Axis broadcasts: error, state, flags (100ms default)
        CYCLIC_MOTOR_ERROR          = 0x03,   // Axis broadcasts: motor error (0ms default = disabled)
        CYCLIC_ENCODER_ERROR        = 0x04,   // Axis broadcasts: encoder error (0ms default = disabled)
        CYCLIC_SENSORLESS_ERROR     = 0x05,   // Axis broadcasts: sensorless error (0ms default = disabled)
        CYCLIC_ENCODER_ESTIMATES    = 0x09,   // Axis broadcasts: position + velocity (10ms default)
        CYCLIC_ENCODER_COUNT        = 0x0A,   // Axis broadcasts: encoder count (0ms default = disabled)
        CYCLIC_IQ                   = 0x14,   // Axis broadcasts: Iq setpoint + measured (0ms default = disabled)
        CYCLIC_SENSORLESS_ESTIMATES = 0x15,   // Axis broadcasts: sensorless estimates (0ms default = disabled)
        CYCLIC_BUS_VI               = 0x17,   // Axis broadcasts: bus voltage + current (0ms default = disabled)
        CYCLIC_CONTROLLER_ERROR     = 0x1D,   // Axis broadcasts: controller error (0ms default = disabled)
    };


    // ===== AXIS STATE (official enum from docs) =====
    enum class AxisState : uint8_t {
        UNDEFINED                   = 0,   // Will fall through to idle
        IDLE                        = 1,   // Disable motor PWM
        STARTUP_SEQUENCE            = 2,   // Run startup procedure
        FULL_CALIBRATION_SEQUENCE   = 3,   // Motor calib + encoder offset calib
        MOTOR_CALIBRATION           = 4,   // Measure phase resistance/inductance
        ENCODER_INDEX_SEARCH        = 6,   // Find encoder index (requires encoder.config.use_index)
        ENCODER_OFFSET_CALIBRATION  = 7,   // Calibrate encoder offset vs electrical phase
        CLOSED_LOOP_CONTROL         = 8,   // Closed loop control (requires motor calib + encoder ready)
        LOCKIN_SPIN                 = 9,   // Lockin spin
        ENCODER_DIR_FIND            = 10,  // Encoder direction search
        HOMING                      = 11,  // Axis homing (requires endstops enabled)
        ENCODER_HALL_POLARITY_CALIB = 12,  // Hall sensor polarity calibration
        ENCODER_HALL_PHASE_CALIB    = 13,  // Hall sensor phase calibration
    };

    // ===== CONTROL MODE (official enum from docs) =====
    // Used with MSG_SET_CONTROLLER_MODES
    enum class ControlMode : int32_t {
        VOLTAGE_CONTROL             = 0,   // Not used internally
        TORQUE_CONTROL              = 1,   // Inner torque loop; use input_torque
        VELOCITY_CONTROL            = 2,   // Inner torque + velocity loops; use input_vel + optional input_torque
        POSITION_CONTROL            = 3,   // All three loops; use input_pos + optional input_vel + input_torque
    };

    // ===== INPUT MODE (official enum from docs) =====
    // https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode
    // Used with MSG_SET_CONTROLLER_MODES
    // Each mode modifies input_xxx → xxx_setpoint differently
    enum class InputMode : int32_t {
        INACTIVE                    = 0,   // Disable inputs; setpoints retain last value
        PASSTHROUGH                 = 1,   // Direct passthrough: input_xxx → xxx_setpoint (all control modes)
        VEL_RAMP                    = 2,   // Velocity ramp (config.vel_ramp_rate); requires VELOCITY_CONTROL
        POS_FILTER                  = 3,   // 2nd order position filter (config.input_filter_bandwidth); requires POSITION_CONTROL
        TRAP_TRAJ                   = 5,   // Trapezoidal trajectory planner; requires POSITION_CONTROL; config.trap_traj.*
        TORQUE_RAMP                 = 6,   // Torque ramp (config.torque_ramp_rate); requires TORQUE_CONTROL
    };

    // ===== RECEIVED DATA STRUCTURES =====

    /** Heartbeat message (0x001 cyclic broadcast ~100ms) */
    struct Heartbeat {
        uint32_t axis_error = 0;           // Axis error flags (bits 0-31)
        uint8_t axis_state = 0;            // AxisState enum (bits 32-39)
        uint8_t motor_error_flag = 0;      // Motor error flag (bit 40)
        uint8_t encoder_error_flag = 0;    // Encoder error flag (bit 41)
        uint8_t controller_error_flag = 0; // Controller error flag (bit 42)
        uint8_t trajectory_done_flag = 0;  // Trajectory done (bit 43, TRAP_TRAJ mode)
    };

    /** Encoder estimate message (0x009 cyclic broadcast ~10ms) */
    struct EncoderEstimate {
        float position = 0.0f;   // Linear position (turns)
        float velocity = 0.0f;   // Linear velocity (turns/second)
    };

    // ===== CYCLIC MESSAGE DATA STRUCTURES =====
    // These describe data received from ODrive broadcasts (we RECEIVE only)
    // Parse these when receiving cyclic messages

    /** Heartbeat message (0x01 cyclic broadcast ~100ms)
     * Contains axis error, state, and status flags
     * Sent automatically every 100ms
     */
    struct CyclicHeartbeat {
        uint32_t axis_error = 0;           // Axis error flags (bits 0-31)
        uint8_t axis_state = 0;            // AxisState enum (bits 32-39)
        uint8_t motor_error_flag = 0;      // Motor error flag (bit 40)
        uint8_t encoder_error_flag = 0;    // Encoder error flag (bit 41)
        uint8_t controller_error_flag = 0; // Controller error flag (bit 42)
        uint8_t trajectory_done_flag = 0;  // Trajectory done (bit 43, TRAP_TRAJ mode)
    };

    /** Motor error message (0x03 cyclic broadcast ~disabled by default)
     * Contains detailed motor error flags
     * Byte 0-3: motor_error (uint32_t)
     */
    struct CyclicMotorError {
        uint32_t motor_error = 0;          // Motor error flags
    };

    /** Encoder error message (0x04 cyclic broadcast ~disabled by default)
     * Contains detailed encoder error flags
     * Byte 0-3: encoder_error (uint32_t)
     */
    struct CyclicEncoderError {
        uint32_t encoder_error = 0;        // Encoder error flags
    };

    /** Sensorless error message (0x05 cyclic broadcast ~disabled by default)
     * Contains detailed sensorless estimator error flags
     * Byte 0-3: sensorless_error (uint32_t)
     */
    struct CyclicSensorlessError {
        uint32_t sensorless_error = 0;     // Sensorless error flags
    };

    /** Encoder estimates message (0x09 cyclic broadcast ~10ms)
     * Contains position and velocity from encoder
     * Byte 0-3: position (float32, turns)
     * Byte 4-7: velocity (float32, turns/second)
     * Sent automatically every 10ms
     */
    struct CyclicEncoderEstimates {
        float position = 0.0f;             // Position (turns)
        float velocity = 0.0f;             // Velocity (turns/second)
    };

    /** Encoder count message (0x0A cyclic broadcast ~disabled by default)
     * Contains current encoder count value
     * Byte 0-3: encoder_count (int32_t)
     */
    struct CyclicEncoderCount {
        int32_t encoder_count = 0;         // Encoder count value
    };

    /** Current (Iq) message (0x14 cyclic broadcast ~disabled by default)
     * Contains motor current setpoint and measured values
     * Byte 0-3: Iq_setpoint (float32, amps)
     * Byte 4-7: Iq_measured (float32, amps)
     * Useful for current loop tuning and diagnostics
     */
    struct CyclicIq {
        float Iq_setpoint = 0.0f;          // Current setpoint (amps)
        float Iq_measured = 0.0f;          // Measured motor current (amps)
    };

    /** Sensorless estimates message (0x15 cyclic broadcast ~disabled by default)
     * Contains position and velocity from sensorless estimator
     * Byte 0-3: position (float32, turns)
     * Byte 4-7: velocity (float32, turns/second)
     * Alternative to encoder estimates if sensorless mode enabled
     */
    struct CyclicSensorlessEstimates {
        float position = 0.0f;             // Estimated position (turns)
        float velocity = 0.0f;             // Estimated velocity (turns/second)
    };

    /** Bus voltage and current message (0x17 cyclic broadcast ~disabled by default)
     * Contains power supply voltage and current measurements
     * Byte 0-3: bus_voltage (float32, volts)
     * Byte 4-7: bus_current (float32, amps)
     * Useful for power monitoring and supply diagnostics
     */
    struct CyclicBusVoltageCurrent {
        float bus_voltage = 0.0f;          // Bus voltage (volts)
        float bus_current = 0.0f;          // Bus current (amps)
    };

    /** Controller error message (0x1D cyclic broadcast ~disabled by default)
     * Contains detailed controller error flags
     * Byte 0-3: controller_error (uint32_t)
     */
    struct CyclicControllerError {
        uint32_t controller_error = 0;     // Controller error flags
    };

    // ===== COMMAND PARAMETER STRUCTURES =====
    // Document exact byte layout and parameter groupings for multi-parameter messages
    // These enforce correct usage when constructing CAN frames

    /** SET_CONTROLLER_MODES (0x00B) parameters
     * CRITICAL: Both control_mode AND input_mode MUST be sent together
     * Byte 0: control_mode (ControlMode enum: 0-3)
     * Byte 4: input_mode (InputMode enum: 0, 1, 2, 3, 5, 6)
     */
    struct SetControllerModesParams {
        ControlMode control_mode;   // Byte 0: 0=Voltage, 1=Torque, 2=Velocity, 3=Position
        InputMode input_mode;       // Byte 4: Input filter/trajectory mode
        
        // Constructor to enforce both parameters supplied
        SetControllerModesParams(ControlMode ctrl, InputMode input) 
            : control_mode(ctrl), input_mode(input) {}
    };

    /** SET_INPUT_POS (0x00C) parameters
     * Byte 0-3: position (float32)
     * Optional parameters (can be added to extend):
     * - Byte 4-7: velocity_feedforward (float32)
     * - Byte 8-9: current_feedforward (int16_t) - but frame max 8 bytes
     */
    struct SetInputPosParams {
        float position;             // Byte 0-3: Target position (turns)
        
        SetInputPosParams(float pos) : position(pos) {}
    };

    /** SET_INPUT_VEL (0x00D) parameters
     * Byte 0-3: velocity (float32)
     * Optional:
     * - Byte 4-7: torque_feedforward (float32) - but frame max 8 bytes
     */
    struct SetInputVelParams {
        float velocity;             // Byte 0-3: Target velocity (turns/second)
        
        SetInputVelParams(float vel) : velocity(vel) {}
    };

    /** SET_INPUT_TORQUE (0x00E) parameters
     * Byte 0-3: torque (float32)
     */
    struct SetInputTorqueParams {
        float torque;               // Byte 0-3: Target torque (Nm)
        
        SetInputTorqueParams(float t) : torque(t) {}
    };

    /** SET_LIMITS (0x00F) parameters
     * Byte 0-3: velocity_limit (float32)
     * Byte 4-7: current_limit (float32)
     * CRITICAL: Both parameters MUST be sent together
     */
    struct SetLimitsParams {
        float velocity_limit;       // Byte 0-3: Velocity limit (turns/second)
        float current_limit;        // Byte 4-7: Current limit (amps)
        
        SetLimitsParams(float vel_lim, float curr_lim) 
            : velocity_limit(vel_lim), current_limit(curr_lim) {}
    };

    /** SET_AXIS_STATE (0x007) parameters
     * Byte 0-3: axis_state (AxisState enum)
     */
    struct SetAxisStateParams {
        AxisState state;            // Byte 0-3: Target axis state
        
        SetAxisStateParams(AxisState s) : state(s) {}
    };

    /** SET_LINEAR_COUNT (0x019) parameters
     * Byte 0-3: count (int32_t)
     * Used to reset encoder to zero (count=0)
     */
    struct SetLinearCountParams {
        int32_t count;              // Byte 0-3: Linear count (typically 0 for reset)
        
        SetLinearCountParams(int32_t c) : count(c) {}
    };

    /** CLEAR_ERRORS (0x018) parameters
     * No data payload - command-only message
     */
    struct ClearErrorsParams {
        // No parameters required
    };

    // ===== REQUEST-ONLY MESSAGES (RTR - Remote Transfer Request) =====
    // These request data from ODrive; ODrive responds with data message
    
    /** HEARTBEAT_REQUEST (0x001) parameters
     * Requests immediate heartbeat response
     * No parameters required
     */
    struct HeartbeatRequestParams {
        // No parameters required
    };

    /** ESTOP (0x002) parameters
     * Emergency stop - cuts motor PWM immediately
     * No parameters required
     */
    struct EstopParams {
        // No parameters required
    };

    /** GET_MOTOR_ERROR (0x003) parameters
     * Request motor error flags (RTR)
     * ODrive responds with motor error value
     * No parameters required
     */
    struct GetMotorErrorParams {
        // No parameters required
    };

    /** GET_ENCODER_ERROR (0x004) parameters
     * Request encoder error flags (RTR)
     * ODrive responds with encoder error value
     * No parameters required
     */
    struct GetEncoderErrorParams {
        // No parameters required
    };

    /** GET_SENSORLESS_ERROR (0x005) parameters
     * Request sensorless estimator error (RTR)
     * ODrive responds with sensorless error value
     * No parameters required
     */
    struct GetSensorlessErrorParams {
        // No parameters required
    };

    /** SET_AXIS_NODE_ID (0x006) parameters
     * Change the CAN node ID of this axis
     * Byte 0-3: new_node_id (uint32_t)
     * WARNING: Changes CAN address; axis must be re-discovered after
     */
    struct SetAxisNodeIdParams {
        uint32_t new_node_id;       // Byte 0-3: New CAN node ID (0-63)
        
        SetAxisNodeIdParams(uint32_t id) : new_node_id(id) {}
    };

    /** SET_AXIS_STARTUP_CONFIG (0x008) parameters
     * Not yet implemented in ODrive firmware
     * No parameters
     */
    struct SetAxisStartupConfigParams {
        // Not implemented
    };

    /** GET_ENCODER_COUNT (0x00A) parameters
     * Request encoder count (RTR)
     * ODrive responds with current encoder count
     * No parameters required
     */
    struct GetEncoderCountParams {
        // No parameters required
    };

    /** START_ANTICOGGING (0x010) parameters
     * Begin anticogging calibration procedure
     * Motor will move in characteristic pattern
     * No parameters required
     */
    struct StartAnticoggingParams {
        // No parameters required
    };

    /** SET_TRAJ_VEL_LIMIT (0x011) parameters
     * Set trajectory planner velocity limit
     * Byte 0-3: traj_vel_limit (float32)
     * Used with InputMode::TRAP_TRAJ
     */
    struct SetTrajVelLimitParams {
        float traj_vel_limit;       // Byte 0-3: Max trajectory velocity (turns/second)
        
        SetTrajVelLimitParams(float limit) : traj_vel_limit(limit) {}
    };

    /** SET_TRAJ_ACCEL_LIMITS (0x012) parameters
     * Set trajectory planner acceleration and deceleration limits
     * Byte 0-3: traj_accel_limit (float32)
     * Byte 4-7: traj_decel_limit (float32)
     * CRITICAL: Both parameters MUST be sent together
     * Used with InputMode::TRAP_TRAJ
     */
    struct SetTrajAccelLimitsParams {
        float traj_accel_limit;     // Byte 0-3: Max acceleration (turns/sec²)
        float traj_decel_limit;     // Byte 4-7: Max deceleration (turns/sec²)
        
        SetTrajAccelLimitsParams(float accel, float decel) 
            : traj_accel_limit(accel), traj_decel_limit(decel) {}
    };

    /** SET_TRAJ_INERTIA (0x013) parameters
     * Set trajectory planner feedforward inertia
     * Byte 0-3: traj_inertia (float32)
     * Used with InputMode::TRAP_TRAJ for feedforward control
     */
    struct SetTrajInertiaParams {
        float traj_inertia;         // Byte 0-3: Feedforward inertia
        
        SetTrajInertiaParams(float inertia) : traj_inertia(inertia) {}
    };

    /** GET_IQ (0x014) parameters
     * Request current Iq setpoint and measured values (RTR)
     * ODrive responds with Iq_setpoint (float32) + Iq_measured (float32)
     * No parameters required
     */
    struct GetIqParams {
        // No parameters required
    };

    /** GET_SENSORLESS_ESTIMATES (0x015) parameters
     * Request sensorless estimator position and velocity (RTR)
     * ODrive responds with position (float32) + velocity (float32)
     * No parameters required
     */
    struct GetSensorlessEstimatesParams {
        // No parameters required
    };

    /** REBOOT (0x016) parameters
     * Reboot the entire ODrive
     * Motor will stop, all state reset
     * No parameters required
     */
    struct RebootParams {
        // No parameters required
    };

    /** GET_BUS_VOLTAGE_CURRENT (0x017) parameters
     * Request bus voltage and current measurements (RTR)
     * ODrive responds with bus_voltage (float32) + bus_current (float32)
     * No parameters required
     */
    struct GetBusVoltageCurrentParams {
        // No parameters required
    };

    /** SET_POSITION_GAIN (0x01A) parameters
     * Set position controller proportional gain
     * Byte 0-3: pos_gain (float32)
     * Affects responsiveness of position loop
     */
    struct SetPositionGainParams {
        float pos_gain;             // Byte 0-3: Position loop proportional gain (1/sec)
        
        SetPositionGainParams(float gain) : pos_gain(gain) {}
    };

    /** SET_VEL_GAINS (0x01B) parameters
     * Set velocity controller gains
     * Byte 0-3: vel_gain (float32) - proportional gain
     * Byte 4-7: vel_integrator_gain (float32) - integrator gain
     * CRITICAL: Both parameters MUST be sent together
     */
    struct SetVelGainsParams {
        float vel_gain;             // Byte 0-3: Velocity loop proportional gain (Nm·sec/turn)
        float vel_integrator_gain;  // Byte 4-7: Velocity loop integrator gain (Nm·sec²/turn)
        
        SetVelGainsParams(float kp, float ki) 
            : vel_gain(kp), vel_integrator_gain(ki) {}
    };

    /** GET_ADC_VOLTAGE (0x01C) parameters
     * Request ADC voltage measurement (RTR)
     * Requires GPIO pin configured for analog input
     * ODrive responds with adc_voltage (float32)
     * No parameters required
     */
    struct GetAdcVoltageParams {
        // No parameters required
    };

    /** GET_CONTROLLER_ERROR (0x01D) parameters
     * Request controller error flags (RTR)
     * ODrive responds with controller_error value
     * No parameters required
     */
    struct GetControllerErrorParams {
        // No parameters required
    };

    // ===== HELPER FUNCTIONS =====

    /** Construct CAN ID from node ID and command ID */
    static uint32_t makeCanId(uint8_t node_id, uint32_t cmd_id) {
        return (node_id << 5) | (cmd_id & 0x1F);
    }

    /** Parse heartbeat message */
    static Heartbeat parseHeartbeat(const can_Message_t& msg);

    /** Parse encoder estimate broadcast */
    static EncoderEstimate parseEncoderEstimate(const can_Message_t& msg);

    // ===== CYCLIC MESSAGE PARSERS (we RECEIVE these) =====
    // Parse incoming cyclic broadcast messages

    /** Parse CYCLIC_HEARTBEAT (0x01) message */
    static CyclicHeartbeat parseCyclicHeartbeat(const can_Message_t& msg);

    /** Parse CYCLIC_MOTOR_ERROR (0x03) message */
    static CyclicMotorError parseCyclicMotorError(const can_Message_t& msg);

    /** Parse CYCLIC_ENCODER_ERROR (0x04) message */
    static CyclicEncoderError parseCyclicEncoderError(const can_Message_t& msg);

    /** Parse CYCLIC_SENSORLESS_ERROR (0x05) message */
    static CyclicSensorlessError parseCyclicSensorlessError(const can_Message_t& msg);

    /** Parse CYCLIC_ENCODER_ESTIMATES (0x09) message */
    static CyclicEncoderEstimates parseCyclicEncoderEstimates(const can_Message_t& msg);

    /** Parse CYCLIC_ENCODER_COUNT (0x0A) message */
    static CyclicEncoderCount parseCyclicEncoderCount(const can_Message_t& msg);

    /** Parse CYCLIC_IQ (0x14) message */
    static CyclicIq parseCyclicIq(const can_Message_t& msg);

    /** Parse CYCLIC_SENSORLESS_ESTIMATES (0x15) message */
    static CyclicSensorlessEstimates parseCyclicSensorlessEstimates(const can_Message_t& msg);

    /** Parse CYCLIC_BUS_VI (0x17) message */
    static CyclicBusVoltageCurrent parseCyclicBusVoltageCurrent(const can_Message_t& msg);

    /** Parse CYCLIC_CONTROLLER_ERROR (0x1D) message */
    static CyclicControllerError parseCyclicControllerError(const can_Message_t& msg);

    // ===== COMMAND BUILDERS =====

    /** Build SET_AXIS_STATE command */
    static can_Message_t buildSetAxisState(uint8_t node_id, AxisState state);

    /** Build SET_CONTROLLER_MODES command */
    static can_Message_t buildSetControllerModes(uint8_t node_id, ControlMode ctrl_mode, InputMode input_mode);

    /** Build SET_INPUT_POS command */
    static can_Message_t buildSetInputPos(uint8_t node_id, float position, 
                                          int16_t velocity = 0, int16_t torque = 0);

    /** Build SET_INPUT_VEL command */
    static can_Message_t buildSetInputVel(uint8_t node_id, float velocity, float torque = 0.0f);

    /** Build SET_INPUT_TORQUE command */
    static can_Message_t buildSetInputTorque(uint8_t node_id, float torque);

    /** Build SET_LIMITS command */
    static can_Message_t buildSetLimits(uint8_t node_id, float vel_limit, float current_limit);

    /** Build SET_LINEAR_COUNT command (reset encoder) */
    static can_Message_t buildSetLinearCount(uint8_t node_id, int32_t count);

    /** Build CLEAR_ERRORS command */
    static can_Message_t buildClearErrors(uint8_t node_id);

    /** Build GET_ENCODER_ESTIMATES request (Remote Transfer Request) */
    static can_Message_t buildGetEncoderEstimates(uint8_t node_id);

    // ===== COMMAND BUILDERS - REQUEST-ONLY (RTR) =====
    // These request data from ODrive; used with Remote Transfer Request (RTR) bit

    /** Build HEARTBEAT_REQUEST command */
    static can_Message_t buildHeartbeatRequest(uint8_t node_id);

    /** Build ESTOP command */
    static can_Message_t buildEstop(uint8_t node_id);

    /** Build GET_MOTOR_ERROR request */
    static can_Message_t buildGetMotorError(uint8_t node_id);

    /** Build GET_ENCODER_ERROR request */
    static can_Message_t buildGetEncoderError(uint8_t node_id);

    /** Build GET_SENSORLESS_ERROR request */
    static can_Message_t buildGetSensorlessError(uint8_t node_id);

    /** Build SET_AXIS_NODE_ID command */
    static can_Message_t buildSetAxisNodeId(uint8_t node_id, uint32_t new_node_id);

    /** Build GET_ENCODER_COUNT request */
    static can_Message_t buildGetEncoderCount(uint8_t node_id);

    /** Build START_ANTICOGGING command */
    static can_Message_t buildStartAnticogging(uint8_t node_id);

    /** Build SET_TRAJ_VEL_LIMIT command */
    static can_Message_t buildSetTrajVelLimit(uint8_t node_id, float traj_vel_limit);

    /** Build SET_TRAJ_ACCEL_LIMITS command */
    static can_Message_t buildSetTrajAccelLimits(uint8_t node_id, float accel_limit, float decel_limit);

    /** Build SET_TRAJ_INERTIA command */
    static can_Message_t buildSetTrajInertia(uint8_t node_id, float inertia);

    /** Build GET_IQ request */
    static can_Message_t buildGetIq(uint8_t node_id);

    /** Build GET_SENSORLESS_ESTIMATES request */
    static can_Message_t buildGetSensorlessEstimates(uint8_t node_id);

    /** Build REBOOT command */
    static can_Message_t buildReboot(uint8_t node_id);

    /** Build GET_BUS_VOLTAGE_CURRENT request */
    static can_Message_t buildGetBusVoltageCurrent(uint8_t node_id);

    /** Build SET_POSITION_GAIN command */
    static can_Message_t buildSetPositionGain(uint8_t node_id, float pos_gain);

    /** Build SET_VEL_GAINS command */
    static can_Message_t buildSetVelGains(uint8_t node_id, float vel_gain, float vel_integrator_gain);

    /** Build GET_ADC_VOLTAGE request */
    static can_Message_t buildGetAdcVoltage(uint8_t node_id);

    /** Build GET_CONTROLLER_ERROR request */
    static can_Message_t buildGetControllerError(uint8_t node_id);

private:
    static constexpr uint8_t NODE_ID_BITS = 6;
    static constexpr uint8_t CMD_ID_BITS = 5;

    /** Get CAN ID from node_id and command_id */
    static inline uint32_t makeCanId(uint8_t node_id, uint8_t cmd_id) {
        return ((uint32_t)node_id << CMD_ID_BITS) | (cmd_id & 0x1F);
    }
};

#endif // __ODRIVE_CAN_PROTOCOL_H__
