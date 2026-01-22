#ifndef __DEBUG_COMMANDS_H__
#define __DEBUG_COMMANDS_H__

#include <Arduino.h>
#include "CanBusHandlerV2.h"

/**
 * Debug Commands Module - CAN Command Testing via Serial
 * 
 * When DebugCommands flag is true in main.cpp:
 * - ESP32 enters debug mode, bypassing main FSM
 * - Reads serial commands from second computer running Arduino IDE
 * - Translates serial commands to CAN messages and sends to ODrive
 * - Outputs formatted 1Hz status with encoder, IQ, voltage, current, ADC
 * - Use odrivetool on first computer to verify ODrive state changes
 * 
 * This allows testing individual CAN commands without running full state machine.
 * 
 * SERIAL COMMAND FORMAT:
 * =====================
 * 
 * AXIS STATE:
 *   axis_state <state_enum>           // Set axis state (0-13)
 *   Example: axis_state 8             // Enter CLOSED_LOOP_CONTROL
 * 
 * CONTROL & INPUT MODES:
 *   controller_modes <ctrl> <input>   // Set control_mode + input_mode (BOTH required)
 *   Example: controller_modes 3 5     // POSITION_CONTROL + TRAP_TRAJ
 *   Valid control modes: 0=Voltage, 1=Torque, 2=Velocity, 3=Position
 *   Valid input modes: 0=Inactive, 1=Passthrough, 2=VelRamp, 3=PosFilter, 5=TrapTraj, 6=TorqueRamp
 * 
 * MOVEMENT COMMANDS:
 *   set_position <pos>                // Set position target (float, turns)
 *   set_velocity <vel>                // Set velocity target (float, turns/sec)
 *   set_torque <torque>               // Set torque target (float, Nm)
 *   Example: set_position 10.5
 *   Example: set_velocity 5.0
 *   Example: set_torque 2.5
 * 
 * LIMITS:
 *   set_limits <vel_limit> <curr_limit>  // Set velocity + current limits (BOTH required)
 *   Example: set_limits 50.0 10.0
 * 
 * ENCODER:
 *   set_linear_count <count>          // Reset encoder to count value (int32)
 *   Example: set_linear_count 0       // Reset to zero
 * 
 * ERROR HANDLING:
 *   clear_errors                      // Clear all ODrive errors
 * 
 * TRAJECTORY CONTROL (InputMode TRAP_TRAJ):
 *   traj_vel_limit <limit>            // Set trajectory velocity limit (float)
 *   traj_accel <accel> <decel>        // Set trajectory accel + decel limits (BOTH required)
 *   traj_inertia <inertia>            // Set trajectory feedforward inertia (float)
 *   Example: traj_vel_limit 20.0
 *   Example: traj_accel 100.0 100.0
 *   Example: traj_inertia 0.0
 * 
 * LOOP TUNING:
 *   pos_gain <gain>                   // Set position loop proportional gain (float)
 *   vel_gains <gain> <integrator>     // Set velocity loop gains (BOTH required)
 *   Example: pos_gain 10.0
 *   Example: vel_gains 0.5 0.05
 * 
 * DIAGNOSTIC REQUESTS (RTR):
 *   heartbeat_request                 // Request immediate heartbeat
 *   get_motor_error                   // Request motor error flags
 *   get_encoder_error                 // Request encoder error flags
 *   get_sensorless_error              // Request sensorless error
 *   get_encoder_count                 // Request encoder count
 *   get_iq                            // Request current Iq values
 *   get_sensorless_estimates          // Request sensorless position/velocity
 *   get_bus_voltage_current           // Request bus voltage/current
 *   get_adc_voltage                   // Request ADC voltage
 *   get_controller_error              // Request controller error flags
 * 
 * ADVANCED:
 *   set_axis_node_id <new_id>         // Change CAN node ID (WARNING: changes address!)
 *   start_anticogging                 // Start anticogging calibration
 *   estop                             // Emergency stop - cuts motor PWM
 *   reboot                            // Reboot entire ODrive
 * 
 * STATUS OUTPUT (1Hz):
 * ==================
 * Time | Position | Velocity | Iq_Set | Iq_Meas | BusVolt | BusCurr | CtrlMode | InputMode | ADC_Vol |
 *  (s) |  (turns) | (T/s)    |  (A)   |   (A)   |  (V)    |  (A)    |   (0-3)  |   (0,1,2,3,5,6)   | (V) |
 */

class DebugCommands {
public:
    /**
     * Initialize debug mode (call from setup())
     * @param canHandler Reference to CanBusHandlerV2 instance
     */
    void begin(CanBusHandlerV2& canHandler);
    
    /**
     * Main debug loop (call from main loop when DebugCommands flag is true)
     * - Reads serial input and parses commands
     * - Sends 1Hz formatted status output
     * Must be called every loop iteration
     */
    void loop();
    
    /**
     * Check if we're in debug mode
     * @return true if debug mode is active
     */
    bool isActive() const { return active_; }
    
    /**
     * Handle a command string (for use in tests/external callers)
     * @param cmd Command string to parse and execute
     */
    void handleCommand(const String& cmd);

private:
    CanBusHandlerV2* can_ = nullptr;
    
    // Status output timing
    uint32_t lastStatusTime_ = 0;
    static constexpr uint32_t STATUS_INTERVAL_MS = 1000;  // 1Hz output
    
    // Track last sent modes for display
    int8_t lastControlMode_ = -1;
    int8_t lastInputMode_ = -1;
    
    bool active_ = true;
    
    // Status output formatting
    void printStatusHeader();
    void printStatusLine();
    String getControlModeName(int mode) const;
    String getInputModeName(int mode) const;
    
    // Helper to parse command and parameters
    bool parseCommand(const String& cmd, String& command, float& param1, float& param2);
};

#endif // __DEBUG_COMMANDS_H__
