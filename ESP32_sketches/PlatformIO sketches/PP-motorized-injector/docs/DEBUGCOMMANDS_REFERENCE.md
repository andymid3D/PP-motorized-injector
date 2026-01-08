# DebugCommands - Serial Command Reference

## Setup

To use DebugCommands debug mode:
1. Set `DebugCommands = true` in `main.cpp`
2. This bypasses the main FSM entirely
3. ESP32 enters pure debug/test mode
4. Open Arduino IDE on second computer connected to ESP32
5. Use Arduino IDE Serial Monitor to send commands
6. On your main computer, use `odrivetool` to verify ODrive state changes

## Serial Command Format

Commands are case-insensitive and sent via Serial Monitor one per line.

---

## AXIS STATE COMMANDS

### axis_state
Set the axis to a specific state (blocks main loop until state achieved).

```
axis_state <state_enum>
```

**Parameters:**
- `state_enum` (int 0-13):
  - 0 = UNDEFINED (will fall through to IDLE)
  - 1 = IDLE (disable motor PWM)
  - 2 = STARTUP_SEQUENCE
  - 3 = FULL_CALIBRATION_SEQUENCE
  - 4 = MOTOR_CALIBRATION
  - 6 = ENCODER_INDEX_SEARCH
  - 7 = ENCODER_OFFSET_CALIBRATION (usually step in homing)
  - 8 = CLOSED_LOOP_CONTROL (ready for movement)
  - 9 = LOCKIN_SPIN
  - 10 = ENCODER_DIR_FIND
  - 11 = HOMING
  - 12 = ENCODER_HALL_POLARITY_CALIB
  - 13 = ENCODER_HALL_PHASE_CALIB

**Examples:**
```
axis_state 1        # Go to IDLE
axis_state 8        # Enter CLOSED_LOOP_CONTROL
axis_state 7        # Encoder offset calibration
```

**Verify with odrivetool:**
```
odrivetool
odrv0.axis0.requested_state
odrv0.axis0.current_state
```

---

## CONTROL & INPUT MODE COMMANDS

### controller_modes
**CRITICAL:** Both parameters MUST be sent together in a single command.

```
controller_modes <control_mode> <input_mode>
```

**Parameters:**
- `control_mode` (int 0-3):
  - 0 = VOLTAGE_CONTROL (not typically used)
  - 1 = TORQUE_CONTROL (direct torque/current control)
  - 2 = VELOCITY_CONTROL (velocity loop + optional torque)
  - 3 = POSITION_CONTROL (all loops: position + velocity + torque)

- `input_mode` (int - one of: 0, 1, 2, 3, 5, 6):
  - 0 = INACTIVE (disable inputs, setpoints retain last value)
  - 1 = PASSTHROUGH (direct passthrough: input_xxx → xxx_setpoint)
  - 2 = VEL_RAMP (velocity ramp - requires VELOCITY_CONTROL)
  - 3 = POS_FILTER (2nd order position filter - requires POSITION_CONTROL)
  - 5 = TRAP_TRAJ (trapezoidal trajectory - requires POSITION_CONTROL)
  - 6 = TORQUE_RAMP (torque ramp - requires TORQUE_CONTROL)

**Examples:**
```
controller_modes 2 1        # Velocity control + passthrough (immediate response)
controller_modes 3 1        # Position control + passthrough
controller_modes 3 5        # Position control + trapezoidal trajectory
controller_modes 1 1        # Torque control + passthrough
```

**Verify with odrivetool:**
```
odrv0.axis0.controller.config.control_mode
odrv0.axis0.controller.config.input_mode
```

---

## MOVEMENT COMMANDS

### set_position
Set position target (requires POSITION_CONTROL mode).

```
set_position <position>
```

**Parameters:**
- `position` (float): Target position in motor turns (can be negative)

**Examples:**
```
set_position 10.5       # Move to 10.5 turns
set_position -5.0       # Move to -5 turns (negative direction)
set_position 0.0        # Move to home (0 turns)
```

### set_velocity
Set velocity target (requires VELOCITY_CONTROL or POSITION_CONTROL mode).

```
set_velocity <velocity>
```

**Parameters:**
- `velocity` (float): Target velocity in turns/second (negative = reverse)

**Examples:**
```
set_velocity 5.0        # Move at 5 turns/second
set_velocity -3.0       # Move at 3 turns/second in reverse
set_velocity 0.0        # Stop
```

### set_torque
Set torque target (requires TORQUE_CONTROL or POSITION_CONTROL mode).

```
set_torque <torque>
```

**Parameters:**
- `torque` (float): Target torque in Nm (negative = reverse)

**Examples:**
```
set_torque 2.5          # Apply 2.5 Nm torque
set_torque -1.0         # Apply 1.0 Nm torque in reverse
```

---

## LIMIT COMMANDS

### set_limits
**CRITICAL:** Both parameters MUST be sent together in a single command.

```
set_limits <velocity_limit> <current_limit>
```

**Parameters:**
- `velocity_limit` (float): Maximum velocity in turns/second
- `current_limit` (float): Maximum motor current in amps

**Examples:**
```
set_limits 50.0 10.0    # Max 50 T/s, max 10 amps
set_limits 100.0 5.0    # Max 100 T/s, max 5 amps (careful!)
set_limits 20.0 15.0    # Conservative: 20 T/s, 15 amps
```

**Verify with odrivetool:**
```
odrv0.axis0.controller.config.vel_limit
odrv0.axis0.motor.config.current_lim
```

---

## ENCODER COMMANDS

### set_linear_count
Reset encoder to a specific count value (typically 0 to zero encoder).

```
set_linear_count <count>
```

**Parameters:**
- `count` (int32): Encoder count value (typically 0)

**Examples:**
```
set_linear_count 0      # Reset encoder to zero
set_linear_count 100    # Set encoder to 100
set_linear_count -50    # Set encoder to -50
```

**Verify with odrivetool:**
```
odrv0.axis0.encoder.pos_estimate
```

---

## ERROR HANDLING

### clear_errors
Clear all ODrive errors. Required before axis can enter CLOSED_LOOP after an error.

```
clear_errors
```

**Verify with odrivetool:**
```
odrv0.axis0.error
odrv0.axis0.motor.error
odrv0.axis0.encoder.error
odrv0.axis0.controller.error
```

---

## TRAJECTORY CONTROL (InputMode TRAP_TRAJ only)

Used when `controller_modes 3 5` (Position + TrapTraj).

### traj_vel_limit
Set maximum velocity for trapezoidal trajectory.

```
traj_vel_limit <limit>
```

**Parameters:**
- `limit` (float): Maximum trajectory velocity in turns/second

**Examples:**
```
traj_vel_limit 20.0     # Limit trajectory to 20 T/s
traj_vel_limit 50.0     # Fast trajectory: 50 T/s
```

### traj_accel
**CRITICAL:** Both parameters MUST be sent together in a single command.

Set acceleration and deceleration limits for trajectory.

```
traj_accel <acceleration> <deceleration>
```

**Parameters:**
- `acceleration` (float): Acceleration limit in turns/sec²
- `deceleration` (float): Deceleration limit in turns/sec²

**Examples:**
```
traj_accel 100.0 100.0  # Symmetric: 100 T/s² accel, 100 T/s² decel
traj_accel 50.0 100.0   # Asymmetric: 50 T/s² accel, 100 T/s² decel (slower accel)
traj_accel 200.0 150.0  # Fast accel, moderate decel
```

### traj_inertia
Set feedforward inertia for trajectory.

```
traj_inertia <inertia>
```

**Parameters:**
- `inertia` (float): Feedforward inertia (typically 0.0 for most applications)

**Examples:**
```
traj_inertia 0.0        # No feedforward (typical)
traj_inertia 0.1        # Small feedforward
```

---

## LOOP TUNING

### pos_gain
Set position controller proportional gain.

```
pos_gain <gain>
```

**Parameters:**
- `gain` (float): Position loop proportional gain (typical range 5-20 1/sec)

**Examples:**
```
pos_gain 10.0           # Moderate responsiveness
pos_gain 20.0           # High responsiveness (may oscillate)
pos_gain 5.0            # Low responsiveness (slow settling)
```

**Verify with odrivetool:**
```
odrv0.axis0.controller.config.pos_gain
```

### vel_gains
**CRITICAL:** Both parameters MUST be sent together in a single command.

Set velocity controller proportional and integrator gains.

```
vel_gains <proportional> <integrator>
```

**Parameters:**
- `proportional` (float): Velocity loop proportional gain (typical 0.1-1.0 Nm·sec/turn)
- `integrator` (float): Velocity loop integrator gain (typical 0.01-0.1 Nm·sec²/turn)

**Examples:**
```
vel_gains 0.5 0.05      # Balanced: P=0.5, I=0.05
vel_gains 1.0 0.1       # Aggressive: P=1.0, I=0.1 (fast response, may overshoot)
vel_gains 0.2 0.02      # Conservative: P=0.2, I=0.02 (slow, stable)
```

**Verify with odrivetool:**
```
odrv0.axis0.controller.config.vel_gain
odrv0.axis0.controller.config.vel_integrator_gain
```

---

## DIAGNOSTIC REQUESTS (Remote Transfer Request)

These request data from ODrive. Enable the corresponding cyclic broadcast in ODrive config to receive responses.

### heartbeat_request
Request immediate heartbeat (normally sent ~100ms, this forces immediate).

```
heartbeat_request
```

### get_motor_error
Request motor error flags.

```
get_motor_error
```

### get_encoder_error
Request encoder error flags.

```
get_encoder_error
```

### get_sensorless_error
Request sensorless estimator error flags.

```
get_sensorless_error
```

### get_encoder_count
Request current encoder count.

```
get_encoder_count
```

### get_iq
Request current Iq (motor current) setpoint and measured values.

```
get_iq
```

### get_sensorless_estimates
Request sensorless estimator position and velocity.

```
get_sensorless_estimates
```

### get_bus_voltage_current
Request bus voltage and current measurements.

```
get_bus_voltage_current
```

### get_adc_voltage
Request ADC voltage reading (requires GPIO configured for analog input).

```
get_adc_voltage
```

### get_controller_error
Request controller error flags.

```
get_controller_error
```

---

## ADVANCED COMMANDS

### set_axis_node_id
**WARNING:** This changes the CAN address of the ODrive!

```
set_axis_node_id <new_id>
```

**Parameters:**
- `new_id` (int 0-63): New CAN node ID

**Examples:**
```
set_axis_node_id 1      # Change to node ID 1
```

### start_anticogging
**WARNING:** Motor will move in a characteristic pattern!

Start anticogging calibration procedure.

```
start_anticogging
```

### estop
**EMERGENCY STOP** - Cuts motor PWM immediately.

```
estop
```

### reboot
**WARNING:** ODrive will restart, losing CAN connection briefly.

Reboot the entire ODrive controller.

```
reboot
```

---

## STATUS OUTPUT

Every 1 second, DebugCommands prints a formatted status line:

```
Time[s] | Position[T] | Velocity[T/s] | Iq_Set[A] | Iq_Meas[A] | BusV[V] | BusI[A] | CtrlMode | InputMode | ADC[V]
```

**Columns:**
- **Time[s]**: Elapsed time since debug mode started
- **Position[T]**: Current encoder position in turns
- **Velocity[T/s]**: Current encoder velocity in turns/second
- **Iq_Set[A]**: Commanded motor current (amps)
- **Iq_Meas[A]**: Measured motor current (amps)
- **BusV[V]**: ODrive power supply voltage
- **BusI[A]**: ODrive power supply current draw
- **CtrlMode**: Current control mode (Voltage/Torque/Velocity/Position)
- **InputMode**: Current input mode (Inactive/Passthru/VelRamp/PosFilt/TrapTraj/TorqRamp)
- **ADC[V]**: Analog input voltage (if ADC cyclic message enabled)

---

## HELP COMMAND

Display all available commands in Serial Monitor:

```
help
```

Or:

```
?
```

---

## TESTING WORKFLOW

1. **Start debug mode**: Set `DebugCommands = true` in main.cpp, upload, open Arduino IDE Serial Monitor
2. **Test basic operation**:
   ```
   axis_state 8                    # Enter CLOSED_LOOP
   controller_modes 2 1            # Velocity mode + passthrough
   set_velocity 5.0                # Command 5 T/s
   ```
3. **Monitor with odrivetool** (on your main computer):
   ```
   odrv0.axis0.current_state       # Should be 8
   odrv0.axis0.encoder.vel_estimate # Should show ~5.0
   ```
4. **Stop motion**:
   ```
   set_velocity 0.0
   ```
5. **Test position control**:
   ```
   controller_modes 3 5            # Position mode + trajectory
   set_position 10.0               # Command position
   ```
6. **Monitor trajectory**:
   ```
   odrv0.axis0.encoder.pos_estimate # Watch position progress
   ```

---

## TIPS FOR CURRENT MEASUREMENT

To measure peak current during different movements:
1. Watch the **Iq_Meas** column in status output
2. Command different speeds/torques and note peak current
3. Use this data to tune `set_limits` current_limit parameter
4. Example: "At 5 T/s with TRAP_TRAJ, peak current is 8.5A, so set limit to 10A for safety margin"
