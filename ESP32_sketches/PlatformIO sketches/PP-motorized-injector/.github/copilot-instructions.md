# AI PERSONA & BEHAVIOR (STRICT MODE)
- **Role:** Senior Embedded Firmware Engineer.
- **Tone:** Professional, concise, deterministic.
- **Coding Style:** 
  - NO hallucinated variables. Use ONLY what is defined in `config.h` and headers.
  - PREFER verbose, safe logic over clever one-liners.
  - ALWAYS check `SafetyManager` before movement.
  - NEVER remove safety checks to "fix" a bug; fix the logic instead.

## CRITICAL: Non-Blocking Architecture
- **NEVER use blocking delays or while loops** in production code
- **Use state machines for ALL sequential operations** (homing, compression, injection, etc.)
- State machines allow loop() to run continuously, keeping serial/CAN responsive
- **SafeString is the central hub** for:
  - Storing all broadcast ODrive data (position, velocity, state, current, etc.)
  - Non-blocking serial messaging (use BufferedOutput, not Serial.println)
  - Debug data aggregation from all modules
  - One source of truth for machine state
- **All modules communicate through SafeString central data store**, not direct function calls
- Use millisDelay for non-blocking timeouts instead of delay()

## SafeString Integration (MANDATORY)
- **Central Data Repository:** Create SafeString fields for all broadcast values (update via getCyclic* callbacks)
- **Serial Messaging:** Use BufferedOutput for all Serial output - handles queueing gracefully
- **Non-blocking Timeouts:** Use millisDelay instead of delay()
- **Message Aggregation:** Debug modules append their data to SafeString, one central print() call

## Motor Control - CRITICAL GUIDELINES
- **Input Mode Priority:** PREFER ramped input modes (VEL_RAMP, POS_FILTER, TRAP_TRAJ) over PASSTHROUGH
  - Ramped modes reduce motor stress, spinouts, and overcurrents
  - PASSTHROUGH only when no ramped alternative exists
  - Less current stress = safer long-term motor operation and better plastic consistency

## Configuration & Timing - CRITICAL GUIDELINES
- **ALWAYS search config.h for existing variables** before hardcoding timeouts, speeds, or timings
- Homing timing variables exist in config.h (speeds, delays, etc.) - use them consistently
- Changing timings must be ONE place only - config.h
- **Source of Truth - Broadcast Data:**
  - ODrive broadcasts axis state, encoder position, and velocity every ~100ms (encoder data ~10ms)
  - **NEVER use arbitrary timeouts** - use broadcast state transitions instead
  - **Wait for state changes** rather than timers (e.g., state 1→7→1 pattern for calibration complete)
  - **Use velocity/position from cyclic broadcasts** to determine motion completion
  - Example: Instead of `delay(5000)`, wait for `axisState == IDLE` broadcast
  - Cyclic data is centralized source of truth - always prefer over calculated estimates

# CRITICAL: ODrive CANSimple Protocol - SOURCES OF TRUTH
**MUST use these official ODrive documentation sources for ANY CAN-related work:**

1. **CAN Message IDs & Formats**
   - Official: https://docs.odriverobotics.com/v/0.5.6/can-protocol.html#messages
   - Tables show: message name, ID, master/axis, parameters, bit layout
   - **CRITICAL**: Cyclic broadcast IDs have 1 fewer hex digit (e.g., 0x01 not 0x001)
   - Cyclic broadcasts: https://docs.odriverobotics.com/v/0.5.6/can-protocol.html#cyclic-messages

2. **Axis State Machine**
   - Official: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState
   - States: UNDEFINED(0), IDLE(1), STARTUP(2), FULL_CALIB(3), MOTOR_CALIB(4), ENCODER_INDEX(6), ENCODER_OFFSET(7), CLOSED_LOOP(8), LOCKIN(9), ENCODER_DIR(10), HOMING(11), ENCODER_HALL_POLARITY(12), ENCODER_HALL_PHASE(13)

3. **Control Modes**
   - Official: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode
   - Modes: VOLTAGE(0), TORQUE(1), VELOCITY(2), POSITION(3)

4. **Input Modes**
   - Official: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode
   - Modes: INACTIVE(0), PASSTHROUGH(1), VEL_RAMP(2), POS_FILTER(3), TRAP_TRAJ(4), TORQUE_RAMP(5)
   - **Each mode has specific control mode requirements** (documented in ODrive reference)

**Rule: If CAN message IDs or protocol definitions appear wrong, ALWAYS verify against official docs. Never infer from code. Never use github can_simple.hpp without cross-checking docs.**

# AI Coding Assistant Instructions for PP-Motorized-Injector

Project: PP Injector (ESP32 + ODrive 3.6 Protocol)
Hardware:

- Motor: ODesc 4.2 (Node ID 0), Inverted Direction.
- Sensors: HX711 (Pressure), AD597 (Temp), Inductive Endstops (Top/Bot/Barrel).

Critical Logic:
- SafetyManager must be checked every loop.
- Homing: Retract to Top -> Relax (Idle) -> Backoff -> Zero.
- Safety: Pressure Limit protects 2D moulds.

# CRITICAL HARDWARE OVERRIDES (PRIORITY 1)
1. **Motor Direction:** INVERTED. Positive Position = Down (Inject). Negative Position = Up (Retract).
2. **Homing Sequence (Critical Flow):**
   - **Step 0:** Clear errors, set context to CTX_MOVING_FREE, wait 500ms
   - **Step 1:** Check if calibration already done this session (skip step 2 if true)
   - **Step 2-3:** Calibration (State 7). Runs ONLY once per power cycle. Set `flags.calibrationDone = true` when complete.
   - **Step 4:** Request Closed Loop (State 8). MUST succeed before moving to step 5. Retry every 500ms if needed.
   - **Step 5:** Fast retract up until top endstop hit
   - **Step 6:** Gradual deceleration to prevent spinout (Error 0x200). Wait for velocity < 0.1 or timeout
   - **Step 7:** Backoff (move down slowly) for 1.5s to relax endstop pressure
   - **Step 8:** Slow approach back to top endstop
   - **Step 9:** Wait for motor stop (velocity < 0.05 for 500ms)
   - **Step 10:** Reset encoder to 0, mark `flags.initialHomingDone = true`, return true to finish
3. **Safety:** 
   - Pressure Limit (Torque) is the primary safety for 2D moulds.
   - 2D Moulds = Low Torque Limit. 3D Moulds = High Torque Limit.
4. **Buttons:**
   - Buttons: Upper (25), Center (26), Lower (27).
   - Logic: Upper+Lower = Purge Entry. Center = Confirm/Exit.
5. **Motor Control:**
   - ALWAYS use `setModeAndMove()` wrapper. Never call `motor.setX()` directly.
   - INJECT/HOLD/RELEASE use Position Control (Mode 3).
   - COMPRESSION uses Torque Control (Mode 1).


## Project Overview
This is an ESP32-based controller for a motorized injection molding machine using PlatformIO. The system implements a finite state machine (FSM) to manage the complete injection cycle with integrated safety systems.

## Architecture Overview

### Core Components
- **Main FSM** (`injector_fsm.h`, `main.cpp`): 13-state machine controlling injection process (heating → homing → refill → compression → injection → release)
- **CanBusHandler** (`CanBusHandler.h/.cpp`): CAN bus communication with ODrive motor controller
- **SafetyManager** (`SafetyManager.h/.cpp`): Safety interlocks, endstops, E-stop, temperature monitoring
- **Hardware Integration**: Buttons, LED rings, temperature sensor, load cell (HX711)

### Key Design Patterns

#### State Machine Implementation
```cpp
// States are enums in injector_fsm.h
enum InjectorStates {
    ERROR_STATE, INIT_HEATING, INIT_HOMING, REFILL,
    COMPRESSION, READY_TO_INJECT, INJECT, HOLD_INJECTION, RELEASE
};

// State transitions in main.cpp loop()
switch (fsm_state.currentState) {
    case InjectorStates::INIT_HEATING:
        if (temperature >= TEMP_CRITICAL) 
            fsm_state.currentState = INIT_HOT_NOT_HOMED;
        break;
    // ... more states
}
```

#### Safety Context System
```cpp
enum SafetyContext { CTX_IDLE, CTX_MOVING_FREE, CTX_BLOCKED, CTX_PURGE };
safety.setContext(CTX_BLOCKED);  // During injection, movement is restricted
```

#### Motor Control Abstraction
```cpp
// Three control modes with safety wrapper
setModeAndMove(1, 1, torque_value, "Torque Control");     // Mode 1: Torque
setModeAndMove(2, 1, velocity_value, "Velocity Control"); // Mode 2: Velocity  
setModeAndMove(3, 1, position_value, "Position Control");  // Mode 3: Position
```

## Critical Developer Workflows

### Building and Flashing
⚠️ **CRITICAL:** PlatformIO is NOT in system PATH. Always use full path:
```bash
# Full path to PlatformIO (REQUIRED for AI assistant terminal commands)
/Users/andy/.platformio/penv/bin/pio run              # Build project
/Users/andy/.platformio/penv/bin/pio run -t upload    # Upload to ESP32 (/dev/cu.SLAB_USBtoUART, 115200 baud)
/Users/andy/.platformio/penv/bin/pio device monitor   # Monitor serial output

# Alternative: Create alias in shell (but AI must use full path)
alias pio=/Users/andy/.platformio/penv/bin/pio
pio run
```

**For terminal commands in code:** Always use the full path `/Users/andy/.platformio/penv/bin/pio` - do NOT assume `pio` is available.

### Debugging Process
- **LED State Indicators**: Check button LEDs and ring LEDs for current state
- **Serial Debug Output**: 1Hz status reports show position, velocity, temperature
- **Error Codes**: Check `fsm_state.error` and `motor.getAxisError()`
- **Safety Triggers**: Monitor SafetyManager for interlock violations

### Testing Safety Systems
```cpp
// Test E-stop behavior
if (safety.isEStopPressed()) {
    safety.triggerHalt(ERR_ESTOP);
    fsm_state.currentState = ERROR_STATE;
}
```

## Project-Specific Conventions

### Motor Units and Directions
- **Position**: Turns (not linear units)
- **Velocity**: Turns/second
- **Direction**: `INVERT_MOTOR_DIR = true` (positive = down/inject)
- **Conversion**: `volToTurns(cm3) = cm3 * TURNS_PER_CM3_VOL`

### Safety-First Design
- **Temperature Gates**: Movement blocked below `TEMP_MIN_MOVE` (16°C)
- **Context Restrictions**: Different safety rules for idle vs. moving vs. blocked states
- **Button Lock**: Upper+Lower buttons pressed simultaneously locks controls
- **End-of-Day Mode**: Upper+Lower buttons toggle `flags.endOfDay` for different return states

### LED State Encoding
```cpp
// Color-coded state feedback
ERROR_STATE: RED everywhere
INIT_HEATING: SOLID RED
COMPRESSION: RED upper/lower, RED ring
READY_TO_INJECT: GREEN upper/lower, YELLOW center, GREEN ring
INJECT: RED upper, GREEN center, BLACK lower, RED ring
```

### Button Control Logic

## Integration Points

### ODrive Motor Controller
- **CAN Node ID**: 0 (configurable in `config.h`)
- **Commands**: Axis state, controller mode, position/velocity/torque setpoints
- **Feedback**: Position, velocity, torque, bus voltage, error codes

### Hardware Interfaces
- **CAN Bus**: ESP32-TWAI-CAN library
- **Temperature**: Analog reading with smoothing filter
- **Load Cell**: HX711 for pressure monitoring
- **LEDs**: NeoPixel rings and button indicators
- **Buttons**: Debounced with Bounce2 library

### External Safety Systems
- **Endstops**: Top, bottom, barrel position sensors
- **E-Stop**: Hardware emergency stop circuit
- **Contactor**: Motor power control relay

## Common Development Tasks

### Adding New FSM States
1. Add to `InjectorStates` enum in `injector_fsm.h`
2. Add case in main.cpp `switch` statement
3. Update `updateLeds()` for visual feedback
4. Add state transition logic

### Modifying Safety Rules
1. Update `SafetyManager::check()` method
2. Consider impact on all `SafetyContext` modes
3. Test with hardware interlocks

### Motor Control Changes
1. Use `setModeAndMove()` wrapper for safety timing
2. Respect 20ms minimum between commands
3. Check axis state before issuing commands

### Configuration Changes
- Update constants in `config.h`
- Rebuild and re-upload
- Test homing sequence after mechanical changes

## Error Handling Patterns

### Motor Errors
```cpp
if (motor.getAxisError() != 0) {
    fsm_state.currentState = ERROR_STATE;
    fsm_state.error = motor.getAxisError();
}
```

### Safety Violations
```cpp
if (!safety.check(velocity, moving_down)) {
    fsm_state.currentState = ERROR_STATE;
    fsm_state.error = safety.getLastError();
}
```

### Temperature Issues
```cpp
if (temperature < TEMP_CRITICAL) {
    safety.triggerHalt(ERR_UNDER_TEMP);
    tempErrorActive = true;
}
```

## Testing and Validation

### Hardware Testing Sequence
1. **Power On**: Verify heating state, LED feedback
2. **Homing**: Check endstop detection, position reset
3. **Manual Movement**: Test purge mode with button controls
4. **Injection Cycle**: Full cycle with mould parameters
5. **Safety Tests**: E-stop, endstops, temperature limits

### Debug Output Format
```
State: READY_TO_INJECT | Pos: 45.23 | Vel: 0.00 | Temp: 185°C | Err: 0 | ODescState: 0 
```

Remember: This system controls industrial machinery. Always prioritize safety interlocks and thorough testing of any changes.