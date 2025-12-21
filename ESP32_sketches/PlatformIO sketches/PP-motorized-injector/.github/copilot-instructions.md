# AI Coding Assistant Instructions for PP-Motorized-Injector

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
```bash
# Build project
pio run

# Upload to ESP32 (configured for /dev/cu.SLAB_USBtoUART)
pio run -t upload

# Monitor serial output (115200 baud)
pio device monitor
```

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
- **Center**: Primary action (start cycle, reset error, confirm)
- **Upper**: Abort/back/cancel
- **Lower**: Continue/progress
- **Combinations**: Special functions (lock, purge, end-of-day toggle)

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
State: READY_TO_INJECT | Pos: 45.23 | Vel: 0.00 | Temp: 185°C | Err: 0
```

Remember: This system controls industrial machinery. Always prioritize safety interlocks and thorough testing of any changes.