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
This is an ESP32-based controller for a motorized injection molding machine using PlatformIO. The system implements a modular non-blocking state machine architecture to manage the complete injection cycle with integrated safety systems.

## Architecture Overview - MODULAR STATE MACHINE (v2 Design)

### Core Components
- **Main FSM Orchestrator** (`main.cpp`): High-level state router, button handling, LED feedback
- **Modular State Machines** (individual `.h/.cpp` files):
  - `Homing.cpp` ✅ LOCKED - Non-blocking 12-state homing sequence
  - `Refill.cpp` - Move plunger to rest position (position control with trap trajectory)
  - `Compression.cpp` - Velocity ramp with contact detection, then torque ramp (Modes 1 & 2)
  - `PurgeZero.cpp` - Manual nozzle purge with button-controlled velocity
  - `AntiDrip.cpp` - Slow upward decompression with timeout logic
  - `Injection.cpp` - Position control with auto-transition from fill → pack phases
  - `ReadyToInject.cpp` - Idle waiting with autonomous micro-compression every 30s
- **CAN Bus** (`CanBusHandlerV2.cpp`): Non-blocking CAN communication with 50ms command gap enforcement
- **Broadcast Data Store** (`BroadcastDataStore.cpp`): Central data aggregation from ODrive cyclic messages
- **SafetyManager** (`SafetyManager.h/.cpp`): Safety interlocks, endstops, E-stop, temperature monitoring
- **Hardware Integration**: Buttons (debounced), LED rings (NeoPixel), temperature sensor (AD597), pressure sensor (HX711)

### Key Design Patterns

#### Non-Blocking State Machine Template (All Modules)
```cpp
// Each module uses identical pattern for consistency
namespace ModuleName {
    static enum { STEP_1, STEP_2, ... } step = DONE;
    static unsigned long stepTimer = 0;
    static bool stateEntry = false;
    static bool complete = false;
    static bool error = false;
    
    void begin() {
        step = STEP_1;
        stateEntry = true;
        complete = false;
        error = false;
    }
    
    bool update(CanBusHandlerV2& motor) {
        unsigned long now = millis();
        unsigned long elapsed = now - stepTimer;
        
        if (stateEntry) { /* initialization */ stateEntry = false; }
        
        switch(step) {
            // Non-blocking handlers: NO delays, NO while loops
            // Return: true = complete, false = still running
        }
        return complete;
    }
    
    bool isComplete() { return complete && !error; }
    bool hasError() { return error; }
    void reset() { stateEntry = true; complete = false; error = false; }
}
```

#### CAN Command Timing (CRITICAL - 50ms Gap Enforcement)
- `CAN_COMMAND_GAP_MS = 50` enforced globally in CanBusHandlerV2
- Every CAN command must wait 50ms after the previous command was SENT (not queued)
- Allows ODrive firmware time to process mode changes before new setpoints arrive
- **Implementation:** CanBusHandlerV2.loop() checks `(now - lastCommandSentTime_) >= CAN_COMMAND_GAP_MS`
- All modules respect this gap; no exceptions except E-stop

#### Motor Control Modes by State (SPECIFICATION)
| State | Control Mode | Input Mode | Direction | Pressure Check | Key Details |
|-------|--------------|-----------|-----------|---|---|
| **Refill** | Position (3) | TRAP_TRAJ (4) | Up | NO | Safe ramp to OFFSET_REFILL_GAP |
| **Compression Mode 1** | Velocity→Torque | VEL_RAMP (2)→TORQUE_RAMP (6) | Down | YES (weak→spike) | Travel until contact, then ramp |
| **Compression Mode 2 (Micro)** | Torque | TORQUE_RAMP (6) | Down | YES | Light ramp, ~2 sec, silent |
| **PurgeZero** | Velocity | PASSTHROUGH (1) | Up/Down | TBD | Manual plunger, button-controlled |
| **AntiDrip** | Velocity | PASSTHROUGH (1) | Up | NO | Slow retract to prevent drip |
| **Inject** | Position | TRAP_TRAJ (4) | Down | YES | Torque limit detects mould full |
| **Hold/Pack** | TBD (Torque or Pos) | TBD | Down | NO | Maintain constant packing pressure |
| **Release** | Position | TRAP_TRAJ (4) | Up | NO | Quick unload relief |

**Input Mode Reference (ODrive 0.5.6):**
- INACTIVE (0): No control
- PASSTHROUGH (1): Direct setpoint, immediate response (manual control)
- VEL_RAMP (2): Velocity with acceleration limiting (contact detection)
- POS_FILTER (3): Position with filtering
- TRAP_TRAJ (4): Trapezoidal trajectory (smooth ramps, safe, PREFERRED for position)
- TORQUE_RAMP (5/6): Torque with ramping (safe compression)

#### Safety Context System (Module-Aware)
```cpp
enum SafetyContext { CTX_IDLE, CTX_MOVING_FREE, CTX_BLOCKED, CTX_PURGE };
// CTX_IDLE: Normal idle, full movement allowed
// CTX_MOVING_FREE: Homing/AntiDrip, careful movement monitoring
// CTX_BLOCKED: Injection/Hold, no reversal allowed, strict pressure/force limits
// CTX_PURGE: Manual purge, user-controlled movement, no safety restrictions
safety.setContext(CTX_BLOCKED);  // Set during injection states
```

#### Pressure Sensor (HX711) Strategy
- **Activated on:** Compression (both modes), Inject, ReadyToInject micro
- **Not used on:** Refill, PurgeZero, AntiDrip, Release
- **Signal characteristics:**
  - Weak initial signal (when block/mould starts contact)
  - Sudden spike when plunger contacts plastic
  - Difficulty: Distinguish "no block present" from "block present but no plastic"
- **Implementation note:** Contact detection = velocity drop + pressure spike (both confirm)

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
- **Direction**: `INVERT_MOTOR_DIR = false` (positive position = down/inject, negative = up/retract)
- **Conversion**: `volToTurns(cm3) = cm3 * TURNS_PER_CM3_VOL`

### Safety-First Design
- **Temperature Gates**: Movement blocked below `TEMP_MIN_MOVE` (16°C)
- **Context Restrictions**: Different safety rules by context:
  - CTX_IDLE: Normal idle, full movement allowed
  - CTX_MOVING_FREE: Homing/AntiDrip, careful monitoring
  - CTX_BLOCKED: Injection/Hold, no reversal, strict pressure/force limits
  - CTX_PURGE: Manual purge, user-controlled, minimal restrictions
- **Button Lock**: Upper+Lower buttons pressed simultaneously locks controls
- **End-of-Day Mode**: `flags.endOfDay` determines return state after cycle (READY_TO_INJECT if true, REFILL if false)

### LED State Encoding
```cpp
// Color-coded state feedback
ERROR_STATE: RED everywhere (flashing)
INIT_HEATING: SOLID RED
INIT_HOT_NOT_HOMED: SOLID YELLOW
INIT_HOMING: YELLOW upper/ring (flashing)
REFILL: CENTER=GREEN, UPPER/LOWER=BLACK (or BLUE if endOfDay), RING=GREEN
COMPRESSION: RED upper/lower, BLACK center, RED ring
READY_TO_INJECT: GREEN upper/lower, YELLOW center, GREEN ring
PURGE_ZERO: YELLOW upper/lower, GREEN center, YELLOW ring
ANTIDRIP: RED upper, GREEN center/lower, RED ring
INJECT: RED upper, GREEN center, BLACK lower, RED ring
HOLD_INJECTION: RED upper, GREEN center/lower, RED ring
RELEASE: GREEN everywhere
CONFIRM_MOULD_REMOVAL: GREEN everywhere
```

### Button Control Logic
- **Upper button (25):** Abort most operations, return to safe state
- **Center button (26):** Confirm actions, proceed to next state
- **Lower button (27):** Same as center in most contexts
- **Upper+Lower combo:** Toggle `flags.endOfDay` (REFILL state only)
- **Button debounce:** 10ms per button, handled by Bounce2 library
- **Double-press detection:** Check released() for button release events

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

### Motor Control Changes (MODULAR PATTERN)
All motor control now goes through individual state machine modules, NOT direct motor calls:
```cpp
// OLD (deprecated): Direct motor calls in main.cpp
motor.setControllerModes(...);
motor.setInputPos(...);

// NEW (required): Module handles all motor control
Compression::begin();
Compression::update(motor);
if (Compression::isComplete()) { /* transition */ }
```

### Adding New Modular States
1. Create `YourModule.h` with namespace, public functions (begin, update, isComplete, hasError, reset)
2. Create `YourModule.cpp` following non-blocking state machine template
3. Add to main.cpp switch statement:
   ```cpp
   case YOUR_STATE:
       if (stateEntry) YourModule::begin();
       if (YourModule::update(motor)) { /* next state */ }
       if (!ignoreButtons && /* button check */) { YourModule::handleButton(); }
       break;
   ```
4. Update `updateLeds()` with LED feedback
5. Test non-blocking behavior (no delays, no blocking calls)

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

---

## MODULAR STATE MACHINE ARCHITECTURE (v2 - COMPLETE)

### File Structure
```
include/
  Homing.h              ✅ LOCKED
  Refill.h
  Compression.h
  Injection.h
  AntiDrip.h
  PurgeZero.h
  ReadyToInject.h

src/
  Homing.cpp            ✅ LOCKED
  Refill.cpp
  Compression.cpp
  Injection.cpp
  AntiDrip.cpp
  PurgeZero.cpp
  ReadyToInject.cpp
  main.cpp (refactored orchestrator)
```

### State Flow Diagram
```
INIT_HEATING
    ↓
INIT_HOT_NOT_HOMED
    ↓ (Upper button)
INIT_HOMING (Homing::update)
    ↓ (auto-complete)
REFILL (Refill::update)
    ├─ Upper+Lower: Toggle endOfDay
    └─ Center: proceed
        ↓
    COMPRESSION (Compression::update - Mode 1)
        ├─ Upper: abort → REFILL
        └─ Lower: complete → READY_TO_INJECT
            ↓
        READY_TO_INJECT (ReadyToInject::update)
            ├─ Auto micro-compression every 30s
            ├─ Upper+Lower: proceed → PURGE_ZERO
            └─ Center: abort → REFILL
                ↓
            PURGE_ZERO (PurgeZero::update)
                └─ Center: confirm → ANTIDRIP
                    ↓
                ANTIDRIP (AntiDrip::update)
                    ├─ Center+Lower: confirm → INJECT
                    └─ Upper OR timeout → READY_TO_INJECT
                        ↓
                    INJECT (Injection::update - FILLING phase)
                        ↓ (auto-transition)
                    HOLD_INJECTION (Injection::update - PACKING phase)
                        ├─ Upper: abort → RELEASE
                        └─ Auto-complete → RELEASE
                            ↓
                        RELEASE
                            ↓ (auto-complete)
                        CONFIRM_MOULD_REMOVAL
                            └─ Any button: endOfDay ? READY_TO_INJECT : REFILL
```

### Detailed Module Specifications

#### Refill.cpp
- **Purpose:** Move plunger to rest position (OFFSET_REFILL_GAP)
- **Control:** Position control with TRAP_TRAJ (safe ramps)
- **Direction:** Up (negative position)
- **Pressure Check:** NO
- **Buttons:** Upper+Lower = toggle endOfDay; Center = proceed
- **Context:** CTX_IDLE
- **Research:** Check for drift accumulation (optional homing-zero on each refill)

#### Compression.cpp
- **Purpose:** Compress plastic in barrel (two modes)
  - **Mode 1:** Travel down from OFFSET_REFILL_GAP until contact, then torque ramp
  - **Mode 2:** Skip travel, go straight to torque ramp (micro-compression only)
- **Control:** VEL_RAMP (travel) → TORQUE_RAMP (compress)
- **Direction:** Down (positive)
- **Pressure Check:** YES - weak signal at start, spike on contact
- **Contact Detection:** Velocity drop + pressure spike
- **Buttons (Mode 1):** Upper = abort → REFILL; Lower = complete → READY_TO_INJECT
- **Context:** CTX_BLOCKED
- **Timeout:** ~10 seconds (no plastic = return to REFILL)
- **Future Enhancement:** Velocity ramp to medium speed with low torque check, then switch to torque mode

#### ReadyToInject.cpp
- **Purpose:** Idle waiting with autonomous micro-compression
- **Micro-Compression:** Runs silently every 30 seconds, ~2 second duration
- **Control:** Torque TORQUE_RAMP for compression
- **Direction:** Down during compression only
- **Pressure Check:** YES (during micro-compression)
- **LED Feedback:** Optional center button LED flash during compression
- **Buttons:** Upper+Lower = proceed to PURGE_ZERO; Center = abort to REFILL
- **Context:** CTX_IDLE
- **Auto-Abort:** If user presses button during micro-compression, stop and proceed

#### PurgeZero.cpp
- **Purpose:** Manual plunger movement to purge nozzle and establish injection zero point
- **Control:** Velocity PASSTHROUGH (direct, immediate response)
- **Direction:** Up (Upper button) or Down (Lower button)
- **Pressure Check:** TBD (investigate signal strength)
- **Button Debounce:** Wait for Upper+Lower release before accepting commands
- **Buttons:** Upper = retract up; Lower = push out; Center = confirm zero
- **Context:** CTX_PURGE
- **Note:** Allows user to manually remove cold plastic from nozzle

#### AntiDrip.cpp
- **Purpose:** Slow upward retract to prevent plastic drip while user places mould
- **Control:** Velocity PASSTHROUGH (direct)
- **Direction:** Up (negative)
- **Speed:** SPEED_ANTIDRIP (2.0 turns/sec)
- **Pressure Check:** NO (already confirmed in previous states)
- **Timeout:** TIME_ANTIDRIP_TIMEOUT (15 seconds)
- **Button Responses:**
  - Center+Lower = confirm mould placed → INJECT
  - Upper released = user abort → READY_TO_INJECT
  - Timeout = return to READY_TO_INJECT
- **Context:** CTX_MOVING_FREE
- **Interruptible:** YES, immediately via buttons

#### Injection.cpp (Inject + Hold together)
- **Purpose:** Execute injection cycle with automatic FILLING → PACKING transition
- **FILLING Phase:**
  - Position control with TRAP_TRAJ
  - Target = startPos + fillVolume
  - Auto-transition on velocity < 0.1 for >500ms
- **PACKING Phase:**
  - Position control with TRAP_TRAJ
  - Target = packStartPos + packVolume
  - Duration = packTime (from actualMouldParams)
  - Auto-transition on timeout
- **Control:** Position (Mode 3) with TRAP_TRAJ
- **Direction:** Down (positive)
- **Pressure Check:** YES (first ms to confirm mould blocked)
- **Torque Limit:** Detects if mould full before reaching position target
- **Buttons:** Upper = abort → RELEASE (only available in some phases)
- **Context:** CTX_BLOCKED
- **Note:** Both phases use actualMouldParams (fillVolume, fillSpeed, fillPressure, packVolume, packSpeed, packPressure, packTime)

### Module Integration Checklist
- [ ] All modules use non-blocking state machines (no delays, no while loops)
- [ ] All modules respect CAN_COMMAND_GAP_MS (50ms between commands)
- [ ] All modules set safety context (CTX_IDLE, CTX_MOVING_FREE, CTX_BLOCKED, CTX_PURGE)
- [ ] All modules check pressure sensor where specified
- [ ] All modules have clear button handlers with proper debouncing
- [ ] All modules return bool from update() (true = complete, false = running)
- [ ] main.cpp only handles state routing, not control logic
- [ ] LEDs updated centrally in updateLeds() based on current state

### Testing Each Module
1. **Homing:** Run full sequence, verify position resets to 0
2. **Refill:** Move to OFFSET_REFILL_GAP, check velocity smoothness
3. **Compression Mode 1:** Manual plastic block, verify contact detection and torque ramp
4. **ReadyToInject:** Verify 30s timer, observe micro-compression (no button LED change)
5. **PurgeZero:** Manual plunger control, smooth response to buttons
6. **AntiDrip:** Slow upward move, verify timeout and button interrupt
7. **Injection:** Full cycle with mould, verify auto-transition from FILLING to PACKING
8. **Integration:** Complete cycle from REFILL → COMPRESSION → READY → PURGE → ANTIDRIP → INJECT → HOLD → RELEASE