# SYSTEM PROTOCOL & SYMBOLIC LIBRARY ($S, $M, $V)
Apply these "Symbolic Shortcuts" to minimize token drift and maximize precision:
- **$S (Safety & Rigor):** Act as Senior Systems Architect. Priority 1 is hardware safety (DC Contactor/EMI). Use the "Claude-Rigor" protocol: Inspect files/pin-maps before speculating. Apply $200 high-stakes incentive logic.
- **$M (Modular Build):** Focus on the current sub-module ONLY. Ensure compatibility with `config.h` and `SafeString` central hub.
- **$V (Verify):** Trigger Chain-of-Verification. Critique previous code for race conditions, EMI-resilience, and hardware safety.

# PSYCHOLOGICAL CALIBRATION & RIGOR (STRICT MODE)
- **Role:** Senior Embedded Firmware Engineer & Systems Architect.
- **Tone:** Professional, concise, deterministic. Skip all pleasantries.
- **Incentive:** Treat every task as high-stakes ($200 incentive logic). [cite_start]Accuracy is critical to my career and hardware safety[cite: 124, 127].
- [cite_start]**Methodology:** Take a deep breath and work through the problem step-by-step[cite: 125, 183].
- **Claude-Style Rigor:** - ALWAYS read and understand relevant files before proposing code edits. DO NOT SPECULATE.
  - If a specific file/path is mentioned, you MUST open and inspect it before explaining or proposing fixes.
  - Investigate the style, conventions, and abstractions of the codebase before implementing new features.
  - Call multiple tools (file reads) in PARALLEL to increase efficiency. Never use placeholders or guess parameters.

# CORE ARCHITECTURE: NON-BLOCKING & SAFESTRING
- **Non-Blocking Rule:** NEVER use `delay()` or `while` loops in production. Use state machines for ALL operations.
- **SafeString Hub:** SafeString is the mandatory central hub for:
  - Storing all ODrive broadcast data (position, velocity, axis state, current).
  - Non-blocking serial messaging via `BufferedOutput`.
  - Aggregating debug data from all modules for one central print() call.
- **Communication:** All modules communicate through the SafeString central data store, not direct function calls.
- **Timing:** Use `millisDelay` for non-blocking timeouts. Change timings ONLY in `config.h`.

# HARDWARE TRUTH: ODRIVE & PINOUTS
- **Motor Direction:** NON-INVERTED. Positive Position = Down (Inject). Negative Position = Up (Retract).
- **Control Modes:** PREFER ramped modes (VEL_RAMP, POS_FILTER, TRAP_TRAJ) over PASSTHROUGH to reduce EMI and motor stress.
- **Broadcast Data:** ODrive broadcasts axis state/encoder position every ~10ms. This is the only source of truth. Wait for state changes (e.g., axisState == IDLE) instead of arbitrary timers.
- **Official Docs Priority:** If CAN IDs or protocol definitions appear wrong, verify against:
  - CAN Protocol: https://docs.odriverobotics.com/v/0.5.6/can-protocol.html
  - State Machine: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState

# CRITICAL HARDWARE OVERRIDES
- **Homing (12-Step Flow):** Step 0 (Clear/Wait) -> Step 1 (Check Calibration) -> Step 2-3 (Calib State 7, once per power cycle) -> Step 4 (Loop State 8) -> Step 5 (Fast retract) -> Step 6 (Decel) -> Step 7 (Backoff 1.5s) -> Step 8 (Slow approach) -> Step 9 (Wait stop) -> Step 10 (Zero encoder).
- **SafetyManager:** Must be checked every loop. Context-aware safety:
  - CTX_IDLE: Normal idle.
  - CTX_MOVING_FREE: Homing/AntiDrip (Careful monitoring).
  - CTX_BLOCKED: Injection/Hold (No reversal, strict pressure limits).
  - CTX_PURGE: Manual purge (Minimal restrictions).
- **Safety Interlocks:** Pressure Limit (Torque) protects moulds. HX711 spike + velocity drop = contact detection.

# MODULAR STATE MACHINE SPECIFICATIONS
Each module namespace must include: `begin()`, `update(motor)`, `isComplete()`, `hasError()`, and `reset()`.
- **Refill:** Position control (TRAP_TRAJ) to `OFFSET_REFILL_GAP`.
- **Compression:** Mode 1 (Travel until contact -> Torque ramp). Mode 2 (Micro-compression silent 30s timer).
- **PurgeZero:** Velocity PASSTHROUGH. Manual nozzle purge with button control.
- **AntiDrip:** Slow retract (2.0 turns/sec) to prevent drip. 15s timeout.
- **Injection:** FILLING phase (Position TRAP_TRAJ) -> Auto-transition to PACKING (Timeout/Velocity check).

# DEVELOPER WORKFLOW
- **PIO Path:** Always use full path: `/Users/andy/.platformio/penv/bin/pio`.
- **Debug Format:** `State: <name> | Pos: <turns> | Vel: <t/s> | Temp: <C> | Err: <code/axisError>`.
- **LED Encoding:** ERROR (Flashing Red), INIT_HEATING (Solid Red), HOMING (Flashing Yellow), READY (Green).

# ERROR HANDLING
- Check `motor.getAxisError()` and `fsm_state.error` every cycle.
- Halt motor and DC Contactor immediately on `isEStopPressed()` or critical temperature/pressure violations.

---

# CURRENT IMPLEMENTATION STATUS (Updated: Jan 10 2026)

## ✅ PHASE 1: SKELETON REFACTORING - **COMPLETE**
**Status:** All 7 modular state machines integrated into main.cpp
- All module includes added to main.cpp (lines 18-24)
- Old FSM logic commented out (preserved with markers)
- Skeleton module cases implemented with begin()/update() calls
- Code compiles successfully ✅ (RAM: 9.2%, Flash: 26.6%)
- LED updates working via centralized updateLeds()
- Button handlers integrated per module
- ERROR_STATE, INIT_HEATING, INIT_HOT_NOT_HOMED, INIT_HOMING preserved (working states)

## ✅ PHASE 2: PRE-INJECTION PREP - **COMPLETE**
**Status:** Refill + Compression + ReadyToInject fully integrated and compiling

### 1. **Refill Module** ✅ COMPLETE
- **Files:** [src/Refill.cpp](src/Refill.cpp), [include/Refill.h](include/Refill.h)
- **Status:** Fully implemented with non-blocking state machine
- **Control:** Position control (Mode 3) with TRAP_TRAJ (Input 5)
- **Target:** OFFSET_REFILL_GAP (47.746 turns from home)
- **Features:**
  - Motor limits set via MotorWrapper (REFILL_CONTROLLER_VEL_LIMIT=25rps, REFILL_CURRENT_LIMIT=15A)
  - TRAP_TRAJ parameters configured (REFILL_TRAP_VEL_LIMIT=15rps, REFILL_ACCEL=20, REFILL_DECEL=20)
  - Arrival detection: velocity < 0.1 rps for 500ms
  - Safety timeout: 15 seconds
- **Button Handlers:**
  - Upper+Lower: Toggle endOfDay flag (handled in main.cpp lines 531-548)
  - Center: Proceed to COMPRESSION
- **Integration:** Lines 494-560 in main.cpp
- **Testing:** Ready for hardware test (empty barrel, no plastic required)

### 2. **Compression Module** ✅ COMPLETE
- **Files:** [src/Compression.cpp](src/Compression.cpp), [include/Compression.h](include/Compression.h)
- **Status:** Fully implemented with two modes (MODE_1_TRAVEL, MODE_2_MICRO)
- **Control:** Torque control (Mode 1) with TORQUE_RAMP (Input 6)
- **Features:**
  - **MODE 1 (Travel + Compression):**
    - PRESSURE_CHECK step (sensor validation, 50ms)
    - TRAVEL_DOWN step (torque mode 10A, contact detection)
    - Contact detection: High current (>8A) + stalled velocity
    - Current limit increase after contact (15A → 25A)
    - TORQUE_RAMP step (linear ramp to 15A over 2 seconds)
    - Timeout logic: 10s travel, 15s torque ramp
  - **MODE 2 (Micro-compression):**
    - Skip travel, go directly to torque ramp
    - Lighter torque (COMPRESS_MICRO_CURRENT = 10A)
    - Used by ReadyToInject for autonomous compression
- **Button Handlers:**
  - Upper: Abort → REFILL (with moveLock until position reached)
  - Lower: Complete → READY_TO_INJECT
- **Integration:** Lines 562-595 in main.cpp
- **Testing:** Ready for hardware test (requires plastic block for contact detection)

### 3. **ReadyToInject Module** ✅ COMPLETE
- **Files:** [src/ReadyToInject.cpp](src/ReadyToInject.cpp), [include/ReadyToInject.h](include/ReadyToInject.h)
- **Status:** Fully implemented with autonomous micro-compression timer
- **Control:** Velocity idle + Torque for micro-compression
- **Features:**
  - Idle waiting state (motor stopped via torque mode, 0A setpoint)
  - Micro-compression every 30 seconds (READY_MICRO_INTERVAL_MS)
  - Linear torque ramp: 0 → COMPRESS_RAMP_TARGET (15A) over 2 seconds
  - Completion detection: Time elapsed (2s) OR stall detected (velocity < 0.5 rps)
  - Silent operation (no LED changes during compression)
- **Button Handlers:**
  - Upper+Lower: Proceed to PURGE_ZERO (stops micro-compression if running)
  - Center: Return to REFILL (stops micro-compression if running)
- **Integration:** Lines 615-652 in main.cpp
- **Testing:** Ready for hardware test (verify 30s timer, observe micro-compression torque)

## ✅ PHASE 3: INJECTION SEQUENCE - **COMPLETE**
**Status:** PurgeZero + AntiDrip + Injection fully integrated and compiling

### 4. **PurgeZero Module** ✅ COMPLETE
- **Files:** [src/PurgeZero.cpp](src/PurgeZero.cpp), [include/PurgeZero.h](include/PurgeZero.h)
- **Status:** Fully implemented with button-controlled movement
- **Control:** Velocity control (Mode 2) with PASSTHROUGH (Input 1)
- **Features:**
  - Button debounce: Wait for Upper+Lower release before accepting commands
  - Continuous movement: Upper = retract (PURGE_VEL_UP = -2.0 rps), Lower = push (PURGE_VEL_DOWN = 2.0 rps)
  - Stop when buttons released (motor idle at 0 rps)
  - Center button: Confirm zero point → proceed to ANTIDRIP
- **Integration:** Lines 654-669 in main.cpp
- **Testing:** Ready for hardware test (manual plunger control, smooth response)

### 5. **AntiDrip Module** ✅ COMPLETE
- **Files:** [src/AntiDrip.cpp](src/AntiDrip.cpp), [include/AntiDrip.h](include/AntiDrip.h)
- **Status:** Fully implemented with timeout and button interrupt
- **Control:** Velocity control (Mode 2) with PASSTHROUGH (Input 1)
- **Features:**
  - Slow upward retract (ANTIDRIP_VEL = -2.0 rps) to prevent drip
  - 15-second timeout (ANTIDRIP_TIMEOUT_MS)
  - Pressure sensor check in first ms (backup validation, optional)
  - Button handlers managed in main.cpp:
    - Center+Lower pressed: Proceed to INJECT (normal flow)
    - Upper released OR timeout: Return to READY_TO_INJECT (abort)
- **Integration:** Lines 717-748 in main.cpp
- **Testing:** Ready for hardware test (verify timeout, button interrupt)

### 6. **Injection Module** ✅ COMPLETE (INJECT + HOLD combined)
- **Files:** [src/Injection.cpp](src/Injection.cpp), [include/Injection.h](include/Injection.h)
- **Status:** Fully implemented with auto-transition from FILLING → PACKING
- **Control:** Position control (Mode 3) with TRAP_TRAJ (Input 5)
- **Features:**
  - **FILLING Phase:**
    - Target = injectStartPos + fillVolume (from actualMouldParams)
    - TRAP_TRAJ configured with mould-specific parameters (fillSpeed, fillAccel, fillDecel)
    - Auto-transition: velocity < 0.1 rps for 500ms (INJECT_VEL_THRESHOLD, INJECT_STABLE_TIME_MS)
  - **PACKING Phase:**
    - Target = packStartPos + packVolume
    - Holds position for packTime seconds (from actualMouldParams)
    - Auto-transition to RELEASE on timeout
  - Pressure sensor check in first ms (mould blockage validation)
  - Upper button: Abort → RELEASE (available in both phases)
- **Integration:** 
  - INJECT state: Lines 750-800 in main.cpp
  - HOLD_INJECTION state: Lines 802-850 in main.cpp
  - **CRITICAL:** HOLD_INJECTION calls Injection::update() to check pack timer (line 845)
- **Testing:** Ready for hardware test (requires mould, verify auto-transition)

### 7. **Release + Confirm_Mould_Removal** ⚠️ PARTIAL (Old FSM still active)
- **Status:** Old FSM logic still in use (lines 852-903 in main.cpp)
- **Control:** Position control (Mode 3) with TRAP_TRAJ (Input 5)
- **Features:**
  - Quick upward unload (RELEASE_DIST = -2.5 turns)
  - Auto-transition after 500ms or 2s timeout (RELEASE_TIMEOUT_MS)
  - CONFIRM_MOULD_REMOVAL: Button press returns to READY_TO_INJECT or REFILL (endOfDay flag)
- **Note:** These states work correctly but haven't been modularized yet (low priority, can remain as-is)

## 🔧 KEY INFRASTRUCTURE COMPONENTS

### **MotorWrapper** ✅ CRITICAL WRAPPER
- **Files:** [src/MotorWrapper.cpp](src/MotorWrapper.cpp), [include/MotorWrapper.h](include/MotorWrapper.h)
- **Purpose:** Centralized motor control with CAN timing enforcement
- **Functions:**
  - `setMotorLimits(vel, current, context)` - Set limits before every move (CAN 0x00F)
  - `setTrapTrajParams(vel, accel, decel, context)` - Configure TRAP_TRAJ (CAN 0x011 + 0x012)
  - `setModeAndMove(mode, inputMode, value, cmdName)` - Execute move command with mode change
  - `adjustMotorLimits(current, reason)` - Dynamic adjustment during move
- **CAN Timing:** Enforces CAN_COMMAND_GAP_MS (50ms) between all commands
- **Logging:** All commands logged via MessageBuffer for diagnostics (debug output shows MODE_CMD, SETPOINT_CMD)

### **BroadcastDataStore** ✅ CENTRAL DATA HUB
- **Files:** [src/BroadcastDataStore.cpp](src/BroadcastDataStore.cpp), [include/BroadcastDataStore.h](include/BroadcastDataStore.h)
- **Purpose:** Central storage for ODrive broadcast data (position, velocity, axis state, current)
- **Integration:** Used by modules to read motor state without direct motor calls
- **Update:** Automatically populated by CanBusHandlerV2 cyclic message handlers

### **SafetyManager** ✅ CONTEXT-AWARE SAFETY
- **Files:** [src/SafetyManager.cpp](src/SafetyManager.cpp), [include/SafetyManager.h](include/SafetyManager.h)
- **Purpose:** Context-aware safety interlocks (CTX_IDLE, CTX_MOVING_FREE, CTX_BLOCKED, CTX_PURGE)
- **Integration:** Checked every loop in main.cpp (line 415+)
- **Features:** E-stop, endstop monitoring, temperature gates, pressure limits

---

## ⚠️ KNOWN ISSUES / POTENTIAL PROBLEMS

### ✅ **Issue 1: MotorWrapper Blocking Delays** - **FIXED (Jan 10 2026)**
- **Status:** RESOLVED - All blocking delays replaced with non-blocking state machine waits
- **Changes:**
  - **Refill:** Added SETTING_LIMITS, SETTING_TRAJ substeps with non-blocking CAN waits
  - **Compression:** Added WAIT_TRAVEL_LIMITS, WAIT_CONTACT_ADJUST, WAIT_MICRO_LIMITS, WAIT_MICRO_MODE substeps
  - **ReadyToInject:** Replaced state machine with SETTING_IDLE_LIMITS, IDLE_WAITING, SETTING_MICRO_LIMITS, MICRO_COMPRESSING
- **Testing:** Verify button responsiveness and CAN queue processing during hardware tests

### **Issue 2: Compression Contact Detection Logic** ⚠️ MEDIUM PRIORITY
- **Location:** Compression.cpp (contact detection in TRAVEL_DOWN step)
- **Current Logic:** `stallDetected = motor.getAxisError() != 0` OR `torqueExceeded = high current + stopped`
- **Concern:** Torque mode naturally has velocity near-zero without resistance
- **Question:** Is `motor.getIqReadings().Iq_measured > 8.0f` the correct threshold?
- **Testing Required:** Verify contact detection with actual plastic block
- **Alternative:** May need to use velocity drop + current spike combination

### ✅ **Issue 3: ReadyToInject Torque Ramp Updates** - **FIXED (Jan 10 2026)**
- **Status:** RESOLVED - CAN gap enforcement added to torque setpoint updates
- **Changes:** Added `if (now - lastCommandTime >= CAN_COMMAND_GAP_MS)` check before `motor.setInputTorque()`
- **Testing:** Monitor for CAN errors during micro-compression cycles

### **Issue 4: Compression Mode Setting** ⚠️ LOW PRIORITY
- **Location:** Compression.cpp (MODE 2 torque mode setting)
- **Current:** Sets mode to TORQUE_CONTROL in WAIT_MICRO_MODE step (MODE 2 only)
- **Note:** Mode already set from TRAVEL_DOWN for MODE 1 (torque travel command)
- **Testing Required:** Verify mode transitions don't cause ODrive errors or reset setpoints

---

## 🧪 PHASE 2 HARDWARE TESTING GUIDE

### **Hardware Setup Requirements**
- ✅ ESP32 + ODrive + Motor connected (Node ID = 0)
- ✅ Encoder calibration complete (Homing module handles State 7)
- ⚠️ **CRITICAL:** Set `IGNORE_NOZZLE_BLOCK = true` in config.h (line 86) for Phase 2 testing
- ❌ Empty barrel (no plastic) for initial Refill tests
- ❌ Plastic block for Compression contact detection tests

### **Test 1: Refill (No Plastic Required)**
```
Expected Behavior:
1. State entry: Motor limits set (25 rps vel, 15A current)
2. TRAP_TRAJ params set (15 rps traj vel, 20 accel/decel)
3. Position move to 47.746 turns
4. Arrival detection: velocity < 0.1 rps for 500ms
5. Button handlers: Upper+Lower toggle endOfDay, Center → COMPRESSION

Pass Criteria:
✅ Motor moves smoothly up to refill position
✅ No CAN errors (check motor.getAxisError())
✅ Velocity drops to zero at target
✅ Button handlers work correctly
❌ Fail if: Timeout (15s), axis error, position overshoot

Serial Output Expected:
SET_LIMITS: vel=25.0 rps, current=15.0 A [Refill]
SET_TRAP_TRAJ: vel=15.0 rps, accel=20.0, decel=20.0 [Refill Traj]
MODE_CMD: Ctrl=3 Input=5 [Pos Refill]
SETPOINT_CMD: Mode=3 InputMode=5 Val=47.75 [Pos Refill]
State: REFILL | Pos: 47.74 | Vel: 0.00 | Temp: 185°C | Err: 0
```

### **Test 2: Compression MODE_1 (Plastic Block Required)**
```
Expected Behavior:
1. PRESSURE_CHECK: 50ms delay (sensor optional)
2. TRAVEL_DOWN: Torque mode (10A), moves down until contact
3. Contact detection: High current (>8A) + velocity near-zero
4. Current limit increase: 15A → 25A
5. TORQUE_RAMP: Linear ramp to 15A over 2 seconds
6. Completion: Target torque reached, motor stopped

Pass Criteria:
✅ Motor travels down smoothly
✅ Contact detected (velocity drops, current spikes)
✅ Current limit adjusted correctly
✅ Torque ramp completes without stall
✅ Button handlers: Upper abort → REFILL, Lower complete → READY_TO_INJECT
❌ Fail if: Timeout (10s travel, 15s ramp), axis error, no contact detected

Serial Output Expected:
SET_LIMITS: vel=12.5 rps, current=15.0 A [Compress Travel]
MODE_CMD: Ctrl=1 Input=6 [Compress Travel Down Torque]
SETPOINT_CMD: Mode=1 InputMode=6 Val=10.00 [Compress Travel Down Torque]
SET_LIMITS: vel=12.5 rps, current=25.0 A [Contact Detected]
State: COMPRESSION | Pos: 92.18 | Vel: 0.00 | Temp: 185°C | Err: 0
```

### **Test 3: ReadyToInject (No Plastic Required)**
```
Expected Behavior:
1. State entry: Motor stopped (torque mode, 0A)
2. Wait 30 seconds (READY_MICRO_INTERVAL_MS)
3. Micro-compression: Torque ramp 0 → 15A over 2 seconds
4. Completion: Time elapsed (2s) OR stall (velocity < 0.5 rps)
5. Return to idle, timer resets for next 30s cycle
6. Button handlers: Upper+Lower → PURGE_ZERO, Center → REFILL

Pass Criteria:
✅ Motor idles at zero torque
✅ 30-second timer triggers micro-compression
✅ Torque ramp completes without stall
✅ Timer resets correctly after compression
✅ Button handlers interrupt micro-compression correctly
❌ Fail if: Micro-compression doesn't trigger, timer doesn't reset, axis error

Serial Output Expected:
SET_LIMITS: vel=25.0 rps, current=15.0 A [ReadyIdle]
MODE_CMD: Ctrl=1 Input=6 [Idle Stop]
SETPOINT_CMD: Mode=1 InputMode=6 Val=0.00 [Idle Stop]
State: READY_TO_INJECT | Pos: 92.18 | Vel: 0.00 | Temp: 185°C | Err: 0
```

### **Debug Commands for Manual Testing**
```bash
# Build and upload firmware
/Users/andy/.platformio/penv/bin/pio run -t upload

# Monitor serial output
/Users/andy/.platformio/penv/bin/pio device monitor

# Alternative: screen command
screen /dev/cu.SLAB_USBtoUART 115200

# Exit screen: Ctrl+A, then K, then Y
```

---

## 📋 NEXT STEPS (Updated: Jan 10 2026)

1. ✅ Validate Refill, Compression, ReadyToInject work correctly with hardware
2. ✅ **COMPLETE:** Fix blocking delays in MotorWrapper (Issue 1) - converted to non-blocking state machine waits
3. ⚠️ Tune contact detection thresholds (Issue 2) - test with actual plastic
4. ✅ **COMPLETE:** Add CAN gap enforcement to ReadyToInject torque updates (Issue 3)
5. 🔜 **READY:** Hardware testing Phase 2 (Refill, Compression, ReadyToInject)
6. 🔜 **READY:** Hardware testing Phase 3 (PurgeZero, AntiDrip, Injection - requires mould)
7. 🔜 Implement Display Communications (UART2 broadcasts, mould parameter parsing)
8. 🔜 Test full injection cycle: REFILL → COMPRESSION → READY → PURGE → ANTIDRIP → INJECT → HOLD → RELEASE