# PHASE 2: PRE-INJECTION PREP TEST PLAN

**Objective:** Test basic motor movements without moulds or blocks. Validate Refill, Compression (Mode 1 travel), and ReadyToInject state machines with empty barrel.

**Status:** READY FOR TESTING ✅ - Phase 1.7 Complete, System Production-Ready

**Build:** Successfully compiled with all 6 modular state machines integrated + ODrive error reporting fixes

**Update:** January 30, 2026 - Phase 1.7 error reporting debug completed, system now production-ready

---

## TEST ENVIRONMENT SETUP

### Hardware Configuration
- **Machine:** PP Motorized Injector (ESP32 + ODrive 3.6)
- **Motor:** ODesc 4.2 (Node ID 0), inverted direction
- **Nozzle:** Empty (no plastic block, no mould)
- **Safety:** Sensors operational but not critical for Phase 2

### Software Configuration
- **IGNORE_NOZZLE_BLOCK:** `true` in config.h ✅
- **Firmware:** Commit 76f652f (Phase 1 skeleton refactored)
- **Debug Output:** Serial 115200 baud, 1Hz status report
- **Temperature:** Should be > 60°C to pass INIT_HEATING gate

---

## PRE-TEST CHECKLIST

- [ ] Motor contactor powered
- [ ] CAN bus connected (ODrive responds)
- [ ] Temperature > 60°C (heating active or pre-heated)
- [ ] Serial monitor ready (115200 baud)
- [ ] No mould in place (empty barrel only)
- [ ] Top/bottom endstops functional (debounced)
- [ ] All buttons debounced and responsive
- [ ] LED rings visible and brightness adequate

---

## TEST SEQUENCE

### Phase 2.1: INIT_HEATING → INIT_HOT_NOT_HOMED

**Expected Behavior:**
1. System boots → ERROR_STATE or INIT_HEATING
2. Wait for temperature ≥ TEMP_CRITICAL (60°C or configured value)
3. Auto-transition to INIT_HOT_NOT_HOMED
4. LEDs: All YELLOW (ready for homing)

**Actions:**
- Monitor temperature in serial output
- Observe LED color (should turn yellow when hot)
- **Log:** Temperature reached, state transitioned

**Success Criteria:**
- ✅ Temperature gate works correctly
- ✅ State transition occurs automatically
- ✅ LEDs indicate correct state

---

### Phase 2.2: INIT_HOMING (Existing Locked Module)

**Expected Behavior:**
1. Press Upper button → Begin homing sequence
2. Homing module runs 12-step non-blocking sequence
3. Final step: Encoder reset to 0
4. Auto-transition to REFILL state
5. LEDs: YELLOW flashing during homing, GREEN + BLUE when complete

**Actions:**
- Press Upper button when in INIT_HOT_NOT_HOMED
- Monitor serial output for each homing step
- Observe motor retracts to top endstop smoothly
- Verify no stalling or overcurrent errors

**Success Criteria:**
- ✅ Motor retracts up (negative velocity)
- ✅ Top endstop detected correctly
- ✅ Encoder resets (position reads 0.0)
- ✅ Transitions to REFILL automatically
- ✅ Position stable at 0.0 turns

---

### Phase 2.3: REFILL STATE

**Expected Behavior:**
1. Auto-entered after homing completes
2. Move plunger DOWN to OFFSET_REFILL_GAP (safe rest position)
3. Use Position Control with TRAP_TRAJ (smooth ramps)
4. Wait for velocity < 0.1 turns/sec (position reached)
5. LEDs: CENTER = GREEN, UPPER/LOWER = BLACK (or BLUE if endOfDay flag set)

**Actions:**
- Observe position target in serial output (should be OFFSET_REFILL_GAP)
- Monitor velocity ramp (should be smooth, not jerky)
- Press Upper+Lower buttons → Toggle endOfDay flag (LEDs change UPPER/LOWER to BLUE)
- Press Center button → Proceed to COMPRESSION

**Success Criteria:**
- ✅ Position ramps smoothly to target
- ✅ Velocity < 0.1 when settled
- ✅ endOfDay toggle works (LEDs update)
- ✅ Center button transitions to COMPRESSION
- ✅ No position overshoot

---

### Phase 2.4: COMPRESSION STATE (MODE_1_TRAVEL)

**Expected Behavior:**
1. Compression::begin(MODE_1_TRAVEL) called
2. Step 1: Set velocity control mode, prepare ramp
3. Step 2: Travel DOWN (positive velocity) with VEL_RAMP
   - Target velocity: SPEED_COMPRESSION_TRAVEL (~10 turns/sec)
   - Continue until contact detected (velocity drops) or timeout
4. Step 3: Contact detection (empty barrel, should timeout after ~5-10s)
   - Velocity threshold: < 0.5 turns/sec
   - Pressure check: IGNORE_NOZZLE_BLOCK = true (skip pressure)
5. Step 4: Apply torque ramp (even though no block present)
6. Step 5: Timeout after ~10s → Return to REFILL (no plastic detected)
7. LEDs: UPPER/LOWER = RED, CENTER = BLACK, RING = RED (compression active)

**Actions:**
- Observe motor begin moving DOWN (positive velocity)
- Monitor velocity in serial output (should rise then plateau)
- Wait for velocity to drop (contact detection trigger)
- If no plastic detected within timeout → Auto-return to REFILL
- Alternative: Press Upper button → Abort and return to REFILL
- Alternative: Press Lower button → Force complete and go to READY_TO_INJECT

**Success Criteria:**
- ✅ Motor moves DOWN smoothly (positive velocity)
- ✅ Velocity ramp works (no jerky movement)
- ✅ Timeout triggers after 5-10 seconds (no block = no contact)
- ✅ Auto-transitions back to REFILL on timeout
- ✅ Upper button abort works
- ✅ Lower button force-complete works
- ✅ No overcurrent errors from ODrive
- ✅ Position update in real-time (every 100ms)

**Debug Output Expected:**
```
[COMPRESSION      ] T: XX P:      0 | OD:8 Err:0x00 | P:12.34 V: 8.50 | Ctrl:Velocity Input:VelRamp | Cmd:Vel Travel
```

---

### Phase 2.5: READY_TO_INJECT STATE (Idle + Micro-Compression Timer)

**Expected Behavior:**
1. Auto-entered after COMPRESSION completes
2. Idle waiting state - motor stationary
3. Every TIME_AUTO_COMPRESS seconds (30s), trigger micro-compression:
   - Change to torque control (TORQUE_RAMP input mode)
   - Apply small torque ramp (~2 seconds)
   - Return to idle
4. LEDs: UPPER/LOWER = GREEN, CENTER = YELLOW, RING = GREEN

**Actions:**
- Let system idle for 40 seconds (to observe one micro-compression cycle)
- Monitor serial output for micro-compression trigger messages
- Observe velocity briefly becomes positive during compression
- Observe micro-compression completes silently (no state change)
- Wait for button input:
  - Upper+Lower: Proceed to PURGE_ZERO
  - Center: Return to REFILL

**Success Criteria:**
- ✅ Idle state maintains (motor stopped, velocity = 0.0)
- ✅ Micro-compression triggers every 30s (check timestamps)
- ✅ Compression applies without changing state
- ✅ Velocity spikes briefly then returns to 0
- ✅ Upper+Lower button proceeds to PURGE_ZERO
- ✅ Center button returns to REFILL
- ✅ No LED flashing during micro-compression (silent operation)

**Timer Validation:**
- Measure time between micro-compression events (should be ~30s)
- Log serial timestamps to verify 30 ± 2 second interval

---

### Phase 2.6: PURGE_ZERO STATE (Manual Button Control)

**Expected Behavior:**
1. Velocity control mode (PASSTHROUGH - immediate response)
2. Wait for button release (debounce)
3. Upper button = retract UP (negative velocity)
4. Lower button = push out DOWN (positive velocity)
5. Center button = confirm zero point (transition to ANTIDRIP)
6. LEDs: UPPER/LOWER = YELLOW, CENTER = GREEN, RING = YELLOW

**Actions:**
- Hold Upper button → Motor retracts upward (smooth deceleration)
- Release Upper button → Motor stops immediately
- Hold Lower button → Motor pushes downward
- Release Lower button → Motor stops immediately
- Center button → Confirm and transition to ANTIDRIP

**Success Criteria:**
- ✅ Upper button movement smooth (no lag)
- ✅ Lower button movement smooth (no lag)
- ✅ Motor stops immediately when released
- ✅ Center button transitions to ANTIDRIP
- ✅ Velocity matches button state (positive/negative/zero)

---

### Phase 2.7: ANTIDRIP STATE (Slow Retract with Timeout)

**Expected Behavior:**
1. Slow upward retract (negative velocity, SPEED_ANTIDRIP ~2 turns/sec)
2. 15-second timeout (TIME_ANTIDRIP_TIMEOUT = 15000ms)
3. User can interrupt:
   - Center+Lower button → Proceed to INJECT (confirm mould placed)
   - Upper button (released) → Abort to READY_TO_INJECT
4. Timeout expires → Return to READY_TO_INJECT
5. LEDs: UPPER = RED, CENTER/LOWER = GREEN, RING = RED

**Actions:**
- Observe motor retracting slowly UP
- Monitor timeout counter (15 seconds)
- Option A: Press Center+Lower → Proceed to INJECT
- Option B: Press Upper button → Abort to READY_TO_INJECT
- Option C: Wait for timeout → Return to READY_TO_INJECT

**Success Criteria:**
- ✅ Slow retract at SPEED_ANTIDRIP (smooth, steady)
- ✅ Center+Lower button triggers INJECT transition
- ✅ Upper button abort works immediately
- ✅ Timeout (15s) triggers READY_TO_INJECT return
- ✅ Velocity consistent and non-zero during move
- ✅ Motor stops immediately after timeout/button

---

## OBSERVATIONAL METRICS

During all tests, monitor and record:

| Metric | Expected Range | Measurement Method |
|--------|-----------------|-------------------|
| Velocity Smoothness | Ramp 0→X → X→0 (no jumps) | Serial output every 100ms |
| Settling Time | < 2s to reach target position | Observe velocity trend to < 0.1 |
| Encoder Position | Smooth increment/decrement | Serial output MotPos field |
| Temperature Stability | ± 2°C over test duration | Observe T: field in debug output |
| Motor Current | < 2A (idle), < 5A (moving) | Monitor if available (not in Phase 2 spec) |
| CAN Responsiveness | < 100ms latency | Check ODrive state updates in broadcast |
| LED Update Latency | < 500ms after state change | Visual observation |
| Button Debounce | 10ms (Bounce2 library) | Should be imperceptible |

---

## ERROR SCENARIOS & RECOVERY

| Scenario | Cause | Recovery |
|----------|-------|----------|
| **E-Stop Triggered** | Manual E-stop or safety violation | Press Center button in ERROR_STATE |
| **Overcurrent Error** | Motor stall or load | Reset error, check for mechanical binding |
| **Temperature Low** | Not heated enough | Wait for temperature > TEMP_CRITICAL |
| **Endstop Not Detected** | Sensor not triggered or wired wrong | Check endstop wiring, manual trigger test |
| **Position Overshoot** | Velocity ramp too aggressive | Reduce SPEED_COMPRESSION_TRAVEL in config.h |
| **No Motor Movement** | Motor contactor off or CAN failure | Check contactor relay, CAN bus communication |
| **Velocity Doesn't Zero** | Velocity control mode stuck | Transition to different state, reset FSM |

---

## EXPECTED SERIAL OUTPUT (1Hz STATUS REPORT)

```
[COMPRESSION      ] T: 72 P:      0 | OD:8 Err:0x00 | P:15.23 V: 9.45 | Ctrl:Velocity Input:VelRamp | Cmd:Vel Travel

[Breakdown]
State: COMPRESSION
Temperature: 72°C
Pressure: 0 (empty nozzle, expected)
ODrive AxisState: 8 (CLOSED_LOOP_CONTROL)
ODrive Error: 0x00 (no error)
MotPos: 15.23 turns
MotVel: 9.45 turns/sec (downward travel)
ControlMode: Velocity
InputMode: VelRamp (velocity with ramping)
LastCommand: "Vel Travel"
```

---

## PHASE 2 SUCCESS CRITERIA (Overall)

**All of the following must pass:**

1. ✅ **Homing:** Completes with encoder reset to 0
2. ✅ **Refill:** Smooth position move to OFFSET_REFILL_GAP
3. ✅ **Compression:** Velocity ramp down works, contact detection or timeout triggers
4. ✅ **ReadyToInject:** Idle + 30s micro-compression timer verified
5. ✅ **PurgeZero:** Manual button control responsive and smooth
6. ✅ **AntiDrip:** Slow retract with timeout/abort options work
7. ✅ **Button Transitions:** All state transitions via buttons functional
8. ✅ **Non-Blocking:** No delays observed, FSM responsive to input
9. ✅ **Motor Control:** All movement smooth, no jerky transitions
10. ✅ **Serial Output:** 1Hz debug reports consistent and accurate

**If all 10 criteria pass → PHASE 2 COMPLETE, proceed to Phase 3**

---

## NEXT STEPS (PHASE 3)

Once Phase 2 passes:
- Install actual nozzle block in barrel
- Add mould to machine
- Test Injection cycle (FILLING → PACKING phase auto-transition)
- Test Release and Confirm states
- Validate pressure sensor detects block presence
- Full cycle from REFILL → COMPRESSION → READY → PURGE → ANTIDRIP → INJECT → HOLD → RELEASE

---

## REFERENCE DOCUMENTS

- **Architecture:** [copilot-instructions.md](../.github/copilot-instructions.md) - Comprehensive design spec
- **Modules:** See `include/*.h` and `src/*.cpp` for implementation details
- **Config:** [config.h](./include/config.h) - All timing constants and speeds
- **Motor Specs:** ODesc 4.2, inverted direction, TURNS_PER_CM3_VOL in config.h

---

**Created:** Phase 1 Completion (2025-12-27)  
**Test Status:** Ready for Hardware Validation  
**Build:** Commit 76f652f (Skeleton refactoring complete)
