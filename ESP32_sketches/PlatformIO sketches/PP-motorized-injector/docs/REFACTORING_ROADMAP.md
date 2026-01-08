# PP-Motorized-Injector: Main.cpp Refactoring Roadmap

**Objective:** Replace monolithic FSM with 7 modular state machines, one phase at a time.

**Strategy:** Phased (Option B) - Skeleton first, then test incrementally.

---

## Quick Reference: Phase Status

| Phase | Status | Focus | Time |
|-------|--------|-------|------|
| **Phase 1** | NOT STARTED | Skeleton refactoring, no motor tests | 1-2h |
| **Phase 2** | NOT STARTED | Pre-injection (Refill, Compression, Ready) | 2-3h |
| **Phase 3** | NOT STARTED | Full injection (Purge, AntiDrip, Inject, Hold, Release) | 3-4h |
| **Phase 4** | FUTURE | Safety interrupt handlers | TBD |
| **Phase 5** | FUTURE | Display communications | TBD |

---

## Phase 1: Refactoring Skeleton (1-2 hours)

### Goal
Replace old FSM logic with new module calls. Code compiles, state routing works, NO motor movement yet.

### What to do
1. Add module `#include` statements
2. Comment out old REFILL through RELEASE cases
3. Add skeleton `if (stateEntry) Module::begin()` + `Module::update(motor)` for each state
4. Keep ERROR_STATE, INIT_HEATING, INIT_HOT_NOT_HOMED, INIT_HOMING as-is

### What NOT to do
- ❌ Don't delete old code (comment it out)
- ❌ Don't enable motor movement yet
- ❌ Don't add button handlers yet (skeleton only)

### Success Criteria
- ✅ Compiles without errors
- ✅ State transitions work via button presses
- ✅ LED updates work
- ✅ Serial output shows state changes
- ✅ NO motor movement

---

## Phase 2: Pre-Injection Prep (2-3 hours)

### Prerequisites
- Phase 1 complete and compiles

### Goal
Test basic motor movements WITHOUT mould/blocks. Validate contact detection and timer logic.

### States to enable
1. **REFILL** - Move to OFFSET_REFILL_GAP
2. **COMPRESSION Mode 1** - Velocity ramp → torque ramp, contact detection
3. **READY_TO_INJECT** - Idle waiting + 30s micro-compression timer

### Testing approach
- Set `IGNORE_NOZZLE_BLOCK = true` in config.h
- Empty barrel (no plastic)
- Test each state transition via buttons
- Monitor serial output for module state progress

### Success Criteria
- ✅ Refill smooth position move to home
- ✅ Compression travels down, detects contact
- ✅ ReadyToInject idle works, micro-compression every 30s
- ✅ All button transitions work
- ✅ Non-blocking execution (no delays)

---

## Phase 3: Full Injection Sequence (3-4 hours)

### Prerequisites
- Phase 2 complete and tested
- Actual mould + NozzleBlock required

### Goal
Complete injection cycle from REFILL through RELEASE with auto-transitions.

### States to enable
1. **PURGE_ZERO** - Manual button control (Up/Down), confirm zero
2. **ANTIDRIP** - Slow upward retract, 15s timeout
3. **INJECTION** - Auto-transition FILLING → PACKING phase
4. **HOLD** - Pack for `packTime` duration
5. **RELEASE** - Quick unload
6. **CONFIRM_MOULD_REMOVAL** - Return to READY or REFILL

### Testing approach
- Requires actual mould + NozzleBlock
- Enable pressure sensor checks
- Use default mould params from config.h
- Full cycle: REFILL → COMPRESSION → READY → PURGE → ANTIDRIP → INJECT → HOLD → RELEASE

### Success Criteria
- ✅ PurgeZero smooth manual control
- ✅ AntiDrip timeout works, button interrupt works
- ✅ Injection auto-transitions FILL→PACK
- ✅ Hold maintains pressure for correct duration
- ✅ Release unloads mould
- ✅ Full cycle completes
- ✅ Pressure sensor validates before injection

---

## Future Phases

### Phase 4: Safety Interrupt Preparation
- Add `safety.check()` to each module
- Test E-stop from any state
- Emergency abort cleanup

### Phase 5: Display Communications
**Broadcasts (ESP32 → Display):**
- Encoder position every 100ms: `"POS|<pos>|<vel>\n"`
- State changes: `"STATE|<state_name>|<timestamp>\n"`
- Compression complete: `"COMPRESS_COMPLETE|<pos>|<timestamp>\n"`

**Parsing (Display → ESP32):**
- Mould updates: `"MOULD|<params>\n"`

**Display-Side Features:**
- RefillBlocks utility (melt time tracking)
- Graphical plunger position
- Mould library selection

---

## Key Module Files

| Module | File | Purpose |
|--------|------|---------|
| Homing | `Homing.cpp` | ✅ LOCKED - 12-state homing |
| Refill | `Refill.cpp` | Move to rest position |
| Compression | `Compression.cpp` | Mode 1 (travel+compress), Mode 2 (micro) |
| Injection | `Injection.cpp` | Inject + Hold (auto-transition) |
| AntiDrip | `AntiDrip.cpp` | Slow retract with timeout |
| PurgeZero | `PurgeZero.cpp` | Manual button-controlled movement |
| ReadyToInject | `ReadyToInject.cpp` | Idle waiting + 30s micro-compression |

---

## Critical Implementation Notes

### Non-blocking execution
- ✅ All modules use non-blocking state machines
- ✅ No `delay()` or `while` loops
- ✅ CAN_COMMAND_GAP_MS = 50ms enforced

### Pressure sensor checks
- **Active on:** Compression (both modes), Injection, ReadyToInject micro
- **Inactive on:** Refill, PurgeZero, AntiDrip, Release

### Motor control by state
- **Refill:** TRAP_TRAJ position (up)
- **Compression:** VEL_RAMP → TORQUE_RAMP (down)
- **PurgeZero:** PASSTHROUGH velocity (manual)
- **AntiDrip:** PASSTHROUGH velocity (up, slow)
- **Inject:** TRAP_TRAJ position (down, torque limit)
- **Hold:** TBD (torque or position)
- **Release:** TRAP_TRAJ position (up, fast)

### Safety contexts
- **CTX_IDLE:** Normal idle (Refill, ReadyToInject)
- **CTX_MOVING_FREE:** Careful movement (Homing, AntiDrip)
- **CTX_BLOCKED:** Injection/Hold, no reversal
- **CTX_PURGE:** Manual mode (PurgeZero)

---

## Debugging Checklist

When things go wrong:

- [ ] Check serial output for state transitions
- [ ] Verify LED colors match expected state
- [ ] Check `IGNORE_NOZZLE_BLOCK` flag if pressure sensor issues
- [ ] Confirm module `begin()` called on state entry
- [ ] Check CAN command timing (50ms gap)
- [ ] Verify button debouncing (10ms)
- [ ] Check motor mode/input mode logs
- [ ] Look for blocking delays in code

---

## References

- **Architecture:** See `.github/copilot-instructions.md` (Modular State Machine Architecture section)
- **Motor Control Specs:** See copilot-instructions.md (Motor Control Modes by State table)
- **State Flow Diagram:** See copilot-instructions.md (State Flow Diagram section)
- **Module Specs:** See copilot-instructions.md (Detailed Module Specifications section)

---

**Last Updated:** 27 December 2025  
**Status:** Ready for Phase 1 Skeleton Refactoring
