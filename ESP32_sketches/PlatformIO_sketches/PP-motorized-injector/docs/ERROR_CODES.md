# ODrive Error Codes & Diagnostics

This document describes ODrive error codes that are broadcast via CAN cyclic messages.

**Active Error Messages (10ms interval):**
- `CYCLIC_MOTOR_ERROR` (0x03): Motor-specific error flags
- `CYCLIC_ENCODER_ERROR` (0x04): Encoder-specific error flags
- `CYCLIC_CONTROLLER_ERROR` (0x1D): Controller logic error flags

All error codes are stored in BroadcastDataStore and logged via SerialMessaging at 1Hz.

---

## Motor Error (CYCLIC_MOTOR_ERROR 0x03)

**Cyclic Message:** 100ms broadcast interval (configurable)

**Byte Layout:** `motor_error` (uint32_t)

### Error Flags (from ODrive 0.5.6 firmware)

| Bit | Hex Value | Flag Name | Meaning |
|-----|-----------|-----------|---------|
| 0 | 0x0000_0001 | PHASE_RESISTANCE_OUT_OF_RANGE | Motor phase resistance not calibrated or out of expected range |
| 1 | 0x0000_0002 | PHASE_INDUCTANCE_OUT_OF_RANGE | Motor phase inductance not calibrated or out of expected range |
| 2 | 0x0000_0004 | ADC_FAILED | ADC sampling failed (analog-to-digital conversion error) |
| 3 | 0x0000_0008 | DRV_FAULT | Motor driver (DRV8301) reported a fault (overtemp, overcurrent, etc.) |
| 4 | 0x0000_0010 | CONTROL_DEADLINE_MISSED | Control loop ran slower than expected (firmware issue) |
| 5 | 0x0000_0020 | NOT_IMPLEMENTED_REMOVED | Removed/deprecated error flag |
| 6 | 0x0000_0040 | UNKNOWN_PHASE_INDUCTANCE | Phase inductance was not measured |
| 7 | 0x0000_0080 | UNKNOWN_PHASE_RESISTANCE | Phase resistance was not measured |
| 8 | 0x0000_0100 | UNKNOWN_TORQUE_CONSTANT | Torque constant (Kt) not measured |
| 9 | 0x0000_0200 | NOT_CALIBRATED | Motor not fully calibrated (need full calibration cycle) |
| 10 | 0x0000_0400 | PHASE_CURRENT_OUT_OF_RANGE | Motor phase current reading out of range |
| 11 | 0x0000_0800 | PHASE_CURRENT_MEASUREMENT_TIMEOUT | Phase current measurement timeout |

### Common Motor Error Patterns

- **0x00000001 or 0x00000002:** Usually appears after power-on. Run motor calibration (AxisState=FULL_CALIB).
- **0x00000008 (DRV_FAULT):** Motor driver in fault state. Check:
  - Overcurrent spike (stalled rotor, misaligned endstops)
  - Overheating of driver IC
  - Supply voltage sagging
- **0x00000200 (NOT_CALIBRATED):** Motor parameters not stored. Run full calibration.
- **Persistent errors during homing:** Likely mechanical issue (jammed motor, misaligned endstops).

---

## Encoder Error (CYCLIC_ENCODER_ERROR 0x04)

**Cyclic Message:** 10ms broadcast interval (configurable)

**Byte Layout:** `encoder_error` (uint32_t)

### Error Flags (from ODrive 0.5.6 firmware)

| Bit | Hex Value | Flag Name | Meaning |
|-----|-----------|-----------|---------|
| 0 | 0x0000_0001 | UNSTABLE_GAIN | Encoder gain measurement unstable (noise on SPI/I2C) |
| 1 | 0x0000_0002 | CPR_POLEPAIR_MISMATCH | CPR does not match pole pair count (calibration mismatch) |
| 2 | 0x0000_0004 | NO_RESPONSE | Encoder not responding on SPI/I2C bus |
| 3 | 0x0000_0008 | UNSUPPORTED_ENCODER_MODE | Encoder mode not supported by firmware |
| 4 | 0x0000_0010 | ILLEGAL_HALL_STATE | Hall sensor in impossible state |
| 5 | 0x0000_0020 | INDEX_NOT_FOUND | Encoder index signal not found during calibration |
| 6 | 0x0000_0040 | ABS_SPI_COM_FAIL | Absolute encoder SPI communication failed |
| 7 | 0x0000_0080 | ABS_SPI_NOT_READY | Absolute encoder not ready after power-on |
| 8 | 0x0000_0100 | HALL_EFFECT_NOT_CALIBRATED | Hall sensor polarity/phase not calibrated |

### Common Encoder Error Patterns

- **0x00000002 (CPR_POLEPAIR_MISMATCH):** Check encoder CPR configuration in ODrive matches mechanical setup.
- **0x00000004 (NO_RESPONSE):** Encoder disconnected or SPI/I2C bus problem:
  - Check connector physical connection
  - Check cable integrity
  - Try power-cycling ODrive
- **0x00000020 (INDEX_NOT_FOUND):** Encoder index signal not seen during homing. Check:
  - Encoder cable connection
  - Encoder rotation speed during index search
  - Encoder health (test on standalone ODrive)
- **0x00000040 (ABS_SPI_COM_FAIL):** Absolute encoder SPI bus error. Check power supply to encoder.

---

## Controller Error (CYCLIC_CONTROLLER_ERROR 0x1D)

**Cyclic Message:** 10ms broadcast interval (configurable)

**Byte Layout:** `controller_error` (uint32_t)

### Error Flags (from ODrive 0.5.6 firmware)

| Bit | Hex Value | Flag Name | Meaning |
|-----|-----------|-----------|---------|
| 0 | 0x0000_0001 | OVERSPEED | Motor velocity exceeded limit |
| 1 | 0x0000_0002 | INVALID_STATE | Invalid requested axis state |
| 2 | 0x0000_0004 | UNKNOWN_STATE | Unknown state transition |
| 3 | 0x0000_0008 | UNKNOWN_CONTROL_MODE | Control mode not supported |
| 4 | 0x0000_0010 | UNKNOWN_INPUT_MODE | Input mode not supported |
| 5 | 0x0000_0020 | UNKNOWN_LIMITS_STATE | Limits configuration error |
| 6 | 0x0000_0040 | UNKNOWN_FLOAT_CTRLMODE | Float mode configuration error |
| 7 | 0x0000_0080 | UNKNOWN_OBSERVER_STATE | Sensorless observer error |
| 8 | 0x0000_0100 | UNKNOWN_COMMUTATION_CTRLMODE | Commutation mode error |
| 9 | 0x0000_0200 | UNKNOWN_SVM_MODE | SVM (Space Vector Modulation) mode error |

### Common Controller Error Patterns

- **0x00000001 (OVERSPEED):** Motor velocity limit exceeded. Check:
  - Velocity setpoint is reasonable
  - Load causing uncontrolled acceleration
  - Mechanical mechanism issues
- **0x00000002 (INVALID_STATE):** Invalid state transition attempted. Usually transient.
- **0x00000008 (UNKNOWN_CONTROL_MODE):** Firmware mode not recognized. Check ODrive firmware version compatibility.

---

## Axis Error (from Heartbeat CYCLIC_HEARTBEAT 0x01)

**Cyclic Message:** 100ms broadcast interval

**Byte Layout:** `axis_error` (uint32_t) - same as controller error codes above

This is the overall axis error state combining motor, encoder, and controller errors.

---

## Broadcast Data Store Integration

All error codes are captured in the centralized `BroadcastDataStore` with:

- **Error Value:** Full 32-bit error code
- **Timestamp:** millis() when error was last updated
- **Flag:** `hasAnyError` = true if any error field != 0

### Accessing Error Data (in state machines)

```cpp
BroadcastDataStore& store = BroadcastDataStore::getInstance();

// Check if any error present
if (store.hasAnyError()) {
    uint32_t motorErr = store.getMotorError();
    uint32_t encoderErr = store.getEncoderError();
    uint32_t ctrlErr = store.getControllerError();
    
    // Take corrective action based on error type
}

// Check when error occurred
uint32_t lastErrTime = store.getLastErrorUpdate();
uint32_t timeSinceError = millis() - lastErrTime;
```

---

## Serial Logging Format

Error codes are logged in hexadecimal format via SerialMessaging at 1Hz:

```
MotorErr:0x00000001 EncoderErr:0x00000000 CtrlErr:0x00000008
```

This allows quick visual identification of error codes for troubleshooting.

---

## ESP32 Machine Errors (SafetyManager)

These are high-level safety faults detected by the ESP32 SafetyManager, separate from ODrive errors:

| Code | Name | Meaning | Possible Causes | Response |
|------|------|---------|-----------------|----------|
| 0 | ERR_NONE | No error | - | Normal operation |
| 1 | ERR_ESTOP | E-stop pressed | Emergency stop button activated | Cut motor power, require manual reset |
| 2 | ERR_BARREL_POSITION_LOST | Barrel endstop open | Barrel removed during operation | Cut motor power, ERROR_STATE |
| 3 | ERR_NOZZLE_NOT_BLOCKED | Nozzle blockage check failed | Nozzle not blocked during test | Cut motor power, ERROR_STATE |
| 4 | ERR_OVER_TEMP | Temperature too high | Heater malfunction or overshoot | Cut motor power, wait for cooldown |
| 5 | ERR_HARD_LIMIT | Hard position limit violated | Motor moved beyond safe range | Cut motor power, require homing |
| 6 | ERR_UNDER_TEMP | Temperature too low | Heater not reaching target | Wait for heating |
| 7 | ERR_BOTTOM_ENDSTOP_COLLISION | Bottom endstop hit unexpectedly | Plunger collided with bottom during downward move | Cut motor power, ERROR_STATE |
| 8 | ERR_TOP_ENDSTOP_COLLISION | Top endstop hit unexpectedly | Plunger collided with top during upward move | Cut motor power, ERROR_STATE |
| 9 | ERR_BROADCAST_STALE | No ODrive broadcast data | CAN bus disconnected, ODrive crashed, ESP32 CAN failure | Flag error, FSM stops motor |

### ERR_BROADCAST_STALE (Code 9) - Critical Communication Fault

**Detection:** No encoder estimates broadcast received in 100ms (matches heartbeat interval)

**Why 100ms?**
- Encoder broadcasts every 10ms (fast feedback)
- Heartbeat broadcasts every 100ms (includes axis errors)
- If 100ms passes with NO broadcasts:
  - Heartbeat WOULD have arrived WITH axis error → existing error handler catches it
  - No heartbeat at all → CAN disconnect/ODrive crash/ESP32 CAN failure

**Root Causes (when 100ms timeout triggers):**
1. **CAN bus physical disconnect** → Hardware failure (loose cable, damaged connector)
2. **ODrive firmware crash/lockup** → ODrive stopped executing completely
3. **ESP32 CAN peripheral failure** → EMI/noise caused ESP32 TWAI reset

**Response:**
- SafetyManager flags error (does NOT cut DC contactor)
- FSM transitions to ERROR_STATE and stops motor via state machine
- Serial log: `SAFETY ERROR: Broadcast Stale (>100ms)`
- **Why not power off?** Avoids unnecessary homing cycle on recovery
- **Requires manual intervention** - investigate CAN bus, check ODrive status

**Why This Is Critical:**
- No feedback = blind operation (position/velocity unknown)
- Cannot detect collisions, overshoot, or stalls
- Similar risk to total sensor failure

**Future Enhancement:**
- Could attempt DC contactor cycle to reset ODrive (90% success rate on firmware lockups)
- Track staleness count to distinguish transient vs permanent failure
- Log last valid broadcast timestamp for diagnostics

---

## Machine Error Code 10: ERR_CAN_RTR_FAILURE

**Trigger Condition:** Critical CAN command failed RTR (Remote Transmission Request) confirmation after 3 retries (15ms total detection time)

**Critical Commands Protected by RTR:**
1. `setControllerModes()` - Mode transitions (Position/Velocity/Torque)
2. `setAxisState()` - State transitions (IDLE ↔ CLOSED_LOOP)
3. `clearErrors()` - Error recovery

**RTR Mechanism:**
- Command sent with RTR flag set (requests ODrive to echo back message)
- 5ms timeout per attempt × 3 retries = 15ms detection
- If no RTR response received, command failed (CAN break or ODrive non-responsive)

**Response:**
1. **If Motor Stationary (velocity < 0.5 rps):**
   - SafetyManager flags ERR_CAN_RTR_FAILURE
   - FSM transitions to ERROR_STATE
   - Serial log: `CRITICAL_TRANSITION_FAIL: <transition_type> | AxisState=X CtrlMode=Y`
   - No power cutoff (motor safe, CAN can recover)

2. **If Motor Moving (velocity ≥ 0.5 rps):**
   - Immediate DC contactor cutoff (forceEmergencyShutdown)
   - Serial log: `TRANS_FAIL: Motor moving, CAN unreliable, CUTTING CONTACTOR`
   - Requires power cycle to reset (hardware safety interlock)

**Why This Is Critical:**
- RTR failure = CAN bus break or ODrive firmware crash
- Continuing operation without confirmation risks:
  - Wrong mode (e.g., velocity mode with position setpoint = runaway)
  - Wrong state (e.g., IDLE when expecting CLOSED_LOOP = no motor control)
  - Uncleared errors (e.g., previous fault still active)
- 15ms detection prevents 1.25cm³ plastic waste (vs 100ms broadcast staleness)

**4 Critical Transitions:**
1. IDLE → CLOSED_LOOP (State 8): Homing/movement enable
2. Mode → POSITION_CONTROL (Mode 3): Refill/Injection moves
3. Mode → TORQUE_CONTROL (Mode 1): Compression/packing
4. Any State → IDLE (State 1): Emergency stop/release

**Recovery:**
- Power cycle ESP32 + ODrive (resets CAN bus and firmware)
- Check CAN bus wiring (loose connections, EMI interference)
- Verify ODrive firmware is responsive (USB odrivetool)

**Blocking Time:**
- Each RTR command blocks for 2-5ms (acceptable for critical commands)
- Non-critical commands (setInputPos, setInputVel, setInputTorque) remain non-blocking
- CAN_COMMAND_GAP_MS reduced to 0 (RTR replaces timing-based protection)

---

## Troubleshooting Workflow

1. **Check Serial Output:** Look for hex error codes in 1Hz status line
2. **Identify Error Type:** Use tables above to understand meaning
3. **Correlate with Action:** What was the motor doing when error occurred?
   - During homing? → Mechanical issue or timing problem
   - During injection? → Load issue or parameter tuning
   - Random? → Electrical noise or firmware issue
4. **Take Corrective Action:**
   - Calibration errors → Run motor/encoder calibration
   - Connection errors → Check cables and connectors
   - Mechanical errors → Inspect mechanical system for jams
   - Parameter errors → Verify ODrive configuration
   - RTR failures → Power cycle, check CAN bus wiring

---

## Future: Solutions Reference Table

As we identify patterns in actual operation, we'll add a "Solutions" section here with:

| Error Code | Context | Root Cause | Solution |
|-----------|---------|-----------|----------|
| TBD | During homing | TBD | TBD |
| TBD | During injection | TBD | TBD |
| TBD | Random | TBD | TBD |

*This table will be populated after we observe error patterns in actual hardware operation.*

---

## References

- ODrive 0.5.6 Documentation: https://docs.odriverobotics.com/v/0.5.6/
- ODrive Fibre Types: https://docs.odriverobotics.com/v/0.5.6/fibre_types/
- CAN Protocol Spec: https://docs.odriverobotics.com/v/0.5.6/can-protocol.html
