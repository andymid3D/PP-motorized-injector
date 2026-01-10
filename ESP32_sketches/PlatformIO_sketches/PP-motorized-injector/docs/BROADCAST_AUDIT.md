# Broadcast Data Store Audit - CORRECTED (Encoder Estimates Focus)

## Critical Terminology Correction

**Encoder Estimates** (Position & Velocity - FLOAT with decimals):
- Sent by ODrive: CYCLIC_ENCODER_ESTIMATES (0x09) every 10ms
- Format: float position (turns), float velocity (turns/second)
- **What you use:** Position moves, drift tracking, display
- **Where stored:** BroadcastDataStore (central)

**Encoder Counts** (Integer step counter):
- Would be sent by: CYCLIC_ENCODER_COUNT (0x0A) if activated
- Format: int32_t count (8192 counts per turn)
- **Status:** NOT activated in ODrive config
- **Where stored:** N/A (not received)
- **Note:** Only used if calling setLinearCount() with non-zero value

---

## Data Flow Summary

All 7 active ODrive broadcast messages properly routed through CanBusHandlerV2 → BroadcastDataStore.

---

## Broadcast Message Routing - CORRECTED

### ✅ Error Messages (10ms interval) - ALL ROUTED

| Message | CAN ID | Data | BroadcastDataStore Method | Status |
|---------|--------|------|--------------------------|--------|
| CYCLIC_MOTOR_ERROR | 0x03 | motor_error flags | `updateMotorError()` | ✅ ROUTED |
| CYCLIC_ENCODER_ERROR | 0x04 | encoder_error flags | `updateEncoderError()` | ✅ ROUTED |
| CYCLIC_CONTROLLER_ERROR | 0x1D | controller_error flags | `updateControllerError()` | ✅ ROUTED |

### ✅ Position/Velocity (10ms interval) - ALL ROUTED

| Message | CAN ID | Data | BroadcastDataStore Method | Status |
|---------|--------|------|--------------------------|--------|
| CYCLIC_ENCODER_ESTIMATES | 0x09 | position, velocity (FLOATS in turns) | `updateEncoderEstimates()` | ✅ ROUTED |

**Note:** CYCLIC_ENCODER_COUNT (0x0A) NOT activated - no encoder count data received. Only encoder ESTIMATES in BroadcastDataStore.

### ✅ State/Error (100ms interval) - ALL ROUTED

| Message | CAN ID | Data | BroadcastDataStore Method | Status |
|---------|--------|------|--------------------------|--------|
| CYCLIC_HEARTBEAT | 0x01 | axis_state, axis_error, flags | `updateAxisState()`, `updateAxisError()` | ✅ ROUTED |

### ⚠️ Power Data (100ms interval) - STORED LOCALLY ONLY

| Message | CAN ID | Data | BroadcastDataStore Method | Status |
|---------|--------|------|--------------------------|--------|
| CYCLIC_IQ | 0x14 | Iq_setpoint, Iq_measured | `updatePowerData()` | 🔴 NOT ROUTED* |
| CYCLIC_BUS_VI | 0x17 | bus_voltage, bus_current | `updatePowerData()` | 🔴 NOT ROUTED* |

**Why not routed:**
- `updatePowerData()` requires SafeString for logging
- These are diagnostic/monitoring data, not needed by position control
- Accessible via CanBusHandlerV2 getters:
  - `getIqReadings()` → `CyclicIq` struct
  - `getBusVoltageCurrentReadings()` → `CyclicBusVoltageCurrent` struct

---

## Critical Data For Homing & Position Control (What Gets Updated to BroadcastDataStore)

✅ **Axis State** (CRITICAL)
- Source: CYCLIC_HEARTBEAT (100ms)
- Method: `updateAxisState()`
- Query: `BroadcastDataStore::getAxisState()`
- Used by: Homing FSM, all state machines

✅ **Axis Error** (CRITICAL)
- Source: CYCLIC_HEARTBEAT (100ms)
- Method: `updateAxisError()`
- Query: `BroadcastDataStore::getAxisError()`
- Used by: Error recovery logic

✅ **Motor/Encoder/Controller Errors** (CRITICAL)
- Source: 10ms error broadcasts
- Methods: `updateMotorError()`, `updateEncoderError()`, `updateControllerError()`
- Query: Via `getMotorError()`, `getEncoderError()`, `getControllerError()`
- Used by: Error diagnostics, recovery decisions

✅ **Encoder Position & Velocity** (CRITICAL for position moves)
- Source: CYCLIC_ENCODER_ESTIMATES (0x09, 10ms) - FLOAT turns with decimals
- Method: `updateEncoderEstimates(float position, float velocity)`
- Query: `BroadcastDataStore::getEncoderPosition()`, `getEncoderVelocity()`
- Used by: Position control, drift reporting, display
- **CPR Conversion:** If needed as counts: position_turns * 8192 = encoder_count

---

## setLinearCount() Function - IMPORTANT

**Purpose:** Set ODrive's encoder count (step counter) to arbitrary value

**Current Usage:** `setLinearCount(0)` to reset encoder after homing

**If Ever Used with Non-Zero Value:**
- Value must be in encoder COUNTS (not turns)
- Conversion: `count = turns * 8192` (rounded to int32_t)
- Example: To set 2.5 turns → `setLinearCount((int32_t)(2.5 * 8192))` = 20480

**Documentation:** Enhanced with clear warnings in CanBusHandlerV2.h

---

## Architecture Decision: Local vs Central Storage

### Data in BroadcastDataStore (Non-Blocking)
- **Purpose:** State machine decisions (axis state, errors, position for moves)
- **Update Frequency:** 100ms (heartbeat, state) or 10ms (errors, position)
- **Logging:** Minimal/optional (SafeString* = nullptr)
- **Access:** Fast non-blocking queries via `getInstance().getXxx()`
- **Data Types:** int32_t, uint32_t, float, bool

### Data in CanBusHandlerV2 Local (Backward Compatible)
- **Purpose:** Diagnostics, monitoring, legacy code
- **Update Frequency:** 100ms (Iq, Bus V/I)
- **Logging:** Via SerialMessaging at 1Hz
- **Access:** Via public getter methods (`getIqReadings()`, `getBusVoltageCurrentReadings()`)

### Why This Split?
1. **Non-Blocking:** State machines query BroadcastDataStore without blocking
2. **Backward Compatible:** Existing code reads from CanBusHandlerV2 getters
3. **Minimal Overhead:** No duplicate processing on fast 10ms cycle
4. **Clean Design:** Position-critical data (estimates, state, errors) centralized

---

## Summary: All Broadcast Data Accounted For

| Category | Data | Source Message | BroadcastDataStore | Update Freq | Query Method |
|----------|------|-----------------|------------------|-------------|--------------|
| **State** | axis_state | CYCLIC_HEARTBEAT | ✅ Yes | 100ms | `getAxisState()` |
| **Errors** | axis/motor/encoder/controller errors | CYCLIC_*_ERROR | ✅ Yes | 10-100ms | `getXxxError()` |
| **Position** | position, velocity (FLOATS) | CYCLIC_ENCODER_ESTIMATES (0x09) | ✅ Yes | 10ms | `getEncoderPosition()`, `getEncoderVelocity()` |
| **Current** | Iq setpoint/measured | CYCLIC_IQ | ⚠️ Local | 100ms | `getIqReadings()` (CanBusHandlerV2) |
| **Power** | Bus voltage/current | CYCLIC_BUS_VI | ⚠️ Local | 100ms | `getBusVoltageCurrentReadings()` (CanBusHandlerV2) |

### NOT Activated (Never Sent)
| Message | CAN ID | Reason |
|---------|--------|--------|
| CYCLIC_ENCODER_COUNT | 0x0A | Not enabled in ODrive config; using Estimates instead |
| CYCLIC_SENSORLESS_* | 0x05, 0x15 | Motor has encoder; sensorless not needed |

✅ **All position-critical broadcast data IS properly routed to BroadcastDataStore for non-blocking access.**

