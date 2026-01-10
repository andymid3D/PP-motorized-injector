# Display Communications Protocol Specification

**Version:** 1.0  
**Date:** January 8, 2026  
**Hardware:** ESP32 Controller ↔ Antigravity Display (UART2)  
**Library:** SafeString (robust message parsing)

---

## Overview

This document specifies the bidirectional UART protocol between the ESP32 injection controller and the Antigravity Display. Messages use pipe-delimited (`|`) SafeString format for robustness and simplicity.

### Key Features
- **Non-blocking:** All message parsing/sending is non-blocking (no delays)
- **Broadcast interval:** Encoder position broadcast every 100ms (configurable)
- **State synchronization:** Immediate state change notifications
- **Error propagation:** Error codes broadcast immediately to Display
- **Parameter updates:** Display can update mould parameters between cycles

### UART Configuration
- **Baud Rate:** 115200  
- **Pins:** TX=GPIO17, RX=GPIO16  
- **Format:** 8N1 (8 data bits, no parity, 1 stop bit)  
- **Message Terminator:** `\n` (newline)

---

## Message Formats

### TX Messages (Controller → Display)

#### 1. Encoder Position Broadcast
**Purpose:** Real-time plunger position for Display graphics  
**Frequency:** Every 100ms (configurable via `DISPLAY_BROADCAST_INTERVAL_MS`)

```
ENC|<position>|<velocity>
```

**Fields:**
- `position` (float): Encoder position in turns (e.g., `45.23`)
- `velocity` (float): Velocity in turns/second (e.g., `-2.51` for upward movement)

**Example:**
```
ENC|45.23|-2.51
```

---

#### 2. State Change Notification
**Purpose:** FSM state synchronization for Display UI updates  
**Frequency:** Immediate on state change (not repeated if state unchanged)

```
STATE|<state_name>|<timestamp>
```

**Fields:**
- `state_name` (string): Current FSM state name
- `timestamp` (unsigned long): Milliseconds since boot (`millis()`)

**State Names:**
- `INIT_HEATING`
- `INIT_HOT_WAIT`
- `INIT_HOMING`
- `REFILL`
- `COMPRESSION`
- `READY_TO_INJECT`
- `PURGE_ZERO`
- `ANTIDRIP`
- `INJECT`
- `HOLD_INJECTION`
- `RELEASE`
- `CONFIRM_REMOVAL`
- `ERROR_STATE`

**Example:**
```
STATE|INJECT|1234567890
```

---

#### 3. Error Code Broadcast
**Purpose:** Error notification for Display error UI  
**Frequency:** Immediate on error (not repeated if error unchanged)

```
ERROR|<code>|<message>
```

**Fields:**
- `code` (hex uint16_t): Error code in hexadecimal format (e.g., `0x0200`)
- `message` (string): Human-readable error description

**Error Codes:**
| Code | Name | Description |
|------|------|-------------|
| `0x0001` | ERR_ESTOP | Emergency stop button pressed |
| `0x0002` | ERR_OVER_TEMP | Temperature above critical limit |
| `0x0003` | ERR_UNDER_TEMP | Temperature below minimum for movement |
| `0x0004` | ERR_BARREL_POSITION_LOST | Barrel endstop open (safety interlock) |
| `0x0005` | ERR_HARD_LIMIT | Position exceeded travel limits |
| `0x0006` | ERR_UNDER_TEMP | Temperature below safe movement threshold |
| `0x0007` | ERR_BOTTOM_ENDSTOP_COLLISION | Plunger hit bottom endstop during downward movement |
| `0x0008` | ERR_TOP_ENDSTOP_COLLISION | Plunger hit top endstop during upward movement |
| `0x0200` | MOTOR_ERROR_SPINOUT_DETECTED | ODrive detected motor spinout |

**Example:**
```
ERROR|0x0007|BOTTOM_ENDSTOP_COLLISION
```

---

#### 4. Mould Parameters Confirmation
**Purpose:** Acknowledge receipt of mould parameter update from Display  
**Frequency:** Response to `MOULD|...` command

```
MOULD_OK|<name>|<fill_vol>|<fill_speed>|<fill_pressure>|<pack_vol>|<pack_speed>|<pack_pressure>|<pack_time>|<cooling_time>|<fill_accel>|<fill_decel>|<pack_accel>|<pack_decel>
```

**Fields:** (13 total)
1. `name` (string): Mould name (max 64 chars)
2. `fill_vol` (float): Fill volume in cm³
3. `fill_speed` (float): Fill velocity limit (turns/sec)
4. `fill_pressure` (float): Fill pressure limit (torque Nm)
5. `pack_vol` (float): Pack volume in cm³
6. `pack_speed` (float): Pack velocity limit (turns/sec)
7. `pack_pressure` (float): Pack pressure limit (torque Nm)
8. `pack_time` (float): Pack duration in seconds
9. `cooling_time` (float): Cooling duration in seconds (Display-side timer)
10. `fill_accel` (float): Fill trap accel limit (turns/sec²)
11. `fill_decel` (float): Fill trap decel limit (turns/sec²)
12. `pack_accel` (float): Pack trap accel limit (turns/sec²)
13. `pack_decel` (float): Pack trap decel limit (turns/sec²)

**Example:**
```
MOULD_OK|TestMould|35.0|25.0|20.0|5.0|2.0|10.0|2.0|0.0|20.0|20.0|10.0|10.0
```

---

#### 5. Common Parameters Confirmation
**Purpose:** Echo common parameters (config.h defines, read-only at runtime)  
**Frequency:** Response to `COMMON|...` or `QUERY_COMMON` command

```
COMMON_OK|<homing_vel>|<refill_gap>|<comp_torque>|<inject_vel>|<release_dist>|<antidrip_vel>|<purge_vel>|<backoff_vel>|<approach_vel>|<comp_travel_vel>
```

**Fields:** (10 total)
1. `homing_vel` (float): Homing fast retract velocity (turns/sec)
2. `refill_gap` (float): Refill rest position offset (turns)
3. `comp_torque` (float): Compression torque limit (Nm)
4. `inject_vel` (float): Injection velocity limit (turns/sec)
5. `release_dist` (float): Release distance (turns, negative)
6. `antidrip_vel` (float): AntiDrip velocity (turns/sec, negative)
7. `purge_vel` (float): Purge velocity (turns/sec, positive)
8. `backoff_vel` (float): Homing backoff velocity (turns/sec, positive)
9. `approach_vel` (float): Homing approach velocity (turns/sec, negative)
10. `comp_travel_vel` (float): Compression travel velocity limit (turns/sec)

**Example:**
```
COMMON_OK|12.5|47.7|10.0|20.0|-2.5|-2.0|2.0|2.5|-1.5|5.0
```

**Note:** These are config.h defines and **cannot be changed at runtime** without modifying config.h and reflashing firmware. This command is informational only.

---

### RX Messages (Display → Controller)

#### 1. Mould Parameter Update
**Purpose:** Update current mould parameters (between cycles only)  
**Response:** `MOULD_OK|...` confirmation

```
MOULD|<name>|<fill_vol>|<fill_speed>|<fill_pressure>|<pack_vol>|<pack_speed>|<pack_pressure>|<pack_time>|<cooling_time>|<fill_accel>|<fill_decel>|<pack_accel>|<pack_decel>
```

**Fields:** Same as `MOULD_OK` response (13 total)

**Safety:** Controller should only accept this command when FSM state is REFILL or READY_TO_INJECT (not during active injection).

**Example:**
```
MOULD|My2DMould|35.0|25.0|20.0|5.0|2.0|10.0|2.0|0.0|20.0|20.0|10.0|10.0
```

---

#### 2. Common Parameters Query
**Purpose:** Request current config.h parameter values  
**Response:** `COMMON_OK|...` with all current values

```
COMMON
```

**Note:** This is a query-only command. Common parameters are config.h defines and cannot be changed at runtime. To change these values, user must modify config.h and reflash firmware.

**Example:**
```
COMMON
```

---

#### 3. Query Commands
**Purpose:** Request current status/parameters from Controller  
**Responses:** Varies by query type

| Command | Response | Description |
|---------|----------|-------------|
| `QUERY_MOULD` | `MOULD_OK\|...` | Return current mould parameters |
| `QUERY_COMMON` | `COMMON_OK\|...` | Return current common parameters |
| `QUERY_STATE` | `STATE\|...\|...` | Return current FSM state |
| `QUERY_ERROR` | `ERROR\|...\|...` | Return current error code |

**Examples:**
```
QUERY_MOULD
QUERY_COMMON
QUERY_STATE
QUERY_ERROR
```

---

## Integration Requirements

### Controller Side (main.cpp)

#### 1. Include Header
```cpp
#include "DisplayComms.h"
```

#### 2. Make currentMould Non-Const
```cpp
// OLD:
const actualMouldParams_t currentMould = { ... };

// NEW:
actualMouldParams_t currentMould = { ... };  // Allow Display updates
```

#### 3. Initialize in setup()
```cpp
void setup() {
    // ... existing setup code ...
    DisplayComms::begin();
}
```

#### 4. Update in loop()
```cpp
void loop() {
    DisplayComms::update();  // Handle RX/TX non-blocking
    // ... rest of loop() ...
}
```

#### 5. Broadcast State Changes
```cpp
// In FSM state entry (when stateEntry == true):
if (stateEntry) {
    DisplayComms::broadcastState(fsm_state.currentState);
    // ... rest of state entry logic ...
}
```

#### 6. Broadcast Errors
```cpp
// In error handlers:
if (error_detected) {
    fsm_state.currentState = ERROR_STATE;
    fsm_state.error = error_code;
    DisplayComms::broadcastError(error_code, "ERROR_MESSAGE");
}
```

---

### Display Side (Antigravity Platform)

#### 1. UART Configuration
- Configure UART with 115200 baud, 8N1 format
- Set RX buffer size ≥512 bytes (for long mould params messages)
- Use non-blocking read/write (avoid blocking main UI loop)

#### 2. Message Parsing
- Split by `|` delimiter using string parsing library
- Validate field count before processing (reject malformed messages)
- Handle newline (`\n`) as message terminator

#### 3. Encoder Position Updates
- Parse `ENC|...` messages every 100ms
- Update plunger graphics in real-time
- Use velocity for direction indicator (+ = down, - = up)

#### 4. State Synchronization
- Parse `STATE|...` messages immediately
- Update UI state machine to match Controller state
- Use state name for display text (e.g., "Ready to Inject")

#### 5. Error Handling
- Parse `ERROR|...` messages immediately
- Display error code and message prominently
- Block user actions until error cleared (user must acknowledge)

#### 6. Mould Parameter Updates
- Send `MOULD|...` command when user selects mould from library
- Wait for `MOULD_OK|...` confirmation before proceeding
- Timeout after 2 seconds if no response (retry or show error)

#### 7. RefillBlocks Utility (Future Feature)
- Track melt time per plastic block
- Use `STATE|COMPRESSION|...` timestamp to calculate time since last refill
- Use `STATE|REFILL|...` timestamp to mark block add time
- Validate MeltTimeMin before allowing injection (safety check)

---

## Safety Considerations

### Parameter Update Timing
- **NEVER** update mould parameters during active injection (INJECT, HOLD_INJECTION states)
- Display should only send `MOULD|...` command when Controller state is REFILL or READY_TO_INJECT
- Controller should reject parameter updates in other states (log error, no action)

### Message Validation
- Controller must validate all incoming field counts (reject malformed messages)
- Controller must validate parameter ranges (e.g., fillVolume > 0, packTime > 0)
- Display must validate all incoming field counts (reject malformed messages)

### Error Handling
- If parsing fails, log error and discard message (do NOT crash or hang)
- If UART buffer overflows, clear buffer and log error
- If confirmation timeout occurs, retry once then show error to user

### End-of-Day Position Safety
- Display should warn user if plunger position is outside heated zone (< 90.2 turns) for >30 minutes
- Display should prompt user to return to READY_TO_INJECT state before shutdown
- Plunger freeze risk: If plastic cools outside heated zone, plunger becomes stuck (requires manual intervention)

---

## Testing Procedure

### 1. UART Loopback Test
- Connect TX to RX (loopback)
- Send test messages from Controller
- Verify messages received correctly

### 2. Encoder Broadcast Test
- Connect Display UART
- Verify encoder position updates every 100ms
- Move plunger manually, verify position changes in Display

### 3. State Synchronization Test
- Step through FSM states (use buttons)
- Verify Display receives STATE|... messages immediately
- Verify Display UI updates to match Controller state

### 4. Error Propagation Test
- Trigger E-stop (or other error)
- Verify Display receives ERROR|... message immediately
- Verify Display shows error prominently

### 5. Mould Parameter Update Test
- Send `MOULD|...` command from Display (while in REFILL state)
- Verify Controller sends `MOULD_OK|...` confirmation
- Verify parameters stored correctly in Controller

### 6. Query Command Test
- Send `QUERY_MOULD`, `QUERY_COMMON`, `QUERY_STATE`, `QUERY_ERROR` from Display
- Verify correct responses received

---

## Troubleshooting

### Problem: No encoder position updates
- **Check:** UART wiring (TX→RX, RX→TX crossed)
- **Check:** Baud rate matches on both sides (115200)
- **Check:** `DisplayComms::update()` called in loop()
- **Check:** Encoder position is changing (motor moving)

### Problem: State changes not received
- **Check:** `DisplayComms::broadcastState()` called on state entry
- **Check:** Display RX buffer not overflowing (increase buffer size)
- **Check:** Message terminator (`\n`) present

### Problem: Mould parameters not updating
- **Check:** Controller state is REFILL or READY_TO_INJECT (not during injection)
- **Check:** All 13 fields present in `MOULD|...` command
- **Check:** Field delimiters are `|` (pipe character)
- **Check:** No extra spaces or special characters

### Problem: RX buffer overflow
- **Check:** Display is sending messages faster than Controller can process
- **Check:** Increase RX buffer size (currently 512 bytes)
- **Check:** Reduce message frequency from Display

---

## Future Enhancements

### 1. Compression Volume Calculation
- Broadcast compression start/end positions
- Display calculates RefillBlock volume added (newPos - oldPos)
- Tracks per-block melt time for MeltTimeMin validation

### 2. Cycle Statistics
- Broadcast cycle duration, injection speed, pack pressure
- Display aggregates statistics (shots per hour, average cycle time)
- Display shows charts/graphs for process monitoring

### 3. Configuration Backup
- Display stores mould library remotely (SD card or cloud)
- Controller can request mould parameters by name
- Display can download firmware updates over UART

### 4. Real-Time Pressure Monitoring
- Broadcast pressure sensor readings during injection
- Display shows real-time pressure graph
- Display alerts if pressure exceeds limits (mould defect detection)

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-08 | Agent + Andy | Initial protocol specification for Antigravity Display integration |

---

**Document Status:** FINAL  
**Approval Required:** Andy (Hardware), Antigravity Team (Display Software)
