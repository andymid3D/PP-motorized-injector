# Integration Checklist: Display Communications

**Version:** 1.0  
**Date:** January 8, 2026  
**Purpose:** Step-by-step guide for integrating DisplayComms module into main.cpp

---

## Overview

This checklist provides a systematic approach to integrating the DisplayComms module into the main ESP32 firmware. Follow these steps in order to ensure proper integration and testing.

---

## Prerequisites

✅ **Hardware:**
- ESP32 Controller with UART2 configured (TX=GPIO17, RX=GPIO16)
- Antigravity Display connected via UART (crossover: TX→RX, RX→TX)
- Display firmware implements SafeString protocol (see DisplayComms_Protocol.md)

✅ **Firmware:**
- DisplayComms.h and DisplayComms.cpp compiled successfully
- SafetyManager endstop collision detection implemented
- config.h updated with DISPLAY_BROADCAST_INTERVAL_MS and DISPLAY_BAUD_RATE

✅ **Documentation:**
- DisplayComms_Protocol.md reviewed by Antigravity team
- Endstop_Safety_Strategy.md reviewed by hardware team
- This integration checklist reviewed by firmware team

---

## Phase 1: Code Changes (main.cpp)

### Step 1.1: Add Include Statement

**Location:** Top of main.cpp, with other #include statements

**Code:**
```cpp
#include "DisplayComms.h"
```

**Verification:** Compile succeeds, no errors

---

### Step 1.2: Make currentMould Non-Const

**Location:** Global variables section (~line 50-100)

**OLD Code:**
```cpp
const actualMouldParams_t currentMould = {
    // ... existing initialization ...
};
```

**NEW Code:**
```cpp
actualMouldParams_t currentMould = {
    // ... existing initialization ...
};
// NOTE: Removed 'const' to allow Display updates via DisplayComms::parseIncomingMessage()
```

**Verification:** Compile succeeds, no errors

---

### Step 1.3: Initialize DisplayComms in setup()

**Location:** setup() function, after other initializations (~line 800)

**Code:**
```cpp
void setup() {
    // ... existing setup code ...
    
    // Initialize Display Communications
    DisplayComms::begin();
    logMessage("DisplayComms: Initialized");
    
    // ... rest of setup ...
}
```

**Verification:** 
- Compile succeeds
- Serial output shows "DisplayComms: Initialized" on boot
- UART2 initialized with correct baud rate (115200)

---

### Step 1.4: Update DisplayComms in loop()

**Location:** loop() function, early in the loop (~line 900)

**Code:**
```cpp
void loop() {
    // Non-blocking updates
    motor.update();
    safety.updateInputs();
    DisplayComms::update();  // ← Add this line
    
    // ... rest of loop ...
}
```

**Verification:**
- Compile succeeds
- Encoder position broadcast every 100ms (check Display)
- RX messages parsed non-blocking (no delays in loop)

---

### Step 1.5: Broadcast State Changes

**Location:** FSM state switch cases, at state entry (~line 1000-1500)

**Pattern:** Add to every state case where `stateEntry == true`:

```cpp
case InjectorStates::YOUR_STATE:
    if (stateEntry) {
        DisplayComms::broadcastState(fsm_state.currentState);  // ← Add this line
        // ... rest of state entry logic ...
    }
    // ... state update logic ...
    break;
```

**States to Update:**
- INIT_HEATING
- INIT_HOT_NOT_HOMED
- INIT_HOMING
- REFILL
- COMPRESSION
- READY_TO_INJECT
- PURGE_ZERO
- ANTIDRIP
- INJECT
- HOLD_INJECTION
- RELEASE
- CONFIRM_MOULD_REMOVAL
- ERROR_STATE

**Verification:**
- Compile succeeds
- State changes visible in Display UI immediately
- Serial log shows "State→Display: <STATE_NAME>"

---

### Step 1.6: Broadcast Errors

**Location:** Error handlers in safety.check() and FSM state cases (~line 1000-1500)

**Pattern:** Add to every error trigger:

```cpp
// Example: In ERROR_STATE entry
if (stateEntry) {
    DisplayComms::broadcastError(fsm_state.error, getErrorMessage(fsm_state.error));
    // ... rest of error entry logic ...
}

// Example: In safety violation
if (!safety.check(velocity, is_moving_down)) {
    fsm_state.currentState = ERROR_STATE;
    fsm_state.error = safety.getLastError();
    DisplayComms::broadcastError(fsm_state.error, getErrorMessage(fsm_state.error));
}
```

**Helper Function (add to main.cpp):**
```cpp
const char* getErrorMessage(uint16_t errorCode) {
    switch (errorCode) {
        case ERR_ESTOP:                     return "ESTOP_PRESSED";
        case ERR_OVER_TEMP:                 return "OVER_TEMP";
        case ERR_UNDER_TEMP:                return "UNDER_TEMP";
        case ERR_BARREL_POSITION_LOST:      return "BARREL_OPEN";
        case ERR_HARD_LIMIT:                return "HARD_LIMIT";
        case ERR_BOTTOM_ENDSTOP_COLLISION:  return "BOTTOM_COLLISION";
        case ERR_TOP_ENDSTOP_COLLISION:     return "TOP_COLLISION";
        default:                            return "UNKNOWN_ERROR";
    }
}
```

**Verification:**
- Compile succeeds
- Error codes visible in Display UI immediately
- Serial log shows "Error→Display: 0x<CODE> (<MESSAGE>)"

---

## Phase 2: Compilation and Flash

### Step 2.1: Clean Build

**Command:**
```bash
cd "/Users/andy/Documents/GitHub/PP-motorized-injector/ESP32_sketches/PlatformIO sketches/PP-motorized-injector"
/Users/andy/.platformio/penv/bin/pio run --target clean
/Users/andy/.platformio/penv/bin/pio run
```

**Expected Output:**
```
RAM:   [==        ]  9.5% (used 31240 bytes from 327680 bytes)
Flash: [===       ]  27.0% (used 354123 bytes from 1310720 bytes)
========================= [SUCCESS] Took X.XX seconds =========================
```

**Verification:**
- ✅ No compilation errors
- ✅ RAM usage < 15% (safe margin)
- ✅ Flash usage < 40% (safe margin)

---

### Step 2.2: Upload to ESP32

**Command:**
```bash
/Users/andy/.platformio/penv/bin/pio run -t upload
```

**Expected Output:**
```
Configuring upload protocol...
AVAILABLE: cmsis-dap, esp-bridge, esp-prog, espota, esptool, iot-bus-jtag, jlink, minimodule, olimex-arm-usb-ocd, olimex-arm-usb-ocd-h, olimex-arm-usb-tiny-h, olimex-jtag-tiny, tumpa
CURRENT: upload_protocol = esptool
Looking for upload port...
Auto-detected: /dev/cu.SLAB_USBtoUART
Uploading .pio/build/esp32dev/firmware.bin
...
========================= [SUCCESS] Took X.XX seconds =========================
```

**Verification:**
- ✅ Upload successful
- ✅ Serial monitor shows boot messages
- ✅ DisplayComms initialization message present

---

### Step 2.3: Monitor Serial Output

**Command:**
```bash
/Users/andy/.platformio/penv/bin/pio device monitor
```

**Expected Output:**
```
--- Forcing Serial port creation in PlatformIO ---
--- Miniterm on /dev/cu.SLAB_USBtoUART  115200,8,N,1 ---
--- Quit: Ctrl+C | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---

PP Injector Controller v2.0
Initializing...
SafetyManager: Begin
CanBusHandlerV2: Begin
DisplayComms: UART2 initialized (TX=17, RX=16, 115200 baud)
State: INIT_HEATING | Temp: 18.5°C | Heating: ON
```

**Verification:**
- ✅ DisplayComms initialization message present
- ✅ No error messages related to DisplayComms
- ✅ State changes logged correctly

---

## Phase 3: Display Integration Testing

### Step 3.1: Encoder Position Broadcast Test

**Procedure:**
1. Connect Display UART to ESP32 (TX→RX, RX→TX crossover)
2. Power on both devices
3. Move plunger manually (or trigger homing sequence)
4. Monitor Display UI for position updates

**Expected Behavior:**
- Display shows encoder position updating every 100ms
- Position changes match manual movement direction
- Velocity shows positive (down) or negative (up) values

**Display RX Messages:**
```
ENC|0.00|0.00
ENC|5.23|-1.25
ENC|10.45|-1.30
ENC|15.60|-1.28
```

**Verification:**
- ✅ Encoder position updates at ~10Hz (100ms interval)
- ✅ Position values match expected range (0 - 355 turns)
- ✅ Velocity values match movement direction

---

### Step 3.2: State Synchronization Test

**Procedure:**
1. Step through FSM states using buttons (REFILL → COMPRESSION → READY_TO_INJECT)
2. Monitor Display UI for state changes
3. Verify Display state matches Controller state

**Expected Behavior:**
- Display shows state name immediately on state change
- State transitions match button presses
- No lag between Controller state change and Display update

**Display RX Messages:**
```
STATE|REFILL|1234567890
STATE|COMPRESSION|1234570000
STATE|READY_TO_INJECT|1234575000
```

**Verification:**
- ✅ State changes appear in Display UI immediately (<100ms)
- ✅ State names match expected FSM states
- ✅ Timestamps increment monotonically

---

### Step 3.3: Error Propagation Test

**Procedure:**
1. Trigger error (E-stop, temperature, or collision)
2. Monitor Display UI for error message
3. Verify error code and message match expected values

**Expected Behavior:**
- Display shows error code prominently (red background, flashing, etc.)
- Error message matches error code (e.g., "BOTTOM_COLLISION" for 0x0007)
- User cannot proceed until error acknowledged

**Display RX Messages:**
```
ERROR|0x0007|BOTTOM_COLLISION
ERROR|0x0008|TOP_COLLISION
ERROR|0x0001|ESTOP_PRESSED
```

**Verification:**
- ✅ Error messages appear in Display UI immediately (<100ms)
- ✅ Error codes match expected values (0x0001 - 0x0008)
- ✅ Error messages human-readable and descriptive

---

### Step 3.4: Mould Parameter Update Test

**Procedure:**
1. Enter REFILL state (safe to update parameters)
2. From Display, send `MOULD|...` command with new parameters
3. Monitor serial output for confirmation message
4. Verify Display receives `MOULD_OK|...` confirmation

**Expected Behavior:**
- Controller parses `MOULD|...` command successfully
- Controller updates `currentMould` struct with new values
- Controller sends `MOULD_OK|...` confirmation to Display
- Next injection cycle uses new parameters

**Display TX Message:**
```
MOULD|TestMould|35.0|25.0|20.0|5.0|2.0|10.0|2.0|0.0|20.0|20.0|10.0|10.0
```

**Display RX Message:**
```
MOULD_OK|TestMould|35.0|25.0|20.0|5.0|2.0|10.0|2.0|0.0|20.0|20.0|10.0|10.0
```

**Serial Log:**
```
Mould params updated: TestMould
Mould params confirmed→Display
```

**Verification:**
- ✅ `MOULD|...` command parsed successfully (all 13 fields)
- ✅ `MOULD_OK|...` confirmation sent to Display
- ✅ Serial log shows "Mould params updated: <NAME>"
- ✅ Next injection uses new parameters

---

### Step 3.5: Query Commands Test

**Procedure:**
1. From Display, send query commands: `QUERY_MOULD`, `QUERY_COMMON`, `QUERY_STATE`, `QUERY_ERROR`
2. Monitor Display for correct responses
3. Verify responses match current Controller state

**Expected Behavior:**
- `QUERY_MOULD` → `MOULD_OK|...` with current mould parameters
- `QUERY_COMMON` → `COMMON_OK|...` with config.h parameters
- `QUERY_STATE` → `STATE|...|...` with current FSM state
- `QUERY_ERROR` → `ERROR|...|...` with current error code (or 0 if no error)

**Display TX Messages:**
```
QUERY_MOULD
QUERY_COMMON
QUERY_STATE
QUERY_ERROR
```

**Display RX Messages:**
```
MOULD_OK|TestMould|35.0|25.0|20.0|5.0|2.0|10.0|2.0|0.0|20.0|20.0|10.0|10.0
COMMON_OK|12.5|47.7|10.0|20.0|-2.5|-2.0|2.0|2.5|-1.5|5.0
STATE|READY_TO_INJECT|1234567890
ERROR|0x0000|NO_ERROR
```

**Verification:**
- ✅ All query commands receive correct responses
- ✅ Response data matches current Controller state
- ✅ No timeout or parsing errors

---

## Phase 4: Stress Testing

### Step 4.1: High-Frequency Message Test

**Procedure:**
1. Send rapid-fire query commands from Display (10 queries/second for 30 seconds)
2. Monitor serial output for buffer overflows or parsing errors
3. Verify Controller responds to all queries correctly

**Expected Behavior:**
- Controller handles all incoming messages without buffer overflow
- RX buffer cleared after each message parsed
- No missed messages or parsing errors

**Verification:**
- ✅ No "RX buffer overflow" messages in serial log
- ✅ All query commands receive responses
- ✅ No hanging or blocking in loop()

---

### Step 4.2: Long-Running Encoder Broadcast Test

**Procedure:**
1. Run machine through 10 full injection cycles
2. Monitor Display for encoder position updates throughout
3. Verify encoder broadcast never stops or hangs

**Expected Behavior:**
- Encoder position broadcast continues every 100ms throughout all states
- No gaps or delays in broadcast
- No memory leaks or buffer overflows

**Verification:**
- ✅ Encoder broadcast maintains 100ms interval for entire test duration
- ✅ No serial output errors or warnings
- ✅ RAM usage stable (no memory leaks)

---

### Step 4.3: Simultaneous TX/RX Test

**Procedure:**
1. Start encoder broadcast (automatic, 100ms interval)
2. While broadcasting, send `MOULD|...` command from Display
3. Verify both TX (encoder) and RX (mould update) work simultaneously

**Expected Behavior:**
- Encoder broadcast continues without interruption
- Mould update command parsed successfully
- Confirmation sent without delay

**Verification:**
- ✅ Encoder broadcast not interrupted by RX message parsing
- ✅ Mould update parsed correctly while TX ongoing
- ✅ No blocking or delays in either direction

---

## Phase 5: Error Handling Testing

### Step 5.1: Malformed Message Test

**Procedure:**
1. Send malformed messages from Display (missing fields, wrong delimiters, etc.)
2. Monitor serial output for error handling
3. Verify Controller does not crash or hang

**Test Messages:**
```
MOULD|TestMould|35.0   (missing fields)
MOULD,TestMould,35.0   (wrong delimiter)
MOULD||||||||||||||||  (empty fields)
ABCD|XYZ|123           (unknown command)
```

**Expected Behavior:**
- Controller logs error: "DisplayComms: MOULD command parsing failed (insufficient fields)"
- Controller logs error: "DisplayComms: Unknown command: ABCD"
- Controller does NOT crash or hang
- Controller continues normal operation

**Verification:**
- ✅ Malformed messages logged as errors
- ✅ Controller continues normal operation (no crash)
- ✅ No buffer overflows or memory corruption

---

### Step 5.2: Buffer Overflow Test

**Procedure:**
1. Send extremely long message from Display (>512 bytes, exceeds RX buffer size)
2. Monitor serial output for buffer overflow handling
3. Verify Controller clears buffer and continues

**Test Message:**
```
MOULD|AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA...
(continued for >512 bytes)
```

**Expected Behavior:**
- Serial log shows: "DisplayComms: RX buffer overflow, message discarded"
- Controller clears RX buffer
- Controller continues normal operation
- Next valid message parsed correctly

**Verification:**
- ✅ Buffer overflow detected and handled gracefully
- ✅ Buffer cleared after overflow
- ✅ Next message parsed correctly

---

## Phase 6: Final Integration

### Step 6.1: Full Cycle Test with Display

**Procedure:**
1. Complete full injection cycle with Display connected
2. Monitor Display UI throughout cycle
3. Verify all states, encoder position, and errors synchronized

**Expected Behavior:**
- Display shows all state transitions (REFILL → COMPRESSION → READY → PURGE → ANTIDRIP → INJECT → HOLD → RELEASE)
- Encoder position updates in real-time during movement
- No lag or delays in Display UI
- Error detection and recovery works correctly

**Verification:**
- ✅ Full cycle completes successfully with Display synchronized
- ✅ No communication errors or timeouts
- ✅ Display UI matches Controller state at all times

---

### Step 6.2: End-of-Day Toggle Test with Display

**Procedure:**
1. Complete injection cycle with `flags.endOfDay = false`
2. Verify Display shows return to REFILL state (47.7 turns)
3. Press Upper+Lower buttons to toggle `flags.endOfDay = true`
4. Complete another cycle
5. Verify Display shows return to READY_TO_INJECT state (~74 turns)

**Expected Behavior:**
- Display UI updates to show correct return state
- Display warns user if plunger outside heated zone for >30 minutes (future feature)
- User can toggle `flags.endOfDay` from Display (future feature)

**Verification:**
- ✅ Display shows correct return state based on `flags.endOfDay`
- ✅ Encoder position matches expected return state

---

### Step 6.3: Commit and Push

**Procedure:**
1. Verify all tests pass
2. Commit changes with clear message
3. Push to GitHub

**Commit Message:**
```
feat: Add UART Display comms + endstop collision safety (Jan 8 2026)

- Added DisplayComms module (DisplayComms.h/cpp)
- Implemented bidirectional SafeString protocol (encoder, state, error, mould params)
- Added endstop collision detection (ERR_BOTTOM_ENDSTOP_COLLISION, ERR_TOP_ENDSTOP_COLLISION)
- Safety context exemption for homing/antidrip (CTX_MOVING_FREE)
- Config defines added (DISPLAY_BROADCAST_INTERVAL_MS, DISPLAY_BAUD_RATE)
- Documentation created (DisplayComms_Protocol.md, Endstop_Safety_Strategy.md, Integration_Checklist.md)

Tested:
- Encoder broadcast every 100ms ✅
- State synchronization immediate ✅
- Error propagation <100ms ✅
- Mould parameter updates ✅
- Query commands ✅
- Buffer overflow handling ✅
- Full cycle with Display ✅

Ready for Antigravity Display integration.
```

**Commands:**
```bash
cd "/Users/andy/Documents/GitHub/PP-motorized-injector/ESP32_sketches/PlatformIO sketches/PP-motorized-injector"
git add -A
git commit -m "feat: Add UART Display comms + endstop collision safety (Jan 8 2026)"
# Use GitHub Desktop to push (command-line may fail for large commits)
```

**Verification:**
- ✅ All files committed
- ✅ Commit message clear and descriptive
- ✅ Pushed to GitHub successfully

---

## Troubleshooting

### Problem: Compile errors after adding DisplayComms.h

**Solution:**
- Check that DisplayComms.h and DisplayComms.cpp are in correct directories (include/ and src/)
- Verify all #include statements are correct
- Check that SafeString library is installed (platformio.lib_deps)

---

### Problem: Encoder position not broadcasting

**Solution:**
- Check that `DisplayComms::update()` is called in loop()
- Verify UART2 wiring (TX→RX crossover)
- Check baud rate matches on both sides (115200)
- Monitor serial output for "DisplayComms: Initialized" message

---

### Problem: State changes not appearing in Display

**Solution:**
- Check that `DisplayComms::broadcastState()` is called on state entry
- Verify message format matches Display parser (pipe-delimited)
- Check that Display is listening on correct UART pins
- Use serial monitor to verify TX messages being sent

---

### Problem: Mould parameter updates not working

**Solution:**
- Check that `currentMould` is non-const (removed `const` keyword)
- Verify all 13 fields present in `MOULD|...` command
- Check that Controller state is REFILL or READY_TO_INJECT (not during injection)
- Monitor serial output for "Mould params updated: <NAME>" confirmation

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-08 | Agent + Andy | Initial integration checklist for DisplayComms module |

---

**Document Status:** FINAL  
**Approval Required:** Andy (Firmware), Antigravity Team (Display Integration)
