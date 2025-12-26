# Broadcast Message Integration - Complete

## Summary

Successfully integrated all 7 active ODrive cyclic broadcast messages into the centralized `BroadcastDataStore` with non-blocking error logging. All error message types now feed directly into the data store for state machine consumption.

**Status:** ✅ COMPLETE - Compilation successful (7.1% RAM, 25.9% Flash)

---

## What Changed

### 1. **BroadcastDataStore.h/cpp** (Previously Updated)
- ✅ Added comprehensive error structures (ErrorData, EncoderData, SensorlessData)
- ✅ Added 7 granular error/encoder/sensorless update methods
- ✅ Added 8 error/encoder/sensorless query methods
- ✅ Made SafeString parameter optional in error update methods (default nullptr)

### 2. **CanBusHandlerV2.cpp** (Updated This Session)
- ✅ Added `#include "BroadcastDataStore.h"`
- ✅ Added 4 new case statements in loop() for error cyclic messages:
  - `CYCLIC_MOTOR_ERROR` (0x03) → `updateMotorError()`
  - `CYCLIC_ENCODER_ERROR` (0x04) → `updateEncoderError()`
  - `CYCLIC_CONTROLLER_ERROR` (0x1D) → `updateControllerError()`
  - `CYCLIC_SENSORLESS_ERROR` (0x05) → `updateSensorlessError()`
- ✅ Added 3 new case statements for encoder/sensorless data:
  - `CYCLIC_ENCODER_COUNT` (0x0A) → `updateEncoderCount()`
  - `CYCLIC_SENSORLESS_ESTIMATES` (0x15) → `updateSensorlessEstimates()`
- ✅ Feed all cyclic message data to BroadcastDataStore
- ✅ Maintain backward compatibility: All data still stored in CanBusHandlerV2 internal structures

### 3. **ERROR_CODES.md** (New Documentation)
- ✅ Complete reference of all ODrive error flags (motor, encoder, controller, sensorless)
- ✅ Explains bit meanings and common error patterns
- ✅ Provides troubleshooting workflow
- ✅ Reserved section for "Solutions" after we observe actual error patterns

---

## Architecture: Error Message Flow

```
ODrive (10ms broadcast)
    ↓
CAN Bus (TWAI)
    ↓
CanBusHandlerV2::loop()
    ├─ Parse via ODriveCANProtocol::parseCyclicXxx()
    ├─ Store in CanBusHandlerV2 (backward compatibility)
    └─ Feed to BroadcastDataStore::getInstance().updateXxx()
        ↓
    BroadcastDataStore (centralized cache)
        ├─ Store error value + timestamp
        ├─ Set hasAnyError flag
        └─ Append hex to SafeString (optional logging)
            ↓
        SerialMessaging (1Hz)
            └─ Print debug log with all error codes
```

### Integration Points

1. **CAN Reception (10ms intervals):**
   - Motor error, encoder error, controller error, sensorless error
   - Parsed via existing ODriveCANProtocol functions
   - Stored in BroadcastDataStore with timestamps

2. **Serial Logging (1Hz):**
   - SafeString accumulates hex error codes during 10 CAN cycles
   - SerialMessaging prints aggregated line at 1Hz
   - No blocking delays on fast CAN reception

3. **State Machine Access (anytime):**
   - Homing FSM, compression FSM, injection FSM
   - Query via `BroadcastDataStore::getInstance().getXxxError()`
   - Non-blocking reads, timestamps allow staleness checks

---

## Active Broadcast Messages

| Message | CAN ID | Interval | Data | Status |
|---------|--------|----------|------|--------|
| CYCLIC_HEARTBEAT | 0x01 | 100ms | axis_error, axis_state, flags | ✅ Integrated |
| CYCLIC_ENCODER_ESTIMATES | 0x09 | 10ms | position, velocity | ✅ Integrated |
| CYCLIC_IQ | 0x14 | 100ms | Iq_setpoint, Iq_measured | ✅ Integrated |
| CYCLIC_BUS_VI | 0x17 | 100ms | bus_voltage, bus_current | ✅ Integrated |
| CYCLIC_MOTOR_ERROR | 0x03 | 10ms | motor_error flags | ✅ Integrated |
| CYCLIC_ENCODER_ERROR | 0x04 | 10ms | encoder_error flags | ✅ Integrated |
| CYCLIC_CONTROLLER_ERROR | 0x1D | 10ms | controller_error flags | ✅ Integrated |

**Note:** Encoder count (0x0A) also integrated for completeness.

---

## Compilation Results

```
RAM:   [=         ]   7.1% (used 23272 bytes from 327680 bytes)
Flash: [===       ]  25.9% (used 340033 bytes from 1310720 bytes)
```

✅ **All errors resolved. Build successful.**

---

## Testing Checklist

### ✅ Code Quality
- [x] No compilation errors
- [x] Memory usage acceptable (7.1% RAM, 25.9% Flash)
- [x] Backward compatibility maintained (CanBusHandlerV2 still stores locally)
- [x] Non-blocking design preserved (no Serial.println on CAN loop)

### ⏳ Hardware Testing (Next Steps)
- [ ] Monitor serial output for error messages during homing
- [ ] Verify error codes appear in 1Hz status line
- [ ] Identify patterns: Which errors appear in normal operation?
- [ ] Test error recovery: Does FSM handle transient errors gracefully?
- [ ] Confirm timestamp tracking works (getLastErrorUpdate())

### ⏳ Integration Testing (Next Steps)
- [ ] Run full homing sequence with error monitoring
- [ ] Check for any spurious error flags
- [ ] Verify compression force feedback with motor error context
- [ ] Test injection cycle for controller/encoder errors

---

## Code Snippets: Usage Examples

### Access Error Data in State Machines

```cpp
// In any state machine (Homing, Compression, Injection)
BroadcastDataStore& store = BroadcastDataStore::getInstance();

// Quick check: any error?
if (store.hasAnyError()) {
    // Something went wrong, take action
    return StateTransition::TO_ERROR_STATE;
}

// Detailed error check
uint32_t motorErr = store.getMotorError();
if (motorErr & 0x0000_0008) {  // DRV_FAULT
    // Motor driver in fault, retry or abort
}

// Staleness check (is error still happening or old?)
uint32_t timeSinceError = millis() - store.getLastErrorUpdate();
if (timeSinceError > 5000) {
    // Error happened 5+ seconds ago, might be cleared
}
```

### Log Error with Context

```cpp
// In debug homing mode
SafeString debugLog;
debugLog = "Homing Step 5 | ";

// Manually log error for visibility
if (store.hasAnyError()) {
    uint32_t motorErr = store.getMotorError();
    if (motorErr != 0) {
        debugLog += "MotorErr:0x";
        char hex[16];
        snprintf(hex, sizeof(hex), "%08lX", motorErr);
        debugLog += hex;
    }
    
    // SerialMessaging will print this at 1Hz
}
```

---

## Next Steps

### Immediate (This Session)
1. ✅ Integrated all error messages
2. ✅ Created ERROR_CODES.md reference
3. ✅ Verified compilation

### Short Term (Next Debug Session)
1. Upload firmware to ESP32
2. Run homing with error monitoring
3. Check serial output for error messages
4. Identify any spurious errors

### Medium Term (After Error Patterns Clear)
1. Update ERROR_CODES.md "Solutions" section with observed patterns
2. Add error recovery logic to state machines
3. Example: If encoder error during homing, retry with slower speed

### Long Term
1. Comprehensive error handling throughout FSM
2. User-facing error codes for LED feedback
3. Telemetry log of all errors for debugging

---

## Files Modified

### CanBusHandlerV2.cpp
- Added `#include "BroadcastDataStore.h"` (line 2)
- Expanded loop() method to handle 7 cyclic message types (lines 55-160)
- Each message type: parse → store locally → feed to BroadcastDataStore

### BroadcastDataStore.h/cpp
- Enhanced with error structures, update methods, query methods (previous session)
- Made SafeString parameter optional (this session)

### ERROR_CODES.md (New)
- Complete error code reference
- Troubleshooting workflows
- Solution tracking template

---

## Design Rationale

### Why Optional SafeString?
- CanBusHandlerV2 is on the fast CAN RX path (10ms for error messages)
- Don't want to accumulate SafeStrings on every message
- Instead: Store error values only, let SerialMessaging handle logging at 1Hz
- Result: Fast CAN path unblocked, error codes still logged

### Why Duplicate Storage?
- CanBusHandlerV2 maintains local copies (backward compatibility)
- BroadcastDataStore also stores (centralized access for state machines)
- Small memory cost, large simplicity benefit
- Easy migration path if needed later

### Why Timestamp Each Error?
- Distinguish between "error happening now" vs "error from 10 seconds ago"
- State machines can make decisions: "retry only if error is fresh"
- Enables error recovery logic with timeout handling

---

## Verification Commands

```bash
# Check compilation
cd "PP-motorized-injector" && /Users/andy/.platformio/penv/bin/pio run

# Upload to ESP32
/Users/andy/.platformio/penv/bin/pio run -t upload

# Monitor serial output
/Users/andy/.platformio/penv/bin/pio device monitor
```

---

## References

- [BroadcastDataStore.h](./include/BroadcastDataStore.h) - Centralized data store interface
- [CanBusHandlerV2.cpp](./src/CanBusHandlerV2.cpp) - CAN message dispatch and integration
- [ERROR_CODES.md](./ERROR_CODES.md) - Error code reference and troubleshooting
- ODrive Docs: https://docs.odriverobotics.com/v/0.5.6/can-protocol.html
