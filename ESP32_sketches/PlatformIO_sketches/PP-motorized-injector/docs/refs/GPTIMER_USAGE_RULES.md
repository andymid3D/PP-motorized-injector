# GPTimer Usage Rules & Audit
**Date:** February 1, 2026  
**Purpose:** Document all `hwTimer.micros()` usage to ensure consistent timestamp handling

**Last Updated:** Added comprehensive system timing usage documentation

---

## **CORE RULES**

### **Rule 1: Always Capture Absolute GPTimer Values**
- `hwTimer.micros()` returns microseconds since boot (uint64_t)
- **ALWAYS** store as absolute timestamp at capture point
- **NEVER** apply offset math at capture time

### **Rule 2: Convert to Relative ONLY at Display/Comparison**
- Relative time = `currentTime - referenceTime`
- Calculate at display time, not at capture time
- Example: `elapsedSinceTx = hwTimer.micros() - txSentTime_us_`

### **Rule 3: Never Mix Offset Math**
- Don't offset at capture AND at display
- Choose ONE: Store absolute (preferred) OR store relative
- Mixing causes double-subtraction errors

---

## **USAGE AUDIT (All Files)**

### **✅ CORRECT USAGE**

#### **1. CanRxHandler.cpp - RX Message Timestamping**
**Lines:** 144, 453 (archived TWAI version)

```cpp
// Capture timestamp immediately after read
uint64_t timestamp = hwTimer.micros();
```

**Usage:** ✅ Absolute capture, stored directly to `msg.timestamp`  
**Flow:** CAN frame read → timestamp → queue → BDS storage  
**Notes:** This is the source of truth for all RX timestamps

---

#### **2. BroadcastDataStore.cpp - Age Calculation**
**Lines:** 124, 133, 142

```cpp
uint64_t age = hwTimer.micros() - latest->timestamp;
```

**Usage:** ✅ Relative calculation at comparison time  
**Purpose:** Check if data is stale (>1000ms = stale flag)  
**Notes:** Correctly subtracts stored absolute timestamp from current time

---

#### **3. CanRxHandler.cpp - Performance Measurement**
**Lines:** 794, 803 (stress test code)

```cpp
uint64_t drainStart = hwTimer.micros();
// ... drain queue ...
uint32_t drainTime = hwTimer.micros() - drainStart;
```

**Usage:** ✅ Interval measurement (start → end → duration)  
**Purpose:** Measure queue drain time for stress tests  
**Notes:** Local scope, no storage confusion

---

#### **4. PhaseTests.cpp - Test Timing**
**Lines:** 221, 230, 279, 334, 345, 457, 459, 520, 533

```cpp
uint64_t drainStart = hwTimer.micros();
uint32_t drainTime = hwTimer.micros() - drainStart;
uint64_t age = hwTimer.micros() - enc->timestamp;
```

**Usage:** ✅ Performance measurements for validation tests  
**Purpose:** Measure test execution times, data freshness  
**Notes:** Test-only code, isolated from production paths

---

#### **5. main.cpp - System Timing & State Management**
**Lines:** 413, 486, 528, 538, 589

```cpp
bootTime = hwTimer.micros();                    // Line 413 - System boot timestamp
uint64_t currentTime = hwTimer.micros();        // Line 486 - Debug timing (1s intervals)
stateTimer = hwTimer.micros();                  // Line 528 - FSM state entry timing
if (hwTimer.micros() - bootTime > 3000000)      // Line 538 - 3s boot safety check
bool ignoreButtons = (hwTimer.micros() - stateTimer < 500000);  // Line 589 - 500ms button lock
```

**Usage:** ✅ System-wide timing for state management, debug intervals, boot safety  
**Purpose:** FSM state transitions, debug reporting, button lock timing, boot safety checks  
**Notes:** Critical system timing - all using absolute timestamps with relative calculations

---

#### **6. CanBusHandlerV2.cpp - Command Gap Timing**
**Lines:** 56

```cpp
uint64_t now = hwTimer.micros();
if (!isQueueEmpty() && (now - lastCommandSentTime_) >= (CAN_COMMAND_GAP_MS * 1000))
```

**Usage:** ✅ CAN command gap enforcement (50ms minimum between commands)  
**Purpose:** Prevent CAN bus overload with microsecond precision  
**Notes:** Converts MS gap requirement to microseconds for precision timing

---

#### **7. Refill.cpp - Module Timing**
**Lines:** 27, 37, 53, 77

```cpp
stepTimer = hwTimer.micros() / 1000;           // Convert to milliseconds for module logic
unsigned long now = hwTimer.micros() / 1000;   // Consistent timing across modules
```

**Usage:** ✅ Module-level timing with millisecond conversion  
**Purpose:** Refill state machine timing, step transitions  
**Notes:** Converts GPTimer microseconds to milliseconds for module compatibility

---

#### **8. ProtectedWindowTest.cpp - Test Timing**
**Lines:** 98, 183, 194

```cpp
uint64_t currentTime = hwTimer.micros();      // Line 98 - Test timing
_commandSentTime_us = hwTimer.micros();        // Line 183 - Command sent timestamp
txMsg.timestamp = hwTimer.micros();            // Line 194 - Message timestamp
```

**Usage:** ✅ Test system timing for response correlation  
**Purpose:** Measure motor response times, command tracking  
**Notes:** Test-only code, follows correct absolute timestamp pattern

---

#### **9. TimingSystemTest.cpp - Performance Testing**
**Lines:** 67, 103, 139, 240

```cpp
testStartTime_ = hwTimer.micros();             // Line 67 - Test start
uint64_t currentTime = hwTimer.micros();      // Line 103 - Current time for age calc
Serial.println(hwTimer.micros());              // Line 139 - Debug timestamp
uint64_t currentTime = hwTimer.micros();      // Line 240 - Data age calculation
```

**Usage:** ✅ Performance measurement and testing  
**Purpose:** Test execution timing, data freshness checks  
**Notes:** Test-only, correctly uses absolute timestamps

---

### **⚠️ PARTIALLY CORRECT (Needs Review)**

#### **10. ProtectedWindowTest.cpp - Capture System**
**Lines:** 31, 196, 220, 240, 261, 279, 294, 327, 389, 450, 458

**MIXED USAGE DETECTED:**

**Absolute timestamps (CORRECT):**
```cpp
txSentTime_us_ = hwTimer.micros();           // Line 196, 220, 240, etc.
captureStartTime_us_ = hwTimer.micros();     // Line 327
msg.timestamp_us = hwTimer.micros();         // Line 389 (⚠️ comment says RAW)
```

**Relative calculation at display (CORRECT):**
```cpp
int64_t elapsedSinceTx = hwTimer.micros() - txSentTime_us_;  // Line 31
```

**Busy-wait loop (CORRECT):**
```cpp
while (hwTimer.micros() < targetTxTime) {    // Line 450 - wait until target
```

**STATUS:** ⚠️ Code appears correct BUT has comment warning about "RAW absolute timestamp (not relative)" at line 389  
**ISSUE:** This was part of the ProtectedWindowTest debugging (timestamp confusion from Phase 1.8)  
**ACTION REQUIRED:** Review ProtectedWindowTest after architecture redesign (Phase 5)

---

#### **6. main.cpp - Loop Time Measurement (TEST_MODE)**
**Lines:** 337, 341, 374

```cpp
uint64_t lastLoopEnd = hwTimer.micros();
// ...
uint64_t loopStart = hwTimer.micros();
// ...
uint64_t loopEnd = hwTimer.micros();
```

**Usage:** ✅ Interval measurement for loop gap detection  
**Purpose:** TEST_MODE performance monitoring  
**Notes:** Test-only, not used in production FSM

---

### **✅ NO USAGE (Correct for Their Role)**

#### **12. MotorWrapper.cpp**
**Status:** ✅ No `hwTimer.micros()` usage  
**Reason:** Uses `millis()` for CAN gap enforcement (50ms resolution sufficient)  
**Notes:** Correct - CAN gap timing doesn't need microsecond precision

---

## **TIMESTAMP FLOW DIAGRAM**

```
CAN Bus
   ↓
CanRxHandler::pollAndQueue()
   ├─ ESP32Can.readFrame()
   ├─ timestamp = hwTimer.micros()  ← CAPTURE (absolute)
   ├─ msg.timestamp = timestamp
   └─ xQueueSend()
   
FreeRTOS Queue
   ↓
CanRxHandler::drainAndStore()
   ├─ xQueueReceive(&msg)
   ├─ bds.storeEncoder(... msg.timestamp ...)  ← STORE (absolute, unchanged)
   
BroadcastDataStore
   ├─ encoder_[index].timestamp = timestamp  ← RING BUFFER (absolute)
   
FSM / Modules
   ├─ motor.getPosition() → reads BDS
   ├─ Check freshness: age = hwTimer.micros() - data.timestamp  ← RELATIVE CALC
   
Display / Debug
   ├─ elapsedTime = current - reference  ← RELATIVE DISPLAY
   └─ Serial.print(elapsedTime)
```

**KEY INSIGHT:** Timestamps captured ONCE at source, stored absolute, converted to relative ONLY when displayed/compared

---

## **VIOLATIONS FOUND: NONE ✅**

All current production code follows the rules correctly:
1. ✅ RX capture: Absolute timestamps stored immediately
2. ✅ Age checks: Relative calculation at comparison time
3. ✅ Performance: Local interval measurements (no storage)
4. ✅ MotorWrapper: Uses millis() appropriately (no sub-ms needs)

---

## **ACTION ITEMS**

1. ✅ **DONE:** Audit complete - no violations found
2. ⚠️ **DEFER:** Review ProtectedWindowTest after Phase 5 redesign (known broken from Phase 1.8 testing)
3. 🔜 **TODO:** Update this doc after BDS General Pool + TX Register (Phase 3-4)
   - Verify TX timestamp capture point
   - Verify Paired Buffer timestamp preservation
4. 🔜 **TODO:** Add guidelines for future timestamp fields:
   - Always use `uint64_t` for absolute timestamps
   - Use `int32_t` or `int64_t` for relative times (can be negative)
   - Document units in variable name (e.g., `timestamp_us`, `duration_ms`)

---

## **REFERENCES**

- **GPTimer Class:** External instance `hwTimer` (from GPTimer.cpp)
- **Resolution:** 1 microsecond (1 MHz timer)
- **Range:** ~584,942 years before uint64_t overflow (2^64 microseconds)
- **Precision:** Hardware timer, NOT affected by FreeRTOS task switches

---

## **BEST PRACTICES FOR NEW CODE**

```cpp
// ✅ CORRECT: Capture absolute, display relative
void captureData() {
    data.timestamp_us = hwTimer.micros();  // Absolute
}

void displayData() {
    int64_t age_us = hwTimer.micros() - data.timestamp_us;  // Relative
    Serial.print("Age: "); Serial.print(age_us); Serial.println(" us");
}

// ❌ WRONG: Offset at capture
void captureDataWrong() {
    data.timestamp_us = hwTimer.micros() - referenceTime;  // DON'T DO THIS
}

// ❌ WRONG: Double offset
void displayDataWrong() {
    int64_t age_us = (hwTimer.micros() - referenceTime) - data.timestamp_us;  // BROKEN
}
```

---

**Audit Status:** ✅ COMPLETE  
**Next:** Task 1.2 (Baseline Timing Test)
