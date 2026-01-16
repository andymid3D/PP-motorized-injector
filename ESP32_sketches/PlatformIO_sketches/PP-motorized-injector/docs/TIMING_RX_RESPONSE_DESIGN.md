# TIMING & RX RESPONSE SYSTEM DESIGN
**Product Requirements Document**  
**Date:** January 15, 2026  
**Status:** Architecture Phase - Points 1-9 COMPLETE

---

## EXECUTIVE SUMMARY

**Objective:** Design a real-time CAN RX management system that:
- ✅ Captures 100% of ODrive broadcast messages (zero loss)
- ✅ Reliably detects TX command responses (2-5ms window)
- ✅ Maintains accurate microsecond timestamps for all messages
- ✅ Supports multi-module command queuing without collisions
- ✅ Achieves loop performance <1ms (target: 0.1-0.3ms)

**Core Strategy:** 
1. SafeString/millisDelay eliminate ALL blocking operations
2. GPTimer provides ISR-immune hardware timestamps
3. Enhanced BroadcastDataStore with ring buffers and TX correlation
4. Protected timing windows avoid ODrive broadcast injection
5. Aggressive TWAI FIFO draining prevents hardware buffer overflow

---

## POINT 1: SAFESTRING BUFFERED OUTPUT

### Current Anti-Pattern (BROKEN)
```cpp
Serial.print("Long debug message");  // Blocks 2-5ms
delay(50);                           // Blocks 50ms
pollCANMessages();                   // Only runs AFTER blocking
```

**Result:** CAN RX polling starved, messages lost during Serial.print() and delay()

### Correct Pattern (SafeString Library)
```cpp
// Setup (once)
createSafeStringStream(sfStream, 512);  // Create stream with 512-byte buffer
BufferedOutput bufferedOut(sfStream, DROP_UNTIL_EMPTY);  // Overflow mode
bufferedOut.connect(Serial);  // ⚠️ CRITICAL - MUST connect to hardware Serial

// Usage in loop
bufferedOut.nextByteOut();  // ⚠️ MUST call every loop - outputs 1 byte, returns immediately
sfStream.print("Data");     // Queues to SafeStringStream buffer, non-blocking
```

**CRITICAL Setup Requirements:**
1. **MUST** call `bufferedOut.connect(Serial)` in `setup()` before any print calls
2. **MUST** call `bufferedOut.nextByteOut()` at top of `loop()` every iteration
3. Recommend `DROP_UNTIL_EMPTY` mode (queues until buffer empties, no char loss)
4. Alternative modes: `DROP_IF_FULL` (drops overflow), `BLOCK_IF_FULL` (defeats purpose)

### Impact
- ✅ **Eliminates Serial.print() blocking** (currently 2-5ms per call)
- ✅ Outputs 1 byte per loop iteration (~100µs vs milliseconds)
- ✅ CAN polling happens every loop regardless of serial activity
- ✅ Proven pattern from SafeString tutorial (Arduino UNO <1ms loop time)

### Implementation Requirements
- Replace ALL `Serial.print()` calls with `sfStream.print()`
- Call `bufferedOut.nextByteOut()` in main loop every iteration
- Pre-allocate buffer (recommend 512-1024 bytes for debug messages)
- Handle buffer overflow gracefully (library provides overflow protection)

### References
- Tutorial: https://www.instructables.com/Simple-Multi-tasking-in-Arduino-on-Any-Board/
- Step 6: BufferedOutput eliminates blocking (100µs vs milliseconds)
- Step 10: Insert `nextByteOut()` between operations for continuous draining

---

## POINT 2: MILLISDELAY NON-BLOCKING TIMERS

### Current Anti-Pattern (BROKEN)
```cpp
delay(50);  // Blocks entire loop, stops CAN polling
```

### Correct Pattern (millisDelay Library)
```cpp
millisDelay myTimer;
myTimer.start(50);  // Start non-blocking timer

// In loop
if (myTimer.justFinished()) {
    // Do thing (triggered once)
    myTimer.repeat();  // Auto-restart for cyclic tasks
}
```

### Key Methods
- `start(ms)` - Begin countdown
- `justFinished()` - Returns true for ONE cycle only (perfect for state transitions)
- `repeat()` - Auto-restart timer with zero drift (ideal for ReadyToInject 30s micro-compression)
- `restart()` - Restart timer (accumulates drift, avoid for cyclic tasks)
- `isRunning()` - Check if timer active
- `stop()` - Cancel timer
- `finish()` - Force immediate finish (next `justFinished()` returns true)
- **`remaining()`** - Returns ms left until finish (0 if stopped/finished) ⭐ USEFUL FOR DISPLAY
- **`getStartTime()`** - Returns last start time in ms (0 if never started)
- **`delay()`** - Returns configured delay value in ms

**Key Difference:** Use `repeat()` not `restart()` for cyclic operations to prevent timer drift accumulation.

### Impact
- ✅ **Eliminates ALL delay() blocking**
- ✅ Loop runs continuously, polling CAN every iteration
- ✅ Multiple timers run "in parallel" (checked sequentially but non-blocking)
- ✅ State machine transitions become trivial (`if (timer.justFinished())`)

### Implementation Requirements
- Replace ALL `delay()` calls with `millisDelay` instances
- Create timer instances for each timing need:
  - Pre-roll capture timer (100ms)
  - Post-TX observation timer (configurable TEST_DELAY)
  - Triple-send spacing timer (5ms)
  - Module-specific timers (ReadyToInject 30s, AntiDrip 15s timeout, etc.)
- Call `justFinished()` in state machine conditions
- Use `repeat()` for cyclic operations

### Example Refactor
```cpp
// OLD (blocking)
void testNoRTR() {
    delay(PREROLL_CAPTURE_MS);
    sendCommand();
    delay(TEST_DELAY);
}

// NEW (non-blocking)
millisDelay preRollTimer;
millisDelay postTxTimer;

void testNoRTR() {
    switch (state) {
        case INIT:
            preRollTimer.start(PREROLL_CAPTURE_MS);
            state = PREROLL;
            break;
        case PREROLL:
            pollCANMessages();  // Runs every loop!
            if (preRollTimer.justFinished()) {
                sendCommand();
                postTxTimer.start(TEST_DELAY);
                state = POST_TX;
            }
            break;
        case POST_TX:
            pollCANMessages();  // Still runs!
            if (postTxTimer.justFinished()) {
                state = COMPLETE;
            }
            break;
    }
}
```

---

## POINT 3: BROADCASTDATASTORE INTEGRATION

### 3.1 RTRDebug vs Production Path
**Current Issue:** RTRDebug likely bypasses BroadcastDataStore with its own `pollCANMessages()`

**Production Requirement:** BroadcastDataStore MUST be single source of truth
- RTRDebug should READ from BroadcastDataStore, not bypass it
- Ensures debug findings translate to production behavior
- Unified code path reduces divergence bugs

### 3.2 CAN TX/RX Simultaneity
**Hardware:** ESP32 TWAI controller supports simultaneous TX/RX (separate pins)
- TX: GPIO5
- RX: GPIO4

**Software Limitation:** Single-threaded polling is sequential, not parallel
- Current: `writeFrame()` → `pollCANMessages()` (one after the other)
- With SafeString: Loop runs so fast (<1ms) that effective simultaneity achieved
- With Dual-Core (Point 9): True parallel RX polling on Core 0

**Conclusion:** Hardware supports it, software is sequential but fast enough (Points 1-2 fix). Dual-core (Point 9) would make it truly parallel.

### 3.3 ESP32 TWAI Hardware Buffer

**Critical Finding:** ESP32 TWAI peripheral has **8-16 message RX FIFO** (not 64 as initially thought)

**Buffer Behavior:**
- Hardware automatically fills FIFO from CAN bus
- `ESP32Can.readFrame(frame, 0)` reads oldest message (FIFO order)
- Non-blocking read (timeout=0) returns false if empty
- **Overflow:** If FIFO full, oldest messages dropped by hardware

**Overflow Risk Calculation:**
```
100ms cyclic rate: 7 messages/cycle = 70 msg/sec
50ms delay(): ~3.5 messages queued (safe, <16 limit)

20ms cyclic rate: 35 messages/cycle = 350 msg/sec  
50ms delay(): ~17.5 messages queued (OVERFLOW RISK!)

With SafeString loop (<0.3ms): 0.021 messages queued (ZERO RISK)
```

**Aggressive FIFO Draining Pattern:**
```cpp
// In main loop, drain FIFO completely
while (ESP32Can.readFrame(rxFrame, 0)) {
    processCAN(rxFrame);  // Process immediately
}
// FIFO now empty, no messages pending
```

**Production Implementation:**
✅ While-loop draining ensures FIFO never fills  
✅ Works regardless of cyclic rate (20ms, 40ms, 100ms)  
✅ Zero hardware overflow risk

### 3.4 BroadcastDataStore Message Storage

**Current Design:** Stores latest value only per message type
- `float encoderPos` (overwrites previous)
- `float encoderVel` (overwrites previous)
- No history, no timestamps

**Overwrite Risk:**
```
T=0ms:   encoderPos = 1.0 → Module starts processing
T=10ms:  encoderPos = 1.5 (cyclic update OVERWRITES)
T=20ms:  Module reads encoderPos = 1.5 (WRONG - expected 1.0!)
```

**Solution:** Ring buffer with timestamps (see 3.7)

### 3.5 ODrive Broadcast Injection - Protected Timing Windows

**Experimental Findings (100ms cyclic rate):**

| TX Timing | Time Before Next Bundle | Injection Behavior |
|-----------|------------------------|-------------------|
| T=65ms | 35ms before T=100 | ✅ ZERO injection |
| T=70ms | 30ms before T=100 | ✅ ZERO injection |
| T=60ms | 40ms before T=100 | 🔴 EXTRA injections |
| T=55ms | 45ms+ before T=100 | 🔴 EXTRA injections |

**Conclusion:** ODrive injects extra cyclic messages if TX occurs >35-40ms before next scheduled bundle.

**Protected Window (100ms cycle):**
```
T=0ms:     [Cyclic Bundle arrives - 7 messages in ~5ms burst]
T=0-60ms:  DANGER ZONE - injections likely occur if TX sent here
T=60-65ms: TRANSITION ZONE (boundary behavior unclear)
T=65-95ms: GOLDEN ZONE - protected from injection (30ms wide)
           └─ This is 30-35ms BEFORE next bundle at T=100ms
T=95-100ms: Too close - need 5ms for TX response window
T=100ms:   [Next cyclic bundle]
```

**Formula:**
```cpp
uint64_t goldenZoneStart = cyclePeriod_us - 35000;  // 35ms before next
uint64_t goldenZoneEnd = cyclePeriod_us - 5000;     // 5ms before next (preserve response time)
```

**For 100ms cycle:**
- Golden Zone: 65-95ms (30ms wide window)
- Send commands here to avoid injection

**Implementation:**
```cpp
bool isSafeToSendCommand() {
    uint64_t lastBroadcastTime = broadcastStore.getLastHeartbeatTime();
    uint64_t elapsed = hwTimer.micros() - lastBroadcastTime;
    uint64_t cycleUs = broadcastStore.getCyclePeriod();  // 100000us for 100ms
    
    uint64_t goldenStart = cycleUs - 35000;
    uint64_t goldenEnd = cycleUs - 5000;
    
    return (elapsed >= goldenStart && elapsed <= goldenEnd);
}
```

**20ms Rate Concern:** 🔴 CRITICAL ISSUE
- 20ms - 35ms = **-15ms** (negative golden zone start!)
- **Protected window impossible if 30-35ms is absolute threshold**

**Testing Protocol Required:**
1. ✅ Verify 100ms rate (baseline - proven safe)
2. ⏸️ Test 40ms rate (40-35=5ms start, 40-5=35ms end → 30ms window)
3. ⏸️ Test 30ms rate (30-35=-5ms → FAILS if absolute, works if percentage)
4. ⏸️ Test 20ms rate (confirm percentage vs absolute behavior)
5. ⏸️ Determine maximum safe cyclic rate for production

**Plan B Options:**
- **40ms cyclic rate:** Safe zone 5-35ms (30ms window), 2.5x faster than 100ms
- **30ms cyclic rate:** IF percentage works, ~20ms window
- **Hybrid rates:** Critical messages at safe rate, others at 100ms

**Data Quality Note:** Only 2-3 sample messages observed, boundary behavior at 60-65ms and 95-100ms requires more testing.

### 3.6 Message Classification by CAN ID

**Current Method (CORRECT):**
```cpp
uint32_t msgId = canId & 0x1F;  // Extract message type from CAN ID
switch(msgId) {
    case 0x001: // Heartbeat
    case 0x003: // Motor_Error
    case 0x009: // Encoder_Estimates
    // etc.
}
```

**ODrive Cyclic Broadcasts (100ms rate, 7 messages):**
- 0x001 Heartbeat
- 0x003 Motor_Error
- 0x004 Encoder_Error  
- 0x009 Encoder_Estimates ⚠️ (also used as GET response)
- 0x014 Get_Iq
- 0x017 Bus_Voltage_Current
- 0x01D Controller_Error

**SET Commands (NOT cyclic - no conflict):**
- 0x007 Set_Axis_State ✅
- 0x00B Set_Controller_Modes ✅
- 0x00C Set_Input_Pos ✅

**Conflict Case:**
- GET 0x009 (Encoder_Estimates) conflicts with cyclic 0x009
- Protected window (3.5) solves this: Response at T+2ms, next cyclic at T+100ms

**Verdict:** CAN ID classification works, protected windows prevent ambiguity.

### 3.7 Ring Buffer Architecture

**Problem:** Single-value storage can't distinguish response from cyclic update

**Solution:** Ring buffer with timestamps per message type

**Data Structure:**
```cpp
struct TimestampedMessage {
    float value;           // 4 bytes - decoded message data
    uint64_t timestamp;    // 8 bytes - GPTimer microseconds
    bool processed;        // 1 byte - module marked as read
    uint8_t padding[3];    // 3 bytes - alignment
};  // 16 bytes per slot

class BroadcastDataStore {
private:
    static const uint8_t HISTORY_SIZE = 10;
    
    // Ring buffers for each message type
    TimestampedMessage encoderPosHistory[HISTORY_SIZE];
    TimestampedMessage encoderVelHistory[HISTORY_SIZE];
    TimestampedMessage iqMeasuredHistory[HISTORY_SIZE];
    TimestampedMessage iqSetpointHistory[HISTORY_SIZE];
    TimestampedMessage busVoltageHistory[HISTORY_SIZE];
    TimestampedMessage busCurrentHistory[HISTORY_SIZE];
    TimestampedMessage axisStateHistory[HISTORY_SIZE];
    
    uint8_t encoderPosIndex = 0;
    uint8_t encoderVelIndex = 0;
    // ... (one index per message type)
    
    // TX Command correlation
    struct TxCommand {
        uint32_t canId;
        uint64_t txTime;
        bool responseReceived;
    };
    TxCommand recentTxCommands[5];  // Last 5 TX commands
    uint8_t txCommandIndex = 0;
    
    // Protected window tracking
    uint64_t lastHeartbeatTime = 0;
    uint64_t cyclePeriod = 100000;  // 100ms default, configurable
    
public:
    // Store new message
    void storeEncoderPos(float pos, uint64_t timestamp) {
        encoderPosIndex = (encoderPosIndex + 1) % HISTORY_SIZE;
        encoderPosHistory[encoderPosIndex] = {pos, timestamp, false};
    }
    
    // Normal module access - get latest
    float getLatestEncoderPos() {
        return encoderPosHistory[encoderPosIndex].value;
    }
    
    // Response detection - find message matching TX command
    bool findResponse(uint32_t canId, uint64_t txTime, uint64_t windowUs) {
        // Search ring buffer for message within time window
        // Return true if found, mark as processed
    }
    
    // Processing verification
    bool hasUnprocessed(MessageType type, uint64_t sinceTime) {
        // Check if any messages after sinceTime are unprocessed
    }
    
    void markProcessed(MessageType type, uint64_t timestamp) {
        // Find message by timestamp, set processed flag
    }
    
    // Protected window helpers
    bool isSafeToSendCommand() {
        uint64_t elapsed = hwTimer.micros() - lastHeartbeatTime;
        uint64_t goldenStart = cyclePeriod - 35000;
        uint64_t goldenEnd = cyclePeriod - 5000;
        return (elapsed >= goldenStart && elapsed <= goldenEnd);
    }
    
    uint64_t getLastHeartbeatTime() { return lastHeartbeatTime; }
    uint64_t getCyclePeriod() { return cyclePeriod; }
    void setCyclePeriod(uint64_t periodUs) { cyclePeriod = periodUs; }
};
```

**Memory Footprint:**
```
Ring Buffers: 7 types × 10 slots × 16 bytes = 1,120 bytes
TX Log: 5 slots × 16 bytes = 80 bytes
Metadata: ~50 bytes
Total: ~1,250 bytes (0.38% of 320KB RAM)
```

**Verdict:** ✅ Minimal memory cost, huge reliability gain

### 3.8 TX Command Correlation & Module Queuing

**Single Command In Flight Model:**
- Only ONE command sent at a time
- Next command waits for response OR timeout
- Simplified correlation (no sequence numbers needed)

**MessageBuffer Queuing:**
```cpp
// Module A wants to send
messageBuffer.queueCANCommand(cmdA);

// Module B wants to send (while A pending)
messageBuffer.queueCANCommand(cmdB);

// Main loop processes queue with spacing
if (messageBuffer.hasPending() && 
    hwTimer.micros() - lastTxTime > 10000) {  // 10ms spacing
    sendNextCommand();
}
```

**Collision Prevention:**
- 10ms minimum spacing = 10,000µs
- Response window = 2-5ms = 2,000-5,000µs
- **No overlap possible** ✅

**Simple Time-Window Pairing:**
```cpp
// When TX sent
uint64_t txTime = hwTimer.micros();
uint32_t txCanId = 0x00C;
motor.sendSetPosition(value);

// In CanBusHandler RX processing
void onCANReceive(uint32_t canId, uint8_t* data, uint64_t timestamp) {
    // Check if this matches recent TX command
    if (canId == txCanId && 
        timestamp - txTime < 5000) {  // Within 5ms
        // This is our response!
        broadcastStore.markResponseReceived(canId, txTime);
    }
    
    // Store in ring buffer regardless
    broadcastStore.storeMessage(canId, data, timestamp);
}
```

**No Sequence Numbers Needed:**
- 10ms spacing ensures previous response arrived before next TX
- Simple time window check is sufficient
- KISS principle maintained ✅

**ESP32 Clock Speed:** 240MHz CPU (not 80MHz - that's APB bus)
- 15x faster than Arduino UNO (16MHz)
- Target loop time: **0.1-0.3ms** (vs UNO's <1ms)
- Polling rate: **5,000+ times/second**
- 100ms broadcast → polled **500 times per cycle**

### 3.9 Hardware Timer Implementation (REQUIRED)

**Decision:** GPTimer REQUIRED (not optional)

**5 Justifications:**

1. **Practice/Learning** - ESP-IDF integration experience valuable for future
2. **Definitive Timestamps** - Immune to ANY ISR interference (WiFi, BT, future additions)
3. **Universal Timing Source** - All modules use same clock (BroadcastDataStore, MotorWrapper, modules)
4. **Superior Debugging** - Compare GPTimer vs micros() to detect ISR interference
5. **No Overflow** - 64-bit counter never rolls over (vs micros() 70-minute rollover)

**Implementation:**
```cpp
#include "driver/gptimer.h"  // ESP-IDF header (works in Arduino Framework)

class HardwareTimer {
private:
    gptimer_handle_t timer_;
    
public:
    void begin() {
        gptimer_config_t config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000,  // 1µs resolution
        };
        gptimer_new_timer(&config, &timer_);
        gptimer_enable(timer_);
        gptimer_start(timer_);
    }
    
    uint64_t micros() {
        uint64_t count;
        gptimer_get_raw_count(timer_, &count);
        return count;
    }
    
    void reset() {
        gptimer_set_raw_count(timer_, 0);
    }
};

// Global instance
HardwareTimer hwTimer;

// Replace ALL micros() calls with:
uint64_t timestamp = hwTimer.micros();
```

**Memory Impact:**
- uint64_t timestamps: 8 bytes (vs 4 for uint32_t)
- Ring buffer overhead: +280 bytes total (vs uint32_t)
- **Negligible** (0.08% of RAM)

**Alternative Considered (REJECTED):**
- **micros():** Adequate for no-WiFi/BT, but vulnerable to future ISRs
- **RMT peripheral:** Designed for PWM/LED, not general timing (overkill)
- **PCNT counter:** Counts pulses, not a timer (wrong tool)

**Verdict:** GPTimer provides perfect source of truth with minimal complexity increase.

---

## POINT 4: LOOP OPTIMIZATION TRICKS (RED HERRING)

### Question: Should We Insert Extra CAN Poll Calls or Use Scheduler Library?

**Answer:** ❌ **NO - Red Herring, Unnecessary with SafeString**

### Option 4A: Insert Multiple `pollCANMessages()` Calls

**Proposal:** Manually insert poll calls throughout code:
```cpp
void someFunction() {
    doThing1();
    pollCANMessages();  // ← Extra poll
    doThing2();
    pollCANMessages();  // ← Extra poll
    doThing3();
    pollCANMessages();  // ← Extra poll
}
```

**Problems:**
1. **Code pollution** - Clutters every function with manual polling
2. **Maintenance nightmare** - Easy to forget poll calls in new code
3. **False solution** - Treats symptom, not root cause (blocking operations)
4. **Emergency fallback only** - If SafeString/millisDelay fails, this is last resort

**Verdict:** ❌ **Reject** - Use only as emergency debugging tool, not production pattern

### Option 4B: Scheduler Library

**Proposal:** Use [Scheduler library](https://github.com/arduino-libraries/Scheduler) for cooperative multitasking:
```cpp
#include <Scheduler.h>

void canPollingTask() {
    while (true) {
        pollCANMessages();
        yield();  // Give other tasks time
    }
}

void setup() {
    Scheduler.startLoop(canPollingTask);
}
```

**Problems:**
1. **Wrong architecture** - Scheduler is for **single-core** Arduino boards (Uno, Mega, etc.)
2. **ESP32 has FreeRTOS** - Already has native dual-core RTOS (see Point 9)
3. **Adds unnecessary layer** - Cooperative scheduling on top of preemptive RTOS
4. **SafeString makes it obsolete** - Non-blocking loop already achieves goal
5. **Not designed for ESP32** - Library documentation targets AVR boards

**Verdict:** ❌ **Reject** - Wrong tool for ESP32, use FreeRTOS (Point 9) if threading needed

### Why These Are Red Herrings

**Root Cause:** Blocking operations (`delay()`, `Serial.print()`) starve CAN polling  
**Real Solution:** SafeString (Points 1-2) eliminates blocking → loop runs continuously

**Performance Math:**
```
Arduino UNO (16MHz, single-core):
- With SafeString: <1ms loop time
- Polling rate: 1,000+ times/second

ESP32 (240MHz, dual-core):
- With SafeString: 0.1-0.3ms loop time (15x faster CPU)
- Polling rate: 5,000+ times/second
- 100ms broadcast polled 500 times per cycle
```

**Conclusion:** With SafeString, loop is **already fast enough** - no optimization tricks needed.

### When to Consider These Options

**Extra Polling (Emergency Fallback):**
- ✅ Debugging blocking code location (insert poll, see if messages recover)
- ✅ Temporary workaround during refactoring (remove after SafeString complete)
- ❌ Production code (fix the blocking, don't work around it)

**Scheduler Library (Never):**
- ❌ ESP32 architecture mismatch (use FreeRTOS tasks instead)
- ❌ Adds complexity without benefit (SafeString already non-blocking)
- ❌ Not needed for CAN polling (loop speed sufficient)

### Alternative: FreeRTOS Tasks (Point 9)

**IF** SafeString proves insufficient (unlikely), use native ESP32 FreeRTOS:
```cpp
// Core 0 dedicated to CAN RX polling (true parallel)
xTaskCreatePinnedToCore(
    canPollingTask,  // Function
    "CAN_RX",        // Name
    2048,            // Stack size
    NULL,            // Parameters
    1,               // Priority
    NULL,            // Handle
    0                // Core 0 (Core 1 runs Arduino loop)
);
```

**Advantages over Scheduler:**
- ✅ Native ESP32 feature (no external library)
- ✅ Preemptive multitasking (not cooperative)
- ✅ True parallel execution (dual-core CPU)
- ✅ Designed for ESP32 architecture

**Note:** This is Point 9 analysis - **defer until SafeString testing complete**.

### Verdict: Skip Point 4 Options

✅ **Points 1-2 (SafeString/millisDelay)** solve the problem correctly  
❌ **Point 4A (extra polling)** is emergency debugging tool only  
❌ **Point 4B (Scheduler)** is wrong architecture for ESP32  
⏸️ **Point 9 (FreeRTOS)** is backup plan if SafeString insufficient  

**Recommendation:** Implement Points 1-2 first, measure loop performance with loopTimer, proceed to Point 9 only if target (<0.3ms) not achieved.

---

## POINT 6: FREERTOS UNDERLYING ARDUINO (CRITICAL CONTEXT)

### Question: Does FreeRTOS Run Beneath Arduino Framework?

**Answer:** ✅ **YES - ESP32 Arduino Framework ALWAYS Runs on FreeRTOS**

### Architecture Reality

**ESP32 Arduino Framework is Built on FreeRTOS:**
```
Hardware Layer: ESP32 dual-core CPU (Core 0 + Core 1)
    ↓
ESP-IDF Layer: FreeRTOS preemptive scheduler
    ↓
Arduino Framework: Wrapper that exposes Arduino API
    ↓
Your Code: setup() + loop()
```

**What Actually Happens:**
```cpp
// Behind the scenes (Arduino core code)
void loopTask(void* pvParameters) {
    setup();  // Runs once
    while(1) {
        loop();  // Your loop() runs as FreeRTOS task
        // FreeRTOS can preempt this task at any time
    }
}

// Arduino core startup
void initArduino() {
    xTaskCreatePinnedToCore(
        loopTask,        // Your Arduino code
        "loopTask",      // Task name
        8192,            // Stack size
        NULL,
        1,               // Priority (normal)
        NULL,
        ARDUINO_RUNNING_CORE  // Default: Core 1
    );
}
```

**Key Facts:**
1. ✅ `loop()` is a FreeRTOS task (pinned to Core 1 by default)
2. ✅ FreeRTOS scheduler can preempt `loop()` at any time
3. ✅ System tasks run on Core 0 (WiFi, BT, network stack)
4. ✅ Tick rate: 1000 Hz (1ms tick, configurable via `CONFIG_FREERTOS_HZ`)
5. ✅ Preemptive scheduling: Higher priority tasks interrupt lower priority

### Impact on CAN Polling

**Scenario 1: No WiFi/BT Active (Current State)**
```
Core 0: Idle (system housekeeping only, minimal interruption)
Core 1: loop() task runs continuously
    └─ CAN polling happens thousands of times per second
    └─ Preemption risk: LOW (no competing high-priority tasks)
```

**Worst Case Preemption:**
- FreeRTOS tick interrupt: 1ms tick = max 1ms interruption
- System tasks: Watchdog, timekeeping (~microseconds)
- **Impact:** Negligible with 0.3ms target loop time

**Scenario 2: WiFi/BT Enabled (Future Risk)**
```
Core 0: WiFi/BT stack tasks (high priority)
Core 1: loop() task
    └─ WiFi events can trigger ISRs
    └─ BT stack can preempt via IPC (inter-processor call)
```

**Measured Impact (from ESP32 community):**
- WiFi active: +0.5-2ms jitter to loop()
- BT active: +1-5ms jitter to loop()
- Combined: +2-10ms jitter (UNACCEPTABLE for CAN)

### Why SafeString/millisDelay is Even More Critical

**Without Non-Blocking Code:**
```cpp
loop() {
    delay(50);  // Holds FreeRTOS task for 50ms
                // Other tasks can't run on Core 1
                // Watchdog can trigger if delay too long
    Serial.print("Long message");  // Blocks task 2-5ms
}
```

**With Non-Blocking Code:**
```cpp
loop() {
    bufferedOut.nextByteOut();  // Returns in ~100µs
    if (timer.justFinished()) {  // Non-blocking check
        // Do thing
    }
    pollCANMessages();  // Runs every iteration
    // FreeRTOS can preempt between these operations
    // Each operation is small, preemption window is tiny
}
```

**Why This Matters:**
- ✅ Short operations = FreeRTOS preempts less disruptively
- ✅ No blocking = Other tasks can schedule properly
- ✅ Watchdog happy = No task starvation
- ✅ Clean task switching = Predictable timing

### FreeRTOS Task Priorities (ESP32 Arduino)

**Default Priorities:**
```
Priority 25: WiFi/BT stack (highest)
Priority 19: Network event handler
Priority  3: System idle
Priority  1: Arduino loop() ← YOUR CODE HERE
Priority  0: Idle task (lowest)
```

**Implication:** WiFi/BT tasks can preempt `loop()` anytime if active.

**Solution Options:**
1. ✅ **Keep WiFi/BT disabled** (current state - zero preemption risk)
2. ⚠️ Increase loop() priority (risky - can starve system tasks)
3. ✅ **Use dedicated CAN task on Core 0** (Point 9 - true isolation)

### Relationship to Point 9 (Dual-Core Strategy)

**Point 6 (Current Analysis):** FreeRTOS is ALWAYS present, hidden beneath Arduino  
**Point 9 (Future Analysis):** EXPLICITLY leverage FreeRTOS for CAN isolation

**Key Difference:**
- **Point 6:** Passive awareness (FreeRTOS exists, design around it)
- **Point 9:** Active usage (create dedicated CAN task on Core 0)

**When to Move from Point 6 to Point 9:**
1. ❌ If SafeString/millisDelay achieves <0.3ms loop time → **Stay at Point 6** (no action needed)
2. ⚠️ If future features require WiFi/BT → **Move to Point 9** (isolate CAN from interference)
3. ⚠️ If loop time exceeds 1ms despite SafeString → **Move to Point 9** (performance issue)

### Testing Protocol

**Validate FreeRTOS Impact:**
1. ✅ Measure loop time with loopTimer (baseline)
2. ✅ Check for jitter (min/max/avg over 5 seconds)
3. ✅ Compare GPTimer vs micros() timestamps (detect task switching)
4. ✅ Monitor vTaskList() output (verify no unexpected high-priority tasks)

**Expected Results (WiFi/BT disabled):**
```
loop time: 0.1-0.3ms (consistent)
jitter: <0.1ms (minimal FreeRTOS overhead)
task switches: Rare (only system housekeeping)
```

**Failure Criteria (triggers Point 9 investigation):**
```
loop time: >1ms (too slow)
jitter: >1ms (excessive preemption)
task switches: Frequent (competing tasks detected)
```

### Verdict: Passive Awareness Sufficient

✅ **FreeRTOS is present** but impact minimal with WiFi/BT disabled  
✅ **SafeString/millisDelay** work perfectly WITH FreeRTOS (no blocking = good citizen)  
✅ **GPTimer** immune to task switching (hardware counter, not software micros())  
✅ **Current architecture safe** (no high-priority competing tasks)  
⏸️ **Point 9 deferred** until WiFi/BT needed or performance insufficient

**Action Items:**
- [ ] Measure loop time with loopTimer to establish baseline
- [ ] Verify no unexpected tasks via `vTaskList()` debug command
- [ ] Document FreeRTOS presence in system architecture docs
- [ ] Reserve Point 9 (explicit dual-core) as future enhancement

**Recommendation:** Proceed with SafeString/millisDelay implementation (Points 1-2), measure FreeRTOS impact, decide Point 9 necessity after testing.

---

## POINT 7: IF VS WHILE LOOPS (CONTROL FLOW IMPACT)

### Question: Do If vs While Loops Affect CAN Polling Frequency?

**Answer:** ✅ **YES - But Only if Used Incorrectly**

### Anti-Pattern: Blocking While Loops

**WRONG - Spin-Wait Blocks Loop:**
```cpp
// Waits in tight loop, CAN polling STOPS
while (!condition) {
    // Stuck here until condition true
    // loop() never returns to poll CAN
}
```

**Impact:**
- ❌ Loop iteration frozen (no CAN polling)
- ❌ FreeRTOS task blocked (can't yield)
- ❌ Watchdog risk if condition never met
- ❌ Classic blocking pattern (same as delay())

**Example from Codebase (FIXED in refactor):**
```cpp
// OLD - Blocking wait for motor response
motor.setPosition(target);
while (motor.getVelocity() > 0.1) {
    delay(10);  // Double-blocking! While + delay
}
// CAN polling STOPPED for entire move duration
```

### Correct Pattern: If Statements + State Machine

**CORRECT - Non-Blocking State Checks:**
```cpp
// State machine with if checks
switch (state) {
    case WAITING_FOR_MOTOR:
        if (motor.getVelocity() < 0.1) {
            state = MOTOR_STOPPED;  // Transition once
        }
        break;  // Returns to loop(), CAN polled
    
    case MOTOR_STOPPED:
        // Next action
        break;
}
// Loop returns, CAN polled every iteration
```

**Impact:**
- ✅ Single pass through state machine (~microseconds)
- ✅ Loop returns immediately to poll CAN
- ✅ Condition checked again on next iteration
- ✅ Non-blocking wait pattern

### Exception: Beneficial While Loops

**GOOD - Drain Hardware FIFO:**
```cpp
// Aggressive FIFO draining (REQUIRED)
while (ESP32Can.readFrame(rxFrame, 0)) {
    processCANMessage(rxFrame);  // Fast processing
    // Keep draining until FIFO empty
}
// Exit when FIFO empty, continue loop
```

**Why This is Correct:**
1. ✅ **Short duration** - Drains 8-16 messages in <1ms total
2. ✅ **Prevents overflow** - Empties FIFO before more messages arrive
3. ✅ **Hardware-bound** - readFrame() returns false when empty (guaranteed exit)
4. ✅ **No external wait** - Not waiting on motor/sensor, just draining buffer

**Performance Math:**
```
FIFO size: 16 messages max
Processing time: ~50µs per message (parse + store)
Total drain time: 16 × 50µs = 800µs = 0.8ms
Target loop time: 0.1-0.3ms normal, 0.8ms if FIFO full
Acceptable: YES (rare burst, not sustained)
```

### While Loop Classification

**❌ BLOCKING (Forbidden):**
```cpp
// Waiting for external condition
while (motor.getVelocity() > 0.1) { }
while (!Serial.available()) { }
while (millis() - start < timeout) { }
while (sensorValue < threshold) { }
```

**✅ NON-BLOCKING (Acceptable):**
```cpp
// Draining bounded buffer
while (ESP32Can.readFrame()) { }
while (Serial.available()) { char c = Serial.read(); }

// Bounded iteration with exit guarantee
while (count < MAX_ITERATIONS) {
    doThing();
    count++;
}
```

**Key Difference:** Bounded vs Unbounded
- ✅ Bounded: Fixed limit (buffer size, iteration count)
- ❌ Unbounded: External condition (motor state, sensor reading)

### Current Codebase Analysis

**Module State Machines (CORRECT):**
```cpp
// Refill.cpp - Non-blocking if statements
void Refill::update(MotorWrapper& motor) {
    switch (step) {
        case SETTING_LIMITS:
            if (now - lastCommandTime >= CAN_COMMAND_GAP_MS) {
                motor.setMotorLimits(...);
                step = WAIT_LIMITS;
            }
            break;  // ← Returns immediately
        
        case WAIT_LIMITS:
            if (now - lastCommandTime >= CAN_COMMAND_GAP_MS) {
                step = SETTING_TRAJ;
            }
            break;  // ← Returns immediately
        
        // ... all steps return quickly
    }
}
// Total execution: <100µs per call
```

**CanBusHandler (CORRECT - Beneficial While):**
```cpp
// CanBusHandlerV2.cpp - Aggressive FIFO draining
void CanBusHandlerV2::processIncomingMessages() {
    CAN_FRAME rxFrame;
    while (ESP32Can.readFrame(rxFrame, 0)) {  // ← Drain until empty
        uint64_t timestamp = hwTimer.micros();
        broadcastStore.storeMessage(rxFrame.id, rxFrame.data, timestamp);
    }
    // Exit when FIFO empty, <1ms total
}
```

**MessageBuffer TX Queue (CORRECT):**
```cpp
// MessageBuffer.cpp - Single message per loop
bool MessageBuffer::processQueue() {
    if (!hasPendingCANCommand()) return false;
    if (now - lastCANSendTime < CAN_SEND_INTERVAL_MS) return false;
    
    // Process ONE command, then return
    sendNextCANCommand();
    return true;
}
// No while loop - processes queue incrementally
```

### Control Flow Best Practices

**1. State Machines Over Loops:**
```cpp
// ❌ WRONG
void injection() {
    motor.startMove();
    while (motor.isMoving()) { }  // Blocks
    motor.hold();
    while (holdTimer < 5000) { }  // Blocks
}

// ✅ CORRECT
enum InjectionState { FILLING, PACKING, COMPLETE };
InjectionState state = FILLING;

void injection() {
    switch (state) {
        case FILLING:
            if (motor.getVelocity() < 0.1) {
                state = PACKING;
                packTimer.start(5000);
            }
            break;
        case PACKING:
            if (packTimer.justFinished()) {
                state = COMPLETE;
            }
            break;
    }
}
```

**2. Early Returns Over Nested Ifs:**
```cpp
// ❌ CLUTTERED
void update() {
    if (condition1) {
        if (condition2) {
            if (condition3) {
                doThing();
            }
        }
    }
}

// ✅ CLEAR
void update() {
    if (!condition1) return;  // Fast exit
    if (!condition2) return;  // Fast exit
    if (!condition3) return;  // Fast exit
    doThing();
}
```

**3. Bounded Loops Only:**
```cpp
// ❌ UNBOUNDED
while (sensorValue < target) {
    sensorValue = readSensor();  // Could loop forever
}

// ✅ BOUNDED
for (int i = 0; i < MAX_RETRIES; i++) {
    sensorValue = readSensor();
    if (sensorValue >= target) break;
    delay(10);  // With non-blocking, use millisDelay
}
// Guaranteed exit after MAX_RETRIES
```

### Performance Impact Measurement

**If Statement (State Machine):**
```
switch (state) { ... }  // ~10 CPU cycles
Single case execution: ~50 CPU cycles
Total: ~60 cycles = ~0.25µs @ 240MHz
```

**Blocking While Loop:**
```
while (motor.isMoving()) { }
Iteration: ~100 cycles per check
Duration: 2000ms motor move
Total: 2,000,000µs BLOCKED
```

**Impact Ratio:** 8,000,000:1 (while loop is 8 MILLION times slower)

### Verdict: If Statements Mandatory

✅ **State machines with if/switch** = fast, non-blocking, CAN-friendly  
❌ **While loops for external waits** = blocking, forbidden  
✅ **While loops for buffer draining** = acceptable, bounded  

**Current Codebase Status:**
- ✅ All modules use state machines (Refill, Compression, ReadyToInject, etc.)
- ✅ No blocking while loops found
- ✅ Beneficial while loop in CanBusHandler (FIFO draining)
- ✅ MessageBuffer uses incremental processing (no loops)

**Action Items:**
- [ ] Code review: Search for `while` patterns, verify all are bounded
- [ ] Add lint rule: Flag unbounded while loops in code reviews
- [ ] Document state machine pattern in architecture guide
- [ ] Measure actual loop time to confirm <0.3ms target

**Recommendation:** Continue using state machine pattern, maintain zero blocking while loops except for hardware buffer draining.

---

## POINT 8: TIMER INTERRUPTS & ESP32 PERIPHERALS

### Question: Should We Use Timer Interrupts or Hardware ISRs for CAN Polling?

**Answer:** ⚠️ **NO for CAN Polling, YES for GPTimer Timestamps (Already Implemented)**

### ESP32 Interrupt Capabilities

**Available Interrupt Sources:**
1. **GPTimer** (General Purpose Timer) - Hardware timer with ISR capability
2. **TWAI (CAN)** - RX/TX interrupt mode available
3. **GPIO** - Pin change interrupts
4. **UART** - Serial RX/TX interrupts
5. **SPI/I2C** - Peripheral interrupts

### Option 8A: Timer ISR to Trigger CAN Polling

**Proposal:** Use GPTimer interrupt to poll CAN at fixed intervals
```cpp
// Timer ISR fires every 100µs
void IRAM_ATTR timerISR() {
    pollCANMessages();  // Call from ISR
}

void setup() {
    // Configure timer to fire ISR every 100µs
    gptimer_alarm_config_t alarm = {
        .alarm_count = 100,  // 100µs interval
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true
    };
    gptimer_set_alarm_action(timer, &alarm);
    gptimer_register_event_callbacks(timer, &cbs, NULL);
}
```

**Problems:**
1. ❌ **ISR Context Limitations:**
   - Can't use malloc/free
   - Can't use FreeRTOS blocking calls
   - Can't use Serial.print() (crashes)
   - Must be in IRAM (faster, limited memory)
   
2. ❌ **Interrupt Overhead:**
   - Context switch: ~2-5µs per ISR entry/exit
   - 100µs interval = 10,000 ISRs/second
   - Overhead: 20,000-50,000µs/sec = 2-5% CPU waste
   
3. ❌ **Race Conditions:**
   - ISR writes to BroadcastDataStore
   - Main loop reads from BroadcastDataStore
   - Need mutex/critical section (complex, slow)
   
4. ❌ **Worse Than Polling:**
   - Loop already runs every 0.3ms = 3,333 polls/sec
   - 100µs ISR = 10,000 polls/sec (3x more, but overhead kills benefit)

**Verdict:** ❌ **Rejected** - Interrupt overhead + complexity > benefit

### Option 8B: TWAI RX Interrupt Mode

**TWAI Interrupt Source Clarification:**
- **ESP-IDF TWAI Driver:** YES - Supports interrupt mode natively (`twai_driver_install()` with `TWAI_ALERT_RX_DATA`)
- **ESP32_CAN Arduino Library:** ⚠️ UNCLEAR - May expose interrupt API, may require dropping to ESP-IDF layer
- **Documentation:** ESP-IDF docs confirm TWAI peripheral has RX interrupt capability

**Proposal:** Use TWAI peripheral interrupt instead of polling (on Core 1 - main loop)
```cpp
// TWAI fires interrupt when message received
void IRAM_ATTR twaiRxISR() {
    CAN_FRAME rxFrame;
    ESP32Can.readFrame(rxFrame, 0);
    // Process message in ISR context
}

void setup() {
    // Enable TWAI RX interrupt (ESP-IDF or library wrapper)
    ESP32Can.setRxInterrupt(twaiRxISR);
}
```

**Problems (When ISR Runs on Core 1 - Same Core as Main Loop):**
1. ❌ **ISR Context Issues:**
   - Can't call BroadcastDataStore methods safely (mutex needed)
   - Can't use BufferedOutput methods (FreeRTOS queue operations)
   - Must queue to ISR-safe buffer, process in loop anyway
   - Note: We use SafeString BufferedOutput (not Serial.print), but ISR still can't call it
   
2. ❌ **Interrupts Main Loop:**
   - Your code on Core 1 gets interrupted by TWAI ISR
   - State machine execution paused mid-operation
   - GPTimer timestamps unaffected (hardware counter), but loop timing disrupted
   
3. ❌ **Interrupt Latency:**
   - ISR response: ~5-10µs after message arrival
   - Polling response: ~150µs average (0.3ms loop / 2)
   - Benefit: ~140µs faster (negligible for 100ms cyclic rate)
   
4. ❌ **FIFO Still Fills:**
   - ISR processes ONE message per interrupt
   - Burst of 7 messages = 7 interrupts = 7 ISR calls = ~70µs interrupt time
   - Loop while-drain processes all 7 in one pass = ~800µs
   
5. ❌ **Library Support:**
   - ESP32_CAN library may not expose interrupt API cleanly
   - May require ESP-IDF low-level TWAI driver (compatibility risk)

**Verdict (Core 1 Interrupts):** ❌ **Rejected** - Interrupts your main code, minimal benefit

**BUT... What if ISR Runs on Core 0?** → See Point 9 for hybrid interrupt + dual-core approach!

### Option 8C: GPTimer for Timestamps (ALREADY CORRECT)

**Current Implementation (Point 3.9):**
```cpp
// GPTimer hardware counter (NO ISR)
class HardwareTimer {
private:
    gptimer_handle_t timer_;
    
public:
    void begin() {
        gptimer_config_t config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000,  // 1µs resolution
        };
        gptimer_new_timer(&config, &timer_);
        gptimer_enable(timer_);
        gptimer_start(timer_);
        // NO ISR registered - just free-running counter
    }
    
    uint64_t micros() {
        uint64_t count;
        gptimer_get_raw_count(timer_, &count);
        return count;  // Read anytime, zero overhead
    }
};
```

**Why This is Correct:**
1. ✅ **Zero ISR overhead** - Free-running counter, no interrupts
2. ✅ **Read anytime** - `gptimer_get_raw_count()` is fast (~0.2µs)
3. ✅ **Immune to ISRs** - Hardware counter unaffected by task switching
4. ✅ **Simple code** - No interrupt handlers, no race conditions
5. ✅ **Perfect timestamps** - 1µs resolution, 64-bit (no rollover)

**Verdict:** ✅ **KEEP** - Already optimal, no ISR needed

### Why Polling Beats Interrupts (ESP32 Context)

**Polling Advantages:**
```
Loop time: 0.3ms = 3,333 polls/second
100ms broadcast = 300 poll opportunities
Message sits in FIFO: <0.15ms average (0.3ms/2)
Processing latency: Negligible vs 100ms cycle time
```

**Interrupt Disadvantages:**
```
Context switch: 2-5µs per ISR
ISR restrictions: No malloc, no FreeRTOS, IRAM only
Race conditions: Need mutexes, critical sections
Complexity: ISR + main loop coordination
Benefit: ~100µs faster response (0.1% of cycle time)
```

**Cost/Benefit Ratio:** Interrupts add 10x complexity for 0.1% benefit

### ESP32 Peripheral Interrupt Summary

**When Interrupts Make Sense:**
1. ✅ **Asynchronous events** - Button presses, encoder pulses (rare, unpredictable)
2. ✅ **Hard real-time** - Stepper motor pulse generation (microsecond precision)
3. ✅ **Low-power modes** - Wake from sleep on event (not applicable here)

**When Polling is Better:**
1. ✅ **High-frequency periodic** - CAN cyclic broadcasts (100ms predictable)
2. ✅ **Buffered data** - FIFO draining (process batch efficiently)
3. ✅ **Simple code** - No ISR constraints, straightforward logic

**Our Use Case:**
- CAN: 100ms cyclic, FIFO buffered → **Polling wins**
- Timestamps: Free-running counter → **No ISR needed**
- Motors: State machine checking → **Polling wins**

### Current Architecture Validation

**GPTimer Usage (CORRECT):**
```cpp
// Point 3.9 - Already implemented correctly
uint64_t timestamp = hwTimer.micros();  // Read counter, no ISR
broadcastStore.storeMessage(canId, data, timestamp);
```

**CAN Polling (CORRECT):**
```cpp
// CanBusHandlerV2.cpp - Aggressive FIFO draining
void loop() {
    // ... other fast operations
    
    // Drain FIFO completely
    while (ESP32Can.readFrame(rxFrame, 0)) {
        uint64_t timestamp = hwTimer.micros();
        broadcastStore.storeMessage(rxFrame.id, rxFrame.data, timestamp);
    }
    
    // ... continue loop
}
// Runs 3,333 times/second, plenty fast
```

**No ISRs Required:**
- ✅ GPTimer: Free-running counter (read-only, no ISR)
- ✅ TWAI: Polling mode with while-drain (simple, fast)
- ✅ Buttons: Polled with debounce (adequate for human input)
- ✅ Temperature: Polled every loop (slow-changing value)

### Exception: Emergency Stop

**One Valid ISR Use Case:**
```cpp
// E-stop button - true safety-critical interrupt
void IRAM_ATTR eStopISR() {
    // Immediate motor shutdown, no loop delay
    digitalWrite(DC_CONTACTOR_PIN, LOW);  // Kill power
    emergencyStopFlag = true;  // Set flag for loop
}

void setup() {
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), eStopISR, FALLING);
}
```

**Why This ISR is Justified:**
1. ✅ **Safety-critical** - Cannot wait for loop iteration
2. ✅ **Simple action** - GPIO write only (safe in ISR)
3. ✅ **Rare event** - Not continuous overhead
4. ✅ **Hardware access** - Direct pin control (no shared data)

**Current Implementation:**
```cpp
// SafetyManager.cpp - Polled E-stop (ADEQUATE)
bool SafetyManager::checkSafety() {
    if (isEStopPressed()) {
        motor.emergencyStop();
        return false;
    }
}
// Called every loop (~300µs response time)
```

**Decision:** 300µs E-stop response is acceptable (motor inertia >> 300µs)

### Verdict: No Additional Interrupts Needed

✅ **GPTimer timestamps** - Already using hardware counter correctly (no ISR)  
❌ **Timer ISR for polling** - Overhead > benefit, polling sufficient  
❌ **TWAI RX interrupt** - Complexity >> 100µs benefit  
⚠️ **E-stop interrupt** - Optional enhancement, current polling adequate  

**Current Architecture Status:**
- ✅ GPTimer: Free-running counter (Point 3.9) - KEEP
- ✅ CAN: Polling with FIFO drain - KEEP
- ✅ Buttons: Polled with debounce - KEEP
- ✅ No ISR race conditions - SIMPLE

**Action Items:**
- [ ] Verify GPTimer implementation complete (Point 3.9 checklist)
- [ ] Measure actual loop time with loopTimer (validate <0.3ms target)
- [ ] Document "no ISR" decision in architecture guide
- [ ] Optional: Add E-stop GPIO interrupt for <100µs response (safety enhancement)

**Recommendation:** Continue with polling architecture, leverage GPTimer counter (no ISR), defer E-stop interrupt unless safety audit requires it.

---

## POINT 9: DUAL-CORE FREERTOS (CAN RX ISOLATION)

### Question: Should We Use Core 0 for Dedicated CAN RX Polling/Interrupts?

**Answer:** ⚠️ **DEFER Until Testing, But Architecture Looks Promising**

### ESP32 Dual-Core Architecture

**Physical Hardware:**
```
ESP32 has TWO Xtensa LX6 cores @ 240MHz each:
- Core 0 (PRO_CPU): Protocol CPU - typically WiFi/BT stack
- Core 1 (APP_CPU): Application CPU - Arduino loop() runs here by default
```

**Current Default Behavior:**
```
Core 0: System tasks (WiFi/BT disabled → mostly idle)
Core 1: Arduino setup() + loop() (your entire application)
```

**TWAI Peripheral:** Shared between both cores (accessed via ESP-IDF driver, thread-safe)

### Option 9A: Core 0 Dedicated Polling Task (No Interrupts)

**Architecture:**
```
Core 0: CAN RX polling task (tight loop, high priority)
    ↓
  Queue (FreeRTOS xQueue - thread-safe, ISR-safe)
    ↓
Core 1: Main loop reads from queue → BroadcastDataStore
```

**Implementation:**
```cpp
// FreeRTOS queue for CAN messages
QueueHandle_t canRxQueue;

struct CANMessage {
    uint32_t canId;
    uint8_t data[8];
    uint64_t timestamp;
};

// Core 0 dedicated CAN polling task
void canRxTaskCore0(void* pvParameters) {
    CANMessage msg;
    CAN_FRAME rxFrame;
    
    while (1) {
        // Tight polling loop - drains FIFO aggressively
        while (ESP32Can.readFrame(rxFrame, 0)) {
            msg.canId = rxFrame.id;
            memcpy(msg.data, rxFrame.data, 8);
            msg.timestamp = hwTimer.micros();  // GPTimer read (fast)
            
            // Send to queue (non-blocking, fails if full)
            xQueueSend(canRxQueue, &msg, 0);
        }
        
        // Yield to other tasks (prevents watchdog)
        vTaskDelay(0);  // Immediate reschedule, ~10µs
    }
}

void setup() {
    // Create queue (depth = 32 messages)
    canRxQueue = xQueueCreate(32, sizeof(CANMessage));
    
    // Launch task on Core 0, high priority
    xTaskCreatePinnedToCore(
        canRxTaskCore0,   // Function
        "CAN_RX_Core0",   // Name
        4096,             // Stack size (4KB)
        NULL,             // Parameters
        2,                // Priority (higher than loop = 1)
        NULL,             // Task handle
        0                 // Core 0 (PRO_CPU)
    );
}

void loop() {
    // Core 1 - Main application code (NEVER INTERRUPTED)
    CANMessage msg;
    
    // Process all queued CAN messages
    while (xQueueReceive(canRxQueue, &msg, 0) == pdTRUE) {
        broadcastStore.storeMessage(msg.canId, msg.data, msg.timestamp);
    }
    
    // Rest of loop() - state machines, modules, etc.
    // Runs WITHOUT interruption from CAN polling
}
```

**Advantages:**
1. ✅ **Core 1 never interrupted** - Main loop runs smoothly, no ISR disruption
2. ✅ **Guaranteed CAN coverage** - Core 0 polls continuously (>10,000 Hz possible)
3. ✅ **True parallelism** - CAN polling + main loop run simultaneously
4. ✅ **Simple code** - No ISR constraints, just tight polling loop
5. ✅ **Thread-safe queue** - FreeRTOS xQueue handles synchronization
6. ✅ **GPTimer timestamps** - Still immune to interrupts (hardware counter)

**Performance Math:**
```
Core 0 polling task:
- Poll + check: ~50µs per iteration
- vTaskDelay(0): ~10µs scheduler overhead
- Total cycle: ~60µs = 16,666 polls/second

100ms broadcast cycle:
- Poll opportunities: 1,666 per cycle (vs 300 with 0.3ms loop)
- Average message latency: ~30µs (vs 150µs)

Queue depth:
- 32 messages × 20 bytes = 640 bytes RAM
- 7 messages per burst → 25 message headroom
- Overflow risk: ZERO (even at 20ms cyclic rate)
```

**Disadvantages:**
1. ⚠️ **Complexity increase** - FreeRTOS task management, queue handling
2. ⚠️ **Memory cost** - 4KB stack + 640 byte queue = 4.6KB (~1.4% of RAM)
3. ⚠️ **Core 0 dedication** - Can't use Core 0 for other tasks (but it's idle anyway)
4. ⚠️ **Debugging harder** - Multi-core debugging more complex

**CRITICAL INSIGHT - Core 1 Performance Impact:**
- 80% Core 0 CPU usage sounds high, BUT it's 80% of Core 0 (which is currently idle)
- If polling stays on Core 1: That 80% comes FROM your main loop performance!
- **High-rate polling on Core 1 = slower state machines, slower module updates**
- Moving to Core 0 frees Core 1 to run main application at full speed
- This alone justifies Core 0 architecture (even without interrupts)

**Verdict:** ✅ **Viable, but defer until testing shows need**

### Option 9B: Core 0 TWAI Interrupt + Queue (HYBRID APPROACH)

**Your Proposed Architecture:**
```
TWAI Interrupt (fires on message arrival)
    ↓
ISR on Core 0 (immediate response, <10µs)
    ↓
xQueueSendFromISR() (ISR-safe queue write)
    ↓
Core 1: Main loop reads queue → BroadcastDataStore
```

**Implementation:**
```cpp
// FreeRTOS queue
QueueHandle_t canRxQueue;

// TWAI interrupt handler (runs on Core 0 automatically)
void IRAM_ATTR twaiRxISR() {
    CAN_FRAME rxFrame;
    CANMessage msg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Read frame (fast, <5µs)
    if (ESP32Can.readFrame(rxFrame, 0)) {
        msg.canId = rxFrame.id;
        memcpy(msg.data, rxFrame.data, 8);
        msg.timestamp = hwTimer.micros();  // GPTimer (ISR-safe)
        
        // Send to queue from ISR (ISR-safe variant)
        xQueueSendFromISR(canRxQueue, &msg, &xHigherPriorityTaskWoken);
    }
    
    // Yield if higher priority task woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setup() {
    canRxQueue = xQueueCreate(32, sizeof(CANMessage));
    
    // Enable TWAI interrupt (ESP-IDF TWAI driver)
    twai_driver_install(&twai_config, &twai_timing_config, &twai_filter_config);
    twai_start();
    // Interrupt automatically assigned to Core 0 (peripheral interrupt)
}

void loop() {
    // Core 1 - Main application (NEVER INTERRUPTED by TWAI ISR)
    CANMessage msg;
    
    while (xQueueReceive(canRxQueue, &msg, 0) == pdTRUE) {
        broadcastStore.storeMessage(msg.canId, msg.data, msg.timestamp);
    }
    
    // Rest of loop() - runs uninterrupted
}
```

**Advantages (vs 9A Polling):**
1. ✅ **Even lower latency** - ISR fires within ~5µs of message arrival (vs ~30µs polling)
2. ✅ **Core 1 still never interrupted** - ISR runs on Core 0 only
3. ✅ **Guaranteed capture** - Hardware interrupt can't miss messages
4. ✅ **Lower Core 0 CPU usage** - ISR only runs on message arrival (not continuous polling)
5. ✅ **Same queue mechanism** - Core 1 reads queue identically

**Disadvantages (vs 9A Polling):**
1. ⚠️ **ISR context limitations** - IRAM code, no malloc, limited functions
2. ⚠️ **ESP-IDF dependency** - Must use ESP-IDF TWAI driver (not Arduino library)
3. ⚠️ **Slightly more complex** - ISR + queue vs just queue
4. ⚠️ **FIFO burst handling** - 7-message burst = 7 ISR calls (vs 1 while-loop drain)

**Critical Clarification on "Core 1 Not Interrupted":**
```
TWAI peripheral interrupt → Interrupt controller routes to Core 0
Core 0: ISR executes (Core 1 UNAFFECTED, continues running)
Core 0: ISR writes to queue (thread-safe, atomic)
Core 1: Reads queue when ready (no forced interruption)

Result: Core 1 main loop runs CONTINUOUSLY, checks queue at its leisure
```

**This is the key advantage you identified:** "Don't like being interrupted" - your main code on Core 1 is NEVER interrupted because ISR runs on Core 0!

**Verdict:** ✅ **Architecturally superior to 9A, but requires ESP-IDF TWAI driver**

### Performance Comparison Table

| Metric | Current (Polling) | 9A (Core 0 Poll) | 9B (Core 0 ISR) |
|--------|------------------|------------------|-----------------|
| **Core 1 Interruption** | None | None | None ✅ |
| **Message Latency** | ~150µs avg | ~30µs avg | ~5µs avg ✅ |
| **Polling Rate** | 3,333 Hz | 16,666 Hz | Event-driven ✅ |
| **Core 0 CPU Usage** | 0% | ~80% | ~0.5% ✅ |
| **Code Complexity** | Simple ✅ | Medium | Medium |
| **Memory Overhead** | 0 bytes ✅ | 4.6KB | 4.6KB |
| **ESP-IDF Dependency** | No ✅ | No ✅ | Yes |
| **Guaranteed Capture** | 300 polls/cycle | 1,666 polls/cycle | Hardware ✅ |

### When to Implement Point 9

**IMPLEMENT IMMEDIATELY - Do NOT Defer:**

**Reasoning:**
1. ✅ **Build robust foundation first** - Don't wait for crashes to rewrite message system
2. ✅ **100% guaranteed RX = confident debugging** - If messages occasionally lost, can't isolate bugs elsewhere
3. ✅ **Future-proof architecture** - WiFi will be on Display ESP32 (UART link), not Controller ESP32
4. ✅ **ISR overhead negligible** - ~0.5% Core 0 usage won't impact future tasks
5. ✅ **Polling alternative worse** - 80% Core 0 polling would compete with any future Core 0 tasks
6. ✅ **Core 1 performance preserved** - Moving CAN to Core 0 keeps main loop fast
7. ✅ **Rock solid base** - GPTimer + Core 0 ISR = guaranteed message capture foundation

**Counter-Argument to "Wait and See" Approach:**
- ❌ Discovering message loss later = extensive debugging, potential rewrites
- ❌ Uncertainty about message capture = can't trust any test results
- ❌ High-rate polling on Core 1 = slower main loop (invisible performance cost)
- ❌ Deferred complexity = harder to integrate into mature codebase

**WiFi/BT on Controller ESP32:**
- ⚠️ Considered reckless for safety-critical machine control
- ✅ Display ESP32 provides wireless interface via UART (safe isolation)
- ✅ Core 0 remains available for ISR-based CAN (not WiFi stack)

### Implementation Roadmap (IMMEDIATE - Point 9B Preferred)

**Phase 1: Foundation (Points 1-3)**
1. ✅ Implement SafeString/millisDelay (Point 1-2)
2. ✅ Implement GPTimer (Point 3.9)
3. ✅ Enhance BroadcastDataStore with ring buffers (Point 3.7)

**Phase 2: Core 0 ISR Architecture (Point 9B - PRIORITY)**
1. Research ESP-IDF TWAI interrupt API compatibility with Arduino framework
   - Confirm `twai_driver_install()` + `TWAI_ALERT_RX_DATA` works
   - Test ESP32_CAN library interrupt exposure vs direct ESP-IDF usage
2. Create FreeRTOS queue (ISR-safe, 32-message depth)
   - `QueueHandle_t canRxQueue = xQueueCreate(32, sizeof(CANMessage));`
3. Implement TWAI ISR on Core 0
   - Read frame, timestamp with GPTimer, `xQueueSendFromISR()`
   - IRAM_ATTR function, minimal processing
4. Modify Core 1 loop() to read from queue
   - `while (xQueueReceive(canRxQueue, &msg, 0))` drain queue
   - Store to BroadcastDataStore ring buffers
5. Test ISR assignment to Core 0 (use ESP-IDF `esp_intr_get_cpu()` or logging)

**Phase 3: Validation & Performance Testing**
1. Measure message latency with GPTimer timestamps (target <10µs)
2. Load test: 20ms cyclic rate + 7-message bursts (verify zero loss)
3. Verify Core 1 loop time with loopTimer (should be faster without polling)
4. Test protected window timing (Point 3.5) with guaranteed capture
5. Stress test: Command queue + simultaneous broadcasts (no collisions)

**Phase 4: Protected Windows & Response Correlation**
1. Implement `isSafeToSendCommand()` (Point 3.5)
2. Add TX command correlation (Point 3.8)
3. Test response detection within 2-5ms window
4. Validate cyclic rate optimization (test 40ms, 30ms, 20ms)

**Fallback Plan (If ESP-IDF TWAI ISR Incompatible):**
- Implement 9A (Core 0 Polling Task) as interim solution
- Still provides Core 1 isolation and guaranteed capture
- 16,666 Hz polling sufficient until ISR path resolved

### Verdict: Implement Point 9B Immediately

✅ **Option 9B (Core 0 ISR + Queue)** is the correct production architecture  
✅ **Guarantees message capture** - Hardware interrupt can't miss messages  
✅ **Never interrupts Core 1** - Main application runs uninterrupted  
✅ **Future-proof foundation** - Rock solid base for debugging everything else  
✅ **Negligible overhead** - ~0.5% Core 0 CPU, won't impact future tasks  
✅ **Better than polling** - ISR beats 80% Core 1 CPU usage from polling  

**Action Items (PRIORITY):**
- [ ] Research ESP-IDF TWAI interrupt API integration with Arduino framework
- [ ] Implement Phase 1 (SafeString + GPTimer + BroadcastDataStore) - foundation
- [ ] Implement Phase 2 (Core 0 ISR + Queue) - guaranteed capture
- [ ] Validate Phase 3 (performance testing) - measure latency, confirm zero loss
- [ ] Deploy Phase 4 (protected windows + response correlation) - complete system

**Recommendation:** Implement full architecture (Points 1-3 + 9B) as integrated system. Core 0 ISR provides 100% guaranteed message capture foundation - essential for confident debugging and future development. Don't wait for problems to appear; build robust system from the start.

---

## POINT 5: MILLISDELAY COMPLETENESS EVALUATION

### Question: Is millisDelay Sufficient or Do We Need Alternatives?

**Answer:** ✅ **millisDelay is COMPLETE and SUFFICIENT**

### Feature Completeness Analysis

**Required Features for CAN RX System:**
1. ✅ Non-blocking delays - `start(ms)`, `justFinished()`
2. ✅ Drift-free cyclic timers - `repeat()` (vs `restart()` which drifts)
3. ✅ Query remaining time - **`remaining()`** returns ms left
4. ✅ Query start time - **`getStartTime()`** returns last start timestamp
5. ✅ Query delay value - **`delay()`** returns configured delay
6. ✅ Manual control - `stop()`, `finish()`, `isRunning()`
7. ✅ Zero memory overhead - Simple class, ~12 bytes per instance
8. ✅ No dependencies - Part of SafeString library (already using)

**Comparison to Alternatives:**

| Feature | millisDelay | SimpleTimer | Ticker | FreeRTOS Timers |
|---------|-------------|-------------|--------|----------------|
| Non-blocking | ✅ | ✅ | ✅ | ✅ |
| Drift-free repeat | ✅ `repeat()` | ❌ | ⚠️ Complex | ✅ |
| Query remaining | ✅ `remaining()` | ❌ | ❌ | ⚠️ API heavy |
| Query start time | ✅ `getStartTime()` | ❌ | ❌ | ⚠️ Complex |
| Memory per timer | 12 bytes | ~50 bytes | ~80 bytes | ~100 bytes |
| Dependencies | SafeString only | External lib | ESP32-specific | RTOS overhead |
| Learning curve | Minimal | Low | Medium | High |

### Previous Research Errors (CORRECTED)

**ERROR 1:** "millisDelay lacks `remaining()` method"  
**TRUTH:** millisDelay HAS `remaining()` - returns ms left until finish

**ERROR 2:** "millisDelay lacks `getStartTime()` method"  
**TRUTH:** millisDelay HAS `getStartTime()` - returns last start time

**ERROR 3:** "Need SRTimer library as alternative"  
**TRUTH:** No such library exists (confusion with SR Library author Tom Jennings, who is NOT SafeString author Matthew Ford)

**ERROR 4:** "loopTimer is from SR Library"  
**TRUTH:** loopTimer is part of SafeString library V3+ (performance monitoring tool, not delay timer)

### Application to Current Codebase

**Existing Usage (Correct Pattern):**
```cpp
// ReadyToInject.cpp - 30-second micro-compression timer
millisDelay microCompressionTimer;
microCompressionTimer.start(READY_MICRO_INTERVAL_MS);  // 30000ms

if (microCompressionTimer.justFinished()) {
    // Trigger micro-compression
    microCompressionTimer.repeat();  // Drift-free restart
}
```

**New Usage (Response Timeout Detection):**
```cpp
// MotorWrapper.cpp - TX command timeout
millisDelay responseTimer;
responseTimer.start(5);  // 5ms timeout

while (responseTimer.isRunning()) {
    if (broadcastStore.findResponse(canId, txTime, 5000)) {
        responseTimer.stop();  // Cancel timer, response received
        break;
    }
    // Loop continues, CAN still polled
}

if (responseTimer.justFinished()) {
    // Timeout - no response within 5ms
    logError("TX timeout");
}
```

**Display Integration (Using `remaining()`):**
```cpp
// Display countdown timer
if (injectionTimer.isRunning()) {
    uint32_t timeLeft = injectionTimer.remaining();
    displayComms.sendCountdown(timeLeft);  // Update display every loop
}
```

### loopTimer Clarification (Separate Tool)

**loopTimer** is ALSO part of SafeString library, but serves different purpose:
- **Purpose:** Performance monitoring (min/max/avg loop execution time)
- **NOT for timing delays** - Use millisDelay for that
- **Usage:**
  ```cpp
  loopTimer timer;
  timer.check(bufferedOut);  // Prints stats every 5 seconds
  // Output: "loop us Latency / 5sec max:1408 avg:254"
  ```
- **Remove after testing** - Adds 1-2ms overhead every 5 seconds

### Verdict: No Alternative Needed

✅ **millisDelay is feature-complete**  
✅ **Already integrated in codebase** (SafeString dependency)  
✅ **Zero additional libraries needed**  
✅ **Minimal memory footprint** (12 bytes per timer)  
✅ **Proven reliability** (SafeString library V4.1.42, mature codebase)  

**Recommendation:** Continue using millisDelay, leverage `remaining()` and `getStartTime()` methods for enhanced functionality.

**Action Items:**
- [ ] Review all existing millisDelay usage for `repeat()` vs `restart()` (use `repeat()` for drift-free)
- [ ] Add `remaining()` queries for display countdown timers
- [ ] Use `getStartTime()` for TX command correlation timestamps
- [ ] Remove loopTimer after performance validation complete

---

## PRODUCTION ARCHITECTURE LAYERS

### Layer 0: Hardware Timing
- **ESP32 GPTimer** (1µs resolution, 64-bit counter)
- `hwTimer.micros()` replaces all `micros()` calls throughout codebase
- Immune to ISRs, no rollover, perfect accuracy
- Single source of truth for all timing

### Layer 1: Hardware CAN (ESP32 TWAI)
- **RX FIFO:** 8-16 message hardware buffer
- **Aggressive draining:** `while (readFrame())` empties FIFO every loop
- **TX/RX pins:** Independent (GPIO5/GPIO4), simultaneous capable
- Hardware fills FIFO automatically, software drains proactively

### Layer 2: CanBusHandler
- Calls `ESP32Can.readFrame()` in while-loop until FIFO empty
- Timestamps ALL messages with `hwTimer.micros()`
- Logs TX commands with timestamps to BroadcastDataStore
- Implements `isSafeToSendCommand()` protected window check
- Routes messages to BroadcastDataStore for storage

### Layer 3: BroadcastDataStore (Enhanced)
**Storage:**
- Ring buffer per message type (10 slots × 16 bytes)
- TX command log (5 recent commands)
- Protected window metadata (last heartbeat time, cycle period)

**API Methods:**
```cpp
// Normal module access
float getLatestEncoderPos();
float getLatestEncoderVel();
// ... (one per message type)

// Response detection
bool findResponse(uint32_t canId, uint64_t txTime, uint64_t windowUs);
bool hasUnprocessed(MessageType type, uint64_t sinceTime);
void markProcessed(MessageType type, uint64_t timestamp);

// Protected window
bool isSafeToSendCommand();
uint64_t getLastHeartbeatTime();
uint64_t getCyclePeriod();
void setCyclePeriod(uint64_t periodUs);

// TX logging (called by CanBusHandler)
void logTxCommand(uint32_t canId, uint64_t txTime);
void markResponseReceived(uint32_t canId, uint64_t txTime);
```

### Layer 4: Module Command Flow
**Sending Commands:**
```cpp
// Check protected window
if (!broadcastStore.isSafeToSendCommand()) {
    return;  // Wait for golden zone
}

// Queue command (MessageBuffer handles 10ms spacing)
messageBuffer.queueCANCommand(cmdData);

// When sent (by CanBusHandler)
uint64_t txTime = hwTimer.micros();
ESP32Can.writeFrame(frame);
broadcastStore.logTxCommand(canId, txTime);
```

**Waiting for Response:**
```cpp
uint64_t txTime = hwTimer.micros();
motor.sendCommand();

// Non-blocking wait (using millisDelay)
millisDelay responseTimer;
responseTimer.start(5);  // 5ms timeout

while (!responseTimer.justFinished()) {
    if (broadcastStore.findResponse(canId, txTime, 5000)) {
        // Response received!
        break;
    }
    // Loop continues, CAN still polled
}

if (!broadcastStore.findResponse(canId, txTime, 5000)) {
    // Timeout - no response
    handleError();
}
```

### Layer 5: SafeString/millisDelay Foundation
**Serial Output:**
```cpp
BufferedOutput bufferedOut;  // Global instance
bufferedOut.connect(Serial);

// In loop
bufferedOut.nextByteOut();  // Every iteration

// Everywhere else
sfStream.print("Debug message");  // Non-blocking queue
```

**Timing:**
```cpp
millisDelay preRollTimer;
millisDelay postTxTimer;
// ... (one timer per timing need)

// In loop/state machine
if (preRollTimer.justFinished()) {
    // Transition state
}
```

**Result:**
- Loop time: **0.1-0.3ms** consistently
- CAN polling: **5,000+ times/second**
- Zero blocking operations

---

## CYCLIC BROADCAST RATE STRATEGY

### Testing Protocol (Sequential)

1. **Baseline: 100ms** ✅
   - Protected window: 65-95ms (30ms wide)
   - Proven safe from experimental data
   - Deploy first, validate zero message loss

2. **Test: 40ms** ⏸️
   - Protected window: 5-35ms (30ms wide, if formula holds)
   - 2.5x faster updates than 100ms
   - Movement per update: 0.1 turns = 0.08g plastic

3. **Test: 30ms** ⏸️
   - Protected window: -5 to 25ms (ONLY if percentage-based, not absolute)
   - 3.3x faster updates
   - Movement per update: 0.075 turns = 0.06g plastic

4. **Test: 20ms** ⏸️
   - Protected window: IMPOSSIBLE if 30-35ms is absolute threshold
   - 5x faster updates
   - Movement per update: 0.05 turns = 0.04g plastic
   - **High injection risk** 🔴

### Hybrid Rate Configuration

**Conservative (Proven Safe):**
```cpp
ALL messages: 100ms
```

**Moderate (If 40ms Works):**
```cpp
Encoder_Estimates: 40ms
Get_Iq: 40ms
Others (Heartbeat, Errors, Voltage): 100ms
```

**Aggressive (If 30ms Works):**
```cpp
Encoder_Estimates: 30ms
Get_Iq: 30ms
Others: 100ms
```

**Target (Requires 20ms Validation):**
```cpp
Encoder_Estimates: 20ms
Get_Iq: 20ms  
Others: 100ms
```

### Decision Tree
```
Deploy 100ms → Test → Measure injection behavior
    ↓
40ms test → If safe, upgrade critical messages
    ↓
30ms test → If safe, upgrade critical messages
    ↓
20ms test → If injection occurs, revert to 30ms or 40ms
    ↓
Final production rate selected
```

---

## PERFORMANCE TARGETS

### Loop Time
- **Current:** 2-3ms (with blocking)
- **Target:** 0.1-0.3ms (with SafeString/millisDelay)
- **Basis:** Arduino UNO (16MHz) achieves <1ms, ESP32 (240MHz) is 15x faster

### CAN Polling Rate
- **Current:** ~330 polls/sec (blocked by delays)
- **Target:** 5,000+ polls/sec (0.2ms loop)
- **100ms broadcast:** Polled 500 times per cycle
- **20ms broadcast:** Polled 100 times per cycle

### Message Loss
- **Current:** ~4 broadcasts lost during delay(50)
- **Target:** ZERO loss (aggressive FIFO draining)

### Response Detection
- **Window:** 2-5ms after TX command
- **Accuracy:** ±100µs (GPTimer resolution)
- **Success Rate:** 100% (protected windows prevent ambiguity)

---

## MEMORY BUDGET

### BroadcastDataStore Enhanced
```
Ring Buffers (7 types × 10 slots × 16 bytes):  1,120 bytes
TX Command Log (5 slots × 16 bytes):              80 bytes
Metadata (indices, timestamps, config):           50 bytes
                                        Total:  1,250 bytes
```

### SafeString Buffers
```
Serial output buffer:                            512 bytes
Message formatting buffers:                      256 bytes
                                        Total:   768 bytes
```

### millisDelay Instances
```
~15 timers × 12 bytes:                           180 bytes
```

### Total Overhead
```
1,250 + 768 + 180 = 2,198 bytes (0.67% of 320KB RAM)
```

**Verdict:** ✅ Negligible memory cost

---

## OPEN QUESTIONS / TODO

### Testing Required
- [ ] Verify TWAI RX FIFO depth (8 or 16 slots?)
- [ ] Test protected window boundaries (60-65ms, 95-100ms ranges)
- [ ] Measure 40ms cyclic rate injection behavior
- [ ] Measure 30ms cyclic rate injection behavior  
- [ ] Measure 20ms cyclic rate injection behavior
- [ ] Confirm GPTimer accessible from Arduino Framework
- [ ] Validate SafeString BufferedOutput pattern with ESP32
- [ ] Measure actual loop time with SafeString/millisDelay (target <0.3ms)

### Design Decisions Pending
- [ ] RTRDebug integration with BroadcastDataStore (unified path)
- [ ] Final cyclic rate selection (awaiting testing)
- [ ] Ring buffer size (10 slots adequate? or optimize to 5/15?)
- [ ] TX command log size (5 slots adequate for queue depth?)

### Implementation Phases
- [ ] **Phase 1:** SafeString/millisDelay refactor (eliminate blocking)
- [ ] **Phase 2:** GPTimer integration (replace micros())
- [ ] **Phase 3:** BroadcastDataStore ring buffers (add history)
- [ ] **Phase 4:** Protected window logic (safe TX timing)
- [ ] **Phase 5:** Response correlation (TX→RX pairing)
- [ ] **Phase 6:** Cyclic rate optimization (testing-driven)

---

## NEXT ANALYSIS POINTS

✅ **ALL POINTS COMPLETE (1-9)**

- **Point 4:** ✅ Loop optimization tricks rejected (red herring)
- **Point 5:** ✅ millisDelay is sufficient
- **Point 6:** ✅ FreeRTOS present but impact minimal (passive awareness)
- **Point 7:** ✅ If statements mandatory, while loops bounded only
- **Point 8:** ✅ No ISRs needed (GPTimer counter only, CAN polling wins)
- **Point 9:** ✅ Dual-core deferred (9B = backup if polling insufficient)

**NEXT: Final synthesis & implementation roadmap**

---

## REFERENCES

### SafeString Library
- Main docs: https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/docs/html/index.html
- Tutorial: https://www.instructables.com/Simple-Multi-tasking-in-Arduino-on-Any-Board/
- Key insight: BufferedOutput + millisDelay enable true non-blocking Arduino

### ESP32 Hardware
- GPTimer docs: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gptimer.html
- TWAI controller: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html

### ODrive CAN Protocol
- CAN Protocol: https://docs.odriverobotics.com/v/0.5.6/can-protocol.html
- State Machine: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState

---

**Document Version:** 2.0  
**Last Updated:** January 15, 2026  
**Status:** Points 1-9 Complete - Ready for Final Synthesis & Implementation
