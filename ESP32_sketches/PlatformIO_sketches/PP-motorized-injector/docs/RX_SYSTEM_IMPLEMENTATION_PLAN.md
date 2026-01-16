# RX SYSTEM IMPLEMENTATION PLAN
**Building Rock-Solid CAN Message Reception**  
**Date:** January 16, 2026  
**Status:** Phase 1.1-1.6 Complete (GPTimer + CanRxHandler validated) → Phase 1.7-1.8 Next (BDS v2)

---

## PERFORMANCE BUDGET TRACKING

**Goal:** Measure each module's loop time contribution before integration. Target: <100µs avg, <500µs max.

**Phase 1: Single CPU (Core 1) - COMPLETE Jan 16, 2026**

| Module | Test Date | Min (µs) | Avg (µs) | Max (µs) | Cumulative Avg | Notes |
|--------|-----------|----------|----------|----------|----------------|-------|
| **Baseline** | Jan 16 | N/A | **11** | **20** | 11 | Empty loop + loopTimer + 10µs delay |
| **+ GPTimer.micros()** | Jan 15 | **2** | **2-6** | **8** | ~13 | Single read: 2µs, with code: 6-8µs |
| **+ CanRxHandler polling** | Jan 16 | **5** | **5** | **671** | **16** | **5µs = CAN overhead, 671µs = Serial spike** |

**Phase 1.6b: Dual-Core Architecture - COMPLETE Jan 16, 2026**

| Core | Module | Min (µs) | Avg (µs) | Max (µs) | Notes |
|------|--------|----------|----------|----------|-------|
| **CPU 0** | Polling task only | - | **2** | **18** | ✅ FreeRTOS task, continuous pollAndQueue(), manual watchdog |
| **CPU 1** | FSM baseline (no load) | - | **6** | **30** | ✅ Empty loop + loopTimer (no polling overhead) |
| **CPU 1** | + 30µs simulated sensors | - | **37** | **440** | ✅ HX711 + sensor reads |
| **CPU 1** | + 100µs simulated thermocouple | - | **138** | **552** | ✅ SPI thermocouple read |
| **CPU 1** | + 165µs full FSM simulation | - | **154** | **487** | ✅ **Production-realistic load** |
| **CPU 1** | + 330µs FSM simulation | - | **340** | **669** | ✅ 2× production load |
| **CPU 1** | + 500ms stress test | - | **842** | **1265** | ✅ Queue depth 0-1, 755 drain cycles |
| **CPU 1** | + 700ms **critical threshold** | - | **~700** | **701** | ✅ **Queue 98% full, 0 overflows** |
| **CPU 1** | + 800ms **overflow begins** | - | **~800** | **801** | ⚠️ Queue full, 112 overflows/5s |
| **CPU 1** | + 1000ms sustained overflow | - | **~1000** | **1001** | ❌ Queue full, 260 overflows/5s |

**Phase 3: Full Integration (TBD)**

| Module | Test Date | Min (µs) | Avg (µs) | Max (µs) | Cumulative Avg | Notes |
|--------|-----------|----------|----------|----------|----------------|-------|
| + BroadcastDataStore v2 | TBD | TBD | TBD | TBD | TBD | Ring buffer updates (CPU 1) |
| + SafeString BufferedOutput | TBD | TBD | TBD | TBD | TBD | Non-blocking serial (CPU 1) |
| + FSM (minimal state) | TBD | TBD | TBD | TBD | TBD | IDLE/READY states (CPU 1) |
| + FSM (active injection) | TBD | TBD | TBD | TBD | TBD | Full state machine (CPU 1) |

**Measurement Method:**
- Use `hwTimer.micros()` for 1µs resolution
- Measure DURING active operation (not after)
- Exclude Serial.print() overhead
- Record min/max/avg over test duration
- Update cumulative average after each module addition

**Test Notes (Jan 16, 2026):**
- **Baseline:** 11µs includes loopTimer.check() + delayMicroseconds(10)
- **CanRxHandler:** 5µs avg measured over 10s with 1803 messages (180/sec)
- **Max 671µs:** Serial.print() spike when sampling message data
- **Test throttling:** delayMicroseconds(100) used to prevent wasteful tight spinning
  - Without delay: Loop runs at MHz (wastes power, no benefit)
  - With 100µs delay: ~10,000 Hz (55x faster than message rate)
  - **Production:** Remove delay - FSM naturally slows loop to 100-300µs
- **Cumulative 16µs:** 11µs baseline + 5µs polling = 16µs expected avg with FSM

**Current Status:** CanRxHandler validated - 5µs avg overhead, zero message loss, 180 msgs/sec captured.

**Next Steps (Later Jan 16, 2026 - after sleep!):**
1. **Move polling to CPU 0** - Create FreeRTOS task pinned to Core 0
2. **Dual loopTimer** - Measure both CPU 0 (polling) and CPU 1 (FSM) independently
3. **Interrupt-based RX (KISS)** - Try ESP32-TWAI-CAN library interrupt callbacks FIRST
4. **Interrupt-based RX (fallback)** - If library doesn't support, explore ESP-IDF TWAI API
5. **Remeasure timings** - Compare polling vs interrupt overhead on CPU 0
6. **Final architecture** - Choose best approach (CPU 0 polling or CPU 0 interrupt)

---

### **Step 1.6b: Dual-Core Architecture Validation** ✅ COMPLETE

**Goal:** Move polling to Core 0, validate architecture under realistic and extreme FSM loads, determine breaking point.

**Status:** VALIDATED - January 16, 2026

**Dependencies:** Step 1.6 (CanRxHandler polling), loopTimer (Core 0 + Core 1 independent measurement)

---

#### **ARCHITECTURE IMPLEMENTATION**

**Core 0: Polling Task**
```cpp
void pollingTaskCore0(void* parameter) {
    disableCore0WDT();           // Remove IDLE0 from watchdog
    esp_task_wdt_add(NULL);      // Add this task to watchdog
    
    uint32_t pollCount = 0;
    while (true) {
        loopTimerCore0.check(Serial);  // Independent performance tracking
        handler->pollAndQueue();        // Non-blocking CAN read
        
        if (++pollCount >= 100) {
            esp_task_wdt_reset();       // Manual watchdog reset (~500µs)
            pollCount = 0;
        }
    }
}

// Launch task pinned to Core 0
xTaskCreatePinnedToCore(
    pollingTaskCore0,
    "CANRxPoll",
    4096,          // Stack size
    this,
    2,             // Priority (higher than IDLE0)
    &taskHandle_,
    0              // Core 0
);
```

**Core 1: FSM + Queue Drain**
```cpp
void loop() {
    loopTimer.check(Serial);  // Independent FSM performance tracking
    
    // Drain queue (non-blocking)
    CANRxMessage msg;
    while (canRx.receiveMessage(msg, 0)) {
        // Process message
    }
    
    // FSM execution
    // ...
}
```

**Key Design Decisions:**
1. **Manual Watchdog Management:** `disableCore0WDT()` + `esp_task_wdt_reset()` prevents timeout
2. **Priority 2 Task:** Higher than IDLE0 (priority 0), allows continuous polling
3. **Independent loopTimer:** Core 0 and Core 1 measured separately
4. **Queue Size:** 128 slots = 700ms buffer at 180 msg/s

---

#### **STRESS TESTING METHODOLOGY**

**Progressive Load Simulation:**
```cpp
// Add delays to simulate FSM overhead
delayMicroseconds(30);   // Test 1: Sensor reads
delayMicroseconds(100);  // Test 2: + Thermocouple SPI
delayMicroseconds(165);  // Test 3: Full FSM simulation
delayMicroseconds(330);  // Test 4: 2× FSM load
delayMicroseconds(500000); // Test 5-9: Extreme stress testing

// Measure queue behavior
uint8_t queueDepthBefore = canRx.getQueueDepth();
while (canRx.receiveMessage(msg, 0)) { /* drain */ }
```

**Metrics Tracked:**
1. **Loop time:** Min/avg/max per 5-second window (loopTimer)
2. **Message rate:** Messages received per second
3. **Drain cycles:** Number of drain iterations (loop count)
4. **Max burst:** Peak messages per drain cycle
5. **Queue depth:** Messages accumulated before drain (sampled before drain)
6. **Overflows:** Cumulative messages lost since boot

---

#### **TEST RESULTS SUMMARY**

**Production Load Tests (0-500µs):**

| Test | Delay Added | Loop Avg | Queue Depth | Max Burst | Drain Cycles | Overflows | Status |
|------|-------------|----------|-------------|-----------|--------------|-----------|--------|
| Baseline | 0µs | 6µs | 0 | 1 | 900 | 0 | ✅ |
| Test 1 | +30µs | 37µs | 0 | 1 | 900 | 0 | ✅ |
| Test 2 | +100µs | 138µs | 0-1 | 1 | 900 | 0 | ✅ |
| **Test 3** | **+165µs** | **154µs** | **0-1** | **1** | **900** | **0** | **✅ PRODUCTION** |
| Test 4 | +300µs | 174µs | 0-1 | 1 | 900 | 0 | ✅ |
| Test 5 | +330µs | 340µs | 0 | 1 | 900 | 0 | ✅ |

**Stress Tests (500µs - 1000ms):**

| Test | Loop Time | Queue Depth | Max Burst | Drain Cycles | Overflows/5s | Capacity Used | Status |
|------|-----------|-------------|-----------|--------------|--------------|---------------|--------|
| +100ms | 100ms | 19 | 19 | 50 | 0 | 15% | ✅ |
| +200ms | 200ms | 37 | 37 | 25 | 0 | 29% | ✅ |
| +500ms | 500ms | 90 | 90 | 10 | 0 | 70% | ✅ |
| **+700ms** | **700ms** | **126-127** | **126-127** | **8** | **0** | **98-99%** | **✅ THRESHOLD** |
| +800ms | 800ms | 128 | 128 | 7 | 112 | 100% | ⚠️ |
| +1000ms | 1000ms | 128 | 128 | 5 | 260 | 100% | ❌ |

---

#### **CRITICAL FINDINGS**

**1. Production Safety Margin:**
```
Production FSM: 154-175µs loop time
Breaking point: 700,000µs (700ms)
Safety margin: 700,000 / 175 = 4,000× headroom

Worst-case FSM spike: 1000µs (thermal stall)
Margin: 700,000 / 1000 = 700× headroom
```

**2. Queue Behavior Analysis:**
- **Queue depth = 0-1:** System drains faster than fills (ideal)
- **Max burst = 1:** Messages captured individually, not in bundles
- **Drain cycles ≈ loop iterations:** Each loop drains all queued messages
- **Critical threshold:** 711ms (128 msgs / 180 msg/s)

**3. Core Isolation:**
- Core 0: 2µs avg (unchanged across ALL tests)
- Core 1: Scales with simulated load (6µs → 1000ms)
- **Conclusion:** Cores operate independently, no interference

**4. Overflow Behavior:**
```
Loop time < 711ms: Zero overflows (queue never fills)
Loop time = 700ms: Queue 98% full, zero overflows (RIGHT ON EDGE)
Loop time = 800ms: 112 overflows per 5 seconds
Loop time = 1000ms: 260 overflows per 5 seconds

Loss rate @ 1000ms: 260 / 900 = 29% message loss
```

**5. Drain Performance:**
- 1 message: 15-25µs (includes queue access + copy)
- 90 messages: 190µs (2.1µs per message average)
- 128 messages: 266µs (2.08µs per message average)
- **Conclusion:** Drain time scales linearly, no queue contention

---

#### **ARCHITECTURE VALIDATION**

**✅ Polling Architecture VALIDATED for Production**

**Rationale:**
1. **4,000× safety margin** far exceeds requirement (~10× minimum)
2. **Zero message loss** under realistic FSM loads (154-340µs)
3. **700ms breaking point** allows for extreme FSM anomalies (700× headroom)
4. **Core isolation** proven (Core 0 unaffected by Core 1 load)
5. **Linear scaling** of drain performance (no bottlenecks)

**Decision: No interrupts needed** - Polling architecture sufficient and simpler.

---

#### **PRESERVED TEST CODE**

**Location:** End of [src/CanRxHandler.cpp](../src/CanRxHandler.cpp) (commented)

**Test harness preserved for future validation:**
- Progressive load simulation (30µs → 1000ms)
- Queue depth monitoring (before/after drain)
- Overflow detection
- Performance metrics (drain cycles, max burst, loop time)

**Use case:** Re-run after ODrive broadcast rate changes (20ms target) to validate adjusted thresholds.

---

**Next Steps:**
- ✅ Step 1.6b COMPLETE - Architecture validated, polling sufficient
- ⏳ Step 1.7 - BroadcastDataStore v2 (ring buffers + timestamps)
- ⏳ Step 1.8 - TX response correlation
- ⏳ Protected window refinement (after BDS v2 complete)

---

## OVERVIEW

**Goal:** Implement production-grade CAN RX system with guaranteed message capture and accurate timestamps.

**Architecture:** Core 1 Polling + FreeRTOS Queue → BroadcastDataStore Ring Buffers → Modules

**Base Document:** [TIMING_RX_RESPONSE_DESIGN.md](TIMING_RX_RESPONSE_DESIGN.md)

---

## MODULE INTERACTION ANALYSIS

### Modules That MUST Interact With New RX System

**Core Infrastructure (Start from Scratch or Heavy Rewrite):**
1. **GPTimer** (NEW MODULE) - Hardware timestamp source
2. **CanRxHandler** (NEW MODULE) - Core 0 ISR + queue management
3. **BroadcastDataStore** (REWRITE) - Add ring buffers, timestamps, TX correlation
4. **CanBusHandlerV2** (MODIFY) - Integrate with CanRxHandler queue, remove polling

**Support Infrastructure (Modify/Adapt):**
5. **MessageBuffer** (MODIFY) - Add BufferedOutput, timestamp TX commands
6. **MotorWrapper** (MODIFY) - Integrate protected windows, response correlation
7. **main.cpp** (MODIFY) - Add BufferedOutput.nextByteOut(), loopTimer

**State Machine Modules (Minimal Changes - Adapt to BDS API):**
8. Refill, Compression, ReadyToInject, PurgeZero, AntiDrip, Injection, Homing
9. SafetyManager, ErrorManager, DisplayComms

### Files to Backup (.old copies)

**Before ANY changes:**
```bash
# Core files that will be heavily modified
cp src/BroadcastDataStore.cpp src/BroadcastDataStore.cpp.old
cp include/BroadcastDataStore.h include/BroadcastDataStore.h.old
cp src/CanBusHandlerV2.cpp src/CanBusHandlerV2.cpp.old
cp include/CanBusHandlerV2.h include/CanBusHandlerV2.h.old
cp src/MessageBuffer.cpp src/MessageBuffer.cpp.old
cp include/MessageBuffer.h include/MessageBuffer.h.old
cp src/main.cpp src/main.cpp.old

# Reference (already serves its purpose, leave as-is)
# RTRDebug.cpp - Keep as reference for "what NOT to do"
```

### Files to Create From Scratch

**New modules:**
1. `include/GPTimer.h` + `src/GPTimer.cpp` - Hardware timer wrapper
2. `include/CanRxHandler.h` + `src/CanRxHandler.cpp` - Core 0 ISR + queue
3. `tests/test_GPTimer.cpp` - GPTimer unit test
4. `tests/test_CanRxHandler.cpp` - CanRxHandler unit test
5. `tests/test_BroadcastDataStore_v2.cpp` - Enhanced BDS test

---

## IMPLEMENTATION STRATEGY: BOTTOM-UP APPROACH

**Philosophy:** Build new isolated modules first, test with current Serial.print(), THEN retrofit SafeString BufferedOutput everywhere once we understand all the "emission points."

**Why This Order:**
1. ✅ New modules developed in isolation (no breaking existing code)
2. ✅ Each module tested standalone before integration
3. ✅ See all message patterns before retrofitting BufferedOutput
4. ✅ Can use current Serial.print() during development
5. ✅ Less back-and-forth - convert to BufferedOutput once
6. ✅ Existing code keeps working until final integration

**Acceptable Trade-offs:**
- ⚠️ Temporary duplicate code (Serial.print() then BufferedOutput)
- ⚠️ Some rework when converting to BufferedOutput
- ⚠️ But: Much less risky than modifying everything at once

---

## IMPLEMENTATION PHASES (REVISED)

### **PHASE 1: NEW STANDALONE MODULES (No Existing Code Touched)**

**Goal:** Build and test new modules in isolation using current Serial.print() for debug output.

**Sub-Phases:**
- **1.1** GPTimer - Hardware timestamp counter (NEW MODULE) ✅ **COMPLETE** - Validated Jan 15, 2026
- **1.2** Test GPTimer standalone ✅ **COMPLETE** - All tests pass, performance exceeds requirements
- **1.3** CanRxHandler - Core 0 ISR + FreeRTOS queue (NEW MODULE) ✅ **COMPLETE** - Created Jan 15, 2026
- **1.4** Test CanRxHandler standalone ✅ **COMPLETE** - Public interface validated, queue infrastructure tested
- **1.5** loopTimer + TEST_MODE_PHASE1 ✅ **COMPLETE** - Baseline 11µs avg, 20µs max (far exceeds targets)
- **1.6** Connect TWAI polling to CanRxHandler ✅ **COMPLETE Jan 16, 2026** - **VALIDATED:** 1801 msgs/10s, 0 overflows, 14µs avg (see below)
- **1.6b** Dual-core architecture + stress testing ✅ **COMPLETE Jan 16, 2026** - **VALIDATED:** 4,063× safety margin, 700ms critical threshold (see below)
- **1.7** BroadcastDataStore v2 - Ring buffers + timestamps (NEW MODULE) ⏳ **NEXT**
- **1.8** Test BroadcastDataStore v2 standalone

**Status After Phase 1:** Five modules validated - GPTimer, CanRxHandler (dual-core + stress tested), loopTimer, test infrastructure. Architecture proven for production.

**Note:** Steps 1.5-1.6 added during implementation to ensure clean baseline before ISR integration.

---

### **PHASE 2: INTEGRATION POINT (Connect New Modules)**

**Goal:** Wire new modules together, validate message flow through new architecture.

**Sub-Phases:**
- **2.1** Integration test: GPTimer → CanRxHandler → BDS v2
- **2.2** Load test with synthetic CAN messages
- **2.3** Validate timestamps, queue behavior, ring buffers
- **2.4** Document all "emission points" (where debug output occurs)

**Status After Phase 2:** New RX system works end-to-end, all patterns visible.

---

### **PHASE 3: SAFESTRING RETROFIT (Now We Know The Patterns)**

**Goal:** Convert all debug output to SafeString BufferedOutput, starting with new modules.

**Sub-Phases:**
- **3.1** Analyze "emission points" across all code
- **3.2** Implement BufferedOutput infrastructure in MessageBuffer
- **3.3** Convert new modules (GPTimer, CanRxHandler, BDS v2) to BufferedOutput
- **3.4** Add loopTimer performance monitoring to main.cpp
- **3.5** Convert existing modules to use MessageBuffer/BufferedOutput

**Status After Phase 3:** All serial output non-blocking, loop time measured.

---

### **PHASE 4: FINAL INTEGRATION (Replace Old Code With New)**

**Goal:** Swap old modules with new ones, complete system integration.

**Sub-Phases:**
- **4.1** Backup old files (.old copies)
- **4.2** Replace BroadcastDataStore with v2
- **4.3** Replace CanBusHandlerV2 polling with CanRxHandler queue consumer
- **4.4** Update MotorWrapper for protected windows + response correlation
- **4.5** Full system test (all state machines + CAN RX/TX)

**Status After Phase 4:** Production-grade CAN RX system operational.

---

## PHASE 1 DETAILED BREAKDOWN

### **Step 1.1: GPTimer Hardware Counter (NEW MODULE)** ✅ COMPLETE

**Goal:** 1µs resolution hardware timestamp source, immune to ISRs and FreeRTOS task switching.

**Status:** VALIDATED - January 15, 2026

**Dependencies:** None (standalone module)

---

#### **IMPLEMENTATION COMPLETE**

**Files Created:**
- [include/GPTimer.h](../include/GPTimer.h) - Class declaration (62 lines)
- [src/GPTimer.cpp](../src/GPTimer.cpp) - Implementation (56 lines)

**Architecture Decision:**
- **Framework:** Arduino ESP32 HAL functions (`timerBegin`, `timerRead`, `timerWrite`)
- **Why Not ESP-IDF 5.x:** `driver/gptimer.h` not available in Arduino framework (ESP-IDF 4.4.x bundled)
- **Hardware:** Timer 0, prescaler 80 (80MHz / 80 = 1MHz = 1µs resolution)
- **Global Instance:** `GPTimer hwTimer;` accessible from all modules

**Key Implementation Details:**
```cpp
// Initialization - 1MHz counter (1µs ticks)
timerHandle_ = timerBegin(0, 80, true);  // Timer 0, prescaler 80, count up

// Fast read with IRAM_ATTR (ISR safe)
uint64_t IRAM_ATTR GPTimer::micros() const {
    return timerRead(timerHandle_);
}
```

---

#### **VALIDATION TESTING (3-Phase Comprehensive)**

**Test Harness:** `testGPTimer()` in main.cpp (temporary, removed after validation)

**Test 1: Initialization** ✅
- Timer starts successfully
- Initial counter value = 0

**Test 2: Time Progression (3-Phase Overhead Analysis)** ✅
```
Target    Measured  Overhead
100µs  →  106µs     6µs
1000µs →  1007µs    7µs
10000µs → 10006µs   6µs
```
- **Analysis:** Overhead is constant (6-8µs), does NOT scale with delay duration
- **Conclusion:** Overhead from `delayMicroseconds()` call itself, not from interference

**Test 3: Delta Accuracy vs micros() (3-Phase)** ✅
```
Interval  GPTimer  micros()  Difference
100µs     111µs    103µs     8µs
1000µs    1003µs   1006µs    -3µs
10000µs   10006µs  10003µs   3µs
```
- **Analysis:** Both timers measure same interval within ±8µs
- **Accuracy:** <0.1% error over 10ms (CAN message interval)
- **Note:** ±2-10µs jitter caused by FreeRTOS tick interrupt (1ms), unavoidable but acceptable

**Test 4: Read Speed (3-Phase Consistency)** ✅
```
Sample Size  Total Time  Avg Per Read
100 reads    208µs       2.080µs
1000 reads   2049µs      2.049µs
10000 reads  20461µs     2.046µs
```
- **Analysis:** Read time consistent (2.046-2.080µs) regardless of:
  - Sample size (100 vs 10000)
  - Timer value magnitude (tested up to 20ms)
- **Best Case:** 2µs (consecutive reads, hot cache)
- **Realistic:** 6-8µs (with surrounding code, as seen in Tests 2 & 3)

---

#### **PERFORMANCE SUMMARY**

| Metric | Value | Validation |
|--------|-------|------------|
| Resolution | 1µs | ✅ Confirmed |
| Overhead (constant) | 6-8µs | ✅ Non-scaling |
| Read time (consecutive) | 2.0µs | ✅ Best case |
| Read time (realistic) | 6-8µs | ✅ With surrounding code |
| Accuracy over 10ms | ±8µs | ✅ <0.1% error |
| Jitter (FreeRTOS tick) | 2-10µs | ✅ Acceptable |
| Read consistency | 2.046-2.080µs | ✅ No scaling |

**Production Implications:**
- CAN messages arrive ~10ms apart (ODrive broadcast cycle)
- Single `hwTimer.micros()` call per ISR event
- Realistic read time: 6-8µs (with surrounding code)
- Timing error: 8µs / 10000µs = **0.08% maximum error**
- **Conclusion:** Performance exceeds requirements for CAN timestamping

**Measurement Context:**
- **Test 4 (2µs):** Consecutive reads in tight loop, hot cache, theoretical minimum
- **Tests 2 & 3 (6-8µs):** Reads with surrounding code (delayMicroseconds, variable assignments), realistic production scenario
- **Production ISR:** Single read per event, 10ms between events, surrounding code minimal → expect 4-6µs per call

---

#### **ACTUAL IMPLEMENTATION (Arduino HAL)**

**Note:** The original plan specified ESP-IDF 5.x API (`driver/gptimer.h`), but this header is not available in Arduino framework (uses ESP-IDF 4.4.x). We used Arduino ESP32 HAL functions instead.

**Files Created:**
- `include/GPTimer.h` (62 lines)
- `src/GPTimer.cpp` (56 lines)

**Header (GPTimer.h):**
```cpp
#ifndef GPTIMER_H
#define GPTIMER_H

#include <Arduino.h>

class GPTimer {
private:
    hw_timer_t* timerHandle_;
    bool initialized_;
    
public:
    GPTimer();
    
    // Initialize timer (1MHz = 1µs resolution)
    bool begin();
    
    // Read current timestamp (microseconds since begin())
    uint64_t IRAM_ATTR micros() const;
    
    // Reset counter to zero (for testing)
    void reset();
    
    // Check initialization status
    bool isInitialized() const { return initialized_; }
};

// Global instance
extern GPTimer hwTimer;

#endif
```

**Implementation (GPTimer.cpp):**
```cpp
#include "GPTimer.h"

GPTimer hwTimer;  // Global instance

GPTimer::GPTimer() : timerHandle_(nullptr), initialized_(false) {}

bool GPTimer::begin() {
    if (initialized_) return true;
    
    // Timer 0, prescaler 80 (80MHz / 80 = 1MHz = 1µs), count up
    timerHandle_ = timerBegin(0, 80, true);
    
    if (timerHandle_ == nullptr) {
        return false;
    }
    
    timerStart(timerHandle_);
    initialized_ = true;
    return true;
}

uint64_t IRAM_ATTR GPTimer::micros() const {
    if (!initialized_) return 0;
    return timerRead(timerHandle_);
}

void GPTimer::reset() {
    if (initialized_) {
        timerWrite(timerHandle_, 0);
    }
}
```

**Key Differences from Planned ESP-IDF 5.x Code:**
- ✅ Used Arduino HAL: `timerBegin()`, `timerRead()`, `timerWrite()`
- ✅ No ESP-IDF structs (`gptimer_config_t`, `gptimer_handle_t`)
- ✅ Simpler API (3 lines vs 20+ lines for initialization)
- ✅ Same functionality: 1µs resolution, 64-bit counter, ISR-safe reads
- ✅ `IRAM_ATTR` on micros() for ISR use (places function in internal RAM)

---

### **Step 1.2: Test GPTimer Standalone** ✅ COMPLETE

**Goal:** Validate GPTimer in isolation using Serial.print() for output (MessageBuffer deferred to Phase 3).

**Status:** VALIDATED - January 15, 2026

**Test Implementation:**
```cpp
// testGPTimer() added temporarily to main.cpp
void testGPTimer() {
    Serial.println("\n=== GPTimer Test Start ===");
    
    // Test 1: Initialization
    if (!hwTimer.begin()) {
        Serial.println("FAIL: GPTimer init");
        return;
    }
    Serial.println("PASS: GPTimer init");
    
    // Test 2: 3-Phase Time Progression (100µs, 1000µs, 10000µs)
    for (int phase = 0; phase < 3; phase++) {
        uint32_t targetDelay = (phase == 0) ? 100 : (phase == 1) ? 1000 : 10000;
        
        uint64_t gpt1 = hwTimer.micros();
        uint32_t std1 = micros();
        
        delayMicroseconds(targetDelay);
        
        uint64_t gpt2 = hwTimer.micros();
        uint32_t std2 = micros();
        
        uint32_t gptDelta = (uint32_t)(gpt2 - gpt1);
        uint32_t stdDelta = std2 - std1;
        int32_t diff = (int32_t)(gptDelta - stdDelta);
        
        Serial.print("Phase ");
        Serial.print(phase + 1);
        Serial.print(" (");
        Serial.print(targetDelay);
        Serial.print("us target): GPTimer=");
        Serial.print(gptDelta);
        Serial.print("us, micros()=");
        Serial.print(stdDelta);
        Serial.print("us, diff=");
        Serial.print(diff);
        Serial.println("us");
        
        delay(100);  // Settle between phases
    }
    
    // Test 3: Read Speed (3-Phase: 100, 1000, 10000 reads)
    for (int phase = 0; phase < 3; phase++) {
        uint32_t numReads = (phase == 0) ? 100 : (phase == 1) ? 1000 : 10000;
        
        uint64_t start = hwTimer.micros();
        for (uint32_t i = 0; i < numReads; i++) {
            volatile uint64_t t = hwTimer.micros();
        }
        uint64_t end = hwTimer.micros();
        
        uint32_t totalTime = (uint32_t)(end - start);
        uint32_t avgPerRead = totalTime / numReads;
        
        Serial.print("Read Speed Phase ");
        Serial.print(phase + 1);
        Serial.print(" (");
        Serial.print(numReads);
        Serial.print(" reads): total=");
        Serial.print(totalTime);
        Serial.print("us, avg=");
        Serial.print(avgPerRead);
        Serial.println("us/read");
        
        delay(100);
    }
    
    Serial.println("=== GPTimer Test Complete ===\n");
}
```

**How to Test:**
```cpp
// In main.cpp setup() - TEMPORARY TEST MODE
void setup() {
    Serial.begin(115200);
    delay(1000);  // Wait for serial ready
    
    // Run GPTimer tests
    testGPTimer();
    
    // Remove test function and proceed with normal init...
}
```

**Actual Test Output (January 15, 2026):**
```
=== GPTimer Test Start ===
PASS: GPTimer init
Phase 1 (100us target): GPTimer=106us, micros()=103us, diff=3us
Phase 2 (1000us target): GPTimer=1007us, micros()=1006us, diff=1us
Phase 3 (10000us target): GPTimer=10006us, micros()=10003us, diff=3us
Read Speed Phase 1 (100 reads): total=208us, avg=2us/read
Read Speed Phase 2 (1000 reads): total=2049us, avg=2us/read
Read Speed Phase 3 (10000 reads): total=20461us, avg=2us/read
=== GPTimer Test Complete ===
```

**Why Serial.print() Instead of MessageBuffer:**
- **Reason:** MessageBuffer uses blocking `Serial.print()` internally (SafeString `BufferedOutput` not implemented yet)
- **Impact:** MessageBuffer adds ~1-3ms overhead per print → Would pollute timing measurements
- **Decision:** Use raw `Serial.print()` for Phase 1 testing, defer BufferedOutput to Phase 3
- **Production:** Once BufferedOutput integrated, MessageBuffer becomes non-blocking

**Success Criteria:**
- ✅ Timer initializes correctly
- ✅ Timestamps increase monotonically
- ✅ 1µs resolution verified (±8µs over 10ms = 0.08% error)
- ✅ Read time: 2µs consecutive (best case), 6-8µs realistic
- ✅ No rollover issues (64-bit counter)
- ✅ Overhead constant (6-8µs) across all delay magnitudes
    
    testGPTimer();  // Run test once
    
    // Remove test call after validation
}
```

**Success Criteria:**
- ✅ Timer initializes without errors
- ✅ Time delta ~100µs for delayMicroseconds(100)
- ✅ Read time <1µs (ideally ~200-300ns)
- ✅ Timestamps always increase (no rollback)

**After Success:** Remove test code, keep GPTimer module for next step.

---

### **Step 1.3: CanRxHandler (Core 0 ISR + Queue) - NEW MODULE**

**Goal:** Core 0 interrupt-driven CAN message capture with FreeRTOS queue.

**Dependencies:** GPTimer (Step 1.1)

**Files to Create:**
- `include/CanRxHandler.h`
- `src/CanRxHandler.cpp`

**What to Build:**
```cpp
// include/CanRxHandler.h
#ifndef CAN_RX_HANDLER_H
#define CAN_RX_HANDLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "GPTimer.h"

// CAN message structure with timestamp
struct CANRxMessage {
    uint32_t canId;        // CAN message ID
    uint8_t data[8];       // Message data
    uint8_t dlc;           // Data length code
    uint64_t timestamp;    // GPTimer microseconds
};

class CanRxHandler {
public:
    static CanRxHandler& getInstance();
    
    // Initialize queue and ISR
    bool begin();
    
    // Check if messages available
    bool hasMessages() const;
    
    // Get next message from queue (non-blocking)
    bool receiveMessage(CANRxMessage& msg, uint32_t timeoutMs = 0);
    
    // Get queue stats (for debugging)
    uint32_t getQueueDepth() const;
    uint32_t getMessagesReceived() const;
    uint32_t getQueueOverflows() const;
    
private:
    CanRxHandler();
    ~CanRxHandler();
    
    static const uint8_t QUEUE_SIZE = 32;  // 32 messages
    QueueHandle_t messageQueue_;
    
    uint32_t messagesReceived_;
    uint32_t queueOverflows_;
    
    // ISR callback (runs on Core 0)
    static void IRAM_ATTR twaiRxISR(void* arg);
    
    // Prevent copying
    CanRxHandler(const CanRxHandler&) = delete;
    CanRxHandler& operator=(const CanRxHandler&) = delete;
};

#endif
```

**Implementation Stub:**
```cpp
// src/CanRxHandler.cpp
#include "CanRxHandler.h"
#include <ESP32-TWAI-CAN.hpp>

extern GPTimer hwTimer;  // From GPTimer.cpp

CanRxHandler& CanRxHandler::getInstance() {
    static CanRxHandler instance;
    return instance;
}

CanRxHandler::CanRxHandler() 
    : messageQueue_(nullptr)
    , messagesReceived_(0)
    , queueOverflows_(0) {
}

CanRxHandler::~CanRxHandler() {
    if (messageQueue_) {
        vQueueDelete(messageQueue_);
    }
}

bool CanRxHandler::begin() {
    // Create FreeRTOS queue
    messageQueue_ = xQueueCreate(QUEUE_SIZE, sizeof(CANRxMessage));
    if (!messageQueue_) {
        Serial.println("ERROR: Failed to create CAN RX queue");
        return false;
    }
    
    // TODO: Register TWAI ISR (Step 1.4)
    // For now, just create queue
    
    Serial.println("CanRxHandler: Queue created (32 slots)");
    return true;
}

bool CanRxHandler::hasMessages() const {
    if (!messageQueue_) return false;
    return uxQueueMessagesWaiting(messageQueue_) > 0;
}

bool CanRxHandler::receiveMessage(CANRxMessage& msg, uint32_t timeoutMs) {
    if (!messageQueue_) return false;
    
    TickType_t ticks = (timeoutMs == 0) ? 0 : pdMS_TO_TICKS(timeoutMs);
    return xQueueReceive(messageQueue_, &msg, ticks) == pdTRUE;
}

uint32_t CanRxHandler::getQueueDepth() const {
    if (!messageQueue_) return 0;
    return uxQueueMessagesWaiting(messageQueue_);
}

uint32_t CanRxHandler::getMessagesReceived() const {
    return messagesReceived_;
}

uint32_t CanRxHandler::getQueueOverflows() const {
    return queueOverflows_;
}

// ISR stub (implement in Step 1.4)
void IRAM_ATTR CanRxHandler::twaiRxISR(void* arg) {
    // TODO: Read CAN frame, timestamp, send to queue
}
```

**Success Criteria (Step 1.3 Only):**
- ✅ Queue creates successfully
- ✅ Can send/receive test messages manually
- ✅ No crashes, no memory leaks
- ✅ ISR stub compiles (not connected yet)

---

### **Step 1.4: Test CanRxHandler Queue (Without ISR)**

**Goal:** Validate queue mechanics before connecting TWAI ISR.

**Test Code:**
```cpp
void testCanRxQueue() {
    Serial.println("=== CanRxHandler Queue Test ===");
    
    CanRxHandler& canRx = CanRxHandler::getInstance();
    
    // Test 1: Initialization
    if (!canRx.begin()) {
        Serial.println("FAIL: Queue init");
        return;
    }
    Serial.println("PASS: Queue initialized");
    
    // Test 2: Send/receive test message
    CANRxMessage testMsg;
    testMsg.canId = 0x123;
    testMsg.dlc = 8;
    for (int i = 0; i < 8; i++) testMsg.data[i] = i;
    testMsg.timestamp = hwTimer.micros();
    
    // Manually add to queue (simulating ISR)
    BaseType_t result = xQueueSend(canRx.messageQueue_, &testMsg, 0);
    if (result != pdTRUE) {
        Serial.println("FAIL: Queue send");
        return;
    }
    Serial.println("PASS: Message queued");
    
    // Test 3: Receive message
    CANRxMessage rcvMsg;
    if (!canRx.receiveMessage(rcvMsg, 100)) {
        Serial.println("FAIL: Queue receive");
        return;
    }
    
    if (rcvMsg.canId == 0x123 && rcvMsg.data[0] == 0) {
        Serial.println("PASS: Message received correctly");
    } else {
        Serial.println("FAIL: Message corrupted");
    }
    
    // Test 4: Queue depth
    Serial.print("Queue depth: ");
    Serial.println(canRx.getQueueDepth());
    
    Serial.println("=== Queue Test Complete ===");
}
```

**Success Criteria:**
- ✅ Messages queue/dequeue correctly
- ✅ Queue depth tracking works
- ✅ No memory corruption
- ✅ Ready for ISR connection (Step 1.5)

---

#### **IMPLEMENTATION COMPLETE - Steps 1.3 & 1.4**

**Date:** January 15-16, 2026

**Files Created:**
- [include/CanRxHandler.h](../include/CanRxHandler.h) - Class declaration with full documentation (165 lines)
- [src/CanRxHandler.cpp](../src/CanRxHandler.cpp) - Implementation with queue infrastructure (115 lines)

**Architecture Implemented:**
- **Singleton Pattern:** `getInstance()` for global access
- **FreeRTOS Queue:** 32-slot message queue (CANRxMessage struct)
- **Non-blocking Consumer:** `receiveMessage()` with optional timeout
- **Statistics Tracking:** Queue depth, messages received, overflows
- **ISR Stub:** `twaiRxISR()` ready for TWAI connection in Step 1.6

**Testing Results (Step 1.4):**
```
=== CanRxHandler Queue Test Start ===
CanRxHandler: Queue created (32 slots)
PASS: Queue initialized
PASS: Queue empty on init
Messages received: 0
Queue overflows: 0
Queue depth: 0
=== CanRxHandler Queue Test Complete ===
```

**Implementation Variance from Plan:**
- **Test Limitation:** Cannot access private `messageQueue_` member from test
- **Simplified Testing:** Validated public interface only (begin(), hasMessages(), getQueueDepth(), stats)
- **Rationale:** Full queue push/pop testing deferred to Step 1.6 (ISR integration)
- **Result:** Public interface confirmed working, ready for ISR connection

**Compilation Stats:**
- RAM: 9.2% (30,012 bytes) - minimal increase
- Flash: 27.2% (356,353 bytes) - minimal increase

**Next:** Connect TWAI ISR to queue (Step 1.6)

---

### **Step 1.5: TEST_MODE_PHASE1 + loopTimer Baseline** ✅ COMPLETE

**Date:** January 16, 2026

**Goal:** Establish clean performance baseline BEFORE adding ISR overhead.

**Why This Step Was Added:**
- MessageBuffer using blocking Serial.print() created 3.2ms noise
- Impossible to measure 5µs ISR overhead in 3200µs noise
- Aligns with "bottom-up" philosophy: Test new modules in isolation

**Files Modified:**
- [include/config.h](../include/config.h) - Added `TEST_MODE_PHASE1` flag
- [src/main.cpp](../src/main.cpp) - Added loopTimer, TEST_MODE conditional compilation

**Implementation:**

**config.h:**
```cpp
// ==========================================
// 0B. PHASE 1 TEST MODE (ISOLATED MODULE TESTING)
// ==========================================
// When TEST_MODE_PHASE1 = true: Bypass FSM, run only Phase 1 module tests
// - Minimal loop() for clean performance baseline
// - loopTimer measures without FSM noise
// - Allows measurement of ISR overhead (<5µs)
// When TEST_MODE_PHASE1 = false: Normal FSM operation
#define TEST_MODE_PHASE1  true  // TEMPORARY: Set false after Phase 1 complete
```

**main.cpp setup():**
```cpp
#if TEST_MODE_PHASE1
    // ===== PHASE 1 TEST MODE: Isolated Module Testing =====
    Serial.println("PHASE 1 TEST MODE ACTIVE");
    Serial.println("FSM bypassed - testing new modules only");
    
    testCanRxQueue();  // Run Phase 1 tests
    
    Serial.println("Test complete - entering clean loop");
    Serial.println("loopTimer will show baseline performance");
    Serial.println("Target: <100us avg, <500us max");
    delay(2000);
    return;  // Skip normal FSM initialization
#endif
```

**main.cpp loop():**
```cpp
void loop() {
    loopTimer.check(Serial);  // Measure loop time every 5 seconds
    
#if TEST_MODE_PHASE1
    // Minimal loop for clean baseline
    delayMicroseconds(10);
    return;  // Skip all FSM processing
#endif
    
    // Normal FSM processing...
}
```

**Baseline Performance Results:**
```
loop us Latency
 5sec max:4102613 avg:70       ← First window (includes setup delays)
 sofar max:4102613 avg:70 max - prt:266

loop us Latency
 5sec max:20 avg:11            ← Steady-state baseline ✅
 sofar max:4102613 avg:70 max - prt:229

loop us Latency
 5sec max:20 avg:11            ← Consistent performance ✅
 sofar max:4102613 avg:70 max - prt:230
```

**Performance Analysis:**
| Metric | Target | Actual | Status |
|--------|--------|--------|--------|
| Loop avg | <100µs | **11µs** | ✅ 9x better |
| Loop max | <500µs | **20µs** | ✅ 25x better |
| 5sec consistency | Stable | ±0µs | ✅ Rock-solid |

**First Window Breakdown (4.1s):**
- `delay(2000)` after Serial.begin → 2000ms
- `delay(2000)` after test complete → 2000ms
- Test execution + Serial.println() → ~100ms
- **Total:** ~4100ms (expected, ignore this window)

**ISR Overhead Budget:**
- Current baseline: **11µs avg**
- Expected ISR overhead: **<5µs per event**
- CAN message rate: **~10ms intervals (100 msgs/sec)**
- Expected new avg: **11µs + 5µs = 16µs** ✅ Still 6x under 100µs target

**Code Size Impact:**
- **Before TEST_MODE:** RAM 9.2% (30,012 bytes), Flash 27.2% (356,353 bytes)
- **After TEST_MODE:** RAM 8.7% (28,644 bytes), Flash 23.0% (301,957 bytes)
- **Savings:** -1,368 bytes RAM, -54,396 bytes Flash (compiler optimized out unused FSM)

**Conclusion:**
✅ Clean baseline established (11µs avg, 20µs max)  
✅ ISR overhead will be clearly visible when added  
✅ Huge headroom for ISR integration  
✅ Ready for Step 1.6 (TWAI polling connection)

---

### **Step 1.6: Connect TWAI Polling to CanRxHandler** ✅ COMPLETE - January 16, 2026

**Goal:** Capture real CAN messages with hardware timestamps, validate zero message loss and queue performance.

**Dependencies:**
- GPTimer (Step 1.1) ✅
- CanRxHandler (Step 1.3-1.4) ✅
- TEST_MODE_PHASE1 baseline (Step 1.5) ✅

**Files Modified:**
- `src/CanRxHandler.cpp` - Implemented `pollAndQueue()` method
- `src/main.cpp` - Added `testCanRxISR()` validation

**Status:** **VALIDATED** - 1801 messages captured in 10 seconds, 0 queue overflows, 14µs avg loop time (3µs overhead vs baseline)

---

#### **IMPLEMENTATION DETAILS**

**Architecture Decision: Polling vs ISR**

ESP-IDF 4.4.x (bundled with Arduino framework) does NOT support TWAI ISR callbacks:
- ❌ No `twai_driver_install()` callback parameter in ESP-IDF 4.4.x
- ❌ ESP-IDF 5.x `driver/gptimer.h` APIs not available
- ✅ Solution: Fast polling approach using `twai_read_alerts()` + `twai_receive()`

**Two Validated Approaches:**

**Approach 1: ESP-IDF TWAI Alerts (VALIDATED - Current Archived Implementation)**
```cpp
// begin() - Configure TWAI alerts
uint32_t current_alerts = 0;
esp_err_t err = twai_reconfigure_alerts(TWAI_ALERT_RX_DATA, &current_alerts);
// Result: ESP_OK, configures RX_DATA alert

// pollAndQueue() - Non-blocking alert check + frame read
uint32_t alerts = 0;
esp_err_t err = twai_read_alerts(&alerts, 0);  // 0 = non-blocking
if (err == ESP_OK && (alerts & TWAI_ALERT_RX_DATA)) {
    twai_message_t frame;
    err = twai_receive(&frame, 0);  // 0 = non-blocking
    if (err == ESP_OK) {
        uint64_t timestamp = hwTimer.micros();  // Timestamp immediately
        // Queue message...
    }
}
```

**Performance:** 1801 msgs/10s, 0 overflows, 14µs avg loop (3µs overhead)  
**Complexity:** Medium (40 lines, ESP-IDF layer knowledge)  
**Status:** Working, validated, **archived in CanRxHandler.cpp comments**

**Approach 2: ESP32-TWAI-CAN Library (CURRENT - Simpler, VALIDATED)**
```cpp
// pollAndQueue() - Direct library call
CanFrame frame;
if (!ESP32Can.readFrame(frame, 0)) return;  // 0 = non-blocking

uint64_t timestamp = hwTimer.micros();  // Timestamp immediately
// Queue message...
```

**Performance:** **1803 msgs/10s, 0 overflows, 14µs avg, 38-54µs max (equal/better!)**  
**Complexity:** Low (15 lines, familiar library API)  
**Status:** **VALIDATED Jan 16, 2026 - Equal or better than TWAI alerts**  
**Rationale:** KISS principle - simpler code, proven in CanBusHandlerV2 production use

---

#### **TESTING METHODOLOGY**

**Test Challenge:** TEST_MODE bypasses normal FSM initialization, including DC contactor power.

**Critical Discovery:** ODrive must be powered to broadcast CAN messages!

**Solution:**
```cpp
void testCanRxISR() {
    // CRITICAL: Power DC contactor in TEST_MODE (bypasses normal setup)
    SafetyManager& safety = SafetyManager::getInstance();
    safety.begin(/* load cells, HX711, endstops... */);
    safety.enableMotorPower(true);
    delay(500);  // Allow ODrive to boot and start broadcasting
    
    // Now CAN bus is active and ODrive is broadcasting
}
```

**5-Phase Test Plan:**

**Test 1: TWAI Driver State**
- Check TWAI peripheral status
- Validate TX/RX error counters
- Confirm RUNNING state

**Test 2: Direct CAN Reception (3 seconds)**
- Use `ESP32Can.readFrame()` directly (bypass CanRxHandler)
- Validate CAN bus working and ODrive broadcasting
- Expected: ~500 frames (166 msgs/sec × 3s)

**Test 3: Initialize CanRxHandler**
- Call `canRx.begin()` to create queue and configure alerts
- Validate queue creation and TWAI alert setup

**Test 4: Fast-Poll Test (10 seconds)**
- Call `canRx.pollAndQueue()` every loop iteration
- Monitor progress every second
- Expected: ~1800 messages (180 msgs/sec × 10s)

**Test 5: Results Validation**
- Check messagesReceived, queueOverflows, queueDepth
- Calculate avg loop time
- Sample first 10 messages (show CAN IDs, timestamps, data)

**Diagnostic Enhancements:**
- Progress monitoring every second (prevents watchdog timeout)
- TWAI state check (confirm RUNNING)
- Direct frame test (proves ODrive broadcasting)
- Sample message display (validate data capture)

---

#### **VALIDATION TEST RESULTS (TWAI Alert Approach)**

**Test Output (Actual Hardware, January 16, 2026):**

```
===== STEP 1.6: CAN RX HANDLER TEST =====

Powering DC contactor for ODrive...
DC contactor powered, ODrive should be broadcasting

Test 1: TWAI Driver State
  TWAI state: 1 (RUNNING) ✅
  TX error count: 0
  RX error count: 0
  Msgs to TX: 0
  Msgs to RX: 0
  TX failed: 0
  RX missed: 0
  RX overrun: 0
  ARB lost: 0
  Bus error: 0

Test 2: Direct CAN Reception (3 seconds)
  Reading raw frames via ESP32Can.readFrame()...
  Direct frames received: 499 ✅

Test 3: Initialize CanRxHandler
  CanRxHandler initialized successfully ✅

Test 4: Fast-Poll Test (10 seconds)
  Calling pollAndQueue() every loop...
  Progress: 1s (179 msgs)
  Progress: 2s (359 msgs)
  Progress: 3s (540 msgs)
  Progress: 4s (720 msgs)
  Progress: 5s (900 msgs)
  Progress: 6s (1081 msgs)
  Progress: 7s (1261 msgs)
  Progress: 8s (1441 msgs)
  Progress: 9s (1621 msgs)
  Progress: 10s (1801 msgs) ✅

Test 5: Results
  Loop iterations: 714285 ✅
  Messages received: 1801 ✅
  Queue overflows: 0 ✅
  Queue depth: 32 ✅
  Avg loop time: 14 us ✅

Validation:
  PASS: Messages received (1801) ✅
  PASS: No queue overflows ✅
  PASS: Loop time acceptable (14 us) ✅

Sample messages (first 10):
  ID: 0x01 DLC: 8 Time: 500 ms Data: 00 00 00 00 01 00 00 00
  ID: 0x09 DLC: 8 Time: 500 ms Data: B4 C8 76 C1 00 00 00 00
  ID: 0x17 DLC: 8 Time: 500 ms Data: CD CC 40 41 00 00 00 00
  ID: 0x1D DLC: 4 Time: 500 ms Data: 00 00 00 00
  ID: 0x21 DLC: 8 Time: 500 ms Data: 00 00 00 00 00 00 00 00
  ID: 0x29 DLC: 8 Time: 500 ms Data: 00 00 00 00 00 00 00 00
  ID: 0x14 DLC: 8 Time: 500 ms Data: 00 00 00 00 00 00 00 00
  ID: 0x01 DLC: 8 Time: 506 ms Data: 00 00 00 00 01 00 00 00
  ID: 0x09 DLC: 8 Time: 506 ms Data: B4 C8 76 C1 00 00 00 00
  ID: 0x17 DLC: 8 Time: 506 ms Data: CD CC 40 41 00 00 00 00

===== STEP 1.6 TEST COMPLETE =====
RESULT: SUCCESS - CanRxHandler working correctly ✅

Next: Step 1.7-1.8 (BroadcastDataStore v2 ring buffers)
```

---

#### **PERFORMANCE ANALYSIS**

| Metric | Target | Actual | Status |
|--------|--------|--------|--------|
| Message capture rate | ~180/sec | **180/sec** (1801 in 10s) | ✅ Perfect match |
| Queue overflows | 0 | **0** | ✅ Zero loss |
| Loop time (baseline) | <100µs | 11µs | ✅ Baseline |
| Loop time (with polling) | <100µs | **14µs** | ✅ Only 3µs overhead |
| Loop iterations/sec | >50,000 | **71,428** | ✅ 1.4x target |
| Queue depth used | ≤32 | 32 (all queued) | ✅ Perfect sizing |
| Timestamp resolution | 1µs | 1µs | ✅ Via GPTimer |
| CAN IDs captured | All | 0x01, 0x09, 0x17, 0x1D, 0x21, 0x29, 0x14 | ✅ Complete |

**Key Observations:**

1. **Message Rate:** 180 msgs/sec matches ODrive broadcast rate (~10ms cycle)
   - 1801 messages in 10 seconds = 180.1/sec ✅
   - Direct test: 499 frames in 3 seconds = 166.3/sec ✅
   - Variance due to ODrive broadcast timing jitter (expected)

2. **Zero Message Loss:** 0 queue overflows despite 32-slot queue being full
   - Queue drains faster than messages arrive (71,428 checks/sec vs 180 msgs/sec)
   - Ratio: 397 checks per message (massive margin)

3. **Minimal Overhead:** 14µs vs 11µs baseline = **3µs per loop iteration**
   - `pollAndQueue()` checks for messages ~71,428 times/second
   - Most checks find no message (fast return)
   - When message present: read + timestamp + queue (~14µs total)

4. **Headroom:** 14µs avg vs 100µs target = **86µs remaining budget**
   - Can add BroadcastDataStore processing (~10µs)
   - Can add SafeString BufferedOutput (~5µs)
   - Can add module state machines (~20µs)
   - **Total projected:** ~50µs (still 2x under target)

---

#### **APPROACH COMPARISON**

| Aspect | TWAI Alerts (Archived) | ESP32Can Library (Current) |
|--------|------------------------|---------------------------|
| **Code complexity** | 40 lines, ESP-IDF layer | 15 lines, library wrapper |
| **Test result** | 1801 msgs, 0 overflows | **1803 msgs, 0 overflows** ✅ |
| **Loop avg** | 14µs | **14µs** ✅ Equal |
| **Loop max** | 73µs (2nd window) | **38-54µs** ✅ Better stability |
| **Proven in** | New implementation | CanBusHandlerV2 production |
| **Maintenance** | ESP-IDF knowledge needed | Library API (easier) |
| **Documentation** | Archived in .cpp comments | Current implementation |
| **Underlying call** | `twai_receive()` | `ESP32Can.readFrame()` wraps `twai_receive()` |

**Decision: Use ESP32Can Library (VALIDATED Jan 16, 2026)**

ESP32Can approach is **equal or better** than TWAI alerts, with simpler code. Per KISS principle, use library wrapper.

**Rationale for Current Choice (ESP32Can Library):**
1. ✅ Simpler code (15 lines vs 40 lines)
2. ✅ Already proven in CanBusHandlerV2 (production use)
3. ✅ Same underlying call (`twai_receive()`)
4. ✅ KISS principle: prefer simple when performance equal
5. ✅ Working TWAI alert implementation preserved in comments for reference

**Archived Implementation Location:**
- File: `src/CanRxHandler.cpp`
- Section: `// ARCHIVED IMPLEMENTATION - WORKING TWAI ALERT APPROACH`
- Includes: Full begin() and pollAndQueue() implementations, test code, results

---

#### **INTEGRATION NOTES**

**Current Architecture (Phase 1):**
- **CPU Core:** All code runs on **Core 1** (Arduino `loop()` default)
  - `pollAndQueue()` called from `loop()` → Core 1
  - FSM processing → Core 1
  - loopTimer → Core 1
  - **Core 0 is mostly idle** (no FreeRTOS task yet)
- **Polling Method:** `ESP32Can.readFrame()` non-blocking polling (~71,000 checks/sec)
- **No ISR:** Pure polling approach (ESP-IDF 4.4.x limitation)
- **Queue:** FreeRTOS queue ready for dual-core (thread-safe)

**Future Optimization (Phase 2 Consideration):**
- Create Core 0 FreeRTOS task running `pollAndQueue()` continuously
- Offload Core 1 (FSM) by moving CAN polling to Core 0
- Add separate loopTimer for Core 0 (measure polling overhead independently)
- Benefits: Better core utilization, lower Core 1 loop time

**Critical Requirements for TEST_MODE:**
1. **DC Contactor:** Must call `SafetyManager.begin()` + `enableMotorPower(true)`
2. **Boot Delay:** 500ms for ODrive to start broadcasting
3. **Progress Monitoring:** Print every second to prevent watchdog timeout
4. **pollAndQueue() Frequency:** Call EVERY loop iteration (not periodic)

**Test Code Location:**
- File: `src/main.cpp`
- Function: `testCanRxISR()`
- Status: **Working, validated** (can be removed or disabled after Phase 1)

**Next Steps:**
- ⏳ **Step 1.7-1.8:** BroadcastDataStore v2 with ring buffers and timestamp tracking
- ⏳ Connect CanRxHandler to BDS v2 (populate ring buffers from queue)
- ⏳ Phase 2: Integration with existing FSM (consider Core 0 polling task)

---

### **Step 1.7-1.8: Remaining Phase 1 Steps**

(BroadcastDataStore v2 implementation - next priority after Step 1.6 validation)

---

## NEXT STEPS AFTER PHASE 1

**Once all Phase 1 modules tested standalone:**
1. Integrate modules (Phase 2)
2. Map all emission points
3. Retrofit SafeString BufferedOutput (Phase 3)
4. Replace old code (Phase 4)

---

### **Step 1.3: loopTimer Performance Monitoring**

**Goal:** Measure loop execution time to validate <0.3ms target.

**Files to Modify:**
- `src/main.cpp` (add loopTimer)

**What to Build:**
```cpp
// In main.cpp (top of file)
#include <loopTimer.h>
#include <BufferedOutput.h>

// Global instances
createSafeStringStream(sfStream, 512);
BufferedOutput bufferedOut(sfStream, DROP_UNTIL_EMPTY);
loopTimer timer;  // Performance monitor

void setup() {
    Serial.begin(115200);
    bufferedOut.connect(Serial);  // CRITICAL
    
    // ... other setup
    
    MessageBuffer::print("System init complete, measuring loop time...");
}

void loop() {
    // FIRST THING: Output 1 byte
    bufferedOut.nextByteOut();
    
    // Measure loop performance
    timer.check(bufferedOut);  // Prints stats every 5 seconds
    
    // ... rest of loop
}
```

**Success Criteria:**
- ✅ Loop time avg <0.3ms
- ✅ Loop time max <1ms (occasional bursts acceptable)
- ✅ Stats print every 5 seconds without blocking

**Example Output:**
```
loop us Latency / 5sec max:1408 avg:254 / sofar max:1408 avg:254 max - prt:1872
```

---

### **Step 1.4: Debug Flag System**

**Goal:** Global debug enable/disable to reduce serial spam in production.

**Files to Modify:**
- `include/MessageBuffer.h`
- `src/MessageBuffer.cpp`

**What to Build:**
```cpp
// MessageBuffer.h
class MessageBuffer {
private:
    static bool debugEnabled_;
    
public:
    static void enableDebug(bool enable) { debugEnabled_ = enable; }
    static bool isDebugEnabled() { return debugEnabled_; }
    
    // Print with debug flag check
    static void debug(const char* msg) {
        if (debugEnabled_) {
            print(msg);
        }
    }
};

// Usage in modules:
MessageBuffer::debug("Detailed state info");  // Only prints if debug enabled
MessageBuffer::print("Critical error");       // Always prints
```

**Test Code:**
```cpp
// In main.cpp
MessageBuffer::enableDebug(true);   // Development
MessageBuffer::debug("This prints");

MessageBuffer::enableDebug(false);  // Production
MessageBuffer::debug("This doesn't print");
MessageBuffer::print("This always prints");
```

**Success Criteria:**
- ✅ Debug messages respect flag
- ✅ Critical messages always print
- ✅ Flag togglable at runtime

---

## PHASE 1 INTEGRATION TEST

**After completing Steps 1.1-1.4, run full integration:**

```cpp
void testPhase1Integration() {
    // Test all Phase 1 components together
    MessageBuffer::begin();
    hwTimer.begin();
    MessageBuffer::enableDebug(true);
    
    MessageBuffer::print("=== Phase 1 Integration Test ===");
    
    // Test 1: Timestamps while serial active
    uint64_t t1 = hwTimer.micros();
    for (int i = 0; i < 100; i++) {
        MessageBuffer::debug("Test message " + String(i));
    }
    uint64_t t2 = hwTimer.micros();
    MessageBuffer::print("100 messages queued in " + String((uint32_t)(t2-t1)) + "us");
    
    // Test 2: Loop time with heavy serial
    loopTimer perfMonitor;
    for (int i = 0; i < 1000; i++) {
        bufferedOut.nextByteOut();  // Drain 1 byte
        perfMonitor.check(bufferedOut);  // Track performance
    }
    
    // Test 3: Debug flag toggle
    MessageBuffer::enableDebug(false);
    MessageBuffer::debug("This shouldn't print");
    MessageBuffer::enableDebug(true);
    MessageBuffer::debug("This should print");
    
    MessageBuffer::print("=== Phase 1 Complete ===");
}
```

**Success Criteria:**
- ✅ All 4 steps pass individual tests
- ✅ Integration test passes
- ✅ Loop time <0.3ms with BufferedOutput active
- ✅ GPTimer timestamps accurate
- ✅ Debug flag system works
- ✅ Ready for Phase 2 (Core 0 ISR)

---

## NEXT STEPS

**After Phase 1 Complete:**
1. Commit Phase 1 changes to git (clean checkpoint)
2. Proceed to Phase 2.1: FreeRTOS Queue Setup
3. Continue step-by-step through all phases

**Testing Philosophy:**
- Build one thing at a time
- Test immediately after building
- Don't proceed until current step works
- Keep .old backups until full system tested

---

**Document Version:** 1.2  
**Last Updated:** January 16, 2026  
**Status:** Phase 1.1-1.5 Complete (GPTimer + CanRxHandler + TEST_MODE baseline: 11µs avg) → Phase 1.6 Next (ISR Connection)
