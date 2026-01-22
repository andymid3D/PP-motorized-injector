// OurLoopTimer.h
// Dual-Core Loop Timer for ESP32
// Measures one CPU's loop time from the other CPU without interference
// Uses the project's custom GPTimer for deterministic, high-resolution timing.
//
// INTEGRATION PROCEDURE:
// 1. Include this header in your main file
// 2. Set CPU_WORKING (0 or 1) - which CPU runs code under test
// 3. Set CPU_MEASURING (opposite) - which CPU only measures
// 4. Call initLoopTimer() in setup() AFTER initializing serial and GPTimer
// 5. In your work loop, toggle at START or END: toggleLoopFlag()
// 6. Call printLoopStats() periodically to output measurements
// 7. Document ALL delays in your code with [DELAY] comments
//
// OUTPUT FORMAT: CX:avg5s/max (microseconds)
// - avg5s: 5-second rolling average of loop time
// - max: maximum loop time since boot

#ifndef OUR_LOOP_TIMER_H
#define OUR_LOOP_TIMER_H

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "GPTimer.h"  // Use the project's custom hardware timer

// ============================================================================
// CONFIGURATION - Which CPU does what
// ============================================================================
// Set which CPU does WORK (code under test) vs MEASUREMENT (timing only)
#define CPU_WORKING 1        // CPU1 runs code under test
#define CPU_MEASURING 0      // CPU0 measures CPU1's loop time

// ============================================================================
// TIMING SOURCE - Common for ALL modules
// ============================================================================
// Make the linker aware of the global hwTimer instance defined elsewhere
extern GPTimer hwTimer;

// Helper function to get current time in microseconds from our custom timer
inline uint64_t getTimerMicros() {
  return hwTimer.micros();  // Use project's custom GPTimer
}

// ============================================================================
// MEASUREMENT VARIABLES - Only compile what's needed
// ============================================================================
#if CPU_WORKING == 0 && CPU_MEASURING == 1
  // CPU0 works, CPU1 measures CPU0
  extern volatile bool cpu0LoopToggle;
  extern volatile uint64_t cpu0LastToggleTime;
  extern volatile uint64_t cpu0LoopTimeUs;
  extern volatile uint64_t cpu0MaxLoopTimeUs;
  extern volatile uint32_t cpu0LoopCount;
  extern volatile uint64_t cpu0SumLast5s;
  extern volatile uint32_t cpu0CountLast5s;
  extern volatile uint32_t cpu0AvgLast5s;
#elif CPU_WORKING == 1 && CPU_MEASURING == 0
  // CPU1 works, CPU0 measures CPU1
  extern volatile bool cpu1LoopToggle;
  extern volatile uint64_t cpu1LastToggleTime;
  extern volatile uint64_t cpu1LoopTimeUs;
  extern volatile uint64_t cpu1MaxLoopTimeUs;
  extern volatile uint32_t cpu1LoopCount;
  extern volatile uint64_t cpu1SumLast5s;
  extern volatile uint32_t cpu1CountLast5s;
  extern volatile uint32_t cpu1AvgLast5s;
#else
  #error "Invalid CPU configuration: Set CPU_WORKING to 0 or 1, CPU_MEASURING to the other"
#endif

// ============================================================================
// PUBLIC API
// ============================================================================

// Initialize loop timer system
// Call in setup() AFTER Serial.begin() and hwTimer.begin()
// Configures watchdog, creates measuring task
void initLoopTimer();

// Toggle loop flag - call ONCE per loop at start or end
// This is what the measuring CPU tracks
inline void toggleLoopFlag() {
#if CPU_WORKING == 0
  cpu0LoopToggle = !cpu0LoopToggle;
#elif CPU_WORKING == 1
  cpu1LoopToggle = !cpu1LoopToggle;
#endif
}

// Print loop statistics: avg5s/max
// Call periodically (e.g., every 5 seconds)
// Resets 5-second window after printing
void printLoopStats();

// Get current average (last 5s window)
inline uint32_t getLoopAvg5s() {
#if CPU_WORKING == 0
  return cpu0AvgLast5s;
#elif CPU_WORKING == 1
  return cpu1AvgLast5s;
#endif
}

// Get maximum loop time since boot
inline uint64_t getLoopMax() {
#if CPU_WORKING == 0
  return cpu0MaxLoopTimeUs;
#elif CPU_WORKING == 1
  return cpu1MaxLoopTimeUs;
#endif
}

// Get total loop count
inline uint32_t getLoopCount() {
#if CPU_WORKING == 0
  return cpu0LoopCount;
#elif CPU_WORKING == 1
  return cpu1LoopCount;
#endif
}

#endif // OUR_LOOP_TIMER_H
