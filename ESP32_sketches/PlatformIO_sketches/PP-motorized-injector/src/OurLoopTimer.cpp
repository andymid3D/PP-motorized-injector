// OurLoopTimer.cpp
// Implementation of dual-core loop timer for ESP32

#include "OurLoopTimer.h"

// ============================================================================
// MEASUREMENT VARIABLES - Define storage
// ============================================================================
#if CPU_WORKING == 0 && CPU_MEASURING == 1
  // CPU0 works, CPU1 measures CPU0
  volatile bool cpu0LoopToggle = false;
  volatile uint64_t cpu0LastToggleTime = 0;
  volatile uint64_t cpu0LoopTimeUs = 0;
  volatile uint64_t cpu0MaxLoopTimeUs = 0;
  volatile uint32_t cpu0LoopCount = 0;
  volatile uint64_t cpu0SumLast5s = 0;
  volatile uint32_t cpu0CountLast5s = 0;
  volatile uint32_t cpu0AvgLast5s = 0;
#elif CPU_WORKING == 1 && CPU_MEASURING == 0
  // CPU1 works, CPU0 measures CPU1
  volatile bool cpu1LoopToggle = false;
  volatile uint64_t cpu1LastToggleTime = 0;
  volatile uint64_t cpu1LoopTimeUs = 0;
  volatile uint64_t cpu1MaxLoopTimeUs = 0;
  volatile uint32_t cpu1LoopCount = 0;
  volatile uint64_t cpu1SumLast5s = 0;
  volatile uint32_t cpu1CountLast5s = 0;
  volatile uint32_t cpu1AvgLast5s = 0;
#endif

// ============================================================================
// MEASURING TASK - Runs on CPU_MEASURING
// ============================================================================
// This task does NOTHING except watch the toggle flag and calculate timing
// Minimal code to minimize measurement overhead
void measuringTask(void *parameter) {
  // FreeRTOS task MUST have infinite loop (while/for) or it will exit and crash
  // This runs continuously on CPU_MEASURING, independent of main loop()
  
#if CPU_MEASURING == 0  // CPU0 measures CPU1
  bool lastSeenToggle = cpu1LoopToggle;
  esp_task_wdt_add(NULL); // Add this task to WDT
  while (true) {
    bool currentToggle = cpu1LoopToggle;
    if (currentToggle != lastSeenToggle) {
      uint64_t now = getTimerMicros();
      if (cpu1LastToggleTime != 0) {
        cpu1LoopTimeUs = now - cpu1LastToggleTime;
        if (cpu1LoopTimeUs > cpu1MaxLoopTimeUs) {
          cpu1MaxLoopTimeUs = cpu1LoopTimeUs;
        }
        cpu1LoopCount++;
        // Accumulate for 5-second average
        cpu1SumLast5s += cpu1LoopTimeUs;
        cpu1CountLast5s++;
      }
      cpu1LastToggleTime = now;
      lastSeenToggle = currentToggle;
    }
    // Reset watchdog manually on our microsecond terms
    esp_task_wdt_reset();
    delayMicroseconds(1);
  }
  
#elif CPU_MEASURING == 1  // CPU1 measures CPU0
  bool lastSeenToggle = cpu0LoopToggle;
  esp_task_wdt_add(NULL); // Add this task to WDT
  while (true) {
    bool currentToggle = cpu0LoopToggle;
    if (currentToggle != lastSeenToggle) {
      uint64_t now = getTimerMicros();
      if (cpu0LastToggleTime != 0) {
        cpu0LoopTimeUs = now - cpu0LastToggleTime;
        if (cpu0LoopTimeUs > cpu0MaxLoopTimeUs) {
          cpu0MaxLoopTimeUs = cpu0LoopTimeUs;
        }
        cpu0LoopCount++;
        // Accumulate for 5-second average
        cpu0SumLast5s += cpu0LoopTimeUs;
        cpu0CountLast5s++;
      }
      cpu0LastToggleTime = now;
      lastSeenToggle = currentToggle;
    }
    // Reset watchdog manually on our microsecond terms
    esp_task_wdt_reset();
    delayMicroseconds(1);
  }
#endif
}

// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

void initLoopTimer() {
  Serial.println("\n=== OurLoopTimer Initialized ===");
  Serial.print("Working CPU: ");
  Serial.println(CPU_WORKING);
  Serial.print("Measuring CPU: ");
  Serial.println(CPU_MEASURING);
  Serial.println("Format: CX:avg5s/max (us)");
  Serial.println("================================\n");
  
  // WDT initialization is now handled by CanRxHandler::begin()
  // Each task will add itself to the WDT
  
  // Start measuring task on CPU_MEASURING (does NOTHING but measure)
  TaskHandle_t measuringTaskHandle = NULL;
  xTaskCreatePinnedToCore(
    measuringTask,
    "Measuring",
    2048,
    NULL,
    1,  // Priority
    &measuringTaskHandle,
    CPU_MEASURING
  );
}

void printLoopStats() {
  // Calculate 5-second average and print: avg5s/max (microseconds)
#if CPU_WORKING == 0
  if (cpu0CountLast5s > 0) {
    cpu0AvgLast5s = cpu0SumLast5s / cpu0CountLast5s;
  }
  Serial.print("C0:");
  Serial.print(cpu0AvgLast5s);
  Serial.print("/");
  Serial.println(cpu0MaxLoopTimeUs);
  // Reset 5-second window
  cpu0SumLast5s = 0;
  cpu0CountLast5s = 0;
#elif CPU_WORKING == 1
  if (cpu1CountLast5s > 0) {
    cpu1AvgLast5s = cpu1SumLast5s / cpu1CountLast5s;
  }
  Serial.print("C1:");
  Serial.print(cpu1AvgLast5s);
  Serial.print("/");
  Serial.println(cpu1MaxLoopTimeUs);
  // Reset 5-second window
  cpu1SumLast5s = 0;
  cpu1CountLast5s = 0;
#endif
}
