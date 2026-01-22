// Example usage of OurLoopTimer library
// Demonstrates minimal integration for loop timing

#include <Arduino.h>
#include <OurLoopTimer.h>

#define LED_PIN 2

// Your work task variables
bool ledOn = false;
unsigned long ledLastToggle = 0;
const unsigned long LED_INTERVAL = 1000;  // [DELAY] Toggle every 1000ms

unsigned long printLastTime = 0;
const unsigned long PRINT_INTERVAL = 5000;  // [DELAY] Print every 5000ms

// ============================================================================
// WORK TASK - Your production code
// ============================================================================
void blinkLedTask() {
  unsigned long now = millis();
  if (now - ledLastToggle >= LED_INTERVAL) {  // [DELAY] 1000ms interval
    ledLastToggle = now;
    ledOn = !ledOn;
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  }
}

void printStatusTask() {
  unsigned long now = millis();
  if (now - printLastTime >= PRINT_INTERVAL) {  // [DELAY] 5000ms interval
    printLastTime = now;
    printLoopStats();  // From OurLoopTimer
  }
}

// ============================================================================
// WORK LOOP - Runs on CPU_WORKING
// ============================================================================
void workLoop(void *parameter) {
  while (true) {
    toggleLoopFlag();  // MUST be at START or END of loop
    
    // Your production code tasks
    blinkLedTask();
    printStatusTask();
    
    // Reset watchdog manually
    esp_task_wdt_reset();
    
    // [DELAY] Your work delay here (0 for fastest loop)
    delayMicroseconds(0);
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize loop timer (creates measuring task)
  initLoopTimer();
  
  // Start work task on CPU_WORKING (code under test)
  TaskHandle_t workTaskHandle = NULL;
  xTaskCreatePinnedToCore(
    workLoop,
    "Working",
    4096,
    NULL,
    1,
    &workTaskHandle,
    CPU_WORKING
  );
  
  // Subscribe work task to watchdog
  if (workTaskHandle != NULL) {
    esp_task_wdt_add(workTaskHandle);
  }
}

// ============================================================================
// MAIN LOOP - Empty (all work in pinned tasks)
// ============================================================================
void loop() {
  delay(1000);
}
