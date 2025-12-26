# Using the Non-Blocking Architecture - Developer Guide

## Quick Reference

### 1. Reading Broadcast Data (Non-Blocking)
```cpp
#include "BroadcastDataStore.h"

// Get singleton instance
BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();

// Query current state (cached from latest broadcast)
uint8_t odState = broadcast.getAxisState();
float position = broadcast.getPosition();        // turns
float velocity = broadcast.getVelocity();        // turns/sec

// Power data
float current = broadcast.getIqMeasured();       // Amps
float voltage = broadcast.getBusVoltage();       // Volts

// Sensor data
float temperature = broadcast.getTemperature();  // °C
float pressure = broadcast.getPressure();        // Nm

// Endstop status
bool atTop = broadcast.isTopEndstopActive();
```

### 2. Making Decisions Without Polling
```cpp
// Instead of: while (velocity > 0.1) { delay(10); }
// Use velocity threshold checks:
if (broadcast.isVelocityBelowThreshold(0.1f)) {
    // Motor has stopped
}

// Or general motion check:
if (broadcast.isMoving()) {
    // Motor is moving (>0.05 turns/sec)
}

// Check if data is fresh:
if (!broadcast.isAxisDataStale(500)) {
    // Data is less than 500ms old
}
```

### 3. Serial Output (Non-Blocking)
```cpp
#include "SerialMessaging.h"

// In setup()
SerialMessaging::begin();

// In loop() - accumulate status
SerialMessaging::appendStatus("Mode", controlMode);
SerialMessaging::appendStatus("Pos", position);
SerialMessaging::appendStatus("Vel", velocity);

// Prints automatically at 1Hz without blocking
SerialMessaging::printStatusMessage();

// For critical errors (OK to block briefly):
SerialMessaging::printError("Motor overcurrent!");
```

### 4. Homing State Machine (Non-Blocking)
```cpp
// Trigger homing from button:
if (btnUpper.fell()) {
    Homing::begin(motor, safety);
}

// In main loop - update state machine once per iteration:
if (!Homing::isComplete()) {
    Homing::update(motor, safety);
}

// Check for completion:
if (Homing::isComplete()) {
    Serial.println("Homing done!");
}

// Check for errors:
if (Homing::hasError()) {
    Serial.println("Homing failed!");
}

// Get current state for UI display:
const char* state = Homing::getStateString();  // "RETRACT_FAST", "APPROACH", etc
```

## Pattern: Creating a New Non-Blocking Control State Machine

### Example: Injection Control

**Step 1: Create Header**
```cpp
// include/InjectionControl.h
class InjectionControl {
public:
    enum class InjectionState {
        IDLE, BUILD_PRESSURE, HOLD, INJECT, RELEASE, DONE, ERROR
    };
    
    static void begin(CanBusHandlerV2& motor, SafetyManager& safety);
    static void update(CanBusHandlerV2& motor, SafetyManager& safety);
    static bool isComplete();
    static const char* getStateString();
    
private:
    static InjectionState currentState_;
    static InjectionState previousState_;
    static uint32_t stateEnteredMs_;
    
    // Handler for each state
    static void handleBuildPressure(...);
    static void handleHold(...);
    static void handleInject(...);
    static void handleRelease(...);
    static void nextState(InjectionState newState);
};
```

**Step 2: Implement Non-Blocking Updates**
```cpp
// src/InjectionControl.cpp
void InjectionControl::update(CanBusHandlerV2& motor, SafetyManager& safety) {
    BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
    
    switch (currentState_) {
        case InjectionState::BUILD_PRESSURE:
            handleBuildPressure(motor, safety);
            break;
        case InjectionState::HOLD:
            handleHold(motor, safety);
            break;
        // ... etc
    }
}

void InjectionControl::handleBuildPressure(CanBusHandlerV2& motor, SafetyManager& safety) {
    BroadcastDataStore& broadcast = BroadcastDataStore::getInstance();
    
    // One-time initialization (when entering this state)
    if (previousState_ != InjectionState::BUILD_PRESSURE) {
        Serial.println("Starting pressure build...");
        motor.setControllerModes(ControlMode::TORQUE, InputMode::PASSTHROUGH);
        motor.setInputTorque(INJECTION_TORQUE);
    }
    
    // Check condition (on every loop iteration)
    float pressure = broadcast.getPressure();
    if (pressure > INJECTION_PRESSURE_TARGET) {
        Serial.println("Pressure reached, holding...");
        nextState(InjectionState::HOLD);
        return;
    }
    
    // Timeout protection
    if (millis() - stateEnteredMs_ > 5000) {
        Serial.println("ERROR: Pressure timeout!");
        nextState(InjectionState::ERROR);
    }
}
```

**Step 3: Use in Main Loop**
```cpp
// In main loop
if (fsm_state.currentState == COMPRESSION) {
    if (InjectionControl::getState() == InjectionControl::InjectionState::IDLE) {
        InjectionControl::begin(motor, safety);
    }
    
    if (!InjectionControl::isComplete()) {
        InjectionControl::update(motor, safety);
    }
    
    // Status output continues uninterrupted
    SerialMessaging::appendStatus("InjState", InjectionControl::getStateString());
    SerialMessaging::printStatusMessage();
    
    if (InjectionControl::isComplete()) {
        fsm_state.currentState = READY_TO_INJECT;
    }
}
```

## Common Mistakes to Avoid

### ❌ DON'T: Query motor state repeatedly
```cpp
// BAD - expensive CAN queries in a loop
while (millis() - start < 5000) {
    uint8_t state = motor.getAxisState();  // CAN query every iteration!
    if (state == 8) break;
    delay(50);
}
```

### ✅ DO: Use cached broadcast data
```cpp
// GOOD - single cached query
if (broadcast.getAxisState() == 8) {
    // Proceed
}
```

### ❌ DON'T: Use delay() for timing
```cpp
// BAD - blocks everything for 1 second
delay(1000);
```

### ✅ DO: Use elapsed time checks
```cpp
// GOOD - non-blocking timing
if (millis() - stateEnteredMs_ > 1000) {
    // 1 second has elapsed
}
```

### ❌ DON'T: Call blocking functions from loop
```cpp
// BAD - runHomingSequence() blocks main loop
if (runHomingSequence()) {
    // Never reached until homing completes
}
```

### ✅ DO: Use state machine pattern
```cpp
// GOOD - update processes one state per call
Homing::update(motor, safety);  // Returns immediately
// Rest of loop executes
```

### ❌ DON'T: Print status inside functions
```cpp
// BAD - Serial output scattered everywhere
void handleRetract() {
    Serial.println("Retracting");  // May interfere with other output
}
```

### ✅ DO: Queue output via SerialMessaging
```cpp
// GOOD - centralized, non-blocking output
SerialMessaging::appendStatus("State", "Retracting");
SerialMessaging::printStatusMessage();  // Prints at 1Hz
```

## Verifying Non-Blocking Behavior

### Test 1: Status Message Continuity
- Run debug homing mode
- Watch serial output for 1-2 minutes
- **Expected:** Status messages every 1 second, no gaps (seconds counter should be steady)
- **Bad sign:** Messages stop for 2+ seconds during homing (indicates blocking)

### Test 2: Button Responsiveness
- Start homing sequence
- Press buttons during motion
- **Expected:** LED colors change immediately
- **Bad sign:** Button presses ignored while homing runs

### Test 3: CPU Utilization
- Monitor main loop rate (add microsecond timer)
- **Expected:** Loop runs continuously, ~100+ times per second
- **Bad sign:** Loop stalls during operations (indicates delay/blocking)

## Configuration & Tuning

### Adjust Print Interval
```cpp
// In main.cpp
SerialMessaging::setPrintIntervalMs(2000);  // Print every 2 seconds instead of 1
```

### Adjust Homing Timeouts
```cpp
// In Homing.cpp - modify timeout constants
#define CALIB_TIMEOUT_MS      15000
#define RETRACT_TIMEOUT_MS    10000
#define APPROACH_TIMEOUT_MS   10000
```

### Adjust BroadcastDataStore Freshness Check
```cpp
// Check if data is less than 200ms old (instead of 500ms)
if (!broadcast.isAxisDataStale(200)) {
    // Only proceed if recent data
}
```

## Debugging Non-Blocking Code

### Add Debug Logging
```cpp
// In state handlers
if (previousState_ != InjectionState::BUILD_PRESSURE) {
    SerialMessaging::printDebug("Entering BUILD_PRESSURE state");
}

// Check transitions
SerialMessaging::appendStatus("InjState", InjectionControl::getStateString());
```

### Monitor Broadcast Data Age
```cpp
uint32_t axisAge = broadcast.getAxisDataAgeMsecs();
if (axisAge > 100) {
    SerialMessaging::printInfo("Axis data is old, may indicate CAN issues");
}
```

### Use State String for UI
```cpp
// Display homing progress via LEDs
const char* state = Homing::getStateString();
if (strcmp(state, "RETRACT_FAST") == 0) {
    setLedColor(RED);  // Retracting
} else if (strcmp(state, "APPROACH") == 0) {
    setLedColor(YELLOW);  // Approaching
}
```

## Performance Characteristics

| Operation | Blocking Time | Non-Blocking Time |
|-----------|---------------|-------------------|
| Homing sequence | 30-60 seconds (blocks loop) | 0μs per call (1000+ calls/sec) |
| Position query | 10-50ms CAN latency | <1μs (cached) |
| Serial output | 50-100ms (115200 baud) | 0μs per call (buffered) |
| State machine step | N/A (blocking) | <10μs typical |

## Memory Usage
- BroadcastDataStore: ~200 bytes (static data structures)
- SerialMessaging: ~512 bytes (SafeString buffer)
- Homing state machine: ~100 bytes (state variables + history)
- **Total: ~800 bytes** (minimal impact on 320KB ESP32 RAM)

## Summary
The non-blocking architecture ensures:
1. **Responsive UI:** Button input always processed
2. **Continuous monitoring:** Status messages print uninterrupted
3. **Scalable:** Easy to add new state machines following same pattern
4. **Maintainable:** State logic clearly organized
5. **Debuggable:** Clear state strings and data values visible
