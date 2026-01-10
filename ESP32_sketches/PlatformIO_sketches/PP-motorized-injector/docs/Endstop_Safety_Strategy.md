# Endstop Safety Strategy

**Version:** 1.0  
**Date:** January 8, 2026  
**Hardware:** ESP32 Controller + ODrive 3.6 + Inductive Endstops

---

## Overview

This document specifies the safety strategy for endstop collision detection, end-of-day position safety, and recovery procedures. The system must prevent unexpected endstop collisions during normal operation while allowing intentional endstop contact during homing and calibration sequences.

---

## Endstop Hardware Configuration

### Physical Endstops
- **Top Endstop (PIN 19):** Detects plunger at home position (0.0 turns)
- **Bottom Endstop (PIN 18):** Detects plunger at maximum travel (~355 turns)
- **Barrel Endstop (PIN 5):** Detects barrel position (safety interlock)

### Endstop Logic
- **Active LOW:** Endstop hit when pin reads LOW (0V)
- **Pulled HIGH:** Internal pullup resistors, pin reads HIGH when endstop not hit
- **Wiring:** Inductive proximity sensors, NO (normally open) configuration

---

## Collision Detection (SafetyManager)

### Safety Context Awareness

The system uses **Safety Context** to determine whether endstop contact is expected or unexpected:

| Context | Description | Endstop Contact Expected? |
|---------|-------------|---------------------------|
| `CTX_IDLE` | Idle waiting, no movement | NO - collision error |
| `CTX_MOVING_FREE` | Homing, AntiDrip | YES - intentional contact allowed |
| `CTX_BLOCKED` | Injection, Hold, Compression | NO - collision error |
| `CTX_PURGE` | Manual purge (button-controlled) | NO - collision error |

### Collision Detection Logic (SafetyManager::check())

```cpp
// Collision detection (only when NOT in CTX_MOVING_FREE)
if (_currentContext != CTX_MOVING_FREE) {
    // Bottom endstop collision (moving down)
    if (isBottomEndstopHit() && is_moving_down && abs(current_velocity) > 0.1f) {
        triggerHalt(ERR_BOTTOM_ENDSTOP_COLLISION);  // Error code 0x0007
        return false;
    }
    
    // Top endstop collision (moving up)
    if (isTopEndstopHit() && !is_moving_down && abs(current_velocity) > 0.1f) {
        triggerHalt(ERR_TOP_ENDSTOP_COLLISION);  // Error code 0x0008
        return false;
    }
}
```

### Collision Detection Criteria
1. **Velocity threshold:** Must be moving (> 0.1 turns/sec) to trigger collision
2. **Direction check:** Bottom collision only when moving down, top collision only when moving up
3. **Context exemption:** CTX_MOVING_FREE (homing/antidrip) allows intentional endstop contact
4. **Immediate halt:** Motor stopped immediately, error state entered

### Error Codes

| Error Code | Name | Description | Recovery |
|------------|------|-------------|----------|
| `0x0007` | ERR_BOTTOM_ENDSTOP_COLLISION | Plunger hit bottom endstop during downward movement | Home again, check position limits |
| `0x0008` | ERR_TOP_ENDSTOP_COLLISION | Plunger hit top endstop during upward movement | Home again, verify encoder zero |

---

## End-of-Day Position Safety

### Problem: Plunger Freeze Risk

**Scenario:**
- Plunger left outside heated zone (< 90.2 turns) for >30 minutes
- Plastic in barrel cools below melt temperature
- Plunger becomes stuck in solidified plastic
- Requires manual intervention (heat gun, disassembly)

**Solution:** `flags.endOfDay` Toggle

### Position Zones

| Zone | Position Range | Description | Safety Status |
|------|----------------|-------------|---------------|
| **Home** | 0.0 turns | Top endstop | ⚠️ COLD ZONE (5-10 min timeout) |
| **Refill** | 47.7 turns | Rest position | ⚠️ COLD ZONE (5-10 min timeout) |
| **Cold Zone** | 0 - 90.2 turns | Below heated zone | ⚠️ FREEZE RISK (5-10 min max) |
| **Heated Zone** | 90.2 - 355 turns | Inside heated barrel | ✅ SAFE (plastic stays molten) |

**Cold Zone Timer Strategy:**
- System tracks time spent below 90.2 turns (Home or Refill positions)
- After **5-10 minutes** in cold zone, system triggers warning to Display
- Display shows countdown timer and prompts user to move to heated zone
- If user ignores warning, system can auto-transition to READY_TO_INJECT (heated zone)
- Timer resets when plunger enters heated zone (>90.2 turns)

### End-of-Day Cycle Flow

**Normal Operation (flags.endOfDay = false):**
```
INJECT → HOLD → RELEASE → CONFIRM_REMOVAL → REFILL (47.7 turns, cold zone)
```

**End-of-Day Mode (flags.endOfDay = true):**
```
INJECT → HOLD → RELEASE → CONFIRM_REMOVAL → READY_TO_INJECT (~74 turns, heated zone)
```

### Toggle Mechanism (REFILL State Only)

**Button Combination:** Upper + Lower buttons pressed simultaneously

**Implementation:**
```cpp
// In REFILL state:
if (buttonUpper.rose() && buttonLower.rose()) {
    flags.endOfDay = !flags.endOfDay;  // Toggle flag
    delay(UI_BUTTON_TOGGLE_DELAY_MS);   // Prevent double-toggle
    logMessage(flags.endOfDay ? "End-of-Day: ON" : "End-of-Day: OFF");
}
```

**LED Feedback:**
- **flags.endOfDay = false:** Upper/Lower buttons BLACK, Center button GREEN
- **flags.endOfDay = true:** Upper/Lower buttons BLUE, Center button GREEN

### Safe Shutdown Positions

| Position | Zone | Safe for Shutdown? | Notes |
|----------|------|-------------------|-------|
| 0.0 turns (Home) | Cold Zone | ⚠️ LIMITED | Safe for 5-10 minutes max, then freeze risk |
| 47.7 turns (Refill) | Cold Zone | ⚠️ LIMITED | Safe for 5-10 minutes max, then freeze risk |
| 74+ turns (Ready) | Heated Zone | ✅ YES | Plastic stays molten, safe indefinitely |
| 90.2+ turns | Heated Zone | ✅ YES | Plastic stays molten, safe indefinitely |

**Recommendation:** If machine will be idle >5 minutes, enable `flags.endOfDay = true` to return to READY_TO_INJECT state (~74 turns, heated zone).

**Cold Zone Timer Implementation (Suggested):**
```cpp
// In main.cpp global variables:
unsigned long coldZoneEntryTime = 0;
bool coldZoneTimerActive = false;
const unsigned long COLD_ZONE_TIMEOUT_MS = 300000;  // 5 minutes (configurable 5-10 min)

// In loop(), check position and manage timer:
float currentPosition = motor.getPosition();
if (currentPosition < POS_HEATED_ZONE_START) {  // Below 90.2 turns
    if (!coldZoneTimerActive) {
        coldZoneEntryTime = millis();
        coldZoneTimerActive = true;
    }
    
    unsigned long timeInColdZone = millis() - coldZoneEntryTime;
    if (timeInColdZone > COLD_ZONE_TIMEOUT_MS) {
        // Send warning to Display
        DisplayComms::broadcastError(0xFFFF, "COLD_ZONE_TIMEOUT_WARNING");
        // Optional: Auto-transition to READY_TO_INJECT after additional grace period
    }
} else {
    coldZoneTimerActive = false;  // Reset timer when in heated zone
}
```

---

## Homing Sequence (Intentional Endstop Contact)

### Safety Context: CTX_MOVING_FREE

During homing, the system **intentionally** contacts the top endstop to establish the encoder zero point. This requires collision detection to be **disabled** for CTX_MOVING_FREE context.

### Homing Sequence (Homing.cpp)

**Phase 5: Fast Retract to Top Endstop**
```cpp
// Step 5: Fast retract up until top endstop hit
motor.setControllerModes(2, 1);  // Velocity control, PASSTHROUGH
motor.setInputVel(HOMING_FAST_VEL);  // -12.5 turns/sec (up)
safety.setContext(CTX_MOVING_FREE);  // Disable collision detection

// Wait for top endstop hit
if (safety.isTopEndstopHit()) {
    // Endstop hit, proceed to deceleration
}
```

**Phase 6: Gradual Deceleration**
```cpp
// Step 6: Slow down to prevent spinout (velocity ramp)
motor.setInputVel(HOMING_FAST_VEL * 0.5);  // 50% velocity
delay(200ms);
motor.setInputVel(HOMING_FAST_VEL * 0.25);  // 25% velocity
delay(200ms);
motor.setControllerModes(1, 1);  // Idle mode
// Wait for velocity < 0.1 turns/sec
```

**Phase 7: Backoff from Endstop**
```cpp
// Step 7: Back off to relax endstop pressure
motor.setControllerModes(2, 1);  // Velocity control, PASSTHROUGH
motor.setInputVel(HOMING_BACKOFF_VEL);  // +2.5 turns/sec (down)
delay(HOMING_BACKOFF_DURATION);  // 1500ms
motor.setControllerModes(1, 1);  // Idle mode
```

**Phase 10: Reset Encoder to Zero**
```cpp
// Step 10: Set encoder position to 0.0 (home position)
motor.setAbsolutePosition(0.0f);
flags.initialHomingDone = true;
safety.setContext(CTX_IDLE);  // Re-enable collision detection
```

### Key Design Points
- **CTX_MOVING_FREE:** Collision detection disabled during homing
- **Intentional contact:** Top endstop hit is expected, not an error
- **Velocity threshold:** Still enforced (velocity > 0.1 to register hit)
- **Backoff sequence:** Relieves endstop pressure to prevent false triggers
- **Context restoration:** After homing complete, restore CTX_IDLE (collision detection re-enabled)

---

## AntiDrip Sequence (Intentional Upward Movement)

### Safety Context: CTX_MOVING_FREE

AntiDrip slowly retracts the plunger upward to prevent plastic drip while the user places the mould. This movement is **intentional** and **user-controlled**, so collision detection is disabled.

### AntiDrip Logic (AntiDrip.cpp)

```cpp
// Set context to MOVING_FREE (allow top endstop hit without error)
safety.setContext(CTX_MOVING_FREE);

// Slow upward movement
motor.setControllerModes(2, 1);  // Velocity control, PASSTHROUGH
motor.setInputVel(SPEED_ANTIDRIP);  // -2.0 turns/sec (up)

// Button interrupt or timeout
if (buttonCenterReleased && buttonLowerReleased) {
    // User confirmed mould placed, proceed to INJECT
    motor.setControllerModes(1, 1);  // Idle mode
    safety.setContext(CTX_BLOCKED);  // Re-enable collision detection
    return true;  // Complete
}

if (buttonUpperReleased || timeout_exceeded) {
    // User abort or timeout, return to READY_TO_INJECT
    motor.setControllerModes(1, 1);  // Idle mode
    safety.setContext(CTX_IDLE);  // Re-enable collision detection
    return true;  // Complete
}
```

### Key Design Points
- **CTX_MOVING_FREE:** Collision detection disabled during antidrip
- **User-controlled:** User can interrupt with buttons at any time
- **Timeout protection:** 15-second timeout prevents indefinite upward movement
- **Context restoration:** After antidrip complete, restore appropriate context

---

## Collision Recovery Procedures

### Bottom Endstop Collision (ERR_BOTTOM_ENDSTOP_COLLISION)

**Symptoms:**
- Motor moving down (positive velocity)
- Bottom endstop hit unexpectedly
- Error code 0x0007 logged

**Possible Causes:**
1. **Position limit misconfigured:** `POS_BOTTOM_MAX` set too high
2. **Encoder drift:** Encoder zero point drifted over time
3. **Injection overshoot:** Injection target position exceeded safe travel
4. **Mould parameter error:** fillVolume + packVolume too large

**Recovery Steps:**
1. **Immediate:** Motor stopped, error state entered
2. **User Action:** Press Center button to acknowledge error
3. **System Action:** Return to ERROR_STATE → INIT_HOT_NOT_HOMED
4. **User Action:** Press Upper button to re-home (establish encoder zero)
5. **Verification:** Check position limits in config.h (POS_BOTTOM_MAX)
6. **Test:** Perform manual purge to verify full travel range
7. **Prevention:** Reduce fillVolume or packVolume if needed

### Top Endstop Collision (ERR_TOP_ENDSTOP_COLLISION)

**Symptoms:**
- Motor moving up (negative velocity)
- Top endstop hit unexpectedly (not during homing)
- Error code 0x0008 logged

**Possible Causes:**
1. **Encoder zero point lost:** Encoder position drifted from true home
2. **Refill overshoot:** Refill target position exceeded safe travel
3. **AntiDrip timeout:** AntiDrip moved too far upward
4. **Manual purge error:** User pushed plunger too far up in PURGE_ZERO

**Recovery Steps:**
1. **Immediate:** Motor stopped, error state entered
2. **User Action:** Press Center button to acknowledge error
3. **System Action:** Return to ERROR_STATE → INIT_HOT_NOT_HOMED
4. **User Action:** Press Upper button to re-home (establish encoder zero)
5. **Verification:** Check refill position in config.h (OFFSET_REFILL_GAP)
6. **Test:** Perform homing sequence to verify encoder accuracy
7. **Prevention:** Adjust OFFSET_REFILL_GAP or SPEED_ANTIDRIP if needed

---

## Position Limit Validation

### Software Limits (config.h)

```cpp
// Position Definitions (Turns)
#define POS_HOME                0.0f        // Home position at top endstop
#define OFFSET_REFILL_GAP       47.746f     // Distance from home to refill rest
#define OFFSET_COLD_ZONE        42.441f     // Cold zone below refill gap
#define POS_HEATED_ZONE_START   (OFFSET_REFILL_GAP + OFFSET_COLD_ZONE)  // 90.187 turns
#define STROKE_HEATED_ZONE      265.25f     // Length of heated zone
#define POS_BOTTOM_MAX          (POS_HEATED_ZONE_START + STROKE_HEATED_ZONE)  // ~355.4 turns
```

### Position Validation (SafetyManager::check())

```cpp
// Hard travel limits
if (current_position < POS_HOME - 5.0f || current_position > POS_BOTTOM_MAX + 5.0f) {
    triggerHalt(ERR_HARD_LIMIT);
    return false;
}
```

### Position Monitoring Strategy
- **Continuous monitoring:** SafetyManager checks position every loop() iteration
- **Hard limits:** ±5 turn buffer around physical endstops (prevents mechanical damage)
- **Encoder validation:** Homing sequence establishes encoder zero point
- **Drift detection:** If encoder drifts >5 turns from expected, trigger ERR_HARD_LIMIT

---

## Testing Procedures

### 1. Bottom Endstop Collision Test

**Objective:** Verify bottom collision detection works correctly

**Procedure:**
1. Home machine (establish encoder zero)
2. Move to READY_TO_INJECT state
3. Manually trigger bottom endstop (short GPIO 18 to GND)
4. Verify error code 0x0007 logged
5. Verify motor stopped immediately
6. Press Center button to acknowledge error
7. Verify system returns to INIT_HOT_NOT_HOMED
8. Re-home to clear error

**Expected Result:** Motor stops, error logged, collision detected

### 2. Top Endstop Collision Test

**Objective:** Verify top collision detection works correctly (except during homing)

**Procedure:**
1. Home machine (intentional top endstop contact allowed)
2. Move to REFILL state
3. Enter PURGE_ZERO state (manual control)
4. Manually push plunger upward (Upper button) until top endstop hit
5. Verify error code 0x0008 logged (not during homing)
6. Verify motor stopped immediately
7. Press Center button to acknowledge error
8. Verify system returns to INIT_HOT_NOT_HOMED
9. Re-home to clear error

**Expected Result:** Motor stops, error logged, collision detected (except during homing)

### 3. End-of-Day Toggle Test

**Objective:** Verify flags.endOfDay toggle changes cycle flow

**Procedure:**
1. Complete full injection cycle (INJECT → HOLD → RELEASE → CONFIRM_REMOVAL)
2. Verify system returns to REFILL state (47.7 turns)
3. Press Upper+Lower buttons simultaneously
4. Verify LED feedback changes (Upper/Lower buttons turn BLUE)
5. Complete another injection cycle
6. Verify system returns to READY_TO_INJECT state (~74 turns, heated zone)
7. Press Upper+Lower buttons again to toggle off
8. Verify LED feedback changes (Upper/Lower buttons turn BLACK)

**Expected Result:** Cycle flow changes based on flags.endOfDay flag

### 4. Homing Collision Exemption Test

**Objective:** Verify CTX_MOVING_FREE allows intentional endstop contact

**Procedure:**
1. Enter INIT_HOMING state
2. Verify safety context set to CTX_MOVING_FREE
3. Verify plunger retracts to top endstop without error
4. Verify top endstop hit does NOT trigger error code 0x0008
5. Verify encoder reset to 0.0 turns
6. Verify safety context restored to CTX_IDLE after homing

**Expected Result:** Top endstop hit allowed during homing, no error logged

---

## Future Enhancements

### 1. Position Drift Monitoring
- Track encoder drift over time (compare to expected positions)
- Trigger automatic re-homing if drift exceeds threshold (e.g., 2 turns)
- Log drift history for predictive maintenance

### 2. Endstop Health Check
- Monitor endstop trigger frequency (detect false triggers)
- Alert user if endstop triggers unexpectedly during idle
- Suggest endstop replacement if reliability degrades

### 3. Position Logging
- Log position history during injection cycles
- Display position trace on Display (debugging tool)
- Export position data for process optimization

### 4. Encoder Validation
- Compare encoder position to expected position (based on velocity integral)
- Detect encoder slip or mechanical binding
- Trigger error if discrepancy exceeds threshold

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-08 | Agent + Andy | Initial endstop safety strategy for collision detection and end-of-day safety |

---

**Document Status:** FINAL  
**Approval Required:** Andy (Hardware)
