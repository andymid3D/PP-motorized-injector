#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// 0. DEBUG CONFIGURATION (SET TO 0 FOR PRODUCTION)
// ==========================================
// When DEBUG_ENABLED = 1: All debug messages compiled and sent to Serial
// When DEBUG_ENABLED = 0: All debug messages compiled out (zero overhead)
// Change to 0 for production builds to eliminate serial message overhead
#define DEBUG_ENABLED 1

// ==========================================
// 0B. PHASE 1 TEST MODE (ISOLATED MODULE TESTING)
// ==========================================
// When TEST_MODE_PHASE1 = true: Bypass FSM, run only Phase 1 module tests
// - Minimal loop() for clean performance baseline
// - loopTimer measures without FSM noise
// - Allows measurement of ISR overhead (<5µs)
// When TEST_MODE_PHASE1 = false: Normal FSM operation
#define TEST_MODE_PHASE1  true  // TEMPORARY: Set false after Phase 1 complete

#if DEBUG_ENABLED
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINTF(...)
#endif

// ==========================================
// 1. PIN DEFINITIONS (HAL)
// ==========================================
#define PIN_CAN_RX              22  
#define PIN_CAN_TX              23  
#define PIN_CONTACTOR           13  
#define PIN_ESTOP               21   
#define PIN_ENDSTOP_TOP         19  
#define PIN_ENDSTOP_BOTTOM      18  
#define PIN_ENDSTOP_BARREL      5   
#define PIN_TEMP_ANALOG         36  
#define PIN_HX711_DAT           15
#define PIN_HX711_CLK           4
#define PIN_UART_TX             17
#define PIN_UART_RX             16

// Display Communications (UART2)
#define DISPLAY_BROADCAST_INTERVAL_MS  100     // Encoder position broadcast interval (ms)
#define DISPLAY_BAUD_RATE              115200  // UART2 baud rate (Display ↔ Controller)

#define PIN_BTN_UPPER           25
#define PIN_BTN_CENTER          26
#define PIN_BTN_LOWER           27
#define PIN_LED_RING            32
#define PIN_LED_BUTTONS         33

// ==========================================
// 2. LED CONFIGURATION
// ==========================================
#define LED_COUNT_RING          35
#define LED_COUNT_BUTTONS       3
#define GREEN_RGB               0x008000
#define RED_RGB                 0xFF0000
#define YELLOW_RGB              0xFF8C00
#define BLUE_RGB                0x0000FF
#define BLACK_RGB               0x000000
#define WHITE_RGB               0xFFFFFF
#define LED_BRIGHT_LOW          50
#define LED_BRIGHT_HIGH         200

// UI Timing
#define UI_BUTTON_TOGGLE_DELAY_MS  250      // Delay after toggle operations (ms)

// ==========================================
// 3. MECHANICAL CONSTANTS & GLOBAL LIMITS
// ==========================================
#define TURNS_PER_CM_LINEAR     5.305f
#define TURNS_PER_CM3_VOL       0.99925f 

// Position Definitions (Turns)
#define POS_HOME                0.0f        // Home position at top endstop
#define OFFSET_REFILL_GAP       47.746f     // Distance from home to refill rest position
#define OFFSET_COLD_ZONE        42.441f     // Cold zone below refill gap
#define POS_HEATED_ZONE_START   (OFFSET_REFILL_GAP + OFFSET_COLD_ZONE)  // 90.187 turns
#define STROKE_HEATED_ZONE      265.25f     // Length of heated zone
#define POS_BOTTOM_MAX          (POS_HEATED_ZONE_START + STROKE_HEATED_ZONE)  // ~355.4 turns

// ODrive Configuration
#define ODRIVE_NODE_ID          0           // CAN node ID for ODrive
#define INVERT_MOTOR_DIR        false       // Encoder A & B swapped: +47 = down (inject)
#define MACHINE_MAX_VEL_LIMIT   25.0f       // Maximum velocity limit (turns/sec) - used for controller.vel_limit in TRAP_TRAJ moves

// Safety & Debugging
#define IGNORE_NOZZLE_BLOCK     true        // Disables pressure sensor errors for testing
#define TEMP_MIN_MOVE           20          // Minimum temperature to allow movement (°C)
#define TEMP_CRITICAL           15          // Critical low temperature (°C)
#define DEBOUNCE_MS_SAFETY      150         // Safety input debounce time (ms)

// CAN Bus Timing
#define CAN_COMMAND_GAP_MS          50       // Minimum gap between CAN commands (prevents buffer overflow)
#define ERROR_CLEAR_DELAY_MS        50      // Delay after error clear command (ms)
#define BROADCAST_STALE_TIMEOUT_MS  100     // No broadcast = CAN failure (matches heartbeat interval)

// RTR Response Tracking (DISABLED - retry storm causes ODrive crashes)
#define RTR_TIMEOUT_MS              5       // Per-attempt timeout for RTR response (ms)
#define RTR_RETRY_COUNT             0       // DISABLED: Retries cause CAN buffer overflow (was 3)

// ==========================================
// 4. MOTOR CONTROL PARAMETERS BY STATE
// ==========================================
// Note: ALL negative velocities/positions defined here with negative sign
//       Code NEVER uses negative signs - always references these constants

// ------------------------------------------------------------
// STATE: HOMING (Homing.cpp)
// ------------------------------------------------------------
// Control: Various modes during sequence
// Phase 1: Fast retract to top endstop
#define HOMING_FAST_VEL         -12.5f      // Velocity up to top endstop (negative = up)
#define HOMING_FAST_CURRENT     15.0f       // Current limit during fast retract

// Phase 2: Backoff from endstop
#define HOMING_BACKOFF_VEL      2.5f        // Velocity down during backoff (positive = down)
#define HOMING_BACKOFF_DURATION 1500        // Backoff duration (ms)
#define HOMING_BACKOFF_DIST     15.0f       // Backoff distance (turns, for reference)

// Phase 3: Slow approach to endstop
#define HOMING_APPROACH_VEL     -2.5f       // Velocity up to endstop (negative = up)
#define HOMING_STOP_THRESHOLD   0.05f       // Velocity threshold for stop detection (turns/sec)

// ------------------------------------------------------------
// STATE: REFILL (Refill.cpp)
// ------------------------------------------------------------
// Control: Mode 3 (Position), Input 5 (TRAP_TRAJ)
// Target: OFFSET_REFILL_GAP (47.746 turns from home)
// Controller uses MACHINE_MAX_VEL_LIMIT (25 rps) for maximum control authority
#define REFILL_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT  // Controller limit (for setMotorLimits)
#define REFILL_TRAP_VEL_LIMIT        15.0f  // Trajectory vel limit (turns/sec) - actual movement speed
#define REFILL_ACCEL            20.0f       // Acceleration (turns/sec²)
#define REFILL_DECEL            20.0f       // Deceleration (turns/sec²)
#define REFILL_CURRENT_LIMIT    15.0f       // Current limit (Amps)
// Timeout: Worst case = full barrel length / speed + 2s margin
// Full length: POS_HOME (0) to POS_BOTTOM_MAX (~355.4 turns)
#define REFILL_TIMEOUT_MS       ((uint32_t)((POS_BOTTOM_MAX / REFILL_TRAP_VEL_LIMIT) * 1000.0f + 2000.0f))  // ~26 seconds

// ------------------------------------------------------------
// STATE: COMPRESSION (Compression.cpp)
// ------------------------------------------------------------
// Control: Mode 1 (Torque), Input 6 (TORQUE_RAMP)
// Two phases: TRAVEL_DOWN (find contact) → TORQUE_RAMP (compress)

// TRAVEL_DOWN Phase (torque mode, no resistance = continuous movement)
#define COMPRESS_TRAVEL_VEL_LIMIT    12.5f  // Velocity limit (turns/sec)
#define COMPRESS_TRAVEL_CURRENT      15.0f  // Current limit (Amps)
#define COMPRESS_TRAVEL_TORQUE       10.0f  // Torque setpoint (Amps, torque_constant=1)
#define COMPRESS_CONTACT_IQ_THRESHOLD 8.0f  // Current threshold for contact detection (Amps)
#define COMPRESS_TRAVEL_TIMEOUT_MS   10000  // Timeout if no contact (ms)

// TORQUE_RAMP Phase (after contact detected)
#define COMPRESS_RAMP_TARGET         15.0f   // Target compression torque (Amps)
#define COMPRESS_RAMP_DURATION       2.0f   // Ramp duration to target (seconds)
#define COMPRESS_CONTACT_CURRENT     25.0f  // Current limit after contact (Amps)
#define COMPRESS_RAMP_TIMEOUT_MS     15000  // Maximum time in torque ramp (ms)

// MODE_2 (Micro Compression in ReadyToInject)
#define COMPRESS_MICRO_VEL_LIMIT     12.0f  // Velocity limit for micro (turns/sec)
#define COMPRESS_MICRO_CURRENT       10.0f   // Current limit for micro (Amps)

// ------------------------------------------------------------
// STATE: READY_TO_INJECT (ReadyToInject.cpp)
// ------------------------------------------------------------
// Control: Idle with periodic micro-compression
// Micro-compression uses Compression module MODE_2
#define READY_MICRO_INTERVAL_MS      30000  // Interval between micro-compressions (ms)
#define READY_MICRO_DURATION_MS      2000   // Duration of micro-compression ramp (ms)

// ------------------------------------------------------------
// STATE: PURGE_ZERO (PurgeZero.cpp)
// ------------------------------------------------------------
// Control: Mode 2 (Velocity), Input 1 (PASSTHROUGH)
// Manual button-controlled movement
#define PURGE_VEL_UP            -2.0f       // Velocity for Upper button (negative = up, turns/sec)
#define PURGE_VEL_DOWN          2.0f        // Velocity for Lower button (positive = down, turns/sec)
#define PURGE_VEL_LIMIT         5.0f        // Maximum velocity (turns/sec)
#define PURGE_CURRENT_LIMIT     10.0f        // Current limit (Amps)

// ------------------------------------------------------------
// STATE: ANTIDRIP (AntiDrip.cpp)
// ------------------------------------------------------------
// Control: Mode 2 (Velocity), Input 1 (PASSTHROUGH)
// Slow upward retract to prevent drip while placing mould
#define ANTIDRIP_VEL            -2.0f       // Retract velocity (negative = up, turns/sec)
#define ANTIDRIP_VEL_LIMIT      5.0f        // Velocity limit (turns/sec)
#define ANTIDRIP_CURRENT_LIMIT  10.0f        // Current limit (Amps)
#define ANTIDRIP_TIMEOUT_MS     15000       // User timeout to place mould (ms)

// ------------------------------------------------------------
// STATE: INJECT (Injection.cpp)
// ------------------------------------------------------------
// Control: Mode 3 (Position), Input 5 (TRAP_TRAJ)
// Two phases: FILLING → PACKING (auto-transition)
// Controller uses MACHINE_MAX_VEL_LIMIT (25 rps) for maximum control authority

// FILLING Phase
#define INJECT_FILL_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT  // Controller limit (for setMotorLimits)
#define INJECT_FILL_TRAP_VEL_LIMIT        20.0f  // Trajectory vel limit (turns/sec) - actual movement speed
#define INJECT_FILL_ACCEL       20.0f       // Acceleration (turns/sec²) - uses mould-specific from actualMouldParams
#define INJECT_FILL_DECEL       20.0f       // Deceleration (turns/sec²) - uses mould-specific from actualMouldParams
#define INJECT_FILL_CURRENT     31.0f       // Current limit (Amps)
#define INJECT_FILL_TIMEOUT_MS  30000       // Maximum fill time (ms)

// PACKING Phase (holding pressure)
#define INJECT_PACK_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT  // Controller limit (for setMotorLimits)
#define INJECT_PACK_TRAP_VEL_LIMIT        10.0f  // Trajectory vel limit (turns/sec) - actual movement speed
#define INJECT_PACK_ACCEL       10.0f       // Acceleration (turns/sec²) - uses mould-specific from actualMouldParams
#define INJECT_PACK_DECEL       10.0f       // Deceleration (turns/sec²) - uses mould-specific from actualMouldParams
#define INJECT_PACK_CURRENT     30.0f       // Current limit (Amps)
// Pack duration from actualMouldParams.packTime

// Auto-transition detection (FILLING → PACKING)
#define INJECT_VEL_THRESHOLD    0.1f        // Velocity threshold for phase change (turns/sec)
#define INJECT_POS_TOLERANCE    1.0f        // Position tolerance for phase change (turns)
#define INJECT_STABLE_TIME_MS   500         // Time velocity must be stable (ms)

// ------------------------------------------------------------
// STATE: HOLD_PACKING (Injection.cpp)
// ------------------------------------------------------------
// Control: Mode 2 (Velocity), Input 1 (PASSTHROUGH) - holding position
// Currently just holds final position from PACKING phase

// ------------------------------------------------------------
// STATE: RELEASE (main.cpp)
// ------------------------------------------------------------
// Control: Mode 3 (Position), Input 5 (TRAP_TRAJ)
// Fast upward unload of mould
// Controller uses MACHINE_MAX_VEL_LIMIT (25 rps) for maximum control authority
#define RELEASE_DIST            -2.5f       // Distance to move (negative = up, turns)
#define RELEASE_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT  // Controller limit (for setMotorLimits)
#define RELEASE_TRAP_VEL_LIMIT        20.0f  // Trajectory vel limit (turns/sec) - actual movement speed
#define RELEASE_ACCEL           40.0f       // Acceleration (turns/sec²)
#define RELEASE_DECEL           40.0f       // Deceleration (turns/sec²)
#define RELEASE_CURRENT_LIMIT   20.0f       // Current limit (Amps)
#define RELEASE_TIMEOUT_MS      2000        // Auto-transition timeout (ms)

// ------------------------------------------------------------
// STATE: CONFIRM_MOULD_REMOVAL (main.cpp)
// ------------------------------------------------------------
// No motor movement - just button press to return to REFILL or READY_TO_INJECT

// ==========================================
// 5. DATA STRUCTURES
// ==========================================
struct MachineFlags {                  
    bool endOfDay;           
    bool initialHomingDone;
    bool barrelHeated;
    bool calibrationDone; // Added to track State 7

};


#endif