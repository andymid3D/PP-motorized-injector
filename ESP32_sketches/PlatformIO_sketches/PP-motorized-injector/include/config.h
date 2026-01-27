#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// 0. DEBUG CONFIGURATION (SET TO 0 FOR PRODUCTION)
// ==========================================
#define DEBUG_ENABLED 1

// ==========================================
// 0B. PHASE 1 TEST MODE (ISOLATED MODULE TESTING)
// ==========================================
#define TEST_MODE_PHASE1  true

// Phase 1 Test Enable Flags (individual module tests)
#define TEST_LOOPTIMER_ENABLED           false
#define TEST_RINGBUFFER_ENABLED          false
#define TEST_STRESS_QUEUE_ENABLED        false
#define TEST_BDS_STORAGE_ENABLED         false
#define TEST_BDS_INTEGRATION_ENABLED     true
#define TEST_BASELINE_TIMING_ENABLED     false
#define TEST_PROTECTED_WINDOW_ENABLED    true

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

#define DISPLAY_BROADCAST_INTERVAL_MS  100
#define DISPLAY_BAUD_RATE              115200

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

#define UI_BUTTON_TOGGLE_DELAY_MS  250

// ==========================================
// 3. MECHANICAL CONSTANTS & GLOBAL LIMITS
// ==========================================
#define TURNS_PER_CM_LINEAR     5.305f
#define TURNS_PER_CM3_VOL       0.99925f 

#define POS_HOME                0.0f
#define OFFSET_REFILL_GAP       47.746f
#define OFFSET_COLD_ZONE        42.441f
#define POS_HEATED_ZONE_START   (OFFSET_REFILL_GAP + OFFSET_COLD_ZONE)
#define STROKE_HEATED_ZONE      265.25f
#define POS_BOTTOM_MAX          (POS_HEATED_ZONE_START + STROKE_HEATED_ZONE)

#define ODRIVE_NODE_ID          0
#define INVERT_MOTOR_DIR        false
#define MACHINE_MAX_VEL_LIMIT   25.0f

#define IGNORE_NOZZLE_BLOCK     true
#define TEMP_MIN_MOVE           20
#define TEMP_CRITICAL           15
#define DEBOUNCE_MS_SAFETY      150

#define CAN_COMMAND_GAP_MS          10
#define ERROR_CLEAR_DELAY_MS        50
#define BROADCAST_STALE_TIMEOUT_MS  100

#define RTR_TIMEOUT_MS              5
#define RTR_RETRY_COUNT             0

// ==========================================
// 3.5. CYCLIC-BASED TIMING SYSTEM
// ==========================================
// Timing constants for deterministic command windows
// Aligned with 10ms cyclic message bundles (encoder + IQ current)

// Command timing windows (microseconds)
#define CMD_WINDOW_OFFSET_US        4000    // 4ms after encoder bundle start
#define CMD_WINDOW_DURATION_US      1000    // 1ms window for command execution
#define CMD_GAP_US                  10000   // 10ms gap between commands

// Movement verification thresholds
#define MOVEMENT_IQ_THRESHOLD_MA    50      // 50mA minimum IQ current change
#define MOVEMENT_POS_THRESHOLD_TICKS 10     // 10 encoder ticks minimum position change
#define MOVEMENT_TIMEOUT_MS         100     // 100ms timeout for movement detection

// Adaptive timing parameters
#define TIMING_BASE_WINDOW_US       5000    // Base timing window (5ms)
#define TIMING_ADAPTIVE_FACTOR      0.8f    // Adaptive scaling factor
#define TIMING_MIN_WINDOW_US        2000    // Minimum window (2ms)
#define TIMING_MAX_WINDOW_US        8000    // Maximum window (8ms)

// ==========================================
// 4. MOTOR CONTROL PARAMETERS BY STATE
// ==========================================
#define HOMING_FAST_VEL         -12.5f
#define HOMING_FAST_CURRENT     15.0f
#define HOMING_BACKOFF_VEL      2.5f
#define HOMING_BACKOFF_DURATION 1500
#define HOMING_BACKOFF_DIST     15.0f
#define HOMING_APPROACH_VEL     -2.5f
#define HOMING_STOP_THRESHOLD   0.05f

#define REFILL_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT
#define REFILL_TRAP_VEL_LIMIT        15.0f
#define REFILL_ACCEL            500.0f
#define REFILL_DECEL            500.0f
#define REFILL_CURRENT_LIMIT    15.0f
#define REFILL_TIMEOUT_MS       ((uint32_t)((POS_BOTTOM_MAX / REFILL_TRAP_VEL_LIMIT) * 1000.0f + 2000.0f))

#define COMPRESS_TRAVEL_VEL_LIMIT    12.5f
#define COMPRESS_TRAVEL_CURRENT      15.0f
#define COMPRESS_TRAVEL_TORQUE       10.0f
#define COMPRESS_CONTACT_IQ_THRESHOLD 8.0f
#define COMPRESS_TRAVEL_TIMEOUT_MS   10000
#define COMPRESS_RAMP_TARGET         15.0f
#define COMPRESS_RAMP_DURATION       2.0f
#define COMPRESS_CONTACT_CURRENT     25.0f
#define COMPRESS_RAMP_TIMEOUT_MS     15000
#define COMPRESS_MICRO_VEL_LIMIT     12.0f
#define COMPRESS_MICRO_CURRENT       10.0f

#define READY_MICRO_INTERVAL_MS      30000
#define READY_MICRO_DURATION_MS      2000

#define PURGE_VEL_UP            -2.0f
#define PURGE_VEL_DOWN          2.0f
#define PURGE_VEL_LIMIT         5.0f
#define PURGE_CURRENT_LIMIT     10.0f

#define ANTIDRIP_VEL            -2.0f
#define ANTIDRIP_VEL_LIMIT      5.0f
#define ANTIDRIP_CURRENT_LIMIT  10.0f
#define ANTIDRIP_TIMEOUT_MS     15000

#define INJECT_FILL_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT
#define INJECT_FILL_TRAP_VEL_LIMIT        20.0f
#define INJECT_FILL_ACCEL       500.0f
#define INJECT_FILL_DECEL       500.0f
#define INJECT_FILL_CURRENT     31.0f
#define INJECT_FILL_TIMEOUT_MS  30000
#define INJECT_PACK_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT
#define INJECT_PACK_TRAP_VEL_LIMIT        10.0f
#define INJECT_PACK_ACCEL       500.0f
#define INJECT_PACK_DECEL       500.0f
#define INJECT_PACK_CURRENT     30.0f
#define INJECT_VEL_THRESHOLD    0.1f
#define INJECT_POS_TOLERANCE    1.0f
#define INJECT_STABLE_TIME_MS   500

#define RELEASE_DIST            -2.5f
#define RELEASE_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT
#define RELEASE_TRAP_VEL_LIMIT        20.0f
#define RELEASE_ACCEL           500.0f
#define RELEASE_DECEL           500.0f
#define RELEASE_CURRENT_LIMIT   20.0f
#define RELEASE_TIMEOUT_MS      2000

// ==========================================
// 5. DATA STRUCTURES
// ==========================================
struct MachineFlags {                  
    bool endOfDay;           
    bool initialHomingDone;
    bool barrelHeated;
    bool calibrationDone;
};

#endif
