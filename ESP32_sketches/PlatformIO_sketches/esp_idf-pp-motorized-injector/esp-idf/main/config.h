#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// =============================
// Build / Debug
// =============================
#ifndef APP_DEBUG
#define APP_DEBUG 1
#endif

#ifndef APP_DEBUG_CONSOLE
#define APP_DEBUG_CONSOLE 1
#endif

#ifndef APP_DISPLAY_UART
#define APP_DISPLAY_UART 0
#endif

// =============================
// GPIO Pins
// =============================
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

#define PIN_BTN_UPPER           25
#define PIN_BTN_CENTER          26
#define PIN_BTN_LOWER           27
#define PIN_LED_RING            32
#define PIN_LED_BUTTONS         33

// UART Display
#define DISPLAY_BROADCAST_INTERVAL_MS  100
#define DISPLAY_BAUD_RATE              115200

// =============================
// LED Configuration (placeholder)
// =============================
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

// =============================
// Mechanical Constants
// =============================
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
#define GENERAL_MACHINE_CURRENT_LIMIT 31.0f

#define IGNORE_NOZZLE_BLOCK     true
#define TEMP_MIN_MOVE           20
#define TEMP_CRITICAL           15
#define TEMP_CRITICAL_RECOVER   17
#define DEBOUNCE_MS_SAFETY      150

#define CAN_COMMAND_GAP_MS          10
#define ERROR_CLEAR_DELAY_MS        50
#define BROADCAST_STALE_TIMEOUT_MS  100
#define BROADCAST_GRACE_AFTER_POWER_MS 1000
#define CONTACTOR_MIN_OFF_MS        5000

#define RTR_TIMEOUT_MS              5
#define RTR_RETRY_COUNT             0

// =============================
// Cyclic Timing System (kept for parity)
// =============================
#define CMD_WINDOW_OFFSET_US        4000
#define CMD_WINDOW_DURATION_US      1000
#define CMD_GAP_US                  10000
#define MOVEMENT_IQ_THRESHOLD_MA    50
#define MOVEMENT_POS_THRESHOLD_TICKS 10
#define MOVEMENT_TIMEOUT_MS         100
#define TIMING_BASE_WINDOW_US       5000
#define TIMING_ADAPTIVE_FACTOR      0.8f
#define TIMING_MIN_WINDOW_US        2000
#define TIMING_MAX_WINDOW_US        8000

// =============================
// Motor Control Parameters
// =============================
#define TRAP_TRAJ_GENERAL_ACCEL_DECEL  50.0f

#define HOMING_FAST_VEL         -12.5f
#define HOMING_FAST_CURRENT     GENERAL_MACHINE_CURRENT_LIMIT
#define HOMING_BACKOFF_VEL      2.5f
#define HOMING_BACKOFF_DURATION 1500
#define HOMING_BACKOFF_DIST     15.0f
#define HOMING_APPROACH_VEL     -2.5f
#define HOMING_STOP_THRESHOLD   0.05f

#define REFILL_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT
#define REFILL_TRAP_VEL_LIMIT        15.0f
#define REFILL_ACCEL            TRAP_TRAJ_GENERAL_ACCEL_DECEL
#define REFILL_DECEL            TRAP_TRAJ_GENERAL_ACCEL_DECEL
#define REFILL_CURRENT_LIMIT    GENERAL_MACHINE_CURRENT_LIMIT
#define REFILL_FROM_BARREL_END_TIMEOUT_MS  ((uint32_t)((POS_BOTTOM_MAX / REFILL_TRAP_VEL_LIMIT) * 1000.0f + 2000.0f))

#define COMPRESS_TRAVEL_VEL_LIMIT    12.5f
#define COMPRESS_TRAVEL_CURRENT      GENERAL_MACHINE_CURRENT_LIMIT
#define COMPRESS_TRAVEL_TORQUE       10.0f
#define COMPRESS_CONTACT_IQ_THRESHOLD 8.0f
#define COMPRESS_TRAVEL_TIMEOUT_MS   10000
#define COMPRESS_RAMP_TARGET         15.0f
#define COMPRESS_RAMP_DURATION       2.0f
#define COMPRESS_CONTACT_CURRENT     GENERAL_MACHINE_CURRENT_LIMIT
#define COMPRESS_RAMP_TIMEOUT_MS     15000
#define COMPRESS_MICRO_VEL_LIMIT     12.0f
#define COMPRESS_MICRO_CURRENT       GENERAL_MACHINE_CURRENT_LIMIT

#define READY_MICRO_INTERVAL_MS      30000
#define READY_MICRO_DURATION_MS      2000

#define PURGE_VEL_UP            -2.0f
#define PURGE_VEL_DOWN          2.0f
#define PURGE_VEL_LIMIT         5.0f
#define PURGE_CURRENT_LIMIT     GENERAL_MACHINE_CURRENT_LIMIT

#define ANTIDRIP_VEL            -2.0f
#define ANTIDRIP_VEL_LIMIT      5.0f
#define ANTIDRIP_CURRENT_LIMIT  GENERAL_MACHINE_CURRENT_LIMIT
#define ANTIDRIP_TIMEOUT_MS     15000

#define INJECT_FILL_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT
#define INJECT_FILL_TRAP_VEL_LIMIT        20.0f
#define INJECT_FILL_ACCEL       TRAP_TRAJ_GENERAL_ACCEL_DECEL
#define INJECT_FILL_DECEL       TRAP_TRAJ_GENERAL_ACCEL_DECEL
#define INJECT_FILL_CURRENT     GENERAL_MACHINE_CURRENT_LIMIT
#define INJECT_FILL_TIMEOUT_MS  30000
#define INJECT_PACK_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT
#define INJECT_PACK_TRAP_VEL_LIMIT        10.0f
#define INJECT_PACK_ACCEL       TRAP_TRAJ_GENERAL_ACCEL_DECEL
#define INJECT_PACK_DECEL       TRAP_TRAJ_GENERAL_ACCEL_DECEL
#define INJECT_PACK_CURRENT     GENERAL_MACHINE_CURRENT_LIMIT
#define INJECT_VEL_THRESHOLD    0.1f
#define INJECT_POS_TOLERANCE    1.0f
#define INJECT_STABLE_TIME_MS   500

#define RELEASE_DIST            -2.5f
#define RELEASE_CONTROLLER_VEL_LIMIT  MACHINE_MAX_VEL_LIMIT
#define RELEASE_TRAP_VEL_LIMIT        20.0f
#define RELEASE_ACCEL           TRAP_TRAJ_GENERAL_ACCEL_DECEL
#define RELEASE_DECEL           TRAP_TRAJ_GENERAL_ACCEL_DECEL
#define RELEASE_CURRENT_LIMIT   GENERAL_MACHINE_CURRENT_LIMIT
#define RELEASE_TIMEOUT_MS      2000

// =============================
// Module IDs (aligned with InjectorStates)
// =============================
#define MODULE_ERROR_STATE              0
#define MODULE_INIT_HEATING             1
#define MODULE_INIT_HOT_NOT_HOMED       2
#define MODULE_INIT_HOMING              3
#define MODULE_INIT_HOMED_ENCODER_ZEROED 4
#define MODULE_REFILL                   5
#define MODULE_COMPRESSION              6
#define MODULE_READY_TO_INJECT          7
#define MODULE_PURGE_ZERO               8
#define MODULE_ANTIDRIP                 9
#define MODULE_INJECT                   10
#define MODULE_HOLD_INJECTION           11
#define MODULE_RELEASE                  12
#define MODULE_CONFIRM_MOULD_REMOVAL    13
#define MODULE_SAFETY_MANAGER          255

// =============================
// Data Structures
// =============================
struct MachineFlags {
    bool endOfDay;
    bool initialHomingDone;
    bool barrelHeated;
    bool calibrationDone;
};

#endif // APP_CONFIG_H
