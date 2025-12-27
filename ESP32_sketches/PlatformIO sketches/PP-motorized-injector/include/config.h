#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

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

// ==========================================
// 3. MECHANICAL CONSTANTS
// ==========================================
#define TURNS_PER_CM_LINEAR     5.305f
#define TURNS_PER_CM3_VOL       0.99925f 

#define POS_HOME                0.0f
#define OFFSET_REFILL_GAP       47.746f 
#define OFFSET_COLD_ZONE        42.441f 
#define POS_HEATED_ZONE_START   (OFFSET_REFILL_GAP + OFFSET_COLD_ZONE) 
#define STROKE_HEATED_ZONE      265.25f 
#define POS_BOTTOM_MAX          (POS_HEATED_ZONE_START + STROKE_HEATED_ZONE) 

// ==========================================
// 4. PROCESS VARIABLES
// ==========================================
#define ODRIVE_NODE_ID          0      
#define VEL_LIMIT_INJECT        25.0f  

// --- MOTOR DIRECTION SETTING ---
#define INVERT_MOTOR_DIR        false   // Encoder A & B swapped: +47 = down (inject)
#define IGNORE_NOZZLE_BLOCK     true    // <--- NEW: Disables Error 3 for testing


// Speeds (Turns/Sec)
#define SPEED_PURGE             2.0f   
#define SPEED_ANTIDRIP          2.0f   // Slow decompression move
#define SPEED_FAST_MOVE         25.0f  
#define SPEED_HOMING_FAST       12.5f  
#define SPEED_HOMING_SLOW       2.5f   
#define SPEED_COMPRESS_INIT     12.5f  // <--- KEPT AS IS (User Request)
#define SPEED_COMPRESS_MIN      1.0f   
#define SPEED_RELEASE           25.0f  

// Acceleration (Turns/Sec^2)
#define ACCEL_RAMP_DEFAULT      2.0f   // Gentle acceleration
#define ACCEL_RAMP_STOP         5.0f   // Faster (but safe) stop

// Distances (Turns)
#define DIST_HOME_BACKOFF       15.0f  // <--- INCREASED: To make pullback visible
#define DIST_RELEASE_MOULD      -2.5f  
#define DIST_ANTIDRIP_REV       -5.3f  

// Homing Sequence Parameters
#define HOMING_VELOCITY_STOP_THRESHOLD  0.05f   // Turns/sec - stop detection threshold
#define HOMING_BACKOFF_VELOCITY         SPEED_HOMING_SLOW  // Down velocity during backoff
#define HOMING_BACKOFF_DURATION         1500    // Milliseconds for backoff move
#define HOMING_APPROACH_VELOCITY        -SPEED_HOMING_SLOW  // Slow approach up (negative)  

// Torque / Pressure 
#define TORQUE_COMPRESSION_HOLD 5.0f   
#define PRESSURE_BLOCK_MIN      50000  

// Timing & Temp
#define TIME_ANTIDRIP_MAX       15000  
#define TIME_AUTO_COMPRESS      30000  
#define TEMP_MIN_MOVE           16     
#define TEMP_CRITICAL           13     
#define DEBOUNCE_MS_SAFETY      150

// Timing Constants (Milliseconds)
#define TIME_ANTIDRIP_TIMEOUT   15000   // 15 seconds to decompression
#define TIME_AUTO_COMPRESS      30000   // Auto-compress after 30s idle

// CAN Bus Command Timing
// Minimum gap between consecutive CAN commands to ODrive
// Allows ODrive to process mode changes before receiving move commands
// Tuning: Reduce until inconsistency observed, then set to 2x that value
#define CAN_COMMAND_GAP_MS      50      // Milliseconds between CAN commands (reduce to 20ms for faster response)

// ==========================================
// 4.5 MOTOR CONTROL LIMITS & TRAP_TRAJ PARAMETERS
// ==========================================

// --- VELOCITY LIMITS BY STATE (turns/sec) ---
#define VEL_LIMIT_REFILL        15.0f    // Refill: moderate speed, safe return to rest
#define VEL_LIMIT_COMPRESSION   12.0f    // Compression: half max, controlled approach to contact
#define VEL_LIMIT_INJECTION     15.0f    // Injection: moderate fill speed
#define VEL_LIMIT_RELEASE       20.0f    // Release: faster unload
#define VEL_LIMIT_PURGE         5.0f     // Purge: manual control, slower for safety
#define VEL_LIMIT_ANTIDRIP      2.0f     // AntiDrip: very slow decompression

// --- CURRENT LIMITS BY STATE (Amps) ---
#define CURRENT_LIMIT_REFILL    5.0f     // Refill: low current, no load expected
#define CURRENT_LIMIT_COMPRESSION_INITIAL  7.0f  // Compression: double friction (3.4A * 2), contact detection
#define CURRENT_LIMIT_COMPRESSION_CONTACT  25.0f // Compression: full force after contact detected
#define CURRENT_LIMIT_INJECTION_FILL       10.0f // Injection fill: moderate pressure
#define CURRENT_LIMIT_INJECTION_PACK       15.0f // Injection pack: higher pressure to maintain
#define CURRENT_LIMIT_RELEASE   10.0f    // Release: moderate force for unload
#define CURRENT_LIMIT_PURGE     8.0f     // Purge: moderate for manual control
#define CURRENT_LIMIT_ANTIDRIP  5.0f     // AntiDrip: low force, gentle decompression

// --- TRAP_TRAJ PARAMETERS (turns/sec²) ---
#define TRAP_ACCEL_NORMAL       20.0f    // Normal acceleration for most moves
#define TRAP_DECEL_NORMAL       20.0f    // Normal deceleration for most moves
#define TRAP_ACCEL_SLOW         10.0f    // Careful/slow acceleration
#define TRAP_DECEL_SLOW         10.0f    // Careful/slow deceleration
#define TRAP_ACCEL_FAST         40.0f    // Fast acceleration (e.g., release)
#define TRAP_DECEL_FAST         40.0f    // Fast deceleration (e.g., release)

// --- CONTACT DETECTION (Current Monitoring) ---
#define CURRENT_FRICTION_BASELINE   3.4f    // Motor friction current (idle, no load)
#define CURRENT_CONTACT_THRESHOLD   5.0f    // Current spike indicating contact (empirical, adjust after testing)
#define CURRENT_MONITOR_INTERVAL_MS 100     // How often to check current (matches IQ broadcast rate)

// --- STATE TIMEOUTS ---
#define TIMEOUT_COMPRESSION_EMPTY_MS    5000    // Compression timeout with empty barrel (no plastic)
#define TIMEOUT_COMPRESSION_LOADED_MS   10000   // Compression timeout with plastic (if contact not detected)
#define TIMEOUT_ANTIDRIP_MS             15000   // AntiDrip user timeout (time to place mould)

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