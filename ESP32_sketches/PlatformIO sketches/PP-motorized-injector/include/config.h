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
#define INVERT_MOTOR_DIR        true    // <--- FIXED: Inverted
#define IGNORE_NOZZLE_BLOCK     true    // <--- NEW: Disables Error 3 for testing


// Speeds (Turns/Sec)
#define SPEED_PURGE             1.0f   
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

// Torque / Pressure 
#define TORQUE_COMPRESSION_HOLD 5.0f   
#define PRESSURE_BLOCK_MIN      50000  

// Timing & Temp
#define TIME_ANTIDRIP_MAX       15000  
#define TIME_AUTO_COMPRESS      30000  
#define TEMP_MIN_MOVE           16     
#define TEMP_CRITICAL           13     
#define DEBOUNCE_MS_SAFETY      150

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