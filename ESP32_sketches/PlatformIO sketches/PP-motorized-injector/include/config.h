#ifndef CONFIG_H
#define CONFIG_H

#include <ESP32Encoder.h>

// Define pins and other constants
#define ENCODER_A_PIN 34
#define ENCODER_B_PIN 35
#define WS2812B_RING_LEDS_PIN 32
#define WS2812B_BUTTON_LEDS_PIN 33
#define BUTTON_SELECT_PIN 25
#define BUTTON_UP_PIN 26
#define BUTTON_DOWN_PIN 27
#define TEMPNozzleVSPI_SCK_CLK 14
#define TEMPNozzleVSPI_MISO_DO 12
#define TEMPNozzleVSPI_Dpin_MOSI_CS 13
#define STEPPER_STEP_PIN 23
#define STEPPER_DIR_PIN 22
#define ENDSTOP_TOP_PLUNGER_PIN 19
#define ENDSTOP_BOTTOM_PLUNGER_PIN 18
#define ENDSTOP_BARREL_PLUNGER_PIN 5
#define UART_tx_ESP32 17
#define UART_rx_ESP32 16
#define EMERGENCY_STOP_PIN 0
#define ENDSTOP_NOZZLE_BLOCK_COMPRESS_OR_PURGE_PIN 15
// #define MOULD_PRESENT_ENDSTOP_PIN 4  // still to see how to implement, to assure that nozzle is blocked by either mould or
// #define NOZZLE_BLOCK_PRESENT_ENDSTOP_PIN 2  // purge bar/cap, so can move down w/o displacing barrel

#define GREEN_RGB ((uint32_t)0x008000)
#define RED_RGB ((uint32_t)0xFF0000)
#define YELLOW_RGB ((uint32_t)0xFF8C00)
#define BLUE_RGB ((uint32_t)0x0000FF)
#define BLACK_RGB ((uint32_t)0x000000)
#define WHITE_RGB ((uint32_t)0xFFFFFF)

// Declare external variables
extern ESP32Encoder encoder;
extern int nozzleTemperature;

#endif // CONFIG_H