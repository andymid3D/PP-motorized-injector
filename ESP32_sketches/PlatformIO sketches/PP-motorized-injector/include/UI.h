#ifndef UI_H
#define UI_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Bounce2.h>
#include "config.h"

/**
 * UI Module - Buttons & LEDs
 * Simple wrapper around existing Bounce2 buttons and LED functions
 */

// Button objects (declared globally in main, exposed here)
extern Bounce2::Button btnCenter;
extern Bounce2::Button btnUpper;
extern Bounce2::Button btnLower;

// LED objects (declared globally in main, exposed here)
extern Adafruit_NeoPixel ledsButtons;
extern Adafruit_NeoPixel ledsRing;

// ===== LED Helpers =====
inline void setRingColor(uint32_t color) {
    ledsRing.fill(color);
    ledsRing.show();
}

inline void setButtonLeds(uint32_t color) {
    ledsButtons.fill(color);
    ledsButtons.show();
}

// Set individual button LED colors (upper, center, lower)
inline void setButtonLeds(uint32_t upperColor, uint32_t centerColor, uint32_t lowerColor) {
    ledsButtons.setPixelColor(0, upperColor);   // Upper button
    ledsButtons.setPixelColor(1, centerColor);  // Center button
    ledsButtons.setPixelColor(2, lowerColor);   // Lower button
    ledsButtons.show();
}

inline void setAllLeds(uint32_t ringColor, uint32_t btnColor) {
    setRingColor(ringColor);
    setButtonLeds(btnColor);
}

// Convenience colors
inline void indicateReady() {
    setAllLeds(GREEN_RGB, GREEN_RGB);
}

inline void indicateError() {
    setAllLeds(RED_RGB, RED_RGB);
}

inline void indicateRunning() {
    setAllLeds(YELLOW_RGB, YELLOW_RGB);
}

inline void indicateWaiting() {
    setAllLeds(BLUE_RGB, BLUE_RGB);
}

#endif // UI_H
