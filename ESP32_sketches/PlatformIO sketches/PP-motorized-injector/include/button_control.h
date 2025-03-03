#ifndef BUTTON_CONTROL_H
#define BUTTON_CONTROL_H

#include <Bounce2.h>
#include <Adafruit_NeoPixel.h>

// Function declarations
void setupButtons();
void updateButtons();
void buttonLEDsColors(uint32_t color1, uint32_t color2, uint32_t color3);
void outputButtonLEDsColors();

extern Bounce2::Button downButton;
extern Bounce2::Button upButton;
extern Bounce2::Button selectButton;
extern Bounce2::Button topEndstop;
extern Bounce2::Button bottomEndstop;
extern Bounce2::Button barrelEndstop;
extern Bounce2::Button EMERGENCYstop;

extern Adafruit_NeoPixel keypadleds;
extern Adafruit_NeoPixel ringleds;

#endif // BUTTON_CONTROL_H