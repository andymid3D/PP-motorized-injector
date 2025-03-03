#include "button_control.h"
#include "config.h"

// Global variables
Bounce2::Button downButton = Bounce2::Button();
Bounce2::Button upButton = Bounce2::Button();
Bounce2::Button selectButton = Bounce2::Button();
Bounce2::Button topEndstop = Bounce2::Button();
Bounce2::Button bottomEndstop = Bounce2::Button();
Bounce2::Button barrelEndstop = Bounce2::Button();
Bounce2::Button EMERGENCYstop = Bounce2::Button();

Adafruit_NeoPixel keypadleds = Adafruit_NeoPixel(3, WS2812B_BUTTON_LEDS_PIN);
Adafruit_NeoPixel ringleds = Adafruit_NeoPixel(35, WS2812B_RING_LEDS_PIN);

void setupButtons()
{
    // Initialize buttons and LEDs here
}

void updateButtons()
{
    // Update button states here
}

void buttonLEDsColors(uint32_t color1, uint32_t color2, uint32_t color3)
{
    // Set the colors of the button LEDs here
}

void outputButtonLEDsColors()
{
    // Output the button LED colors here
}