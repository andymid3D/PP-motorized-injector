#ifndef DEBOUNCED_INPUT_H
#define DEBOUNCED_INPUT_H

#include <cstdint>
#include "driver/gpio.h"

class DebouncedInput {
public:
    void begin(gpio_num_t pin, bool active_low, uint32_t debounce_ms, bool pullup);
    void update();

    bool readRaw() const;
    bool isPressed() const;
    bool fell();
    bool rose();

private:
    gpio_num_t pin_ = GPIO_NUM_NC;
    bool active_low_ = true;
    bool state_ = false;
    bool last_state_ = false;
    bool fell_ = false;
    bool rose_ = false;
    uint32_t debounce_ms_ = 20;
    uint64_t last_change_ms_ = 0;
};

#endif // DEBOUNCED_INPUT_H
