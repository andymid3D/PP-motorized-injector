#include "debounced_input.h"
#include "time_utils.h"

void DebouncedInput::begin(gpio_num_t pin, bool active_low, uint32_t debounce_ms, bool pullup) {
    pin_ = pin;
    active_low_ = active_low;
    debounce_ms_ = debounce_ms;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << pin_);
    io_conf.pull_up_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    state_ = readRaw();
    last_state_ = state_;
    last_change_ms_ = time_utils::millis();
}

bool DebouncedInput::readRaw() const {
    if (pin_ == GPIO_NUM_NC) return false;
    int level = gpio_get_level(pin_);
    return active_low_ ? (level == 0) : (level != 0);
}

void DebouncedInput::update() {
    fell_ = false;
    rose_ = false;

    bool raw = readRaw();
    if (raw != last_state_) {
        last_change_ms_ = time_utils::millis();
        last_state_ = raw;
    }

    if ((time_utils::millis() - last_change_ms_) >= debounce_ms_) {
        if (raw != state_) {
            state_ = raw;
            if (state_) fell_ = true; else rose_ = true;
        }
    }
}

bool DebouncedInput::isPressed() const {
    return state_;
}

bool DebouncedInput::fell() { return fell_; }
bool DebouncedInput::rose() { return rose_; }
