#include "hx711.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

void HX711::begin(int dout_pin, int sck_pin) {
    dout_pin_ = dout_pin;
    sck_pin_ = sck_pin;
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << dout_pin_);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << sck_pin_);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level((gpio_num_t)sck_pin_, 0);
}

bool HX711::is_ready() const {
    if (dout_pin_ < 0) return false;
    return gpio_get_level((gpio_num_t)dout_pin_) == 0;
}

int32_t HX711::read() {
    if (dout_pin_ < 0 || sck_pin_ < 0) return 0;

    int32_t value = 0;
    // Read 24 bits
    for (int i = 0; i < 24; ++i) {
        gpio_set_level((gpio_num_t)sck_pin_, 1);
        esp_rom_delay_us(1);
        value = (value << 1) | (gpio_get_level((gpio_num_t)dout_pin_) & 0x1);
        gpio_set_level((gpio_num_t)sck_pin_, 0);
        esp_rom_delay_us(1);
    }

    // Gain pulse (128x)
    gpio_set_level((gpio_num_t)sck_pin_, 1);
    esp_rom_delay_us(1);
    gpio_set_level((gpio_num_t)sck_pin_, 0);
    esp_rom_delay_us(1);

    // Sign extend 24-bit
    if (value & 0x800000) {
        value |= ~0xFFFFFF;
    }

    return value - offset_;
}

void HX711::tare() {
    // Simple tare: average 10 readings
    int64_t sum = 0;
    int samples = 10;
    for (int i = 0; i < samples; ++i) {
        while (!is_ready()) {
            esp_rom_delay_us(10);
        }
        sum += read();
    }
    offset_ = static_cast<int32_t>(sum / samples);
}
