#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/rmt_tx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    rmt_channel_handle_t channel;
    rmt_encoder_handle_t encoder;
    gpio_num_t gpio;
    uint8_t *pixels;
    size_t led_count;
    size_t byte_count;
    bool initialized;
} ws2812_strip_t;

bool ws2812_init(ws2812_strip_t *strip, gpio_num_t gpio, uint8_t *pixel_buf, size_t led_count);
void ws2812_set_pixel(ws2812_strip_t *strip, int index, uint8_t r, uint8_t g, uint8_t b);
void ws2812_refresh(ws2812_strip_t *strip);
void ws2812_clear(ws2812_strip_t *strip);

#ifdef __cplusplus
}
#endif
