#include "ws2812.h"
#include "ws2812_encoder.h"
#include "esp_log.h"
#include <string.h>

#define WS2812_RESOLUTION_HZ     (10 * 1000 * 1000)
#define WS2812_MEM_BLOCK_SYMBOLS 64
#define WS2812_TX_QUEUE_DEPTH    2

static const char *TAG = "ws2812";

bool ws2812_init(ws2812_strip_t *strip, gpio_num_t gpio, uint8_t *pixel_buf, size_t led_count)
{
    if (!strip || !pixel_buf || led_count == 0) {
        return false;
    }

    memset(strip, 0, sizeof(*strip));
    strip->gpio = gpio;
    strip->pixels = pixel_buf;
    strip->led_count = led_count;
    strip->byte_count = led_count * 3;
    strip->initialized = false;

    memset(strip->pixels, 0, strip->byte_count);

    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = WS2812_RESOLUTION_HZ,
        .mem_block_symbols = WS2812_MEM_BLOCK_SYMBOLS,
        .trans_queue_depth = WS2812_TX_QUEUE_DEPTH,
    };

    if (rmt_new_tx_channel(&tx_chan_config, &strip->channel) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel");
        return false;
    }

    ws2812_encoder_config_t encoder_config = {
        .resolution = WS2812_RESOLUTION_HZ,
    };
    if (rmt_new_ws2812_encoder(&encoder_config, &strip->encoder) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create WS2812 encoder");
        rmt_del_channel(strip->channel);
        strip->channel = NULL;
        return false;
    }

    if (rmt_enable(strip->channel) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel");
        rmt_del_encoder(strip->encoder);
        rmt_del_channel(strip->channel);
        strip->channel = NULL;
        strip->encoder = NULL;
        return false;
    }

    strip->initialized = true;
    return true;
}

void ws2812_set_pixel(ws2812_strip_t *strip, int index, uint8_t r, uint8_t g, uint8_t b)
{
    if (!strip || !strip->initialized || index < 0 || (size_t)index >= strip->led_count) {
        return;
    }
    size_t offset = (size_t)index * 3;
    // WS2812 expects GRB order
    strip->pixels[offset + 0] = g;
    strip->pixels[offset + 1] = r;
    strip->pixels[offset + 2] = b;
}

void ws2812_refresh(ws2812_strip_t *strip)
{
    if (!strip || !strip->initialized) {
        return;
    }
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    if (rmt_transmit(strip->channel, strip->encoder, strip->pixels, strip->byte_count, &tx_config) != ESP_OK) {
        return;
    }
    rmt_tx_wait_all_done(strip->channel, -1);
}

void ws2812_clear(ws2812_strip_t *strip)
{
    if (!strip || !strip->initialized) {
        return;
    }
    memset(strip->pixels, 0, strip->byte_count);
    ws2812_refresh(strip);
}
