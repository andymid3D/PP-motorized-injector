#include "leds.h"
#include "time_utils.h"
#include "esp_log.h"
#include "ws2812.h"

static const char* TAG = "LEDS";

static ws2812_strip_t strip_ring = {};
static ws2812_strip_t strip_buttons = {};
static uint8_t ring_pixels[LED_COUNT_RING * 3] = {};
static uint8_t button_pixels[LED_COUNT_BUTTONS * 3] = {};

static void apply_brightness(uint32_t color, uint8_t brightness, uint8_t& r, uint8_t& g, uint8_t& b) {
    uint8_t rr = (color >> 16) & 0xFF;
    uint8_t gg = (color >> 8) & 0xFF;
    uint8_t bb = color & 0xFF;
    r = (uint8_t)((rr * brightness) / 255);
    g = (uint8_t)((gg * brightness) / 255);
    b = (uint8_t)((bb * brightness) / 255);
}

static void set_strip_color(ws2812_strip_t* strip, int count, uint32_t color, uint8_t brightness) {
    if (!strip || !strip->initialized) return;
    uint8_t r, g, b;
    apply_brightness(color, brightness, r, g, b);
    for (int i = 0; i < count; ++i) {
        ws2812_set_pixel(strip, i, r, g, b);
    }
    ws2812_refresh(strip);
}

static void set_button_colors(uint32_t upper, uint32_t center, uint32_t lower, uint8_t brightness) {
    if (!strip_buttons.initialized) return;
    uint8_t r, g, b;

    apply_brightness(upper, brightness, r, g, b);
    ws2812_set_pixel(&strip_buttons, 0, r, g, b);

    apply_brightness(center, brightness, r, g, b);
    ws2812_set_pixel(&strip_buttons, 1, r, g, b);

    apply_brightness(lower, brightness, r, g, b);
    ws2812_set_pixel(&strip_buttons, 2, r, g, b);

    ws2812_refresh(&strip_buttons);
}

void leds_init() {
    if (!ws2812_init(&strip_ring, (gpio_num_t)PIN_LED_RING, ring_pixels, LED_COUNT_RING)) {
        ESP_LOGE(TAG, "Failed to init ring strip");
    }

    if (!ws2812_init(&strip_buttons, (gpio_num_t)PIN_LED_BUTTONS, button_pixels, LED_COUNT_BUTTONS)) {
        ESP_LOGE(TAG, "Failed to init button strip");
    }

    if (strip_ring.initialized) {
        ws2812_clear(&strip_ring);
    }
    if (strip_buttons.initialized) {
        ws2812_clear(&strip_buttons);
    }
}

void leds_update(InjectorStates state, bool endOfDay, bool anyButtonPressed) {
    uint32_t colUpper = BLACK_RGB;
    uint32_t colCenter = BLACK_RGB;
    uint32_t colLower = BLACK_RGB;
    uint32_t colRing = BLACK_RGB;

    bool blink_on = ((time_utils::millis() / 500) % 2) == 0;

    switch (state) {
        case ERROR_STATE:
            if (blink_on) { colUpper = RED_RGB; colCenter = RED_RGB; colLower = RED_RGB; colRing = RED_RGB; }
            break;
        case INIT_HEATING:
            colUpper = RED_RGB; colCenter = RED_RGB; colLower = RED_RGB; colRing = RED_RGB;
            break;
        case INIT_HOT_NOT_HOMED:
            colUpper = YELLOW_RGB; colCenter = YELLOW_RGB; colLower = YELLOW_RGB; colRing = YELLOW_RGB;
            break;
        case INIT_HOMING:
            if (blink_on) { colUpper = YELLOW_RGB; colRing = YELLOW_RGB; }
            break;
        case REFILL:
            colCenter = GREEN_RGB;
            if (endOfDay) { colUpper = BLUE_RGB; colLower = BLUE_RGB; }
            colRing = GREEN_RGB;
            break;
        case COMPRESSION:
            colUpper = RED_RGB; colLower = RED_RGB; colRing = RED_RGB;
            break;
        case READY_TO_INJECT:
            colUpper = GREEN_RGB; colCenter = YELLOW_RGB; colLower = GREEN_RGB; colRing = GREEN_RGB;
            break;
        case PURGE_ZERO:
            colUpper = YELLOW_RGB; colCenter = GREEN_RGB; colLower = YELLOW_RGB; colRing = YELLOW_RGB;
            break;
        case ANTIDRIP:
            colUpper = RED_RGB; colCenter = GREEN_RGB; colLower = GREEN_RGB; colRing = RED_RGB;
            break;
        case INJECT:
            colUpper = RED_RGB; colCenter = GREEN_RGB; colRing = RED_RGB;
            break;
        case HOLD_INJECTION:
            colUpper = RED_RGB; colCenter = GREEN_RGB; colLower = GREEN_RGB; colRing = RED_RGB;
            break;
        case RELEASE:
            colUpper = GREEN_RGB; colCenter = GREEN_RGB; colLower = GREEN_RGB; colRing = GREEN_RGB;
            break;
        case CONFIRM_MOULD_REMOVAL:
            colUpper = GREEN_RGB; colLower = GREEN_RGB; colRing = GREEN_RGB;
            break;
        default:
            break;
    }

    uint8_t brightness = anyButtonPressed ? LED_BRIGHT_HIGH : LED_BRIGHT_LOW;

    set_button_colors(colUpper, colCenter, colLower, brightness);
    set_strip_color(&strip_ring, LED_COUNT_RING, colRing, brightness);
}
