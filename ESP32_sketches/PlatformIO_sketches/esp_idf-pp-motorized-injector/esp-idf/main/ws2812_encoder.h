#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t resolution; /*!< Encoder resolution, in Hz */
} ws2812_encoder_config_t;

esp_err_t rmt_new_ws2812_encoder(const ws2812_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

#ifdef __cplusplus
}
#endif
