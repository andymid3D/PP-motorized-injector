#include "temperature.h"
#include "esp_adc/adc_oneshot.h"
#include "config.h"

static bool adc_initialized = false;
static adc_oneshot_unit_handle_t adc_handle = nullptr;

static void init_adc() {
    if (adc_initialized) return;

    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = ADC_UNIT_1;
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    if (adc_oneshot_new_unit(&unit_cfg, &adc_handle) != ESP_OK) {
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten = ADC_ATTEN_DB_11;
    chan_cfg.bitwidth = ADC_BITWIDTH_12;
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_cfg); // GPIO36

    adc_initialized = true;
}

int readTemperatureC() {
    init_adc();
    static int lastValidTemp = 20;
    if (!adc_initialized) return lastValidTemp;

    long sum = 0;
    const int samples = 10;
    for (int i = 0; i < samples; i++) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &raw) == ESP_OK) {
            sum += raw;
        }
    }
    float avgAdc = sum / (float)samples;
    float voltage = (avgAdc / 4095.0f) * 3.3f;
    float currentTemp = voltage / 0.01f;

    if (currentTemp < 5.0f) return lastValidTemp;
    static float smoothedTemp = 0;
    if (smoothedTemp == 0) smoothedTemp = currentTemp;
    smoothedTemp = (smoothedTemp * 0.80f) + (currentTemp * 0.20f);
    lastValidTemp = (int)smoothedTemp;
    return lastValidTemp;
}
