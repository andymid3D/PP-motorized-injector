#include "debug_console.h"

#if APP_DEBUG_CONSOLE

#include "driver/uart.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "DEBUG_CONSOLE";
static CanBus* g_motor = nullptr;
static char rx_buf[128];
static size_t rx_len = 0;

static void handle_command(const char* cmd) {
    if (!cmd || cmd[0] == '\0') return;

    if (strcmp(cmd, "help") == 0) {
        ESP_LOGI(TAG, "Commands: help, estop, clear_errors, axis_state <n>");
        return;
    }

    if (strcmp(cmd, "estop") == 0) {
        if (g_motor) g_motor->estop();
        ESP_LOGI(TAG, "ESTOP sent");
        return;
    }

    if (strcmp(cmd, "clear_errors") == 0) {
        if (g_motor) g_motor->clearErrors();
        ESP_LOGI(TAG, "CLEAR_ERRORS sent");
        return;
    }

    if (strncmp(cmd, "axis_state", 10) == 0) {
        int state = atoi(cmd + 10);
        if (g_motor) g_motor->setAxisState(static_cast<odrive_can::AxisState>(state));
        ESP_LOGI(TAG, "SET_AXIS_STATE %d", state);
        return;
    }

    ESP_LOGW(TAG, "Unknown command: %s", cmd);
}

void debug_console_init(CanBus* motor) {
    g_motor = motor;
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 2048, 0, 0, nullptr, 0);
    ESP_LOGI(TAG, "Debug console ready (UART0)");
}

void debug_console_poll() {
    uint8_t c = 0;
    int len = uart_read_bytes(UART_NUM_0, &c, 1, 0);
    if (len <= 0) return;

    if (c == '\n' || c == '\r') {
        if (rx_len > 0) {
            rx_buf[rx_len] = '\0';
            handle_command(rx_buf);
            rx_len = 0;
        }
    } else {
        if (rx_len < sizeof(rx_buf) - 1) {
            rx_buf[rx_len++] = (char)c;
        } else {
            rx_len = 0;
        }
    }
}

#endif // APP_DEBUG_CONSOLE
