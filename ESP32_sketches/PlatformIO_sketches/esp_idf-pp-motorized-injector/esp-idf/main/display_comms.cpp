#include "display_comms.h"

#if APP_DISPLAY_UART

#include "driver/uart.h"
#include "time_utils.h"
#include "broadcast_data_store.h"
#include "esp_log.h"
#include <cstring>
#include <cstdlib>

static const char* TAG = "DISPLAY_COMMS";

namespace DisplayComms {

static unsigned long broadcastInterval = DISPLAY_BROADCAST_INTERVAL_MS;
static char rxBuffer[512];
static size_t rxLen = 0;

struct DisplayCommsState {
    unsigned long lastEncoderBroadcast;
    InjectorStates lastStateBroadcast;
    uint16_t lastErrorBroadcast;
};

static DisplayCommsState state = {0, InjectorStates::ERROR_STATE, 0xFFFF};

extern commonInjectParams_t commonParams;
extern actualMouldParams_t currentMould;
extern fsm_state_t fsm_state;
extern MachineFlags flags;

static void uart_send(const char* msg) {
    if (!msg) return;
    uart_write_bytes(UART_NUM_2, msg, strlen(msg));
}

static const char* getStateName(InjectorStates st) {
    switch (st) {
        case INIT_HEATING: return "INIT_HEATING";
        case INIT_HOT_NOT_HOMED: return "INIT_HOT_WAIT";
        case INIT_HOMING: return "INIT_HOMING";
        case REFILL: return "REFILL";
        case COMPRESSION: return "COMPRESSION";
        case READY_TO_INJECT: return "READY_TO_INJECT";
        case PURGE_ZERO: return "PURGE_ZERO";
        case ANTIDRIP: return "ANTIDRIP";
        case INJECT: return "INJECT";
        case HOLD_INJECTION: return "HOLD_INJECTION";
        case RELEASE: return "RELEASE";
        case CONFIRM_MOULD_REMOVAL: return "CONFIRM_REMOVAL";
        case ERROR_STATE: return "ERROR_STATE";
        default: return "UNKNOWN";
    }
}

void begin() {
    uart_config_t uart_config = {};
    uart_config.baud_rate = DISPLAY_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, PIN_UART_TX, PIN_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, 1024, 0, 0, nullptr, 0);

    rxLen = 0;
    state.lastEncoderBroadcast = 0;
    state.lastStateBroadcast = InjectorStates::ERROR_STATE;
    state.lastErrorBroadcast = 0xFFFF;

    ESP_LOGI(TAG, "UART2 initialized (TX=%d RX=%d baud=%d)", PIN_UART_TX, PIN_UART_RX, DISPLAY_BAUD_RATE);
}

void update() {
    uint8_t data[64];
    int len = uart_read_bytes(UART_NUM_2, data, sizeof(data), 0);
    for (int i = 0; i < len; ++i) {
        char c = (char)data[i];
        if (c == '\n' || c == '\r') {
            if (rxLen > 0) {
                rxBuffer[rxLen] = '\0';
                parseIncomingMessage(rxBuffer);
                rxLen = 0;
            }
        } else {
            if (rxLen < sizeof(rxBuffer) - 1) {
                rxBuffer[rxLen++] = c;
            } else {
                rxLen = 0; // overflow, reset
            }
        }
    }

    unsigned long now = (unsigned long)time_utils::millis();
    if (now - state.lastEncoderBroadcast >= broadcastInterval) {
        if (fsm_state.currentState != InjectorStates::ERROR_STATE && flags.initialHomingDone) {
            BroadcastDataStore& bds = BroadcastDataStore::getInstance();
            broadcastEncoder(bds.getPosition(), bds.getVelocity());
        }
        state.lastEncoderBroadcast = now;
    }
}

void broadcastEncoder(float position, float velocity) {
    char msg[64];
    snprintf(msg, sizeof(msg), "ENC|%.3f|%.3f\n", position, velocity);
    uart_send(msg);
}

void broadcastState(InjectorStates st) {
    if (st == state.lastStateBroadcast) return;
    char msg[96];
    snprintf(msg, sizeof(msg), "STATE|%s|%llu\n", getStateName(st), (unsigned long long)time_utils::millis());
    uart_send(msg);
    state.lastStateBroadcast = st;
}

void broadcastError(uint16_t errorCode, const char* errorMsg) {
    if (errorCode == state.lastErrorBroadcast) return;
    char msg[128];
    snprintf(msg, sizeof(msg), "ERROR|0x%X|%s\n", errorCode, errorMsg ? errorMsg : "");
    uart_send(msg);
    state.lastErrorBroadcast = errorCode;
}

void sendMouldParamsConfirm(const actualMouldParams_t& params) {
    char msg[512];
    snprintf(msg, sizeof(msg),
             "MOULD_OK|%s|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f\n",
             params.mouldName,
             params.fillVolume, params.fillSpeed, params.fillPressure,
             params.packVolume, params.packSpeed, params.packPressure, params.packTime,
             params.coolingTime, params.fillTrapAccel, params.fillTrapDecel,
             params.packTrapAccel, params.packTrapDecel);
    uart_send(msg);
}

void sendCommonParamsConfirm() {
    char msg[512];
    snprintf(msg, sizeof(msg),
             "COMMON_OK|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%lu\n",
             commonParams.refillTrapVelLimit,
             commonParams.refillAccel,
             commonParams.refillDecel,
             commonParams.compressRampTarget,
             commonParams.compressRampDuration,
             commonParams.compressMicroCurrent,
             commonParams.injectFillTrapVelLimit,
             commonParams.injectFillAccel,
             commonParams.injectFillDecel,
             commonParams.injectFillCurrent,
             commonParams.injectPackTrapVelLimit,
             commonParams.injectPackAccel,
             commonParams.injectPackDecel,
             commonParams.injectPackCurrent,
             commonParams.injectVelThreshold,
             commonParams.injectPosLolerance,
             (unsigned long)commonParams.injectStableTimeMs);
    uart_send(msg);
}

static const char* next_token(const char* str, char* out, size_t out_len, char delim) {
    if (!str || !out || out_len == 0) return nullptr;
    const char* p = strchr(str, delim);
    if (p) {
        size_t len = (size_t)(p - str);
        if (len >= out_len) len = out_len - 1;
        memcpy(out, str, len);
        out[len] = '\0';
        return p + 1;
    }
    strncpy(out, str, out_len - 1);
    out[out_len - 1] = '\0';
    return nullptr;
}

void parseIncomingMessage(const char* message) {
    if (!message || message[0] == '\0') return;

    char cmd[32];
    const char* rest = next_token(message, cmd, sizeof(cmd), '|');

    if (strcmp(cmd, "MOULD") == 0) {
        actualMouldParams_t newParams = currentMould;
        char field[64];
        int fieldIdx = 0;
        while (rest) {
            rest = next_token(rest, field, sizeof(field), '|');
            switch (fieldIdx) {
                case 0: strncpy(newParams.mouldName, field, sizeof(newParams.mouldName) - 1); newParams.mouldName[sizeof(newParams.mouldName) - 1] = '\0'; break;
                case 1: newParams.fillVolume = (float)atof(field); break;
                case 2: newParams.fillSpeed = (float)atof(field); break;
                case 3: newParams.fillPressure = (float)atof(field); break;
                case 4: newParams.packVolume = (float)atof(field); break;
                case 5: newParams.packSpeed = (float)atof(field); break;
                case 6: newParams.packPressure = (float)atof(field); break;
                case 7: newParams.packTime = (float)atof(field); break;
                case 8: newParams.coolingTime = (float)atof(field); break;
                case 9: newParams.fillTrapAccel = (float)atof(field); break;
                case 10: newParams.fillTrapDecel = (float)atof(field); break;
                case 11: newParams.packTrapAccel = (float)atof(field); break;
                case 12: newParams.packTrapDecel = (float)atof(field); break;
                default: break;
            }
            fieldIdx++;
        }

        if (fieldIdx >= 13) {
            currentMould = newParams;
            sendMouldParamsConfirm(currentMould);
        } else {
            ESP_LOGW(TAG, "MOULD parse failed (fields=%d)", fieldIdx);
        }
        return;
    }

    if (strcmp(cmd, "COMMON") == 0) {
        commonInjectParams_t newParams = commonParams;
        char field[64];
        int fieldIdx = 0;
        while (rest) {
            rest = next_token(rest, field, sizeof(field), '|');
            switch (fieldIdx) {
                case 0: newParams.refillTrapVelLimit = (float)atof(field); break;
                case 1: newParams.refillAccel = (float)atof(field); break;
                case 2: newParams.refillDecel = (float)atof(field); break;
                case 3: newParams.compressRampTarget = (float)atof(field); break;
                case 4: newParams.compressRampDuration = (float)atof(field); break;
                case 5: newParams.compressMicroCurrent = (float)atof(field); break;
                case 6: newParams.injectFillTrapVelLimit = (float)atof(field); break;
                case 7: newParams.injectFillAccel = (float)atof(field); break;
                case 8: newParams.injectFillDecel = (float)atof(field); break;
                case 9: newParams.injectFillCurrent = (float)atof(field); break;
                case 10: newParams.injectPackTrapVelLimit = (float)atof(field); break;
                case 11: newParams.injectPackAccel = (float)atof(field); break;
                case 12: newParams.injectPackDecel = (float)atof(field); break;
                case 13: newParams.injectPackCurrent = (float)atof(field); break;
                case 14: newParams.injectVelThreshold = (float)atof(field); break;
                case 15: newParams.injectPosLolerance = (float)atof(field); break;
                case 16: newParams.injectStableTimeMs = (uint32_t)atol(field); break;
                default: break;
            }
            fieldIdx++;
        }
        if (fieldIdx >= 17) {
            commonParams = newParams;
            sendCommonParamsConfirm();
        } else {
            ESP_LOGW(TAG, "COMMON parse failed (fields=%d)", fieldIdx);
        }
        return;
    }

    if (strcmp(cmd, "QUERY_MOULD") == 0) {
        sendMouldParamsConfirm(currentMould);
        return;
    }

    if (strcmp(cmd, "QUERY_COMMON") == 0) {
        sendCommonParamsConfirm();
        return;
    }

    if (strcmp(cmd, "QUERY_STATE") == 0) {
        broadcastState(fsm_state.currentState);
        return;
    }

    if (strcmp(cmd, "QUERY_ERROR") == 0) {
        broadcastError(fsm_state.error, "QUERY_RESPONSE");
        return;
    }

    ESP_LOGW(TAG, "Unknown command: %s", cmd);
}

} // namespace DisplayComms

#endif // APP_DISPLAY_UART
