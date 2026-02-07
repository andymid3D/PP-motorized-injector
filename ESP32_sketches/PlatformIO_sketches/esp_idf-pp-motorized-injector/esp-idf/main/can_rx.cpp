#include "can_rx.h"
#include "driver/twai.h"
#include "broadcast_data_store.h"
#include "odrive_can.h"
#include "time_utils.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_rom_sys.h"

static const char* TAG = "CAN_RX";

CanRxHandler& CanRxHandler::getInstance() {
    static CanRxHandler instance;
    return instance;
}

CanRxHandler::CanRxHandler() : pollingTaskHandle_(nullptr) {}

bool CanRxHandler::begin() {
    // WDT for Core 0 task
    esp_task_wdt_config_t wdt_config = {};
    wdt_config.timeout_ms = 3000;
    wdt_config.idle_core_mask = 0;
    wdt_config.trigger_panic = true;
    esp_err_t err = esp_task_wdt_reconfigure(&wdt_config);
    if (err == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_init(&wdt_config);
    }
    TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCore(0);
    if (idle0 && esp_task_wdt_status(idle0) == ESP_OK) {
        esp_task_wdt_delete(idle0);
    }
    return true;
}

bool CanRxHandler::startCore0Task() {
    xTaskCreatePinnedToCore(
        pollingTaskCore0,
        "CANRxPoll",
        4096,
        this,
        1,
        &pollingTaskHandle_,
        0
    );
    return pollingTaskHandle_ != nullptr;
}

void CanRxHandler::pollingTaskCore0(void* pvParameters) {
    esp_task_wdt_add(NULL);

    CanRxHandler* handler = static_cast<CanRxHandler*>(pvParameters);
    uint32_t pollCount = 0;

    while (true) {
        handler->pollAndProcess();
        if (++pollCount >= 100) {
            esp_task_wdt_reset();
            pollCount = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void CanRxHandler::pollAndProcess() {
    twai_message_t frame = {};
    if (twai_receive(&frame, 0) != ESP_OK) {
        return;
    }

    uint64_t timestamp = time_utils::micros();

    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    uint32_t cmdId = frame.identifier & 0x1F;

    if (cmdId == odrive_can::CYCLIC_HEARTBEAT) {
        uint32_t axis_error = odrive_can::get_u32_le(frame.data);
        uint8_t axis_state = frame.data[4];
        uint8_t motor_error_flag = frame.data[5];
        uint8_t encoder_error_flag = frame.data[6];
        uint8_t controller_error_flag = frame.data[7] & 0x7F;
        uint8_t trajectory_done_flag = (frame.data[7] >> 7) & 0x01;

        bds.storeHeartbeat(axis_error, axis_state, motor_error_flag, encoder_error_flag,
                           controller_error_flag, trajectory_done_flag, timestamp, false);
    } else if (cmdId == odrive_can::CYCLIC_ENCODER_ESTIMATES) {
        float pos = odrive_can::get_f32_le(frame.data);
        float vel = odrive_can::get_f32_le(frame.data + 4);
        bds.storeEncoder(pos, vel, timestamp, false);
    } else if (cmdId == odrive_can::CYCLIC_IQ) {
        float iq_set = odrive_can::get_f32_le(frame.data);
        float iq_meas = odrive_can::get_f32_le(frame.data + 4);
        bds.storeIq(iq_set, iq_meas, timestamp, false);
    } else if (cmdId == odrive_can::CYCLIC_MOTOR_ERROR) {
        uint64_t motor_error = odrive_can::get_u64_le(frame.data);
        bds.storeMotorError(motor_error, timestamp, false);
    } else if (cmdId == odrive_can::CYCLIC_ENCODER_ERROR) {
        uint32_t encoder_error = odrive_can::get_u32_le(frame.data);
        bds.storeEncoderError(encoder_error, timestamp, false);
    } else if (cmdId == odrive_can::CYCLIC_CONTROLLER_ERROR) {
        uint32_t controller_error = odrive_can::get_u32_le(frame.data);
        bds.storeControllerError(controller_error, timestamp, false);
    } else {
        // Ignore other messages for now
    }
}
