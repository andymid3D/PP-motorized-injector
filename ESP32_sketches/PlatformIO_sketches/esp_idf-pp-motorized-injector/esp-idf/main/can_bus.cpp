#include "can_bus.h"
#include "broadcast_data_store.h"
#include "time_utils.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "CAN_BUS";

void CanBus::begin() {
    static bool initialized = false;
    if (initialized) {
        return;
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX,
        (gpio_num_t)PIN_CAN_RX,
        TWAI_MODE_NORMAL
    );
    // We handle TX queueing ourselves
    g_config.tx_queue_len = 0;
    g_config.rx_queue_len = 5;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI install failed: %d", err);
        return;
    }

    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed: %d", err);
        return;
    }

    initialized = true;
    ESP_LOGI(TAG, "TWAI started (250kbps)");
}

bool CanBus::isMotorStopped() const {
    BroadcastDataStore& bds = BroadcastDataStore::getInstance();
    float v = bds.getVelocity();
    return (v > -STOP_VELOCITY_THRESHOLD) && (v < STOP_VELOCITY_THRESHOLD);
}

void CanBus::loop() {
    if (waitingForStop_) {
        uint64_t now = time_utils::micros();
        if ((now - stopCommandTime_) > (STOP_TIMEOUT_MS * 1000ULL)) {
            waitingForStop_ = false;
            ESP_LOGW(TAG, "STOP_VERIFY_TIMEOUT");
        } else if (isMotorStopped()) {
            if ((now - stopCommandTime_) > (STOP_SETTLE_TIME_MS * 1000ULL)) {
                waitingForStop_ = false;
            }
        } else {
            return;
        }
    }

    if (isQueueEmpty()) {
        return;
    }

    uint64_t now = time_utils::micros();
    if ((now - lastCommandSentTime_) < (CAN_COMMAND_GAP_MS * 1000ULL)) {
        return;
    }

    // Peek
    const twai_message_t& cmd = commandQueue_[queueTail_];
    esp_err_t err = twai_transmit(&cmd, 0);
    if (err != ESP_OK) {
        return;
    }

    // Pop
    queueTail_ = (queueTail_ + 1) % CMD_QUEUE_SIZE;
    queueFull_ = false;
    lastCommandSentTime_ = now;

    // Stop verification if velocity command = 0
    if (cmd.identifier == odrive_can::make_can_id(ODRIVE_NODE_ID, odrive_can::MSG_SET_INPUT_VEL)) {
        float vel = odrive_can::get_f32_le(cmd.data);
        if (vel > -STOP_VELOCITY_THRESHOLD && vel < STOP_VELOCITY_THRESHOLD) {
            waitingForStop_ = true;
            stopCommandTime_ = time_utils::micros();
        }
    }
}

bool CanBus::queueCommand(const twai_message_t& cmd) {
    if (queueFull_) {
        return false;
    }
    commandQueue_[queueHead_] = cmd;
    queueHead_ = (queueHead_ + 1) % CMD_QUEUE_SIZE;
    if (queueHead_ == queueTail_) {
        queueFull_ = true;
    }
    return true;
}

bool CanBus::isQueueEmpty() const {
    return (queueHead_ == queueTail_) && !queueFull_;
}

bool CanBus::isQueueFull() const {
    return queueFull_;
}

uint8_t CanBus::getQueueDepth() const {
    if (queueFull_) return CMD_QUEUE_SIZE;
    if (queueHead_ >= queueTail_) return queueHead_ - queueTail_;
    return CMD_QUEUE_SIZE - (queueTail_ - queueHead_);
}

// ===== Commands =====

bool CanBus::setAxisState(odrive_can::AxisState state) {
    return queueCommand(odrive_can::build_set_axis_state(ODRIVE_NODE_ID, state));
}

bool CanBus::setControllerModes(odrive_can::ControlMode ctrl, odrive_can::InputMode input) {
    return queueCommand(odrive_can::build_set_controller_modes(ODRIVE_NODE_ID, ctrl, input));
}

bool CanBus::setInputPos(float pos) {
    return queueCommand(odrive_can::build_set_input_pos(ODRIVE_NODE_ID, pos));
}

bool CanBus::setInputVel(float vel, float torque_ff) {
    return queueCommand(odrive_can::build_set_input_vel(ODRIVE_NODE_ID, vel, torque_ff));
}

bool CanBus::setInputTorque(float torque) {
    return queueCommand(odrive_can::build_set_input_torque(ODRIVE_NODE_ID, torque));
}

bool CanBus::setLimits(float vel_limit, float current_limit) {
    return queueCommand(odrive_can::build_set_limits(ODRIVE_NODE_ID, vel_limit, current_limit));
}

bool CanBus::setTrajVelLimit(float vel_limit) {
    return queueCommand(odrive_can::build_set_traj_vel_limit(ODRIVE_NODE_ID, vel_limit));
}

bool CanBus::setTrajAccelLimits(float accel, float decel) {
    return queueCommand(odrive_can::build_set_traj_accel_limits(ODRIVE_NODE_ID, accel, decel));
}

bool CanBus::setLinearCount(int32_t count) {
    return queueCommand(odrive_can::build_set_linear_count(ODRIVE_NODE_ID, count));
}

bool CanBus::clearErrors() {
    return queueCommand(odrive_can::build_clear_errors(ODRIVE_NODE_ID));
}

bool CanBus::estop() {
    return queueCommand(odrive_can::build_estop(ODRIVE_NODE_ID));
}
