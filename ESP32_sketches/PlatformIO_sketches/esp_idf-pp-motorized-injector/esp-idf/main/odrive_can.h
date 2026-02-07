#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

#include <cstdint>
#include <cstring>
#include "driver/twai.h"

namespace odrive_can {

// Request/response message IDs
enum RequestMessageID : uint32_t {
    MSG_HEARTBEAT_REQUEST       = 0x001,
    MSG_ESTOP                   = 0x002,
    MSG_GET_MOTOR_ERROR         = 0x003,
    MSG_GET_ENCODER_ERROR       = 0x004,
    MSG_GET_SENSORLESS_ERROR    = 0x005,
    MSG_SET_AXIS_NODE_ID        = 0x006,
    MSG_SET_AXIS_STATE          = 0x007,
    MSG_SET_AXIS_STARTUP_CONFIG = 0x008,
    MSG_GET_ENCODER_ESTIMATES   = 0x009,
    MSG_GET_ENCODER_COUNT       = 0x00A,
    MSG_SET_CONTROLLER_MODES    = 0x00B,
    MSG_SET_INPUT_POS           = 0x00C,
    MSG_SET_INPUT_VEL           = 0x00D,
    MSG_SET_INPUT_TORQUE        = 0x00E,
    MSG_SET_LIMITS              = 0x00F,
    MSG_START_ANTICOGGING       = 0x010,
    MSG_SET_TRAJ_VEL_LIMIT      = 0x011,
    MSG_SET_TRAJ_ACCEL_LIMITS   = 0x012,
    MSG_SET_TRAJ_INERTIA        = 0x013,
    MSG_GET_IQ                  = 0x014,
    MSG_GET_SENSORLESS_ESTIMATES= 0x015,
    MSG_REBOOT                  = 0x016,
    MSG_GET_BUS_VOLTAGE_CURRENT = 0x017,
    MSG_CLEAR_ERRORS            = 0x018,
    MSG_SET_LINEAR_COUNT        = 0x019,
    MSG_SET_POSITION_GAIN       = 0x01A,
    MSG_SET_VEL_GAINS           = 0x01B,
    MSG_GET_ADC_VOLTAGE         = 0x01C,
    MSG_GET_CONTROLLER_ERROR    = 0x01D,
};

// Cyclic broadcast IDs
enum CyclicMessageID : uint32_t {
    CYCLIC_HEARTBEAT            = 0x01,
    CYCLIC_MOTOR_ERROR          = 0x03,
    CYCLIC_ENCODER_ERROR        = 0x04,
    CYCLIC_SENSORLESS_ERROR     = 0x05,
    CYCLIC_ENCODER_ESTIMATES    = 0x09,
    CYCLIC_ENCODER_COUNT        = 0x0A,
    CYCLIC_IQ                   = 0x14,
    CYCLIC_SENSORLESS_ESTIMATES = 0x15,
    CYCLIC_BUS_VI               = 0x17,
    CYCLIC_CONTROLLER_ERROR     = 0x1D,
};

enum class AxisState : uint8_t {
    UNDEFINED                   = 0,
    IDLE                        = 1,
    STARTUP_SEQUENCE            = 2,
    FULL_CALIBRATION_SEQUENCE   = 3,
    MOTOR_CALIBRATION           = 4,
    ENCODER_INDEX_SEARCH        = 6,
    ENCODER_OFFSET_CALIBRATION  = 7,
    CLOSED_LOOP_CONTROL         = 8,
    LOCKIN_SPIN                 = 9,
    ENCODER_DIR_FIND            = 10,
    HOMING                      = 11,
    ENCODER_HALL_POLARITY_CALIB = 12,
    ENCODER_HALL_PHASE_CALIB    = 13,
};

enum class ControlMode : int32_t {
    VOLTAGE_CONTROL             = 0,
    TORQUE_CONTROL              = 1,
    VELOCITY_CONTROL            = 2,
    POSITION_CONTROL            = 3,
};

enum class InputMode : int32_t {
    INACTIVE                    = 0,
    PASSTHROUGH                 = 1,
    VEL_RAMP                    = 2,
    POS_FILTER                  = 3,
    TRAP_TRAJ                   = 5,
    TORQUE_RAMP                 = 6,
};

inline uint32_t make_can_id(uint8_t node_id, uint32_t cmd_id) {
    return (static_cast<uint32_t>(node_id) << 5) | (cmd_id & 0x1F);
}

inline void set_u32_le(uint8_t* data, uint32_t v) {
    data[0] = static_cast<uint8_t>(v & 0xFF);
    data[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    data[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

inline void set_u64_le(uint8_t* data, uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        data[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
    }
}

inline void set_f32_le(uint8_t* data, float v) {
    static_assert(sizeof(float) == 4, "float must be 32-bit");
    uint32_t temp;
    std::memcpy(&temp, &v, sizeof(float));
    set_u32_le(data, temp);
}

inline uint32_t get_u32_le(const uint8_t* data) {
    return (static_cast<uint32_t>(data[0])      ) |
           (static_cast<uint32_t>(data[1]) <<  8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

inline uint64_t get_u64_le(const uint8_t* data) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {
        v |= (static_cast<uint64_t>(data[i]) << (8 * i));
    }
    return v;
}

inline float get_f32_le(const uint8_t* data) {
    uint32_t temp = get_u32_le(data);
    float v;
    std::memcpy(&v, &temp, sizeof(float));
    return v;
}

// Build TWAI message helpers
inline twai_message_t build_set_axis_state(uint8_t node_id, AxisState state) {
    twai_message_t msg = {};
    msg.identifier = make_can_id(node_id, MSG_SET_AXIS_STATE);
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    int32_t s = static_cast<int32_t>(state);
    set_u32_le(msg.data, static_cast<uint32_t>(s));
    return msg;
}

inline twai_message_t build_set_controller_modes(uint8_t node_id, ControlMode ctrl, InputMode input) {
    twai_message_t msg = {};
    msg.identifier = make_can_id(node_id, MSG_SET_CONTROLLER_MODES);
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    int32_t c = static_cast<int32_t>(ctrl);
    int32_t i = static_cast<int32_t>(input);
    set_u32_le(msg.data, static_cast<uint32_t>(c));
    set_u32_le(msg.data + 4, static_cast<uint32_t>(i));
    return msg;
}

inline twai_message_t build_set_input_pos(uint8_t node_id, float position) {
    twai_message_t msg = {};
    msg.identifier = make_can_id(node_id, MSG_SET_INPUT_POS);
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    set_f32_le(msg.data, position);
    // remaining bytes are zero (vel/torque feedforward omitted)
    return msg;
}

inline twai_message_t build_set_input_vel(uint8_t node_id, float velocity, float torque_ff = 0.0f) {
    twai_message_t msg = {};
    msg.identifier = make_can_id(node_id, MSG_SET_INPUT_VEL);
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    set_f32_le(msg.data, velocity);
    set_f32_le(msg.data + 4, torque_ff);
    return msg;
}

inline twai_message_t build_set_input_torque(uint8_t node_id, float torque) {
    twai_message_t msg = {};
    msg.identifier = make_can_id(node_id, MSG_SET_INPUT_TORQUE);
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    set_f32_le(msg.data, torque);
    return msg;
}

inline twai_message_t build_set_limits(uint8_t node_id, float vel_limit, float current_limit) {
    twai_message_t msg = {};
    msg.identifier = make_can_id(node_id, MSG_SET_LIMITS);
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    set_f32_le(msg.data, vel_limit);
    set_f32_le(msg.data + 4, current_limit);
    return msg;
}

inline twai_message_t build_set_traj_vel_limit(uint8_t node_id, float vel_limit) {
    twai_message_t msg = {};
    msg.identifier = make_can_id(node_id, MSG_SET_TRAJ_VEL_LIMIT);
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    set_f32_le(msg.data, vel_limit);
    return msg;
}

inline twai_message_t build_set_traj_accel_limits(uint8_t node_id, float accel, float decel) {
    twai_message_t msg = {};
    msg.identifier = make_can_id(node_id, MSG_SET_TRAJ_ACCEL_LIMITS);
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    set_f32_le(msg.data, accel);
    set_f32_le(msg.data + 4, decel);
    return msg;
}

inline twai_message_t build_set_linear_count(uint8_t node_id, int32_t count) {
    twai_message_t msg = {};
    msg.identifier = make_can_id(node_id, MSG_SET_LINEAR_COUNT);
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    set_u32_le(msg.data, static_cast<uint32_t>(count));
    return msg;
}

inline twai_message_t build_clear_errors(uint8_t node_id) {
    twai_message_t msg = {};
    msg.identifier = make_can_id(node_id, MSG_CLEAR_ERRORS);
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    return msg;
}

inline twai_message_t build_estop(uint8_t node_id) {
    twai_message_t msg = {};
    msg.identifier = make_can_id(node_id, MSG_ESTOP);
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;
    return msg;
}

} // namespace odrive_can

#endif // ODRIVE_CAN_H
