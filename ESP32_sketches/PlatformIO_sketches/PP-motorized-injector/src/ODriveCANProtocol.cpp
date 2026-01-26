#include "ODriveCANProtocol.h"

// ===== Parsing Incoming Messages =====

ODriveCANProtocol::Heartbeat ODriveCANProtocol::parseHeartbeat(const can_Message_t& msg) {
    Heartbeat hb;
    // Bytes 0-3: axis_error (u32, Intel byte order)
    hb.axis_error = can_getSignal<uint32_t>(msg, 0, 32, true);
    // Byte 4: axis_state (u8)
    hb.axis_state = can_getSignal<uint8_t>(msg, 32, 8, true);
    // Byte 5: motor_error_flag (u8)
    hb.motor_error_flag = can_getSignal<uint8_t>(msg, 40, 8, true);
    // Byte 6: encoder_error_flag (u8)
    hb.encoder_error_flag = can_getSignal<uint8_t>(msg, 48, 8, true);
    // Byte 7: controller_error_flag (u8, bit 7 is trajectory_done)
    uint8_t controller_byte = can_getSignal<uint8_t>(msg, 56, 8, true);
    hb.controller_error_flag = controller_byte & 0x7F;  // Bits 0-6
    hb.trajectory_done_flag = (controller_byte >> 7) & 0x01;  // Bit 7
    return hb;
}

ODriveCANProtocol::EncoderEstimate ODriveCANProtocol::parseEncoderEstimate(const can_Message_t& msg) {
    EncoderEstimate est;
    // Bytes 0-3: position (float32, Intel byte order)
    est.position = can_getSignal<float>(msg, 0, 32, true);
    // Bytes 4-7: velocity (float32, Intel byte order)
    est.velocity = can_getSignal<float>(msg, 32, 32, true);
    return est;
}

// ===== Building Command Messages =====

can_Message_t ODriveCANProtocol::buildSetAxisState(uint8_t node_id, AxisState state) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_AXIS_STATE);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: requested_state (int32, Intel byte order)
    can_setSignal<int32_t>(msg, static_cast<int32_t>(state), 0, 32, true);
    return msg;
}

can_Message_t ODriveCANProtocol::buildSetControllerModes(uint8_t node_id, ControlMode ctrl_mode, InputMode input_mode) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_CONTROLLER_MODES);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: control_mode (int32, Intel byte order)
    can_setSignal<int32_t>(msg, static_cast<int32_t>(ctrl_mode), 0, 32, true);
    // Bytes 4-7: input_mode (int32, Intel byte order)
    can_setSignal<int32_t>(msg, static_cast<int32_t>(input_mode), 32, 32, true);
    return msg;
}

can_Message_t ODriveCANProtocol::buildSetInputPos(uint8_t node_id, float position, int16_t velocity, int16_t torque) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_INPUT_POS);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: position (float32, Intel byte order)
    can_setSignal<float>(msg, position, 0, 32, true);
    // Bytes 4-5: velocity (int16, Intel byte order, scaled by 0.001)
    can_setSignal<int16_t>(msg, velocity, 32, 16, true, 0.001f, 0);
    // Bytes 6-7: torque (int16, Intel byte order, scaled by 0.001)
    can_setSignal<int16_t>(msg, torque, 48, 16, true, 0.001f, 0);
    return msg;
}

can_Message_t ODriveCANProtocol::buildSetInputVel(uint8_t node_id, float velocity, float torque) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_INPUT_VEL);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: velocity (float32, Intel byte order)
    can_setSignal<float>(msg, velocity, 0, 32, true);
    // Bytes 4-7: torque (float32, Intel byte order)
    can_setSignal<float>(msg, torque, 32, 32, true);
    return msg;
}

can_Message_t ODriveCANProtocol::buildSetInputTorque(uint8_t node_id, float torque) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_INPUT_TORQUE);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: torque (float32, Intel byte order)
    can_setSignal<float>(msg, torque, 0, 32, true);
    return msg;
}

can_Message_t ODriveCANProtocol::buildSetLimits(uint8_t node_id, float vel_limit, float current_limit) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_LIMITS);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: velocity limit (float32, Intel byte order)
    can_setSignal<float>(msg, vel_limit, 0, 32, true);
    // Bytes 4-7: current limit (float32, Intel byte order)
    can_setSignal<float>(msg, current_limit, 32, 32, true);
    return msg;
}

can_Message_t ODriveCANProtocol::buildSetLinearCount(uint8_t node_id, int32_t count) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_LINEAR_COUNT);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: encoder count (int32, Intel byte order)
    can_setSignal<int32_t>(msg, count, 0, 32, true);
    return msg;
}

can_Message_t ODriveCANProtocol::buildClearErrors(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_CLEAR_ERRORS);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // No payload
    return msg;
}

can_Message_t ODriveCANProtocol::buildGetEncoderEstimates(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_GET_ENCODER_ESTIMATES);
    msg.rtr = true;  // Remote Transfer Request
    msg.len = 0;
    return msg;
}

// ===== RTR Request Functions (Remote Transfer Request - No Parameters) =====

can_Message_t ODriveCANProtocol::buildHeartbeatRequest(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_HEARTBEAT_REQUEST);
    msg.rtr = true;
    msg.len = 0;
    return msg;
}

can_Message_t ODriveCANProtocol::buildEstop(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_ESTOP);
    msg.len = 8;
    // No payload
    return msg;
}

can_Message_t ODriveCANProtocol::buildGetMotorError(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_GET_MOTOR_ERROR);
    msg.rtr = true;
    msg.len = 0;
    return msg;
}

can_Message_t ODriveCANProtocol::buildGetEncoderError(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_GET_ENCODER_ERROR);
    msg.rtr = true;
    msg.len = 0;
    return msg;
}

can_Message_t ODriveCANProtocol::buildGetSensorlessError(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_GET_SENSORLESS_ERROR);
    msg.rtr = true;
    msg.len = 0;
    return msg;
}

can_Message_t ODriveCANProtocol::buildGetEncoderCount(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_GET_ENCODER_COUNT);
    msg.rtr = true;
    msg.len = 0;
    return msg;
}

can_Message_t ODriveCANProtocol::buildGetIq(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_GET_IQ);
    msg.rtr = true;
    msg.len = 0;
    return msg;
}

can_Message_t ODriveCANProtocol::buildGetSensorlessEstimates(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_GET_SENSORLESS_ESTIMATES);
    msg.rtr = true;
    msg.len = 0;
    return msg;
}

can_Message_t ODriveCANProtocol::buildReboot(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_REBOOT);
    msg.len = 8;
    // No payload
    return msg;
}

can_Message_t ODriveCANProtocol::buildGetBusVoltageCurrent(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_GET_BUS_VOLTAGE_CURRENT);
    msg.rtr = true;
    msg.len = 0;
    return msg;
}

can_Message_t ODriveCANProtocol::buildGetAdcVoltage(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_GET_ADC_VOLTAGE);
    msg.rtr = true;
    msg.len = 0;
    return msg;
}

can_Message_t ODriveCANProtocol::buildGetControllerError(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_GET_CONTROLLER_ERROR);
    msg.rtr = true;
    msg.len = 0;
    return msg;
}

// ===== Parameter Command Functions =====

can_Message_t ODriveCANProtocol::buildSetAxisNodeId(uint8_t node_id, uint32_t new_node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_AXIS_NODE_ID);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: new_node_id (uint32, Intel byte order)
    can_setSignal<uint32_t>(msg, new_node_id, 0, 32, true);
    return msg;
}

can_Message_t ODriveCANProtocol::buildStartAnticogging(uint8_t node_id) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_START_ANTICOGGING);
    msg.len = 8;
    // No payload
    return msg;
}

can_Message_t ODriveCANProtocol::buildSetTrajVelLimit(uint8_t node_id, float vel_limit) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_TRAJ_VEL_LIMIT);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: velocity limit (float32, Intel byte order)
    can_setSignal<float>(msg, vel_limit, 0, 32, true);
    return msg;
}

can_Message_t ODriveCANProtocol::buildSetTrajAccelLimits(uint8_t node_id, float accel, float decel) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_TRAJ_ACCEL_LIMITS);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: accel limit (float32, Intel byte order)
    can_setSignal<float>(msg, accel, 0, 32, true);
    // Bytes 4-7: decel limit (float32, Intel byte order)
    can_setSignal<float>(msg, decel, 32, 32, true);
    return msg;
}

can_Message_t ODriveCANProtocol::buildSetTrajInertia(uint8_t node_id, float inertia) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_TRAJ_INERTIA);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: inertia (float32, Intel byte order)
    can_setSignal<float>(msg, inertia, 0, 32, true);
    return msg;
}

can_Message_t ODriveCANProtocol::buildSetPositionGain(uint8_t node_id, float gain) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_POSITION_GAIN);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: position gain (float32, Intel byte order)
    can_setSignal<float>(msg, gain, 0, 32, true);
    return msg;
}

can_Message_t ODriveCANProtocol::buildSetVelGains(uint8_t node_id, float p_gain, float i_gain) {
    can_Message_t msg;
    msg.id = makeCanId(node_id, MSG_SET_VEL_GAINS);
    msg.rtr = false;  // Explicitly set to false for SET commands
    msg.len = 8;
    // Bytes 0-3: velocity proportional gain (float32, Intel byte order)
    can_setSignal<float>(msg, p_gain, 0, 32, true);
    // Bytes 4-7: velocity integral gain (float32, Intel byte order)
    can_setSignal<float>(msg, i_gain, 32, 32, true);
    return msg;
}

// ===== Parsing Cyclic (Broadcast) Messages =====

ODriveCANProtocol::CyclicHeartbeat ODriveCANProtocol::parseCyclicHeartbeat(const can_Message_t& msg) {
    CyclicHeartbeat hb;
    hb.axis_error = can_getSignal<uint32_t>(msg, 0, 32, true);
    hb.axis_state = can_getSignal<uint8_t>(msg, 32, 8, true);
    hb.motor_error_flag = can_getSignal<uint8_t>(msg, 40, 8, true);
    hb.encoder_error_flag = can_getSignal<uint8_t>(msg, 48, 8, true);
    uint8_t controller_byte = can_getSignal<uint8_t>(msg, 56, 8, true);
    hb.controller_error_flag = controller_byte & 0x7F;
    hb.trajectory_done_flag = (controller_byte >> 7) & 0x01;
    return hb;
}

ODriveCANProtocol::CyclicMotorError ODriveCANProtocol::parseCyclicMotorError(const can_Message_t& msg) {
    CyclicMotorError me;
    me.motor_error = can_getSignal<uint32_t>(msg, 0, 32, true);
    return me;
}

ODriveCANProtocol::CyclicEncoderError ODriveCANProtocol::parseCyclicEncoderError(const can_Message_t& msg) {
    CyclicEncoderError ee;
    ee.encoder_error = can_getSignal<uint32_t>(msg, 0, 32, true);
    return ee;
}

ODriveCANProtocol::CyclicSensorlessError ODriveCANProtocol::parseCyclicSensorlessError(const can_Message_t& msg) {
    CyclicSensorlessError se;
    se.sensorless_error = can_getSignal<uint32_t>(msg, 0, 32, true);
    return se;
}

ODriveCANProtocol::CyclicEncoderEstimates ODriveCANProtocol::parseCyclicEncoderEstimates(const can_Message_t& msg) {
    CyclicEncoderEstimates est;
    est.position = can_getSignal<float>(msg, 0, 32, true);
    est.velocity = can_getSignal<float>(msg, 32, 32, true);
    return est;
}

ODriveCANProtocol::CyclicEncoderCount ODriveCANProtocol::parseCyclicEncoderCount(const can_Message_t& msg) {
    CyclicEncoderCount ec;
    ec.encoder_count = can_getSignal<int32_t>(msg, 0, 32, true);
    return ec;
}

ODriveCANProtocol::CyclicIq ODriveCANProtocol::parseCyclicIq(const can_Message_t& msg) {
    CyclicIq iq;
    iq.Iq_setpoint = can_getSignal<float>(msg, 0, 32, true);
    iq.Iq_measured = can_getSignal<float>(msg, 32, 32, true);
    return iq;
}

ODriveCANProtocol::CyclicSensorlessEstimates ODriveCANProtocol::parseCyclicSensorlessEstimates(const can_Message_t& msg) {
    CyclicSensorlessEstimates est;
    est.position = can_getSignal<float>(msg, 0, 32, true);
    est.velocity = can_getSignal<float>(msg, 32, 32, true);
    return est;
}

ODriveCANProtocol::CyclicBusVoltageCurrent ODriveCANProtocol::parseCyclicBusVoltageCurrent(const can_Message_t& msg) {
    CyclicBusVoltageCurrent bvc;
    bvc.bus_voltage = can_getSignal<float>(msg, 0, 32, true);
    bvc.bus_current = can_getSignal<float>(msg, 32, 32, true);
    return bvc;
}

ODriveCANProtocol::CyclicControllerError ODriveCANProtocol::parseCyclicControllerError(const can_Message_t& msg) {
    CyclicControllerError ce;
    ce.controller_error = can_getSignal<uint32_t>(msg, 0, 32, true);
    return ce;
}
