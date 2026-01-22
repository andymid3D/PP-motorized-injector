#include "DebugCommands.h"
#include "config.h"
#include "MessageBuffer.h"
#include <Arduino.h>

void DebugCommands::begin(CanBusHandlerV2& canHandler) {
    can_ = &canHandler;
    active_ = true;
    
    // Enable motor power contactor (DEBUG MODE - user responsible for safety)
    pinMode(PIN_CONTACTOR, OUTPUT);
    digitalWrite(PIN_CONTACTOR, HIGH);
    
    MessageBuffer::getInstance().sendMessage("=== DEBUG COMMANDS MODE ACTIVE ===");
    MessageBuffer::getInstance().sendMessage("Type commands to test CAN messages. Use 'help' for command list.");
    MessageBuffer::getInstance().sendMessage("Monitoring ODrive via serial every 1 second.");
    printStatusHeader();
}

void DebugCommands::loop() {
    // Read serial input
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) {
            handleCommand(cmd);
        }
    }
    
    // Print status every 1 second
    uint32_t now = millis();
    if (now - lastStatusTime_ >= STATUS_INTERVAL_MS) {
        printStatusLine();
        lastStatusTime_ = now;
    }
}

void DebugCommands::handleCommand(const String& cmd) {
    String command = cmd;
    int spacePos = cmd.indexOf(' ');
    if (spacePos > 0) {
        command = cmd.substring(0, spacePos);
    }
    
    // Convert to lowercase for case-insensitive matching
    command.toLowerCase();
    
    char buf[128];
    snprintf(buf, sizeof(buf), ">> %s", cmd.c_str());
    MessageBuffer::getInstance().sendMessage(buf);
    
    // ===== AXIS STATE =====
    if (command == "axis_state") {
        int state = atoi(cmd.c_str() + spacePos + 1);
        can_->setAxisState((ODriveCANProtocol::AxisState)state);
        Serial.printf("  Sent: SET_AXIS_STATE = %d\n", state);
    }
    
    // ===== CONTROL & INPUT MODES =====
    else if (command == "controller_modes") {
        float ctrl, input;
        if (parseCommand(cmd, command, ctrl, input)) {
            can_->setControllerModes((ODriveCANProtocol::ControlMode)(int)ctrl, 
                                     (ODriveCANProtocol::InputMode)(int)input);
            lastControlMode_ = (int)ctrl;
            lastInputMode_ = (int)input;
            Serial.printf("  Sent: SET_CONTROLLER_MODES ctrl=%d input=%d\n", (int)ctrl, (int)input);
        }
    }
    
    // ===== MOVEMENT COMMANDS =====
    else if (command == "set_position") {
        float pos = atof(cmd.c_str() + spacePos + 1);
        can_->setInputPos(pos);
        Serial.printf("  Sent: SET_INPUT_POS = %.3f turns\n", pos);
    }
    
    else if (command == "set_velocity") {
        float vel = atof(cmd.c_str() + spacePos + 1);
        can_->setInputVel(vel);
        Serial.printf("  Sent: SET_INPUT_VEL = %.3f turns/sec\n", vel);
    }
    
    else if (command == "set_torque") {
        float torque = atof(cmd.c_str() + spacePos + 1);
        can_->setInputTorque(torque);
        Serial.printf("  Sent: SET_INPUT_TORQUE = %.3f Nm\n", torque);
    }
    
    // ===== LIMITS =====
    else if (command == "set_limits") {
        float vel_lim, curr_lim;
        if (parseCommand(cmd, command, vel_lim, curr_lim)) {
            can_->setLimits(vel_lim, curr_lim);
            Serial.printf("  Sent: SET_LIMITS vel=%.1f T/s curr=%.1f A\n", vel_lim, curr_lim);
        }
    }
    
    // ===== ENCODER =====
    else if (command == "set_linear_count") {
        int32_t count = (int32_t)atol(cmd.c_str() + spacePos + 1);
        can_->setLinearCount(count);
        Serial.printf("  Sent: SET_LINEAR_COUNT = %ld\n", count);
    }
    
    // ===== ERROR HANDLING =====
    else if (command == "clear_errors") {
        can_->clearErrors();
        Serial.println("  Sent: CLEAR_ERRORS");
    }
    
    // ===== TRAJECTORY CONTROL =====
    else if (command == "traj_vel_limit") {
        float limit = atof(cmd.c_str() + spacePos + 1);
        can_->setTrajVelLimit(limit);
        Serial.printf("  Sent: SET_TRAJ_VEL_LIMIT = %.1f T/s\n", limit);
    }
    
    else if (command == "traj_accel") {
        float accel, decel;
        if (parseCommand(cmd, command, accel, decel)) {
            can_->setTrajAccelLimits(accel, decel);
            Serial.printf("  Sent: SET_TRAJ_ACCEL_LIMITS accel=%.1f decel=%.1f T/s²\n", accel, decel);
        }
    }
    
    else if (command == "traj_inertia") {
        float inertia = atof(cmd.c_str() + spacePos + 1);
        can_->setTrajInertia(inertia);
        Serial.printf("  Sent: SET_TRAJ_INERTIA = %.3f\n", inertia);
    }
    
    // ===== LOOP TUNING =====
    else if (command == "pos_gain") {
        float gain = atof(cmd.c_str() + spacePos + 1);
        can_->setPositionGain(gain);
        Serial.printf("  Sent: SET_POSITION_GAIN = %.1f\n", gain);
    }
    
    else if (command == "vel_gains") {
        float gain, integrator;
        if (parseCommand(cmd, command, gain, integrator)) {
            can_->setVelGains(gain, integrator);
            Serial.printf("  Sent: SET_VEL_GAINS gain=%.3f integrator=%.3f\n", gain, integrator);
        }
    }
    
    // ===== DIAGNOSTIC REQUESTS (RTR) =====
    else if (command == "heartbeat_request") {
        can_->heartbeatRequest();
        Serial.println("  Sent: HEARTBEAT_REQUEST");
    }
    
    else if (command == "get_motor_error") {
        can_->getMotorError();
        Serial.println("  Sent: GET_MOTOR_ERROR");
    }
    
    else if (command == "get_encoder_error") {
        can_->getEncoderError();
        Serial.println("  Sent: GET_ENCODER_ERROR");
    }
    
    else if (command == "get_sensorless_error") {
        can_->getSensorlessError();
        Serial.println("  Sent: GET_SENSORLESS_ERROR");
    }
    
    else if (command == "get_encoder_count") {
        can_->getEncoderCount();
        Serial.println("  Sent: GET_ENCODER_COUNT");
    }
    
    else if (command == "get_iq") {
        can_->getIq();
        Serial.println("  Sent: GET_IQ");
    }
    
    else if (command == "get_sensorless_estimates") {
        can_->getSensorlessEstimates();
        Serial.println("  Sent: GET_SENSORLESS_ESTIMATES");
    }
    
    else if (command == "get_bus_voltage_current") {
        can_->getBusVoltageCurrent();
        Serial.println("  Sent: GET_BUS_VOLTAGE_CURRENT");
    }
    
    else if (command == "get_adc_voltage") {
        can_->getAdcVoltage();
        Serial.println("  Sent: GET_ADC_VOLTAGE");
    }
    
    else if (command == "get_controller_error") {
        can_->getControllerError();
        Serial.println("  Sent: GET_CONTROLLER_ERROR");
    }
    
    // ===== ADVANCED =====
    else if (command == "set_axis_node_id") {
        uint32_t node_id = (uint32_t)atol(cmd.c_str() + spacePos + 1);
        can_->setAxisNodeId(node_id);
        Serial.printf("  Sent: SET_AXIS_NODE_ID = %lu (WARNING: address changes!)\n", node_id);
    }
    
    else if (command == "start_anticogging") {
        can_->startAnticogging();
        Serial.println("  Sent: START_ANTICOGGING (motor will move!)");
    }
    
    else if (command == "estop") {
        can_->estop();
        Serial.println("  Sent: ESTOP (motor stopped!)");
    }
    
    else if (command == "reboot") {
        can_->reboot();
        Serial.println("  Sent: REBOOT (ODrive rebooting...)");
    }
    
    // ===== HELP & INFO =====
    else if (command == "help" || command == "?" || cmd == "") {
        Serial.println("\n=== AVAILABLE COMMANDS ===\n");
        Serial.println("AXIS STATE:");
        Serial.println("  axis_state <state>              // 0-13 (8=CLOSED_LOOP)");
        Serial.println("\nCONTROL & INPUT MODES:");
        Serial.println("  controller_modes <ctrl> <input> // ctrl: 0-3, input: 0,1,2,3,5,6");
        Serial.println("\nMOVEMENT:");
        Serial.println("  set_position <pos>              // position in turns");
        Serial.println("  set_velocity <vel>              // velocity in turns/sec");
        Serial.println("  set_torque <torque>             // torque in Nm");
        Serial.println("\nLIMITS:");
        Serial.println("  set_limits <vel> <curr>         // velocity limit, current limit");
        Serial.println("\nENCODER:");
        Serial.println("  set_linear_count <count>        // reset encoder (0 = zero)");
        Serial.println("\nERRORS:");
        Serial.println("  clear_errors                    // clear all errors");
        Serial.println("\nTRAJECTORY (TRAP_TRAJ mode):");
        Serial.println("  traj_vel_limit <limit>          // velocity limit");
        Serial.println("  traj_accel <accel> <decel>      // acceleration, deceleration");
        Serial.println("  traj_inertia <inertia>          // feedforward inertia");
        Serial.println("\nTUNING:");
        Serial.println("  pos_gain <gain>                 // position loop gain");
        Serial.println("  vel_gains <gain> <integrator>   // velocity loop gains");
        Serial.println("\nDIAGNOSTICS:");
        Serial.println("  heartbeat_request");
        Serial.println("  get_motor_error");
        Serial.println("  get_encoder_error");
        Serial.println("  get_sensorless_error");
        Serial.println("  get_encoder_count");
        Serial.println("  get_iq");
        Serial.println("  get_sensorless_estimates");
        Serial.println("  get_bus_voltage_current");
        Serial.println("  get_adc_voltage");
        Serial.println("  get_controller_error");
        Serial.println("\nADVANCED:");
        Serial.println("  set_axis_node_id <id>           // WARNING: changes CAN address!");
        Serial.println("  start_anticogging               // WARNING: motor moves!");
        Serial.println("  estop                           // EMERGENCY STOP");
        Serial.println("  reboot                          // Reboot ODrive");
        Serial.println();
    }
    
    else {
        Serial.println("  ERROR: Unknown command. Type 'help' for commands.");
    }
}

void DebugCommands::printStatusHeader() {
    // Headers removed - inline labels on each data line for continuous reference
}

void DebugCommands::printStatusLine() {
    static uint32_t startTime = millis();
    uint32_t elapsed = (millis() - startTime) / 1000;
    
    auto encoder = can_->getEncoderEstimates();
    auto iq = can_->getIqReadings();
    auto bus = can_->getBusVoltageCurrentReadings();
    
    // Get current heartbeat for axis state (to track mode changes)
    auto heartbeat = can_->getHeartbeat();
    
    char buffer[200];
    snprintf(buffer, sizeof(buffer),
        "s:%4lu  P:%7.2f  V:%5.2f  IqS:%5.1f  IqM:%5.1f  bV:%5.1f  bI:%5.1f  Ctrl:%3s  Input:%1d  OD:%1d  ADC:%4.1f",
        elapsed,
        encoder.position,
        encoder.velocity,
        iq.Iq_setpoint,
        iq.Iq_measured,
        bus.bus_voltage,
        bus.bus_current,
        getControlModeName(lastControlMode_).c_str(),
        (lastInputMode_ >= 0) ? lastInputMode_ : -1,
        heartbeat.axis_state,
        0.0f  // TODO: add ADC voltage from cyclic message if enabled
    );
    
    Serial.println(buffer);
}

String DebugCommands::getControlModeName(int mode) const {
    switch (mode) {
        case 0: return "VOL";
        case 1: return "TOR";
        case 2: return "VEL";
        case 3: return "POS";
        default: return "---";
    }
}

String DebugCommands::getInputModeName(int mode) const {
    switch (mode) {
        case 0: return "Inactive";
        case 1: return "Passthru";
        case 2: return "VelRamp";
        case 3: return "PosFilt";
        case 5: return "TrapTraj";
        case 6: return "TorqRamp";
        default: return "---";
    }
}

bool DebugCommands::parseCommand(const String& cmd, String& command, float& param1, float& param2) {
    int firstSpace = cmd.indexOf(' ');
    if (firstSpace < 0) return false;
    
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (secondSpace < 0) {
        // Only one parameter
        param1 = atof(cmd.c_str() + firstSpace + 1);
        return true;
    }
    
    // Two parameters
    String p1Str = cmd.substring(firstSpace + 1, secondSpace);
    String p2Str = cmd.substring(secondSpace + 1);
    
    param1 = atof(p1Str.c_str());
    param2 = atof(p2Str.c_str());
    
    return true;
}
