#pragma once

#include <Arduino.h>
#include <SafeString.h>
#include "config.h"
#include "mould.h"
#include "injector_fsm.h"

// ===== UART DISPLAY COMMUNICATIONS MODULE =====
// Handles bidirectional UART communication between ESP32 Controller and Display
// Uses SafeString library for robust message parsing and formatting
//
// TX (Controller → Display):
//   - Encoder position (configurable interval, default 100ms)
//   - FSM state changes (immediate)
//   - Error codes (immediate on error)
//
// RX (Display → Controller):
//   - actualMouldParams updates (write struct, respond with confirmation)
//   - commonInjectParams updates (write params, respond with all params)
//   - Query commands (respond with current values)

namespace DisplayComms {

// ===== CONFIGURATION =====
extern unsigned long broadcastInterval;  // Encoder broadcast interval (ms), set from config.h

// ===== INITIALIZATION =====
void begin();  // Initialize UART2 with pins from config.h

// ===== PERIODIC UPDATES (Call from loop()) =====
void update();  // Handle RX parsing and periodic TX broadcasts

// ===== TX: BROADCAST FUNCTIONS (Controller → Display) =====
void broadcastEncoder(float position, float velocity);  // Send encoder data
void broadcastState(InjectorStates state);              // Send FSM state change
void broadcastError(uint16_t errorCode, const char* errorMsg);  // Send error notification

// ===== TX: RESPONSE FUNCTIONS (Controller → Display) =====
void sendMouldParamsConfirm(const actualMouldParams_t& params);  // Confirm mould params received
void sendCommonParamsConfirm();  // Confirm common params received (send ALL current values)

// ===== RX: MESSAGE PARSING (Display → Controller) =====
void parseIncomingMessage(const char* message);  // Parse received SafeString message

// ===== MESSAGE FORMATS (SafeString Protocol) =====
//
// TX FORMATS (Controller → Display):
//   ENC|<position>|<velocity>               // Example: "ENC|45.23|-2.51"
//   STATE|<state_name>|<timestamp>          // Example: "STATE|INJECT|1234567890"
//   ERROR|<code>|<message>                  // Example: "ERROR|0x0200|MOTOR_SPINOUT"
//   MOULD_OK|<name>|<fill_vol>|...          // Example: "MOULD_OK|TestMould|35.0|25.0|..."
//   COMMON_OK|<homing_vel>|<refill_gap>|... // Example: "COMMON_OK|12.5|47.7|..."
//
// RX FORMATS (Display → Controller):
//   MOULD|<name>|<fill_vol>|<fill_speed>|<fill_pressure>|<pack_vol>|<pack_speed>|<pack_pressure>|<pack_time>|<cooling_time>|<fill_accel>|<fill_decel>|<pack_accel>|<pack_decel>
//     Example: "MOULD|TestMould|35.0|25.0|20.0|5.0|2.0|10.0|2.0|0.0|20.0|20.0|10.0|10.0"
//   
//   COMMON|<homing_vel>|<refill_gap>|<compression_torque>|<inject_vel>|<release_dist>|<antidrip_vel>|<purge_vel>
//     Example: "COMMON|12.5|47.7|10.0|20.0|-2.5|-2.0|2.0"
//   
//   QUERY_MOULD                              // Request current mould params
//   QUERY_COMMON                             // Request current common params
//   QUERY_STATE                              // Request current FSM state
//   QUERY_ERROR                              // Request current error code

// ===== INTERNAL STATE =====
struct DisplayCommsState {
    unsigned long lastEncoderBroadcast;
    InjectorStates lastStateBroadcast;
    uint16_t lastErrorBroadcast;
    // Note: rxBuffer initialized separately in DisplayComms.cpp as global SafeString
};

extern DisplayCommsState state;

} // namespace DisplayComms
