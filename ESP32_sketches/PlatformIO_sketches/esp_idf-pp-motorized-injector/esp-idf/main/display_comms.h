#ifndef DISPLAY_COMMS_H
#define DISPLAY_COMMS_H

#include <cstdint>
#include "config.h"
#include "injector_fsm.h"
#include "mould.h"

#if APP_DISPLAY_UART

namespace DisplayComms {

void begin();
void update();

void broadcastEncoder(float position, float velocity);
void broadcastState(InjectorStates state);
void broadcastError(uint16_t errorCode, const char* errorMsg);

void sendMouldParamsConfirm(const actualMouldParams_t& params);
void sendCommonParamsConfirm();

void parseIncomingMessage(const char* message);

} // namespace DisplayComms

#else

namespace DisplayComms {

inline void begin() {}
inline void update() {}
inline void broadcastEncoder(float, float) {}
inline void broadcastState(InjectorStates) {}
inline void broadcastError(uint16_t, const char*) {}
inline void sendMouldParamsConfirm(const actualMouldParams_t&) {}
inline void sendCommonParamsConfirm() {}
inline void parseIncomingMessage(const char*) {}

} // namespace DisplayComms

#endif // APP_DISPLAY_UART

#endif // DISPLAY_COMMS_H
