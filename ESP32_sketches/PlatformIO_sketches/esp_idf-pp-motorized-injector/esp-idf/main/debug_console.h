#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include "config.h"

#if APP_DEBUG_CONSOLE

#include "can_bus.h"

void debug_console_init(CanBus* motor);
void debug_console_poll();

#else

inline void debug_console_init(void*) {}
inline void debug_console_poll() {}

#endif

#endif // DEBUG_CONSOLE_H
