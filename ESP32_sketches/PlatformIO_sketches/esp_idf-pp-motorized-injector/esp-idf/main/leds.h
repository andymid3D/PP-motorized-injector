#ifndef LEDS_H
#define LEDS_H

#include <cstdint>
#include "injector_fsm.h"
#include "config.h"

void leds_init();
void leds_update(InjectorStates state, bool endOfDay, bool anyButtonPressed);

#endif // LEDS_H
