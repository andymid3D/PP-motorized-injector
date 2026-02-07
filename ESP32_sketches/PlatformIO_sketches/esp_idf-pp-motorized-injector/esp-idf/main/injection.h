#ifndef INJECTION_H
#define INJECTION_H

#include "can_bus.h"
#include "injector_fsm.h"

namespace Injection {
    enum InjectionPhase { FILLING, PACKING, DONE };

    void begin(const actualMouldParams_t& mouldParams);
    bool update(CanBus& motor);

    bool isComplete();
    bool hasError();
    InjectionPhase getPhase();

    float getInjectStartPos();
    float getTargetInjectPos();
    float getTargetPackPos();
    float getInjectVolumeFromPos(float currentPos);
    float getTargetInjectVolume();

    void reset();
}

#endif // INJECTION_H
