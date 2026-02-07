#ifndef PURGE_ZERO_H
#define PURGE_ZERO_H

#include "can_bus.h"

namespace PurgeZero {
    void begin();
    bool update(CanBus& motor, bool buttonUp, bool buttonDown, bool buttonCenterReleased);
    bool isComplete();
    bool hasError();
    float getPurgeZeroPosition();
    void reset();
}

#endif // PURGE_ZERO_H
