#ifndef REFILL_H
#define REFILL_H

#include "can_bus.h"

namespace Refill {
    void begin();
    bool update(CanBus& motor);
    bool isComplete();
    bool hasError();
    void reset();
}

#endif // REFILL_H
