#ifndef READY_TO_INJECT_H
#define READY_TO_INJECT_H

#include "can_bus.h"

namespace ReadyToInject {
    void begin();
    bool update(CanBus& motor);

    bool isComplete();
    bool isMicroCompressing();
    bool hasError();

    void reset();
}

#endif // READY_TO_INJECT_H
