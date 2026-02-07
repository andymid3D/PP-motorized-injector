#ifndef ANTIDRIP_H
#define ANTIDRIP_H

#include "can_bus.h"

namespace AntiDrip {
    void begin();
    bool update(CanBus& motor);

    bool isComplete();
    bool isTimeout();
    bool isAborted();
    bool hasError();

    void reset();
}

#endif // ANTIDRIP_H
