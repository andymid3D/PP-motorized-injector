#ifndef COMPRESSION_H
#define COMPRESSION_H

#include "can_bus.h"

namespace Compression {
    enum CompressionMode { MODE_1_TRAVEL, MODE_2_MICRO };

    void begin(CompressionMode mode);
    bool update(CanBus& motor);

    bool isComplete();
    bool hasError();
    bool isTimeout();

    void reset();
}

#endif // COMPRESSION_H
