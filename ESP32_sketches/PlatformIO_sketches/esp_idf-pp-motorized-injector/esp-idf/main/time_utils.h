#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <cstdint>

namespace time_utils {
    // Returns microseconds since boot
    uint64_t micros();

    // Returns milliseconds since boot
    uint64_t millis();
}

#endif // TIME_UTILS_H
