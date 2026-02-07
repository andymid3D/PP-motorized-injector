#include "time_utils.h"
#include "esp_timer.h"

namespace time_utils {

uint64_t micros() {
    return static_cast<uint64_t>(esp_timer_get_time());
}

uint64_t millis() {
    return micros() / 1000ULL;
}

} // namespace time_utils
