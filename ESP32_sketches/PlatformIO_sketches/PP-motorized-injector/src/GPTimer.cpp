#include "GPTimer.h"

// Global instance definition
GPTimer hwTimer;

GPTimer::GPTimer() 
    : timerHandle_(nullptr) {
}

GPTimer::~GPTimer() {
    if (timerHandle_) {
        timerEnd(timerHandle_);
        timerHandle_ = nullptr;
    }
}

bool GPTimer::begin() {
    if (timerHandle_) {
        Serial.println("GPTimer: Already initialized");
        return true;
    }
    
    // Configure hardware timer:
    // - Timer 0 (of 4 available)
    // - 1MHz frequency (1 microsecond resolution)
    // - Count up mode
    timerHandle_ = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (80MHz/80=1MHz), count up
    
    if (!timerHandle_) {
        Serial.println("ERROR: GPTimer create failed");
        return false;
    }
    
    // Start timer from zero
    timerStart(timerHandle_);
    
    Serial.println("GPTimer: Initialized (1MHz / 1µs resolution)");
    return true;
}

uint64_t IRAM_ATTR GPTimer::micros() const {
    if (!timerHandle_) {
        return 0;
    }
    
    return timerRead(timerHandle_);
}

void GPTimer::reset() {
    if (timerHandle_) {
        timerWrite(timerHandle_, 0);
        Serial.println("GPTimer: Reset to zero");
    }
}
