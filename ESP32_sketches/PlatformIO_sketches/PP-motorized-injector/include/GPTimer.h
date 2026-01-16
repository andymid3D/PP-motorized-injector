#ifndef GPTIMER_H
#define GPTIMER_H

#include <Arduino.h>

/**
 * GPTimer - Hardware Timer Wrapper for 1µs Resolution Timestamps
 * 
 * Purpose: Provide immune-to-ISR timestamp source for CAN message capture
 * Resolution: 1 microsecond (1MHz counter)
 * Performance: <300ns read time (vs ~1500ns for micros())
 * 
 * Uses ESP32 hardware timer (hw_timer_t) in counter mode
 * 
 * Usage:
 *   hwTimer.begin();              // Initialize (call once in setup)
 *   uint64_t now = hwTimer.micros();  // Get current time in microseconds
 *   
 * Integration:
 *   - Used by CanRxHandler ISR for CAN message timestamps
 *   - Used by BroadcastDataStore for arrival time tracking
 *   - Replaces micros() in performance-critical paths
 */
class GPTimer {
public:
    GPTimer();
    ~GPTimer();
    
    /**
     * Initialize hardware timer (1MHz resolution)
     * Returns: true if successful, false on error
     * Call once in setup() before any CAN operations
     */
    bool begin();
    
    /**
     * Get current timestamp in microseconds
     * IRAM_ATTR: Fast inline access (~200-300ns)
     * Returns: 64-bit microsecond counter (wraps after 584,942 years)
     */
    uint64_t IRAM_ATTR micros() const;
    
    /**
     * Reset timer to zero (optional, mainly for testing)
     */
    void reset();
    
    /**
     * Check if timer is running
     */
    bool isRunning() const { return timerHandle_ != nullptr; }
    
private:
    hw_timer_t* timerHandle_;
    
    // Prevent copying
    GPTimer(const GPTimer&) = delete;
    GPTimer& operator=(const GPTimer&) = delete;
};

// Global instance - accessible from all modules
extern GPTimer hwTimer;

#endif // GPTIMER_H
