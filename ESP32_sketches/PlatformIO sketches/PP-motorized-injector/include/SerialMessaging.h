#ifndef SERIAL_MESSAGING_H
#define SERIAL_MESSAGING_H

#include <Arduino.h>
#include <SafeString.h>

/**
 * SerialMessaging - Non-Blocking Serial Output Management
 * 
 * Purpose:
 *   - Use SafeString BufferedOutput for non-blocking serial I/O
 *   - Queue status messages from all modules without blocking
 *   - Centralized 1Hz debug output to avoid message collisions
 *   - Graceful handling of slow serial (115200 baud on ESP32)
 * 
 * Design:
 *   - Each module appends to a SafeString accumulator
 *   - Dispatcher prints once per loop (or periodically)
 *   - No Serial.println() calls outside this module
 *   - Uses SafeString's non-blocking capabilities
 */

class SerialMessaging {
public:
    // Buffer size constant
    static const size_t MAX_BUFFER_SIZE = 512;
    
    // ===== INITIALIZATION =====
    // Call once in setup()
    static void begin();
    
    // ===== PERIODIC PRINTING (call from main loop) =====
    // Print status at ~1Hz without blocking
    // Returns true if message was printed
    static bool printStatusMessage();
    
    // ===== MESSAGE QUEUING =====
    // Append to current status message (non-blocking)
    static void appendStatus(const char* msg);
    static void appendStatus(const char* label, float value);
    static void appendStatus(const char* label, int value);
    static void appendStatus(const char* label, uint8_t value);
    static void appendStatus(const char* label, bool value);
    
    // ===== ERROR/DEBUG LOGGING =====
    // Print immediately (blocking) - use sparingly
    static void printError(const char* msg);
    static void printDebug(const char* msg);
    static void printInfo(const char* msg);
    
    // ===== CLEAR BUFFER =====
    // Reset accumulator for next message cycle
    static void clearBuffer();
    
    // ===== CONFIGURATION =====
    // Set print frequency (default 1000ms = 1Hz)
    static void setPrintIntervalMs(uint32_t intervalMs);
    static uint32_t getPrintIntervalMs();
    
    // Get current buffer contents (for debugging)
    static const char* getBufferContents();

private:
    // Status message buffer (SafeString)
    static SafeString statusBuffer_;
    
    // Timing for 1Hz output
    static uint32_t lastPrintMs_;
    static uint32_t printIntervalMs_;
};

#endif // SERIAL_MESSAGING_H
