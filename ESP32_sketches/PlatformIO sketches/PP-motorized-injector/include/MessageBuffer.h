#ifndef MESSAGE_BUFFER_H
#define MESSAGE_BUFFER_H

#include <Arduino.h>

/**
 * @class MessageBuffer
 * @brief Central message buffering system for all diagnostic and debug output
 * 
 * Purpose:
 * - Centralize all serial output to prevent spam and interleaving
 * - Allow any module to queue messages via sendMessage()
 * - Output organized in two sections:
 *   1. 1Hz repetitive messages (axis state, position, velocity, temperature, etc.)
 *   2. Event-based messages (state transitions, errors, debug info)
 * 
 * Usage:
 *   MessageBuffer::getInstance().sendMessage("Motor error: 0x%X", error_code);
 *   // Output appears in next main loop flush
 */
class MessageBuffer {
public:
    static MessageBuffer& getInstance() {
        static MessageBuffer instance;
        return instance;
    }
    
    /**
     * Queue a message for output (thread-safe, non-blocking)
     * Formatted similarly to sprintf/printf
     */
    void sendMessage(const char* format, ...);
    
    /**
     * Get the combined output string (1Hz messages + event messages)
     * Call this once per main loop to get complete output
     * Returns pointer to internal buffer
     */
    const char* getOutput();
    
    /**
     * Clear all buffered messages after output
     */
    void clearBuffer();
    
    /**
     * Add to 1Hz message section (called from main.cpp status logging)
     * These appear at top of output block every second
     */
    void set1HzMessage(const char* format, ...);
    
private:
    MessageBuffer();
    ~MessageBuffer() = default;
    
    // Prevent copying
    MessageBuffer(const MessageBuffer&) = delete;
    MessageBuffer& operator=(const MessageBuffer&) = delete;
    
    // Buffer sizes
    static constexpr size_t MESSAGE_1HZ_SIZE = 512;
    static constexpr size_t EVENT_MESSAGE_SIZE = 2048;
    static constexpr size_t OUTPUT_BUFFER_SIZE = 3000;
    
    // Internal buffers
    char message1Hz_[MESSAGE_1HZ_SIZE];
    char eventMessages_[EVENT_MESSAGE_SIZE];
    char outputBuffer_[OUTPUT_BUFFER_SIZE];
    
    // Track current positions
    size_t eventMessageLen_;
};

#endif  // MESSAGE_BUFFER_H

