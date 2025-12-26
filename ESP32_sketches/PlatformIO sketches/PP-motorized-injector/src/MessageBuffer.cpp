#include "MessageBuffer.h"
#include <cstdarg>
#include <cstring>

MessageBuffer::MessageBuffer() : eventMessageLen_(0) {
    message1Hz_[0] = '\0';
    eventMessages_[0] = '\0';
    outputBuffer_[0] = '\0';
}

void MessageBuffer::sendMessage(const char* format, ...) {
    if (!format || eventMessageLen_ >= (EVENT_MESSAGE_SIZE - 100)) return;
    
    // Append newline and space before message
    if (eventMessageLen_ > 0) {
        eventMessages_[eventMessageLen_++] = '\n';
        eventMessages_[eventMessageLen_++] = ' ';
    }
    
    // Format message into temporary buffer
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Append to eventMessages
    size_t msgLen = strlen(buffer);
    if ((eventMessageLen_ + msgLen) < EVENT_MESSAGE_SIZE) {
        strcpy(eventMessages_ + eventMessageLen_, buffer);
        eventMessageLen_ += msgLen;
    }
}

void MessageBuffer::set1HzMessage(const char* format, ...) {
    if (!format) return;
    
    // Format message
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Copy to 1Hz buffer
    strncpy(message1Hz_, buffer, MESSAGE_1HZ_SIZE - 1);
    message1Hz_[MESSAGE_1HZ_SIZE - 1] = '\0';
}

const char* MessageBuffer::getOutput() {
    outputBuffer_[0] = '\0';
    
    // Section 1: 1Hz status messages (if available)
    if (message1Hz_[0] != '\0') {
        snprintf(outputBuffer_, OUTPUT_BUFFER_SIZE, " [%s]", message1Hz_);
    }
    
    // Section 2: Event messages (separated by newline)
    if (eventMessages_[0] != '\0') {
        size_t currentLen = strlen(outputBuffer_);
        if (currentLen > 0 && currentLen < (OUTPUT_BUFFER_SIZE - 10)) {
            strcat(outputBuffer_, "\n");
        }
        if ((currentLen + eventMessageLen_) < OUTPUT_BUFFER_SIZE) {
            strcat(outputBuffer_, eventMessages_);
        }
    }
    
    return outputBuffer_;
}

void MessageBuffer::clearBuffer() {
    eventMessages_[0] = '\0';
    eventMessageLen_ = 0;
}
