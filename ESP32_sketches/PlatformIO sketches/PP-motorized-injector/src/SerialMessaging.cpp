#include "SerialMessaging.h"

// ===== STATIC MEMBER INITIALIZATION =====
// SafeString requires a pre-allocated buffer
static char g_statusBufferSpace[SerialMessaging::MAX_BUFFER_SIZE];
SafeString SerialMessaging::statusBuffer_(SerialMessaging::MAX_BUFFER_SIZE, g_statusBufferSpace, "statusBuffer");

uint32_t SerialMessaging::lastPrintMs_ = 0;
uint32_t SerialMessaging::printIntervalMs_ = 1000;  // 1Hz default

void SerialMessaging::begin() {
    // Initialize SafeString for buffered output
    SafeString::setOutput(Serial);  // Already set in main.cpp, but ensure it's set
    statusBuffer_.clear();
    lastPrintMs_ = millis();
}

bool SerialMessaging::printStatusMessage() {
    // Check if it's time to print (1Hz default)
    uint32_t now = millis();
    if (now - lastPrintMs_ < printIntervalMs_) {
        return false;  // Not time yet
    }
    
    lastPrintMs_ = now;
    
    // Print accumulated buffer
    if (statusBuffer_.length() > 0) {
        Serial.println(statusBuffer_);  // Non-blocking due to SafeString BufferedOutput
        statusBuffer_.clear();
        return true;
    }
    
    return false;
}

void SerialMessaging::appendStatus(const char* msg) {
    if (statusBuffer_.length() + strlen(msg) < MAX_BUFFER_SIZE - 10) {
        statusBuffer_ += msg;
    }
}

void SerialMessaging::appendStatus(const char* label, float value) {
    if (statusBuffer_.length() + 50 < MAX_BUFFER_SIZE) {
        statusBuffer_ += " ";
        statusBuffer_ += label;
        statusBuffer_ += ":";
        statusBuffer_ += value;
    }
}

void SerialMessaging::appendStatus(const char* label, int value) {
    if (statusBuffer_.length() + 50 < MAX_BUFFER_SIZE) {
        statusBuffer_ += " ";
        statusBuffer_ += label;
        statusBuffer_ += ":";
        statusBuffer_ += value;
    }
}

void SerialMessaging::appendStatus(const char* label, uint8_t value) {
    if (statusBuffer_.length() + 50 < MAX_BUFFER_SIZE) {
        statusBuffer_ += " ";
        statusBuffer_ += label;
        statusBuffer_ += ":";
        statusBuffer_ += (int)value;
    }
}

void SerialMessaging::appendStatus(const char* label, bool value) {
    if (statusBuffer_.length() + 50 < MAX_BUFFER_SIZE) {
        statusBuffer_ += " ";
        statusBuffer_ += label;
        statusBuffer_ += ":";
        statusBuffer_ += (value ? "true" : "false");
    }
}

void SerialMessaging::printError(const char* msg) {
    // Errors are important - use direct Serial output
    Serial.print("[ERROR] ");
    Serial.println(msg);
}

void SerialMessaging::printDebug(const char* msg) {
    Serial.print("[DEBUG] ");
    Serial.println(msg);
}

void SerialMessaging::printInfo(const char* msg) {
    Serial.print("[INFO] ");
    Serial.println(msg);
}

void SerialMessaging::clearBuffer() {
    statusBuffer_.clear();
}

void SerialMessaging::setPrintIntervalMs(uint32_t intervalMs) {
    printIntervalMs_ = intervalMs;
}

uint32_t SerialMessaging::getPrintIntervalMs() {
    return printIntervalMs_;
}

const char* SerialMessaging::getBufferContents() {
    return statusBuffer_.c_str();
}
