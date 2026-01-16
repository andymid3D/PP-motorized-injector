#include "ErrorManager.h"
#include "config.h"  // For DEBUG macros
#include <Arduino.h>

// ===== GLOBAL ERROR HISTORY =====
ErrorEvent errorHistory[ERROR_HISTORY_SIZE];
uint8_t errorHistoryIndex = 0;

// ===== ERROR LOGGING =====
void logError(uint32_t axis, uint32_t motor, uint32_t encoder, uint32_t controller, InjectorStates state) {
    // Store error event in circular buffer
    errorHistory[errorHistoryIndex].axisError = axis;
    errorHistory[errorHistoryIndex].motorError = motor;
    errorHistory[errorHistoryIndex].encoderError = encoder;
    errorHistory[errorHistoryIndex].controllerError = controller;
    errorHistory[errorHistoryIndex].stateWhenOccurred = state;
    errorHistory[errorHistoryIndex].timestamp = millis();
    
    errorHistoryIndex = (errorHistoryIndex + 1) % ERROR_HISTORY_SIZE;
    
    #if DEBUG_ENABLED
    // Print to serial for immediate debug visibility
    Serial.print("ERROR LOGGED | State: ");
    Serial.print(static_cast<int>(state));
    Serial.print(" | AX:0x");
    Serial.print(axis, HEX);
    Serial.print(" MX:0x");
    Serial.print(motor, HEX);
    Serial.print(" EX:0x");
    Serial.print(encoder, HEX);
    Serial.print(" CX:0x");
    Serial.print(controller, HEX);
    Serial.print(" | Time: ");
    Serial.println(millis());
    #endif
}

void printErrorHistory() {
    DEBUG_PRINTLN("===== ERROR HISTORY =====");
    for (int i = 0; i < ERROR_HISTORY_SIZE; i++) {
        int idx = (errorHistoryIndex + i) % ERROR_HISTORY_SIZE;
        if (errorHistory[idx].timestamp == 0) continue;  // Skip empty entries
        
        DEBUG_PRINT(i + 1);
        DEBUG_PRINT(". State: ");
        DEBUG_PRINT(static_cast<int>(errorHistory[idx].stateWhenOccurred));
        DEBUG_PRINT(" | AX:0x");
        DEBUG_PRINT(errorHistory[idx].axisError, HEX);
        DEBUG_PRINT(" MX:0x");
        DEBUG_PRINT(errorHistory[idx].motorError, HEX);
        DEBUG_PRINT(" EX:0x");
        DEBUG_PRINT(errorHistory[idx].encoderError, HEX);
        DEBUG_PRINT(" CX:0x");
        DEBUG_PRINT(errorHistory[idx].controllerError, HEX);
        DEBUG_PRINT(" | Time: ");
        DEBUG_PRINTLN(errorHistory[idx].timestamp);
    }
    DEBUG_PRINTLN("========================");
}

void clearErrorHistory() {
    for (int i = 0; i < ERROR_HISTORY_SIZE; i++) {
        errorHistory[i].axisError = 0;
        errorHistory[i].motorError = 0;
        errorHistory[i].encoderError = 0;
        errorHistory[i].controllerError = 0;
        errorHistory[i].stateWhenOccurred = InjectorStates::ERROR_STATE;
        errorHistory[i].timestamp = 0;
    }
    errorHistoryIndex = 0;
    DEBUG_PRINTLN("Error history cleared");
}

// ===== ERROR CLASSIFICATION =====
ErrorSeverity classifyError(uint32_t axisError, uint32_t motorError, uint32_t encoderError, uint32_t controllerError) {
    ErrorSeverity worstSeverity = ERR_EXPECTED_TRANSIENT;
    
    // Check axis errors
    for (int i = 0; i < AXIS_ERROR_COUNT; i++) {
        if (axisError & AXIS_ERRORS[i].code) {
            if (AXIS_ERRORS[i].severity > worstSeverity) {
                worstSeverity = AXIS_ERRORS[i].severity;
            }
        }
    }
    
    // Check motor errors
    for (int i = 0; i < MOTOR_ERROR_COUNT; i++) {
        if (motorError & MOTOR_ERRORS[i].code) {
            if (MOTOR_ERRORS[i].severity > worstSeverity) {
                worstSeverity = MOTOR_ERRORS[i].severity;
            }
        }
    }
    
    // Check encoder errors
    for (int i = 0; i < ENCODER_ERROR_COUNT; i++) {
        if (encoderError & ENCODER_ERRORS[i].code) {
            if (ENCODER_ERRORS[i].severity > worstSeverity) {
                worstSeverity = ENCODER_ERRORS[i].severity;
            }
        }
    }
    
    // Check controller errors
    for (int i = 0; i < CONTROLLER_ERROR_COUNT; i++) {
        if (controllerError & CONTROLLER_ERRORS[i].code) {
            if (CONTROLLER_ERRORS[i].severity > worstSeverity) {
                worstSeverity = CONTROLLER_ERRORS[i].severity;
            }
        }
    }
    
    return worstSeverity;
}

// ===== ERROR NAME LOOKUP =====
const char* getAxisErrorName(uint32_t code) {
    for (int i = 0; i < AXIS_ERROR_COUNT; i++) {
        if (code == AXIS_ERRORS[i].code) {
            return AXIS_ERRORS[i].name;
        }
    }
    return "UNKNOWN_AXIS_ERROR";
}

const char* getMotorErrorName(uint32_t code) {
    for (int i = 0; i < MOTOR_ERROR_COUNT; i++) {
        if (code == MOTOR_ERRORS[i].code) {
            return MOTOR_ERRORS[i].name;
        }
    }
    return "UNKNOWN_MOTOR_ERROR";
}

const char* getEncoderErrorName(uint32_t code) {
    for (int i = 0; i < ENCODER_ERROR_COUNT; i++) {
        if (code == ENCODER_ERRORS[i].code) {
            return ENCODER_ERRORS[i].name;
        }
    }
    return "UNKNOWN_ENCODER_ERROR";
}

const char* getControllerErrorName(uint32_t code) {
    for (int i = 0; i < CONTROLLER_ERROR_COUNT; i++) {
        if (code == CONTROLLER_ERRORS[i].code) {
            return CONTROLLER_ERRORS[i].name;
        }
    }
    return "UNKNOWN_CONTROLLER_ERROR";
}

// ===== ERROR DESCRIPTION PRINTING =====
void printAxisError(uint32_t code) {
    DEBUG_PRINT("AXIS ERROR 0x");
    DEBUG_PRINT(code, HEX);
    DEBUG_PRINT(": ");
    for (int i = 0; i < AXIS_ERROR_COUNT; i++) {
        if (code & AXIS_ERRORS[i].code) {
            DEBUG_PRINT(AXIS_ERRORS[i].name);
            DEBUG_PRINT(" (");
            DEBUG_PRINT(AXIS_ERRORS[i].description);
            DEBUG_PRINT(") ");
        }
    }
    DEBUG_PRINTLN();
}

void printMotorError(uint32_t code) {
    DEBUG_PRINT("MOTOR ERROR 0x");
    DEBUG_PRINT(code, HEX);
    DEBUG_PRINT(": ");
    for (int i = 0; i < MOTOR_ERROR_COUNT; i++) {
        if (code & MOTOR_ERRORS[i].code) {
            DEBUG_PRINT(MOTOR_ERRORS[i].name);
            DEBUG_PRINT(" (");
            DEBUG_PRINT(MOTOR_ERRORS[i].description);
            DEBUG_PRINT(") ");
        }
    }
    DEBUG_PRINTLN();
}

void printEncoderError(uint32_t code) {
    DEBUG_PRINT("ENCODER ERROR 0x");
    DEBUG_PRINT(code, HEX);
    DEBUG_PRINT(": ");
    for (int i = 0; i < ENCODER_ERROR_COUNT; i++) {
        if (code & ENCODER_ERRORS[i].code) {
            DEBUG_PRINT(ENCODER_ERRORS[i].name);
            DEBUG_PRINT(" (");
            DEBUG_PRINT(ENCODER_ERRORS[i].description);
            DEBUG_PRINT(") ");
        }
    }
    DEBUG_PRINTLN();
}

void printControllerError(uint32_t code) {
    DEBUG_PRINT("CONTROLLER ERROR 0x");
    DEBUG_PRINT(code, HEX);
    DEBUG_PRINT(": ");
    for (int i = 0; i < CONTROLLER_ERROR_COUNT; i++) {
        if (code & CONTROLLER_ERRORS[i].code) {
            DEBUG_PRINT(CONTROLLER_ERRORS[i].name);
            DEBUG_PRINT(" (");
            DEBUG_PRINT(CONTROLLER_ERRORS[i].description);
            DEBUG_PRINT(") ");
        }
    }
    DEBUG_PRINTLN();
}

// ===== ERROR CHECKING =====
bool hasAnyError(uint32_t axis, uint32_t motor, uint32_t encoder, uint32_t controller) {
    return (axis != 0 || motor != 0 || encoder != 0 || controller != 0);
}
