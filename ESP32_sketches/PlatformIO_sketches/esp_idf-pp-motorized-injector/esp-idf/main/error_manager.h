#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <cstdint>
#include "injector_fsm.h"

enum ErrorSeverity {
    ERR_EXPECTED_TRANSIENT,
    ERR_RECOVERABLE_RETRY,
    ERR_RECOVERABLE_HOMING,
    ERR_SAFETY_CRITICAL
};

struct ErrorEvent {
    uint32_t axisError;
    uint64_t motorError;
    uint32_t encoderError;
    uint32_t controllerError;
    uint64_t timestamp;
    InjectorStates stateWhenOccurred;
};

#define ERROR_HISTORY_SIZE 20
extern ErrorEvent errorHistory[ERROR_HISTORY_SIZE];
extern uint8_t errorHistoryIndex;

struct AxisErrorInfo {
    uint32_t code;
    const char* name;
    const char* description;
    ErrorSeverity severity;
};

struct MotorErrorInfo {
    uint64_t code;
    const char* name;
    const char* description;
    ErrorSeverity severity;
};

struct EncoderErrorInfo {
    uint32_t code;
    const char* name;
    const char* description;
    ErrorSeverity severity;
};

struct ControllerErrorInfo {
    uint32_t code;
    const char* name;
    const char* description;
    ErrorSeverity severity;
};

extern const AxisErrorInfo AXIS_ERRORS[];
extern const int AXIS_ERROR_COUNT;
extern const MotorErrorInfo MOTOR_ERRORS[];
extern const int MOTOR_ERROR_COUNT;
extern const EncoderErrorInfo ENCODER_ERRORS[];
extern const int ENCODER_ERROR_COUNT;
extern const ControllerErrorInfo CONTROLLER_ERRORS[];
extern const int CONTROLLER_ERROR_COUNT;

void logError(uint32_t axis, uint64_t motor, uint32_t encoder, uint32_t controller, InjectorStates state);
ErrorSeverity classifyError(uint32_t axisError, uint64_t motorError, uint32_t encoderError, uint32_t controllerError);

bool hasAnyError(uint32_t axis, uint64_t motor, uint32_t encoder, uint32_t controller);

void handleRecoverableError(ErrorSeverity severity, uint32_t axisError, uint64_t motorError, uint32_t encoderError, uint32_t controllerError, bool moveComplete);

bool errorManagerNeedsShutdown();
void resetErrorManagerState();

#endif // ERROR_MANAGER_H
