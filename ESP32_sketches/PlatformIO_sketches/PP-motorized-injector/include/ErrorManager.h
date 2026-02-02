#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <Arduino.h>
#include "config.h"  // For DEBUG macros
#include "injector_fsm.h"

// ===== ERROR SEVERITY CLASSIFICATION =====
// Determines recovery strategy for each error type
enum ErrorSeverity {
    ERR_EXPECTED_TRANSIENT,    // Expected during normal operations (e.g., 0x100 during calibration)
    ERR_RECOVERABLE_RETRY,     // Clear + retry state 8 works (e.g., 0x40 phase estimate)
    ERR_RECOVERABLE_HOMING,    // Requires recalibration (driver reset, lost position)
    ERR_SAFETY_CRITICAL        // Hardware fault, requires user intervention
};

// ===== ERROR EVENT LOGGING =====
// Tracks all errors for diagnostics and pattern analysis
struct ErrorEvent {
    uint32_t axisError;
    uint64_t motorError;          // 64-bit for ODrive motor errors
    uint32_t encoderError;
    uint32_t controllerError;
    uint64_t timestamp;           // millis() when error occurred
    InjectorStates stateWhenOccurred;  // FSM state when error happened
};

#define ERROR_HISTORY_SIZE 20
extern ErrorEvent errorHistory[ERROR_HISTORY_SIZE];
extern uint8_t errorHistoryIndex;

// ===== AXIS ERROR TABLE (ODrive.Axis.Error) =====
// Source: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.Error
struct AxisErrorInfo {
    uint32_t code;
    const char* name;
    const char* description;
    ErrorSeverity severity;
};

const AxisErrorInfo AXIS_ERRORS[] = {
    {0x00000001, "INVALID_STATE", "Requested state not allowed (e.g., CLC before calibration)", ERR_RECOVERABLE_RETRY},
    {0x00000040, "MOTOR_FAILED", "Check motor.error for details", ERR_SAFETY_CRITICAL},
    {0x00000080, "SENSORLESS_ESTIMATOR_FAILED", "Sensorless estimator error", ERR_SAFETY_CRITICAL},
    {0x00000100, "ENCODER_FAILED", "Check encoder.error for details", ERR_RECOVERABLE_HOMING},
    {0x00000200, "CONTROLLER_FAILED", "Controller error occurred", ERR_SAFETY_CRITICAL},
    {0x00000800, "WATCHDOG_TIMER_EXPIRED", "Axis watchdog timeout", ERR_SAFETY_CRITICAL},
    {0x00001000, "MIN_ENDSTOP_PRESSED", "Min endstop triggered", ERR_SAFETY_CRITICAL},
    {0x00002000, "MAX_ENDSTOP_PRESSED", "Max endstop triggered", ERR_SAFETY_CRITICAL},
    {0x00004000, "ESTOP_REQUESTED", "E-stop via CAN", ERR_SAFETY_CRITICAL},
    {0x00020000, "HOMING_WITHOUT_ENDSTOP", "Homing requested but endstop not enabled", ERR_SAFETY_CRITICAL},
    {0x00040000, "OVER_TEMP", "Temperature exceeded limit", ERR_SAFETY_CRITICAL},
    {0x00080000, "UNKNOWN_POSITION", "No valid position estimate available", ERR_RECOVERABLE_HOMING}
};
const int AXIS_ERROR_COUNT = sizeof(AXIS_ERRORS) / sizeof(AXIS_ERRORS[0]);

// ===== MOTOR ERROR TABLE (ODrive.Motor.Error) =====
// Source: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Motor.Error
struct MotorErrorInfo {
    uint64_t code;                // 64-bit for ODrive motor errors
    const char* name;
    const char* description;
    ErrorSeverity severity;
};

const MotorErrorInfo MOTOR_ERRORS[] = {
    {0x00000001, "PHASE_RESISTANCE_OUT_OF_RANGE", "Measured resistance outside plausible range", ERR_SAFETY_CRITICAL},
    {0x00000002, "PHASE_INDUCTANCE_OUT_OF_RANGE", "Measured inductance outside plausible range", ERR_SAFETY_CRITICAL},
    {0x00000008, "DRV_FAULT", "Gate driver chip fault (hardware issue)", ERR_SAFETY_CRITICAL},
    {0x00000010, "CONTROL_DEADLINE_MISSED", "Control loop timing violation", ERR_SAFETY_CRITICAL},
    {0x00000080, "MODULATION_MAGNITUDE", "Bus voltage insufficient for requested current", ERR_RECOVERABLE_RETRY},
    {0x00000400, "CURRENT_SENSE_SATURATION", "Current sense amplifier saturated", ERR_SAFETY_CRITICAL},
    {0x00001000, "CURRENT_LIMIT_VIOLATION", "Current exceeded limit + margin", ERR_RECOVERABLE_RETRY},
    {0x00010000, "MODULATION_IS_NAN", "NaN in modulation calculation", ERR_SAFETY_CRITICAL},
    {0x00020000, "MOTOR_THERMISTOR_OVER_TEMP", "Motor thermistor over temp", ERR_SAFETY_CRITICAL},
    {0x00040000, "FET_THERMISTOR_OVER_TEMP", "Inverter thermistor over temp", ERR_SAFETY_CRITICAL},
    {0x00080000, "TIMER_UPDATE_MISSED", "Timer update event missed", ERR_SAFETY_CRITICAL},
    {0x00100000, "CURRENT_MEASUREMENT_UNAVAILABLE", "Phase current measurement not available", ERR_SAFETY_CRITICAL},
    {0x00200000, "CONTROLLER_FAILED", "FOC controller failed", ERR_SAFETY_CRITICAL},
    {0x00400000, "I_BUS_OUT_OF_RANGE", "DC current exceeded hard limits", ERR_SAFETY_CRITICAL},
    {0x00800000, "BRAKE_RESISTOR_DISARMED", "Brake resistor configured but disarmed", ERR_RECOVERABLE_RETRY},
    {0x01000000, "SYSTEM_LEVEL", "System-wide error (check ODrive.error)", ERR_SAFETY_CRITICAL},
    {0x02000000, "BAD_TIMING", "Control loop sync lost", ERR_SAFETY_CRITICAL},
    {0x04000000, "UNKNOWN_PHASE_ESTIMATE", "No valid angle input (calibrate encoder)", ERR_RECOVERABLE_HOMING},
    {0x08000000, "UNKNOWN_PHASE_VEL", "No valid phase velocity input", ERR_RECOVERABLE_HOMING},
    {0x10000000, "UNKNOWN_TORQUE", "No valid torque input", ERR_RECOVERABLE_RETRY},
    {0x20000000, "UNKNOWN_CURRENT_COMMAND", "No valid current setpoint", ERR_RECOVERABLE_RETRY},
    {0x40000000, "UNKNOWN_CURRENT_MEASUREMENT", "No valid current measurement", ERR_SAFETY_CRITICAL},
    {0x80000000, "UNKNOWN_VBUS_VOLTAGE", "No valid vbus measurement", ERR_SAFETY_CRITICAL},
    {0x100000000, "UNKNOWN_VOLTAGE_COMMAND", "The current controller did not get a valid feedforward voltage setpoint", ERR_RECOVERABLE_RETRY}
};
const int MOTOR_ERROR_COUNT = sizeof(MOTOR_ERRORS) / sizeof(MOTOR_ERRORS[0]);

// ===== ENCODER ERROR TABLE (ODrive.Encoder.Error) =====
// Source: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Encoder.Error
struct EncoderErrorInfo {
    uint32_t code;
    const char* name;
    const char* description;
    ErrorSeverity severity;
};

const EncoderErrorInfo ENCODER_ERRORS[] = {
    {0x00000001, "UNSTABLE_GAIN", "Encoder gain unstable", ERR_SAFETY_CRITICAL},
    {0x00000002, "CPR_POLEPAIRS_MISMATCH", "CPR/pole pairs config mismatch (KNOWN: recoverable)", ERR_EXPECTED_TRANSIENT},
    {0x00000004, "NO_RESPONSE", "Encoder not responding", ERR_SAFETY_CRITICAL},
    {0x00000008, "UNSUPPORTED_ENCODER_MODE", "Invalid encoder mode", ERR_SAFETY_CRITICAL},
    {0x00000010, "ILLEGAL_HALL_STATE", "Invalid hall effect state", ERR_SAFETY_CRITICAL},
    {0x00000020, "INDEX_NOT_FOUND_YET", "Index pulse not found", ERR_RECOVERABLE_RETRY},
    {0x00000040, "ABS_SPI_TIMEOUT", "Absolute SPI encoder timeout", ERR_SAFETY_CRITICAL},
    {0x00000080, "ABS_SPI_COM_FAIL", "Absolute SPI communication failure", ERR_SAFETY_CRITICAL},
    {0x00000100, "ABS_SPI_NOT_READY", "Absolute SPI encoder not ready (KNOWN: during calib)", ERR_EXPECTED_TRANSIENT},
    {0x00000200, "HALL_NOT_CALIBRATED_YET", "Hall effect encoder not calibrated", ERR_RECOVERABLE_HOMING}
};
const int ENCODER_ERROR_COUNT = sizeof(ENCODER_ERRORS) / sizeof(ENCODER_ERRORS[0]);

// ===== CONTROLLER ERROR TABLE (ODrive.Controller.Error) =====
// Source: https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.Error
struct ControllerErrorInfo {
    uint32_t code;
    const char* name;
    const char* description;
    ErrorSeverity severity;
};

const ControllerErrorInfo CONTROLLER_ERRORS[] = {
    {0x00000001, "OVERSPEED", "Velocity exceeded vel_limit * tolerance", ERR_RECOVERABLE_RETRY},
    {0x00000002, "INVALID_INPUT_MODE", "Invalid input_mode setting", ERR_SAFETY_CRITICAL},
    {0x00000004, "UNSTABLE_GAIN", "Bandwidth too high for stable control", ERR_SAFETY_CRITICAL},
    {0x00000008, "INVALID_MIRROR_AXIS", "Invalid axis_to_mirror selection", ERR_SAFETY_CRITICAL},
    {0x00000010, "INVALID_LOAD_ENCODER", "Invalid load_encoder_axis selection", ERR_SAFETY_CRITICAL},
    {0x00000020, "INVALID_ESTIMATE", "Encoder declined to output position/velocity", ERR_RECOVERABLE_HOMING},
    {0x00000040, "INVALID_CIRCULAR_RANGE", "Encoder declined circular position", ERR_RECOVERABLE_HOMING},
    {0x00000080, "SPINOUT_DETECTED", "Mechanical/electrical power mismatch (KNOWN: sudden contradictory move)", ERR_RECOVERABLE_RETRY}
};
const int CONTROLLER_ERROR_COUNT = sizeof(CONTROLLER_ERRORS) / sizeof(CONTROLLER_ERRORS[0]);

// ===== KNOWN ERROR OCCURRENCES (from conversation history) =====
// Documents errors actually encountered in hardware testing
struct KnownError {
    const char* errorType;  // "Motor", "Encoder", "Controller", "Axis"
    uint32_t code;
    const char* context;
    const char* recovery;
};

const KnownError KNOWN_ERRORS[] = {
    {"Encoder", 0x00000100, "Homing calibration request (State 7)", "Auto-clear, expected transient"},
    {"Encoder", 0x00000002, "Homing calibration (polepair mismatch hiccup)", "Auto-clear, retry calibration"},
    {"Motor", 0x04000000, "Requesting CLC without completed calibration", "Clear + State 8, or recalibrate"},
    {"Motor", 0x10000000, "Compression→Refill abort (sudden contradictory move)", "Clear + State 8"},
    {"Controller", 0x00000080, "Compression→Refill abort (sudden contradictory move)", "Clear + State 8"},
    {"Axis", 0x00000001, "Requesting CLC when encoder not ready", "Clear + State 8, ensure calibration complete"}
};
const int KNOWN_ERROR_COUNT = sizeof(KNOWN_ERRORS) / sizeof(KNOWN_ERRORS[0]);

// ===== FUNCTION DECLARATIONS =====

// Error logging
void logError(uint32_t axis, uint64_t motor, uint32_t encoder, uint32_t controller, InjectorStates state);  // 64-bit for ODrive motor errors
void printErrorHistory();
void clearErrorHistory();

// Error lookup and classification
ErrorSeverity classifyError(uint32_t axisError, uint64_t motorError, uint32_t encoderError, uint32_t controllerError);
const char* getAxisErrorName(uint32_t code);
const char* getMotorErrorName(uint64_t code);  // 64-bit for ODrive motor errors
const char* getEncoderErrorName(uint32_t code);
const char* getControllerErrorName(uint32_t code);

// Error description helpers (for debug output)
void printAxisError(uint32_t code);
void printMotorError(uint64_t code);  // 64-bit for ODrive motor errors
void printEncoderError(uint32_t code);
void printControllerError(uint32_t code);

// Check if any error is present
bool hasAnyError(uint32_t axis, uint64_t motor, uint32_t encoder, uint32_t controller);

// Centralized error recovery (extends existing ErrorSeverity system)
void handleRecoverableError(ErrorSeverity severity, uint32_t axisError, uint64_t motorError, uint32_t encoderError, uint32_t controllerError, bool moveComplete);

// ErrorManager status for SafetyManager coordination
bool errorManagerNeedsShutdown();  // Returns true if SafetyManager should trigger shutdown
void resetErrorManagerState();     // Clear retry counters and state

#endif // ERROR_MANAGER_H
