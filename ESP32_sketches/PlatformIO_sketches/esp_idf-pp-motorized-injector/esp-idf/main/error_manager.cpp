#include "error_manager.h"
#include "config.h"
#include "can_bus.h"
#include "safety_manager.h"
#include "time_utils.h"
#include "odrive_can.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ERROR_MGR";

ErrorEvent errorHistory[ERROR_HISTORY_SIZE];
uint8_t errorHistoryIndex = 0;

const AxisErrorInfo AXIS_ERRORS[] = {
    {0x00000001, "INVALID_STATE", "Requested state not allowed", ERR_RECOVERABLE_RETRY},
    {0x00000040, "MOTOR_FAILED", "Check motor.error", ERR_RECOVERABLE_RETRY},
    {0x00000080, "SENSORLESS_ESTIMATOR_FAILED", "Sensorless estimator error", ERR_SAFETY_CRITICAL},
    {0x00000100, "ENCODER_FAILED", "Check encoder.error", ERR_RECOVERABLE_RETRY},
    {0x00000200, "CONTROLLER_FAILED", "Controller error", ERR_RECOVERABLE_RETRY},
    {0x00000800, "WATCHDOG_TIMER_EXPIRED", "Axis watchdog timeout", ERR_SAFETY_CRITICAL},
    {0x00001000, "MIN_ENDSTOP_PRESSED", "Min endstop", ERR_SAFETY_CRITICAL},
    {0x00002000, "MAX_ENDSTOP_PRESSED", "Max endstop", ERR_SAFETY_CRITICAL},
    {0x00004000, "ESTOP_REQUESTED", "E-stop via CAN", ERR_SAFETY_CRITICAL},
    {0x00020000, "HOMING_WITHOUT_ENDSTOP", "Homing requested but endstop not enabled", ERR_SAFETY_CRITICAL},
    {0x00040000, "OVER_TEMP", "Temperature exceeded limit", ERR_SAFETY_CRITICAL},
    {0x00080000, "UNKNOWN_POSITION", "No valid position estimate", ERR_RECOVERABLE_HOMING}
};
const int AXIS_ERROR_COUNT = sizeof(AXIS_ERRORS) / sizeof(AXIS_ERRORS[0]);

const MotorErrorInfo MOTOR_ERRORS[] = {
    {0x00000001, "PHASE_RESISTANCE_OUT_OF_RANGE", "Resistance outside range", ERR_SAFETY_CRITICAL},
    {0x00000002, "PHASE_INDUCTANCE_OUT_OF_RANGE", "Inductance outside range", ERR_SAFETY_CRITICAL},
    {0x00000008, "DRV_FAULT", "Gate driver fault", ERR_SAFETY_CRITICAL},
    {0x00000010, "CONTROL_DEADLINE_MISSED", "Control loop timing", ERR_SAFETY_CRITICAL},
    {0x00000080, "MODULATION_MAGNITUDE", "Bus voltage insufficient", ERR_RECOVERABLE_RETRY},
    {0x00000400, "CURRENT_SENSE_SATURATION", "Current sense saturated", ERR_SAFETY_CRITICAL},
    {0x00001000, "CURRENT_LIMIT_VIOLATION", "Current exceeded limit", ERR_RECOVERABLE_RETRY},
    {0x00010000, "MODULATION_IS_NAN", "NaN in modulation", ERR_SAFETY_CRITICAL},
    {0x00020000, "MOTOR_THERMISTOR_OVER_TEMP", "Motor thermistor over temp", ERR_SAFETY_CRITICAL},
    {0x00040000, "FET_THERMISTOR_OVER_TEMP", "Inverter thermistor over temp", ERR_SAFETY_CRITICAL},
    {0x00080000, "TIMER_UPDATE_MISSED", "Timer update missed", ERR_SAFETY_CRITICAL},
    {0x00100000, "CURRENT_MEASUREMENT_UNAVAILABLE", "Current measurement missing", ERR_SAFETY_CRITICAL},
    {0x00200000, "CONTROLLER_FAILED", "FOC controller failed", ERR_SAFETY_CRITICAL},
    {0x00400000, "I_BUS_OUT_OF_RANGE", "DC current exceeded limits", ERR_SAFETY_CRITICAL},
    {0x00800000, "BRAKE_RESISTOR_DISARMED", "Brake resistor disarmed", ERR_RECOVERABLE_RETRY},
    {0x01000000, "SYSTEM_LEVEL", "System-wide error", ERR_RECOVERABLE_RETRY},
    {0x02000000, "BAD_TIMING", "Control loop sync lost", ERR_SAFETY_CRITICAL},
    {0x04000000, "UNKNOWN_PHASE_ESTIMATE", "No valid angle input", ERR_RECOVERABLE_HOMING},
    {0x08000000, "UNKNOWN_PHASE_VEL", "No valid phase velocity", ERR_RECOVERABLE_HOMING},
    {0x10000000, "UNKNOWN_TORQUE", "No valid torque input", ERR_RECOVERABLE_RETRY},
    {0x20000000, "UNKNOWN_CURRENT_COMMAND", "No current setpoint", ERR_RECOVERABLE_RETRY},
    {0x40000000, "UNKNOWN_CURRENT_MEASUREMENT", "No current measurement", ERR_SAFETY_CRITICAL},
    {0x80000000, "UNKNOWN_VBUS_VOLTAGE", "No vbus measurement", ERR_SAFETY_CRITICAL},
    {0x100000000, "UNKNOWN_VOLTAGE_COMMAND", "No voltage setpoint", ERR_RECOVERABLE_RETRY}
};
const int MOTOR_ERROR_COUNT = sizeof(MOTOR_ERRORS) / sizeof(MOTOR_ERRORS[0]);

const EncoderErrorInfo ENCODER_ERRORS[] = {
    {0x00000001, "UNSTABLE_GAIN", "Encoder gain unstable", ERR_SAFETY_CRITICAL},
    {0x00000002, "CPR_POLEPAIRS_MISMATCH", "CPR/pole pairs mismatch", ERR_EXPECTED_TRANSIENT},
    {0x00000004, "NO_RESPONSE", "Encoder not responding", ERR_SAFETY_CRITICAL},
    {0x00000008, "UNSUPPORTED_ENCODER_MODE", "Invalid encoder mode", ERR_SAFETY_CRITICAL},
    {0x00000010, "ILLEGAL_HALL_STATE", "Invalid hall state", ERR_SAFETY_CRITICAL},
    {0x00000020, "INDEX_NOT_FOUND_YET", "Index not found yet", ERR_RECOVERABLE_RETRY},
    {0x00000040, "ABS_SPI_TIMEOUT", "ABS SPI timeout", ERR_SAFETY_CRITICAL},
    {0x00000080, "ABS_SPI_COM_FAIL", "ABS SPI fail", ERR_SAFETY_CRITICAL},
    {0x00000100, "ABS_SPI_NOT_READY", "ABS SPI not ready", ERR_EXPECTED_TRANSIENT},
    {0x00000200, "HALL_NOT_CALIBRATED_YET", "Hall not calibrated", ERR_RECOVERABLE_HOMING}
};
const int ENCODER_ERROR_COUNT = sizeof(ENCODER_ERRORS) / sizeof(ENCODER_ERRORS[0]);

const ControllerErrorInfo CONTROLLER_ERRORS[] = {
    {0x00000001, "OVERSPEED", "Velocity exceeded limit", ERR_RECOVERABLE_RETRY},
    {0x00000002, "INVALID_INPUT_MODE", "Invalid input_mode", ERR_SAFETY_CRITICAL},
    {0x00000004, "UNSTABLE_GAIN", "Bandwidth too high", ERR_SAFETY_CRITICAL},
    {0x00000008, "INVALID_MIRROR_AXIS", "Invalid axis_to_mirror", ERR_SAFETY_CRITICAL},
    {0x00000010, "INVALID_LOAD_ENCODER", "Invalid load encoder axis", ERR_SAFETY_CRITICAL},
    {0x00000020, "INVALID_ESTIMATE", "Invalid estimate", ERR_RECOVERABLE_HOMING},
    {0x00000040, "INVALID_CIRCULAR_RANGE", "Invalid circular range", ERR_RECOVERABLE_HOMING},
    {0x00000080, "SPINOUT_DETECTED", "Spinout detected", ERR_RECOVERABLE_RETRY}
};
const int CONTROLLER_ERROR_COUNT = sizeof(CONTROLLER_ERRORS) / sizeof(CONTROLLER_ERRORS[0]);

void logError(uint32_t axis, uint64_t motor, uint32_t encoder, uint32_t controller, InjectorStates state) {
    errorHistory[errorHistoryIndex].axisError = axis;
    errorHistory[errorHistoryIndex].motorError = motor;
    errorHistory[errorHistoryIndex].encoderError = encoder;
    errorHistory[errorHistoryIndex].controllerError = controller;
    errorHistory[errorHistoryIndex].stateWhenOccurred = state;
    errorHistory[errorHistoryIndex].timestamp = time_utils::micros();
    errorHistoryIndex = (errorHistoryIndex + 1) % ERROR_HISTORY_SIZE;

#if APP_DEBUG
    ESP_LOGI(TAG, "ERROR LOGGED | State:%d AX:0x%08X MX:0x%llX EX:0x%08X CX:0x%08X", (int)state, axis, (unsigned long long)motor, encoder, controller);
#endif
}

ErrorSeverity classifyError(uint32_t axisError, uint64_t motorError, uint32_t encoderError, uint32_t controllerError) {
    ErrorSeverity worstSeverity = ERR_EXPECTED_TRANSIENT;

    for (int i = 0; i < AXIS_ERROR_COUNT; i++) {
        if (axisError & AXIS_ERRORS[i].code) {
            if (AXIS_ERRORS[i].severity > worstSeverity) worstSeverity = AXIS_ERRORS[i].severity;
        }
    }

    for (int i = 0; i < MOTOR_ERROR_COUNT; i++) {
        if (motorError & MOTOR_ERRORS[i].code) {
            if (MOTOR_ERRORS[i].severity > worstSeverity) worstSeverity = MOTOR_ERRORS[i].severity;
        }
    }

    for (int i = 0; i < ENCODER_ERROR_COUNT; i++) {
        if (encoderError & ENCODER_ERRORS[i].code) {
            if (ENCODER_ERRORS[i].severity > worstSeverity) worstSeverity = ENCODER_ERRORS[i].severity;
        }
    }

    for (int i = 0; i < CONTROLLER_ERROR_COUNT; i++) {
        if (controllerError & CONTROLLER_ERRORS[i].code) {
            if (CONTROLLER_ERRORS[i].severity > worstSeverity) worstSeverity = CONTROLLER_ERRORS[i].severity;
        }
    }

    return worstSeverity;
}

bool hasAnyError(uint32_t axis, uint64_t motor, uint32_t encoder, uint32_t controller) {
    return (axis != 0 || motor != 0 || encoder != 0 || controller != 0);
}

static bool needsShutdown = false;

bool errorManagerNeedsShutdown() { return needsShutdown; }

void resetErrorManagerState() { needsShutdown = false; }

void handleRecoverableError(ErrorSeverity severity, uint32_t axisError, uint64_t motorError, uint32_t encoderError, uint32_t controllerError, bool moveComplete) {
    extern CanBus motor;
    extern SafetyManager safety;

    static int retryCount = 0;
    static uint64_t lastErrorTime = 0;
    uint64_t currentTime = time_utils::millis();

    if (currentTime - lastErrorTime > 1000) {
        retryCount = 0;
    }
    lastErrorTime = currentTime;

    switch (severity) {
        case ERR_EXPECTED_TRANSIENT:
            motor.clearErrors();
            vTaskDelay(pdMS_TO_TICKS(ERROR_CLEAR_DELAY_MS));
            motor.setAxisState(odrive_can::AxisState::CLOSED_LOOP_CONTROL);
            retryCount = 0;
            needsShutdown = false;
            break;

        case ERR_RECOVERABLE_RETRY:
            retryCount++;
            if (retryCount > 3) {
                needsShutdown = true;
                return;
            }
            motor.clearErrors();
            vTaskDelay(pdMS_TO_TICKS(ERROR_CLEAR_DELAY_MS));
            motor.setAxisState(odrive_can::AxisState::CLOSED_LOOP_CONTROL);
            if (moveComplete) retryCount = 0;
            needsShutdown = false;
            break;

        case ERR_RECOVERABLE_HOMING:
            motor.clearErrors();
            vTaskDelay(pdMS_TO_TICKS(ERROR_CLEAR_DELAY_MS));
            motor.setAxisState(odrive_can::AxisState::CLOSED_LOOP_CONTROL);
            retryCount = 0;
            needsShutdown = false;
            break;

        case ERR_SAFETY_CRITICAL:
        default:
            safety.triggerHalt(ERR_OVER_TEMP);
            needsShutdown = true;
            break;
    }
}
