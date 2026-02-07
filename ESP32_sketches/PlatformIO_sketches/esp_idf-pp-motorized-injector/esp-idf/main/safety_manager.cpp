#include "safety_manager.h"
#include "broadcast_data_store.h"
#include "time_utils.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>

static const char* TAG = "SAFETY";

SafetyManager::SafetyManager()
    : _lastError(ERR_NONE),
      _currentContext(CTX_IDLE),
      _powerCutLatched(false),
      _currentPressure(0),
      _pressureBaseline(0),
      _moveStartTime(0),
      _wasMovingDown(false),
      _startPosition(0.0f),
      _loadCellScale(1.0f),
      _loadCellOffset(0),
      _loadCellAvgSamples(1),
      _loadCellBuffer(nullptr),
      _loadCellBufferIdx(0),
      _loadCellUseMedian(false),
      _lastValidReading(0),
      _estopCounter(0),
      _barrelCounter(0),
      _topCounter(0),
      _botCounter(0),
      _bootTime(0),
      _motorPowerEnabled(false),
      _lastPowerEnableTimeUs(0),
      _lastPowerDisableTimeUs(0),
      _lastPowerEnableBlockLogUs(0) {
}

SafetyManager& SafetyManager::getInstance() {
    static SafetyManager instance;
    return instance;
}

void SafetyManager::begin() {
    _bootTime = time_utils::micros();
    _powerCutLatched = false;
    _motorPowerEnabled = false;
    _lastPowerEnableTimeUs = 0;
    _lastPowerDisableTimeUs = 0;
    _lastPowerEnableBlockLogUs = 0;

    // Endstops / Estop: external pullups, active state handled in debouncer
    dbEStop.begin((gpio_num_t)PIN_ESTOP, false, 20, false);
    dbBarrel.begin((gpio_num_t)PIN_ENDSTOP_BARREL, false, 20, false);
    dbTop.begin((gpio_num_t)PIN_ENDSTOP_TOP, true, 20, false);
    dbBot.begin((gpio_num_t)PIN_ENDSTOP_BOTTOM, true, 20, false);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_CONTACTOR);
    gpio_config(&io_conf);

    enableMotorPower(false);

    _loadCell.begin(PIN_HX711_DAT, PIN_HX711_CLK);
    _loadCell.tare();

    _loadCellScale = 1.0f;
    _loadCellOffset = 0;
    _loadCellAvgSamples = 1;
    _loadCellBuffer = nullptr;
    _loadCellBufferIdx = 0;
    _loadCellUseMedian = false;
    _lastValidReading = 0;
}

void SafetyManager::updateInputs() {
    dbEStop.update();
    dbBarrel.update();
    dbTop.update();
    dbBot.update();

    if (dbEStop.readRaw()) {
        if (_estopCounter < CONFIDENCE_THRESHOLD) _estopCounter++;
    } else {
        _estopCounter = 0;
    }

    if (dbBarrel.readRaw()) {
        if (_barrelCounter < CONFIDENCE_THRESHOLD) _barrelCounter++;
    } else {
        _barrelCounter = 0;
    }

    if (dbTop.isPressed()) {
        if (_topCounter < CONFIDENCE_THRESHOLD) _topCounter++;
    } else {
        _topCounter = 0;
    }

    if (dbBot.isPressed()) {
        if (_botCounter < CONFIDENCE_THRESHOLD) _botCounter++;
    } else {
        _botCounter = 0;
    }

    if (_loadCell.is_ready()) {
        long rawReading = _loadCell.read();
        const long MAX_VALID_READING = 10000000;
        if (rawReading < -MAX_VALID_READING || rawReading > MAX_VALID_READING) {
            rawReading = _lastValidReading;
        } else {
            _lastValidReading = rawReading;
        }

        if (_loadCellAvgSamples > 1 && _loadCellBuffer != nullptr) {
            _loadCellBuffer[_loadCellBufferIdx] = rawReading;
            _loadCellBufferIdx = (_loadCellBufferIdx + 1) % _loadCellAvgSamples;

            if (_loadCellUseMedian) {
                long sorted[20];
                for (uint8_t i = 0; i < _loadCellAvgSamples; i++) {
                    sorted[i] = _loadCellBuffer[i];
                }
                for (uint8_t i = 0; i < _loadCellAvgSamples - 1; i++) {
                    for (uint8_t j = 0; j < _loadCellAvgSamples - i - 1; j++) {
                        if (sorted[j] > sorted[j + 1]) {
                            long temp = sorted[j];
                            sorted[j] = sorted[j + 1];
                            sorted[j + 1] = temp;
                        }
                    }
                }
                if (_loadCellAvgSamples % 2 == 1) {
                    rawReading = sorted[_loadCellAvgSamples / 2];
                } else {
                    rawReading = (sorted[_loadCellAvgSamples / 2 - 1] + sorted[_loadCellAvgSamples / 2]) / 2;
                }
            } else {
                long sum = 0;
                for (uint8_t i = 0; i < _loadCellAvgSamples; i++) sum += _loadCellBuffer[i];
                rawReading = sum / _loadCellAvgSamples;
            }
        }

        _currentPressure = (long)((rawReading - _loadCellOffset) * _loadCellScale);
    }
}

bool SafetyManager::isEStopPressed() { return _estopCounter >= CONFIDENCE_THRESHOLD; }
bool SafetyManager::isBarrelOpen() { return _barrelCounter >= CONFIDENCE_THRESHOLD; }
bool SafetyManager::isTopEndstopHit() { return _topCounter >= CONFIDENCE_THRESHOLD; }
bool SafetyManager::isBottomEndstopHit() { return _botCounter >= CONFIDENCE_THRESHOLD; }

void SafetyManager::enableMotorPower(bool enable) {
    if (!enable) {
        if (_motorPowerEnabled) {
#if APP_DEBUG
            ESP_LOGI(TAG, "CONTACTOR OFF");
#endif
        }
        gpio_set_level((gpio_num_t)PIN_CONTACTOR, 0);
        _motorPowerEnabled = false;
        _lastPowerDisableTimeUs = time_utils::micros();
        return;
    }
    if (_powerCutLatched) return;
    if (isEStopPressed()) {
        triggerHalt(ERR_ESTOP);
        return;
    }
    bool wasEnabled = _motorPowerEnabled;
    if (!wasEnabled) {
        uint64_t now = time_utils::micros();
        if (_lastPowerDisableTimeUs > 0 &&
            (now - _lastPowerDisableTimeUs) < (CONTACTOR_MIN_OFF_MS * 1000ULL)) {
#if APP_DEBUG
            if (now - _lastPowerEnableBlockLogUs > 1000000ULL) {
                uint64_t remaining = (CONTACTOR_MIN_OFF_MS * 1000ULL) - (now - _lastPowerDisableTimeUs);
                ESP_LOGI(TAG, "CONTACTOR_ENABLE_BLOCKED cooldown %llums remaining", (unsigned long long)(remaining / 1000ULL));
                _lastPowerEnableBlockLogUs = now;
            }
#endif
            return;
        }
        _lastPowerEnableTimeUs = now;
    }
    gpio_set_level((gpio_num_t)PIN_CONTACTOR, 1);
    _motorPowerEnabled = true;
#if APP_DEBUG
    if (!wasEnabled) {
        ESP_LOGI(TAG, "CONTACTOR ON");
    }
#endif
}

void SafetyManager::triggerHalt(MachineError err) {
    if (_lastError != err) {
        ESP_LOGW(TAG, "SAFETY HALT: %d", err);
        _lastError = err;
    }
    if (err == ERR_ESTOP || err == ERR_BARREL_POSITION_LOST ||
        err == ERR_BOTTOM_ENDSTOP_COLLISION || err == ERR_TOP_ENDSTOP_COLLISION) {
        _powerCutLatched = true;
    }
    enableMotorPower(false);
}

void SafetyManager::setContext(SafetyContext ctx) { _currentContext = ctx; }

bool SafetyManager::check(float current_velocity, bool is_moving_down) {
    if (isEStopPressed()) {
        triggerHalt(ERR_ESTOP);
        return false;
    }
    if (isBarrelOpen()) {
        triggerHalt(ERR_BARREL_POSITION_LOST);
        return false;
    }

    uint64_t uptime = time_utils::micros() - _bootTime;
    if (_motorPowerEnabled && uptime >= 10000000ULL) {
        if (BroadcastDataStore::getInstance().isBroadcastDataStale()) {
            uint64_t now = time_utils::micros();
            if (_lastPowerEnableTimeUs > 0 &&
                (now - _lastPowerEnableTimeUs) < (BROADCAST_GRACE_AFTER_POWER_MS * 1000ULL)) {
                return true;
            }
            if (_lastError != ERR_BROADCAST_STALE) {
                ESP_LOGW(TAG, "Broadcast stale > %dms", BROADCAST_STALE_TIMEOUT_MS);
                _lastError = ERR_BROADCAST_STALE;
            }
            return false;
        }
    }

    if (_currentContext != CTX_MOVING_FREE) {
        if (isBottomEndstopHit() && is_moving_down && std::fabs(current_velocity) > 0.1f) {
#if APP_DEBUG
            if (_lastError != ERR_BOTTOM_ENDSTOP_COLLISION) {
                ESP_LOGW(TAG, "BOTTOM_ENDSTOP_HIT");
            }
#endif
            triggerHalt(ERR_BOTTOM_ENDSTOP_COLLISION);
            return false;
        }
        if (isTopEndstopHit() && !is_moving_down && std::fabs(current_velocity) > 0.1f) {
#if APP_DEBUG
            if (_lastError != ERR_TOP_ENDSTOP_COLLISION) {
                ESP_LOGW(TAG, "TOP_ENDSTOP_HIT");
            }
#endif
            triggerHalt(ERR_TOP_ENDSTOP_COLLISION);
            return false;
        }
    }

    if (is_moving_down && std::fabs(current_velocity) > 0.1f) {
        if (!_wasMovingDown) {
            _moveStartTime = time_utils::micros();
            _pressureBaseline = _currentPressure;
            BroadcastDataStore& bds = BroadcastDataStore::getInstance();
            _startPosition = bds.getPosition();
            _wasMovingDown = true;
        }

        long pressureDelta = _currentPressure - _pressureBaseline;
        BroadcastDataStore& bds = BroadcastDataStore::getInstance();
        float distMoved = std::fabs(bds.getPosition() - _startPosition);

        if (_currentContext == CTX_BLOCKED) {
#ifndef IGNORE_NOZZLE_BLOCK
            if (distMoved > 10.0f && std::abs(pressureDelta) < PRESSURE_BLOCK_MIN) {
                triggerHalt(ERR_NOZZLE_NOT_BLOCKED);
                return false;
            }
#endif
        }
    } else {
        _wasMovingDown = false;
    }

    return (_lastError == ERR_NONE);
}

void SafetyManager::resetError() {
    _lastError = ERR_NONE;
    _powerCutLatched = false;
    _estopCounter = _barrelCounter = _topCounter = _botCounter = 0;
}

void SafetyManager::retareLoadCell() { _loadCell.tare(); }
void SafetyManager::setLoadCellScale(float scale) { _loadCellScale = scale; }
void SafetyManager::setLoadCellOffset(long offset) { _loadCellOffset = offset; }
long SafetyManager::getLoadCellRaw() {
    if (_loadCell.is_ready()) return _loadCell.read();
    return 0;
}

void SafetyManager::setLoadCellAveraging(uint8_t samples) {
    if (samples < 1) samples = 1;
    if (samples > 20) samples = 20;
    if (_loadCellBuffer != nullptr) {
        delete[] _loadCellBuffer;
        _loadCellBuffer = nullptr;
    }
    _loadCellAvgSamples = samples;
    if (samples > 1) {
        _loadCellBuffer = new long[samples];
        long currentReading = 0;
        if (_loadCell.is_ready()) currentReading = _loadCell.read();
        for (uint8_t i = 0; i < samples; i++) _loadCellBuffer[i] = currentReading;
        _loadCellBufferIdx = 0;
    }
}

void SafetyManager::setLoadCellMedianFilter(bool enable) { _loadCellUseMedian = enable; }

void SafetyManager::forceEmergencyShutdown(const char* reason) {
    ESP_LOGE(TAG, "EMERGENCY_SHUTDOWN: %s", reason);
    _powerCutLatched = true;
    enableMotorPower(false);
    _lastError = ERR_CAN_RTR_FAILURE;
}

void SafetyManager::flagError(MachineError err) {
    _lastError = err;
    ESP_LOGW(TAG, "SAFETY_ERROR_FLAGGED: %d", err);
}
