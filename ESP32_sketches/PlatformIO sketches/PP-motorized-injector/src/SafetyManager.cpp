#include "SafetyManager.h"
#include "CanBusHandlerV2.h"
#include "MessageBuffer.h" 

extern CanBusHandlerV2 motor; 

SafetyManager::SafetyManager() 
    : _lastError(ERR_NONE), _wasMovingDown(false), _currentContext(CTX_IDLE),
      _estopCounter(0), _barrelCounter(0), _topCounter(0), _botCounter(0),
      _loadCellScale(1.0), _loadCellOffset(0), _loadCellAvgSamples(1),
      _loadCellBuffer(nullptr), _loadCellBufferIdx(0), _loadCellUseMedian(false),
      _lastValidReading(0) {}

void SafetyManager::begin() {
    // 1. Initialize Debouncers
    // NOTE: We use INPUT because you added external Pull-up Resistors (1k)
    // If you didn't add resistors to a specific pin, change to INPUT_PULLUP
    
    dbEStop.attach(PIN_ESTOP, INPUT); 
    dbEStop.setPressedState(LOW);  // Triggered when sensor reads LOW
    dbEStop.interval(20); // Short debounce, let the counter handle EMI
    
    dbBarrel.attach(PIN_ENDSTOP_BARREL, INPUT); // External 1k Pullup
    dbBarrel.setPressedState(LOW);  // Triggered when sensor reads LOW
    dbBarrel.interval(20);

    dbTop.attach(PIN_ENDSTOP_TOP, INPUT); // External 1k Pullup
    dbTop.setPressedState(LOW);  // Triggered when sensor reads LOW (plunger NOT at endstop)
    dbTop.interval(20);

    dbBot.attach(PIN_ENDSTOP_BOTTOM, INPUT); // External 1k Pullup
    dbBot.setPressedState(LOW);  // Triggered when sensor reads LOW (plunger NOT at endstop)
    dbBot.interval(20);

    // 2. Outputs & Sensors
    pinMode(PIN_CONTACTOR, OUTPUT);
    enableMotorPower(false);
    
    _loadCell.begin(PIN_HX711_DAT, PIN_HX711_CLK);
    _loadCell.tare();  // Initial tare (motor off, clean baseline)
    
    // Initialize load cell with default settings (no averaging, no scale, no offset)
    _loadCellScale = 1.0;
    _loadCellOffset = 0;
    _loadCellAvgSamples = 1;  // No averaging by default
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
    
    
    if (dbEStop.read() == HIGH) { 
        if (_estopCounter < CONFIDENCE_THRESHOLD) _estopCounter++;} else {_estopCounter = 0;}

    if (dbBarrel.read() == HIGH) {  if (_barrelCounter < CONFIDENCE_THRESHOLD) _barrelCounter++; } else { _barrelCounter = 0;}

    if (dbTop.read() == LOW) {if (_topCounter < CONFIDENCE_THRESHOLD) _topCounter++;} else { _topCounter = 0;}

    if (dbBot.read() == LOW) {if (_botCounter < CONFIDENCE_THRESHOLD) _botCounter++;} else {_botCounter = 0;}
    
    // HX711 Load Cell Reading with EMI Rejection and Optional Filtering
    if (_loadCell.is_ready()) {
        long rawReading = _loadCell.read();
        
        // VALIDITY CHECK: Reject extreme EMI outliers (likely motor noise)
        // HX711 is 24-bit: max range is ±8,388,608 (2^23)
        // Reject readings beyond ±10 million (clearly EMI corruption)
        const long MAX_VALID_READING = 10000000;
        if (rawReading < -MAX_VALID_READING || rawReading > MAX_VALID_READING) {
            // EMI corruption detected, use last valid reading
            rawReading = _lastValidReading;
        } else {
            _lastValidReading = rawReading;  // Store for next time
        }
        
        // Apply filtering if enabled
        if (_loadCellAvgSamples > 1 && _loadCellBuffer != nullptr) {
            _loadCellBuffer[_loadCellBufferIdx] = rawReading;
            _loadCellBufferIdx = (_loadCellBufferIdx + 1) % _loadCellAvgSamples;
            
            if (_loadCellUseMedian) {
                // MEDIAN FILTER: More robust against EMI spikes than average
                // Copy buffer and sort to find median
                long sorted[20];  // Max 20 samples
                for (uint8_t i = 0; i < _loadCellAvgSamples; i++) {
                    sorted[i] = _loadCellBuffer[i];
                }
                
                // Simple bubble sort (sufficient for small arrays)
                for (uint8_t i = 0; i < _loadCellAvgSamples - 1; i++) {
                    for (uint8_t j = 0; j < _loadCellAvgSamples - i - 1; j++) {
                        if (sorted[j] > sorted[j + 1]) {
                            long temp = sorted[j];
                            sorted[j] = sorted[j + 1];
                            sorted[j + 1] = temp;
                        }
                    }
                }
                
                // Take middle value (or average of two middle values if even)
                if (_loadCellAvgSamples % 2 == 1) {
                    rawReading = sorted[_loadCellAvgSamples / 2];
                } else {
                    rawReading = (sorted[_loadCellAvgSamples / 2 - 1] + sorted[_loadCellAvgSamples / 2]) / 2;
                }
            } else {
                // AVERAGE FILTER: Simple but less robust to outliers
                long sum = 0;
                for (uint8_t i = 0; i < _loadCellAvgSamples; i++) {
                    sum += _loadCellBuffer[i];
                }
                rawReading = sum / _loadCellAvgSamples;
            }
        }
        
        // Apply offset and scale
        _currentPressure = (long)((rawReading - _loadCellOffset) * _loadCellScale);
    }
}

// --- Getters (Now use the Counters) ---
bool SafetyManager::isEStopPressed() { return _estopCounter >= CONFIDENCE_THRESHOLD; }
bool SafetyManager::isBarrelOpen()   { return _barrelCounter >= CONFIDENCE_THRESHOLD; }
bool SafetyManager::isTopEndstopHit() { return _topCounter >= CONFIDENCE_THRESHOLD; }
bool SafetyManager::isBottomEndstopHit() { return _botCounter >= CONFIDENCE_THRESHOLD; }

void SafetyManager::enableMotorPower(bool enable) {
    if (enable && isEStopPressed()) {
        triggerHalt(ERR_ESTOP);
        return;
    }
    digitalWrite(PIN_CONTACTOR, enable ? HIGH : LOW);
}

void SafetyManager::triggerHalt(MachineError err) {
    if (_lastError != err) {
        char buf[64];
        snprintf(buf, sizeof(buf), "SAFETY HALT: Error Code %d", err);
        MessageBuffer::getInstance().sendMessage(buf);
        _lastError = err;
    }
    enableMotorPower(false);
}

void SafetyManager::setContext(SafetyContext ctx) {
    _currentContext = ctx;
}

bool SafetyManager::check(float current_velocity, bool is_moving_down) {
    // Checks are based on the filtered counters from updateInputs()

    if (isEStopPressed()) {
        triggerHalt(ERR_ESTOP);
        return false;
    }

    if (isBarrelOpen()) { 
        triggerHalt(ERR_BARREL_POSITION_LOST);
        return false;
    }

    // ===== ENDSTOP COLLISION DETECTION =====
    // Prevent unexpected endstop collisions during movement (except during homing/antidrip)
    if (_currentContext != CTX_MOVING_FREE) {
        // Bottom endstop collision (moving down)
        if (isBottomEndstopHit() && is_moving_down && abs(current_velocity) > 0.1f) {
            triggerHalt(ERR_BOTTOM_ENDSTOP_COLLISION);
            return false;
        }
        
        // Top endstop collision (moving up)
        if (isTopEndstopHit() && !is_moving_down && abs(current_velocity) > 0.1f) {
            triggerHalt(ERR_TOP_ENDSTOP_COLLISION);
            return false;
        }
    }

    // Pressure Logic
    if (is_moving_down && abs(current_velocity) > 0.1f) {
        if (!_wasMovingDown) {
            _moveStartTime = millis();
            _pressureBaseline = _currentPressure;
            _startPosition = motor.getPosition(); 
            _wasMovingDown = true;
        }

        long pressureDelta = _currentPressure - _pressureBaseline;
        float distMoved = abs(motor.getPosition() - _startPosition);

        if (_currentContext == CTX_BLOCKED) {
            // BYPASS FOR TESTING
            #ifndef IGNORE_NOZZLE_BLOCK
            if (distMoved > 10.0f && abs(pressureDelta) < PRESSURE_BLOCK_MIN) {
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
    // Reset counters too to avoid immediate re-trigger
    _estopCounter = 0;
    _barrelCounter = 0;
    _topCounter = 0;
    _botCounter = 0;
}

// ===== HX711 Load Cell Configuration Methods =====

void SafetyManager::retareLoadCell() {
    // Re-tare the load cell (useful after motor power on to compensate for EMI baseline shift)
    _loadCell.tare();
}

void SafetyManager::setLoadCellScale(float scale) {
    // Set calibration factor to convert raw ADC counts to engineering units
    // Example: If 1000 counts = 1kg, scale = 0.001
    _loadCellScale = scale;
}

void SafetyManager::setLoadCellOffset(long offset) {
    // Manual baseline offset adjustment (alternative to tare)
    // Useful if you know the baseline value and want to subtract it
    _loadCellOffset = offset;
}

long SafetyManager::getLoadCellRaw() {
    // Return raw ADC value for diagnostics (bypass offset/scale)
    if (_loadCell.is_ready()) {
        return _loadCell.read();
    }
    return 0;
}

void SafetyManager::setLoadCellAveraging(uint8_t samples) {
    // Enable moving average filter to reduce EMI noise
    // samples: 1 = disabled, 2-10 recommended (higher = smoother but slower response)
    
    if (samples < 1) samples = 1;
    if (samples > 20) samples = 20;  // Cap at 20 to avoid excessive memory usage
    
    // Free old buffer if it exists
    if (_loadCellBuffer != nullptr) {
        delete[] _loadCellBuffer;
        _loadCellBuffer = nullptr;
    }
    
    _loadCellAvgSamples = samples;
    
    if (samples > 1) {
        // Allocate new circular buffer
        _loadCellBuffer = new long[samples];
        
        // Initialize buffer with current reading
        long currentReading = 0;
        if (_loadCell.is_ready()) {
            currentReading = _loadCell.read();
        }
        
        for (uint8_t i = 0; i < samples; i++) {
            _loadCellBuffer[i] = currentReading;
        }
        
        _loadCellBufferIdx = 0;
    }
}

void SafetyManager::setLoadCellMedianFilter(bool enable) {
    // Use median filter instead of average (more robust against EMI spikes)
    // Requires averaging to be enabled with setLoadCellAveraging() first
    _loadCellUseMedian = enable;
}