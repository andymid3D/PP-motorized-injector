#include "SafetyManager.h"
#include "CanBusHandler.h" 

extern CanBusHandler motor; 

SafetyManager::SafetyManager() 
    : _lastError(ERR_NONE), _wasMovingDown(false), _currentContext(CTX_IDLE),
      _estopCounter(0), _barrelCounter(0), _topCounter(0), _botCounter(0) {}

void SafetyManager::begin() {
    // 1. Initialize Debouncers
    // NOTE: We use INPUT because you added external Pull-up Resistors (1k)
    // If you didn't add resistors to a specific pin, change to INPUT_PULLUP
    
    dbEStop.attach(PIN_ESTOP, INPUT); 
    dbEStop.interval(20); // Short debounce, let the counter handle EMI
    
    dbBarrel.attach(PIN_ENDSTOP_BARREL, INPUT); // External 1k Pullup
    dbBarrel.interval(20);

    dbTop.attach(PIN_ENDSTOP_TOP, INPUT); // External 1k Pullup
    dbTop.interval(20);

    dbBot.attach(PIN_ENDSTOP_BOTTOM, INPUT); // External 1k Pullup
    dbBot.interval(20);

    // 2. Outputs & Sensors
    pinMode(PIN_CONTACTOR, OUTPUT);
    enableMotorPower(false);
    
    _loadCell.begin(PIN_HX711_DAT, PIN_HX711_CLK);
    _loadCell.tare(); 
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
    
    if (_loadCell.is_ready()) { _currentPressure = _loadCell.read();}
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
        Serial.printf("!!! SAFETY HALT: Error Code %d !!!\n", err);
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