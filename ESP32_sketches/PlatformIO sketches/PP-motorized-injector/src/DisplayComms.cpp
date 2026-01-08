#include "DisplayComms.h"
#include "MessageBuffer.h"
#include "SerialMessaging.h"  // For logMessage()
#include "injector_fsm.h"
#include "config.h"
#include "BroadcastDataStore.h"
#include <HardwareSerial.h>

namespace DisplayComms {

// ===== CONFIGURATION =====
unsigned long broadcastInterval = DISPLAY_BROADCAST_INTERVAL_MS;  // From config.h

// ===== UART INSTANCE =====
HardwareSerial DisplaySerial(2);  // UART2 (TX=GPIO17, RX=GPIO16 from config.h)

// ===== RX BUFFER =====
createSafeString(rxBuffer, 512);

// ===== INTERNAL STATE =====
DisplayCommsState state;

// ===== EXTERNAL REFERENCES =====
extern actualMouldParams_t currentMould;  // From main.cpp
extern MachineFlags flags;                // From main.cpp
extern fsm_state_t fsm_state;             // From main.cpp
extern BroadcastDataStore* g_broadcastData;  // From main.cpp

// ===== HELPER: Get State Name String =====
const char* getStateName(InjectorStates state) {
    switch (state) {
        case InjectorStates::INIT_HEATING:          return "INIT_HEATING";
        case InjectorStates::INIT_HOT_NOT_HOMED:    return "INIT_HOT_WAIT";
        case InjectorStates::INIT_HOMING:           return "INIT_HOMING";
        case InjectorStates::REFILL:                return "REFILL";
        case InjectorStates::COMPRESSION:           return "COMPRESSION";
        case InjectorStates::READY_TO_INJECT:       return "READY_TO_INJECT";
        case InjectorStates::PURGE_ZERO:            return "PURGE_ZERO";
        case InjectorStates::ANTIDRIP:              return "ANTIDRIP";
        case InjectorStates::INJECT:                return "INJECT";
        case InjectorStates::HOLD_INJECTION:        return "HOLD_INJECTION";
        case InjectorStates::RELEASE:               return "RELEASE";
        case InjectorStates::CONFIRM_MOULD_REMOVAL: return "CONFIRM_REMOVAL";
        case InjectorStates::ERROR_STATE:           return "ERROR_STATE";
        default:                                    return "UNKNOWN";
    }
}

// ===== INITIALIZATION =====
void begin() {
    DisplaySerial.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    rxBuffer.clear();
    state.lastEncoderBroadcast = 0;
    state.lastStateBroadcast = InjectorStates::ERROR_STATE;  // Force initial broadcast
    state.lastErrorBroadcast = 0xFFFF;  // Force initial broadcast
    
    logMessage("DisplayComms: UART2 initialized (TX=17, RX=16, 115200 baud)");
}

// ===== PERIODIC UPDATES =====
void update() {
    // Handle incoming RX messages
    while (DisplaySerial.available()) {
        char c = DisplaySerial.read();
        
        if (c == '\n' || c == '\r') {
            if (rxBuffer.length() > 0) {
                parseIncomingMessage(rxBuffer.c_str());
                rxBuffer.clear();
            }
        } else if (rxBuffer.availableForWrite() > 0) {
            rxBuffer += c;
        } else {
            // Buffer overflow, discard and reset
            rxBuffer.clear();
            logMessage("DisplayComms: RX buffer overflow, message discarded");
        }
    }
    
    // Periodic encoder broadcast (configurable interval)
    unsigned long now = millis();
    if (now - state.lastEncoderBroadcast >= broadcastInterval) {
        // Get current encoder data from BroadcastDataStore
        if (g_broadcastData) {
            float position = g_broadcastData->getPosition();
            float velocity = g_broadcastData->getVelocity();
            broadcastEncoder(position, velocity);
        }
        state.lastEncoderBroadcast = now;
    }
}

// ===== TX: BROADCAST ENCODER =====
void broadcastEncoder(float position, float velocity) {
    createSafeString(msg, 64);
    msg = "ENC|";
    msg += position;
    msg += "|";
    msg += velocity;
    msg += "\n";
    
    DisplaySerial.print(msg.c_str());
}

// ===== TX: BROADCAST STATE =====
void broadcastState(InjectorStates state_val) {
    if (state_val == state.lastStateBroadcast) return;  // No change, skip
    
    createSafeString(msg, 128);
    msg = "STATE|";
    msg += getStateName(state_val);
    msg += "|";
    msg += millis();
    msg += "\n";
    
    DisplaySerial.print(msg.c_str());
    state.lastStateBroadcast = state_val;
    
    // Also log locally
    createSafeString(logMsg, 64);
    logMsg = "State→Display: ";
    logMsg += getStateName(state_val);
    logMessage(logMsg.c_str());
}

// ===== TX: BROADCAST ERROR =====
void broadcastError(uint16_t errorCode, const char* errorMsg) {
    if (errorCode == state.lastErrorBroadcast) return;  // No change, skip
    
    createSafeString(msg, 128);
    msg = "ERROR|0x";
    msg += errorCode, HEX;
    msg += "|";
    msg += errorMsg;
    msg += "\n";
    
    DisplaySerial.print(msg.c_str());
    state.lastErrorBroadcast = errorCode;
    
    // Also log locally
    createSafeString(logMsg, 80);
    logMsg = "Error→Display: 0x";
    logMsg += errorCode, HEX;
    logMsg += " (";
    logMsg += errorMsg;
    logMsg += ")";
    logMessage(logMsg.c_str());
}

// ===== TX: SEND MOULD PARAMS CONFIRMATION =====
void sendMouldParamsConfirm(const actualMouldParams_t& params) {
    createSafeString(msg, 512);
    msg = "MOULD_OK|";
    msg += params.mouldName;
    msg += "|";
    msg += params.fillVolume;
    msg += "|";
    msg += params.fillSpeed;
    msg += "|";
    msg += params.fillPressure;
    msg += "|";
    msg += params.packVolume;
    msg += "|";
    msg += params.packSpeed;
    msg += "|";
    msg += params.packPressure;
    msg += "|";
    msg += params.packTime;
    msg += "|";
    msg += params.coolingTime;
    msg += "|";
    msg += params.fillTrapAccel;
    msg += "|";
    msg += params.fillTrapDecel;
    msg += "|";
    msg += params.packTrapAccel;
    msg += "|";
    msg += params.packTrapDecel;
    msg += "\n";
    
    DisplaySerial.print(msg.c_str());
    logMessage("Mould params confirmed→Display");
}

// ===== TX: SEND COMMON PARAMS CONFIRMATION =====
void sendCommonParamsConfirm() {
    createSafeString(msg, 512);
    msg = "COMMON_OK|";
    msg += HOMING_FAST_VEL;
    msg += "|";
    msg += OFFSET_REFILL_GAP;
    msg += "|";
    msg += COMPRESS_RAMP_TARGET;  // Compression torque limit
    msg += "|";
    msg += INJECT_FILL_TRAP_VEL_LIMIT;
    msg += "|";
    msg += RELEASE_DIST;
    msg += "|";
    msg += ANTIDRIP_VEL;  // AntiDrip velocity
    msg += "|";
    msg += PURGE_VEL_DOWN;  // Purge velocity (down direction)
    msg += "|";
    msg += HOMING_BACKOFF_VEL;
    msg += "|";
    msg += HOMING_APPROACH_VEL;
    msg += "|";
    msg += COMPRESS_TRAVEL_VEL_LIMIT;  // Compression travel velocity
    msg += "\n";
    
    DisplaySerial.print(msg.c_str());
    logMessage("Common params confirmed→Display");
}

// ===== RX: PARSE INCOMING MESSAGE =====
void parseIncomingMessage(const char* message) {
    createSafeString(msgCopy, 512);
    msgCopy = message;
    
    // Split by '|' delimiter
    createSafeString(cmd, 32);
    size_t idx = msgCopy.indexOf('|');
    if (idx == (size_t)-1) {  // SafeString returns (size_t)-1 for not found
        // No delimiter, treat entire message as command
        cmd = msgCopy;
    } else {
        msgCopy.substring(cmd, 0, idx);
    }
    
    // ===== COMMAND: MOULD PARAMS UPDATE =====
    if (cmd == "MOULD") {
        // Format: MOULD|name|fillVol|fillSpeed|fillPressure|packVol|packSpeed|packPressure|packTime|coolingTime|fillAccel|fillDecel|packAccel|packDecel
        createSafeString(field, 64);
        size_t fieldIdx = 0;
        actualMouldParams_t newParams = currentMould;  // Start with current values
        
        // Skip command field, start parsing params
        size_t pos = idx + 1;
        while (pos < msgCopy.length()) {
            size_t nextIdx = msgCopy.indexOf('|', pos);
            if (nextIdx == (size_t)-1) nextIdx = msgCopy.length();  // SafeString returns (size_t)-1 for not found
            
            msgCopy.substring(field, pos, nextIdx);
            
            switch (fieldIdx) {
                case 0: strncpy(newParams.mouldName, field.c_str(), sizeof(newParams.mouldName) - 1); break;
                case 1: newParams.fillVolume = atof(field.c_str()); break;
                case 2: newParams.fillSpeed = atof(field.c_str()); break;
                case 3: newParams.fillPressure = atof(field.c_str()); break;
                case 4: newParams.packVolume = atof(field.c_str()); break;
                case 5: newParams.packSpeed = atof(field.c_str()); break;
                case 6: newParams.packPressure = atof(field.c_str()); break;
                case 7: newParams.packTime = atof(field.c_str()); break;
                case 8: newParams.coolingTime = atof(field.c_str()); break;
                case 9: newParams.fillTrapAccel = atof(field.c_str()); break;
                case 10: newParams.fillTrapDecel = atof(field.c_str()); break;
                case 11: newParams.packTrapAccel = atof(field.c_str()); break;
                case 12: newParams.packTrapDecel = atof(field.c_str()); break;
            }
            
            fieldIdx++;
            pos = nextIdx + 1;
        }
        
        // Validate and update
        if (fieldIdx >= 13) {  // Minimum required fields
            currentMould = newParams;
            sendMouldParamsConfirm(currentMould);
            
            createSafeString(logMsg, 64);
            logMsg = "Mould params updated: ";
            logMsg += currentMould.mouldName;
            logMessage(logMsg.c_str());
        } else {
            logMessage("DisplayComms: MOULD command parsing failed (insufficient fields)");
        }
    }
    
    // ===== COMMAND: COMMON PARAMS UPDATE =====
    else if (cmd == "COMMON") {
        // Format: COMMON|homingVel|refillGap|compressionTorque|injectVel|releaseDist|antidripVel|purgeVel|...
        // NOTE: These are config.h defines, cannot be changed at runtime without modifying config.h
        // For now, just acknowledge receipt and send back current values
        logMessage("DisplayComms: COMMON command received (config.h params are read-only at runtime)");
        sendCommonParamsConfirm();
    }
    
    // ===== COMMAND: QUERY MOULD PARAMS =====
    else if (cmd == "QUERY_MOULD") {
        sendMouldParamsConfirm(currentMould);
    }
    
    // ===== COMMAND: QUERY COMMON PARAMS =====
    else if (cmd == "QUERY_COMMON") {
        sendCommonParamsConfirm();
    }
    
    // ===== COMMAND: QUERY STATE =====
    else if (cmd == "QUERY_STATE") {
        broadcastState(fsm_state.currentState);
    }
    
    // ===== COMMAND: QUERY ERROR =====
    else if (cmd == "QUERY_ERROR") {
        // Send current error code (from fsm_state or SafetyManager)
        broadcastError(fsm_state.error, "QUERY_RESPONSE");
    }
    
    // ===== UNKNOWN COMMAND =====
    else {
        createSafeString(logMsg, 80);
        logMsg = "DisplayComms: Unknown command: ";
        logMsg += cmd;
        logMessage(logMsg.c_str());
    }
}

} // namespace DisplayComms
