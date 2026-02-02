#include "TransitionErrorHandler.h"
#include "MessageBuffer.h"
#include "SafetyManager.h"
#include "BroadcastDataStore.h"

void TransitionErrorHandler::handleTransitionFailure(TransitionType type, 
                                                     uint8_t axisState, 
                                                     uint8_t controlMode) {
    // Log detailed error message
    char buf[128];
    snprintf(buf, sizeof(buf), "CRITICAL_TRANSITION_FAIL: %s | AxisState=%d CtrlMode=%d | Time=%lu",  // SAFE: Test code logging only, no CANbus interaction
             getTransitionName(type), axisState, controlMode, millis());
    MessageBuffer::getInstance().sendMessage(buf);
    
    // Check if motor is moving (velocity > threshold)
    BroadcastDataStore& broadcastStore = BroadcastDataStore::getInstance();
    float velocity = broadcastStore.getVelocity();
    const float MOVING_THRESHOLD = 0.5f;  // 0.5 turns/sec
    
    if (fabs(velocity) > MOVING_THRESHOLD) {
        // Motor is moving - CAN control is compromised, cut contactor immediately
        MessageBuffer::getInstance().sendMessage("TRANS_FAIL: Motor moving, CAN unreliable, CUTTING CONTACTOR");
        SafetyManager& safety = SafetyManager::getInstance();
        safety.forceEmergencyShutdown("RTR failure + motor moving");
    } else {
        // Motor already stopped or stationary
        MessageBuffer::getInstance().sendMessage("TRANS_FAIL: Motor stationary (safe)");
        
        // Flag error for FSM transition to ERROR_STATE
        SafetyManager& safety = SafetyManager::getInstance();
        safety.flagError(ERR_CAN_RTR_FAILURE);
    }
}

const char* TransitionErrorHandler::getTransitionName(TransitionType type) {
    switch (type) {
        case TRANS_IDLE_TO_CLOSED_LOOP:
            return "IDLE→CLOSED_LOOP";
        case TRANS_MODE_TO_POSITION:
            return "MODE→POSITION";
        case TRANS_MODE_TO_TORQUE:
            return "MODE→TORQUE";
        case TRANS_ANY_TO_IDLE:
            return "ANY→IDLE";
        default:
            return "UNKNOWN";
    }
}
