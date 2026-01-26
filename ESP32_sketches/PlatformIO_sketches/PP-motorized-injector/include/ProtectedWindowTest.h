#ifndef PROTECTED_WINDOW_TEST_H
#define PROTECTED_WINDOW_TEST_H

#include <Arduino.h>
#include "CanBusHandlerV2.h"
#include "CanRxHandler.h"
#include "GPTimer.h"
#include "RingBuffer.h"
#include "ODriveCANProtocol.h" // Include for enums

// The CANRxMessage struct is defined in CanRxHandler.h, which is included.
// No need to redefine it here.

class ProtectedWindowTest {
public:
    ProtectedWindowTest();
    void begin(CanBusHandlerV2& motor);
    void loop();

private:
    enum TestState {
        IDLE,
        WAITING_FOR_BUNDLE,
        PRE_ROLL_COLLECTING,
        SENDING_COMMAND,
        POST_ROLL_COLLECTING,
        ANALYZING,
        FINISHED
    };

    CanBusHandlerV2* _motor;
    TestState _state;
    uint32_t _testOffset_us; // Offset from bundle start to send TX
    uint64_t _bundleStartTime_us; // Timestamp of the bundle marker (Heartbeat)
    uint32_t _tests_run;

    // --- Test Configuration ---
    static const uint32_t TEST_COMMAND_ID = ODriveCANProtocol::MSG_SET_CONTROLLER_MODES; // Using Set_Controller_Modes
    static const uint32_t PRE_ROLL_DURATION_US = 200000; // MODIFIED: 250ms before TX
    static const uint32_t POST_ROLL_DURATION_US = 300000; // MODIFIED: 300ms after TX
    static const uint32_t MESSAGE_BUFFER_CAPACITY = 150; // MODIFIED: Increased capacity for larger window (approx 750ms * 180 msg/s = 135 messages + TX)

    // --- Automated Sweep Parameters (for future use) ---
    static const uint32_t TEST_OFFSET_INCREMENT_US = 5000; // 5ms increment for offset
    static const uint32_t MAX_TEST_OFFSET_US = 100000;

    // --- Internal State ---
    RingBuffer<CANRxMessage, MESSAGE_BUFFER_CAPACITY> _collectedMessages;
    uint64_t _captureWindowStart_us;
    uint64_t _commandSentTime_us;
    ODriveCANProtocol::SetControllerModesParams _sentCommandParams; // Store what we sent
    bool _bundleDetectedForRun; // New flag to ensure bundle start is set once per run

    void startTestRun();
    void sendTestCommand();
    void collectMessages();
    void printAnalysis();
};

#endif // PROTECTED_WINDOW_TEST_H
