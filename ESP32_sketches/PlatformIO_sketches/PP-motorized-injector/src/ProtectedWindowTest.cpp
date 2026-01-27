#include "ProtectedWindowTest.h"
#include "BroadcastDataStore.h"
#include "ODriveCANProtocol.h" // For command IDs and parameters
#include "CanRxHandler.h"      // For draining messages
#include "GPTimer.h"           // For high-resolution timestamps
#include "OurLoopTimer.h"      // For toggleLoopFlag function
#include "config.h"            // To get ODRIVE_NODE_ID
#include "TimingSystemTest.h"  // For timing system test commands
#include <algorithm>           // For std::sort
#include <vector>              // For std::vector

// Reference to the global hardware timer
extern GPTimer hwTimer;

ProtectedWindowTest::ProtectedWindowTest()
    : _motor(nullptr),
    _state(IDLE),
    _testOffset_us(0),
    _bundleStartTime_us(0),
    _tests_run(0),
    _captureWindowStart_us(0),
    _commandSentTime_us(0),
    _sentCommandParams(ODriveCANProtocol::ControlMode::POSITION_CONTROL, ODriveCANProtocol::InputMode::PASSTHROUGH), // Default benign command
    _bundleDetectedForRun(false) // Initialize new flag
{}

void ProtectedWindowTest::begin(CanBusHandlerV2& motor) {
    _motor = &motor;
    
    // Initialize timing system test with CAN access
    TimingSystemTest::getInstance().begin(motor);
    
    Serial.println("ProtectedWindowTest initialized. Type 'start' to begin a test run.");
    Serial.println("Timing commands available: timing_start, timing_baseline, timing_check, timing_window, timing_status, timing_auto, timing_iq_history");
    Serial.println("Features: Position + IQ current movement detection (BUS current disabled)");
    Serial.println("Note: timing_auto = movement test only (calibration via USB)");
}

void ProtectedWindowTest::loop() {
    // Handle serial input for commands
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "start" && (_state == IDLE || _state == FINISHED)) {
            startTestRun();
        }
        else if (command == "help") {
            Serial.println("\n=== PROTECTED WINDOW TEST COMMANDS ===");
            Serial.println("TIMING SYSTEM:");
            Serial.println("  timing_start                   - Start timing test");
            Serial.println("  timing_baseline                  - Capture baseline");
            Serial.println("  timing_check                     - Check movement");
            Serial.println("  timing_debug                     - Show raw data");
            Serial.println("  timing_status                    - Show system status");
            Serial.println("  timing_window                    - Test command windows");
            Serial.println("  timing_auto                      - Movement test sequence (calibration via USB)");
            Serial.println("  timing_iq_history                - Show IQ current profile");
            Serial.println("\nMOVEMENT COMMANDS (Tx-triggered):");
            Serial.println("  set_velocity 5.0                - Send velocity command (triggers analysis)");
            Serial.println("  set_position 1.0                - Send position command (triggers analysis)");
            Serial.println("  set_torque 0.5                  - Send torque command (triggers analysis)");
            Serial.println("\nPROTECTED WINDOW TEST:");
            Serial.println("  start                           - Start protected window test");
            Serial.println("  help                            - Show this help");
            Serial.println("=====================================\n");
        }
        // ===== TIMING SYSTEM TEST COMMANDS =====
        else if (command.startsWith("timing_")) {
            toggleLoopFlag(); // Mark start of timing operation
            TimingSystemTest::getInstance().handleCommand(command);
            toggleLoopFlag(); // Mark end of timing operation
        }
        // ===== MOVEMENT COMMANDS FOR TX-TRIGGERED TESTING =====
        else if (command.startsWith("set_velocity")) {
            // Extract velocity value
            float vel = command.substring(12).toFloat();  // After "set_velocity "
            _motor->setInputVel(vel);
            Serial.printf("Sent: SET_INPUT_VEL = %.3f turns/sec (Tx-triggered test)\n", vel);
        }
        else if (command.startsWith("set_position")) {
            // Extract position value
            float pos = command.substring(13).toFloat();  // After "set_position "
            _motor->setInputPos(pos);
            Serial.printf("Sent: SET_INPUT_POS = %.3f turns (Tx-triggered test)\n", pos);
        }
        else if (command.startsWith("set_torque")) {
            // Extract torque value
            float torque = command.substring(12).toFloat();  // After "set_torque "
            _motor->setInputTorque(torque);
            Serial.printf("Sent: SET_INPUT_TORQUE = %.3f Nm (Tx-triggered test)\n", torque);
        }
        else {
            Serial.println("Unknown command. Type 'help' for available commands.");
        }
    }

    uint64_t currentTime = hwTimer.micros();

    // CRITICAL: Call motor.loop() to process outgoing CAN messages
    if (_motor) { // Ensure _motor is initialized
        _motor->loop(); // <--- UNCOMMENTED: Re-enable TX sending
    }
    
    // Check if timing test collection is complete
    TimingSystemTest::getInstance().checkTestCompletion();

    switch (_state) {
        case IDLE:
        case FINISHED:
            // Do nothing, waiting for 'start' command
            break;

        case WAITING_FOR_BUNDLE: {
            // Continuously collect messages to keep the CanRxHandler queue clear
            collectMessages();

            // Use Heartbeat as the bundle marker
            const TimestampedHeartbeat* hb = BroadcastDataStore::getInstance().getLatestHeartbeat();
            // Check if we have a fresh heartbeat (e.g., less than 1ms old) AND it's the first for this run
            if (hb && (currentTime - hb->timestamp < 1000) && !_bundleDetectedForRun) {
                _bundleStartTime_us = hb->timestamp;
                _captureWindowStart_us = _bundleStartTime_us - PRE_ROLL_DURATION_US; // Start collecting before bundle
                Serial.printf("Bundle detected (Heartbeat) at %llu us. Starting pre-roll collection.\n", _bundleStartTime_us);
                _bundleDetectedForRun = true; // Set flag to prevent re-detecting for this run
                _state = PRE_ROLL_COLLECTING;
            }
            break;
        }

        case PRE_ROLL_COLLECTING: {
            collectMessages(); // Keep collecting messages

            if (currentTime >= (_bundleStartTime_us + _testOffset_us)) {
                _state = SENDING_COMMAND;
            }
            break;
        }

        case SENDING_COMMAND: {
            sendTestCommand();
            _state = POST_ROLL_COLLECTING;
            break;
        }

        case POST_ROLL_COLLECTING: {
            collectMessages(); // Keep collecting messages
            if (currentTime >= (_commandSentTime_us + POST_ROLL_DURATION_US)) {
                _state = ANALYZING;
            }

            break;
        }

        case ANALYZING: {
            printAnalysis();
            _tests_run++;
            _testOffset_us += TEST_OFFSET_INCREMENT_US;
            if (_testOffset_us > MAX_TEST_OFFSET_US) {
                Serial.println("All test offsets completed for this run.");
                _state = FINISHED;
            } else {
                _collectedMessages.clear(); // Clear buffer for next test run
                _bundleDetectedForRun = false; // Reset flag for next test run
                _state = WAITING_FOR_BUNDLE; // Prepare for next offset test
            }
            break;
        }

    }
}

void ProtectedWindowTest::startTestRun() {
    Serial.println("Starting Protected Window Test Run...");
    _testOffset_us = 0; // Start with 0 offset for the first command
    _tests_run = 0;
    _collectedMessages.clear();
    _bundleDetectedForRun = false; // Reset flag for the very first run
    _state = WAITING_FOR_BUNDLE;
}

void ProtectedWindowTest::sendTestCommand() {
    _commandSentTime_us = hwTimer.micros();

    // Send 1x set_controller_modes command (3, 1) per test block
    // Testing if single SET commands get responses when not repeated
    _motor->setControllerModes(ODriveCANProtocol::ControlMode::POSITION_CONTROL, ODriveCANProtocol::InputMode::PASSTHROUGH);
    
    // Create a CANRxMessage for the TX event
    CANRxMessage txMsg;
    txMsg.canId = ODriveCANProtocol::MSG_SET_CONTROLLER_MODES;
    txMsg.dlc = 8; // SET commands have 8 bytes payload
    memset(txMsg.data, 0, 8);
    txMsg.timestamp = hwTimer.micros();
    _collectedMessages.push(txMsg);
    
    Serial.printf("TX:%llu:%u:0x%X:1\n", txMsg.timestamp, _testOffset_us, txMsg.canId);
}

void ProtectedWindowTest::collectMessages() {
    CanRxHandler& canRx = CanRxHandler::getInstance();
    CANRxMessage msg;

    // Drain all available messages from CanRxHandler's internal queue
    while (canRx.receiveMessage(msg, 0)) { // 0 timeout = non-blocking
        // Only store messages within the defined capture window
        // This ensures we don't fill the buffer with irrelevant messages
        if (msg.timestamp >= _captureWindowStart_us) {
            _collectedMessages.push(msg);
        }
    }
}

// Comparison function for sorting CANRxMessage by timestamp
bool compareCANRxMessages(const CANRxMessage& a, const CANRxMessage& b) {
    return a.timestamp < b.timestamp;
}

void ProtectedWindowTest::printAnalysis() {
    Serial.printf("--- Analysis for Offset %u us (Test %u) ---\n", _testOffset_us, _tests_run);


    // Convert RingBuffer to a temporary std::vector for sorting
    std::vector<CANRxMessage> sortedMessages;
    _collectedMessages.forEach([&sortedMessages](const CANRxMessage& msg, size_t index) {
        sortedMessages.push_back(msg);
        return true;
    });

    // Sort the messages by timestamp
    std::sort(sortedMessages.begin(), sortedMessages.end(), compareCANRxMessages);

    // Iterate through sorted messages and print
    for (const auto& msg : sortedMessages) {
        int32_t relativeTime = (int32_t)(msg.timestamp - _bundleStartTime_us);
        // Check if this is our TX message (using TEST_COMMAND_ID and timestamp)
        if (msg.canId == TEST_COMMAND_ID && msg.timestamp == _commandSentTime_us) {
            // Print TX message with its specific format
            Serial.printf("TX:%llu:%d:0x%X:%d:%d\n", msg.timestamp, relativeTime, msg.canId, (int)_sentCommandParams.control_mode, (int)_sentCommandParams.input_mode);
        } else {
            // Print RX message
            Serial.printf("RX:%llu:%d:0x%X\n", msg.timestamp, relativeTime, msg.canId);
        }
    }
    Serial.println("--- End Analysis ---");
}
