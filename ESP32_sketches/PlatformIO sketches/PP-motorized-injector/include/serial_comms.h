#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#include <Arduino.h>
#include <SafeString.h>

// Function declarations
void setupSerial();
void serialRegular100msMessages();
void handleSerialCommands();

#endif // SERIAL_COMMS_H