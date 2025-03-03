#include "serial_comms.h"
#include "config.h"
#include "state_machine.h"  // eventually will be sending state machine data to serial

void setupSerial()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }
}

void serialRegular100msMessages()
{
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 100)
    {
        lastTime = currentTime;
        // Send regular messages every 100ms
        Serial.print("Encoder Position: ");
        Serial.println(encoder.getCount());
        Serial.print("Nozzle Temperature: ");
        Serial.println(nozzleTemperature);
    }
}

void handleSerialCommands()
{
    // Handle incoming serial commands here
}