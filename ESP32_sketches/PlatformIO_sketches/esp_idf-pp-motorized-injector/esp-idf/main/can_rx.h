#ifndef CAN_RX_H
#define CAN_RX_H

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class CanRxHandler {
public:
    static CanRxHandler& getInstance();

    bool begin();
    bool startCore0Task();

private:
    CanRxHandler();
    static void pollingTaskCore0(void* pvParameters);
    void pollAndProcess();

    TaskHandle_t pollingTaskHandle_;
};

#endif // CAN_RX_H
