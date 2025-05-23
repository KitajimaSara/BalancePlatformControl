#ifndef BLE_COMM_H
#define BLE_COMM_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct CommandMessage {
    char text[32];
};

class BleComm {
public:
    void begin(QueueHandle_t q);

private:
    QueueHandle_t _queue;
};

#endif