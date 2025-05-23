#include "ble_comm.h"

static NimBLECharacteristic *pCharacteristic;
static const char *SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *CHAR_UUID    = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

class CmdCallback : public NimBLECharacteristicCallbacks {
public:
    CmdCallback(QueueHandle_t q) : _queue(q) {}
    void onWrite(NimBLECharacteristic *pChr) override {
        std::string v = pChr->getValue();
        if (v.empty()) return;
        CommandMessage msg;
        memset(msg.text, 0, sizeof(msg.text));
        strncpy(msg.text, v.c_str(), sizeof(msg.text)-1);
        xQueueSend(_queue, &msg, 0);
    }
private:
    QueueHandle_t _queue;
};

void BleComm::begin(QueueHandle_t q) {
    _queue = q;
    NimBLEDevice::init("ESP32S3_TetherPad");
    NimBLEServer *server = NimBLEDevice::createServer();
    NimBLEService *service = server->createService(SERVICE_UUID);

    pCharacteristic = service->createCharacteristic(CHAR_UUID,
                        NIMBLE_PROPERTY::WRITE);
    pCharacteristic->setCallbacks(new CmdCallback(_queue));
    service->start();
    NimBLEAdvertising *ad = NimBLEDevice::getAdvertising();
    ad->addServiceUUID(SERVICE_UUID);
    NimBLEDevice::startAdvertising();
}