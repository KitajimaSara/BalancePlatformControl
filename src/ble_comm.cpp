#include "ble_comm.h"
#include <NimBLEDevice.h>

/* ---------- UUID ---------- */
static const char *UUID_SVC = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *UUID_RX  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // Central → ESP32
static const char *UUID_TX  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // ESP32 → Central

/* ---------- 全局 ---------- */
static NimBLECharacteristic *gTxChr = nullptr;
static QueueHandle_t gQueue = nullptr;

/* ---------- 写回调 ---------- */
class CmdCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *chr) override {
        std::string v = chr->getValue();
        if (v.empty()) return;
        CommandMessage m{};
        strncpy(m.text, v.c_str(), sizeof(m.text) - 1);
        xQueueSend(gQueue, &m, 0);
    }
};

void BleComm::begin(QueueHandle_t q) {
    gQueue = q;

    NimBLEDevice::init("TetherPad");
    NimBLEDevice::setPower(ESP_PWR_LVL_P6);

    auto *srv = NimBLEDevice::createServer();
    auto *svc = srv->createService(UUID_SVC);

    /* ---- 0002: RX 写入 ---- */
    auto *rx = svc->createCharacteristic(
        UUID_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rx->setCallbacks(new CmdCB());
    rx->createDescriptor("2901", NIMBLE_PROPERTY::READ)->setValue("RX-Write");

    /* ---- 0003: TX Notify ---- */
    gTxChr = svc->createCharacteristic(
        UUID_TX, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    gTxChr->createDescriptor("2901", NIMBLE_PROPERTY::READ)->setValue("TX-Notify");
    gTxChr->setValue("boot");                        // 默认值

    svc->start();

    auto *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(UUID_SVC);
    adv->setName("TetherPad");
    adv->start();

    Serial.println("[BLE] Advertising with RX(0002) & TX(0003)");
}

/* ---- 发送给手机 ---- */
void BleComm::send(const char *msg) {
    if (!gTxChr) {
        Serial.println("[BLE] TX Characteristic not initialized!");
        return;
    }
    gTxChr->setValue((uint8_t *)msg, strlen(msg)); // 设置值
    gTxChr->notify();            // 手机需先订阅 TX-Notify
    Serial.printf("[BLE TX] %s\n", msg);  // 调试输出
}
