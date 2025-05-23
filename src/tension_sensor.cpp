#include "tension_sensor.h"

const uint8_t TensionSensor::CMD_READ[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
const uint8_t TensionSensor::CMD_WRITE_AVAILABLE[8] = {0x01, 0x06, 0x00, 0x17, 0x00, 0x01, 0xF8, 0x0E};
const uint8_t TensionSensor::CMD_TARE_ON[8] = {0x01, 0x06, 0x00, 0x15, 0x00, 0x01, 0x59, 0xCE};
const uint8_t TensionSensor::CMD_TARE_OFF[8] = {0x01, 0x06, 0x00, 0x15, 0x00, 0x02, 0x19, 0xCF};

void TensionSensor::begin(HardwareSerial& port, int rxPin, int txPin, uint32_t baud) {
    _serial = &port;
    _serial->begin(baud, SERIAL_8N1, rxPin, txPin);
}

void TensionSensor::sendCmd(const uint8_t* cmd, size_t len) {
    while (_serial->available())
        _serial->read();  // flush
    _serial->write(cmd, len);
}

bool TensionSensor::readBuf(uint8_t* buf, size_t len, uint16_t timeout) {
    size_t idx = 0;
    uint32_t t0 = millis();
    while (idx < len && millis() - t0 < timeout) {
        if (_serial->available()) {
            buf[idx++] = _serial->read();
        }
    }
    return idx == len;
}

bool TensionSensor::writeCmdWithAck(const uint8_t* cmd, size_t len) {
    sendCmd(cmd, len);
    delay(50);
    uint8_t resp[8];
    if (!readBuf(resp, 8))
        return false;
    for (int i = 0; i < 8; ++i) {
        if (resp[i] != cmd[i])
            return false;
    }
    return true;
}

bool TensionSensor::readForce(float& f) {
    uint8_t resp[9];
    sendCmd(CMD_READ, sizeof(CMD_READ));
    delay(5);
    if (!readBuf(resp, 9))
        return false;

    uint32_t raw = ((uint32_t)resp[5] << 24) |
                   ((uint32_t)resp[6] << 16) |
                   ((uint32_t)resp[3] << 8) |
                   ((uint32_t)resp[4]);

    f = (int32_t)raw / 1000.0f;
    _lastForce = f;

    // static uint32_t tsDataPrint = 0;
    // if (millis() - tsDataPrint > 1500) {
    //     // 串口打印传感器值（调试用）
    //     Serial.printf("[TS] force: %.5f N\n", f);
    //     tsDataPrint = millis();
    // }

    return true;
}

void TensionSensor::loop() {
    float f;
    if (readForce(f)) {
        if (abs(f) > _thresholdN) {
            // TODO: 调用联动逻辑（如停止拉绳）
        }
    }
}

void TensionSensor::tare() {
    float f;
    // if (readForce(f)) {
    //     _tareOffset += f;
    // }
    writeCmdWithAck(CMD_WRITE_AVAILABLE, sizeof(CMD_WRITE_AVAILABLE));
    delay(200);  // 等设备处理写保护解除
    writeCmdWithAck(CMD_TARE_ON, sizeof(CMD_TARE_ON));
}

void TensionSensor::untare() {
    // _tareOffset = 0;
    writeCmdWithAck(CMD_WRITE_AVAILABLE, sizeof(CMD_WRITE_AVAILABLE));
    delay(200);  // 等设备处理写保护解除
    writeCmdWithAck(CMD_TARE_OFF, sizeof(CMD_TARE_OFF));
}