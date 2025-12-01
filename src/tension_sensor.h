#ifndef TENSION_SENSOR_H
#define TENSION_SENSOR_H

#include <Arduino.h>
extern volatile bool gTensionStreamEnabled;

class TensionSensor {
public:
    void begin(HardwareSerial &port, int rxPin, int txPin, uint32_t baud = 9600);
    void loop();                     // 周期性读取张力值
    bool readForce(float &force);    // 立即读取一次
    void tare();                     // 去皮
    void untare();                   // 取消去皮
    float getLastForce() const { return _lastForce; }
    void setThreshold(float n) { _thresholdN = n; }

private:
    HardwareSerial *_serial{nullptr};
    // float _tareOffset{0.0f};
    float _lastForce{0.0f};
    float _thresholdN{1000.0f};

    // --- 协议相关 ---
    static const uint8_t CMD_READ[8];
    static const uint8_t CMD_WRITE_AVAILABLE[8];
    static const uint8_t CMD_TARE_ON[8];
    static const uint8_t CMD_TARE_OFF[8];

    void sendCmd(const uint8_t *cmd, size_t len);
    bool readBuf(uint8_t *buf, size_t len, uint16_t timeout = 200);
    bool writeCmdWithAck(const uint8_t *cmd, size_t len);
};

extern volatile bool gWinchActive;
extern volatile bool gLandingActive;
extern float gWinchBaseForceN;

#endif