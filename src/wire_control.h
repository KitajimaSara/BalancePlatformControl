#ifndef WIRE_CONTROL_H
#define WIRE_CONTROL_H

#include <Arduino.h>

class WireControl {
public:
    void begin(HardwareSerial &port, int txPin, uint32_t baud = 9600);
    void cmdOne();
    void cmdTwo();
    void cmdThree();
    void cmdFour();
    void stop();
private:
    HardwareSerial *_serial{nullptr};
    void send(const char *cmd);
};

#endif