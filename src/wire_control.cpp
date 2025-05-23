#include "wire_control.h"

static const char *CMD_ONE   = "CMD_ONE";
static const char *CMD_TWO   = "CMD_TWO";
static const char *CMD_THREE = "CMD_THREE";
static const char *CMD_FOUR  = "CMD_FOUR";
static const char *CMD_STOP  = "CMD_STOP";

void WireControl::begin(HardwareSerial &port, int txPin, uint32_t baud) {
    _serial = &port;
    _serial->begin(baud, SERIAL_8N1, -1, txPin); // TX only
}

void WireControl::send(const char *cmd) {
    _serial->println(cmd);

    Serial.printf("[WIRE] TX: %s\n", cmd);   // ★ 调试
    
}

void WireControl::cmdOne()   { send(CMD_ONE);  }
void WireControl::cmdTwo()   { send(CMD_TWO);  }
void WireControl::cmdThree() { send(CMD_THREE);}
void WireControl::cmdFour()  { send(CMD_FOUR); }
void WireControl::stop()     { send(CMD_STOP); }