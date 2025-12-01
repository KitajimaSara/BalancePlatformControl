#include "command_dispatch.h"
#include <cstring>

extern volatile bool gTensionStreamEnabled;
extern volatile bool gAngleStreamEnabled;
// ★ 新增：使用全局 BLE 对象
extern BleComm ble;

void CommandDispatch::process(const char *cmd) {
    if (strcmp(cmd, "CMD_ONE") == 0)      { _wc.cmdOne(); }
    else if (strcmp(cmd, "CMD_TWO") == 0) { _wc.cmdTwo(); }
    else if (strcmp(cmd, "CMD_THREE")==0) { _wc.cmdThree();}
    else if (strcmp(cmd, "CMD_FOUR") ==0) { _wc.cmdFour(); }
    else if (strcmp(cmd, "CMD_STOP") ==0) { _wc.stop(); }
    else if (strcmp(cmd, "StartBalancing") == 0) { _lp.start(); }
    else if (strcmp(cmd, "StopBalancing")  == 0) { _lp.stop();  }
    else if (strcmp(cmd, "Tare") == 0)    { _ts.tare(); }
    else if (strcmp(cmd, "Untare") == 0)  { _ts.untare(); }
    else if (strcmp(cmd,"StartWinchAttempt")==0) {
        _wc.cmdFour();            // 拉线启动
        gWinchActive = true;
        gWinchBaseForceN = _ts.getLastForce();   // 记录基准
        Serial.println("[WINCH] Attempt started");
    }
    else if (strcmp(cmd, "PassiveLanding")==0) {
        _wc.cmdFour();            // 拉线启动
        gLandingActive = true;
        gWinchBaseForceN = _ts.getLastForce();   // 记录基准
        Serial.println("[WINCH] Passive landing started");
    }
    // ★★★ 新增：单次输出 YPR + 张力到 BLE
    else if (strcmp(cmd, "DumpSensors") == 0) {
        float yaw, pitch, roll;
        _lp.getYPR(yaw, pitch, roll);           // 读取当前姿态（度）

        float forceN = _ts.getLastForce();      // 使用最近一次张力值，避免和 loop() 抢串口

        char buf[96];
        snprintf(buf, sizeof(buf),
                 "YPR(deg): %.2f, %.2f, %.2f; F(N): %.3f",
                 yaw, pitch, roll, forceN);

        ble.send(buf);
        Serial.println("[CMD] DumpSensors sent over BLE");
    }
    // ★★★ 第一组：张力记录开关
    else if (strcmp(cmd, "StartTensionStream") == 0) {
        gTensionStreamEnabled = true;
        ble.send("TS,START");              // 可选：给手机一个确认
        Serial.println("[CMD] Tension stream ON");
    }
    else if (strcmp(cmd, "StopTensionStream") == 0) {
        gTensionStreamEnabled = false;
        ble.send("TS,STOP");
        Serial.println("[CMD] Tension stream OFF");
    }
    // ★★★ 第二组：角度记录开关
    else if (strcmp(cmd, "StartAngleStream") == 0) {
        gAngleStreamEnabled = true;
        ble.send("YPR,START");
        Serial.println("[CMD] Angle stream ON");
    }
    else if (strcmp(cmd, "StopAngleStream") == 0) {
        gAngleStreamEnabled = false;
        ble.send("YPR,STOP");
        Serial.println("[CMD] Angle stream OFF");
    }
    else { Serial.printf("[CMD] Unknown: %s\n", cmd); }
}

void CommandDispatch::run() {
    CommandMessage msg;
    while (true) {
        if (xQueueReceive(_queue, &msg, portMAX_DELAY)) {
            process(msg.text);
        }
    }
}