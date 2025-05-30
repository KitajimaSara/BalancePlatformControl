#include "command_dispatch.h"
#include <cstring>

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