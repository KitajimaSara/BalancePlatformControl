#ifndef COMMAND_DISPATCH_H
#define COMMAND_DISPATCH_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "tension_sensor.h"
#include "wire_control.h"
#include "leveling_platform.h"
#include "ble_comm.h"

class CommandDispatch {
public:
    CommandDispatch(QueueHandle_t q,
                    TensionSensor &ts,
                    WireControl &wc,
                    LevelingPlatform &lp)
        : _queue(q), _ts(ts), _wc(wc), _lp(lp) {}
    void run(); // 主循环

private:
    QueueHandle_t _queue;
    TensionSensor &_ts;
    WireControl   &_wc;
    LevelingPlatform &_lp;

    void process(const char *cmd);
};

#endif