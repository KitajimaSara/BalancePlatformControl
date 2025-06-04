#include <Arduino.h>
#include "ble_comm.h"
#include "command_dispatch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "leveling_platform.h"
#include "tension_sensor.h"
#include "wire_control.h"

static constexpr int RX2_PIN = 6;  // 避免与I2C冲突
static constexpr int TX2_PIN = 7;
static constexpr int TX1_PIN = 15;  // TX1

QueueHandle_t cmdQueue;

TensionSensor tension;
WireControl wireCtrl;
LevelingPlatform platform;
BleComm ble;

void TensionSensorTask(void* param) {
    (void)param;
    for (;;) {
        static uint32_t tsPrint = 0;
        if (millis() - tsPrint > 5000) {
            Serial.println("[TS] alive");
            tsPrint = millis();
        }

        tension.loop();
        vTaskDelay(100 / portTICK_PERIOD_MS);  // 10Hz
    }
}

void BalancingTask(void* param) {
    (void)param;
    for (;;) {
        static uint32_t blPrint = 0;
        if (millis() - blPrint > 5000) {
            Serial.println("[BL] alive");
            blPrint = millis();
        }

        platform.loop();
    }
}

void SerialCmdTask(void* param) {
    (void)param;
    CommandMessage msg;
    while (true) {
        static uint32_t scPrint = 0;
        if (millis() - scPrint > 5000) {
            Serial.println("[SC] alive");
            scPrint = millis();
        }

        if (Serial.available() && Serial.read() == 'p') {
            ble.send("ping");
        }

        if (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line.length() > 0) {
                memset(msg.text, 0, sizeof(msg.text));
                line.toCharArray(msg.text, sizeof(msg.text) - 1);
                xQueueSend(cmdQueue, &msg, 0);
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void DispatchTask(void* param) {
    CommandDispatch* dispatcher = static_cast<CommandDispatch*>(param);
    dispatcher->run();
}

// void setup() {
//     Serial.begin(115200);
//     delay(100);

//     cmdQueue = xQueueCreate(10, sizeof(CommandMessage));

//     // 初始化各模块
//     tension.begin(Serial2, RX2_PIN, TX2_PIN, 9600);
//     wireCtrl.begin(Serial1, TX1_PIN);
//     platform.begin();
//     ble.begin(cmdQueue);

//     // 创建任务
//     // xTaskCreatePinnedToCore(TensionSensorTask, "TS", 4096, nullptr, 2, nullptr, 1);
//     xTaskCreatePinnedToCore(BalancingTask, "BAL", 4096, nullptr, 4, nullptr, 1);
//     xTaskCreatePinnedToCore(SerialCmdTask, "SERIAL", 4096, nullptr, 1, nullptr, 1);

//     static CommandDispatch dispatcher(cmdQueue, tension, wireCtrl, platform);
//     xTaskCreatePinnedToCore(DispatchTask, "DISPATCH", 4096, &dispatcher, 3, nullptr, 1);

//     // 删除Arduino loop task
//     vTaskDelete(NULL);
// }

void setup() {
    Serial.begin(115200);
    delay(50);
    Serial.println("== STEP 1: Serial OK ==");

    cmdQueue = xQueueCreate(10, sizeof(CommandMessage));
    Serial.println("== STEP 2: Queue OK ==");

    tension.begin(Serial2, RX2_PIN, TX2_PIN, 9600);
    Serial.println("== STEP 3: Tension.begin OK ==");

    wireCtrl.begin(Serial1, TX1_PIN);
    Serial.println("== STEP 4: WireCtrl.begin OK ==");

    platform.begin();
    Serial.println("== STEP 5: Platform.begin OK ==");

    ble.begin(cmdQueue);
    Serial.println("== STEP 6: BLE.begin OK ==");

    // 创建任务…
    xTaskCreatePinnedToCore(TensionSensorTask, "TS", 4096, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(BalancingTask, "BAL", 4096, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(SerialCmdTask, "SER", 4096, nullptr, 1, nullptr, 1);
    static CommandDispatch dispatcher(cmdQueue, tension, wireCtrl, platform);
    xTaskCreatePinnedToCore(DispatchTask, "DISP", 4096, &dispatcher, 3, nullptr, 1);

    Serial.println("== STEP 7: Tasks created ==");

    // vTaskDelete(NULL);   // 暂时别删
}

void loop() {
    // 不再使用
}