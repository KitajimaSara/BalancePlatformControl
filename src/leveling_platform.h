#ifndef LEVELING_PLATFORM_H
#define LEVELING_PLATFORM_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
extern volatile bool gAngleStreamEnabled;

class LevelingPlatform {
public:
    void begin();
    void start();                 // 启用闭环
    void stop();                  // 关闭闭环并锁定当前舵机角
    bool isActive() const { return _active; }

    // 动态调整 PID
    void setPID(float kpPitch, float kiPitch, float kdPitch,
                float kpRoll,  float kiRoll,  float kdRoll);

    void loop();                  // 周期性控制（50 Hz）

    // ★ 新增：获取当前 YPR（单位：度）
    void getYPR(float &yaw, float &pitch, float &roll) const;
    void getBottomYPR(float &yaw, float &pitch, float &roll) const;

private:
    /* ---------- 硬件映射 ---------- */
    static const int I2C_SDA = 18;
    static const int I2C_SCL = 17;
    static const int I2C_BOTTOM_SDA = 4;   // ★ 新增
    static const int I2C_BOTTOM_SCL = 5;   // ★ 新增
    static const int SERVO1_PIN = 9;
    static const int SERVO2_PIN = 10;
    static const int SERVO1_FEEDBACK_PIN = 14;
    static const int SERVO2_FEEDBACK_PIN = 16;

    /* ---------- 舵机静态参数 ---------- */
    static const int SERVO1_STOP_ANGLE = 90;
    static const int SERVO2_STOP_ANGLE = 90;
    static const int SERVO1_MIN_ANGLE  = 227;
    static const int SERVO1_MAX_ANGLE  = 340;
    static const int SERVO2_MIN_ANGLE  = 175;
    static const int SERVO2_MAX_ANGLE  = 283;

    /* ---------- 控制目标 ---------- */
    const float _targetPitch = 2.5f;      // °  实测偏置
    const float _targetRoll  = 2.0f;      // °
    const float _tolerance   = 2.0f;      // °  允许误差

    /* ---------- PID 参数 ---------- */
    float _kpPitch = 300.0f, _kiPitch = 0.0f, _kdPitch = 0.0f;
    float _kpRoll  = 200.0f, _kiRoll  = 0.0f, _kdRoll  = 40.0f;

    /* ---------- 状态量 ---------- */
    bool   _active = false;
    float  _pitchIntegral = 0, _pitchLastErr = 0;
    float  _rollIntegral  = 0, _rollLastErr  = 0;
    unsigned long _prevPitchMicros = 0;
    unsigned long _prevRollMicros  = 0;

    /* ---------- 设备对象 ---------- */
    MPU6050 _mpu;
    MPU6050 _mpuBottom;
    Servo   _servo1, _servo2;

    uint8_t      _fifoBuf[64];
    uint8_t     _fifoBufBottom[64];
    Quaternion   _q, _qBottom;
    VectorFloat  _gravity, _gravityBottom;
    float        _ypr[3];
    float        _yprBottom[3];

    /* ---------- 工具函数 ---------- */
    float readFeedbackAngle(int pin);     // 把 0‑4095 ADC → 0‑360°
};

#endif
