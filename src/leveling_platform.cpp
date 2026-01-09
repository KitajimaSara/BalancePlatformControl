#include "leveling_platform.h"
#include "ble_comm.h"          // 为了发 BLE
extern BleComm ble;

volatile bool gAngleStreamEnabled = false;     // ★ 新增

/* ====== 初始化 ====== */
void LevelingPlatform::begin() {
    Wire.begin(I2C_SDA, I2C_SCL, 400000);
    _mpu.initialize();
    _mpu.dmpInitialize();
    _mpu.setDMPEnabled(true);

    Wire1.begin(I2C_BOTTOM_SDA, I2C_BOTTOM_SCL, 400000);
    _mpuBottom.initialize();
    _mpuBottom.dmpInitialize();
    _mpuBottom.setDMPEnabled(true);

    pinMode(SERVO1_FEEDBACK_PIN, INPUT);
    pinMode(SERVO2_FEEDBACK_PIN, INPUT);

    _servo1.attach(SERVO1_PIN);
    _servo2.attach(SERVO2_PIN);
    _servo1.write(SERVO1_STOP_ANGLE);
    _servo2.write(SERVO2_STOP_ANGLE);

    _prevPitchMicros = micros();
    _prevRollMicros = micros();
}

/* ====== PID 调参接口 ====== */
void LevelingPlatform::setPID(float kpP, float kiP, float kdP, float kpR, float kiR, float kdR) {
    _kpPitch = kpP;
    _kiPitch = kiP;
    _kdPitch = kdP;
    _kpRoll = kpR;
    _kiRoll = kiR;
    _kdRoll = kdR;
}

/* ====== 启停 ====== */
void LevelingPlatform::start() {
    _active = true;
    _pitchIntegral = _rollIntegral = 0;
    _pitchLastErr = _rollLastErr = 0;
    _prevPitchMicros = _prevRollMicros = micros();
}
void LevelingPlatform::stop() {
    _active = false;
    _servo1.write(SERVO1_STOP_ANGLE);
    _servo2.write(SERVO2_STOP_ANGLE);
}

/* ====== ADC → 角度 ====== */
float LevelingPlatform::readFeedbackAngle(int pin) {
    return map(analogRead(pin), 0, 4095, 0, 360);
}

/* ====== 主闭环 ====== */
void LevelingPlatform::loop() {
    if (!_active) {  // 未启用则小睡
        vTaskDelay(50 / portTICK_PERIOD_MS);
        return;
    }

    /* 1. 读取姿态 */
    if (_mpu.dmpGetCurrentFIFOPacket(_fifoBuf)) {
        _mpu.dmpGetQuaternion(&_q, _fifoBuf);
        _mpu.dmpGetGravity(&_gravity, &_q);
        _mpu.dmpGetYawPitchRoll(_ypr, &_q, &_gravity);
        for (float& v : _ypr)
            v *= 180.0f / M_PI;
    }
    if (_mpuBottom.dmpGetCurrentFIFOPacket(_fifoBufBottom)) {
        _mpuBottom.dmpGetQuaternion(&_qBottom, _fifoBufBottom);
        _mpuBottom.dmpGetGravity(&_gravityBottom, &_qBottom);
        _mpuBottom.dmpGetYawPitchRoll(_yprBottom, &_qBottom, &_gravityBottom);
        for (float &v : _yprBottom) v *= 180.0f / M_PI;
    }
    float pitch = _ypr[1];
    float roll = _ypr[2];

    // ★★★ 新增：如果开启了角度流，就通过 BLE 发一行
    if (gAngleStreamEnabled) {
        float yaw, pitch, roll, yawBottom, pitchBottom, rollBottom;
        getYPR(yaw, pitch, roll);
        getBottomYPR(yawBottom, pitchBottom, rollBottom);

        char buf[160];
        snprintf(buf, sizeof(buf),
                 "YPR,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                 (unsigned long)millis(),
                 yaw, pitch, roll,
                 yawBottom, pitchBottom, rollBottom);
        ble.send(buf);
    }

    // static uint32_t dbg = 0;
    // if (millis() - dbg > 1000 && _active) {
    //     Serial.printf("[BAL] Pitch=%.2f Roll=%.2f\n", _ypr[1], _ypr[2]);
    //     dbg = millis();
    // }

    /* 2. 误差与时间差 */
    float eP = _targetPitch - pitch;
    float eR = _targetRoll - roll;
    bool inP = fabs(eP) < _tolerance;
    bool inR = fabs(eR) < _tolerance;

    unsigned long now = micros();
    float dtP = (now - _prevPitchMicros) / 1e6f;
    float dtR = (now - _prevRollMicros) / 1e6f;
    if (dtP == 0)
        dtP = 1e-3f;
    if (dtR == 0)
        dtR = 1e-3f;

    /* 3‑A. Pitch PID & 补偿 */
    if (inP) {
        _servo1.write(SERVO1_STOP_ANGLE);
        _pitchIntegral = 0;
    } else {
        _pitchIntegral += eP * dtP;
        _pitchIntegral = constrain(_pitchIntegral, -50.0f, 50.0f);
        float dP = (eP - _pitchLastErr) / dtP;
        _pitchLastErr = eP;

        float out = _kpPitch * eP + _kiPitch * _pitchIntegral + _kdPitch * dP;
        out = constrain(out, -8000.0f, 8000.0f);
        out = out * 90.0f / 8000.0f;  // 映射到 ±90°

        if (out > 0 && out < 30)
            out = min(out + 20, 40.0f);  // 功率补偿

        float fb = readFeedbackAngle(SERVO1_FEEDBACK_PIN);
        if (fb > 120 && fb < SERVO1_MIN_ANGLE && out < 0)
            out = 10;
        else if ((fb > SERVO1_MAX_ANGLE || fb <= 120) && out > 0)
            out = -10;

        _servo1.write(SERVO1_STOP_ANGLE + out);
    }
    _prevPitchMicros = now;

    /* 3‑B. Roll PID & 补偿 */
    if (inR) {
        _servo2.write(SERVO2_STOP_ANGLE);
        _rollIntegral = 0;
    } else {
        _rollIntegral += eR * dtR;
        _rollIntegral = constrain(_rollIntegral, -50.0f, 50.0f);
        float dR = (eR - _rollLastErr) / dtR;
        _rollLastErr = eR;

        float out = _kpRoll * eR + _kiRoll * _rollIntegral + _kdRoll * dR;
        out = constrain(out, -8000.0f, 8000.0f);
        out = out * 90.0f / 8000.0f;

        if (out > 0 && out < 30)
            out = min(out + 20, 40.0f);

        float fb = readFeedbackAngle(SERVO2_FEEDBACK_PIN);
        if (fb < SERVO2_MIN_ANGLE && out < 0)
            out = 10;
        else if (fb > SERVO2_MAX_ANGLE && out > 0)
            out = -10;

        _servo2.write(SERVO2_STOP_ANGLE + out);
    }
    _prevRollMicros = now;

    /* 4. 维持 50 Hz */
    vTaskDelay(20 / portTICK_PERIOD_MS);
}

void LevelingPlatform::getYPR(float &yaw, float &pitch, float &roll) const {
    yaw   = _ypr[0];
    pitch = _ypr[1];
    roll  = _ypr[2];
}

void LevelingPlatform::getBottomYPR(float &yaw, float &pitch, float &roll) const {
    yaw   = _yprBottom[0];
    pitch = _yprBottom[1];
    roll  = _yprBottom[2];
}