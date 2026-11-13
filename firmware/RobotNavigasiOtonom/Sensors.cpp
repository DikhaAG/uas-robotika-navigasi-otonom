#include "Sensors.h"

#ifdef ARDUINO
#include <Wire.h>
#endif

float Sensors::gyroBiasZ = 0.0f;
float Sensors::currentDistance = 400.0f;
uint32_t Sensors::lastSonarPingMs = 0;
bool Sensors::wallLeftDetected = false;
bool Sensors::wallRightDetected = false;

#ifndef ARDUINO
float Sensors::mockGyroZ = 0.0f;
float Sensors::mockDistance = 400.0f;
bool Sensors::mockIrLeft = false;
bool Sensors::mockIrRight = false;
#endif

static float gyroCalibSumZ = 0.0f;
static uint16_t gyroCalibCount = 0;

void Sensors::init() {
    Hal::pinMode(PIN_TRIG, OUTPUT);
    Hal::pinMode(PIN_ECHO, INPUT);
    Hal::digitalWrite(PIN_TRIG, LOW);

    Hal::pinMode(PIN_IR_KIRI, INPUT);
    Hal::pinMode(PIN_IR_KANAN, INPUT);

#ifdef ARDUINO
    Wire.begin();
    #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true); // 3ms timeout, reset on timeout
    #endif
    initMPU6050();
#else
    mockDistance = 400.0f;
    mockGyroZ = 0.0f;
    mockIrLeft = false;
    mockIrRight = false;
#endif
}

void Sensors::resetGyroCalibration() {
    gyroCalibSumZ = 0.0f;
    gyroCalibCount = 0;
}

void Sensors::sampleGyroCalibration() {
#ifdef ARDUINO
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x47); // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)2, (uint8_t)true);

    if (Wire.available() >= 2) {
        int16_t rawZ = (int16_t)((Wire.read() << 8) | Wire.read());
        gyroCalibSumZ += (float)rawZ / GYRO_SCALE_250DPS;
        gyroCalibCount++;
    }
#endif
}

void Sensors::finalizeGyroCalibration() {
    if (gyroCalibCount > 0) {
        gyroBiasZ = gyroCalibSumZ / (float)gyroCalibCount;
    } else {
        gyroBiasZ = 0.0f;
    }
}

void Sensors::calibrateGyro(uint16_t samples) {
    if (samples == 0) samples = 200;

#ifdef ARDUINO
    float sumZ = 0.0f;
    for (uint16_t i = 0; i < samples; ++i) {
        Wire.beginTransmission(MPU6050_I2C_ADDR);
        Wire.write(0x47); // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)2, (uint8_t)true);

        if (Wire.available() >= 2) {
            int16_t rawZ = (int16_t)((Wire.read() << 8) | Wire.read());
            sumZ += (float)rawZ / GYRO_SCALE_250DPS;
        }
    }
    gyroBiasZ = sumZ / (float)samples;
#else
    (void)samples;
    gyroBiasZ = 0.0f;
#endif
}

float Sensors::readGyroZRate() {
#ifdef ARDUINO
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x47); // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)2, (uint8_t)true);

    if (Wire.available() >= 2) {
        int16_t rawZ = (int16_t)((Wire.read() << 8) | Wire.read());
        float dps = ((float)rawZ / GYRO_SCALE_250DPS) - gyroBiasZ;
        return dps * GYRO_SIGN;
    }
    return 0.0f;
#else
    return mockGyroZ;
#endif
}

float Sensors::getGyroBias() {
    return gyroBiasZ;
}

void Sensors::updateUltrasonic() {
    uint32_t now = Hal::millis();
    if (now - lastSonarPingMs < PERIODE_SONAR_MS) {
        return;
    }
    lastSonarPingMs = now;

#ifdef ARDUINO
    // Non-blocking trigger pulse
    Hal::digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    Hal::digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    Hal::digitalWrite(PIN_TRIG, LOW);

    // Timeout of 25000 us (~400 cm)
    unsigned long duration = pulseIn(PIN_ECHO, HIGH, 25000UL);
    if (duration == 0) {
        currentDistance = 400.0f;
    } else {
        currentDistance = (float)duration * 0.0343f / 2.0f;
    }
#else
    currentDistance = mockDistance;
#endif
}

float Sensors::getDistance() {
#ifdef ARDUINO
    return currentDistance;
#else
    return mockDistance;
#endif
}

bool Sensors::isWallLeft() {
#ifdef ARDUINO
    // FC-51 sensor is active LOW
    return (Hal::digitalRead(PIN_IR_KIRI) == LOW);
#else
    return mockIrLeft;
#endif
}

bool Sensors::isWallRight() {
#ifdef ARDUINO
    return (Hal::digitalRead(PIN_IR_KANAN) == LOW);
#else
    return mockIrRight;
#endif
}

#ifndef ARDUINO
void Sensors::setMockGyroZ(float rate_dps) {
    mockGyroZ = rate_dps;
}

void Sensors::setMockDistance(float dist_cm) {
    mockDistance = dist_cm;
}

void Sensors::setMockIr(bool left_detected, bool right_detected) {
    mockIrLeft = left_detected;
    mockIrRight = right_detected;
}
#endif
