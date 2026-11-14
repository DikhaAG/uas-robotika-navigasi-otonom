#include "Gyroscope.h"

#ifdef ARDUINO
#include <Wire.h>
#endif

float Gyroscope::gyroBiasZ = 0.0f;
float Gyroscope::gyroCalibSumZ = 0.0f;
uint16_t Gyroscope::gyroCalibCount = 0;

#ifndef ARDUINO
float Gyroscope::mockGyroZ = 0.0f;
uint8_t Gyroscope::mockWhoAmI = 0x68;
#endif

#ifdef ARDUINO
bool Gyroscope::readRawZ(int16_t& rawZ) {
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x47); // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)2, (uint8_t)true);

    if (Wire.available() >= 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        rawZ = (int16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
        return true;
    }
    return false;
}
#endif

void Gyroscope::init() {
#ifdef ARDUINO
    Wire.begin();
    #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true); // 3ms timeout, reset on timeout
    #endif

    // Wake up MPU6050 from default sleep mode (PWR_MGMT_1 register 0x6B = 0)
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Set gyro full-scale range to +/- 250 deg/s (GYRO_CONFIG register 0x1B = 0)
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);
#else
    mockGyroZ = 0.0f;
    mockWhoAmI = 0x68;
#endif
    gyroBiasZ = 0.0f;
    gyroCalibSumZ = 0.0f;
    gyroCalibCount = 0;
}

uint8_t Gyroscope::getWhoAmI() {
#ifdef ARDUINO
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x75); // WHO_AM_I register
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)1, (uint8_t)true);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0x00;
#else
    return mockWhoAmI;
#endif
}

bool Gyroscope::isConnected() {
    return (getWhoAmI() == 0x68);
}

void Gyroscope::resetCalibration() {
    gyroCalibSumZ = 0.0f;
    gyroCalibCount = 0;
}

void Gyroscope::sampleCalibration() {
#ifdef ARDUINO
    int16_t rawZ = 0;
    if (readRawZ(rawZ)) {
        gyroCalibSumZ += (float)rawZ / GYRO_SCALE_250DPS;
        gyroCalibCount++;
    }
#else
    gyroCalibSumZ += mockGyroZ;
    gyroCalibCount++;
#endif
}

void Gyroscope::finalizeCalibration() {
    if (gyroCalibCount > 0) {
        gyroBiasZ = gyroCalibSumZ / (float)gyroCalibCount;
    } else {
        gyroBiasZ = 0.0f;
    }
}

float Gyroscope::readZRate() {
#ifdef ARDUINO
    int16_t rawZ = 0;
    if (readRawZ(rawZ)) {
        float dps = ((float)rawZ / GYRO_SCALE_250DPS) - gyroBiasZ;
        return dps * GYRO_SIGN;
    }
    return 0.0f;
#else
    return mockGyroZ;
#endif
}

float Gyroscope::getBias() {
    return gyroBiasZ;
}

#ifndef ARDUINO
void Gyroscope::setMockZRate(float rate_dps) {
    mockGyroZ = rate_dps;
}

void Gyroscope::setMockWhoAmI(uint8_t who) {
    mockWhoAmI = who;
}
#endif
