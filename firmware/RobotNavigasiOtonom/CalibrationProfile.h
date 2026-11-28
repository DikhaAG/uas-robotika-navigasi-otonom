#ifndef CALIBRATION_PROFILE_H
#define CALIBRATION_PROFILE_H

#include "Config.h"
#include "Hal.h"
#include <stdint.h>

#define CALIB_MAGIC_SIGNATURE 0xCA1B
#define CALIB_PROFILE_VERSION 0x02
#define CALIB_PROFILE_VERSION_V1 0x01
#define CALIB_EEPROM_START_ADDR 0

#pragma pack(push, 1)
struct CalibrationData {
    uint16_t magic;           // 0xCA1B
    uint8_t  version;         // 0x01
    float    diameterRodaMm;  // effective rolling diameter (mm)
    float    trimMotorKiri;   // left motor PWM trim multiplier
    float    trimMotorKanan;  // right motor PWM trim multiplier
    float    gyroBiasZ;       // zero-rate yaw drift (deg/s)
    int8_t   invertMotorKiri; // +1 (normal) or -1 (inverted)
    int8_t   invertMotorKanan;// +1 (normal) or -1 (inverted)
    int8_t   gyroSign;        // +1 (MPU6050 chip facing up) or -1 (facing down)
    uint8_t  checksum;        // 8-bit CRC over preceding fields
};

// Version-1 layout (before gyroSign), read once to migrate older EEPROM profiles
struct CalibrationDataV1 {
    uint16_t magic;
    uint8_t  version;
    float    diameterRodaMm;
    float    trimMotorKiri;
    float    trimMotorKanan;
    float    gyroBiasZ;
    int8_t   invertMotorKiri;
    int8_t   invertMotorKanan;
    uint8_t  checksum;
};
#pragma pack(pop)

class CalibrationProfile {
public:
    CalibrationProfile();

    void resetToDefaults();
    bool isValid() const;
    bool load();
    bool save();
    void clearEeprom();

    // Getters
    float getDiameterRodaMm() const;
    float getTrimMotorKiri() const;
    float getTrimMotorKanan() const;
    float getGyroBiasZ() const;
    int8_t getInvertMotorKiri() const;
    int8_t getInvertMotorKanan() const;
    int8_t getGyroSign() const;

    // Setters
    void setDiameterRodaMm(float d);
    void setTrimMotorKiri(float t);
    void setTrimMotorKanan(float t);
    void setGyroBiasZ(float b);
    void setInvertMotorKiri(int8_t inv);
    void setInvertMotorKanan(int8_t inv);
    void setGyroSign(int8_t sign); // stored as given; isValid() rejects anything but +1/-1

    // Compute CRC-8 checksum over byte buffer
    static uint8_t computeChecksum(const uint8_t* data, uint16_t length);

private:
    bool loadVersion1();
    CalibrationData data;
};

#endif // CALIBRATION_PROFILE_H
