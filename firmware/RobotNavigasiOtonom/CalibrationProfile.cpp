#include "CalibrationProfile.h"
#include <math.h>

CalibrationProfile::CalibrationProfile() {
    resetToDefaults();
}

uint8_t CalibrationProfile::computeChecksum(const uint8_t* buffer, uint16_t length) {
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void CalibrationProfile::resetToDefaults() {
    data.magic = CALIB_MAGIC_SIGNATURE;
    data.version = CALIB_PROFILE_VERSION;
    data.diameterRodaMm = DIAMETER_RODA_MM;
    data.trimMotorKiri = TRIM_MOTOR_KIRI;
    data.trimMotorKanan = TRIM_MOTOR_KANAN;
    data.gyroBiasZ = 0.0f;
    data.invertMotorKiri = 1;
    data.invertMotorKanan = 1;
    data.checksum = computeChecksum((const uint8_t*)&data, sizeof(CalibrationData) - 1);
}

bool CalibrationProfile::isValid() const {
    if (data.magic != CALIB_MAGIC_SIGNATURE) return false;
    if (data.version != CALIB_PROFILE_VERSION) return false;
    if (data.diameterRodaMm < 30.0f || data.diameterRodaMm > 150.0f) return false;
    if (data.trimMotorKiri < 0.70f || data.trimMotorKiri > 1.30f) return false;
    if (data.trimMotorKanan < 0.70f || data.trimMotorKanan > 1.30f) return false;
    if (isnan(data.gyroBiasZ) || isinf(data.gyroBiasZ) || fabs(data.gyroBiasZ) > 50.0f) return false;
    if (data.invertMotorKiri != 1 && data.invertMotorKiri != -1) return false;
    if (data.invertMotorKanan != 1 && data.invertMotorKanan != -1) return false;
    return true;
}

bool CalibrationProfile::load() {
    CalibrationData temp;
    uint8_t* bytePtr = (uint8_t*)&temp;
    for (uint16_t i = 0; i < sizeof(CalibrationData); ++i) {
        bytePtr[i] = Hal::eepromRead(CALIB_EEPROM_START_ADDR + i);
    }

    if (temp.magic != CALIB_MAGIC_SIGNATURE || temp.version != CALIB_PROFILE_VERSION) {
        resetToDefaults();
        return false;
    }

    uint8_t expectedCrc = computeChecksum((const uint8_t*)&temp, sizeof(CalibrationData) - 1);
    if (temp.checksum != expectedCrc) {
        resetToDefaults();
        return false;
    }

    data = temp;
    if (!isValid()) {
        resetToDefaults();
        return false;
    }

    return true;
}

bool CalibrationProfile::save() {
    data.magic = CALIB_MAGIC_SIGNATURE;
    data.version = CALIB_PROFILE_VERSION;
    data.checksum = computeChecksum((const uint8_t*)&data, sizeof(CalibrationData) - 1);

    const uint8_t* bytePtr = (const uint8_t*)&data;
    for (uint16_t i = 0; i < sizeof(CalibrationData); ++i) {
        Hal::eepromUpdate(CALIB_EEPROM_START_ADDR + i, bytePtr[i]);
    }
    return true;
}

void CalibrationProfile::clearEeprom() {
    Hal::eepromUpdate(CALIB_EEPROM_START_ADDR, 0xFF);
    Hal::eepromUpdate(CALIB_EEPROM_START_ADDR + 1, 0xFF);
    resetToDefaults();
}

float CalibrationProfile::getDiameterRodaMm() const {
    return data.diameterRodaMm;
}

float CalibrationProfile::getTrimMotorKiri() const {
    return data.trimMotorKiri;
}

float CalibrationProfile::getTrimMotorKanan() const {
    return data.trimMotorKanan;
}

float CalibrationProfile::getGyroBiasZ() const {
    return data.gyroBiasZ;
}

int8_t CalibrationProfile::getInvertMotorKiri() const {
    return data.invertMotorKiri;
}

int8_t CalibrationProfile::getInvertMotorKanan() const {
    return data.invertMotorKanan;
}

void CalibrationProfile::setDiameterRodaMm(float d) {
    data.diameterRodaMm = d;
}

void CalibrationProfile::setTrimMotorKiri(float t) {
    data.trimMotorKiri = t;
}

void CalibrationProfile::setTrimMotorKanan(float t) {
    data.trimMotorKanan = t;
}

void CalibrationProfile::setGyroBiasZ(float b) {
    data.gyroBiasZ = b;
}

void CalibrationProfile::setInvertMotorKiri(int8_t inv) {
    data.invertMotorKiri = (inv == -1) ? -1 : 1;
}

void CalibrationProfile::setInvertMotorKanan(int8_t inv) {
    data.invertMotorKanan = (inv == -1) ? -1 : 1;
}
