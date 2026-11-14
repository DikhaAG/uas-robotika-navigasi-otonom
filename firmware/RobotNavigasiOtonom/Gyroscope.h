#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include "Config.h"
#include <stdint.h>

class Gyroscope {
public:
    static void init();
    static bool isConnected();
    static uint8_t getWhoAmI();
    static void resetCalibration();
    static void sampleCalibration();
    static void finalizeCalibration();
    static float readZRate(); // Angular velocity Z in deg/sec
    static float getBias();

#ifndef ARDUINO
    // Mock methods for desktop TDD & simulation
    static void setMockZRate(float rate_dps);
    static void setMockWhoAmI(uint8_t who);
#endif

private:
#ifdef ARDUINO
    static bool readRawZ(int16_t& rawZ);
#endif
    static float gyroBiasZ;
    static float gyroCalibSumZ;
    static uint16_t gyroCalibCount;

#ifndef ARDUINO
    static float mockGyroZ;
    static uint8_t mockWhoAmI;
#endif
};

#endif // GYROSCOPE_H
