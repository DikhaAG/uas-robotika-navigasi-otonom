#ifndef SENSORS_H
#define SENSORS_H

#include "Config.h"
#include "Hal.h"
#include <stdint.h>

class Sensors {
public:
    static void init();

    // MPU6050 Gyroscope
    static void initMPU6050();
    static uint8_t getWhoAmI();
    static bool isMpuConnected();
    static void resetGyroCalibration();
    static void sampleGyroCalibration();
    static void finalizeGyroCalibration();
    static void calibrateGyro(uint16_t samples = 200);
    static float readGyroZRate(); // Returns rate in deg/sec
    static float getGyroBias();

    // HC-SR04 Ultrasonic Sensor (Non-blocking)
    static void updateUltrasonic();
    static float getDistance(); // Returns distance in cm

    // Lateral FC-51 Infrared Proximity Sensors
    static bool isWallLeft();   // True if left wall detected
    static bool isWallRight();  // True if right wall detected

#ifndef ARDUINO
    // Mock methods for desktop TDD
    static void setMockGyroZ(float rate_dps);
    static void setMockDistance(float dist_cm);
    static void setMockIr(bool left_detected, bool right_detected);
    static void setMockWhoAmI(uint8_t who);
#endif

private:
    static float gyroBiasZ;
    static float currentDistance;
    static uint32_t lastSonarPingMs;
    static bool wallLeftDetected;
    static bool wallRightDetected;

#ifndef ARDUINO
    static float mockGyroZ;
    static float mockDistance;
    static bool mockIrLeft;
    static bool mockIrRight;
    static uint8_t mockWhoAmI;
#endif
};

#endif // SENSORS_H
