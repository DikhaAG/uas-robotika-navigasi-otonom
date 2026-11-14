#ifndef ULTRASONIC_SONAR_H
#define ULTRASONIC_SONAR_H

#include "Config.h"
#include "Hal.h"
#include <stdint.h>

class UltrasonicSonar {
public:
    static void init();
    static void update();
    static float getDistance(); // Returns distance in cm

#ifndef ARDUINO
    // Mock methods for desktop TDD & simulation
    static void setMockDistance(float dist_cm);
#endif

private:
    static float currentDistance;
    static uint32_t lastSonarPingMs;
};

#endif // ULTRASONIC_SONAR_H
