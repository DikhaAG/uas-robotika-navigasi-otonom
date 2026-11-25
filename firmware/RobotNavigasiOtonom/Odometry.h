#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Config.h"
#include "Hal.h"
#include "Motors.h"
#include <stdint.h>

struct Pose {
    float x;     // cm
    float y;     // cm
    float theta; // degrees, [-180, +180]
};

class Odometry {
public:
    static void init();
    static void resetPose(float x = 0.0f, float y = 0.0f, float theta = 0.0f);
    static void update(float dt_sec);

    static float getX();
    static float getY();
    static float getTheta();
    static Pose getPose();
    static float getDistanceMoved();
    static long getTotalPulseCount();
    static long getPulsaKiri();
    static long getPulsaKanan();
    static void resetPulses();

    static void setWheelDiameterMm(float diameterMm);
    static float getWheelDiameterMm();
    static float getCmPerPulse();

    static float normalizeAngleDeg(float deg);

    // Interrupt handlers
    static void isrKiri();
    static void isrKanan();

    // Testing helpers
    static void injectPulses(long left, long right);

private:
    static Pose currentPose;
    static float totalDistanceMoved;
    static float wheelDiameterMm;
    static float cmPerPulse;

    static volatile long pulsaKiri;
    static volatile long pulsaKanan;
    static volatile uint32_t tIsrKiri;
    static volatile uint32_t tIsrKanan;

    static long lastPulsaKiri;
    static long lastPulsaKanan;
};

#endif // ODOMETRY_H
