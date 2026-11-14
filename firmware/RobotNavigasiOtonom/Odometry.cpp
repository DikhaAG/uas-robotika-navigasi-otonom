#include "Odometry.h"
#include "Sensors.h"
#include <math.h>

Pose Odometry::currentPose = {0.0f, 0.0f, 0.0f};
float Odometry::totalDistanceMoved = 0.0f;

volatile long Odometry::pulsaKiri = 0;
volatile long Odometry::pulsaKanan = 0;
volatile uint32_t Odometry::tIsrKiri = 0;
volatile uint32_t Odometry::tIsrKanan = 0;

long Odometry::lastPulsaKiri = 0;
long Odometry::lastPulsaKanan = 0;

void Odometry::init() {
    Hal::pinMode(PIN_ENC_KIRI, INPUT);
    Hal::pinMode(PIN_ENC_KANAN, INPUT);

#ifdef ARDUINO
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_KIRI), Odometry::isrKiri, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_KANAN), Odometry::isrKanan, RISING);
#endif

    resetPose(0.0f, 0.0f, 0.0f);
}

void Odometry::resetPose(float x, float y, float theta) {
    Hal::noInterrupts();
    pulsaKiri = 0;
    pulsaKanan = 0;
    tIsrKiri = 0;
    tIsrKanan = 0;
    Hal::interrupts();

    lastPulsaKiri = 0;
    lastPulsaKanan = 0;
    currentPose.x = x;
    currentPose.y = y;
    currentPose.theta = normalizeAngleDeg(theta);
    totalDistanceMoved = 0.0f;
}

float Odometry::normalizeAngleDeg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

void Odometry::isrKiri() {
    uint32_t now = Hal::micros();
    if (now - tIsrKiri >= 1000UL) { // 1 ms debounce threshold
        pulsaKiri++;
        tIsrKiri = now;
    }
}

void Odometry::isrKanan() {
    uint32_t now = Hal::micros();
    if (now - tIsrKanan >= 1000UL) {
        pulsaKanan++;
        tIsrKanan = now;
    }
}

void Odometry::injectPulses(long left, long right) {
    Hal::noInterrupts();
    pulsaKiri += left;
    pulsaKanan += right;
    Hal::interrupts();
}

void Odometry::update(float dt_sec) {
    Hal::noInterrupts();
    long curKiri = pulsaKiri;
    long curKanan = pulsaKanan;
    Hal::interrupts();

    long dPulsaKiri = curKiri - lastPulsaKiri;
    long dPulsaKanan = curKanan - lastPulsaKanan;
    lastPulsaKiri = curKiri;
    lastPulsaKanan = curKanan;

    int dirL = Motors::getDirectionLeft();
    int dirR = Motors::getDirectionRight();

    float dL = (float)dPulsaKiri * CM_PER_PULSA * (float)(dirL != 0 ? dirL : 1);
    float dR = (float)dPulsaKanan * CM_PER_PULSA * (float)(dirR != 0 ? dirR : 1);

    float d = (dL + dR) / 2.0f;
    totalDistanceMoved += (d >= 0.0f ? d : -d);

    float gyroRateZ = Sensors::readGyroZRate();
    float dTheta = gyroRateZ * dt_sec;

    // Midpoint heading formulation (Spec 8)
    float midTheta = currentPose.theta + (dTheta / 2.0f);
    float midRad = midTheta * (PI / 180.0f);

    currentPose.x += d * cosf(midRad);
    currentPose.y += d * sinf(midRad);
    currentPose.theta = normalizeAngleDeg(currentPose.theta + dTheta);
}

float Odometry::getX() {
    return currentPose.x;
}

float Odometry::getY() {
    return currentPose.y;
}

float Odometry::getTheta() {
    return currentPose.theta;
}

Pose Odometry::getPose() {
    return currentPose;
}

float Odometry::getDistanceMoved() {
    return totalDistanceMoved;
}

long Odometry::getTotalPulseCount() {
    Hal::noInterrupts();
    long total = pulsaKiri + pulsaKanan;
    Hal::interrupts();
    return total;
}

long Odometry::getPulsaKiri() {
    Hal::noInterrupts();
    long p = pulsaKiri;
    Hal::interrupts();
    return p;
}

long Odometry::getPulsaKanan() {
    Hal::noInterrupts();
    long p = pulsaKanan;
    Hal::interrupts();
    return p;
}

void Odometry::resetPulses() {
    Hal::noInterrupts();
    pulsaKiri = 0;
    pulsaKanan = 0;
    Hal::interrupts();
    lastPulsaKiri = 0;
    lastPulsaKanan = 0;
}
