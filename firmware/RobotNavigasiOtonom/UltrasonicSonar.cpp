#include "UltrasonicSonar.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

float UltrasonicSonar::currentDistance = 400.0f;
uint32_t UltrasonicSonar::lastSonarPingMs = 0;

void UltrasonicSonar::init() {
    Hal::pinMode(PIN_TRIG, OUTPUT);
    Hal::pinMode(PIN_ECHO, INPUT);
    Hal::digitalWrite(PIN_TRIG, LOW);
    currentDistance = 400.0f;
    lastSonarPingMs = 0;
}

void UltrasonicSonar::update() {
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
#endif
}

float UltrasonicSonar::getDistance() {
    return currentDistance;
}

#ifndef ARDUINO
void UltrasonicSonar::setMockDistance(float dist_cm) {
    currentDistance = dist_cm;
}
#endif
