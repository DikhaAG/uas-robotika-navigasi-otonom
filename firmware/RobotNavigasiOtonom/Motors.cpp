#include "Motors.h"
#include <stdlib.h>

int Motors::arahKiri = 0;
int Motors::arahKanan = 0;

int Motors::clampPwm(int pwm) {
    if (pwm > 255) return 255;
    if (pwm < -255) return -255;
    return pwm;
}

void Motors::init() {
    Hal::pinMode(PIN_ENA, OUTPUT);
    Hal::pinMode(PIN_IN1, OUTPUT);
    Hal::pinMode(PIN_IN2, OUTPUT);
    Hal::pinMode(PIN_IN3, OUTPUT);
    Hal::pinMode(PIN_IN4, OUTPUT);
    Hal::pinMode(PIN_ENB, OUTPUT);
    stopMotors();
}

void Motors::setMotor(int pwmLeft, int pwmRight) {
    pwmLeft = clampPwm(pwmLeft);
    pwmRight = clampPwm(pwmRight);

    // --- Left Motor Control ---
    if (pwmLeft > 0) {
        Hal::digitalWrite(PIN_IN1, HIGH);
        Hal::digitalWrite(PIN_IN2, LOW);
        Hal::setPwm(PIN_ENA, (uint8_t)pwmLeft);
        arahKiri = 1;
    } else if (pwmLeft < 0) {
        Hal::digitalWrite(PIN_IN1, LOW);
        Hal::digitalWrite(PIN_IN2, HIGH);
        Hal::setPwm(PIN_ENA, (uint8_t)(-pwmLeft));
        arahKiri = -1;
    } else {
        Hal::digitalWrite(PIN_IN1, LOW);
        Hal::digitalWrite(PIN_IN2, LOW);
        Hal::setPwm(PIN_ENA, 0);
        arahKiri = 0;
    }

    // --- Right Motor Control ---
    if (pwmRight > 0) {
        Hal::digitalWrite(PIN_IN3, HIGH);
        Hal::digitalWrite(PIN_IN4, LOW);
        Hal::setPwm(PIN_ENB, (uint8_t)pwmRight);
        arahKanan = 1;
    } else if (pwmRight < 0) {
        Hal::digitalWrite(PIN_IN3, LOW);
        Hal::digitalWrite(PIN_IN4, HIGH);
        Hal::setPwm(PIN_ENB, (uint8_t)(-pwmRight));
        arahKanan = -1;
    } else {
        Hal::digitalWrite(PIN_IN3, LOW);
        Hal::digitalWrite(PIN_IN4, LOW);
        Hal::setPwm(PIN_ENB, 0);
        arahKanan = 0;
    }
}

void Motors::stopMotors() {
    Hal::setPwm(PIN_ENA, 0);
    Hal::setPwm(PIN_ENB, 0);
    Hal::digitalWrite(PIN_IN1, LOW);
    Hal::digitalWrite(PIN_IN2, LOW);
    Hal::digitalWrite(PIN_IN3, LOW);
    Hal::digitalWrite(PIN_IN4, LOW);
    arahKiri = 0;
    arahKanan = 0;
}

int Motors::getDirectionLeft() {
    return arahKiri;
}

int Motors::getDirectionRight() {
    return arahKanan;
}
