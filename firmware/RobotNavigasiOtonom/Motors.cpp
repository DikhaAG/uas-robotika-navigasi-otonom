#include "Motors.h"
#include <stdlib.h>

int Motors::arahKiri = 0;
int Motors::arahKanan = 0;
int8_t Motors::invertLeft = 1;
int8_t Motors::invertRight = 1;

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

void Motors::setInversion(int8_t invLeft, int8_t invRight) {
    invertLeft = (invLeft == -1) ? -1 : 1;
    invertRight = (invRight == -1) ? -1 : 1;
}

int8_t Motors::getInvertLeft() {
    return invertLeft;
}

int8_t Motors::getInvertRight() {
    return invertRight;
}

void Motors::setMotor(int pwmLeft, int pwmRight) {
    if (pwmLeft != 0) {
        pwmLeft = (int)((float)pwmLeft * TRIM_MOTOR_KIRI);
    }
    if (pwmRight != 0) {
        pwmRight = (int)((float)pwmRight * TRIM_MOTOR_KANAN);
    }

    pwmLeft = clampPwm(pwmLeft);
    pwmRight = clampPwm(pwmRight);

    // Track logical direction (+1 forward, -1 reverse, 0 stop)
    arahKiri = (pwmLeft > 0) ? 1 : ((pwmLeft < 0) ? -1 : 0);
    arahKanan = (pwmRight > 0) ? 1 : ((pwmRight < 0) ? -1 : 0);

    // Apply software direction polarity inversion to physical H-Bridge control
    int effectivePwmLeft = pwmLeft * (int)invertLeft;
    int effectivePwmRight = pwmRight * (int)invertRight;

    // --- Left Motor Control ---
    if (effectivePwmLeft > 0) {
        Hal::digitalWrite(PIN_IN1, HIGH);
        Hal::digitalWrite(PIN_IN2, LOW);
        Hal::setPwm(PIN_ENA, (uint8_t)effectivePwmLeft);
    } else if (effectivePwmLeft < 0) {
        Hal::digitalWrite(PIN_IN1, LOW);
        Hal::digitalWrite(PIN_IN2, HIGH);
        Hal::setPwm(PIN_ENA, (uint8_t)(-effectivePwmLeft));
    } else {
        Hal::digitalWrite(PIN_IN1, LOW);
        Hal::digitalWrite(PIN_IN2, LOW);
        Hal::setPwm(PIN_ENA, 0);
    }

    // --- Right Motor Control ---
    if (effectivePwmRight > 0) {
        Hal::digitalWrite(PIN_IN3, HIGH);
        Hal::digitalWrite(PIN_IN4, LOW);
        Hal::setPwm(PIN_ENB, (uint8_t)effectivePwmRight);
    } else if (effectivePwmRight < 0) {
        Hal::digitalWrite(PIN_IN3, LOW);
        Hal::digitalWrite(PIN_IN4, HIGH);
        Hal::setPwm(PIN_ENB, (uint8_t)(-effectivePwmRight));
    } else {
        Hal::digitalWrite(PIN_IN3, LOW);
        Hal::digitalWrite(PIN_IN4, LOW);
        Hal::setPwm(PIN_ENB, 0);
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
