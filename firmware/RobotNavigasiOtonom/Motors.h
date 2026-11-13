#ifndef MOTORS_H
#define MOTORS_H

#include "Config.h"
#include "Hal.h"
#include <stdint.h>

class Motors {
public:
    static void init();
    static void setMotor(int pwmLeft, int pwmRight);
    static void stopMotors();

    // Direction tracking for single-channel encoders
    // Returns: +1 for forward, -1 for reverse, 0 for stopped
    static int getDirectionLeft();
    static int getDirectionRight();

private:
    static int arahKiri;
    static int arahKanan;
    static int clampPwm(int pwm);
};

#endif // MOTORS_H
