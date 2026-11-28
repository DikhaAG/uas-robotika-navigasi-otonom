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

    // Software polarity inversion (+1 normal, -1 inverted)
    static void setInversion(int8_t invLeft, int8_t invRight);
    static int8_t getInvertLeft();
    static int8_t getInvertRight();

    // Motor balance trim multipliers
    static void setTrim(float left, float right);
    static float getTrimLeft();
    static float getTrimRight();

    // Battery voltage compensation: every command is multiplied by
    // pwmScaleForBattery(volts) so wheel speed does not depend on the pack fitted.
    static float pwmScaleForBattery(float batteryVolts);
    static void setBatteryVoltage(float batteryVolts);
    static float getBatteryPwmScale();

private:
    static int arahKiri;
    static int arahKanan;
    static int8_t invertLeft;
    static int8_t invertRight;
    static float trimLeft;
    static float trimRight;
    static float batteryPwmScale;
    static int clampPwm(int pwm);
};

#endif // MOTORS_H
