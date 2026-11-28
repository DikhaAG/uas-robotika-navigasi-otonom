#include <cassert>
#include <cmath>
#include <iostream>
#include "../firmware/RobotNavigasiOtonom/Config.h"
#include "../firmware/RobotNavigasiOtonom/Hal.h"
#include "../firmware/RobotNavigasiOtonom/Motors.h"

int main() {
    std::cout << "[TDD] Running test_motors..." << std::endl;

    // Test 1: Initialization
    Motors::init();
    assert(Motors::getDirectionLeft() == 0);
    assert(Motors::getDirectionRight() == 0);
    assert(Hal::getPwm(PIN_ENA) == 0);
    assert(Hal::getPwm(PIN_ENB) == 0);
    assert(Hal::getPin(PIN_IN1) == 0);
    assert(Hal::getPin(PIN_IN2) == 0);
    assert(Hal::getPin(PIN_IN3) == 0);
    assert(Hal::getPin(PIN_IN4) == 0);
    // init() compensates for the battery pack configured in Config.h
    assert(std::fabs(Motors::getBatteryPwmScale() - Motors::pwmScaleForBattery(TEGANGAN_BATERAI_V)) < 1e-6f);
    std::cout << "  ✓ Test 1: Motors::init() passed." << std::endl;

    // Tests 2-7 use the design pack, on which PWM passes through unscaled
    Motors::setBatteryVoltage(TEGANGAN_DESAIN_V);
    assert(std::fabs(Motors::getBatteryPwmScale() - 1.0f) < 1e-6f);

    // Test 2: Forward drive (positive PWM)
    Motors::setMotor(170, 170);
    assert(Motors::getDirectionLeft() == 1);
    assert(Motors::getDirectionRight() == 1);
    assert(Hal::getPin(PIN_IN1) == 1);
    assert(Hal::getPin(PIN_IN2) == 0);
    assert(Hal::getPin(PIN_IN3) == 1);
    assert(Hal::getPin(PIN_IN4) == 0);
    assert(Hal::getPwm(PIN_ENA) == 170);
    assert(Hal::getPwm(PIN_ENB) == 170);
    std::cout << "  ✓ Test 2: Forward drive (positive PWM) passed." << std::endl;

    // Test 3: Reverse drive (negative PWM)
    Motors::setMotor(-150, -150);
    assert(Motors::getDirectionLeft() == -1);
    assert(Motors::getDirectionRight() == -1);
    assert(Hal::getPin(PIN_IN1) == 0);
    assert(Hal::getPin(PIN_IN2) == 1);
    assert(Hal::getPin(PIN_IN3) == 0);
    assert(Hal::getPin(PIN_IN4) == 1);
    assert(Hal::getPwm(PIN_ENA) == 150);
    assert(Hal::getPwm(PIN_ENB) == 150);
    std::cout << "  ✓ Test 3: Reverse drive (negative PWM) passed." << std::endl;

    // Test 4: In-place turn (left reverse, right forward)
    Motors::setMotor(-140, 140);
    assert(Motors::getDirectionLeft() == -1);
    assert(Motors::getDirectionRight() == 1);
    assert(Hal::getPin(PIN_IN1) == 0);
    assert(Hal::getPin(PIN_IN2) == 1);
    assert(Hal::getPin(PIN_IN3) == 1);
    assert(Hal::getPin(PIN_IN4) == 0);
    assert(Hal::getPwm(PIN_ENA) == 140);
    assert(Hal::getPwm(PIN_ENB) == 140);
    std::cout << "  ✓ Test 4: In-place turn passed." << std::endl;

    // Test 5: Input clamping (-255 to +255)
    Motors::setMotor(320, -400);
    assert(Motors::getDirectionLeft() == 1);
    assert(Motors::getDirectionRight() == -1);
    assert(Hal::getPwm(PIN_ENA) == 255);
    assert(Hal::getPwm(PIN_ENB) == 255);
    std::cout << "  ✓ Test 5: Input clamping passed." << std::endl;

    // Test 6: stopMotors()
    Motors::stopMotors();
    assert(Motors::getDirectionLeft() == 0);
    assert(Motors::getDirectionRight() == 0);
    assert(Hal::getPwm(PIN_ENA) == 0);
    assert(Hal::getPwm(PIN_ENB) == 0);
    assert(Hal::getPin(PIN_IN1) == 0);
    assert(Hal::getPin(PIN_IN2) == 0);
    assert(Hal::getPin(PIN_IN3) == 0);
    assert(Hal::getPin(PIN_IN4) == 0);
    std::cout << "  ✓ Test 6: stopMotors() passed." << std::endl;

    // Test 7: Software polarity inversion
    Motors::setInversion(-1, 1);
    assert(Motors::getInvertLeft() == -1);
    assert(Motors::getInvertRight() == 1);
    // Forward command with inverted left motor: IN1=0, IN2=1
    Motors::setMotor(170, 170);
    assert(Hal::getPin(PIN_IN1) == 0);
    assert(Hal::getPin(PIN_IN2) == 1);
    assert(Hal::getPin(PIN_IN3) == 1);
    assert(Hal::getPin(PIN_IN4) == 0);
    assert(Motors::getDirectionLeft() == 1);
    assert(Motors::getDirectionRight() == 1);
    // Revert inversion
    Motors::setInversion(1, 1);
    Motors::stopMotors();
    std::cout << "  ✓ Test 7: Software polarity inversion passed." << std::endl;

    // Test 8: Battery scale = motor voltage the PWM constants were tuned on / motor voltage now
    assert(std::fabs(Motors::pwmScaleForBattery(TEGANGAN_DESAIN_V) - 1.0f) < 1e-6f);
    float scale3S = Motors::pwmScaleForBattery(11.1f);
    assert(std::fabs(scale3S - (7.4f - DROP_DRIVER_V) / (11.1f - DROP_DRIVER_V)) < 1e-6f);
    assert(scale3S < 1.0f);
    assert(Motors::pwmScaleForBattery(6.0f) > 1.0f);              // weaker pack: boost PWM
    assert(Motors::pwmScaleForBattery(DROP_DRIVER_V) == 1.0f);    // nonsense voltage: no compensation
    assert(Motors::pwmScaleForBattery(0.0f) == 1.0f);
    std::cout << "  ✓ Test 8: pwmScaleForBattery() passed." << std::endl;

    // Test 9: A 3S pack scales every command (forward, turn) so wheel speed matches the 2S design
    Motors::setBatteryVoltage(11.1f);
    Motors::setMotor(170, 170);
    assert(Hal::getPwm(PIN_ENA) == (int)(170.0f * scale3S));
    assert(Hal::getPwm(PIN_ENB) == (int)(170.0f * scale3S));
    assert(Hal::getPwm(PIN_ENA) < 170);
    Motors::setMotor(-140, 140);
    assert(Motors::getDirectionLeft() == -1);
    assert(Motors::getDirectionRight() == 1);
    assert(Hal::getPwm(PIN_ENA) == (int)(140.0f * scale3S));
    assert(Hal::getPwm(PIN_ENB) == (int)(140.0f * scale3S));
    std::cout << "  ✓ Test 9: 3S battery compensation passed." << std::endl;

    // Test 10: Compensation composes with motor trim, and boosted PWM still clamps at 255
    Motors::setTrim(1.1f, 1.0f);
    Motors::setMotor(170, 170);
    assert(Hal::getPwm(PIN_ENA) == (int)(170.0f * (1.1f * scale3S)));
    assert(Hal::getPwm(PIN_ENB) == (int)(170.0f * scale3S));
    Motors::setTrim(1.0f, 1.0f);
    Motors::setBatteryVoltage(6.0f);
    Motors::setMotor(250, -250);
    assert(Hal::getPwm(PIN_ENA) == 255);
    assert(Hal::getPwm(PIN_ENB) == 255);
    Motors::setBatteryVoltage(TEGANGAN_DESAIN_V);
    Motors::stopMotors();
    std::cout << "  ✓ Test 10: Compensation with trim and clamping passed." << std::endl;

    std::cout << "[TDD] ALL test_motors passed successfully!" << std::endl;
    return 0;
}
