#include <cassert>
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
    std::cout << "  ✓ Test 1: Motors::init() passed." << std::endl;

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

    std::cout << "[TDD] ALL test_motors passed successfully!" << std::endl;
    return 0;
}
