#include <cassert>
#include <iostream>
#include <cmath>
#include "../firmware/RobotNavigasiOtonom/Config.h"
#include "../firmware/RobotNavigasiOtonom/Hal.h"
#include "../firmware/RobotNavigasiOtonom/Motors.h"
#include "../firmware/RobotNavigasiOtonom/Odometry.h"
#include "../firmware/RobotNavigasiOtonom/Sensors.h"

static bool approxEqual(float a, float b, float tol = 0.05f) {
    return std::fabs(a - b) <= tol;
}

int main() {
    std::cout << "[TDD] Running test_odometry..." << std::endl;

    // Test 1: Angle normalization
    assert(approxEqual(Odometry::normalizeAngleDeg(0.0f), 0.0f));
    assert(approxEqual(Odometry::normalizeAngleDeg(180.0f), 180.0f));
    assert(approxEqual(Odometry::normalizeAngleDeg(-180.0f), -180.0f));
    assert(approxEqual(Odometry::normalizeAngleDeg(190.0f), -170.0f));
    assert(approxEqual(Odometry::normalizeAngleDeg(-195.0f), 165.0f));
    assert(approxEqual(Odometry::normalizeAngleDeg(360.0f), 0.0f));
    assert(approxEqual(Odometry::normalizeAngleDeg(-360.0f), 0.0f));
    assert(approxEqual(Odometry::normalizeAngleDeg(720.0f), 0.0f));
    std::cout << "  ✓ Test 1: normalizeAngleDeg() passed." << std::endl;

    // Test 2: Initial pose reset
    Odometry::init();
    Odometry::resetPose(0.0f, 0.0f, 0.0f);
    assert(approxEqual(Odometry::getX(), 0.0f));
    assert(approxEqual(Odometry::getY(), 0.0f));
    assert(approxEqual(Odometry::getTheta(), 0.0f));
    std::cout << "  ✓ Test 2: resetPose() passed." << std::endl;

    // Test 3: Straight line forward motion (Midpoint odometry integration)
    // 20 pulses on 20-slot encoder with 65mm diameter wheel is 1 full rotation = 20.420 cm
    Motors::setMotor(170, 170); // Sets forward direction
    Odometry::injectPulses(20, 20); // Simulates 20 pulses left and 20 pulses right
    Odometry::update(0.02f); // 20 ms integration step, 0 deg/s gyro
    assert(approxEqual(Odometry::getX(), 20.42f, 0.1f));
    assert(approxEqual(Odometry::getY(), 0.0f, 0.05f));
    assert(approxEqual(Odometry::getTheta(), 0.0f, 0.05f));
    std::cout << "  ✓ Test 3: Straight line forward odometry passed (x = " << Odometry::getX() << " cm)." << std::endl;

    // Test 4: Reverse motion (direction tracking)
    Motors::setMotor(-170, -170); // Sets reverse direction
    Odometry::injectPulses(10, 10); // 10 pulses in reverse
    Odometry::update(0.02f);
    // x was ~20.42, reversing 10.21cm should yield ~10.21cm
    assert(approxEqual(Odometry::getX(), 10.21f, 0.15f));
    assert(approxEqual(Odometry::getY(), 0.0f, 0.05f));
    std::cout << "  ✓ Test 4: Reverse motion odometry passed (x = " << Odometry::getX() << " cm)." << std::endl;

    // Test 5: Gyroscope angular rate integration & heading update
    Odometry::resetPose(0.0f, 0.0f, 0.0f);
    // Turn in place at 45 deg/s for 1 second (50 steps of 20 ms)
    for (int i = 0; i < 50; ++i) {
        Sensors::setMockGyroZ(45.0f);
        Odometry::update(0.02f);
    }
    assert(approxEqual(Odometry::getTheta(), 45.0f, 0.1f));
    std::cout << "  ✓ Test 5: Gyro angular rate integration passed (theta = " << Odometry::getTheta() << " deg)." << std::endl;

    // Test 6: Diagonal displacement at 45 degrees
    Motors::setMotor(170, 170);
    // 20 pulses = 20.42 cm along 45 deg line:
    // dx = 20.42 * cos(45) = 14.44 cm, dy = 20.42 * sin(45) = 14.44 cm
    Odometry::injectPulses(20, 20);
    Sensors::setMockGyroZ(0.0f);
    Odometry::update(0.02f);
    assert(approxEqual(Odometry::getX(), 14.44f, 0.2f));
    assert(approxEqual(Odometry::getY(), 14.44f, 0.2f));
    std::cout << "  ✓ Test 6: Diagonal 2D odometry passed (x = " << Odometry::getX() << ", y = " << Odometry::getY() << ")." << std::endl;

    // Test 7: Sensor mock readings (Ultrasonic and IR)
    Sensors::init();
    Sensors::setMockDistance(23.5f);
    Sensors::setMockIr(true, false); // Left wall detected, right clear
    assert(approxEqual(Sensors::getDistance(), 23.5f, 0.1f));
    assert(Sensors::isWallLeft() == true);
    assert(Sensors::isWallRight() == false);
    std::cout << "  ✓ Test 7: Sensor mock acquisitions passed." << std::endl;

    std::cout << "[TDD] ALL test_odometry passed successfully!" << std::endl;
    return 0;
}
