#include <cassert>
#include <cstdio>
#include <cmath>
#include "../firmware/RobotNavigasiOtonom/Gyroscope.h"
#include "../firmware/RobotNavigasiOtonom/UltrasonicSonar.h"
#include "../firmware/RobotNavigasiOtonom/WallDetector.h"

static bool approxEqual(float a, float b, float tol = 0.01f) {
    return std::fabs(a - b) <= tol;
}

void test_gyroscope() {
    printf("Running test_gyroscope...\n");
    Gyroscope::init();

    // Default mock is 0x68 -> connected
    assert(Gyroscope::isConnected() == true);
    assert(Gyroscope::getWhoAmI() == 0x68);

    // Test disconnected mock
    Gyroscope::setMockWhoAmI(0x00);
    assert(Gyroscope::isConnected() == false);
    assert(Gyroscope::getWhoAmI() == 0x00);
    Gyroscope::setMockWhoAmI(0x68); // restore

    // Rate reading
    assert(approxEqual(Gyroscope::readZRate(), 0.0f));
    Gyroscope::setMockZRate(35.5f);
    assert(approxEqual(Gyroscope::readZRate(), 35.5f));

    // Bias check
    assert(approxEqual(Gyroscope::getBias(), 0.0f));
    Gyroscope::resetCalibration();
    Gyroscope::finalizeCalibration();
    assert(approxEqual(Gyroscope::getBias(), 0.0f));

    // Test non-blocking bias calibration in mock environment
    Gyroscope::resetCalibration();
    Gyroscope::setMockZRate(2.4f);
    Gyroscope::sampleCalibration();
    Gyroscope::setMockZRate(2.6f);
    Gyroscope::sampleCalibration();
    Gyroscope::finalizeCalibration();
    assert(approxEqual(Gyroscope::getBias(), 2.5f, 0.05f));

    // Reset bias for subsequent tests
    Gyroscope::resetCalibration();
    Gyroscope::finalizeCalibration();
    assert(approxEqual(Gyroscope::getBias(), 0.0f));
    Gyroscope::setMockZRate(0.0f);

    printf("  [PASS] test_gyroscope\n");
}

void test_ultrasonic_sonar() {
    printf("Running test_ultrasonic_sonar...\n");
    UltrasonicSonar::init();

    // Default distance is 400 cm
    assert(approxEqual(UltrasonicSonar::getDistance(), 400.0f));

    // Mock distance injection
    UltrasonicSonar::setMockDistance(27.4f);
    assert(approxEqual(UltrasonicSonar::getDistance(), 27.4f));

    UltrasonicSonar::setMockDistance(12.0f);
    assert(approxEqual(UltrasonicSonar::getDistance(), 12.0f));

    printf("  [PASS] test_ultrasonic_sonar\n");
}

void test_wall_detector() {
    printf("Running test_wall_detector...\n");
    WallDetector::init();

    // Default mock walls: false, false
    assert(WallDetector::isWallLeft() == false);
    assert(WallDetector::isWallRight() == false);

    // Left wall detected
    WallDetector::setMockWalls(true, false);
    assert(WallDetector::isWallLeft() == true);
    assert(WallDetector::isWallRight() == false);

    // Right wall detected
    WallDetector::setMockWalls(false, true);
    assert(WallDetector::isWallLeft() == false);
    assert(WallDetector::isWallRight() == true);

    // Both walls detected (corridor)
    WallDetector::setMockWalls(true, true);
    assert(WallDetector::isWallLeft() == true);
    assert(WallDetector::isWallRight() == true);

    // Both clear
    WallDetector::setMockWalls(false, false);
    assert(WallDetector::isWallLeft() == false);
    assert(WallDetector::isWallRight() == false);

    printf("  [PASS] test_wall_detector\n");
}

int main() {
    printf("=== SENSORS PERIPHERALS UNIT TEST (ADR-0007) ===\n");
    test_gyroscope();
    test_ultrasonic_sonar();
    test_wall_detector();
    printf("ALL SENSORS PERIPHERAL TESTS PASSED!\n");
    return 0;
}
