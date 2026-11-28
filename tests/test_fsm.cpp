#include <cassert>
#include <iostream>
#include <cmath>
#include <cstring>
#include "test_common.h"

static int countOccurrences(const char* text, const char* needle) {
    int count = 0;
    for (const char* p = strstr(text, needle); p != NULL; p = strstr(p + 1, needle)) {
        ++count;
    }
    return count;
}

int main() {
    std::cout << "[TDD] Running test_fsm..." << std::endl;

    // Test 1: Initialization & IDLE state
    resetAllMocks();
    assert(NavigationFSM::getState() == STATE_IDLE);
    assert(NavigationFSM::getTargetRoom() == 1); // Default Room 1
    std::cout << "  ✓ Test 1: FSM initialized in STATE_IDLE." << std::endl;

    // Test 2: Room selection via command
    resetAllMocks();
    NavigationFSM::selectRoom(3);
    assert(NavigationFSM::getTargetRoom() == 3);
    std::cout << "  ✓ Test 2: Room selection updated to Room 3." << std::endl;

    // Test 3: Transition IDLE -> KALIBRASI -> ORIENTASI
    resetAllMocks();
    NavigationFSM::triggerStart();
    assert(NavigationFSM::getState() == STATE_KALIBRASI);
    // Simulate 2000 ms calibration completion
    Hal::advanceTimeMs(2100);
    NavigationFSM::update();
    assert(NavigationFSM::getState() == STATE_ORIENTASI);
    std::cout << "  ✓ Test 3: Transition IDLE -> KALIBRASI -> ORIENTASI passed." << std::endl;

    // Test 4: ORIENTASI -> MAJU (Robot facing 0 deg, WP1 at 235, 0)
    resetAllMocks();
    NavigationFSM::triggerStart();
    Hal::advanceTimeMs(2100);
    NavigationFSM::update(); // Now in ORIENTASI
    NavigationFSM::update(); // Now in MAJU
    assert(NavigationFSM::getState() == STATE_MAJU);
    std::cout << "  ✓ Test 4: ORIENTASI -> MAJU passed." << std::endl;

    // Test 5: Obstacle avoidance trigger MAJU -> PINDAI
    resetAllMocks();
    NavigationFSM::triggerStart();
    Hal::advanceTimeMs(2100);
    NavigationFSM::update(); // ORIENTASI
    NavigationFSM::update(); // MAJU
    UltrasonicSonar::setMockDistance(20.0f); // Obstacle ahead
    NavigationFSM::update();
    assert(NavigationFSM::getState() == STATE_PINDAI);
    std::cout << "  ✓ Test 5: MAJU -> PINDAI triggered on obstacle < 25 cm." << std::endl;

    // Test 6: Final arrival at Room WP2 -> STATE_TIBA
    resetAllMocks();
    WaypointQueue::loadRoom(1);
    WaypointQueue::advance(); // On WP2 (180, 70)
    Odometry::resetPose(180.0f, 69.0f, 90.0f); // rho = 1.0 cm <= 2.0 cm
    UltrasonicSonar::setMockDistance(15.0f); // Back wall confirmed
    NavigationFSM::setState(STATE_MAJU);
    NavigationFSM::update();
    assert(NavigationFSM::getState() == STATE_TIBA);
    assert(Motors::getDirectionLeft() == 0 && Motors::getDirectionRight() == 0);
    std::cout << "  ✓ Test 6: Final room arrival -> STATE_TIBA verified." << std::endl;

    // Test 7: Failsafe 1 - Global Timeout (> 90s)
    resetAllMocks();
    NavigationFSM::selectRoom(1);
    NavigationFSM::triggerStart();
    Hal::advanceTimeMs(2100);
    NavigationFSM::update(); // Now in ORIENTASI / MAJU
    // Advance time past 90 seconds (91,000 ms)
    Hal::advanceTimeMs(91000);
    NavigationFSM::update();
    assert(NavigationFSM::getState() == STATE_GAGAL);
    assert(NavigationFSM::getLastFailsafe() == FAILSAFE_TIMEOUT);
    std::cout << "  ✓ Test 7: Failsafe 1 (Global Timeout > 90s) triggered STATE_GAGAL." << std::endl;

    // Test 8: Failsafe 2 - Stall Detection (active PWM with 0 pulses for >= 1s)
    resetAllMocks();
    NavigationFSM::selectRoom(1);
    NavigationFSM::triggerStart();
    Hal::advanceTimeMs(2100);
    NavigationFSM::update(); // Transitions KALIBRASI -> ORIENTASI
    NavigationFSM::update(); // Transitions ORIENTASI -> MAJU
    assert(NavigationFSM::getState() == STATE_MAJU);
    NavigationFSM::update(); // Executes MAJU cycle, commanding active motor drive
    // Motors running (PWM > 0) but zero pulses injected for 1100 ms
    Hal::advanceTimeMs(1100);
    NavigationFSM::update();
    assert(NavigationFSM::getState() == STATE_GAGAL);
    assert(NavigationFSM::getLastFailsafe() == FAILSAFE_STALL);
    std::cout << "  ✓ Test 8: Failsafe 2 (Motor Stall >= 1.0s) triggered STATE_GAGAL." << std::endl;

    // Test 9: Failsafe 3 - Boundary Excursion (|y| > 40 cm outside room entry)
    resetAllMocks();
    NavigationFSM::selectRoom(1);
    NavigationFSM::triggerStart();
    Hal::advanceTimeMs(2100);
    NavigationFSM::update(); // Transitions KALIBRASI -> ORIENTASI
    NavigationFSM::update(); // Transitions ORIENTASI -> MAJU
    assert(NavigationFSM::getState() == STATE_MAJU);
    // Drifts to y = 45 cm (outside 40 cm corridor wall) while on WP1
    Odometry::resetPose(50.0f, 45.0f, 0.0f);
    NavigationFSM::update();
    assert(NavigationFSM::getState() == STATE_GAGAL);
    assert(NavigationFSM::getLastFailsafe() == FAILSAFE_BOUNDARY);
    std::cout << "  ✓ Test 9: Failsafe 3 (Boundary Excursion |y| > 40cm) triggered STATE_GAGAL." << std::endl;

    // Test 10: Failsafe 4 - Entrapment (> 3 escape retries)
    resetAllMocks();
    NavigationFSM::selectRoom(1);
    NavigationFSM::triggerStart();
    Hal::advanceTimeMs(2100);
    NavigationFSM::update();
    NavigationFSM::setState(STATE_MUNDUR);
    NavigationFSM::setEscapeRetries(4); // Exceeded 3 retries
    NavigationFSM::update();
    assert(NavigationFSM::getState() == STATE_GAGAL);
    assert(NavigationFSM::getLastFailsafe() == FAILSAFE_ENTRAPMENT);
    std::cout << "  ✓ Test 10: Failsafe 4 (Entrapment > 3 escapes) triggered STATE_GAGAL." << std::endl;

    // Test 11: Telemetry only while navigating, so the Serial Monitor stays readable in IDLE
    resetAllMocks();
    Hal::setMockPin(PIN_BUTTON_START, HIGH);        // INPUT_PULLUP, START wire not touching GND
    Hal::clearMockSerialOutput();
    for (int i = 0; i < 100; ++i) {                 // 2 s waiting in IDLE
        Hal::advanceTimeMs(20);
        NavigationFSM::update();
    }
    assert(countOccurrences(Hal::getMockSerialOutput(), ",IDLE,") == 0);

    NavigationFSM::triggerStart();                  // 1 s of KALIBRASI streams every 200 ms
    Hal::clearMockSerialOutput();
    for (int i = 0; i < 50; ++i) {
        Hal::advanceTimeMs(20);
        NavigationFSM::update();
    }
    assert(countOccurrences(Hal::getMockSerialOutput(), ",KALIBRASI,") >= 4);

    resetAllMocks();                                // arrival: exactly one final-pose row
    WaypointQueue::loadRoom(1);
    WaypointQueue::advance();
    Odometry::resetPose(180.0f, 69.0f, 90.0f);
    UltrasonicSonar::setMockDistance(15.0f);
    Hal::advanceTimeMs(1000);
    NavigationFSM::setState(STATE_MAJU);
    Hal::clearMockSerialOutput();
    for (int i = 0; i < 150; ++i) {                 // 3 s parked in TIBA
        Hal::advanceTimeMs(20);
        NavigationFSM::update();
    }
    assert(NavigationFSM::getState() == STATE_TIBA);
    assert(countOccurrences(Hal::getMockSerialOutput(), ",TIBA,") == 1);
    std::cout << "  ✓ Test 11: Telemetry silent in IDLE, streamed while active, one row on arrival." << std::endl;

    std::cout << "[TDD] ALL test_fsm passed successfully!" << std::endl;
    return 0;
}
