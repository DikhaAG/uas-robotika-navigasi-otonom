#include <cassert>
#include <iostream>
#include <cmath>
#include "../firmware/RobotNavigasiOtonom/Config.h"
#include "../firmware/RobotNavigasiOtonom/Hal.h"
#include "../firmware/RobotNavigasiOtonom/Motors.h"
#include "../firmware/RobotNavigasiOtonom/Odometry.h"
#include "../firmware/RobotNavigasiOtonom/Sensors.h"
#include "../firmware/RobotNavigasiOtonom/Waypoints.h"

static bool approxEqual(float a, float b, float tol = 0.05f) {
    return std::fabs(a - b) <= tol;
}

int main() {
    std::cout << "[TDD] Running test_waypoints..." << std::endl;

    // Test 1: Waypoint queue initialization and room loading
    Waypoints::init();
    Waypoints::loadRoom(1);
    assert(Waypoints::getTotalWaypoints() == 2);
    assert(Waypoints::getCurrentIndex() == 0);
    assert(!Waypoints::isFinalWaypoint());
    
    Waypoint wp = Waypoints::getCurrentWaypoint();
    assert(approxEqual(wp.x, 180.0f) && approxEqual(wp.y, 0.0f));
    std::cout << "  ✓ Test 1: Room 1 WP1 loaded (180, 0)." << std::endl;

    // Test 2: Advancing to final waypoint
    Waypoints::advanceWaypoint();
    assert(Waypoints::getCurrentIndex() == 1);
    assert(Waypoints::isFinalWaypoint());
    wp = Waypoints::getCurrentWaypoint();
    assert(approxEqual(wp.x, 180.0f) && approxEqual(wp.y, 70.0f));
    std::cout << "  ✓ Test 2: Room 1 WP2 loaded (180, 70)." << std::endl;

    // Test 3: Temporary waypoint injection
    // Queue is at index 1 (WP2 at 180, 70). Inject temp waypoint (100, 20)
    bool injected = Waypoints::injectWaypointFront(100.0f, 20.0f);
    assert(injected);
    assert(Waypoints::getCurrentIndex() == 1);
    assert(!Waypoints::isFinalWaypoint()); // Now intermediate!
    wp = Waypoints::getCurrentWaypoint();
    assert(approxEqual(wp.x, 100.0f) && approxEqual(wp.y, 20.0f));
    std::cout << "  ✓ Test 3: Temporary waypoint injected." << std::endl;

    // Test 4: Distance error and angular error calculation
    Odometry::resetPose(0.0f, 0.0f, 0.0f);
    Waypoints::loadRoom(3); // WP1: (235, 0), WP2: (235, -70)
    float rho = 0.0f, e = 0.0f;
    Waypoints::computeErrors(Odometry::getPose(), rho, e);
    assert(approxEqual(rho, 235.0f, 0.1f));
    assert(approxEqual(e, 0.0f, 0.1f));
    std::cout << "  ✓ Test 4: Error calculation along corridor axis passed." << std::endl;

    // Test 5: In-place turn command when |e| > 25 degrees
    Odometry::resetPose(0.0f, 0.0f, 0.0f);
    // Temporary target directly to the left (0, 50) -> target angle = 90 deg, e = 90 deg
    Waypoints::clearQueue();
    Waypoints::injectWaypointFront(0.0f, 50.0f);
    int pwmL = 0, pwmR = 0;
    bool isTurning = Waypoints::computeMotorCommands(Odometry::getPose(), pwmL, pwmR);
    assert(isTurning == true);
    assert(pwmL == -PWM_TURN);
    assert(pwmR == +PWM_TURN);
    std::cout << "  ✓ Test 5: In-place turn command triggered for large angle error." << std::endl;

    // Test 6: Proportional forward drive and deceleration
    Waypoints::clearQueue();
    Waypoints::injectWaypointFront(20.0f, 0.0f); // 20 cm ahead (< 30 cm decel limit)
    Odometry::resetPose(0.0f, 0.0f, 0.0f);
    isTurning = Waypoints::computeMotorCommands(Odometry::getPose(), pwmL, pwmR);
    assert(isTurning == false);
    // Base speed should decelerate to PWM_SLOW (120) with e = 0
    assert(pwmL == PWM_SLOW);
    assert(pwmR == PWM_SLOW);
    std::cout << "  ✓ Test 6: Deceleration into waypoint passed (PWM = " << pwmL << ")." << std::endl;

    // Test 7: Intermediate waypoint achieved condition
    Odometry::resetPose(19.0f, 0.0f, 0.0f); // 1 cm to target (20, 0)
    bool achieved = Waypoints::checkWaypointAchieved(Odometry::getPose());
    assert(achieved == true);
    std::cout << "  ✓ Test 7: checkWaypointAchieved() passed." << std::endl;

    std::cout << "[TDD] ALL test_waypoints passed successfully!" << std::endl;
    return 0;
}
