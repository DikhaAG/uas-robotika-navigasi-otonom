#include <cassert>
#include <iostream>
#include <cmath>
#include "../firmware/RobotNavigasiOtonom/Config.h"
#include "../firmware/RobotNavigasiOtonom/Hal.h"
#include "../firmware/RobotNavigasiOtonom/Motors.h"
#include "../firmware/RobotNavigasiOtonom/Odometry.h"
#include "../firmware/RobotNavigasiOtonom/Sensors.h"
#include "../firmware/RobotNavigasiOtonom/WaypointQueue.h"
#include "../firmware/RobotNavigasiOtonom/ObstacleAvoidance.h"

static bool approxEqual(float a, float b, float tol = 0.05f) {
    return std::fabs(a - b) <= tol;
}

int main() {
    std::cout << "[TDD] Running test_obstacle_avoidance..." << std::endl;

    // Test 1: Obstacle trigger conditions
    Odometry::resetPose(50.0f, 0.0f, 5.0f); // Facing along corridor (|theta| < 20)
    WaypointQueue::loadRoom(1); // Room 1: WP1(180, 0), WP2(180, 70)
    Sensors::setMockDistance(20.0f); // Obstacle < 25 cm
    assert(ObstacleAvoidance::shouldTrigger(Odometry::getPose(), Sensors::getDistance(), WaypointQueue::isFinal()) == true);
    std::cout << "  ✓ Test 1a: Valid obstacle triggers avoidance." << std::endl;

    // Angled robot (|theta| > 20 deg) looking at corridor side wall
    Odometry::resetPose(50.0f, 0.0f, 25.0f);
    assert(ObstacleAvoidance::shouldTrigger(Odometry::getPose(), Sensors::getDistance(), WaypointQueue::isFinal()) == false);
    std::cout << "  ✓ Test 1b: Angled view (|theta| > 20) suppresses trigger." << std::endl;

    // Inside final room entry segment
    WaypointQueue::advance(); // Now on WP2 (final room entry)
    Odometry::resetPose(180.0f, 50.0f, 90.0f);
    assert(ObstacleAvoidance::shouldTrigger(Odometry::getPose(), Sensors::getDistance(), WaypointQueue::isFinal()) == false);
    std::cout << "  ✓ Test 1c: Final room entry segment suppresses trigger." << std::endl;

    // Test 2: Clearance lane decision
    float laneY = ObstacleAvoidance::decideBypassLane(45.0f, 20.0f); // Left clearance > right
    assert(approxEqual(laneY, +20.0f));
    laneY = ObstacleAvoidance::decideBypassLane(15.0f, 50.0f); // Right clearance > left
    assert(approxEqual(laneY, -20.0f));
    std::cout << "  ✓ Test 2: Bypass lane decision passed (+20 cm / -20 cm)." << std::endl;

    // Test 3: Both sides blocked (< 30 cm)
    bool entrapped = ObstacleAvoidance::isBothBlocked(22.0f, 24.0f);
    assert(entrapped == true);
    assert(ObstacleAvoidance::isBothBlocked(35.0f, 20.0f) == false);
    std::cout << "  ✓ Test 3: Both sides blocked detection passed." << std::endl;

    // Test 4: Injection of 3 temporary bypass waypoints
    WaypointQueue::loadRoom(1); // WP1: (180, 0), WP2: (180, 70)
    Odometry::resetPose(50.0f, 0.0f, 0.0f);
    float measuredDist = 20.0f;
    bool ok = ObstacleAvoidance::injectBypassWaypoints(Odometry::getPose(), measuredDist, +20.0f);
    assert(ok == true);
    assert(WaypointQueue::getCount() == 5); // 3 temp + 2 room
    assert(WaypointQueue::getCurrentIndex() == 0);

    // WP_A: x + 15 = 65, y = +20
    Waypoint wpA = WaypointQueue::getCurrentWaypoint();
    assert(approxEqual(wpA.x, 65.0f) && approxEqual(wpA.y, 20.0f));

    // WP_B: x + d + 45 = 50 + 20 + 45 = 115, y = +20
    WaypointQueue::advance();
    Waypoint wpB = WaypointQueue::getCurrentWaypoint();
    assert(approxEqual(wpB.x, 115.0f) && approxEqual(wpB.y, 20.0f));

    // WP_C: x + d + 65 = 50 + 20 + 65 = 135, y = 0
    WaypointQueue::advance();
    Waypoint wpC = WaypointQueue::getCurrentWaypoint();
    assert(approxEqual(wpC.x, 135.0f) && approxEqual(wpC.y, 0.0f));

    // WP1: 180, 0
    WaypointQueue::advance();
    Waypoint wp1 = WaypointQueue::getCurrentWaypoint();
    assert(approxEqual(wp1.x, 180.0f) && approxEqual(wp1.y, 0.0f));

    // WP2: 180, 70
    WaypointQueue::advance();
    Waypoint wp2 = WaypointQueue::getCurrentWaypoint();
    assert(approxEqual(wp2.x, 180.0f) && approxEqual(wp2.y, 70.0f));
    assert(WaypointQueue::isFinal());
    std::cout << "  ✓ Test 4: Three-point detour sequence verified." << std::endl;

    std::cout << "[TDD] ALL test_obstacle_avoidance passed successfully!" << std::endl;
    return 0;
}
