#include <cassert>
#include <iostream>
#include <cmath>
#include "../firmware/RobotNavigasiOtonom/Config.h"
#include "../firmware/RobotNavigasiOtonom/WaypointQueue.h"

static bool approxEqual(float a, float b, float tol = 0.05f) {
    return std::fabs(a - b) <= tol;
}

int main() {
    std::cout << "[TDD] Running test_waypoint_queue..." << std::endl;

    // Test 1: Queue initialization and empty state
    {
        WaypointQueue::init();
        assert(WaypointQueue::isEmpty());
        assert(WaypointQueue::getCount() == 0);
        assert(WaypointQueue::getCurrentIndex() == 0);
        assert(!WaypointQueue::isFinal());
        std::cout << "  ✓ Test 1: Empty queue initialized." << std::endl;
    }

    // Test 2: Bounded capacity clamping (MAX_WAYPOINTS = 8)
    {
        WaypointQueue::clear();
        for (int i = 0; i < MAX_WAYPOINTS; ++i) {
            bool ok = WaypointQueue::append((float)i * 10.0f, 0.0f);
            assert(ok);
        }
        assert(WaypointQueue::getCount() == MAX_WAYPOINTS);
        assert(!WaypointQueue::isEmpty());

        // 9th append must fail cleanly without buffer overflow
        bool overflow = WaypointQueue::append(999.0f, 999.0f);
        assert(!overflow);
        assert(WaypointQueue::getCount() == MAX_WAYPOINTS);
        std::cout << "  ✓ Test 2: Bounded static capacity (8) and overflow protection verified." << std::endl;
    }

    // Test 3: Room loading from Config.h (Rooms 1..4)
    {
        // Room 1: (180, 0), (180, 70)
        WaypointQueue::loadRoom(1);
        assert(WaypointQueue::getCount() == 2);
        assert(WaypointQueue::getCurrentIndex() == 0);
        assert(!WaypointQueue::isFinal());
        Waypoint wp = WaypointQueue::getCurrentWaypoint();
        assert(approxEqual(wp.x, 180.0f) && approxEqual(wp.y, 0.0f));

        bool advanced = WaypointQueue::advance();
        assert(advanced);
        assert(WaypointQueue::getCurrentIndex() == 1);
        assert(WaypointQueue::isFinal());
        wp = WaypointQueue::getCurrentWaypoint();
        assert(approxEqual(wp.x, 180.0f) && approxEqual(wp.y, 70.0f));

        advanced = WaypointQueue::advance();
        assert(advanced);
        assert(WaypointQueue::isEmpty());
        assert(!WaypointQueue::isFinal());

        // Room 2: (295, 0), (295, 70)
        WaypointQueue::loadRoom(2);
        assert(WaypointQueue::getCount() == 2);
        wp = WaypointQueue::getCurrentWaypoint();
        assert(approxEqual(wp.x, 295.0f) && approxEqual(wp.y, 0.0f));
        WaypointQueue::advance();
        wp = WaypointQueue::getCurrentWaypoint();
        assert(approxEqual(wp.x, 295.0f) && approxEqual(wp.y, 70.0f));

        // Room 3: (235, 0), (235, -70)
        WaypointQueue::loadRoom(3);
        wp = WaypointQueue::getCurrentWaypoint();
        assert(approxEqual(wp.x, 235.0f) && approxEqual(wp.y, 0.0f));
        WaypointQueue::advance();
        wp = WaypointQueue::getCurrentWaypoint();
        assert(approxEqual(wp.x, 235.0f) && approxEqual(wp.y, -70.0f));

        // Room 4: (355, 0), (355, -70)
        WaypointQueue::loadRoom(4);
        wp = WaypointQueue::getCurrentWaypoint();
        assert(approxEqual(wp.x, 355.0f) && approxEqual(wp.y, 0.0f));
        WaypointQueue::advance();
        wp = WaypointQueue::getCurrentWaypoint();
        assert(approxEqual(wp.x, 355.0f) && approxEqual(wp.y, -70.0f));

        // Default out-of-range room (e.g. 99) falls back to Room 1
        WaypointQueue::loadRoom(99);
        wp = WaypointQueue::getCurrentWaypoint();
        assert(approxEqual(wp.x, 180.0f) && approxEqual(wp.y, 0.0f));
        std::cout << "  ✓ Test 3: Room waypoint loading for all 4 rooms verified." << std::endl;
    }

    // Test 4: Dynamic temporary detour waypoint injection
    {
        WaypointQueue::loadRoom(1); // WP1: (180, 0), WP2: (180, 70)
        WaypointQueue::advance();    // Now pointing to WP2 (180, 70), index 1
        assert(WaypointQueue::isFinal());

        // Inject detour at front: (100, 20)
        bool injected = WaypointQueue::injectFront(100.0f, 20.0f);
        assert(injected);
        assert(WaypointQueue::getCount() == 3);
        assert(WaypointQueue::getCurrentIndex() == 1);
        assert(!WaypointQueue::isFinal()); // Now intermediate, WP2 shifted back

        Waypoint wp = WaypointQueue::getCurrentWaypoint();
        assert(approxEqual(wp.x, 100.0f) && approxEqual(wp.y, 20.0f));

        // Advance to next waypoint (should be the original WP2)
        WaypointQueue::advance();
        assert(WaypointQueue::isFinal());
        wp = WaypointQueue::getCurrentWaypoint();
        assert(approxEqual(wp.x, 180.0f) && approxEqual(wp.y, 70.0f));
        std::cout << "  ✓ Test 4: Dynamic detour waypoint injection verified." << std::endl;
    }

    std::cout << "[TDD] ALL test_waypoint_queue passed successfully!" << std::endl;
    return 0;
}
