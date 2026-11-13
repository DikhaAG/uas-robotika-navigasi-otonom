#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include "Config.h"
#include "Odometry.h"
#include <stdint.h>

#define MAX_WAYPOINTS 8

struct Waypoint {
    float x; // cm
    float y; // cm
};

class Waypoints {
public:
    static void init();
    static void clearQueue();
    static void loadRoom(uint8_t roomNum);

    static bool appendWaypoint(float x, float y);
    static bool injectWaypointFront(float x, float y);

    static Waypoint getCurrentWaypoint();
    static uint8_t getCurrentIndex();
    static uint8_t getTotalWaypoints();

    static bool advanceWaypoint();
    static bool isFinalWaypoint();
    static bool isQueueEmpty();

    static void computeErrors(const Pose& pose, float& rho, float& e);
    static bool computeMotorCommands(const Pose& pose, int& pwmL, int& pwmR);
    static bool checkWaypointAchieved(const Pose& pose);

private:
    static Waypoint queue[MAX_WAYPOINTS];
    static uint8_t currentIndex;
    static uint8_t totalWaypoints;
};

#endif // WAYPOINTS_H
