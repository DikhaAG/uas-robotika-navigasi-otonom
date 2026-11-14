#ifndef WAYPOINT_QUEUE_H
#define WAYPOINT_QUEUE_H

#include "Config.h"
#include <stdint.h>

#define MAX_WAYPOINTS 8

struct Waypoint {
    float x; // cm
    float y; // cm
};

class WaypointQueue {
public:
    static void init();
    static void clear();
    static void loadRoom(uint8_t roomNum);

    static bool append(float x, float y);
    static bool injectFront(float x, float y);

    static Waypoint getCurrentWaypoint();
    static uint8_t getCurrentIndex();
    static uint8_t getCount();

    static bool advance();
    static bool isFinal();
    static bool isEmpty();

private:
    static Waypoint queue[MAX_WAYPOINTS];
    static uint8_t currentIndex;
    static uint8_t totalWaypoints;
};

#endif // WAYPOINT_QUEUE_H
