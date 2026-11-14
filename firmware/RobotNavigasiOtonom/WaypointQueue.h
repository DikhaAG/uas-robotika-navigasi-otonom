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
    static void clearQueue() { clear(); }
    static void loadRoom(uint8_t roomNum);

    static bool append(float x, float y);
    static bool appendWaypoint(float x, float y) { return append(x, y); }

    static bool injectFront(float x, float y);
    static bool injectWaypointFront(float x, float y) { return injectFront(x, y); }

    static Waypoint getCurrentWaypoint();
    static uint8_t getCurrentIndex();
    static uint8_t getCount();
    static uint8_t getTotalWaypoints() { return getCount(); }

    static bool advance();
    static bool advanceWaypoint() { return advance(); }

    static bool isFinal();
    static bool isFinalWaypoint() { return isFinal(); }

    static bool isEmpty();
    static bool isQueueEmpty() { return isEmpty(); }

private:
    static Waypoint queue[MAX_WAYPOINTS];
    static uint8_t currentIndex;
    static uint8_t totalWaypoints;
};

#endif // WAYPOINT_QUEUE_H
