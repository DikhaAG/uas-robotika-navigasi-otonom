#include "WaypointQueue.h"

Waypoint WaypointQueue::queue[MAX_WAYPOINTS];
uint8_t WaypointQueue::currentIndex = 0;
uint8_t WaypointQueue::totalWaypoints = 0;

void WaypointQueue::init() {
    clear();
}

void WaypointQueue::clear() {
    currentIndex = 0;
    totalWaypoints = 0;
    for (int i = 0; i < MAX_WAYPOINTS; ++i) {
        queue[i].x = 0.0f;
        queue[i].y = 0.0f;
    }
}

void WaypointQueue::loadRoom(uint8_t roomNum) {
    clear();
    uint8_t idx = 0;
    if (roomNum >= 1 && roomNum <= 4) {
        idx = roomNum - 1;
    }
    append(ROOM_WP1[idx][0], ROOM_WP1[idx][1]);
    append(ROOM_WP2[idx][0], ROOM_WP2[idx][1]);
}

bool WaypointQueue::append(float x, float y) {
    if (totalWaypoints >= MAX_WAYPOINTS) {
        return false;
    }
    queue[totalWaypoints].x = x;
    queue[totalWaypoints].y = y;
    totalWaypoints++;
    return true;
}

bool WaypointQueue::injectFront(float x, float y) {
    if (totalWaypoints >= MAX_WAYPOINTS) {
        return false;
    }
    for (int i = totalWaypoints; i > (int)currentIndex; --i) {
        queue[i] = queue[i - 1];
    }
    queue[currentIndex].x = x;
    queue[currentIndex].y = y;
    totalWaypoints++;
    return true;
}

Waypoint WaypointQueue::getCurrentWaypoint() {
    if (currentIndex < totalWaypoints) {
        return queue[currentIndex];
    }
    Waypoint zero = {0.0f, 0.0f};
    return zero;
}

uint8_t WaypointQueue::getCurrentIndex() {
    return currentIndex;
}

uint8_t WaypointQueue::getCount() {
    return totalWaypoints;
}

bool WaypointQueue::advance() {
    if (currentIndex < totalWaypoints) {
        currentIndex++;
        return true;
    }
    return false;
}

bool WaypointQueue::isFinal() {
    if (isEmpty()) return false;
    return (currentIndex == totalWaypoints - 1);
}

bool WaypointQueue::isEmpty() {
    return (currentIndex >= totalWaypoints);
}
