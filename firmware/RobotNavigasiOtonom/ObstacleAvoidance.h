#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include "Config.h"
#include "Odometry.h"
#include "Waypoints.h"
#include <stdint.h>

enum ScanState {
    SCAN_IDLE,
    SCAN_TURN_LEFT,
    SCAN_PAUSE_LEFT,
    SCAN_TURN_RIGHT,
    SCAN_PAUSE_RIGHT,
    SCAN_REALIGN,
    SCAN_DONE,
    SCAN_ESCAPE_NEEDED
};

class ObstacleAvoidance {
public:
    static bool shouldTrigger(const Pose& pose, float frontDistance, bool isFinalSegment);
    static float decideBypassLane(float leftClearance, float rightClearance);
    static bool isBothBlocked(float leftClearance, float rightClearance);
    static bool injectBypassWaypoints(const Pose& pose, float measuredDist, float laneY);

    // Non-blocking scan routine
    static void startScan(float baseHeadingDeg);
    static ScanState updateScan(const Pose& pose, float currentDist, int& pwmL, int& pwmR);
    static float getDetectedDistance();
    static float getLeftClearance();
    static float getRightClearance();

private:
    static ScanState scanState;
    static float initialHeading;
    static float detectedDistance;
    static float leftClearance;
    static float rightClearance;
    static uint32_t stateTimerMs;
};

#endif // OBSTACLE_AVOIDANCE_H
