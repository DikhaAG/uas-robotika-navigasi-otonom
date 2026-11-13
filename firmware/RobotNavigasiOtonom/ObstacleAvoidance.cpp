#include "ObstacleAvoidance.h"
#include "Hal.h"
#include <math.h>

ScanState ObstacleAvoidance::scanState = SCAN_IDLE;
float ObstacleAvoidance::initialHeading = 0.0f;
float ObstacleAvoidance::detectedDistance = 25.0f;
float ObstacleAvoidance::leftClearance = 0.0f;
float ObstacleAvoidance::rightClearance = 0.0f;
uint32_t ObstacleAvoidance::stateTimerMs = 0;

bool ObstacleAvoidance::shouldTrigger(const Pose& pose, float frontDistance, bool isFinalSegment) {
    if (isFinalSegment) {
        return false; // Suppressed during final room entry
    }
    // Only trigger when moving along corridor and distance < threshold
    return (frontDistance < JARAK_RINTANGAN_CM && fabsf(pose.theta) < 20.0f);
}

float ObstacleAvoidance::decideBypassLane(float leftClear, float rightClear) {
    if (leftClear >= rightClear) {
        return +LANE_OFFSET_Y_CM; // Left corridor lane (y = +20 cm)
    } else {
        return -LANE_OFFSET_Y_CM; // Right corridor lane (y = -20 cm)
    }
}

bool ObstacleAvoidance::isBothBlocked(float leftClear, float rightClear) {
    return (leftClear < JARAK_LOLOS_MIN_CM && rightClear < JARAK_LOLOS_MIN_CM);
}

bool ObstacleAvoidance::injectBypassWaypoints(const Pose& pose, float measuredDist, float laneY) {
    detectedDistance = measuredDist;

    // Target doorway waypoint for clamping
    Waypoint target = Waypoints::getCurrentWaypoint();
    float maxX = target.x - 5.0f;

    Waypoint wpA = { pose.x + 15.0f, laneY };
    Waypoint wpB = { pose.x + measuredDist + 45.0f, laneY };
    Waypoint wpC = { pose.x + measuredDist + 65.0f, 0.0f };

    // Clamp X so waypoints never overshoot the room doorway
    if (target.x > pose.x) {
        if (wpA.x > maxX) wpA.x = maxX;
        if (wpB.x > maxX) wpB.x = maxX;
        if (wpC.x > maxX) wpC.x = maxX;
    }

    // Inject in reverse order so wpA is at currentIndex
    if (!Waypoints::injectWaypointFront(wpC.x, wpC.y)) return false;
    if (!Waypoints::injectWaypointFront(wpB.x, wpB.y)) return false;
    if (!Waypoints::injectWaypointFront(wpA.x, wpA.y)) return false;

    return true;
}

void ObstacleAvoidance::reset() {
    scanState = SCAN_IDLE;
    initialHeading = 0.0f;
    detectedDistance = 25.0f;
    leftClearance = 0.0f;
    rightClearance = 0.0f;
    stateTimerMs = 0;
}

void ObstacleAvoidance::startScan(float baseHeadingDeg) {
    initialHeading = baseHeadingDeg;
    scanState = SCAN_TURN_LEFT;
    leftClearance = 0.0f;
    rightClearance = 0.0f;
    stateTimerMs = Hal::millis();
}

ScanState ObstacleAvoidance::updateScan(const Pose& pose, float currentDist, int& pwmL, int& pwmR) {
    uint32_t now = Hal::millis();
    pwmL = 0;
    pwmR = 0;

    switch (scanState) {
        case SCAN_TURN_LEFT: {
            float targetDeg = Odometry::normalizeAngleDeg(initialHeading + SUDUT_PINDAI_DEG);
            float err = Odometry::normalizeAngleDeg(targetDeg - pose.theta);
            if (fabsf(err) <= TOL_ORIENTASI_DEG) {
                // Reached +30 deg, pause to settle
                scanState = SCAN_PAUSE_LEFT;
                stateTimerMs = now;
            } else {
                pwmL = (err > 0.0f) ? -PWM_TURN : +PWM_TURN;
                pwmR = (err > 0.0f) ? +PWM_TURN : -PWM_TURN;
            }
            break;
        }

        case SCAN_PAUSE_LEFT: {
            if (now - stateTimerMs >= 150UL) {
                leftClearance = currentDist;
                scanState = SCAN_TURN_RIGHT;
            }
            break;
        }

        case SCAN_TURN_RIGHT: {
            float targetDeg = Odometry::normalizeAngleDeg(initialHeading - SUDUT_PINDAI_DEG);
            float err = Odometry::normalizeAngleDeg(targetDeg - pose.theta);
            if (fabsf(err) <= TOL_ORIENTASI_DEG) {
                scanState = SCAN_PAUSE_RIGHT;
                stateTimerMs = now;
            } else {
                pwmL = (err > 0.0f) ? -PWM_TURN : +PWM_TURN;
                pwmR = (err > 0.0f) ? +PWM_TURN : -PWM_TURN;
            }
            break;
        }

        case SCAN_PAUSE_RIGHT: {
            if (now - stateTimerMs >= 150UL) {
                rightClearance = currentDist;
                scanState = SCAN_REALIGN;
            }
            break;
        }

        case SCAN_REALIGN: {
            float err = Odometry::normalizeAngleDeg(initialHeading - pose.theta);
            if (fabsf(err) <= TOL_ORIENTASI_DEG) {
                if (isBothBlocked(leftClearance, rightClearance)) {
                    scanState = SCAN_ESCAPE_NEEDED;
                } else {
                    scanState = SCAN_DONE;
                }
            } else {
                pwmL = (err > 0.0f) ? -PWM_TURN : +PWM_TURN;
                pwmR = (err > 0.0f) ? +PWM_TURN : -PWM_TURN;
            }
            break;
        }

        case SCAN_DONE:
        case SCAN_ESCAPE_NEEDED:
        case SCAN_IDLE:
        default:
            pwmL = 0;
            pwmR = 0;
            break;
    }

    return scanState;
}

float ObstacleAvoidance::getDetectedDistance() {
    return detectedDistance;
}

float ObstacleAvoidance::getLeftClearance() {
    return leftClearance;
}

float ObstacleAvoidance::getRightClearance() {
    return rightClearance;
}
