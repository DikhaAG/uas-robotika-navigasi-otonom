#include "Waypoints.h"
#include "Sensors.h"
#include <math.h>

Waypoint Waypoints::queue[MAX_WAYPOINTS];
uint8_t Waypoints::currentIndex = 0;
uint8_t Waypoints::totalWaypoints = 0;

void Waypoints::init() {
    clearQueue();
}

void Waypoints::clearQueue() {
    currentIndex = 0;
    totalWaypoints = 0;
    for (int i = 0; i < MAX_WAYPOINTS; ++i) {
        queue[i].x = 0.0f;
        queue[i].y = 0.0f;
    }
}

void Waypoints::loadRoom(uint8_t roomNum) {
    clearQueue();
    switch (roomNum) {
        case 1:
            appendWaypoint(180.0f, 0.0f);   // WP1 Doorway
            appendWaypoint(180.0f, 70.0f);  // WP2 Room Center
            break;
        case 2:
            appendWaypoint(295.0f, 0.0f);   // WP1 Doorway
            appendWaypoint(295.0f, 70.0f);  // WP2 Room Center
            break;
        case 3:
            appendWaypoint(235.0f, 0.0f);   // WP1 Doorway
            appendWaypoint(235.0f, -70.0f); // WP2 Room Center
            break;
        case 4:
            appendWaypoint(355.0f, 0.0f);   // WP1 Doorway
            appendWaypoint(355.0f, -70.0f); // WP2 Room Center
            break;
        default:
            // Default to Room 1
            appendWaypoint(180.0f, 0.0f);
            appendWaypoint(180.0f, 70.0f);
            break;
    }
}

bool Waypoints::appendWaypoint(float x, float y) {
    if (totalWaypoints >= MAX_WAYPOINTS) {
        return false;
    }
    queue[totalWaypoints].x = x;
    queue[totalWaypoints].y = y;
    totalWaypoints++;
    return true;
}

bool Waypoints::injectWaypointFront(float x, float y) {
    if (totalWaypoints >= MAX_WAYPOINTS) {
        return false;
    }
    // Shift elements from totalWaypoints down to currentIndex
    for (int i = totalWaypoints; i > (int)currentIndex; --i) {
        queue[i] = queue[i - 1];
    }
    queue[currentIndex].x = x;
    queue[currentIndex].y = y;
    totalWaypoints++;
    return true;
}

Waypoint Waypoints::getCurrentWaypoint() {
    if (currentIndex < totalWaypoints) {
        return queue[currentIndex];
    }
    Waypoint zero = {0.0f, 0.0f};
    return zero;
}

uint8_t Waypoints::getCurrentIndex() {
    return currentIndex;
}

uint8_t Waypoints::getTotalWaypoints() {
    return totalWaypoints;
}

bool Waypoints::advanceWaypoint() {
    if (currentIndex < totalWaypoints) {
        currentIndex++;
        return true;
    }
    return false;
}

bool Waypoints::isFinalWaypoint() {
    if (totalWaypoints == 0) return true;
    return (currentIndex >= totalWaypoints - 1);
}

bool Waypoints::isQueueEmpty() {
    return (currentIndex >= totalWaypoints);
}

void Waypoints::computeErrors(const Pose& pose, float& rho, float& e) {
    Waypoint target = getCurrentWaypoint();
    float dx = target.x - pose.x;
    float dy = target.y - pose.y;

    rho = sqrtf(dx * dx + dy * dy);
    float targetAngleDeg = atan2f(dy, dx) * (180.0f / PI);
    e = Odometry::normalizeAngleDeg(targetAngleDeg - pose.theta);
}

bool Waypoints::computeMotorCommands(const Pose& pose, int& pwmL, int& pwmR) {
    float rho = 0.0f;
    float e = 0.0f;
    computeErrors(pose, rho, e);

    // Large angle error -> in-place rotation
    if (fabsf(e) > AMBANG_BELOK_DEG) {
        if (e > 0.0f) {
            // Turn left
            pwmL = -PWM_TURN;
            pwmR = +PWM_TURN;
        } else {
            // Turn right
            pwmL = +PWM_TURN;
            pwmR = -PWM_TURN;
        }
        return true; // is turning
    }

    // Small angle error -> Proportional forward drive
    int base = (rho < JARAK_DECEL_CM) ? PWM_SLOW : PWM_BASE;
    int u = (int)(KP_HEADING * e);

    // Lateral wall centering fine correction (User story 19)
    // To steer away from left wall: turn right (pwmL higher, pwmR lower)
    // To steer away from right wall: turn left (pwmL lower, pwmR higher)
    int wallCorrection = 0;
    if (Sensors::isWallLeft() && !Sensors::isWallRight()) {
        wallCorrection = +15; // Steer right away from left wall
    } else if (Sensors::isWallRight() && !Sensors::isWallLeft()) {
        wallCorrection = -15; // Steer left away from right wall
    }

    pwmL = base - u + wallCorrection;
    pwmR = base + u - wallCorrection;

    // Clamp forward drive between 0 and 255
    if (pwmL > 255) pwmL = 255;
    if (pwmL < 0) pwmL = 0;
    if (pwmR > 255) pwmR = 255;
    if (pwmR < 0) pwmR = 0;

    return false; // is forward driving
}

bool Waypoints::checkWaypointAchieved(const Pose& pose) {
    float rho = 0.0f;
    float e = 0.0f;
    computeErrors(pose, rho, e);

    float threshold = isFinalWaypoint() ? TOL_AKHIR_CM : TOL_WP_CM;
    return (rho <= threshold);
}
