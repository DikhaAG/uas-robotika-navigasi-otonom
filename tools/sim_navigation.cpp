#include "Config.h"
#include "Hal.h"
#include "Motors.h"
#include "Sensors.h"
#include "Odometry.h"
#include "WaypointQueue.h"
#include "MotionController.h"
#include "ObstacleAvoidance.h"
#include "CalibrationProfile.h"
#include "NavigationFSM.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cstdio>

struct Obstacle {
    float x;
    float y;
    float w;
    float h;

    float xmin() const { return x - w / 2.0f; }
    float xmax() const { return x + w / 2.0f; }
    float ymin() const { return y - h / 2.0f; }
    float ymax() const { return y + h / 2.0f; }

    bool raycast(float rx, float ry, float dx, float dy, float& tHit) const {
        float x0 = xmin();
        float x1 = xmax();
        float y0 = ymin();
        float y1 = ymax();

        float tx1 = -1e9f, tx2 = 1e9f;
        if (fabsf(dx) > 1e-6f) {
            tx1 = (x0 - rx) / dx;
            tx2 = (x1 - rx) / dx;
            if (tx1 > tx2) std::swap(tx1, tx2);
        } else {
            // Ray parallel to Y axis: must be within X bounds to intersect
            if (rx < x0 || rx > x1) return false;
        }

        float ty1 = -1e9f, ty2 = 1e9f;
        if (fabsf(dy) > 1e-6f) {
            ty1 = (y0 - ry) / dy;
            ty2 = (y1 - ry) / dy;
            if (ty1 > ty2) std::swap(ty1, ty2);
        } else {
            // Ray parallel to X axis: must be within Y bounds to intersect
            if (ry < y0 || ry > y1) return false;
        }

        float tEnter = std::max(tx1, ty1);
        float tExit = std::min(tx2, ty2);

        if (tExit >= tEnter && tExit > 0.0f) {
            tHit = (tEnter > 0.0f) ? tEnter : 0.0f;
            return true;
        }
        return false;
    }

    float distanceToPoint(float px, float py) const {
        float distToEdgeX = std::max(0.0f, std::max(xmin() - px, px - xmax()));
        float distToEdgeY = std::max(0.0f, std::max(ymin() - py, py - ymax()));
        return sqrtf(distToEdgeX * distToEdgeX + distToEdgeY * distToEdgeY);
    }
};

class ArenaPhysics {
public:
    static const float CORRIDOR_X_MIN;
    static const float CORRIDOR_X_MAX;
    static const float CORRIDOR_Y_MAX;
    static const float CORRIDOR_Y_MIN;
    static const float ROOM_BACK_WALL_Y; // +/- 100 cm (40 cm corridor + 60 cm room depth)
    static const float DOORWAY_HALF_WIDTH; // 25 cm (50 cm doorway aperture)
    static const float MAX_SONAR_RANGE;

    static float raycast(float rx, float ry, float thetaDeg, const std::vector<Obstacle>& obstacles, uint8_t targetRoom) {
        float rad = thetaDeg * (PI / 180.0f);
        float dx = cosf(rad);
        float dy = sinf(rad);
        float minDist = MAX_SONAR_RANGE;

        // 1. Raycast against obstacles
        for (const auto& obs : obstacles) {
            float tHit = 0.0f;
            if (obs.raycast(rx, ry, dx, dy, tHit)) {
                if (tHit < minDist) minDist = tHit;
            }
        }

        // Room doorway opening check (doorway width 50 cm, radius 25 cm)
        float doorCenter = 0.0f;
        if (targetRoom >= 1 && targetRoom <= 4) {
            doorCenter = ROOM_WP1[targetRoom - 1][0];
        }

        // 2. Raycast against corridor walls
        // Top corridor wall (y = +40)
        if (dy > 1e-6f) {
            float t = (CORRIDOR_Y_MAX - ry) / dy;
            float hitX = rx + t * dx;
            bool inDoorway = (targetRoom <= 2 && fabsf(hitX - doorCenter) < DOORWAY_HALF_WIDTH);
            if (!inDoorway && t > 0.0f && t < minDist) {
                minDist = t;
            }
        }

        // Bottom corridor wall (y = -40)
        if (dy < -1e-6f) {
            float t = (CORRIDOR_Y_MIN - ry) / dy;
            float hitX = rx + t * dx;
            bool inDoorway = (targetRoom >= 3 && fabsf(hitX - doorCenter) < DOORWAY_HALF_WIDTH);
            if (!inDoorway && t > 0.0f && t < minDist) {
                minDist = t;
            }
        }

        // End corridor wall (x = 480)
        if (dx > 1e-6f) {
            float t = (CORRIDOR_X_MAX - rx) / dx;
            float hitY = ry + t * dy;
            if (hitY >= CORRIDOR_Y_MIN && hitY <= CORRIDOR_Y_MAX && t > 0.0f && t < minDist) {
                minDist = t;
            }
        }

        // Start corridor wall (x = 0)
        if (dx < -1e-6f) {
            float t = (CORRIDOR_X_MIN - rx) / dx;
            float hitY = ry + t * dy;
            if (hitY >= CORRIDOR_Y_MIN && hitY <= CORRIDOR_Y_MAX && t > 0.0f && t < minDist) {
                minDist = t;
            }
        }

        // 3. Room back walls (y = +100 for rooms 1-2, y = -100 for rooms 3-4, Brief p. 7 & 29)
        if (targetRoom <= 2 && dy > 1e-6f) {
            float t = (ROOM_BACK_WALL_Y - ry) / dy;
            if (t > 0.0f && t < minDist) minDist = t;
        } else if (targetRoom >= 3 && dy < -1e-6f) {
            float t = (-ROOM_BACK_WALL_Y - ry) / dy;
            if (t > 0.0f && t < minDist) minDist = t;
        }

        return std::min(minDist, MAX_SONAR_RANGE);
    }
};

const float ArenaPhysics::CORRIDOR_X_MIN = 0.0f;
const float ArenaPhysics::CORRIDOR_X_MAX = 480.0f;
const float ArenaPhysics::CORRIDOR_Y_MAX = 40.0f;
const float ArenaPhysics::CORRIDOR_Y_MIN = -40.0f;
const float ArenaPhysics::ROOM_BACK_WALL_Y = 100.0f;
const float ArenaPhysics::DOORWAY_HALF_WIDTH = 25.0f;
const float ArenaPhysics::MAX_SONAR_RANGE = 400.0f;

int main(int argc, char** argv) {
    uint8_t roomNum = 1;
    std::string csvPath = "";
    int maxSteps = 4500; // 90 seconds max (matches TIMEOUT_GLOBAL_MS @ 20ms/step)
    std::vector<Obstacle> obstacles;

    // Parse CLI arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--room" && i + 1 < argc) {
            roomNum = (uint8_t)std::atoi(argv[++i]);
        } else if (arg == "--csv" && i + 1 < argc) {
            csvPath = argv[++i];
        } else if (arg == "--max-steps" && i + 1 < argc) {
            maxSteps = std::atoi(argv[++i]);
        } else if ((arg == "--obstacle" || arg == "--obs") && i + 1 < argc) {
            std::string obsStr = argv[++i];
            std::stringstream ss(obsStr);
            std::string item;
            std::vector<float> vals;
            while (std::getline(ss, item, ',')) {
                vals.push_back(std::stof(item));
            }
            if (vals.size() == 4) {
                obstacles.push_back({vals[0], vals[1], vals[2], vals[3]});
            }
        }
    }

    // Initialize mock environment and firmware modules
    Hal::resetMock();
    Motors::init();
    Sensors::init();
    Odometry::init();
    WaypointQueue::init();
    ObstacleAvoidance::reset();
    NavigationFSM::init();

    // Select target room and trigger start
    NavigationFSM::selectRoom(roomNum);
    NavigationFSM::triggerStart();

    // Virtual Ground-Truth State
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;

    // Pulse accumulator fractions
    float pulseAccL = 0.0f;
    float pulseAccR = 0.0f;

    // Tracking stats
    float minWallClearance = 40.0f;
    float minObstacleClearance = 999.0f;

    std::ofstream csvFile;
    if (!csvPath.empty()) {
        csvFile.open(csvPath);
        csvFile << "t_ms,state,x,y,theta,jarak,iWp\n";
    }

    const float dt = 0.02f; // 20 ms deterministic step
    const float KV = 0.165f; // cm/s per PWM
    uint32_t simTimeMs = 0;
    int steps = 0;

    while (steps < maxSteps) {
        steps++;
        simTimeMs += 20;
        Hal::advanceTimeMs(20);

        NavState currentState = NavigationFSM::getState();
        if (currentState == STATE_TIBA || currentState == STATE_GAGAL) {
            break;
        }

        // 1. Raycast virtual sonar sensor
        float sonarDist = ArenaPhysics::raycast(x, y, theta, obstacles, roomNum);
        Sensors::setMockDistance(sonarDist);

        // 2. Lateral IR wall sensors (within corridor segment)
        bool wallLeft = false;
        bool wallRight = false;
        if (fabsf(theta) < 30.0f) {
            float distToLeftWall = ArenaPhysics::CORRIDOR_Y_MAX - y;
            float distToRightWall = y - ArenaPhysics::CORRIDOR_Y_MIN;
            if (distToLeftWall <= 20.0f) wallLeft = true;
            if (distToRightWall <= 20.0f) wallRight = true;
        }
        Sensors::setMockIr(wallLeft, wallRight);

        // 3. Tick Odometry and FSM
        Odometry::update(dt);
        NavigationFSM::update();

        // 4. Read Motor PWM outputs and calculate physical differential drive response
        int dirL = Motors::getDirectionLeft();
        int dirR = Motors::getDirectionRight();
        int pwmL = (int)Hal::getPwm(PIN_ENA) * dirL;
        int pwmR = (int)Hal::getPwm(PIN_ENB) * dirR;

        float vL = (float)pwmL * KV;
        float vR = (float)pwmR * KV;

        float v = (vL + vR) / 2.0f;
        float omegaRad = (vR - vL) / TRACK_WIDTH_CM;
        float omegaDeg = omegaRad * (180.0f / PI);

        // 5. Update ground-truth position
        float rad = theta * (PI / 180.0f);
        x += v * cosf(rad) * dt;
        y += v * sinf(rad) * dt;
        theta = Odometry::normalizeAngleDeg(theta + omegaDeg * dt);

        // 6. Inject encoder pulses & gyro angular velocity for next Odometry tick
        float dsL = fabsf(vL * dt);
        float dsR = fabsf(vR * dt);
        pulseAccL += dsL / CM_PER_PULSA;
        pulseAccR += dsR / CM_PER_PULSA;

        long pL = (long)pulseAccL;
        long pR = (long)pulseAccR;
        pulseAccL -= (float)pL;
        pulseAccR -= (float)pR;

        Odometry::injectPulses(pL, pR);
        Sensors::setMockGyroZ(omegaDeg);

        // 7. Track clearances
        bool inFinalRoom = WaypointQueue::isFinal();
        if (!inFinalRoom) {
            float wallClear = 40.0f - fabsf(y);
            if (wallClear < minWallClearance) minWallClearance = wallClear;
        }

        for (const auto& obs : obstacles) {
            float obsDist = obs.distanceToPoint(x, y);
            if (obsDist < minObstacleClearance) minObstacleClearance = obsDist;
        }

        // 8. Stream telemetry with deterministic fixed-precision formatting
        if (csvFile.is_open()) {
            char lineBuf[128];
            snprintf(lineBuf, sizeof(lineBuf), "%lu,%s,%.2f,%.2f,%.2f,%.2f,%u\n",
                     (unsigned long)simTimeMs,
                     NavigationFSM::getStateName(currentState),
                     x, y, theta,
                     sonarDist,
                     (unsigned int)WaypointQueue::getCurrentIndex());
            csvFile << lineBuf;
        }
    }

    if (csvFile.is_open()) {
        csvFile.close();
    }

    // Target Room Waypoint WP2
    uint8_t roomIdx = (roomNum >= 1 && roomNum <= 4) ? (roomNum - 1) : 0;
    float targetX = ROOM_WP2[roomIdx][0];
    float targetY = ROOM_WP2[roomIdx][1];
    float errX = targetX - x;
    float errY = targetY - y;
    float finalTargetError = sqrtf(errX * errX + errY * errY);

    NavState finalState = NavigationFSM::getState();
    const char* finalStateName = NavigationFSM::getStateName(finalState);

    // Print human-readable summary
    std::cout << "  Result State : " << finalStateName << "\n";
    std::cout << "  Final Pose   : (" << x << " cm, " << y << " cm, " << theta << "°)\n";
    std::cout << "  Target Error : " << finalTargetError << " cm (Spec <= 5.0 cm)\n";
    std::cout << "  Min Wall Clr : " << minWallClearance << " cm\n";
    if (!obstacles.empty()) {
        std::cout << "  Min Obs Clr  : " << minObstacleClearance << " cm\n";
    }

    // Verification check: Must arrive in TIBA within target tolerance
    // Robot half-width is ~7 cm, so chassis clearance = minWallClearance - 7 cm >= 3 cm (minWallClearance >= 10 cm)
    bool pass = (finalState == STATE_TIBA && finalTargetError <= 5.0f && minWallClearance >= 10.0f);
    if (!obstacles.empty()) {
        if (minObstacleClearance <= 7.0f) pass = false; // Robot radius ~7cm
    }

    return pass ? 0 : 1;
}
