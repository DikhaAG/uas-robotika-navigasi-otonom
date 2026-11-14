#ifndef NAVIGATION_FSM_H
#define NAVIGATION_FSM_H

#include "Config.h"
#include "Hal.h"
#include "Motors.h"
#include "Odometry.h"
#include "WaypointQueue.h"
#include "MotionController.h"
#include "ObstacleAvoidance.h"
#include <stdint.h>

enum NavState {
    STATE_IDLE,
    STATE_KALIBRASI,
    STATE_ORIENTASI,
    STATE_MAJU,
    STATE_PINDAI,
    STATE_MUNDUR,
    STATE_TIBA,
    STATE_GAGAL
};

enum FailsafeReason {
    FAILSAFE_NONE,
    FAILSAFE_TIMEOUT,
    FAILSAFE_STALL,
    FAILSAFE_BOUNDARY,
    FAILSAFE_ENTRAPMENT
};

class NavigationFSM {
public:
    static void init();
    static void update();

    static void selectRoom(uint8_t room);
    static uint8_t getTargetRoom();
    static void triggerStart();

    static NavState getState();
    static const char* getStateName(NavState state);
    static FailsafeReason getLastFailsafe();

    static void setState(NavState state);
    static void setEscapeRetries(uint8_t retries);

private:
    static NavState currentState;
    static uint8_t targetRoom;
    static FailsafeReason lastFailsafe;

    static uint32_t navStartTimeMs;
    static uint32_t stateEnterTimeMs;
    static uint32_t lastTelemetryTimeMs;

    // Failsafe tracking
    static uint32_t lastMotionTimestampMs;
    static float lastMotionDistanceCm;
    static uint8_t escapeRetries;
    static float reverseStartDistCm;

    static void transitionTo(NavState newState);
    static bool checkFailsafes(const Pose& pose);
    static void streamTelemetry(const Pose& pose);
    static void handleArrivalSignaling();
    static void handleFailsafeSignaling();
};

#endif // NAVIGATION_FSM_H
