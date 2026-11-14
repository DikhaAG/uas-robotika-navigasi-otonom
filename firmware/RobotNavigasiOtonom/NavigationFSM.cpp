#include "NavigationFSM.h"
#include "AsciiProtocolDecoder.h"
#include "CalibrationService.h"
#include <stdio.h>
#include <math.h>

NavState NavigationFSM::currentState = STATE_IDLE;
uint8_t NavigationFSM::targetRoom = 1;
FailsafeReason NavigationFSM::lastFailsafe = FAILSAFE_NONE;

uint32_t NavigationFSM::navStartTimeMs = 0;
uint32_t NavigationFSM::stateEnterTimeMs = 0;
uint32_t NavigationFSM::lastTelemetryTimeMs = 0;

uint32_t NavigationFSM::lastMotionTimestampMs = 0;
float NavigationFSM::lastMotionDistanceCm = 0.0f;
static long lastStallCheckPulses = 0;
uint8_t NavigationFSM::escapeRetries = 0;
float NavigationFSM::reverseStartDistCm = 0.0f;

const char* NavigationFSM::getStateName(NavState state) {
    switch (state) {
        case STATE_IDLE:      return "IDLE";
        case STATE_KALIBRASI: return "KALIBRASI";
        case STATE_ORIENTASI: return "ORIENTASI";
        case STATE_MAJU:      return "MAJU";
        case STATE_PINDAI:    return "PINDAI";
        case STATE_MUNDUR:    return "MUNDUR";
        case STATE_TIBA:      return "TIBA";
        case STATE_GAGAL:     return "GAGAL";
        default:              return "UNKNOWN";
    }
}

void NavigationFSM::transitionTo(NavState newState) {
    currentState = newState;
    stateEnterTimeMs = Hal::millis();

    char buf[40];
    snprintf(buf, sizeof(buf), "# STATE -> %s\r\n", getStateName(newState));
    Hal::serialPrint(buf);
}

void NavigationFSM::init() {
    Hal::pinMode(PIN_BUTTON_START, INPUT_PULLUP);
    Hal::pinMode(PIN_LED, OUTPUT);
    Hal::pinMode(PIN_BUZZER, OUTPUT);
    Hal::digitalWrite(PIN_LED, LOW);
    Hal::digitalWrite(PIN_BUZZER, LOW);

    currentState = STATE_IDLE;
    targetRoom = 1;
    lastFailsafe = FAILSAFE_NONE;
    escapeRetries = 0;
    navStartTimeMs = 0;
    stateEnterTimeMs = Hal::millis();
    lastTelemetryTimeMs = 0;
    lastMotionTimestampMs = Hal::millis();
    lastMotionDistanceCm = 0.0f;

    Motors::stopMotors();
    WaypointQueue::loadRoom(targetRoom);
    AsciiProtocolDecoder::init(NavigationFSM::selectRoom);
    CalibrationService::init();
}

void NavigationFSM::selectRoom(uint8_t room) {
    if (room >= 1 && room <= 4) {
        targetRoom = room;
        WaypointQueue::loadRoom(targetRoom);
    }
}

uint8_t NavigationFSM::getTargetRoom() {
    return targetRoom;
}

void NavigationFSM::triggerStart() {
    if (currentState == STATE_IDLE) {
        WaypointQueue::loadRoom(targetRoom);
        transitionTo(STATE_KALIBRASI);
    }
}

NavState NavigationFSM::getState() {
    return currentState;
}

FailsafeReason NavigationFSM::getLastFailsafe() {
    return lastFailsafe;
}

void NavigationFSM::setState(NavState state) {
    currentState = state;
    stateEnterTimeMs = Hal::millis();
    lastMotionTimestampMs = Hal::millis();
    lastMotionDistanceCm = Odometry::getDistanceMoved();
}

void NavigationFSM::setEscapeRetries(uint8_t retries) {
    escapeRetries = retries;
}

bool NavigationFSM::checkFailsafes(const Pose& pose) {
    // Only enforce active watchdogs during navigation states
    if (currentState != STATE_ORIENTASI &&
        currentState != STATE_MAJU &&
        currentState != STATE_PINDAI &&
        currentState != STATE_MUNDUR) {
        return false;
    }

    uint32_t now = Hal::millis();

    // 1. Failsafe: Global Timeout (> 90 s)
    if (now - navStartTimeMs > TIMEOUT_GLOBAL_MS) {
        lastFailsafe = FAILSAFE_TIMEOUT;
        Motors::stopMotors();
        transitionTo(STATE_GAGAL);
        return true;
    }

    // 2. Failsafe: Motor Stall (active drive with zero pulses for >= 1.0s)
    bool isMotorActive = (Motors::getDirectionLeft() != 0 || Motors::getDirectionRight() != 0);
    long totalPulses = Odometry::getTotalPulseCount();
    if (isMotorActive) {
        if (totalPulses != lastStallCheckPulses) {
            lastStallCheckPulses = totalPulses;
            lastMotionTimestampMs = now;
        } else {
            if (now - lastMotionTimestampMs >= TIMEOUT_STALL_MS) {
                lastFailsafe = FAILSAFE_STALL;
                Motors::stopMotors();
                transitionTo(STATE_GAGAL);
                return true;
            }
        }
    } else {
        lastMotionTimestampMs = now;
        lastStallCheckPulses = totalPulses;
    }

    // 3. Failsafe: Boundary Excursion (|y| > 40 cm outside room entry, or x < -30 cm)
    if (!WaypointQueue::isFinal()) {
        if (fabsf(pose.y) > BATAS_KORIDOR_Y_CM || pose.x < BATAS_KORIDOR_X_MIN) {
            lastFailsafe = FAILSAFE_BOUNDARY;
            Motors::stopMotors();
            transitionTo(STATE_GAGAL);
            return true;
        }
    }

    // 4. Failsafe: Entrapment (> 3 escape attempts failed)
    if (escapeRetries > MAX_RETRIES_ESCAPE) {
        lastFailsafe = FAILSAFE_ENTRAPMENT;
        Motors::stopMotors();
        transitionTo(STATE_GAGAL);
        return true;
    }

    return false;
}

void NavigationFSM::streamTelemetry(const Pose& pose) {
    uint32_t now = Hal::millis();
    if (now - lastTelemetryTimeMs >= PERIODE_TELEMETRI_MS) {
        lastTelemetryTimeMs = now;
        char line[96];
        snprintf(line, sizeof(line), "%lu,%s,%.2f,%.2f,%.2f,%.2f,%u\r\n",
                 (unsigned long)now,
                 getStateName(currentState),
                 pose.x, pose.y, pose.theta,
                 Sensors::getDistance(),
                 WaypointQueue::getCurrentIndex());
        Hal::serialPrint(line);
    }
}

void NavigationFSM::handleArrivalSignaling() {
    Motors::stopMotors();
    Hal::digitalWrite(PIN_LED, HIGH);

    // 3 distinct audible beeps (100 ms tone, 100 ms silent)
    uint32_t elapsed = Hal::millis() - stateEnterTimeMs;
    if (elapsed < 600UL) {
        uint32_t cycle = elapsed % 200UL;
        Hal::digitalWrite(PIN_BUZZER, (cycle < 100UL) ? HIGH : LOW);
    } else {
        Hal::digitalWrite(PIN_BUZZER, LOW);
    }
}

void NavigationFSM::handleFailsafeSignaling() {
    Motors::stopMotors();
    uint32_t elapsed = Hal::millis() - stateEnterTimeMs;

    // Continuous buzzer for 2.0 seconds
    Hal::digitalWrite(PIN_BUZZER, (elapsed < 2000UL) ? HIGH : LOW);

    // Rapid LED blink (100 ms on / 100 ms off)
    uint32_t blinkCycle = elapsed % 200UL;
    Hal::digitalWrite(PIN_LED, (blinkCycle < 100UL) ? HIGH : LOW);
}

void NavigationFSM::update() {
    uint32_t now = Hal::millis();
    Sensors::updateUltrasonic();
    Pose pose = Odometry::getPose();

    // Check safety watchdogs
    if (checkFailsafes(pose)) {
        return;
    }

    switch (currentState) {
        case STATE_IDLE: {
            Motors::stopMotors();
            Hal::digitalWrite(PIN_LED, (now % 1000UL < 500UL) ? HIGH : LOW);
            Hal::digitalWrite(PIN_BUZZER, LOW);

            // Handle room selection ('1'-'4') and ASCII calibration protocol frames ('!...')
            CalibrationService::update();

            // Check hardware START button (active LOW)
            if (Hal::digitalRead(PIN_BUTTON_START) == LOW) {
                triggerStart();
            }
            break;
        }

        case STATE_KALIBRASI: {
            Motors::stopMotors();
            Hal::digitalWrite(PIN_LED, HIGH);

            // Accumulate gyro bias samples non-blockingly across 2.0 s window
            Sensors::sampleGyroCalibration();

            if (now - stateEnterTimeMs >= 2000UL) {
                Sensors::finalizeGyroCalibration();
                Odometry::resetPose(0.0f, 0.0f, 0.0f);
                navStartTimeMs = now;
                lastMotionTimestampMs = now;
                lastStallCheckPulses = Odometry::getTotalPulseCount();
                transitionTo(STATE_ORIENTASI);
            }
            break;
        }

        case STATE_ORIENTASI: {
            Hal::digitalWrite(PIN_LED, HIGH);
            Waypoint target = WaypointQueue::getCurrentWaypoint();
            float targetHeading = MotionController::computeTargetHeading(pose.x, pose.y, target.x, target.y);

            int pwmL = 0, pwmR = 0;
            if (MotionController::rotateTowards(pose.theta, targetHeading, pwmL, pwmR)) {
                Motors::stopMotors();
                lastMotionTimestampMs = now;
                lastMotionDistanceCm = Odometry::getDistanceMoved();
                transitionTo(STATE_MAJU);
            } else {
                Motors::setMotor(pwmL, pwmR);
            }
            break;
        }

        case STATE_MAJU: {
            Hal::digitalWrite(PIN_LED, HIGH);

            // 1. Check obstacle avoidance trigger
            float frontDist = Sensors::getDistance();
            if (ObstacleAvoidance::shouldTrigger(pose, frontDist, WaypointQueue::isFinal())) {
                Motors::stopMotors();
                ObstacleAvoidance::startScan(pose.theta);
                transitionTo(STATE_PINDAI);
                break;
            }

            // 2. Drive towards target waypoint using MotionController
            Waypoint target = WaypointQueue::getCurrentWaypoint();
            bool isFinal = WaypointQueue::isFinal();
            float tol = isFinal ? TOL_AKHIR_CM : TOL_WP_CM;

            MotionCommand cmd = MotionController::driveTowards(
                pose.x, pose.y, pose.theta,
                target.x, target.y,
                Sensors::isWallLeft(), Sensors::isWallRight(),
                tol
            );

            // 3. Check waypoint arrival
            if (cmd.achieved) {
                if (isFinal) {
                    // Back wall verification: distance to back wall confirms settlement <= 25 cm
                    float backWallDist = Sensors::getDistance();
                    if (backWallDist <= 25.0f || backWallDist >= 350.0f) {
                        transitionTo(STATE_TIBA);
                        break;
                    }
                } else {
                    WaypointQueue::advance();
                    // Re-orient to next waypoint
                    transitionTo(STATE_ORIENTASI);
                    break;
                }
            }

            // 4. Drive motors or re-orient if large heading divergence
            if (cmd.isTurning) {
                transitionTo(STATE_ORIENTASI);
            } else {
                Motors::setMotor(cmd.pwmL, cmd.pwmR);
            }
            break;
        }

        case STATE_PINDAI: {
            int pwmL = 0, pwmR = 0;
            float currentDist = Sensors::getDistance();
            ScanState scState = ObstacleAvoidance::updateScan(pose, currentDist, pwmL, pwmR);

            if (scState == SCAN_DONE) {
                Motors::stopMotors();
                float laneY = ObstacleAvoidance::decideBypassLane(
                    ObstacleAvoidance::getLeftClearance(),
                    ObstacleAvoidance::getRightClearance()
                );
                ObstacleAvoidance::injectBypassWaypoints(pose, ObstacleAvoidance::getDetectedDistance(), laneY);
                escapeRetries = 0;
                transitionTo(STATE_ORIENTASI);
            } else if (scState == SCAN_ESCAPE_NEEDED) {
                Motors::stopMotors();
                reverseStartDistCm = Odometry::getDistanceMoved();
                escapeRetries++;
                transitionTo(STATE_MUNDUR);
            } else {
                Motors::setMotor(pwmL, pwmR);
            }
            break;
        }

        case STATE_MUNDUR: {
            float reversedCm = Odometry::getDistanceMoved() - reverseStartDistCm;
            if (reversedCm >= JARAK_MUNDUR_CM) {
                Motors::stopMotors();
                ObstacleAvoidance::startScan(pose.theta);
                transitionTo(STATE_PINDAI);
            } else {
                Motors::setMotor(-PWM_SLOW, -PWM_SLOW);
            }
            break;
        }

        case STATE_TIBA: {
            handleArrivalSignaling();
            break;
        }

        case STATE_GAGAL: {
            handleFailsafeSignaling();
            break;
        }
    }

    streamTelemetry(pose);
}
