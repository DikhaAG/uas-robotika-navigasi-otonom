#include "CalibrationService.h"
#include "Gyroscope.h"
#include "UltrasonicSonar.h"
#include "WallDetector.h"
#include "FloatText.h"
#include "TripRecorder.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

CalibrationProfile CalibrationService::activeProfile;

void CalibrationService::init() {
    if (!activeProfile.load()) {
        activeProfile.resetToDefaults();
    }
    Motors::setInversion(activeProfile.getInvertMotorKiri(), activeProfile.getInvertMotorKanan());
    Motors::setTrim(activeProfile.getTrimMotorKiri(), activeProfile.getTrimMotorKanan());
    Odometry::setWheelDiameterMm(activeProfile.getDiameterRodaMm());
}

CalibrationProfile& CalibrationService::getActiveProfile() {
    return activeProfile;
}

void CalibrationService::update() {
    while (Hal::serialAvailable() > 0) {
        char c = (char)Hal::serialRead();
        char frame[96];
        if (AsciiProtocolDecoder::feedChar(c, frame, sizeof(frame))) {
            char response[96];
            response[0] = '\0';
            if (executeCommand(frame, response, sizeof(response))) {
                if (response[0] != '\0') {
                    Hal::serialPrintln(response);
                }
            }
        }
    }
}

bool CalibrationService::stepCadence(uint32_t now, uint32_t& lastStepMs, uint32_t& lastMotionMs, long& lastPulses) {
    (void)now;
    (void)lastStepMs;

#ifndef ARDUINO
    // Desktop digital twin stepping: advance time and compute real motor kinematics
    int dirL = Motors::getDirectionLeft();
    int dirR = Motors::getDirectionRight();
    int pwmL = (int)Hal::getPwm(PIN_ENA) * dirL;
    int pwmR = (int)Hal::getPwm(PIN_ENB) * dirR;
    float vL = (float)pwmL * 0.165f;
    float vR = (float)pwmR * 0.165f;
    float omegaRad = (vR - vL) / TRACK_WIDTH_CM;
    float omegaDeg = omegaRad * (180.0f / PI);
    Gyroscope::setMockZRate(omegaDeg);

    float dsL = fabsf(vL * 0.02f);
    float dsR = fabsf(vR * 0.02f);
    Odometry::injectPulses((long)(dsL / CM_PER_PULSA), (long)(dsR / CM_PER_PULSA));
    Hal::advanceTimeMs(20);
#endif

    Odometry::update(0.02f);
    uint32_t tCurrent = Hal::millis();
    long curPulses = Odometry::getTotalPulseCount();
    if (curPulses != lastPulses) {
        lastPulses = curPulses;
        lastMotionMs = tCurrent;
    } else if (tCurrent - lastMotionMs >= 1000UL) {
        return true; // Stall watchdog tripped
    }
    return false;
}

bool CalibrationService::handlePing(char* response, size_t responseSize) {
    if (response != NULL && responseSize > 0) {
        snprintf(response, responseSize, "!PONG");
    }
    return true;
}

bool CalibrationService::handleDiagAll(char* response, size_t responseSize) {
    UltrasonicSonar::update();
    uint8_t mpuOk = Gyroscope::isConnected() ? 1 : 0;
    float dist = UltrasonicSonar::getDistance();
    uint8_t irL = WallDetector::isWallLeft() ? 1 : 0;
    uint8_t irR = WallDetector::isWallRight() ? 1 : 0;

    if (response != NULL && responseSize > 0) {
        char distText[FloatText::BUFFER_SIZE];
        snprintf(response, responseSize, "!DIAG:RES:%u:%s:%u:%u",
                 mpuOk, FloatText::format(distText, sizeof(distText), dist, 2), irL, irR);
    }
    return true;
}

bool CalibrationService::handleDiagMotor(const char* args, char* response, size_t responseSize) {
    int pwmL = 0, pwmR = 0;
    unsigned long duration = 0;
    if (sscanf(args, "%d:%d:%lu", &pwmL, &pwmR, &duration) == 3) {
        Odometry::resetPulses();
        Motors::setMotor(pwmL, pwmR);

        uint32_t tStart = Hal::millis();
        uint32_t lastStepMs = tStart;
        uint32_t lastMotionMs = tStart;
        long lastPulses = Odometry::getTotalPulseCount();

        while (Hal::millis() - tStart < duration) {
#ifdef ARDUINO
            uint32_t now = Hal::millis();
            if (now - lastStepMs >= 20) {
                lastStepMs = now;
                stepCadence(now, lastStepMs, lastMotionMs, lastPulses);
            }
#else
            stepCadence(0, lastStepMs, lastMotionMs, lastPulses);
#endif
        }
        Motors::stopMotors();
        long pL = Odometry::getPulsaKiri();
        long pR = Odometry::getPulsaKanan();
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!PULSES:%ld:%ld", pL, pR);
        }
        return true;
    } else {
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!ERR:PARAM");
        }
        return true;
    }
}

bool CalibrationService::handleDiagBeep(const char* args, char* response, size_t responseSize) {
    unsigned long dur = 200;
    if (args != NULL && args[0] == ':') {
        sscanf(args + 1, "%lu", &dur);
    }
    Hal::digitalWrite(PIN_BUZZER, HIGH);
    Hal::digitalWrite(PIN_LED, HIGH);
#ifdef ARDUINO
    uint32_t tStart = Hal::millis();
    while (Hal::millis() - tStart < dur) {
        // Active buzzer
    }
#else
    Hal::advanceTimeMs(dur);
#endif
    Hal::digitalWrite(PIN_BUZZER, LOW);
    Hal::digitalWrite(PIN_LED, LOW);
    if (response != NULL && responseSize > 0) {
        snprintf(response, responseSize, "!OK");
    }
    return true;
}

bool CalibrationService::handleStepU1(char* response, size_t responseSize) {
    Odometry::resetPose(0.0f, 0.0f, 0.0f);
    Odometry::resetPulses();
    Motors::setMotor(140, 140);

    uint32_t tStart = Hal::millis();
    uint32_t lastStepMs = tStart;
    uint32_t lastMotionMs = tStart;
    long lastPulses = Odometry::getTotalPulseCount();

    while (Odometry::getDistanceMoved() < 100.0f && (Hal::millis() - tStart < 8000UL)) {
#ifdef ARDUINO
        uint32_t now = Hal::millis();
        if (now - lastStepMs >= 20) {
            lastStepMs = now;
            if (stepCadence(now, lastStepMs, lastMotionMs, lastPulses)) {
                break;
            }
        }
#else
        if (stepCadence(0, lastStepMs, lastMotionMs, lastPulses)) {
            break;
        }
#endif
    }
    Motors::stopMotors();
    long pL = Odometry::getPulsaKiri();
    long pR = Odometry::getPulsaKanan();
    float dist = Odometry::getDistanceMoved();

    if (response != NULL && responseSize > 0) {
        char distText[FloatText::BUFFER_SIZE];
        snprintf(response, responseSize, "!U1:DONE:%ld:%ld:%s", pL, pR,
                 FloatText::format(distText, sizeof(distText), dist, 2));
    }
    return true;
}

bool CalibrationService::handleStepU2(const char* args, char* response, size_t responseSize) {
    float targetAngle = 90.0f;
    if (args != NULL && args[0] == ':') {
        const char* cursor = args + 1;
        FloatText::parseFloatField(cursor, '\0', targetAngle);  // keeps 90 deg if unparseable
    }
    Odometry::resetPose(0.0f, 0.0f, 0.0f);

    uint32_t tStart = Hal::millis();
    uint32_t lastStepMs = tStart;
    uint32_t lastMotionMs = tStart;
    long lastPulses = Odometry::getTotalPulseCount();

    while (Hal::millis() - tStart < 5000UL) {
        float currentTheta = Odometry::getTheta();
        int pwmL = 0, pwmR = 0;
        if (MotionController::rotateTowards(currentTheta, targetAngle, pwmL, pwmR)) {
            Motors::stopMotors();
            break;
        }
        Motors::setMotor(pwmL, pwmR);

#ifdef ARDUINO
        uint32_t now = Hal::millis();
        if (now - lastStepMs >= 20) {
            lastStepMs = now;
            if (stepCadence(now, lastStepMs, lastMotionMs, lastPulses)) {
                break;
            }
        }
#else
        if (stepCadence(0, lastStepMs, lastMotionMs, lastPulses)) {
            break;
        }
#endif
    }
    Motors::stopMotors();
    float finalTheta = Odometry::getTheta();

    if (response != NULL && responseSize > 0) {
        char thetaText[FloatText::BUFFER_SIZE];
        snprintf(response, responseSize, "!U2:DONE:%s",
                 FloatText::format(thetaText, sizeof(thetaText), finalTheta, 2));
    }
    return true;
}

bool CalibrationService::handleStepU3(char* response, size_t responseSize) {
    Odometry::resetPose(0.0f, 0.0f, 0.0f);
    Odometry::resetPulses();

    float maxErr = 0.0f;
    uint32_t tStart = Hal::millis();
    uint32_t lastStepMs = tStart;
    uint32_t lastMotionMs = tStart;
    long lastPulses = Odometry::getTotalPulseCount();

    while (Odometry::getDistanceMoved() < 150.0f && (Hal::millis() - tStart < 12000UL)) {
        Pose pose = Odometry::getPose();
        float th = pose.theta;
        if (fabsf(th) > maxErr) maxErr = fabsf(th);

        MotionCommand cmd = MotionController::driveTowards(
            pose.x, pose.y, pose.theta,
            150.0f, 0.0f,
            false, false,
            TOL_WP_CM
        );
        Motors::setMotor(cmd.pwmL, cmd.pwmR);

#ifdef ARDUINO
        uint32_t now = Hal::millis();
        if (now - lastStepMs >= 20) {
            lastStepMs = now;
            if (stepCadence(now, lastStepMs, lastMotionMs, lastPulses)) {
                break;
            }
        }
#else
        if (stepCadence(0, lastStepMs, lastMotionMs, lastPulses)) {
            break;
        }
#endif
    }
    Motors::stopMotors();
    long pL = Odometry::getPulsaKiri();
    long pR = Odometry::getPulsaKanan();

    if (response != NULL && responseSize > 0) {
        char errText[FloatText::BUFFER_SIZE];
        snprintf(response, responseSize, "!U3:DONE:%ld:%ld:%s", pL, pR,
                 FloatText::format(errText, sizeof(errText), maxErr, 2));
    }
    return true;
}

bool CalibrationService::handleStepU4(char* response, size_t responseSize) {
    UltrasonicSonar::update();
    float dist = UltrasonicSonar::getDistance();
    if (response != NULL && responseSize > 0) {
        char distText[FloatText::BUFFER_SIZE];
        snprintf(response, responseSize, "!U4:DONE:%s",
                 FloatText::format(distText, sizeof(distText), dist, 2));
    }
    return true;
}

bool CalibrationService::handleCalibRead(char* response, size_t responseSize) {
    if (response != NULL && responseSize > 0) {
        char diamText[FloatText::BUFFER_SIZE];
        char trimLText[FloatText::BUFFER_SIZE];
        char trimRText[FloatText::BUFFER_SIZE];
        char biasText[FloatText::BUFFER_SIZE];
        snprintf(response, responseSize, "!CALIB:DATA:%s:%s:%s:%s:%d:%d",
                 FloatText::format(diamText, sizeof(diamText), activeProfile.getDiameterRodaMm(), 2),
                 FloatText::format(trimLText, sizeof(trimLText), activeProfile.getTrimMotorKiri(), 3),
                 FloatText::format(trimRText, sizeof(trimRText), activeProfile.getTrimMotorKanan(), 3),
                 FloatText::format(biasText, sizeof(biasText), activeProfile.getGyroBiasZ(), 3),
                 activeProfile.getInvertMotorKiri(),
                 activeProfile.getInvertMotorKanan());
    }
    return true;
}

bool CalibrationService::handleCalibWrite(const char* args, char* response, size_t responseSize) {
    float diam = 0.0f, trimL = 0.0f, trimR = 0.0f, biasZ = 0.0f;
    long invL = 1, invR = 1;
    const char* cursor = args;

    if (FloatText::parseFloatField(cursor, ':', diam) &&
        FloatText::parseFloatField(cursor, ':', trimL) &&
        FloatText::parseFloatField(cursor, ':', trimR) &&
        FloatText::parseFloatField(cursor, ':', biasZ) &&
        FloatText::parseLongField(cursor, ':', invL) &&
        FloatText::parseLongField(cursor, '\0', invR)) {
        CalibrationProfile candidate;
        candidate.setDiameterRodaMm(diam);
        candidate.setTrimMotorKiri(trimL);
        candidate.setTrimMotorKanan(trimR);
        candidate.setGyroBiasZ(biasZ);
        candidate.setInvertMotorKiri((int8_t)invL);
        candidate.setInvertMotorKanan((int8_t)invR);

        if (candidate.isValid()) {
            activeProfile = candidate;
            activeProfile.save();
            Motors::setInversion(activeProfile.getInvertMotorKiri(), activeProfile.getInvertMotorKanan());
            Motors::setTrim(activeProfile.getTrimMotorKiri(), activeProfile.getTrimMotorKanan());
            Odometry::setWheelDiameterMm(activeProfile.getDiameterRodaMm());

            if (response != NULL && responseSize > 0) {
                snprintf(response, responseSize, "!OK");
            }
            return true;
        } else {
            if (response != NULL && responseSize > 0) {
                snprintf(response, responseSize, "!ERR:INVALID");
            }
            return true;
        }
    } else {
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!ERR:PARAM");
        }
        return true;
    }
}

bool CalibrationService::handleCalibReset(char* response, size_t responseSize) {
    activeProfile.clearEeprom();
    activeProfile.resetToDefaults();
    Motors::setInversion(1, 1);
    Motors::setTrim(TRIM_MOTOR_KIRI, TRIM_MOTOR_KANAN);
    Odometry::setWheelDiameterMm(DIAMETER_RODA_MM);

    if (response != NULL && responseSize > 0) {
        snprintf(response, responseSize, "!RESET:OK");
    }
    return true;
}

bool CalibrationService::handleTripRead(char* response, size_t responseSize) {
    long pulsesLeft = 0, pulsesRight = 0;
    if (response != NULL && responseSize > 0) {
        if (TripRecorder::readSaved(pulsesLeft, pulsesRight)) {
            snprintf(response, responseSize, "!TRIP:%ld:%ld", pulsesLeft, pulsesRight);
        } else {
            snprintf(response, responseSize, "!TRIP:NONE");
        }
    }
    return true;
}

bool CalibrationService::executeCommand(const char* cmd, char* response, size_t responseSize) {
    if (cmd == NULL || cmd[0] != '!') {
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!ERR:MALFORMED");
        }
        return true;
    }

    if (strcmp(cmd, "!PING") == 0) {
        return handlePing(response, responseSize);
    }
    if (strcmp(cmd, "!DIAG:ALL") == 0) {
        return handleDiagAll(response, responseSize);
    }
    if (strncmp(cmd, "!DIAG:MOTOR:", 12) == 0) {
        return handleDiagMotor(cmd + 12, response, responseSize);
    }
    if (strncmp(cmd, "!DIAG:BEEP", 10) == 0) {
        return handleDiagBeep(cmd + 10, response, responseSize);
    }
    if (strcmp(cmd, "!STEP:U1") == 0) {
        return handleStepU1(response, responseSize);
    }
    if (strncmp(cmd, "!STEP:U2", 8) == 0) {
        return handleStepU2(cmd + 8, response, responseSize);
    }
    if (strcmp(cmd, "!STEP:U3") == 0) {
        return handleStepU3(response, responseSize);
    }
    if (strcmp(cmd, "!STEP:U4") == 0) {
        return handleStepU4(response, responseSize);
    }
    if (strcmp(cmd, "!CALIB:READ") == 0) {
        return handleCalibRead(response, responseSize);
    }
    if (strncmp(cmd, "!CALIB:WRITE:", 13) == 0) {
        return handleCalibWrite(cmd + 13, response, responseSize);
    }
    if (strcmp(cmd, "!CALIB:RESET") == 0) {
        return handleCalibReset(response, responseSize);
    }
    if (strcmp(cmd, "!TRIP:READ") == 0) {
        return handleTripRead(response, responseSize);
    }

    if (response != NULL && responseSize > 0) {
        snprintf(response, responseSize, "!ERR:UNKNOWN");
    }
    return true;
}
