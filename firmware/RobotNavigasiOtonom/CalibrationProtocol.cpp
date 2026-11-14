#include "CalibrationProtocol.h"
#include "Motors.h"
#include "Sensors.h"
#include "Odometry.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

char CalibrationProtocol::rxBuffer[96];
size_t CalibrationProtocol::rxIndex = 0;
bool CalibrationProtocol::inFrame = false;
CalibrationProfile CalibrationProtocol::activeProfile;
CalibrationProtocol::RoomSelectHandler CalibrationProtocol::activeRoomHandler = NULL;

#ifndef ARDUINO
MockMotionData CalibrationProtocol::mockMotion;
#endif

void CalibrationProtocol::resetBuffer() {
    rxIndex = 0;
    inFrame = false;
    rxBuffer[0] = '\0';
}

bool CalibrationProtocol::isInFrame() {
    return inFrame;
}

void CalibrationProtocol::init(RoomSelectHandler roomHandler) {
    resetBuffer();
    activeRoomHandler = roomHandler;

    if (!activeProfile.load()) {
        activeProfile.resetToDefaults();
    }
    Motors::setInversion(activeProfile.getInvertMotorKiri(), activeProfile.getInvertMotorKanan());
    Motors::setTrim(activeProfile.getTrimMotorKiri(), activeProfile.getTrimMotorKanan());

#ifndef ARDUINO
    mockMotion.reset();
#endif
}

CalibrationProfile& CalibrationProtocol::getActiveProfile() {
    return activeProfile;
}

#ifndef ARDUINO
void CalibrationProtocol::setMockMotionPulses(long left, long right) {
    mockMotion.pulsesL = left;
    mockMotion.pulsesR = right;
}

void CalibrationProtocol::setMockMotionDist(float dist) {
    mockMotion.dist = dist;
}

void CalibrationProtocol::setMockMotionTheta(float theta) {
    mockMotion.theta = theta;
}

void CalibrationProtocol::setMockMotionHeadingError(float err) {
    mockMotion.headingError = err;
}
#endif

bool CalibrationProtocol::feedChar(char c, char* frameOut, size_t maxLen, RoomSelectHandler roomHandler) {
    RoomSelectHandler handler = (roomHandler != NULL) ? roomHandler : activeRoomHandler;

    if (!inFrame) {
        if (c == '!') {
            inFrame = true;
            rxIndex = 0;
            rxBuffer[rxIndex++] = c;
            rxBuffer[rxIndex] = '\0';
            return false;
        } else if (c >= '1' && c <= '4') {
            if (handler != NULL) {
                handler((uint8_t)(c - '0'));
            }
            return false;
        }
        return false;
    }

    // Inside frame
    if (c == '\n') {
        rxBuffer[rxIndex] = '\0';
        if (frameOut != NULL && maxLen > 0) {
            snprintf(frameOut, maxLen, "%s", rxBuffer);
        }
        resetBuffer();
        return true;
    }

    if (c == '\r') {
        // Ignore carriage return
        return false;
    }

    if (rxIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[rxIndex++] = c;
        rxBuffer[rxIndex] = '\0';
    } else {
        // Buffer overflow protection: drop corrupt frame
        resetBuffer();
    }

    return false;
}

bool CalibrationProtocol::executeCommand(const char* cmd, char* response, size_t responseSize) {
    if (cmd == NULL || cmd[0] != '!') {
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!ERR:MALFORMED");
        }
        return true;
    }

    // 1. Connection Handshake
    if (strcmp(cmd, "!PING") == 0) {
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!PONG");
        }
        return true;
    }

    // 2. Hardware Diagnostic Probe (U0.1 - U0.3)
    if (strcmp(cmd, "!DIAG:ALL") == 0) {
        Sensors::updateUltrasonic();
        uint8_t mpuOk = Sensors::isMpuConnected() ? 1 : 0;
        float dist = Sensors::getDistance();
        uint8_t irL = Sensors::isWallLeft() ? 1 : 0;
        uint8_t irR = Sensors::isWallRight() ? 1 : 0;
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!DIAG:RES:%u:%.2f:%u:%u",
                     mpuOk, dist, irL, irR);
        }
        return true;
    }

    // 3. Motor Spin Diagnostic (U0.5)
    if (strncmp(cmd, "!DIAG:MOTOR:", 12) == 0) {
        int pwmL = 0, pwmR = 0;
        unsigned long duration = 0;
        if (sscanf(cmd + 12, "%d:%d:%lu", &pwmL, &pwmR, &duration) == 3) {
            Odometry::resetPulses();
            Motors::setMotor(pwmL, pwmR);
#ifdef ARDUINO
            uint32_t tStart = Hal::millis();
            while (Hal::millis() - tStart < duration) {
                // Spin motors for duration
            }
#else
            Hal::advanceTimeMs(duration);
            if (mockMotion.pulsesL >= 0 && mockMotion.pulsesR >= 0) {
                Odometry::injectPulses(mockMotion.pulsesL, mockMotion.pulsesR);
                mockMotion.pulsesL = -1;
                mockMotion.pulsesR = -1;
            }
#endif
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

    // 4. Indicator / Buzzer Test (U0.4)
    if (strncmp(cmd, "!DIAG:BEEP", 10) == 0) {
        unsigned long dur = 200;
        if (cmd[10] == ':') {
            sscanf(cmd + 11, "%lu", &dur);
        }
        Hal::digitalWrite(PIN_BUZZER, HIGH);
        Hal::digitalWrite(PIN_LED, HIGH);
#ifdef ARDUINO
        uint32_t tStart = Hal::millis();
        while (Hal::millis() - tStart < dur) {
            // Keep active
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

    // 5. Calibration Step U1 (100 cm forward run)
    if (strcmp(cmd, "!STEP:U1") == 0) {
        Odometry::resetPose(0.0f, 0.0f, 0.0f);
        Odometry::resetPulses();
#ifdef ARDUINO
        Motors::setMotor(140, 140);
        uint32_t tStart = Hal::millis();
        uint32_t lastStepMs = tStart;
        uint32_t lastMotionMs = tStart;
        long lastPulses = Odometry::getTotalPulseCount();

        while (Odometry::getDistanceMoved() < 100.0f && (Hal::millis() - tStart < 8000UL)) {
            uint32_t now = Hal::millis();
            if (now - lastStepMs >= 20) {
                lastStepMs = now;
                Odometry::update(0.02f);
                long currentPulses = Odometry::getTotalPulseCount();
                if (currentPulses != lastPulses) {
                    lastPulses = currentPulses;
                    lastMotionMs = now;
                } else if (now - lastMotionMs >= 1000UL) {
                    // Cumulative stall detection failsafe
                    break;
                }
            }
        }
        Motors::stopMotors();
        long pL = Odometry::getPulsaKiri();
        long pR = Odometry::getPulsaKanan();
        float dist = Odometry::getDistanceMoved();
#else
        long pL = (mockMotion.pulsesL >= 0) ? mockMotion.pulsesL : 98;
        long pR = (mockMotion.pulsesR >= 0) ? mockMotion.pulsesR : 99;
        float dist = (mockMotion.dist >= 0.0f) ? mockMotion.dist : 100.12f;
        mockMotion.pulsesL = -1;
        mockMotion.pulsesR = -1;
        mockMotion.dist = -1.0f;
#endif
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!U1:DONE:%ld:%ld:%.2f", pL, pR, dist);
        }
        return true;
    }

    // 6. Calibration Step U2 (In-place rotation)
    if (strncmp(cmd, "!STEP:U2:", 9) == 0) {
        float targetAngle = 90.0f;
        sscanf(cmd + 9, "%f", &targetAngle);
        Odometry::resetPose(0.0f, 0.0f, 0.0f);
#ifdef ARDUINO
        int pTurn = (targetAngle > 0) ? 140 : -140;
        Motors::setMotor(-pTurn, pTurn);
        uint32_t tStart = Hal::millis();
        uint32_t lastStepMs = tStart;
        uint32_t lastMotionMs = tStart;
        long lastPulses = Odometry::getTotalPulseCount();

        while (fabs(Odometry::getTheta()) < fabs(targetAngle) && (Hal::millis() - tStart < 5000UL)) {
            uint32_t now = Hal::millis();
            if (now - lastStepMs >= 20) {
                lastStepMs = now;
                Odometry::update(0.02f);
                long currentPulses = Odometry::getTotalPulseCount();
                if (currentPulses != lastPulses) {
                    lastPulses = currentPulses;
                    lastMotionMs = now;
                } else if (now - lastMotionMs >= 1000UL) {
                    break;
                }
            }
        }
        Motors::stopMotors();
        float finalTheta = Odometry::getTheta();
#else
        float finalTheta = (mockMotion.theta > -900.0f) ? mockMotion.theta : targetAngle;
        mockMotion.theta = -999.0f;
#endif
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!U2:DONE:%.2f", finalTheta);
        }
        return true;
    }

    // 7. Calibration Step U3 (150 cm straight line run with gyro heading hold)
    if (strcmp(cmd, "!STEP:U3") == 0) {
        Odometry::resetPose(0.0f, 0.0f, 0.0f);
        Odometry::resetPulses();
#ifdef ARDUINO
        float maxErr = 0.0f;
        uint32_t tStart = Hal::millis();
        uint32_t lastStepMs = tStart;
        uint32_t lastMotionMs = tStart;
        long lastPulses = Odometry::getTotalPulseCount();

        while (Odometry::getDistanceMoved() < 150.0f && (Hal::millis() - tStart < 12000UL)) {
            uint32_t now = Hal::millis();
            if (now - lastStepMs >= 20) {
                lastStepMs = now;
                float th = Odometry::getTheta();
                if (fabs(th) > maxErr) maxErr = fabs(th);
                int corr = (int)(th * 3.0f);
                Motors::setMotor(140 - corr, 140 + corr);
                Odometry::update(0.02f);
                long currentPulses = Odometry::getTotalPulseCount();
                if (currentPulses != lastPulses) {
                    lastPulses = currentPulses;
                    lastMotionMs = now;
                } else if (now - lastMotionMs >= 1000UL) {
                    break;
                }
            }
        }
        Motors::stopMotors();
        long pL = Odometry::getPulsaKiri();
        long pR = Odometry::getPulsaKanan();
#else
        long pL = (mockMotion.pulsesL >= 0) ? mockMotion.pulsesL : 147;
        long pR = (mockMotion.pulsesR >= 0) ? mockMotion.pulsesR : 148;
        float maxErr = (mockMotion.headingError >= 0.0f) ? mockMotion.headingError : 1.85f;
        mockMotion.pulsesL = -1;
        mockMotion.pulsesR = -1;
        mockMotion.headingError = -1.0f;
#endif
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!U3:DONE:%ld:%ld:%.2f", pL, pR, maxErr);
        }
        return true;
    }

    // 8. Calibration Step U4 (Ultrasonic distance reading)
    if (strcmp(cmd, "!STEP:U4") == 0) {
        Sensors::updateUltrasonic();
        float dist = Sensors::getDistance();
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!U4:DONE:%.2f", dist);
        }
        return true;
    }

    // 9. Profile Read
    if (strcmp(cmd, "!CALIB:READ") == 0) {
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!CALIB:DATA:%.2f:%.3f:%.3f:%.3f:%d:%d",
                     activeProfile.getDiameterRodaMm(),
                     activeProfile.getTrimMotorKiri(),
                     activeProfile.getTrimMotorKanan(),
                     activeProfile.getGyroBiasZ(),
                     activeProfile.getInvertMotorKiri(),
                     activeProfile.getInvertMotorKanan());
        }
        return true;
    }

    // 10. Profile Write
    if (strncmp(cmd, "!CALIB:WRITE:", 13) == 0) {
        float diam = 0.0f, trimL = 0.0f, trimR = 0.0f, biasZ = 0.0f;
        int invL = 1, invR = 1;
        int parsed = sscanf(cmd + 13, "%f:%f:%f:%f:%d:%d",
                            &diam, &trimL, &trimR, &biasZ, &invL, &invR);
        if (parsed == 6) {
            if (diam < 50.0f || diam > 100.0f ||
                trimL < 0.70f || trimL > 1.30f ||
                trimR < 0.70f || trimR > 1.30f ||
                biasZ < -10.0f || biasZ > 10.0f ||
                (invL != 1 && invL != -1) ||
                (invR != 1 && invR != -1)) {
                if (response != NULL && responseSize > 0) {
                    snprintf(response, responseSize, "!ERR:INVALID");
                }
                return true;
            }

            activeProfile.setDiameterRodaMm(diam);
            activeProfile.setTrimMotorKiri(trimL);
            activeProfile.setTrimMotorKanan(trimR);
            activeProfile.setGyroBiasZ(biasZ);
            activeProfile.setInvertMotorKiri((int8_t)invL);
            activeProfile.setInvertMotorKanan((int8_t)invR);

            if (activeProfile.save()) {
                Motors::setInversion((int8_t)invL, (int8_t)invR);
                Motors::setTrim(trimL, trimR);
                if (response != NULL && responseSize > 0) {
                    snprintf(response, responseSize, "!OK");
                }
            } else {
                if (response != NULL && responseSize > 0) {
                    snprintf(response, responseSize, "!ERR:SAVE_FAILED");
                }
            }
            return true;
        } else {
            if (response != NULL && responseSize > 0) {
                snprintf(response, responseSize, "!ERR:PARAM");
            }
            return true;
        }
    }

    // 11. Profile Reset: invalidates magic bytes in EEPROM and returns !RESET:OK
    if (strcmp(cmd, "!CALIB:RESET") == 0) {
        activeProfile.clearEeprom();
        Motors::setInversion(1, 1);
        Motors::setTrim(TRIM_MOTOR_KIRI, TRIM_MOTOR_KANAN);
        if (response != NULL && responseSize > 0) {
            snprintf(response, responseSize, "!RESET:OK");
        }
        return true;
    }

    if (response != NULL && responseSize > 0) {
        snprintf(response, responseSize, "!ERR:UNKNOWN");
    }
    return true;
}

void CalibrationProtocol::update() {
    while (Hal::serialAvailable()) {
        char c = (char)Hal::serialRead();
        char frame[96];
        if (feedChar(c, frame, sizeof(frame))) {
            char resp[128];
            if (executeCommand(frame, resp, sizeof(resp))) {
                Hal::serialPrintln(resp);
            }
        }
    }
}
