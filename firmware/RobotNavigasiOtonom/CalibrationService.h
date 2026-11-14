#ifndef CALIBRATION_SERVICE_H
#define CALIBRATION_SERVICE_H

#include "Config.h"
#include "Hal.h"
#include "CalibrationProfile.h"
#include "Motors.h"
#include "Odometry.h"
#include "MotionController.h"
#include "AsciiProtocolDecoder.h"
#include <stddef.h>
#include <stdint.h>

class CalibrationService {
public:
    static void init();
    static void update();
    static bool executeCommand(const char* cmd, char* response, size_t responseSize);

    static CalibrationProfile& getActiveProfile();

private:
    static CalibrationProfile activeProfile;

    // Cadence & stall watchdog helper for kinematic steps
    static bool stepCadence(uint32_t now, uint32_t& lastStepMs, uint32_t& lastMotionMs, long& lastPulses);

    // Diagnostics
    static bool handlePing(char* response, size_t responseSize);
    static bool handleDiagAll(char* response, size_t responseSize);
    static bool handleDiagMotor(const char* args, char* response, size_t responseSize);
    static bool handleDiagBeep(const char* args, char* response, size_t responseSize);

    // Kinematic Steps
    static bool handleStepU1(char* response, size_t responseSize);
    static bool handleStepU2(const char* args, char* response, size_t responseSize);
    static bool handleStepU3(char* response, size_t responseSize);
    static bool handleStepU4(char* response, size_t responseSize);

    // Profile Management
    static bool handleCalibRead(char* response, size_t responseSize);
    static bool handleCalibWrite(const char* args, char* response, size_t responseSize);
    static bool handleCalibReset(char* response, size_t responseSize);
};

#endif // CALIBRATION_SERVICE_H
