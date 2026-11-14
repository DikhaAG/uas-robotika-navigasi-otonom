#ifndef CALIBRATION_PROTOCOL_H
#define CALIBRATION_PROTOCOL_H

#include "Config.h"
#include "Hal.h"
#include "CalibrationProfile.h"
#include <stddef.h>
#include <stdint.h>

#ifndef ARDUINO
struct MockMotionData {
    long pulsesL;
    long pulsesR;
    float dist;
    float theta;
    float headingError;

    void reset() {
        pulsesL = -1;
        pulsesR = -1;
        dist = -1.0f;
        theta = -999.0f;
        headingError = -1.0f;
    }
};
#endif

class CalibrationProtocol {
public:
    typedef void (*RoomSelectHandler)(uint8_t room);

    static void init(RoomSelectHandler roomHandler = NULL);
    static bool feedChar(char c, char* frameOut, size_t maxLen, RoomSelectHandler roomHandler = NULL);
    static bool executeCommand(const char* cmd, char* response, size_t responseSize);
    static void update();
    static void resetBuffer();
    static bool isInFrame();

    static CalibrationProfile& getActiveProfile();

#ifndef ARDUINO
    static void setMockMotionPulses(long left, long right);
    static void setMockMotionDist(float dist);
    static void setMockMotionTheta(float theta);
    static void setMockMotionHeadingError(float err);
#endif

private:
    static char rxBuffer[96];
    static size_t rxIndex;
    static bool inFrame;
    static CalibrationProfile activeProfile;
    static RoomSelectHandler activeRoomHandler;

#ifndef ARDUINO
    static MockMotionData mockMotion;
#endif
};

#endif // CALIBRATION_PROTOCOL_H
