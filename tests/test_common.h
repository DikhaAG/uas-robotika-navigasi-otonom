#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include "../firmware/RobotNavigasiOtonom/Config.h"
#include "../firmware/RobotNavigasiOtonom/Hal.h"
#include "../firmware/RobotNavigasiOtonom/Motors.h"
#include "../firmware/RobotNavigasiOtonom/Gyroscope.h"
#include "../firmware/RobotNavigasiOtonom/UltrasonicSonar.h"
#include "../firmware/RobotNavigasiOtonom/WallDetector.h"
#include "../firmware/RobotNavigasiOtonom/Odometry.h"
#include "../firmware/RobotNavigasiOtonom/WaypointQueue.h"
#include "../firmware/RobotNavigasiOtonom/MotionController.h"
#include "../firmware/RobotNavigasiOtonom/ObstacleAvoidance.h"
#include "../firmware/RobotNavigasiOtonom/NavigationFSM.h"
#include "../firmware/RobotNavigasiOtonom/CalibrationProfile.h"
#include "../firmware/RobotNavigasiOtonom/AsciiProtocolDecoder.h"
#include "../firmware/RobotNavigasiOtonom/CalibrationService.h"

// Canonical test fixture: cleans all global mock state before each test case
inline void resetAllMocks() {
    Hal::resetMock();
    Motors::init();
    Gyroscope::init();
    UltrasonicSonar::init();
    WallDetector::init();
    Odometry::init();
    WaypointQueue::init();
    ObstacleAvoidance::reset();
    AsciiProtocolDecoder::init();
    CalibrationService::init();
    NavigationFSM::init();
}

#endif // TEST_COMMON_H
