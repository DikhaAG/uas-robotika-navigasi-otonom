#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include "../firmware/RobotNavigasiOtonom/Config.h"
#include "../firmware/RobotNavigasiOtonom/Hal.h"
#include "../firmware/RobotNavigasiOtonom/Motors.h"
#include "../firmware/RobotNavigasiOtonom/Sensors.h"
#include "../firmware/RobotNavigasiOtonom/Odometry.h"
#include "../firmware/RobotNavigasiOtonom/Waypoints.h"
#include "../firmware/RobotNavigasiOtonom/ObstacleAvoidance.h"
#include "../firmware/RobotNavigasiOtonom/NavigationFSM.h"
#include "../firmware/RobotNavigasiOtonom/CalibrationProfile.h"
#include "../firmware/RobotNavigasiOtonom/CalibrationProtocol.h"

// Canonical test fixture: cleans all global mock state before each test case
inline void resetAllMocks() {
    Hal::resetMock();
    Motors::init();
    Sensors::init();
    Odometry::init();
    ObstacleAvoidance::reset();
    CalibrationProtocol::init();
    NavigationFSM::init();
}

#endif // TEST_COMMON_H
