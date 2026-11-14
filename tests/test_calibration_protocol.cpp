#include <cassert>
#include <iostream>
#include <cstring>
#include <cmath>
#include "test_common.h"
#include "../firmware/RobotNavigasiOtonom/CalibrationProtocol.h"

static bool approxEqual(float a, float b, float tol = 0.05f) {
    return std::fabs(a - b) <= tol;
}

int main() {
    std::cout << "[TDD] Running test_calibration_protocol..." << std::endl;
    resetAllMocks();

    CalibrationProtocol::init();

    // ----------------------------------------------------
    // Test 1: Frame accumulator (feedChar)
    // ----------------------------------------------------
    char frame[64];
    memset(frame, 0, sizeof(frame));

    // Noise before '!' should be discarded
    assert(!CalibrationProtocol::feedChar('x', frame, sizeof(frame)));
    assert(!CalibrationProtocol::feedChar('y', frame, sizeof(frame)));
    assert(!CalibrationProtocol::feedChar('\n', frame, sizeof(frame)));

    // Feed "!PING\n"
    assert(!CalibrationProtocol::feedChar('!', frame, sizeof(frame)));
    assert(!CalibrationProtocol::feedChar('P', frame, sizeof(frame)));
    assert(!CalibrationProtocol::feedChar('I', frame, sizeof(frame)));
    assert(!CalibrationProtocol::feedChar('N', frame, sizeof(frame)));
    assert(!CalibrationProtocol::feedChar('G', frame, sizeof(frame)));
    assert(!CalibrationProtocol::feedChar('\r', frame, sizeof(frame)));
    bool completed = CalibrationProtocol::feedChar('\n', frame, sizeof(frame));
    assert(completed);
    assert(strcmp(frame, "!PING") == 0);
    std::cout << "  ✓ Test 1: Frame accumulator (feedChar) passed." << std::endl;

    // ----------------------------------------------------
    // Test 2: Basic Command Dispatch: !PING -> !PONG and Unknown
    // ----------------------------------------------------
    char response[128];
    memset(response, 0, sizeof(response));

    bool handled = CalibrationProtocol::executeCommand("!PING", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!PONG") == 0);

    handled = CalibrationProtocol::executeCommand("!FOOBAR", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!ERR:UNKNOWN") == 0);

    // Malformed commands without '!'
    handled = CalibrationProtocol::executeCommand("PING", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!ERR:MALFORMED") == 0);

    std::cout << "  ✓ Test 2: Basic Command Dispatch (!PING, !ERR) passed." << std::endl;

    // ----------------------------------------------------
    // Test 3: Diagnostic Queries (!DIAG:ALL, !DIAG:MOTOR, !DIAG:BEEP)
    // ----------------------------------------------------
    Sensors::setMockDistance(25.40f);
    Sensors::setMockIr(false, true); // left false, right true
    Sensors::setMockWhoAmI(0x68);    // MPU6050 OK

    handled = CalibrationProtocol::executeCommand("!DIAG:ALL", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!DIAG:RES:1:25.40:0:1") == 0);

    // If MPU6050 whoAmI != 0x68
    Sensors::setMockWhoAmI(0x00);
    handled = CalibrationProtocol::executeCommand("!DIAG:ALL", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!DIAG:RES:0:25.40:0:1") == 0);
    Sensors::setMockWhoAmI(0x68); // restore

    // Motor Spin Test: !DIAG:MOTOR:<pwmL>:<pwmR>:<duration_ms>
    CalibrationProtocol::setMockMotionPulses(15, 18);
    handled = CalibrationProtocol::executeCommand("!DIAG:MOTOR:150:150:200", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!PULSES:15:18") == 0);

    // Indicator / Buzzer Test: !DIAG:BEEP:100
    handled = CalibrationProtocol::executeCommand("!DIAG:BEEP:100", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!OK") == 0);
    std::cout << "  ✓ Test 3: Diagnostic Queries (!DIAG:ALL, !DIAG:MOTOR, !DIAG:BEEP) passed." << std::endl;

    // ----------------------------------------------------
    // Test 4: Discrete Calibration Steps (!STEP:U1, U2, U3, U4)
    // ----------------------------------------------------
    // !STEP:U1 -> !U1:DONE:<pL>:<pR>:<dist>
    CalibrationProtocol::setMockMotionPulses(98, 99);
    CalibrationProtocol::setMockMotionDist(100.12f);
    handled = CalibrationProtocol::executeCommand("!STEP:U1", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!U1:DONE:98:99:100.12") == 0);

    // !STEP:U2:<angle> -> !U2:DONE:<finalTheta>
    CalibrationProtocol::setMockMotionTheta(90.25f);
    handled = CalibrationProtocol::executeCommand("!STEP:U2:90.0", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!U2:DONE:90.25") == 0);

    // !STEP:U3 -> !U3:DONE:<pL>:<pR>:<maxHeadingError>
    CalibrationProtocol::setMockMotionPulses(147, 148);
    CalibrationProtocol::setMockMotionHeadingError(1.85f);
    handled = CalibrationProtocol::executeCommand("!STEP:U3", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!U3:DONE:147:148:1.85") == 0);

    // !STEP:U4 -> !U4:DONE:<sonarCm>
    Sensors::setMockDistance(20.15f);
    handled = CalibrationProtocol::executeCommand("!STEP:U4", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!U4:DONE:20.15") == 0);
    std::cout << "  ✓ Test 4: Discrete Calibration Steps (!STEP:U1..U4) passed." << std::endl;

    // ----------------------------------------------------
    // Test 5: Profile Management (!CALIB:READ, !CALIB:WRITE, !CALIB:RESET)
    // ----------------------------------------------------
    // Default profile
    handled = CalibrationProtocol::executeCommand("!CALIB:READ", response, sizeof(response));
    assert(handled);
    // Defaults: diam 65.0, trimL 1.0, trimR 1.0, biasZ 0.0, invL 1, invR 1
    assert(strcmp(response, "!CALIB:DATA:65.00:1.000:1.000:0.000:1:1") == 0);

    // Write updated profile
    handled = CalibrationProtocol::executeCommand("!CALIB:WRITE:66.50:0.960:1.040:-0.025:-1:1", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!OK") == 0);

    // Read back to confirm EEPROM persistence
    handled = CalibrationProtocol::executeCommand("!CALIB:READ", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!CALIB:DATA:66.50:0.960:1.040:-0.025:-1:1") == 0);
    assert(Motors::getInvertLeft() == -1);
    assert(Motors::getInvertRight() == 1);
    assert(approxEqual(Motors::getTrimLeft(), 0.960f, 0.005f));
    assert(approxEqual(Motors::getTrimRight(), 1.040f, 0.005f));

    // Reject out-of-range values (e.g. diameter 20mm or trim 2.50)
    handled = CalibrationProtocol::executeCommand("!CALIB:WRITE:20.00:0.960:1.040:-0.025:-1:1", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!ERR:INVALID") == 0);

    // Reset back to factory defaults (invalidates EEPROM magic bytes)
    handled = CalibrationProtocol::executeCommand("!CALIB:RESET", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!RESET:OK") == 0);

    // Verify EEPROM magic signature is invalidated (0xFF, 0xFF)
    assert(Hal::getMockEeprom(0) == 0xFF);
    assert(Hal::getMockEeprom(1) == 0xFF);

    handled = CalibrationProtocol::executeCommand("!CALIB:READ", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!CALIB:DATA:65.00:1.000:1.000:0.000:1:1") == 0);
    assert(Motors::getInvertLeft() == 1);
    assert(Motors::getInvertRight() == 1);
    std::cout << "  ✓ Test 5: Profile Management (!CALIB:READ, WRITE, RESET) passed." << std::endl;

    // ----------------------------------------------------
    // Test 6: Serial Streaming update() Integration & Digit Preservation
    // ----------------------------------------------------
    NavigationFSM::init();
    Hal::clearMockSerialOutput();
    Hal::injectMockSerial("!PING\n");
    CalibrationProtocol::update();
    const char* out = Hal::getMockSerialOutput();
    assert(strstr(out, "!PONG\r\n") != NULL);

    Hal::clearMockSerialOutput();
    Hal::injectMockSerial("!CALIB:READ\n");
    CalibrationProtocol::update();
    out = Hal::getMockSerialOutput();
    assert(strstr(out, "!CALIB:DATA:65.00:1.000:1.000:0.000:1:1\r\n") != NULL);

    // Test that digits 1-4 inside frames are NOT consumed as room commands
    Hal::clearMockSerialOutput();
    NavigationFSM::selectRoom(1);
    Hal::injectMockSerial("!STEP:U2:90.0\n");
    CalibrationProtocol::update();
    out = Hal::getMockSerialOutput();
    assert(strstr(out, "!U2:DONE:") != NULL);
    assert(NavigationFSM::getTargetRoom() == 1); // Room remains 1, not altered by '2'

    // Test that bare '3' outside frames triggers room selection
    Hal::injectMockSerial("3");
    CalibrationProtocol::update();
    assert(NavigationFSM::getTargetRoom() == 3);
    std::cout << "  ✓ Test 6: Serial Streaming update() Integration & Digit Preservation passed." << std::endl;

    std::cout << "[TDD] ALL test_calibration_protocol passed successfully!" << std::endl;
    return 0;
}
