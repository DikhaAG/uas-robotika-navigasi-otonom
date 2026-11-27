#include <cassert>
#include <iostream>
#include <cstring>
#include <cmath>
#include "test_common.h"
#include "../firmware/RobotNavigasiOtonom/AsciiProtocolDecoder.h"
#include "../firmware/RobotNavigasiOtonom/CalibrationService.h"
#include "../firmware/RobotNavigasiOtonom/FloatText.h"

static bool approxEqual(float a, float b, float tol = 0.05f) {
    return std::fabs(a - b) <= tol;
}

static uint8_t g_selectedRoom = 0;
static void testRoomCallback(uint8_t room) {
    g_selectedRoom = room;
}

int main() {
    std::cout << "[TDD] Running test_calibration_protocol..." << std::endl;
    resetAllMocks();

    // ----------------------------------------------------
    // Test 1: AsciiProtocolDecoder pure string framing
    // ----------------------------------------------------
    AsciiProtocolDecoder::init(testRoomCallback);
    char frame[64];
    memset(frame, 0, sizeof(frame));

    // Noise before '!' should be discarded
    assert(!AsciiProtocolDecoder::feedChar('x', frame, sizeof(frame)));
    assert(!AsciiProtocolDecoder::feedChar('y', frame, sizeof(frame)));
    assert(!AsciiProtocolDecoder::feedChar('\n', frame, sizeof(frame)));
    assert(!AsciiProtocolDecoder::isInFrame());

    // Feed "!PING\n"
    assert(!AsciiProtocolDecoder::feedChar('!', frame, sizeof(frame)));
    assert(AsciiProtocolDecoder::isInFrame());
    assert(!AsciiProtocolDecoder::feedChar('P', frame, sizeof(frame)));
    assert(!AsciiProtocolDecoder::feedChar('I', frame, sizeof(frame)));
    assert(!AsciiProtocolDecoder::feedChar('N', frame, sizeof(frame)));
    assert(!AsciiProtocolDecoder::feedChar('G', frame, sizeof(frame)));
    assert(!AsciiProtocolDecoder::feedChar('\r', frame, sizeof(frame)));
    bool completed = AsciiProtocolDecoder::feedChar('\n', frame, sizeof(frame));
    assert(completed);
    assert(!AsciiProtocolDecoder::isInFrame());
    assert(strcmp(frame, "!PING") == 0);

    // Bare digit room triggers
    g_selectedRoom = 0;
    assert(!AsciiProtocolDecoder::feedChar('2', frame, sizeof(frame)));
    assert(g_selectedRoom == 2);
    std::cout << "  ✓ Test 1: Pure AsciiProtocolDecoder framing passed." << std::endl;

    // ----------------------------------------------------
    // Test 2: Basic Command Dispatch: !PING -> !PONG and Unknown
    // ----------------------------------------------------
    char response[128];
    memset(response, 0, sizeof(response));

    bool handled = CalibrationService::executeCommand("!PING", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!PONG") == 0);

    handled = CalibrationService::executeCommand("!FOOBAR", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!ERR:UNKNOWN") == 0);

    // Malformed commands without '!'
    handled = CalibrationService::executeCommand("PING", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!ERR:MALFORMED") == 0);

    std::cout << "  ✓ Test 2: Basic Command Dispatch (!PING, !ERR) passed." << std::endl;

    // ----------------------------------------------------
    // Test 3: Diagnostic Queries (!DIAG:ALL, !DIAG:MOTOR, !DIAG:BEEP)
    // ----------------------------------------------------
    UltrasonicSonar::setMockDistance(25.40f);
    WallDetector::setMockWalls(false, true); // left false, right true
    Gyroscope::setMockWhoAmI(0x68);    // MPU6050 OK

    handled = CalibrationService::executeCommand("!DIAG:ALL", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!DIAG:RES:1:25.40:0:1") == 0);

    // If MPU6050 whoAmI != 0x68
    Gyroscope::setMockWhoAmI(0x00);
    handled = CalibrationService::executeCommand("!DIAG:ALL", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!DIAG:RES:0:25.40:0:1") == 0);
    Gyroscope::setMockWhoAmI(0x68); // restore

    // Motor Spin Test: !DIAG:MOTOR:<pwmL>:<pwmR>:<duration_ms>
    // Runs closed-loop differential drive simulation on desktop without test backdoors
    handled = CalibrationService::executeCommand("!DIAG:MOTOR:150:150:200", response, sizeof(response));
    assert(handled);
    assert(strncmp(response, "!PULSES:", 8) == 0);

    // Indicator / Buzzer Test: !DIAG:BEEP:100
    handled = CalibrationService::executeCommand("!DIAG:BEEP:100", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!OK") == 0);
    std::cout << "  ✓ Test 3: Diagnostic Queries (!DIAG:ALL, !DIAG:MOTOR, !DIAG:BEEP) passed." << std::endl;

    // ----------------------------------------------------
    // Test 4: Discrete Calibration Steps (!STEP:U1, U2, U3, U4)
    // ----------------------------------------------------
    // !STEP:U1 -> !U1:DONE:<pL>:<pR>:<dist> (simulates closed-loop 100 cm drive)
    handled = CalibrationService::executeCommand("!STEP:U1", response, sizeof(response));
    assert(handled);
    assert(strncmp(response, "!U1:DONE:", 9) == 0);

    // !STEP:U2:<angle> -> !U2:DONE:<finalTheta> (simulates closed-loop in-place turn)
    handled = CalibrationService::executeCommand("!STEP:U2:90.0", response, sizeof(response));
    assert(handled);
    assert(strncmp(response, "!U2:DONE:", 9) == 0);

    // !STEP:U3 -> !U3:DONE:<pL>:<pR>:<maxHeadingError> (simulates 150 cm straight line cruise)
    handled = CalibrationService::executeCommand("!STEP:U3", response, sizeof(response));
    assert(handled);
    assert(strncmp(response, "!U3:DONE:", 9) == 0);

    // !STEP:U4 -> !U4:DONE:<sonarCm>
    UltrasonicSonar::setMockDistance(20.15f);
    handled = CalibrationService::executeCommand("!STEP:U4", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!U4:DONE:20.15") == 0);
    std::cout << "  ✓ Test 4: Discrete Calibration Steps (!STEP:U1..U4) passed." << std::endl;

    // ----------------------------------------------------
    // Test 5: Profile Management (!CALIB:READ, !CALIB:WRITE, !CALIB:RESET)
    // ----------------------------------------------------
    // Default profile
    handled = CalibrationService::executeCommand("!CALIB:READ", response, sizeof(response));
    assert(handled);
    // Defaults: diam 65.0, trimL 1.0, trimR 1.0, biasZ 0.0, invL 1, invR 1
    assert(strcmp(response, "!CALIB:DATA:65.00:1.000:1.000:0.000:1:1") == 0);

    // Write updated profile
    handled = CalibrationService::executeCommand("!CALIB:WRITE:66.50:0.960:1.040:-0.025:-1:1", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!OK") == 0);

    // Read back to confirm EEPROM persistence
    handled = CalibrationService::executeCommand("!CALIB:READ", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!CALIB:DATA:66.50:0.960:1.040:-0.025:-1:1") == 0);
    assert(Motors::getInvertLeft() == -1);
    assert(Motors::getInvertRight() == 1);
    assert(approxEqual(Motors::getTrimLeft(), 0.960f, 0.005f));
    assert(approxEqual(Motors::getTrimRight(), 1.040f, 0.005f));
    assert(approxEqual(Odometry::getWheelDiameterMm(), 66.50f, 0.01f));

    // Reject out-of-range values (e.g. diameter 20mm or trim 2.50)
    handled = CalibrationService::executeCommand("!CALIB:WRITE:20.00:0.960:1.040:-0.025:-1:1", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!ERR:INVALID") == 0);

    // Reset back to factory defaults (invalidates EEPROM magic bytes)
    handled = CalibrationService::executeCommand("!CALIB:RESET", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!RESET:OK") == 0);

    // Verify EEPROM magic signature is invalidated (0xFF, 0xFF)
    assert(Hal::getMockEeprom(0) == 0xFF);
    assert(Hal::getMockEeprom(1) == 0xFF);

    handled = CalibrationService::executeCommand("!CALIB:READ", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!CALIB:DATA:65.00:1.000:1.000:0.000:1:1") == 0);
    assert(Motors::getInvertLeft() == 1);
    assert(Motors::getInvertRight() == 1);
    assert(approxEqual(Odometry::getWheelDiameterMm(), DIAMETER_RODA_MM, 0.01f));
    std::cout << "  ✓ Test 5: Profile Management (!CALIB:READ, WRITE, RESET) passed." << std::endl;

    // ----------------------------------------------------
    // Test 6: Serial Streaming update() Integration & Digit Preservation
    // ----------------------------------------------------
    NavigationFSM::init();
    Hal::clearMockSerialOutput();
    Hal::injectMockSerial("!PING\n");
    CalibrationService::update();
    const char* out = Hal::getMockSerialOutput();
    assert(strstr(out, "!PONG\r\n") != NULL);

    Hal::clearMockSerialOutput();
    Hal::injectMockSerial("!CALIB:READ\n");
    CalibrationService::update();
    out = Hal::getMockSerialOutput();
    assert(strstr(out, "!CALIB:DATA:65.00:1.000:1.000:0.000:1:1\r\n") != NULL);

    // Test that digits 1-4 inside frames are NOT consumed as room commands
    Hal::clearMockSerialOutput();
    NavigationFSM::selectRoom(1);
    Hal::injectMockSerial("!STEP:U2:90.0\n");
    CalibrationService::update();
    out = Hal::getMockSerialOutput();
    assert(strstr(out, "!U2:DONE:") != NULL);
    assert(NavigationFSM::getTargetRoom() == 1); // Room remains 1, not altered by '2'

    // Test that bare '3' outside frames triggers room selection
    Hal::injectMockSerial("3");
    CalibrationService::update();
    assert(NavigationFSM::getTargetRoom() == 3);
    std::cout << "  ✓ Test 6: Serial Streaming update() Integration & Digit Preservation passed." << std::endl;

    // ----------------------------------------------------
    // Test 7: FloatText (AVR-safe replacement for printf/scanf "%f")
    // ----------------------------------------------------
    char num[FloatText::BUFFER_SIZE];
    assert(strcmp(FloatText::format(num, sizeof(num), 25.4f, 2), "25.40") == 0);
    assert(strcmp(FloatText::format(num, sizeof(num), 1.0f, 3), "1.000") == 0);
    assert(strcmp(FloatText::format(num, sizeof(num), -0.025f, 3), "-0.025") == 0);
    assert(strcmp(FloatText::format(num, sizeof(num), -178.015f, 2), "-178.02") == 0);
    assert(strcmp(FloatText::format(num, sizeof(num), -0.004f, 2), "0.00") == 0);   // no "-0.00"
    assert(strcmp(FloatText::format(num, sizeof(num), 7.0f, 0), "7") == 0);
    assert(strcmp(FloatText::format(num, sizeof(num), 1.0e12f, 2), "inf") == 0);    // out of range, still parseable by Python
    assert(strcmp(FloatText::format(num, sizeof(num), -1.0e12f, 2), "-inf") == 0);
    assert(strcmp(FloatText::format(num, sizeof(num), NAN, 2), "nan") == 0);

    const char* cursor = "66.50:-0.025:-1:1";
    float f = 0.0f;
    long n = 0;
    assert(FloatText::parseFloatField(cursor, ':', f) && approxEqual(f, 66.5f, 0.001f));
    assert(FloatText::parseFloatField(cursor, ':', f) && approxEqual(f, -0.025f, 0.0001f));
    assert(FloatText::parseLongField(cursor, ':', n) && n == -1);
    assert(FloatText::parseLongField(cursor, '\0', n) && n == 1);

    cursor = "abc:1";
    assert(!FloatText::parseFloatField(cursor, ':', f));      // not a number
    cursor = "1.5x:2";
    assert(!FloatText::parseFloatField(cursor, ':', f));      // trailing junk before separator
    cursor = "1.5";
    assert(!FloatText::parseFloatField(cursor, ':', f));      // separator missing
    cursor = "2.5:3";
    assert(!FloatText::parseLongField(cursor, ':', n));       // integer field rejects decimals

    // !STEP:U2:<angle> honours a non-default target angle (parsed without sscanf "%f")
    handled = CalibrationService::executeCommand("!STEP:U2:45.00", response, sizeof(response));
    assert(handled);
    float theta = (float)atof(response + 9);
    assert(theta > 30.0f && theta < 60.0f);

    // Malformed !CALIB:WRITE is rejected instead of half-applied
    handled = CalibrationService::executeCommand("!CALIB:WRITE:66.50:0.960:x:0.000:1:1", response, sizeof(response));
    assert(handled);
    assert(strcmp(response, "!ERR:PARAM") == 0);
    std::cout << "  ✓ Test 7: FloatText formatting/parsing and float protocol args passed." << std::endl;

    std::cout << "[TDD] ALL test_calibration_protocol passed successfully!" << std::endl;
    return 0;
}
