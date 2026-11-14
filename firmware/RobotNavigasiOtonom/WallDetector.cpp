#include "WallDetector.h"

#ifndef ARDUINO
bool WallDetector::mockIrLeft = false;
bool WallDetector::mockIrRight = false;
#endif

void WallDetector::init() {
    Hal::pinMode(PIN_IR_KIRI, INPUT);
    Hal::pinMode(PIN_IR_KANAN, INPUT);
#ifndef ARDUINO
    mockIrLeft = false;
    mockIrRight = false;
#endif
}

bool WallDetector::isWallLeft() {
#ifdef ARDUINO
    // FC-51 sensor is active LOW
    return (Hal::digitalRead(PIN_IR_KIRI) == LOW);
#else
    return mockIrLeft;
#endif
}

bool WallDetector::isWallRight() {
#ifdef ARDUINO
    return (Hal::digitalRead(PIN_IR_KANAN) == LOW);
#else
    return mockIrRight;
#endif
}

#ifndef ARDUINO
void WallDetector::setMockWalls(bool left_detected, bool right_detected) {
    mockIrLeft = left_detected;
    mockIrRight = right_detected;
}
#endif
