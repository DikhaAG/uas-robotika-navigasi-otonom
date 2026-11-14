#ifndef WALL_DETECTOR_H
#define WALL_DETECTOR_H

#include "Config.h"
#include "Hal.h"
#include <stdint.h>

class WallDetector {
public:
    static void init();
    static bool isWallLeft();   // True if left wall detected (within IR distance threshold)
    static bool isWallRight();  // True if right wall detected

#ifndef ARDUINO
    // Mock methods for desktop TDD & simulation
    static void setMockWalls(bool left_detected, bool right_detected);
#endif

private:
#ifndef ARDUINO
    static bool mockIrLeft;
    static bool mockIrRight;
#endif
};

#endif // WALL_DETECTOR_H
