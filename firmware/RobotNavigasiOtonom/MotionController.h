#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "Config.h"

struct MotionCommand {
    int pwmL;
    int pwmR;
    float distError;
    float angleError;
    bool isTurning;
    bool achieved;
};

class MotionController {
public:
    /**
     * @brief Drive closed-loop towards a target 2D waypoint.
     *
     * Calculates distance error rho and angular error e relative to current heading.
     * If |e| > AMBANG_BELOK_DEG (25 deg), sets isTurning = true and commands in-place pivot.
     * Otherwise sets isTurning = false and commands forward cruise with proportional heading
     * hold, deceleration within JARAK_DECEL_CM, lateral wall-centering bias, and [0, 255] clamping.
     * Sets achieved = true if distError <= toleranceCm.
     */
    static MotionCommand driveTowards(float currentX, float currentY, float currentTheta,
                                      float targetX, float targetY,
                                      bool wallLeft = false, bool wallRight = false,
                                      float toleranceCm = TOL_WP_CM);

    /**
     * @brief In-place pivot steering towards targetHeading.
     *
     * Returns true when |targetHeading - currentTheta| <= TOL_ORIENTASI_DEG.
     * Commands pwmL = -PWM_TURN, pwmR = +PWM_TURN when err > 0 (turn left),
     * and pwmL = +PWM_TURN, pwmR = -PWM_TURN when err < 0 (turn right).
     * Commands pwmL = 0, pwmR = 0 when aligned.
     */
    static bool rotateTowards(float currentTheta, float targetHeading, int& pwmL, int& pwmR);
};

#endif // MOTION_CONTROLLER_H
