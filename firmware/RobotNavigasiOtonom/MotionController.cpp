#include "MotionController.h"
#include "Odometry.h"
#include <math.h>

bool MotionController::rotateTowards(float currentTheta, float targetHeading, int& pwmL, int& pwmR) {
    float err = Odometry::normalizeAngleDeg(targetHeading - currentTheta);

    if (fabsf(err) <= TOL_ORIENTASI_DEG) {
        pwmL = 0;
        pwmR = 0;
        return true;
    }

    if (err > 0.0f) {
        // Target is counter-clockwise -> turn left
        pwmL = -PWM_TURN;
        pwmR = +PWM_TURN;
    } else {
        // Target is clockwise -> turn right
        pwmL = +PWM_TURN;
        pwmR = -PWM_TURN;
    }

    return false;
}

MotionCommand MotionController::driveTowards(float currentX, float currentY, float currentTheta,
                                            float targetX, float targetY,
                                            bool wallLeft, bool wallRight,
                                            float toleranceCm) {
    MotionCommand cmd;

    float dx = targetX - currentX;
    float dy = targetY - currentY;

    cmd.distError = sqrtf(dx * dx + dy * dy);
    float targetAngleDeg = atan2f(dy, dx) * (180.0f / PI);
    cmd.angleError = Odometry::normalizeAngleDeg(targetAngleDeg - currentTheta);
    cmd.achieved = (cmd.distError <= toleranceCm);

    // Large angle error -> in-place pivot rotation
    if (fabsf(cmd.angleError) > AMBANG_BELOK_DEG) {
        cmd.isTurning = true;
        rotateTowards(currentTheta, targetAngleDeg, cmd.pwmL, cmd.pwmR);
        return cmd;
    }

    // Small angle error -> proportional forward cruise
    cmd.isTurning = false;
    int base = (cmd.distError < JARAK_DECEL_CM) ? PWM_SLOW : PWM_BASE;
    int u = (int)(KP_HEADING * cmd.angleError);

    // Lateral corridor wall centering correction (steer away from detected walls)
    int wallCorrection = 0;
    if (wallLeft && !wallRight) {
        wallCorrection = +15; // Steer right away from left wall
    } else if (wallRight && !wallLeft) {
        wallCorrection = -15; // Steer left away from right wall
    }

    cmd.pwmL = base - u + wallCorrection;
    cmd.pwmR = base + u - wallCorrection;

    // Clamp forward drive between 0 and 255
    if (cmd.pwmL > 255) cmd.pwmL = 255;
    if (cmd.pwmL < 0) cmd.pwmL = 0;
    if (cmd.pwmR > 255) cmd.pwmR = 255;
    if (cmd.pwmR < 0) cmd.pwmR = 0;

    return cmd;
}
