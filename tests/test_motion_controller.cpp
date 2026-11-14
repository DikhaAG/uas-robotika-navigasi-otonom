#include <cassert>
#include <iostream>
#include <cmath>
#include "../firmware/RobotNavigasiOtonom/Config.h"
#include "../firmware/RobotNavigasiOtonom/MotionController.h"

static bool approxEqual(float a, float b, float tol = 0.05f) {
    return std::fabs(a - b) <= tol;
}

int main() {
    std::cout << "[TDD] Running test_motion_controller..." << std::endl;

    // Test 1: rotateTowards - in-place pivot steering
    {
        int pwmL = 0, pwmR = 0;
        // Robot facing 0 deg, target is +45 deg (turn left)
        bool aligned = MotionController::rotateTowards(0.0f, 45.0f, pwmL, pwmR);
        assert(!aligned);
        assert(pwmL == -PWM_TURN);
        assert(pwmR == +PWM_TURN);

        // Robot facing 0 deg, target is -45 deg (turn right)
        aligned = MotionController::rotateTowards(0.0f, -45.0f, pwmL, pwmR);
        assert(!aligned);
        assert(pwmL == +PWM_TURN);
        assert(pwmR == -PWM_TURN);

        // Robot facing 42 deg, target is 45 deg (|err| = 3 deg <= TOL_ORIENTASI_DEG 8 deg)
        aligned = MotionController::rotateTowards(42.0f, 45.0f, pwmL, pwmR);
        assert(aligned);
        assert(pwmL == 0);
        assert(pwmR == 0);

        // computeTargetHeading trigonometry helper
        float headingNorth = MotionController::computeTargetHeading(0.0f, 0.0f, 0.0f, 100.0f);
        assert(approxEqual(headingNorth, 90.0f));
        float headingEast = MotionController::computeTargetHeading(0.0f, 0.0f, 100.0f, 0.0f);
        assert(approxEqual(headingEast, 0.0f));
        float headingSouth = MotionController::computeTargetHeading(0.0f, 0.0f, 0.0f, -100.0f);
        assert(approxEqual(headingSouth, -90.0f));
        std::cout << "  ✓ Test 1: rotateTowards() and computeTargetHeading() verified." << std::endl;
    }

    // Test 2: driveTowards - large angle error triggers in-place turn command
    {
        // Robot at (0, 0, theta=0), target directly left at (0, 50) -> target angle = 90 deg (> AMBANG_BELOK_DEG 25 deg)
        MotionCommand cmd = MotionController::driveTowards(0.0f, 0.0f, 0.0f, 0.0f, 50.0f);
        assert(cmd.isTurning == true);
        assert(!cmd.achieved);
        assert(approxEqual(cmd.distError, 50.0f));
        assert(approxEqual(cmd.angleError, 90.0f));
        assert(cmd.pwmL == -PWM_TURN);
        assert(cmd.pwmR == +PWM_TURN);
        std::cout << "  ✓ Test 2: driveTowards() large angle error delegates to in-place turn." << std::endl;
    }

    // Test 3: driveTowards - proportional forward drive & deceleration
    {
        // Robot at (0, 0, theta=0), target straight ahead at (100, 0)
        // Distance 100 cm (> JARAK_DECEL_CM 30 cm), angle error 0 deg
        MotionCommand cmd = MotionController::driveTowards(0.0f, 0.0f, 0.0f, 100.0f, 0.0f);
        assert(cmd.isTurning == false);
        assert(!cmd.achieved);
        assert(approxEqual(cmd.distError, 100.0f));
        assert(approxEqual(cmd.angleError, 0.0f));
        assert(cmd.pwmL == PWM_BASE);
        assert(cmd.pwmR == PWM_BASE);

        // Deceleration: target at (20, 0) (< JARAK_DECEL_CM 30 cm)
        cmd = MotionController::driveTowards(0.0f, 0.0f, 0.0f, 20.0f, 0.0f);
        assert(cmd.isTurning == false);
        assert(!cmd.achieved);
        assert(cmd.pwmL == PWM_SLOW);
        assert(cmd.pwmR == PWM_SLOW);
        std::cout << "  ✓ Test 3: driveTowards() forward cruise and deceleration verified." << std::endl;
    }

    // Test 4: driveTowards - proportional heading hold steering
    {
        // Robot at (0, 0, theta=-5 deg), target at (100, 0) (target angle = 0 deg)
        // angleError = target - current = 0 - (-5) = +5 deg
        // With e > 0, robot needs to turn left: pwmL = base - KP*e, pwmR = base + KP*e
        MotionCommand cmd = MotionController::driveTowards(0.0f, 0.0f, -5.0f, 100.0f, 0.0f);
        assert(cmd.isTurning == false);
        int expectedCorrection = (int)(KP_HEADING * 5.0f);
        assert(cmd.pwmL == PWM_BASE - expectedCorrection);
        assert(cmd.pwmR == PWM_BASE + expectedCorrection);
        std::cout << "  ✓ Test 4: driveTowards() proportional heading steering verified." << std::endl;
    }

    // Test 5: driveTowards - lateral wall-centering offsets
    {
        // Robot at (0, 0, theta=0), target at (100, 0)
        // Near left wall (wallLeft=true, wallRight=false): should steer right (pwmL higher, pwmR lower)
        MotionCommand cmd = MotionController::driveTowards(0.0f, 0.0f, 0.0f, 100.0f, 0.0f, true, false);
        assert(cmd.isTurning == false);
        assert(cmd.pwmL == PWM_BASE + 15);
        assert(cmd.pwmR == PWM_BASE - 15);

        // Near right wall (wallRight=true, wallLeft=false): should steer left (pwmL lower, pwmR higher)
        cmd = MotionController::driveTowards(0.0f, 0.0f, 0.0f, 100.0f, 0.0f, false, true);
        assert(cmd.isTurning == false);
        assert(cmd.pwmL == PWM_BASE - 15);
        assert(cmd.pwmR == PWM_BASE + 15);
        std::cout << "  ✓ Test 5: driveTowards() lateral wall-centering bias verified." << std::endl;
    }

    // Test 6: driveTowards - waypoint achievement thresholds
    {
        // Robot at (98, 0, theta=0), target at (100, 0) -> distError = 2.0 cm
        // Intermediate waypoint threshold TOL_WP_CM (5.0 cm)
        MotionCommand cmd = MotionController::driveTowards(98.0f, 0.0f, 0.0f, 100.0f, 0.0f, false, false, TOL_WP_CM);
        assert(cmd.achieved == true);

        // Robot at (96, 0, theta=0), target at (100, 0) -> distError = 4.0 cm
        // For intermediate (5.0 cm) -> achieved
        cmd = MotionController::driveTowards(96.0f, 0.0f, 0.0f, 100.0f, 0.0f, false, false, TOL_WP_CM);
        assert(cmd.achieved == true);

        // For final waypoint (TOL_AKHIR_CM = 2.0 cm) -> not achieved yet
        cmd = MotionController::driveTowards(96.0f, 0.0f, 0.0f, 100.0f, 0.0f, false, false, TOL_AKHIR_CM);
        assert(cmd.achieved == false);

        // Robot at (99, 0, theta=0), target at (100, 0) -> distError = 1.0 cm <= 2.0 cm -> achieved
        cmd = MotionController::driveTowards(99.0f, 0.0f, 0.0f, 100.0f, 0.0f, false, false, TOL_AKHIR_CM);
        assert(cmd.achieved == true);
        std::cout << "  ✓ Test 6: driveTowards() achievement thresholds (intermediate vs final) verified." << std::endl;
    }

    std::cout << "[TDD] ALL test_motion_controller passed successfully!" << std::endl;
    return 0;
}
