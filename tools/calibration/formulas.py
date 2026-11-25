"""
Pure kinematic calculation functions and tolerance validation rules.
"""

from tools.calibration.models import MotorTrimPair, ToleranceOutcome, U1ToleranceOutcome


def calculate_wheel_diameter_correction(current_diam: float, ground_truth_dist: float, firmware_dist: float) -> float:
    """
    Computes updated wheel diameter using the closed-form scaling formula:
    D_new = D_current * (D_ground_truth / D_firmware)
    """
    if firmware_dist <= 0:
        firmware_dist = 100.0
    return float(current_diam * (ground_truth_dist / firmware_dist))


def calculate_motor_trim_correction(trim_l: float, trim_r: float, pulses_l: int, pulses_r: int) -> MotorTrimPair:
    """
    Computes motor trim factors to balance wheel rotation speeds:
    - If left wheel produced more pulses (faster), scale down trim_l by (pulses_r / pulses_l).
    - If right wheel produced more pulses (faster), scale down trim_r by (pulses_l / pulses_r).
    Clamped to safe bounds [0.75, 1.25].
    """
    if pulses_l <= 0 or pulses_r <= 0:
        return MotorTrimPair(float(trim_l), float(trim_r))

    if pulses_l > pulses_r:
        ratio = float(pulses_r) / float(pulses_l)
        trim_l = trim_l * ratio
    elif pulses_r > pulses_l:
        ratio = float(pulses_l) / float(pulses_r)
        trim_r = trim_r * ratio

    trim_l = max(0.75, min(1.25, trim_l))
    trim_r = max(0.75, min(1.25, trim_r))
    return MotorTrimPair(float(trim_l), float(trim_r))


def check_u1_tolerances(ground_truth_dist: float, pulses_l: int, pulses_r: int) -> U1ToleranceOutcome:
    """
    Checks U1 tolerances:
    1. Distance translation error <= 3.0 cm relative to 100.0 cm nominal.
    2. Wheel pulse drift disparity <= 3.0%.
    """
    dist_error = abs(ground_truth_dist - 100.0)
    max_pulses = max(pulses_l, pulses_r)
    drift_pct = (abs(pulses_l - pulses_r) / float(max_pulses) * 100.0) if max_pulses > 0 else 0.0
    passed = (dist_error <= 3.0) and (drift_pct <= 3.0)
    return U1ToleranceOutcome(passed, float(dist_error), float(drift_pct))


def check_u2_tolerances(actual_angle: float, target_angle: float = 90.0) -> ToleranceOutcome:
    """
    Checks U2 tolerance: in-place rotation angle error <= 5.0 deg.
    Preserves sign to prevent reversed rotation (-90 deg) from passing when +90 deg is target.
    """
    angular_error = abs(actual_angle - target_angle)
    passed = (angular_error <= 5.0)
    return ToleranceOutcome(passed, float(angular_error))


def check_u3_tolerances(lateral_deviation_cm: float) -> ToleranceOutcome:
    """
    Checks U3 tolerance: lateral corridor deviation <= 5.0 cm.
    """
    lat_err = abs(lateral_deviation_cm)
    passed = (lat_err <= 5.0)
    return ToleranceOutcome(passed, float(lat_err))


def check_u4_tolerances(measured_cm: float, target_cm: float) -> ToleranceOutcome:
    """
    Checks U4 tolerance: ultrasonic ranging error <= 1.0 cm.
    """
    err = abs(measured_cm - target_cm)
    passed = (err <= 1.0)
    return ToleranceOutcome(passed, float(err))
