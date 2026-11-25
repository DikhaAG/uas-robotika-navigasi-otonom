"""
Domain models, data classes, and safety invariants for the Calibration Wizard.
"""

from typing import Optional, List


class SafetyGateError(RuntimeError):
    """Raised when motion calibration is attempted while hardware safety gate is locked."""
    pass


class DiagnosticResult:
    """Snapshot of hardware peripheral responsiveness from !DIAG:ALL."""
    def __init__(self, mpu_ok: bool, sonar_dist_cm: float, ir_left: bool, ir_right: bool):
        self.mpu_ok = bool(mpu_ok)
        self.sonar_dist_cm = float(sonar_dist_cm)
        self.ir_left = bool(ir_left)
        self.ir_right = bool(ir_right)

    def __repr__(self):
        return (f"DiagnosticResult(mpu_ok={self.mpu_ok}, sonar_dist_cm={self.sonar_dist_cm:.2f}, "
                f"ir_left={self.ir_left}, ir_right={self.ir_right})")


class CalibrationProfileData:
    """Non-volatile kinematic and sensor tuning parameters stored in microcontroller EEPROM."""
    def __init__(self,
                 diameter_roda_mm: float = 65.0,
                 trim_motor_kiri: float = 1.0,
                 trim_motor_kanan: float = 1.0,
                 gyro_bias_z: float = 0.0,
                 invert_motor_kiri: int = 1,
                 invert_motor_kanan: int = 1):
        self.diameter_roda_mm = float(diameter_roda_mm)
        self.trim_motor_kiri = float(trim_motor_kiri)
        self.trim_motor_kanan = float(trim_motor_kanan)
        self.gyro_bias_z = float(gyro_bias_z)
        self.invert_motor_kiri = int(invert_motor_kiri)
        self.invert_motor_kanan = int(invert_motor_kanan)

    def to_command_args(self) -> str:
        return (f"{self.diameter_roda_mm:.2f}:{self.trim_motor_kiri:.3f}:{self.trim_motor_kanan:.3f}:"
                f"{self.gyro_bias_z:.3f}:{self.invert_motor_kiri}:{self.invert_motor_kanan}")

    def clone(self) -> "CalibrationProfileData":
        return CalibrationProfileData(
            diameter_roda_mm=self.diameter_roda_mm,
            trim_motor_kiri=self.trim_motor_kiri,
            trim_motor_kanan=self.trim_motor_kanan,
            gyro_bias_z=self.gyro_bias_z,
            invert_motor_kiri=self.invert_motor_kiri,
            invert_motor_kanan=self.invert_motor_kanan
        )

    def __repr__(self):
        return (f"CalibrationProfileData(diameter={self.diameter_roda_mm:.2f}mm, "
                f"trimL={self.trim_motor_kiri:.3f}, trimR={self.trim_motor_kanan:.3f}, "
                f"biasZ={self.gyro_bias_z:.3f}, invL={self.invert_motor_kiri}, invR={self.invert_motor_kanan})")


class U0SubtestReport:
    """Outcome and wiring repair guidance for an individual Stage U0 subtest."""
    def __init__(self, name: str, passed: bool, details: str, pin_hints: Optional[List[str]] = None):
        self.name = name
        self.passed = bool(passed)
        self.details = details
        self.pin_hints = list(pin_hints or [])

    def __repr__(self):
        return f"U0SubtestReport(name='{self.name}', passed={self.passed}, details='{self.details}')"


class U0Report:
    """Aggregated pre-flight diagnostic report evaluating 100% hardware readiness."""
    def __init__(self):
        self.subtests: List[U0SubtestReport] = []
        self.all_passed = False
        self.u01_mpu_passed = False
        self.u02_sonar_passed = False
        self.u03_ir_passed = False
        self.u04_indicators_passed = False
        self.u05_motors_passed = False
        self.polarity_inversion_applied = False

    def add_subtest(self, subtest: U0SubtestReport):
        self.subtests.append(subtest)

    def get_wiring_repair_instructions(self) -> List[str]:
        hints = []
        for st in self.subtests:
            if not st.passed:
                hints.extend(st.pin_hints)
        return hints

    def __repr__(self):
        return (f"U0Report(all_passed={self.all_passed}, mpu={self.u01_mpu_passed}, "
                f"sonar={self.u02_sonar_passed}, ir={self.u03_ir_passed}, "
                f"indicators={self.u04_indicators_passed}, motors={self.u05_motors_passed})")


class SafetyGate:
    """Enforces hard-gated safety: kinematic motion is locked until 100% of U0 passes."""
    def __init__(self, u0_report: Optional[U0Report] = None):
        self.report = u0_report or U0Report()

    def update_report(self, report: U0Report):
        self.report = report

    def is_motion_permitted(self) -> bool:
        return self.report.all_passed

    def assert_motion_permitted(self):
        if not self.is_motion_permitted():
            failed_tests = [st.name for st in self.report.subtests if not st.passed]
            raise SafetyGateError(
                f"HARD-GATE LOCKED: Motion calibration (U1-U4) DILARANG sebelum pengujian hardware U0 100% lulus! "
                f"Sub-test gagal: {', '.join(failed_tests)}"
            )

    def __repr__(self):
        return f"SafetyGate(permitted={self.is_motion_permitted()})"


class U1Result:
    """Trial results from Stage U1: wheel distance and motor balance calibration."""
    def __init__(self, passed: bool, iteration: int, ground_truth_cm: float,
                 firmware_dist_cm: float, pulses_l: int, pulses_r: int,
                 dist_error_cm: float, pulse_drift_pct: float,
                 new_diameter_mm: float, new_trim_l: float, new_trim_r: float):
        self.passed = bool(passed)
        self.iteration = int(iteration)
        self.ground_truth_cm = float(ground_truth_cm)
        self.firmware_dist_cm = float(firmware_dist_cm)
        self.pulses_l = int(pulses_l)
        self.pulses_r = int(pulses_r)
        self.dist_error_cm = float(dist_error_cm)
        self.pulse_drift_pct = float(pulse_drift_pct)
        self.new_diameter_mm = float(new_diameter_mm)
        self.new_trim_l = float(new_trim_l)
        self.new_trim_r = float(new_trim_r)

    def __repr__(self):
        return (f"U1Result(passed={self.passed}, iter={self.iteration}, dist_err={self.dist_error_cm:.2f}cm, "
                f"drift={self.pulse_drift_pct:.2f}%, diam={self.new_diameter_mm:.2f}mm, "
                f"trimL={self.new_trim_l:.3f}, trimR={self.new_trim_r:.3f})")


class U2Result:
    """Trial results from Stage U2: gyroscope bias and in-place rotation."""
    def __init__(self, passed: bool, target_angle_deg: float,
                 actual_angle_deg: float, angular_error_deg: float,
                 polarity_verified: bool = True):
        self.passed = bool(passed)
        self.target_angle_deg = float(target_angle_deg)
        self.actual_angle_deg = float(actual_angle_deg)
        self.angular_error_deg = float(angular_error_deg)
        self.polarity_verified = bool(polarity_verified)

    def __repr__(self):
        return (f"U2Result(passed={self.passed}, target={self.target_angle_deg:.1f}°, "
                f"actual={self.actual_angle_deg:.1f}°, err={self.angular_error_deg:.2f}°)")


class U3Result:
    """Trial results from Stage U3: gyro-guided straight line tracking."""
    def __init__(self, passed: bool, pulses_l: int, pulses_r: int,
                 max_heading_error_deg: float, lateral_error_cm: float):
        self.passed = bool(passed)
        self.pulses_l = int(pulses_l)
        self.pulses_r = int(pulses_r)
        self.max_heading_error_deg = float(max_heading_error_deg)
        self.lateral_error_cm = float(lateral_error_cm)

    def __repr__(self):
        return (f"U3Result(passed={self.passed}, pulses=({self.pulses_l}, {self.pulses_r}), "
                f"max_heading_err={self.max_heading_error_deg:.2f}°, lat_err={self.lateral_error_cm:.2f}cm)")


class U4SubResult:
    """Measurement outcome for an individual ultrasonic distance test point."""
    def __init__(self, target_cm: float, measured_cm: float,
                 error_cm: float, passed: bool):
        self.target_cm = float(target_cm)
        self.measured_cm = float(measured_cm)
        self.error_cm = float(error_cm)
        self.passed = bool(passed)

    def __repr__(self):
        return f"U4SubResult(target={self.target_cm:.1f}cm, measured={self.measured_cm:.2f}cm, err={self.error_cm:.2f}cm, passed={self.passed})"


class U4Result:
    """Aggregated trial results from Stage U4: ultrasonic obstacle ranging accuracy."""
    def __init__(self, passed: bool, sub_results: List[U4SubResult]):
        self.passed = bool(passed)
        self.sub_results = list(sub_results)

    def __repr__(self):
        return f"U4Result(passed={self.passed}, items={len(self.sub_results)})"


class CalibrationReport:
    """Complete summary report covering pre-flight hardware and kinematic stages."""
    def __init__(self,
                 u0: Optional[U0Report] = None,
                 u1: Optional[U1Result] = None,
                 u2: Optional[U2Result] = None,
                 u3: Optional[U3Result] = None,
                 u4: Optional[U4Result] = None):
        self.u0 = u0
        self.u1 = u1
        self.u2 = u2
        self.u3 = u3
        self.u4 = u4

    @property
    def all_passed(self) -> bool:
        kinematics = [self.u1, self.u2, self.u3, self.u4]
        active = [k for k in kinematics if k is not None]
        if not active:
            return self.u0.all_passed if self.u0 else False
        return (self.u0.all_passed if self.u0 else True) and all(k.passed for k in active)

    def __repr__(self):
        return f"CalibrationReport(all_passed={self.all_passed}, u0={bool(self.u0)}, u1={bool(self.u1)}, u2={bool(self.u2)}, u3={bool(self.u3)}, u4={bool(self.u4)})"
