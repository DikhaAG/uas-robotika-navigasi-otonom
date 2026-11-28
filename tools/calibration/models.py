"""
Domain models, data classes, and safety invariants for the Calibration Wizard.
"""

from dataclasses import dataclass, field
from typing import ClassVar, Dict, Iterator, List, Optional, Tuple, Union


class SafetyGateError(RuntimeError):
    """Raised when motion calibration is attempted while hardware safety gate is locked."""
    pass


@dataclass
class DiagnosticResult:
    """Snapshot of hardware peripheral responsiveness from !DIAG:ALL."""
    mpu_ok: bool
    sonar_dist_cm: float
    ir_left: bool
    ir_right: bool

    def __post_init__(self) -> None:
        self.mpu_ok = bool(self.mpu_ok)
        self.sonar_dist_cm = float(self.sonar_dist_cm)
        self.ir_left = bool(self.ir_left)
        self.ir_right = bool(self.ir_right)

    def __repr__(self) -> str:
        return (f"DiagnosticResult(mpu_ok={self.mpu_ok}, sonar_dist_cm={self.sonar_dist_cm:.2f}, "
                f"ir_left={self.ir_left}, ir_right={self.ir_right})")


@dataclass
class CalibrationProfileData:
    """Non-volatile kinematic and sensor tuning parameters stored in microcontroller EEPROM."""
    diameter_roda_mm: float = 65.0
    trim_motor_kiri: float = 1.0
    trim_motor_kanan: float = 1.0
    gyro_bias_z: float = 0.0
    invert_motor_kiri: int = 1
    invert_motor_kanan: int = 1
    gyro_sign: int = 1  # +1 MPU6050 chip facing up, -1 facing down

    def __post_init__(self) -> None:
        self.diameter_roda_mm = float(self.diameter_roda_mm)
        self.trim_motor_kiri = float(self.trim_motor_kiri)
        self.trim_motor_kanan = float(self.trim_motor_kanan)
        self.gyro_bias_z = float(self.gyro_bias_z)
        self.invert_motor_kiri = int(self.invert_motor_kiri)
        self.invert_motor_kanan = int(self.invert_motor_kanan)
        self.gyro_sign = int(self.gyro_sign)

    def to_command_args(self) -> str:
        return (f"{self.diameter_roda_mm:.2f}:{self.trim_motor_kiri:.3f}:{self.trim_motor_kanan:.3f}:"
                f"{self.gyro_bias_z:.3f}:{self.invert_motor_kiri}:{self.invert_motor_kanan}:{self.gyro_sign}")

    def clone(self) -> "CalibrationProfileData":
        return CalibrationProfileData(
            diameter_roda_mm=self.diameter_roda_mm,
            trim_motor_kiri=self.trim_motor_kiri,
            trim_motor_kanan=self.trim_motor_kanan,
            gyro_bias_z=self.gyro_bias_z,
            invert_motor_kiri=self.invert_motor_kiri,
            invert_motor_kanan=self.invert_motor_kanan,
            gyro_sign=self.gyro_sign,
        )

    def __repr__(self) -> str:
        return (f"CalibrationProfileData(diameter={self.diameter_roda_mm:.2f}mm, "
                f"trimL={self.trim_motor_kiri:.3f}, trimR={self.trim_motor_kanan:.3f}, "
                f"biasZ={self.gyro_bias_z:.3f}, invL={self.invert_motor_kiri}, invR={self.invert_motor_kanan}, "
                f"gyroSign={self.gyro_sign})")


@dataclass
class MpuMounting:
    """
    How the GY-521 board sits on the chassis. The firmware only integrates the gyro
    Z axis (yaw), so the sign depends solely on whether the chip faces up or down;
    the printed X arrow is recorded for the log but never changes the result.
    """
    chip_facing: str    # "atas" | "bawah" | "tegak"
    x_axis_facing: str  # "depan" | "belakang" | "kiri" | "kanan"

    CHIP_FACING_CHOICES: ClassVar[Tuple[str, ...]] = ("atas", "bawah", "tegak")
    X_AXIS_CHOICES: ClassVar[Tuple[str, ...]] = ("depan", "belakang", "kiri", "kanan")
    _SIGN_BY_FACING: ClassVar[Dict[str, Optional[int]]] = {"atas": 1, "bawah": -1, "tegak": None}

    def __post_init__(self) -> None:
        if self.chip_facing not in self.CHIP_FACING_CHOICES:
            raise ValueError(f"chip_facing harus salah satu dari {self.CHIP_FACING_CHOICES}")
        if self.x_axis_facing not in self.X_AXIS_CHOICES:
            raise ValueError(f"x_axis_facing harus salah satu dari {self.X_AXIS_CHOICES}")

    @property
    def gyro_sign(self) -> Optional[int]:
        """+1 chip up, -1 chip down, None when upright (yaw cannot be measured)."""
        return self._SIGN_BY_FACING[self.chip_facing]

    def describe(self) -> str:
        return f"chip menghadap {self.chip_facing.upper()}, panah X ke {self.x_axis_facing.upper()}"


@dataclass
class MotorPulses:
    """Encoder pulse counts captured from left and right wheels."""
    left: int
    right: int

    def __iter__(self) -> Iterator[int]:
        yield self.left
        yield self.right


@dataclass
class EncoderTranslationTelemetry:
    """Telemetry returned from firmware Stage U1 straight-line translation."""
    pulses_l: int
    pulses_r: int
    firmware_dist_cm: float

    def __iter__(self) -> Iterator[Union[int, float]]:
        yield self.pulses_l
        yield self.pulses_r
        yield self.firmware_dist_cm


@dataclass
class HeadingTelemetry:
    """Telemetry returned from firmware Stage U3 gyro-guided straight-line tracking."""
    pulses_l: int
    pulses_r: int
    max_heading_err_deg: float

    def __iter__(self) -> Iterator[Union[int, float]]:
        yield self.pulses_l
        yield self.pulses_r
        yield self.max_heading_err_deg


@dataclass
class MotorTrimPair:
    """Balanced left and right motor trim multipliers."""
    left: float
    right: float

    def __iter__(self) -> Iterator[float]:
        yield self.left
        yield self.right


@dataclass
class U1ToleranceOutcome:
    """Tolerance evaluation result for Stage U1 translation and drift checks."""
    passed: bool
    dist_error_cm: float
    pulse_drift_pct: float

    def __iter__(self) -> Iterator[Union[bool, float]]:
        yield self.passed
        yield self.dist_error_cm
        yield self.pulse_drift_pct


@dataclass
class ToleranceOutcome:
    """Tolerance evaluation result for single-metric stages (U2, U3, U4)."""
    passed: bool
    error: float

    def __iter__(self) -> Iterator[Union[bool, float]]:
        yield self.passed
        yield self.error


@dataclass
class TrialRecord:
    """Structured telemetry record emitted after each calibration trial."""
    test_id: str
    iteration: int
    status: str
    target: float
    measured: float
    ground_truth: float
    error: float
    profile: Optional[CalibrationProfileData] = None
    notes: str = ""


@dataclass
class U0SubtestReport:
    """Outcome and wiring repair guidance for an individual Stage U0 subtest."""
    name: str
    passed: bool
    details: str
    pin_hints: List[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.passed = bool(self.passed)
        self.pin_hints = list(self.pin_hints or [])

    def __repr__(self) -> str:
        return f"U0SubtestReport(name='{self.name}', passed={self.passed}, details='{self.details}')"


@dataclass
class U0Report:
    """Aggregated pre-flight diagnostic report evaluating 100% hardware readiness."""
    subtests: List[U0SubtestReport] = field(default_factory=list)
    all_passed: bool = False
    u01_mpu_passed: bool = False
    u02_sonar_passed: bool = False
    u03_ir_passed: bool = False
    u04_indicators_passed: bool = False
    u05_motors_passed: bool = False
    polarity_inversion_applied: bool = False

    def add_subtest(self, subtest: U0SubtestReport) -> None:
        self.subtests.append(subtest)

    def get_wiring_repair_instructions(self) -> List[str]:
        hints: List[str] = []
        for st in self.subtests:
            if not st.passed:
                hints.extend(st.pin_hints)
        return hints

    def __repr__(self) -> str:
        return (f"U0Report(all_passed={self.all_passed}, mpu={self.u01_mpu_passed}, "
                f"sonar={self.u02_sonar_passed}, ir={self.u03_ir_passed}, "
                f"indicators={self.u04_indicators_passed}, motors={self.u05_motors_passed})")


@dataclass
class SafetyGate:
    """Enforces hard-gated safety: kinematic motion is locked until 100% of U0 passes."""
    report: U0Report = field(default_factory=U0Report)

    def __post_init__(self) -> None:
        if self.report is None:
            self.report = U0Report()

    def update_report(self, report: U0Report) -> None:
        self.report = report

    def is_motion_permitted(self) -> bool:
        return self.report.all_passed

    def assert_motion_permitted(self) -> None:
        if not self.is_motion_permitted():
            failed_tests = [st.name for st in self.report.subtests if not st.passed]
            raise SafetyGateError(
                f"HARD-GATE LOCKED: Motion calibration (U1-U4) DILARANG sebelum pengujian hardware U0 100% lulus! "
                f"Sub-test gagal: {', '.join(failed_tests)}"
            )

    def __repr__(self) -> str:
        return f"SafetyGate(permitted={self.is_motion_permitted()})"


@dataclass
class U1Result:
    """Trial results from Stage U1: wheel distance and motor balance calibration."""
    passed: bool
    iteration: int
    ground_truth_cm: float
    firmware_dist_cm: float
    pulses_l: int
    pulses_r: int
    dist_error_cm: float
    pulse_drift_pct: float
    new_diameter_mm: float
    new_trim_l: float
    new_trim_r: float

    def __post_init__(self) -> None:
        self.passed = bool(self.passed)
        self.iteration = int(self.iteration)
        self.ground_truth_cm = float(self.ground_truth_cm)
        self.firmware_dist_cm = float(self.firmware_dist_cm)
        self.pulses_l = int(self.pulses_l)
        self.pulses_r = int(self.pulses_r)
        self.dist_error_cm = float(self.dist_error_cm)
        self.pulse_drift_pct = float(self.pulse_drift_pct)
        self.new_diameter_mm = float(self.new_diameter_mm)
        self.new_trim_l = float(self.new_trim_l)
        self.new_trim_r = float(self.new_trim_r)

    def __repr__(self) -> str:
        return (f"U1Result(passed={self.passed}, iter={self.iteration}, dist_err={self.dist_error_cm:.2f}cm, "
                f"drift={self.pulse_drift_pct:.2f}%, diam={self.new_diameter_mm:.2f}mm, "
                f"trimL={self.new_trim_l:.3f}, trimR={self.new_trim_r:.3f})")


@dataclass
class U2Result:
    """Trial results from Stage U2: gyroscope bias and in-place rotation."""
    passed: bool
    target_angle_deg: float
    actual_angle_deg: float
    angular_error_deg: float
    polarity_verified: bool = True

    def __post_init__(self) -> None:
        self.passed = bool(self.passed)
        self.target_angle_deg = float(self.target_angle_deg)
        self.actual_angle_deg = float(self.actual_angle_deg)
        self.angular_error_deg = float(self.angular_error_deg)
        self.polarity_verified = bool(self.polarity_verified)

    def __repr__(self) -> str:
        return (f"U2Result(passed={self.passed}, target={self.target_angle_deg:.1f}°, "
                f"actual={self.actual_angle_deg:.1f}°, err={self.angular_error_deg:.2f}°)")


@dataclass
class U3Result:
    """Trial results from Stage U3: gyro-guided straight line tracking."""
    passed: bool
    pulses_l: int
    pulses_r: int
    max_heading_error_deg: float
    lateral_error_cm: float

    def __post_init__(self) -> None:
        self.passed = bool(self.passed)
        self.pulses_l = int(self.pulses_l)
        self.pulses_r = int(self.pulses_r)
        self.max_heading_error_deg = float(self.max_heading_error_deg)
        self.lateral_error_cm = float(self.lateral_error_cm)

    def __repr__(self) -> str:
        return (f"U3Result(passed={self.passed}, pulses=({self.pulses_l}, {self.pulses_r}), "
                f"max_heading_err={self.max_heading_error_deg:.2f}°, lat_err={self.lateral_error_cm:.2f}cm)")


@dataclass
class U4SubResult:
    """Measurement outcome for an individual ultrasonic distance test point."""
    target_cm: float
    measured_cm: float
    error_cm: float
    passed: bool

    def __post_init__(self) -> None:
        self.target_cm = float(self.target_cm)
        self.measured_cm = float(self.measured_cm)
        self.error_cm = float(self.error_cm)
        self.passed = bool(self.passed)

    def __repr__(self) -> str:
        return f"U4SubResult(target={self.target_cm:.1f}cm, measured={self.measured_cm:.2f}cm, err={self.error_cm:.2f}cm, passed={self.passed})"


@dataclass
class U4Result:
    """Aggregated trial results from Stage U4: ultrasonic obstacle ranging accuracy."""
    passed: bool
    sub_results: List[U4SubResult]

    def __post_init__(self) -> None:
        self.passed = bool(self.passed)
        self.sub_results = list(self.sub_results)

    def __repr__(self) -> str:
        return f"U4Result(passed={self.passed}, items={len(self.sub_results)})"


@dataclass
class CalibrationReport:
    """Complete summary report covering pre-flight hardware and kinematic stages."""
    u0: Optional[U0Report] = None
    u1: Optional[U1Result] = None
    u2: Optional[U2Result] = None
    u3: Optional[U3Result] = None
    u4: Optional[U4Result] = None

    @property
    def all_passed(self) -> bool:
        kinematics = [self.u1, self.u2, self.u3, self.u4]
        active = [k for k in kinematics if k is not None]
        if not active:
            return self.u0.all_passed if self.u0 else False
        return (self.u0.all_passed if self.u0 else True) and all(k.passed for k in active)

    def __repr__(self) -> str:
        return f"CalibrationReport(all_passed={self.all_passed}, u0={bool(self.u0)}, u1={bool(self.u1)}, u2={bool(self.u2)}, u3={bool(self.u3)}, u4={bool(self.u4)})"
