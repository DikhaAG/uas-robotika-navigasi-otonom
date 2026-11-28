"""
Autonomous Mobile Robot Calibration Package.
Polsri TC253421 - Praktikum Robotika.
"""

from tools.calibration.models import (
    CalibrationProfileData,
    CalibrationReport,
    DiagnosticResult,
    EncoderTranslationTelemetry,
    HeadingTelemetry,
    MotorPulses,
    MpuMounting,
    MotorTrimPair,
    SafetyGate,
    SafetyGateError,
    ToleranceOutcome,
    TrialRecord,
    U0Report,
    U0SubtestReport,
    U1Result,
    U1ToleranceOutcome,
    U2Result,
    U3Result,
    U4Result,
    U4SubResult,
)
from tools.calibration.formulas import (
    calculate_motor_trim_correction,
    calculate_wheel_diameter_correction,
    check_u1_tolerances,
    check_u2_tolerances,
    check_u3_tolerances,
    check_u4_tolerances,
)
from tools.calibration.adapters.transport import (
    MockRobotSerial,
    SerialTransportAdapter,
    TransportAdapter,
    autodetect_serial_port,
)
from tools.calibration.adapters.observation import (
    ConsoleObservationAdapter,
    ObservationAdapter,
    ScriptedObservationAdapter,
)
from tools.calibration.adapters.reporter import (
    AnsiConsoleReporter,
    CalibrationLogger,
    Colors,
    NullReporter,
    ReporterAdapter,
)
from tools.calibration.engine import CalibrationEngine
from tools.calibration.cli import main, show_interactive_menu

__all__ = [
    "CalibrationEngine",
    "SafetyGateError",
    "DiagnosticResult",
    "CalibrationProfileData",
    "MpuMounting",
    "MotorPulses",
    "EncoderTranslationTelemetry",
    "HeadingTelemetry",
    "MotorTrimPair",
    "U1ToleranceOutcome",
    "ToleranceOutcome",
    "TrialRecord",
    "U0SubtestReport",
    "U0Report",
    "SafetyGate",
    "U1Result",
    "U2Result",
    "U3Result",
    "U4SubResult",
    "U4Result",
    "CalibrationReport",
    "calculate_wheel_diameter_correction",
    "calculate_motor_trim_correction",
    "check_u1_tolerances",
    "check_u2_tolerances",
    "check_u3_tolerances",
    "check_u4_tolerances",
    "TransportAdapter",
    "SerialTransportAdapter",
    "MockRobotSerial",
    "autodetect_serial_port",
    "ObservationAdapter",
    "ConsoleObservationAdapter",
    "ScriptedObservationAdapter",
    "ReporterAdapter",
    "AnsiConsoleReporter",
    "NullReporter",
    "CalibrationLogger",
    "Colors",
    "show_interactive_menu",
    "main",
]
