"""
Autonomous Mobile Robot Calibration Package.
Polsri TC253421 - Praktikum Robotika.
"""

from tools.calibration.models import (
    SafetyGateError,
    DiagnosticResult,
    CalibrationProfileData,
    U0SubtestReport,
    U0Report,
    SafetyGate,
    U1Result,
    U2Result,
    U3Result,
    U4SubResult,
    U4Result,
    CalibrationReport,
)
from tools.calibration.formulas import (
    calculate_wheel_diameter_correction,
    calculate_motor_trim_correction,
    check_u1_tolerances,
    check_u2_tolerances,
    check_u3_tolerances,
    check_u4_tolerances,
)
from tools.calibration.adapters.transport import (
    TransportAdapter,
    SerialTransportAdapter,
    MockRobotSerial,
    autodetect_serial_port,
)
from tools.calibration.adapters.observation import (
    ObservationAdapter,
    ConsoleObservationAdapter,
    ScriptedObservationAdapter,
)
from tools.calibration.adapters.reporter import (
    ReporterAdapter,
    AnsiConsoleReporter,
    NullReporter,
    CalibrationLogger,
    CompositeReporter,
    Colors,
)
from tools.calibration.engine import CalibrationEngine
from tools.calibration.cli import main

__all__ = [
    "CalibrationEngine",
    "SafetyGateError",
    "DiagnosticResult",
    "CalibrationProfileData",
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
    "CompositeReporter",
    "Colors",
    "main",
]
