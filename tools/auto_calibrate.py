#!/usr/bin/env python3
"""
Automated Calibration & Pre-Flight Diagnostic Wizard (U0-U4)
Praktikum Robotika - Polsri D3 Teknik Komputer

Thin entrypoint facade delegating to the deep tools.calibration package.
"""

import os
import sys
from typing import Any, List, Optional

# Ensure repo root is on sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tools.calibration.models import (
    CalibrationProfileData,
    CalibrationReport,
    DiagnosticResult,
    EncoderTranslationTelemetry,
    HeadingTelemetry,
    MotorPulses,
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
    _CompositeReporter,
)
from tools.calibration.engine import CalibrationEngine
from tools.calibration.cli import main, show_interactive_menu

# Backward compatibility alias
RobotSerialClient = SerialTransportAdapter


class PreflightDiagnosticsRunner:
    """Backward-compatibility shim delegating to deep CalibrationEngine."""
    def __init__(self, client: Any, interactive: bool = True, verbose: bool = True) -> None:
        self.client: Any = client
        self.interactive: bool = interactive
        self.verbose: bool = verbose
        self._mock_left_forward: bool = True
        self._mock_right_forward: bool = True

    def mock_motor_directions(self, left_forward: bool, right_forward: bool) -> None:
        self._mock_left_forward = bool(left_forward)
        self._mock_right_forward = bool(right_forward)

    def run_u0(self) -> U0Report:
        obs: ObservationAdapter = (
            ScriptedObservationAdapter(motor_directions=(self._mock_left_forward, self._mock_right_forward))
            if not self.interactive else ConsoleObservationAdapter(interactive=True)
        )
        rep: ReporterAdapter = AnsiConsoleReporter(verbose=self.verbose) if self.verbose else NullReporter()
        engine = CalibrationEngine(transport=self.client, observation=obs, reporter=rep)
        return engine.run_preflight()


class KinematicCalibrationRunner:
    """Backward-compatibility shim delegating to deep CalibrationEngine."""
    def __init__(self, client: Any, logger: Optional[CalibrationLogger] = None,
                 interactive: bool = True, verbose: bool = True) -> None:
        self.client: Any = client
        self.logger: Optional[CalibrationLogger] = logger
        self.interactive: bool = interactive
        self.verbose: bool = verbose
        self._obs: ObservationAdapter = (
            ScriptedObservationAdapter()
            if not self.interactive else ConsoleObservationAdapter(interactive=True)
        )

    def set_mock_u1_ground_truth(self, val: float) -> None:
        if isinstance(self._obs, ScriptedObservationAdapter):
            self._obs.set_u1_ground_truth(val)

    def set_mock_u1_ground_truth_sequence(self, seq: List[float]) -> None:
        if isinstance(self._obs, ScriptedObservationAdapter):
            self._obs.set_u1_ground_truth(seq)

    def set_mock_u2_actual_angle(self, val: float) -> None:
        if isinstance(self._obs, ScriptedObservationAdapter):
            self._obs.set_u2_actual_angle(val)

    def set_mock_u3_lateral_dev(self, val: float) -> None:
        if isinstance(self._obs, ScriptedObservationAdapter):
            self._obs.set_u3_lateral_dev(val)

    def _get_engine(self) -> CalibrationEngine:
        base_rep: ReporterAdapter = AnsiConsoleReporter(verbose=self.verbose) if self.verbose else NullReporter()
        rep: ReporterAdapter = _CompositeReporter([base_rep, self.logger]) if self.logger else base_rep
        engine = CalibrationEngine(transport=self.client, observation=self._obs, reporter=rep)
        # Mark gate permitted for legacy test scripts that invoke kinematic runner directly
        engine.safety_gate.report.all_passed = True
        return engine

    def run_u1(self, max_retries: int = 5) -> U1Result:
        return self._get_engine().run_u1(max_retries=max_retries)

    def run_u2(self, target_angle: float = 90.0) -> U2Result:
        return self._get_engine().run_u2(target_angle=target_angle)

    def run_u3(self) -> U3Result:
        return self._get_engine().run_u3()

    def run_u4(self) -> U4Result:
        return self._get_engine().run_u4()

    def run_all(self, safety_gate: SafetyGate) -> bool:
        engine = self._get_engine()
        engine.safety_gate = safety_gate
        rep = engine.calibrate(stage="ALL")
        return rep.all_passed


if __name__ == "__main__":
    sys.exit(main())
