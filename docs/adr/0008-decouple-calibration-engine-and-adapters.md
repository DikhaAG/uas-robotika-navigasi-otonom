# ADR-0008: Decouple Deep CalibrationEngine from CLI, Transport, and Observation Adapters

## Status
Accepted

## Context & Problem
Originally, `tools/auto_calibrate.py` (1,311 lines) functioned as a monolithic script combining multiple disparate responsibilities:
1. Low-level serial communication and frame handling (`RobotSerialClient`, `MockRobotSerial`, PySerial driver management).
2. ANSI terminal styling, color formatting, and interactive CLI menu loops (`Colors`, `show_interactive_menu`, `print_status`, `self.log()`).
3. Blocking console input (`input()`) for human-in-the-loop ground truth verification (tape measure, protractor, direction checks).
4. Physical motion tests, safety gating, closed-loop convergence retries, and parameter optimization formulas.
5. Telemetry persistence to disk (`CalibrationLogger`).

This entanglement created several critical architectural frictions:
- **Leaky Test Mock Backdoors in Production Classes**:
  Because interactive `input()` and physical pauses were hardcoded inside `PreflightDiagnosticsRunner` and `KinematicCalibrationRunner`, automated testing required adding private backdoor attributes (`_mock_left_forward`, `_mock_right_forward`, `_mock_u1_gt`, `_mock_u1_gt_sequence`, `_mock_u2_angle`, `_mock_u3_lat_dev`) directly onto production classes.
- **Tangled I/O and Lack of Locality**:
  Changes to terminal formatting or wording risked breaking calibration convergence or telemetry output. Testing required capturing or redirecting `sys.stdout` to suppress terminal escape sequences.
- **Lack of Architectural Symmetry with Firmware**:
  While firmware decoupled ASCII frame decoding from execution via ADR-0006 (`AsciiProtocolDecoder` and `CalibrationService`), the host tooling remained monolithic.

## Decision
We decompose the 1,311-line monolith into a modular Python package `tools/calibration/` built around a deep `CalibrationEngine` and explicit seams:

1. **`CalibrationEngine` (`tools/calibration/engine.py`)**:
   - Deep domain module encapsulating Stage U0 hardware pre-flight verification, the `SafetyGate` invariant, closed-loop kinematic convergence loops (U1–U4), closed-form parameter scaling, and EEPROM calibration profile persistence.
   - Small, high-leverage interface: `run_preflight() -> U0Report` and `calibrate(stage: str) -> CalibrationReport`.
   - Zero terminal `print()`, zero `input()`, zero mock backdoor fields.

2. **Seams and Concrete Adapters (`tools/calibration/adapters/`)**:
   - **`TransportAdapter` (`adapters/transport.py`)**:
     Seam for wire communication satisfied by `SerialTransportAdapter` (physical PySerial) and `MockRobotSerial` (in-memory simulator).
   - **`ObservationAdapter` (`adapters/observation.py`)**:
     Seam for ground-truth data entry satisfied by `ConsoleObservationAdapter` (interactive human `input()`) in production and `ScriptedObservationAdapter` (deterministic canned sequences) in automated tests, completely eliminating mock backdoors.
   - **`ReporterAdapter` (`adapters/reporter.py`)**:
     Seam for progress and telemetry reporting satisfied by `AnsiConsoleReporter` (interactive colored CLI), `NullReporter` (silent testing), and `CalibrationLogger` (CSV file persistence).

3. **Domain Models & Pure Formulas (`tools/calibration/models.py`, `formulas.py`)**:
   - Dataclasses (`DiagnosticResult`, `CalibrationProfileData`, `U0Report`, `SafetyGate`, `CalibrationReport`) and pure math functions (`calculate_wheel_diameter_correction`, `calculate_motor_trim_correction`, `check_u1_tolerances`, etc.).

4. **Backward-Compatible Entrypoint Facade (`tools/auto_calibrate.py`)**:
   - Reduced from 1,311 lines to ~130 lines.
   - Re-exports all public symbols, aliases `RobotSerialClient = SerialTransportAdapter`, and provides delegation shims for `PreflightDiagnosticsRunner` and `KinematicCalibrationRunner` so external scripts and existing tests continue running seamlessly.

## Consequences & Benefits
- **Zero Mock Backdoors**: Production classes contain no testing hooks or mock attributes.
- **High Depth & Leverage**: `CalibrationEngine` presents a minimal interface with deep internal behavior, reusable across interactive CLI, automated CI, and future GUI dashboards.
- **Clean Test Surface**: Unit tests test `CalibrationEngine` directly across its public seams without `sys.stdout` redirection or terminal pollution.
- **100% Backward Compatibility**: All CLI arguments (`--mock`, `--headless`, `--test`, `--port`) and existing test commands (`make check`) work without modification.
