#!/usr/bin/env python3
"""
Unit tests for Automated Calibration Wizard (tools/auto_calibrate.py)
Validates public interfaces and seams:
- Seam 1: Serial Protocol Client (command formatting & response parsing)
- Seam 2: Stage U0 Pre-Flight Diagnostics (MPU6050, Sonar, IR, Beep, Motors/Encoders)
- Seam 3: Hard-Gated Safety & Wiring Diagnostic Advice
- Seam 4: CLI Headless / Mock Execution
"""

import contextlib
import io
import os
import sys
import unittest
from typing import Dict, List, Optional

# Add repo root to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tools.auto_calibrate import (
    CalibrationEngine,
    CalibrationLogger,
    CalibrationProfileData,
    DiagnosticResult,
    KinematicCalibrationRunner,
    MockRobotSerial,
    NullReporter,
    PreflightDiagnosticsRunner,
    RobotSerialClient,
    SafetyGate,
    SafetyGateError,
    ScriptedObservationAdapter,
    calculate_motor_trim_correction,
    calculate_wheel_diameter_correction,
    check_u1_tolerances,
    check_u2_tolerances,
    check_u3_tolerances,
    check_u4_tolerances,
    main,
    show_interactive_menu,
)
from tools.calibration.adapters.transport import response_timeout_s


class FakeArduinoSerial:
    """
    pyserial stand-in that behaves like a real Uno link: readline() returns b""
    when its per-read timeout expires, replies only appear after the command is
    written, and reset_input_buffer() drops bytes that are already waiting.
    """
    def __init__(self, replies: Dict[str, List[bytes]], stale: Optional[List[bytes]] = None) -> None:
        self.replies: Dict[str, List[bytes]] = replies
        self.buffer: List[bytes] = list(stale or [])

    def write(self, data: bytes) -> None:
        cmd = data.decode("ascii").strip()
        self.buffer.extend(self.replies.get(cmd, []))

    def readline(self) -> bytes:
        return self.buffer.pop(0) if self.buffer else b""

    def reset_input_buffer(self) -> None:
        self.buffer.clear()

    def close(self) -> None:
        pass


class TestSerialTransportTiming(unittest.TestCase):
    def test_long_step_reply_survives_read_timeouts_and_log_lines(self) -> None:
        # !STEP:U1 drives 100 cm (several seconds): pyserial times out on the
        # first reads and the firmware may print a '#' log line before replying.
        backend = FakeArduinoSerial({"!STEP:U1": [b"", b"", b"# STATE -> IDLE\r\n", b"!U1:DONE:310:305:100.40\r\n"]})
        client = RobotSerialClient(port=None, baud=115200, serial_backend=backend)
        telemetry = client.step_u1()
        self.assertEqual((telemetry.pulses_l, telemetry.pulses_r), (310, 305))
        self.assertAlmostEqual(telemetry.firmware_dist_cm, 100.40, places=2)

    def test_stale_late_reply_is_discarded_before_next_command(self) -> None:
        backend = FakeArduinoSerial({"!PING": [b"!PONG\r\n"]}, stale=[b"!U1:DONE:310:305:100.40\r\n"])
        client = RobotSerialClient(port=None, baud=115200, serial_backend=backend)
        self.assertTrue(client.ping())

    def test_response_timeout_covers_firmware_motion_limits(self) -> None:
        # Firmware gives up after 8 s (U1), 5 s (U2) and 12 s (U3); the host must wait longer.
        self.assertGreater(response_timeout_s("!STEP:U1"), 8.0)
        self.assertGreater(response_timeout_s("!STEP:U2:90.00"), 5.0)
        self.assertGreater(response_timeout_s("!STEP:U3"), 12.0)
        self.assertGreater(response_timeout_s("!DIAG:MOTOR:140:0:4000"), 4.0)
        self.assertGreater(response_timeout_s("!DIAG:BEEP:3000"), 3.0)
        self.assertGreaterEqual(response_timeout_s("!PING"), 1.5)

    def test_wizard_default_baud_matches_firmware_serial_baud(self) -> None:
        # The wizard must open the port at the firmware's Config.h SERIAL_BAUD,
        # which follows the HC-05 factory baud so Bluetooth works without AT setup.
        import re
        from tools.calibration.adapters.transport import DEFAULT_BAUD
        config_path = os.path.join(os.path.dirname(__file__), "..", "firmware", "RobotNavigasiOtonom", "Config.h")
        with open(config_path, encoding="utf-8") as f:
            match = re.search(r"#define\s+SERIAL_BAUD\s+(\d+)UL", f.read())
        self.assertIsNotNone(match)
        self.assertEqual(DEFAULT_BAUD, int(match.group(1)))
        self.assertEqual(DEFAULT_BAUD, 9600)


class TestSerialProtocolClient(unittest.TestCase):
    def setUp(self) -> None:
        self.mock_serial = MockRobotSerial()
        self.client = RobotSerialClient(port=None, baud=115200, serial_backend=self.mock_serial)

    def test_ping_success(self) -> None:
        self.assertTrue(self.client.ping())

    def test_probe_diagnostics(self) -> None:
        diag = self.client.probe_diagnostics()
        self.assertIsInstance(diag, DiagnosticResult)
        self.assertTrue(diag.mpu_ok)
        self.assertAlmostEqual(diag.sonar_dist_cm, 25.0, places=1)
        self.assertFalse(diag.ir_left)
        self.assertFalse(diag.ir_right)

    def test_spin_motors(self) -> None:
        pulses_l, pulses_r = self.client.spin_motors(140, 140, 500)
        self.assertGreater(pulses_l, 0)
        self.assertGreater(pulses_r, 0)

    def test_buzzer(self) -> None:
        self.assertTrue(self.client.test_buzzer(200))

    def test_read_and_write_profile(self) -> None:
        profile = self.client.read_profile()
        self.assertIsInstance(profile, CalibrationProfileData)
        self.assertAlmostEqual(profile.diameter_roda_mm, 65.0, places=1)
        self.assertEqual(profile.invert_motor_kiri, 1)
        self.assertEqual(profile.invert_motor_kanan, 1)

        profile.invert_motor_kiri = -1
        success = self.client.write_profile(profile)
        self.assertTrue(success)

        updated = self.client.read_profile()
        self.assertEqual(updated.invert_motor_kiri, -1)


class TestPreflightDiagnostics(unittest.TestCase):
    def setUp(self) -> None:
        self.mock_serial = MockRobotSerial()
        self.client = RobotSerialClient(port=None, baud=115200, serial_backend=self.mock_serial)
        self.runner = PreflightDiagnosticsRunner(self.client, interactive=False, verbose=False)

    def test_u0_all_pass(self) -> None:
        report = self.runner.run_u0()
        self.assertTrue(report.all_passed)
        self.assertTrue(report.u01_mpu_passed)
        self.assertTrue(report.u02_sonar_passed)
        self.assertTrue(report.u03_ir_passed)
        self.assertTrue(report.u04_indicators_passed)
        self.assertTrue(report.u05_motors_passed)

    def test_u0_mpu_fail_gives_wiring_hint(self) -> None:
        self.mock_serial.set_mpu_ok(False)
        report = self.runner.run_u0()
        self.assertFalse(report.all_passed)
        self.assertFalse(report.u01_mpu_passed)
        hints = report.get_wiring_repair_instructions()
        self.assertTrue(any("A4" in h and "A5" in h for h in hints))

    def test_u0_sonar_fail_gives_wiring_hint(self) -> None:
        self.mock_serial.set_sonar_dist(-1.0)
        report = self.runner.run_u0()
        self.assertFalse(report.all_passed)
        self.assertFalse(report.u02_sonar_passed)
        hints = report.get_wiring_repair_instructions()
        self.assertTrue(any("A0" in h and "A1" in h for h in hints))

    def test_u0_motor_zero_pulses_detected(self) -> None:
        self.mock_serial.set_motor_pulses(0, 50)
        report = self.runner.run_u0()
        self.assertFalse(report.all_passed)
        self.assertFalse(report.u05_motors_passed)
        hints = report.get_wiring_repair_instructions()
        self.assertTrue(any("D2" in h or "encoder" in h.lower() for h in hints))

    def test_u0_software_polarity_inversion_applied(self) -> None:
        self.runner.mock_motor_directions(left_forward=False, right_forward=True)
        report = self.runner.run_u0()
        self.assertTrue(report.polarity_inversion_applied)
        profile = self.client.read_profile()
        self.assertEqual(profile.invert_motor_kiri, -1)
        self.assertEqual(profile.invert_motor_kanan, 1)


class TestHardGating(unittest.TestCase):
    def test_hard_gate_blocks_when_u0_fails(self) -> None:
        mock_serial = MockRobotSerial()
        mock_serial.set_mpu_ok(False)
        client = RobotSerialClient(port=None, baud=115200, serial_backend=mock_serial)
        runner = PreflightDiagnosticsRunner(client, interactive=False, verbose=False)
        report = runner.run_u0()

        gate = SafetyGate(report)
        self.assertFalse(gate.is_motion_permitted())
        with self.assertRaises(SafetyGateError):
            gate.assert_motion_permitted()

    def test_hard_gate_allows_when_u0_passes(self) -> None:
        mock_serial = MockRobotSerial()
        client = RobotSerialClient(port=None, baud=115200, serial_backend=mock_serial)
        runner = PreflightDiagnosticsRunner(client, interactive=False, verbose=False)
        report = runner.run_u0()

        gate = SafetyGate(report)
        self.assertTrue(gate.is_motion_permitted())
        gate.assert_motion_permitted()


class TestCLIHeadlessExecution(unittest.TestCase):
    def test_cli_mock_pass(self) -> None:
        with contextlib.redirect_stdout(io.StringIO()):
            exit_code = main(["--mock", "--headless"])
        self.assertEqual(exit_code, 0)

    def test_cli_mock_fail_mpu(self) -> None:
        with contextlib.redirect_stdout(io.StringIO()):
            exit_code = main(["--mock", "--headless", "--fail-mpu"])
        self.assertNotEqual(exit_code, 0)

    def test_show_interactive_menu_exported(self) -> None:
        self.assertTrue(callable(show_interactive_menu))


class TestKinematicCalculations(unittest.TestCase):
    def test_wheel_diameter_correction(self) -> None:
        new_diam = calculate_wheel_diameter_correction(65.0, 102.0, 100.0)
        self.assertAlmostEqual(new_diam, 65.0 * (102.0 / 100.0), places=3)

    def test_motor_trim_correction_left_faster(self) -> None:
        trim_l, trim_r = calculate_motor_trim_correction(1.0, 1.0, 105, 95)
        self.assertAlmostEqual(trim_l, 95.0 / 105.0, places=3)
        self.assertAlmostEqual(trim_r, 1.0, places=3)

    def test_motor_trim_correction_right_faster(self) -> None:
        trim_l, trim_r = calculate_motor_trim_correction(1.0, 1.0, 95, 105)
        self.assertAlmostEqual(trim_l, 1.0, places=3)
        self.assertAlmostEqual(trim_r, 95.0 / 105.0, places=3)

    def test_motor_trim_clamping(self) -> None:
        trim_l, trim_r = calculate_motor_trim_correction(1.0, 1.0, 200, 50)
        self.assertGreaterEqual(trim_l, 0.75)
        self.assertLessEqual(trim_l, 1.25)

    def test_u1_tolerances(self) -> None:
        passed, dist_err, drift = check_u1_tolerances(101.5, 98, 99)
        self.assertTrue(passed)
        self.assertAlmostEqual(dist_err, 1.5, places=2)
        self.assertLessEqual(drift, 3.0)

        passed, dist_err, _ = check_u1_tolerances(104.5, 98, 98)
        self.assertFalse(passed)

        passed, _, drift = check_u1_tolerances(100.0, 98, 88)
        self.assertFalse(passed)
        self.assertGreater(drift, 3.0)

    def test_u2_tolerances(self) -> None:
        passed, err = check_u2_tolerances(92.0, 90.0)
        self.assertTrue(passed)
        self.assertAlmostEqual(err, 2.0, places=2)

        passed, err = check_u2_tolerances(96.5, 90.0)
        self.assertFalse(passed)
        self.assertAlmostEqual(err, 6.5, places=2)

        passed, err = check_u2_tolerances(-90.0, 90.0)
        self.assertFalse(passed)
        self.assertAlmostEqual(err, 180.0, places=2)

    def test_u3_tolerances(self) -> None:
        passed, err = check_u3_tolerances(3.2)
        self.assertTrue(passed)
        self.assertAlmostEqual(err, 3.2, places=2)

        passed, err = check_u3_tolerances(5.8)
        self.assertFalse(passed)
        self.assertAlmostEqual(err, 5.8, places=2)

    def test_u4_tolerances(self) -> None:
        passed, err = check_u4_tolerances(10.4, 10.0)
        self.assertTrue(passed)
        self.assertAlmostEqual(err, 0.4, places=2)

        passed, err = check_u4_tolerances(11.5, 10.0)
        self.assertFalse(passed)
        self.assertAlmostEqual(err, 1.5, places=2)


class TestKinematicProtocols(unittest.TestCase):
    def setUp(self) -> None:
        self.mock_serial = MockRobotSerial()
        self.client = RobotSerialClient(port=None, baud=115200, serial_backend=self.mock_serial)
        self.runner = KinematicCalibrationRunner(self.client, interactive=False, verbose=False)

    def test_run_u1_success_single_pass(self) -> None:
        self.runner.set_mock_u1_ground_truth(100.5)
        res = self.runner.run_u1(max_retries=3)
        self.assertTrue(res.passed)
        self.assertEqual(res.iteration, 1)
        self.assertAlmostEqual(res.dist_error_cm, 0.5, places=2)

    def test_run_u1_convergence_multi_trial(self) -> None:
        self.runner.set_mock_u1_ground_truth_sequence([105.0, 101.0])
        res = self.runner.run_u1(max_retries=3)
        self.assertTrue(res.passed)
        self.assertEqual(res.iteration, 2)
        profile = self.client.read_profile()
        self.assertNotEqual(profile.diameter_roda_mm, 65.0)

    def test_run_u1_max_retries_exceeded(self) -> None:
        self.runner.set_mock_u1_ground_truth(110.0)
        res = self.runner.run_u1(max_retries=3)
        self.assertFalse(res.passed)
        self.assertEqual(res.iteration, 3)

    def test_run_u2_in_place_rotation(self) -> None:
        self.runner.set_mock_u2_actual_angle(91.2)
        res = self.runner.run_u2(target_angle=90.0)
        self.assertTrue(res.passed)
        self.assertAlmostEqual(res.angular_error_deg, 1.2, places=2)

    def test_run_u3_straight_line(self) -> None:
        self.runner.set_mock_u3_lateral_dev(2.1)
        res = self.runner.run_u3()
        self.assertTrue(res.passed)
        self.assertAlmostEqual(res.lateral_error_cm, 2.1, places=2)

    def test_run_u4_ranging_accuracy(self) -> None:
        res = self.runner.run_u4()
        self.assertTrue(res.passed)
        self.assertEqual(len(res.sub_results), 3)


class TestCalibrationLogger(unittest.TestCase):
    def test_log_export(self) -> None:
        log_path = "tests/test_log_tmp.csv"
        if os.path.exists(log_path):
            os.remove(log_path)

        logger = CalibrationLogger(filepath=log_path)
        logger.log_result(
            test_id="U1",
            iteration=1,
            status="PASS",
            target=100.0,
            measured=100.12,
            ground_truth=100.5,
            error=0.5,
            profile=CalibrationProfileData(diameter_roda_mm=65.25),
            notes="Toleransi terpenuhi",
        )

        self.assertTrue(os.path.exists(log_path))
        with open(log_path, "r", encoding="utf-8") as f:
            lines = f.readlines()
        self.assertGreaterEqual(len(lines), 2)
        self.assertIn("timestamp,test_id,iteration", lines[0])
        self.assertIn("U1,1,PASS,100.00,100.12,100.50,0.50", lines[1])

        if os.path.exists(log_path):
            os.remove(log_path)


class TestSafetyGatingWithKinematics(unittest.TestCase):
    def test_kinematic_run_aborted_when_gate_locked(self) -> None:
        with self.assertRaises(SafetyGateError):
            with contextlib.redirect_stdout(io.StringIO()):
                main(["--mock", "--headless", "--test", "U1", "--fail-mpu"])


class TestDeepCalibrationEngine(unittest.TestCase):
    """Direct verification of the deep CalibrationEngine module across its seams."""

    def setUp(self) -> None:
        self.mock_serial = MockRobotSerial()
        self.transport = RobotSerialClient(port=None, baud=115200, serial_backend=self.mock_serial)
        self.observation = ScriptedObservationAdapter()
        self.reporter = NullReporter()
        self.engine = CalibrationEngine(
            transport=self.transport,
            observation=self.observation,
            reporter=self.reporter,
        )

    def test_engine_preflight_success(self) -> None:
        report = self.engine.run_preflight()
        self.assertTrue(report.all_passed)
        self.assertTrue(self.engine.is_motion_permitted())

    def test_engine_preflight_blocks_motion_on_mpu_failure(self) -> None:
        self.mock_serial.set_mpu_ok(False)
        report = self.engine.run_preflight()
        self.assertFalse(report.all_passed)
        self.assertFalse(self.engine.is_motion_permitted())
        with self.assertRaises(SafetyGateError):
            self.engine.calibrate(stage="U1")

    def test_engine_preflight_polarity_inversion(self) -> None:
        self.observation.set_motor_directions(left_forward=False, right_forward=True)
        report = self.engine.run_preflight()
        self.assertTrue(report.all_passed)
        self.assertTrue(report.polarity_inversion_applied)
        prof = self.engine.get_active_profile()
        self.assertEqual(prof.invert_motor_kiri, -1)

    def test_engine_u1_convergence_multi_trial(self) -> None:
        self.engine.run_preflight()
        self.observation.set_u1_ground_truth([105.0, 100.5])
        res = self.engine.run_u1(max_retries=3)
        self.assertTrue(res.passed)
        self.assertEqual(res.iteration, 2)
        prof = self.engine.get_active_profile()
        self.assertNotEqual(prof.diameter_roda_mm, 65.0)

    def test_engine_u2_in_place_rotation(self) -> None:
        self.engine.run_preflight()
        self.observation.set_u2_actual_angle(91.5)
        res = self.engine.run_u2(target_angle=90.0)
        self.assertTrue(res.passed)
        self.assertAlmostEqual(res.angular_error_deg, 1.5, places=2)

    def test_engine_u3_straight_line(self) -> None:
        self.engine.run_preflight()
        self.observation.set_u3_lateral_dev(1.8)
        res = self.engine.run_u3()
        self.assertTrue(res.passed)
        self.assertAlmostEqual(res.lateral_error_cm, 1.8, places=2)

    def test_engine_u4_ranging_accuracy(self) -> None:
        self.engine.run_preflight()
        res = self.engine.run_u4()
        self.assertTrue(res.passed)
        self.assertEqual(len(res.sub_results), 3)

    def test_engine_calibrate_all_success(self) -> None:
        self.engine.run_preflight()
        self.observation.set_u1_ground_truth(100.5)
        self.observation.set_u2_actual_angle(90.5)
        self.observation.set_u3_lateral_dev(1.2)
        rep = self.engine.calibrate(stage="ALL")
        self.assertTrue(rep.all_passed)
        self.assertIsNotNone(rep.u1)
        self.assertIsNotNone(rep.u2)
        self.assertIsNotNone(rep.u3)
        self.assertIsNotNone(rep.u4)

    def test_engine_accepts_mock_serial_and_logger_directly_as_adapters(self) -> None:
        log_path = "tests/test_engine_adapter_tmp.csv"
        if os.path.exists(log_path):
            os.remove(log_path)

        try:
            mock_transport = MockRobotSerial()
            logger_reporter = CalibrationLogger(filepath=log_path)
            engine = CalibrationEngine(
                transport=mock_transport,
                observation=ScriptedObservationAdapter(u1_ground_truth=100.5),
                reporter=logger_reporter,
            )
            u0_report = engine.run_preflight()
            self.assertTrue(u0_report.all_passed)
            rep = engine.calibrate(stage="ALL")
            self.assertTrue(rep.all_passed)

            with open(log_path, "r", encoding="utf-8") as f:
                content = f.read()
            self.assertIn("U0.1 MPU6050", content)
            self.assertIn("U1,1,PASS", content)
            self.assertIn("U4.3,1,PASS", content)
        finally:
            if os.path.exists(log_path):
                os.remove(log_path)

    def test_cli_mock_fail_mpu_without_headless_is_non_interactive(self) -> None:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            exit_code = main(["--mock", "--fail-mpu", "--log-file", "tests/test_tmp_fail_mpu.csv"])
        if os.path.exists("tests/test_tmp_fail_mpu.csv"):
            os.remove("tests/test_tmp_fail_mpu.csv")
        self.assertEqual(exit_code, 1)

    def test_plot_trajectory_parse_obstacle_args(self) -> None:
        from tools.plot_trajectory import parse_obstacle_args
        self.assertIsNone(parse_obstacle_args(None))
        self.assertEqual(parse_obstacle_args(["none"]), [])
        self.assertEqual(parse_obstacle_args(["110,0,20,20", "80,0,20,20"]), [(110.0, 0.0, 20.0, 20.0), (80.0, 0.0, 20.0, 20.0)])


if __name__ == "__main__":
    unittest.main()
