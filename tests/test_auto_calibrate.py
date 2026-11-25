#!/usr/bin/env python3
"""
Unit tests for Automated Calibration Wizard (tools/auto_calibrate.py)
Validates public interfaces and seams:
- Seam 1: Serial Protocol Client (command formatting & response parsing)
- Seam 2: Stage U0 Pre-Flight Diagnostics (MPU6050, Sonar, IR, Beep, Motors/Encoders)
- Seam 3: Hard-Gated Safety & Wiring Diagnostic Advice
- Seam 4: CLI Headless / Mock Execution
"""

import unittest
import sys
import os
import io
import contextlib

# Add repo root to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tools.auto_calibrate import (
    RobotSerialClient,
    DiagnosticResult,
    CalibrationProfileData,
    PreflightDiagnosticsRunner,
    SafetyGate,
    SafetyGateError,
    MockRobotSerial,
    calculate_wheel_diameter_correction,
    calculate_motor_trim_correction,
    check_u1_tolerances,
    check_u2_tolerances,
    check_u3_tolerances,
    check_u4_tolerances,
    CalibrationLogger,
    KinematicCalibrationRunner,
    main,
    CalibrationEngine,
    ScriptedObservationAdapter,
    NullReporter,
)


class TestSerialProtocolClient(unittest.TestCase):
    def setUp(self):
        self.mock_serial = MockRobotSerial()
        self.client = RobotSerialClient(port=None, baud=115200, serial_backend=self.mock_serial)

    def test_ping_success(self):
        self.assertTrue(self.client.ping())

    def test_probe_diagnostics(self):
        diag = self.client.probe_diagnostics()
        self.assertIsInstance(diag, DiagnosticResult)
        self.assertTrue(diag.mpu_ok)
        self.assertAlmostEqual(diag.sonar_dist_cm, 25.0, places=1)
        self.assertFalse(diag.ir_left)
        self.assertFalse(diag.ir_right)

    def test_spin_motors(self):
        pulses_l, pulses_r = self.client.spin_motors(140, 140, 500)
        self.assertGreater(pulses_l, 0)
        self.assertGreater(pulses_r, 0)

    def test_buzzer(self):
        self.assertTrue(self.client.test_buzzer(200))

    def test_read_and_write_profile(self):
        profile = self.client.read_profile()
        self.assertIsInstance(profile, CalibrationProfileData)
        self.assertAlmostEqual(profile.diameter_roda_mm, 65.0, places=1)
        self.assertEqual(profile.invert_motor_kiri, 1)
        self.assertEqual(profile.invert_motor_kanan, 1)

        # Invert left motor and write back
        profile.invert_motor_kiri = -1
        success = self.client.write_profile(profile)
        self.assertTrue(success)

        # Read back and verify
        updated = self.client.read_profile()
        self.assertEqual(updated.invert_motor_kiri, -1)


class TestPreflightDiagnostics(unittest.TestCase):
    def setUp(self):
        self.mock_serial = MockRobotSerial()
        self.client = RobotSerialClient(port=None, baud=115200, serial_backend=self.mock_serial)
        self.runner = PreflightDiagnosticsRunner(self.client, interactive=False, verbose=False)

    def test_u0_all_pass(self):
        report = self.runner.run_u0()
        self.assertTrue(report.all_passed)
        self.assertTrue(report.u01_mpu_passed)
        self.assertTrue(report.u02_sonar_passed)
        self.assertTrue(report.u03_ir_passed)
        self.assertTrue(report.u04_indicators_passed)
        self.assertTrue(report.u05_motors_passed)

    def test_u0_mpu_fail_gives_wiring_hint(self):
        self.mock_serial.set_mpu_ok(False)
        report = self.runner.run_u0()
        self.assertFalse(report.all_passed)
        self.assertFalse(report.u01_mpu_passed)
        hints = report.get_wiring_repair_instructions()
        self.assertTrue(any("A4" in h and "A5" in h for h in hints))

    def test_u0_sonar_fail_gives_wiring_hint(self):
        self.mock_serial.set_sonar_dist(-1.0)
        report = self.runner.run_u0()
        self.assertFalse(report.all_passed)
        self.assertFalse(report.u02_sonar_passed)
        hints = report.get_wiring_repair_instructions()
        self.assertTrue(any("A0" in h and "A1" in h for h in hints))

    def test_u0_motor_zero_pulses_detected(self):
        self.mock_serial.set_motor_pulses(0, 50)
        report = self.runner.run_u0()
        self.assertFalse(report.all_passed)
        self.assertFalse(report.u05_motors_passed)
        hints = report.get_wiring_repair_instructions()
        self.assertTrue(any("D2" in h or "encoder" in h.lower() for h in hints))

    def test_u0_software_polarity_inversion_applied(self):
        # Configure mock runner to signal left wheel was reversed
        self.runner.mock_motor_directions(left_forward=False, right_forward=True)
        report = self.runner.run_u0()
        self.assertTrue(report.polarity_inversion_applied)
        # Check that profile now has invert_motor_kiri = -1
        profile = self.client.read_profile()
        self.assertEqual(profile.invert_motor_kiri, -1)
        self.assertEqual(profile.invert_motor_kanan, 1)


class TestHardGating(unittest.TestCase):
    def test_hard_gate_blocks_when_u0_fails(self):
        mock_serial = MockRobotSerial()
        mock_serial.set_mpu_ok(False)
        client = RobotSerialClient(port=None, baud=115200, serial_backend=mock_serial)
        runner = PreflightDiagnosticsRunner(client, interactive=False, verbose=False)
        report = runner.run_u0()

        gate = SafetyGate(report)
        self.assertFalse(gate.is_motion_permitted())
        with self.assertRaises(SafetyGateError):
            gate.assert_motion_permitted()

    def test_hard_gate_allows_when_u0_passes(self):
        mock_serial = MockRobotSerial()
        client = RobotSerialClient(port=None, baud=115200, serial_backend=mock_serial)
        runner = PreflightDiagnosticsRunner(client, interactive=False, verbose=False)
        report = runner.run_u0()

        gate = SafetyGate(report)
        self.assertTrue(gate.is_motion_permitted())
        # Should not raise exception
        gate.assert_motion_permitted()


class TestCLIHeadlessExecution(unittest.TestCase):
    def test_cli_mock_pass(self):
        with contextlib.redirect_stdout(io.StringIO()):
            exit_code = main(["--mock", "--headless"])
        self.assertEqual(exit_code, 0)

    def test_cli_mock_fail_mpu(self):
        with contextlib.redirect_stdout(io.StringIO()):
            exit_code = main(["--mock", "--headless", "--fail-mpu"])
        self.assertNotEqual(exit_code, 0)


class TestKinematicCalculations(unittest.TestCase):
    def test_wheel_diameter_correction(self):
        # Current diameter 65.0 mm. Actual ground truth: 102.0 cm. Firmware measured: 100.0 cm.
        new_diam = calculate_wheel_diameter_correction(65.0, 102.0, 100.0)
        self.assertAlmostEqual(new_diam, 65.0 * (102.0 / 100.0), places=3)

    def test_motor_trim_correction_left_faster(self):
        # Left wheel produced 105 pulses, right produced 95 pulses
        # Left motor should be trimmed down by 95 / 105
        trim_l, trim_r = calculate_motor_trim_correction(1.0, 1.0, 105, 95)
        self.assertAlmostEqual(trim_l, 95.0 / 105.0, places=3)
        self.assertAlmostEqual(trim_r, 1.0, places=3)

    def test_motor_trim_correction_right_faster(self):
        # Right wheel produced 105 pulses, left produced 95 pulses
        trim_l, trim_r = calculate_motor_trim_correction(1.0, 1.0, 95, 105)
        self.assertAlmostEqual(trim_l, 1.0, places=3)
        self.assertAlmostEqual(trim_r, 95.0 / 105.0, places=3)

    def test_motor_trim_clamping(self):
        # Severe disparity should clamp to [0.75, 1.25]
        trim_l, trim_r = calculate_motor_trim_correction(1.0, 1.0, 200, 50)
        self.assertGreaterEqual(trim_l, 0.75)
        self.assertLessEqual(trim_l, 1.25)

    def test_u1_tolerances(self):
        # 101.5 cm with 98 and 99 pulses -> pass (error 1.5 cm <= 3.0 cm, drift 1.01% <= 3%)
        passed, dist_err, drift = check_u1_tolerances(101.5, 98, 99)
        self.assertTrue(passed)
        self.assertAlmostEqual(dist_err, 1.5, places=2)
        self.assertLessEqual(drift, 3.0)

        # 104.5 cm -> fail (error 4.5 cm > 3.0 cm)
        passed, dist_err, _ = check_u1_tolerances(104.5, 98, 98)
        self.assertFalse(passed)

        # 100.0 cm but pulses 98 vs 88 -> drift > 3% -> fail
        passed, _, drift = check_u1_tolerances(100.0, 98, 88)
        self.assertFalse(passed)
        self.assertGreater(drift, 3.0)

    def test_u2_tolerances(self):
        passed, err = check_u2_tolerances(92.0, 90.0)
        self.assertTrue(passed)
        self.assertAlmostEqual(err, 2.0, places=2)

        passed, err = check_u2_tolerances(96.5, 90.0)
        self.assertFalse(passed)
        self.assertAlmostEqual(err, 6.5, places=2)

        # Inverted turn (-90 deg when +90 requested) must fail
        passed, err = check_u2_tolerances(-90.0, 90.0)
        self.assertFalse(passed)
        self.assertAlmostEqual(err, 180.0, places=2)

    def test_u3_tolerances(self):
        passed, err = check_u3_tolerances(3.2)
        self.assertTrue(passed)
        self.assertAlmostEqual(err, 3.2, places=2)

        passed, err = check_u3_tolerances(5.8)
        self.assertFalse(passed)
        self.assertAlmostEqual(err, 5.8, places=2)

    def test_u4_tolerances(self):
        passed, err = check_u4_tolerances(10.4, 10.0)
        self.assertTrue(passed)
        self.assertAlmostEqual(err, 0.4, places=2)

        passed, err = check_u4_tolerances(11.5, 10.0)
        self.assertFalse(passed)
        self.assertAlmostEqual(err, 1.5, places=2)


class TestKinematicProtocols(unittest.TestCase):
    def setUp(self):
        self.mock_serial = MockRobotSerial()
        self.client = RobotSerialClient(port=None, baud=115200, serial_backend=self.mock_serial)
        self.runner = KinematicCalibrationRunner(self.client, interactive=False, verbose=False)

    def test_run_u1_success_single_pass(self):
        # Configure runner mock ground truth: 100.5 cm
        self.runner.set_mock_u1_ground_truth(100.5)
        res = self.runner.run_u1(max_retries=3)
        self.assertTrue(res.passed)
        self.assertEqual(res.iteration, 1)
        self.assertAlmostEqual(res.dist_error_cm, 0.5, places=2)

    def test_run_u1_convergence_multi_trial(self):
        # Trial 1: ground truth 105.0 cm (fail, updates profile)
        # Trial 2: ground truth 101.0 cm (pass)
        self.runner.set_mock_u1_ground_truth_sequence([105.0, 101.0])
        res = self.runner.run_u1(max_retries=3)
        self.assertTrue(res.passed)
        self.assertEqual(res.iteration, 2)
        # Verify EEPROM profile was updated
        profile = self.client.read_profile()
        self.assertNotEqual(profile.diameter_roda_mm, 65.0)

    def test_run_u1_max_retries_exceeded(self):
        # Continuous large errors exceeding max retries
        self.runner.set_mock_u1_ground_truth(110.0)
        res = self.runner.run_u1(max_retries=3)
        self.assertFalse(res.passed)
        self.assertEqual(res.iteration, 3)

    def test_run_u2_in_place_rotation(self):
        self.runner.set_mock_u2_actual_angle(91.2)
        res = self.runner.run_u2(target_angle=90.0)
        self.assertTrue(res.passed)
        self.assertAlmostEqual(res.angular_error_deg, 1.2, places=2)

    def test_run_u3_straight_line(self):
        self.runner.set_mock_u3_lateral_dev(2.1)
        res = self.runner.run_u3()
        self.assertTrue(res.passed)
        self.assertAlmostEqual(res.lateral_error_cm, 2.1, places=2)

    def test_run_u4_ranging_accuracy(self):
        # Sonar distance matches targets (10, 20, 40) within 1.0 cm
        res = self.runner.run_u4()
        self.assertTrue(res.passed)
        self.assertEqual(len(res.sub_results), 3)


class TestCalibrationLogger(unittest.TestCase):
    def test_log_export(self):
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
            notes="Toleransi terpenuhi"
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
    def test_kinematic_run_aborted_when_gate_locked(self):
        mock_serial = MockRobotSerial()
        mock_serial.set_mpu_ok(False)  # Force U0 failure
        client = RobotSerialClient(port=None, baud=115200, serial_backend=mock_serial)
        
        # When attempting to run kinematic test directly without passing U0
        with self.assertRaises(SafetyGateError):
            with contextlib.redirect_stdout(io.StringIO()):
                main(["--mock", "--headless", "--test", "U1", "--fail-mpu"])


class TestDeepCalibrationEngine(unittest.TestCase):
    """Direct verification of the deep CalibrationEngine module across its seams."""

    def setUp(self):
        self.mock_serial = MockRobotSerial()
        self.transport = RobotSerialClient(port=None, baud=115200, serial_backend=self.mock_serial)
        self.observation = ScriptedObservationAdapter()
        self.reporter = NullReporter()
        self.engine = CalibrationEngine(
            transport=self.transport,
            observation=self.observation,
            reporter=self.reporter
        )

    def test_engine_preflight_success(self):
        report = self.engine.run_preflight()
        self.assertTrue(report.all_passed)
        self.assertTrue(self.engine.is_motion_permitted())

    def test_engine_preflight_blocks_motion_on_mpu_failure(self):
        self.mock_serial.set_mpu_ok(False)
        report = self.engine.run_preflight()
        self.assertFalse(report.all_passed)
        self.assertFalse(self.engine.is_motion_permitted())
        with self.assertRaises(SafetyGateError):
            self.engine.calibrate(stage="U1")

    def test_engine_preflight_polarity_inversion(self):
        self.observation.set_motor_directions(left_forward=False, right_forward=True)
        report = self.engine.run_preflight()
        self.assertTrue(report.all_passed)
        self.assertTrue(report.polarity_inversion_applied)
        prof = self.engine.get_active_profile()
        self.assertEqual(prof.invert_motor_kiri, -1)

    def test_engine_u1_convergence_multi_trial(self):
        self.engine.run_preflight()
        # Trial 1: ground truth 105.0 cm (drift error triggers scaling)
        # Trial 2: ground truth 100.5 cm (converged)
        self.observation.set_u1_ground_truth([105.0, 100.5])
        res = self.engine.run_u1(max_retries=3)
        self.assertTrue(res.passed)
        self.assertEqual(res.iteration, 2)
        prof = self.engine.get_active_profile()
        self.assertNotEqual(prof.diameter_roda_mm, 65.0)

    def test_engine_u2_in_place_rotation(self):
        self.engine.run_preflight()
        self.observation.set_u2_actual_angle(91.5)
        res = self.engine.run_u2(target_angle=90.0)
        self.assertTrue(res.passed)
        self.assertAlmostEqual(res.angular_error_deg, 1.5, places=2)

    def test_engine_u3_straight_line(self):
        self.engine.run_preflight()
        self.observation.set_u3_lateral_dev(1.8)
        res = self.engine.run_u3()
        self.assertTrue(res.passed)
        self.assertAlmostEqual(res.lateral_error_cm, 1.8, places=2)

    def test_engine_u4_ranging_accuracy(self):
        self.engine.run_preflight()
        res = self.engine.run_u4()
        self.assertTrue(res.passed)
        self.assertEqual(len(res.sub_results), 3)

    def test_engine_calibrate_all_success(self):
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


if __name__ == "__main__":
    unittest.main()
