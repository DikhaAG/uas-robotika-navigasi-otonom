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
    main,
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
        exit_code = main(["--mock", "--headless"])
        self.assertEqual(exit_code, 0)

    def test_cli_mock_fail_mpu(self):
        exit_code = main(["--mock", "--headless", "--fail-mpu"])
        self.assertNotEqual(exit_code, 0)


if __name__ == "__main__":
    unittest.main()
