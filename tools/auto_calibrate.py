#!/usr/bin/env python3
"""
Automated Calibration & Pre-Flight Diagnostic Wizard (U0)
Praktikum Robotika - Polsri D3 Teknik Komputer

Provides interactive hardware diagnostics, hard-gated safety, and
serial ASCII protocol communication with the robot navigation firmware.
"""

import sys
import os
import time
import argparse
from typing import Optional, Tuple, List, Dict, Any

# ANSI Colors & Formatting
class Colors:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    GREEN = "\033[32m"
    RED = "\033[31m"
    YELLOW = "\033[33m"
    CYAN = "\033[36m"
    MAGENTA = "\033[35m"
    BLUE = "\033[34m"
    DIM = "\033[2m"

    @classmethod
    def disable(cls):
        cls.RESET = ""
        cls.BOLD = ""
        cls.GREEN = ""
        cls.RED = ""
        cls.YELLOW = ""
        cls.CYAN = ""
        cls.MAGENTA = ""
        cls.BLUE = ""
        cls.DIM = ""


if not sys.stdout.isatty():
    Colors.disable()


class SafetyGateError(RuntimeError):
    """Raised when motion calibration is attempted while hardware safety gate is locked."""
    pass


class DiagnosticResult:
    def __init__(self, mpu_ok: bool, sonar_dist_cm: float, ir_left: bool, ir_right: bool):
        self.mpu_ok = mpu_ok
        self.sonar_dist_cm = sonar_dist_cm
        self.ir_left = ir_left
        self.ir_right = ir_right

    def __repr__(self):
        return (f"DiagnosticResult(mpu_ok={self.mpu_ok}, sonar_dist_cm={self.sonar_dist_cm:.2f}, "
                f"ir_left={self.ir_left}, ir_right={self.ir_right})")


class CalibrationProfileData:
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

    def __repr__(self):
        return (f"CalibrationProfileData(diameter={self.diameter_roda_mm:.2f}mm, "
                f"trimL={self.trim_motor_kiri:.3f}, trimR={self.trim_motor_kanan:.3f}, "
                f"biasZ={self.gyro_bias_z:.3f}, invL={self.invert_motor_kiri}, invR={self.invert_motor_kanan})")


class MockRobotSerial:
    """Mock serial device simulating firmware response for headless testing and CI."""
    def __init__(self):
        self.is_open = True
        self.mpu_ok = True
        self.sonar_dist = 25.0
        self.ir_left = False
        self.ir_right = False
        self.pulses_left = 52
        self.pulses_right = 51
        self.profile = CalibrationProfileData()
        self.last_command = ""
        self.mock_u1_data = (98, 99, 100.12)
        self.mock_u2_theta = 90.0
        self.mock_u3_data = (147, 148, 1.85)
        self.mock_u4_distances: List[float] = [10.2, 20.1, 39.8]

    def set_mpu_ok(self, ok: bool):
        self.mpu_ok = ok

    def set_sonar_dist(self, dist: float):
        self.sonar_dist = dist

    def set_ir_states(self, left: bool, right: bool):
        self.ir_left = left
        self.ir_right = right

    def set_motor_pulses(self, left: int, right: int):
        self.pulses_left = left
        self.pulses_right = right

    def set_mock_u1_data(self, left: int, right: int, dist: float):
        self.mock_u1_data = (left, right, dist)

    def set_mock_u2_theta(self, theta: float):
        self.mock_u2_theta = theta

    def set_mock_u3_data(self, left: int, right: int, heading_err: float):
        self.mock_u3_data = (left, right, heading_err)

    def set_mock_u4_distances(self, dists: List[float]):
        self.mock_u4_distances = list(dists)

    def write(self, data: bytes):
        cmd = data.decode("ascii", errors="ignore").strip()
        self.last_command = cmd

    def readline(self) -> bytes:
        cmd = self.last_command
        self.last_command = ""

        if cmd == "!PING":
            return b"!PONG\n"
        elif cmd == "!DIAG:ALL":
            mpu = 1 if self.mpu_ok else 0
            irl = 1 if self.ir_left else 0
            irr = 1 if self.ir_right else 0
            return f"!DIAG:RES:{mpu}:{self.sonar_dist:.2f}:{irl}:{irr}\n".encode("ascii")
        elif cmd.startswith("!DIAG:MOTOR:"):
            return f"!PULSES:{self.pulses_left}:{self.pulses_right}\n".encode("ascii")
        elif cmd.startswith("!DIAG:BEEP"):
            return b"!OK\n"
        elif cmd == "!CALIB:READ":
            return f"!CALIB:DATA:{self.profile.to_command_args()}\n".encode("ascii")
        elif cmd.startswith("!CALIB:WRITE:"):
            parts = cmd[13:].split(":")
            if len(parts) == 6:
                try:
                    self.profile.diameter_roda_mm = float(parts[0])
                    self.profile.trim_motor_kiri = float(parts[1])
                    self.profile.trim_motor_kanan = float(parts[2])
                    self.profile.gyro_bias_z = float(parts[3])
                    self.profile.invert_motor_kiri = int(parts[4])
                    self.profile.invert_motor_kanan = int(parts[5])
                    return b"!OK\n"
                except ValueError:
                    return b"!ERR:PARAM\n"
            return b"!ERR:PARAM\n"
        elif cmd == "!CALIB:RESET":
            self.profile = CalibrationProfileData()
            return b"!RESET:OK\n"
        elif cmd == "!STEP:U1":
            pulses_l, pulses_r, dist = self.mock_u1_data
            return f"!U1:DONE:{pulses_l}:{pulses_r}:{dist:.2f}\n".encode("ascii")
        elif cmd.startswith("!STEP:U2:"):
            return f"!U2:DONE:{self.mock_u2_theta:.2f}\n".encode("ascii")
        elif cmd == "!STEP:U3":
            pulses_l, pulses_r, err = self.mock_u3_data
            return f"!U3:DONE:{pulses_l}:{pulses_r}:{err:.2f}\n".encode("ascii")
        elif cmd == "!STEP:U4":
            if self.mock_u4_distances:
                d = self.mock_u4_distances.pop(0)
            else:
                d = self.sonar_dist
            return f"!U4:DONE:{d:.2f}\n".encode("ascii")
        return b"!ERR:UNKNOWN\n"

    def close(self):
        self.is_open = False


class RobotSerialClient:
    """Client handling ASCII protocol frames over physical or mock serial link."""
    def __init__(self, port: Optional[str] = None, baud: int = 115200, timeout: float = 1.5, serial_backend: Any = None):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = serial_backend

        if self.ser is None and self.port:
            try:
                import serial
                self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
                # Wait for Arduino bootloader reset
                time.sleep(1.8)
                self.ser.reset_input_buffer()
            except ImportError:
                raise RuntimeError("Modul 'pyserial' belum terinstall. Install dengan: pip install pyserial")
            except Exception as e:
                raise RuntimeError(f"Gagal membuka port serial '{self.port}': {e}")

    def send_command(self, cmd: str) -> str:
        if not self.ser:
            raise RuntimeError("Koneksi serial tidak aktif.")

        frame = (cmd.strip() + "\n").encode("ascii")
        self.ser.write(frame)
        line = self.ser.readline().decode("ascii", errors="replace").strip()
        return line

    def ping(self) -> bool:
        resp = self.send_command("!PING")
        return resp == "!PONG"

    def probe_diagnostics(self) -> DiagnosticResult:
        resp = self.send_command("!DIAG:ALL")
        if not resp.startswith("!DIAG:RES:"):
            raise ValueError(f"Respon diagnostik tidak valid: {resp}")

        parts = resp[10:].split(":")
        if len(parts) < 4:
            raise ValueError(f"Format respon diagnostik kurang lengkap: {resp}")

        mpu_ok = (parts[0] == "1")
        sonar_dist = float(parts[1])
        ir_left = (parts[2] == "1")
        ir_right = (parts[3] == "1")
        return DiagnosticResult(mpu_ok, sonar_dist, ir_left, ir_right)

    def spin_motors(self, pwmL: int, pwmR: int, duration_ms: int) -> Tuple[int, int]:
        resp = self.send_command(f"!DIAG:MOTOR:{pwmL}:{pwmR}:{duration_ms}")
        if not resp.startswith("!PULSES:"):
            raise ValueError(f"Respon motor spin tidak valid: {resp}")

        parts = resp[8:].split(":")
        if len(parts) < 2:
            raise ValueError(f"Format pulsa encoder tidak valid: {resp}")

        return int(parts[0]), int(parts[1])

    def test_buzzer(self, duration_ms: int = 200) -> bool:
        resp = self.send_command(f"!DIAG:BEEP:{duration_ms}")
        return resp == "!OK"

    def read_profile(self) -> CalibrationProfileData:
        resp = self.send_command("!CALIB:READ")
        if not resp.startswith("!CALIB:DATA:"):
            raise ValueError(f"Respon baca profile tidak valid: {resp}")

        parts = resp[12:].split(":")
        if len(parts) < 6:
            raise ValueError(f"Format profile tidak lengkap: {resp}")

        return CalibrationProfileData(
            diameter_roda_mm=float(parts[0]),
            trim_motor_kiri=float(parts[1]),
            trim_motor_kanan=float(parts[2]),
            gyro_bias_z=float(parts[3]),
            invert_motor_kiri=int(parts[4]),
            invert_motor_kanan=int(parts[5])
        )

    def write_profile(self, profile: CalibrationProfileData) -> bool:
        resp = self.send_command(f"!CALIB:WRITE:{profile.to_command_args()}")
        return resp == "!OK"

    def reset_profile(self) -> bool:
        resp = self.send_command("!CALIB:RESET")
        return resp == "!RESET:OK"

    @staticmethod
    def _parse_triple_telemetry(resp: str, expected_prefix: str) -> Tuple[int, int, float]:
        if not resp.startswith(expected_prefix):
            raise ValueError(f"Respon tidak valid: {resp}")
        parts = resp[len(expected_prefix):].split(":")
        if len(parts) < 3:
            raise ValueError(f"Format respon telemetry tidak lengkap: {resp}")
        return int(parts[0]), int(parts[1]), float(parts[2])

    def step_u1(self) -> Tuple[int, int, float]:
        resp = self.send_command("!STEP:U1")
        return self._parse_triple_telemetry(resp, "!U1:DONE:")

    def step_u2(self, angle: float = 90.0) -> float:
        resp = self.send_command(f"!STEP:U2:{angle:.2f}")
        if not resp.startswith("!U2:DONE:"):
            raise ValueError(f"Respon STEP:U2 tidak valid: {resp}")
        return float(resp[9:])

    def step_u3(self) -> Tuple[int, int, float]:
        resp = self.send_command("!STEP:U3")
        return self._parse_triple_telemetry(resp, "!U3:DONE:")

    def step_u4(self) -> float:
        resp = self.send_command("!STEP:U4")
        if not resp.startswith("!U4:DONE:"):
            raise ValueError(f"Respon STEP:U4 tidak valid: {resp}")
        return float(resp[9:])

    def close(self):
        if self.ser and hasattr(self.ser, "close"):
            self.ser.close()


class U0SubtestReport:
    def __init__(self, name: str, passed: bool, details: str, pin_hints: Optional[List[str]] = None):
        self.name = name
        self.passed = passed
        self.details = details
        self.pin_hints = pin_hints or []


class U0Report:
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


class PreflightDiagnosticsRunner:
    """Orchestrates Stage U0 pre-flight hardware diagnostics."""
    def __init__(self, client: RobotSerialClient, interactive: bool = True, verbose: bool = True):
        self.client = client
        self.interactive = interactive
        self.verbose = verbose
        self._mock_left_forward = True
        self._mock_right_forward = True

    def log(self, *args, **kwargs):
        if self.verbose:
            print(*args, **kwargs)

    def mock_motor_directions(self, left_forward: bool, right_forward: bool):
        self._mock_left_forward = left_forward
        self._mock_right_forward = right_forward

    def run_u0(self) -> U0Report:
        report = U0Report()

        self.log(f"\n{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}  STAGE U0: PRE-FLIGHT HARDWARE & WIRING DIAGNOSTICS{Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}\n")

        # Step 1: Probe sensors via !DIAG:ALL
        try:
            diag = self.client.probe_diagnostics()
        except Exception as e:
            st = U0SubtestReport("Koneksi & Sensor Bus", False, f"Gagal membaca diagnostik sensor: {e}",
                                 ["Cek kabel USB, baudrate 115200, dan daya baterai robot."])
            report.add_subtest(st)
            return report

        # U0.1: I2C & MPU6050 WHO_AM_I
        self.log(f"{Colors.BOLD}[U0.1] Pengujian I2C Bus & MPU6050 Gyro/Accelerometer...{Colors.RESET}")
        if diag.mpu_ok:
            self.log(f"  {Colors.GREEN}✓ MPU6050 responsif pada I2C address 0x68 (WHO_AM_I valid).{Colors.RESET}")
            report.u01_mpu_passed = True
            report.add_subtest(U0SubtestReport("U0.1 MPU6050", True, "I2C 0x68 responsif"))
        else:
            self.log(f"  {Colors.RED}✗ MPU6050 TIDAK terdeteksi pada I2C address 0x68!{Colors.RESET}")
            report.u01_mpu_passed = False
            report.add_subtest(U0SubtestReport("U0.1 MPU6050", False, "WHO_AM_I gagal / I2C bus error", [
                "Periksa kabel jumper MPU6050: VCC->5V, GND->GND.",
                "Periksa pin I2C Arduino Uno: SDA terhubung ke Pin A4, SCL terhubung ke Pin A5."
            ]))

        # U0.2: Ultrasonic Sonar HC-SR04
        self.log(f"{Colors.BOLD}[U0.2] Pengujian Sonar Ultrasonik HC-SR04...{Colors.RESET}")
        if 2.0 <= diag.sonar_dist_cm <= 400.0:
            self.log(f"  {Colors.GREEN}✓ HC-SR04 terbaca valid: {diag.sonar_dist_cm:.1f} cm (bebas timeout).{Colors.RESET}")
            report.u02_sonar_passed = True
            report.add_subtest(U0SubtestReport("U0.2 Sonar", True, f"Jarak: {diag.sonar_dist_cm:.1f} cm"))
        else:
            self.log(f"  {Colors.RED}✗ HC-SR04 timeout atau pembacaan tidak valid ({diag.sonar_dist_cm:.1f} cm)!{Colors.RESET}")
            report.u02_sonar_passed = False
            report.add_subtest(U0SubtestReport("U0.2 Sonar", False, f"Timeout sonar ({diag.sonar_dist_cm:.1f} cm)", [
                "Periksa pin Sonar: Trig terhubung ke Pin A0, Echo terhubung ke Pin A1.",
                "Pastikan sensor mendapat suplai 5V bersih dan tidak terhalang kabel."
            ]))

        # U0.3: FC-51 IR Lateral Proximity Sensors
        self.log(f"{Colors.BOLD}[U0.3] Pengujian Sensor Jarak Dinding IR Lateral (FC-51)...{Colors.RESET}")
        # In non-interactive mode, as long as read returns a valid boolean state it passes
        ir_passed = True
        ir_details = f"IR Kiri={diag.ir_left}, IR Kanan={diag.ir_right}"
        if self.interactive:
            self.log(f"  Status awal: IR Kiri={'TERHALANG' if diag.ir_left else 'BEBAS'}, "
                     f"IR Kanan={'TERHALANG' if diag.ir_right else 'BEBAS'}")
            self.log(f"  {Colors.YELLOW}Panduan: Letakkan tangan di depan sensor IR Kiri (A3), lalu IR Kanan (D11).{Colors.RESET}")
            try:
                # Prompt user for manual verification
                ans = input("  Apakah LED indikator modul FC-51 aktif saat dihalangi? (y/n) [default y]: ").strip().lower()
                if ans == "n":
                    ir_passed = False
            except EOFError:
                pass

        if ir_passed:
            self.log(f"  {Colors.GREEN}✓ Sensor IR Lateral FC-51 terverifikasi ({ir_details}).{Colors.RESET}")
            report.u03_ir_passed = True
            report.add_subtest(U0SubtestReport("U0.3 IR Proximity", True, ir_details))
        else:
            self.log(f"  {Colors.RED}✗ Verifikasi IR Lateral gagal.{Colors.RESET}")
            report.u03_ir_passed = False
            report.add_subtest(U0SubtestReport("U0.3 IR Proximity", False, "Respon logika LOW tidak terdeteksi", [
                "Periksa pin FC-51 IR Kiri: OUT terhubung ke Pin A3.",
                "Periksa pin FC-51 IR Kanan: OUT terhubung ke Pin D11.",
                "Putar potensiometer sensitivitas pada modul FC-51 hingga LED deteksi responsif."
            ]))

        # U0.4: Buzzer & LED Indicators
        self.log(f"{Colors.BOLD}[U0.4] Pengujian Indikator Suara & Visual (Buzzer & LED)...{Colors.RESET}")
        beep_ok = self.client.test_buzzer(200)
        indicator_passed = beep_ok
        if self.interactive:
            ans = input("  Apakah Buzzer berbunyi dan LED berkedip singkat? (y/n) [default y]: ").strip().lower()
            if ans == "n":
                indicator_passed = False

        if indicator_passed:
            self.log(f"  {Colors.GREEN}✓ Indikator Buzzer & LED terverifikasi.{Colors.RESET}")
            report.u04_indicators_passed = True
            report.add_subtest(U0SubtestReport("U0.4 Buzzer & LED", True, "Indikator aktif normal"))
        else:
            self.log(f"  {Colors.RED}✗ Indikator Buzzer atau LED tidak berfungsi.{Colors.RESET}")
            report.u04_indicators_passed = False
            report.add_subtest(U0SubtestReport("U0.4 Buzzer & LED", False, "Buzzer/LED tidak merespons", [
                "Periksa Buzzer: Positif terhubung ke Pin A2, negatif ke GND.",
                "Periksa LED status: Anoda ke Pin D13 (dengan resistor 220Ω), katoda ke GND."
            ]))

        # U0.5: Motor Driver Channels & Optical Encoders
        self.log(f"{Colors.BOLD}[U0.5] Pengujian Motor Driver L298N & Dual Optical Encoder...{Colors.RESET}")
        self.log("  Memutar roda kiri secara independen (PWM 140, 400ms)...")
        pulses_l1, _ = self.client.spin_motors(140, 0, 400)
        self.log(f"    Pulsa terdeteksi roda kiri: {pulses_l1}")

        self.log("  Memutar roda kanan secara independen (PWM 140, 400ms)...")
        _, pulses_r2 = self.client.spin_motors(0, 140, 400)
        self.log(f"    Pulsa terdeteksi roda kanan: {pulses_r2}")

        motors_passed = True
        motor_hints = []

        if pulses_l1 <= 0:
            motors_passed = False
            motor_hints.append("Encoder Kiri tidak mendeteksi pulsa! Cek disk optical encoder & Pin D2 (INT0).")
        if pulses_r2 <= 0:
            motors_passed = False
            motor_hints.append("Encoder Kanan tidak mendeteksi pulsa! Cek disk optical encoder & Pin D3 (INT1).")

        # Verify rotation direction & software polarity inversion
        left_forward = self._mock_left_forward
        right_forward = self._mock_right_forward

        if self.interactive:
            ans_l = input("  Apakah RODA KIRI berputar MAJU? (y/n) [default y]: ").strip().lower()
            if ans_l == "n":
                left_forward = False
            ans_r = input("  Apakah RODA KANAN berputar MAJU? (y/n) [default y]: ").strip().lower()
            if ans_r == "n":
                right_forward = False

        if not left_forward or not right_forward:
            self.log(f"  {Colors.YELLOW}⚠ Deteksi motor terbalik: Kiri={'MAJU' if left_forward else 'MUNDUR'}, "
                     f"Kanan={'MAJU' if right_forward else 'MUNDUR'}.{Colors.RESET}")
            self.log(f"  {Colors.CYAN}Menerapkan Software Polarity Inversion otomatis ke EEPROM...{Colors.RESET}")
            try:
                profile = self.client.read_profile()
                if not left_forward:
                    profile.invert_motor_kiri = -profile.invert_motor_kiri
                if not right_forward:
                    profile.invert_motor_kanan = -profile.invert_motor_kanan

                if self.client.write_profile(profile):
                    self.log(f"  {Colors.GREEN}✓ Polaritas motor berhasil dikoreksi di EEPROM: "
                             f"invertL={profile.invert_motor_kiri}, invertR={profile.invert_motor_kanan}.{Colors.RESET}")
                    report.polarity_inversion_applied = True
                else:
                    motors_passed = False
                    motor_hints.append("Gagal menyimpan koreksi polaritas ke EEPROM.")
            except Exception as e:
                motors_passed = False
                motor_hints.append(f"Gagal memperbarui profil EEPROM: {e}")

        if motors_passed:
            self.log(f"  {Colors.GREEN}✓ Motor & Optical Encoder lulus uji (L={pulses_l1}, R={pulses_r2} pulsa).{Colors.RESET}")
            report.u05_motors_passed = True
            report.add_subtest(U0SubtestReport("U0.5 Motor & Encoder", True, f"Pulsa: L={pulses_l1}, R={pulses_r2}"))
        else:
            self.log(f"  {Colors.RED}✗ Pengujian Motor / Encoder gagal.{Colors.RESET}")
            report.u05_motors_passed = False
            report.add_subtest(U0SubtestReport("U0.5 Motor & Encoder", False, "Pulsa encoder 0 atau error arah", motor_hints))

        # Overall Status
        report.all_passed = (report.u01_mpu_passed and
                             report.u02_sonar_passed and
                             report.u03_ir_passed and
                             report.u04_indicators_passed and
                             report.u05_motors_passed)
        return report


class SafetyGate:
    """Enforces hard-gated safety: kinematic motion is locked until 100% of U0 passes."""
    def __init__(self, u0_report: U0Report):
        self.report = u0_report

    def is_motion_permitted(self) -> bool:
        return self.report.all_passed

    def assert_motion_permitted(self):
        if not self.is_motion_permitted():
            failed_tests = [st.name for st in self.report.subtests if not st.passed]
            raise SafetyGateError(
                f"HARD-GATE LOCKED: Motion calibration (U1-U4) DILARANG sebelum pengujian hardware U0 100% lulus! "
                f"Sub-test gagal: {', '.join(failed_tests)}"
            )

    def print_status(self):
        if self.is_motion_permitted():
            print(f"\n{Colors.BOLD}{Colors.GREEN}===================================================={Colors.RESET}")
            print(f"{Colors.BOLD}{Colors.GREEN}  HARD-GATE STATUS: UNLOCKED (MOTION PERMITTED)      {Colors.RESET}")
            print(f"{Colors.BOLD}{Colors.GREEN}===================================================={Colors.RESET}")
            print("  Seluruh perangkat keras terverifikasi normal. Tahap kalibrasi kinematika (U1-U4) siap dijalankan.\n")
        else:
            print(f"\n{Colors.BOLD}{Colors.RED}===================================================={Colors.RESET}")
            print(f"{Colors.BOLD}{Colors.RED}  HARD-GATE STATUS: LOCKED (MOTION BLOCKED!)        {Colors.RESET}")
            print(f"{Colors.BOLD}{Colors.RED}===================================================={Colors.RESET}")
            print("  DILARANG melakukan kalibrasi gerak sebelum kerusakan hardware diperbaiki!\n")
            print(f"{Colors.BOLD}Petunjuk Perbaikan Perkabelan (Wiring Repair):{Colors.RESET}")
            hints = self.report.get_wiring_repair_instructions()
            if hints:
                for idx, hint in enumerate(hints, 1):
                    print(f"  {idx}. {Colors.YELLOW}{hint}{Colors.RESET}")
            else:
                print(f"  {Colors.YELLOW}Periksa koneksi fisik dan ulangi pengujian U0.{Colors.RESET}")
            print()


def calculate_wheel_diameter_correction(current_diam: float, ground_truth_dist: float, firmware_dist: float) -> float:
    """
    Computes updated wheel diameter using the closed-form scaling formula:
    D_new = D_current * (D_ground_truth / D_firmware)
    """
    if firmware_dist <= 0:
        firmware_dist = 100.0
    return float(current_diam * (ground_truth_dist / firmware_dist))


def calculate_motor_trim_correction(trim_l: float, trim_r: float, pulses_l: int, pulses_r: int) -> Tuple[float, float]:
    """
    Computes motor trim factors to balance wheel rotation speeds:
    - If left wheel produced more pulses (faster), scale down trim_l by (pulses_r / pulses_l).
    - If right wheel produced more pulses (faster), scale down trim_r by (pulses_l / pulses_r).
    Clamped to safe bounds [0.75, 1.25].
    """
    if pulses_l <= 0 or pulses_r <= 0:
        return trim_l, trim_r

    if pulses_l > pulses_r:
        ratio = float(pulses_r) / float(pulses_l)
        trim_l = trim_l * ratio
    elif pulses_r > pulses_l:
        ratio = float(pulses_l) / float(pulses_r)
        trim_r = trim_r * ratio

    # Clamp to safe bounds [0.75, 1.25]
    trim_l = max(0.75, min(1.25, trim_l))
    trim_r = max(0.75, min(1.25, trim_r))
    return trim_l, trim_r


def check_u1_tolerances(ground_truth_dist: float, pulses_l: int, pulses_r: int) -> Tuple[bool, float, float]:
    """
    Checks U1 tolerances:
    1. Distance translation error <= 3.0 cm relative to 100.0 cm nominal.
    2. Wheel pulse drift disparity <= 3.0%.
    """
    dist_error = abs(ground_truth_dist - 100.0)
    max_pulses = max(pulses_l, pulses_r)
    drift_pct = (abs(pulses_l - pulses_r) / float(max_pulses) * 100.0) if max_pulses > 0 else 0.0
    passed = (dist_error <= 3.0) and (drift_pct <= 3.0)
    return passed, dist_error, drift_pct


def check_u2_tolerances(actual_angle: float, target_angle: float = 90.0) -> Tuple[bool, float]:
    """
    Checks U2 tolerance: in-place rotation angle error <= 5.0 deg.
    Preserves sign to prevent reversed rotation (-90 deg) from passing when +90 deg is target.
    """
    angular_error = abs(actual_angle - target_angle)
    passed = (angular_error <= 5.0)
    return passed, angular_error


def check_u3_tolerances(lateral_deviation_cm: float) -> Tuple[bool, float]:
    """
    Checks U3 tolerance: lateral corridor deviation <= 5.0 cm.
    """
    lat_err = abs(lateral_deviation_cm)
    passed = (lat_err <= 5.0)
    return passed, lat_err


def check_u4_tolerances(measured_cm: float, target_cm: float) -> Tuple[bool, float]:
    """
    Checks U4 tolerance: ultrasonic ranging error <= 1.0 cm.
    """
    err = abs(measured_cm - target_cm)
    passed = (err <= 1.0)
    return passed, err


class CalibrationLogger:
    """Exports structured calibration telemetry, trials, and tuned profiles to CSV."""
    def __init__(self, filepath: str = "tools/calibration_log.csv"):
        self.filepath = filepath
        self._ensure_header()

    def _ensure_header(self):
        dir_name = os.path.dirname(os.path.abspath(self.filepath))
        if dir_name and not os.path.exists(dir_name):
            os.makedirs(dir_name, exist_ok=True)
        if not os.path.exists(self.filepath) or os.path.getsize(self.filepath) == 0:
            with open(self.filepath, "w", encoding="utf-8") as f:
                f.write("timestamp,test_id,iteration,status,target,measured,ground_truth,error,diameter_mm,trim_l,trim_r,bias_z,notes\n")

    def log_result(self,
                   test_id: str,
                   iteration: int,
                   status: str,
                   target: float,
                   measured: float,
                   ground_truth: float,
                   error: float,
                   profile: Optional[CalibrationProfileData] = None,
                   notes: str = ""):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        diam = f"{profile.diameter_roda_mm:.2f}" if profile else ""
        trim_l = f"{profile.trim_motor_kiri:.3f}" if profile else ""
        trim_r = f"{profile.trim_motor_kanan:.3f}" if profile else ""
        bias_z = f"{profile.gyro_bias_z:.3f}" if profile else ""
        clean_notes = notes.replace(",", ";").replace("\n", " ").strip()
        line = f"{timestamp},{test_id},{iteration},{status},{target:.2f},{measured:.2f},{ground_truth:.2f},{error:.2f},{diam},{trim_l},{trim_r},{bias_z},{clean_notes}\n"
        with open(self.filepath, "a", encoding="utf-8") as f:
            f.write(line)

    def log_u0(self, report: U0Report, profile: Optional[CalibrationProfileData] = None):
        """Logs all pre-flight diagnostic subtests (U0.1 - U0.5) to the CSV log."""
        for st in report.subtests:
            status = "PASS" if st.passed else "FAIL"
            self.log_result(
                test_id=st.name,
                iteration=1,
                status=status,
                target=0.0,
                measured=0.0,
                ground_truth=0.0,
                error=0.0,
                profile=profile,
                notes=st.details
            )


class U1Result:
    def __init__(self, passed: bool, iteration: int, ground_truth_cm: float,
                 firmware_dist_cm: float, pulses_l: int, pulses_r: int,
                 dist_error_cm: float, pulse_drift_pct: float,
                 new_diameter_mm: float, new_trim_l: float, new_trim_r: float):
        self.passed = passed
        self.iteration = iteration
        self.ground_truth_cm = ground_truth_cm
        self.firmware_dist_cm = firmware_dist_cm
        self.pulses_l = pulses_l
        self.pulses_r = pulses_r
        self.dist_error_cm = dist_error_cm
        self.pulse_drift_pct = pulse_drift_pct
        self.new_diameter_mm = new_diameter_mm
        self.new_trim_l = new_trim_l
        self.new_trim_r = new_trim_r

    def __repr__(self):
        return (f"U1Result(passed={self.passed}, iter={self.iteration}, dist_err={self.dist_error_cm:.2f}cm, "
                f"drift={self.pulse_drift_pct:.2f}%, diam={self.new_diameter_mm:.2f}mm, "
                f"trimL={self.new_trim_l:.3f}, trimR={self.new_trim_r:.3f})")


class U2Result:
    def __init__(self, passed: bool, target_angle_deg: float,
                 actual_angle_deg: float, angular_error_deg: float,
                 polarity_verified: bool = True):
        self.passed = passed
        self.target_angle_deg = target_angle_deg
        self.actual_angle_deg = actual_angle_deg
        self.angular_error_deg = angular_error_deg
        self.polarity_verified = polarity_verified

    def __repr__(self):
        return (f"U2Result(passed={self.passed}, target={self.target_angle_deg:.1f}°, "
                f"actual={self.actual_angle_deg:.1f}°, err={self.angular_error_deg:.2f}°)")


class U3Result:
    def __init__(self, passed: bool, pulses_l: int, pulses_r: int,
                 max_heading_error_deg: float, lateral_error_cm: float):
        self.passed = passed
        self.pulses_l = pulses_l
        self.pulses_r = pulses_r
        self.max_heading_error_deg = max_heading_error_deg
        self.lateral_error_cm = lateral_error_cm

    def __repr__(self):
        return (f"U3Result(passed={self.passed}, pulses=({self.pulses_l}, {self.pulses_r}), "
                f"max_heading_err={self.max_heading_error_deg:.2f}°, lat_err={self.lateral_error_cm:.2f}cm)")


class U4SubResult:
    def __init__(self, target_cm: float, measured_cm: float,
                 error_cm: float, passed: bool):
        self.target_cm = target_cm
        self.measured_cm = measured_cm
        self.error_cm = error_cm
        self.passed = passed

    def __repr__(self):
        return f"U4SubResult(target={self.target_cm:.1f}cm, measured={self.measured_cm:.2f}cm, err={self.error_cm:.2f}cm, passed={self.passed})"


class U4Result:
    def __init__(self, passed: bool, sub_results: List[U4SubResult]):
        self.passed = passed
        self.sub_results = sub_results

    def __repr__(self):
        return f"U4Result(passed={self.passed}, items={len(self.sub_results)})"


class KinematicCalibrationRunner:
    """Orchestrates kinematic calibration tests (U1 to U4), auto-retries, and parameter tuning."""
    def __init__(self, client: RobotSerialClient, logger: Optional[CalibrationLogger] = None,
                 interactive: bool = True, verbose: bool = True):
        self.client = client
        self.logger = logger
        self.interactive = interactive
        self.verbose = verbose

        self._mock_u1_gt: Optional[float] = None
        self._mock_u1_gt_sequence: Optional[List[float]] = None
        self._mock_u2_angle: Optional[float] = None
        self._mock_u3_lat_dev: Optional[float] = None

    def log(self, *args, **kwargs):
        if self.verbose:
            print(*args, **kwargs)

    def set_mock_u1_ground_truth(self, val: float):
        self._mock_u1_gt = val

    def set_mock_u1_ground_truth_sequence(self, seq: List[float]):
        self._mock_u1_gt_sequence = list(seq)

    def set_mock_u2_actual_angle(self, val: float):
        self._mock_u2_angle = val

    def set_mock_u3_lateral_dev(self, val: float):
        self._mock_u3_lat_dev = val

    def run_u1(self, max_retries: int = 5) -> U1Result:
        self.log(f"\n{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}  STAGE U1: WHEEL DISTANCE & MOTOR BALANCE CALIBRATION{Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log("Target translasi: 100.0 cm | Toleransi galat jarak <= 3.0 cm | Disparitas pulsa <= 3.0%\n")

        last_result: Optional[U1Result] = None

        for iteration in range(1, max_retries + 1):
            self.log(f"{Colors.BOLD}[U1 - Iterasi {iteration}/{max_retries}] Menjalankan translasi maju 100 cm...{Colors.RESET}")
            pulses_l, pulses_r, fw_dist = self.client.step_u1()
            self.log(f"  Pulsa tercatat: L={pulses_l}, R={pulses_r} | Jarak kalkulasi firmware: {fw_dist:.2f} cm")

            # Ground-truth measurement
            if self.interactive:
                self.log(f"  {Colors.YELLOW}Panduan: Ukur jarak fisik aktual dari garis awal menggunakan meteran pita.{Colors.RESET}")
                prompt_str = f"  Masukkan jarak aktual meteran pita (cm) [default {fw_dist:.1f}]: "
                try:
                    user_input = input(prompt_str).strip()
                    gt_dist = float(user_input) if user_input else fw_dist
                except (ValueError, EOFError):
                    gt_dist = fw_dist
            else:
                if self._mock_u1_gt_sequence and len(self._mock_u1_gt_sequence) > 0:
                    gt_dist = self._mock_u1_gt_sequence.pop(0)
                elif self._mock_u1_gt is not None:
                    gt_dist = self._mock_u1_gt
                else:
                    gt_dist = fw_dist

            passed, dist_err, drift_pct = check_u1_tolerances(gt_dist, pulses_l, pulses_r)
            profile = self.client.read_profile()

            new_diam = calculate_wheel_diameter_correction(profile.diameter_roda_mm, gt_dist, fw_dist)
            new_trim_l, new_trim_r = calculate_motor_trim_correction(profile.trim_motor_kiri, profile.trim_motor_kanan, pulses_l, pulses_r)

            status_str = "PASS" if passed else ("RETRY" if iteration < max_retries else "FAIL")
            notes = f"Galat={dist_err:.2f}cm, Drift={drift_pct:.2f}%"

            if self.logger:
                self.logger.log_result(
                    test_id="U1",
                    iteration=iteration,
                    status=status_str,
                    target=100.0,
                    measured=fw_dist,
                    ground_truth=gt_dist,
                    error=dist_err,
                    profile=profile,
                    notes=notes
                )

            last_result = U1Result(
                passed=passed,
                iteration=iteration,
                ground_truth_cm=gt_dist,
                firmware_dist_cm=fw_dist,
                pulses_l=pulses_l,
                pulses_r=pulses_r,
                dist_error_cm=dist_err,
                pulse_drift_pct=drift_pct,
                new_diameter_mm=new_diam,
                new_trim_l=new_trim_l,
                new_trim_r=new_trim_r
            )

            if passed:
                self.log(f"  {Colors.GREEN}✓ U1 Toleransi Terpenuhi! Galat={dist_err:.2f} cm (<= 3.0 cm), "
                         f"Drift={drift_pct:.2f}% (<= 3.0%).{Colors.RESET}")
                # Save final tuned values
                profile.diameter_roda_mm = new_diam
                profile.trim_motor_kiri = new_trim_l
                profile.trim_motor_kanan = new_trim_r
                self.client.write_profile(profile)
                self.log(f"  {Colors.CYAN}Profil EEPROM diperbarui: Diameter={new_diam:.2f} mm, TrimL={new_trim_l:.3f}, TrimR={new_trim_r:.3f}.{Colors.RESET}\n")
                return last_result
            else:
                self.log(f"  {Colors.YELLOW}⚠ U1 Belum Konvergen: Galat={dist_err:.2f} cm, Drift={drift_pct:.2f}%.{Colors.RESET}")
                self.log(f"    Penyetelan Otomatis: Diameter {profile.diameter_roda_mm:.2f} -> {new_diam:.2f} mm, "
                         f"Trim L {profile.trim_motor_kiri:.3f} -> {new_trim_l:.3f}, Trim R {profile.trim_motor_kanan:.3f} -> {new_trim_r:.3f}")
                profile.diameter_roda_mm = new_diam
                profile.trim_motor_kiri = new_trim_l
                profile.trim_motor_kanan = new_trim_r
                self.client.write_profile(profile)

                if iteration < max_retries:
                    if self.interactive:
                        self.log(f"  {Colors.YELLOW}Panduan: Kembalikan posisi robot ke garis start (0 cm).{Colors.RESET}")
                        try:
                            input("  Tekan ENTER untuk memulai iterasi kalibrasi berikutnya...")
                        except (EOFError, KeyboardInterrupt):
                            pass
                    self.log(f"  Mengulang pengujian U1 dengan parameter baru...\n")

        self.log(f"  {Colors.RED}✗ U1 GAGAL setelah {max_retries} iterasi!{Colors.RESET}")
        self.log(f"  {Colors.YELLOW}Petunjuk Mekanik: Periksa kemungkinan roda selip, baut kopling motor longgar, "
                 f"atau piringan encoder D2/D3 tidak sejajar.{Colors.RESET}\n")
        return last_result

    def run_u2(self, target_angle: float = 90.0) -> U2Result:
        self.log(f"\n{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}  STAGE U2: GYROSCOPE BIAS & IN-PLACE ROTATION        {Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log(f"Target rotasi in-place: {target_angle:.1f}° | Toleransi sudut <= 5.0°\n")

        self.log(f"{Colors.BOLD}[U2.1] Pengambilan Sampel Bias Gyroscope Z-axis (2.0 detik)...{Colors.RESET}")
        self.log("  Pastikan robot berada dalam kondisi diam sempurna di lantai datar.")
        if self.interactive:
            input("  Tekan ENTER untuk memulai sampling bias zero-rate...")
        self.log("  Sampling 200 sampel gyro rate...")

        # In physical operation, sampling pause takes 2 seconds
        if not (hasattr(self.client, "ser") and isinstance(self.client.ser, MockRobotSerial)):
            time.sleep(2.0)

        self.log(f"  {Colors.GREEN}✓ Sampling bias selesai.{Colors.RESET}")

        self.log(f"\n{Colors.BOLD}[U2.2] Menjalankan Rotasi In-Place ({target_angle:.1f}°)...{Colors.RESET}")
        reported_theta = self.client.step_u2(target_angle)
        self.log(f"  Sudut orientasi akhir firmware: {reported_theta:.2f}°")

        polarity_ok = True
        if self.interactive:
            ans_dir = input("  Apakah robot berputar ke arah kiri (berlawanan jarum jam)? (y/n) [default y]: ").strip().lower()
            if ans_dir == "n":
                polarity_ok = False
                self.log(f"  {Colors.YELLOW}⚠ Arah rotasi terbalik! Periksa orientasi pin MPU6050 atau GYRO_SIGN.{Colors.RESET}")

            prompt_str = f"  Masukkan sudut fisik aktual busur derajat lantai (°) [default {reported_theta:.1f}]: "
            try:
                user_input = input(prompt_str).strip()
                actual_angle = float(user_input) if user_input else reported_theta
            except (ValueError, EOFError):
                actual_angle = reported_theta
        else:
            actual_angle = self._mock_u2_angle if self._mock_u2_angle is not None else target_angle

        passed, angular_error = check_u2_tolerances(actual_angle, target_angle)
        passed = passed and polarity_ok

        profile = self.client.read_profile()
        status_str = "PASS" if passed else "FAIL"
        notes = f"Target={target_angle:.1f}°, Aktual={actual_angle:.1f}°, Galat={angular_error:.2f}°"

        if self.logger:
            self.logger.log_result(
                test_id="U2",
                iteration=1,
                status=status_str,
                target=target_angle,
                measured=reported_theta,
                ground_truth=actual_angle,
                error=angular_error,
                profile=profile,
                notes=notes
            )

        if passed:
            self.log(f"  {Colors.GREEN}✓ U2 Toleransi Terpenuhi! Galat Sudut={angular_error:.2f}° (<= 5.0°).{Colors.RESET}\n")
        else:
            self.log(f"  {Colors.RED}✗ U2 Gagal Memenuhi Toleransi! Galat Sudut={angular_error:.2f}° (> 5.0°).{Colors.RESET}")
            self.log(f"  {Colors.YELLOW}Petunjuk: Redam getaran chassis MPU6050 atau pastikan roda tidak slip saat berputar.{Colors.RESET}\n")

        return U2Result(passed, target_angle, actual_angle, angular_error, polarity_ok)

    def run_u3(self) -> U3Result:
        self.log(f"\n{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}  STAGE U3: GYRO-GUIDED STRAIGHT LINE TRACKING        {Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log("Target translasi lurus: 150.0 cm | Toleransi deviasi lateral sumbu Y <= 5.0 cm\n")

        self.log(f"{Colors.BOLD}[U3] Menjalankan lintasan lurus 150 cm dengan kemudi heading gyro...{Colors.RESET}")
        pulses_l, pulses_r, max_heading_err = self.client.step_u3()
        self.log(f"  Pulsa tercatat: L={pulses_l}, R={pulses_r} | Maksimum galat heading: {max_heading_err:.2f}°")

        if self.interactive:
            self.log(f"  {Colors.YELLOW}Panduan: Ukur pergeseran lateral ujung robot dari garis tengah lintasan (sumbu Y).{Colors.RESET}")
            prompt_str = "  Masukkan deviasi lateral terukur (cm) [default 0.0]: "
            try:
                user_input = input(prompt_str).strip()
                lat_dev = float(user_input) if user_input else 0.0
            except (ValueError, EOFError):
                lat_dev = 0.0
        else:
            lat_dev = self._mock_u3_lat_dev if self._mock_u3_lat_dev is not None else 0.0

        passed, lat_error = check_u3_tolerances(lat_dev)
        profile = self.client.read_profile()
        status_str = "PASS" if passed else "FAIL"
        notes = f"MaxHeadingErr={max_heading_err:.2f}°, DeviasiLateral={lat_dev:.2f}cm"

        if self.logger:
            self.logger.log_result(
                test_id="U3",
                iteration=1,
                status=status_str,
                target=0.0,
                measured=max_heading_err,
                ground_truth=lat_dev,
                error=lat_error,
                profile=profile,
                notes=notes
            )

        if passed:
            self.log(f"  {Colors.GREEN}✓ U3 Toleransi Terpenuhi! Deviasi Lateral={lat_error:.2f} cm (<= 5.0 cm).{Colors.RESET}\n")
        else:
            self.log(f"  {Colors.RED}✗ U3 Gagal Memenuhi Toleransi! Deviasi Lateral={lat_error:.2f} cm (> 5.0 cm).{Colors.RESET}")
            self.log(f"  {Colors.YELLOW}Petunjuk: Ulangi kalibrasi U1 motor balance trim atau naikkan KP_HEADING di Config.h.{Colors.RESET}\n")

        return U3Result(passed, pulses_l, pulses_r, max_heading_err, lat_error)

    def run_u4(self) -> U4Result:
        self.log(f"\n{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}  STAGE U4: ULTRASONIC OBSTACLE RANGING ACCURACY      {Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log("Titik uji jarak: 10.0 cm, 20.0 cm, 40.0 cm | Toleransi galat jarak <= 1.0 cm\n")

        targets = [10.0, 20.0, 40.0]
        sub_results: List[U4SubResult] = []
        profile = self.client.read_profile()

        for idx, target in enumerate(targets, 1):
            self.log(f"{Colors.BOLD}[U4.{idx}] Pengujian Jarak Rintangan {target:.1f} cm...{Colors.RESET}")
            if self.interactive:
                self.log(f"  {Colors.YELLOW}Posisikan kotak rintangan tegak lurus sensor pada jarak {target:.1f} cm.{Colors.RESET}")
                input("  Tekan ENTER jika kotak sudah siap diukur...")

            measured = self.client.step_u4()
            passed, err = check_u4_tolerances(measured, target)
            sub_results.append(U4SubResult(target, measured, err, passed))

            status_str = "PASS" if passed else "FAIL"
            notes = f"Target={target:.1f}cm, Terbaca={measured:.2f}cm, Galat={err:.2f}cm"

            if self.logger:
                self.logger.log_result(
                    test_id=f"U4.{idx}",
                    iteration=1,
                    status=status_str,
                    target=target,
                    measured=measured,
                    ground_truth=target,
                    error=err,
                    profile=profile,
                    notes=notes
                )

            if passed:
                self.log(f"  {Colors.GREEN}✓ U4.{idx} Terbaca: {measured:.2f} cm | Galat={err:.2f} cm (<= 1.0 cm).{Colors.RESET}")
            else:
                self.log(f"  {Colors.RED}✗ U4.{idx} Terbaca: {measured:.2f} cm | Galat={err:.2f} cm (> 1.0 cm)!{Colors.RESET}")

        all_passed = all(sr.passed for sr in sub_results)
        if all_passed:
            self.log(f"\n{Colors.GREEN}✓ Seluruh titik uji ultrasonik U4 lulus kalibrasi akurasi.{Colors.RESET}\n")
        else:
            self.log(f"\n{Colors.RED}✗ U4 belum memenuhi toleransi akurasi pada sebagian titik.{Colors.RESET}")
            self.log(f"{Colors.YELLOW}Petunjuk: Pastikan sensor tegak lurus dan bebas pantulan dari permukaan lantai.{Colors.RESET}\n")

        return U4Result(all_passed, sub_results)

    def run_all(self, safety_gate: SafetyGate) -> bool:
        safety_gate.assert_motion_permitted()

        self.log(f"\n{Colors.BOLD}{Colors.MAGENTA}===================================================={Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.MAGENTA}   MEMULAI SELURUH SUITE KALIBRASI KINEMATIKA (U1-U4){Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.MAGENTA}===================================================={Colors.RESET}")

        u1_res = self.run_u1()
        u2_res = self.run_u2()
        u3_res = self.run_u3()
        u4_res = self.run_u4()

        all_ok = u1_res.passed and u2_res.passed and u3_res.passed and u4_res.passed

        self.log(f"\n{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}          RINGKASAN STATUS KALIBRASI KINEMATIKA     {Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log(f"  U1 Pulsa & Diameter Roda : {'LULUS' if u1_res.passed else 'GAGAL'} (Galat={u1_res.dist_error_cm:.2f} cm, Drift={u1_res.pulse_drift_pct:.2f}%)")
        self.log(f"  U2 Rotasi Gyro In-Place  : {'LULUS' if u2_res.passed else 'GAGAL'} (Galat={u2_res.angular_error_deg:.2f}°)")
        self.log(f"  U3 Lintasan Lurus 150 cm : {'LULUS' if u3_res.passed else 'GAGAL'} (Deviasi={u3_res.lateral_error_cm:.2f} cm)")
        self.log(f"  U4 Akurasi Sonar HC-SR04 : {'LULUS' if u4_res.passed else 'GAGAL'}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}----------------------------------------------------{Colors.RESET}")
        if all_ok:
            self.log(f"  {Colors.BOLD}{Colors.GREEN}STATUS AKHIR: SELURUH PROTOKOL KALIBRASI LULUS!{Colors.RESET}\n")
        else:
            self.log(f"  {Colors.BOLD}{Colors.RED}STATUS AKHIR: ADA PROTOKOL YANG BELUM LULUS TOLERANSI.{Colors.RESET}\n")

        return all_ok


def autodetect_serial_port() -> Optional[str]:
    """Auto-detects available Arduino serial ports."""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            desc = p.description.lower()
            if any(key in desc for key in ["arduino", "ch340", "cp210", "ftdi", "usb serial"]):
                return p.device
        if ports:
            return ports[0].device
    except Exception:
        pass

    candidates = [
        "/dev/cu.usbmodem14101",
        "/dev/cu.usbserial-1410",
        "/dev/ttyUSB0",
        "/dev/ttyACM0"
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return None


def show_interactive_menu(client: RobotSerialClient,
                          kin_runner: KinematicCalibrationRunner,
                          diag_runner: PreflightDiagnosticsRunner,
                          gate: SafetyGate,
                          logger: CalibrationLogger):
    """Interactive CLI menu for selecting individual tests or complete calibration."""
    current_gate = gate

    while True:
        print(f"\n{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.CYAN}   MENU KALIBRASI KINEMATIKA & DIAGNOSTIK PRE-FLIGHT{Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        status_gate = f"{Colors.GREEN}UNLOCKED{Colors.RESET}" if current_gate.is_motion_permitted() else f"{Colors.RED}LOCKED{Colors.RESET}"
        print(f"  Status Hard-Gate Keamanan: [{status_gate}]")
        print("  [0] Jalankan Stage U0 (Pre-Flight Diagnostics & Wiring Check)")
        print("  [1] Jalankan U1 (Pulsa & Diameter Roda 100 cm)")
        print("  [2] Jalankan U2 (Rotasi Gyro In-Place ±90°)")
        print("  [3] Jalankan U3 (Lintasan Lurus 150 cm Berpemandu Gyro)")
        print("  [4] Jalankan U4 (Akurasi Ranging Sonar HC-SR04)")
        print("  [5] Jalankan Seluruh Suite Kalibrasi (U1 -> U4)")
        print("  [6] Tampilkan Calibration Profile EEPROM Aktif")
        print("  [7] Reset Profile EEPROM ke Default Pabrik")
        print("  [8] Keluar")
        print(f"{Colors.CYAN}----------------------------------------------------{Colors.RESET}")

        try:
            choice = input("Pilih menu [0-8]: ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if choice == "0":
            rep = diag_runner.run_u0()
            current_gate = SafetyGate(rep)
            current_gate.print_status()
        elif choice == "1":
            if not current_gate.is_motion_permitted():
                print(f"{Colors.RED}Ditolak: Hard-gate keamanan terkunci. Perbaiki U0 terlebih dahulu.{Colors.RESET}")
            else:
                kin_runner.run_u1()
        elif choice == "2":
            if not current_gate.is_motion_permitted():
                print(f"{Colors.RED}Ditolak: Hard-gate keamanan terkunci. Perbaiki U0 terlebih dahulu.{Colors.RESET}")
            else:
                kin_runner.run_u2()
        elif choice == "3":
            if not current_gate.is_motion_permitted():
                print(f"{Colors.RED}Ditolak: Hard-gate keamanan terkunci. Perbaiki U0 terlebih dahulu.{Colors.RESET}")
            else:
                kin_runner.run_u3()
        elif choice == "4":
            if not current_gate.is_motion_permitted():
                print(f"{Colors.RED}Ditolak: Hard-gate keamanan terkunci. Perbaiki U0 terlebih dahulu.{Colors.RESET}")
            else:
                kin_runner.run_u4()
        elif choice == "5":
            if not current_gate.is_motion_permitted():
                print(f"{Colors.RED}Ditolak: Hard-gate keamanan terkunci. Perbaiki U0 terlebih dahulu.{Colors.RESET}")
            else:
                kin_runner.run_all(current_gate)
        elif choice == "6":
            prof = client.read_profile()
            print(f"\n{Colors.BOLD}Profil Kalibrasi Aktif di EEPROM:{Colors.RESET}")
            print(f"  Diameter Roda      : {prof.diameter_roda_mm:.2f} mm")
            print(f"  Trim Motor Kiri    : {prof.trim_motor_kiri:.3f}")
            print(f"  Trim Motor Kanan   : {prof.trim_motor_kanan:.3f}")
            print(f"  Bias Gyro Z        : {prof.gyro_bias_z:.3f} deg/s")
            print(f"  Inversi Motor Kiri : {prof.invert_motor_kiri}")
            print(f"  Inversi Motor Kanan: {prof.invert_motor_kanan}\n")
        elif choice == "7":
            if client.reset_profile():
                print(f"{Colors.GREEN}✓ Profil EEPROM berhasil di-reset ke nilai default pabrik.{Colors.RESET}")
            else:
                print(f"{Colors.RED}✗ Gagal mereset profil EEPROM.{Colors.RESET}")
        elif choice == "8":
            print("Keluar dari Wizard Kalibrasi.")
            break
        else:
            print("Pilihan tidak valid.")


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Automated Calibration & Pre-Flight Diagnostic Wizard (U0-U4)")
    parser.add_argument("-p", "--port", type=str, default=None, help="Serial port perangkat (contoh: /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--mock", action="store_true", help="Gunakan mock robot untuk testing offline/CI")
    parser.add_argument("--headless", action="store_true", help="Jalankan dalam mode non-interaktif tanpa input pengguna")
    parser.add_argument("--test", type=str, default="ALL", choices=["U0", "U1", "U2", "U3", "U4", "ALL"],
                        help="Pilih pengujian spesifik untuk dijalankan (default: ALL)")
    parser.add_argument("--mode", type=str, default="full", choices=["full", "individual"],
                        help="Mode eksekusi: full (U0->U4) atau individual (menu pilihan)")
    parser.add_argument("--log-file", type=str, default="tools/calibration_log.csv",
                        help="Path output file log telemetri CSV")
    parser.add_argument("--fail-mpu", action="store_true", help="Simulasikan kegagalan MPU6050 pada mock mode")
    parser.add_argument("--fail-sonar", action="store_true", help="Simulasikan kegagalan Sonar pada mock mode")
    parser.add_argument("--fail-motors", action="store_true", help="Simulasikan kegagalan Motor/Encoder pada mock mode")
    parser.add_argument("--reverse-left", action="store_true", help="Simulasikan roda kiri berputar terbalik")

    args = parser.parse_args(argv)

    if args.mock:
        mock = MockRobotSerial()
        if args.fail_mpu:
            mock.set_mpu_ok(False)
        if args.fail_sonar:
            mock.set_sonar_dist(-1.0)
        if args.fail_motors:
            mock.set_motor_pulses(0, 0)

        client = RobotSerialClient(port=None, baud=args.baud, serial_backend=mock)
    else:
        port = args.port or autodetect_serial_port()
        if not port:
            print(f"{Colors.RED}Error: Port serial tidak ditemukan. Tentukan dengan opsi --port atau gunakan --mock.{Colors.RESET}")
            return 1
        print(f"{Colors.CYAN}Membuka koneksi serial ke robot pada {port} ({args.baud} baud)...{Colors.RESET}")
        try:
            client = RobotSerialClient(port=port, baud=args.baud)
        except Exception as e:
            print(f"{Colors.RED}Error: {e}{Colors.RESET}")
            return 1

    # Verify communication
    if not client.ping():
        print(f"{Colors.RED}Gagal berkomunikasi dengan firmware robot (!PING timeout).{Colors.RESET}")
        client.close()
        return 1

    print(f"{Colors.GREEN}✓ Handshake koneksi robot berhasil (!PONG).{Colors.RESET}")

    diag_runner = PreflightDiagnosticsRunner(client, interactive=(not args.headless))
    if args.reverse_left:
        diag_runner.mock_motor_directions(left_forward=False, right_forward=True)

    report = diag_runner.run_u0()
    gate = SafetyGate(report)
    gate.print_status()

    # Hard-gate enforcement
    if not gate.is_motion_permitted():
        if args.test in ["U1", "U2", "U3", "U4"]:
            gate.assert_motion_permitted()
        client.close()
        return 1

    if args.test == "U0":
        client.close()
        return 0

    logger = CalibrationLogger(filepath=args.log_file)
    logger.log_u0(report, client.read_profile())
    kin_runner = KinematicCalibrationRunner(client, logger=logger, interactive=(not args.headless))

    if args.headless:
        # Non-interactive CLI execution
        if args.test == "ALL":
            ok = kin_runner.run_all(gate)
            client.close()
            return 0 if ok else 1
        elif args.test == "U1":
            res = kin_runner.run_u1()
            client.close()
            return 0 if res.passed else 1
        elif args.test == "U2":
            res = kin_runner.run_u2()
            client.close()
            return 0 if res.passed else 1
        elif args.test == "U3":
            res = kin_runner.run_u3()
            client.close()
            return 0 if res.passed else 1
        elif args.test == "U4":
            res = kin_runner.run_u4()
            client.close()
            return 0 if res.passed else 1
    else:
        # Interactive mode
        mode_choice = args.mode
        if args.mode == "full" and args.test == "ALL":
            print(f"\n{Colors.BOLD}Pilih Mode Operasional Kalibrasi:{Colors.RESET}")
            print("  [1] Full Calibration Wizard (U0 -> U4)")
            print("  [2] Individual Test Mode (Pilih U0/U1/U2/U3/U4)")
            try:
                ans_mode = input("Pilih [1/2] (default 1): ").strip()
                if ans_mode == "2":
                    mode_choice = "individual"
            except (EOFError, KeyboardInterrupt):
                mode_choice = "full"

        if mode_choice == "individual":
            show_interactive_menu(client, kin_runner, diag_runner, gate, logger)
            client.close()
            return 0
        else:
            # Full wizard (U0 already passed, run U1-U4)
            ok = kin_runner.run_all(gate)
            client.close()
            return 0 if ok else 1

    client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
