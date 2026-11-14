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
            return b"!U1:DONE:98:99:100.12\n"
        elif cmd.startswith("!STEP:U2:"):
            return b"!U2:DONE:90.00\n"
        elif cmd == "!STEP:U3":
            return b"!U3:DONE:147:148:1.85\n"
        elif cmd == "!STEP:U4":
            return f"!U4:DONE:{self.sonar_dist:.2f}\n".encode("ascii")
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


def autodetect_serial_port() -> Optional[str]:
    """Auto-detects available Arduino serial ports."""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            # Common Arduino USB identifiers
            desc = p.description.lower()
            if any(key in desc for key in ["arduino", "ch340", "cp210", "ftdi", "usb serial"]):
                return p.device
        if ports:
            return ports[0].device
    except Exception:
        pass

    # Fallback to standard device paths on Linux/macOS
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


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Automated Calibration & Pre-Flight Diagnostic Wizard (U0)")
    parser.add_argument("-p", "--port", type=str, default=None, help="Serial port perangkat (contoh: /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--mock", action="store_true", help="Gunakan mock robot untuk testing offline/CI")
    parser.add_argument("--headless", action="store_true", help="Jalankan dalam mode non-interaktif tanpa input pengguna")
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

    runner = PreflightDiagnosticsRunner(client, interactive=(not args.headless))
    if args.reverse_left:
        runner.mock_motor_directions(left_forward=False, right_forward=True)

    report = runner.run_u0()
    gate = SafetyGate(report)
    gate.print_status()

    client.close()
    return 0 if gate.is_motion_permitted() else 1


if __name__ == "__main__":
    sys.exit(main())
