"""
Transport adapter seam and implementations for communicating with the robot MCU.
"""

import os
import time
from typing import Optional, Tuple, List, Any
try:
    from typing import Protocol
except ImportError:
    from typing_extensions import Protocol

from tools.calibration.models import DiagnosticResult, CalibrationProfileData


class TransportAdapter(Protocol):
    """Interface for sending ASCII commands and receiving telemetry frames from the robot MCU."""
    def send_command(self, cmd: str) -> str: ...
    def ping(self) -> bool: ...
    def probe_diagnostics(self) -> DiagnosticResult: ...
    def spin_motors(self, pwmL: int, pwmR: int, duration_ms: int) -> Tuple[int, int]: ...
    def test_buzzer(self, duration_ms: int = 200) -> bool: ...
    def read_profile(self) -> CalibrationProfileData: ...
    def write_profile(self, profile: CalibrationProfileData) -> bool: ...
    def reset_profile(self) -> bool: ...
    def step_u1(self) -> Tuple[int, int, float]: ...
    def step_u2(self, angle: float = 90.0) -> float: ...
    def step_u3(self) -> Tuple[int, int, float]: ...
    def step_u4(self) -> float: ...
    def close(self) -> None: ...


class MockRobotSerial:
    """Mock serial device simulating microcontroller responses for headless testing and CI."""
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
        self.mpu_ok = bool(ok)

    def set_sonar_dist(self, dist: float):
        self.sonar_dist = float(dist)

    def set_ir_states(self, left: bool, right: bool):
        self.ir_left = bool(left)
        self.ir_right = bool(right)

    def set_motor_pulses(self, left: int, right: int):
        self.pulses_left = int(left)
        self.pulses_right = int(right)

    def set_mock_u1_data(self, left: int, right: int, dist: float):
        self.mock_u1_data = (int(left), int(right), float(dist))

    def set_mock_u2_theta(self, theta: float):
        self.mock_u2_theta = float(theta)

    def set_mock_u3_data(self, left: int, right: int, heading_err: float):
        self.mock_u3_data = (int(left), int(right), float(heading_err))

    def set_mock_u4_distances(self, dists: List[float]):
        self.mock_u4_distances = [float(d) for d in dists]

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


class SerialTransportAdapter:
    """Transport adapter communicating with physical hardware or mock backends over serial."""
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
