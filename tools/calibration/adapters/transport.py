"""
Transport adapter seam and implementations for communicating with the robot MCU.
"""

import os
import time
from typing import Any, Dict, List, Optional, Tuple
try:
    from typing import Protocol
except ImportError:
    from typing_extensions import Protocol

from tools.calibration.models import (
    CalibrationProfileData,
    DiagnosticResult,
    EncoderTranslationTelemetry,
    HeadingTelemetry,
    MotorPulses,
)


class TransportAdapter(Protocol):
    """Interface for sending ASCII commands and receiving telemetry frames from the robot MCU."""
    def send_command(self, cmd: str) -> str: ...
    def ping(self) -> bool: ...
    def probe_diagnostics(self) -> DiagnosticResult: ...
    def spin_motors(self, pwmL: int, pwmR: int, duration_ms: int) -> MotorPulses: ...
    def test_buzzer(self, duration_ms: int = 200) -> bool: ...
    def read_profile(self) -> CalibrationProfileData: ...
    def write_profile(self, profile: CalibrationProfileData) -> bool: ...
    def reset_profile(self) -> bool: ...
    def step_u1(self) -> EncoderTranslationTelemetry: ...
    def step_u2(self, angle: float = 90.0) -> float: ...
    def step_u3(self) -> HeadingTelemetry: ...
    def step_u4(self) -> float: ...
    def wait_for_gyro_bias(self, duration_s: float = 2.0) -> None: ...
    def close(self) -> None: ...


# The firmware answers a command only after the action finishes. These are its
# worst-case durations (CalibrationService.cpp loop limits), plus margin for
# serial latency; a reply that arrives after the host gave up would otherwise be
# read as the answer to the NEXT command.
_DEFAULT_RESPONSE_TIMEOUT_S: float = 2.0
_RESPONSE_MARGIN_S: float = 3.0
_STEP_FIRMWARE_LIMIT_S: Dict[str, float] = {
    "!STEP:U1": 8.0,   # 100 cm drive
    "!STEP:U2": 5.0,   # in-place turn
    "!STEP:U3": 12.0,  # 150 cm gyro-guided drive
}


def response_timeout_s(cmd: str) -> float:
    """Seconds the host waits for the reply to `cmd` before giving up."""
    for prefix, limit_s in _STEP_FIRMWARE_LIMIT_S.items():
        if cmd.startswith(prefix):
            return limit_s + _RESPONSE_MARGIN_S
    if cmd.startswith("!DIAG:MOTOR:") or cmd.startswith("!DIAG:BEEP:"):
        try:
            duration_ms = int(cmd.rsplit(":", 1)[1])
        except ValueError:
            duration_ms = 0
        return duration_ms / 1000.0 + _RESPONSE_MARGIN_S
    return _DEFAULT_RESPONSE_TIMEOUT_S


class _AsciiProtocolTransportMixin:
    """Shared ASCII command/response protocol mapping over send_command()."""

    def send_command(self, cmd: str) -> str:
        raise NotImplementedError

    def ping(self) -> bool:
        return self.send_command("!PING") == "!PONG"

    def probe_diagnostics(self) -> DiagnosticResult:
        resp = self.send_command("!DIAG:ALL")
        if not resp.startswith("!DIAG:RES:"):
            raise ValueError(f"Respon diagnostik tidak valid: {resp}")

        parts = resp[10:].split(":")
        if len(parts) < 4:
            raise ValueError(f"Format respon diagnostik kurang lengkap: {resp}")

        return DiagnosticResult(
            mpu_ok=(parts[0] == "1"),
            sonar_dist_cm=float(parts[1]),
            ir_left=(parts[2] == "1"),
            ir_right=(parts[3] == "1"),
        )

    def spin_motors(self, pwmL: int, pwmR: int, duration_ms: int) -> MotorPulses:
        resp = self.send_command(f"!DIAG:MOTOR:{pwmL}:{pwmR}:{duration_ms}")
        if not resp.startswith("!PULSES:"):
            raise ValueError(f"Respon motor spin tidak valid: {resp}")

        parts = resp[8:].split(":")
        if len(parts) < 2:
            raise ValueError(f"Format pulsa encoder tidak valid: {resp}")

        return MotorPulses(int(parts[0]), int(parts[1]))

    def test_buzzer(self, duration_ms: int = 200) -> bool:
        return self.send_command(f"!DIAG:BEEP:{duration_ms}") == "!OK"

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
            invert_motor_kanan=int(parts[5]),
        )

    def write_profile(self, profile: CalibrationProfileData) -> bool:
        return self.send_command(f"!CALIB:WRITE:{profile.to_command_args()}") == "!OK"

    def reset_profile(self) -> bool:
        return self.send_command("!CALIB:RESET") == "!RESET:OK"

    @staticmethod
    def _parse_triple_telemetry(resp: str, expected_prefix: str) -> Tuple[int, int, float]:
        if not resp.startswith(expected_prefix):
            raise ValueError(f"Respon tidak valid: {resp}")
        parts = resp[len(expected_prefix):].split(":")
        if len(parts) < 3:
            raise ValueError(f"Format respon telemetry tidak lengkap: {resp}")
        return int(parts[0]), int(parts[1]), float(parts[2])

    @staticmethod
    def _parse_single_float_telemetry(resp: str, expected_prefix: str, stage_label: str) -> float:
        if not resp.startswith(expected_prefix):
            raise ValueError(f"Respon {stage_label} tidak valid: {resp}")
        return float(resp[len(expected_prefix):])

    def step_u1(self) -> EncoderTranslationTelemetry:
        resp = self.send_command("!STEP:U1")
        pulses_l, pulses_r, dist = self._parse_triple_telemetry(resp, "!U1:DONE:")
        return EncoderTranslationTelemetry(pulses_l, pulses_r, dist)

    def step_u2(self, angle: float = 90.0) -> float:
        resp = self.send_command(f"!STEP:U2:{angle:.2f}")
        return self._parse_single_float_telemetry(resp, "!U2:DONE:", "STEP:U2")

    def step_u3(self) -> HeadingTelemetry:
        resp = self.send_command("!STEP:U3")
        pulses_l, pulses_r, err = self._parse_triple_telemetry(resp, "!U3:DONE:")
        return HeadingTelemetry(pulses_l, pulses_r, err)

    def step_u4(self) -> float:
        resp = self.send_command("!STEP:U4")
        return self._parse_single_float_telemetry(resp, "!U4:DONE:", "STEP:U4")


class MockRobotSerial(_AsciiProtocolTransportMixin):
    """
    In-memory robot simulator satisfying both the raw serial backend interface
    and the TransportAdapter seam for headless testing and CI.
    """
    def __init__(self) -> None:
        self.is_open: bool = True
        self.mpu_ok: bool = True
        self.sonar_dist: float = 25.0
        self.ir_left: bool = False
        self.ir_right: bool = False
        self.pulses_left: int = 52
        self.pulses_right: int = 51
        self.profile: CalibrationProfileData = CalibrationProfileData()
        self.last_command: str = ""
        self.mock_u1_data: Tuple[int, int, float] = (98, 99, 100.12)
        self.mock_u2_theta: float = 90.0
        self.mock_u3_data: Tuple[int, int, float] = (147, 148, 1.85)
        self.mock_u4_distances: List[float] = [10.2, 20.1, 39.8]

    def set_mpu_ok(self, ok: bool) -> None:
        self.mpu_ok = bool(ok)

    def set_sonar_dist(self, dist: float) -> None:
        self.sonar_dist = float(dist)

    def set_ir_states(self, left: bool, right: bool) -> None:
        self.ir_left = bool(left)
        self.ir_right = bool(right)

    def set_motor_pulses(self, left: int, right: int) -> None:
        self.pulses_left = int(left)
        self.pulses_right = int(right)

    def set_mock_u1_data(self, left: int, right: int, dist: float) -> None:
        self.mock_u1_data = (int(left), int(right), float(dist))

    def set_mock_u2_theta(self, theta: float) -> None:
        self.mock_u2_theta = float(theta)

    def set_mock_u3_data(self, left: int, right: int, heading_err: float) -> None:
        self.mock_u3_data = (int(left), int(right), float(heading_err))

    def set_mock_u4_distances(self, dists: List[float]) -> None:
        self.mock_u4_distances = [float(d) for d in dists]

    def send_command(self, cmd: str) -> str:
        self.write((cmd.strip() + "\n").encode("ascii"))
        return self.readline().decode("ascii", errors="replace").strip()

    def write(self, data: bytes) -> None:
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

    def reset_input_buffer(self) -> None:
        """Raw-backend parity with pyserial; the simulator never has stale bytes."""

    def wait_for_gyro_bias(self, duration_s: float = 2.0) -> None:
        pass

    def close(self) -> None:
        self.is_open = False


# Must equal SERIAL_BAUD in firmware/RobotNavigasiOtonom/Config.h (HC-05 factory baud).
DEFAULT_BAUD: int = 9600


class SerialTransportAdapter(_AsciiProtocolTransportMixin):
    """Transport adapter communicating with physical hardware or mock backends over serial."""
    def __init__(self, port: Optional[str] = None, baud: int = DEFAULT_BAUD,
                 timeout: float = 1.5, serial_backend: Any = None) -> None:
        self.port: Optional[str] = port
        self.baud: int = baud
        self.timeout: float = timeout
        self.ser: Any = serial_backend

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

        # Drop anything already waiting (boot banner, a reply that arrived after
        # an earlier timeout) so it cannot be mistaken for this command's reply.
        self.ser.reset_input_buffer()
        self.ser.write((cmd.strip() + "\n").encode("ascii"))

        # readline() returns b"" each time the per-read timeout expires; keep
        # reading until a protocol reply ('!...') arrives or the command's own
        # deadline passes. Non-protocol lines ('# STATE -> ...') are skipped.
        deadline = time.monotonic() + response_timeout_s(cmd.strip())
        while time.monotonic() < deadline:
            line = self.ser.readline().decode("ascii", errors="replace").strip()
            if line.startswith("!"):
                return line
        return ""

    def wait_for_gyro_bias(self, duration_s: float = 2.0) -> None:
        if self.ser is not None and not isinstance(self.ser, MockRobotSerial):
            time.sleep(duration_s)

    def close(self) -> None:
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
        "/dev/ttyACM0",
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return None
