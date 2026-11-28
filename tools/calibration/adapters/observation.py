"""
Observation adapter seam and implementations for ground-truth data collection.
"""

from typing import List, Optional, Tuple, Union
try:
    from typing import Protocol
except ImportError:
    from typing_extensions import Protocol

from tools.calibration.models import MpuMounting


class ObservationAdapter(Protocol):
    """Interface for obtaining ground-truth physical observations from a human or test script."""
    def verify_ir_sensors(self, initial_left: bool, initial_right: bool) -> bool: ...
    def ask_mpu_mounting(self) -> Optional[MpuMounting]: ...
    def verify_indicators(self) -> bool: ...
    def verify_motor_directions(self) -> Tuple[bool, bool]: ...
    def measure_u1_distance(self, firmware_dist_cm: float, iteration: int) -> float: ...
    def pause_for_u1_retry(self, iteration: int) -> None: ...
    def pause_for_u2_bias_sampling(self) -> None: ...
    def measure_u2_rotation(self, target_angle_deg: float, firmware_theta_deg: float) -> Tuple[float, bool]: ...
    def measure_u3_lateral_deviation(self, max_heading_err_deg: float) -> float: ...
    def prepare_u4_target(self, target_cm: float) -> None: ...


class ConsoleObservationAdapter:
    """Interactive CLI adapter prompting human operator via stdin."""
    def __init__(self, interactive: bool = True) -> None:
        self.interactive: bool = interactive

    def verify_ir_sensors(self, initial_left: bool, initial_right: bool) -> bool:
        if not self.interactive:
            return True
        try:
            ans = input("  Apakah LED indikator modul FC-51 aktif saat dihalangi? (y/n) [default y]: ").strip().lower()
            return ans != "n"
        except EOFError:
            return True

    def verify_indicators(self) -> bool:
        if not self.interactive:
            return True
        try:
            ans = input("  Apakah Buzzer berbunyi dan LED berkedip singkat? (y/n) [default y]: ").strip().lower()
            return ans != "n"
        except EOFError:
            return True

    def ask_mpu_mounting(self) -> Optional[MpuMounting]:
        """Asks how the GY-521 is mounted; None (keep stored sign) when not interactive."""
        if not self.interactive:
            return None
        print("  Bagaimana posisi modul MPU6050 (GY-521) di sasis?")
        print("    [1] Datar, chip/komponen menghadap ATAS")
        print("    [2] Datar, chip/komponen menghadap BAWAH (terbalik, pin header di atas)")
        print("    [3] Tegak / berdiri / miring (tidak datar)")
        print("  Ke mana panah sumbu X (tercetak di PCB, label 'X') menghadap?")
        print("    [1] Depan  [2] Belakang  [3] Kiri  [4] Kanan")
        try:
            facing = _pick(input("  Posisi chip [1-3] (default 1): "), MpuMounting.CHIP_FACING_CHOICES)
            x_axis = _pick(input("  Arah panah X [1-4] (default 1): "), MpuMounting.X_AXIS_CHOICES)
        except (EOFError, KeyboardInterrupt):
            return None
        return MpuMounting(facing, x_axis)

    def verify_motor_directions(self) -> Tuple[bool, bool]:
        if not self.interactive:
            return True, True
        left_forward = True
        right_forward = True
        try:
            ans_l = input("  Apakah RODA KIRI berputar MAJU? (y/n) [default y]: ").strip().lower()
            if ans_l == "n":
                left_forward = False
            ans_r = input("  Apakah RODA KANAN berputar MAJU? (y/n) [default y]: ").strip().lower()
            if ans_r == "n":
                right_forward = False
        except EOFError:
            pass
        return left_forward, right_forward

    def measure_u1_distance(self, firmware_dist_cm: float, iteration: int) -> float:
        if not self.interactive:
            return firmware_dist_cm
        prompt_str = f"  Masukkan jarak aktual meteran pita (cm) [default {firmware_dist_cm:.1f}]: "
        try:
            user_input = input(prompt_str).strip()
            return float(user_input) if user_input else firmware_dist_cm
        except (ValueError, EOFError):
            return firmware_dist_cm

    def pause_for_u1_retry(self, iteration: int) -> None:
        if not self.interactive:
            return
        try:
            input("  Tekan ENTER untuk memulai iterasi kalibrasi berikutnya...")
        except (EOFError, KeyboardInterrupt):
            pass

    def pause_for_u2_bias_sampling(self) -> None:
        if not self.interactive:
            return
        try:
            input("  Tekan ENTER untuk memulai sampling bias zero-rate...")
        except (EOFError, KeyboardInterrupt):
            pass

    def measure_u2_rotation(self, target_angle_deg: float, firmware_theta_deg: float) -> Tuple[float, bool]:
        if not self.interactive:
            return target_angle_deg, True
        direction_ok = True
        try:
            ans_dir = input("  Apakah robot berputar ke arah kiri (berlawanan jarum jam)? (y/n) [default y]: ").strip().lower()
            if ans_dir == "n":
                direction_ok = False
            prompt_str = f"  Masukkan sudut fisik aktual busur derajat lantai (°) [default {firmware_theta_deg:.1f}]: "
            user_input = input(prompt_str).strip()
            actual_angle = float(user_input) if user_input else firmware_theta_deg
        except (ValueError, EOFError):
            actual_angle = firmware_theta_deg
        return actual_angle, direction_ok

    def measure_u3_lateral_deviation(self, max_heading_err_deg: float) -> float:
        if not self.interactive:
            return 0.0
        prompt_str = "  Masukkan deviasi lateral terukur (cm) [default 0.0]: "
        try:
            user_input = input(prompt_str).strip()
            return float(user_input) if user_input else 0.0
        except (ValueError, EOFError):
            return 0.0

    def prepare_u4_target(self, target_cm: float) -> None:
        if not self.interactive:
            return
        try:
            input("  Tekan ENTER jika kotak sudah siap diukur...")
        except (EOFError, KeyboardInterrupt):
            pass


def _pick(answer: str, choices: Tuple[str, ...]) -> str:
    """Maps a 1-based menu answer to its choice; blank or invalid picks the first one."""
    answer = answer.strip()
    if answer.isdigit() and 1 <= int(answer) <= len(choices):
        return choices[int(answer) - 1]
    return choices[0]


def _split_sequence_or_single(
    val: Optional[Union[float, List[float]]]
) -> Tuple[Optional[List[float]], Optional[float]]:
    if isinstance(val, list):
        return [float(x) for x in val], None
    if val is None:
        return None, None
    return None, float(val)


class ScriptedObservationAdapter:
    """Deterministic observation adapter for automated testing and CI execution."""
    def __init__(self,
                 u1_ground_truth: Optional[Union[float, List[float]]] = None,
                 u2_actual_angle: Optional[Union[float, List[float]]] = None,
                 u2_direction_ok: bool = True,
                 u3_lateral_dev: Optional[Union[float, List[float]]] = None,
                 motor_directions: Tuple[bool, bool] = (True, True),
                 ir_verified: bool = True,
                 indicators_verified: bool = True,
                 mpu_mounting: Optional[MpuMounting] = None) -> None:
        self.u1_sequence, self.u1_single = _split_sequence_or_single(u1_ground_truth)
        self.u2_sequence, self.u2_single = _split_sequence_or_single(u2_actual_angle)
        self.u3_sequence, self.u3_single = _split_sequence_or_single(u3_lateral_dev)
        self.u2_direction_ok: bool = bool(u2_direction_ok)
        self.motor_directions: Tuple[bool, bool] = motor_directions
        self.ir_verified: bool = bool(ir_verified)
        self.indicators_verified: bool = bool(indicators_verified)
        self.mpu_mounting: Optional[MpuMounting] = mpu_mounting

    def set_u1_ground_truth(self, val: Union[float, List[float]]) -> None:
        self.u1_sequence, self.u1_single = _split_sequence_or_single(val)

    def set_u2_actual_angle(self, val: Union[float, List[float]]) -> None:
        self.u2_sequence, self.u2_single = _split_sequence_or_single(val)

    def set_u3_lateral_dev(self, val: Union[float, List[float]]) -> None:
        self.u3_sequence, self.u3_single = _split_sequence_or_single(val)

    def set_motor_directions(self, left_forward: bool, right_forward: bool) -> None:
        self.motor_directions = (bool(left_forward), bool(right_forward))

    def set_mpu_mounting(self, mounting: Optional[MpuMounting]) -> None:
        self.mpu_mounting = mounting

    def verify_ir_sensors(self, initial_left: bool, initial_right: bool) -> bool:
        return self.ir_verified

    def ask_mpu_mounting(self) -> Optional[MpuMounting]:
        return self.mpu_mounting

    def verify_indicators(self) -> bool:
        return self.indicators_verified

    def verify_motor_directions(self) -> Tuple[bool, bool]:
        return self.motor_directions

    def measure_u1_distance(self, firmware_dist_cm: float, iteration: int) -> float:
        if self.u1_sequence and len(self.u1_sequence) > 0:
            return float(self.u1_sequence.pop(0))
        if self.u1_single is not None:
            return float(self.u1_single)
        return float(firmware_dist_cm)

    def pause_for_u1_retry(self, iteration: int) -> None:
        pass

    def pause_for_u2_bias_sampling(self) -> None:
        pass

    def measure_u2_rotation(self, target_angle_deg: float, firmware_theta_deg: float) -> Tuple[float, bool]:
        if self.u2_sequence and len(self.u2_sequence) > 0:
            angle = float(self.u2_sequence.pop(0))
        elif self.u2_single is not None:
            angle = float(self.u2_single)
        else:
            angle = float(target_angle_deg)
        return angle, self.u2_direction_ok

    def measure_u3_lateral_deviation(self, max_heading_err_deg: float) -> float:
        if self.u3_sequence and len(self.u3_sequence) > 0:
            return float(self.u3_sequence.pop(0))
        if self.u3_single is not None:
            return float(self.u3_single)
        return 0.0

    def prepare_u4_target(self, target_cm: float) -> None:
        pass
