"""
Observation adapter seam and implementations for ground-truth data collection.
"""

from typing import Optional, Tuple, List, Union
try:
    from typing import Protocol
except ImportError:
    from typing_extensions import Protocol


class ObservationAdapter(Protocol):
    """Interface for obtaining ground-truth physical observations from a human or test script."""
    def verify_ir_sensors(self, initial_left: bool, initial_right: bool) -> bool: ...
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
    def __init__(self, interactive: bool = True):
        self.interactive = interactive

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


class ScriptedObservationAdapter:
    """Deterministic observation adapter for automated testing and CI execution."""
    def __init__(self,
                 u1_ground_truth: Optional[Union[float, List[float]]] = None,
                 u2_actual_angle: Optional[Union[float, List[float]]] = None,
                 u2_direction_ok: bool = True,
                 u3_lateral_dev: Optional[Union[float, List[float]]] = None,
                 motor_directions: Tuple[bool, bool] = (True, True),
                 ir_verified: bool = True,
                 indicators_verified: bool = True):
        if isinstance(u1_ground_truth, list):
            self.u1_sequence = list(u1_ground_truth)
            self.u1_single = None
        else:
            self.u1_sequence = None
            self.u1_single = u1_ground_truth

        if isinstance(u2_actual_angle, list):
            self.u2_sequence = list(u2_actual_angle)
            self.u2_single = None
        else:
            self.u2_sequence = None
            self.u2_single = u2_actual_angle

        if isinstance(u3_lateral_dev, list):
            self.u3_sequence = list(u3_lateral_dev)
            self.u3_single = None
        else:
            self.u3_sequence = None
            self.u3_single = u3_lateral_dev

        self.u2_direction_ok = bool(u2_direction_ok)
        self.motor_directions = motor_directions
        self.ir_verified = bool(ir_verified)
        self.indicators_verified = bool(indicators_verified)

    def set_u1_ground_truth(self, val: Union[float, List[float]]):
        if isinstance(val, list):
            self.u1_sequence = list(val)
            self.u1_single = None
        else:
            self.u1_sequence = None
            self.u1_single = float(val)

    def set_u2_actual_angle(self, val: Union[float, List[float]]):
        if isinstance(val, list):
            self.u2_sequence = list(val)
            self.u2_single = None
        else:
            self.u2_sequence = None
            self.u2_single = float(val)

    def set_u3_lateral_dev(self, val: Union[float, List[float]]):
        if isinstance(val, list):
            self.u3_sequence = list(val)
            self.u3_single = None
        else:
            self.u3_sequence = None
            self.u3_single = float(val)

    def set_motor_directions(self, left_forward: bool, right_forward: bool):
        self.motor_directions = (bool(left_forward), bool(right_forward))

    def verify_ir_sensors(self, initial_left: bool, initial_right: bool) -> bool:
        return self.ir_verified

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
