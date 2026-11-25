"""
Reporter and telemetry adapters for displaying UI progress and writing CSV logs.
"""

import os
import sys
import time
from typing import Optional, List
try:
    from typing import Protocol
except ImportError:
    from typing_extensions import Protocol

from tools.calibration.models import U0Report, CalibrationProfileData


class Colors:
    """ANSI color codes for terminal styling."""
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


class ReporterAdapter(Protocol):
    """Interface for reporting calibration events, progress, and telemetry."""
    def on_stage_start(self, stage_name: str, description: str) -> None: ...
    def on_subtest_result(self, name: str, passed: bool, details: str, hints: Optional[List[str]] = None) -> None: ...
    def on_trial_progress(self, test_id: str, iteration: int, max_iterations: int, details: str) -> None: ...
    def on_trial_result(self, test_id: str, iteration: int, status: str, target: float,
                        measured: float, ground_truth: float, error: float,
                        profile: Optional[CalibrationProfileData] = None, notes: str = "") -> None: ...
    def on_stage_summary(self, title: str, passed: bool, summary_lines: List[str]) -> None: ...
    def on_hard_gate_status(self, is_permitted: bool, report: U0Report) -> None: ...
    def on_info(self, message: str) -> None: ...
    def on_warning(self, message: str) -> None: ...
    def on_error(self, message: str) -> None: ...
    def on_success(self, message: str) -> None: ...


class NullReporter:
    """Silent reporter discarding all events (ideal for unit testing)."""
    def on_stage_start(self, stage_name: str, description: str) -> None: pass
    def on_subtest_result(self, name: str, passed: bool, details: str, hints: Optional[List[str]] = None) -> None: pass
    def on_trial_progress(self, test_id: str, iteration: int, max_iterations: int, details: str) -> None: pass
    def on_trial_result(self, test_id: str, iteration: int, status: str, target: float,
                        measured: float, ground_truth: float, error: float,
                        profile: Optional[CalibrationProfileData] = None, notes: str = "") -> None: pass
    def on_stage_summary(self, title: str, passed: bool, summary_lines: List[str]) -> None: pass
    def on_hard_gate_status(self, is_permitted: bool, report: U0Report) -> None: pass
    def on_info(self, message: str) -> None: pass
    def on_warning(self, message: str) -> None: pass
    def on_error(self, message: str) -> None: pass
    def on_success(self, message: str) -> None: pass


class AnsiConsoleReporter:
    """Rich interactive terminal reporter using ANSI colors and formatted banners."""
    def __init__(self, verbose: bool = True):
        self.verbose = verbose

    def log(self, *args, **kwargs):
        if self.verbose:
            print(*args, **kwargs)

    def on_stage_start(self, stage_name: str, description: str) -> None:
        self.log(f"\n{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}  {stage_name.upper()}: {description.upper()}{Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}\n")

    def on_subtest_result(self, name: str, passed: bool, details: str, hints: Optional[List[str]] = None) -> None:
        if passed:
            self.log(f"  {Colors.GREEN}✓ [{name}] {details}{Colors.RESET}")
        else:
            self.log(f"  {Colors.RED}✗ [{name}] {details}{Colors.RESET}")
            if hints:
                for h in hints:
                    self.log(f"    {Colors.YELLOW}Petunjuk: {h}{Colors.RESET}")

    def on_trial_progress(self, test_id: str, iteration: int, max_iterations: int, details: str) -> None:
        self.log(f"{Colors.BOLD}[{test_id} - Iterasi {iteration}/{max_iterations}] {details}{Colors.RESET}")

    def on_trial_result(self, test_id: str, iteration: int, status: str, target: float,
                        measured: float, ground_truth: float, error: float,
                        profile: Optional[CalibrationProfileData] = None, notes: str = "") -> None:
        if status == "PASS":
            self.log(f"  {Colors.GREEN}✓ {test_id} Lulus: Terukur={measured:.2f}, Aktual={ground_truth:.2f}, Galat={error:.2f}. {notes}{Colors.RESET}")
        elif status == "RETRY":
            self.log(f"  {Colors.YELLOW}⚠ {test_id} Iterasi {iteration} Belum Konvergen: Galat={error:.2f}. {notes}{Colors.RESET}")
        else:
            self.log(f"  {Colors.RED}✗ {test_id} Gagal: Galat={error:.2f}. {notes}{Colors.RESET}")

    def on_stage_summary(self, title: str, passed: bool, summary_lines: List[str]) -> None:
        self.log(f"\n{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}          {title.upper()}     {Colors.RESET}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        for line in summary_lines:
            self.log(f"  {line}")
        self.log(f"{Colors.BOLD}{Colors.CYAN}----------------------------------------------------{Colors.RESET}")
        if passed:
            self.log(f"  {Colors.BOLD}{Colors.GREEN}STATUS AKHIR: PROTOKOL LULUS!{Colors.RESET}\n")
        else:
            self.log(f"  {Colors.BOLD}{Colors.RED}STATUS AKHIR: PROTOKOL BELUM LULUS TOLERANSI.{Colors.RESET}\n")

    def on_hard_gate_status(self, is_permitted: bool, report: U0Report) -> None:
        if is_permitted:
            self.log(f"\n{Colors.BOLD}{Colors.GREEN}===================================================={Colors.RESET}")
            self.log(f"{Colors.BOLD}{Colors.GREEN}  HARD-GATE STATUS: UNLOCKED (MOTION PERMITTED)      {Colors.RESET}")
            self.log(f"{Colors.BOLD}{Colors.GREEN}===================================================={Colors.RESET}")
            self.log("  Seluruh perangkat keras terverifikasi normal. Tahap kalibrasi kinematika (U1-U4) siap dijalankan.\n")
        else:
            self.log(f"\n{Colors.BOLD}{Colors.RED}===================================================={Colors.RESET}")
            self.log(f"{Colors.BOLD}{Colors.RED}  HARD-GATE STATUS: LOCKED (MOTION BLOCKED!)        {Colors.RESET}")
            self.log(f"{Colors.BOLD}{Colors.RED}===================================================={Colors.RESET}")
            self.log("  DILARANG melakukan kalibrasi gerak sebelum kerusakan hardware diperbaiki!\n")
            self.log(f"{Colors.BOLD}Petunjuk Perbaikan Perkabelan (Wiring Repair):{Colors.RESET}")
            hints = report.get_wiring_repair_instructions()
            if hints:
                for idx, hint in enumerate(hints, 1):
                    self.log(f"  {idx}. {Colors.YELLOW}{hint}{Colors.RESET}")
            else:
                self.log(f"  {Colors.YELLOW}Periksa koneksi fisik dan ulangi pengujian U0.{Colors.RESET}")
            self.log("")

    def on_info(self, message: str) -> None:
        self.log(f"  {Colors.CYAN}{message}{Colors.RESET}")

    def on_warning(self, message: str) -> None:
        self.log(f"  {Colors.YELLOW}{message}{Colors.RESET}")

    def on_error(self, message: str) -> None:
        self.log(f"  {Colors.RED}{message}{Colors.RESET}")

    def on_success(self, message: str) -> None:
        self.log(f"  {Colors.GREEN}{message}{Colors.RESET}")


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


class CompositeReporter:
    """Dispatches reporter events to multiple underlying adapters."""
    def __init__(self, reporters: List[ReporterAdapter]):
        self.reporters = list(reporters)

    def on_stage_start(self, stage_name: str, description: str) -> None:
        for r in self.reporters:
            r.on_stage_start(stage_name, description)

    def on_subtest_result(self, name: str, passed: bool, details: str, hints: Optional[List[str]] = None) -> None:
        for r in self.reporters:
            r.on_subtest_result(name, passed, details, hints)

    def on_trial_progress(self, test_id: str, iteration: int, max_iterations: int, details: str) -> None:
        for r in self.reporters:
            r.on_trial_progress(test_id, iteration, max_iterations, details)

    def on_trial_result(self, test_id: str, iteration: int, status: str, target: float,
                        measured: float, ground_truth: float, error: float,
                        profile: Optional[CalibrationProfileData] = None, notes: str = "") -> None:
        for r in self.reporters:
            r.on_trial_result(test_id, iteration, status, target, measured, ground_truth, error, profile, notes)

    def on_stage_summary(self, title: str, passed: bool, summary_lines: List[str]) -> None:
        for r in self.reporters:
            r.on_stage_summary(title, passed, summary_lines)

    def on_hard_gate_status(self, is_permitted: bool, report: U0Report) -> None:
        for r in self.reporters:
            r.on_hard_gate_status(is_permitted, report)

    def on_info(self, message: str) -> None:
        for r in self.reporters:
            r.on_info(message)

    def on_warning(self, message: str) -> None:
        for r in self.reporters:
            r.on_warning(message)

    def on_error(self, message: str) -> None:
        for r in self.reporters:
            r.on_error(message)

    def on_success(self, message: str) -> None:
        for r in self.reporters:
            r.on_success(message)
