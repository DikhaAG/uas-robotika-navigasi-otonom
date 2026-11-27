#!/usr/bin/env python3
"""
Deterministic documentation-to-code consistency checker for README.md.
Verifies links, test counts, Config.h constants, FSM states, ASCII protocol
commands, and encoder interrupt mode against the actual repository sources.
"""

import pathlib
import re
import sys
from typing import Dict, List


def check_markdown_links(readme_text: str, repo_root: pathlib.Path) -> List[str]:
    """Verify every relative markdown link in README.md resolves to an existing path."""
    errors: List[str] = []
    for match in re.finditer(r"\[[^\]]+\]\(([^)]+)\)", readme_text):
        target = match.group(1).strip()
        if target.startswith(("http://", "https://", "#")):
            continue
        rel_path = target.split("#", 1)[0]
        if not rel_path:
            continue
        resolved = repo_root / rel_path
        if not resolved.exists():
            errors.append(f"README.md link target does not exist: '{rel_path}'")
    return errors


def check_test_counts(readme_text: str, repo_root: pathlib.Path) -> List[str]:
    """Verify C++ suite count and Python unit test count match Makefile and tests/."""
    errors: List[str] = []
    makefile_text = (repo_root / "Makefile").read_text(encoding="utf-8")
    cpp_suites = re.findall(r"\btests/test_[a-zA-Z0-9_]+\b", makefile_text.split("tests/test_motors:", 1)[0])
    cpp_count = len(cpp_suites)

    m_cpp = re.search(r"(\d+)\s+suite unit test C\+\+", readme_text)
    if not m_cpp or int(m_cpp.group(1)) != cpp_count:
        found = m_cpp.group(1) if m_cpp else "missing"
        errors.append(f"README.md C++ suite count ({found}) != Makefile TESTS count ({cpp_count})")

    py_test_text = (repo_root / "tests" / "test_auto_calibrate.py").read_text(encoding="utf-8")
    py_count = len(re.findall(r"^\s*def test_[a-zA-Z0-9_]+\(", py_test_text, flags=re.MULTILINE))

    m_py = re.search(r"(\d+)\s+unit test Python", readme_text)
    if not m_py or int(m_py.group(1)) != py_count:
        found = m_py.group(1) if m_py else "missing"
        errors.append(f"README.md Python test count ({found}) != test_auto_calibrate.py count ({py_count})")

    return errors


def check_config_constants(readme_text: str, repo_root: pathlib.Path) -> List[str]:
    """Verify Config.h line references, constants, and room coordinates in README.md."""
    errors: List[str] = []
    config_lines = (repo_root / "firmware" / "RobotNavigasiOtonom" / "Config.h").read_text(encoding="utf-8").splitlines()
    config_text = "\n".join(config_lines)

    # Check lines 12-14 BOARD_TYPE block matches README reference "baris 12–14"
    board_block = "\n".join(config_lines[11:14])
    if "#define BOARD_TYPE BOARD_ARDUINO_UNO" not in board_block:
        errors.append("Config.h lines 12-14 no longer contain '#define BOARD_TYPE BOARD_ARDUINO_UNO' as cited in README.md")

    expected_tokens: Dict[str, str] = {
        "DIAMETER_RODA_MM": "65.0f",
        "JUMLAH_SLOT_ENCODER": "20",
        "JARAK_RODA_MM": "135.0f",
        "MPU6050_I2C_ADDR": "0x68",
        "JARAK_DECEL_CM": "30.0f",
        "JARAK_RINTANGAN_CM": "25.0f",
        "SUDUT_PINDAI_DEG": "30.0f",
        "JARAK_MUNDUR_CM": "15.0f",
        "LANE_OFFSET_Y_CM": "20.0f",
        "CORRIDOR_HALF_WIDTH": "40.0f",
        "PERIODE_ODOMETRI_MS": "20UL",
        "PERIODE_TELEMETRI_MS": "200UL",
        "TIMEOUT_GLOBAL_MS": "90000UL",
        "TIMEOUT_STALL_MS": "1000UL",
        "MAX_RETRIES_ESCAPE": "3",
        "BATAS_KORIDOR_Y_CM": "40.0f",
        "BATAS_KORIDOR_X_MIN": "-30.0f",
    }
    for macro, val in expected_tokens.items():
        pattern = rf"#define\s+{re.escape(macro)}\s+{re.escape(val)}\b"
        if not re.search(pattern, config_text):
            errors.append(f"Config.h macro {macro} != {val} expected by README.md")

    room_wp_expectations = [
        ("Room 1", "180.0f, 0.0f", "180.0f, 70.0f", "(180, 0) cm", "(180, +70) cm"),
        ("Room 2", "295.0f, 0.0f", "295.0f, 70.0f", "(295, 0) cm", "(295, +70) cm"),
        ("Room 3", "235.0f, 0.0f", "235.0f, -70.0f", "(235, 0) cm", "(235, -70) cm"),
        ("Room 4", "355.0f, 0.0f", "355.0f, -70.0f", "(355, 0) cm", "(355, -70) cm"),
    ]
    for room_name, wp1_c, wp2_c, wp1_doc, wp2_doc in room_wp_expectations:
        if wp1_c not in config_text or wp2_c not in config_text:
            errors.append(f"Config.h room waypoints changed for {room_name}")
        if wp1_doc not in readme_text or wp2_doc not in readme_text:
            errors.append(f"README.md missing expected waypoint docs for {room_name}: {wp1_doc}, {wp2_doc}")

    return errors


def check_protocol_and_fsm(readme_text: str, repo_root: pathlib.Path) -> List[str]:
    """Verify all ASCII commands and FSM states appear in README.md."""
    errors: List[str] = []

    cal_svc = (repo_root / "firmware" / "RobotNavigasiOtonom" / "CalibrationService.cpp").read_text(encoding="utf-8")
    cmds = sorted(set(re.findall(r'"(![A-Z0-9:]+)"', cal_svc)))
    # Exclude response prefixes/errors; check command tokens handled in executeCommand
    exec_section = cal_svc.split("CalibrationService::executeCommand", 1)[1]
    dispatch_cmds = sorted(set(re.findall(r'"(![A-Z0-9:]+)"', exec_section)))
    for raw_cmd in dispatch_cmds:
        if raw_cmd.startswith("!ERR:"):
            continue
        cmd_token = raw_cmd.rstrip(":")
        if cmd_token not in readme_text:
            errors.append(f"CalibrationService command '{cmd_token}' not documented in README.md")
    if not cmds:
        errors.append("Failed to parse command literals from CalibrationService.cpp")

    fsm_cpp = (repo_root / "firmware" / "RobotNavigasiOtonom" / "NavigationFSM.cpp").read_text(encoding="utf-8")
    states = re.findall(r'case\s+STATE_[A-Z]+:\s*return\s+"([A-Z]+)"', fsm_cpp)
    for st in states:
        if f"`{st}`" not in readme_text:
            errors.append(f"NavigationFSM state '{st}' missing from README.md")

    return errors


def check_interrupt_mode(readme_text: str, repo_root: pathlib.Path) -> List[str]:
    """Verify encoder interrupt edge mode matches across firmware and docs."""
    errors: List[str] = []
    odom_cpp = (repo_root / "firmware" / "RobotNavigasiOtonom" / "Odometry.cpp").read_text(encoding="utf-8")
    uji_enc = (repo_root / "firmware" / "Uji_Encoder" / "Uji_Encoder.ino").read_text(encoding="utf-8")
    pinout_md = (repo_root / "docs" / "hardware" / "pinout.md").read_text(encoding="utf-8")

    modes = set(re.findall(r"attachInterrupt\([^,]+,[^,]+,\s*(RISING|FALLING|CHANGE)\s*\)", odom_cpp + "\n" + uji_enc))
    if modes != {"CHANGE"}:
        errors.append(f"Unexpected encoder interrupt mode in firmware: {sorted(modes)}")
    if "Interupsi eksternal (`CHANGE` + filter glitch)" not in readme_text:
        errors.append("README.md encoder rows must document interrupt mode 'CHANGE' + glitch filter")
    if "INT0 (`CHANGE` + filter glitch)" not in pinout_md or "INT1 (`CHANGE` + filter glitch)" not in pinout_md:
        errors.append("docs/hardware/pinout.md encoder rows must document interrupt mode 'CHANGE' + glitch filter")

    return errors


def main() -> int:
    """Runs all README.md and docs consistency checks against the codebase."""
    repo_root = pathlib.Path(__file__).resolve().parent.parent
    readme_text = (repo_root / "README.md").read_text(encoding="utf-8")

    errors: List[str] = []
    errors.extend(check_markdown_links(readme_text, repo_root))
    errors.extend(check_test_counts(readme_text, repo_root))
    errors.extend(check_config_constants(readme_text, repo_root))
    errors.extend(check_protocol_and_fsm(readme_text, repo_root))
    errors.extend(check_interrupt_mode(readme_text, repo_root))

    if errors:
        for err in errors:
            print(f"ERROR: {err}", file=sys.stderr)
        return 1

    print("  ✓ Documentation consistency checks passed (README.md & docs/ match codebase).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
