"""
Command-line interface (CLI) and interactive wizard menu for calibration.
"""

import argparse
import sys
from typing import List, Optional

from tools.calibration.adapters.observation import (
    ConsoleObservationAdapter,
    ObservationAdapter,
    ScriptedObservationAdapter,
)
from tools.calibration.adapters.reporter import (
    AnsiConsoleReporter,
    CalibrationLogger,
    Colors,
    _CompositeReporter,
)
from tools.calibration.adapters.transport import (
    MockRobotSerial,
    SerialTransportAdapter,
    autodetect_serial_port,
)
from tools.calibration.engine import CalibrationEngine
from tools.calibration.models import SafetyGateError


def show_interactive_menu(engine: CalibrationEngine) -> None:
    """Interactive CLI menu for selecting individual tests or complete calibration."""
    while True:
        print(f"\n{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.CYAN}   MENU KALIBRASI KINEMATIKA & DIAGNOSTIK PRE-FLIGHT{Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.CYAN}===================================================={Colors.RESET}")
        status_gate = f"{Colors.GREEN}UNLOCKED{Colors.RESET}" if engine.is_motion_permitted() else f"{Colors.RED}LOCKED{Colors.RESET}"
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
            engine.run_preflight()
        elif choice in ("1", "2", "3", "4", "5"):
            if not engine.is_motion_permitted():
                print(f"{Colors.RED}Ditolak: Hard-gate keamanan terkunci. Jalankan Stage U0 terlebih dahulu.{Colors.RESET}")
            else:
                stage_map = {"1": "U1", "2": "U2", "3": "U3", "4": "U4", "5": "ALL"}
                engine.calibrate(stage=stage_map[choice])
        elif choice == "6":
            prof = engine.get_active_profile()
            print(f"\n{Colors.BOLD}Profil Kalibrasi Aktif di EEPROM:{Colors.RESET}")
            print(f"  Diameter Roda      : {prof.diameter_roda_mm:.2f} mm")
            print(f"  Trim Motor Kiri    : {prof.trim_motor_kiri:.3f}")
            print(f"  Trim Motor Kanan   : {prof.trim_motor_kanan:.3f}")
            print(f"  Bias Gyro Z        : {prof.gyro_bias_z:.3f} deg/s")
            print(f"  Inversi Motor Kiri : {prof.invert_motor_kiri}")
            print(f"  Inversi Motor Kanan: {prof.invert_motor_kanan}\n")
        elif choice == "7":
            if engine.reset_profile():
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

    # 1. Wire Transport Adapter
    if args.mock:
        mock = MockRobotSerial()
        if args.fail_mpu:
            mock.set_mpu_ok(False)
        if args.fail_sonar:
            mock.set_sonar_dist(-1.0)
        if args.fail_motors:
            mock.set_motor_pulses(0, 0)
        transport = SerialTransportAdapter(port=None, baud=args.baud, serial_backend=mock)
    else:
        port = args.port or autodetect_serial_port()
        if not port:
            print(f"{Colors.RED}Error: Port serial tidak ditemukan. Tentukan dengan opsi --port atau gunakan --mock.{Colors.RESET}")
            return 1
        print(f"{Colors.CYAN}Membuka koneksi serial ke robot pada {port} ({args.baud} baud)...{Colors.RESET}")
        try:
            transport = SerialTransportAdapter(port=port, baud=args.baud)
        except Exception as e:
            print(f"{Colors.RED}Error: {e}{Colors.RESET}")
            return 1

    # Verify communication
    if not transport.ping():
        print(f"{Colors.RED}Gagal berkomunikasi dengan firmware robot (!PING timeout).{Colors.RESET}")
        transport.close()
        return 1

    print(f"{Colors.GREEN}✓ Handshake koneksi robot berhasil (!PONG).{Colors.RESET}")

    # 2. Wire Observation Adapter
    observation: ObservationAdapter
    if args.headless:
        motor_dirs = (False, True) if args.reverse_left else (True, True)
        observation = ScriptedObservationAdapter(motor_directions=motor_dirs)
    elif args.mock and (args.reverse_left or args.fail_mpu or args.fail_sonar or args.fail_motors):
        motor_dirs = (False, True) if args.reverse_left else (True, True)
        observation = ScriptedObservationAdapter(motor_directions=motor_dirs)
    else:
        observation = ConsoleObservationAdapter(interactive=True)

    # 3. Wire Reporter Adapter (Console + CSV Logger via ReporterAdapter seam)
    reporter = _CompositeReporter([
        AnsiConsoleReporter(verbose=True),
        CalibrationLogger(filepath=args.log_file),
    ])

    # 4. Construct Deep CalibrationEngine
    engine = CalibrationEngine(
        transport=transport,
        observation=observation,
        reporter=reporter,
    )

    try:
        # Step 1: Pre-flight Diagnostics
        engine.run_preflight()

        # Hard-gate enforcement
        if not engine.is_motion_permitted():
            if args.test in ("U1", "U2", "U3", "U4"):
                raise SafetyGateError(
                    f"HARD-GATE LOCKED: Motion calibration ({args.test}) DILARANG sebelum pengujian hardware U0 100% lulus!"
                )
            return 1

        if args.test == "U0":
            return 0

        # Step 2: Kinematic Calibration
        if args.headless:
            cal_rep = engine.calibrate(stage=args.test)
            return 0 if cal_rep.all_passed else 1

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
            show_interactive_menu(engine)
            return 0

        cal_rep = engine.calibrate(stage=args.test)
        return 0 if cal_rep.all_passed else 1
    finally:
        transport.close()


if __name__ == "__main__":
    sys.exit(main())
