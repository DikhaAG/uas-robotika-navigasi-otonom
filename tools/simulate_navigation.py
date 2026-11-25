#!/usr/bin/env python3
"""
Simulasi Kinematika Robot Navigasi Otonom (Praktikum Robotika TC253421)
Harness wrapper yang mengeksekusi binary native firmware C++ (tools/sim_navigation)
terhadap model fisika arena 480x80 cm dan rintangan secara deterministik.
"""

import sys
import os
import argparse
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, List, Union

REPO_ROOT: Path = Path(__file__).resolve().parent.parent
SIM_BIN: Path = REPO_ROOT / "tools" / "sim_navigation"

@dataclass
class Obstacle:
    """Bounding box rintangan arena (x, y, width, height) dalam cm."""
    x: float
    y: float
    w: float
    h: float

    def to_cli_arg(self) -> str:
        return f"{self.x:.1f},{self.y:.1f},{self.w:.1f},{self.h:.1f}"

@dataclass
class SimulationResult:
    """Hasil eksekusi skenario simulasi C++."""
    success: bool
    stdout: str
    stderr: str

def ensure_sim_binary() -> None:
    """Memastikan binary native C++ tools/sim_navigation sudah terkompilasi."""
    if not SIM_BIN.exists():
        print("[simulate_navigation] Mengompilasi binary tools/sim_navigation via make...")
        subprocess.run(["make", "tools/sim_navigation"], cwd=str(REPO_ROOT), check=True)
    if not os.access(SIM_BIN, os.X_OK):
        SIM_BIN.chmod(0o755)

def parse_obstacle_str(obs_str: str) -> Optional[Obstacle]:
    """Parse string format 'x,y,w,h' menjadi dataclass Obstacle."""
    parts = [float(p.strip()) for p in obs_str.split(",")]
    if len(parts) == 4:
        return Obstacle(x=parts[0], y=parts[1], w=parts[2], h=parts[3])
    return None

def run_scenario(
    room: int,
    obstacles: Optional[List[Obstacle]] = None,
    csv_output: Optional[Union[str, Path]] = None
) -> SimulationResult:
    """Mengeksekusi simulasi satu skenario menggunakan C++ harness."""
    ensure_sim_binary()
    cmd: List[str] = [str(SIM_BIN), "--room", str(room)]

    if csv_output:
        cmd.extend(["--csv", str(csv_output)])

    if obstacles:
        for obs in obstacles:
            cmd.extend(["--obs", obs.to_cli_arg()])

    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return SimulationResult(
        success=(proc.returncode == 0),
        stdout=proc.stdout.strip(),
        stderr=proc.stderr.strip()
    )

def main() -> int:
    parser = argparse.ArgumentParser(description="Simulasi Kinematika Robot Navigasi Otonom (C++ Native)")
    parser.add_argument("--room", type=int, choices=[1, 2, 3, 4], help="Jalankan simulasi spesifik untuk ruangan (1-4)")
    parser.add_argument("--obs", action="append", help="Tambah rintangan format 'x,y,w,h' (contoh: '100,0,20,20')")
    parser.add_argument("-o", "--output", default="tools/sample_telemetry.csv", help="Lokasi file output CSV telemetri (default: tools/sample_telemetry.csv)")
    parser.add_argument("--plot", action="store_true", help="Otomatis buat file SVG lintasan setelah simulasi selesai")
    parser.add_argument("--svg", default="tools/trajectory_plot.svg", help="Lokasi file output plot SVG (default: tools/trajectory_plot.svg)")
    args = parser.parse_args()

    ensure_sim_binary()

    cli_obstacles: List[Obstacle] = []
    if args.obs:
        for s in args.obs:
            parsed = parse_obstacle_str(s)
            if parsed:
                cli_obstacles.append(parsed)
            else:
                print(f"Peringatan: Format rintangan diabaikan: {s} (gunakan format x,y,w,h)", file=sys.stderr)

    if args.room is not None:
        # Single room simulation mode
        print(f"=== MENJALANKAN SIMULASI NAVIGASI: ROOM {args.room} ===")
        res = run_scenario(args.room, obstacles=cli_obstacles, csv_output=args.output)
        if res.stdout:
            print(res.stdout)
        if res.stderr:
            print(res.stderr, file=sys.stderr)

        if res.success:
            print(f"  ✓ Telemetri diekspor ke: {args.output}")
            if args.plot:
                plot_cmd: List[str] = [sys.executable, str(REPO_ROOT / "tools" / "plot_trajectory.py"), args.output, "-o", args.svg]
                if cli_obstacles:
                    for obs in cli_obstacles:
                        plot_cmd.extend(["--obs", obs.to_cli_arg()])
                else:
                    plot_cmd.extend(["--obs", "none"])
                subprocess.run(plot_cmd, check=True)
            return 0
        else:
            print(f"  ✗ Simulasi Room {args.room} GAGAL!")
            return 1

    print("=== RUNNING FULL AUTONOMOUS NAVIGATION SIMULATION SUITE ===")

    # Scenarios from Course Brief TC253421 (Babak 1, 2, 3)
    scenarios = [
        {"name": "Babak 1 (Room 1, No Obstacles)", "room": 1, "obs": []},
        {"name": "Babak 2 (Room 2, Single Obstacle)", "room": 2, "obs": [Obstacle(100.0, 0.0, 20.0, 20.0)]},
        {"name": "Babak 3 (Room 3, Dual Obstacles)", "room": 3, "obs": [Obstacle(80.0, 0.0, 20.0, 20.0), Obstacle(160.0, 0.0, 20.0, 20.0)]},
        {"name": "Babak 3 Alt (Room 4, Single Obstacle)", "room": 4, "obs": [Obstacle(120.0, 0.0, 20.0, 20.0)]},
    ]

    all_passed = True
    for sc in scenarios:
        csv_out = args.output if sc["room"] == 1 else None
        res = run_scenario(sc["room"], obstacles=sc["obs"], csv_output=csv_out)

        print(f"\n--- Scenario: {sc['name']} ---")
        if res.stdout:
            print(res.stdout)
        if res.stderr:
            print(res.stderr, file=sys.stderr)

        if res.success:
            print(f"  ✓ Scenario {sc['name']} PASSED.")
        else:
            all_passed = False
            print(f"  ✗ Scenario {sc['name']} FAILED!")

    if not all_passed:
        print("\n✗ ONE OR MORE SIMULATION SCENARIOS FAILED!", file=sys.stderr)
        return 1

    print(f"  ✓ Telemetri Room 1 diekspor ke: {args.output}")
    if args.plot:
        subprocess.run([sys.executable, str(REPO_ROOT / "tools" / "plot_trajectory.py"), args.output, "-o", args.svg], check=True)

    print("\n✓ ALL SIMULATION SUITE SCENARIOS (BABAK 1, 2, 3 / ROOMS 1-4) PASSED!")
    return 0

if __name__ == "__main__":
    sys.exit(main())
