#!/usr/bin/env python3
"""
Trajectory Plotter for Autonomous Mobile Robot Navigation (TC253421 Polsri)
Parses serial CSV telemetry (t_ms,state,x,y,theta,jarak,iWp) and renders
publication-quality SVG / PNG trajectory diagrams over the 480x80cm arena.
"""

import argparse
import csv
import sys
from typing import Any, Dict, List, Optional, Tuple

# Arena definitions in cm
CORRIDOR_X_MIN = 0.0
CORRIDOR_X_MAX = 480.0
CORRIDOR_Y_MIN = -40.0
CORRIDOR_Y_MAX = 40.0

ROOMS = {
    1: {"name": "Room 1", "door_x": 180.0, "side": +1, "door_wp": (180.0, 0.0), "center_wp": (180.0, 70.0)},
    2: {"name": "Room 2", "door_x": 295.0, "side": +1, "door_wp": (295.0, 0.0), "center_wp": (295.0, 70.0)},
    3: {"name": "Room 3", "door_x": 235.0, "side": -1, "door_wp": (235.0, 0.0), "center_wp": (235.0, -70.0)},
    4: {"name": "Room 4", "door_x": 355.0, "side": -1, "door_wp": (355.0, 0.0), "center_wp": (355.0, -70.0)},
}

OBSTACLES = [
    (90.0, 0.0, 20.0, 20.0) # Standard 20x20 cm obstacle box
]

def parse_telemetry(csv_filepath: str) -> List[Dict[str, Any]]:
    points = []
    with open(csv_filepath, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#") or row[0].strip() == "t_ms":
                continue
            try:
                t_ms = float(row[0])
                state = row[1].strip()
                x = float(row[2])
                y = float(row[3])
                theta = float(row[4])
                jarak = float(row[5])
                iWp = int(row[6])
                points.append({
                    "t_ms": t_ms, "state": state, "x": x, "y": y,
                    "theta": theta, "jarak": jarak, "iWp": iWp
                })
            except (ValueError, IndexError):
                continue
    return points

def render_svg(
    points: List[Dict[str, Any]],
    output_svg: str,
    obstacles: Optional[List[Tuple[float, float, float, float]]] = None,
) -> None:
    if obstacles is None:
        obstacles = OBSTACLES

    # SVG Canvas dimensions
    width = 1100
    height = 550
    margin_left = 60
    margin_top = 40

    # Coordinate mapping:
    # X: [0, 480] cm -> [margin_left, margin_left + 960] (scale = 2.0 px/cm)
    # Y: [-100, +100] cm -> [height/2 + 200, height/2 - 200]
    scale_x = 2.0
    scale_y = 2.0
    center_y = height / 2.0

    def to_svg_x(x_cm):
        return margin_left + x_cm * scale_x

    def to_svg_y(y_cm):
        return center_y - y_cm * scale_y

    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}">')
    svg.append('<style>')
    svg.append('  .bg { fill: #f8fafc; }')
    svg.append('  .corridor { fill: #ffffff; stroke: #334155; stroke-width: 2.5; stroke-dasharray: none; }')
    svg.append('  .centerline { stroke: #94a3b8; stroke-width: 1.2; stroke-dasharray: 6,4; }')
    svg.append('  .room { fill: #f1f5f9; stroke: #475569; stroke-width: 2; }')
    svg.append('  .target-circle { fill: #bbf7d0; stroke: #16a34a; stroke-width: 2; }')
    svg.append('  .obstacle { fill: #fed7aa; stroke: #ea580c; stroke-width: 2; }')
    svg.append('  .traj { fill: none; stroke: #2563eb; stroke-width: 2.5; stroke-linecap: round; stroke-linejoin: round; }')
    svg.append('  .marker-start { fill: #10b981; stroke: #047857; stroke-width: 2; }')
    svg.append('  .marker-end { fill: #ef4444; stroke: #b91c1c; stroke-width: 2; }')
    svg.append('  .text-label { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; font-size: 12px; fill: #1e293b; font-weight: 600; text-anchor: middle; }')
    svg.append('  .title { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; font-size: 16px; fill: #0f172a; font-weight: bold; }')
    svg.append('</style>')

    # Background
    svg.append(f'<rect width="{width}" height="{height}" class="bg" />')

    # Title & Legend
    svg.append('<text x="60" y="25" class="title">Visualisasi Lintasan Navigasi Robot Otonom (Polsri TC253421)</text>')

    # Corridor Rect: [0, 480] x [-40, 40]
    c_x1 = to_svg_x(0)
    c_y1 = to_svg_y(40)
    c_w = 480 * scale_x
    c_h = 80 * scale_y
    svg.append(f'<rect x="{c_x1}" y="{c_y1}" width="{c_w}" height="{c_h}" class="corridor" />')
    svg.append(f'<line x1="{to_svg_x(0)}" y1="{to_svg_y(0)}" x2="{to_svg_x(480)}" y2="{to_svg_y(0)}" class="centerline" />')

    # Rooms 1-4
    for r_num, r in ROOMS.items():
        dx = r["door_x"]
        if r["side"] > 0: # Right side (y: 40 to 95, x: dx-25 to dx+25)
            rx = to_svg_x(dx - 25)
            ry = to_svg_y(95)
            rw = 50 * scale_x
            rh = 55 * scale_y
            svg.append(f'<rect x="{rx}" y="{ry}" width="{rw}" height="{rh}" class="room" />')
            # Door opening (cut wall)
            svg.append(f'<line x1="{to_svg_x(dx-15)}" y1="{to_svg_y(40)}" x2="{to_svg_x(dx+15)}" y2="{to_svg_y(40)}" stroke="#ffffff" stroke-width="4" />')
        else: # Left side (y: -40 to -95, x: dx-25 to dx+25)
            rx = to_svg_x(dx - 25)
            ry = to_svg_y(-40)
            rw = 50 * scale_x
            rh = 55 * scale_y
            svg.append(f'<rect x="{rx}" y="{ry}" width="{rw}" height="{rh}" class="room" />')
            svg.append(f'<line x1="{to_svg_x(dx-15)}" y1="{to_svg_y(-40)}" x2="{to_svg_x(dx+15)}" y2="{to_svg_y(-40)}" stroke="#ffffff" stroke-width="4" />')

        # Target Circle (r = 5 cm = 10 cm diameter) at center_wp
        cx = to_svg_x(r["center_wp"][0])
        cy = to_svg_y(r["center_wp"][1])
        r_name = r["name"]
        offset_y = 18 if r["side"] > 0 else -10
        svg.append(f'<circle cx="{cx}" cy="{cy}" r="{5 * scale_x}" class="target-circle" />')
        svg.append(f'<text x="{cx}" y="{cy + offset_y}" class="text-label">{r_name}</text>')

    # Obstacle boxes
    for (ox, oy, ow, oh) in obstacles:
        bx = to_svg_x(ox - ow/2.0)
        by = to_svg_y(oy + oh/2.0)
        bw = ow * scale_x
        bh = oh * scale_y
        svg.append(f'<rect x="{bx}" y="{by}" width="{bw}" height="{bh}" class="obstacle" />')
        svg.append(f'<text x="{to_svg_x(ox)}" y="{to_svg_y(oy)+4}" class="text-label" font-size="10">Rintangan</text>')

    # Robot Trajectory Path & Events
    if points:
        path_d = []
        pindai_events = []
        calib_end_pt = None

        prev_state = None
        for i, pt in enumerate(points):
            sx = to_svg_x(pt["x"])
            sy = to_svg_y(pt["y"])
            cmd = "M" if i == 0 else "L"
            path_d.append(f"{cmd} {sx:.1f} {sy:.1f}")

            # Detect state transitions
            curr_state = pt["state"]
            if prev_state == "KALIBRASI" and curr_state != "KALIBRASI" and calib_end_pt is None:
                calib_end_pt = (sx, sy)
            elif prev_state == "MAJU" and curr_state == "PINDAI":
                pindai_events.append((sx, sy, pt["x"], pt["y"]))
            prev_state = curr_state

        svg.append(f'<path d="{" ".join(path_d)}" class="traj" />')

        # Start Marker (0,0)
        start_x = to_svg_x(points[0]["x"])
        start_y = to_svg_y(points[0]["y"])
        svg.append(f'<circle cx="{start_x}" cy="{start_y}" r="6" class="marker-start" />')
        svg.append(f'<text x="{start_x}" y="{start_y - 12}" class="text-label">START (0,0)</text>')

        # Calibration End Marker
        if calib_end_pt:
            cx, cy = calib_end_pt
            svg.append(f'<circle cx="{cx}" cy="{cy}" r="4" fill="#6366f1" stroke="#4338ca" stroke-width="1.5" />')
            svg.append(f'<text x="{cx}" y="{cy + 15}" class="text-label" font-size="10" fill="#4338ca">Kalibrasi Selesai</text>')

        # Obstacle Detection Markers
        for (ox, oy, rx, ry) in pindai_events:
            svg.append(f'<circle cx="{ox}" cy="{oy}" r="5" fill="#f59e0b" stroke="#b45309" stroke-width="2" />')
            svg.append(f'<text x="{ox}" y="{oy - 12}" class="text-label" font-size="10" fill="#b45309">Deteksi Rintangan ({rx:.0f},{ry:.0f})</text>')

        # End Marker
        end_x = to_svg_x(points[-1]["x"])
        end_y = to_svg_y(points[-1]["y"])
        svg.append(f'<circle cx="{end_x}" cy="{end_y}" r="7" class="marker-end" />')
        svg.append(f'<text x="{end_x}" y="{end_y - 14}" class="text-label">TIBA ({points[-1]["x"]:.1f}, {points[-1]["y"]:.1f})</text>')

    svg.append('</svg>')

    with open(output_svg, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))
    print(f"Generated trajectory SVG: {output_svg}")

def parse_obstacle_args(obs_args: Optional[List[str]]) -> Optional[List[Tuple[float, float, float, float]]]:
    """Parse CLI --obs arguments ('x,y,w,h' or 'none') into obstacle tuples."""
    if obs_args is None:
        return None
    parsed: List[Tuple[float, float, float, float]] = []
    for item in obs_args:
        if item.strip().lower() == "none":
            continue
        parts = [float(p.strip()) for p in item.split(",")]
        if len(parts) == 4:
            parsed.append((parts[0], parts[1], parts[2], parts[3]))
    return parsed


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot Robot Navigation Telemetry Path")
    parser.add_argument("csv_file", help="Path to telemetry CSV log")
    parser.add_argument("-o", "--output", default="tools/trajectory_plot.svg", help="Output SVG filepath")
    parser.add_argument("--obs", action="append", default=None, help="Rintangan format 'x,y,w,h' atau 'none'")
    args = parser.parse_args()

    points = parse_telemetry(args.csv_file)
    print(f"Parsed {len(points)} telemetry data points from {args.csv_file}")
    obstacles = parse_obstacle_args(args.obs)
    render_svg(points, args.output, obstacles=obstacles)

if __name__ == "__main__":
    main()
