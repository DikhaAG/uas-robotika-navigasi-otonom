#!/usr/bin/env python3
"""
Builds the complete 3D robot (Arduino Uno R3 + L298N, 41 wires) in Blender.

Run (normally via `make blender_model`):
  "$BLENDER" -b --factory-startup --python-exit-code 1 -P tools/blender/build_robot.py -- \\
      --out build/robot_3d.blend --renders docs/hardware/renders [--no-render] [--closeups DIR]

Steps: empty metric scene → check model files → build and place every part from
layout.placements() → route and build the 41 netlist wires → check_scene() (wire
count, wire ends ≤ 0.5 mm from their pins, no wire through a part) → collections
Chassis / Electronics / Wires/<stage> → save .blend → renders (iso, top, bottom).
Any failed check prints every problem and exits 1.
"""

import argparse
import sys
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.blender.bl_import import missing_model_files  # noqa: E402  (no bpy work before this check)


def _missing_models_message() -> Optional[str]:
    missing = missing_model_files()
    if not missing:
        return None
    lines = ["ERROR: file model 3D belum ada — jalankan: make blender_fetch && make blender_convert", *[f"  - {m}" for m in missing]]
    return "\n".join(lines)


import bpy  # noqa: E402
from mathutils import Vector  # noqa: E402

from tools.blender import bl_common, bl_import, bl_parts  # noqa: E402
from tools.blender.bl_checks import check_scene  # noqa: E402
from tools.blender.bl_common import PartResult  # noqa: E402
from tools.blender.bl_wires import assign_lanes, make_wire, world_anchor  # noqa: E402
from tools.blender.layout import (  # noqa: E402
    CHASSIS_MID_Z_MM,
    CHASSIS_SLOT_SIZES_MM,
    CHASSIS_SLOTS_MM,
    DUPONT_STRAIGHT_MM,
    Vec3,
    placements,
)
from tools.blender.netlist import Wire, build_netlist, wire_name  # noqa: E402
from tools.blender.routing import route  # noqa: E402

SLOT_MARGIN_MM = 2.0
STAGE_NAMES = {1: "Tahap 1 - Daya", 2: "Tahap 2 - L298N & Motor", 3: "Tahap 3 - Encoder",
               4: "Tahap 4 - Sensor", 5: "Tahap 5 - START & Buzzer"}
CHASSIS_PARTS = {"chassis", "wheel_l", "wheel_r", "motor_l", "motor_r", "disk_l", "disk_r", "batt",
                 "cell_1", "cell_2", "caster", "cap_l", "cap_r"}
CLOSEUPS: Dict[str, Vec3] = {
    "uno": (-1.0, 0.7, 0.55), "l298n": (-1.0, -0.4, 1.2), "sonar": (-1.0, 0.5, 1.0), "mpu": (0.6, -1.0, 1.3),
    "bb": (-1.0, -0.6, 1.4), "ir_l": (0.5, -1.0, 1.0), "ir_r": (0.5, 1.0, 1.0),
    "enc_l": (1.0, -0.6, -0.8), "enc_r": (1.0, 0.6, -0.8),
}


def builders() -> Dict[str, Callable[[], PartResult]]:
    """part id → function that builds it at the local origin."""
    imported = {"uno": "uno", "l298n": "l298n", "sonar": "sonar", "mpu": "mpu", "btn": "btn",
                "motor_l": "motor_tt", "motor_r": "motor_tt", "batt": "batt_holder",
                "disk_l": "enc_disk", "disk_r": "enc_disk"}
    table: Dict[str, Callable[[], PartResult]] = {
        "chassis": bl_parts.make_chassis,
        "wheel_l": lambda: bl_parts.make_wheel("wheel_l"),
        "wheel_r": lambda: bl_parts.make_wheel("wheel_r"),
        "enc_l": lambda: bl_parts.make_lm393("enc_l"),
        "enc_r": lambda: bl_parts.make_lm393("enc_r"),
        "ir_l": lambda: bl_parts.make_fc51("ir_l"),
        "ir_r": lambda: bl_parts.make_fc51("ir_r"),
        "switch": bl_parts.make_switch,
        "buzzer": bl_parts.make_buzzer,
        "bb": bl_parts.make_breadboard,
        "caster": bl_parts.make_caster,
        "cell_1": lambda: bl_parts.make_cell("cell_1"),
        "cell_2": lambda: bl_parts.make_cell("cell_2"),
        "cap_l": lambda: bl_parts.make_cap_100nf("cap_l"),
        "cap_r": lambda: bl_parts.make_cap_100nf("cap_r"),
    }
    for part_id, key in imported.items():
        table[part_id] = lambda key=key, part_id=part_id: bl_import.import_part(key, part_id)
    return table


def build_parts() -> Dict[str, PartResult]:
    """Builds and places every part of the layout."""
    table = builders()
    parts: Dict[str, PartResult] = {}
    for part_id, placement in placements().items():
        result = table[part_id]()
        bl_common.place(result, placement)
        parts[part_id] = result
    return parts


def _offset(p: Vec3, d: Vec3, k: float) -> Vec3:
    return (p[0] + d[0] * k, p[1] + d[1] * k, p[2] + d[2] * k)


def slot_via(point: Vec3) -> Vec3:
    """Point inside the nearest chassis slot (kept SLOT_MARGIN_MM off its edges), at plate mid-height."""
    best: Optional[Tuple[float, Vec3]] = None
    for name, (cx, cy, _cz) in CHASSIS_SLOTS_MM.items():
        lx, ly = CHASSIS_SLOT_SIZES_MM[name]
        x = min(max(point[0], cx - lx / 2 + SLOT_MARGIN_MM), cx + lx / 2 - SLOT_MARGIN_MM)
        y = min(max(point[1], cy - ly / 2 + SLOT_MARGIN_MM), cy + ly / 2 - SLOT_MARGIN_MM)
        dist = (x - point[0]) ** 2 + (y - point[1]) ** 2
        if best is None or dist < best[0]:
            best = (dist, (x, y, CHASSIS_MID_Z_MM))
    assert best is not None
    return best[1]


def build_wires(parts: Dict[str, PartResult]) -> Dict[str, "bpy.types.Object"]:
    """Routes all netlist wires (bottom parts thread a chassis slot) and builds them."""
    zones = {k: v.zone for k, v in placements().items()}
    wires: List[Wire] = build_netlist()
    plans: Dict[str, Tuple[Wire, Vec3, Vec3, Vec3, Vec3, List[Vec3]]] = {}
    spans: Dict[str, Tuple[Vec3, Vec3]] = {}
    for w in wires:
        a, a_dir = world_anchor(parts[w.src.component], w.src.pin)
        b, b_dir = world_anchor(parts[w.dst.component], w.dst.pin)
        vias: List[Vec3] = []
        if zones[w.src.component] == "bottom":
            vias.append(slot_via(_offset(a, a_dir, DUPONT_STRAIGHT_MM)))
        start = vias[-1] if vias else _offset(a, a_dir, DUPONT_STRAIGHT_MM)
        spans[wire_name(w)] = (start, _offset(b, b_dir, DUPONT_STRAIGHT_MM))
        plans[wire_name(w)] = (w, a, a_dir, b, b_dir, vias)
    lanes = assign_lanes(wires, spans)
    objs: Dict[str, "bpy.types.Object"] = {}
    for name, (w, a, a_dir, b, b_dir, vias) in plans.items():
        obj = make_wire(w, route(a, a_dir, b, b_dir, lanes[name], vias))
        obj["vias"] = [list(v) for v in vias]
        obj["lane"] = lanes[name]
        objs[name] = obj
    return objs


def _move_tree(root: "bpy.types.Object", collection: "bpy.types.Collection") -> None:
    for obj in [root, *root.children_recursive]:
        for c in list(obj.users_collection):
            c.objects.unlink(obj)
        collection.objects.link(obj)


def organise(parts: Dict[str, PartResult], wires: Dict[str, "bpy.types.Object"]) -> None:
    """Collections Chassis / Electronics / Wires/<stage>, so each stage can be hidden on its own."""
    scene_root = bpy.context.scene.collection

    def new(name: str, parent: "bpy.types.Collection") -> "bpy.types.Collection":
        col = bpy.data.collections.new(name)
        parent.children.link(col)
        return col

    chassis, electronics, wires_col = new("Chassis", scene_root), new("Electronics", scene_root), new("Wires", scene_root)
    stages = {stage: new(title, wires_col) for stage, title in STAGE_NAMES.items()}
    for part_id, part in parts.items():
        _move_tree(part.obj, chassis if part_id in CHASSIS_PARTS else electronics)
    for obj in wires.values():
        _move_tree(obj, stages[int(obj["stage"])])


def ensure_studio() -> Vector:
    """White studio world and three sun lights, set up once for every render; returns the scene centre."""
    center = Vector((0.0, 0.0, 0.012))
    if bpy.context.scene.world is None:
        bl_common.studio_lighting(center, 0.25)
        bpy.context.scene.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 1.0
        bpy.context.scene.view_settings.view_transform = "Standard"
    return center


def render_views(out_dir: Path) -> None:
    """Iso (front-left-above), top and bottom orthographic renders, 1920×1200, white background."""
    center = ensure_studio()
    views = {
        "robot_iso": (Vector((1.0, 0.85, 0.75)), None),
        "robot_top": (Vector((0.0, 0.0, 1.0)), 0.27),
        "robot_bottom": (Vector((0.0, 0.0, -1.0)), 0.27),
    }
    for name, (direction, ortho) in views.items():
        cam = bl_common.aim_camera(f"cam_{name}", center, direction, 0.55 if ortho is None else 0.75, ortho_scale=ortho)
        if ortho is None:
            cam.data.lens = 50.0
        bl_common.render_to(out_dir / f"{name}.png", cam, 1920, 1200)


def render_closeups(parts: Dict[str, PartResult], out_dir: Path) -> None:
    """One close-up per header (Uno, L298N, each sensor, breadboard rails)."""
    ensure_studio()
    for part_id, direction in CLOSEUPS.items():
        lo, hi = bl_common.local_bbox_mm(parts[part_id].obj)
        center_local = Vector(((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)) * 0.001
        center = parts[part_id].obj.matrix_world @ center_local
        radius = max(Vector(hi) - Vector(lo)) * 0.0005 + 0.01
        cam = bl_common.aim_camera(f"close_{part_id}", center, Vector(direction), radius * 4.0)
        bl_common.render_to(out_dir / f"{part_id}.png", cam, 1200, 900)


def main() -> int:
    """Entry point when run inside Blender; returns the process exit status."""
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--renders", type=Path, default=None)
    parser.add_argument("--no-render", action="store_true")
    parser.add_argument("--closeups", type=Path, default=None)
    args = parser.parse_args(argv)

    message = _missing_models_message()
    if message:
        print(message)
        return 1

    bl_common.reset_scene()
    parts = build_parts()
    wires = build_wires(parts)
    errors = check_scene(parts, wires)
    if errors:
        for e in errors:
            print(f"ERROR: {e}")
        print(f"ROBOT MODEL FAILED: {len(errors)} problem(s)")
        return 1

    organise(parts, wires)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(args.out.resolve()))
    if not args.no_render and args.renders is not None:
        render_views(args.renders)
    if args.closeups is not None:
        render_closeups(parts, args.closeups)
    print(f"ROBOT MODEL OK: {len(wires)} wires, {len(parts)} parts")
    return 0


if __name__ == "__main__":
    sys.exit(main())
