#!/usr/bin/env python3
"""
Measuring aid for pin anchors of a downloaded model.

Run:  "$BLENDER" -b --factory-startup -P tools/blender/measure_model.py -- <key> [--view top|bottom|front|back|left|right]

Imports the model normalised by assets/3d/anchors.json (or raw, if the key has no
entry yet), prints the overall bbox, the suggested offset_mm that puts the origin
at the bbox bottom-centre, and every object's bbox and material; then renders an
orthographic 1600 px view to build/renders/measure/<key>_<view>.png with a 2.54 mm
grid (every 5th line red) and existing anchors marked by red Ø1 mm spheres.
Measure the first pin of a header there; the rest follow from the 2.54 mm pitch.
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402
from mathutils import Vector  # noqa: E402

from tools.blender import bl_common, bl_import  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
VIEWS: Dict[str, Vector] = {
    "top": Vector((0.0, 0.0, 1.0)),
    "bottom": Vector((0.0, 0.0, -1.0)),
    "front": Vector((1.0, 0.0, 0.0)),
    "back": Vector((-1.0, 0.0, 0.0)),
    "left": Vector((0.0, 1.0, 0.0)),
    "right": Vector((0.0, -1.0, 0.0)),
}


def describe(result: bl_common.PartResult) -> List[str]:
    """Human-readable bbox lines for the part and each of its meshes (mm, part-local frame)."""
    lo, hi = bl_common.local_bbox_mm(result.obj)
    size = [round(hi[i] - lo[i], 3) for i in range(3)]
    offset = [round(-(lo[0] + hi[0]) / 2, 3), round(-(lo[1] + hi[1]) / 2, 3), round(-lo[2], 3)]
    lines = [f"BBOX size {size} min {[round(v, 3) for v in lo]} max {[round(v, 3) for v in hi]}",
             f"SUGGESTED extra offset_mm {offset}"]
    inv = np.array(result.obj.matrix_world.inverted())
    for obj in sorted(result.obj.children_recursive, key=lambda o: o.name):
        if obj.type != "MESH":
            continue
        pts = bl_common.mesh_world_vertices(obj) @ inv[:3, :3].T + inv[:3, 3]
        if not len(pts):
            continue
        pts = pts * 1000.0
        mats = [m.name for m in obj.data.materials if m is not None]
        lines.append(f"OBJ {obj.name:28s} min {np.round(pts.min(0), 2).tolist()} max {np.round(pts.max(0), 2).tolist()} {mats}")
    return lines


def grid(result: bl_common.PartResult, view: str) -> None:
    """2.54 mm grid lines just in front of the model, perpendicular to the view axis."""
    lo, hi = bl_common.local_bbox_mm(result.obj)
    axis = VIEWS[view]
    a, b = [i for i in range(3) if abs(axis[i]) < 0.5]
    depth = [i for i in range(3) if abs(axis[i]) > 0.5][0]
    level = (hi[depth] + 0.2) if axis[depth] > 0 else (lo[depth] - 0.2)
    dark = bl_common.material("grid_dark", "#475569", roughness=1.0)
    red = bl_common.material("grid_red", "#dc2626", roughness=1.0)
    for along, across in ((a, b), (b, a)):
        start = np.floor(lo[along] / 2.54) * 2.54
        k = 0
        x = start
        while x <= hi[along] + 2.54:
            size = [0.05, 0.05, 0.05]
            center = [0.0, 0.0, 0.0]
            size[along] = 0.08 if round(x / 2.54) % 5 else 0.16
            size[across] = hi[across] - lo[across] + 5.0
            center[along] = x
            center[across] = (hi[across] + lo[across]) / 2
            center[depth] = level
            mat = red if round(x / 2.54) % 5 == 0 else dark
            bl_common.box(f"grid_{along}_{k}", tuple(size), tuple(center), mat, result.obj)
            x += 2.54
            k += 1


def main() -> int:
    """Entry point when run inside Blender."""
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("key")
    parser.add_argument("--view", default="top", choices=sorted(VIEWS))
    args = parser.parse_args(argv)

    bl_common.reset_scene()
    result = bl_import.import_part(args.key, args.key)
    for line in describe(result):
        print(line)
    grid(result, args.view)
    red = bl_common.material("anchor_marker", "#ef4444", roughness=0.3)
    for pin, (pos, _d) in result.anchors.items():
        bl_common.sphere(f"anchor_{pin}", 0.5, pos, red, result.obj)
    lo, hi = bl_common.local_bbox_mm(result.obj)
    center = bl_common.mm(((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2))
    extent = max(hi[i] - lo[i] for i in range(3)) * 0.001 * 1.1
    bl_common.studio_lighting(center, extent)
    direction = VIEWS[args.view]
    cam = bl_common.aim_camera("measure_cam", center, direction, extent * 3, ortho_scale=extent)
    out = REPO_ROOT / "build" / "renders" / "measure" / f"{args.key}_{args.view}.png"
    bl_common.render_to(out, cam, 1600, 1600)
    print(f"RENDERED {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
