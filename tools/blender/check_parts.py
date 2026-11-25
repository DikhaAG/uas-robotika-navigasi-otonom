#!/usr/bin/env python3
"""
Headless check of every 3D part against the layout and the netlist.

Run:  "$BLENDER" -b --factory-startup -P tools/blender/check_parts.py [-- --render DIR]

For each part it builds the part alone in an empty scene and checks that
  * its bounding box matches layout.Placement.footprint_mm within ±1 mm,
  * its anchor names equal the pins of its netlist component exactly,
  * every anchor lies inside the part's bounding box.
Prints "PARTS OK" or every error and exits 1. With --render DIR it also renders
a labelled close-up of each part to DIR/<part>.png for visual review.
"""

import argparse
import sys
from pathlib import Path
from typing import Callable, Dict, List, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.blender import bl_common, bl_parts  # noqa: E402
from tools.blender.bl_common import PartResult  # noqa: E402
from tools.blender.layout import placements  # noqa: E402
from tools.blender.netlist import COMPONENTS  # noqa: E402

TOLERANCE_MM = 1.0
ANCHOR_SLACK_MM = 0.5

# part id in layout -> builder
PROCEDURAL: Dict[str, Callable[[], PartResult]] = {
    "chassis": bl_parts.make_chassis,
    "wheel_l": lambda: bl_parts.make_wheel("wheel_l"),
    "ir_l": lambda: bl_parts.make_fc51("ir_l"),
    "enc_l": lambda: bl_parts.make_lm393("enc_l"),
    "switch": bl_parts.make_switch,
    "buzzer": bl_parts.make_buzzer,
    "bb": bl_parts.make_breadboard,
    "caster": bl_parts.make_caster,
    "cell_1": lambda: bl_parts.make_cell("cell_1"),
    "cap_l": lambda: bl_parts.make_cap_100nf("cap_l"),
}


def check_part(part_id: str, result: PartResult) -> List[str]:
    """Bounding-box, anchor-name and anchor-position checks for one freshly built part."""
    errors: List[str] = []
    expected = placements()[part_id].footprint_mm
    errors.extend(f"{part_id}: {e}" for e in bl_common.check_dimensions(result, expected, TOLERANCE_MM))

    lo, hi = bl_common.local_bbox_mm(result.obj)
    want = set(COMPONENTS.get(part_id, {}))
    have = set(result.anchors)
    if want != have:
        errors.append(f"{part_id}: anchors missing {sorted(want - have)} extra {sorted(have - want)}")
    for pin, (pos, _direction) in result.anchors.items():
        inside = all(lo[i] - ANCHOR_SLACK_MM <= pos[i] <= hi[i] + ANCHOR_SLACK_MM for i in range(3))
        if not inside:
            errors.append(f"{part_id}.{pin}: anchor {pos} outside bbox {lo}..{hi}")
    return errors


def run(builders: Dict[str, Callable[[], PartResult]], render_dir: Path = None) -> List[str]:
    """Builds and checks each part in isolation; optionally renders a labelled close-up."""
    errors: List[str] = []
    for part_id, build in builders.items():
        bl_common.reset_scene()
        result = build()
        part_errors = check_part(part_id, result)
        errors.extend(part_errors)
        status = "ok" if not part_errors else "FAIL"
        lo, hi = bl_common.local_bbox_mm(result.obj)
        size: Tuple[float, ...] = tuple(round(hi[i] - lo[i], 2) for i in range(3))
        print(f"  {status:4s} {part_id:10s} bbox {size} anchors {len(result.anchors)}")
        if render_dir is not None:
            bl_common.render_closeup(result, render_dir / f"{part_id}.png")
    return errors


def main() -> int:
    """Entry point when run inside Blender."""
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--render", type=Path, default=None, help="directory for close-up PNGs")
    args = parser.parse_args(argv)

    errors = run(PROCEDURAL, args.render)
    if errors:
        for e in errors:
            print(f"ERROR: {e}")
        return 1
    print("PARTS OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
