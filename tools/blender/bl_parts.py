#!/usr/bin/env python3
"""
Procedural parts for the 3D robot model (runs inside Blender only).

Parts with no login-free model are built here at their real size. Each builder
returns a PartResult whose root sits at the centre of the part's bounding-box
bottom face (the layout convention) and whose anchors are named exactly like the
part's pins in netlist.COMPONENTS. Anchor position = mating face of the pin
(header housing face, terminal tab tip, breadboard hole surface); direction =
the way a connector or wire leaves the pin.
"""

from typing import Dict, List

import bpy

from tools.blender.bl_common import (
    Anchor,
    PartResult,
    box,
    box_span,
    cylinder,
    empty,
    header_pins,
    material,
    merged_boxes,
    sphere,
    text_label,
    tube,
)
from tools.blender.layout import (
    CHASSIS_BOTTOM_Z_MM,
    CHASSIS_MM,
    CHASSIS_SLOT_SIZES_MM,
    CHASSIS_SLOTS_MM,
    FLOOR_Z_MM,
    WHEEL_DIAMETER_MM,
    WHEEL_NOTCH_ABS_Y_MM,
    WHEEL_NOTCH_X_MM,
    WHEEL_WIDTH_MM,
    Vec3,
)
from tools.blender.netlist import RAIL_HOLES

PCB_MM = 1.6
PITCH = 2.54
PCB_BLUE = "#1d4ed8"


def _pcb_blue() -> "bpy.types.Material":
    return material("pcb_blue", PCB_BLUE, roughness=0.45)


def _black() -> "bpy.types.Material":
    return material("plastic_black", "#111827", roughness=0.55)


def _tin() -> "bpy.types.Material":
    return material("tin", "#cbd5e1", roughness=0.3)


def make_chassis() -> PartResult:
    """Clear acrylic 2WD plate 210×150×3 with wheel notches, cable slots and M3 holes."""
    root = empty("chassis")
    length, width, thick = CHASSIS_MM
    acrylic = material("acrylic", "#e0f2fe", roughness=0.05, transmission=1.0)
    plate = box("chassis_plate", (length, width, thick), (0.0, 0.0, thick / 2), acrylic, root)

    cut: List = []
    notch_len = WHEEL_NOTCH_X_MM[1] - WHEEL_NOTCH_X_MM[0]
    notch_w = WHEEL_NOTCH_ABS_Y_MM[1] - WHEEL_NOTCH_ABS_Y_MM[0] + 2.0
    for sign in (1.0, -1.0):
        cy = sign * (WHEEL_NOTCH_ABS_Y_MM[0] + notch_w / 2)
        cut.append(((notch_len, notch_w, thick + 2), (0.0, cy, thick / 2)))
    for name, (sx, sy, _sz) in CHASSIS_SLOTS_MM.items():
        lx, ly = CHASSIS_SLOT_SIZES_MM[name]
        cut.append(((lx, ly, thick + 2), (sx, sy, thick / 2)))
    cutter = merged_boxes("chassis_cutter", cut, None, root)
    holes: Dict[str, Vec3] = {
        f"m3_{i}": (x, y, thick / 2)
        for i, (x, y) in enumerate(
            [(-98.0, 68.0), (-98.0, -68.0), (98.0, 68.0), (98.0, -68.0), (-90.0, 0.0), (50.0, 0.0), (-20.0, 45.0)]
        )
    }
    hole_objs = [cylinder(n, 1.6, thick + 2, c, "Z", None, root, segments=16) for n, c in holes.items()]
    for obj in (cutter, *hole_objs):
        obj.hide_render = True
        obj.display_type = "WIRE"
        mod = plate.modifiers.new(f"cut_{obj.name}", "BOOLEAN")
        mod.operation = "DIFFERENCE"
        mod.object = obj
    return PartResult(root, {})


def make_wheel(name: str) -> PartResult:
    """Yellow TT wheel with black tyre, Ø65×26 mm, axis along Y."""
    root = empty(name)
    r = WHEEL_DIAMETER_MM / 2
    center = (0.0, 0.0, r)
    tube(f"{name}_tyre", r, r - 6.5, WHEEL_WIDTH_MM, center, "Y", material("rubber", "#1f2937", roughness=0.85), root)
    yellow = material("tt_yellow", "#facc15", roughness=0.5)
    cylinder(f"{name}_rim", r - 6.4, WHEEL_WIDTH_MM - 4.0, center, "Y", yellow, root, segments=48)
    cylinder(f"{name}_hub", 5.0, WHEEL_WIDTH_MM, center, "Y", material("tt_yellow_dark", "#ca8a04"), root)
    return PartResult(root, {})


def make_fc51(name: str) -> PartResult:
    """FC-51 IR obstacle sensor 14×32×7: LEDs at +Y, 3-pin header OUT,GND,VCC pointing −Y."""
    root = empty(name)
    box_span(f"{name}_pcb", (-7.0, -11.0, 0.0), (7.0, 16.0, PCB_MM), _pcb_blue(), root)
    led_z = PCB_MM + 2.5
    clear = material("ir_led_clear", "#e2e8f0", roughness=0.05, transmission=0.9)
    cylinder(f"{name}_ir_led", 2.5, 6.0, (-3.0, 13.0, led_z), "Y", clear, root)
    cylinder(f"{name}_photodiode", 2.5, 6.0, (3.0, 13.0, led_z), "Y", _black(), root)
    pot_blue = material("trimpot_blue", "#2563eb", roughness=0.4)
    box(f"{name}_trimpot", (6.5, 6.5, 4.5), (0.0, 3.5, PCB_MM + 2.25), pot_blue, root)
    cylinder(f"{name}_trimpot_screw", 1.6, 0.9, (0.0, 3.5, PCB_MM + 4.95), "Z", _tin(), root)
    box(f"{name}_ic", (4.0, 3.0, 1.5), (0.0, -2.0, PCB_MM + 0.75), _black(), root)
    # Right-angle header at the back edge: pins leave the housing towards −Y.
    names = ["OUT", "GND", "VCC"]
    origin = (-PITCH, -8.46, PCB_MM + PITCH / 2)
    faces = header_pins(f"{name}_hdr", 3, origin, "X", (0.0, -1.0, 0.0), root, pin_length_mm=5.0)
    for pin, face in zip(names, faces):
        text_label(f"{name}_lbl_{pin}", pin, (face[0], -6.0, PCB_MM + 0.01), 1.1, root, (0.0, 0.0, 90.0))
    anchors: Dict[str, Anchor] = {pin: (face, (0.0, -1.0, 0.0)) for pin, face in zip(names, faces)}
    return PartResult(root, anchors)


def make_lm393(name: str) -> PartResult:
    """LM393 (FC-03) slot encoder 32×14×12, mounted upside down under the chassis.

    PCB on top, optical fork (5 mm gap across Y) hanging down at −X, 4-pin header
    VCC,GND,D0,A0 at +X with pins pointing +X.
    """
    root = empty(name)
    board_z = 12.0 - PCB_MM
    box_span(f"{name}_pcb", (-16.0, -7.0, board_z), (16.0, 7.0, 12.0), _pcb_blue(), root)
    fork = material("fork_black", "#0f172a", roughness=0.6)
    for side, (y0, y1) in (("a", (2.5, 4.3)), ("b", (-4.3, -2.5))):
        box_span(f"{name}_prong_{side}", (-16.0, y0, 0.0), (-10.0, y1, board_z), fork, root)
    box_span(f"{name}_fork_bridge", (-16.0, -4.3, 8.6), (-10.0, 4.3, board_z), fork, root)
    box(f"{name}_ic", (5.0, 4.0, 1.4), (-3.0, 0.0, board_z - 0.7), _black(), root)
    names = ["VCC", "GND", "D0", "A0"]
    hdr_z = board_z - PITCH / 2
    origin = (8.92, 3.81, hdr_z)
    # Pins are spaced along −Y (VCC at +Y), so build from the A0 end along +Y and reverse.
    faces = header_pins(f"{name}_hdr", 4, (origin[0], -3.81, hdr_z), "Y", (1.0, 0.0, 0.0), root, pin_length_mm=4.5)
    faces = list(reversed(faces))
    for pin, face in zip(names, faces):
        text_label(f"{name}_lbl_{pin}", pin, (5.0, face[1], board_z - 0.01), 1.0, root, (180.0, 0.0, 0.0))
    anchors: Dict[str, Anchor] = {pin: (face, (1.0, 0.0, 0.0)) for pin, face in zip(names, faces)}
    return PartResult(root, anchors)


def make_switch() -> PartResult:
    """Rocker switch 15×21×18 on the rear edge: rocker faces −X, solder tabs IN/OUT point +X."""
    root = empty("switch")
    box_span("switch_bezel", (-5.5, -10.5, 0.0), (-3.5, 10.5, 18.0), _black(), root)
    box_span("switch_rocker", (-7.5, -7.0, 3.0), (-5.5, 7.0, 15.0), material("rocker_red", "#dc2626", 0.4), root)
    box_span("switch_body", (-3.5, -9.5, 2.0), (3.5, 9.5, 16.0), _black(), root)
    brass = material("brass", "#b7791f", roughness=0.3)
    anchors: Dict[str, Anchor] = {}
    for pin, y in (("IN", -4.0), ("OUT", 4.0)):
        box_span(f"switch_tab_{pin}", (3.5, y - 0.4, 7.0), (7.5, y + 0.4, 11.0), brass, root)
        text_label(f"switch_lbl_{pin}", pin, (0.0, y, 16.01), 1.6, root, (0.0, 0.0, 90.0))
        anchors[pin] = ((7.5, y, 9.0), (1.0, 0.0, 0.0))
    return PartResult(root, anchors)


def make_buzzer() -> PartResult:
    """Active 5 V buzzer Ø12×9.5 with its legs bent out towards +X (+ at +Y)."""
    root = empty("buzzer")
    cylinder("buzzer_body", 6.0, 9.5, (-2.0, 0.0, 4.75), "Z", _black(), root)
    cylinder("buzzer_port", 1.0, 0.2, (-2.0, 0.0, 9.45), "Z", material("hole_dark", "#020617"), root)
    text_label("buzzer_lbl_plus", "+", (-2.0, 3.6, 9.51), 2.5, root)
    anchors: Dict[str, Anchor] = {}
    for pin, y in (("+", 3.8), ("-", -3.8)):
        box_span(f"buzzer_leg_{pin}", (1.0, y - 0.3, 1.2), (8.0, y + 0.3, 1.8), _tin(), root)
        anchors[pin] = ((3.0, y, 1.5), (1.0, 0.0, 0.0))
    return PartResult(root, anchors)


def make_breadboard() -> PartResult:
    """Half-size 400-point breadboard 82.5×54.5×8.5; the top rail pair (+Y edge) is the power hub."""
    root = empty("bb")
    height = 8.5
    box_span("bb_body", (-41.25, -27.25, 0.0), (41.25, 27.25, height), material("bb_white", "#f1f5f9", 0.6), root)
    hole = material("bb_hole", "#334155", roughness=0.9)
    hole_size = (1.0, 1.0, 0.2)
    holes = []
    rail_x = [-30.48 + PITCH * i for i in range(RAIL_HOLES)]
    rail_rows = {"P5V": 24.0, "GND": 24.0 - PITCH, "bP5V": -24.0 + PITCH, "bGND": -24.0}
    for y in rail_rows.values():
        holes += [(hole_size, (x, y, height - 0.09)) for x in rail_x]
    for col in range(30):
        x = -36.83 + PITCH * col
        for row in range(5):
            for sign in (1.0, -1.0):
                holes.append((hole_size, (x, sign * (3.81 + PITCH * row), height - 0.09)))
    merged_boxes("bb_holes", holes, hole, root)
    red = material("rail_red", "#dc2626", 0.5)
    blue = material("rail_blue", "#2563eb", 0.5)
    box("bb_line_p5v", (64.0, 0.5, 0.05), (0.0, 25.9, height + 0.02), red, root)
    box("bb_line_gnd", (64.0, 0.5, 0.05), (0.0, 24.0 - PITCH - 1.9, height + 0.02), blue, root)
    box("bb_line_p5v_b", (64.0, 0.5, 0.05), (0.0, -24.0 + PITCH + 1.9, height + 0.02), red, root)
    box("bb_line_gnd_b", (64.0, 0.5, 0.05), (0.0, -25.9, height + 0.02), blue, root)
    box("bb_channel", (78.0, 2.5, 0.3), (0.0, 0.0, height - 0.15), hole, root)
    text_label("bb_lbl_p5v", "+5V", (-36.5, 24.0, height + 0.01), 2.0, root, hex_color="#dc2626")
    text_label("bb_lbl_gnd", "GND", (-36.5, 24.0 - PITCH, height + 0.01), 2.0, root, hex_color="#2563eb")
    anchors: Dict[str, Anchor] = {}
    for i, x in enumerate(rail_x, start=1):
        anchors[f"P5V_{i}"] = ((x, rail_rows["P5V"], height), (0.0, 0.0, 1.0))
        anchors[f"GND_{i}"] = ((x, rail_rows["GND"], height), (0.0, 0.0, 1.0))
    return PartResult(root, anchors)


def make_caster() -> PartResult:
    """Ball caster on a brass standoff, from the chassis underside down to the floor."""
    root = empty("caster")
    height = CHASSIS_BOTTOM_Z_MM - FLOOR_Z_MM
    steel = material("steel", "#94a3b8", roughness=0.2)
    box("caster_plate", (25.0, 25.0, 2.0), (0.0, 0.0, height - 1.0), steel, root)
    cylinder("caster_standoff", 3.0, height - 2.0 - 20.0, (0.0, 0.0, (height - 2.0 + 20.0) / 2), "Z",
             material("brass", "#b7791f", 0.3), root)
    cylinder("caster_housing", 10.0, 12.0, (0.0, 0.0, 14.0), "Z", steel, root)
    sphere("caster_ball", 8.0, (0.0, 0.0, 8.0), material("chrome", "#e2e8f0", roughness=0.08), root)
    return PartResult(root, {})


def make_cell(name: str) -> PartResult:
    """18650 Li-ion cell Ø18×65 along Y, coloured wrap, + nub at +Y."""
    root = empty(name)
    cylinder(f"{name}_wrap", 9.0, 64.0, (0.0, -0.5, 9.0), "Y", material("cell_wrap", "#0d9488", 0.35), root)
    cylinder(f"{name}_nub", 3.5, 1.0, (0.0, 32.0, 9.0), "Y", _tin(), root)
    return PartResult(root, {})


def make_cap_100nf(name: str) -> PartResult:
    """Blue ceramic disc capacitor '104' (100 nF), 5×2.5×4, disc face along Y."""
    root = empty(name)
    disc = cylinder(f"{name}_disc", 2.5, 2.5, (0.0, 0.0, 2.0), "Y", material("cap_blue", "#1e40af", 0.4), root)
    disc.scale = (1.0, 1.0, 0.8)
    text_label(f"{name}_lbl", "104", (0.0, -1.26, 2.0), 1.0, root, (90.0, 0.0, 0.0))
    return PartResult(root, {})
