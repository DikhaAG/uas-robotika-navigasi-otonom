#!/usr/bin/env python3
"""
Chassis layout for the Blender 3D model: where every part sits on the robot.

Robot frame: +X forward, +Y left, +Z up, origin at the wheel-axle centre, units mm.
Every part's local origin is the centre of the bottom face of its local bounding
box (footprint_mm = local size along X, Y, Z). The part is rotated about that
origin by rotation_deg (Euler XYZ) and then moved to position_mm.

Zones follow the design spec §9: motors, wheels, battery and encoders under the
chassis; Uno, L298N, MPU6050 and breadboard on top; HC-SR04 at the front; FC-51
on the left/right edges. Pure Python: no Blender dependency.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Literal, Tuple

Vec3 = Tuple[float, float, float]
Zone = Literal["top", "bottom", "front", "left", "right"]

CHASSIS_MM: Vec3 = (210.0, 150.0, 3.0)
WHEEL_DIAMETER_MM: float = 65.0  # Config.h DIAMETER_RODA_MM
WHEEL_WIDTH_MM: float = 26.0
TRACK_WIDTH_MM: float = 135.0  # Config.h JARAK_RODA_MM
DUPONT_STRAIGHT_MM: float = 16.0
ROUTE_CLEARANCE_MM: float = 10.0

FLOOR_Z_MM: float = -WHEEL_DIAMETER_MM / 2
CHASSIS_BOTTOM_Z_MM: float = 15.0
CHASSIS_TOP_Z_MM: float = CHASSIS_BOTTOM_Z_MM + CHASSIS_MM[2]
CHASSIS_MID_Z_MM: float = (CHASSIS_BOTTOM_Z_MM + CHASSIS_TOP_Z_MM) / 2
STANDOFF_MM: float = 3.0

# Wheel cut-outs in the acrylic plate: x range and |y| range.
WHEEL_NOTCH_X_MM: Tuple[float, float] = (-36.0, 36.0)
WHEEL_NOTCH_ABS_Y_MM: Tuple[float, float] = (52.5, CHASSIS_MM[1] / 2)

# Cable slots through the plate (centre); sizes are (length X, width Y).
CHASSIS_SLOTS_MM: Dict[str, Vec3] = {
    "left": (57.5, 35.0, CHASSIS_MID_Z_MM),
    "right": (57.5, -35.0, CHASSIS_MID_Z_MM),
    "rear": (-44.0, -61.0, CHASSIS_MID_Z_MM),
}
CHASSIS_SLOT_SIZES_MM: Dict[str, Tuple[float, float]] = {
    "left": (36.0, 24.0),
    "right": (36.0, 24.0),
    "rear": (8.0, 16.0),
}

# Motor geometry in its normalised local frame (origin bottom-centre of bbox):
# shaft axis along Y at this x/z offset from the origin.
MOTOR_SHAFT_LOCAL_MM: Vec3 = (-18.25, 0.0, 11.0)
MOTOR_CENTER_ABS_Y_MM: float = 42.5
ENC_DISK_CENTER_ABS_Y_MM: float = 26.1


@dataclass(frozen=True)
class Placement:
    """Where one part sits: local-origin position, Euler XYZ rotation, local bbox size, zone."""

    component: str
    position_mm: Vec3
    rotation_deg: Vec3
    footprint_mm: Vec3
    zone: Zone


def _p(component: str, pos: Vec3, size: Vec3, zone: Zone, rot: Vec3 = (0.0, 0.0, 0.0)) -> Placement:
    return Placement(component=component, position_mm=pos, rotation_deg=rot, footprint_mm=size, zone=zone)


def placements() -> Dict[str, Placement]:
    """All parts of the robot, keyed by component id (netlist ids plus mechanical parts)."""
    top = CHASSIS_TOP_Z_MM
    raised = top + STANDOFF_MM
    shaft_dx = -MOTOR_SHAFT_LOCAL_MM[0]
    shaft_dz = MOTOR_SHAFT_LOCAL_MM[2]
    half_track = TRACK_WIDTH_MM / 2
    flip_x: Vec3 = (180.0, 0.0, 0.0)
    turn_z: Vec3 = (0.0, 0.0, 180.0)

    items: List[Placement] = [
        # Plate and running gear (under the chassis)
        _p("chassis", (0.0, 0.0, CHASSIS_BOTTOM_Z_MM), CHASSIS_MM, "bottom"),
        _p("wheel_l", (0.0, half_track, FLOOR_Z_MM), (WHEEL_DIAMETER_MM, WHEEL_WIDTH_MM, WHEEL_DIAMETER_MM), "bottom"),
        _p("wheel_r", (0.0, -half_track, FLOOR_Z_MM), (WHEEL_DIAMETER_MM, WHEEL_WIDTH_MM, WHEEL_DIAMETER_MM), "bottom"),
        _p("motor_l", (shaft_dx, MOTOR_CENTER_ABS_Y_MM, -shaft_dz), (69.5, 36.5, 22.0), "bottom"),
        _p("motor_r", (shaft_dx, -MOTOR_CENTER_ABS_Y_MM, shaft_dz), (69.5, 36.5, 22.0), "bottom", flip_x),
        _p("disk_l", (0.0, ENC_DISK_CENTER_ABS_Y_MM, -11.0), (22.0, 3.7, 22.0), "bottom"),
        _p("disk_r", (0.0, -ENC_DISK_CENTER_ABS_Y_MM, -11.0), (22.0, 3.7, 22.0), "bottom"),
        _p("enc_l", (13.0, ENC_DISK_CENTER_ABS_Y_MM, CHASSIS_BOTTOM_Z_MM - 12.0), (32.0, 14.0, 12.0), "bottom"),
        _p("enc_r", (13.0, -ENC_DISK_CENTER_ABS_Y_MM, CHASSIS_BOTTOM_Z_MM - 12.0), (32.0, 14.0, 12.0), "bottom"),
        _p("cap_l", (53.0, MOTOR_CENTER_ABS_Y_MM, -2.0), (5.0, 2.5, 4.0), "bottom"),
        _p("cap_r", (53.0, -MOTOR_CENTER_ABS_Y_MM, -2.0), (5.0, 2.5, 4.0), "bottom"),
        _p("batt", (-44.0, 0.0, CHASSIS_BOTTOM_Z_MM), (41.8, 89.0, 15.0), "bottom", flip_x),
        _p("cell_1", (-53.5, 0.0, -5.0), (18.0, 65.0, 18.0), "bottom"),
        _p("cell_2", (-34.5, 0.0, -5.0), (18.0, 65.0, 18.0), "bottom"),
        _p("caster", (-90.0, 0.0, FLOOR_Z_MM), (25.0, 25.0, CHASSIS_BOTTOM_Z_MM - FLOOR_Z_MM), "bottom"),
        # On top of the chassis
        _p("l298n", (58.0, 0.0, raised), (43.04, 43.04, 26.12), "top"),
        _p("mpu", (22.0, 18.0, top + 2.0), (20.32, 15.1, 11.5), "top"),
        _p("uno", (-77.33, 37.135, top + 2.0), (75.27, 53.34, 14.5), "top", (0.0, 0.0, -90.0)),
        _p("bb", (-21.75, -11.15, top), (82.5, 54.5, 8.5), "top", (0.0, 0.0, 90.0)),
        _p("switch", (-95.0, -40.0, top), (15.0, 21.0, 18.0), "top"),
        _p("btn", (-78.0, -40.0, top), (12.0, 12.9, 10.3), "top"),
        _p("buzzer", (-62.0, -40.0, top), (12.0, 12.0, 9.5), "top"),
        # Edges
        _p("sonar", (95.0, 0.0, top), (17.95, 45.0, 25.15), "front"),
        _p("ir_l", (52.0, 66.0, top), (14.0, 32.0, 7.0), "left"),
        _p("ir_r", (52.0, -66.0, top), (14.0, 32.0, 7.0), "right", turn_z),
    ]
    return {pl.component: pl for pl in items}


def rotate(v: Vec3, rotation_deg: Vec3) -> Vec3:
    """Rotates v by Euler XYZ angles (degrees), matching Blender's default 'XYZ' order."""
    x, y, z = v
    rx, ry, rz = (math.radians(a) for a in rotation_deg)
    y, z = y * math.cos(rx) - z * math.sin(rx), y * math.sin(rx) + z * math.cos(rx)
    x, z = x * math.cos(ry) + z * math.sin(ry), -x * math.sin(ry) + z * math.cos(ry)
    x, y = x * math.cos(rz) - y * math.sin(rz), x * math.sin(rz) + y * math.cos(rz)
    return (x, y, z)


def to_world(p: Placement, local: Vec3) -> Vec3:
    """Transforms a point from the part's local frame (mm) into the robot frame (mm)."""
    r = rotate(local, p.rotation_deg)
    return (r[0] + p.position_mm[0], r[1] + p.position_mm[1], r[2] + p.position_mm[2])


def world_aabb(p: Placement) -> Tuple[Vec3, Vec3]:
    """Axis-aligned bounding box (min, max) of a placed part in the robot frame."""
    lx, ly, lz = p.footprint_mm
    corners = [
        to_world(p, (sx * lx / 2, sy * ly / 2, z))
        for sx in (-1.0, 1.0)
        for sy in (-1.0, 1.0)
        for z in (0.0, lz)
    ]
    lo = (min(c[0] for c in corners), min(c[1] for c in corners), min(c[2] for c in corners))
    hi = (max(c[0] for c in corners), max(c[1] for c in corners), max(c[2] for c in corners))
    return (_snap(lo), _snap(hi))


def _snap(v: Vec3) -> Vec3:
    return (round(v[0], 6), round(v[1], 6), round(v[2], 6))


def route_height_mm() -> float:
    """Height of the wire routing layer: tallest top-zone part plus clearance."""
    return max(world_aabb(p)[1][2] for p in placements().values() if p.zone == "top") + ROUTE_CLEARANCE_MM
