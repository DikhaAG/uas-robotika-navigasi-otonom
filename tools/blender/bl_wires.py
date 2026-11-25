#!/usr/bin/env python3
"""
Wires for the 3D robot model (runs inside Blender only).

A wire is a POLY curve (bevel Ø1.6 mm) through the routed points, corners filleted.
Its first and last curve points sit exactly on the two pin anchors, so the scene
check can prove every wire ends on its pin. Ends:
  * dupont: black 2.54×2.54×14 mm housing from the anchor along the pin direction,
    covering the pin;
  * bare:   the first 5 mm are thin tinned copper (into the terminal / onto the tab),
    insulation starts after that.
"""

import math
from typing import Dict, List, Sequence, Tuple

import bpy
from mathutils import Vector

from tools.blender.bl_common import Anchor, PartResult, box, material, mm
from tools.blender.layout import DUPONT_STRAIGHT_MM, Vec3
from tools.blender.netlist import Wire, wire_name
from tools.blender.routing import RoutePlan, WireSpan, plan_routes

WIRE_RADIUS_MM = 0.8
COPPER_RADIUS_MM = 0.45
BARE_LENGTH_MM = 5.0
DUPONT_SIZE_MM: Vec3 = (2.54, 2.54, 14.0)
FILLET_MM = 2.0  # small enough that a rounded corner never cuts towards a neighbouring riser
FILLET_STEPS = 6
END_FILLET_MM = DUPONT_STRAIGHT_MM - DUPONT_SIZE_MM[2] - 0.5


def world_anchor(part: PartResult, pin: str) -> Anchor:
    """Pin anchor of a placed part in the robot frame: (position mm, unit direction)."""
    pos, direction = part.anchors[pin]
    m = part.obj.matrix_world
    w = m @ mm(pos)
    d = (m.to_3x3() @ Vector(direction)).normalized()
    return ((w.x * 1000.0, w.y * 1000.0, w.z * 1000.0), (d.x, d.y, d.z))


def _v(p: Vec3) -> Vector:
    return Vector(p)


def _dedupe(points: Sequence[Vec3]) -> List[Vector]:
    out: List[Vector] = []
    for p in points:
        v = _v(p)
        if not out or (v - out[-1]).length > 1e-6:
            out.append(v)
    cleaned = [out[0]]
    for i in range(1, len(out) - 1):
        a, b, c = cleaned[-1], out[i], out[i + 1]
        if (b - a).normalized().cross((c - b).normalized()).length > 1e-6:
            cleaned.append(b)
    cleaned.append(out[-1])
    return cleaned


def fillet(points: Sequence[Vec3]) -> List[Vec3]:
    """Rounds every interior corner with a quadratic arc (radius ≤ 4 mm, ≤ half of each leg);
    the corners next to the ends stay tight so the dupont/bare straight is kept."""
    pts = _dedupe(points)
    out: List[Vector] = [pts[0]]
    last = len(pts) - 1
    for i in range(1, last):
        prev, corner, nxt = pts[i - 1], pts[i], pts[i + 1]
        limit = END_FILLET_MM if i in (1, last - 1) else FILLET_MM
        r = min(limit, (prev - corner).length / 2, (nxt - corner).length / 2)
        if r <= 0.05:
            out.append(corner)
            continue
        t1 = corner + (prev - corner).normalized() * r
        t2 = corner + (nxt - corner).normalized() * r
        for k in range(FILLET_STEPS + 1):
            t = k / FILLET_STEPS
            out.append(t1 * (1 - t) ** 2 + corner * 2 * (1 - t) * t + t2 * t * t)
    out.append(pts[-1])
    return [(p.x, p.y, p.z) for p in out]


def _add_spline(curve: "bpy.types.Curve", points: Sequence[Vec3], radius: float, mat_index: int) -> None:
    spline = curve.splines.new("POLY")
    spline.points.add(len(points) - 1)
    for sp, p in zip(spline.points, points):
        sp.co = (p[0] * 0.001, p[1] * 0.001, p[2] * 0.001, 1.0)
        sp.radius = radius
    spline.material_index = mat_index


def _dupont(name: str, anchor: Vec3, direction: Vec3, parent: "bpy.types.Object") -> None:
    d = Vector(direction).normalized()
    center = Vector(anchor) + d * (DUPONT_SIZE_MM[2] / 2)
    housing = box(name, DUPONT_SIZE_MM, tuple(center), material("dupont_black", "#0b0f19", roughness=0.5), parent)
    housing.rotation_mode = "QUATERNION"
    housing.rotation_quaternion = Vector((0.0, 0.0, 1.0)).rotation_difference(d)


def make_wire(w: Wire, points_mm: List[Vec3]) -> "bpy.types.Object":
    """Bevelled curve along points_mm (first/last point = pin anchors) with dupont or bare ends."""
    name = wire_name(w)
    path = fillet(points_mm)
    curve = bpy.data.curves.new(name, type="CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = WIRE_RADIUS_MM * 0.001
    curve.bevel_resolution = 3
    curve.use_fill_caps = True
    curve.materials.append(material(f"wire_{w.color}", w.color, roughness=0.45))
    curve.materials.append(material("copper_tinned", "#d6b16a", roughness=0.25))
    copper_r = COPPER_RADIUS_MM / WIRE_RADIUS_MM

    start, end = Vector(path[0]), Vector(path[-1])
    first_dir = (Vector(path[1]) - start).normalized()
    last_dir = (Vector(path[-2]) - end).normalized()
    body: List[Vec3] = list(path)
    if w.src_end == "bare":
        cut = start + first_dir * BARE_LENGTH_MM
        _add_spline(curve, [tuple(start), tuple(cut)], copper_r, 1)
        body[0] = tuple(cut)
    if w.dst_end == "bare":
        cut = end + last_dir * BARE_LENGTH_MM
        body[-1] = tuple(cut)
    _add_spline(curve, body, 1.0, 0)
    if w.dst_end == "bare":
        _add_spline(curve, [tuple(end + last_dir * BARE_LENGTH_MM), tuple(end)], copper_r, 1)

    obj = bpy.data.objects.new(name, curve)
    bpy.context.scene.collection.objects.link(obj)
    obj["src"] = f"{w.src.component}.{w.src.pin}"
    obj["dst"] = f"{w.dst.component}.{w.dst.pin}"
    obj["stage"] = w.stage
    if w.src_end == "dupont":
        _dupont(f"hsg_{name}_a", tuple(start), tuple(first_dir), obj)
    if w.dst_end == "dupont":
        _dupont(f"hsg_{name}_b", tuple(end), tuple(last_dir), obj)
    return obj


def assign_lanes(wires: List[Wire], spans: Dict[str, Tuple[Vec3, Vec3]]) -> Dict[str, RoutePlan]:
    """wire_name -> (lane, L-shape). spans[name] = (start, end) of the wire's horizontal run in the
    routing layer; see routing.plan_routes for the layering rules."""
    return plan_routes([WireSpan(wire_name(w), w.stage, *spans[wire_name(w)]) for w in wires])
