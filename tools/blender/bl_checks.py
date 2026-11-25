#!/usr/bin/env python3
"""
Scene integrity checks for the assembled robot (runs inside Blender only).

check_scene() returns one message per problem:
  (1) the number of wire_* curve objects differs from the netlist,
  (2) a wire end lies more than 0.5 mm from its pin anchor,
  (3) a wire comes within the wire radius of any part's mesh. Only the wire's own two
      parts are ignored within 18 mm of the wire ends (where it meets its connector);
      the chassis is exempt around the wire's slot vias, where the wire threads the plate,
  (4) two wires come closer than one wire diameter (they would pass through each
      other), except right at a pin both of them are soldered to,
  (5) a dupont housing intersects any part other than the one whose pin it sits on.
check_part() checks one built part against the layout footprint (±1 mm, catches a
wrongly scaled model) and the netlist pin names.
"""

import math
from typing import Dict, List, Sequence, Tuple

import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from mathutils.kdtree import KDTree

from tools.blender.bl_common import PartResult, check_dimensions, local_bbox_mm, mesh_world_vertices
from tools.blender.bl_wires import WIRE_RADIUS_MM, world_anchor
from tools.blender.layout import Vec3, placements
from tools.blender.netlist import COMPONENTS

END_TOLERANCE_MM = 0.5
END_EXCLUSION_MM = 18.0
SAMPLE_STEP_MM = 0.5
VIA_EXEMPT_MM = 4.0
CLEARANCE_SLACK_MM = 0.05
WIRE_GAP_MM = 2 * WIRE_RADIUS_MM - CLEARANCE_SLACK_MM
SHARED_PIN_EXEMPT_MM = 8.0
DIMENSION_TOLERANCE_MM = 1.0
ANCHOR_SLACK_MM = 0.5


def check_part(part_id: str, result: PartResult) -> List[str]:
    """Footprint (±1 mm, origin at bbox bottom-centre), exact pin-name set and anchors inside the bbox."""
    errors = [f"{part_id}: {e}" for e in check_dimensions(result, placements()[part_id].footprint_mm, DIMENSION_TOLERANCE_MM)]
    lo, hi = local_bbox_mm(result.obj)
    want, have = set(COMPONENTS.get(part_id, {})), set(result.anchors)
    if want != have:
        errors.append(f"{part_id}: anchors missing {sorted(want - have)} extra {sorted(have - want)}")
    for pin, (pos, _direction) in result.anchors.items():
        if not all(lo[i] - ANCHOR_SLACK_MM <= pos[i] <= hi[i] + ANCHOR_SLACK_MM for i in range(3)):
            errors.append(f"{part_id}.{pin}: anchor {pos} outside bbox {lo}..{hi}")
    return errors


def wire_points_mm(obj: "bpy.types.Object") -> List[Vec3]:
    """All spline points of a wire curve in world space (mm), in path order."""
    m = obj.matrix_world
    pts: List[Vec3] = []
    for spline in obj.data.splines:
        for p in spline.points:
            w = m @ Vector((p.co[0], p.co[1], p.co[2]))
            q = (w.x * 1000.0, w.y * 1000.0, w.z * 1000.0)
            if not pts or math.dist(pts[-1], q) > 1e-6:
                pts.append(q)
    return pts


def _split_ref(ref: str) -> Tuple[str, str]:
    component, pin = ref.split(".", 1)
    return component, pin


def _samples(points: Sequence[Vec3], step: float) -> List[Tuple[Vec3, float]]:
    """(point, distance to the nearer wire end) every `step` mm along the polyline."""
    lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    total = sum(lengths)
    samples: List[Tuple[Vec3, float]] = []
    s = 0.0
    seg, seg_start = 0, 0.0
    while s <= total and seg < len(lengths):
        while seg < len(lengths) and seg_start + lengths[seg] < s:
            seg_start += lengths[seg]
            seg += 1
        if seg >= len(lengths):
            break
        t = (s - seg_start) / lengths[seg] if lengths[seg] > 0 else 0.0
        a, b = points[seg], points[seg + 1]
        p = tuple(a[i] + (b[i] - a[i]) * t for i in range(3))
        samples.append((p, min(s, total - s)))  # type: ignore[arg-type]
        s += step
    return samples


class _PartMesh:
    def __init__(self, name: str, part: PartResult) -> None:
        self.name = name
        verts: List[np.ndarray] = []
        polys: List[List[int]] = []
        offset = 0
        depsgraph = bpy.context.evaluated_depsgraph_get()
        for obj in [part.obj, *part.obj.children_recursive]:
            if obj.type != "MESH" or obj.hide_render:
                continue
            world = mesh_world_vertices(obj) * 1000.0
            evaluated = obj.evaluated_get(depsgraph)
            me = evaluated.to_mesh()
            polys.extend([[offset + v for v in poly.vertices] for poly in me.polygons])
            evaluated.to_mesh_clear()
            verts.append(world)
            offset += len(world)
        all_verts = np.concatenate(verts) if verts else np.zeros((0, 3))
        self.lo = all_verts.min(axis=0) if len(all_verts) else np.zeros(3)
        self.hi = all_verts.max(axis=0) if len(all_verts) else np.zeros(3)
        self.bvh = BVHTree.FromPolygons([tuple(v) for v in all_verts], polys) if polys else None

    def overlaps(self, other: BVHTree) -> bool:
        return self.bvh is not None and bool(self.bvh.overlap(other))

    def near(self, p: Vec3, radius: float) -> bool:
        if self.bvh is None:
            return False
        if any(p[i] < self.lo[i] - radius or p[i] > self.hi[i] + radius for i in range(3)):
            return False
        hit = self.bvh.find_nearest(Vector(p), radius)
        return hit[0] is not None


def _wire_clearance(wire_objs: Dict[str, "bpy.types.Object"]) -> List[str]:
    """(4) pairs of wires whose axes come closer than one wire diameter."""
    names = list(wire_objs)
    samples: List[Tuple[int, Vec3]] = []
    shared: Dict[Tuple[int, int], List[Vec3]] = {}
    ends: List[Dict[str, Vec3]] = []
    for i, name in enumerate(names):
        obj = wire_objs[name]
        pts = wire_points_mm(obj)
        ends.append({obj["src"]: pts[0], obj["dst"]: pts[-1]})
        samples.extend((i, p) for p, _d in _samples(pts, SAMPLE_STEP_MM))
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            common = set(ends[i]) & set(ends[j])
            if common:
                shared[(i, j)] = [ends[i][ref] for ref in common]
    tree = KDTree(len(samples))
    for k, (_i, p) in enumerate(samples):
        tree.insert(Vector(p), k)
    tree.balance()
    reported: Dict[Tuple[int, int], Vec3] = {}
    for i, p in samples:
        for _co, k, _dist in tree.find_range(Vector(p), WIRE_GAP_MM):
            j = samples[k][0]
            if j == i:
                continue
            pair = (min(i, j), max(i, j))
            if pair in reported:
                continue
            if any(math.dist(p, a) <= SHARED_PIN_EXEMPT_MM for a in shared.get(pair, [])):
                continue
            reported[pair] = p
    return [
        f"(4) {names[a]} and {names[b]} pass through each other near ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm"
        for (a, b), p in sorted(reported.items())
    ]


def check_scene(
    parts: Dict[str, PartResult], wire_objs: Dict[str, "bpy.types.Object"], expected_wires: int = 41
) -> List[str]:
    """All integrity problems of the assembled scene; empty when the model is consistent."""
    errors: List[str] = []
    bpy.context.view_layer.update()
    curves = [o for o in bpy.context.scene.objects if o.type == "CURVE" and o.name.startswith("wire_")]
    if len(curves) != expected_wires or len(wire_objs) != expected_wires:
        errors.append(f"(1) expected {expected_wires} wire_* curves, scene has {len(curves)} ({len(wire_objs)} built)")

    meshes = [_PartMesh(name, part) for name, part in parts.items()]
    for name, obj in wire_objs.items():
        points = wire_points_mm(obj)
        for label, ref, end in (("start", obj["src"], points[0]), ("end", obj["dst"], points[-1])):
            component, pin = _split_ref(ref)
            anchor, _direction = world_anchor(parts[component], pin)
            gap = math.dist(anchor, end)
            if gap > END_TOLERANCE_MM:
                errors.append(f"(2) {name}: {label} is {gap:.2f} mm from anchor {ref}")

        vias = [tuple(v) for v in obj.get("vias", [])]
        own = {_split_ref(obj["src"])[0], _split_ref(obj["dst"])[0]}
        hits: Dict[str, Vec3] = {}
        for p, from_end in _samples(points, SAMPLE_STEP_MM):
            for mesh in meshes:
                if mesh.name in hits:
                    continue
                if mesh.name in own and from_end < END_EXCLUSION_MM:
                    continue
                if mesh.name == "chassis" and any(math.dist(p, v) <= VIA_EXEMPT_MM for v in vias):
                    continue
                if mesh.near(p, WIRE_RADIUS_MM + CLEARANCE_SLACK_MM):
                    hits[mesh.name] = p
        for part_name, p in hits.items():
            errors.append(f"(3) {name}: passes through {part_name} near ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm")

        for housing in obj.children:
            if not housing.name.startswith("hsg_"):
                continue
            pin_part = _split_ref(obj["src"] if housing.name.endswith("_a") else obj["dst"])[0]
            verts = mesh_world_vertices(housing) * 1000.0
            bvh = BVHTree.FromPolygons([tuple(v) for v in verts], [list(f.vertices) for f in housing.data.polygons])
            for mesh in meshes:
                if mesh.name != pin_part and mesh.overlaps(bvh):
                    errors.append(f"(5) {housing.name}: dupont housing cuts into {mesh.name}")

    errors.extend(_wire_clearance(wire_objs))
    return errors
