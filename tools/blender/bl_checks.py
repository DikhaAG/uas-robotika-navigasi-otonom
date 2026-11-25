#!/usr/bin/env python3
"""
Scene integrity checks for the assembled robot (runs inside Blender only).

check_scene() returns one message per problem:
  (1) the number of wire_* curve objects differs from the netlist,
  (2) a wire end lies more than 0.5 mm from its pin anchor,
  (3) a wire body (ignoring 18 mm at each end, where it legitimately meets its
      connector) comes within the wire radius of any part's mesh; the chassis is
      exempt around the wire's slot vias, where the wire threads the plate.
"""

import math
from typing import Dict, List, Sequence, Tuple

import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

from tools.blender.bl_common import PartResult, mesh_world_vertices
from tools.blender.bl_wires import WIRE_RADIUS_MM, world_anchor
from tools.blender.layout import Vec3

END_TOLERANCE_MM = 0.5
END_EXCLUSION_MM = 18.0
SAMPLE_STEP_MM = 0.5
VIA_EXEMPT_MM = 4.0
CLEARANCE_SLACK_MM = 0.05


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


def _trimmed_samples(points: Sequence[Vec3], trim: float, step: float) -> List[Vec3]:
    lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    total = sum(lengths)
    samples: List[Vec3] = []
    s = trim
    seg, seg_start = 0, 0.0
    while s <= total - trim and seg < len(lengths):
        while seg < len(lengths) and seg_start + lengths[seg] < s:
            seg_start += lengths[seg]
            seg += 1
        if seg >= len(lengths):
            break
        t = (s - seg_start) / lengths[seg] if lengths[seg] > 0 else 0.0
        a, b = points[seg], points[seg + 1]
        samples.append(tuple(a[i] + (b[i] - a[i]) * t for i in range(3)))  # type: ignore[arg-type]
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

    def near(self, p: Vec3, radius: float) -> bool:
        if self.bvh is None:
            return False
        if any(p[i] < self.lo[i] - radius or p[i] > self.hi[i] + radius for i in range(3)):
            return False
        hit = self.bvh.find_nearest(Vector(p), radius)
        return hit[0] is not None


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
        hits: Dict[str, Vec3] = {}
        for p in _trimmed_samples(points, END_EXCLUSION_MM, SAMPLE_STEP_MM):
            for mesh in meshes:
                if mesh.name in hits:
                    continue
                if mesh.name == "chassis" and any(math.dist(p, v) <= VIA_EXEMPT_MM for v in vias):
                    continue
                if mesh.near(p, WIRE_RADIUS_MM + CLEARANCE_SLACK_MM):
                    hits[mesh.name] = p
        for part_name, p in hits.items():
            errors.append(f"(3) {name}: passes through {part_name} near ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm")
    return errors
