#!/usr/bin/env python3
"""
Unit tests for the 3D model chassis layout (tools/blender/layout.py) and wire
routing (tools/blender/routing.py). Pure Python, no Blender.
Robot frame: +X forward, +Y left, +Z up, origin at the wheel-axle centre, mm.
"""

import itertools
import math
import os
import sys
import unittest
from typing import Dict, List, Tuple

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, REPO_ROOT)

from tools.blender.layout import (  # noqa: E402
    CHASSIS_MM,
    CHASSIS_SLOTS_MM,
    DUPONT_STRAIGHT_MM,
    TRACK_WIDTH_MM,
    WHEEL_DIAMETER_MM,
    Vec3,
    placements,
    route_height_mm,
    world_aabb,
)
from tools.blender.netlist import COMPONENTS  # noqa: E402
from tools.blender.routing import RoutePlan, WireSpan, plan_routes, route  # noqa: E402

Box = Tuple[Vec3, Vec3]
UPPER_ZONES = ("top", "front", "left", "right")


def _overlap(a: Box, b: Box) -> bool:
    return all(a[0][i] < b[1][i] - 1e-9 and b[0][i] < a[1][i] - 1e-9 for i in range(3))


def _add(a: Vec3, b: Vec3, k: float = 1.0) -> Vec3:
    return (a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2])


def _close(a: Vec3, b: Vec3, tol: float = 1e-6) -> bool:
    return all(abs(a[i] - b[i]) <= tol for i in range(3))


def _segments(pts: List[Vec3]) -> List[Tuple[Vec3, Vec3]]:
    return list(zip(pts, pts[1:]))


class TestLayout(unittest.TestCase):
    def setUp(self) -> None:
        self.p = placements()

    def test_constants_match_config_h(self) -> None:
        self.assertEqual(WHEEL_DIAMETER_MM, 65.0)
        self.assertEqual(TRACK_WIDTH_MM, 135.0)
        self.assertEqual(CHASSIS_MM, (210.0, 150.0, 3.0))

    def test_wheel_track_and_diameter(self) -> None:
        self.assertAlmostEqual(self.p["wheel_l"].position_mm[1], 67.5, delta=0.01)
        self.assertAlmostEqual(self.p["wheel_r"].position_mm[1], -67.5, delta=0.01)
        for key in ("wheel_l", "wheel_r"):
            lo, hi = world_aabb(self.p[key])
            self.assertAlmostEqual(hi[0] - lo[0], 65.0, delta=0.01)
            self.assertAlmostEqual(hi[2] - lo[2], 65.0, delta=0.01)
            self.assertAlmostEqual(lo[2], -32.5, delta=0.01, msg="wheel must touch the floor")
            self.assertAlmostEqual((lo[2] + hi[2]) / 2, 0.0, delta=0.01, msg="wheel centre on the axle")

    def test_top_components_do_not_overlap(self) -> None:
        upper = [k for k, v in self.p.items() if v.zone in UPPER_ZONES]
        self.assertGreaterEqual(len(upper), 10)
        for a, b in itertools.combinations(upper, 2):
            with self.subTest(a=a, b=b):
                self.assertFalse(_overlap(world_aabb(self.p[a]), world_aabb(self.p[b])))

    def test_top_components_inside_chassis(self) -> None:
        half_l, half_w = CHASSIS_MM[0] / 2, CHASSIS_MM[1] / 2
        for key, pl in self.p.items():
            if pl.zone != "top":
                continue
            lo, hi = world_aabb(pl)
            with self.subTest(key=key):
                self.assertGreaterEqual(lo[0], -half_l)
                self.assertLessEqual(hi[0], half_l)
                self.assertGreaterEqual(lo[1], -half_w)
                self.assertLessEqual(hi[1], half_w)

    def test_top_components_sit_on_chassis(self) -> None:
        chassis_top = world_aabb(self.p["chassis"])[1][2]
        for key, pl in self.p.items():
            if pl.zone in UPPER_ZONES:
                with self.subTest(key=key):
                    self.assertGreaterEqual(world_aabb(pl)[0][2], chassis_top - 1e-9)

    def test_upper_components_clear_wheels(self) -> None:
        wheels = [world_aabb(self.p[k]) for k in ("wheel_l", "wheel_r")]
        for key, pl in self.p.items():
            if pl.zone in UPPER_ZONES:
                for wheel in wheels:
                    with self.subTest(key=key):
                        self.assertFalse(_overlap(world_aabb(pl), wheel))

    def test_mpu_near_axle_center(self) -> None:
        mpu = self.p["mpu"]
        self.assertLessEqual(math.hypot(mpu.position_mm[0], mpu.position_mm[1]), 30.0)
        self.assertEqual(mpu.rotation_deg[0] % 360, 0.0)
        self.assertEqual(mpu.rotation_deg[1] % 360, 0.0)
        self.assertEqual(mpu.zone, "top")

    def test_sonar_faces_forward_at_front(self) -> None:
        sonar = self.p["sonar"]
        self.assertEqual(sonar.zone, "front")
        for key, pl in self.p.items():
            if key not in ("sonar", "chassis"):
                with self.subTest(key=key):
                    self.assertLess(pl.position_mm[0], sonar.position_mm[0])

    def test_ir_sensors_face_outward(self) -> None:
        self.assertEqual(self.p["ir_l"].zone, "left")
        self.assertGreater(self.p["ir_l"].position_mm[1], 0)
        self.assertEqual(self.p["ir_r"].zone, "right")
        self.assertLess(self.p["ir_r"].position_mm[1], 0)

    def test_bottom_components_under_chassis(self) -> None:
        chassis_bottom = world_aabb(self.p["chassis"])[0][2]
        for key in ("motor_l", "motor_r", "batt", "enc_l", "enc_r", "caster", "disk_l", "disk_r"):
            with self.subTest(key=key):
                self.assertEqual(self.p[key].zone, "bottom")
                self.assertLessEqual(world_aabb(self.p[key])[1][2], chassis_bottom + 1e-9)

    def test_caster_reaches_floor(self) -> None:
        lo, _ = world_aabb(self.p["caster"])
        self.assertAlmostEqual(lo[2], -WHEEL_DIAMETER_MM / 2, delta=0.01)
        self.assertLess(self.p["caster"].position_mm[0], 0, "caster at the rear")

    def test_every_netlist_component_placed(self) -> None:
        for key in COMPONENTS:
            with self.subTest(key=key):
                self.assertIn(key, self.p)
                self.assertEqual(self.p[key].component, key)

    def test_chassis_slots_inside_chassis(self) -> None:
        self.assertEqual(set(CHASSIS_SLOTS_MM), {"left", "right", "rear"})
        lo, hi = world_aabb(self.p["chassis"])
        for name, (x, y, z) in CHASSIS_SLOTS_MM.items():
            with self.subTest(slot=name):
                self.assertTrue(lo[0] < x < hi[0] and lo[1] < y < hi[1] and lo[2] <= z <= hi[2])

    def test_world_aabb_applies_rotation(self) -> None:
        lo, hi = world_aabb(self.p["motor_r"])
        self.assertAlmostEqual((lo[2] + hi[2]) / 2, 0.0, delta=0.01, msg="flipped motor keeps body centred on axle")


class TestRouting(unittest.TestCase):
    A: Vec3 = (-60.0, 30.0, 26.0)
    B: Vec3 = (60.0, -10.0, 40.0)
    UP: Vec3 = (0.0, 0.0, 1.0)
    BACK: Vec3 = (-1.0, 0.0, 0.0)

    def test_route_endpoints_and_straight_exit(self) -> None:
        pts = route(self.A, self.UP, self.B, self.BACK, lane=0)
        self.assertTrue(_close(pts[0], self.A))
        self.assertTrue(_close(pts[-1], self.B))
        self.assertTrue(_close(pts[1], _add(self.A, self.UP, DUPONT_STRAIGHT_MM)))
        self.assertTrue(_close(pts[-2], _add(self.B, self.BACK, DUPONT_STRAIGHT_MM)))

    def test_route_clears_tallest_top_component(self) -> None:
        height = route_height_mm()
        pts = route(self.A, self.UP, self.B, self.BACK, lane=0)
        for p in pts[2:-2]:
            self.assertGreaterEqual(p[2], height - 1e-9)
        for p, q in _segments(pts)[1:-1]:
            if abs(p[2] - q[2]) < 1e-9:
                self.assertGreaterEqual(p[2], height - 1e-9, "horizontal run below route layer")
            else:
                self.assertTrue(_close((p[0], p[1], 0), (q[0], q[1], 0)), "non-horizontal runs must be vertical")

    def test_route_height_above_tallest_top_component(self) -> None:
        tallest = max(world_aabb(pl)[1][2] for pl in placements().values() if pl.zone == "top")
        self.assertAlmostEqual(route_height_mm(), tallest + 10.0)

    def test_route_lanes_separate(self) -> None:
        z0 = max(p[2] for p in route(self.A, self.UP, self.B, self.BACK, lane=0))
        z3 = max(p[2] for p in route(self.A, self.UP, self.B, self.BACK, lane=3))
        self.assertAlmostEqual(z3 - z0, 6.0)

    def test_route_passes_vias(self) -> None:
        a: Vec3 = (40.0, 30.0, 8.0)
        via = CHASSIS_SLOTS_MM["left"]
        pts = route(a, (1.0, 0.0, 0.0), self.B, self.BACK, lane=1, vias=[via])
        self.assertTrue(any(_close(p, via) for p in pts), f"via {via} not in {pts}")
        idx = next(i for i, p in enumerate(pts) if _close(p, via))
        self.assertTrue(_close((pts[idx - 1][0], pts[idx - 1][1], 0), (via[0], via[1], 0)), "via entered vertically")
        self.assertTrue(_close((pts[idx + 1][0], pts[idx + 1][1], 0), (via[0], via[1], 0)), "via left vertically")
        self.assertLess(pts[idx - 1][2], via[2])
        self.assertGreater(pts[idx + 1][2], via[2])

    def test_route_is_axis_aligned(self) -> None:
        pts = route(self.A, self.UP, self.B, self.BACK, lane=2, vias=[CHASSIS_SLOTS_MM["rear"]])
        for p, q in _segments(pts)[1:-1]:
            moved = sum(abs(p[i] - q[i]) > 1e-9 for i in range(3))
            self.assertLessEqual(moved, 1, f"diagonal segment {p} -> {q}")

    def test_route_has_no_zero_length_segments(self) -> None:
        pts = route(self.A, self.UP, self.B, self.BACK, lane=0)
        for p, q in _segments(pts):
            self.assertFalse(_close(p, q))


class TestRoutePlanning(unittest.TestCase):
    """Lane + L-shape choice so that no two wires pass through each other in 3D."""

    def test_overlapping_parallel_runs_get_different_lanes(self) -> None:
        plans = plan_routes([
            WireSpan("a", 1, (0.0, 10.0, 0.0), (50.0, 10.0, 0.0)),
            WireSpan("b", 1, (5.0, 11.0, 0.0), (40.0, 11.0, 0.0)),
        ])
        self.assertNotEqual(plans["a"].lane, plans["b"].lane)

    def test_mutually_covering_wires_are_rejected(self) -> None:
        # Each run passes over the other's drop: no lane order avoids a collision, so refuse loudly.
        with self.assertRaises(ValueError) as ctx:
            plan_routes([
                WireSpan("a", 1, (0.0, 10.0, 0.0), (50.0, 10.0, 0.0)),
                WireSpan("b", 1, (5.0, 11.0, 0.0), (60.0, 11.0, 0.0)),
            ])
        self.assertIn("'a'", str(ctx.exception))

    def test_disjoint_runs_share_lane_zero(self) -> None:
        plans = plan_routes([
            WireSpan("a", 1, (0.0, 10.0, 0.0), (20.0, 30.0, 0.0)),
            WireSpan("b", 2, (40.0, 10.0, 0.0), (60.0, 30.0, 0.0)),
        ])
        self.assertEqual((plans["a"].lane, plans["b"].lane), (0, 0))

    def test_crossing_runs_never_share_a_lane(self) -> None:
        plans = plan_routes([
            WireSpan("a", 1, (0.0, 0.0, 0.0), (40.0, 5.0, 0.0)),
            WireSpan("b", 1, (20.0, -20.0, 0.0), (21.0, 20.0, 0.0)),
        ])
        self.assertNotEqual(plans["a"].lane, plans["b"].lane)

    def test_run_over_a_riser_is_lifted_above_that_wire(self) -> None:
        # b's riser stands at (25, 0); a's only shape-free path must pass over it in one of its shapes.
        plans = plan_routes([
            WireSpan("a", 1, (0.0, 0.0, 0.0), (50.0, 0.5, 0.0)),
            WireSpan("b", 1, (25.0, 0.0, 0.0), (25.0, 40.0, 0.0)),
        ])
        self.assertGreater(plans["a"].lane, plans["b"].lane)

    def test_shape_avoids_other_risers_when_it_can(self) -> None:
        # Going X first would pass over b's riser at (25, 0); going Y first does not.
        plans = plan_routes([
            WireSpan("a", 1, (0.0, 0.0, 0.0), (50.0, 40.0, 0.0)),
            WireSpan("b", 2, (25.0, 0.0, 0.0), (25.0, -30.0, 0.0)),
        ])
        self.assertEqual(plans["a"].shape, "YX")

    def test_parallel_header_columns_use_z_shape(self) -> None:
        # Like Uno D9..D4 -> L298N ENA..IN4: two pin columns along Y, rows in a different order.
        spans = [WireSpan(f"w{i}", 2, (-50.0, 10.0 + 2.54 * i, 0.0), (50.0, -2.54 * i, 0.0)) for i in range(4)]
        plans = plan_routes(spans)
        self.assertIn("XYX", {p.shape for p in plans.values()}, "inner wires must leave/enter the columns square")
        self._assert_apart(spans, plans)

    def _assert_apart(self, spans: List[WireSpan], plans: Dict[str, RoutePlan]) -> None:
        lines = {}
        for sp in spans:
            pl = plans[sp.name]
            a = (sp.start[0], sp.start[1], 20.0)
            b = (sp.end[0], sp.end[1], 20.0)
            lines[sp.name] = route(a, (0.0, 0.0, 1.0), b, (0.0, 0.0, 1.0), pl.lane, shape=pl.shape)
        for x, y in itertools.combinations(spans, 2):
            gap = min(_segment_distance(p, q, r, t) for p, q in _segments(lines[x.name])
                      for r, t in _segments(lines[y.name]))
            with self.subTest(a=x.name, b=y.name):
                self.assertGreaterEqual(gap, 1.6, f"plans {plans[x.name]} {plans[y.name]}")

    def test_planned_routes_keep_wires_apart(self) -> None:
        spans = [WireSpan(f"w{i}", 1 + i % 3, (float(7 * i % 50), float(5 * i), 0.0),
                          (float(60 - 9 * i % 40), float(40 - 3 * i), 0.0)) for i in range(10)]
        self._assert_apart(spans, plan_routes(spans))

    def test_same_stage_bundle_uses_contiguous_lanes(self) -> None:
        spans = [WireSpan(f"w{i}", 2, (0.0, 3.0 * i, 0.0), (50.0, 40.0 + 3.0 * i, 0.0)) for i in range(4)]
        lanes = {p.lane for p in plan_routes(spans).values()}
        self.assertEqual(lanes, set(range(min(lanes), max(lanes) + 1)), "bundle lanes must be contiguous")


def _segment_distance(p1: Vec3, p2: Vec3, q1: Vec3, q2: Vec3) -> float:
    """Minimum distance between two 3D segments (sampled finely; segments here are axis-aligned)."""
    def pts(a: Vec3, b: Vec3) -> List[Vec3]:
        n = max(1, int(math.dist(a, b) / 0.5))
        return [tuple(a[i] + (b[i] - a[i]) * k / n for i in range(3)) for k in range(n + 1)]  # type: ignore[misc]
    return min(math.dist(a, b) for a in pts(p1, p2) for b in pts(q1, q2))


if __name__ == "__main__":
    unittest.main()
