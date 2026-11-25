#!/usr/bin/env python3
"""
Orthogonal wire routing for the Blender 3D model (pure Python, mm, robot frame).

A wire leaves its pin straight along the pin direction for DUPONT_STRAIGHT_MM,
climbs vertically to its lane in the routing layer above the tallest top part,
runs horizontally (X first, then Y) and drops vertically onto the far pin.
Wires from parts under the chassis pass through chassis slots ("vias"): each via
is entered and left vertically, so the wire threads the slot instead of the plate.
"""

from typing import List, Sequence

from tools.blender.layout import DUPONT_STRAIGHT_MM, Vec3, route_height_mm

LANE_PITCH_MM: float = 2.0
VIA_EXIT_MM: float = 6.0
_EPS = 1e-9


def _offset(p: Vec3, d: Vec3, k: float) -> Vec3:
    return (p[0] + d[0] * k, p[1] + d[1] * k, p[2] + d[2] * k)


def _same(a: Vec3, b: Vec3) -> bool:
    return all(abs(a[i] - b[i]) <= _EPS for i in range(3))


class _Polyline:
    def __init__(self, start: Vec3) -> None:
        self.points: List[Vec3] = [start]

    @property
    def cur(self) -> Vec3:
        return self.points[-1]

    def go(self, p: Vec3) -> None:
        if not _same(p, self.cur):
            self.points.append(p)

    def manhattan(self, x: float, y: float) -> None:
        """Horizontal move at the current height: X first, then Y."""
        z = self.cur[2]
        self.go((x, self.cur[1], z))
        self.go((x, y, z))


def route(
    a: Vec3, a_dir: Vec3, b: Vec3, b_dir: Vec3, lane: int, vias: Sequence[Vec3] = ()
) -> List[Vec3]:
    """Polyline (mm) from pin a to pin b; a_dir/b_dir are unit vectors pointing out of each pin."""
    layer = route_height_mm() + lane * LANE_PITCH_MM
    a1 = _offset(a, a_dir, DUPONT_STRAIGHT_MM)
    b1 = _offset(b, b_dir, DUPONT_STRAIGHT_MM)

    line = _Polyline(a)
    line.go(a1)
    for via in vias:
        if line.cur[2] > via[2]:
            line.go((line.cur[0], line.cur[1], layer))
            line.manhattan(via[0], via[1])
            line.go(via)
            line.go((via[0], via[1], via[2] - VIA_EXIT_MM))
        else:
            line.manhattan(via[0], via[1])
            line.go(via)
            line.go((via[0], via[1], layer))

    below = bool(vias) and line.cur[2] < vias[-1][2]
    if not below:
        line.go((line.cur[0], line.cur[1], layer))
    line.manhattan(b1[0], b1[1])
    line.go(b1)
    line.go(b)
    return line.points
