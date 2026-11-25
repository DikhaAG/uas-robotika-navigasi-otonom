#!/usr/bin/env python3
"""
Orthogonal wire routing for the Blender 3D model (pure Python, mm, robot frame).

A wire leaves its pin straight along the pin direction for DUPONT_STRAIGHT_MM,
climbs vertically to its lane in the routing layer above the tallest top part,
runs horizontally (X then Y, or Y then X per its planned shape) and drops vertically
onto the far pin. plan_routes() layers the wires so none pass through each other.
Wires from parts under the chassis pass through chassis slots ("vias"): each via
is entered and left vertically, so the wire threads the slot instead of the plate.
"""

from dataclasses import dataclass
from typing import Dict, List, Literal, Sequence, Set, Tuple

from tools.blender.layout import DUPONT_STRAIGHT_MM, Vec3, route_height_mm

Shape = Literal["XY", "YX", "XYX", "YXY"]
SHAPES: Tuple[Shape, ...] = ("XY", "YX", "XYX", "YXY")  # preference order on equal cost
Point2 = Tuple[float, float]
Seg2 = Tuple[Point2, Point2]

LANE_PITCH_MM: float = 2.0
POST_CLEAR_MM: float = 2.0  # a run closer than this to another wire's riser/drop must pass above it
SAME_LANE_CLEAR_MM: float = 3.0  # runs closer than this (incl. crossings) must not share a lane
VIA_EXIT_MM: float = 6.0
_EPS = 1e-9


@dataclass(frozen=True)
class WireSpan:
    """A wire's footprint in the routing layer: it rises at start (x, y), runs, and drops at end."""

    name: str
    stage: int
    start: Vec3
    end: Vec3


@dataclass(frozen=True)
class RoutePlan:
    """Lane (height step above the routing layer) and run shape: an L ("XY", "YX": which axis first)
    or a Z ("XYX", "YXY": leave and arrive along the first axis, turning halfway between the pins),
    which lets a wire leave and enter header columns square without running along them."""

    lane: int
    shape: Shape


def corners(start: Point2, end: Point2, shape: Shape) -> List[Point2]:
    """The horizontal path in the routing layer from start to end (inclusive) for a shape."""
    (px, py), (qx, qy) = start, end
    if shape == "XY":
        mids = [(qx, py)]
    elif shape == "YX":
        mids = [(px, qy)]
    elif shape == "XYX":
        mx = (px + qx) / 2
        mids = [(mx, py), (mx, qy)]
    else:
        my = (py + qy) / 2
        mids = [(px, my), (qx, my)]
    return [start, *mids, end]


def _legs(span: WireSpan, shape: Shape) -> List[Seg2]:
    path = corners((span.start[0], span.start[1]), (span.end[0], span.end[1]), shape)
    return [(a, b) for a, b in zip(path, path[1:]) if abs(a[0] - b[0]) + abs(a[1] - b[1]) > _EPS]


def _posts(span: WireSpan) -> List[Point2]:
    return [(span.start[0], span.start[1]), (span.end[0], span.end[1])]


def _point_segment(p: Point2, seg: Seg2) -> float:
    (ax, ay), (bx, by) = seg
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 < _EPS else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / length2))
    return ((p[0] - ax - t * dx) ** 2 + (p[1] - ay - t * dy) ** 2) ** 0.5


def _segments_cross(s: Seg2, t: Seg2) -> bool:
    def orient(a: Point2, b: Point2, c: Point2) -> float:
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    d1, d2 = orient(t[0], t[1], s[0]), orient(t[0], t[1], s[1])
    d3, d4 = orient(s[0], s[1], t[0]), orient(s[0], s[1], t[1])
    return d1 * d2 < 0 and d3 * d4 < 0


def _segment_segment(s: Seg2, t: Seg2) -> float:
    if _segments_cross(s, t):
        return 0.0
    return min(_point_segment(s[0], t), _point_segment(s[1], t), _point_segment(t[0], s), _point_segment(t[1], s))


def _covers(w: WireSpan, shape: Shape, u: WireSpan) -> bool:
    """True when one of w's runs passes over a riser/drop of u (so w must be in a higher lane)."""
    own = _posts(w)
    for post in _posts(u):
        if any(abs(post[0] - o[0]) + abs(post[1] - o[1]) < _EPS for o in own):
            continue
        if any(_point_segment(post, leg) < POST_CLEAR_MM for leg in _legs(w, shape)):
            return True
    return False


def _same_lane_conflict(w: WireSpan, ws: Shape, v: WireSpan, vs: Shape) -> bool:
    return any(_segment_segment(a, b) < SAME_LANE_CLEAR_MM for a in _legs(w, ws) for b in _legs(v, vs))


def _above_graph(spans: Sequence[WireSpan], shapes: Dict[str, Shape]) -> Dict[str, Set[str]]:
    """name -> names of wires it must be above."""
    return {
        w.name: {u.name for u in spans if u.name != w.name and _covers(w, shapes[w.name], u)} for w in spans
    }


def _cyclic_nodes(graph: Dict[str, Set[str]]) -> Set[str]:
    """Nodes on a cycle (members of strongly connected components larger than one)."""
    index: Dict[str, int] = {}
    low: Dict[str, int] = {}
    stack: List[str] = []
    on_stack: Set[str] = set()
    result: Set[str] = set()
    counter = [0]

    def visit(v: str) -> None:
        index[v] = low[v] = counter[0]
        counter[0] += 1
        stack.append(v)
        on_stack.add(v)
        for w in graph[v]:
            if w not in index:
                visit(w)
                low[v] = min(low[v], low[w])
            elif w in on_stack:
                low[v] = min(low[v], index[w])
        if low[v] == index[v]:
            component = []
            while True:
                w = stack.pop()
                on_stack.discard(w)
                component.append(w)
                if w == v:
                    break
            if len(component) > 1:
                result.update(component)

    for v in graph:
        if v not in index:
            visit(v)
    return result


def _choose_shapes(spans: Sequence[WireSpan]) -> Dict[str, Shape]:
    shapes: Dict[str, Shape] = {}
    for w in spans:
        cost = {s: sum(_covers(w, s, u) for u in spans if u.name != w.name) for s in SHAPES}
        shapes[w.name] = min(SHAPES, key=lambda s: (cost[s], SHAPES.index(s)))
    for _ in range(4 * len(spans)):
        cyclic = _cyclic_nodes(_above_graph(spans, shapes))
        if not cyclic:
            return shapes
        best = None
        for name in sorted(cyclic):
            for alternative in SHAPES:
                if alternative == shapes[name]:
                    continue
                trial = dict(shapes)
                trial[name] = alternative
                remaining = len(_cyclic_nodes(_above_graph(spans, trial)))
                if best is None or remaining < best[0]:
                    best = (remaining, trial)
        assert best is not None
        if best[0] >= len(cyclic):
            break
        shapes = best[1]
    cyclic = _cyclic_nodes(_above_graph(spans, shapes))
    if cyclic:
        raise ValueError(f"wires cannot be layered without passing through each other: {sorted(cyclic)}")
    return shapes


def plan_routes(spans: Sequence[WireSpan]) -> Dict[str, RoutePlan]:
    """Picks each wire's L-shape (fewest other risers under its runs) and lane, so that a run that
    passes over another wire's riser/drop is above that wire, and wires sharing a lane never come
    within SAME_LANE_CLEAR_MM of each other (crossings included). Lanes are allocated bottom-up in
    dependency order, then by stage, so each stage's bundle stays together."""
    shapes = _choose_shapes(spans)
    above = _above_graph(spans, shapes)
    order_key = {w.name: (w.stage, i) for i, w in enumerate(spans)}
    by_name = {w.name: w for w in spans}
    pending = {name: set(deps) for name, deps in above.items()}
    plans: Dict[str, RoutePlan] = {}
    while pending:
        ready = sorted((n for n, deps in pending.items() if not deps), key=order_key.__getitem__)
        name = ready[0]
        w = by_name[name]
        lane = max((plans[u].lane + 1 for u in above[name]), default=0)
        while any(
            plans[v].lane == lane and _same_lane_conflict(w, shapes[name], by_name[v], plans[v].shape) for v in plans
        ):
            lane += 1
        plans[name] = RoutePlan(lane=lane, shape=shapes[name])
        del pending[name]
        for deps in pending.values():
            deps.discard(name)
    return plans


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
    a: Vec3, a_dir: Vec3, b: Vec3, b_dir: Vec3, lane: int, vias: Sequence[Vec3] = (), shape: Shape = "XY"
) -> List[Vec3]:
    """Polyline (mm) from pin a to pin b; a_dir/b_dir are unit vectors pointing out of each pin.
    shape picks the run in the routing layer (see RoutePlan)."""
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
        for x, y in corners((line.cur[0], line.cur[1]), (b1[0], b1[1]), shape)[1:]:
            line.go((x, y, layer))
    else:
        line.manhattan(b1[0], b1[1])
    line.go(b1)
    line.go(b)
    return line.points
