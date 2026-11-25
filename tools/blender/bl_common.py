#!/usr/bin/env python3
"""
Shared Blender helpers for the 3D robot model (runs inside Blender only).

Conventions: scene unit is the metre (1 BU = 1 m); every helper takes millimetres
and converts with mm(). A part is an Empty root at the part's local origin (centre
of its bounding box's bottom face) with mesh children; anchors are pin positions in
that local frame (mm) plus a unit vector pointing out of the pin, along which the
mating dupont housing or bare wire end leaves the part.
"""

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Literal, Optional, Sequence, Tuple

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

from tools.blender.layout import Vec3

Anchor = Tuple[Vec3, Vec3]
Axis = Literal["X", "Y", "Z"]

PIN_PITCH_MM = 2.54
PIN_SIZE_MM = 0.64


@dataclass
class PartResult:
    """A built part: its root object and its pin anchors in the root's local frame (mm)."""

    obj: "bpy.types.Object"
    anchors: Dict[str, Anchor] = field(default_factory=dict)


def mm(v: Vec3) -> Vector:
    """Millimetre triple to a Blender vector in metres."""
    return Vector((v[0] * 0.001, v[1] * 0.001, v[2] * 0.001))


def reset_scene() -> None:
    """Empties the scene and sets metric units with scale 1.0 (1 BU = 1 m)."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.unit_settings.length_unit = "MILLIMETERS"


def _srgb_to_linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def hex_rgba(hex_color: str, alpha: float = 1.0) -> Tuple[float, float, float, float]:
    """'#rrggbb' to linear RGBA for shader inputs."""
    h = hex_color.lstrip("#")
    r, g, b = (_srgb_to_linear(int(h[i : i + 2], 16) / 255.0) for i in (0, 2, 4))
    return (r, g, b, alpha)


def _set_input(node: "bpy.types.Node", names: Sequence[str], value: object) -> None:
    for name in names:
        if name in node.inputs:
            node.inputs[name].default_value = value
            return


def material(
    name: str, hex_color: str, roughness: float = 0.5, transmission: float = 0.0, alpha: float = 1.0
) -> "bpy.types.Material":
    """Principled material, reused by name so identical colours share one datablock."""
    existing = bpy.data.materials.get(name)
    if existing is not None:
        return existing
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    mat.diffuse_color = hex_rgba(hex_color, alpha)
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    _set_input(bsdf, ["Base Color"], hex_rgba(hex_color))
    _set_input(bsdf, ["Roughness"], roughness)
    _set_input(bsdf, ["Transmission Weight", "Transmission"], transmission)
    _set_input(bsdf, ["Alpha"], alpha)
    if transmission > 0.0:
        _set_input(bsdf, ["IOR"], 1.49)
    if alpha < 1.0 and hasattr(mat, "surface_render_method"):
        mat.surface_render_method = "BLENDED"
    return mat


def _link(obj: "bpy.types.Object", parent: Optional["bpy.types.Object"]) -> "bpy.types.Object":
    collection = parent.users_collection[0] if parent is not None else bpy.context.scene.collection
    collection.objects.link(obj)
    if parent is not None:
        obj.parent = parent
    return obj


def empty(name: str, parent: Optional["bpy.types.Object"] = None) -> "bpy.types.Object":
    """Plain axes Empty used as a part root."""
    obj = bpy.data.objects.new(name, None)
    obj.empty_display_type = "PLAIN_AXES"
    obj.empty_display_size = 0.01
    return _link(obj, parent)


def _mesh_object(
    name: str, bm: "bmesh.types.BMesh", mat: Optional["bpy.types.Material"], parent: Optional["bpy.types.Object"]
) -> "bpy.types.Object":
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    if mat is not None:
        me.materials.append(mat)
    obj = bpy.data.objects.new(name, me)
    return _link(obj, parent)


def _axis_matrix(axis: Axis) -> Matrix:
    if axis == "X":
        return Matrix.Rotation(math.radians(90.0), 4, "Y")
    if axis == "Y":
        return Matrix.Rotation(math.radians(-90.0), 4, "X")
    return Matrix.Identity(4)


def box(
    name: str,
    size_mm: Vec3,
    center_mm: Vec3,
    mat: Optional["bpy.types.Material"],
    parent: Optional["bpy.types.Object"] = None,
) -> "bpy.types.Object":
    """Axis-aligned box of size_mm centred at center_mm (parent-local)."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=mm(size_mm), verts=bm.verts)
    obj = _mesh_object(name, bm, mat, parent)
    obj.location = mm(center_mm)
    return obj


def box_span(
    name: str,
    lo_mm: Vec3,
    hi_mm: Vec3,
    mat: Optional["bpy.types.Material"],
    parent: Optional["bpy.types.Object"] = None,
) -> "bpy.types.Object":
    """Axis-aligned box between two corners (mm)."""
    size = (hi_mm[0] - lo_mm[0], hi_mm[1] - lo_mm[1], hi_mm[2] - lo_mm[2])
    center = ((hi_mm[0] + lo_mm[0]) / 2, (hi_mm[1] + lo_mm[1]) / 2, (hi_mm[2] + lo_mm[2]) / 2)
    return box(name, size, center, mat, parent)


def cylinder(
    name: str,
    radius_mm: float,
    depth_mm: float,
    center_mm: Vec3,
    axis: Axis,
    mat: Optional["bpy.types.Material"],
    parent: Optional["bpy.types.Object"] = None,
    segments: int = 32,
) -> "bpy.types.Object":
    """Closed cylinder along axis, centred at center_mm."""
    bm = bmesh.new()
    r = radius_mm * 0.001
    bmesh.ops.create_cone(
        bm, cap_ends=True, cap_tris=False, segments=segments, radius1=r, radius2=r, depth=depth_mm * 0.001
    )
    bmesh.ops.transform(bm, matrix=_axis_matrix(axis), verts=bm.verts)
    obj = _mesh_object(name, bm, mat, parent)
    obj.location = mm(center_mm)
    return obj


def tube(
    name: str,
    outer_mm: float,
    inner_mm: float,
    depth_mm: float,
    center_mm: Vec3,
    axis: Axis,
    mat: Optional["bpy.types.Material"],
    parent: Optional["bpy.types.Object"] = None,
    segments: int = 48,
) -> "bpy.types.Object":
    """Hollow cylinder (e.g. a tyre) along axis, centred at center_mm."""
    bm = bmesh.new()
    h = depth_mm * 0.0005
    rings: List[List["bmesh.types.BMVert"]] = []
    for radius, z in ((outer_mm, -h), (outer_mm, h), (inner_mm, h), (inner_mm, -h)):
        r = radius * 0.001
        rings.append(
            [
                bm.verts.new((r * math.cos(2 * math.pi * i / segments), r * math.sin(2 * math.pi * i / segments), z))
                for i in range(segments)
            ]
        )
    for k in range(4):
        a, b = rings[k], rings[(k + 1) % 4]
        for i in range(segments):
            j = (i + 1) % segments
            bm.faces.new((a[i], a[j], b[j], b[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bmesh.ops.transform(bm, matrix=_axis_matrix(axis), verts=bm.verts)
    obj = _mesh_object(name, bm, mat, parent)
    obj.location = mm(center_mm)
    return obj


def sphere(
    name: str,
    radius_mm: float,
    center_mm: Vec3,
    mat: Optional["bpy.types.Material"],
    parent: Optional["bpy.types.Object"] = None,
) -> "bpy.types.Object":
    """UV sphere centred at center_mm."""
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16, radius=radius_mm * 0.001)
    obj = _mesh_object(name, bm, mat, parent)
    obj.location = mm(center_mm)
    return obj


def merged_boxes(
    name: str,
    boxes_mm: Iterable[Tuple[Vec3, Vec3]],
    mat: Optional["bpy.types.Material"],
    parent: Optional["bpy.types.Object"] = None,
) -> "bpy.types.Object":
    """Many (size, center) boxes in a single mesh object (e.g. breadboard holes)."""
    bm = bmesh.new()
    for size, center in boxes_mm:
        geom = bmesh.ops.create_cube(bm, size=1.0)
        verts = geom["verts"]
        bmesh.ops.scale(bm, vec=mm(size), verts=verts)
        bmesh.ops.translate(bm, vec=mm(center), verts=verts)
    return _mesh_object(name, bm, mat, parent)


def text_label(
    name: str,
    body: str,
    center_mm: Vec3,
    size_mm: float,
    parent: Optional["bpy.types.Object"] = None,
    rotation_deg: Vec3 = (0.0, 0.0, 0.0),
    hex_color: str = "#f8fafc",
) -> "bpy.types.Object":
    """Flat silkscreen-style text, centred at center_mm, facing +Z before rotation."""
    curve = bpy.data.curves.new(name, type="FONT")
    curve.body = body
    curve.size = size_mm * 0.001
    curve.align_x = "CENTER"
    curve.align_y = "CENTER"
    curve.materials.append(material(f"silk_{hex_color}", hex_color, roughness=0.6))
    obj = bpy.data.objects.new(name, curve)
    obj.location = mm(center_mm)
    obj.rotation_euler = tuple(math.radians(a) for a in rotation_deg)
    return _link(obj, parent)


def header_pins(
    name: str,
    count: int,
    origin_mm: Vec3,
    axis: Literal["X", "Y"],
    pin_dir: Vec3,
    parent: Optional["bpy.types.Object"],
    pin_length_mm: float = 6.0,
) -> List[Vec3]:
    """Male pin header: black housing (2.54 cube per pin) plus square pins along pin_dir.

    origin_mm is the base-centre of pin 0 (where the housing sits); pins are spaced
    2.54 mm along +axis. Returns, per pin, the mating point on the housing face from
    which the pin sticks out (the anchor position for a dupont housing).
    """
    step = (PIN_PITCH_MM, 0.0, 0.0) if axis == "X" else (0.0, PIN_PITCH_MM, 0.0)
    d = Vector(pin_dir).normalized()
    black = material("header_black", "#111827", roughness=0.6)
    gold = material("pin_gold", "#d4a017", roughness=0.25)
    housing_boxes: List[Tuple[Vec3, Vec3]] = []
    pin_boxes: List[Tuple[Vec3, Vec3]] = []
    faces: List[Vec3] = []
    for i in range(count):
        base = Vector(origin_mm) + Vector(step) * i
        center = base + d * (PIN_PITCH_MM / 2)
        housing_boxes.append(((PIN_PITCH_MM,) * 3, tuple(center)))
        face = base + d * PIN_PITCH_MM
        faces.append((face.x, face.y, face.z))
        pin_center = face + d * (pin_length_mm / 2)
        pin_size = tuple(pin_length_mm if abs(c) > 0.5 else PIN_SIZE_MM for c in d)
        pin_boxes.append((pin_size, tuple(pin_center)))
    merged_boxes(f"{name}_housing", housing_boxes, black, parent)
    merged_boxes(f"{name}_pins", pin_boxes, gold, parent)
    return faces


def _visible_meshes(root: "bpy.types.Object") -> List["bpy.types.Object"]:
    objs = [root, *root.children_recursive]
    return [o for o in objs if o.type == "MESH" and not o.hide_render]


def mesh_world_vertices(obj: "bpy.types.Object") -> np.ndarray:
    """Evaluated (modifiers applied) vertex positions of one object in world space, metres, (N, 3)."""
    depsgraph = bpy.context.evaluated_depsgraph_get()
    evaluated = obj.evaluated_get(depsgraph)
    me = evaluated.to_mesh()
    try:
        coords = np.empty(len(me.vertices) * 3, dtype=np.float64)
        me.vertices.foreach_get("co", coords)
    finally:
        evaluated.to_mesh_clear()
    coords = coords.reshape(-1, 3)
    m = np.array(evaluated.matrix_world)
    return coords @ m[:3, :3].T + m[:3, 3]


def local_bbox_mm(root: "bpy.types.Object") -> Tuple[Vec3, Vec3]:
    """Bounding box (min, max) of all render-visible meshes under root, in root's local frame, mm."""
    bpy.context.view_layer.update()
    inv = np.array(root.matrix_world.inverted())
    chunks = []
    for obj in _visible_meshes(root):
        world = mesh_world_vertices(obj)
        if len(world):
            chunks.append(world @ inv[:3, :3].T + inv[:3, 3])
    if not chunks:
        return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    pts = np.concatenate(chunks) * 1000.0
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return ((float(lo[0]), float(lo[1]), float(lo[2])), (float(hi[0]), float(hi[1]), float(hi[2])))


def check_dimensions(result: PartResult, expected_mm: Vec3, tol_mm: float = 1.0) -> List[str]:
    """Errors when the part's bbox size differs from expected_mm, or its origin is not bottom-centre."""
    lo, hi = local_bbox_mm(result.obj)
    errors: List[str] = []
    for i, axis in enumerate("XYZ"):
        size = hi[i] - lo[i]
        if abs(size - expected_mm[i]) > tol_mm:
            errors.append(f"bbox {axis} = {size:.2f} mm, expected {expected_mm[i]:.2f} ±{tol_mm}")
    center = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2)
    if abs(center[0]) > tol_mm or abs(center[1]) > tol_mm or abs(lo[2]) > tol_mm:
        errors.append(
            f"origin not at bbox bottom-centre: centre xy=({center[0]:.2f}, {center[1]:.2f}) bottom z={lo[2]:.2f}"
        )
    return errors


def set_render_engine(scene: "bpy.types.Scene") -> None:
    """EEVEE under whichever identifier this Blender build uses."""
    for engine in ("BLENDER_EEVEE", "BLENDER_EEVEE_NEXT"):
        try:
            scene.render.engine = engine
            return
        except TypeError:
            continue


def studio_lighting(center: Vector, radius: float, collection: Optional["bpy.types.Collection"] = None) -> None:
    """White world plus key/fill/rim sun lights; radius only places the light objects (metres)."""
    scene = bpy.context.scene
    target = collection or scene.collection
    world = bpy.data.worlds.new("studio_world")
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    bg.inputs["Color"].default_value = (1.0, 1.0, 1.0, 1.0)
    bg.inputs["Strength"].default_value = 0.35
    scene.world = world
    for name, offset, strength in (
        ("key", Vector((1.0, -1.2, 1.6)), 3.0),
        ("fill", Vector((-1.4, -0.6, 0.8)), 1.0),
        ("rim", Vector((-0.4, 1.4, 1.2)), 1.5),
    ):
        light = bpy.data.lights.new(f"light_{name}", type="SUN")
        light.energy = strength
        light.angle = 0.2
        obj = bpy.data.objects.new(f"light_{name}", light)
        obj.location = center + offset.normalized() * radius * 4
        obj.rotation_euler = (-offset).to_track_quat("-Z", "Y").to_euler()
        target.objects.link(obj)


def aim_camera(
    name: str, center: Vector, direction: Vector, distance: float, ortho_scale: Optional[float] = None
) -> "bpy.types.Object":
    """Camera at center + direction*distance looking at center (metres)."""
    cam_data = bpy.data.cameras.new(name)
    cam_data.clip_start = 0.001
    cam_data.clip_end = 10.0
    if ortho_scale is not None:
        cam_data.type = "ORTHO"
        cam_data.ortho_scale = ortho_scale
    else:
        cam_data.lens = 50.0
    cam = bpy.data.objects.new(name, cam_data)
    cam.location = center + direction.normalized() * distance
    up = "Y" if abs(direction.normalized().z) < 0.99 else "Y"
    cam.rotation_euler = (center - cam.location).to_track_quat("-Z", up).to_euler()
    bpy.context.scene.collection.objects.link(cam)
    return cam


def render_to(path: Path, camera: "bpy.types.Object", width: int, height: int) -> None:
    """Renders the scene through camera to a PNG file."""
    scene = bpy.context.scene
    set_render_engine(scene)
    scene.camera = camera
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    scene.render.image_settings.file_format = "PNG"
    path.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


def render_closeup(
    result: PartResult, path: Path, direction: Vec3 = (0.8, -1.0, 1.1), show_anchors: bool = True
) -> None:
    """Labelled close-up of one part (800×600), optionally with a red Ø1 mm marker on every anchor."""
    lo, hi = local_bbox_mm(result.obj)
    center = mm(((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2))
    radius = max(Vector(mm(hi)) - Vector(mm(lo))) * 0.5 + 0.004
    if show_anchors:
        red = material("anchor_marker", "#ef4444", roughness=0.3)
        for pin, (pos, _d) in result.anchors.items():
            sphere(f"anchor_{pin}", 0.5, pos, red, result.obj)
    studio_lighting(center, radius)
    cam = aim_camera("closeup_cam", center, Vector(direction), radius * 4.2)
    render_to(path, cam, 800, 600)
