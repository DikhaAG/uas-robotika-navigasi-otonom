#!/usr/bin/env python3
"""
Imports downloaded models (GLB/STL/OBJ) as parts, normalised by assets/3d/anchors.json
(runs inside Blender only).

anchors.json, one entry per manifest key:
    scale          multiplied after the built-in unit conversion (GLB is metres → 1.0;
                   STL/OBJ are assumed mm → 0.001 × scale, e.g. 10 for a model drawn in cm)
    rotation_deg   Euler XYZ applied after scaling, so the PCB lies flat with headers up
    offset_mm      translation applied last, putting the origin at the bbox bottom-centre
    pins           {pin: {"pos": [x, y, z], "dir": [dx, dy, dz]}} in the normalised frame (mm)
    exclude        optional object-name prefixes to drop (e.g. a ruler shipped with the model)
    colors         optional {material-name prefix: "#rrggbb"}; "*" applies to every material
    extra_boxes    optional [{"name", "lo": [x,y,z], "hi": [x,y,z], "color"}] added in the normalised
                   frame for details the model lacks (motor solder tabs, a jumper cap, holder contacts)
"""

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Set

import bpy
from mathutils import Euler, Matrix

from tools.blender.assets import ASSETS_DIR, VENDOR_DIR, ModelSource, load_sources
from tools.blender.bl_common import Anchor, PartResult, box_span, check_dimensions, empty, material, mm
from tools.blender.convert_step import glb_path_for, is_step
from tools.blender.layout import Vec3

ANCHORS_JSON = ASSETS_DIR / "anchors.json"

__all__ = ["import_part", "check_dimensions", "missing_model_files", "model_path", "load_anchor_config"]


@dataclass
class ModelConfig:
    """Normalisation and pin anchors for one downloaded model (one anchors.json entry)."""

    scale: float = 1.0
    rotation_deg: Vec3 = (0.0, 0.0, 0.0)
    offset_mm: Vec3 = (0.0, 0.0, 0.0)
    pins: Dict[str, Anchor] = field(default_factory=dict)
    exclude: List[str] = field(default_factory=list)
    colors: Dict[str, str] = field(default_factory=dict)
    extra_boxes: List[Dict[str, object]] = field(default_factory=list)


def load_anchor_config(path: Path = ANCHORS_JSON) -> Dict[str, ModelConfig]:
    """Parses anchors.json into ModelConfig objects keyed by manifest key."""
    raw = json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}
    configs: Dict[str, ModelConfig] = {}
    for key, entry in raw.items():
        if key.startswith("_"):
            continue
        pins = {
            name: (tuple(p["pos"]), tuple(p["dir"]))  # type: ignore[misc]
            for name, p in entry.get("pins", {}).items()
        }
        configs[key] = ModelConfig(
            scale=float(entry.get("scale", 1.0)),
            rotation_deg=tuple(entry.get("rotation_deg", (0.0, 0.0, 0.0))),  # type: ignore[arg-type]
            offset_mm=tuple(entry.get("offset_mm", (0.0, 0.0, 0.0))),  # type: ignore[arg-type]
            pins=pins,
            exclude=list(entry.get("exclude", [])),
            colors=dict(entry.get("colors", {})),
            extra_boxes=list(entry.get("extra_boxes", [])),
        )
    return configs


def model_path(source: ModelSource) -> Path:
    """File Blender imports for a manifest entry: the converted GLB for STEP, else the vendor file."""
    return glb_path_for(source) if is_step(source) else VENDOR_DIR / source.filename


def missing_model_files() -> List[str]:
    """Vendor files and converted GLBs that must exist before a build, relative to the repo."""
    missing: List[str] = []
    for source in load_sources():
        for path in {VENDOR_DIR / source.filename, model_path(source)}:
            if not path.is_file():
                missing.append(str(path.relative_to(ASSETS_DIR.parents[1])))
    return sorted(set(missing))


def _import_file(path: Path) -> Set["bpy.types.Object"]:
    before = set(bpy.data.objects)
    suffix = path.suffix.lower()
    if suffix == ".glb":
        bpy.ops.import_scene.gltf(filepath=str(path))
    elif suffix == ".stl":
        bpy.ops.wm.stl_import(filepath=str(path))
    elif suffix == ".obj":
        bpy.ops.wm.obj_import(filepath=str(path))
    else:
        raise ValueError(f"unsupported model format: {path.name}")
    return set(bpy.data.objects) - before


def _recolor(objects: Set["bpy.types.Object"], key: str, colors: Dict[str, str]) -> None:
    for obj in objects:
        if obj.type != "MESH":
            continue
        if not obj.data.materials:
            obj.data.materials.append(None)
        for i, slot_mat in enumerate(obj.data.materials):
            slot_name = slot_mat.name if slot_mat is not None else ""
            for prefix, hex_color in colors.items():
                if prefix == "*" or slot_name.startswith(prefix):
                    obj.data.materials[i] = material(f"{key}_{prefix}_{hex_color}", hex_color, roughness=0.45)
                    break


def import_part(key: str, name: str) -> PartResult:
    """Imports one downloaded model as a part rooted at an Empty, normalised per anchors.json."""
    sources = {s.key: s for s in load_sources()}
    config = load_anchor_config().get(key, ModelConfig())
    path = model_path(sources[key])
    unit = 1.0 if path.suffix.lower() == ".glb" else 0.001

    new = _import_file(path)
    for obj in list(new):
        if any(obj.name.startswith(prefix) for prefix in config.exclude):
            new.discard(obj)
            bpy.data.objects.remove(obj, do_unlink=True)
    _recolor(new, key, config.colors)

    root = empty(name)
    rotation = Euler(tuple(math.radians(a) for a in config.rotation_deg), "XYZ").to_matrix().to_4x4()
    normalise = Matrix.Translation(mm(config.offset_mm)) @ rotation @ Matrix.Scale(unit * config.scale, 4)
    bpy.context.view_layer.update()
    for obj in new:
        if obj.parent is None or obj.parent not in new:
            world = obj.matrix_world.copy()
            for collection in list(obj.users_collection):
                collection.objects.unlink(obj)
            root.users_collection[0].objects.link(obj)
            obj.parent = root
            obj.matrix_parent_inverse = Matrix.Identity(4)
            obj.matrix_basis = normalise @ world
        else:
            for collection in list(obj.users_collection):
                collection.objects.unlink(obj)
            root.users_collection[0].objects.link(obj)
    for extra in config.extra_boxes:
        lo, hi = tuple(extra["lo"]), tuple(extra["hi"])  # type: ignore[arg-type]
        mat = material(f"extra_{extra['color']}", str(extra["color"]), roughness=0.35)
        box_span(f"{name}_{extra['name']}", lo, hi, mat, root)  # type: ignore[arg-type]
    bpy.context.view_layer.update()
    return PartResult(root, dict(config.pins))
