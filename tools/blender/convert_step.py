#!/usr/bin/env python3
"""
Converts STEP models from assets/3d/vendor/ into binary glTF (assets/3d/cache/<key>.glb).

Blender cannot read STEP, so this runs once in the repo-local .venv with cadquery-ocp
(OpenCascade bindings). Colours and part names from the STEP assembly (XCAF) are kept.
Output is in metres, converted from STEP Z-up to glTF Y-up; Blender's glTF importer
turns it back to Z-up, so models arrive in Blender at true size.

Usage: .venv/bin/python tools/blender/convert_step.py
OCP is imported inside convert() so this module stays importable by plain python3/tests.
"""

import sys
from pathlib import Path
from typing import List

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.blender.assets import CACHE_DIR, VENDOR_DIR, ModelSource, load_sources  # noqa: E402

STEP_SUFFIXES = (".step", ".stp")


def glb_path_for(source: ModelSource, cache_dir: Path = CACHE_DIR) -> Path:
    """Where the converted GLB for a manifest entry lives."""
    return cache_dir / f"{source.key}.glb"


def is_step(source: ModelSource) -> bool:
    """True when the manifest file is a STEP model that needs conversion before Blender can use it."""
    return Path(source.filename).suffix.lower() in STEP_SUFFIXES


def needs_conversion(
    sources: List[ModelSource], vendor_dir: Path = VENDOR_DIR, cache_dir: Path = CACHE_DIR
) -> List[ModelSource]:
    """STEP sources whose GLB is missing or older than the STEP file."""
    pending: List[ModelSource] = []
    for source in sources:
        if not is_step(source):
            continue
        glb = glb_path_for(source, cache_dir)
        step = vendor_dir / source.filename
        if not glb.is_file() or glb.stat().st_mtime < step.stat().st_mtime:
            pending.append(source)
    return pending


def convert(step_file: Path, out_glb: Path, linear_deflection_mm: float = 0.05) -> None:
    """Meshes one STEP assembly and writes it as GLB in metres with XCAF colours preserved."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Message import Message_ProgressRange
    from OCP.RWGltf import RWGltf_CafWriter
    from OCP.RWMesh import RWMesh_CoordinateSystem, RWMesh_CoordinateSystemConverter
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.collections import IndexedDataMap_TCollection_AsciiString_TCollection_AsciiString as FileInfo
    from OCP.collections import Sequence_TDF_Label as LabelSequence

    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(True)
    if reader.ReadFile(str(step_file)) != IFSelect_RetDone:
        raise RuntimeError(f"cannot read STEP file {step_file.name}")
    if not reader.Transfer(doc):
        raise RuntimeError(f"cannot transfer STEP file {step_file.name} into an XCAF document")

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    labels = LabelSequence()
    shape_tool.GetFreeShapes(labels)
    if labels.Length() == 0:
        raise RuntimeError(f"STEP file {step_file.name} contains no shapes")
    for i in range(1, labels.Length() + 1):
        shape = shape_tool.GetShape_s(labels.Value(i))
        BRepMesh_IncrementalMesh(shape, linear_deflection_mm, False, 0.3, True)

    converter = RWMesh_CoordinateSystemConverter()
    converter.SetInputLengthUnit(0.001)
    converter.SetInputCoordinateSystem(RWMesh_CoordinateSystem.RWMesh_CoordinateSystem_Zup)
    converter.SetOutputLengthUnit(1.0)
    converter.SetOutputCoordinateSystem(RWMesh_CoordinateSystem.RWMesh_CoordinateSystem_glTF)

    out_glb.parent.mkdir(parents=True, exist_ok=True)
    writer = RWGltf_CafWriter(TCollection_AsciiString(str(out_glb)), True)
    writer.SetCoordinateSystemConverter(converter)
    if not writer.Perform(doc, FileInfo(), Message_ProgressRange()):
        raise RuntimeError(f"glTF export failed for {step_file.name}")


def main() -> int:
    """Converts every STEP whose GLB is missing or stale; returns a process exit status."""
    sources = load_sources()
    pending = needs_conversion(sources)
    for source in pending:
        out = glb_path_for(source)
        print(f"  ⚙ {source.key:12s} {source.filename} → {out.name}")
        try:
            convert(VENDOR_DIR / source.filename, out)
        except (RuntimeError, OSError) as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
    fresh = [s.key for s in sources if is_step(s) and s not in pending]
    print(f"Converted {len(pending)} STEP file(s); up to date: {', '.join(fresh) or '-'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
