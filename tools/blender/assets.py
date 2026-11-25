#!/usr/bin/env python3
"""
Manifest of downloaded 3D models (assets/3d/sources.json) and helpers to check them.

Downloaded files live in assets/3d/vendor/ and converted GLBs in assets/3d/cache/;
both are git-ignored because several sources carry no redistribution licence.
Pure Python (stdlib only): used by fetch_models.py, convert_step.py and the Blender build.
"""

import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import List, Literal

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSETS_DIR = REPO_ROOT / "assets" / "3d"
SOURCES_JSON = ASSETS_DIR / "sources.json"
VENDOR_DIR = ASSETS_DIR / "vendor"
CACHE_DIR = ASSETS_DIR / "cache"

SourceKind = Literal["github", "printables"]


@dataclass(frozen=True)
class ModelSource:
    """One downloadable model file, pinned by SHA-256."""

    key: str
    filename: str
    kind: SourceKind
    url: str
    print_id: int
    file_id: int
    sha256: str
    license: str
    attribution: str


def load_sources(path: Path = SOURCES_JSON) -> List[ModelSource]:
    """Reads the manifest; raises ValueError on unknown kinds or missing fields."""
    raw = json.loads(path.read_text(encoding="utf-8"))
    sources: List[ModelSource] = []
    for entry in raw["sources"]:
        try:
            source = ModelSource(**entry)
        except TypeError as exc:
            raise ValueError(f"{path}: malformed source entry {entry.get('key', '?')!r}: {exc}") from exc
        if source.kind not in ("github", "printables"):
            raise ValueError(f"{path}: source '{source.key}' has unknown kind '{source.kind}'")
        sources.append(source)
    return sources


def save_sources(sources: List[ModelSource], path: Path = SOURCES_JSON) -> None:
    """Writes the manifest back (used by fetch_models.py --pin)."""
    payload = {"sources": [asdict(s) for s in sources]}
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def sha256_of(path: Path) -> str:
    """Hex SHA-256 of a file's bytes."""
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_sha256(path: Path, expected: str) -> None:
    """Raises ValueError naming the file and both hashes when the content differs from the pin."""
    actual = sha256_of(path)
    if actual != expected:
        raise ValueError(
            f"hash mismatch for {path.name}: expected {expected}, actual {actual} "
            "(upstream file changed? re-check the source, then re-pin with fetch_models.py --pin)"
        )


def missing_assets(sources: List[ModelSource], vendor_dir: Path) -> List[str]:
    """Filenames from the manifest that are not present in vendor_dir, in manifest order."""
    return [s.filename for s in sources if not (vendor_dir / s.filename).is_file()]
