#!/usr/bin/env python3
"""
Downloads every model in assets/3d/sources.json into assets/3d/vendor/ and verifies its SHA-256.

Usage:
    python3 tools/blender/fetch_models.py          # download missing files, verify all hashes
    python3 tools/blender/fetch_models.py --pin    # (manifest authoring only) record current hashes

Printables files are fetched anonymously through the public GraphQL mutation
getDownloadLink; GitHub files through raw URLs pinned to a commit SHA. Stdlib only.
"""

import argparse
import dataclasses
import http.client
import json
import sys
import tempfile
import urllib.request
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.blender.assets import (  # noqa: E402
    SOURCES_JSON,
    VENDOR_DIR,
    ModelSource,
    load_sources,
    save_sources,
    sha256_of,
    verify_sha256,
)

PRINTABLES_GRAPHQL = "https://api.printables.com/graphql/"
USER_AGENT = "robot-3d-model-fetch/1.0"
TIMEOUT_S = 60


def _http(request: urllib.request.Request) -> bytes:
    with urllib.request.urlopen(request, timeout=TIMEOUT_S) as response:
        return response.read()


def printables_download_url(source: ModelSource) -> str:
    """Asks Printables for a short-lived anonymous download link of one file."""
    query = (
        "mutation($id: ID!, $printId: ID!) { getDownloadLink(id: $id, printId: $printId, "
        "fileType: stl, source: model_detail) { ok errors { field messages } output { link } } }"
    )
    body = json.dumps(
        {"query": query, "variables": {"id": str(source.file_id), "printId": str(source.print_id)}}
    ).encode("utf-8")
    request = urllib.request.Request(
        PRINTABLES_GRAPHQL,
        data=body,
        headers={"Content-Type": "application/json", "User-Agent": USER_AGENT},
    )
    reply = json.loads(_http(request))
    result = (reply.get("data") or {}).get("getDownloadLink") or {}
    if not result.get("ok") or not result.get("output"):
        raise RuntimeError(f"Printables refused download link for {source.filename}: {reply}")
    return str(result["output"]["link"])


def download(source: ModelSource, dest: Path, expected_sha256: Optional[str]) -> None:
    """Downloads one source to dest. The bytes land in a temp file first and are only renamed
    into place once their hash matches, so a changed upstream file never sits in vendor/.
    Any transfer failure is re-raised as RuntimeError naming the file."""
    try:
        url = printables_download_url(source) if source.kind == "printables" else source.url
        data = _http(urllib.request.Request(url, headers={"User-Agent": USER_AGENT}))
    except (OSError, http.client.HTTPException, ValueError, KeyError) as exc:
        raise RuntimeError(f"download failed for {source.filename}: {exc}") from exc
    if not data:
        raise RuntimeError(f"empty download for {source.filename} from {url}")
    dest.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=dest.parent, delete=False, suffix=".part") as tmp:
        tmp.write(data)
    part = Path(tmp.name)
    try:
        if expected_sha256 is not None:
            actual = sha256_of(part)
            if actual != expected_sha256:
                raise ValueError(
                    f"hash mismatch for {source.filename}: expected {expected_sha256}, actual {actual} "
                    "(upstream file changed? re-check the source, then re-pin with fetch_models.py --pin)"
                )
        part.replace(dest)
    finally:
        part.unlink(missing_ok=True)


def fetch_all(sources: List[ModelSource], pin: bool, vendor_dir: Path = VENDOR_DIR) -> List[ModelSource]:
    """Fetches missing files and verifies hashes; with pin=True returns sources carrying fresh hashes."""
    updated: List[ModelSource] = []
    for source in sources:
        dest = vendor_dir / source.filename
        if not dest.is_file():
            print(f"  ↓ {source.key:12s} {source.filename}")
            download(source, dest, None if pin else source.sha256)
        if pin:
            source = dataclasses.replace(source, sha256=sha256_of(dest))
        else:
            verify_sha256(dest, source.sha256)
        print(f"  ✓ {source.key:12s} {source.filename} ({dest.stat().st_size} bytes)")
        updated.append(source)
    return updated


def main() -> int:
    """CLI entry point; returns a process exit status."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pin", action="store_true", help="record current file hashes into sources.json")
    args = parser.parse_args()

    try:
        sources = fetch_all(load_sources(), pin=args.pin)
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    if args.pin:
        save_sources(sources)
        print(f"Pinned {len(sources)} hashes into {SOURCES_JSON}")
    print(f"All {len(sources)} model files present in {VENDOR_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
