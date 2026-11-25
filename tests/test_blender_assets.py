#!/usr/bin/env python3
"""
Unit tests for the 3D model source manifest and fetch helpers (tools/blender/assets.py).
Pure Python, no network and no Blender: safe for `make test`.
"""

import hashlib
import os
import re
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, REPO_ROOT)

from tools.blender.assets import (  # noqa: E402
    CACHE_DIR,
    ModelSource,
    load_sources,
    missing_assets,
    verify_sha256,
)
from tools.blender.convert_step import glb_path_for, needs_conversion  # noqa: E402

# "wheel" is procedural: the only login-free TT wheel (Printables 306463) is Ø60, Config.h needs Ø65.
REQUIRED_KEYS = {"uno", "l298n", "sonar", "mpu", "motor_tt", "enc_disk", "batt_holder", "btn"}


def _fake_source(key: str, filename: str) -> ModelSource:
    return ModelSource(
        key=key,
        filename=filename,
        kind="github",
        url=f"https://example.invalid/{filename}",
        print_id=0,
        file_id=0,
        sha256="0" * 64,
        license="CC0",
        attribution="test",
    )


class TestModelSources(unittest.TestCase):
    def test_sources_schema(self) -> None:
        sources = load_sources()
        keys = [s.key for s in sources]
        self.assertEqual(len(keys), len(set(keys)), "duplicate manifest keys")
        self.assertTrue(REQUIRED_KEYS.issubset(keys), f"missing keys: {REQUIRED_KEYS - set(keys)}")
        for s in sources:
            with self.subTest(key=s.key):
                self.assertRegex(s.sha256, r"^[0-9a-f]{64}$")
                self.assertTrue(s.license.strip())
                self.assertTrue(s.filename.strip())
                if s.kind == "github":
                    self.assertTrue(s.url.startswith("https://raw.githubusercontent.com/"))
                    self.assertRegex(s.url, r"/[0-9a-f]{40}/", "github URL must be pinned to a commit SHA")
                else:
                    self.assertEqual(s.kind, "printables")
                    self.assertGreater(s.print_id, 0)
                    self.assertGreater(s.file_id, 0)

    def test_filenames_unique(self) -> None:
        names = [s.filename for s in load_sources()]
        self.assertEqual(len(names), len(set(names)))


class TestVerifySha256(unittest.TestCase):
    def test_verify_sha256_ok(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "model.stl"
            path.write_bytes(b"solid test\nendsolid test\n")
            verify_sha256(path, hashlib.sha256(path.read_bytes()).hexdigest())

    def test_verify_sha256_mismatch_names_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "ArduinoUNO_3D.step"
            path.write_bytes(b"ISO-10303-21;\n")
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            with self.assertRaises(ValueError) as ctx:
                verify_sha256(path, "f" * 64)
            message = str(ctx.exception)
            self.assertIn("ArduinoUNO_3D.step", message)
            self.assertIn("f" * 64, message)
            self.assertIn(actual, message)


class TestMissingAssets(unittest.TestCase):
    def test_missing_assets_lists_absent_files(self) -> None:
        sources = [_fake_source("a", "a.step"), _fake_source("b", "b.stl")]
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(missing_assets(sources, Path(tmp)), ["a.step", "b.stl"])

    def test_missing_assets_skips_present_files(self) -> None:
        sources = [_fake_source("a", "a.step"), _fake_source("b", "b.stl")]
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "a.step").write_bytes(b"x")
            self.assertEqual(missing_assets(sources, Path(tmp)), ["b.stl"])


class TestStepConversion(unittest.TestCase):
    def test_glb_path_for_uses_key(self) -> None:
        source = _fake_source("uno", "ArduinoUNO_3D.step")
        self.assertEqual(glb_path_for(source), CACHE_DIR / "uno.glb")
        self.assertEqual(glb_path_for(source, Path("/c")), Path("/c/uno.glb"))

    def test_needs_conversion_skips_non_step_and_fresh_cache(self) -> None:
        sources = [
            _fake_source("fresh", "fresh.step"),
            _fake_source("stale", "stale.STEP"),
            _fake_source("absent", "absent.stp"),
            _fake_source("mesh", "mesh.stl"),
            _fake_source("obj", "board.obj"),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            vendor, cache = Path(tmp) / "vendor", Path(tmp) / "cache"
            vendor.mkdir()
            cache.mkdir()
            for s in sources:
                (vendor / s.filename).write_bytes(b"x")
                os.utime(vendor / s.filename, (1000, 1000))
            (cache / "fresh.glb").write_bytes(b"glb")
            os.utime(cache / "fresh.glb", (2000, 2000))
            (cache / "stale.glb").write_bytes(b"glb")
            os.utime(cache / "stale.glb", (500, 500))

            pending = needs_conversion(sources, vendor_dir=vendor, cache_dir=cache)
            self.assertEqual([s.key for s in pending], ["stale", "absent"])

    def test_convert_step_module_imports_without_ocp(self) -> None:
        self.assertNotIn("OCP", sys.modules)


class TestGitignore(unittest.TestCase):
    def test_gitignore_excludes_vendor_and_cache(self) -> None:
        lines = Path(REPO_ROOT, ".gitignore").read_text(encoding="utf-8").splitlines()
        stripped = {re.sub(r"\s+#.*$", "", line).strip() for line in lines}
        self.assertIn("assets/3d/vendor/", stripped)
        self.assertIn("assets/3d/cache/", stripped)


if __name__ == "__main__":
    unittest.main()
