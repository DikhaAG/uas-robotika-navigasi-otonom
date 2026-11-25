#!/usr/bin/env python3
"""
Deterministic Python standards checker for tools/ and tests/.
Enforces explicit typing on public module/class interfaces and CalibrationEngine seam isolation.
"""

import ast
import pathlib
import sys
from typing import List, Set

BANNED_ENGINE_IMPORTS: Set[str] = {
    "MockRobotSerial",
    "SerialTransportAdapter",
    "ConsoleObservationAdapter",
    "ScriptedObservationAdapter",
    "AnsiConsoleReporter",
    "CalibrationLogger",
}


def _check_function_typing(filepath: pathlib.Path, fn: ast.FunctionDef, errors: List[str]) -> None:
    if fn.name.startswith("_") and fn.name not in ("__init__", "__repr__"):
        return

    missing: List[str] = []
    if fn.returns is None:
        missing.append("return")
    for arg in fn.args.args + fn.args.kwonlyargs:
        if arg.arg not in ("self", "cls") and arg.annotation is None:
            missing.append(arg.arg)
    if fn.args.vararg and fn.args.vararg.annotation is None:
        missing.append(f"*{fn.args.vararg.arg}")
    if fn.args.kwarg and fn.args.kwarg.annotation is None:
        missing.append(f"**{fn.args.kwarg.arg}")

    if missing:
        errors.append(f"{filepath}:{fn.lineno}: public function '{fn.name}' missing annotations for {', '.join(missing)}")


def check_file(filepath: pathlib.Path) -> List[str]:
    """Validates explicit typing and seam isolation rules for a single Python file."""
    errors: List[str] = []
    source = filepath.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(filepath))

    for node in tree.body:
        if isinstance(node, ast.FunctionDef):
            _check_function_typing(filepath, node, errors)
        elif isinstance(node, ast.ClassDef):
            for item in node.body:
                if isinstance(item, ast.FunctionDef):
                    _check_function_typing(filepath, item, errors)

    if filepath.as_posix().endswith("tools/calibration/engine.py"):
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                for alias in node.names:
                    if alias.name in BANNED_ENGINE_IMPORTS:
                        errors.append(
                            f"{filepath}:{node.lineno}: CalibrationEngine must depend on Protocol seams, not '{alias.name}'"
                        )

    return errors


def main() -> int:
    """Runs AST standards checks across all Python files in tools/ and tests/."""
    py_files = sorted(pathlib.Path("tools").rglob("*.py")) + sorted(pathlib.Path("tests").rglob("*.py"))
    all_errors: List[str] = []
    for path in py_files:
        all_errors.extend(check_file(path))

    if all_errors:
        for err in all_errors:
            print(f"ERROR: {err}", file=sys.stderr)
        return 1

    print(f"  ✓ Python AST typing & seam isolation checks passed ({len(py_files)} files).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
