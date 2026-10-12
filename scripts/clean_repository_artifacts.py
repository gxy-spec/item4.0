#!/usr/bin/env python3
"""Remove reproducible caches and explicitly superseded repository artifacts.

Dry-run is the default.  The allow-list is intentionally repository-relative;
resolved paths must remain strict children of the detected project root.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OBSOLETE = (
    Path("configs/experiments/multiscene_abc_formal_20261010"),
)


def strict_child(path: Path) -> Path:
    resolved = path.resolve()
    if resolved == ROOT or ROOT not in resolved.parents:
        raise RuntimeError(f"Refusing path outside repository: {resolved}")
    return resolved


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    candidates = [strict_child(ROOT / item) for item in OBSOLETE]
    candidates.extend(
        strict_child(path) for path in ROOT.rglob("*")
        if path.is_dir() and path.name in {"__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache"}
    )
    selected: list[Path] = []
    for path in sorted(set(candidates), key=lambda item: (len(item.parts), str(item).lower())):
        if not path.exists() or any(parent in path.parents for parent in selected):
            continue
        selected.append(path)
    print(f"mode={'apply' if args.apply else 'dry_run'} targets={len(selected)}")
    for path in selected:
        print(path)
    if args.apply:
        for path in selected:
            shutil.rmtree(strict_child(path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
