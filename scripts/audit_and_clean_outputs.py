#!/usr/bin/env python3
"""Audit experiment outputs and remove only explicitly classified artifacts.

Dry-run is the default.  FAIL/ERROR run directories are discovered from their
own validation_report.json.  Whole obsolete batches require an explicit direct
child name, preventing a broad or computed path from escaping the data root.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path


DEFAULT_ROOT = Path(r"E:\CarlaAirData\CityInspection_GOC")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--obsolete-batch", action="append", default=[])
    parser.add_argument("--apply", action="store_true")
    return parser.parse_args()


def strict_child(path: Path, root: Path) -> Path:
    resolved = path.resolve()
    if resolved == root or root not in resolved.parents:
        raise RuntimeError(f"Refusing path outside data root: {resolved}")
    return resolved


def directory_bytes(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def main() -> int:
    args = parse_args()
    root = args.data_root.resolve()
    if not root.is_dir():
        raise RuntimeError(f"Data root does not exist: {root}")
    targets: dict[Path, dict] = {}
    for report_path in root.rglob("validation_report.json"):
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        status = str(report.get("status", "UNKNOWN")).upper()
        if status not in {"FAIL", "ERROR"}:
            continue
        run_dir = strict_child(report_path.parent, root)
        targets[run_dir] = {
            "reason": f"validation_status_{status.lower()}",
            "validation_status": status,
            "validation_report_sha256": hashlib.sha256(report_path.read_bytes()).hexdigest(),
            "recorded_error": str(report.get("error", ""))[:500],
        }
    for name in args.obsolete_batch:
        if not name or Path(name).name != name or name in {".", ".."}:
            raise RuntimeError(f"Obsolete batch must be a direct child name: {name!r}")
        batch = strict_child(root / name, root)
        if batch.is_dir():
            targets[batch] = {
                "reason": "explicitly_obsolete_superseded_batch",
                "validation_status": "MIXED_OR_HISTORICAL",
                "validation_report_sha256": None,
                "recorded_error": "",
            }
    # A selected whole batch subsumes any failed run beneath it.
    selected: list[Path] = []
    for path in sorted(targets, key=lambda item: (len(item.parts), str(item).lower())):
        if any(parent == path or parent in path.parents for parent in selected):
            continue
        selected.append(path)
    records = []
    for path in selected:
        metadata = targets[path]
        records.append({
            "path": str(path),
            "relative_path": str(path.relative_to(root)),
            "bytes": directory_bytes(path),
            **metadata,
        })
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "mode": "apply" if args.apply else "dry_run",
        "data_root": str(root),
        "selection_rule": "FAIL/ERROR validation runs plus explicitly named obsolete batches",
        "target_count": len(records),
        "total_bytes": sum(item["bytes"] for item in records),
        "targets": records,
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: manifest[key] for key in ("mode", "target_count", "total_bytes")}, indent=2))
    if not args.apply:
        return 0
    for record in records:
        target = strict_child(Path(record["path"]), root)
        if target.is_dir():
            shutil.rmtree(target)
    # Remove newly empty directories, bottom-up, but never the data root.
    for directory in sorted((item for item in root.rglob("*") if item.is_dir()),
                            key=lambda item: len(item.parts), reverse=True):
        try:
            directory.rmdir()
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
