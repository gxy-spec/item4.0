#!/usr/bin/env python3
"""Run the formal perception collection configs with restart and resume support."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from run_e7_comparison_batch import restart_simulator  # noqa: E402

RUNNER = ROOT / "scripts" / "run_stage1_dynamic.py"
DEFAULT_CONFIGS = ROOT / "configs" / "experiments" / "perception_collection_town10hd_v1"
DEFAULT_INDEX = Path(r"E:\CarlaAirData\CityInspection_GOC\perception_collection_town10hd_v1\batch_index.json")


def stop_simulator(package_root: Path) -> None:
    subprocess.run(
        ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(package_root / "CarlaAir.ps1"), "--kill"],
        cwd=package_root,
        check=False,
        timeout=60,
        capture_output=True,
    )


def newest_report(output_root: Path, started: datetime) -> Path | None:
    candidates = [
        path for path in output_root.glob("*/validation_report.json")
        if datetime.fromtimestamp(path.stat().st_mtime, timezone.utc) >= started
    ]
    return max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-dir", type=Path, default=DEFAULT_CONFIGS)
    parser.add_argument("--index", type=Path, default=DEFAULT_INDEX)
    parser.add_argument("--cases", nargs="*", help="Optional config stems to run")
    parser.add_argument("--resume", action="store_true", help="Skip cases already recorded as PASS")
    parser.add_argument("--continue-on-error", action="store_true")
    args = parser.parse_args()

    configs = sorted(args.config_dir.glob("*.yaml"))
    if args.cases:
        wanted = set(args.cases)
        configs = [path for path in configs if path.stem in wanted]
        missing = wanted - {path.stem for path in configs}
        if missing:
            raise ValueError(f"Unknown cases: {sorted(missing)}")
    if not configs:
        raise RuntimeError(f"No collection configs found in {args.config_dir}")

    args.index.parent.mkdir(parents=True, exist_ok=True)
    log_dir = args.index.parent / "batch_logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    results = json.loads(args.index.read_text(encoding="utf-8")) if args.index.exists() else {}
    failures = 0
    for number, config_path in enumerate(configs, start=1):
        case = config_path.stem
        if args.resume and results.get(case, {}).get("status") == "PASS":
            print(f"[{number:02d}/{len(configs):02d}] SKIP {case} (recorded PASS)", flush=True)
            continue
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        package = Path(config["simulation"]["package_root"])
        map_name = str(config["simulation"]["map"])
        output_root = Path(config["output"]["root"])
        log_path = log_dir / f"{case}.log"
        started = datetime.now(timezone.utc)
        print(f"[{number:02d}/{len(configs):02d}] START {case}", flush=True)
        result: dict[str, object]
        try:
            restart_simulator(package, map_name, case)
            with log_path.open("w", encoding="utf-8") as log:
                process = subprocess.run(
                    [sys.executable, "-u", str(RUNNER), "--config", str(config_path)],
                    cwd=ROOT,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=1800,
                )
            report_path = newest_report(output_root, started)
            if report_path is None:
                raise RuntimeError(f"runner exit={process.returncode}; no fresh validation report")
            report = json.loads(report_path.read_text(encoding="utf-8"))
            result = {
                "case": case,
                "status": report.get("status"),
                "runner_exit_code": process.returncode,
                "run_dir": str(report_path.parent),
                "validation_report": str(report_path),
                "log": str(log_path),
                "started_utc": started.isoformat(),
            }
            if result["status"] != "PASS":
                raise RuntimeError(f"validation status={result['status']}")
        except Exception as exc:
            failures += 1
            result = {
                "case": case,
                "status": "ERROR",
                "error": repr(exc),
                "log": str(log_path),
                "started_utc": started.isoformat(),
            }
        finally:
            stop_simulator(package)
        results[case] = result
        args.index.write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"[{number:02d}/{len(configs):02d}] {result['status']} {case}", flush=True)
        if result["status"] != "PASS" and not args.continue_on_error:
            return 1

    passed = sum(row.get("status") == "PASS" for row in results.values())
    print(f"BATCH COMPLETE: {passed} PASS, {failures} new failures; index={args.index}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
