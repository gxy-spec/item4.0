#!/usr/bin/env python3
"""Run independently restarted short A-policy admission episodes, with an index."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from run_e7_comparison_batch import restart_simulator  # noqa: E402

INDEX = ROOT / "configs/experiments/multiscene_admission_20261009/frozen_admission_index.json"
PACKAGE = Path("D:/CarlaAir/CarlaAir-v0.1.7-Windows11-x86_64")
RUNNER = ROOT / "scripts/run_stage1_dynamic.py"
DATA = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_admission_20261009")


def stop_simulator() -> None:
    subprocess.run(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
                    "-File", str(PACKAGE / "CarlaAir.ps1"), "--kill"],
                   cwd=PACKAGE, check=True, timeout=50, capture_output=True)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cases", nargs="+", default=None)
    args = p.parse_args()
    frozen = json.loads(INDEX.read_text(encoding="utf-8"))
    entries = {row["case"]: row for row in frozen["configurations"]}
    wanted = args.cases or list(entries)
    if any(case not in entries for case in wanted):
        raise ValueError(f"Unknown case(s): {set(wanted) - set(entries)}")
    DATA.mkdir(parents=True, exist_ok=True)
    index_path = DATA / "short_smoke_index.json"
    results = json.loads(index_path.read_text(encoding="utf-8")) if index_path.exists() else {}
    try:
        for case in wanted:
            row = entries[case]
            config = yaml.safe_load(Path(row["config"]).read_text(encoding="utf-8"))
            print(f"=== {case} ===", flush=True)
            restart_simulator(PACKAGE, "Town10HD", case)
            started = datetime.now(timezone.utc)
            command = [sys.executable, "-u", str(RUNNER), "--config", row["config"], "--timeout", "40"]
            log_path = DATA / f"{case}_runner.log"
            try:
                with log_path.open("w", encoding="utf-8") as log:
                    process = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                             timeout=1800)
                candidates = [path for path in Path(config["output"]["root"]).glob("*/validation_report.json")
                              if datetime.fromtimestamp(path.stat().st_mtime, timezone.utc) >= started]
                if not candidates:
                    raise RuntimeError(f"Runner exit {process.returncode}, no fresh validation_report.json")
                report_path = max(candidates, key=lambda path: path.stat().st_mtime)
                report = json.loads(report_path.read_text(encoding="utf-8"))
                result = {"case": case, "status": report.get("status"), "runner_exit_code": process.returncode,
                          "validation_report": str(report_path), "run_dir": str(report_path.parent),
                          "log": str(log_path), "started_utc": started.isoformat()}
            except Exception as exc:
                result = {"case": case, "status": "ERROR", "error": repr(exc), "log": str(log_path),
                          "started_utc": started.isoformat()}
            results[case] = result
            index_path.write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(json.dumps(result, ensure_ascii=False), flush=True)
            stop_simulator()
            if result["status"] != "PASS":
                print(f"{case} did not pass; stopping batch for diagnosis", flush=True)
                return 1
    finally:
        stop_simulator()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
