#!/usr/bin/env python3
"""Run the frozen 36-run multi-scene A/B/C batch with per-run audit records."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG_INDEX = ROOT / "configs" / "experiments" / "multiscene_abc_formal_20261010_v2" / "formal_36_run_index.json"
PACKAGE = Path(r"D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64")
LAUNCHER = PACKAGE / "CarlaAir.ps1"
RUNNER = ROOT / "scripts" / "run_stage1_dynamic.py"
DATA = Path(r"E:\CarlaAirData\CityInspection_GOC\multiscene_abc_formal_20261010_v2")
PROGRESS = DATA / "batch_progress.json"
LOG_ROOT = DATA / "runner_logs"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def stop_simulator() -> None:
    result = subprocess.run(
        ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(LAUNCHER), "--kill"],
        cwd=PACKAGE, text=True, encoding="utf-8", errors="replace", capture_output=True, timeout=60,
    )
    if result.stdout.strip():
        print(result.stdout.strip(), flush=True)
    if result.returncode:
        raise RuntimeError(f"CARLA-Air stop failed: {result.stderr.strip()}")


def read_report(run_dir: Path) -> dict[str, Any] | None:
    path = run_dir / "validation_report.json"
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def launch_run(item: dict[str, Any]) -> dict[str, Any]:
    label = f"{item['case']}_{item['group']}"
    config_path = Path(item["config"])
    expected_experiment_id = str(yaml.safe_load(config_path.read_text(encoding="utf-8"))["experiment_id"])
    log_path = LOG_ROOT / f"{label}.log"
    print(f"START {label} seed={item['seed']} duration={item['horizon_seconds']}s", flush=True)
    stop_simulator()
    from run_e7_comparison_batch import restart_simulator

    restart_simulator(PACKAGE, "Town10HD", label)
    output_root = Path(item["output_root"])
    existing_runs = {path.resolve() for path in output_root.iterdir() if path.is_dir()} if output_root.is_dir() else set()
    start_utc = utc_now()
    with log_path.open("w", encoding="utf-8", newline="\n") as log:
        process = subprocess.Popen(
            [sys.executable, "-u", str(RUNNER), "--config", str(config_path), "--timeout", "3600"],
            cwd=ROOT,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
        )
        next_heartbeat = time.monotonic() + 20.0
        while process.poll() is None:
            time.sleep(3.0)
            if time.monotonic() >= next_heartbeat:
                print(f"RUNNING {label}; log={log_path}", flush=True)
                next_heartbeat = time.monotonic() + 20.0
        exit_code = process.wait()
    fresh = []
    for path in output_root.iterdir() if output_root.is_dir() else []:
        if not path.is_dir() or path.resolve() in existing_runs:
            continue
        report = read_report(path)
        if report is None:
            continue
        try:
            report_started = datetime.fromisoformat(str(report["started_at"]).replace("Z", "+00:00"))
            belongs_to_run = (
                report_started >= datetime.fromisoformat(start_utc)
                and Path(str(report["run_directory"])).resolve() == path.resolve()
                and report.get("experiment_id") == expected_experiment_id
            )
        except (KeyError, TypeError, ValueError):
            belongs_to_run = False
        if belongs_to_run:
            fresh.append(path)
    if not fresh:
        return {
            "label": label,
            "case": item["case"],
            "group": item["group"],
            "seed": item["seed"],
            "started_utc": start_utc,
            "runner_exit_code": exit_code,
            "status": "MISSING_REPORT",
            "log": str(log_path),
        }
    run_dir = max(fresh, key=lambda path: (path / "validation_report.json").stat().st_mtime)
    report = read_report(run_dir) or {}
    return {
        "label": label,
        "case": item["case"],
        "scene": item["scene"],
        "group": item["group"],
        "policy": item["policy"],
        "seed": item["seed"],
        "started_utc": start_utc,
        "completed_utc": utc_now(),
        "runner_exit_code": exit_code,
        "status": report.get("status", "UNKNOWN"),
        "run_dir": str(run_dir),
        "validation_report": str(run_dir / "validation_report.json"),
        "log": str(log_path),
        "summary": report.get("summary", {}),
        "failed_checks": [
            {"name": c.get("name"), "value": c.get("value"), "criterion": c.get("criterion")}
            for c in report.get("checks", []) if not c.get("passed", False)
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=Path, default=CONFIG_INDEX, help="frozen 36-run index")
    parser.add_argument("--data-root", type=Path, default=DATA, help="new output/progress/log root")
    parser.add_argument("--limit", type=int, default=0, help="run at most this many pending episodes")
    parser.add_argument("--case", action="append", default=[], help="restrict to case(s), e.g. N1 or E2")
    parser.add_argument("--group", action="append", default=[], help="restrict to group(s): A, B, or C")
    parser.add_argument(
        "--resume-errors", action="store_true",
        help="allow a preserved technical ERROR run to be retried after its cause is fixed",
    )
    parser.add_argument(
        "--rerun", action="store_true",
        help="rerun all selected cases, including cases previously marked PASS or FAIL",
    )
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    config_index = args.index.resolve()
    data_root = args.data_root.resolve()
    progress_path = data_root / "batch_progress.json"
    log_root = data_root / "runner_logs"
    index = json.loads(config_index.read_text(encoding="utf-8"))
    data_root.mkdir(parents=True, exist_ok=True)
    log_root.mkdir(parents=True, exist_ok=True)
    global PROGRESS, LOG_ROOT
    PROGRESS = progress_path
    LOG_ROOT = log_root
    selected = [
        item for item in index["runs"]
        if (not args.case or item["case"] in args.case)
        and (not args.group or item["group"] in args.group)
    ]
    if args.check_only:
        print(f"Validated frozen formal plan: {len(selected)} episode configs", flush=True)
        return 0
    state = json.loads(PROGRESS.read_text(encoding="utf-8")) if PROGRESS.exists() else {
        "experiment": index["experiment"],
        "index": str(config_index),
        "started_utc": utc_now(),
        "planned_runs": len(index["runs"]),
        "status": "RUNNING",
        "results": [],
    }
    previous = {result["label"]: result for result in state.get("results", [])}
    pending = (
        selected
        if args.rerun
        else [
            item for item in selected
            if previous.get(f"{item['case']}_{item['group']}", {}).get("status") != "PASS"
        ]
    )
    failed_previously = [
        r for r in state.get("results", [])
        if r.get("status") not in ({"PASS", "RUNNING", "ERROR"} if args.resume_errors else {"PASS", "RUNNING"})
    ]
    if failed_previously and not args.rerun:
        raise RuntimeError("A previous run failed; inspect and explicitly resolve it before resuming: " + ", ".join(r["label"] for r in failed_previously))
    if args.limit > 0:
        pending = pending[:args.limit]
    state["status"] = "RUNNING"
    write_json(PROGRESS, state)
    try:
        for item in pending:
            result = launch_run(item)
            state["results"] = [r for r in state["results"] if r.get("label") != result["label"]]
            state["results"].append(result)
            state["completed_runs"] = sum(r.get("status") == "PASS" for r in state["results"])
            state["updated_utc"] = utc_now()
            write_json(PROGRESS, state)
            print(f"RESULT {result['label']} {result['status']} ({state['completed_runs']}/{len(index['runs'])})", flush=True)
            stop_simulator()
            if result["status"] != "PASS":
                state["status"] = "PAUSED_ON_FAILURE"
                write_json(PROGRESS, state)
                raise RuntimeError(f"Formal episode failed: {result['label']} {result['status']}; preserved at {result.get('run_dir', result.get('log'))}")
            time.sleep(2.0)
    finally:
        try:
            stop_simulator()
        except Exception as exc:  # preserve the primary run failure
            print(f"Simulator cleanup warning: {exc}", flush=True)
    total = sum(r.get("status") == "PASS" for r in state.get("results", []))
    if total == len(index["runs"]):
        state["status"] = "COMPLETE"
        state["completed_utc"] = utc_now()
    else:
        state["status"] = "RUNNING"
    write_json(PROGRESS, state)
    print(f"Batch status: {state['status']} ({total}/{len(index['runs'])})", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
