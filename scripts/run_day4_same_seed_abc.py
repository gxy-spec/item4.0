#!/usr/bin/env python3
"""Run the Day 4 paired A/B/C episodes from fresh CARLA-Air sessions."""

from __future__ import annotations

import argparse
import copy
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from run_e7_comparison_batch import restart_simulator

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))
from scenario.configuration import resolve_experiment, validate_online_control_config, validate_schema  # noqa: E402

CONFIGS = {
    group: PROJECT_ROOT / "configs" / "experiments" / f"ci_day4_same_seed_{group}.yaml"
    for group in "ABC"
}
RUNNER = PROJECT_ROOT / "scripts" / "run_stage1_dynamic.py"
OUTPUT_ROOT = Path("E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/day4_same_seed_abc/S1001")
PACKAGE_ROOT = Path("D:/CarlaAir/CarlaAir-v0.1.7-Windows11-x86_64")
LAUNCHER = PACKAGE_ROOT / "CarlaAir.ps1"


def frozen_scene(config: dict) -> dict:
    result = copy.deepcopy(config)
    for key in ("_source_config", "_included_configs", "experiment_id", "acceptance", "output"):
        result.pop(key, None)
    result["dynamic"]["persistent_target_communication"].pop("control_policy", None)
    return result


def check_configs() -> dict[str, dict]:
    configs = {group: resolve_experiment(path) for group, path in CONFIGS.items()}
    schema = PROJECT_ROOT / "configs" / "schemas" / "dynamic_experiment_schema.json"
    for group, config in configs.items():
        errors = validate_schema(config, schema) + validate_online_control_config(config)
        if errors:
            raise ValueError(f"{group} config invalid: {errors}")
        if config["random_seed"] != 1001:
            raise ValueError(f"{group} seed is not 1001")
        expected_policy = {"A": "none", "B": "ugv_local", "C": "dual"}[group]
        if config["dynamic"]["persistent_target_communication"]["control_policy"] != expected_policy:
            raise ValueError(f"{group} policy differs from {expected_policy}")
    baseline = frozen_scene(configs["A"])
    for group in "BC":
        if frozen_scene(configs[group]) != baseline:
            raise ValueError(f"{group} scene differs from A beyond decision source and acceptance")
    print("Day 4 preflight: A/B/C share seed, scene, sensors, communication and RGB-D safety.", flush=True)
    return configs


def stop_simulator() -> None:
    subprocess.run(
        ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(LAUNCHER), "--kill"],
        cwd=PACKAGE_ROOT, check=True, timeout=45,
    )


def run_episode(group: str, config: dict) -> dict:
    print(f"\n=== Day 4 group {group}: starting CARLA-Air ===", flush=True)
    restart_simulator(PACKAGE_ROOT, str(config["simulation"]["map"]), group)
    started = datetime.now(timezone.utc).isoformat()
    command = [sys.executable, "-u", str(RUNNER), "--config", str(CONFIGS[group]), "--timeout", "40"]
    proc = subprocess.run(command, cwd=PROJECT_ROOT, text=True, encoding="utf-8", errors="replace", timeout=7200)
    runs = sorted(Path(config["output"]["root"]).glob("*/validation_report.json"), key=lambda p: p.stat().st_mtime)
    if not runs:
        raise RuntimeError(f"Group {group} returned {proc.returncode}, no validation report in {config['output']['root']}")
    report_path = runs[-1]
    report = json.loads(report_path.read_text(encoding="utf-8"))
    result = {
        "group": group, "seed": config["random_seed"], "policy": config["dynamic"]["persistent_target_communication"]["control_policy"],
        "started_at_utc": started, "runner_exit_code": proc.returncode,
        "status": report.get("status", "UNKNOWN"), "run_dir": str(report_path.parent),
        "validation_report": str(report_path),
    }
    print(f"=== Day 4 group {group}: {result['status']} | {result['run_dir']} ===", flush=True)
    if report.get("status") != "PASS":
        raise RuntimeError(f"Group {group} failed acceptance: {report_path}")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--groups", nargs="+", choices=list("ABC"), default=list("ABC"))
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    configs = check_configs()
    if args.check_only:
        return 0
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    results = []
    try:
        for group in args.groups:
            results.append(run_episode(group, configs[group]))
            (OUTPUT_ROOT / "day4_batch_index.json").write_text(
                json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            stop_simulator()
            time.sleep(2)
    finally:
        stop_simulator()
    print(f"Completed {len(results)} Day 4 episodes: {OUTPUT_ROOT / 'day4_batch_index.json'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
