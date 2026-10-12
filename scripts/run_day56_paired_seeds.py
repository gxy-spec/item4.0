#!/usr/bin/env python3
"""Run one qualified Day 5-6 scene or its paired A/B/C episodes."""

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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from scenario.configuration import resolve_experiment, validate_online_control_config, validate_schema  # noqa: E402

CONFIG_DIR = ROOT / "configs" / "experiments"
RUNNER = ROOT / "scripts" / "run_stage1_dynamic.py"
PACKAGE = Path("D:/CarlaAir/CarlaAir-v0.1.7-Windows11-x86_64")
LAUNCHER = PACKAGE / "CarlaAir.ps1"
OUTPUT = Path("E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/day56_paired_seeds")


def config_path(seed: int, group: str) -> Path:
    return CONFIG_DIR / f"ci_day56_s{seed}_{group}.yaml"


def frozen_scene(config: dict) -> dict:
    scene = copy.deepcopy(config)
    for key in ("_source_config", "_included_configs", "experiment_id", "acceptance", "output"):
        scene.pop(key, None)
    scene["dynamic"]["persistent_target_communication"].pop("control_policy", None)
    return scene


def preflight(seed: int, groups: list[str]) -> dict[str, dict]:
    configs = {group: resolve_experiment(config_path(seed, group)) for group in groups}
    schema = ROOT / "configs" / "schemas" / "dynamic_experiment_schema.json"
    for group, config in configs.items():
        errors = validate_schema(config, schema) + validate_online_control_config(config)
        if errors:
            raise ValueError(f"{seed}/{group} config invalid: {errors}")
        if config["random_seed"] != seed:
            raise ValueError(f"{seed}/{group}: random seed mismatch")
        policy = "none" if group in {"survey", "A"} else {"B": "ugv_local", "C": "dual"}[group]
        if config["dynamic"]["persistent_target_communication"]["control_policy"] != policy:
            raise ValueError(f"{seed}/{group}: unexpected policy")
        safety = config["dynamic"]["ugv_safety"]
        if not safety["enabled"] or safety["obstacle_source"] != "ugv_rgbd" or config["dynamic"]["uav_ignore_collision"]:
            raise ValueError(f"{seed}/{group}: safety or UAV collision protection bypassed")
    if "A" in configs:
        baseline = frozen_scene(configs["A"])
        for group in "BC":
            if frozen_scene(configs[group]) != baseline:
                raise ValueError(f"{seed}: {group} differs from A beyond decision policy/acceptance/output")
    return configs


def stop_simulator() -> None:
    subprocess.run(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(LAUNCHER), "--kill"],
                   cwd=PACKAGE, check=True, timeout=45)


def run_one(seed: int, group: str, config: dict) -> dict:
    print(f"Starting seed {seed} / {group} in fresh CARLA-Air session", flush=True)
    restart_simulator(PACKAGE, str(config["simulation"]["map"]), f"{seed}_{group}")
    started = datetime.now(timezone.utc)
    command = [sys.executable, "-u", str(RUNNER), "--config", str(config_path(seed, group)), "--timeout", "40"]
    proc = subprocess.run(command, cwd=ROOT, text=True, encoding="utf-8", errors="replace", timeout=7200)
    reports = [p for p in Path(config["output"]["root"]).glob("*/validation_report.json")
               if datetime.fromtimestamp(p.stat().st_mtime, timezone.utc) >= started]
    if not reports:
        raise RuntimeError(f"{seed}/{group}: runner exit {proc.returncode}; no fresh validation report")
    report_path = max(reports, key=lambda p: p.stat().st_mtime)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    result = {"seed": seed, "group": group, "policy": config["dynamic"]["persistent_target_communication"]["control_policy"],
              "started_utc": started.isoformat(), "runner_exit_code": proc.returncode,
              "native_status": report.get("status"), "run_dir": str(report_path.parent), "validation_report": str(report_path)}
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, choices=(1001, 2001, 3001), required=True)
    parser.add_argument("--phase", choices=("survey", "paired"), required=True)
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    groups = ["survey"] if args.phase == "survey" else list("ABC")
    configs = preflight(args.seed, groups)
    if args.phase == "survey" and args.seed == 1001:
        raise ValueError("Seed 1001 was already surveyed on Day 3; use --phase paired")
    if args.phase == "paired" and args.seed != 1001:
        survey = OUTPUT / f"S{args.seed}" / "survey" / "scene_qualification.json"
        if not survey.exists():
            raise RuntimeError(f"Refusing paired run: scene qualification not PASS: {survey}")
        qualification = json.loads(survey.read_text(encoding="utf-8"))
        if qualification.get("status") != "PASS":
            raise RuntimeError(f"Refusing paired run: scene qualification not PASS: {survey}")
        survey_index = json.loads((OUTPUT / f"S{args.seed}" / "survey_index.json").read_text(encoding="utf-8"))
        if len(survey_index) != 1 or qualification.get("run_dir") != survey_index[0].get("run_dir") \
                or survey_index[0].get("native_status") != "PASS":
            raise RuntimeError(f"Paired {args.seed} qualifier does not match its latest successful survey")
        surveyed_run = Path(survey_index[0]["run_dir"])
        # The run snapshot, not the possibly edited current survey YAML, is the
        # frozen evidence. Only duration and the message-expiry policy may
        # differ; none of the geometry, actors, sensors or safety may drift.
        import yaml
        surveyed = yaml.safe_load((surveyed_run / "resolved_config.yaml").read_text(encoding="utf-8"))
        paired = configs["A"]
        for key in ("random_seed", "region", "actors", "sensors", "task", "communication"):
            if surveyed.get(key) != paired.get(key):
                raise RuntimeError(f"Paired {args.seed} {key} no longer matches qualified scene")
        survey_dynamic = copy.deepcopy(surveyed["dynamic"])
        paired_dynamic = copy.deepcopy(paired["dynamic"])
        survey_dynamic.pop("persistent_target_communication", None)
        paired_dynamic.pop("persistent_target_communication", None)
        if survey_dynamic != paired_dynamic:
            raise RuntimeError(f"Paired {args.seed} dynamic scene/safety differs from qualified survey")
    if args.check_only:
        print(f"{args.seed}/{args.phase}: configurations, scene qualification and safety invariant intact", flush=True)
        return 0
    index_path = OUTPUT / f"S{args.seed}" / ("survey_index.json" if args.phase == "survey" else "paired_index.json")
    index_path.parent.mkdir(parents=True, exist_ok=True)
    results = []
    try:
        for group in groups:
            results.append(run_one(args.seed, group, configs[group]))
            index_path.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
            stop_simulator()
            time.sleep(2)
            if results[-1]["native_status"] != "PASS":
                raise RuntimeError(f"{args.seed}/{group} native acceptance failed; inspect {results[-1]['validation_report']}")
    finally:
        stop_simulator()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
