#!/usr/bin/env python3
"""Run paired E7 route-influence and communication-outage experiments."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUNNER = PROJECT_ROOT / "scripts" / "run_stage1_dynamic.py"
TEMPLATE = PROJECT_ROOT / "configs" / "experiments" / "ci_e7_comparison_preflight_20261002.yaml"
OUTPUT_ROOT = Path(r"E:\CarlaAirData\CityInspection_GOC\e7_revised_v2")
CONFIG_ROOT = OUTPUT_ROOT / "generated_configs"
sys.path.insert(0, str(PROJECT_ROOT / "src"))


def restart_simulator(package_root: Path, map_name: str, label: str) -> None:
    """Start every paired run from a clean CARLA-Air/AirSim process state."""
    package_root = package_root.resolve()
    launcher = package_root / "CarlaAir.ps1"
    if not launcher.exists():
        raise FileNotFoundError(f"CARLA-Air Windows launcher not found: {launcher}")
    package = package_root / "WindowsNoEditor"
    if not package.exists():
        raise FileNotFoundError(f"CARLA-Air WindowsNoEditor package not found: {package}")
    command = [
        "powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(launcher),
        map_name, "--port", "2000", "--quality", "Low", "--res", "960x540",
        "--no-traffic", "--package-root", str(package),
    ]
    result = subprocess.run(
        command, cwd=launcher.parent, capture_output=True, text=True,
        encoding="utf-8", errors="replace", timeout=360,
    )
    output = (result.stdout or "") + (result.stderr or "")
    if output.strip():
        print(output.rstrip(), flush=True)
    if result.returncode != 0:
        raise RuntimeError(f"CARLA-Air restart failed before {label} (exit={result.returncode})")
    if "CarlaAir is ready." not in output:
        raise RuntimeError(f"CARLA-Air did not confirm readiness before {label}")


def write_config(config: dict[str, Any], name: str) -> Path:
    CONFIG_ROOT.mkdir(parents=True, exist_ok=True)
    path = CONFIG_ROOT / f"{name}.yaml"
    path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return path


def run_one(config_path: Path, label: str, package_root: Path, map_name: str) -> dict[str, Any]:
    print(f"\n=== START {label} ===", flush=True)
    restart_simulator(package_root, map_name, label)
    process = subprocess.Popen(
        [sys.executable, "-u", str(RUNNER), "--config", str(config_path)],
        cwd=PROJECT_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
    )
    assert process.stdout is not None
    started = time.monotonic()
    last_heartbeat = started
    lines: list[str] = []
    while process.poll() is None:
        line = process.stdout.readline()
        if line:
            print(line.rstrip(), flush=True)
            lines.append(line.rstrip())
        elif time.monotonic() - last_heartbeat >= 20:
            print(f"[{label}] still running ({time.monotonic() - started:.0f}s elapsed)", flush=True)
            last_heartbeat = time.monotonic()
            time.sleep(0.5)
    for line in process.stdout:
        print(line.rstrip(), flush=True)
        lines.append(line.rstrip())
    exit_code = process.wait()
    output_line = next((line for line in reversed(lines) if "output:" in line), "")
    run_dir = output_line.split("output:", 1)[-1].strip() if output_line else ""
    report: dict[str, Any] = {"label": label, "exit_code": exit_code, "run_dir": run_dir}
    if run_dir:
        report_path = Path(run_dir) / "validation_report.json"
        if report_path.exists():
            report["status"] = json.loads(report_path.read_text(encoding="utf-8")).get("status")
    # CARLA-Air can abort in its native cleanup after the Python runner has
    # already written a complete PASS report.  A complete machine-readable
    # report is authoritative for the experiment; preserve the native exit
    # code for audit but do not discard an otherwise valid run.
    if report.get("status") != "PASS":
        raise RuntimeError(
            f"Run failed; preserving output and stopping the batch (label={label}, run_dir={run_dir}, exit={exit_code}, status={report.get('status')})"
        )
    print(f"=== PASS {label}: {run_dir} ===", flush=True)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=[1001, 2001, 3001])
    parser.add_argument("--smoke", action="store_true", help="Run one paired seed and one UGV-local positive control with shortened duration")
    args = parser.parse_args()

    from scenario.configuration import resolve_experiment

    template = resolve_experiment(TEMPLATE)
    template.pop("_source_config", None)
    template.pop("_included_configs", None)
    # Safety is a project-wide invariant: communication comparisons may
    # change route-selection policy, but must never bypass the already
    # validated RGB-D obstacle controller.
    safety_cfg = template.get("dynamic", {}).get("ugv_safety", {})
    if not bool(safety_cfg.get("enabled", False)):
        raise RuntimeError(
            "Refusing E7 batch: dynamic.ugv_safety.enabled must remain true; "
            "safety avoidance cannot be disabled for route comparisons."
        )
    seeds = [args.seeds[0]] if args.smoke else args.seeds
    if args.smoke:
        template["simulation"]["duration_seconds"] = 30.0
        template["acceptance"]["minimum_uav_path_m"] = 0.0
        template["acceptance"]["minimum_ugv_path_m"] = 5.0
    else:
        # These paired runs use a deliberately bounded 35 m UGV patrol segment
        # and a 0.5 m/s fixed UAV search sweep.  The template's generic E6
        # thresholds (100 m UAV path, 30 m UGV path) are incompatible with
        # those frozen scenario lengths, so require traversal of most of the
        # planned patrol and a clearly dynamic UAV path instead.
        template["acceptance"]["minimum_ugv_path_m"] = 25.0
        template["acceptance"]["minimum_uav_path_m"] = 30.0
    # Controlled visual decoys are deliberately non-red. The frozen target is
    # the only red vehicle, so the comparison measures information source and
    # routing rather than accidental red paint on a distractor.
    template.setdefault("actors", {}).setdefault("distractors", {}).update(
        {
            "same_category_wrong_color": 1,
            "same_color_wrong_category": 0,
            "random_vehicles": 4,
            "pedestrians": 0,
        }
    )
    template.setdefault("dynamic", {}).update(
        {
            "comparison_distractor_color_rgb": "0,0,255",
            "distractor_spawn_clearance_from_patrol_m": 12.0,
        }
    )
    template["region"]["target_transform"] = {
        # This surveyed road point is on the UAV search leg and beyond the
        # truncated UGV patrol segment. The positive control uses the same
        # point with the longer regional patrol route.
        "location_xyz": [-39.2600327, -41.0209694, 0.35],
        "rotation_pyr_degrees": [0.0, 0.0, 0.0],
    }
    # The fixed CARLA step is 50 ms. Sampling cameras on every step avoids the
    # independent 10 Hz actors drifting onto adjacent frames during long runs.
    template["sensors"]["frequency_hz"] = 20.0
    template["dynamic"]["local_target_search"]["red_ratio_threshold"] = 0.20
    template["dynamic"]["local_target_search"]["infer_every_n_sensor_frames"] = 10
    template["dynamic"]["maximum_initial_uav_pose_error_m"] = 1.5
    # Keep UAV motion on a fixed, non-hovering search line, but begin the sweep
    # beside the target so perception can send a continuous stream of fresh
    # (1 s TTL) states before the UGV reaches the patrol-route branch.  The
    # previous start at x=-60 only produced a short detection burst followed
    # by multi-second gaps; expiry correctly returned the UGV to patrol before
    # dual-mode routing could diverge from A.
    template["region"]["uav"]["initial_position_xyz"][0] = -45.0
    template["region"]["uav"]["waypoints_xyz"][0][0] = -45.0
    template["region"]["uav"]["speed_mps"] = 0.5
    template["dynamic"]["uav_speed_mps"] = 0.5
    completed: list[dict[str, Any]] = []

    comparison_jobs: list[tuple[str, int, str, str]] = []
    for seed in seeds:
        for group, policy, route_case in (
            ("A_patrol", "none", "off_route"),
            ("B_ugv_local", "ugv_local", "off_route"),
            ("C_dual", "dual", "off_route"),
            ("B_local_positive", "ugv_local", "target_visible"),
        ):
            comparison_jobs.append((group, seed, policy, route_case))

    for group, seed, policy, route_case in comparison_jobs:
        config = json.loads(json.dumps(template))
        config["random_seed"] = seed
        group_token = {"A_patrol": "A", "B_ugv_local": "B", "C_dual": "C", "B_local_positive": "BP"}[group]
        config["experiment_id"] = f"CI_E7_ROUTE_COMPARE_{group_token}_T10_S{seed}_20260929"
        output_root = OUTPUT_ROOT / ("smoke" if args.smoke else "formal") / group
        config["output"]["root"] = str(output_root)
        if route_case == "target_visible":
            # Positive control: use the surveyed regional route that passes the
            # target area. This checks that B can react when its own RGB-D view
            # actually observes the target.
            config["region"]["target_transform"] = {
                "location_xyz": [-39.2600327, -41.0209694, 0.35],
                # Present the van broadside to the approaching UGV so the
                # local-positive-control tests a real vehicle body, not a
                # tiny rear-view red patch.
                "rotation_pyr_degrees": [0.0, 0.0, 0.0],
            }
            config["dynamic"].pop("ugv_patrol_route_target_spawn_point_id", None)
            config["dynamic"].pop("ugv_patrol_route_limit_m", None)
            if args.smoke:
                # The surveyed route reaches the target region later than the
                # off-route 55 m segment, so keep enough time to exercise the
                # local-positive-control behavior.
                config["simulation"]["duration_seconds"] = 60.0
        else:
            # Paired A/B/C scene: identical road-graph patrol segment that stays
            # away from the target. Only C should receive useful UAV evidence.
            config["dynamic"]["ugv_patrol_route_target_spawn_point_id"] = 0
            config["dynamic"]["ugv_patrol_route_limit_m"] = 35.0
        communication = config["dynamic"]["persistent_target_communication"]
        communication.update(
            {
                "enabled": True,
                "control_policy": policy,
                "comparison_case": route_case,
                "minimum_depth_reliability_for_navigation_by_device": {
                    "uav": 0.15,
                    "ugv": 0.25,
                },
                "expiry_policy": "return_to_patrol",
                "outage_intervals_s": [],
                "require_cross_device_consensus": False,
            }
        )
        acceptance = config["acceptance"]
        acceptance["minimum_target_routes_planned"] = 1 if policy == "dual" or route_case == "target_visible" else 0
        acceptance["maximum_target_routes_planned"] = 0 if route_case == "off_route" and policy != "dual" else None
        acceptance["minimum_uav_target_routes_planned"] = 1 if policy == "dual" else 0
        acceptance["minimum_ugv_target_routes_planned"] = 1 if route_case == "target_visible" else 0
        acceptance["minimum_target_route_follow_ticks"] = 10 if acceptance["minimum_target_routes_planned"] else 0
        # This target lies outside the UGV sensor's observed route in the
        # off-route ABC scene, so dual mode must have at least one UAV update;
        # UGV updates are optional there. The positive control instead proves
        # that UGV-only evidence can trigger an approach.
        acceptance["minimum_updates_per_device"] = 0
        acceptance["minimum_uav_updates"] = 1 if policy == "dual" else 0
        acceptance["minimum_ugv_updates"] = 0
        acceptance["require_message_delivery"] = policy == "dual" or route_case == "target_visible"
        acceptance["require_expiry_safe_hold"] = False
        name = f"{group}_S{seed}"
        config_path = write_config(config, name)
        completed.append(
            run_one(
                config_path,
                name,
                Path(config["simulation"]["package_root"]),
                str(config["simulation"]["map"]),
            )
        )
        # Write the index incrementally. An interrupted run then cannot leave a
        # stale index that accidentally mixes prior and revised batches.
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        (OUTPUT_ROOT / ("smoke_run_index.json" if args.smoke else "batch_run_index.json")).write_text(
            json.dumps(completed, indent=2), encoding="utf-8"
        )

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    summary_path = OUTPUT_ROOT / ("smoke_run_index.json" if args.smoke else "batch_run_index.json")
    summary_path.write_text(json.dumps(completed, indent=2), encoding="utf-8")
    print(f"\nBatch completed: {len(completed)} runs; index={summary_path}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"BATCH STOPPED: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
