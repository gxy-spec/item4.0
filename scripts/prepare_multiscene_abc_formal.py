#!/usr/bin/env python3
"""Create frozen paired A/B/C configs for the 12 reviewed Town10HD instances."""

from __future__ import annotations

import copy
import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from scenario.configuration import validate_online_control_config, validate_schema  # noqa: E402

SOURCE_DIR = ROOT / "configs" / "experiments" / "multiscene_admission_20261009"
OUT_DIR = ROOT / "configs" / "experiments" / "multiscene_abc_formal_20261010_v2"
DATA_ROOT = Path(r"E:\CarlaAirData\CityInspection_GOC\multiscene_abc_formal_20261010_v2")
SCENE_HORIZON = {"north": 180.0, "east": 190.0, "west": 210.0, "whole": 450.0}
POLICY = {"A": "none", "B": "ugv_local", "C": "dual"}


def load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def paired_scene_view(cfg: dict[str, Any]) -> dict[str, Any]:
    """Fields required to remain identical within each A/B/C triplet."""
    result = copy.deepcopy(cfg)
    for key in ("experiment_id", "acceptance", "output"):
        result.pop(key, None)
    result["dynamic"]["persistent_target_communication"].pop("control_policy", None)
    return result


def first_differences(left: Any, right: Any, prefix: str = "") -> list[str]:
    if isinstance(left, dict) and isinstance(right, dict):
        keys = sorted(set(left) | set(right))
        result: list[str] = []
        for key in keys:
            result.extend(first_differences(left.get(key), right.get(key), f"{prefix}.{key}" if prefix else key))
            if len(result) >= 12:
                break
        return result
    if isinstance(left, list) and isinstance(right, list):
        if len(left) != len(right):
            return [f"{prefix}: list length {len(left)} != {len(right)}"]
        for index, (a, b) in enumerate(zip(left, right)):
            result = first_differences(a, b, f"{prefix}[{index}]")
            if result:
                return result
        return []
    return [] if left == right else [f"{prefix}: {left!r} != {right!r}"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--data-root", type=Path, default=DATA_ROOT)
    parser.add_argument("--expiry-policy", choices=("return_to_patrol", "coast_then_patrol"), default="return_to_patrol")
    parser.add_argument("--stale-grace-s", type=float, default=5.0)
    parser.add_argument("--stale-speed-fraction", type=float, default=0.5)
    parser.add_argument("--stale-max-distance-m", type=float, default=8.0)
    args = parser.parse_args()
    output_dir = args.output_dir.resolve()
    data_root = args.data_root.resolve()
    if args.expiry_policy == "coast_then_patrol" and (
        not 0.0 < args.stale_grace_s <= 30.0
        or not 0.0 < args.stale_speed_fraction <= 1.0
        or not 0.0 < args.stale_max_distance_m <= 50.0
    ):
        parser.error("coast_then_patrol requires positive bounded grace, speed fraction, and distance")
    frozen = json.loads((SOURCE_DIR / "frozen_admission_index.json").read_text(encoding="utf-8"))
    admission_digest = hashlib.sha256(
        (SOURCE_DIR / "frozen_admission_index.json").read_bytes()
    ).hexdigest()
    schema = ROOT / "configs" / "schemas" / "dynamic_experiment_schema.json"
    output_dir.mkdir(parents=True, exist_ok=True)
    entries: list[dict[str, Any]] = []

    for item in frozen["configurations"]:
        case = str(item["case"])
        scene = str(item["scene"])
        seed = int(item["random_seed"])
        base = load_yaml(Path(item["config"]))
        horizon = SCENE_HORIZON[scene]
        route_length = float(item["uav_route_length_m"])
        required_uav_path = round(route_length * 0.95, 3)

        triplet: dict[str, dict[str, Any]] = {}
        for group, policy in POLICY.items():
            cfg = copy.deepcopy(base)
            cfg["experiment_id"] = f"CI_E7_MULTISCENE_FORMAL_{case}_{group}"
            cfg["simulation"]["duration_seconds"] = horizon
            cfg["task"]["max_duration_seconds"] = horizon
            cfg["dynamic"]["persistent_target_communication"]["control_policy"] = policy
            # TTL governs message freshness. A bounded coast preserves task
            # continuity while local RGB-D sensing tries to reacquire the
            # target, then rejoins the patrol route from forward progress.
            communication = cfg["dynamic"]["persistent_target_communication"]
            communication["expiry_policy"] = args.expiry_policy
            if args.expiry_policy == "coast_then_patrol":
                communication["stale_grace_s"] = args.stale_grace_s
                communication["stale_speed_fraction"] = args.stale_speed_fraction
                communication["stale_max_distance_m"] = args.stale_max_distance_m
            else:
                communication.pop("stale_grace_s", None)
                communication.pop("stale_speed_fraction", None)
                communication.pop("stale_max_distance_m", None)
            # Formal runs preserve RGB-D safety and compact archival: retain
            # every 100th frame and all synchronized metadata/trajectories.
            cfg["dynamic"]["save_all_samples"] = False
            cfg["dynamic"]["sensor_save_every_n_frames"] = 100
            cfg["dynamic"]["ugv_safety"]["enabled"] = True
            cfg["dynamic"]["ugv_safety"]["obstacle_source"] = "ugv_rgbd"
            cfg["dynamic"]["ugv_safety"]["target_stop_enabled"] = False
            cfg["dynamic"]["ugv_safety"]["test_sensor_stale_interval_s"] = None
            cfg["dynamic"]["uav_ignore_collision"] = False
            cfg["dynamic"]["background_traffic_safety"] = {
                "ignore_lights_percentage": 0.0,
                "ignore_signs_percentage": 0.0,
                "ignore_walkers_percentage": 0.0,
                "auto_lane_change": False,
                "minimum_leading_distance_m": 6.0,
                "desired_speed_kmh": 22.0,
            }
            cfg["acceptance"].update({
                "minimum_common_frame_ratio": 0.98,
                "minimum_ugv_rgbd_pair_ratio": 0.95,
                "minimum_ugv_path_m": 0.5,
                "minimum_ugv_path_first_2s_m": 0.5,
                "minimum_safety_tick_coverage": 0.95,
                "minimum_uav_path_m": required_uav_path,
                "maximum_ugv_collisions": 0,
                "maximum_uav_collisions": 0,
                "maximum_target_drift_m": 0.2,
                "maximum_uav_tracking_p95_m": 0.5,
                "minimum_moving_vehicles": 1,
                "minimum_moving_pedestrians": 1,
                "minimum_target_routes_planned": 0,
                "minimum_target_route_follow_ticks": 0,
                "minimum_updates_per_device": 0,
                "require_sensor_stale_stop_observed": False,
                "require_sensor_recovery_observed": False,
                "require_recovery_after_injected_stale": False,
                "require_expiry_safe_hold": False,
                "maximum_target_routes_planned": 0 if group == "A" else 1000,
            })
            cfg["output"]["root"] = str(data_root / case / group).replace("\\", "/")
            cfg["output"]["save_depth_float_m"] = False
            problems = validate_schema(cfg, schema) + validate_online_control_config(cfg)
            if problems:
                raise RuntimeError(f"{case}/{group}: config validation failed: {problems}")
            if int(cfg["random_seed"]) != seed:
                raise RuntimeError(f"{case}/{group}: seed changed during preparation")
            if not cfg["dynamic"]["ugv_safety"]["enabled"] or cfg["dynamic"]["ugv_safety"]["obstacle_source"] != "ugv_rgbd":
                raise RuntimeError(f"{case}/{group}: RGB-D safety invariant broken")
            if cfg["dynamic"]["uav_ignore_collision"]:
                raise RuntimeError(f"{case}/{group}: UAV collision protection was disabled")
            path = output_dir / f"{case}_{group}_formal.yaml"
            path.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
            triplet[group] = cfg
            entries.append({
                "case": case,
                "scene": scene,
                "candidate": int(item["candidate"]),
                "seed": seed,
                "group": group,
                "policy": policy,
                "horizon_seconds": horizon,
                "uav_route_length_m": route_length,
                "minimum_uav_path_m": required_uav_path,
                "config": str(path),
                "output_root": cfg["output"]["root"],
            })
        reference = paired_scene_view(triplet["A"])
        for group in "BC":
            if paired_scene_view(triplet[group]) != reference:
                diff = first_differences(reference, paired_scene_view(triplet[group]))
                raise RuntimeError(f"{case}: A/{group} differs beyond policy/output/acceptance: {diff}")

    if len(entries) != 36:
        raise RuntimeError(f"Expected exactly 36 configs; got {len(entries)}")
    index = {
        "experiment": "Town10HD multi-scene paired A/B/C formal batch v3 (bounded stale-route recovery)",
        "created_utc": __import__("datetime").datetime.now(__import__("datetime").timezone.utc).isoformat(),
        "source_admission_index": str(SOURCE_DIR / "frozen_admission_index.json"),
        "source_admission_sha256": admission_digest,
        "formal_data_root": str(data_root),
        "paired_design": "12 reviewed scene instances x 3 policies; A/B/C share seed, actors, start, target, routes, sensors, and duration",
        "horizon_by_scene_seconds": SCENE_HORIZON,
        "sensor_archive": "compact, save one synchronized sample per 100 fixed ticks; all telemetry and trajectories retained",
        "background_traffic_policy": "obey traffic controls; no automatic lane changes; explicit UGV collision avoidance; 6 m leading distance",
        "expiry_recovery": {
            "policy": args.expiry_policy,
            "message_ttl_s": 1.0,
            "coast_grace_s": args.stale_grace_s if args.expiry_policy == "coast_then_patrol" else None,
            "coast_speed_fraction": args.stale_speed_fraction if args.expiry_policy == "coast_then_patrol" else None,
            "coast_max_distance_m": args.stale_max_distance_m if args.expiry_policy == "coast_then_patrol" else None,
            "safety": "RGB-D obstacle guard remains active during coast and patrol rejoin",
        },
        "runs": entries,
    }
    index_path = output_dir / "formal_36_run_index.json"
    index_path.write_text(json.dumps(index, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"count": len(entries), "index": str(index_path), "data_root": str(data_root)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
