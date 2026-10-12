#!/usr/bin/env python3
"""Freeze the reviewed Town10HD instances for pre-ABC admission runs.

This only writes configuration and a manifest. It does not start an episode.
The source manifest and physical preflight must refer to the same route revision.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import sys
from pathlib import Path

import carla
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from scenario.configuration import resolve_experiment, validate_online_control_config, validate_schema  # noqa: E402
sys.path.insert(0, str(ROOT / "tools"))
from validate_multiscene_feasibility import route_metrics  # noqa: E402

DEFAULT_MANIFEST = ROOT / "docs/figures/multiscene_design_20261009_v2/proposal_manifest.json"
DEFAULT_PREFLIGHT = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2/preflight.json")
DEFAULT_ORIGINAL_PREFLIGHT = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009/preflight.json")
DEFAULT_ORIGINAL_MANIFEST = ROOT / "docs/figures/multiscene_design_20261009/proposal_manifest.json"
DEFAULT_OUTPUT = ROOT / "configs/experiments/multiscene_admission_20261009"
DEFAULT_DATA = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_admission_20261009")
SCENE_CODE = {"north": "N", "east": "E", "west": "W", "whole": "F"}
# The reviewed N3 start is beside persistent curbside traffic; use a distinct
# road spawn, retaining the same target, patrol goal and UAV route.
SAFE_NORTH_STARTS = {"N3": 21}


def xyzw(transform: carla.Transform) -> dict:
    return {
        "location_xyz": [float(transform.location.x), float(transform.location.y), float(transform.location.z)],
        "rotation_pyr_degrees": [float(transform.rotation.pitch), float(transform.rotation.yaw), float(transform.rotation.roll)],
    }


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    p.add_argument("--preflight", type=Path, default=DEFAULT_PREFLIGHT)
    p.add_argument("--original-preflight", type=Path, default=DEFAULT_ORIGINAL_PREFLIGHT)
    p.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    p.add_argument("--data-root", type=Path, default=DEFAULT_DATA)
    args = p.parse_args()
    source = args.manifest.read_bytes()
    source_hash = hashlib.sha256(source).hexdigest()
    manifest = json.loads(source)
    physical = json.loads(args.preflight.read_text(encoding="utf-8"))
    if physical.get("manifest_sha256") != source_hash:
        raise RuntimeError("Physical preflight does not match reviewed manifest hash")
    by_case = {(row["scene"], int(row["candidate"])): row for row in physical["candidates"]}
    original = json.loads(args.original_preflight.read_text(encoding="utf-8"))
    original_manifest = json.loads(DEFAULT_ORIGINAL_MANIFEST.read_text(encoding="utf-8"))
    for scene_name in ("north", "east"):
        current = manifest["scenarios"][scene_name]
        old = original_manifest["scenarios"][scene_name]
        for field in ("uav_lawnmower_waypoints_xyz", "review_candidates", "bounds_xyxy"):
            if current[field] != old[field]:
                raise RuntimeError(f"{scene_name}: original physical checks cannot qualify changed {field}")
    by_case.update({(row["scene"], int(row["candidate"])): row for row in original["candidates"]
                    if row["scene"] in {"north", "east"}})
    if len(by_case) != 12:
        raise RuntimeError(f"Expected 12 reviewed instances, found {len(by_case)}")

    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(25)
    world = client.get_world()
    if not world.get_map().name.endswith("Town10HD"):
        raise RuntimeError("Town10HD simulator is required to resolve spawn transforms")
    spawns = world.get_map().get_spawn_points()
    base = resolve_experiment(ROOT / "configs/experiments/ci_day56_s2001_A.yaml")
    schema = ROOT / "configs/schemas/dynamic_experiment_schema.json"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    result = {"version": 1, "source_manifest": str(args.manifest), "source_sha256": source_hash,
              "physical_preflight": str(args.preflight), "configurations": []}

    for scene_name, scene in manifest["scenarios"].items():
        # The design manifest stores [x_min, x_max, y_min, y_max].
        x0, x1, y0, y1 = map(float, scene["bounds_xyxy"])
        allowed = [i for i, spawn in enumerate(spawns)
                   if x0 <= spawn.location.x <= x1 and y0 <= spawn.location.y <= y1]
        if len(allowed) < 7:
            raise RuntimeError(f"{scene_name}: too few vehicle spawn points within the reviewed bounds")
        route = [[float(v) for v in point] for point in scene["uav_lawnmower_waypoints_xyz"]]
        route_length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
        nominal_seconds = route_length / 3.0
        for item in scene["review_candidates"]:
            number = int(item["candidate"])
            key = (scene_name, number)
            check = by_case[key]
            if not (check["target_spawn_pass"] and check["flight"]["clearance_pass"]
                    and check["patrol_route"]["map_route_pass"] and check["target_route"]["map_route_pass"]
                    and check["uav_target_visibility"]["visibility_pass"]):
                raise RuntimeError(f"{key}: physical preflight is not accepted")
            case = f"{SCENE_CODE[scene_name]}{number}"
            start_id = SAFE_NORTH_STARTS.get(case, int(item["ugv_start_spawn_id"]))
            start_transform = spawns[start_id]
            if start_id != int(item["ugv_start_spawn_id"]):
                patrol = route_metrics(world.get_map(), start_transform,
                                       spawns[int(item["ugv_patrol_goal_spawn_id"])])
                branch = route_metrics(world.get_map(), start_transform,
                                       spawns[int(item["target_candidate_spawn_id"])])
                if not patrol["map_route_pass"] or not branch["map_route_pass"]:
                    raise RuntimeError(f"{case}: replacement start has no valid directed road routes")
            else:
                patrol, branch = check["patrol_route"], check["target_route"]
            # A traffic-manager vehicle spawned behind a slow UGV can ram it
            # before the forward RGB-D guard can observe it. Keep random
            # traffic outside a 40 m initial protection disk; scripted
            # crossing hazards are tested separately in controlled runs.
            allowed_case = [index for index in allowed
                            if math.dist((spawns[index].location.x, spawns[index].location.y),
                                         (start_transform.location.x, start_transform.location.y)) >= 40.0]
            if len(allowed_case) < 7:
                raise RuntimeError(f"{case}: too few non-overlapping distractor spawn points")
            seed = {"north": 1100, "east": 2100, "west": 3100, "whole": 4100}[scene_name] + number
            cfg = copy.deepcopy(base)
            for metadata in ("_source_config", "_included_configs", "includes", "scenario_survey"):
                cfg.pop(metadata, None)
            cfg["experiment_id"] = f"CI_E7_MULTISCENE_ADMISSION_{case}"
            cfg["random_seed"] = seed
            cfg["simulation"]["duration_seconds"] = 35.0
            cfg["region"].update({
                "region_id": f"town10hd_{scene_name}_{number}",
                "selection_status": "admission_reviewed",
                "boundary": {"type": "polygon", "points": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]},
                "ugv_spawn_point_id": start_id,
                "target_transform": xyzw(spawns[int(item["target_candidate_spawn_id"])]),
                "target_lane_transform": xyzw(spawns[int(item["target_candidate_spawn_id"])]),
                "planned_ugv_route": patrol["route_xyz"],
                "planned_ugv_route_length_m": patrol["length_m"],
                "allowed_vehicle_spawn_point_ids": allowed_case,
                "uav": {**cfg["region"]["uav"], "altitude_m": route[0][2],
                        "initial_position_xyz": route[0], "waypoints_xyz": route, "speed_mps": 3.0},
            })
            cfg["actors"]["distractors"].update({"same_category_wrong_color": 1,
                                                   "same_color_wrong_category": 0,
                                                   "random_vehicles": 2, "pedestrians": 2})
            cfg["dynamic"].update({"ugv_patrol_route_target_spawn_point_id": None,
                                   "ugv_patrol_route_limit_m": None,
                                   "uav_altitude_m": route[0][2], "uav_speed_mps": 3.0,
                                   "uav_ignore_collision": False,
                                   "save_all_samples": False, "sensor_save_every_n_frames": 100,
                                   "distractor_spawn_clearance_from_patrol_m": 8.0,
                                   "background_vehicle_autopilot": True})
            cfg["dynamic"]["ugv_safety"]["test_sensor_stale_interval_s"] = None
            cfg["dynamic"]["persistent_target_communication"]["control_policy"] = "none"
            cfg["acceptance"].update({"minimum_target_routes_planned": 0,
                                      "minimum_target_route_follow_ticks": 0,
                                      "minimum_updates_per_device": 0,
                                      "minimum_ugv_path_m": 0.5,
                                      "minimum_uav_path_m": 20.0,
                                      "minimum_moving_vehicles": 1,
                                      "minimum_moving_pedestrians": 1,
                                      "maximum_uav_tracking_p95_m": 0.5,
                                      "require_sensor_stale_stop_observed": False,
                                      "require_sensor_recovery_observed": False,
                                      "require_recovery_after_injected_stale": False})
            cfg["output"]["root"] = str(args.data_root / "short_smoke" / case).replace("\\", "/")
            cfg["output"]["save_depth_float_m"] = False
            problems = validate_schema(cfg, schema) + validate_online_control_config(cfg)
            if problems:
                raise RuntimeError(f"{case}: {problems}")
            path = args.output_dir / f"{case}_A_smoke.yaml"
            path.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
            result["configurations"].append({"case": case, "scene": scene_name, "candidate": number,
                                             "random_seed": seed, "config": str(path),
                                             "reviewed_start_spawn_id": int(item["ugv_start_spawn_id"]),
                                             "admission_start_spawn_id": start_id,
                                             "admission_start_xyz": [start_transform.location.x,
                                                                     start_transform.location.y,
                                                                     start_transform.location.z],
                                             "patrol_route_pass": patrol["map_route_pass"],
                                             "target_route_pass": branch["map_route_pass"],
                                             "uav_route_length_m": round(route_length, 2),
                                             "uav_nominal_seconds": round(nominal_seconds, 2),
                                             "patrol_length_m": patrol["length_m"]})

    (args.output_dir / "frozen_admission_index.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"cases": len(result["configurations"]), "index": str(args.output_dir / "frozen_admission_index.json")},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
