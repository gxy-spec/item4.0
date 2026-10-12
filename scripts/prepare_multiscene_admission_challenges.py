#!/usr/bin/env python3
"""Prepare controlled RGB-D safety and representative long-run admission configs."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "configs/experiments/multiscene_admission_20261009"
DATA = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_admission_20261009")
INDEX = json.loads((CONFIG_DIR / "frozen_admission_index.json").read_text(encoding="utf-8"))
CASES = {row["case"]: row for row in INDEX["configurations"]}


def write(case: str, suffix: str, config: dict) -> Path:
    path = CONFIG_DIR / f"{case}_{suffix}.yaml"
    path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return path


def main() -> None:
    output = []
    for case, kind in (("N2", "pedestrian"), ("E1", "vehicle"),
                       ("W1", "pedestrian"), ("F2", "vehicle")):
        cfg = yaml.safe_load(Path(CASES[case]["config"]).read_text(encoding="utf-8"))
        cfg["experiment_id"] = f"CI_E7_MULTISCENE_SAFETY_{case}_{kind.upper()}"
        cfg["simulation"]["duration_seconds"] = 40.0
        cfg["dynamic"]["background_vehicle_autopilot"] = False
        cfg["dynamic"]["safety_test_crossing_actor"] = (
            {"enabled": True, "actor_type": "pedestrian", "blueprint": "walker.pedestrian.0001",
             "route_distance_m": 18.0, "lateral_start_m": 3.5, "lateral_end_m": -3.5,
             "start_seconds": 9.0, "traverse_seconds": 4.0, "pause_at_center_seconds": 1.0,
             "root_height_above_road_m": 0.92, "despawn_after_seconds": None}
            if kind == "pedestrian" else
            {"enabled": True, "actor_type": "vehicle", "blueprint": "vehicle.audi.tt",
             "route_distance_m": 26.0, "lateral_start_m": 9.0, "lateral_end_m": -9.0,
             "start_seconds": 9.0, "traverse_seconds": 5.0,
             "pause_at_center_seconds": 2.0, "despawn_after_seconds": None}
        )
        cfg["acceptance"]["minimum_moving_vehicles"] = 0
        cfg["acceptance"]["minimum_crossing_actor_displacement_m"] = 5.0
        cfg["acceptance"]["require_crossing_motion_complete"] = True
        cfg["acceptance"]["require_crossing_actor_persists_to_end"] = True
        cfg["acceptance"]["require_pedestrian_stop_policy" if kind == "pedestrian"
                          else "require_crossing_vehicle_stop_policy"] = True
        cfg["output"]["root"] = str(DATA / "safety_challenges" / case).replace("\\", "/")
        path = write(case, f"{kind}_challenge", cfg)
        output.append({"case": case, "kind": kind, "config": str(path)})

    cfg = yaml.safe_load(Path(CASES["W1"]["config"]).read_text(encoding="utf-8"))
    cfg["experiment_id"] = "CI_E7_MULTISCENE_CLOSED_LOOP_W1"
    cfg["simulation"]["duration_seconds"] = 180.0
    cfg["dynamic"]["persistent_target_communication"]["control_policy"] = "dual"
    cfg["acceptance"].update({"minimum_target_routes_planned": 1,
                              "minimum_uav_target_routes_planned": 1,
                              "minimum_target_route_follow_ticks": 10,
                              "minimum_uav_waypoints_reached": 10,
                              "minimum_uav_segments_completed": 9,
                              "minimum_uav_foldbacks_completed": 4})
    cfg["output"]["root"] = str(DATA / "closed_loop" / "W1_C").replace("\\", "/")
    output.append({"case": "W1_C", "kind": "closed_loop", "config": str(write("W1", "C_closed_loop", cfg))})

    cfg = yaml.safe_load(Path(CASES["F1"]["config"]).read_text(encoding="utf-8"))
    cfg["experiment_id"] = "CI_E7_MULTISCENE_WHOLE_LONG_STABILITY_F1"
    cfg["simulation"]["duration_seconds"] = 410.0
    cfg["acceptance"].update({"minimum_uav_waypoints_reached": 10,
                              "minimum_uav_segments_completed": 9,
                              "minimum_uav_foldbacks_completed": 4,
                              "minimum_uav_path_m": 1110.0})
    cfg["output"]["root"] = str(DATA / "long_stability" / "F1_A").replace("\\", "/")
    output.append({"case": "F1_A", "kind": "whole_long_stability", "config": str(write("F1", "A_long", cfg))})

    (CONFIG_DIR / "admission_challenge_index.json").write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"count": len(output), "index": str(CONFIG_DIR / "admission_challenge_index.json")},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
