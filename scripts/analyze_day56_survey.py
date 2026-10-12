#!/usr/bin/env python3
"""Post-run qualification gate for each new paired-seed scene."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from analyze_day3_survey_smoke import distance_to_route, jsonl
from analyze_day4_same_seed_abc import load_one

ROOT = Path("E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/day56_paired_seeds")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, choices=(2001, 3001), required=True)
    args = parser.parse_args()
    root = ROOT / f"S{args.seed}" / "survey"
    index = json.loads((ROOT / f"S{args.seed}" / "survey_index.json").read_text(encoding="utf-8"))
    if len(index) != 1 or index[0]["group"] != "survey":
        raise RuntimeError("Expected one and only one fresh survey run")
    run = load_one(index[0])
    m, config = run["metrics"], run["config"]
    candidates = {role: jsonl(Path(m["run_dir"]) / "perception" / f"{role}_target_candidates_synchronized.jsonl")
                  for role in ("uav", "ugv")}
    correct_candidates = {}
    for role, rows in candidates.items():
        correct_candidates[role] = [row for frame in rows for row in frame["candidates"]
                                    if row.get("world_position_xyz") is not None
                                    and math.dist(row["world_position_xyz"][:2], run["target"][:2]) <= 5]
    first = {role: m[f"first_correct_discovery_{role}_s"] for role in ("uav", "ugv")}
    separation = distance_to_route(run["target"], run["patrol"])
    checks = []

    def check(name: str, passed: bool, value, criterion: str) -> None:
        checks.append({"name": name, "passed": bool(passed), "value": value, "criterion": criterion})

    check("native_scene_healthy", m["native_status"] == "PASS", m["native_status"], "PASS")
    check("target_spawned_at_planned_road_location",
          math.dist(run["target"][:2], config["region"]["target_transform"]["location_xyz"][:2]) <= 1,
          {"actual_xy": run["target"][:2], "planned_xy": config["region"]["target_transform"]["location_xyz"][:2]},
          "target actor within 1 m of planned road spawn")
    check("patrol_and_target_separated", separation >= 25, separation, ">=25 m finite patrol-to-target separation")
    check("A_no_target_route_and_follows_patrol", m["correct_route_plans"] == m["wrong_route_plans"] == 0
          and m["maximum_patrol_deviation_m"] <= 2 and m["minimum_target_distance_m"] > 10,
          {k: m[k] for k in ("correct_route_plans", "wrong_route_plans", "maximum_patrol_deviation_m", "minimum_target_distance_m")},
          "no target route; <=2 m route deviation; never within 10 m of target")
    check("UAV_real_observation_not_instantaneous", first["uav"] is not None and 5 <= first["uav"] <= 45
          and bool(correct_candidates["uav"]),
          {"first_message_s": first["uav"], "correct_rgbd_candidates": len(correct_candidates["uav"])},
          "first correct gated RGB-D candidate message 5–45 s after start")
    if args.seed == 2001:
        check("UGV_local_cannot_discover_target_on_patrol", first["ugv"] is None and not correct_candidates["ugv"],
              {"first_message_s": first["ugv"], "correct_rgbd_candidates": len(correct_candidates["ugv"])},
              "no correct UGV target detection during survey")
    else:
        check("UGV_local_late_discovery", first["ugv"] is not None and first["uav"] is not None
              and first["ugv"] - first["uav"] >= 10 and bool(correct_candidates["ugv"]),
              {"first_uav_s": first["uav"], "first_ugv_s": first["ugv"],
               "correct_ugv_rgbd_candidates": len(correct_candidates["ugv"])},
              "UGV has correct RGB-D candidate >=10 s after UAV")
    check("zero_collision_and_rgbd_safety", m["ugv_collisions"] == m["uav_collisions"] == 0
          and m["sensor_guard_coverage"] >= 0.95,
          {k: m[k] for k in ("ugv_collisions", "uav_collisions", "sensor_guard_coverage")},
          "zero collisions and >=95% safety guard coverage")
    check("source_time_and_frame_aligned", m["common_frame_ratio"] >= 0.98 and m["maximum_frame_spread"] == 0
          and m["maximum_source_time_delta_s"] <= 1e-6,
          {k: m[k] for k in ("common_frame_ratio", "maximum_frame_spread", "maximum_source_time_delta_s")},
          ">=98% common frames, exact frame ID and source timestamp")
    check("actor_motion_sufficient", m["ugv_path_m"] >= 30 and m["uav_path_m"] >= 60,
          {k: m[k] for k in ("ugv_path_m", "uav_path_m")}, "UGV >=30 m and UAV >=60 m")
    report = {"stage": "Day 5–6 scene qualification before paired A/B/C", "seed": args.seed,
              "status": "PASS" if all(c["passed"] for c in checks) else "FAIL", "run_dir": m["run_dir"],
              "checks": checks, "metrics": m,
              "decision": "Paired A/B/C authorized only on PASS; survey is not a performance result"}
    path = root / "scene_qualification.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"seed": args.seed, "status": report["status"], "passed": sum(c["passed"] for c in checks),
                      "total": len(checks), "output": str(path)}, ensure_ascii=False), flush=True)
    for c in checks:
        print(f"{'PASS' if c['passed'] else 'FAIL'} {c['name']}: {c['value']}", flush=True)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
