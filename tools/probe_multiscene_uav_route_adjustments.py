"""Probe minimal UAV route extensions for low-visibility reviewed candidates.

Writes a suggestion only; it does not change the approved proposal manifest.
"""

from __future__ import annotations

import json
from pathlib import Path

import carla
import yaml

from validate_multiscene_feasibility import flight_metrics, visibility, settle_static_target

ROOT = Path(__file__).resolve().parents[1]
OUT = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2")
MANIFEST = ROOT / "docs/figures/multiscene_design_20261009/proposal_manifest.json"
ROUTE_CONFIG = ROOT / "configs/regions/town10hd_multiscene_uav_routes_v2.yaml"


def main() -> None:
    scenarios = json.loads(MANIFEST.read_text(encoding="utf-8"))["scenarios"]
    world = carla.Client("127.0.0.1", 2000).get_world()
    spawns = world.get_map().get_spawn_points()
    route_config = yaml.safe_load(ROUTE_CONFIG.read_text(encoding="utf-8"))
    changes = {name: override["uav_waypoints_xyz"]
               for name, override in route_config["scenarios"].items()}
    OUT.mkdir(parents=True, exist_ok=True)
    records = []
    for name, route in changes.items():
        flight = flight_metrics(world, {"uav_lawnmower_waypoints_xyz": route})
        item = {"scene": name, "suggested_uav_waypoints_xyz": route,
                "flight_clearance": flight, "candidate_visibility": []}
        for candidate in scenarios[name]["review_candidates"]:
            bp = world.get_blueprint_library().find("vehicle.mercedes.sprinter")
            target = world.try_spawn_actor(bp, spawns[candidate["target_candidate_spawn_id"]])
            if target is None:
                item["candidate_visibility"].append({"candidate": candidate["candidate"],
                                                     "target_spawn_failed": True})
                continue
            try:
                settle_static_target(world, target)
                v = visibility(world, route, target)
                item["candidate_visibility"].append({"candidate": candidate["candidate"],
                                                     "visible_samples": v["visible_samples"],
                                                     "visibility_pass": v["visibility_pass"]})
            finally:
                target.destroy()
        records.append(item)
        print(name, flight["clearance_pass"], item["candidate_visibility"], flush=True)
    (OUT / "proposed_uav_route_adjustments.json").write_text(
        json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
