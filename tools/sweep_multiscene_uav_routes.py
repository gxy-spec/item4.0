"""Collision-aware AirSim pose-command sweep of reviewed UAV routes.

Matches the current experiment runner's UAV control abstraction. This does not
verify flight dynamics of a real multirotor; it verifies that the simulator
accepts route points without a new collision or large pose rejection.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import airsim
import carla

ROOT = Path(__file__).resolve().parents[1]
OUT = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009")
MANIFEST = ROOT / "docs/figures/multiscene_design_20261009/proposal_manifest.json"


def points(route: list[list[float]], stride: float = 4.0):
    for a, b in zip(route, route[1:]):
        count = max(1, math.ceil(math.dist(a, b) / stride))
        for i in range(count):
            yield [a[k] + (b[k] - a[k]) * i / count for k in range(3)]
    yield route[-1]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=OUT)
    parser.add_argument("--scenes", nargs="+", choices=("north", "east", "west", "whole"))
    parser.add_argument("--maximum-pose-error", type=float, default=0.5)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    scenes = json.loads(args.manifest.read_text(encoding="utf-8"))["scenarios"]
    world = carla.Client("127.0.0.1", 2000).get_world()
    drone = next(iter(world.get_actors().filter("airsim.drone")))
    air = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=20)
    air.confirmConnection()
    air.enableApiControl(True)
    air.armDisarm(True)
    initial = air.simGetVehiclePose()
    drone_loc = drone.get_location()
    offset = [drone_loc.x - initial.position.x_val,
              drone_loc.y - initial.position.y_val,
              drone_loc.z + initial.position.z_val]
    results = []
    try:
        for name, scene in scenes.items():
            if args.scenes and name not in args.scenes:
                continue
            route = scene["uav_lawnmower_waypoints_xyz"]
            samples = list(points(route))
            baseline = None
            failures = []
            max_error = 0.0
            for index, p in enumerate(samples):
                desired = airsim.Pose(airsim.Vector3r(p[0] - offset[0], p[1] - offset[1],
                                                     -(p[2] - offset[2])),
                                      airsim.to_quaternion(0, 0, 0))
                air.simSetVehiclePose(desired, index == 0)
                time.sleep(0.04)
                observed = air.simGetVehiclePose()
                xyz = [observed.position.x_val + offset[0],
                       observed.position.y_val + offset[1],
                       -observed.position.z_val + offset[2]]
                error = math.dist(p, xyz)
                max_error = max(max_error, error)
                collision = air.simGetCollisionInfo()
                timestamp = int(getattr(collision, "time_stamp", 0) or 0)
                if index == 0:
                    baseline = timestamp
                if error > args.maximum_pose_error or (timestamp != baseline and bool(collision.has_collided)):
                    failures.append({"index": index, "desired": p, "observed": xyz,
                                     "error_m": round(error, 2), "collision_timestamp": timestamp})
            result = {"scene": name, "samples": len(samples),
                      "maximum_pose_error_m": round(max_error, 2),
                      "failures": failures[:30], "failure_count": len(failures),
                      "accepted": not failures, "pose_error_limit_m": args.maximum_pose_error,
                      "manifest": str(args.manifest.resolve()),
                      "manifest_sha256": hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
                      "control_type": "simSetVehiclePose collision-aware, not aerodynamic path following"}
            results.append(result)
            print(name, result["accepted"], result["maximum_pose_error_m"],
                  len(failures), flush=True)
            (args.output_dir / "uav_route_sweep.json").write_text(json.dumps(results, ensure_ascii=False,
                                                                indent=2), encoding="utf-8")
    finally:
        air.armDisarm(False)
        air.enableApiControl(False)


if __name__ == "__main__":
    main()
