"""Short physical driving trial for selected Town10HD proposal variants.

Uses CARLA Traffic Manager only as an infrastructure preflight driver; it does
not claim the project's RGB-D controller or perception has passed these scenes.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import carla

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "docs/figures/multiscene_design_20261009/proposal_manifest.json"
OUT = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scene", choices=("north", "east", "west", "whole"))
    parser.add_argument("candidate", type=int)
    parser.add_argument("--destination", choices=("patrol", "target"), default="patrol")
    parser.add_argument("--timeout", type=float, default=80.0)
    parser.add_argument("--stall-timeout", type=float, default=45.0)
    args = parser.parse_args()
    config = json.loads(MANIFEST.read_text(encoding="utf-8"))["scenarios"][args.scene]
    variant = next(c for c in config["review_candidates"] if c["candidate"] == args.candidate)
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(20)
    world = client.get_world()
    if world.get_settings().synchronous_mode:
        raise RuntimeError("Physical drive trial requires asynchronous simulator mode")
    spawns = world.get_map().get_spawn_points()
    start = spawns[variant["ugv_start_spawn_id"]]
    end_id = variant["ugv_patrol_goal_spawn_id"] if args.destination == "patrol" else variant["target_candidate_spawn_id"]
    goal = spawns[end_id].location
    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    vehicle = world.try_spawn_actor(bp, start)
    if vehicle is None:
        raise RuntimeError(f"Could not spawn UGV at {args.scene}/{args.candidate}")
    collision = None
    collisions = []
    samples = []
    try:
        collision = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"),
                                      carla.Transform(), attach_to=vehicle)
        collision.listen(lambda e: collisions.append({"frame": e.frame, "other": e.other_actor.type_id}))
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(False)
        tm.set_global_distance_to_leading_vehicle(2.0)
        vehicle.set_autopilot(True, 8000)
        tm.set_desired_speed(vehicle, 24.0)
        tm.set_path(vehicle, [goal])
        t0 = time.monotonic()
        best = float("inf")
        reached = False
        stalled = False
        last_progress = t0
        while time.monotonic() - t0 < args.timeout:
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            d = transform.location.distance(goal)
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            samples.append({"wall_s": round(time.monotonic() - t0, 2),
                            "xyz": [round(transform.location.x, 2), round(transform.location.y, 2),
                                    round(transform.location.z, 2)],
                            "yaw_deg": round(transform.rotation.yaw, 1),
                            "speed_mps": round(speed, 2), "goal_distance_m": round(d, 2),
                            "at_traffic_light": vehicle.is_at_traffic_light(),
                            "traffic_light_state": str(vehicle.get_traffic_light_state())})
            if d < best - 1.0:
                best = d
                last_progress = time.monotonic()
            if d < 8:
                reached = True
                break
            if time.monotonic() - last_progress > args.stall_timeout:
                stalled = True
                break
            time.sleep(0.25)
        result = {"scene": args.scene, "candidate": args.candidate,
                  "destination": args.destination, "start_spawn_id": variant["ugv_start_spawn_id"],
                  "goal_spawn_id": end_id, "goal_xyz": [goal.x, goal.y, goal.z],
                  "reached_within_8m": reached, "stalled": stalled,
                  "minimum_goal_distance_m": round(best, 2), "collisions": collisions,
                  "duration_wall_s": round(time.monotonic() - t0, 2), "samples": samples}
        OUT.mkdir(parents=True, exist_ok=True)
        path = OUT / f"drive_{args.scene}_{args.candidate}_{args.destination}.json"
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"{path}: reached={reached} best={best:.1f}m collisions={len(collisions)} "
              f"duration={result['duration_wall_s']}s", flush=True)
    finally:
        vehicle.set_autopilot(False, 8000)
        if collision is not None:
            collision.stop()
            collision.destroy()
        vehicle.destroy()


if __name__ == "__main__":
    main()
