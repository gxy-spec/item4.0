"""CARLA-Air spatial preflight for reviewed Town10HD scene candidates.

This is a survey, not a perception or A/B/C experiment. Ground truth and
ray casting are used only to decide whether a proposed scene can be built.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path
from datetime import datetime, timezone

import carla

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from navigation.ugv_oracle_planner import plan_route_from_message  # noqa: E402

MANIFEST = ROOT / "docs/figures/multiscene_design_20261009/proposal_manifest.json"
OUT = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009")


def loc(p: list[float]) -> carla.Location:
    return carla.Location(x=float(p[0]), y=float(p[1]), z=float(p[2]))


def sample_line(a: list[float], b: list[float], step: float) -> list[list[float]]:
    n = max(1, math.ceil(math.dist(a, b) / step))
    return [[a[k] + (b[k] - a[k]) * i / n for k in range(3)] for i in range(n + 1)]


def settle_static_target(world: carla.World, target: carla.Actor) -> dict:
    """Ground the parked actor before fixing a ray target or capturing images."""
    before = target.get_location()
    start_xyz = [before.x, before.y, before.z]
    target.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
    start_sim = world.get_snapshot().timestamp.elapsed_seconds
    deadline = time.monotonic() + 20
    while world.get_snapshot().timestamp.elapsed_seconds - start_sim < 2.0:
        if time.monotonic() > deadline:
            raise TimeoutError("Target grounding did not advance for 2 simulation seconds")
        world.tick() if world.get_settings().synchronous_mode else world.wait_for_tick(5)
    target.set_simulate_physics(False)
    after = target.get_location()
    return {"initial_cached_xyz": start_xyz,
            "initial_pose_source": "actor cache before grounding; may be uninitialized",
            "parked_xyz": [after.x, after.y, after.z],
            "physics_frozen_after_grounding": True}


def route_metrics(world_map: carla.Map, start: carla.Transform, goal: carla.Transform) -> dict:
    route = plan_route_from_message(world_map, start.location,
                                    [goal.location.x, goal.location.y, goal.location.z],
                                    standoff_m=0.0)
    length = sum(math.dist(a[:2], b[:2]) for a, b in zip(route, route[1:]))
    endpoint = math.dist(route[-1][:2], [goal.location.x, goal.location.y]) if route else None
    max_step = max((math.dist(a[:2], b[:2]) for a, b in zip(route, route[1:])), default=0.0)
    lane_failures = 0
    junction_samples = 0
    turn_angles = []
    for i, p in enumerate(route):
        wp = world_map.get_waypoint(loc(p), project_to_road=False, lane_type=carla.LaneType.Driving)
        if wp is None:
            lane_failures += 1
        elif wp.is_junction:
            junction_samples += 1
        if 0 < i < len(route) - 1:
            v1 = [route[i][j] - route[i - 1][j] for j in (0, 1)]
            v2 = [route[i + 1][j] - route[i][j] for j in (0, 1)]
            d1, d2 = math.hypot(*v1), math.hypot(*v2)
            if d1 > 0.5 and d2 > 0.5:
                turn_angles.append(math.degrees(math.acos(max(-1.0, min(1.0,
                    (v1[0] * v2[0] + v1[1] * v2[1]) / (d1 * d2))))))
    return {"route_points": len(route), "length_m": round(length, 2),
            "endpoint_error_m": round(endpoint, 2) if endpoint is not None else None,
            "max_waypoint_step_m": round(max_step, 2), "non_driving_lane_points": lane_failures,
            "junction_points": junction_samples,
            "max_local_turn_deg": round(max(turn_angles, default=0), 1),
            "map_route_pass": bool(route and endpoint is not None and endpoint < 8
                                   and max_step < 10 and lane_failures == 0),
            "route_xyz": route}


def flight_metrics(world: carla.World, scene: dict) -> dict:
    route = scene["uav_lawnmower_waypoints_xyz"]
    hits = []
    sample_count = 0
    for leg, (a, b) in enumerate(zip(route, route[1:])):
        for p, q in zip(sample_line(a, b, 3.0), sample_line(a, b, 3.0)[1:]):
            sample_count += 1
            for dx, dy in ((0, 0), (1.2, 0), (-1.2, 0), (0, 1.2), (0, -1.2)):
                from_p = carla.Location(x=p[0] + dx, y=p[1] + dy, z=p[2])
                to_p = carla.Location(x=q[0] + dx, y=q[1] + dy, z=q[2])
                ray = world.cast_ray(from_p, to_p)
                if ray:
                    hits.append({"leg": leg + 1, "xyz": [round(ray[0].location.x, 2),
                        round(ray[0].location.y, 2), round(ray[0].location.z, 2)],
                        "label": str(ray[0].label), "offset_xy": [dx, dy]})
                    break
    return {"legs": len(route) - 1, "sampled_segments": sample_count,
            "collision_ray_hits": hits[:20], "clearance_pass": len(hits) == 0,
            "test": "5 longitudinal rays, center and ±1.2 m footprint, every <=3 m"}


def visibility(world: carla.World, route: list[list[float]], target: carla.Actor) -> dict:
    transform = target.get_transform()
    box = target.bounding_box
    aim = transform.transform(carla.Location(x=box.location.x, y=box.location.y,
        z=box.location.z + box.extent.z - 0.5))
    samples = []
    previous = None
    distance_along_route = 0.0
    visible_run_start = None
    longest_visible_span = 0.0
    for leg, (a, b) in enumerate(zip(route, route[1:])):
        for p in sample_line(a, b, 4.0):
            if previous is not None and math.dist(previous, p) < 1e-6:
                continue
            if previous is not None:
                distance_along_route += math.dist(previous, p)
            previous = p
            dx, dy = aim.x - p[0], aim.y - p[1]
            horizontal = math.hypot(dx, dy)
            camera_point = [p[0], p[1], p[2] - 1.5]
            vertical = camera_point[2] - aim.z
            # Nadir camera, 100-degree horizontal FoV, 4:3 image: vertical
            # half-FoV is narrower. Use conservative inscribed cone.
            if vertical <= 0 or horizontal > vertical * math.tan(math.radians(38)):
                visible_run_start = None
                continue
            ray = world.cast_ray(loc(camera_point), aim)
            first = ray[0] if ray else None
            visible = first is not None and math.dist(
                [first.location.x, first.location.y, first.location.z],
                [aim.x, aim.y, aim.z]) < 3.0 and str(first.label) in ("Car", "Truck", "Bus")
            samples.append({"leg": leg + 1, "uav_xyz": [round(x, 2) for x in p],
                            "distance_along_route_m": round(distance_along_route, 2),
                            "range_m": round(math.hypot(horizontal, vertical), 1),
                            "visible": visible, "first_hit": str(first.label) if first else None})
            if visible:
                if visible_run_start is None:
                    visible_run_start = distance_along_route
                longest_visible_span = max(longest_visible_span, distance_along_route - visible_run_start)
            else:
                visible_run_start = None
    good = [s for s in samples if s["visible"]]
    return {"in_fov_samples": len(samples), "visible_samples": len(good),
            "visible_legs": sorted({s["leg"] for s in good}),
            "first_visible_pose": good[0]["uav_xyz"] if good else None,
            "visibility_pass": len(good) >= 3,
            "maximum_contiguous_visible_path_m": round(longest_visible_span, 2),
            "nominal_visible_seconds_at_3mps": round(longest_visible_span / 3.0, 2),
            "method": "fixed grounded actor; camera -1.5 m; unique spatial samples; conservative FoV cone",
            "samples": samples}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=OUT)
    parser.add_argument("--scenes", nargs="+", choices=("north", "east", "west", "whole"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(30)
    world = client.get_world()
    world_map = world.get_map()
    if not world_map.name.endswith("Town10HD"):
        raise RuntimeError(f"Expected Town10HD, got {world_map.name}")
    spawns = world_map.get_spawn_points()
    blueprint = world.get_blueprint_library().find("vehicle.mercedes.sprinter")
    blueprint.set_attribute("color", "255,0,0")
    results = {"scope": "physical preflight; no formal comparisons",
               "map": world_map.name, "manifest": str(args.manifest.resolve()),
               "manifest_sha256": hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
               "started_utc": datetime.now(timezone.utc).isoformat(), "candidates": []}
    for scene_name, scene in manifest["scenarios"].items():
        if args.scenes and scene_name not in args.scenes:
            continue
        flight = flight_metrics(world, scene)
        print(f"{scene_name}: UAV flight ray clearance={flight['clearance_pass']}", flush=True)
        for candidate in scene["review_candidates"]:
            start = spawns[candidate["ugv_start_spawn_id"]]
            patrol = spawns[candidate["ugv_patrol_goal_spawn_id"]]
            target_tf = spawns[candidate["target_candidate_spawn_id"]]
            record = {"scene": scene_name, "candidate": candidate["candidate"],
                      "start_spawn_id": candidate["ugv_start_spawn_id"],
                      "patrol_spawn_id": candidate["ugv_patrol_goal_spawn_id"],
                      "target_spawn_id": candidate["target_candidate_spawn_id"],
                      "flight": flight,
                      "patrol_route": route_metrics(world_map, start, patrol),
                      "target_route": route_metrics(world_map, start, target_tf)}
            actor = world.try_spawn_actor(blueprint, target_tf)
            record["target_spawn_pass"] = actor is not None
            if actor is not None:
                try:
                    record["target_grounding"] = settle_static_target(world, actor)
                    record["uav_target_visibility"] = visibility(
                        world, scene["uav_lawnmower_waypoints_xyz"], actor)
                    final_location = actor.get_location()
                    record["target_pose_drift_m"] = math.dist(
                        record["target_grounding"]["parked_xyz"],
                        [final_location.x, final_location.y, final_location.z])
                finally:
                    actor.destroy()
            results["candidates"].append(record)
            (args.output_dir / "preflight.json").write_text(json.dumps(results, ensure_ascii=False,
                                                            indent=2), encoding="utf-8")
            v = record.get("uav_target_visibility", {})
            print(f"  #{candidate['candidate']} target_spawn={actor is not None} "
                  f"patrol={record['patrol_route']['map_route_pass']} "
                  f"branch={record['target_route']['map_route_pass']} "
                  f"UAV_visible={v.get('visibility_pass')} ({v.get('visible_samples', 0)})",
                  flush=True)
    print(args.output_dir / "preflight.json")


if __name__ == "__main__":
    main()
