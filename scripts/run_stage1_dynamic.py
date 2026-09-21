#!/usr/bin/env python3
"""CI-E1 dynamic RGB/depth synchronization acceptance for CityInspection_GOC."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import queue
import random
import shutil
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml
from PIL import Image, ImageDraw


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
SCRIPTS_ROOT = PROJECT_ROOT / "scripts"
for import_root in (str(SRC_ROOT), str(SCRIPTS_ROOT)):
    if import_root not in sys.path:
        sys.path.insert(0, import_root)

import run_stage0_preview as s0  # noqa: E402
from scenario.configuration import resolve_experiment, validate_schema, write_yaml  # noqa: E402
from sensors.depth import bgra_to_rgb, carla_depth_to_metres, colourise_depth  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "experiments" / "ci_e1_dynamic_town10hd_zone_a.yaml",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument("--airsim-port", type=int, default=41451)
    parser.add_argument("--tm-port", type=int, default=None)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output-root", type=Path, default=None)
    return parser.parse_args()


def horizontal_distance(a: Iterable[float], b: Iterable[float]) -> float:
    aa = list(a)
    bb = list(b)
    return math.hypot(float(aa[0]) - float(bb[0]), float(aa[1]) - float(bb[1]))


def cumulative_distance(points: list[list[float]]) -> float:
    return sum(horizontal_distance(a, b) for a, b in zip(points, points[1:]))


def interpolate_polyline(points: list[list[float]], distance_m: float) -> tuple[list[float], float]:
    """Return xyz and travel yaw at distance along a 3D polyline."""
    if len(points) < 2:
        raise ValueError("Polyline must contain at least two points")
    remaining = max(0.0, float(distance_m))
    for start, end in zip(points, points[1:]):
        segment = math.dist(start, end)
        if segment <= 1e-6:
            continue
        if remaining <= segment:
            ratio = remaining / segment
            xyz = [float(start[i]) + ratio * (float(end[i]) - float(start[i])) for i in range(3)]
            yaw = math.degrees(math.atan2(float(end[1]) - float(start[1]), float(end[0]) - float(start[0])))
            return xyz, yaw
        remaining -= segment
    start, end = points[-2], points[-1]
    yaw = math.degrees(math.atan2(float(end[1]) - float(start[1]), float(end[0]) - float(start[0])))
    return [float(v) for v in points[-1]], yaw


def wrap_angle_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def actor_state(actor: Any, role: str, frame: int, timestamp: float) -> dict[str, Any]:
    transform = actor.get_transform()
    velocity = actor.get_velocity()
    angular = actor.get_angular_velocity()
    return {
        "frame": int(frame),
        "timestamp": float(timestamp),
        "actor_id": int(actor.id),
        "role": role,
        "type_id": str(actor.type_id),
        "x": float(transform.location.x),
        "y": float(transform.location.y),
        "z": float(transform.location.z),
        "roll": float(transform.rotation.roll),
        "pitch": float(transform.rotation.pitch),
        "yaw": float(transform.rotation.yaw),
        "vx": float(velocity.x),
        "vy": float(velocity.y),
        "vz": float(velocity.z),
        "speed_mps": float(math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)),
        "angular_x": float(angular.x),
        "angular_y": float(angular.y),
        "angular_z": float(angular.z),
    }


def follow_route(vehicle: Any, route: list[list[float]], target_speed: float) -> int:
    import carla

    transform = vehicle.get_transform()
    location = transform.location
    nearest = min(
        range(len(route)),
        key=lambda index: (route[index][0] - location.x) ** 2 + (route[index][1] - location.y) ** 2,
    )
    lookahead = min(len(route) - 1, nearest + 6)
    target = route[lookahead]
    desired_yaw = math.degrees(math.atan2(target[1] - location.y, target[0] - location.x))
    yaw_error = wrap_angle_degrees(desired_yaw - transform.rotation.yaw)
    steer = float(np.clip(yaw_error / 42.0, -0.72, 0.72))
    velocity = vehicle.get_velocity()
    speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    remaining = horizontal_distance([location.x, location.y], route[-1])
    if remaining < 3.0:
        control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=steer, hand_brake=False)
    else:
        speed_error = target_speed - speed
        throttle = float(np.clip(0.30 + speed_error * 0.16, 0.0, 0.72))
        brake = float(np.clip((speed - target_speed - 1.0) * 0.25, 0.0, 0.65))
        control = carla.VehicleControl(throttle=throttle, brake=brake, steer=steer, hand_brake=False)
    vehicle.apply_control(control)
    return nearest


def sample_nav_location(world: Any, bounds: dict[str, float], rng: random.Random, origin: Any | None = None) -> Any:
    for _ in range(160):
        location = world.get_random_location_from_navigation()
        if location is None:
            continue
        if not (
            bounds["x_min"] + 3.0 <= location.x <= bounds["x_max"] - 3.0
            and bounds["y_min"] + 3.0 <= location.y <= bounds["y_max"] - 3.0
        ):
            continue
        if origin is not None and horizontal_distance([location.x, location.y], [origin.x, origin.y]) < 10.0:
            continue
        return location
    return None


def spawn_walkers(world: Any, count: int, bounds: dict[str, float], rng: random.Random) -> tuple[list[Any], list[Any]]:
    walker_bps = list(world.get_blueprint_library().filter("walker.pedestrian.*"))
    controller_bp = world.get_blueprint_library().find("controller.ai.walker")
    walkers: list[Any] = []
    controllers: list[Any] = []
    import carla

    for index in range(count):
        spawn_location = sample_nav_location(world, bounds, rng)
        if spawn_location is None:
            continue
        spawn_location.z += 0.5
        bp = rng.choice(walker_bps)
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "false")
        walker = world.try_spawn_actor(bp, carla.Transform(spawn_location))
        if walker is None:
            continue
        controller = world.try_spawn_actor(controller_bp, carla.Transform(), attach_to=walker)
        if controller is None:
            walker.destroy()
            continue
        walkers.append(walker)
        controllers.append(controller)
    return walkers, controllers


def build_run_directories(output_root: Path, experiment_id: str) -> dict[str, Path]:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    run_dir = output_root / f"{experiment_id}_{stamp}"
    directories = {
        "run": run_dir,
        "actors": run_dir / "actors",
        "trajectories": run_dir / "trajectories",
        "synchronization": run_dir / "synchronization",
        "preview": run_dir / "preview",
        "logs": run_dir / "logs",
        "config": run_dir / "config_snapshot",
    }
    for device in ("uav", "ugv"):
        for modality in ("rgb", "depth_raw", "depth_metres", "depth_colour"):
            directories[f"{device}_{modality}"] = run_dir / "sensors" / device / modality
        directories[f"{device}_metadata"] = run_dir / "sensors" / device / "metadata"
    for path in directories.values():
        path.mkdir(parents=True, exist_ok=True)
    return directories


def save_rgb(image: Any, destination: Path) -> tuple[str, np.ndarray]:
    array = bgra_to_rgb(image.raw_data, image.width, image.height)
    Image.fromarray(array).save(destination)
    return hashlib.sha256(array.tobytes()).hexdigest(), array


def save_depth(image: Any, raw_destination: Path, metres_destination: Path, colour_destination: Path) -> dict[str, float]:
    encoded = bgra_to_rgb(image.raw_data, image.width, image.height)
    metres = carla_depth_to_metres(image.raw_data, image.width, image.height)
    Image.fromarray(encoded).save(raw_destination)
    np.save(metres_destination, metres.astype(np.float32, copy=False))
    colourise_depth(metres, display_max_m=100.0).save(colour_destination)
    finite = np.isfinite(metres)
    return {
        "minimum_m": float(np.nanmin(metres)),
        "maximum_m": float(np.nanmax(metres)),
        "mean_m": float(np.nanmean(metres)),
        "finite_fraction": float(np.mean(finite)),
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def add_acceptance_check(
    checks: list[dict[str, Any]], name: str, passed: bool, value: Any, criterion: str
) -> None:
    checks.append(
        {
            "name": name,
            "passed": bool(passed),
            "value": value,
            "criterion": criterion,
            "severity": "error",
        }
    )


def draw_dynamic_map(
    world_map: Any,
    region: dict[str, Any],
    trajectories: dict[str, list[list[float]]],
    actor_tracks: dict[str, list[list[float]]],
    output_path: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    polygon_points = region["boundary"]["points"]
    bounds = {
        "x_min": min(point[0] for point in polygon_points),
        "x_max": max(point[0] for point in polygon_points),
        "y_min": min(point[1] for point in polygon_points),
        "y_max": max(point[1] for point in polygon_points),
    }
    fig, axis = plt.subplots(figsize=(11.5, 9), dpi=170)
    topology = world_map.get_topology()
    for waypoint_a, waypoint_b in topology:
        points = [[waypoint_a.transform.location.x, waypoint_a.transform.location.y]]
        current = waypoint_a
        for _ in range(120):
            if current.transform.location.distance(waypoint_b.transform.location) < 2.0:
                break
            following = current.next(2.0)
            if not following:
                break
            current = min(following, key=lambda wp: wp.transform.location.distance(waypoint_b.transform.location))
            points.append([current.transform.location.x, current.transform.location.y])
        points.append([waypoint_b.transform.location.x, waypoint_b.transform.location.y])
        values = np.asarray(points)
        if np.any(
            (values[:, 0] >= bounds["x_min"] - 15)
            & (values[:, 0] <= bounds["x_max"] + 15)
            & (values[:, 1] >= bounds["y_min"] - 15)
            & (values[:, 1] <= bounds["y_max"] + 15)
        ):
            axis.plot(values[:, 0], values[:, 1], color="#c8ced7", linewidth=1.4, alpha=0.8, zorder=1)

    polygon = np.asarray(polygon_points + [polygon_points[0]])
    axis.fill(polygon[:, 0], polygon[:, 1], color="#eef3f7", alpha=0.35, zorder=0)
    axis.plot(polygon[:, 0], polygon[:, 1], color="#536a83", linestyle="--", linewidth=1.5, label="Zone A boundary")

    vehicle_colour = "#d98918"
    walker_colour = "#2a9d6f"
    for role, points in actor_tracks.items():
        if len(points) < 2:
            continue
        values = np.asarray(points)
        colour = walker_colour if role.startswith("pedestrian") else vehicle_colour
        axis.plot(values[:, 0], values[:, 1], color=colour, linewidth=0.9, alpha=0.65, zorder=2)
        axis.scatter(values[-1, 0], values[-1, 1], s=12, color=colour, zorder=3)

    ugv = np.asarray(trajectories["ugv"])
    uav = np.asarray(trajectories["uav"])
    target = np.asarray(trajectories["target"])
    axis.plot(ugv[:, 0], ugv[:, 1], color="#1565c0", linewidth=3.2, label="UGV actual trajectory", zorder=5)
    axis.plot(uav[:, 0], uav[:, 1], color="#7b2cbf", linewidth=3.0, label="UAV actual trajectory", zorder=5)
    axis.scatter(ugv[0, 0], ugv[0, 1], marker="o", s=65, color="#1565c0", edgecolor="white", zorder=6)
    axis.scatter(uav[0, 0], uav[0, 1], marker="^", s=80, color="#7b2cbf", edgecolor="white", zorder=6)
    axis.scatter(target[-1, 0], target[-1, 1], marker="*", s=210, color="#c62828", edgecolor="white", label="Target vehicle", zorder=7)
    axis.scatter([], [], color=vehicle_colour, s=20, label="Distractor vehicle tracks")
    axis.scatter([], [], color=walker_colour, s=20, label="Pedestrian tracks")
    axis.set_title("CI-E1 Dynamic Scene — Town10HD Zone A", fontsize=16, weight="bold", pad=14)
    axis.set_xlabel("CARLA world X (m)")
    axis.set_ylabel("CARLA world Y (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlim(bounds["x_min"] - 8, bounds["x_max"] + 8)
    axis.set_ylim(bounds["y_min"] - 8, bounds["y_max"] + 8)
    axis.grid(color="#d8dee7", linewidth=0.65, alpha=0.7)
    axis.legend(loc="upper right", fontsize=8.5, framealpha=0.95)
    fig.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def draw_alignment(frame_rows: list[dict[str, Any]], expected_count: int, output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    frames = np.asarray([int(row["frame"]) for row in frame_rows])
    timestamps = np.asarray([float(row["timestamp"]) for row in frame_rows])
    samples = np.arange(len(frames))
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), dpi=170)
    axes[0].plot(samples, frames, color="#1565c0", linewidth=1.8)
    axes[0].set_title("Common frame sequence")
    axes[0].set_xlabel("Saved sample index")
    axes[0].set_ylabel("CARLA frame ID")
    axes[0].grid(alpha=0.3)
    intervals = np.diff(timestamps) if len(timestamps) > 1 else np.asarray([])
    axes[1].plot(np.arange(len(intervals)), intervals, color="#7b2cbf", linewidth=1.4)
    axes[1].axhline(0.1, color="#c62828", linestyle="--", linewidth=1.2, label="10 Hz target")
    axes[1].set_title("Timestamp interval")
    axes[1].set_xlabel("Sample transition")
    axes[1].set_ylabel("Interval (s)")
    axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=8)
    fig.suptitle(f"CI-E1 Four-stream Synchronization — {len(frame_rows)}/{expected_count} expected frames", fontsize=14, weight="bold")
    fig.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def draw_sensor_composite(run_dirs: dict[str, Path], first_frame: int, last_frame: int, summary: dict[str, Any]) -> None:
    canvas = Image.new("RGB", (1640, 1030), "#f5f7fa")
    draw = ImageDraw.Draw(canvas)
    font_title = s0.load_font(36, bold=True)
    font_heading = s0.load_font(24, bold=True)
    font_body = s0.load_font(20)
    draw.text((42, 30), "CI-E1 Dynamic RGB + Depth Acceptance", fill="#152238", font=font_title)
    draw.text((42, 82), f"First synchronized frame {first_frame}  |  Last synchronized frame {last_frame}", fill="#51647e", font=font_body)

    cells = [
        ("UAV RGB — first", run_dirs["uav_rgb"] / f"{first_frame:08d}.png"),
        ("UAV depth — last", run_dirs["uav_depth_colour"] / f"{last_frame:08d}.png"),
        ("UGV RGB — first", run_dirs["ugv_rgb"] / f"{first_frame:08d}.png"),
        ("UGV depth — last", run_dirs["ugv_depth_colour"] / f"{last_frame:08d}.png"),
    ]
    positions = [(42, 140), (832, 140), (42, 555), (832, 555)]
    for (label, path), (x, y) in zip(cells, positions):
        draw.rounded_rectangle((x, y, x + 765, y + 380), radius=14, fill="white", outline="#ccd5e1", width=2)
        draw.text((x + 18, y + 13), label, fill="#1e3550", font=font_heading)
        image = Image.open(path).convert("RGB")
        image.thumbnail((729, 315), Image.Resampling.LANCZOS)
        canvas.paste(image, (x + 18 + (729 - image.width) // 2, y + 55 + (315 - image.height) // 2))
    footer = (
        f"common frames {summary['common_frames']}  |  common-frame ratio {summary['common_frame_ratio']:.1%}  |  "
        f"UGV path {summary['ugv_path_m']:.1f} m  |  UAV path {summary['uav_path_m']:.1f} m"
    )
    draw.text((42, 982), footer, fill="#314962", font=font_body)
    canvas.save(run_dirs["preview"] / "sensor_composite.png")


def main() -> int:
    args = parse_args()
    config_path = args.config.resolve()
    resolved = resolve_experiment(config_path)
    schema_errors = validate_schema(
        resolved, PROJECT_ROOT / "configs" / "schemas" / "dynamic_experiment_schema.json"
    )
    if schema_errors:
        raise RuntimeError(f"Dynamic experiment schema validation failed: {schema_errors}")

    output_root = args.output_root.resolve() if args.output_root else Path(resolved["output"]["root"])
    run_dirs = build_run_directories(output_root, resolved["experiment_id"])
    shutil.copy2(config_path, run_dirs["config"] / config_path.name)
    write_yaml(run_dirs["run"] / "resolved_config.yaml", resolved)

    seed = int(resolved["random_seed"])
    rng = random.Random(seed)
    np.random.seed(seed)
    duration = float(resolved["simulation"]["duration_seconds"])
    fixed_delta = float(resolved["simulation"]["fixed_delta_seconds"])
    sensor_tick = 1.0 / float(resolved["sensors"]["frequency_hz"])
    total_ticks = int(round(duration / fixed_delta))
    expected_frames = int(round(duration / sensor_tick))
    dynamic_cfg = resolved["dynamic"]
    acceptance = resolved["acceptance"]
    tm_port = int(args.tm_port or resolved["simulation"]["traffic_manager_port"])

    run_log = run_dirs["logs"] / "run.log"
    log_lines: list[str] = []

    def log(message: str) -> None:
        line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {message}"
        print(line, flush=True)
        log_lines.append(line)
        run_log.write_text("\n".join(log_lines) + "\n", encoding="utf-8")

    world = None
    traffic_manager = None
    original_settings = None
    owned_actors: list[Any] = []
    sensors: list[Any] = []
    walker_controllers: list[Any] = []
    airsim_client = None
    report: dict[str, Any] = {
        "experiment_id": resolved["experiment_id"],
        "run_directory": str(run_dirs["run"]),
        "started_at": s0.now_utc(),
        "status": "RUNNING",
        "checks": [],
    }

    try:
        import airsim
        import carla

        log(f"Connecting to CARLA {args.host}:{args.carla_port}")
        client = carla.Client(args.host, args.carla_port)
        client.set_timeout(args.timeout)
        world = client.get_world()
        world_map = world.get_map()
        map_name = world_map.name.split("/")[-1]
        configured_map = str(resolved["simulation"]["map"])
        if configured_map.lower() not in map_name.lower():
            raise RuntimeError(f"Expected {configured_map}, connected to {map_name}")

        original_settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(tm_port)
        traffic_manager.set_random_device_seed(seed)
        traffic_manager.set_synchronous_mode(False)

        region = resolved["region"]
        polygon_points = region["boundary"]["points"]
        bounds = {
            "x_min": min(point[0] for point in polygon_points),
            "x_max": max(point[0] for point in polygon_points),
            "y_min": min(point[1] for point in polygon_points),
            "y_max": max(point[1] for point in polygon_points),
        }
        route = [[float(v) for v in point] for point in region["planned_ugv_route"]]
        uav_route = [[float(v) for v in point] for point in region["uav"]["waypoints_xyz"]]
        spawn_points = world_map.get_spawn_points()
        ugv_spawn = int(region["ugv_spawn_point_id"])
        allowed_spawns = [int(value) for value in region["allowed_vehicle_spawn_point_ids"] if int(value) != ugv_spawn]
        blueprint_library = world.get_blueprint_library()

        ugv_bp = blueprint_library.find(resolved["actors"]["ugv"]["blueprint"])
        s0.configure_blueprint(ugv_bp, "inspection_ugv", resolved["actors"]["ugv"].get("color_rgb"))
        ugv = world.try_spawn_actor(ugv_bp, spawn_points[ugv_spawn])
        if ugv is None:
            raise RuntimeError(f"Failed to spawn UGV at Town10HD spawn {ugv_spawn}")
        owned_actors.append(ugv)

        target_cfg = resolved["actors"]["target_vehicle"]
        target_bp, target_bp_id = s0.resolve_target_blueprint(blueprint_library, resolved)
        s0.configure_blueprint(target_bp, "inspection_target", target_cfg["color_rgb"])
        target = world.try_spawn_actor(target_bp, s0.dict_to_transform(region["target_transform"]))
        if target is None:
            raise RuntimeError("Failed to spawn the frozen target vehicle")
        target.set_simulate_physics(False)
        owned_actors.append(target)

        composition = resolved["actors"]["distractors"]
        distractors: list[Any] = []
        distractor_roles: list[str] = []
        shuffled_spawns = allowed_spawns[:]
        rng.shuffle(shuffled_spawns)
        sedan_ids = s0.resolve_sedan_blueprints(blueprint_library, {target_bp_id})
        vehicle_bps = [bp for bp in blueprint_library.filter("vehicle.*") if int(bp.get_attribute("number_of_wheels")) == 4]

        def add_distractor(blueprint_id: str, colour: str | None, role_prefix: str) -> None:
            actor = None
            while shuffled_spawns and actor is None:
                spawn_id = shuffled_spawns.pop()
                actor = s0.spawn_vehicle(
                    world, blueprint_library, blueprint_id, spawn_points[spawn_id], role_prefix, colour
                )
            if actor is None:
                return
            distractors.append(actor)
            distractor_roles.append(f"vehicle_{len(distractors):02d}")
            owned_actors.append(actor)

        for _ in range(int(composition["same_category_wrong_color"])):
            add_distractor(target_bp_id, "0,0,255", "same_category_wrong_color")
        wrong_category_pool = [blueprint_id for blueprint_id in sedan_ids if blueprint_id != target_bp_id]
        for _ in range(int(composition["same_color_wrong_category"])):
            add_distractor(rng.choice(wrong_category_pool), target_cfg["color_rgb"], "same_color_wrong_category")
        for _ in range(int(composition["random_vehicles"])):
            add_distractor(rng.choice(vehicle_bps).id, None, "random_background")

        walkers, walker_controllers = spawn_walkers(
            world,
            int(composition["pedestrians"]),
            bounds,
            rng,
        )
        owned_actors.extend(walkers)
        owned_actors.extend(walker_controllers)
        pedestrian_roles = [f"pedestrian_{index + 1:02d}" for index in range(len(walkers))]

        log(f"Spawned UGV, target, {len(distractors)} distractor vehicles and {len(walkers)} pedestrians")

        airsim_client = airsim.MultirotorClient(ip=args.host, port=args.airsim_port, timeout_value=args.timeout)
        airsim_client.confirmConnection()
        vehicle_name = resolved["actors"]["uav"].get("vehicle_name", "")
        airsim_client.enableApiControl(True, vehicle_name=vehicle_name)
        airsim_client.armDisarm(True, vehicle_name=vehicle_name)
        drone_candidates = list(world.get_actors().filter("*drone*"))
        if not drone_candidates:
            drone_candidates = [actor for actor in world.get_actors() if "airsim" in actor.type_id.lower()]
        if not drone_candidates:
            raise RuntimeError("No AirSim drone actor is visible in CARLA")
        drone = drone_candidates[0]
        initial_drone_location = drone.get_location()
        initial_air_pose = airsim_client.simGetVehiclePose(vehicle_name=vehicle_name)
        ned_offset = [
            initial_drone_location.x - float(initial_air_pose.position.x_val),
            initial_drone_location.y - float(initial_air_pose.position.y_val),
            initial_drone_location.z + float(initial_air_pose.position.z_val),
        ]

        def set_drone_pose(carla_xyz: list[float], yaw_deg: float) -> None:
            ned = airsim.Vector3r(
                float(carla_xyz[0] - ned_offset[0]),
                float(carla_xyz[1] - ned_offset[1]),
                float(-(carla_xyz[2] - ned_offset[2])),
            )
            orientation = airsim.to_quaternion(0.0, 0.0, math.radians(yaw_deg))
            airsim_client.simSetVehiclePose(airsim.Pose(ned, orientation), True, vehicle_name=vehicle_name)

        initial_uav, initial_uav_yaw = interpolate_polyline(uav_route, 0.0)
        set_drone_pose(initial_uav, initial_uav_yaw)
        time.sleep(0.15)

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = fixed_delta
        settings.substepping = True
        settings.max_substep_delta_time = min(0.01, fixed_delta)
        settings.max_substeps = max(5, int(math.ceil(fixed_delta / settings.max_substep_delta_time)))
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(True)
        world.tick()

        for actor in distractors:
            actor.set_autopilot(True, tm_port)
            traffic_manager.ignore_lights_percentage(actor, 100.0)
            traffic_manager.ignore_signs_percentage(actor, 100.0)
            traffic_manager.auto_lane_change(actor, True)
            try:
                traffic_manager.set_desired_speed(actor, 24.0 + rng.uniform(-3.0, 5.0))
            except RuntimeError:
                pass

        world.set_pedestrians_seed(seed)
        for walker, controller in zip(walkers, walker_controllers):
            controller.start()
            destination = sample_nav_location(world, bounds, rng, origin=walker.get_location())
            if destination is not None:
                controller.go_to_location(destination)
            controller.set_max_speed(1.35 + rng.uniform(-0.1, 0.2))

        queues: dict[str, queue.Queue[Any]] = {}
        buffers: dict[str, dict[int, Any]] = {}
        sensor_config = resolved["sensors"]
        stream_specs = {
            "uav_rgb": sensor_config["streams"]["uav_rgb"],
            "uav_depth": sensor_config["streams"]["uav_depth"],
            "ugv_rgb": sensor_config["streams"]["ugv_rgb"],
            "ugv_depth": sensor_config["streams"]["ugv_depth"],
        }
        for name, cfg in stream_specs.items():
            bp = s0.sensor_blueprint(blueprint_library, cfg["type"], sensor_config)
            if name.startswith("uav"):
                world_transform = carla.Transform(
                    carla.Location(initial_uav[0], initial_uav[1], initial_uav[2] - 1.5),
                    carla.Rotation(pitch=-90.0, yaw=initial_uav_yaw),
                )
                sensor = world.spawn_actor(bp, world_transform)
            else:
                transform = carla.Transform(
                    carla.Location(*[float(v) for v in cfg["location_xyz_m"]]),
                    carla.Rotation(
                        pitch=float(cfg["rotation_pyr_degrees"][0]),
                        yaw=float(cfg["rotation_pyr_degrees"][1]),
                        roll=float(cfg["rotation_pyr_degrees"][2]),
                    ),
                )
                sensor = world.spawn_actor(bp, transform, attach_to=ugv)
            packet_queue: queue.Queue[Any] = queue.Queue(maxsize=128)
            sensor.listen(lambda packet, q=packet_queue: s0.push(q, packet))
            sensors.append(sensor)
            owned_actors.append(sensor)
            queues[name] = packet_queue
            buffers[name] = {}

        roles: list[tuple[str, Any]] = [("ugv", ugv), ("uav", drone), ("target", target)]
        roles.extend(zip(distractor_roles, distractors))
        roles.extend(zip(pedestrian_roles, walkers))
        initial_actor_manifest = [s0.actor_record(actor, role, role.split("_")[0]) for role, actor in roles]
        s0.json_dump(run_dirs["actors"] / "initial_actor_states.json", initial_actor_manifest)

        actor_state_handle = (run_dirs["actors"] / "actor_states.jsonl").open("w", encoding="utf-8")
        metadata_handles = {
            "uav": (run_dirs["uav_metadata"] / "frames.jsonl").open("w", encoding="utf-8"),
            "ugv": (run_dirs["ugv_metadata"] / "frames.jsonl").open("w", encoding="utf-8"),
        }

        frame_rows: list[dict[str, Any]] = []
        trajectory_rows: dict[str, list[dict[str, Any]]] = defaultdict(list)
        actor_tracks: dict[str, list[list[float]]] = defaultdict(list)
        rgb_hashes: dict[str, list[str]] = defaultdict(list)
        depth_finite: dict[str, list[float]] = defaultdict(list)
        follow_errors: list[float] = []
        world_state_by_frame: dict[int, dict[str, Any]] = {}
        saved_frames: set[int] = set()
        started_sim_time: float | None = None

        log(f"Running {duration:.1f}s dynamic acquisition ({total_ticks} fixed ticks, expected {expected_frames} sensor frames)")
        for tick_index in range(total_ticks):
            sim_elapsed = tick_index * fixed_delta
            desired_uav, desired_uav_yaw = interpolate_polyline(
                uav_route,
                sim_elapsed * float(dynamic_cfg["uav_speed_mps"]),
            )
            set_drone_pose(desired_uav, desired_uav_yaw)
            for name, sensor in zip(("uav_rgb", "uav_depth"), sensors[:2]):
                offset = float(stream_specs[name].get("vertical_offset_m", -1.5))
                sensor.set_transform(
                    carla.Transform(
                        carla.Location(desired_uav[0], desired_uav[1], desired_uav[2] + offset),
                        carla.Rotation(pitch=-90.0, yaw=desired_uav_yaw),
                    )
                )
            follow_route(ugv, route, float(dynamic_cfg["ugv_target_speed_mps"]))
            frame = world.tick()
            snapshot = world.get_snapshot()
            elapsed = float(snapshot.timestamp.elapsed_seconds)
            if started_sim_time is None:
                started_sim_time = elapsed
            states = [actor_state(actor, role, frame, elapsed) for role, actor in roles if actor.is_alive]
            state_by_role = {state["role"]: state for state in states}
            world_state_by_frame[frame] = {"elapsed": elapsed, "states": states, "desired_uav": desired_uav}
            for state in states:
                actor_state_handle.write(json.dumps(state, ensure_ascii=False) + "\n")
                actor_tracks[state["role"]].append([state["x"], state["y"], state["z"]])
                if state["role"] in {"ugv", "uav", "target"}:
                    trajectory_rows[state["role"]].append(state)
            actual_uav = state_by_role.get("uav")
            if actual_uav is not None:
                follow_errors.append(horizontal_distance([actual_uav["x"], actual_uav["y"]], desired_uav))

            # GPU camera callbacks are asynchronous even while the CARLA world is
            # synchronous.  A short render barrier prevents long runs from outrunning
            # the four camera streams and dropping most of the second half.
            time.sleep(0.035)
            for stream_name, packet_queue in queues.items():
                for packet in s0.drain_all(packet_queue):
                    buffers[stream_name][int(packet.frame)] = packet

            common = set.intersection(*(set(buffer.keys()) for buffer in buffers.values()))
            for common_frame in sorted(common - saved_frames):
                if common_frame not in world_state_by_frame:
                    continue
                packets = {name: buffers[name][common_frame] for name in buffers}
                frame_state = world_state_by_frame[common_frame]
                rgb_arrays: dict[str, np.ndarray] = {}
                for device in ("uav", "ugv"):
                    rgb_packet = packets[f"{device}_rgb"]
                    depth_packet = packets[f"{device}_depth"]
                    rgb_path = run_dirs[f"{device}_rgb"] / f"{common_frame:08d}.png"
                    depth_raw_path = run_dirs[f"{device}_depth_raw"] / f"{common_frame:08d}.png"
                    depth_m_path = run_dirs[f"{device}_depth_metres"] / f"{common_frame:08d}.npy"
                    depth_colour_path = run_dirs[f"{device}_depth_colour"] / f"{common_frame:08d}.png"
                    digest, rgb_array = save_rgb(rgb_packet, rgb_path)
                    depth_stats = save_depth(depth_packet, depth_raw_path, depth_m_path, depth_colour_path)
                    rgb_hashes[device].append(digest)
                    rgb_arrays[device] = rgb_array
                    depth_finite[device].append(depth_stats["finite_fraction"])
                    metadata = {
                        "frame": common_frame,
                        "timestamp": float(rgb_packet.timestamp),
                        "rgb_path": str(rgb_path),
                        "depth_raw_path": str(depth_raw_path),
                        "depth_metres_path": str(depth_m_path),
                        "sensor_transform": {
                            "location": s0.xyz(rgb_packet.transform.location),
                            "rotation": s0.rotation_pyr(rgb_packet.transform.rotation),
                        },
                        "depth_statistics": depth_stats,
                    }
                    metadata_handles[device].write(json.dumps(metadata, ensure_ascii=False) + "\n")

                packet_frames = [int(packet.frame) for packet in packets.values()]
                frame_rows.append(
                    {
                        "sample_index": len(frame_rows),
                        "frame": int(common_frame),
                        "timestamp": float(packets["uav_rgb"].timestamp),
                        "uav_rgb_frame": packet_frames[0],
                        "uav_depth_frame": packet_frames[1],
                        "ugv_rgb_frame": packet_frames[2],
                        "ugv_depth_frame": packet_frames[3],
                        "frame_spread": max(packet_frames) - min(packet_frames),
                    }
                )
                saved_frames.add(common_frame)
                for buffer in buffers.values():
                    buffer.pop(common_frame, None)

            prune_before = frame - 100
            for buffer in buffers.values():
                for old_frame in [value for value in buffer if value < prune_before]:
                    buffer.pop(old_frame, None)
            for old_frame in [value for value in world_state_by_frame if value < prune_before and value in saved_frames]:
                world_state_by_frame.pop(old_frame, None)

        actor_state_handle.close()
        for handle in metadata_handles.values():
            handle.close()

        for role, rows in trajectory_rows.items():
            write_csv(run_dirs["trajectories"] / f"{role}_trajectory.csv", rows)
        write_csv(run_dirs["synchronization"] / "frame_index.csv", frame_rows)

        final_manifest = [
            s0.actor_record(actor, role, role.split("_")[0]) for role, actor in roles if actor.is_alive
        ]
        s0.json_dump(run_dirs["actors"] / "final_actor_states.json", final_manifest)
        initial_by_role = {item["role"]: item for item in initial_actor_manifest}
        final_by_role = {item["role"]: item for item in final_manifest}

        paths_xyz = {
            role: [[row["x"], row["y"], row["z"]] for row in rows]
            for role, rows in trajectory_rows.items()
        }
        ugv_path = cumulative_distance(paths_xyz.get("ugv", []))
        uav_path = cumulative_distance(paths_xyz.get("uav", []))
        target_path = cumulative_distance(paths_xyz.get("target", []))
        common_ratio = len(frame_rows) / max(1, expected_frames)

        vehicle_displacements: dict[str, float] = {}
        for role in distractor_roles:
            if role in initial_by_role and role in final_by_role:
                vehicle_displacements[role] = horizontal_distance(
                    initial_by_role[role]["location_xyz"], final_by_role[role]["location_xyz"]
                )
        pedestrian_displacements: dict[str, float] = {}
        for role in pedestrian_roles:
            track = actor_tracks.get(role, [])
            if len(track) >= 2:
                pedestrian_displacements[role] = horizontal_distance(track[0], track[-1])

        moving_vehicles = sum(
            value >= float(acceptance["vehicle_motion_threshold_m"])
            for value in vehicle_displacements.values()
        )
        moving_pedestrians = sum(
            value >= float(acceptance["pedestrian_motion_threshold_m"])
            for value in pedestrian_displacements.values()
        )
        uav_follow_p95 = float(np.percentile(follow_errors, 95)) if follow_errors else float("inf")
        unique_rgb = {
            device: len(set(values)) / max(1, len(values)) for device, values in rgb_hashes.items()
        }
        max_frame_spread = max((int(row["frame_spread"]) for row in frame_rows), default=999)

        checks: list[dict[str, Any]] = []
        add_acceptance_check(checks, "map_is_frozen", map_name.lower().endswith("town10hd"), map_name, "Town10HD")
        add_acceptance_check(checks, "common_frame_ratio", common_ratio >= float(acceptance["minimum_common_frame_ratio"]), common_ratio, f">={acceptance['minimum_common_frame_ratio']}")
        add_acceptance_check(checks, "exact_four_stream_frame_alignment", max_frame_spread == 0, max_frame_spread, "0")
        add_acceptance_check(checks, "ugv_dynamic_path", ugv_path >= float(acceptance["minimum_ugv_path_m"]), ugv_path, f">={acceptance['minimum_ugv_path_m']} m")
        add_acceptance_check(checks, "uav_dynamic_path", uav_path >= float(acceptance["minimum_uav_path_m"]), uav_path, f">={acceptance['minimum_uav_path_m']} m")
        add_acceptance_check(checks, "uav_control_tracking_p95", uav_follow_p95 <= float(acceptance["maximum_uav_tracking_p95_m"]), uav_follow_p95, f"<={acceptance['maximum_uav_tracking_p95_m']} m")
        add_acceptance_check(checks, "target_remains_stationary", target_path <= float(acceptance["maximum_target_drift_m"]), target_path, f"<={acceptance['maximum_target_drift_m']} m")
        add_acceptance_check(checks, "moving_distractor_vehicles", moving_vehicles >= int(acceptance["minimum_moving_vehicles"]), moving_vehicles, f">={acceptance['minimum_moving_vehicles']}")
        add_acceptance_check(checks, "moving_pedestrians", moving_pedestrians >= int(acceptance["minimum_moving_pedestrians"]), moving_pedestrians, f">={acceptance['minimum_moving_pedestrians']}")
        for device in ("uav", "ugv"):
            add_acceptance_check(checks, f"{device}_rgb_is_dynamic", unique_rgb[device] >= float(acceptance["minimum_unique_rgb_fraction"]), unique_rgb[device], f">={acceptance['minimum_unique_rgb_fraction']}")
            minimum_finite = min(depth_finite[device]) if depth_finite[device] else 0.0
            add_acceptance_check(checks, f"{device}_depth_is_finite", minimum_finite >= 0.999, minimum_finite, ">=0.999")

        summary = {
            "duration_seconds": duration,
            "fixed_delta_seconds": fixed_delta,
            "sensor_tick_seconds": sensor_tick,
            "expected_sensor_frames": expected_frames,
            "common_frames": len(frame_rows),
            "common_frame_ratio": common_ratio,
            "max_frame_spread": max_frame_spread,
            "ugv_path_m": ugv_path,
            "uav_path_m": uav_path,
            "target_drift_m": target_path,
            "uav_tracking_p95_m": uav_follow_p95,
            "moving_distractor_vehicles": moving_vehicles,
            "moving_pedestrians": moving_pedestrians,
            "vehicle_displacements_m": vehicle_displacements,
            "pedestrian_displacements_m": pedestrian_displacements,
            "unique_rgb_ratio": unique_rgb,
            "minimum_depth_finite_fraction": {
                device: min(values) if values else 0.0 for device, values in depth_finite.items()
            },
        }
        report.update(
            {
                "completed_at": s0.now_utc(),
                "status": "PASS" if all(item["passed"] for item in checks) else "FAIL",
                "checks": checks,
                "summary": summary,
            }
        )
        scene_manifest = {
            "experiment": {
                "id": resolved["experiment_id"],
                "type": resolved["experiment_type"],
                "random_seed": seed,
            },
            "map": map_name,
            "region": region,
            "actors": initial_actor_manifest,
            "sensor_streams": list(stream_specs),
            "output_layout": {key: str(value) for key, value in run_dirs.items()},
        }
        s0.json_dump(run_dirs["run"] / "scene_manifest.json", scene_manifest)
        s0.json_dump(run_dirs["run"] / "validation_report.json", report)

        if frame_rows:
            draw_dynamic_map(world_map, region, paths_xyz, actor_tracks, run_dirs["preview"] / "trajectory_map.png")
            draw_alignment(frame_rows, expected_frames, run_dirs["preview"] / "synchronization_plot.png")
            draw_sensor_composite(run_dirs, int(frame_rows[0]["frame"]), int(frame_rows[-1]["frame"]), summary)

        readme = f"""# CI-E1 Dynamic Sensor Acceptance Output

- Status: **{report['status']}**
- Experiment: `{resolved['experiment_id']}`
- Map/region: `{map_name}` / `{region['region_id']}`
- Duration: {duration:.1f} s
- Four-stream common frames: {len(frame_rows)} / {expected_frames} ({common_ratio:.1%})
- UGV path: {ugv_path:.2f} m
- UAV path: {uav_path:.2f} m
- Moving distractor vehicles: {moving_vehicles}
- Moving pedestrians: {moving_pedestrians}

## Key outputs

- `validation_report.json`: machine-readable acceptance result
- `scene_manifest.json`: actors, sensors, map and output layout
- `preview/sensor_composite.png`: UAV/UGV RGB and depth acceptance panel
- `preview/trajectory_map.png`: complete two-dimensional dynamic scene
- `preview/synchronization_plot.png`: four-stream synchronization evidence
- `synchronization/frame_index.csv`: exact frame mapping
- `trajectories/`: UGV, UAV and target trajectories
- `actors/actor_states.jsonl`: per-frame ground-truth state for all actors
- `sensors/`: RGB, encoded depth, metric depth and metadata by device
"""
        (run_dirs["run"] / "README.md").write_text(readme, encoding="utf-8")
        log(f"CI-E1 result: {report['status']}; output: {run_dirs['run']}")
        return 0 if report["status"] == "PASS" else 2

    except Exception as exc:
        report.update({"completed_at": s0.now_utc(), "status": "ERROR", "error": repr(exc)})
        s0.json_dump(run_dirs["run"] / "validation_report.json", report)
        log(f"ERROR: {exc!r}")
        raise
    finally:
        for sensor in sensors:
            try:
                sensor.stop()
            except Exception:
                pass
        for controller in walker_controllers:
            try:
                controller.stop()
            except Exception:
                pass
        if traffic_manager is not None:
            try:
                traffic_manager.set_synchronous_mode(False)
            except Exception:
                pass
        if world is not None and original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass
        for actor in reversed(owned_actors):
            try:
                if actor.is_alive:
                    actor.destroy()
            except Exception:
                pass
        if airsim_client is not None:
            try:
                vehicle_name = resolved["actors"]["uav"].get("vehicle_name", "")
                airsim_client.armDisarm(False, vehicle_name=vehicle_name)
                airsim_client.enableApiControl(False, vehicle_name=vehicle_name)
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
