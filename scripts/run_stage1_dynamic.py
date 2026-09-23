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
import cv2
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
from communication.oracle_channel import OracleChannel  # noqa: E402
from communication.semantic_channel import SemanticChannel  # noqa: E402
from navigation.ugv_oracle_planner import plan_route_from_message  # noqa: E402
from perception.candidate_pipeline import CandidatePipeline, PipelineConfig  # noqa: E402
from perception.target_ranker import (  # noqa: E402
    LocalTargetVerifier,
    LocalVerificationConfig,
    TargetRanker,
    TransmissionGateConfig,
)
from task.oracle_state_machine import OracleTaskStateMachine  # noqa: E402
from task.perception_state_machine import PerceptionTaskStateMachine  # noqa: E402


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


def route_completion_metrics(
    actual_points: list[list[float]], route: list[list[float]], waypoint_radius_m: float
) -> dict[str, int]:
    """Count consecutively reached waypoints, segments, and horizontal foldbacks."""
    next_waypoint = 0
    for actual in actual_points:
        while next_waypoint < len(route) and math.dist(actual, route[next_waypoint]) <= waypoint_radius_m:
            next_waypoint += 1
    segments_completed = max(0, next_waypoint - 1)
    horizontal_directions: list[int] = []
    for start, end in list(zip(route, route[1:]))[:segments_completed]:
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        if abs(dx) > abs(dy):
            horizontal_directions.append(1 if dx > 0 else -1)
    foldbacks = sum(a != b for a, b in zip(horizontal_directions, horizontal_directions[1:]))
    return {
        "waypoints_reached": next_waypoint,
        "segments_completed": segments_completed,
        "foldbacks_completed": foldbacks,
    }


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


def forward_obstacle_clearance(
    ego: Any, obstacles: list[Any], lateral_limit_m: float = 3.0
) -> float:
    """Ground-truth longitudinal clearance used only as a simulator safety shield."""
    transform = ego.get_transform()
    origin = transform.location
    forward = transform.get_forward_vector()
    right_x, right_y = -forward.y, forward.x
    best = float("inf")
    for actor in obstacles:
        if actor is None or not actor.is_alive or actor.id == ego.id:
            continue
        location = actor.get_location()
        dx, dy = location.x - origin.x, location.y - origin.y
        longitudinal = dx * forward.x + dy * forward.y
        lateral = abs(dx * right_x + dy * right_y)
        if longitudinal > 0.0 and lateral <= lateral_limit_m:
            best = min(best, float(longitudinal))
    return best


def apply_safe_route_control(
    vehicle: Any,
    route: list[list[float]],
    target_speed_mps: float,
    target_actor: Any,
    obstacles: list[Any],
    safety: dict[str, Any],
) -> dict[str, float | str]:
    import carla

    target_distance = horizontal_distance(s0.xyz(vehicle.get_location()), s0.xyz(target_actor.get_location()))
    clearance = forward_obstacle_clearance(
        vehicle, obstacles, float(safety.get("forward_corridor_half_width_m", 3.0))
    )
    stop_distance = float(safety.get("target_stop_distance_m", 5.0))
    emergency_distance = float(safety.get("emergency_brake_distance_m", 7.0))
    slow_distance = float(safety.get("slowdown_distance_m", 14.0))
    mode = "cruise"
    if target_distance <= stop_distance + 1.2 or clearance <= emergency_distance:
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=False))
        mode = "emergency_stop" if clearance <= emergency_distance else "target_stop"
    else:
        commanded_speed = target_speed_mps
        if target_distance < slow_distance:
            commanded_speed = min(commanded_speed, max(1.0, (target_distance - stop_distance) * 0.70))
            mode = "target_approach"
        if clearance < slow_distance:
            commanded_speed = min(commanded_speed, max(0.8, (clearance - emergency_distance) * 0.75))
            mode = "obstacle_approach"
        follow_route(vehicle, route, commanded_speed)
    return {
        "ugv_target_distance_m": float(target_distance),
        "ugv_forward_clearance_m": float(clearance),
        "ugv_safety_mode": mode,
    }


def apply_safe_route_control_to_position(
    vehicle: Any,
    route: list[list[float]],
    target_speed_mps: float,
    target_xyz: list[float],
    obstacles: list[Any],
    safety: dict[str, Any],
) -> dict[str, float | str]:
    """Oracle controller that consumes only the position delivered in a message."""
    import carla

    target_distance = horizontal_distance(s0.xyz(vehicle.get_location()), target_xyz)
    clearance = forward_obstacle_clearance(
        vehicle, obstacles, float(safety.get("forward_corridor_half_width_m", 3.0))
    )
    stop_distance = float(safety.get("target_stop_distance_m", 5.0))
    emergency_distance = float(safety.get("emergency_brake_distance_m", 6.0))
    slow_distance = float(safety.get("slowdown_distance_m", 14.0))
    mode = "cruise"
    if target_distance <= stop_distance + 1.2 or clearance <= emergency_distance:
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=False))
        mode = "emergency_stop" if clearance <= emergency_distance else "target_stop"
    else:
        commanded_speed = target_speed_mps
        if target_distance < slow_distance:
            commanded_speed = min(commanded_speed, max(1.0, (target_distance - stop_distance) * 0.70))
            mode = "target_approach"
        if clearance < slow_distance:
            commanded_speed = min(commanded_speed, max(0.8, (clearance - emergency_distance) * 0.75))
            mode = "obstacle_approach"
        follow_route(vehicle, route, commanded_speed)
    return {
        "ugv_target_distance_m": float(target_distance),
        "ugv_forward_clearance_m": float(clearance),
        "ugv_safety_mode": mode,
    }


def oracle_target_in_nadir_view(
    uav_xyz: list[float],
    target_xyz: list[float],
    width: int,
    height: int,
    horizontal_fov_degrees: float,
) -> tuple[bool, list[float]]:
    """Oracle frustum test for the fixed-north nadir camera used by S0."""
    vertical_distance = float(uav_xyz[2] - 1.5 - (target_xyz[2] + 1.2))
    if vertical_distance <= 0.0:
        return False, [float("nan"), float("nan")]
    focal = width / (2.0 * math.tan(math.radians(horizontal_fov_degrees) / 2.0))
    u = width / 2.0 + focal * (target_xyz[1] - uav_xyz[1]) / vertical_distance
    v = height / 2.0 + focal * (target_xyz[0] - uav_xyz[0]) / vertical_distance
    margin = 4.0
    visible = margin <= u < width - margin and margin <= v < height - margin
    return bool(visible), [float(u), float(v)]


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
        "safety": run_dir / "safety",
        "task": run_dir / "task",
        "communication": run_dir / "communication",
        "planning": run_dir / "planning",
        "perception": run_dir / "perception",
        "evaluation": run_dir / "ground_truth" / "evaluation_only",
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


def save_depth(
    image: Any, raw_destination: Path, metres_destination: Path, colour_destination: Path
) -> tuple[dict[str, float], np.ndarray]:
    encoded = bgra_to_rgb(image.raw_data, image.width, image.height)
    metres = carla_depth_to_metres(image.raw_data, image.width, image.height)
    Image.fromarray(encoded).save(raw_destination)
    np.save(metres_destination, metres.astype(np.float32, copy=False))
    colourise_depth(metres, display_max_m=100.0).save(colour_destination)
    finite = np.isfinite(metres)
    height, width = metres.shape
    central = metres[int(height * 0.4) : int(height * 0.6), int(width * 0.4) : int(width * 0.6)]
    central_finite = central[np.isfinite(central)]
    statistics = {
        "minimum_m": float(np.nanmin(metres)),
        "maximum_m": float(np.nanmax(metres)),
        "mean_m": float(np.nanmean(metres)),
        "finite_fraction": float(np.mean(finite)),
        "central_clearance_p01_m": float(np.percentile(central_finite, 1)) if central_finite.size else 0.0,
    }
    return statistics, metres


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )


def append_jsonl(path: Path, row: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, ensure_ascii=False) + "\n")


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
    experiment_label: str = "CI-E1 Dynamic Scene",
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
    axis.set_title(f"{experiment_label} — Town10HD Zone A", fontsize=16, weight="bold", pad=14)
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


def draw_alignment(
    frame_rows: list[dict[str, Any]],
    expected_count: int,
    output_path: Path,
    experiment_label: str = "CI-E1",
) -> None:
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
    fig.suptitle(f"{experiment_label} Four-stream Synchronization — {len(frame_rows)}/{expected_count} expected frames", fontsize=14, weight="bold")
    fig.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def draw_sensor_composite(run_dirs: dict[str, Path], first_frame: int, last_frame: int, summary: dict[str, Any]) -> None:
    canvas = Image.new("RGB", (1640, 1030), "#f5f7fa")
    draw = ImageDraw.Draw(canvas)
    font_title = s0.load_font(36, bold=True)
    font_heading = s0.load_font(24, bold=True)
    font_body = s0.load_font(20)
    title = (
        "S0 ORACLE RGB + Depth Acceptance — NOT PERCEPTION"
        if "oracle_closed_loop" in summary
        else (
            "S1 PERCEPTION RGB + Depth Closed-Loop Acceptance"
            if "perception_closed_loop" in summary
            else "CI-E1 Dynamic RGB + Depth Acceptance"
        )
    )
    draw.text(
        (42, 30),
        title,
        fill="#c62828" if "oracle_closed_loop" in summary else "#152238",
        font=font_title,
    )
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
    experiment_type = resolved.get("experiment_type")
    schema_file = {
        "oracle_closed_loop_acceptance": "oracle_experiment_schema.json",
        "perception_closed_loop_acceptance": "perception_experiment_schema.json",
    }.get(experiment_type, "dynamic_experiment_schema.json")
    schema_errors = validate_schema(resolved, PROJECT_ROOT / "configs" / "schemas" / schema_file)
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
    oracle_mode = experiment_type == "oracle_closed_loop_acceptance"
    perception_mode = experiment_type == "perception_closed_loop_acceptance"
    closed_loop_mode = oracle_mode or perception_mode
    oracle_cfg = resolved.get("oracle", {})
    perception_cfg = resolved.get("perception", {})
    closed_loop_cfg = oracle_cfg if oracle_mode else perception_cfg
    negative_control = bool(closed_loop_cfg.get("negative_control", False))
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
    safety_sensors: list[Any] = []
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
        if "uav_altitude_m" in dynamic_cfg:
            for point in uav_route:
                point[2] = float(dynamic_cfg["uav_altitude_m"])
        uav_route_length_m = cumulative_distance(uav_route)
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
            airsim_client.simSetVehiclePose(
                airsim.Pose(ned, orientation),
                bool(dynamic_cfg.get("uav_ignore_collision", True)),
                vehicle_name=vehicle_name,
            )

        def camera_yaw(path_yaw: float) -> float:
            if dynamic_cfg.get("uav_camera_yaw_mode") == "world_fixed":
                return float(dynamic_cfg.get("uav_camera_yaw_degrees", 0.0))
            return path_yaw

        initial_uav, initial_uav_yaw = interpolate_polyline(uav_route, 0.0)
        set_drone_pose(initial_uav, initial_uav_yaw)
        time.sleep(0.15)
        initial_collision_info = airsim_client.simGetCollisionInfo(vehicle_name=vehicle_name)
        baseline_uav_collision_timestamp = int(
            getattr(initial_collision_info, "time_stamp", 0) or 0
        )

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
                    carla.Rotation(pitch=-90.0, yaw=camera_yaw(initial_uav_yaw)),
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

        ugv_collision_events: list[dict[str, Any]] = []
        collision_bp = blueprint_library.find("sensor.other.collision")
        collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=ugv)
        collision_sensor.listen(
            lambda event: ugv_collision_events.append(
                {
                    "frame": int(event.frame),
                    "other_actor_id": int(event.other_actor.id),
                    "other_type_id": str(event.other_actor.type_id),
                    "normal_impulse": s0.xyz(event.normal_impulse),
                }
            )
        )
        safety_sensors.append(collision_sensor)
        owned_actors.append(collision_sensor)

        roles: list[tuple[str, Any]] = [("ugv", ugv), ("uav", drone), ("target", target)]
        roles.extend(zip(distractor_roles, distractors))
        roles.extend(zip(pedestrian_roles, walkers))
        initial_actor_manifest = [s0.actor_record(actor, role, role.split("_")[0]) for role, actor in roles]
        s0.json_dump(run_dirs["actors"] / "initial_actor_states.json", initial_actor_manifest)

        task_machine: OracleTaskStateMachine | PerceptionTaskStateMachine | None = None
        oracle_channel: OracleChannel | SemanticChannel | None = None
        oracle_message_sent: dict[str, Any] | None = None
        received_oracle_message: dict[str, Any] | None = None
        active_ugv_route: list[list[float]] = route
        planning_summary: dict[str, Any] = {}
        communication_events: list[dict[str, Any]] = []
        task_timeline: list[dict[str, Any]] = []
        oracle_usage_audit = {
            "oracle": bool(oracle_mode),
            "perception_enabled": bool(perception_mode and perception_cfg.get("enabled", True)),
            "target_truth_scope": (
                "frustum trigger, single message payload, post-run evaluation"
                if oracle_mode
                else "recording and post-run evaluation only"
            ),
            "visibility_truth_reads": 0,
            "message_payload_truth_reads": 0,
            "ugv_controller_direct_target_actor_reads": 0,
            "perception_runtime_target_truth_reads": 0,
        }
        visible_tick_streak = 0
        arrival_hold_ticks = 0
        arrival_hold_required_ticks = max(
            1, int(round(float(closed_loop_cfg.get("arrival_hold_seconds", 2.0)) / fixed_delta))
        )
        planned_frame: int | None = None
        received_frame: int | None = None
        initial_ugv_xyz = s0.xyz(ugv.get_location())
        maximum_pre_message_ugv_displacement = 0.0
        uav_pipeline: CandidatePipeline | None = None
        ugv_pipeline: CandidatePipeline | None = None
        target_ranker: TargetRanker | None = None
        local_verifier: LocalTargetVerifier | None = None
        perception_candidate_rows: list[dict[str, Any]] = []
        local_confirmation_rows: list[dict[str, Any]] = []
        transmitted_candidate: dict[str, Any] | None = None
        message_localization_error_m: float | None = None
        wrong_target_messages = 0
        if oracle_mode:
            snapshot = world.get_snapshot()
            task_machine = OracleTaskStateMachine(
                task_id=str(resolved["task"]["task_id"]),
                instruction=str(resolved["task"]["instruction"]),
            )
            task_machine.transition(
                "SEARCHING",
                int(snapshot.frame),
                float(snapshot.timestamp.elapsed_seconds),
                "instruction_loaded_and_uav_route_started",
                oracle=True,
                parsed_goal=resolved["task"]["parsed_goal"],
            )
            oracle_channel = OracleChannel(
                enabled=bool(oracle_cfg.get("enabled", True)),
                delay_ticks=int(oracle_cfg.get("delivery_delay_ticks", 1)),
            )
            active_ugv_route = []
        elif perception_mode:
            snapshot = world.get_snapshot()
            task_machine = PerceptionTaskStateMachine(
                task_id=str(resolved["task"]["task_id"]),
                instruction=str(resolved["task"]["instruction"]),
            )
            task_machine.transition(
                "SEARCHING",
                int(snapshot.frame),
                float(snapshot.timestamp.elapsed_seconds),
                "instruction_loaded_and_uav_route_started",
                oracle=False,
                perception_enabled=bool(perception_cfg.get("enabled", True)),
                parsed_goal=resolved["task"]["parsed_goal"],
            )
            oracle_channel = SemanticChannel(
                enabled=bool(perception_cfg.get("enabled", True)),
                delay_ticks=int(perception_cfg.get("delivery_delay_ticks", 1)),
            )
            active_ugv_route = []
            if bool(perception_cfg.get("enabled", True)):
                model_path = (PROJECT_ROOT / str(perception_cfg["model_path"])).resolve()
                if not model_path.exists():
                    raise FileNotFoundError(f"S1 detector weight is missing: {model_path}")
                pipeline_config = PipelineConfig(
                    model_path=model_path,
                    image_width=int(sensor_config["width"]),
                    image_height=int(sensor_config["height"]),
                    fov_degrees=float(sensor_config["fov_degrees"]),
                    detector_confidence=float(perception_cfg.get("detector_confidence", 0.03)),
                    detector_image_size=int(perception_cfg.get("detector_image_size", 1280)),
                    red_ratio_threshold=float(perception_cfg.get("red_ratio_threshold", 0.10)),
                    temporal_window=int(perception_cfg.get("temporal_window", 5)),
                    temporal_hits=int(perception_cfg.get("temporal_hits", 4)),
                    association_radius_m=float(perception_cfg.get("association_radius_m", 5.0)),
                )
                uav_pipeline = CandidatePipeline(pipeline_config)
                ugv_pipeline = CandidatePipeline(pipeline_config, model=uav_pipeline.model)
                gate = perception_cfg["transmission_gate"]
                target_ranker = TargetRanker(
                    TransmissionGateConfig(
                        minimum_red_ratio=float(gate["minimum_red_ratio"]),
                        minimum_temporal_hits=int(gate["minimum_temporal_hits"]),
                        minimum_detector_score=float(gate["minimum_detector_score"]),
                        allowed_sources=tuple(gate["allowed_sources"]),
                        minimum_world_z_m=float(gate["minimum_world_z_m"]),
                        maximum_world_z_m=float(gate["maximum_world_z_m"]),
                    )
                )
                verification = perception_cfg["local_verification"]
                local_verifier = LocalTargetVerifier(
                    LocalVerificationConfig(
                        minimum_red_ratio=float(verification["minimum_red_ratio"]),
                        minimum_temporal_hits=int(verification["minimum_temporal_hits"]),
                        minimum_detector_score=float(verification["minimum_detector_score"]),
                        association_radius_m=float(verification["association_radius_m"]),
                        minimum_van_like_score=float(verification["minimum_van_like_score"]),
                    )
                )

        warmup_ticks = int(dynamic_cfg.get("sensor_warmup_ticks", 0)) if closed_loop_mode else 0
        if warmup_ticks > 0:
            log(f"Warming up four camera streams for {warmup_ticks} fixed ticks")
            ugv.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
            for _ in range(warmup_ticks):
                world.tick()
                time.sleep(float(dynamic_cfg.get("render_settle_seconds", 0.035)))
                for packet_queue in queues.values():
                    s0.drain_all(packet_queue)
            log("Sensor warm-up complete; warm-up packets discarded")

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
        uav_central_clearances: list[float] = []
        follow_errors: list[float] = []
        safety_rows: list[dict[str, Any]] = []
        safety_by_frame: dict[int, dict[str, Any]] = {}
        uav_collision_events: list[dict[str, Any]] = []
        seen_uav_collision_timestamps: set[int] = (
            {baseline_uav_collision_timestamp} if baseline_uav_collision_timestamp else set()
        )
        world_state_by_frame: dict[int, dict[str, Any]] = {}
        saved_frames: set[int] = set()
        started_sim_time: float | None = None
        sensor_health_checked = False

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
                        carla.Rotation(pitch=-90.0, yaw=camera_yaw(desired_uav_yaw)),
                    )
                )
            initial_target_distance = (
                horizontal_distance(s0.xyz(ugv.get_location()), received_oracle_message["target_world_position_xyz"])
                if perception_mode and received_oracle_message is not None
                else (
                    float("nan")
                    if perception_mode
                    else horizontal_distance(s0.xyz(ugv.get_location()), s0.xyz(target.get_location()))
                )
            )
            ugv_safety = {
                "ugv_target_distance_m": initial_target_distance,
                "ugv_forward_clearance_m": float("inf"),
                "ugv_safety_mode": "delayed",
            }
            if oracle_mode:
                pre_snapshot = world.get_snapshot()
                pre_frame = int(pre_snapshot.frame)
                pre_timestamp = float(pre_snapshot.timestamp.elapsed_seconds)
                oracle_target_xyz = s0.xyz(target.get_location())
                oracle_usage_audit["visibility_truth_reads"] += 1
                target_visible, target_uv = oracle_target_in_nadir_view(
                    desired_uav,
                    oracle_target_xyz,
                    int(sensor_config["width"]),
                    int(sensor_config["height"]),
                    float(sensor_config["fov_degrees"]),
                )
                visible_tick_streak = visible_tick_streak + 1 if target_visible else 0
                if (
                    bool(oracle_cfg.get("enabled", True))
                    and oracle_message_sent is None
                    and visible_tick_streak >= int(oracle_cfg.get("visibility_consecutive_ticks", 3))
                ):
                    assert task_machine is not None and oracle_channel is not None
                    task_machine.transition(
                        "ORACLE_TARGET_VISIBLE",
                        pre_frame,
                        pre_timestamp,
                        "target_in_uav_frustum",
                        projected_uv=target_uv,
                        consecutive_ticks=visible_tick_streak,
                    )
                    oracle_usage_audit["message_payload_truth_reads"] += 1
                    oracle_message_sent = oracle_channel.send(
                        {
                            "target_role": "inspection_target",
                            "target_category": "vehicle",
                            "target_subcategory": "van",
                            "target_color": "red",
                            "target_world_position_xyz": [float(value) for value in oracle_target_xyz],
                            "source_frame": pre_frame,
                            "source_timestamp": pre_timestamp,
                            "source_kind": "carla_ground_truth",
                        },
                        tick_index,
                        pre_frame,
                        pre_timestamp,
                    )
                    if oracle_message_sent is not None:
                        task_machine.transition(
                            "MESSAGE_SENT",
                            pre_frame,
                            pre_timestamp,
                            "oracle_channel_send",
                            message_id=oracle_message_sent["message_id"],
                        )
                        communication_events.append({"event": "sent", **oracle_message_sent})

                assert oracle_channel is not None and task_machine is not None
                delivered = oracle_channel.receive(tick_index, pre_frame, pre_timestamp)
                if delivered is not None and received_oracle_message is None:
                    received_oracle_message = delivered
                    received_frame = pre_frame
                    communication_events.append({"event": "received", **delivered})
                    task_machine.transition(
                        "TARGET_RECEIVED",
                        pre_frame,
                        pre_timestamp,
                        "oracle_message_delivered",
                        message_id=delivered["message_id"],
                    )
                    task_machine.transition(
                        "PLANNING",
                        pre_frame,
                        pre_timestamp,
                        "received_position_submitted_to_global_route_planner",
                        message_id=delivered["message_id"],
                    )
                    active_ugv_route = plan_route_from_message(
                        world_map,
                        ugv.get_location(),
                        delivered["target_world_position_xyz"],
                        standoff_m=float(oracle_cfg.get("standoff_m", 6.0)),
                    )
                    planned_frame = pre_frame
                    planning_summary = {
                        "oracle": True,
                        "route_source": "received_oracle_message",
                        "message_id": delivered["message_id"],
                        "received_frame": received_frame,
                        "planned_frame": planned_frame,
                        "target_world_position_xyz": delivered["target_world_position_xyz"],
                        "route_points": len(active_ugv_route),
                        "route_length_m": cumulative_distance(active_ugv_route),
                        "route_final_position_xyz": active_ugv_route[-1],
                    }
                    task_machine.transition(
                        "NAVIGATING",
                        pre_frame,
                        pre_timestamp,
                        "global_route_planner_route_ready",
                        route_points=len(active_ugv_route),
                    )

                current_displacement = horizontal_distance(initial_ugv_xyz, s0.xyz(ugv.get_location()))
                if received_oracle_message is None:
                    maximum_pre_message_ugv_displacement = max(
                        maximum_pre_message_ugv_displacement, current_displacement
                    )
                    ugv.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                    ugv_safety["ugv_safety_mode"] = "waiting_for_oracle_message"
                elif task_machine.ugv_arrived:
                    ugv.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                    ugv_safety["ugv_safety_mode"] = "arrived_hold"
                    ugv_safety["ugv_target_distance_m"] = horizontal_distance(
                        s0.xyz(ugv.get_location()), received_oracle_message["target_world_position_xyz"]
                    )
                else:
                    ugv_safety = apply_safe_route_control_to_position(
                        ugv,
                        active_ugv_route,
                        float(dynamic_cfg["ugv_target_speed_mps"]),
                        received_oracle_message["target_world_position_xyz"],
                        [*distractors, *walkers],
                        dynamic_cfg.get("ugv_safety", {}),
                    )
            elif perception_mode:
                pre_snapshot = world.get_snapshot()
                pre_frame = int(pre_snapshot.frame)
                pre_timestamp = float(pre_snapshot.timestamp.elapsed_seconds)
                assert oracle_channel is not None and isinstance(task_machine, PerceptionTaskStateMachine)
                delivered = oracle_channel.receive(tick_index, pre_frame, pre_timestamp)
                if delivered is not None and received_oracle_message is None:
                    received_oracle_message = delivered
                    received_frame = pre_frame
                    communication_events.append({"event": "received", **delivered})
                    append_jsonl(
                        run_dirs["perception"] / "runtime_events.jsonl",
                        {"event": "message_received", **delivered},
                    )
                    task_machine.transition(
                        "TARGET_RECEIVED",
                        pre_frame,
                        pre_timestamp,
                        "semantic_message_delivered",
                        message_id=delivered["message_id"],
                    )
                    task_machine.transition(
                        "PLANNING_TO_OBSERVATION_POINT",
                        pre_frame,
                        pre_timestamp,
                        "received_perception_position_submitted_to_route_planner",
                        message_id=delivered["message_id"],
                    )
                    active_ugv_route = plan_route_from_message(
                        world_map,
                        ugv.get_location(),
                        delivered["target_world_position_xyz"],
                        standoff_m=float(perception_cfg.get("standoff_m", 6.0)),
                        reference_route=route,
                    )
                    planned_frame = pre_frame
                    planning_summary = {
                        "oracle": False,
                        "perception_enabled": True,
                        "route_source": "received_perception_message",
                        "message_id": delivered["message_id"],
                        "received_frame": received_frame,
                        "planned_frame": planned_frame,
                        "target_world_position_xyz": delivered["target_world_position_xyz"],
                        "route_points": len(active_ugv_route),
                        "route_length_m": cumulative_distance(active_ugv_route),
                        "route_final_position_xyz": active_ugv_route[-1],
                    }
                    task_machine.transition(
                        "NAVIGATING",
                        pre_frame,
                        pre_timestamp,
                        "global_route_planner_route_ready",
                        route_points=len(active_ugv_route),
                    )

                current_displacement = horizontal_distance(initial_ugv_xyz, s0.xyz(ugv.get_location()))
                if received_oracle_message is None:
                    maximum_pre_message_ugv_displacement = max(
                        maximum_pre_message_ugv_displacement, current_displacement
                    )
                    ugv.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                    ugv_safety["ugv_safety_mode"] = "waiting_for_perception_message"
                elif task_machine.ugv_arrived:
                    ugv.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                    ugv_safety["ugv_safety_mode"] = "arrived_hold"
                    ugv_safety["ugv_target_distance_m"] = horizontal_distance(
                        s0.xyz(ugv.get_location()), received_oracle_message["target_world_position_xyz"]
                    )
                else:
                    message_distance = horizontal_distance(
                        s0.xyz(ugv.get_location()), received_oracle_message["target_world_position_xyz"]
                    )
                    if (
                        not task_machine.local_confirmation_started
                        and message_distance <= float(perception_cfg.get("local_confirmation_start_distance_m", 25.0))
                    ):
                        task_machine.start_local_confirmation(
                            pre_frame,
                            pre_timestamp,
                            message_distance_m=message_distance,
                        )
                    if (
                        not task_machine.target_verified
                        and message_distance <= float(perception_cfg.get("unverified_stop_distance_m", 8.0))
                    ):
                        ugv.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=False))
                        ugv_safety = {
                            "ugv_target_distance_m": message_distance,
                            "ugv_forward_clearance_m": float("inf"),
                            "ugv_safety_mode": "waiting_for_local_confirmation",
                        }
                    else:
                        commanded_speed = float(dynamic_cfg["ugv_target_speed_mps"])
                        if task_machine.local_confirmation_started and not task_machine.target_verified:
                            commanded_speed = min(commanded_speed, 2.0)
                        ugv_safety = apply_safe_route_control_to_position(
                            ugv,
                            active_ugv_route,
                            commanded_speed,
                            received_oracle_message["target_world_position_xyz"],
                            [*distractors, *walkers],
                            dynamic_cfg.get("ugv_safety", {}),
                        )
            else:
                ugv_start_delay = float(dynamic_cfg.get("ugv_start_delay_seconds", 0.0))
                if sim_elapsed < ugv_start_delay:
                    ugv.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                else:
                    safety_cfg = dynamic_cfg.get("ugv_safety", {})
                    if safety_cfg.get("enabled", False):
                        ugv_safety = apply_safe_route_control(
                            ugv,
                            route,
                            float(dynamic_cfg["ugv_target_speed_mps"]),
                            target,
                            [target, *distractors, *walkers],
                            safety_cfg,
                        )
                    else:
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
            if closed_loop_mode and task_machine is not None:
                ugv_state = state_by_role.get("ugv", {})
                if received_oracle_message is not None and not task_machine.ugv_arrived:
                    message_distance = horizontal_distance(
                        [ugv_state.get("x", 0.0), ugv_state.get("y", 0.0)],
                        received_oracle_message["target_world_position_xyz"],
                    )
                    within_standoff = 3.0 <= message_distance <= 7.0
                    stopped = float(ugv_state.get("speed_mps", float("inf"))) <= float(
                        acceptance.get("maximum_final_ugv_speed_mps", 0.2)
                    )
                    verified_for_arrival = oracle_mode or bool(getattr(task_machine, "target_verified", False))
                    arrival_hold_ticks = (
                        arrival_hold_ticks + 1 if within_standoff and stopped and verified_for_arrival else 0
                    )
                    if arrival_hold_ticks >= arrival_hold_required_ticks:
                        task_machine.mark_ugv_arrived(
                            frame,
                            elapsed,
                            target_distance_m=message_distance,
                            speed_mps=float(ugv_state.get("speed_mps", 0.0)),
                            hold_seconds=arrival_hold_ticks * fixed_delta,
                        )
                if sim_elapsed * float(dynamic_cfg["uav_speed_mps"]) >= uav_route_length_m - 0.05:
                    task_machine.mark_uav_route_complete(
                        frame,
                        elapsed,
                        planned_route_length_m=uav_route_length_m,
                    )
                task_timeline.append(
                    {
                        "frame": int(frame),
                        "timestamp": elapsed,
                        "state": task_machine.state,
                        "oracle": bool(oracle_mode),
                        "perception_enabled": bool(perception_mode),
                        "message_id": (
                            received_oracle_message or oracle_message_sent or {}
                        ).get("message_id", ""),
                        "message_received": received_oracle_message is not None,
                        "local_confirmation_started": bool(
                            getattr(task_machine, "local_confirmation_started", False)
                        ),
                        "target_verified": bool(getattr(task_machine, "target_verified", False)),
                        "ugv_x": float(ugv_state.get("x", float("nan"))),
                        "ugv_y": float(ugv_state.get("y", float("nan"))),
                        "ugv_speed_mps": float(ugv_state.get("speed_mps", float("nan"))),
                        "uav_x": float(actual_uav["x"]) if actual_uav else float("nan"),
                        "uav_y": float(actual_uav["y"]) if actual_uav else float("nan"),
                        "uav_waypoint_progress_m": min(
                            uav_route_length_m,
                            sim_elapsed * float(dynamic_cfg["uav_speed_mps"]),
                        ),
                        "arrival_hold_seconds": arrival_hold_ticks * fixed_delta,
                    }
                )
            if tick_index % 2 == 0:
                collision_info = airsim_client.simGetCollisionInfo(vehicle_name=vehicle_name)
                collision_timestamp = int(getattr(collision_info, "time_stamp", 0) or 0)
                if bool(collision_info.has_collided) and collision_timestamp not in seen_uav_collision_timestamps:
                    seen_uav_collision_timestamps.add(collision_timestamp)
                    uav_collision_events.append(
                        {
                            "frame": int(frame),
                            "timestamp": collision_timestamp,
                            "object_name": str(getattr(collision_info, "object_name", "")),
                            "object_id": int(getattr(collision_info, "object_id", -1)),
                        }
                    )
            safety_row = {
                "frame": int(frame),
                "timestamp": elapsed,
                **ugv_safety,
                "ugv_speed_mps": float(state_by_role.get("ugv", {}).get("speed_mps", 0.0)),
                "ugv_collision_count": len(ugv_collision_events),
                "uav_altitude_m": float(actual_uav["z"]) if actual_uav else float("nan"),
                "uav_collision_count": len(uav_collision_events),
                "uav_central_clearance_m": float("nan"),
            }
            safety_rows.append(safety_row)
            safety_by_frame[frame] = safety_row

            # GPU camera callbacks are asynchronous even while the CARLA world is
            # synchronous.  A short render barrier prevents long runs from outrunning
            # the four camera streams and dropping most of the second half.
            time.sleep(float(dynamic_cfg.get("render_settle_seconds", 0.035)))
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
                depth_arrays: dict[str, np.ndarray] = {}
                metadata_by_device: dict[str, dict[str, Any]] = {}
                for device in ("uav", "ugv"):
                    rgb_packet = packets[f"{device}_rgb"]
                    depth_packet = packets[f"{device}_depth"]
                    rgb_path = run_dirs[f"{device}_rgb"] / f"{common_frame:08d}.png"
                    depth_raw_path = run_dirs[f"{device}_depth_raw"] / f"{common_frame:08d}.png"
                    depth_m_path = run_dirs[f"{device}_depth_metres"] / f"{common_frame:08d}.npy"
                    depth_colour_path = run_dirs[f"{device}_depth_colour"] / f"{common_frame:08d}.png"
                    digest, rgb_array = save_rgb(rgb_packet, rgb_path)
                    depth_stats, depth_array = save_depth(
                        depth_packet, depth_raw_path, depth_m_path, depth_colour_path
                    )
                    rgb_hashes[device].append(digest)
                    rgb_arrays[device] = rgb_array
                    depth_arrays[device] = depth_array
                    depth_finite[device].append(depth_stats["finite_fraction"])
                    if device == "uav":
                        uav_central_clearances.append(depth_stats["central_clearance_p01_m"])
                        if common_frame in safety_by_frame:
                            safety_by_frame[common_frame]["uav_central_clearance_m"] = depth_stats[
                                "central_clearance_p01_m"
                            ]
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
                    metadata_by_device[device] = metadata
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
                if (
                    perception_mode
                    and bool(perception_cfg.get("enabled", True))
                    and (len(frame_rows) - 1)
                    % max(1, int(perception_cfg.get("inference_every_sensor_frames", 5)))
                    == 0
                ):
                    assert (
                        uav_pipeline is not None
                        and ugv_pipeline is not None
                        and target_ranker is not None
                        and local_verifier is not None
                        and isinstance(task_machine, PerceptionTaskStateMachine)
                        and oracle_channel is not None
                    )
                    requests: list[tuple[str, CandidatePipeline]] = []
                    if oracle_message_sent is None:
                        requests.append(("uav", uav_pipeline))
                    if (
                        received_oracle_message is not None
                        and task_machine.local_confirmation_started
                        and not task_machine.target_verified
                    ):
                        requests.append(("ugv", ugv_pipeline))
                    if requests:
                        inference_images = [
                            cv2.cvtColor(rgb_arrays[device], cv2.COLOR_RGB2BGR)
                            for device, _ in requests
                        ]
                        detector_results = uav_pipeline.infer_images(inference_images)
                        for (device, pipeline), detector_result, image_bgr in zip(
                            requests, detector_results, inference_images
                        ):
                            metadata = metadata_by_device[device]
                            candidates = pipeline.process(
                                common_frame,
                                float(metadata["timestamp"]),
                                image_bgr,
                                depth_arrays[device],
                                metadata["sensor_transform"],
                                detector_result,
                            )
                            if device == "uav":
                                selected = target_ranker.select(candidates)
                                perception_candidate_rows.append(
                                    {
                                        "frame": int(common_frame),
                                        "timestamp": float(metadata["timestamp"]),
                                        "selected_candidate_id": None if selected is None else selected["candidate_id"],
                                        "candidates": candidates,
                                    }
                                )
                                if selected is not None and oracle_message_sent is None:
                                    append_jsonl(
                                        run_dirs["perception"] / "runtime_events.jsonl",
                                        {
                                            "event": "uav_candidate_selected",
                                            "frame": int(common_frame),
                                            "timestamp": float(metadata["timestamp"]),
                                            "candidate": selected,
                                        },
                                    )
                                    task_machine.transition(
                                        "UAV_CANDIDATE_CONFIRMED",
                                        common_frame,
                                        float(metadata["timestamp"]),
                                        "instruction_conditioned_candidate_gate",
                                        candidate_id=selected["candidate_id"],
                                        track_id=selected["track_id"],
                                        instruction_match_score=selected["instruction_match_score"],
                                    )
                                    transmitted_candidate = dict(selected)
                                    oracle_message_sent = oracle_channel.send(
                                        {
                                            "target_category": "vehicle",
                                            "target_subcategory_hypothesis": "van_like",
                                            "target_color": "red",
                                            "target_world_position_xyz": [
                                                float(value) for value in selected["world_position_xyz"]
                                            ],
                                            "position_uncertainty_m": float(
                                                perception_cfg.get("association_radius_m", 5.0)
                                            ),
                                            "detection_confidence": float(selected["detector_score"]),
                                            "color_confidence": float(selected["color_score"]),
                                            "red_pixel_ratio": float(selected["red_pixel_ratio"]),
                                            "van_like_score": float(selected["van_like_score"]),
                                            "instruction_match_score": float(
                                                selected["instruction_match_score"]
                                            ),
                                            "track_id": selected["track_id"],
                                            "source_frame": int(common_frame),
                                            "source_timestamp": float(metadata["timestamp"]),
                                            "source_kind": "uav_rgb_depth_perception",
                                            "evidence": {
                                                "rgb_frame": int(common_frame),
                                                "depth_frame": int(common_frame),
                                                "proposal_source": selected["proposal_source"],
                                                "temporal_hits": int(selected["temporal_hits"]),
                                            },
                                        },
                                        tick_index + 1,
                                        common_frame,
                                        float(metadata["timestamp"]),
                                    )
                                    if oracle_message_sent is not None:
                                        task_machine.transition(
                                            "MESSAGE_SENT",
                                            common_frame,
                                            float(metadata["timestamp"]),
                                            "semantic_channel_send",
                                            message_id=oracle_message_sent["message_id"],
                                        )
                                        communication_events.append(
                                            {"event": "sent", **oracle_message_sent}
                                        )
                                        append_jsonl(
                                            run_dirs["perception"] / "runtime_events.jsonl",
                                            {"event": "message_sent", **oracle_message_sent},
                                        )
                            else:
                                verification = local_verifier.evaluate(
                                    common_frame,
                                    float(metadata["timestamp"]),
                                    candidates,
                                    received_oracle_message["target_world_position_xyz"],
                                )
                                verification["candidate_count"] = len(candidates)
                                local_confirmation_rows.append(verification)
                                append_jsonl(
                                    run_dirs["perception"] / "runtime_events.jsonl",
                                    {"event": "ugv_local_verification", **verification},
                                )
                                if verification["confirmed"]:
                                    task_machine.mark_target_verified(
                                        common_frame,
                                        float(metadata["timestamp"]),
                                        matched_candidate_id=verification["matched_candidate_id"],
                                        candidate_category=verification["candidate_category"],
                                        red_pixel_ratio=verification["red_pixel_ratio"],
                                        van_like_score=verification["van_like_score"],
                                        position_difference_m=verification["position_difference_m"],
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
            health_check_seconds = float(dynamic_cfg.get("sensor_health_check_seconds", 10.0))
            if closed_loop_mode and not sensor_health_checked and sim_elapsed >= health_check_seconds:
                expected_so_far = max(1, int(round(health_check_seconds / sensor_tick)))
                health_ratio = len(saved_frames) / expected_so_far
                if health_ratio < 0.80:
                    raise RuntimeError(
                        f"Early sensor health check failed: {len(saved_frames)}/{expected_so_far} common frames"
                    )
                sensor_health_checked = True

        actor_state_handle.close()
        for handle in metadata_handles.values():
            handle.close()

        for role, rows in trajectory_rows.items():
            write_csv(run_dirs["trajectories"] / f"{role}_trajectory.csv", rows)
        write_csv(run_dirs["synchronization"] / "frame_index.csv", frame_rows)
        write_csv(run_dirs["safety"] / "safety_state.csv", safety_rows)
        s0.json_dump(run_dirs["safety"] / "ugv_collision_events.json", ugv_collision_events)
        s0.json_dump(run_dirs["safety"] / "uav_collision_events.json", uav_collision_events)
        if closed_loop_mode and task_machine is not None and oracle_channel is not None:
            write_csv(run_dirs["task"] / "state_timeline.csv", task_timeline)
            write_jsonl(run_dirs["task"] / "task_events.jsonl", task_machine.events)
            write_jsonl(run_dirs["communication"] / "messages.jsonl", oracle_channel.messages)
            write_jsonl(run_dirs["communication"] / "communication_events.jsonl", communication_events)
            s0.json_dump(
                run_dirs["communication"] / "message_summary.json",
                {
                    "oracle": bool(oracle_mode),
                    "perception_enabled": bool(perception_mode and perception_cfg.get("enabled", True)),
                    "enabled": bool(closed_loop_cfg.get("enabled", True)),
                    "negative_control": negative_control,
                    "messages_sent": int(oracle_message_sent is not None),
                    "messages_received": len(oracle_channel.messages),
                    "delivery_delay_ticks": int(closed_loop_cfg.get("delivery_delay_ticks", 1)),
                },
            )
            usage_name = "oracle_usage_audit.json" if oracle_mode else "perception_usage_audit.json"
            s0.json_dump(run_dirs["task"] / usage_name, oracle_usage_audit)
            if perception_mode:
                write_jsonl(run_dirs["perception"] / "uav_candidates.jsonl", perception_candidate_rows)
                write_jsonl(run_dirs["perception"] / "ugv_confirmations.jsonl", local_confirmation_rows)
            if planning_summary:
                s0.json_dump(run_dirs["planning"] / "planning_summary.json", planning_summary)
                write_csv(
                    run_dirs["planning"] / "ugv_planned_route.csv",
                    [
                        {"route_index": index, "x": point[0], "y": point[1], "z": point[2]}
                        for index, point in enumerate(active_ugv_route)
                    ],
                )

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
        route_metrics = route_completion_metrics(
            paths_xyz.get("uav", []),
            uav_route,
            float(acceptance.get("uav_waypoint_radius_m", 1.0)),
        )

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
        final_ugv_target_distance = (
            horizontal_distance(paths_xyz["ugv"][-1], paths_xyz["target"][-1])
            if paths_xyz.get("ugv") and paths_xyz.get("target")
            else float("inf")
        )
        perception_evaluation: dict[str, Any] = {}
        if perception_mode:
            target_eval_xyz = paths_xyz.get("target", [[float("nan")] * 3])[-1]
            if transmitted_candidate is not None and transmitted_candidate.get("world_position_xyz") is not None:
                message_localization_error_m = horizontal_distance(
                    transmitted_candidate["world_position_xyz"], target_eval_xyz
                )
                wrong_target_messages = int(
                    message_localization_error_m
                    > float(acceptance.get("maximum_message_localization_error_m", 3.0))
                )
            perception_evaluation = {
                "oracle": False,
                "ground_truth_scope": "post-run evaluation only",
                "runtime_ground_truth_reads": 0,
                "message_localization_error_m": message_localization_error_m,
                "wrong_target_messages": wrong_target_messages,
                "transmitted_candidate_id": (
                    None if transmitted_candidate is None else transmitted_candidate.get("candidate_id")
                ),
                "target_world_position_xyz": [float(value) for value in target_eval_xyz],
                "local_confirmation_success": bool(
                    isinstance(task_machine, PerceptionTaskStateMachine) and task_machine.target_verified
                ),
            }
            s0.json_dump(
                run_dirs["evaluation"] / "perception_post_run_evaluation.json",
                perception_evaluation,
            )
        minimum_uav_clearance = min(uav_central_clearances) if uav_central_clearances else 0.0

        checks: list[dict[str, Any]] = []
        add_acceptance_check(checks, "map_is_frozen", map_name.lower().endswith("town10hd"), map_name, "Town10HD")
        add_acceptance_check(checks, "common_frame_ratio", common_ratio >= float(acceptance["minimum_common_frame_ratio"]), common_ratio, f">={acceptance['minimum_common_frame_ratio']}")
        add_acceptance_check(checks, "exact_four_stream_frame_alignment", max_frame_spread == 0, max_frame_spread, "0")
        add_acceptance_check(checks, "ugv_dynamic_path", ugv_path >= float(acceptance["minimum_ugv_path_m"]), ugv_path, f">={acceptance['minimum_ugv_path_m']} m")
        add_acceptance_check(checks, "uav_dynamic_path", uav_path >= float(acceptance["minimum_uav_path_m"]), uav_path, f">={acceptance['minimum_uav_path_m']} m")
        if "minimum_uav_waypoints_reached" in acceptance:
            add_acceptance_check(
                checks,
                "uav_waypoints_reached",
                route_metrics["waypoints_reached"] >= int(acceptance["minimum_uav_waypoints_reached"]),
                route_metrics["waypoints_reached"],
                f">={acceptance['minimum_uav_waypoints_reached']}",
            )
        if "minimum_uav_segments_completed" in acceptance:
            add_acceptance_check(
                checks,
                "uav_segments_completed",
                route_metrics["segments_completed"] >= int(acceptance["minimum_uav_segments_completed"]),
                route_metrics["segments_completed"],
                f">={acceptance['minimum_uav_segments_completed']}",
            )
        if "minimum_uav_foldbacks_completed" in acceptance:
            add_acceptance_check(
                checks,
                "uav_foldbacks_completed",
                route_metrics["foldbacks_completed"] >= int(acceptance["minimum_uav_foldbacks_completed"]),
                route_metrics["foldbacks_completed"],
                f">={acceptance['minimum_uav_foldbacks_completed']}",
            )
        add_acceptance_check(checks, "uav_control_tracking_p95", uav_follow_p95 <= float(acceptance["maximum_uav_tracking_p95_m"]), uav_follow_p95, f"<={acceptance['maximum_uav_tracking_p95_m']} m")
        add_acceptance_check(checks, "target_remains_stationary", target_path <= float(acceptance["maximum_target_drift_m"]), target_path, f"<={acceptance['maximum_target_drift_m']} m")
        if "maximum_ugv_collisions" in acceptance:
            add_acceptance_check(
                checks,
                "ugv_collision_free",
                len(ugv_collision_events) <= int(acceptance["maximum_ugv_collisions"]),
                len(ugv_collision_events),
                f"<={acceptance['maximum_ugv_collisions']}",
            )
        if "maximum_uav_collisions" in acceptance:
            add_acceptance_check(
                checks,
                "uav_collision_free",
                len(uav_collision_events) <= int(acceptance["maximum_uav_collisions"]),
                len(uav_collision_events),
                f"<={acceptance['maximum_uav_collisions']}",
            )
        if "minimum_final_ugv_target_distance_m" in acceptance:
            lower = float(acceptance["minimum_final_ugv_target_distance_m"])
            upper = float(acceptance["maximum_final_ugv_target_distance_m"])
            add_acceptance_check(
                checks,
                "ugv_safe_target_standoff",
                lower <= final_ugv_target_distance <= upper,
                final_ugv_target_distance,
                f"{lower}..{upper} m",
            )
        if "minimum_uav_central_clearance_m" in acceptance:
            add_acceptance_check(
                checks,
                "uav_minimum_central_clearance",
                minimum_uav_clearance >= float(acceptance["minimum_uav_central_clearance_m"]),
                minimum_uav_clearance,
                f">={acceptance['minimum_uav_central_clearance_m']} m",
            )
        add_acceptance_check(checks, "moving_distractor_vehicles", moving_vehicles >= int(acceptance["minimum_moving_vehicles"]), moving_vehicles, f">={acceptance['minimum_moving_vehicles']}")
        add_acceptance_check(checks, "moving_pedestrians", moving_pedestrians >= int(acceptance["minimum_moving_pedestrians"]), moving_pedestrians, f">={acceptance['minimum_moving_pedestrians']}")
        for device in ("uav", "ugv"):
            add_acceptance_check(checks, f"{device}_rgb_is_dynamic", unique_rgb[device] >= float(acceptance["minimum_unique_rgb_fraction"]), unique_rgb[device], f">={acceptance['minimum_unique_rgb_fraction']}")
            minimum_finite = min(depth_finite[device]) if depth_finite[device] else 0.0
            add_acceptance_check(checks, f"{device}_depth_is_finite", minimum_finite >= 0.999, minimum_finite, ">=0.999")

        final_ugv_speed = float(trajectory_rows.get("ugv", [{}])[-1].get("speed_mps", float("inf")))
        if oracle_mode and task_machine is not None and oracle_channel is not None:
            sent_count = int(oracle_message_sent is not None)
            received_count = len(oracle_channel.messages)
            add_acceptance_check(
                checks,
                "oracle_label_is_explicit",
                bool(resolved.get("oracle")) and resolved["experiment_type"] == "oracle_closed_loop_acceptance",
                True,
                "oracle=true and oracle_closed_loop_acceptance",
            )
            add_acceptance_check(
                checks,
                "instruction_and_parsed_goal_loaded",
                bool(resolved["task"].get("instruction")) and bool(resolved["task"].get("parsed_goal")),
                resolved["task"]["task_id"],
                "non-empty frozen instruction and parsed_goal",
            )
            add_acceptance_check(
                checks,
                "task_state_transitions_valid",
                task_machine.transitions_valid,
                task_machine.transitions_valid,
                "true",
            )
            add_acceptance_check(
                checks,
                "oracle_messages_sent",
                sent_count == int(acceptance["required_oracle_messages_sent"]),
                sent_count,
                str(acceptance["required_oracle_messages_sent"]),
            )
            add_acceptance_check(
                checks,
                "oracle_messages_received",
                received_count == int(acceptance["required_oracle_messages_received"]),
                received_count,
                str(acceptance["required_oracle_messages_received"]),
            )
            add_acceptance_check(
                checks,
                "ugv_waits_before_message",
                maximum_pre_message_ugv_displacement
                <= float(
                    acceptance.get(
                        "maximum_pre_message_ugv_displacement_m",
                        acceptance.get("maximum_negative_control_ugv_displacement_m", 0.5),
                    )
                ),
                maximum_pre_message_ugv_displacement,
                "<=0.5 m",
            )
            add_acceptance_check(
                checks,
                "ugv_controller_has_no_direct_target_truth_reads",
                oracle_usage_audit["ugv_controller_direct_target_actor_reads"] == 0,
                oracle_usage_audit["ugv_controller_direct_target_actor_reads"],
                "0",
            )
            if negative_control:
                add_acceptance_check(
                    checks,
                    "negative_control_no_route_planned",
                    not planning_summary and not active_ugv_route,
                    len(active_ugv_route),
                    "0 route points",
                )
                add_acceptance_check(
                    checks,
                    "negative_control_ugv_stationary",
                    ugv_path <= float(acceptance["maximum_negative_control_ugv_displacement_m"]),
                    ugv_path,
                    f"<={acceptance['maximum_negative_control_ugv_displacement_m']} m",
                )
            else:
                add_acceptance_check(
                    checks,
                    "route_planned_after_message_receive",
                    planned_frame is not None
                    and received_frame is not None
                    and planned_frame >= received_frame
                    and planning_summary.get("route_source") == "received_oracle_message",
                    {"received_frame": received_frame, "planned_frame": planned_frame},
                    "planned_frame >= received_frame and received_oracle_message source",
                )
                add_acceptance_check(
                    checks,
                    "ugv_arrival_hold_completed",
                    task_machine.ugv_arrived
                    and arrival_hold_ticks * fixed_delta
                    >= float(acceptance["minimum_arrival_hold_seconds"]),
                    arrival_hold_ticks * fixed_delta,
                    f">={acceptance['minimum_arrival_hold_seconds']} s",
                )
                add_acceptance_check(
                    checks,
                    "ugv_final_speed",
                    final_ugv_speed <= float(acceptance["maximum_final_ugv_speed_mps"]),
                    final_ugv_speed,
                    f"<={acceptance['maximum_final_ugv_speed_mps']} m/s",
                )
                add_acceptance_check(
                    checks,
                    "oracle_task_completed",
                    task_machine.state == "COMPLETED",
                    task_machine.state,
                    "COMPLETED",
                )
        elif perception_mode and isinstance(task_machine, PerceptionTaskStateMachine) and oracle_channel is not None:
            sent_count = int(oracle_message_sent is not None)
            received_count = len(oracle_channel.messages)
            add_acceptance_check(
                checks,
                "perception_label_is_explicit",
                not bool(resolved.get("oracle"))
                and resolved["experiment_type"] == "perception_closed_loop_acceptance",
                {"oracle": False, "perception_enabled": bool(perception_cfg.get("enabled", True))},
                "oracle=false and perception_closed_loop_acceptance",
            )
            add_acceptance_check(
                checks,
                "instruction_and_parsed_goal_loaded",
                bool(resolved["task"].get("instruction")) and bool(resolved["task"].get("parsed_goal")),
                resolved["task"]["task_id"],
                "non-empty frozen instruction and parsed_goal",
            )
            add_acceptance_check(
                checks,
                "task_state_transitions_valid",
                task_machine.transitions_valid,
                task_machine.transitions_valid,
                "true",
            )
            add_acceptance_check(
                checks,
                "semantic_messages_sent",
                sent_count == int(acceptance["required_semantic_messages_sent"]),
                sent_count,
                str(acceptance["required_semantic_messages_sent"]),
            )
            add_acceptance_check(
                checks,
                "semantic_messages_received",
                received_count == int(acceptance["required_semantic_messages_received"]),
                received_count,
                str(acceptance["required_semantic_messages_received"]),
            )
            add_acceptance_check(
                checks,
                "ugv_waits_before_message",
                maximum_pre_message_ugv_displacement
                <= float(
                    acceptance.get(
                        "maximum_pre_message_ugv_displacement_m",
                        acceptance.get("maximum_negative_control_ugv_displacement_m", 0.5),
                    )
                ),
                maximum_pre_message_ugv_displacement,
                "<=0.5 m",
            )
            add_acceptance_check(
                checks,
                "perception_and_ugv_control_have_no_target_truth_reads",
                oracle_usage_audit["perception_runtime_target_truth_reads"] == 0
                and oracle_usage_audit["ugv_controller_direct_target_actor_reads"] == 0,
                {
                    "perception": oracle_usage_audit["perception_runtime_target_truth_reads"],
                    "ugv_controller": oracle_usage_audit["ugv_controller_direct_target_actor_reads"],
                },
                "both 0",
            )
            if negative_control:
                add_acceptance_check(
                    checks,
                    "negative_control_no_route_planned",
                    not planning_summary and not active_ugv_route,
                    len(active_ugv_route),
                    "0 route points",
                )
                add_acceptance_check(
                    checks,
                    "negative_control_ugv_stationary",
                    ugv_path <= float(acceptance["maximum_negative_control_ugv_displacement_m"]),
                    ugv_path,
                    f"<={acceptance['maximum_negative_control_ugv_displacement_m']} m",
                )
            else:
                add_acceptance_check(
                    checks,
                    "semantic_payload_is_not_oracle",
                    bool(oracle_message_sent) and oracle_message_sent.get("oracle") is False,
                    None if oracle_message_sent is None else oracle_message_sent.get("oracle"),
                    "false",
                )
                add_acceptance_check(
                    checks,
                    "route_planned_after_perception_message",
                    planned_frame is not None
                    and received_frame is not None
                    and planned_frame >= received_frame
                    and planning_summary.get("route_source") == "received_perception_message",
                    {"received_frame": received_frame, "planned_frame": planned_frame},
                    "planned_frame >= received_frame and received_perception_message source",
                )
                delivered_latency_ms = (
                    None if not oracle_channel.messages else oracle_channel.messages[0].get("latency_ms")
                )
                expected_latency_ms = float(acceptance.get("expected_message_latency_ms", 50.0))
                maximum_latency_error_ms = float(
                    acceptance.get("maximum_message_latency_error_ms", 1.0)
                )
                add_acceptance_check(
                    checks,
                    "semantic_message_latency",
                    delivered_latency_ms is not None
                    and abs(float(delivered_latency_ms) - expected_latency_ms)
                    <= maximum_latency_error_ms,
                    delivered_latency_ms,
                    f"{expected_latency_ms} +/- {maximum_latency_error_ms} ms",
                )
                add_acceptance_check(
                    checks,
                    "transmitted_candidate_localization",
                    message_localization_error_m is not None
                    and message_localization_error_m
                    <= float(acceptance["maximum_message_localization_error_m"]),
                    message_localization_error_m,
                    f"<={acceptance['maximum_message_localization_error_m']} m",
                )
                add_acceptance_check(
                    checks,
                    "wrong_target_messages",
                    wrong_target_messages <= int(acceptance["maximum_wrong_target_messages"]),
                    wrong_target_messages,
                    f"<={acceptance['maximum_wrong_target_messages']}",
                )
                add_acceptance_check(
                    checks,
                    "ugv_local_rgb_depth_confirmation",
                    task_machine.target_verified,
                    task_machine.target_verified,
                    "true",
                )
                add_acceptance_check(
                    checks,
                    "ugv_arrival_hold_completed",
                    task_machine.ugv_arrived
                    and arrival_hold_ticks * fixed_delta
                    >= float(acceptance["minimum_arrival_hold_seconds"]),
                    arrival_hold_ticks * fixed_delta,
                    f">={acceptance['minimum_arrival_hold_seconds']} s",
                )
                add_acceptance_check(
                    checks,
                    "ugv_final_speed",
                    final_ugv_speed <= float(acceptance["maximum_final_ugv_speed_mps"]),
                    final_ugv_speed,
                    f"<={acceptance['maximum_final_ugv_speed_mps']} m/s",
                )
                add_acceptance_check(
                    checks,
                    "perception_task_completed",
                    (
                        task_machine.state == "COMPLETED"
                        if bool(acceptance.get("require_full_task_completion", True))
                        else task_machine.state in {"ARRIVED_SAFE", "UAV_ROUTE_COMPLETE", "COMPLETED"}
                    ),
                    task_machine.state,
                    (
                        "COMPLETED"
                        if bool(acceptance.get("require_full_task_completion", True))
                        else "ARRIVED_SAFE or later"
                    ),
                )

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
            "uav_route_completion": route_metrics,
            "ugv_start_delay_seconds": float(dynamic_cfg.get("ugv_start_delay_seconds", 0.0)),
            "ugv_final_target_distance_m": final_ugv_target_distance,
            "ugv_collision_count": len(ugv_collision_events),
            "uav_collision_count": len(uav_collision_events),
            "uav_minimum_central_clearance_m": minimum_uav_clearance,
            "uav_camera": {
                "width": int(sensor_config["width"]),
                "height": int(sensor_config["height"]),
                "horizontal_fov_degrees": float(sensor_config["fov_degrees"]),
                "yaw_mode": str(dynamic_cfg.get("uav_camera_yaw_mode", "path_aligned")),
            },
            "moving_distractor_vehicles": moving_vehicles,
            "moving_pedestrians": moving_pedestrians,
            "vehicle_displacements_m": vehicle_displacements,
            "pedestrian_displacements_m": pedestrian_displacements,
            "unique_rgb_ratio": unique_rgb,
            "minimum_depth_finite_fraction": {
                device: min(values) if values else 0.0 for device, values in depth_finite.items()
            },
        }
        if oracle_mode and task_machine is not None and oracle_channel is not None:
            summary["oracle_closed_loop"] = {
                "oracle": True,
                "perception_enabled": False,
                "negative_control": negative_control,
                "outcome": "PASS_EXPECTED_NO_MESSAGE" if negative_control else task_machine.state,
                "task_final_state": task_machine.state,
                "state_transitions_valid": task_machine.transitions_valid,
                "messages_sent": int(oracle_message_sent is not None),
                "messages_received": len(oracle_channel.messages),
                "maximum_pre_message_ugv_displacement_m": maximum_pre_message_ugv_displacement,
                "planning": planning_summary,
                "arrival_hold_seconds": arrival_hold_ticks * fixed_delta,
                "final_ugv_speed_mps": final_ugv_speed,
                "usage_audit": oracle_usage_audit,
            }
        elif perception_mode and isinstance(task_machine, PerceptionTaskStateMachine) and oracle_channel is not None:
            summary["perception_closed_loop"] = {
                "oracle": False,
                "perception_enabled": bool(perception_cfg.get("enabled", True)),
                "negative_control": negative_control,
                "outcome": "PASS_EXPECTED_NO_MESSAGE" if negative_control else task_machine.state,
                "task_final_state": task_machine.state,
                "state_transitions_valid": task_machine.transitions_valid,
                "messages_sent": int(oracle_message_sent is not None),
                "messages_received": len(oracle_channel.messages),
                "message_payload_bytes": (
                    None if oracle_message_sent is None else oracle_message_sent.get("payload_bytes")
                ),
                "message_latency_ms": (
                    None if not oracle_channel.messages else oracle_channel.messages[0].get("latency_ms")
                ),
                "message_localization_error_m": message_localization_error_m,
                "wrong_target_messages": wrong_target_messages,
                "uav_inference_frames": len(perception_candidate_rows),
                "ugv_confirmation_attempts": len(local_confirmation_rows),
                "local_confirmation_success": task_machine.target_verified,
                "maximum_pre_message_ugv_displacement_m": maximum_pre_message_ugv_displacement,
                "planning": planning_summary,
                "arrival_hold_seconds": arrival_hold_ticks * fixed_delta,
                "final_ugv_speed_mps": final_ugv_speed,
                "usage_audit": oracle_usage_audit,
                "post_run_evaluation": perception_evaluation,
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
                "oracle": bool(oracle_mode),
                "perception_enabled": bool(perception_mode and perception_cfg.get("enabled", True)),
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
            experiment_label = (
                "S0 ORACLE (ground truth; not perception)"
                if oracle_mode
                else ("S1 PERCEPTION CLOSED LOOP" if perception_mode else "CI-E1 Dynamic Scene")
            )
            draw_dynamic_map(
                world_map,
                region,
                paths_xyz,
                actor_tracks,
                run_dirs["preview"] / "trajectory_map.png",
                experiment_label,
            )
            draw_alignment(
                frame_rows,
                expected_frames,
                run_dirs["preview"] / "synchronization_plot.png",
                "S0 ORACLE" if oracle_mode else ("S1 PERCEPTION" if perception_mode else "CI-E1"),
            )
            draw_sensor_composite(run_dirs, int(frame_rows[0]["frame"]), int(frame_rows[-1]["frame"]), summary)

        title = (
            "S0 ORACLE Closed-Loop Acceptance Output"
            if oracle_mode
            else (
                "S1 Perception Closed-Loop Acceptance Output"
                if perception_mode
                else "CI-E1 Dynamic Sensor Acceptance Output"
            )
        )
        oracle_notice = (
            "\n> **ORACLE — CARLA ground-truth target position. This is not a perception result.**\n"
            if oracle_mode
            else ""
        )
        readme = f"""# {title}
{oracle_notice}

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
        if oracle_mode:
            readme += """
- `preview/synchronized_multiview_replay.mp4`: Oracle-watermarked synchronized replay
- `preview/oracle_closed_loop_summary.png`: state, UGV approach and UAV-route summary
- `task/state_timeline.csv`: per-tick task state
- `task/task_events.jsonl`: state-transition audit trail
- `task/oracle_usage_audit.json`: ground-truth access boundary audit
- `communication/messages.jsonl`: delivered Oracle message
- `communication/communication_events.jsonl`: send/receive event ordering
"""
            if planning_summary:
                readme += """
- `planning/ugv_planned_route.csv`: route generated after message delivery
- `planning/planning_summary.json`: message-to-route provenance
"""
        elif perception_mode:
            readme += """
- `preview/synchronized_multiview_replay.mp4`: S1 synchronized five-view replay
- `preview/s1_perception_closed_loop_summary.png`: perception/message/confirmation/task summary
- `task/state_timeline.csv`: per-tick S1 task state
- `task/task_events.jsonl`: state-transition audit trail
- `task/perception_usage_audit.json`: runtime ground-truth isolation audit
- `perception/uav_candidates.jsonl`: observation-only UAV candidates and selected candidate
- `perception/ugv_confirmations.jsonl`: UGV close-range RGB/depth verification attempts
- `communication/messages.jsonl`: delivered target-level semantic message
- `ground_truth/evaluation_only/perception_post_run_evaluation.json`: post-run-only target evaluation
"""
            if planning_summary:
                readme += """
- `planning/ugv_planned_route.csv`: route generated after perception-message delivery
- `planning/planning_summary.json`: message-to-route provenance
"""
        (run_dirs["run"] / "README.md").write_text(readme, encoding="utf-8")
        stage_label = "S0 Oracle" if oracle_mode else ("S1 Perception" if perception_mode else "CI-E1")
        log(f"{stage_label} result: {report['status']}; output: {run_dirs['run']}")
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
        for sensor in safety_sensors:
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
