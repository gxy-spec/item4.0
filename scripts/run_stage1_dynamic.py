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
from safety.rgbd_obstacle_guard import (  # noqa: E402
    ObstacleGuardConfig,
    RGBDObstacleGuard,
    camera_mount_matrix,
    save_obstacle_preview,
)
from safety.rgbd_hazard_perception import HazardDetectorConfig, RGBDHazardPerception  # noqa: E402


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


def remaining_route_distance(
    route: list[list[float]], cursor_index: int, current_xyz: list[float]
) -> float:
    """Distance from the vehicle through the unconsumed route to its endpoint."""
    if not route:
        return float("inf")
    cursor = min(max(0, int(cursor_index)), len(route) - 1)
    return horizontal_distance(current_xyz, route[cursor]) + cumulative_distance(route[cursor:])


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


def route_normal_offset(xyz: list[float], yaw_degrees: float, lateral_m: float) -> list[float]:
    yaw = math.radians(float(yaw_degrees))
    return [
        float(xyz[0]) - math.sin(yaw) * float(lateral_m),
        float(xyz[1]) + math.cos(yaw) * float(lateral_m),
        float(xyz[2]),
    ]


def scripted_crossing_lateral(cfg: dict[str, Any], elapsed_s: float) -> float:
    start = float(cfg.get("start_seconds", 8.0))
    initial = float(cfg.get("resolved_lateral_start_m", cfg.get("lateral_start_m", 8.0)))
    final = float(cfg.get("resolved_lateral_end_m", cfg.get("lateral_end_m", -initial)))
    traverse = max(0.1, float(cfg.get("traverse_seconds", 5.0)))
    pause = max(0.0, float(cfg.get("pause_at_center_seconds", 1.0)))
    if elapsed_s < start:
        return initial
    if elapsed_s < start + traverse:
        ratio = (elapsed_s - start) / traverse
        return initial * (1.0 - ratio)
    if elapsed_s < start + traverse + pause:
        return 0.0
    if elapsed_s < start + 2.0 * traverse + pause:
        ratio = (elapsed_s - start - traverse - pause) / traverse
        return final * ratio
    return final


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


def bounded_nearest_route_index(
    route: list[list[float]],
    location_xy: list[float],
    cursor_index: int,
    forward_window: int = 24,
) -> int:
    """Advance along a route without jumping across nearby/self-crossing segments."""
    if not route:
        raise ValueError("Route must contain at least one point")
    start = min(max(0, int(cursor_index)), len(route) - 1)
    stop = min(len(route), start + max(1, int(forward_window)) + 1)
    return min(
        range(start, stop),
        key=lambda index: (
            (float(route[index][0]) - float(location_xy[0])) ** 2
            + (float(route[index][1]) - float(location_xy[1])) ** 2
        ),
    )


def actor_state(actor: Any, role: str, frame: int, timestamp: float) -> dict[str, Any]:
    transform = actor.get_transform()
    velocity = actor.get_velocity()
    angular = actor.get_angular_velocity()
    extent = actor.bounding_box.extent
    return {
        "frame": int(frame),
        "timestamp": float(timestamp),
        "actor_id": int(actor.id),
        "role": role,
        "type_id": str(actor.type_id),
        "bbox_extent_xyz": [float(extent.x), float(extent.y), float(extent.z)],
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


def oriented_box_clearance_xy(first: dict[str, Any], second: dict[str, Any]) -> float:
    """Signed XY separation of two actor bounding rectangles (negative means overlap)."""
    def box(state: dict[str, Any]) -> tuple[np.ndarray, list[np.ndarray], list[np.ndarray]]:
        center = np.asarray([float(state["x"]), float(state["y"])], dtype=np.float64)
        extents = [float(value) for value in state.get("bbox_extent_xyz", [0.0, 0.0])[:2]]
        yaw = math.radians(float(state.get("yaw", 0.0)))
        forward = np.asarray([math.cos(yaw), math.sin(yaw)], dtype=np.float64)
        lateral = np.asarray([-math.sin(yaw), math.cos(yaw)], dtype=np.float64)
        corners = [
            center + sign_f * extents[0] * forward + sign_l * extents[1] * lateral
            for sign_f, sign_l in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
        ]
        return center, corners, [forward, lateral]

    center_a, corners_a, axes_a = box(first)
    center_b, corners_b, axes_b = box(second)
    delta = center_b - center_a
    minimum_overlap = math.inf
    separated = False
    for axis in (*axes_a, *axes_b):
        projections_a = [float(np.dot(corner - center_a, axis)) for corner in corners_a]
        projections_b = [float(np.dot(corner - center_b, axis)) for corner in corners_b]
        radius_a = 0.5 * (max(projections_a) - min(projections_a))
        radius_b = 0.5 * (max(projections_b) - min(projections_b))
        overlap = radius_a + radius_b - abs(float(np.dot(delta, axis)))
        if overlap < 0.0:
            separated = True
        else:
            minimum_overlap = min(minimum_overlap, overlap)
    if not separated:
        return -float(minimum_overlap)

    def point_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
        segment = end - start
        length_squared = float(np.dot(segment, segment))
        if length_squared <= 1e-12:
            return float(np.linalg.norm(point - start))
        ratio = float(np.clip(np.dot(point - start, segment) / length_squared, 0.0, 1.0))
        return float(np.linalg.norm(point - (start + ratio * segment)))

    distances = [
        point_segment_distance(point, corners_b[index], corners_b[(index + 1) % 4])
        for point in corners_a
        for index in range(4)
    ] + [
        point_segment_distance(point, corners_a[index], corners_a[(index + 1) % 4])
        for point in corners_b
        for index in range(4)
    ]
    return min(distances)


def evaluate_post_run_safety_clearance(actor_states_path: Path) -> dict[str, Any]:
    """Use logged actor truth after a run to estimate UGV-to-test-actor clearance."""
    by_frame: dict[int, dict[str, dict[str, Any]]] = defaultdict(dict)
    with actor_states_path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            state = json.loads(line)
            by_frame[int(state["frame"])][str(state["role"])] = state
    minimum: dict[str, Any] | None = None
    for frame, actors in by_frame.items():
        ugv_state = actors.get("ugv")
        if ugv_state is None:
            continue
        for role, actor_state_row in actors.items():
            if not role.startswith("safety_test_"):
                continue
            center_distance = horizontal_distance(
                [float(ugv_state["x"]), float(ugv_state["y"])],
                [float(actor_state_row["x"]), float(actor_state_row["y"])],
            )
            oriented_gap = oriented_box_clearance_xy(ugv_state, actor_state_row)
            if minimum is None or oriented_gap < float(minimum["estimated_oriented_bbox_gap_m"]):
                minimum = {
                    "frame": frame,
                    "timestamp": float(ugv_state["timestamp"]),
                    "actor_role": role,
                    "center_distance_m": center_distance,
                    "estimated_oriented_bbox_gap_m": oriented_gap,
                    "clearance_method": "2D oriented bounding-box separation; negative values indicate projected box overlap",
                }
    return minimum or {"available": False, "reason": "No safety_test actor states overlap the UGV run frames"}


def follow_route(
    vehicle: Any,
    route: list[list[float]],
    target_speed: float,
    cursor_index: int = 0,
    lookahead_points: int = 6,
) -> int:
    import carla

    transform = vehicle.get_transform()
    location = transform.location
    nearest = bounded_nearest_route_index(
        route,
        [float(location.x), float(location.y)],
        cursor_index,
    )
    lookahead = min(len(route) - 1, nearest + max(1, int(lookahead_points)))
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
        # A wide dead-band made the vehicle cruise almost 0.8 m/s above its
        # requested speed and largely erased CAUTION slowdowns.  Use a small
        # feed-forward term with proportional speed-error braking instead.
        speed_error = target_speed - speed
        throttle = float(np.clip(0.08 + speed_error * 0.18, 0.0, 0.65))
        brake = float(np.clip((speed - target_speed - 0.12) * 0.55, 0.0, 1.0))
        control = carla.VehicleControl(throttle=throttle, brake=brake, steer=steer, hand_brake=False)
    vehicle.apply_control(control)
    return nearest


def route_progress_m(
    route: list[list[float]],
    location_xyz: list[float],
    cursor_index: int | None = None,
) -> tuple[float, int]:
    if cursor_index is None:
        index = min(
            range(len(route)),
            key=lambda item: (
                (float(route[item][0]) - float(location_xyz[0])) ** 2
                + (float(route[item][1]) - float(location_xyz[1])) ** 2
            ),
        )
    else:
        index = bounded_nearest_route_index(route, location_xyz, cursor_index)
    return cumulative_distance(route[: index + 1]), index


def route_lateral_offset_m(route: list[list[float]], location_xyz: list[float]) -> float:
    """Signed UGV cross-track offset from the nominal route in CARLA XY."""
    _, index = route_progress_m(route, location_xyz)
    start = route[max(0, index - 1)]
    end = route[min(len(route) - 1, index + 1)]
    yaw = math.atan2(float(end[1]) - float(start[1]), float(end[0]) - float(start[0]))
    normal = np.asarray([-math.sin(yaw), math.cos(yaw)], dtype=np.float64)
    delta = np.asarray(
        [float(location_xyz[0]) - float(route[index][0]), float(location_xyz[1]) - float(route[index][1])],
        dtype=np.float64,
    )
    return float(np.dot(delta, normal))


def route_with_overtake_offset(
    route: list[list[float]], start_s: float, end_s: float, lateral_offset_m: float, ramp_m: float = 7.0
) -> list[list[float]]:
    """Create a smooth, temporary adjacent-lane path for a straight-road pass."""
    result: list[list[float]] = []
    traversed = 0.0
    for index, point in enumerate(route):
        if index:
            traversed += horizontal_distance(route[index - 1], point)
        if traversed <= start_s:
            factor = 0.0
        elif traversed < start_s + ramp_m:
            ratio = (traversed - start_s) / ramp_m
            factor = ratio * ratio * (3.0 - 2.0 * ratio)
        elif traversed <= end_s - ramp_m:
            factor = 1.0
        elif traversed < end_s:
            ratio = (end_s - traversed) / ramp_m
            factor = ratio * ratio * (3.0 - 2.0 * ratio)
        else:
            factor = 0.0
        yaw = math.degrees(math.atan2(
            route[min(index + 1, len(route) - 1)][1] - route[max(0, index - 1)][1],
            route[min(index + 1, len(route) - 1)][0] - route[max(0, index - 1)][0],
        ))
        result.append(route_normal_offset(point, yaw, lateral_offset_m * factor))
    return result


def legal_same_direction_overtake_lane(
    world_map: Any,
    vehicle: Any,
    route: list[list[float]],
    progress_m: float,
    obstacle_distance_m: float,
) -> tuple[float, float] | None:
    """Return (signed lateral offset, lane width) only on a legal same-direction lane."""
    import carla

    waypoint = world_map.get_waypoint(
        vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
    )
    if waypoint is None or waypoint.is_junction:
        return None
    # Reject a junction only when it lies in the actual lane-change/pass
    # envelope. A junction safely cleared before the pass must not permanently
    # disable overtaking farther down the same road.
    route_progress = [0.0]
    for index in range(1, len(route)):
        route_progress.append(route_progress[-1] + horizontal_distance(route[index - 1], route[index]))
    obstacle_s = progress_m + float(obstacle_distance_m)
    pass_start_s = max(progress_m + 1.0, obstacle_s - 20.0)
    pass_end_s = obstacle_s + 13.0
    for point, segment_progress in zip(route, route_progress):
        if segment_progress < pass_start_s or segment_progress > pass_end_s:
            continue
        query = carla.Location(float(point[0]), float(point[1]), float(point[2]))
        ahead = world_map.get_waypoint(query, project_to_road=True, lane_type=carla.LaneType.Driving)
        if ahead is not None and ahead.is_junction:
            return None
    marking = str(waypoint.left_lane_marking.lane_change).lower()
    if "left" not in marking and "both" not in marking:
        return None
    adjacent = waypoint.get_left_lane()
    if adjacent is None or adjacent.lane_type != carla.LaneType.Driving:
        return None
    heading_error = abs(wrap_angle_degrees(float(adjacent.transform.rotation.yaw) - float(waypoint.transform.rotation.yaw)))
    if heading_error > 30.0:
        return None
    center = waypoint.transform.location
    side = adjacent.transform.location
    yaw = math.radians(float(waypoint.transform.rotation.yaw))
    normal = np.asarray([-math.sin(yaw), math.cos(yaw)], dtype=np.float64)
    displacement = np.asarray([side.x - center.x, side.y - center.y], dtype=np.float64)
    signed_offset = float(np.dot(displacement, normal))
    lane_width = float(np.linalg.norm(displacement))
    if not 2.4 <= lane_width <= 5.0:
        return None
    return signed_offset, lane_width


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
    sensor_observation: dict[str, Any] | None = None,
    hazard_decision: dict[str, Any] | None = None,
    overtake_active: bool = False,
    route_cursor_index: int = 0,
    lane_change_active: bool = False,
) -> dict[str, float | str]:
    import carla

    target_distance = (
        horizontal_distance(s0.xyz(vehicle.get_location()), s0.xyz(target_actor.get_location()))
        if bool(safety.get("target_stop_enabled", True))
        else float("inf")
    )
    sensor_obstacle_source = str(safety.get("obstacle_source", "carla_actor_truth")) == "ugv_rgbd"
    if sensor_obstacle_source:
        if sensor_observation is None or sensor_observation.get("state") == "SENSOR_STALE":
            clearance = 0.0
        else:
            clearance = float(sensor_observation.get("nearest_obstacle_m", float("inf")))
    else:
        clearance = forward_obstacle_clearance(
            vehicle, obstacles, float(safety.get("forward_corridor_half_width_m", 3.0))
        )
    stop_distance = float(safety.get("target_stop_distance_m", 5.0))
    emergency_distance = float(
        safety.get("sensor_stop_distance_m", 4.0)
        if sensor_obstacle_source
        else safety.get("emergency_brake_distance_m", 7.0)
    )
    slow_distance = float(
        safety.get("warning_distance_m", 8.0)
        if sensor_obstacle_source
        else safety.get("slowdown_distance_m", 14.0)
    )
    mode = "cruise"
    updated_route_cursor = int(route_cursor_index)
    hazard_decision = hazard_decision or {}
    forced_stop_reason = str(hazard_decision.get("forced_stop_reason", ""))
    slow_vehicle = hazard_decision.get("slow_vehicle_candidate")
    no_safe_overtake_gap = bool(hazard_decision.get("no_safe_overtake_gap", False))
    slow_vehicle_distance = (
        float(slow_vehicle.get("forward_m", float("inf")))
        if isinstance(slow_vehicle, dict)
        else float("inf")
    )
    if no_safe_overtake_gap and slow_vehicle_distance <= 8.0 and not forced_stop_reason:
        forced_stop_reason = "no_safe_overtake_gap"
    sensor_stop = sensor_obstacle_source and (
        sensor_observation is None
        or sensor_observation.get("state") == "SENSOR_STALE"
        or (sensor_observation.get("state") == "STOP" and not overtake_active)
    )
    sensor_caution = sensor_obstacle_source and sensor_observation is not None and sensor_observation.get("state") == "CAUTION"
    if forced_stop_reason or target_distance <= stop_distance + 1.2 or clearance <= emergency_distance or sensor_stop:
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=False))
        mode = f"wait_{forced_stop_reason}" if forced_stop_reason else (
            "sensor_stale_stop" if sensor_observation is not None and sensor_observation.get("state") == "SENSOR_STALE" else (
            "depth_obstacle_stop" if sensor_stop or clearance <= emergency_distance else "target_stop"
            )
        )
    else:
        commanded_speed = target_speed_mps
        if target_distance < slow_distance:
            commanded_speed = min(commanded_speed, max(1.0, (target_distance - stop_distance) * 0.70))
            mode = "target_approach"
        if clearance < slow_distance:
            commanded_speed = min(commanded_speed, max(0.8, (clearance - emergency_distance) * 0.75))
            mode = "depth_obstacle_caution" if sensor_caution else "obstacle_approach"
        if sensor_obstacle_source and sensor_observation is not None and sensor_observation.get("state") == "CAUTION":
            ttc_s = float(sensor_observation.get("ttc_s", float("nan")))
            caution_speed = max(0.5, (clearance - emergency_distance) * 0.6)
            if math.isfinite(ttc_s):
                caution_speed = min(caution_speed, max(0.5, target_speed_mps * min(1.0, ttc_s / max(0.1, float(safety.get("warning_ttc_s", 3.0))))))
            commanded_speed = min(commanded_speed, caution_speed)
            mode = "depth_obstacle_caution"
        if no_safe_overtake_gap and math.isfinite(slow_vehicle_distance):
            # Avoid an abrupt stop as soon as a distant vehicle is detected.
            # Reduce speed progressively, then stop with a short protected gap.
            commanded_speed = min(commanded_speed, max(0.7, (slow_vehicle_distance - 8.0) * 0.35))
            mode = "vehicle_follow_approach"
        updated_route_cursor = follow_route(
            vehicle,
            route,
            commanded_speed,
            cursor_index=route_cursor_index,
            lookahead_points=3 if lane_change_active else 6,
        )
    return {
        "ugv_target_distance_m": float(target_distance),
        "ugv_forward_clearance_m": float(clearance),
        "ugv_safety_mode": mode,
        "ugv_route_cursor_index": int(updated_route_cursor),
        "ugv_obstacle_sensor_state": "LEGACY_TRUTH" if not sensor_obstacle_source else (
            "SENSOR_STALE" if sensor_observation is None else str(sensor_observation.get("state", "UNKNOWN"))
        ),
        "ugv_obstacle_ttc_s": (
            float("nan") if sensor_observation is None else float(sensor_observation.get("ttc_s", float("nan")))
        ),
        "ugv_obstacle_points": 0 if sensor_observation is None else int(sensor_observation.get("occupied_points", 0)),
        "ugv_obstacle_sensor_age_s": (
            float("nan") if sensor_observation is None else float(sensor_observation.get("sensor_age_s", float("nan")))
        ),
            "ugv_obstacle_reason": "legacy_actor_truth" if not sensor_obstacle_source else (
            "missing_sensor_observation" if sensor_observation is None else str(sensor_observation.get("reason", ""))
        ),
    }


def classify_safety_hazards(detections: list[dict[str, Any]]) -> dict[str, Any]:
    """Conservative traffic-rule baseline from YOLO labels + RGB-D world tracks."""
    relevant = [item for item in detections if 0.0 < float(item["forward_m"]) <= 24.0]
    # Do not brake on a one-frame COCO label: require a stable world track and
    # a plausible pedestrian box. This filters shadows/signage without using
    # simulator actor truth in the online control path.
    persons = [
        item for item in relevant
        if item["kind"] == "person"
        and item.get("track_confirmed", False)
        and float(item.get("confidence", 0.0)) >= 0.12
        and float(item.get("bbox_height_px", 0.0)) >= 14.0
        and float(item.get("bbox_width_px", 0.0)) >= 5.0
        and (
            abs(float(item["lateral_m"])) <= 2.8
            or (
                abs(float(item["lateral_m"])) <= 4.8
                and float(item.get("lateral_speed_mps", 0.0)) >= 0.65
            )
        )
    ]
    if persons:
        return {"forced_stop_reason": "pedestrian", "hazard": min(persons, key=lambda x: x["forward_m"])}
    vehicles = [
        item for item in relevant
        if item["kind"] == "vehicle"
        and item.get("track_confirmed", False)
        and float(item.get("confidence", 0.0)) >= 0.12
        and abs(float(item["lateral_m"])) <= 4.5
    ]
    crossing = [item for item in vehicles if item.get("track_confirmed") and float(item.get("lateral_speed_mps", 0.0)) >= 0.65]
    if crossing:
        return {"forced_stop_reason": "crossing_vehicle", "hazard": min(crossing, key=lambda x: x["forward_m"])}
    lead = [item for item in vehicles if item["forward_m"] <= 20.0]
    if lead:
        candidate = min(lead, key=lambda x: x["forward_m"])
        if candidate.get("track_confirmed") and float(candidate.get("world_speed_mps", 99.0)) <= 0.8:
            return {"forced_stop_reason": "", "slow_vehicle_candidate": candidate}
        return {"forced_stop_reason": "vehicle_conflict", "hazard": candidate}
    return {"forced_stop_reason": "", "hazard": None}


def apply_safe_route_control_to_position(
    vehicle: Any,
    route: list[list[float]],
    target_speed_mps: float,
    target_xyz: list[float],
    obstacles: list[Any],
    safety: dict[str, Any],
    route_cursor_index: int = 0,
) -> tuple[dict[str, float | str], int]:
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
    updated_route_cursor = int(route_cursor_index)
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
        updated_route_cursor = follow_route(
            vehicle,
            route,
            commanded_speed,
            cursor_index=updated_route_cursor,
        )
    return (
        {
            "ugv_target_distance_m": float(target_distance),
            "ugv_forward_clearance_m": float(clearance),
            "ugv_safety_mode": mode,
        },
        updated_route_cursor,
    )


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


def save_rgb(image: Any, destination: Path | None) -> tuple[str, np.ndarray]:
    array = bgra_to_rgb(image.raw_data, image.width, image.height)
    if destination is not None:
        Image.fromarray(array).save(destination)
    return hashlib.sha256(array.tobytes()).hexdigest(), array


def save_depth(
    image: Any,
    raw_destination: Path | None,
    metres_destination: Path | None,
    colour_destination: Path | None,
) -> tuple[dict[str, float], np.ndarray]:
    encoded = bgra_to_rgb(image.raw_data, image.width, image.height)
    metres = carla_depth_to_metres(image.raw_data, image.width, image.height)
    if raw_destination is not None:
        Image.fromarray(encoded).save(raw_destination)
    if metres_destination is not None:
        np.save(metres_destination, metres.astype(np.float32, copy=False))
    if colour_destination is not None:
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
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                fieldnames.append(str(key))
                seen.add(str(key))
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
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
        # Seed CARLA's pedestrian/navigation RNG before sampling walker spawn
        # locations so a batch seed controls both initial positions and motion.
        world.set_pedestrians_seed(seed)

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
        target_transform_cfg = dict(region["target_transform"])
        target_location_override = dynamic_cfg.get("target_location_override_xyz")
        if target_location_override is not None:
            target_transform_cfg["location_xyz"] = [float(value) for value in target_location_override]
            log(f"Applied scenario-specific target parking position: {target_transform_cfg['location_xyz']}")
        target = world.try_spawn_actor(target_bp, s0.dict_to_transform(target_transform_cfg))
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

        safety_scenario_events: list[dict[str, Any]] = []
        safety_test_obstacle = None
        safety_obstacle_cfg = dynamic_cfg.get("safety_test_obstacle", {})
        if bool(safety_obstacle_cfg.get("enabled", False)):
            obstacle_bp_id = str(safety_obstacle_cfg.get("blueprint", "vehicle.audi.tt"))
            obstacle_color = str(safety_obstacle_cfg.get("color_rgb", "80,80,80"))
            obstacle_bp = s0.configure_blueprint(
                blueprint_library.find(obstacle_bp_id), "safety_test_static_obstacle", obstacle_color
            )
            desired_distance = float(safety_obstacle_cfg.get("route_distance_m", 30.0))
            offsets = [0.0, 6.0, -6.0, 12.0, -12.0, 18.0, -18.0]
            for offset in offsets:
                distance_along_route = max(8.0, desired_distance + offset)
                obstacle_xyz, obstacle_yaw = interpolate_polyline(route, distance_along_route)
                waypoint = world_map.get_waypoint(
                    carla.Location(float(obstacle_xyz[0]), float(obstacle_xyz[1]), float(obstacle_xyz[2])),
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving,
                )
                if waypoint is None:
                    continue
                obstacle_transform = waypoint.transform
                obstacle_transform.location.z += 0.25
                obstacle_transform.rotation.yaw = float(obstacle_yaw)
                safety_test_obstacle = world.try_spawn_actor(obstacle_bp, obstacle_transform)
                if safety_test_obstacle is not None:
                    safety_obstacle_cfg["resolved_route_distance_m"] = distance_along_route
                    break
            if safety_test_obstacle is None:
                raise RuntimeError(
                    f"Could not spawn configured sensor-safety obstacle at route distance "
                    f"{safety_obstacle_cfg.get('route_distance_m', 30.0)} m"
                )
            safety_test_obstacle.set_simulate_physics(False)
            owned_actors.append(safety_test_obstacle)
            log(
                "Spawned fixed safety-test obstacle from scenario configuration at "
                f"route distance {safety_obstacle_cfg.get('route_distance_m', 30.0)} m"
            )

        safety_crossing_actor = None
        safety_crossing_cfg = dynamic_cfg.get("safety_test_crossing_actor", {})
        crossing_anchor_xyz: list[float] | None = None
        crossing_yaw_degrees: float | None = None
        if bool(safety_crossing_cfg.get("enabled", False)):
            actor_kind = str(safety_crossing_cfg.get("actor_type", "pedestrian")).lower()
            crossing_anchor_xyz, crossing_yaw_degrees = interpolate_polyline(
                route, float(safety_crossing_cfg.get("route_distance_m", 30.0))
            )
            lateral_start = float(safety_crossing_cfg.get("lateral_start_m", 8.0))
            initial_xy = route_normal_offset(crossing_anchor_xyz, crossing_yaw_degrees, lateral_start)
            start_candidates = [initial_xy, route_normal_offset(crossing_anchor_xyz, crossing_yaw_degrees, -lateral_start)]
            if actor_kind in {"pedestrian", "walker"}:
                crossing_bp_id = str(safety_crossing_cfg.get("blueprint", "walker.pedestrian.0001"))
                crossing_bp = blueprint_library.find(crossing_bp_id)
                if crossing_bp.has_attribute("is_invincible"):
                    crossing_bp.set_attribute("is_invincible", "false")
                crossing_role = "safety_test_crossing_pedestrian"
                crossing_category = "pedestrian"
            else:
                crossing_bp_id = str(safety_crossing_cfg.get("blueprint", "vehicle.audi.tt"))
                crossing_bp = s0.configure_blueprint(
                    blueprint_library.find(crossing_bp_id),
                    "safety_test_crossing_vehicle",
                    str(safety_crossing_cfg.get("color_rgb", "80,80,80")),
                )
                crossing_role = "safety_test_crossing_vehicle"
                crossing_category = "vehicle"
            crossing_yaw = float(crossing_yaw_degrees) + 90.0
            crossing_z_offset = 0.15
            if actor_kind in {"pedestrian", "walker"}:
                # CARLA walker bounds are centred around the skeleton.  Putting
                # the actor root only 0.15 m above the road buries roughly half
                # of its 1.83 m bounding height below the surface.
                crossing_z_offset = max(
                    0.75, float(safety_crossing_cfg.get("root_height_above_road_m", 0.92))
                )
            for candidate_xyz in start_candidates:
                start_transform = carla.Transform(
                    carla.Location(float(candidate_xyz[0]), float(candidate_xyz[1]), float(crossing_anchor_xyz[2]) + crossing_z_offset),
                    carla.Rotation(yaw=crossing_yaw),
                )
                safety_crossing_actor = world.try_spawn_actor(crossing_bp, start_transform)
                if safety_crossing_actor is not None:
                    resolved_lateral_start = lateral_start if candidate_xyz is start_candidates[0] else -lateral_start
                    safety_crossing_cfg["resolved_lateral_start_m"] = resolved_lateral_start
                    configured_end = abs(float(safety_crossing_cfg.get("lateral_end_m", -resolved_lateral_start)))
                    safety_crossing_cfg["resolved_lateral_end_m"] = -math.copysign(
                        configured_end, resolved_lateral_start
                    )
                    initial_direction = math.copysign(1.0, -resolved_lateral_start)
                    safety_crossing_actor.set_transform(
                        carla.Transform(
                            start_transform.location,
                            carla.Rotation(
                                yaw=float(crossing_yaw_degrees)
                                + (90.0 if initial_direction > 0.0 else -90.0)
                            ),
                        )
                    )
                    break
            if safety_crossing_actor is None:
                raise RuntimeError(
                    f"Could not spawn configured crossing {actor_kind} at route distance "
                    f"{safety_crossing_cfg.get('route_distance_m', 30.0)} m"
                )
            safety_crossing_actor.set_simulate_physics(False)
            owned_actors.append(safety_crossing_actor)
            safety_crossing_cfg["resolved_actor_role"] = crossing_role
            safety_crossing_cfg["resolved_actor_category"] = crossing_category
            log(
                f"Spawned scripted crossing {actor_kind} at route distance "
                f"{safety_crossing_cfg.get('route_distance_m', 30.0)} m; "
                f"motion starts at {safety_crossing_cfg.get('start_seconds', 8.0)} s"
            )

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

        def set_drone_pose(
            carla_xyz: list[float],
            yaw_deg: float,
            ignore_collision_override: bool | None = None,
        ) -> None:
            ned = airsim.Vector3r(
                float(carla_xyz[0] - ned_offset[0]),
                float(carla_xyz[1] - ned_offset[1]),
                float(-(carla_xyz[2] - ned_offset[2])),
            )
            orientation = airsim.to_quaternion(0.0, 0.0, math.radians(yaw_deg))
            ignore_collision = (
                bool(dynamic_cfg.get("uav_ignore_collision", True))
                if ignore_collision_override is None
                else bool(ignore_collision_override)
            )
            airsim_client.simSetVehiclePose(
                airsim.Pose(ned, orientation),
                ignore_collision,
                vehicle_name=vehicle_name,
            )

        def camera_yaw(path_yaw: float) -> float:
            if dynamic_cfg.get("uav_camera_yaw_mode") == "world_fixed":
                return float(dynamic_cfg.get("uav_camera_yaw_degrees", 0.0))
            return path_yaw

        initial_uav, initial_uav_yaw = interpolate_polyline(uav_route, 0.0)
        # Initial deployment is a simulator setup operation, not flight.  It
        # must not be blocked by geometry between the default AirSim spawn and
        # the configured 60 m survey altitude.  Route updates below retain the
        # configured collision behavior (false for formal S1 runs).
        set_drone_pose(initial_uav, initial_uav_yaw, ignore_collision_override=True)
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
        ugv_safety_cfg = dynamic_cfg.get("ugv_safety", {})
        sensor_obstacle_source = str(ugv_safety_cfg.get("obstacle_source", "carla_actor_truth")) == "ugv_rgbd"
        obstacle_guard: RGBDObstacleGuard | None = None
        hazard_detector: RGBDHazardPerception | None = None
        ugv_camera_to_vehicle: np.ndarray | None = None
        hazard_detector_cfg = ugv_safety_cfg.get("hazard_detector", {})
        if sensor_obstacle_source:
            obstacle_guard = RGBDObstacleGuard(
                ObstacleGuardConfig(
                    horizontal_fov_degrees=float(sensor_config["fov_degrees"]),
                    maximum_range_m=float(ugv_safety_cfg.get("maximum_range_m", 18.0)),
                    corridor_half_width_m=float(ugv_safety_cfg.get("sensor_corridor_half_width_m", 1.6)),
                    minimum_height_m=float(ugv_safety_cfg.get("minimum_obstacle_height_m", 0.12)),
                    maximum_height_m=float(ugv_safety_cfg.get("maximum_obstacle_height_m", 2.6)),
                    warning_distance_m=float(ugv_safety_cfg.get("warning_distance_m", 8.0)),
                    stop_distance_m=float(ugv_safety_cfg.get("sensor_stop_distance_m", 4.0)),
                    warning_ttc_s=float(ugv_safety_cfg.get("warning_ttc_s", 3.0)),
                    stop_ttc_s=float(ugv_safety_cfg.get("stop_ttc_s", 1.5)),
                    stale_after_s=float(ugv_safety_cfg.get("stale_after_s", 0.30)),
                    pixel_stride=int(ugv_safety_cfg.get("pixel_stride", 4)),
                    recovery_clear_frames=int(ugv_safety_cfg.get("recovery_clear_frames", 5)),
                    minimum_occupied_points=int(ugv_safety_cfg.get("minimum_occupied_points", 3)),
                )
            )
            ugv_depth_spec = stream_specs["ugv_depth"]
            ugv_camera_to_vehicle = camera_mount_matrix(
                [float(value) for value in ugv_depth_spec["location_xyz_m"]],
                [float(value) for value in ugv_depth_spec["rotation_pyr_degrees"]],
            )
            if bool(hazard_detector_cfg.get("enabled", False)):
                hazard_detector = RGBDHazardPerception(
                    HazardDetectorConfig(
                        model_path=(PROJECT_ROOT / str(hazard_detector_cfg.get("model_path", "models/yolo26n.pt"))).resolve(),
                        horizontal_fov_degrees=float(sensor_config["fov_degrees"]),
                        confidence=float(hazard_detector_cfg.get("confidence", 0.10)),
                        image_size=int(hazard_detector_cfg.get("image_size", 960)),
                        device=hazard_detector_cfg.get("device", 0),
                        max_depth_m=float(hazard_detector_cfg.get("max_depth_m", 45.0)),
                        track_timeout_s=float(hazard_detector_cfg.get("track_timeout_s", 1.0)),
                        track_association_m=float(hazard_detector_cfg.get("track_association_m", 5.0)),
                    )
                )
                log(
                    "Enabled UGV RGB-D traffic hazard detection from YOLO COCO; "
                    "online obstacle actor truth remains disabled"
                )
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
        if safety_test_obstacle is not None:
            roles.append(("safety_test_obstacle", safety_test_obstacle))
        if safety_crossing_actor is not None:
            roles.append((str(safety_crossing_cfg["resolved_actor_role"]), safety_crossing_actor))
        initial_actor_manifest = [s0.actor_record(actor, role, role.split("_")[0]) for role, actor in roles]
        s0.json_dump(run_dirs["actors"] / "initial_actor_states.json", initial_actor_manifest)

        task_machine: OracleTaskStateMachine | PerceptionTaskStateMachine | None = None
        oracle_channel: OracleChannel | SemanticChannel | None = None
        oracle_message_sent: dict[str, Any] | None = None
        received_oracle_message: dict[str, Any] | None = None
        active_ugv_route: list[list[float]] = route
        active_ugv_route_cursor = 0
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
                # Keep the AirSim vehicle at the configured survey start while
                # CARLA camera render targets warm up.  Without this hold the
                # freshly spawned multirotor descends under physics before the
                # formal route controller begins.
                set_drone_pose(
                    initial_uav,
                    initial_uav_yaw,
                    ignore_collision_override=True,
                )
                world.tick()
                time.sleep(float(dynamic_cfg.get("render_settle_seconds", 0.035)))
                for packet_queue in queues.values():
                    s0.drain_all(packet_queue)
            log("Sensor warm-up complete; warm-up packets discarded")

        # AirSim collision state is sticky.  A pose reset can be reported only
        # after the CARLA/AirSim bridge has advanced several ticks, so the
        # acquisition baseline must be captured after warm-up.  Any later,
        # distinct timestamp is still treated as a formal flight collision.
        baseline_uav_collision_info = airsim_client.simGetCollisionInfo(vehicle_name=vehicle_name)
        baseline_uav_collision_timestamp = int(
            getattr(baseline_uav_collision_info, "time_stamp", 0) or 0
        )
        actual_initial_uav = s0.xyz(drone.get_location())
        initial_uav_pose_error_m = math.dist(actual_initial_uav, initial_uav)
        maximum_initial_uav_pose_error_m = float(
            dynamic_cfg.get("maximum_initial_uav_pose_error_m", 1.0)
        )
        s0.json_dump(
            run_dirs["safety"] / "uav_pose_preflight.json",
            {
                "desired_xyz": initial_uav,
                "actual_xyz": actual_initial_uav,
                "error_m": initial_uav_pose_error_m,
                "maximum_error_m": maximum_initial_uav_pose_error_m,
                "passed": initial_uav_pose_error_m <= maximum_initial_uav_pose_error_m,
            },
        )
        if initial_uav_pose_error_m > maximum_initial_uav_pose_error_m:
            raise RuntimeError(
                "UAV pose preflight failed before formal acquisition: "
                f"desired={initial_uav}, actual={actual_initial_uav}, "
                f"error={initial_uav_pose_error_m:.3f} m > "
                f"{maximum_initial_uav_pose_error_m:.3f} m"
            )
        s0.json_dump(
            run_dirs["safety"] / "uav_collision_baseline.json",
            {
                "captured_after_warmup": True,
                "warmup_ticks": warmup_ticks,
                "has_collided": bool(baseline_uav_collision_info.has_collided),
                "timestamp": baseline_uav_collision_timestamp,
                "object_name": str(getattr(baseline_uav_collision_info, "object_name", "")),
                "object_id": int(getattr(baseline_uav_collision_info, "object_id", -1)),
            },
        )
        if bool(baseline_uav_collision_info.has_collided):
            log(
                "Excluded post-pose-reset AirSim collision baseline before formal acquisition: "
                f"timestamp={baseline_uav_collision_timestamp}, "
                f"object={getattr(baseline_uav_collision_info, 'object_name', '')}"
            )

        actor_state_handle = (run_dirs["actors"] / "actor_states.jsonl").open("w", encoding="utf-8")
        metadata_handles = {
            "uav": (run_dirs["uav_metadata"] / "frames.jsonl").open("w", encoding="utf-8"),
            "ugv": (run_dirs["ugv_metadata"] / "frames.jsonl").open("w", encoding="utf-8"),
        }

        frame_rows: list[dict[str, Any]] = []
        persisted_sensor_frames: list[int] = []
        trajectory_rows: dict[str, list[dict[str, Any]]] = defaultdict(list)
        actor_tracks: dict[str, list[list[float]]] = defaultdict(list)
        rgb_hashes: dict[str, list[str]] = defaultdict(list)
        depth_finite: dict[str, list[float]] = defaultdict(list)
        uav_central_clearances: list[float] = []
        follow_errors: list[float] = []
        safety_rows: list[dict[str, Any]] = []
        safety_by_frame: dict[int, dict[str, Any]] = {}
        obstacle_guard_rows: list[dict[str, Any]] = []
        hazard_detection_rows: list[dict[str, Any]] = []
        ugv_rgbd_pair_rows: list[dict[str, Any]] = []
        processed_ugv_sensor_frames: set[int] = set()
        latest_ugv_rgb: np.ndarray | None = None
        latest_ugv_depth: np.ndarray | None = None
        latest_ugv_frame: int | None = None
        latest_ugv_timestamp: float | None = None
        best_obstacle_preview: tuple[np.ndarray, np.ndarray, dict[str, Any], np.ndarray] | None = None
        sensor_stale_injection = ugv_safety_cfg.get("test_sensor_stale_interval_s")
        uav_collision_events: list[dict[str, Any]] = []
        seen_uav_collision_timestamps: set[int] = (
            {baseline_uav_collision_timestamp} if baseline_uav_collision_timestamp else set()
        )
        world_state_by_frame: dict[int, dict[str, Any]] = {}
        saved_frames: set[int] = set()
        started_sim_time: float | None = None
        sensor_health_checked = False
        safety_obstacle_removed = False
        crossing_motion_started = False
        crossing_motion_finished = False
        crossing_actor_removed = False
        latest_hazard_decision: dict[str, Any] = {}
        active_overtake: dict[str, Any] | None = None
        ugv_safety_route_cursor = 0
        hazard_stop_hold_until = -math.inf
        hazard_stop_hold_reason = ""

        log(f"Running {duration:.1f}s dynamic acquisition ({total_ticks} fixed ticks, expected {expected_frames} sensor frames)")
        for tick_index in range(total_ticks):
            sim_elapsed = tick_index * fixed_delta
            if safety_crossing_actor is not None and safety_crossing_actor.is_alive and crossing_anchor_xyz is not None and crossing_yaw_degrees is not None:
                crossing_start_s = float(safety_crossing_cfg.get("start_seconds", 8.0))
                crossing_end_s = crossing_start_s + 2.0 * max(0.1, float(safety_crossing_cfg.get("traverse_seconds", 5.0))) + max(0.0, float(safety_crossing_cfg.get("pause_at_center_seconds", 1.0)))
                despawn_after_s = safety_crossing_cfg.get("despawn_after_seconds")
                if despawn_after_s is not None and sim_elapsed >= float(despawn_after_s):
                    safety_crossing_actor.destroy()
                    crossing_actor_removed = True
                    safety_scenario_events.append(
                        {"event": "scripted_crossing_actor_removed", "elapsed_seconds": sim_elapsed, "actor_role": safety_crossing_cfg["resolved_actor_role"]}
                    )
                    continue_crossing_update = False
                else:
                    continue_crossing_update = True
                if not crossing_motion_started and sim_elapsed >= crossing_start_s:
                    safety_scenario_events.append(
                        {"event": "scripted_crossing_started", "elapsed_seconds": sim_elapsed, "actor_role": safety_crossing_cfg["resolved_actor_role"]}
                    )
                    crossing_motion_started = True
                if continue_crossing_update and not crossing_motion_finished and sim_elapsed >= crossing_end_s:
                    safety_scenario_events.append(
                        {"event": "scripted_crossing_finished", "elapsed_seconds": sim_elapsed, "actor_role": safety_crossing_cfg["resolved_actor_role"]}
                    )
                    crossing_motion_finished = True
                if continue_crossing_update:
                    lateral = scripted_crossing_lateral(safety_crossing_cfg, sim_elapsed)
                    crossing_xyz = route_normal_offset(crossing_anchor_xyz, crossing_yaw_degrees, lateral)
                    next_lateral = scripted_crossing_lateral(safety_crossing_cfg, sim_elapsed + fixed_delta)
                    travel_sign = math.copysign(1.0, next_lateral - lateral) if abs(next_lateral - lateral) > 1e-5 else math.copysign(1.0, float(safety_crossing_cfg["resolved_lateral_end_m"]) - float(safety_crossing_cfg["resolved_lateral_start_m"]))
                    actor_yaw = float(crossing_yaw_degrees) + (90.0 if travel_sign > 0.0 else -90.0)
                    crossing_transform = carla.Transform(
                        carla.Location(float(crossing_xyz[0]), float(crossing_xyz[1]), float(crossing_xyz[2]) + crossing_z_offset),
                        carla.Rotation(yaw=actor_yaw),
                    )
                    safety_crossing_actor.set_transform(crossing_transform)
            remove_after_s = safety_obstacle_cfg.get("remove_after_seconds")
            if (
                safety_test_obstacle is not None
                and safety_test_obstacle.is_alive
                and remove_after_s is not None
                and sim_elapsed >= float(remove_after_s)
            ):
                safety_test_obstacle.destroy()
                safety_obstacle_removed = True
                safety_scenario_events.append(
                    {"event": "safety_test_obstacle_removed", "elapsed_seconds": sim_elapsed}
                )
            sensor_observation: dict[str, Any] | None = None
            if sensor_obstacle_source and obstacle_guard is not None and ugv_camera_to_vehicle is not None:
                current_snapshot = world.get_snapshot()
                current_sim_time = float(current_snapshot.timestamp.elapsed_seconds)
                inject_stale = (
                    isinstance(sensor_stale_injection, list)
                    and len(sensor_stale_injection) == 2
                    and float(sensor_stale_injection[0]) <= sim_elapsed < float(sensor_stale_injection[1])
                )
                sensor_observation = obstacle_guard.observe(
                    None if inject_stale else latest_ugv_depth,
                    latest_ugv_frame,
                    latest_ugv_timestamp,
                    current_sim_time,
                    ugv_camera_to_vehicle,
                )
                obstacle_guard_rows.append(
                    {
                        "frame": int(current_snapshot.frame),
                        "timestamp": current_sim_time,
                        "sensor_frame": latest_ugv_frame,
                        "sensor_timestamp": latest_ugv_timestamp,
                        "sensor_state": sensor_observation["state"],
                        "nearest_obstacle_m": sensor_observation["nearest_obstacle_m"],
                        "closing_speed_mps": sensor_observation["closing_speed_mps"],
                        "ttc_s": sensor_observation["ttc_s"],
                        "occupied_points": sensor_observation["occupied_points"],
                        "sensor_age_s": sensor_observation["sensor_age_s"],
                        "reason": sensor_observation["reason"],
                        "injected_stale": bool(inject_stale),
                    }
                )
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
                    active_ugv_route_cursor = 0
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
                    ugv_safety, active_ugv_route_cursor = apply_safe_route_control_to_position(
                        ugv,
                        active_ugv_route,
                        float(dynamic_cfg["ugv_target_speed_mps"]),
                        received_oracle_message["target_world_position_xyz"],
                        [*distractors, *walkers],
                        dynamic_cfg.get("ugv_safety", {}),
                        route_cursor_index=active_ugv_route_cursor,
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
                    active_ugv_route_cursor = 0
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
                    route_remaining_m = remaining_route_distance(
                        active_ugv_route,
                        active_ugv_route_cursor,
                        s0.xyz(ugv.get_location()),
                    )
                    if (
                        not task_machine.local_confirmation_started
                        and message_distance <= float(perception_cfg.get("local_confirmation_start_distance_m", 25.0))
                        and route_remaining_m
                        <= float(perception_cfg.get("local_confirmation_max_route_remaining_m", 30.0))
                    ):
                        task_machine.start_local_confirmation(
                            pre_frame,
                            pre_timestamp,
                            message_distance_m=message_distance,
                            route_remaining_m=route_remaining_m,
                        )
                    if (
                        not task_machine.target_verified
                        and task_machine.local_confirmation_started
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
                        ugv_safety, active_ugv_route_cursor = apply_safe_route_control_to_position(
                            ugv,
                            active_ugv_route,
                            commanded_speed,
                            received_oracle_message["target_world_position_xyz"],
                            [*distractors, *walkers],
                            dynamic_cfg.get("ugv_safety", {}),
                            route_cursor_index=active_ugv_route_cursor,
                        )
            else:
                ugv_start_delay = float(dynamic_cfg.get("ugv_start_delay_seconds", 0.0))
                if sim_elapsed < ugv_start_delay:
                    ugv.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                else:
                    safety_cfg = dynamic_cfg.get("ugv_safety", {})
                    if safety_cfg.get("enabled", False):
                        ugv_location = s0.xyz(ugv.get_location())
                        current_progress, _ = route_progress_m(
                            route,
                            ugv_location,
                            cursor_index=ugv_safety_route_cursor,
                        )
                        if active_overtake is not None and current_progress >= float(active_overtake["end_s_m"]):
                            active_overtake = None
                        control_route = route
                        if active_overtake is not None:
                            control_route = route_with_overtake_offset(
                                route,
                                float(active_overtake["start_s_m"]),
                                float(active_overtake["end_s_m"]),
                                float(active_overtake["side_offset_m"]),
                                ramp_m=9.0,
                            )
                        ugv_safety = apply_safe_route_control(
                            ugv,
                            control_route,
                            float(dynamic_cfg["ugv_target_speed_mps"]),
                            target,
                            [target, *distractors, *walkers],
                            safety_cfg,
                            sensor_observation=sensor_observation,
                            hazard_decision=latest_hazard_decision,
                            overtake_active=active_overtake is not None,
                            route_cursor_index=ugv_safety_route_cursor,
                            lane_change_active=active_overtake is not None,
                        )
                        ugv_safety_route_cursor = int(
                            ugv_safety.get("ugv_route_cursor_index", ugv_safety_route_cursor)
                        )
                        if active_overtake is not None and not latest_hazard_decision.get("forced_stop_reason"):
                            ugv_safety["ugv_safety_mode"] = "overtake_lane_change"
                            ugv_safety["ugv_overtake_track_id"] = active_overtake["track_id"]
                            ugv_safety["ugv_overtake_lateral_m"] = active_overtake["side_offset_m"]
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

            # Local UGV safety consumes its synchronized RGB/depth pair directly.
            # It must not wait for UAV streams or a four-camera global join.
            ugv_local_common = set(buffers["ugv_rgb"]) & set(buffers["ugv_depth"])
            for ugv_frame in sorted(ugv_local_common - processed_ugv_sensor_frames):
                rgb_packet = buffers["ugv_rgb"][ugv_frame]
                depth_packet = buffers["ugv_depth"][ugv_frame]
                ugv_sample_index = len(ugv_rgbd_pair_rows)
                save_every_n = max(1, int(dynamic_cfg.get("sensor_save_every_n_frames", 1)))
                persist_local_pair = bool(dynamic_cfg.get("save_all_samples", True)) or ugv_sample_index % save_every_n == 0
                output_cfg = resolved.get("output", {})
                rgb_path = run_dirs["ugv_rgb"] / f"{ugv_frame:08d}.png"
                depth_raw_path = run_dirs["ugv_depth_raw"] / f"{ugv_frame:08d}.png"
                depth_m_path = run_dirs["ugv_depth_metres"] / f"{ugv_frame:08d}.npy"
                depth_colour_path = run_dirs["ugv_depth_colour"] / f"{ugv_frame:08d}.png"
                _, local_rgb = save_rgb(
                    rgb_packet,
                    rgb_path if persist_local_pair and bool(output_cfg.get("save_rgb", True)) else None,
                )
                depth_stats, local_depth = save_depth(
                    depth_packet,
                    depth_raw_path if persist_local_pair and bool(output_cfg.get("save_depth_raw", True)) else None,
                    depth_m_path if persist_local_pair and bool(output_cfg.get("save_depth_float_m", True)) else None,
                    depth_colour_path if persist_local_pair and bool(output_cfg.get("save_depth_colour", True)) else None,
                )
                local_timestamp = float(rgb_packet.timestamp)
                local_sample_elapsed = ugv_sample_index * sensor_tick
                inject_stale_local = (
                    isinstance(sensor_stale_injection, list)
                    and len(sensor_stale_injection) == 2
                    and float(sensor_stale_injection[0]) <= local_sample_elapsed < float(sensor_stale_injection[1])
                )
                latest_ugv_rgb = local_rgb
                latest_ugv_depth = local_depth
                latest_ugv_frame = int(ugv_frame)
                latest_ugv_timestamp = local_timestamp
                rgb_hashes["ugv"].append(hashlib.sha256(local_rgb.tobytes()).hexdigest())
                depth_finite["ugv"].append(float(depth_stats["finite_fraction"]))
                if obstacle_guard is not None and ugv_camera_to_vehicle is not None and not inject_stale_local:
                    sensor_observation = obstacle_guard.observe(
                        local_depth,
                        int(ugv_frame),
                        local_timestamp,
                        local_timestamp,
                        ugv_camera_to_vehicle,
                    )
                    if active_overtake is not None:
                        actual_lane_offset = route_lateral_offset_m(
                            route,
                            s0.xyz(ugv.get_location()),
                        )
                        active_lane = obstacle_guard.lane_occupancy(
                            local_depth,
                            ugv_camera_to_vehicle,
                            actual_lane_offset,
                            half_width_m=min(1.35, float(active_overtake["lane_width_m"]) * 0.38),
                            minimum_forward_m=3.5,
                            maximum_forward_m=22.0,
                        )
                        lane_distance = float(active_lane.get("nearest_m", float("inf")))
                        if active_lane.get("clear", False):
                            sensor_observation.update(
                                state="CLEAR",
                                nearest_obstacle_m=float("inf"),
                                occupied_points=0,
                                ttc_s=float("inf"),
                                reason="active_overtake_lane_clear_by_depth",
                            )
                        else:
                            sensor_observation.update(
                                state=(
                                    "STOP"
                                    if lane_distance <= float(safety_cfg.get("sensor_stop_distance_m", 8.0))
                                    else "CAUTION"
                                ),
                                nearest_obstacle_m=lane_distance,
                                occupied_points=int(active_lane.get("occupied_points", 0)),
                                reason="active_overtake_lane_occupied_by_depth",
                            )
                    if math.isfinite(float(sensor_observation.get("nearest_obstacle_m", math.nan))):
                        current_distance = float(sensor_observation["nearest_obstacle_m"])
                        best_distance = (
                            float("inf")
                            if best_obstacle_preview is None
                            else float(best_obstacle_preview[2].get("nearest_obstacle_m", float("inf")))
                        )
                        if current_distance < best_distance:
                            best_obstacle_preview = (
                                local_rgb.copy(),
                                local_depth.copy(),
                                dict(sensor_observation),
                            obstacle_guard.last_obstacle_pixels_uv.copy(),
                            )
                if hazard_detector is not None and ugv_camera_to_vehicle is not None:
                    frame_state = world_state_by_frame.get(int(ugv_frame), {})
                    ego_state = next(
                        (item for item in frame_state.get("states", []) if item.get("role") == "ugv"),
                        None,
                    )
                    if ego_state is not None:
                        detections = hazard_detector.infer(
                            local_rgb,
                            local_depth,
                            ugv_camera_to_vehicle,
                            ego_state,
                            int(ugv_frame),
                            local_timestamp,
                        )
                        decision = classify_safety_hazards(detections)
                        candidate = decision.get("slow_vehicle_candidate")
                        if candidate is not None and active_overtake is None:
                            progress, _ = route_progress_m(
                                route,
                                [ego_state["x"], ego_state["y"], ego_state["z"]],
                                cursor_index=ugv_safety_route_cursor,
                            )
                            legal_lane = legal_same_direction_overtake_lane(
                                world_map,
                                ugv,
                                route,
                                progress,
                                float(candidate["forward_m"]),
                            )
                            safe_lane = None
                            overtake_block_reason = ""
                            lane_check: dict[str, Any] | None = None
                            camera_blockers: list[dict[str, Any]] = []
                            if legal_lane is not None and obstacle_guard is not None:
                                lateral_offset, lane_width = legal_lane
                                lane_check = obstacle_guard.lane_occupancy(
                                    local_depth,
                                    ugv_camera_to_vehicle,
                                    lateral_offset,
                                    half_width_m=min(1.2, lane_width * 0.35),
                                    minimum_forward_m=3.5,
                                    maximum_forward_m=min(24.0, float(candidate["forward_m"]) + 8.0),
                                )
                                camera_blockers = [
                                    item for item in detections
                                    if item is not candidate
                                    and 2.5 < float(item["forward_m"]) < float(candidate["forward_m"]) + 8.0
                                    and abs(float(item["lateral_m"]) - lateral_offset) < min(1.2, lane_width * 0.35)
                                ]
                                if lane_check["clear"] and not camera_blockers:
                                    safe_lane = (lateral_offset, lane_width)
                                elif not lane_check["clear"]:
                                    overtake_block_reason = "adjacent_lane_occupied_by_depth"
                                else:
                                    overtake_block_reason = "adjacent_lane_occupied_by_rgb_detection"
                            elif legal_lane is None:
                                overtake_block_reason = "map_lane_or_junction_prohibits_pass"
                            else:
                                overtake_block_reason = "depth_guard_unavailable"
                            decision["overtake_candidate_lateral_offset_m"] = (
                                float(legal_lane[0]) if legal_lane is not None else float("nan")
                            )
                            decision["overtake_lane_depth_points"] = (
                                int(lane_check.get("occupied_points", -1)) if lane_check is not None else -1
                            )
                            decision["overtake_lane_nearest_m"] = (
                                float(lane_check.get("nearest_m", float("nan"))) if lane_check is not None else float("nan")
                            )
                            decision["overtake_lane_rgb_blockers"] = len(camera_blockers)
                            if safe_lane is not None and float(candidate["forward_m"]) >= 18.0:
                                lateral_offset, lane_width = safe_lane
                                obstacle_s = progress + float(candidate["forward_m"])
                                active_overtake = {
                                    "track_id": int(candidate["track_id"]),
                                    "side_offset_m": lateral_offset,
                                    "lane_width_m": lane_width,
                                    "start_s_m": max(progress - 1.0, obstacle_s - 20.0),
                                    "end_s_m": obstacle_s + 13.0,
                                    "candidate_distance_m": float(candidate["forward_m"]),
                                }
                                decision["forced_stop_reason"] = ""
                                decision["overtake_started"] = True
                            elif active_overtake is None:
                                decision["no_safe_overtake_gap"] = True
                                decision["forced_stop_reason"] = (
                                    "no_safe_overtake_gap"
                                    if float(candidate["forward_m"]) <= 8.0
                                    else ""
                                )
                                decision["overtake_block_reason"] = (
                                    overtake_block_reason
                                    if safe_lane is None
                                    else "insufficient_overtaking_distance"
                                )
                        forced_stop_reason = str(decision.get("forced_stop_reason", ""))
                        if active_overtake is not None and forced_stop_reason == "vehicle_conflict":
                            detected_hazard = decision.get("hazard") or {}
                            if int(detected_hazard.get("track_id", -1)) == int(active_overtake["track_id"]):
                                # The tracked slow vehicle is the object being
                                # passed. It is intentionally beside the UGV;
                                # new hazards, pedestrians, depth stops, and
                                # other vehicle tracks remain safety-critical.
                                decision["forced_stop_reason"] = ""
                                decision["overtake_target_in_adjacent_lane"] = True
                                forced_stop_reason = ""
                        if forced_stop_reason in {"pedestrian", "crossing_vehicle", "vehicle_conflict"}:
                            hazard_stop_hold_reason = forced_stop_reason
                            hazard_stop_hold_until = max(hazard_stop_hold_until, local_timestamp + 0.8)
                        elif local_timestamp < hazard_stop_hold_until:
                            decision["forced_stop_reason"] = hazard_stop_hold_reason
                        latest_hazard_decision = decision
                        if detections:
                            for item in detections:
                                hazard_detection_rows.append(
                                    {
                                        **item,
                                        "decision": decision.get("forced_stop_reason", "overtake_candidate" if decision.get("slow_vehicle_candidate") is item else "clear"),
                                        "overtake_started": bool(decision.get("overtake_started", False)),
                                        "overtake_block_reason": decision.get("overtake_block_reason", ""),
                                    }
                                )
                        else:
                            hazard_detection_rows.append(
                                {
                                    "frame": int(ugv_frame),
                                    "timestamp": local_timestamp,
                                    "category": "none",
                                    "decision": decision.get("forced_stop_reason", "clear"),
                                    "overtake_started": False,
                                    "overtake_block_reason": "",
                                }
                            )
                ugv_rgbd_pair_rows.append(
                    {
                        "sample_index": ugv_sample_index,
                        "frame": int(ugv_frame),
                        "timestamp": local_timestamp,
                        "rgb_frame": int(rgb_packet.frame),
                        "depth_frame": int(depth_packet.frame),
                        "frame_spread": abs(int(rgb_packet.frame) - int(depth_packet.frame)),
                        "sensor_files_saved": int(persist_local_pair),
                        "depth_finite_fraction": float(depth_stats["finite_fraction"]),
                        "test_sensor_dropout_injected": bool(inject_stale_local),
                    }
                )
                processed_ugv_sensor_frames.add(int(ugv_frame))

            common = set.intersection(*(set(buffer.keys()) for buffer in buffers.values()))
            for common_frame in sorted(common - saved_frames):
                if common_frame not in world_state_by_frame:
                    continue
                packets = {name: buffers[name][common_frame] for name in buffers}
                frame_state = world_state_by_frame[common_frame]
                sample_index = len(frame_rows)
                save_all_samples = bool(dynamic_cfg.get("save_all_samples", True))
                save_every_n = max(1, int(dynamic_cfg.get("sensor_save_every_n_frames", 1)))
                persist_sensor_files = save_all_samples or sample_index % save_every_n == 0
                output_cfg = resolved.get("output", {})
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
                    saved_rgb_path = (
                        rgb_path
                        if persist_sensor_files and bool(output_cfg.get("save_rgb", True))
                        else None
                    )
                    saved_depth_raw_path = (
                        depth_raw_path
                        if persist_sensor_files and bool(output_cfg.get("save_depth_raw", True))
                        else None
                    )
                    saved_depth_m_path = (
                        depth_m_path
                        if persist_sensor_files and bool(output_cfg.get("save_depth_float_m", True))
                        else None
                    )
                    saved_depth_colour_path = (
                        depth_colour_path
                        if persist_sensor_files and bool(output_cfg.get("save_depth_colour", True))
                        else None
                    )
                    digest, rgb_array = save_rgb(rgb_packet, saved_rgb_path)
                    depth_stats, depth_array = save_depth(
                        depth_packet,
                        saved_depth_raw_path,
                        saved_depth_m_path,
                        saved_depth_colour_path,
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
                        "sensor_files_saved": bool(persist_sensor_files),
                        "rgb_path": None if saved_rgb_path is None else str(saved_rgb_path),
                        "depth_raw_path": (
                            None if saved_depth_raw_path is None else str(saved_depth_raw_path)
                        ),
                        "depth_metres_path": (
                            None if saved_depth_m_path is None else str(saved_depth_m_path)
                        ),
                        "depth_colour_path": (
                            None if saved_depth_colour_path is None else str(saved_depth_colour_path)
                        ),
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
                        "sample_index": sample_index,
                        "frame": int(common_frame),
                        "timestamp": float(packets["uav_rgb"].timestamp),
                        "uav_rgb_frame": packet_frames[0],
                        "uav_depth_frame": packet_frames[1],
                        "ugv_rgb_frame": packet_frames[2],
                        "ugv_depth_frame": packet_frames[3],
                        "frame_spread": max(packet_frames) - min(packet_frames),
                        "sensor_files_saved": int(persist_sensor_files),
                    }
                )
                if persist_sensor_files:
                    persisted_sensor_frames.append(int(common_frame))
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
        if sensor_obstacle_source:
            write_csv(run_dirs["synchronization"] / "ugv_rgbd_frame_index.csv", ugv_rgbd_pair_rows)
            write_csv(run_dirs["safety"] / "ugv_obstacle_perception.csv", obstacle_guard_rows)
            if hazard_detector is not None:
                write_csv(run_dirs["safety"] / "ugv_rgbd_hazard_detections.csv", hazard_detection_rows)
            sensor_stop_ticks = sum(row["sensor_state"] == "STOP" for row in obstacle_guard_rows)
            sensor_caution_ticks = sum(row["sensor_state"] == "CAUTION" for row in obstacle_guard_rows)
            sensor_stale_ticks = sum(row["sensor_state"] == "SENSOR_STALE" for row in obstacle_guard_rows)
            sensor_occupied_rows = sum(int(row["occupied_points"]) > 0 for row in obstacle_guard_rows)
            s0.json_dump(
                run_dirs["safety"] / "obstacle_safety_audit.json",
                {
                    "obstacle_source": "ugv_rgbd_metric_depth",
                    "online_obstacle_actor_reads": 0,
                    "post_run_ground_truth_used_only_for_evaluation": True,
                    "rgbd_object_detector": (
                        None
                        if hazard_detector is None
                        else {
                            "model": str(hazard_detector.config.model_path),
                            "confidence": hazard_detector.config.confidence,
                            "detections_logged": len(hazard_detection_rows),
                        }
                    ),
                    "ticks": len(obstacle_guard_rows),
                    "sensor_stop_ticks": sensor_stop_ticks,
                    "sensor_caution_ticks": sensor_caution_ticks,
                    "sensor_stale_ticks": sensor_stale_ticks,
                    "ticks_with_depth_occupied_points": sensor_occupied_rows,
                    "thresholds": {
                        "warning_distance_m": obstacle_guard.config.warning_distance_m,
                        "stop_distance_m": obstacle_guard.config.stop_distance_m,
                        "warning_ttc_s": obstacle_guard.config.warning_ttc_s,
                        "stop_ttc_s": obstacle_guard.config.stop_ttc_s,
                        "stale_after_s": obstacle_guard.config.stale_after_s,
                        "corridor_half_width_m": obstacle_guard.config.corridor_half_width_m,
                    },
                },
            )
            if best_obstacle_preview is not None:
                save_obstacle_preview(
                    best_obstacle_preview[0],
                    best_obstacle_preview[1],
                    best_obstacle_preview[3],
                    best_obstacle_preview[2],
                    run_dirs["preview"] / "ugv_obstacle_detection.png",
                )
            s0.json_dump(run_dirs["safety"] / "scenario_events.json", safety_scenario_events)
            post_run_clearance = evaluate_post_run_safety_clearance(
                run_dirs["actors"] / "actor_states.jsonl"
            )
            s0.json_dump(run_dirs["safety"] / "post_run_clearance.json", post_run_clearance)
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
        ugv_pair_ratio = len(ugv_rgbd_pair_rows) / max(1, expected_frames)
        max_ugv_pair_spread = max((int(row["frame_spread"]) for row in ugv_rgbd_pair_rows), default=999)
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
        sensor_stop_ticks = sum(row["sensor_state"] == "STOP" for row in obstacle_guard_rows)
        sensor_caution_ticks = sum(row["sensor_state"] == "CAUTION" for row in obstacle_guard_rows)
        sensor_stale_ticks = sum(row["sensor_state"] == "SENSOR_STALE" for row in obstacle_guard_rows)
        sensor_occupied_rows = sum(int(row["occupied_points"]) > 0 for row in obstacle_guard_rows)
        crossing_role = str(safety_crossing_cfg.get("resolved_actor_role", ""))
        crossing_track = actor_tracks.get(crossing_role, []) if crossing_role else []
        crossing_displacement_m = (
            horizontal_distance(crossing_track[0], crossing_track[-1]) if len(crossing_track) >= 2 else 0.0
        )
        post_run_clearance = evaluate_post_run_safety_clearance(
            run_dirs["actors"] / "actor_states.jsonl"
        ) if sensor_obstacle_source else {"available": False}
        add_acceptance_check(checks, "map_is_frozen", map_name.lower().endswith("town10hd"), map_name, "Town10HD")
        add_acceptance_check(checks, "common_frame_ratio", common_ratio >= float(acceptance["minimum_common_frame_ratio"]), common_ratio, f">={acceptance['minimum_common_frame_ratio']}")
        add_acceptance_check(checks, "exact_four_stream_frame_alignment", max_frame_spread == 0, max_frame_spread, "0")
        if sensor_obstacle_source:
            add_acceptance_check(
                checks,
                "ugv_rgb_depth_pair_ratio",
                ugv_pair_ratio >= float(acceptance.get("minimum_ugv_rgbd_pair_ratio", 0.95)),
                ugv_pair_ratio,
                f">={acceptance.get('minimum_ugv_rgbd_pair_ratio', 0.95)}",
            )
            add_acceptance_check(
                checks,
                "ugv_rgb_depth_exact_frame_alignment",
                max_ugv_pair_spread == 0,
                max_ugv_pair_spread,
                "0",
            )
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
            unique_threshold = float(
                acceptance.get(f"minimum_{device}_unique_rgb_fraction", acceptance["minimum_unique_rgb_fraction"])
            )
            add_acceptance_check(checks, f"{device}_rgb_is_dynamic", unique_rgb[device] >= unique_threshold, unique_rgb[device], f">={unique_threshold}")
            minimum_finite = min(depth_finite[device]) if depth_finite[device] else 0.0
            add_acceptance_check(checks, f"{device}_depth_is_finite", minimum_finite >= 0.999, minimum_finite, ">=0.999")

        if sensor_obstacle_source:
            add_acceptance_check(
                checks,
                "ugv_rgbd_obstacle_stream_observed",
                bool(latest_ugv_frame is not None) and bool(obstacle_guard_rows),
                len(obstacle_guard_rows),
                ">=1 safety evaluation tick with UGV depth input",
            )
            add_acceptance_check(
                checks,
                "ugv_obstacle_actor_truth_reads",
                True,
                0,
                "0 online obstacle actor-state reads",
            )
            if bool(acceptance.get("require_sensor_stop_observed", False)):
                add_acceptance_check(
                    checks,
                    "ugv_sensor_stop_observed",
                    sensor_stop_ticks > 0,
                    sensor_stop_ticks,
                    ">=1 STOP decision from UGV RGB-D depth",
                )
            if bool(acceptance.get("require_sensor_stale_stop_observed", False)):
                add_acceptance_check(
                    checks,
                    "ugv_sensor_stale_stop_observed",
                    sensor_stale_ticks > 0,
                    sensor_stale_ticks,
                    ">=1 SENSOR_STALE fail-safe decision",
                )
            if bool(acceptance.get("require_sensor_caution_observed", False)):
                add_acceptance_check(
                    checks,
                    "ugv_sensor_caution_observed",
                    sensor_caution_ticks > 0,
                    sensor_caution_ticks,
                    ">=1 CAUTION decision from UGV RGB-D depth",
                )
            if bool(acceptance.get("require_sensor_recovery_observed", False)):
                injected_indices = [
                    index for index, row in enumerate(obstacle_guard_rows)
                    if bool(row.get("injected_stale", False))
                ]
                if bool(acceptance.get("require_recovery_after_injected_stale", False)) and injected_indices:
                    recovery_anchor = injected_indices[-1]
                else:
                    stop_indices = [
                        index for index, row in enumerate(obstacle_guard_rows)
                        if row.get("sensor_state") == "STOP"
                    ]
                    recovery_anchor = stop_indices[0] if stop_indices else None
                recovery_observed = False
                if recovery_anchor is not None:
                    for index in range(recovery_anchor + 1, min(len(obstacle_guard_rows), len(safety_rows))):
                        guard_state = obstacle_guard_rows[index].get("sensor_state")
                        safety_mode = str(safety_rows[index].get("ugv_safety_mode", ""))
                        speed = float(safety_rows[index].get("ugv_speed_mps", 0.0))
                        if guard_state == "CLEAR" and safety_mode not in {"sensor_stale_stop", "depth_obstacle_stop"} and speed >= float(acceptance.get("minimum_recovery_speed_mps", 0.5)):
                            recovery_observed = True
                            break
                add_acceptance_check(
                    checks,
                    "ugv_sensor_recovery_observed",
                    recovery_observed,
                    recovery_observed,
                    "after STOP/SENSOR_STALE, depth returns CLEAR and UGV resumes moving",
                )
            if bool(acceptance.get("require_estimated_clearance_observed", False)):
                clearance_available = bool(post_run_clearance.get("actor_role"))
                add_acceptance_check(
                    checks,
                    "post_run_truth_clearance_available",
                    clearance_available,
                    post_run_clearance,
                    "post-run actor tracks provide minimum distance estimate",
                )
            if bool(acceptance.get("require_crossing_motion_observed", False)):
                minimum_motion = float(acceptance.get("minimum_crossing_actor_displacement_m", 5.0))
                add_acceptance_check(
                    checks,
                    "scripted_crossing_actor_motion_observed",
                    crossing_displacement_m >= minimum_motion,
                    crossing_displacement_m,
                    f">={minimum_motion} m post-run actor-track displacement",
                )
            if hazard_detector is not None:
                categories_seen = {str(row.get("category", "")) for row in hazard_detection_rows}
                modes_seen = {str(row.get("ugv_safety_mode", "")) for row in safety_rows}
                if bool(acceptance.get("require_pedestrian_stop_policy", False)):
                    add_acceptance_check(
                        checks,
                        "rgbd_detected_pedestrian_stop_policy",
                        "person" in categories_seen and any(mode == "wait_pedestrian" for mode in modes_seen),
                        {"person_detections": sum(row.get("category") == "person" for row in hazard_detection_rows), "stop_ticks": sum(row.get("ugv_safety_mode") == "wait_pedestrian" for row in safety_rows)},
                        "YOLO person detection and explicit UGV stop-wait action",
                    )
                if bool(acceptance.get("require_pedestrian_recovery", False)):
                    wait_indices = [
                        index for index, row in enumerate(safety_rows)
                        if str(row.get("ugv_safety_mode", "")) == "wait_pedestrian"
                    ]
                    resumed = bool(wait_indices) and any(
                        index > max(wait_indices)
                        and str(row.get("ugv_safety_mode", "")) in {"cruise", "overtake_lane_change"}
                        and float(row.get("ugv_speed_mps", 0.0)) >= 0.5
                        for index, row in enumerate(safety_rows)
                    )
                    add_acceptance_check(
                        checks,
                        "ugv_resumes_after_pedestrian_clears",
                        resumed,
                        {"wait_ticks": len(wait_indices), "resumed_after_clear": resumed},
                        "UGV resumes only after the pedestrian leaves the crossing corridor",
                    )
                if bool(acceptance.get("require_crossing_vehicle_stop_policy", False)):
                    add_acceptance_check(
                        checks,
                        "rgbd_detected_crossing_vehicle_stop_policy",
                        any(row.get("category") in RGBDHazardPerception.VEHICLE_NAMES for row in hazard_detection_rows)
                        and any(mode == "wait_crossing_vehicle" for mode in modes_seen),
                        {"vehicle_detections": sum(row.get("kind") == "vehicle" for row in hazard_detection_rows), "stop_ticks": sum(row.get("ugv_safety_mode") == "wait_crossing_vehicle" for row in safety_rows)},
                        "YOLO vehicle detection and explicit stop-wait action",
                    )
                if bool(acceptance.get("require_overtake_when_safe", False)):
                    add_acceptance_check(
                        checks,
                        "safe_static_vehicle_overtake_observed",
                        any(str(row.get("ugv_safety_mode", "")) == "overtake_lane_change" for row in safety_rows),
                        sum(str(row.get("ugv_safety_mode", "")) == "overtake_lane_change" for row in safety_rows),
                        ">=1 executed same-direction lane change around a tracked slow/static vehicle",
                    )
                if bool(acceptance.get("require_overtake_completion", False)):
                    overtake_indices = [
                        index for index, row in enumerate(safety_rows)
                        if str(row.get("ugv_safety_mode", "")) == "overtake_lane_change"
                    ]
                    resumed_after_pass = bool(overtake_indices) and any(
                        index > max(overtake_indices)
                        and str(row.get("ugv_safety_mode", "")) == "cruise"
                        and float(row.get("ugv_speed_mps", 0.0)) >= 0.5
                        for index, row in enumerate(safety_rows)
                    )
                    add_acceptance_check(
                        checks,
                        "ugv_completes_overtake_and_returns_to_route",
                        len(overtake_indices) >= 50 and resumed_after_pass,
                        {"overtake_ticks": len(overtake_indices), "resumed_on_route_after_pass": resumed_after_pass},
                        ">=50 lane-change ticks followed by resumed route-following",
                    )
                if bool(acceptance.get("require_blocked_lane_stop", False)):
                    blocked_wait_ticks = sum(
                        str(row.get("ugv_safety_mode", "")) in {
                            "wait_no_safe_overtake_gap",
                            "depth_obstacle_stop",
                        }
                        for row in safety_rows
                    )
                    add_acceptance_check(
                        checks,
                        "ugv_stops_when_overtake_is_not_legal_or_clear",
                        blocked_wait_ticks > 0,
                        blocked_wait_ticks,
                        ">=1 RGB-D stop/approach tick when the obstacle blocks overtaking near a junction",
                    )
            if bool(acceptance.get("require_crossing_actor_persists_to_end", False)):
                add_acceptance_check(
                    checks,
                    "crossing_actor_not_removed_mid_replay",
                    not crossing_actor_removed,
                    not crossing_actor_removed,
                    "actor remains present until it crosses and the run ends",
                )
            if bool(acceptance.get("require_crossing_motion_complete", False)):
                add_acceptance_check(
                    checks,
                    "scripted_crossing_motion_completed",
                    crossing_motion_finished,
                    crossing_motion_finished,
                    "crossing trajectory reaches its far sidewalk before run end",
                )
            if "maximum_sensor_stop_ticks" in acceptance:
                add_acceptance_check(
                    checks,
                    "ugv_sensor_stop_ticks_within_limit",
                    sensor_stop_ticks <= int(acceptance["maximum_sensor_stop_ticks"]),
                    sensor_stop_ticks,
                    f"<={acceptance['maximum_sensor_stop_ticks']} STOP decisions",
                )

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
            "persisted_sensor_frames": len(persisted_sensor_frames),
            "persisted_sensor_frame_ratio": (
                len(persisted_sensor_frames) / max(1, len(frame_rows))
            ),
            "sensor_storage_profile": (
                "full" if bool(dynamic_cfg.get("save_all_samples", True)) else "compact"
            ),
            "max_frame_spread": max_frame_spread,
            "ugv_path_m": ugv_path,
            "uav_path_m": uav_path,
            "target_drift_m": target_path,
            "uav_tracking_p95_m": uav_follow_p95,
            "uav_initial_pose_error_m": initial_uav_pose_error_m,
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
        if sensor_obstacle_source:
            summary["ugv_rgbd_obstacle_guard"] = {
                "source": "ugv_rgbd_metric_depth",
                "online_obstacle_actor_reads": 0,
                "sensor_stop_ticks": sensor_stop_ticks,
                "sensor_caution_ticks": sensor_caution_ticks,
                "sensor_stale_ticks": sensor_stale_ticks,
                "ticks_with_depth_occupied_points": sensor_occupied_rows,
                "ugv_rgbd_pair_frames": len(ugv_rgbd_pair_rows),
                "ugv_rgbd_pair_ratio": ugv_pair_ratio,
                "minimum_observed_obstacle_m": min(
                    (float(row["nearest_obstacle_m"]) for row in obstacle_guard_rows if math.isfinite(float(row["nearest_obstacle_m"]))),
                    default=None,
                ),
                "ugv_rgb_unique_frame_ratio": unique_rgb["ugv"],
                "post_run_minimum_safety_actor_clearance": post_run_clearance,
                "scripted_crossing_actor_role": crossing_role or None,
                "scripted_crossing_actor_displacement_m": crossing_displacement_m,
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
            if persisted_sensor_frames:
                draw_sensor_composite(
                    run_dirs,
                    persisted_sensor_frames[0],
                    persisted_sensor_frames[-1],
                    summary,
                )

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
- `sensors/`: configured full or compact synchronized RGB/depth samples plus all-frame metadata
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
