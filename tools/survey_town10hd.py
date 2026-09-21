from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import carla
import matplotlib
import numpy as np
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle


PREFERRED_TARGETS = (
    "vehicle.mercedes.sprinter",
    "vehicle.carlamotors.carlacola",
    "vehicle.volkswagen.t2",
)


def xyz(location: carla.Location) -> list[float]:
    return [float(location.x), float(location.y), float(location.z)]


def transform_dict(transform: carla.Transform) -> dict[str, list[float]]:
    return {
        "location_xyz": xyz(transform.location),
        "rotation_pyr_degrees": [
            float(transform.rotation.pitch),
            float(transform.rotation.yaw),
            float(transform.rotation.roll),
        ],
    }


def direction(yaw_degrees: float) -> np.ndarray:
    yaw = math.radians(yaw_degrees)
    return np.asarray([math.cos(yaw), math.sin(yaw)], dtype=float)


def next_straight(waypoint: carla.Waypoint, step: float) -> carla.Waypoint | None:
    options = waypoint.next(step)
    if not options:
        return None
    current = direction(waypoint.transform.rotation.yaw)
    ranked = sorted(
        options,
        key=lambda candidate: (
            -float(np.dot(current, direction(candidate.transform.rotation.yaw))),
            int(candidate.road_id),
            int(candidate.lane_id),
        ),
    )
    return ranked[0]


def trace_route(carla_map: carla.Map, spawn: carla.Transform, distance_m: float = 120.0, step_m: float = 2.0) -> list[carla.Waypoint]:
    waypoint = carla_map.get_waypoint(
        spawn.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if waypoint is None:
        return []
    route = [waypoint]
    travelled = 0.0
    while travelled < distance_m:
        waypoint = next_straight(waypoint, step_m)
        if waypoint is None:
            break
        route.append(waypoint)
        travelled += step_m
    return route


def route_distance(points: list[carla.Waypoint]) -> float:
    if len(points) < 2:
        return 0.0
    values = np.asarray([[wp.transform.location.x, wp.transform.location.y] for wp in points], dtype=float)
    return float(np.linalg.norm(np.diff(values, axis=0), axis=1).sum())


def inside_box(location: carla.Location, bounds: tuple[float, float, float, float]) -> bool:
    x_min, x_max, y_min, y_max = bounds
    return x_min <= location.x <= x_max and y_min <= location.y <= y_max


def build_candidates(carla_map: carla.Map, spawns: list[carla.Transform]) -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    for spawn_index, spawn in enumerate(spawns):
        route = trace_route(carla_map, spawn)
        length = route_distance(route)
        if length < 100.0 or len(route) < 46:
            continue
        target_index = min(int(90.0 / 2.0), len(route) - 1)
        target_wp = route[target_index]
        route_xy = np.asarray([[wp.transform.location.x, wp.transform.location.y] for wp in route[: target_index + 1]], dtype=float)
        midpoint = 0.5 * (route_xy[0] + route_xy[-1])
        half = 75.0
        bounds = (midpoint[0] - half, midpoint[0] + half, midpoint[1] - half, midpoint[1] + half)
        if np.ptp(route_xy[:, 0]) > 138.0 or np.ptp(route_xy[:, 1]) > 138.0:
            continue
        local_spawn_ids = [index for index, item in enumerate(spawns) if inside_box(item.location, bounds)]
        junction_positions = [index for index, waypoint in enumerate(route[: target_index + 1]) if waypoint.is_junction]
        useful_junction = any(8 <= index <= target_index - 5 for index in junction_positions)
        score = float(len(local_spawn_ids)) + (30.0 if useful_junction else 0.0) + min(length, 120.0) * 0.02
        candidates.append(
            {
                "score": score,
                "ugv_spawn_point_id": spawn_index,
                "route": route[: target_index + 1],
                "route_length_m": route_distance(route[: target_index + 1]),
                "target_waypoint": target_wp,
                "bounds": bounds,
                "allowed_vehicle_spawn_point_ids": local_spawn_ids,
                "contains_route_junction": useful_junction,
                "junction_route_indices": junction_positions,
            }
        )
    candidates.sort(key=lambda item: (-item["score"], item["ugv_spawn_point_id"]))
    return candidates


def parked_target_transform(waypoint: carla.Waypoint, lateral_offset_m: float = 2.4) -> carla.Transform:
    transform = waypoint.transform
    yaw = math.radians(transform.rotation.yaw)
    right = carla.Vector3D(-math.sin(yaw), math.cos(yaw), 0.0)
    location = transform.location + right * lateral_offset_m + carla.Location(z=0.35)
    return carla.Transform(location, transform.rotation)


def lawnmower(bounds: tuple[float, float, float, float], z: float) -> list[list[float]]:
    x_min, x_max, y_min, y_max = bounds
    margin = 15.0
    rows = np.linspace(y_min + margin, y_max - margin, 5)
    result: list[list[float]] = []
    for row_index, y_value in enumerate(rows):
        endpoints = [x_min + margin, x_max - margin]
        if row_index % 2:
            endpoints.reverse()
        result.extend([[float(x_value), float(y_value), float(z)] for x_value in endpoints])
    return result


def blueprint_inventory(world: carla.World) -> tuple[list[dict[str, Any]], str]:
    rows: list[dict[str, Any]] = []
    for blueprint in sorted(world.get_blueprint_library().filter("vehicle.*"), key=lambda item: item.id):
        colors: list[str] = []
        if blueprint.has_attribute("color"):
            colors = list(blueprint.get_attribute("color").recommended_values)
        wheels = None
        if blueprint.has_attribute("number_of_wheels"):
            try:
                wheels = int(blueprint.get_attribute("number_of_wheels"))
            except (TypeError, ValueError):
                wheels = None
        rows.append({"blueprint": blueprint.id, "number_of_wheels": wheels, "supports_color": bool(colors), "recommended_colors": colors})
    ids = {row["blueprint"] for row in rows if row["supports_color"]}
    selected = next((item for item in PREFERRED_TARGETS if item in ids), None)
    if selected is None:
        keywords = ("sprinter", "van", "carlacola", "t2", "truck")
        selected = next((row["blueprint"] for row in rows if row["supports_color"] and any(word in row["blueprint"] for word in keywords)), None)
    if selected is None:
        raise RuntimeError("No colour-configurable van or truck blueprint is available in this CARLA build")
    return rows, selected


def topology_segments(carla_map: carla.Map) -> list[list[list[float]]]:
    return [
        [xyz(start.transform.location), xyz(end.transform.location)]
        for start, end in carla_map.get_topology()
    ]


def junction_rows(carla_map: carla.Map) -> list[dict[str, Any]]:
    result: dict[int, dict[str, Any]] = {}
    for waypoint in carla_map.generate_waypoints(4.0):
        if not waypoint.is_junction:
            continue
        junction = waypoint.get_junction()
        if junction is None or junction.id in result:
            continue
        box = junction.bounding_box
        result[junction.id] = {
            "junction_id": int(junction.id),
            "center_xyz": xyz(box.location),
            "extent_xyz": xyz(box.extent),
        }
    return sorted(result.values(), key=lambda item: item["junction_id"])


def write_csv(path: Path, rows: list[dict[str, Any]], columns: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def plot_map(
    path: Path,
    topology: list[list[list[float]]],
    spawns: list[carla.Transform],
    junctions: list[dict[str, Any]],
    selected: dict[str, Any],
    uav_route: list[list[float]],
    zoom: bool,
) -> None:
    fig, ax = plt.subplots(figsize=(12, 10), dpi=160)
    fig.patch.set_facecolor("#f5f2eb")
    ax.set_facecolor("#f5f2eb")
    for segment in topology:
        ax.plot([segment[0][0], segment[1][0]], [segment[0][1], segment[1][1]], color="#a7adb4", linewidth=0.75, zorder=1)
    ax.scatter([item.location.x for item in spawns], [item.location.y for item in spawns], s=7, color="#718096", alpha=0.5, zorder=2)
    if junctions:
        ax.scatter([item["center_xyz"][0] for item in junctions], [item["center_xyz"][1] for item in junctions], marker="x", s=28, color="#2f855a", linewidths=1.0, zorder=3)
    route_xy = np.asarray([[wp.transform.location.x, wp.transform.location.y] for wp in selected["route"]], dtype=float)
    ax.plot(route_xy[:, 0], route_xy[:, 1], color="#1769aa", linewidth=3.0, label="UGV reference route", zorder=5)
    target = selected["target_transform"].location
    start = spawns[selected["ugv_spawn_point_id"]].location
    ax.scatter([start.x], [start.y], marker="s", s=90, color="#1769aa", edgecolor="white", linewidth=1.2, zorder=7)
    ax.scatter([target.x], [target.y], marker="*", s=230, color="#d32f2f", edgecolor="white", linewidth=0.9, zorder=8)
    uav_xy = np.asarray([[item[0], item[1]] for item in uav_route], dtype=float)
    ax.plot(uav_xy[:, 0], uav_xy[:, 1], color="#7b1fa2", linewidth=2.0, linestyle="--", label="UAV lawnmower route", zorder=4)
    ax.scatter([uav_xy[0, 0]], [uav_xy[0, 1]], marker="D", s=80, color="#7b1fa2", edgecolor="white", zorder=7)
    x_min, x_max, y_min, y_max = selected["bounds"]
    ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=True, facecolor="#3182ce", alpha=0.06, edgecolor="#2457a6", linewidth=2.0, linestyle="--", zorder=0))
    if zoom:
        ax.set_xlim(x_min - 15.0, x_max + 15.0)
        ax.set_ylim(y_min - 15.0, y_max + 15.0)
        title = "Town10HD Zone A — fixed city-inspection region"
    else:
        title = "Town10HD survey — selected Zone A"
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("CARLA world X (m)")
    ax.set_ylabel("CARLA world Y (m)")
    ax.set_title(title, loc="left", fontsize=16, fontweight="bold")
    ax.grid(color="white", linewidth=0.8, alpha=0.8)
    legend = [
        Line2D([0], [0], color="#a7adb4", lw=2, label="Road topology / lane links"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#718096", label="Vehicle spawn point", markersize=6),
        Line2D([0], [0], marker="x", color="#2f855a", label="Junction centre", markersize=7),
        Line2D([0], [0], color="#1769aa", lw=3, label="UGV reference route"),
        Line2D([0], [0], color="#7b1fa2", lw=2, linestyle="--", label="UAV search route"),
        Line2D([0], [0], marker="*", color="none", markerfacecolor="#d32f2f", label="Target vehicle", markersize=12),
    ]
    ax.legend(handles=legend, loc="best", framealpha=0.96, fontsize=9)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Survey Town10HD and propose a reproducible city-inspection region")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--seed", type=int, default=1001)
    args = parser.parse_args()

    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()
    snapshot = world.get_snapshot()
    carla_map = world.get_map()
    actual_map = carla_map.name.split("/")[-1]
    if actual_map != "Town10HD":
        raise RuntimeError(f"Expected Town10HD, got {carla_map.name}")

    spawns = carla_map.get_spawn_points()
    topology = topology_segments(carla_map)
    junctions = junction_rows(carla_map)
    candidates = build_candidates(carla_map, spawns)
    if not candidates:
        raise RuntimeError("No 150 m candidate region with a 100 m drivable reference route was found")
    selected = candidates[0]
    selected["target_lane_transform"] = selected["target_waypoint"].transform
    selected["target_transform"] = parked_target_transform(selected["target_waypoint"])
    base_z = max(item.transform.location.z for item in selected["route"])
    uav_route = lawnmower(selected["bounds"], base_z + 40.0)
    inventory, target_blueprint = blueprint_inventory(world)

    spawn_rows = []
    for index, transform in enumerate(spawns):
        spawn_rows.append({
            "spawn_point_id": index,
            "x": float(transform.location.x),
            "y": float(transform.location.y),
            "z": float(transform.location.z),
            "pitch": float(transform.rotation.pitch),
            "yaw": float(transform.rotation.yaw),
            "roll": float(transform.rotation.roll),
            "inside_selected_zone": inside_box(transform.location, selected["bounds"]),
        })
    write_csv(
        output_root / "town10hd_spawn_points.csv",
        spawn_rows,
        ["spawn_point_id", "x", "y", "z", "pitch", "yaw", "roll", "inside_selected_zone"],
    )
    junction_csv = []
    for item in junctions:
        junction_csv.append({
            "junction_id": item["junction_id"],
            "x": item["center_xyz"][0],
            "y": item["center_xyz"][1],
            "z": item["center_xyz"][2],
            "extent_x": item["extent_xyz"][0],
            "extent_y": item["extent_xyz"][1],
            "extent_z": item["extent_xyz"][2],
        })
    write_csv(output_root / "town10hd_junctions.csv", junction_csv, ["junction_id", "x", "y", "z", "extent_x", "extent_y", "extent_z"])
    (output_root / "town10hd_topology.json").write_text(json.dumps({"map": carla_map.name, "segments": topology}, ensure_ascii=False, indent=2), encoding="utf-8")
    (output_root / "vehicle_blueprints.json").write_text(json.dumps({"selected_target_blueprint": target_blueprint, "vehicles": inventory}, ensure_ascii=False, indent=2), encoding="utf-8")

    x_min, x_max, y_min, y_max = selected["bounds"]
    region_payload = {
        "region": {
            "region_id": "town10hd_zone_a",
            "selection_status": "survey_selected",
            "map": "Town10HD",
            "survey_seed": args.seed,
            "boundary": {
                "type": "polygon",
                "points": [
                    [float(x_min), float(y_min)],
                    [float(x_max), float(y_min)],
                    [float(x_max), float(y_max)],
                    [float(x_min), float(y_max)],
                ],
            },
            "ugv_spawn_point_id": int(selected["ugv_spawn_point_id"]),
            "target_transform": transform_dict(selected["target_transform"]),
            "target_lane_transform": transform_dict(selected["target_lane_transform"]),
            "planned_ugv_route": [xyz(item.transform.location) for item in selected["route"]],
            "planned_ugv_route_length_m": float(selected["route_length_m"]),
            "contains_route_junction": bool(selected["contains_route_junction"]),
            "allowed_vehicle_spawn_point_ids": [int(item) for item in selected["allowed_vehicle_spawn_point_ids"]],
            "uav": {
                "altitude_m": 40.0,
                "initial_position_xyz": uav_route[0],
                "search_pattern": "lawnmower",
                "speed_mps": 5.0,
                "waypoints_xyz": uav_route,
            },
            "recommended_target_blueprint": target_blueprint,
            "survey_artifacts": {
                "root": str(output_root.resolve()),
                "full_map": str((output_root / "town10hd_overview.png").resolve()),
                "zone_map": str((output_root / "town10hd_zone_a.png").resolve()),
                "spawn_points_csv": str((output_root / "town10hd_spawn_points.csv").resolve()),
                "junctions_csv": str((output_root / "town10hd_junctions.csv").resolve()),
                "blueprints_json": str((output_root / "vehicle_blueprints.json").resolve()),
            },
        }
    }
    (output_root / "proposed_town10hd_zone_a.yaml").write_text(
        yaml.safe_dump(region_payload, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )

    selected_for_plot = dict(selected)
    plot_map(output_root / "town10hd_overview.png", topology, spawns, junctions, selected_for_plot, uav_route, zoom=False)
    plot_map(output_root / "town10hd_zone_a.png", topology, spawns, junctions, selected_for_plot, uav_route, zoom=True)
    summary = {
        "status": "PASS",
        "surveyed_at_unix": time.time(),
        "carla_frame": int(snapshot.frame),
        "map": carla_map.name,
        "client_version": client.get_client_version(),
        "server_version": client.get_server_version(),
        "spawn_point_count": len(spawns),
        "junction_count": len(junctions),
        "candidate_count": len(candidates),
        "selected_region": "town10hd_zone_a",
        "selected_ugv_spawn_point_id": int(selected["ugv_spawn_point_id"]),
        "planned_route_length_m": float(selected["route_length_m"]),
        "allowed_vehicle_spawn_points": len(selected["allowed_vehicle_spawn_point_ids"]),
        "contains_route_junction": bool(selected["contains_route_junction"]),
        "selected_target_blueprint": target_blueprint,
        "proposed_region_config": str((output_root / "proposed_town10hd_zone_a.yaml").resolve()),
    }
    (output_root / "survey_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
