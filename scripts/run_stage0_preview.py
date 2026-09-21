from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import queue
import random
import shutil
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

import airsim
import carla
import matplotlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon

from scenario.configuration import resolve_experiment, validate_schema, write_yaml
from sensors.depth import bgra_to_rgb, carla_depth_to_metres, colourise_depth


def json_dump(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def now_utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def xyz(location: carla.Location) -> list[float]:
    return [float(location.x), float(location.y), float(location.z)]


def rotation_pyr(rotation: carla.Rotation) -> list[float]:
    return [float(rotation.pitch), float(rotation.yaw), float(rotation.roll)]


def dict_to_transform(payload: dict[str, Any]) -> carla.Transform:
    location = payload["location_xyz"]
    rotation = payload["rotation_pyr_degrees"]
    return carla.Transform(
        carla.Location(x=float(location[0]), y=float(location[1]), z=float(location[2])),
        carla.Rotation(pitch=float(rotation[0]), yaw=float(rotation[1]), roll=float(rotation[2])),
    )


def role_name(actor: carla.Actor) -> str:
    return str(actor.attributes.get("role_name", ""))


def inside_polygon_box(point: list[float], polygon: list[list[float]]) -> bool:
    xs = [item[0] for item in polygon]
    ys = [item[1] for item in polygon]
    return min(xs) <= point[0] <= max(xs) and min(ys) <= point[1] <= max(ys)


def point_to_polyline_distance(point_xy: list[float], polyline_xyz: list[list[float]]) -> float:
    point = np.asarray(point_xy[:2], dtype=float)
    polyline = np.asarray([item[:2] for item in polyline_xyz], dtype=float)
    best = float("inf")
    for start, end in zip(polyline[:-1], polyline[1:]):
        segment = end - start
        denominator = float(np.dot(segment, segment))
        ratio = float(np.clip(np.dot(point - start, segment) / denominator, 0.0, 1.0)) if denominator > 1e-9 else 0.0
        best = min(best, float(np.linalg.norm(point - (start + ratio * segment))))
    return best


def add_check(checks: list[dict[str, Any]], name: str, passed: bool, detail: str, severity: str = "error") -> None:
    checks.append({"name": name, "passed": bool(passed), "severity": severity, "detail": detail})


def preflight_checks(config: dict[str, Any], schema_path: Path) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    schema_errors = validate_schema(config, schema_path)
    add_check(checks, "schema", not schema_errors, "valid" if not schema_errors else json.dumps(schema_errors, ensure_ascii=False))
    simulation = config.get("simulation", {})
    region = config.get("region", {})
    task = config.get("task", {})
    sensors = config.get("sensors", {})
    output = config.get("output", {})
    add_check(checks, "map_requested", simulation.get("map") == "Town10HD", str(simulation.get("map")))
    add_check(checks, "region_selected", region.get("selection_status") in {"survey_selected", "frozen"}, str(region.get("selection_status")))
    polygon = region.get("boundary", {}).get("points", [])
    add_check(checks, "region_boundary", len(polygon) == 4, f"{len(polygon)} points")
    add_check(checks, "ugv_spawn_defined", isinstance(region.get("ugv_spawn_point_id"), int), str(region.get("ugv_spawn_point_id")))
    add_check(checks, "target_transform_defined", isinstance(region.get("target_transform"), dict), str(region.get("target_transform")))
    route = region.get("planned_ugv_route", [])
    route_length = float(region.get("planned_ugv_route_length_m", 0.0) or 0.0)
    add_check(checks, "ugv_route_defined", len(route) >= 20 and route_length >= 70.0, f"{len(route)} points, {route_length:.1f} m")
    uav_waypoints = region.get("uav", {}).get("waypoints_xyz", [])
    add_check(checks, "uav_route_defined", len(uav_waypoints) >= 6, f"{len(uav_waypoints)} waypoints")
    if len(polygon) == 4 and uav_waypoints:
        add_check(checks, "uav_route_inside_region", all(inside_polygon_box(item, polygon) for item in uav_waypoints), "all waypoints inside Zone A")
    target_transform = region.get("target_transform")
    if isinstance(target_transform, dict) and len(uav_waypoints) >= 2:
        target_xy = target_transform["location_xyz"][:2]
        minimum_distance = point_to_polyline_distance(target_xy, uav_waypoints)
        footprint_radius = float(region.get("uav", {}).get("altitude_m", 40.0)) * math.tan(math.radians(float(sensors.get("fov_degrees", 90.0))) / 2.0)
        add_check(checks, "target_covered_by_uav_route", minimum_distance <= footprint_radius * 0.9, f"nearest path distance {minimum_distance:.1f} m; conservative half-footprint {footprint_radius * 0.9:.1f} m")
    instruction = str(task.get("instruction", ""))
    goal = task.get("parsed_goal", {})
    consistent = "红" in instruction and goal.get("attributes", {}).get("color") == "red" and goal.get("subcategory") == "van"
    add_check(checks, "instruction_target_consistency", consistent, instruction)
    streams = sensors.get("streams", {})
    required_streams = {"uav_rgb", "uav_depth", "ugv_rgb", "ugv_depth"}
    add_check(checks, "sensor_set", required_streams.issubset(streams), ", ".join(sorted(streams)))
    add_check(checks, "sensor_resolution", sensors.get("width") == 640 and sensors.get("height") == 360, f"{sensors.get('width')}x{sensors.get('height')}")
    package_root = Path(str(simulation.get("package_root", "")))
    add_check(checks, "carla_air_package", (package_root / "CarlaAir.ps1").exists(), str(package_root))
    output_root = Path(str(output.get("root", "")))
    writable = output_root.exists() and output_root.is_dir()
    add_check(checks, "output_root", writable, str(output_root))
    return checks


def configure_blueprint(blueprint: carla.ActorBlueprint, role: str, color: str | None = None) -> carla.ActorBlueprint:
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", role)
    if color is not None and blueprint.has_attribute("color"):
        blueprint.set_attribute("color", color)
    return blueprint


def resolve_target_blueprint(library: carla.BlueprintLibrary, config: dict[str, Any]) -> tuple[carla.ActorBlueprint, str]:
    target = config["actors"]["target_vehicle"]
    options = []
    if target.get("resolved_blueprint"):
        options.append(target["resolved_blueprint"])
    options.extend(target.get("preferred_blueprints", []))
    available = {item.id for item in library.filter("vehicle.*")}
    for blueprint_id in options:
        if blueprint_id in available:
            blueprint = library.find(blueprint_id)
            if blueprint.has_attribute("color"):
                return blueprint, blueprint_id
    raise RuntimeError(f"No colour-configurable target blueprint found from {options}")


def resolve_sedan_blueprints(library: carla.BlueprintLibrary, excluded: set[str]) -> list[str]:
    preferred = [
        "vehicle.tesla.model3",
        "vehicle.audi.tt",
        "vehicle.lincoln.mkz_2020",
        "vehicle.mercedes.coupe_2020",
    ]
    available = {item.id: item for item in library.filter("vehicle.*")}
    selected = [item for item in preferred if item in available and item not in excluded and available[item].has_attribute("color")]
    if len(selected) >= 2:
        return selected
    for item in sorted(available):
        bp = available[item]
        if item in excluded or not bp.has_attribute("color"):
            continue
        if bp.has_attribute("number_of_wheels"):
            try:
                if int(bp.get_attribute("number_of_wheels")) != 4:
                    continue
            except (TypeError, ValueError):
                continue
        selected.append(item)
        if len(selected) >= 4:
            break
    return selected


def spawn_vehicle(
    world: carla.World,
    library: carla.BlueprintLibrary,
    blueprint_id: str,
    transform: carla.Transform,
    role: str,
    color: str | None,
) -> carla.Vehicle | None:
    blueprint = configure_blueprint(library.find(blueprint_id), role, color)
    actor = world.try_spawn_actor(blueprint, transform)
    return actor


def push(queue_object: queue.Queue, data: Any) -> None:
    try:
        queue_object.put_nowait(data)
    except queue.Full:
        try:
            queue_object.get_nowait()
        except queue.Empty:
            pass
        queue_object.put_nowait(data)


def drain_latest(queue_object: queue.Queue) -> Any | None:
    result = None
    while True:
        try:
            result = queue_object.get_nowait()
        except queue.Empty:
            return result


def drain_all(queue_object: queue.Queue) -> list[Any]:
    result = []
    while True:
        try:
            result.append(queue_object.get_nowait())
        except queue.Empty:
            return result


def sensor_blueprint(library: carla.BlueprintLibrary, sensor_type: str, sensors: dict[str, Any]) -> carla.ActorBlueprint:
    blueprint = library.find(sensor_type)
    blueprint.set_attribute("image_size_x", str(sensors["width"]))
    blueprint.set_attribute("image_size_y", str(sensors["height"]))
    blueprint.set_attribute("fov", str(sensors["fov_degrees"]))
    blueprint.set_attribute("sensor_tick", str(1.0 / float(sensors["frequency_hz"])))
    return blueprint


def actor_record(actor: carla.Actor, role: str, category: str, attributes: dict[str, Any] | None = None) -> dict[str, Any]:
    transform = actor.get_transform()
    record = {
        "actor_id": int(actor.id),
        "role": role,
        "category": category,
        "type_id": actor.type_id,
        "location_xyz": xyz(transform.location),
        "rotation_pyr_degrees": rotation_pyr(transform.rotation),
    }
    if attributes:
        record["attributes"] = attributes
    return record


def plot_scene_map(
    path: Path,
    world: carla.World,
    config: dict[str, Any],
    actor_records: list[dict[str, Any]],
) -> None:
    region = config["region"]
    polygon = np.asarray(region["boundary"]["points"], dtype=float)
    route = np.asarray(region["planned_ugv_route"], dtype=float)
    uav_route = np.asarray(region["uav"]["waypoints_xyz"], dtype=float)
    x_min, x_max = polygon[:, 0].min(), polygon[:, 0].max()
    y_min, y_max = polygon[:, 1].min(), polygon[:, 1].max()
    fig, ax = plt.subplots(figsize=(10, 9), dpi=170)
    fig.patch.set_facecolor("#f5f2eb")
    ax.set_facecolor("#f5f2eb")
    for start, end in world.get_map().get_topology():
        x_values = [start.transform.location.x, end.transform.location.x]
        y_values = [start.transform.location.y, end.transform.location.y]
        if max(x_values) < x_min - 20 or min(x_values) > x_max + 20 or max(y_values) < y_min - 20 or min(y_values) > y_max + 20:
            continue
        ax.plot(x_values, y_values, color="#91979f", linewidth=1.1, zorder=1)
    ax.add_patch(Polygon(polygon, closed=True, facecolor="#3182ce", alpha=0.07, edgecolor="#2457a6", linewidth=2.2, linestyle="--", zorder=0))
    ax.plot(route[:, 0], route[:, 1], color="#1769aa", linewidth=3.2, zorder=4)
    ax.plot(uav_route[:, 0], uav_route[:, 1], color="#7b1fa2", linestyle="--", linewidth=2.2, zorder=3)
    style = {
        "inspection_ugv": ("s", "#1769aa", 110),
        "inspection_target": ("*", "#d32f2f", 260),
        "same_category_wrong_color": ("^", "#f59e0b", 80),
        "same_color_wrong_category": ("^", "#ef4444", 80),
        "random_vehicle": ("^", "#6b7280", 60),
        "pedestrian": ("o", "#22a06b", 42),
        "inspection_uav": ("D", "#7b1fa2", 100),
    }
    for record in actor_records:
        role = record["role"]
        if role not in style:
            continue
        marker, color, size = style[role]
        x_value, y_value = record["location_xyz"][:2]
        ax.scatter([x_value], [y_value], marker=marker, s=size, color=color, edgecolor="white", linewidth=0.8, zorder=7)
    ax.set_xlim(x_min - 12, x_max + 12)
    ax.set_ylim(y_min - 12, y_max + 12)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("CARLA world X (m)")
    ax.set_ylabel("CARLA world Y (m)")
    ax.set_title("CI-E0 Town10HD Zone A — scene preview", loc="left", fontsize=15, fontweight="bold")
    ax.grid(color="white", linewidth=0.8)
    legend = [
        Line2D([0], [0], color="#91979f", lw=2, label="Road topology / lane links"),
        Line2D([0], [0], color="#1769aa", lw=3, label="UGV reference route"),
        Line2D([0], [0], color="#7b1fa2", lw=2, linestyle="--", label="UAV search route"),
        Line2D([0], [0], marker="s", color="none", markerfacecolor="#1769aa", label="UGV start", markersize=8),
        Line2D([0], [0], marker="*", color="none", markerfacecolor="#d32f2f", label="Red van target", markersize=13),
        Line2D([0], [0], marker="^", color="none", markerfacecolor="#f59e0b", label="Vehicle distractor", markersize=8),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#22a06b", label="Pedestrian", markersize=7),
        Line2D([0], [0], marker="D", color="none", markerfacecolor="#7b1fa2", label="UAV", markersize=7),
    ]
    ax.legend(handles=legend, loc="lower right", framealpha=0.96, fontsize=8)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def labelled_panel(image: Image.Image, label: str, size: tuple[int, int]) -> Image.Image:
    panel = Image.new("RGB", (size[0], size[1] + 34), "white")
    fitted = ImageOps.contain(image.convert("RGB"), size)
    x_value = (size[0] - fitted.width) // 2
    y_value = 34 + (size[1] - fitted.height) // 2
    panel.paste(fitted, (x_value, y_value))
    ImageDraw.Draw(panel).text((12, 7), label, fill="#172033", font=load_font(18, bold=True))
    return panel


def load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        Path("C:/Windows/Fonts/msyhbd.ttc" if bold else "C:/Windows/Fonts/msyh.ttc"),
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size=size)
    return ImageFont.load_default()


def build_composite(run_dir: Path, manifest: dict[str, Any]) -> None:
    canvas = Image.new("RGB", (1700, 1160), "#f3f5f8")
    draw = ImageDraw.Draw(canvas)
    draw.text((45, 22), "CI-E0 CITY INSPECTION SCENE ACCEPTANCE", fill="#172033", font=load_font(28, bold=True))
    draw.text((45, 58), f"{manifest['run_id']} | {manifest['map']} | synchronized RGB + depth", fill="#506078", font=load_font(16))
    map_image = Image.open(run_dir / "preview" / "map_overview.png")
    canvas.paste(labelled_panel(map_image, "Zone A: roads, routes and actors", (790, 940)), (35, 95))
    panels = [
        ("uav_rgb.png", "UAV RGB — world nadir"),
        ("uav_depth_preview.png", "UAV depth — colourised 0–100 m"),
        ("ugv_rgb.png", "UGV RGB — forward view"),
        ("ugv_depth_preview.png", "UGV depth — colourised 0–100 m"),
    ]
    for index, (filename, label) in enumerate(panels):
        image = Image.open(run_dir / "preview" / filename)
        panel = labelled_panel(image, label, (390, 250))
        x_value = 850 + (index % 2) * 405
        y_value = 125 + (index // 2) * 310
        canvas.paste(panel, (x_value, y_value))
    info_y = 785
    draw.rounded_rectangle((850, info_y, 1655, 1055), radius=18, fill="white", outline="#cbd5e1", width=2)
    details = [
        f"指令：{manifest['instruction']}",
        f"Target: {manifest['target']['type_id']} | red | actor {manifest['target']['actor_id']}",
        f"Distractors: {manifest['counts']['vehicle_distractors']} vehicles + {manifest['counts']['pedestrians']} pedestrians",
        f"UGV route: {manifest['ugv_reference_route_length_m']:.1f} m",
        f"UAV: fixed {manifest['uav']['configured_altitude_m']:.0f} m, lawnmower route",
        f"Common sensor frame: {manifest['sensor_alignment']['common_frame']} (spread 0)",
        "Stage boundary: scene validation only; no detector or GOC policy in CI-E0.",
    ]
    for index, line in enumerate(details):
        draw.text((880, info_y + 22 + index * 34), line, fill="#26364d", font=load_font(16))
    output = run_dir / "preview" / "scene_composite.png"
    canvas.save(output)


def save_sensor_previews(run_dir: Path, packets: dict[str, Any], sensors: dict[str, Any]) -> None:
    preview = run_dir / "preview"
    preview.mkdir(parents=True, exist_ok=True)
    for name in ("uav_rgb", "ugv_rgb"):
        packet = packets[name]
        image = Image.fromarray(bgra_to_rgb(packet.raw_data, packet.width, packet.height), mode="RGB")
        image.save(preview / f"{name}.png")
    for name in ("uav_depth", "ugv_depth"):
        packet = packets[name]
        encoded = Image.fromarray(bgra_to_rgb(packet.raw_data, packet.width, packet.height), mode="RGB")
        encoded.save(preview / f"{name}_raw.png")
        depth = carla_depth_to_metres(packet.raw_data, packet.width, packet.height, float(sensors.get("depth_max_m", 1000.0)))
        np.save(preview / f"{name}_metres.npy", depth.astype(np.float32))
        colourise_depth(depth, display_max_m=100.0).save(preview / f"{name}_preview.png")


def write_actor_csv(path: Path, records: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        columns = ["actor_id", "role", "category", "type_id", "x", "y", "z", "pitch", "yaw", "roll", "attributes_json"]
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for item in records:
            writer.writerow({
                "actor_id": item["actor_id"],
                "role": item["role"],
                "category": item["category"],
                "type_id": item["type_id"],
                "x": item["location_xyz"][0],
                "y": item["location_xyz"][1],
                "z": item["location_xyz"][2],
                "pitch": item["rotation_pyr_degrees"][0],
                "yaw": item["rotation_pyr_degrees"][1],
                "roll": item["rotation_pyr_degrees"][2],
                "attributes_json": json.dumps(item.get("attributes", {}), ensure_ascii=False),
            })


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the CI-E0 five-second Town10HD scene preview")
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    config = resolve_experiment(config_path)
    checks = preflight_checks(config, PROJECT_ROOT / "configs" / "schemas" / "experiment_schema.json")
    if any(not item["passed"] and item["severity"] == "error" for item in checks):
        raise RuntimeError("Preflight validation failed: " + json.dumps(checks, ensure_ascii=False))

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_id = f"{config['experiment_id']}_{timestamp}"
    run_dir = Path(config["output"]["root"]) / run_id
    if run_dir.exists():
        raise FileExistsError(f"Refusing to overwrite existing run: {run_dir}")
    for subdir in ("preview", "logs", "config_snapshot"):
        (run_dir / subdir).mkdir(parents=True, exist_ok=False)

    write_yaml(run_dir / "resolved_config.yaml", config)
    for source_name in config.get("_included_configs", []) + [config.get("_source_config")]:
        if source_name:
            source = Path(source_name)
            shutil.copy2(source, run_dir / "config_snapshot" / source.name)

    simulation = config["simulation"]
    actors_config = config["actors"]
    region = config["region"]
    sensors_config = config["sensors"]
    random_generator = random.Random(int(config["random_seed"]))
    created_actors: list[carla.Actor] = []
    actor_records: list[dict[str, Any]] = []
    queues: dict[str, queue.Queue] = {}
    sensor_actors: dict[str, carla.Sensor] = {}
    client = carla.Client(simulation["host"], int(simulation["carla_port"]))
    client.set_timeout(20.0)
    world = None
    original_settings = None
    traffic_manager = None
    airsim_client = None
    manifest: dict[str, Any] = {
        "run_id": run_id,
        "status": "RUNNING",
        "started_utc": now_utc(),
        "instruction": config["task"]["instruction"],
        "config_sha256": hashlib.sha256((run_dir / "resolved_config.yaml").read_bytes()).hexdigest(),
    }
    try:
        world = client.get_world()
        carla_map = world.get_map()
        actual_map = carla_map.name.split("/")[-1]
        add_check(checks, "actual_map", actual_map == simulation["map"], carla_map.name)
        if actual_map != simulation["map"]:
            raise RuntimeError(f"Expected {simulation['map']}, got {carla_map.name}")
        library = world.get_blueprint_library()
        spawns = carla_map.get_spawn_points()
        spawn_id = int(region["ugv_spawn_point_id"])
        add_check(checks, "ugv_spawn_in_range", 0 <= spawn_id < len(spawns), f"{spawn_id}/{len(spawns)}")
        if not (0 <= spawn_id < len(spawns)):
            raise RuntimeError("UGV spawn point is outside the Town10HD spawn list")

        target_blueprint, target_blueprint_id = resolve_target_blueprint(library, config)
        add_check(checks, "target_blueprint", target_blueprint.has_attribute("color"), target_blueprint_id)
        target_color = actors_config["target_vehicle"]["color_rgb"]
        add_check(checks, "target_color", target_color == "255,0,0", target_color)

        ugv_config = actors_config["ugv"]
        ugv = spawn_vehicle(world, library, ugv_config["blueprint"], spawns[spawn_id], ugv_config["role_name"], None)
        if ugv is None:
            raise RuntimeError(f"Failed to spawn UGV at spawn point {spawn_id}")
        created_actors.append(ugv)
        ugv.apply_control(carla.VehicleControl(hand_brake=True))
        actor_records.append(actor_record(ugv, "inspection_ugv", "ugv", {"spawn_point_id": spawn_id}))

        target_transforms = [dict_to_transform(region["target_transform"]), dict_to_transform(region["target_lane_transform"])]
        target = None
        placement = None
        for candidate_placement, transform in zip(("roadside", "lane_fallback"), target_transforms):
            target = spawn_vehicle(world, library, target_blueprint_id, transform, actors_config["target_vehicle"]["role_name"], target_color)
            if target is not None:
                placement = candidate_placement
                break
        if target is None:
            raise RuntimeError("Failed to spawn the configured target vehicle at both roadside and lane transforms")
        created_actors.append(target)
        try:
            target.apply_control(carla.VehicleControl(hand_brake=True))
            target.set_simulate_physics(False)
        except RuntimeError:
            pass
        actor_records.append(actor_record(target, "inspection_target", "target_vehicle", {"color_rgb": target_color, "placement": placement}))
        add_check(checks, "target_roadside_spawn", placement == "roadside", placement, severity="warning")

        excluded_positions = [ugv.get_location(), target.get_location()]
        allowed_ids = list(region["allowed_vehicle_spawn_point_ids"])
        random_generator.shuffle(allowed_ids)
        usable_spawns = []
        for index in allowed_ids:
            if not 0 <= int(index) < len(spawns):
                continue
            candidate = spawns[int(index)]
            if min(candidate.location.distance(item) for item in excluded_positions) < 12.0:
                continue
            usable_spawns.append(candidate)
        needed_vehicles = (
            int(actors_config["distractors"]["same_category_wrong_color"])
            + int(actors_config["distractors"]["same_color_wrong_category"])
            + int(actors_config["distractors"]["random_vehicles"])
        )
        if len(usable_spawns) < needed_vehicles:
            raise RuntimeError(f"Zone A has only {len(usable_spawns)} usable vehicle spawns; {needed_vehicles} required")

        distractor_vehicles: list[carla.Vehicle] = []
        spawn_cursor = 0
        wrong_color_count = int(actors_config["distractors"]["same_category_wrong_color"])
        for item_index in range(wrong_color_count):
            actor = spawn_vehicle(world, library, target_blueprint_id, usable_spawns[spawn_cursor], "same_category_wrong_color", "255,255,255")
            spawn_cursor += 1
            if actor is not None:
                created_actors.append(actor)
                actor.apply_control(carla.VehicleControl(hand_brake=True))
                distractor_vehicles.append(actor)
                actor_records.append(actor_record(actor, "same_category_wrong_color", "vehicle_distractor", {"color_rgb": "255,255,255"}))

        sedan_ids = resolve_sedan_blueprints(library, {target_blueprint_id})
        same_color_count = int(actors_config["distractors"]["same_color_wrong_category"])
        for item_index in range(same_color_count):
            blueprint_id = sedan_ids[item_index % len(sedan_ids)]
            actor = spawn_vehicle(world, library, blueprint_id, usable_spawns[spawn_cursor], "same_color_wrong_category", "255,0,0")
            spawn_cursor += 1
            if actor is not None:
                created_actors.append(actor)
                actor.apply_control(carla.VehicleControl(hand_brake=True))
                distractor_vehicles.append(actor)
                actor_records.append(actor_record(actor, "same_color_wrong_category", "vehicle_distractor", {"color_rgb": "255,0,0"}))

        random_blueprints = []
        for blueprint in library.filter("vehicle.*"):
            if blueprint.id in {target_blueprint_id, ugv_config["blueprint"]}:
                continue
            if blueprint.has_attribute("number_of_wheels"):
                try:
                    if int(blueprint.get_attribute("number_of_wheels")) != 4:
                        continue
                except (TypeError, ValueError):
                    continue
            random_blueprints.append(blueprint.id)
        random_blueprints.sort()
        random_count = int(actors_config["distractors"]["random_vehicles"])
        for item_index in range(random_count):
            blueprint_id = random_generator.choice(random_blueprints)
            actor = spawn_vehicle(world, library, blueprint_id, usable_spawns[spawn_cursor], "random_vehicle", None)
            spawn_cursor += 1
            if actor is not None:
                created_actors.append(actor)
                actor.apply_control(carla.VehicleControl(hand_brake=True))
                distractor_vehicles.append(actor)
                actor_records.append(actor_record(actor, "random_vehicle", "vehicle_distractor"))
        add_check(checks, "vehicle_distractor_count", len(distractor_vehicles) == needed_vehicles, f"{len(distractor_vehicles)}/{needed_vehicles}")

        polygon = region["boundary"]["points"]
        walker_blueprints = list(library.filter("walker.pedestrian.*"))
        walker_count_requested = int(actors_config["distractors"]["pedestrians"])
        walkers: list[carla.Walker] = []
        walker_destinations: list[carla.Location] = []
        attempts = 0
        while len(walkers) < walker_count_requested and attempts < 500:
            attempts += 1
            location = world.get_random_location_from_navigation()
            if location is None or not inside_polygon_box(xyz(location), polygon):
                continue
            if min(location.distance(item) for item in excluded_positions) < 8.0:
                continue
            blueprint = random_generator.choice(walker_blueprints)
            if blueprint.has_attribute("is_invincible"):
                blueprint.set_attribute("is_invincible", "false")
            walker = world.try_spawn_actor(blueprint, carla.Transform(location + carla.Location(z=0.4)))
            if walker is None:
                continue
            created_actors.append(walker)
            walkers.append(walker)
            destination = world.get_random_location_from_navigation()
            walker_destinations.append(destination if destination is not None else location)
            actor_records.append(actor_record(walker, "pedestrian", "pedestrian"))
        add_check(checks, "pedestrian_count", len(walkers) >= 4, f"{len(walkers)}/{walker_count_requested}")

        drone_actors = [item for item in world.get_actors() if "drone" in item.type_id.lower() or "airsim" in item.type_id.lower()]
        add_check(checks, "airsim_drone_actor", len(drone_actors) == 1, f"{len(drone_actors)} actor(s)")
        if len(drone_actors) != 1:
            raise RuntimeError(f"Expected exactly one AirSim drone actor, found {len(drone_actors)}")
        drone_actor = drone_actors[0]
        airsim_client = airsim.MultirotorClient(ip=simulation["host"], port=int(simulation["airsim_port"]), timeout_value=10)
        airsim_client.confirmConnection()
        airsim_client.enableApiControl(True)
        airsim_client.armDisarm(True)
        carla_drone_initial = np.asarray(xyz(drone_actor.get_location()), dtype=float)
        airsim_initial = airsim_client.simGetVehiclePose().position
        airsim_initial_ned = np.asarray([airsim_initial.x_val, airsim_initial.y_val, airsim_initial.z_val], dtype=float)
        ned_offset = airsim_initial_ned - carla_drone_initial * np.asarray([1.0, 1.0, -1.0])
        desired_uav = np.asarray(region["uav"]["initial_position_xyz"], dtype=float)
        desired_ned = desired_uav * np.asarray([1.0, 1.0, -1.0]) + ned_offset
        current_pose = airsim_client.simGetVehiclePose()
        desired_pose = airsim.Pose(airsim.Vector3r(*[float(item) for item in desired_ned]), current_pose.orientation)
        airsim_client.simSetVehiclePose(desired_pose, True)
        time.sleep(0.8)

        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = bool(simulation["synchronous_mode"])
        settings.fixed_delta_seconds = float(simulation["fixed_delta_seconds"])
        world.apply_settings(settings)
        traffic_manager = client.get_trafficmanager(int(simulation["traffic_manager_port"]))
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(int(config["random_seed"]))

        controller_blueprint = library.find("controller.ai.walker")
        controllers = []
        for walker, destination in zip(walkers, walker_destinations):
            controller = world.try_spawn_actor(controller_blueprint, carla.Transform(), attach_to=walker)
            if controller is not None:
                created_actors.append(controller)
                controllers.append((controller, destination))
        world.tick()
        for controller, destination in controllers:
            controller.start()
            controller.go_to_location(destination)
            controller.set_max_speed(1.2)

        stream_configs = sensors_config["streams"]
        for stream_name in ("ugv_rgb", "ugv_depth"):
            stream = stream_configs[stream_name]
            location_values = stream["location_xyz_m"]
            rotation_values = stream["rotation_pyr_degrees"]
            transform = carla.Transform(
                carla.Location(x=location_values[0], y=location_values[1], z=location_values[2]),
                carla.Rotation(pitch=rotation_values[0], yaw=rotation_values[1], roll=rotation_values[2]),
            )
            blueprint = sensor_blueprint(library, stream["type"], sensors_config)
            sensor = world.spawn_actor(blueprint, transform, attach_to=ugv)
            created_actors.append(sensor)
            sensor_actors[stream_name] = sensor
            queue_object: queue.Queue = queue.Queue(maxsize=32)
            queues[stream_name] = queue_object
            sensor.listen(lambda data, target=queue_object: push(target, data))

        current_uav_location = drone_actor.get_location()
        if current_uav_location.distance(carla.Location(*desired_uav)) > 10.0:
            current_uav_location = carla.Location(x=float(desired_uav[0]), y=float(desired_uav[1]), z=float(desired_uav[2]))
        for stream_name in ("uav_rgb", "uav_depth"):
            stream = stream_configs[stream_name]
            camera_location = current_uav_location + carla.Location(z=float(stream.get("vertical_offset_m", -1.5)))
            transform = carla.Transform(camera_location, carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0))
            blueprint = sensor_blueprint(library, stream["type"], sensors_config)
            sensor = world.spawn_actor(blueprint, transform)
            created_actors.append(sensor)
            sensor_actors[stream_name] = sensor
            queue_object = queue.Queue(maxsize=32)
            queues[stream_name] = queue_object
            sensor.listen(lambda data, target=queue_object: push(target, data))

        latest: dict[str, Any] = {}
        frame_buffers: dict[str, dict[int, Any]] = {name: {} for name in queues}
        seen_common_frames: set[int] = set()
        common_packets: dict[str, Any] | None = None
        total_ticks = int(round(float(simulation["preview_duration_seconds"]) / float(simulation["fixed_delta_seconds"])))
        event_log = (run_dir / "logs" / "events.jsonl").open("w", encoding="utf-8")
        for tick_index in range(total_ticks):
            # CI-E0 validates scene geometry rather than multirotor dynamics.
            # Re-apply the fixed preview pose so gravity/controller settling
            # cannot move the platform away from its configured 40 m altitude.
            if tick_index % 2 == 0:
                airsim_client.simSetVehiclePose(desired_pose, True)
            drone_location = drone_actor.get_location()
            if drone_location.distance(carla.Location(x=float(desired_uav[0]), y=float(desired_uav[1]), z=float(desired_uav[2]))) > 12.0:
                drone_location = carla.Location(x=float(desired_uav[0]), y=float(desired_uav[1]), z=float(desired_uav[2]))
            for stream_name in ("uav_rgb", "uav_depth"):
                stream = stream_configs[stream_name]
                camera_location = drone_location + carla.Location(z=float(stream.get("vertical_offset_m", -1.5)))
                camera_transform = carla.Transform(camera_location, carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0))
                sensor_actors[stream_name].set_transform(camera_transform)
            frame = world.tick()
            time.sleep(0.01)
            for stream_name, queue_object in queues.items():
                for packet in drain_all(queue_object):
                    packet_frame = int(packet.frame)
                    latest[stream_name] = packet
                    frame_buffers[stream_name][packet_frame] = packet
                oldest_allowed = int(frame) - 30
                frame_buffers[stream_name] = {
                    packet_frame: packet
                    for packet_frame, packet in frame_buffers[stream_name].items()
                    if packet_frame >= oldest_allowed
                }
            common_frames = set.intersection(*(set(buffer) for buffer in frame_buffers.values()))
            new_common_frames = sorted(common_frames - seen_common_frames)
            if new_common_frames:
                selected_frame = new_common_frames[-1]
                common_packets = {name: buffer[selected_frame] for name, buffer in frame_buffers.items()}
                seen_common_frames.update(new_common_frames)
            if tick_index % 20 == 0:
                event_log.write(json.dumps({"tick_index": tick_index, "carla_frame": frame, "latest_sensor_frames": {name: int(packet.frame) for name, packet in latest.items()}}, ensure_ascii=False) + "\n")
        event_log.close()
        if common_packets is None:
            raise RuntimeError(f"No common sensor frame was observed; latest={ {name: int(packet.frame) for name, packet in latest.items()} }")

        airsim_client.simSetVehiclePose(desired_pose, True)
        world.tick()
        time.sleep(0.05)

        for sensor in sensor_actors.values():
            sensor.stop()
        save_sensor_previews(run_dir, common_packets, sensors_config)
        # Refresh records after the world has advanced. CARLA actors report a
        # zero transform immediately after spawn until at least one tick.
        for record in actor_records:
            live_actor = world.get_actor(int(record["actor_id"]))
            if live_actor is None:
                continue
            live_transform = live_actor.get_transform()
            record["location_xyz"] = xyz(live_transform.location)
            record["rotation_pyr_degrees"] = rotation_pyr(live_transform.rotation)
        final_drone_location = drone_actor.get_location()
        drone_error = final_drone_location.distance(carla.Location(x=float(desired_uav[0]), y=float(desired_uav[1]), z=float(desired_uav[2])))
        actor_records.append({
            "actor_id": int(drone_actor.id),
            "role": "inspection_uav",
            "category": "uav",
            "type_id": drone_actor.type_id,
            "location_xyz": xyz(final_drone_location),
            "rotation_pyr_degrees": rotation_pyr(drone_actor.get_transform().rotation),
            "attributes": {"configured_position_xyz": desired_uav.tolist(), "position_error_m": float(drone_error)},
        })
        add_check(checks, "uav_position", drone_error <= 12.0, f"error {drone_error:.2f} m", severity="warning")
        packet_frames = {name: int(packet.frame) for name, packet in common_packets.items()}
        add_check(checks, "sensor_frame_alignment", len(set(packet_frames.values())) == 1, json.dumps(packet_frames))
        common_count = len(seen_common_frames)
        add_check(checks, "synchronized_samples", common_count >= 10, f"{common_count} common frames")

        plot_scene_map(run_dir / "preview" / "map_overview.png", world, config, actor_records)
        write_actor_csv(run_dir / "actors.csv", actor_records)
        manifest.update({
            "status": "PASS",
            "ended_utc": now_utc(),
            "map": carla_map.name,
            "carla_client_version": client.get_client_version(),
            "carla_server_version": client.get_server_version(),
            "region_id": region["region_id"],
            "target": {"actor_id": int(target.id), "type_id": target.type_id, "color_rgb": target_color, "placement": placement, "location_xyz": xyz(target.get_location())},
            "ugv": {"actor_id": int(ugv.id), "type_id": ugv.type_id, "location_xyz": xyz(ugv.get_location())},
            "uav": {"actor_id": int(drone_actor.id), "type_id": drone_actor.type_id, "configured_altitude_m": float(region["uav"]["altitude_m"]), "configured_position_xyz": desired_uav.tolist(), "actual_position_xyz": xyz(final_drone_location), "position_error_m": float(drone_error), "ned_offset": ned_offset.tolist()},
            "counts": {"vehicle_distractors": len(distractor_vehicles), "pedestrians": len(walkers), "total_interferers": len(distractor_vehicles) + len(walkers)},
            "ugv_reference_route_length_m": float(region["planned_ugv_route_length_m"]),
            "sensor_alignment": {"common_frame": next(iter(packet_frames.values())), "frames": packet_frames, "frame_spread": max(packet_frames.values()) - min(packet_frames.values()), "common_frame_count": common_count},
            "streams": {
                "uav_rgb": "preview/uav_rgb.png",
                "uav_camera_vertical_offset_m": float(stream_configs["uav_rgb"].get("vertical_offset_m", -1.5)),
                "uav_depth_raw": "preview/uav_depth_raw.png",
                "uav_depth_metres": "preview/uav_depth_metres.npy",
                "uav_depth_preview": "preview/uav_depth_preview.png",
                "ugv_rgb": "preview/ugv_rgb.png",
                "ugv_depth_raw": "preview/ugv_depth_raw.png",
                "ugv_depth_metres": "preview/ugv_depth_metres.npy",
                "ugv_depth_preview": "preview/ugv_depth_preview.png",
            },
            "stage_boundary": "CI-E0 scene validation; CARLA truth is not a perception result",
        })
        config["actors"]["target_vehicle"]["resolved_blueprint"] = target_blueprint_id
        config["_runtime_resolution"] = {
            "target_actor_id": int(target.id),
            "target_actual_transform": {"location_xyz": xyz(target.get_location()), "rotation_pyr_degrees": rotation_pyr(target.get_transform().rotation)},
            "target_placement": placement,
            "ugv_actor_id": int(ugv.id),
            "uav_actor_id": int(drone_actor.id),
            "sensor_frame": next(iter(packet_frames.values())),
        }
        write_yaml(run_dir / "resolved_config.yaml", config)
        manifest["config_sha256"] = hashlib.sha256((run_dir / "resolved_config.yaml").read_bytes()).hexdigest()
        build_composite(run_dir, manifest)
        output_files = [
            "resolved_config.yaml", "actors.csv", "preview/map_overview.png", "preview/uav_rgb.png",
            "preview/uav_depth_preview.png", "preview/ugv_rgb.png", "preview/ugv_depth_preview.png",
            "preview/scene_composite.png",
        ]
        for relative in output_files:
            add_check(checks, f"output:{relative}", (run_dir / relative).exists(), relative)
        status = "PASS" if not any(not item["passed"] and item["severity"] == "error" for item in checks) else "FAIL"
        manifest["status"] = status
        validation_report = {
            "status": status,
            "run_id": run_id,
            "checks_total": len(checks),
            "checks_passed": sum(bool(item["passed"]) for item in checks),
            "errors": [item for item in checks if not item["passed"] and item["severity"] == "error"],
            "warnings": [item for item in checks if not item["passed"] and item["severity"] == "warning"],
            "checks": checks,
        }
        json_dump(run_dir / "validation_report.json", validation_report)
        json_dump(run_dir / "scene_manifest.json", manifest)
        print(json.dumps({"status": status, "run_dir": str(run_dir), "validation": validation_report}, ensure_ascii=False, indent=2))
        return 0 if status == "PASS" else 2
    except Exception as exc:
        manifest["status"] = "FAIL"
        manifest["ended_utc"] = now_utc()
        manifest["error"] = str(exc)
        (run_dir / "logs" / "error.txt").write_text(traceback.format_exc(), encoding="utf-8")
        json_dump(run_dir / "validation_report.json", {"status": "FAIL", "error": str(exc), "checks": checks})
        json_dump(run_dir / "scene_manifest.json", manifest)
        print(traceback.format_exc())
        return 1
    finally:
        if world is not None and original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass
        if traffic_manager is not None:
            try:
                traffic_manager.set_synchronous_mode(False)
            except Exception:
                pass
        for actor in reversed(created_actors):
            try:
                if hasattr(actor, "stop") and actor not in sensor_actors.values():
                    actor.stop()
                actor.destroy()
            except Exception:
                pass
        if airsim_client is not None:
            try:
                airsim_client.armDisarm(False)
                airsim_client.enableApiControl(False)
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
