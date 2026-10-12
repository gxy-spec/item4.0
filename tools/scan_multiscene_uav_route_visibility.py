"""Short synchronized RGB-D probes along added UAV scanning rows.

This is route qualification with a stationary target, not a formal A/B/C run.
The existing candidate pipeline sees RGB, depth, and camera calibration only.
Target truth is used by this survey to select a probe window and then evaluate
its rendered visibility and candidate matches; it never enters the detector.
Only representative sensor images, a replay, and compact logs are saved.
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import sys
import time
from pathlib import Path

import airsim
import carla
import cv2
import numpy as np
from PIL import Image

from validate_multiscene_feasibility import settle_static_target

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from perception.candidate_pipeline import CandidatePipeline, PipelineConfig  # noqa: E402
from sensors.depth import bgra_to_rgb, carla_depth_to_metres, colourise_depth  # noqa: E402


def project_bbox(target: carla.Actor, camera: carla.Transform) -> tuple[list[float], float]:
    inverse = np.asarray(camera.get_inverse_matrix())
    focal = 800 / (2 * math.tan(math.radians(50)))
    points = []
    for vertex in target.bounding_box.get_world_vertices(target.get_transform()):
        p = inverse @ np.asarray([vertex.x, vertex.y, vertex.z, 1])
        if p[0] > 0:
            points.append([400 + focal * p[1] / p[0], 300 - focal * p[2] / p[0]])
    if not points:
        raise RuntimeError("Target is behind the nadir camera")
    box = target.bounding_box
    aim = target.get_transform().transform(carla.Location(
        x=box.location.x, y=box.location.y, z=box.location.z + box.extent.z - 0.5))
    camera_depth = (inverse @ np.asarray([aim.x, aim.y, aim.z, 1]))[0]
    return [min(x[0] for x in points), min(x[1] for x in points),
            max(x[0] for x in points), max(x[1] for x in points)], float(camera_depth)


def wait_pair(queues: dict[str, queue.Queue], frame: int) -> dict:
    pair = {}
    for name, received in queues.items():
        deadline = time.monotonic() + 12
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Missing synchronized {name} frame {frame}")
            sample = received.get(timeout=remaining)
            if sample.frame == frame:
                pair[name] = sample
                break
            if sample.frame > frame:
                raise RuntimeError(f"Skipped {name} frame {frame}; got {sample.frame}")
    return pair


def select_window(record: dict, route: list[list[float]], leg: int, duration: float) -> tuple[list[float], list[float], float]:
    samples = [s for s in record["uav_target_visibility"]["samples"]
               if s["leg"] == leg and s["visible"]]
    if not samples:
        raise RuntimeError(f"New scan row {leg} has no visible samples")
    start, end = route[leg - 1], route[leg]
    length = math.dist(start, end)
    center = sum(math.dist(start, s["uav_xyz"]) for s in samples) / len(samples)
    window_length = min(length, duration * 3.0)
    offset = max(0.0, min(length - window_length, center - window_length / 2))
    return start, end, offset


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--preflight", type=Path,
                        default=Path("E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2/preflight.json"))
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--cases", nargs="+", default=["W1", "W2", "F1"],
                        help="Reviewed case IDs, or ALL for all twelve")
    parser.add_argument("--original-preflight", type=Path,
                        default=Path("E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009/preflight.json"))
    args = parser.parse_args()
    scenes = json.loads(args.manifest.read_text(encoding="utf-8"))["scenarios"]
    preflight = json.loads(args.preflight.read_text(encoding="utf-8"))
    original_preflight = json.loads(args.original_preflight.read_text(encoding="utf-8"))
    if any(case["scene"] in {"north", "east"} for case in preflight["candidates"]):
        raise RuntimeError("Revised-route preflight unexpectedly includes north/east; check route provenance")
    preflight["candidates"].extend(row for row in original_preflight["candidates"]
                                   if row["scene"] in {"north", "east"})
    scene_codes = {"N": "north", "E": "east", "W": "west", "F": "whole"}
    wanted = ([f"{prefix}{i}" for prefix in scene_codes for i in (1, 2, 3)]
              if args.cases == ["ALL"] else args.cases)
    for case in wanted:
        if len(case) != 2 or case[0] not in scene_codes or case[1] not in "123":
            raise ValueError(f"Unknown reviewed case {case}")
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(25)
    world = client.get_world()
    original_settings = world.get_settings()
    if original_settings.synchronous_mode:
        raise RuntimeError("Expected idle asynchronous simulator before route survey")
    if not world.get_map().name.endswith("Town10HD"):
        raise RuntimeError("Route qualification requires Town10HD")
    spawns = world.get_map().get_spawn_points()
    air = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=20)
    air.confirmConnection()
    drone = next(iter(world.get_actors().filter("airsim.drone")))
    initial_air = air.simGetVehiclePose()
    initial_carla = drone.get_location()
    offset = [initial_carla.x - initial_air.position.x_val,
              initial_carla.y - initial_air.position.y_val,
              initial_carla.z + initial_air.position.z_val]
    pipeline = CandidatePipeline(PipelineConfig(
        model_path=ROOT / "models/yolo26n.pt", image_width=800, image_height=600,
        fov_degrees=100, detector_image_size=800, detector_confidence=0.08,
        red_ratio_threshold=0.20, temporal_window=5, temporal_hits=2))
    sensors = []
    queues = {name: queue.Queue() for name in ("rgb", "depth")}
    air.enableApiControl(True)
    air.armDisarm(True)
    results = []
    try:
        # Follow the main runner's proven lifecycle: change world mode before
        # spawning a render sensor. Switching with live cameras can crash this
        # CARLA-Air Windows build inside the game thread.
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 5
        world.apply_settings(settings)
        print("Entered fixed-step mode before sensor creation", flush=True)
        for modality in ("rgb", "depth"):
            bp = world.get_blueprint_library().find(f"sensor.camera.{modality}")
            for key, value in (("image_size_x", "800"), ("image_size_y", "600"),
                               ("fov", "100"), ("sensor_tick", "0.0")):
                bp.set_attribute(key, value)
            sensor = world.spawn_actor(bp, carla.Transform(
                carla.Location(x=0, y=0, z=70), carla.Rotation(pitch=-90)))
            sensor.listen(queues[modality].put)
            sensors.append(sensor)

        def set_pose(p: list[float], deployment: bool = False) -> tuple[list[float], carla.Transform]:
            air.simSetVehiclePose(airsim.Pose(airsim.Vector3r(
                p[0] - offset[0], p[1] - offset[1], -(p[2] - offset[2])),
                airsim.to_quaternion(0, 0, 0)), deployment)
            observed = air.simGetVehiclePose()
            xyz = [observed.position.x_val + offset[0], observed.position.y_val + offset[1],
                   -observed.position.z_val + offset[2]]
            transform = carla.Transform(carla.Location(x=xyz[0], y=xyz[1], z=xyz[2] - 1.5),
                                        carla.Rotation(pitch=-90, yaw=0))
            for sensor in sensors:
                sensor.set_transform(transform)
            return xyz, transform

        for requested_case in wanted:
            scene_name, candidate_id = scene_codes[requested_case[0]], int(requested_case[1])
            scene = scenes[scene_name]
            candidate = next(c for c in scene["review_candidates"] if c["candidate"] == candidate_id)
            record = next(c for c in preflight["candidates"]
                          if c["scene"] == scene_name and c["candidate"] == candidate_id)
            visible_legs = record["uav_target_visibility"]["visible_legs"]
            if not visible_legs:
                raise RuntimeError(f"No geometrically visible leg for {requested_case}")
            leg = max(visible_legs, key=lambda value: sum(
                sample["leg"] == value and sample["visible"]
                for sample in record["uav_target_visibility"]["samples"]))
            route = scene["uav_lawnmower_waypoints_xyz"]
            start, end, distance_offset = select_window(record, route, leg, args.duration)
            leg_length = math.dist(start, end)
            case_name = requested_case
            case_dir = args.output_dir / "rgbd_probe" / case_name
            case_dir.mkdir(parents=True, exist_ok=True)
            target_bp = world.get_blueprint_library().find("vehicle.mercedes.sprinter")
            target_bp.set_attribute("color", "255,0,0")
            target = world.try_spawn_actor(target_bp, spawns[candidate["target_candidate_spawn_id"]])
            if target is None:
                raise RuntimeError(f"Target spawn failed: {case_name}")
            writer = None
            rows = []
            inference_rows = []
            try:
                print(f"{case_name}: grounding target", flush=True)
                grounding = settle_static_target(world, target)
                target_xyz = grounding["parked_xyz"]
                pipeline.history.clear()
                p0 = [start[k] + (end[k] - start[k]) * distance_offset / leg_length for k in range(3)]
                for _ in range(3):
                    set_pose(p0, deployment=True)
                    time.sleep(0.04)
                for received in queues.values():
                    while not received.empty():
                        received.get_nowait()
                # Settle both camera render streams before collecting evidence.
                for _ in range(6):
                    set_pose(p0)
                    time.sleep(0.03)
                    frame = world.tick()
                    wait_pair(queues, frame)
                baseline_collision = int(air.simGetCollisionInfo().time_stamp or 0)
                path = case_dir / "uav_rgbd_visibility_replay.mp4"
                writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (1600, 650))
                if not writer.isOpened():
                    raise RuntimeError("Could not open replay encoder")
                latest_candidates = []
                sample_count = int(args.duration / 0.05)
                print(f"{case_name}: recording {sample_count} synchronized frames", flush=True)
                for tick in range(sample_count):
                    elapsed = tick * 0.05
                    distance = min(leg_length, distance_offset + 3.0 * elapsed)
                    desired = [start[k] + (end[k] - start[k]) * distance / leg_length for k in range(3)]
                    observed, camera_tf = set_pose(desired)
                    time.sleep(0.03)
                    frame = world.tick()
                    pair = wait_pair(queues, frame)
                    rgb = bgra_to_rgb(pair["rgb"].raw_data, 800, 600)
                    depth = carla_depth_to_metres(pair["depth"].raw_data, 800, 600)
                    bbox, expected_depth = project_bbox(target, camera_tf)
                    x1, y1 = max(0, int(bbox[0])), max(0, int(bbox[1]))
                    x2, y2 = min(800, math.ceil(bbox[2])), min(600, math.ceil(bbox[3]))
                    red_ratio = 0.0
                    depth_ratio = 0.0
                    if x2 > x1 and y2 > y1:
                        mask = pipeline._red_mask(cv2.cvtColor(rgb[y1:y2, x1:x2], cv2.COLOR_RGB2BGR))
                        red_ratio = float((mask > 0).mean())
                        depth_ratio = float((np.abs(depth[y1:y2, x1:x2] - expected_depth) <= 1.0).mean())
                    rendered_visible = red_ratio >= 0.10 and depth_ratio >= 0.15
                    collision = air.simGetCollisionInfo()
                    collision_new = bool(collision.has_collided) and int(collision.time_stamp or 0) != baseline_collision
                    row = {"frame": frame, "window_sim_seconds": round(elapsed, 3),
                           "sensor_timestamp": pair["rgb"].timestamp,
                           "rgb_depth_frame_match": pair["rgb"].frame == pair["depth"].frame,
                           "rgb_depth_timestamp_error_s": abs(pair["rgb"].timestamp - pair["depth"].timestamp),
                           "uav_xyz": observed, "pose_error_m": math.dist(desired, observed),
                           "target_bbox_xyxy_px_evaluation_only": bbox,
                           "red_fraction_evaluation_only": red_ratio,
                           "roof_depth_fraction_evaluation_only": depth_ratio,
                           "rendered_target_visible_heuristic": rendered_visible,
                           "new_uav_collision": collision_new}
                    rows.append(row)
                    if tick % 10 == 0:
                        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                        response = pipeline.infer_images([bgr])[0]
                        transform_dict = {"location": [camera_tf.location.x, camera_tf.location.y, camera_tf.location.z],
                                          "rotation": [-90.0, 0.0, 0.0]}
                        latest_candidates = pipeline.process(frame, pair["rgb"].timestamp,
                                                             bgr, depth, transform_dict, response)
                        yolo_target = any(pipeline._iou(b.xyxy[0].tolist(), bbox) >= 0.3
                                          for b in response.boxes)
                        evaluated = []
                        for item in latest_candidates:
                            position = item.get("world_position_xyz")
                            error = math.dist(position[:2], target_xyz[:2]) if position is not None else None
                            evaluated.append({**item, "target_xy_error_m_evaluation_only": error,
                                "target_match_evaluation_only": error is not None and error <= 5.0
                                    and item["color"] == "red"})
                        inference_rows.append({"frame": frame, "window_sim_seconds": elapsed,
                                               "yolo_target_box_match_evaluation_only": yolo_target,
                                               "candidates": evaluated})
                    if tick % 2 == 0:
                        left = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                        for item in latest_candidates:
                            b = [int(x) for x in item["bbox_xyxy"]]
                            color = (30, 210, 30) if item["color"] == "red" and item["temporal_confirmed"] else (0, 200, 255)
                            cv2.rectangle(left, (b[0], b[1]), (b[2], b[3]), color, 2)
                        cv2.rectangle(left, (x1, y1), (x2, y2), (230, 40, 200), 2)
                        right = cv2.cvtColor(np.asarray(colourise_depth(depth)), cv2.COLOR_RGB2BGR)
                        canvas = np.zeros((650, 1600, 3), dtype=np.uint8)
                        canvas[:600, :800] = left
                        canvas[:600, 800:] = right
                        cv2.putText(canvas, f"{case_name} | t={elapsed:.1f}s | RGB (purple: GT QA; green/yellow: candidates)",
                                    (15, 622), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                        cv2.putText(canvas, f"DEPTH same frame | last inference t={inference_rows[-1]['window_sim_seconds']:.1f}s", (850, 622),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                        writer.write(canvas)
                    if tick in (0, sample_count // 2, sample_count - 1):
                        Image.fromarray(rgb).save(case_dir / f"sample_{tick:03d}_rgb.png")
                        colourise_depth(depth).save(case_dir / f"sample_{tick:03d}_depth.png")
                matches = [r for r in inference_rows if any(c["target_match_evaluation_only"] for c in r["candidates"])]
                confirmed = [r for r in inference_rows if any(c["target_match_evaluation_only"] and c["temporal_confirmed"] for c in r["candidates"])]
                visible_count = sum(r["rendered_target_visible_heuristic"] for r in rows)
                summary = {"case": case_name, "scene": scene_name, "candidate": candidate_id,
                           "manifest": str(args.manifest.resolve()), "new_scan_leg": leg,
                           "target_grounding": grounding, "scope": "short route-leg RGB-D survey, no UGV controller",
                           "camera": {"width": 800, "height": 600, "fov": 100, "pitch": -90},
                           "model": "yolo26n", "detector_image_size": 800,
                           "frames": len(rows), "matched_rgb_depth_frames": sum(r["rgb_depth_frame_match"] for r in rows),
                           "maximum_timestamp_error_s": max(r["rgb_depth_timestamp_error_s"] for r in rows),
                           "maximum_uav_pose_error_m": max(r["pose_error_m"] for r in rows),
                           "new_collision_frames": sum(r["new_uav_collision"] for r in rows),
                           "rendered_visible_frames": visible_count,
                           "rendered_visible_seconds": visible_count * 0.05,
                           "inference_frames": len(inference_rows), "target_candidate_frames": len(matches),
                           "yolo_target_box_frames": sum(r["yolo_target_box_match_evaluation_only"] for r in inference_rows),
                           "target_confirmed_frames": len(confirmed), "replay": str(path)}
                (case_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
                (case_dir / "frame_checks.json").write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
                (case_dir / "candidate_checks.json").write_text(json.dumps(inference_rows, ensure_ascii=False, indent=2), encoding="utf-8")
                results.append(summary)
                (args.output_dir / "rgbd_probe_summary.json").write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
                print(f"{case_name}: visible={visible_count}/{len(rows)} candidate={len(matches)}/{len(inference_rows)} "
                      f"confirmed={len(confirmed)} pose_error={summary['maximum_uav_pose_error_m']:.3f}m "
                      f"collision={summary['new_collision_frames']}", flush=True)
            finally:
                if writer is not None:
                    writer.release()
                target.destroy()
    finally:
        for sensor in sensors:
            sensor.stop()
            sensor.destroy()
        # Remove live render sensors before restoring asynchronous mode.
        world.tick()
        world.apply_settings(original_settings)
        air.armDisarm(False)
        air.enableApiControl(False)


if __name__ == "__main__":
    main()
