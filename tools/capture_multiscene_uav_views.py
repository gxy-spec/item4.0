"""Capture actual CARLA RGB views from candidate UAV poses with target actors.

AirSim drone is moved to each reviewed pose; CARLA camera follows the measured
drone pose as in the project's real acquisition runner. This is a sparse
feasibility probe, not a complete UAV flight or a perception performance test.
"""

from __future__ import annotations

import json
import math
import queue
import time
from pathlib import Path

import airsim
import carla
from PIL import Image, ImageDraw
import numpy as np

from validate_multiscene_feasibility import settle_static_target

ROOT = Path(__file__).resolve().parents[1]
OUT = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009")
MANIFEST = ROOT / "docs/figures/multiscene_design_20261009/proposal_manifest.json"
CHOICES = (("north", 1), ("east", 1), ("west", 3), ("whole", 2))


def main() -> None:
    scenes = json.loads(MANIFEST.read_text(encoding="utf-8"))["scenarios"]
    preflight = json.loads((OUT / "preflight.json").read_text(encoding="utf-8"))
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(20)
    world = client.get_world()
    spawns = world.get_map().get_spawn_points()
    drone = next(iter(world.get_actors().filter("airsim.drone")))
    air = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=20)
    air.confirmConnection()
    air.enableApiControl(True)
    air.armDisarm(True)
    air_pose = air.simGetVehiclePose()
    drone_loc = drone.get_location()
    offset = [drone_loc.x - air_pose.position.x_val,
              drone_loc.y - air_pose.position.y_val,
              drone_loc.z + air_pose.position.z_val]
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", "800")
    bp.set_attribute("image_size_y", "600")
    bp.set_attribute("fov", "100")
    bp.set_attribute("sensor_tick", "0.1")
    camera = world.spawn_actor(bp, carla.Transform(
        carla.Location(x=0, y=0, z=60), carla.Rotation(pitch=-90)))
    received: queue.Queue = queue.Queue()
    camera.listen(lambda data: received.put(data))
    records = []
    try:
        for scene_name, candidate_id in CHOICES:
            scene = scenes[scene_name]
            candidate = next(x for x in scene["review_candidates"] if x["candidate"] == candidate_id)
            static = next(x for x in preflight["candidates"]
                          if x["scene"] == scene_name and x["candidate"] == candidate_id)
            target_location = spawns[candidate["target_candidate_spawn_id"]].location
            visible = [s for s in static["uav_target_visibility"]["samples"] if s["visible"]]
            # Avoid a near-nadir pose where the modeled UAV itself covers the
            # target in the project camera. Prefer a 30-m horizontal offset.
            desired = min(visible, key=lambda s: abs(math.hypot(
                s["uav_xyz"][0] - target_location.x,
                s["uav_xyz"][1] - target_location.y) - 30))["uav_xyz"] if visible else None
            if desired is None:
                raise RuntimeError(f"No visible UAV waypoint: {scene_name}/{candidate_id}")
            target_bp = world.get_blueprint_library().find("vehicle.mercedes.sprinter")
            target_bp.set_attribute("color", "255,0,0")
            target = world.try_spawn_actor(target_bp, spawns[candidate["target_candidate_spawn_id"]])
            if target is None:
                raise RuntimeError(f"Target spawn failed: {scene_name}/{candidate_id}")
            try:
                settle_static_target(world, target)
                ned = airsim.Vector3r(desired[0] - offset[0], desired[1] - offset[1],
                                      -(desired[2] - offset[2]))
                for _ in range(3):
                    air.simSetVehiclePose(airsim.Pose(ned, airsim.to_quaternion(0, 0, 0)), True)
                    time.sleep(0.1)
                    measured = air.simGetVehiclePose()
                    if math.dist(desired, [measured.position.x_val + offset[0],
                                           measured.position.y_val + offset[1],
                                           -measured.position.z_val + offset[2]]) < 1.0:
                        break
                observed = [measured.position.x_val + offset[0],
                            measured.position.y_val + offset[1],
                            -measured.position.z_val + offset[2]]
                # Same -1.5 m nadir mount used by run_stage1_dynamic.py.
                camera.set_transform(carla.Transform(
                    carla.Location(x=observed[0], y=observed[1], z=observed[2] - 1.5),
                    carla.Rotation(pitch=-90, yaw=0)))
                time.sleep(0.8)
                while not received.empty():
                    received.get_nowait()
                image = received.get(timeout=10)
                path = OUT / f"uav_view_{scene_name}_{candidate_id}.png"
                image.save_to_disk(str(path))
                # Oracle-only QA overlay: bounding box proves where the target
                # should be in the image. It is never an online detector output.
                inverse = np.asarray(camera.get_transform().get_inverse_matrix())
                focal = 800 / (2 * math.tan(math.radians(100) / 2))
                projected = []
                for vertex in target.bounding_box.get_world_vertices(target.get_transform()):
                    p = inverse @ np.asarray([vertex.x, vertex.y, vertex.z, 1.0])
                    depth = p[0]
                    if depth > 0.1:
                        projected.append((400 + focal * p[1] / depth,
                                          300 - focal * p[2] / depth))
                overlay_path = OUT / f"uav_view_{scene_name}_{candidate_id}_oracle_qa.png"
                projected_bbox = None
                if projected:
                    preview = Image.open(path).convert("RGB")
                    draw = ImageDraw.Draw(preview)
                    xs, ys = [p[0] for p in projected], [p[1] for p in projected]
                    draw.rectangle((min(xs), min(ys), max(xs), max(ys)), outline=(255, 30, 30), width=3)
                    projected_bbox = [round(min(xs), 1), round(min(ys), 1),
                                      round(max(xs), 1), round(max(ys), 1)]
                    preview.save(overlay_path)
                record = {"scene": scene_name, "candidate": candidate_id,
                          "desired_uav_xyz": desired, "observed_uav_xyz": observed,
                          "pose_error_m": math.dist(desired, observed),
                          "image": str(path), "oracle_qa_overlay": str(overlay_path),
                          "target_bbox_xyxy_px": projected_bbox,
                          "frame": image.frame,
                          "target_actor_id": target.id}
                records.append(record)
                print(f"{scene_name}/{candidate_id}: pose_error={record['pose_error_m']:.2f}m image={path}", flush=True)
            finally:
                target.destroy()
        (OUT / "uav_capture_check.json").write_text(json.dumps(records, ensure_ascii=False,
                                                           indent=2), encoding="utf-8")
    finally:
        camera.stop()
        camera.destroy()
        air.armDisarm(False)
        air.enableApiControl(False)


if __name__ == "__main__":
    main()
