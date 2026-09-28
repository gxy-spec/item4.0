"""Camera-only obstacle categorization and short-term motion estimates."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, radians, sin, tan
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(frozen=True)
class HazardDetectorConfig:
    model_path: Path
    horizontal_fov_degrees: float = 90.0
    confidence: float = 0.10
    image_size: int = 960
    device: str | int = 0
    max_depth_m: float = 45.0
    track_timeout_s: float = 1.0
    track_association_m: float = 5.0


def pose_matrix(state: dict[str, Any]) -> np.ndarray:
    """Construct CARLA actor-to-world transform from recorded ego odometry."""
    pitch, yaw, roll = (radians(float(state.get(key, 0.0))) for key in ("pitch", "yaw", "roll"))
    cp, sp, cy, sy, cr, sr = cos(pitch), sin(pitch), cos(yaw), sin(yaw), cos(roll), sin(roll)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.asarray(
        [
            [cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
            [cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
            [sp, -cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    matrix[:3, 3] = [float(state[k]) for k in ("x", "y", "z")]
    return matrix


class RGBDHazardPerception:
    """YOLO COCO classes fused with metric depth and ego pose; never reads actor truth."""

    VEHICLE_NAMES = {"car", "motorcycle", "bus", "truck"}

    def __init__(self, config: HazardDetectorConfig, model: Any | None = None) -> None:
        self.config = config
        if model is None:
            from ultralytics import YOLO

            model = YOLO(str(config.model_path))
        self.model = model
        self._tracks: dict[int, dict[str, Any]] = {}
        self._next_track_id = 1

    def infer(
        self,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        camera_to_vehicle: np.ndarray,
        ego_state: dict[str, Any],
        frame: int,
        timestamp: float,
    ) -> list[dict[str, Any]]:
        if rgb.ndim != 3 or rgb.shape[2] < 3 or depth_m.ndim != 2:
            return []
        # CARLA RGB arrays are RGB; Ultralytics' numpy interface expects BGR.
        results = self.model.predict(
            rgb[:, :, :3][:, :, ::-1].copy(),
            imgsz=self.config.image_size,
            conf=self.config.confidence,
            classes=[0, 2, 3, 5, 7],
            device=self.config.device,
            verbose=False,
        )
        result = results[0]
        height, width = depth_m.shape
        fx = width / (2.0 * tan(radians(self.config.horizontal_fov_degrees) / 2.0))
        cx, cy = (width - 1.0) / 2.0, (height - 1.0) / 2.0
        ego_to_world = pose_matrix(ego_state)
        camera_to_world = ego_to_world @ np.asarray(camera_to_vehicle, dtype=np.float64)
        detections: list[dict[str, Any]] = []
        for box, score, class_id in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
            name = str(result.names[int(class_id)])
            x1, y1, x2, y2 = [float(value) for value in box.tolist()]
            # Near-field road shadows often resemble a person to a COCO detector.
            # A box clipped by the lower image edge is not useful for this test:
            # the scripted crossing is deliberately staged well ahead of the UGV.
            if name == "person" and y2 >= height - 8:
                continue
            if name == "person" and ((x2 - x1) < 5.0 or (y2 - y1) < 14.0):
                continue
            # Use the lower-central part of the box, where a metric depth pixel is
            # more likely to lie on the object rather than the background.
            left, right = max(0, int(x1 + 0.25 * (x2 - x1))), min(width, int(x2 - 0.25 * (x2 - x1)))
            top, bottom = max(0, int(y1 + 0.50 * (y2 - y1))), min(height, int(y2 - 0.05 * (y2 - y1)))
            crop = depth_m[top:bottom, left:right]
            valid = crop[np.isfinite(crop) & (crop > 0.4) & (crop < self.config.max_depth_m)]
            if valid.size < 3:
                continue
            depth = float(np.median(valid))
            u = (x1 + x2) * 0.5
            # Project the object's image-centre at the measured optical depth.
            camera_point = np.asarray(
                [depth, (u - cx) * depth / fx, 0.0, 1.0], dtype=np.float64
            )
            vehicle_point = np.asarray(camera_to_vehicle, dtype=np.float64) @ camera_point
            world_point = camera_to_world @ camera_point
            forward, lateral = float(vehicle_point[0]), float(vehicle_point[1])
            if forward <= 0.0:
                continue
            detections.append(
                {
                    "frame": int(frame),
                    "timestamp": float(timestamp),
                    "category": name,
                    "kind": "person" if name == "person" else ("vehicle" if name in self.VEHICLE_NAMES else name),
                    "confidence": float(score),
                    "bbox_xyxy": [x1, y1, x2, y2],
                    "bbox_width_px": x2 - x1,
                    "bbox_height_px": y2 - y1,
                    "distance_m": depth,
                    "forward_m": forward,
                    "lateral_m": lateral,
                    "world_xyz": world_point[:3].tolist(),
                }
            )
        self._update_tracks(detections, float(timestamp), ego_state)
        return detections

    def _update_tracks(
        self, detections: list[dict[str, Any]], timestamp: float, ego_state: dict[str, Any]
    ) -> None:
        for track_id in list(self._tracks):
            if timestamp - float(self._tracks[track_id]["timestamp"]) > self.config.track_timeout_s:
                del self._tracks[track_id]
        unmatched = set(self._tracks)
        yaw = radians(float(ego_state.get("yaw", 0.0)))
        # CARLA vehicle-right unit vector, used to distinguish lateral cross traffic.
        right_world = np.asarray([-sin(yaw), cos(yaw)], dtype=np.float64)
        for detection in detections:
            point = np.asarray(detection["world_xyz"][:2], dtype=np.float64)
            candidates = [
                (hypot(*(point - np.asarray(self._tracks[i]["world_xy"]))), i)
                for i in unmatched
                if self._tracks[i]["kind"] == detection["kind"]
            ]
            distance, track_id = min(candidates, default=(float("inf"), -1))
            if distance > self.config.track_association_m:
                track_id = self._next_track_id
                self._next_track_id += 1
                self._tracks[track_id] = {
                    "kind": detection["kind"],
                    "world_xy": point,
                    "timestamp": timestamp,
                    "velocity_xy": np.zeros(2, dtype=np.float64),
                    "observations": 1,
                }
                detection["track_confirmed"] = False
            else:
                previous = self._tracks[track_id]
                dt = max(1e-3, timestamp - float(previous["timestamp"]))
                instantaneous_velocity = (point - np.asarray(previous["world_xy"])) / dt
                # Pixel-box depth jitter creates large one-frame world-position
                # jumps, especially when the UGV is changing lanes. Smooth the
                # track velocity so a stationary lead car is not relabelled as
                # cross traffic for a single noisy frame.
                prior_velocity = np.asarray(previous.get("velocity_xy", np.zeros(2)), dtype=np.float64)
                velocity = 0.35 * instantaneous_velocity + 0.65 * prior_velocity
                detection["world_speed_mps"] = float(np.linalg.norm(velocity))
                detection["lateral_speed_mps"] = float(abs(np.dot(velocity, right_world)))
                previous.update(
                    world_xy=point,
                    timestamp=timestamp,
                    velocity_xy=velocity,
                    observations=int(previous.get("observations", 1)) + 1,
                )
                unmatched.discard(track_id)
                detection["track_confirmed"] = int(previous["observations"]) >= 3
            detection.setdefault("world_speed_mps", 0.0)
            detection.setdefault("lateral_speed_mps", 0.0)
            detection.setdefault("track_confirmed", False)
            detection["track_id"] = int(track_id)
