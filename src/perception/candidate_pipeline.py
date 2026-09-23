"""UAV RGB/depth target-candidate pipeline.

Inference consumes only synchronized UAV RGB, metric depth and camera pose.  CARLA
actor states are deliberately excluded; they are loaded by the evaluator only.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin, tan
from pathlib import Path
from typing import Any

import cv2
import numpy as np


VEHICLE_CLASS_IDS = (2, 3, 5, 7)


@dataclass(frozen=True)
class PipelineConfig:
    model_path: Path
    image_width: int = 800
    image_height: int = 600
    fov_degrees: float = 100.0
    detector_confidence: float = 0.03
    detector_image_size: int = 1280
    red_ratio_threshold: float = 0.10
    temporal_window: int = 5
    temporal_hits: int = 3
    association_radius_m: float = 5.0


def transform_matrix(transform: dict[str, list[float]]) -> np.ndarray:
    """Return CARLA sensor-to-world homogeneous transform."""
    x, y, z = transform["location"]
    pitch, yaw, roll = map(radians, transform["rotation"])
    cp, sp, cy, sy, cr, sr = cos(pitch), sin(pitch), cos(yaw), sin(yaw), cos(roll), sin(roll)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.asarray(
        [
            [cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
            [cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
            [sp, -cp * sr, cp * cr],
        ]
    )
    matrix[:3, 3] = (x, y, z)
    return matrix


def project_world_point(
    point_xyz: tuple[float, float, float],
    transform: dict[str, list[float]],
    width: int,
    height: int,
    fov_degrees: float,
) -> tuple[float, float, float]:
    """Project a CARLA world point; third value is camera-forward depth."""
    world_to_sensor = np.linalg.inv(transform_matrix(transform))
    local = world_to_sensor @ np.asarray([*point_xyz, 1.0])
    focal = width / (2.0 * tan(radians(fov_degrees) / 2.0))
    u = width / 2.0 + focal * local[1] / local[0]
    v = height / 2.0 - focal * local[2] / local[0]
    return float(u), float(v), float(local[0])


class CandidatePipeline:
    """Fuse a generic vehicle detector, instruction-guided red proposals and depth."""

    def __init__(self, config: PipelineConfig, model: Any | None = None) -> None:
        self.config = config
        if model is None:
            from ultralytics import YOLO

            model = YOLO(str(config.model_path))
        self.model = model
        self.history: list[list[dict[str, Any]]] = []

    def infer_batch(self, image_paths: list[Path]) -> list[Any]:
        return self.model.predict(
            [str(path) for path in image_paths],
            imgsz=self.config.detector_image_size,
            conf=self.config.detector_confidence,
            classes=list(VEHICLE_CLASS_IDS),
            device=0,
            verbose=False,
        )

    def infer_images(self, images_bgr: list[np.ndarray]) -> list[Any]:
        """Run the shared detector on in-memory synchronized images."""
        return self.model.predict(
            images_bgr,
            imgsz=self.config.detector_image_size,
            conf=self.config.detector_confidence,
            classes=list(VEHICLE_CLASS_IDS),
            device=0,
            verbose=False,
        )

    @staticmethod
    def _iou(a: list[float], b: list[float]) -> float:
        x1, y1 = max(a[0], b[0]), max(a[1], b[1])
        x2, y2 = min(a[2], b[2]), min(a[3], b[3])
        intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        union = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
        union += max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1]) - intersection
        return intersection / union if union > 0 else 0.0

    @classmethod
    def _class_agnostic_nms(cls, proposals: list[dict[str, Any]], threshold: float = 0.5) -> list[dict[str, Any]]:
        kept: list[dict[str, Any]] = []
        for proposal in sorted(proposals, key=lambda item: item["detector_score"], reverse=True):
            if all(cls._iou(proposal["bbox_xyxy"], other["bbox_xyxy"]) < threshold for other in kept):
                kept.append(proposal)
        return kept

    @staticmethod
    def _red_mask(image_bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        low = cv2.inRange(hsv, (0, 75, 45), (12, 255, 255))
        high = cv2.inRange(hsv, (168, 75, 45), (179, 255, 255))
        return cv2.bitwise_or(low, high)

    def _red_proposals(self, image_bgr: np.ndarray, existing: list[dict[str, Any]]) -> list[dict[str, Any]]:
        mask = self._red_mask(image_bgr)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        proposals: list[dict[str, Any]] = []
        for index in range(1, count):
            x, y, w, h, area = map(int, stats[index])
            if area < 35 or w < 7 or h < 7 or w * h > 8000:
                continue
            aspect = w / max(h, 1)
            fill = area / float(w * h)
            if not (0.35 <= aspect <= 4.8 and fill >= 0.20):
                continue
            pad_x, pad_y = max(2, int(w * 0.12)), max(2, int(h * 0.12))
            bbox = [max(0, x - pad_x), max(0, y - pad_y), min(image_bgr.shape[1] - 1, x + w + pad_x), min(image_bgr.shape[0] - 1, y + h + pad_y)]
            if any(self._iou(bbox, item["bbox_xyxy"]) >= 0.25 for item in existing):
                continue
            proposals.append(
                {
                    "bbox_xyxy": [float(value) for value in bbox],
                    "category": "vehicle_candidate",
                    "detector_score": float(min(0.49, 0.25 + fill * 0.35)),
                    "proposal_source": "instruction_guided_red_region",
                }
            )
        return proposals

    def _color(self, image_bgr: np.ndarray, bbox: list[float]) -> tuple[str, float, float]:
        x1, y1, x2, y2 = map(int, bbox)
        dx, dy = max(1, int((x2 - x1) * 0.12)), max(1, int((y2 - y1) * 0.12))
        roi = image_bgr[max(0, y1 + dy) : min(image_bgr.shape[0], y2 - dy), max(0, x1 + dx) : min(image_bgr.shape[1], x2 - dx)]
        if roi.size == 0:
            return "unknown", 0.0, 0.0
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        saturated = hsv[..., 1] >= 55
        red = (((hsv[..., 0] <= 12) | (hsv[..., 0] >= 168)) & (hsv[..., 1] >= 75) & (hsv[..., 2] >= 45))
        red_ratio = float(red.mean())
        color = "red" if red_ratio >= self.config.red_ratio_threshold else "other"
        confidence = min(1.0, red_ratio / max(self.config.red_ratio_threshold * 2.0, 1e-6)) if color == "red" else min(1.0, float(saturated.mean()))
        return color, float(confidence), red_ratio

    def _localize(
        self,
        depth: np.ndarray,
        bbox: list[float],
        transform: dict[str, list[float]],
    ) -> tuple[float | None, list[float] | None]:
        x1, y1, x2, y2 = map(int, bbox)
        dx, dy = max(1, int((x2 - x1) * 0.25)), max(1, int((y2 - y1) * 0.25))
        crop = depth[max(0, y1 + dy) : min(depth.shape[0], y2 - dy), max(0, x1 + dx) : min(depth.shape[1], x2 - dx)]
        valid = crop[np.isfinite(crop) & (crop > 0.5) & (crop < 250.0)]
        if valid.size < 4:
            return None, None
        distance = float(np.median(valid))
        u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        focal = self.config.image_width / (2.0 * tan(radians(self.config.fov_degrees) / 2.0))
        local = np.asarray([distance, (u - self.config.image_width / 2.0) * distance / focal, -(v - self.config.image_height / 2.0) * distance / focal, 1.0])
        world = transform_matrix(transform) @ local
        return distance, [float(world[0]), float(world[1]), float(world[2])]

    def process(
        self,
        frame: int,
        timestamp: float,
        image_bgr: np.ndarray,
        depth: np.ndarray,
        transform: dict[str, list[float]],
        detector_result: Any,
    ) -> list[dict[str, Any]]:
        raw: list[dict[str, Any]] = []
        for box, score, class_id in zip(detector_result.boxes.xyxy, detector_result.boxes.conf, detector_result.boxes.cls):
            raw.append(
                {
                    "bbox_xyxy": [float(value) for value in box.tolist()],
                    "category": detector_result.names[int(class_id)],
                    "detector_score": float(score),
                    "proposal_source": "yolo26x_coco",
                }
            )
        proposals = self._class_agnostic_nms(raw)
        proposals.extend(self._red_proposals(image_bgr, proposals))
        candidates: list[dict[str, Any]] = []
        for proposal_index, proposal in enumerate(proposals):
            color, color_score, red_ratio = self._color(image_bgr, proposal["bbox_xyxy"])
            depth_m, world_xyz = self._localize(depth, proposal["bbox_xyxy"], transform)
            if proposal["proposal_source"].startswith("instruction") and (depth_m is None or world_xyz is None):
                continue
            if proposal["proposal_source"].startswith("instruction"):
                x1, y1, x2, y2 = proposal["bbox_xyxy"]
                focal = self.config.image_width / (2.0 * tan(radians(self.config.fov_degrees) / 2.0))
                physical_width = (x2 - x1) * depth_m / focal
                physical_height = (y2 - y1) * depth_m / focal
                shorter, longer = sorted((physical_width, physical_height))
                # Reject red façades, road markings and tiny signs using only
                # observation-derived geometry.  The limits cover passenger cars,
                # vans, buses and trucks under a nadir camera.
                plausible_footprint = 0.8 <= shorter <= 4.8 and 2.0 <= longer <= 12.0
                plausible_surface_height = 1.5 <= world_xyz[2] <= 4.5
                if not (plausible_footprint and plausible_surface_height):
                    continue
            score = float(proposal["detector_score"] * (0.55 + 0.45 * color_score))
            candidates.append(
                {
                    "candidate_id": f"{frame}_{proposal_index:02d}",
                    "frame": frame,
                    "timestamp": timestamp,
                    **proposal,
                    "color": color,
                    "color_score": color_score,
                    "red_pixel_ratio": red_ratio,
                    "depth_m": depth_m,
                    "world_position_xyz": world_xyz,
                    "candidate_score": score,
                    "temporal_confirmed": False,
                    "temporal_hits": 1,
                }
            )
        self._temporal_confirm(candidates)
        return candidates

    def _temporal_confirm(self, candidates: list[dict[str, Any]]) -> None:
        recent = self.history[-(self.config.temporal_window - 1) :]
        for candidate in candidates:
            if candidate["color"] != "red" or candidate["world_position_xyz"] is None:
                continue
            position = np.asarray(candidate["world_position_xyz"][:2])
            hits = 1
            for frame_candidates in recent:
                if any(
                    other["color"] == "red"
                    and other["world_position_xyz"] is not None
                    and np.linalg.norm(position - np.asarray(other["world_position_xyz"][:2])) <= self.config.association_radius_m
                    for other in frame_candidates
                ):
                    hits += 1
            candidate["temporal_hits"] = hits
            candidate["temporal_confirmed"] = hits >= self.config.temporal_hits
        self.history.append(candidates)
