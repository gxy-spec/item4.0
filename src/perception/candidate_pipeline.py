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

from .attributes import analyze_color, color_binary_mask
from .task_spec import TaskSpec, canonical_category


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
    detector_backend: str = "auto"
    device: str | int = 0
    task_spec: TaskSpec | None = None
    enable_attribute_proposals: bool = True
    detector_prompts: tuple[str, ...] | None = None
    attribute_model_path: Path | None = None
    attribute_type_threshold: float = 0.65
    attribute_color_threshold: float = 0.65
    depth_proposal_refinement: bool = False
    auxiliary_detector_path: Path | None = None


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
    """Fuse instruction-conditioned object proposals, attributes and RGB-D geometry."""
    _auxiliary_models = {}

    def __init__(self, config: PipelineConfig, model: Any | None = None) -> None:
        self.config = config
        self.task_spec = config.task_spec or TaskSpec.legacy_red_van()
        requested_backend = config.detector_backend.lower()
        if requested_backend not in {"auto", "coco", "open_vocabulary", "custom"}:
            raise ValueError(f"unsupported detector_backend: {config.detector_backend}")
        self.open_vocabulary = requested_backend == "open_vocabulary" or (
            requested_backend == "auto" and "yoloe" in config.model_path.stem.lower()
        )
        if model is None:
            if self.open_vocabulary:
                from ultralytics import YOLOE

                model = YOLOE(str(config.model_path))
            else:
                from ultralytics import YOLO

                model = YOLO(str(config.model_path))
        self.model = model
        fixed_head = bool(getattr(getattr(self.model, "model", None), "model", []) and
                          getattr(self.model.model.model[-1], "is_fused", False))
        self.custom_classes = requested_backend == "custom" or (self.open_vocabulary and fixed_head)
        if self.custom_classes:
            self.open_vocabulary = False
            names = self.model.names
            known_names = list(names.values()) if isinstance(names, dict) else list(names)
            required = config.detector_prompts or self.task_spec.detector_prompts
            missing = set(required) - set(known_names)
            if missing:
                raise ValueError(f"Custom detector does not support requested categories: {sorted(missing)}")
        if self.open_vocabulary:
            if not hasattr(self.model, "set_classes"):
                raise TypeError("open-vocabulary detector must provide set_classes()")
            self.model.set_classes(list(config.detector_prompts or self.task_spec.detector_prompts))
        self.history: list[list[dict[str, Any]]] = []
        self.attribute_recognizer = None
        self.auxiliary_model = None
        if config.auxiliary_detector_path is not None:
            from ultralytics import YOLO
            key=str(config.auxiliary_detector_path.resolve())
            if key not in self._auxiliary_models:self._auxiliary_models[key]=YOLO(key)
            self.auxiliary_model=self._auxiliary_models[key]
        if config.attribute_model_path is not None:
            from .vehicle_attributes import VehicleAttributeRecognizer
            self.attribute_recognizer=VehicleAttributeRecognizer(config.attribute_model_path,config.device)

    def _predict(self, source: Any) -> list[Any]:
        kwargs: dict[str, Any] = {
            "imgsz": self.config.detector_image_size,
            "conf": self.config.detector_confidence,
            "device": self.config.device,
            "verbose": False,
        }
        if not self.open_vocabulary and not self.custom_classes:
            kwargs["classes"] = list(VEHICLE_CLASS_IDS)
        return self.model.predict(source, **kwargs)

    def infer_batch(self, image_paths: list[Path]) -> list[Any]:
        return self._predict([str(path) for path in image_paths])

    def infer_images(self, images_bgr: list[np.ndarray]) -> list[Any]:
        """Run the shared detector on in-memory synchronized images."""
        return self._predict(images_bgr)

    @property
    def detector_proposal_source(self) -> str:
        """Source label that reflects the configured model file, not an old default."""
        stem = self.config.model_path.stem.lower().replace("-", "_")
        backend = "custom" if self.custom_classes else "open_vocab" if self.open_vocabulary else "coco"
        return f"{stem}_{backend}"

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
        """Compatibility shim for earlier experiments and regression tests."""
        return color_binary_mask(image_bgr, "red")

    def _attribute_proposals(
        self, image_bgr: np.ndarray, existing: list[dict[str, Any]]
    ) -> list[dict[str, Any]]:
        target_color = self.task_spec.target_color
        if not self.config.enable_attribute_proposals or target_color is None:
            return []
        mask = color_binary_mask(image_bgr, target_color)
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
                    "proposal_source": f"instruction_guided_{target_color}_region",
                }
            )
        return proposals

    # Kept for callers that used the old private helper during analysis.
    def _red_proposals(self, image_bgr: np.ndarray, existing: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return self._attribute_proposals(image_bgr, existing)

    @staticmethod
    def _instance_mask(detector_result: Any, index: int, image_shape: tuple[int, int]) -> np.ndarray | None:
        masks = getattr(detector_result, "masks", None)
        data = getattr(masks, "data", None)
        if data is None or index >= len(data):
            return None
        item = data[index]
        if hasattr(item, "detach"):
            item = item.detach()
        if hasattr(item, "cpu"):
            item = item.cpu()
        if hasattr(item, "numpy"):
            item = item.numpy()
        mask = np.asarray(item, dtype=np.uint8)
        height, width = image_shape
        if mask.shape != (height, width):
            mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
        return mask.astype(bool)

    def _localize(
        self,
        depth: np.ndarray,
        bbox: list[float],
        transform: dict[str, list[float]],
        instance_mask: np.ndarray | None = None,
    ) -> tuple[float | None, list[float] | None]:
        x1, y1, x2, y2 = map(int, bbox)
        dx, dy = max(1, int((x2 - x1) * 0.25)), max(1, int((y2 - y1) * 0.25))
        crop = depth[max(0, y1 + dy) : min(depth.shape[0], y2 - dy), max(0, x1 + dx) : min(depth.shape[1], x2 - dx)]
        valid_mask = np.isfinite(crop) & (crop > 0.5) & (crop < 250.0)
        if instance_mask is not None and instance_mask.shape == depth.shape:
            object_crop = instance_mask[
                max(0, y1 + dy) : min(depth.shape[0], y2 - dy),
                max(0, x1 + dx) : min(depth.shape[1], x2 - dx),
            ]
            if object_crop.shape == crop.shape and object_crop.any():
                valid_mask &= object_crop
        valid = crop[valid_mask]
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
        for detection_index, (box, score, class_id) in enumerate(
            zip(detector_result.boxes.xyxy, detector_result.boxes.conf, detector_result.boxes.cls)
        ):
            instance_mask = self._instance_mask(detector_result, detection_index, image_bgr.shape[:2])
            raw.append(
                {
                    "bbox_xyxy": [float(value) for value in box.tolist()],
                    "category": detector_result.names[int(class_id)],
                    "detector_score": float(score),
                    "proposal_source": self.detector_proposal_source,
                    "_instance_mask": instance_mask,
                }
            )
        if self.auxiliary_model is not None:
            result=self.auxiliary_model.predict(image_bgr,imgsz=self.config.detector_image_size,
                conf=self.config.detector_confidence,classes=[2,3,5,7],device=self.config.device,verbose=False)[0]
            for box,score,category in zip(result.boxes.xyxy,result.boxes.conf,result.boxes.cls):
                raw.append({'bbox_xyxy':[float(v) for v in box.tolist()],'category':result.names[int(category)],
                    'detector_score':float(score),'proposal_source':'yolo26x_coco_aux'})
        proposals = self._class_agnostic_nms(raw)
        if self.auxiliary_model is not None:
            from .auxiliary_vehicle_proposals import suppress_contained_vehicle_parts
            proposals=suppress_contained_vehicle_parts(proposals,depth)
        proposals.extend(self._attribute_proposals(image_bgr, proposals))
        if self.config.depth_proposal_refinement:
            from .depth_proposal_refinement import refine_depth_proposal
            proposals=self._class_agnostic_nms([refine_depth_proposal(depth,p) for p in proposals])
        learned_attributes=(self.attribute_recognizer.predict(image_bgr,[p['bbox_xyxy'] for p in proposals])
                            if self.attribute_recognizer is not None else [None]*len(proposals))
        candidates: list[dict[str, Any]] = []
        for proposal_index, proposal in enumerate(proposals):
            learned=learned_attributes[proposal_index]
            detector_category=proposal.get('category')
            instance_mask = proposal.get("_instance_mask")
            color_evidence = analyze_color(image_bgr, proposal["bbox_xyxy"], instance_mask)
            target_color = self.task_spec.target_color
            target_color_ratio = color_evidence.score(target_color)
            target_color_match = target_color is None or (
                target_color_ratio >= self.config.red_ratio_threshold
            )
            learned_color=None;learned_color_score=None;target_color_probability=None
            if learned is not None:
                proposal={**proposal,'detector_category':detector_category,
                    'category':learned['predicted_type'] if learned['type_probability']>=self.config.attribute_type_threshold else 'unknown'}
                learned_color=learned['predicted_color'] if learned['color_probability']>=self.config.attribute_color_threshold else 'unknown'
                learned_color_score=learned['color_probability']
                target_color_probability=learned['color_probabilities'].get(target_color,0.) if target_color else 1.
                target_color_match=target_color is None or (learned_color==target_color and target_color_probability>=self.config.attribute_color_threshold)
            depth_m, world_xyz = self._localize(
                depth, proposal["bbox_xyxy"], transform, instance_mask
            )
            physical_footprint_m2: float | None = None
            if proposal["proposal_source"].startswith("instruction") and (depth_m is None or world_xyz is None):
                continue
            if proposal["proposal_source"].startswith("instruction"):
                x1, y1, x2, y2 = proposal["bbox_xyxy"]
                focal = self.config.image_width / (2.0 * tan(radians(self.config.fov_degrees) / 2.0))
                physical_width = (x2 - x1) * depth_m / focal
                physical_height = (y2 - y1) * depth_m / focal
                shorter, longer = sorted((physical_width, physical_height))
                physical_footprint_m2 = float(physical_width * physical_height)
                # Reject red façades, road markings and tiny signs using only
                # observation-derived geometry.  The limits cover passenger cars,
                # vans, buses and trucks under a nadir camera.
                plausible_footprint = 0.8 <= shorter <= 4.8 and 2.0 <= longer <= 12.0
                plausible_surface_height = 1.5 <= world_xyz[2] <= 4.5
                if not (plausible_footprint and plausible_surface_height):
                    continue
            category_score = self.task_spec.category_score(proposal.get("category"))
            if self.custom_classes and self.task_spec.target.category != "vehicle":
                category_score = float(canonical_category(proposal.get("category")) == self.task_spec.target.category)
            attribute_factor = 1.0 if target_color is None else 0.50 + 0.50 * target_color_ratio
            if learned is not None:
                attribute_factor=learned['type_probability']*(target_color_probability if target_color else 1.)
            score = float(proposal["detector_score"] * attribute_factor * (0.65 + 0.35 * category_score))
            serializable_proposal = {key: value for key, value in proposal.items() if not key.startswith("_")}
            candidates.append(
                {
                    "candidate_id": f"{frame}_{proposal_index:02d}",
                    "frame": frame,
                    "timestamp": timestamp,
                    **serializable_proposal,
                    "color": learned_color if learned is not None else color_evidence.dominant_color,
                    "color_score": learned_color_score if learned is not None else color_evidence.confidence,
                    "color_ratios": color_evidence.ratios,
                    "target_color": target_color,
                    "target_color_ratio": target_color_ratio,
                    "target_color_match": target_color_match,
                    # Retained for compatibility with frozen red-van reports.
                    "red_pixel_ratio": float(color_evidence.ratios.get("red", 0.0)),
                    "category_match_score": category_score,
                    "mask_area_px": None if instance_mask is None else int(instance_mask.sum()),
                    "depth_m": depth_m,
                    "world_position_xyz": world_xyz,
                    "physical_footprint_m2": physical_footprint_m2,
                    "candidate_score": score,
                    "temporal_confirmed": False,
                    "temporal_hits": 1,
                }
            )
            if learned is not None:
                candidates[-1].update(learned)
                candidates[-1]['target_color_probability']=target_color_probability
                candidates[-1]['attribute_model']=str(self.config.attribute_model_path)
                candidates[-1]['color_decision_source']='learned_rgb_classifier'
        self._temporal_confirm(candidates)
        return candidates

    def _temporal_confirm(self, candidates: list[dict[str, Any]]) -> None:
        recent = self.history[-(self.config.temporal_window - 1) :]
        for candidate in candidates:
            if not candidate.get("target_color_match", True) or candidate["world_position_xyz"] is None:
                continue
            if float(candidate.get("category_match_score", 0.0)) <= 0.0:
                continue
            position = np.asarray(candidate["world_position_xyz"][:2])
            hits = 1
            for frame_candidates in recent:
                if any(
                    other.get("target_color_match", True)
                    and float(other.get("category_match_score", 0.0)) > 0.0
                    and other["world_position_xyz"] is not None
                    and np.linalg.norm(position - np.asarray(other["world_position_xyz"][:2])) <= self.config.association_radius_m
                    for other in frame_candidates
                ):
                    hits += 1
            candidate["temporal_hits"] = hits
            candidate["temporal_confirmed"] = hits >= self.config.temporal_hits
        self.history.append(candidates)
