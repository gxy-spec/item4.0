"""Instruction-conditioned S1 candidate transmission and local verification gates."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Any

from .task_spec import TaskSpec


@dataclass(frozen=True)
class TransmissionGateConfig:
    # Field name retained so frozen red-van configs remain loadable.  For a
    # general TaskSpec it is the minimum requested-color ratio.
    minimum_red_ratio: float = 0.70
    minimum_temporal_hits: int = 4
    minimum_detector_score: float = 0.03
    allowed_sources: tuple[str, ...] = ("yolo26x_coco",)
    minimum_world_z_m: float = 0.4
    maximum_world_z_m: float = 4.5
    minimum_category_score: float = 0.50
    minimum_relation_score: float = 0.50


@dataclass(frozen=True)
class LocalVerificationConfig:
    minimum_red_ratio: float = 0.35
    minimum_temporal_hits: int = 3
    minimum_detector_score: float = 0.10
    association_radius_m: float = 5.0
    minimum_van_like_score: float = 0.60
    allowed_sources: tuple[str, ...] = ("yolo26x_coco",)
    minimum_category_score: float = 0.50


def van_like_score(candidate: dict[str, Any]) -> float:
    """Transparent COCO-class/shape heuristic; it is not a learned van classifier."""
    category = str(candidate.get("category", ""))
    class_prior = {"bus": 0.95, "truck": 0.90, "car": 0.62}.get(category, 0.0)
    x1, y1, x2, y2 = candidate.get("bbox_xyxy", [0.0, 0.0, 0.0, 0.0])
    width = max(1.0, float(x2) - float(x1))
    height = max(1.0, float(y2) - float(y1))
    aspect = max(width, height) / min(width, height)
    shape_bonus = min(0.08, max(0.0, aspect - 1.4) * 0.04)
    return float(min(1.0, class_prior + shape_bonus))


class TargetRanker:
    def __init__(self, config: TransmissionGateConfig, task_spec: TaskSpec | None = None) -> None:
        self.config = config
        self.task_spec = task_spec or TaskSpec.legacy_red_van()

    def _attribute_score(self, candidate: dict[str, Any]) -> float:
        target_color = self.task_spec.target_color
        if target_color is None:
            return 1.0
        if candidate.get("target_color") == target_color:
            return float(candidate.get("target_color_ratio", 0.0))
        if isinstance(candidate.get("color_ratios"), dict):
            return float(candidate["color_ratios"].get(target_color, 0.0))
        if target_color == "red":
            if candidate.get("color") not in {None, "unknown", "red"}:
                return 0.0
            return float(candidate.get("red_pixel_ratio", 0.0))
        return float(candidate.get("color_score", 0.0)) if candidate.get("color") == target_color else 0.0

    @staticmethod
    def _bbox_center(candidate: dict[str, Any]) -> tuple[float, float] | None:
        bbox = candidate.get("bbox_xyxy")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return None
        return ((float(bbox[0]) + float(bbox[2])) / 2.0, (float(bbox[1]) + float(bbox[3])) / 2.0)

    @staticmethod
    def _object_color_score(candidate: dict[str, Any], color: str | None) -> float:
        if color is None:
            return 1.0
        ratios = candidate.get("color_ratios")
        if isinstance(ratios, dict):
            return float(ratios.get(color, 0.0))
        return float(candidate.get("color_score", 0.0)) if candidate.get("color") == color else 0.0

    def _relation_score(
        self,
        target: dict[str, Any],
        candidates: list[dict[str, Any]],
    ) -> tuple[float, list[dict[str, Any]]]:
        """Score requested scene relations using detector/depth observations only.

        Metric ``near``/``beside`` relations use reconstructed world distance.
        Directional relations use the camera image frame (or depth ordering for
        front/behind), which keeps the meaning explicit when object heading is
        not observable from one RGB-D frame.
        """
        if not self.task_spec.relations:
            return 1.0, []
        evidence: list[dict[str, Any]] = []
        scores: list[float] = []
        target_center = self._bbox_center(target)
        target_world = target.get("world_position_xyz")
        target_depth = target.get("depth_m")
        for relation in self.task_spec.relations:
            best_score = 0.0
            best_reference: dict[str, Any] | None = None
            best_measurement: dict[str, Any] = {}
            for reference in candidates:
                if reference is target or reference.get("candidate_id") == target.get("candidate_id"):
                    continue
                category_score = self.task_spec.category_score(reference.get("category"), relation.reference)
                color_score = self._object_color_score(reference, relation.reference.color)
                if category_score < 0.50 or color_score < 0.30:
                    continue
                relation_score = 0.0
                measurement: dict[str, Any] = {}
                reference_world = reference.get("world_position_xyz")
                reference_center = self._bbox_center(reference)
                if relation.relation in {"near", "beside"} and target_world is not None and reference_world is not None:
                    distance = hypot(
                        float(target_world[0]) - float(reference_world[0]),
                        float(target_world[1]) - float(reference_world[1]),
                    )
                    limit = relation.maximum_distance_m or (10.0 if relation.relation == "beside" else 15.0)
                    relation_score = max(0.0, 1.0 - distance / max(0.1, limit))
                    measurement = {"distance_m": distance, "maximum_distance_m": limit}
                elif target_center is not None and reference_center is not None:
                    horizontal_gap = reference_center[0] - target_center[0]
                    if relation.relation == "left_of":
                        relation_score = 1.0 if horizontal_gap > 0.0 else 0.0
                    elif relation.relation == "right_of":
                        relation_score = 1.0 if horizontal_gap < 0.0 else 0.0
                    measurement = {"horizontal_gap_px": horizontal_gap, "coordinate_frame": "camera_image"}
                if relation.relation in {"in_front_of", "behind"}:
                    reference_depth = reference.get("depth_m")
                    if target_depth is not None and reference_depth is not None:
                        depth_gap = float(reference_depth) - float(target_depth)
                        relation_score = 1.0 if (
                            (relation.relation == "in_front_of" and depth_gap > 0.0)
                            or (relation.relation == "behind" and depth_gap < 0.0)
                        ) else 0.0
                        measurement = {"depth_gap_m": depth_gap, "coordinate_frame": "camera_depth"}
                combined = relation_score * (0.60 * category_score + 0.40 * color_score)
                if combined > best_score:
                    best_score = combined
                    best_reference = reference
                    best_measurement = measurement
            scores.append(best_score)
            evidence.append({
                "relation": relation.relation,
                "score": best_score,
                "reference_candidate_id": None if best_reference is None else best_reference.get("candidate_id"),
                **best_measurement,
            })
        return min(scores, default=1.0), evidence

    def select(self, candidates: list[dict[str, Any]]) -> dict[str, Any] | None:
        eligible: list[dict[str, Any]] = []
        for candidate in candidates:
            world = candidate.get("world_position_xyz")
            if world is None:
                continue
            if candidate.get("proposal_source") not in self.config.allowed_sources:
                continue
            attribute_score = self._attribute_score(candidate)
            if attribute_score < self.config.minimum_red_ratio:
                continue
            category_score = self.task_spec.category_score(candidate.get("category"))
            if category_score < self.config.minimum_category_score:
                continue
            if int(candidate.get("temporal_hits", 0)) < self.config.minimum_temporal_hits:
                continue
            if float(candidate.get("detector_score", 0.0)) < self.config.minimum_detector_score:
                continue
            if not self.config.minimum_world_z_m <= float(world[2]) <= self.config.maximum_world_z_m:
                continue
            relation_score, relation_evidence = self._relation_score(candidate, candidates)
            if relation_score < self.config.minimum_relation_score:
                continue
            item = dict(candidate)
            item["van_like_score"] = van_like_score(candidate)
            item["category_match_score"] = category_score
            item["target_attribute_score"] = attribute_score
            item["relation_match_score"] = relation_score
            item["relation_evidence"] = relation_evidence
            item["instruction_match_score"] = float(
                0.30 * attribute_score
                + 0.25 * float(candidate["detector_score"])
                + 0.20 * category_score
                + 0.10 * min(1.0, float(candidate["temporal_hits"]) / 5.0)
                + 0.15 * relation_score
            )
            item["track_id"] = "uav_track_{:+04d}_{:+04d}".format(
                int(round(float(world[0]) / 2.0)), int(round(float(world[1]) / 2.0))
            )
            eligible.append(item)
        return max(eligible, key=lambda item: item["instruction_match_score"], default=None)


class LocalTargetVerifier:
    def __init__(self, config: LocalVerificationConfig, task_spec: TaskSpec | None = None) -> None:
        self.config = config
        self.task_spec = task_spec or TaskSpec.legacy_red_van()

    def _attribute_score(self, candidate: dict[str, Any]) -> float:
        target_color = self.task_spec.target_color
        if target_color is None:
            return 1.0
        ratios = candidate.get("color_ratios")
        if isinstance(ratios, dict) and target_color in ratios:
            return float(ratios[target_color])
        if target_color == "red":
            if candidate.get("color") not in {None, "unknown", "red"}:
                return 0.0
            return float(candidate.get("red_pixel_ratio", 0.0))
        return float(candidate.get("color_score", 0.0)) if candidate.get("color") == target_color else 0.0

    def evaluate(
        self,
        frame: int,
        timestamp: float,
        candidates: list[dict[str, Any]],
        message_position_xyz: list[float],
    ) -> dict[str, Any]:
        ranked: list[tuple[float, dict[str, Any]]] = []
        for candidate in candidates:
            world = candidate.get("world_position_xyz")
            if world is None or candidate.get("proposal_source") not in self.config.allowed_sources:
                continue
            distance = hypot(
                float(world[0]) - float(message_position_xyz[0]),
                float(world[1]) - float(message_position_xyz[1]),
            )
            legacy_shape_score = van_like_score(candidate)
            category_score = self.task_spec.category_score(candidate.get("category"))
            semantic_score = max(category_score, legacy_shape_score if self.task_spec.target.category == "van" else 0.0)
            if (
                distance <= self.config.association_radius_m
                and self._attribute_score(candidate) >= self.config.minimum_red_ratio
                and int(candidate.get("temporal_hits", 0)) >= self.config.minimum_temporal_hits
                and float(candidate.get("detector_score", 0.0)) >= self.config.minimum_detector_score
                and semantic_score >= max(
                    self.config.minimum_category_score, self.config.minimum_van_like_score
                )
            ):
                ranked.append((distance, candidate))
        best_distance, best = min(ranked, key=lambda item: item[0]) if ranked else (None, None)
        return {
            "frame": int(frame),
            "timestamp": float(timestamp),
            "status": "CONFIRMED_TASK_TARGET" if best is not None else "PENDING",
            "confirmed": best is not None,
            "message_position_xyz": [float(value) for value in message_position_xyz],
            "matched_candidate_id": None if best is None else best["candidate_id"],
            "candidate_category": None if best is None else best["category"],
            "red_pixel_ratio": None if best is None else float(best["red_pixel_ratio"]),
            "van_like_score": None if best is None else van_like_score(best),
            "target_color": self.task_spec.target_color,
            "target_attribute_score": None if best is None else self._attribute_score(best),
            "category_match_score": None if best is None else self.task_spec.category_score(best.get("category")),
            "position_difference_m": best_distance,
            "method": "instruction_conditioned_detector+mask_attribute+depth+temporal_consistency",
        }
