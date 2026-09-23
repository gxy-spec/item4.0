"""Instruction-conditioned S1 candidate transmission and local verification gates."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Any


@dataclass(frozen=True)
class TransmissionGateConfig:
    minimum_red_ratio: float = 0.70
    minimum_temporal_hits: int = 4
    minimum_detector_score: float = 0.03
    allowed_sources: tuple[str, ...] = ("yolo26x_coco",)
    minimum_world_z_m: float = 0.4
    maximum_world_z_m: float = 4.5


@dataclass(frozen=True)
class LocalVerificationConfig:
    minimum_red_ratio: float = 0.35
    minimum_temporal_hits: int = 3
    minimum_detector_score: float = 0.10
    association_radius_m: float = 5.0
    minimum_van_like_score: float = 0.60


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
    def __init__(self, config: TransmissionGateConfig) -> None:
        self.config = config

    def select(self, candidates: list[dict[str, Any]]) -> dict[str, Any] | None:
        eligible: list[dict[str, Any]] = []
        for candidate in candidates:
            world = candidate.get("world_position_xyz")
            if world is None or candidate.get("color") != "red":
                continue
            if candidate.get("proposal_source") not in self.config.allowed_sources:
                continue
            if float(candidate.get("red_pixel_ratio", 0.0)) < self.config.minimum_red_ratio:
                continue
            if int(candidate.get("temporal_hits", 0)) < self.config.minimum_temporal_hits:
                continue
            if float(candidate.get("detector_score", 0.0)) < self.config.minimum_detector_score:
                continue
            if not self.config.minimum_world_z_m <= float(world[2]) <= self.config.maximum_world_z_m:
                continue
            item = dict(candidate)
            item["van_like_score"] = van_like_score(candidate)
            item["instruction_match_score"] = float(
                0.45 * float(candidate["red_pixel_ratio"])
                + 0.30 * float(candidate["detector_score"])
                + 0.15 * min(1.0, float(candidate["temporal_hits"]) / 5.0)
                + 0.10 * item["van_like_score"]
            )
            item["track_id"] = "uav_track_{:+04d}_{:+04d}".format(
                int(round(float(world[0]) / 2.0)), int(round(float(world[1]) / 2.0))
            )
            eligible.append(item)
        return max(eligible, key=lambda item: item["instruction_match_score"], default=None)


class LocalTargetVerifier:
    def __init__(self, config: LocalVerificationConfig) -> None:
        self.config = config

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
            if world is None or candidate.get("proposal_source") != "yolo26x_coco":
                continue
            distance = hypot(
                float(world[0]) - float(message_position_xyz[0]),
                float(world[1]) - float(message_position_xyz[1]),
            )
            score = van_like_score(candidate)
            if (
                distance <= self.config.association_radius_m
                and candidate.get("color") == "red"
                and float(candidate.get("red_pixel_ratio", 0.0)) >= self.config.minimum_red_ratio
                and int(candidate.get("temporal_hits", 0)) >= self.config.minimum_temporal_hits
                and float(candidate.get("detector_score", 0.0)) >= self.config.minimum_detector_score
                and score >= self.config.minimum_van_like_score
            ):
                ranked.append((distance, candidate))
        best_distance, best = min(ranked, key=lambda item: item[0]) if ranked else (None, None)
        return {
            "frame": int(frame),
            "timestamp": float(timestamp),
            "status": "CONFIRMED_RED_VAN_LIKE" if best is not None else "PENDING",
            "confirmed": best is not None,
            "message_position_xyz": [float(value) for value in message_position_xyz],
            "matched_candidate_id": None if best is None else best["candidate_id"],
            "candidate_category": None if best is None else best["category"],
            "red_pixel_ratio": None if best is None else float(best["red_pixel_ratio"]),
            "van_like_score": None if best is None else van_like_score(best),
            "position_difference_m": best_distance,
            "method": "generic_vehicle_detector+red_attribute+depth+temporal_consistency+transparent_van_like_heuristic",
        }
