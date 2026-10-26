"""Interpretable visual-attribute measurements inside an object mask."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class ColorEvidence:
    dominant_color: str
    confidence: float
    ratios: dict[str, float]
    valid_pixel_count: int

    def score(self, color: str | None) -> float:
        if not color:
            return 1.0
        return float(self.ratios.get(color, 0.0))


def _selection_mask(
    image_shape: tuple[int, int],
    bbox_xyxy: list[float],
    instance_mask: np.ndarray | None,
) -> np.ndarray:
    height, width = image_shape
    selected = np.zeros((height, width), dtype=np.uint8)
    if instance_mask is not None:
        mask = np.asarray(instance_mask, dtype=np.uint8)
        if mask.shape != selected.shape:
            mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
        if mask.any():
            kernel = np.ones((3, 3), dtype=np.uint8)
            eroded = cv2.erode(mask, kernel, iterations=1)
            return eroded if eroded.any() else mask

    x1, y1, x2, y2 = map(int, bbox_xyxy)
    x1, x2 = max(0, x1), min(width, x2)
    y1, y2 = max(0, y1), min(height, y2)
    dx, dy = max(1, int((x2 - x1) * 0.12)), max(1, int((y2 - y1) * 0.12))
    selected[min(height, y1 + dy) : max(0, y2 - dy), min(width, x1 + dx) : max(0, x2 - dx)] = 1
    return selected


def analyze_color(
    image_bgr: np.ndarray,
    bbox_xyxy: list[float],
    instance_mask: np.ndarray | None = None,
) -> ColorEvidence:
    """Return auditable HSV color evidence for a box or instance mask.

    Ratios use every selected object pixel as the denominator, so a small red
    tail light cannot obtain a high red score on a blue or gray vehicle.
    """
    selected = _selection_mask(image_bgr.shape[:2], bbox_xyxy, instance_mask).astype(bool)
    pixel_count = int(selected.sum())
    if pixel_count < 9:
        return ColorEvidence("unknown", 0.0, {}, pixel_count)

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = (channel[selected] for channel in cv2.split(hsv))
    masks = {
        "red": (((h <= 12) | (h >= 168)) & (s >= 70) & (v >= 45)),
        "orange": ((h >= 13) & (h <= 24) & (s >= 65) & (v >= 55)),
        "yellow": ((h >= 25) & (h <= 38) & (s >= 55) & (v >= 65)),
        "green": ((h >= 39) & (h <= 88) & (s >= 45) & (v >= 40)),
        "blue": ((h >= 89) & (h <= 132) & (s >= 50) & (v >= 40)),
        "purple": ((h >= 133) & (h <= 167) & (s >= 45) & (v >= 40)),
        "black": ((v <= 55) & (s <= 150)),
        "white": ((s <= 42) & (v >= 185)),
        "gray": ((s <= 48) & (v > 55) & (v < 185)),
    }
    ratios = {name: float(mask.mean()) for name, mask in masks.items()}
    dominant = max(ratios, key=ratios.get)
    confidence = ratios[dominant]
    if confidence < 0.08:
        dominant = "unknown"
    return ColorEvidence(dominant, float(confidence), ratios, pixel_count)


def color_binary_mask(image_bgr: np.ndarray, color: str) -> np.ndarray:
    """Create the same HSV color mask used for fallback proposal generation."""
    height, width = image_bgr.shape[:2]
    evidence_masks = np.zeros((height, width), dtype=np.uint8)
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    predicates = {
        "red": ((h <= 12) | (h >= 168)) & (s >= 70) & (v >= 45),
        "orange": (h >= 13) & (h <= 24) & (s >= 65) & (v >= 55),
        "yellow": (h >= 25) & (h <= 38) & (s >= 55) & (v >= 65),
        "green": (h >= 39) & (h <= 88) & (s >= 45) & (v >= 40),
        "blue": (h >= 89) & (h <= 132) & (s >= 50) & (v >= 40),
        "purple": (h >= 133) & (h <= 167) & (s >= 45) & (v >= 40),
        "black": (v <= 55) & (s <= 150),
        "white": (s <= 42) & (v >= 185),
        "gray": (s <= 48) & (v > 55) & (v < 185),
    }
    predicate = predicates.get(color)
    if predicate is not None:
        evidence_masks[predicate] = 255
    return evidence_masks
