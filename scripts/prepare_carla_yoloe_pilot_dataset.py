#!/usr/bin/env python3
"""Materialize a leakage-safe CARLA pilot segmentation dataset for YOLOE.

This pilot deliberately tests whether CARLA aerial/ground viewpoint adaptation
helps the detector.  It is not the final multi-category dataset: only the
frozen red-van task has trustworthy automatic mask cues in existing replays.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import shutil
from collections import Counter
from pathlib import Path
from typing import Any

import cv2
import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--benchmark-root", type=Path,
        default=Path(r"E:\CarlaAirData\CityInspection_GOC\perception_benchmark_v1"),
    )
    parser.add_argument(
        "--output", type=Path,
        default=Path(r"E:\CarlaAirData\CityInspection_GOC\datasets\carla_red_van_pilot_v1"),
    )
    parser.add_argument("--negative-ratio", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=20261016)
    return parser.parse_args()


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def target_polygon(image: np.ndarray, row: dict[str, Any]) -> np.ndarray | None:
    box = row.get("target_projected_bbox_xyxy")
    if box is None:
        return None
    height, width = image.shape[:2]
    x1, y1, x2, y2 = map(float, box)
    pad_x, pad_y = 0.12 * (x2 - x1), 0.12 * (y2 - y1)
    x1, x2 = int(max(0, x1 - pad_x)), int(min(width, x2 + pad_x))
    y1, y2 = int(max(0, y1 - pad_y)), int(min(height, y2 + pad_y))
    if x2 <= x1 or y2 <= y1:
        return None
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red = ((((hsv[..., 0] <= 12) | (hsv[..., 0] >= 168))
            & (hsv[..., 1] >= 65) & (hsv[..., 2] >= 40))).astype(np.uint8)
    region = np.zeros_like(red)
    region[y1:y2, x1:x2] = 1
    red &= region
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=1)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(red, 8)
    if count <= 1:
        return None
    u, v = map(lambda value: int(round(value)), row["target_projected_center_uv"])
    chosen = int(labels[v, u]) if 0 <= v < height and 0 <= u < width else 0
    if chosen == 0:
        candidates = [index for index in range(1, count) if int(stats[index, cv2.CC_STAT_AREA]) >= 8]
        if not candidates:
            return None
        chosen = max(candidates, key=lambda index: int(stats[index, cv2.CC_STAT_AREA]))
    component = (labels == chosen).astype(np.uint8)
    if int(component.sum()) < 8:
        return None
    contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    epsilon = max(0.75, 0.01 * cv2.arcLength(contour, True))
    polygon = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
    return polygon if len(polygon) >= 3 else None


def yolo_polygon(polygon: np.ndarray, width: int, height: int) -> str:
    coords = []
    for x, y in polygon:
        coords.extend((np.clip(float(x) / width, 0.0, 1.0), np.clip(float(y) / height, 0.0, 1.0)))
    return "0 " + " ".join(f"{value:.6f}" for value in coords)


def link_or_copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, destination)
    except OSError:
        shutil.copy2(source, destination)


def save_audit(examples: list[tuple[Path, np.ndarray]], path: Path) -> None:
    if not examples:
        return
    fig, axes = plt.subplots(3, 4, figsize=(12.8, 9.0), constrained_layout=True)
    for axis, (image_path, polygon) in zip(axes.flat, examples[:12]):
        image = cv2.cvtColor(cv2.imread(str(image_path)), cv2.COLOR_BGR2RGB)
        axis.imshow(image)
        closed = np.vstack((polygon, polygon[0]))
        axis.plot(closed[:, 0], closed[:, 1], color="#00A6D6", linewidth=1.5)
        axis.set_title(image_path.stem, fontsize=8)
        axis.axis("off")
    for axis in axes.flat[len(examples[:12]):]:
        axis.axis("off")
    fig.suptitle("CARLA red-van pilot masks (automatic HSV + projected actor prior)",
                 fontsize=12, fontweight="semibold")
    fig.savefig(path, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main() -> int:
    args = parse_args()
    rng = random.Random(args.seed)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    counts: Counter[str] = Counter()
    audit_examples: list[tuple[Path, np.ndarray]] = []

    for split in ("train", "validation", "test"):
        rows = load_jsonl(args.benchmark_root / f"{split}.jsonl")
        positives: list[tuple[dict[str, Any], np.ndarray]] = []
        negatives: list[dict[str, Any]] = []
        for row in rows:
            image_path = Path(row["image_path"])
            if row.get("target_visible_depth_proxy") is True:
                image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
                polygon = None if image is None else target_polygon(image, row)
                if polygon is not None:
                    positives.append((row, polygon))
                else:
                    counts[f"{split}_positive_mask_failed"] += 1
            elif row.get("target_visible_depth_proxy") is False:
                negatives.append(row)

        rng.shuffle(negatives)
        negatives = negatives[: int(round(len(positives) * args.negative_ratio))]
        selected: list[tuple[dict[str, Any], np.ndarray | None]] = positives + [
            (row, None) for row in negatives
        ]
        rng.shuffle(selected)
        for row, polygon in selected:
            source = Path(row["image_path"])
            filename = f"{row['sample_id']}{source.suffix.lower()}"
            destination = output / "images" / split / filename
            label_path = output / "labels" / split / f"{Path(filename).stem}.txt"
            link_or_copy(source, destination)
            label_path.parent.mkdir(parents=True, exist_ok=True)
            if polygon is None:
                label_path.write_text("", encoding="utf-8")
                counts[f"{split}_negative"] += 1
            else:
                image = cv2.imread(str(source), cv2.IMREAD_COLOR)
                label_path.write_text(
                    yolo_polygon(polygon, image.shape[1], image.shape[0]) + "\n", encoding="utf-8"
                )
                counts[f"{split}_positive"] += 1
                if len(audit_examples) < 12 and split == "train":
                    audit_examples.append((source, polygon))

    dataset_yaml = f"""path: {output.as_posix()}
train: images/train
val: images/validation
test: images/test
names:
  0: van
"""
    (output / "dataset.yaml").write_text(dataset_yaml, encoding="utf-8")
    summary = {
        "schema_version": "carla_yoloe_pilot_dataset_v1",
        "output": str(output),
        "source_benchmark": str(args.benchmark_root.resolve()),
        "purpose": "CARLA viewpoint domain-adaptation pilot; not the final open-vocabulary dataset",
        "split_rule": "scene-number grouped split inherited from perception_benchmark_v1",
        "automatic_mask_method": "projected target cuboid prior plus HSV red connected component",
        "counts": dict(counts),
    }
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    save_audit(audit_examples, output / "mask_audit.png")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
