#!/usr/bin/env python3
"""Create a compact visual audit of exported YOLO instance-segmentation labels."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from PIL import Image


PALETTE = ("#D1495B", "#00798C", "#EDAe49", "#30638E")


def read_records(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def choose_records(records: list[dict], count: int) -> list[dict]:
    selected: list[dict] = []
    for device in ("uav", "ugv"):
        candidates = [row for row in records if row["device"] == device]
        if not candidates:
            continue
        take = max(1, count // 2)
        indices = np.linspace(0, len(candidates) - 1, min(take, len(candidates)), dtype=int)
        selected.extend(candidates[index] for index in indices)
    return selected[:count]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--samples", type=int, default=12)
    args = parser.parse_args()
    manifest = args.dataset / "manifest.jsonl"
    records = choose_records(read_records(manifest), max(1, args.samples))
    if not records:
        raise RuntimeError(f"No exported records in {manifest}")

    columns = 3
    rows = int(np.ceil(len(records) / columns))
    fig, axes = plt.subplots(rows, columns, figsize=(15.5, 4.2 * rows), facecolor="white")
    axes = np.atleast_1d(axes).ravel()
    for axis, record in zip(axes, records):
        image = np.asarray(Image.open(record["image"]).convert("RGB"))
        height, width = image.shape[:2]
        label_lines = Path(record["label"]).read_text(encoding="utf-8").splitlines()
        descriptions: list[str] = []
        polygons: list[tuple[int, np.ndarray]] = []
        for annotation, line in zip(record["annotations"], label_lines):
            values = line.split()
            class_id = int(values[0])
            points = np.asarray([float(value) for value in values[1:]], dtype=float).reshape(-1, 2)
            points[:, 0] *= width
            points[:, 1] *= height
            polygons.append((class_id, points))
            descriptions.append(f"{annotation['category']} / {annotation['color']}")
        all_points = np.concatenate([points for _, points in polygons], axis=0)
        left, top = np.floor(all_points.min(axis=0)).astype(int)
        right, bottom = np.ceil(all_points.max(axis=0)).astype(int)
        span = max(right - left, bottom - top, 120)
        margin = int(0.45 * span)
        center_x, center_y = (left + right) // 2, (top + bottom) // 2
        half = max(100, span // 2 + margin)
        x0, x1 = max(0, center_x - half), min(width, center_x + half)
        y0, y1 = max(0, center_y - half), min(height, center_y + half)
        axis.imshow(image[y0:y1, x0:x1])
        for class_id, points in polygons:
            color = PALETTE[class_id % len(PALETTE)]
            shifted = points - np.asarray([x0, y0])
            axis.add_patch(Polygon(shifted, closed=True, facecolor=color, edgecolor=color, alpha=0.48, linewidth=2.2))
        axis.set_title(
            f"{record['device'].upper()} · frame {record['frame']}\n" + ", ".join(descriptions),
            fontsize=10,
            loc="left",
        )
        axis.set_axis_off()
    for axis in axes[len(records):]:
        axis.set_axis_off()
    fig.suptitle("CARLA Instance Segmentation Export Audit", fontsize=17, fontweight="bold", y=0.995)
    fig.text(0.5, 0.005, "Translucent regions are exported YOLO segmentation polygons.", ha="center", fontsize=10)
    fig.tight_layout(rect=(0, 0.02, 1, 0.98))
    output = args.output or args.dataset / "audit_montage.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
