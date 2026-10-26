#!/usr/bin/env python3
"""Evaluate a YOLOE segmentation model on a frozen YOLO dataset split."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml
import numpy as np
from ultralytics import YOLOE


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "models/yoloe-26s-seg.pt")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--split", choices=("val", "test"), default="val")
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--image-size", type=int, default=960)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()

    dataset = yaml.safe_load(args.data.read_text(encoding="utf-8"))
    raw_names = dataset["names"]
    if isinstance(raw_names, dict):
        names = [raw_names[key] for key in sorted(raw_names, key=lambda value: int(value))]
    else:
        names = list(raw_names)
    model = YOLOE(str(args.model.resolve()))
    fixed_head = bool(getattr(model.model.model[-1], "is_fused", False))
    if fixed_head:
        existing = model.names
        existing_names = list(existing.values()) if isinstance(existing, dict) else list(existing)
        if existing_names != names:
            raise ValueError(f"Fixed model classes {existing_names} do not match dataset {names}")
    else:
        model.set_classes(names)
    # Initialize CUDA and NMS before timed validation; cold kernels can exceed
    # the per-batch NMS deadline and silently discard predictions in that batch.
    for _ in range(2):
        model.predict(
            np.zeros((args.image_size, args.image_size, 3), dtype=np.uint8),
            imgsz=args.image_size, device=args.device, verbose=False,
        )
    metrics = model.val(
        data=str(args.data.resolve()),
        split=args.split,
        imgsz=args.image_size,
        batch=args.batch_size,
        device=args.device,
        project=str(args.project.resolve()),
        name=args.name,
        exist_ok=False,
        conf=0.001,
        iou=0.7,
        agnostic_nms=False,
        rect=True,
        plots=True,
        verbose=True,
    )
    save_dir = Path(metrics.save_dir).resolve()
    payload = {
        "schema_version": "yoloe_segmentation_evaluation_v1",
        "model": str(args.model.resolve()),
        "dataset": str(args.data.resolve()),
        "split": args.split,
        "classes": names,
        "fixed_class_head": fixed_head,
        "protocol": {"confidence": 0.001, "iou_nms": 0.7, "agnostic_nms": False, "rect": True},
        "image_size": args.image_size,
        "batch_size": args.batch_size,
        "save_dir": str(save_dir),
        "metrics": {key: float(value) for key, value in metrics.results_dict.items()},
        "speed_ms_per_image": {key: float(value) for key, value in metrics.speed.items()},
    }
    (save_dir / "evaluation_manifest.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
