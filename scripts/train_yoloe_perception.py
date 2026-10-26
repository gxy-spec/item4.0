#!/usr/bin/env python3
"""Fine-tune YOLOE segmentation on the scene-split multi-category CARLA dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml
from ultralytics import YOLOE
from ultralytics.models.yolo.yoloe import YOLOEPESegTrainer


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "models/yoloe-26s-seg.pt")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", default="0")
    parser.add_argument("--patience", type=int, default=8)
    args = parser.parse_args()

    dataset = yaml.safe_load(args.data.read_text(encoding="utf-8"))
    raw_names = dataset["names"]
    names = (
        [raw_names[key] for key in sorted(raw_names, key=lambda value: int(value))]
        if isinstance(raw_names, dict)
        else list(raw_names)
    )
    args.project.mkdir(parents=True, exist_ok=True)
    model = YOLOE(str(args.model.resolve()))
    results = model.train(
        data=str(args.data.resolve()),
        epochs=args.epochs,
        imgsz=args.image_size,
        batch=args.batch_size,
        workers=args.workers,
        device=args.device,
        project=str(args.project.resolve()),
        name=args.name,
        exist_ok=False,
        pretrained=True,
        optimizer="AdamW",
        lr0=0.001,
        weight_decay=0.0005,
        warmup_epochs=2.0,
        patience=args.patience,
        close_mosaic=5,
        seed=20261016,
        deterministic=True,
        plots=True,
        verbose=True,
        trainer=YOLOEPESegTrainer,
    )
    run_directory = Path(results.save_dir).resolve()
    payload = {
        "schema_version": "yoloe_perception_training_v1",
        "source_model": str(args.model.resolve()),
        "dataset": str(args.data.resolve()),
        "classes": names,
        "run_directory": str(run_directory),
        "best_weights": str(run_directory / "weights/best.pt"),
        "last_weights": str(run_directory / "weights/last.pt"),
        "epochs_requested": args.epochs,
        "image_size": args.image_size,
        "batch_size": args.batch_size,
        "scope": "Town10HD multi-category domain adaptation; physical-scene-disjoint validation",
    }
    (run_directory / "training_manifest.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
