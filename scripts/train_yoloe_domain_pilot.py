#!/usr/bin/env python3
"""Fine-tune YOLOE on the leakage-safe CARLA viewpoint pilot dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from ultralytics import YOLOE
from ultralytics.models.yolo.yoloe import YOLOEPESegTrainer


ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "models/yoloe-26s-seg.pt")
    parser.add_argument(
        "--data", type=Path,
        default=Path(r"E:\CarlaAirData\CityInspection_GOC\datasets\carla_red_van_pilot_v1\dataset.yaml"),
    )
    parser.add_argument(
        "--project", type=Path,
        default=Path(r"E:\CarlaAirData\CityInspection_GOC\training\yoloe_domain_pilot"),
    )
    parser.add_argument("--name", default="yoloe26s_carla_red_van_v1")
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--image-size", type=int, default=960)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", default="0")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
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
        patience=6,
        close_mosaic=5,
        seed=20261016,
        deterministic=True,
        plots=True,
        verbose=True,
        trainer=YOLOEPESegTrainer,
    )
    run_directory = Path(results.save_dir).resolve()
    payload = {
        "schema_version": "yoloe_domain_pilot_training_v1",
        "source_model": str(args.model.resolve()),
        "dataset": str(args.data.resolve()),
        "run_directory": str(run_directory),
        "best_weights": str(run_directory / "weights/best.pt"),
        "last_weights": str(run_directory / "weights/last.pt"),
        "epochs_requested": args.epochs,
        "image_size": args.image_size,
        "batch_size": args.batch_size,
        "scope": "red-van CARLA viewpoint adaptation pilot; not final multi-category training",
    }
    (run_directory / "training_manifest.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
