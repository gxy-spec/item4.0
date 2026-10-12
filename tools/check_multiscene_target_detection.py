"""Supplementary one-frame UAV detector smoke, not a formal perception score."""

from __future__ import annotations

import json
from pathlib import Path

from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]
OUT = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009")


def iou(a: list[float], b: list[float]) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    return intersection / (area_a + area_b - intersection) if area_a + area_b > intersection else 0


def main() -> None:
    cases = json.loads((OUT / "uav_capture_check.json").read_text(encoding="utf-8"))
    rows = []
    for model_name, image_size, confidence in (("yolo26n", 640, 0.08),
                                                ("yolo26x", 1280, 0.03)):
        model = YOLO(str(ROOT / "models" / f"{model_name}.pt"))
        for case in cases:
            response = model.predict(case["image"], imgsz=image_size, conf=confidence,
                                     classes=[2, 3, 5, 7], device=0, verbose=False)[0]
            boxes = []
            for item in response.boxes:
                box = [float(x) for x in item.xyxy[0].tolist()]
                boxes.append({"xyxy": box, "score": float(item.conf[0]),
                              "class_id": int(item.cls[0]),
                              "iou_to_target": iou(box, case["target_bbox_xyxy_px"])})
            best = max((x["iou_to_target"] for x in boxes), default=0.0)
            rows.append({"scene": case["scene"], "candidate": case["candidate"],
                         "model": model_name, "image_size": image_size,
                         "confidence": confidence, "vehicle_proposals": len(boxes),
                         "best_target_iou": round(best, 3),
                         "target_detected_iou_ge_0_3": best >= 0.3, "boxes": boxes})
            print(model_name, case["scene"], len(boxes), round(best, 3), flush=True)
    (OUT / "one_frame_detector_smoke.json").write_text(json.dumps(rows, ensure_ascii=False,
                                                                indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
