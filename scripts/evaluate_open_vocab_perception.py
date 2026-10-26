#!/usr/bin/env python3
"""Evaluate zero-shot YOLOE category and mask-color selection on frozen replays."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path
from statistics import median
from typing import Any

import cv2
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.perception.attributes import analyze_color  # noqa: E402
from src.perception.candidate_pipeline import CandidatePipeline, PipelineConfig  # noqa: E402
from src.perception.task_spec import TaskSpec  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest", type=Path,
        default=Path(r"E:\CarlaAirData\CityInspection_GOC\perception_benchmark_v1\test.jsonl"),
    )
    parser.add_argument("--model", type=Path, default=ROOT / "models/yoloe-26s-seg.pt")
    parser.add_argument("--backend", choices=("open_vocabulary", "coco"), default="open_vocabulary")
    parser.add_argument("--device-view", choices=("uav", "ugv", "all"), default="uav")
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument("--batch-size", type=int, default=12)
    parser.add_argument("--image-size", type=int, default=1280)
    parser.add_argument("--confidence", type=float, default=0.05)
    parser.add_argument(
        "--selection-confidence", type=float, default=None,
        help="Post-inference candidate threshold; defaults to --confidence.",
    )
    parser.add_argument("--target-color-threshold", type=float, default=0.10)
    parser.add_argument(
        "--prompts", nargs="+", default=("van",),
        help="Open-vocabulary text prompts; labels are still scored against TaskSpec.",
    )
    parser.add_argument("--minimum-target-area-px", type=float, default=36.0)
    parser.add_argument(
        "--output", type=Path,
        default=Path(r"E:\CarlaAirData\CityInspection_GOC\perception_benchmark_v1\results\yoloe26s_zero_shot_uav"),
    )
    return parser.parse_args()


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def balanced_subset(rows: list[dict[str, Any]], limit: int) -> list[dict[str, Any]]:
    if limit <= 0 or len(rows) <= limit:
        return rows
    groups: dict[tuple[str, str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[(row["scene"], row["policy"], row["device"])].append(row)
    for values in groups.values():
        values.sort(key=lambda item: item["frame"])
    selected: list[dict[str, Any]] = []
    positions = {key: 0 for key in groups}
    keys = sorted(groups)
    while len(selected) < limit:
        progressed = False
        for key in keys:
            values = groups[key]
            index = positions[key]
            if index >= len(values):
                continue
            stride = max(1, len(values) // max(1, limit // len(keys)))
            selected.append(values[min(index, len(values) - 1)])
            positions[key] += stride
            progressed = True
            if len(selected) >= limit:
                break
        if not progressed:
            break
    return sorted(selected, key=lambda item: (item["scene"], item["policy"], item["frame"]))


def task_spec_from_manifest(row: dict[str, Any]) -> TaskSpec:
    saved = row["task_spec"]
    return TaskSpec.from_task_config({
        "task_id": saved["task_id"],
        "instruction": saved["instruction"],
        "language": saved["language"],
        "parsed_goal": {
            "target": saved["target"],
            "relations": saved.get("relations", []),
            "action": saved.get("action", "approach"),
        },
    })


def to_list(value: Any) -> list[float]:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "tolist"):
        value = value.tolist()
    return [float(item) for item in value]


def clipped_area(box: list[float] | None, width: int = 800, height: int = 600) -> float:
    if box is None:
        return 0.0
    x1, y1, x2, y2 = box
    return max(0.0, min(width, x2) - max(0.0, x1)) * max(0.0, min(height, y2) - max(0.0, y1))


def point_in_box(point: list[float], box: list[float]) -> bool:
    return box[0] <= point[0] <= box[2] and box[1] <= point[1] <= box[3]


def evaluate_detection_result(
    row: dict[str, Any], image: np.ndarray, result: Any, pipeline: CandidatePipeline,
    task_spec: TaskSpec, color_threshold: float, selection_confidence: float, minimum_area: float,
) -> dict[str, Any]:
    predictions: list[dict[str, Any]] = []
    boxes = getattr(result, "boxes", None)
    if boxes is not None:
        for index, (box, confidence, class_id) in enumerate(zip(boxes.xyxy, boxes.conf, boxes.cls)):
            bbox = to_list(box)
            instance_mask = pipeline._instance_mask(result, index, image.shape[:2])
            evidence = analyze_color(image, bbox, instance_mask)
            category = str(result.names[int(class_id)])
            category_score = task_spec.category_score(category)
            color_score = evidence.score(task_spec.target_color)
            predictions.append({
                "bbox_xyxy": bbox,
                "category": category,
                "detector_score": float(confidence),
                "category_match_score": category_score,
                "dominant_color": evidence.dominant_color,
                "target_color_ratio": color_score,
                "target_candidate": bool(
                    category_score >= 0.5
                    and float(confidence) >= selection_confidence
                    and color_score >= color_threshold
                ),
                "mask_area_px": None if instance_mask is None else int(instance_mask.sum()),
                "selection_score": float(confidence) * (0.65 + 0.35 * category_score) * (0.5 + 0.5 * color_score),
            })

    target_box = row.get("target_projected_bbox_xyxy")
    centre = row["target_projected_center_uv"]
    evaluable = bool(
        row.get("target_visible_depth_proxy") is True
        and clipped_area(target_box) >= minimum_area
    )
    negative_frame = bool(
        row.get("target_visible_depth_proxy") is False
        or not row["target_projected_in_frame"]
    )
    for prediction in predictions:
        prediction["contains_target_center"] = point_in_box(centre, prediction["bbox_xyxy"])
    category_hit = any(
        item["category_match_score"] >= 0.5 and item["contains_target_center"] for item in predictions
    )
    attribute_hit = any(
        item["target_candidate"] and item["contains_target_center"] for item in predictions
    )
    candidates = [item for item in predictions if item["target_candidate"]]
    selected = max(candidates, key=lambda item: item["selection_score"], default=None)
    selected_correct = bool(selected and selected["contains_target_center"] and evaluable)
    speed = getattr(result, "speed", {}) or {}
    latency = float(sum(float(speed.get(key, 0.0)) for key in ("preprocess", "inference", "postprocess")))
    return {
        "sample_id": row["sample_id"],
        "scene": row["scene"],
        "policy": row["policy"],
        "device": row["device"],
        "frame": row["frame"],
        "image_path": row["image_path"],
        "target_projected_center_uv": centre,
        "target_projected_bbox_xyxy": target_box,
        "target_area_px": clipped_area(target_box),
        "evaluable_target": evaluable,
        "negative_frame": negative_frame,
        "prediction_count": len(predictions),
        "target_candidate_count": len(candidates),
        "category_hit": bool(category_hit and evaluable),
        "attribute_hit": bool(attribute_hit and evaluable),
        "selected": selected is not None,
        "selected_correct": selected_correct,
        "selected_bbox_xyxy": None if selected is None else selected["bbox_xyxy"],
        "selected_score": None if selected is None else selected["selection_score"],
        "selected_color_ratio": None if selected is None else selected["target_color_ratio"],
        "latency_ms": latency,
        "predictions": predictions,
    }


def build_summary(rows: list[dict[str, Any]], args: argparse.Namespace, task_spec: TaskSpec) -> dict[str, Any]:
    evaluable = [item for item in rows if item["evaluable_target"]]
    negative = [item for item in rows if item["negative_frame"]]
    selected = [item for item in rows if item["selected"]]
    correct = [item for item in rows if item["selected_correct"]]
    latencies = [item["latency_ms"] for item in rows]
    return {
        "schema_version": "open_vocab_evaluation_v1",
        "model": str(args.model.resolve()),
        "manifest": str(args.manifest.resolve()),
        "task_spec": task_spec.to_dict(),
        "device_view": args.device_view,
        "image_size": args.image_size,
        "detector_confidence": args.confidence,
        "selection_confidence": (
            args.confidence if args.selection_confidence is None else args.selection_confidence
        ),
        "backend": args.backend,
        "target_color_threshold": args.target_color_threshold,
        "detector_prompts": list(args.prompts),
        "frames_evaluated": len(rows),
        "evaluable_target_frames": len(evaluable),
        "negative_frames": len(negative),
        "unknown_visibility_frames": sum(
            not item["evaluable_target"] and not item["negative_frame"] for item in rows
        ),
        "category_recall": sum(item["category_hit"] for item in evaluable) / max(1, len(evaluable)),
        "attribute_conditioned_recall": sum(item["attribute_hit"] for item in evaluable) / max(1, len(evaluable)),
        "selection_recall": len(correct) / max(1, len(evaluable)),
        "selection_frame_precision": len(correct) / max(1, len(selected)),
        "negative_false_trigger_rate": sum(item["selected"] for item in negative) / max(1, len(negative)),
        "median_latency_ms": median(latencies) if latencies else None,
        "p90_latency_ms": float(np.percentile(latencies, 90)) if latencies else None,
        "mean_predictions_per_frame": float(np.mean([item["prediction_count"] for item in rows])),
        "mean_target_candidates_per_frame": float(np.mean([item["target_candidate_count"] for item in rows])),
        "evaluation_note": (
            "Target-centre containment uses projected CARLA actor state. Frozen data has no metric-depth files, "
            "so occlusion and final RGB-D localization are evaluated separately in live/replay runs."
        ),
    }


def save_summary_figure(summary: dict[str, Any], output: Path) -> None:
    plt.rcParams.update({"font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9})
    fig, axes = plt.subplots(1, 3, figsize=(10.8, 3.2), constrained_layout=True)
    metrics = ["category_recall", "attribute_conditioned_recall", "selection_recall"]
    labels = ["Category", "Category + color", "Final selection"]
    values = [summary[key] for key in metrics]
    bars = axes[0].bar(labels, values, color=["#4C78A8", "#59A14F", "#E15759"], width=0.65)
    axes[0].bar_label(bars, labels=[f"{value:.2f}" for value in values], padding=2)
    axes[0].set(title="A. Recall on projected target frames", ylabel="Recall", ylim=(0, 1.08))

    precision_values = [summary["selection_frame_precision"], 1.0 - summary["negative_false_trigger_rate"]]
    bars = axes[1].bar(["Selection precision", "Negative-frame rejection"], precision_values,
                       color=["#F28E2B", "#76B7B2"], width=0.62)
    axes[1].bar_label(bars, labels=[f"{value:.2f}" for value in precision_values], padding=2)
    axes[1].set(title="B. Candidate reliability", ylim=(0, 1.08))

    latency_values = [summary["median_latency_ms"], summary["p90_latency_ms"]]
    bars = axes[2].bar(["Median", "P90"], latency_values, color=["#4C78A8", "#B279A2"], width=0.62)
    axes[2].bar_label(bars, labels=[f"{value:.0f}" for value in latency_values], padding=2)
    axes[2].set(title="C. End-to-end model latency", ylabel="Milliseconds")
    for axis in axes:
        axis.grid(axis="y", alpha=0.2, linewidth=0.6)
        axis.spines[["top", "right"]].set_visible(False)
        axis.tick_params(axis="x", rotation=10)
    fig.suptitle("YOLOE-26s zero-shot perception benchmark", fontsize=12, fontweight="semibold")
    fig.savefig(output, dpi=240, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def save_examples(rows: list[dict[str, Any]], output: Path) -> None:
    failures = [item for item in rows if item["evaluable_target"] and not item["selected_correct"]]
    successes = [item for item in rows if item["selected_correct"]]
    examples = failures[:6] + successes[:6]
    if not examples:
        return
    fig, axes = plt.subplots(3, 4, figsize=(13.2, 9.2), constrained_layout=True)
    for axis, item in zip(axes.flat, examples):
        image = cv2.imread(item["image_path"], cv2.IMREAD_COLOR)
        if image is None:
            axis.axis("off")
            continue
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        axis.imshow(image)
        gt = item["target_projected_bbox_xyxy"]
        if gt:
            x1, y1, x2, y2 = gt
            axis.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False,
                                         edgecolor="#00A6D6", linewidth=1.4, linestyle="--"))
        for prediction in item["predictions"]:
            if not prediction["target_candidate"]:
                continue
            x1, y1, x2, y2 = prediction["bbox_xyxy"]
            color = "#2CA02C" if prediction["contains_target_center"] else "#D62728"
            axis.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False,
                                         edgecolor=color, linewidth=1.5))
        status = "correct" if item["selected_correct"] else "miss"
        axis.set_title(f"{item['scene']}/{item['policy']} · {item['device']} · {status}", fontsize=9)
        axis.axis("off")
    for axis in axes.flat[len(examples):]:
        axis.axis("off")
    fig.suptitle("Projected target (cyan dashed) and instruction-matched candidates", fontsize=12,
                 fontweight="semibold")
    fig.savefig(output, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main() -> int:
    args = parse_args()
    rows = load_jsonl(args.manifest.resolve())
    if args.device_view != "all":
        rows = [item for item in rows if item["device"] == args.device_view]
    rows = balanced_subset(rows, args.max_samples)
    if not rows:
        raise RuntimeError("no evaluation samples after filtering")
    task_spec = task_spec_from_manifest(rows[0])
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    pipeline = CandidatePipeline(PipelineConfig(
        model_path=args.model.resolve(),
        detector_confidence=args.confidence,
        detector_image_size=args.image_size,
        detector_backend=args.backend,
        task_spec=task_spec,
        enable_attribute_proposals=False,
        detector_prompts=tuple(args.prompts),
    ))
    evaluations: list[dict[str, Any]] = []
    for start in range(0, len(rows), args.batch_size):
        batch = rows[start:start + args.batch_size]
        images = [cv2.imread(item["image_path"], cv2.IMREAD_COLOR) for item in batch]
        if any(image is None for image in images):
            raise FileNotFoundError("one or more benchmark images could not be read")
        results = pipeline.infer_images(images)
        for row, image, result in zip(batch, images, results):
            evaluations.append(evaluate_detection_result(
                row, image, result, pipeline, task_spec,
                args.target_color_threshold,
                args.confidence if args.selection_confidence is None else args.selection_confidence,
                args.minimum_target_area_px,
            ))
        print(f"Processed {min(start + args.batch_size, len(rows))}/{len(rows)}", flush=True)

    summary = build_summary(evaluations, args, task_spec)
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    with (output / "frame_metrics.jsonl").open("w", encoding="utf-8") as handle:
        for item in evaluations:
            handle.write(json.dumps(item, ensure_ascii=False) + "\n")
    csv_fields = [key for key in evaluations[0] if key not in {"predictions"}]
    with (output / "frame_metrics.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=csv_fields)
        writer.writeheader()
        writer.writerows({key: item[key] for key in csv_fields} for item in evaluations)
    save_summary_figure(summary, output / "benchmark_summary.png")
    save_examples(evaluations, output / "qualitative_examples.png")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
