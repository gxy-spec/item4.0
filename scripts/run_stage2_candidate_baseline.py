#!/usr/bin/env python
"""Run CI-E2 UAV vehicle/color/depth candidate inference and Oracle evaluation."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from statistics import median
from typing import Any

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.perception.candidate_pipeline import CandidatePipeline, PipelineConfig, project_world_point


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/experiments/ci_e2_uav_candidate_baseline.yaml")
    parser.add_argument("--input-run", type=Path)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--max-frames", type=int, default=0)
    return parser.parse_args()


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def json_dump(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def draw_candidate_frame(image: np.ndarray, candidates: list[dict[str, Any]], frame_eval: dict[str, Any]) -> np.ndarray:
    canvas = image.copy()
    for candidate in candidates:
        x1, y1, x2, y2 = map(int, candidate["bbox_xyxy"])
        oracle_match = candidate["candidate_id"] == frame_eval.get("oracle_target_match_id")
        red = candidate["color"] == "red"
        color = (30, 190, 45) if oracle_match else ((40, 40, 235) if red else (20, 185, 240))
        thickness = 4 if oracle_match else (3 if candidate["temporal_confirmed"] else 2)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)
        prefix = "ORACLE MATCH | " if oracle_match else ""
        label = prefix + f"{candidate['category']} {candidate['detector_score']:.2f} | {candidate['color']} {candidate['color_score']:.2f}"
        if candidate["depth_m"] is not None:
            label += f" | {candidate['depth_m']:.1f}m"
        if candidate["temporal_confirmed"]:
            label += " | CONFIRMED"
        cv2.putText(canvas, label, (x1, max(18, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    status = "TARGET MATCH" if frame_eval["target_detected"] else ("TARGET VISIBLE" if frame_eval["target_visible"] else "TARGET OUT OF VIEW")
    cv2.rectangle(canvas, (0, 0), (800, 35), (12, 19, 33), -1)
    cv2.putText(canvas, f"Frame {frame_eval['frame']}  {frame_eval['timestamp']:.2f}s  {status}", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (245, 245, 245), 2, cv2.LINE_AA)
    return canvas


def depth_panel(depth: np.ndarray, frame_eval: dict[str, Any]) -> np.ndarray:
    display = np.clip(depth, 0.0, 100.0)
    display = cv2.normalize(display, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    display = cv2.applyColorMap(255 - display, cv2.COLORMAP_TURBO)
    display = cv2.resize(display, (800, 300), interpolation=cv2.INTER_AREA)
    cv2.rectangle(display, (0, 0), (800, 30), (12, 19, 33), -1)
    error = frame_eval["localization_error_m"]
    detail = f"UAV metric depth | localization error: {error:.2f} m" if error is not None else "UAV metric depth | no matched target candidate"
    cv2.putText(display, detail, (12, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (245, 245, 245), 1, cv2.LINE_AA)
    return display


def map_panel(
    candidates: list[dict[str, Any]],
    target: dict[str, Any],
    uav: dict[str, Any],
    boundary: list[list[float]],
    frame_eval: dict[str, Any],
) -> np.ndarray:
    panel = np.full((300, 800, 3), 247, dtype=np.uint8)
    xs, ys = [p[0] for p in boundary], [p[1] for p in boundary]
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)

    def point(x: float, y: float) -> tuple[int, int]:
        px = int(40 + (x - xmin) / (xmax - xmin) * 720)
        py = int(270 - (y - ymin) / (ymax - ymin) * 240)
        return px, py

    cv2.rectangle(panel, (40, 30), (760, 270), (180, 187, 198), 2)
    tx, ty = point(target["x"], target["y"])
    ux, uy = point(uav["x"], uav["y"])
    cv2.drawMarker(panel, (tx, ty), (110, 110, 110), cv2.MARKER_TILTED_CROSS, 14, 2)
    cv2.circle(panel, (ux, uy), 6, (220, 110, 30), -1)
    for candidate in candidates:
        world = candidate["world_position_xyz"]
        if world is None:
            continue
        cx, cy = point(world[0], world[1])
        oracle_match = candidate["candidate_id"] == frame_eval.get("oracle_target_match_id")
        color = (30, 180, 40) if oracle_match else ((30, 35, 225) if candidate["color"] == "red" else (20, 170, 230))
        cv2.circle(panel, (cx, cy), 7 if oracle_match else 5, color, -1)
    cv2.putText(panel, "WORLD MAP (GT target is evaluation-only)", (50, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 27, 42), 1, cv2.LINE_AA)
    cv2.putText(panel, "UAV", (ux + 8, uy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 60, 10), 1, cv2.LINE_AA)
    cv2.putText(panel, "GT target", (tx + 9, ty + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1, cv2.LINE_AA)
    cv2.putText(panel, f"red candidates={frame_eval['red_candidate_count']}  confirmed={frame_eval['confirmed_red_count']}", (500, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20, 27, 42), 1, cv2.LINE_AA)
    return panel


def save_metrics_figure(rows: list[dict[str, Any]], path: Path) -> None:
    time = np.asarray([row["timestamp"] - rows[0]["timestamp"] for row in rows])
    detected = np.asarray([row["target_detected"] for row in rows], dtype=float)
    visible = np.asarray([row["target_visible"] for row in rows], dtype=float)
    error = np.asarray([np.nan if row["localization_error_m"] is None else row["localization_error_m"] for row in rows])
    confidence = np.asarray([np.nan if row["matched_candidate_score"] is None else row["matched_candidate_score"] for row in rows])
    red_score = np.asarray([np.nan if row["matched_red_score"] is None else row["matched_red_score"] for row in rows])
    counts = np.asarray([row["candidate_count"] for row in rows])
    confirmed = np.asarray([row["confirmed_red_count"] for row in rows])
    fig, axes = plt.subplots(2, 2, figsize=(11.2, 6.7), constrained_layout=True)
    axes[0, 0].fill_between(time, 0, visible, step="mid", alpha=0.25, label="Target visible (Oracle)", color="#64748b")
    axes[0, 0].step(time, detected, where="mid", label="Target candidate matched", color="#c62828", linewidth=1.4)
    axes[0, 0].set(ylim=(-0.05, 1.08), ylabel="Binary state", title="A. Target availability and recall")
    axes[0, 0].legend(frameon=False, loc="upper right")
    valid_error = error[np.isfinite(error)]
    if valid_error.size:
        sorted_error = np.sort(valid_error)
        axes[0, 1].plot(sorted_error, np.arange(1, len(sorted_error) + 1) / len(sorted_error), color="#1565c0", linewidth=1.8)
    axes[0, 1].axvline(3.0, color="#374151", linestyle="--", linewidth=1, label="3 m criterion")
    axes[0, 1].set(xlabel="Planar localization error (m)", ylabel="Empirical CDF", title="B. Localization error distribution")
    axes[0, 1].legend(frameon=False)
    axes[1, 0].plot(time, confidence, label="Fused candidate score", color="#1565c0", linewidth=1.1)
    axes[1, 0].plot(time, red_score, label="Red attribute score", color="#c62828", linewidth=1.1)
    axes[1, 0].set(xlabel="Elapsed time (s)", ylabel="Score", ylim=(-0.02, 1.02), title="C. Matched-target evidence")
    axes[1, 0].legend(frameon=False)
    axes[1, 1].plot(time, counts, label="All vehicle candidates", color="#455a64", linewidth=1.1)
    axes[1, 1].plot(time, confirmed, label="Confirmed red candidates", color="#c62828", linewidth=1.1)
    axes[1, 1].set(xlabel="Elapsed time (s)", ylabel="Count per frame", title="D. Candidate load and temporal confirmation")
    axes[1, 1].legend(frameon=False)
    for axis in axes.flat:
        axis.grid(alpha=0.18, linewidth=0.6)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle("CI-E2 UAV Target-Candidate Baseline", fontsize=14, fontweight="semibold")
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def save_keyframes(frames: list[np.ndarray], labels: list[str], path: Path) -> None:
    selected = np.linspace(0, len(frames) - 1, min(6, len(frames)), dtype=int)
    fig, axes = plt.subplots(2, 3, figsize=(13.2, 7.6), constrained_layout=True)
    for axis, index in zip(axes.flat, selected):
        axis.imshow(cv2.cvtColor(frames[index], cv2.COLOR_BGR2RGB))
        axis.set_title(labels[index], fontsize=9)
        axis.axis("off")
    for axis in axes.flat[len(selected) :]:
        axis.axis("off")
    fig.suptitle("Representative UAV Candidate Outputs", fontsize=14, fontweight="semibold")
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    args = parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    input_run = (args.input_run or Path(config["input_run"])).resolve()
    output_root = (args.output_root or Path(config["output"]["root"])).resolve()
    run_id = f"{config['experiment_id']}_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    output = output_root / run_id
    for name in ("detections", "metrics", "visualizations", "config_snapshot", "logs"):
        (output / name).mkdir(parents=True, exist_ok=True)
    (output / "config_snapshot" / args.config.name).write_text(args.config.read_text(encoding="utf-8"), encoding="utf-8")

    metadata = load_jsonl(input_run / "sensors/uav/metadata/frames.jsonl")
    if args.max_frames > 0:
        metadata = metadata[: args.max_frames]
    states = load_jsonl(input_run / "actors/actor_states.jsonl")
    state_index = {(row["frame"], row["role"]): row for row in states}
    manifest = json.loads((input_run / "scene_manifest.json").read_text(encoding="utf-8"))
    boundary = manifest["region"]["boundary"]["points"]
    pconfig = config["perception"]
    model_path = (ROOT / pconfig["model_path"]).resolve()
    pipeline = CandidatePipeline(
        PipelineConfig(
            model_path=model_path,
            detector_confidence=float(pconfig["confidence_threshold"]),
            detector_image_size=int(pconfig["inference_size"]),
            red_ratio_threshold=float(pconfig["red_ratio_threshold"]),
        )
    )

    candidates_path = output / "detections/candidates.jsonl"
    frame_rows: list[dict[str, Any]] = []
    annotated_frames: list[np.ndarray] = []
    annotated_labels: list[str] = []
    writer = cv2.VideoWriter(str(output / "visualizations/uav_candidate_replay.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (1600, 600))
    batch_size = 12
    with candidates_path.open("w", encoding="utf-8") as candidate_file:
        for start in range(0, len(metadata), batch_size):
            batch = metadata[start : start + batch_size]
            images = [Path(row["rgb_path"]) for row in batch]
            detector_results = pipeline.infer_batch(images)
            for meta, detector_result in zip(batch, detector_results):
                frame, timestamp = int(meta["frame"]), float(meta["timestamp"])
                image = cv2.imread(meta["rgb_path"], cv2.IMREAD_COLOR)
                depth = np.load(meta["depth_metres_path"])
                candidates = pipeline.process(frame, timestamp, image, depth, meta["sensor_transform"], detector_result)
                target, uav = state_index[(frame, "target")], state_index[(frame, "uav")]
                u, v, expected_depth = project_world_point((target["x"], target["y"], target["z"] + 1.2), meta["sensor_transform"], 800, 600, 100.0)
                in_frame = 0 <= u < 800 and 0 <= v < 600 and expected_depth > 0
                observed = float(depth[int(np.clip(round(v), 0, 599)), int(np.clip(round(u), 0, 799))]) if in_frame else float("nan")
                visible = bool(in_frame and np.isfinite(observed) and abs(observed - expected_depth) <= float(config["evaluation"]["visible_depth_tolerance_m"]))
                matched = []
                for candidate in candidates:
                    world = candidate["world_position_xyz"]
                    if world is None:
                        continue
                    error = float(np.hypot(world[0] - target["x"], world[1] - target["y"]))
                    if error <= float(config["evaluation"]["match_radius_m"]):
                        matched.append((error, candidate))
                best_error, best = min(matched, key=lambda item: item[0]) if matched else (None, None)
                row = {
                    "frame": frame,
                    "timestamp": timestamp,
                    "target_visible": visible,
                    "target_projected_uv": [u, v],
                    "target_detected": best is not None,
                    "target_red_correct": bool(best and best["color"] == "red"),
                    "target_temporal_confirmed": bool(best and best["temporal_confirmed"]),
                    "localization_error_m": best_error,
                    "matched_candidate_score": None if best is None else best["candidate_score"],
                    "matched_red_score": None if best is None else best["color_score"],
                    "matched_source": None if best is None else best["proposal_source"],
                    "oracle_target_match_id": None if best is None else best["candidate_id"],
                    "candidate_count": len(candidates),
                    "red_candidate_count": sum(item["color"] == "red" for item in candidates),
                    "confirmed_red_count": sum(item["color"] == "red" and item["temporal_confirmed"] for item in candidates),
                }
                frame_rows.append(row)
                candidate_file.write(json.dumps({"frame": frame, "timestamp": timestamp, "candidates": candidates}, ensure_ascii=False) + "\n")
                annotated = draw_candidate_frame(image, candidates, row)
                composite = np.hstack([annotated, np.vstack([depth_panel(depth, row), map_panel(candidates, target, uav, boundary, row)])])
                writer.write(composite)
                frame_index = len(frame_rows) - 1
                if best is not None and frame_index % 35 == 0:
                    annotated_frames.append(annotated)
                    annotated_labels.append(f"t={timestamp - metadata[0]['timestamp']:.1f}s | {'matched' if best else 'visible, missed'}")
            print(f"Processed {min(start + batch_size, len(metadata))}/{len(metadata)} frames", flush=True)
    writer.release()

    visible_rows = [row for row in frame_rows if row["target_visible"]]
    detected_visible = [row for row in visible_rows if row["target_detected"]]
    errors = [row["localization_error_m"] for row in detected_visible if row["localization_error_m"] is not None]
    red_candidates_total = sum(row["red_candidate_count"] for row in frame_rows)
    matched_red_total = sum(row["target_detected"] and row["target_red_correct"] for row in frame_rows)
    summary = {
        "experiment_id": config["experiment_id"],
        "input_run": str(input_run),
        "output_run": str(output),
        "model": pconfig["detector"],
        "inference_uses_ground_truth": False,
        "ground_truth_scope": "post-inference Oracle visibility and localization evaluation only",
        "frames_processed": len(frame_rows),
        "visible_target_frames": len(visible_rows),
        "detected_visible_target_frames": len(detected_visible),
        "visible_target_recall": len(detected_visible) / len(visible_rows) if visible_rows else 0.0,
        "target_color_accuracy": sum(row["target_red_correct"] for row in detected_visible) / len(detected_visible) if detected_visible else 0.0,
        "depth_valid_rate": sum(candidate["depth_m"] is not None for history in pipeline.history for candidate in history) / max(1, sum(len(history) for history in pipeline.history)),
        "median_localization_error_m": median(errors) if errors else None,
        "p90_localization_error_m": float(np.percentile(errors, 90)) if errors else None,
        "temporally_confirmed_target_frames": sum(row["target_temporal_confirmed"] for row in detected_visible),
        "red_candidate_precision_proxy": matched_red_total / red_candidates_total if red_candidates_total else 0.0,
        "candidate_count_total": sum(row["candidate_count"] for row in frame_rows),
        "red_candidate_count_total": red_candidates_total,
        "proposal_source_counts": {
            source: sum(candidate["proposal_source"] == source for history in pipeline.history for candidate in history)
            for source in ("yolo26x_coco", "instruction_guided_red_region")
        },
    }
    acceptance_cfg = config["evaluation"]["acceptance"]
    checks = {
        "visible_target_recall": summary["visible_target_recall"] >= acceptance_cfg["minimum_visible_target_recall"],
        "target_color_accuracy": summary["target_color_accuracy"] >= acceptance_cfg["minimum_target_color_accuracy"],
        "depth_valid_rate": summary["depth_valid_rate"] >= acceptance_cfg["minimum_depth_valid_rate"],
        "median_localization_error": summary["median_localization_error_m"] is not None and summary["median_localization_error_m"] <= acceptance_cfg["maximum_median_localization_error_m"],
        "p90_localization_error": summary["p90_localization_error_m"] is not None and summary["p90_localization_error_m"] <= acceptance_cfg["maximum_p90_localization_error_m"],
        "temporal_confirmation_observed": summary["temporally_confirmed_target_frames"] > 0,
    }
    acceptance = {"status": "PASS" if all(checks.values()) else "FAIL", "checks": checks, "criteria": acceptance_cfg}
    json_dump(output / "metrics/summary.json", summary)
    json_dump(output / "metrics/acceptance.json", acceptance)
    with (output / "metrics/frame_metrics.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer_csv = csv.DictWriter(handle, fieldnames=list(frame_rows[0]))
        writer_csv.writeheader()
        writer_csv.writerows(frame_rows)
    save_metrics_figure(frame_rows, output / "visualizations/academic_metrics.png")
    if annotated_frames:
        save_keyframes(annotated_frames, annotated_labels, output / "visualizations/candidate_keyframes.png")
    readme = f"""# CI-E2 UAV target-candidate baseline\n\n- Status: **{acceptance['status']}**\n- Input: `{input_run}`\n- Frames: {summary['frames_processed']}\n- Oracle-visible target recall: {summary['visible_target_recall']:.3f}\n- Target red-color accuracy: {summary['target_color_accuracy']:.3f}\n- Median / P90 localization error: {summary['median_localization_error_m']:.3f} m / {summary['p90_localization_error_m']:.3f} m\n- Temporally confirmed target frames: {summary['temporally_confirmed_target_frames']}\n\nInference reads only UAV RGB, metric depth and camera pose. CARLA actor states are isolated to post-inference evaluation.\n"""
    (output / "README.md").write_text(readme, encoding="utf-8")
    (output_root / "LATEST.txt").write_text(str(output), encoding="utf-8")
    print(json.dumps({"output": str(output), "summary": summary, "acceptance": acceptance}, ensure_ascii=False, indent=2))
    return 0 if acceptance["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
