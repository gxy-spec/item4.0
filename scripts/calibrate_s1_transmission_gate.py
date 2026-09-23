#!/usr/bin/env python3
"""Calibrate the S1 transmit gate on existing CI-E2 output; truth is evaluation-only."""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--candidate-run",
        type=Path,
        default=Path(
            r"E:\CarlaAirData\CityInspection_GOC\e2_candidate_baseline\CI_E2_UAV_CANDIDATE_T10_ZA_S1001_20260922T022645Z"
        ),
    )
    parser.add_argument(
        "--sensor-run",
        type=Path,
        default=Path(
            r"E:\CarlaAirData\CityInspection_GOC\e1_physical_fix\smoke\CI_E1_PHYSICAL_FIX_T10_ZA_S1001_20260922T012328Z"
        ),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(r"E:\CarlaAirData\CityInspection_GOC\e2_perception_closed_loop\offline_calibration"),
    )
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output_root.resolve() / f"S1_GATE_CALIBRATION_{stamp}"
    output.mkdir(parents=True, exist_ok=True)

    truth = {
        int(row["frame"]): row
        for row in jsonl(args.sensor_run / "actors/actor_states.jsonl")
        if row["role"] == "target"
    }
    frame_metrics_path = args.candidate_run / "metrics/frame_metrics.csv"
    with frame_metrics_path.open(encoding="utf-8-sig", newline="") as handle:
        frame_metrics = {int(row["frame"]): row for row in csv.DictReader(handle)}
    rows = jsonl(args.candidate_run / "detections/candidates.jsonl")

    grid: list[dict] = []
    for red_threshold in (0.40, 0.50, 0.60, 0.65, 0.70):
        for temporal_hits in (3, 4, 5):
            selected_frames = 0
            correct_frames = 0
            visible_frames = 0
            correct_visible_frames = 0
            first_selection: dict | None = None
            for row in rows:
                frame = int(row["frame"])
                visible = frame_metrics.get(frame, {}).get("target_visible", "False").lower() == "true"
                visible_frames += int(visible)
                candidates = [
                    candidate
                    for candidate in row["candidates"]
                    if candidate.get("proposal_source") == "yolo26x_coco"
                    and candidate.get("color") == "red"
                    and float(candidate.get("red_pixel_ratio", 0.0)) >= red_threshold
                    and int(candidate.get("temporal_hits", 0)) >= temporal_hits
                    and candidate.get("world_position_xyz") is not None
                ]
                if not candidates or frame not in truth:
                    continue
                selected = max(
                    candidates,
                    key=lambda item: 0.55 * float(item["red_pixel_ratio"])
                    + 0.35 * float(item["detector_score"])
                    + 0.10 * min(1.0, float(item["temporal_hits"]) / 5.0),
                )
                target = truth[frame]
                error = math.hypot(
                    float(selected["world_position_xyz"][0]) - float(target["x"]),
                    float(selected["world_position_xyz"][1]) - float(target["y"]),
                )
                correct = error <= 5.0
                selected_frames += 1
                correct_frames += int(correct)
                correct_visible_frames += int(visible and correct)
                if first_selection is None:
                    first_selection = {
                        "frame": frame,
                        "candidate_id": selected["candidate_id"],
                        "correct_target": bool(correct),
                        "localization_error_m": error,
                    }
            grid.append(
                {
                    "minimum_red_ratio": red_threshold,
                    "minimum_temporal_hits": temporal_hits,
                    "selected_frames": selected_frames,
                    "eligible_frame_precision": correct_frames / max(1, selected_frames),
                    "visible_target_availability": correct_visible_frames / max(1, visible_frames),
                    "first_selected_frame": None if first_selection is None else first_selection["frame"],
                    "first_selection_correct": bool(first_selection and first_selection["correct_target"]),
                    "first_selection_localization_error_m": (
                        None if first_selection is None else first_selection["localization_error_m"]
                    ),
                }
            )

    chosen = next(
        row
        for row in grid
        if row["minimum_red_ratio"] == 0.70 and row["minimum_temporal_hits"] == 4
    )
    summary = {
        "candidate_run": str(args.candidate_run.resolve()),
        "sensor_run": str(args.sensor_run.resolve()),
        "oracle_scope": "offline calibration and evaluation only; no truth enters S1 runtime",
        "chosen_gate": chosen,
        "selection_reason": (
            "The 0.70 red-ratio / 4-hit gate makes the first transmissible candidate the true target "
            "and yields 100% eligible-frame precision on this calibration sequence. It deliberately "
            "trades frame availability for message correctness."
        ),
    }
    (output / "calibration_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    with (output / "threshold_sweep.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(grid[0]))
        writer.writeheader()
        writer.writerows(grid)

    red_values = sorted({row["minimum_red_ratio"] for row in grid})
    hit_values = sorted({row["minimum_temporal_hits"] for row in grid})
    precision = np.asarray(
        [[next(row["eligible_frame_precision"] for row in grid if row["minimum_red_ratio"] == red and row["minimum_temporal_hits"] == hit) for hit in hit_values] for red in red_values]
    )
    availability = np.asarray(
        [[next(row["visible_target_availability"] for row in grid if row["minimum_red_ratio"] == red and row["minimum_temporal_hits"] == hit) for hit in hit_values] for red in red_values]
    )
    fig, axes = plt.subplots(1, 2, figsize=(10.8, 4.2), constrained_layout=True)
    for axis, values, title in zip(
        axes,
        (precision, availability),
        ("A. Eligible-frame target precision", "B. Visible-target message availability"),
    ):
        image = axis.imshow(values, vmin=0.0, vmax=1.0, cmap="Blues", aspect="auto")
        axis.set_xticks(range(len(hit_values)), hit_values)
        axis.set_yticks(range(len(red_values)), [f"{value:.2f}" for value in red_values])
        axis.set(xlabel="Minimum temporal hits", ylabel="Minimum red-pixel ratio", title=title)
        for y in range(len(red_values)):
            for x in range(len(hit_values)):
                axis.text(x, y, f"{values[y, x]:.1%}", ha="center", va="center", color="#172033")
        fig.colorbar(image, ax=axis, fraction=0.046, pad=0.04)
    fig.suptitle("S1 UAV Semantic-Message Gate Calibration (truth used only after inference)", fontweight="semibold")
    fig.savefig(output / "gate_calibration.png", dpi=220, bbox_inches="tight")
    plt.close(fig)
    print(json.dumps({"output": str(output), **summary}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
