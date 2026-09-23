#!/usr/bin/env python3
"""Create the S1 perception/message/confirmation/navigation acceptance figure."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def jsonl_rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    report = json.loads((run_dir / "validation_report.json").read_text(encoding="utf-8"))
    timeline = csv_rows(run_dir / "task/state_timeline.csv")
    safety = csv_rows(run_dir / "safety/safety_state.csv")
    events = jsonl_rows(run_dir / "task/task_events.jsonl")
    candidates = jsonl_rows(run_dir / "perception/uav_candidates.jsonl")
    confirmations = jsonl_rows(run_dir / "perception/ugv_confirmations.jsonl")
    if not timeline or not safety:
        raise RuntimeError("S1 summary inputs are empty")

    t0 = float(timeline[0]["timestamp"])
    time = np.asarray([float(row["timestamp"]) - t0 for row in timeline])
    state_names = [
        "SEARCHING",
        "MESSAGE_SENT",
        "NAVIGATING",
        "LOCAL_CONFIRMING",
        "TARGET_VERIFIED",
        "ARRIVED_SAFE",
        "UAV_ROUTE_COMPLETE",
        "COMPLETED",
    ]
    state_code = {name: index for index, name in enumerate(state_names)}
    state_values = np.asarray([state_code.get(row["state"], -1) for row in timeline])
    safety_time = np.asarray([float(row["timestamp"]) - float(safety[0]["timestamp"]) for row in safety])
    distance = np.asarray([float(row["ugv_target_distance_m"]) for row in safety])
    speed = np.asarray([float(row["ugv_speed_mps"]) for row in safety])
    progress = np.asarray([float(row["uav_waypoint_progress_m"]) for row in timeline])

    selected_time: list[float] = []
    selected_score: list[float] = []
    selected_red: list[float] = []
    for row in candidates:
        selected_id = row.get("selected_candidate_id")
        selected = next((item for item in row.get("candidates", []) if item.get("candidate_id") == selected_id), None)
        if selected is not None:
            selected_time.append(float(row["timestamp"]) - t0)
            selected_score.append(float(selected.get("candidate_score", 0.0)))
            selected_red.append(float(selected.get("red_pixel_ratio", 0.0)))

    fig, axes = plt.subplots(2, 2, figsize=(12.6, 8.0), constrained_layout=True)
    axes[0, 0].step(time, state_values, where="post", color="#365f91", linewidth=1.8)
    axes[0, 0].set_yticks(range(len(state_names)), state_names)
    axes[0, 0].set(xlabel="Simulation time (s)", ylabel="Task state", title="A. Perception-driven task progression")

    if selected_time:
        axes[0, 1].plot(selected_time, selected_score, color="#1565c0", marker="o", ms=3, label="Candidate score")
        axes[0, 1].plot(selected_time, selected_red, color="#c62828", marker="o", ms=3, label="Red-pixel ratio")
    axes[0, 1].axhline(0.70, color="#555", linestyle="--", linewidth=1, label="Transmit red gate")
    axes[0, 1].set(xlabel="Simulation time (s)", ylabel="Score", ylim=(0, 1.02), title="B. UAV transmission evidence")
    axes[0, 1].legend(frameon=False)

    axes[1, 0].plot(safety_time, distance, color="#1565c0", linewidth=1.5, label="UGV-to-message distance")
    axes[1, 0].axhspan(3.0, 7.0, color="#2e7d32", alpha=0.12, label="Safe arrival band")
    speed_axis = axes[1, 0].twinx()
    speed_axis.plot(safety_time, speed, color="#ef6c00", linewidth=1.0, alpha=0.8, label="UGV speed")
    axes[1, 0].set(xlabel="Simulation time (s)", ylabel="Distance (m)", title="C. Message-triggered approach and safe stop")
    speed_axis.set_ylabel("Speed (m/s)")
    handles_a, labels_a = axes[1, 0].get_legend_handles_labels()
    handles_b, labels_b = speed_axis.get_legend_handles_labels()
    axes[1, 0].legend(handles_a + handles_b, labels_a + labels_b, frameon=False, loc="upper right")

    axes[1, 1].plot(time, progress, color="#7b2cbf", linewidth=1.7, label="UAV route progress")
    axes[1, 1].axhline(720.0, color="#374151", linestyle="--", linewidth=1.0, label="Full route 720 m")
    confirmed = [row for row in confirmations if row.get("confirmed")]
    if confirmed:
        axes[1, 1].axvline(float(confirmed[0]["timestamp"]) - t0, color="#2e7d32", linestyle=":", label="UGV local confirmation")
    axes[1, 1].set(xlabel="Simulation time (s)", ylabel="Route progress (m)", title="D. UAV coverage and UGV confirmation")
    axes[1, 1].legend(frameon=False)

    event_colours = {
        "MESSAGE_SENT": "#c62828",
        "TARGET_RECEIVED": "#d84315",
        "LOCAL_CONFIRMING": "#6a1b9a",
        "TARGET_VERIFIED": "#2e7d32",
        "ARRIVED_SAFE": "#00695c",
    }
    for event in events:
        if event.get("new_state") not in event_colours:
            continue
        event_time = float(event["timestamp"]) - t0
        for axis in axes.flat:
            axis.axvline(event_time, color=event_colours[event["new_state"]], linestyle=":", linewidth=0.8, alpha=0.55)
    for axis in axes.flat:
        axis.grid(alpha=0.18, linewidth=0.6)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle(
        f"S1 Perception Closed-Loop Acceptance — {report['status']}\nUAV RGB/depth semantic message; UGV RGB/depth local confirmation; oracle=false",
        fontsize=14,
        fontweight="semibold",
    )
    output = run_dir / "preview/s1_perception_closed_loop_summary.png"
    fig.savefig(output, dpi=220, bbox_inches="tight")
    plt.close(fig)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
