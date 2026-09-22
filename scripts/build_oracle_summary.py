#!/usr/bin/env python3
"""Create an auditable S0 Oracle state/communication/navigation summary figure."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    report = json.loads((run_dir / "validation_report.json").read_text(encoding="utf-8"))
    timeline = rows(run_dir / "task/state_timeline.csv")
    safety = rows(run_dir / "safety/safety_state.csv")
    events = [json.loads(line) for line in (run_dir / "task/task_events.jsonl").read_text(encoding="utf-8").splitlines() if line]
    if not timeline or not safety:
        raise RuntimeError("Oracle summary inputs are empty")

    t0 = float(timeline[0]["timestamp"])
    time = np.asarray([float(row["timestamp"]) - t0 for row in timeline])
    state_names = [
        "SEARCHING",
        "MESSAGE_SENT",
        "NAVIGATING",
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

    fig, axes = plt.subplots(3, 1, figsize=(12.2, 8.0), sharex=True, constrained_layout=True)
    axes[0].step(time, state_values, where="post", color="#6a1b9a", linewidth=1.8)
    axes[0].set_yticks(range(len(state_names)), state_names)
    axes[0].set_ylabel("Task state")
    axes[0].set_title("A. Auditable state-machine progression")

    axes[1].plot(safety_time, distance, color="#1565c0", linewidth=1.5, label="UGV-to-target distance")
    axes[1].axhspan(3.0, 7.0, color="#2e7d32", alpha=0.12, label="Safe arrival band (3–7 m)")
    speed_axis = axes[1].twinx()
    speed_axis.plot(safety_time, speed, color="#ef6c00", linewidth=1.0, alpha=0.8, label="UGV speed")
    axes[1].set_ylabel("Distance (m)")
    speed_axis.set_ylabel("Speed (m/s)")
    axes[1].set_title("B. Message-triggered UGV approach and safe stop")
    handles_a, labels_a = axes[1].get_legend_handles_labels()
    handles_b, labels_b = speed_axis.get_legend_handles_labels()
    axes[1].legend(handles_a + handles_b, labels_a + labels_b, frameon=False, loc="upper right")

    axes[2].plot(time, progress, color="#7b2cbf", linewidth=1.7, label="UAV route progress")
    axes[2].axhline(720.0, color="#374151", linestyle="--", linewidth=1.0, label="Planned route 720 m")
    axes[2].set(xlabel="Elapsed simulation time (s)", ylabel="Route progress (m)")
    axes[2].set_title("C. UAV full lawnmower-route completion")
    axes[2].legend(frameon=False, loc="lower right")

    event_colours = {
        "MESSAGE_SENT": "#c62828",
        "TARGET_RECEIVED": "#d84315",
        "ARRIVED_SAFE": "#2e7d32",
        "UAV_ROUTE_COMPLETE": "#6a1b9a",
    }
    for event in events:
        name = event["new_state"]
        if name not in event_colours:
            continue
        event_time = float(event["timestamp"]) - t0
        for axis in axes:
            axis.axvline(event_time, color=event_colours[name], linestyle=":", linewidth=1.0, alpha=0.8)
        axes[0].text(event_time, len(state_names) - 0.25, name, rotation=90, va="top", ha="right", fontsize=7, color=event_colours[name])

    for axis in axes:
        axis.grid(alpha=0.18, linewidth=0.6)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle(
        f"S0 ORACLE Closed-Loop Acceptance — {report['status']}\nCARLA ground-truth target position; not a perception result",
        fontsize=14,
        fontweight="semibold",
        color="#8e1b1b",
    )
    output = run_dir / "preview/oracle_closed_loop_summary.png"
    fig.savefig(output, dpi=220, bbox_inches="tight")
    plt.close(fig)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
