#!/usr/bin/env python3
"""Aggregate completed S1 multi-seed/target-position episodes."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


EVENTS = ("MESSAGE_SENT", "TARGET_VERIFIED", "ARRIVED_SAFE", "COMPLETED")


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def read_events(path: Path) -> list[dict]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def directory_size_bytes(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def planned_episode_count(batch_dir: Path) -> int | None:
    """Read the frozen batch size used to decide whether aggregation is complete."""
    progress_path = batch_dir / "batch_progress.json"
    if not progress_path.exists():
        return None
    value = read_json(progress_path).get("planned_episodes")
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        return None
    return value


def build_batch_summary(
    rows: list[dict], analysis: Path, episodes_planned: int | None
) -> dict:
    episodes_available = len(rows)
    episodes_passed = sum(row["status"] == "PASS" for row in rows)
    episodes_failed = sum(row["status"] != "PASS" for row in rows)
    batch_complete = (
        episodes_planned is not None and episodes_available == episodes_planned
    )
    return {
        "episodes_planned": episodes_planned,
        "episodes_available": episodes_available,
        "episodes_passed": episodes_passed,
        "episodes_failed": episodes_failed,
        "batch_complete": batch_complete,
        "all_passed": batch_complete and episodes_failed == 0,
        "total_storage_gb": sum(float(row["storage_gb"]) for row in rows),
        "analysis_directory": str(analysis),
    }


def collect(batch_dir: Path) -> list[dict]:
    rows: list[dict] = []
    for report_path in sorted((batch_dir / "episodes").glob("*/validation_report.json")):
        run_dir = report_path.parent
        report = read_json(report_path)
        config = yaml.safe_load((run_dir / "resolved_config.yaml").read_text(encoding="utf-8"))
        summary = report.get("summary", {})
        closed = summary.get("perception_closed_loop", {})
        events = read_events(run_dir / "task" / "task_events.jsonl")
        initial_time = next(
            (float(event["timestamp"]) for event in events if event.get("new_state") == "SEARCHING"),
            None,
        )
        event_times = {}
        for name in EVENTS:
            match = next((event for event in events if event.get("new_state") == name), None)
            event_times[name] = (
                None if match is None or initial_time is None else float(match["timestamp"]) - initial_time
            )
        rows.append(
            {
                "episode_id": config["experiment_id"],
                "seed": int(config["random_seed"]),
                "target_position": str(config.get("batch", {}).get("target_position_id", "?")),
                "status": report.get("status", "UNKNOWN"),
                "final_state": closed.get("task_final_state"),
                "message_time_s": event_times["MESSAGE_SENT"],
                "verification_time_s": event_times["TARGET_VERIFIED"],
                "arrival_time_s": event_times["ARRIVED_SAFE"],
                "completion_time_s": event_times["COMPLETED"],
                "message_localization_error_m": closed.get("message_localization_error_m"),
                "final_target_distance_m": summary.get("ugv_final_target_distance_m"),
                "ugv_path_m": summary.get("ugv_path_m"),
                "uav_path_m": summary.get("uav_path_m"),
                "common_frame_ratio": summary.get("common_frame_ratio"),
                "persisted_sensor_frames": summary.get("persisted_sensor_frames"),
                "moving_vehicles": summary.get("moving_distractor_vehicles"),
                "moving_pedestrians": summary.get("moving_pedestrians"),
                "ugv_collisions": summary.get("ugv_collision_count"),
                "uav_collisions": summary.get("uav_collision_count"),
                "wrong_target_messages": closed.get("wrong_target_messages"),
                "local_confirmation_success": closed.get("local_confirmation_success"),
                "storage_gb": directory_size_bytes(run_dir) / (1024 ** 3),
                "run_directory": str(run_dir),
            }
        )
    return rows


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_summary(path: Path, rows: list[dict]) -> None:
    labels = [f"{row['target_position']}-S{row['seed']}" for row in rows]
    x = np.arange(len(rows))
    fig, axes = plt.subplots(2, 2, figsize=(12.2, 7.5), constrained_layout=True)

    colours = {
        "message_time_s": "#2f5597",
        "verification_time_s": "#70ad47",
        "arrival_time_s": "#ed7d31",
        "completion_time_s": "#7030a0",
    }
    for key, label in (
        ("message_time_s", "Message sent"),
        ("verification_time_s", "Target verified"),
        ("arrival_time_s", "UGV arrived"),
        ("completion_time_s", "Task completed"),
    ):
        values = [np.nan if row[key] is None else float(row[key]) for row in rows]
        axes[0, 0].scatter(x, values, s=34, label=label, color=colours[key])
    axes[0, 0].set(title="A. Closed-loop event timing", ylabel="Time from task start (s)")
    axes[0, 0].legend(frameon=False, ncol=2, fontsize=8)

    errors = [np.nan if row["message_localization_error_m"] is None else float(row["message_localization_error_m"]) for row in rows]
    axes[0, 1].scatter(x, errors, s=42, color="#2f5597")
    axes[0, 1].axhline(3.0, color="#c00000", linestyle="--", linewidth=1.1, label="3 m limit")
    axes[0, 1].set(title="B. Semantic-message localization", ylabel="XY error (m)")
    axes[0, 1].legend(frameon=False)

    distances = [np.nan if row["final_target_distance_m"] is None else float(row["final_target_distance_m"]) for row in rows]
    axes[1, 0].scatter(x, distances, s=42, color="#ed7d31")
    axes[1, 0].axhspan(3.0, 7.0, color="#70ad47", alpha=0.16, label="Accepted 3–7 m")
    axes[1, 0].set(title="C. Final UGV standoff", ylabel="Distance to target (m)")
    axes[1, 0].legend(frameon=False)

    vehicles = [float(row["moving_vehicles"] or 0) for row in rows]
    pedestrians = [float(row["moving_pedestrians"] or 0) for row in rows]
    width = 0.36
    axes[1, 1].bar(x - width / 2, vehicles, width, color="#5b9bd5", label="Moving vehicles")
    axes[1, 1].bar(x + width / 2, pedestrians, width, color="#a5a5a5", label="Moving pedestrians")
    axes[1, 1].set(title="D. Dynamic distractor coverage", ylabel="Count")
    axes[1, 1].legend(frameon=False)

    for axis in axes.flat:
        axis.set_xticks(x, labels, rotation=35, ha="right")
        axis.grid(axis="y", alpha=0.22, linewidth=0.6)
        axis.spines[["top", "right"]].set_visible(False)
    passed = sum(row["status"] == "PASS" for row in rows)
    fig.suptitle(
        f"S1 controlled robustness batch — {passed}/{len(rows)} episodes passed",
        fontsize=14,
        fontweight="semibold",
    )
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-dir", required=True, type=Path)
    args = parser.parse_args()
    batch_dir = args.batch_dir.resolve()
    analysis = batch_dir / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)
    rows = collect(batch_dir)
    write_csv(analysis / "episode_metrics.csv", rows)
    (analysis / "episode_metrics.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    if rows:
        plot_summary(analysis / "s1_batch_academic_summary.png", rows)
    summary = build_batch_summary(rows, analysis, planned_episode_count(batch_dir))
    (analysis / "batch_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["all_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
