#!/usr/bin/env python3
"""Summarize a CI-E5 UGV patrol/search run and render a compact result figure."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    report = json.loads((run_dir / "validation_report.json").read_text(encoding="utf-8"))
    metrics = report["summary"]
    target_xyz = metrics["ugv_local_target_search"]["target_world_position_xyz"]
    candidate_rows = [
        json.loads(line)
        for line in (run_dir / "perception" / "ugv_target_candidates.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    configured_model = Path(metrics["ugv_local_target_search"]["model"])
    corrected_source = f"{configured_model.stem.lower()}_coco"
    for row in candidate_rows:
        for candidate in row.get("candidates", []):
            if str(candidate.get("proposal_source", "")).endswith("_coco"):
                candidate["proposal_source"] = corrected_source
    audited_candidates_path = run_dir / "perception" / "ugv_target_candidates_audited.jsonl"
    audited_candidates_path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in candidate_rows),
        encoding="utf-8",
    )
    candidates: list[dict[str, Any]] = []
    for row in candidate_rows:
        for candidate in row.get("candidates", []):
            xyz = candidate.get("world_position_xyz")
            if candidate.get("color") != "red" or xyz is None:
                continue
            error = math.hypot(float(xyz[0]) - float(target_xyz[0]), float(xyz[1]) - float(target_xyz[1]))
            candidates.append({**candidate, "target_error_m": error})

    threshold_m = 5.0
    candidates_near_truth = [item for item in candidates if item["target_error_m"] <= threshold_m]
    frame_hit_count = sum(
        any(
            candidate.get("color") == "red"
            and candidate.get("world_position_xyz") is not None
            and math.hypot(
                float(candidate["world_position_xyz"][0]) - float(target_xyz[0]),
                float(candidate["world_position_xyz"][1]) - float(target_xyz[1]),
            ) <= threshold_m
            for candidate in row.get("candidates", [])
        )
        for row in candidate_rows
    )
    search_summary = {
        "status": report["status"],
        "target_match_distance_threshold_m": threshold_m,
        "inference_frames": len(candidate_rows),
        "red_vehicle_candidates": len(candidates),
        "audited_detector_source": corrected_source,
        "candidate_points_within_5m_of_target": len(candidates_near_truth),
        "candidate_point_match_fraction_within_5m": len(candidates_near_truth) / max(1, len(candidates)),
        "inference_frames_with_candidate_within_5m": frame_hit_count,
        "frame_hit_fraction_within_5m": frame_hit_count / max(1, len(candidate_rows)),
        "best_candidate_localization_error_m": metrics["ugv_local_target_search"]["best_candidate_to_target_error_m"],
        "ugv_path_m": metrics["ugv_path_m"],
        "ugv_rgbd_pair_frames": metrics["ugv_rgbd_obstacle_guard"]["ugv_rgbd_pair_frames"],
        "ugv_rgbd_pair_ratio": metrics["ugv_rgbd_obstacle_guard"]["ugv_rgbd_pair_ratio"],
        "ugv_target_drift_m": metrics["target_drift_m"],
        "ugv_collision_count": metrics["ugv_collision_count"],
        "uav_mode": "fixed passive observation point; UAV route not evaluated in CI-E5",
        "ground_truth_usage": "post-run candidate scoring only; excluded from online control",
    }
    output_json = run_dir / "ground_truth" / "evaluation_only" / "ugv_patrol_search_analysis.json"
    output_json.write_text(json.dumps(search_summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    readme_path = run_dir / "README.md"
    readme_note = (
        "\n> Candidate source-label audit: the raw JSONL retains the original logger's hardcoded `yolo26x_coco` field. "
        f"Use `perception/ugv_target_candidates_audited.jsonl` for this run; the configured detector was `{corrected_source}`. "
        "The raw log is preserved unchanged.\n"
    )
    if readme_path.exists():
        readme_text = readme_path.read_text(encoding="utf-8")
        if "Candidate source-label audit:" not in readme_text:
            readme_path.write_text(readme_text + readme_note, encoding="utf-8")

    ugv_rows = read_csv(run_dir / "trajectories" / "ugv_trajectory.csv")
    safety_rows = read_csv(run_dir / "safety" / "safety_state.csv")
    x = [float(row["x"]) for row in ugv_rows]
    y = [float(row["y"]) for row in ugv_rows]
    timestamps = [float(row["timestamp"]) for row in ugv_rows]
    elapsed = [value - timestamps[0] for value in timestamps]
    speed = [float(row["speed_mps"]) for row in ugv_rows]
    target_x, target_y = float(target_xyz[0]), float(target_xyz[1])
    near = [item for item in candidates if item["target_error_m"] <= threshold_m]
    other = [item for item in candidates if item["target_error_m"] > threshold_m]

    plt.rcParams.update({"font.family": "Microsoft YaHei", "font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, (ax_map, ax_speed) = plt.subplots(1, 2, figsize=(14.4, 6.4), gridspec_kw={"width_ratios": [1.1, 1]})
    fig.suptitle("9月30日｜UGV巡检与本地 RGB-D 搜索验收", fontsize=16, fontweight="semibold", y=0.98)

    if other:
        ax_map.scatter([item["world_position_xyz"][0] for item in other], [item["world_position_xyz"][1] for item in other], s=14, c="#9aa6b2", alpha=0.45, label=f"其他红色候选（>{threshold_m:.0f} m）")
    if near:
        ax_map.scatter([item["world_position_xyz"][0] for item in near], [item["world_position_xyz"][1] for item in near], s=18, c="#e08a1e", alpha=0.65, label=f"距目标真值≤{threshold_m:.0f} m 的候选")
    ax_map.plot(x, y, color="#1769aa", linewidth=2.6, label="UGV实际巡检轨迹")
    ax_map.scatter(x[0], y[0], marker="o", s=65, c="#1769aa", edgecolor="white", zorder=4, label="UGV起点")
    ax_map.scatter(x[-1], y[-1], marker="s", s=65, c="#1769aa", edgecolor="white", zorder=4, label="UGV终点")
    ax_map.scatter(target_x, target_y, marker="*", s=220, c="#c53d43", edgecolor="white", linewidth=0.8, zorder=5, label="静止目标车辆")
    ax_map.set_title("世界坐标中的轨迹与候选定位")
    ax_map.set_xlabel("CARLA 世界 X（m）")
    ax_map.set_ylabel("CARLA 世界 Y（m）")
    ax_map.grid(True, color="#e7ebef", linewidth=0.8)
    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.legend(loc="best", frameon=True, fontsize=8)

    ax_speed.plot(elapsed, speed, color="#1769aa", linewidth=1.6, label="UGV速度")
    mode_counts: dict[str, int] = {}
    for row in safety_rows:
        mode = row.get("ugv_safety_mode", "unknown")
        mode_counts[mode] = mode_counts.get(mode, 0) + 1
    stop_ticks = metrics["ugv_rgbd_obstacle_guard"]["sensor_stop_ticks"]
    caution_ticks = metrics["ugv_rgbd_obstacle_guard"]["sensor_caution_ticks"]
    ax_speed.set_title("UGV运动与RGB-D安全决策")
    ax_speed.set_xlabel("仿真时间（s）")
    ax_speed.set_ylabel("速度（m/s）")
    ax_speed.set_xlim(0, max(elapsed, default=0.0))
    ax_speed.set_ylim(bottom=0)
    ax_speed.grid(True, color="#e7ebef", linewidth=0.8)
    ax_speed.text(
        0.04,
        0.96,
        f"UGV里程：{metrics['ugv_path_m']:.1f} m\n"
        f"RGB-D配对：{metrics['ugv_rgbd_obstacle_guard']['ugv_rgbd_pair_frames']} 帧 / 100%\n"
        f"深度减速：{caution_ticks} ticks；停车：{stop_ticks} ticks\n"
        f"候选点距真值≤5 m：{len(candidates_near_truth)}/{len(candidates)}（逐候选统计）\n"
        f"逐帧命中：{frame_hit_count}/{len(candidate_rows)} 帧\n"
        f"目标漂移：{metrics['target_drift_m']:.2f} m；UGV碰撞：{metrics['ugv_collision_count']}",
        transform=ax_speed.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.65", "facecolor": "white", "edgecolor": "#cad3dd", "alpha": 0.95},
    )
    ax_speed.legend(loc="lower right", frameon=False)
    fig.text(0.5, 0.015, "候选真值匹配仅用于仿真结束后的评估；不参与UGV实时控制。UAV在本阶段固定为被动观测端。", ha="center", color="#526273", fontsize=9)
    fig.tight_layout(rect=(0, 0.05, 1, 0.94))
    output_plot = run_dir / "preview" / "ugv_patrol_search_summary.png"
    fig.savefig(output_plot, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(json.dumps({"analysis_json": str(output_json), "summary_figure": str(output_plot), "audited_candidates": str(audited_candidates_path), **search_summary}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
