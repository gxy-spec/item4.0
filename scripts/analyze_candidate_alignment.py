#!/usr/bin/env python3
"""Create compact post-run diagnostics for the 2026-10-01 dual-device RGB-D run."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


def read_jsonl(path: Path) -> list[dict]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    preview = run_dir / "preview"
    preview.mkdir(exist_ok=True)
    config = yaml.safe_load((run_dir / "resolved_config.yaml").read_text(encoding="utf-8"))
    evaluation = json.loads((run_dir / "ground_truth" / "evaluation_only" / "dual_device_candidate_alignment_evaluation.json").read_text(encoding="utf-8"))
    candidates = read_jsonl(run_dir / "perception" / "candidate_records.jsonl")
    device_rows = {
        device: read_jsonl(run_dir / "perception" / f"{device}_target_candidates_synchronized.jsonl")
        for device in ("uav", "ugv")
    }
    pair_radius = float(config.get("acceptance", {}).get("candidate_pair_distance_m", 10.0))
    rows_by_device_frame = {
        device: {int(row["frame_id"]): row for row in rows}
        for device, rows in device_rows.items()
    }
    candidate_only_pairs = []
    for frame in sorted(set(rows_by_device_frame["uav"]) & set(rows_by_device_frame["ugv"])):
        first = [item for item in rows_by_device_frame["uav"][frame].get("candidates", []) if item.get("color") == "red" and item.get("world_position_xyz") is not None]
        second = [item for item in rows_by_device_frame["ugv"][frame].get("candidates", []) if item.get("color") == "red" and item.get("world_position_xyz") is not None]
        proposals = sorted(
            [(float(np.linalg.norm(np.asarray(a["world_position_xyz"]) - np.asarray(b["world_position_xyz"]))), a, b) for a in first for b in second],
            key=lambda row: row[0],
        )
        used_first, used_second = set(), set()
        for distance, a, b in proposals:
            if distance > pair_radius:
                break
            if a["candidate_id"] in used_first or b["candidate_id"] in used_second:
                continue
            used_first.add(a["candidate_id"])
            used_second.add(b["candidate_id"])
            candidate_only_pairs.append({
                "frame_id": frame,
                "timestamp_s": float(rows_by_device_frame["uav"][frame]["timestamp_s"]),
                "uav_candidate_id": str(a["candidate_id"]),
                "ugv_candidate_id": str(b["candidate_id"]),
                "uav_world_xyz_m": a["world_position_xyz"],
                "ugv_world_xyz_m": b["world_position_xyz"],
                "candidate_position_difference_m": distance,
                "match_uses_ground_truth": False,
            })
    pair_distances = [row["candidate_position_difference_m"] for row in candidate_only_pairs]
    evaluation.update({
        "candidate_only_spatial_pair_count": len(candidate_only_pairs),
        "candidate_only_pair_distance_threshold_m": pair_radius,
        "candidate_only_pair_position_error_median": float(np.median(pair_distances)) if pair_distances else None,
        "candidate_only_pair_position_error_p90": float(np.percentile(pair_distances, 90)) if pair_distances else None,
    })
    (run_dir / "ground_truth" / "evaluation_only" / "dual_device_candidate_alignment_evaluation.json").write_text(json.dumps(evaluation, ensure_ascii=False, indent=2), encoding="utf-8")
    (run_dir / "ground_truth" / "evaluation_only" / "candidate_only_spatial_pairs.jsonl").write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in candidate_only_pairs), encoding="utf-8")
    report_path = run_dir / "validation_report.json"
    if report_path.exists():
        report = json.loads(report_path.read_text(encoding="utf-8"))
        checks = report.setdefault("checks", [])
        if not any(item.get("name") == "candidate_only_cross_device_spatial_pairs" for item in checks):
            minimum_pairs = int(config.get("acceptance", {}).get("minimum_candidate_only_spatial_pairs", 10))
            checks.append({
                "name": "candidate_only_cross_device_spatial_pairs",
                "passed": len(candidate_only_pairs) >= minimum_pairs,
                "value": len(candidate_only_pairs),
                "criterion": f">={minimum_pairs} same-frame red candidate pairs within {pair_radius} m; no truth used",
                "severity": "error",
            })
            summary = report.setdefault("summary", {}).setdefault("dual_device_candidate_alignment", {})
            summary.update({
                "candidate_only_spatial_pair_count": len(candidate_only_pairs),
                "candidate_only_pair_distance_threshold_m": pair_radius,
                "candidate_only_pair_position_error_median": evaluation["candidate_only_pair_position_error_median"],
                "candidate_only_pair_position_error_p90": evaluation["candidate_only_pair_position_error_p90"],
            })
            report["status"] = "PASS" if all(item.get("passed", False) for item in checks) else "FAIL"
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    states = read_jsonl(run_dir / "actors" / "actor_states.jsonl")
    target_states = [row for row in states if "target" in str(row.get("role", "")).lower()]
    if not target_states:
        target_states = [row for row in states if "target" in str(row.get("type_id", "")).lower()]
    target_xy = np.asarray([[float(row["x"]), float(row["y"])] for row in target_states]) if target_states else np.empty((0, 2))
    target_truth_xyz = config.get("region", {}).get("target_transform", {}).get("location_xyz")
    if target_truth_xyz is None and target_states:
        target_truth_xyz = [float(target_states[-1][key]) for key in ("x", "y", "z")]

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig = plt.figure(figsize=(15, 9))
    grid = fig.add_gridspec(
        2, 2, height_ratios=[1.1, 1],
        left=0.06, right=0.98, bottom=0.08, top=0.86,
        wspace=0.22, hspace=0.30,
    )
    ax_map = fig.add_subplot(grid[0, 0])
    ax_error = fig.add_subplot(grid[0, 1])
    ax_count = fig.add_subplot(grid[1, 0])
    ax_overlay = fig.add_subplot(grid[1, 1])

    for role, color, label in (("ugv", "#2369a8", "UGV patrol"), ("uav", "#7b4ab5", "UAV hold")):
        path = run_dir / "trajectories" / f"{role}_trajectory.csv"
        if path.exists():
            with path.open(encoding="utf-8-sig", newline="") as handle:
                rows = list(csv.DictReader(handle))
            if rows:
                ax_map.plot([float(row["x"]) for row in rows], [float(row["y"]) for row in rows], color=color, lw=2, label=label)
    if len(target_xy):
        ax_map.scatter(target_xy[:, 0], target_xy[:, 1], marker="*", s=150, color="#c43d3d", label="Target truth (evaluation only)", zorder=4)
    for device, color, marker in (("uav", "#7651a6", "o"), ("ugv", "#168a73", "s")):
        points = [row.get("world_xyz_m") for row in candidates if row.get("device") == device and row.get("color") == "red" and row.get("world_xyz_m")]
        if points:
            values = np.asarray(points, dtype=float)
            ax_map.scatter(values[:, 0], values[:, 1], s=20, marker=marker, color=color, alpha=0.55, label=f"{device.upper()} red candidates")
    ax_map.set(title="World-frame paths and red candidate positions", xlabel="CARLA world X (m)", ylabel="CARLA world Y (m)")
    ax_map.axis("equal")
    ax_map.grid(alpha=0.25)
    ax_map.legend(fontsize=8, loc="best")

    for device, color in (("uav", "#7651a6"), ("ugv", "#168a73")):
        rows = read_jsonl(run_dir / "perception" / f"{device}_target_candidates_synchronized.jsonl")
        times, errors = [], []
        if target_truth_xyz is not None:
            for row in rows:
                red = [candidate for candidate in row.get("candidates", []) if candidate.get("color") == "red" and candidate.get("world_position_xyz") is not None]
                if not red:
                    continue
                truth = np.asarray(target_truth_xyz, dtype=float)
                best = min(red, key=lambda item: float(np.linalg.norm(np.asarray(item["world_position_xyz"], dtype=float)[:2] - truth[:2])))
                times.append(float(row["timestamp_s"]))
                errors.append(float(np.linalg.norm(np.asarray(best["world_position_xyz"], dtype=float)[:2] - truth[:2])))
        if times:
            ax_error.plot(times, errors, color=color, marker=".", ms=4, lw=1.2, label=device.upper())
    ax_error.axhline(5.0, color="#d19b25", ls="--", lw=1, label="5 m reference")
    ax_error.set(title="Nearest red-candidate position error (post-run truth)", xlabel="Simulation time (s)", ylabel="Horizontal error (m)")
    ax_error.grid(alpha=0.25)
    ax_error.legend(fontsize=8)

    for device, color in (("uav", "#7651a6"), ("ugv", "#168a73")):
        rows = read_jsonl(run_dir / "perception" / f"{device}_target_candidates_synchronized.jsonl")
        if rows:
            ax_count.plot([float(row["timestamp_s"]) for row in rows], [int(row["candidate_count"]) for row in rows], color=color, marker=".", ms=4, lw=1, label=device.upper())
    ax_count.set(title="Candidates per synchronized inference frame", xlabel="Simulation time (s)", ylabel="Candidate count")
    ax_count.grid(alpha=0.25)
    ax_count.legend(fontsize=8)

    indexed: dict[tuple[str, int], list[dict]] = {}
    for row in candidates:
        indexed.setdefault((str(row["device"]), int(row["frame_id"])), []).append(row)
    persisted_frames = set()
    index_path = run_dir / "synchronization" / "frame_index.csv"
    if index_path.exists():
        with index_path.open(encoding="utf-8-sig", newline="") as handle:
            persisted_frames = {
                int(row["frame"]) for row in csv.DictReader(handle)
                if int(row.get("sensor_files_saved", "1")) == 1
            }
    shared_candidate_frames = sorted(
        frame for frame in {frame for device, frame in indexed if device == "uav"}
        if ("ugv", frame) in indexed
        and indexed[("uav", frame)]
        and indexed[("ugv", frame)]
        and frame in persisted_frames
    )
    overlay_images = []
    chosen_frame = shared_candidate_frames[len(shared_candidate_frames) // 2] if shared_candidate_frames else None
    if chosen_frame is not None:
        for device, color in (("uav", (150, 65, 180)), ("ugv", (20, 135, 100))):
            image_path = run_dir / "sensors" / device / "rgb" / f"{chosen_frame:08d}.png"
            image = cv2.imread(str(image_path))
            if image is None:
                continue
            for item in indexed[(device, chosen_frame)]:
                x1, y1, x2, y2 = map(int, item["bbox_xyxy"])
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
                text = f"{item.get('category')} / {item.get('color')} z={item.get('depth_m') or float('nan'):.1f}m"
                cv2.putText(image, text, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
            output = preview / f"{device}_candidate_overlay.png"
            cv2.imwrite(str(output), image)
            overlay_images.append((device.upper(), image))
    if overlay_images:
        images = [cv2.resize(image, (520, 390)) for _, image in overlay_images]
        canvas = np.full((430, 1040, 3), 248, dtype=np.uint8)
        for index, ((label, _), image) in enumerate(zip(overlay_images, images)):
            x = index * 520
            canvas[35:425, x:x + 520] = image
            cv2.putText(canvas, label, (x + 12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (35, 45, 65), 2, cv2.LINE_AA)
        ax_overlay.imshow(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
        ax_overlay.set_title(f"Same CARLA frame {chosen_frame}: detector boxes and RGB-D candidates")
    else:
        ax_overlay.text(0.5, 0.5, "No persisted common frame had candidates from both devices", ha="center", va="center", wrap=True)
        ax_overlay.set_title("Candidate overlay")
    ax_overlay.axis("off")

    joint = int(evaluation.get("joint_target_candidate_frames", 0))
    candidate_pairs = int(evaluation.get("candidate_only_spatial_pair_count", 0))
    median = evaluation.get("cross_device_position_error_median")
    p90 = evaluation.get("cross_device_position_error_p90")
    fig.suptitle("CI-E6 UAV–UGV RGB-D Candidate Alignment", fontsize=15, fontweight="bold")
    fig.text(
        0.5,
        0.92,
        f"joint target frames: {joint} | candidate-only pairs: {candidate_pairs} | "
        f"cross-device position difference median/P90: "
        f"{float(median):.2f} / {float(p90):.2f} m" if median is not None and p90 is not None
        else f"joint target frames: {joint} | candidate-only pairs: {candidate_pairs}",
        ha="center",
        fontsize=10,
        color="#45566d",
    )
    output = preview / "candidate_alignment_summary.png"
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)
    manifest = {
        "run_dir": str(run_dir),
        "summary_plot": str(output),
        "uav_candidate_overlay": str(preview / "uav_candidate_overlay.png") if (preview / "uav_candidate_overlay.png").exists() else None,
        "ugv_candidate_overlay": str(preview / "ugv_candidate_overlay.png") if (preview / "ugv_candidate_overlay.png").exists() else None,
        "cross_device_position_error_median": median,
        "cross_device_position_error_p90": p90,
        "joint_target_candidate_frames": joint,
        "candidate_only_spatial_pair_count": candidate_pairs,
        "online_ground_truth_reads": 0,
        "ground_truth_used_for_evaluation_only": True,
    }
    (preview / "candidate_alignment_visualization_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    readme_path = run_dir / "README.md"
    if readme_path.exists():
        readme = readme_path.read_text(encoding="utf-8")
        readme = readme.replace(
            "# CI-E5 UGV Patrol and Local RGB-D Search Output",
            "# CI-E6 Dual-device Candidate Alignment Output",
        )
        old_section = "## UGV local RGB-D search (observation-only)"
        if old_section in readme:
            readme = readme[: readme.index(old_section)].rstrip()
        if "## Dual-device candidate alignment (observation-only)" not in readme:
            readme += """

## Dual-device candidate alignment (observation-only)

- `perception/candidate_records.jsonl`: unified UAV/UGV candidate records in CARLA world XYZ metres
- `perception/candidate_frames.jsonl`: exact common frame/time and sensor-pose records, including empty candidate frames
- `perception/uav_target_candidates_synchronized.jsonl`, `perception/ugv_target_candidates_synchronized.jsonl`: per-device RGB-D candidates and independent temporal histories
- `ground_truth/evaluation_only/dual_device_candidate_alignment_evaluation.json`: target-relative localization and cross-device coordinate error, evaluated after the run only
- `ground_truth/evaluation_only/candidate_only_spatial_pairs.jsonl`: same-frame UAV/UGV spatial pairs, computed without target truth
- `preview/candidate_alignment_summary.png`: paths, post-run position errors, candidate counts and synchronized RGB candidate overlays
- `preview/synchronized_multiview_replay.mp4`: sparse synchronized four-sensor plus map replay

No target truth, candidate matching, or communication is used online to control the UGV in this experiment.
"""
        readme_path.write_text(readme, encoding="utf-8")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
