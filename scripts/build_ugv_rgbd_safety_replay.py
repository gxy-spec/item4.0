#!/usr/bin/env python3
"""Build a compact, timestamped UGV RGB-D replay from persisted local pairs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=1.0)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    index_path = run_dir / "synchronization" / "ugv_rgbd_frame_index.csv"
    observation_path = run_dir / "safety" / "ugv_obstacle_perception.csv"

    with index_path.open(encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    rows = [
        row for row in rows
        if row.get("sensor_files_saved") == "1"
        and (run_dir / "sensors" / "ugv" / "rgb" / f"{int(row['frame']):08d}.png").is_file()
        and (run_dir / "sensors" / "ugv" / "depth_colour" / f"{int(row['frame']):08d}.png").is_file()
    ]
    if not rows:
        raise RuntimeError(f"No persisted UGV RGB-D pairs found in {index_path}")

    observations: dict[int, dict[str, str]] = {}
    if observation_path.exists():
        with observation_path.open(encoding="utf-8-sig", newline="") as handle:
            for row in csv.DictReader(handle):
                if row.get("sensor_frame"):
                    observations[int(row["sensor_frame"])] = row

    detections_by_frame: dict[int, list[dict[str, str]]] = {}
    detection_path = run_dir / "safety" / "ugv_rgbd_hazard_detections.csv"
    if detection_path.exists():
        with detection_path.open(encoding="utf-8-sig", newline="") as handle:
            for row in csv.DictReader(handle):
                if row.get("frame") and row.get("bbox_xyxy"):
                    detections_by_frame.setdefault(int(row["frame"]), []).append(row)

    vehicle_states: dict[int, dict[str, str]] = {}
    safety_state_path = run_dir / "safety" / "safety_state.csv"
    if safety_state_path.exists():
        with safety_state_path.open(encoding="utf-8-sig", newline="") as handle:
            for row in csv.DictReader(handle):
                vehicle_states[int(row["frame"])] = row

    first_timestamp = float(rows[0]["timestamp"])
    output = run_dir / "preview" / "ugv_rgbd_safety_replay.mp4"
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (1600, 660))
    if not writer.isOpened():
        raise RuntimeError(f"Could not open video writer for {output}")

    try:
        for row in rows:
            frame = int(row["frame"])
            rgb_path = run_dir / "sensors" / "ugv" / "rgb" / f"{frame:08d}.png"
            depth_path = run_dir / "sensors" / "ugv" / "depth_colour" / f"{frame:08d}.png"
            rgb = cv2.imread(str(rgb_path))
            depth = cv2.imread(str(depth_path))
            if rgb is None or depth is None:
                raise FileNotFoundError(f"Missing UGV RGB-D image for frame {row['frame']}")
            for detection in detections_by_frame.get(frame, []):
                try:
                    x1, y1, x2, y2 = [int(round(float(value))) for value in json.loads(detection["bbox_xyxy"])]
                except (ValueError, TypeError, json.JSONDecodeError):
                    continue
                category = detection.get("category", "object")
                action = detection.get("decision", "clear")
                colour = (0, 0, 255) if action in {"pedestrian", "crossing_vehicle", "vehicle_conflict"} else (0, 200, 255)
                label = f"{category} {float(detection.get('confidence') or 0.0):.2f}"
                cv2.rectangle(rgb, (x1, y1), (x2, y2), colour, 2)
                cv2.putText(rgb, label, (max(2, x1), max(18, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, colour, 2, cv2.LINE_AA)
            rgb = cv2.resize(rgb, (800, 600), interpolation=cv2.INTER_AREA)
            depth = cv2.resize(depth, (800, 600), interpolation=cv2.INTER_AREA)
            canvas = np.zeros((660, 1600, 3), dtype=np.uint8)
            canvas[60:660, 0:800] = rgb
            canvas[60:660, 800:1600] = depth
            timestamp = float(row["timestamp"])
            elapsed = timestamp - first_timestamp
            observation = observations.get(frame, {})
            state = observation.get("sensor_state", "not logged")
            nearest = observation.get("nearest_obstacle_m", "")
            nearest_text = f" | nearest obstacle {float(nearest):.2f} m" if nearest not in ("", "nan") else ""
            vehicle = vehicle_states.get(frame, {})
            speed = vehicle.get("ugv_speed_mps", "")
            mode = vehicle.get("ugv_safety_mode", "")
            motion_text = f" | {mode} | speed {float(speed):.2f} m/s" if speed not in ("", "nan") else ""
            cv2.putText(canvas, f"UGV RGB-D safety replay | t={elapsed:05.1f}s | CARLA frame={frame}{motion_text}", (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.78, (245, 245, 245), 2, cv2.LINE_AA)
            cv2.putText(canvas, "UGV RGB", (20, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (20, 20, 20), 2, cv2.LINE_AA)
            cv2.putText(canvas, f"Depth color | safety={state}{nearest_text}", (820, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 2, cv2.LINE_AA)
            writer.write(canvas)
    finally:
        writer.release()

    report = {
        "run_directory": str(run_dir),
        "output_video": str(output),
        "frame_count": len(rows),
        "fps": args.fps,
        "duration_seconds": len(rows) / args.fps,
        "frame_alignment": "exact CARLA frame ID for each UGV RGB-depth pair",
        "sampling": "persisted compact sensor cadence; not every simulation tick",
    }
    (run_dir / "preview" / "ugv_rgbd_safety_replay.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
