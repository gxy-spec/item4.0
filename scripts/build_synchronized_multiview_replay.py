#!/usr/bin/env python3
"""Create a synchronized map + UAV/UGV RGB/depth replay from persisted frames."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def geometry(config: dict, map_image: np.ndarray) -> dict[str, float]:
    points = config["region"]["boundary"]["points"]
    height, width = map_image.shape[:2]
    dark = np.all(map_image < 90, axis=2)
    row_counts = dark.sum(axis=1)
    candidate_rows = np.flatnonzero(
        (row_counts > width * 0.40)
        & (np.arange(height) > height * 0.04)
        & (np.arange(height) < height * 0.96)
    )
    if len(candidate_rows) < 2:
        raise RuntimeError("Could not locate the plotted map axes in trajectory_map.png")
    top_row = int(candidate_rows[0])
    horizontal_axis_pixels = np.flatnonzero(dark[top_row])
    if len(horizontal_axis_pixels) < 2:
        raise RuntimeError("Could not locate the horizontal map axis in trajectory_map.png")
    return {
        "x_min": min(point[0] for point in points) - 8.0,
        "x_max": max(point[0] for point in points) + 8.0,
        "y_min": min(point[1] for point in points) - 8.0,
        "y_max": max(point[1] for point in points) + 8.0,
        "left": float(horizontal_axis_pixels[0]),
        "right": float(horizontal_axis_pixels[-1]),
        "top": float(candidate_rows[0]),
        "bottom": float(candidate_rows[-1]),
    }


def world_to_pixel(x: float, y: float, geo: dict[str, float]) -> tuple[int, int]:
    px = geo["left"] + (x - geo["x_min"]) / (geo["x_max"] - geo["x_min"]) * (geo["right"] - geo["left"])
    py = geo["bottom"] - (y - geo["y_min"]) / (geo["y_max"] - geo["y_min"]) * (geo["bottom"] - geo["top"])
    return int(round(px)), int(round(py))


def load_states(path: Path) -> dict[int, dict[str, list[float]]]:
    states: dict[int, dict[str, list[float]]] = defaultdict(dict)
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            state = json.loads(line)
            states[int(state["frame"])][state["role"]] = [float(state["x"]), float(state["y"])]
    return states


def draw_actor_markers(image, states: dict[str, list[float]], geo: dict[str, float]) -> None:
    for role, xy in states.items():
        point = world_to_pixel(xy[0], xy[1], geo)
        if role == "uav":
            cv2.drawMarker(image, point, (170, 40, 150), cv2.MARKER_TRIANGLE_UP, 24, 4, cv2.LINE_AA)
        elif role == "ugv":
            cv2.drawMarker(image, point, (210, 90, 20), cv2.MARKER_SQUARE, 20, 4, cv2.LINE_AA)
        elif role == "target":
            cv2.drawMarker(image, point, (30, 30, 210), cv2.MARKER_STAR, 24, 4, cv2.LINE_AA)
        elif role.startswith("vehicle"):
            cv2.circle(image, point, 7, (20, 140, 225), -1, cv2.LINE_AA)
        elif role.startswith("pedestrian"):
            cv2.circle(image, point, 6, (100, 160, 35), -1, cv2.LINE_AA)


def fit_panel(image, width: int, height: int):
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(image, (int(image.shape[1] * scale), int(image.shape[0] * scale)), interpolation=cv2.INTER_AREA)
    panel = 255 * np.ones((height, width, 3), dtype="uint8")
    x = (width - resized.shape[1]) // 2
    y = (height - resized.shape[0]) // 2
    panel[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
    return panel


def put_label(canvas, text: str, x: int, y: int, scale: float = 0.75) -> None:
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (27, 42, 64), 2, cv2.LINE_AA)


def read_jsonl(path: Path) -> list[dict]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def overlay_candidates(image: np.ndarray, candidates: list[dict], target_xyz: list[float] | None,
                       role: str, sent: set[tuple[str, int, str]], selected: set[tuple[str, int, str]],
                       frame: int) -> np.ndarray:
    result = image.copy()
    height, width = result.shape[:2]
    for candidate in candidates:
        box = candidate.get("bbox_xyxy")
        if not box or len(box) != 4:
            continue
        x1, y1, x2, y2 = [int(round(float(v))) for v in box]
        x1, x2 = max(0, min(width - 1, x1)), max(0, min(width - 1, x2))
        y1, y2 = max(0, min(height - 1, y1)), max(0, min(height - 1, y2))
        candidate_id = str(candidate.get("candidate_id", "?"))
        key = (role, frame, candidate_id)
        world = candidate.get("world_position_xyz")
        matches_target = bool(
            target_xyz is not None and world is not None
            and np.linalg.norm(np.asarray(world[:2], dtype=float) - np.asarray(target_xyz[:2], dtype=float)) <= 5.0
        )
        if key in selected:
            color = (30, 165, 35) if matches_target else (30, 30, 220)
            status = "SEL_OK" if matches_target else "SEL_ERR"
        elif key in sent:
            color = (0, 165, 235) if matches_target else (30, 30, 220)
            status = "SENT_OK" if matches_target else "SENT_ERR"
        else:
            color = (30, 165, 35) if matches_target else (135, 135, 135)
            status = "GT_NEAR" if matches_target else "CAND"
        cv2.rectangle(result, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        label = f"{candidate_id} {status}"
        cv2.putText(result, label, (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)
    return result


def main() -> int:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    config = yaml.safe_load((run_dir / "resolved_config.yaml").read_text(encoding="utf-8"))
    states = load_states(run_dir / "actors" / "actor_states.jsonl")
    with (run_dir / "synchronization" / "frame_index.csv").open(encoding="utf-8-sig", newline="") as handle:
        frame_rows = list(csv.DictReader(handle))
    all_synchronized_frames = len(frame_rows)
    # Compact batch runs process every synchronized frame but only persist a
    # fixed-rate subset of sensor files.  Replay that subset without weakening
    # the exact-frame alignment requirement.
    frame_rows = [
        row for row in frame_rows
        if int(row.get("sensor_files_saved", "1")) == 1
    ]
    safety_by_frame: dict[int, dict[str, str]] = {}
    safety_path = run_dir / "safety" / "safety_state.csv"
    if safety_path.exists():
        with safety_path.open(encoding="utf-8-sig", newline="") as handle:
            safety_by_frame = {int(row["frame"]): row for row in csv.DictReader(handle)}
    task_by_frame: dict[int, dict[str, str]] = {}
    task_path = run_dir / "task" / "state_timeline.csv"
    if task_path.exists():
        with task_path.open(encoding="utf-8-sig", newline="") as handle:
            task_by_frame = {int(row["frame"]): row for row in csv.DictReader(handle)}
    oracle_mode = config.get("experiment_type") == "oracle_closed_loop_acceptance"
    perception_mode = config.get("experiment_type") == "perception_closed_loop_acceptance"
    if not frame_rows:
        raise RuntimeError("No synchronized frames found")
    if any(int(row["frame_spread"]) != 0 for row in frame_rows):
        raise RuntimeError("Replay input contains non-zero four-stream frame spread")

    background = cv2.imread(str(run_dir / "preview" / "trajectory_map.png"))
    if background is None:
        raise FileNotFoundError("trajectory_map.png is missing")
    geo = geometry(config, background)
    output = args.output.resolve() if args.output else run_dir / "preview" / "synchronized_multiview_replay.mp4"
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), float(args.fps), (1920, 1080))
    if not writer.isOpened():
        raise RuntimeError("Could not open MP4 writer")

    panel_width, panel_height = 445, 250
    panel_positions = {
        "uav_rgb": (970, 135),
        "uav_depth": (1445, 135),
        "ugv_rgb": (970, 505),
        "ugv_depth": (1445, 505),
    }
    labels = {
        "uav_rgb": "UAV RGB",
        "uav_depth": "UAV depth",
        "ugv_rgb": "UGV RGB",
        "ugv_depth": "UGV depth",
    }
    first_timestamp = float(frame_rows[0]["timestamp"])
    saved_times = [float(row["timestamp"]) for row in frame_rows]
    sample_interval = float(np.median(np.diff(saved_times))) if len(saved_times) > 1 else 0.0
    candidates_by_role: dict[str, dict[int, list[dict]]] = {"uav": {}, "ugv": {}}
    for role in ("uav", "ugv"):
        for record in read_jsonl(run_dir / "perception" / f"{role}_target_candidates_synchronized.jsonl"):
            candidates_by_role[role][int(record["frame_id"])] = record.get("candidates", [])
    sent = {
        (str(message.get("source")), int(message.get("source_frame", -1)), str(message.get("candidate_id")))
        for message in read_jsonl(run_dir / "communication" / "messages.jsonl")
        if message.get("event") == "sent"
    }
    selected = {
        (str(message.get("source")), int(message.get("source_frame", -1)), str(message.get("candidate_id")))
        for message in read_jsonl(run_dir / "communication" / "selected_target_states.jsonl")
        if message.get("event") == "selected"
    }
    for index, row in enumerate(frame_rows):
        frame = int(row["frame"])
        map_frame = background.copy()
        draw_actor_markers(map_frame, states.get(frame, {}), geo)
        map_panel = fit_panel(map_frame, 910, 930)
        canvas = 255 * np.ones((1080, 1920, 3), dtype="uint8")
        canvas[95:1025, 25:935] = map_panel
        put_label(canvas, "GLOBAL TRAJECTORY + CURRENT ACTOR POSITIONS", 35, 72, 0.72)
        elapsed = float(row["timestamp"]) - first_timestamp
        replay_name = (
            "S0 ORACLE synchronized replay"
            if oracle_mode
            else ("S1 PERCEPTION synchronized replay" if perception_mode else "CI-E1 synchronized replay")
        )
        replay_name = str(config.get("experiment_id", config.get("experiment", {}).get("id", replay_name)))
        put_label(canvas, f"{replay_name}   t={float(row['timestamp']):06.1f}s   frame={frame}", 970, 67, 0.72)
        if sample_interval > 1.0:
            put_label(canvas, f"SPARSE SAVED SAMPLES: median interval {sample_interval:.1f}s; not continuous video", 970, 100, 0.53)
        if oracle_mode:
            cv2.rectangle(canvas, (20, 8), (1900, 42), (34, 34, 190), -1)
            cv2.putText(
                canvas,
                "ORACLE - CARLA GROUND-TRUTH TARGET POSITION - NOT A PERCEPTION RESULT",
                (130, 33),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.78,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        elif perception_mode:
            cv2.rectangle(canvas, (20, 8), (1900, 42), (105, 63, 20), -1)
            cv2.putText(
                canvas,
                "S1 PERCEPTION - UAV RGB/DEPTH MESSAGE + UGV LOCAL CONFIRMATION - ORACLE=FALSE",
                (105, 33),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        paths = {
            "uav_rgb": run_dir / "sensors" / "uav" / "rgb" / f"{frame:08d}.png",
            "uav_depth": run_dir / "sensors" / "uav" / "depth_colour" / f"{frame:08d}.png",
            "ugv_rgb": run_dir / "sensors" / "ugv" / "rgb" / f"{frame:08d}.png",
            "ugv_depth": run_dir / "sensors" / "ugv" / "depth_colour" / f"{frame:08d}.png",
        }
        frame_actors = states.get(frame, {})
        target_xyz = frame_actors.get("target")
        for name, path in paths.items():
            image = cv2.imread(str(path))
            if image is None:
                raise FileNotFoundError(f"Missing replay frame: {path}")
            if name.endswith("_rgb"):
                role = "uav" if name.startswith("uav") else "ugv"
                image = overlay_candidates(
                    image, candidates_by_role[role].get(frame, []), target_xyz, role, sent, selected, frame
                )
            x, y = panel_positions[name]
            put_label(canvas, labels[name], x, y - 18, 0.72)
            canvas[y : y + panel_height, x : x + panel_width] = fit_panel(image, panel_width, panel_height)

        put_label(canvas, "All five panels use this exact CARLA frame ID (4 sensor streams + map)", 970, 800, 0.65)
        task = task_by_frame.get(frame)
        if (oracle_mode or perception_mode) and task:
            confirmation_text = ""
            if perception_mode:
                confirmation_text = f" | verified={task.get('target_verified', 'False')}"
            put_label(
                canvas,
                f"TASK STATE: {task['state']} | message_received={task['message_received']}{confirmation_text}",
                970,
                830,
                0.62,
            )
        safety = safety_by_frame.get(frame)
        if safety:
            target_distance = float(safety["ugv_target_distance_m"])
            target_distance_text = f"{target_distance:.1f} m" if np.isfinite(target_distance) else "not selected"
            uav_clearance = float(safety["uav_central_clearance_m"])
            put_label(
                canvas,
                f"UGV {float(safety['ugv_speed_mps']):.1f} m/s | target {target_distance_text} | {safety['ugv_safety_mode']}",
                970,
                862,
                0.60,
            )
            put_label(
                canvas,
                f"UAV altitude {float(safety['uav_altitude_m']):.1f} m | central clearance {uav_clearance:.1f} m",
                970,
                898,
                0.60,
            )
            put_label(
                canvas,
                f"Collisions: UGV {safety['ugv_collision_count']} | UAV {safety['uav_collision_count']}",
                970,
                934,
                0.60,
            )
        else:
            put_label(canvas, "Purple: UAV   Blue: UGV   Red star: target", 970, 865, 0.62)
            put_label(canvas, "Orange: distractor vehicles   Green: pedestrians", 970, 905, 0.62)
        put_label(canvas, "Map truth and candidate GT labels are offline annotations only; never control inputs", 970, 948, 0.48)
        progress_x = int(970 + 850 * (index + 1) / len(frame_rows))
        cv2.rectangle(canvas, (970, 960), (1820, 974), (220, 225, 232), -1)
        cv2.rectangle(canvas, (970, 960), (progress_x, 974), (160, 65, 120), -1)
        put_label(canvas, f"sample {index + 1}/{len(frame_rows)}", 970, 1018, 0.66)
        writer.write(canvas)
        if (index + 1) % 100 == 0:
            print(f"Rendered {index + 1}/{len(frame_rows)}", flush=True)
    writer.release()

    manifest = {
        "input_run": str(run_dir),
        "output_video": str(output),
        "frame_count": len(frame_rows),
        "all_synchronized_frame_count": all_synchronized_frames,
        "storage_profile": "compact" if len(frame_rows) < all_synchronized_frames else "full",
        "fps": float(args.fps),
        "duration_seconds": len(frame_rows) / float(args.fps),
        "median_persisted_sample_interval_seconds": sample_interval,
        "sparse_sensor_archive": bool(sample_interval > 1.0),
        "playback_note": "Synchronized among persisted samples; skipped simulation intervals are not interpolated.",
        "frame_alignment": "exact CARLA frame ID across map, UAV RGB/depth and UGV RGB/depth",
        "first_frame": int(frame_rows[0]["frame"]),
        "last_frame": int(frame_rows[-1]["frame"]),
    }
    output.with_suffix(".json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
