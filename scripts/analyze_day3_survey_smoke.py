#!/usr/bin/env python3
"""Post-run A-policy scene qualification; target truth is evaluation-only."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import yaml


def jsonl(path: Path) -> list[dict]:
    return [json.loads(s) for s in path.read_text(encoding="utf-8").splitlines() if s.strip()] if path.exists() else []


def csv_rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def dump(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def distance_to_route(point, route) -> float:
    """Distance to finite segments, not an infinite extrapolation of the patrol."""
    p = np.asarray(point[:2], dtype=float)
    best = math.inf
    for a, b in zip(route, route[1:]):
        a, b = np.asarray(a[:2], dtype=float), np.asarray(b[:2], dtype=float)
        d = b - a
        fraction = float(np.clip(np.dot(p - a, d) / max(np.dot(d, d), 1e-12), 0, 1))
        best = min(best, float(np.linalg.norm(p - a - fraction * d)))
    return best


def first_correct_message(messages, device, target_xyz, radius, t0):
    rows = [m for m in messages if m.get("source") == device and m.get("world_position_xyz")
            and math.dist(m["world_position_xyz"][:2], target_xyz[:2]) <= radius]
    return min(float(m["source_timestamp_s"]) - t0 for m in rows) if rows else None


def survey_geometry(output: Path) -> list[dict]:
    if output.exists():
        return json.loads(output.read_text(encoding="utf-8"))["lanes"]
    import carla
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(30.0)
    world_map = client.get_world().get_map()
    lanes = []
    for start, end in world_map.get_topology():
        current, points, left, right = start, [], [], []
        visited = set()
        for _ in range(300):
            if current.id in visited:
                break
            visited.add(current.id)
            tr = current.transform
            x, y = tr.location.x, tr.location.y
            yaw = math.radians(tr.rotation.yaw)
            half = current.lane_width / 2
            side = np.asarray([-math.sin(yaw), math.cos(yaw)]) * half
            points.append([x, y])
            left.append([x + side[0], y + side[1]])
            right.append([x - side[0], y - side[1]])
            if tr.location.distance(end.transform.location) < 2:
                break
            options = current.next(2.0)
            if not options:
                break
            current = min(options, key=lambda wp: wp.transform.location.distance(end.transform.location))
        if len(points) >= 2:
            lanes.append({"center": points, "polygon": left + right[::-1], "is_junction": start.is_junction})
    dump(output, {"coordinate_frame": "CARLA_WORLD_XY_M", "source": "static road geometry; never used as target observation", "lanes": lanes})
    return lanes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args()
    run = args.run_dir.resolve()
    preview = run / "preview"
    preview.mkdir(exist_ok=True)
    config = yaml.safe_load((run / "resolved_config.yaml").read_text(encoding="utf-8"))
    limits = config["scenario_survey"]
    native_report = json.loads((run / "validation_report.json").read_text(encoding="utf-8"))
    trajectories = {d: csv_rows(run / "trajectories" / f"{d}_trajectory.csv") for d in ("ugv", "uav", "target")}
    t0 = float(trajectories["ugv"][0]["timestamp"])
    target = [float(trajectories["target"][-1][k]) for k in ("x", "y", "z")]
    route = json.loads((run / "planning" / "ugv_patrol_route_executed.json").read_text(encoding="utf-8"))["points_xyz"]
    messages = jsonl(run / "communication" / "messages.jsonl")
    events = jsonl(run / "communication" / "communication_events.jsonl")
    selected = jsonl(run / "communication" / "selected_target_states.jsonl")
    route_events = jsonl(run / "planning" / "target_route_events.jsonl")
    pairs = csv_rows(run / "synchronization" / "frame_index.csv")
    safety = csv_rows(run / "safety" / "safety_state.csv")
    guard = csv_rows(run / "safety" / "ugv_obstacle_perception.csv")
    uav_meta = jsonl(run / "sensors" / "uav" / "metadata" / "frames.jsonl")
    ugv_meta = jsonl(run / "sensors" / "ugv" / "metadata" / "frames.jsonl")
    first = {d: first_correct_message(messages, d, target, limits["match_radius_m"], t0) for d in ("uav", "ugv")}
    lead = first["ugv"] - first["uav"] if all(v is not None for v in first.values()) else None
    camera_errors = [float(m["camera_own_vehicle_position_error_m"]) for m in uav_meta if "camera_own_vehicle_position_error_m" in m]
    deviations = [distance_to_route([float(r["x"]), float(r["y"])], route) for r in trajectories["ugv"]]
    paths = {d: sum(math.dist([float(a[k]) for k in ("x", "y")], [float(b[k]) for k in ("x", "y")])
                    for a, b in zip(rows, rows[1:])) for d, rows in trajectories.items()}
    expected = round(config["simulation"]["duration_seconds"] * config["sensors"]["frequency_hz"])
    frame_errors = [int(r["frame_spread"]) for r in pairs]
    time_errors = []
    ugv_by_frame = {m["frame"]: m for m in ugv_meta}
    for m in uav_meta:
        other = ugv_by_frame.get(m["frame"])
        if other:
            time_errors.append(abs(m["timestamp"] - other["timestamp"]))
    # The observer is evaluated before tick N+1; the control log records its
    # input explicitly after that tick. Joining observer frame N to control
    # frame N is off by one, and simultaneous pedestrian stops have a different
    # label despite issuing the same full-brake command.
    stale_rows = [s for s in safety if s.get("ugv_obstacle_sensor_state") == "SENSOR_STALE"]
    def is_stopping(mode):
        return mode in {"sensor_stale_stop", "depth_obstacle_stop", "target_stop"} or mode.startswith("wait_")
    stale_brake_errors = [s for s in stale_rows if not is_stopping(s["ugv_safety_mode"])]
    stale_other_hazard_ticks = sum(s["ugv_safety_mode"].startswith("wait_") for s in stale_rows)
    collision_counts = {d: len(json.loads((run / "safety" / f"{d}_collision_events.json").read_text(encoding="utf-8"))) for d in ("ugv", "uav")}
    checks = []
    def check(name, passed, value, criterion):
        checks.append({"name": name, "passed": bool(passed), "value": value, "criterion": criterion})
    check("native_dynamic_acceptance", native_report["status"] == "PASS", native_report["status"], "PASS without changing native results")
    separation = distance_to_route(target, route)
    check("target_outside_A_patrol_success_radius", separation > limits["target_success_radius_m"], separation, f"> {limits['target_success_radius_m']} m")
    check("A_never_uses_target_replanning", not selected and not any(e.get("event") == "target_route_planned_from_selected_message" for e in route_events), len(selected), "0 selected target commands and 0 target replans")
    polygon = config["region"]["boundary"]["points"]
    bx = [min(p[0] for p in polygon), max(p[0] for p in polygon)]
    by = [min(p[1] for p in polygon), max(p[1] for p in polygon)]
    outside_messages = [m for m in messages if not (bx[0] <= m["world_position_xyz"][0] <= bx[1] and by[0] <= m["world_position_xyz"][1] <= by[1])]
    check("messages_remain_inside_task_region", bool(messages) and not outside_messages, len(outside_messages), "0 out-of-task-area messages; task boundary only, no target truth")
    check("A_follows_reference_patrol", max(deviations) <= limits["maximum_patrol_deviation_m"], max(deviations), f"max <= {limits['maximum_patrol_deviation_m']} m; safety remains active")
    check("UGV_moves_from_start", paths["ugv"] >= 30, paths["ugv"], ">=30 m")
    for d in ("uav", "ugv"):
        check(f"{d}_effective_target_discovery", first[d] is not None, first[d], "message passes live gating and post-run target XY error <=5 m")
        check(f"{d}_collision_free", collision_counts[d] == 0, collision_counts[d], "0 collisions in formal acquisition")
    check("UAV_earlier_discovery", lead is not None and lead >= limits["minimum_uav_discovery_lead_s"], lead, f">={limits['minimum_uav_discovery_lead_s']} s")
    check("four_stream_common_frame_coverage", len(pairs) / expected >= 0.95, len(pairs) / expected, ">=95%")
    check("source_frame_exact_alignment", bool(frame_errors) and max(frame_errors) == 0, max(frame_errors) if frame_errors else None, "frame spread=0")
    check("source_time_exact_alignment", bool(time_errors) and max(time_errors) <= 1e-6, max(time_errors) if time_errors else None, "same source timestamp; not receipt/wall time")
    check("UAV_camera_follows_actual_airframe", len(camera_errors) == len(uav_meta) and max(camera_errors) <= limits["maximum_camera_own_vehicle_error_m"], max(camera_errors) if camera_errors else None, f"every frame own-pose error <= {limits['maximum_camera_own_vehicle_error_m']} m")
    check("RGBD_safety_continuous", len(guard) / len(safety) >= 0.95, len(guard) / len(safety), ">=95% safety control ticks with guard evaluation")
    check("observed_stale_is_protected", bool(stale_rows) and not stale_brake_errors,
          {"stale_ticks": len(stale_rows), "unprotected_ticks": len(stale_brake_errors), "overlap_other_hazard_stop_ticks": stale_other_hazard_ticks,
           "max_speed_during_stale_mps": max(float(s["ugv_speed_mps"]) for s in stale_rows) if stale_rows else None},
          "SENSOR_STALE always coincides with a full-brake control branch; not an isolated causal failure test")
    lanes = survey_geometry(run / "planning" / "scene_road_geometry.json")
    font_path = Path("C:/Windows/Fonts/msyh.ttc")
    if font_path.exists():
        font_manager.fontManager.addfont(str(font_path))
        font_name = font_manager.FontProperties(fname=str(font_path)).get_name()
    else:
        font_name = "DejaVu Sans"
    plt.rcParams.update({"font.family": font_name, "font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "axes.unicode_minus": False})
    colors = {"ugv": "#2877ad", "uav": "#8a55a2"}
    fig, ax = plt.subplots(figsize=(8, 8), dpi=180)
    for lane in lanes:
        poly = np.asarray(lane["polygon"])
        ax.fill(poly[:, 0], poly[:, 1], color="#e0e3e6", lw=0, zorder=0)
    rxy = np.asarray(route)
    ax.plot(rxy[:, 0], rxy[:, 1], "--", color="black", lw=2, label="A组预设巡检路线", zorder=3)
    for d in ("ugv", "uav"):
        xy = np.asarray([[float(r[k]) for k in ("x", "y")] for r in trajectories[d]])
        ax.plot(xy[:, 0], xy[:, 1], color=colors[d], lw=1.7, label=f"{d.upper()}实际轨迹", zorder=4)
        ax.scatter(*xy[0], marker="o", s=45, color=colors[d], zorder=5)
        ax.scatter(*xy[-1], marker="s", s=40, color=colors[d], zorder=5)
    ax.scatter(*target[:2], marker="*", s=170, color="#bd3b3b", label="目标车（仅事后评价）", zorder=6)
    ax.add_patch(plt.Circle(target[:2], limits["target_success_radius_m"], fill=False, color="#bd3b3b", ls=":", lw=1.2, label="10 m任务接近半径"))
    for d in ("uav", "ugv"):
        if first[d] is not None:
            rr = min(trajectories[d], key=lambda r: abs(float(r["timestamp"]) - t0 - first[d]))
            xy = [float(rr["x"]), float(rr["y"])]
            ax.scatter(*xy, marker="D", s=65, color=colors[d], edgecolors="white", zorder=7)
            ax.annotate(f"{d.upper()}首次有效发现\n{first[d]:.1f} s", xy, xytext=(8, 10), textcoords="offset points", color=colors[d], fontsize=9)
    ax.set(xlim=(-116, 35), ylim=(-77, 79), xlabel="CARLA世界X（m）", ylabel="CARLA世界Y（m）", title="第3天场景勘测：固定巡检与旁侧静止目标")
    ax.set_aspect("equal")
    ax.grid(alpha=0.18)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(preview / f"scene_survey_map.{ext}", bbox_inches="tight")
    plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(9, 6.5), dpi=180, sharex=True)
    times = [float(s["timestamp"]) - t0 for s in safety]
    axes[0].plot(times, [float(s["ugv_speed_mps"]) for s in safety], color=colors["ugv"], lw=1.5)
    axes[0].set(ylabel="UGV速度（m/s）", title="同步与安全验收：RGB-D持续保护，不绕过避障")
    interval = config["dynamic"]["ugv_safety"].get("test_sensor_stale_interval_s")
    if interval:
        axes[0].axvspan(*interval, color="#bf5656", alpha=0.18, label="注入深度失效")
    wait_times = [float(s["timestamp"]) - t0 for s in safety if s["ugv_safety_mode"].startswith("wait_")]
    if wait_times:
        axes[0].plot(wait_times, [0] * len(wait_times), "|", color="#808080", ms=9, label="其他危险停车（含行人等待）")
    axes[0].legend(fontsize=8)
    pose_times = [float(m["timestamp"]) - t0 for m in uav_meta]
    axes[1].plot(pose_times, camera_errors, color=colors["uav"], lw=1)
    axes[1].axhline(limits["maximum_camera_own_vehicle_error_m"], color="black", ls="--", lw=1)
    axes[1].set(ylabel="UAV相机-机体误差（m）", ylim=(0, 0.6))
    axes[2].plot([float(r["timestamp"]) - t0 for r in pairs], frame_errors, color="#27604a", lw=1)
    axes[2].set(ylabel="四端传感器帧号跨度", xlabel="仿真经过时间（s）", ylim=(-0.1, 1.1))
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(preview / f"synchronization_safety_checks.{ext}", bbox_inches="tight")
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(9, 5.2), dpi=180, sharex=True, gridspec_kw={"height_ratios": [1, 1.5]})
    rows_out = []
    for i, d in enumerate(("uav", "ugv")):
        for m in messages:
            if m["source"] != d:
                continue
            time = float(m["source_timestamp_s"]) - t0
            error = math.dist(m["world_position_xyz"][:2], target[:2])
            match = error <= limits["match_radius_m"]
            rows_out.append({"device": d, "frame": m["source_frame"], "time_s": time, "message_id": m["message_id"], "target_error_xy_m": error, "post_run_target_match": match})
            axes[0].plot(time, i, "|" if match else "x", color=colors[d] if match else "#b0b0b0", ms=8, mew=1)
            axes[1].plot(time, error, ".", color=colors[d], ms=3, alpha=0.8)
        if first[d] is not None:
            axes[0].annotate(f"首次 {first[d]:.1f} s", (first[d], i), xytext=(5, 10), textcoords="offset points", color=colors[d], fontsize=9)
    axes[0].set(yticks=[0, 1], yticklabels=["UAV", "UGV"], ylim=(-0.45, 1.55), title=f"有效目标发现时序 | UAV领先 {lead:.1f} s" if lead is not None else "有效目标发现时序 | 尚未形成双端发现")
    axes[0].text(0.98, 0.04, "彩色竖线：通过消息门控且匹配目标；灰叉：门控通过但目标匹配失败", transform=axes[0].transAxes, ha="right", fontsize=8)
    axes[1].axhline(limits["match_radius_m"], ls="--", color="black", lw=1, label="5 m事后匹配门限")
    axes[1].set(xlabel="仿真经过时间（s）", ylabel="候选与目标真值的XY距离（m）", xlim=(0, config["simulation"]["duration_seconds"]))
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(preview / f"visibility_timeline.{ext}", bbox_inches="tight")
    plt.close(fig)
    if rows_out:
        with (run / "ground_truth" / "evaluation_only" / "discovery_events.csv").open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows_out[0]))
            writer.writeheader()
            writer.writerows(rows_out)
    # Render map and sensors with one explicitly defined coordinate-to-pixel transform.
    geo = {"xmin": -116, "xmax": 35, "ymin": -77, "ymax": 79}
    def pixel(xy):
        return (int(55 + (float(xy[0]) - geo["xmin"]) / 151 * 780), int(955 - (float(xy[1]) - geo["ymin"]) / 156 * 810))
    map_bg = np.full((1080, 900, 3), 248, np.uint8)
    for lane in lanes:
        poly = np.asarray([pixel(p) for p in lane["polygon"]], np.int32)
        cv2.fillPoly(map_bg, [poly], (225, 228, 231))
    cv2.polylines(map_bg, [np.asarray([pixel(p) for p in route], np.int32)], False, (40, 40, 40), 2)
    cv2.drawMarker(map_bg, pixel(target), (60, 60, 190), cv2.MARKER_STAR, 25, 2)
    trajectory_by_frame = {d: {int(r["frame"]): r for r in trajectories[d]} for d in ("ugv", "uav")}
    safety_by_frame = {int(r["frame"]): r for r in safety}
    messages_by_frame = {(m["source"], int(m["source_frame"])): m for m in messages}
    replay_rows = [r for r in pairs if int(r.get("sensor_files_saved", 1)) == 1]
    fps = 1 / np.median(np.diff([float(r["timestamp"]) for r in replay_rows]))
    video_path = preview / "smoke_five_view.mp4"
    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (1920, 1080))
    if not writer.isOpened():
        raise RuntimeError("MP4 writer failed")
    rgb_centers = {}
    history = {d: [] for d in ("ugv", "uav")}
    def label(canvas, text, xy, scale=0.6):
        cv2.putText(canvas, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (40, 40, 40), 1, cv2.LINE_AA)
    for replay_index, r in enumerate(replay_rows):
        frame = int(r["frame"])
        canvas = np.full((1080, 1920, 3), 255, np.uint8)
        canvas[:, :900] = map_bg
        for d, col in (("ugv", (170, 110, 35)), ("uav", (150, 75, 130))):
            state = trajectory_by_frame[d].get(frame)
            if state is None:
                raise RuntimeError(f"Map state missing for sensor source frame {frame}")
            pt = pixel([state["x"], state["y"]])
            history[d].append(pt)
            if len(history[d]) > 1:
                cv2.polylines(canvas, [np.asarray(history[d], np.int32)], False, col, 3)
            cv2.circle(canvas, pt, 8, col, -1)
        for d, y in (("uav", 140), ("ugv", 530)):
            for modal, x in (("rgb", 930), ("depth_colour", 1430)):
                img = cv2.imread(str(run / "sensors" / d / modal / f"{frame:08d}.png"))
                if img is None:
                    raise FileNotFoundError(f"Missing source frame {frame}: {d}/{modal}")
                msg = messages_by_frame.get((d, frame))
                if modal == "rgb" and msg:
                    candidate_rows = jsonl(run / "perception" / f"{d}_target_candidates_synchronized.jsonl") if d not in rgb_centers else rgb_centers[d]
                    rgb_centers[d] = candidate_rows
                    cand = next((c for rr in candidate_rows if int(rr["frame_id"]) == frame for c in rr["candidates"] if c["candidate_id"] == msg["candidate_id"]), None)
                    if cand:
                        x1, y1, x2, y2 = map(int, cand["bbox_xyxy"])
                        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 190, 255), 2)
                img = cv2.resize(img, (460, 345), interpolation=cv2.INTER_AREA)
                canvas[y:y + 345, x:x + 460] = img
                label(canvas, f"{d.upper()} {'RGB + transmitted candidate' if modal == 'rgb' else 'metric depth (colour)'}", (x, y - 15), 0.58)
        s = safety_by_frame.get(frame, {})
        label(canvas, "DAY 3 | A: fixed patrol; no target-triggered replan", (930, 45), 0.72)
        label(canvas, f"Simulation t={float(r['timestamp']) - t0:.1f}s | all views source frame={frame}", (930, 80), 0.64)
        label(canvas, "Road surfaces (grey) | reference patrol (black)", (35, 55), 0.66)
        label(canvas, "UGV (blue) | UAV (purple) | target GT (red; evaluation only)", (35, 90), 0.59)
        label(canvas, f"UGV speed: {float(s.get('ugv_speed_mps', 0)):.2f} m/s | {s.get('ugv_safety_mode', 'unknown')}", (930, 935), 0.64)
        label(canvas, "Safety input: UGV RGB-D | collision bypass: disabled", (930, 975), 0.64)
        label(canvas, "Sparse 2 Hz replay at simulation speed; NOT a full raw sensor dataset", (930, 1020), 0.54)
        writer.write(canvas)
        if replay_index in {0, len(replay_rows) // 2, len(replay_rows) - 1}:
            cv2.imwrite(str(preview / f"smoke_replay_keyframe_{replay_index:03d}.png"), canvas)
    writer.release()
    check("five_view_source_frame_replay", len(replay_rows) > 0 and video_path.stat().st_size > 1000, len(replay_rows), "every replay sample has four images and actual poses with same source frame")
    summary = {
        "stage": "Day3 scene survey + single A-policy smoke", "seed": config["random_seed"],
        "status": "PASS" if all(c["passed"] for c in checks) else "FAIL", "run_dir": str(run),
        "scope": "Scene qualification only; no B/C task gain or physical-channel claim", "target_ground_truth_scope": "post-run matching/map/evaluation only",
        "truth_boundary_evidence": "validated config + sensor-only controller regression tests; not simulator-wide instrumentation",
        "scenario_scope": "single red static target, three non-red vehicles, two pedestrians; same-colour hard-negative discrimination is NOT qualified by this run",
        "limitations": ["Not an ABC task-performance comparison", "No same-colour hard-negative discrimination test",
                        "Depth failure overlaps another hazard stop; do not claim isolated failure braking causality" if stale_other_hazard_ticks else "No isolated failure-response claim"],
        "metrics": {"target_route_separation_m": separation, "first_valid_discovery_s": first, "uav_lead_s": lead,
                    "paths_m": paths, "max_A_patrol_deviation_m": max(deviations), "collisions": collision_counts,
                    "common_frame_count": len(pairs), "expected_sensor_frames": expected, "camera_actual_pose_error_max_m": max(camera_errors) if camera_errors else None,
                    "messages_sent": {d: sum(m["source"] == d for m in messages) for d in ("uav", "ugv")},
                    "messages_received": sum(e.get("event") == "received" for e in events), "raw_float_depth_saved": config["output"]["save_depth_float_m"],
                    "messages_post_run_matched_by_device": {d: sum(r["device"] == d and r["post_run_target_match"] for r in rows_out) for d in ("uav", "ugv")},
                    "messages_post_run_unmatched_by_device": {d: sum(r["device"] == d and not r["post_run_target_match"] for r in rows_out) for d in ("uav", "ugv")},
                    "replay_samples": len(replay_rows), "replay_fps": float(fps)}, "checks": checks,
        "artifacts": {"scene_map": str(preview / "scene_survey_map.png"), "visibility_timeline": str(preview / "visibility_timeline.png"),
                      "safety_sync_checks": str(preview / "synchronization_safety_checks.png"), "five_view_video": str(video_path)},
    }
    dump(run / "smoke_acceptance.json", summary)
    project = Path(__file__).resolve().parents[1]
    source_files = ["scripts/run_stage1_dynamic.py", "scripts/analyze_day3_survey_smoke.py",
                    "configs/experiments/ci_day3_survey_smoke.yaml", "src/safety/rgbd_obstacle_guard.py", "src/communication/persistent_target.py"]
    dump(run / "source_fingerprints.json", {
        "scope": "SHA256 of local working-tree source at acceptance time; not a clean Git-release claim",
        "files": {name: hashlib.sha256((project / name).read_bytes()).hexdigest() for name in source_files},
    })
    if summary["status"] == "PASS":
        dump(run.parent.parent / "latest_passed_smoke.json", {
            "run_dir": str(run), "acceptance_report": str(run / "smoke_acceptance.json"),
            "seed": config["random_seed"], "scope": summary["scope"], "artifacts": summary["artifacts"],
        })
    text = ["# 第3天：场景勘测＋单回合 smoke 验收", "", f"结论：{summary['status']}。仅验收A组场景，不代表ABC增益实验完成。", "", "|验收项|结果|实测值与判据|", "|---|---|---|"]
    for c in checks:
        text.append(f"|{c['name']}|{'通过' if c['passed'] else '未通过'}|{c['value']}；{c['criterion']}|")
    text += ["", "目标有效发现使用实际发出的门控消息，并在仿真结束后用目标位置核验（XY误差≤5 m）；不是仅检查几何视场。",
             "本轮保存2 Hz同步RGB和深度伪彩图，以及逐帧元数据、候选、消息、控制与轨迹；不保存完整原始深度数据。",
             "真值边界证据来自配置验证与控制器回归测试，不能把硬编码计数字段解释成全仿真动态插桩证据。",
             "本轮是单个红色静止目标的几何/发现时序资格场景，设置非红色车辆及行人；同色不同车型的辨别能力需后续专项验证，本轮不能证明。",
             f"失效保护日志与其他危险停车重叠 {stale_other_hazard_ticks} ticks；仅证明失效期间没有解除停车保护，不能声称是独立的失效制动因果实验。",
             "", "## 输出", ""]
    text.extend(f"- {v}" for v in summary["artifacts"].values())
    (run / "smoke_summary.md").write_text("\n".join(text) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
