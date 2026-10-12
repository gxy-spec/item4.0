#!/usr/bin/env python3
"""Evaluate one same-seed Day 4 A/B/C batch and render paired evidence."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import yaml

from analyze_day3_survey_smoke import csv_rows, distance_to_route, jsonl

DEFAULT_ROOT = Path("E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/day4_same_seed_abc/S1001")
COLORS = {"A": "#68737d", "B": "#2176a3", "C": "#c76931"}


def dump(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def path_length(rows: list[dict]) -> float:
    return sum(math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])) for a, b in zip(rows, rows[1:]))


def first_threshold(times: np.ndarray, distances: np.ndarray, threshold: float) -> float | None:
    reached = np.flatnonzero(distances <= threshold)
    return float(times[reached[0]]) if len(reached) else None


def actor_signature(run_dir: Path) -> dict:
    actors = json.loads((run_dir / "actors" / "initial_actor_states.json").read_text(encoding="utf-8"))
    return {
        item["role"]: {
            "type_id": item["type_id"],
            "xyz": [round(float(value), 2) for value in item["location_xyz"]],
        }
        for item in actors if item["role"] not in {"ugv", "uav"}
    }


def load_one(indexed: dict) -> dict:
    group = indexed["group"]
    run = Path(indexed["run_dir"])
    config = yaml.safe_load((run / "resolved_config.yaml").read_text(encoding="utf-8"))
    native = json.loads((run / "validation_report.json").read_text(encoding="utf-8"))
    trajectories = {role: csv_rows(run / "trajectories" / f"{role}_trajectory.csv") for role in ("ugv", "uav", "target")}
    safety = csv_rows(run / "safety" / "safety_state.csv")
    pairs = csv_rows(run / "synchronization" / "frame_index.csv")
    guard = csv_rows(run / "safety" / "ugv_obstacle_perception.csv")
    messages = jsonl(run / "communication" / "messages.jsonl")
    events = jsonl(run / "communication" / "communication_events.jsonl")
    routes = jsonl(run / "planning" / "target_route_events.jsonl")
    patrol = json.loads((run / "planning" / "ugv_patrol_route_executed.json").read_text(encoding="utf-8"))["points_xyz"]
    if not trajectories["ugv"] or not trajectories["target"] or not safety or not pairs:
        raise ValueError(f"{group}: missing motion/safety/synchronization evidence: {run}")
    t0 = float(trajectories["ugv"][0]["timestamp"])
    target = [float(trajectories["target"][-1][key]) for key in ("x", "y", "z")]
    times = np.asarray([float(row["timestamp"]) - t0 for row in trajectories["ugv"]])
    points = np.asarray([[float(row[key]) for key in ("x", "y")] for row in trajectories["ugv"]])
    distances = np.linalg.norm(points - np.asarray(target[:2]), axis=1)
    message_by_id = {row["message_id"]: row for row in messages}
    target_routes = [event for event in routes if event.get("event") == "target_route_planned_from_selected_message"]
    valid_routes = [event for event in target_routes if math.dist(event["target_xyz"][:2], target[:2]) <= 5.0]
    wrong_routes = [event for event in target_routes if event not in valid_routes]
    stale_route_commands = [
        event for event in target_routes if event["message_id"] not in message_by_id
        or not 0 <= float(event["timestamp_s"]) - float(message_by_id[event["message_id"]]["source_timestamp_s"]) < 1.0 + 1e-6
    ]
    synced_time = {
        role: {int(row["frame"]): float(row["timestamp"]) for row in jsonl(run / "sensors" / role / "metadata" / "frames.jsonl")}
        for role in ("uav", "ugv")
    }
    common = set(synced_time["uav"]) & set(synced_time["ugv"])
    max_source_delta = max((abs(synced_time["uav"][frame] - synced_time["ugv"][frame]) for frame in common), default=math.inf)
    collisions = {
        role: len(json.loads((run / "safety" / f"{role}_collision_events.json").read_text(encoding="utf-8")))
        for role in ("ugv", "uav")
    }
    first_correct_message = {
        role: min((float(msg["source_timestamp_s"]) - t0 for msg in messages
                   if msg["source"] == role and math.dist(msg["world_position_xyz"][:2], target[:2]) <= 5.0), default=None)
        for role in ("uav", "ugv")
    }
    metrics = {
        "group": group, "seed": config["random_seed"], "control_policy": config["dynamic"]["persistent_target_communication"]["control_policy"],
        "native_status": native["status"], "run_dir": str(run),
        "initial_target_distance_m": float(distances[0]), "minimum_target_distance_m": float(distances.min()),
        "final_target_distance_m": float(distances[-1]), "first_within_10m_s": first_threshold(times, distances, 10.0),
        "first_within_5m_s": first_threshold(times, distances, 5.0),
        "ugv_path_m": path_length(trajectories["ugv"]), "uav_path_m": path_length(trajectories["uav"]),
        "maximum_patrol_deviation_m": max(distance_to_route(point, patrol) for point in points),
        "first_correct_discovery_uav_s": first_correct_message["uav"],
        "first_correct_discovery_ugv_s": first_correct_message["ugv"],
        "first_correct_route_s": min((float(item["timestamp_s"]) - t0 for item in valid_routes), default=None),
        "correct_route_plans": len(valid_routes), "wrong_route_plans": len(wrong_routes),
        "uav_route_plans": sum(item["source"] == "uav" for item in valid_routes),
        "ugv_route_plans": sum(item["source"] == "ugv" for item in valid_routes),
        "stale_route_commands": len(stale_route_commands),
        "stale_control_ticks": sum(row["stale_target_follow_command"].lower() == "true" for row in safety),
        "message_route_ticks": sum(row["target_tracking_mode"] == "message_driven" for row in safety),
        "sensor_guard_coverage": len(guard) / len(safety), "common_frame_ratio": len(pairs) / len(safety),
        "maximum_frame_spread": max(int(row["frame_spread"]) for row in pairs),
        "maximum_source_time_delta_s": max_source_delta,
        "ugv_collisions": collisions["ugv"], "uav_collisions": collisions["uav"],
        "route_complete_ticks": sum(row["ugv_safety_mode"] == "route_complete" for row in safety),
        "pedestrian_wait_ticks": sum(row["ugv_safety_mode"] == "wait_pedestrian" for row in safety),
        "message_sent_uav": sum(row["source"] == "uav" for row in messages),
        "message_sent_ugv": sum(row["source"] == "ugv" for row in messages),
        "message_received_count": sum(row.get("event") == "received" for row in events),
    }
    return {"metrics": metrics, "config": config, "actors": actor_signature(run), "trajectories": trajectories,
            "times": times, "points": points, "distances": distances, "target": target, "safety": safety,
            "pairs": pairs, "patrol": patrol, "routes": target_routes, "messages": messages, "t0": t0}


def configure_plotting() -> None:
    font = Path("C:/Windows/Fonts/msyh.ttc")
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.unicode_minus": False, "savefig.dpi": 200})


def save_figure(fig, output: Path, stem: str) -> None:
    for suffix in ("png", "pdf"):
        fig.savefig(output / f"{stem}.{suffix}", bbox_inches="tight")
    plt.close(fig)


def render_figures(runs: dict[str, dict], output: Path) -> None:
    configure_plotting()
    a = runs["A"]
    scene = a["metrics"]["run_dir"]
    geometry_path = Path(scene) / "planning" / "scene_road_geometry.json"
    if not geometry_path.exists():
        geometry_path = Path("E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/scenario_smoke/S1001/CI_E7_DAY3_SURVEY_A_T10_ZA_S1001_20261008T061926Z/planning/scene_road_geometry.json")
    geometry = json.loads(geometry_path.read_text(encoding="utf-8"))["lanes"]

    fig, ax = plt.subplots(figsize=(7.8, 7.3), layout="constrained")
    for lane in geometry:
        poly = np.asarray(lane["polygon"])
        ax.fill(poly[:, 0], poly[:, 1], color="#e5e8eb", linewidth=0, zorder=0)
    route = np.asarray(a["patrol"])
    ax.plot(route[:, 0], route[:, 1], "--", color="#333333", linewidth=1.35, label="预设巡检路线", zorder=2)
    for group in "ABC":
        xy = runs[group]["points"]
        style = {"A": ("-", 3), "B": ("-", 4), "C": ((0, (6, 3)), 5)}[group]
        ax.plot(xy[:, 0], xy[:, 1], linestyle=style[0], linewidth=1.8 if group == "A" else 2.0,
                color=COLORS[group], label=f"{group}组UGV", zorder=style[1])
        ax.scatter(*xy[-1], s=40, color=COLORS[group], marker="s", edgecolors="white", zorder=5)
        event = next((r for r in runs[group]["routes"] if r.get("event") == "target_route_planned_from_selected_message"), None)
        if event:
            j = int(np.argmin(abs(runs[group]["times"] - (float(event["timestamp_s"]) - runs[group]["t0"]))))
            ax.scatter(*xy[j], s=65, color=COLORS[group], marker="D", edgecolors="black", linewidths=0.5, zorder=6)
    target = a["target"]
    ax.scatter(*a["points"][0], s=35, marker="o", color="#222222", label="共同起点", zorder=7)
    ax.scatter(*target[:2], s=170, marker="*", color="#b72e31", label="目标车（事后评价）", zorder=7)
    ax.add_patch(plt.Circle(target[:2], 10, fill=False, linestyle=":", color="#b72e31", linewidth=1.3, label="10 m接近范围"))
    ax.set(xlabel="CARLA世界X（m）", ylabel="CARLA世界Y（m）", title="同种子三组UGV实际轨迹", xlim=(-75, 25), ylim=(-77, 65))
    ax.set_aspect("equal")
    ax.grid(alpha=0.17)
    ax.legend(frameon=True, fontsize=8, loc="upper right")
    save_figure(fig, output, "day4_trajectory_comparison")

    fig, ax = plt.subplots(figsize=(8.5, 4.7), layout="constrained")
    for group in "ABC":
        run = runs[group]
        ax.plot(run["times"], run["distances"], color=COLORS[group], linewidth=1.8, label=f"{group}组")
        hit = run["metrics"]["first_within_10m_s"]
        if hit is not None:
            ax.scatter(hit, 10, color=COLORS[group], s=32, zorder=5)
    ax.axhline(10, color="#b72e31", linestyle="--", linewidth=1, label="10 m接近门限")
    ax.set(xlabel="仿真经过时间（s）", ylabel="UGV到目标车的水平距离（m）", title="同种子三组目标接近过程", xlim=(0, 80), ylim=(0, None))
    ax.grid(alpha=0.22)
    ax.legend(frameon=False, ncol=4)
    save_figure(fig, output, "day4_target_distance_time")

    fig, axes = plt.subplots(3, 1, figsize=(8.8, 5.7), sharex=True, layout="constrained")
    for ax, group in zip(axes, "ABC"):
        run = runs[group]
        for source, y, color in (("uav", 1, "#8b5b9f"), ("ugv", 0, "#2476a5")):
            times = [float(row["source_timestamp_s"]) - run["t0"] for row in run["messages"] if row["source"] == source]
            ax.scatter(times, [y] * len(times), marker="|", s=48, linewidths=0.8, color=color, alpha=0.65)
        for event in run["routes"]:
            ax.axvline(float(event["timestamp_s"]) - run["t0"], color=COLORS[group], alpha=0.5, linewidth=0.9)
        ax.set(yticks=[0, 1], yticklabels=["UGV", "UAV"], ylabel=f"{group}组", ylim=(-0.45, 1.45))
        ax.grid(axis="x", alpha=0.2)
    axes[0].set_title("候选消息发送与目标路线生成时刻（短线：消息；竖线：路线）")
    axes[-1].set(xlabel="仿真经过时间（s）", xlim=(0, 80))
    save_figure(fig, output, "day4_message_decision_timeline")


def render_replay(run: dict, output: Path) -> Path:
    group = run["metrics"]["group"]
    root = Path(run["metrics"]["run_dir"])
    frame_rows = [row for row in run["pairs"] if int(row["sensor_files_saved"]) == 1]
    trajectory_by_frame = {
        role: {int(row["frame"]): row for row in run["trajectories"][role]}
        for role in ("uav", "ugv")
    }
    safety_by_frame = {int(row["frame"]): row for row in run["safety"]}
    x_min, x_max, y_min, y_max = -116.0, 35.0, -77.0, 79.0
    def pixel(x, y):
        return (int(40 + (x - x_min) / (x_max - x_min) * 800),
                int(1000 - (y - y_min) / (y_max - y_min) * 850))
    bg = np.full((1080, 900, 3), 248, np.uint8)
    geometry_path = root / "planning" / "scene_road_geometry.json"
    if not geometry_path.exists():
        geometry_path = Path("E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/scenario_smoke/S1001/CI_E7_DAY3_SURVEY_A_T10_ZA_S1001_20261008T061926Z/planning/scene_road_geometry.json")
    for lane in json.loads(geometry_path.read_text(encoding="utf-8"))["lanes"]:
        polygon = np.asarray([pixel(point[0], point[1]) for point in lane["polygon"]], np.int32)
        cv2.fillPoly(bg, [polygon], (229, 232, 235))
    route = np.asarray([pixel(point[0], point[1]) for point in run["patrol"]], np.int32)
    cv2.polylines(bg, [route], False, (65, 65, 65), 2)
    cv2.drawMarker(bg, pixel(*run["target"][:2]), (45, 45, 190), cv2.MARKER_STAR, 25, 2)
    fps = 1 / np.median(np.diff([float(row["timestamp"]) for row in frame_rows]))
    result = output / f"day4_{group}_five_view.mp4"
    writer = cv2.VideoWriter(str(result), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (1920, 1080))
    if not writer.isOpened():
        raise RuntimeError(f"Cannot open video writer for {result}")
    history = {"uav": [], "ugv": []}
    for row in frame_rows:
        frame = int(row["frame"])
        canvas = np.full((1080, 1920, 3), 255, np.uint8)
        canvas[:, :900] = bg
        for role, color in (("uav", (150, 75, 130)), ("ugv", (170, 110, 35))):
            state = trajectory_by_frame[role].get(frame)
            if state is None:
                raise RuntimeError(f"Missing {role} trajectory at frame {frame}")
            point = pixel(float(state["x"]), float(state["y"]))
            history[role].append(point)
            if len(history[role]) > 1:
                cv2.polylines(canvas, [np.asarray(history[role], np.int32)], False, color, 2)
            cv2.circle(canvas, point, 7, color, -1)
        for role, top in (("uav", 140), ("ugv", 550)):
            for modal, left in (("rgb", 930), ("depth_colour", 1430)):
                path = root / "sensors" / role / modal / f"{frame:08d}.png"
                image = cv2.imread(str(path))
                if image is None:
                    raise FileNotFoundError(path)
                canvas[top:top + 345, left:left + 460] = cv2.resize(image, (460, 345), interpolation=cv2.INTER_AREA)
                cv2.putText(canvas, f"{role.upper()} {modal.replace('_', ' ')}", (left, top - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40), 1, cv2.LINE_AA)
        safety = safety_by_frame.get(frame, {})
        t = float(row["timestamp"]) - run["t0"]
        cv2.putText(canvas, f"DAY 4 | {group}: {run['metrics']['control_policy']}", (930, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(canvas, f"t={t:.1f}s | synchronized source frame {frame}", (930, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"UGV speed {float(safety.get('ugv_speed_mps', 0)):.2f} m/s | {safety.get('ugv_safety_mode', 'unknown')}", (930, 960), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"Decision: {safety.get('target_tracking_mode', 'unknown')} | RGB-D safety active", (930, 1000), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40), 1, cv2.LINE_AA)
        cv2.putText(canvas, "UGV (blue), UAV (purple), target truth (red; offline only)", (35, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.57, (40, 40, 40), 1, cv2.LINE_AA)
        writer.write(canvas)
    writer.release()
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    root = args.root.resolve()
    index = json.loads((root / "day4_batch_index.json").read_text(encoding="utf-8"))
    if [item["group"] for item in index] != list("ABC"):
        raise ValueError("Day 4 index must contain exactly A, B, C in order")
    runs = {item["group"]: load_one(item) for item in index}
    metrics = {group: runs[group]["metrics"] for group in "ABC"}
    checks = []
    def check(name, passed, evidence, criterion):
        checks.append({"name": name, "passed": bool(passed), "evidence": evidence, "criterion": criterion})
    reference = runs["A"]["actors"]
    check("same_seed_initial_actors", all(runs[g]["actors"] == reference for g in "BC"),
          {g: runs[g]["actors"] for g in "ABC"}, "same actor type and initial XYZ rounded to 0.01 m")
    for group in "ABC":
        m = metrics[group]
        check(f"{group}_native_acceptance", m["native_status"] == "PASS", m["native_status"], "PASS")
        check(f"{group}_safe_and_synchronized",
              m["ugv_collisions"] == m["uav_collisions"] == 0 and m["sensor_guard_coverage"] >= 0.95
              and m["common_frame_ratio"] >= 0.98 and m["maximum_frame_spread"] == 0
              and m["maximum_source_time_delta_s"] <= 1e-6 and m["stale_route_commands"] == 0
              and m["stale_control_ticks"] == 0,
              {key: m[key] for key in ("ugv_collisions", "uav_collisions", "sensor_guard_coverage", "common_frame_ratio",
                                        "maximum_frame_spread", "maximum_source_time_delta_s", "stale_route_commands", "stale_control_ticks")},
              "zero collisions/stale target commands, >=95% guard, >=98% exact synchronized frames")
    check("A_fixed_patrol", metrics["A"]["correct_route_plans"] == 0 and metrics["A"]["wrong_route_plans"] == 0
          and metrics["A"]["maximum_patrol_deviation_m"] <= 2 and metrics["A"]["minimum_target_distance_m"] > 10,
          {key: metrics["A"][key] for key in ("correct_route_plans", "wrong_route_plans", "maximum_patrol_deviation_m", "minimum_target_distance_m")},
          "zero message-driven routes, <=2 m patrol deviation, never within 10 m of target")
    check("A_route_end_explicit", metrics["A"]["route_complete_ticks"] > 0,
          metrics["A"]["route_complete_ticks"], ">0 route_complete ticks")
    for group, source in (("B", "ugv"), ("C", "uav")):
        m = metrics[group]
        check(f"{group}_correct_source_changes_route", m[f"{source}_route_plans"] > 0 and m["message_route_ticks"] >= 10
              and m["wrong_route_plans"] == 0,
              {key: m[key] for key in (f"{source}_route_plans", "message_route_ticks", "wrong_route_plans")},
              f">=1 correct {source} route, >=10 route-control ticks, zero wrong routes")
        check(f"{group}_target_approach", m["first_within_10m_s"] is not None,
              m["first_within_10m_s"], "reach target 10 m radius during shared observation window")
    b, c = metrics["B"], metrics["C"]
    check("C_earlier_valid_route", b["first_correct_route_s"] is not None and c["first_correct_route_s"] is not None
          and c["first_correct_route_s"] + 5.0 <= b["first_correct_route_s"],
          {"B_s": b["first_correct_route_s"], "C_s": c["first_correct_route_s"]}, "C plans correct target >=5 s before B")
    check("C_earlier_target_approach", c["first_within_10m_s"] is not None and
          (b["first_within_10m_s"] is None or c["first_within_10m_s"] < b["first_within_10m_s"]),
          {"B_s": b["first_within_10m_s"], "C_s": c["first_within_10m_s"]}, "C reaches 10 m before B")
    for group in "ABC":
        dump(Path(metrics[group]["run_dir"]) / "day4_episode_metrics.json", metrics[group])
    with (root / "day4_comparison_metrics.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metrics["A"]))
        writer.writeheader()
        writer.writerows([metrics[group] for group in "ABC"])
    output = root / "preview"
    output.mkdir(exist_ok=True)
    surveyed_geometry = Path("E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/scenario_smoke/S1001/CI_E7_DAY3_SURVEY_A_T10_ZA_S1001_20261008T061926Z/planning/scene_road_geometry.json")
    for group in "ABC":
        local_geometry = Path(metrics[group]["run_dir"]) / "planning" / "scene_road_geometry.json"
        if not local_geometry.exists():
            shutil.copy2(surveyed_geometry, local_geometry)
    render_figures(runs, output)
    videos = {group: str(render_replay(runs[group], output)) for group in "ABC"}
    summary = {"stage": "Day 4 same-seed A/B/C", "seed": 1001,
               "status": "PASS" if all(item["passed"] for item in checks) else "FAIL",
               "scope": "one paired seed; route influence under ideal 50 ms / 1 s TTL semantic communication, not physical-channel or population claim",
               "checks": checks, "metrics": metrics, "videos": videos,
               "figures": [str(path) for path in output.glob("day4_*.png")],
               "data_retention": "sparse RGB/depth-colour replay frames; no full metric-depth dataset"}
    dump(root / "day4_comparison_acceptance.json", summary)
    print(json.dumps({"status": summary["status"], "checks_passed": sum(item["passed"] for item in checks),
                      "checks_total": len(checks), "output": str(root / "day4_comparison_acceptance.json")}, ensure_ascii=False))
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
