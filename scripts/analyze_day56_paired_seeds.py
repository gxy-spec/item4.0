#!/usr/bin/env python3
"""Evaluate three paired seeds and render evidence with perception boxes."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

from analyze_day3_survey_smoke import jsonl
from analyze_day4_same_seed_abc import COLORS, first_threshold, load_one

ROOT = Path("E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/day56_paired_seeds")
GEOMETRY = Path("E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/scenario_smoke/S1001/CI_E7_DAY3_SURVEY_A_T10_ZA_S1001_20261008T061926Z/planning/scene_road_geometry.json")


def dump(path: Path, value) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def check(checks: list[dict], name: str, passed: bool, value, criterion: str) -> None:
    checks.append({"name": name, "passed": bool(passed), "value": value, "criterion": criterion})


def draw_boxes(image: np.ndarray, role: str, frame: int, candidates: dict, sent: dict, selected: dict,
               target_xy: list[float]) -> np.ndarray:
    result = cv2.resize(image, (460, 345), interpolation=cv2.INTER_AREA)
    sx, sy = 460 / image.shape[1], 345 / image.shape[0]
    messages = {(m["source"], int(m["source_frame"]), m["candidate_id"]) for m in sent.get((role, frame), [])}
    choices = {(m["source"], int(m["source_frame"]), m["candidate_id"]) for m in selected.get((role, frame), [])}
    for candidate in candidates.get((role, frame), []):
        box = candidate.get("bbox_xyxy")
        if not box:
            continue
        key = (role, frame, candidate["candidate_id"])
        xy = candidate.get("world_position_xyz")
        is_correct = xy is not None and math.dist(xy[:2], target_xy) <= 5
        if key in choices:
            color, label = ((20, 170, 20), "SEL OK") if is_correct else ((35, 35, 220), "SEL ERR")
        elif key in messages:
            color, label = ((0, 180, 255), "SENT") if is_correct else ((35, 70, 235), "SENT ERR")
        elif candidate.get("color") == "red":
            color, label = (0, 140, 225), "RED"
        else:
            color, label = (155, 155, 155), "OTHER"
        x1, y1, x2, y2 = [int(round(v * scale)) for v, scale in zip(box, (sx, sy, sx, sy))]
        cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)
        cv2.putText(result, f"{candidate['candidate_id']} {label}", (max(0, x1), max(14, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
    return result


def replay(seed: int, run: dict, output: Path, geometry: list[dict]) -> Path:
    group = run["metrics"]["group"]
    root = Path(run["metrics"]["run_dir"])
    rows = [r for r in run["pairs"] if int(r["sensor_files_saved"]) == 1]
    by_frame = {role: {int(r["frame"]): r for r in run["trajectories"][role]} for role in ("uav", "ugv")}
    safety = {int(r["frame"]): r for r in run["safety"]}
    candidates = {(role, int(row["frame_id"])): row["candidates"]
                  for role in ("uav", "ugv")
                  for row in jsonl(root / "perception" / f"{role}_target_candidates_synchronized.jsonl")}
    sent: dict[tuple[str, int], list[dict]] = {}
    chosen: dict[tuple[str, int], list[dict]] = {}
    for store, records in ((sent, run["messages"]), (chosen, jsonl(root / "communication" / "selected_target_states.jsonl"))):
        for message in records:
            store.setdefault((message["source"], int(message["source_frame"])), []).append(message)
    def pixel(x, y):
        return (int(42 + (x + 116) / 151 * 790), int(1000 - (y + 77) / 156 * 850))
    bg = np.full((1080, 900, 3), 248, np.uint8)
    for lane in geometry:
        poly = np.asarray([pixel(p[0], p[1]) for p in lane["polygon"]], np.int32)
        cv2.fillPoly(bg, [poly], (229, 232, 235))
    cv2.polylines(bg, [np.asarray([pixel(*p[:2]) for p in run["patrol"]], np.int32)], False, (50, 50, 50), 2)
    cv2.drawMarker(bg, pixel(*run["target"][:2]), (50, 50, 190), cv2.MARKER_STAR, 25, 2)
    fps = 1 / float(np.median(np.diff([float(r["timestamp"]) for r in rows])))
    path = output / f"S{seed}_{group}_five_view_boxed.mp4"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (1920, 1080))
    if not writer.isOpened():
        raise RuntimeError(f"Video writer failed: {path}")
    history: dict[str, list[tuple[int, int]]] = {"uav": [], "ugv": []}
    try:
        for row in rows:
            frame = int(row["frame"])
            canvas = np.full((1080, 1920, 3), 255, np.uint8)
            canvas[:, :900] = bg
            for role, color in (("uav", (150, 75, 130)), ("ugv", (170, 110, 35))):
                state = by_frame[role][frame]
                point = pixel(float(state["x"]), float(state["y"]))
                history[role].append(point)
                if len(history[role]) > 1:
                    cv2.polylines(canvas, [np.asarray(history[role], np.int32)], False, color, 2)
                cv2.circle(canvas, point, 7, color, -1)
            for role, top in (("uav", 140), ("ugv", 550)):
                for modal, left in (("rgb", 930), ("depth_colour", 1430)):
                    source = root / "sensors" / role / modal / f"{frame:08d}.png"
                    image = cv2.imread(str(source))
                    if image is None:
                        raise FileNotFoundError(source)
                    if modal == "rgb":
                        image = draw_boxes(image, role, frame, candidates, sent, chosen, run["target"][:2])
                    else:
                        image = cv2.resize(image, (460, 345), interpolation=cv2.INTER_AREA)
                    canvas[top:top + 345, left:left + 460] = image
                    cv2.putText(canvas, f"{role.upper()} {'RGB / proposals' if modal == 'rgb' else 'depth'}",
                                (left, top - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40), 1, cv2.LINE_AA)
            s = safety.get(frame, {})
            t = float(row["timestamp"]) - run["t0"]
            cv2.putText(canvas, f"DAY 5-6 | SEED {seed} | GROUP {group}", (930, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.77, (20, 20, 20), 2, cv2.LINE_AA)
            cv2.putText(canvas, f"t={t:.1f}s | synchronized frame {frame}", (930, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (40, 40, 40), 1, cv2.LINE_AA)
            cv2.putText(canvas, "Boxes: grey other; orange red; yellow sent; green selected GT-match; red selected GT-error", (910, 920), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (35, 35, 35), 1, cv2.LINE_AA)
            cv2.putText(canvas, "GT match/error is annotated OFFLINE ONLY; it is never a controller input", (910, 947), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (35, 35, 35), 1, cv2.LINE_AA)
            cv2.putText(canvas, f"UGV {float(s.get('ugv_speed_mps', 0)):.2f} m/s | RGB-D safety {s.get('ugv_safety_mode', 'unknown')}", (910, 982), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (35, 35, 35), 1, cv2.LINE_AA)
            cv2.putText(canvas, f"Decision {s.get('target_tracking_mode', 'unknown')}", (910, 1010), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (35, 35, 35), 1, cv2.LINE_AA)
            cv2.putText(canvas, "Road surface grey | patrol black | target truth star (offline only)", (30, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.51, (35, 35, 35), 1, cv2.LINE_AA)
            writer.write(canvas)
    finally:
        writer.release()
    return path


def plot_seed(seed: int, runs: dict, output: Path, geometry: list[dict]) -> list[str]:
    font = Path("C:/Windows/Fonts/msyh.ttc")
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "axes.unicode_minus": False})
    fig, axes = plt.subplots(1, 2, figsize=(12.2, 5.6), layout="constrained")
    ax = axes[0]
    for lane in geometry:
        polygon = np.asarray(lane["polygon"])
        ax.fill(polygon[:, 0], polygon[:, 1], color="#e7e9eb", lw=0, zorder=0)
    patrol = np.asarray(runs["A"]["patrol"])
    ax.plot(patrol[:, 0], patrol[:, 1], "--", color="#282c30", lw=1.4, label="预设巡检路线")
    for group in "ABC":
        run = runs[group]
        ax.plot(run["points"][:, 0], run["points"][:, 1], lw=1.7, color=COLORS[group], label=f"{group}组UGV")
        ax.scatter(*run["points"][-1], color=COLORS[group], marker="s", s=35)
    target = runs["A"]["target"]
    ax.scatter(*target[:2], color="#b43136", marker="*", s=150, label="目标车（事后真值）")
    ax.add_patch(plt.Circle(target[:2], 10, fill=False, color="#b43136", ls=":", lw=1.2))
    ax.set(xlabel="CARLA 世界 X（m）", ylabel="CARLA 世界 Y（m）", title=f"种子 {seed}：实际轨迹与目标")
    ax.set_xlim(-116, 35); ax.set_ylim(-77, 79); ax.set_aspect("equal"); ax.grid(alpha=0.15)
    ax.legend(fontsize=8, loc="upper right")
    ax = axes[1]
    for group in "ABC":
        run = runs[group]
        ax.plot(run["times"], run["distances"], color=COLORS[group], lw=1.6, label=f"{group}组")
        hit = run["metrics"]["first_within_10m_s"]
        if hit is not None:
            ax.scatter(hit, 10, color=COLORS[group], s=28, zorder=5)
    ax.axhline(10, color="#b43136", ls="--", lw=1, label="10 m接近门限")
    ax.axhline(15, color="#7a858f", ls=":", lw=1, label="15 m安全接近（补充）")
    ax.set(xlabel="仿真时间（s）", ylabel="UGV 到目标车的水平距离（m）", title="目标接近过程", xlim=(0, 120))
    ax.grid(alpha=0.2); ax.legend(frameon=False)
    output_path = output / f"S{seed}_route_and_distance"
    for suffix in ("png", "pdf"):
        fig.savefig(output_path.with_suffix(f".{suffix}"), dpi=220, bbox_inches="tight")
    plt.close(fig)
    return [str(output_path.with_suffix(f".{suffix}")) for suffix in ("png", "pdf")]


def analyze_seed(seed: int, render_videos: bool) -> dict:
    root = ROOT / f"S{seed}"
    index = json.loads((root / "paired_index.json").read_text(encoding="utf-8"))
    if [entry["group"] for entry in index] != list("ABC"):
        raise RuntimeError(f"S{seed}: incomplete paired index")
    runs = {entry["group"]: load_one(entry) for entry in index}
    metrics = {group: runs[group]["metrics"] for group in "ABC"}
    for group in "ABC":
        metrics[group]["first_within_15m_s"] = first_threshold(runs[group]["times"], runs[group]["distances"], 15)
    checks: list[dict] = []
    reference = runs["A"]["actors"]
    check(checks, "same_seed_same_initial_actors", all(runs[g]["actors"] == reference for g in "BC"),
          {g: runs[g]["actors"] for g in "ABC"}, "same actor type and initial XYZ, rounded to 0.01 m")
    for group in "ABC":
        m = metrics[group]
        check(checks, f"{group}_native_and_safe", m["native_status"] == "PASS" and m["ugv_collisions"] == m["uav_collisions"] == 0
              and m["sensor_guard_coverage"] >= 0.95 and m["common_frame_ratio"] >= 0.98
              and m["maximum_frame_spread"] == 0 and m["maximum_source_time_delta_s"] <= 1e-6,
              {k: m[k] for k in ("native_status", "ugv_collisions", "uav_collisions", "sensor_guard_coverage",
                                 "common_frame_ratio", "maximum_frame_spread", "maximum_source_time_delta_s")},
              "native PASS; zero collision; RGB-D guard >=95%; exact source frame/time")
        check(checks, f"{group}_no_wrong_or_stale_route", m["wrong_route_plans"] == m["stale_route_commands"] == 0,
              {k: m[k] for k in ("wrong_route_plans", "stale_route_commands")}, "zero false/stale route commands")
    a, b, c = (metrics[g] for g in "ABC")
    check(checks, "A_fixed_patrol_misses_target", a["correct_route_plans"] == 0
          and a["maximum_patrol_deviation_m"] <= 2 and a["minimum_target_distance_m"] > 10,
          {k: a[k] for k in ("correct_route_plans", "maximum_patrol_deviation_m", "minimum_target_distance_m")},
          "no target replanning; <=2 m route deviation; >10 m to target")
    check(checks, "C_uses_uav_and_approaches", c["uav_route_plans"] > 0 and c["first_within_10m_s"] is not None,
          {k: c[k] for k in ("uav_route_plans", "first_within_10m_s", "minimum_target_distance_m")},
          "UAV message causes correct route and UGV enters 10 m zone")
    if seed == 2001:
        check(checks, "B_local_only_misses", b["correct_route_plans"] == 0 and b["minimum_target_distance_m"] > 10,
              {k: b[k] for k in ("correct_route_plans", "minimum_target_distance_m")},
              "UGV-only has no correct target route and remains outside 10 m")
    else:
        check(checks, "B_uses_local_detection", b["ugv_route_plans"] > 0 and b["first_within_10m_s"] is not None,
              {k: b[k] for k in ("ugv_route_plans", "first_within_10m_s")},
              "UGV-only correct target route and reaches 10 m")
        check(checks, "C_route_earlier_than_B", b["first_correct_route_s"] is not None
              and c["first_correct_route_s"] is not None
              and c["first_correct_route_s"] + 5 <= b["first_correct_route_s"],
              {"B_s": b["first_correct_route_s"], "C_s": c["first_correct_route_s"]},
              "C correct route >=5 s earlier than B")
        check(checks, "C_arrival_earlier_than_B", c["first_within_10m_s"] is not None
              and (b["first_within_10m_s"] is None or c["first_within_10m_s"] < b["first_within_10m_s"]),
              {"B_s": b["first_within_10m_s"], "C_s": c["first_within_10m_s"]},
              "C reaches 10 m earlier than B")
        if seed == 3001:
            check(checks, "supplementary_15m_safe_approach_gain",
                  b["first_within_15m_s"] is not None and c["first_within_15m_s"] is not None
                  and c["first_within_15m_s"] + 2 <= b["first_within_15m_s"],
                  {"B_s": b["first_within_15m_s"], "C_s": c["first_within_15m_s"]},
                  "supplementary safety-aware analysis: C enters 15 m >=2 s earlier; does not replace strict 10 m test")
    output = root / "preview"
    output.mkdir(exist_ok=True)
    geometry = json.loads(GEOMETRY.read_text(encoding="utf-8"))["lanes"]
    figures = plot_seed(seed, runs, output, geometry)
    videos = {g: str(replay(seed, runs[g], output, geometry)) for g in "ABC"} if render_videos else {}
    summary = {"stage": "Day 5–6 paired A/B/C, same scene within seed, 120 s horizon", "seed": seed,
               "status": "PASS" if all(item["passed"] for item in checks) else "FAIL",
               "checks": checks, "metrics": metrics, "figures": figures, "videos": videos,
               "scope": "Ideal 50 ms semantic transport; no physical channel. GT used for offline evaluation/box annotations only."}
    dump(root / "paired_acceptance.json", summary)
    with (root / "paired_metrics.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(a))
        writer.writeheader(); writer.writerows(metrics[g] for g in "ABC")
    print(json.dumps({"seed": seed, "status": summary["status"], "checks_passed": sum(i["passed"] for i in checks),
                      "checks_total": len(checks), "report": str(root / "paired_acceptance.json")}, ensure_ascii=False), flush=True)
    for item in checks:
        if not item["passed"]:
            print(f"FAIL {item['name']}: {item['value']}", flush=True)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, choices=(1001, 2001, 3001), required=True)
    parser.add_argument("--skip-videos", action="store_true")
    args = parser.parse_args()
    result = analyze_seed(args.seed, not args.skip_videos)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
