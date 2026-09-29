#!/usr/bin/env python3
"""Summarize E7 paired route-control and outage-recovery experiments."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def route_distance(point: tuple[float, float], route: list[list[float]]) -> float:
    best = math.inf
    for a, b in zip(route, route[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        denom = dx * dx + dy * dy
        t = 0.0 if denom == 0 else min(1.0, max(0.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / denom))
        px, py = a[0] + t * dx, a[1] + t * dy
        best = min(best, math.hypot(point[0] - px, point[1] - py))
    return best


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, help="Output root (defaults to the selected run index's parent)")
    parser.add_argument("--index", type=Path, help="Explicit run-index JSON, e.g. smoke_run_index.json")
    args = parser.parse_args()
    runs = []
    default_root = Path(r"E:\CarlaAirData\CityInspection_GOC\e7_revised")
    index_path = args.index.resolve() if args.index else (args.root or default_root).resolve() / "batch_run_index.json"
    root = (args.root.resolve() if args.root else (index_path.parent if args.index else default_root.resolve()))
    if not args.index and not index_path.exists() and (root / "smoke_run_index.json").exists():
        index_path = root / "smoke_run_index.json"
    if index_path.exists():
        indexed = read_json(index_path)
        report_paths = [Path(item["run_dir"]) / "validation_report.json" for item in indexed]
        missing = [str(path) for path in report_paths if not path.exists()]
        if missing:
            raise RuntimeError(f"Batch index references missing validation reports: {missing}")
    else:
        report_paths = list(root.rglob("validation_report.json"))
    for report_path in report_paths:
        run_dir = report_path.parent
        report = read_json(report_path)
        config = read_json(run_dir / "resolved_config.json") if (run_dir / "resolved_config.json").exists() else None
        if config is None:
            import yaml
            config = yaml.safe_load((run_dir / "resolved_config.yaml").read_text(encoding="utf-8"))
        trajectories = read_csv(run_dir / "trajectories" / "ugv_trajectory.csv")
        targets = read_csv(run_dir / "trajectories" / "target_trajectory.csv")
        safety = read_csv(run_dir / "safety" / "safety_state.csv")
        route_events_path = run_dir / "planning" / "target_route_events.jsonl"
        route_events = [json.loads(line) for line in route_events_path.read_text(encoding="utf-8").splitlines() if line.strip()] if route_events_path.exists() else []
        if not trajectories or not targets:
            continue
        target_by_timestamp = {float(row["timestamp"]): (float(row["x"]), float(row["y"])) for row in targets}
        target_xy = next(iter(target_by_timestamp.values()))
        path_xy = [(float(row["x"]), float(row["y"])) for row in trajectories]
        dist = [math.hypot(p[0] - target_xy[0], p[1] - target_xy[1]) for p in path_xy]
        executed_route_path = run_dir / "planning" / "ugv_patrol_route_executed.json"
        if executed_route_path.exists():
            route_record = read_json(executed_route_path)
            route = route_record["points_xyz"]
        else:
            # Compatibility for older runs, whose runtime road-graph route was
            # not yet recorded. New comparison batches must always have the
            # exact executed route artifact.
            route = config["region"]["planned_ugv_route"]
            limit = config.get("dynamic", {}).get("ugv_patrol_route_limit_m")
            if limit is not None:
                trimmed = [route[0]]
                length = 0.0
                for a, b in zip(route, route[1:]):
                    segment = math.hypot(b[0] - a[0], b[1] - a[1])
                    if length + segment >= float(limit):
                        frac = (float(limit) - length) / max(segment, 1e-9)
                        trimmed.append([a[i] + frac * (b[i] - a[i]) for i in range(3)])
                        break
                    trimmed.append(b)
                    length += segment
                route = trimmed
        nearest_baseline = [route_distance(point, route) for point in path_xy]
        source_counts = {source: sum(row.get("source") == source for row in read_jsonl(run_dir / "communication" / "messages.jsonl")) for source in ("uav", "ugv")}
        modes = [row.get("target_tracking_mode", "") for row in safety]
        target_route_ticks = sum(mode == "message_driven" for mode in modes)
        stale_grace_ticks = sum(mode == "stale_grace" for mode in modes)
        safe_hold_ticks = sum(mode == "safe_hold" for mode in modes)
        first_close_timestamp = next(
            (float(row["timestamp"]) for row, value in zip(trajectories, dist) if value <= 10.0),
            None,
        )
        first_timestamp = float(trajectories[0]["timestamp"])
        first_close = (
            None if first_close_timestamp is None else first_close_timestamp - first_timestamp
        )
        summary = report.get("summary", {})
        obstacle_summary = summary.get("ugv_rgbd_obstacle_guard", {})
        communication_summary = summary.get("persistent_target_communication", {})
        runs.append({
            "run_dir": str(run_dir),
            "experiment_id": report.get("experiment_id", ""),
            "status": report.get("status", "UNKNOWN"),
            "seed": int(config.get("random_seed", -1)),
            "control_policy": config.get("dynamic", {}).get("persistent_target_communication", {}).get("control_policy", "dual"),
            "comparison_case": config.get("dynamic", {}).get("persistent_target_communication", {}).get("comparison_case", "legacy"),
            "duration_s": float(report.get("summary", {}).get("duration_seconds", 0.0)),
            "four_stream_common_frames": int(summary.get("common_frames", 0)),
            "four_stream_common_frame_ratio": float(summary.get("common_frame_ratio", 0.0)),
            "ugv_rgbd_pair_ratio": float(obstacle_summary.get("ugv_rgbd_pair_ratio", 0.0)),
            "ugv_path_m": float(report.get("summary", {}).get("ugv_path_m", 0.0)),
            "initial_target_distance_m": dist[0],
            "final_target_distance_m": dist[-1],
            "minimum_target_distance_m": min(dist),
            "target_distance_reduction_m": dist[0] - dist[-1],
            "time_to_10m_s": first_close,
            "mean_departure_from_patrol_route_m": float(np.mean(nearest_baseline)),
            "max_departure_from_patrol_route_m": max(nearest_baseline),
            "ugv_collisions": int(report.get("summary", {}).get("ugv_collision_count", 0)),
            "ugv_safety_stop_ticks": int(obstacle_summary.get("sensor_stop_ticks", 0)),
            "ugv_safety_caution_ticks": int(obstacle_summary.get("sensor_caution_ticks", 0)),
            "message_route_plan_count": sum(event.get("event") == "target_route_planned_from_selected_message" for event in route_events),
            "uav_sourced_route_plan_count": sum(
                event.get("event") == "target_route_planned_from_selected_message"
                and event.get("source") == "uav" for event in route_events
            ),
            "ugv_sourced_route_plan_count": sum(
                event.get("event") == "target_route_planned_from_selected_message"
                and event.get("source") == "ugv" for event in route_events
            ),
            "message_route_ticks": target_route_ticks,
            "stale_grace_ticks": stale_grace_ticks,
            "safe_hold_ticks": safe_hold_ticks,
            "uav_sent": source_counts["uav"],
            "ugv_sent": source_counts["ugv"],
            "_config": config,
            "_trajectory": trajectories,
            "_target": target_xy,
            "_route": route,
            "_safety": safety,
            "_events": route_events,
        })

    if not runs:
        raise RuntimeError(f"No completed runs found beneath {root}")
    root.mkdir(parents=True, exist_ok=True)
    fields = [key for key in runs[0] if not key.startswith("_")]
    with (root / "comparison_metrics.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows([{key: row[key] for key in fields} for row in runs])
    serializable = [{key: row[key] for key in fields} for row in runs]
    (root / "comparison_metrics.json").write_text(json.dumps(serializable, indent=2), encoding="utf-8")

    # The local positive-control deliberately places the target on the patrol
    # route. Keep it out of paired A/B/C means and plots, which use the shared
    # off-route target scene. It remains present in the full per-run metrics.
    comparison_runs = [
        row for row in runs
        if "ROUTE_COMPARE" in row["experiment_id"] and row["comparison_case"] == "off_route"
    ]
    positive_control_runs = [row for row in runs if row["comparison_case"] == "target_visible"]
    grouped: list[dict[str, Any]] = []
    for policy in ("none", "ugv_local", "dual"):
        group = [row for row in comparison_runs if row["control_policy"] == policy]
        if not group:
            continue
        grouped.append({
            "control_policy": policy,
            "run_count": len(group),
            "pass_count": sum(row["status"] == "PASS" for row in group),
            "mean_final_target_distance_m": float(np.mean([row["final_target_distance_m"] for row in group])),
            "std_final_target_distance_m": float(np.std([row["final_target_distance_m"] for row in group], ddof=1)) if len(group) > 1 else 0.0,
            "mean_target_distance_reduction_m": float(np.mean([row["target_distance_reduction_m"] for row in group])),
            "mean_ugv_path_m": float(np.mean([row["ugv_path_m"] for row in group])),
            "mean_route_departure_m": float(np.mean([row["mean_departure_from_patrol_route_m"] for row in group])),
            "collisions": sum(row["ugv_collisions"] for row in group),
            "mean_uav_messages": float(np.mean([row["uav_sent"] for row in group])),
            "mean_ugv_messages": float(np.mean([row["ugv_sent"] for row in group])),
        })
    with (root / "comparison_group_summary.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(grouped[0]) if grouped else [])
        writer.writeheader()
        writer.writerows(grouped)
    positive_fields = [
        "experiment_id", "seed", "status", "message_route_plan_count",
        "ugv_sourced_route_plan_count", "initial_target_distance_m",
        "final_target_distance_m", "target_distance_reduction_m",
        "mean_departure_from_patrol_route_m", "ugv_collisions",
    ]
    with (root / "ugv_local_positive_control.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=positive_fields)
        writer.writeheader()
        writer.writerows([{key: row[key] for key in positive_fields} for row in positive_control_runs])

    formal = comparison_runs
    by_seed = {seed: [row for row in formal if row["seed"] == seed] for seed in sorted({row["seed"] for row in formal})}
    outage = [row for row in runs if "outage_" in Path(row["run_dir"]).name.lower() or "OUTAGE" in row["experiment_id"]]
    colors = {"none": "#777777", "ugv_local": "#2878B5", "dual": "#D55E00"}
    figure, axes = plt.subplots(2, 2, figsize=(14, 10), constrained_layout=True)
    path_ax, distance_ax, stale_ax, summary_ax = axes.flat
    seed1001 = by_seed.get(1001, [])
    for row in seed1001:
        traj = row["_trajectory"]
        xs = [float(item["x"]) for item in traj]
        ys = [float(item["y"]) for item in traj]
        path_ax.plot(xs, ys, lw=2.0, color=colors[row["control_policy"]], label=row["control_policy"])
    if seed1001:
        path_ax.plot([p[0] for p in seed1001[0]["_route"]], [p[1] for p in seed1001[0]["_route"]], "k--", lw=1.2, label="pre-set patrol route")
        path_ax.scatter([seed1001[0]["_target"][0]], [seed1001[0]["_target"][1]], marker="*", s=180, color="#C62828", label="target")
    path_ax.set(title="Same-seed UGV paths (seed 1001)", xlabel="CARLA X (m)", ylabel="CARLA Y (m)", aspect="equal")
    path_ax.legend(frameon=False)

    for row in seed1001:
        traj = row["_trajectory"]
        tx, ty = row["_target"]
        times = [float(item["timestamp"]) - float(traj[0]["timestamp"]) for item in traj]
        distances = [math.hypot(float(item["x"]) - tx, float(item["y"]) - ty) for item in traj]
        distance_ax.plot(times, distances, color=colors[row["control_policy"]], lw=1.8, label=row["control_policy"])
    distance_ax.axhline(10.0, color="#777777", ls="--", lw=1)
    distance_ax.set(title="Distance to target", xlabel="Simulation time (s)", ylabel="Distance (m)")
    distance_ax.legend(frameon=False)

    for row in outage:
        safety = row["_safety"]
        if not safety:
            continue
        times = [float(item["timestamp"]) - float(safety[0]["timestamp"]) for item in safety]
        mode = [item.get("target_tracking_mode", "") for item in safety]
        code = [{"message_driven": 0, "stale_grace": 1, "safe_hold": 2, "patrol": -1}.get(value, -1) for value in mode]
        outage_label = Path(row["run_dir"]).name.split("OUTAGE_")[-1].split("_T10")[0]
        stale_ax.step(times, [value + 0.08 * outage.index(row) for value in code], where="post", lw=1.2, label=outage_label)
    if outage:
        stale_ax.set(title="State response during communication outages", xlabel="Simulation time (s)", ylabel="Mode (see legend)", yticks=[-1, 0, 1, 2], yticklabels=["patrol", "message route", "stale grace", "safe hold"])
        stale_ax.legend(frameon=False, title="Outage")
    else:
        stale_ax.set_title("Communication outage tests")
        stale_ax.text(
            0.5,
            0.5,
            "No outage-recovery runs in this ABC comparison batch",
            ha="center",
            va="center",
            transform=stale_ax.transAxes,
            color="#5B6573",
        )
        stale_ax.set_axis_off()

    policies = ["none", "ugv_local", "dual"]
    metric_labels = ["Final target distance (m)", "Route departure (m)"]
    x = np.arange(len(policies))
    width = 0.34
    for offset, (key, label) in enumerate((("final_target_distance_m", metric_labels[0]), ("mean_departure_from_patrol_route_m", metric_labels[1]))):
        means = [float(np.mean([r[key] for r in formal if r["control_policy"] == policy])) if any(r["control_policy"] == policy for r in formal) else 0 for policy in policies]
        summary_ax.bar(x + (offset - 0.5) * width, means, width, label=label, color=["#AAB2BD", "#55A6D9", "#E58955"][offset])
    summary_ax.set(title="Across completed comparison runs", ylabel="Meters", xticks=x, xticklabels=policies)
    summary_ax.legend(frameon=False)
    figure.suptitle(
        "E7 route-influence and stale-message recovery comparison" if outage
        else "E7 route-influence comparison (ABC; three paired seeds)",
        fontsize=16,
    )
    figure.savefig(root / "e7_comparison_overview.png", dpi=180)
    plt.close(figure)
    print(json.dumps({"run_count": len(runs), "paired_abc_run_count": len(comparison_runs), "ugv_local_positive_control_count": len(positive_control_runs), "metrics_csv": str(root / "comparison_metrics.csv"), "metrics_json": str(root / "comparison_metrics.json"), "group_summary_csv": str(root / "comparison_group_summary.csv"), "positive_control_csv": str(root / "ugv_local_positive_control.csv"), "figure": str(root / "e7_comparison_overview.png")}, ensure_ascii=False, indent=2))
    return 0


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


if __name__ == "__main__":
    raise SystemExit(main())
