#!/usr/bin/env python3
"""Audit and plot the three-scene target-memory smoke against frozen v3 runs."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt


DATA = Path(r"E:\CarlaAirData\CityInspection_GOC\persistent_memory_smoke_20261012")
REFERENCE = Path(r"E:\CarlaAirData\CityInspection_GOC\multiscene_abc_expiry_coast_20261010")
CASES = ("N1", "E1", "W2")


def run_dir(root: Path, case: str) -> Path:
    choices = sorted((root / case / "C").glob("CI_E7_MULTISCENE_FORMAL_*"))
    if len(choices) != 1:
        raise RuntimeError(f"Expected one frozen {case}/C run under {root}, found {choices}")
    return choices[0]


def read_csv(path: Path) -> list[dict]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def distance_trace(run: Path) -> tuple[list[float], list[float], list[float], list[float]]:
    ugv = read_csv(run / "trajectories" / "ugv_trajectory.csv")
    uav = read_csv(run / "trajectories" / "uav_trajectory.csv")
    target = read_csv(run / "trajectories" / "target_trajectory.csv")[0]
    tx, ty = float(target["x"]), float(target["y"])
    t0 = float(ugv[0]["timestamp"])
    time = [float(row["timestamp"]) - t0 for row in ugv]
    dist = [math.hypot(float(row["x"]) - tx, float(row["y"]) - ty) for row in ugv]
    return time, dist, [float(row["x"]) for row in uav], [float(row["y"]) for row in uav]


def main() -> None:
    verification = {}
    fig, axes = plt.subplots(3, 2, figsize=(11.0, 9.0), constrained_layout=True)
    for row_index, case in enumerate(CASES):
        new, old = run_dir(DATA, case), run_dir(REFERENCE, case)
        current = json.loads((new / "validation_report.json").read_text(encoding="utf-8"))
        previous = json.loads((old / "validation_report.json").read_text(encoding="utf-8"))
        comm = current["summary"]["persistent_target_communication"]
        memory = read_jsonl(new / "communication" / "target_memory_events.jsonl")
        routes = read_jsonl(new / "planning" / "target_route_events.jsonl")
        safety = read_csv(new / "safety" / "safety_state.csv")
        selected = read_jsonl(new / "communication" / "selected_target_states.jsonl")
        commits = next(check for check in current["checks"]
                       if check["name"] == "memory_commits_match_offline_target_truth")
        memory_ticks = [r for r in safety if r["target_tracking_mode"] == "memory_target"]
        selected_times = sorted(float(item["timestamp_s"]) for item in selected)
        longest_update_gap = max((b - a for a, b in zip(selected_times, selected_times[1:])), default=0.0)
        verification[case] = {
            "new_run": str(new), "reference_v3_run": str(old),
            "status": current["status"],
            "target_route_plans": comm["target_route_plan_count"],
            "ttl_route_revocations": comm["target_route_return_to_patrol_count"],
            "confirmed_target_switches": comm["memory_confirmed_switch_count"],
            "unconfirmed_challenger_observations": comm["memory_unconfirmed_challenger_count"],
            "offline_commit_errors_m": commits["value"]["commit_errors_m"],
            "memory_route_control_ticks": len(memory_ticks),
            "memory_route_rgbd_guard_ticks": sum(
                r.get("ugv_obstacle_sensor_state") in {"CLEAR", "CAUTION", "STOP", "SENSOR_STALE"}
                for r in memory_ticks
            ),
            "longest_gap_between_selected_messages_s": longest_update_gap,
            "new_final_target_distance_m": current["summary"]["ugv_final_target_distance_m"],
            "reference_v3_final_target_distance_m": previous["summary"]["ugv_final_target_distance_m"],
            "ugv_collisions": current["summary"]["ugv_collision_count"],
            "uav_collisions": current["summary"]["uav_collision_count"],
            "route_events": len(routes), "memory_events": len(memory),
        }
        ax_map, ax_dist = axes[row_index]
        trace_new = read_csv(new / "trajectories" / "ugv_trajectory.csv")
        trace_old = read_csv(old / "trajectories" / "ugv_trajectory.csv")
        uav = read_csv(new / "trajectories" / "uav_trajectory.csv")
        target = read_csv(new / "trajectories" / "target_trajectory.csv")[0]
        ax_map.plot([float(r["x"]) for r in uav], [float(r["y"]) for r in uav],
                    color="#8161a9", lw=1.0, alpha=0.55, label="UAV survey")
        ax_map.plot([float(r["x"]) for r in trace_old], [float(r["y"]) for r in trace_old],
                    color="#a8adb5", lw=1.7, label="Previous TTL recovery")
        ax_map.plot([float(r["x"]) for r in trace_new], [float(r["y"]) for r in trace_new],
                    color="#1766a4", lw=2.0, label="Persistent target memory")
        ax_map.scatter(float(target["x"]), float(target["y"]), marker="*", s=95,
                       color="#bd3d31", label="Target", zorder=5)
        ax_map.set(xlabel="CARLA world X (m)", ylabel="CARLA world Y (m)", title=f"{case}: trajectories")
        ax_map.set_aspect("equal", adjustable="datalim")
        old_time, old_dist, _, _ = distance_trace(old)
        new_time, new_dist, _, _ = distance_trace(new)
        ax_dist.plot(old_time, old_dist, color="#a8adb5", lw=1.7, label="Previous TTL recovery")
        ax_dist.plot(new_time, new_dist, color="#1766a4", lw=2.0, label="Persistent target memory")
        ax_dist.axhline(10.0, color="#bd3d31", lw=1.0, ls="--", label="10 m reference")
        ax_dist.set(xlabel="Simulation time (s)", ylabel="UGV-target distance (m)",
                    title=f"{case}: target approach")
        for axis in (ax_map, ax_dist):
            axis.grid(alpha=0.25, lw=0.6)
            axis.spines[["top", "right"]].set_visible(False)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    distance_handles, distance_labels = axes[0, 1].get_legend_handles_labels()
    handles.append(distance_handles[-1])
    labels.append(distance_labels[-1])
    fig.legend(handles, labels, loc="lower center", ncol=5, frameon=False,
               bbox_to_anchor=(0.5, -0.025), fontsize=9)
    fig.suptitle("Packet expiry versus persistent target memory: three-scene smoke",
                 fontsize=13, fontweight="semibold", y=1.025)
    out_dir = DATA / "comparison"
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / "three_scene_memory_comparison.png", dpi=190, bbox_inches="tight")
    plt.close(fig)
    (out_dir / "verification.json").write_text(
        json.dumps(verification, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(out_dir)


if __name__ == "__main__":
    main()
