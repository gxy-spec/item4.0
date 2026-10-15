"""Plot a full A2G task run as a compact trajectory and radio timeline."""

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


def csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run = args.run_dir.resolve()
    config = yaml.safe_load((run / "resolved_config.yaml").read_text(encoding="utf-8"))
    ugv = csv_rows(run / "trajectories/ugv_trajectory.csv")
    uav = csv_rows(run / "trajectories/uav_trajectory.csv")
    target = csv_rows(run / "trajectories/target_trajectory.csv")
    group = str(config["transport_link"]["control_group"])
    radio = [e for e in jsonl(run / "communication" / f"a2g_transport_events_{group}.jsonl")
             if e["event"] == "channel_sample"]
    arrivals = [e for e in jsonl(run / "communication" / f"a2g_transport_events_{group}.jsonl")
                if e["event"] == "available_valid"]
    if not ugv or not uav or not target or not radio:
        raise ValueError("Complete trajectories and radio samples are required")

    t0 = float(ugv[0]["timestamp"])
    ugv_t = np.array([float(row["timestamp"]) - t0 for row in ugv])
    ugv_xy = np.array([[float(row["x"]), float(row["y"])] for row in ugv])
    uav_xy = np.array([[float(row["x"]), float(row["y"])] for row in uav])
    target_xy = np.array([float(target[0]["x"]), float(target[0]["y"])])
    distance = np.linalg.norm(ugv_xy - target_xy, axis=1)
    radio_t = np.array([float(row["timestamp_s"]) - t0 for row in radio])
    snr = np.array([float(row["snr_db"]) for row in radio])
    rate = np.array([float(row["rate_bit_s"]) / 1e6 for row in radio])
    first_arrival_s = min(float(row["arrival_timestamp_s"]) - t0 for row in arrivals) if arrivals else None

    plt.rcParams.update({
        "font.family": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
        "font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
        "axes.linewidth": 0.8, "savefig.dpi": 300,
    })
    fig = plt.figure(figsize=(13.2, 6.3), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, width_ratios=[1.05, 1.25], hspace=0.18)
    ax_map = fig.add_subplot(grid[:, 0])
    ax_dist = fig.add_subplot(grid[0, 1])
    ax_radio = fig.add_subplot(grid[1, 1])
    blue, purple, orange = "#0072B2", "#7B3294", "#D55E00"

    boundary = np.array(config["region"]["boundary"]["points"], dtype=float)
    boundary = np.vstack((boundary, boundary[0]))
    ax_map.plot(boundary[:, 0], boundary[:, 1], ls="--", lw=1.0, color="#9BA7B4", label="E1 区域")
    ax_map.plot(ugv_xy[:, 0], ugv_xy[:, 1], color=blue, lw=2.5, label="UGV 轨迹")
    ax_map.plot(uav_xy[:, 0], uav_xy[:, 1], color=purple, lw=2.0, label="UAV 轨迹")
    ax_map.scatter(*ugv_xy[0], s=50, color=blue, marker="o", zorder=5)
    ax_map.scatter(*ugv_xy[-1], s=80, facecolor="white", edgecolor=blue, marker="o", lw=2.2, zorder=5)
    ax_map.scatter(*uav_xy[0], s=85, color=purple, marker="^", zorder=5)
    ax_map.scatter(*target_xy, s=180, color=orange, marker="*", edgecolor="white", lw=0.8,
                   label="目标车辆", zorder=6)
    ax_map.annotate("UGV 起点", ugv_xy[0], xytext=(-63, -17), textcoords="offset points", color=blue)
    ax_map.annotate("UGV 终点", ugv_xy[-1], xytext=(9, -16), textcoords="offset points", color=blue)
    ax_map.set(xlabel="CARLA X (m)", ylabel="CARLA Y (m)", title="(a) 同一回合的空地轨迹")
    ax_map.set_aspect("equal", adjustable="box")
    ax_map.grid(alpha=0.18)
    ax_map.legend(loc="lower left", fontsize=8.5, frameon=True, framealpha=0.95)

    ax_dist.plot(ugv_t, distance, color=blue, lw=2.0)
    ax_dist.scatter([ugv_t[0], ugv_t[-1]], [distance[0], distance[-1]], color=blue, s=28, zorder=4)
    ax_dist.annotate(f"起始 {distance[0]:.1f} m", (ugv_t[0], distance[0]),
                     xytext=(8, 5), textcoords="offset points", color=blue)
    ax_dist.annotate(f"结束 {distance[-1]:.1f} m", (ugv_t[-1], distance[-1]),
                     xytext=(-88, 9), textcoords="offset points", color=blue)
    if first_arrival_s is not None:
        ax_dist.axvline(first_arrival_s, color=orange, ls="--", lw=1.2,
                        label="首条 UAV 消息到达")
    ax_dist.set(xlim=(0, ugv_t[-1]), ylabel="UGV 与目标平面距离 (m)",
                title="(b) 任务进程（真实位置仅用于离线评价）")
    ax_dist.grid(alpha=0.2)
    ax_dist.legend(loc="upper right", fontsize=8.5, frameon=False)

    ax_radio.plot(radio_t, snr, color=purple, lw=1.25, label="SNR")
    ax_radio.axhline(float(config["transport_link"]["service"]["snr_working_threshold_db"]),
                     color=orange, ls="--", lw=1.1, label="停传门限")
    ax_radio.set(xlim=(0, ugv_t[-1]), xlabel="仿真时间 (s)", ylabel="SNR (dB)",
                 title="(c) 空地物理信道与有效服务速率")
    ax_radio.grid(alpha=0.2)
    ax_rate = ax_radio.twinx()
    ax_rate.plot(radio_t, rate, color="#009E73", lw=1.2, alpha=0.85, label="服务速率")
    ax_rate.set_ylabel("服务速率 (Mb/s)")
    ax_rate.set_ylim(0, max(11, 1.1 * max(rate)))
    lines = ax_radio.get_lines() + ax_rate.get_lines()
    ax_radio.legend(lines, [line.get_label() for line in lines], loc="lower right",
                    fontsize=8.5, frameon=False, ncol=3)

    fig.suptitle("E1-C 完整任务回合｜A2G 物理信道 v1｜种子 2101", fontsize=14)
    output = run / "preview" / "a2g_full_task_process"
    for suffix in (".png", ".pdf"):
        fig.savefig(output.with_suffix(suffix), bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(json.dumps({"figure": str(output.with_suffix(".png")), "duration_s": float(ugv_t[-1]),
                      "first_uav_arrival_s": first_arrival_s, "final_distance_m": float(distance[-1]),
                      "radio_sample_count": len(radio)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
