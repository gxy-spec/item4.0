#!/usr/bin/env python3
"""Combine three 120-s paired-seed experiments without hiding failed checks."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT = Path("E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/day56_paired_seeds")
COLORS = {"A": "#68737d", "B": "#2176a3", "C": "#c76931"}


def main() -> int:
    reports = {seed: json.loads((ROOT / f"S{seed}" / "paired_acceptance.json").read_text(encoding="utf-8"))
               for seed in (1001, 2001, 3001)}
    rows = []
    for seed, report in reports.items():
        for group in "ABC":
            metrics = report["metrics"][group]
            rows.append({"seed": seed, "group": group, "seed_status": report["status"],
                         "min_target_distance_m": metrics["minimum_target_distance_m"],
                         "first_uav_discovery_s": metrics["first_correct_discovery_uav_s"],
                         "first_ugv_discovery_s": metrics["first_correct_discovery_ugv_s"],
                         "first_correct_route_s": metrics["first_correct_route_s"],
                         "first_within_10m_s": metrics["first_within_10m_s"],
                         "first_within_15m_s": metrics["first_within_15m_s"],
                         "uav_routes": metrics["uav_route_plans"], "ugv_routes": metrics["ugv_route_plans"],
                         "wrong_routes": metrics["wrong_route_plans"],
                         "ugv_collisions": metrics["ugv_collisions"], "uav_collisions": metrics["uav_collisions"],
                         "run_dir": metrics["run_dir"]})
    output_csv = ROOT / "three_seed_comparison.csv"
    with output_csv.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    font = Path("C:/Windows/Fonts/msyh.ttc")
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.unicode_minus": False})
    fig, axes = plt.subplots(1, 3, figsize=(13.7, 4.3), layout="constrained")
    seeds = np.asarray([1001, 2001, 3001])
    x = np.arange(3)
    for index, group in enumerate("ABC"):
        values = [reports[int(seed)]["metrics"][group]["minimum_target_distance_m"] for seed in seeds]
        axes[0].bar(x + (index - 1) * 0.25, values, width=0.22, color=COLORS[group], label=f"{group}组")
    axes[0].axhline(10, color="#aa3439", ls="--", lw=1, label="10 m门限")
    axes[0].set(xticks=x, xticklabels=seeds, ylabel="最小目标距离（m）", title="(a) 任务接近")
    axes[0].legend(frameon=False, fontsize=8)
    for index, role in enumerate(("uav", "ugv")):
        color = "#8b5b9f" if role == "uav" else "#2877ad"
        for i, seed in enumerate(seeds):
            value = reports[int(seed)]["metrics"]["A"][f"first_correct_discovery_{role}_s"]
            xx = x[i] + (index - 0.5) * 0.28
            if value is None:
                axes[1].text(xx, 112, "未发现", ha="center", va="center", fontsize=8, color=color,
                             rotation=90)
            else:
                axes[1].vlines(xx, 0, value, color=color, lw=1.4, alpha=0.65)
                axes[1].scatter(xx, value, color=color, s=46, zorder=4)
                axes[1].annotate(f"{value:.1f}", (xx, value), xytext=(0, 5), textcoords="offset points",
                                 ha="center", fontsize=7, color=color)
        axes[1].plot([], [], marker="o", color=color, linestyle="-", label=role.upper())
    axes[1].set(xticks=x, xticklabels=seeds, ylabel="首次正确候选时间（s）", title="(b) 两端真实观测", ylim=(0, 125))
    axes[1].legend(frameon=False, fontsize=8)
    for index, group in enumerate("BC"):
        color = COLORS[group]
        for i, seed in enumerate(seeds):
            value = reports[int(seed)]["metrics"][group]["first_correct_route_s"]
            xx = x[i] + (index - 0.5) * 0.28
            if value is None:
                axes[2].text(xx, 112, "未生成路线", ha="center", va="center", fontsize=8,
                             color=color, rotation=90)
            else:
                axes[2].vlines(xx, 0, value, color=color, lw=1.4, alpha=0.65)
                axes[2].scatter(xx, value, color=color, s=46, zorder=4)
                axes[2].annotate(f"{value:.1f}", (xx, value), xytext=(0, 5), textcoords="offset points",
                                 ha="center", fontsize=7, color=color)
        axes[2].plot([], [], marker="o", color=color, linestyle="-", label=f"{group}组")
    axes[2].set(xticks=x, xticklabels=seeds, ylabel="首次正确目标路线时间（s）", title="(c) 观测转化为决策", ylim=(0, 125))
    axes[2].legend(frameon=False, fontsize=8)
    for axis in axes:
        axis.set_xlabel("配对随机种子")
        axis.grid(axis="y", alpha=0.18)
    figure = ROOT / "three_seed_academic_summary"
    for suffix in ("png", "pdf"):
        fig.savefig(figure.with_suffix(f".{suffix}"), dpi=230, bbox_inches="tight")
    plt.close(fig)
    status = "PASS" if all(report["status"] == "PASS" for report in reports.values()) else "PARTIAL"
    summary = {"stage": "Day 5–6 three 120-s paired seeds", "status": status,
               "note": "Each seed uses a frozen scene across A/B/C; different target/UAV geometry across seeds is deliberate."
                       " All GT matches are post-run evaluation only.",
               "seed_statuses": {str(seed): report["status"] for seed, report in reports.items()},
               "metrics_csv": str(output_csv),
               "figures": [str(figure.with_suffix(f".{suffix}")) for suffix in ("png", "pdf")],
               "seed_reports": {str(seed): str(ROOT / f"S{seed}" / "paired_acceptance.json") for seed in reports}}
    out = ROOT / "three_seed_summary.json"
    out.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"status": status, "summary": str(out), "figure": str(figure.with_suffix(".png"))}, ensure_ascii=False))
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
