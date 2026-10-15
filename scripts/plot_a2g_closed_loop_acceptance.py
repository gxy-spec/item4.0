"""Plot the online A2G closed-loop acceptance timeline from saved artifacts."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np


BLUE = "#0072B2"
ORANGE = "#E69F00"
GREEN = "#009E73"
RED = "#D55E00"
GREY = "#6B7280"
LIGHT_GREY = "#D1D5DB"


def read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def configure_chinese_font() -> None:
    available = {item.name for item in font_manager.fontManager.ttflist}
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Arial Unicode MS"):
        if name in available:
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["pdf.fonttype"] = 42
    plt.rcParams["ps.fonttype"] = 42


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run = args.run_dir.resolve()
    preview = run / "preview"
    preview.mkdir(exist_ok=True)

    transport = json.loads((run / "communication/a2g_transport_summary.json").read_text(encoding="utf-8"))
    radio = read_jsonl(run / "communication/a2g_transport_events_geo.jsonl")
    communication = read_jsonl(run / "communication/communication_events.jsonl")
    memory = read_jsonl(run / "communication/target_memory_events.jsonl")
    routes = read_jsonl(run / "planning/target_route_events.jsonl")
    safety = list(csv.DictReader((run / "safety/safety_state.csv").open(encoding="utf-8-sig", newline="")))
    trace = list(csv.DictReader((run / "communication/spatial_link_trace.csv").open(encoding="utf-8-sig", newline="")))

    t0 = float(safety[0]["timestamp"])
    rel = lambda value: float(value) - t0
    time_s = np.array([rel(row["timestamp"]) for row in safety])
    trace_time_s = np.array([rel(row["timestamp_s"]) for row in trace])
    raw_snr = np.array([float(row["geo_snr_db"]) for row in trace])
    effective_snr = np.array([float(row["geo_effective_snr_db"]) for row in trace])
    tracking = np.array([1.0 if row["target_tracking_mode"] == "memory_target" else 0.0 for row in safety])
    speed = np.array([float(row["ugv_speed_mps"]) for row in safety])

    fade = transport["diagnostic_attenuation_intervals_s"][0]
    fade_start, fade_end = float(fade["start_s"]), float(fade["end_s"])
    stale_times = [rel(row["timestamp"]) for row in safety if row["ugv_obstacle_sensor_state"] == "SENSOR_STALE"]
    stale_start, stale_end = min(stale_times), max(stale_times) + np.median(np.diff(time_s))

    sent_times = [rel(event["source_timestamp_s"]) for event in communication if event["event"] == "sent"]
    valid_times = [rel(event["timestamp_s"]) for event in radio if event["event"] == "available_valid"]
    expired_times = [rel(event["timestamp_s"]) for event in radio if event["event"].startswith("expired_")]
    route_times = [rel(event["timestamp_s"]) for event in routes if event["event"] == "target_route_planned_from_selected_message"]
    refresh_times = [rel(event["timestamp_s"]) for event in memory if event["event"] == "memory_same_target_refresh"]

    configure_chinese_font()
    plt.rcParams.update({
        "font.size": 10.5,
        "axes.labelsize": 10.5,
        "axes.titlesize": 11.5,
        "legend.fontsize": 9.5,
        "xtick.labelsize": 9.5,
        "ytick.labelsize": 9.5,
        "axes.linewidth": 0.8,
    })
    fig, axes = plt.subplots(
        4, 1, figsize=(11.8, 9.2), sharex=True,
        gridspec_kw={"height_ratios": [1.45, 1.0, 0.9, 1.25], "hspace": 0.24},
    )
    fig.suptitle("A2G 物理链路接入闭环验收（Town10HD，Seed 2101）", fontsize=15, y=0.985)

    for ax in axes:
        ax.axvspan(fade_start, fade_end, color=RED, alpha=0.075, lw=0)
        ax.grid(axis="x", color=LIGHT_GREY, linewidth=0.6, alpha=0.65)
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_xlim(0, max(time_s))

    ax = axes[0]
    ax.plot(trace_time_s, raw_snr, color=GREY, lw=1.2, ls="--", label="传播模型原始 SNR")
    ax.plot(trace_time_s, effective_snr, color=BLUE, lw=1.8, label="进入传输队列的有效 SNR")
    ax.axhline(5.0, color=RED, lw=1.1, ls=":", label="停传门限 5 dB")
    ax.set_ylabel("SNR（dB）")
    ax.set_title("(a) 信道状态：诊断衰减使有效 SNR 低于门限")
    ax.legend(loc="lower right", ncol=3, frameon=False)
    ax.text((fade_start + fade_end) / 2, ax.get_ylim()[0] + 2.5, "诊断衰减 80 dB",
            ha="center", va="bottom", color=RED, fontsize=9.5)

    ax = axes[1]
    ax.scatter(sent_times, np.full(len(sent_times), 2), s=25, facecolors="none", edgecolors=GREY,
               linewidths=1.1, label=f"发送（{len(sent_times)}）", zorder=3)
    ax.scatter(valid_times, np.full(len(valid_times), 1), s=38, marker="o", color=GREEN,
               label=f"TTL 内到达（{len(valid_times)}）", zorder=3)
    ax.scatter(expired_times, np.full(len(expired_times), 0), s=43, marker="x", color=RED,
               linewidths=1.7, label=f"TTL 过期（{len(expired_times)}）", zorder=3)
    ax.set_yticks([0, 1, 2], ["过期", "有效到达", "发送"])
    ax.set_ylim(-0.55, 2.55)
    ax.set_title("(b) 消息时序：过期消息未进入接收与目标选择")
    ax.legend(loc="upper right", ncol=3, frameon=False)

    ax = axes[2]
    ax.step(time_s, tracking, where="post", color=BLUE, lw=1.8)
    ax.fill_between(time_s, 0, tracking, step="post", color=BLUE, alpha=0.10)
    for index, value in enumerate(route_times):
        ax.axvline(value, color=GREEN, lw=1.4, label="首条有效消息生成航线" if index == 0 else None)
    if refresh_times:
        ax.scatter(refresh_times, np.full(len(refresh_times), 1.08), marker="|", s=70,
                   color=ORANGE, linewidths=1.4, label="目标记忆刷新", clip_on=False)
    ax.set_yticks([0, 1], ["巡检", "目标记忆"])
    ax.set_ylim(-0.12, 1.22)
    ax.set_title("(c) 任务控制：断联期间保持已确认目标航线")
    ax.legend(loc="lower right", ncol=2, frameon=False)

    ax = axes[3]
    ax.plot(time_s, speed, color=BLUE, lw=1.7, label="UGV 速度")
    ax.axvspan(stale_start, stale_end, color=ORANGE, alpha=0.20, lw=0,
               label="RGB-D SENSOR_STALE 安全停车")
    ax.axhline(0, color=GREY, lw=0.7)
    ax.set_ylabel("速度（m/s）")
    ax.set_xlabel("仿真相对时间（s）")
    ax.set_title("(d) 本地安全：传感器过期时停车，恢复后继续行驶")
    ax.legend(loc="upper right", ncol=2, frameon=False)

    fig.text(0.5, 0.012,
             "红色浅色区：3–12 s 诊断衰减；橙色区：实测 SENSOR_STALE 状态。阴影均为验收干预，不代表实测城市信道。",
             ha="center", va="bottom", fontsize=9, color="#4B5563")
    fig.subplots_adjust(left=0.115, right=0.985, top=0.94, bottom=0.075)
    png = preview / "a2g_closed_loop_acceptance.png"
    pdf = preview / "a2g_closed_loop_acceptance.pdf"
    fig.savefig(png, dpi=300, bbox_inches="tight", facecolor="white")
    fig.savefig(pdf, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(json.dumps({"png": str(png), "pdf": str(pdf)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
