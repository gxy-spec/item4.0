#!/usr/bin/env python3
"""Build the full Markdown report for the frozen multiscene ABC batch."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


CASE_ORDER = ["N1", "N2", "N3", "E1", "E2", "E3", "W1", "W2", "W3", "F1", "F2", "F3"]
SCENE_NAMES = {"north": "北区", "east": "东区", "west": "西区", "whole": "整图"}
GROUP_NAMES = {"A": "固定巡检", "B": "UGV 本地感知", "C": "UAV+UGV 协同"}


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def link(label: str, path: Path | str) -> str:
    return f"[{label}]({Path(path).as_posix()})"


def fmt(value: Any, digits: int = 1) -> str:
    return f"{float(value):.{digits}f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    report_path = args.report.resolve()
    progress = load_json(root / "batch_progress.json")
    replay = load_json(root / "replay_validation.json")
    analysis = load_json(root / "analysis" / "analysis_summary.json")
    index = load_json(Path(progress["index"]))
    metrics_rows = read_csv(root / "analysis" / "paired_metrics_36.csv")
    if progress.get("status") != "COMPLETE" or len(progress.get("results", [])) != 36:
        raise RuntimeError("Batch index is not a complete 36-run batch")
    if replay.get("status") != "PASS" or len(replay.get("entries", [])) != 36:
        raise RuntimeError("Replay validation is not PASS for all 36 runs")

    run_by_label = {item["label"]: item for item in progress["results"]}
    replay_by_label = {item["label"]: item for item in replay["entries"]}
    metric_by_label = {row["label"]: row for row in metrics_rows}
    config_by_case = {item["case"]: item for item in index["runs"] if item["group"] == "A"}
    paired_plots = {Path(value).stem.replace("_route_distance", ""): Path(value)
                    for value in analysis["outputs"]["paired_trajectory_maps"]}

    lines: list[str] = []
    add = lines.append
    add("# Town10HD 多场景配对 A/B/C 正式实验报告（2026-10-10）")
    add("")
    add("## 目的与实验边界")
    add("")
    add("在 Town10HD 的北区、东区、西区和整图范围内，验证目标信息来源是否改变 UGV 巡检决策。共 12 个场景实例，每个场景用相同种子、目标/干扰物、起终点、UAV 航线、传感器与仿真时长分别运行 A/B/C，共 36 回合。")
    add("")
    add("- A：UGV 按预设巡检路线运行，不使用目标候选消息。")
    add("- B：UGV 仅使用自身 RGB-D 产生的本地候选。")
    add("- C：UAV 与 UGV 候选经语义消息选择后协同；UGV 的 RGB-D 安全避障始终保留。")
    add("- 本批次仍是理想语义通信：固定时延/TTL，不含带宽竞争、衰落和物理层随机丢包；因此不是物理信道 GOC 验证。CARLA 真值只用于仿真后评价、地图和候选框着色，不进入在线感知、选目标、规划或避障。")
    add("- 传感器图像采用 compact 保存：每 100 个固定步长 tick 保存 1 个四流同步样本，约每 5 s 一帧；轨迹、状态、消息与同步索引按仿真过程完整记录。回放视频是这些已保存帧的稀疏同步回放，不是连续录像。")
    add("")
    add("## 场景覆盖与正式验收")
    add("")
    add("| 区域 | 场景实例 / 种子 | 单回合仿真时长 | UAV 规划航线长度 |")
    add("|---|---|---:|---:|")
    for scene in ["north", "east", "west", "whole"]:
        cases = [case for case in CASE_ORDER if config_by_case[case]["scene"] == scene]
        meta = config_by_case[cases[0]]
        seeds = ", ".join(str(config_by_case[case]["seed"]) for case in cases)
        durations = ", ".join(f"{config_by_case[case]['horizon_seconds']:.0f} s" for case in cases)
        routes = ", ".join(f"{config_by_case[case]['uav_route_length_m']:.0f} m" for case in cases)
        add(f"| {SCENE_NAMES[scene]} | {', '.join(cases)} / {seeds} | {durations} | {routes} |")
    acceptance = analysis["acceptance"]
    replay_checks = replay["checks"]
    min_samples = min(item["frame_count"] for item in replay["entries"])
    max_samples = max(item["frame_count"] for item in replay["entries"])
    total_mb = sum(item["video_bytes"] for item in replay["entries"]) / 1_000_000
    add("")
    add(f"**验收结论：36/36 原生仿真 PASS，36/36 五视图回放 PASS。** 原生检查失败项 {acceptance['failed_check_count']}；四路共同帧比例最低 {100 * acceptance['minimum_common_frame_ratio']:.3f}%；UGV/UAV 碰撞各 {acceptance['total_ugv_collisions']}/{acceptance['total_uav_collisions']}；每回合至少有 {acceptance['minimum_moving_vehicles']} 辆运动干扰车和 {acceptance['minimum_moving_pedestrians']} 名运动行人。回放帧数范围 {min_samples}–{max_samples}，四路源帧 spread 全为 0，视频总计约 {total_mb:.1f} MB。")
    add("")
    add("回放把每个保存时刻的全局轨迹图、UAV RGB/深度、UGV RGB/深度放在同一画面中，四路使用同一 CARLA frame ID。视频为 2 fps 展示稀疏保存帧，画面标注实际仿真时间并声明中间时段没有保存/插帧；它适合核对同步观测与候选框，不可据此判断 5 s 间隔内的瞬态动作。")
    add("")
    add("## 正式结果")
    add("")
    add("指标定义：‘接近量’=初始 UGV–目标距离−最终距离，正值表示最终更近；‘路线偏离’为 UGV 轨迹各采样点到预设巡检折线的平均 2D 距离；‘≤10 m’按回合结束时距离统计。所有组间比较是 12 个配对场景上的描述性统计，不是显著性检验。")
    add("")
    add("| 组别 | n | 平均接近量 | 中位接近量 | 平均最终目标距离 | 结束时 ≤10 m | 平均路线偏离 | 目标路线规划 |")
    add("|---|---:|---:|---:|---:|---:|---:|---:|")
    for row in analysis["group_summary"]:
        plans = (f"{row['target_route_plans']}（UAV {row['uav_sourced_route_plans']}，"
                 f"UGV {row['ugv_sourced_route_plans']}）")
        add(f"| {row['method']} | {row['n']} | {fmt(row['mean_target_distance_reduction_m'])} m | "
            f"{fmt(row['median_target_distance_reduction_m'])} m | {fmt(row['mean_final_target_distance_m'])} m | "
            f"{row['final_within_10m_runs']}/{row['n']} | {fmt(row['mean_patrol_departure_m'])} m | {plans} |")
    add("")
    add("| 配对比较 | 接近量平均增益 | 中位增益 | 更近 / 更远 / 持平场景数 |")
    add("|---|---:|---:|---:|")
    for row in analysis["paired_effects"]:
        add(f"| {row['contrast']} | {fmt(row['mean_target_reduction_delta_m'])} m | "
            f"{fmt(row['median_target_reduction_delta_m'])} m | {row['cases_with_more_target_approach']} / "
            f"{row['cases_with_less_target_approach']} / {row['cases_tied_target_approach']} |")
    add("")
    add("**结果解释。** A 组平均路线偏离仅 0.34 m 且目标路线规划为 0，符合固定巡检基线；B 组 12 个场景中有 8 个生成目标路线，均由 UGV 本地候选触发；C 组 12 个都生成目标路线，其中 6 个由 UAV 候选、6 个由 UGV 候选触发。相对 B，C 的平均接近量增加 15.79 m，中位增加 5.43 m，12 个配对场景中 C 更近 9 个、更远 2 个、持平 1 个，说明协同信息整体有增益，但并非每个场景都更好。安全规则仍可能阻止进一步靠近：结束时只有 C 的 1/12 回合进入 10 m 内，不能将本批次表述为普遍完成近距离到达任务。")
    add("")
    add("C 组相对 B 组的增益因场景差异明显：例如有些场景两组最终路线几乎相同，而另一些场景 C 明显更接近目标；也存在 C 选到不利信息/路线后不如 B 的个例。应把该结果解释为多场景下的系统级趋势和异质性，而不是声称 UAV 一定带来增益。")
    add("")
    add("### 逐场景结果、轨迹图与同步回放")
    add("")
    add("下表‘C−B 接近增益’为 C 组接近量减 B 组接近量，正值表示 C 最终更接近目标。每个场景图左侧叠加 UAV 航线及 A/B/C 三条 UGV 实际轨迹，右侧为 UGV–目标距离随时间变化；A/B/C 五视图视频与原生验收 JSON 均逐组链接。")
    add("")
    add("| 场景 / 种子 | 时长 | 最终目标距离 A / B / C | C−B 接近增益 | UAV/UGV 轨迹与距离 | 同步回放 A / B / C | 原生验收 A / B / C |")
    add("|---|---:|---:|---:|---|---|---|")
    for case in CASE_ORDER:
        meta = config_by_case[case]
        final = {g: float(metric_by_label[f"{case}_{g}"]["final_target_distance_m"]) for g in "ABC"}
        gain = float(metric_by_label[f"{case}_C"]["target_distance_reduction_m"]) - float(metric_by_label[f"{case}_B"]["target_distance_reduction_m"])
        plot = paired_plots[case]
        replay_links = " / ".join(link(g, replay_by_label[f"{case}_{g}"]["video"]) for g in "ABC")
        report_links = " / ".join(link(g, run_by_label[f"{case}_{g}"]["validation_report"]) for g in "ABC")
        add(f"| {case} / {meta['seed']} | {meta['horizon_seconds']:.0f} s | "
            f"{fmt(final['A'])} / {fmt(final['B'])} / {fmt(final['C'])} m | {fmt(gain)} m | "
            f"{link('查看图', plot)} | {replay_links} | {report_links} |")
    add("")
    add("### 代表性可视化")
    add("")
    add(f"**四区域汇总：目标接近与巡检路线偏离。** 这两图是 12 个配对场景的描述性汇总，不代表显著性检验。")
    add("")
    add(f"![按场景的目标接近量]({Path(analysis['outputs']['target_plot']).as_posix()})")
    add("")
    add(f"![按场景的预设路线偏离]({Path(analysis['outputs']['route_plot']).as_posix()})")
    add("")
    add("以下每张图均含 UAV/UGV 实际路径；完整 12 张见上表或‘全部逐场景轨迹图’入口。")
    add("")
    for case in ["N1", "E1", "W1", "F1"]:
        add(f"**{case} 配对轨迹与目标距离：** {link('打开原图', paired_plots[case])}")
        add("")
        add(f"![{case} 的 UAV/UGV 配对轨迹]({paired_plots[case].as_posix()})")
        add("")
    add("## 输出与复核入口")
    add("")
    add(f"- 批次根目录：`{root}`。")
    add(f"- 批次进度与 36 组目录：{link('batch_progress.json', root / 'batch_progress.json')}；包含全部回合路径及运行概要。")
    add(f"- 36 组原生验收路径索引：{link('all_validation_paths.tsv', root / 'all_validation_paths.tsv')}；包含逐回合 `validation_report.json`、preview、日志和关键检查字段。")
    add(f"- 回放验收：{link('replay_validation.json', root / 'replay_validation.json')}；逐个记录 36 段视频、首帧图、源帧范围、帧数、尺寸、同步 spread 和通过状态。")
    add(f"- 可重绘指标：{link('paired_metrics_36.csv', root / 'analysis' / 'paired_metrics_36.csv')}、{link('group_summary.csv', root / 'analysis' / 'group_summary.csv')}、{link('paired_effects.csv', root / 'analysis' / 'paired_effects.csv')}。")
    add(f"- 机器可读分析和验收摘要：{link('analysis_summary.json', root / 'analysis' / 'analysis_summary.json')}。")
    add(f"- 12 张逐场景 UAV/UGV 轨迹与距离图：{link('paired_trajectories/', root / 'analysis' / 'paired_trajectories')}。")
    add(f"- 正式实验配置索引：{link('formal_36_run_index.json', Path(progress['index']))}。")
    add("")
    add("## 限制与下一步")
    add("")
    add("1. 12 个配对场景可支持系统级趋势比较，但仍是描述性结果；后续需要更多独立随机种子，并报告配对置信区间/适当统计检验。")
    add("2. 本批次还没有物理带宽、空地衰落、阴影遮挡与随机丢包，不能据此回答物理信道约束下的层级通信与 VoI 问题。")
    add("3. 传感器仅存每约 5 s 的同步样本，因此目前回放用于核对采样对齐与典型观测，不用于判断间隔内瞬时行为；完整轨迹、状态和通信日志仍按回合保存。需要连续回放时必须重新采集更高频传感器数据。")
    add("4. 目标接近受 RGB-D 避障和安全停车约束；10 m 内回合数只有 C 组 1/12。不能为提高接近率而绕过既有安全模块，应先明确任务成功定义与安全停车语义。")
    add("")
    add("本报告引用已完成的 36 组批次与离线生成的逐组回放，不触发仿真重跑。")

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"REPORT={report_path}")
    print(f"CASE_ROWS={len(CASE_ORDER)} VIDEO_LINKS={3 * len(CASE_ORDER)} NATIVE_REPORT_LINKS={3 * len(CASE_ORDER)}")
    print(f"BYTES={report_path.stat().st_size}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
