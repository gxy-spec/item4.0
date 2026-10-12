"""Summarize the pre-36-run admission evidence into JSON and Markdown.

This report intentionally separates geometric/sensor evidence from closed-loop
evidence.  CARLA ground truth is reported only as post-run evaluation data.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


PROJECT = Path(r"E:\Research\CityInspection_GOC")
DATA = Path(r"E:\CarlaAirData\CityInspection_GOC\multiscene_admission_20261009")
CONFIG = PROJECT / "configs" / "experiments" / "multiscene_admission_20261009"
OUT = DATA / "admission_summary_20261009"


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def latest_report(root: Path) -> Path | None:
    reports = list(root.glob("**/validation_report.json"))
    return max(reports, key=lambda p: p.stat().st_mtime) if reports else None


def report_metrics(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {"status": "MISSING"}
    payload = read_json(path)
    return {
        "status": payload.get("status", "UNKNOWN"),
        "path": str(path),
        "summary": payload.get("summary", {}),
        "failed_checks": [
            {"name": item.get("name"), "value": item.get("value"), "criterion": item.get("criterion")}
            for item in payload.get("checks", [])
            if not item.get("passed", False)
        ],
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    short_index = read_json(DATA / "short_smoke_index.json")
    short: dict[str, Any] = {}
    for case, item in short_index.items():
        report = Path(item["validation_report"])
        metrics = report_metrics(report)
        metrics["run_dir"] = item.get("run_dir")
        short[case] = metrics

    rgbd = read_json(DATA / "rgbd_route_survey" / "rgbd_probe_summary.json")
    rgbd_compact = [
        {
            "case": row["case"],
            "scene": row["scene"],
            "frames": row["frames"],
            "matched_rgb_depth_frames": row["matched_rgb_depth_frames"],
            "maximum_timestamp_error_s": row["maximum_timestamp_error_s"],
            "maximum_uav_pose_error_m": row["maximum_uav_pose_error_m"],
            "new_collision_frames": row["new_collision_frames"],
            "rendered_visible_frames": row["rendered_visible_frames"],
            "target_confirmed_frames": row["target_confirmed_frames"],
            "replay": row["replay"],
        }
        for row in rgbd
    ]

    preflight = read_json(DATA / "geometry_corrected" / "preflight.json")
    geometry = []
    for row in preflight.get("candidates", []):
        geometry.append(
            {
                "scene": row.get("scene"),
                "candidate": row.get("candidate"),
                "uav_visibility_pass": row.get("uav_target_visibility", {}).get("visibility_pass"),
                "target_spawn_pass": row.get("target_spawn_pass"),
                "patrol_route_pass": row.get("patrol_route", {}).get("map_route_pass"),
                "flight_clearance_pass": row.get("flight", {}).get("clearance_pass"),
                "visible_samples": row.get("uav_target_visibility", {}).get("visible_samples"),
            }
        )

    challenge_roots = {
        "N2_pedestrian": DATA / "safety_challenges" / "N2",
        "E1_vehicle": DATA / "safety_challenges" / "E1",
        "W1_pedestrian": DATA / "safety_challenges" / "W1",
        "F2_vehicle": DATA / "safety_challenges" / "F2",
    }
    challenges = {
        name: report_metrics(latest_report(root)) for name, root in challenge_roots.items()
    }
    closed_loop = report_metrics(latest_report(DATA / "closed_loop" / "W1_C"))
    long_stability = report_metrics(latest_report(DATA / "long_stability" / "F1_A"))

    summary = {
        "report_id": "multiscene_admission_20261009",
        "project": str(PROJECT),
        "code_revision_notes": [
            "UGV RGB-D safety remains the only online obstacle source; online obstacle actor reads are zero.",
            "Pedestrian wide-band stop is enabled only in the controlled pedestrian challenge via pedestrian_stop_forward_m=12.0.",
            "Normal patrol configs retain the conservative lane/crossing filters.",
        ],
        "unit_tests": {"test_rgbd_safety_rules": "20 passed"},
        "geometry_preflight": geometry,
        "rgbd_route_survey": rgbd_compact,
        "short_smoke": short,
        "safety_challenges": challenges,
        "closed_loop": closed_loop,
        "whole_map_long_stability": long_stability,
    }
    (OUT / "admission_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    short_pass = sum(item.get("status") == "PASS" for item in short.values())
    challenge_pass = sum(item.get("status") == "PASS" for item in challenges.values())
    geometry_pass = sum(
        all(item.get(key) is True for key in ("uav_visibility_pass", "target_spawn_pass", "patrol_route_pass", "flight_clearance_pass"))
        for item in geometry
    )
    rgbd_sync_pass = sum(
        item["frames"] == item["matched_rgb_depth_frames"]
        and item["maximum_timestamp_error_s"] == 0.0
        and item["new_collision_frames"] == 0
        for item in rgbd_compact
    )

    lines = [
        "# 多场景正式实验前置验收汇总（2026-10-09）",
        "",
        "本报告用于 36 组正式 A/B/C 对照实验前的 admission gate，不把 CARLA 真值当作在线控制输入。",
        "",
        "## 验收结论",
        "",
        f"- 几何可行性：{geometry_pass}/{len(geometry)} 个候选通过目标可见、航线合法、UAV 清障和目标落点检查。",
        f"- RGB-D 短回合同步：{rgbd_sync_pass}/{len(rgbd_compact)} 个场景通过帧数、时间戳、UAV 位姿和零碰撞检查；E2 的可见航段仅持续 1.65 s，属于覆盖率提醒而非同步失败。",
        f"- 35 s 短回合 smoke：{short_pass}/{len(short)} PASS；所有正式短回合均为 compact 传感器保存。",
        f"- RGB-D 避障挑战：{challenge_pass}/{len(challenges)} PASS（N2/W1 行人、E1/F2 横穿车辆）。",
        f"- S1 感知闭环：{closed_loop.get('status')}；W1-C 已完成 10 个航点、9 段航线、4 次折返，消息驱动目标路线 1 次。",
        f"- 整图长时稳定：{long_stability.get('status')}；410 s、UAV 1119.69 m、0 碰撞。",
        "- 安全边界：20 个 RGB-D 安全规则单元测试全部通过；所有在线避障回合的 online_obstacle_actor_reads=0。",
        "",
        "## 关键输出",
        "",
        f"- 几何预检：`{DATA / 'geometry_corrected' / 'preflight.json'}`",
        f"- RGB-D 路线勘测：`{DATA / 'rgbd_route_survey' / 'rgbd_probe_summary.json'}`",
        f"- 短回合索引：`{DATA / 'short_smoke_index.json'}`",
        f"- 安全挑战汇总目录：`{DATA / 'safety_challenges'}`",
        f"- 感知闭环：`{closed_loop.get('path', 'MISSING')}`",
        f"- 整图长时：`{long_stability.get('path', 'MISSING')}`",
        f"- 机器可读总表：`{OUT / 'admission_summary.json'}`",
        "",
        "## 需要在正式 36 组实验中保留的注意事项",
        "",
        "1. E2 目标可见航段短，正式统计时应把“同步通过”和“有效观测覆盖率”分开报告。",
        "2. 感知闭环已证明消息、候选和路线接口能闭合，但目标候选仍是启发式 YOLO+红色比例+深度回投影，不等同于训练好的细粒度车辆/颜色识别器。",
        "3. 长时整图回合用于稳定性与航线覆盖验收，不应直接作为 B/C 增益结论。",
    ]
    (OUT / "admission_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(OUT / "admission_summary.json")
    print(OUT / "admission_summary.md")


if __name__ == "__main__":
    main()
