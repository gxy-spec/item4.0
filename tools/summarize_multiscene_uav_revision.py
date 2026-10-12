"""Assemble measured evidence for the W/F UAV route revision."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, Normalize
from matplotlib.patches import Rectangle
import numpy as np
from PIL import Image

from plot_multiscene_design import setup_font

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from sensors.depth import colourise_depth  # noqa: E402
OUT = Path("E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2")
FIGURES = ROOT / "docs/figures/multiscene_design_20261009_v2"
REPORT = ROOT / "docs/experiments/multiscene_uav_route_revision_v2_20261009.md"


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def length(route):
    return sum(math.dist(a, b) for a, b in zip(route, route[1:]))


def main() -> None:
    before = {(c["scene"], c["candidate"]): c for c in load(OUT / "corrected_original/preflight.json")["candidates"]}
    after = {(c["scene"], c["candidate"]): c for c in load(OUT / "preflight.json")["candidates"]}
    sweep = load(OUT / "uav_route_sweep.json")
    probes = load(OUT / "rgbd_probe_summary.json")
    old_manifest = load(ROOT / "docs/figures/multiscene_design_20261009/proposal_manifest.json")
    manifest = load(FIGURES / "proposal_manifest.json")
    comparison = []
    for key, current in after.items():
        prior = before[key]
        a, b = prior["uav_target_visibility"], current["uav_target_visibility"]
        comparison.append({"case": ("W" if key[0] == "west" else "F") + str(key[1]),
            "original_unique_visible_samples": a["visible_samples"],
            "revised_unique_visible_samples": b["visible_samples"],
            "original_longest_geometric_visible_path_m": a["maximum_contiguous_visible_path_m"],
            "revised_longest_geometric_visible_path_m": b["maximum_contiguous_visible_path_m"],
            "target_pose_drift_m": current["target_pose_drift_m"]})
    geometry_ok = all(c["flight"]["clearance_pass"] and c["uav_target_visibility"]["visibility_pass"]
                      and c["target_pose_drift_m"] < 0.001 for c in after.values())
    structure = []
    for name in ("west", "whole"):
        route = manifest["scenarios"][name]["uav_lawnmower_waypoints_xyz"]
        legs = [tuple(sorted((tuple(a), tuple(b)))) for a, b in zip(route, route[1:])]
        structure.append({"scene": name, "waypoints": len(route), "legs": len(legs),
                          "non_repeated_legs": len(set(legs)) == len(legs),
                          "nonzero_legs": all(math.dist(a, b) > 0 for a, b in zip(route, route[1:])),
                          "length_m": length(route), "nominal_flight_seconds_at_3mps": length(route) / 3})
    replay_checks = []
    for p in probes:
        cap = cv2.VideoCapture(p["replay"])
        replay_checks.append({"case": p["case"], "opened": cap.isOpened(),
                              "frames": int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),
                              "fps": cap.get(cv2.CAP_PROP_FPS)})
        cap.release()
    result = {"revision": 2,
              "scope": "UAV route geometry, collision-aware command sweep and 12-second RGB-D windows",
              "historical_visibility_counts_invalidated": True,
              "historical_error": "unsettled target actor with a fixed ray endpoint, plus repeated leg endpoints",
              "route_structure": structure, "geometric_comparison": comparison,
              "airsim_route_sweep": sweep, "rgbd_probes": probes,
              "replay_checks": replay_checks,
              "actor_deployments_unchanged": all(old_manifest["scenarios"][k]["review_candidates"] ==
                  manifest["scenarios"][k]["review_candidates"] for k in manifest["scenarios"]),
              "route_revision_physical_pass": geometry_ok and all(s["accepted"] for s in sweep)
                  and all(s["non_repeated_legs"] and s["nonzero_legs"] for s in structure),
              "short_rgbd_probe_pass": all(p["frames"] == p["matched_rgb_depth_frames"]
                  and p["maximum_timestamp_error_s"] == 0 and p["maximum_uav_pose_error_m"] <= 0.5
                  and p["new_collision_frames"] == 0 and p["rendered_visible_frames"] >= 0.9 * p["frames"]
                  and p["target_confirmed_frames"] > 0 for p in probes),
              "formal_abc_runs_performed": False,
              "f1_yolo_target_box_match_fraction": next(
                  p["yolo_target_box_frames"] / p["inference_frames"] for p in probes if p["case"] == "F1")}
    (OUT / "route_revision_summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    setup_font()
    fig, axes = plt.subplots(3, 2, figsize=(11, 12.6))
    fig.subplots_adjust(left=0.025, right=0.925, top=0.92, bottom=0.07, hspace=0.18, wspace=0.035)
    for row, p in enumerate(probes):
        case_dir = OUT / "rgbd_probe" / p["case"]
        checks = load(case_dir / "frame_checks.json")
        candidates = load(case_dir / "candidate_checks.json")
        current = checks[120]
        axes[row, 0].imshow(Image.open(case_dir / "sample_120_rgb.png"))
        axes[row, 1].imshow(Image.open(case_dir / "sample_120_depth.png"))
        box = current["target_bbox_xyxy_px_evaluation_only"]
        axes[row, 0].add_patch(Rectangle((box[0], box[1]), box[2] - box[0], box[3] - box[1],
                                       fill=False, edgecolor="#d62cac", linewidth=1.5))
        frame_candidates = next((r["candidates"] for r in candidates if r["frame"] == current["frame"]), [])
        for candidate in frame_candidates:
            if candidate["color"] != "red":
                continue
            bbox = candidate["bbox_xyxy"]
            axes[row, 0].add_patch(Rectangle((bbox[0], bbox[1]), bbox[2] - bbox[0], bbox[3] - bbox[1],
                                           fill=False, edgecolor="#31a354", linewidth=1.0))
        axes[row, 0].set_title(f"{p['case']}｜RGB｜目标与观测候选", fontsize=12)
        axes[row, 1].set_title(f"{p['case']}｜同帧深度｜t = 6.0 s", fontsize=12)
        for axis in axes[row]:
            axis.set_axis_off()
    fig.suptitle("修订 UAV 航线｜新增扫描排的真实 RGB-D 观测", fontsize=16)
    depth_palette = np.asarray(colourise_depth(np.linspace(0, 100, 256).reshape(1, -1)))[0] / 255.0
    scalar = matplotlib.cm.ScalarMappable(norm=Normalize(0, 100), cmap=ListedColormap(depth_palette))
    colorbar = fig.colorbar(scalar, cax=fig.add_axes([0.942, 0.16, 0.012, 0.65]))
    colorbar.set_label("相机深度（m）", fontsize=10)
    colorbar.ax.tick_params(labelsize=9)
    fig.text(0.045, 0.035,
             "紫框＝目标真值投影，仅用于验收；绿框＝RGB-D 候选模块的红色候选。深度图颜色仅作距离展示。\n"
             "三组均为新增扫描排的 12 秒局部验证；W1/W2 目标位置相同，不代表两个独立感知场景。", fontsize=10)
    contact = OUT / "uav_rgbd_probe_contact_sheet.png"
    fig.savefig(contact, dpi=160)
    fig.savefig(OUT / "uav_rgbd_probe_contact_sheet.pdf")
    plt.close(fig)

    geometry_table = "\n".join(f"| {c['case']} | {c['original_unique_visible_samples']} | {c['revised_unique_visible_samples']} | "
        f"{c['original_longest_geometric_visible_path_m']:.1f} → {c['revised_longest_geometric_visible_path_m']:.1f} |" for c in comparison)
    probe_table = "\n".join(f"| {p['case']} | {p['rendered_visible_frames']}/{p['frames']} | {p['matched_rgb_depth_frames']}/{p['frames']} | "
        f"{p['target_candidate_frames']}/{p['inference_frames']} | {p['target_confirmed_frames']} | {p['yolo_target_box_frames']}/{p['inference_frames']} |" for p in probes)
    report = f"""# W1/W2/F1 UAV 弓字航线修订 v2（2026-10-09）

本阶段完成航线修订、物理净空与 AirSim 航点执行检查，以及新增扫描排的短时同步 RGB-D 验证。没有进行正式 A/B/C 对照。航线配置为 `configs/regions/town10hd_multiscene_uav_routes_v2.yaml`；新 manifest 位于 `docs/figures/multiscene_design_20261009_v2/proposal_manifest.json`。同一场景的三套部署和 A/B/C 共用同一 UAV 航线。

## 修订内容

- 西部：增加 y=90 m 扫描排；相邻排交替飞行，10 个航点、9 段、4 次折返，无重复扫描段。高度 60 m；总长 384 → 480 m；3 m/s 名义完整航时 160 s。
- 整图：在 y=5 与 y=65 m 之间增加 y=20 m 扫描排，并调整后续方向以保持连续弓字形。10 个航点、9 段、4 次折返。高度 80 m；总长 907 → 1120 m；名义完整航时 373.33 s。
- UGV 起点、巡检终点、目标位置、北部/东部 UAV 航线、相机与感知算法均未改变。当前配置只提供新 UAV 航线；未来正式回合须按新完整航时设置时长。

## 原可见性统计勘误

此前“西部 1/2 仅 1 个点、整图 1 仅 2 个点”的数字和“不可见性不足”的判定作废。目标生成后仍在落地，射线终点却固定在初始车顶高度，部分射线没有触及落稳后的车体；相邻航段端点还重复计数。现让目标落稳后固定，使用相机实际 -1.5 m 挂载位置、去除重复采样点，并对原航线和修订航线按同一方法重新检查。原航线本身也具有可见机会，修订提升的是街道覆盖和有效观测跨度。

目标真值只用于此次离线场景勘测和评估。RGB-D 候选推理仍只接收图像、深度和相机标定/位姿。

| 部署 | 原航线可见空间点（更正后） | 修订后可见空间点 | 最长连续几何可见航段（m） |
|---|---:|---:|---:|
{geometry_table}

这里的“点”按不超过 4 m 间距进行空间抽样，不是视频帧、检测召回率或任务成功率。连续跨度也是几何射线统计，不能直接等同于模型持续检测时长。其他候选的可见点数未减少；F3 最长连续跨度略有变化，仍保有多次观测机会。

## 物理与传感器验证

两条完整修订航线的中心及 ±1.2 m 轮廓射线未发现碰撞；AirSim 的非忽略碰撞位置命令扫过完整航线，无新碰撞和航点拒绝，最大位姿误差 {max(s['maximum_pose_error_m'] for s in sweep):.2f} m。本项验证现有位置命令控制方式，不是多旋翼动力学或完整任务闭环验收。

新增扫描排各运行 12 s、3 m/s，固定步长 0.05 s，共 240 帧；候选推理每 0.5 s 一次，共 24 次。RGB-D 同帧率 100%，时间戳误差 0，三组 UAV 碰撞帧为 0，短段最大位姿误差 {max(p['maximum_uav_pose_error_m'] for p in probes):.3f} m。目标可见性使用目标投影区域中的红色像素和车顶深度一致性辅助核验，并抽查真实图像。

| 部署 | 图像/深度可见帧 | 同帧 RGB-D | 目标候选推理帧 | 时序确认帧 | 纯 YOLO 目标框命中帧 |
|---|---:|---:|---:|---:|---:|
{probe_table}

候选模块沿用轻量 YOLO、HSV 红色区域提议、深度几何筛选和 2/5 时序确认。最后一列是纯 YOLO 框与目标投影 IoU≥0.3 的单独统计；不能把颜色/几何产生的候选称作 YOLO 成功检测。F1 的纯 YOLO 仍漏检，候选产生也有间断；本阶段确认航线提供了清晰观测机会，完整感知闭环仍须另行验收。W1/W2 的目标与 UAV 路线相同，这两个回放只是相同 UAV 观测条件下的重复检查。

一次探测会话曾在创建相机后切换同步模式时触发 UE4 GameThread 空指针访问崩溃，发生于 `world.apply_settings`。已改为沿用主项目的“先同步、后创建传感器”流程，保持模式到会话末尾，先销毁相机再恢复异步；重试完成了上述三组。原崩溃的因果内部堆栈无符号，不能进一步归因于某个 UE4 函数。

## 输出

- 航线前后图：[西部](../figures/multiscene_design_20261009_v2/west_route_revision.png)、[整图](../figures/multiscene_design_20261009_v2/whole_route_revision.png)。
- [机器可读验收总表]({OUT.as_posix()}/route_revision_summary.json)。
- [修订路线几何检查]({OUT.as_posix()}/preflight.json)、[原路线更正检查]({OUT.as_posix()}/corrected_original/preflight.json)、[AirSim 完整航线扫查]({OUT.as_posix()}/uav_route_sweep.json)。
- [RGB-D 观测联系图]({contact.as_posix()})。
- 回放：[W1]({OUT.as_posix()}/rgbd_probe/W1/uav_rgbd_visibility_replay.mp4)、[W2]({OUT.as_posix()}/rgbd_probe/W2/uav_rgbd_visibility_replay.mp4)、[F1]({OUT.as_posix()}/rgbd_probe/F1/uav_rgbd_visibility_replay.mp4)。紫框是验收真值框，绿/黄框为最近一次推理的候选；画面注明该次推理时间。
- 各组 `rgbd_probe/<组名>/summary.json`、`frame_checks.json`、`candidate_checks.json` 保存同步、位姿、可见性和候选来源记录；每组仅保存 3 对代表性 RGB/深度图，不保存完整传感器数据集。

结论：航线修订物理检查与新增扫描排 RGB-D 短段验收通过，配置和地图已更新。完整多场景感知、UGV 安全闭环和 36 回合 A/B/C 尚未运行。
"""
    REPORT.write_text(report, encoding="utf-8")
    print("Physical route revision:", result["route_revision_physical_pass"])
    print("Short RGB-D probes:", result["short_rgbd_probe_pass"])
    print(REPORT)
    print(contact)


if __name__ == "__main__":
    main()
