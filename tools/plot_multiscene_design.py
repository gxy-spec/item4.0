"""Draw review-only Town10HD scenario proposals from the saved CARLA road survey.

These figures are NOT validated simulator routes.  The saved topology is used
as an undirected geometric graph so proposals stay on known lane links; CARLA
route legality, actor placement, occlusion and UAV clearance remain to survey.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import networkx as nx
import yaml

from probe_town10hd_route_pairs import build as build_directed_road_graph


PROJECT = Path("E:/Research/CityInspection_GOC")
SURVEY = Path("E:/CarlaAirData/CityInspection_GOC/e0_platform/scene_definition_v1")
OUTPUT = PROJECT / "docs" / "figures" / "multiscene_design_20261009_v2"
ROUTE_CONFIG = PROJECT / "configs/regions/town10hd_multiscene_uav_routes_v2.yaml"
OLD_ZONE = (-116.6644, -71.0668, 33.3356, 78.9332)

PROPOSALS = {
    "north": {
        "title": "N｜北部上层街区",
        "bounds": (-85, 42, 78, 147),
        "ugv_start_spawn": 16,
        "ugv_patrol_goal_spawn": 26,
        "target_spawn": 122,
        "uav_xyz": [[-78, 86, 60], [34, 86, 60], [34, 111, 60], [-78, 111, 60], [-78, 136, 60], [34, 136, 60]],
        "hypothesis": "北部岔口：预设路线从顶层道路转向南，目标在西侧上层道路",
    },
    "east": {
        "title": "E｜东部纵向街区",
        "bounds": (20, 116, -31, 99),
        "ugv_start_spawn": 108,
        "ugv_patrol_goal_spawn": 41,
        "target_spawn": 36,
        "uav_xyz": [[22, 66, 60], [106, 66, 60], [106, 36, 60], [22, 36, 60], [22, 6, 60], [106, 6, 60], [106, -24, 60], [22, -24, 60]],
        "hypothesis": "东部岔口：预设路线转向西侧下层道路，目标在北侧支路",
    },
    "west": {
        "title": "W｜西部外围街道",
        "bounds": (-120, -38, -36, 95),
        "ugv_start_spawn": 55,
        "ugv_patrol_goal_spawn": 93,
        "target_spawn": 84,
        "uav_xyz": [[-116, 66, 60], [-44, 66, 60], [-44, 34, 60], [-116, 34, 60], [-116, 2, 60], [-44, 2, 60], [-44, -30, 60], [-116, -30, 60]],
        "hypothesis": "西部联络岔口：预设路线转向西，目标位于北侧道路",
    },
    "whole": {
        "title": "F｜Town10HD 整图搜索组",
        "bounds": (-122, 116, -77, 148),
        "ugv_start_spawn": 37,
        "ugv_patrol_goal_spawn": 69,
        "target_spawn": 112,
        "uav_xyz": [[-20, -55, 80], [-108, -55, 80], [-108, 5, 80], [105, 5, 80], [105, 65, 80], [-108, 65, 80], [-108, 125, 80], [105, 125, 80]],
        "hypothesis": "全图组：UGV 预设沿东侧纵向道路巡检，目标位于西部横向道路；独立统计长航时与性能",
    },
}

# Three review candidates per scenario.  These are not finalized random seeds:
# positions are deliberately distinct, while the within-scenario UAV coverage
# template is held constant so the paired A/B/C comparison remains interpretable.
SEED_VARIANTS = {
    "north": [(16, 26, 122), (86, 26, 122), (16, 122, 26)],
    "east": [(108, 41, 36), (92, 69, 130), (37, 148, 46)],
    "west": [(55, 93, 84), (105, 93, 84), (153, 119, 9)],
    "whole": [(37, 69, 112), (108, 152, 5), (37, 148, 9)],
}


def setup_font() -> None:
    font_path = Path("C:/Windows/Fonts/msyh.ttc")
    if font_path.exists():
        fm.fontManager.addfont(str(font_path))
        plt.rcParams["font.family"] = fm.FontProperties(fname=str(font_path)).get_name()
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["pdf.fonttype"] = 42


def xy(point):
    return tuple(round(float(x), 3) for x in point[:2])


def length(points):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))


def point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    denom = dx * dx + dy * dy
    factor = 0 if denom == 0 else max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / denom))
    return math.dist(point, (start[0] + factor * dx, start[1] + factor * dy))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--route-config", type=Path, default=ROUTE_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT)
    args = parser.parse_args()
    output_dir = args.output_dir.resolve()
    route_config = yaml.safe_load(args.route_config.read_text(encoding="utf-8"))
    original_proposals = copy.deepcopy(PROPOSALS)
    for key, override in route_config["scenarios"].items():
        PROPOSALS[key]["uav_xyz"] = override["uav_waypoints_xyz"]
    setup_font()
    output_dir.mkdir(parents=True, exist_ok=True)
    topology = json.loads((SURVEY / "town10hd_topology.json").read_text(encoding="utf-8"))["segments"]
    with (SURVEY / "town10hd_spawn_points.csv").open(encoding="utf-8-sig", newline="") as source:
        spawns = {int(row["spawn_point_id"]): row for row in csv.DictReader(source)}
    graph, spawn_xy = build_directed_road_graph()

    def position(spawn_id):
        row = spawns[spawn_id]
        return (float(row["x"]), float(row["y"]))

    def road_path(start_id, goal_id):
        nodes = nx.shortest_path(graph, ("spawn", start_id), ("spawn", goal_id), weight="weight")
        return [spawn_xy[node] if node in spawn_xy else node for node in nodes]

    manifest = {
        "status": "ROUTE_REVISION_V2_VALIDATION_RECORDED_SEPARATELY",
        "route_config": str(args.route_config.resolve()),
        "revision": route_config["revision"],
        "road_source": str(SURVEY / "town10hd_topology.json"),
        "note": "Gray links are CARLA lane topology, not buildings; all routes are geometric suggestions, not validated driving or flight paths.",
        "scenarios": {},
    }

    def base_map(ax, bounds, *, old_zone=True):
        for a, b in topology:
            ax.plot((a[0], b[0]), (a[1], b[1]), color="#bdc5cc", lw=1.4, zorder=1)
        if old_zone:
            x0, y0, x1, y1 = OLD_ZONE
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                                   fill=False, edgecolor="#9e7e7e", linewidth=1.1,
                                   linestyle=(0, (4, 3)), zorder=2))
        x0, x1, y0, y1 = bounds
        ax.set(xlim=(x0, x1), ylim=(y0, y1), xlabel="CARLA 世界 X (m)", ylabel="CARLA 世界 Y (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(color="#edf0f3", linewidth=0.7)
        ax.set_axisbelow(True)

    for key, proposal in PROPOSALS.items():
        start_id = proposal["ugv_start_spawn"]
        goal_id = proposal["ugv_patrol_goal_spawn"]
        target_id = proposal["target_spawn"]
        patrol = road_path(start_id, goal_id)
        toward_target = road_path(start_id, target_id)
        prefix = 0
        for a, b in zip(patrol, toward_target):
            if a != b:
                break
            prefix += 1
        shared_m = length(patrol[:prefix])
        target = position(target_id)
        target_to_patrol = min(point_segment_distance(target, a, b) for a, b in zip(patrol, patrol[1:]))
        uav = proposal["uav_xyz"]
        fig, ax = plt.subplots(figsize=(11.2, 8.5))
        fig.subplots_adjust(left=0.10, right=0.98, top=0.88, bottom=0.22)
        base_map(ax, proposal["bounds"])
        ax.plot(*zip(*patrol), color="#155a88", lw=3.2, zorder=4, label="UGV 预设巡检路线（候选）")
        ax.plot(*zip(*toward_target), color="#328566", lw=2.1, linestyle=(0, (3, 2)),
                zorder=3, label="目标分支参考（非已执行轨迹）")
        ax.plot([p[0] for p in uav], [p[1] for p in uav], color="#8e43ad", lw=2.5,
                linestyle=(0, (7, 2)), zorder=3, label="UAV 弓字形巡检航线（候选）")
        start = position(start_id)
        goal = position(goal_id)
        ax.scatter(*start, s=130, c="#155a88", edgecolors="white", linewidths=1.5, zorder=7, marker="o")
        ax.scatter(*goal, s=115, c="#155a88", edgecolors="white", linewidths=1.2, zorder=7, marker="s")
        ax.scatter(*target, s=300, c="#b8333b", edgecolors="white", linewidths=1.3, zorder=8, marker="*")
        ax.scatter(uav[0][0], uav[0][1], s=130, c="#8e43ad", edgecolors="white", linewidths=1.2, zorder=7, marker="D")
        x0, x1, y0, y1 = proposal["bounds"]
        for point, label, dx, dy in [
            (start, f"UGV 起点 #{start_id}", 2, 3),
            (goal, f"巡检终点 #{goal_id}", 2, 3),
            (target, f"目标候选 #{target_id}", 2, -7),
            (uav[0], "UAV 起点", 2, 3),
        ]:
            right_edge = point[0] > x1 - 17
            top_edge = point[1] > y1 - 10
            ax.annotate(label, point[:2], xytext=(point[0] - 2 if right_edge else point[0] + dx,
                                                 point[1] - 5 if top_edge else point[1] + dy),
                        ha="right" if right_edge else "left",
                        color="#182c39", fontsize=10, zorder=10,
                        bbox={"boxstyle": "round,pad=0.18", "facecolor": "white", "alpha": 0.78, "edgecolor": "none"})
        ax.set_title(proposal["title"] + "｜正式仿真前的位置与航线审阅稿", fontsize=16, pad=13)
        fig.legend(loc="lower center", bbox_to_anchor=(0.5, 0.075), ncol=2, frameon=False, fontsize=10)
        fig.text(0.06, 0.025,
                 "灰线＝CARLA 车道拓扑（不是建筑或障碍物）；浅红虚框＝旧 Zone A 范围。道路合法性、遮挡和 UAV 净空尚待现场勘测。",
                 fontsize=9.5, color="#4a5560")
        fig.savefig(output_dir / f"{key}_proposal.png", dpi=190, bbox_inches="tight")
        fig.savefig(output_dir / f"{key}_proposal.pdf", bbox_inches="tight")
        plt.close(fig)
        manifest["scenarios"][key] = {
            "title": proposal["title"],
            "bounds_xyxy": proposal["bounds"],
            "ugv_start_spawn_id": start_id,
            "ugv_start_xy": start,
            "ugv_patrol_goal_spawn_id": goal_id,
            "ugv_patrol_goal_xy": goal,
            "target_candidate_spawn_id": target_id,
            "target_candidate_xy": target,
            "ugv_patrol_candidate_xy": patrol,
            "target_branch_candidate_xy": toward_target,
            "uav_lawnmower_waypoints_xyz": uav,
            "ugv_patrol_length_m": round(length(patrol), 1),
            "uav_lawnmower_length_m": round(length(uav), 1),
            "uav_nominal_flight_seconds": round(length(uav) / route_config["reference_speed_mps"], 2),
            "uav_route_revision": route_config["revision"] if key in route_config["scenarios"] else 1,
            "shared_ugv_prefix_m": round(shared_m, 1),
            "target_to_patrol_m": round(target_to_patrol, 1),
            "route_geometry_status": "directed-topology proposal; CARLA route planner not checked",
            "hypothesis": proposal["hypothesis"],
        }

        fig, axes = plt.subplots(1, 3, figsize=(17.5, 6.8))
        fig.subplots_adjust(left=0.045, right=0.99, top=0.87, bottom=0.18, wspace=0.10)
        variants = []
        for ordinal, (start_seed, goal_seed, target_seed) in enumerate(SEED_VARIANTS[key], start=1):
            axis = axes[ordinal - 1]
            base_map(axis, proposal["bounds"], old_zone=False)
            a_route = road_path(start_seed, goal_seed)
            target_route = road_path(start_seed, target_seed)
            axis.plot(*zip(*a_route), color="#155a88", lw=2.6, zorder=4)
            axis.plot(*zip(*target_route), color="#328566", lw=1.7, linestyle=(0, (3, 2)), zorder=3)
            axis.plot([p[0] for p in uav], [p[1] for p in uav], color="#8e43ad", lw=1.6,
                      linestyle=(0, (6, 2)), zorder=2)
            initial_xy, goal_xy, target_xy = (position(start_seed), position(goal_seed), position(target_seed))
            axis.scatter(*initial_xy, marker="o", s=110, color="#155a88", edgecolors="white", zorder=6)
            axis.scatter(*goal_xy, marker="s", s=100, color="#155a88", edgecolors="white", zorder=6)
            axis.scatter(*target_xy, marker="*", s=210, color="#b8333b", edgecolors="white", zorder=7)
            axis.scatter(uav[0][0], uav[0][1], marker="D", s=95, color="#8e43ad", edgecolors="white", zorder=6)
            axis.set_title(f"候选实例 {ordinal}｜UGV #{start_seed} → 巡检 #{goal_seed}；目标 #{target_seed}", fontsize=11)
            axis.tick_params(labelsize=8)
            if ordinal != 1:
                axis.set_ylabel("")
            if ordinal == 1:
                axis.set_xlabel("CARLA X (m)")
            else:
                axis.set_xlabel("CARLA X (m)")
            target_distance = min(point_segment_distance(target_xy, a, b) for a, b in zip(a_route, a_route[1:]))
            variants.append({"candidate": ordinal, "ugv_start_spawn_id": start_seed,
                             "ugv_patrol_goal_spawn_id": goal_seed, "target_candidate_spawn_id": target_seed,
                             "ugv_patrol_candidate_xy": a_route, "target_branch_candidate_xy": target_route,
                             "target_to_patrol_m": round(target_distance, 1)})
        fig.suptitle(proposal["title"] + "｜场景内三组位置候选（非已运行种子）", fontsize=15, y=0.96)
        fig.legend(handles=[
            Line2D([0], [0], color="#155a88", lw=2.6, label="UGV 预设巡检"),
            Line2D([0], [0], color="#328566", lw=1.7, ls="--", label="目标分支参考"),
            Line2D([0], [0], color="#8e43ad", lw=1.6, ls="--", label="UAV 弓字巡检（本场景保持一致）"),
            Line2D([0], [0], marker="*", color="none", markerfacecolor="#b8333b", markersize=11, label="目标候选"),
        ], loc="lower center", bbox_to_anchor=(0.5, 0.07), ncol=4, frameon=False, fontsize=9)
        fig.text(0.06, 0.02, "候选位置来自实际 CARLA 生成点；道路路线按有向拓扑推导，尚未通过 CARLA 路由、遮挡和飞行净空验收。",
                 fontsize=9, color="#4a5560")
        fig.savefig(output_dir / f"{key}_three_candidates.png", dpi=180, bbox_inches="tight")
        fig.savefig(output_dir / f"{key}_three_candidates.pdf", bbox_inches="tight")
        plt.close(fig)
        manifest["scenarios"][key]["review_candidates"] = variants

        if key in route_config["scenarios"]:
            fig, axes = plt.subplots(1, 2, figsize=(12.5, 8.5))
            fig.subplots_adjust(left=0.08, right=0.98, top=0.88, bottom=0.17, wspace=0.2)
            prefix = "W" if key == "west" else "F"
            for axis, flight, revision_name in zip(axes,
                    (original_proposals[key]["uav_xyz"], uav), ("原航线", "修订航线 v2")):
                base_map(axis, proposal["bounds"], old_zone=False)
                axis.plot([p[0] for p in flight], [p[1] for p in flight],
                          color="#8e43ad", lw=2.3, zorder=4)
                for index, (a, b) in enumerate(zip(flight, flight[1:]), start=1):
                    axis.annotate("", xy=(a[0]*0.35+b[0]*0.65, a[1]*0.35+b[1]*0.65),
                                  xytext=(a[0]*0.60+b[0]*0.40, a[1]*0.60+b[1]*0.40),
                                  arrowprops={"arrowstyle": "->", "color": "#8e43ad", "lw": 1.5})
                axis.scatter(flight[0][0], flight[0][1], marker="D", s=90,
                             c="#8e43ad", edgecolors="white", zorder=7)
                targets = {}
                for ordinal, (_, _, target_seed) in enumerate(SEED_VARIANTS[key], start=1):
                    targets.setdefault(target_seed, []).append(f"{prefix}{ordinal}")
                for target_seed, names in targets.items():
                    tx, ty = position(target_seed)
                    axis.scatter(tx, ty, marker="*", s=210, c="#b8333b",
                                 edgecolors="white", zorder=7)
                    axis.annotate("/".join(names), (tx, ty), xytext=(7, -15),
                                  textcoords="offset points", fontsize=10,
                                  bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"})
                axis.set_title(f"{revision_name}｜{length(flight):.0f} m / {flight[0][2]:.0f} m 高度", fontsize=12)
            fig.suptitle(proposal["title"] + "｜弓字航线覆盖修订", fontsize=16)
            fig.legend(handles=[Line2D([0], [0], color="#8e43ad", lw=2.3, label="UAV 航线与飞行方向"),
                                Line2D([0], [0], marker="D", color="none", markerfacecolor="#8e43ad", markersize=8, label="UAV 起点"),
                                Line2D([0], [0], marker="*", color="none", markerfacecolor="#b8333b", markersize=11, label="目标部署")],
                       loc="lower center", bbox_to_anchor=(0.5, 0.07), ncol=3, frameon=False)
            fig.text(0.06, 0.025, "灰线为车道拓扑；紫线为规划航线。相同场景各部署与 A/B/C 共用同一修订航线。物理检查见 v2 验收报告。", fontsize=9)
            fig.savefig(output_dir / f"{key}_route_revision.png", dpi=180, bbox_inches="tight")
            fig.savefig(output_dir / f"{key}_route_revision.pdf", bbox_inches="tight")
            plt.close(fig)

    fig, ax = plt.subplots(figsize=(12, 10))
    fig.subplots_adjust(left=0.10, right=0.98, top=0.90, bottom=0.13)
    base_map(ax, (-124, 118, -79, 151), old_zone=True)
    colors = {"north": "#38735a", "east": "#b16b16", "west": "#425a9e"}
    for key in ("north", "east", "west"):
        x0, x1, y0, y1 = PROPOSALS[key]["bounds"]
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               fill=False, lw=2.2, edgecolor=colors[key], zorder=5))
        ax.text(x0 + 2, y1 - 6, PROPOSALS[key]["title"], color=colors[key], fontsize=11,
                bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"}, zorder=6)
    ax.set_title("Town10HD 道路拓扑｜候选街区与整图组范围", fontsize=17, pad=13)
    fig.text(0.06, 0.035, "F 整图组使用全部道路范围；浅红虚框为此前 Zone A。灰线仅代表车道拓扑，不表示建筑或遮挡物。",
             fontsize=10, color="#4a5560")
    fig.savefig(output_dir / "town10hd_regions_overview.png", dpi=190, bbox_inches="tight")
    fig.savefig(output_dir / "town10hd_regions_overview.pdf", bbox_inches="tight")
    plt.close(fig)
    (output_dir / "proposal_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for key, item in manifest["scenarios"].items():
        print(key, "UGV", item["ugv_patrol_length_m"], "UAV", item["uav_lawnmower_length_m"],
              "shared", item["shared_ugv_prefix_m"], "target-to-patrol", item["target_to_patrol_m"],
              "geometry-only")
    print(output_dir)


if __name__ == "__main__":
    main()
