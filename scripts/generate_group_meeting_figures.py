"""Generate consistent, paper-style figures for the group-meeting report."""
from pathlib import Path
import json
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = Path(r"E:\Research\CityInspection_GOC")
OUT = ROOT / "docs" / "figures" / "group_meeting_20260924_20261002"
OUT.mkdir(parents=True, exist_ok=True)
font_manager.fontManager.addfont(r"C:\Windows\Fonts\NotoSansSC-VF.ttf")
plt.rcParams.update({
    "font.family": "Noto Sans SC", "font.size": 9,
    "axes.titlesize": 11, "axes.labelsize": 9,
    "legend.fontsize": 8, "figure.dpi": 160,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.22, "grid.linewidth": 0.6,
})
BLUE, ORANGE, GREY, GREEN, RED = "#1f5a94", "#c85a2a", "#6f7782", "#238b58", "#b83b3b"

def save(fig, name):
    fig.tight_layout(pad=1.1)
    fig.savefig(OUT / name, dpi=300, bbox_inches="tight")
    plt.close(fig)

# Fig 1: stage progression, separated by validation layer.
stages = ["S1\n感知闭环", "RGB-D\n避障", "UGV\n本地搜索", "候选\n对齐", "E7-A/B/C\n路径影响"]
success = [6/6, 6/6, 1/1, 1/1, 9/9]
fig, ax = plt.subplots(figsize=(7.2, 3.7))
bars = ax.bar(np.arange(len(stages)), success, color=[BLUE, GREEN, ORANGE, GREY, "#6b4c9a"], width=.62)
ax.set_ylim(0, 1.12); ax.set_ylabel("通过率"); ax.set_xticks(range(len(stages)), stages)
ax.set_yticks([0, .25, .5, .75, 1.0], ["0", "25%", "50%", "75%", "100%"])
ax.set_title("9月24日至今各阶段有效验收通过率")
for b, s in zip(bars, ["6/6", "6/6", "1/1", "1/1", "9/9"]): ax.text(b.get_x()+b.get_width()/2, b.get_height()+.035, s, ha="center", va="bottom", fontsize=9)
ax.text(.01, -.24, "注：仅计入最终有效运行；失败尝试保留在审计目录但不计入正式分母。", transform=ax.transAxes, fontsize=8, color="#555")
save(fig, "fig01_stage_acceptance.png")

# Fig 2: RGB-D safety scenarios from latest PASS runs.
scenarios = ["clear", "static\nrecovery", "crossing\nvehicle", "crossing\npedestrian", "occlusion", "depth\nfailure"]
stop = [0, 106, 112, 44, 123, 0]
caution = [0, 36, 52, 42, 72, 0]
stale = [1, 1, 1, 1, 1, 81]
fig, ax = plt.subplots(figsize=(7.2, 3.9))
x=np.arange(len(scenarios)); w=.25
ax.bar(x-w, caution, w, label="CAUTION", color="#d6a43a")
ax.bar(x, stop, w, label="STOP", color=RED)
ax.bar(x+w, stale, w, label="SENSOR_STALE", color=GREY)
ax.set_xticks(x, scenarios); ax.set_ylabel("控制 tick 数（50 ms/tick）"); ax.set_title("UGV RGB-D 避障状态响应")
ax.legend(frameon=False, ncol=3, loc="upper left")
ax.text(.01, -.25, "所有最终有效场景碰撞数均为 0；tick 数是控制器状态持续时间，不是独立危险事件数。", transform=ax.transAxes, fontsize=8, color="#555")
save(fig, "fig02_rgbd_safety_states.png")

# Fig 3: E6 candidate alignment error distribution.
e6 = Path(r"E:\CarlaAirData\CityInspection_GOC\e6_candidate_alignment\20261001\formal\CI_E6_CANDIDATE_ALIGNMENT_T10_ZA_S1001_20261001_20260928T125312Z\validation_report.json")
e6j = json.loads(e6.read_text(encoding="utf-8"))
align = e6j["summary"]["dual_device_candidate_alignment"]
uav_med = align["per_device"]["uav"]["target_error_median"]
ugv_med = align["per_device"]["ugv"]["target_error_median"]
cross_med = align["cross_device_position_error_median"]
cross_p90 = align["cross_device_position_error_p90"]
fig, ax = plt.subplots(figsize=(7.2, 3.8))
labels=["UAV→目标", "UGV→目标", "UAV↔UGV候选"]
med=[uav_med, ugv_med, cross_med]
p90=[align["per_device"]["uav"]["target_error_p90"], align["per_device"]["ugv"]["target_error_p90"], cross_p90]
xx=np.arange(3); ax.bar(xx, med, .48, color=[BLUE, ORANGE, GREY], label="中位误差")
ax.errorbar(xx, med, yerr=np.array(p90)-np.array(med), fmt="none", ecolor="#222", capsize=4, label="P90范围")
ax.axhline(5, ls="--", color=RED, lw=1.1, label="验收上限 5 m")
ax.set_xticks(xx, labels); ax.set_ylabel("XY平面误差（m）"); ax.set_title("E6 双端候选定位与坐标对齐误差")
ax.legend(frameon=False, loc="upper left")
ax.text(.01, -.24, f"共同观测帧比例 {align['joint_target_frame_ratio']:.1%}；跨端误差 P90={cross_p90:.2f} m。", transform=ax.transAxes, fontsize=8, color="#555")
save(fig, "fig03_candidate_alignment_error.png")

# Fig 4: E7 paired ABC final distance and route departure.
csv_path = Path(r"E:\CarlaAirData\CityInspection_GOC\e7_revised_v2\comparison_metrics.csv")
rows=list(csv.DictReader(csv_path.open(encoding="utf-8-sig")))
rows=[r for r in rows if r["comparison_case"]=="off_route"]
policies=[("none", "A 巡检", GREY), ("ugv_local", "B UGV", BLUE), ("dual", "C 双端", ORANGE)]
means=[]; sems=[]; deps=[]
for p,lab,col in policies:
    vals=np.array([float(r["final_target_distance_m"]) for r in rows if r["control_policy"]==p])
    ds=np.array([float(r["mean_departure_from_patrol_route_m"]) for r in rows if r["control_policy"]==p])
    means.append(vals.mean()); sems.append(vals.std(ddof=1)/np.sqrt(len(vals))); deps.append(ds.mean())
fig, (ax1, ax2)=plt.subplots(1,2,figsize=(7.5,3.7))
xx=np.arange(3); ax1.bar(xx, means, yerr=sems, capsize=4, color=[c for _,_,c in policies], alpha=.92)
ax1.set_xticks(xx,[lab for _,lab,_ in policies]); ax1.set_ylabel("末时刻目标距离（m）"); ax1.set_title("E7 目标接近效果\n均值 ± 标准误")
ax2.bar(xx, deps, color=[c for _,_,c in policies], alpha=.92)
ax2.set_xticks(xx,[lab for _,lab,_ in policies]); ax2.set_ylabel("偏离预设路线均值（m）"); ax2.set_title("E7 路线影响强度")
for ax in (ax1,ax2): ax.grid(axis="y"); ax.set_axisbelow(True)
fig.suptitle("E7 路径影响对照（3 个配对随机种子）", y=1.02, fontsize=12)
save(fig, "fig04_e7_abc_effect.png")

# Fig 5: E7 distance trajectories for seed 1001, with exact naming.
runrows={r["control_policy"]:r for r in rows if r["seed"]=="1001"}
fig, ax=plt.subplots(figsize=(7.2,3.7))
for p,lab,col in policies:
    r=runrows[p]
    run=Path(r["run_dir"]); traj=list(csv.DictReader((run/"trajectories"/"ugv_trajectory.csv").open(encoding="utf-8-sig")))
    target=list(csv.DictReader((run/"trajectories"/"target_trajectory.csv").open(encoding="utf-8-sig")))
    tx,ty=float(target[0]["x"]),float(target[0]["y"])
    t=np.array([float(q["timestamp"]) for q in traj]); d=np.array([np.hypot(float(q["x"])-tx,float(q["y"])-ty) for q in traj])
    ax.plot(t,d,label=lab,color=col,lw=1.8)
ax.axhline(10, ls="--", color=RED, lw=1, label="10 m参考线")
ax.set_xlabel("仿真时间（s）"); ax.set_ylabel("UGV 到目标距离（m）"); ax.set_title("E7 同种子距离轨迹（seed 1001）"); ax.legend(frameon=False, ncol=2)
save(fig, "fig05_e7_seed1001_trajectories.png")

print("Generated", len(list(OUT.glob("*.png"))), "figures in", OUT)
