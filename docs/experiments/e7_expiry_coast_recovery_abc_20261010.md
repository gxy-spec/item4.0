# 消息过期有界恢复与 A/B/C 配对实验报告（2026-10-10）

## 目的与保护边界

在不覆盖原最小 baseline 的前提下，为消息过期后的目标跟踪增加“有限续行—新消息重获—有界回归巡检”策略，并在原 12 个配对场景中重跑 A/B/C，共 36 回合。

- 旧版完整 baseline 保留在 `E:\CarlaAirData\CityInspection_GOC\multiscene_abc_formal_20261010_v2`，仍为 36/36 PASS；新实验独立写入 `E:\CarlaAirData\CityInspection_GOC\multiscene_abc_expiry_coast_20261010`。
- A：固定巡检；B：UGV 本地候选；C：UAV 与 UGV 候选协同。每个场景的 A/B/C 使用相同种子、目标、干扰物、路线、传感器和时长。
- 新恢复规则：单条消息 TTL 为 1 s。消息过期后，UGV 不立即急停或跳回路线起点；它沿当前目标路线以 50% 速度有限续行，最长 5 s 或 8 m。续行期间若有新鲜有效目标消息则重获目标；达到上限后，从当前位姿沿预设路线的前向窗口并入巡检。RGB-D 安全避障在目标跟踪、有限续行和回归巡检期间始终启用。
- 本批次没有引入物理带宽竞争、衰落或随机物理层丢包；通信仍使用已有固定时延与 TTL。每 100 个仿真 tick 保存一个四流同步传感器样本，其余轨迹、状态、消息、候选和安全日志完整记录。没有为这次快速验证重新制作 36 段视频。

## 正式验收

**36/36 回合 PASS；缺失验收报告 0；验收失败项 0；UGV/UAV 碰撞均为 0。** 四路传感器共同帧比例最低 99.972%，车辆干扰为每回合 3 辆、行人为每回合 2 名。A/B/C 的 RGB-D 安全避障控制没有关闭或绕过。

独立 30 s TTL 到期 smoke 也通过 33/33 项：新鲜消息重新获得后继续目标路线；无新鲜消息时在 3 s/4 m smoke 上限内有限续行后并入前向巡检路线，过期目标直接控制为 0，恢复巡检仍持续运行 RGB-D 防护。

## 消息过期恢复结果

计数是所有回合日志中的事件计数；单回合允许多次过期、重获或并入，不能将其理解为不同消息的唯一数量。

| 组别 | 目标路线规划 | 有界续行开始 | 续行控制 tick | 新消息重获 | 并入巡检 | 过期后巡检 tick / RGB-D 防护 tick |
|---|---:|---:|---:|---:|---:|---:|
| A 固定巡检 | 0 | 0 | 0 | 0 | 0 | 不适用 |
| B UGV 本地感知 | 14（UGV 14） | 23 | 1,421 | 11 | 12 | 30,874 / 30,874 |
| C UAV+UGV 协同 | 28（UAV 19、UGV 9） | 36 | 2,522 | 12 | 24 | 30,583 / 30,583 |

所有 36 份验收报告中的“过期消息不得控制目标”检查均为 0 条违规。B/C 的过期后巡检 tick 与 RGB-D 防护 tick 相等，说明回归巡检没有因为通信中断而停掉本地安全保护。

## A/B/C 任务结果（描述性统计）

“接近量”定义为初始 UGV–目标距离减去最终距离，正值表示更接近；路线偏离是轨迹采样点到预设巡检路线的平均 2D 距离。统计单位是 12 个配对场景，不是统计显著性结论。

| 组别 | 平均接近量 | 平均最终目标距离 | 平均路线偏离 | 目标路线规划数 |
|---|---:|---:|---:|---:|
| A 固定巡检 | 14.77 m | 101.99 m | 0.34 m | 0 |
| B UGV 本地感知 | 20.09 m | 96.67 m | 2.88 m | 14（UGV 触发） |
| C UAV+UGV 协同 | 21.98 m | 94.78 m | 15.52 m | 28（UAV 19、UGV 9） |

C 相对 B 的平均接近量增加 1.89 m，12 个配对场景中 C 更接近 7 个、更远 4 个、持平 1 个；这是小幅、场景依赖的描述性增益，不能据此宣称 UAV 协同在所有场景都更好。C 的路线偏离明显增大，且 0/12 回合结束时进入目标 10 m 内，说明候选误选、路线质量和任务成功定义仍需继续改善。消息过期恢复通过验收，并不等于目标识别和最终到达任务已经解决。

## 结果与复核入口

- 批次状态与逐回合统计：[batch_progress.json](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_expiry_coast_20261010/batch_progress.json)
- 全部 36 个逐回合验收、日志与预览路径：[all_validation_paths.tsv](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_expiry_coast_20261010/all_validation_paths.tsv)
- 有界续行/重获/回归逐回合计数：[message_expiry_recovery_by_run.csv](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_expiry_coast_20261010/message_expiry_recovery_by_run.csv)
- 有界恢复分组汇总：[message_expiry_recovery_summary.json](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_expiry_coast_20261010/message_expiry_recovery_summary.json)
- 机器可读 A/B/C 汇总与验收结论：[analysis_summary.json](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_expiry_coast_20261010/analysis/analysis_summary.json)
- 目标接近图：[target_approach_by_scene.png](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_expiry_coast_20261010/analysis/target_approach_by_scene.png)
- UGV 预设路线偏离图：[patrol_route_departure_by_scene.png](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_expiry_coast_20261010/analysis/patrol_route_departure_by_scene.png)
- 12 张 UAV/UGV 逐场景轨迹与目标距离图：[paired_trajectories](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_expiry_coast_20261010/analysis/paired_trajectories)
- 可重绘的配对指标：[paired_metrics_36.csv](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_expiry_coast_20261010/analysis/paired_metrics_36.csv)、[group_summary.csv](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_expiry_coast_20261010/analysis/group_summary.csv)、[paired_effects.csv](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_expiry_coast_20261010/analysis/paired_effects.csv)
- 本批次正式配置：[formal_36_run_index.json](E:/Research/CityInspection_GOC/configs/experiments/multiscene_abc_expiry_coast_20261010/formal_36_run_index.json)
- 独立 TTL 到期烟雾验收：[validation_report.json](E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/message_expiry_recovery_coast_v1/CI_E7_MESSAGE_EXPIRY_COAST_RECOVERY_T10_ZA_S1001_SMOKE_20261010_20261010T123106Z/validation_report.json)

各回合目录内还保留 `preview/trajectory_map.png`、`preview/sensor_composite.png`、`preview/synchronization_plot.png`，以及 `communication/communication_events.jsonl`、`planning/target_route_events.jsonl`、`safety/safety_state.csv`。逐回合绝对路径见 TSV 索引。

## 解释边界

该批次验证的是 A/B/C 的配对运行以及 TTL 后的有限续行/重获/前向并回；不是物理信道实验。传感器图像以约 5 s 间隔稀疏留存，因此不能用于判断两帧之间的瞬时动作；若后续需要连续同步回放，应单独挑选代表场景补采，而不应重跑 36 组。全量结果仍是小样本系统级描述统计，C 的增益较小且有反例，应先处理候选质量和路线偏离，再进入物理信道层级比较。
