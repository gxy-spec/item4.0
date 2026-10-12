# 第4天：同种子 A/B/C 三回合（2026-10-08）

## 结论

Town10HD Zone A、seed 1001、80 秒的 A/B/C 三回合均完成，原始动态验收分别为 PASS。跨组专门验收 14/15 项通过，整体严格状态记为 FAIL：B 组最终距目标 10.45 m，未进入事前确定的 10 m 接近范围。该回合不能写成“三组全部达到目标”，也不能据此声称完整 5 m 到达与本地确认已完成。

统一结果目录：`E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/day4_same_seed_abc/S1001`

三组重启 CARLA-Air 独立运行；同一静止红色目标、3 辆非红色干扰车和 2 名行人，实际初始演员类型和位置一致。UGV 与 UAV 均从同一配置初始状态运行；除控制允许的候选来源（A:none、B:ugv_local、C:dual）和相应的验收计数外，场景、相机、候选算法、50 ms 理想链路延迟、1 s 消息 TTL、RGB-D 安全控制均一致。不注入物理信道损失。原始浮点深度不全量保存，仅稀疏保存可回放帧。

## 主要结果

| 指标 | A 固定巡检 | B UGV 本地 | C 双端候选 |
|---|---:|---:|---:|
| 首次正确目标路线 / s | 无 | 36.55 | 0.55（UAV 来源） |
| 正确 / 错误目标路线数 | 0 / 0 | 1 / 0 | 3 / 0 |
| UGV 执行消息路线的控制 tick | 0 | 868 | 1356 |
| 最小/最终目标距离 / m | 34.02 | 10.45 | 9.54 |
| 首次进入 10 m / s | 未进入 | 未进入 | 74.85 |
| UGV 实际行程 / m | 79.16 | 105.51 | 105.14 |
| 两端碰撞数 | 0 | 0 | 0 |

A 在距预设路线最近 31.44 m 的旁侧目标前没有改道，最大巡检路线偏差 0.039 m；终点自动制动的 453 个控制 tick 已标为 `route_complete`。B 在本地有效候选出现后由 UGV 消息规划路线；C 从开始即由 UAV 消息规划路线。所有规划目标在事后均落入目标真值 5 m 匹配范围，实时规划没有读取目标真值。

C 的目标路线比 B 早形成 36.00 s，但最初两组仍沿同一条道路接近，50 m 距离门限均为 47.40 s。到 12 m、11 m 时，C 分别比 B 提前约 0.95 s、1.20 s。这说明本种子的 UAV 信息已改变早期决策，转化成的几何/时间增益仍较小；不能把“早规划 36 秒”等同于“早到 36 秒”。

C 的“双端”实现为对 UAV、UGV 发送的目标状态做时效与分数选择，随后按被选中的位置规划；并未融合两端原始图像或深层特征。

B、C 回合末均停在 `wait_no_safe_overtake_gap`，UGV RGB-D 传感器状态为 `CAUTION`，前方净空约 7.77 m、7.58 m，速度为 0。该安全停车解释了 B 未跨过 10 m 门限；不应为了达标关闭或放松已验收的避障。B 的原始 PASS 只说明动态、通信和路线执行达到其原生验收项，跨组严格接近判据仍为 FAIL。

安全与数据质量：三组 100% 控制时段有 RGB-D 障碍评估，四路共同帧均为 1599/1600（99.9375%），共同帧号跨度 0、UAV/UGV 源时间差 0；过期消息驱动路线命令为 0，两端碰撞均为 0。三组均记录到 205 个行人等待控制 tick，安全模块未因 A/B/C 决策来源变化而关闭。

## 证据文件

- `day4_batch_index.json`：本次三个回合的唯一索引及原始验收路径。
- `day4_comparison_acceptance.json`：15 项跨组检查；明确记录 B 的 10 m 接近失败及各项原因。
- `day4_comparison_metrics.csv`：逐组指标。每组目录中的 `day4_episode_metrics.json` 可直接核查。
- `preview/day4_trajectory_comparison.png/.pdf`：实际轨迹、静态道路、共同起点、巡检路线和事后目标位置。
- `preview/day4_target_distance_time.png/.pdf`：UGV 到目标中心的水平距离与 10 m 接近门限。
- `preview/day4_message_decision_timeline.png/.pdf`：两端候选状态发送时刻与各组目标路线生成时刻。
- `preview/day4_A_five_view.mp4`、`day4_B_five_view.mp4`、`day4_C_five_view.mp4`：各组地图、UAV RGB/深度、UGV RGB/深度同源帧同步回放。
- 各回合的 `validation_report.json`、`communication/messages.jsonl`、`planning/target_route_events.jsonl`、`safety/safety_state.csv`、`trajectories/ugv_trajectory.csv`：原始判定、消息来源、路线来源、安全停车原因和实际位置。

本轮是单一种子的机制比较。它支持“UAV 消息可以提前触发正确路线、C 在本回合达到较近位置”的结论；不支持多种子统计增益、5 m 完整任务成功或物理信道鲁棒性的结论。
