# S1 感知驱动闭环验收结果（2026-09-23）

## 结论

9 月 25 日计划中的 `S1：感知驱动闭环` 已在当前实验配置上跑通。正式组与无消息负对照组均为 `PASS`。

- 正式组：UAV 仅依据自身同步 RGB 与深度产生候选，经时间一致性门控后发送一条 `oracle=false` 的目标级语义消息；UGV 收到消息后才规划和运动，并使用自身 RGB 与深度做近距离复核，最终安全到达。
- 负对照组：关闭感知消息后，UGV 在 30 秒内位移为 0 m，说明正式组的运动由感知消息触发，不是由隐藏真值或预设时间触发。
- CARLA 目标真值只写入记录并用于仿真结束后的评价；运行期感知、消息内容和 UGV 控制中的目标真值读取次数均为 0。

## 冻结配置

| 项目 | 配置 |
|---|---|
| 地图与区域 | Town10HD / Zone A |
| 指令 | 寻找红色厢式车辆并接近目标 |
| UAV 传感器 | RGB + 深度，800×600，100°，10 Hz |
| UGV 传感器 | RGB + 深度，800×600，100°，10 Hz |
| 仿真步长 | 0.05 s |
| 正式组时长 | 150 s |
| 通信 | 目标级语义消息，固定 1 个仿真步延迟 |
| UAV 候选门控 | 红色像素比例 ≥ 0.70，5 帧窗口至少命中 4 帧 |
| UGV 本地复核 | 红色像素比例 ≥ 0.35，至少连续命中 3 次 |

离线门控标定使用旧候选序列的真值做阈值评价，真值不进入 S1 在线运行。选定的 0.70/4 门控在该标定序列上的候选精度为 100%，可用目标帧覆盖率为 80.39%。

## 正式组结果

结果目录：

`E:\CarlaAirData\CityInspection_GOC\e2_perception_closed_loop\formal\S1_PERCEPTION_T10_ZA_S1001_20260923T023836Z`

| 指标 | 结果 | 验收要求 | 状态 |
|---|---:|---:|---|
| 验收项 | 36/36 | 全部通过 | PASS |
| 四路共同帧 | 1500/1500 | 比例 ≥ 98% | PASS |
| 最大帧号差 | 0 | 0 | PASS |
| UAV 航程 | 720.00 m | ≥ 715 m | PASS |
| 航点/航段/折返 | 10/9/4 | ≥ 10/9/4 | PASS |
| UGV 航程 | 84.96 m | ≥ 60 m | PASS |
| 消息发送/接收 | 1/1 | 1/1 | PASS |
| 消息延迟 | 50.000 ms | 50 ± 1 ms | PASS |
| 消息定位误差 | 0.616 m | ≤ 3 m | PASS |
| 错误目标消息 | 0 | 0 | PASS |
| UGV 本地复核 | 成功 | 必须成功 | PASS |
| UGV 最终目标距离 | 5.53 m | 3–7 m | PASS |
| 到达后保持 | 2.00 s | ≥ 2 s | PASS |
| UAV/UGV 碰撞 | 0/0 | 0/0 | PASS |
| 动态干扰车/行人 | 6/6 | ≥ 4/4 | PASS |
| 最终任务状态 | `COMPLETED` | `COMPLETED` | PASS |

以正式数据记录起点为零点，关键事件为：约 3.55 s 产生并发送候选，3.60 s 收到消息并规划，16.05 s 进入本地复核，17.55 s 完成 UGV 复核，22.90 s 安全到达，144.00 s UAV 完成完整航线并进入 `COMPLETED`。

## 无消息负对照

结果目录：

`E:\CarlaAirData\CityInspection_GOC\e2_perception_closed_loop\negative_control\S1_PERCEPTION_NO_MESSAGE_T10_ZA_S1001_20260923T025042Z`

- 25/25 项验收通过；300/300 组四路传感器帧严格对齐。
- 感知消息发送/接收为 0/0。
- UGV 航程为 0 m，状态保持为 `SEARCHING`。
- 运行期目标真值读取为 0。

## 输出文件

正式组的主要文件包括：

- `validation_report.json`：36 项机器可读验收结论与完整指标。
- `preview/s1_perception_closed_loop_summary.png`：感知、消息、UGV 接近与 UAV 航线进度总览。
- `preview/synchronized_multiview_replay.mp4`：全局地图、UAV RGB/深度和 UGV RGB/深度五视图同步回放。
- `preview/trajectory_map.png`：UAV、UGV、目标和干扰物的完整二维轨迹。
- `perception/uav_candidates.jsonl`：UAV 在线候选与入选记录。
- `perception/ugv_confirmations.jsonl`：UGV 近距离复核记录。
- `communication/messages.jsonl`：已送达的非 Oracle 语义消息。
- `task/task_events.jsonl`：闭环状态转换审计轨迹。
- `task/perception_usage_audit.json`：运行期真值隔离审计。
- `planning/ugv_planned_route.csv`：消息到达后生成的 UGV 路线。
- `ground_truth/evaluation_only/perception_post_run_evaluation.json`：仅在仿真结束后执行的真值评价。

## 研究边界

本阶段已经证明“观测→候选→语义消息→规划→UGV 本地复核→安全到达”的闭环能够工作，但还不是最终感知结论：

- 车辆候选来自通用 COCO 车辆检测器；`van_like_score` 是公开、可审计的类别与框形状启发式分数，不是专门训练的厢式车分类网络。
- 颜色判断是检测框内的 HSV 红色像素比例，不是学习式颜色属性模型。
- 深度定位使用同步深度图反投影，不使用运行期目标真值。
- 50 ms 是配置的单步确定性通信延迟，只用于闭环接口验收，不代表真实无线网络测量。
- 当前结果为单场景、单随机种子通过结果；下一阶段需要多种子统计、专用车辆细粒度分类/再识别以及时延、丢包、带宽消融。

因此，本结果可以作为 `S1 单场景功能验收通过`，不能直接写成最终模型精度或泛化性能结论。
