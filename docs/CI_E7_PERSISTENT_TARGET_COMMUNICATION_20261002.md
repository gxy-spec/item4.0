# CI-E7 持续目标信息通信验收（2026-10-02）

## 实验目的与设置

在 Town10HD Zone A、seed 1001、静止红色厢式目标下，验证 UAV 固定巡检航线与 UGV 巡检同时运行时，双端能否从 RGB-D 候选反复生成带序号的目标状态；协同端按任务匹配、深度可靠性、时效性和时间连续性选择当前状态。固定延迟 50 ms、TTL 1 s、不注入信道丢包。完整图像/深度仅稀疏保存。

UAV 沿穿越目标区域的固定折线航线以 3 m/s 飞行；正式组 60 s，路径 118 m。UGV 按原巡检路线以 RGB-D 障碍保护运行，不以 CARLA 目标真值控制。

## 最终验收结果

- Smoke：PASS，20 s，输出目录：`E:/CarlaAirData/CityInspection_GOC/e7_persistent_target_communication/20261002/smoke/CI_E7_PERSISTENT_TARGET_COMMUNICATION_T10_ZA_S1001_SMOKE_20261002_20260929T041126Z`。
- Formal：PASS，最终采用目录：`E:/CarlaAirData/CityInspection_GOC/e7_persistent_target_communication/20261002/formal/CI_E7_PERSISTENT_TARGET_COMMUNICATION_T10_ZA_S1001_20261002_20260929T041236Z`。
- 正式组双端各 120 次推理；UAV 发出 52 条，UGV 发出 109 条；161/161 条完成接收，选择记录 30 次；两端序号严格递增。
- 同步共帧 600/600（100%），四流最大帧差 0；UGV RGB-D 配对率 100%；UGV 行驶 83.12 m，UAV 航迹 118 m、到达 3 个航点；零碰撞。干扰流中观察到 7 辆移动车辆和 6 名移动行人。
- 一条新鲜、双端位置相互印证的状态生成了目标路线；UGV 按消息路线运行 300 个仿真控制帧。正式组最后 5 s 停止新状态上报，TTL 到期后路线撤销并返回巡检；记录到 0 条过期目标跟踪控制命令。此处是计划内协议到期检查，不是随机信道丢包实验。
- 两次早期正式尝试均原样保留为失败目录（`...T033059Z`、`...T040508Z`）；最终修正组为 `...T041236Z`，没有覆盖失败证据。

## 未通过项和边界

本阶段先后发现并修正了三类问题：CARLA 绝对时间与局部计时混用；红色误检导致目标路线跳变；控制后采样把刚过期的消息误计为过期控制。最终正式组采用统一 CARLA 时间、双端位置一致性和连续性门控，并在末段受控停止上报以验收 TTL 到期回退。最终组已实测根据消息路线跟踪、到期撤销并回到巡检。

本轮未注入随机丢包。重复/乱序消息的拒绝由协议单元测试覆盖；正式场景中自然产生的消息序号均递增，因此不能声称仿真中观察到了重复包。

## 主要输出

- `validation_report.json`：本轮原始 PASS/FAIL 检查，保留未通过状态。
- `communication/messages.jsonl`：发送的双端目标状态及序号、候选分数、位置和过期时间。
- `communication/communication_events.jsonl`：发送、接收、选择事件顺序。
- `communication/selected_target_states.jsonl`：被选中的状态记录。
- `communication/communication_audit.json`：消息计数、序号与离线 TTL 回放审计。
- `planning/target_route_events.jsonl`：目标路线生成、过期撤销及回到巡检的来源记录。
- `preview/persistent_communication_timeline.png`：双端发送密度与被选来源/分数。
- `preview/trajectory_map.png`、`preview/synchronization_plot.png`、`preview/candidate_alignment_summary.png`：轨迹、传感器对齐和候选观察结果。

## 复现

配置：`configs/experiments/ci_e7_persistent_target_communication_20261002_formal.yaml`。启动入口：`scripts/Run-PersistentTargetCommunication.ps1`。TTL 与序号逻辑：`src/communication/persistent_target.py`；回放分析：`scripts/analyze_persistent_target_communication.py`。

## ABC 路径影响对照批次（2026-09-29 完成）

此批次与上面的单次 E7 通信机制正式验收分开统计。固定 Town10HD Zone A、静止目标、UAV 固定巡检路线、UGV RGB-D 安全避障、50 ms 通信延迟和 1 s 消息 TTL；使用配对随机种子 1001、2001、3001。A 仅按巡检路线控制（不采用候选消息），B 采用 UGV 本地候选，C 采用 UAV+UGV 候选。每组均为 70 s。

- 9/9 组通过验收；每组四流共同帧 700/700，逐传感器回调均为 700 帧、队列丢帧 0；UGV RGB-D 安全避障始终开启，在线障碍物 CARLA 真值读取为 0，UGV 碰撞为 0。
- 按组平均最终目标距离：A 77.30 m，B 13.41 m，C 22.07 m。B、C 相比 A 均明显接近目标；但 C 未稳定优于 B：seed 1001 的 C 组最终距离为 47.66 m，期间进入较长安全保持。因此当前结果支持“候选信息能影响路线”，不能据此声称“双端协同稳定优于车端单独感知”。
- 同步故障修复：传感器回调现在逐流计数并记录队列丢帧；仿真每步等待所需的四流精确帧连接，不再依赖固定短暂等待。失败时把逐流状态写入 `synchronization/sensor_callback_audit.json`，便于定位漏帧来源。
- 失败尝试目录保留作审计，不纳入本批统计；统计脚本只读取 `batch_run_index.json` 中列出的本批九组，避免历史失败/旧批次混入。

本批输出根目录：`E:/CarlaAirData/CityInspection_GOC/e7_persistent_target_communication/20261002/comparison`。九组清单、逐组指标、组汇总和图分别为 `batch_run_index.json`、`comparison_metrics.csv`、`comparison_group_summary.csv` 和 `e7_comparison_overview.png`。
