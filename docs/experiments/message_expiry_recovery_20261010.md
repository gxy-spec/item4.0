# 目标消息过期恢复验收（2026-10-10）

## 目的与边界

验证目标状态消息超过 1 s TTL 后，旧目标路线会立即撤销，UGV 回到预设巡检路线；恢复期间 RGB-D 避障仍是最高优先级。该短回合只验收消息过期恢复，不评价双端融合效果或物理信道性能。场景为 Town10HD Zone A，30 s；配置停止发送目标消息以制造连续无新消息窗口。

## 实现

- 非 Oracle 且启用目标消息导航时，只允许 `expiry_policy: return_to_patrol`；旧的 `hold_after_grace` 配置会在启动前拒绝，避免再次进入长时间宽限等待。
- TTL 到期后清除目标路线和消息引用，重新接入预设巡检路径；恢复后的每个控制 tick 仍调用 RGB-D 安全控制器。
- 记录 `expired_patrol_only` 状态，并在验收中分别检查路线撤销、恢复巡检、恢复段位移和 RGB-D 状态覆盖。
- 历史 36 组实验配置及其结果未改写；新的正式配置生成器改为输出立即恢复策略。

## 验收结果

最终回合状态为 PASS，所有 33 项验收检查通过。

| 项目 | 结果 | 判定 |
|---|---:|---|
| 目标消息 TTL | 1.0 s | 固定 |
| UAV 发出的目标消息 | 8 条 | 有效消息输入 |
| 目标路线规划 / 过期后回巡检 | 2 / 2 次 | 每次都撤销旧路线 |
| 最后一次过期后的巡检控制 tick | 468 | 超过 20 tick 门槛 |
| 最后一次过期后的 UGV 巡检位移 | 16.60 m | 超过 1.0 m 门槛 |
| 恢复段有运动的 tick | 194 | 车辆确实继续走动 |
| 恢复段 RGB-D 状态覆盖 | 468 tick | 持续受本地安全模块保护 |
| UGV RGB-D 配对帧 | 299/300（99.7%） | 通过 |
| UGV 碰撞 | 0 | 通过 |
| 过期目标控制命令 | 0 | 通过 |

全回合另记录 240 个深度 STOP tick、34 个 CAUTION tick、21 个 SENSOR_STALE tick。它们属于 RGB-D 避障/传感器保护状态，不是目标消息过期后的等待策略；UGV 在恢复巡检后仍能行驶，遇到传感器判断的近障碍时仍会按安全规则减速或停车。

## 验证文件

- 完整运行目录：`E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/message_expiry_recovery/CI_E7_MESSAGE_EXPIRY_RECOVERY_T10_ZA_S1001_SMOKE_20261010_20261010T120101Z`
- 验收总表：`validation_report.json`
- 目标路线规划、过期和回巡检事件：`planning/target_route_events.jsonl`
- 每个控制 tick 的目标状态、车速和 RGB-D 安全状态：`safety/safety_state.csv`
- UGV / UAV 轨迹：`trajectories/ugv_trajectory.csv`、`trajectories/uav_trajectory.csv`
- 消息序列与收发记录：`communication/messages.jsonl`、`communication/communication_events.jsonl`
- RGB-D 避障判定：`safety/ugv_obstacle_perception.csv`、`safety/obstacle_safety_audit.json`
- 全套自动化测试：`E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/message_expiry_recovery/verification/pytest_all.xml`（75 passed）

## 解释限制

该验收回合有意关闭跨设备一致性门控，只使用 UAV 候选触发目标路线，以单独验证“消息过期—路线撤销—回巡检”状态链；UGV 本地候选数为 0。因此它不能用来声称 UAV/UGV 融合或候选选择效果提升。

输出根目录还保留了前序诊断尝试，没有混入本次 PASS 结论：初次 UAV 位姿预检误差为 1.102 m，超过 1.000 m 的启动容差；随后一次因测试配置缺少交通运动阈值而提前结束；再一次启用了双端一致性门控，但只有 UAV 端产生候选，因而没有触发路线；之后一次已完成路线恢复，但旧状态标签导致恢复验收被误判失败。分别修正容差、补全配置、将 smoke 限定为 UAV 消息 TTL 链路、修正恢复状态标记后，最终回合通过。正式实验的候选融合配置和安全规则未被放宽或绕过。
