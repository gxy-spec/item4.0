# 目标记忆规则：三场景小批量验收（2026-10-12）

## 目的与边界

仅验证“单条目标消息 TTL 到期不撤销已确认目标路线；异位候选未经确认不切换目标”。本次运行 N1、E1、W2 三个既有场景的 C 组，共 3 回合；**不是**新的 36 组 A/B/C 统计对照，也不能用作论文最终性能结论。旧版 36 组输出保持原样。

在线决策不读取 CARLA 目标真值。1 秒 TTL 只用于判断消息能否进入状态机。已确认目标按感知坐标保留；10 m 内的新观测视为同一目标。相距更远的候选需在 4 秒内有至少 3 条一致更新、UAV 与 UGV 双源支持、选择分数高于现有目标至少 0.08，且旧目标已 2 秒未被观测，才可切换。UGV 的 RGB-D 避障全程开启。CARLA 真值仅在回合结束后检验目标提交误差与碰撞。

## 结果

| 场景 | 用途 | 状态 | 目标路线规划 | TTL 导致路线撤销 | 错误切换 | 未确认异位候选 | 记忆路线 RGB-D 守护 | 终点距目标：旧版→新版 |
|---|---|---|---:|---:|---:|---:|---:|---:|
| N1 | 稳定观测 | PASS | 1 | 0 | 0 | 1 | 3588/3588 tick | 12.9→12.8 m |
| E1 | 长消息间断 | PASS | 1 | 0 | 0 | 6 | 3788/3788 tick | 51.0→10.9 m |
| W2 | 异位候选干扰 | PASS | 1 | 0 | 0 | 5 | 4188/4188 tick | 10.9→10.9 m |

三处 UAV/UGV 均为零碰撞。目标首次提交位置与离线真值的平面误差分别为 2.92、0.57、2.89 m；未观察到实际目标切换，因而“切换后的正确性”只由自动化合成候选测试覆盖。E1 两次有效选中消息之间最长间隔约 63.5 s，UGV 仍沿已提交目标路线前进；这是**静止目标**条件下的预期行为，不应直接推广到移动目标。

配对轨迹与距离曲线：`E:\CarlaAirData\CityInspection_GOC\persistent_memory_smoke_20261012\comparison\three_scene_memory_comparison.png`。蓝色是本版，灰色是旧版有界过期回退；左列为世界坐标轨迹，右列为 UGV 与目标的平面距离。最明显的改善在 E1：旧版过期后多次回巡检，最终停在约 51 m；新版只规划一次，继续接近到约 10.9 m。N1、W2 两版基本重合，说明这两处没有因修改产生明显路线回归。

## 验证入口

- 批次状态：`E:\CarlaAirData\CityInspection_GOC\persistent_memory_smoke_20261012\batch_progress.json`
- 数值复核：`E:\CarlaAirData\CityInspection_GOC\persistent_memory_smoke_20261012\comparison\verification.json`
- 每回合：`N1/C`、`E1/C`、`W2/C` 下的运行目录，各含 `validation_report.json`、`communication/target_memory_events.jsonl`、`planning/target_route_events.jsonl`、`safety/safety_state.csv`、`preview/trajectory_map.png`。
- 三处 `preview/synchronized_multiview_replay.mp4` 均为五视图**稀疏同步回放**：四路 RGB/深度与地图按同一 CARLA 帧对齐，但为节省空间每约 5 s 存一帧，不能解读为连续 20 Hz 影像。
- 自动化测试：`tests/test_persistent_target.py` 含消息过期保持目标、孤立异位候选拒绝及多源高分候选切换测试；相关安全边界测试共 26 项通过。

## 后续实验边界

保留本次 3 回合作为状态机 smoke，旧版 36 组作为历史最小基线，不立即重跑 36 组。接下来先建立受限信道、再提高目标识别鲁棒性。最终论文基线须冻结同一版感知模型、场景与随机种子，在相同物理信道条件下配对比较通信策略；否则“感知升级”和“通信策略改进”的作用会混淆。静止目标记忆规则还需要在移动目标、目标消失与本地负证据场景中单独验证。
