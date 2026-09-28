# 9/28 UGV RGB-D 避障实验记录

## 目的与边界

本阶段验证 UGV 是否能仅凭车载 RGB-D 观测识别前方占用区域，并据此限速/停车；同时检查无障碍通行和深度传感器失效时的安全策略。障碍物真值只用于实验后核验，避障在线决策不得读取 CARLA 障碍物位置。该阶段是短时功能验收，不是最终算法性能评估。

> 日期说明：这是实验安排中的 9/28 项目；实际运行时间为本机 2026-09-27 14:45 左右（Asia/Shanghai），因此目录中的 `20260927T...Z` 是 UTC 时间戳，并非实验组号或随机种子。

## 实验设置

- 平台：CARLA-Air，Town10HD Zone A；固定步长 0.05 s，RGB-D 传感器 10 Hz，图像 800×600。
- 参与对象：UAV、UGV、静止目标；按每组测试需要添加一个固定障碍，或注入深度数据失效；未加入随机交通干扰物。
- UGV 以约 2 m/s 巡检；避障以 UGV 深度图反投影到车体坐标系，按前方走廊、障碍高度、距离和 TTC 判定 CLEAR / CAUTION / STOP。传感器过期时进入 SENSOR_STALE 并采取停车保护。
- 为节省空间，仅持久化紧凑传感器样本；运行期间仍逐帧计算安全状态。UGV RGB 与深度的本地帧配对单独验收，不要求等待 UAV 全局多视图帧 join。

## 结果

| 用例 | 结果 | 关键结果 |
|---|---|---|
| 固定障碍 | PASS | 400 个控制 tick 中 STOP 207 tick、CAUTION 36 tick；最近检测距离 3.89 m；UGV 行驶 24.04 m 后安全停下；碰撞 0；在线障碍真值读取 0。 |
| 无障碍通行 | PASS | UGV 行驶 25.65 m；STOP/CAUTION 均为 0；走廊未检出占用点；碰撞 0。 |
| 深度失效 | PASS | 在配置的失效区间内记录 61 个 SENSOR_STALE tick，并通过失效停车保护验收；UGV 行驶 15.77 m；碰撞 0。 |

三组 UGV RGB-D 本地帧配对率均为 100%，深度有限值比例均为 100%，在线障碍 actor 真值读取均为 0。车辆静止目标保持不动；UAV 轨迹与碰撞检查也均通过。首轮畅通组曾因将“图像唯一帧比例”错误地设为通用 0.8 门槛而报 FAIL；检查确认 UGV 仍正常行驶且避障状态正确后，改为按本组实际特征验收并重跑，最终验收结果为 PASS。首轮失败输出保留，没有删除。

## 输出目录

根目录：`E:\CarlaAirData\CityInspection_GOC\e4_ugv_sensor_safety\smoke`

- `obstacle\CI_E1_UGV_RGBD_SAFETY_OBSTACLE_S1001_20260927T064531Z`
- `clear_path\CI_E1_UGV_RGBD_SAFETY_CLEAR_S1001_20260927T064836Z`
- `sensor_stale\CI_E1_UGV_RGBD_SAFETY_STALE_S1001_20260927T064907Z`

每组的 `validation_report.json` 是验收总表；`safety/ugv_obstacle_perception.csv` 记录逐帧 RGB-D 避障状态；`safety/obstacle_safety_audit.json` 汇总阈值、停车/失效 tick 和真值读取审计；`synchronization/ugv_rgbd_frame_index.csv` 记录本地 RGB-D 对齐；`trajectories/ugv_trajectory.csv` 记录车辆轨迹。紧凑 RGB 与彩色深度样本位于 `sensors/ugv/`。

`preview/ugv_obstacle_detection.png` 是障碍组检测检查图；三组各自的 `preview/ugv_rgbd_safety_replay.mp4` 是 UGV RGB 与深度彩色图逐帧并列的本地回放，使用保存样本频率（1 Hz），因此视频时长对应仿真时长，但不是逐仿真帧录制。全局 `synchronized_multiview_replay.mp4` 仍受四流共同持久化帧数量限制：障碍组只有 1 帧，不可用于判断完整的 UAV/UGV 同步动态过程。

## 结论和限制

这次实验支持一个有限结论：UGV 的基础避障决策已从在线查询 CARLA 障碍物真值切换为本地深度图几何判定；障碍触发、畅通不误停、深度失效保护这三种行为均通过短时验收。它尚不证明复杂动态交通中的检测可靠性，也没有验证语义识别、完整路线避障或统计性能。

当前明确待办是修复/解释全局四流回放在 UGV 停车用例中共同保存帧过少的问题；后续扩大到行人、移动车辆、多随机种子前，应先确认同步数据记录策略。另外，障碍检测目前是深度几何占用走廊法，不是训练得到的检测网络。

## 本阶段代码

- `src/safety/rgbd_obstacle_guard.py`：深度反投影、前方走廊筛选、距离/TTC 与传感器陈旧状态机。
- `scripts/run_stage1_dynamic.py`：接入 UGV RGB-D 安全状态，记录逐帧状态、对齐索引和真值读取审计。
- `scripts/build_ugv_rgbd_safety_replay.py`：生成 UGV 本地 RGB-D 回放。
- `scripts/Run-UGV-RGBDSafetySmoke.ps1`：启动 CARLA-Air 并按顺序运行短时验收组。
- `configs/experiments/ci_e1_ugv_rgbd_safety_{obstacle,clear,stale}_smoke.yaml`：三种验收用例配置。
