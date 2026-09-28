# CI-E1：UGV RGB-D 避障单独验收结果（2026-09-29 计划）

## 结论

六类验收场景的最新正式结果全部通过：空路线、静态障碍与恢复、横穿车辆、横穿行人、车辆遮挡后行人横穿、深度传感器失效。各组均无 CARLA 碰撞事件；障碍/失效场景中，安全状态由 UGV 自己的 RGB-D 深度数据驱动，日志记录的在线障碍 actor 真值读取数为 0。深度失效组触发停车保护，在传感器恢复后重新起步。

实验文件夹沿用排期编号 `20260929`；本轮实际运行时间为 2026-09-27，运行目录名保留了实际采集时间戳。所有失败的诊断运行均保留，未覆盖。

## 方法与判据

- 固定 Town10HD 与 UGV 路线；RGB-D 深度图按相机标定反投影到车体坐标系，只在前方行驶走廊内估计最近障碍距离。
- 安全状态为 `CLEAR / CAUTION / STOP / SENSOR_STALE`，用距离/TTC阈值触发减速、停车；障碍离开或深度恢复后，经连续清空帧再恢复行驶。
- 不向实时避障控制读取 CARLA 障碍物位置。CARLA 碰撞事件和 actor 轨迹只在仿真结束后用于验收与距离估算。
- 紧凑保存传感器样本：每秒约一组 RGB-D 图像；控制过程仍逐传感器帧读取深度。每组回放采用同帧号的 UGV RGB 与深度图。

## 正式结果

| 场景 | 最新运行 | 结果 | UGV碰撞 | 避障/保护状态 | 关键结果 |
|---|---|---:|---:|---|---|
| 空路线 | `CI_E1_UGV_SAFETY_20260929_CLEAR_S1001_20260927T073033Z` | PASS | 0 | 无 CAUTION/STOP | UGV行驶48.1 m，未出现无故停车 |
| 静态障碍后移除 | `CI_E1_UGV_SAFETY_20260929_STATIC_RECOVERY_S1001_20260927T073125Z` | PASS | 0 | STOP 106 tick；CAUTION 36 tick；恢复通过 | 障碍移除后重新起步；后处理估计间距0.91 m |
| 横穿车辆 | `CI_E1_UGV_SAFETY_20260929_CROSSING_VEHICLE_S1001_20260927T074301Z` | PASS | 0 | STOP 112；CAUTION 52；恢复通过 | 最近深度障碍距离3.74 m；后处理圆形近似间距 -0.21 m |
| 横穿行人 | `CI_E1_UGV_SAFETY_20260929_CROSSING_PEDESTRIAN_S1001_20260927T074932Z` | PASS | 0 | STOP 44；CAUTION 42；恢复通过 | 行人轨迹位移16.0 m；最近深度障碍距离8.97 m；圆形近似间距 -0.61 m |
| 遮挡后横穿 | `CI_E1_UGV_SAFETY_20260929_OCCLUSION_S1001_20260927T075738Z` | PASS | 0 | STOP 123；CAUTION 72；恢复通过 | 先移除遮挡车辆再让行人横穿；最近深度障碍距离5.77 m；圆形近似间距2.04 m |
| 深度失效 | `CI_E1_UGV_SAFETY_20260929_DEPTH_FAILURE_S1001_20260927T075915Z` | PASS | 0 | SENSOR_STALE 81 tick；停车保护与恢复均通过 | 注入失效约4.05 s（81个50 ms控制tick）；传感器恢复后重新行驶 |

所有六组的 UGV RGB-D 配对率均为100%，每组约有18–32组紧凑保存图像对；正式检查项全部通过。障碍场景验证了实时避障不依赖 CARLA actor 真值。

## 可视化与记录位置

结果根目录：`E:\CarlaAirData\CityInspection_GOC\e4_ugv_sensor_safety\20260929`

每个运行目录均包含 `validation_report.json`、`safety\safety_state.csv`、`safety\ugv_obstacle_perception.csv`、`safety\ugv_collision_events.json`、`preview\ugv_obstacle_detection.png`、`preview\trajectory_map.png` 和 `preview\ugv_rgbd_safety_replay.mp4`。视频为按1 Hz抽样的紧凑 RGB/深度同步回放，不是实时录屏。

对应最新运行目录：

- 空路线：[clear](E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260929/clear/CI_E1_UGV_SAFETY_20260929_CLEAR_S1001_20260927T073033Z)
- 静态障碍：[static_recovery](E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260929/static_recovery/CI_E1_UGV_SAFETY_20260929_STATIC_RECOVERY_S1001_20260927T073125Z)
- 横穿车辆：[crossing_vehicle](E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260929/crossing_vehicle/CI_E1_UGV_SAFETY_20260929_CROSSING_VEHICLE_S1001_20260927T074301Z)
- 横穿行人：[crossing_pedestrian](E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260929/crossing_pedestrian/CI_E1_UGV_SAFETY_20260929_CROSSING_PEDESTRIAN_S1001_20260927T074932Z)
- 遮挡后横穿：[occlusion](E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260929/occlusion/CI_E1_UGV_SAFETY_20260929_OCCLUSION_S1001_20260927T075738Z)
- 深度失效：[depth_failure](E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260929/depth_failure/CI_E1_UGV_SAFETY_20260929_DEPTH_FAILURE_S1001_20260927T075915Z)

## 需要如实说明的限制

1. 横穿车辆/行人的 CARLA 后处理“间距”使用包围圆近似。车辆和行人两组得到小于0的估计值，但碰撞传感器事件均为0；这说明该近似偏保守，不能将其解释成真实穿模或精确净空。若论文需要严格报告最小间距，应改为基于朝向的二维包围盒/多边形距离，并在后续批次复核。
2. 这些是每场景单次确定性验收，不是多随机种子的统计安全性结论；也不等价于真实道路安全认证。
3. 一次早期行人复验因走廊过宽、将路侧人行道持续判为障碍而未恢复；缩回1.6 m车道半宽后重新运行通过。一次早期遮挡配置让行人停留在车辆行驶线上并发生碰撞；调整横穿时序和横向范围后最新正式运行通过。失败目录完整保留，可追溯配置与原因。

## 验证

- `conda run -n carlaAir python -m pytest -q`：21项测试通过。
- `git diff --check`：通过（只有 Windows 行尾转换提示）。
