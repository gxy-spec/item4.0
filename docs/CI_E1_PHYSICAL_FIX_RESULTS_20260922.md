# CI-E1.1 物理可信度修复验收

## 结论

30秒短验收已通过。该运行只用于证明UGV安全停车、UAV无穿模、UAV宽视角和五视图同步回放有效，不作为新的完整训练数据集。

正式修复结果：

`E:\CarlaAirData\CityInspection_GOC\e1_physical_fix\smoke\CI_E1_PHYSICAL_FIX_T10_ZA_S1001_20260922T012328Z`

## 修复内容

- UGV增加基于CARLA真值的安全保护层、分级减速、紧急制动和碰撞传感器；
- UGV任务终点改为在目标车辆前保持安全距离，不再以接触目标车为完成条件；
- UAV高度由40 m提高至60 m；
- UAV强制位姿更新禁止忽略碰撞，并在进入航线后记录AirSim新增碰撞；
- UAV RGB/深度相机由640×360、90°改为800×600、100°；
- UAV相机使用固定世界北向，不再随飞行方向旋转；
- 五视图回放增加UGV速度、目标距离、安全模式、UAV高度、中央净空和碰撞计数。

## 验收结果

| 指标 | 结果 | 阈值 | 判定 |
|---|---:|---:|---|
| 四路公共帧 | 300/300 | ≥95% | PASS |
| 四路最大帧差 | 0 | =0 | PASS |
| UGV碰撞次数 | 0 | =0 | PASS |
| UAV航线新增碰撞 | 0 | =0 | PASS |
| UGV最终目标距离 | 6.11 m | 3–7 m | PASS |
| UGV最终速度 | 0.0 m/s | 停车 | PASS |
| UAV最低中央净空 | 20.21 m | ≥5 m | PASS |
| UAV高度 | 60 m | 配置值 | PASS |
| UAV视角 | 800×600、100° | 宽视角 | PASS |
| UAV航线 | 149.75 m、3个航点、2个航段 | 短验收要求 | PASS |

## 关键输出

- `validation_report.json`：19项验收结果；
- `safety/safety_state.csv`：逐仿真步安全状态；
- `safety/ugv_collision_events.json`：空列表；
- `safety/uav_collision_events.json`：空列表；
- `preview/synchronized_multiview_replay.mp4`：30秒五视图同步证明视频；
- `preview/synchronized_multiview_replay.json`：视频帧映射说明；
- `preview/trajectory_map.png`：短验收全局轨迹。

## 说明

调试运行 `CI_E1_PHYSICAL_FIX_T10_ZA_S1001_20260922T011942Z` 未通过，原因包括高分辨率相机回调等待不足和AirSim初始定位碰撞被误计入航线。最终版本增加了800×600模式的渲染屏障，并在进入航线后建立碰撞基线。随后生成的 `20260922T012328Z` 运行是本阶段唯一正式修复结果；失败的调试目录已于2026-09-22完成审核后清理。
