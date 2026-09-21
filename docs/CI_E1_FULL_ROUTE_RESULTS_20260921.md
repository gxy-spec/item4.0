# CI-E1 完整弓字航线与五视图同步回放验收

## 结论

完整航线验收已通过。该实验与原20秒双端传感器同步验收分开保存，不覆盖原正式结果。

正式结果目录：

`E:\CarlaAirData\CityInspection_GOC\e1_full_route_acceptance\formal\CI_E1_FULL_ROUTE_T10_ZA_S1001_20260921T112105Z`

## 配置

| 项目 | 配置 |
|---|---:|
| 仿真时长 | 150 s |
| 固定仿真步长 | 0.05 s |
| 传感器频率 | 10 Hz |
| UAV速度 | 5 m/s |
| 完整规划航线 | 720 m、10个航点、9个航段 |
| UGV策略 | 前120 s等待，第120 s开始巡检 |
| 传感器流 | UAV RGB/depth、UGV RGB/depth |

## 硬性验收结果

| 指标 | 结果 | 阈值 | 判定 |
|---|---:|---:|---|
| 四路公共帧 | 1500/1500 | ≥98% | PASS |
| 最大帧号差 | 0 | =0 | PASS |
| UAV航线长度 | 720.00 m | ≥715 m | PASS |
| UAV到达航点 | 10 | ≥10 | PASS |
| UAV完成航段 | 9 | ≥9 | PASS |
| UAV完成折返 | 4 | ≥4 | PASS |
| UAV跟踪误差P95 | 0.000023 m | ≤1 m | PASS |
| UGV行驶距离 | 75.01 m | ≥40 m | PASS |
| 目标车辆漂移 | 0.00 m | ≤0.20 m | PASS |
| 有效移动干扰车 | 6 | ≥4 | PASS |
| 有效移动行人 | 6 | ≥4 | PASS |

## 数据与回放

两端各生成1500张RGB、1500张编码深度图、1500个米制深度数组、1500张彩色深度图和1500条元数据。正式目录共12020个文件，约4.421 GB。

`preview/synchronized_multiview_replay.mp4` 是150秒、10 fps的严格同步回放，包含：

1. 全局二维轨迹和当前参与者位置；
2. UAV RGB；
3. UAV彩色深度；
4. UGV RGB；
5. UGV彩色深度。

五个窗口通过 `synchronization/frame_index.csv` 使用同一CARLA帧号。视频包含1500帧，分辨率1920×1080，配套映射信息保存在 `preview/synchronized_multiview_replay.json`。

## 阶段边界

本组只验收完整航线执行、UGV延迟启动、动态参与者、双端双模态采集以及逐帧回放。它不代表视觉语言目标识别、跨智能体通信或任务规划算法已经实现。
