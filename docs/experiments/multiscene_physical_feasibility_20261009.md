# Town10HD 多场景候选物理可行性预验收（2026-10-09）

> 勘误（航线修订 v2 复核）：本报告中原始“9/12 可见、西部 1/2 与整图 1 可见点不足”的判定作废。原脚本没有固定仍在落地的目标车，却固定了射线终点，造成假阴性；还重复计数了相邻航段的共用端点。当前已修复，原路线与修订路线均重新检查。旧 JSON 作为历史记录保留，不应继续引用旧可见点数。详见 `multiscene_uav_route_revision_v2_20261009.md`。

本次只验证场景能否在 CARLA-Air 中搭建和运动，**不是 A/B/C 正式对照实验**。勘测允许使用目标真值、CARLA 射线和 Traffic Manager；这些均未接入在线感知或 UGV 正式控制。原设计文件保持不变。

## 验收口径与结果

| 项目 | 检查方法 | 结果 |
|---|---|---|
| 12 套起点／巡检终点／目标候选 | Town10HD 真实 spawn、驾驶车道与项目路径规划器 | 12/12 目标可生成，12/12 巡检路线和目标分支可达 |
| 4 条原定 UAV 弓字航线 | 每 ≤3 m 沿中心及 ±1.2 m 机身轮廓射线扫查；AirSim 非忽略碰撞的位置命令扫过航线 | 4/4 无新碰撞、无位姿拒绝；AirSim 最大位姿误差 0.39 m |
| 原定航线中的目标视线 | 停放真实车辆；按 800×600、100°、俯视相机视锥检查目标车顶射线 | 9/12 有至少 3 个无遮挡采样点；西部 1、2 号和整图 1 号不足 |
| 代表性 UGV 真实行驶 | 北 1、东 1、西 3、整图 2；各自分别驶向巡检终点和目标分支，遵守 Traffic Manager 路口控制 | 8/8 到达终点 8 m 内；8/8 碰撞 0。东、西、整图的目标分支有红灯等待，延长等待后均通过 |
| 代表性 UAV RGB | 将 AirSim 实测机位用于 CARLA 相机，配置与当前项目相同的 800×600、100°、俯视姿态；保存原图及仅用于验收的真值框 | 4/4 红色目标出现在图像内；机位误差 ≤0.78 m，但目标尺寸较小 |

代表方案动态行驶结果：

| 街区/候选 | 巡检终点最近距离 | 目标分支最近距离 | 碰撞 | 观察 |
|---|---:|---:|---:|---|
| 北部 1 | 6.94 m | 7.95 m | 0 | 巡检路线完成路口转弯 |
| 东部 1 | 6.90 m | 6.95 m | 0 | 目标分支等灯后完成转弯 |
| 西部 3 | 6.74 m | 7.17 m | 0 | 目标分支等灯后完成转弯 |
| 整图 2 | 6.73 m | 7.72 m | 0 | 长目标分支等灯后完成转弯 |

注意：这里的“合法”只表示 CARLA 驾驶车道路径可规划、Traffic Manager 按仿真交通灯驾驶并完成转弯，**不等于已经证明项目的 RGB-D 控制器在该场景安全完成行驶**，也不等于对所有真实交通法规逐条形式化验证。4 条 UAV 航线验证的是当前项目采用的 AirSim 位置命令控制及几何净空，并非真实多旋翼动力学性能。

## 发现的边界问题

1. **三套原航线视线不足**：西部候选 1/2 各只有 1 个可见采样点，整图候选 1 只有 2 个，不应直接进入正式随机种子实验。保留原设计图与 manifest；仅在独立建议文件中尝试西部增设 y=90 m 扫描排、整图增设 y=20 m 扫描排。新路线的射线净空通过，西部 3/3、整图 3/3 候选均达到至少 3 个无遮挡点；**建议尚未替换用户已审阅的航线，也未跑正式实验**。
2. **物理可见不等于感知可检**：代表性 RGB 真值框约为北部 24×46 px、东部 12×44 px、西部 36×21 px、整图 32×12 px。单帧烟测中，当前 yolo26n（640/0.08）仅检出西部目标；yolo26x（1280/0.03）检出西部与整图目标，北部、东部仍漏检。这只是每组一帧，不可作为召回率估计，但提示正式实验前必须做沿航线多帧感知验收，必要时调整观测距离／高度／分辨率或模型。
3. **早期“停滞”不是物理障碍**：初版预验收把等红灯超过 18 s 误作不可达；增加交通灯状态日志及等待时间后，东、西、整图目标分支均到达且无碰撞。

## 输出索引

- 完整 12 候选原路线、可见性和几何净空：[preflight.json](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009/preflight.json)
- AirSim 四条航线位置命令扫查：[uav_route_sweep.json](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009/uav_route_sweep.json)
- 8 次 UGV 实际行驶轨迹：`E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009/drive_<scene>_<candidate>_<patrol|target>.json`
- UAV 原始图与 Oracle 验收框：同目录 `uav_view_<scene>_<candidate>.png` 和 `uav_view_<scene>_<candidate>_oracle_qa.png`
- 目标像素尺寸、AirSim 实测机位：[uav_capture_check.json](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009/uav_capture_check.json)
- 单帧模型补充检查：[one_frame_detector_smoke.json](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009/one_frame_detector_smoke.json)
- 未生效的航线修订建议：[proposed_uav_route_adjustments.json](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009/proposed_uav_route_adjustments.json)

下一步应先审阅并选定修订航线，对每个拟进入正式实验的种子做短回合多帧 RGB-D／目标候选／UGV 本地避障 smoke；只有通过后再做配对 A/B/C。整图显卡与长时运行边界未在本次低负载预验收中测量。
