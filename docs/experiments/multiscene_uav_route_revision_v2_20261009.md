# W1/W2/F1 UAV 弓字航线修订 v2（2026-10-09）

本阶段完成航线修订、物理净空与 AirSim 航点执行检查，以及新增扫描排的短时同步 RGB-D 验证。没有进行正式 A/B/C 对照。航线配置为 `configs/regions/town10hd_multiscene_uav_routes_v2.yaml`；新 manifest 位于 `docs/figures/multiscene_design_20261009_v2/proposal_manifest.json`。同一场景的三套部署和 A/B/C 共用同一 UAV 航线。

## 修订内容

- 西部：增加 y=90 m 扫描排；相邻排交替飞行，10 个航点、9 段、4 次折返，无重复扫描段。高度 60 m；总长 384 → 480 m；3 m/s 名义完整航时 160 s。
- 整图：在 y=5 与 y=65 m 之间增加 y=20 m 扫描排，并调整后续方向以保持连续弓字形。10 个航点、9 段、4 次折返。高度 80 m；总长 907 → 1120 m；名义完整航时 373.33 s。
- UGV 起点、巡检终点、目标位置、北部/东部 UAV 航线、相机与感知算法均未改变。当前配置只提供新 UAV 航线；未来正式回合须按新完整航时设置时长。

## 原可见性统计勘误

此前“西部 1/2 仅 1 个点、整图 1 仅 2 个点”的数字和“不可见性不足”的判定作废。目标生成后仍在落地，射线终点却固定在初始车顶高度，部分射线没有触及落稳后的车体；相邻航段端点还重复计数。现让目标落稳后固定，使用相机实际 -1.5 m 挂载位置、去除重复采样点，并对原航线和修订航线按同一方法重新检查。原航线本身也具有可见机会，修订提升的是街道覆盖和有效观测跨度。

目标真值只用于此次离线场景勘测和评估。RGB-D 候选推理仍只接收图像、深度和相机标定/位姿。

| 部署 | 原航线可见空间点（更正后） | 修订后可见空间点 | 最长连续几何可见航段（m） |
|---|---:|---:|---:|
| W1 | 13 | 25 | 48.0 → 96.0 |
| W2 | 13 | 25 | 48.0 → 96.0 |
| W3 | 21 | 37 | 44.0 → 100.0 |
| F1 | 45 | 80 | 102.7 → 202.8 |
| F2 | 33 | 46 | 127.7 → 127.7 |
| F3 | 33 | 37 | 127.1 → 119.9 |

这里的“点”按不超过 4 m 间距进行空间抽样，不是视频帧、检测召回率或任务成功率。连续跨度也是几何射线统计，不能直接等同于模型持续检测时长。其他候选的可见点数未减少；F3 最长连续跨度略有变化，仍保有多次观测机会。

## 物理与传感器验证

两条完整修订航线的中心及 ±1.2 m 轮廓射线未发现碰撞；AirSim 的非忽略碰撞位置命令扫过完整航线，无新碰撞和航点拒绝，最大位姿误差 0.39 m。本项验证现有位置命令控制方式，不是多旋翼动力学或完整任务闭环验收。

新增扫描排各运行 12 s、3 m/s，固定步长 0.05 s，共 240 帧；候选推理每 0.5 s 一次，共 24 次。RGB-D 同帧率 100%，时间戳误差 0，三组 UAV 碰撞帧为 0，短段最大位姿误差 0.418 m。目标可见性使用目标投影区域中的红色像素和车顶深度一致性辅助核验，并抽查真实图像。

| 部署 | 图像/深度可见帧 | 同帧 RGB-D | 目标候选推理帧 | 时序确认帧 | 纯 YOLO 目标框命中帧 |
|---|---:|---:|---:|---:|---:|
| W1 | 240/240 | 240/240 | 24/24 | 23 | 1/24 |
| W2 | 240/240 | 240/240 | 24/24 | 23 | 3/24 |
| F1 | 240/240 | 240/240 | 12/24 | 11 | 0/24 |

候选模块沿用轻量 YOLO、HSV 红色区域提议、深度几何筛选和 2/5 时序确认。最后一列是纯 YOLO 框与目标投影 IoU≥0.3 的单独统计；不能把颜色/几何产生的候选称作 YOLO 成功检测。F1 的纯 YOLO 仍漏检，候选产生也有间断；本阶段确认航线提供了清晰观测机会，完整感知闭环仍须另行验收。W1/W2 的目标与 UAV 路线相同，这两个回放只是相同 UAV 观测条件下的重复检查。

一次探测会话曾在创建相机后切换同步模式时触发 UE4 GameThread 空指针访问崩溃，发生于 `world.apply_settings`。已改为沿用主项目的“先同步、后创建传感器”流程，保持模式到会话末尾，先销毁相机再恢复异步；重试完成了上述三组。原崩溃的因果内部堆栈无符号，不能进一步归因于某个 UE4 函数。

## 输出

- 航线前后图：[西部](../figures/multiscene_design_20261009_v2/west_route_revision.png)、[整图](../figures/multiscene_design_20261009_v2/whole_route_revision.png)。
- [机器可读验收总表](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2/route_revision_summary.json)。
- [修订路线几何检查](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2/preflight.json)、[原路线更正检查](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2/corrected_original/preflight.json)、[AirSim 完整航线扫查](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2/uav_route_sweep.json)。
- [RGB-D 观测联系图](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2/uav_rgbd_probe_contact_sheet.png)。
- 回放：[W1](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2/rgbd_probe/W1/uav_rgbd_visibility_replay.mp4)、[W2](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2/rgbd_probe/W2/uav_rgbd_visibility_replay.mp4)、[F1](E:/CarlaAirData/CityInspection_GOC/multiscene_feasibility_20261009_v2/rgbd_probe/F1/uav_rgbd_visibility_replay.mp4)。紫框是验收真值框，绿/黄框为最近一次推理的候选；画面注明该次推理时间。
- 各组 `rgbd_probe/<组名>/summary.json`、`frame_checks.json`、`candidate_checks.json` 保存同步、位姿、可见性和候选来源记录；每组仅保存 3 对代表性 RGB/深度图，不保存完整传感器数据集。

结论：航线修订物理检查与新增扫描排 RGB-D 短段验收通过，配置和地图已更新。完整多场景感知、UGV 安全闭环和 36 回合 A/B/C 尚未运行。
