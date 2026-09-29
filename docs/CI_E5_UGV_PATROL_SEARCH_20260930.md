# 9月30日实验记录：UGV巡检与本地RGB-D搜索

## 任务与方法

- 平台/场景：CARLA-Air、Town10HD Zone A、随机种子1001；固定步长0.05秒，正式时长60秒。
- 目标：红色厢式货车固定在场景中，仿真期间不受控制器移动。
- UGV：从启动即沿冻结巡检路线行驶，目标速度2 m/s；避障输入仅来自UGV RGB-D深度安全守卫，不读取障碍物Actor位置/速度。
- 搜索：UGV RGB-D每10 Hz采样；检测器每5个传感器帧推理一次（2 Hz）。候选由YOLO26n COCO车辆框与HSV红色区域提议构成，颜色比例、深度反投影和时序关联用于生成候选位置。该阶段不发送目标消息、不用候选控制UGV。
- 存储：完整运行元数据与轨迹逐帧保存；RGB和深度彩色预览每10帧保存一次（约1 Hz），未保存完整高体量原始深度数据。
- UAV：固定在初始观测点作为被动传感器；9月30日不验收UAV航线。

## 正式结果

正式组通过全部验收项。UGV行驶83.14 m；目标漂移0.00 m；UGV碰撞0次；最终距目标7.21 m。UGV RGB-D配对600/600（100%），深度避障模块覆盖固定仿真步长，记录到160个STOP tick、94个CAUTION tick及1个SENSOR_STALE tick。停车/减速统计表示控制器输出的安全状态，不代表160次独立危险事件。

候选日志共171个红色车辆候选点，其中120个点的二维位置距目标真值不超过5 m（70.2%）；120/120个推理帧都至少包含一个距目标真值5 m以内的候选，最佳误差2.89 m。真值仅用于仿真后的空间匹配统计，未进入在线感知、候选选择或UGV控制。该比例是单静态目标场景下的候选点匹配代理指标，不是独立目标检测精度；场景没有车辆/行人干扰物，不能据此宣称复杂环境下已可靠识别。

## 特别说明

首个60秒尝试中UGV任务达到上述运动、感知和安全结果，但UAV沿旧航线撞到Town10HD高层建筑，导致整体报告未通过。由于9月30日主任务只验收UGV巡检与本地搜索，修正后的正式组将UAV固定为被动观测端并重新完整采集；首个失败运行保留，不覆盖、不删除。

## 输出位置

- 正式运行目录：`E:/CarlaAirData/CityInspection_GOC/e5_ugv_patrol_search/20260930/formal/CI_E5_UGV_PATROL_SEARCH_T10_ZA_S1001_20260930_20260928T070116Z`
- 20秒预检目录：`E:/CarlaAirData/CityInspection_GOC/e5_ugv_patrol_search/20260930/smoke/CI_E5_UGV_PATROL_SEARCH_T10_ZA_S1001_SMOKE_20260930_20260928T070014Z`
- 首次未通过（UAV航线撞建筑）：`E:/CarlaAirData/CityInspection_GOC/e5_ugv_patrol_search/20260930/formal/CI_E5_UGV_PATROL_SEARCH_T10_ZA_S1001_20260930_20260928T065403Z`
- 正式验收报告：正式运行目录下 `validation_report.json`
- 轨迹/指标总图：正式运行目录下 `preview/ugv_patrol_search_summary.png`
- 候选匹配统计：正式运行目录下 `ground_truth/evaluation_only/ugv_patrol_search_analysis.json`
- Town10HD道路背景轨迹图：正式运行目录下 `preview/trajectory_map.png`
- UGV RGB-D同步回放：正式运行目录下 `preview/ugv_rgbd_safety_replay.mp4`
- 候选时序记录：正式运行目录下 `perception/ugv_target_candidates_audited.jsonl`
- 原始候选记录：正式运行目录下 `perception/ugv_target_candidates.jsonl`（保留原始内容；来源字段曾固定误写为yolo26x，分析副本已按配置校正为yolo26n）
- 候选后处理评估：正式运行目录下 `ground_truth/evaluation_only/ugv_patrol_search_analysis.json`
- UGV轨迹、目标轨迹：正式运行目录下 `trajectories/ugv_trajectory.csv`、`trajectories/target_trajectory.csv`
- RGB-D对齐索引、安全状态及碰撞记录：正式运行目录下 `synchronization/ugv_rgbd_frame_index.csv`、`safety/ugv_obstacle_perception.csv`、`safety/ugv_collision_events.json`
- 可复现实验入口：`scripts/Run-UGVPatrolSearch.ps1`

实验后检查：`carlaAir`环境下全套测试37项通过；正式配置的JSON Schema校验通过。
