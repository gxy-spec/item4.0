# CI-E6：UAV/UGV 双端候选与坐标时间对齐验收（2026-10-01）

## 本阶段问题

检验 UAV 与 UGV 能否各自从同步 RGB-D 观测中产生红色车辆候选，并把 RGB、深度、CARLA 仿真帧、仿真时间和候选三维坐标写成统一格式。此阶段不做候选融合、不发通信、不据候选控制 UGV。

## 固定实验条件

- Town10HD Zone A，随机种子 1001，静止红色厢式目标车。
- UGV 从既定巡检路线起步并使用已有 RGB-D 安全保护；UAV 固定悬停在目标上方约 35 m，仅作为独立观测端。
- 两端各用 800×600、100° FOV 的 RGB 与深度相机；每 5 个传感器帧（0.5 s）在相同 CARLA 帧上执行一次候选推理。
- 两端统一使用 YOLO26n COCO 车辆类提议、HSV 红色属性判断和深度反投影；模型权重和阈值一致，两个端点的时序候选历史彼此独立。
- 在线三维位置统一记录为 CARLA 世界坐标 XYZ（米）。此名称指 CARLA 地图全局坐标，不误称地理 ENU；AirSim NED 只保留在仿真桥接控制边界，不用于候选文件坐标。
- 完整 RGB/深度仅每 20 个传感器帧（2 s）稀疏保存；每 0.5 s 保存候选记录。深度保存为米制浮点图，便于复核反投影。

## 执行与验收

1. 12 s smoke：检查仿真连接、四路 RGB-D 同帧、两端推理与输出结构。Smoke 不要求目标已被 UGV 看见。
2. 60 s formal：检查共帧比例、UGV 巡检、两端候选及坐标对齐。候选帧的目标距离只在仿真结束后用真值计算；在线检测和控制中目标真值读取数必须为 0。
3. 正式验收要求：每端至少 10 个有效红色 RGB-D 候选帧；至少 10 个两端同时观测且各自距目标真值不超过 10 m 的帧；两端坐标差中位数 ≤5 m、P90 ≤10 m；候选记录中的 RGB/深度帧号相同，坐标框架为 `CARLA_WORLD_XYZ_M`。
4. 生成同步回放、候选位置对齐图和图像候选叠框，用于人工核查可见性与错误候选。

## 关键输出

- `perception/candidate_records.jsonl`：统一候选主表。
- `perception/uav_target_candidates_synchronized.jsonl`、`perception/ugv_target_candidates_synchronized.jsonl`：逐端逐推理帧（包括无候选帧）的原始记录。
- `perception/candidate_frames.jsonl`：两端共帧、时间、传感器位姿与每帧候选数。
- `ground_truth/evaluation_only/dual_device_candidate_alignment_evaluation.json`：仅离线使用真值计算的设备间误差与共同观测统计。
- `preview/candidate_alignment_summary.png`、`preview/synchronized_multiview_replay.mp4`：可视化验收结果。

## 边界说明

这次验收证明的是观测数据时间/坐标对齐和候选生成接口可用，不代表目标识别已达到学术级准确率，也不代表完成 UAV-UGV 候选融合或任务通信。若两端候选偏差未通过阈值，应先定位相机姿态、深度几何、候选误检和目标可见性，不把它包装成通过。
