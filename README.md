# CityInspection_GOC

面向城市巡检的空地协同视觉语言导航实验项目。

本项目与旧的交叉路口风险实验完全隔离：

- 项目代码：`E:\Research\CityInspection_GOC`
- 实验输出：`E:\CarlaAirData\CityInspection_GOC`
- CARLA-Air平台：`D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`

## 当前阶段

`CI-E0` 已完成：Town10HD实验区域、语言指令、目标车辆、干扰物组成、RGB/深度传感器和输出格式均已冻结，并生成了可人工审核的5秒场景预览。

正式结果：`E:\CarlaAirData\CityInspection_GOC\e0_platform\CI_E0_T10_ZA_S1001_20260921T091152Z`

`CI-E1` 已完成：无人机、地面车、目标车辆、7辆动态干扰车和6名动态行人已在冻结区域中运行；无人机端与地面车端的RGB/深度相机以10 Hz完成20秒严格同帧采集。

正式结果：`E:\CarlaAirData\CityInspection_GOC\e1_dynamic_sensor_acceptance\formal\CI_E1_T10_ZA_S1001_20260921T104109Z`

`CI-E1 Full Route` 已完成：150秒内完成720米弓字航线、10个航点、9个航段和4次折返；UGV在第120秒延迟启动。1500组四路传感器帧全部对齐，并生成全局轨迹与双端双模态同步回放。

完整航线结果：`E:\CarlaAirData\CityInspection_GOC\e1_full_route_acceptance\formal\CI_E1_FULL_ROUTE_T10_ZA_S1001_20260921T112105Z`

`CI-E1.1 Physical Fix` 已完成30秒短验收：UGV和UAV碰撞均为0，UGV停在目标车辆外6.11 m，UAV高度60 m、最低中央净空20.21 m，UAV相机改为800×600、100°固定北向宽视角，并生成带安全状态的五视图同步回放。

修复结果：`E:\CarlaAirData\CityInspection_GOC\e1_physical_fix\smoke\CI_E1_PHYSICAL_FIX_T10_ZA_S1001_20260922T012328Z`

`CI-E2 UAV Candidate Baseline` 已完成：使用UAV同步RGB/深度，经预训练车辆检测、红色属性判断、深度反投影与3/5帧时间确认生成目标候选；CARLA真值与感知入口严格隔离，仅用于推理后的Oracle评价。正式结果目录由 `E:\CarlaAirData\CityInspection_GOC\e2_candidate_baseline\LATEST.txt` 指向。

`S0 Oracle Closed Loop` 已完成：UAV在真值可见性触发后发送一条明确标记的Oracle目标消息，UGV在消息前保持静止、消息后规划并安全到达，UAV继续完成完整弓字航线。150秒正向闭环和20秒无消息负对照均通过。该结果是闭环系统上界，不是感知结果。详见 `docs/S0_ORACLE_CLOSED_LOOP_RESULTS_20260922.md`。

`S1 Perception Closed Loop` 已完成单场景功能验收：UAV 使用自身 RGB/深度生成并门控目标候选，发送一条 `oracle=false` 的目标级语义消息；UGV 收到消息后规划、使用自身 RGB/深度复核并安全到达。正式组 36/36 项通过，无消息负对照 25/25 项通过。正式结果目录由 `E:\CarlaAirData\CityInspection_GOC\e2_perception_closed_loop\LATEST.txt` 指向，详见 `docs/S1_PERCEPTION_CLOSED_LOOP_RESULTS_20260923.md`。

## 运行入口

```powershell
cd E:\Research\CityInspection_GOC
.\scripts\Run-Stage0-Survey.ps1
.\scripts\Run-Stage0-Preview.ps1
.\scripts\Run-Stage1-Dynamic.ps1 -Mode Smoke
.\scripts\Run-Stage1-Dynamic.ps1 -Mode Formal
.\scripts\Run-Stage1-Dynamic.ps1 -Mode FullRoute
.\scripts\Run-Stage1-Dynamic.ps1 -Mode PhysicalFix
.\scripts\Run-Stage2-CandidateBaseline.ps1
.\scripts\Run-S0-OracleClosedLoop.ps1 -Mode All
.\scripts\Run-S1-PerceptionClosedLoop.ps1 -Mode All
python .\scripts\build_dynamic_global_view.py --run-dir <CI-E1结果目录> --fragment <HTML输出路径>
python .\scripts\build_synchronized_multiview_replay.py --run-dir <CI-E1结果目录>
```

每次正式运行都保存展开后的配置、校验报告、场景清单、智能体列表和可视化文件。大体量数据不会写入代码仓库。

详细验收结论见：

- `docs/CI_E0_RESULTS_20260921.md`
- `docs/CI_E1_RESULTS_20260921.md`
- `docs/CI_E1_FULL_ROUTE_RESULTS_20260921.md`
- `docs/CI_E1_PHYSICAL_FIX_RESULTS_20260922.md`
- `docs/CI_E2_UAV_CANDIDATE_BASELINE.md`
- `docs/S0_ORACLE_CLOSED_LOOP_RESULTS_20260922.md`
- `docs/S1_PERCEPTION_CLOSED_LOOP_RESULTS_20260923.md`
- `docs/PROJECT_AUDIT_20260922.md`

## 当前边界

- `CI-E2` 目前只是高召回候选生成基线。正式结果的可见目标召回率为 98.04%，但红色候选精度代理仅为 20.67%；该结果不能替代最终车辆子类识别，也不能直接作为 `S1` 感知闭环结论。
- `S0` 使用 CARLA 真值，仅验证任务状态机、通信、规划、控制、同步采集和记录链路；其 50 ms 时延是配置的一仿真步固定时延，不是实测网络时延。
- `S1` 的在线闭环不读取目标真值，但车辆细分类仍是 COCO 车辆检测加可审计的 `van_like` 启发式，不是专用厢式车分类器；当前结论是单场景功能验收，而非最终感知精度与泛化结论。
- 预训练权重保存在本机 `models/`，不提交 Git。运行 `CI-E2` 前需确保配置引用的权重文件存在；Ultralytics 也可按模型名自动获取公开权重。
