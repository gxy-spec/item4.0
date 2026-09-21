# CI-E0 场景定义与5秒预览验收结果

## 结论

CI-E0通过。冻结配置运行共检查34项，34项通过，无错误、无警告。

## 正式输出

- 区域勘察：`E:\CarlaAirData\CityInspection_GOC\e0_platform\scene_definition_v1`
- 正式预览：`E:\CarlaAirData\CityInspection_GOC\e0_platform\CI_E0_T10_ZA_S1001_20260921T091152Z`
- 调试过程：`E:\CarlaAirData\CityInspection_GOC\e0_platform\_debug_attempts`

调试过程仅保留用于审计，不应作为论文结果或后续实验输入。

## 冻结场景

- 地图：Town10HD；
- 区域：Zone A，150 m × 150 m；
- UGV出生点：95；
- UGV参考路线：89.95 m；
- UAV高度：40 m；
- UAV航线：10个弓字形航点；
- 目标：`vehicle.mercedes.sprinter`，红色，路侧静止；
- 指令：寻找一辆红色厢式货车，并前往其所在位置；
- 干扰物：7辆车辆、6名行人；
- 传感器：UAV RGB/Depth、UGV RGB/Depth，640 × 360；
- 仿真：同步模式，固定步长0.05 s，传感器10 Hz；
- 随机种子：1001。

干扰车辆包含同类不同色和同色不同类，避免把任务退化成单一类别检索。

## 关键验收数值

- Town10HD出生点：155；
- 路口：9；
- Zone A可用车辆出生点：85；
- 目标到UAV搜索航线最近距离：15.0 m；
- 保守半视场覆盖：36.0 m；
- UAV固定位置误差：0.00 m；
- 5秒预览公共同步帧：49；
- 正式保存帧：342；
- 四路帧差：0。

## 正式文件

- `resolved_config.yaml`：展开并加入运行时解析信息的配置；
- `validation_report.json`：34项验收结果；
- `scene_manifest.json`：地图、智能体、传感器和输出清单；
- `actors.csv`：UGV、目标、干扰车辆、行人与UAV位置；
- `preview/map_overview.png`：Zone A道路、两端路线和智能体位置；
- `preview/uav_rgb.png`、`preview/uav_depth_*`：无人机端预览；
- `preview/ugv_rgb.png`、`preview/ugv_depth_*`：地面车端预览；
- `preview/scene_composite.png`：综合人工审核图。

## 阶段边界

CI-E0只证明场景生成、配置管理、智能体生成、传感器对齐和数据输出正确。当前尚未执行目标检测、自然语言模型推理、UGV闭环导航或GOC资源优化。下一阶段为CI-E1 Oracle闭环。

