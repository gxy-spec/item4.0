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

## 运行入口

```powershell
cd E:\Research\CityInspection_GOC
.\scripts\Run-Stage0-Survey.ps1
.\scripts\Run-Stage0-Preview.ps1
.\scripts\Run-Stage1-Dynamic.ps1 -Mode Smoke
.\scripts\Run-Stage1-Dynamic.ps1 -Mode Formal
python .\scripts\build_dynamic_global_view.py --run-dir <CI-E1结果目录> --fragment <HTML输出路径>
```

每次正式运行都保存展开后的配置、校验报告、场景清单、智能体列表和可视化文件。大体量数据不会写入代码仓库。

详细验收结论见：

- `docs/CI_E0_RESULTS_20260921.md`
- `docs/CI_E1_RESULTS_20260921.md`
