# CI-E1 动态双端 RGB/深度同步采集验收

## 结论

CI-E1 已通过。Town10HD Zone A 中的无人机、地面车、目标车辆、7辆干扰车和6名行人均成功生成；除固定目标外，其余任务参与者按配置产生动态位移。无人机端与地面车端各自的 RGB 和深度相机完成20秒、10 Hz严格同帧采集。

正式结果目录：

`E:\CarlaAirData\CityInspection_GOC\e1_dynamic_sensor_acceptance\formal\CI_E1_T10_ZA_S1001_20260921T104109Z`

最终冒烟测试目录：

`E:\CarlaAirData\CityInspection_GOC\e1_dynamic_sensor_acceptance\smoke\CI_E1_SMOKE_T10_ZA_S1001_20260921T104038Z`

## 正式组配置

| 项目 | 配置 |
|---|---:|
| 地图与区域 | Town10HD / Zone A |
| 随机种子 | 1001 |
| 仿真时长 | 20.0 s |
| 固定仿真步长 | 0.05 s |
| 传感器频率 | 10 Hz |
| 图像尺寸 | 640 × 360 |
| 任务参与者 | 1 UAV、1 UGV、1固定红色厢式目标车 |
| 动态干扰项 | 7辆车辆、6名行人 |
| 传感器流 | UAV RGB、UAV depth、UGV RGB、UGV depth |

## 验收指标

| 指标 | 正式结果 | 阈值 | 判定 |
|---|---:|---:|---|
| 四路公共帧 | 200 / 200 | ≥ 95% | PASS |
| 四路最大帧号差 | 0 | = 0 | PASS |
| UGV轨迹长度 | 88.43 m | ≥ 40 m | PASS |
| UAV轨迹长度 | 99.75 m | ≥ 50 m | PASS |
| UAV控制误差P95 | 0.000018 m | ≤ 1 m | PASS |
| 目标车辆漂移 | 0.00 m | ≤ 0.20 m | PASS |
| 有效移动干扰车辆 | 7 | ≥ 4 | PASS |
| 有效移动行人 | 6 | ≥ 4 | PASS |
| UAV RGB唯一帧比例 | 100% | ≥ 80% | PASS |
| UGV RGB唯一帧比例 | 100% | ≥ 80% | PASS |
| UAV/UGV深度有限值比例 | 100% | ≥ 99.9% | PASS |

## 数据完整性

两端各保存：

- RGB图像：200张；
- 编码深度图：200张；
- 米制深度数组：200个；
- 彩色深度预览：200张；
- 帧级传感器元数据：200条。

正式目录共1618个文件，约0.585 GB。全体参与者的状态以20 Hz写入 `actors/actor_states.jsonl`；UGV、UAV和目标车辆轨迹分别写入 `trajectories/`；四路传感器的精确帧映射写入 `synchronization/frame_index.csv`。

## 关键输出

- `validation_report.json`：全部验收项及数值；
- `scene_manifest.json`：地图、区域、参与者和传感器清单；
- `preview/sensor_composite.png`：两端RGB与深度可视化；
- `preview/trajectory_map.png`：UAV、UGV、目标、车辆和行人的二维轨迹；
- `preview/synchronization_plot.png`：四路公共帧和时间间隔；
- `sensors/uav/`、`sensors/ugv/`：按设备和模态分层的数据；
- `actors/`、`trajectories/`：逐帧真值和轨迹。

## 工程说明

长序列测试中，CARLA同步世界推进仍可能快于GPU相机回调。采集入口已加入短渲染屏障，并将参与者真值轨迹与传感器回调解耦，最终正式组实现200/200帧完整采集。调试失败目录被保留用于问题追溯，但不作为正式实验结果。

本阶段只证明动态场景、两端传感器、同步采集和数据组织可用；尚未执行视觉语言目标识别、协同消息传输或导航策略评估。
