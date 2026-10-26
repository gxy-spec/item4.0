# CityInspection_GOC：目标查找与 RGB-D 空地协同主线

当前发布：**2026-10-26，R10 感知/闭环修复基线＋已验证的真值隔离与严格成功评价。**

任务是根据车型、颜色等文字指令，发现指定车辆，由地面端确认身份并安全接近。在线输入为任务文字、UAV/UGV 的 RGB-D、自身定位和已知地图；目标对象真值仅用于场景初始化、训练标注和独立事后评价。

本次只同步已完成的实验主线代码与证据。原生 LiDAR 接入、UAV LiDAR 可配置试验和纯交通拥塞复现保留在本地扩展目录，没有并入本发布，也不计作已完成的主线成果。

## 已实现与实际验证范围

| 内容 | 状态与证据 |
|---|---|
| 目标识别 | 冻结 YOLOE 四类车辆检测/分割、辅助车辆提议、ResNet18 车型/颜色分支及 RGB-D 几何处理 |
| 空地目标消息与闭环 | 候选确认、目标语义消息、持久目标状态、规划、安全避障及地面连续确认 |
| 定向修复验证 | 已有22回合：8个正例通过、2个无目标负例通过、4个安全场景独立审核通过；原问题场景4/6完成 |
| 严格任务成功评价 | 正确实例、完整连续停稳记录、新鲜独立观测、可行驶足迹、包围框距离及全回合无碰撞 |
| 在线真值边界 | 目标/背景对象属性拒绝访问，感知状态缓存仅包含自身状态 |
| 发布时代码验证 | **269项测试通过；22份已执行配置的模式与冻结权重哈希预检通过** |

E2、F1任务未完成，N3严格轨迹配对未通过，深色车辆空中识别仍有弱项，均如实保留。该版本不是“所有场景全部修复成功”的版本。

完整新旧36对／72回合没有在此次修复版上重跑。历史36组及旧结果继续作为历史记录，不能当作本发布的新统计；共同导航、安全和评价改动的收益也不能全部归因于识别模型。

当前只有目标语义消息等既有通信功能；原始数据/特征/目标语义三层真实通信、最终融合架构和GOC动态资源分配尚未完成。

## 结果与版本入口

- [本次发布说明与已知限制](docs/experiments/mainline_20261026/README.md)
- [22回合验证明细](docs/experiments/mainline_20261026/22回合验证明细.csv)
- [原R10定向验收报告](docs/experiments/mainline_20261026/修复定向验收报告.md)
- [原问题场景与修复边界](docs/experiments/mainline_20261026/逐回合分析.md)
- [18个非安全专用任务的严格成功重审](docs/experiments/mainline_20261026/strict_success_reaudit.json)
- [22份已执行配置索引](configs/releases/mainline_20261026/cases.json)
- [模型文件名、来源路径与SHA256](configs/releases/mainline_20261026/assets.json)
- [测试环境清单](docs/experiments/mainline_20261026/tested_environment.json)

## 使用

大体量原始数据、视频、模型权重及缓存不进入Git。自训练权重需从本机已有实验资产取得，或按原训练流程生成；仓库中的元数据不是权重下载链接。

```powershell
python -m pytest tests -q
Get-Content configs/releases/mainline_20261026/cases.json

# 把某一源配置展开到新的输出区，校验权重并替换机器路径。
python scripts/prepare_mainline_config.py `
  --source configs/releases/mainline_20261026/acceptance/upgraded_red_van_C.yaml `
  --destination outputs/configs/red_van.yaml `
  --output-root outputs/mainline_20261026

# 启动独占的CARLA-Air实例后执行；该命令不是本次同步新增的实验结果。
python scripts/run_stage1_dynamic.py --config outputs/configs/red_van.yaml
```

本机可复用资产清单中的现有文件路径；其他机器用 --assets-root 指定按清单命名的权重目录，用 --package-root 指定CARLA-Air安装目录。准备器检查模型SHA256并拒绝覆盖源配置。依赖参考 requirements/windows.txt 和测试环境清单。

独立评价器为 scripts/evaluate_r10.py，仅在全新输出区写报告；默认 outputs/mainline_20261026，可通过 CITYINSPECTION_MAINLINE_OUTPUT 指定。底层运行器PASS/FAIL不等于独立S1/S2/S3，必须分别解释。

## 距离与真值边界

最终任务成功不再由目标中心距单独决定。独立事后检查要求正确目标、连续地面确认、停稳2秒、速度≤0.3m/s、可行驶区域足迹、全回合无碰撞，以及 **CARLA API包围框水平间距3–8米**；在线传感器间距证据为4–7米，至少6次新鲜独立观测。路口允许短暂停稳确认并单独标注。

包围框距离是几何代理量，不是精确碰撞/渲染网格净距。历史报告中的“真实车体间隙”须按此限定理解。严格重审仍通过的12次成功，不等于所有网格表面间距已得到认证。

自身定位采用仿真理想定位，已知地图与标定可用于控制。准确表述是“禁止目标/背景对象真值进入非oracle在线感知与决策”，不是“系统完全不使用任何仿真状态”。

旧入口及历史进度见 [历史README](docs/README_before_mainline_20261026.md)。scripts/中的旧采集/训练/审计工具保留发展记录，部分仍使用原机器绝对路径；当前配置入口以本页及发布说明为准。
