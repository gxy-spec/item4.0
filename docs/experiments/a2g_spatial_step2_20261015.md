# 空地信道第 2 步：Town10HD 空间链路诊断（2026-10-15）

## 结论与范围

已将理论定稿中的空间几何、几何 LoS/NLoS、概率视距对照、自由空间路径损耗、状态附加损耗、空间相关阴影衰落和接收 SNR 落地为可复现的**只读空间链路**。它在 CARLA-Air 回合中按同一仿真帧读取 UAV/UGV 的天线位置，通过 `world.cast_ray` 判定指定静态遮挡类别，并保存逐样本结果。两组在同一条实际轨迹上计算；主组使用 Town10HD 几何状态，对照组使用仰角概率状态。现有目标消息的固定时延送达、TTL、路线和 RGB-D 安全控制均没有调用新模型。本实验不评价吞吐、排队、传输时延或丢包；这些属于后续第 3 步。

20 秒 Town10HD 诊断回合通过原有兼容性检查及新增的空间链路检查。200 个 10 Hz 空间样本均有有限数值；几何状态出现 LoS 与 NLoS 两类；独立重算与保存文件逐值一致。此处 PASS 说明实现与仿真数据一致，**不代表射频参数已被 Town10HD 实测标定**。

## 实现与固定参数

- 实现：[a2g_spatial.py](../../src/communication/a2g_spatial.py)；runner 只读接入：[run_stage1_dynamic.py](../../scripts/run_stage1_dynamic.py)。
- 参数快照：[a2g_spatial_v1.yaml](../../configs/communication/a2g_spatial_v1.yaml)；回合配置：[ci_e1_a2g_spatial_smoke_20261015.yaml](../../configs/experiments/ci_e1_a2g_spatial_smoke_20261015.yaml)。
- 天线坐标使用 actor 的局部安装偏移经完整 CARLA 变换转换到世界坐标：UAV `(0,0,0.2)` m、UGV `(0,0,1.6)` m。两者是安装高度假设，并非实测相位中心。
- 静态遮挡类别固定为 `Buildings, Walls, Bridge, Terrain, Ground, Static`。动态车辆和行人不据视觉标签直接等同于完全射频遮挡。射线返回的首次命中类别及总命中数均留在原始轨迹中。
- 载波 2 GHz，LoS/NLoS 平均附加损耗 1/20 dB，作为 [Al-Hourani 等 2014 年 GLOBECOM 论文 Table II 的 2 GHz Urban 参考组参数](https://www.researchgate.net/profile/Akram-Al-Hourani/publication/263582265_Modeling_Air-to-Ground_Path_Loss_for_Low_Altitude_Platforms_in_Urban_Environments/links/53e355be0cf2187dccf38884/Modeling-Air-to-Ground-Path-Loss-for-Low-Altitude-Platforms-in-Urban-Environments.pdf)。该论文聚焦的飞行高度约 200–3000 m；本回合 UAV 高度约 40 m。因此这些取值只能作仿真参考，不能写成 Town10HD 或当前设备的标定值。
- 概率视距对照采用城市参考值 `a=9.61, b=0.16`，不是从 Town10HD 拟合的。发射功率 20 dBm、噪声带宽 10 MHz、噪声系数 7 dB、零天线增益及零系统损耗均为显式场景假设。阴影标准差 LoS/NLoS 为 4/8 dB，相关距离各 50 m；潜在视距场相关距离为 50 m。这些值已列出敏感性扫描范围，尚未进行扫描。
- 随机场在观测到的联合 UAV/UGV 位置上一次性生成，协方差按定稿的 `exp(-D/d_corr)` 构造；LoS、NLoS 阴影场和概率组潜在高斯场使用独立随机子种子。相同轨迹和种子得到相同结果；相同联合位置复用同一随机值。当前离线联合采样适用于**固定轨迹诊断**，不是在线消息发送器。

## 运行和结果

运行时启动 Town10HD CARLA-Air，然后执行：

```powershell
& 'D:\program\minicoonda\envs\carlaAir\python.exe' -u scripts\run_stage1_dynamic.py --config configs\experiments\ci_e1_a2g_spatial_smoke_20261015.yaml
```

通过回合：`E:\CarlaAirData\CityInspection_GOC\a2g_spatial_20261015\smoke\CI_E1_A2G_SPATIAL_T10_ZA_S1001_SMOKE_20261015T021646Z`。主要产物为 `communication/spatial_link_trace.csv`、`communication/spatial_link_summary.json`、`communication/spatial_link_verification.json`，以及新版 `preview/spatial_geometry_v2.png/.pdf` 和 `preview/spatial_propagation_v2.png/.pdf`；早期 `preview/spatial_link_trace.png` 仅供版本溯源。`resolved_config.yaml` 保存了所有参数。无需 CARLA 即可从轨迹重画：

```powershell
& 'D:\program\minicoonda\envs\carlaAir\python.exe' scripts\plot_a2g_spatial.py 'E:\CarlaAirData\CityInspection_GOC\a2g_spatial_20261015\smoke\CI_E1_A2G_SPATIAL_T10_ZA_S1001_SMOKE_20261015T021646Z'
```

验证命令：

```powershell
& 'D:\program\minicoonda\envs\carlaAir\python.exe' scripts\verify_a2g_spatial_run.py 'E:\CarlaAirData\CityInspection_GOC\a2g_spatial_20261015\smoke\CI_E1_A2G_SPATIAL_T10_ZA_S1001_SMOKE_20261015T021646Z'
```

该轨迹的 3D 距离为 53.49–126.75 m，仰角为 17.47–46.15°。几何组有 58 个样本被 `Buildings` 遮挡，LoS 占 71%；概率组的单种子 LoS 占 58.5%。几何组 SNR 为 19.17–41.05 dB，概率组为 17.31–41.05 dB。概率比例只是这一固定轨迹的一次随机实现，不能当作模型概率的统计校准；两组任务效果也未在此比较。

图示解读、问题边界与后续验证见 [a2g_spatial_result_review_20261015.md](a2g_spatial_result_review_20261015.md)。

独立验证结果 `PASS`：200/200 样本完整、时间与帧号递增、遮挡标签与二元状态一致、全部可重算数值的最大差值为 0。项目测试为 84/84 通过。初次 5 秒运行和一次重启前的 20 秒运行留在同一数据根目录，分别触发了旧 E1 的 RGB 动态性检查和传感器健康检查；本次空间诊断配置仅将与空间链路无关的 RGB 唯一帧比例从验收门槛中排除，其余 E1 检查保持执行，另加入空间样本覆盖、数值有限性及遮挡状态转换检查。

## 下一阶段接口和限制

第 3 步应读取同一仿真时间下的 SNR，并在独立消息服务层计算有效速率、排队、传输进度、接收处理和 TTL。现阶段 `spatial_link_trace.csv` 仅供诊断、参数敏感性分析与固定轨迹回放，不应用于提前读取真实遮挡状态来控制发送策略。正式论文参数和任务级比较前，仍需进行传播参数敏感性实验或取得实测/可信射频标定依据。
