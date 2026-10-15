# E1-C 单回合：加入物理信道后的完整轨迹与同步回放

选取原 36 组对照设计中的 **E1-C**（Town10HD 东区场景 E1、UAV+UGV 协同 C 组、种子 2101），使用修复后的目标记忆和已冻结的 A2G 物理信道 v1 重新运行完整 **190 秒**。正式运行器结果为 **PASS**。

![E1-C 空地轨迹与信道时间线](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/full_replay_e1c/CI_E1_A2G_FULL_REPLAY_E1_C_S2101_20261015T114444Z/preview/a2g_full_task_process.png)

## 如何看回放

[打开 190 秒五视图同步回放（MP4）](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/full_replay_e1c/CI_E1_A2G_FULL_REPLAY_E1_C_S2101_20261015T114444Z/preview/synchronized_multiview_replay.mp4)。左侧显示完整 UAV/UGV 轨迹与当前所在位置；右侧依次是 UAV RGB、UAV 深度、UGV RGB、UGV 深度。顶部标出本回合的仿真相对时间、当前 SNR、服务速率及 UAV 消息有效到达数。地图坐标来自 CARLA 世界坐标，候选框中的真值近邻标签仅用于离线看图，未进入在线控制。

视频每秒播放 2 帧，对应仿真过程中**每 0.5 秒保存的同源传感器帧**。共 380 帧、1920×1080、190 秒；四路保存图像及地图位置的 CARLA 帧号完全相同，最大帧号差为 0。完整回合 3800 个固定步长中有 3799 个四路共同帧。视频是固定频率同步采样回放，0.5 秒之间没有补帧。

## 这一轮实际发生了什么

- 首条 UAV 目标消息在约第 0.55 秒到达，生成 1 条目标路线。全程 32 条 UAV 消息全部在 1 秒 TTL 内有效到达，无过期消息；参考链路速率保持在 10 Mb/s 上限。
- UAV 飞行约 426.0 m，UGV 行驶约 104.8 m。UGV 与目标的平面距离由约 94.0 m 缩至约 10.9 m。图中的目标真实位置和距离只用于仿真后评价。
- 约第 75 秒后，UGV 因本地 RGB-D 避障判断为 `wait_no_safe_overtake_gap`，停在前方红色车辆后等待。此状态在全回合占 2297 个控制 tick。UGV/UAV 均无碰撞，在线目标和障碍物真值读取均为 0。

因此，**本次“PASS”表示完整实验回合、通信接入、目标航线与安全验收通过；不表示 UGV 已抵达目标**。10.9 m 的停靠距离主要受本地安全决策限制。完整 E1-C 轨迹全程为几何 LoS（SNR 约 27.3–45.4 dB），这段视频适合看正常参考信道下的任务过程；遮挡与断联行为见独立诊断实验。

## 结果文件

- [正式验收报告](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/full_replay_e1c/CI_E1_A2G_FULL_REPLAY_E1_C_S2101_20261015T114444Z/validation_report.json)
- [视频同步与播放信息](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/full_replay_e1c/CI_E1_A2G_FULL_REPLAY_E1_C_S2101_20261015T114444Z/preview/synchronized_multiview_replay.json)
- [轨迹与信道图 PDF](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/full_replay_e1c/CI_E1_A2G_FULL_REPLAY_E1_C_S2101_20261015T114444Z/preview/a2g_full_task_process.pdf)
- [本次实验配置](../../configs/experiments/ci_e1_a2g_full_replay_e1c_20261015.yaml)
