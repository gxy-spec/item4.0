# 空地物理信道 v1：最小规模敏感性实验与冻结

## 结论

**本阶段通过并冻结为仿真参考版本 v1。** 空间传播、有效速率、排队和 1 秒 TTL 的独立验证为 20/20；最小敏感性实验的 21 个配对条件全部完成；修复版 `persistent_target_memory` 的 190 秒 E1-C 正式回合通过，额外日志审计为 16/16。

冻结的是当前仿真模型、参数和语义消息闭环的可复现实验基准。Town10HD 参数仍是有出处的参考设定，没有进行实测无线链路标定；特征和原始图像仍是**按字节数构造的占位载荷**，其结果只说明链路承载能力，不能作为对应感知算法的任务效果。

![最小规模敏感性实验](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/minimal_sensitivity_v1/a2g_minimal_sensitivity.png)

## 做了什么

固定一条 20 秒 UAV–UGV 空间轨迹，使用 1001、2001、3001 三个配对信道种子。每个条件同时计算几何射线链路和仰角概率链路，共得到 **7 个单因素条件 × 3 个种子 = 21 组；两类链路合计 42 行结果**。变化量为速率上限 0.5/2/10 Mb/s、额外衰减 0/10/15 dB，以及在 15 dB 衰减下的停传门限 0/5/10 dB。其他参数固定。

目标语义载荷取自修复版 E1-C 已保存的 UAV 消息，经当前线上序列化器计算，每条 683–704 字节。特征和 RGB 图像分别设为 262,144 字节与 1,440,000 字节的占位探针。每个条件发送 40 条语义、2 条特征占位探针及 1 条图像占位探针，统计 **1 秒 TTL 内有效到达**的比例。

图 (a) 表明：在本固定轨迹上，真实目标语义消息在三个速率上限下均 100% 有效到达；特征占位载荷在 10 Mb/s 基准条件下 100%，在 0.5 和 2 Mb/s 下为 0%；图像占位载荷在三档均为 0%。图 (b) 表明额外衰减由 0 增至 10、15 dB 后，语义有效到达率由 100% 降至 97.5%、90.4%。图 (c) 表明在 15 dB 衰减条件下，将停传门限由 0 升至 5、10 dB，语义有效到达率依次为 97.5%、90.4%、82.9%。图中点为六条“种子 × 链路模型”结果的均值，误差线为最小值至最大值。上述响应符合预期单调性，同种子重放完全一致，并观测到了零速率、可变速率、有效到达及过期。

## 完整任务回合

在 Town10HD 中用修复版 `persistent_target_memory` 配置执行了 190 秒 E1-C，物理链路基准速率上限为 10 Mb/s。**运行器 PASS，额外审计 16/16 PASS**。UAV 31 条语义消息全部在 TTL 内到达，平均有效消息时龄为 36.0 ms；首条 UAV 消息生成了目标路线，UGV 按该路线控制 3787 个 tick。最终距目标 10.857 m，与修复版原基线的 10.866 m 相差约 0.009 m。UGV/UAV 碰撞均为 0，本地 RGB-D 安全控制持续工作，在线目标及障碍物真值读取均为 0。

本完整任务轨迹全程为几何 LoS，因此不把“同一回合必须出现 LoS/NLoS 切换”列为任务验收条件。遮挡变化和断联恢复由前面的固定轨迹验证及 22 秒闭环诊断回合覆盖。完整回合运行器内的该检查仍显示旧的“short transport-interface smoke”说明文字；实际配置明确关闭了这一场景覆盖要求。完整回合的其余验收项均通过。

## 冻结范围与后续使用

冻结清单记录了空间模型、传输模型、修复后的目标记忆、运行脚本、实验配置及原始结果的 SHA-256。后续多层级载荷和信息价值策略实验应以此版本为通信基线；若调整物理参数、速率映射、TTL 或记忆策略，应建立新版本，并重跑对应验收。

这组 20 秒重放和单个 190 秒任务回合**不能推断跨场景任务成功率，也不能证明特征或图像级消息已真正接入接收端**。后续实现真实编码、解码和下游识别后，需要另做任务级配对对照。信道参数的真实环境代表性也需进一步校准；本阶段可支持的是仿真模型行为和参考参数下的可复现性。

## 数据与复现

- [冻结清单与哈希](../../configs/communication/a2g_channel_v1_freeze_20261015.json)
- [敏感性汇总](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/minimal_sensitivity_v1/sensitivity_summary.json)、[逐条件结果](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/minimal_sensitivity_v1/case_metrics.csv)、[PDF 图](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/minimal_sensitivity_v1/a2g_minimal_sensitivity.pdf)
- [完整任务验收](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/frozen_candidate_e1c/CI_E1_A2G_FROZEN_CANDIDATE_E1C_S2101_20261015T091028Z/validation_report.json)、[额外审计](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/frozen_candidate_e1c/CI_E1_A2G_FROZEN_CANDIDATE_E1C_S2101_20261015T091028Z/freeze_candidate_audit.json)
- [独立验证](a2g_independent_validation_20261015.md)、[短回合闭环验收](a2g_closed_loop_acceptance_20261015.md)

完整回合配置为 `configs/experiments/ci_e1_a2g_frozen_candidate_e1c_20261015.yaml`；运行脚本为 `scripts/run_stage1_dynamic.py`。敏感性重放及额外审计分别由 `scripts/run_a2g_minimal_sensitivity.py` 和 `scripts/audit_a2g_freeze_candidate.py` 执行。三项脚本均保留在仓库中。
