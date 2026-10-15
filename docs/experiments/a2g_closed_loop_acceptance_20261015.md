# A2G 链路接入现有闭环：正式验收

## 结论

22 秒 CARLA 在线回合通过：原运行器 38/38 项检查通过；额外的逐条消息与控制日志审计 13/13 项通过。UAV 识别目标并发送语义消息，消息经信道到达后生成 UGV 目标航点；通信中断时已确认航点保持，恢复后新消息继续更新目标记忆。UGV 的 RGB-D 安全控制在本地传感器过期时停车，未发生碰撞。

本次是**链路接入验收**，没有用 22 秒短回合评价完整任务成功率。3–12 秒的 80 dB 衰减是可重复的诊断干预，不代表 Town10HD 实测信道。

![A2G 链路闭环验收时序图](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/closed_loop_acceptance/CI_E1_A2G_CLOSED_LOOP_T10_ZA_S2101_ACCEPTANCE_20261015T082701Z/preview/a2g_closed_loop_acceptance.png)

## 实验安排与实测结果

- 固定种子 2101；同一回合保持 500 Kb/s 速率上限和 1 秒消息 TTL。
- 0–3 秒正常传输；3–12 秒施加额外 80 dB 衰减使链路停传；12 秒后恢复。第 7–8 秒另注入一次 UGV 本地传感器过期。
- UAV 共发送 15 条目标消息：9 条在 TTL 内到达并被选用，6 条在中断期间过期。过期消息没有进入 UGV 的目标选择。
- 首条有效 UAV 消息生成 1 条目标航线。衰减期间 180 个控制 tick 均保持目标记忆航线，因单包 TTL 到期撤销航线的次数为 0。恢复后收到新的有效消息；原先过期的 6 条不会补送。
- 本地传感器过期状态持续 29 个 tick。进入停车状态 0.2 秒后，实测速度始终低于 0.001 m/s；传感器恢复后重新行驶。RGB-D 在线障碍物真值读取为 0，UGV/UAV 碰撞均为 0。
- UGV 共运行 33.15 m，其中 427 个 tick 按目标记忆航线控制。该距离和时长仅用于验证接线及安全行为。

## 复现与数据

实验配置：[ci_e1_a2g_closed_loop_acceptance_20261015.yaml](../../configs/experiments/ci_e1_a2g_closed_loop_acceptance_20261015.yaml)。运行命令：

```powershell
& 'D:\program\minicoonda\envs\carlaAir\python.exe' -u scripts\run_stage1_dynamic.py --config configs\experiments\ci_e1_a2g_closed_loop_acceptance_20261015.yaml --timeout 40
```

运行目录：[正式验收数据](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/closed_loop_acceptance/CI_E1_A2G_CLOSED_LOOP_T10_ZA_S2101_ACCEPTANCE_20261015T082701Z)。其中 `validation_report.json` 是运行器验收，`closed_loop_audit.json` 是消息、航点和安全状态的逐项审计；通信事件、目标记忆事件、航点事件和安全状态也逐条保存。

审计命令：

```powershell
& 'D:\program\minicoonda\envs\carlaAir\python.exe' scripts\audit_a2g_closed_loop.py 'E:\CarlaAirData\CityInspection_GOC\a2g_transport_20261015\closed_loop_acceptance\CI_E1_A2G_CLOSED_LOOP_T10_ZA_S2101_ACCEPTANCE_20261015T082701Z'
```

正常链路的两次 20 秒在线短测见[传输实验记录](a2g_transport_step3_experiment_20261015.md)。下一步按原顺序进行第 6 步小规模敏感性实验，之后再判断模型参数是否需要调整。
