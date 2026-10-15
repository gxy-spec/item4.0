# 空地信道与传输过程：独立验证结果

## 结论

独立验证共 20 项，全部通过。验证不启动 CARLA，也不依赖任务是否成功，而是把程序结果直接与可手算的理论值比较。

这说明当前代码正确实现了已确定的空间传播公式、SNR 与速率换算、传输队列、暂停恢复、1 秒 TTL 和到达时间逻辑。

![独立验证图](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/independent_validation_v1/independent_validation.png)

## 最关键的检查结果

- 2 GHz、100 m 时，自由空间路径损耗为 78.468 dB；距离加倍后增加 6.021 dB。程序与理论值一致。
- 同一位置由 LoS 改为 NLoS，路径损耗增加 19 dB，SNR 同时下降 19 dB。
- SNR 低于 5 dB 时停止传输；在约 7.8 dB 时达到 10 Mb/s 业务上限。
- 704 字节目标语义消息在 500 Kb/s 下需要 11.264 ms，程序结果相同。
- 人为设置一段断联后，消息先暂停、再从剩余比特继续传输，并在理论时刻 1.30 s 完成。
- 消息恰好在 1 s TTL 时刻到达会被判为过期，符合“必须在 TTL 前到达”的约定。
- 已保存的 200 个空间链路样本能够精确重放，最大数值误差为 0。

## 结果边界

本次验证回答的是“程序有没有按已定公式工作”。它不能单独证明发射功率、附加损耗等参数已经代表真实 Town10HD 无线环境；参数合理性仍需通过后续敏感性实验和论文依据说明。

完整逐项结果见 [independent_validation.json](E:/CarlaAirData/CityInspection_GOC/a2g_transport_20261015/independent_validation_v1/independent_validation.json)。验证脚本为 `scripts/verify_a2g_transport_independent.py`。
