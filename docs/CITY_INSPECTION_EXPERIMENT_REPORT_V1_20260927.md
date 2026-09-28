# 城市巡检空地协同实验报告（第一版）

**报告日期：** 2026-09-27  
**实验平台：** CARLA-Air / Town10HD  /  
**项目：** CityInspection_GOC  
**报告性质：** 阶段性工程与实验报告，不代表最终感知泛化结论

> 说明：本报告是9月27日新增的独立汇总文档。已有的《本周实验总结（2026-09-24）》保持原样，不在本报告中修改。

## 1. 本阶段目标

本阶段的目标是把前期已经完成的实验整理成一套可复核的城市巡检空地协同证据链：

1. 用S0 Oracle闭环验证通信、状态机、UGV道路规划、控制和数据记录；
2. 用UAV候选基线验证RGB+深度候选生成、颜色属性判断和几何定位；
3. 用S1感知闭环验证“UAV感知—目标级消息—UGV规划—近距离复核—安全到达”；
4. 用3个动态干扰种子和2个目标位置形成6组可比较批量结果；
5. 汇总学术图表、同步回放视频和当前问题，形成后续论文实验的基线。

## 2. 固定实验条件

| 项目 | 设置 |
|---|---|
| 地图 | Town10HD |
| 区域 | Zone A，目标位置A/B |
| 动态种子 | 1001、1002、1003 |
| UAV | 预设弓字型搜索航线，720 m，10个航点，9段航线，4次折返 |
| 传感器 | UAV/UGV同步RGB与深度，相同仿真时刻对齐 |
| 仿真时长 | S0/S1正式闭环150 s；候选基线30 s |
| 车辆控制 | CARLA道路全局路线 + 前视航点控制 + 速度比例控制 |
| UGV安全停车 | 目标附近3–7 m，停车后保持2 s |
| 干扰物 | 批量实验每组5–7辆移动汽车、4–6名移动行人 |

## 3. 方法链路

### 3.1 UAV感知

UAV使用COCO车辆检测模型生成车辆候选，再用HSV规则计算红色像素比例；将检测框中心的深度值通过针孔模型反投影到世界坐标，得到候选目标位置。候选还要满足最近5次推理至少4次命中，才允许进入消息发送门控。

当前的车辆类别只具有`van-like`启发式含义，不是专门训练的厢式车细粒度分类器。颜色判断是HSV规则，也不是学习式颜色分类网络。

### 3.2 GOC消息

UAV只发送一次目标级语义消息，而不是连续发送原始图像。消息包含目标类别/颜色语义和世界坐标；正式S1在线消息标记为`oracle=false`。

### 3.3 UGV导航与复核

UGV收到消息后将目标坐标投影到最近可驾驶车道，并使用CARLA `GlobalRoutePlanner`生成道路路线；局部控制采用前视点转向和基于速度误差的比例控制。接近目标后，UGV用自身RGB+深度进行二次确认，再进入3–7 m安全停车区间。

当前动态障碍紧急制动安全盾仍使用CARLA参与者真值，因此本阶段不能宣称已经完成纯视觉动态避障。

## 4. S0 Oracle闭环结果

正式运行：`E:\CarlaAirData\CityInspection_GOC\e1_oracle_closed_loop\formal\S0_ORACLE_T10_ZA_S1001_20260922T063718Z`

| 验收项 | 结果 |
|---|---:|
| 总体验收 | PASS |
| 共同帧数 | 1500/1500 |
| 共同帧比例 | 100% |
| UGV轨迹长度 | 84.89 m |
| UAV轨迹长度 | 720.00 m |
| 到达规划航点 | 10/10 |
| 完成航线段 | 9/9 |
| 完成折返 | 4/4 |
| UGV最终目标距离 | 5.60 m |
| UAV/UGV碰撞 | 0/0 |
| 移动车辆/行人 | 6/6 |
| 负对照 | 无消息时UGV保持不动 |

S0证明了系统在目标真值完全正确时能够完成闭环，但它不证明UAV感知有效。目标坐标直接来自CARLA真值，必须明确标记为Oracle上界实验。

## 5. 感知候选基线结果

正式运行：`E:\CarlaAirData\CityInspection_GOC\e2_candidate_baseline\CI_E2_UAV_CANDIDATE_T10_ZA_S1001_20260922T022645Z`

| 指标 | 结果 | 解释 |
|---|---:|---|
| 处理帧数 | 300 | 30 s候选实验 |
| Oracle可见目标帧 | 204 | 仅用于实验后评价 |
| 找到匹配候选帧 | 200 | 与真值距离不超过5 m |
| 可见目标召回率 | 98.04% | 目标基本能进入候选集 |
| 深度有效率 | 100% | 当前CARLA深度数据完整 |
| 中位定位误差 | 0.589 m | 米级定位 |
| P90定位误差 | 0.688 m | 90%样本低于该误差 |
| 总候选数 | 1443 | 候选负载较高 |
| 红色候选数 | 982 | 颜色规则保留的候选 |
| 红色候选精度代理 | 20.67% | 误候选仍然较多 |

该结果支持“高召回候选生成和米级几何定位”，但不支持“高精度厢式车识别”。当前瓶颈是候选过多，而不是完全看不到目标。

图表：

- [候选基线学术指标图](figures/weekly_20260924/candidate_baseline_metrics.png)
- [候选关键帧可视化](/E:/CarlaAirData/CityInspection_GOC/e2_candidate_baseline/CI_E2_UAV_CANDIDATE_T10_ZA_S1001_20260922T022645Z/visualizations/candidate_keyframes.png)

## 6. S1感知闭环结果

正式运行：`E:\CarlaAirData\CityInspection_GOC\e2_perception_closed_loop\formal\S1_PERCEPTION_T10_ZA_S1001_20260923T023836Z`

单组结果：

| 指标 | 结果 |
|---|---:|
| 状态机与闭环检查 | 36/36通过 |
| 消息定位误差 | 0.616 m |
| 错误目标消息 | 0 |
| UGV本地复核 | 成功 |
| 最终停车距离 | 5.53 m |
| UAV/UGV碰撞 | 0/0 |
| 四路传感器共同帧 | 1500/1500 |
| 无消息负对照UGV位移 | 0 m |

该结果证明目标级感知消息可以驱动UGV完成道路导航、近距离确认和安全停车。完整任务的最终完成时间主要受UAV完成720 m搜索航线限制，不能直接当作UGV响应时间。

图表与视频：

- [S1闭环结果图](figures/weekly_20260924/s1_closed_loop_summary.png)
- `E:\CarlaAirData\CityInspection_GOC\e2_perception_closed_loop\formal\S1_PERCEPTION_T10_ZA_S1001_20260923T023836Z\preview\synchronized_multiview_replay.mp4`
- `E:\CarlaAirData\CityInspection_GOC\e2_perception_closed_loop\formal\S1_PERCEPTION_T10_ZA_S1001_20260923T023836Z\preview\trajectory_map.png`

## 7. 6组批量实验结果

批量运行：`E:\CarlaAirData\CityInspection_GOC\e3_s1_multiseed_batch\S1_MULTI_SEED_3X2_20260923T125945Z`

实验设计为3个动态干扰种子 × 2个目标位置，共6组。6组全部完成，均为`PASS`，每组36/36项检查通过。

| 指标 | 均值 | 标准差 | 范围 |
|---|---:|---:|---:|
| 消息发送时刻 | 32.267 s | 31.478 s | 5.600–70.100 s |
| UGV确认时刻 | 75.100 s | 59.935 s | 19.600–138.600 s |
| UGV到达时刻 | 80.458 s | 59.897 s | 24.950–143.950 s |
| 消息定位误差 | 0.751 m | 0.174 m | 0.618–1.092 m |
| 最终停车距离 | 5.588 m | 0.532 m | 5.110–6.589 m |
| UGV实际航程 | 240.375 m | 170.276 m | 84.857–396.364 m |
| 移动车辆数 | 6.333 | 1.033 | 5–7 |
| 移动行人数 | 5.333 | 1.033 | 4–6 |

批量验收结论：

- 6/6组成功完成；
- 所有组共同帧比例为100%；
- 错误目标消息为0；
- UGV和UAV碰撞均为0；
- 所有最终停车距离均位于3–7 m安全区间；
- 位置B的发送和到达更晚，主要由于搜索可见时刻更晚且道路绕行距离更长。

图表：

- [S1批量学术指标图](figures/weekly_20260924/s1_batch_metrics.png)
- [批量分析汇总](/E:/CarlaAirData/CityInspection_GOC/e3_s1_multiseed_batch/S1_MULTI_SEED_3X2_20260923T125945Z/analysis/batch_summary.json)
- [逐组指标](/E:/CarlaAirData/CityInspection_GOC/e3_s1_multiseed_batch/S1_MULTI_SEED_3X2_20260923T125945Z/analysis/episode_metrics.csv)

## 8. 视频与可视化材料

当前已有的回放材料包括：

1. S0 Oracle空地同步回放；
2. S0无消息负对照回放；
3. UAV候选基线回放；
4. S1正式单组同步回放；
5. S1无消息负对照回放；
6. 6组批量实验的同步回放。

视频主要用于检查以下内容：UAV搜索是否持续、UGV是否由消息触发、双端RGB/深度是否同步、目标复核和停车是否发生在正确阶段。

主要视频位置：

- `E:\CarlaAirData\CityInspection_GOC\e2_candidate_baseline\CI_E2_UAV_CANDIDATE_T10_ZA_S1001_20260922T022645Z\visualizations\uav_candidate_replay.mp4`
- `E:\CarlaAirData\CityInspection_GOC\e1_oracle_closed_loop\formal\S0_ORACLE_T10_ZA_S1001_20260922T063718Z\preview\synchronized_multiview_replay.mp4`
- `E:\CarlaAirData\CityInspection_GOC\e2_perception_closed_loop\formal\S1_PERCEPTION_T10_ZA_S1001_20260923T023836Z\preview\synchronized_multiview_replay.mp4`

## 9. 当前已知问题与实验边界

1. **细粒度车辆识别不足**：COCO车辆检测加`van-like`启发式不能等价于厢式车识别。
2. **颜色候选误检较多**：红色候选精度代理为20.67%，说明HSV规则适合提高召回，不适合作为最终身份判定。
3. **动态避障尚未纯视觉化**：UGV安全盾仍读取CARLA参与者真值。
4. **统计规模有限**：当前批量只有6组，只能说明固定区域、固定航线和有限种子下的受控鲁棒性，不能推导跨地图泛化或统计显著性。
5. **尚未完成GOC消融**：还没有系统比较原图、检测框、目标状态和压缩特征在带宽、时延、丢包和能耗下的任务效用。

## 10. 本阶段结论

当前项目已经完成从平台验收到S0 Oracle闭环、UAV候选基线、S1感知闭环和6组批量实验的第一条可复核证据链：UAV可以用RGB+深度生成目标候选，目标级语义消息可以触发UGV道路规划，UGV能够完成近距离复核和安全停车，双端传感器和轨迹能够严格同步记录。

当前最准确的学术表述是：

> 在CARLA-Air Town10HD固定区域内，构建了一个由UAV RGB-深度候选感知、目标级语义消息和UGV道路规划组成的空地协同闭环。3个动态干扰种子与2个目标位置的6组实验全部完成，平均消息定位误差为0.751 m，最终停车距离为5.110–6.589 m，未出现错误目标消息或空地碰撞。结果验证了受控场景下目标级GOC接口的可行性，但尚不能代表细粒度车辆识别、纯视觉动态避障或跨场景泛化已经解决。

## 11. 可复核文件索引

- [S0 Oracle结果](S0_ORACLE_CLOSED_LOOP_RESULTS_20260922.md)
- [UAV候选基线结果](CI_E2_UAV_CANDIDATE_BASELINE.md)
- [S1感知闭环结果](S1_PERCEPTION_CLOSED_LOOP_RESULTS_20260923.md)
- [S1多种子批量结果](S1_MULTI_SEED_3X2_RESULTS_20260923.md)
- [中文版方法流程图](figures/weekly_20260924/method_pipeline.png)
- [原有本周实验总结（未修改）](WEEKLY_EXPERIMENT_SUMMARY_20260924.md)
