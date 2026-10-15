# CARLA-Air 及其上游生态的通信建模研究核查

检索日期：2026-10-15。平台范围：本项目使用的 CARLA-Air v0.1.7，以及其上游 CARLA、AirSim 的直接相关研究。以下区分论文已公开的方法、源码实际功能与本项目建议；不把 API/RPC 本机耗时当作空口时延，也不把车车传播参数直接视为空地标定参数。

## 1. CARLA-Air 原论文和空地协同应用论文

[CARLA-Air 平台论文](https://arxiv.org/html/2603.28032)第 4.4 节的 Communication Latency 测试是 loopback 接口上的 API 调用往返耗时，如 actor transform、image capture 和 velocity command，并未给出发射功率、路径损耗、无线带宽与误包的端到端模型。这些测试支持平台接口效率的论断。

[Can Aerial VLA Models Cooperate?](https://arxiv.org/html/2605.31066)研究无通信、UGV→UAV 语义提示、双向动作耦合三个模式；有效协调时延 ECL 从状态/动作产生计至伙伴使用。本文公开方法未定义带宽、载荷发送耗时或物理误包模型。

[Air-Ground Collaborative Vision-and-Language Navigation via Shared Bird’s-Eye Maps](https://arxiv.org/html/2609.03483)第 III-B 节明确通过 shared memory 传送标注鸟瞰图及其对应 UAV 位姿。它提供了图像协作的应用先例，但没有让共享图像经历基于空间传播与载荷大小的无线传输过程。上述均为本次核查的 arXiv 版本，不能仅凭预印本页面声称已同行评审。

## 2. 关键源码发现：CARLA-Air 已继承简化 V2X 传播模块

Grimm 等，**Co-simulate no more: The CARLA V2X Sensor**，IEEE IV 2024，pp. 2429–2436，DOI：[10.1109/IV55156.2024.10588488](https://doi.org/10.1109/IV55156.2024.10588488)。本次验证了出版信息，未取得该论文全文；实现细节的依据是 [CARLA 官方传感器文档](https://carla.readthedocs.io/en/latest/ref_sensors/#v2x-sensor)与 CARLA-Air 自身源码。

CARLA-Air 核查提交：`46154110eb1a4a4db655b9431dd1cd2371aecb66`。

- [PathLossModel.cpp](https://github.com/louiszengCN/CarlaAir/blob/46154110eb1a4a4db655b9431dd1cd2371aecb66/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/V2X/PathLossModel.cpp)：场景射线查询→区分 LoS、建筑遮挡 NLoSb、车辆遮挡 NLoSv→状态相关路径损耗与随机阴影→接收功率与接收灵敏度比较。
- `geometric` 模式中，LoS 使用双径地面反射，建筑遮挡使用对数距离模型，车辆遮挡使用自由空间损耗叠加刀刃绕射损耗；另一选择是 WINNER/ETSI 相关模型。该实现面向 V2V，使用其公式前需核对空地高度、天线和适用范围。
- [CustomV2XSensor.cpp](https://github.com/louiszengCN/CarlaAir/blob/46154110eb1a4a4db655b9431dd1cd2371aecb66/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/CustomV2XSensor.cpp)：消息按仿真 tick 进入下一帧消息容器，通过传播判决后复制到接收端。此处没有载荷比特数除以有效速率的发送时间积分，也没有完整 MAC 接入、拥塞、分片重组和重传流程。
- [CustomV2XBytes.h](https://github.com/louiszengCN/CarlaAir/blob/46154110eb1a4a4db655b9431dd1cd2371aecb66/LibCarla/source/carla/rpc/CustomV2XBytes.h)：自定义载荷缓冲区为 **100 字节**。本机已安装 Python 客户端也存在 V2X/CAM 类型，调用 `carla.CustomV2XBytes().max_data_size()` 返回 **100**。本次未能连接本机 2000 端口的服务端，因此未完成运行时 blueprint/无人机挂载测试。

这个功能可以作为几何遮挡与小消息收包判决的参考，但不能直接作为 MB 图像、KB/MB 特征及语义消息的统一传输器。参数化接收灵敏度也不等于已经建模热噪声、同频干扰和具体 MCS 对应的误块率。

## 3. CARLA 应用论文中的可控消息退化

Justo 等，**LFF-V2V: A Late Fusion Cooperative Framework in V2V Scenarios**，[项目方提供的 IV 2025 稿件](https://www.events-project.eu/wp-content/uploads/2025/06/IV25_L1.pdf)，第 III-B、IV 节。

该研究在 CARLA Town03 和 V2X-Real 数据上测试后期融合。通信模块把检测/跟踪对象转换为 CAM/CPM 和 ROS 2 消息；V2X 条件显式加入 **100 ms 延迟**与 **0.1 的消息丢失率**。这是消息层故障注入，可以隔离感知融合对陈旧信息与丢失的敏感性；该实验设置并不从距离、传播损耗和 SNR 推导丢包概率，也未证明不同大小载荷的排队性能。

## 4. CARLA + ns-3：由网络协议仿真决定分组交付

Risma Carletti、Casetti、Härri、Risso，**ms-van3t-CARLA: An Open-Source Co-Simulation Framework for Cooperative Perception Evaluation**，WONS 2024，pp. 93–96，DOI：[10.23919/WONS60642.2024.10449533](https://doi.org/10.23919/WONS60642.2024.10449533)，[作者机构全文](https://iris.polito.it/retrieve/88414ddc-f5cf-479b-adec-9dececb66fec/ms_van3t_CARLA__a_co_simulation_framework_for_realistic_cooperative_perception_evaluation.pdf)。

其分工是：CARLA 产生移动与传感器观测，OpenCDA 执行感知融合，ns-3/ms-van3t 承担无线网络。每个 CARLA 连接车辆映射到 ns-3 节点，利用 Protobuf/gRPC 同步位置和数据；检测结果编码成标准 CAM/CPM，通过配置的网络接口发送，接收结果进入本地动态地图再同步回 OpenCDA。可选网络技术包括 IEEE 802.11p、LTE、LTE-V2X 和 NR-V2X，具体协议必须在配置中选择。这个架构允许研究分组接收率等网络效应，不能把“支持某技术”等同于每次实验都使用该技术。

对本项目的价值是用独立网络模块产生消息到达事件，再交给 UGV 决策；局限是论文案例为地面车辆，移植仍需定义 UAV 节点、空地传播模型及时间同步。

## 5. CARLA + Sionna：用相同几何场景产生多径信道

Gharsallah 与 Kaddoum，**MVX-ViT: Multimodal Collaborative Perception for 6G V2X Network Management Decisions Using Vision Transformer**，IEEE Open Journal of the Communications Society，2024，5:5619–5634，DOI：[10.1109/OJCOMS.2024.3452591](https://doi.org/10.1109/OJCOMS.2024.3452591)，[作者机构记录](https://espace2.etsmtl.ca/id/eprint/29674/)。其 MVX 框架将 CARLA 与 Sionna 连接，联合生成多模态观测和无线数据。

同作者后续 **Multimodal Collaborative Perception for Dynamic Channel Prediction in 6G V2X Networks**，IEEE Transactions on Machine Learning in Communications and Networking，2025，3:725–743，DOI：[10.1109/TMLCN.2025.3578577](https://doi.org/10.1109/TMLCN.2025.3578577)，[作者机构全文](https://espace2.etsmtl.ca/id/eprint/33319/1/Kaddoum-G-2025-33319.pdf)，第 III、V、VI-D 节给出了更具体的过程：在 Sionna 中匹配 CARLA 场景及节点位置，用多径的复增益与传播时延构造 CIR/CFR，并用于 MIMO-OFDM 链路仿真；16QAM 及检测/信道估计方法进一步影响 BER。该链路级分析与完整的分组网络、任务 TTL 实验仍是不同层面。

这是当前“中心连线遮挡 + 经验损耗”的一种可选高精度对照，额外工作包括场景导出、坐标对齐、RF 材料/天线定义、动态物体同步和射线计算；视觉几何一致并不自动代表射频材料已标定。

## 6. 更接近无人机任务的上游 AirSim 研究：CAVIAR

Borges、Bastos、Correa、Batista、Klautau，**CAVIAR: Co-Simulation of 6G Communications, 3-D Scenarios, and AI for Digital Twins**，IEEE Internet of Things Journal，2024，11(19):31287–31300，DOI：[10.1109/JIOT.2024.3418675](https://doi.org/10.1109/JIOT.2024.3418675)，[公开全文](https://arxiv.org/html/2401.03310)，[作者代码](https://github.com/lasseufpa/caviar)。

搜救案例中 AirSim 负责飞行和视觉、Sionna 负责传播、YOLOv8 负责目标检测，NATS 负责模块消息交换与编排。相同三维场景简化后供射线追踪；无人机更新位置时同步无线节点，并按虚拟时间步暂停/交换结果。案例设置为 **40 GHz、8×8 发射阵列、2×2 接收阵列、ITU 材料、最多 5 次射线反弹，启用绕射和散射**。这些是该案例的假设，不能直接替换本项目 2 GHz 链路。论文把 Sionna 与 ns-3/5G-LENA 联合的端到端仿真列为后续工作，故不能称其已完成整个协议栈。

## 7. 代码项目与近期预印本的核查边界

[FICSAgenticDroneSim](https://github.com/AkbasLab/FICSAgenticDroneSim)自称支持 CARLA-Air/AirSim。核查提交 `34e69d0754cd1242f106eb8f61c0dc43929e6183` 的 [message_bus.py](https://github.com/AkbasLab/FICSAgenticDroneSim/blob/34e69d0754cd1242f106eb8f61c0dc43929e6183/agentic_uav/coordination/message_bus.py)实际提供固定延迟、参数化随机丢失、TTL 和消息日志；但 [network_model.py](https://github.com/AkbasLab/FICSAgenticDroneSim/blob/34e69d0754cd1242f106eb8f61c0dc43929e6183/agentic_uav/coordination/network_model.py)仍为返回 True 的占位实现。README 宣称的高级网络功能不能仅据文字当作全部已验证的实现，更不是实测无线模型。

2026 年预印本 [World Model-Enabled Causal Digital Twins for Semantic Communications in Physical AI Systems](https://arxiv.org/html/2605.16547)使用 AirSim-Sionna 槽级耦合与语义 token 比特预算研究导航，选题与本项目相关。其仿真节描述了几何、损耗、衰落与噪声，但本次所读版本没有给出足以直接复现空地链路的完整频率、功率、带宽、MCS/误包参数集，故暂不作为本项目参数标定依据。

## 8. 对本项目的实施判断

现有 A2G 空间模型有明确的同生态方法先例：从同步几何产生传播状态和接收预算。第 3 步还需增加实际字节载荷、随时间变化的有效服务速率、发送队列、接收处理、过期与丢包原因。应保存消息生成→入队→开始发送→完成→接收/丢弃的仿真时间事件。

建议先按该模块分工完成可解释的空地传输过程，再根据论文论断选择验证强度：研究任务级目标导向载荷选择可用独立消息服务；需要比较 802.11/NR 等协议与竞争效应时再使用 ns-3；需要声称场景特定多径、材料传播精度时选取少量轨迹使用 Sionna 对照。每条路线都需说明适用范围。上述文献说明架构的可行性，不能单独证明当前 17–41 dB SNR 已被 Town10HD 真实射频验证。
