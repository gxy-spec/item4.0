# S0 Oracle闭环正式验收

> 本阶段使用CARLA真值目标位置，`oracle=true`，`perception_enabled=false`。结果只能作为闭环系统上界和接口验收，不能作为最终感知实验结果。

## 正式结果

- 正向闭环：`E:\CarlaAirData\CityInspection_GOC\e1_oracle_closed_loop\formal\S0_ORACLE_T10_ZA_S1001_20260922T063718Z`
- 无消息负对照：`E:\CarlaAirData\CityInspection_GOC\e1_oracle_closed_loop\negative_control\S0_ORACLE_NO_MESSAGE_T10_ZA_S1001_20260922T061808Z`
- 正向状态：`PASS`；最终任务状态：`COMPLETED`。
- 负对照状态：`PASS`；预期结果：`PASS_EXPECTED_NO_MESSAGE`。

## 正向闭环结果

| 验收项 | 结果 |
|---|---:|
| 冻结指令与结构化目标 | 成功加载 |
| 状态转移 | 全部合法 |
| Oracle消息 | 发送1条、接收1条 |
| 固定通信时延 | 50 ms |
| 消息前UGV位移 | 0.00 m |
| 路线来源 | `received_oracle_message` |
| 消息接收/规划帧 | 365 / 365 |
| UGV实际路径 | 84.89 m |
| UGV最终目标距离 | 5.60 m |
| UGV最终速度 | 0.00 m/s |
| 安全停车保持 | 2.00 s |
| UAV实际路径 | 720.00 m |
| UAV航点/航段/折返 | 10 / 9 / 4 |
| UAV/UGV碰撞 | 0 / 0 |
| 四路公共帧 | 1500/1500 |
| 四路最大帧差 | 0 |
| RGB动态有效率 | UAV 100%、UGV 100% |
| 深度有限值比例 | UAV 100%、UGV 100% |

关键事件顺序：`SEARCHING → ORACLE_TARGET_VISIBLE → MESSAGE_SENT → TARGET_RECEIVED → PLANNING → NAVIGATING → ARRIVED_SAFE → UAV_ROUTE_COMPLETE → COMPLETED`。

## 无消息负对照

20秒内未生成消息、未生成路线，UGV路径与最大位移均为0.00 m，UAV正常飞行99.75 m，四路传感器200/200同帧保存。该结果证明UGV没有绕过通信接口直接访问目标位置。

## 真值使用边界

- OracleProvider执行目标可见性真值检查，并且仅在消息构造时读取一次目标位置快照。
- UGV控制器直接访问目标actor的次数为0。
- UGV路线由收到的消息坐标生成。
- CARLA真值还可由独立验收器用于最终距离评价。

正式视频、轨迹图、同步图、传感器面板和状态图均带有 `S0 ORACLE` 或“非感知结果”标识。
