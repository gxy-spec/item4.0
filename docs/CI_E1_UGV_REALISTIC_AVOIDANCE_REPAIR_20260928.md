# 2026-09-28 UGV 避障场景修复与验收

## 结论

行人、横穿车辆、遮挡停车和安全条件下超车四个独立用例均通过。实时避障只使用 UGV 的 RGB-D/YOLO 观测；CARLA actor 真值仅用于仿真后的碰撞、轨迹和间距审计。

## 修复内容

- 行人根节点高度改为贴近地面，避免身体埋入路面；横穿行人完整穿越车道，UGV检测到进入冲突区域后停车，行人通过后恢复。
- 横穿车辆的朝向与横向运动方向一致，并保持到回放结束，避免倒车和中途消失。
- 遮挡用例保留静止障碍车，UGV通过前向深度走廊执行减速/停车；没有合法且空闲的同向车道时不强行绕行。
- 超车只在地图允许变道、相邻车道RGB-D检查为空且障碍物被连续跟踪时触发；变道期间缩短路径跟踪前视距离。无足够空间时保持等待。
- 专用超车用例将红色目标车停在下游路边，为静止障碍车留出合理的回并空间；其他用例的目标位置不变。
- 事后最小间距统计改为按车辆朝向计算二维包围矩形间距，不再用容易产生误读的外接圆作为主指标。

## 验收结果

| 用例 | 关键证据 | UGV碰撞 | 结果目录 |
|---|---|---:|---|
| 横穿行人 | 检测197次；停车196个仿真步；行人清空后恢复；完整穿越 | 0 | `E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260928_realistic/pedestrian/CI_E1_UGV_SAFETY_REALISTIC_PEDESTRIAN_S1001_20260928T033033Z` |
| 横穿车辆 | 检测310次；等待138步；横穿位移18 m；未中途移除 | 0 | `E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260928_realistic/crossing_vehicle/CI_E1_UGV_SAFETY_REALISTIC_CROSSING_VEHICLE_S1001_20260928T031658Z` |
| 遮挡/静止障碍 | 深度停车402步；无合法安全空隙时保持停车 | 0 | `E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260928_realistic/occlusion/CI_E1_UGV_SAFETY_REALISTIC_OCCLUSION_S1001_20260928T034757Z` |
| 安全超车 | 变道429步；随后恢复路线巡航；UGV行驶88.04 m；最小朝向包围框间距约0.775 m | 0 | `E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260928_realistic/overtake/CI_E1_UGV_SAFETY_REALISTIC_OVERTAKE_S1001_20260928T051907Z` |

四组均达到100%双端同步帧比例；行人/横穿车/遮挡用例分别保留300/300/340组RGB-D配对帧，超车用例保留1100组。超车用例的原始验收报告仍记录旧的外接圆保守间距；新朝向包围框计算基于同一份 actor 状态日志，未覆盖原始实验报告。

## 回放与复核

- 超车同步回放：`E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260928_realistic/overtake/CI_E1_UGV_SAFETY_REALISTIC_OVERTAKE_S1001_20260928T051907Z/preview/ugv_rgbd_safety_replay.mp4`
- 行人完整身体画面：`E:/CarlaAirData/CityInspection_GOC/e4_ugv_sensor_safety/20260928_realistic/pedestrian/CI_E1_UGV_SAFETY_REALISTIC_PEDESTRIAN_S1001_20260928T033033Z/sensors/ugv/rgb/00000513.png`
- 自动测试：`36 passed`。

注：超车间距按同一时刻两车朝向包围矩形计算；约0.775 m是二维几何估计，并非道路安全法规阈值。碰撞传感器记录为0，间距值用于本轮避障对比验收。
