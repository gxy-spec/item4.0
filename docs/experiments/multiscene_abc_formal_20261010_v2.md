# Town10HD 多场景配对 A/B/C 正式实验报告（2026-10-10）

## 目的与实验边界

在 Town10HD 的北区、东区、西区和整图范围内，验证目标信息来源是否改变 UGV 巡检决策。共 12 个场景实例，每个场景用相同种子、目标/干扰物、起终点、UAV 航线、传感器与仿真时长分别运行 A/B/C，共 36 回合。

- A：UGV 按预设巡检路线运行，不使用目标候选消息。
- B：UGV 仅使用自身 RGB-D 产生的本地候选。
- C：UAV 与 UGV 候选经语义消息选择后协同；UGV 的 RGB-D 安全避障始终保留。
- 本批次仍是理想语义通信：固定时延/TTL，不含带宽竞争、衰落和物理层随机丢包；因此不是物理信道 GOC 验证。CARLA 真值只用于仿真后评价、地图和候选框着色，不进入在线感知、选目标、规划或避障。
- 传感器图像采用 compact 保存：每 100 个固定步长 tick 保存 1 个四流同步样本，约每 5 s 一帧；轨迹、状态、消息与同步索引按仿真过程完整记录。回放视频是这些已保存帧的稀疏同步回放，不是连续录像。

## 场景覆盖与正式验收

| 区域 | 场景实例 / 种子 | 单回合仿真时长 | UAV 规划航线长度 |
|---|---|---:|---:|
| 北区 | N1, N2, N3 / 1101, 1102, 1103 | 180 s, 180 s, 180 s | 386 m, 386 m, 386 m |
| 东区 | E1, E2, E3 / 2101, 2102, 2103 | 190 s, 190 s, 190 s | 426 m, 426 m, 426 m |
| 西区 | W1, W2, W3 / 3101, 3102, 3103 | 210 s, 210 s, 210 s | 480 m, 480 m, 480 m |
| 整图 | F1, F2, F3 / 4101, 4102, 4103 | 450 s, 450 s, 450 s | 1120 m, 1120 m, 1120 m |

**验收结论：36/36 原生仿真 PASS，36/36 五视图回放 PASS。** 原生检查失败项 0；四路共同帧比例最低 99.972%；UGV/UAV 碰撞各 0/0；每回合至少有 2 辆运动干扰车和 2 名运动行人。回放帧数范围 36–90，四路源帧 spread 全为 0，视频总计约 110.9 MB。

回放把每个保存时刻的全局轨迹图、UAV RGB/深度、UGV RGB/深度放在同一画面中，四路使用同一 CARLA frame ID。视频为 2 fps 展示稀疏保存帧，画面标注实际仿真时间并声明中间时段没有保存/插帧；它适合核对同步观测与候选框，不可据此判断 5 s 间隔内的瞬态动作。

## 正式结果

指标定义：‘接近量’=初始 UGV–目标距离−最终距离，正值表示最终更近；‘路线偏离’为 UGV 轨迹各采样点到预设巡检折线的平均 2D 距离；‘≤10 m’按回合结束时距离统计。所有组间比较是 12 个配对场景上的描述性统计，不是显著性检验。

| 组别 | n | 平均接近量 | 中位接近量 | 平均最终目标距离 | 结束时 ≤10 m | 平均路线偏离 | 目标路线规划 |
|---|---:|---:|---:|---:|---:|---:|---:|
| A 固定巡检 | 12 | 14.8 m | 10.8 m | 102.0 m | 0/12 | 0.3 m | 0（UAV 0，UGV 0） |
| B UGV 本地感知 | 12 | 40.7 m | 48.9 m | 76.0 m | 0/12 | 19.1 m | 8（UAV 0，UGV 8） |
| C UAV+UGV 协同 | 12 | 56.5 m | 62.8 m | 60.2 m | 1/12 | 36.4 m | 12（UAV 6，UGV 6） |

| 配对比较 | 接近量平均增益 | 中位增益 | 更近 / 更远 / 持平场景数 |
|---|---:|---:|---:|
| B-A | 26.0 m | 23.0 m | 8 / 2 / 2 |
| C-A | 41.8 m | 40.8 m | 11 / 1 / 0 |
| C-B | 15.8 m | 5.4 m | 9 / 2 / 1 |

**结果解释。** A 组平均路线偏离仅 0.34 m 且目标路线规划为 0，符合固定巡检基线；B 组 12 个场景中有 8 个生成目标路线，均由 UGV 本地候选触发；C 组 12 个都生成目标路线，其中 6 个由 UAV 候选、6 个由 UGV 候选触发。相对 B，C 的平均接近量增加 15.79 m，中位增加 5.43 m，12 个配对场景中 C 更近 9 个、更远 2 个、持平 1 个，说明协同信息整体有增益，但并非每个场景都更好。安全规则仍可能阻止进一步靠近：结束时只有 C 的 1/12 回合进入 10 m 内，不能将本批次表述为普遍完成近距离到达任务。

C 组相对 B 组的增益因场景差异明显：例如有些场景两组最终路线几乎相同，而另一些场景 C 明显更接近目标；也存在 C 选到不利信息/路线后不如 B 的个例。应把该结果解释为多场景下的系统级趋势和异质性，而不是声称 UAV 一定带来增益。

### 逐场景结果、轨迹图与同步回放

下表‘C−B 接近增益’为 C 组接近量减 B 组接近量，正值表示 C 最终更接近目标。每个场景图左侧叠加 UAV 航线及 A/B/C 三条 UGV 实际轨迹，右侧为 UGV–目标距离随时间变化；A/B/C 五视图视频与原生验收 JSON 均逐组链接。

| 场景 / 种子 | 时长 | 最终目标距离 A / B / C | C−B 接近增益 | UAV/UGV 轨迹与距离 | 同步回放 A / B / C | 原生验收 A / B / C |
|---|---:|---:|---:|---|---|---|
| N1 / 1101 | 180 s | 44.2 / 12.9 / 12.9 m | 0.0 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/N1_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N1/A/CI_E7_MULTISCENE_FORMAL_N1_A_20261010T023740Z/preview/N1_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N1/B/CI_E7_MULTISCENE_FORMAL_N1_B_20261010T024134Z/preview/N1_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N1/C/CI_E7_MULTISCENE_FORMAL_N1_C_20261010T024526Z/preview/N1_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N1/A/CI_E7_MULTISCENE_FORMAL_N1_A_20261010T023740Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N1/B/CI_E7_MULTISCENE_FORMAL_N1_B_20261010T024134Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N1/C/CI_E7_MULTISCENE_FORMAL_N1_C_20261010T024526Z/validation_report.json) |
| N2 / 1102 | 180 s | 43.7 / 10.9 / 10.8 m | 0.0 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/N2_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N2/A/CI_E7_MULTISCENE_FORMAL_N2_A_20261010T024918Z/preview/N2_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N2/B/CI_E7_MULTISCENE_FORMAL_N2_B_20261010T025316Z/preview/N2_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N2/C/CI_E7_MULTISCENE_FORMAL_N2_C_20261010T025702Z/preview/N2_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N2/A/CI_E7_MULTISCENE_FORMAL_N2_A_20261010T024918Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N2/B/CI_E7_MULTISCENE_FORMAL_N2_B_20261010T025316Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N2/C/CI_E7_MULTISCENE_FORMAL_N2_C_20261010T025702Z/validation_report.json) |
| N3 / 1103 | 180 s | 45.4 / 47.7 / 6.9 m | 40.7 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/N3_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N3/A/CI_E7_MULTISCENE_FORMAL_N3_A_20261010T030054Z/preview/N3_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N3/B/CI_E7_MULTISCENE_FORMAL_N3_B_20261010T030452Z/preview/N3_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N3/C/CI_E7_MULTISCENE_FORMAL_N3_C_20261010T030846Z/preview/N3_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N3/A/CI_E7_MULTISCENE_FORMAL_N3_A_20261010T030054Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N3/B/CI_E7_MULTISCENE_FORMAL_N3_B_20261010T030452Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/N3/C/CI_E7_MULTISCENE_FORMAL_N3_C_20261010T030846Z/validation_report.json) |
| E1 / 2101 | 190 s | 54.0 / 53.9 / 10.9 m | 43.1 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/E1_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E1/A/CI_E7_MULTISCENE_FORMAL_E1_A_20261010T031247Z/preview/E1_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E1/B/CI_E7_MULTISCENE_FORMAL_E1_B_20261010T031654Z/preview/E1_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E1/C/CI_E7_MULTISCENE_FORMAL_E1_C_20261010T032119Z/preview/E1_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E1/A/CI_E7_MULTISCENE_FORMAL_E1_A_20261010T031247Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E1/B/CI_E7_MULTISCENE_FORMAL_E1_B_20261010T031654Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E1/C/CI_E7_MULTISCENE_FORMAL_E1_C_20261010T032119Z/validation_report.json) |
| E2 / 2102 | 190 s | 80.5 / 80.5 / 92.6 m | -12.0 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/E2_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E2/A/CI_E7_MULTISCENE_FORMAL_E2_A_20261010T032535Z/preview/E2_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E2/B/CI_E7_MULTISCENE_FORMAL_E2_B_20261010T032933Z/preview/E2_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E2/C/CI_E7_MULTISCENE_FORMAL_E2_C_20261010T023300Z/preview/E2_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E2/A/CI_E7_MULTISCENE_FORMAL_E2_A_20261010T032535Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E2/B/CI_E7_MULTISCENE_FORMAL_E2_B_20261010T032933Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E2/C/CI_E7_MULTISCENE_FORMAL_E2_C_20261010T023300Z/validation_report.json) |
| E3 / 2103 | 190 s | 86.3 / 26.8 / 76.2 m | -49.4 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/E3_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E3/A/CI_E7_MULTISCENE_FORMAL_E3_A_20261010T033355Z/preview/E3_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E3/B/CI_E7_MULTISCENE_FORMAL_E3_B_20261010T033820Z/preview/E3_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E3/C/CI_E7_MULTISCENE_FORMAL_E3_C_20261010T034300Z/preview/E3_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E3/A/CI_E7_MULTISCENE_FORMAL_E3_A_20261010T033355Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E3/B/CI_E7_MULTISCENE_FORMAL_E3_B_20261010T033820Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/E3/C/CI_E7_MULTISCENE_FORMAL_E3_C_20261010T034300Z/validation_report.json) |
| W1 / 3101 | 210 s | 73.1 / 21.7 / 10.9 m | 10.8 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/W1_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W1/A/CI_E7_MULTISCENE_FORMAL_W1_A_20261010T034740Z/preview/W1_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W1/B/CI_E7_MULTISCENE_FORMAL_W1_B_20261010T035244Z/preview/W1_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W1/C/CI_E7_MULTISCENE_FORMAL_W1_C_20261010T035748Z/preview/W1_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W1/A/CI_E7_MULTISCENE_FORMAL_W1_A_20261010T034740Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W1/B/CI_E7_MULTISCENE_FORMAL_W1_B_20261010T035244Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W1/C/CI_E7_MULTISCENE_FORMAL_W1_C_20261010T035748Z/validation_report.json) |
| W2 / 3102 | 210 s | 73.3 / 10.9 / 10.9 m | 0.0 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/W2_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W2/A/CI_E7_MULTISCENE_FORMAL_W2_A_20261010T040252Z/preview/W2_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W2/B/CI_E7_MULTISCENE_FORMAL_W2_B_20261010T040750Z/preview/W2_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W2/C/CI_E7_MULTISCENE_FORMAL_W2_C_20261010T041257Z/preview/W2_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W2/A/CI_E7_MULTISCENE_FORMAL_W2_A_20261010T040252Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W2/B/CI_E7_MULTISCENE_FORMAL_W2_B_20261010T040750Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W2/C/CI_E7_MULTISCENE_FORMAL_W2_C_20261010T041257Z/validation_report.json) |
| W3 / 3103 | 210 s | 95.3 / 95.3 / 75.1 m | 20.2 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/W3_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W3/A/CI_E7_MULTISCENE_FORMAL_W3_A_20261010T041758Z/preview/W3_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W3/B/CI_E7_MULTISCENE_FORMAL_W3_B_20261010T042250Z/preview/W3_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W3/C/CI_E7_MULTISCENE_FORMAL_W3_C_20261010T042748Z/preview/W3_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W3/A/CI_E7_MULTISCENE_FORMAL_W3_A_20261010T041758Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W3/B/CI_E7_MULTISCENE_FORMAL_W3_B_20261010T042250Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/W3/C/CI_E7_MULTISCENE_FORMAL_W3_C_20261010T042748Z/validation_report.json) |
| F1 / 4101 | 450 s | 183.4 / 121.8 / 121.8 m | 0.0 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/F1_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F1/A/CI_E7_MULTISCENE_FORMAL_F1_A_20261010T043240Z/preview/F1_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F1/B/CI_E7_MULTISCENE_FORMAL_F1_B_20261010T044312Z/preview/F1_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F1/C/CI_E7_MULTISCENE_FORMAL_F1_C_20261010T045341Z/preview/F1_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F1/A/CI_E7_MULTISCENE_FORMAL_F1_A_20261010T043240Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F1/B/CI_E7_MULTISCENE_FORMAL_F1_B_20261010T044312Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F1/C/CI_E7_MULTISCENE_FORMAL_F1_C_20261010T045341Z/validation_report.json) |
| F2 / 4102 | 450 s | 214.2 / 214.2 / 116.1 m | 98.1 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/F2_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F2/A/CI_E7_MULTISCENE_FORMAL_F2_A_20261010T050416Z/preview/F2_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F2/B/CI_E7_MULTISCENE_FORMAL_F2_B_20261010T083441Z/preview/F2_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F2/C/CI_E7_MULTISCENE_FORMAL_F2_C_20261010T084518Z/preview/F2_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F2/A/CI_E7_MULTISCENE_FORMAL_F2_A_20261010T050416Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F2/B/CI_E7_MULTISCENE_FORMAL_F2_B_20261010T083441Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F2/C/CI_E7_MULTISCENE_FORMAL_F2_C_20261010T084518Z/validation_report.json) |
| F3 / 4103 | 450 s | 230.5 / 215.8 / 177.8 m | 38.0 m | [查看图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/F3_route_distance.png) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F3/A/CI_E7_MULTISCENE_FORMAL_F3_A_20261010T091744Z/preview/F3_A_five_view_boxed_sparse.mp4) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F3/B/CI_E7_MULTISCENE_FORMAL_F3_B_20261010T092824Z/preview/F3_B_five_view_boxed_sparse.mp4) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F3/C/CI_E7_MULTISCENE_FORMAL_F3_C_20261010T093910Z/preview/F3_C_five_view_boxed_sparse.mp4) | [A](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F3/A/CI_E7_MULTISCENE_FORMAL_F3_A_20261010T091744Z/validation_report.json) / [B](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F3/B/CI_E7_MULTISCENE_FORMAL_F3_B_20261010T092824Z/validation_report.json) / [C](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/F3/C/CI_E7_MULTISCENE_FORMAL_F3_C_20261010T093910Z/validation_report.json) |

### 代表性可视化

**四区域汇总：目标接近与巡检路线偏离。** 这两图是 12 个配对场景的描述性汇总，不代表显著性检验。

![按场景的目标接近量](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/target_approach_by_scene.png)

![按场景的预设路线偏离](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/patrol_route_departure_by_scene.png)

以下每张图均含 UAV/UGV 实际路径；完整 12 张见上表或‘全部逐场景轨迹图’入口。

**N1 配对轨迹与目标距离：** [打开原图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/N1_route_distance.png)

![N1 的 UAV/UGV 配对轨迹](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/N1_route_distance.png)

**E1 配对轨迹与目标距离：** [打开原图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/E1_route_distance.png)

![E1 的 UAV/UGV 配对轨迹](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/E1_route_distance.png)

**W1 配对轨迹与目标距离：** [打开原图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/W1_route_distance.png)

![W1 的 UAV/UGV 配对轨迹](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/W1_route_distance.png)

**F1 配对轨迹与目标距离：** [打开原图](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/F1_route_distance.png)

![F1 的 UAV/UGV 配对轨迹](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories/F1_route_distance.png)

## 输出与复核入口

- 批次根目录：`E:\CarlaAirData\CityInspection_GOC\multiscene_abc_formal_20261010_v2`。
- 批次进度与 36 组目录：[batch_progress.json](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/batch_progress.json)；包含全部回合路径及运行概要。
- 36 组原生验收路径索引：[all_validation_paths.tsv](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/all_validation_paths.tsv)；包含逐回合 `validation_report.json`、preview、日志和关键检查字段。
- 回放验收：[replay_validation.json](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/replay_validation.json)；逐个记录 36 段视频、首帧图、源帧范围、帧数、尺寸、同步 spread 和通过状态。
- 可重绘指标：[paired_metrics_36.csv](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_metrics_36.csv)、[group_summary.csv](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/group_summary.csv)、[paired_effects.csv](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_effects.csv)。
- 机器可读分析和验收摘要：[analysis_summary.json](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/analysis_summary.json)。
- 12 张逐场景 UAV/UGV 轨迹与距离图：[paired_trajectories/](E:/CarlaAirData/CityInspection_GOC/multiscene_abc_formal_20261010_v2/analysis/paired_trajectories)。
- 正式实验配置索引：[formal_36_run_index.json](E:/Research/CityInspection_GOC/configs/experiments/multiscene_abc_formal_20261010_v2/formal_36_run_index.json)。

## 限制与下一步

1. 12 个配对场景可支持系统级趋势比较，但仍是描述性结果；后续需要更多独立随机种子，并报告配对置信区间/适当统计检验。
2. 本批次还没有物理带宽、空地衰落、阴影遮挡与随机丢包，不能据此回答物理信道约束下的层级通信与 VoI 问题。
3. 传感器仅存每约 5 s 的同步样本，因此目前回放用于核对采样对齐与典型观测，不用于判断间隔内瞬时行为；完整轨迹、状态和通信日志仍按回合保存。需要连续回放时必须重新采集更高频传感器数据。
4. 目标接近受 RGB-D 避障和安全停车约束；10 m 内回合数只有 C 组 1/12。不能为提高接近率而绕过既有安全模块，应先明确任务成功定义与安全停车语义。

本报告引用已完成的 36 组批次与离线生成的逐组回放，不触发仿真重跑。
