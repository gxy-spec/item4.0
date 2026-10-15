# 项目代码审查（2026-10-15）

审查覆盖 `src` 的 24 个 Python 文件、`scripts` 的 55 个 Python 脚本和 13 个 PowerShell 入口、`tools` 的 11 个工具及 14 个测试文件。全仓执行语法检查，人工重点追踪感知→消息→物理信道→目标记忆→路线→RGB-D 安全控制，以及批量运行、验收和统计的数据来源。此次没有启动新的 CARLA 回合，也没有修改运行实现。

现有测试 **95/95 通过**；Python 与 PowerShell 语法检查通过；57 份当前入口及修复版配置通过 schema 和在线安全配置校验。以下问题来自现有测试之外的最小复现；测试全通过不能排除它们。P1 表示应优先修复的安全或实验结果完整性问题，P2 表示应在对应功能继续扩展前修复的问题。

1. **[P1] 正前方深度失效仍可能被判为 CLEAR。**

   位置：[rgbd_obstacle_guard.py:90](E:/Research/CityInspection_GOC/src/safety/rgbd_obstacle_guard.py:90)、[rgbd_obstacle_guard.py:164](E:/Research/CityInspection_GOC/src/safety/rgbd_obstacle_guard.py:164)。有效深度比例以整张采样图为分母；通过这个门槛后，前方通道没有有效点就被当成没有障碍。构造一张仅左侧 10% 像素有效、正前方全部为 NaN 的新深度图，实际返回 `CLEAR`、障碍点数 0。当前测试覆盖全无效或极少量有效像素，没有覆盖“边缘有效而行驶通道失效”。应对行驶通道所需的观测区域单独检查有效覆盖，将无法观察与已观察到空闲区分开。

2. **[P1] 批量重跑失败后可能引用历史 PASS 报告。**

   位置：[run_multiscene_abc_formal.py:79](E:/Research/CityInspection_GOC/scripts/run_multiscene_abc_formal.py:79)。`fresh` 实际选取输出目录内全部已有报告，再按修改时间取最新一份，没有限定报告必须属于本次启动。将子进程模拟为退出码 1，且本次不生成报告，目录里放一份一小时前的 PASS 报告，函数仍返回 `status=PASS`，随后主循环按 PASS 累计完成数。配置检查在建立新运行目录之前执行，因此旧配置校验失败就能触发这一条件。应绑定本次唯一运行目录或本次启动时间及配置身份，并在没有新报告时返回失败。

   已额外核对现有 36 组历史归档：36 份报告路径唯一，报告开始时间均不早于各自启动时间；此次未发现该归档已经受到此漏洞影响的证据。

3. **[P2] `standoff_m` 停靠距离参数没有生效。**

   位置：[ugv_oracle_planner.py:50](E:/Research/CityInspection_GOC/src/navigation/ugv_oracle_planner.py:50)、[run_stage1_dynamic.py:2810](E:/Research/CityInspection_GOC/scripts/run_stage1_dynamic.py:2810)。规划器接受 `standoff_m`，但所有成功分支直接返回通往目标投影车道点的路线，没有使用该参数；E7 目标路线控制分支也未传入消息中的 `target_xyz`，因此控制器的感知目标距离为无穷大。最小规划器替身复现中，`standoff_m=6` 与 `20` 生成完全相同的路线，终点都在目标投影点。现有局部避障仍会停车，但不能把停距解释为已按配置执行了观测停靠。应明确用路线截断或消息位置控制实现停靠距离，同时保留安全避障。

4. **[P2] 目标记忆模式忽略了“双端共识”开关。**

   位置：[run_stage1_dynamic.py:2188](E:/Research/CityInspection_GOC/scripts/run_stage1_dynamic.py:2188)。`require_cross_device_consensus` 只传给 `PersistentTargetSelector`；`persistent_target_memory` 分支却直接把接收包传给 `memory.consider()`，没有经过选择器的共识检查。复现中，将开关设为 true 且仅收到一条 UAV 消息，选择器返回无可选目标，记忆分支仍返回 `initial` 并可规划路线。当前冻结 E1-C 的开关为 false，因此此项不改变对该回合的解释；它会使后续开启共识的消融配置失效。应使初次提交和后续提交明确遵守已声明的共识策略，或拒绝不支持的配置组合。

5. **[P2] S1 本地复核写死模型名称，更换权重后可能永远无法确认。**

   位置：[target_ranker.py:90](E:/Research/CityInspection_GOC/src/perception/target_ranker.py:90)。候选管线根据模型文件名生成 `proposal_source`，但 `LocalTargetVerifier` 只接受 `yolo26x_coco`。同一个满足颜色、类别、位置和时间条件的候选，仅将来源从 `yolo26x_coco` 改为 `yolo11n_coco`，确认结果就由 true 变成 false。即使同时调整 UAV 发送门槛，UGV 复核仍会卡住。该问题影响 S1 的局部复核链路，当前 E7 使用另一条路径。应将允许的来源由同一模型配置传入，而不是在复核器内固定名称。

6. **[P2] 相同信道种子尚不能保证不同策略共享同一个空间随机环境。**

   位置：[a2g_spatial.py:159](E:/Research/CityInspection_GOC/src/communication/a2g_spatial.py:159)。在线 Gaussian field 按访问顺序消耗随机数并条件采样；相同坐标在另一条访问路径上会得到不同值。以 `sigma=4`、相关距离 50 m、种子 1001 复现：坐标 X 首先访问时为 3.729 dB，先访问另一坐标 Y 后再访问 X 时为 0.870 dB。单条给定轨迹内部的协方差和同路径复现仍然成立，但未来 A/B/C 或层级策略改变轨迹后，不能仅凭相同 seed 声称共享同一份外部阴影场。应使用可按世界坐标查询的共同环境场或预先固定的共同信道实现，并增加跨路径同坐标一致性检查。当前 21 组敏感性重放使用固定轨迹，其组内配对结论不因这一点直接失效。

优先修复第 1、2 项，再处理停靠距离和共识配置，随后接入可更换感知模型与正式配对策略实验。现有“启发式识别、特征/图像占位载荷、验收 PASS 不等于任务成功”等已声明阶段边界，未作为新增代码缺陷重复列入。

最小复现观测保存在 [project_code_review_evidence_20261015.json](E:/Research/CityInspection_GOC/docs/reviews/project_code_review_evidence_20261015.json)。
