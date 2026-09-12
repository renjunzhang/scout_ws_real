# 防晃领域公开论文集：按原分类整理

> 更新：2026-09-12。主分类依据：[液体防晃方法分类与领域分析](../液体防晃方法分类与领域分析.md)第 4 章（4.1–4.9）。
>
> R01–R66 为本目录编号，原有 R01–R37 和 PDF 路径保持稳定；AR01–AR29 已建立可追溯关联，见[旧库导入记录](20260912_旧库导入记录.md)。

现有 **66 篇 PDF，404.1 MiB**。原 37 篇中 34 篇匹配正式出版记录、3 篇暂按预印本使用；2026-09-12 从用户 ZIP 新增 29 篇，其刊会信息来自旧索引／arXiv 元数据，尚未在本次独立复核。来源为 61 篇 arXiv、4 篇机构库和 1 篇期刊公开全文。

**PDF 全文仅保存在本地，不提交或上传 Git 仓库。** 本目录的 [.gitignore](.gitignore) 递归忽略大小写扩展名的 PDF，索引、BibTeX 和元数据保留为可版本控制材料。上面的数量是本机文件快照；新克隆仓库后需自行恢复本地全文，PDF 相对链接才可打开，在线来源仍见逐篇索引。

## 分类规则

- **主分类与交叉分类**沿用原文 4.1–4.9；一篇论文可以跨类，按 R 编号去重计数。
- **通用方法基础**单列，并注明可借鉴到哪些类别；MPCC、TOPPRA、funnel 等未被算成直接防晃成果。
- **液体建模／物理基础**对应原文第 3 章，避免把相位实验和模型推导硬塞进控制算法类别。
- 原来的四组保留为**阅读标签**；P1/P2/P3 表示本题阅读优先级，不替代方法分类或发表级别。

当前为 21 篇直接液体方法、27 篇通用方法基础、18 篇液体建模／物理基础。“直接液体方法”包括理论和数值工作，不等于均有实物实验。分类依据摘要、首页与所检查段落；细分状态反馈等级、模型条件仍待全文核对。新增批次按旧笔记与首页作暂定映射，导入成功不等于精读或独立证实旧笔记中的效果。

[逐篇分类对照](论文分类对照.md) · [阅读卡片](论文阅读卡片.md) · [待补齐与合并清单](待补齐与合并清单.md)

## 推荐阅读顺序

| 顺序 | 本地 PDF | 首先回答的问题 |
|---:|---|---|
| 1 | [R36 · Antisloshing Trajectories for High-Acceleration Motions in Automatic Machines](pdf/R36_DOI_10.1115_1.4054224_Antisloshing_Trajectories_for_High_Acceleration_Motions_in_Automatic_Machines.pdf) | 规定相同运输时长时，已有防晃滤波能做到什么？ |
| 2 | [R32 · Time-Optimal Anti-Sloshing Trajectory Planning for Multiple Liquid-Filled Containers Subject to SCARA Motion](pdf/R32_DOI_10.1109_lra.2025.3643281_Time_Optimal_Anti_Sloshing_Trajectory_Planning_for_Multiple_Liquid_Filled_Containers_Subject_to_SCAR.pdf) | 固定竖直杯体的离线液体轨迹优化已有多强的基线？ |
| 3 | [R08 · Model Predictive Contouring Control for Time-Optimal Quadrotor Flight](pdf/R08_2108.13205v4_Model_Predictive_Contouring_Control_for_Time_Optimal_Quadrotor_Flight.pdf) | 在线 MPCC 怎样统一路径进度与运动控制？ |
| 4 | [R13 · A New Approach to Time-Optimal Path Parameterization based on Reachability Analysis](pdf/R13_1707.07239v2_A_New_Approach_to_Time_Optimal_Path_Parameterization_based_on_Reachability_Analysis.pdf) | 怎样表达剩余动作的受约束可行性？ |
| 5 | [R15 · Generalizing Trajectory Retiming to Quadratic Objective Functions](pdf/R15_2309.10176v1_Generalizing_Trajectory_Retiming_to_Quadratic_Objective_Functions.pdf) | 时间参数化如何处理最短时间之外的目标？ |
| 6 | [R16 · Funnel Libraries for Real-Time Robust Feedback Motion Planning](pdf/R16_1601.04037v3_Funnel_Libraries_for_Real_Time_Robust_Feedback_Motion_Planning.pdf) | 规划怎样预先考虑反馈修正能力？ |
| 7 | [R19 · Minimum Time Learning Model Predictive Control](pdf/R19_1911.09239v4_Minimum_Time_Learning_Model_Predictive_Control.pdf) | 短时域如何利用剩余全程的终端指导？ |
| 8 | [R06 · Phase-lag predicts nonlinear response maxima in liquid-sloshing experiments](pdf/R06_2011.02726v3_Phase_lag_predicts_nonlinear_response_maxima_in_liquid_sloshing_experiments.pdf) | 液体相位何时有决策价值，模型何时失真？ |

R01、R03、R26 继续作为离线防晃、在线急停和防洒规划管线的近邻对照；R31 是平滑基线。**C06 的 LMPC 指 Lyapunov-based MPC，R19／R28 的 LMPC 指 Learning MPC，二者不能混用。**

## 对当前方法的建议

最值得先融合的是三种思想：**TOPPRA／二次目标重定时的可行时序表达（R13、R15）＋ funnel 的反馈能力意识（R16）＋ MPCC 的统一进度优化（R08）**。R19／R28 的终端价值思想用于后续解决短时域看不到到站沉降的问题。这里是候选设计，不是这些论文已经证明组合有效。

- **上层**：联合规划可执行车体运动与液体响应，评价受扰后剩余时间和动作自由度还能减少多少液体残余；选择既有名义性能又保留修正空间的计划。
- **下层**：用同一时刻的车体、液体和执行器状态，在 MPCC 内优化唯一一套进度／时序决策；使用预测的实际运动激励液体。
- **先验证收益**：准确状态、充分求解条件下，在相同路径、到达时刻和动作约束下比较固定执行、简单减速、剩余时序优化。随后做“名义／考虑恢复能力的规划”×“普通／液体感知在线控制”交叉对照。
- **再增加复杂度**：只有上述修正收益稳定存在，才考虑 tube、终端安全集合或学习价值函数。各论文的递归可行性与安全结论均须重新检查适用条件。

潜在论文贡献是“约束下的液体恢复能力如何被刻画、如何影响上层计划并被在线利用”。两层结构、液体模型、进度优化、鲁棒性分别都有成熟近邻，仅叠加模块不足以支撑 T-RO 创新。详见[方法融合建议](方法融合建议.md)与[逐篇阅读卡片](论文阅读卡片.md)。

对应既有分类，当前主线是 **4.4（离线规划）＋4.6（闭环跟踪抑晃）＋4.7（滚动预测）**。R02 的容器倾斜／几何补偿属于 4.5，作为相邻路线参考。

## 按 4.1–4.9 的方法索引

交叉归类的论文可能出现在多个小节，PDF 只保留一份；总数始终按唯一 R 编号计算。

### 4.1 被动容器、结构与载荷接口

| 本地 PDF | 发表记录 | 主／交叉定位 | 优先级 |
|---|---|---|---|
| [R44 · Sloshing in vertical cylinders with circular walls: the effect of radial baffles](pdf/R44_2107.09501v1_Radial_Baffles_Vertical_Cylinders.pdf) | 导入题录，刊会待复核 | 4.1 | P2 |
| [R46 · Damping of liquid sloshing by foams](pdf/R46_1411.6542v2_Damping_Liquid_Sloshing_by_Foams.pdf) | 导入题录，刊会待复核 | 4.1 | P2 |
| [R47 · Vibration Mitigation in Partially Liquid-Filled Vessel using Passive Energy Absorbers](pdf/R47_1608.06358_Passive_Energy_Absorbers_Liquid_Filled_Vessel.pdf) | 导入题录，刊会待复核 | 4.1 | P2 |

### 4.2 主动辅助机构、悬架与平台原生姿态补偿

| 本地 PDF | 发表记录 | 主／交叉定位 | 优先级 |
|---|---|---|---|
| [R37 · Damping control of sloshing during liquid container transfer by active vibration reducer with 6-DOF parallel linkage (In the case of straight path on horizontal plane)](pdf/R37_DOI_10.1299_mej.2014dr0061_Damping_control_of_sloshing_during_liquid_container_transfer_by_active_vibration_reducer_with_6_DOF.pdf) | Mechanical Engineering Journal 2014 · [DOI](https://doi.org/10.1299/mej.2014dr0061) | 4.2；兼 4.6 | P2 |
| [R41 · Model Based Active Slosh Damping Experiment](pdf/R41_1801.10017v1_Model_Based_Active_Slosh_Damping_Experiment.pdf) | 导入题录，刊会待复核 | 4.2；兼 4.6 | P2 |
| [R48 · Sloshing suppression with a controlled elastic baffle via deep reinforcement learning and SPH simulation](pdf/R48_2505.02354v1_Controlled_Elastic_Baffle_DRL_SPH.pdf) | 导入题录，刊会待复核 | 4.2；兼 4.9 | P2 |

### 4.3 输入整形、滤波与运动剖面

| 本地 PDF | 发表记录 | 主／交叉定位 | 优先级 |
|---|---|---|---|
| [R35 · A Plug-In Feed-Forward Control for Sloshing Suppression in Robotic Teleoperation Tasks](pdf/R35_DOI_10.1109_iros.2018.8593962_A_Plug_In_Feed_Forward_Control_for_Sloshing_Suppression_in_Robotic_Teleoperation_Tasks.pdf) | IROS 2018 · [DOI](https://doi.org/10.1109/iros.2018.8593962) | 4.5；兼 4.3 | P1 |
| [R36 · Antisloshing Trajectories for High-Acceleration Motions in Automatic Machines](pdf/R36_DOI_10.1115_1.4054224_Antisloshing_Trajectories_for_High_Acceleration_Motions_in_Automatic_Machines.pdf) | Journal of Dynamic Systems, Measurement, and Control 2022 · [DOI](https://doi.org/10.1115/1.4054224) | 4.3 | P1 |

### 4.4 离线路径、时间律与整段轨迹优化

| 本地 PDF | 发表记录 | 主／交叉定位 | 优先级 |
|---|---|---|---|
| [R01 · A Solution to Slosh-free Robot Trajectory Optimization](pdf/R01_2210.12614v1_A_Solution_to_Slosh_free_Robot_Trajectory_Optimization.pdf) | IROS 2022 · [DOI](https://doi.org/10.1109/iros47612.2022.9981173) | 4.4 | P1 |
| [R26 · Clutter-Aware Spill-Free Liquid Transport via Learned Dynamics](pdf/R26_2408.00215v1_Clutter_Aware_Spill_Free_Liquid_Transport_via_Learned_Dynamics.pdf) | IROS 2024 · [DOI](https://doi.org/10.1109/iros58592.2024.10802215) | 4.8；兼 4.4、4.9 | P1 |
| [R32 · Time-Optimal Anti-Sloshing Trajectory Planning for Multiple Liquid-Filled Containers Subject to SCARA Motion](pdf/R32_DOI_10.1109_lra.2025.3643281_Time_Optimal_Anti_Sloshing_Trajectory_Planning_for_Multiple_Liquid_Filled_Containers_Subject_to_SCAR.pdf) | IEEE RA-L 2026 · [DOI](https://doi.org/10.1109/lra.2025.3643281) | 4.4 | P1 |
| [R33 · Time-Optimal Transport of Loosely Placed Liquid Filled Cups along Prescribed Paths](pdf/R33_2510.25255v1_Time_Optimal_Transport_of_Loosely_Placed_Liquid_Filled_Cups_Along_Prescribed_Paths.pdf) | RAAD 2024 · [DOI](https://doi.org/10.1007/978-3-031-59257-7_41) | 4.4 | P2 |

### 4.5 在线前馈与几何防晃生成

| 本地 PDF | 发表记录 | 主／交叉定位 | 优先级 |
|---|---|---|---|
| [R02 · Geometric Slosh-Free Tracking for Robotic Manipulators](pdf/R02_2402.05197v1_Geometric_Slosh_Free_Tracking_for_Robotic_Manipulators.pdf) | ICRA 2024 · [DOI](https://doi.org/10.1109/icra57147.2024.10610813) | 4.5 | P2 |
| [R35 · A Plug-In Feed-Forward Control for Sloshing Suppression in Robotic Teleoperation Tasks](pdf/R35_DOI_10.1109_iros.2018.8593962_A_Plug_In_Feed_Forward_Control_for_Sloshing_Suppression_in_Robotic_Teleoperation_Tasks.pdf) | IROS 2018 · [DOI](https://doi.org/10.1109/iros.2018.8593962) | 4.5；兼 4.3 | P1 |

### 4.6 反馈稳定化、观测器与给定参考闭环跟踪

| 本地 PDF | 发表记录 | 主／交叉定位 | 优先级 |
|---|---|---|---|
| [R04 · Spill-Free Transfer and Stabilization of Viscous Liquid](pdf/R04_2108.11052v1_Spill_Free_Transfer_and_Stabilization_of_Viscous_Liquid.pdf) | IEEE TAC 2022 · [DOI](https://doi.org/10.1109/tac.2022.3162551) | 4.6 | P2 |
| [R05 · Output-Feedback Control of Viscous Liquid-Tank System and its Numerical Approximation](pdf/R05_2201.13272v2_Output_Feedback_Control_of_Viscous_Liquid_Tank_System_and_its_Numerical_Approximation.pdf) | Automatica 2023 · [DOI](https://doi.org/10.1016/j.automatica.2022.110827) | 4.6 | P2 |
| [R07 · Safe Learning Reference Governor: Theory and Application to Fuel Truck Rollover Avoidance](pdf/R07_2101.09298v2_Safe_Learning_Reference_Governor_Theory_and_Application_to_Fuel_Truck_Rollover_Avoidance.pdf) | Journal of Autonomous Vehicles and Systems 2021 · [DOI](https://doi.org/10.1115/1.4053244) | 4.6；兼 4.9 | P2 |
| [R37 · Damping control of sloshing during liquid container transfer by active vibration reducer with 6-DOF parallel linkage (In the case of straight path on horizontal plane)](pdf/R37_DOI_10.1299_mej.2014dr0061_Damping_control_of_sloshing_during_liquid_container_transfer_by_active_vibration_reducer_with_6_DOF.pdf) | Mechanical Engineering Journal 2014 · [DOI](https://doi.org/10.1299/mej.2014dr0061) | 4.2；兼 4.6 | P2 |
| [R41 · Model Based Active Slosh Damping Experiment](pdf/R41_1801.10017v1_Model_Based_Active_Slosh_Damping_Experiment.pdf) | 导入题录，刊会待复核 | 4.2；兼 4.6 | P2 |
| [R42 · Closed-loop control of sloshing fuel in a spinning spacecraft](pdf/R42_2510.08121v1_Closed_Loop_Control_Spinning_Spacecraft.pdf) | 导入题录，刊会待复核 | 4.6 | P2 |

### 4.7 MPC/GPC 预测控制与急停安全层

| 本地 PDF | 发表记录 | 主／交叉定位 | 优先级 |
|---|---|---|---|
| [R03 · Emergency Stopping for Liquid-manipulating Robots](pdf/R03_2604.16667v1_Emergency_Stopping_for_Liquid_manipulating_Robots.pdf) | arXiv 预印本 2026（正式发表未核实） | 4.7 | P1 |
| [R27 · Shared Telemanipulation with VR controllers in an anti slosh scenario](pdf/R27_2309.07714v1_Shared_Telemanipulation_with_VR_controllers_in_an_anti_slosh_scenario.pdf) | IEEE SMC 2023 · [DOI](https://doi.org/10.1109/smc53992.2023.10393900) | 4.7 | P2 |
| [R39 · MPC-based Deep Reinforcement Learning Method for Space Robotic Control with Fuel Sloshing Mitigation](pdf/R39_2509.21045_MPC_DRL_Space_Robot_Fuel_Sloshing.pdf) | 导入题录，刊会待复核 | 4.7；兼 4.9 | P2 |
| [R40 · Towards Universal Shared Control in Teleoperation Without Haptic Feedback](pdf/R40_2506.23624v2_Universal_Shared_Control_Teleoperation_Slosh.pdf) | 导入题录，刊会待复核 | 4.7；兼 4.8 | P2 |

### 4.8 障碍环境中的防洒规划

| 本地 PDF | 发表记录 | 主／交叉定位 | 优先级 |
|---|---|---|---|
| [R26 · Clutter-Aware Spill-Free Liquid Transport via Learned Dynamics](pdf/R26_2408.00215v1_Clutter_Aware_Spill_Free_Liquid_Transport_via_Learned_Dynamics.pdf) | IROS 2024 · [DOI](https://doi.org/10.1109/iros58592.2024.10802215) | 4.8；兼 4.4、4.9 | P1 |
| [R40 · Towards Universal Shared Control in Teleoperation Without Haptic Feedback](pdf/R40_2506.23624v2_Universal_Shared_Control_Teleoperation_Slosh.pdf) | 导入题录，刊会待复核 | 4.7；兼 4.8 | P2 |

### 4.9 学习、数字孪生与混合模型

| 本地 PDF | 发表记录 | 主／交叉定位 | 优先级 |
|---|---|---|---|
| [R07 · Safe Learning Reference Governor: Theory and Application to Fuel Truck Rollover Avoidance](pdf/R07_2101.09298v2_Safe_Learning_Reference_Governor_Theory_and_Application_to_Fuel_Truck_Rollover_Avoidance.pdf) | Journal of Autonomous Vehicles and Systems 2021 · [DOI](https://doi.org/10.1115/1.4053244) | 4.6；兼 4.9 | P2 |
| [R26 · Clutter-Aware Spill-Free Liquid Transport via Learned Dynamics](pdf/R26_2408.00215v1_Clutter_Aware_Spill_Free_Liquid_Transport_via_Learned_Dynamics.pdf) | IROS 2024 · [DOI](https://doi.org/10.1109/iros58592.2024.10802215) | 4.8；兼 4.4、4.9 | P1 |
| [R39 · MPC-based Deep Reinforcement Learning Method for Space Robotic Control with Fuel Sloshing Mitigation](pdf/R39_2509.21045_MPC_DRL_Space_Robot_Fuel_Sloshing.pdf) | 导入题录，刊会待复核 | 4.7；兼 4.9 | P2 |
| [R48 · Sloshing suppression with a controlled elastic baffle via deep reinforcement learning and SPH simulation](pdf/R48_2505.02354v1_Controlled_Elastic_Baffle_DRL_SPH.pdf) | 导入题录，刊会待复核 | 4.2；兼 4.9 | P2 |

## 通用方法基础

以下是可迁移的规划、控制或滤波方法，未作为直接防晃论文归入上面各类。

| 本地 PDF | 发表记录 | 可借鉴类别 | 阅读标签／优先级 |
|---|---|---|---|
| [R08 · Model Predictive Contouring Control for Time-Optimal Quadrotor Flight](pdf/R08_2108.13205v4_Model_Predictive_Contouring_Control_for_Time_Optimal_Quadrotor_Flight.pdf) | IEEE T-RO 2022 · [DOI](https://doi.org/10.1109/tro.2022.3173711) | 4.6、4.7 | 进度优化与轨迹重定时／P1 |
| [R09 · MPCC++: Model Predictive Contouring Control for Time-Optimal Flight with Safety Constraints](pdf/R09_2403.17551v2_MPCC_Model_Predictive_Contouring_Control_for_Time_Optimal_Flight_with_Safety_Constraints.pdf) | RSS 2024 · [DOI](https://doi.org/10.15607/rss.2024.xx.109) | 4.7、4.9 | 进度优化与轨迹重定时／P2 |
| [R10 · Model Predictive Contouring Control for Collision Avoidance in Unstructured Dynamic Environments](pdf/R10_2010.10190v1_Model_Predictive_Contouring_Control_for_Collision_Avoidance_in_Unstructured_Dynamic_Environments.pdf) | IEEE RA-L 2019 · [DOI](https://doi.org/10.1109/lra.2019.2929976) | 4.7、4.8 | 进度优化与轨迹重定时／P2 |
| [R11 · Implementation of Nonlinear Model Predictive Path-Following Control for an Industrial Robot](pdf/R11_1506.09084v2_Implementation_of_Nonlinear_Model_Predictive_Path_Following_Control_for_an_Industrial_Robot.pdf) | IEEE TCST 2017 · [DOI](https://doi.org/10.1109/tcst.2016.2601624) | 4.6、4.7 | 进度优化与轨迹重定时／P2 |
| [R12 · Reactive Model Predictive Contouring Control for Robot Manipulators](pdf/R12_2508.09502v1_Reactive_Model_Predictive_Contouring_Control_for_Robot_Manipulators.pdf) | IROS 2025 · [DOI](https://doi.org/10.1109/iros60139.2025.11247346) | 4.7、4.8 | 进度优化与轨迹重定时／P3 |
| [R13 · A New Approach to Time-Optimal Path Parameterization based on Reachability Analysis](pdf/R13_1707.07239v2_A_New_Approach_to_Time_Optimal_Path_Parameterization_based_on_Reachability_Analysis.pdf) | IEEE T-RO 2018 · [DOI](https://doi.org/10.1109/tro.2018.2819195) | 4.4 | 进度优化与轨迹重定时／P1 |
| [R14 · On the Structure of the Time-Optimal Path Parameterization Problem with Third-Order Constraints](pdf/R14_1609.05307v3_On_the_Structure_of_the_Time_Optimal_Path_Parameterization_Problem_with_Third_Order_Constraints.pdf) | ICRA 2017 · [DOI](https://doi.org/10.1109/icra.2017.7989084) | 4.3、4.4 | 进度优化与轨迹重定时／P2 |
| [R15 · Generalizing Trajectory Retiming to Quadratic Objective Functions](pdf/R15_2309.10176v1_Generalizing_Trajectory_Retiming_to_Quadratic_Objective_Functions.pdf) | ICRA 2024 · [DOI](https://doi.org/10.1109/icra57147.2024.10610854) | 4.4 | 进度优化与轨迹重定时／P1 |
| [R16 · Funnel Libraries for Real-Time Robust Feedback Motion Planning](pdf/R16_1601.04037v3_Funnel_Libraries_for_Real_Time_Robust_Feedback_Motion_Planning.pdf) | IJRR 2017 · [DOI](https://doi.org/10.1177/0278364917712421) | 4.4、4.6、4.8 | 恢复能力与终端指导／P1 |
| [R17 · FaSTrack: a Modular Framework for Real-Time Motion Planning and Guaranteed Safe Tracking](pdf/R17_2102.07039v2_FaSTrack_a_Modular_Framework_for_Real_Time_Motion_Planning_and_Guaranteed_Safe_Tracking.pdf) | IEEE TAC 2021 · [DOI](https://doi.org/10.1109/tac.2021.3059838) | 4.4、4.6、4.8 | 恢复能力与终端指导／P2 |
| [R18 · PiP-X: Online feedback motion planning/replanning in dynamic environments using invariant funnels](pdf/R18_2202.00772v3_PiP_X_Online_feedback_motion_planning_replanning_in_dynamic_environments_using_invariant_funnels.pdf) | WAFR 2022 · [DOI](https://doi.org/10.1007/978-3-031-21090-7_9) | 4.6、4.8 | 恢复能力与终端指导／P3 |
| [R19 · Minimum Time Learning Model Predictive Control](pdf/R19_1911.09239v4_Minimum_Time_Learning_Model_Predictive_Control.pdf) | International Journal of Robust and Nonlinear Control 2021 · [DOI](https://doi.org/10.1002/rnc.5284) | 4.7、4.9 | 恢复能力与终端指导／P1 |
| [R20 · Output-Lifted Learning Model Predictive Control](pdf/R20_2004.05173v3_Output_Lifted_Learning_Model_Predictive_Control.pdf) | IFAC-PapersOnLine 2021 · [DOI](https://doi.org/10.1016/j.ifacol.2021.08.571) | 4.7、4.9 | 恢复能力与终端指导／P3 |
| [R21 · Robust Output-Lifted Learning Model Predictive Control](pdf/R21_2303.12127v1_Robust_Output_Lifted_Learning_Model_Predictive_Control.pdf) | arXiv 预印本 2023（正式发表未核实） | 4.7、4.9 | 恢复能力与终端指导／P3 |
| [R22 · Learning Model Predictive Control for Quadrotors](pdf/R22_2202.07716v3_Learning_Model_Predictive_Control_for_Quadrotors.pdf) | ICRA 2022 · [DOI](https://doi.org/10.1109/icra46639.2022.9812077) | 4.7、4.9 | 恢复能力与终端指导／P2 |
| [R23 · A predictive safety filter for learning-based control of constrained nonlinear dynamical systems](pdf/R23_1812.05506v4_A_predictive_safety_filter_for_learning_based_control_of_constrained_nonlinear_dynamical_systems.pdf) | Automatica 2021 · [DOI](https://doi.org/10.1016/j.automatica.2021.109597) | 4.7、4.9 | 鲁棒控制与安全覆盖／P2 |
| [R24 · A computationally efficient robust model predictive control framework for uncertain nonlinear systems -- extended version](pdf/R24_1910.12081v2_A_computationally_efficient_robust_model_predictive_control_framework_for_uncertain_nonlinear_system.pdf) | IEEE TAC 2021 · [DOI](https://doi.org/10.1109/tac.2020.2982585) | 4.7 | 鲁棒控制与安全覆盖／P2 |
| [R25 · Differentiable Robust Model Predictive Control](pdf/R25_2308.08426v3_Differentiable_Robust_Model_Predictive_Control.pdf) | RSS 2024 · [DOI](https://doi.org/10.15607/rss.2024.xx.003) | 4.7、4.9 | 鲁棒控制与安全覆盖／P3 |
| [R28 · Learning Model Predictive Control for iterative tasks. A Data-Driven Control Framework](pdf/R28_1609.01387v7_Learning_Model_Predictive_Control_for_iterative_tasks_A_Data_Driven_Control_Framework.pdf) | IEEE TAC 2018 · [DOI](https://doi.org/10.1109/tac.2017.2753460) | 4.7、4.9 | 恢复能力与终端指导／P1 |
| [R29 · SIT-LMPC: Safe Information-Theoretic Learning Model Predictive Control for Iterative Tasks](pdf/R29_2602.16187v1_SIT_LMPC_Safe_Information_Theoretic_Learning_Model_Predictive_Control_for_Iterative_Tasks.pdf) | IEEE RA-L 2026 · [DOI](https://doi.org/10.1109/lra.2025.3634881) | 4.7、4.9 | 恢复能力与终端指导／P3 |
| [R31 · Jerk-limited Real-time Trajectory Generation with Arbitrary Target States](pdf/R31_2105.04830v2_Jerk_limited_Real_time_Trajectory_Generation_with_Arbitrary_Target_States.pdf) | RSS 2021 · [DOI](https://doi.org/10.15607/rss.2021.xvii.015) | 4.3 | 进度优化与轨迹重定时／P1 |
| [R34 · Optimal Trajectories for Vibration Reduction Based on Exponential Filters](pdf/R34_DOI_10.1109_tcst.2015.2460693_Optimal_Trajectories_for_Vibration_Reduction_Based_on_Exponential_Filters.pdf) | IEEE TCST 2016 · [DOI](https://doi.org/10.1109/tcst.2015.2460693) | 4.3 | 进度优化与轨迹重定时／P2 |
| [R38 · Uncertainty-aware Planning with Inaccurate Models for Robotized Liquid Handling](pdf/R38_2507.20861v1_Uncertainty_Aware_Planning_Robotized_Liquid_Handling.pdf) | 导入题录，刊会待复核 | 基础 → 4.4、4.9 | 可迁移方法基础／P2 |
| [R63 · Safe Trajectory Tracking in Uncertain Environments](pdf/R63_2001.11602v2_Safe_Trajectory_Tracking_Uncertain_Environments.pdf) | 导入题录，刊会待复核 | 基础 → 4.6、4.7 | 可迁移方法基础／P2 |
| [R64 · A predictive safety filter for learning-based racing control](pdf/R64_2102.11907v1_Predictive_Safety_Filter_Learning_Based_Racing.pdf) | 导入题录，刊会待复核 | 基础 → 4.7、4.9 | 可迁移方法基础／P2 |
| [R65 · Delay-aware Robust Control for Safe Autonomous Driving and Racing](pdf/R65_2208.13856v1_Delay_Aware_Robust_Control.pdf) | 导入题录，刊会待复核 | 基础 → 4.6、4.7 | 可迁移方法基础／P2 |
| [R66 · Explicit Reference Governor for the Constrained Control of Time-Delayed Linear Systems](pdf/R66_1712.08248v1_Explicit_Reference_Governor_Time_Delayed_Linear_Systems.pdf) | 导入题录，刊会待复核 | 基础 → 4.6 | 可迁移方法基础／P2 |

## 液体建模与物理基础

| 本地 PDF | 发表记录 | 原文基础章节 | 用途 |
|---|---|---|---|
| [R06 · Phase-lag predicts nonlinear response maxima in liquid-sloshing experiments](pdf/R06_2011.02726v3_Phase_lag_predicts_nonlinear_response_maxima_in_liquid_sloshing_experiments.pdf) | Journal of Fluid Mechanics 2021 · [DOI](https://doi.org/10.1017/jfm.2021.576) | 3.1、3.4、3.5 | 实验液体相位与模型适用域研究，不提出机器人防晃规划器。 |
| [R30 · Equivalent Mechanical Models for Sloshing](pdf/R30_2511.10172v2_Equivalent_Mechanical_Models_for_Sloshing.pdf) | arXiv 预印本 2025（正式发表未核实） | 3.1、3.2 | 等效摆／质量弹簧阻尼建模基础；没有提出本题的机器人防晃控制方法。 |
| [R43 · Data-driven Learning of LPV Surrogate Models of Fuel Sloshing](pdf/R43_2604.12505v1_LPV_Surrogate_Models_Fuel_Sloshing.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.2；关联 4.9 | SPH 到 LPV 的快速晃液代理，按建模基础收录。 |
| [R45 · Effect of baffles on pressurization and thermal stratification in a LN2 tank under micro-gravity](pdf/R45_1807.01547v1_Baffles_Cryogenic_Tank_Microgravity.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.2、3.5；关联 4.1 | 隔板热流耦合的邻近物理证据；指标不是开口运输防洒。 |
| [R49 · Swirling against the forcing: evidence of stable counter-directed sloshing waves in orbital-shaken reservoirs](pdf/R49_2302.14579v2_Counter_Directed_Sloshing_Waves.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.1、3.5 | 非线性模态、能量耗散或自由面响应的物理基础，非机器人规划器。 |
| [R50 · Response Regimes in Equivalent Mechanical Model of Moderately Nonlinear Liquid Sloshing](pdf/R50_1703.02017_Moderately_Nonlinear_Equivalent_Mechanical_Model.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.1、3.5 | 非线性模态、能量耗散或自由面响应的物理基础，非机器人规划器。 |
| [R51 · Response Regimes in Equivalent Mechanical Model of Strongly Nonlinear Liquid Sloshing](pdf/R51_1605.09648_Strongly_Nonlinear_Equivalent_Mechanical_Model.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.1、3.5 | 非线性模态、能量耗散或自由面响应的物理基础，非机器人规划器。 |
| [R52 · Internal resonances and dynamic responses in equivalent mechanical model of partially liquid-filled vessel](pdf/R52_1603.06931_Internal_Resonances_Liquid_Filled_Vessel.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.1、3.5 | 非线性模态、能量耗散或自由面响应的物理基础，非机器人规划器。 |
| [R53 · Mechanical energy dissipation induced by sloshing and wave breaking in a fully coupled angular motion system. Part I: Theoretical formulation and Numerical Investigation](pdf/R53_1307.6064v2_Energy_Dissipation_Sloshing_Part_I_Numerical.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.1、3.5 | 非线性模态、能量耗散或自由面响应的物理基础，非机器人规划器。 |
| [R54 · Mechanical energy dissipation induced by sloshing and wave breaking in a fully coupled angular motion system. Part II: Experimental Investigation](pdf/R54_1307.6063v2_Energy_Dissipation_Sloshing_Part_II_Experiment.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.1、3.5 | 非线性模态、能量耗散或自由面响应的物理基础，非机器人规划器。 |
| [R55 · Surface wave dynamics in orbital shaken cylindrical containers](pdf/R55_1412.0919v1_Surface_Wave_Dynamics_Orbital_Shaking.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.1、3.5 | 非线性模态、能量耗散或自由面响应的物理基础，非机器人规划器。 |
| [R56 · Real-time data assimilation for the thermodynamic modeling of cryogenic storage tanks](pdf/R56_2310.11399v2_Real_Time_Data_Assimilation_Cryogenic_Tanks.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.2、3.3；关联 4.9 | 热质传递、辨识或低温晃液的物理与测量基础，非运输防晃控制。 |
| [R57 · Experimental analysis of heat and mass transfer in non-isothermal sloshing using a model-based inverse method](pdf/R57_2212.12246v1_Non_Isothermal_Sloshing_Inverse_Method.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.2、3.3 | 热质传递、辨识或低温晃液的物理与测量基础，非运输防晃控制。 |
| [R58 · Experimental Characterization of Non-Isothermal Sloshing in Microgravity](pdf/R58_2410.06590v2_Non_Isothermal_Sloshing_Microgravity_Experiment.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.2、3.3 | 热质传递、辨识或低温晃液的物理与测量基础，非运输防晃控制。 |
| [R59 · Real-time identification of parametric sloshing-induced heat and mass transfer in a horizontally oriented cylindrical tank](pdf/R59_2510.19540v3_Real_Time_Identification_Parametric_Sloshing.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.2、3.3 | 热质传递、辨识或低温晃液的物理与测量基础，非运输防晃控制。 |
| [R60 · Extension of the consistent $δ^{+}$-SPH model for multiphase flows considering the compressibility of different phases](pdf/R60_2505.10215v1_Multiphase_Compressible_Delta_SPH.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.2、3.5 | 多相高保真 SPH 建模基础，非在线 MPC 直接模型。 |
| [R61 · A Machine Learning-based Characterization Framework for Parametric Representation of Nonlinear Sloshing](pdf/R61_2201.11663v1_ML_Characterization_Nonlinear_Sloshing.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.2、3.3、3.5；关联 4.9 | 液体辨识、学习或自由面重建基础；未验证本题在线控制收益。 |
| [R62 · A Thermodynamics-informed Active Learning Approach to Perception and Reasoning about Fluids](pdf/R62_2203.05775v2_Thermodynamics_Informed_Active_Learning_Fluids.pdf) | 导入题录，刊会待复核 | 第 3 章 → 3.2、3.3、3.5；关联 4.9 | 液体辨识、学习或自由面重建基础；未验证本题在线控制收益。 |

## 文件、来源与核验

- [论文分类对照](论文分类对照.md)：每篇的主类、交叉类或方法基础定位。
- [论文阅读卡片](论文阅读卡片.md)：作者、来源、版本、借鉴点、迁移限制及精读问题。
- [方法融合建议](方法融合建议.md)：当前候选设计与最小潜力实验。
- [待补齐与合并清单](待补齐与合并清单.md)：仍待补齐的旧库材料、已完成导入及后续合并规则。
- [检索与核验说明](检索与核验说明.md)：首批与追加批次的范围、来源、文件检查及限制。
- [BibTeX](references.bib) · [完整元数据与哈希](metadata/manifest.json) · [分类数据](metadata/taxonomy_mapping.json)。

66 个 PDF 均完成文件头、文本可读性、页数和 SHA-256 检查。原 37 篇的发表核验状态保留；新增 29 篇仅作题录与旧笔记／首页筛查，刊会记录本次未独立复核，均待全文精读。R03、R21、R30 的正式发表仍未核实；arXiv／机构作者稿未被声称为出版终稿。机构库可能附加下载日期封面，同篇论文不同来源可能具有不同文件哈希。

2026-09-12 的 36 份交接 PDF 已全部对应到本库：新增 29 篇，7 份为字节完全相同的已有文件。见[逐文件导入记录](20260912_旧库导入记录.md)和[旧 AR 索引](arXiv_新增论文索引.md)。原压缩包保留；C06 等 5 项仍待全文。
