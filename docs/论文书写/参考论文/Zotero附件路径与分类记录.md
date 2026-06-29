# Zotero 附件路径与 SPMPC 参考论文分类记录

日期：2026-06-29

## 1. Zotero 附件根路径

当前用于 SPMPC 论文参考文献整理的 Zotero 附件目录为：

```text
/data/a/zotero-data/storage
```

说明：

- 该目录是 Zotero 的附件存储目录，可读取 PDF、HTML 快照与 `.zotero-ft-cache` 全文缓存。
- 该目录本身不等于 Zotero collection 分类树；A/B/C/D 分类来自人工整理结果。
- 若后续需要读取 Zotero 原始 collection、tags、notes、citation key，应使用 Zotero 导出文件，例如 Better BibTeX / Better BibLaTeX / Zotero RDF / CSL JSON，或只读查询 `zotero.sqlite` 副本。

## 2. 当前已确认可在 storage 中找到的论文分类

### A. 液体晃动建模 / 估计 / 防晃控制

用途：支撑 SPMPC 的液体模态建模、状态估计、液面观测与防晃控制背景。

重点论文：

- `Sloshing Dynamics Estimation for Liquid-filled Containers under 2-Dimensional Excitation`
  - 当前方法最直接的液体模型参考。
  - 采用二维平面激励下的等效质量-弹簧-阻尼模型。
  - 核心状态是模态广义坐标，而不是直接把液面高度作为状态。
  - 液面高度可由模态状态后续映射得到。
  - 对 SPMPC 的启发：二维激励主输入是容器平动加速度；yaw rate / alpha 本身不应被误写成液体模型的直接输入，横向激励应通过底盘运动学产生的加速度进入模型。

其他论文包括：

- `A Simple Model-Based Method for Sloshing Estimation in Liquid Transfer in Automatic Machines`
- `Modeling of liquid sloshing with application in robotics and automation`
- `Simulation of liquid sloshing in 2D containers using the volume of fluid method`
- `Nonlinear finite element analysis of liquid sloshing in complex vehicle motion scenarios`
- `Physically sound, self-learning digital twins for sloshing fluids`
- `Preshaping Command Inputs to Reduce System Vibration`
- `Slosh Measuring Sensor System for Liquid-Carrying Robots`
- `Liquid Level Detection in Standard Capacity Measures with Machine Vision`
- `Efficient image feature-driven machine learning for meniscus volume measurement in transparent glass vials`
- `Capacitive and Non-Contact Liquid Level Detection Sensor Based on Interdigitated Electrodes with Flexible Substrate`

### B. 移动底盘液体运输 / 防晃路径设计

用途：这是 SPMPC 论文最关键的近邻文献组，用于说明现有移动底盘液体运输方法的能力边界。

核心定位：

- 这些工作与 SPMPC 最接近，因为它们同样关注移动平台上的液体运输和防晃。
- 但很多方法更偏向固定路径、速度剖面、input shaping、离线轨迹优化、特定机构抑振或特定平台控制。
- SPMPC 的区别应写成：在线 receding-horizon slosh-aware MPCC local planner/controller，在同一优化问题中联合考虑路径进度、底盘控制和平面液体模态响应。

论文包括：

- `2D Trajectory Optimization Using Spherical Pendulum Dynamic Constraints for Reducing Liquid Sloshing on Mobile Robots`
- `Path design and trace control of a wheeled mobile robot to damp liquid sloshing in a cylindrical container`
- `Damping and transfer of liquid in cylindrical container using a wheeled mobile robot employing velocity control and path design`
- `TRANSFER CONTROL AND CURVED PATH DESIGN FOR CYLINDRICAL LIQUID CONTAINER`
- `Damping Control of Sloshing in Liquid Container in Cart With Active Vibration Reducer: The Case of a Curved Path on a Horizontal Plane`
- `Damping and Transfer Control System With Parallel Linkage Mechanism-Based Active Vibration Reducer for Omnidirectional Wheeled ...`
- `Control Strategy for Liquid Transfer Using a Four-wheel Mecanum Mobile Robot Platform`
- `Trajectory tracking control of a four mecanum wheeled mobile platform: an extended state observer-based sliding mode approach`
- `Preview-based MPC for active suspension control of tank vehicle with lateral liquid sloshing suppression`
- `Liquid Container Transfer Considering the Suppression of Sloshing for the Change of Liquid Level`
- `Time-Optimal Motion Planning and Anti-Sloshing Control For a Container Under Disturbances`

### C. 机械臂 / SCARA / 机器人操作中的防晃

用途：主要放在 Related Work，用于说明显式液体建模与防晃轨迹优化在机器人领域已有基础。

核心定位：

- 这组文献不适合作为 Scout Mini 实车同层 baseline。
- 原因是平台、控制变量和任务层级不同：多数方法优化机械臂末端轨迹、关节轨迹或特定操作任务，而 SPMPC 关注差速/轮式移动底盘上的在线局部规划与控制。

论文包括：

- `Time-Optimal Anti-Sloshing Trajectory Planning for Multiple Liquid-Filled Containers Subject to SCARA Motion`
- `A Solution to Slosh-free Robot Trajectory Optimization`
- `Manipulating liquids with robots: A sloshing-Free solution`
- `Trajectory planning for meal assist robot considering spilling avoidance`
- `Sloshing Suppression Control by using Physical Boundary Element Model and Predictive Control in Liquid Container Transfer System`
- `Anti-sloshing control: Flatness-based trajectory planning and tracking control with an integrated extended state observer`
- `A robust output feedback strategy for liquid handling using reconfigurable robots`

### D. 普通移动机器人 local planner / MPC / 轨迹优化

用途：支撑“普通 local planner 可以跟踪路径、避障、平滑控制，但不显式考虑液体动态记忆”的论证。

核心定位：

- DWA、TEB、RPP、MPPI、MPC local planner 等是外部 baseline 的主要来源。
- 这组文献主要用于说明移动机器人局部规划和轨迹优化已有成熟方法，但通常不包含液体模态状态，也不预测 residual sloshing。

论文包括：

- `Regulated Pure Pursuit for Robot Path Tracking`
- `Long-Term Dynamic Window Approach for Kinodynamic Local Planning in Static and Crowd Environments`
- `Universal Trajectory Optimization Framework for Differential Drive Robot Class`
- `Real-Time Multilevel Terrain-Aware Path Planning for Ground Mobile Robots in Large-Scale Rough Terrains`
- `Path Planning For Autonomous Mobile Robots: A Review`
- `Model Predictive Path Integral Control: From Theory to Parallel Computation`
- `Dynamic Risk-Aware MPPI for Mobile Robots in Crowds via Efficient Monte Carlo Approximations`
- `Efficient Trajectory Planning for Multiple Non-Holonomic Mobile Robots via Prioritized Trajectory Optimization`
- `Distributed Data-driven Predictive Control via Dissipative Behavior Synthesis`
- `Deep Reinforcement Learning for Motion Planning of Mobile Robots`
- `An Actor-Critic Reinforcement Learning Scheme For Reactive 3D Optimal Motion Planning Based on Fluid Dynamics`
- `A New Approach to Time-Optimal Path Parameterization based on Reachability Analysis`
- `Jerk-limited Real-time Trajectory Generation with Arbitrary Target States`

## 3. 后续建议输出文件

建议后续基于上述附件路径与分类，继续整理以下文件：

```text
docs/论文书写/书写思路/SPMPC_related_work_matrix.md
docs/论文书写/书写思路/SPMPC_claim_gap_evidence_matrix.md
docs/论文书写/书写思路/SPMPC_related_work_draft.md
```

其中优先级：

1. 先写 `SPMPC_related_work_matrix.md`，建立每篇论文在本文中的角色。
2. 再写 `SPMPC_claim_gap_evidence_matrix.md`，把 claim、文献支撑、实验支撑和审稿人质疑对应起来。
3. 最后写 `SPMPC_related_work_draft.md`，避免 Related Work 变成简单堆文献。
