# SPMPC 普通移动机器人局部规划器文献缺口检查

日期：2026-07-02  
角色：Agent B  
范围：站在论文作者视角，判断“普通移动机器人局部规划器 / MPC / MPCC”相关文献是否足以支撑 SPMPC 相关工作第一小节；本文件只做文献缺口判断和写作建议，不直接修改 `02_related_work.tex`。

## 1. 结论先行

- 如果**只引用当前 `02_related_work.tex` 第 58 行已经使用的 D 组文献**（`D01_Macenski2023`、`D02_Jian2023`、`D03_Zhang2025`、`D06_Williams2017`、`D07_Trevisan2025`、`D13_Berscheid2021`），相关工作第一小节的论证**还不够扎实**。这些文献能说明普通局部规划器、MPPI、轨迹优化和平滑轨迹生成已经成熟，但还缺少 DWA 原始论文、TEB、普通 MPC 局部规划器、MPCC 主干、移动机器人 MPCC 和局部规划器评价基准这些关键支点。
- 如果采用**当前 `references.bib` 中已经补入的 D15--D21 条目**，文献基础已经基本够用。也就是说，问题不再是“完全缺文献”，而是后续改正文时必须把这些文献真正用到 Related Work 第一小节中，而不是继续只引用旧的 D01/D02/D03/D06/D07/D13。
- 主文必须引用或至少应进入 Related Work 第一小节的文献类型是：DWA 原始论文、TEB、普通 MPC / `mpc_local_planner` 谱系、移动机器人 MPCC，以及 LT-DWA。如果 Method 中明确使用 MPCC 叙事，还应补一个 MPCC 路径进度或轮廓误差主干文献。
- RPP、MPPI、Ruckig、UTO、TOPPRA、DWPP 属于可选文献。它们能增强“普通方法已经能做路径跟踪、速度约束、风险、平滑性和时间效率”的背景，但不是证明 SPMPC 文献缺口的最小必需集合。
- 地形规划、DRL、多机器人轨迹优化、一般数据驱动预测控制和“流体启发”的运动规划不建议在主文第一小节展开。它们容易拉散主线，且不能直接支撑“标准 WMR 在线局部规划器通常不传播液体状态”这句话。
- 建议写作顺序是：**先确认并固定 `references.bib` 中 DWA、TEB、MPC 局部规划器、MPCC、MRPB 的 citation key，再重写 `02_related_work.tex` 第一小节**。否则正文中容易出现 D02 与 D14 混引，或 Obsidian 编号与 `references.bib` key 不一致的问题。
- 这些文献最终服务于论文中的核心论断是：**普通移动机器人局部规划器能够在线生成可执行底盘命令，并可优化安全性、效率、平滑性、路径进度或避障性能；但它们的预测状态通常围绕机器人和环境，不传播被运输液体的模态状态。因此，SPMPC 的贡献不是发明普通局部规划或首次把晃液放入 MPC，而是在标准 WMR 在线 MPCC 局部规划层中加入低阶晃液状态传播。**

## 2. 当前已有 D 组文献能支撑什么

| 文献 key | 方法或论文 | 属于哪类局部规划 / 运动生成方法 | 是否已在 references.bib | 是否适合主文引用 | 在 SPMPC 论文中的作用 |
|---|---|---|---|---|---|
| `D01_Macenski2023` | Regulated Pure Pursuit for Robot Path Tracking | RPP / 普通路径跟踪器 | 是 | 可选 | 支撑“普通路径跟踪器可以在线输出底盘速度命令，并根据曲率、碰撞启发和安全规则调节速度”。它适合作轻量路径跟踪背景，但不能单独支撑 MPC / MPCC 局部规划层。 |
| `D02_Jian2023` | Long-Term Dynamic Window Approach for Kinodynamic Local Planning in Static and Crowd Environments | LT-DWA / 长时域 DWA 类局部规划器 | 是 | 建议主文引用；若实验使用 LT-DWA 则必引 | 支撑“DWA 家族已经有现代长时域和图优化扩展，能够在线生成更安全、更可跟踪的局部运动”。它说明普通局部规划器可以很强，但仍不传播液体模态状态。注意：`D02_Jian2023` 是 LT-DWA，不是 D-CBF-MPC。 |
| `D03_Zhang2025` | Universal Trajectory Optimization Framework for Differential Drive Robot Class | 差速机器人轨迹优化 / UTO | 是 | 可选 | 支撑“普通差速机器人轨迹优化可以直接优化线速度、角速度及其积分形式，生成可行、平滑、高质量运动”。它可作为普通轨迹优化背景，但不是 SPMPC 缺口的最关键文献。 |
| `D04_Li2025` | Real-Time Multilevel Terrain-Aware Path Planning for Ground Mobile Robots in Large-Scale Rough Terrains | 地形感知路径规划 | 是 | 不建议主文第一小节展开 | 支撑移动机器人规划可扩展到地形和稳定性，但第一版 SPMPC 不纳入 terrain prior。主文展开它会偏离“液体状态预测”主线。 |
| `D05_SnchezIbez2021` | Path Planning for Autonomous Mobile Robots: A Review | 移动机器人路径规划综述 | 是 | 可选背景，不建议重点展开 | 可作为术语或综述背景，但不能替代 DWA、TEB、MPC、MPCC 这些具体同层方法。 |
| `D06_Williams2017` | Model Predictive Path Integral Control: From Theory to Parallel Computation | MPPI / 采样式 MPC | 是 | 可选到建议主文引用 | 支撑“MPC/MPPI 类方法可以在线处理非线性系统和复杂代价”。它有助于说明普通预测控制并不天然等于液体感知，只有显式加入液体状态后才会成为 slosh-aware。 |
| `D07_Trevisan2025` | Dynamic Risk-Aware MPPI for Mobile Robots in Crowds | 风险感知 MPPI | 是 | 可选 | 支撑“普通局部规划器甚至可以优化人群风险和不确定性”。但其风险对象是外部人群/障碍，而不是容器内液体动态记忆。 |
| `D08_Li2021` | Efficient Trajectory Planning for Multiple Non-Holonomic Mobile Robots via Prioritized Trajectory Optimization | 多非完整机器人轨迹优化 | 是 | 不建议主文第一小节展开 | 说明优化式运动生成可用于多机器人协调，但任务层级不是单个 WMR 携带开口液体的在线局部规划。 |
| `D09_Yan2024` | Distributed Data-driven Predictive Control via Dissipative Behavior Synthesis | 数据驱动预测控制理论 | 是 | 不建议主文展开 | 是控制理论背景，不是移动机器人局部规划器基线。 |
| `D10_Butyrev2019` | Deep Reinforcement Learning for Motion Planning of Mobile Robots | DRL 运动规划 | 是 | 不建议主文展开 | 可作为学习式运动规划背景，但若论文不做学习式基线，主文展开会分散论证。 |
| `D11_Malliaropoulos2024` | Actor-Critic RL for Reactive 3D Optimal Motion Planning Based on Fluid Dynamics | “流体启发”的三维反应式规划 | 是 | 不建议主文展开 | 这里的“流体”是规划场类比，不是被运输液体晃动。主文展开容易造成术语混淆。 |
| `D12_Pham2017` | TOPPRA / time-optimal path parameterization | 固定路径速度剖面 | 是 | 可选 | 支撑“给定路径上可以做时间最优速度剖面”。它适合补充 smooth-only 或 speed-profile 对照，但不是在线局部规划层的主支点。 |
| `D13_Berscheid2021` | Jerk-limited Real-time Trajectory Generation with Arbitrary Target States | jerk-limited 在线轨迹生成 | 是 | 可选 | 支撑“普通运动生成可以实时满足速度、加速度和 jerk 约束，从而获得平滑命令”。它有助于说明“平滑不等于防晃”，但不传播液体模态状态。 |
| `D14_Jian2023` | Dynamic Control Barrier Function-based MPC to Safety-Critical Obstacle-Avoidance of Mobile Robot | D-CBF-MPC / 安全关键避障叙事参考 | 是 | 不建议放入普通局部规划器第一小节的主线 | 这是 D-CBF-MPC 叙事参考，不是 LT-DWA。可用于安全约束或叙事方式参考，但不能替代 DWA、TEB 或普通 MPC 局部规划器文献。 |
| `D15_Fox1997` | The Dynamic Window Approach to Collision Avoidance | DWA 原始论文 / 速度空间局部规划器 | 是 | 必引 | 支撑“经典局部规划器可以在线选择满足动力学和避障约束的底盘速度命令”。它是证明 ordinary local planner 在线可执行的基础文献之一。 |
| `D16_Rosmann2017` | Integrated Online Trajectory Planning and Optimization in Distinctive Topologies | TEB / 拓扑感知在线轨迹优化 | 是 | 必引 | 支撑“优化式局部规划器可在线处理非完整约束、速度/加速度限制、障碍距离和不同拓扑候选”。它是 TEB 基线和普通优化式局部规划器的关键文献。 |
| `D17_Rosmann2021` | Online Motion Planning based on Nonlinear MPC with Non-Euclidean Rotation Groups | 普通 NMPC 局部规划器 / `mpc_local_planner` 谱系 | 是 | 建议必引 | 支撑“普通 MPC 局部规划器可以在线处理非线性模型、约束和位姿误差”。ROS Index 与仓库页面均将 `mpc_local_planner` 指向该论文；它直接服务于“SPMPC 不是普通 MPC 加平滑项，而是在局部 MPC/MPCC 中加入液体状态”。 |
| `D18_Lam2013` | Model Predictive Contouring Control for Biaxial Systems | MPCC / 轮廓误差控制基础 | 是 | 可选；Method 中可引 | 支撑“轮廓误差、滞后误差和路径进度优化是 MPCC 中已有概念”。当前缺少对应 Obsidian 单篇笔记，正式使用前建议再核验原文。 |
| `D19_Liniger2015` | Optimization-Based Autonomous Racing of 1:43 Scale RC Cars | 移动系统 MPCC / 路径进度优化 | 是 | 建议主文或 Method 引用 | 支撑“MPCC 可在移动系统中实时联合优化路径进度、跟踪误差和约束”。它能支撑 SPMPC 的 MPCC 主干，但不涉及液体运输。 |
| `D20_Brito2019` | Model Predictive Contouring Control for Collision Avoidance in Unstructured Dynamic Environments | 移动机器人 MPCC / 动态环境避障 | 是 | 必引 | 支撑“MPCC 已可作为移动机器人在线局部规划和避障主干”。它是保护 novelty 的关键文献：SPMPC 的贡献不是首次把 MPCC 用于移动机器人，而是在该层加入液体模态状态。 |
| `D21_Wen2021` | MRPB 1.0 | 局部规划器评价基准 | 是 | 主文可选，实验章节建议引用 | 支撑“普通局部规划器评价通常关注安全性、效率和平滑性，而不评价开口液体的内部响应”。它适合解释为什么 SPMPC 需要额外的 slosh 指标。 |
| 待定 / Obsidian `D20` | DWPP: Dynamic Window Pure Pursuit | DWA + Pure Pursuit / 速度与加速度约束路径跟踪 | 否 | 暂不建议主文引用 | 可补充说明普通路径跟踪器正在更显式地考虑速度和加速度约束，但当前 `references.bib` 无条目，信息应标注待核验。 |

## 3. 可能缺失的经典局部规划文献

### 3.1 DWA

- **是否已有**：当前 `references.bib` 已有 `D15_Fox1997`；Obsidian D 组也有 DWA 原始论文笔记，但 Obsidian 编号是 D14。
- **是否建议补**：如果从旧正文引用集合看，必须补；如果按当前 `references.bib`，则应保留并在正文中实际引用。
- **推荐补哪篇**：Fox、Burgard、Thrun 的 *The Dynamic Window Approach to Collision Avoidance*。
- **它在本文中的作用**：作为 DWA 家族源头，支撑“普通局部规划器能够在线生成满足动力学和避障约束的可执行速度命令”。
- **它支撑的论文表述**：DWA 类方法在速度空间内选择可执行且安全的底盘速度，是移动机器人在线局部规划的经典路线；但其评价目标围绕机器人运动和障碍安全，不包含被运输液体的模态位移和速度。

### 3.2 TEB

- **是否已有**：当前 `references.bib` 已有 `D16_Rosmann2017`；Obsidian D 组中对应 TEB 笔记编号为 D19。
- **是否建议补**：必须补入主文引用。TEB 是 ROS 局部规划基线中最常被审稿人预期看到的优化式方法之一。
- **推荐补哪篇**：Rösmann、Hoffmann、Bertram 的 *Integrated Online Trajectory Planning and Optimization in Distinctive Topologies*。
- **它在本文中的作用**：支撑“普通优化式局部规划器已经能在线处理非完整约束、速度/加速度限制、障碍距离和拓扑不同的候选轨迹”。
- **它支撑的论文表述**：TEB 等优化式局部规划器可以在线生成满足运动学和避障约束的局部轨迹，但其预测变量仍是机器人轨迹和时间间隔，不包含开口容器内液体的动态状态。

### 3.3 mpc_local_planner

- **是否已有**：当前 `references.bib` 已有 `D17_Rosmann2021`，Obsidian 中也有对应 NMPC 局部规划笔记。
- **是否建议补**：建议补入主文，尤其当实验或叙事中提到 `mpc_local_planner` 或 ordinary MPC 局部规划器时。
- **推荐补哪篇**：Rösmann、Makarow、Bertram 的 *Online Motion Planning based on Nonlinear Model Predictive Control with Non-Euclidean Rotation Groups*。
- **它在本文中的作用**：支撑“普通 MPC 局部规划器已经能够在线处理非线性机器人模型、约束和位姿误差”。这样才能把 SPMPC 的差异写清楚：SPMPC 不是“普通 MPC 加平滑项”，而是把液体模态状态也放进预测模型。
- **它支撑的论文表述**：普通 MPC 局部规划器能够在滚动时域内优化底盘运动，但若状态中没有液体模态位移和速度，它仍不能显式处理被运输液体的动态记忆。
- **核验结果**：当前 Obsidian 将该论文作为 `mpc_local_planner` 谱系参考；ROS Index 与仓库页面也将 `mpc_local_planner` 指向该论文。正式引用建议以 ECC 论文为主，ROS 页面只作为本地追踪信息，不作为主文引用。

### 3.4 MPCC / 轮廓误差局部规划主干

- **是否已有**：当前 `references.bib` 已有 `D18_Lam2013` 和 `D19_Liniger2015`。Obsidian 中已整理 Liniger，Lam 目前未见单独精读笔记。
- **是否建议补**：建议至少引用一个 MPCC 主干文献。若 Method 中解释 contouring error、lag error、路径进度和 `s, v_s`，则更应引用。
- **推荐补哪篇**：优先用 `D19_Liniger2015` 支撑移动系统上的路径进度优化；若要强调一般 MPCC 轮廓误差定义，可补 `D18_Lam2013`，但正式使用前建议核验原文。
- **它在本文中的作用**：支撑“SPMPC 是局部规划层的 MPCC，而不是单纯给定轨迹跟踪控制器”。MPCC 主干说明路径进度本身可以进入优化，而不是只跟踪预先给定的时间参数轨迹。
- **它支撑的论文表述**：MPCC 已经提供了将路径跟踪误差、路径进度和约束放入同一滚动优化问题的主干；SPMPC 在该主干上加入低阶晃液状态传播和液体响应代价。

### 3.5 移动机器人 MPCC

- **是否已有**：当前 `references.bib` 已有 `D20_Brito2019`；Obsidian 中对应移动机器人 MPCC 笔记编号为 D18。
- **是否建议补**：必须补。它是保护 novelty 的关键文献。
- **推荐补哪篇**：Brito、Floor、Ferranti、Alonso-Mora 的 *Model Predictive Contouring Control for Collision Avoidance in Unstructured Dynamic Environments*。
- **它在本文中的作用**：说明移动机器人语境下的 MPCC 局部规划已经存在，并且可以在线处理路径跟踪、路径进度和避障。因此 SPMPC 不能把“MPCC 用于移动机器人局部规划”写成贡献。
- **它支撑的论文表述**：已有移动机器人 MPCC 可以在线优化路径跟踪、路径进度和障碍约束，但其预测状态仍主要围绕机器人和障碍物；SPMPC 的差异在于加入被运输液体的低阶模态状态。

### 3.6 局部规划器评价基准

- **是否已有**：当前 `references.bib` 已有 `D21_Wen2021`；Obsidian 中对应 MRPB 笔记编号为 D17。
- **是否建议补**：建议补，但不一定必须放在 Related Work 第一小节。若篇幅紧张，可放到实验设计或评价指标小节。
- **推荐补哪篇**：Wen 等的 *MRPB 1.0: A Unified Benchmark for the Evaluation of Mobile Robot Local Planning Approaches*。
- **它在本文中的作用**：支撑“普通局部规划器评价通常围绕安全性、效率和平滑性展开”，从而引出 SPMPC 还需要评价预测液体响应或外部液面指标。
- **它支撑的论文表述**：标准局部规划器基准可以评价导航层面的安全、效率和平滑，但这些指标不能替代开口液体运输中的液体响应评价。

## 4. 建议补入或保留在 references.bib 的条目

说明：以下条目在当前 `references.bib` 中已经存在。这里的“必补”是从论文写作判断出发：如果后续换成干净 bib 或重排 key，这些条目不应丢失。

### 4.1 主文建议必补

#### DWA 原始论文

- 推荐引用：`D15_Fox1997`
- 为什么必须补：没有 DWA 原始论文，DWA 家族只引用 LT-DWA 会显得谱系不完整。
- 支撑论断：普通移动机器人局部规划器可以在线输出满足动力学和避障约束的可执行速度命令，但不传播液体状态。
- BibTeX 候选：

```bibtex
@article{D15_Fox1997,
  title = {{The Dynamic Window Approach to Collision Avoidance}},
  author = {Dieter Fox and Wolfram Burgard and Sebastian Thrun},
  year = {1997},
  journal = {IEEE Robotics \& Automation Magazine},
  volume = {4},
  number = {1},
  pages = {23--33},
  doi = {10.1109/100.580977},
  keywords = {SPMPC,D15,classic-DWA,local-planner,baseline}
}
```

#### TEB

- 推荐引用：`D16_Rosmann2017`
- 为什么必须补：TEB 是普通优化式局部规划器的代表，也是 ROS 语境下常见基线。
- 支撑论断：普通优化式局部规划器可在线优化可行轨迹和避障约束，但其预测变量仍是机器人轨迹而非液体模态状态。
- BibTeX 候选：

```bibtex
@article{D16_Rosmann2017,
  title = {{Integrated Online Trajectory Planning and Optimization in Distinctive Topologies}},
  author = {Christoph R{\"o}smann and Frank Hoffmann and Torsten Bertram},
  year = {2017},
  journal = {Robotics and Autonomous Systems},
  volume = {88},
  pages = {142--153},
  doi = {10.1016/j.robot.2016.11.007},
  keywords = {SPMPC,D16,TEB,local-planner,baseline}
}
```

#### ordinary MPC / `mpc_local_planner` 谱系

- 推荐引用：`D17_Rosmann2021`
- 为什么必须补：SPMPC 的方法层级是 MPC/MPCC 局部规划，如果没有普通 MPC 局部规划器作为对照，审稿人会觉得缺少同层参照。
- 支撑论断：普通 MPC 局部规划器可以在线处理非线性约束，但不显式加入液体状态时不能处理液体动态记忆。
- BibTeX 候选：

```bibtex
@inproceedings{D17_Rosmann2021,
  title = {{Online Motion Planning based on Nonlinear Model Predictive Control with Non-Euclidean Rotation Groups}},
  author = {Christoph R{\"o}smann and Artemi Makarow and Torsten Bertram},
  year = {2021},
  booktitle = {2021 European Control Conference (ECC)},
  pages = {1583--1590},
  doi = {10.23919/ECC54610.2021.9654872},
  eprint = {2006.03534},
  archivePrefix = {arXiv},
  primaryClass = {cs.RO},
  keywords = {SPMPC,D17,mpc-local-planner,NMPC,ROS},
  ros_index_url = {https://index.ros.org/r/mpc_local_planner/}
}
```

- 待核验：是否在正文中直接称其为 `mpc_local_planner` 官方论文，建议再核对 ROS 文档。

#### 移动机器人 MPCC

- 推荐引用：`D20_Brito2019`
- 为什么必须补：它能避免 SPMPC novelty 被写歪。移动机器人 MPCC 局部规划已存在，SPMPC 的新意是加入液体状态。
- 支撑论断：MPCC 已可用于移动机器人在线局部规划和动态避障，但普通 MPCC 不传播被运输液体模态状态。
- BibTeX 候选：

```bibtex
@article{D20_Brito2019,
  title = {{Model Predictive Contouring Control for Collision Avoidance in Unstructured Dynamic Environments}},
  author = {Bruno Brito and Boaz Floor and Laura Ferranti and Javier Alonso-Mora},
  year = {2019},
  journal = {IEEE Robotics and Automation Letters},
  volume = {4},
  number = {4},
  pages = {4459--4466},
  doi = {10.1109/LRA.2019.2929976},
  keywords = {SPMPC,D20,MPCC,mobile-robot,collision-avoidance}
}
```

#### LT-DWA

- 推荐引用：`D02_Jian2023`
- 为什么必须补：若实验或正文对比 LT-DWA，它必须作为现代 DWA-family 局部规划器引用。
- 支撑论断：DWA 家族不仅有经典版本，也有面向长时域和复杂环境的现代变体；但它仍不处理被运输液体状态。
- BibTeX 状态：当前 `references.bib` 已有，信息已核验到 DOI、期刊卷期和页码。
- 特别注意：不要误用 `D14_Jian2023`。`D14_Jian2023` 是 D-CBF-MPC 叙事参考。

### 4.2 主文建议可选或放在 Method / 实验背景

#### MPCC 路径进度主干

- 推荐引用：`D19_Liniger2015`
- 为什么可选但很有价值：它能支撑“路径进度优化是 MPCC 主干中的成熟做法”，适合放在 Related Work 或 Method 中。
- 支撑论断：SPMPC 的 `s, v_s` 和路径进度优化来自 MPCC 层级，不是简单轨迹跟踪。
- BibTeX 候选：

```bibtex
@article{D19_Liniger2015,
  title = {{Optimization-Based Autonomous Racing of 1:43 Scale RC Cars}},
  author = {Alexander Liniger and Alexander Domahidi and Manfred Morari},
  year = {2015},
  journal = {Optimal Control Applications and Methods},
  volume = {36},
  number = {5},
  pages = {628--647},
  doi = {10.1002/oca.2123},
  keywords = {SPMPC,D19,MPCC,path-progress,autonomous-racing}
}
```

#### 一般 MPCC 轮廓误差基础

- 推荐引用：`D18_Lam2013`
- 为什么可选：如果正文只用 Brito 和 Liniger，MPCC 主线已经够用；如果要更规范解释 contouring/lag error，可引用 Lam。
- 待核验：当前 `references.bib` 有条目，但 Obsidian 中未见单篇笔记，正式主文引用前建议再读原文。

#### MRPB 局部规划器评价基准

- 推荐引用：`D21_Wen2021`
- 为什么可选但建议保留：它能支撑 baseline 和评价指标设计，尤其是“普通局部规划器指标不覆盖液体响应”。
- 更适合位置：Related Work 第一小节短提，或实验章节评价指标背景。
- BibTeX 候选：

```bibtex
@inproceedings{D21_Wen2021,
  title = {{MRPB 1.0: A Unified Benchmark for the Evaluation of Mobile Robot Local Planning Approaches}},
  author = {Jian Wen and Xuebo Zhang and Qingchen Bi and Zhangchao Pan and Yanghe Feng and Jing Yuan and Yongchun Fang},
  year = {2021},
  booktitle = {2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages = {8238--8244},
  doi = {10.1109/ICRA48506.2021.9561901},
  keywords = {SPMPC,D21,benchmark,local-planner,MRPB}
}
```

### 4.3 可放附录、实验背景或暂不引用

- `D01_Macenski2023`：RPP。适合作轻量路径跟踪基线或背景；不是第一小节最关键支点。
- `D03_Zhang2025`：UTO。可说明差速机器人普通轨迹优化成熟；不必重点展开。
- `D06_Williams2017`：MPPI。可用于扩展普通 MPC/MPC-like 方法谱系；若篇幅有限可短提。
- `D13_Berscheid2021`：Ruckig。适合说明 smooth-only 或 jerk-limited 运动生成；不必作为主文核心。
- `D12_Pham2017`：TOPPRA。若设计固定路径速度剖面对照，可在实验背景或附录使用。
- DWPP：当前仅在 Obsidian 中有 note，`references.bib` 未见条目；除非加入 DWPP 或 Nav2 controller 对比，否则暂不引用，信息标注待核验。

## 5. 相关工作第一小节建议写法

普通移动机器人在线局部规划器已经形成了较成熟的方法谱系。经典 DWA 在速度空间中选择满足动力学和避障约束的可执行底盘命令，后续 LT-DWA 等方法进一步扩展规划时域并改善复杂环境中的局部运动生成。TEB 等优化式局部规划器能够在线优化带时间信息的轨迹，并同时考虑非完整约束、速度和加速度限制、障碍距离以及不同拓扑候选。RPP、Ruckig 等路径跟踪或在线轨迹生成方法则从曲率调速、速度约束、加速度约束和 jerk-limited 平滑性等角度提高底盘命令的可执行性。与此同时，MPC、MPPI 和 NMPC 局部规划器已经能够在滚动时域内处理非线性模型、控制约束和复杂代价；MPCC 进一步将路径进度、轮廓误差和滞后误差纳入统一优化框架，并已用于移动机器人动态环境避障。

这些工作说明，将 SPMPC 放在标准 WMR 的在线局部规划层是合理的：同层方法本来就需要在每个控制周期生成可执行的底盘速度或短时轨迹，并在路径跟踪、安全性、效率和平滑性之间折中。然而，普通局部规划器的预测状态和代价通常围绕机器人位姿、速度、路径几何、障碍距离、碰撞风险、控制平滑性或路径进度构建。即使某些方法采用 MPC、MPPI、NMPC 或 MPCC 形式，只要没有把被运输液体的模态位移和模态速度作为预测状态传播，规划器就无法显式处理液体的动态记忆，也无法区分“底盘运动同样平滑、但与当前晃液相位相互作用不同”的控制序列。因此，普通局部规划器适合作为 SPMPC 的同层基线和方法背景，但它们原本并不是为开口液体运输中的液体状态预测而设计的。

SPMPC 的贡献不在于首次提出 DWA、TEB、MPC、MPPI 或 MPCC，也不在于首次将晃液状态放入 MPC。本文关注的是两条研究线之间的交叉缺口：普通移动机器人局部规划器在线、可执行，但通常不具备液体状态预测能力；已有防晃方法显式考虑液体动态，但多数不位于标准 WMR 导航栈的在线局部规划层。SPMPC 因此在标准 WMR 的在线 MPCC 局部规划层中嵌入低阶晃液状态，使规划器在滚动时域内联合优化路径跟踪、路径进度、底盘控制平滑性和预测液体响应，并输出普通 `/cmd_vel` 命令。

## 6. 推荐引用优先级

### 6.1 主文第一小节建议必引

- **DWA**：建议引用 `D15_Fox1997`；若正文还讨论 LT-DWA，则同时引用 `D02_Jian2023`。作用是建立“速度空间在线局部规划器可输出可执行命令”的经典支点。
- **TEB**：建议引用 `D16_Rosmann2017`。作用是建立“优化式在线局部规划器可处理轨迹、时间、约束和障碍”的支点。
- **MPC local planner / `mpc_local_planner`**：建议引用 `D17_Rosmann2021`。ROS Index 与仓库页面均指向该论文，作用是建立“普通 MPC 局部规划器已存在，但不天然包含液体状态”。
- **移动机器人 MPCC**：建议引用 `D20_Brito2019`。作用是建立“MPCC 已可作为移动机器人局部规划主干，SPMPC 的新意是加入液体模态状态”。
- **LT-DWA**：建议引用 `D02_Jian2023`，尤其当实验或正文把 LT-DWA 作为基线时。必须明确它不是 `D14_Jian2023`。

### 6.2 主文第一小节可选引用

- **RPP**：`D01_Macenski2023`。适合补充“轻量路径跟踪器也能在线输出底盘命令”。
- **MPPI**：`D06_Williams2017`。适合补充“预测控制类方法可在线处理复杂代价”。若篇幅紧张可不展开。
- **MPCC 路径进度主干**：`D19_Liniger2015`。适合与 `D20_Brito2019` 搭配，说明路径进度优化和 MPCC 主干成熟。
- **一般 MPCC 轮廓误差基础**：`D18_Lam2013`。适合 Method 或附录解释 contouring/lag error；正式引用前建议核验原文。
- **Ruckig / jerk-limited 生成**：`D13_Berscheid2021`。适合说明 smooth-only 运动生成很成熟，但不等于液体状态预测。
- **MRPB**：`D21_Wen2021`。若第一小节需要一句评价框架背景可引用；否则更适合实验章节。

### 6.3 不建议主文第一小节展开

- `D04_Li2025`：地形感知规划，偏离第一版 SPMPC 范围。
- `D05_SnchezIbez2021`：综述背景，可不用在第一小节展开。
- `D07_Trevisan2025`：风险感知 MPPI，可选背景但不是核心支点。
- `D08_Li2021`：多机器人轨迹优化，任务层级不同。
- `D09_Yan2024`：一般数据驱动预测控制，不是移动机器人局部规划器基线。
- `D10_Butyrev2019`：DRL 运动规划，除非有实验基线，否则不展开。
- `D11_Malliaropoulos2024`：“流体启发”规划，不是被运输液体晃动，容易混淆。
- `D14_Jian2023`：D-CBF-MPC 叙事参考，不应与 `D02_Jian2023` 混用；不建议放在普通局部规划器第一小节作为主要证据。

### 6.4 对基线集合的建议

- 若主文实验或仿真基线空间有限，普通局部规划器基线建议至少覆盖：**DWA / LT-DWA、TEB、MPC local planner**。这三类分别代表速度空间方法、优化式局部规划和普通 MPC 局部规划。
- RPP 可以作为轻量路径跟踪补充基线；MPPI 可作为预测控制/采样控制补充基线；Ruckig 或 TOPPRA 更适合作 smooth-only 或速度剖面补充，而不是第一层主基线。
- MPCC 文献主要用于支撑 SPMPC 的方法层级和 novelty 边界；若没有可复现实装，Brito 或 Liniger 不一定要作为实验基线，但应该作为 Related Work / Method 引用。
- MRPB 更像评价框架参考，而不是单个基线。它适合支撑“普通局部规划器评价 safety、efficiency、smoothness；SPMPC 还需要 liquid-response 评价”的写作逻辑。
