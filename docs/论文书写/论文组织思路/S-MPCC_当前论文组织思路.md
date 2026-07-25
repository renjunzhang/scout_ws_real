# S-MPCC 当前论文组织思路

> 本文档是整篇 S-MPCC 论文的章节、主张和证据组织总纲，以 `docs/论文书写/草稿/spmpc_paper_core` 的当前方法定义为准。
>
> 实验章节的唯一上位协议是 `docs/论文书写/实验章节设计/S-MPCC_experimental_design.md`，正式次数和执行速查以 `docs/论文书写/实验章节设计/SPMPC实验矩阵设计.md` 为准。本文件只保留论文级实验摘要和证据映射；若实验细节不一致，以实验总章、矩阵速查表和正式 freeze artifacts 为准。
>
> K6 模型—视觉一致性与敏感性分析的唯一详细口径是 `docs/论文书写/实验章节设计/K6_Ferrari模型视觉一致性冻结协议.md`（K6-FID-v1.0）。该协议已经冻结分析总体、信号、公式、窗口、同步、敏感性与失败规则；正式分析脚本、同步标定和 freeze manifest 通过 no-go 检查前，仍不得开始正式采集。
>
> **当前方法版本冻结说明：** 本轮 88 次正式实物实验遵循“先冻结、后采集”，全部绑定同一个实验 release。首个正式 trial 前应一次性冻结当前 S-MPCC 液体动力学与状态传播、代价结构与权重、三方法的预声明差异、两条路径、\(C_1/C_2\) 参数、执行代码与限制以及统计规则。40/64 次只是阶段性证据包和资源检查点，不是根据中间结果修改动力学、代价、权重、路径或分析口径的调参节点。若正式采集中途确需改方法，已采数据只能保留为旧版本正式数据或重新标记为 pilot，不能与新版本拼接成同一组 88 次；新版本必须另行冻结并独立重新采集。`S-MPCC_旋转一致液体状态与相位能量优化推进方案.md` 中的 rotation-consistent dynamics、相位能量/有符号功率等改动属于后续独立方法版本，不进入本轮矩阵。
>
> **探索性诊断说明：** `Long-horizon propagation and command-regularity audit` 降级为第二层探索性诊断，不属于 40/64/88 次正式证据包，也不再作为 96/104 次正式扩展计数。它不是 RQ1–RQ4、三项核心贡献或投稿完整性的必要条件。只有主实验、计算实验和常规日志审计完成后仍有资源，且扩展路径、安全运行、连续状态传播、视觉链和分层命令日志全部通过独立预验收时才考虑执行。默认投稿稿不设置该模块的 pending 小节；若完成且具有独立解释价值，优先放入补充材料，正文最多作一句受限说明。
>
> 旧文档中出现的 `B_ours`、四格因子实验、Reference Governor 核心消融、modal hard cap、`NoConstraint/NoCost` 核心消融、CA-MPCC inspired baseline、终端独有贡献和外部 planner 排名，不再属于当前论文主线。

## 1. 论文的核心定位

### 1.1 一句话定义

S-MPCC 是一个沿预先给定几何可行路径运行的在线滚动时域局部运动规划与控制方法。它在同一 MPCC 预测时域内联合优化机器人运动、虚拟路径进度和传播的液体动态，并执行第一步优化控制。

建议在英文正文中统一写为：

> S-MPCC is an online receding-horizon local motion planner and controller that jointly optimizes robot motion, virtual path progress, and propagated liquid dynamics along a prescribed geometrically feasible path.

论文名称使用 `S-MPCC`，强调其本质是 slosh-aware model predictive contouring control。代码、ROS 包和工程目录仍可沿用 `spmpc`，但不据此将论文方法改名为 SPMPC。

### 1.2 核心科学问题

普通平滑或整体减速只能约束运动变化，不能显式表示液体当前的模态位移、模态速度和振荡相位。相同机器人状态和相似平滑程度的候选运动，在不同液体相位下可能导致不同的未来液面响应。

本文要回答的核心问题是：

> 在给定几何可行路径的条件下，能否把液体动态记忆放入在线 MPCC 决策层，使移动底盘根据当前液体状态重新分配速度和加速度，并降低实际测得的液面响应？

### 1.3 总论证主线

全文应围绕以下逻辑展开：

1. 开放液体运输不是单纯的轨迹平滑问题，而是具有动态记忆的状态预测问题。
2. 现有通用局部规划器通常不传播液体状态；许多防晃方法则工作在离线路径、速度剖面、整段轨迹优化或参考跟踪层。
3. S-MPCC 将机器人、路径进度和低阶液体模态状态放入同一滚动时域 OCP。
4. 内部比较先证明显式液体机制相对普通 MPCC 和通用增强平滑的作用。
5. 等完成时间实验排除“只是更慢”的替代解释。
6. 跨容器实验检验基于几何参数重计算的迁移能力。
7. 纵向/横向四相位规划与实际传播状态反事实 replay 共同证明液体动态记忆确实改变优化动作，而不只是生成固定速度曲线。
8. 全部正式日志的实时性与执行层干预统计说明在线计划能否按控制周期可靠落地。

实验章节的证据顺序应保持为：

\[
\text{是否有效}
\rightarrow
\text{运动与激励为何改变}
\rightarrow
\text{是否只是更慢}
\rightarrow
\text{能否跨容器}
\rightarrow
\text{传播状态是否真正改变优化}
\rightarrow
\text{是否实时且被实际执行}.
\]

长时传播与命令规律性审计只保留为主证据链完成后的探索性诊断，优先进入补充材料，不能成为摘要、贡献列表、主结论或实验完整性的前置条件。

### 1.4 当前允许的主要主张

正式实验完成后，论文可以谨慎主张：

> S-MPCC uses propagated liquid dynamics to generate state-dependent receding-horizon motion plans online and improves the measured liquid response relative to matched slosh-agnostic and smooth-only MPCC variants across the reported paths and container geometries.

这个主张只在正式数据支持的条件下成立，并且必须限定于：

- 预先给定的几何可行路径；
- 当前两条实验路径；
- 容器安装在底盘旋转中心；
- 当前低阶液体模型和冻结的执行配置；
- Baseline MPCC、Smooth-only MPCC 与 S-MPCC 三种匹配内部方法。

### 1.5 不应扩大的主张

正文不得把当前方法描述为：

- 全局路径规划器或完整自主导航系统；
- 已包含 occupancy grid、动态障碍预测或碰撞走廊的避障 planner；
- 首个使用液体状态的 MPC；
- 首个移动机器人防晃预测控制方法；
- 高保真自由液面模拟器；
- 使用 RGB 液面反馈的闭环控制器；
- 具有严格无溢出、稳定性或递归可行性保证的方法；
- 将 Lim 2024 描述为一维临界加速度梯形速度曲线，或声称其提供真实液体防溢保证；
- 已经优于 Hamaguchi、Lim、DWA、TEB 或其他未按统一协议正式比较的方法。

## 2. 全文结构总览

完整论文采用五个正文部分，实验结果和解释按研究问题合并，不另设一个内容重复的通用 Discussion：

| 位置     | 章节                       | 本章核心作用                               |
| -------- | -------------------------- | ------------------------------------------ |
| 前置部分 | Title、Abstract、Keywords  | 用最短篇幅交代问题、方法、证据和结论       |
| I        | Introduction               | 建立问题、缺口、方法定位、贡献和适用边界   |
| II       | Related Work               | 从物理任务和决策层两条轴线定位 S-MPCC      |
| III      | S-MPCC Method              | 给出可复现的机器人—液体增广预测和在线 OCP |
| IV       | Experimental Evaluation    | 按 RQ1–RQ4 建立完整实验因果与可行性证据链 |
| V        | Conclusion and Limitations | 汇总得到支持的结论，并明确不能外推的范围   |
| 补充材料 | Supplementary Material     | 保存完整配置、正式矩阵、开发数据和详细诊断 |

章节之间的逻辑关系为：

> Introduction 提出动态记忆问题；Related Work 证明这个问题位于现有研究的交叉缺口；Method 说明如何把液体记忆放入 MPCC；Experiments 检验这一机制是否有效、是否只是减速、能否迁移、能否实时；Conclusion 只总结已经被证据支持的部分。

## 3. Title、Abstract 与 Keywords

### 3.1 标题

当前推荐标题：

> **S-MPCC: Slosh-Aware Model Predictive Contouring Control for Open-Liquid Transport on Mobile Bases**

标题应突出：

- 方法是 S-MPCC，而不是泛称 SPMPC；
- 核心技术是 slosh-aware MPCC；
- 任务是移动底盘开放液体运输；
- 不在标题中加入 obstacle avoidance、spill-free 或 autonomous navigation。

### 3.2 摘要写什么

摘要最终采用“背景—缺口—方法—实验—结果—边界”的紧凑结构：

1. **背景：** 移动底盘的加速、制动和转向会激发开放液体晃动。
2. **缺口：** 通用平滑无法显式表示液体状态和相位，许多防晃方法又不在在线局部规划层决策。
3. **方法：** S-MPCC 联合预测机器人、虚拟路径进度和容器参数化的一阶液体模态，并传播液体动态记忆。
4. **实验：** 三种匹配内部方法、等完成时间对照、跨容器迁移、纵横向相位规划、实际传播状态反事实 replay 和实时性。
5. **结果：** 正式数据完成后填入 RGB p95、残余振荡、任务时间和求解时间的关键定量结果。
6. **边界：** 沿给定几何可行路径进行在线局部运动生成，不声称完整避障导航或严格防溢保证。

摘要中暂时不能写入 `pending` 数值，更不能用旧三次 pilot 数据代替正式结果。

### 3.3 Keywords

建议保留：

- mobile robots；
- open-liquid transport；
- liquid sloshing；
- model predictive contouring control；
- receding-horizon planning。

## 4. 第一章：Introduction

### 4.1 本章回答的问题

本章要让读者理解：为什么移动机器人开放液体运输值得研究，为什么普通平滑不足，以及本文究竟解决现有工作的哪一层问题。

### 4.2 推荐内容顺序

#### 第一段：任务背景与实际困难

介绍实验室、服务和工业场景中的开放液体运输。指出底盘加速、制动和转弯会激发自由液面振荡，进而增加稳定时间、降低运输可靠性并提高溢出风险。

这一段只建立问题重要性，不展开方法公式。

#### 第二段：核心科学困难

明确提出“液体动态记忆”：

- 液体响应不仅取决于当前速度、加速度或 jerk；
- 还取决于当前模态位移、模态速度和残余振荡相位；
- 因此相似平滑度的两段运动可能产生不同后续液面响应。

这一段是全文最重要的问题定义，应直接为增广状态和 RQ4 铺垫。

#### 第三段：现有研究的两条线及其缺口

简要概括：

- 防晃研究已经包括低阶建模、input shaping、速度剖面设计、轨迹优化和跟踪控制；
- 移动机器人局部规划已经包括 DWA、TEB、MPC 和 MPCC 等在线运动生成方法；
- 两条线通常位于不同决策层：前者常设计离线轨迹或参考，后者通常不传播液体状态。

这里不要展开完整文献比较，详细分析留给 Related Work。

#### 第四段：本文方法概述

给出 S-MPCC 的一句话定义，并说明三个关键动作：

1. 用 odometry 激励在线传播液体模态状态；
2. 在 MPCC horizon 内联合传播机器人、路径进度和液体状态；
3. 只执行第一步优化控制，然后在下一周期重新求解。

#### 第五段：贡献列表

建议保留三项贡献，但措辞必须具体且可由正文验证：

1. **Online liquid-dynamic-memory MPCC formulation：** 将一阶横向液体模态在两个正交方向上的状态加入 acceleration-level path-progress MPCC。
2. **Container-parameterized prediction and online propagation：** 给出圆柱容器几何到模态参数的明确映射，并基于 odometry 激励持续传播内部液体状态，使当前模态相位能够影响未来优化动作。
3. **Matched physical validation of state-dependent slosh-aware planning：** 使用三种匹配内部方法、独立等完成时间对照、无权重重调的跨容器迁移、纵横向四相位规划、实际传播状态反事实 replay 和视觉物理测量，区分液体机制、普通平滑和整体减速。

正式数据完成前，第三项仍可暂写为 matched experimental evidence structure；投稿版本必须改为由正式结果支持的 matched physical validation，不应把“设计了一套实验”本身包装成新的控制理论贡献。

三项贡献的证据映射固定为：

\[
\begin{aligned}
\mathrm{C1}&\leftarrow \mathrm{RQ1+RQ2+RQ4+runtime},\\
\mathrm{C2}&\leftarrow \mathrm{RQ3+RQ4+counterfactual\ replay},\\
\mathrm{C3}&\leftarrow \mathrm{RQ1\mbox{--}RQ4\ as\ a\ matched\ whole}.
\end{aligned}
\]

#### 第六段：范围声明

明确说明：

- 输入路径已经几何可行；
- 当前 OCP 不负责全局路径和障碍推理；
- RGB 是实验参考测量，不进入控制闭环；
- 当前比较对象限于三个匹配内部方法；
- 不提供严格 spill-free guarantee。

### 4.3 本章图表

Introduction 可保留一张紧凑的“研究定位图”，表达：

\[
\text{online local planning}
+
\text{liquid-aware motion design}
\rightarrow
\text{S-MPCC}.
\]

该图只解释研究交叉位置，不能与 Method 中的完整闭环架构图重复。

### 4.4 本章不应写入

- 完整 OCP 公式；
- 所有容器参数公式；
- solver 细节；
- 88 次矩阵的细节；
- 旧三次开发数据；
- Reference Governor、hard cap 等可选模块；
- “首次 slosh-aware MPC”一类难以成立的优先权主张。

### 4.5 向下一章的过渡

本章结尾应让读者带着一个问题进入 Related Work：已有工作分别覆盖了在线局部规划和防晃控制，S-MPCC 与它们在物理任务和决策层上究竟有何不同？

## 5. 第二章：Related Work

### 5.1 本章回答的问题

本章不是简单证明“别人没有做过”，而是精确确定本文的创新边界：哪些方法与本文物理任务相近，哪些方法与本文决策层相近，S-MPCC 的具体组合差异在哪里。

### 5.2 组织原则

每类文献都按三步写：

1. 这类工作已经解决了什么；
2. 它与本文在哪个方面接近；
3. 它与本文在平台、输入输出或决策层上有何具体差异。

避免按照年份逐篇罗列，也不要用模糊的 “few studies” 或 “to the best of our knowledge” 代替具体比较。

### 5.3 推荐小节

#### 2.1 Online Local Planning and Trajectory Optimization for Mobile Robots

介绍 DWA、TEB、MPC、MPCC 及相关实时运动生成方法。重点说明：

- 它们与本文处于相近的在线运动决策层；
- 能处理路径进度、跟踪、运动约束、平滑和实时求解；
- 但通常不传播容器内液体的模态位移和速度。

本节用于说明 MPCC backbone 的来源以及“普通平滑不等于液体动态预测”。当前实验不对这些 obstacle-oriented planners 做排行榜比较。

#### 2.2 Mobile-Base Liquid Transport and Anti-Slosh Methods

介绍与移动底盘液体运输最接近的研究，包括：

- Hamaguchi 系列的路径、速度剖面、input shaping 和跟踪；
- Lim 等的液体约束离线轨迹优化；
- 规划加跟踪、特殊平台或主动防晃机构；
- 其他移动平台上的 slosh-aware MPC 或 tracking control。

比较重点应落在决策层：S-MPCC 每个控制周期直接联合选择路径进度和底盘运动，而不是预先生成完整 profile 后再跟踪。

可以保留一张“决策层比较表”，但只能依据已核实文献事实填写，不能把 inspired baseline 当成原论文的严格复现。

Lim 2024 的事实边界必须写准确：该工作使用球摆动力学进行二维移动机器人整段轨迹优化，通过配点法离线生成线速度、角速度和位姿轨迹；其主要局限是整段预先求解、目标时间与终点预先给定、运行时不根据当前液体状态滚动重规划。不能把它改写成“一维临界加速度梯形速度曲线”。若把

\[
\dot v^2+(v\omega)^2\le a_R^2
\]

嵌入在线 MPCC，这应称为本文团队另行设计的 Lim-inspired CA-MPCC 或低复杂度合加速度约束方法，而不是 Lim 原方法的复现。该方向需要独立推导、实现和验证，当前只作为 future work 记录，不进入本论文核心 baseline。

#### 2.3 Cross-Platform Slosh Modeling, Trajectory Generation, and Predictive Control

介绍机械臂、SCARA、罐体、船舶或其他平台上的：

- CFD、VOF、FEM 等高保真模型；
- equivalent pendulum、MSD 和低阶模态模型；
- input shaping、轨迹生成和预测控制；
- 视觉液面作为外部实验依据的做法。

本节要说明低阶模型适合在线优化，但不是本文原创；Ferrari 和 Leva 等工作可支持建模与评价方法的合理性，但不能自动证明本论文 RGB 测量的准确性。

#### 2.4 Positioning of S-MPCC

用一段综合定位结束本章：

- 普通在线 planner 与本文决策层相近，但缺少液体状态；
- 移动底盘防晃研究与本文物理任务相近，但常工作在离线设计或 tracking 层；
- 跨平台防晃 MPC 提供重要基础，因此本文不声称首次使用 slosh state in MPC；
- 本文贡献是把低阶液体动态记忆嵌入标准轮式底盘的在线 path-progress MPCC。

### 5.4 本章图表

建议最多保留一张紧凑比较表，字段可包括：

- platform/task；
- liquid model；
- generation layer；
- online replanning；
- output interface；
- 与本文的主要差异。

表格不是 baseline 结果表，也不能暗示本文已经在实验上优于这些工作。

### 5.5 本章不应写入

- 未核实的“首次”声明；
- 把所有文献都批评成 offline 或不实时；
- 把特殊平台工作说成与本文无关；
- 用 Ferrari 的结果证明本论文 RGB 为绝对真值；
- 预先宣布本文性能优于尚未复现的 Hamaguchi/Lim 方法。
- 把自行设计的 CA-MPCC、临界加速度 heuristic 或 Lim-style retiming 写成外部论文方法的严格复现。

### 5.6 向下一章的过渡

Related Work 的最后一句应自然引出方法：既然缺口位于在线 path-progress 决策层，下一章就需要说明液体状态如何被参数化、传播并进入 MPCC OCP。

## 6. 第三章：S-MPCC Method

### 6.1 本章回答的问题

本章要完整回答：在每一个控制周期中，S-MPCC 收到什么、传播什么、优化什么、约束什么，以及最终执行什么。

方法章只定义核心 S-MPCC，不把可选部署模块混入核心创新。

### 6.2 3.1 Problem Definition and Architecture

本节定义：

- 输入：几何可行参考路径、当前底盘状态、上一接受控制、传播的液体模态状态；
- 输出：第一步优化得到的线速度和角速度命令；
- 假设：容器安装在底盘旋转中心；
- 范围：不包含全局路径、occupancy grid 和动态障碍预测；
- 控制循环索引：在线周期用 \(j\)，预测节点用 \(k\mid j\)。

本节应放 core 稿中当前标为 Fig. 1 的双栏闭环架构图，展示：

1. online inputs；
2. state and path construction；
3. slosh-aware MPCC；
4. first optimized action；
5. shared execution layer；
6. mobile base and liquid；
7. feedback update。

图中的箭头保持横平竖直。图注必须明确哪些模块是核心方法，哪些是三种方法共享的执行层。

同步到完整论文后，如果 Introduction 保留研究定位图，闭环架构图的最终编号会自动后移；正文引用应始终使用 LaTeX label，不手写固定图号。

### 6.3 3.2 Path Progress and Contouring Geometry

本节写：

- 参考路径 \(\mathbf r(s)\) 和弧长参数 \(s\)；
- 防止路径投影倒退的 guarded projection；
- 局部三次多项式路径拟合；
- 参考航向 \(\phi(s)\) 和曲率 \(\kappa(s)\)；
- contour error 与 lag error；
- 虚拟路径进度状态为何由优化器决定，而不是由时间固定。

这一节是标准 MPCC 几何基础。要引用对应 MPCC 文献，但不把这些标准公式写成本文创新。

### 6.4 3.3 Robot–Liquid Augmented Dynamics

#### 3.3.1 Alpha-state base model

定义：

\[
\mathbf x_r=[p_x,p_y,\theta,v,s,\omega]^\top,
\qquad
\mathbf u=[a,\alpha,v_s]^\top.
\]

解释 \(a=\dot v\)、\(\alpha=\dot\omega\)、\(v_s=\dot s\)，并给出连续运动学。说明将 \(\omega\) 放入状态的作用是保证旋转连续性和直接施加角加速度约束，但它不是独立创新点。

#### 3.3.2 Container-parameterized first sloshing mode

这一小节是方法复现的关键，应写清：

- 圆柱容器半径 \(R_c\)、液深 \(h\)、液体密度 \(\rho\)；
- \(J_1'(\xi_{11})=0\) 和 \(\xi_{11}=1.8412\)；
- 总液体质量、第一模态固有频率、模态质量和 \(c_h\) 的映射；
- 阻尼比 \(\zeta\) 从冻结配置读取，除非确实完成并归档辨识，否则不声称来自独立辨识；
- 第一横向模态沿两个正交方向表示，而不是两个不同阶次模态；
- \(\eta_x,\eta_y\) 是广义模态位移，不是直接物理液面高度；
- 车体前向和横向坐标定义；
- 中心安装时 \(a_x=a\)、\(a_y=v\omega\)。

同时明确：当前模型不支持偏心容器。偏心安装需要在 OCP 和在线传播中同时加入 \(\alpha r\) 与 \(\omega^2r\) 项。

#### 3.3.3 Online liquid-state propagation

本节写：

- trial 开始前满足静止判据后才把液体状态初始化为零；
- trial 内液体状态持续传播，不随控制周期清零；
- 使用 odometry 得到纵向和横向激励；
- 在线状态传播使用 ZOH 离散化；
- OCP 联合预测使用 ERK；
- RGB 不用于修正或反馈该模态状态；
- 正式模型量为
  \(H_{\mathrm{modal}}=c_h\sqrt{\eta_x^2+\eta_y^2}\)；
- 抛物面修正关闭，正文不再使用 \(H_{\mathrm{diag}}\)。

### 6.5 3.4 Slosh-Aware MPCC OCP

本节依次写：

1. 有限时域 OCP 与初始条件；
2. 离散增广动力学；
3. 状态和控制约束；
4. tracking、path progress、control、continuity 和 slosh 代价；
5. 曲率相关参考速度；
6. 模态位移和速度归一化；
7. terminal cost；
8. 三种实验方法如何由同一个 OCP 定义派生。

必须保留的液体归一化为：

\[
\eta_{\mathrm{ref}}=\frac{H_{\mathrm{ref}}}{c_h},
\qquad
\dot\eta_{\mathrm{ref}}
=\omega_1\eta_{\mathrm{ref}}
=\frac{\omega_1H_{\mathrm{ref}}}{c_h}.
\]

代价中不要再次乘 \(c_h\)，避免重复归一化。

正文给出代价结构和关键约束即可。全部权重、normalizer 和三种方法的数值差异放补充材料，但正式采集前必须冻结并填写。

### 6.6 3.5 Online Solution and Executed Command

本节写：

- acados SQP-RTI；
- 每个控制周期一次 RTI；
- partial-condensing HPIPM；
- ERK 设置；
- horizon、采样周期和控制频率；
- warm start 和失败回退；
- 只执行第一步优化控制；
- OCP 内部约束与 raw solver、post-gate、published command limits 的区别；
- shared execution layer 的干预标志与 \(r_{\mathrm{int}}\) 如何记录。

所有实物实验使用的 terminal handling、delay predictor、command gate、rate limiter 和 fallback 都必须作为共享部署层报告，并在三种方法中保持一致。

方法章结尾只需说明：

- Reference Governor 与 modal hard cap 是可选液体感知扩展，不属于当前核心 S-MPCC；
- delay predictor、terminal controller、command gate 和 rate limiter 属于共享部署层；
- 它们不能被写成本文核心贡献，也不能只对 S-MPCC 启用后再与其他方法比较。

### 6.7 方法章核心图表

正文建议保留：

- Fig. 1：完整闭环架构；
- 主要公式：projection、contour/lag、robot dynamics、liquid parameters、liquid dynamics、state propagation、\(H_{\mathrm{modal}}\)、OCP、stage/slosh cost、bounds 和 optimized first action。

三方法定义表放在实验章，不放在方法章。完整 solver 和权重表放补充材料。

### 6.8 本章不应写入

- RGB 提取算法的详细流程；
- 正式实验矩阵；
- 开发阶段三次结果；
- 将执行层功能包装成 S-MPCC 核心；
- 偏心容器有效性主张；
- 高保真液面或形式化防溢保证。

### 6.9 向下一章的过渡

方法章结束时，读者应明确 S-MPCC 与普通 MPCC 的区别在于传播液体状态和 slosh cost。实验章随即检验：这个显式液体机制是否真的改善物理液面，而不是只在内部模型上变小。

## 7. 第四章：Experimental Evaluation

### 7.1 本章回答的问题

本章按“四个核心 RQ + 一个支持性证据部分”组织，而不是分别按 RGB、轨迹、速度、模型和求解器拆章。每个 RQ 内同时使用物理液面、运动、激励、任务表现和计算指标回答一个明确问题。第五部分不是 RQ5，其主体是模型一致性、敏感性和执行链诊断；长时传播与命令规律性只作为可完全省略的探索性附加项。

### 7.2 4.1 Evaluation Questions and Evidence Structure

开头明确四个 RQ：

- **RQ1 — Physical effectiveness and task performance：** S-MPCC 是否比 Baseline 和 Smooth-only MPCC 降低真实液面响应，同时完成规定路径？
- **RQ2 — Completion-time confound：** 完成时间匹配后，S-MPCC 的收益是否仍然存在？
- **RQ3 — Container transfer：** 只根据容器几何重算模型参数而不重新调权时，收益能否跨容器保持？
- **RQ4 — State-dependent online planning and propagated-state effect：** 构造相位和正式运行中的传播状态是否改变未来规划与 optimized first action，并且求解是否满足实时期限？

实验证据在章内保持五部分结构：RQ1、RQ2、RQ3、RQ4，以及 `Supporting Robustness, Model Consistency, and Sensitivity`。后者只用于说明方法的实用边界和模型局限，不改变四个 RQ 的主次关系。

随后给出证据链：

\[
(\kappa,\hat{\mathbf x}_r,\hat{\mathbf x}_\ell)
\rightarrow
\mathbf u_{0:N-1}^{\star}
\rightarrow
\mathbf u_{\mathrm{solver}}
\rightarrow
\mathbf u_{\mathrm{post\mbox{-}gate}}
\rightarrow
\mathbf u_{\mathrm{exec}}
\rightarrow
(a_{x,\mathrm{exec}},a_{y,\mathrm{exec}})
\Rightarrow
\{H_{\mathrm{modal}},H_{\mathrm{vis}}\}.
\]

这里必须说明：

- \(H_{\mathrm{modal}}\) 和 \(H_{\mathrm{vis}}\) 是并行的模型与实验响应，不应画成前者导致后者；
- 原始 OCP 命令、gate 后命令和最终 published command 必须分层记录；
- 只有最终执行轨迹可以直接连接到真实激励和物理液面；
- RQ4 replay 若未经过共享执行层，只能称 optimized first-action difference。

### 7.3 4.2 Experimental Setup and Compared Methods

#### 4.2.1 Platform, paths, and containers

介绍：

- Scout Mini；
- 固定 RGB 相机和容器安装；
- 车体 \(x_b/y_b\) 轴；
- 容器处于旋转中心；
- 名义容器 \(C_1\)；
- 参数明显不同的容器 \(C_2\)；
- 低风险路径和高风险路径。

放一张综合实验设置图：机器人与相机、两条路径、两个容器和尺寸、坐标轴。完整路径点、相机型号、软件版本和标定记录放补充材料。

#### 4.2.2 Compared methods

正文在这里放三方法定义表：

| Method           | Frozen code mapping | Liquid state | Slosh cost | Smoothing | Role             |
| ---------------- | ------------------- | -----------: | ---------: | --------: | ---------------- |
| Baseline MPCC    | `B0`              |          Off |        Off |   Nominal | 基础 MPCC 对照   |
| Smooth-only MPCC | `B_smooth`        |          Off |        Off |  Enhanced | 通用增强平滑对照 |
| S-MPCC           | `B_slosh`         |           On |         On |   Nominal | 本文方法         |

解释三种比较：

- Baseline 与 S-MPCC：显式液体状态与 slosh cost 的整体机制比较；
- Baseline 与 Smooth-only：增强通用平滑的作用；
- Smooth-only 与 S-MPCC：普通平滑策略与液体状态感知规划的实际比较。

不存在核心 `B_ours`。已有 “S-MPCC + enhanced smoothing” 数据只能作为补充开发结果，不能进入核心主表，也不能替代 `B_slosh`。三种论文方法必须分别归档只读配置快照。

RQ2 的 Smooth-match 是独立 pilot 后只按完成时间冻结的 `B_smooth` 速度参考配置，用于排除“只是整体更慢”，不作为第四个核心方法。当前核心 S-MPCC 只包含传播液体状态和 slosh cost，不施加 modal hard constraint，因此 `S-MPCC-NoConstraint` 与当前完整方法不构成独立消融，`S-MPCC-NoCost` 则会使液体状态失去优化作用并基本退化为 Baseline。实时液体初态的作用由 RQ4 同一 pre-solve 快照下的 actual/zero/phase-flip replay 检验，不另设大规模 `NoState` 实物组。

当前论文不把 CA-MPCC 放入 baseline，也不把现有 Lim-style heuristic retiming 称为 Lim 原方法。Lim 2024 作为同物理任务、不同决策层的强近邻在 Related Work 中比较；只有未来完成其二维离线 OCP 的忠实复现和统一协议验证后，才考虑作为补充实验，且不改变当前三方法主矩阵。

#### 4.2.3 Experimental matrix

完整方案采用 \(n=8\) 个随机区组、共 88 次正式实物实验：

| 证据块           | 条件                    | 方法                 | 次数 | 用途                 |
| ---------------- | ----------------------- | -------------------- | ---: | -------------------- |
| 低风险区组       | \(C_1\)，低风险路径     | 三种核心方法         |   24 | RQ1 双路径一致性     |
| 容器 super-block | \(C_1/C_2\)，高风险路径 | 三种核心方法         |   48 | RQ1 高风险结果与 RQ3 |
| 等时间区组       | \(C_1\)，高风险路径     | Smooth-match、S-MPCC |   16 | RQ2                  |
| 合计             | —                      | —                   |   88 | RQ1–RQ4 与实时性    |

可选物理参数失配组：在 \(C_2\) 上错误使用 \(C_1\) 参数运行 S-MPCC，每个 super-block 增加一次，共增加 8 次。若只完成该扩展，总数为 96。只有该组的外部 \(H_{\mathrm{vis}}\) 支持时，才能讨论几何参数更新对物理迁移效果的必要性。

长时传播与命令规律性审计不再定义为正式实验组，不预留固定的 8 次样本，也不改变 40/64/88/96 次证据包计数。若主实验完成后另行开展，应使用独立的 exploratory protocol 记录路径、重复数、预验收和停止规则，并与正式区组数据分开归档。

> **本章的硬边界：** 88 次是唯一主实验矩阵；只有预先冻结的 \(C_2\) 物理参数 mismatch 扩展可形成 96 次。长时传播与命令规律性审计始终不计入正式总数，默认不占正文篇幅。若探索性采集已经完成，则无论结果有利或不利都应在开发记录中归档；只有数据链完整且确有独立解释价值时才进入补充材料。

> **版本硬边界：** 40、64 和 88 次三个证据包必须来自同一冻结 release；分阶段完成只表示证据覆盖范围逐步扩大，不表示允许在阶段之间修改方法。可以在 40 或 64 次处停止并同步缩小论文主张，但若修改动力学、代价、权重、路径或统计规则后继续采集，修改前后的数据不得合并为同一正式矩阵。

RQ4 纵向/横向四相位规划、实际传播状态 actual/zero replay，以及 RQ3 参数切换和计算 mismatch replay 均属于计算实验，不增加 88 次主方案的实物次数。

若实验资源不足，可按证据包缩减，但必须同步缩小论文主张：

| 证据包         | 正式实物组成（\(n=8\)）             | 次数 | 可以保留的结论                   |
| -------------- | ----------------------------------- | ---: | -------------------------------- |
| 核心机制包     | \(C_1\) 高风险三方法 + 等时间两方法 |   40 | RQ1 高风险、RQ2、RQ4、实时性     |
| 双路径包       | 核心机制包 +\(C_1\) 低风险三方法    |   64 | 增加两条路径的一致性             |
| 完整主方案     | 双路径包 +\(C_2\) 高风险三方法      |   88 | 完整 RQ1–RQ4 和跨容器 RQ3       |
| 参数必要性扩展 | 完整主方案 +\(C_2\) 错误参数 S-MPCC |   96 | 可检验参数更新对物理迁移是否必要 |

如果最终只完成 40 或 64 次，摘要、贡献、RQ 列表、结果表和结论中必须删除或降级尚未获得对应数据的跨路径或跨容器主张。

### 7.4 4.3 Measurements, Protocol, and Analysis

#### 4.3.1 Recorded signals and physical reference

按证据层记录：

- 路径：参考路径、\(s\)、\(\kappa(s)\)；
- 机器人：\((x,y,\theta,v,\omega)\) 和路径误差；
- OCP 输入：实际送入 solver 的机器人/液体状态、\(s_{\min}\)、有效 \(v_{\mathrm{ref}}\) 和必要的 pre-solve 状态；
- OCP 输出：第一控制量和预测 \(v,\omega,v_s,H_{\mathrm{modal}}\)；
- 执行：raw solver command、post-gate command、最终 published command 和 limiter/fallback 标志；
- 激励：\(\dot v\) 和 \(v\omega\)；
- 液体模型：\((\eta_x,\dot\eta_x,\eta_y,\dot\eta_y)\) 与 \(H_{\mathrm{modal}}\)；
- 物理液面：\(H_{\mathrm{vis}}\)；
- 求解器：solve time、status、deadline miss、fallback 和 intervention。

必须区分预测控制与真实执行。运动和激励指标使用 odometry 与 executed command；预测量只用于机制解释。RQ4 计算实验必须由冻结的 replay 工具导出完整 \(v,\omega,v_s,\eta,\dot\eta,a,\alpha\) horizon，不能只依赖 XY path、前三个预测点或 horizon 摘要。

执行层干预必须预先定义命令差异容差 \(\epsilon_v,\epsilon_\omega\)、干预比例 \(r_{\mathrm{int}}\) 和采集前准入阈值 \(r_{\mathrm{int,max}}\)。若共享执行层在大量周期内覆盖 OCP 差异，正文只能把物理结果解释为最终执行轨迹的效果，不能直接归因于未执行的 raw OCP command。

RGB 的论文定位统一为：

> \(H_{\mathrm{vis}}\) is treated as the calibrated vision-based experimental reference measurement, while \(H_{\mathrm{modal}}\) remains an internal model response.

正文至少说明相机、ROI、像素—毫米标定、统一提取设置、缺失帧处理、时间同步和标定误差。不能称其为无条件的 absolute ground truth，也不能用内部模型证明 RGB 准确。

#### 4.3.2 Primary, secondary, and diagnostic outcomes

主要结果固定为首次有效运动到统一到达时刻的全运动窗口 trial-level RGB p95：

\[
\mathcal W_{\mathrm{full}}
=[t_{\mathrm{move}},t_{\mathrm{arrival}}],
\qquad
H_{\mathrm{vis,p95}}^{\mathrm{full}}
=Q_{0.95}\!\left(
H_{\mathrm{vis}}(t):t\in\mathcal W_{\mathrm{full}}
\right).
\]

全运动窗口包括 \(Z_1\) 起步和 \(Z_5\) 制动，但不包含开始前静止等待和到达后观察。10%–90% 路径进度 p95 作为关键敏感性指标，用于隔离 start/terminal handling；\(Z_1\)–\(Z_5\) 作为预注册机制区段。

次要和权衡指标包括：

- 10%–90% 路径进度 RGB p95；
- full-motion RGB RMS；
- post-arrival RGB RMS；
- completion time；
- path-error p95；
- success rate。

机制指标包括：

- \(H_{\mathrm{modal}}\) p95；
- \(a_x\) RMS；
- \(|a_y|\) p95；
- \(\alpha\) 或命令变化指标；
- 执行层干预比例 \(r_{\mathrm{int}}\) 和 raw-to-published command 差异。

实时性指标包括：

- solve-time median、p95、maximum；
- deadline-miss rate；
- solver failure 和 fallback；
- 实际控制频率。

一次完整 trial 才是统计样本，视频帧和过程曲线采样点不是独立样本。

#### 4.3.3 Randomization, paired analysis, and failure handling

写清：

- 每个 block 内包含全部待比较方法；
- 方法顺序随机，super-block 内容器顺序随机或平衡；
- RQ1 第一主比较预注册为 S-MPCC − Smooth-only，S-MPCC − Baseline 为关键次比较，Smooth-only − Baseline 为机制诊断；
- RQ2 主比较预注册为 S-MPCC − Smooth-match；
- 主要估计量为 block 内 paired difference；
- 报告原始 trial 点、配对线、平均配对差、相对变化和以 block 为单位 bootstrap 的 95% CI；
- 第一主比较同时报告 exact paired sign-flip/randomization inference 和 leave-one-block-out 敏感性；
- \(n=8\) 只是采集目标，不自动等于统计确认；
- solver failure、timeout、tracking failure 或 safety termination 计为方法失败；
- 只有与方法无关的采集故障可以排除并在同一区组补采；
- 不能按结果是否符合预期选择 trial 或代表曲线。

### 7.5 4.4 RQ1: Physical Liquid Reduction and Task Performance

#### 4.4.1 Physical liquid response

比较 \(C_1\) 上两条路径的三种方法。主要对比为：

- S-MPCC − Smooth-only：第一主比较；
- S-MPCC − Baseline：关键次比较；
- Smooth-only − Baseline：机制诊断比较。

正文主结果表采用 `Method × Path` 六行，报告：

- RGB p95；
- post-arrival RMS；
- completion time；
- path p95；
- success。

表中的方法汇总只是描述性结果，正式解释依赖 block-paired effects 和区间。

#### 4.4.2 Motion and excitation redistribution

用高风险路径的过程图回答“为什么发生变化”。所有曲线以归一化路径进度 \(\sigma=s/L\) 对齐，依次展示：

1. path curvature；
2. executed \(v\)；
3. executed \(\omega\)；
4. realized \(a_x,a_y\)；
5. \(H_{\mathrm{modal}}\)；
6. \(H_{\mathrm{vis}}\)。

重点讨论 S-MPCC 是否在高曲率或高风险区段重新分配激励，而不是只比较整段平均速度。

#### 4.4.3 Mechanism and task tradeoffs

讨论：

- Smooth-only 是否确实降低通用运动变化；
- S-MPCC 是否降低液面但显著损害路径跟踪或成功率；
- S-MPCC 的运动变化是否与液体状态和关键路径区段一致；
- 不能仅凭 \(H_{\mathrm{modal}}\) 下降宣布物理效果成立。

详细机制表可移至补充材料，但正文必须保留关键过程图和任务权衡讨论。

### 7.6 4.5 RQ2: Completion-Time-Matched Comparison

RQ2 只比较：

\[
\text{Smooth-match MPCC}
\quad\text{vs.}\quad
\text{S-MPCC}.
\]

Smooth-match 的参考速度必须用独立 pilot 数据按完成时间调节，只看时间、不看正式 RGB，随后冻结再采集正式结果。

本节主要用图而不是大表，展示：

- 完成时间—RGB p95 配对散点；
- \(v(\sigma)\)；
- \(a_y(\sigma)\)；
- \(H_{\mathrm{modal}}(\sigma)\)；
- \(H_{\mathrm{vis}}(\sigma)\)。

本节要证明的不是两者每一时刻速度完全相同，而是总完成时间接近时，S-MPCC 仍能依据路径和液体状态改变分段运动分配。

### 7.7 4.6 RQ3: Transfer Across Containers

#### 4.6.1 Parameter-transfer protocol

跨容器时只更新：

- \(\omega_{1,c}\)；
- \(c_{h,c}\)；
- \(\eta_{\mathrm{ref},c}\)；
- \(\dot\eta_{\mathrm{ref},c}\)。

保持不变：

- MPCC 权重；
- \(\zeta\)；
- horizon 和 solver；
- 路径、参考速度和运动约束；
- 液体类型；
- 共享部署层。

两个容器采用统一的 freeboard fraction \(\lambda_H\)，不要再使用容易与液体密度 \(\rho\) 混淆的 \(\rho_f\)。结果同时报告毫米值和 freeboard-normalized 指标。

\(C_2\) 冻结前还要检查相机采样能力。当前 \(C_1\) 第一模态约为 \(4.97\,\mathrm{Hz}\)，30 Hz 相机每周期约 6 帧；除非提高相机帧率并重新验证视觉链，否则 \(C_2\) 不宜通过显著提高 \(f_1\) 来制造参数差异。

#### 4.6.2 Physical and process results

正文 RQ3 表采用 `Container × Method` 六行，报告：

- RGB p95；
- normalized RGB p95；
- \(H_{\mathrm{modal}}\) p95；
- completion time；
- path p95。

主图包括：

- method × container interaction plot；
- S-MPCC 在 \(C_1/C_2\) 上的 \(v(\sigma)\)；
- \(a_y(\sigma)\)；
- \(H_{\mathrm{modal}}\) 与 \(H_{\mathrm{vis}}\)。

另外做一个控制计算比较：固定机器人状态、路径位置和归一化模态状态，只切换 \(C_1/C_2\) 参数集，观察规划变化。物理试验说明重参数化后的实际效果；计算比较说明参数变化如何影响规划。两者不能混为同一个因果结论。

计算或 log-replay 中还可比较：在 \(C_2\) 日志上分别使用正确 \(C_2\) 参数和错误 \(C_1\) 参数。该计算 mismatch 只能证明规划对参数敏感，不能证明正确参数带来更好的真实液面。完整 88 次方案默认只支持“无权重重调的有限跨容器迁移”；只有额外 8 次物理 mismatch 组的 \(H_{\mathrm{vis}}\) 支持时，才能讨论参数更新对物理迁移的必要作用。

### 7.8 4.7 RQ4: State-Dependent Planning, Propagated-State Effect, and Real-Time Feasibility

#### 4.7.1 Liquid-phase-dependent replanning

至少冻结两个机制检查点：

- \(Z_1\) 起步或 \(Z_4\) 重新加速前的纵向检查点；
- \(Z_2\) 入弯或 \(Z_3\) 曲率反转前的横向检查点。

固定相同机器人状态、路径、容器参数和模态能量，纵向 \(x\) 方向使用四个等能量不同相位的液体初始状态：

\[
[A,0,0,0]^\top,
\quad[0,\omega_1A,0,0]^\top,
\quad[-A,0,0,0]^\top,
\quad[0,-\omega_1A,0,0]^\top,
\]

横向 \(y\) 方向相应使用：

\[
[0,0,A,0]^\top,
\quad[0,0,0,\omega_1A]^\top,
\quad[0,0,-A,0]^\top,
\quad[0,0,0,-\omega_1A]^\top.
\]

其中 \(A=0.5\eta_{\mathrm{ref}}\) 是预先冻结的机制研究参数，不应描述成当前代码自动事实。正文可展示与主要高风险动作最相关的一组，另一组放补充材料；不能在查看结果后选择更有利的方向或检查点。

展示不同相位下的：

- predicted \(v_{k\mid j}\)；
- predicted \(\omega_{k\mid j}\)；
- \(v_{s,k\mid j}\)；
- \(H_{\mathrm{modal},k\mid j}\)；
- optimized first action；
- 可选 predicted path。

如果相同机器人状态和路径在不同液体相位下产生不同未来计划，就直接支撑“控制器原则上对液体相位敏感”。

#### 4.7.2 Counterfactual replay of the propagated runtime state

四相位实验仍然使用人为构造状态，因此还需要使用正式高风险路径日志做 actual/zero 反事实 replay：

\[
\mathcal P_{\mathrm{actual},j}
=\mathcal P(
\hat{\mathbf x}_{r,j},
\hat{\mathbf x}_{\ell,j},
\mathrm{path}_j,
\mathcal S_j),
\qquad
\mathcal P_{\mathrm{zero},j}
=\mathcal P(
\hat{\mathbf x}_{r,j},
\mathbf 0,
\mathrm{path}_j,
\mathcal S_j).
\]

其中 \(\mathcal S_j\) 表示两个分支共享的 pre-solve solver、warm-start、previous solution、上一控制、\(s_{\min}\)、有效 \(v_{\mathrm{ref}}\) 和必要的部署上下文。

回放必须遵守：

- actual/zero 从同一个 pre-solve 快照克隆，不能在同一个可变 solver 实例上顺序求解；
- actual 分支先在冻结数值容差内复现在线 solver status、第一控制量和 raw solver command；
- replay 工具导出完整预测 horizon；
- trial 是统计样本，控制周期只用于形成 trial-level 中位数、p95 和超过数值容差的比例；
- 未经过 terminal、gate 和 rate limiter 回放时，只称 optimized first-action difference，不称 counterfactual executed command。

该实验回答“正式运行中由 odometry 传播出来的内部状态是否实际改变优化动作”。它不证明内部传播状态等于真实液体状态，也不能替代 \(H_{\mathrm{vis}}\) 的物理结果。

#### 4.7.3 Real-time performance

使用全部正式实物实验统计：

- solve-time median、p95、maximum；
- solve-time ECDF；
- deadline-miss rate；
- solver failure；
- fallback；
- achieved control frequency。

这一节必须用实际运行日志证明 online feasibility，不能仅凭采用 SQP-RTI 或 acados 就声称实时。

### 7.9 4.8 Supporting Robustness, Model Consistency, and Sensitivity

这是实验章的第五个证据部分，但不是第五个 RQ，也不能替代 RGB 主结果。本节以常规模型一致性和敏感性诊断为主体；长时传播只在资源允许时作为低优先级探索性附加项。

#### 4.8.1 Model consistency and sensitivity

该模块做常规支持性诊断：

- \(H_{\mathrm{modal}}\) 与 \(H_{\mathrm{vis}}\) 的代表性对齐时序；
- Ferrari-form signed bias：保持 \(H_{\mathrm{modal}}\) 积分作分母；
- 单独命名的 absolute disagreement、RMSE、raw correlation 和局部低估量；
- \(\omega_1,\zeta,c_h\)、初始状态和执行延迟敏感性；
- actual/zero/phase-flip replay 的完整分布和 reproduction/failure rate；
- \(C_2\) 正确/错误参数的计算 mismatch；
- 可选的 8 次物理 mismatch 组。

K6 主要总体固定为 E1–E3 的 32 次正式 S-MPCC 尝试，主要窗口固定为 \([t_{\mathrm{move}},t_{\mathrm{arrival}}+5\,\mathrm{s}]\)。禁止 per-trial 最佳时滞、幅值拟合、模型 topic 回退和依据正式结果重调参数。详细敏感性曲线和数值表放补充材料，正文只保留能够帮助解释模型适用范围的简洁结果；全部执行细节以 K6-FID-v1.0 为准。

#### 4.8.2 Exploratory long-horizon propagation and command-regularity audit

该项目默认不进入正文实验结构，也不设正式样本数。只有主实验、计算实验和 4.8.1 的常规诊断完成后仍有资源，并且扩展路径安全、连续状态传播、视觉同步、失败记录以及 raw/post-gate/published command 日志全部通过独立预验收时，才另立 exploratory protocol 执行。

可检查多个连续高风险曲率序列之间的 modal–vision disagreement、\(H_{\mathrm{vis}}\)、轨迹误差、求解失败、首末序列差异、total-variation rate、方向反转率、高频能量和执行层干预。主矩阵中 raw/post/published command 与 \(r_{\mathrm{int}}\) 的最小执行链审计仍是必做项，不能等待该探索性项目补足。

该项目不构成闭环稳定性、递归可行性、长期无误差累积或命令非劣性的证明。即使结果支持，也只能描述为特定测试时长与条件下的探索性观察。完成后优先放入补充材料；未执行时不保留 pending，占用的试次也不加入正式实验总数。

### 7.10 实验章正文图表规划

正文建议控制为：

#### 三张主表

1. 三种方法定义；
2. RQ1 两路径结果；
3. RQ3 跨容器结果。

#### 五组核心图

1. 实验装置、路径和容器；
2. RQ1 路径—运动—激励—液面过程链；
3. RQ2 等完成时间比较；
4. RQ3 容器 interaction 与过程曲线；
5. RQ4 纵横向四相位规划、actual/zero optimized first-action difference 与 runtime ECDF。

长时传播/命令规律性审计不进入上述五组核心图，默认只在补充材料中保存探索性全分布。只有其结果对解释主实验中的明确异常具有不可替代作用且篇幅允许时，正文才增加一句说明或一张紧凑表。

表格应出现在首次讨论相应 RQ 的附近，不能把 RQ3 表统一堆到实验章末尾。

### 7.11 本章不应写入

- 把旧三次 pilot 混入正式 \(n=8\)；
- 把 frame 当成独立样本；
- 根据结果好坏排除 trial；
- 用模型量替代 RGB 主结果；
- 把方法失败归类为采集故障后重跑；
- 用没有同期区组的历史数据直接比较 \(C_1/C_2\)；
- 把同一可变 solver 上顺序执行的 actual/zero 求解当成公平反事实；
- 把计算 mismatch 写成参数更新对真实物理迁移的必要性证据；
- 把 optimized first-action difference 写成机器人已经执行的反事实命令；
- 把长时审计包装成 RQ5、稳定性证明或三项贡献的必需证据；
- 把长时探索性试次加入 40/64/88/96 次正式总数，或在未执行时保留 pending 小节和无退化/防抖主张；
- 在没有统一复现协议的情况下加入外部 planner 排名。

## 8. 第五章：Conclusion and Limitations

### 8.1 本章回答的问题

结论只回答：四个 RQ 最终得到了什么证据支持，以及这些结论可以推广到哪里、不能推广到哪里。

### 8.2 推荐内容顺序

#### 第一段：方法总结

用一段话回顾 S-MPCC：沿给定路径，将 acceleration-level MPCC、虚拟进度和两个正交方向上的第一液体模态联合预测，通过 odometry 保存液体动态记忆并滚动执行第一步控制。

#### 第二段：正式结果总结

数据完成后按 RQ1–RQ4 顺序写入：

- 相对 Baseline 和 Smooth-only 的物理液面变化；
- 等完成时间条件下的结果；
- 跨容器重参数化结果；
- 相位相关规划、实际传播状态 replay 和实时求解结果。

只写最重要的效应量和区间，不重复整张结果表。
长时传播与命令规律性审计原则上不进入结论。只有其探索性结果对解释主实验中的异常或适用边界具有不可替代作用时，才可增加一句明确标注 exploratory 的受限观察。

#### 第三段：局限性

明确：

- 给定路径而非完整避障导航；
- 容器安装在旋转中心；
- 低阶模型而非高保真自由液面；
- 模态状态没有 RGB 校正；
- 无形式化 spill-free guarantee；
- 当前主比较只有匹配内部 MPCC 变体。

#### 第四段：后续工作

后续方向可以包括：

- 偏心容器的刚体激励模型；
- 液面或其他传感器状态校正；
- collision-aware S-MPCC；
- 无液体状态传感器的低复杂度 CA-MPCC，例如基于 \(\sqrt{\dot v^2+(v\omega)^2}\) 的合加速度约束；
- 忠实复现的液体感知近邻 baseline；
- 更丰富容器、液体和负载条件。

这些内容只能写成 future work，不能作为当前能力。

### 8.3 本章不应写入

- 新公式、新实验或首次出现的贡献；
- 超过正式结果支持范围的泛化结论；
- “全面优于现有方法”；
- “保证不溢出”；
- 将未完成实验写成已经验证的事实。

## 9. Supplementary Material 的内容边界

补充材料服务于复现、审计和正文减负，不承担核心结论。当前应包含：

1. solver、horizon、integration、warm start 和 fallback 的完整设置；
2. Baseline、Smooth-only、S-MPCC 的全部权重和 normalizer；
3. 实验装置、路径、容器、相机、同步和软件提交的 freeze checklist；
4. required signals 与 derived metrics checklist；
5. 40/64/88 次主证据包，以及唯一计入正式总数的可选 +8 参数必要性扩展；只有该扩展实际完成时列出 96 次总数；
6. 旧三次 development-only 数据；
7. 旧 “S-MPCC + enhanced smoothing” 的补充记录；
8. 完整 sensitivity 和 runtime 表；
9. actual/zero/phase-flip replay 的快照、复现容差、完整 horizon 和 trial-level 分布；
10. \(C_2\) 正确/错误参数的计算 mismatch 与可选物理 mismatch；
11. raw/post-gate/published command 和执行层干预统计；
12. 正文放不下的低风险过程曲线和机制诊断。
13. 仅当探索性长时审计完成时，另设 development/exploratory 小节收录扩展路径、预验收、序列级传播结果和分层命令规律性全分布，并明确其不计入正式总数。

当前 `spmpc_paper_core/main.tex` 不会自动编译 `supplementary/supplementary_material.tex`。投稿阶段可根据期刊要求把它作为独立补充 PDF，或由完整论文入口显式 `\input`/`\include`；在未显式引用前，它不会出现在 core PDF 中。

补充材料不能用来隐藏以下关键内容：

- 方法的核心状态、动力学和 OCP；
- 三种方法的定义；
- 主要结果指标；
- RQ1 和 RQ3 的核心结果；
- 关键失败率与实时性结论。

## 10. 章节—证据映射

| 核心判断                                                     | 主要出现位置                    | 必要证据                                                                   |
| ------------------------------------------------------------ | ------------------------------- | -------------------------------------------------------------------------- |
| 液体具有动态记忆，普通平滑不能显式表示相位                   | Introduction、Related Work、RQ4 | 文献定位、纵横向四相位规划与 actual/zero replay                            |
| S-MPCC 把液体状态放入在线 MPCC                               | Method                          | 增广状态、传播方程、slosh cost、optimized first action                     |
| 物理液面得到改善                                             | RQ1                             | 正式 block-paired\(H_{\mathrm{vis}}\) 结果                                 |
| 改善不只是通用平滑                                           | RQ1                             | Baseline、Smooth-only、S-MPCC 三方法比较                                   |
| 改善不只是更慢                                               | RQ2                             | 独立 pilot 冻结后的等完成时间比较                                          |
| 容器参数化可实现有限迁移                                     | RQ3                             | \(C_1/C_2\) super-block、参数切换规划与计算 mismatch                       |
| 正式传播状态实际改变优化动作                                 | RQ4                             | 同一 pre-solve 快照下 actual/zero optimized first-action difference        |
| 方法是真正的状态相关在线规划                                 | RQ4                             | 纵横向不同液体相位的完整预测计划与 optimized first action                  |
| 方法可以实时运行                                             | RQ4                             | solve-time ECDF、p95、miss 和 failure                                      |
| OCP 差异是否真正到达执行层                                   | Measurements、RQ1、RQ4          | raw/post-gate/published command 与\(r_{\mathrm{int}}\)                     |
| 特定长序列中是否出现值得进一步调查的传播或命令异常（探索性） | Supplementary diagnostics       | 非核心、非必需；仅在另立 exploratory protocol 后使用序列级全分布作边界观察 |
| 内部模型具有何种一致性和局限                                 | Supporting Analysis             | modal–vision 对齐和敏感性，不替代 RGB                                     |

## 11. 统一术语和符号

后续所有章节应统一：

| 项目               | 统一写法                                                            | 不再使用或谨慎使用                                  |
| ------------------ | ------------------------------------------------------------------- | --------------------------------------------------- |
| 论文方法名         | S-MPCC                                                              | SPMPC 作为论文名                                    |
| 内部 OCP           | slosh-aware MPCC formulation                                        | 泛称完整自主 planner                                |
| 物理参考液面       | calibrated vision-based experimental reference,\(H_{\mathrm{vis}}\) | absolute ground truth                               |
| 内部模型响应       | \(H_{\mathrm{modal}}\)                                              | \(H_{\mathrm{diag}}\)、物理真值                     |
| 液体模态           | first lateral mode represented in two orthogonal directions         | two-mode model                                      |
| 核心方法组         | S-MPCC                                                              | \(B_{\mathrm{ours}}\)                               |
| 增强平滑组         | Smooth-only MPCC                                                    | ours-smooth                                         |
| 等时间组           | Smooth-match MPCC                                                   | 与核心三方法混为第四组                              |
| 代码映射           | Baseline=`B0`、Smooth-only=`B_smooth`、S-MPCC=`B_slosh`       | 用`B_ours` 代表论文方法                           |
| RQ4 反事实量       | optimized first-action difference                                   | 未经过执行层却称 counterfactual executed command    |
| 长时审计           | exploratory supplementary diagnostic                                | RQ5、stability proof、正式扩展计数、必做主实验      |
| freeboard fraction | \(\lambda_H\)                                                       | \(\rho_f\)                                          |
| 路径范围           | prescribed geometrically feasible path                              | obstacle-free guarantee / safe under all conditions |

## 12. 推荐写作与执行顺序

当前阶段不宜先反复润色摘要和引言。推荐顺序为：

1. **冻结 Method：** 核心定义、符号、公式、执行层边界和方法名全部一致。
2. **冻结实验协议：** 三方法映射、权重、\(\zeta\)、全运动窗口 primary、10%–90% 敏感性窗口、统计主比较、执行限制、RGB 处理和失败规则。
3. **完成实验工具准备：** 冻结两条路径、\(C_2\)、相机标定和随机区组表；验证 raw/post-gate/published command 记录；验收 replay 快照克隆、actual 复现和完整 horizon 导出。
4. **采集正式数据：** 使用唯一 freeze ID 按矩阵速查表优先完成 40 次核心机制包，再扩展到 64/88 次；这一分阶段安排只服务于资源管理和证据完整度判断，E1–E3 全程不得根据中间结果改动力学、代价、权重、路径、三方法映射或统计规则。若任何一项发生变化，应结束当前版本、隔离已有数据并从新冻结版本重新建立正式证据包。88 次主矩阵完成后，只有需要参数物理必要性主张时才按预冻结协议增加 8 次物理 mismatch。
5. **完成计算实验：** 执行纵横向四相位、actual/zero replay、容器参数切换和计算 mismatch。
6. **填写 RQ1–RQ4：** 先生成原始配对点、过程曲线、效应量、区间、sign-flip 和 leave-one-block-out，再写结果解释。
7. **反向修改 Introduction 与 Related Work：** 让贡献和主张与实际证据完全一致，并保持 Lim、CA-MPCC 和外部 baseline 的事实边界。
8. **低优先级决定长时诊断：** 只有前述工作完成后仍有资源，且决定依据是路径安全、日志完整性和诊断需求而非主结果好坏时，才另立 exploratory protocol；其试次不改变正式总数。
9. **最后写 Abstract 与 Conclusion：** 只使用正式结果中的最关键数字；探索性长时结果原则上不进入摘要和结论。
10. **同步完整论文：** `spmpc_paper_core` 确定后，再将方法和实验同步到 `spmpc_paper`。

## 13. 正式采集前的论文冻结检查

在开始正式实验前，论文与配置至少应共同确认：

- [ ] 全文方法名统一为 S-MPCC；
- [ ] 方法映射固定为 Baseline=`B0`、Smooth-only=`B_smooth`、S-MPCC=`B_slosh`，不存在核心 `B_ours`；
- [ ] CA-MPCC、Lim-inspired retiming、`NoConstraint/NoCost` 不进入核心方法表，Lim 2024 不被误写成一维临界加速度方法；
- [ ] 三方法的只读配置快照和唯一数值差异已归档；
- [ ] E1–E3 已绑定同一 freeze ID，包含软件 revision、动力学与代价版本、权重、两条路径、\(C_1/C_2\) 参数、执行限制、随机表和分析规则；
- [ ] 已确认 40/64 次仅为证据节点，阶段之间不调参；若产生新方法版本，旧数据与新数据分开归档且不拼接为同一 88 次矩阵；
- [ ] rotation-consistent dynamics、相位能量/有符号功率等后续改动明确排除在本轮冻结版本之外；
- [ ] \(\zeta\) 的实际值与来源已填写；
- [ ] \(H_{\mathrm{modal}}\) 为 modal-only，抛物面项关闭；
- [ ] \(H_{\mathrm{vis}}\) 的标定、同步和缺失帧规则已冻结；
- [ ] 全运动窗口 primary、10%–90% 敏感性窗口、\(Z_1\)–\(Z_5\) 和 5 s post-arrival window 已冻结；
- [ ] 首次有效运动阈值、持续时间、2 s rest criterion 和统一到达判据已冻结；
- [ ] Smooth-match 只用独立 pilot 的完成时间调节；
- [ ] \(C_2\)、统一 \(\lambda_H\) 和 \(f_{\mathrm{cam}}/f_1\) 测量准入已确定；
- [ ] raw solver、post-gate、published command 和 executed-command limits 已分别记录；
- [ ] \(\epsilon_v,\epsilon_\omega,r_{\mathrm{int,max}}\) 已冻结；
- [ ] terminal、gate、delay、rate 和 fallback 在三方法中一致；
- [ ] RQ1/RQ2 主比较、block bootstrap、exact sign-flip 和 leave-one-block-out 已冻结；
- [ ] 正式随机区组顺序已生成；
- [ ] 失败 trial 和采集故障的处理规则已写入实验协议；
- [ ] 纵向/横向四相位的幅值和检查点已冻结；
- [ ] replay 工具能从同一 pre-solve 快照克隆 actual/zero 分支并导出完整 horizon；
- [ ] actual replay 能在冻结容差内复现在线第一控制量和 raw solver command；
- [ ] 计算 mismatch 不会被写成物理必要性证据；
- [ ] K6-FID-v1.0 的 no-go 检查全部通过，唯一 K6 脚本、同步标定、视觉配置与 freeze manifest 已归档；
- [ ] 若另行执行探索性长时审计，已建立与正式区组分离的 protocol、目录和停止规则，且不会改变 40/64/88/96 次计数；
- [ ] 正式软件 revision、文档、配置和分析脚本已归档；
- [ ] 旧三次结果明确标记为 pilot，不计入正式 \(n\)。

## 14. 当前版本的最终叙事

整篇论文最终应让读者得到以下清晰认识：

> S-MPCC 的价值不在于单纯让机器人运动更平滑，也不在于提出新的高保真液体模型。它把一个可实时传播的低阶液体动态状态放入移动底盘的 path-progress MPCC 决策层，使当前液体位移、速度和相位能够改变下一段在线运动计划。论文通过三种匹配内部方法、独立等完成时间比较、跨容器无权重重调迁移、纵横向四相位规划、正式传播状态 actual/zero replay、执行层干预诊断和实时求解统计，逐层验证这一有限但明确的主张；真实物理效果始终由独立的 \(H_{\mathrm{vis}}\) 结果承担。

上述最终叙事不依赖长时传播与命令规律性审计。该项目若完成，默认只作为补充材料中的探索性诊断，用于记录特定测试时长和条件下的异常或边界；若不做，论文仍以 88 次主矩阵和 RQ1–RQ4 完整收敛。
