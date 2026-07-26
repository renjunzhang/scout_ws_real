# S-MPCC 当前论文组织思路

> **文档状态（2026-07-26）：论文实验主线与 0717 v2.0 候选现场协议已对齐，执行依赖待实现。** 本文档确定论文级问题、对照、证据层级和拟议矩阵；它不授权开始正式采集。
>
> **权威边界：** 0717 矩阵与启动命令已升为候选 `SMPCC-REAL-40-64-88-v2.0`，并与本文的五条件核心 40、双路径 64、条件性跨容器 88 对齐；正式采集仍为 `NO-GO`。仓库仍只有旧 manifest 模板，没有可执行的 `freeze_manifest.yaml/FREEZE_ID`；实验总章、分析协议、K6、随机表、validator、FixedProfile 实物链和 trajectory/replay 工具尚未全部同步或验收。只有这些依赖升级并生成唯一只读 release 后，v2.0 才是可执行的正式协议。
>
> **后续同步顺序：** 在已对齐的论文核心与 0717 文档基础上，继续升级实验总章、分析/K6 协议、freeze template、validator 和随机表；同时实现并验收 fixed-profile comparator、trajectory extraction、online-input/zero-state replay、视觉链和方法 release gate；最后生成唯一只读 `FREEZE_ID`。在此之前不得把候选计数或方法映射写成已经冻结的事实。
>
> **K6 边界：** K6-FID-v1.0 当前仍是旧矩阵下的详细分析协议，其 32 个 S-MPCC 单元总体与本提案不兼容。若采用新矩阵，必须在正式采集前重算 K6 strata、分母和 no-go 条件。视觉标定、同步、质量控制和 RGB 提取链仍是正式采集的必要准入；完整 modal–vision fidelity 分析只作为支持性模型诊断，不替代物理效果，也不验证真实带符号液体相位。四相位与 actual/zero replay 只检验优化器对模型传播内部状态的敏感性。
>
> **方法 release 边界：** 当前动力学与 rotation-consistent 候选必须先经过 rotation-relevance replay/量级 gate；独立 RGB development pilot 必须验证候选方法相对 Smooth-only 的物理方向。通过这些门槛后只选择一个方法 release 进入正式矩阵。不同 release 的数据不得拼接。
>
> **探索性诊断说明：** `Long-horizon propagation and command-regularity audit` 保持第二层探索性诊断，不属于 40/64/88 次正式证据包，也不形成 96/104 次扩展计数。默认投稿正文不设置该模块的 pending 小节。
>
> **对照边界：** 拟议实验加入一个同平台的 model-informed fixed-profile comparator，用于检验液体感知固定 timing 与模型传播状态相关 online progress 的差异。它是本文按先验工作原理构造的透明 comparator；除非完成逐项忠实复现，否则不得称为 Hamaguchi 或 Lim 原方法的复现，也不得据此声称全面优于原论文。
>
> `B_ours`、四格因子实验、Reference Governor 核心消融、modal hard cap、`NoConstraint/NoCost` 核心消融、终端独有贡献、宽泛 DWA/TEB 排名和结果驱动追加实验不属于本提案主线。

## 1. 论文的核心定位

### 1.1 一句话定义

S-MPCC 是一个沿预先给定几何可行路径运行的、由 odometry 驱动的模型传播内部液体模态状态条件化的在线轨迹生成与控制方法。几何路径 \(\mathbf r(s)\) 由上游给定；S-MPCC 在同一 MPCC 预测时域内联合优化有限时域虚拟路径进度、底盘运动和传播的液体动态，并执行第一步优化控制。由此产生的真实运动 timing 必须再从 odometry 独立测量，不能把优化器内部进度直接当作实际时间律。

建议在英文正文中统一写为：

> S-MPCC is a receding-horizon trajectory generator and controller that uses an odometry-driven, model-propagated internal liquid modal state to jointly optimize finite-horizon virtual path progress, chassis motion, and liquid dynamics along a prescribed geometrically feasible path.

论文名称使用 `S-MPCC`，强调其本质是 slosh-aware model predictive contouring control。代码、ROS 包和工程目录仍可沿用 `spmpc`，但不据此将论文方法改名为 SPMPC。

### 1.2 核心科学问题

普通平滑或整体减速只能约束运动变化，不能显式表示液体的模态位移、模态速度和振荡相位。物理上，相同机器人状态和相似平滑程度的候选运动，在不同液体相位下可能导致不同的未来液面响应；当前控制器并不直接测量该真实相位，而是传播一个低阶内部状态估计。

本文要回答的核心问题是：

> 在给定几何可行路径的条件下，能否把 odometry 驱动的模型传播液体动态记忆放入在线 MPCC 决策层，使移动底盘根据该内部模态状态局部重分配虚拟路径进度、速度和纵横向激励，并在相近任务效率下比普通平滑、整体减速和液体感知固定 timing 更有效地降低实际测得的液面响应？

### 1.3 总论证主线

全文应围绕以下逻辑展开：

1. 开放液体运输不是单纯的轨迹平滑问题，而是具有动态记忆的状态预测与有限时域虚拟路径进度优化问题。
2. 现有通用规划/控制器通常不传播液体状态；已有防晃工作已经覆盖 input shaping、液体感知固定速度剖面、离线轨迹优化和在线 tracking/MPC。
3. 因此本文不能只问“是否优于普通 MPCC”，还必须问“模型传播的在线状态记忆是否比名义模型驱动的固定 timing 提供额外价值”。
4. S-MPCC 将机器人、虚拟路径进度和低阶内部液体模态状态放入同一滚动时域 OCP，联合生成有限时域 progress 与底盘运动；它由此诱导实际运动 timing，但不直接定义一条保证严格执行的全局 \(s(t)\)。
5. Baseline、Smooth-only、Smooth-match 与 model-informed fixed-profile 对普通控制、通用平滑、整体减速和液体感知固定 timing 四类替代解释形成分层检验；其中 fixed-profile 只提供端到端系统比较，不单独识别动态记忆的物理因果效应。
6. 纵向/横向四相位规划与正式日志的 online-input/zero-state（简称 actual/zero）replay 检验模型传播状态是否真正改变预测 timing、完整 horizon 和第一动作；`actual` 仅指在线 solver 实际收到的内部状态。
7. 高风险路径承担核心物理效果与机制验证；低风险路径检验路径特征选择性和不必要保守性。
8. 跨容器只作为条件性扩展：只有有限配置迁移仍被预先保留为结果目标且前述机制成立时，才进入正式正文主张；标准几何—模态参数映射本身不包装成创新。
9. 全部正式日志的实时性与执行层干预统计说明在线计划差异能否按控制周期落地到真实执行轨迹。

实验章节的证据顺序应保持为：

\[
\text{真实液面是否改善}
\rightarrow
\text{轨迹 timing 与激励如何改变}
\rightarrow
\text{是否只是平滑、减速或固定液体感知 timing}
\rightarrow
\text{传播状态是否真正改变优化}
\rightarrow
\text{是否实时且被实际执行}
\rightarrow
\text{能否跨路径及条件性跨容器}.
\]

长时传播与命令规律性审计只保留为主证据链完成后的探索性诊断，优先进入补充材料，不能成为摘要、贡献列表、主结论或实验完整性的前置条件。

### 1.4 当前允许的主要主张

正式实验完成后，方法层可以固定主张：

> S-MPCC uses an odometry-driven, model-propagated internal liquid modal state to adapt finite-horizon virtual path progress and chassis motion along prescribed paths.

性能主张不能写成默认同时优于全部对照的 umbrella sentence，而应按 gate 逐项解锁：

- 只有 RQ1 第一主比较通过，才追加 `improved the measured liquid response relative to Smooth-only MPCC under the reported conditions`；
- 只有 Smooth-match 完成时间门与物理比较通过，才追加 `the result was not explained by a completion-time-matched slowdown alone`；
- 只有 Fixed-profile novelty gate 也通过，才追加 `outperformed the reported model-informed fixed-profile comparator`；
- Baseline、路径选择性和有限容器迁移各自只在对应预冻结准则通过后单独陈述，不把未通过或未执行的 contrast 藏在 `relative to the reported comparisons` 中。

这个主张只在正式数据支持的条件下成立，并且必须限定于：

- 预先给定的几何可行路径；
- 正式完成并报告的实验路径；
- 容器安装在底盘旋转中心；
- 当前低阶液体模型和冻结的执行配置；
- 三种内部 MPCC variants、Smooth-match 和本文实现的 model-informed fixed-profile comparator；
- 若执行条件性 \(C_2\) 扩展，只能主张所报告容器和冻结参数下的有限迁移。

### 1.5 不应扩大的主张

正文不得把当前方法描述为：

- 全局路径规划器或完整自主导航系统；
- 已包含 occupancy grid、动态障碍预测或碰撞走廊的避障 planner；
- 首个使用液体状态的 MPC；
- 首个移动机器人防晃预测控制方法；
- 自由几何 path planning、time-optimal trajectory planning 或任意场景 replanning；
- 高保真自由液面模拟器；
- 使用 RGB 液面反馈的闭环控制器；
- 具有严格无溢出、稳定性或递归可行性保证的方法；
- 将 Lim 2024 描述为一维临界加速度梯形速度曲线，或声称其提供真实液体防溢保证；
- 忠实复现 Hamaguchi 或 Lim，除非后续逐项实现和审计确实满足其原始方法定义；
- 已经优于 Hamaguchi、Lim、DWA、TEB 或其他未按统一协议正式比较的方法；本文只能比较自己透明实现并冻结的 fixed-profile comparator。

## 2. 全文结构总览

完整论文采用五个正文部分，实验结果和解释按研究问题合并，不另设一个内容重复的通用 Discussion：

| 位置     | 章节                       | 本章核心作用                               |
| -------- | -------------------------- | ------------------------------------------ |
| 前置部分 | Title、Abstract、Keywords  | 用最短篇幅交代问题、方法、证据和结论       |
| I        | Introduction               | 建立问题、缺口、方法定位、贡献和适用边界   |
| II       | Related Work               | 从物理任务和决策层两条轴线定位 S-MPCC      |
| III      | S-MPCC Method              | 给出可复现的机器人—液体增广预测和在线 OCP |
| IV       | Experimental Evaluation    | 按 RQ1–RQ4 建立物理效果、轨迹 timing、机制、路径选择性与有限迁移证据链 |
| V        | Conclusion and Limitations | 汇总得到支持的结论，并明确不能外推的范围   |
| 复现材料 | Repository / permitted supplement | 保存完整配置、正式矩阵、开发数据和详细诊断 |

章节之间的逻辑关系为：

> Introduction 提出动态记忆与在线虚拟路径进度问题；Related Work 说明固定液体感知 timing 与模型状态相关 online progress 之间仍有待验证的决策层差异；Method 说明如何把内部液体状态记忆放入 MPCC；Experiments 检验物理收益、轨迹重分配、替代解释、状态相关性、实时性、路径特征选择性和有限配置迁移；Conclusion 只总结已经被正式证据支持的部分。

## 3. Title、Abstract 与 Keywords

### 3.1 标题

重构后优先考虑的标题：

> **S-MPCC: Slosh-Aware Online Trajectory Generation for Prescribed-Path Liquid Transport**

若希望更突出算法名称，也可使用：

> **S-MPCC: Slosh-Aware Model Predictive Contouring Control for Prescribed-Path Open-Liquid Transport**

在没有真实液体状态传感/估计验证前，不优先使用未经限定的 `Liquid-State-Conditioned` 标题；若后续确需使用，应明确写成 `Model-Propagated Slosh-State-Conditioned`，并接受标题更长的代价。

标题应突出：

- 方法是 S-MPCC，而不是泛称 SPMPC；
- 核心技术是 model-propagated-state-conditioned path-progress MPCC；
- 任务是移动底盘开放液体运输；
- 明确几何路径预先给定，不在标题中加入 obstacle avoidance、spill-free 或 autonomous navigation。

### 3.2 摘要写什么

摘要最终采用“背景—缺口—方法—实验—结果—边界”的紧凑结构：

1. **背景：** 移动底盘的加速、制动和转向会激发开放液体晃动。
2. **缺口：** 通用平滑不表示液体动态记忆，而液体感知固定 profile 或离线轨迹又不能根据运行中模型传播的残余状态在线调整下一段 motion timing。
3. **方法：** S-MPCC 联合预测机器人、虚拟路径进度和容器参数化的一阶液体模态，使内部传播状态条件化有限时域 progress 与底盘激励，并由 odometry 独立重建真实执行 timing。
4. **实验：** 五条件高风险核心比较、等完成时间与 model-informed fixed-profile 对照、双路径验证、纵横向相位规划、actual/zero replay、执行链与实时性；跨容器只在条件性扩展完成后写入。
5. **结果：** 正式数据完成后填入 RGB p95、残余振荡、任务时间和求解时间的关键定量结果。
6. **边界：** 沿给定几何可行路径进行在线局部运动生成，不声称完整避障导航或严格防溢保证。

摘要中暂时不能写入 `pending` 数值，更不能用旧三次 pilot 数据代替正式结果。

### 3.3 Keywords

建议保留：

- mobile robots；
- open-liquid transport；
- liquid sloshing；
- model predictive contouring control；
- prescribed-path trajectory generation；
- online virtual path-progress optimization。

## 4. 第一章：Introduction

### 4.1 本章回答的问题

本章要让读者理解：为什么移动机器人开放液体运输值得研究，为什么普通平滑和液体感知固定 timing 都不能直接回答运行中残余状态问题，以及本文究竟解决现有工作的哪一层问题。

### 4.2 推荐内容顺序

#### 第一段：任务背景与实际困难

介绍实验室、服务和工业场景中的开放液体运输。指出底盘加速、制动和转弯会激发自由液面振荡，进而增加稳定时间、降低运输可靠性并提高溢出风险。

这一段只建立问题重要性，不展开方法公式。

#### 第二段：核心科学困难

明确提出“液体动态记忆”：

- 液体响应不仅取决于当前速度、加速度或 jerk；
- 还取决于当前模态位移、模态速度和残余振荡相位；
- 因此相似平滑度的两段运动可能产生不同后续液面响应。

这一段是全文最重要的问题定义，应直接为增广状态和 RQ3 铺垫。

#### 第三段：现有研究的两条线及其缺口

简要概括：

- 防晃研究已经包括低阶建模、input shaping、液体感知固定速度剖面、轨迹优化和在线跟踪/MPC；
- 移动机器人局部规划已经包括 DWA、TEB、MPC 和 MPCC 等在线运动生成方法；
- 关键未决问题不是“是否有人做过防晃”，而是名义模型生成的固定 timing 与利用运行中模型传播残余状态的 online progress 在同一平台和任务下有何差异。

这里不要展开完整文献比较，详细分析留给 Related Work。

#### 第四段：本文方法概述

给出 S-MPCC 的一句话定义，并说明三个关键动作：

1. 用 odometry 激励在线传播内部液体模态状态估计；
2. 在 MPCC horizon 内联合传播机器人、虚拟路径进度和内部液体状态，在线优化有限时域 progress 与底盘运动，并以 odometry 独立测量实际 motion timing；
3. 只执行第一步优化控制，然后在下一周期重新求解。

#### 第五段：贡献列表

贡献数量不能在结果前固定为三项，也不要用 `C2` 同时指 Contribution 2 和容器 \(C_2\)。先保留三个候选贡献元素：

- **Element A — Model-propagated-state-conditioned path-progress MPCC：** 将一阶横向液体模态在两个正交方向上的内部传播状态加入 acceleration-level MPCC，使该状态条件化有限时域虚拟路径进度、底盘运动和纵横向激励分配。
- **Element B — Odometry-driven internal modal-state memory and configuration：** 采用已有的圆柱容器几何—第一模态映射来配置低阶模型，并把 odometry 激励下持续传播的内部模态状态接入每次在线求解；创新不归于标准几何—模态公式本身。
- **Element C — Matched physical validation against competing timing explanations：** 使用三种内部 MPCC variants、Smooth-match、model-informed fixed-profile、已完成路径上的视觉物理测量、构造内部四相位、actual/zero replay 和执行链统计，检验普通平滑、整体减速、液体感知 fixed timing 与模型传播状态相关 online progress 等竞争解释。

投稿版本按最终证据范围选择模板：

| 正式证据范围 | 最终贡献模板 |
| --- | --- |
| 仅 40 | 两项：A+B 合并为方法贡献；C 只写高风险路径 matched validation，不写双路径或迁移 |
| 完成 64 | 两项：A+B 合并为方法贡献；C 增加所报告 H1/L1 上的路径特征选择性 |
| 完成条件性 88 | 可保留三项：A 为 OCP formulation；B 只把有限物理配置迁移验证作为结果性增量；C 为完整 matched validation |

正式数据完成前，Element C 仍只能暂写为 matched experimental evidence structure；投稿版本必须改为由正式结果支持的 matched physical validation，不应把“设计了一套实验”本身包装成新的控制理论贡献。若 88 未完成，Element B 必须并入 A，不能留下独立“容器参数化创新”。

#### 第六段：范围声明

明确说明：

- 输入路径已经几何可行；
- 当前 OCP 不负责全局路径和障碍推理；
- RGB 是实验参考测量，不进入控制闭环；
- fixed-profile 是本文实现并审计的 prior-art-inspired comparator，不冒充原论文复现；
- 比较和主张限于正式冻结的方法、路径、容器与执行条件；
- 不提供严格 spill-free guarantee。

### 4.3 本章图表

Introduction 可保留一张紧凑的“研究定位图”，表达：

\[
\text{prescribed geometry}
+
\text{online path progress}
+
\text{model-propagated liquid-state memory}
\rightarrow
\text{finite-horizon progress and executed motion timing}.
\]

该图只解释研究交叉位置，不能与 Method 中的完整闭环架构图重复。

### 4.4 本章不应写入

- 完整 OCP 公式；
- 所有容器参数公式；
- solver 细节；
- 40/64/条件性 88 矩阵的细节；
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

#### 2.1 Prescribed-Path Trajectory Generation and Online MPCC for Mobile Robots

介绍 path/trajectory/time-parameterization 的区别，并讨论 MPC、MPCC 及相关实时运动生成方法；DWA/TEB 只作为移动机器人部署背景。重点说明：

- 它们与本文处于相近的在线运动决策层；
- 能处理路径进度、跟踪、运动约束、平滑和实时求解；
- 但通常不传播容器内液体的内部模态位移和速度估计，也不根据非刚性负载状态调整有限时域虚拟路径进度与底盘运动。

本节用于说明 MPCC backbone 的来源以及“普通平滑不等于液体动态预测”。当前实验不对这些 obstacle-oriented planners 做排行榜比较。

#### 2.2 Mobile-Base Liquid Transport and Anti-Slosh Methods

介绍与移动底盘液体运输最接近的研究，包括：

- Hamaguchi 系列的路径、速度剖面、input shaping 和跟踪；
- Lim 等的液体约束离线轨迹优化；
- 规划加跟踪、特殊平台或主动防晃机构；
- 其他移动平台上的 slosh-aware MPC 或 tracking control。

比较重点应落在决策层：Hamaguchi-style input shaping 和其他 model-informed profile 先生成固定 timing，再由跟踪器执行；S-MPCC 每个控制周期根据模型传播的内部状态联合选择虚拟路径进度和底盘运动。这个差异不能只在 Related Work 中口头声明，必须由同平台 fixed-profile comparator 正式检验。

可以保留一张“决策层比较表”，字段增加“是否使用液体模型”“是否保留运行中液体状态”“fixed/online timing”。表中必须把原论文与本文实现的 comparator 分开，不能把 inspired baseline 当成原论文的严格复现。

Lim 2024 的事实边界必须写准确：该工作使用球摆动力学进行二维移动机器人整段轨迹优化，通过配点法离线生成线速度、角速度和位姿轨迹；其主要局限是整段预先求解、目标时间与终点预先给定、运行时不根据当前液体状态滚动重规划。不能把它改写成“一维临界加速度梯形速度曲线”。若把

\[
\dot v^2+(v\omega)^2\le a_R^2
\]

嵌入在线 MPCC，这应称为本文团队另行设计的 Lim-inspired CA-MPCC 或低复杂度合加速度约束方法，而不是 Lim 原方法的复现。拟议 `Model-informed fixed-profile` 优先采用透明的固定路径 input-shaped/offline timing，实现目标是比较 fixed physics-aware timing 与 model-propagated-state-conditioned online progress；它同样不能称为 Lim 原方法复现。

#### 2.3 Cross-Platform Slosh Modeling, Trajectory Generation, and Predictive Control

介绍机械臂、SCARA、罐体、船舶或其他平台上的：

- CFD、VOF、FEM 等高保真模型；
- equivalent pendulum、MSD 和低阶模态模型；
- input shaping、轨迹生成和预测控制；
- 视觉液面作为外部实验依据的做法。

本节要说明低阶模型适合在线优化，但不是本文原创；Ferrari 和 Leva 等工作可支持建模与评价方法的合理性，但不能自动证明本论文 RGB 测量的准确性。

#### 2.4 Positioning of S-MPCC

用一段综合定位结束本章：

- 普通在线 planner 与本文决策层相近，但缺少模型传播的液体内部状态；
- 移动底盘防晃研究与本文物理任务相近，且已经证明 fixed profile、input shaping 和在线 tracking/MPC 有效；
- 跨平台防晃 MPC 提供重要基础，因此本文不声称首次使用 slosh state in MPC；
- 本文待验证的增量是：在标准轮式底盘的 prescribed-path MPCC 中保留 odometry 驱动的低阶内部液体动态记忆，并检验 S-MPCC 相对同平台 model-informed fixed timing 是否产生非平凡的局部轨迹重分配和物理收益。

### 5.4 本章图表

建议最多保留一张紧凑比较表，字段可包括：

- platform/task；
- liquid model；
- generation layer；
- fixed or online timing；
- runtime liquid-state memory；
- online replanning；
- output interface；
- 与本文的主要差异。

表格不是 baseline 结果表，也不能暗示本文已经在实验上优于这些工作。

### 5.5 本章不应写入

- 未核实的“首次”声明；
- 把所有文献都批评成 offline 或不实时；
- 把特殊平台工作说成与本文无关；
- 用 Ferrari 的结果证明本论文 RGB 为绝对真值；
- 预先宣布本文性能优于 Hamaguchi/Lim 原方法；正式结果只覆盖本文实现的 comparator。
- 把自行设计的 fixed-profile、CA-MPCC、临界加速度 heuristic 或 Lim-style retiming 写成外部论文方法的严格复现。

### 5.6 向下一章的过渡

Related Work 的最后一句应自然引出方法：既然缺口位于在线 path-progress 决策层，下一章就需要说明液体状态如何被参数化、传播并进入 MPCC OCP。

## 6. 第三章：S-MPCC Method

### 6.1 本章回答的问题

本章要完整回答：在每一个控制周期中，S-MPCC 收到什么、传播什么、如何联合优化给定路径上的有限时域虚拟进度与底盘运动、约束什么，以及最终执行什么；实际 motion timing 由实验章从 odometry 独立重建。

方法章只定义核心 S-MPCC，不把可选部署模块混入核心创新。

### 6.2 3.1 Problem Definition and Architecture

本节定义：

- 输入：几何可行参考路径、当前底盘状态、上一接受控制、odometry 驱动的模型传播内部液体模态状态；
- 输出：预测时域内由该内部状态条件化的虚拟路径进度/底盘轨迹，以及第一步优化得到的线速度和角速度命令；
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

图中的箭头保持横平竖直。图注必须明确哪些模块是核心方法，哪些是全部正式物理 comparison conditions 共享的执行层。

同步到完整论文后，如果 Introduction 保留研究定位图，闭环架构图的最终编号会自动后移；正文引用应始终使用 LaTeX label，不手写固定图号。

### 6.3 3.2 Path Progress and Contouring Geometry

本节写：

- 参考路径 \(\mathbf r(s)\) 和弧长参数 \(s\)；
- 固定几何与可变运动的关系 \(\mathbf p(t)\approx\mathbf r(s_{\mathrm{ocp}}(t))\)：本文不选择路线几何，核心决策是有限时域虚拟 progress 和 contouring motion；contour/lag 代价不保证机器人严格执行一条全局 time law；
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

#### 3.3.3 Online internal liquid-state propagation

本节写：

- trial 开始前满足静止判据后才把内部液体模态状态初始化为零；
- trial 内该模型状态持续传播，不随控制周期清零；
- 使用 odometry 得到纵向和横向激励；
- 在线状态传播使用 ZOH 离散化；
- OCP 联合预测使用 ERK；
- RGB 不用于修正或反馈该模态状态，因此它不是经视觉验证的真实带符号相位估计；
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
8. Baseline、Smooth-only 和 S-MPCC 三个内部 variants 如何由同一个 OCP 定义派生；Smooth-match 与 fixed-profile 的完整定义留在实验章，不能伪装成相同核心 OCP 的开关消融。

必须保留的液体归一化为：

\[
\eta_{\mathrm{ref}}=\frac{H_{\mathrm{ref}}}{c_h},
\qquad
\dot\eta_{\mathrm{ref}}
=\omega_1\eta_{\mathrm{ref}}
=\frac{\omega_1H_{\mathrm{ref}}}{c_h}.
\]

代价中不要再次乘 \(c_h\)，避免重复归一化。

正文给出代价结构和关键约束即可。全部权重、normalizer、三个内部 variants 的数值差异，以及 fixed-profile 的生成算法、参数、离线 profile 和 hash 放补充材料，但正式采集前必须冻结并填写。

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

所有实物实验使用的 terminal handling、delay predictor、command gate、rate limiter 和 fallback 都必须作为共享部署层报告，并在五个核心 comparison conditions 中保持一致。

方法章结尾只需说明：

- Reference Governor 与 modal hard cap 是可选液体感知扩展，不属于当前核心 S-MPCC；
- delay predictor、terminal controller、command gate 和 rate limiter 属于共享部署层；
- 它们不能被写成本文核心贡献，也不能只对 S-MPCC 启用后再与其他方法比较。

### 6.7 方法章核心图表

正文建议保留：

- Fig. 1：完整闭环架构；
- 主要公式：projection、contour/lag、robot dynamics、liquid parameters、liquid dynamics、state propagation、\(H_{\mathrm{modal}}\)、OCP、stage/slosh cost、bounds 和 optimized first action。

五个核心 comparison conditions 的定义表放在实验章，不放在方法章。完整 solver、内部权重和 fixed-profile 生成设置放补充材料。

### 6.8 本章不应写入

- RGB 提取算法的详细流程；
- 正式实验矩阵；
- 开发阶段三次结果；
- 将执行层功能包装成 S-MPCC 核心；
- 偏心容器有效性主张；
- 高保真液面或形式化防溢保证。

### 6.9 向下一章的过渡

方法章结束时，读者应明确 S-MPCC 与普通 MPCC 的区别在于传播内部液体模态状态并使用 slosh cost，并且其主要规划自由度是给定几何上的 finite-horizon virtual progress 与底盘运动。实验章随即检验：这个机制是否改善物理液面、是否诱导可在 odometry 中观察到的局部 motion-timing 重分配，以及是否比普通平滑、整体减速和液体感知 fixed timing 提供额外价值。

## 7. 第四章：Experimental Evaluation

### 7.1 本章回答的问题

本章不再以“做了多少容器、多少诊断”为组织中心，而是围绕一个轨迹生成问题建立证据：在相同给定几何路径上，模型传播的内部液体模态状态是否改变有限时域 progress 与底盘运动，是否由此诱导可在 odometry 中独立观察的非平凡局部 timing/激励重分配，并在相近任务效率下改善真实液面响应。

实验章必须同时回答两类不同问题：

1. **内部机制与替代解释问题：** S-MPCC 是否优于普通 MPCC、通用平滑和整体减速；模型传播状态是否改变 OCP 决策，以及正式 S-MPCC 的在线计划差异是否保留到实际执行；
2. **相对系统问题：** S-MPCC 端到端系统是否比已知液体参数驱动的 fixed-profile timing 提供额外物理与任务价值。

轨迹变化是机制证据，\(H_{\mathrm{vis}}\) 是物理效果证据；两者都不能替代对方。实验设计严谨不等于方法已经有效，所有结果性主张都以正式数据为条件。

### 7.2 4.1 Evaluation Questions and Evidence Structure

开头明确四个 RQ，并按证据顺序排列：

- **RQ1 — Physical effectiveness and trajectory redistribution：** 在高风险给定路径上，S-MPCC 是否相对 Baseline 和 Smooth-only 降低真实液面响应、保持任务性能，并产生路径区段相关的 timing 与纵横向激励重分配？
- **RQ2 — Competing timing explanations：** 等完成时间后收益是否仍存在；相对使用相同容器参数但不保留运行中内部模态状态的 model-informed fixed-profile，S-MPCC 端到端系统是否产生额外物理收益和不同的实际局部 motion timing？该比较不单独识别 memory 因果效应；memory 对优化决策的特异作用由 RQ3 检验。
- **RQ3 — Model-state-dependent online generation and execution：** 构造内部模态相位与正式运行中传播给 solver 的模型状态是否改变完整预测计划和 optimized first action；正式 S-MPCC 的在线计划差异是否通过执行链反映在实际运动中，且求解是否满足在线运行要求？
- **RQ4 — Path-feature selectivity and limited configuration transfer：** 冻结方法后，效果能否在所报告的低风险路径上保持合理选择性；若容器配置扩展仍被预先保留，能否在冻结的第二容器上满足有限物理—任务迁移准则？

`Supporting Model and Measurement Diagnostics` 只说明模型与测量边界，不设置 RQ5，不承担方法有效性结论。

随后给出证据链：

\[
(\kappa,\hat{\mathbf x}_r,\hat{\mathbf x}_\ell)
\rightarrow
\{s_{0:N}^{\star},\mathbf u_{0:N-1}^{\star}\}
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
- fixed-profile 的参考 timing、S-MPCC 的 optimized timing 与实际执行 timing 必须分层保存；
- 原始 OCP 命令、gate 后命令和最终 published command 必须分层记录；
- 只有最终执行轨迹可以直接连接到真实激励和物理液面；
- RQ3 replay 若未经过共享执行层，只能称 optimized first-action difference；
- 区段 timing、激励与 RGB 同方向变化只构成机制一致性证据，不宣称完成严格 mediation causal analysis。

### 7.3 4.2 Experimental Setup and Compared Methods

#### 4.2.1 Platform, paths, and containers

介绍：

- Scout Mini；
- 固定 RGB 相机和容器安装；
- 车体 \(x_b/y_b\) 轴；
- 容器处于旋转中心；
- 名义容器 \(C_1\)；
- 高风险路径 \(H_1\) 与低风险路径 \(L_1\)；
- 只有在条件性跨容器包被预先保留时，才介绍参数明显不同且在首条正式 trial 前冻结的 \(C_2\)。

放一张综合实验设置图：机器人与相机、两条路径、名义容器和坐标轴；\(C_2\) 只有在对应正式扩展完成后才加入正文。完整路径点、相机型号、软件版本和标定记录放仓库复现材料。

H1 必须包含不间断的起步、持续曲率、曲率反转、出弯再加速和终端制动，使残余状态有机会跨区段影响 timing；区段之间不得人为重置液体状态。L1 保留相同任务接口但采用低曲率/低激励几何，用作检验“方法是否不必要保守”的低风险选择性条件，而不把它包装成严格的统计 negative control。两条路径的角色、长度、曲率统计和 Z1–Z5 边界在任何正式结果前冻结。

Stage I 每个 block 连续执行五个 trial，Stage II 每个 block 连续执行三个 trial；每次 trial 都必须重新满足方法无关的入场门：起点位置/航向、零命令、由独立 physical ring-down 预验证的最小静置时长、可用时的连续 method-independent monitor 阈值、适用方法状态与公共 monitor reset 成功，以及最大等待时间。若没有可用的在线物理液面门，运动前门只使用冻结的 \(T_{\mathrm{settle}}\)；运动前至少 2 s raw RGB 必须由 timestamp 留证，但不能单独证明液体已经静稳。对 condition label 盲化的离线 visual-start QC 在采集后判定视觉资格，失败时保留原 bag 并按方法无关 acquisition failure 处理。超时、无法静稳、跨时段恢复和 split-block 的配对资格必须预先定义，避免上一方法的残余液体状态污染下一方法。

#### 4.2.2 Compared methods

正文放五个核心 comparison conditions 的定义表：

| Condition | 拟议映射 | 使用液体参数 | 运行中内部液体状态 | Timing 形式 | 证据角色 |
| --- | --- | ---: | ---: | --- | --- |
| Baseline MPCC | `B0` | 否 | 否 | online MPCC, nominal | 普通 MPCC 锚点 |
| Smooth-only MPCC | `B_smooth` | 否 | 否 | online MPCC, enhanced smoothing | 检验通用平滑解释 |
| Smooth-match MPCC | `B_smooth` + frozen matched `v_ref` | 否 | 否 | online MPCC, completion-time matched | 检验整体减速解释 |
| Model-informed fixed-profile | offline profile + frozen tracker（无 S-MPCC variant） | 是 | 否 | precomputed fixed \(s(t)/v(s)\) | 检验 physics-aware fixed timing |
| S-MPCC | `B_slosh` | 是 | 是（内部模型传播） | model-propagated-state-conditioned online progress | 本文方法 |

其中 Baseline、Smooth-only 和 S-MPCC 是同一 MPCC formulation 的三个内部 variants；Smooth-match 是专门的 completion-time comparator；fixed-profile 是独立生成固定 timing、再通过共享跟踪/执行层落地的 prior-art-inspired comparator。五者不是完整因子设计，不声称分别识别状态、slosh cost、平滑及其全部交互作用。

fixed-profile 的最低定义要求为：

- 使用相同冻结几何路径、容器 \(\omega_1/\zeta\) 和运动硬约束；
- 只选择一种冻结算法。默认候选是基于同一 \(\omega_1/\zeta\) 的 Hamaguchi-inspired 两脉冲 ZV fixed-path timing；ZVD 或独立离线 retiming 只能作为互斥替代方案，不能在同一个 `FixedProfile` condition ID 下事后择优；
- ZV 脉冲幅值和由 \(\omega_1,\zeta\) 决定的脉冲间隔 \(\Delta T\) 固定不动。若采用 ±5% 等完成时间，唯一调节旋钮必须是预先指定的未整形 base profile 参数（推荐 nominal cruise-speed/plateau parameter）；每个候选值都从 base profile 重新生成并经过同一 shaper、积分和硬约束检查，禁止对已经生成的 shaped profile 直接 time-warp；
- 显式考虑纵向激励以及固定路径上的 \(a_y\approx v^2\kappa=v\omega\)；
- profile 从与正式静稳门一致的零/近零名义液体初态生成，并明确该假设；
- 执行中不保留或更新液体模态状态估计，不根据运行中的残余模型相位重新规划；
- 允许共同 tracker 修正几何误差，但不得依据液体状态在线修改 timing；tracker 的进度/时间纠偏逻辑必须冻结并报告；
- 使用与其他方法相同的底盘、命令限制、terminal/gate/rate/fallback 和测量链；
- 报告离线生成时间、实际完成时间、tracking 和 published command；
- 名称固定为 `Model-informed fixed-profile` 或 `Hamaguchi-inspired fixed-path input shaping`，除非完成忠实复现审计，否则不得使用 `Hamaguchi reproduction` 或 `Lim reproduction`。

Fixed-profile 与 S-MPCC 的正式物理比较是两个完整 planning/tracking systems 的端到端比较，不单独识别“有无动态记忆”一个因素。即使使用共同 tracker，二者的 timing generation 和误差修正结构仍可能不同；必须报告 tracker error、延迟和命令修正。动态记忆对 S-MPCC 自身优化的特异作用由 RQ3 phase/actual-zero 支持，而不是由外部 comparator 单独识别。

不存在核心 `B_ours`。当前 S-MPCC 只包含内部传播液体状态和 slosh cost，不施加 modal hard constraint；在线 solver 实际收到的模型状态对优化的作用由 RQ3 actual/zero/phase replay 检验，不另设大规模 `NoState` 实物组，也不据此声称已经识别真实液体状态的物理因果效应。

#### 4.2.3 Experimental matrix

本提案继续以 \(n=8\) 作为资源平衡下的拟议最小 block 数，但它不是统计充分性的先验保证。正式 \(n\) 必须由预先声明的最小有意义 RGB 差 \(\delta_H\)、development paired SD、目标 CI 宽度或 power、预期方法失败率和视觉无效率共同决定并归档。若 \(n=8\) 不能提供所需精度，应在首条正式 trial 前增加 \(n\) 并重新命名全部证据包，不能看到正式结果后追加有利样本。

拟议分阶段矩阵为：

| 证据包 | 条件 | 每个 block 的方法 | Block 数 | 新增 | 累计 | 主要作用 |
| --- | --- | --- | ---: | ---: | ---: | --- |
| Stage I | \(C_1+H_1\) | B0、Smooth-only、Smooth-match、Fixed-profile、S-MPCC | 8 | 40 | 40 | RQ1–RQ3 核心证据 |
| Stage II-A | \(C_1+L_1\) | Smooth-only、Fixed-profile、S-MPCC | 8 | 24 | 64 | RQ4 路径特征选择性 |
| Stage II-B（条件性） | \(C_2+H_1\) | Smooth-only、Fixed-profile、S-MPCC | 8 | 24 | 88 | RQ4 有限跨容器迁移 |

Stage I 每个 randomized complete block 含五个独立 trial，每种 condition 恰好一次。它用 Fixed-profile 取代旧协议 E3 中第二次重复采集的 S-MPCC，使正式总数仍为 40，同时增加真正回答 novelty 替代解释的比较。所有四个与 S-MPCC 相关的 contrast 共享同一个 block-level S-MPCC observation；这种相关性由整体 block 重采样和预注册比较层级处理，不把多个 contrast 当成独立实验。

Stage II-A 不重复低风险 B0 和 Smooth-match，因为它的目的不是重新证明基础方法排序，而是检验 S-MPCC 相对两个竞争性 timing 策略在简单路径上是否产生不必要保守性。Stage II-B 同理只保留最有解释力的三条件比较；Fixed-profile 必须按相同冻结规则使用 \(C_2\) 参数重生成 profile，S-MPCC 则只按预声明规则更新物理模型参数。

由于 Stage II 不含 Smooth-match，Fixed-profile 的唯一 base-profile timing 参数必须分别对 \((C_1,L_1)\) 和拟议 \((C_2,H_1)\) 在独立 development pilot 中预冻结；ZV 脉冲参数不随完成时间目标缩放，正式样本无论实际匹配是否漂移都保留。RQ4 主要解释为所报告配置上的 liquid-response–efficiency–tracking 联合结果，不把 Smooth-only 的 RGB 差异单独写成排除减速后的纯方法效应，也不外推成宽泛泛化。若希望在 Stage II 继续作严格等时间因果比较，必须把 Smooth-match 加回并重算矩阵，而不能沿用 64/88 计数。

Stage II 的“选择性”和“有限迁移”不能由不显著结果推出。首条正式 trial 前必须同时冻结：L1 上 completion time、tracking、success 和 intervention 的等价/非劣界，以及 H1→L1 timing-intervention effect-difference 的判定规则；若保留 C2，还必须冻结容器内 RGB 最小效应、任务代价与 success 容限、主要 comparator 和迁移通过规则。精度或界值不足时，Stage II 只能作为描述性支持。

条件性 88 不能由结果好坏临时决定。若在确认本方案时保留有限跨容器迁移这一结果目标，必须在首条正式 trial 前冻结 \(C_2\)、触发/停止规则、方法配置、视觉准入、分析口径和随机表。允许在核心 gate 失败或资源不足时停止而不执行 C2；不允许因为 40/64 结果“漂亮”才事后选择容器、参数或新增主张。

纵向/横向四相位、actual/zero replay、rotation relevance 和参数切换均为计算/development gate，不增加正式物理计数。长时传播、错误容器参数物理组和其他扩展不属于本提案 40/64/88；若以后执行，必须另立协议并与正式数据分开。

正式证据包的解释固定为：

| 最终完成范围 | 可以保留的论文结论 |
| --- | --- |
| 40 | 高风险路径物理效果、竞争 timing 对照、模型状态相关规划、执行链与实时性 |
| 64 | 增加不同路径风险/曲率特征下的选择性和任务代价 |
| 条件性 88 | 在前述结论之外增加冻结参数下的有限跨容器迁移 |

如果最终只完成 40 或 64，摘要、贡献、RQ 列表、结果表和结论必须同步删除未获得证据的双路径或跨容器主张。

Stage I 之后必须先完成核心 claim gate，再决定扩展：

- S-MPCC 相对 Smooth-only 的 block-paired \(H_{\mathrm{vis}}\) 达到预冻结 \(\delta_H\)、区间和随机化准则，且不由单个 block 独占；
- Smooth-match 完成时间满足冻结误差，S-MPCC 的物理收益仍存在；
- 在固定 gatekeeping 下，相对 Fixed-profile 的结果达到预声明准则；若未达到，必须删除“端到端性能优于固定液体感知 timing”的表述；即使通过，该比较与 RQ3 也不能替代真实 `NoState` 物理消融，因而不主张在线动态记忆具有物理因果必要性；
- timing 差异集中在预注册区段，不能主要由全局常数降速解释；
- raw timing 差异经过执行层并反映在 odometry；
- phase/actual-zero、replay reproduction、runtime、tracking、failure 与 fallback 全部达到冻结门槛。

核心方向失败时停止并诊断，不能用 L1、C2 或增加样本掩盖。是否达到传统显著性阈值不是唯一判据，但效应方向、幅度、区间、leave-one-block-out 和失败模式必须共同支持主张。

### 7.4 Formal 前的 Development Gates

本节是采集前决策规则，不要求在 RA-L 正文逐项展开。新方案不得从“实现能跑”直接跳到正式 40；以下门槛必须在升级后的正式协议中逐项定义并通过：

1. **G0 — Claim and comparator definition：** 冻结 prescribed-path trajectory-generation 主张、五个 comparison conditions、fixed-profile 算法类别和公平性变量；在 comparator 实现前不生成正式随机表。
2. **G1 — Rotation relevance：** 用历史日志和高曲率/曲率反转快照比较 current 与 rotation-consistent 候选的模态传播、第一动作和 \(t(\sigma)/v(\sigma)\)。差异低于冻结数值与执行噪声门槛时可保留当前近似；差异可辨识时必须完成 rotation-only 小规模 RGB pilot，再选择唯一正式 release。
3. **G2 — Internal candidate screening：** W1/W2/W5 等候选只用于安全、实时、机制与模型侧 screening，检查状态是否产生局部 timing 差异而非统一降速；内部 proxy 不再承担最终物理有效性判决。
4. **G3 — Independent RGB efficacy pilot：** 在不进入正式推断的 H0/H0b development 路径上，先冻结一个最终 S-MPCC 候选、准确 block 数（推荐默认 \(n_{\mathrm{dev}}=4\)）、调试预算、\(\delta_{H,\mathrm{dev}}\)、success 门槛和无提前停止规则，再与 Smooth-only 做完整随机区组；fixed-profile 若加入，也必须在首个 RGB block 前冻结。记录 RGB、完成时间、tracking、执行层干预和失败。只有真实 RGB 方向达到冻结门槛且不由单一 block 驱动，才允许进入正式 freeze。若修改候选或 gate，应建立新的 development release 和完整新 pilot，不能在同一批数据上反复筛选。历史上 internal-model 排序与 RGB 排序相反，因此不能跳过此 gate。
5. **G4 — Trajectory and replay toolchain：** 冻结工具必须从 bag 一致导出 actual/reference \(s,\sigma,\kappa,t(\sigma)\)、Z1–Z5、\(v,\omega,a_x,a_y\)、完整 horizon、first action 和 method-native raw/post-gate/published command；纵向与横向各四个构造模型相位必须在正式前产生超过数值容差的 timing/action 差异，online-input 分支（即在线 solver-input state）replay 必须复现在线求解。
6. **G5 — Competitive tuning and fairness：** Smooth-match 与 fixed-profile 分别在独立 development 数据上冻结。前者只匹配完成时间；后者获得与 S-MPCC 合理相当的容器参数、硬约束和调试预算，并冻结 profile generator、输入参数、唯一 base-profile timing 旋钮、ZV 脉冲参数、profile hash 和唯一比较规则；默认采用 ±5% 完成时间匹配，但不得 time-warp shaped profile。若改做 Pareto，必须在 formal 前扩展工作点和矩阵。不得故意使用失调 baseline。
7. **G6 — Measurement, analysis, and release freeze：** 完成 RGB 标定/同步/重复性、路径 replay/hash、失败保留、区组顺序、comparison hierarchy、滤波/微分、trajectory feature 和 runtime 规则；随后才生成单一只读 `FREEZE_ID`。

G3 不是形式性新增项：0706 同日 development 复分析中，当前 `B_slosh` 相对 `B_smooth` 的 RGB p95/peak/RMS 分别恶化约 32.5%/26.4%/21.4%，而内部模型排序相反。因此“模型侧选权后直接进入正式物理检验”已被现有事实证明风险过高。

G4 同样仍是实际缺口：recorder 已保存 reference、odom、完整 horizon 和分层 command，但当前 `summarize_spmpc_real_trial.py` 尚不能从 odometry/path 独立重建 \(s_{\mathrm{proj}}\)、Z1–Z5、terminal completion 和论文过程图。必须先实现单独的冻结 trajectory-analysis pipeline。

Development RGB 可以用于选择方法 release，因为正式 H1 数据仍保持完全留出；这些 pilot 必须完整归档、明确标为训练/开发数据且永不并入正式推断。若任一关键 gate 失败，应先修改方法或测量链并重新建立新 release，而不是启动 40 次“碰结果”。

### 7.5 4.3 Measurements, Protocol, and Analysis

#### 4.3.1 Recorded signals and physical reference

按证据层记录：

- 路径（所有条件）：冻结参考路径/hash、由 odometry 独立投影得到的 \(s_{\mathrm{proj}}\)、\(\sigma=s_{\mathrm{proj}}/L\)、\(\kappa(s)\)、Z1–Z5 边界与进入/离开时间；online MPCC 另记录 OCP 虚拟进度 \(s_{\mathrm{ocp}}\)；
- 机器人：\((x,y,\theta,v,\omega)\) 和路径误差；
- timing：基于 \(s_{\mathrm{proj}}\) 的 actual \(t(\sigma)\)、各区段 traversal time，以及 fixed-profile 的参考 \(s(t)/v(s)/\omega(s)\)；
- OCP 输入（online MPCC）：实际送入 solver 的机器人状态、\(s_{\min}\)、有效 \(v_{\mathrm{ref}}\) 和必要的 pre-solve 状态；仅 S-MPCC 另记录实际送入 solver 的内部液体模态状态；
- OCP 输出（online MPCC）：第一控制量和通用完整预测 \(s,v,\omega,v_s,a,\alpha\) horizon；仅 S-MPCC 另记录 modal horizon 与 \(H_{\mathrm{modal}}\)；
- 执行（所有条件）：method-native raw、post-gate、最终 published command 和 limiter/fallback 标志；online 的 raw 是 solver command，Fixed-profile 的 raw 是 tracker command；
- 激励：\(\dot v\) 和 \(v\omega\)；
- 液体模型（仅 S-MPCC）：\((\eta_x,\dot\eta_x,\eta_y,\dot\eta_y)\) 与 \(H_{\mathrm{modal}}\)；
- Fixed-profile：profile reference/index/progress 与 tracker state/error/latency；
- 物理液面（所有条件）：\(H_{\mathrm{vis}}\)；
- runtime：online MPCC 记录 solve time/overrun/status，Fixed-profile 记录 tracking latency/status；所有条件记录 fallback、intervention 与实际控制间隔。

必须区分参考 timing、预测 timing 与真实执行。运动和激励指标使用 odometry 与 executed command；预测量只用于机制解释。RQ3 计算实验必须由冻结的 replay 工具导出完整 \(v,\omega,v_s,\eta,\dot\eta,a,\alpha\) horizon，不能只依赖 XY path、前三个预测点或 horizon 摘要。

论文中的 actual \(t(\sigma)\) 不使用 optimizer 自己的 \(s_{\mathrm{ocp}}\) 作为真值。正式分析前必须冻结 odometry-to-path 最近点/连续投影、单调 guard、投影回退、跳变、停滞、首次到达、插值、cross-track 超限和终端处理规则。由于每周期初值满足 \(s_{\mathrm{ocp},0\mid j}=s_{\mathrm{proj}}(t_j)\)，同一节点的差恒为零，不能作为证据；若保留一致性诊断，应报告 \(k>0\) 时保存的预测 \(s^\star_{\mathrm{ocp},k\mid j}\) 与未来 odometry 投影 \(s_{\mathrm{proj}}(t_j+k\Delta t)\) 的误差，并预先冻结插值、有效 horizon 与汇总规则。该诊断不替代 actual \(t(\sigma)\)。

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

- progress–time curve \(t(\sigma)\) 与 Z1–Z5 traversal-time vector；
- 冻结定义下的弯前减速提前量、入弯/反转速度和出弯再加速延后量；
- S-MPCC 相对 comparator 的全局常数速度缩放残差，用于检查差异能否由统一降速解释；
- \(H_{\mathrm{modal}}\) p95；
- \(a_x\) RMS；
- \(|a_y|\) p95；
- \(\alpha\) 或命令变化指标；
- 执行层干预比例 \(r_{\mathrm{int}}\) 和 raw-to-published command 差异。

实时性指标包括：

- solve-time median、p95、maximum；
- solve-budget overrun rate 与 observed command-intervention inter-arrival gap proxy；
- solver failure 和 fallback；
- 实际控制频率；
- fixed-profile 的离线生成时间与在线跟踪开销。

一次完整 trial 才是统计样本，视频帧和过程曲线采样点不是独立样本。\(t(\sigma)\)、\(v(\sigma)\) 等网格曲线用于描述机制，不对每个网格点分别做显著性检验；区段边界、onset 阈值、滤波和派生公式必须在正式结果前冻结。

#### 4.3.3 Randomization, paired analysis, and failure handling

写清：

- Stage I 每个 block 内包含五个 conditions 各一次；Stage II 的三条件 block 独立随机化，不把跨路径或跨日期同编号强行连接成配对；
- 方法顺序在新正式协议中重新生成位置平衡表，旧 v1.0 顺序不得沿用；
- RQ1 第一主比较预注册为 S-MPCC − Smooth-only，回答相对竞争性通用平滑的物理收益；
- RQ2 关键 novelty 比较为 S-MPCC − Fixed-profile，completion-time confound 比较为 S-MPCC − Smooth-match；
- S-MPCC − Baseline 为关键次比较，Smooth-only − Baseline 为内部机制诊断；
- 确认性 family 推荐采用固定顺序 gatekeeping：先检验 S-MPCC − Smooth-only；只有其效应达到预冻结 \(\delta_H\)/区间/随机化准则后，才把 S-MPCC − Fixed-profile 作为确认性 novelty test。任一步失败，后续只作描述性结果；
- exact/randomization inference 必须复现实际五条件区组的随机化机制；Smooth-match、Baseline 和其他次比较采用预声明层级或 Holm 调整；
- 完整“物理有效且优于固定液体感知 timing”主张要求前两项均通过，同时 Smooth-match 时间门和 success 容限成立；不能根据哪一项显著再改变主次；
- 主要估计量为 block 内 paired difference；
- 报告原始 trial 点、配对线、平均配对差、相对变化和以 block 为单位 bootstrap 的 95% CI；
- 第一主比较和关键 novelty 比较均采用与实际五条件区组分配机制一致的 exact randomization/permutation inference，并报告 leave-one-block-out；paired sign-flip 若保留，只作为 block-difference 对称性假设下的敏感性分析，不称为实际设计的 exact randomization inference；
- \(n=8\) 只是采集目标，不自动等于统计确认；
- solver failure、timeout、tracking failure 或 safety termination 计为方法失败；
- fallback 必须保留为方法相关事件；它使整条 trial 失败还是按冻结 event-rate 容限判定，必须在正式采集前确定；
- 只有与方法无关的采集故障可以排除并在同一区组补采；
- 不能按结果是否符合预期选择 trial 或代表曲线。

连续液面指标与失败必须采用预声明的双层 estimand：

1. 对全部有效正式尝试报告 success/failure 和原因，分母固定为计划单元；
2. 连续 RGB/timing 主估计量使用同一 block 中两方法均产生定义良好指标的 pair，并明确报告 \(n_{\mathrm{pair}}/n\)，称为 success-conditional paired effect；
3. 另做 failure-penalized sensitivity，例如把方法相关失败排在全部成功 trial 之后的 worst-rank/复合 win-loss 分析；
4. 完整性能主张要求连续效果成立且 success 不越过预冻结劣化容限，不能以 complete-case 的漂亮结果掩盖方法失败，也不能用补跑成功替换失败。

### 7.6 4.4 RQ1: Physical Effectiveness and Trajectory Redistribution

#### 4.4.1 Physical liquid response

RQ1 只使用 Stage I 的 \(C_1+H_1\) 五条件 block 建立核心物理事实。第一主比较为：

- S-MPCC − Smooth-only：第一主比较；
- S-MPCC − Baseline：关键次比较；
- Smooth-only − Baseline：机制诊断比较。

正文主结果表采用五条件或更紧凑的 paired-contrast 形式，报告：

- RGB p95；
- full-motion 与 post-arrival RMS；
- completion time；
- path-error p95；
- success。

表中的方法汇总只是描述性结果，正式解释依赖 block-paired effects、区间、原始点、失败分母和 leave-one-block-out。RQ1 不用内部 \(H_{\mathrm{modal}}\) 判定物理胜负。

#### 4.4.2 Motion and excitation redistribution

用高风险路径的过程图回答“执行轨迹如何变化”。所有实际执行曲线以 \(\sigma=s_{\mathrm{proj}}/L\) 对齐，预测曲线另用清楚标记的 \(s_{\mathrm{ocp}}\)，建议最小共享面板为：

1. path curvature；
2. actual progress–time \(t(\sigma)\) 或 Z1–Z5 traversal time；
3. executed \(v\) 与必要的 \(\omega\)；
4. realized \(a_x,a_y\)；
5. \(H_{\mathrm{vis}}\)；
6. \(H_{\mathrm{modal}}\) 仅在版面允许时作为内部机制曲线。

重点讨论 S-MPCC 是否在 Z2/Z3/Z4 提前减速、改变入弯/反转速度、延后或提前再加速，并使实际激励时机与随后液面响应呈机制一致变化。XY 轨迹只证明完成同一几何任务，不作为核心机制图。

#### 4.4.3 Mechanism and task tradeoffs

讨论：

- Smooth-only 是否确实降低通用运动变化；
- S-MPCC 是否降低液面但显著损害路径跟踪或成功率；
- S-MPCC 的运动变化是否集中于关键路径区段，还是可由单一常数速度缩放近似解释；
- raw solver timing 差异是否经过共享执行层并出现在 odometry；
- 不能仅凭 \(H_{\mathrm{modal}}\) 下降宣布物理效果成立。

详细机制表可移至补充材料，但正文必须保留关键过程图和任务权衡讨论。

### 7.7 4.5 RQ2: Competing Timing Explanations

RQ2 使用同一 Stage I block 中的两类关键 comparator：

\[
\underbrace{\text{Smooth-match MPCC}}_{\text{检验整体减速解释}}
\quad\text{和}\quad
\underbrace{\text{Model-informed fixed-profile}}_{\text{检验 fixed physics-aware timing}}
\quad\text{vs.}\quad
\text{S-MPCC}.
\]

Smooth-match 的参考速度只按独立 pilot 完成时间调节。Fixed-profile 使用相同容器物理参数，并在独立 development 数据上通过唯一 base-profile timing 参数反复重新生成完整 shaped profile，冻结最终 profile 与 hash；由 \(\omega_1,\zeta\) 决定的 ZV 脉冲参数不能为追求等时间而缩放。本提案推荐把正式主比较冻结为 ±5% completion-time matching；time–RGB Pareto 只在正式前预先生成多个固定 profile 且为每个工作点分配独立正式 trial 时才成立，否则只能作为 development 描述。不得按正式 trial 是否落入 ±5% 来删样本，也不得查看正式 RGB 后再次调节。

本节主要用图而不是大表，展示：

- 完成时间—RGB p95 配对散点；
- fixed-profile 与 S-MPCC 的 \(t(\sigma)\)/区段时间；
- \(v(\sigma)\)；
- \(a_y(\sigma)\)；
- \(H_{\mathrm{vis}}(\sigma)\)。

本节要分别回答：总时间相近时收益是否仍存在；已知名义液体参数并预先整形 timing 后，S-MPCC 端到端系统是否仍产生额外收益。若 S-MPCC 只优于 Smooth-match 而不优于 Fixed-profile，只能结合 RQ3 主张模型传播状态会改变在线计划，不能主张在线动态记忆在物理性能上优于固定 physics-aware timing。

### 7.8 4.6 RQ3: Model-State-Dependent Online Generation and Execution

#### 4.6.1 Controlled phase-dependent timing

在正式采集前冻结两个机制检查点：Z1/Z4 的纵向检查点和 Z2/Z3 的横向检查点。固定机器人状态、路径、容器参数和模态能量，只改变一个方向上人为构造的内部模态状态相位，比较：

- 完整 predicted \(s,v,\omega,v_s,a,\alpha\) horizon；
- predicted progress/time law 与关键区段进入时刻；
- optimized first action；
- predicted modal response。

纵向和横向均使用四个等能量内部模型相位状态，幅值与检查点在查看结果前冻结。若不同相位只产生 solver 数值噪声级差异，不能声称传播模态相位对 trajectory generation 具有数值可辨识作用；即使差异成立，也只说明优化器对其内部模型相位敏感，不验证真实液体带符号相位。

#### 4.6.2 Online-input/zero-state replay of formal runtime state

四相位只检验 OCP 对构造内部状态的原理敏感性；正式高风险日志还必须使用同一 immutable pre-solve snapshot 分叉：

\[
\mathcal P_{\mathrm{actual},j}
=\mathcal P(\hat{\mathbf x}_{r,j},\hat{\mathbf x}_{\ell,j},\mathrm{path}_j,\mathcal S_j),
\qquad
\mathcal P_{\mathrm{zero},j}
=\mathcal P(\hat{\mathbf x}_{r,j},\mathbf 0,\mathrm{path}_j,\mathcal S_j).
\]

这里的 `actual` 分支只表示在线 solver 实际收到的模型传播内部状态，不表示传感器测得的真实液体状态。该分支必须先复现在线 solver status、第一动作和 raw command。正式比较在 trial level 汇总 \(\Delta a_0,\Delta\alpha_0,\Delta v_{s,0}\)、完整 horizon timing difference 和超过冻结容差的比例。未经 terminal/gate/rate limiter 双分支回放时，只称 optimized first-action/timing difference，不称 counterfactual executed command。

该实验只能表明模型传播状态影响了优化，不能表明该状态等于真实液体相位，也不能推断 zero-state 反事实会产生更好的物理液面。

#### 4.6.3 Execution-chain preservation and real-time feasibility

使用正式日志报告 raw solver、post-gate、published command 和 odometry 差异保留率、\(r_{\mathrm{int}}\)、solve-time median/p95/maximum、solve-budget overrun、solver failure、fallback 与 achieved frequency。只有 timing 差异通过共享执行层并出现在实际运动中，才能把 RQ1 的机制解释连接到 S-MPCC OCP；否则只能归因于最终执行轨迹。

### 7.9 4.7 RQ4: Path-Feature Selectivity and Limited Configuration Transfer

#### 4.7.1 Low-risk path as a selectivity task

Stage II-A 使用冻结 \(C_1+L_1\) 比较 Smooth-only、Fixed-profile 和 S-MPCC。L1 不是简单重复 H1，而是检验：

- 高风险路径上观察到的 timing 机制是否与曲率/激励特征相关；
- 低风险路径上 S-MPCC 是否减少不必要介入，而不是始终全程保守；
- 相对两个竞争性 timing 策略的 RGB、完成时间、tracking 和 success 代价是否保持合理；
- \(t(\sigma),v(\sigma),a_y(\sigma)\) 的差异是否随路径风险下降。

正文优先报告 H1/L1 的 paired effect summary 与简化过程图，不用两张完整七面板图重复占版。跨路径同编号 block 不作配对，路径间比较是分层描述或预注册 effect-difference 支持分析。

#### 4.7.2 Conditional container transfer

Stage II-B 只有在有限跨容器迁移仍作为预注册结果目标且触发规则满足时执行。跨容器时：

- S-MPCC 只按预声明映射更新 \(\omega_{1,c},c_{h,c},\eta_{\mathrm{ref},c},\dot\eta_{\mathrm{ref},c}\) 等物理参数；
- Fixed-profile 使用同一 \(C_2\) 参数重新离线生成固定 timing；
- Smooth-only、S-MPCC 权重、\(\zeta\)、horizon、solver、路径、硬约束、共享执行层和测量链保持不变；
- 两容器采用预先冻结的液体/freeboard 口径，并满足视觉采样带宽准入。

结果只做容器内 randomized blocks，不把跨日期同编号连成配对线。正文报告每个容器内 S-MPCC 相对 Smooth-only/Fixed-profile 的方向、效应量、完成时间、tracking、success 和带批次限制的 effect difference。毫米值和归一化指标同时报告。

固定机器人/路径/归一化模态状态、只切换参数集的 replay 只能说明规划对参数敏感；没有错误参数物理组时，不能声称正确参数更新对真实物理效果具有必要性。

#### 4.7.3 Selectivity and transfer interpretation boundaries

- H1 有效而 L1 差异很小，可支持“按路径风险选择性介入”，不能写成所有路径上都显著降低液面；
- “L1 差异很小”必须依据预冻结的等价/非劣界与 H1→L1 effect-difference 判据，不能把 \(p>0.05\) 当成无差异证据；
- L1 上出现明显时间或 tracking 代价，必须作为保守性边界报告；
- C2 未执行时，容器参数化只属于方法定义和计算敏感性，不属于物理迁移结果；
- C2 中仅 S-MPCC 成功运行不能证明几何参数更新必要，必须看容器内相对 comparator 的物理效果；
- “有限迁移”必须同时满足预冻结的 C2 容器内液面效应、任务代价和 success 规则；非显著 container interaction 不能证明跨容器等效；
- 不使用第二路径或第二容器的更多次数掩盖 Stage I 核心方向不稳定。

### 7.10 4.8 Supporting Model and Measurement Diagnostics

这是实验章的支持性部分，不是第五个 RQ，也不能替代 RGB 主结果。正文只保留测量有效性和模型边界所必需的信息；完整审计进入仓库复现材料。

#### 4.8.1 Visual validity and model consistency

必须分开两层：

1. **Measurement admission（正式前必需）：** 相机固定、ROI、手动曝光/白平衡、像素—毫米标定、静止噪声、重复性、缺帧、clipping、同步和起始静稳规则；
2. **Model consistency（支持性）：** \(H_{\mathrm{modal}}\) 与非负 \(H_{\mathrm{vis}}\) 包络的幅值/趋势一致性与参数敏感性。

支持性诊断可包括：

- \(H_{\mathrm{modal}}\) 与 \(H_{\mathrm{vis}}\) 的代表性对齐时序；
- Ferrari-form signed bias：保持 \(H_{\mathrm{modal}}\) 积分作分母；
- 单独命名的 absolute disagreement、RMSE、raw correlation 和局部低估量；
- \(\omega_1,\zeta,c_h\)、初始状态和执行延迟敏感性；
- actual/zero/phase-flip replay 的完整分布和 reproduction/failure rate；
- 若执行 C2，再加入正确/错误参数的计算 mismatch。

若采用本提案，新 K6 可用 S-MPCC 总体随证据包自然增长为 Stage I 8、完成 64 后 16、完成条件性 88 后 24 个计划单元；旧 K6-FID-v1.0 的 32 单元口径必须废止或升级。禁止 per-trial 最佳时滞、幅值拟合、模型 topic 回退和依据正式结果重调参数。K6 只能评价非负幅值包络，不能验证带符号相位或 rotation consistency。

#### 4.8.2 Exploratory long-horizon propagation and command-regularity audit

该项目默认不进入正文实验结构，也不设正式样本数。只有 RQ1–RQ4、常规诊断和投稿正文已经闭环后仍有资源，才另立 exploratory protocol 执行。

可检查多个连续高风险曲率序列之间的 modal–vision disagreement、\(H_{\mathrm{vis}}\)、轨迹误差、求解失败、首末序列差异、total-variation rate、方向反转率、高频能量和执行层干预。主矩阵中 raw/post/published command 与 \(r_{\mathrm{int}}\) 的最小执行链审计仍是必做项，不能等待该探索性项目补足。

该项目不构成闭环稳定性、递归可行性、长期无误差累积或命令非劣性的证明。即使结果支持，也只能描述为特定测试时长与条件下的探索性观察。完成后优先放入补充材料；未执行时不保留 pending，占用的试次也不加入正式实验总数。

### 7.11 实验章正文图表规划

正文建议控制为：

#### 两张必需表 + 一张条件表

1. 五个 comparison conditions、关键公平性变量与 40/64/条件性 88 矩阵（可合并）；
2. Stage I 核心物理/任务结果和预注册 paired contrasts；
3. Stage II 路径/容器扩展结果，仅在对应数据完成时加入。

#### 四组核心图

1. 实验装置、H1/L1、容器坐标和 RGB ROI；
2. RQ1 block-paired RGB 效果 + \(\kappa\rightarrow t(\sigma)/v/a_y\rightarrow H_{\mathrm{vis}}\) 核心过程链；
3. RQ2 Smooth-match 与 Fixed-profile 的完成时间/trajectory-timing 对照；
4. RQ3 四相位计划、actual/zero first-action/timing difference 与 runtime ECDF。

RQ4 优先并入一张紧凑 extension 表；C2 未执行时不为其预留空图。正文不同时铺开所有五条件的七行过程曲线：主图保留 S-MPCC、Smooth-only、Fixed-profile，B0/Smooth-match 的完整过程曲线进入复现材料，物理主结果仍报告全部五条件。

长时传播/命令规律性审计不进入上述五组核心图，默认只在补充材料中保存探索性全分布。只有其结果对解释主实验中的明确异常具有不可替代作用且篇幅允许时，正文才增加一句说明或一张紧凑表。

表格应出现在首次讨论相应 RQ 的附近。图表预算以 RA-L 6–8 页完整稿为约束，不能让 QC、K6 或 conditional C2 挤掉 RQ1/RQ2/RQ3 的核心证据。

### 7.12 本章不应写入

- 把旧三次 pilot 混入正式 \(n=8\)；
- 把 frame 当成独立样本；
- 根据结果好坏排除 trial；
- 用模型量替代 RGB 主结果；
- 把方法失败归类为采集故障后重跑；
- 把跨路径或跨日期同编号画成配对线，或据此声称强 path/container 因果交互；
- 把同一可变 solver 上顺序执行的 actual/zero 求解当成公平反事实；
- 把计算 mismatch 写成参数更新对真实物理迁移的必要性证据；
- 把 optimized first-action difference 写成机器人已经执行的反事实命令；
- 把长时审计包装成 RQ5、稳定性证明或核心贡献的必需证据；
- 把预测 timing 当作 actual executed trajectory，或只凭 XY 轨迹重合/分离解释机制；
- 把本文 fixed-profile comparator 称为 Hamaguchi/Lim 原方法忠实复现；
- 因 comparator 表现过强而在正式分析中删除，或给 comparator 更少的参数、调试预算和硬约束权限；
- 把长时探索性试次加入 40/64/88 次正式总数，或在未执行时保留 pending 小节和无退化/防抖主张；
- 在没有统一复现协议的情况下加入宽泛 external planner 排名。

## 8. 第五章：Conclusion and Limitations

### 8.1 本章回答的问题

结论只回答：四个 RQ 最终得到了什么证据支持，以及这些结论可以推广到哪里、不能推广到哪里。

### 8.2 推荐内容顺序

#### 第一段：方法总结

用一段话回顾 S-MPCC：沿给定路径，将 acceleration-level MPCC、虚拟进度和两个正交方向上的第一液体模态联合预测，通过 odometry 保存液体动态记忆并滚动执行第一步控制。

#### 第二段：正式结果总结

数据完成后按 RQ1–RQ4 顺序写入：

- 相对 Baseline/Smooth-only 的物理液面与局部轨迹 timing 变化；
- 等完成时间和 Model-informed fixed-profile 条件下的结果；
- 相位相关规划、actual/zero replay、执行链与实时求解结果；
- 低风险路径特征选择性，以及仅在完成条件性扩展时加入的有限跨容器结果。

只写最重要的效应量和区间，不重复整张结果表。
长时传播与命令规律性审计原则上不进入结论。只有其探索性结果对解释主实验中的异常或适用边界具有不可替代作用时，才可增加一句明确标注 exploratory 的受限观察。

#### 第三段：局限性

明确：

- 给定路径而非完整避障导航；
- 容器安装在旋转中心；
- 低阶模型而非高保真自由液面；
- 模态状态没有 RGB 校正；
- 无形式化 spill-free guarantee；
- fixed-profile 是本文的 prior-art-inspired 实现，不是原论文直接复现；
- 正式结论只覆盖冻结路径、容器、液体和速度范围。

#### 第四段：后续工作

后续方向可以包括：

- 偏心容器的刚体激励模型；
- 液面或其他传感器状态校正；
- collision-aware S-MPCC；
- 无液体状态传感器的低复杂度 CA-MPCC，例如基于 \(\sqrt{\dot v^2+(v\omega)^2}\) 的合加速度约束；
- 更高忠实度的 Lim/Hamaguchi 原方法复现或多种液体感知近邻比较；
- 更丰富容器、液体和负载条件。

这些内容只能写成 future work，不能作为当前能力。

### 8.3 本章不应写入

- 新公式、新实验或首次出现的贡献；
- 超过正式结果支持范围的泛化结论；
- “全面优于现有方法”；
- “保证不溢出”；
- 将未完成实验写成已经验证的事实。

## 9. Repository / Permitted Supplementary Material 的内容边界

仓库复现材料以及期刊明确允许的补充形式服务于复现、审计和正文减负，不承担核心结论；不能假定普通附录可绕过 RA-L 正文页数。当前应包含：

1. solver、horizon、integration、warm start 和 fallback 的完整设置；
2. Baseline、Smooth-only、S-MPCC 的全部权重和 normalizer，以及 Smooth-match 配置；
3. Fixed-profile generator、理论来源、输入参数、离线 profile/hash、生成时间、冻结 tracker（共同 tracker 可行时优先）与公平性审计；
4. 实验装置、路径、容器、相机、同步和软件提交的 freeze checklist；
5. required signals、trajectory extraction 与 derived metrics checklist；
6. 五条件核心 40、双路径 64 和条件性跨容器 88 的正式矩阵与完整失败记录；
7. rotation/release gate、独立 RGB development pilot 和旧三次 development-only 数据；
8. 旧 “S-MPCC + enhanced smoothing” 的开发记录；
9. 完整 sensitivity、runtime、trajectory feature 和 command-chain 表；
10. actual/zero/phase-flip replay 的快照、复现容差、完整 horizon 和 trial-level 分布；
11. 仅在执行 C2 时加入正确/错误参数的计算 mismatch；
12. 正文放不下的 B0/Smooth-match/低风险过程曲线和机制诊断；
13. 仅当探索性长时审计完成时，收录其独立 protocol 和全分布，并明确不计入正式总数。

当前 `spmpc_paper_core/main.tex` 不会自动编译 `supplementary/supplementary_material.tex`。投稿阶段可根据期刊要求把它作为独立补充 PDF，或由完整论文入口显式 `\input`/`\include`；在未显式引用前，它不会出现在 core PDF 中。

补充材料不能用来隐藏以下关键内容：

- 方法的核心状态、动力学和 OCP；
- 五个核心 comparison conditions 的定义与 fixed-profile 公平性；
- 主要结果指标；
- RQ1、RQ2 和 RQ3 的核心结果；
- 关键失败率与实时性结论。

## 10. 章节—证据映射

| 核心判断 | 主要出现位置 | 必要证据 |
| --- | --- | --- |
| 液体具有动态记忆，普通平滑不能显式表示相位 | Introduction、Related Work、RQ3 | 文献定位、构造的纵横向内部模态相位与 actual/zero replay；不声称真实 signed phase 已测得 |
| S-MPCC 在给定路径上优化模型状态相关的有限时域 progress | Method | \(\mathbf p(t)\approx\mathbf r(s_{\mathrm{ocp}}(t))\)、增广状态、progress control、完整 predicted horizon；actual timing 由 odometry 独立测量 |
| 物理液面得到改善 | RQ1 | Stage I block-paired \(H_{\mathrm{vis}}\) 与失败记录 |
| 改善不只是通用平滑 | RQ1 | S-MPCC vs Smooth-only；Baseline 作为内部锚点 |
| 改变的是局部 timing/激励而非统一降速 | RQ1、RQ2 | \(t(\sigma)\)、Z1–Z5、局部 feature、常数缩放残差与 executed excitation |
| 改善不只是总完成时间更长 | RQ2 | S-MPCC vs Smooth-match 的预冻结等时间比较 |
| S-MPCC 端到端系统相对 fixed physics-aware timing 有额外价值 | RQ2 | 预冻结等完成时间下的 S-MPCC vs Fixed-profile 物理比较；memory 特异作用另由 RQ3 支持 |
| 正式 online-input 模型状态改变优化动作 | RQ3 | 同一 pre-solve snapshot 下 actual/zero first-action 与 horizon timing difference |
| 优化器对内部模态相位具有原则和数值上可辨识的敏感性 | RQ3 | 纵横向等能量构造相位的完整预测计划；不外推为真实相位估计准确性 |
| OCP 差异到达执行层且方法可实时运行 | RQ3 | raw/post/published/odom、\(r_{\mathrm{int}}\)、solve-time ECDF、failure/fallback |
| 方法对不同路径风险具有合理选择性 | RQ4 | H1/L1 分层的 RGB、timing、tracking 和 success |
| 所报告容器配置间可实现有限迁移（条件性） | RQ4 | 完成 C2 且通过预冻结物理—任务准则后的容器内 S-MPCC vs comparators；计算 mismatch 只作机制支持 |
| 内部模型具有何种一致性和局限 | Supporting Diagnostics | modal–vision 幅值包络与敏感性，不替代 RGB 或相位验证 |

## 11. 统一术语和符号

后续所有章节应统一：

| 项目               | 统一写法                                                            | 不再使用或谨慎使用                                  |
| ------------------ | ------------------------------------------------------------------- | --------------------------------------------------- |
| 论文方法名         | S-MPCC                                                              | SPMPC 作为论文名                                    |
| 任务层级           | prescribed-path trajectory generation / model-state-conditioned online progress | path planning、free trajectory planning、严格全局 time law |
| 固定与可变对象     | geometry \(\mathbf r(s)\) fixed; finite-horizon \(s_{\mathrm{ocp}}\) and chassis motion optimized; actual timing measured from odometry | “相同轨迹只改变速度”、把 \(s_{\mathrm{ocp}}\) 当实际进度 |
| 内部 OCP           | model-propagated-state-conditioned path-progress MPCC formulation     | 未限定的真实 liquid-state feedback、泛称完整自主 planner |
| 物理参考液面       | calibrated vision-based experimental reference,\(H_{\mathrm{vis}}\) | absolute ground truth                               |
| 内部模型响应       | \(H_{\mathrm{modal}}\)                                              | \(H_{\mathrm{diag}}\)、物理真值                     |
| 液体模态           | first lateral mode represented in two orthogonal directions         | two-mode model                                      |
| 核心方法组         | S-MPCC                                                              | \(B_{\mathrm{ours}}\)                               |
| 增强平滑组         | Smooth-only MPCC                                                    | ours-smooth                                         |
| 等时间组           | Smooth-match MPCC                                                   | 写成独立算法贡献或与三个内部 variants 混淆          |
| 固定 timing 对照   | Model-informed fixed-profile / `FixedProfile`（实现后冻结）         | Hamaguchi/Lim reproduction（未忠实复现时）          |
| 代码映射           | Baseline=`B0`、Smooth-only/Smooth-match=`B_smooth`、S-MPCC=`B_slosh`；FixedProfile 走独立 profile+frozen-tracker backend，无 S-MPCC variant | 用 `B_ours` 或虚构 `B_fixed` 代表相应方法 |
| RQ3 反事实量       | online-input/zero-state optimized first-action/timing difference；`actual`=online solver-input internal state | 把 `actual` 写成真实液体状态，或未经过执行层却称 counterfactual executed command |
| 长时审计           | exploratory supplementary diagnostic                                | RQ5、stability proof、正式扩展计数、必做主实验      |
| freeboard fraction | \(\lambda_H\)                                                       | \(\rho_f\)                                          |
| 路径范围           | prescribed geometrically feasible path                              | obstacle-free guarantee / safe under all conditions |

## 12. 推荐写作与执行顺序

当前论文主线、核心实验章节和 0717 候选现场协议已经对齐，但不能启动 formal。后续顺序为：

1. **冻结尚未决项：** 五条件与 40→64→条件性 88 已作为候选主线对齐；继续冻结 Fixed-profile 唯一实现、正式样本量，以及 C2 是否保留为预注册条件扩展。
2. **完成其余上位协议升级：** 0717 矩阵/命令已升为候选 v2.0；继续同步实验总章、K6/replay/trajectory analysis 口径和 freeze template。旧 v1.0 只保留为被 supersede 的历史版本。
3. **选择方法 release：** 完成 rotation relevance、内部候选 screening 和独立 RGB efficacy pilot，只允许一个通过门槛的动力学/代价 release 进入正式矩阵。
4. **实现并验收 comparators/tools：** 完成 Fixed-profile、Smooth-match、trajectory extractor、完整 horizon/actual replay、视觉链和 command-chain smoke。
5. **冻结正式设计：** 固定五条件配置、两条路径、可选 C2、样本数、primary/trajectory outcomes、comparison hierarchy、失败规则、随机表和全部 hash，生成唯一 `FREEZE_ID`。
6. **采集 Stage I 40：** 五条件 randomized blocks；正式期间不改方法、baseline、路径、测量或分析。
7. **完成核心 gate：** 同时检查 RGB、matched time、Fixed-profile、局部 timing、phase/actual-zero、执行链和 runtime。核心失败即停止，不用扩展掩盖。
8. **按预注册规则扩展：** 核心通过后执行 L1 到 64；只有预先保留 C2 且触发规则满足时才执行条件性 88。
9. **填写 RQ1–RQ4：** 先生成 trial-level 原始点、配对效应、过程曲线、区间、与实际区组分配一致的 randomization inference 和 leave-one-block-out，再写结果解释。
10. **最后同步论文：** 先修改 `spmpc_paper_core`，再同步完整论文；Abstract 与 Conclusion 只使用正式结果支持的关键数字。

## 13. 正式采集前的论文冻结检查

在开始正式实验前，论文与配置至少应共同确认：

- [ ] 本提案已获确认，实验总章、矩阵、现场协议、K6/replay/trajectory 口径和 freeze template 已升级为同一新版本；旧 v1.0 已明确标为 superseded，正式状态仍从 `NO-GO` 开始；
- [ ] 全文方法名统一为 S-MPCC；
- [ ] rotation relevance、内部 candidate screening 和独立 RGB efficacy pilot 已完成，只选择一个通过门槛的方法 release；
- [ ] 五条件映射已冻结：B0、Smooth-only、Smooth-match、Model-informed fixed-profile、S-MPCC；不存在核心 `B_ours`；
- [ ] Fixed-profile 的唯一算法、理论来源、容器参数、生成代码、profile/hash、离线时间、冻结 tracker（共同 tracker 可行时优先）和唯一 completion-time（或另行扩展的 Pareto）规则已归档，未冒充 Hamaguchi/Lim 原方法；
- [ ] 若 Fixed-profile 采用 ZV，脉冲幅值与 \(\Delta T\) 固定；完成时间只通过预声明的 unshaped base-profile 参数调节并完整重生成，未对 shaped profile 事后 time-warp；
- [ ] 五条件的调试预算、公平性、运动硬约束、共享执行层和失败规则已审计，不存在故意失调 comparator；
- [ ] Stage I 40、Stage II-A 64 和条件性 Stage II-B 88 的组成、\(n\)、comparison hierarchy、随机表和停止规则已冻结；
- [ ] \(n\) 已依据 \(\delta_H\)、paired SD、目标 CI/power 和失败/视觉无效率计算；若不等于 8，全部计数名称已同步更新；
- [ ] C2 若被保留，实物、参数、视觉准入和条件性触发规则已在首条正式 trial 前冻结；若不保留，摘要与贡献不写跨容器物理迁移；
- [ ] L1 的等价/非劣界和 H1→L1 effect-difference 判据已冻结；若保留 C2，其容器内液面最小效应、任务代价、success 容限、主要 comparator 与通过规则已冻结；
- [ ] 40/64/88 只表示证据范围，不允许阶段间调参；新 release 与旧数据独立归档；
- [ ] \(\zeta\) 的实际值与来源已填写；
- [ ] \(H_{\mathrm{modal}}\) 为 modal-only，抛物面项关闭；
- [ ] \(H_{\mathrm{vis}}\) 的标定、同步和缺失帧规则已冻结；
- [ ] 全运动窗口 primary、10%–90% 敏感性窗口、\(Z_1\)–\(Z_5\) 和 5 s post-arrival window 已冻结；
- [ ] \(s_{\mathrm{ocp}}\) 与 odometry-derived \(s_{\mathrm{proj}}\) 已分开；projection/guard/jump/stall/interpolation/cross-track 规则、\(t(\sigma)\)、segment time、减速/再加速 feature、常数缩放残差和 \(a_x/a_y\) 已冻结并通过 smoke；
- [ ] 每个 trial 的起点/航向、零命令、ring-down 验证的 \(T_{\mathrm{settle}}\)、可用 monitor、适用状态 reset、最大等待与 split-block 规则已冻结；运动前至少 2 s RGB 已由 timestamp 留证，采集后离线 visual-start QC 的 acquisition-failure 规则已冻结；
- [ ] Smooth-match 只用独立 pilot 的完成时间调节；
- [ ] efficacy pilot 已在首个 RGB block 前冻结唯一 candidate、准确 \(n_{\mathrm{dev}}\)、调试预算、\(\delta_{H,\mathrm{dev}}\)、success 门槛和无提前停止规则；
- [ ] 每个 backend 的 method-native raw、post-gate、published topic/publisher contract 和 executed-command limits 已分别冻结、记录并验收；
- [ ] \(\epsilon_v,\epsilon_\omega,r_{\mathrm{int,max}}\) 已冻结；
- [ ] terminal、gate、delay、rate 和 fallback 的共享配置/hash 在五个 conditions 中一致，fallback 的 trial-level/event-rate 规则已冻结；
- [ ] \(\delta_H\)、S-MPCC−Smooth-only → S-MPCC−Fixed 的顺序 gatekeeping、实际五条件随机表对应的 exact randomization inference、block bootstrap、leave-one-block-out 和次比较调整已冻结；paired sign-flip 若保留只作额外敏感性；
- [ ] 正式随机区组顺序已生成；
- [ ] 失败 trial 与采集故障规则、\(n_{\mathrm{pair}}/n\)、success-conditional estimand、success 容限和 failure-penalized sensitivity 已写入实验协议；
- [ ] 纵向/横向构造内部模态四相位的幅值和检查点已冻结，且结论明确不验证真实 signed liquid phase；
- [ ] replay 工具能从同一 pre-solve 快照克隆 actual/zero 分支并导出完整 horizon；
- [ ] `actual` 已定义为 online solver-input internal state，且该 replay 能在冻结容差内复现在线第一控制量和 raw solver command；
- [ ] 计算 mismatch 不会被写成物理必要性证据；
- [ ] K6 已按新矩阵升级为 8/16/24 个 S-MPCC planned units，视觉测量准入与支持性 fidelity 口径已分开；
- [ ] 若另行执行探索性长时审计，已建立独立 protocol、目录和停止规则，且不会改变 40/64/88 次计数；
- [ ] 正式软件 revision、文档、配置和分析脚本已归档；
- [ ] 所有历史数据、rotation/efficacy pilot 和 baseline tuning 数据明确标为 development，不计入正式 \(n\)。

## 14. 当前版本的最终叙事

整篇论文最终应让读者得到以下清晰认识：

> S-MPCC 的价值不在于单纯让机器人运动更平滑，也不在于提出新的高保真液体模型或真实液体状态观测器。它在给定几何路径上把 odometry 驱动的低阶内部液体模态状态放入 path-progress MPCC，使模型传播的位移、速度与相位条件化有限时域虚拟进度、底盘运动和纵横向激励。论文用 Baseline、Smooth-only、Smooth-match 和 Model-informed fixed-profile 分层检验普通控制、通用平滑、整体减速和液体感知 fixed timing 等竞争解释，再用构造内部相位、online-input/zero-state replay、执行链和 runtime 检验 model-state-conditioned online generation；这些机制实验不验证真实 signed liquid phase，也不单独识别 memory 的物理因果必要性。真实物理效果始终由独立 \(H_{\mathrm{vis}}\) 承担，路径选择性和有限容器迁移只在对应阶段达到预冻结准则时成立。

上述最终叙事不依赖长时传播与命令规律性审计，也不预设必须做到 88。Stage I 40 是核心闭环，64 提供更自然的双路径轨迹证据，88 只在预先保留有限跨容器迁移目标且 gate 通过时形成条件性扩展。若 S-MPCC 未在独立 RGB pilot 或正式 Stage I 中优于竞争性对照，应停止并修改方法，而不是依靠更多路径、容器或诊断包装结果。

## 15. 尚待冻结的关键决策

在实现正式代码链或生成 formal freeze 前，需要明确冻结：

| 决策 | 推荐默认项 | 选择影响 |
| --- | --- | --- |
| Stage I 结构 | 五条件同一 randomized block，\(5\times8=40\) | 用 Fixed-profile 替代旧矩阵中第二次 S-MPCC；需重做随机表、配对与 K6 |
| Fixed-profile 类型 | Hamaguchi-inspired two-impulse ZV fixed-path timing | ZV 最接近 WMR input-shaping 文献谱系且较易落地；同模型离线 retiming/OCP 是更强的受控 fixed-vs-online 方案但实现更重。二者只能预先选一个标签，不能事后择优 |
| Fixed-profile 公平性 | 推荐独立 pilot 后冻结 ±5% 完成时间匹配；只调 unshaped base-profile 参数并重生成，ZV 脉冲不缩放 | 正式样本不因失配删除；单一工作点不声称 Pareto；禁止 shaped-profile time-warp |
| Comparator 解释 | 端到端 system comparison；共同 tracker 可行时优先，并单报 tracking/修正量 | phase/actual-zero 只识别 memory 对优化的作用，不主张物理因果必要性 |
| 正式 release | current 与 rotation-consistent 先过 relevance/RGB gate，再二选一 | 不预先承诺沿用当前简化模型，也不混合 release |
| Efficacy pilot | 唯一 final candidate，推荐固定 \(n_{\mathrm{dev}}=4\) complete blocks，无提前停止 | 修改 candidate/gate 即建立新 development release，不反复筛同一批数据 |
| 样本量 | \(n=8\) 为拟议最小值，按 \(\delta_H\)、paired SD、CI/power 和失效率最终冻结 | 若需增加，40/64/88 名称和矩阵全部重算 |
| 确认性统计 | S-MPCC−Smooth-only → S-MPCC−Fixed 顺序 gatekeeping；按真实五臂随机表做 exact inference | 两项均通过才保留“物理有效且优于 fixed timing”主张；sign-flip 仅作敏感性 |
| RQ4 判据 | L1 预冻结等价/非劣与 effect-difference；C2 预冻结物理—任务联合门槛 | 未冻结或精度不足时只作描述性配置结果，不写宽泛 generalization |
| 状态/相位措辞 | model-propagated internal state；`actual`=online solver input | 不声称测得真实 signed phase；标题优先使用 Slosh-Aware 而非无限定 Liquid-State-Conditioned |
| 贡献模板 | 40/64 固定为两项；只有完成条件性 88 才考虑拆出有限配置迁移第三项 | 不用 `C2` 表示 Contribution 2；每个性能 claim 按实际通过的 contrast 单独解锁 |
| C2 | 在首条正式 trial 前预注册为条件性扩展；正文优先级低于 L1 | 若不保留，论文收敛于 64 和 trajectory-generation 主线 |
| K6 | 视觉准入必做；完整 fidelity 降为支持性，按 8/16/24 重建总体 | 需升级 K6-FID-v1.0，旧 32 单元口径失效 |

五条件主线、核心实验 TeX 与 0717 候选现场协议已经对齐。剩余决策冻结后，下一步是完成其余上位协议与代码/工具实现；在对应 development wrapper 和 gate 就绪前不启动签署型 pilot，在唯一 `FREEZE_ID` 生成前不启动 formal trial。
