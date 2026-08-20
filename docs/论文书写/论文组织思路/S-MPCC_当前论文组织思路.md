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

$$
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
$$

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

$$
\text{prescribed geometry}
+
\text{online path progress}
+
\text{model-propagated liquid-state memory}
\rightarrow
\text{finite-horizon progress and executed motion timing}.
$$

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

$$
\dot v^2+(v\omega)^2\le a_R^2
$$

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

## 6. 第三章：Phase-Rejoining Residual S-MPCC Method

### 6.1 本章回答的问题与核心 idea

本章只回答一个问题：离线防晃序列已经安排好“激励—抵消—沉降”后，在线控制器怎样纠正真实底盘的小幅跟踪偏差，又不破坏尚未执行的抵消尾段？

> **一句话核心 idea：在线只做小幅 residual 纠偏；只有当该修正在双通道执行延迟后的预测终端仍满足离线防晃尾段的相位索引经验重接条件，才把第一拍作为候选放行。**

审稿人应能据此明确区分本文与三类近邻方法：

- 与纯 OfflineSloshOCP 的区别：本文允许运行时小幅纠偏；
- 与普通 residual MPC/MPCC 的区别：修正不是“求解成功就执行”，而必须通过经验重接检查；
- 与长时域在线防晃 MPC 的区别：在线不重新预测并优化完整尾段，只优化短 residual，并以离线尾段重接条件收口。

**论文判断：**核心创新是“以尾段重接资格约束在线 residual”，双通道执行前沿对齐是这一机制成立的必要支撑。OfflineSloshOCP 只是名义序列来源；经验 gate 不是 funnel/certificate，固定 stored action 也不是反馈 recovery policy。

当前 B0 的决策依赖尚未闭合，G3 的 held-out gate 证据尚未完成，因此实物 enforce 仍为 **NO-GO**。最终论文 Method 描述修复后的目标算法；当前阻塞项只在实验准入和工程状态中说明。

### 6.2 Problem Setup and Overall Architecture

#### 输入、输出和适用范围

方法输入必须具体写成：

1. trial 前冻结的几何可行路径 \(\mathbf r(s)\)；
2. 与该路径、容器和模型绑定的完整离线 artifact；
3. 带 source timestamp 的机器人—液体状态估计；
4. 经过 limiter 和 safety chain 后的最终发布命令历史。

每个控制周期只输出一个候选底盘命令，随后仍需经过统一发布链。本文只处理静态净空环境中的冻结路径跟踪和小幅偏离，不处理在线障碍检测、碰撞走廊、homotopy 或重规划。MBF 可以在 trial 前生成路径，但必须冻结路径点、坐标系和 hash，并重新生成对应 artifact。

#### 用一个具体例子解释必要性

假设普通 MPCC 为消除横向误差临时增大角速度。Scout 的线速度命令约 \(150\,\mathrm{ms}\) 后生效，
角速度命令约 \(220\,\mathrm{ms}\) 后生效。若从计算时刻直接检查未来 \(100\,\mathrm{ms}\)，
检查结束时角速度修正甚至尚未产生物理作用。因此本文在同一个 delay-augmented OCP 中连续传播穿过两条延迟，
在共同前沿后的终端检查能否接回离线尾段。

方法结构图使用：

[Phase-Rejoining Residual S-MPCC 中文结构图](../../../docs_for_offlineslosh/Methods/assets/figures/phase_rejoining_method_structure_zh.svg)

图中只保留六个主块：离线 artifact、发布时间/邻近相位对齐、双通道 delay-augmented residual OCP、联合终端检查、supervisor、唯一发布链。

### 6.3 Shared Robot–Liquid–Execution Model

#### 三类状态必须分清

用于经验 gate 的显式机器人—液体状态为

$$
\chi=
[p_x,p_y,\psi,v^r,\omega^r,\eta_x,\dot\eta_x,\eta_y,\dot\eta_y]
\in\mathbb R^9.
$$

其中 \(v^r,\omega^r\) 是真实运动状态，四个 \(\eta\) 量是两个正交方向的一阶液体模态。RGB 不进入控制闭环，只在实验中承担独立物理测量。

基础 OCP 状态、真实执行状态和执行增广状态分别为：

$$
X=[p_x,p_y,\psi,v^c,s,\omega^c,z^{\ell\top}]^\top,
\qquad
x^a=[v^r,\omega^r]^\top,
\qquad
X^{\mathrm{aug}}=\operatorname{col}(X,b^v,b^\omega,x^a).
$$

正文必须用一条因果链说明这些量的关系：

$$
q_k
\rightarrow u_k^{\mathrm{sol}}
\rightarrow u_k^{\mathrm{pred}}
\rightarrow (b_k^v,b_k^\omega)
\rightarrow (v_k^r,\omega_k^r)
\rightarrow (\dot v_k^r,v_k^r\omega_k^r)
\rightarrow z_{k+1}^{\ell}.
$$

- \(q_k=[a_k,\alpha_k,v_{s,k}]\) 是 OCP 决策量；
- \(u_k^{\mathrm{sol}}=[v_k^c+a_k\Delta t,\,\omega_k^c+\alpha_k\Delta t]^\top\) 是第 \(k\) 步原始速度命令，只有 \(u_0^{\mathrm{sol}}\) 是当前候选第一拍；
- \(u^{\mathrm{pred}}\) 是 OCP 内按冻结限幅规则得到的假设可发布命令；
- \(u^{\mathrm{pub}}\) 是 supervisor、limiter 和 safety override 后真正发出的命令。

跨周期 FIFO **只能写入 \(u^{\mathrm{pub}}\)**。未执行的 \(u^{\mathrm{sol}}\) 或 \(u^{\mathrm{pred}}\) 不得污染命令历史。yaw 差统一使用 \((-\pi,\pi]\) wrap。

正文保留 contour/lag 定义、低阶液体方程和双通道 FIFO/一阶执行器的紧凑离散方程；容器 Bessel 推导、全部参数和离散化细节移入补充材料。

### 6.4 Complete Offline Artifact

OfflineSloshOCP 输出的不是“到达路径终点即结束”的速度曲线，而是完整的 motion–slowdown–settle–zero-hold 序列：

$$
\bar{\mathcal A}=
\left(
\{\bar X_i^{\mathrm{aug}},\bar\chi_i,\bar t_i\}_{i=0}^{M},
\{\bar q_i,\bar u_i^{\mathrm{pub}}\}_{i=0}^{M-1},
\{\widehat{\mathcal R}^{\mathrm{emp}}_i,
\mathcal B_i^{\mathrm{exec}},
u_{\mathrm{rec}}(i)\}_{i=0}^{M}
\right).
$$

三个逐相位对象的职责不能混写：

| 对象 | 回答的问题 |
| --- | --- |
| \(\widehat{\mathcal R}^{\mathrm{emp}}_i\) | 终端 9 维机器人—液体误差是否落在经验可接受范围内？ |
| \(\mathcal B_i^{\mathrm{exec}}\) | pending command、双 buffer 和执行器状态是否与该尾段兼容？ |
| \(u_{\mathrm{rec}}(i)\) | residual 不可用、但当前经验 gate、执行兼容和合同仍通过时，提交哪个固定候选动作？ |

这些对象由每个相位附近的 recovery rollout 构造；只有完整执行 stored action、重接名义尾段、满足约束并达到预注册沉降条件的样本才标为成功。fit 数据选择半径和动作，held-out 数据只评价，不能再调 gate。

artifact 合同冻结 path/frame hash、时间网格、执行模型、液体模型、容器与装液范围、约束、尾段、gate schema 和软件版本。任一对象改变都必须重建 artifact。

**论文判断：**除非 OfflineSloshOCP 本身形成新的优化算法，否则本节不把“生成一条离线防晃轨迹”单独列为创新。

### 6.5 Execution-Front Alignment and Phase Rejoining

#### 统一时间原点

令 \(t_s\) 为状态 source time，\(t_c\) 为本周期计算开始时刻，\(\widehat d_c\) 为计算到发布的延迟估计，\(t_0\) 为本次 artifact 被准入时的相对时钟原点。先用带时间戳的最终发布历史把状态从 \(t_s\) 传播到预计发布时间：

$$
t_{\mathrm{pub}}=t_c+\widehat d_c,
\qquad
\widetilde\tau_m=t_{\mathrm{pub}}-t_0,
\qquad
\tau_m=\max(\tau_{m-1},\widetilde\tau_m)\quad(m>0),
$$

初始化时令 \(\tau_0=\widetilde\tau_0\)；重新准入另一条 artifact 时才重置。
\(i_{\mathrm{clock},m}\) 由 \(\tau_m\) 在 \(\{\bar t_i\}\) 中的最近时间索引得到。
这个 monotone guard 防止 \(\widehat d_c\) 波动造成时钟后退；\(\tau_m\) 不是优化变量，
也不能通过自由 \(\dot\tau\) 拉伸来伪造重接。实际发布延迟 \(d_c\) 及误差
\(d_c-\widehat d_c\) 必须记录并纳入 G0。

双通道前沿和终端步数为

$$
n_f=\max\!\left(
\left\lceil d_v/\Delta t\right\rceil,
\left\lceil d_\omega/\Delta t\right\rceil
\right),
\qquad
N_e=n_f+N_\ell.
$$

以当前候选值 \(d_v\approx150\,\mathrm{ms}\)、\(d_\omega\approx220\,\mathrm{ms}\)、
\(\Delta t\approx33.3\,\mathrm{ms}\)、\(N_\ell=3\) 为例，共同栅格前沿为
\(7\Delta t\approx233.3\,\mathrm{ms}\)，联合终端约在预计发布时间后 \(333.3\,\mathrm{ms}\)。
所谓“\(100\,\mathrm{ms}\) 短窗”仅指共同前沿之后的三步，不是完整预测 lead。

从最新状态样本到联合终端的完整 lead 为

$$
T_{\mathrm{lead}}=(t_{\mathrm{pub}}-t_s)+N_e\Delta t.
$$

#### 只允许邻近、单调重接

控制周期 \(m\) 的候选相位集合写为

$$
\mathcal J_m=
\left\{
j:
i_{\mathrm{clock},m}-r_-\le j\le i_{\mathrm{clock},m}+r_+,\;
j\ge j_{m-1},\;
0\le j\le M-N_e
\right\}.
$$

在 \(\mathcal J_m\) 内，用 wrap 后的 9 维状态误差和 artifact 时钟误差选择一个 \(j_m\)，终端相位为 \(j_e=j_m+N_e\)。禁止全局跳相位、向后重接和任意时间缩放；候选集为空或误差超界时直接判为不可重接。

### 6.6 Nominal-Relative Residual OCP and Joint Terminal Test

在线 OCP 围绕选定相位 \(j_m\) 优化有限 residual：

$$
\begin{aligned}
\min_{\{X_k^{\mathrm{aug}},q_k\}}\quad
&\sum_{k=0}^{N_e-1}
\left(
J_{\mathrm{track},k}
+\|\xi_k^c-\bar\xi_{j_m+k}^c\|_{R_\xi}^2
+\|q_k-\bar q_{j_m+k}\|_{R_q}^2
+\|z_k^\ell-\bar z_{j_m+k}^\ell\|_{R_\ell}^2
\right)+J_f,\\
\text{s.t.}\quad
&X_0^{\mathrm{aug}}=\widehat X^{\mathrm{aug}}(t_{\mathrm{pub}}),\\
&u_k^{\mathrm{pred}}=\Pi_{\mathrm{cmd}}(u_k^{\mathrm{sol}},b_k^v,b_k^\omega),\\
&X_{k+1}^{\mathrm{aug}}
=F_{\mathrm{exec-\ell}}(X_k^{\mathrm{aug}},u_k^{\mathrm{pred}},v_{s,k}),
\quad k=0,\ldots,N_e-1,\\
&(X_k^{\mathrm{aug}},q_k)\in\mathcal Z,\\
&|s_k-\bar s_{j_m+k}|\le\Delta s_{\max},\qquad
\|u_{0}^{\mathrm{pred}}-\bar u_{j_m}^{\mathrm{pub}}\|_\infty
\le\Delta u_{\max},\\
&e_{N_e|m}^{(9)}
\in\widehat{\mathcal R}^{\mathrm{emp}}_{j_e},\qquad
e_{N_e|m}^{\mathrm{exec}}
\in\mathcal B^{\mathrm{exec}}_{j_e}.
\end{aligned}
$$

其中

$$
e_{N_e|m}^{(9)}=
\mathcal E_\chi(\chi_{N_e|m},\bar\chi_{j_e}),
\qquad
e_{N_e|m}^{\mathrm{exec}}=
\operatorname{col}(b^v-\bar b^v,b^\omega-\bar b^\omega,x^a-\bar x^a)_{N_e|m}.
$$

\(\xi^c=[v^c,\omega^c]\) 是命令积分状态；真实 \(v^r,\omega^r\) 通过增广执行模型进入 tracking 和液体激励。正文只保留这一套紧凑 OCP，不再展开旧稿的自由 virtual progress、长时域 dynamic memory 或多套候选 cost。

两个终端条件必须同时是硬条件。只检查 9 维状态会漏掉“表面状态相同但 pending command history 不同”的情况；只检查执行状态又不能判断液体尾段是否匹配。

关键风险指标是

$$
P(\text{recovery fail}\mid\text{gate accept}),
$$

即 conditional false-safe fraction。零 accept 时该指标未定义，必须同时报告 coverage。没有鲁棒前驱包含证明前，只使用 **phase-indexed empirical recovery gate/set**，不使用 recovery funnel 或 certificate。

### 6.7 Supervisor, Publication, and Algorithm

监督器只保留三条确定性分支：

| 条件 | 提交给统一发布链的候选 |
| --- | --- |
| OCP 成功且联合终端通过 | residual-bounded solver 第一拍 |
| residual OCP 不可用或终端拒绝，但当前 9 维经验 gate、执行兼容、stored action 和合同仍有效 | \(u_{\mathrm{rec}}(j_m)\) |
| 状态过旧、无候选相位、当前 gate 拒绝、执行不兼容或合同失效 | 请求 \((0,0)\)，fail closed |

三类候选都经过同一个 publication gate、limiter 和独立 safety override，最后形成唯一 \(u^{\mathrm{pub}}\)。\((0,0)\) 只是确定性失败语义，不应写成“保证防晃的制动策略”。

Algorithm 1 用六步即可复现：

1. 读取 source-stamped 状态和最终发布历史；
2. 传播至 \(t_{\mathrm{pub}}\)，建立双通道增广初值；
3. 在时钟邻域内选择单调相位 \(j_m\)；
4. 求解 \(N_e\) 步 nominal-relative residual OCP；
5. 检查联合终端并由 supervisor 选候选；
6. 统一发布，记录最终 \(u^{\mathrm{pub}}\) 并写回两个 FIFO。

### 6.8 方法章图表、主张边界与过渡

正文方法章只需要：

1. **Fig. 1：**整体结构图；
2. **Fig. 2：**\(t_s,t_c,t_{\mathrm{pub}}\)、150/220 ms、共同前沿和前沿后约 100 ms 窗口的时间线；
3. **Algorithm 1：**上述六步；
4. **一张符号/合同表：**区分 \(u^{\mathrm{sol}},u^{\mathrm{pred}},u^{\mathrm{pub}}\) 和三类相位对象。

本章不写实验结果、代码类名、旧 40/64/88 计数、Fixed-profile、第二容器、动态障碍或“已证明安全”。章末只提出下一章要验证的三个条件：完整 lead 是否可信、独立 RGB 是否改善、经验 gate 是否以可接受的 false-safe/coverage 工作。

## 7. 第四章：Experimental Evaluation

### 7.1 本章回答的问题与论文判断

实验不再堆叠大量路径、容器和 proxy，而按以下顺序回答四个问题：

| RQ | 核心判断 | 决定性证据 |
| --- | --- | --- |
| RQ1 | 双通道执行模型、完整 lead 和第一拍作用是否可信？ | held-out command–motion–liquid 预测与 G0 |
| RQ2 | 完整方法是否真实防晃，且收益不是明显变慢、跟踪变差或失败增多造成？ | 独立 RGB；C4 vs C0，随后 C4 vs C1 |
| RQ3 | 离线液体目标、有限 residual、联合 recovery 机制和执行模型各自贡献什么？ | A0–A3 严格配对消融 |
| RQ4 | empirical gate 能否减少错误放行，并在失败时按合同降级且实时运行？ | held-out gate、C3 vs C4、runtime 和发布链审计 |

**论文判断只由证据解锁：**独立 RGB 的 C4 vs C0 是唯一主物理比较。内部 slosh monitor、模型状态或仿真方向一致，只能解释机制，不能替代真实液面结果。

### 7.2 Setup, Scope, and Outcomes

#### 场景与冻结对象

主实验固定使用 Scout、一个冻结容器/装液量和一条高激励但静态净空的 P-core 路径。所有 trial 共享 localization、底盘固件、limiter、安全层、起始静置条件和终止规则。

P-MBF 仅作可选扩展：由 MBF 在 trial 前生成一次，随后冻结点列和 hash，并为其单独生成 artifact。运行中不调用 MBF；出现新障碍时终止 trial，不把它写成在线避障。

每个 release 至少冻结 Git commit、路径和 TF、容器与液体参数、执行模型、OfflineSloshOCP artifact、gate、RGB 标定、主时间窗、对照参数和随机区组表。

#### 主测量和统计单位

记独立 RGB 提取的 max-LCR 信号为 \(h_{\mathrm{RGB}}(t)\)。
唯一主物理指标是 motion + tail 窗口内该信号的 trial-level 95% 分位数：

$$
Y_{\mathrm{RGB}} = Q_{0.95}
\left(
\left\{
h_{\mathrm{RGB}}(t)
\;\middle|\;
t\in\mathcal W_{\mathrm{motion+tail}}
\right\}
\right).
$$

motion 起点、到达判定、tail 长度、无效帧和 L/C/R 合成规则必须在 formal 前冻结。Peak、RMS、tail RMS、settling time 为次指标。

任务公平性同时报告完成时间、contour/yaw/endpoint error、success/timeout、零命令、安全事件以及 \(v,\omega,a,\alpha,\mathrm{jerk}\)。一次完整 trial 是统计单位；控制周期和视频帧不是独立样本。方法失败、超时和安全停车保留在分母。

### 7.3 Comparators and Strict Ablations

#### 系统比较组

| 条件 | 具体定义 | 用途 |
| --- | --- | --- |
| C0 OrdinaryMPCC | 普通 contour/lag/progress MPCC；控制器/OCP 内无液体状态与代价，也无离线 artifact 或 recovery 机制 | 唯一主 baseline |
| C1 SmoothMatch | 在独立 development 数据上冻结平滑参数，并用一个全局尺度匹配 C4 完成时间 | 排除“只是更慢/更平滑” |
| C2 OfflineReplay | 按冻结时钟回放与 C4 相同 artifact，\(\delta u=0\)，无 gate/stored action | 观察纯离线序列 |
| C3 ResidualNoGate | 与 C4 冻结相同 artifact、相位规则、residual OCP/边界和安全链，但关闭 gate、执行兼容集及 stored action；solver、候选相位、状态新鲜度或合同失败时确定性请求 \((0,0)\) | recovery 对照 |
| C4 Full | 双通道执行增广、邻近相位、有限 residual、联合终端、stored action 和 fail-closed 全启用 | 完整方法 |

主 nominal matrix 只比较 C0、C1、C2、C4；C3 与 C4 放在受控偏离的 recovery matrix 中。C0–C4 是 comparison family，不要求结果单调，不能用相邻条件差值冒充组件因果结论。

#### 严格单因素消融

| 消融 | 唯一变化 | 可支持的结论 |
| --- | --- | --- |
| A0 OfflineSmoothReplay vs C2 | 离线液体目标关闭/开启 | 离线相位安排的价值 |
| A1 PhaseAlignedNoResidual vs C3 | finite residual 关闭/开启 | residual 的独立价值 |
| A2 C3 vs C4 | gate＋执行兼容集＋stored action 联合关闭/开启 | recovery 联合机制的价值；不拆分三项 |
| A3 IdealExec vs IdentifiedExec | 瞬时单位增益模型 \((d_v=d_\omega=\tau_v=\tau_\omega=0)\) / 已辨识双通道延迟—惯性模型，并各自重建一致 artifact | 显式建模非理想执行链的价值 |

每个消融先做 manipulation check：若最终 \(u^{\mathrm{pub}}\)、buffer 预测或 residual 没有可检测差异，就不能对物理结果作因果解释。

### 7.4 Pre-release Validation

formal 采集前必须逐项通过：

| Gate | 必须回答的具体问题 | 当前状态 |
| --- | --- | --- |
| G0 Execution | 修复 B0（history-only 前沿漏掉候选命令在差分延迟中的作用，且混用约 0.22 s 物理前沿与 \(7\Delta t\) 索引）；完整 lead \((t_{\mathrm{pub}}-t_s)+N_e\Delta t\) 是否准确（当前栅格终端在预计发布时间后约 333.3 ms）？第一拍是否有可检测作用？延迟和液体模态频率附近的 processed-IMU 幅相是否跨工况稳定？ | **NO-GO** |
| G1 Artifact | P-core artifact 是否包含 slowdown–settle–zero-hold，满足状态/命令约束且合同 hash 完整？ | Pending |
| G2 RGB | source time、motion + tail 有效率、重复标定、噪声和最小可检测差异是否合格？ | Pending |
| G3 Recovery | fit/test 是否按完整 trial 隔离？执行兼容、coverage、conditional false-safe、false reject、最差相位和真实重接是否达阈值？ | **NO-GO** |
| G4 Pilot | 冻结小样本中命令确实分离、RGB 方向可审计、tracking 不崩、所有分支和 recorder 完整？ | Pending |

任何 gate 失败都不能靠放宽 gate、增大 residual、删除失败 trial 或追加 formal 样本绕过。G0–G4 未全部通过前，只能报告 development 结果，不能写“防晃性能已经改善”。

### 7.5 Formal Experimental Sequence

正式实验按依赖顺序执行：

1. **模型/测量验证：**在 held-out trial 和冻结 pilot 上完成 G0–G4；
2. **P-core nominal blocks：**每个随机区组包含 C0、C1、C2、C4，比较正常跟踪；
3. **受控偏离 blocks：**配对比较 C3、C4；偏离只用小姿态偏置、人工附加命令延迟/限幅或可重复短速度门；
4. **A0–A3：**只执行预冻结的严格配对消融；
5. **P-MBF 可选扩展：**仅在 P-core 主结论通过后运行，仍是冻结路径，不引入动态障碍。

fit、tune、pilot、main confirmation 和 recovery confirmation 按完整 trial/block 分离。任何调参都会产生新 release，旧 confirmation 数据不得并入。

### 7.6 Main Physical Result and Decision Rule

对随机区组 \(b\)，主效应定义为

$$
\Delta_b^{\mathrm{C4-C0}}=Y_{b,\mathrm{C4}}-Y_{b,\mathrm{C0}}.
$$

数值越低越好。正文同时给 raw paired points、稳健中心效应、区间、失败分母和 leave-one-block-out 结果，不只给均值柱状图。

只有同时满足以下条件，论文才能写“相对 C0 OrdinaryMPCC 防晃改善”：

1. C4 vs C0 的 RGB 效应达到预注册 SESOI，且冻结的区间判据通过（例如 \(\Delta^{\mathrm{C4-C0}}\) 的上置信界低于 \(-\delta_{\mathrm{RGB}}\)）；
2. 完成时间和 tracking 落在非劣界内；
3. failure、安全事件和 fail-closed 触发没有超出容限。

第一项主比较通过后，才按固定顺序检验 C4 vs C1。若 RGB 更低但机器人明显更慢、跟踪更差或失败更多，只能报告 trade-off；若只有内部 slosh 量降低而 RGB 不支持，不能宣称物理防晃改善。

### 7.7 Mechanism, Recovery, and Runtime

机制结果按 A0–A3 报告，每项同时展示中间变量和最终发布命令，避免“消融开关变了但执行没变”。

recovery 结果在独立数据上先筛选 \(\mathcal B_i^{\mathrm{exec}}\)，再报告：

$$
r_{\mathrm{FS|A}}=P(\text{recovery fail}\mid\text{gate accept}).
$$

以及该比例的区间/上置信界、coverage、false reject、最差相位、实际重接时间、tail 结果和 stored action 的“提出—通过 limiter/safety—最终发布”比例。G3 依据预冻结的上置信界与 coverage 双门槛判定；零 accept 时 \(r_{\mathrm{FS|A}}\) 记为未定义，不能记成 0。

runtime 与命令完整性至少报告 solver p50/p95/max、超过 \(33.3\,\mathrm{ms}\) 的比例、实际发布频率、solver/stale/contract failure、各 supervisor 分支、limiter/safety 改写率，并核对只有最终 \(u^{\mathrm{pub}}\) 写入双 buffer。

### 7.8 实验章图表、删减项与过渡

正文优先保留：

1. C4 vs C0/C1 的 RGB paired effect 与代表性 motion + tail 曲线；
2. A0–A3 紧凑效应图；
3. gate 的 false-safe/coverage、实际重接和 C3 vs C4；
4. 一张汇总表报告任务、失败和 runtime；
5. 一张 G0–G4 状态表。

删除旧 40/64/88 固定计数、Fixed-profile 主线、H1/L1、第二容器、四相位 actual/zero、K6 和长时传播审计。它们不能挤占 RA-L 正文，也不能在主结果失败时补叙事。

章末只保留四个结论接口：执行模型是否可信、RGB 是否正向、收益是否超越减速解释、gate 是否以可接受错误率和 coverage 工作。下一章只能总结这些实际通过的判断。

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
