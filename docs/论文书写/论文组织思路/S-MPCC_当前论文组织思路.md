# PR-RMPC 当前论文组织思路

- 更新日期：2026-08-24
- 当前方法名：**Phase-Rejoining Residual MPC（PR-RMPC）**
- 当前论文题目：**Execution-Aware Phase-Rejoining Residual MPC for Slosh-Sensitive Mobile Transport**
- 正式仿真：**0/96**
- 正式实物：**未开始**

> 文件名中的 “S-MPCC” 是历史遗留名称，不再代表当前论文方法。本文是论文级组织总纲；方法细节以 [Methods 章节组织思路](../../../docs_for_offlineslosh/Methods/Methods章节组织思路.md) 和 [论文 Methods](../草稿/spmpc_paper_core/sections/01_method.tex) 为准，实验定义以 [论文 Experiments](../草稿/spmpc_paper_core/sections/02_experiments.tex)、[仿真实验方案](../../../docs_for_offlineslosh/仿真实验/仿真实验方案.md) 和 [仿真到实物验证路线](../../../docs_for_offlineslosh/思路/防晃论文的仿真到实物验证思路.md) 为准。

> 当前方法、六条件、失败语义和分析规则已经闭合，但新的正式 release 尚未绑定最终 clean HEAD。只有该 HEAD 的 clean build、session/freeze/readiness 和 session-specific 人工 approval 全部通过，才能启动 seeds 3101--3116。readiness 和 approval 只是执行许可，不是防晃效果证据。

## 1. 论文核心定位

### 1.1 问题

移动机器人搬运开口液体时，可以先沿冻结几何路径生成一条包含激励、抵消、减速和沉降尾段的离线防晃序列。然而，真实执行中的定位偏差、跟踪修正以及线速度和角速度不同的命令延迟，可能破坏尚未执行的抵消关系。

本文研究的问题是：

> **如何在保留离线防晃尾段的前提下进行有限在线纠偏，并让纠偏命令在预计发布时间、不等执行延迟和最终发布事务下仍与一个邻近名义相位兼容？**

### 1.2 方法的一句话定义

PR-RMPC 先把 source-time 状态和最终发布命令历史传播到预计发布时间，在时钟邻域内选择一个不后退的兼容名义相位，再求解短时域、名义相对的 delay-augmented residual MPC；只有满足全预测域执行兼容和经验恢复准入的候选，才允许进入统一发布链。

在线层是 **residual trajectory-tracking MPC**，不是 contour/lag MPCC。普通 MPCC 只出现在 C0/C1 基线中。离线层是参数化路径速度剖面搜索，不主张任意状态—控制 direct transcription 或全局最优。

### 1.3 核心论证链

论文按以下依赖关系展开：

1. 离线序列提供完整 motion--slowdown--settle--hold 名义尾段；
2. 22 维模型显式传播线/角 pending-command 的不等纯延迟；
3. publication-epoch phase rejoining 只在邻近、单调、执行兼容的相位中选择；
4. residual OCP 在完整十阶段预测域内满足发布命令、rate、residual 和执行集合约束；
5. 9 维 empirical gate 约束 C4 的 terminal residual admission 和 current recovery admission；
6. supervisor 在 audited residual、有界 recovery feedback 和 fail-closed zero request 之间确定性选择；
7. 独立 Plant 或真实 RGB 承担效果评价，控制器内部液体状态不替代外部测量；
8. 正式比较再判断总体效果、公平性解释和 gate-enforcement 作用。

### 1.4 主张边界

当前可以主张：

- PR-RMPC 的方法公式、22 维因果执行模型、经验准入对象和最终发布事务已经实现；
- C3/C4 除 9 维 empirical gate 的 monitor/enforce 外保持相同；
- Layer S 的六条件、16 个 seed、失败处理和统计规则已经确定。

当前不能主张：

- C4 已在独立 Plant 上降低液面；
- 仿真结论已经迁移到 Scout 或真实液体；
- empirical gate 是 funnel、recovery certificate 或安全证书；
- 方法具有递归可行性、任意扰动恢复、在线避障或 spill-free 保证；
- 当前单路径结果可以外推到任意路径、容器或装液量。

## 2. 全文结构

| 章节 | 标题 | 职责 |
| --- | --- | --- |
| I | Introduction | 建立离线抵消尾段与在线纠偏之间的冲突，概述方法和证据边界 |
| II | Related Work | 定位离线防晃序列、在线预测控制和执行延迟处理之间的缺口 |
| III | Phase-Rejoining Residual MPC Method | 给出共享模型、离线 artifact、phase rejoining、residual OCP、supervisor 和发布事务 |
| IV | Experimental Evaluation | 先组织 Layer S 正式仿真，再规划 Layer H Scout/真实液体验证 |
| V | Conclusion and Limitations | 只总结正式证据支持的结论，并限定外推范围 |

全文逻辑为：

> 离线序列为什么会被在线纠偏破坏 $\rightarrow$ 如何在真实发布时间和不等延迟下重接相位 $\rightarrow$ 如何限制 residual 并经验判断可恢复性 $\rightarrow$ 独立 Plant 是否支持总体效果 $\rightarrow$ 效果是否超出一般慢/平滑解释 $\rightarrow$ gate enforcement 是否确有可见作用 $\rightarrow$ 是否值得进入真实 Scout 与液体验证。

## 3. Introduction 组织

Introduction 建议按五段组织。

### 3.1 任务与困难

说明开口液体运输同时要求路径跟踪、任务效率和液面抑制。几何路径可以预先给定，但控制器仍需处理执行偏差；直接在线修正可能打乱离线序列的抵消尾段。

### 3.2 现有路线与缺口

简述两条已有路线：

- input shaping、离线 retiming 或离线 trajectory optimization 生成固定防晃 timing；
- MPC/MPCC 在线处理跟踪误差和运动约束。

缺口不是“从未有人做过防晃 MPC”，而是：离线序列、在线残差修正、不等命令延迟和尾段重接通常没有形成一个因果闭合的执行合同。

### 3.3 方法概述

用一段话说明：

- 共享的机器人—液体—pending-command 模型；
- publication-epoch 邻近单调 phase rejoining；
- 受全预测域执行集合与 empirical gate 约束的 residual MPC；
- residual/recovery/zero 三分支和唯一最终发布事务。

### 3.4 贡献写法

正式结果出来前，贡献只写成方法和验证设计，不提前写性能结论：

1. 一个显式处理线/角不等发布延迟的 phase-rejoining residual MPC；
2. 一组区分 9 维 empirical recovery gate、14 维 execution set 和 bounded recovery feedback 的经验恢复对象及监督逻辑；
3. 一套先用 controller-blind independent Plant 检验总体效果和 gate enforcement、再迁移到 Scout/真实液体的分层验证。

正式数据完成后，第三项必须改写成实际获得的证据，不能把“设计了实验”本身当作结果性贡献。

### 3.5 范围声明

Introduction 末尾明确：路径在运动前冻结，环境静态且净空；本文不处理动态障碍、在线重规划、homotopy 或任意 time scaling。

## 4. Related Work 组织

Related Work 保留三组直接相关工作：

1. **Open-liquid transport and offline anti-slosh planning**：input shaping、速度剖面优化和离线轨迹生成；
2. **Online MPC/MPCC for mobile robots**：路径跟踪、contouring control、运动约束和在线修正；
3. **Delay-aware and recovery-aware predictive control**：执行延迟、命令历史、terminal/recovery admission 和经验可恢复域。

本章最后只做精确定位：

- 与纯 OfflineReplay 不同，PR-RMPC 允许运行时有限 residual 纠偏；
- 与普通 residual MPC 不同，求解成功不是发布的充分条件，还要通过执行兼容、经验准入和最终事务；
- 与长时域在线防晃重规划不同，在线层不重新优化完整尾段，而是围绕冻结相位做短时域 residual；
- empirical gate 和 recovery feedback 都是冻结数据合同下的经验对象，不是形式安全结论。

不要把本文实现的 ZVD、普通 MPCC 或自建 offline planner 写成对某篇外部工作的逐项忠实复现。

## 5. Methods 组织

### 5.1 Problem, Architecture, and Scope

先定义冻结路径、离线 artifact、source-stamped 状态、最终 published-command history 和每周期候选输出。结构图只保留：

    parameterised offline plan
      → recovery objects and V3 artifact
      → publication-epoch alignment
      → neighboring monotone phase
      → 22-D residual OCP and admission
      → residual / recovery / zero supervisor
      → safety, arbitration, finalizer, sink and receipt

### 5.2 Shared Delay-Augmented Robot--Liquid Model

经验 gate 状态为

$$
\chi=[p_x,p_y,\psi,v^r,\omega^r,\eta_x,\dot\eta_x,\eta_y,\dot\eta_y]^\top
\in\mathbb R^9.
$$

在线优化状态为

$$
X^{\mathrm{aug}}
=\operatorname{col}(p_x,p_y,\psi,v^r,s,\omega^r,z^\ell,b^v,b^\omega)
\in\mathbb R^{22},
$$

其中 $b^v\in\mathbb R^5$、$b^\omega\in\mathbb R^7$ 是从旧到新的 pending final-published-command queues；执行兼容子状态

$$
x^{\mathrm{exec}}=\operatorname{col}(v^r,\omega^r,b^v,b^\omega)
\in\mathbb R^{14}.
$$

输入为

$$
q=[a^{\mathrm{pub}},\alpha^{\mathrm{pub}},v_s]^\top.
$$

前两项描述待发布命令的变化率，不是瞬时物理加速度。候选命令从 pending queue 最新端点积分。当前 controller contract 使用 $d_v=150\,\mathrm{ms}$、$d_\omega=220\,\mathrm{ms}$、单位增益、零 deadzone 和零 actuator time constant。ROS/transport receipt 不能写成 Scout/CAN 物理执行确认。

### 5.3 Parameterised Offline Artifact

离线层用 $K=8$ 个路径索引 knot 参数化分段线性速度尺度，经确定性 delayed path-following rollout 生成角速度命令、传播共享模型、执行减速，并追加固定 $4\,\mathrm{s}$ 零命令 hold。

逐相位 artifact 紧凑写为

$$
\bar{\mathcal A}
=\left\{
(\bar t_i,\bar X_i^{\mathrm{aug}},\bar q_i,
\bar u_i^{\mathrm{pub}},\bar\kappa_i,\rho_i,\beta_i)
\right\}_{i=0}^{M}.
$$

三个恢复对象必须分开：

| 对象 | 作用 |
| --- | --- |
| $\widehat{\mathcal R}^{\mathrm{emp}}_i$ | 9 维 phase-indexed diagonal ellipsoid，判断机器人—液体误差是否落入经验接受域 |
| $\mathcal B_i^{\mathrm{exec}}$ | 14 维 phase-indexed box，检查实现速度与 pending queues 是否和相位兼容 |
| $\kappa_i(e_r)$ | 冻结、有界的 pose/velocity recovery feedback，在 recovery 获准时生成候选 |

recovery rollout 使用相同反馈推进完整尾段，由 controller 不可见的独立 Plant 根据路径、液面、终端位姿/速度和沉降阈值标注。fit、tune、held-out 按完整 rollout 和 seed 隔离；held-out 只评价，不能回调 gate 或 feedback。

### 5.4 Publication-Epoch Alignment and Phase Rejoining

状态先从共同 source epoch $t_s$ 对齐到预计发布时间

$$
t=t_c+\widehat d_c.
$$

传播只使用带时间戳的最终发布命令历史和计算期间已知保持命令；随后在对齐后的机器人位置上重新做 monotone guarded path projection。

当前预测域为

$$
n_f=\max\!\left(
\left\lceil d_v/\Delta t\right\rceil,
\left\lceil d_\omega/\Delta t\right\rceil
\right)=7,
\qquad
N_e=n_f+N_\ell=10,
$$

其中 $N_\ell=3$。完整 source-to-terminal lead 为

$$
T_{\mathrm{lead}}=(t-t_s)+N_e\Delta t.
$$

“共同前沿后约 $100\,\mathrm{ms}$”只指最后三步联合响应窗，不是总预测 lead。

候选相位必须在 clock 邻域内、不早于已提交相位、最多领先 clock 一个栅格、为完整预测域保留尾段，并通过 current execution compatibility 和 full-horizon causal witness。相位不是自由优化变量，不允许全局搜索、后跳或任意时间缩放。

### 5.5 Residual OCP and C3/C4 Factor

C4 围绕选定相位求解十阶段 22 维 Full SQP。代价是 nominal-relative trajectory tracking，不使用 contour/lag MPCC 代价。硬条件包括：

- 共享不等延迟机器人—液体动力学；
- published-command envelope、rate 和 phase-relative residual bounds；
- $k=0,\ldots,N_e$ 全预测域的 $\mathcal B^{\mathrm{exec}}$；
- C4 terminal 9 维 empirical gate；
- KKT、独立 constraint audit 和 causal rollout。

C3 与 C4 的唯一实验因素是同一 9 维 empirical gate 的 monitor/enforce：

| 对象 | C3 GateMonitorPR-RMPC | C4 PhaseRejoinResidual |
| --- | --- | --- |
| 22 维 Full SQP、phase selector、V3 | 相同 | 相同 |
| residual authority 和全时域 $\mathcal B^{\mathrm{exec}}$ | enforce | enforce |
| bounded recovery feedback 和最终发布链 | 相同 | 相同 |
| terminal empirical metric | 计算记录，不拒绝 | NLP 与 post-solve 均 enforce |
| current empirical metric | 计算记录，不拒绝 recovery | recovery admission enforce |

工程中的 residual_no_gate 只是 legacy mode 名，不能据此把 C3 写成关闭 execution set 或 recovery。

### 5.6 Supervisor and Publication Transaction

监督器区分：

| 情况 | 请求 |
| --- | --- |
| residual solve、admission 和独立 audits 全部通过 | causal rollout 的 audited residual 第一拍 |
| recovery-eligible optimization failure，或成功求解后 terminal admission 被拒；current recovery 条件通过 | rate-reachable 的 $\kappa_j(e_r)$ |
| state/history/artifact/contract/integrity failure，或 residual/recovery 均不可用 | fail-closed $(0,0)$ |

zero request 是失败关闭语义，不是已验证的防晃制动策略。只有数值优化确实执行后产生的 recovery-eligible optimization failure 才能进入 recovery：求解器状态码、收敛或 KKT 验收失败可以归入这一类；pre-solve 合同/参数图像/warm-start 因果性错误，以及 post-solve 独立 constraint 或 causal audit 错误属于 integrity failure，必须直接 fail closed。

所有分支共用：

$$
u^{\mathrm{cand}}
\rightarrow\mathcal S_{\mathrm{safety}}
\rightarrow\mathcal A_{\mathrm{priority}}
\rightarrow\mathcal G_{\mathrm{exec}}
\rightarrow u^{\mathrm{final}}
\rightarrow\mathrm{sink/receipt}.
$$

有限、delivered 的最终 receipt 更新 pending-command history。**相位提交条件更严格：只有 audited residual 分支、没有 recovery/controlled stop/zero 或任何 command intervention、最终命令未被 safety/arbitration/finalizer/sink 改写，并且 receipt 与最终命令及时间一致时，才允许提交 proposed phase。** recovery、controlled stop、zero request、任意改写或 receipt 不一致均不得提交相位。

## 6. Experimental Evaluation 组织

### 6.1 两条主线

    共用方法与工程准备
      → 主线一：Layer S 正式仿真，先建立可独立成稿的仿真保底
      → 主线二：Layer H Scout 与真实液体验证，用于提升证据层级

Layer S 与 Layer H 不得混写：

- Layer S 使用 controller-blind independent Plant；液面真值只给 evaluator；
- Layer H 使用独立标定 RGB 作为真实液面主测量；
- 仿真 release 不证明实物参数有效，实物必须重新辨识和冻结；
- 只有 Layer S 的 C4--C0 primary 受支持，才形成“仿真有效”的论文保底并进入实物效果主线；
- 实物失败时可以保留仿真稿，但不能声称真实液面降低。

### 6.2 Research Questions

| RQ | 问题 | 证据 |
| --- | --- | --- |
| RQ1 | 实现是否因果传播不等 pending-command 延迟并保持命令历史完整？ | solver/KKT、constraint/causal audit、source/publication timestamp 和 runtime telemetry |
| RQ2 | C4 是否相对普通 MPCC 降低独立 Plant 液面且任务不劣？ | C4--C0 唯一 primary；通过后再解释 C4--C1 |
| RQ3 | enforce empirical gate 是否有增量作用？ | C4--C3 paired effect、activation、recovery、zero request 和失败 |
| RQ4 | gate 是否能拒绝独立标注的不可恢复状态且不以零 coverage 隐藏失败？ | held-out confusion matrix、denominator-specific Wilson intervals 和 coverage |
| RQ5 | 仿真结论能否迁移到 Scout 和真实液体？ | 硬件执行辨识、shadow mode 和独立 RGB 正式结果 |

### 6.3 Layer S Frozen Matrix

Layer S 冻结同一 P2 路径、P2/V3/recovery assets、独立 Plant、起始条件、测量窗、终止规则、六个 condition configs、seeds 3101--3116、96 行交错顺序和 analyzer。

六条件定义为：

| 条件 | 定义 | 论文角色 |
| --- | --- | --- |
| C0 OrdinaryMPCC | 普通 contour/lag/progress MPCC；无液体预测、offline artifact、phase rejoining 或 recovery admission | 总体效果主基线 |
| C1 SmoothMPCC | C0 加 task-only development 冻结的 smoothing，global_time_scale=1.0 | 削弱一般慢/平滑解释；不是精确匹配 |
| C2 OfflineReplay | 按时钟回放 C4 的同一 final-command nominal；online residual 为零，无 empirical/execution-gate 干预 | 描述性离线参考 |
| C3 GateMonitorPR-RMPC | 与 C4 共用 22 维 solver、phase、V3、$\mathcal B^{\mathrm{exec}}$、recovery、失败语义和发布链；9 维 metric 只监测 | gate-enforcement 严格配对 |
| C4 PhaseRejoinResidual | 完整 PR-RMPC | 提出方法 |
| IS ZVD | 冻结的单模态 ZVD，使用普通在线跟踪栈 | 描述性外部方法参考 |

16 个 seed 与六条件全交叉：

$$
16\ \mathrm{blocks}\times6\ \mathrm{conditions}=96\ \mathrm{trials}.
$$

统计单位是完整 trial/seed block，不是 MPC cycle、Plant integration step 或视频帧。

### 6.4 Comparison Hierarchy

| 层级 | 对比 | 解释 |
| --- | --- | --- |
| P0 | C4--C0 | 唯一 preregistered primary，检验完整方法的总体效果，不做单组件归因 |
| P1 | C4--C1 | P0 通过后的 fixed-sequence fairness comparison；正向只能削弱慢/平滑替代解释，失败不撤销 P0 |
| P2 | C4--C3 | empirical-gate enforcement 单因素对比；结合 activation/recovery/zero/failure 解释，不使用 0.5 mm hard gate，不 veto P0 |
| D0 | C2、IS | **只报告各条件 trial-level 分布和失败；不承诺 C4--C2 或 C4--IS paired effect/CI，不进入 hard decision，也不作组件归因** |

当前矩阵只能严格识别 C3/C4 的 gate-enforcement 因素。C0--C4 的编号或结果单调性不能冒充逐组件消融。

### 6.5 Outcome, Statistics, and Failure Semantics

Layer S 主指标为独立 Plant 液面高度在 motion plus fixed $4\,\mathrm{s}$ tail 完整窗口内的 trial-level P95：

$$
Y^{\mathrm{sim}}_{b,c}
=\operatorname{P95}_{t\in\mathcal W_{\mathrm{motion+4s}}}
h^{\mathrm{plant}}_{b,c}(t).
$$

对已声明的核心 comparator $c$，

$$
\Delta_b^{\mathrm{C4}-c}
=Y^{\mathrm{sim}}_{b,\mathrm{C4}}-Y^{\mathrm{sim}}_{b,c},
$$

数值越低越好。报告 paired mean 和 10,000 次确定性 paired-bootstrap 95% CI。

C4--C0 只有同时满足以下条件才支持 primary：

1. 液面效应 CI 上端不高于 $-0.5\,\mathrm{mm}$；
2. C4 对 C0 的完成时间相对差 95% 上界不高于 10%；
3. C4 对 C0 的 tracking P95 差 95% 上界不高于 $0.05\,\mathrm{m}$；
4. C4/C0 为零 method-failed pairs。

C4--C1 只在 P0 通过后按冻结顺序检验，使用同一冻结液面 margin 及 C1-specific task/failure gate。正向只能说收益相对 independently tuned SmoothMPCC 仍存在，不能声称排除了所有速度/平滑解释。C4--C3 报告 effect/CI、gate activation、recovery、zero request 和失败；activation 稀少或为零时，机制识别力也必须写成稀少或不存在。

method failure 包括：

- task/runtime failure；
- solver failure；
- controlled stop；
- publication/receipt/history failure；
- fail-closed zero request；
- C3/C4 缺失或失败的 KKT audit。

失败门按 comparison 分开：C4/C0 失败作用于 P0，C1 失败作用于 P1，C3 失败作用于 P2；C2/IS 失败只保留在各自描述性结果中，不能否决 P0。

失败 trial、停车、超时和未完成 trial 全部保留。失败 trial 若完成了 motion-plus-tail 窗口，仍使用其数值；若 runtime failure 使完整窗口不存在，则：

- trial 和对应 pair 仍保留；
- 不删除、不插补；
- 涉及该 pair 的 effect/CI 标为 **NOT_ESTIMABLE**；
- 对应 claim 判定为失败。

合法记录为 RUNTIME_ERROR 或 TRIAL_COMPLETE_WITH_FAILURE 的正式 trial，即使进程非零退出，也作为方法/任务结果保留并继续 campaign。只有进程无法启动，或 formal summary/cycle 文件缺失、不可读等冻结定义的基础设施故障，才停止执行并保留处置记录；不得自动补跑到成功。

16 个 blocks 和 0.5 mm margin 尚无充分的 prospective power/precision 论证，必须作为局限报告，不能在结果出来后包装成既定统计功效。

正式实验开始后不得依据任何中间或最终液面结果修改 C1、C4、gate、V3、Plant、路径、seed、顺序、阈值或 analyzer。

### 6.6 Gate and Runtime Diagnostics

held-out gate 至少报告：

- 未恢复样本中的 false-accept rate $\mathrm{FA}/(\mathrm{FA}+\mathrm{TR})$；
- 被接受样本中的 conditional false-safe $\mathrm{FA}/(\mathrm{TA}+\mathrm{FA})$；
- recovered-sample coverage $\mathrm{TA}/(\mathrm{TA}+\mathrm{FR})$；
- 各自正确分母下的 Wilson interval；
- zero accept 时 conditional false-safe 为 undefined，而不是 0；
- per-phase/worst-phase、actual rejoining、execution-set rejection 和 zero request。

runtime 与命令完整性至少报告 solve-time p50/p95/max、超过 $33.3\,\mathrm{ms}$ 的比例、Full SQP iterations/KKT、constraint/causal audit、solver/recovery/zero 分支、controlled stop、residual saturation、command rewrite、receipt/history 和 phase commit。

### 6.7 Layer H Planned Validation

Layer H 在 Layer S 结果冻结解释后使用独立 hardware release：

1. 重新辨识 Scout 线/角 command-to-motion delay、gain、惯性、deadzone 和 saturation；
2. 区分 ROS/transport receipt 与 Scout/CAN 真实执行；
3. 冻结 final command、Scout feedback、odometry、Nokov、IMU 和 RGB 的 source-time 对齐；
4. 验证 RGB max-LCR 标定、有效帧率、漂移和最小可检测差异；
5. 重新生成实物专用 offline artifact、execution set、gate/recovery 和安全边界；
6. 依次完成 no-motion、shadow mode、低风险 development pilot 和单独批准的 randomized physical confirmation。

实物主指标是 motion-plus-tail 窗口内独立 RGB max-LCR 的 trial-level P95。实物仍以 C4--C0 为 primary，之后才解释 C4--C1；C4--C3 只有在安全、预注册且确有 activation 的范围内解释。仿真与实物不得共用数值 Plant、执行参数、gate/recovery assets 或正式数据。

## 7. Results、Conclusion 与写作解锁

当前 Layer S 为 **0/96**，Layer H 未开始，因此结果章只能展示实验设计和 Pending 状态，不能写正向防晃结果。

正式仿真后按以下规则解锁措辞：

| 结果 | 可以写 | 不可以写 |
| --- | --- | --- |
| C4--C0 通过 | 在冻结路径、独立 planar Plant、当前样本与精度边界内，C4 相对 OrdinaryMPCC 降低液面且任务非劣 | 真实液体有效、任意路径有效或单一组件已归因 |
| C4--C1 也通过 | 收益相对 independently frozen SmoothMPCC 仍存在，削弱一般慢/平滑解释 | 精确等时/等平滑或所有替代解释已排除 |
| C4--C1 未通过 | 保留已支持的 C4--C0，同时报告 fairness trade-off | 相对 SmoothMPCC 仍有收益 |
| C4--C3 有 activation 和效果 | 在当前合同和介入样本中，empirical-gate enforcement 与差异一致 | gate 是安全证书或整个 recovery 机制已被独立归因 |
| C4--C3 很少/无 activation | 报告机制证据稀少或不可识别 | 为制造 activation 修改路径、gate 或参数 |
| C4--C0 未通过或不可估计 | 如实报告 primary 负向、不确定或失败 | 换 seed、删 trial、调参、追加条件来改写 |
| 实物未通过 | 保留受支持的仿真论文保底 | 声称真实液面降低 |

Conclusion 只总结实际通过的层级，并再次限制到冻结路径、静态净空环境、合同内模型与样本精度。

## 8. 正式实验前的最小冻结边界

正式仿真前只完成下列必要手续：

1. 将所有并发修改明确处理并形成最终 clean HEAD；
2. 对该 HEAD 做一次 current/clean build；
3. 生成绑定该 HEAD 的 formal session；
4. readiness 核对 P2/V3、Plant、六条件、seeds 3101--3116、96 行顺序、指标、失败和 analyzer 规则；
5. readiness 必须是 READY_NOT_EXECUTED、formal_trials_started=false 且 reasons 为空；
6. reviewer 对该 session 签发 session-specific approval；
7. 仅由正式 runner 按冻结顺序运行 96 trials。

任一方法、配置或分析对象变化都会使旧 session/readiness/approval 失效。正式数据出现后不再调参。

以下内容明确不阻塞当前 96-trial 主矩阵，也不应在开跑前继续扩张：

- A0/A1/A3 扩展消融；
- 第二路径、第二容器、新 seed replication；
- 大规模 delay/frequency/damping robustness scan；
- DualSPHysics/CFD；
- 动态障碍和在线重规划；
- 逐文件 hash 层、复杂 claim 状态树、manifest 事务、深层配置等价和大量 mutation tests。

## 9. 术语与禁用叙事

| 使用 | 不再使用 |
| --- | --- |
| Phase-Rejoining Residual MPC / PR-RMPC | S-MPCC 作为当前论文方法名 |
| residual trajectory-tracking MPC | 把 C4 写成 contour/lag MPCC |
| parameterised offline path-speed-profile search | 无限定的 OfflineSloshOCP 全局最优轨迹 |
| 9-D empirical recovery gate | recovery funnel、certificate、安全集 |
| 14-D execution compatibility set | 把 pending-command 历史并入 9-D gate 后不加区分 |
| frozen bounded recovery feedback $\kappa_i(e_r)$ | fixed stored recovery action |
| ROS/transport publication receipt | Scout/CAN actuator acknowledgement |
| controller-blind independent Plant | CFD 真值或真实 Scout 参数 |
| C1 SmoothMPCC | exact time/smoothness matched baseline |
| C3 GateMonitorPR-RMPC | 把 residual_no_gate 解读为关闭 execution set/recovery |
| C2/IS condition-level descriptive references | C4--C2/C4--IS hard decision 或配对效应承诺 |

## 10. 当前最终叙事

整篇论文应让读者得到以下认识：

> PR-RMPC 不是重新在线规划完整防晃轨迹，而是在预计发布时间对齐机器人、内部液体和不等延迟 pending-command 状态，从冻结离线尾段中选择邻近单调相位，并求解受执行兼容和经验恢复准入约束的有限 residual。独立 Plant 首先检验完整方法相对普通和平滑 MPCC 的总体效果及 empirical-gate enforcement；真实 Scout 与液体使用独立标定和 release 作为第二条验证主线。仿真可以在实物失败时形成有限范围的论文保底，但不能替代真实液面证据。

截至 2026-08-24，方法与正式实验合同已经闭合，正式结果仍为 **0/96**。下一步不是继续扩建实验平台，而是完成最终 clean HEAD 的 build/freeze/readiness 和人工 approval，然后按冻结顺序执行正式主矩阵。
