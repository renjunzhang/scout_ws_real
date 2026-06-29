# SPMPC 论文草稿写作框架

日期：2026-06-30  
目标：在本目录中逐步形成 SPMPC 论文草稿。本文档固定写作原则、故事线、Related Work 分类、claim 链条、章节结构和草稿文件职责。

---

## 0. 总原则：Story first, experiments as evidence

这篇论文不能写成实验排行榜，也不能写成“我实现了一个 MPC，所以做了一堆对比实验”。

核心原则是：

> **先确定论文要讲的故事，再决定需要哪些实验。实验必须严谨，但实验不是目的，而是证据；baseline 不是排行榜，而是 gap 的证据。**

也就是说：

```text
不是：我做了很多实验 → 所以论文成立
而是：我有一个清楚的问题、gap 和 claim → 所以这些实验是必要证据
```

这里的“故事”不是文学化表达，而是**论文的论证叙事**：

```text
为什么这个问题值得研究？
现有方法为什么不够？
为什么我选择这个模型/方法？
为什么不用另一个更常见的方法？
为什么这些对比能证明我的 claim？
读者最后应该相信什么？
```

每个方法细节都要回答：

```text
Why this?
Why not that?
What does it prove?
```

每个实验也都必须回答三个问题：

1. 它支持哪个 claim？
2. 它回答审稿人的哪个质疑？
3. 它在论文故事链条里放在哪一环？

如果一个实验回答不了这三个问题，就不应该进入主线表格；最多作为附录、诊断或工程记录。

---

## 1. 当前主定位

第一版论文不要定位成：

```text
泛泛研究液体晃动；
提出新的流体力学模型；
提出新的液面测量方法；
提出普通 mobile robot local planner；
给定轨迹后的单纯 anti-slosh tracking controller。
```

当前最合适的定位是：

> **面向移动底盘开口液体运输的在线防晃局部轨迹规划。**

英文：

> **Online anti-slosh local trajectory planning for mobile-base open-liquid transport.**

更完整的问题定义：

> 本文研究移动底盘开口液体运输中的在线防晃局部轨迹规划问题：在给定安全参考路径的条件下，机器人需要实时生成满足底盘运动约束的局部运动，同时显式预测并抑制由平动加速度和转向运动诱发的液体晃动。

最重要的一句话：

> **核心问题不是让机器人轨迹看起来更平滑，而是规划一段与液体动态记忆相容的运动。**

---

## 2. 论文故事线：平台优先，落到 A5

当前 Related Work 和 story positioning 采用平台优先分类：

| 类别 | 内容 | 论文中的角色 |
|---|---|---|
| D | 跨平台基础方法：低阶模型、输入整形、测量评价 | 说明可用于规划的液体模型、smooth/profile baseline 的意义、真实评价方法 |
| B | 机械臂 / SCARA / 操作机器人液体防晃 | 说明机器人防晃已有研究，但平台自由度不同 |
| C | 液罐车 / 车辆 / 船舶等载液平台 | 说明移动载液系统中晃动影响安全，但尺度和目标不同 |
| A | 移动底盘 / 服务机器人液体运输 | **核心近邻方向** |
| E | 非液体感知移动机器人 local planner | 外部 baseline，说明 ordinary planner 没有液体动态记忆 |

A 类内部重点细分为：

```text
A1 固定路径 / 曲线路径设计
A2 速度剖面 / input shaping
A3 离线防晃轨迹优化 / time-optimal planning
A4 给定轨迹后的防晃跟踪控制 / 特殊机构抑振
A5 在线滚动防晃局部规划  ← SPMPC 所属位置
A6 普通 local planner 对照
```

SPMPC 的位置：

```text
A 移动底盘 / 服务机器人液体运输
└── A5 在线滚动防晃局部规划
    └── SPMPC: slosh-aware MPCC for mobile-base open-liquid transport
```

---

## 3. 一句话故事线

中文主线：

> 液体防晃已经在机械臂、SCARA、液罐车和移动底盘等平台上被研究；跨平台基础方法提供了低阶液体模型、输入整形和液面测量评价。对于移动底盘液体运输，已有方法多集中在固定路径设计、速度剖面、离线轨迹优化或给定轨迹后的防晃控制。另一方面，普通移动机器人 local planner 虽然能够在线生成平滑可行轨迹，但不包含液体动态记忆。因此，本文提出 SPMPC，将低阶液体模态状态嵌入 MPCC，在移动底盘局部规划层在线优化路径进度、底盘控制和预测液体响应。

英文主线：

> Existing anti-slosh studies cover manipulators, operation robots, liquid-carrying vehicles, and mobile-base transport, supported by low-order slosh models, input shaping, and liquid-surface measurements. However, mobile-base liquid-transport methods often rely on fixed path design, velocity profiles, offline trajectory optimization, or tracking control, while ordinary mobile-robot local planners are online but lack the dynamic memory of the carried liquid. SPMPC addresses this gap by embedding low-order liquid modal states into an online receding-horizon MPCC local planner for mobile-base open-liquid transport.

这条故事线应贯穿：

```text
Title → Abstract → Introduction → Related Work → Method → Experiments → Discussion
```

---

## 4. 第一版论文范围

第一版论文建议固定为：

> 面向化学实验室开口/半开口液体运输的在线 slosh-aware alpha-state MPCC local trajectory planner/controller。

### 4.1 应该写入主线的内容

- chemical laboratory open/semi-open liquid transport；
- predefined safe laboratory route / fixed global reference path；
- wheeled mobile robot / Scout Mini；
- online receding-horizon local trajectory planning and control；
- alpha-state chassis dynamics：`omega` 是状态，`alpha = dot(omega)` 是控制；
- liquid modal state augmentation：`eta_x, eta_x_dot, eta_y, eta_y_dot`；
- slosh-aware MPCC objective；
- smooth-only vs slosh-aware 消融；
- ordinary local planner baseline；
- RGB / external observation 作为真实液面评价，`/spmpc/slosh_height` 作为 model proxy。

### 4.2 第一版不应过度宣称的内容

不要把下面内容写成主方法贡献：

- Map-vref；
- terrain-conditioned slosh experience prior；
- 完整 obstacle-aware MPCC；
- homotopy / corridor 主方法；
- stochastic MPC / covariance propagation / chance constraints；
- CBF 是 SPMPC 主体；
- `/spmpc/slosh_height` 等同真实液面高度；
- CFD/FEM/SPH 高保真模型进入在线规划内核。

推荐 scope statement：

> This work focuses on slosh-aware local trajectory planning and control along a predefined safe laboratory route. Full obstacle-aware MPCC, homotopy reasoning, corridor constraints, stochastic chance constraints, and formal closed-loop stability proofs are outside the scope of this first version.

---

## 5. 论文 claim 链条

论文主线应围绕下面 8 个 claim 展开。

| ID | Claim | 对应分类 | 主要证据 |
|---|---|---|---|
| C1 | 液体晃动有动态记忆，轨迹规划需要规划可用的低阶液体状态，而不只是瞬时平滑指标。 | D1 | A01/A02/A03 + SPMPC slosh dynamics |
| C2 | input shaping / 速度剖面 / smooth profile 有价值，但不等于 online slosh-aware planning。 | D2 / A2 | A07、A2 文献 + `B_ours vs B_smooth` |
| C3 | 机械臂、SCARA 和载液车辆平台证明防晃重要，但平台自由度、任务目标和同层 baseline 不同。 | B / C | C 组文献 + tank vehicle/vehicle 文献 |
| C4 | 移动底盘液体运输已有 fixed path、velocity profile、offline optimization 和 tracking control。 | A1-A4 | B 组近邻文献矩阵 |
| C5 | 普通 mobile-robot local planner 虽然 online，但没有液体动态状态。 | E | DWA/TEB/RPP/LT-DWA/MPPI 等文献 + baseline 实验 |
| C6 | SPMPC 的核心贡献是 A5：在线滚动地联合优化路径进度、底盘控制和预测液体响应。 | A5 | alpha-state MPCC + acados/ROS 实现 |
| C7 | 真实评价应使用 RGB / 外部液面观测，不能只依赖内部 model proxy。 | D3 | A08-A11 + RGB max-LCR 协议 |
| C8 | 第一版不宣称完整 obstacle-aware / homotopy / corridor MPCC 主方法。 | scope | 代码边界 + scope statement |

---

## 6. 推荐草稿文件结构

```text
docs/论文书写/草稿/
├── README.md                         # 本文件：写作原则与框架
├── spmpc_paper_cn/                   # 中文论文框架/草稿 LaTeX
│   ├── main.tex
│   └── sections/
│       ├── 01_intro.tex
│       ├── 02_related_work.tex
│       ├── 03_method.tex
│       ├── 04_experiments.tex
│       └── 05_conclusion.tex
├── 00_storyline_and_claims.md         # 故事线、claim、贡献、审稿人质疑
├── 01_introduction_draft.md           # Introduction 草稿
├── 02_related_work_draft.md           # Related Work 草稿
├── 03_method_draft.md                 # Method 草稿
├── 04_experiment_protocol_draft.md    # 实验协议，不是结果堆砌
├── 05_results_discussion_draft.md     # Results + Discussion 草稿
├── 06_abstract_conclusion_draft.md    # Abstract 和 Conclusion 最后写
└── figures_and_tables_plan.md         # 图表规划
```

---

## 7. 推荐写作顺序

实际写作顺序不要按论文最终章节顺序来。建议：

```text
1. 00_storyline_and_claims.md / README.md
2. 02_related_work_draft.md
3. 01_introduction_draft.md
4. spmpc_paper_cn/sections/01_intro.tex 和 02_related_work.tex
5. 03_method_draft.md / 03_method.tex
6. 04_experiment_protocol_draft.md / 04_experiments.tex
7. figures_and_tables_plan.md
8. 05_results_discussion_draft.md
9. 06_abstract_conclusion_draft.md / 05_conclusion.tex
```

原因：

- Related Work 决定 gap；
- gap 决定 Introduction；
- claim 决定 Method 叙事；
- claim 决定实验；
- Results 只是证据组织；
- Abstract 和 Conclusion 必须最后压缩。

---

## 8. 论文整体结构建议

### 8.1 Title

标题应包含三个关键词：

1. anti-slosh / slosh-aware；
2. online local trajectory planning / MPCC；
3. mobile-base open-liquid transport。

候选：

```text
Online Anti-Slosh Local Trajectory Planning for Mobile Robots Transporting Open Liquids
```

或：

```text
Slosh-Aware Model Predictive Contouring Control for Mobile-Base Open-Liquid Transport
```

中文草稿标题：

```text
面向移动底盘开口液体运输的在线防晃局部轨迹规划方法
```

### 8.2 Abstract

Abstract 不要写成技术清单，建议 5 句结构：

1. 应用场景和风险：chemical laboratory open-liquid transport；
2. gap：mobile-base anti-slosh methods 多偏 profile/offline/tracking，ordinary local planners 缺少液体动态记忆；
3. 方法：slosh-aware alpha-state MPCC with modal liquid states；
4. 实验：ablation + local planner baselines + RGB liquid evaluation；
5. 结论：reduced liquid oscillation while maintaining path following and real-time execution。

### 8.3 Introduction

Introduction 建议 6 段：

1. 化学实验室开口液体运输的需求和风险；
2. 液体晃动是由运动历史诱发的动态响应，不是瞬时平滑问题；
3. 已有防晃研究覆盖 B/C/D 类平台与基础方法；
4. A 类移动底盘近邻仍多偏 fixed path/profile/offline/tracking，E 类 ordinary planner 又没有液体状态；
5. 提出 SPMPC 的核心思想；
6. 贡献列表和文章结构。

贡献列表建议最多 3 条：

```text
1. Formulation: slosh-aware alpha-state MPCC with modal liquid-state augmentation.
2. Deployment: real-time ROS/acados local planner for mobile-base open-liquid transport.
3. Evaluation: ablation and baseline comparisons showing explicit slosh prediction is not equivalent to smooth-only online planning.
```

### 8.4 Related Work

推荐结构：

```text
2.1 Cross-platform foundations: slosh models, input shaping, and measurement
2.2 Anti-slosh motion planning in manipulators and operation robots
2.3 Sloshing-aware control and planning in liquid-carrying vehicles
2.4 Mobile-base liquid transport and anti-slosh trajectory planning
2.5 Mobile-robot local planning without liquid dynamics
2.6 Summary of the gap and positioning of SPMPC
```

每节目的：

| 小节 | 对应分类 | 目的 |
|---|---|---|
| 2.1 | D | 支撑低阶模型、smooth/profile 对照和真实评价 |
| 2.2 | B | 说明机器人防晃有价值，但平台自由度不同 |
| 2.3 | C | 说明移动载液系统的安全意义，但尺度不同 |
| 2.4 | A1-A4 | 建立最关键 near-neighbor gap |
| 2.5 | E | 说明普通 online planner 缺少液体动态记忆 |
| 2.6 | A5 | 把 SPMPC 放到 online slosh-aware local planning |

不要把 Related Work 写成论文列表，要写成 gap 推导。

### 8.5 Method

Method 建议结构：

```text
1. Problem Formulation and System Overview
2. Reference Path and Progress Parameterization
3. Alpha-State Mobile-Robot Dynamics
4. Slosh Modal Dynamics and Markov Augmentation
5. Augmented Slosh-Aware MPCC Model
6. MPCC Objective: Tracking, Progress, Smoothness, and Slosh Cost
7. Constraints and Command Generation
8. Receding-Horizon Solution with acados SQP-RTI
9. Method Variants for Ablation
```

Method 的重点不是“公式多”，而是把公式服务于故事：

- 为什么需要 `s, v_s`？因为这是局部轨迹规划/MPCC；
- 为什么需要 `omega` state 和 `alpha` control？因为要约束转向激励；
- 为什么需要 `eta, eta_dot`？因为液体有动态记忆；
- 为什么 slosh height 是 output/proxy？因为真实液面需要外部评价。

### 8.6 Experiments

Experiments 不要写成“我们跑了很多算法”。建议按 claim 分组：

```text
1. Experimental setup and fairness protocol
2. Internal ablation: slosh-aware vs smooth-only
3. External local-planner comparison
4. Real-liquid evaluation with RGB max-LCR
5. Real-time and diagnostic analysis
```

核心实验映射：

| 实验 | 支撑 claim |
|---|---|
| `B_ours vs B_smooth` | smooth-only 不等于 slosh-aware |
| `B_ours vs B_slosh` | slosh state/cost 与 smooth shaping 的贡献分离 |
| `B_ours vs DWA/TEB/LT-DWA` | ordinary local planner 缺少液体动态记忆 |
| RGB max-LCR | 真实液面评价，不把 proxy 当真值 |
| solver time | online local planner 可实时运行 |

### 8.7 Results and Discussion

Results 的写法：

```text
Claim → Evidence → Interpretation → Limitation
```

推荐每个结果段落回答：

1. 这个结果说明哪个 claim？
2. 它是否排除了“只是更慢/更平滑”的质疑？
3. 它有什么 trade-off？
4. 它有什么边界？

### 8.8 Conclusion

Conclusion 不要重复实验数字。建议回到故事：

- open-liquid transport 需要局部规划器理解液体动态记忆；
- SPMPC 给出了一个在线 slosh-aware MPCC formulation；
- 实验证明显式 slosh prediction 相比 smooth-only 和 ordinary local planners 有独立价值；
- future work：obstacle-aware MPCC、corridor/homotopy、better sensing、model adaptation、real chemical lab deployment。

---

## 9. 图表规划

| 图/表 | 作用 |
|---|---|
| Fig. 1 System overview | fixed lab path、Scout、container、SPMPC loop、RGB evaluation |
| Fig. 2 Literature positioning diagram | D/B/C/A/E 平台优先分类，并突出 A5 SPMPC |
| Fig. 3 Dynamics and OCP structure | robot state + slosh state + MPCC objective |
| Table I Platform-first related work matrix | 按 D/B/C/A/E 和 A1-A5 整理 |
| Table II Method variants | B0/B_slosh/B_smooth/B_ours |
| Table III Main results | success/time/tracking/RGB max-LCR/solver time |
| Fig. 4 Slosh-aware vs smooth-only example | 展示相似平滑度下不同液面响应 |
| Fig. 5 Real robot / RGB evaluation | 展示外部评价，不依赖 proxy |

---

## 10. 写作风格规则

1. **每段只服务一个 claim。**
2. **每个 baseline 都要说明为什么选它。**
3. **不要用“我们最好”作为中心，要用“我们解决了这个 gap”。**
4. **数字结果必须解释成 story evidence。**
5. **失败、trade-off 和边界要主动写，不能躲。**
6. **不要把未实现功能写成贡献。**
7. **不要让公式淹没故事。**
8. **不要把文献综述写成堆引用。**
9. **普通 local planner 是外部 baseline，不是液体防晃方向本身。**
10. **机械臂/罐车/船舶是 broader related work，不是 Scout Mini 同层 baseline。**

---

## 11. 关键参考文件

写草稿时优先查看：

```text
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/参考论文整理/轨迹规划中的液体晃动问题_研究框架.md
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/参考论文整理/索引_按平台分类.md
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/参考论文整理/SPMPC_四组文献综合分析.md

docs/论文书写/书写思路/SPMPC_related_work_matrix.md
docs/论文书写/书写思路/SPMPC_claim_gap_evidence_matrix.md
docs/论文书写/书写思路/SPMPC_near_neighbor_mobile_liquid_matrix.md
docs/论文书写/书写思路/SPMPC论文写作路线_文献优先.md
```

方法代码入口：

```text
src/scout_apps/control/spmpc_local_planner
```

---

## 12. 写作检查清单

每写完一节，检查：

- [ ] 这一节是否推进了主故事？
- [ ] 这一节是否对应至少一个 claim？
- [ ] 是否有文献支撑？
- [ ] 是否有方法或实验支撑？
- [ ] 是否避免了未实现功能的过度宣称？
- [ ] 是否明确了 SPMPC 位于 A5 在线滚动防晃局部规划？
- [ ] 是否明确了 A1-A4 近邻与 SPMPC 的差异？
- [ ] 是否明确了 ordinary local planner 缺少液体动态记忆？
- [ ] 是否明确了 smooth/profile 不等于 slosh-aware？
- [ ] 是否明确了真实评价不能只看 model proxy？
- [ ] 是否明确了实验不是排行榜，而是 claim evidence？

最终原则：

> **写论文不是堆方法、堆实验、堆引用，而是讲一个可信、克制、有证据链的故事。**
