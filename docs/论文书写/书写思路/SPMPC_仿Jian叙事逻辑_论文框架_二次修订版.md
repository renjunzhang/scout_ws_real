# SPMPC 论文叙事逻辑框架：仿 Jian et al. 全文章节对齐版（二次修订版）

修订日期：2026-06-30  
用途：用于 SPMPC 论文的 Abstract、Introduction、Related Work、Method、Experiments、Discussion 和 Conclusion 的整体组织。  
核心目标：让论文的每个章节都服务同一条故事线，而不是只在 Introduction 里讲故事。

---

## 0. 本次二次修订做了什么

上一版已经完成了四件事：

1. 将 `observer` 表述收敛为 **model-based slosh-state propagation / initial-state update**；
2. 将实验拆成 **内部消融** 和 **外部 local planner 对比**；
3. 将标题中的 `continuous MPCC` 收敛为 **MPCC**；
4. 避免 `safe / no spilling / guarantee` 这类过强表述，优先使用 **reduced sloshing / slosh-aware / spill-risk-aware**。

本次二次修订新增的重点是：

1. **把“仿 Jian et al. 的叙事逻辑”扩展到论文每个章节**，而不是只用于 Introduction 和 Related Work；
2. 增加 **章节级叙事对齐表**，明确每一章回答什么问题、怎么承接上一章、怎么引出下一章；
3. 增加 **段落级写作模板**，尤其是 Introduction、Related Work、Method 和 Experiments 的开头句、收束句；
4. 增加 **Claim–Gap–Evidence 对齐框架**，保证每个 claim 都有文献支撑和实验支撑；
5. 增加 **图表叙事功能表**，让 Fig. 1、Fig. 2、Fig. 3、Table I、Table II、Table III 都服务故事，而不是只展示结果；
6. 增加 **章节之间的桥接句**，避免文章变成“相关工作一章、方法一章、实验一章”互相割裂。

一句话：

> **这一版的目标不是继续堆内容，而是把论文变成一条闭环叙事：问题定义 → 缺口形成 → 方法回应 → 实验证明 → 边界说明。**

---

## 1. 从 Jian et al. 学到的“全文章节叙事法”

Jian et al. 这篇 D-CBF-MPC 论文的核心叙事不是：

```text
我用了 LiDAR + KF + CBF + MPC。
```

而是：

```text
动态障碍物不是静态障碍物
→ 因此必须检测、参数化、预测未来状态和不确定性
→ 然后把这些未来动态信息放进 MPC 的安全约束
→ 最后用模块拆解 baseline 证明“预测”和“动态约束”各自有用。
```

它的文章结构大致是：

| 章节 | Jian et al. 的写法 | 叙事功能 |
|---|---|---|
| Abstract | 任务 → 点云/MBE/KF/D-CBF-MPC → 实验 | 先把完整链路告诉读者 |
| Introduction | 动态环境导航重要 → 三个难点 → 本文方案 | 把问题拆成可解决的模块 |
| Related Work | Local Perception + Local Planning | 按方法链路综述，而不是按年份堆文献 |
| Framework | Sensor → SLAM → local perception → prediction → local planning → controller | 用系统图让方法链路可视化 |
| Method | D-CBF + MPC + MBE/KF prediction | 每个模块回答 Introduction 里的一个难点 |
| Experiments | Real-world scenario + simulation baseline | 用真实场景证明可运行，用仿真消融证明模块有效 |
| Conclusion | 回到 dynamic obstacle avoidance 主线 | 收束到最初的问题定义 |

SPMPC 应仿照成：

| 章节 | SPMPC 应该怎么写 | 叙事功能 |
|---|---|---|
| Abstract | 任务 → 两条研究线断层 → slosh-aware MPCC → 消融和实测 | 快速交代本文填什么 gap |
| Introduction | 移动液体运输重要 → 三个难点 → 本文方案 | 把“晃液不是平滑问题”讲清楚 |
| Related Work | Slosh model → robotic anti-slosh → mobile-base transport → ordinary local planner | 让每类文献服务 gap |
| Framework / Method Overview | odom/path → slosh-state propagation → MPCC OCP → `/cmd_vel` → external evaluation | 用系统链路说明方法不是孤立 cost |
| Method | slosh model + MPCC formulation + receding-horizon command | 每个模块回应一个难点 |
| Experiments | B0/B_smooth/B_slosh/B_ours + external planners | 用消融证明 smooth-only 不等于 slosh-aware |
| Discussion | 模型 proxy、无硬防溢出保证、未来真实液面反馈 | 控制表达边界，增强可信度 |
| Conclusion | 回到 liquid sloshing as dynamic state | 收束到一句话故事 |

---

## 2. 论文的一句话故事

英文：

> **Liquid sloshing is not merely a smoothness issue of robot trajectories, but a dynamic-state prediction problem. Therefore, a mobile robot carrying an open liquid container should plan local motions by predicting future sloshing states inside a receding-horizon planner, rather than only smoothing velocity commands.**

中文：

> **液体晃动不是简单的轨迹平滑问题，而是具有动态记忆的状态预测问题。因此，移动机器人在运输开口液体时，局部规划器不应只平滑速度命令，而应在滚动时域内预测未来晃液状态，并据此生成可执行控制。**

这句话必须贯穿全文。每章都要从不同角度证明它：

```text
Introduction：为什么液体晃动不是平滑问题？
Related Work：为什么已有工作还没有把这个状态放进在线 local planner？
Method：我们如何把这个状态放进 MPCC？
Experiments：为什么只平滑不够？为什么 slosh state prediction 有用？
Discussion：这个结论在什么边界内成立？
Conclusion：回到这一句话。
```

---

## 3. 论文核心矛盾：两条线没有接起来

SPMPC 的故事不要讲成：

```text
别人方法不好，所以我们更好。
```

应该讲成：

```text
已有研究沿两条线发展，但这两条线没有在移动底盘在线局部规划层接起来。
```

两条线是：

| 研究线 | 已有能力 | 缺口 |
|---|---|---|
| **Anti-sloshing motion generation** | 有液体模型、input shaping、路径设计、速度剖面、离线轨迹优化、防晃控制 | 通常不是导航栈里的在线 local planner |
| **Mobile robot local planning** | 能在线生成可行、平滑、可执行 `/cmd_vel` | 通常没有液体模态状态，无法传播晃液动态记忆 |

SPMPC 的定位是：

> **把液体晃动的低阶模态状态嵌入移动底盘滚动时域 MPCC，使 local planner 同时具备在线性和液体动态感知能力。**

这也决定了 Related Work 的收束句应该是：

```text
Anti-sloshing methods are liquid-aware but often not online local planners; ordinary local planners are online but not liquid-aware. SPMPC bridges these two lines by embedding predicted sloshing states into a receding-horizon MPCC local planner.
```

---

## 4. 研究方向与题目表述

### 4.1 研究方向

不建议写成：

```text
Trajectory planning with liquid sloshing
```

更建议写成：

```text
Slosh-aware local trajectory planning for mobile liquid transportation
```

中文：

```text
面向移动液体运输任务的晃液感知局部轨迹规划方法
```

层级关系：

```text
Liquid sloshing suppression
    ↓
Anti-slosh trajectory/path planning
    ↓
Mobile-base liquid transportation
    ↓
Online receding-horizon slosh-aware local planning
```

中文：

```text
液体晃动抑制
    ↓
防晃轨迹/路径规划
    ↓
移动底盘液体运输
    ↓
在线滚动晃液感知局部规划
```

### 4.2 题目候选

最推荐：

> **SPMPC: Slosh-Aware Model Predictive Contouring Control for Mobile Robot Liquid Transportation**

其他候选：

1. **Slosh-Aware Model Predictive Contouring Control for Mobile Robot Liquid-Transport Local Planning**
2. **Receding-Horizon Slosh-Aware Local Planning for Mobile-Base Open-Liquid Transport**
3. **Liquid-State-Aware MPCC for Mobile Robot Liquid Transportation**

不建议：

```text
Smooth and Safe Mobile Robot Liquid Transportation
```

原因：`safe` 容易暗示 hard safety guarantee 或 no-spilling guarantee。当前更稳妥的词是：

- reduced sloshing；
- slosh-aware；
- liquid-friendly motion；
- spill-risk-aware。

---

## 5. 章节级叙事对齐总表

这张表是本次修订最重要的内容。后续写论文时，每一章都应该对照这张表检查。

| 章节 | 这一章回答的问题 | 仿 Jian 的写法 | SPMPC 的写法 | 章末应该留下什么印象 |
|---|---|---|---|---|
| Abstract | 本文到底解决什么问题？ | 任务 + 方法链路 + 实验 | open-liquid transport + anti-slosh/local-planner gap + slosh-aware MPCC + ablation | 读者马上知道本文填的是 online slosh-aware local planning gap |
| Introduction | 为什么这个问题不是普通规划问题？ | 动态环境三难点 | 晃液有动态记忆；ordinary planner 不传播液体状态；anti-slosh 方法多不是 online local planner | 液体晃动必须作为动态状态进入 local planner |
| Related Work | 前人做了什么，缺在哪里？ | Local Perception / Local Planning | Slosh model / robotic anti-slosh / mobile-base transport / ordinary local planner | 两条线没有接起来 |
| Method Overview | 方法整体链路是什么？ | Fig. 2 系统框架 | odom + path → slosh-state propagation → MPCC → `/cmd_vel` | SPMPC 是一个闭环 local planner，不是离线轨迹生成器 |
| Slosh Model | 未来动态信息从哪里来？ | MBE + KF 预测未来障碍物 | low-order modal model 传播未来晃液状态 | 液体状态在时域内被显式传播 |
| MPCC Formulation | 动态信息如何进入优化？ | D-CBF 进入 MPC 约束 | slosh dynamics + slosh cost 进入 MPCC OCP | OCP 同时优化 path tracking、progress、smoothness、slosh response |
| Implementation | 如何在线运行？ | local planning 10 Hz、只执行控制命令 | receding-horizon solve、执行第一帧 `/cmd_vel` | 方法能接入移动机器人控制回路 |
| Experiments | 每个模块是否真的有用？ | MPC / MPC-CBF / MPC-KF / curvefit / Ours | B0 / B_smooth / B_slosh / B_ours + external planners | smooth-only 不等于 slosh-aware，完整方法最稳 |
| Discussion | 结论边界是什么？ | 总结有效性和实时性 | 不声称 spill-free；模型 proxy 与真实测量区分；未来加入反馈和约束 | 方法可信但不过度宣称 |
| Conclusion | 本文贡献是什么？ | 回到 dynamic obstacle avoidance | 回到 liquid sloshing as predicted dynamic state | 读者记住一句话故事 |

---

## 6. Abstract 写法

### 6.1 Abstract 的叙事公式

仿 Jian et al.，Abstract 不要先写公式，而要写完整链路：

```text
任务问题
→ 现有方法缺口
→ 本文方法
→ 方法中最关键的新东西
→ 实验如何验证
→ 主要结论
```

SPMPC 对应为：

```text
移动机器人运输开口液体需要平滑且晃液感知的运动
→ anti-slosh 方法和 ordinary local planner 各有缺口
→ 提出 SPMPC
→ 将低阶晃液模态模型嵌入 receding-horizon MPCC
→ 用内部消融和外部 planner 对比验证
→ 显式预测晃液状态比 smooth-only 更有效
```

### 6.2 英文 Abstract 草稿

```text
Mobile robots carrying open liquid containers must generate motions that are not only feasible and smooth, but also aware of liquid sloshing. Existing anti-sloshing methods reduce liquid motion through input shaping, path design, transfer control, or offline trajectory optimization, while conventional mobile robot local planners ignore the dynamic memory of liquid states. This paper presents SPMPC, a slosh-aware Model Predictive Contouring Control local planner for mobile-base liquid transportation. A low-order sloshing modal model is embedded into a receding-horizon optimal control problem, where path tracking, path progress, executable velocity commands, control smoothness, and predicted sloshing states are jointly optimized. A model-based slosh-state propagation module provides the initial liquid state at each control cycle, and the first optimized control input is applied as the mobile-base command. Simulation and real-world experiments compare the proposed method with basic MPCC, smooth-only MPCC, slosh-only MPCC, and non-slosh-aware local planners. The results show that explicitly predicting sloshing states reduces liquid motion beyond what can be achieved by trajectory smoothness alone, while maintaining path tracking and real-time performance.
```

### 6.3 中文 Abstract 草稿

> 移动机器人在运输开口液体容器时，需要生成不仅可行、平滑，而且能够感知液体晃动的运动。已有防晃方法通过输入整形、路径设计、转运控制或离线轨迹优化降低液体晃动，而普通移动机器人局部规划器通常忽略液体状态的动态记忆。本文提出 SPMPC，一种面向移动底盘液体运输任务的晃液感知 MPCC 局部规划器。该方法将低阶晃液模态模型嵌入滚动时域最优控制问题，在同一框架中联合优化路径跟踪、路径进度、可执行速度命令、控制平滑性和预测晃液状态。基于模型的晃液状态传播模块在每个控制周期提供液体初始状态，优化得到的第一帧控制量被发送到底盘执行。仿真与真实实验将本文方法与基础 MPCC、smooth-only MPCC、slosh-only MPCC 以及非液体感知 local planner 进行比较。结果表明显式预测晃液状态能够在轨迹平滑之外进一步降低液体晃动，同时保持路径跟踪和实时性能。

注意：如果当前没有真实液面反馈闭环，不建议在 Abstract 里使用 **observer**，更稳妥的是 **model-based slosh-state propagation**。

---

## 7. Introduction 叙事框架

Introduction 应该仿 Jian et al. 的三步：

```text
任务重要
→ 问题困难拆成三点
→ 本文提出完整链路
```

但 SPMPC 要再多一步：把已有研究的“两条线断层”讲出来。

### 7.1 段落 1：任务动机

目标：告诉读者为什么 mobile liquid transportation 重要。

英文草稿：

```text
Mobile robots are increasingly expected to transport open or partially filled liquid containers in service, laboratory, and industrial environments. During such tasks, acceleration, braking, and turning commands generated by ordinary local planners can excite liquid sloshing. Excessive sloshing may degrade transport stability and increase the risk of spilling or contamination, which limits the deployment of mobile robots in liquid-handling applications.
```

中文含义：

> 移动机器人越来越多地被期望用于服务、实验室和工业环境中的液体运输任务。与普通固体负载不同，开口或半开口液体容器会在加速、制动和转向过程中发生晃动；过大的晃动会降低运输稳定性，并增加溢出和污染风险。

段末钩子：

```text
Therefore, liquid transportation requires a local planner that is not only feasible and smooth, but also aware of liquid motion.
```

中文：

> 因此，液体运输任务需要的不只是可行和平滑的局部规划器，还需要能感知液体动态的局部规划器。

---

### 7.2 段落 2：拆三个难点

仿 Jian et al. 的 Introduction，把问题拆成三个具体困难。

英文草稿：

```text
The difficulties of slosh-aware mobile liquid transportation are mainly threefold. First, liquid sloshing has dynamic memory, so the future liquid response depends on the internal modal state rather than only on instantaneous acceleration. Second, conventional mobile robot local planners optimize path tracking, feasibility, progress, and smoothness, but do not propagate liquid states. Third, existing anti-sloshing methods often rely on predefined paths, velocity profiles, offline trajectory optimization, manipulator motion, tank-vehicle dynamics, or special platforms, rather than online local planning for a wheeled mobile robot carrying an open liquid container.
```

中文：

> 这个问题的难点主要有三点。第一，液体晃动具有动态记忆，未来晃动响应不仅取决于瞬时加速度，还取决于当前液体模态状态。第二，普通移动机器人局部规划器通常优化路径跟踪、可行性、路径进度和控制平滑性，但不传播液体状态。第三，已有防晃方法多依赖预定义路径、速度剖面、离线轨迹优化、机械臂运动、液罐车动力学或特殊平台，而不是面向开口液体运输的轮式移动机器人在线局部规划。

这三点后续必须一一对应：

| Introduction 难点 | Related Work 回答 | Method 回答 | Experiments 回答 |
|---|---|---|---|
| 液体有动态记忆 | D1 低阶模型 / A01-A03 | low-order slosh state propagation | B_slosh vs B0 |
| ordinary planner 不传播液体状态 | E 类普通 planner | slosh states in MPCC horizon | B_ours vs external planners |
| anti-slosh 方法多不是 online local planner | A1-A4、B/C 类 | receding-horizon MPCC local planner | real-time solver time + `/cmd_vel` execution |

---

### 7.3 段落 3：已有方法各自解决一部分，但没有合在一起

英文草稿：

```text
Previous studies have investigated liquid sloshing from several perspectives. Low-order sloshing models and input-shaping techniques provide compact descriptions of liquid dynamics and vibration suppression. Robotic manipulation studies show that slosh-free or spill-avoiding motions can be generated by planning container trajectories. Mobile-platform studies further demonstrate that path geometry, velocity profiles, active vibration reducers, and slosh-constrained trajectory optimization can reduce liquid motion. Meanwhile, conventional mobile robot local planners can generate feasible and smooth commands online. However, these two lines of work remain largely separated: anti-sloshing methods are often not embedded in an online local planner, while ordinary local planners do not contain liquid states.
```

中文：

> 已有研究已经从低阶晃液建模、输入整形、机械臂防晃轨迹规划、移动平台路径/速度设计、主动抑振机构以及液体约束轨迹优化等角度研究了防晃问题。同时，普通移动机器人局部规划器已经能够在线生成可行且平滑的运动命令。然而，这两条线索仍存在断层：传统防晃方法通常不是导航栈中的在线局部规划器，而普通局部规划器又没有液体动态状态。

段末钩子：

```text
This separation motivates a local planner that embeds liquid-state prediction directly into online motion generation.
```

---

### 7.4 段落 4：提出 SPMPC

英文草稿：

```text
To address these issues, this paper proposes SPMPC, a slosh-aware Model Predictive Contouring Control local planner for mobile-base liquid transportation. A low-order sloshing modal model is embedded into a receding-horizon optimal control problem. At each control cycle, the planner jointly optimizes contouring and lag errors, path progress, executable chassis commands, control smoothness, and predicted sloshing states, and outputs the first velocity command to the mobile base.
```

中文：

> 为解决上述问题，本文提出 SPMPC，一种面向移动底盘液体运输任务的晃液感知 MPCC 局部规划器。该方法将低阶晃液模态模型嵌入滚动时域最优控制问题，在每个控制周期内联合优化 contour/lag 路径误差、路径进度、可执行底盘命令、控制平滑性以及预测晃液状态，并将第一帧控制量发送到底盘执行。

---

### 7.5 段落 5：贡献列表

建议三条贡献，不要太散。

英文草稿：

```text
The main contributions are summarized as follows:

1. We formulate an online slosh-aware MPCC local planner for mobile robots carrying open liquid containers, bridging anti-sloshing motion generation and mobile robot local planning.

2. We embed a low-order sloshing modal model into the receding-horizon OCP, enabling the planner to reason about predicted liquid states rather than relying only on instantaneous acceleration or trajectory smoothness.

3. We validate the proposed planner through module-wise ablation and mobile-base liquid-transport experiments, comparing basic MPCC, smooth-only MPCC, slosh-only MPCC, and non-slosh-aware local planners using both planning metrics and external liquid-motion measurements.
```

中文：

1. 提出一种面向移动机器人开口液体运输任务的在线晃液感知 MPCC 局部规划器，将防晃运动生成与移动机器人局部规划连接起来；
2. 将低阶晃液模态模型嵌入滚动时域 OCP，使局部规划器能够预测未来液体状态，而不仅依赖瞬时加速度或轨迹平滑性；
3. 通过模块拆解式消融和移动底盘液体运输实验验证方法，对比 basic MPCC、smooth-only MPCC、slosh-only MPCC 以及非液体感知 local planner，并同时使用规划性能指标和外部液面晃动指标进行评价。

---

## 8. Related Work 叙事框架

Related Work 不应写成平台流水账，而应让每类文献服务一个 gap。建议正式论文压缩成四节：

```text
A. Sloshing Models, Input Shaping, and Measurement
B. Anti-Sloshing Motion Planning on Robotic Platforms
C. Mobile-Base Liquid Transportation
D. Online Local Planning without Liquid States
```

最后用一段 `Summary of Gap` 收束，不一定单独成节。

---

### 8.1 Sloshing Models, Input Shaping, and Measurement

本节回答：

```text
液体晃动如何被建模？
为什么可以用低阶模型而不是 CFD？
为什么 smooth / input shaping baseline 有意义？
实验如何评价真实晃动？
```

推荐文献作用：

| 论文 ID | 用途 |
|---|---|
| A01 Sloshing Dynamics Estimation for Liquid-filled Containers under 2-Dimensional Excitation | 低阶模态 / 等效动力学，最接近 SPMPC 的 slosh state 建模依据 |
| A02 A Simple Model-Based Method for Sloshing Estimation in Liquid Transfer in Automatic Machines | model-based sloshing estimation，支撑状态预测思想 |
| A03 Modeling of liquid sloshing with application in robotics and automation | robotics and automation 中的 slosh modeling 背景 |
| A04 Simulation of liquid sloshing in 2D containers using the VOF method | 高保真仿真背景，用来说明 CFD 不适合在线 planner |
| A05 Nonlinear finite element analysis of liquid sloshing in complex vehicle motion scenarios | 复杂运动下高保真分析，作为大尺度 / 高保真背景 |
| A07 Preshaping Command Inputs to Reduce System Vibration | input shaping / command preshaping 基础 |
| A08 Slosh Measuring Sensor System for Liquid-Carrying Robots | 液体运输机器人真实 slosh 测量 |
| A09 / A10 / A11 | 机器视觉、弯月面、电容式液位检测，评价方法背景 |

段落模板：

```text
Low-order sloshing models have been widely used to represent the dominant liquid motion in a compact form suitable for control and planning. Compared with high-fidelity VOF or FEM simulations, such models provide a practical trade-off between physical interpretability and real-time computation. Input shaping and command preshaping further show that liquid or vibration responses can be reduced by shaping the excitation signal. These studies provide the modeling and baseline foundations for SPMPC. However, modeling and input shaping alone do not define how a mobile robot should online generate local commands while tracking a global path.
```

中文：

> 低阶晃液模型能够用紧凑状态描述主导液体运动，相比 VOF/FEM 等高保真方法更适合在线规划与控制。输入整形和命令预整形方法也表明，通过改变激励输入可以降低振动或晃动响应。这些工作为 SPMPC 的液体模型和 smooth/profile baseline 提供基础，但它们本身并没有解决移动机器人在跟踪全局路径时如何在线生成局部控制命令的问题。

收束句：

> **模型基础 ≠ 在线 local planner。**

---

### 8.2 Anti-Sloshing Motion Planning on Robotic Platforms

本节主要讲机械臂 / SCARA / 操作机器人。作用不是同层比较，而是说明机器人防晃规划有价值。

推荐文献作用：

| 论文 ID | 用途 |
|---|---|
| C01 Time-Optimal Anti-Sloshing Trajectory Planning for Multiple Liquid-Filled Containers Subject to SCARA Motion | SCARA 多容器 time-optimal 防晃 |
| C02 A Solution to Slosh-free Robot Trajectory Optimization | slosh-free robot trajectory optimization |
| C03 Manipulating liquids with robots: A sloshing-free solution | 机器人操作液体经典防晃 |
| C04 Trajectory planning for meal assist robot considering spilling avoidance | 防洒 / meal assist 应用 |
| C05 Sloshing Suppression Control by using Physical Boundary Element Model and Predictive Control | BEM + predictive control |
| C06 Anti-sloshing control: Flatness-based trajectory planning and tracking control with ESO | flatness + observer + tracking |
| C07 Robust output feedback strategy for liquid handling using reconfigurable robots | reconfigurable robot / robust feedback |

段落模板：

```text
Anti-sloshing motion planning has also been investigated for manipulators, SCARA robots, and liquid-handling robots. These studies plan slosh-free or spill-avoiding end-effector trajectories, exploit the extra degrees of freedom of robotic arms, or design predictive and observer-based controllers for liquid handling. They demonstrate the benefit of explicitly considering liquid dynamics in robotic motion generation. Nevertheless, manipulator-based liquid handling differs from mobile-base transportation: the control variables are joint or end-effector motions, while a mobile base local planner must generate executable linear and angular velocity commands under nonholonomic constraints.
```

中文：

> 机械臂、SCARA 和液体操作机器人中的防晃轨迹规划已经证明了显式考虑液体动力学的价值。这些方法通常规划末端执行器或容器轨迹，并可利用机械臂额外自由度来减小晃动。然而，机械臂液体操作与移动底盘液体运输的控制接口和运动约束不同：前者控制关节或末端位姿，后者必须在非完整约束下在线生成线速度和角速度命令。

收束句：

> **机械臂防晃有启发，但不是移动底盘 local planner 的同层 baseline。**

---

### 8.3 Mobile-Base Liquid Transportation

这是 SPMPC 的核心近邻。建议按 A1–A4 逐层收束到 A5。

#### 8.3.1 固定路径 / 曲线路径设计

参考论文：B02、B04。

段落模板：

```text
Several mobile-base studies reduce sloshing by designing the path geometry or curved transfer path. These methods reveal that curvature and turning profiles strongly affect liquid excitation. However, they are often formulated around predefined paths or transfer tasks, whereas a local planner must repeatedly optimize a short horizon according to the current robot state and reference path.
```

中文：

> 一些移动底盘研究通过设计路径几何或曲线路径来降低晃动。这些方法说明曲率和转向过程会显著影响液体激励。然而，它们通常围绕预定义路径或转运任务展开，而局部规划器需要根据当前机器人状态和参考路径反复优化短时域运动。

#### 8.3.2 速度剖面 / input shaping

参考论文：B03、B10、A07、D12、D13。

段落模板：

```text
Velocity-profile design and input-shaping methods suppress sloshing by reducing excitation around the liquid natural frequency. They are important baselines because they show that smooth commands can reduce sloshing. However, smoothness alone does not encode the internal sloshing state; two trajectories with similar acceleration smoothness can lead to different future liquid responses depending on the current modal displacement and velocity.
```

中文：

> 速度剖面设计和输入整形方法通过降低液体固有频率附近的激励来抑制晃动。它们是重要的传统 baseline，因为它们说明平滑命令确实能够降低液体激励。然而，平滑性本身并不包含液体内部状态；两个具有相似加速度平滑性的轨迹，可能因为当前模态位移和速度不同而产生不同的未来液体响应。

核心句：

> **smooth-only 不等于 slosh-aware。**

#### 8.3.3 离线防晃轨迹优化 / time-optimal planning

参考论文：B01、B11。

段落模板：

```text
More recent studies formulate anti-sloshing motion generation as a trajectory optimization problem with explicit liquid dynamics or time-optimal objectives. These works are closest to our formulation because they also reason about liquid states during motion generation. The main difference is the planning level: SPMPC focuses on online receding-horizon local planning for a mobile base following a reference path, where the current robot state and sloshing state are updated every control cycle.
```

中文：

> 近期一些工作将防晃运动生成表述为包含显式液体动力学或时间最优目标的轨迹优化问题。这类工作与本文最接近，因为它们同样在运动生成过程中考虑液体状态。主要区别在于规划层级：SPMPC 面向移动底盘跟踪参考路径时的在线滚动局部规划，每个控制周期都会更新机器人状态和晃液状态。

#### 8.3.4 给定轨迹后的防晃跟踪控制 / 特殊机构

参考论文：B05、B06、B07、B08。

段落模板：

```text
Another line of work suppresses sloshing using active vibration reducers, parallel linkage mechanisms, omnidirectional platforms, or tracking controllers. These approaches are effective in their dedicated platforms, but they require additional hardware, different actuation capabilities, or assume a given trajectory. In contrast, SPMPC targets a standard mobile base and modifies the local planning objective itself.
```

中文：

> 另一类工作通过主动抑振机构、并联机构、全向平台或跟踪控制器抑制晃动。这些方法在各自平台中是有效的，但通常需要额外硬件、不同驱动能力或给定轨迹。相比之下，SPMPC 面向普通移动底盘，并直接修改 local planning objective。

#### 8.3.5 小结：落到 A5

英文：

```text
In summary, mobile-base liquid transportation has been studied through path design, velocity shaping, active vibration reduction, tracking control, and slosh-constrained trajectory optimization. However, these methods do not fully address the online local planning problem in which a mobile robot must follow a global path while continuously updating both the robot state and the liquid sloshing state. This motivates an online slosh-aware MPCC local planner.
```

中文：

> 总结来看，移动底盘液体运输已有路径设计、速度整形、主动抑振、跟踪控制和液体约束轨迹优化等方法。然而，这些方法尚未充分解决导航栈中的在线局部规划问题：移动机器人需要在跟踪全局路径的同时，持续更新机器人状态和液体晃动状态，并实时生成下一段可执行控制。因此，本文提出在线晃液感知 MPCC 局部规划器。

---

### 8.4 Online Local Planning without Liquid States

这一节不是防晃文献，而是外部 baseline 来源。

参考论文：D01、D02、D03、D05、D06、D07、D12、D13。

段落模板：

```text
Conventional mobile robot local planners, including path tracking, DWA-style kinodynamic planners, trajectory optimization, MPC/MPPI, and jerk-limited trajectory generation, can generate feasible and smooth commands online. They are important comparisons because they represent what a navigation system can do without liquid awareness. Nevertheless, these planners do not include sloshing modal states in their prediction model or objective, and thus cannot explicitly trade path tracking and progress against predicted liquid motion.
```

中文：

> 普通移动机器人局部规划器，包括路径跟踪、DWA 类 kinodynamic planner、轨迹优化、MPC/MPPI 和 jerk-limited 轨迹生成方法，能够在线生成可行且平滑的控制命令。它们是重要对照，因为它们代表了“不考虑液体状态”的导航系统能力。然而，这些规划器的预测模型和目标函数中没有晃液模态状态，因此无法显式权衡路径跟踪、路径进度与未来液体响应。

收束句：

> **ordinary online/smooth ≠ slosh-aware online planning。**

---

### 8.5 Summary of Gap

英文版：

```text
The above literature suggests that sloshing-aware motion generation and mobile robot local planning have been developed largely along two separate lines. Anti-sloshing studies provide liquid models, input shaping, path design, trajectory optimization, and transfer control, but are often not formulated as an online local planner for a standard mobile base. Ordinary local planners provide online feasibility and smoothness, but ignore the dynamic memory of liquid sloshing. This paper bridges this gap by embedding a low-order sloshing model into a receding-horizon MPCC local planner.
```

中文版：

> 综上，防晃运动生成和移动机器人局部规划长期沿两条线发展。防晃研究提供了液体模型、输入整形、路径设计、轨迹优化和转运控制方法，但通常不是面向普通移动底盘的在线局部规划器；普通局部规划器具有在线性、可行性和平滑性，但忽略了液体晃动的动态记忆。本文通过将低阶晃液模型嵌入滚动时域 MPCC 局部规划器来连接这两条线。

---

## 9. Method 叙事框架

Method 要学 Jian et al. 的系统链路写法：先给整体框架图，再解释每个模块如何回答 Introduction 里的困难。

推荐结构：

```text
III. Method
A. Overview of the SPMPC Framework
B. Low-Order Sloshing Model and State Propagation
C. Slosh-Aware MPCC Formulation
D. Receding-Horizon Implementation and Command Generation
```

本版本继续避免使用：

```text
Sloshing Modal Model and Observer
```

更推荐：

```text
Low-Order Sloshing Model and State Propagation
```

原因：如果当前没有真实液面测量反馈校正，不应过度使用 `observer` 一词。

---

### 9.1 Overview of the SPMPC Framework

建议系统框架图：

```text
Odom / optional IMU + Reference Path
        ↓
Slosh-State Propagation + Progress Projection
        ↓
Local Reference Construction
        ↓
Slosh-Aware MPCC OCP
        ↓
Local Trajectory + /cmd_vel
        ↓
Mobile Robot + Liquid Container
        ↓
External Liquid-Motion Evaluation
```

段落模板：

```text
Fig. X shows the overall framework of SPMPC. Given odometry, optional IMU information, and a global reference path, the planner updates a model-based slosh-state propagation module and projects the robot state onto the reference path. Then, a local reference is constructed and passed to the MPCC solver. The OCP predicts both the mobile-base motion and the liquid modal response over a finite horizon. The first optimized control input is converted into executable linear and angular velocity commands.
```

中文：

> 图 X 展示了 SPMPC 的整体框架。系统输入为里程计、可选 IMU 信息和全局参考路径。规划器首先更新基于模型的晃液状态传播模块，并将机器人当前位置投影到参考路径进度上；随后构造局部参考并传入 MPCC 求解器。OCP 在有限时域内同时预测移动底盘运动和液体模态响应，最后将第一帧优化控制量转换为可执行的线速度和角速度命令。

章内功能：

> 这节对应 Jian et al. 的 Fig. 2。它告诉读者：SPMPC 不是一个孤立的 cost，而是完整的 online local planning loop。

---

### 9.2 Low-Order Sloshing Model and State Propagation

这一节对应 Jian et al. 的 obstacle trajectory prediction。Jian et al. 把障碍物未来轨迹参数化成椭圆序列；SPMPC 把液体未来状态参数化成模态状态序列。

建议状态写法：

```text
x_s = [eta_x, eta_dot_x, eta_y, eta_dot_y]
```

激励：

```text
a_x = \dot{v}
a_y \approx v \omega
```

如果 IMU lateral acceleration 最终作为正式配置，则写：

```text
a_y can be obtained either from the kinematic approximation v omega or from a preprocessed IMU lateral acceleration channel.
```

段落模板：

```text
To make liquid dynamics available to the local planner, SPMPC adopts a low-order sloshing modal model. The current sloshing state is propagated using the estimated container acceleration, and the resulting state is used as the initial liquid state for the OCP at each planning cycle. Unlike methods that only penalize acceleration or jerk, this formulation carries the liquid modal displacement and velocity across control cycles.
```

中文：

> 为了让局部规划器能够使用液体动态信息，SPMPC 采用低阶晃液模态模型。当前晃液状态根据估计的容器加速度进行传播，并作为每个规划周期 OCP 的液体初值。与仅惩罚加速度或 jerk 的方法不同，这种形式在控制周期之间保留了液体模态位移和速度。

段末收束句：

```text
This propagated modal state is the key variable that distinguishes slosh-aware planning from smooth trajectory generation.
```

中文：

> 这个被跨周期传播的模态状态，是晃液感知规划区别于普通平滑轨迹生成的关键变量。

---

### 9.3 Slosh-Aware MPCC Formulation

核心不是“公式很多”，而是告诉读者 OCP 在优化什么。

状态可写成：

```text
x = [p_x, p_y, theta, v, s, omega, eta_x, eta_dot_x, eta_y, eta_dot_y]
```

控制可写成：

```text
u = [a, alpha, v_s]
```

目标函数可分解为：

```text
J = J_contour + J_lag + J_progress + J_velocity + J_control + J_smooth + J_slosh
```

各项作用：

| cost | 作用 | 对应故事 |
|---|---|---|
| `J_contour` | 减少法向路径误差 | 不偏离参考路径 |
| `J_lag` | 减少切向滞后 | 不落后路径 |
| `J_progress` | 鼓励前进 | 保持效率 |
| `J_velocity` | 跟踪参考速度 | 可控速度 |
| `J_control` | 限制控制幅值 | 可执行 |
| `J_smooth` | 平滑控制变化 | ordinary smooth planner 能做到的部分 |
| `J_slosh` | 惩罚预测晃液状态 | 本文核心新增部分 |

段落模板：

```text
The key difference from ordinary MPCC local planners is the inclusion of the sloshing modal dynamics and sloshing cost. The planner therefore optimizes not only where the robot should move, but also how the planned motion will excite the liquid over the prediction horizon.
```

中文：

> 与普通 MPCC 局部规划器相比，SPMPC 的关键差异在于 OCP 中包含晃液模态动力学和晃液代价。因此，规划器优化的不仅是机器人应该走哪里，还包括这段未来运动将如何激发容器内液体。

段末收束句：

```text
Thus, SPMPC turns liquid sloshing from an after-the-fact evaluation metric into a predicted state that participates in local planning.
```

中文：

> 因此，SPMPC 将液体晃动从事后评价指标，变成了参与局部规划的预测状态。

---

### 9.4 Receding-Horizon Implementation and Command Generation

段落模板：

```text
At each control cycle, the OCP is solved in a receding-horizon fashion. Only the first control input is applied to the robot. The optimized linear acceleration and angular acceleration are converted to velocity commands by forward integration. The remaining predicted trajectory and liquid states are used for diagnostics and warm start in the next cycle.
```

中文：

> 每个控制周期内，SPMPC 以滚动时域方式求解 OCP，并只执行第一帧控制。优化得到的线加速度和角加速度通过前向积分转换为线速度和角速度命令，其余预测轨迹和液体状态用于诊断和下一周期 warm start。

注意边界：

- 不要说 `slosh_constraint_enable` 已经实现硬约束；
- 不要说 guaranteed spill-free；
- 可以说 `slosh-aware cost`、`predicted sloshing state`、`reduced liquid motion`。

---

## 10. Experiments 叙事框架

Jian et al. 的实验强在模块拆解式 baseline。SPMPC 也应这样做。

实验不应只回答：

```text
ours 是否比某个 planner 好？
```

而应回答：

```text
Q1: 只平滑控制是否足够防晃？
Q2: 显式 slosh model 是否有效？
Q3: slosh-only 是否足够？
Q4: SPMPC 是否能在保持路径跟踪和效率的同时降低真实液面晃动？
Q5: 普通 online local planner 是否因为缺少液体状态而表现不足？
```

---

### 10.1 内部消融实验

回答：

> SPMPC 内部每个模块是否有用？

| Method | Slosh model/cost | Smooth shaping | Role |
|---|---:|---:|---|
| **B0** | No | Weak | ordinary alpha-state MPCC |
| **B_smooth** | No | Strong | smooth-only ablation |
| **B_slosh** | Yes | Weak | slosh-only ablation |
| **B_ours** | Yes | Strong | full SPMPC |

关键比较：

| Comparison | 证明什么 |
|---|---|
| `B_ours vs B_smooth` | slosh-aware 不是普通平滑 |
| `B_ours vs B_slosh` | smooth shaping 对最终效果必要 |
| `B_smooth vs B0` | 平滑本身有收益 |
| `B_slosh vs B0` | 液体模型和液体代价本身有贡献 |

英文说明：

```text
To evaluate the role of each component, we first conduct same-framework ablation studies. B0 removes both the sloshing cost and the enhanced smoothness shaping. B_smooth keeps the smoothness emphasis but does not propagate liquid states. B_slosh embeds the sloshing model without enhanced smoothness shaping. B_ours combines sloshing prediction and smooth command generation.
```

中文：

> 为评估每个模块的作用，首先进行同框架消融实验。B0 不包含晃液代价也不强调平滑；B_smooth 只加强平滑但不传播液体状态；B_slosh 引入晃液模型但不加强平滑整形；B_ours 同时包含晃液预测和平滑控制。

---

### 10.2 外部 local planner 对比

回答：

> 普通导航栈方法是否足够？

| Method | Type | Slosh-aware? | Role |
|---|---|---:|---|
| RPP / Pure Pursuit | path tracking | No | lightweight tracking baseline |
| DWA / LT-DWA | velocity-space local planner | No | classic kinodynamic baseline |
| TEB / MPC local planner | optimization-based local planner | No | stronger local-planner baseline |
| B_ours | slosh-aware MPCC | Yes | proposed method |

不要将这组和内部消融混成一张总排名表。二者回答的问题不同：

| 实验组 | 回答问题 |
|---|---|
| 内部消融 | SPMPC 的 slosh prediction 和 smooth shaping 是否有贡献 |
| 外部 planner 对比 | 普通 non-slosh-aware local planner 是否足够 |

---

### 10.3 评价指标

主指标：

| 指标 | 含义 | 对应故事 |
|---|---|---|
| `Max-LCR / Max slosh height` | 最大液面晃动 | 是否防晃 |
| `RMS-LCR / mean liquid motion` | 平均晃动强度 | 是否整体更稳 |
| `Residual oscillation` | 到达后残余晃动 | 是否降低动态记忆残留 |
| `Travel time` | 完成时间 | 不是靠无限慢降低晃动 |
| `Mean / max contour error` | 路径跟踪 | 没有牺牲路径跟踪 |
| `Speed variance` | 速度平稳性 | 是否控制平滑 |
| `Acceleration / angular acceleration RMS` | 控制平滑 | 对应 smoothness |
| `Solver time` | 实时性 | 是否能在线运行 |
| `Success rate / timeout` | 完成任务能力 | 外部 baseline 比较 |

注意：

- `/spmpc/slosh_height` 是 model proxy，不能作为唯一真实评价；
- 实物实验应使用 RGB max-LCR 或其他外部液面观测作为主要液体评价指标；
- 不要用自己的模型输出直接评价 DWA/TEB 的真实液体晃动。

---

### 10.4 实验结果叙事模板

英文：

```text
B0 tracks the path with relatively aggressive velocity and turning changes, which excites large liquid oscillations. B_smooth reduces acceleration variations, but it cannot account for the current modal displacement and velocity of the liquid; therefore, it may still produce commands that amplify the future sloshing response. B_slosh explicitly penalizes predicted liquid states and reduces sloshing, but without sufficient smoothness regularization the generated commands can be less smooth. B_ours combines sloshing prediction and smooth command generation, achieving lower liquid motion while maintaining comparable travel time and tracking accuracy.
```

中文：

> B0 能够跟踪路径，但速度和转向变化较激进，容易激发较大液体振荡。B_smooth 降低了控制变化，但它不知道当前液体模态位移和速度，因此仍可能生成会放大后续晃动响应的命令。B_slosh 显式惩罚预测液体状态，可以降低晃动，但如果没有足够的平滑正则，控制命令可能不够平顺。B_ours 结合晃液预测和平滑控制，在保持相近完成时间和路径跟踪精度的同时，实现更低的液体晃动。

---

## 11. Discussion / Limitations 叙事框架

Discussion 的作用不是“给自己找借口”，而是让论文更可信。这里尤其要克制表述。

建议写三类边界：

### 11.1 低阶模型边界

```text
SPMPC uses a low-order modal model to obtain a real-time planning representation of the dominant sloshing motion. This model is suitable for online optimization but cannot capture all nonlinear free-surface phenomena such as wave breaking, impacts, or strong three-dimensional effects.
```

中文：

> SPMPC 使用低阶模态模型来获得适合实时规划的主导晃液表示。该模型适合在线优化，但不能覆盖所有非线性自由液面现象，例如破波、冲击或强三维效应。

### 11.2 评价边界

```text
The model-predicted slosh-height topic is used for diagnostics and planning interpretation, while external liquid-motion measurements are used to evaluate real liquid behavior in experiments.
```

中文：

> 模型预测的 slosh-height 话题用于诊断和解释规划行为，而真实实验中的液体晃动应通过外部液面观测指标评价。

### 11.3 安全表述边界

```text
The proposed method reduces predicted and measured liquid motion, but it does not provide a formal spill-free guarantee. Hard slosh or spilling constraints and closed-loop liquid-surface feedback are left for future work.
```

中文：

> 本文方法能够降低预测和实测液体晃动，但不提供形式化的防溢出保证。硬晃液约束、防溢出约束和真实液面闭环反馈可作为未来工作。

---

## 12. Conclusion 叙事框架

Conclusion 不要再展开所有细节，只要回到核心故事。

英文草稿：

```text
This paper presented SPMPC, a slosh-aware MPCC local planner for mobile robot liquid transportation. The main idea is to treat liquid sloshing as a predicted dynamic state in the local planning horizon, rather than as an after-the-fact smoothness or acceleration metric. By embedding a low-order sloshing modal model into a receding-horizon MPCC formulation, SPMPC jointly optimizes path tracking, progress, executable commands, smoothness, and predicted liquid response. Ablation and mobile-base liquid-transport experiments show that slosh-state prediction can reduce liquid motion beyond smooth-only planning while maintaining tracking and real-time performance. Future work will investigate closed-loop liquid-surface feedback, hard spilling constraints, and obstacle-aware slosh-aware navigation.
```

中文：

> 本文提出 SPMPC，一种面向移动机器人液体运输任务的晃液感知 MPCC 局部规划器。核心思想是将液体晃动视为局部规划时域内的预测动态状态，而不是事后的平滑性或加速度评价指标。通过将低阶晃液模态模型嵌入滚动时域 MPCC，SPMPC 在同一优化问题中联合考虑路径跟踪、路径进度、可执行控制、控制平滑性和预测液体响应。消融实验和移动底盘液体运输实验表明，晃液状态预测能够在 smooth-only 规划之外进一步降低液体晃动，同时保持路径跟踪和实时性。未来工作将探索真实液面反馈、硬防溢出约束和障碍物感知的晃液感知导航。

---

## 13. Claim–Gap–Evidence 矩阵

这个矩阵用于防止论文 claim 没有支撑。

| Claim | Related Work 支撑 | Method 支撑 | Experiment 支撑 | 可能审稿问题 |
|---|---|---|---|---|
| 液体晃动具有动态记忆，不能只看瞬时加速度 | A01、A02、A03 | `eta, eta_dot` state propagation | B_slosh vs B0；residual oscillation | 低阶模型是否足够准确？ |
| smooth-only 不等于 slosh-aware | A07、D13、B03 | `J_smooth` 与 `J_slosh` 分离 | B_ours vs B_smooth | smooth 权重调大是否就够？ |
| anti-slosh 和 local planner 两条线没有接起来 | A1-A4、B/C 类、E 类 | SPMPC = slosh-aware MPCC local planner | real-time `/cmd_vel` + external planner 对比 | B01/B11 是否已经解决？ |
| SPMPC 能在线生成可执行底盘命令 | E 类 local planner 背景 | receding-horizon OCP，first input as `/cmd_vel` | solver time、success rate | 实时性是否足够？ |
| SPMPC 降低晃液而不只是变慢 | A/B/D 文献 | 同时有 progress / tracking / slosh cost | LCR + travel time + contour error | 是否靠降低速度取胜？ |
| 模型 proxy 与真实液面评价应区分 | A08-A11 | `/spmpc/slosh_height` 仅诊断 | RGB max-LCR / external measurement | 是否用自己的模型评估自己？ |

---

## 14. 图和表的叙事功能

### Fig. 1：任务动机图

内容：

```text
移动机器人携带开口液体容器通过转弯路径；
ordinary planner 产生较大晃动；
SPMPC 通过预测液体状态降低晃动。
```

图注示例：

```text
Fig. 1. Slosh-aware mobile liquid transportation. Ordinary local planners may generate smooth and feasible commands, but without liquid-state prediction they can still excite sloshing. SPMPC predicts liquid states in the planning horizon and generates slosh-aware commands.
```

叙事功能：

> 用一张图把“普通平滑轨迹 ≠ 晃液感知轨迹”讲出来。

---

### Fig. 2：系统框架图

仿 Jian et al. 的系统框架图：

```text
Odom / optional IMU + Reference Path
        ↓
Slosh-State Propagation + Progress Projection
        ↓
Local Reference Construction
        ↓
Slosh-Aware MPCC OCP
        ↓
Local Trajectory + /cmd_vel
        ↓
Mobile Robot + Liquid Container
        ↓
External Liquid-Motion Evaluation
```

叙事功能：

> 证明 SPMPC 是一个接入机器人闭环的 local planner，而不是离线轨迹优化器。

---

### Fig. 3：SPMPC horizon 示意图

仿 Jian et al. 的 Fig. 3。

Jian et al. 画的是：

```text
未来障碍物椭圆序列 + 机器人预测轨迹 + CBF zone
```

SPMPC 可画成：

```text
参考路径 + 机器人预测轨迹 + 未来 slosh modal response / slosh height envelope
```

表达重点：

- 不同候选轨迹对液体模态的未来激励不同；
- SPMPC 在路径误差、进度、平滑性和晃液响应之间折中；
- 未来时域内 `eta_x, eta_y` 被显式惩罚。

叙事功能：

> 把抽象的 `eta` 状态变成读者能直观看懂的“未来液体响应”。

---

### Table I：Related Work 对比表

| Category | Representative works | Online local planner | Liquid state in planner | Standard mobile base | Main limitation relative to SPMPC |
|---|---|---:|---:|---:|---|
| Slosh modeling / input shaping | A01, A02, A03, A07 | No | Partial | Platform-independent | Modeling or command shaping, not local planning |
| Manipulator anti-slosh | C01-C07 | Usually no | Yes | No | Different platform/control inputs |
| Mobile-base path / velocity design | B02-B04, B10 | Limited | Partial | Yes | Predefined path/profile |
| Slosh-constrained trajectory optimization | B01, B11 | Limited / task-dependent | Yes | Yes/related | Not navigation-stack local planner |
| Active reducer / special platform | B05-B08 | Task-dependent | Yes | No/partial | Extra mechanism or different actuation |
| Ordinary local planner | D01-D13 | Yes | No | Yes | No liquid dynamic memory |
| **SPMPC** | Ours | **Yes** | **Yes** | **Yes** | — |

---

### Table II：内部消融实验表

| Method | Slosh model/cost | Smooth shaping | Max-LCR ↓ | RMS-LCR ↓ | Tracking error ↓ | Travel time ↓ | Solver time ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|
| B0 | No | Weak |  |  |  |  |  |
| B_smooth | No | Strong |  |  |  |  |  |
| B_slosh | Yes | Weak |  |  |  |  |  |
| B_ours | Yes | Strong |  |  |  |  |  |

叙事功能：

> 回答“是不是只要平滑就够？”和“slosh model 是否真的有贡献？”

---

### Table III：外部 local planner 对比表

| Method | Type | Slosh-aware? | Max-LCR ↓ | Travel time ↓ | Tracking error ↓ | Success rate ↑ |
|---|---|---:|---:|---:|---:|---:|
| RPP / Pure Pursuit | path tracking | No |  |  |  |  |
| DWA / LT-DWA | velocity-space local planner | No |  |  |  |  |
| TEB / MPC local planner | optimization-based local planner | No |  |  |  |  |
| B_ours | slosh-aware MPCC | Yes |  |  |  |  |

叙事功能：

> 回答“普通 online local planner 是否已经足够？”

---

## 15. 章节之间的桥接句

写论文时最容易散。建议在每章末尾加桥接句。

### Introduction → Related Work

```text
To clarify this gap, we review existing studies from four perspectives: sloshing models and input shaping, anti-sloshing planning on robotic platforms, mobile-base liquid transportation, and ordinary mobile robot local planning.
```

中文：

> 为明确这一缺口，下面从液体模型与输入整形、机器人平台防晃规划、移动底盘液体运输以及普通移动机器人局部规划四个角度回顾相关工作。

### Related Work → Method

```text
The literature indicates that liquid-aware motion generation and online local planning are often treated separately. We therefore formulate SPMPC to embed sloshing-state prediction directly into a receding-horizon MPCC local planner.
```

中文：

> 综上，已有文献表明液体感知运动生成与在线局部规划通常被分开处理。因此，本文构建 SPMPC，将晃液状态预测直接嵌入滚动时域 MPCC 局部规划器。

### Method → Experiments

```text
The formulation separates the contributions of path tracking, smooth command generation, and sloshing-state prediction. This structure naturally leads to the following ablation studies.
```

中文：

> 上述形式将路径跟踪、平滑控制生成和晃液状态预测的作用进行了分离，因此可以通过下面的消融实验分别验证各模块贡献。

### Experiments → Discussion

```text
The experiments show reduced liquid motion under the proposed planner, but the results should be interpreted within the modeling and sensing assumptions of the current implementation.
```

中文：

> 实验表明本文方法能够降低液体晃动，但该结论应放在当前模型和传感假设范围内理解。

### Discussion → Conclusion

```text
Despite these limitations, the results support the central claim that sloshing should be treated as a predicted dynamic state in online local planning.
```

中文：

> 尽管存在上述限制，实验结果仍支持本文核心观点：液体晃动应作为在线局部规划中的预测动态状态处理。

---

## 16. 表述边界

### 16.1 推荐使用

| 推荐表述 | 原因 |
|---|---|
| reduced sloshing | 稳妥，不承诺绝对安全 |
| slosh-aware | 准确表达方法特点 |
| liquid-friendly local planning | 适合 Introduction / Discussion |
| predicted sloshing states | 和方法一致 |
| model-predicted slosh-height proxy | 区分模型输出和真实测量 |
| external liquid-motion measurement | 强调实验评价独立性 |
| online receding-horizon local planning | 准确定位方法层级 |

### 16.2 谨慎使用或避免

| 表述 | 问题 |
|---|---|
| spill-free guarantee | 除非有硬约束和严格实验，否则不要说 |
| safety-critical liquid transport | 容易被要求形式化安全证明 |
| true slosh observer | 若无液面测量反馈，不要说 |
| complete obstacle-aware MPCC | 第一版未完成，不应宣称 |
| global optimality | 非凸 OCP 不应宣称 |
| closed-loop stability guarantee | 若无证明，不应宣称 |
| `/spmpc/slosh_height` as ground truth | 模型 proxy 不能当真实液面 |

---

## 17. 最终叙事原则

1. **不要说“我们用了 MPCC”，要说“我们把液体动态记忆放进 local planner”。**
2. **不要把 Related Work 写成平台流水账，要让每类文献服务一个 gap。**
3. **不要只和一个 baseline 比，要用消融回答审稿人的问题。**
4. **不要把 smoothness 说成敌人；smoothness 是有用 baseline，但它不是 slosh-state prediction。**
5. **不要过度声称 hard safety / no spilling guarantee；当前主线更适合说 slosh-aware prediction and reduced sloshing。**
6. **每一章都要回到一句话故事：liquid sloshing is a dynamic-state prediction problem, not merely a trajectory smoothness issue。**
7. **核心结构始终是：ordinary local planner online but not liquid-aware；anti-slosh methods liquid-aware but often not online local planners；SPMPC bridges them。**

---

## 18. 下一步任务

建议按这个顺序推进：

1. **整理 A 类移动底盘液体运输近邻文献矩阵**  
   重点区分 A1 固定路径、A2 速度剖面、A3 离线优化、A4 tracking/control、A5 online local planning gap。

2. **整理 D 类低阶模型与测量评价文献**  
   重点支撑 slosh modal model 和 RGB / LCR 外部评价。

3. **整理 E 类普通 local planner baseline**  
   明确哪些方法进入外部对比，哪些只作为 related work。

4. **建立 Claim–Gap–Evidence 矩阵**  
   每个 claim 必须对应文献支撑和实验支撑。

5. **写 Introduction 初稿**  
   建议直接按本文件第 7 节五段式写，不要从 Method 开始写。

6. **画 Fig. 2 系统框架图和 Fig. 3 horizon 示意图**  
   先把故事图画出来，再填公式，避免 Method 写散。

7. **把实验表格先搭出来**  
   Table II 和 Table III 的空表先放进论文，这样后续跑实验时知道每个实验要回答什么问题。

