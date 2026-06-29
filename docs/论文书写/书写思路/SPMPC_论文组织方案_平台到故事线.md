# SPMPC 相关工作组织方案：从平台分类到论文故事线

创建目的：把“面向移动液体运输任务的晃液感知轨迹规划与局部规划方法”这个研究方向下的参考文献，整理成可直接服务论文写作的结构。

---

## 0. 核心判断

当前分组是合理的，但建议明确区分两件事：

1. **找论文 / 管理论文时**：可以按平台找，即移动底盘、机械臂 / SCARA、液罐车 / 车辆 / 船舶，再加模型与普通 local planner。
2. **论文写作时**：不能只按平台平铺，要把平台分类收束到本文 gap：普通移动底盘上的在线、滚动时域、slosh-aware local planner。

一句话定位：

> SPMPC 属于“移动底盘 / 服务机器人液体运输”中的“在线滚动防晃局部规划”，它借用低阶液体模型和真实评价方法，对比普通 local planner，并区别于已有的固定路径设计、速度剖面、离线轨迹优化和跟踪控制方法。

---

## 1. 最终推荐的论文故事线

建议 Related Work 不是简单写成“机械臂、底盘、罐车”三段，而是写成下面五段。

```text
1. Sloshing models, command shaping, and measurement for planning
   说明防晃规划需要什么液体模型、传统输入整形思路、真实液面评价方法。

2. Anti-sloshing motion planning and control on different robotic / vehicle platforms
   先讲机械臂 / SCARA 和罐车 / 车辆，说明防晃问题在不同平台上重要，但这些平台不是本文同层问题。

3. Mobile-base liquid transportation
   重点展开移动底盘 / 服务机器人液体运输，这是本文最核心近邻方向。

4. Ordinary mobile robot local planners
   说明普通 local planner 能在线、能平滑、能跟踪路径，但没有液体动态状态。

5. Gap and positioning of SPMPC
   收束到：在线滚动时域 + MPCC + slosh modal state + local planner。
```

---

## 2. 平台分类与写作角色

| 大类 | 文献范围 | 论文中的作用 | 是否主线 baseline |
|---|---|---|---|
| A. 移动底盘 / 服务机器人液体运输 | B01-B08, B10, B11 | 核心近邻，重点比较已有方法与 SPMPC 的差异 | 是，最重要 |
| B. 机械臂 / SCARA / 操作机器人液体防晃 | C01-C07 | 说明机器人液体防晃已有大量研究，但平台自由度和控制接口不同 | 否，主要 Related Work |
| C. 液罐车 / 车辆 / 船舶等载液平台 | B09, A05 | 说明移动载液系统中的晃动会影响安全和稳定性，但尺度与目标不同 | 否，背景对照 |
| D. 跨平台基础方法 | A01-A11, A07, D12, D13 | 支撑模型、输入整形、测量评价和低阶模型合理性 | 部分可作为方法依据 |
| E. 非液体感知移动机器人 local planner | D01-D13 | 外部 baseline，说明 smooth-only / ordinary planner 不等于 slosh-aware | 是，作为外部对照 |

---

## 3. A 类：移动底盘 / 服务机器人液体运输

这是论文最核心的近邻文献。建议在论文中把 A 类细分为 A1-A5。

| 子类 | 文献 | 方法特征 | 主要贡献 | 与 SPMPC 的差异 / gap 写法 |
|---|---|---|---|---|
| A1 固定路径 / 曲线路径设计 | B02, B04 | 预设路径、曲线路径、路径几何设计 | 说明路径几何、转弯半径和曲率会影响晃动 | 多数关注路径预设计或指定轨迹，不是导航栈中的在线局部规划 |
| A2 速度剖面 / input shaping | B03, B10, A07 | velocity profile, command preshaping, input shaping | 说明沿给定路径调速度可以降低液体激励 | 需要已知路径 / 频率 / 任务条件，难以直接处理在线局部规划中的状态记忆与重规划 |
| A3 离线防晃轨迹优化 / time-optimal planning | B01, B11 | slosh-constrained trajectory optimization, time-optimal motion planning | 显式将液体模型或防晃目标纳入轨迹优化 | 强近邻，但通常偏整段轨迹规划或任务级规划，不等价于实时 local planner |
| A4 给定轨迹后的防晃跟踪控制 / 特殊机构 | B05, B06, B07, B08 | tracking control, observer-based control, active vibration reducer, mecanum / omni platform | 说明控制器或额外机构可以抑制晃动 | 多数依赖特定平台、全向 / 麦轮运动学或主动抑振机构，和普通差速底盘 local planner 不同 |
| A5 在线滚动防晃局部规划 | SPMPC | receding horizon, MPCC, slosh modal state, local planner | 本文位置 | 本文要强调的 gap：普通移动底盘上在线生成下一段 slosh-aware 局部轨迹 |

### A 类核心写作句型

> Existing studies on mobile liquid transportation have investigated path design, velocity-profile shaping, anti-sloshing transfer control, active vibration suppression mechanisms, and slosh-constrained trajectory optimization. However, many of these approaches focus on predefined paths, task-level trajectory generation, specific mobile platforms, or additional mechanical devices. In contrast, this work targets online local planning for a differential-drive mobile base carrying an open liquid container.

中文版本：

> 已有移动底盘液体运输研究主要从路径设计、速度剖面、转运控制、主动抑振机构和液体约束轨迹优化等角度展开。但这些方法多关注预设路径、任务级轨迹生成、特定移动平台或额外机械机构。相比之下，本文关注普通差速移动底盘携带开口液体容器时的在线局部规划问题。

---

## 4. B 类：机械臂 / SCARA / 操作机器人液体防晃

| 子类 | 文献 | 写作角色 | 与 SPMPC 的区别 |
|---|---|---|---|
| B1 末端轨迹规划 / time-optimal trajectory | C01, C02, C03, C04 | 证明机器人液体防晃轨迹规划已有基础 | 机械臂可直接规划末端位姿，有更多自由度 |
| B2 防晃控制 / MPC / flatness / observer | C05, C06, C07 | 支撑显式液体建模、预测控制、flatness、observer 等方法背景 | 控制对象和输入变量与差速底盘 local planner 不同 |

### B 类核心写作句型

> Anti-sloshing manipulation has been studied for manipulators, SCARA robots, and service robots, where the end-effector or container trajectory can be directly optimized to reduce liquid motion. These works demonstrate the value of sloshing-aware planning, but their actuation structure and task formulation differ from local planning for a ground mobile base.

中文版本：

> 机械臂、SCARA 和服务机器人中的液体防晃研究表明，将液体模型显式纳入轨迹规划或控制具有价值。然而，这类平台通常直接规划末端或容器位姿，并拥有额外姿态自由度，与地面移动底盘的局部轨迹规划问题不同。

---

## 5. C 类：液罐车 / 车辆 / 船舶等载液平台

| 子类 | 文献 | 写作角色 | 与 SPMPC 的区别 |
|---|---|---|---|
| C1 载液车辆 / tank vehicle | B09, A05 | 说明车辆尺度载液系统中的晃动会影响稳定性和安全 | 控制目标偏车辆稳定性、悬架控制、侧倾抑制，不是服务机器人 local planner |
| C2 船舶 / 航天 / 大尺度载液系统 | 后续可补充 | broader background | 通常关注结构载荷、自由液面大幅非线性和耦合动力学 |

### C 类核心写作句型

> Vehicle-scale liquid sloshing studies mainly address stability, active suspension, lateral load transfer, and coupled vehicle-liquid dynamics. Although related in physical motivation, these works operate at a different scale and do not directly solve the online local planning problem for a mobile robot carrying a small open container.

中文版本：

> 车辆尺度的液体晃动研究主要关注车辆稳定性、主动悬架、侧向载荷转移以及车辆-液体耦合动力学。它们说明移动载液系统中晃动具有安全意义，但并不直接解决移动机器人携带小型开口容器时的在线局部规划问题。

---

## 6. D 类：跨平台基础方法

| 子类 | 文献 | 论文中回答的问题 | 写作建议 |
|---|---|---|---|
| D1 低阶液体模型 / 状态估计 | A01, A02, A03, A06 | SPMPC 为什么能在 planner 中使用 eta, eta_dot 这样的低阶模态状态？ | 精读 A01/A02/A03，A06 可作为 future work 或 modeling alternative |
| D2 输入整形 / 速度整形 | A07, D12, D13 | 为什么 smooth/profile baseline 有意义？为什么它还不等于 slosh-aware？ | A07 支撑传统 input shaping；D12/D13 支撑速度参数化和平滑轨迹生成 |
| D3 液面测量 / 真实评价 | A08, A09, A10, A11 | 如何证明真实液面晃动降低，而不是只看模型 proxy？ | 支撑 RGB / LCR / liquid level measurement 等评价方法 |
| D4 高保真仿真 | A04, A05 | 为什么不用 CFD / FEM 直接放进在线 planner？ | 用作“高保真但计算代价高”的背景，不写成大综述 |

### D 类核心写作句型

> Low-order sloshing models provide a tractable representation for planning and control, while high-fidelity CFD or FEM models are more suitable for offline analysis and validation. In this work, a low-order modal state is embedded in the online planner, and external visual or sensor-based measurements can be used for experimental evaluation.

中文版本：

> 低阶晃液模型为规划和控制提供了可实时计算的状态表示，而 CFD/FEM 等高保真模型更适合离线分析和验证。本文将低阶模态状态嵌入在线规划器，并通过外部视觉或传感评价真实液面晃动。

---

## 7. E 类：非液体感知移动机器人 local planner

| 子类 | 文献 | 论文中角色 | 与 SPMPC 的区别 |
|---|---|---|---|
| E1 经典路径跟踪 / local planner | D01, D02, D05 | 普通导航 / 路径跟踪 baseline | 可在线跟踪 / 避障 / 平滑，但无液体状态 |
| E2 MPC / MPPI / trajectory optimization planner | D03, D06, D07, D08, D09 | 方法机制背景与外部对照 | 有在线优化机制，但目标通常是风险、避障、轨迹平滑或非完整约束，不考虑 slosh dynamics |
| E3 jerk-limited / smooth trajectory generation | D12, D13 | 支撑 smooth-only baseline | 平滑速度 / 加速度 / jerk 不等于考虑液体动态记忆 |

### E 类核心写作句型

> Conventional local planners and trajectory optimization methods can generate feasible and smooth motions for mobile robots. However, they generally do not include liquid sloshing states, and therefore cannot reason about the dynamic memory of the liquid surface when choosing future control commands.

中文版本：

> 普通局部规划器和轨迹优化方法能够为移动机器人生成可行、平滑的运动，但通常不包含液体晃动状态，因此无法在选择未来控制指令时显式考虑液体表面的动态记忆。

---

## 8. 逐篇文献组织矩阵初稿

| ID | 建议写作分组 | 论文角色 | 精读优先级 | 需要重点摘取的信息 |
|---|---|---|---|---|
| A01 | D1 低阶模型 | 最直接模型支撑 | 核心精读 | 低阶模态 / 等效动力学、二维激励、状态形式 |
| A02 | D1 低阶模型 | slosh estimation 支撑 | 精读 | model-based estimation、状态预测思想 |
| A03 | D1 低阶模型 | robotics/automation 建模背景 | 精读 | robotics 中如何建模 slosh |
| A04 | D4 高保真仿真 | CFD/VOF 背景 | 泛读 | 高保真模型为何不适合在线 local planner |
| A05 | C / D4 车辆高保真 | 车辆尺度 + FEM 背景 | 泛读 | 复杂车辆运动中的液体晃动 |
| A06 | D1 learning/digital twin | future work / alternative model | 泛读 | 学习型 slosh predictor 可能性 |
| A07 | D2 input shaping | 传统防振 / 防晃基础 | 精读 | command preshaping 如何降低振动 |
| A08 | D3 测量评价 | 液体机器人测量参考 | 精读 | 传感器评价晃动幅值 |
| A09 | D3 视觉测量 | RGB/LCR 支撑 | 泛读 | machine vision liquid level detection |
| A10 | D3 视觉测量 | 透明容器测量背景 | 泛读 | meniscus / 图像特征测量 |
| A11 | D3 液位传感 | 非接触传感背景 | 泛读 | capacitive / non-contact liquid level detection |
| B01 | A3 离线轨迹优化 | 核心近邻 | 核心精读 | 是否在线、是否 receding horizon、液体约束如何进入优化 |
| B02 | A1 路径设计 | 核心近邻 | 核心精读 | path design + trace control，路径几何如何影响晃动 |
| B03 | A2 速度控制 | 核心近邻 | 核心精读 | velocity control / transfer control，能否作为 inspired baseline |
| B04 | A1 曲线路径设计 | 经典路径设计 | 精读 | curved path 与 transfer control |
| B05 | A4 主动抑振机构 | 机构对照 | 精读 | active vibration reducer，与普通底盘差异 |
| B06 | A4 主动抑振机构 / 全向平台 | 平台 / 机构对照 | 精读 | parallel linkage + omni/mobile platform |
| B07 | A4 麦轮液体转运 | 移动平台近邻 | 精读 | mecanum 平台控制策略，与差速平台差异 |
| B08 | A4 麦轮轨迹跟踪 | tracking/control 背景 | 泛读 | observer-based tracking，不是 local planning 主线 |
| B09 | C1 载液车辆 | tank vehicle 背景 | 泛读 | preview MPC / active suspension / lateral sloshing |
| B10 | A2 液位变化速度策略 | 历史 / profile 背景 | 泛读 | liquid-level change 对防晃策略影响 |
| B11 | A3 time-optimal + anti-slosh | 强近邻 | 精读 | time-optimal planning、disturbances、与在线 local planner 差异 |
| C01 | B1 SCARA time-optimal | manipulator 背景 | 精读 | time-optimal anti-sloshing trajectory planning |
| C02 | B1 slosh-free robot trajectory | 机器人轨迹优化代表 | 精读 | slosh-free trajectory optimization |
| C03 | B1 manipulating liquids | 经典 broader related work | 精读 | sloshing-free manipulation |
| C04 | B1 meal-assist robot | 应用动机 | 泛读 | spilling avoidance |
| C05 | B2 predictive control | MPC 防晃背景 | 精读 | BEM + predictive control |
| C06 | B2 flatness/ESO | alternative control | 精读 | flatness-based trajectory + tracking |
| C07 | B2 robust output feedback | broader related work | 泛读 | robust output feedback / reconfigurable robots |
| D01 | E1 path tracking | ordinary baseline | 精读 | path tracking，无液体状态 |
| D02 | E1 DWA local planner | external baseline | 核心精读 | kinodynamic local planning，无 slosh state |
| D03 | E2 differential-drive optimization | 方法背景 | 精读 | 差速机器人轨迹优化框架 |
| D04 | E3 terrain-aware planning | 非主线背景 | 泛读 | 第一版不重点写 |
| D05 | E1 path planning review | 术语背景 | 泛读 | mobile robot planning 术语 |
| D06 | E2 MPPI | MPC/MPPI 背景 | 精读 | online sampling / predictive control |
| D07 | E2 risk-aware MPPI | 普通 risk-aware 对照 | 泛读 | risk-aware 不等于 liquid-aware |
| D08 | E2 non-holonomic traj opt | 方法背景 | 泛读 | 非完整约束轨迹优化 |
| D09 | E2 data-driven predictive control | predictive control 背景 | 泛读 | 第一版可不重点写 |
| D10 | E3 DRL motion planning | learning planner 背景 | 泛读 | 不作为主 baseline |
| D11 | E3 fluid-inspired RL | 避免概念混淆 | 泛读 | fluid-inspired motion planning 不是液体运输防晃 |
| D12 | D2 / E3 TOPPRA | 速度参数化背景 | 精读 | time-optimal path parameterization |
| D13 | D2 / E3 jerk-limited trajectory | smooth-only 支撑 | 精读 | jerk-limited real-time trajectory generation |

---

## 9. Related Work 草稿结构

### 9.1 第一小节：Sloshing modeling and measurement for planning

写作目的：先说明本文不是做自由液面 CFD，而是为在线规划采用低阶模型，并使用外部液面测量做评价。

可用文献：A01, A02, A03, A04, A05, A07, A08-A11。

建议段落核心：

> Prior work has developed low-order sloshing models, model-based estimation methods, input-shaping techniques, and liquid-level measurement systems. These studies provide the modeling and evaluation basis for sloshing-aware planning. Compared with high-fidelity simulation methods, low-order modal models are more suitable for online optimization.

### 9.2 第二小节：Anti-sloshing planning and control on robotic and vehicle platforms

写作目的：讲机械臂 / SCARA / 车辆平台，证明问题重要，但不是本文同层 baseline。

可用文献：C01-C07, B09, A05。

建议段落核心：

> Anti-sloshing planning has been widely studied for manipulators, SCARA robots, and vehicle-scale liquid tanks. These platforms either exploit end-effector pose freedom or focus on vehicle stability and suspension control. Their control interfaces and task objectives differ from local planning for mobile-base liquid transport.

### 9.3 第三小节：Mobile-base liquid transportation

写作目的：重点比较 A 类，这是本文最核心相关工作。

可用文献：B01-B08, B10, B11。

建议段落核心：

> For mobile robots transporting liquid containers, existing methods include curved path design, velocity-profile shaping, anti-sloshing transfer control, active vibration suppression, mecanum / omni-platform control, and slosh-constrained trajectory optimization. However, these methods typically focus on predefined paths, offline or task-level trajectories, special mechanisms, or tracking control, rather than online receding-horizon local planning with liquid state prediction.

### 9.4 第四小节：Mobile robot local planners without liquid awareness

写作目的：说明普通 planner 作为 baseline 的必要性，以及为什么它们不解决液体动态记忆。

可用文献：D01-D13，重点 D01, D02, D03, D06, D12, D13。

建议段落核心：

> Conventional local planners, trajectory optimization frameworks, MPPI controllers, and jerk-limited trajectory generators can produce smooth and feasible mobile robot motions. Nevertheless, they usually omit liquid sloshing states and therefore cannot explicitly account for the dynamic memory of the carried liquid.

### 9.5 Gap 段：本文位置

建议直接落到：

> Motivated by these observations, this paper proposes a slosh-aware continuous MPCC local planner for mobile-base open-liquid transportation. The planner embeds low-order sloshing modal states into a receding-horizon OCP and jointly optimizes path tracking, progress, velocity commands, smoothness, and predicted liquid motion.

中文版本：

> 基于上述观察，本文提出一种面向移动底盘开口液体运输任务的晃液感知 continuous MPCC 局部规划器。该方法将低阶晃液模态状态嵌入滚动时域 OCP，在同一优化问题中联合优化路径跟踪、路径进度、速度指令、控制平滑性和预测液体晃动。

---

## 10. Claim-Gap-Evidence 初稿

| Claim | 文献支撑 | 实验 / 方法支撑 | 审稿人可能问什么 | 回答方向 |
|---|---|---|---|---|
| C1. 液体晃动不能只靠轨迹平滑完全解释 | A01-A03, A07 | B_smooth vs B_slosh vs B_ours | smooth trajectory 是否已经足够？ | 用实验说明 smooth-only 降低激励但没有 slosh state prediction |
| C2. 机械臂 / SCARA 防晃已有研究，但不是本文同层 baseline | C01-C07 | Related Work 对比 | 为什么不和机械臂方法直接比较？ | 平台自由度、控制输入和任务层级不同 |
| C3. 移动底盘防晃已有路径设计 / 速度控制 / 优化 / 机构方法，但缺少在线 slosh-aware local planner | B01-B08, B10, B11 | 本文 MPCC OCP | 你的 novelty 在哪里？ | 在线、滚动时域、局部规划、普通差速底盘、slosh modal state |
| C4. 普通 local planner 可以平滑和跟踪，但没有液体动态记忆 | D01-D13 | B0, B_smooth baseline | 为什么不直接用 DWA/TEB/MPPI/jerk-limited？ | 它们没有 eta/eta_dot，因此不能预测液体状态对未来控制的影响 |
| C5. SPMPC 的核心是把 slosh modal state 放进 OCP | A01-A03 + 方法章节 | B_ours | 是否只是“加权重调参”？ | 强调状态扩展 + horizon prediction + ablation |

---

## 11. 当前最应该先精读的 12 篇

1. B01：最关键移动底盘液体约束轨迹优化近邻。
2. B02：经典 WMR path design / trace control。
3. B03：经典 velocity control / transfer control。
4. B11：time-optimal motion planning + anti-sloshing under disturbances。
5. A01：低阶模态 / 等效动力学最直接模型支撑。
6. A02：model-based sloshing estimation。
7. A03：robotics and automation slosh modeling 背景。
8. A07：input shaping / command preshaping 基础。
9. D02：DWA 类 local planner baseline。
10. D03：差速机器人 trajectory optimization 背景。
11. D06：MPPI / predictive planning 背景。
12. D13：jerk-limited smooth trajectory generation，用于 smooth-only 对照。

---

## 12. 论文写作时的注意边界

1. 不要把机械臂 / SCARA 写成本文同层 baseline，它们是 broader related work。
2. 不要把液罐车 / 船舶写太重，它们主要是说明移动载液系统安全背景。
3. 不要把 E 类普通 local planner 写成液体防晃方法；它们是 external baseline。
4. 不要过早宣称“硬约束防溢出”，若当前实现主要是 slosh model / slosh cost，应表述为 slosh-aware model/cost/state prediction。
5. Related Work 的重心应落在 A 类移动底盘液体运输，尤其是 A1-A4 与 A5 的差异。

