# SPMPC Method 与实验主体组织思路

> 本文档只处理论文主体，即 Method 与 Experiments。它用于统一方法贡献、文献边界、消融定义、实验优先级和结果表述。Introduction、Related Work 与 Conclusion 应在主体稳定后反向改写。

## 0. 当前总判断

SPMPC 最稳固的研究问题是：给定一条几何可行参考路径，标准轮式移动底盘如何在在线滚动时域中联合决定路径进度、可执行底盘运动和预测液体响应。

论文的核心不在于首次使用低阶晃液模型，也不在于首次采用 MPC。已有工作已经覆盖低阶模态建模、移动底盘液体运输、在线防晃预测控制和 MPCC 路径进度优化。本文真正可能成立的贡献，是把这些要素放到标准轮式移动底盘的在线 path-progress MPCC 决策层中，使液体动态记忆直接参与局部运动选择。

当前证据只能稳固支撑一个核心方法贡献。预测参考整形模块可以成为第二贡献，但必须通过同参数消融。实验评价可以作为实证贡献，前提是补齐完整二维消融、外部液面观测、在线求解统计和至少一个同层外部基线。

## 1. 研究范围必须先锁定

### 1.1 当前问题定义

当前论文处理的是沿预先给定几何可行路径的在线滚动运动生成。输入包括当前底盘状态、路径进度、上一控制输入和传播得到的液体模态状态。输出是底盘可执行的线速度与角速度命令。

优化目标同时考虑路径误差、路径推进、控制幅值、控制变化和预测液体响应。每个控制周期只执行第一帧动作，随后重新求解。

### 1.2 当前不应扩大的范围

现有主 OCP 尚未明确包含 occupancy grid、动态障碍预测、碰撞走廊或车体多边形约束。因此，当前数据能够证明的是给定路径上的在线 path-progress motion generation，而不是完整的 dynamic-obstacle local navigation。

论文可以继续使用 local-planning layer 这一定位，但必须同时写清限定条件。若后续没有加入障碍约束和局部环境实验，不应使用 complete obstacle-aware local planner、autonomous navigation system 或 dynamic-environment planner 等更强表述。

### 1.3 建议固定的一句话定位

> SPMPC is an online slosh-aware path-progress planner for a standard wheeled mobile base that propagates low-order liquid states within an MPCC horizon and jointly optimizes path following, motion feasibility, control smoothness, and predicted liquid response.

这句话比“首次提出 slosh-aware MPC”更准确，也比“若干防晃工程模块”更集中。

## 2. 权威文献给出的创新边界

| 文献方向 | 代表文献 | 已有能力 | 对 SPMPC 的约束或支撑 |
|---|---|---|---|
| 低阶晃液建模 | A01、A02 | 用模态质量-弹簧-阻尼状态表达二维晃液，并映射到液面输出 | 支撑低阶状态传播；不能据此声称新模型 |
| 早期在线防晃预测控制 | A12 | 液体状态空间模型进入滚动 GPC，并有真实水实验 | 否定“首次 slosh-aware MPC” |
| 移动底盘固定路径与输入整形 | B02、B03、B04 | 对路径、速度剖面和输入进行预先整形，再做跟踪 | 支撑本文与 fixed-profile 方法的层级差异 |
| 移动机器人液体约束轨迹优化 | B01 | 将移动机器人与球摆动力学放入整段离线轨迹优化 | 是最重要的同任务近邻，但不是在线局部规划 |
| 规划加跟踪防晃控制 | B11 | 离线 time-optimal OCP，执行期采用鲁棒跟踪和 CBF | 说明“障碍加防晃”并非空白，差异仍在在线决策层 |
| 特殊移动平台 slosh-aware MPC | B12 | Kalman filter 与约束 MPC 跟踪给定楼梯任务轨迹 | 否定“首次移动平台在线防晃 MPC” |
| 在线液体应急停止 | C11 | 20 Hz 滚动 QP、低阶液体状态和真实液体急停 | 证明 smooth 或 jerk 限制不能替代液体状态；该文目前是预印本 |
| MPCC 路径进度优化 | D19 | contour/lag error、虚拟路径进度和实时滚动控制 | 支撑 MPCC backbone；不能声称首次使用 MPCC |
| 在线 Local MPCC | D20 | 路径进度、静态区域和动态障碍进入在线 MPCC | 是 local-planning 层级的重要参照 |
| 加速度空间局部规划 | D02 | 状态含 v、omega，控制为线/角加速度 | 说明 alpha-state 不是独立创新 |
| 内部动力学增强 MPC | D23 | 将脚轮内部状态加入 MPC，控制为 a、alpha | 支撑“内部动态前移到预测层”的方法类比 |
| Local planner benchmark | D21 | 安全、效率、平滑和计算性能评价 | 支撑普通 planner 指标，但不能替代液体指标 |

因此，论文必须避免以下表述：

- 首次将液体状态放入 MPC；
- 首次实现在线防晃预测控制；
- 首次使用 MPCC 或路径进度状态；
- 将 omega 作为状态、alpha 作为控制本身构成创新；
- 内部模型代理量等于真实液面高度；
- 当前方法提供形式化无溢出保证。

## 3. 方法贡献的最终分层

### 3.1 核心方法贡献

核心贡献只有一条：液体动态记忆增强的在线 MPCC 运动规划。

具体而言，方法将底盘与路径进度状态

\[
x_r=[p_x,p_y,\theta,v,s,\omega]^\top
\]

和液体模态状态

\[
x_l=[\eta_x,\dot\eta_x,\eta_y,\dot\eta_y]^\top
\]

组合为统一预测状态

\[
x=[x_r^\top,x_l^\top]^\top.
\]

液体状态不是求解后的诊断量。它在 horizon 内随候选控制传播，并通过 slosh-oriented cost 或约束影响优化结果。这样，几何误差相近但未来液体相位响应不同的候选运动可以被显式区分。

建议使用下面的贡献表述：

> We formulate an online slosh-aware MPCC problem for open-liquid transport on a standard wheeled mobile base by augmenting the robot and path-progress states with low-order liquid modal dynamics. The resulting receding-horizon planner jointly selects path progress and executable base motion while accounting for the predicted phase-dependent liquid response.

这一贡献需要三类证据同时支撑：方法公式中的增强状态和联合 OCP；保持平滑配置一致的 slosh-on/slosh-off 消融；外部 RGB 或液位观测中的同向改善。

### 3.2 alpha-state 的正确位置

当前主线使用

\[
u=[a,\alpha,v_s]^\top,\qquad \dot\omega=\alpha.
\]

这个设计合理，但不是独立创新。D02 与 D23 已经采用相同或近似的角速度状态和角加速度控制结构。

它在本文中的作用是改善角速度命令连续性，并为角加速度约束和控制变化惩罚提供明确变量。若横向激励近似为

\[
a_y\approx v\omega,
\]

则激励变化率满足近似关系

\[
\dot a_y\approx a\omega+v\alpha.
\]

因此，限制 alpha 可以影响横向激励变化率。它并不自动保证晃动降低。论文应把 alpha-state 写成内层动态设计，而不是第二个创新点。

### 3.3 条件性第二贡献：预测参考整形

当前 Slosh-risk Reference Governor 基于短时域液体预测，在有限候选集合中选择速度参考缩放因子，并对缩放变化率进行限制。它只修改进入 MPCC 的参考，不覆盖求解后的底盘命令。

从控制术语看，该模块更接近 soft predictive reference shaping。经典 reference governor 通常还讨论约束可容许集合、递归可行性或闭环约束满足。当前模块尚未达到这一理论强度。

内部文档可以继续使用 Slosh-risk Reference Governor 名称。投稿正文必须加入 reference governor 权威综述，并明确其软参考整形属性。若审稿风险较高，可以改称 Predictive Slosh-risk Reference Shaper。

只有满足以下条件，它才升级为第二方法贡献：

- 完成 `B_ours_noRG` 与 `B_ours_RG` 的同参数比较；
- 在高风险场景中，外部液面 p95 或 RMS 稳定下降；
- 收益不能完全由更长任务时间解释；
- saturated rate 与 beta 变化表明模块确实参与决策；
- 在线计算开销仍满足控制周期。

若结果不稳定，Governor 留在 Method 的扩展部分或附录。Introduction 不再将其列为主要贡献。

### 3.4 实证贡献

完整的实证贡献可以表述为：通过因子化内部消融、外部液面观测、普通局部规划器对比、模型敏感性分析和在线运行统计，识别显式液体状态预测相对于 smooth-only motion generation 的独立作用。

这一贡献尚未完全成立。当前实物数据只覆盖 `B0`、`B_smooth` 和 `B_slosh`，缺少 `B_ours`、Governor、外部 planner 和完整 runtime 结果。现阶段只能写 evaluation protocol 或 representative evidence，不能写 comprehensive validation。

### 3.5 当前不应列为贡献的内容

- alpha-state 本身；
- acados SQP-RTI 求解器的使用；
- RGB 液面提取工具，除非视觉算法本身形成独立方法；
- `H_model` 的话题映射或日志字段；
- 终端顺滑停车，除非完成独立消融并证明终端液面收益；
- 可选模型软约束，除非它进入主实验并有约束结果。

### 3.6 贡献状态总表

| 方法或证据项 | 当前定位 | 进入 Introduction 的条件 | 当前建议 |
|---|---|---|---|
| 液体动态记忆增强的在线 MPCC | 核心方法贡献 | 完整二维消融、外部液面证据和 runtime | 直接保留，优先补证据 |
| alpha-state 角动力学 | 内层建模与执行设计 | 不单独进入贡献列表 | 在 Method 中解释，不声称创新 |
| Predictive Slosh-risk Reference Shaping | 条件性第二方法贡献 | noRG/RG 结果稳定，且收益不只是减速 | 实验后决定主文或附录 |
| 终端顺滑停车 | 可选扩展 | 独立终端窗口消融支持 | 暂不进入主贡献 |
| 模型与外部液面分离的评价协议 | 实验方法贡献候选 | 完整实物、敏感性和外部 baseline | 当前写成评价原则 |
| acados 在线实现 | 工程可行性证据 | 实测 solve-time 和 deadline 数据 | 放在实现与实验，不算创新 |

## 4. Method 章节的推荐结构

### 4.1 Problem Definition and Scope

本节只定义问题。先写输入、输出、给定路径假设和滚动时域目标，再说明当前不处理完整动态障碍导航。

必须回答：

- 上游提供什么参考；
- 当前状态如何获得；
- 优化器决定什么；
- 第一帧动作如何执行；
- 液体状态为何不是事后指标。

不要在本节展开 ROS topic、脚本、日志或 solver code generation。

### 4.2 Robot-Liquid Augmented Prediction Model

这一节是核心。正文应保留底盘状态、控制输入、液体模态动力学、激励耦合和增强状态总式。

液体模型只需解释到足以支持在线预测。容器几何到液面高度的详细映射、连续到离散的完整推导和参数识别过程可以移到附录或补充材料。

建议用 Remark，不使用 Proposition：

> When the objective or constraints depend on the propagated liquid state, the augmented OCP can distinguish candidate motions that have similar geometric tracking errors but induce different phase-dependent liquid responses. A slosh-agnostic planner cannot represent this difference explicitly at the prediction level.

这个判断带有明确条件，因此不应包装成理论定理。

### 4.3 Slosh-Aware MPCC Formulation

正文保留 contour error、lag error、path-progress state、离散 OCP 和统一阶段代价：

\[
\ell_k=
q_c e_{c,k}^2+q_l e_{l,k}^2-q_pv_{s,k}
+q_v(v_k-v_{ref,k})^2
+\|u_k\|_R^2
+\|u_k-u_{k-1}\|_S^2
+\|x_{l,k}\|_{Q_s}^2.
\]

标准 MPCC 项不需要逐项做创新性解释。重点说明 slosh term 只有和液体状态传播同时存在时才具有前瞻意义。

路径推进项同样重要。没有它，控制器可能通过停止来降低晃动。论文应明确说明防晃收益必须和任务完成、路径误差及到达时间共同评价。

### 4.4 Predictive Speed-Reference Shaping

Governor 在正文中应压缩。保留候选缩放集合、风险评分、最大可行候选和变化率限制即可。

正文只证明已有的有限性质：缩放后参考不超过名义参考；候选集合非空时选择满足风险阈值的最大离散候选。不要把这两点扩写成递归可行性、稳定性或无溢出保证。

完整伪代码、阈值来源和所有诊断字段移到附录。

### 4.5 Receding-Horizon Implementation

这一节说明 acados SQP-RTI、horizon、采样周期、warm start、first-action execution 和失败处理。引用 acados 论文只能证明求解框架的能力，在线性必须由本系统的实测 solve-time 分布证明。

正文应报告实际 backend。不能只写“采用 acados”，还需确认实物试验没有退化到 stub 或其他后端。

### 4.6 Method 章节的删除项

- 过长的运行流程枚举；
- ROS 节点和 topic 名；
- 实验脚本顺序；
- 未用于主实验的模型软约束长推导；
- 把模型代理量与 RGB 反复比较的讨论；
- 消融结果和实验观察。

Method 负责定义方法。结果解释留给 Experiments。

## 5. 核心内部消融必须按二维因子设计

### 5.1 变体定义

所有内部变体必须使用相同路径、底盘约束、horizon、solver、初始条件和终止条件。Governor 默认关闭。

| 变体 | 液体状态与代价 | 平滑配置 | 解释 |
|---|---:|---:|---|
| `B0` | 关 | 弱 | 基础 alpha-state MPCC |
| `B_smooth` | 关 | 强 | smooth-only 基线 |
| `B_slosh` | 开 | 弱 | 弱平滑条件下的 slosh-aware 变体 |
| `B_ours` | 开 | 强 | 完整基础 SPMPC，不含 Governor |

### 5.2 干净的一因素比较

| 比较 | 唯一变化因素 | 能回答的问题 |
|---|---|---|
| `B0` vs. `B_smooth` | 平滑配置 | 不含液体模型时，增强平滑有何作用 |
| `B0` vs. `B_slosh` | 液体机制 | 弱平滑条件下，液体状态与代价有何增量作用 |
| `B_smooth` vs. `B_ours` | 液体机制 | 强平滑条件下，完整 SPMPC 是否超过 smooth-only |
| `B_slosh` vs. `B_ours` | 平滑配置 | 含液体模型时，增强平滑如何改变性能 |

`B_smooth` 与 `B_slosh` 位于二维矩阵的对角线，两项因素同时变化。这个比较很有叙事价值，因为它可以展示弱平滑的 slosh-aware 方法是否仍超过强平滑方法，但它不能单独估计液体状态的纯主效应。

当前论文把这一对比写成“分离平滑与状态预测”，表述过强。应改成“检验显式液体预测能否超过强 smooth-only 解释”。

### 5.3 当前实物证据的缺口

现有代表性实物表只有 `B0`、`B_smooth` 和 `B_slosh`，每种三次。它支持下面的有限结论：在所选固定路径与统计窗口内，`B_slosh` 相对 `B_smooth` 的模型代理量和 RGB p95/RMS 更低，且该结果不能由更低的角速度 p95 单独解释。

它尚不能证明完整 `B_ours` 的性能。缺少第四个单元后，二维设计无法估计强平滑条件下的液体机制效应，也无法判断平滑与液体机制是否存在交互。

因此，下一批实物实验的第一优先级是补齐 `B_ours`，而不是继续挑选更多 `B_slosh` 试次。

## 6. Governor 消融

### 6.1 唯一有效的比较

\[
B_{ours}^{noRG}\quad\text{vs.}\quad B_{ours}^{RG}.
\]

两组必须保持相同的 SPMPC 权重、名义速度参考、路径、底盘约束、终端策略和初始状态。只允许 Governor 开关变化。

### 6.2 场景设计

至少区分低风险与高风险场景。低风险场景检查 Governor 是否不必要地减速。高风险场景应包含连续转弯、较高名义速度或容易激发残余振荡的曲率变化。

若只在一个普通固定路径上比较，Governor 可能长期不触发，实验无法说明模块是否有效。

### 6.3 必报指标

- 外部液面 p95 与 RMS；
- 到达时间和平均进度速度；
- `beta_raw`、`beta_f` 和 saturated rate；
- 高风险窗口内的 Governor 激活比例；
- solve-time 增量；
- 路径误差和任务成功率。

评价时不能只看液面下降。若所有收益都来自持续低速，必须将其写成性能权衡。

## 7. 模型不确定性实验必须进入高优先级

A01 的二维低阶模型实验已经显示约 18% 至 19% 的峰值误差。C11 的参数偏差实验也表明，低阶液体参数误差会直接改变约束或风险预测表现。

SPMPC 当前主要传播模型状态，而不是使用真实液面闭环校正。因此，模型敏感性比继续扩充经典 planner 数量更重要。

建议在仿真中扫描：

| 因素 | 推荐范围 | 目的 |
|---|---:|---|
| 固有频率 | 名义值的 ±10%、±20% | 检查相位和共振位置偏差 |
| 阻尼比 | 名义值的 ±20%、±40% | 检查残余振荡衰减误差 |
| 输入增益 | 名义值的 ±10%、±20% | 检查模型响应幅值偏差 |
| 初始液体状态 | 零状态与非零残余状态 | 检查动态记忆初始化 |
| 执行延迟 | 名义值附近扰动 | 检查预测和真实命令错位 |

主要报告外部或高保真仿真液面指标、模型代理量偏差、任务完成和性能退化曲线。目标不是证明鲁棒稳定，而是说明方法对合理参数误差的敏感程度。

## 8. 外部局部规划器对比

### 8.1 基线层级

内部 `B0` 是最重要的 ordinary MPCC 基线，因为它与 SPMPC 共享路径进度、求解器、horizon 和底盘模型。它比任何外部 planner 更适合识别液体状态的独立作用。

外部主基线建议保留两个：一个可稳定复现的 ordinary MPC/MPCC local planner，以及 LT-DWA。前者用于同优化范式比较，后者用于强 kinodynamic planner 比较。

LT-DWA 不是轻量基线。它包含加速度空间长时域状态树、未来距离场和 EB-MPC 图优化，应描述为强 ordinary local planner。

DWA 与 TEB 可以作为补充基线。若论文篇幅有限，优先放附录或补充材料。

### 8.2 固定路径条件下的解释限制

若所有方法只运行固定路径、没有障碍，DWA、TEB 和 LT-DWA 的主要局部避障能力没有被使用。此时比较更接近命令生成和平滑性比较，不能泛化为完整导航性能排名。

如果要强化 local planner 主张，应增加至少一个局部障碍或曲率重规划场景。若不准备扩展环境，论文需要主动收窄措辞。

### 8.3 公平性要求

- 起点、终点和参考路径一致；
- 速度、角速度及加速度边界尽量一致；
- 到达与失败判据一致；
- 所有方法使用同一外部液面评价流程；
- 参数调节预算和调节依据需要记录；
- 不稳定或无法公平复现的方法不进入排名表。

## 9. 移动底盘防晃近邻方法

B01、B02、B03 和 B11 与本文任务接近，但决策层不同。它们主要用于界定 novelty，不必强行做同层主表。

只有在能够统一路径、容器、速度边界、输入接口和评价窗口时，才进行定量复现。否则采用结构化定性对比，明确在线或离线、是否滚动重规划、是否使用当前液体状态、是否输出标准底盘命令。

## 10. 实物实验设计

### 10.1 最小完整矩阵

| 优先级 | 实验组 | 最低重复数 | 作用 |
|---|---|---:|---|
| P0 | `B0`、`B_smooth`、`B_slosh`、`B_ours` | 每组至少 3 次 | 完成核心二维消融 |
| P0 | 同步记录 solve time、最终命令和 RGB | 随每次试验 | 支撑在线性与执行解释 |
| P1 | `B_ours_noRG`、`B_ours_RG` | 每组至少 3 次 | 决定 Governor 是否进入贡献列表 |
| P1 | 一个 ordinary MPC/MPCC 与 SPMPC | 每组至少 3 次 | 同层外部比较 |
| P1 | LT-DWA 与 SPMPC | 每组至少 3 次 | 强 kinodynamic baseline |
| P2 | 终端停车消融 | 每组至少 3 次 | 只评价到达后的残余晃动 |
| P2 | DWA、TEB | 每组至少 3 次 | 补充完整性 |

### 10.2 数据选择规则

论文统计表必须使用所有满足预先定义有效性条件的试次。不能在不同方法之间自由拼接最优 run 作为均值统计。

代表性时间序列可以选用接近该方法中位数的试次。选择规则要提前固定，例如按 RGB RMS 与组中位数的距离最小。异常试次只有在传感器失效、路径未加载、求解器未运行或录包损坏等明确条件下才排除。

试验日期不进入论文叙事。运行编号保留在内部数据索引中。

### 10.3 统计窗口

全程指标、10% 至 90% 路径进度窗口和到达后残余振荡窗口承担不同作用，不能混用。

- 10% 至 90% 窗口用于比较稳定运动阶段；
- 全程窗口用于任务完成和总体能量；
- 到达后固定窗口用于终端残余晃动；
- max 只作补充，主结论优先使用 p95 和 RMS。

## 11. 指标层级

### 11.1 一级证据：真实液体响应

真实液面结论只依赖外部 RGB 或液位观测。主指标使用 p95 和 RMS。若视觉尺度经过可靠标定，可报告毫米；否则使用归一化位移并说明口径。

### 11.2 二级证据：模型代理量

`H_model`、模态状态和预测 horizon peak 用于解释优化器行为。它们不能单独证明真实液体改善。

模型与视觉的组间趋势一致可以支持解释。逐帧精确一致、真实高度重建或形式化安全则需要更强传感与标定证据。

### 11.3 任务与跟踪

至少报告成功率、到达时间、路径投影误差 p95 和最终误差。液面降低若伴随明显变慢，必须明确给出代价。

### 11.4 控制行为

建议报告角速度 p95、线加速度 RMS、角加速度 RMS，以及命令变化总量。它们用于排除“只是更平滑”解释，不是液体性能主指标。

### 11.5 在线计算

报告 median、p95、max、deadline miss rate、solver failure 和控制频率。只报告平均时间不足以证明在线部署。

## 12. 主张到证据的映射

| 论文主张 | Method 中需要出现 | Experiments 中需要出现 | 当前状态 |
|---|---|---|---|
| 液体状态进入在线 MPCC 预测层 | 增强状态、液体动力学、联合 OCP | `B0/B_smooth/B_slosh/B_ours` 二维消融 | 公式已有，实物缺 `B_ours` |
| 提升不只是更平滑 | slosh term 与 smooth term 分开定义 | 两组一因素比较，加控制平滑指标 | 当前只有对角线代表性比较 |
| 真实液体响应改善 | 明确 `H_model` 不是测量 | RGB 或液位 p95/RMS | 已有小样本代表性证据 |
| 方法保持在线运行 | first-action execution、实际 solver backend | solve-time 分布和 deadline miss | 结果尚未进入主表 |
| Governor 提供额外收益 | soft reference shaping 定义 | noRG 与 RG 同参数消融 | 方法已有，结果不足 |
| 低阶模型具有可用鲁棒性 | 模型假设和适用范围 | 参数与初始状态敏感性 | 尚缺 |
| 普通 planner 不能替代液体预测 | 文献层级定位 | ordinary MPC/MPCC 与 LT-DWA | 候选已有，结果尚缺 |

## 13. Experiments 章节推荐结构

### 13.1 Experimental Setup and Evaluation Protocol

说明平台、容器、液体、路径、控制周期、solver backend、约束、公平性、有效试次标准和统计窗口。指标只定义一次。

### 13.2 Factorial Ablation of Slosh Prediction and Motion Smoothing

这是实验章主体。先给完整二维矩阵，再报告四个一因素比较。实物表与仿真表使用相同变体定义。

### 13.3 Model Sensitivity and Online Computation

把参数误差、初始状态误差和 solve-time 放在一起。它们共同回答低阶模型能否作为在线决策内核，而不是只在名义模型下得到好结果。

### 13.4 Comparison with Ordinary Online Motion Planners

主文只保留最相关且公平复现的基线。固定路径条件下避免写成导航排行榜。

### 13.5 Real-Robot Liquid-Response Validation

集中报告外部 RGB、任务完成、路径误差和模型趋势一致性。所有关于模型代理量不等于真实液面的说明在这里完成，不再开独立 Discussion 重复解释。

### 13.6 Predictive Reference Shaping

只有 Governor 结果稳定时保留为独立小节。否则移入附录，并从 Introduction 贡献列表删除。

## 14. 主文图表配置

| 图表 | 内容 | 状态 |
|---|---|---|
| Method Fig. 1 | 路径、底盘状态、液体状态、MPCC 和首帧命令关系 | 应保留 |
| Method Table 1 | 四个内部消融变体的精确定义 | 应保留 |
| Experiment Table 1 | 完整二维消融统计 | 当前缺 `B_ours` |
| Experiment Fig. 1 | smooth-only 与 slosh-aware 的代表性 RGB/模型时间序列 | 待整理 |
| Experiment Fig. 2 | 四变体液面、任务时间和路径误差对比 | 待补完整矩阵 |
| Experiment Table 2 | 模型敏感性和 runtime | 尚缺 |
| Experiment Table 3 | ordinary planner 对比 | 尚缺 |
| Experiment Fig. 3 | 实物帧序列或外部液面曲线 | 可从现有视频整理 |
| Governor Table/Fig. | noRG 与 RG 的收益和 beta 行为 | 结果稳定后决定去留 |

## 15. 当前论文能够与不能够得出的结论

### 15.1 当前能够写

当前代表性固定路径实物结果显示，`B_slosh` 相对 `B_smooth` 的模型代理量 p95/RMS 和 RGB p95/RMS 同向下降。`B_slosh` 的角速度 p95 并非最低，因此观察到的液体改善不能由更低角速度单独解释。

该结论限于当前路径、容器、液位、小样本和统计窗口。它是趋势证据。

### 15.2 当前不能写

- 完整 `B_ours` 已经优于所有基线；
- Governor 已经带来稳定收益；
- SPMPC 在动态障碍环境中优于普通 local planner；
- 低阶模型能够精确重建真实液面；
- 方法对容器参数变化具有鲁棒保证；
- 方法满足无溢出安全约束；
- 当前结果具有广泛统计显著性。

## 16. 贡献进入 Introduction 的判定门槛

### 核心贡献

直接保留。它由方法结构和已有代表性实物趋势共同支撑，但最终投稿前必须补齐 `B_ours` 和 runtime。

### Governor

只有 noRG/RG 结果在高风险场景中稳定改善外部液面，并且代价合理时保留为第二贡献。否则降级。

### 终端顺滑停车

它只解决到达附近的制动冲击和残余振荡。必须有独立终端窗口消融，才能成为扩展贡献。不要与全程液体状态预测混写。

### 实证贡献

完成完整二维消融、至少一个外部同层基线、模型敏感性和 runtime 后，可以写成第三贡献。缺少这些证据时，只写 evaluation protocol。

## 17. 下一步执行顺序

1. 冻结四个内部变体的唯一配置定义，确认除两个实验因子外没有隐藏差异。
2. 补齐 `B_ours` 实物试验，完成二维矩阵。
3. 整理四变体的 runtime、路径误差、控制平滑和外部 RGB 主表。
4. 完成低成本模型参数与初始状态敏感性仿真。
5. 运行 Governor 的 noRG/RG 高风险场景消融，决定其是否留在主文贡献列表。
6. 选择一个 ordinary MPC/MPCC 和 LT-DWA 做公平外部比较。
7. 按本文件重写 Method 与 Experiments。
8. 主体稳定后，再回写 Introduction、Related Work、Abstract 和 Conclusion。

## 18. 需要补入参考文献库的材料

当前 `references.bib` 已包含 A01、A02、A12、B01、B02、B03、B11、B12、D02、D19、D20 和 D21。还应补充：

- C11 Hynninen and Kyrki，作为最新 online slosh-aware MPC novelty guardrail，引用时注明其为预印本；
- D23 Arrizabalaga et al.，用于说明 alpha-state 与内部动力学增强 MPC 已有先例；
- Garone、Di Cairano、Kolmanovsky 的 reference and command governor 综述；
- acados 的正式方法论文，用于求解器背景，不用于替代本系统 runtime 证据。

## 19. 写作约束

段落长度不要机械一致。关键判断可以单独成短段。

句子也需要变化。定义句可以较长，结论句应更短。

避免连续使用相同句型。少写重复的“本文首先、其次、最后”。

不使用解释性破折号。优先使用句号、逗号、分号和冒号。

并列信息只有在确实需要对照时才使用列表。正文叙事优先依靠因果关系和实验逻辑推进。

Method 的语言保持克制。Experiments 更克制。

所有“证明”“保证”“显著优于”都必须有对应理论或统计证据。否则使用 evaluate、support、indicate、representative evidence 和 trend-level consistency。
