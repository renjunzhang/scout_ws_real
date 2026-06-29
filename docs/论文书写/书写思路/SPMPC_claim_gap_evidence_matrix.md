# SPMPC Claim–Gap–Evidence Matrix

日期：2026-06-29  
用途：把 SPMPC 论文的核心 claim、文献 gap、代码/方法依据、实验支撑和审稿人可能质疑对应起来。该文件基于四组文献矩阵：A 液体建模/估计，B 移动底盘液体运输，C 机械臂/操作防晃，D 普通 local planner/MPC。

---

## 0. 论文主线

建议第一版论文主线固定为：

> Existing slosh-suppression studies demonstrate the value of explicit liquid modeling, and mobile-robot local planners provide mature online navigation and smoothing mechanisms. However, existing mobile-base liquid-transport methods are mostly based on fixed velocity profiles, path design, offline trajectory optimization, tracking control, or special mechanisms, while ordinary local planners do not model the dynamic memory of the liquid. We propose a slosh-aware alpha-state MPCC local planner that augments the mobile-robot state with liquid modal states and jointly optimizes path progress, chassis controls, and predicted slosh response in a receding-horizon problem for laboratory open-liquid transportation.

中文主线：

> 机械臂/专用机构防晃方法证明了显式液体建模的重要性；普通移动机器人 local planner 能在线规划和平滑控制；但已有移动底盘液体运输方法多偏固定路径、速度整形、离线轨迹优化、参考轨迹跟踪或特殊机构抑振。SPMPC 的定位是在轮式移动底盘局部规划控制层中显式加入液体模态状态，在滚动优化中联合考虑路径进度、底盘控制和液体预测响应。

---

## 1. Claim–Gap–Evidence 总表

| ID | Claim | 文献支撑 | 代码/方法支撑 | 实验支撑 | 审稿人可能质疑 | 应对方式 | 必要图表/结果 |
|---|---|---|---|---|---|---|---|
| C1 | 显式液体模态状态是有物理依据的，液面高度应作为输出/评价而非直接状态。 | A01/A02 低维 mass-spring-damper modal coordinates；A03/A04/A05 高保真模型作为背景。 | SPMPC slosh state `eta_x, eta_x_dot, eta_y, eta_y_dot`；输入为容器平动加速度，横向项由 `v omega` 产生。 | 模型预测 slosh proxy + RGB max-LCR 或外部观测对比。 | 简化模型是否过粗？为什么不用 CFD/FEM？ | 说明在线 local planner 需要低阶实时模型；高保真模型适合离线校准/验证。 | 模型框图；slosh state propagation 图；RGB 与 proxy 语义边界说明。 |
| C2 | 普通 smooth motion 不等于 slosh-aware motion。 | D01-D13 说明普通 local planner/trajectory generation 可平滑但无液体状态；A07 input shaping 背景。 | `B_smooth` 无 slosh state/cost；`B_ours` 有 slosh dynamics/cost；二者同框架可对比。 | `B_ours vs B_smooth`，速度/时间尽量匹配，比较 RGB max-LCR、peak/p95 slosh、success、tracking error。 | SPMPC 是否只是更慢/更保守？ | 使用 common limits、相同 fixed path、报告任务时间/平均速度；必要时做 matched-time 或 matched-vref 对比。 | 主消融表；速度曲线；RGB max-LCR 箱线图；slosh phase 示例。 |
| C3 | SPMPC 相对已有移动底盘液体运输方法的核心差异是在线 receding-horizon local planning，而不是固定 profile 或离线轨迹优化。 | B01/B07/B11 是近邻但多为离线或预规划+跟踪；B02-B04 是 path design / input shaping；B05/B06 是 special mechanism。 | 每周期 acados SQP-RTI 求解 MPCC；状态含路径进度 `s/v_s` 与 slosh states；输出首帧 `cmd_vel`。 | 在线 ROS 实验、solver time、closed-loop trajectory、fresh-sim/real-robot runs。 | 近邻方法是否也可在线化？ | 承认可以启发未来工作；本文比较的是已发表方法定位与本文实现层级。强调 SPMPC 是 ROS local planner/controller。 | Related work gap 表；solver time 统计；closed-loop轨迹图。 |
| C4 | 机械臂/SCARA/操作防晃不能直接作为 Scout Mini 同层 baseline。 | C01-C07：end-effector/joint trajectory、SCARA、专用 transfer system、reconfigurable robot controller。 | SPMPC 控制变量是 wheeled base `v, omega, a, alpha, v_s`，非机械臂末端/关节轨迹。 | 不作为主 baseline；只在 Related Work 中讨论。 | 为什么不和 Muchacho/Ferrari/Moriello 等直接比？ | 说明平台、执行器、任务层级、控制变量不同；它们是 inspiration 而非同层 local planner。 | Related Work 分类表；baseline selection rationale。 |
| C5 | alpha-state formulation 有助于限制转向激励并对齐底盘约束。 | D 组 local planner 支撑移动机器人平滑/约束重要性；A/B 组说明横向加速度会激励 slosh。 | 状态含 `omega`，控制含 `alpha=omega_dot`；OCP 内约束 `|alpha|<=alpha_max`；`a_y=v omega` 激励横向 slosh。 | 比较 alpha/omega/ay/jerk-like metrics；可加 direct-omega legacy 诊断或 alpha-state ablation。 | 是否只是把角速度限小？ | 保持相同 `omega_max`，报告 `alpha`、角速度变化率、tracking/time。强调是 OCP 内角加速度约束而非单纯输出 clamp。 | 角速度/角加速度曲线；lateral acceleration 与 slosh 指标。 |
| C6 | SPMPC 可在保持路径跟踪和到点能力的同时降低液面晃动。 | B 组显示液体运输需要同时考虑 transfer 与 damping；D 组提供普通 planner baseline。 | MPCC cost 包含 contour/lag、progress、control/smooth、slosh eta/eta_dot。 | Success rate、timeout、final error、tracking RMS/max、RGB max-LCR、task time。 | 降晃是否牺牲太多效率或跟踪？ | 同时报告效率和跟踪指标，不只报 slosh；失败按失败计入。 | 主结果表：success、time、tracking、max-LCR、solver time。 |
| C7 | `/spmpc/slosh_height` 是控制器内部模型 proxy，真实评价应以外部视觉/传感为主。 | A08/A09/A10/A11 支撑液面测量/视觉/传感；A01/A02 支撑 state-to-height mapping。 | SPMPC 发布模型预测 slosh diagnostics，但 README 已强调 proxy 语义。 | 实物/视频 RGB max-LCR 作为主评价；proxy 作为诊断。 | 模型 proxy 是否可作为真实液面证据？ | 明确不把 proxy 当真值；将其写为 predicted/model-based indicator。 | 指标定义表；RGB pipeline 图；proxy vs RGB 对比或边界说明。 |
| C8 | 第一版论文不宣称完整 obstacle-aware MPCC / homotopy / corridor 主方法。 | D 组说明普通 local planner 可处理避障，但本文第一版范围聚焦 fixed laboratory route 的 liquid-aware local planning。 | 当前代码 policy 禁止 `continuous_mpcc_acados` 开启 obstacle/corridor/homotopy。 | 实验采用 fixed global path / predefined safe route；可做无障碍或受控环境。 | 没有避障是否削弱 local planner 贡献？ | 定位为 fixed laboratory reference path 上的 slosh-aware local planning/control；把完整 obstacle-aware MPCC 留作 future work。 | Scope statement；limitations；future work。 |

---

## 2. Claim 细化说明

### 2.1 Claim C1：液体模态状态与液面高度映射

**推荐写法：**

> Following low-order sloshing models for liquid-filled containers under planar excitation, we represent the liquid response using modal coordinates and velocities rather than treating the free-surface height as a state variable. The maximum sloshing height is then used as a model-derived output or experimental metric.

**中文解释：**

SPMPC 的液体状态不是“液面高度”，而是：

\[
x_s = [\eta_x, \dot{\eta}_x, \eta_y, \dot{\eta}_y]^T
\]

液面高度或最大液面高度是由状态映射得到的输出。这一点由 A01/A02 直接支撑。

**对当前 SPMPC 代码的写法边界：**

- 可写：低维模态状态、平动加速度输入、预测液体响应、模型 proxy。
- 不应写：高保真 CFD/FEM 在线规划、真实液面高度直接作为状态。

---

### 2.2 Claim C2：smooth-only 不等于 slosh-aware

**推荐写法：**

> Smoothness penalties reduce abrupt robot motions, but they do not encode the phase-dependent residual dynamics of the liquid. Two equally smooth control sequences may interact differently with the current sloshing state. By propagating modal liquid states inside the prediction horizon, SPMPC can distinguish such cases.

**必须用实验支撑：**

主消融必须突出：

| Variant | Slosh dynamics/cost | Strong smooth shaping | Purpose |
|---|---:|---:|---|
| B0 | No | No | ordinary alpha-state MPCC |
| B_slosh | Yes | No | isolate slosh prediction/cost |
| B_smooth | No | Yes | isolate smooth-only effect |
| B_ours | Yes | Yes | final method |

最关键比较：

1. `B_ours vs B_smooth`：证明显式 slosh prediction 超越普通平滑；
2. `B_ours vs B_slosh`：证明 smooth shaping 仍有实际部署价值；
3. `B_smooth vs B0`：证明平滑本身有效；
4. `B_slosh vs B0`：证明 slosh cost 本身有效。

---

### 2.3 Claim C3：相对近邻移动底盘液体运输工作的 gap

B 组矩阵显示：

| 子类 | 代表文献 | 与 SPMPC 的差异 |
|---|---|---|
| Fixed path/profile / input shaping | B02/B03/B04 | 预先设计路径/速度，不是在线 rolling optimization |
| Offline slosh-constrained trajectory optimization | B01 | 整段轨迹优化，不是在线 local planner |
| Special mechanism / active vibration reducer | B05/B06 | 依赖额外机构，不是普通底盘速度控制 |
| Preplanned trajectory + robust tracking | B07/B11 | 生成参考再跟踪，不是 MPCC local planning |
| Vehicle / suspension control | B09 | 控制层级不同，不是机器人导航 |

**推荐写法：**

> Closest mobile-base liquid-transport studies often rely on precomputed paths, velocity profiles, input shaping, offline trajectory optimization, or mechanism-assisted damping. SPMPC instead embeds a slosh model into an online MPCC local planner and solves it repeatedly in closed-loop.

---

### 2.4 Claim C4：机械臂防晃是 inspiration，不是同层 baseline

**推荐写法：**

> Manipulator-based liquid handling methods show that slosh-aware trajectory generation is effective, but they usually optimize end-effector or joint trajectories and exploit manipulator-specific actuation or orientation redundancy. These settings differ from nonholonomic wheeled-base local planning, where the control variables are chassis acceleration, angular acceleration, and path progress.

**Baseline 口径：**

- 不把 C01-C07 放入主实验 baseline 表；
- 可在 Related Work 中提及它们支持显式液体建模和防晃轨迹优化的重要性；
- 如果审稿人问为什么不直接比，回答平台/变量/任务层级不一致。

---

### 2.5 Claim C5：alpha-state 的作用

**推荐写法：**

> We lift the angular velocity into the state and use angular acceleration as a control input. This allows the OCP to constrain steering-rate excitation directly, which is important because lateral liquid excitation is coupled to the base motion through the predicted lateral acceleration.

**实验指标建议：**

- `omega` peak / RMS；
- `alpha` peak / RMS；
- `v*omega` lateral acceleration proxy；
- RGB max-LCR；
- tracking RMS / max；
- task time。

注意：如果没有完整 alpha-state ablation，不要把它写成被充分实验证明的独立贡献。可以写为 formulation and deployment feature。

---

### 2.6 Claim C6：到点/跟踪/降晃三者都要报告

SPMPC 论文不能只报液体指标，否则会被质疑“是不是只是变慢”。建议主表至少包含：

| Metric | Meaning |
|---|---|
| success rate | 是否 60s 内到达终点，失败计入失败 |
| task time | 运输效率 |
| final error | 到点精度 |
| tracking RMS / max | 路径跟踪能力 |
| RGB max-LCR / p95-LCR | 真实液面评价 |
| model-predicted slosh proxy | 控制器内部预测诊断 |
| solver p95 / max ms | 实时性 |

---

## 3. 实验设计与 claim 对应

### 3.1 内部消融实验

| Experiment | Claim supported | Required variants | Main metrics |
|---|---|---|---|
| Slosh vs smooth ablation | C2/C6 | B0, B_slosh, B_smooth, B_ours | RGB max-LCR, time, tracking, success |
| Alpha-state diagnostics | C5 | B_ours + optional legacy/direct-omega diagnostic | omega/alpha/lateral acceleration, slosh |
| Model proxy boundary | C1/C7 | B_ours logs + RGB observation | proxy vs RGB qualitative/quantitative relation |

### 3.2 外部 local-planner 对比

| Baseline | Literature source | Claim supported | Fairness requirements |
|---|---|---|---|
| DWA | D group / classic local planner | C2/C6 | same path, same limits, same timeout |
| TEB | ordinary optimization-based local planner | C2/C6 | same path, footprint/limits aligned |
| LT-DWA | D02 | C2/C6 | common limits and fresh-sim protocol |
| mpc_local_planner | D group / ordinary MPC | C2/C6 | same constraints as close as possible |

### 3.3 Supplementary mobile-liquid comparison

| Inspired method | Literature source | Claim supported | Note |
|---|---|---|---|
| Hamaguchi-style fixed profile | B02/B03/B04 | C3 | Supplementary, not same local-planner layer |
| Lim-style offline trajectory optimization | B01 | C3 | Strong near-neighbor, but offline |
| TOPPRA / jerk-limited smooth profile | D12/D13 | C2 | Good smooth-only comparator |

---

## 4. Related Work 段落草稿骨架

### 4.1 Slosh modeling and estimation

> Low-order sloshing models have been widely used to estimate the liquid response under container motion. In particular, planar-excitation studies represent the fluid motion using modal coordinates driven by container translational accelerations and map these states to maximum sloshing height. High-fidelity CFD, SPH, FEM, and learning-based digital twins can capture richer free-surface phenomena, but their computational cost makes them less suitable as the inner model of an online local planner. Therefore, SPMPC adopts a low-dimensional modal representation that can be propagated inside an MPC horizon.

### 4.2 Robotic manipulation and transfer-system anti-slosh control

> Robotic manipulation studies have shown that explicit slosh modeling, trajectory optimization, predictive control, and observer-based tracking can effectively reduce liquid oscillations. However, these methods typically optimize end-effector or joint trajectories for manipulators, SCARA robots, or dedicated transfer mechanisms. They are not directly comparable to a wheeled mobile robot local planner whose control variables are chassis velocity, acceleration, angular acceleration, and path progress.

### 4.3 Mobile-base liquid transport

> Mobile-base liquid-transport studies are the closest to our problem. Existing works have investigated path design, velocity shaping, input shaping, offline slosh-constrained trajectory optimization, robust tracking, and mechanism-assisted damping. These methods demonstrate the importance of considering liquid dynamics, but they are often tied to precomputed references, fixed profiles, or special actuation mechanisms. In contrast, SPMPC performs online receding-horizon local planning and control with liquid modal states embedded in the optimization problem.

### 4.4 Mobile-robot local planning

> Classical and modern local planners, including pure pursuit variants, DWA-style methods, trajectory optimization, MPC/MPPI, and online trajectory generation, provide efficient solutions for path tracking, smoothing, obstacle avoidance, and risk-aware navigation. However, these planners generally reason over robot states, path geometry, obstacles, terrain, or high-order kinematic limits, but not over the internal dynamic state of a transported liquid. Thus, smooth mobile-robot motion is not necessarily slosh-aware motion.

---

## 5. 第一版论文 scope statement

建议在 Method 或 Discussion 中明确写：

> This work focuses on slosh-aware local planning and control along a predefined safe laboratory route. Full obstacle-aware MPCC, homotopy reasoning, corridor constraints, stochastic chance constraints, and formal closed-loop stability proofs are outside the scope of this first version.

中文：

> 本文第一版聚焦预定义安全实验室路径上的液体晃动感知局部规划控制。完整 obstacle-aware MPCC、homotopy/corridor、多模态不确定性机会约束和严格闭环稳定性证明不作为本文主贡献。
