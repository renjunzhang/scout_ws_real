# SPMPC Related Work Matrix

日期：2026-06-29
用途：把 Zotero/Obsidian 中 A/B/C/D 四组参考论文整理成论文写作矩阵，用于后续 Related Work、Introduction 和 claim-gap-evidence 设计。

新增层级标签：本矩阵显式区分 `slosh state in controller`、`liquid state propagated inside online planner`、`online local planner`、`path progress optimization` 和 `standard WMR baseline`。这样可以承认已有 slosh-aware MPC / predictive tracking control，同时保护 SPMPC 的 novelty：普通轮式移动底盘的 online MPCC local planning。

Obsidian 源目录：

```text
/data/a/Obsidian/科大云盘/obsidian/vaults/StudyVault/30-Projects/MPC/参考论文整理
```

已生成的分组矩阵：

- `A_液体晃动建模_估计_防晃控制/A_建模估计文献矩阵.md`
- `B_移动底盘液体运输_防晃路径设计/B_近邻文献矩阵.md`
- `C_机械臂_SCARA_机器人操作防晃/C_机械臂防晃文献矩阵.md`
- `D_普通移动机器人_local_planner_MPC_轨迹优化/D_local_planner文献矩阵.md`

已生成的图示对比位于：

```text
/data/a/Obsidian/科大云盘/obsidian/vaults/StudyVault/30-Projects/MPC/参考论文整理/assets/spmpc_comparison_figures
```

核心图示索引：

- `A01_A02_LowOrder_Model_to_SPMPC.png`：低阶晃液模型如何支撑 SPMPC horizon 内状态传播。
- `A12_C05_Predictive_Control_vs_SPMPC.png`：predictive/MPC anti-slosh control 已存在，SPMPC novelty 不应写成“首次把晃液状态放进 MPC”。
- `B01_Lim2024_vs_SPMPC.png`：移动机器人离线液体约束轨迹优化 vs online MPCC local planner。
- `B02_B03_Hamaguchi_Profile_vs_SPMPC.png`：fixed path/profile/input shaping vs online slosh-state propagation。
- `B11_TimeOptimal_AntiSlosh_vs_SPMPC.png`：time-optimal reference planning + tracking vs online local planner。
- `B12_Prabakaran2026_vs_SPMPC.png`：special-platform KF--MPC tracking/control vs standard WMR online MPCC local planning。
- `D_Ordinary_Local_Planners_vs_SPMPC.png`：ordinary local planners are online but not liquid-aware。
- `Ordinary_MPCC_vs_SPMPC.png`：ordinary MPCC backbone vs slosh-augmented MPCC。

---

## 0. 总体分类逻辑

SPMPC 论文的相关工作不应按“谁更先进”简单排序，而应按论文论证链组织：

| Group | 中文分类                                      | 主要作用                                      | 与 SPMPC 的关系                                             |
| ----- | --------------------------------------------- | --------------------------------------------- | ----------------------------------------------------------- |
| A     | 液体晃动建模 / 估计 / 防晃控制                | 支撑液体模态建模、液面高度映射、传感/评价     | Method support + evaluation support                         |
| B     | 移动底盘液体运输 / 防晃路径设计               | 最关键近邻文献，支撑 gap                      | 近邻 related work，少数可做 supplementary baseline          |
| C     | 机械臂 / SCARA / 机器人操作防晃               | 证明机器人防晃已有价值，但平台/层级不同       | Related work / inspiration，不作为 Scout Mini 同层 baseline |
| D     | 普通移动机器人 local planner / MPC / 轨迹优化 | 支撑外部 baseline 与“smooth ≠ slosh-aware” | External baseline source + related work                     |

论文主线建议固定为：

> Robotic manipulation studies show the value of explicit slosh modeling, ordinary mobile-robot local planners provide mature online path-following and smoothing mechanisms, and mobile-base liquid-transport studies have explored path design, velocity shaping, offline optimization, and special mechanisms. Some existing methods already incorporate sloshing dynamics into predictive/MPC tracking controllers, but they do not directly address online slosh-aware MPCC local planning for a standard wheeled mobile robot following a fixed laboratory route. SPMPC fills this gap by augmenting MPCC with liquid modal states and optimizing path progress, chassis controls, and predicted slosh response in the same receding-horizon problem.

---

## 1. A 组：液体晃动建模 / 估计 / 防晃控制

| ID  | Paper / Method                                                                                                      | Type                                     | Model / Method                                                             | State / Measurement                                                 | Input excitation                             | Role in SPMPC paper        | Key support / Gap                                                                                                                        |
| --- | ------------------------------------------------------------------------------------------------------------------- | ---------------------------------------- | -------------------------------------------------------------------------- | ------------------------------------------------------------------- | -------------------------------------------- | -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| A01 | Sloshing Dynamics Estimation for Liquid-filled Containers under 2-Dimensional Excitation                            | Modal modeling / estimation              | 2D equivalent mass-spring-damper, linear + weakly nonlinear                | Modal coordinates and velocities; output is maximum sloshing height | Container translational accelerations in x/y | Core method support        | Directly supports SPMPC's low-dimensional modal state and acceleration-driven slosh dynamics.                                            |
| A02 | A Simple Model-Based Method for Sloshing Estimation in Liquid Transfer in Automatic Machines                        | Modal modeling / estimation              | Linear / weakly nonlinear mass-spring-damper, compared with pendulum model | Modal coordinate and velocity; output slosh height                  | Mainly 1D translational acceleration         | Method support             | Supports using simple ODE modal models for engineering slosh estimation instead of CFD in control loops.                                 |
| A03 | Modeling of liquid sloshing with application in robotics and automation                                             | High-fidelity simulation                 | SPH free-surface simulation                                                | Particle / free-surface / wall-force states                         | Container/wall motion                        | Related work               | Shows high-fidelity modeling exists; motivates reduced models for real-time planning.                                                    |
| A04 | Simulation of liquid sloshing in 2D containers using the volume of fluid method                                     | High-fidelity simulation                 | VOF CFD with liquid-solid coupling                                         | Velocity, pressure, interface fraction                              | Translational + rotational acceleration      | Related work               | CFD can represent complex fluid-solid coupling but is too heavy for online MPCC.                                                         |
| A05 | Nonlinear finite element analysis of liquid sloshing in complex vehicle motion scenarios                            | High-fidelity vehicle-coupled simulation | ANCF FEM fluid + multibody vehicle dynamics                                | FE fluid state, free surface, wheel loads                           | Braking, lane changes, curve negotiation     | Motivation / related work  | Shows slosh can affect vehicle stability; supports safety motivation.                                                                    |
| A06 | Physically sound, self-learning digital twins for sloshing fluids                                                   | Learning-based prediction                | Thermodynamics-informed data-driven reduced-order digital twin from video  | Latent ROM states / video-observed surface                          | Container motion + video                     | Related work / future work | Provides learning-based alternative to analytic modal models.                                                                            |
| A07 | Preshaping Command Inputs to Reduce System Vibration                                                                | Command shaping / vibration suppression  | Input shaping / preshaping                                                 | Residual vibration response                                         | Command input / acceleration profile         | Related work               | Supports traditional anti-vibration / anti-slosh methods, usually open-loop or offline profile shaping.                                  |
| A08 | Slosh Measuring Sensor System for Liquid-Carrying Robots                                                            | Sensing / measurement                    | ToF sensor array + 3D liquid surface reconstruction                        | Discrete liquid heights + reconstructed surface                     | Mobile robot motion                          | Evaluation support         | Supports the need and feasibility of measuring slosh on liquid-carrying robots.                                                          |
| A09 | Liquid Level Detection in Standard Capacity Measures with Machine Vision                                            | Vision liquid-level measurement          | Single-camera meniscus / scale detection                                   | Meniscus position and converted level                               | Image acquisition                            | Evaluation support         | Supports visual liquid-level reading as an experimental measurement path.                                                                |
| A10 | Efficient image feature-driven machine learning for meniscus volume measurement in transparent glass vials          | Vision + ML measurement                  | Optical analysis + image-feature ML                                        | Meniscus image features / volume                                    | Vial images                                  | Evaluation support         | Supports the need to compensate meniscus/refraction effects in RGB-based evaluation.                                                     |
| A11 | Capacitive and Non-Contact Liquid Level Detection Sensor Based on Interdigitated Electrodes with Flexible Substrate | Sensor measurement                       | Flexible IDT capacitive non-contact sensor                                 | Capacitance mapped to liquid level                                  | Liquid level / geometry                      | Evaluation support         | Alternative non-vision sensing route; not core method.                                                                                   |
| A12 | Sloshing Suppression Control by Using Model Predictive Control in Liquid Container Transfer System                  | MPC / predictive control                 | Model predictive control for liquid-container transfer                     | Slosh-related model states in transfer-control system               | Transfer-system motion                       | Novelty guardrail          | Shows slosh-aware predictive control existed before SPMPC; gap is online local planning with path-progress optimization on standard WMR. |

### A 组写作结论

- A01/A02 是 SPMPC 液体模态模型的主要支撑：状态是低维模态坐标，输入是容器平动加速度，液面高度是输出/映射量。
- A03/A04/A05 说明高保真流体模型存在，但更适合作离线分析/校准，不适合作在线 local planner 内核。
- A08/A09/A10/A11 支撑实验评价中使用外部液面观测，而不是把 `/spmpc/slosh_height` 直接写成真实液面高度。
- A12 支撑 novelty 边界：已有 slosh-aware MPC / predictive control，SPMPC 不应宣称“首次把晃液状态放进 MPC”。

---

## 2. B 组：移动底盘液体运输 / 防晃路径设计（近邻文献）

| ID  | Paper / Method                                                                                                                       | Platform                                           | Planning/control level                             | Liquid model used?                                   | Slosh state in controller?                          | Liquid state propagated inside online planner?                               | Online local planner? | Path progress optimization? | Standard WMR baseline?                    | Key gap vs SPMPC                                                                                                                                        |
| --- | ------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------- | -------------------------------------------------- | ---------------------------------------------------- | --------------------------------------------------- | ---------------------------------------------------------------------------- | --------------------- | --------------------------- | ----------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| B01 | 2D Trajectory Optimization Using Spherical Pendulum Dynamic Constraints for Reducing Liquid Sloshing on Mobile Robots                | 2D WMR carrying container                          | offline trajectory optimization                    | Yes, spherical pendulum                              | Yes, in offline optimizer                           | No; propagated in offline trajectory OCP, not an online local planner        | No                    | No                          | Partial; WMR but not online local planner | Optimizes an entire 2D trajectory offline; does not do online MPCC local planning or path-progress optimization.                                        |
| B02 | Path design and trace control of a wheeled mobile robot to damp liquid sloshing in a cylindrical container                           | WMR with cylindrical container                     | path design + tracking control                     | Yes, spherical-pendulum-type model                   | Partial, mainly for design/control law              | No                                                                           | No                    | No                          | Partial; WMR but fixed path/profile       | Relies on predesigned curvature/velocity pattern and trace control, not online slosh-aware planning.                                                    |
| B03 | Damping and transfer of liquid in cylindrical container using a wheeled mobile robot employing velocity control and path design      | WMR with cylindrical container                     | velocity profile / input shaping                   | Yes, spherical-pendulum-type model                   | Partial                                             | No                                                                           | No                    | No                          | Partial; WMR but profile/control baseline | Fixed-path and velocity shaping, not navigation-layer receding-horizon planning.                                                                        |
| B04 | Transfer Control and Curved Path Design for Cylindrical Liquid Container                                                             | Cart/container transfer                            | path design / transfer control                     | Yes, spherical-pendulum-type model                   | Partial                                             | No                                                                           | No                    | No                          | No                                        | Historical path/acceleration shaping; no online local planner.                                                                                          |
| B05 | Damping Control of Sloshing in Liquid Container in Cart With Active Vibration Reducer                                                | Cart + 6-DOF active vibration reducer              | special-mechanism tracking control                 | Yes, pendulum-type slosh model                       | Yes, in active damping controller                   | No                                                                           | No                    | No                          | No                                        | Performance comes from active container pose mechanism, not ordinary wheeled-base velocity planning.                                                    |
| B06 | Damping and Transfer Control System With Parallel Linkage Mechanism-Based Active Vibration Reducer for Omnidirectional Wheeled Robot | Omni-wheel robot + pneumatic parallel-link reducer | special-mechanism tracking control                 | Yes, pendulum surrogate                              | Yes / partial                                       | No                                                                           | No                    | No                          | No                                        | Uses additional active mechanism; not software-only local planning on standard base.                                                                    |
| B07 | Control Strategy for Liquid Transfer Using a Four-wheel Mecanum Mobile Robot Platform                                                | Four-wheel mecanum robot                           | special-platform tracking control                  | Yes, simplified slosh model                          | Yes, for tracking/control                           | No                                                                           | No                    | No                          | No; mecanum/special platform              | Preplanned rest-to-rest reference plus tracking, not online MPCC local planning.                                                                        |
| B08 | Trajectory tracking control of a four mecanum wheeled mobile platform                                                                | Mecanum platform                                   | tracking control                                   | No explicit liquid model                             | No                                                  | No                                                                           | No                    | No                          | No                                        | Platform control background only; no slosh model/objective.                                                                                             |
| B09 | Preview-based MPC for active suspension control of tank vehicle with lateral liquid sloshing suppression                             | Tank vehicle active suspension                     | special-platform MPC control                       | Yes, equivalent pendulum liquid-cargo model          | Yes, in suspension/roll MPC                         | No; MPC is for vehicle suspension/stability                                  | No                    | No                          | No                                        | Optimizes suspension and vehicle stability, not robot navigation/local planning.                                                                        |
| B10 | Liquid Container Transfer Considering the Suppression of Sloshing for the Change of Liquid Level                                     | Cart/container transfer system                     | velocity profile / transfer control                | Yes, rectangular-container slosh with changing level | Yes / partial                                       | No                                                                           | No                    | No                          | No                                        | Transfer-control level, not mobile-robot local planning.                                                                                                |
| B11 | Time-Optimal Motion Planning and Anti-Sloshing Control For a Container Under Disturbances                                            | Moving container / generic transport               | offline trajectory optimization + tracking control | Yes, nonlinear mass-spring-damper                    | Yes, in planning/tracking controller                | No; mostly reference planning + tracking                                     | No                    | No                          | No                                        | Strong near-neighbor, but still reference planning + tracking; not online MPCC local planner.                                                           |
| B12 | Slosh-Aware Trajectory Control in a Reconfigurable Staircase Service Robot                                                           | Reconfigurable staircase service robot / sTetro-SR | special-platform MPC tracking control              | Yes, simple pendulum dynamics                        | Yes, Kalman-filter MPC / output-feedback controller | No; liquid state is used in tracking controller, not an online local planner | No                    | No                          | No; special service robot                 | Confirms slosh-aware MPC tracking exists, but it is prescribed-trajectory/special-platform control rather than standard WMR online MPCC local planning. |

### B 组写作结论

B 组是最关键 gap 证据。结论是：已有移动底盘/移动容器液体运输研究确实很多，而且其中已经出现 slosh-aware MPC / predictive tracking control。它们大多属于：

1. fixed path / velocity profile / input shaping；
2. offline trajectory optimization；
3. preplanned rest-to-rest trajectory + tracking control；
4. active vibration reducer / special mechanism；
5. special-platform MPC/control or vehicle suspension control。

因此，B 组可支撑如下论文定位：

> Existing mobile-base and liquid-transfer studies have explored path design, velocity shaping, offline trajectory optimization, robust tracking, special mechanisms, and even slosh-aware predictive/MPC tracking control. In contrast, SPMPC targets online local planning: it embeds liquid modal dynamics into a receding-horizon MPCC formulation and jointly optimizes path progress, chassis controls, control smoothness, and predicted slosh response for a standard wheeled mobile robot.

---

## 3. C 组：机械臂 / SCARA / 机器人操作中的防晃

| ID  | Paper / Method                                                                                                                   | Platform                              | Online / Offline                                        | Method Type                                           | Liquid Model                         | Baseline?          | Role                              | Key difference vs SPMPC                                                         |
| --- | -------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------- | ------------------------------------------------------- | ----------------------------------------------------- | ------------------------------------ | ------------------ | --------------------------------- | ------------------------------------------------------------------------------- |
| C01 | Time-Optimal Anti-Sloshing Trajectory Planning for Multiple Liquid-Filled Containers Subject to SCARA Motion                     | SCARA / industrial manipulator        | Offline                                                 | Time-optimal trajectory optimization                  | Mass-spring-damper                   | No                 | Related work / inspiration        | Manipulator end-effector trajectory, not WMR local planning.                    |
| C02 | A Solution to Slosh-free Robot Trajectory Optimization                                                                           | 7-DoF Franka Panda                    | Near-real-time trajectory generation, not local planner | QP-based trajectory optimization                      | Spherical pendulum + point mass      | No                 | Related work / inspiration        | Optimizes liquid-carrying end-effector motion, not navigation decisions.        |
| C03 | Manipulating liquids with robots: A sloshing-free solution                                                                       | Industrial manipulator                | Online trajectory generation                            | Exponential-filter shaping + orientation compensation | Spherical pendulum                   | No                 | Related work                      | Uses manipulator orientation redundancy; not available to Scout Mini base.      |
| C04 | Trajectory planning for meal assist robot considering spilling avoidance                                                         | Meal-assist robot                     | Offline / task-specific                                 | CFD-assisted spilling-avoidance trajectory design     | CFD evaluation                       | No                 | Motivation / early related work   | Task-specific spoon/meal assist motion, not generic mobile-base local planning. |
| C05 | Sloshing Suppression Control by using Physical Boundary Element Model and Predictive Control in Liquid Container Transfer System | Dedicated transfer device             | Online control                                          | BEM-based GPC/MPC                                     | Boundary element physical model      | No                 | Method inspiration                | MPC appears, but object is a dedicated transfer mechanism.                      |
| C06 | Anti-sloshing control: Flatness-based trajectory planning and tracking control with an integrated extended state observer        | 2D container transfer/tracking system | Offline planning + online control                       | Flatness-based planning + LMPC + LESO                 | Mass-spring-damper                   | No                 | Related work / inspiration        | Given-reference anti-slosh tracking, not online obstacle/local planning.        |
| C07 | A robust output feedback strategy for liquid handling using reconfigurable robots                                                | Reconfigurable mobile robot           | Online control                                          | Robust output feedback + observer                     | Planar/spherical-pendulum-like slosh | No, not same layer | Related work, closest in platform | Controller design, not obstacle-aware local planner.                            |

### C 组写作结论

机械臂/操作类文献证明了显式液体模型和防晃轨迹优化的重要性，但平台和规划层级与 SPMPC 不同。它们主要优化：

- manipulator end-effector trajectory；
- joint trajectory；
- container orientation；
- task-specific transfer motion；
- dedicated transfer mechanisms。

因此 C 组适合作 Related Work / inspiration，不适合作 Scout Mini 同层 baseline。

---

## 4. D 组：普通移动机器人 local planner / MPC / 轨迹优化

| ID  | Paper / Method                                                                                                 | Planner Type                               | Online / Offline                   | State / Objective                                                            | Liquid Model?    | Baseline Role                        | Key limitation for liquid transport                                         |
| --- | -------------------------------------------------------------------------------------------------------------- | ------------------------------------------ | ---------------------------------- | ---------------------------------------------------------------------------- | ---------------- | ------------------------------------ | --------------------------------------------------------------------------- |
| D01 | Regulated Pure Pursuit for Robot Path Tracking                                                                 | Regulated pure pursuit local planner       | Online                             | Robot pose/path curvature; speed regulated by curvature/collision heuristics | No               | Direct external baseline candidate   | Smooth/safe path tracking, but no liquid state memory.                      |
| D02 | Long-Term Dynamic Window Approach for Kinodynamic Local Planning                                               | DWA + graph optimization local planner     | Online                             | Robot pose/velocity/acceleration and obstacle predictions                    | No               | Direct external baseline candidate   | Strong online local planner, but predicts robot/obstacle states only.       |
| D03 | Universal Trajectory Optimization Framework for Differential Drive Robot Class                                 | Differential-drive trajectory optimization | Online / real-time capable         | Linear/angular motion primitives, safety/efficiency objective                | No               | Supplementary external baseline      | Robot-centric trajectory optimization, no slosh dynamics.                   |
| D04 | Real-Time Multilevel Terrain-Aware Path Planning for Ground Mobile Robots                                      | Terrain-aware planner                      | Online/offline map compatible      | Terrain, stability, path smoothness                                          | No               | Not first-version baseline           | Terrain-aware but not slosh-aware; excluded from first-version scope.       |
| D05 | Path Planning for Autonomous Mobile Robots: A Review                                                           | Survey / taxonomy                          | Both                               | Taxonomy of path planning objectives                                         | No               | Background only                      | Mobile planning taxonomies generally omit payload fluid dynamics.           |
| D06 | Model Predictive Path Integral Control                                                                         | MPPI / sampling-based MPC                  | Online                             | System state/control, running and terminal costs                             | No by default    | External baseline family             | Can be made slosh-aware only if liquid states/costs are explicitly added.   |
| D07 | Dynamic Risk-Aware MPPI for Mobile Robots in Crowds                                                            | Risk-aware MPPI                            | Online                             | Robot state, obstacle uncertainty, collision risk                            | No               | Supplementary external baseline      | Risk-aware crowd navigation, not liquid internal dynamics.                  |
| D08 | Efficient Trajectory Planning for Multiple Non-Holonomic Mobile Robots via Prioritized Trajectory Optimization | Multi-robot trajectory optimization        | Offline-ish / online applicable    | Multi-robot trajectories, collision and nonholonomic constraints             | No               | Usually not first-version baseline   | Multi-robot coordination, not carried-liquid constraint.                    |
| D09 | Distributed Data-driven Predictive Control via Dissipative Behavior Synthesis                                  | Data-driven predictive control theory      | Online predictive control          | LTI subsystem data and dissipativity                                         | No               | Not baseline                         | General control theory, not mobile local planner.                           |
| D10 | Deep Reinforcement Learning for Motion Planning of Mobile Robots                                               | DRL motion planner                         | Online policy after training       | Robot state/reward/action                                                    | No               | Usually not baseline                 | Learned smooth motion, but no liquid memory unless added to state/reward.   |
| D11 | Actor-Critic RL for Reactive 3D Optimal Motion Planning Based on Fluid Dynamics                                | Fluid-dynamics-inspired RL planner         | Online/reactive after learning     | Potential-flow-like planning field                                           | No payload slosh | Not baseline                         | “Fluid” refers to planning analogy, not carried liquid.                   |
| D12 | Time-Optimal Path Parameterization based on Reachability Analysis                                              | TOPPRA / path parameterization             | Offline or fast reparameterization | Path position/speed/acceleration; time optimality                            | No               | Supplementary speed-profile baseline | Time-optimal velocity profile is not liquid-safe velocity profile.          |
| D13 | Jerk-limited Real-time Trajectory Generation with Arbitrary Target States                                      | Jerk-limited online trajectory generation  | Online                             | Position/velocity/acceleration under v/a/j limits                            | No               | Supplementary smooth-motion baseline | Jerk-limited smoothness does not predict slosh phase or residual vibration. |

### D 组写作结论

D 组支撑最重要的一句话：

> Smooth motion is not necessarily slosh-aware motion.

普通 local planner / MPC / MPPI / trajectory generation 已经可以做到平滑、安全、可行甚至风险感知的运动，但它们通常只建模：

- 机器人位姿、速度、加速度；
- 路径几何；
- 障碍物 / 人群风险；
- 地形 / 稳定性；
- jerk / acceleration limits。

它们通常没有液体状态：

\[
[\eta_x, \dot{\eta}_x, \eta_y, \dot{\eta}_y]^T
\]

因此不能区分“同样平滑但与残余晃动相位关系不同”的控制序列。

---

## 5. Baseline 推荐与论文角色

### 5.1 主实验 external local-planner baseline

| Baseline                                       | Role                                          | 原因                                                       |
| ---------------------------------------------- | --------------------------------------------- | ---------------------------------------------------------- |
| DWA / LT-DWA                                   | Classic / modern velocity-space local planner | 与局部规划层级一致，可对比在线导航与平滑运动；无液体模型。 |
| TEB                                            | Optimization-based local planner              | 常见 ROS local planner baseline；无液体动态记忆。          |
| mpc_local_planner / ordinary MPC local planner | MPC-style local planner baseline              | 最接近“普通 MPC local planner”，但无 slosh state/cost。  |
| Regulated Pure Pursuit                         | Tracking baseline / supplementary             | 可作为轻量路径跟踪 baseline，但不是完整局部优化器。        |

### 5.2 Supplementary / inspired liquid-transport baseline

| Baseline idea                                                  | Source      | Role                                                                                |
| -------------------------------------------------------------- | ----------- | ----------------------------------------------------------------------------------- |
| Hamaguchi-style fixed path + input-shaped velocity profile     | B02/B03/B04 | 传统 mobile-base anti-slosh inspired baseline。                                     |
| Lim-style offline slosh-constrained 2D trajectory optimization | B01         | 离线液体约束轨迹优化对照。                                                          |
| TOPPRA / jerk-limited profile                                  | D12/D13     | smooth-only / time-profile supplementary baseline，证明 smooth 不等于 slosh-aware。 |

### 5.3 不建议作为第一版实车同层 baseline

- 机械臂/SCARA 防晃方法：平台和决策变量不同；
- active vibration reducer / parallel linkage methods：依赖特殊机构；
- tank active suspension：控制层级不同；
- CFD/SPH/FEM/digital twin：模型层级不同，不是 local planner baseline；
- DRL / data-driven predictive control：除非有完整实现，否则只作为 related work。

---

## 6. Related Work 推荐结构

建议论文 Related Work 分为四小节：

1. **Slosh modeling and estimation for robotic liquid transport**用 A 组支撑低阶 modal model、传感/视觉评价与高保真模型取舍。
2. **Slosh suppression in robotic manipulation and transfer systems**用 C 组说明机械臂/专用机构防晃已有成果，但非 WMR local planner。
3. **Mobile-base liquid transport and anti-slosh path design**用 B 组建立核心 gap：现有移动底盘液体运输多为 fixed profile、offline optimization、tracking control 或 special mechanism。
4. **Local planning and trajectory optimization for mobile robots**
   用 D 组说明普通 local planner 成熟，但没有液体动态记忆，支撑 baseline 选择。

---

## 7. 论文中应避免的过度宣称

基于文献和当前代码状态，第一版论文不建议宣称：

- stochastic / covariance / chance-constrained MPC；
- CBF 是 SPMPC 主方法核心；
- 完整 obstacle-aware MPCC / corridor / homotopy 已实现；
- `/spmpc/slosh_height` 是真实液面高度；
- 机械臂防晃方法是 Scout Mini 同层 baseline；
- high-fidelity CFD/FEM 被纳入在线规划内核。

建议主方法宣称保持为：

> deterministic slosh-aware alpha-state MPCC local planner with modal slosh-state augmentation and real-time ROS/acados implementation.
