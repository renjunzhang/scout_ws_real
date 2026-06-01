# spmpc_local_planner 规控一体新主线方案

> 创建：2026-06-01  
> 目的：讨论是否在 `src/scout_apps/control` 下新建 `spmpc_local_planner`，把 SloshPriorityMPC 从当前“控制层 tracking MPC”推进到“规控一体 local MPC”新主线。  
> 结论：建议新建独立包，但必须分阶段落地；当前 `scout_local_planner` 保留为已验证控制器主线和论文实物对比链路。

## 1. 结论

建议新建：

```text
src/scout_apps/control/spmpc_local_planner/
```

不要继续把规控一体逻辑塞进 `scout_local_planner`。

原因很直接：

```text
scout_local_planner
  当前定位: fixed/global path tracking controller
  输入: reference path + v_ref
  输出: cmd_vel
  价值: 已有实物验证链路、C/D/E/F 消融、RGB 分析流程

spmpc_local_planner
  新定位: slosh-aware integrated local planner/controller
  输入: goal/global path/corridor/map + robot/slosh state
  输出: local trajectory + cmd_vel
  价值: 面向下一篇或升级版论文的规控一体主线
```

这等于重启一条干净主线。这样做比在旧包里继续加开关更清楚，也更容易让论文框图、代码结构和实验设计对齐。

## 2. 不建议直接改 scout_local_planner 的原因

当前 `scout_local_planner` 已经承担了这些职责：

```text
1. C / D / E / F 内部消融；
2. RPP-style / profile baseline 辅助实验；
3. terminal d200 终点逻辑；
4. /mpc/cost_breakdown、/slosh/*、/terminal/*、/reference/* 诊断；
5. 当前实物 fixed-path 验证方案。
```

如果继续往里面加入规控一体 MPC，会出现三个问题：

```text
1. 输入语义冲突
   旧逻辑吃 reference path，新逻辑要吃 corridor/map/goal 并生成 local trajectory。

2. cost 语义冲突
   旧 cost 是 tracking error + control + slosh；
   新 cost 需要 progress/guidance/corridor/obstacle/trajectory smoothness + slosh。

3. 实验风险变大
   当前控制器实物链路已经能跑。把规控一体改动混进去，会污染已有 C/D/E/F 数据口径。
```

所以正确做法是：

```text
scout_local_planner 保持稳定；
spmpc_local_planner 作为新包从零搭建；
两者共享 slosh model 和分析工具，但不共享控制主循环。
```

## 3. 新包目标

第一阶段目标不是一次性做完整 `src/mpc_planner` 级别系统，而是做一个最小可验证的规控一体 SPMPC。

### 3.1 输入

```text
/odom
/tf
/scout/global_path 或 /scout/global_path_fixed
/local_costmap 或 corridor mask   # 第一阶段可选，可先用无障碍 corridor
/slosh observer state             # eta / eta_dot
```

### 3.1.1 reference 来源与避障职责边界（唯一权威口径）

`§3.1` 的输入里同时列了 `global_path` 和 `global_path_fixed`，`§4.6`/实验 yaml 又有
`reference_source`。为避免口径漂移，这里把 reference 来源和"避障归谁"一次定死。后续文档、
launch、论文都以本节为准。

#### reference 来源按实验类型固定

```text
固定路径实验 (Route B 主因果):
  reference_source = global_path_fixed
  来源: template_fixed_path_generator 生成的固定 S 弯, 不经 MBF
  理由: 锁死几何, 干净归因; 与 Route A 同一条 P2 路径, 跨线可比。
  MBF 不介入。

点到点实验 (Route B 工程泛化):
  reference_source = global_path (MBF 输出) 或 goal
  来源: MBF 全局规划器, 已用 global costmap 避开静态障碍, 输出 collision-free 路径。
  定位: 工程泛化 / supplementary; 因 MBF 路径每次几何可能不同, 不作主因果。
```

#### 避障职责边界

```text
全局规划器 (MBF):    负责全局静态避障, 给出 collision-free 全局路径。
SPMPC (本方法):      负责在该路径附近, 生成 slosh-aware 的局部轨迹 + 第一步控制;
                     用 corridor 约束保持在路径管道内 (|e_contour| <= corridor_width/2)。
                     corridor 是"不越界", 不是"绕开障碍物"。

→ 因此第一/二版 SPMPC 本身不做障碍规避。静态避障由全局层负责;
  SPMPC 的价值在液体模态感知, 不在避障。
```

#### 分阶段与"不做"

```text
Phase 1-2:  无障碍 cost; obstacle_cost_terms 文件预留但 enable=false (见 §4.2.1)。
Phase 3:    才点亮 corridor / costmap obstacle penalty (静态, 软约束)。
明确不做:    动态障碍预测 / homotopy / topology 候选 (见 §8.3)。
```

#### 对比实验场景的公平性（关键）

```text
baseline mpc_planner (T-MPC++) 的核心卖点是 topology / 动态避障。
若对比场景含障碍, 等于拿"不避障的 SPMPC"比"专门避障的它", 不公平、归因也乱。

因此对比场景统一约定:
  - 无障碍 (P2 S 弯本身无障碍), 或
  - 静态障碍由全局规划器统一处理 (各方法吃同一条 collision-free 路径)。
  不让"是否会动态避障"成为实验变量。
  这样 mpc_planner 的避障能力用不上, 比较点回到"无障碍下谁晃得少", 才公平。
```

### 3.2 输出

```text
/spmpc/local_trajectory
/cmd_vel
/spmpc/status
/spmpc/cost_breakdown
/spmpc/slosh_horizon_summary
```

注意：不要复用 `/mpc/cost_breakdown` 话题，避免污染当前 `scout_local_planner` 分析脚本。新包用 `/spmpc/*` 命名空间。

### 3.3 优化变量

当前控制层 MPC 的核心是控制输入：

```text
u_k = [a_k, omega_k]
```

规控一体版本应显式优化局部轨迹状态：

```text
x_k = [p_x, p_y, theta, v, eta_x, dot_eta_x, eta_y, dot_eta_y, s]
u_k = [a, omega]
```

其中 `s` 是沿全局参考路径的进度。

### 3.4 目标函数

规控一体 cost 建议写成：

```text
J = J_progress
  + J_guidance
  + J_corridor
  + J_control
  + J_smooth
  + J_slosh
  + J_terminal
```

其中 slosh 项为：

$$
J_{\mathrm{slosh}}
=
\sum_{k=0}^{N}
\left[
Q_{\eta}(\eta_{x,k}^2+\eta_{y,k}^2)
+Q_{\dot{\eta}}(\dot{\eta}_{x,k}^2+\dot{\eta}_{y,k}^2)
\right]
$$

这和当前控制层 SPMPC 的 slosh cost 保持连续，但它不再只是“跟踪参考时避晃”，而是参与局部轨迹生成。

### 3.5 论文 Methods 必须补齐的数学闭环

当前 §3.3/§3.4 只是工程骨架。若要支撑论文 methods，必须至少补齐下面这些方程，不能只写 cost 名字。

#### 3.5.1 规控一体状态动力学

机器人平面动力学第一版按 unicycle + acceleration input 写：

$$
\dot p_x = v\cos\theta
$$

$$
\dot p_y = v\sin\theta
$$

$$
\dot \theta = \omega
$$

$$
\dot v = a
$$

路径进度 `s` 作为优化状态。最简单实现可先用：

$$
\dot s = v_s
$$

其中 `v_s` 可由 `v` 和航向误差近似得到；后续若使用 MPCC 形式，可引入独立进度输入：

$$
\dot s = \nu
$$

第一版代码中必须明确选择一种，不要在文档里混用。

#### 3.5.2 slosh 模态动力学

液体模态状态为：

$$
x_{\mathrm{slosh}}
=
\begin{bmatrix}
\eta_x & \dot\eta_x & \eta_y & \dot\eta_y
\end{bmatrix}^{\top}
$$

每个方向使用二阶模态模型：

$$
\ddot \eta_x
+ 2\zeta\omega_n\dot\eta_x
+ \omega_n^2\eta_x
=
-k_a a_x
$$

$$
\ddot \eta_y
+ 2\zeta\omega_n\dot\eta_y
+ \omega_n^2\eta_y
=
-k_a a_y
$$

其中 `a_x/a_y` 来自预测轨迹的车体系加速度。`omega_n`、`zeta`、`k_a` 的来源必须在论文里说明：

```text
omega_n:
  由容器半径、液深、重力和一阶模态公式得到。

zeta:
  由实物数据或 Ferrari-style 参数标定得到。

k_a:
  由当前 slosh_models 中的高度/模态转换系数得到。
```

这部分是本文方法成立的物理入口，不能只写“使用 slosh_models”。

#### 3.5.3 contour / lag / progress 定义

给定全局参考路径样条：

$$
r(s)=
\begin{bmatrix}
x_r(s) \\
y_r(s)
\end{bmatrix}
$$

切向和法向为：

$$
t(s)=
\frac{r'(s)}{\|r'(s)\|}
$$

$$
n(s)=
\begin{bmatrix}
-t_y(s) \\
t_x(s)
\end{bmatrix}
$$

机器人位置：

$$
p_k=
\begin{bmatrix}
p_{x,k} \\
p_{y,k}
\end{bmatrix}
$$

lag error 和 contour error 定义为：

$$
e_{\mathrm{lag},k}
=
t(s_k)^{\top}(p_k-r(s_k))
$$

$$
e_{\mathrm{contour},k}
=
n(s_k)^{\top}(p_k-r(s_k))
$$

对应 cost 至少写成：

$$
J_{\mathrm{guidance}}
=
\sum_{k=0}^{N}
\left(
Q_l e_{\mathrm{lag},k}^2
+
Q_c e_{\mathrm{contour},k}^2
\right)
$$

progress 项用于鼓励向前推进：

$$
J_{\mathrm{progress}}
=
-Q_s s_N
$$

或使用等价的参考进度跟踪形式。最终论文和代码只能保留一种主写法。

#### 3.5.4 控制、平滑和终端项

控制代价：

$$
J_{\mathrm{control}}
=
\sum_{k=0}^{N-1}
\left(
R_a a_k^2
+
R_{\omega}\omega_k^2
\right)
$$

平滑代价：

$$
J_{\mathrm{smooth}}
=
\sum_{k=1}^{N-1}
\left(
R_{\Delta a}(a_k-a_{k-1})^2
+
R_{\Delta\omega}(\omega_k-\omega_{k-1})^2
\right)
$$

终端项至少包含终点位置/进度和低速要求：

$$
J_{\mathrm{terminal}}
=
\|p_N-p_{\mathrm{goal}}\|_{Q_p}^2
+
Q_{v,T}v_N^2
$$

terminal 的实物停车诊断仍然单独统计，不进入 slosh 主效果窗口。

#### 3.5.5 cost 归一化口径

Route B 必须做 cost 归一化。规控一体 MPC 同时包含 path error、progress、control、smooth、corridor 和 slosh 项，量纲和数值尺度差别很大；其中 slosh 是 mm 级，裸数值通常在 `1e-6` 量级。如果只靠裸权重调参，`Q_slosh` 会失去可解释性，也不利于跨路径、跨容器、跨实验复现。

误差类项统一写成无量纲形式：

$$
J_i
=
\sum_k
Q_i
\left(
\frac{e_{i,k}}{e_{i,\mathrm{ref}}}
\right)^2
$$

推荐参考尺度：

| 项 | 误差 | 参考尺度 |
|---|---|---|
| contour | `e_contour` | `e_contour_ref = corridor_width / 2` |
| lag | `e_lag` | `e_lag_ref = v_nominal * dt` |
| velocity | `v - v_ref` 或 `v - v_nominal` | `v_ref_scale = v_max` |
| acceleration | `a` | `a_ref = a_max` |
| angular velocity | `omega` | `omega_ref = omega_max` |
| acceleration change | `a_k-a_{k-1}` | `da_ref = a_max * dt` 或实验固定 jerk 尺度 |
| angular-rate change | `omega_k-omega_{k-1}` | `domega_ref = omega_max * dt` 或实验固定角加速度尺度 |
| slosh height | `h_pred` 或 `h_coeff * eta` | `eta_ref = slosh_height_ref` |
| slosh modal velocity | `dot_eta` | `eta_dot_ref = omega_n * eta_ref` |

slosh 归一化沿用当前 `scout_local_planner` 的口径：

$$
J_{\mathrm{slosh},\eta}
=
\sum_k
Q_{\mathrm{slosh}}
\left(
\frac{h_{\mathrm{coeff}}\eta_k}{h_{\mathrm{ref}}}
\right)^2
$$

因此实现中等价于：

$$
Q_{\eta}
=
Q_{\mathrm{slosh}}
\frac{h_{\mathrm{coeff}}^2}{h_{\mathrm{ref}}^2}
$$

其中：

```text
h_ref = slosh_height_ref
默认沿用 Route A 的 0.005 m (5 mm)
```

模态速度项建议写成：

$$
J_{\mathrm{slosh},\dot\eta}
=
\sum_k
Q_{\mathrm{slosh}}
\rho_{\dot\eta}
\left(
\frac{\dot\eta_k}{\omega_n h_{\mathrm{ref}}/h_{\mathrm{coeff}}}
\right)^2
$$

工程实现可继续使用当前 Route A 的等价形式：

$$
Q_{\dot\eta}
=
\rho_{\dot\eta}
\frac{Q_{\eta}}{\omega_n^2}
$$

其中 `rho_dot_eta` 对应当前 `slosh_eta_dot_ratio`。

progress 项不要和误差类项混在一起归一化。它是“鼓励向前”的奖励项，而 contour/slosh/control 是“抑制偏差”的惩罚项。若 progress 权重过强，会产生抢进度、绕路或容忍大 contour error 的退化行为。

progress 推荐单独处理：

```text
1. 固定路径主因果实验:
   progress 权重保守，只保证完成任务；
   corridor/path deviation 约束负责防止绕路。

2. 点到点工程泛化实验:
   progress 可稍强，但必须同时报告 completion time 和 path deviation。

3. 论文表格:
   报告归一化后的无量纲 Q_i；
   不只报告裸 Q_eta / Q_contour。
```

#### 3.5.6 约束形式

输入和状态 bounds：

$$
v_{\min}\le v_k\le v_{\max}
$$

$$
a_{\min}\le a_k\le a_{\max}
$$

$$
|\omega_k|\le \omega_{\max}
$$

固定路径主实验中，corridor 用横向误差约束表达：

$$
|e_{\mathrm{contour},k}|\le w_{\mathrm{corridor}}
$$

点到点/障碍实验中，corridor 可写成线性不等式：

$$
A_k p_k \le b_k
$$

可选 slosh hard constraint 第一轮默认关闭，只预留形式：

$$
h_{\mathrm{pred},k}\le h_{\lim}
$$

论文主方法第一版应以 slosh soft cost 为主，不靠 hard constraint 制造效果。

#### 3.5.7 离散化和线性化

连续动力学统一写为：

$$
\dot x=f(x,u)
$$

离散化：

$$
x_{k+1}=F(x_k,u_k)
$$

第一版可用 Euler 或 RK4，但论文和代码必须一致。若后续使用 SQP/OSQP，需要在 nominal trajectory 附近线性化：

$$
\delta x_{k+1}
=
A_k\delta x_k
+
B_k\delta u_k
$$

其中：

$$
A_k=
\left.
\frac{\partial F}{\partial x}
\right|_{\bar x_k,\bar u_k}
$$

$$
B_k=
\left.
\frac{\partial F}{\partial u}
\right|_{\bar x_k,\bar u_k}
$$

这部分决定 `spmpc_solver` 是否只是 rollout sampling，还是 SQP/linearized QP。文档和代码必须同步。

## 4. 推荐文件结构

```text
src/scout_apps/control/
├── scout_local_planner/
│   └── 当前控制层主线: C/D/E/F + fixed-path 实物实验
│
├── spmpc_local_planner/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/spmpc_local_planner/
│   │   ├── spmpc_local_planner_ros.h
│   │   ├── spmpc_problem.h
│   │   ├── spmpc_state.h
│   │   ├── reference_corridor.h
│   │   ├── trajectory_rollout.h
│   │   ├── slosh_cost_terms.h
│   │   ├── obstacle_cost_terms.h
│   │   ├── terminal_policy.h
│   │   └── diagnostics_publisher.h
│   ├── src/
│   │   ├── spmpc_local_planner_ros.cpp
│   │   ├── spmpc_problem.cpp
│   │   ├── reference_corridor.cpp
│   │   ├── trajectory_rollout.cpp
│   │   ├── slosh_cost_terms.cpp
│   │   ├── obstacle_cost_terms.cpp
│   │   ├── terminal_policy.cpp
│   │   └── diagnostics_publisher.cpp
│   ├── config/
│   │   ├── spmpc_default.yaml
│   │   └── spmpc_baseline.yaml
│   ├── launch/
│   │   ├── spmpc_experiment.launch
│   │   └── spmpc_sim.launch
│   ├── scripts/
│   │   ├── run_spmpc_smoke.sh
│   │   └── validate_spmpc_bag.py
│   └── README.md
│
└── slosh_models/
    └── 共享物理模型库，后续可逐步抽成真正公共库
```

第一阶段不要把所有东西抽象成插件。先让 `spmpc_local_planner` 最小闭环跑通，再考虑模块化扩展。

### 4.1 实验驱动的最终代码结构

上面的结构是最小草图。真正开工时建议按“实验组可切换、模块可诊断、主线不污染”的原则组织：

```text
spmpc_local_planner/
├── include/spmpc_local_planner/
│   ├── ros/
│   │   ├── spmpc_local_planner_ros.h       # ROS 适配层，只负责订阅/发布/参数读取
│   │   └── diagnostics_publisher.h         # /spmpc/* 诊断发布
│   │
│   ├── core/
│   │   ├── spmpc_problem.h                 # 一次 MPC 问题的输入/输出/权重集合
│   │   ├── spmpc_solver.h                  # solver 接口，第一版可只接一个实现
│   │   ├── spmpc_state.h                   # robot + optional slosh state
│   │   ├── trajectory.h                    # local trajectory 数据结构
│   │   └── variant_config.h                # B0/B-slosh/B-smooth/B-ours 派生配置
│   │
│   ├── reference/
│   │   ├── reference_path.h                # global/fixed path 缓存
│   │   ├── progress_projector.h            # 最近点、s 投影、contour/lag 误差
│   │   └── corridor_builder.h              # fixed corridor / costmap corridor
│   │
│   ├── dynamics/
│   │   ├── robot_dynamics.h                # [x,y,psi,v,s] rollout
│   │   └── slosh_dynamics.h                # [eta,dot_eta] rollout
│   │
│   ├── costs/
│   │   ├── cost_terms.h                    # cost term 统一接口
│   │   ├── contour_cost.h
│   │   ├── lag_cost.h
│   │   ├── progress_cost.h
│   │   ├── control_cost.h
│   │   ├── smooth_cost.h
│   │   ├── terminal_cost.h
│   │   ├── corridor_cost.h
│   │   ├── obstacle_cost.h
│   │   └── slosh_cost.h
│   │
│   ├── constraints/
│   │   ├── bounds_constraint.h
│   │   ├── corridor_constraint.h
│   │   ├── obstacle_constraint.h
│   │   └── slosh_constraint.h              # 第一阶段只预留，默认关闭
│   │
│   └── terminal/
│       └── terminal_policy.h               # 终点判断与低 jerk 停车策略
│
├── src/
│   └── 与 include 对应的实现文件
│
├── config/
│   ├── spmpc_common.yaml                   # 机器人、horizon、topic、诊断通用配置
│   ├── variants.yaml                       # B0/B-slosh/B-smooth/B-ours 权重
│   ├── fixed_path_experiment.yaml          # 固定路径实验参数
│   ├── point_to_point_experiment.yaml      # 点到点实验参数
│   └── slosh_model.yaml                    # 容器、液体、阻尼、频率参数
│
├── launch/
│   ├── spmpc_experiment.launch
│   ├── spmpc_fixed_path.launch
│   └── spmpc_point_to_point.launch
│
├── scripts/
│   ├── run_spmpc_fixed_path_smoke.sh
│   ├── run_spmpc_p2p_smoke.sh
│   ├── record_spmpc_experiment.sh
│   ├── validate_spmpc_bag.py
│   └── analyze_spmpc_cost_breakdown.py
│
└── README.md
```

### 4.2 模块边界

必须保持下面的边界，避免重蹈 `scout_local_planner` 后期功能堆叠的问题：

| 模块 | 只负责 | 禁止 |
|---|---|---|
| `ros/` | ROS topic、launch 参数、diagnostics | 写 MPC 数学逻辑 |
| `core/` | MPC 问题组织、variant 派生、solver 调用 | 直接订阅 ROS topic |
| `reference/` | path 缓存、s 投影、corridor 生成 | 根据 slosh 修改 global path |
| `dynamics/` | robot/slosh rollout | 读参数服务器或发布 topic |
| `costs/` | 每个 cost term 的数值计算 | 做实验组判断 |
| `constraints/` | bounds/corridor/obstacle/slosh 约束 | 改权重或切 variant |
| `terminal/` | reached/capture/stop 策略 | 评价 slosh 效果 |
| `diagnostics_publisher` | 发布结构化诊断 | 反向影响控制 |

核心原则：

```text
costs 定义能力；
variants 定义实验组；
ros 只做胶水；
diagnostics 不参与控制决策。
```

#### 4.2.1 依赖方向（三圈，里圈绝不依赖外圈）

模块边界表说了"谁负责什么"，这里钉死"谁能依赖谁"。依赖只能从外向里，禁止反向：

```text
外圈  ROS adapter（唯一允许 #include <ros/ros.h> 的层）
        spmpc_local_planner_ros.{h,cpp}      订阅/发布/定时器/组装输入→调内圈→发布
        diagnostics_publisher.{h,cpp}        只发结构化 topic，不回写控制
          │ 依赖
          ▼
中圈  纯算法（禁止 #include <ros/ros.h>，可脱 ROS 单测）
        spmpc_problem.{h,cpp}      唯一编排者：组 cost+约束+调 OSQP→出 trajectory
        spmpc_state.h              9 维状态 struct + index
        reference_corridor.{h,cpp} 路径投影 / contour-lag / corridor 生成
        trajectory_rollout.{h,cpp} 动力学积分 + 线性化 A_k/B_k
        slosh_cost_terms.{h,cpp}   slosh ODE + J_slosh
        obstacle_cost_terms.{h,cpp} corridor / 障碍约束（Phase 3 才点亮）
        terminal_policy.{h,cpp}    终点 cost / 收敛
          │ 依赖
          ▼
内圈  复用既有（只读，不反向耦合）
        slosh_models               ω_n / ζ / k_a 模态 ODE（Route A/B 同口径）
        PathHandler 几何工具         样条 / s 投影 / kappa（只读，不依赖其控制循环）
```

四条硬规则（比文件清单更重要，违反就失去解耦）：

```text
规则 1  中圈零 ROS 依赖
  spmpc_problem / reference_corridor / trajectory_rollout / slosh_cost_terms /
  obstacle_cost_terms / terminal_policy 内不得出现 #include <ros/ros.h>。
  输入用纯 struct，输出用纯 struct。→ 能脱 ROS 单测，也是论文可复现的底气。

规则 2  spmpc_problem 是唯一编排者
  ROS 层只调它一个 solve(input)->output。各 cost_terms 互不引用，
  只被 spmpc_problem 按 variant 开关组装进 QP。

规则 3  每个 cost term 是"可独立开关的纯函数对象"
  slosh_cost_terms / obstacle_cost_terms / terminal_policy 各自 addTo(QP, weights, enable)。
  enable=false 就完全不进 QP（不是进了再乘 0）。→ 直接支撑 §4.3 的 B0/B-slosh/
  B-smooth/B-ours 消融：换组只改 enable，不改代码。

规则 4  variant 派生集中一处
  planner_variant=B0/B_slosh/B_smooth/B_ours → 派生 slosh_enable/smooth_priority_enable/...
  由一个 deriveVariant() 说了算，启动期 validate（对齐 Route A 的 configureExperimentVariant）。
  cost / dynamics / reference 层都不许自己判断"现在是哪个实验组"。
```

落地顺序（结构一次建全，功能分阶段点亮，对齐 §6 Phase）：

```text
Phase 1  ros + spmpc_problem + reference_corridor + trajectory_rollout + terminal_policy
         slosh_cost_terms / obstacle_cost_terms 文件建好但 enable=false 空实现
Phase 2  点亮 slosh_cost_terms（复用 slosh_models）
Phase 3  点亮 obstacle_cost_terms（corridor）
```

#### 4.2.2 命名空间口径（两条线互斥启动，单发布者）

前提：`scout_local_planner`（Route A）与 `spmpc_local_planner`（Route B）**同一时刻只启一个**，
实验时二选一。因此 `/cmd_vel` 不存在双发布者冲突，命名空间只需保证"诊断不串台"。

```text
节点         /spmpc/spmpc_local_planner   （ns="spmpc"）

诊断输出（私有，相对名 → 自动加 /spmpc 前缀）：
  /spmpc/local_trajectory
  /spmpc/cost_breakdown
  /spmpc/status
  /spmpc/slosh_horizon_summary

输入/输出（绝对名，全局契约，不进 ns）：
  sub  /odom                                   传感器栈契约（launch_real_sensors_stack.sh 提供）
  sub  /scout/global_path 或 /scout/global_path_fixed   与 Route A 共用同一路径源
  pub  /cmd_vel                                底盘契约
```

理由：

```text
1. 诊断全部私有 /spmpc/* → 与 Route A 的 /mpc/*、/slosh/* 物理隔离，
   bag 不串台、分析脚本不混；这是 §3.2 "独立命名空间"的落地方式。
2. /odom、/cmd_vel、/scout/global_path* 是全局契约，必须绝对名，
   否则接不上传感器栈和底盘。
3. cmd_vel topic 仍做成 arg（default /cmd_vel），仅用于调试期临时改成
   /spmpc/cmd_vel 看轨迹而不驱动底盘；正式跑用 /cmd_vel。
```

实物启动顺序（复用现有传感器栈，不重写）：

```text
1. scripts/launch_real_sensors_stack.sh   传感器/底盘/定位/IMU/RealSense（共用，不动）
2. spmpc_experiment.launch                只起 spmpc planner 节点（ns=spmpc）
   - load spmpc_common.yaml + variants.yaml + 实验 yaml
   - arg: planner_variant / experiment_mode / reference_source / cmd_vel_topic
   - <node ns="spmpc"> + 绝对 topic remap
```

与 Route A 的 `slosh_experiment.launch` 完全平行：传感器栈不变，只换第 2 步的 planner launch。

### 4.3 对比实验组先固化

`spmpc_local_planner` 的第一版就按以下 4 个内部组设计，不要后面再临时拼参数：

```text
B0:
  普通 integrated MPC
  state = [x,y,psi,v,s]
  cost = contour + lag + progress/v + control + smooth + terminal
  slosh_enable = false

B-slosh:
  B0 + slosh state + slosh cost
  不额外强化 smooth

B-smooth:
  B0 + 强 smooth / ax / jerk 相关项
  slosh_enable = false

B-ours:
  B0 + slosh state + slosh cost + 强 smooth
  作为主方法
```

外部 baseline：

```text
TEB:
  点到点主外部 baseline

mpc_local_planner:
  点到点主外部 MPC baseline；
  ROS1 move_base/base_local_planner 插件；
  比 DWA 更接近 integrated MPC 论文主线。

DWA:
  点到点 classic lower bound；
  可放 supplementary，不作为第一优先主表方法。

src/mpc_planner:
  可选。能跑就作为 supplementary / sim-only baseline；
  跑不顺就只作为结构参考和 related work。
```

主实验：

```text
固定路径:
  B0 / B-slosh / B-smooth / B-ours

点到点:
  B0 / B-ours / TEB / mpc_local_planner
  DWA supplementary
  mpc_planner optional
```

### 4.4 参数文件设计

不要把所有参数堆进 launch。launch 只暴露实验必要入口，其余写 YAML。

#### `spmpc_common.yaml`

```yaml
spmpc:
  control_frequency: 20.0
  horizon_steps: 30
  dt: 0.1

  topics:
    odom: /odom
    global_path: /scout/global_path
    fixed_path: /scout/global_path_fixed
    cmd_vel: /cmd_vel

  robot:
    v_min: 0.0
    v_max: 0.8
    a_min: -0.8
    a_max: 0.6
    omega_max: 1.2

  diagnostics:
    namespace: /spmpc
    publish_local_trajectory: true
    publish_cost_breakdown: true
```

#### `variants.yaml`

```yaml
variants:
  B0:
    slosh_enable: false
    smooth_priority_enable: false
    slosh_constraint_enable: false

  B_slosh:
    slosh_enable: true
    smooth_priority_enable: false
    slosh_constraint_enable: false

  B_smooth:
    slosh_enable: false
    smooth_priority_enable: true
    slosh_constraint_enable: false

  B_ours:
    slosh_enable: true
    smooth_priority_enable: true
    slosh_constraint_enable: false
```

#### `fixed_path_experiment.yaml`

```yaml
experiment:
  mode: fixed_path
  reference_source: global_path_fixed
  corridor_width: 0.30
  terminal_exclude_sec: 1.0
```

#### `point_to_point_experiment.yaml`

```yaml
experiment:
  mode: point_to_point
  reference_source: global_path
  corridor_width: 0.50
  obstacle_enable: true
```

### 4.5 launch 参数设计

launch 暴露少量权威字段：

```text
planner_variant:
  B0 | B_slosh | B_smooth | B_ours

experiment_mode:
  fixed_path | point_to_point

reference_source:
  global_path_fixed | global_path | goal

config_profile:
  fixed_path_experiment | point_to_point_experiment
```

不建议暴露一堆互斥 bool：

```text
slosh_enable
smooth_enable
cost_xxx_enable
```

这些应由 `planner_variant` 派生。否则实物实验容易跑出非法组合。

### 4.6 ROS topic 与 diagnostics

新包必须使用独立 `/spmpc/*` 命名空间，不复用当前 `/mpc/*`。

```text
/spmpc/local_trajectory
/spmpc/status
/spmpc/controller_variant
/spmpc/experiment_mode
/spmpc/cost_breakdown
/spmpc/slosh_horizon_summary
/spmpc/reference_path
/spmpc/corridor
/spmpc/solver_time_ms
/spmpc/debug/robot_state
/spmpc/debug/slosh_state
```

`/spmpc/cost_breakdown` 第一版建议字段固定：

```text
total
J_contour
J_lag
J_progress
J_v
J_control
J_smooth
J_terminal
J_corridor
J_obstacle
J_slosh_eta
J_slosh_eta_dot
pct_contour
pct_lag
pct_progress
pct_v
pct_control
pct_smooth
pct_terminal
pct_corridor
pct_obstacle
pct_slosh_total
```

注意：

```text
不要改 /mpc/cost_breakdown 21 字段 layout；
/spmpc/cost_breakdown 可以新设计，但一旦实物录包后也不要随便改。
```

### 4.7 借鉴 mpc_planner 和 scout_local_planner 的点

从 `src/mpc_planner` 借鉴：

```text
1. RealTimeData / ModuleData 思想：把每周期输入统一进 ProblemInput；
2. PlannerOutput：同时包含 trajectory 和 success；
3. state 中包含路径进度 s；
4. module 化 objective / constraint；
5. planned_trajectory 和 warmstart 诊断。
```

从当前 `scout_local_planner` 借鉴：

```text
1. slosh_integration / slosh_feedback 的观测器经验；
2. cost_breakdown 的百分比诊断思路；
3. terminal d200 的教训：terminal 单独诊断，不混进主效果；
4. record_slosh_experiment / RGB 流程；
5. launch 参数必须少而稳定。
```

不要照搬：

```text
1. mpc_planner 的完整 codegen / Acados / FORCES 体系；
2. mpc_planner 的 topology / guidance 多候选系统；
3. scout_local_planner 中控制层 tracking error 状态结构；
4. scout_local_planner 的 /mpc/* topic 名。
```

## 5. 与 src/mpc_planner 的关系

`src/mpc_planner` 适合作为规控一体方向的重要参考，但不建议直接在它上面改成你的主线。

推荐关系：

```text
src/mpc_planner
  用作 baseline / 结构参考 / 论文对比对象

spmpc_local_planner
  用作本文或下一篇的自研规控一体 SPMPC 主线
```

可以参考 `src/mpc_planner` 的：

```text
1. planner output 同时包含 trajectory 和 first control；
2. guidance / topology 候选输入 MPC；
3. module 化 cost / constraint 组织；
4. diagnostics 和 visualization 输出。
```

不要第一阶段照搬：

```text
1. 完整 topology / homotopy 候选系统；
2. 动态障碍预测模块；
3. 大量通用 planner 插件机制；
4. 复杂配置树。
```

理由：当前目标是液体运输规控一体验证，不是复刻一个通用 MPC planner 框架。

### 5.1 第一版求解器策略

第一版不要直接做完整非线性 SQP。建议先做“结构正确、数值可诊断”的轻量实现：

```text
Phase 1:
  shooting rollout + 小维度优化接口
  先跑 B0，不开 slosh，不开 obstacle

Phase 2:
  在同一 rollout 上加入 slosh state propagation 和 J_slosh

Phase 3:
  再决定是否升级到 SQP / iLQR / OSQP 线性化 QP
```

原因：

```text
1. 规控一体 MPC 的难点不是写 cost，而是定位数值问题；
2. 一开始就写完整 SQP，失败时无法判断是 reference、dynamics、cost、constraint 还是 solver；
3. 先把 ProblemInput / ProblemOutput / diagnostics 接口固定，后续替换 solver 不影响 ROS 层和实验脚本。
```

因此 `spmpc_solver.h` 第一版只需要定义接口：

```cpp
struct SpmpcSolveInput;
struct SpmpcSolveOutput;

class SpmpcSolver {
 public:
  virtual bool solve(const SpmpcSolveInput& in, SpmpcSolveOutput& out) = 0;
};
```

具体 solver 实现可以先叫：

```text
rollout_sampling_solver
```

后续再增加：

```text
sqp_solver
osqp_linearized_solver
```

但 ROS node、参数文件、诊断 topic 不应该因为 solver 替换而变化。

### 5.2 最小可用数据流

第一版必须先把数据流固定，避免后面每加一个 cost 就改主循环：

```text
ROS callbacks
  -> SpmpcRuntimeData
  -> ReferencePath / ProgressProjector
  -> CorridorBuilder
  -> SpmpcProblemBuilder
  -> SpmpcSolver
  -> TerminalPolicy
  -> DiagnosticsPublisher
  -> cmd_vel + local_trajectory
```

其中：

```text
SpmpcRuntimeData:
  odom
  tf pose
  global/fixed path
  optional costmap/corridor
  slosh state estimate
  last command

SpmpcProblemBuilder:
  不订阅 ROS
  不发布 topic
  只把 RuntimeData + VariantConfig 转成 solver input

SpmpcSolver:
  不知道实验组名字
  只看权重、约束、初值和 reference/corridor
```

这条边界是后续可维护性的关键。

### 5.3 包依赖建议

第一版 `package.xml` 依赖保持克制：

```text
roscpp
nav_msgs
geometry_msgs
std_msgs
visualization_msgs
tf2
tf2_ros
tf2_geometry_msgs
Eigen3
```

暂时不要引入：

```text
acados
FORCES Pro
complex pluginlib planner interface
dynamic_reconfigure
```

等最小闭环稳定后，再决定是否接 `nav_core` 插件接口。第一版可以先是普通 ROS node，这样更容易仿真 smoke 和 bag 诊断。

## 6. 分阶段路线

### Phase 0: 保持当前主线稳定

不动 `scout_local_planner` 的实验主线：

```text
C / D / E / F
fixed-path 实物验证
RGB max(left, center, right)
terminal 排除窗口
```

成功标准：

```text
catkin_make --pkg scout_local_planner 通过；
当前对比实验命令仍可用；
不改 /mpc/* 和 /slosh/* 既有接口。
```

### Phase 1: spmpc_local_planner 最小骨架

只做无障碍、固定全局路径引导的 local trajectory MPC。

```text
输入: /odom + /scout/global_path_fixed
输出: /spmpc/local_trajectory + /cmd_vel
cost: guidance + progress + control + smooth + terminal
slosh: 先不开
```

Phase 1 必须能单独证明三件事：

```text
1. reference/progress 是对的：
   /spmpc/debug/progress_s 单调增加，终点附近接近 1.0。

2. local trajectory 是规控一体输出：
   /spmpc/local_trajectory 中包含未来 N 步 x/y/theta/v/s，不只是当前 cmd_vel。

3. terminal 不靠外层硬停：
   terminal policy 能让速度平滑降到 0，并发布状态原因。
```

成功标准：

```text
仿真能沿 P2_s_curve 前进并停车；
输出 local trajectory；
不依赖 scout_local_planner 的主循环。
```

Phase 1 禁止做：

```text
1. slosh cost；
2. obstacle avoidance；
3. homotopy / topology candidate；
4. RGB 分析结论；
5. 实物主效果统计。
```

Phase 1 只回答一个问题：

```text
这个新包是否能作为普通 integrated MPC local planner 跑通。
```

### Phase 2: 加入 slosh 状态和 slosh cost

加入：

```text
eta / eta_dot 初值注入；
slosh rollout；
J_slosh；
/spmpc/slosh_horizon_summary。
```

成功标准：

```text
Q_slosh=0 与 Q_slosh>0 的 local trajectory 或控制行为出现可解释差异；
模型峰值和 ax/ay 变化能被诊断脚本看到。
```

Phase 2 要求额外发布：

```text
/spmpc/debug/slosh_state
/spmpc/slosh_horizon_summary
```

`slosh_horizon_summary` 至少包含：

```text
h_peak_pred
h_p95_pred
eta_x_peak
eta_y_peak
eta_dot_norm_peak
peak_k
```

注意：Phase 2 只能说明“预测模型内的 slosh-aware 优化生效”，不能直接宣称 RGB 真值有效。RGB 结论必须等 Phase 4 实物对比。

### Phase 3: 加入 corridor / obstacle 约束

第一版不做复杂 homotopy，只做局部 corridor：

```text
global path 周围 corridor；
costmap obstacle penalty；
必要时 hard bound。
```

成功标准：

```text
能在简单障碍环境中生成局部绕行轨迹；
不把绕行能力和 slosh 效果混成一个指标。
```

Phase 3 的公平边界：

```text
fixed-path 主因果实验仍使用固定 corridor；
点到点工程泛化实验才允许 obstacle/corridor 影响轨迹形状；
不要把“绕得更远所以更不晃”写成 slosh cost 的贡献。
```

### Phase 4: 对比实验

同一规控一体框架内做消融：

```text
B0 integrated MPC:
  J_base, no slosh

B-smooth:
  J_base + stronger smooth terms

B-slosh:
  J_base + J_slosh

B-ours:
  J_base + smooth shaping + J_slosh
```

外部 baseline：

```text
固定路径主因果实验:
  B0 / B-slosh / B-smooth / B-ours

点到点工程泛化实验:
  B0 / B-ours / TEB / mpc_local_planner
  DWA supplementary

mpc_planner:
  optional，能跑就 supplementary；
  跑不顺就 related work / 结构参考。

MPPI / Nav2:
  related work 或后续扩展。
```

Phase 4 的窗口口径沿用当前实物经验：

```text
主效果窗口:
  TRACKING / LOCAL_PLANNING 主过程，排除 terminal 前 1s。

terminal 窗口:
  单独报告，不进入主效果结论。

RGB 指标:
  max(left, center, right) 作为主液面高度。

模型指标:
  /spmpc/slosh_horizon_summary 与 RGB 做方向性对比，不自证效果。
```

Phase 4 每个 bag 必须记录：

```text
/spmpc/local_trajectory
/spmpc/status
/spmpc/controller_variant
/spmpc/experiment_mode
/spmpc/cost_breakdown
/spmpc/slosh_horizon_summary
/cmd_vel
/odom
/camera/color/image_raw
/slosh/*
/tf
```

这里保留 `/slosh/*` 是为了和当前 observer、RGB 流程、历史实验对齐；但 `spmpc_local_planner` 自己的控制诊断必须走 `/spmpc/*`。

### Phase 4.1 公平性口径

Route B 是规控一体实验，不再锁死 `v_ref`。因此公平性不能沿用控制器实验的“same v_ref”口径，必须改为：

```text
固定路径主因果实验锁定:
  same fixed global path
  same start pose tolerance
  same goal
  same corridor width
  same v/a/omega bounds
  same terminal exclusion window
  same RGB calibration
  same evaluation window

点到点工程泛化实验锁定:
  same map
  same start/goal
  same robot bounds
  same global planner or same global path source
  same obstacle/costmap input
  same safety envelope
```

不锁：

```text
v_ref
local trajectory shape
completion time
```

因为规控一体 planner 的能力之一就是选择局部轨迹和速度。但这会带来混杂，所以必须同时报告：

```text
RGB p95 / RMS / peak / AUC
model p95 / peak
completion time
path deviation p95 / max
mean speed
ax / ay / jerk
tracking or corridor violation count
solver failure count
```

主结论优先使用两种表达：

```text
1. 相近 completion time 下，谁的 RGB 液面更低；
2. 在 slosh-time trade-off 曲线上，B-ours 是否形成更好的 Pareto 点。
```

如果 `B-ours` 明显更慢，不能写成“纯粹更优”，只能写：

```text
slosh reduction with a time/speed trade-off
```

如果 `B-ours` 明显偏离路径，必须把 path deviation 作为代价报告，不能把“绕远所以不晃”归因给 slosh cost。

### Phase 4.2 主张-证据映射

每个对比必须对应一句明确主张：

| 对比 | 证明什么 | 不能证明什么 |
|---|---|---|
| `B-slosh` vs `B0` | slosh modal cost 单独是否改变规控一体行为并降低预测/实测晃动 | 不能证明最终方法最优 |
| `B-smooth` vs `B0` | 通用平滑/低激励策略能带来多少收益 | 不能证明液体模态建模有额外价值 |
| `B-ours` vs `B-smooth` | slosh-aware 项是否优于纯平滑 | 不能用 terminal 段效果证明 |
| `B-ours` vs `B-slosh` | slosh + smooth 是否优于只加 slosh | 不能证明 smooth 项是液体模型贡献 |
| `B-ours` vs `B0` | 完整方法相对普通 integrated MPC 的总体收益 | 需要同时报告 duration/path deviation |
| `B-ours` vs `TEB` | 相比标准 ROS1 优化局部规划器的工程优势 | 不能做严格内部因果归因 |
| `B-ours` vs `mpc_local_planner` | 相比非晃液 MPC local planner 的工程优势 | 需要保证相同 bounds/map/goal |
| `B-ours` vs `DWA` | 相比传统下界方法的工程优势 | 不应作为最强 baseline |
| `B-ours` vs `src/mpc_planner` | 相比现代 MPCC 框架的扩展性或 sim-only 对照 | 只有在依赖可跑、配置公平时才进主表 |

论文主表建议：

```text
固定路径主因果表:
  B0 / B-slosh / B-smooth / B-ours

点到点工程泛化表:
  B0 / B-ours / TEB / mpc_local_planner

Supplementary:
  DWA
  src/mpc_planner if runnable
```

统计方法：

```text
固定路径 4 组:
  若每个 block 都跑齐 4 组，用 Friedman + post-hoc Wilcoxon + Holm。

点到点外部 baseline:
  样本少时先报告 effect size、均值/方差和 Pareto 图；
  不强行做过度显著性声明。
```

### Phase 4.3 Q5/Q6/Q7 当前决策

Q5: `src/mpc_planner` 怎么处理？

```text
决策:
  不作为第一优先实物主 baseline。

执行:
  先确认 Acados / solver 依赖能不能装；
  能跑则做 supplementary 或 sim-only；
  跑不顺则降为 related work + 结构参考。

理由:
  它是很好的现代 MPCC 参考，但依赖和配置成本高；
  不能让它阻塞 spmpc_local_planner 主线。
```

Q6: “完整”到底是什么意思？

```text
决策:
  完整 = 架构完整 + 接口完整 + 实验组完整 + 诊断完整；
  不是 Phase 1 一次写满所有 NLP / SQP / obstacle / slosh hard constraint。

执行:
  Phase 1 只跑 B0；
  Phase 2 加 slosh；
  Phase 3 加 corridor/obstacle；
  Phase 4 做实验。
```

Q7: 外部 baseline 主力是谁？

```text
决策:
  点到点主外部 baseline = TEB + mpc_local_planner。

DWA:
  放 supplementary / classic lower bound。

理由:
  TEB 是 ROS1 经典优化局部规划器；
  mpc_local_planner 是 ROS1 MPC local planner，更贴近本文 integrated MPC 主线；
  DWA 过老，适合作为下界，不适合作为最强主对手。
```

## 7. 论文框图口径

当前控制层论文框图：

```text
Global / fixed path
  -> reference horizon
  -> SloshPriorityMPC tracking controller
  -> cmd_vel
```

规控一体论文框图：

```text
Goal / global path / map
  -> reference corridor
  -> Slosh-aware integrated MPC
       state: robot + liquid modal state
       cost : progress + guidance + control + smooth + slosh
  -> local slosh-safe trajectory
  -> first cmd_vel
```

核心变化：

```text
1. 当前方法只决定怎么跟踪 reference；
2. 新方法同时决定局部轨迹形状和控制输入；
3. slosh 不再只是 tracking cost，而是进入 local planning objective。
```

## 8. 风险与边界

### 8.1 最大风险

```text
1. 工程量变大；
2. 规控一体 solver 调试周期明显长于 tracking MPC；
3. 实物验证需要重新定义公平 baseline；
4. 如果同时引入 obstacle/corridor/slosh，归因会变乱。
```

### 8.2 控制策略

```text
1. 第一阶段只做无障碍 fixed-path guided local MPC；
2. 不删除 scout_local_planner；
3. 不复用 /mpc/* 话题，避免分析混淆；
4. 每个 phase 单独 smoke；
5. 每个新增 cost 都必须能单独关掉。
```

### 8.3 不做的事

第一阶段不做：

```text
1. OSCRS / GeoRef / homotopy candidate selection；
2. 完整 T-MPC++ 复刻；
3. 动态障碍预测；
4. 多容器优化；
5. 复杂全局重规划。
```

这些可以作为下一篇或 L3 方向。

整个 `spmpc_local_planner` 第一轮也不做：

```text
1. 删除或替换 scout_local_planner；
2. 复用 /mpc/cost_breakdown；
3. 把 OSCRS / GeoRef 搬进新包；
4. 把 RPP / TOPPRA / Ruckig / Biagiotti 当作新包内部模式；
5. 用 mpc_planner 的 Acados/FORCES codegen 作为硬依赖。
```

这些如果要做，必须作为单独方案，不混入第一轮 SPMPC integrated 主线。

## 9. 推荐执行顺序

```text
1. 先保留当前 scout_local_planner，用它完成当前论文控制器主线实验；
2. 同时新建 spmpc_local_planner skeleton，不影响实物主线；
3. 先在仿真跑通无 slosh integrated MPC；
4. 再加 slosh rollout 和 cost；
5. 最后才考虑障碍/corridor/homotopy。
```

如果论文时间紧：

```text
当前论文: 控制层 SloshPriorityMPC
下一篇/扩展: 规控一体 SloshPriorityMPC
```

如果决定把当前论文直接升级成规控一体：

```text
必须先冻结当前控制层实验数据；
然后把 spmpc_local_planner 作为新主线重做实验；
不要在两条主线之间混用结果。
```

## 10. 最终判断

新建 `spmpc_local_planner` 是正确方向。

它的价值不是“代码更好看”，而是把方法边界重新变清楚：

```text
scout_local_planner:
  slosh-aware tracking controller

spmpc_local_planner:
  slosh-aware integrated local planner/controller
```

这会让后续论文更容易和 `mpc_local_planner`、TEB、`src/mpc_planner`、DWA、MPPI 等规划器 baseline 对比，也能避免当前控制层主线继续膨胀。

但执行上必须保守：

```text
先建新包；
先跑仿真；
先无障碍；
先不删除旧主线；
等新主线实验证据稳定后，再决定是否替代当前论文主线。
```
