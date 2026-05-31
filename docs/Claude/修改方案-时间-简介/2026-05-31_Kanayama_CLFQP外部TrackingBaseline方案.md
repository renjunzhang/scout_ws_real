# Kanayama / CLF-QP 外部 Tracking Baseline 方案

> 创建：2026-05-31  
> 目的：确定新的正文主对比 baseline，并给出代码解耦方案。  
> 约束：baseline 必须是 tracking controller；给定同一 reference 和同一 `v_ref`，只输出 `/cmd_vel`，不改 path，不重塑 `v_ref(s)`。

## 0. 参考论文口径

| 方法 | 建议引用 | 用法 |
|---|---|---|
| Kanayama | Y. Kanayama, Y. Kimura, F. Miyazaki, T. Noguchi, "A Stable Tracking Control Method for an Autonomous Mobile Robot," ICRA 1990, pp. 384-389, DOI: `10.1109/ROBOT.1990.126006`. | 经典 Lyapunov 非完整移动机器人 tracking controller；作为老经典外部控制器 baseline。 |
| Kanayama 补充 | Y. Kanayama et al., "A stable tracking control method for a non-holonomic mobile robot," IROS 1991, pp. 1236-1241, DOI: `10.1109/IROS.1991.174669`. | 同一类非完整移动机器人稳定跟踪方法的补充引用。 |
| CLF-QP | "Obstacle Avoidance for Unicycle-Modelled Mobile Robots with Time-varying Control Barrier Functions," arXiv:`2307.08227`, 2023. | 现代 CLF/CBF-QP 控制范式参考；本文只取 CLF-QP tracking 部分，不启用 CBF 避障。 |
| CLF-QP 理论补充 | "Trajectory Tracking of Nonlinear Systems with Convex Input Constraints Based on Tracking Control Lyapunov Functions," *Applied Sciences*, 2024. | 支撑“tracking CLF + convex input constraints + online optimization”的控制器形式。 |

论文措辞边界：

```text
Kanayama is implemented as a classical Lyapunov tracking controller.
CLF-QP is implemented as a modern optimization-based tracking controller.
Both controllers receive the same reference path and nominal reference speed as the MPC variants.
Neither controller uses liquid-state feedback or sloshing measurements.
```

## 1. 新正文主表结构

当前建议把正文主表收敛为真正的 tracking controller 对比：

```text
Kanayama       classical nonlinear tracking controller
CLF-QP         modern optimization-based tracking controller
C              nominal MPC
E              smooth-only MPC
F              SloshPriorityMPC / ours
```

内部消融仍然保留：

```text
C / D / E / F
```

Supplementary / related work：

```text
RPP-style
TOPPRA-style
Ruckig-style
Biagiotti-style
TEB / DWA
OSCRS / GeoRef / homotopy candidate planning
```

原因：

```text
1. Kanayama / CLF-QP 是控制器，不改 reference / v_ref；
2. TOPPRA / Ruckig / Biagiotti 本质是 reference profile / open-loop shaping，不适合和控制器 baseline 放进同一主表；
3. RPP-style 会调节速度参考，仍可作为补充实验，但不是“固定 v_ref tracking controller”。
```

## 2. 代码包结构

不要把 Kanayama / CLF-QP 塞进 `scout_local_planner`。新增独立包：

```text
src/scout_apps/control/
├── scout_local_planner/
│   └── SloshPriorityMPC / C-D-E-F / profile baselines
│
├── tracking_baselines/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/tracking_baselines/
│   │   ├── tracking_controller_base.h
│   │   ├── reference_buffer.h
│   │   ├── kanayama_controller.h
│   │   └── clf_qp_controller.h
│   ├── src/
│   │   ├── reference_buffer.cpp
│   │   ├── kanayama_controller.cpp
│   │   ├── clf_qp_controller.cpp
│   │   └── tracking_baseline_node.cpp
│   ├── config/
│   │   ├── kanayama.yaml
│   │   └── clf_qp.yaml
│   ├── launch/
│   │   └── tracking_baseline.launch
│   └── scripts/
│       └── validate_tracking_baseline_bag.py
│
└── slosh_models/
    └── 暂不强拆；当前仍由 scout_local_planner 使用
```

第一阶段不拆 `slosh_models`。Kanayama / CLF-QP 不读液体状态，也不需要链接液体模型。

### 2.1 依赖方向

必须保持单向依赖：

```text
tracking_baselines
  依赖: roscpp, nav_msgs, geometry_msgs, std_msgs, tf2, Eigen
  不依赖: scout_local_planner, slosh_models

scout_local_planner
  不依赖: tracking_baselines
```

两个包只通过 ROS topic 和 rosbag 数据发生关系：

```text
/scout/global_path_fixed
/odom
/cmd_vel
/diagnostics/*
```

这样可以保证：

```text
1. Kanayama / CLF-QP 是真正外部 controller baseline；
2. scout_local_planner 的 MPC 主线不会继续膨胀；
3. 后续删掉 tracking_baselines 不会影响 SPMPC 主线；
4. 后续替换外部 baseline 不需要改 MPC 代码。
```

### 2.2 模块职责边界

| 模块 | 允许做什么 | 禁止做什么 |
|---|---|---|
| `tracking_baseline_node` | ROS 编排、参数读取、订阅/发布、状态机 | 写具体控制律 |
| `reference_buffer` | 路径缓存、最近点、progress、tracking error、终点判定 | 生成新路径、重塑 `v_ref(s)` |
| `kanayama_controller` | 根据 reference error 计算 `v/omega` | 读 ROS topic、做 terminal 状态机 |
| `clf_qp_controller` | 求小规模 tracking CLF-QP | 读 ROS topic、做 CBF 避障、调用 MPC solver |
| `command_limiter` | 统一限幅和 rate limit | 根据液体状态改命令 |
| `diagnostics` | 发布 baseline 身份和 tracking 诊断 | 发布 `/slosh/*` 或伪造 MPC topic |

建议把 `command_limiter` 做成独立小类，而不是散落在 Kanayama / CLF-QP 内部。这样两个外部 controller 使用完全相同的速度、角速度、加速度、角加速度限制。

## 3. tracking_baselines 输入输出契约

输入：

```text
/odom
/scout/global_path_fixed
固定参数 v_ref_nominal
固定参数 terminal / goal tolerance
```

输出：

```text
/cmd_vel
/diagnostics/experiment_group
/diagnostics/controller_variant
/tracking_baseline/status
/tracking_baseline/reference_error
/tracking_baseline/v_ref
/tracking_baseline/cmd_v_raw
/tracking_baseline/cmd_omega_raw
/tracking_baseline/cmd_v_limited
/tracking_baseline/cmd_omega_limited
```

严禁：

```text
不读 /slosh/*
不读 RGB
不调用 scout_local_planner 的 MPC solver
不改 /scout/global_path_fixed
不生成新的 v_ref(s)
不做 slosh-aware 逻辑
```

这样论文中可以明确写：

```text
The external controllers are non-slosh-aware tracking baselines under the same reference path and nominal reference speed.
```

### 3.1 不复用 `/mpc/*` 话题

`tracking_baselines` 不发布：

```text
/mpc/status_val
/mpc/cost_breakdown
/mpc/slosh_horizon_summary
```

原因：

```text
这些 topic 是 MPC 内部诊断。外部 controller 伪造这些 topic 会污染分析脚本。
```

分析脚本应按 `diagnostics/controller_variant` 区分：

```text
mpc                  -> 读取 /mpc/*
kanayama / clf_qp    -> 读取 /tracking_baseline/*
```

### 3.2 record_slosh_experiment.sh 兼容

录包脚本后续需要补充 `tracking_baseline` 话题：

```text
/tracking_baseline/status
/tracking_baseline/reference_error
/tracking_baseline/v_ref
/tracking_baseline/cmd_v_raw
/tracking_baseline/cmd_omega_raw
/tracking_baseline/cmd_v_limited
/tracking_baseline/cmd_omega_limited
```

但不要删除现有 `/mpc/*`、`/slosh/*`、`/reference/*`、`/terminal/*`、`/profile_cap/*`，否则 C/D/E/F 和历史分析会断。

## 4. reference_buffer 是关键模块

Kanayama / CLF-QP 本身不难，公平性关键在 `reference_buffer`。

它需要统一处理：

```text
1. 订阅并缓存 nav_msgs/Path；
2. 重采样路径；
3. 查找最近点；
4. 计算 path progress；
5. 计算 tracking error；
6. 给出参考位姿 q_ref；
7. 给出同一 nominal v_ref；
8. 终点处按统一 tolerance 停车。
```

注意：

```text
baseline 不允许自己生成速度剖面。
第一版直接使用固定 v_ref_nominal。
如果后续需要完全复刻 MPC 的 PathHandler v_ref(s)，再做只读式 profile 导出，不在第一阶段做。
```

### 4.1 reference_buffer 输出结构

建议内部输出一个纯数据结构，供 Kanayama / CLF-QP 共用：

```text
ReferenceSample:
  bool valid
  bool goal_reached
  double s
  double s_norm
  double x_ref
  double y_ref
  double yaw_ref
  double v_ref
  double omega_ref
  double e_x_body
  double e_y_body
  double e_yaw
  double dist_to_goal
```

注意：

```text
1. `v_ref` 第一版来自固定参数 `v_ref_nominal`；
2. `omega_ref` 可以由路径 heading 差分估计；
3. 若 `omega_ref` 噪声大，允许置 0，但必须在报告里说明；
4. `goal_reached` 只负责停车判定，不做 terminal slosh 逻辑。
```

### 4.2 终点逻辑保持最小

外部 controller 的 terminal 逻辑只做普通 tracking baseline 需要的最小停车：

```text
if dist_to_goal < goal_tolerance:
  cmd_vel = 0
  status = REACHED
```

第一版不要复刻 `scout_local_planner` 的 terminal envelope / capture / residual 逻辑。原因：

```text
1. 复刻会把外部 controller 做成另一个 MPC terminal 子系统；
2. terminal 本来不进主评价窗口；
3. 主效果窗口只统计 TRACKING -> terminal/capture - 1s。
```

如果实物发现外部 controller 终点停车过冲严重，只在 appendix 说明 terminal 行为不进主评价窗口；不要为了终点把 baseline 改得比主方法更复杂。

## 5. Kanayama 最小实现

状态误差使用机器人坐标系下的 tracking error：

```text
e_x, e_y, e_theta
```

控制形式按经典 Kanayama 思路实现：

```text
v_cmd     = v_ref * cos(e_theta) + k_x * e_x
omega_cmd = omega_ref + v_ref * (k_y * e_y + k_theta * sin(e_theta))
```

第一版可简化：

```text
omega_ref 由 reference heading 差分估计；
若估计不稳定，先置 0 并只做 path tangent tracking smoke。
```

输出经过统一限幅：

```text
|v| <= v_max
|omega| <= omega_max
|dv/dt| <= a_max
|domega/dt| <= alpha_max
```

Kanayama controller 本身不做滤波和 rate limit；这些统一交给 `command_limiter`。

## 6. CLF-QP 最小实现

第一版只做 tracking CLF-QP，不做 CBF。

QP 目标：

```text
min ||u - u_nom||_W^2 + p * delta^2
```

变量：

```text
u = [v, omega]
delta >= 0
```

约束：

```text
CLF decrease:
  L_f V + L_g V u <= -c V + delta

input bounds:
  v_min <= v <= v_max
  omega_min <= omega <= omega_max
```

第一版 `u_nom` 可以来自 Kanayama 或 nominal reference command。这样 CLF-QP 表达的是：

```text
在尽量接近 nominal tracking command 的同时，满足 Lyapunov 下降约束和输入边界。
```

实现策略：

```text
优先用小规模解析 / active-set 解法；
如果实现复杂，再引入轻量 QP solver；
不要把 OSQP 直接和 scout_local_planner 的 MPC solver 绑定。
```

### 6.1 QP 求解器边界

第一阶段不引入大型依赖。优先顺序：

```text
1. 解析 / active-set 小 QP；
2. header-only 或包内最小 QP 工具；
3. 必要时独立链接 OSQP，但不得复用 scout_local_planner 的 mpc_solver。
```

失败策略：

```text
if QP infeasible or solver fails:
  use bounded u_nom
  publish status = QP_FALLBACK
```

这保证 CLF-QP 失败不会导致底盘收到未定义命令，也不会污染 SPMPC 主线。

## 7. 实验分组更新

正文主表：

```text
KANAYAMA
CLF_QP
C
E
F
```

内部消融：

```text
C
D
E
F
```

补充实验：

```text
RPP_STYLE
TOPPRA
RUCKIG
BIAGIOTTI
```

所有组共用：

```text
同一 P2_s_curve fixed path
同一 goal
同一 RGB 固定参数
同一 recording script
同一主评价窗口: TRACKING start -> terminal/capture - 1s
```

## 8. 实施顺序

### Step 1: 建包骨架

交付：

```text
tracking_baselines package
tracking_baseline_node
launch/config skeleton
```

验证：

```text
catkin_make --pkg tracking_baselines
roslaunch --nodes tracking_baselines tracking_baseline.launch
```

### Step 2: reference_buffer

交付：

```text
能读取 /scout/global_path_fixed
能输出 nearest/progress/error/q_ref
```

验证：

```text
固定路径仿真下 /tracking_baseline/reference_error 有稳定输出
```

### Step 3: Kanayama

交付：

```text
controller_variant:=kanayama
能沿 P2_s_curve 跑通并到达终点
```

验证：

```text
仿真 smoke 1 包
实物 pilot 1 包
```

### Step 4: CLF-QP

交付：

```text
controller_variant:=clf_qp
QP 可解，失败时 fallback 到 bounded nominal command
```

验证：

```text
仿真 smoke 1 包
实物 pilot 1 包
```

### Step 5: 正式对比实验

正式组：

```text
KANAYAMA / CLF_QP / C / E / F
```

每组先 `n=1` pilot，通过后再 `n=3~5`。

## 9. 三层验证门槛

### 第一层：静态构建检查

```text
catkin_make --pkg tracking_baselines
roslaunch --nodes tracking_baselines tracking_baseline.launch
bash -n 相关脚本
```

### 第二层：仿真 fixed-path smoke

每个 controller 至少跑一包：

```text
KANAYAMA_sim_smoke_run01
CLF_QP_sim_smoke_run01
```

检查：

```text
1. 能订阅 /scout/global_path_fixed；
2. 能进入 TRACKING；
3. /cmd_vel 有输出；
4. 能到达 goal 或至少沿 P2 path 正常前进；
5. /tracking_baseline/* 诊断完整；
6. 不发布 /mpc/*。
```

### 第三层：实物 pilot smoke

每个 controller 先 1 包：

```text
P2_s_curve_KANAYAMA_pilot_run01
P2_s_curve_CLF_QP_pilot_run01
```

通过条件：

```text
1. 不撞、不原地振荡、不长时间倒车；
2. 主窗口内能稳定沿固定路径前进；
3. bag 中有 RGB、odom、cmd_vel、tracking_baseline 诊断；
4. terminal 行为只做诊断，不纳入主效果统计。
```

## 10. 不做的事

```text
不把 Kanayama / CLF-QP 放进 scout_local_planner；
不让外部 controller 读 /slosh/*；
不让外部 controller 改 path 或 v_ref(s)；
不让外部 controller 发布 /mpc/*；
不让外部 controller 复刻 SPMPC terminal/slosh 逻辑；
不在 CLF-QP 第一版里做 CBF 避障；
不把 TOPPRA/Ruckig/Biagiotti 当 controller baseline；
不在这一步恢复 OSCRS/GeoRef/规划层候选选择。
```

## 11. 成功标准

代码结构成功：

```text
scout_local_planner 仍只维护 MPC 主线和已有 profile baseline；
tracking_baselines 独立维护 Kanayama / CLF-QP；
两者通过 topic 接口共享 reference 和 cmd_vel 语义。
```

实验口径成功：

```text
Kanayama / CLF-QP / C / E / F 都是给定 reference 的 tracking controller；
只有 F 使用液体模态状态和 slosh-aware cost；
因此 F 的优势可以归因到 SloshPriorityMPC，而不是 reference shaping。
```

## 12. README / 实验方案同步要求

实施代码前后需要同步三处文档：

```text
1. src/scout_apps/control/scout_local_planner/README.md
   说明 Kanayama / CLF-QP 已移到 tracking_baselines，不属于 scout_local_planner 内部分支。

2. src/scout_apps/control/tracking_baselines/README.md
   新包自己的启动、参数、topic、边界说明。

3. docs/重要文档/20260527_SloshPriorityMPC正式对比实验验证方案.md
   新增 KANAYAMA / CLF_QP pilot 和正式录包流程。
```
