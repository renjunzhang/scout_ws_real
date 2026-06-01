# mpc_planner 外部框架结构梳理

> 梳理日期：2026-06-01  
> 来源：`src/mpc_planner/`  
> 目的：把 `src/mpc_planner` 作为未来规控一体 MPC baseline 参考，明确它与当前 `scout_local_planner` 的边界，并为新建 `src/scout_apps/control/spmpc_local_planner` 提供对齐依据。  
> 结论：`mpc_planner` 是 integrated local MPC planner/controller，不是 `scout_local_planner` 的上游全局规划器；后续建议保留 `mpc_planner` 作为外部 baseline / 结构参考，新建 `spmpc_local_planner` 实现我们的 slosh-aware integrated MPC。

---

## 1. 项目识别

`src/mpc_planner` 是 TU Delft / AMR 系列 MPC planning 框架，核心作者包括 Oscar de Groot。它包含 T-MPC++、SH-MPC、contouring MPC、动态障碍约束、guidance/topology 等模块。

建议论文口径：

```text
mpc_planner is an integrated local MPC planner/controller baseline.
It receives robot state, reference path or goal, obstacle/costmap information,
and solves a receding-horizon optimization that outputs a local trajectory and the first control command.
```

不要写成：

```text
mpc_planner = global planner + controller
```

更准确是：

```text
global/reference path 或 goal
  -> local MPC planner/controller
  -> planned local trajectory + first cmd_vel
```

---

## 2. 多包结构

```text
src/mpc_planner/
├── mpc_planner/                    # 核心 Planner 类
├── mpc_planner_modules/            # Objective / Constraint / Guidance 模块
├── mpc_planner_solver/             # Solver 抽象层: Acados / FORCES Pro
├── mpc_planner_types/              # RealTimeData / ModuleData / ReferencePath 等类型
├── mpc_planner_util/               # 参数、YAML、可视化工具
├── mpc_planner_msgs/               # ObstacleArray.msg
├── mpc_planner_rosnavigation/      # ROS1 nav_core local planner plugin
├── mpc_planner_jackal/             # Jackal 实物/ROS 适配
├── mpc_planner_jackalsimulator/    # Jackal 仿真适配
├── mpc_planner_dingo/              # Dingo 适配
└── solver_generator/               # Python DSL 生成 solver 代码
```

核心边界：

```text
Planner 核心层:
  mpc_planner/
  mpc_planner_modules/
  mpc_planner_solver/
  mpc_planner_types/

ROS 适配层:
  mpc_planner_jackal/
  mpc_planner_jackalsimulator/
  mpc_planner_rosnavigation/

Solver 生成层:
  solver_generator/
  各适配包/scripts/generate_*_solver.py
```

---

## 3. 主调用链

### 3.1 ROS 适配层

典型 ROS1 入口：

```text
mpc_planner_jackal/src/ros1_jackal.cpp
mpc_planner_jackalsimulator/src/ros1_jackalsimulator.cpp
mpc_planner_rosnavigation/src/ros1_rosnavigation.cpp
```

主循环结构：

```text
ROS callback 更新 RealTimeData / State
  -> Planner::solveMPC(state, data)
  -> PlannerOutput{trajectory, success}
  -> cmd_vel.linear.x  = getSolution(1, "v")
  -> cmd_vel.angular.z = getSolution(0, "w")
```

代码位置：

```text
mpc_planner/src/planner.cpp:Planner::solveMPC
mpc_planner/src/planner.cpp:Planner::getSolution
mpc_planner_jackal/src/ros1_jackal.cpp:JackalPlanner::loop
mpc_planner_jackalsimulator/src/ros1_jackalsimulator.cpp:JackalPlanner::loop
mpc_planner_rosnavigation/src/ros1_rosnavigation.cpp:ROSNavigationPlanner::loop
```

关键点：

```text
linear velocity 用第 1 个预测状态的 v:
  getSolution(1, "v")

angular velocity 用第 0 个控制输入的 w:
  getSolution(0, "w")
```

这说明它是典型 receding-horizon 结构：优化整段局部轨迹，但只执行第一步控制。

### 3.2 Planner::solveMPC 调用链

`Planner::solveMPC` 做的事情：

```text
1. 重置 PlannerOutput 和 ModuleData
2. 逐个 module 检查 data 是否 ready
3. 根据上一次是否 feasible 决定 warm start / braking initial guess
4. setXinit(state)
5. module->update(state, data, module_data)
6. module->setParameters(data, module_data, k)
7. _solver->loadWarmstart()
8. module->optimize(...) 或 _solver->solve()
9. 从 solver 取出 trajectory
10. 可视化 planned_trajectory / warmstart / obstacles / robot area
```

代码位置：

```text
mpc_planner/src/planner.cpp:Planner::solveMPC
mpc_planner/include/mpc_planner/planner.h:PlannerOutput
```

`PlannerOutput` 只包含：

```text
Trajectory trajectory;
bool success;
```

---

## 4. 输入接口

核心输入集中在：

```text
mpc_planner_types/include/mpc_planner_types/realtime_data.h:RealTimeData
```

`RealTimeData` 包含：

```text
robot_area
past_trajectory
dynamic_obstacles
costmap
reference_path
left_bound / right_bound
goal
goal_received
intrusion
planning_start_time
```

### 4.1 ROS topic 输入

典型 ROS1 适配层订阅：

| 输入 | topic | 作用 | 代码位置 |
|---|---|---|---|
| odom | `/input/state` | 更新 `State{x,y,psi,v}` | `ros1_jackal.cpp:stateCallback`, `ros1_jackalsimulator.cpp:stateCallback`, `ros1_rosnavigation.cpp:stateCallback` |
| pose state | `/input/state_pose` | 另一种状态输入，z 有时编码速度 | 同上 |
| goal | `/input/goal` | 更新 `data.goal` 和 `goal_received` | `goalCallback` |
| reference path | `/input/reference_path` 或 nav_core `setPlan()` | 更新 `data.reference_path` | `pathCallback`, `ROSNavigationPlanner::setPlan` |
| obstacles | `/input/obstacles` | 更新 `data.dynamic_obstacles` | `obstacleCallback` |
| costmap | nav_core plugin 内部 `costmap_ros_->getCostmap()` | 静态障碍 / decomp constraints | `mpc_planner_rosnavigation/src/ros1_rosnavigation.cpp` |
| collision feedback | `/feedback/collisions` | 记录 intrusion | `collisionCallback` |

注意：`mpc_planner_rosnavigation` 是 `nav_core::BaseLocalPlanner` plugin，能从 move_base/MBF 接收 global plan；`jackal` 和 `jackalsimulator` 适配则主要通过 remap 后的 `/input/*` topic 工作。

---

## 5. 输出接口

| 输出 | 语义 | 代码位置 |
|---|---|---|
| `/output/command` 或 nav_core `cmd_vel` | 最终执行控制命令 | `ros1_jackal.cpp:loop`, `ros1_jackalsimulator.cpp:loop`, `ros1_rosnavigation.cpp:loop` |
| planned trajectory visualization | MPC 预测局部轨迹 | `Planner::visualize`, `visualizeTrajectory(_output.trajectory, "planned_trajectory")` |
| warmstart trajectory visualization | warm start 轨迹 | `Planner::visualize` |
| obstacle / prediction markers | 障碍物和预测 | `Planner::visualize` |
| data saver fields | runtime、status、module 数据 | `Planner::saveData` |

对我们设计 `spmpc_local_planner` 的启发：

```text
必须同时输出:
  /spmpc/local_trajectory
  /cmd_vel
  /spmpc/status

不要只输出 cmd_vel。
规控一体方法的“规划层差异”需要通过 local trajectory 可视化和记录体现出来。
```

---

## 6. 默认模型、状态与控制输入

### 6.1 默认 unicycle contouring model

默认 Jackal / simulator 常用模型：

```text
solver_generator/solver_model.py:ContouringSecondOrderUnicycleModel
```

状态：

```text
x = [x, y, psi, v, spline]
```

控制：

```text
u = [a, w]
```

连续动力学：

```text
dot{x}     = v cos(psi)
dot{y}     = v sin(psi)
dot{psi}   = w
dot{v}     = a
dot{spline}= v
```

这里 `spline` 是路径进度状态。它使该框架不是简单 tracking controller，而是把路径进度也纳入优化状态。

### 6.2 带 slack 的模型

一些 ROS navigation / safe horizon / T-MPC 配置使用：

```text
solver_generator/solver_model.py:ContouringSecondOrderUnicycleModelWithSlack
```

状态：

```text
x = [x, y, psi, v, spline, slack]
```

控制：

```text
u = [a, w]
```

`slack` 用于软化约束，例如 corridor / obstacle 约束。

### 6.3 其他模型

代码中还有 bicycle / curvature-aware 等模型，但不是默认 Jackal unicycle baseline。文档中不要把 `delta` 方向盘状态写成默认 baseline 状态。

---

## 7. Solver 与优化问题生成

### 7.1 Solver backend

`mpc_planner_solver` 抽象了两种求解器：

```text
Acados
FORCES Pro
```

接口位置：

```text
mpc_planner_solver/include/mpc_planner_solver/solver_interface.h
mpc_planner_solver/include/mpc_planner_solver/acados_solver_interface.h
mpc_planner_solver/include/mpc_planner_solver/forces_solver_interface.h
```

### 7.2 Python DSL 生成

MPC 问题不是纯手写 C++，而是由 Python 脚本选择 model + modules，再生成求解器代码。

典型生成脚本：

```text
mpc_planner_jackal/scripts/generate_jackal_solver.py
mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py
mpc_planner_rosnavigation/scripts/generate_rosnavigation_solver.py
```

这对 `spmpc_local_planner` 的启发：

```text
如果追求快速验证，不建议第一版复制 Python DSL + solver codegen。
第一版可以使用我们已有 QP/OSQP 或一个更小的自研 integrated MPC skeleton。
等方法稳定后，再考虑 solver 生成化。
```

---

## 8. Cost function 模块

`mpc_planner` 的 cost 主要由 `ObjectiveModule` 组合出来。

### 8.1 MPCBaseModule

代码位置：

```text
mpc_planner_modules/scripts/mpc_base.py
mpc_planner_modules/src/mpc_base.cpp
```

作用：

```text
对任意 state/input 加二次惩罚。
常见变量:
  a
  w
  v - reference_velocity
  slack
```

典型配置：

```text
base_module.weigh_variable("a", "acceleration")
base_module.weigh_variable("w", "angular_velocity")
base_module.weigh_variable("v", ["velocity", "reference_velocity"],
  cost = velocity_weight * (v - reference_velocity)^2)
```

### 8.2 ContouringModule

代码位置：

```text
mpc_planner_modules/scripts/contouring.py
mpc_planner_modules/src/contouring.cpp
```

作用：

```text
MPCC / contouring path tracking。
```

核心误差：

```text
lag_error:
  沿路径切向的误差

contour_error:
  路径法向误差
```

cost：

```text
J_contouring =
  w_lag * lag_error^2
  + w_contour * contour_error^2
```

terminal 项：

```text
stage_idx == N-1:
  terminal_angle * angle_error^2
  + terminal_contouring * lag/contour cost
```

### 8.3 GoalModule

代码位置：

```text
mpc_planner_modules/scripts/goal_module.py
mpc_planner_modules/src/goal_module.cpp
```

作用：

```text
点到点 goal tracking。
```

cost：

```text
goal_weight * ((x-goal_x)^2 + (y-goal_y)^2)
```

注意：这不是 fixed-path contouring 主线；选择 GoalModule 会改变任务语义。

### 8.4 PathReferenceVelocityModule

代码位置：

```text
mpc_planner_modules/scripts/path_reference_velocity.py
mpc_planner_modules/src/path_reference_velocity.cpp
```

作用：

```text
当 reference path 带 velocity profile 时，把 v_ref(s) 作为 spline 参数提供给 ContouringModule。
```

如果 `reference_path.hasVelocity()` 为真，则使用路径自带速度；否则使用常数：

```text
CONFIG["weights"]["reference_velocity"]
```

### 8.5 Cost 模块总表

| 模块 | 类型 | 数学作用 | 是否改变局部几何 |
|---|---|---|---|
| `MPCBaseModule` | objective | `a^2`, `w^2`, `(v-v_ref)^2`, `slack^2` | 间接影响 |
| `ContouringModule` | objective | lag/contour/path heading/terminal contouring | 是，优化局部轨迹相对参考路径的位置 |
| `GoalModule` | objective | 到 goal 的距离 | 是，点到点 |
| `PathReferenceVelocityModule` | parameter/objective helper | 给 `v_ref(s)` spline | 不直接改变几何 |

---

## 9. Constraint 模块

### 9.1 state/input bounds

基础速度、加速度、角速度、状态上下界来自 model 的 bound：

```text
solver_generator/solver_model.py
```

例如 `ContouringSecondOrderUnicycleModel`：

```text
inputs: a, w
states: x, y, psi, v, spline
```

上下界在 `lower_bound / upper_bound` 中定义。

### 9.2 EllipsoidConstraintModule

代码位置：

```text
mpc_planner_modules/scripts/ellipsoid_constraints.py
mpc_planner_modules/src/ellipsoid_constraints.cpp
```

作用：

```text
动态障碍物椭球约束。
```

约束形式大致为：

```text
(p_robot - p_obstacle)^T A (p_robot - p_obstacle) >= 1
```

### 9.3 LinearizedConstraintModule

代码位置：

```text
mpc_planner_modules/scripts/linearized_constraints.py
```

作用：

```text
线性化动态障碍约束:
  A x <= b
```

### 9.4 DecompConstraintModule

代码位置：

```text
mpc_planner_modules/scripts/decomp_constraints.py
mpc_planner_modules/src/decomp_constraints.cpp
```

作用：

```text
静态障碍 / corridor halfspace 约束。
依赖 costmap / decomp_util。
```

### 9.5 Contouring road constraints

代码位置：

```text
mpc_planner_modules/src/contouring.cpp:constructRoadConstraints*
```

作用：

```text
如果 contouring.add_road_constraints=true，
根据 reference path 中心线和 road.width 生成左右边界 halfspace。
```

### 9.6 ScenarioConstraintModule

代码位置：

```text
mpc_planner_modules/scripts/scenario_constraints.py
mpc_planner_modules/src/scenario_constraints.cpp
```

作用：

```text
SH-MPC / 多场景不确定障碍约束。
```

第一阶段做 baseline 对比时不建议打开。

---

## 10. Guidance / topology / T-MPC++ 模块

关键模块：

```text
mpc_planner_modules/scripts/guidance_constraints.py
mpc_planner_modules/src/guidance_constraints.cpp
```

作用：

```text
1. 根据 reference path / goal / obstacle 生成多个 guidance trajectory 或局部候选；
2. 对每个候选构造局部 solver / constraints；
3. 并行求解或评估；
4. 选择 objective / consistency cost 最好的结果；
5. 把对应结果加载回主 solver。
```

这正是 `mpc_planner` 作为 T-MPC++ baseline 的强点，但也是公平性风险。

如果论文问题是：

```text
same global path 下，slosh-aware integrated MPC 是否优于 non-slosh integrated MPC
```

那么建议第一阶段关闭或固定：

```text
t-mpc.use_t-mpc++ = false 或不使用 GuidanceConstraintModule
guidance constraints disabled
dynamic obstacles disabled
road/corridor 固定
```

如果论文问题是：

```text
我们的规控一体 SPMPC 是否优于完整 T-MPC++ planner
```

则可以保留 GuidanceConstraintModule，但必须承认：

```text
对比对象不仅是 cost 不同，而是 topology/guidance/planning 能力也不同。
```

---

## 11. Terminal / goal handling

### 11.1 module objective reached

核心函数：

```text
mpc_planner/src/planner.cpp:Planner::isObjectiveReached
```

逻辑：

```text
所有 module 的 isObjectiveReached 都为 true，才认为 objective reached。
```

相关 module：

```text
Contouring::isObjectiveReached:
  距离 spline 末端 < 1.0 m

GoalModule::isObjectiveReached:
  距离 goal < 1.0 m
```

### 11.2 ROS navigation plugin

`mpc_planner_rosnavigation` 通过 nav_core 接口：

```text
setPlan()
computeVelocityCommands()
isGoalReached()
```

`isGoalReached()` 内部调用：

```text
_planner->isObjectiveReached(_state, _data)
```

并在 reached 后调用 `reset()`。

### 11.3 rotate-to-goal / infeasible braking

ROS 适配层有额外逻辑：

```text
rotateToGoal:
  起步前先转向目标方向

infeasible braking:
  solver fail 时用 deceleration_at_infeasible 减速到 0
```

代码位置：

```text
ros1_jackal.cpp:rotateToGoal
ros1_jackal.cpp:loop
ros1_jackalsimulator.cpp:loop
ros1_rosnavigation.cpp:rotateToGoal
ros1_rosnavigation.cpp:loop
```

与当前 `scout_local_planner` 的 terminal d200 逻辑相比，`mpc_planner` 的终点处理更粗：

```text
1. goal tolerance 默认约 1 m；
2. 没有专门为低 ax / 低 jerk / 液体残振设计终点状态机；
3. fail braking 是外层兜底，不是 slosh-aware terminal policy。
```

---

## 12. 当前 scout_local_planner 对比

| 维度 | `mpc_planner` | 当前 `scout_local_planner` |
|---|---|---|
| 定位 | integrated local MPC planner/controller | control-layer MPC tracking controller |
| 输入 | state + goal/path + obstacles/costmap + bounds | odom/imu + fixed/global path reference |
| 输出 | local trajectory + first command | cmd_vel + diagnostics |
| 优化状态 | `[x,y,psi,v,spline]` 或带 slack | tracking error + `v` + liquid modal state |
| 控制输入 | `[a,w]` | `[a,omega]` |
| 是否改变局部几何 | 是 | 否，跟踪给定 reference |
| 是否有 corridor/obstacle | 有，可开关 | 当前主线没有 |
| 是否有 topology/guidance | 有，可开关 | 当前主线没有 |
| 是否有 slosh state | 无 | 有 |
| 是否有 slosh cost | 无 | 有 |
| 终点逻辑 | module reached + rotate/fail brake | terminal d200 + capture/stop/diagnostics |
| 诊断 | visualization + DataSaver | `/mpc/*`, `/slosh/*`, `/terminal/*`, `/reference/*` |

关键判断：

```text
scout_local_planner 是控制层 tracking MPC；
mpc_planner 是规控一体 local MPC planner/controller；
spmpc_local_planner 应该向 mpc_planner 的结构靠拢，而不是继续扩展 scout_local_planner。
```

---

## 13. 作为 baseline 时推荐选择哪一档

`mpc_planner` 不是一个单一方法，而是一套框架。必须指定 baseline 配置。

### 13.1 最公平的 same-framework integrated MPC baseline

建议 baseline 先选：

```text
ContouringSecondOrderUnicycleModel
MPCBaseModule
ContouringModule
PathReferenceVelocityModule 可选
无 GuidanceConstraintModule
无 dynamic obstacle constraints
固定 road/corridor
无 slosh state
无 slosh cost
```

这相当于：

```text
non-slosh-aware integrated contouring MPC
```

它最适合和 `spmpc_local_planner` 的第一阶段对比。

### 13.2 强 external baseline

如果要展示“比完整 T-MPC++/mpc_planner 框架还强或不同”，可选：

```text
configuration_tmpc
GuidanceConstraintModule
EllipsoidConstraintModule / DecompConstraintModule
```

但这会引入 topology/guidance 能力差异，不适合做 slosh cost 的干净因果对比。

### 13.3 不建议第一阶段使用

```text
ScenarioConstraintModule
dynamic obstacle prediction
probabilistic / SH-MPC
multi-solver topology parallelism
```

这些会把论文问题从“液体防晃规控一体”扩大成“动态障碍风险规划”，不利于当前主线。

---

## 14. 在 spmpc_local_planner 中如何借鉴

建议新包：

```text
src/scout_apps/control/spmpc_local_planner/
```

### 14.1 需要对齐 baseline 的结构

```text
1. 同样输入 global/reference path 或 corridor；
2. 同样优化 local trajectory；
3. 同样输出 local trajectory + first cmd_vel；
4. 同样使用 [a,w] 或 [a,omega] 控制输入；
5. 同样记录 success/fail/solver runtime；
6. 同样保留 non-slosh integrated MPC baseline 开关。
```

### 14.2 我们新增的结构

在 baseline robot state 基础上增加液体模态状态：

```text
x_base = [x, y, psi, v, s]
x_slosh = [eta_x, dot_eta_x, eta_y, dot_eta_y]
x_ours = [x, y, psi, v, s, eta_x, dot_eta_x, eta_y, dot_eta_y]
```

动力学：

```text
x_robot,k+1 = f_robot(x_robot,k, u_k)
x_slosh,k+1 = f_slosh(x_slosh,k, a_x,k, a_y,k)
```

新增 cost：

$$
J_{\mathrm{slosh}}
=
\sum_{k=0}^{N}
\left[
Q_{\eta}(\eta_{x,k}^2+\eta_{y,k}^2)
+Q_{\dot{\eta}}(\dot{\eta}_{x,k}^2+\dot{\eta}_{y,k}^2)
\right]
$$

可选 hard constraint：

$$
h_k \le h_{\lim}
$$

但第一阶段建议只做 soft cost，避免 solver 可行性问题。

### 14.3 不要直接污染旧接口

建议新诊断命名空间：

```text
/spmpc/local_trajectory
/spmpc/status
/spmpc/cost_breakdown
/spmpc/slosh_horizon_summary
/spmpc/reference_corridor
```

不要复用当前：

```text
/mpc/cost_breakdown
/terminal/*
/reference/*
```

这样可以保证当前 `scout_local_planner` 实物实验链路不被新主线污染。

---

## 15. Baseline vs Ours 对比表

| 项目 | `mpc_planner` baseline | `spmpc_local_planner` ours |
|---|---|---|
| 输入 | state + global/reference path + optional obstacle/costmap | same + liquid modal state |
| state | `[x,y,psi,v,s]` | `[x,y,psi,v,s,eta_x,dot_eta_x,eta_y,dot_eta_y]` |
| control | `[a,w]` | `[a,w]` |
| robot dynamics | unicycle / contouring model | same or Scout-specific unicycle |
| slosh dynamics | none | second-order modal ODE rollout |
| objective | control + velocity + contour/lag + terminal | baseline objective + slosh cost |
| constraint | bounds + optional obstacle/corridor | same + optional slosh height constraint |
| output | local trajectory + first cmd_vel | slosh-aware local trajectory + first cmd_vel |
| terminal | objective reached / rotate / braking fallback | terminal policy must be redesigned for low ax/jerk |
| diagnostics | visuals + data saver | `/spmpc/*` structured diagnostics |
| 是否改变 global path | 不改变 global path，但可改变局部轨迹 | 同 |
| 是否使用 slosh state | 否 | 是 |
| 是否使用 slosh cost | 否 | 是 |

---

## 16. 需要固定或关闭的模块

如果 `mpc_planner` 用作干净 baseline，建议：

### 必须固定

```text
1. global/reference path；
2. control frequency；
3. horizon N 和 dt；
4. robot footprint / bounds；
5. terminal tolerance；
6. velocity / acceleration / angular velocity limits；
7. corridor width。
```

### 建议关闭

```text
1. GuidanceConstraintModule / T-MPC++ topology；
2. dynamic obstacle prediction；
3. ScenarioConstraintModule；
4. probabilistic obstacle uncertainty；
5. automatic path velocity changes，除非 ours 同样使用。
```

### 可以保留

```text
1. ContouringModule；
2. MPCBaseModule；
3. PathReferenceVelocityModule，如果所有方法共用同一 v_ref(s)；
4. simple road/corridor constraints，如果 ours 同样使用。
```

---

## 17. 对新代码的建议

`spmpc_local_planner` 不建议第一版直接 fork `mpc_planner`。

推荐路线：

```text
Phase 1:
  新建最小 integrated MPC skeleton。
  只做无障碍 fixed-path/corridor。
  输出 local trajectory + cmd_vel。

Phase 2:
  加入 slosh state / rollout / soft cost。

Phase 3:
  加入 simple corridor / static obstacle。

Phase 4:
  再考虑是否引入 mpc_planner 风格模块化 / code generation。
```

原因：

```text
1. mpc_planner 框架重，solver/codegen/模块系统成本高；
2. 我们当前需要验证 slosh-aware integrated MPC 机制，不需要一开始复刻 T-MPC++；
3. 新包可以保持接口清楚，不破坏 scout_local_planner 的实物实验链路。
```

---

## 18. 框图建议

### 18.1 Baseline integrated MPC

```text
Global path / Goal / Corridor
        +
Robot state
        +
Obstacle / Costmap
        |
        v
Reference / corridor preparation
        |
        v
Integrated MPC
  state: [x,y,psi,v,s]
  cost : progress/guidance + contour/lag + control + smooth + terminal
  cons : bounds + corridor/obstacle
        |
        v
Local trajectory + first cmd_vel
```

### 18.2 Ours slosh-aware integrated MPC

```text
Global path / Goal / Corridor
        +
Robot state
        +
Obstacle / Costmap
        +
Slosh observer state: [eta_x,dot_eta_x,eta_y,dot_eta_y]
        |
        v
Reference / corridor preparation
        |
        v
Slosh-aware Integrated MPC
  state: [x,y,psi,v,s,eta_x,dot_eta_x,eta_y,dot_eta_y]
  dynamics: robot dynamics + slosh modal dynamics
  cost : baseline cost + J_slosh
  cons : baseline constraints + optional h <= h_lim
        |
        v
Slosh-aware local trajectory + first cmd_vel
```

图中高亮的差异：

```text
1. Slosh observer state input；
2. Slosh modal dynamics inside prediction；
3. Slosh cost / optional constraint；
4. Output trajectory is slosh-aware, not only smooth or obstacle-safe。
```

---

## 19. 最终判断

`mpc_planner` 非常适合作为规控一体方向的结构参考和外部 baseline，但不适合直接改成我们的主线。

推荐决策：

```text
1. 保持 src/mpc_planner 原样，作为 external integrated MPC baseline；
2. 新建 src/scout_apps/control/spmpc_local_planner；
3. 第一阶段对齐 mpc_planner 的最小 contouring MPC 结构；
4. 只在 ours 中加入 slosh state / slosh dynamics / slosh cost；
5. 等无障碍 fixed-path integrated MPC 跑通后，再考虑 corridor/obstacle/topology。
```

这样后续论文可以清楚地写：

```text
Compared with a non-slosh-aware integrated MPC local planner, the proposed method augments the prediction state and objective with liquid modal dynamics, allowing the local trajectory itself to be optimized for slosh suppression.
```
