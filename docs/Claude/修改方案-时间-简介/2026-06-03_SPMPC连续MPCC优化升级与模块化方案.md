# SPMPC 连续 MPCC 优化升级与模块化方案

> 创建：2026-06-03  
> 目标：在已经证明 `scout_local_planner` 中 slosh cost 有效的基础上，把 `spmpc_local_planner` 从当前“运动基元择优式规控一体原型”升级为更正统的连续优化 MPCC / OCP 规控一体方法。  
> 原则：保留当前 primitive solver 可跑、可回退；新增连续优化 solver，不把升级做成一次性不可控重写。

## 1. 总结结论

建议升级，但不是直接复制 `src/mpc_planner` 的工程实现。

正确路线是：

```text
保留当前 spmpc_local_planner 框架
  +
新增一个连续优化 solver 后端
  +
把优化问题结构对齐 MPCC / mpc_planner 思想
```

也就是说，`src/mpc_planner` 的价值主要是参考它的规控一体问题组织方式：

```text
路径进度 s 进入优化状态
contour / lag error 进入 cost
progress reward 推动轨迹前进
约束统一放入 OCP / QP
```

但不建议直接 fork 它的完整代码作为主线，原因是：

```text
1. 依赖重，acados / FORCES / codegen 接入成本高；
2. 工程结构和 Scout 当前 ROS1 实物链路不同；
3. slosh observer、RGB 评估、现有 fixed-path 流程都已经围绕 scout_ws 建好；
4. 直接改外部框架会让论文方法边界和回退路径变复杂。
```

## 2. 当前状态

当前 `spmpc_local_planner` 已经具备规控一体雏形：

```text
输入:
  /odom
  /scout/global_path 或 /scout/global_path_fixed
  /map 或 costmap
  slosh state

输出:
  /spmpc/local_trajectory
  /cmd_vel
  /spmpc/cost_breakdown
  /spmpc/slosh_horizon_summary

当前 solver:
  deterministic motion-primitive rollout + argmin
```

当前版本的特点：

```text
优点:
  能跑通仿真；
  B0 / B_slosh / B_smooth / B_ours 可以做内部消融；
  anti-slosh primitives 已经能让预测 h_peak / h_p95 下降；
  结构上已经有 SpmpcSolver 接口，可插拔。

不足:
  解空间由人工 primitive 决定；
  不是连续优化整条控制序列；
  s 目前是 ProgressProjector 投影量，不是优化状态；
  论文中若想称为完整 MPCC/NMPC，需要连续优化后端。
```

## 3. 升级目标

升级后的目标是形成两个 solver 后端：

```text
SpmpcSolver
  ├── PrimitiveSolver
  │     当前 motion primitive 择优版本
  │     用于快速 smoke、回退、primitive ablation
  │
  └── ContinuousMpccSolverAcados
        新增连续优化版本
        用于论文主方法或增强版主方法
```

论文口径相应变成：

```text
The proposed SloshPriorityMPC formulates liquid-aware local planning
as an integrated MPCC/OCP problem by augmenting the robot trajectory
optimization with modal sloshing states and costs.
```

中文：

```text
本文将液体晃动模态状态引入局部规控一体 MPC，
在同一个预测优化问题中同时优化局部轨迹、速度时序和晃动风险。
```

## 4. 新连续优化问题

### 4.1 状态

连续 MPCC 版本应把路径进度 `s` 放入优化状态。

第一阶段 B0 continuous 不加 slosh，状态是 5 维：

```text
x_k^{B0} = [
  p_x,
  p_y,
  theta,
  v,
  s
]
```

加入 slosh 后，状态扩展为 9 维：

```text
x_k = [
  p_x,
  p_y,
  theta,
  v,
  s,
  eta_x,
  eta_x_dot,
  eta_y,
  eta_y_dot
]
```

注意：

```text
当前 primitive 版本:
  s = ProgressProjector 投影结果，不是优化状态。

连续 MPCC 版本:
  s 是优化状态，用于定义参考点、contour error、lag error 和 progress reward。
```

这个区别必须在代码和 Methods 里写清楚，避免把当前原型和未来连续 OCP 混在一起。

不要再写“B0 是 7 维状态”。这是错误口径：

```text
B0 continuous       = 5 维状态 [px, py, theta, v, s]
B_slosh / B_ours    = 9 维状态 [px, py, theta, v, s, eta_x, eta_x_dot, eta_y, eta_y_dot]
```

实现说明（2026-06-03）：B0 严格保持 5 维。代价用 per-node EXTERNAL，只能表达
stage-0 的 `|u - u_prev|`（跨周期连续性）与各步控制幅值；horizon 内 k>0 的逐步
`Δu` 平滑若要直接表达需把控制并入状态（B0 将变 8 维，与本口径冲突），故暂不做，
Phase C 视实测连续性再定。

### 4.2 控制输入

推荐第一版：

```text
u_k = [
  a,
  omega,
  v_s
]
```

含义：

```text
a      : 纵向加速度
omega  : 角速度
v_s    : 路径进度推进速度
```

其中 `v_s` 是 MPCC 常见写法，用于让 `s` 由优化器主动推进，而不是每周期只靠几何投影。

### 4.3 动力学

机器人模型：

$$
p_{x,k+1}=p_{x,k}+\Delta t\,v_k\cos\theta_k
$$

$$
p_{y,k+1}=p_{y,k}+\Delta t\,v_k\sin\theta_k
$$

$$
\theta_{k+1}=\theta_k+\Delta t\,\omega_k
$$

$$
v_{k+1}=v_k+\Delta t\,a_k
$$

路径进度：

$$
s_{k+1}=s_k+\Delta t\,v_{s,k}
$$

液体模态：

$$
\ddot{\eta}_x+2\zeta\omega_n\dot{\eta}_x+\omega_n^2\eta_x=-\kappa_x a_{x,k}
$$

$$
\ddot{\eta}_y+2\zeta\omega_n\dot{\eta}_y+\omega_n^2\eta_y=-\kappa_y a_{y,k}
$$

其中：

```text
a_x ≈ a
a_y ≈ v * omega
```

第一版可以继续沿用当前 `slosh_dynamics` 的离散传播函数，保证与 `scout_local_planner` 和 primitive SPMPC 的 slosh 模型一致。
其中 `κ_x / κ_y` 必须来自同一套 `slosh_dynamics` / `slosh_models` 参数，不能在 continuous solver 里另写一套隐式增益。否则 primitive vs continuous 的对比不再公平。

### 4.4 Cost

推荐写成归一化 cost：

$$
J =
J_{\mathrm{contour}}
+J_{\mathrm{lag}}
+J_{\mathrm{progress}}
+J_{\mathrm{control}}
+J_{\mathrm{smooth}}
+J_{\mathrm{terminal}}
+J_{\mathrm{slosh}}
+J_{\mathrm{corridor}}
+J_{\mathrm{obstacle}}
$$

其中：

$$
J_{\mathrm{slosh}}
=
\frac{1}{N}
\sum_{k=0}^{N}
\left[
Q_\eta
\frac{\eta_{x,k}^{2}+\eta_{y,k}^{2}}{\eta_{\mathrm{ref}}^2}
+
Q_{\dot{\eta}}
\frac{\dot{\eta}_{x,k}^{2}+\dot{\eta}_{y,k}^{2}}{\dot{\eta}_{\mathrm{ref}}^2}
\right]
$$

归一化口径：

```text
e_contour_ref = corridor_width / 2
e_lag_ref     = v_ref * dt 或一个 horizon step 的典型路径推进量
a_ref         = a_max
omega_ref     = omega_max
eta_ref       = slosh_height_ref
eta_dot_ref   = omega_n * slosh_height_ref
```

实现说明（2026-06-03）：`e_l_ref` 代码默认取 `max(0.1, v_max*dt)`，偏离字面的
`v_ref*dt`（后者过小会放大 lag 权重）；该值是运行时参数，wrapper 可覆盖。
slosh 归一化用 `eta/eta_ref`；为与 primitive 的 `h/h_ref` 一致（c_h·‖η‖=h），
wrapper 设 `eta_ref = slosh_height_ref / c_h`，保证两后端尺度可比（§11.6）。

原则：

```text
误差类项统一无量纲化；
stage cost 统一按 horizon 步数 N 平均；
progress reward 单独处理，不能和误差项混在一起解释；
论文报告归一化后的权重，而不是裸量纲权重。
```

`1/N` 不是只针对 `J_slosh`，而是对所有逐步累加的 stage cost 使用同一口径：

```text
J_stage = (1/N) * Σ_k l_k
```

原因：

```text
1. 避免不同 horizon 步数下同一权重对应不同绝对代价；
2. 保证 primitive 和 continuous 后端权重可迁移；
3. 避免 stage 累加项淹没只计算一次的 terminal/progress 类项。
```

### 4.5 `J_smooth` 的接口要求

连续 OCP 中如果要惩罚控制变化率：

$$
J_{\mathrm{smooth}}
=
\sum_{k=0}^{N-1}
\left\|
\frac{u_k-u_{k-1}}{u_{\mathrm{ref}}}
\right\|_{R_{\Delta u}}^2
$$

必须提前设计 `u_{k-1}` 的来源。推荐第一版采用：

```text
u_prev 作为 OCP 参数传入:
  u_{-1} = 上一控制周期实际发布的 [a_prev, omega_prev, v_s_prev]

stage 0:
  Δu_0 = u_0 - u_prev

stage k>0:
  Δu_k = u_k - u_{k-1}
```

这是最终完整连续 OCP 的目标形式：`J_smooth` 既能约束 horizon 内的控制连续性，也能约束跨控制周期的第一帧跳变。

注意：

```text
如果不传 u_prev，连续求解器可能每周期重新规划出不同第一步，
即使 horizon 内 smooth，实际 /cmd_vel 仍可能跳。
```

实现说明（2026-06-03）：第一版 acados 后端只实现“跨周期第一帧连续性”：

```text
stage 0:
  J_smooth = ||u_0 - u_prev||^2

stage k>0:
  不显式惩罚 u_k - u_{k-1}
```

原因是当前 per-node EXTERNAL cost 只能看到本 stage 的 `x_k,u_k,p_k`，不能直接引用上一 stage 的 `u_{k-1}`。因此当前 `/spmpc/cost_breakdown` 中的 `J_smooth` 也必须只统计 stage 0 相对上一周期 `u_prev` 的跳变，不能把尚未进入 OCP 的 horizon 内差分伪装成真实代价。

后续若要实现完整 horizon 内 `Δu_k`，需要二选一：

```text
1. 状态增广: 把上一控制量 u_prev_state=[a,omega,v_s] 放进状态并随动力学传播；
2. 换 cost 结构: 使用能表达相邻 stage 控制差分的 OCP 建模方式。
```

在此之前，论文中只能把第一版 continuous 的 smooth 项描述为：

```text
cross-cycle first-step smoothing + control effort regularization
```

### 4.6 连续 reference 的前置条件

连续 MPCC 不能只用离散 path 最近点。必须提供可微的弧长参考：

```text
s -> x_ref(s)
s -> y_ref(s)
s -> psi_ref(s)
s -> kappa_ref(s)
```

推荐做法：

```text
1. 复用或移植 scout_local_planner 中已有 spline / path interpolation 思路；
2. 新增 ReferenceSpline，不依赖 ROS msg；
3. ReferencePath 保留离散路径存储；
4. ReferenceSpline 负责连续采样；
5. ProgressProjector 仍可用于初始化 s0 / warm start。
```

连续 MPCC 的 contour / lag error 都应基于 `ReferenceSpline.sample(s)` 计算，而不是只靠离散路径点。

## 5. 模块化代码结构

### 5.1 保持 ROS 层薄

ROS 层只负责：

```text
订阅 odom / path / map
读取参数
组装 SolverInput
调用 problem.solve()
发布 cmd_vel / local_trajectory / diagnostics
```

ROS 层不应包含：

```text
cost 计算
slosh rollout 细节
优化变量组织
solver 后端判断逻辑
```

### 5.2 推荐目录

在现有 `spmpc_local_planner` 下新增：

```text
include/spmpc_local_planner/solvers/
  primitive_solver.h
  continuous_mpcc_solver_acados.h
  solver_factory.h

src/solvers/
  primitive_solver.cpp
  continuous_mpcc_solver_acados.cpp
  solver_factory.cpp

scripts/acados/
  generate_spmpc_acados.py
  spmpc_acados_model.py
  spmpc_acados_cost.py
  spmpc_acados_constraints.py

generated/acados/
  # acados codegen 输出目录，默认不手写
```

reference 层建议新增：

```text
include/spmpc_local_planner/reference/reference_spline.h
src/reference/reference_spline.cpp
```

职责：

```text
ReferencePath:
  保存离散 path 点，供 ROS 层更新和 debug。

ReferenceSpline:
  从 ReferencePath 构建弧长参数化曲线；
  提供 sample(s) = [x_ref, y_ref, psi_ref, kappa_ref]。

ProgressProjector:
  把当前机器人位置投影到 ReferencePath / ReferenceSpline，得到初始 s0。
```

这个文件树是 acados 优先路线。核心区别是：

```text
cost / constraints / discretization:
  在 Python + CasADi 里描述；
  由 acados codegen 生成 C solver；
  不在 C++ 里手写 ocp_cost.cpp / ocp_constraints.cpp / ocp_discretization.cpp。

C++ ContinuousMpccSolverAcados:
  只做参数打包、warm start、调用生成求解器、读取解、发布诊断。
```

因此不要提前铺一套完整 C++ `ocp/` 目录。等后续确实需要非 acados 后端时，再独立增加 `ocp/` 抽象。

现有文件的处理：

```text
core/rollout_sampling_solver.*
  保留，但建议后续改名为 primitive_solver.*
  因为它不是随机 sampling，也不是 MPPI。

core/spmpc_solver.h
  保留为统一接口。

core/spmpc_problem.*
  保留，作为上层 problem facade。
  内部通过 solver_factory 创建 PrimitiveSolver 或 ContinuousMpccSolverAcados。
```

### 5.3 SolverFactory

新增参数：

```yaml
solver_backend: primitive   # primitive | continuous_mpcc_acados
```

创建逻辑：

```text
solver_backend=primitive
  → PrimitiveSolver

solver_backend=continuous_mpcc_acados
  → ContinuousMpccSolverAcados
```

这样实验命令可以只改一个参数，不改 launch 结构。

### 5.4 参数文件

继续保持当前分层：

```text
config/planner/common.yaml
config/planner/variants.yaml
config/platforms/scout_mini.yaml
config/containers/tube_default.yaml
config/experiments/fixed_path.yaml
config/experiments/point_to_point.yaml
```

新增：

```text
config/planner/solver_backends.yaml
```

示例：

```yaml
primitive:
  solver_backend: primitive
  primitive_library: anti_slosh

continuous_mpcc_acados:
  solver_backend: continuous_mpcc_acados
  acados_source_dir: /home/a/acados
  generated_solver_dir: src/scout_apps/control/spmpc_local_planner/generated/acados
  warm_start_enable: true
  max_sqp_iter: 5
```

## 6. 连续优化后端选择

### 6.1 第一优先：acados

现在的连续 MPCC 主线明确选择 acados，而不是 SQP + OSQP。

原因：

```text
1. 论文口径更正统：nonlinear OCP / SQP-RTI / codegen；
2. 更接近 src/mpc_planner 的问题组织方式；
3. cost / dynamics / constraints 在 CasADi 中一次定义，便于和 Methods 方程对齐；
4. C++ 侧保持薄包装，模块边界比手写 SQP 更清楚；
5. 可以去掉 primitive 方案里“候选集覆盖性”的论文尾巴。
```

代价：

```text
1. 需要本机安装 acados；
2. 需要维护 Python codegen 脚本；
3. 需要处理 codegen 产物和 catkin 编译的衔接；
4. infeasible / warm start / solver status 要单独诊断。
```

### 6.2 环境现状

当前环境探测结果：

```text
ACADOS_SOURCE_DIR 未设置；
常见路径未发现 acados；
Python acados_template 未安装；
CasADi 已安装。
```

因此 Phase A 可以先做不依赖 acados 的接口地基；但 Phase B 之后要进入 continuous MPCC，就必须安装 acados。

acados 工作流参考 `src/mpc_planner`：

```text
CasADi 描述模型 / cost / constraints
  ↓
acados codegen 生成 C solver
  ↓
C++ ContinuousMpccSolverAcados 包装求解器
  ↓
ROS 层仍只调用 problem.solve()
```

### 6.3 SQP + OSQP 的定位

SQP + OSQP 暂不作为主实现路线。

保留为 fallback 讨论：

```text
如果 acados 在当前机器上长期无法安装；
或者 codegen 与 catkin 集成成本不可控；
才考虑手写 sequential convexification + OSQP。
```

但文档、代码结构和下一阶段开发默认按 acados 优先推进。

## 7. 分阶段落地

### Phase A：接口准备，不改变行为

目标：

```text
只重构接口，不改变当前 primitive solver 输出。
```

任务：

```text
1. 新增 solver_backend 参数；
2. 新增 SolverFactory；
3. 把当前 RolloutSamplingSolver 包装为 PrimitiveSolver；
4. 新增 ContinuousMpccSolverAcados stub，返回 NOT_IMPLEMENTED；
5. 新增 ReferenceSpline 框架，但 primitive backend 暂时不依赖它；
6. 保证 solver_backend=primitive 时仿真结果基本不变；
7. diagnostics 增加 /spmpc/solver_backend。
```

验收：

```text
catkin_make 通过；
B0 / B_slosh / B_smooth / B_ours primitive smoke 通过；
旧 phase3 / phase4 脚本不需要改或只改参数名。
```

### Phase B：acados B0 生成器

目标：

```text
建立 B0 continuous 的 CasADi/acados 模型生成链路。
```

任务：

```text
1. 新增 scripts/acados/generate_spmpc_acados.py；
2. 新增 scripts/acados/spmpc_acados_model.py；
3. 新增 scripts/acados/spmpc_acados_cost.py；
4. 新增 scripts/acados/spmpc_acados_constraints.py；
5. B0 状态先用 [px, py, theta, v, s]；
6. B0 控制先用 [a, omega, v_s]；
7. cost 先做 contour + lag + progress + control + smooth + terminal；
8. 约束先做 v / omega / a / v_s bounds；
9. codegen 输出到 generated/acados/。
```

验收：

```text
python3 scripts/acados/generate_spmpc_acados.py 能生成 solver；
生成目录不需要手写修改；
不启动 ROS 也能完成 codegen。
```

### Phase C：B0 acados 最小闭环

目标：

```text
让 B0 continuous MPCC 通过 acados 在仿真中闭环运行。
```

第一版只做：

```text
状态: [px, py, theta, v, s]
控制: [a, omega, v_s]
cost: contour + lag + progress + control + smooth + terminal
约束: v / omega / a / v_s bounds
```

暂不加入：

```text
slosh cost
obstacle / costmap
hard corridor constraint
复杂 homotopy
```

第一版 B0 continuous 可以保留 corridor soft penalty，但不要把 obstacle、costmap、hard corridor 一起塞进来。否则 B0 跑不起来时，无法判断问题来自 MPCC 本体、约束建模还是 costmap 接口。

新增 C++ 包装：

```text
ContinuousMpccSolverAcados:
  读取 SolverInput / ReferenceSpline；
  打包 acados 参数；
  设置 x0 / u warm start；
  调用生成求解器；
  读取 u0 / trajectory / status；
  填充 SolverOutput 和 diagnostics。
```

验收：

```text
B0_continuous 可以在 fixed_path 仿真跑到 goal；
progress_s 单调；
cmd_vel 无明显跳变；
solver_ms 满足 30 Hz 实时口径。
```

30 Hz 实时口径：

```text
control_frequency = 30 Hz
dt = 0.0333333333 s
horizon_steps = 60
预测时长约 2.0 s

solver_ms 平均值:  < 20 ms
solver_ms p95:     < 33 ms
偶发最大值:        < 50 ms 可以接受，但不能作为正式验收标准
```

### Phase D：加入 slosh state / slosh cost

目标：

```text
把已经在 scout_local_planner 证明有效的 slosh state/cost 放入连续 OCP。
```

任务：

```text
1. CasADi 状态扩展 eta_x / eta_x_dot / eta_y / eta_y_dot；
2. CasADi dynamics 接入同一套 slosh 参数和 κ 增益；
3. acados cost 加 J_slosh_eta / J_slosh_eta_dot；
4. 复用 slosh_height_ref / eta_dot_ref 归一化；
5. 重新 codegen；
6. diagnostics 对齐 primitive 版本。
```

验收：

```text
B0_continuous vs B_slosh_continuous 有可观察差异；
B_slosh_continuous 的预测 h_peak / h_p95 相对 B0 下降；
cmd_v / omega 变化可解释。
```

### Phase E：内部消融和实物 fixed-path

实验组：

```text
B0_continuous
B_slosh_continuous
B_smooth_continuous
B_ours_continuous
```

指标：

```text
RGB max-LCR p95 / RMS / peak / AUC
model h_p95 / h_peak
duration
tracking/path deviation
cmd_v / odom_v
ax / ay / jerk
solver_ms
```

主窗口：

```text
tracking_pre_terminal
terminal approach 单独诊断，不混入主效果统计。
```

## 8. 对比实验口径

### 8.1 内部主表

连续优化版本主表建议：

```text
B0_continuous:
  ordinary integrated MPC

B_slosh_continuous:
  B0 + slosh modal cost

B_smooth_continuous:
  B0 + stronger control / smooth cost

B_ours_continuous:
  B0 + slosh modal cost + smooth shaping
```

证明关系：

```text
B_slosh vs B0:
  slosh cost 单独是否有效。

B_smooth vs B0:
  普通平滑是否有效。

B_ours vs B_smooth:
  slosh-aware 是否优于 smooth-only。

B_ours vs B_slosh:
  slosh + smooth 是否优于 slosh-only。
```

### 8.2 外部 baseline

点到点或工程泛化可以继续保留：

```text
TEB
DWA
mpc_local_planner
SPMPC B0
SPMPC B_ours
```

注意：

```text
外部 baseline 不作为 slosh cost 因果证明主表；
主因果仍然依赖 B0 / B_slosh / B_smooth / B_ours 内部消融。
```

### 8.3 primitive solver 的定位

当前 primitive solver 不丢弃。

论文/文档中可以定位为：

```text
1. early prototype / real-time fallback；
2. anti-slosh motion primitive ablation；
3. continuous solver infeasible 时的 fallback；
4. supplementary baseline。
```

不要把 primitive solver 和 continuous MPCC solver 混成同一个方法。

同时，primitive vs continuous 本身可以成为一组正式方法学对比：

```text
Primitive SPMPC:
  运动基元库择优，物理核 = 同一套 slosh_dynamics。

Continuous MPCC SPMPC:
  acados 连续优化整条控制序列，物理核 = 同一套 slosh_dynamics。
```

这组对比的公平性前提必须锁死：

```text
control_frequency = 30 Hz
dt = 0.0333333333 s
horizon_steps = 60
预测时长约 2.0 s
stage cost 均按 N 平均
slosh 参数 / κ 增益 / 归一化尺度一致
```

也就是说，primitive vs continuous 只能改：

```text
optimizer / solver backend
```

不能同时改：

```text
控制频率
预测步长
horizon 步数
预测时域长度
stage cost 缩放方式
```

否则这组对比只能作为定性方法学展示，不能支撑“只换 optimizer”的因果结论。

这组对比可以证明：

```text
slosh modal modeling 是 solver-agnostic 的；
收益不是某个 primitive 模板偶然带来的；
continuous optimizer 能否在同一物理模型下取得更好的晃动-时间折中。
```

## 9. 风险与回退

### 9.1 风险

```text
1. acados codegen / catkin 集成失败；
2. acados solver infeasible 或 status 不稳定；
3. solver_ms 超过 30 Hz 控制周期；
4. progress reward 过强导致抢进度；
5. corridor 约束过弱导致偏离路径减晃；
6. slosh cost 数值尺度过小或过大；
7. terminal approach 再次污染主效果统计。
```

### 9.2 回退

```text
solver_backend=primitive
  随时回到当前可跑版本。

continuous backend smoke 不通过
  不影响当前实物 primitive fixed-path 链路。

slosh continuous 不稳定
  先跑 B0_continuous / B_smooth_continuous，
  把 slosh 加入延后。
```

## 10. 代码边界红线

必须遵守：

```text
1. ROS msg 不进入 core/solver/acados wrapper 内部；
2. ROS 层不写 cost；
3. slosh dynamics 独立于 solver；
4. reference/projection 独立于 solver；
5. PrimitiveSolver 和 ContinuousMpccSolverAcados 只通过 SpmpcSolver 接口暴露；
6. 所有实验组通过参数切换，不写多套 launch 主流程；
7. 不删除当前 primitive solver；
8. Phase B 之后 acados 是 continuous MPCC 主线依赖，Phase A 仍必须可在无 acados 环境下编译；
9. B0 continuous 第一版不启用 obstacle/costmap/hard corridor；
10. `J_smooth` 必须有跨周期 `u_prev` 参数接口；
11. 连续 MPCC 必须使用 ReferenceSpline 的 s 连续采样，不能只用离散 path 最近点。
```

## 11. 模块化验收清单

这部分是写代码时的硬验收标准。只要违反其中任意一条，就说明升级后的 `spmpc_local_planner` 开始失去解耦。

### 11.1 依赖方向

允许的依赖方向：

```text
ros/
  → core/
  → solvers/
  → reference/
  → dynamics/

solvers/
  → reference/
  → dynamics/
  → generated/acados/

scripts/acados/
  → CasADi / acados_template
  → 生成 generated/acados/

reference/
  → 只依赖标准 C++ / Eigen

dynamics/
  → 只依赖标准 C++ / Eigen
```

禁止反向依赖：

```text
core/、solvers/、reference/、dynamics/
  不允许 include ros/ros.h
  不允许 include geometry_msgs / nav_msgs / tf
  不允许直接发布或订阅 topic

generated/acados/
  不允许反向依赖 ROS 层
```

### 11.2 模块职责

```text
SpmpcLocalPlannerROS:
  订阅、发布、参数读取、SolverInput 组装。

SpmpcProblem:
  统一 problem facade，负责调用当前 solver_backend。

SolverFactory:
  根据 solver_backend 创建 PrimitiveSolver 或 ContinuousMpccSolverAcados。

PrimitiveSolver:
  当前运动基元库择优，可作为 fallback 和 ablation。

ContinuousMpccSolverAcados:
  acados 生成求解器的薄包装，只负责参数打包、warm start、solve、读回结果。

ReferencePath:
  离散路径存储。

ReferenceSpline:
  弧长连续采样 s -> [x_ref, y_ref, psi_ref, kappa_ref]。

ProgressProjector:
  当前位姿投影到 s0，用于初始化和诊断。

SloshDynamics:
  唯一液体模态物理核，primitive 和 continuous 共用。

scripts/acados:
  CasADi 建模、cost、constraints、codegen。

generated/acados:
  生成物，只由脚本生成，不手写。
```

### 11.3 参数切换

所有实验必须通过参数切换：

```text
solver_backend:
  primitive
  continuous_mpcc_acados

controller_variant:
  B0
  B_slosh
  B_smooth
  B_ours
```

禁止：

```text
为每个实验组写一套不同 C++ 主流程；
在代码里 hard-code B0 / B_slosh / B_ours；
用不同 launch 文件偷偷改变 dt / horizon / slosh 参数。
```

### 11.4 生成物管理

```text
scripts/acados/*.py 是唯一手写建模入口；
generated/acados/ 是 codegen 产物；
generated/acados/ 内文件默认不手改；
如果生成物太大，可在 git 管理策略里单独决定是否提交；
但生成命令、依赖版本和输出路径必须记录。
```

### 11.5 诊断一致性

Primitive 和 Continuous 必须都填充：

```text
/spmpc/status
/spmpc/controller_variant
/spmpc/solver_backend
/spmpc/local_trajectory
/spmpc/cost_breakdown
/spmpc/slosh_horizon_summary
/spmpc/solver_time_ms
/spmpc/debug/progress_s
```

字段含义必须一致：

```text
J_slosh_eta      同一归一化尺度
J_slosh_eta_dot  同一归一化尺度
progress_s       同一 s 定义
h_peak_pred      同一 slosh height 计算方式
solver_time_ms   同一计时范围: solve() 内部
```

这样后续分析脚本才能跨 solver backend 比较。

### 11.6 公平性验收

primitive vs continuous 这组对比只有在下面条件同时满足时，才允许写成“只换 optimizer”：

```text
control_frequency = 30 Hz
dt = 0.0333333333 s
horizon_steps = 60
stage cost 按 N 平均
slosh 参数一致
κ 增益一致
ReferenceSpline / ProgressProjector 口径一致
terminal 统计窗口一致
```

否则只能写成定性对照，不能作为因果证明。

## 12. 最小文件改动顺序

推荐执行顺序：

```text
1. 新增 solvers/solver_factory
2. 当前 RolloutSamplingSolver 迁移/包装成 PrimitiveSolver
3. 增加 solver_backend 参数与诊断
4. 新增 ReferenceSpline
5. 新增 ContinuousMpccSolverAcados stub
6. 新增 scripts/acados/generate_spmpc_acados.py 与 B0 CasADi 模型
7. 生成 acados B0 solver
8. C++ wrapper 接入 acados B0 solver，先跑 B0_continuous
9. CasADi 模型加 slosh state/cost 并重新 codegen
10. 做 continuous B0/B_slosh/B_smooth/B_ours smoke
11. 做 primitive vs continuous 同物理核对比
```

每一步都必须可编译、可 smoke、可回退。

## 13. 最终论文表述

如果连续优化版本跑通，论文主方法可以写成：

```text
SloshPriorityMPC formulates liquid-aware robot navigation as an
integrated MPCC problem. The robot state, path progress, and modal
sloshing states are predicted over the same horizon, while contour
tracking, progress, smoothness, and sloshing-risk objectives are
jointly optimized.
```

中文：

```text
SloshPriorityMPC 将液体运输局部导航建模为规控一体 MPCC 问题。
机器人状态、路径进度和液体模态状态在同一预测域中联合传播，
并在同一目标函数中同时优化路径跟踪、前进效率、控制平滑性和液体晃动风险。
```

如果连续优化版本暂时没跑通，则论文主线保持当前可验证版本：

```text
motion-primitive-based integrated SPMPC
```

同时把 continuous MPCC 写成后续升级方向。
