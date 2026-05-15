# 当前 MPC 数学化架构总结（含通用 MPC 对比）

本文基于当前代码实现，给出可直接对应到工程的数学表达。

补充说明：

- 这是**结构化 QP 的 tracking MPC**，不是自由空间 NMPC。
- 当前每个控制周期求解的并不是固定 LTI QP，而是围绕名义轨迹 successive linearization 后得到的**affine time-varying QP**。
- 当前 near-goal 控制结构是：
  - 远场：tracking MPC
  - 执行层速度入口：`v_des_rate_limit` 先限制进入 `PathHandler` 的速度上限变化率
  - 终点前：`terminal_slowdown` 通过参考速度和末端速度权重引导平缓减速
  - 捕获区：`terminal_capture_stop` 进入 `CAPTURE_BRAKE`，按命令变化率限制逐步刹停
  - 兜底恢复：外层 `terminal recovery` 默认不进主实验，只在终点异常/调头回点时手动开启
  - capture/recovery 阶段不再代表纯 MPC cost 行为，晃动有效性主对比应优先截在进入这些阶段之前

## 目录

- 0) 参数入口速查
  - 0.1 代价函数相关参数
  - 0.2 约束与 near-goal 相关参数
  - 0.3 slosh 模型相关参数
  - 0.4 IMU / 实验覆盖常用参数
  - 0.5 上游参考路径语义
  - 0.6 执行层速度参考链路
- 1) 当前项目 MPC 的优化问题
- 2) 运动学/动力学模型（项目特化）
  - 2.1 基础 Frenet 跟踪子系统（4 维）
  - 2.2 液体晃动增广子系统（4 维）
- 3) 代价函数（项目特化）
- 4) 约束系统（项目特化）
  - 4.1 等式约束
  - 4.2 不等式约束
- 5) 回答核心问题：没有“液面约束层”会怎样？
- 6) 通用 MPC 与本项目 MPC 的数学化对比
  - 6.1 通用 MPC（抽象）
  - 6.2 本项目 MPC（工程落地）
- 7) 监测量与终点行为的工程语义
  - 7.1 `/slosh/height`
  - 7.2 `/slosh/height_pred_max`
  - 7.3 `/mpc/slosh_horizon_summary`
  - 7.4 与视觉 `height_peak_rel_mm` 的关系
  - 7.5 near-goal 行为

## 0) 参数入口速查

当前文档里提到的参数，运行时主要有 3 个入口：

- 实物默认参数真源：
  - `/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params.yaml`
- 仿真默认参数真源：
  - `/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml`
- 实验时常用 launch 覆盖入口：
  - `/home/a/scout_ws/src/scout_apps/control/scout_local_planner/launch/slosh_experiment.launch`

优先级上，一般是：

- YAML 提供默认值
- `slosh_experiment.launch` 在启动时再覆盖一部分实验参数

### 0.1 代价函数相关参数

- 跟踪项权重：
  - `mpc/Q_el`: 纵向 lag 误差权重（未启用 contour/lag 结构时使用）
  - `mpc/Q_ec`: 横向 contour 误差权重（未启用 contour/lag 结构时使用）
  - `mpc/Q_etheta`: 航向误差权重
  - `mpc/Q_v`: 速度跟踪误差 `v-v_ref` 的权重
- contour / lag 结构：
  - `mpc/use_contour_lag`: 是否用 contour/lag 误差定义替代原始 `e_c/e_l`
  - `mpc/Q_contour`: contour 横向误差权重
  - `mpc/Q_lag`: lag 纵向误差权重
- 角速度前馈项：
  - `mpc/enable_omega_ff`: 是否加入 `(\omega-\omega_ref)^2` 这一项
  - `mpc/Q_omega_ff`: 角速度前馈项的权重
- 控制项权重：
  - `mpc/R_a`: 线加速度控制量 `a` 的惩罚权重
  - `mpc/R_omega`: 角速度控制量 `omega` 的惩罚权重
- 控制变化率项权重：
  - `mpc/R_da`: 相邻时刻加速度变化 `Δa` 的惩罚权重
  - `mpc/R_domega`: 相邻时刻角速度变化 `Δomega` 的惩罚权重
- 终端渐进权重：
  - `mpc/terminal_factor_ec`: 预测末段横向误差权重放大倍数
  - `mpc/terminal_factor_etheta`: 预测末段航向误差权重放大倍数
  - `mpc/terminal_factor_v`: 预测末段速度误差权重放大倍数
  - `mpc/terminal_ramp_steps`: 末段渐进放大权重的步数
- slosh 软代价：
  - `mpc/Q_slosh`: 归一化晃动风险权重，最终映射到 `Q_slosh_eta`
  - `mpc/slosh_height_ref`: 参考晃动高度，用于把模型液面高度归一化，默认 `0.005 m`
  - `mpc/slosh_eta_dot_ratio`: `eta_dot` 等效位移项相对 `eta` 项的比例
  - `mpc/Q_slosh_eta_dot`: 旧的手动 `eta_dot` QP 权重；仅当 `slosh_eta_dot_ratio <= 0` 时作为兼容入口使用

其中：

- `Q_slosh = 0` 表示不加入 slosh 软代价
- `Q_slosh > 0` 表示把归一化后的 `eta_x / eta_y / eta_dot` 二次惩罚加入 cost
- `enable_omega_ff = true` 表示额外加入 `(\omega-\omega_ref)^2` 这一项，作用是让 MPC 输出更接近一个预设的参考角速度（通常来自路径曲率）

### 0.2 约束与 near-goal 相关参数

- slosh 盒约束：
  - `mpc/enable_slosh_box_constraint`: 是否启用第一版 `ETA_X/ETA_Y` modal proxy 盒约束
  - `mpc/slosh_height_max`: 允许的液面高度预算，用于计算盒约束阈值
- 控制边界与变化率边界：
  - `vehicle/v_max`: 最大线速度上界
  - `vehicle/omega_max`: 最大角速度上界
  - `vehicle/a_max`: 最大线加速度上界
  - `vehicle/alpha_max`: 最大角加速度上界，对应 `omega` 变化率约束
  - `vehicle/j_max`: 最大 jerk 上界，对应 `a` 变化率约束
  - `mpc/constrain_omega_rate`: 是否启用角速度变化率硬约束
  - `mpc/constrain_accel_rate`: 是否启用加速度变化率硬约束
- near-goal / 路径处理：
  - `v_des_rate_limit/enable`: 是否限制进入 `PathHandler` 前的执行层速度上限变化率
  - `v_des_rate_limit/accel_limit`: `v_des` 上升变化率上限
  - `v_des_rate_limit/decel_limit`: `v_des` 下降变化率上限
  - `path_handler/lookahead_distance`: 前视距离，影响参考点采样与转弯激进程度
  - `path_handler/goal_tolerance`: 判定到达目标的位置容差
  - `path_handler/yaw_tolerance`: 判定到达目标的航向容差
  - `path_handler/goal_reached_max_speed`: 允许切到 `REACHED` 的最大线速度
  - `path_handler/goal_reached_max_omega`: 允许切到 `REACHED` 的最大角速度
  - `path_handler/goal_capture_distance`: 终点捕获区半径
  - `path_handler/goal_capture_min_speed`: 捕获区内维持的最低参考速度
  - `path_handler/max_tan_accel`: 速度曲线生成时的最大切向加速度
  - `path_handler/max_tan_decel`: 速度曲线生成时的最大切向减速度
  - `path_handler/goal_speed`: 路径终点的目标参考速度
  - `terminal_slowdown/enable`: 是否在终点捕获前提前压低参考速度
  - `terminal_capture_stop/enable`: 是否启用终点捕获制动
  - `terminal_recovery/enable`: 是否启用外层终点恢复；当前实物主实验默认关闭

### 0.3 slosh 模型相关参数

- 容器与液体参数：
  - `slosh/container_radius`: 容器内半径
  - `slosh/liquid_height`: 静止液面高度
  - `slosh/liquid_density`: 液体密度
  - `slosh/damping_ratio`: 主模态阻尼比
  - `slosh/mode_index`: 当前采用的晃动模态阶数
- 模型口径：
  - `slosh/use_linear_model`: 是否使用线性高度系数映射；当前不切换传播动力学
  - `slosh/use_parabola_term`: 是否在高度监测中叠加 `R^2\omega^2/4g` 抛物面项
- 安装偏心：
  - `slosh/offset_x`: 容器相对机体旋转中心的 X 偏心
  - `slosh/offset_y`: 容器相对机体旋转中心的 Y 偏心

### 0.4 IMU / 实验覆盖常用参数

这些通常优先在 `slosh_experiment.launch` 里改：

- `Q_slosh`: 实验时覆盖 `mpc/Q_slosh`，控制 slosh 软代价是否启用及其强度
- `slosh_height_ref`: 实验时覆盖 `mpc/slosh_height_ref`，决定多大模型液面高度开始被 MPC 明显关注
- `slosh_eta_dot_ratio`: 实验时覆盖 `mpc/slosh_eta_dot_ratio`，控制残余模态速度项权重
- `enable_slosh_box_constraint`: 实验时覆盖盒约束开关
- `slosh_use_imu_yaw_rate`: 是否用 IMU `angular_velocity.z` 替代 odom `omega`
- `slosh_use_imu_lateral_accel`: 是否用 IMU `linear_acceleration.y` 替代 `v*omega`
- `slosh_use_imu_alpha_z`: 是否用 IMU 差分得到的 `alpha_z`
- `slosh_imu_topic`: IMU 话题名，默认 `/imu/data`
- `slosh_imu_filter_alpha`: IMU 输入的 EMA 滤波系数
- `slosh_imu_ay_bias_compensation_enable`: 是否启用 `ay` 静止零偏扣除
- `slosh_imu_ay_bias_init_duration`: 估计 `ay` 零偏所需的连续静止时间
- `slosh_speed_governor_enable`: 是否启用外环残余晃动感知速度治理

如果你只是做实物实验切换，优先改 launch 覆盖值；如果你要改“默认系统行为”，再回到 `mpc_params.yaml`。

### 0.5 上游参考路径语义

当前 `scout_local_planner` 的上游输入是 `/scout/global_path`，类型为 `nav_msgs/Path`。

这条路径本质上提供的是**几何路径**：

- `x`
- `y`
- `pose orientation`（可视为 `yaw` 入口）

它**不直接提供**：

- 速度轨迹
- 角速度轨迹
- 时间参数化轨迹

因此当前项目不是“上游直接给完整时参参考轨迹，MPC 原样跟踪”，而是：

1. 上游给几何路径；
2. `PathHandler` 在本地做截窗、样条拟合和参考生成；
3. 中间参考点里的 `theta_path` 由局部样条切线在线计算；
4. `time_parameterize=true` 时，再由 `PathHandler` 内部生成 `v(s)` 并按 `dt` 采样成 `v_ref`。

要特别区分两类“航向”：

- 路径中间参考点的航向，当前主要是样条切线方向 `theta_path`
- 终点姿态判定时，当前优先使用 goal pose 自身 `orientation`
- 只有当 goal `orientation` 不可用时，才回退到路径尾部切线方向

### 0.6 执行层速度参考链路

当前进入 MPC horizon 的速度参考不是单一来源，而是一个分层链路：

```text
vehicle v_nominal
  -> risk_scheduler / terminal_slowdown / feasibility guard / curvature cap
  -> v_des_target
  -> v_des_rate_limit
  -> v_des_eff
  -> PathHandler getReferencePoints()
  -> horizon 内 v_ref
```

其中：

- `v_des_raw`：名义速度或 risk scheduler 输出的原始速度上限；
- `v_des_target`：经过 terminal、feasibility、curvature、governor 等上游逻辑裁剪后的目标上限；
- `v_des_eff`：经过 `v_des_rate_limit` 平滑后的实际执行层速度上限；
- `v_ref`：`PathHandler` 在 horizon 内按路径弧长、曲率、末端制动距离和 `v_des_eff` 生成的参考速度序列。

所以 `Q_v` 惩罚的并不是 `v_des_raw`，而是 horizon 每一步的：

\[
(v_k-v_{\text{ref},k})^2
\]

`v_des_rate_limit` 不是 MPC cost 项，也不是硬约束；它是进入参考生成器之前的外层速度上限平滑器。
它的作用是减少参考速度一帧突降/突升带来的纵向 \(a_x\) 脉冲，让后续晃动对比更干净。

对应诊断话题：

- `/reference/v_des_raw`
- `/reference/v_des_target`
- `/reference/v_des_eff`
- `/reference/v_des_rate_limited`
- `/reference/v_ref_horizon`
- `/reference/s_horizon`

## 1) 当前项目 MPC 的优化问题

设预测步长为 \(N\)，状态维度 \(n_x=8\)，控制维度 \(n_u=2\)。

- 状态：
\[
\mathbf x_k=
\begin{bmatrix}
 e_{l,k}&e_{c,k}&e_{\theta,k}&v_k&\eta_{x,k}&\dot\eta_{x,k}&\eta_{y,k}&\dot\eta_{y,k}
\end{bmatrix}^\top
\]
- 控制：
\[
\mathbf u_k=
\begin{bmatrix}a_k&\omega_k\end{bmatrix}^\top
\]

决策变量采用 OSQP 结构化堆叠：
\[
\mathbf z=[\mathbf x_0,\mathbf u_0,\mathbf x_1,\mathbf u_1,\dots,\mathbf x_{N-1},\mathbf u_{N-1},\mathbf x_N]
\]

QP 形式：
\[
\min_{\mathbf z}\ \frac12\mathbf z^\top \mathbf P\mathbf z+\mathbf q^\top\mathbf z
\quad
\text{s.t.}\quad
\mathbf l\le \mathbf A\mathbf z\le \mathbf u
\]

## 2) 运动学/动力学模型（项目特化）

### 2.1 基础 Frenet 跟踪子系统（4 维）

离散更新：
\[
\begin{aligned}
e_{l,k+1}&=e_{l,k}+\Delta t\,(v_k-v_{\text{path},k})\\
e_{c,k+1}&=e_{c,k}+\Delta t\,v_k e_{\theta,k}\\
e_{\theta,k+1}&=e_{\theta,k}+\Delta t\,(\omega_k-\kappa_k v_k)\\
v_{k+1}&=v_k+\Delta t\,a_k
\end{aligned}
\]

### 2.2 液体晃动增广子系统（4 维）

晃动状态定义：
\[
\mathbf x^{\text{slosh}}_k=
\begin{bmatrix}
\eta_{x,k}&\dot\eta_{x,k}&\eta_{y,k}&\dot\eta_{y,k}
\end{bmatrix}^\top
\]
其中：
- \(\eta_x,\eta_y\)：主模态**广义坐标**（不是最终液面高度）
- \(\dot\eta_x,\dot\eta_y\)：对应模态速度

当前实现里，模型估计的 modal height 为：
\[
h_{\text{modal}}=h_{\text{coeff}}\sqrt{\eta_x^2+\eta_y^2}
\]

若启用抛物面项，监测高度还会再叠加：
\[
h_{\text{parabola}}=\frac{R^2\omega^2}{4g}
\]

因此 \(\eta_x,\eta_y\) 本身不是液面高度，而是构成液面代理量的模态状态。

离散模型：
\[
\mathbf x^{\text{slosh}}_{k+1}=\mathbf A_s\mathbf x^{\text{slosh}}_k+\mathbf B_s
\begin{bmatrix}
a_k\\a_{y,k}
\end{bmatrix},
\quad a_{y,k}\approx v_k\omega_k
\]

因此该项目是“Frenet 跟踪 + 晃动线性子系统”增广模型，并在 \(a_y=v\omega\) 处发生耦合。

工程上还要补一条当前实现的边界条件：

- 默认配置里 `slosh/offset_x = 0`、`slosh/offset_y = 0`
- 因而在线估计侧 `LiquidSloshModel::update()` 里的旋转修正
\[
a_{cx}=a_x-\alpha_z r_y-\omega_z^2 r_x,\quad
a_{cy}=a_y+\alpha_z r_x-\omega_z^2 r_y
\]
会退化为 \(a_{cx}=a_x,\ a_{cy}=a_y\)
- 所以在当前零偏心配置下，`alpha_z` 与 `yaw_rate` 不会实质改变 **modal state propagation**
- 但 `yaw_rate` 仍会通过抛物面项进入当前高度监测，因此在 governor 打开时仍可能通过风险链路间接影响控制

这也是为什么当前版本里，真正直接改变模态状态演化的 IMU 通道主要是 `a_y`，而不是 `alpha_z`。

进一步说，当前 QP 在每个 horizon 上实际使用的是：
\[
\mathbf x_{k+1}=\mathbf A_k\mathbf x_k+\mathbf B_k\mathbf u_k+\mathbf c_k
\]

这里的 \((\mathbf A_k,\mathbf B_k,\mathbf c_k)\) 不是固定常数，而是围绕当前名义轨迹逐步线性化得到的时变仿射模型。  
根源就是 \(a_y=v\omega\) 这类乘积项会让动力学耦合随步变化。

若后续试管不再位于机体旋转中心，且准备把 `offset_x/offset_y` 设为非零，则必须把同样的偏心旋转修正同步并入预测模型与线性化。  
否则在线估计器和 MPC 优化器将对应两套不同的物理世界：前者按偏心容器传播，后者仍按中心容器传播。

## 3) 代价函数（项目特化）

可写为：
\[
J=\sum_{k=0}^{N}\left(
\mathbf x_k^\top\mathbf Q_k\mathbf x_k+\mathbf q_k^\top\mathbf x_k
\right)
+\sum_{k=0}^{N-1}\left(
\mathbf u_k^\top\mathbf R\mathbf u_k+\mathbf r_k^\top\mathbf u_k
\right)
+\sum_{k=0}^{N-1}\Delta\mathbf u_k^\top\mathbf R_\Delta\Delta\mathbf u_k
\]

其核心项包括：
- 跟踪项：\(e_l,e_c,e_\theta,v\)；
- 控制项：\(a,\omega\)；
- 控制变化率项：\(\Delta a,\Delta\omega\)；
- 液体软代价项：
$$
J_{\text{slosh}}
=Q_{\text{slosh},\eta}(\eta_x^2+\eta_y^2)
+Q_{\text{slosh},\dot\eta}(\dot\eta_x^2+\dot\eta_y^2)
$$

其中当前归一化实现为：

$$
Q_{\text{slosh},\eta}
=Q_{\text{slosh}}\cdot\frac{h_{\text{coeff}}^2}{h_{\text{ref}}^2}
$$

$$
Q_{\text{slosh},\dot\eta}
=\lambda_{\dot\eta}\cdot
\frac{Q_{\text{slosh},\eta}}{\omega_n^2}
$$

这里：

- \(h_{\text{ref}}\) 对应 `mpc/slosh_height_ref`，默认 `0.005 m`
- \(\lambda_{\dot\eta}\) 对应 `mpc/slosh_eta_dot_ratio`，默认 `0.3`
- 当 `slosh_eta_dot_ratio <= 0` 时，`mpc/Q_slosh_eta_dot` 作为旧的手动兼容入口使用

也就是说，软代价直接惩罚的是主模态广义坐标及其模态速度对应的二次型，而不是把“总液面高度”本身直接作为 QP 状态去惩罚。
归一化后的物理含义更接近：

- \((h_{\text{modal}}/h_{\text{ref}})^2\)：模型 modal 液面高度相对参考高度的风险
- \((\dot\eta/\omega_n)^2\)：把模态速度折算成等效位移后的残余晃动风险

若按“标量惩罚项个数”理解，当前代码更准确的口径是：

- 基础 tracking cost：4 项
  - \(e_l\) / \(e_c\) / \(e_\theta\) / \((v-v_{\text{ref}})\)
- 基础 control cost：2 项
  - \(a\) / \(\omega\)
- 基础 control-rate cost：2 项
  - \(\Delta a\) / \(\Delta\omega\)

因此默认基础骨架可以理解为 **4 + 2 + 2 = 8 项**。

在此之上，还有两类可选增强：

- slosh 软代价：默认归一化版本为 4 个状态项
  - \(\eta_x^2\) / \(\eta_y^2\)
  - \(\dot\eta_x^2\) / \(\dot\eta_y^2\)
- `omega_ff` 项：1 项
  - \((\omega-\omega_{\text{ref}})^2\)

所以：

- 不开 slosh、不开 `omega_ff` 时，可理解为 **8 项基础惩罚**
- 只开 eta 型 slosh 时，可理解为 **8 + 2 = 10 项**
- 使用当前默认归一化 eta + eta_dot slosh 时，可理解为 **8 + 4 = 12 项**
- 再开 `omega_ff` 时，可理解为 **13 项**

这里还要补一条当前实现细节：

- terminal ramp 不是“新增 cost 项”
- 它只是把预测域末段已有的 \(e_c/e_\theta/v\) 权重渐进放大

因此论文或文档里若要写“当前 cost 有多少项”，建议写成：

- **基础 8 项 + 可选 2/4 项 slosh + 可选 1 项 omega feedforward**

比简单写成一个固定整数更准确。

## 4) 约束系统（项目特化）

### 4.1 等式约束

- 初值约束：\(\mathbf x_0=\mathbf x_{\text{meas}}\)
- 动力学约束：\(\mathbf x_{k+1}=\mathbf A_k\mathbf x_k+\mathbf B_k\mathbf u_k+\mathbf c_k\)

### 4.2 不等式约束

- 状态边界：\(v_{\min}\le v_k\le v_{\max}\)
- 控制边界：\(|a_k|\le a_{\max},\ |\omega_k|\le \omega_{\max}\)
- 变化率边界（可开关）：
\[
|\omega_k-\omega_{k-1}|\le \alpha_{\max}\Delta t,
\quad
|a_k-a_{k-1}|\le j_{\max}\Delta t
\]
- 液体盒约束（可开关）：
\[
-\bar\eta\le \eta_{x,k}\le \bar\eta,
\quad
-\bar\eta\le \eta_{y,k}\le \bar\eta
\]

这里的 \(\bar\eta\) 按当前实现不是简单取：
\[
\bar\eta = \frac{h_{\max}}{h_{\text{coeff}}}
\]

而是先为抛物面项预留预算，再把剩余 modal budget 均分到 \(\eta_x,\eta_y\) 两个方向：
\[
h_{\text{parabola,budget}}=\frac{R^2\omega_{\max}^2}{4g}
\]
\[
h_{\text{modal,budget}}=\max\left(0,\ h_{\max}-h_{\text{parabola,budget}}\right)
\]
\[
\bar\eta=\frac{h_{\text{modal,budget}}}{h_{\text{coeff}}\sqrt{2}}
\]

因此当前硬约束本质上是 **modal proxy**，不是对总液面高度的直接 hard constraint。

## 5) 回答核心问题：没有“液面约束层”会怎样？

“液面约束层”可分两层看：

1. **硬约束层**（box constraint）关闭：
   - 仅移除 \(\eta_x,\eta_y\) 的 modal proxy 不等式边界；
   - 若软代价仍在，抑制变软，不保证峰值不越界。

2. **软代价层**（\(Q_{\text{slosh},\eta}\)）也关闭：
   - 优化器不再主动惩罚晃动；
   - 晃动状态仅作为“随动动态变量”存在，对目标函数无直接影响；
   - 此时系统行为近似回到“纯路径跟踪 MPC”（只管跟踪与控制平滑/边界）。

3. 若进一步不注入 slosh 模型（回到 4 维状态）：
   - 即恢复无增广的标准 Frenet 跟踪结构。

## 6) 通用 MPC 与本项目 MPC 的数学化对比

### 6.1 通用 MPC（抽象）

\[
\begin{aligned}
\min_{x_{0:N},u_{0:N-1}}\ &\sum_{k=0}^{N-1}\ell(x_k,u_k)+\ell_f(x_N)\\
\text{s.t. }&x_{k+1}=f(x_k,u_k),\\
&g(x_k,u_k)\le 0,\ h(x_N)\le 0
\end{aligned}
\]

### 6.2 本项目 MPC（工程落地）

- 采用**successive linearization 后的 affine time-varying QP**（OSQP）；
- 状态是 **8 维增广 Frenet+slosh**；
- 约束是“动力学等式 + 速度/角速度/加速度/变化率边界 + 可选 slosh modal proxy 盒约束”；
- 目标是“路径跟踪 + 控制平滑 + 可选晃动软抑制”。

即：本项目是通用 MPC 在移动机器人路径跟踪场景下的一个“结构化 QP 特例”，而非自由空间 NMPC。

## 7) 监测量与终点行为的工程语义

### 7.1 `/slosh/height`

当前 `/slosh/height` 的语义是：

\[
h_{\text{now}} = \eta_{\text{slosh}} + \eta_{\text{parabola}}
\]

其中：

\[
\eta_{\text{slosh}} = \text{height\_coeff}\cdot\sqrt{x_n^2+y_n^2}
\]
\[
\eta_{\text{parabola}} = \frac{R^2\omega_z^2}{4g}
\quad (\text{仅当 use\_parabola\_term=true})
\]

工程上更准确的解释是：

- 它是**当前时刻模型估计的液面最大抬升高度**
- 它是**相对静止液面**的高度增量
- 它是**模型输出量**，不是视觉/传感器实测值

因此它可以理解成：

- 当前模型认为“液面最高点大约抬了多少”

但不要把它误写成：

- 已被相机直接测到的真实最高点
- 完整 3D 液面重建结果

### 7.2 `/slosh/height_pred_max`

当前 `/slosh/height_pred_max` 的语义是：

\[
\max_{k\in[0,N]}\left(h_{\text{modal},k}+h_{\text{parabola},k}\right)
\]

也就是“预测域内 modal height 再叠加 predicted \(\omega\) 对应 parabola term”的监测值。  
它用于监测和调试，不等价于“QP 已经对总液面高度直接施加了 hard constraint”。

与 `/slosh/height` 的区别是：

- `/slosh/height`
  - 当前时刻的模型瞬时估计
- `/slosh/height_pred_max`
  - 整个预测时域内的最大监测值

因此：

- 如果要看“当前这一刻模型估计了多少”，优先看 `/slosh/height`
- 如果要看“这一拍 MPC 预判未来最危险会到多高”，优先看 `/slosh/height_pred_max`
- 如果要看“horizon 内 eta/eta_dot 是否真的被预测出来、在哪一步达到峰值”，优先看 `/mpc/slosh_horizon_summary`

### 7.3 `/mpc/slosh_horizon_summary`

当前 `/mpc/slosh_horizon_summary` 是 MPC 求解后发布的预测域摘要，用于检查 slosh 项是否真的进入 horizon 并形成可见代价。

消息类型为 `std_msgs/Float32MultiArray`，当前字段顺序是：

| index | 含义 |
|---:|---|
| 0 | 初始 eta 范数，单位 m |
| 1 | horizon 内 eta 范数最大值，单位 m |
| 2 | 初始 eta_dot 范数，单位 m/s |
| 3 | horizon 内 eta_dot 范数最大值，单位 m/s |
| 4 | horizon 内 modal height 最大值，单位 mm |
| 5 | horizon 内 modal + parabola 总高度最大值，单位 mm |
| 6 | 总高度最大值出现的 horizon index |
| 7 | horizon 内 `|v|` p95，单位 m/s |
| 8 | horizon 内 `|omega|` p95，单位 rad/s |
| 9 | horizon 内 `|ax|` p95，单位 m/s² |
| 10 | horizon 内 `|ay|` p95，单位 m/s² |
| 11 | eta growth ratio，约等于 `eta_norm_max / eta_norm_0` |
| 12 | horizon 初始总高度，单位 mm |

这个 topic 主要回答三个问题：

1. 当前 horizon 内的 eta/eta_dot 是否不是全零；
2. slosh 峰值是否发生在未来步，而不是只来自当前状态；
3. 预测到的高风险是否和 `v/omega/ax/ay` 激励同时出现。

它仍然是模型内部诊断，不是实物液面真值。实物结论仍然必须以 RGB 视觉液面为主。

### 7.4 与视觉 `height_peak_rel_mm` 的关系

当前视觉链输出的 `height_peak_rel_mm` 更准确的语义是：

- 单相机侧视观测平面内
- 当前帧最高表观液面
- 相对静止液面的抬升量

补充当前工程口径：

- 若 calibration 的 `mm_per_pixel` 仍为空，则当前主回归通常先看 `height_peak_rel_px`
- 只有在标尺补齐、`mm_per_pixel` 非空时，`height_peak_rel_mm` 才是可直接使用的毫米口径

它不是：

- 模型内部状态
- 预测域峰值上界
- 完整 3D 真液面高度

因此当前比较关系应写成：

- 逐帧对齐比较：
  - `height_peak_rel_mm` vs `/slosh/height`
- 峰值包络/保守性比较：
  - `height_peak_rel_mm` vs `/slosh/height_pred_max`
- horizon 行为诊断：
  - `height_peak_rel_mm` vs `/mpc/slosh_horizon_summary` 中的 `h_total_max_mm` / `eta_growth_ratio`

也就是说：

- `/slosh/height` 更适合作为视觉逐帧误差的主比较对象
- `/slosh/height_pred_max` 更适合作为“模型预测上界是否保守”的参考对象
- `/mpc/slosh_horizon_summary` 更适合作为“QP horizon 里是否真的产生了 slosh 预测”的调试对象

### 7.5 near-goal 行为

当前 near-goal 控制结构可以直接表述为：

1. 远场仍由 tracking MPC 生成控制；
2. 执行层速度上限先经过 `v_des_rate_limit`，避免上游限速一帧突变后直接进入 horizon；
3. 接近终点但尚未进入捕获区时，`terminal_slowdown` 会压低参考速度并提高末端速度权重，用于让 MPC 提前减速；
4. 进入 `terminal_capture_stop_distance` 后，若 `terminal_capture_stop_enable=true`，外层进入 `CAPTURE_BRAKE`，目标速度为 0，但通过 `terminal_cmd_v_rate_limit / terminal_cmd_omega_rate_limit` 逐步刹停；
5. 速度和角速度低于 `REACHED` 门槛后，才切到 `REACHED`；
6. 若发生过冲、位置未收敛或需要最终恢复，可以手动开启外层 terminal recovery；
7. terminal recovery 当前有 3 个 mode：
   - `ALIGN_TO_POINT`
   - `APPROACH_POINT`
   - `ALIGN_FINAL_YAW`
8. `CAPTURE_BRAKE` 和 recovery 下，控制不是由 MPC 求解器给出，而是外层显式控制律直接发布 `cmd_vel`。

因此当前实现的工程语义是：

- 远场：tracking MPC
- 执行层速度平滑：`v_des_rate_limit` 限制进入 `PathHandler` 的速度上限变化率
- 终点前减速：`terminal_slowdown` 修改参考速度和末端速度权重
- 捕获区停车：`terminal_capture_stop` 进入 `CAPTURE_BRAKE`，按命令变化率限制刹停
- 兜底恢复：外层 terminal recovery，当前实物主实验默认关闭

所以分析“slosh cost 是否改变 MPC 行为”时，要尽量把统计窗口截在 capture stop / terminal recovery 之前。
否则终点阶段的 `CAPTURE_BRAKE` 或外层恢复控制会把 MPC cost 的因果关系混进去。

当前建议的终点/速度诊断优先看：

- `/terminal/mode`
- `/reference/v_des_raw`
- `/reference/v_des_target`
- `/reference/v_des_eff`
- `/reference/v_des_rate_limited`
- `/reference/v_ref_horizon`
- `/reference/implied_ax`
- `/cmd_vel`
- `/scout/odom`
