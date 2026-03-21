# 当前 MPC 数学化架构总结（含通用 MPC 对比）

本文基于当前代码实现，给出可直接对应到工程的数学表达。

补充说明：

- 这是**结构化 QP 的 tracking MPC**，不是自由空间 NMPC。
- 当前每个控制周期求解的并不是固定 LTI QP，而是围绕名义轨迹 successive linearization 后得到的**affine time-varying QP**。
- 终点最后一小段当前也不是“全程由 MPC 连续优化到停下”，而是 near-goal 进入 `goal_stop_pending_` 后由外层状态机直接发布 `0` 速制动。

## 目录

- 0) 参数入口速查
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
  - 7.1 `/slosh/height_pred_max`
  - 7.2 near-goal 行为

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
  - `mpc/Q_slosh`: 晃动抑制主权重，最终映射到 `Q_slosh_eta`

其中：

- `Q_slosh = 0` 表示不加入 slosh 软代价
- `Q_slosh > 0` 表示把 `eta_x / eta_y` 的二次惩罚加入 cost
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
\[
J_{\text{slosh}}=Q_{\text{slosh},\eta}(\eta_x^2+\eta_y^2),
\quad
Q_{\text{slosh},\eta}=Q_{\text{slosh}}\cdot h_{\text{coeff}}^2
\]

也就是说，软代价直接惩罚的是主模态广义坐标对应的二次型，而不是把“总液面高度”本身直接作为 QP 状态去惩罚。

若按“标量惩罚项个数”理解，当前代码更准确的口径是：

- 基础 tracking cost：4 项
  - \(e_l\) / \(e_c\) / \(e_\theta\) / \((v-v_{\text{ref}})\)
- 基础 control cost：2 项
  - \(a\) / \(\omega\)
- 基础 control-rate cost：2 项
  - \(\Delta a\) / \(\Delta\omega\)

因此默认基础骨架可以理解为 **4 + 2 + 2 = 8 项**。

在此之上，还有两类可选增强：

- slosh 软代价：2 项
  - \(\eta_x^2\) / \(\eta_y^2\)
- `omega_ff` 项：1 项
  - \((\omega-\omega_{\text{ref}})^2\)

所以：

- 不开 slosh、不开 `omega_ff` 时，可理解为 **8 项基础惩罚**
- 开 slosh 时，可理解为 **8 + 2 = 10 项**
- 再开 `omega_ff` 时，可理解为 **11 项**

这里还要补一条当前实现细节：

- terminal ramp 不是“新增 cost 项”
- 它只是把预测域末段已有的 \(e_c/e_\theta/v\) 权重渐进放大

因此论文或文档里若要写“当前 cost 有多少项”，建议写成：

- **基础 8 项 + 可选 2 项 slosh + 可选 1 项 omega feedforward**

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

### 7.1 `/slosh/height_pred_max`

当前 `/slosh/height_pred_max` 的语义是：

\[
\max_{k\in[0,N]}\left(h_{\text{modal},k}+h_{\text{parabola},k}\right)
\]

也就是“预测域内 modal height 再叠加 predicted \(\omega\) 对应 parabola term”的监测值。  
它用于监测和调试，不等价于“QP 已经对总液面高度直接施加了 hard constraint”。

### 7.2 near-goal 行为

当前 near-goal 逻辑不能表述成“全程由 MPC 连续优化到停下”。  
更准确的说法是：

1. 远场仍由 tracking MPC 生成控制；
2. 接近终点时，可能先进入外层 terminal recovery；
3. terminal recovery 当前有 3 个 mode：
   - `ALIGN_TO_POINT`
   - `APPROACH_POINT`
   - `ALIGN_FINAL_YAW`
4. 这些 mode 下，控制不是由 MPC 求解器给出，而是外层显式控制律直接发布 `cmd_vel`；
5. 当目标 pose 已达标后，状态机置 `goal_stop_pending_`；
6. `goal_stop_pending_` 下，系统仍然走 MPC，只是把 `v_des_cmd` 压到 `0`，由 MPC 负责最后一段减速与纠偏；
7. 只有当位置/姿态达标且速度、角速度都足够低时，才切到 `REACHED`。

因此当前实现更准确的工程语义是：

- 远场：tracking MPC
- 近终点恢复：外层 terminal recovery
- 最后收尾：`goal_stop_pending_` 下的 MPC 减速

它不是“全程由 MPC 独立完成终点收敛”，也不是旧版本那种“进入目标容差区后直接外层硬切零速”的实现。
