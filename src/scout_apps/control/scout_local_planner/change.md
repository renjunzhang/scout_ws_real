# 2026-02-02
codex resume 019c1d22-5c8c-7852-a9ca-2a2646325970
- 约束维度修复（`include/scout_local_planner/constraint_manager.h/.cpp`）：
  - `StateBoundsConstraint::numConstraints()` 改为 1，仅约束 `v`，与实际 `evaluate()`/上下界一致。
  - 约束构建时保持 A/l/u 行数严格匹配。
- 控制变化率硬约束（`constraint_manager.{h,cpp}` + `mpc_solver.cpp`）：
  - 新增 Δω 约束（可选 Δa），k=0 使用 `u_prev_` 做边界：`u0 - u_prev ∈ [-alpha_max*dt, +alpha_max*dt]`。
  - 约束总数计算与 `buildQPConstraints()` 对齐，避免维度不一致。
- 线性化改进（`src/mpc_solver.cpp`）：
  - 从 `z_prev_` 恢复整段 `u_prev_seq`，滚动预测得到名义 `x_lin`；
  - 每个 k 用 `(x_lin, u_prev_seq[k])` 线性化，避免 k>0 用 0。
- 预测轨迹可视化修正（`src/local_planner_ros.cpp`）：
  - `local_path` 发布由“简单积分”改为“参考点 + Frenet 误差”恢复到笛卡尔坐标；
  - 轨迹姿态用 `theta_path + e_theta` 还原。
- PathHandler 性能优化（`path_handler.{h,cpp}`）：
  - 最近点搜索改在 map 坐标系完成，仅对“最近点窗口”做 map->base 变换；
  - 允许窗口仅 2 点时线性样条拟合（避免终点处失败）；
  - `findClosestPointIndex()` 接口新增 `robot_pos` 参数。
- 参数与配置调整：
  - 新增 `mpc.constrain_omega_rate / mpc.constrain_accel_rate`（`config/mpc_params*.yaml`）。
  - 仿真参数：`path_handler.min_ref_speed` 与 `path_handler.goal_speed` 设为 0，终点可停车（`config/mpc_params_sim.yaml`）。
- 动力学一致性与终点稳定性增强：
  - `DiffDriveModel::predict()` 去掉 `v` 硬裁剪，避免非光滑动力学引起抖动。
  - `DiffDriveModel::linearize()` 使用 `c = predict(x,u) - A*x - B*u`，保证双线性项线性化一致。
  - `PathHandler::getReferencePoints()` 修复 time_parameterize 的 v_ref 回退逻辑，仅在速度曲线无效时 fallback。
  - `s_global_` 在前进时禁止倒退（ds<0 钳制），降低参考点抖动。
- 参考与可视化优化：
  - `local_path` 发布支持 map frame（内部 base->map 变换，避免 RViz 误判）。
  - time_parameterize 模式下按每步 `v_ref` 推进弧长，终点更平滑。
- 路径缓存与 warm-start：
  - `PathHandler` 缓存 `global_points_map_ / global_path_s_`，避免每周期重采样与弧长重算。
  - 新路径/跳变触发 reset hint，`LocalPlannerROS` 重置 warm-start。
  - `MPCSolver::resetWarmStart()` 新增，可选择保留/清零 `u_prev_`。
- 配置补充：
  - 新增 `mpc.terminal_factor_ec / terminal_factor_etheta / terminal_factor_v`（终端权重）。
  - 新增 `vehicle.j_max`，Δa 约束使用 `j_max * dt`。
  - 仿真 `max_tan_accel/max_tan_decel` 下调至 `<= a_max`。
- 起点对齐优化：
  - `heading_align` 仅在起点附近生效，新增 `heading_align/start_distance`；
  - 超过起点距离阈值后强制关闭对齐，避免终点附近被原地对齐锁住。
- 参数调整（仿真配置）：
  - `vehicle.v_min` 设为 0 禁止倒车，避免终点附近负速度抖动。
  - `vehicle.a_max` 与 `path_handler.max_tan_accel/decel` 提高到 2.0，缩短制动距离，避免过早减速。
  - 降低 `mpc.terminal_factor_etheta` 与 `mpc.terminal_factor_v`，减少终点前过早原地转向。
- 终点前提前停的修复方案（待实施，先记录）：
  - **优先方案（根因修复）**：
    - 解耦速度参考与几何前视：`v_ref` 用 `s_progress` 查表，不叠加 `lookahead`；几何采样仍用 `s_geom = s_progress + lookahead`。
    - `s_proj` 从“最近离散点弧长”改为“最近线段投影弧长”，避免靠近终点时 `s` 突跳到末端。
  - **兜底方案（输出端防死区）**：
    - 非终点停区，强制 `|v_cmd| >= min_cmd_speed`；停区内允许 `v_cmd -> 0`。
    - 建议新增 `output/min_cmd_speed`、`output/stop_zone_dist` 等参数，数值如 `0.06` / `0.25`。
- 终点前提前停修复（已实施）：
  - `PathHandler` 新增线段投影弧长 `projectToPathS()`，用连续弧长更新 `s_global_`，避免靠近终点 `s` 突跳。
  - time_parameterize 模式解耦：`v_ref` 基于 `s_progress`，几何采样仍用 `s_progress + lookahead`，避免提前衰减速度参考。

待办（文档输出）：
- 需要按“源码讲解与技术审计”要求输出完整说明，覆盖架构/状态控制/路径处理/模型/代价/约束/求解器/输出/对标等 9 大节，逐文件给出路径、关键函数、代码片段与维度定义。

# 2026-02-02 源码讲解与技术审计输出（按代码实现）

## 1) 项目总体架构与数据流
- 入口与调用链：`LocalPlannerROS` -> `PathHandler` -> `MPCSolver` -> `OSQP` -> `cmd_vel`。
  - 代码位置：`src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp` ( `LocalPlannerROS::controlLoop()` )。
  - 关键逻辑（伪代码）：
    ```cpp
    getReferencePoints(N, dt, v_des, refs)
    getFrenetState(frenet)
    current_state = [e_l, e_c, e_theta, v, slosh...] 
    mpc_solver_.setPreviousControl(last_control_)
    solution = mpc_solver_.solve(current_state, refs)
    publishCmdVel(solution.v_cmd, solution.omega_cmd)
    ```
- 主要模块职责：
  - `LocalPlannerROS`：ROS I/O、状态机、参数加载、控制循环、发布 cmd_vel / local_path / mpc_status。
  - `PathHandler`：全局路径缓存、局部窗口拟合、Frenet 误差与参考点生成、速度剖面 v(s)。
  - `MPCSolver`：构建 QP（H/g/A/l/u）、线性化与约束拼装、OSQP 求解与 warm-start。
  - `CostFunction / ConstraintManager / DiffDriveModel`：分别负责代价、约束、动力学。
- Topic 与参数加载：
  - 输入：`global_path` (nav_msgs/Path), `odom` (nav_msgs/Odometry)。
  - 输出：`cmd_vel`, `local_path`, `mpc_status`，可选 `global_path_smooth`。
  - YAML -> params：`LocalPlannerROS::loadParameters()` 从 `mpc/*`、`vehicle/*`、`path_handler/*` 等加载到 `MPCParams/VehicleParams/PathHandlerParams`。

## 2) 状态、控制、参考与坐标系
- 状态向量 `x_k`（`include/scout_local_planner/types.h`，`StateIndex`）：
  - 维度 `nx = 8`：`[e_l, e_c, e_theta, v, eta_x, eta_x_dot, eta_y, eta_y_dot]`。
  - 单位：`e_l/e_c` m，`e_theta` rad，`v` m/s，晃动 `eta_*` m / m/s。
- 控制向量 `u_k`（`ControlIndex`）：
  - 维度 `nu = 2`：`[a, omega]`，单位 m/s^2 与 rad/s。
- 决策变量 `z` 排列（`mpc_solver.cpp::buildQP`）：
  - `z = [x0, u0, x1, u1, ..., xN]`，`nz = N*(nx+nu) + nx`。
  - 索引：`x_idx = k*(nx+nu)`, `u_idx = x_idx + nx`。
- 参考点 `ReferencePoint`（`types.h`）：
  - 字段：`x,y,theta_path,kappa,v_path,v_ref,s`。
  - 来源：`PathHandler::getReferencePoints()` 从局部样条 `CubicSpline2D` 采样；`v_ref` 来自 v(s) 或 `v_des`。
- 坐标系与 TF：
  - 全局路径在 `map`（或 `global_path.header.frame_id`）；`PathHandler` 使用 `map->base` 变换找最近点并拟合局部样条（base 坐标）。
  - Frenet 误差在 `base_link` 下计算（机器人位姿为原点、朝向=0）。
  - `local_path` 发布优先用 `map_frame_`，若 TF 不可用则回落 `base_frame_`。

## 3) 路径处理与“跟踪的路径”
- 全局路径缓存/重采样（`path_handler.cpp::updateGlobalPath()`）：
  - 缓存 `global_points_map_`，可按 `resample_spacing` 重采样。
  - 计算 `global_path_s_`（累计弧长），并构建 `global_spline_`（速度剖面用）。
- 局部窗口与样条（`PathHandler::getReferencePoints()`）：
  - 最近点查找：`findClosestPointIndex()` 在 map 坐标中基于上次索引搜索。
  - 弧长投影：`projectToPathS()` 用线段投影得到连续 `s_proj`，更新 `s_global_`（含单调性与 `s_jump_threshold`）。
  - 窗口：`[idx-window_back, idx+N+window_forward]`，仅对窗口点做 map->base 变换，拟合局部样条。
- time_parameterize 模式速度剖面（`updateSpeedProfile()`）：
  - v(s) 初始化为 `v_des`，按曲率限速 `max_lat_accel`，末端速度设为 `goal_speed`。
  - 前向加速限制、反向减速限制，`min_ref_speed`（末端除外）。
  - `getSpeedAtS(s)` 线性插值。
- lookahead / 速度解耦（已修复）：
  - 速度参考 `v_ref` 使用 `s_progress` 查表（不加 lookahead）。
  - 几何参考 `s_geom = s_progress + lookahead` 用于 `x/y/theta/kappa`。
- s_global/s_proj 终点策略：
  - `s_proj` 用连续投影；`s_global_` 前进时不允许倒退；`goal_reached` 用距离+航向。

## 4) 动力学/运动学模型与离散化
- 模型定义（`diff_drive_model.h/.cpp`）：
  - 连续近似：
    - `e_l_dot = v - v_path`
    - `e_c_dot = v * e_theta`
    - `e_theta_dot = omega - kappa * v`
    - `v_dot = a`
  - 离散化：显式 Euler（`x_{k+1} = x_k + dt * f(...)`）。
  - `predict()` 中不做 v 硬裁剪。
- 线性化（`DiffDriveModel::linearize()`）：
  - `A = I + dt * df/dx`，`B = dt * df/du`。
  - 仿射项：`c = predict(x,u) - A*x - B*u`。
- 名义轨迹：
  - `MPCSolver::buildQP()` 从上次 `z_prev_` 还原 `u_prev_seq`，滚动 `predict()` 得 `x_lin`，逐步线性化。

## 5) 代价函数（QP 目标）
- 目标形式：`min 0.5 z^T H z + g^T z`（`cost_function.cpp::buildQPCost`）。
- 状态误差：
  - `Q_el/Q_ec/Q_etheta/Q_v`，或 `Q_contour/Q_lag`（`use_contour_lag` 时替代）。
  - 线性项：`g_V += -2 * Q_v * v_ref`。
- 控制代价：`R_a, R_omega`。
- 变化率代价：
  - `R_da, R_domega`，跨步耦合：`(u_k - u_{k-1})^2`。
  - `k=0` 使用 `u_prev_`。
- omega_ff：
  - 在 `g` 中加线性项 `-2 * Q_omega_ff * omega_ref`（`omega_ref = v_ref*kappa`）。
  - 注意：当前实现**未**在 `H` 中添加 `Q_omega_ff * omega^2` 二次项。
- 终端权重：
  - `k==N` 时放大 `E_C/E_THETA/V` 对应对角元素，且 `q_V` 同比放大。

## 6) 约束（QP 约束）
- 动力学等式：
  - `x_{k+1} - A_k x_k - B_k u_k = c_k`。
  - 约束矩阵在 `MPCSolver::buildQP()` 中拼装。
- 初始条件：`x_0 = x0`，`nx` 行等式。
- 状态/控制边界（`ConstraintManager::buildQPConstraints()`）：
  - 状态：仅约束 `v`，`v_min ≤ v_k ≤ v_max`，k=0..N。
  - 控制：`-a_max ≤ a_k ≤ a_max`，`-omega_max ≤ omega_k ≤ omega_max`，k=0..N-1。
- 控制变化率约束：
  - `-alpha_max*dt ≤ omega_k - omega_{k-1} ≤ alpha_max*dt`。
  - `k=0` 使用 `u_prev_`。
  - 可选 Δa：`-j_max*dt ≤ a_k - a_{k-1} ≤ j_max*dt`。
- 约束维度：
  - `nz = N*(nx+nu) + nx`。
  - `num_bounds = (N+1)*1 + N*2 + (enable_omega_rate?N:0) + (enable_accel_rate?N:0)`。
  - `nc = nx + N*nx + num_bounds`。
- 不可行降级：`LocalPlannerROS` 在失败时按 `infeasible_decel` 减速，`omega *= infeasible_omega_scale`。

## 7) 求解器与数值实现
- OSQP 旧 API（`mpc_solver.cpp`）：
  - 每周期重新生成 `P/A/q/l/u` CSC 数据，更新工作空间。
  - 设置：`warm_start=true, polish=true, eps_abs=1e-4, eps_rel=1e-4, max_iter=4000`。
  - 结构变化会触发重新 `setup`。
- 热启动：
  - `z_prev_` 前移一拍作为初始猜测（`warmStart()`）。
- 稀疏结构：
  - `H`/`A` 通过 triplets 生成；`Q/R` 仅对角，但实现未跳过 0 值（结构可能偏稠）。

## 8) 控制输出与执行层
- 控制量提取：
  - `u_first = u_optimal[0]`；`omega_cmd = u_first(OMEGA)`。
  - `v_cmd = x_predicted[1](V)`（注意使用的是 k=1 的 v 状态）。
- 发布：
  - `publishCmdVel(v_cmd, omega_cmd)`；频率 = `control_rate`。
  - `local_path` 用 `ref + Frenet误差` 恢复笛卡尔轨迹；可用 map frame。
- 状态机：`IDLE/TRACKING/REACHED/ERROR`；`isGoalReached()` 基于终点距离 + 航向误差（末段切向方向）。

## 9) 对标 MPCC/ mpc_planner 的差异
- 当前实现没有把进度 `s` 作为优化变量；`s_progress` 由 `PathHandler` 外部计算，未进入 QP。
- “contour/lag” 仅体现在权重结构，仍属于固定参考轨迹跟踪。
- 若升级为 MPCC：需要引入 `s`（或 progress）状态/控制，添加进度动力学、路径约束和可行性域（道路走廊/障碍），并将 `contour/lag` 与 `s` 绑定。

## 10) 快速自检清单（易导致偏差/不丝滑/不收敛）
1. `s_progress/total_len` 是否过早接近末端（打印 `s_progress, total_len, v_ref`）。
2. `lookahead_distance` 过大导致切弯（打印 `ref.s` 与曲率）。
3. `max_lat_accel` 过高导致弯道超速（打印 `v_ref` 与 `kappa`）。
4. `R_omega/R_domega` 过大导致转向跟不上（观察 `omega_cmd` 饱和）。
5. `Q_ec/Q_etheta` 过低导致贴合差（观察 e_c/e_theta 变化）。
6. `alpha_max` 过小导致 ω 变化受限（打印 Δω 约束是否打满）。
7. TF 失效导致路径/姿态不同步（关注 `[PathHandler] TF error` 日志）。
8. 局部样条窗口过短（`Window too small` 警告）。
9. OSQP 重建频繁（`Matrix structure changed` 警告）。
10. 输出 v_cmd 取 `x_predicted[1]`，若模型/线性化异常会导致 v 抖动（打印 `x_predicted[0..2].V`）。
