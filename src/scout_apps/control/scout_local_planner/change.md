# 后续接入：液体晃动模型 (Slosh Integration Roadmap)

> 当前状态：`DiffDriveModel` 的 `predict()` / `linearize()` 已包含完整的增广晃动动力学
> （`A_slosh`、`B_slosh`、`ay = v * ω` 离心耦合），`SloshIntegration` 类已完整实现。
> **但尚未接线**：`LocalPlannerROS` / `MPCSolver` 没有实例化 `SloshIntegration`，
> 没有调用 `setSloshIntegration()`，`CostFunction` 也没有添加 slosh 代价项。
> 以下是接入所需的全部步骤。

## 步骤 1：实例化 SloshIntegration 并注入 DiffDriveModel

**文件**：`local_planner_ros.h`、`local_planner_ros.cpp`

```cpp
// === local_planner_ros.h ===
// 新增成员:
#include "scout_local_planner/slosh_integration.h"
SloshIntegration slosh_integration_;

// === local_planner_ros.cpp::loadParameters() ===
// 加载 slosh 参数:
SloshParams slosh_params;
pnh.param("slosh/container_radius",  slosh_params.container_radius, 0.15);
pnh.param("slosh/liquid_height",     slosh_params.liquid_height, 0.20);
pnh.param("slosh/liquid_density",    slosh_params.liquid_density, 1000.0);
pnh.param("slosh/damping_ratio",     slosh_params.damping_ratio, 0.05);
pnh.param("slosh/mode_index",        slosh_params.mode_index, 1);
pnh.param("slosh/offset_x",          slosh_params.offset_x, 0.0);
pnh.param("slosh/offset_y",          slosh_params.offset_y, 0.0);
slosh_params.dt = mpc_params_.dt;  // 与 MPC 步长一致

// === local_planner_ros.cpp::initialize() ===
// 初始化 slosh 模型（在 mpc_solver_.initialize 之前）:
if (mpc_params_.Q_slosh > 0.0) {
    if (slosh_integration_.configure(slosh_params)) {
        ROS_INFO("[LocalPlannerROS] Slosh model configured");
    } else {
        ROS_WARN("[LocalPlannerROS] Failed to configure slosh model, disabling");
        mpc_params_.Q_slosh = 0.0;
    }
}

// 注入到 MPC 求解器内的 DiffDriveModel:
mpc_solver_.setSloshIntegration(&slosh_integration_);  // 需新增此方法
```

## 步骤 2：MPCSolver 传递 SloshIntegration 指针

**文件**：`mpc_solver.h`、`mpc_solver.cpp`

```cpp
// === mpc_solver.h ===
#include "scout_local_planner/slosh_integration.h"
void setSloshIntegration(SloshIntegration* slosh);

// === mpc_solver.cpp ===
void MPCSolver::setSloshIntegration(SloshIntegration* slosh) {
    model_.setSloshIntegration(slosh);  // DiffDriveModel 已有此接口
}
```

> `DiffDriveModel::setSloshIntegration()` 已存在。一旦指针非空且 `isConfigured()`，
> `predict()` 和 `linearize()` 就会**自动**使用增广晃动动力学，无需改动。

## 步骤 3：添加晃动代价项到 CostFunction

**文件**：`cost_function.h`、`cost_function.cpp`

```cpp
// === cost_function.h ===
class SloshCost : public CostTermBase {
public:
    SloshCost(double Q_slosh, const Eigen::Matrix4d& H_slosh);
    std::string name() const override { return "SloshCost"; }
    // ...
private:
    double Q_slosh_;
    Eigen::Matrix4d H_slosh_;  // 从 SloshIntegration::getSloshCostMatrix() 获取
};

// === cost_function.cpp::buildQPCost() 或 initialize() ===
// 条件添加:
if (params_.Q_slosh > 0.0 && slosh_integration->isConfigured()) {
    auto H_slosh = slosh_integration->getSloshCostMatrix(params_.Q_slosh);
    // 在 Q_total 的 [ETA_X:ETA_Y_DOT, ETA_X:ETA_Y_DOT] 子块添加 H_slosh:
    Q_total.block<4,4>(StateIndex::ETA_X, StateIndex::ETA_X) += H_slosh;
}
```

> `SloshIntegration::getSloshCostMatrix(Q_slosh)` 已实现，返回
> $H_{slosh} = Q_{slosh} \cdot h_{coeff}^2 \cdot I_{4\times4}$，
> 将液面高度 $\eta = h_{coeff} \cdot \|x_{slosh}\|$ 的二次惩罚映射到状态空间。

## 步骤 4：初始化增广状态中的晃动分量

**文件**：`local_planner_ros.cpp::controlLoop()`

```cpp
// 当前代码: current_state.setZero() 已将 η 初始化为 0
// 更精确的做法：从 SloshIntegration 的内部状态填充
if (mpc_params_.Q_slosh > 0.0 && slosh_integration_.isConfigured()) {
    slosh_integration_.writeToAugmentedState(current_state);  // 已实现
}

// 在每次 MPC 求解后，用实际控制输入更新 slosh 内部状态（状态估计）:
if (solution.success && mpc_params_.Q_slosh > 0.0) {
    double ax = solution.u_first(ControlIndex::A);
    double ay = current_v_ * solution.omega_cmd;
    slosh_integration_.update(ax, ay, current_omega_);
}
```

## 步骤 5：（可选）液面高度硬约束

**文件**：`constraint_manager.h`、`constraint_manager.cpp`

```cpp
// 在 StateIndex::ETA_X 和 ETA_Y 上添加上下界:
// |η_x| ≤ slosh_height_max / h_coeff
// |η_y| ≤ slosh_height_max / h_coeff
// 或直接对状态 η 添加线性约束行。
// 当前 slosh_height_max = 0.05 m 已在 YAML 配置。
```

## 步骤 6：YAML 配置

**文件**：`config/mpc_params.yaml`、`config/mpc_params_sim.yaml`

```yaml
# 启用晃动抑制:
mpc:
  Q_slosh: 5.0              # > 0 即启用
  slosh_height_max: 0.05    # 液面约束 (m)

# 晃动模型参数（需根据实际容器标定）:
slosh:
  container_radius: 0.15    # 容器内半径 [m]
  liquid_height: 0.20       # 液体静液高度 [m]
  liquid_density: 1000.0    # 液体密度 [kg/m³]
  damping_ratio: 0.05       # 阻尼比
  mode_index: 1             # 模态阶数
  offset_x: 0.0             # 容器偏心距 x [m]
  offset_y: 0.0             # 容器偏心距 y [m]
```

## 关键注意事项

1. **已就绪的代码**（无需改动）：
   - `DiffDriveModel::predict()` — 已有 `ay = v * omega` 离心耦合 + `predictSlosh()`
   - `DiffDriveModel::linearize()` — 已有 `A_slosh`、`B_slosh[:,0]→a`、`B_slosh[:,1]*v→ω`
   - `SloshIntegration` 全部接口 — `configure()`、`getDiscreteMatrices()`、`predictSlosh()`、
     `getSloshCostMatrix()`、`writeToAugmentedState()`、`update()`
   - `types.h` 状态维度 — `TOTAL_DIM=8`，晃动索引 `ETA_X..ETA_Y_DOT` 已定义

2. **需要新写的代码**（约 60 行）：
   - `MPCSolver::setSloshIntegration()` 传递指针（~5 行）
   - `LocalPlannerROS` 实例化 + 参数加载 + 注入（~25 行）
   - `CostFunction` 添加 `H_slosh` 到 Q 矩阵（~15 行）
   - `controlLoop()` 状态填充 + 内部更新（~10 行）

3. **液面高度硬约束**是可选的增强项，初期建议仅用软约束（`Q_slosh`）验证效果。

4. **参数标定**：`container_radius`、`liquid_height`、`damping_ratio` 需根据实际容器测量。
   `damping_ratio` 建议从 0.05 开始，通过实物 step response 实验确定。

5. **性能影响**：增广状态从 4D 变为 8D 有效（当前 QP 维度 408 已按 8D 分配），
   slosh 子系统矩阵是常数（不依赖状态），不增加线性化计算量。

---

# 2025-02-28 — 平滑性改进

## 1. cmd_vel 低通滤波（EMA）
- **文件**：`include/scout_local_planner/local_planner_ros.h`、`src/local_planner_ros.cpp`
- `publishCmdVel()` 增加一阶指数移动平均（EMA）滤波器，消除控制指令高频抖动。
  - `filtered_v = α_v * v + (1 − α_v) * filtered_v_prev`
  - `filtered_omega = α_omega * ω + (1 − α_omega) * filtered_omega_prev`
- 停车指令（v≈0 且 ω≈0）直接下发并重置滤波器状态，避免缓慢趋零。
- `resetWarmStart(false)` 同步清零滤波器状态。
- 新增 YAML 参数 `filter/alpha_v`（默认 0.3）、`filter/alpha_omega`（默认 0.4）。

## 2. 控制频率 10 → 20 Hz
- **文件**：`config/mpc_params.yaml`、`config/mpc_params_sim.yaml`
- `control_rate: 10.0` → `20.0`，与 MPC 预测步长 dt = 0.05 s 对齐，
  消除 "2 个预测周期才出一步控制" 带来的阶梯感。

## 3. 启用 jerk 约束（加速度变化率硬约束）
- **文件**：`config/mpc_params.yaml`、`config/mpc_params_sim.yaml`
- `constrain_accel_rate: false` → `true`
- `j_max: 0.0` → `3.0`（m/s³），激活 `constraint_manager.cpp` 中已有的
  Δa 硬约束逻辑：`|a_k − a_{k−1}| ≤ j_max × dt`。
- `R_da: 0.1` → `0.5`，配合 jerk 硬约束同步加强软惩罚。
- **液体晃动模型接口无影响**：jerk 约束仅作用于加速度控制通道 (ControlIndex::A)，
  不修改 DiffDriveModel 的 slosh 耦合（`a_lat = v * ω`）与 B_slosh 矩阵。

## 4. Frenet 投影 Newton 法精化
- **文件**：`src/path_handler.cpp`、`include/scout_local_planner/cubic_spline.h`
- `computeFrenetProjection()` 在原有粗搜 50 + 细搜 20 之后，追加 **3 步 Newton 迭代**。
  - 目标：最小化 $\|C(s) - P\|^2$，令 $f(s) = (C(s)-P) \cdot C'(s) = 0$。
  - 迭代公式：$s \leftarrow s - f/f'$，其中 $f'(s) = C'(s)^2 + (C(s)-P)\cdot C''(s)$。
  - 收敛精度 $\sim 10^{-8}$ m，消除曲率突变处 ~1 cm 级噪声。
- `CubicSpline2D` 新增 `splineX()` / `splineY()` 公开访问器，供 Newton 法获取一阶/二阶导数。
- **液体晃动接口无影响**：仅改变投影精度，不涉及动力学。

## 5. 速度曲线高斯平滑
- **文件**：`src/path_handler.cpp`
- `updateSpeedProfile()` 在前向/反向遍历之后增加一趟 **滑动窗口均值平滑**
  （半径 5 个采样点），消除曲率突变处 v(s) 的阶梯跳变。
- 平滑结果取 `min(smoothed, original)` 保证不超过安全约束值。
- 仅作用于预处理阶段，不影响运行时 MPC 求解。

## 6. 渐进式终端权重
- **文件**：`src/cost_function.cpp`、`include/scout_local_planner/types.h`、
  `src/local_planner_ros.cpp`、`config/mpc_params.yaml`、`config/mpc_params_sim.yaml`
- 原实现：仅 k==N 时权重放大 → 预测尾部与中段不连续，引起收敛振荡。
- 新实现：**最后 `terminal_ramp_steps` 步线性递增**到 `terminal_factor`。
  - $\alpha(k) = (k - k_{start} + 1) / (ramp\_steps + 1)$
  - $Q(k) = Q_{base} \times [1 + \alpha \times (factor - 1)]$
- `MPCParams` 新增 `terminal_ramp_steps`（默认 1 保持兼容，YAML 设为 5）。
- **液体晃动接口无影响**：仅改变 Q 矩阵对角项数值。

## 7. v/ω 输出半步时间对齐
- **文件**：`src/mpc_solver.cpp`
- 原实现：`v_cmd = x_predicted[1].v`（t+dt）、`omega_cmd = u[0].ω`（t+0）→ 时间不对齐。
- 新实现：
  - $v_{cmd} = v_0 + a_0 \times 0.5 \cdot dt$ — **半步外推**，对齐到 $t + 0.5dt$
  - $\omega_{cmd} = 0.5 \times (\omega_0 + \omega_1)$ — **首两步均值**，对齐到 $t + 0.5dt$
- 消除 v 和 ω 的时间偏移引起的轨迹扭动。
- **液体晃动接口无影响**：仅改变输出提取方式。

---

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

---

# 2026-02-06 MPC 局部规划器完整源码讲解与技术审计

> 文档目的：作为后续维护、调参、升级 MPCC 的技术基线，完整覆盖架构、数学模型、代码实现与易错点。

---

## 1. 项目结构总览

### 1.1 目录布局

```
scout_local_planner/
├── include/scout_local_planner/
│   ├── types.h                  # 状态/控制索引、参数结构、解结构
│   ├── local_planner_ros.h      # ROS 节点封装
│   ├── mpc_solver.h             # QP 构建与 OSQP 接口
│   ├── path_handler.h           # 路径处理、Frenet 误差
│   ├── diff_drive_model.h       # 差速底盘动力学
│   ├── cost_function.h          # 代价函数（H/g 构建）
│   ├── constraint_manager.h     # 约束管理（A/l/u 构建）
│   ├── cubic_spline_2d.h        # 二维三次样条
│   └── slosh_integration.h      # 液体晃动模型接口
├── src/
│   ├── local_planner_ros.cpp    # 入口节点实现
│   ├── mpc_solver.cpp           # QP 核心实现
│   ├── path_handler.cpp         # 路径与 v(s) 实现
│   ├── diff_drive_model.cpp     # 动力学实现
│   ├── cost_function.cpp        # 代价实现
│   └── constraint_manager.cpp   # 约束实现
├── config/
│   ├── mpc_params.yaml          # 实物参数（保守）
│   └── mpc_params_sim.yaml      # 仿真参数（激进）
└── launch/
    ├── scout_local_planner.launch
    └── scout_local_planner_sim.launch
```

### 1.2 模块职责与调用链

```
┌──────────────────────────────────────────────────────────────┐
│                    LocalPlannerROS                           │
│  - ROS I/O, 状态机, 参数加载, 控制循环                        │
│  - 发布: cmd_vel, local_path, mpc_status                     │
└───────────────┬──────────────────────────────────────────────┘
                │ 1. getReferencePoints()
                ▼
┌──────────────────────────────────────────────────────────────┐
│                      PathHandler                             │
│  - 全局路径缓存, 局部样条拟合                                  │
│  - Frenet 误差计算, 速度曲线 v(s) 生成                        │
└───────────────┬──────────────────────────────────────────────┘
                │ 2. solve(x0, refs)
                ▼
┌──────────────────────────────────────────────────────────────┐
│                       MPCSolver                              │
│  - 构建 QP (H, g, A, l, u)                                   │
│  - 调用 OSQP 求解, 热启动                                     │
├──────────────────────────────────────────────────────────────┤
│ CostFunction │ ConstraintManager │ DiffDriveModel           │
│  - buildQPCost()   - buildQPConstraints()   - linearize()   │
└───────────────┬──────────────────────────────────────────────┘
                │ 3. osqp_solve()
                ▼
┌──────────────────────────────────────────────────────────────┐
│                        OSQP                                  │
│  - 稀疏 QP 求解器 (osqp-vendor / osqp-eigen)                 │
└───────────────┬──────────────────────────────────────────────┘
                │ 4. extractSolution()
                ▼
┌──────────────────────────────────────────────────────────────┐
│                     控制输出                                  │
│  v_cmd = x_predicted[1].V                                    │
│  ω_cmd = u_optimal[0].OMEGA （直接控制！）                    │
└──────────────────────────────────────────────────────────────┘
```

---

## 2. ROS 接口与数据流

### 2.1 Topic 订阅

| Topic 名称 | 消息类型 | 作用 |
|-----------|---------|-----|
| `global_path` | `nav_msgs/Path` | 全局规划路径（来自 A*/Dijkstra/TEB） |
| `odom` | `nav_msgs/Odometry` | 里程计（v, ω 来自底盘反馈） |

### 2.2 Topic 发布

| Topic 名称 | 消息类型 | 作用 |
|-----------|---------|-----|
| `cmd_vel` | `geometry_msgs/Twist` | 控制指令（linear.x, angular.z） |
| `local_path` | `nav_msgs/Path` | MPC 预测轨迹（可视化） |
| `mpc_status` | `std_msgs/String` | 状态机与求解状态 |
| `global_path_smooth` | `nav_msgs/Path` | 平滑后的局部样条（可选） |

### 2.3 TF 依赖

```
map → odom → base_link
      └───────┘ 通常由里程计/定位系统维护
```

- `PathHandler` 需要 `map → base_link` 变换
- 局部样条在 `base_link` 坐标系下拟合

### 2.4 参数加载路径

```yaml
mpc/*          → MPCParams
vehicle/*      → VehicleParams
path_handler/* → PathHandlerParams
control_rate   → 控制频率 (Hz)
safety/*       → 不可行时的降级策略
heading_align/* → 原地对齐参数
```

---

## 3. 状态/控制量与维度定义

### 3.1 状态向量 $x_k \in \mathbb{R}^8$

| 索引 | 符号 | 含义 | 单位 |
|------|-----|------|-----|
| 0 | $e_l$ | 纵向误差（lag error） | m |
| 1 | $e_c$ | 横向误差（contour error） | m |
| 2 | $e_\theta$ | 航向误差 | rad |
| 3 | $v$ | 线速度 | m/s |
| 4 | $\eta_x$ | X方向晃动模态位移 | m |
| 5 | $\dot{\eta}_x$ | X方向晃动模态速度 | m/s |
| 6 | $\eta_y$ | Y方向晃动模态位移 | m |
| 7 | $\dot{\eta}_y$ | Y方向晃动模态速度 | m/s |

**关键设计**：$\omega$ **不是状态，而是控制量**！这是实现平滑控制的关键。

### 3.2 控制向量 $u_k \in \mathbb{R}^2$

| 索引 | 符号 | 含义 | 单位 |
|------|-----|------|-----|
| 0 | $a$ | 线加速度 | m/s² |
| 1 | $\omega$ | 角速度（直接控制） | rad/s |

### 3.3 决策变量 $z$

$$z = \begin{bmatrix} x_0 \\ u_0 \\ x_1 \\ u_1 \\ \vdots \\ x_{N-1} \\ u_{N-1} \\ x_N \end{bmatrix} \in \mathbb{R}^{n_z}$$

- 维度：$n_z = N \cdot (n_x + n_u) + n_x = N \cdot 10 + 8$
- 默认 $N=40$：$n_z = 408$

### 3.4 索引公式

```cpp
int x_idx = k * (nx + nu);       // x_k 在 z 中的起始索引
int u_idx = x_idx + nx;           // u_k 在 z 中的起始索引
```

---

## 4. 参考轨迹与 Frenet 误差

### 4.1 路径处理流程

```
updateGlobalPath()
    │
    ├─→ 重采样 (resample_spacing)
    ├─→ 构建全局样条 (global_spline_)
    └─→ 生成速度曲线 v(s) (updateSpeedProfile)
         │
         ├─→ 曲率限速: v ≤ √(max_lat_accel / κ)
         ├─→ 前向加速限制
         ├─→ 反向减速限制 (×0.8 保守系数)
         └─→ 末端 goal_tolerance 范围内速度→goal_speed
```

### 4.2 参考点生成 (`getReferencePoints`)

1. **最近点查找**：在 `map` 坐标系下找 `closest_idx`
2. **弧长投影**：`projectToPathS()` 得到连续 `s_proj` → 更新 `s_global_`
3. **局部窗口**：`[idx-window_back, idx+N+window_forward]`
4. **样条拟合**：仅对窗口点做 `map→base` 变换后拟合 `CubicSpline2D`
5. **参考点采样**：

```cpp
// 速度参考用 s_progress（不加 lookahead），避免提前减速
double v_ref = getSpeedAtS(s_progress);

// 几何参考用 s_geom = s_progress + lookahead
double s_geom = s_progress + lookahead_distance;
ref.x = local_spline_.evaluate(s_geom).x();
ref.kappa = local_spline_.evaluateKappa(s_geom);
```

### 4.3 Frenet 误差计算 (`computeFrenetProjection`)

```cpp
// 在 base_link 坐标系中，机器人在原点
Eigen::Vector2d robot_pos(0, 0);
double robot_theta = 0;

// 找样条上最近点
Eigen::Vector2d closest = spline.evaluate(best_s);
double theta_path = spline.evaluateTheta(best_s);
Eigen::Vector2d error = robot_pos - closest;

// Frenet 分解
Eigen::Vector2d tangent(cos(theta_path), sin(theta_path));
Eigen::Vector2d normal(-sin(theta_path), cos(theta_path));

e_l = error.dot(tangent);      // 纵向误差
e_c = error.dot(normal);       // 横向误差
e_theta = robot_theta - theta_path;  // 航向误差（归一化到 [-π, π]）
```

---

## 5. 动力学模型

### 5.1 连续时间 Frenet 动力学

$$\begin{aligned}
\dot{e}_l &= v - v_{path} \\
\dot{e}_c &= v \cdot e_\theta \\
\dot{e}_\theta &= \omega - \kappa \cdot v \\
\dot{v} &= a
\end{aligned}$$

**注意**：$\omega$ 是控制量，直接作用于 $\dot{e}_\theta$，无需通过 $\alpha$ 积分！

### 5.2 离散化（显式欧拉）

$$x_{k+1} = x_k + dt \cdot f(x_k, u_k, ref_k)$$

代码实现 (`diff_drive_model.cpp::predict`):
```cpp
x_next(E_L) = e_l + dt * (v - v_path);
x_next(E_C) = e_c + dt * v * e_theta;
x_next(E_THETA) = e_theta + dt * (omega - kappa * v);
x_next(V) = v + dt * a;
```

### 5.3 线性化

状态方程线性化：$x_{k+1} = A_k x_k + B_k u_k + c_k$

**A 矩阵** ($\partial f / \partial x$)：
$$A = I + dt \cdot \begin{bmatrix}
0 & 0 & 0 & 1 \\
0 & 0 & v & e_\theta \\
0 & 0 & 0 & -\kappa \\
0 & 0 & 0 & 0
\end{bmatrix}$$

**B 矩阵** ($\partial f / \partial u$)：
$$B = dt \cdot \begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 1 \\
1 & 0
\end{bmatrix}$$

**仿射项**（保证线性化与名义轨迹一致）：
$$c = predict(x, u) - A \cdot x - B \cdot u$$

### 5.4 液体晃动模型（可选）

当 `Q_slosh > 0` 时启用：
- 晃动状态由 `SloshIntegration` 模块预测
- 横向加速度：$a_y = v \cdot \omega$（离心力）
- 线性化时添加对应 A/B 块

---

## 6. 代价函数

### 6.1 QP 目标形式

$$\min_z \frac{1}{2} z^T H z + g^T z$$

### 6.2 代价项分解

| 代价项 | 公式 | 参数 |
|-------|-----|------|
| 横向误差 | $Q_{ec} \cdot e_c^2$ 或 $Q_{contour} \cdot e_c^2$ | `Q_ec` / `Q_contour` |
| 纵向误差 | $Q_{el} \cdot e_l^2$ 或 $Q_{lag} \cdot e_l^2$ | `Q_el` / `Q_lag` |
| 航向误差 | $Q_{e\theta} \cdot e_\theta^2$ | `Q_etheta` |
| 速度跟踪 | $Q_v \cdot (v - v_{ref})^2$ | `Q_v` |
| 加速度 | $R_a \cdot a^2$ | `R_a` |
| 角速度 | $R_\omega \cdot \omega^2$ | `R_omega` |
| 加速度变化 | $R_{da} \cdot (a_k - a_{k-1})^2$ | `R_da` |
| 角速度变化 | $R_{d\omega} \cdot (\omega_k - \omega_{k-1})^2$ | `R_domega` |
| 角速度前馈 | $Q_{\omega ff} \cdot (\omega - v_{ref} \cdot \kappa)^2$ | `Q_omega_ff` |

### 6.3 终端代价放大

当 $k = N$ 时：
```cpp
Q(E_C, E_C) *= terminal_factor_ec;
Q(E_THETA, E_THETA) *= terminal_factor_etheta;
Q(V, V) *= terminal_factor_v;
g(V) *= terminal_factor_v;
```

### 6.4 H 矩阵结构

```
H = diag([Q_0, R_0, Q_1, R_1, ..., Q_{N-1}, R_{N-1}, Q_N])
    + 变化率耦合项
```

变化率耦合（以 $R_{d\omega}$ 为例）：
$$\sum_{k=0}^{N-1} R_{d\omega} \cdot (\omega_k - \omega_{k-1})^2$$

展开后在 H 中添加：
- 对角线：$+2R_{d\omega}$（每个 $\omega_k$ 和 $\omega_{k-1}$）
- 非对角线：$-2R_{d\omega}$（$\omega_k$ 与 $\omega_{k-1}$ 的交叉项）

---

## 7. 约束

### 7.1 约束矩阵结构

$$l \leq A z \leq u$$

约束行数：$n_c = n_x + N \cdot n_x + n_{bounds}$

### 7.2 初始条件约束

$$x_0 = \bar{x}_0 \quad (n_x \text{ 行等式约束})$$

### 7.3 动力学约束

$$x_{k+1} - A_k x_k - B_k u_k = c_k \quad (N \cdot n_x \text{ 行等式约束})$$

在约束矩阵中表示为：
```
[I, 0, -A_k, -B_k, 0, ...] * z = c_k
```

### 7.4 状态边界

$$v_{min} \leq v_k \leq v_{max}, \quad k = 0, \ldots, N$$

共 $N+1$ 行。

### 7.5 控制边界

$$\begin{aligned}
-a_{max} &\leq a_k \leq a_{max} \\
-\omega_{max} &\leq \omega_k \leq \omega_{max}
\end{aligned}, \quad k = 0, \ldots, N-1$$

共 $2N$ 行。

### 7.6 控制变化率约束（硬约束）

$$-\alpha_{max} \cdot dt \leq \omega_k - \omega_{k-1} \leq \alpha_{max} \cdot dt$$

- 当 $k=0$ 时使用 $u_{prev}$
- 可选：加速度变化率约束（`constrain_accel_rate`）

### 7.7 约束维度汇总

| 约束类型 | 行数 |
|---------|-----|
| 初始条件 | $n_x = 8$ |
| 动力学 | $N \cdot n_x = 320$ |
| 状态边界 | $N + 1 = 41$ |
| 控制边界 | $2N = 80$ |
| ω变化率 | $N = 40$ (可选) |
| a变化率 | $N = 40$ (可选) |
| **总计** | $n_c = 489$ (含ω变化率) |

---

## 8. 求解器与 Warm-start

### 8.1 OSQP 配置

```cpp
osqp_settings_->verbose = false;
osqp_settings_->warm_start = true;
osqp_settings_->polish = true;
osqp_settings_->eps_abs = 1e-4;
osqp_settings_->eps_rel = 1e-4;
osqp_settings_->max_iter = 4000;
```

### 8.2 求解流程

```cpp
bool MPCSolver::solve(const StateVector& x0, const std::vector<ReferencePoint>& refs) {
    // 1. 构建 QP
    buildQP(x0, refs);  // 生成 P_, q_, A_, l_, u_
    
    // 2. 更新 OSQP 工作空间
    updateOSQP();       // CSC 格式转换，warm_start 设置
    
    // 3. 热启动
    warmStart();        // z_prev_ 前移一拍
    
    // 4. 求解
    osqp_solve(osqp_work_);
    
    // 5. 提取解
    extractSolution(solution_);
    
    return (osqp_work_->info->status_val == OSQP_SOLVED);
}
```

### 8.3 热启动策略

```cpp
void MPCSolver::warmStart() {
    // 将上一次解前移一步：z_init[k] = z_prev[k+1]
    for (int k = 0; k < N - 1; ++k) {
        z_init.segment(k*(nx+nu), nx+nu) = z_prev_.segment((k+1)*(nx+nu), nx+nu);
    }
    // 最后一步复制
    z_init.segment((N-1)*(nx+nu), nx+nu) = z_prev_.segment((N-1)*(nx+nu), nx+nu);
    z_init.segment(N*(nx+nu), nx) = z_prev_.segment(N*(nx+nu), nx);
    
    osqp_warm_start_x(osqp_work_, z_init.data());
}
```

### 8.4 重置时机

- 收到新路径
- 弧长跳变超过 `s_jump_threshold`
- 求解失败

---

## 9. 关键参数对照表

### 9.1 MPC 参数 (`mpc/*`)

| 参数 | 仿真值 | 实物值 | 作用 |
|-----|--------|--------|-----|
| `N` | 40 | 40 | 预测步长 |
| `dt` | 0.05 | 0.05 | 时间步长 (s) |
| `Q_ec` / `Q_contour` | 60 | 30 | 横向误差权重 |
| `Q_etheta` | 12 | 12 | 航向误差权重 |
| `Q_v` | 15 | 15 | 速度跟踪权重 |
| `R_a` | 0.8 | 0.8 | 加速度权重 |
| `R_omega` | 0.5 | **3.0** | 角速度权重（实物增大）|
| `R_domega` | 1.0 | **5.0** | 角速度变化权重（实物增大）|

### 9.2 车辆参数 (`vehicle/*`)

| 参数 | 仿真值 | 实物值 | 作用 |
|-----|--------|--------|-----|
| `v_max` | 2.0 | **1.5** | 最大线速度 (m/s) |
| `omega_max` | 2.5 | **1.5** | 最大角速度 (rad/s) |
| `a_max` | 2.0 | 2.0 | 最大线加速度 (m/s²) |
| `alpha_max` | 7.0 | **4.0** | 最大角加速度 (rad/s²) |

### 9.3 路径处理参数 (`path_handler/*`)

| 参数 | 仿真值 | 实物值 | 作用 |
|-----|--------|--------|-----|
| `goal_tolerance` | 0.1 | **0.25** | 到达目标容差 (m) |
| `yaw_tolerance` | 0.1 | **0.15** | 航向容差 (rad) |
| `lookahead_distance` | 0.6 | 0.6 | 前视距离 (m) |
| `max_lat_accel` | 2.0 | 2.0 | 最大横向加速度 (m/s²) |
| `max_tan_decel` | 2.0 | 2.0 | 最大切向减速度 (m/s²) |
| `decel_safety_factor` | - | 0.8 | 减速保守系数（硬编码）|

---

## 10. 常见问题排查清单

### 10.1 路径跟踪偏差大

| 症状 | 可能原因 | 排查方法 | 解决方案 |
|-----|---------|---------|---------|
| 横向偏差大 | `Q_ec` 过小 | 打印 `e_c` | 增大 `Q_ec` 或 `Q_contour` |
| 切弯严重 | `lookahead_distance` 过大 | 减小 lookahead | 减小到 0.3~0.6m |
| 弯道超速 | `max_lat_accel` 过大 | 打印 `v_ref` 和 `kappa` | 减小到 1.5~2.0 |

### 10.2 终点不收敛

| 症状 | 可能原因 | 排查方法 | 解决方案 |
|-----|---------|---------|---------|
| 距终点0.3m停住 | `goal_tolerance` 过小 | 打印距离 | 增大到 0.25m |
| 终点速度过快 | 减速余量不足 | 打印 `v_ref` 和 `s_progress` | 增大 `decel_safety_factor` |
| 绕终点打转 | `yaw_tolerance` 过小 | 打印航向误差 | 增大到 0.15 rad |

### 10.3 控制不平滑

| 症状 | 可能原因 | 排查方法 | 解决方案 |
|-----|---------|---------|---------|
| ω 抖动 | `R_omega`/`R_domega` 过小 | 打印 `omega_cmd` | 增大到 3.0/5.0 |
| 急加急减 | `R_da` 过小 | 打印 `a_cmd` | 增大 `R_da` |
| ω 变化受限 | `alpha_max` 过小 | 打印 Δω 是否饱和 | 增大 `alpha_max` |

### 10.4 求解问题

| 症状 | 可能原因 | 排查方法 | 解决方案 |
|-----|---------|---------|---------|
| OSQP 失败 | 约束不可行 | 打印 `osqp_work_->info->status_val` | 检查初始状态是否越界 |
| 频繁重建 | 矩阵结构变化 | 观察 `Matrix structure changed` 日志 | 检查动态约束启用情况 |
| 求解变慢 | 热启动失效 | 打印 `solve_time_ms` | 检查 `z_prev_` 重置时机 |

### 10.5 TF / 坐标系问题

| 症状 | 可能原因 | 排查方法 | 解决方案 |
|-----|---------|---------|---------|
| 路径漂移 | TF 延迟 | 观察 `[PathHandler] TF error` | 检查 TF 发布频率 |
| 位置跳变 | 定位抖动 | 打印 `s_global_` 变化 | 增大 `s_jump_threshold` |
| 样条拟合失败 | 窗口过短 | 观察 `Window too small` | 增大 `window_forward` |

---

## 附录 A：与 MPCC 的差异

| 方面 | 当前实现 | MPCC |
|-----|---------|------|
| 进度 $s$ | 外部计算，不参与优化 | 作为状态/控制变量 |
| 路径参数化 | 固定参考轨迹 | 软约束 + 进度动力学 |
| contour/lag | 仅体现在权重 | 与 $s$ 绑定的真正误差 |
| 障碍约束 | 无 | 可行走廊约束 |

升级路径：
1. 引入 $s$ 作为控制变量
2. 添加进度动力学 $\dot{s} = v_{progress}$
3. 将 $e_c, e_l$ 定义为相对于 $s$ 的真正 contour/lag 误差
4. 添加道路走廊约束

---

## 附录 B：代码片段速查

### B.1 状态向量索引

```cpp
// types.h
struct StateIndex {
    static constexpr int E_L = 0, E_C = 1, E_THETA = 2, V = 3;
    static constexpr int ETA_X = 4, ETA_X_DOT = 5, ETA_Y = 6, ETA_Y_DOT = 7;
    static constexpr int TOTAL_DIM = 8;
};

struct ControlIndex {
    static constexpr int A = 0, OMEGA = 1;
    static constexpr int DIM = 2;
};
```

### B.2 决策变量索引

```cpp
// mpc_solver.cpp
const int nx = StateIndex::TOTAL_DIM;  // 8
const int nu = ControlIndex::DIM;       // 2
const int nz = N * (nx + nu) + nx;      // N*10 + 8

// x_k 的起始索引
int x_idx = k * (nx + nu);
// u_k 的起始索引
int u_idx = x_idx + nx;
```

### B.3 控制输出

```cpp
// local_planner_ros.cpp
double v_cmd = solution.x_predicted[1](StateIndex::V);
double omega_cmd = solution.u_first(ControlIndex::OMEGA);  // 直接控制！
```

---

*文档结束*
