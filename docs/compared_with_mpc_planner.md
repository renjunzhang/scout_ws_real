# scout_local_planner 改进计划（对标 mpc_planner）

> 基于 `/home/a/scout_ws/src/mpc_planner` 开源项目的分析，以下是 `scout_local_planner` 的详细改进计划。

## 改进总览

| 优先级 | 改进项 | 现状 | 目标 | 涉及文件 |
|--------|--------|------|------|----------|
| 🔴 P1 | 速度规划 v(s) | 固定 `v_ref = v_max * 0.8` | 前向/后向约束 + 曲率限速 | `path_handler.*` |
| 🔴 P1 | 时间参数化 | 固定 dt 采样 | 按时间采样参考点 | `path_handler.*`, `mpc_solver.*` |
| 🟡 P2 | 路径预处理 | 无预处理 | 等距重采样 + 平滑滤波 | `path_handler.*` |
| 🟡 P2 | 求解失败恢复 | 停车 | 重试 + 降级 + 安全制动 | `local_planner_ros.*` |
| 🟡 P2 | 线性化增强 | 固定名义点 | 自适应线性化 + 数值稳定 | `mpc_solver.*` |
| 🟢 P3 | 终端约束 | 无 | 零终端速度/位置约束 | `constraint_manager.*` |

---

## 改进 1：速度规划 v(s)（🔴 优先级最高）

### 1.1 原理说明

`mpc_planner` 使用 **v(s) 速度规划**，根据路径弧长 s 生成可行速度曲线：

```
1. 前向遍历：根据最大加速度计算 v_forward(s)
   v_forward[i+1] = sqrt(v[i]^2 + 2 * a_max * ds)

2. 后向遍历：根据最大减速度计算 v_backward(s)
   v_backward[i-1] = sqrt(v[i]^2 + 2 * a_max * ds)

3. 曲率限速：根据曲率计算 v_curvature(s)
   v_curvature[i] = sqrt(a_lat_max / |kappa[i]|)

4. 取最小值：v(s) = min(v_forward, v_backward, v_curvature, v_max)
```

### 1.2 修改文件（对齐当前实现）

**文件**：`scout_local_planner/include/scout_local_planner/path_handler.h`

```cpp
// 在 PathHandler 类中维护：
private:
  std::vector<double> s_samples_;    // 样条弧长采样
  std::vector<double> v_samples_;    // 速度曲线 v(s)

public:
  void updateSpeedProfile(double v_des);
  double getSpeedAtS(double s) const;
```

**文件**：`scout_local_planner/src/path_handler.cpp`

```cpp
// 说明：现实现以 local_spline_ 为基础，生成 s_samples_ / v_samples_
// （曲率限速 + 前向/后向加减速约束），并通过 getSpeedAtS(s) 线性插值。
```

### 1.3 配置参数（对齐当前实现）

**文件**：`config/mpc_params.yaml` 和 `config/mpc_params_sim.yaml`

```yaml
path_handler:
  time_parameterize: true
  speed_profile_ds: 0.05
  max_tan_accel: 0.8
  max_tan_decel: 0.8
  max_lat_accel: 1.0
  goal_speed: 0.0
```

---

## 改进 2：时间参数化采样（🔴 优先级高）

### 2.1 原理说明

当前 MPC 使用固定间距采样参考点，改为按**时间**采样：

```
给定速度曲线 v(s) 和采样周期 dt：
1. 初始化 t=0, s=s_current
2. 对于每个预测步 k：
   s_k = s + ∫[0, t_k] v(s) ds  （数值积分）
   t_k = k * dt
3. 在 s_k 处采样参考点 (x, y, theta, v, omega)
```

### 2.2 修改文件（对齐当前实现）

**文件**：`scout_local_planner/src/path_handler.cpp`

```cpp
// 说明：当前实现直接在 getReferencePoints() 内完成时间化推进：
// s_{k+1} = s_k + v(s_k) * dt，并在 s_k 处采样参考点。
// ReferencePoint 不包含 omega_ref，使用 kappa + v_ref 由动力学计算。

ReferencePoint PathHandler::interpolateAtArcLength(double s) const {
  // 二分查找 + 线性插值
  auto it = std::lower_bound(arc_lengths_.begin(), arc_lengths_.end(), s);
  if (it == arc_lengths_.begin()) return local_path_[0];
  if (it == arc_lengths_.end()) return local_path_.back();
  
  size_t idx = std::distance(arc_lengths_.begin(), it);
  double ratio = (s - arc_lengths_[idx-1]) / (arc_lengths_[idx] - arc_lengths_[idx-1]);
  
  ReferencePoint ref;
  ref.x = local_path_[idx-1].x + ratio * (local_path_[idx].x - local_path_[idx-1].x);
  ref.y = local_path_[idx-1].y + ratio * (local_path_[idx].y - local_path_[idx-1].y);
  ref.theta = interpolateAngle(local_path_[idx-1].theta, local_path_[idx].theta, ratio);
  ref.kappa = local_path_[idx-1].kappa + ratio * (local_path_[idx].kappa - local_path_[idx-1].kappa);
  ref.v_ref = getVelocityAtArcLength(s);
  ref.omega_ref = ref.v_ref * ref.kappa;
  
  return ref;
}
```

---

## 改进 3：路径预处理（🟡 优先级中）

### 3.1 等距重采样

```cpp
void PathHandler::resamplePath(double spacing) {
  if (global_path_.poses.size() < 2) return;
  
  std::vector<geometry_msgs::PoseStamped> resampled;
  resampled.push_back(global_path_.poses[0]);
  
  double accumulated = 0.0;
  for (size_t i = 1; i < global_path_.poses.size(); ++i) {
    double dx = global_path_.poses[i].pose.position.x - global_path_.poses[i-1].pose.position.x;
    double dy = global_path_.poses[i].pose.position.y - global_path_.poses[i-1].pose.position.y;
    double dist = std::hypot(dx, dy);
    accumulated += dist;
    
    if (accumulated >= spacing) {
      resampled.push_back(global_path_.poses[i]);
      accumulated = 0.0;
    }
  }
  
  // 确保终点被包含
  if (resampled.back() != global_path_.poses.back()) {
    resampled.push_back(global_path_.poses.back());
  }
  
  global_path_.poses = resampled;
  ROS_DEBUG("Path resampled: %zu -> %zu points", 
            global_path_.poses.size(), resampled.size());
}
```

### 3.2 滑动平均滤波

```cpp
void PathHandler::smoothPathMovingAverage(int window_size) {
  if (local_path_.size() < window_size) return;
  
  std::vector<ReferencePoint> smoothed = local_path_;
  int half = window_size / 2;
  
  for (size_t i = half; i < local_path_.size() - half; ++i) {
    double sum_x = 0, sum_y = 0;
    for (int j = -half; j <= half; ++j) {
      sum_x += local_path_[i + j].x;
      sum_y += local_path_[i + j].y;
    }
    smoothed[i].x = sum_x / window_size;
    smoothed[i].y = sum_y / window_size;
  }
  
  local_path_ = smoothed;
  
  // 重新计算航向和曲率
  computeHeadingsAndCurvatures();
}
```

---

## 改进 4：求解失败恢复（🟡 优先级中）

### 4.1 安全降级策略

**文件**：`scout_local_planner/src/local_planner_ros.cpp`

```cpp
void LocalPlannerROS::controlLoop(const ros::TimerEvent& event) {
  // ... 获取状态和参考点 ...
  
  // MPC 求解
  bool solve_success = mpc_solver_->solve(x0, ref_points, u_opt);
  
  if (solve_success) {
    // 正常输出
    consecutive_failures_ = 0;
    last_valid_cmd_ = u_opt[0];
    publishCmdVel(u_opt[0]);
  } else {
    consecutive_failures_++;
    ROS_WARN("MPC solve failed (%d consecutive)", consecutive_failures_);
    
    // 降级策略
    if (consecutive_failures_ <= 3) {
      // 策略 1：使用上次有效指令（衰减）
      geometry_msgs::Twist cmd;
      cmd.linear.x = last_valid_cmd_.linear.x * 0.8;
      cmd.angular.z = last_valid_cmd_.angular.z * 0.5;
      publishCmdVel(cmd);
      ROS_INFO("Using degraded command: v=%.2f, omega=%.2f", 
               cmd.linear.x, cmd.angular.z);
    } else if (consecutive_failures_ <= 5) {
      // 策略 2：缓慢制动
      geometry_msgs::Twist cmd;
      cmd.linear.x = std::max(0.0, last_valid_cmd_.linear.x - 0.1);
      cmd.angular.z = 0.0;
      publishCmdVel(cmd);
      ROS_WARN("Slow braking: v=%.2f", cmd.linear.x);
    } else {
      // 策略 3：完全停止
      geometry_msgs::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      publishCmdVel(cmd);
      ROS_ERROR("Emergency stop due to %d consecutive failures", consecutive_failures_);
    }
  }
}
```

### 4.2 配置参数

```yaml
mpc:
  # 求解失败恢复
  max_consecutive_failures: 5       # 最大连续失败次数
  degraded_velocity_ratio: 0.8      # 降级模式速度衰减比例
  brake_deceleration: 0.5           # 制动减速度 (m/s^2)
```

---

## 改进 5：线性化增强（🟡 优先级中）

### 5.1 自适应线性化点

```cpp
void MpcSolver::updateLinearization(const State& x0, 
                                    const std::vector<ReferencePoint>& refs) {
  // 名义轨迹：不再固定为 x0，而是随预测步推进
  nominal_trajectory_.resize(horizon_ + 1);
  nominal_trajectory_[0] = x0;
  
  for (int k = 0; k < horizon_; ++k) {
    // 使用上一次优化结果作为名义输入
    Eigen::Vector2d u_nom = (k < last_u_opt_.size()) ? 
                            last_u_opt_[k] : Eigen::Vector2d::Zero();
    
    // 前向积分
    nominal_trajectory_[k+1] = model_->predict(nominal_trajectory_[k], u_nom, dt_);
    
    // 在名义点处线性化
    model_->linearize(nominal_trajectory_[k], u_nom, A_[k], B_[k]);
  }
}
```

### 5.2 数值稳定性增强

```cpp
void MpcSolver::ensureNumericalStability() {
  // 1. 约束 Hessian 对角线最小值
  for (int i = 0; i < H_.rows(); ++i) {
    if (H_(i, i) < 1e-6) {
      H_(i, i) = 1e-6;
    }
  }
  
  // 2. 添加正则化项
  double regularization = 1e-8;
  H_ += regularization * Eigen::MatrixXd::Identity(H_.rows(), H_.cols());
  
  // 3. 限制梯度范数
  double grad_norm = g_.norm();
  if (grad_norm > 1e6) {
    g_ *= 1e6 / grad_norm;
    ROS_WARN("Gradient clipped: norm=%.2e", grad_norm);
  }
}
```

---

## 改进 6：终端约束（🟢 优先级低）

### 6.1 终端速度约束

```cpp
void ConstraintManager::addTerminalConstraints() {
  int terminal_idx = horizon_;
  
  // 终端速度约束：v_N = 0, omega_N = 0
  if (enable_terminal_velocity_constraint_) {
    // v_N = 0
    A_eq_.row(eq_count_) = Eigen::VectorXd::Zero(n_vars_);
    A_eq_(eq_count_, getStateIndex(terminal_idx, 3)) = 1.0;  // v
    b_eq_(eq_count_) = 0.0;
    eq_count_++;
    
    // omega_N = 0
    A_eq_.row(eq_count_) = Eigen::VectorXd::Zero(n_vars_);
    A_eq_(eq_count_, getStateIndex(terminal_idx, 4)) = 1.0;  // omega
    b_eq_(eq_count_) = 0.0;
    eq_count_++;
  }
}
```

---

## 实施顺序建议

```
第 1 周：速度规划 v(s) + 时间参数化
  ├── 修改 path_handler.h/cpp
  ├── 添加配置参数
  ├── 单元测试：速度曲线生成
  └── 集成测试：仿真验证

第 2 周：路径预处理 + 求解失败恢复
  ├── 添加等距重采样
  ├── 添加滑动平均滤波
  ├── 实现降级策略
  └── 集成测试：故障注入验证

第 3 周：线性化增强 + 终端约束
  ├── 自适应线性化点
  ├── 数值稳定性增强
  ├── 终端约束（可选）
  └── 完整流程测试
```

---

## 测试验证

### 速度规划验证

```bash
# 1. 查看速度曲线话题（需添加发布）
rostopic echo /scout/velocity_profile

# 2. RViz 可视化
# 添加 Marker 显示速度曲线
```

### 求解失败恢复验证

```bash
# 1. 故障注入：临时让 OSQP 返回失败
# 修改 mpc_solver.cpp，强制返回 false

# 2. 观察降级行为
rostopic echo /cmd_vel
rostopic echo /mpc_status
```

### 性能对比

| 指标 | 改进前 | 改进后（预期） |
|------|--------|----------------|
| 转弯跟踪误差 | ~0.3m | ~0.1m |
| 终点停止精度 | 不稳定 | 精确停止 |
| 求解失败处理 | 急停 | 平滑降级 |
| 高曲率通过性 | 容易丢失 | 稳定跟踪 |


- 整体方案是靠谱的，属于在现有架构上做增量增强，不会推翻你现在的独立节点结构。
  - 它主要借鉴了 mpc_planner 的“速度曲线 + 轨迹跟随”思想，但保持你当前的 OSQP + Frenet 框架。
  - 风险点在于：路径质量和参数约束依然是主导因素，B‑spline 还没做，障碍约束也没做，所以“转弯不顺”仍可能出现。

  - 这份方案主要做了/规划了什么（按实际完成的修改）：

  - 已完成：时间化速度规划 v(s)（曲率限速 + 前后向加减速）
  - 已完成：contour + lag 误差结构（用 Q_contour/Q_lag 控制）
  - 已完成：求解不可行时安全退化（制动降速）
  - 已完成：局部重采样 + 窗口样条
  - 计划中：B‑spline 安全版（局部窗口）

  ———

  ## 你当前 scout_local_planner 的真实状态（重点回答你问的几个点）

  1) 输入路径是什么？

  - 订阅话题：/scout/global_path（nav_msgs/Path）
  - 来源：move_base 全局规划
  - 对应 launch：src/scout_apps/control/scout_local_planner/launch/test_mpc_sim.launch

  2) 路径是平滑后的吗？

  - 话题本身不是平滑的（仍是原始 global_path）
  - 内部会平滑：先重采样，再局部三次样条拟合
  - /scout/global_path_smooth 只是可视化输出，不参与控制

  3) 在哪个坐标系下？

  - /scout/global_path 原始是 map
  - PathHandler 会 用 TF 转到 base_link
  - MPC 参考点是 base_link 坐标系

  4) 是时间参数化轨迹吗？

  - 是“时间化速度曲线 + 路径采样”
  - 不是带时间戳的轨迹消息
  - 当前仿真已启用 time_parameterize: true

  5) 是规控一体还是只有控制器？

  - 只有控制器（局部跟踪）
  - 全局规划由 move_base 提供，MPC不做规划

  6) 有避障功能吗？

  - 没有
  - 需要全局规划 / 其他模块处理障碍
  - MPC 只跟随路径，不考虑障碍约束

  7) 适用于什么小车模型？

  - 差速/自转型（unicycle）
  - Scout Mini（滑移转向）在低速可近似使用
  - 不是 Ackermann 模型

  8) 使用的优化求解器是？

  - OSQP（osqp_vendor）

  ———

  如果你想继续改进“转弯不顺”，下一步建议走你计划里的 B‑spline 安全版（局部窗口），同时再给我当前参数值（max_lat_accel /
  max_tan_accel / omega_max / window_forward），我可以更精确地调。


  ---

  ## 优化计划：让 scout_local_planner 更接近 mpc_planner

  0) 对齐现状与测量口径（不改逻辑）
  - 固定评估场景：同一条 global_path、同一速度上限、同一转弯半径
  - 记录：/scout/global_path、/local_path、/cmd_vel、/odom（方便对比改动前后）

  1) 参考路径“稳定化”（减少抖动的关键）
  - 在 map 坐标系下构建全局样条（一次性），保留连续弧长 s
  - 运行时只做“投影 + 局部截取”，避免每周期重拟合导致曲率/切线跳变
  - 仍输出 base_link 参考点，但 s 不再“抖动”

  2) 速度参考对齐 mpc_planner 思路
  - 把 v(s) 作为“路径速度参考”，并随 s 推进（已有 time_parameterize 的基础上改成更稳定的 s 推进）
  - 在代价中对 (v - v_ref(s)) 建惩罚（当前已有 Q_v，可继续沿用）
  - 目标是让“参考速度”与“路径推进”一致，减少转弯时的拖拽感

  3) 控制通道对齐（提升转向响应）
  - 将控制量从 (a, alpha) 改为 (a, omega) 或 (v, omega)
  - 用简单的“角速度变化率限幅”代替二阶角加速度模型
  - 理由：mpc_planner 直接控 w，转弯响应更直接

  4) 预测视野对齐（让转弯提前量更足）
  - 将预测时域提高到 2~3 秒（例如 N=30, dt=0.1 或 N=40, dt=0.05）
  - 配合调整 Q_ec/Q_etheta/Q_v 与 R_*，避免“慢吞吞”

  5) 目标航向与终端约束对齐
  - 终端 cost 增加“切线方向误差”权重（更像 mpc_planner 的 terminal_angle）
  - 保持当前用路径末端切线方向作为目标航向

  6) 平滑策略升级（安全版）
  - 仅在 base_link 局部窗口做 B‑spline
  - 作为 MPC 内部参考点平滑，不改变全局路径，避免穿墙

  7) 失败退化策略对齐
  - 不可行时使用“可控减速 + 角速度缩放”而非急停
  - 保持与仿真一致的 safety 参数（避免实物突变）

  8) 验证与回滚
  - 每步只改一个方向（模型/参考/权重/视野）
  - 记录：轨迹误差、最大角速度、到达时间
  - 若效果变差，回滚该步参数

---

## 优化计划：修订版 v2

> **原则**：增量式改进，每步可验证、可回滚；优先调参数，再改算法，最后改架构。

### 原计划评估

| 步骤 | 评价 | 风险点 |
|------|------|--------|
| 0) 基准测量 | ✅ 很好 | 无 |
| 1) 全局样条 | ⚠️ 过度工程 | map 坐标系样条在转弯时可能导致累积误差 |
| 2) v(s) 速度参考 | ✅ 核心改进 | 已部分实现，需稳定化 |
| 3) 控制通道 (a,α)→(a,ω) | ⚠️ 风险较大 | 需要重构 QP 矩阵，可能引入新 bug |
| 4) 预测视野 | ✅ 简单有效 | 参数调整即可 |
| 5) 终端约束 | ✅ 合理 | 增量式改动 |
| 6) B-spline 平滑 | ✅ 已实现 | 需验证参数 |
| 7) 失败退化 | ✅ 已实现 | 需验证 |
| 8) 验证回滚 | ✅ 必要 | 无 |

---

### 核心差异分析（scout_local_planner vs mpc_planner）

| 特性 | scout_local_planner | mpc_planner |
|------|---------------------|-------------|
| 坐标系 | base_link（每周期重变换） | map（全局累积弧长） |
| 路径表示 | 局部三次样条（每周期重拟合） | 全局样条 + 当前段索引 |
| 速度规划 | v(s) 时间化（已实现） | 样条插值速度曲线 |
| 控制量 | (a, α) 二阶模型 | (vx, vy) 或 (v, ω) 直接控制 |
| 模块化 | 单体式 | 高度模块化（ControllerModule） |

---

### 第 0 步：基准测量与对比框架（不改代码）

**目标**：建立可复现的评估基准

- [ ] 固定测试场景：
  - 直线段（10m）+ 90° 弯道（R=2m）+ S 弯（R=1.5m）
  - 速度上限 v_max = 0.5 m/s
- [ ] 录制 bag 文件（改动前）：
  ```bash
  rosbag record /scout/global_path /scout/local_path /scout/cmd_vel /odom /tf -O baseline.bag
  ```
- [ ] 定义评估指标：
  - 横向误差 RMS（e_c）
  - 纵向误差 RMS（e_l）
  - 航向误差 RMS（e_θ）
  - 最大角速度（ω_max）
  - 到达时间
  - 求解成功率

---

### 第 1 步：稳定弧长跟踪（🔴 关键改进）

**问题**：当前 `current_s_` 在每周期重置，导致速度规划抖动

**解决方案**：维护全局连续弧长 `s_global`

```cpp
// path_handler.h 新增
private:
  double s_global_ = 0.0;           // 全局累积弧长
  double last_projection_s_ = 0.0;  // 上次投影位置
  bool s_initialized_ = false;

// path_handler.cpp 修改 getReferencePoints()
// 1. 投影当前机器人位置到路径，得到 s_proj
// 2. 计算 ds = s_proj - last_projection_s_
// 3. 若 |ds| < 阈值（如 0.5m），则 s_global_ += ds
// 4. 否则重置（路径跳变或重新规划）
```

**验证方法**：
```bash
rostopic echo /scout/mpc_debug | grep s_global  # 观察弧长是否平滑递增
```

**预期收益**：速度曲线 v(s) 不再因 s 抖动而跳变

---

### 第 2 步：预测视野调参（⚡ 立即生效）

**问题**：当前 N=20, dt=0.05 → 1.0 秒视野，转弯提前量不足

**调整方案**（仅改 yaml）：

```yaml
# config/mpc_params_sim.yaml
mpc:
  N: 30           # 20 → 30
  dt: 0.1         # 0.05 → 0.1，总视野 3.0 秒

# 配套权重调整（避免过于激进）
cost:
  Q_ec: 50.0      # 横向误差权重（可能需要降低，避免过度转向）
  Q_etheta: 20.0  # 航向误差权重
  R_a: 10.0       # 加速度平滑
  R_alpha: 15.0   # 角加速度平滑（关键：抑制角速度抖动）
```

**验证方法**：直接测试 S 弯场景，对比转向提前量

---

### 第 3 步：曲率前馈优化（🟡 中等优先级）

**问题**：当前 MPC 仅用误差反馈，缺少曲率前馈

**解决方案**：在代价函数中加入参考角速度项

```cpp
// cost_function.cpp 修改
// 对于每个预测步 k：
double omega_ref = ref.v_ref * ref.kappa;  // 前馈角速度
// 在代价中加入：Q_omega_ff * (omega - omega_ref)^2
```

**配置参数**：
```yaml
cost:
  Q_omega_ff: 5.0     # 角速度前馈权重（新增）
  enable_omega_ff: true
```

**预期收益**：转弯时角速度提前响应，减少滞后

---

### 第 4 步：控制通道简化（🟡 可选，风险较大）

> ⚠️ **注意**：这步改动较大，建议在前 3 步稳定后再做

**当前**：控制量 (a, α)，状态 [x, y, θ, v, ω]

**目标**：控制量 (a, ω)，状态 [x, y, θ, v]

**好处**：
- 直接控制角速度，转向响应更快
- QP 规模减小（少一个状态维度）
- 更接近 mpc_planner 的 unicycle 模型

**实现要点**：
1. 修改 `DiffDriveModel::predict()` 和 `linearize()`
2. 修改约束管理器（ω 约束代替 α 约束）
3. 添加 ω 变化率约束（软约束）：|ω[k] - ω[k-1]| ≤ Δω_max

**回滚条件**：若转弯抖动加剧或求解失败率上升，回滚

---

### 第 5 步：终端代价增强（🟢 低优先级）

**问题**：当前终端代价仅考虑位置误差

**解决方案**：增加终端航向和终端速度代价

```cpp
// 终端步 k = N 时：
double e_theta_terminal = normalizeAngle(x[N].theta - ref[N].theta_path);
double e_v_terminal = x[N].v - 0.0;  // 终点减速

cost += Q_theta_terminal * e_theta_terminal^2;
cost += Q_v_terminal * e_v_terminal^2;
```

**配置参数**：
```yaml
cost:
  Q_theta_terminal: 30.0   # 终端航向权重
  Q_v_terminal: 20.0       # 终端速度权重（鼓励减速）
```

---

### 第 6 步：失败恢复验证（✅ 已实现，需验证）

当前 `local_planner_ros.cpp` 已有降级策略，建议验证：

```bash
# 故障注入测试
rosparam set /scout_local_planner/force_solver_fail true
rostopic echo /scout/cmd_vel  # 观察降级行为
```

---

### 第 7 步：与 mpc_planner 对齐的长期方向

以下是更大的架构改进，建议作为后续独立项目：

| 特性 | 当前 scout_local_planner | mpc_planner | 改进建议 |
|------|--------------------------|-------------|----------|
| 障碍物处理 | 无 | 动态障碍预测 + 约束 | 添加 costmap 障碍约束 |
| 路径表示 | 局部样条 | 全局样条 + 段索引 | 考虑持久化全局样条 |
| 模块化 | 单体 | ControllerModule | 可选：重构为模块化架构 |
| 求解器 | OSQP | FORCES Pro / acados | 可选：替换为 acados（开源） |

---

### 实施时间线

```
第 1 周（立即）：
├── 步骤 0：录制基准 bag
├── 步骤 2：调整 N/dt 和权重
└── 验证：S 弯场景改善程度

第 2 周：
├── 步骤 1：稳定弧长跟踪
├── 步骤 3：曲率前馈
└── 验证：速度曲线平滑度

第 3 周（可选）：
├── 步骤 4：控制通道简化
├── 步骤 5：终端代价
└── 全流程回归测试

持续：
├── 步骤 6：失败恢复验证
└── 步骤 7：长期路线图评估
```

---

### 参数调优速查表

| 场景 | 问题表现 | 调整参数 | 方向 |
|------|----------|----------|------|
| 转弯滞后 | 进入弯道时偏离外侧 | `N` 或 `Q_ec` | ↑ |
| 转弯抖动 | 角速度频繁正负切换 | `R_alpha` | ↑ |
| 直线摆动 | 直线路径上左右晃动 | `Q_etheta` 或 `Q_omega_ff` | ↑ |
| 终点冲过 | 到目标后还在走 | `Q_v_terminal` 或 `goal_speed` | ↑ / 0 |
| 求解慢 | solve_time > 20ms | `N` | ↓ |
| 求解失败 | OSQP 返回非 SOLVED | 检查约束边界、添加正则化 | - |


