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