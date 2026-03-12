# scout_ws 话题清单

> 仅记录当前工作区配置与文档中明确涉及的话题，按模块分类。
> 若节点放在 `/scout` 命名空间下，下面的相对话题会自动变为 `/scout/...`。
> 部分话题为“配置/规划”用途，实际是否存在取决于对应节点是否已启动。

---

## 0) 当前系统已观测话题

### 实车环境完整话题列表（2026-03-08，MBF + MPC + slosh）

#### 核心导航与控制

| 话题 | 说明 |
|------|------|
| `/cmd_vel` | 最终下发到底盘的速度指令 |
| `/local_path` | MPC 局部轨迹可视化 |
| `/mpc/solve_ms` | MPC 单次求解耗时 |
| `/mpc/status_val` | MPC 求解状态（1=成功，0=失败） |
| `/mpc_status` | MPC 状态机状态 |
| `/odom` | 里程计 |
| `/scout/current_goal` | 当前 goal 回显 |
| `/scout/global_path` | 全局路径输入 |
| `/scout/global_path_smooth` | 平滑/重采样后的路径可视化 |
| `/scout/goal` | 导航目标输入 |
| `/scout/move_base_cmd_vel` | MBF/move_base 旁路速度输出（不直接控车） |

#### 液体晃动与实验调试

| 话题 | 说明 |
|------|------|
| `/slosh/alpha_est` | 角加速度估计 |
| `/slosh/ax_est` | 纵向加速度估计 |
| `/slosh/ay_est` | 横向加速度估计 |
| `/slosh/constraint_active` | 预测峰值是否越过阈值 |
| `/slosh/episode_id` | 当前实验 episode 编号 |
| `/slosh/height` | 实际液面高度估计 |
| `/slosh/height_pred_max` | 预测域内最大液面高度 |
| `/slosh/speed_governor_active` | 速度治理是否介入 |
| `/slosh/state` | slosh 状态 `[eta_x, eta_x_dot, eta_y, eta_y_dot]` |
| `/slosh/v_des_eff` | governor 生效后的参考速度 |

#### MBF 与 costmap

| 话题 | 说明 |
|------|------|
| `/scout/mbf_costmap_nav/GlobalPlanner/plan` | MBF 全局规划插件原始输出路径 |
| `/scout/mbf_costmap_nav/GlobalPlanner/potential` | GlobalPlanner 势场 |
| `/scout/mbf_costmap_nav/current_goal` | MBF 当前目标 |
| `/scout/mbf_costmap_nav/get_path/*` | `GetPath` action 相关话题 |
| `/scout/mbf_costmap_nav/exe_path/*` | `ExePath` action 相关话题（当前主要作内部话题保留） |
| `/scout/mbf_costmap_nav/move_base/*` | `move_base` action 兼容接口 |
| `/scout/mbf_costmap_nav/recovery/*` | recovery action 兼容接口 |
| `/scout/mbf_costmap_nav/global_costmap/costmap` | 全局代价地图 |
| `/scout/mbf_costmap_nav/global_costmap/costmap_updates` | 全局代价地图增量更新 |
| `/scout/mbf_costmap_nav/global_costmap/footprint` | 全局代价地图足迹 |
| `/scout/mbf_costmap_nav/local_costmap/costmap` | 局部代价地图 |
| `/scout/mbf_costmap_nav/local_costmap/costmap_updates` | 局部代价地图增量更新 |
| `/scout/mbf_costmap_nav/local_costmap/footprint` | 局部代价地图足迹 |
| `/scout/mbf_costmap_nav/*/parameter_descriptions` | MBF/GlobalPlanner/costmap 动态参数描述 |
| `/scout/mbf_costmap_nav/*/parameter_updates` | MBF/GlobalPlanner/costmap 动态参数更新 |

#### 传感器与定位/建图

| 话题 | 说明 |
|------|------|
| `/clicked_point` | RViz 点击点 |
| `/extended_laser_scan` | 扩展激光扫描 |
| `/initialpose` | 初始位姿输入 |
| `/landmark_poses_list` | Cartographer 地标可视化 |
| `/map` | 地图 |
| `/map_updates` | 地图增量更新 |
| `/output_paths` | 定位/建图输出路径集合 |
| `/raw_data` | 原始雷达数据 |
| `/scan_front` | 前向激光扫描 |
| `/scan_front_filtered` | 过滤后的前向激光扫描 |
| `/scan_matched_points2` | 匹配点云 |
| `/submap_list` | 子图列表 |
| `/trajectory_node_list` | 轨迹节点列表 |

#### 底盘/BMS/系统

| 话题 | 说明 |
|------|------|
| `/BMS_status` | 电池状态 |
| `/constraint_list` | Cartographer 约束列表可视化 |
| `/diagnostics` | 诊断信息 |
| `/rosout` | ROS 日志 |
| `/rosout_agg` | ROS 日志聚合 |
| `/rs_status` | 底盘运行状态 |
| `/scout_light_control` | 灯光控制 |
| `/scout_status` | 底盘状态 |
| `/sick_safetyscanners_front/parameter_descriptions` | 雷达动态参数描述 |
| `/sick_safetyscanners_front/parameter_updates` | 雷达动态参数更新 |
| `/tf` | 动态坐标变换 |
| `/tf_static` | 静态坐标变换 |

### 仿真环境完整话题列表（2026-02-01）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz 点击点 |
| `/clock` | `rosgraph_msgs/Clock` | 仿真时钟 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制输出（scout_local_planner → 底盘） |
| `/constraint_list` | `visualization_msgs/MarkerArray` | Cartographer 约束可视化 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL 初始位姿 |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 |
| `/landmark_poses_list` | `visualization_msgs/MarkerArray` | Cartographer 地标可视化 |
| `/local_path` | `nav_msgs/Path` | MPC 预测轨迹可视化 |
| `/map` | `nav_msgs/OccupancyGrid` | 静态地图 |
| `/map_updates` | `map_msgs/OccupancyGridUpdate` | 地图增量更新 |
| `/mpc/solve_ms` | `std_msgs/Float32` | MPC 单次求解耗时（ms） |
| `/mpc/status_val` | `std_msgs/Int32` | MPC 求解结果标志（1=成功，0=失败） |
| `/mpc_status` | `std_msgs/String` | MPC 求解状态 |
| `/odom` | `nav_msgs/Odometry` | 里程计 |
| `/scan` | `sensor_msgs/LaserScan` | 激光扫描（主） |
| `/scan_front` | `sensor_msgs/LaserScan` | 前置激光扫描 |
| `/scan_matched_points2` | `sensor_msgs/PointCloud2` | Cartographer 匹配点云 |
| `/slosh/alpha_est` | `std_msgs/Float32` | 角加速度估计（EMA 后） |
| `/slosh/ax_est` | `std_msgs/Float32` | 纵向加速度估计（EMA 后） |
| `/slosh/ay_est` | `std_msgs/Float32` | 横向加速度估计（EMA 后，当前近似 `v*omega`） |
| `/slosh/height` | `std_msgs/Float32` | 液面晃动高度估计 |
| `/slosh/state` | `std_msgs/Float32MultiArray` | 液体晃动状态 `[eta_x, eta_x_dot, eta_y, eta_y_dot]` |
| `/scout/current_goal` | `geometry_msgs/PoseStamped` | MBF 当前导航目标回显 |
| `/scout/global_path` | `nav_msgs/Path` | 全局路径（move_base → local_planner） |
| `/scout/global_path_smooth` | `nav_msgs/Path` | **局部平滑路径**（local_planner 可视化输出） |
| `/scout/goal` | `geometry_msgs/PoseStamped` | 导航目标点 |
| `/scout/move_base_cmd_vel` | `geometry_msgs/Twist` | move_base 原始速度输出（已重映射，不使用） |
| `/scout/odom` | `nav_msgs/Odometry` | 命名空间内里程计 |
| `/submap_list` | `cartographer_ros_msgs/SubmapList` | Cartographer 子图列表 |
| `/tf` / `/tf_static` | `tf2_msgs/TFMessage` | 坐标变换 |
| `/trajectory_node_list` | `visualization_msgs/MarkerArray` | Cartographer 轨迹节点 |

### move_base 内部话题（仿真）

| 话题 | 说明 |
|------|------|
| `/scout/move_base/DWAPlannerROS/*` | DWA 局部规划器相关（cost_cloud, global_plan, local_plan） |
| `/scout/move_base/GlobalPlanner/potential` | 全局规划势场 |
| `/scout/move_base/cancel` | 取消导航目标 |
| `/scout/move_base/current_goal` | 当前目标 |
| `/scout/move_base/feedback` / `result` / `status` | ActionLib 反馈 |
| `/scout/move_base/goal` | ActionLib 目标 |
| `/scout/move_base/global_costmap/costmap` | 全局代价地图 |
| `/scout/move_base/local_costmap/costmap` | 局部代价地图 |
| `/scout/move_base/recovery_status` | 恢复状态 |

### Gazebo 仿真话题

| 话题 | 说明 |
|------|------|
| `/gazebo/link_states` / `model_states` | 仿真状态 |
| `/gazebo/set_link_state` / `set_model_state` | 设置状态服务 |
| `/gazebo/parameter_*` | 参数服务 |
| `/gazebo/performance_metrics` | 性能指标 |
| `/gazebo_ros_control/pid_gains/*/...` | PID 参数（四轮） |

---

## 1) 全局规划（move_base / scout_global_planner）

| 话题 | 类型 | 发布方 | 订阅方 | 说明 |
|------|------|--------|--------|------|
| `/scout/goal` | `geometry_msgs/PoseStamped` | RViz / 上层任务 | move_base | 目标点输入（需在 RViz 将 2D Nav Goal 话题改为此） |
| `/scout/global_path` | `nav_msgs/Path` | move_base 全局规划器 | scout_local_planner | 全局路径输出（由 `~GlobalPlanner/plan` / `~NavfnROS/plan` 重映射） |
| `/scout/move_base_cmd_vel` | `geometry_msgs/Twist` | move_base | 无（仅调试） | move_base 输出速度，已重映射避免干扰 `/cmd_vel` |

> 说明：move_base 插件原始私有话题 `~GlobalPlanner/plan` / `~NavfnROS/plan` 已在 launch 中 remap 到 `/scout/global_path`。

### 当前 MBF 模式（`mbf_global*.launch`）

| 话题 | 类型 | 发布方 | 订阅方 | 说明 |
|------|------|--------|--------|------|
| `/scout/goal` | `geometry_msgs/PoseStamped` | RViz / 上层任务 | `mbf_path_publisher` | 目标点输入 |
| `/scout/global_path` | `nav_msgs/Path` | `mbf_path_publisher_node` | `scout_local_planner` | MBF `GetPath` 结果转发后的统一全局路径 |
| `/scout/current_goal` | `geometry_msgs/PoseStamped` | `mbf_costmap_nav` | 调试/可视化 | MBF 当前处理中的目标 |
| `/scout/move_base_cmd_vel` | `geometry_msgs/Twist` | `mbf_costmap_nav` | 无（旁路） | MBF 的 `cmd_vel` 被隔离输出，不参与底盘控制 |
| `/scout/mbf_costmap_nav/GlobalPlanner/plan` | `nav_msgs/Path` | MBF 全局规划插件 | 调试/可视化 | 插件内部原始路径输出；下游实际跟踪仍以 `/scout/global_path` 为准 |
| `/scout/mbf_costmap_nav/get_path/*` | `actionlib` 相关话题 | `mbf_costmap_nav` / `mbf_path_publisher` | 调试 | `GetPath` action 的 goal/feedback/result/status |
| `/scout/mbf_costmap_nav/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | `mbf_costmap_nav` | 全局规划插件 / RViz | MBF 全局代价地图 |
| `/scout/mbf_costmap_nav/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | `mbf_costmap_nav` | 调试/插件 | MBF 局部代价地图（当前不直接驱动 MPC） |

> 说明：当前 MBF launch 已禁用恢复行为和 controller，主要使用 `mbf_costmap_nav/get_path` 生成全局路径，再由 `mbf_path_publisher_node` 转发到 `/scout/global_path`。

---

## 2) 局部规划（scout_local_planner）

| 话题 | 类型 | 发布方 | 订阅方 | 说明 |
|------|------|--------|--------|------|
| `/scout/global_path` | `nav_msgs/Path` | move_base | scout_local_planner | 全局路径输入 |
| `/odom` | `nav_msgs/Odometry` | 底盘驱动 | scout_local_planner | 里程计输入 |
| `/cmd_vel` | `geometry_msgs/Twist` | scout_local_planner | 底盘驱动 | 速度控制输出 |
| `/local_path` | `nav_msgs/Path` | scout_local_planner | RViz（可选） | 局部轨迹可视化 |
| `/scout/global_path_smooth` | `nav_msgs/Path` | scout_local_planner | RViz（可选） | 局部平滑/重采样后的路径可视化 |
| `/mpc_status` | `std_msgs/String` | scout_local_planner | 监控（可选） | MPC 状态机状态 |
| `/mpc/solve_ms` | `std_msgs/Float32` | scout_local_planner | 监控（可选） | 单次求解耗时 |
| `/mpc/status_val` | `std_msgs/Int32` | scout_local_planner | 监控（可选） | 求解结果标志（1=成功，0=失败） |
| `/slosh/state` | `std_msgs/Float32MultiArray` | scout_local_planner | 监控（可选） | 液体晃动状态 `[eta_x, eta_x_dot, eta_y, eta_y_dot]` |
| `/slosh/height` | `std_msgs/Float32` | scout_local_planner | 监控（可选） | 晃动高度输出 |
| `/slosh/ax_est` | `std_msgs/Float32` | scout_local_planner | 监控（可选） | 纵向加速度估计 |
| `/slosh/ay_est` | `std_msgs/Float32` | scout_local_planner | 监控（可选） | 横向加速度估计 |
| `/slosh/alpha_est` | `std_msgs/Float32` | scout_local_planner | 监控（可选） | 角加速度估计 |

---

## 3) 定位/建图（AMCL / Gmapping）

| 话题 | 类型 | 发布方 | 订阅方 | 说明 |
|------|------|--------|--------|------|
| `/map` | `nav_msgs/OccupancyGrid` | map_server / gmapping | move_base / RViz | 地图 |
| `/tf` | `tf2_msgs/TFMessage` | 多个节点 | 所有 | 坐标变换（含 `map -> odom -> base_link`） |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz | amcl | AMCL 初始位姿设置 |

---

## 4) 传感器（Nanoscan3）

| 话题 | 类型 | 发布方 | 订阅方 | 说明 |
|------|------|--------|--------|------|
| `/scan_front` | `sensor_msgs/LaserScan` | nanoscan3_bringup | gmapping / amcl | 前置激光雷达扫描 |
| `/scan_front_filtered` | `sensor_msgs/LaserScan` | nanoscan3_bringup | 可选 | 过滤后的扫描 |

---

## 5) 底盘与基础状态（已观测）

| 话题 | 类型 | 发布方 | 订阅方 | 说明 |
|------|------|--------|--------|------|
| `/odom` | `nav_msgs/Odometry` | 底盘驱动 | 多模块 | 里程计 |
| `/cmd_vel` | `geometry_msgs/Twist` | 上层控制 | 底盘驱动 | 速度控制 |
| `/tf` | `tf2_msgs/TFMessage` | 底盘/静态TF | 多模块 | 坐标变换 |

## 6) BMS / 底盘状态

| 话题 | 类型 | 发布方 | 订阅方 | 说明 |
|------|------|--------|--------|------|
| `/BMS_status` | `scout_msgs/BMSStatus` | 底盘驱动/监控节点 | 监控端 | 电池状态 |
| `/scout_status` | `scout_msgs/ScoutStatus` | 底盘驱动 | 监控端 | 底盘状态 |
| `/rs_status` | `scout_msgs/ScoutStatus`（待确认） | 底盘驱动 | 监控端 | 运行状态（不同固件可能复用） |
| `/scout_light_control` | `scout_msgs/ScoutLightCmd`（待确认） | 上层控制 | 底盘驱动 | 灯光控制 |

---

## 7) 系统话题（ROS）

| 话题 | 类型 | 发布方 | 订阅方 | 说明 |
|------|------|--------|--------|------|
| `/rosout` | `rosgraph_msgs/Log` | ROS core | 工具节点 | 日志输出 |
| `/rosout_agg` | `rosgraph_msgs/Log` | ROS core | 工具节点 | 日志聚合 |


## 8) 数据流链路详解

### 8.1 全局路径 → MPC 控制指令 完整链路

```
┌─────────────────────────────────────────────────────────────────────┐
│  全局规划器（move_base / MBF / simple_global_planner）              │
│  输出: nav_msgs::Path（几何路径）                                   │
│  常见信息: x, y, yaw                                                │
│  不直接包含: vx, vy, omega, 时间参数化轨迹                         │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           │ /scout/global_path (nav_msgs::Path)
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  LocalPlannerROS::globalPathCallback()                              │
│  → path_handler_.updateGlobalPath(path, v_des)                      │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  PathHandler（scout_local_planner 内部类）                          │
│                                                                     │
│  ① updateGlobalPath(path, v_des)                                    │
│     - 缓存原始 global_path（通常是 map 系）                         │
│     - 做路径相似性检测，必要时设置 reset_hint                       │
│     - 可选重采样：resamplePath()                                    │
│     - 构建 global_spline_（全局样条）                               │
│     - 调用 updateSpeedProfile(v_des) 生成 v(s)                      │
│                                                                     │
│  ② updateSpeedProfile(v_des)                                        │
│     - 基于 global_spline_ 生成速度曲线 v(s)                         │
│     - 主要步骤：                                                    │
│       1) 曲率限速: v <= sqrt(max_lat_accel / |kappa|)              │
│       2) 前向加速扫描                                               │
│       3) 反向减速扫描                                               │
│     - 末端会按 goal_speed 收尾                                      │
│     - 注意：goal_capture_min_speed 不在这里生效                     │
│                                                                     │
│  ③ getReferencePoints(N, dt, v_des, ref_points)                     │
│     - 只把“局部窗口”从 global_path frame 变换到 base_link           │
│     - 拟合 local_spline_（局部样条）                                │
│     - 若 time_parameterize=true：                                   │
│       用 s_progress + v_ref*dt 推进参考序列                         │
│     - 终点捕获区在这里补最低参考速度：                              │
│       goal_capture_min_speed                                        │
│     - 输出 vector<ReferencePoint>                                   │
│                                                                     │
│  ④ getFrenetState(frenet)                                           │
│     - 在 base_link 下用 local_spline_ 投影                          │
│     - 输出 e_l / e_c / e_theta                                      │
│                                                                     │
│  ⑤ getMaxCurvatureAhead(lookahead_dist, preview_dist)               │
│     - 基于 s_global_ 和 global_spline_                              │
│     - 返回前方窗口最大 |kappa|                                       │
│                                                                     │
│  ⑥ isGoalReached() / getGoalDistance()                              │
│     - 在 base_link 下检查终点距离                                   │
│     - yaw 使用路径末端切线方向，不是直接用 goal.pose.orientation    │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           │ ref_points[0..N-1], frenet
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  LocalPlannerROS::controlLoop()                                     │
│                                                                     │
│  ① updateSloshEstimate()                                            │
│     - odom 差分估计 ax / alpha                                      │
│     - ay 默认用 v * omega 近似                                      │
│     - 若阶段 7 IMU 参数启用且有数据，可切到 IMU ay / omega / alpha  │
│                                                                     │
│  ② speed governor（外环启发式速度治理）                              │
│     - 基础速度目标: v_des_cmd = vehicle.v_max * 0.8                 │
│     - 输入: slosh_height, predicted_height_max, kappa_preview       │
│     - 输出: v_des_eff                                                │
│     - 若激活，不是直接改 refs[k]，而是重新调用                      │
│       getReferencePoints(..., v_des_eff, ...)                        │
│                                                                     │
│  ③ 构建 MPC 初始状态 x0                                             │
│     x0 = [e_l, e_c, e_theta, v,                                     │
│           eta_x, eta_x_dot, eta_y, eta_y_dot]                        │
│                                                                     │
│  ④ MPCSolver::solve(x0, ref_points)                                 │
│     - 线性化动力学                                                  │
│     - 构建 QP                                                       │
│     - OSQP 求解                                                     │
│     - 提取 v_cmd / omega_cmd / x_predicted / u_optimal              │
│                                                                     │
│  ⑤ publishCmdVel(v_cmd, omega_cmd)                                  │
│     - 执行端做 EMA 低通                                              │
│     - 发布 /cmd_vel                                                 │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           │ /cmd_vel
                           ▼
                      底盘驱动执行
```

### 8.2 MBF 相关的话题前后关系

如果你用的是 `mbf_global.launch` / `mbf_global_sim.launch`，当前链路是：

- 输入 goal：
  - `/scout/goal`
- MBF 内部 action：
  - `/scout/mbf_costmap_nav/get_path`
- MBF / 插件内部可见话题：
  - `/scout/mbf_costmap_nav/GlobalPlanner/plan`
  - `/scout/mbf_costmap_nav/GlobalPlanner/potential`
- 统一发布给局部规划器的全局路径：
  - `/scout/global_path`

也就是说：

- **MBF 负责生成并发布全局几何路径**
- **PathHandler 负责把 `/scout/global_path` 处理成 MPC 的局部参考序列**
- `/scout/global_path_smooth` 是 local planner 侧的平滑路径可视化，不是 MBF 直接输出
- `/local_path` 是 MPC 预测/局部轨迹可视化，也不是 MBF 输出

### 8.3 关键接口数据结构

#### ReferencePoint（PathHandler → MPCSolver）

当前真实结构为：

```cpp
struct ReferencePoint {
  double x;           // 路径点 x 坐标（base_link 系）
  double y;           // 路径点 y 坐标（base_link 系）
  double theta_path;  // 路径切线方向 [rad]
  double kappa;       // 路径曲率 [1/m]
  double v_path;      // 路径推进速度 [m/s]
  double s;           // 弧长参数
  double v_ref;       // 参考速度 [m/s]
};
```

说明：

- `ReferencePoint` **没有**单独的 `omega_ref` 字段
- 当前 `omega_ref` 是在代价函数里临时计算：
  - `omega_ref = v_ref * kappa`

#### StateVector（MPC 增广状态，8 维）

```text
索引  名称          含义
[0]   E_L           纵向误差 (m)
[1]   E_C           横向误差 (m)
[2]   E_THETA       航向误差 (rad)
[3]   V             线速度 (m/s)
[4]   ETA_X         X方向模态位移 [m]
[5]   ETA_X_DOT     X方向模态速度 [m/s]
[6]   ETA_Y         Y方向模态位移 [m]
[7]   ETA_Y_DOT     Y方向模态速度 [m/s]
```

#### ControlVector（MPC 控制量，2 维）

```text
索引  名称          含义                    约束范围
[0]   A             纵向加速度 (m/s²)       [-a_max, a_max]
[1]   OMEGA         角速度 (rad/s)          [-omega_max, omega_max]
```

### 8.4 当前代价函数与约束的真实含义

当前 MPC 本质上是 tracking MPC。核心目标是：

- 跟踪误差最小：
  - `e_l`
  - `e_c`
  - `e_theta`
- 跟踪参考速度：
  - `v -> v_ref`
- 控制平滑：
  - `a`
  - `omega`
  - `Δa`
  - `Δomega`
- 可选曲率前馈：
  - `omega_ref = v_ref * kappa`
- 可选 slosh 软代价：
  - `Q_slosh_eta * (eta_x^2 + eta_y^2)`

当前主要约束有：

- 状态约束：
  - `v_min <= v <= v_max`
- 控制约束：
  - `|a| <= a_max`
  - `|omega| <= omega_max`
- 控制变化率约束：
  - `|a_k - a_{k-1}| <= j_max * dt`（若启用）
  - `|omega_k - omega_{k-1}| <= alpha_max * dt`
- 第一版 slosh 盒约束（若启用）：
  - `|eta_x| <= eta_bar`
  - `|eta_y| <= eta_bar`
  - 其中 `eta_bar` 不是简单 `slosh_height_max / height_coeff`
  - 当前实现会先扣掉抛物面项预算，再除以 `height_coeff * sqrt(2)`

### 8.5 当前不是 MPCC，而是“局部参考跟踪型 MPC”

当前系统的真实定位是：

- ROS 接口上：局部规划器
- 优化本质上：参考路径跟踪型 MPC

它不是 MPCC 的原因是：

- 没有把路径进度 `s` 放进优化变量
- 没有 progress reward
- 没有把走廊/障碍物半空间约束作为主几何约束并入优化
- 参考轨迹是 `PathHandler` 先生成，再由 `MPCSolver` 去跟踪

所以当前主范式是：

- `global_path -> local reference -> MPC tracking`

而不是：

- `progress + geometry + obstacle + control` 的统一联合优化

### 8.6 speed governor 外环 vs MPC 内环 的职责分离

```
                          ┌──────────────────────────┐
                          │  PathHandler             │
                          │  生成 ref_points         │
                          └────────────┬─────────────┘
                                       │
                    ┌──────────────────▼──────────────────┐
                    │  Speed Governor（外环启发式）        │
                    │                                     │
                    │  输入:                               │
                    │    - slosh_height（当前估计）        │
                    │    - predicted_height_max（预测峰值）│
                    │    - kappa_preview（前方曲率）       │
                    │                                     │
                    │  输出:                               │
                    │    v_des_eff                         │
                    │                                     │
                    │  实现方式:                           │
                    │    重新生成一套 ref_points，而不是   │
                    │    直接就地改 refs[k].v_ref         │
                    └──────────────────┬──────────────────┘
                                       │
                    ┌──────────────────▼──────────────────┐
                    │  MPC 求解器（内环优化）              │
                    │                                     │
                    │  目标:                               │
                    │    - 跟踪 ref_points                 │
                    │    - 最小化 e_c, e_l, e_theta       │
                    │    - 跟踪 v_ref                      │
                    │    - 最小化 slosh 软代价             │
                    │    - 最小化控制量和变化率            │
                    │                                     │
                    │  局限:                               │
                    │    当前仍是 tracking MPC，不是 MPCC  │
                    └──────────────────┬──────────────────┘
                                       │
                                       ▼
                                   /cmd_vel
```

总结：

- Governor 负责“预测域外的启发式风险治理”
- MPC 负责“预测域内的最优跟踪与约束满足”
- 两者当前是互补关系，而不是重复关系

### 8.7 输出与可视化注意点

- `MPCSolver` 当前输出不是旧版的：
  - `v_cmd != x_1(V)`
  - `omega_cmd != u_0(OMEGA)`
- 当前实际提取方式是：
  - `v_cmd = v0 + 0.5 * a0 * dt`
  - `omega_cmd = 0.5 * (omega0 + omega1)`

- `publishCmdVel()` 当前角速度滤波是：
  - `effective_alpha_omega = clamp(alpha_omega + kappa_boost * |omega|, 0, 1)`
  - 然后再做 EMA

- `/local_path` 当前是预测轨迹可视化：
  - 起点强制放在当前 `base_link` 原点
  - 位置恢复时主要使用 `e_c`，刻意忽略 `e_l`，以减少弯道可视化锯齿
