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
