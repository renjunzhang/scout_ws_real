# scout_ws 话题清单

> 仅记录当前工作区配置与文档中明确涉及的话题，按模块分类。
> 若节点放在 `/scout` 命名空间下，下面的相对话题会自动变为 `/scout/...`。
> 部分话题为“配置/规划”用途，实际是否存在取决于对应节点是否已启动。

---

## 0) 当前系统已观测话题（2026-01-29）

| 话题 | 说明 |
|------|------|
| `/BMS_status` | 电池状态 |
| `/cmd_vel` | 速度控制 |
| `/odom` | 里程计 |
| `/rosout` | ROS 日志 |
| `/rosout_agg` | ROS 日志聚合 |
| `/rs_status` | 底盘状态（类型待确认） |
| `/scout_light_control` | 灯光控制 |
| `/scout_status` | 底盘状态 |
| `/tf` | 坐标变换 |

---

## 1) 全局规划（move_base / scout_global_planner）

| 话题 | 类型 | 发布方 | 订阅方 | 说明 |
|------|------|--------|--------|------|
| `/scout/goal` | `geometry_msgs/PoseStamped` | RViz / 上层任务 | move_base | 目标点输入（需在 RViz 将 2D Nav Goal 话题改为此） |
| `/scout/global_path` | `nav_msgs/Path` | move_base 全局规划器 | scout_local_planner | 全局路径输出（由 `~GlobalPlanner/plan` / `~NavfnROS/plan` 重映射） |
| `/scout/move_base_cmd_vel` | `geometry_msgs/Twist` | move_base | 无（仅调试） | move_base 输出速度，已重映射避免干扰 `/cmd_vel` |

> 说明：move_base 插件原始私有话题 `~GlobalPlanner/plan` / `~NavfnROS/plan` 已在 launch 中 remap 到 `/scout/global_path`。

---

## 2) 局部规划（scout_local_planner）

| 话题 | 类型 | 发布方 | 订阅方 | 说明 |
|------|------|--------|--------|------|
| `/scout/global_path` | `nav_msgs/Path` | move_base | scout_local_planner | 全局路径输入 |
| `/odom` | `nav_msgs/Odometry` | 底盘驱动 | scout_local_planner | 里程计输入 |
| `/cmd_vel` | `geometry_msgs/Twist` | scout_local_planner | 底盘驱动 | 速度控制输出 |
| `/local_path` | `nav_msgs/Path` | scout_local_planner | RViz（可选） | 局部轨迹可视化 |
| `/slosh_height` | `std_msgs/Float64` | scout_local_planner | 监控（可选） | 晃动高度输出（第 2 步集成后启用） |

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
