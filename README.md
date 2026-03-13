# Scout Mini 自主导航工作空间

> **工作空间路径**：`/home/a/scout_ws`（本地）  
> **运行环境**：ROS 1（Noetic）、Ubuntu 20.04、Python 3

本工作空间基于 **Scout Mini 差速移动机器人**，实现了完整的自主导航功能，并在 MPC 局部规划器中集成了**液体晃动抑制（Anti-Slosh）**模块。当前主控链路为：

```
/scout/goal
    ↓
全局规划器（MBF / move_base / simple）
    ↓
/scout/global_path  (nav_msgs/Path)
    ↓
scout_local_planner（MPC + Anti-Slosh）
    ↑
/odom + TF
    ↓
/cmd_vel → scout_base_node → 底盘
```

---

## 目录

- [项目结构](#项目结构)
- [功能包说明](#功能包说明)
- [编译](#编译)
- [实物启动流程](#实物启动流程)
- [仿真启动流程](#仿真启动流程)
- [关键 ROS 话题](#关键-ros-话题)
- [关键参数速查](#关键参数速查)
- [文档索引](#文档索引)

---

## 项目结构

```
scout_ws_real/
├── src/
│   ├── scout_apps/
│   │   ├── control/
│   │   │   ├── scout_local_planner/       # 主用 MPC 局部规划器（含 Anti-Slosh）
│   │   │   ├── scout_omni_local_planner/  # 全向运动 MPC（备用）
│   │   │   ├── slosh_models/              # 液体晃动动力学模型库
│   │   │   └── teb_local_planner/         # TEB 局部规划器（备用/对比）
│   │   ├── navigation/
│   │   │   └── scout_global_planner/      # 全局规划入口（MBF / move_base / simple）
│   │   └── sensors/
│   │       ├── nanoscan3_bringup/         # SICK nanoscan3 激光雷达驱动
│   │       ├── nanoscan3_mapping/         # 建图（Gmapping / Cartographer）
│   │       ├── nanoscan3_localization/    # 定位（AMCL / Cartographer）
│   │       └── scout_maps/               # 地图存储
│   ├── scout_ros/                         # 底盘驱动、消息、URDF
│   │   ├── scout_base/                    # 底盘通信节点
│   │   ├── scout_bringup/                 # 启动文件集合
│   │   ├── scout_description/             # URDF / Gazebo 仿真
│   │   └── scout_msgs/                    # 自定义消息
│   ├── move_base_flex/                    # MBF 导航框架
│   ├── ugv_sdk/                           # Scout 底层 CAN SDK
│   └── mpc_planner/                       # MPC 算法参考工程（仅参考）
└── docs/                                  # 项目文档（中文）
```

---

## 功能包说明

| 功能包 | 职责 |
|---|---|
| `scout_local_planner` | 核心 MPC 局部规划器；`PathHandler` 生成局部参考，`MPCSolver` 求解控制量并输出 `/cmd_vel`；集成液体晃动抑制（软代价、盒约束、速度治理） |
| `slosh_models` | 提供液体晃动线性弹簧-质量动力学模型，由 `SloshIntegration` 调用 |
| `scout_global_planner` | 全局路径生成入口；支持 MBF（推荐）、move_base、simple 三种模式；统一输出 `/scout/global_path` |
| `nanoscan3_bringup` | SICK nanoscan3 激光雷达驱动，输出 `/scan_front` |
| `nanoscan3_mapping` | Gmapping / Cartographer 建图 |
| `nanoscan3_localization` | AMCL / Cartographer 定位 |
| `scout_base` | CAN 总线通信，订阅 `/cmd_vel`，发布 `/odom`、`/scout_status`、`/BMS_status` |
| `scout_bringup` | 汇总启动文件（底盘、仿真、键盘控制等） |
| `scout_description` | Scout Mini URDF 与 Gazebo 仿真世界 |
| `move_base_flex` | MBF 导航框架（全局规划 + 代价地图） |

---

## 编译

```bash
# 克隆后在工作空间根目录执行
cd /home/a/scout_ws
catkin_make

# 若只编译特定包（白名单会被缓存，用完需清除！）
catkin_make -DCATKIN_WHITELIST_PACKAGES="scout_local_planner"
# 恢复编译所有包
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

source devel/setup.bash
```

> ⚠️ `catkin_make` 的白名单会被缓存，若只编译了单包后忘记清除，后续修改其他包不会生效。

---

## 实物启动流程

按顺序依次在新终端中执行：

### 1. 建立 CAN 通信并启动底盘

```bash
sudo modprobe gs_usb          # 若需要加载驱动
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
roslaunch scout_bringup scout_mini_robot_base.launch
```

### 2. 键盘控制（可选，调试用）

```bash
roslaunch scout_bringup scout_teleop_keyboard.launch
```

### 3. 激光雷达

```bash
roslaunch nanoscan3_bringup nanoscan3_front.launch use_rviz:=false
```

### 4. 建图（首次建图或地图更新时执行）

```bash
# Gmapping
roslaunch nanoscan3_mapping scout_nanoscan3_gmapping.launch fake_odom_tf:=false use_rviz:=true
# 或 Cartographer（推荐）
roslaunch nanoscan3_mapping scout_nanoscan3_cartographer.launch
```

### 5. 定位

```bash
# AMCL
roslaunch nanoscan3_localization scout_nanoscan3_amcl.launch use_rviz:=true
# 或 Cartographer 定位
roslaunch nanoscan3_localization scout_nanoscan3_cartographer_localization.launch
```

### 6. 全局规划

```bash
# 推荐：MBF 模式
roslaunch scout_global_planner mbf_global.launch
# 或 move_base 模式
roslaunch scout_global_planner move_base_global.launch mode:=mpc
# 或轻量 simple 模式
roslaunch scout_global_planner simple_global_planner.launch
```

### 7. MPC 局部规划

```bash
# 日常跟踪（无晃动抑制）
roslaunch scout_local_planner test_mpc.launch

# 液体晃动抑制实验（推荐使用 slosh_experiment.launch）
roslaunch scout_local_planner slosh_experiment.launch \
  Q_slosh:=5 \
  enable_slosh_box_constraint:=true \
  slosh_speed_governor_enable:=true
```

### 8. 录制实验数据（与步骤 7 同时，另开终端）

```bash
cd $(rospack find scout_local_planner)
./scripts/record_slosh_experiment.sh 5    # 参数 = 当前 Q_slosh 值
# Ctrl+C 停止录制
```

---

## 仿真启动流程

### 1. 启动 Gazebo

```bash
roslaunch scout_description scout_mini_gazebo.launch use_rviz:=false
```

### 2. 激光雷达（可选）

```bash
roslaunch nanoscan3_bringup nanoscan3_front_sim.launch use_rviz:=false
```

### 3. 建图（仿真）

```bash
# Gmapping
roslaunch nanoscan3_mapping scout_nanoscan3_gmapping_sim.launch use_rviz:=true
# Cartographer（需先 source cartographer 工作空间）
source /home/a/scout_ws/devel/setup.bash
source /home/a/scout_ws/src/scout_apps/sensors/cartographer_ws/install_isolated/setup.bash
roslaunch nanoscan3_mapping scout_nanoscan3_cartographer_sim.launch
```

### 4. 定位（仿真）

```bash
roslaunch nanoscan3_localization scout_nanoscan3_amcl_sim.launch use_rviz:=true
```

### 5. 全局规划（仿真）

```bash
roslaunch scout_global_planner mbf_global_sim.launch
```

### 6. MPC 局部规划（仿真）

```bash
# 日常仿真跟踪（默认 Q_slosh=5）
roslaunch scout_local_planner test_mpc_sim.launch

# 液体晃动实验
roslaunch scout_local_planner slosh_experiment.launch \
  sim:=true \
  Q_slosh:=5 \
  enable_slosh_box_constraint:=true \
  slosh_speed_governor_enable:=true
```

---

## 关键 ROS 话题

### 控制与导航

| 话题 | 类型 | 说明 |
|---|---|---|
| `/scout/goal` | `geometry_msgs/PoseStamped` | 导航目标输入 |
| `/scout/global_path` | `nav_msgs/Path` | 全局规划路径（MPC 上游输入） |
| `/odom` | `nav_msgs/Odometry` | 里程计（MPC 当前状态输入） |
| `/cmd_vel` | `geometry_msgs/Twist` | 最终速度指令输出至底盘 |
| `/mpc_status` | `std_msgs/String` | MPC 状态机：`IDLE / TRACKING / REACHED / ERROR` |
| `/mpc/status_val` | `std_msgs/Int32` | MPC 求解结果：1=成功，0=失败 |
| `/local_path` | `nav_msgs/Path` | MPC 预测轨迹（可视化） |

### 液体晃动抑制

| 话题 | 说明 |
|---|---|
| `/slosh/state` | 晃动状态 `[eta_x, eta_x_dot, eta_y, eta_y_dot]` |
| `/slosh/height` | 当前液面高度估计（模型值） |
| `/slosh/height_pred_max` | 预测域内最大液面高度 |
| `/slosh/v_des_eff` | speed governor 输出的有效参考速度 |
| `/slosh/speed_governor_active` | speed governor 是否介入 |
| `/slosh/constraint_active` | 预测峰值是否超阈 |

---

## 关键参数速查

> 配置文件：`src/scout_apps/control/scout_local_planner/config/mpc_params.yaml`（实物）  
> 仿真配置：`config/mpc_params_sim.yaml`

| 参数 | 当前值（实物） | 说明 |
|---|---|---|
| `mpc/Q_slosh` | `0.0`（配置文件默认；实验推荐从 `5` 起步，通过 launch arg 覆盖） | 晃动软代价权重（消融：0 / 5 / 10 / 20） |
| `mpc/Q_ec` | `30.0` | 横向误差权重 |
| `mpc/Q_v` | `8.0` | 速度跟踪权重 |
| `vehicle/v_max` | `1.0` m/s | 最大线速度 |
| `vehicle/omega_max` | `1.0` rad/s | 最大角速度 |
| `path_handler/lookahead_distance` | `0.5` m | 前视距离 |
| `path_handler/max_lat_accel` | `1.5` m/s² | 最大横向加速度 |
| `mpc/enable_slosh_box_constraint` | `false` | 液面盒约束开关 |
| `slosh_speed_governor/enable` | `false` | 速度治理开关 |
| `slosh/container_radius` | `0.15` m | 容器半径（按实物量测） |
| `slosh/liquid_height` | `0.20` m | 液面高度（按实物量测） |

---

## 文档索引

| 文档 | 说明 |
|---|---|
| [`docs/总结1.md`](docs/总结1.md) | 项目技术总结：系统定位、MPC 结构、路径信息流、液体晃动抑制接入情况 |
| [`docs/change_log.md`](docs/change_log.md) | 开发日志：启动流程、参数速查、变更记录 |
| [`docs/topic_list.md`](docs/topic_list.md) | 完整实物 ROS 话题清单 |
| [`docs/compared_with_mpc_planner.md`](docs/compared_with_mpc_planner.md) | 与参考 MPC 工程的对比分析 |
| [`docs/gazebo_vs_reality.md`](docs/gazebo_vs_reality.md) | 仿真与实物差异说明 |
| [`src/scout_apps/control/scout_local_planner/launch/README.md`](src/scout_apps/control/scout_local_planner/launch/README.md) | 三个 launch 文件区别与调参指导 |
