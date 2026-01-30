# 仿真 vs 实物 对齐记录（话题 / 命名空间 / TF）
codex resume 019c0a43-c32b-71f1-b6e9-938e78ce16dc
> 目标：仿真话题、命名空间、frame 与实物对齐。

## 实物流程

### 1. 底盘 + CAN
```
roslaunch scout_bringup scout_mini_robot_base.launch
```
- 话题
  - 输入：`/cmd_vel`
  - 输出：`/odom`
- 命名空间：无
- TF
  - `odom -> base_link`

### 2. 键盘控制
```
roslaunch scout_bringup scout_teleop_keyboard.launch
```
- 话题
  - 输出：`/cmd_vel`
- 命名空间：无
- TF：无

### 3. 激光雷达（真实）
```
roslaunch nanoscan3_bringup nanoscan3_front.launch use_rviz:=false
```
- 话题
  - 输出：`/scan_front`（`/scan_front_filtered` 可选）
- 命名空间：无
- TF
  - `base_link -> nanoscan3_front`
- LaserScan frame_id：`nanoscan3_front`

### 4. 建图（真实）
#### Gmapping
```
roslaunch nanoscan3_mapping scout_nanoscan3_gmapping.launch fake_odom_tf:=false use_rviz:=true
```
- 话题
  - 输入：`/scan_front`
  - 输出：`/map`
- 命名空间：无
- TF
  - `map -> odom`

#### Cartographer
```
roslaunch nanoscan3_mapping scout_nanoscan3_cartographer_sim.launch
```
- 话题
  - 输入：`/scan_front`, `/odom`
  - 输出：`/map`
- 命名空间：无
- TF
  - `map -> odom`

### 5. 定位（真实）
```
roslaunch nanoscan3_localization scout_nanoscan3_amcl.launch use_rviz:=true
```
- 话题
  - 输入：`/scan_front`, `/map`, `/odom`
- 命名空间：无
- TF
  - `map -> odom`

### 6. 全局规划
```
roslaunch scout_global_planner move_base_global.launch
```
- 话题（命名空间 `/scout`）
  - 输入：`/scout/goal`, `/map`, `/odom`
  - 输出：`/scout/global_path`, `/scout/move_base_cmd_vel`
- 命名空间：`/scout`
- TF
  - `map -> odom -> base_link`

### 7. MPC 局部规划
```
roslaunch scout_local_planner test_mpc.launch
```
- 话题
  - 输入：`/scout/global_path`, `/odom`
  - 输出：`/cmd_vel`
- 命名空间：无（话题显式 remap）
- TF
  - 依赖 `map/odom/base_link`

---

## 仿真流程

### 1. 启动仿真环境
```
roslaunch scout_description scout_mini_gazebo.launch use_rviz:=false
```
- 话题
  - 输入：`/cmd_vel`
  - 输出：`/odom`
- 命名空间：无
- TF
  - `odom -> base_link`
  - `base_link -> base_footprint`（静态）

### 2. 键盘控制（仿真）
```
roslaunch scout_bringup scout_teleop_keyboard.launch
```
- 话题
  - 输出：`/cmd_vel`
- 命名空间：无
- TF：无

### 3. 激光雷达（仿真）
```
roslaunch nanoscan3_bringup nanoscan3_front_sim.launch use_rviz:=false
```
- 话题
  - 输入：`/scan`
  - 输出：`/scan_front`
- 命名空间：无
- TF
  - `base_link -> nanoscan3_front`
- LaserScan frame_id：`nanoscan3_front`

### 4. 建图（仿真）
#### Gmapping
```
roslaunch nanoscan3_mapping scout_nanoscan3_gmapping_sim.launch use_rviz:=true
```
- 话题
  - 输入：`/scan_front`
  - 输出：`/map`
- 命名空间：无
- TF
  - `map -> odom`

#### Cartographer
```
roslaunch nanoscan3_mapping scout_nanoscan3_cartographer_sim.launch
```
- 话题
  - 输入：`/scan_front`, `/odom`
  - 输出：`/map`
- 命名空间：无
- TF
  - `map -> odom`

### 5. 定位（仿真）
#### AMCL
```
roslaunch nanoscan3_localization scout_nanoscan3_amcl_sim.launch use_rviz:=true
```
- 话题
  - 输入：`/scan_front`, `/map`, `/odom`
- 命名空间：无
- TF
  - `map -> odom`

#### Cartographer Localization
```
roslaunch nanoscan3_localization scout_nanoscan3_cartographer_localization_sim.launch
```
- 话题
  - 输入：`/scan_front`, `/odom`
  - 输出：`/map`
- 命名空间：无
- TF
  - `map -> odom`

### 6. 全局规划（仿真）
```
roslaunch scout_global_planner move_base_global_sim.launch
```
- 话题（命名空间 `/scout`）
  - 输入：`/scout/goal`, `/map`, `/odom`
  - 输出：`/scout/global_path`, `/scout/move_base_cmd_vel`
- 命名空间：`/scout`
- TF
  - `map -> odom -> base_link`

### 7. MPC 局部规划（仿真）
```
roslaunch scout_local_planner test_mpc.launch
```
- 话题
  - 输入：`/scout/global_path`, `/odom`
  - 输出：`/cmd_vel`
- 命名空间：无（话题显式 remap）
- TF
  - 依赖 `map/odom/base_link`

---

## 对齐说明（关键点）
- 激光话题统一为：`/scan_front`
- 雷达 frame 统一为：`nanoscan3_front`
- 底盘 base frame 统一为：`base_link`
- 里程计链路统一为：`map -> odom -> base_link`
- 全局规划统一命名空间：`/scout`
