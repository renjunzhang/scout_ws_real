# scout_local_planner

更新时间：2026-05-30

本包当前维护一条主线：**SloshPriorityMPC 固定路径对比实验**。

当前目标不是维护历史路径后处理方案，而是支撑下面这条可复现实物实验链：

```text
固定 P2_s_curve 路径
  -> PathHandler 生成 reference horizon
  -> 可选 external_speed_profile_csv 注入 TOPPRA/Ruckig/Biagiotti-style v_ref(s)
  -> 8 维增广 MPC tracking
  -> terminal envelope / profile execution cap / cmd_vel
  -> rosbag
  -> RGB 真值 + /slosh/height 模型辅助 + cost breakdown + terminal 诊断
```

当前正式对比实验 SOP 以此文档为准：

```text
docs/重要文档/20260527_SloshPriorityMPC正式对比实验验证方案.md
```

`20260518_MPC终点收敛与固定路径验证方案.md` 只作为 terminal / fixed-path 基础流程的历史参考。

## 当前实验对象

正文主线只比较固定路径上的控制策略，不比较几何路径规划器。

| 方法 | 含义 | 路径几何 | 速度参考 |
|---|---|---|---|
| C | 普通 tracking MPC | 固定 | 内部 `v_ref(s)` |
| D | slosh modal cost only | 固定 | 内部 `v_ref(s)` |
| E | non-slosh smooth MPC | 固定 | 内部 `v_ref(s)` |
| F | SloshPriorityMPC | 固定 | 内部 `v_ref(s)` |
| RPP-style | RPP 启发的非晃液 `v_ref` 速度调节 baseline | 固定 | 内部 `v_ref(s)` + regulator |
| TOPPRA-style | 限加速度 retiming baseline | 固定 | 外部 CSV |
| Ruckig-style | 限 jerk retiming baseline | 固定 | 外部 CSV |
| Biagiotti-style | 开环 slosh-aware reference shaping baseline | 固定 | 外部 CSV |

主效果窗口固定为：

```text
TRACKING start -> first terminal/capture - 1s
```

terminal approach 只做工程诊断，不进入 slosh cost 主效果统计。

## 核心模块

| 文件 | 职责 |
|---|---|
| `src/local_planner_ros.cpp` / `include/.../local_planner_ros.h` | ROS 节点编排：订阅、参数、状态机、控制循环 |
| `src/path_handler.cpp` / `include/.../path_handler.h` | 路径清洗、平滑、内部速度剖面、外部速度 CSV 注入、reference horizon |
| `src/mpc_solver.cpp` / `include/.../mpc_solver.h` | OSQP QP 构建、求解、warm start、解提取 |
| `src/cost_function.cpp` / `include/.../cost_function.h` | MPC 优化代价：tracking、速度、控制、控制变化率、slosh state |
| `src/cost_breakdown.cpp` / `include/.../cost_breakdown.h` | `/mpc/cost_breakdown` 统计计算 |
| `src/diff_drive_model.cpp` / `include/.../diff_drive_model.h` | 差速底盘 + slosh 增广状态预测模型 |
| `src/constraint_manager.cpp` / `include/.../constraint_manager.h` | 速度、角速度、加速度、控制变化率等约束 |
| `src/slosh_integration.cpp` / `include/.../slosh_integration.h` | 运行时 slosh 二阶模型状态传播和 `/slosh/height` |
| `src/slosh_feedback.cpp` / `include/.../slosh_feedback.h` | odom/IMU 到 `ax/ay/omega/alpha` 的反馈估计 |
| `src/terminal_controller.cpp` / `include/.../terminal_controller.h` | terminal slowdown、capture、REACHED 判定、post-MPC 速度 clamp |
| `src/profile_execution_cap.cpp` / `include/.../profile_execution_cap.h` | TOPPRA/Ruckig/Biagiotti 外部速度剖面的执行层 hard cap |
| `src/diagnostics_publisher.cpp` / `include/.../diagnostics_publisher.h` | `/slosh/*`、`/mpc/*`、`/reference/*`、`/terminal/*`、`/profile_cap/*` 发布 |

## MPC 状态和代价

状态：

```text
x = [e_l, e_c, e_theta, v, eta_x, eta_dot_x, eta_y, eta_dot_y]
```

控制：

```text
u = [a, omega]
```

单步代价口径：

```text
tracking error
+ velocity tracking
+ control effort
+ control-rate smoothness
+ slosh eta / eta_dot state penalty
```

`eta / eta_dot` 是二阶 ODE 递推状态，包含历史激励的记忆；不能把它理解成当前瞬时加速度的简单函数。

## 速度参考

默认速度参考由 `PathHandler` 内部生成：

```text
fixed path -> curvature / acceleration constraints -> v_ref(s)
```

外部 profile baseline 使用：

```text
external_speed_profile_csv
```

CSV 最小接口：

```text
s_normalized,v_ref_m_s
0.000,0.000
...
1.000,0.000
```

CSV 中可以带 `a_ref_m_s2`、`jerk_ref_m_s3` 等列，但 `PathHandler` 当前只消费 `s_normalized` 和 `v_ref_m_s`。

## 主要 launch

| 文件 | 用途 |
|---|---|
| `launch/slosh_experiment.launch` | 实物 MPC 入口 |
| `launch/slosh_experiment_sim.launch` | 仿真 MPC 入口 |

实物默认路径话题：

```text
global_path_topic:=/scout/global_path_fixed
```

外部速度 baseline 额外设置：

```text
external_speed_profile_csv:=<csv>
external_profile_execution_cap_enable:=true
```

## 实物入口

传感器 / 底盘 / 定位：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

录包：

```bash
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh <Q_SLOSH> <suffix>
```

## 必须保留的话题

录包和分析依赖：

```text
/camera/color/image_raw
/camera/color/camera_info
/imu/data
/odom
/cmd_vel
/scout/global_path_fixed
/local_path
/mpc/status_val
/mpc/cost_breakdown
/mpc/slosh_horizon_summary
/reference/v_ref
/reference/v_ref_horizon
/reference/s_horizon
/reference/implied_ax
/reference/implied_ay
/reference/implied_jerk
/terminal/*
/profile_cap/*
/slosh/*
```

## 边界

当前主线不维护：

```text
在线 path post-processor
历史路径后处理策略验证
risk scheduler
历史 runtime input shaping / ISR
speed governor
旧内置低激励 profile
terminal recovery 几何接管分支
heading_align / settling / tracking_curvature_speed_cap 旧分支
```

脚本目录中可能仍保留历史分析或旧数据复查文件；它们不属于当前实物 SOP。

## 判断原则

- RGB 视觉液面是实物主指标。
- `/slosh/height` 是模型辅助指标，不能自证真实防晃。
- C/D/E/F 不改变路径几何，也不改变外部速度剖面。
- TOPPRA/Ruckig/Biagiotti-style 只改变同一固定路径上的 `v_ref(s)`。
- RPP-style 不完整复现 Nav2 RPP controller，只作为同 MPC 后端下的速度调节 baseline。
- terminal 段只诊断停车平顺性，不进入主效果统计。
- completion time 差异超过 10% 时，按 trade-off 解释。
