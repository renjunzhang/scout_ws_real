# scout_local_planner

更新时间：2026-05-31

本包当前维护一条主线：**SloshPriorityMPC 控制器层固定路径对比实验**。

当前目标不是维护历史路径后处理 / OSCRS 规划层方案，而是支撑下面这条可复现实物实验链：

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

## 包内方法谱系

### 1. 主线：SloshPriorityMPC

`SloshPriorityMPC` 是本文当前主方法。它是**控制器层方法**，不生成路径、不选择路径、不做规划层 candidate selection。

```text
输入: fixed/global path reference
核心: 8 维增广 MPC state + slosh prediction model + slosh-priority cost
输出: /cmd_vel
评价: RGB max(left, center, right) 为实物液面主指标
```

论文和实验里对应工程组：

```text
F = SloshPriorityMPC / ours
```

如果论文为了叙事把最终方法写成 `D` 或其他符号，代码和 rosbag 里仍以 `experiment_group:=F` 为准。

### 2. 内部 MPC 消融分支

这些分支都使用同一个 MPC solver、同一个 PathHandler、同一个 terminal 逻辑、同一个 fixed path。差别只在 cost 权重和控制平滑权重。

| group | 名称 | 含义 | 路径几何 | `v_ref(s)` | `Q_slosh` |
|---|---|---|---|---|---|
| C | ordinary MPC / nominal MPC | 普通 tracking MPC，不惩罚液体模态状态 | 固定 | 内部生成 | 0 |
| D | slosh-only MPC | 只加入 modal slosh state cost，不额外强化 smooth shaping | 固定 | 内部生成 | >0 |
| E | smooth-only MPC | 不用 slosh cost，只强化控制平滑 / 激励整形 | 固定 | 内部生成 | 0 |
| F | SloshPriorityMPC | modal slosh cost + smooth shaping，当前主方法 | 固定 | 内部生成 | >0 |

这四组是最干净的 controller-layer 因果消融：

```text
D vs C: modal slosh cost 是否有用
E vs C: 非晃液平滑控制是否有用
F vs D: smooth shaping 是否补强 slosh-only
F vs E: slosh-aware 是否超过 smooth-only
```

### 3. 包内速度调节分支

| group | 名称 | 含义 | 路径几何 | `v_ref(s)` |
|---|---|---|---|---|
| RPP_STYLE | RPP-style regulated-speed baseline | Macenski RPP 启发的曲率 / approach 速度调节，不是完整 Nav2 RPP controller | 固定 | 内部 `v_ref(s)` + regulator |

这个分支仍在 `scout_local_planner` 内，目的是复用同一 MPC 后端和同一录包链路，降低 baseline 混杂。论文中应写成：

```text
RPP-inspired regulated-speed baseline
```

不要写成完整复现 Nav2 RPP controller。

### 4. 外部 profile / 开环整形分支

这些分支不改路径几何，但会通过 CSV 改变同一条路径上的 `v_ref(s)`。它们用于比较“开环 retiming / shaping”与闭环 SloshPriorityMPC。

| group | 名称 | 含义 | 路径几何 | `v_ref(s)` |
|---|---|---|---|---|
| TOPPRA | TOPPRA-style | 限速度 / 加速度的 fixed-path retiming baseline | 固定 | 外部 CSV |
| RUCKIG | Ruckig-style | 限速度 / 加速度 / jerk 的 fixed-path retiming baseline | 固定 | 外部 CSV |
| BIAGIOTTI | Biagiotti-style | slosh-aware open-loop shaping baseline，按液体模态频率整形参考速度 | 固定 | 外部 CSV |
| custom_csv | custom external profile | 调试入口，不属于正式实验组 | 固定 | 外部 CSV |

外部 CSV 统一走：

```text
external_speed_profile_csv
external_profile_mode
ProfileExecutionCap
```

其中 TOPPRA / Ruckig 是 generic smooth-motion baseline；Biagiotti 是 open-loop slosh-aware shaping baseline。

### 5. `experiment_group` 切换关系

正式实验只应显式传 `experiment_group`，由代码派生 `controller_variant` 和 `external_profile_mode`。

| group | controller_variant | external_profile_mode | 是否改路径 | 是否改 `v_ref(s)` | 定位 |
|---|---|---|---:|---:|---|
| LEGACY | 手动参数 | 手动参数 | 否 | 视参数而定 | 兼容旧调试入口 |
| C | `mpc` | `none` | 否 | 否 | ordinary MPC |
| D | `mpc` | `none` | 否 | 否 | slosh-only |
| E | `mpc` | `none` | 否 | 否 | smooth-only |
| F | `mpc` | `none` | 否 | 否 | SloshPriorityMPC |
| RPP_STYLE | `rpp_speed_reg` | `none` | 否 | 内部调节 | RPP-style speed regulator |
| TOPPRA | `mpc` | `toppra` | 否 | 是 | open-loop retiming |
| RUCKIG | `mpc` | `ruckig` | 否 | 是 | open-loop jerk-limited retiming |
| BIAGIOTTI | `mpc` | `biagiotti` | 否 | 是 | open-loop slosh-aware shaping |

### 6. 不在本包内实现的对比方向

这些方向可以作为论文 related work / appendix / 下一阶段，但不属于当前 `scout_local_planner` 主线：

| 方法 | 当前定位 |
|---|---|
| Kanayama controller | 计划作为独立 `tracking_baselines` 包实现，不塞进 MPC 包 |
| CLF-QP tracking controller | 计划作为独立 `tracking_baselines` 包实现，不塞进 MPC 包 |
| TEB / DWA | related work 或 appendix P2P 工程泛化，不进 fixed-path 主因果表 |
| TOPPRA / Ruckig 完整库复现 | 当前只做 `*-style` CSV profile，不声称完整复现 |
| OSCRS / GeoRef / homotopy candidate selection | 规划层下一篇方向，不属于当前控制器层实验 |

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

---

## 调参与排障

> 自 control/说明.md 迁入(2026-06-03)。实物以 `config/mpc_params.yaml` 为准。

### 先确认链路，再调参数

- `/scout/global_path` 有持续输出，且 `header.frame_id=map`
- TF `map -> odom -> base_link` 连通且稳定
- `/odom` 的 `linear.x` 和 `angular.z` 数值正常、方向正确
- `control_rate` 与 `mpc/dt` 一致（默认 20 Hz 与 0.05 s）
- `/cmd_vel` 没有被其他节点抢占

### 主要参数分组

#### 状态与参考

| 参数 | 作用 | 说明 |
|------|------|------|
| `Q_contour` / `Q_ec` | 横向贴路径强度 | 默认 `use_contour_lag=true`，优先看 `Q_contour` |
| `Q_lag` / `Q_el` | 纵向误差强度 | 影响沿路径推进误差 |
| `Q_etheta` | 航向误差权重 | 过大易转向过激 |
| `Q_v` | 速度跟踪权重 | 影响跟随 `v_ref` 的积极程度 |
| `enable_omega_ff` / `Q_omega_ff` | 曲率前馈 | `omega_ref = v_ref * kappa` 提前给转向趋势 |

#### 控制平滑与约束

| 参数 | 作用 | 说明 |
|------|------|------|
| `R_omega` | 角速度惩罚 | 增大后转向更保守 |
| `R_domega` | 角速度变化率惩罚 | 最关键的平滑参数 |
| `R_a` / `R_da` | 加速度 / 其变化率惩罚 | 增大后加减速更柔和 |
| `vehicle/omega_max` / `alpha_max` | 角速度上限 / 变化率硬约束 | 响应能力上界 |
| `vehicle/v_max` / `a_max` / `j_max` | 纵向响应能力 | 过大可能打滑或冲击更强 |

#### 路径与速度曲线

| 参数 | 作用 | 说明 |
|------|------|------|
| `path_handler/lookahead_distance` | 前视距离 | 越大越平滑，越小越贴路径 |
| `path_handler/max_lat_accel` | 曲率限速 | 限制弯道速度 |
| `path_handler/max_tan_accel` / `max_tan_decel` | 切向加/减速上限 | 影响提速与急弯/终点减速 |
| `path_handler/goal_capture_distance` / `goal_capture_min_speed` | 终点捕获区 / 最低参考速度 | 避免末端过早停死 |

### 推荐调参顺序

1. 先做基础跟踪基线（`test_mpc.launch`，或 `slosh_experiment.launch` 但 `Q_slosh:=0`、关盒约束与 governor）。
2. 先调约束和路径层（`vehicle/*`、`path_handler/*`），再调权重（`Q_*`、`R_*`）。
3. 每次只改 1~2 个参数，单次幅度 10%~30%；不要同时改 `lookahead_distance`、`Q_contour`、`R_domega`。
4. 基础跟踪稳了再开 anti-slosh：先 `Q_slosh:=5` → 再 `enable_slosh_box_constraint:=true` → 最后 `slosh_speed_governor_enable:=true`。

### 观察话题

```bash
rostopic echo /mpc_status        # 状态机
rostopic echo /mpc/status_val    # 求解成功标志(1/0)
rostopic echo /mpc/solve_ms      # 求解耗时
rostopic echo /cmd_vel
# anti-slosh 实验再看:
rostopic echo /slosh/height /slosh/height_pred_max /slosh/speed_governor_active /slosh/constraint_active
```

### 常见现象与优先调整

| 现象 | 优先看 | 方向 |
|------|--------|------|
| `cmd_vel.angular.z` 锯齿、不顺滑 | `R_domega` → `R_omega` → `lookahead_distance` → `alpha_max`/`omega_max` | 先增 `R_domega`，再增 `R_omega`，必要时增前视或降响应上限 |
| 直道蛇行、摆头 | `R_omega`、`R_domega`、`lookahead_distance`、`Q_contour`、`Q_etheta` | 先压过激转向，再降路径贴合激进度 |
| 弯道跟不上 / 切弯 / 贴墙 | `Q_contour`、`Q_etheta`、`lookahead_distance`、`max_lat_accel`、`omega_max` | 先提横向/航向跟踪，再看前视/响应能力 |
| 终点附近提前停 | `goal_capture_distance`、`goal_capture_min_speed`、`goal_tolerance`、`yaw_tolerance` | 先调终点捕获区，再放宽到点判据 |
| `status_val` 频繁 0 | `vehicle/*`、`path_handler/max_*`、anti-slosh 附加机制 | 先降激进度，必要时先关 anti-slosh |
