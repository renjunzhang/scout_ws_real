# scout_local_planner 当前主线说明

更新时间：2026-05-26

本包当前只维护 **SloshPriorityMPC 固定路径对比实验主线**。旧的 OSCRS / GeoRef、risk scheduler、input shaping、speed governor、旧内置低激励速度剖面等在线分支已经从主线移除。历史复盘请切到对应 git commit 或安全备份。

## 当前目标

保留一条清晰、可验证的实物实验链路：

```text
固定 P2_s_curve 路径
  -> PathHandler 生成参考序列
  -> 可选 external_speed_profile_csv 注入 TOPPRA/Ruckig-style v_ref(s)
  -> 8 维增广 tracking MPC
  -> terminal envelope / profile cap / cmd_vel
  -> rosbag + RGB 真值 + model 辅助指标
```

当前论文主实验只关注：

- C：普通 tracking MPC；
- D：modal slosh cost only；
- E：non-slosh smooth MPC；
- F：SloshPriorityMPC；
- TOPPRA-style：外部限加速度 `v_ref(s)` baseline；
- Ruckig-style：外部限 jerk `v_ref(s)` baseline。

## 关键文件

| 文件 | 职责 |
|---|---|
| `src/local_planner_ros.cpp` / `include/scout_local_planner/local_planner_ros.h` | ROS 节点主逻辑、状态机、terminal、profile cap、slosh observer、诊断 topic |
| `src/mpc_solver.cpp` / `include/scout_local_planner/mpc_solver.h` | OSQP QP 构建、求解、warm start、解提取 |
| `src/cost_function.cpp` / `include/scout_local_planner/cost_function.h` | tracking、速度、控制、控制变化率、slosh eta/eta_dot、terminal cost |
| `src/diff_drive_model.cpp` / `include/scout_local_planner/diff_drive_model.h` | 8 维增广状态离散动力学，含 slosh 二阶模态 |
| `src/constraint_manager.cpp` / `include/scout_local_planner/constraint_manager.h` | 速度、加速度、角速度、控制变化率、可选 eta box 约束 |
| `src/path_handler.cpp` / `include/scout_local_planner/path_handler.h` | 路径清洗/平滑、参考点生成、默认速度剖面、外部速度 CSV 注入 |
| `src/slosh_integration.cpp` / `include/scout_local_planner/slosh_integration.h` | 运行时 slosh 状态传播与 `/slosh/height` 估计 |
| `config/mpc_params.yaml` | 实物默认参数 |
| `config/mpc_params_sim.yaml` | 仿真默认参数 |
| `launch/slosh_experiment.launch` | 实物实验入口 |
| `launch/slosh_experiment_sim.launch` | 仿真实验入口 |
| `scripts/run_sim_fixed_path_bag.sh` | 仿真固定路径录包；支持 internal / TOPPRA / Ruckig smoke |
| `scripts/record_slosh_experiment.sh` | 实物录包 |
| `scripts/template_fixed_path_generator.py` | 从当前位姿到指定终点生成模板固定路径 |
| `scripts/fixed_global_path_runner.py` | 固定路径采集/回放 |
| `scripts/analysis/retime_toppra_style.py` | TOPPRA-style 离线速度剖面生成 |
| `scripts/analysis/retime_ruckig_style.py` | Ruckig-style 离线速度剖面生成 |

## MPC 数学结构

状态为 8 维：

```text
x = [e_l, e_c, e_theta, v, eta_x, eta_dot_x, eta_y, eta_dot_y]
```

控制为 2 维：

```text
u = [a, omega]
```

单步代价：

```text
Q_lag      * e_l^2
+ Q_contour * e_c^2
+ Q_etheta  * e_theta^2
+ Q_v       * (v - v_ref)^2
+ R_a       * a^2
+ R_omega   * omega^2
+ R_da      * (a - a_prev)^2
+ R_domega  * (omega - omega_prev)^2
+ Q_slosh   * h_coeff^2 * (eta_x^2 + eta_y^2)
+ Q_slosh_eta_dot * (eta_dot_x^2 + eta_dot_y^2)
```

`eta / eta_dot` 是二阶 ODE 状态递推结果，不是瞬时加速度函数：

```text
x_slosh,k+1 = A_d x_slosh,k + B_d [a_x, a_y]
```

所以 slosh cost 能表达历史激励留下来的残余晃动；前提是 observer/MPC 初始 slosh state 不被每轮清零。

## 速度参考

默认路径速度来自 `PathHandler::updateSpeedProfile()`：

- 基于路径曲率和 `max_lat_accel`、`speed_profile_omega_max`、`speed_profile_alpha_max` 生成 `v_ref(s)`；
- 再用 `max_tan_accel / max_tan_decel` 做前后向速度包络；
- `PathHandler::getReferencePoints()` 按当前路径进度采样 `v_ref`。

外部 baseline 使用：

```text
external_speed_profile_csv
```

CSV 约定：

```text
s_normalized,v_ref_m_s
0.000,0.000
...
1.000,0.000
```

当 CSV 有效时，外部 `v_ref(s)` 替代默认速度剖面；`external_profile_execution_cap_enable` 可在发布 `cmd_vel` 前用同一 profile 做执行层速度 hard cap。

## Terminal 口径

当前 terminal 不进入 slosh cost 主效果统计。

主论文窗口：

```text
TRACKING_PRE_TERMINAL = TRACKING -> first terminal/capture - 1s
```

terminal 单独作为工程诊断窗口：

```text
terminal_approach_1s = first terminal/capture - 1s -> first terminal/capture
```

原因：

- terminal 阶段有独立 envelope、capture、REACHED、cmd_v 限幅逻辑；
- terminal jerk/ax 脉冲会污染 slosh cost 主效果归因；
- C/D/E/F 和 TOPPRA/Ruckig 的主结论必须在正常跟踪窗口比较。

## 保留和删除边界

保留：

- slosh modal cost；
- smooth MPC cost；
- terminal envelope / capture stop；
- external speed profile CSV；
- profile execution cap；
- cost breakdown 与 reference/terminal/profile 诊断 topic；
- RGB 真值分析与 Ferrari-style 指标脚本。

不再维护：

- OSCRS / GeoRef 在线 path post-processor；
- risk scheduler；
- input shaping / ISR；
- speed governor；
- 旧内置低激励 profile；
- 其他历史条件入口。

## 实验入口

实物传感器：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

固定路径 / cost 对比实验请以：

```text
docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md
```

为准。该文档是当前实物验证 SOP。

## 判断原则

- RGB 视觉液面是主指标；
- `/slosh/height` 是模型辅助指标，不能自证真实防晃效果；
- C/D/E/F 内部消融不改几何路径；
- TOPPRA/Ruckig-style 可以改变 `v_ref(s)`，这是外部 baseline 的实验自变量；
- terminal 段只做诊断，不混入主效果统计；
- completion time 差异超过 10% 时，论文应按 trade-off 解释，不能写成纯 anti-slosh 优势。
