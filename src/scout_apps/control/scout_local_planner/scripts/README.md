# scout_local_planner/scripts

更新时间：2026-05-26

本目录当前服务 **SloshPriorityMPC 固定路径实物/仿真对比实验**。

主流程：

```text
启动实物传感器/底盘/定位
  -> 生成或回放固定 P2_s_curve
  -> 启动 slosh_experiment.launch
  -> record_slosh_experiment.sh 录包
  -> RGB / model / cost / terminal 分析
```

完整 SOP：

```text
docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md
```

## 实物脚本

| 脚本 | 当前用途 |
|---|---|
| `launch_real_sensors_stack.sh` | 实物启动入口：CAN、底盘、nanoscan3、Cartographer 纯定位、IMU、RealSense |
| `record_slosh_experiment.sh` | rosbag 录制入口；覆盖 RGB、IMU、odom、cmd、path、MPC、reference、terminal、profile_cap、slosh 话题 |
| `send_fixed_goal.py` | 发布目标点；可用于 MBF goal 或模板路径 goal |

实物启动：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

录包：

```bash
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 5 P2_s_curve_F_run01
```

## 固定路径脚本

| 脚本 | 当前用途 |
|---|---|
| `template_fixed_path_generator.py` | 根据当前位姿和目标点生成模板路径，如 `s_curve`、`straight`、`mixed` |
| `fixed_global_path_runner.py` | 固定路径 JSON capture/replay，发布 `/scout/global_path_fixed` |
| `run_sim_fixed_path_bag.sh` | 仿真固定路径 trial + 自动录包；支持 internal / TOPPRA / Ruckig smoke |
| `launch_sim_nav_stack.sh` | 仿真导航栈启动入口 |
| `launch_fixed_path_slosh_stack.sh` | 可选历史一键入口；当前实物 SOP 默认使用分终端手动流程 |

当前实物对比实验推荐：

```text
每包前回到同一物理起点和朝向；
用同一个 goal；
用同一套 template_fixed_path_generator.py 参数生成 /scout/global_path_fixed；
再启动 MPC 和录包。
```

## 仿真 smoke 示例

先启动仿真导航栈：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash

SIM_ENV=open USE_RVIZ=true \
SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
rosrun scout_local_planner launch_sim_nav_stack.sh
```

内部速度剖面 smoke：

```bash
PATH_MODE=template_goal \
PATH_ID=P2_s_curve \
TEMPLATE_NAME=s_curve \
CONDITION=CUSTOM \
RUN_ID=internal01 \
START_DELAY=2 \
RECORD_DURATION=0 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

TOPPRA-style smoke：

```bash
PATH_MODE=template_goal \
PATH_ID=P2_s_curve \
TEMPLATE_NAME=s_curve \
CONDITION=CUSTOM \
RETIME_METHOD=toppra \
EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE=true \
RUN_ID=toppra01 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

Ruckig-style smoke：

```bash
PATH_MODE=template_goal \
PATH_ID=P2_s_curve \
TEMPLATE_NAME=s_curve \
CONDITION=CUSTOM \
RETIME_METHOD=ruckig \
EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE=true \
RUN_ID=ruckig01 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

## Retiming baseline

| 脚本 | 当前用途 |
|---|---|
| `analysis/retime_toppra_style.py` | 生成 acceleration-limited `v_ref(s)` CSV |
| `analysis/retime_ruckig_style.py` | 生成 jerk-limited `v_ref(s)` CSV；需要 Python `ruckig` 包 |

CSV 接口：

```text
s_normalized,v_ref_m_s
0.000,0.000
...
1.000,0.000
```

`PathHandler` 只读取：

```text
s_normalized
v_ref_m_s
```

其他列只用于离线画图和检查。

Ruckig 依赖说明：

```text
ROS Noetic 常见 Python 是 3.8；
最新版 ruckig 可能要求 Python >= 3.9；
实物机建议先尝试固定旧 wheel。
```

```bash
python3 -m pip install --user --only-binary=:all: 'ruckig==0.9.2'
```

如果安装失败，不要用手写近似曲线冒充 Ruckig-style baseline；可以先跳过 Ruckig，只跑 TOPPRA-style / E / F。

## 主线分析脚本

| 脚本 | 当前用途 |
|---|---|
| `extract_slosh_metrics.py` | 从 bag 提取 `/slosh/*`、状态、阶段指标 |
| `analysis/extract_mpc_cost_breakdown.py` | 提取 `/mpc/cost_breakdown` |
| `analysis/analyze_fixed_path_cost_effect.py` | 固定路径组间效果分析和图表 |
| `analysis/analyze_ferrari_indices.py` | Ferrari-style 指标、`gamma_opt`、RGB/model 对照 |
| `analysis/analyze_slosh_peak_context.py` | slosh peak 上下文诊断，区分当前峰和 horizon 未来峰 |
| `analysis/analyze_terminal_approach_1s.py` | terminal 前 1 秒专项诊断 |
| `analysis/analyze_terminal_transition.py` | terminal/capture/reached 过渡诊断 |
| `analysis/diagnose_terminal_overshoot.py` | 终点过冲原因诊断 |
| `analysis/diagnose_speed_profile.py` | `v_ref -> cmd_v -> odom_v` 速度链路诊断 |
| `analysis/simulate_slosh_ode.py` | 二阶 slosh ODE 输入输出仿真 |

## 录包白名单要求

`record_slosh_experiment.sh` 当前必须覆盖：

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

## 历史目录

当前脚本目录里可能仍有历史目录或历史分析脚本，例如：

```text
oscrs/
reference_generation/
analysis/red_group_0424_*.py
analysis/model_truth_20260513_fidelity.py
```

这些不属于当前 fixed-path baseline SOP。需要复查旧数据时可以使用；当前实物对比实验不要从这些目录启动流程。
