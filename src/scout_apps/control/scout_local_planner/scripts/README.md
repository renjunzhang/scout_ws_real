# scout_local_planner scripts

当前脚本目录只服务 SloshPriorityMPC 主线和固定路径 baseline 对比实验：

```text
固定 P2_s_curve / template path
  -> C/D/E/F 内部 MPC cost 消融
  -> TOPPRA-style / Ruckig-style external v_ref(s) baseline
  -> rosbag record
  -> RGB 真值、/slosh/height 模型量、terminal 诊断、cost breakdown 分析
```

OSCRS / GeoRef 在线 path post-processor 已从主线移除。历史结果需要复查时请切到删除前的 git commit 或安全备份。

## 实物启动脚本

| 脚本 | 作用 |
|---|---|
| `launch_real_sensors_stack.sh` | 实物传感器和底盘启动入口，包含 CAN、nanoscan3、Cartographer 纯定位、IMU、RealSense |
| `record_slosh_experiment.sh` | 实物/仿真通用 rosbag 录制脚本，默认全量录制，白名单模式覆盖 RGB、MPC、reference、terminal、profile_cap、/slosh 等主线话题 |
| `send_fixed_goal.py` | 发布固定目标点 |

实物流程以 `docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md` 为准。

## 固定路径与仿真脚本

| 脚本 | 作用 |
|---|---|
| `template_fixed_path_generator.py` | 从当前位姿和目标点生成模板路径，如 `s_curve`、`mixed`、`straight` |
| `fixed_global_path_runner.py` | 固定路径 capture/replay，向 `/scout/global_path_fixed` 或指定 topic 发布 `nav_msgs/Path` |
| `run_sim_fixed_path_bag.sh` | 仿真固定路径 smoke + 自动录包，支持内部 MPC 消融和 external speed profile baseline |
| `launch_sim_nav_stack.sh` | 仿真导航栈启动入口 |
| `launch_fixed_path_slosh_stack.sh` | 历史一键固定路径入口，实物主流程优先使用上面的分步脚本 |

### 仿真 fixed-path smoke 示例

```bash
source /home/a/scout_ws/devel/setup.bash

PATH_MODE=template_goal \
PATH_ID=P2_s_curve \
TEMPLATE_NAME=s_curve \
CONDITION=CUSTOM \
RUN_ID=internal01 \
START_DELAY=2 \
RECORD_DURATION=0 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

### TOPPRA-style baseline smoke

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

### Ruckig-style baseline smoke

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

| 脚本 | 作用 |
|---|---|
| `analysis/retime_toppra_style.py` | 对固定路径生成 acceleration-limited `v_ref(s)` CSV |
| `analysis/retime_ruckig_style.py` | 对固定路径生成 jerk-limited `v_ref(s)` CSV；无 ruckig Python 包时使用内置 fallback |

CSV 接口约定：

```text
s_normalized,v_ref_m_s
0.000,0.000
...
1.000,0.000
```

PathHandler 通过 `external_speed_profile_csv` 读取该 CSV，并在 reference horizon 中覆盖内部 `v_ref`。

## 主线分析脚本

| 脚本 | 作用 |
|---|---|
| `analysis/analyze_fixed_path_cost_effect.py` | 固定路径 C/D/E/F/G 等组别的主窗口统计和图表 |
| `analysis/analyze_ferrari_indices.py` | Ferrari-style 指标、`gamma_opt`、模型/RGB 对照表 |
| `analysis/analyze_slosh_peak_context.py` | 自动化 slosh peak 回放诊断，区分 `k=0` 当前峰和 horizon 未来峰 |
| `analysis/analyze_terminal_approach_1s.py` | terminal 前 1 秒专项诊断 |
| `analysis/analyze_terminal_transition.py` | terminal/capture/reached 过渡诊断 |
| `analysis/diagnose_terminal_overshoot.py` | 终点过冲原因诊断 |
| `analysis/extract_mpc_cost_breakdown.py` | 从 bag 提取 MPC cost contribution |
| `analysis/diagnose_speed_profile.py` | 参考速度、cmd、odom 速度链路诊断 |
| `analysis/simulate_slosh_ode.py` | 二阶 slosh ODE 输入输出示意和离线仿真 |

## 保留的历史模型保真度脚本

这些脚本主要用于 2026-04/05 历史数据复查，不进入当前 fixed-path baseline 主流程：

```text
analysis/model_truth_20260513_fidelity.py
analysis/red_group_0424_ferrari_fidelity.py
analysis/red_group_0424_model_fidelity_report.py
analysis/red_group_0424_model_truth_summary.py
analysis/analyze_zeta_fidelity_ablation.py
```

后续若继续做清理，应先确认历史报告不再需要重跑，再删除这些脚本。
