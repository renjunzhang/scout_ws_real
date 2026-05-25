# `scripts` 目录说明

本目录存放 `scout_local_planner` 相关的实验辅助脚本，主要用于固定目标/固定路径实验、录包、离线指标提取，以及 IMU 标定验证。

目录整理后的原则：

```text
scripts/                       # 现场高频入口和 rosrun 入口
scripts/oscrs/                 # OSCRS 在线选择器核心：G/F/R/S/diagnostics
scripts/reference_generation/  # 路径/轨迹参考生成库
scripts/analysis/              # 低频离线分析、历史诊断和论文指标脚本
```

如果旧文档中写的是 `scripts/<name>.py`，而本 README 标注为 `analysis/<name>.py`，以后以本 README 为准。

## 当前脚本路径总表

### 根目录入口脚本

这些脚本保留在 `scripts/` 根目录，优先作为现场运行、仿真、录包、验收和高频分析入口。

| 路径 | 作用 |
|---|---|
| `scripts/anti_slosh_path_post_processor.py` | Online GeoRef / OSCRS path post-processor ROS 节点 |
| `scripts/launch_sim_nav_stack.sh` | 启动仿真世界、定位、MBF 全局规划器；不启动 local planner，不录 bag |
| `scripts/run_sim_fixed_path_bag.sh` | 单次仿真 trial + local planner + 路径/目标发布 + rosbag 录制 |
| `scripts/record_slosh_experiment.sh` | 实物/仿真通用正式录包脚本 |
| `scripts/record_slosh_debug.sh` | 实物轻量调试录包脚本 |
| `scripts/send_fixed_goal.py` | 向 `/scout/goal` 重复发布同一个 PoseStamped |
| `scripts/fixed_global_path_runner.py` | 固定 `/scout/global_path` 的采集与 JSON replay |
| `scripts/template_fixed_path_generator.py` | 从当前位姿到 goal 生成模板固定路径 |
| `scripts/validate_georef_oscrs_bag.py` | 单包行为验收：RAW / GeoRef / fixed / OSCRS 是否按预期接管 |
| `scripts/check_oscrs_takeover.py` | 汇总 OSCRS active/takeover/fallback/fb 分布 |
| `scripts/check_oscrs_model_consistency.py` | 检查 OSCRS rollout 与 `/slosh/height` 的一致性 |
| `scripts/extract_slosh_metrics.py` | 主指标提取：height / eta_dot / energy / tracking / solver |
| `scripts/analyze_oscrs_candidates.py` | 离线复算 OSCRS 候选 feasible / score / fallback |
| `scripts/analyze_slosh_peak_precursors.py` | 峰值前激励窗口分析 |
| `scripts/analyze_day3_abc_smoke.py` | Day3 A/B/C smoke 对比 |
| `scripts/imu_ay_tool.py` | IMU 横向加速度动作、分析、标定 |
| `scripts/run_imu_stage4_sequence.py` | IMU alpha_z 原地旋转动作 |
| `scripts/launch_fixed_path_slosh_stack.sh` | 实物固定路径 slosh 链路一键启动 |
| `scripts/run_day4_profile.sh` | Day4 profile 参数组 wrapper |
| `scripts/diagnose_georef_budget_gap.py` | GeoRef budget / execution gap 诊断 |
| `scripts/diagnose_reference_execution_chain.py` | reference 执行链路诊断 |
| `scripts/diagnose_slosh_guided_georef_score.py` | slosh-guided GeoRef score 诊断 |
| `scripts/evaluate_anti_slosh_path_candidates.py` | 离线候选路径评价 |
| `scripts/optimize_anti_slosh_reference.py` | 低激励参考优化历史入口 |
| `scripts/sweep_anti_slosh_timing_candidates.py` | timing candidate sweep |
| `scripts/sweep_p3_geometry_candidates.py` | P3 几何 candidate sweep |
| `scripts/test_candidate_generators_equivalence.py` | GeoRef 候选生成等价性测试 |
| `scripts/candidate_generators.py` | 旧 import 兼容 shim，转到 `reference_generation/candidate_generators.py` |
| `scripts/generate_anti_slosh_path_candidates.py` | 旧 CLI/import 兼容 shim，转到 `reference_generation/geometry_candidates.py` |

### OSCRS 核心模块

这些不是直接运行入口，主要被 `anti_slosh_path_post_processor.py` 调用。

| 路径 | 作用 |
|---|---|
| `scripts/oscrs/pipeline.py` | G→F→R→S 纯编排器 |
| `scripts/oscrs/feasibility.py` | F 层：几何 / collision / ay gate |
| `scripts/oscrs/slosh_rollout.py` | R 层：速度 rollout + 二阶 slosh ODE 指标 |
| `scripts/oscrs/selector.py` | S 层：hard gate / score / OSCRS selection / fixed candidate |
| `scripts/oscrs/diagnostics.py` | candidate_report / safety_alarm / metrics 格式化 |
| `scripts/oscrs/path_utils.py` | 纯几何小工具 |
| `scripts/oscrs/types.py` | dataclass 接口文档和后续类型迁移锚点 |
| `scripts/oscrs/generators/georef.py` | 当前 G 实例：GeoRef candidate wrapper |

### 路径/轨迹参考生成库

| 路径 | 作用 |
|---|---|
| `scripts/reference_generation/geometry_candidates.py` | GeoRef 平滑、路径采样、曲率/几何指标、离线 candidate 生成 |
| `scripts/reference_generation/candidate_generators.py` | 在线 G 层候选生成接口，当前实例为 GeoRef smoothing |

### 低频离线分析与历史诊断

| 路径 | 作用 |
|---|---|
| `scripts/analysis/compute_modal_params.py` | 根据容器/液位/液体参数重算模态派生量 |
| `scripts/analysis/ferrari_oracle.py` | Ferrari assigned-path time-optimal oracle 离线工具 |
| `scripts/analysis/compute_ferrari_indices.py` | Ferrari 风格模型/视觉/优化指标 |
| `scripts/analysis/retime_toppra_style.py` | 固定路径 TOPPRA-style 限加速度速度重定时，输出 `v_ref(s)` CSV |
| `scripts/analysis/retime_ruckig_style.py` | 固定路径 Ruckig-style 限 jerk 速度重定时，输出 `v_ref(s)` CSV |
| `scripts/analysis/extract_visual_height.py` | RealSense 侧视视觉液面高度骨架 |
| `scripts/analysis/check_time_sync.py` | `/slosh/height` 与图像时间戳同步检查 |
| `scripts/analysis/summarize_oscrs_step2.py` | OSCRS Step 2 PASS / SATURATED / FAIL 汇总 |
| `scripts/analysis/offline_pmg_replay.py` | PMG 历史离线 replay |
| `scripts/analysis/diagnose_p3_failure_modes.py` | P3 failure mode 历史诊断 |
| `scripts/analysis/diagnose_real_tuning.py` | 实物 bag 调参诊断 |
| `scripts/analysis/diagnose_speed_profile.py` | 速度剖面限速来源诊断 |
| `scripts/analysis/analyze_path_geometry_slosh_triggers.py` | 路径几何与 slosh 触发关系分析 |
| `scripts/analysis/analyze_global_path_duplicates.py` | 全局路径重复点/极短段检查 |
| `scripts/analysis/analyze_global_path_prefix_window.py` | 全局路径前缀窗口几何检查 |
| `scripts/analysis/analyze_tracking_infeasible.py` | TRACKING 阶段 infeasible 根因分析 |
| `scripts/analysis/analyze_sim_speed_issue.py` | 仿真速度偏慢分析 |
| `scripts/analysis/analyze_settling_day3.py` | Day3 settling 验证 |
| `scripts/analysis/observe_terminal_recovery.py` | terminal recovery 在线/回放观察 |
| `scripts/analysis/sim_fixed_goal_tool.py` | 仿真固定 goal capture/show/replay |
| `scripts/analysis/trajectory_analysis.py` | 实物/仿真执行轨迹几何分析 |
| `scripts/analysis/mpc_bug_analysis.py` | MPC 历史 bug 分析 |
| `scripts/analysis/validate_sim_imu.py` | 仿真 IMU 话题在线验收 |

## 分类索引

当前主线是：

```text
MBF global path
  -> Online GeoRef / OSCRS path post-processor
  -> normal MPC tracking
  -> bag / offline analysis
```

优先看下面几类。

### A. Online GeoRef / OSCRS 在线主线

这些文件直接参与当前 `RAW_REAL / GEOREF_TUNED_STRONG_REAL / GEOREF_OSCRS_ACTIVE_REAL` 主线。

```text
anti_slosh_path_post_processor.py
  在线 path post-processor。
  GEOREF_TUNED: geometry-only candidate selection。
  GEOREF_OSCRS_ACTIVE: 同一候选集 + OSCRS hard gate + score selection。
  发布 /scout/global_path_anti_slosh、candidate_report、debug paths、safety_alarm。

check_oscrs_takeover.py
  检查 bag 中 OSCRS 是否 active、是否 takeover、fallback 分布。
  用于区分"OSCRS 运行了"和"OSCRS 实际改变了参考路径"。

validate_georef_oscrs_bag.py
  单包行为验收脚本。检查 RAW / GEOREF_TUNED / GEOREF_FIXED_STRONG /
  GEOREF_OSCRS_ACTIVE 是否走了预期路径，汇总 selected/fb/takeover、
  safety_alarm、/scout/global_path_anti_slosh 和基础 h/eta_dot 指标。
  takeover smoke 时使用 --require-non-original 或 --require-takeover。

analyze_oscrs_candidates.py
  离线复算候选路径的 OSCRS 指标，输出每个 candidate 的 feasible / score / fallback 依据。

analysis/summarize_oscrs_step2.py
  汇总 OSCRS Step 2 判据，生成 PASS / SATURATED / FAIL 结论。

check_oscrs_model_consistency.py
  检查 OSCRS 预测高度与 bag 中 /slosh/height 的一致性。

analysis/compute_modal_params.py
  换容器/液位/液体后，重算 omega_n / height_coeff / Ferrari zeta，
  并同步 config/oscrs_container.yaml。
```

配套配置不在 `scripts/` 下，但必须一起看：

```text
../config/oscrs_container.yaml
  OSCRS 容器物理量、height gate、residual gate、score 权重。

../config/scenarios.yaml
  仿真 open 场景 goal 列表；run_sim 可用 SCENARIO=<name> 读取 goal。

../launch/anti_slosh_path_post_processor.launch
  post-processor 启动参数，实物和仿真都走这里。
```

### B. 仿真/实物录包与固定 goal / 固定轨迹

```text
run_sim_fixed_path_bag.sh
  仿真单包 wrapper。支持固定轨迹 replay、固定 goal/global planner、
  template path 三种 PATH_MODE。当前最容易误用，文件头有详细说明。

record_slosh_experiment.sh
  实物/仿真通用 rosbag 录制脚本，覆盖 slosh、MPC、GeoRef/OSCRS、
  RealSense、IMU、TF、costmap 等关键话题。

send_fixed_goal.py
  向 /scout/goal 重复发布同一个 PoseStamped，适合实物同终点对比。

fixed_global_path_runner.py
  采集/回放固定 /scout/global_path JSON。
  用于"固定轨迹"而不是"固定 goal"。

template_fixed_path_generator.py
  从当前位姿到 goal 生成 straight / single_turn / s_curve / mixed 等模板路径。
  这是模板固定轨迹，不是 MBF 全局路径。
```

固定 goal 与固定轨迹的区别：

```text
固定 goal:
  固定的是 /scout/goal。
  MBF 每次根据当前地图、起点、costmap 重新生成 /scout/global_path。
  适合验证 Online GeoRef / OSCRS 是否能接在真实全局规划器后面。

固定轨迹:
  固定的是已经保存好的 path JSON。
  不经过 MBF 重新规划。
  适合重复跑 P2/P3 和旧方案消融。
```

### C. 路径/轨迹参考生成库

```text
reference_generation/geometry_candidates.py
  GeoRef 几何候选生成、曲率/路径指标、离线 candidate JSON 生成。

reference_generation/candidate_generators.py
  在线 G 层候选生成接口，当前实例为 GeoRef smoothing。

candidate_generators.py
generate_anti_slosh_path_candidates.py
  根目录兼容 shim，保留旧 import / rosrun 路径。
```

### D. Ferrari / RA-L 离线参考与视觉指标

```text
analysis/ferrari_oracle.py
  Ferrari 2026 RA-L assigned-path time-optimal oracle 的 Scout 改写。
  当前作为离线上界/参考工具，不进入在线控制链。

analysis/compute_ferrari_indices.py
  计算 Ferrari 风格指标，可接 /slosh/h_visual topic 或 RGB 离线 CSV。

analysis/extract_visual_height.py
  RealSense 侧视 + ArUco + Canny/Hough 的视觉液面高度骨架。
  当前主要作为实物 D5 视觉 GT 管线骨架。

analysis/check_time_sync.py
  检查 /slosh/height 与图像时间戳配对质量。
```

配套配置：

```text
../config/ferrari_oracle.yaml
../config/visual_height.yaml
```

### E. 指标提取与历史分析

```text
extract_slosh_metrics.py
  主指标提取脚本，统计 /slosh/height、eta_dot、energy、tracking_time、
  solve_success_ratio 等。

analyze_slosh_peak_precursors.py
analysis/analyze_path_geometry_slosh_triggers.py
diagnose_reference_execution_chain.py
diagnose_georef_budget_gap.py
diagnose_slosh_guided_georef_score.py
  GeoRef/旧方案失效分析脚本。

analysis/offline_pmg_replay.py
analysis/diagnose_p3_failure_modes.py
  PMG / P3 failure history 分析，当前不是主线控制器。
```

### F. IMU / 旧阶段诊断

```text
imu_ay_tool.py
analysis/validate_sim_imu.py
analyze_day3_abc_smoke.py
analysis/analyze_settling_day3.py
analysis/diagnose_real_tuning.py
```

这些脚本仍可用于 IMU 标定、settling 状态机和旧风险调度器分析，但不属于当前 GeoRef/OSCRS 主表。

## 脚本列表

### `send_fixed_goal.py`

向指定 ROS 话题发布一个固定的 `geometry_msgs/PoseStamped` 目标点，默认发到 `/scout/goal`。

适用场景：
- 手动触发一次固定终点跟踪
- 做对照实验时保证 goal pose 一致

### `analysis/sim_fixed_goal_tool.py`

仿真专用固定 goal 工具，支持：
- `capture`：抓取当前 `/scout/goal` 并保存为 JSON
- `show`：显示已保存 goal 的 `frame/x/y/yaw`
- `replay`：从 JSON 重发同一个 goal 到 `/scout/goal`

适用场景：
- Day4/Day5 仿真实验要求终点完全一致
- 不再依赖 RViz 手点 goal，避免人为偏差

### `fixed_global_path_runner.py`

固定 `/scout/global_path` 的采集与回放工具，支持以下模式：
- `capture`：抓取第一次收到的全局路径并保存到 JSON
- `replay`：从 JSON 读取固定路径并持续发布
- `capture_and_replay`：先抓取再回放
- `goal_only`：只发布已保存路径的起点或终点 goal

适用场景：
- 先记录第一次规划出的路径，再让 Q0/Q5 都复用同一条 `/scout/global_path`
- 正式 tracking 前，先将机器人移回固定路径起点并对齐，再开始局部跟踪

补充能力：
- `--manual-start`：实物场景下先人工摆位，按 Enter 后再进入起点门控 / replay
- `--skip-start-wait`：直接跳过起点门控，用于快速 debug

### `template_fixed_path_generator.py`

从当前机器人位姿到点击终点，自动生成标准化固定路径模板并发布到 `/scout/global_path_fixed`。默认等待 RViz `2D Nav Goal` 发布到 `/scout/goal`。

仿真 open 场景采集固定路径时，推荐加 `--start-heading current`，让路径起点朝向等于当前车头方向：
```bash
rosrun scout_local_planner template_fixed_path_generator.py \
  --template s_curve \
  --start-heading current \
  --goal-topic /scout/goal \
  --output-topic /scout/global_path_fixed \
  --path-file /data/a/fixed_paths/sim/P2_s_curve.json
```
使用该模式时，RViz 终点需要点在当前车头前方。

当前支持模板：
- `straight`
- `single_turn`
- `s_curve`
- `mixed`
- `multi_s`
- `sharp_turn`

适用场景：
- 空旷场地快速生成可重复的直线 / 单弯 / S 弯 / 连续 S 弯实验路径
- 生成后直接给 `slosh_experiment.launch global_path_topic:=/scout/global_path_fixed`
- 可选同时保存为 JSON，供后续 fixed-path replay 复用

### `launch_fixed_path_slosh_stack.sh`

固定路径 slosh 实验的一键启动脚本，按顺序启动：
- `nanoscan3_localization scout_nanoscan3_cartographer_localization.launch`
- `scout_global_planner mbf_global.launch`
- `scout_local_planner slosh_experiment.launch`

默认让 local planner 订阅 `/scout/global_path_fixed`，并保持第一轮有效性验证的固定口径：
- `enable_slosh_box_constraint=false`
- `slosh_speed_governor_enable=false`
- `filter_alpha_v=1.0`
- `filter_alpha_omega=1.0`
- `slosh_use_imu_lateral_accel=false`
- `slosh_use_imu_yaw_rate=true`

用法：
```bash
rosrun scout_local_planner launch_fixed_path_slosh_stack.sh 0
rosrun scout_local_planner launch_fixed_path_slosh_stack.sh 5
rosrun scout_local_planner launch_fixed_path_slosh_stack.sh 10
```

脚本会在启动 `nanoscan3_localization scout_nanoscan3_cartographer_localization.launch` 后暂停，等待定位准确度达到 `70%` 后再继续启动 global planner 和 local planner。当前 Cartographer launch 中没有明确的“定位准确度百分比”ROS topic，因此默认是人工确认门：看到定位准确度达到 `70%` 后按 Enter 继续。

如果后续已有可读的准确度话题，可以用环境变量启用自动等待：
```bash
LOCALIZATION_ACCURACY_TOPIC=/your/localization_accuracy_topic \
LOCALIZATION_ACCURACY_THRESHOLD=70 \
rosrun scout_local_planner launch_fixed_path_slosh_stack.sh 5
```

如果需要临时换固定路径话题：
```bash
GLOBAL_PATH_TOPIC=/scout/global_path_fixed rosrun scout_local_planner launch_fixed_path_slosh_stack.sh 5
```

如果需要跑 TOPPRA/Ruckig-style 外部速度剖面：
```bash
EXTERNAL_SPEED_PROFILE_CSV=/path/to/P2_s_curve_toppra_style.csv \
GLOBAL_PATH_TOPIC=/scout/global_path_fixed \
rosrun scout_local_planner launch_fixed_path_slosh_stack.sh 0
```

### `record_slosh_experiment.sh`

`rosbag` 录包脚本，用于记录 anti-slosh MPC 实验相关话题。

主要覆盖：
- 液面/晃动估计话题，如 `/slosh/height`、`/slosh/height_pred_max`
- 控制与状态话题，如 `/cmd_vel`、`/odom`、IMU、MPC 状态
- 路径与任务上下文，如 `/scout/goal`、`/scout/global_path`
- RealSense 液面测量和终点恢复诊断相关话题

适用场景：
- 录制 Q0/Q5 对照 bag
- 录制 IMU 标定或终点恢复行为分析 bag

### `analysis/compute_modal_params.py`

OSCRS / Ferrari 模型参数一致性工具。换容器、换液位或换液体后，用这个脚本从一手物理量重算模态派生量，并同步 `config/oscrs_container.yaml`。

输入物理量：
- `R`: 容器内半径，单位 m
- `h`: 静止液面高度，单位 m
- `rho`: 液体密度，单位 kg/m³
- `nu`: 液体动力黏度，单位 Pa·s

派生量：
- `omega_n`: 一阶模态频率，Ferrari 式(2)
- `height_coeff_observer`: 在线 observer 口径，`4*h*m_n/(m_F*R)`
- `height_coeff_ferrari`: Ferrari 闭式口径，`xi^2*h*m_n/(m_F*R)`
- `zeta_ferrari`: Ferrari 式(3) 物理阻尼比

只检查当前 `oscrs_container.yaml` 是否与物理量一致，不写文件：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/analysis/compute_modal_params.py \
  --yaml src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml
```

临时指定一组新容器 / 液体参数，只打印派生量：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/analysis/compute_modal_params.py \
  --R 0.025 \
  --h 0.070 \
  --rho 900 \
  --nu 5.0e-2
```

手工修改 `oscrs_container.yaml` 中的 `slosh.container_radius / liquid_height / liquid_density / liquid_dynamic_viscosity` 后，同步写回派生量：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/analysis/compute_modal_params.py \
  --yaml src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  --write
```

写回内容：
- `slosh_score.omega_n`
- `slosh_score.height_coeff`

注意：默认不会覆盖 `slosh.damping_ratio`。当前在线 observer 默认仍使用 manual/observer 拟合阻尼口径；Ferrari 物理阻尼只打印出来作为参考。

如果要做 Ferrari 物理阻尼 ablation，可显式写回：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/analysis/compute_modal_params.py \
  --yaml src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  --write \
  --write-zeta-ferrari
```

如需保留原文件，只导出派生量块：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/analysis/compute_modal_params.py \
  --R 0.025 \
  --h 0.070 \
  --rho 900 \
  --nu 5.0e-2 \
  --emit-yaml /data/a/slosh_bags/analysis/modal_params_25mm_70mm.yaml
```

使用纪律：
- 改容器或液位后，先运行只检查模式；
- 只有确认物理量正确后再加 `--write`；
- 不要把 `--write-zeta-ferrari` 当默认操作，除非这轮实验明确要比较 Ferrari 物理阻尼；
- 写回后用 `git diff config/oscrs_container.yaml` 检查实际改动。

### `run_sim_fixed_path_bag.sh`

路径：`scripts/run_sim_fixed_path_bag.sh`

仿真单次 trial + 录包 wrapper。它不负责启动 Gazebo / Cartographer / MBF；这些应先由 `scripts/launch_sim_nav_stack.sh` 启动。该脚本负责在已有仿真导航栈上：
- 按 `CONDITION` 启动 `slosh_experiment_sim.launch` 和可选 post-processor；
- 按 `PATH_MODE` 发布固定轨迹、固定 goal 或模板路径；
- 启动 `rosbag record`；
- 生成标准化 bag 文件名；
- `Ctrl+C` 或 `RECORD_DURATION` 到时停止本 trial 的 MPC、路径发布和录包。

和 `launch_sim_nav_stack.sh` 的分工：

```text
launch_sim_nav_stack.sh:
  启动仿真环境、定位、MBF 全局规划器。
  长时间保持运行，一次启动后可跑多包。

run_sim_fixed_path_bag.sh:
  在已经启动好的仿真环境里跑一次实验 trial。
  每执行一次生成一个 bag。
```

三种路径模式：

```text
PATH_MODE=global_goal
  发布 /scout/goal，让 MBF 生成 /scout/global_path。
  用于验证 Online GeoRef / OSCRS 接在真实全局规划器后面的链路。

PATH_MODE=replay
  读取 PATH_FILE 或 FIXED_PATH_DIR/PATH_ID.json，直接 replay 固定路径。
  不经过 MBF 重新规划。
  用于固定 P2/P3 或消融复现实验。

PATH_MODE=template_goal
  用 template_fixed_path_generator.py 从当前位姿到 goal 生成模板路径。
  不是 MBF 全局路径。
  适合“每次只给终点、以当前车位姿为起点”生成 P2_s_curve。
```

仿真启动后，跑固定 goal / MBF 全局路径 trial：

```bash
source /home/a/scout_ws/devel/setup.bash

PATH_MODE=global_goal \
PATH_ID=open_custom_goal \
CONDITION=GEOREF_OSCRS_ACTIVE \
RUN_ID=active01 \
START_DELAY=10 \
RECORD_DURATION=0 \
TEMPLATE_GOAL_X=-3.014343023300171 \
TEMPLATE_GOAL_Y=2.987114429473877 \
TEMPLATE_GOAL_QZ=0.9999403278718936 \
TEMPLATE_GOAL_QW=0.010924316704027428 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

仿真启动后，跑固定轨迹 replay trial：

```bash
source /home/a/scout_ws/devel/setup.bash

PATH_MODE=replay \
PATH_ID=P2_s_curve \
CONDITION=GEOREF_OSCRS_ACTIVE \
RUN_ID=active01 \
START_DELAY=10 \
RECORD_DURATION=0 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

从 `config/scenarios.yaml` 读取 goal 的快捷写法：

```bash
source /home/a/scout_ws/devel/setup.bash

SCENARIO=open_user_goal \
CONDITION=GEOREF_OSCRS_ACTIVE \
RUN_ID=active01 \
START_DELAY=10 \
RECORD_DURATION=0 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

常用方式：
```bash
PATH_ID=P2_s_curve CONDITION=FAS_Q5_DOT RUN_ID=01 \
START_DELAY=30 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

常用变量：
- `PATH_ID`: `P0_straight / P1_single_turn / P2_s_curve / P3_mixed`
- `CONDITION`: `NOM / FAS_Q5 / FAS_Q5_DOT / FAS_Q10 / FAS_Q5_TERM / PROP_Q5 / ISR / CUSTOM`
- `RUN_ID`: bag run 编号
- `START_DELAY`: 启动后等待秒数；默认 `30`
- `APPROACH_START_ENABLE`: 默认 `false`；固定路径应从定位刷新后的当前位姿采集
- `RECORD_DURATION`: 固定录制秒数；默认 `0` 表示手动 `Ctrl+C`
- `PATH_PUBLISH_ONCE_KEEPALIVE`: 默认 `true`，只发布一次 latched 路径并保持发布者存活，避免重复触发 `PathHandler` 新路径逻辑
- `FIXED_PATH_DIR`: 默认 `/data/a/fixed_paths/sim`
- `BAG_DIR`: 默认 `/data/a/slosh_bags/sim/YYYYMMDD`

外部 baseline 速度剖面：
- `RETIME_METHOD`: `none / toppra / ruckig`，默认 `none`
- `EXTERNAL_SPEED_PROFILE_CSV`: 直接指定已有 `v_ref(s)` CSV；非空时优先使用
- `RETIME_PROFILE_DIR`: 自动生成 CSV/PNG 的目录，默认 `/data/a/fixed_paths/sim/baseline_profiles`
- `RETIME_V_MAX / RETIME_A_MAX / RETIME_DECEL_MAX`: TOPPRA/Ruckig 共用速度、加速度、减速度约束
- `RETIME_J_MAX / RETIME_DELTA_TIME`: Ruckig-style jerk 和离散步长
- `RETIME_DS`: TOPPRA-style 输出路径间隔

从当前位姿到给定终点自动生成 P2_s_curve，并自动生成 TOPPRA-style `v_ref(s)`：

```bash
source /home/a/scout_ws/devel/setup.bash

PATH_MODE=template_goal \
PATH_ID=P2_s_curve \
CONDITION=CUSTOM \
RUN_ID=toppra_smoke01 \
RETIME_METHOD=toppra \
RETIME_V_MAX=0.80 \
RETIME_A_MAX=0.60 \
RETIME_DECEL_MAX=0.80 \
TEMPLATE_GOAL_X=3.5 \
TEMPLATE_GOAL_Y=0.0 \
TEMPLATE_GOAL_QZ=0.0 \
TEMPLATE_GOAL_QW=1.0 \
RECORD_DURATION=0 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

该流程是：

```text
当前车位姿 + TEMPLATE_GOAL
  -> template_fixed_path_generator.py 生成本次 P2_s_curve JSON
  -> retime_toppra_style.py / retime_ruckig_style.py 生成速度剖面 CSV
  -> slosh_experiment_sim.launch 通过 external_speed_profile_csv 读取 CSV
```

`PATH_MODE=template_goal` 默认把本次生成的 JSON 保存为
`/data/a/fixed_paths/sim/<bag_name>.json`，避免误读旧的 `P2_s_curve.json`。
bag 会同步记录 `/scout/global_path_fixed`、`/reference/v_ref_horizon`、
`/reference/s_horizon`、`/mpc/cost_breakdown` 和 terminal clamp 诊断话题。

注意：正式对比实验中，同一个 block 内的 TOPPRA/Ruckig/E/F 应 replay 同一条已生成的 JSON；
不要每个方法各自重新生成路径，否则路径几何不一致。

注意：
- 如果目的是验证“真实在线全局规划 + post-processor”，用 `PATH_MODE=global_goal` 或 `SCENARIO=<name>`。
- 如果目的是最干净地重复同一条路径，用 `PATH_MODE=replay`。
- 如果传了 `TEMPLATE_GOAL_X/Y/QZ/QW` 但没显式设置 `PATH_MODE`，脚本会自动切到 `PATH_MODE=global_goal`，避免误跑默认 `P2_s_curve` replay。

### `record_slosh_debug.sh`

轻量级调试录包脚本，面向实物参数整定与根因定位。

与 `record_slosh_experiment.sh` 的区别：
- 不录 RealSense 原始图像
- 不录地图、costmap、MBF 大量接口话题
- 只保留 `cmd_vel / odom / mpc_status / v_des_eff / terminal / 路径几何 / IMU / TF` 等核心诊断信号

适用场景：
- 实物 `Q0/Q5` 调参
- 配合 `diagnose_real_tuning.py` 快速判断下一轮该调哪些参数
- 需要更小 bag 体积、方便从工控机拷回开发机

### `extract_slosh_metrics.py`

从 bag 文件中提取 slosh/MPC 实验指标的离线分析脚本。

可提取内容包括：
- `/slosh/height`、`/slosh/height_pred_max` 的峰值和统计量
- `/mpc/solve_ms`、`/mpc/status_val` 的求解性能
- governor、约束触发、速度命令等运行指标
- 按 `mpc_status` 分段统计成功率（`TRACKING / SETTLING / REACHED / IDLE` 各段单独输出），避免整包 success_ratio 被 near-goal 段污染

适用场景：
- 比较不同参数组下的晃动指标和控制开销
- 判断 success_ratio 低的根因来自哪个阶段
- 批量导出 CSV 做进一步统计

### `analyze_slosh_peak_precursors.py`

离线检查液面峰值前的主要激励源，用于判断 anti-slosh 速度剖面是否触发错位。

关注内容：
- 在 `TRACKING && terminal/mode==NONE` 口径下找 `/slosh/height`、`eta_dot` 或 `modal_energy_norm` 峰值
- 统计峰值前 `0.5/1.0/1.5s` 窗口中的 `odom_ay`、`kappa`、`omega`、`domega`、`v_des_eff`
- 导出 CSV，便于比较 P2/P3 中真正的峰前激励段

用法：
```bash
rosrun scout_local_planner analyze_slosh_peak_precursors.py \
  --peak-signal height \
  --top-k 3 \
  --csv /tmp/peak_precursors.csv \
  /data/a/slosh_bags/sim/20260428/20260428_P3_mixed_PROFILE_SELECTIVE_run01_191925.bag
```

说明：`PROFILE_* / OUTPUT_GUARD / PMG_*` 属于 2026-04-27 至 2026-04-29 的失败路线，控制器入口已从主线撤回；离线脚本仍可分析这些历史 bag。

### `analysis/analyze_settling_day3.py`

Day 3 T2 settling 验证专用分析脚本，从 bag 中检查 settling 状态机的进出行为。

关注内容包括：
- `mpc_status` 是否经过 `TRACKING → SETTLING → REACHED` 完整链路
- `/slosh/settling_time` 是否正常发布及其数值
- `SETTLING` 阶段的 `status_val` 成功率
- `/risk_scheduler/u_k`、`fallback_active`、`rho_k` 在 settling 阶段的行为

适用场景：
- 验证 T2 settling 状态机能否在实物/仿真中正常进出
- 排查"SETTLING 一直没进入"或"settling_time 没发布"的问题

### `analyze_day3_abc_smoke.py`

Day 3 A/B/C 配置对照的仿真 smoke 检查脚本，对比三种 IMU 使用配置下的关键指标。

三种配置：
- A：`yaw_rate=true, lateral_accel=false`（默认）
- B：`yaw_rate=true, lateral_accel=true`
- C：`yaw_rate=true, lateral_accel=true + risk_scheduler`

关注内容：
- 各配置的 `TRACKING success rate`、`solve_ms p95`
- `/slosh/height p95`、`fallback_active true_ratio`
- IMU ay bias ready 比例、`imu_ay_filtered` 与 `ay_est` 的一致性

用法：
```bash
rosrun scout_local_planner analyze_day3_abc_smoke.py \
  A=<bag_A> B=<bag_B> C=<bag_C>
```

### `analysis/analyze_sim_speed_issue.py`

仿真 MPC 速度偏慢问题诊断脚本，从 bag 中分析机器人在哪些阶段速度低于预期。

关注内容：
- 低速（默认阈值 0.25 m/s）帧数占比及分布
- `/slosh/v_des_eff` 与实际 `/cmd_vel` 的差异
- `risk_scheduler/rho_k`、`Q_eta_k`、`fallback_active` 在低速段的状态
- `terminal/mode` 是否在 near-goal 段拉低整体速度

适用场景：
- 区分"全程慢"还是"near-goal 段慢"
- 判断速度低是 risk scheduler 激进还是 terminal recovery 保守

### `analysis/analyze_global_path_duplicates.py`

定位 `/scout/global_path` 中重复点/极短段的脚本，输出每个可疑位置的索引和坐标。

关注内容：
- 段长低于阈值（默认 1e-3 m）的相邻点对
- 每对可疑短段的 `idx → idx+1`、段长、前后点坐标

适用场景：
- 确认全局路径发布链是否存在重复 waypoint
- 为 `PathHandler::sanitizePolyline` 的阈值设置提供依据

### `analysis/analyze_tracking_infeasible.py`

TRACKING 阶段 OSQP -3 根因分析脚本，将 solver 失败时刻与路径几何异常对齐。

关注内容：
- `/mpc/status_val` 失败时刻最近的 `/mpc/reference_path`、`/scout/global_path_smooth` 几何量（max_kappa、max_dkappa、min_seg）
- 失败事件的几何特征分布（按路径来源分层）
- 判断失败主因：全局路径几何过激 / 局部参考几何病态 / 非几何主导

适用场景：
- 定位 TRACKING infeasible 的真正根因
- 区分"geometric failure"和"constraint structure failure"（如 u_prev 冻结）

### `analysis/analyze_global_path_prefix_window.py`

分析 `/scout/global_path` 前段窗口几何的专项脚本，面向"路径起步阶段 fitLocalSpline 频繁失败"场景。

关注内容：
- 前 K 个路径点（默认 89 个，对应 `closest_idx=0, end=88` 的典型失败窗口）的段长、航向跳变和曲率变化率
- 最短段、最大航向跳变的具体索引和坐标

用法：
```bash
python3 src/scout_apps/control/scout_local_planner/scripts/analysis/analyze_global_path_prefix_window.py <bag> --window-size 89
```

适用场景：
- 定位前段局部窗口几何过激的具体点位
- 判断是单点折返尖刺还是正常大曲率弯道

### `analysis/diagnose_speed_profile.py`

速度剖面限速来源诊断脚本，对单条 bag 分析当前速度慢的根因属于哪一候选。

三类候选根因：
- **A**：`global_spline kappa` 远大于实际路径几何（cubic spline 二阶导数放大）
- **B**：`dkappa` 有限差分噪声放大（`dkappa >> 100 1/m²`）
- **C**：局部 reactive cap 仍在主导全局速度剖面（速度剖面计算正确，但被执行层重写）

输出内容：
- 全局平滑路径的 kappa/dkappa 统计及各项 `v_geom_min`（a_lat / omega / alpha 三约束）
- 实际 cmd_vel 速度分布（均值、中位数、低速段占比）

用法：
```bash
python3 src/scout_apps/control/scout_local_planner/scripts/analysis/diagnose_speed_profile.py <bag> --omega-max 2.0 --alpha-max 4.0 --a-lat-max 1.0
```

适用场景：
- 判断速度慢的主因在规划层还是执行层
- 验证 P1 v_plan/v_exec 解耦是否真正生效

### `analysis/observe_terminal_recovery.py`

终点恢复行为观察脚本，用于在线或回放时判断机器人在终点附近是否进入终点恢复逻辑，并输出阶段性状态。

关注内容包括：
- 当前位置与 goal 的相对关系
- 是否处于终点对点接近、终点朝向对齐等恢复阶段
- `/cmd_vel` 与 `/odom` 是否符合预期恢复行为

适用场景：
- 分析 near-goal/terminal recovery 行为是否合理
- 排查“到点后转不正”“终点附近抖动”之类问题

### `imu_ay_tool.py`

IMU 横向加速度一体化工具，合并 Stage-2 标准动作、离线健康检查和 `imu_ay_scale` 标定。

子命令：
- `sequence`：向 `/cmd_vel` 发布固定低速动作序列：静止、左弧线、静止、右弧线、静止。
- `analyze`：读取 bag 中的 `/imu/data`、`/odom` 和 `/slosh/imu_ay_*`，检查静止残差、左右转符号、与 `v*omega` 的相关性。
- `calibrate`：从单个标定 bag 估计 `slosh_estimator/imu_ay_scale`，输出 YAML，可选保存验证图。

典型用法：
```bash
rosrun scout_local_planner imu_ay_tool.py sequence --linear 0.30 --omega 0.30
python3 scripts/imu_ay_tool.py analyze /path/to/imu_calib.bag
python3 scripts/imu_ay_tool.py calibrate /path/to/imu_calib.bag --output /data/a/imu_calib/imu_ay_calibration.yaml --plot
```

适用场景：
- 采集 IMU 横向加速度 `a_y` 标定 bag
- 判断 IMU `a_y` 是否已经达到可接入控制器的质量
- 生成可写入 launch / yaml 的 `imu_ay_scale`

### `run_imu_stage4_sequence.py`

Stage-4 IMU `alpha_z` 验证动作脚本，向 `/cmd_vel` 发布一组固定的原地旋转序列。

典型序列：
- 静止
- 左自旋
- 静止
- 右自旋
- 静止

适用场景：
- 采集 IMU 角加速度/角速度变化验证 bag
- 为 Stage-4 相关分析提供标准化激励

### `analysis/validate_sim_imu.py`

在线验证仿真 IMU 话题质量的脚本（需要 ROS 节点在线，非离线 bag 分析）。

关注内容：
- `/imu/data` 频率、时间戳一致性、`frame_id`
- IMU `linear_acceleration.y` 与运动学估计 `v * omega` 的偏差
- TF `imu_link` 是否可查询

适用场景：
- 仿真首次接入真 IMU 时的链路验收
- 确认 Gazebo IMU 插件输出与 `/odom` 时间戳对齐

### `launch_sim_nav_stack.sh`

路径：`scripts/launch_sim_nav_stack.sh`

仿真导航栈一键启动脚本，顺序拉起 Gazebo、Cartographer 定位、MBF 全局规划器。它只负责“环境与全局规划栈”，不负责单包实验、local planner 参数和 rosbag 录制。

典型完整流程：

```bash
# 终端 1：启动仿真环境 + 定位 + MBF 全局规划
source /home/a/scout_ws/devel/setup.bash
SIM_ENV=open USE_RVIZ=true \
SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
rosrun scout_local_planner launch_sim_nav_stack.sh

# 终端 2：启动一次 trial 并录 bag
source /home/a/scout_ws/devel/setup.bash
SCENARIO=open_user_goal CONDITION=GEOREF_OSCRS_ACTIVE RUN_ID=active01 \
START_DELAY=10 RECORD_DURATION=0 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

支持环境：

```text
SIM_ENV=open
  world = open_walled.world
  map   = map_sim_empty.pbstream
  推荐用于 OSCRS / GeoRef 主线 open-field 对比。

SIM_ENV=maze
  world = maze_course.world
  map   = map_carto.pbstream
  用于迷宫环境可达性/碰撞检查调试，不建议作为当前主线有效性验证场地。

SIM_ENV=custom
  必须手动提供 WORLD_NAME 和 MAP_FILE。
```

支持环境变量配置：
- `USE_RVIZ`：是否启动 RViz（默认 false）
- `SIM_ENV`：`open / maze / custom`
- `WORLD_NAME`、`MAP_FILE`：`SIM_ENV=custom` 时必须提供
- `SPAWN_X/Y/Z`：机器人初始位置
- `SPAWN_YAW`：机器人初始 yaw
- `GAZEBO_WAIT_S`：等待 Gazebo 就绪时间
- `LOCALIZATION_BACKUP_V`：定位初始化倒退速度
- `OPEN_LOCALIZATION_FORWARD_S/V`：open 场地定位刷新前进动作
- `OPEN_LOCALIZATION_TURN_S/OMEGA`：open 场地定位刷新左右转动作

用法：
```bash
rosrun scout_local_planner launch_sim_nav_stack.sh
USE_RVIZ=true rosrun scout_local_planner launch_sim_nav_stack.sh
SIM_ENV=maze USE_RVIZ=true rosrun scout_local_planner launch_sim_nav_stack.sh
```

注意：
- 本脚本只启动到全局规划器，不启动 local planner。
- local planner 一般不要手动直接启动，而是交给 `run_sim_fixed_path_bag.sh` 按 `CONDITION` 启动，这样 bag 命名、路径发布和参数口径一致。
- 如果只想手动调试 local planner，也可以另开终端直接启动 `slosh_experiment_sim.launch`。

### `run_day4_profile.sh`

Day 4 仿真参数组合包装脚本，统一管理 baseline / conservative / no_imu_ay / relaxed_settling 四组预定义参数，避免手工切换时漏参。

支持的 profile：

| profile | 说明 |
|---|---|
| `baseline` | Day4 C baseline，默认参数 |
| `conservative` | 保守风险调度（gamma=3.0, rho_0=0.4, rate_limit=0.02, beta=0.2） |
| `no_imu_ay` | 关闭 IMU 横向加速度，保留风险调度 |
| `relaxed_settling` | 放宽 settling 终止阈值（eta_tol=0.002, eta_dot_tol=0.05） |

用法：
```bash
rosrun scout_local_planner run_day4_profile.sh conservative
rosrun scout_local_planner run_day4_profile.sh conservative Q_slosh:=10
```

适用场景：
- Day 4 仿真参数 sweep，快速切换对照组
- 复现指定参数组的仿真 bag

## 推荐用法

如果目标是做“固定路径下的 Q0/Q5 anti-slosh 对照实验”，推荐顺序如下：

1. 用实时规划先跑一次，并用 `fixed_global_path_runner.py` 的 `capture` 模式保存第一次 `/scout/global_path`
2. 用 `send_fixed_goal.py` 或已有导航链将机器人移回固定起点
3. 用 `fixed_global_path_runner.py` 的 `replay` 模式发布固定路径
4. 用 `record_slosh_experiment.sh` 录制 Q0 和 Q5 两组 bag
5. 用 `extract_slosh_metrics.py` 做离线对比分析
