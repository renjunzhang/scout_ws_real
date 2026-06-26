# 仿真对比试验运行命令与 fresh-sim 策略（agent 入口）

> 本文件给后续新对话/agent 使用：先读这里，再跑点到点或固定路径对比。
> 目标是避免误用旧仿真、旧 LT-DWA adapter、同一个 Gazebo session 反复 reset 等问题。

## 0. 红线与默认环境

- 默认仿真根目录：`/data/a/scout_sim_replacement`
- 默认 Scout 项目根目录：`/home/a/scout_ws`
- 默认地图必须显式传入：

```bash
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
```

- 不要依赖 launch/script 内部默认地图。
- 不使用旧 `/home/a/scout_ws` 仿真启动栈；`/home/a/scout_ws` 只作为只读代码/overlay 来源。
- 不使用 broad `killall` / `pkill`。
- 不使用 `git reset`、`git clean`、`git checkout`、`git push`。
- 正式对比必须 fresh sim：每个 case 单独启动/关闭隔离 Gazebo/ROS，不在同一个 Gazebo session 里反复 reset 后跑正式数据。
- 60s 规则：从 planner 开始 tracking/观测窗口开始计时，60s 内未 `GOAL_REACHED` 统一 FAIL；超过 60s 后才到也 FAIL。
- `slosh` 只作为外部评价指标；外部 baseline 不允许使用液面反馈。

相关 SOP：

```text
docs/实物实验注意事项/仿真环境/20260611_隔离Scout仿真环境使用SOP.md
```

## 1. 算法入口状态（2026-06-25）

| 算法 | 固定路径 fresh-sim | 点到点 smoke | 备注 |
|---|---|---|---|
| `spmpc` / `B_ours` | 可用 | 可用 | 主方法，固定路径用 `run_proxy_spmpc_mainline_smoke.sh` 或 strict N=3 wrapper。 |
| `teb` | 可用 | 可用 | nav_core baseline。 |
| `dwa` | 可用 | 可用 | nav_core baseline。 |
| `mpc_local_planner` | 原始配置未到点；tuned fixed-path 配置 2026-06-25 strict N=3 已 3/3 到点 | 可用/需 overlay | fixed-path formal 推荐显式传 `MPC_PLANNER_CONFIG=/home/a/scout_ws/src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_tuned_sim.yaml`；点到点若手动跑，要 source isolated MPC overlay。 |
| `lt_dwa_official_wrapper` | 可用 | 暂不作为正式 P2P 入口 | 正确入口是 `src/scout_apps/control/lt_dwa_official_wrapper`；不要再用旧 `lt_dwa_adapter`。 |

**LT-DWA 特别注意：**

- 不要把 `BASELINE=lt_dwa` 直接交给旧的 generic baseline runner，除非已确认它内部改成 `lt_dwa_official_wrapper`。
- 2026-06-25 原矩阵中的 `lt_dwa_adapter_resource_not_found` 是旧入口错误，不是 LT-DWA 性能结论。
- 当前验证通过的 LT-DWA fixed-path 复跑脚本：

```text
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/规控一体的实验记录/仿真实验/20260625_s_curve_common_limit_n3_20260625_182535/run_lt_dwa_official_corrected_n3.sh
```

- LT-DWA official-wrapper 需要 runtime overlay 优先解析：

```bash
export ROS_PACKAGE_PATH=/home/a/scout_ws/tools/lt_dwa/local_planner_runtime:${ROS_PACKAGE_PATH:-}
```

## 2. Fresh-sim 正式更新策略

正式结果更新时按下面流程执行，agent 不要跳步：

1. **预检查端口为空**
   - SPMPC mainline 常用：`ROS_MASTER_URI=http://localhost:11330`，`GAZEBO_MASTER_URI=http://localhost:11364`
   - baseline 常用：`ROS_MASTER_URI=http://localhost:11331`，`GAZEBO_MASTER_URI=http://localhost:11365`
   - 如果预检查发现 ROS/Gazebo 已存在，本 case 标记 `strict_fresh_invalid`，不要 reset 后继续。

2. **每个 case 单独 fresh 启动**
   - 显式 `MAP_FILE=...pbstream`。
   - 等 `/map`、`/odom`、`/scan_front`、`/tf` ready。
   - 定位稳定等待建议 `STRICT_PRE_CONTROL_SETTLE_SEC=30`。

3. **正式 fixed-path 录包顺序**
   - 发布固定路径/goal。
   - 启动 slosh monitor。
   - **先启动 rosbag**。
   - 再启动 planner/允许 bounded `/cmd_vel`。
   - 观测窗口 `RECORD_SEC=60`；60s 未到点 FAIL。

4. **结束与校验**
   - 只停止脚本自己追踪到的子进程 PID。
   - 等待 `STRICT_POST_SHUTDOWN_SEC=30`。
   - 再检查 ROS/Gazebo 端口为空；不为空则该 case 标记 invalid。

5. **结果更新**
   - raw result/bag 保留在 `/data/a/scout_sim_replacement/results` 和 `/data/a/scout_sim_replacement/bags`。
   - Obsidian 只做报告镜像：`/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/规控一体的实验记录/仿真实验/`。
   - 如果替换某算法入口（如 LT-DWA adapter -> official wrapper），主表要明确：旧行是接入错误审计，正式性能使用新入口复跑结果。

## 3. 固定路径跟踪对比：formal fresh-sim 命令

### 3.1 SPMPC / TEB / DWA / mpc_local_planner：N=3 S 曲线矩阵

> 这个 generic strict wrapper 当前适合 `spmpc teb dwa mpc_local_planner`。
> **不要把 `lt_dwa` 放进 MATRIX，除非已确认 wrapper 内部不再走旧 `lt_dwa_adapter`。**

```bash
cd /home/a/scout_ws

BATCH_STAMP=$(date +%Y%m%d_%H%M%S)_s_curve_common_limit_n3 \
N=3 \
MATRIX="spmpc teb dwa mpc_local_planner" \
MODE=closed_loop \
RECORD_SEC=60 \
STRICT_PRE_CONTROL_SETTLE_SEC=30 \
STRICT_POST_SHUTDOWN_SEC=30 \
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream \
GOAL_X=5.0 \
GOAL_Y=0.0 \
GOAL_YAW=0.0 \
PATH_TEMPLATE=s_curve \
PATH_START_HEADING=current \
PATH_AMPLITUDE_RATIO=0.18 \
PATH_MIN_AMPLITUDE=0.25 \
PATH_MAX_AMPLITUDE=1.20 \
PATH_SIDE=left \
PATH_SMOOTH_ITERATIONS=3 \
/data/a/scout_sim_replacement/scripts/run_strict_fresh_fair_comparison_n3.sh
```

输出位置由脚本打印，并写入：

```text
/data/a/scout_sim_replacement/results/<BATCH_STAMP>/strict_fresh_manifest.csv
/data/a/scout_sim_replacement/bags/<BATCH_STAMP>/...
```

#### 3.1.1 fixed-path tuned `mpc_local_planner` 正式入口

原始 `mpc_local_planner_standalone_sim.yaml` 在 2026-06-25 strict N=3 中未到点。修复后的 fixed-path 对比入口必须显式传 tuned config：

```bash
cd /home/a/scout_ws

BATCH_STAMP=$(date +%Y%m%d_%H%M%S)_mpc_local_planner_tuned_quad_shortlookahead_n3 \
N=3 \
MATRIX="mpc_local_planner" \
MPC_PLANNER_CONFIG=/home/a/scout_ws/src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_tuned_sim.yaml \
MODE=closed_loop \
RECORD_SEC=60 \
STRICT_PRE_CONTROL_SETTLE_SEC=30 \
STRICT_POST_SHUTDOWN_SEC=30 \
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream \
GOAL_X=5.0 \
GOAL_Y=0.0 \
GOAL_YAW=0.0 \
PATH_TEMPLATE=s_curve \
PATH_START_HEADING=current \
PATH_AMPLITUDE_RATIO=0.18 \
PATH_MIN_AMPLITUDE=0.25 \
PATH_MAX_AMPLITUDE=1.20 \
PATH_SIDE=left \
PATH_SMOOTH_ITERATIONS=3 \
/data/a/scout_sim_replacement/scripts/run_strict_fresh_fair_comparison_n3.sh
```

已验证结果：`strict_fresh_fair_n3_20260625_224326_mpc_local_planner_tuned_quad_shortlookahead_n3`，3/3 `GOAL_REACHED`，tracking RMS mean `0.100m`，tracking max mean `0.192m`。

### 3.2 单算法 fixed-path fresh smoke：SPMPC

```bash
cd /home/a/scout_ws

RUN_STAMP=$(date +%Y%m%d_%H%M%S)_spmpc_B_ours_s_curve \
VARIANTS=B_ours \
MODE=closed_loop \
RECORD_SEC=60 \
RECORD_BAG=true \
SLOSH_MONITOR_ENABLE=true \
STOP_ON_GOAL=true \
STOP_ON_NO_GO=true \
STRICT_PRE_CONTROL_SETTLE_SEC=30 \
SOLVER_BACKEND=continuous_mpcc_acados \
ALPHA_MAX=1.2 \
MAX_ANGULAR_ACCEL=1.2 \
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream \
GOAL_X=5.0 \
GOAL_Y=0.0 \
GOAL_YAW=0.0 \
PATH_TEMPLATE=s_curve \
PATH_START_HEADING=current \
PATH_AMPLITUDE_RATIO=0.18 \
PATH_MIN_AMPLITUDE=0.25 \
PATH_MAX_AMPLITUDE=1.20 \
PATH_SIDE=left \
PATH_SMOOTH_ITERATIONS=3 \
/data/a/scout_sim_replacement/scripts/run_proxy_spmpc_mainline_smoke.sh
```

### 3.3 单算法 fixed-path fresh smoke：TEB / DWA / mpc_local_planner

把 `BASELINE` 改成 `teb`、`dwa` 或 `mpc_local_planner`：

```bash
cd /home/a/scout_ws

RUN_STAMP=$(date +%Y%m%d_%H%M%S)_teb_s_curve \
BASELINE=teb \
MODE=closed_loop \
RECORD_SEC=60 \
RECORD_BAG=true \
SLOSH_MONITOR_ENABLE=true \
STOP_ON_GOAL=true \
STOP_ON_NO_GO=true \
REQUIRE_GOAL=true \
STRICT_PRE_CONTROL_SETTLE_SEC=30 \
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream \
GOAL_X=5.0 \
GOAL_Y=0.0 \
GOAL_YAW=0.0 \
GOAL_YAW_MODE=path_end \
PATH_TEMPLATE=s_curve \
PATH_START_HEADING=current \
PATH_AMPLITUDE_RATIO=0.18 \
PATH_MIN_AMPLITUDE=0.25 \
PATH_MAX_AMPLITUDE=1.20 \
PATH_SIDE=left \
PATH_SMOOTH_ITERATIONS=3 \
TARGET_V_MAX_MPS=0.8 \
TARGET_OMEGA_MAX_RADPS=1.2 \
TARGET_ACC_LIM_X_MPS2=0.6 \
TARGET_ACC_LIM_THETA_RADPS2=1.2 \
/data/a/scout_sim_replacement/scripts/run_proxy_baseline_mainline_smoke.sh
```

示例替换：

```bash
BASELINE=dwa              # DWA
BASELINE=mpc_local_planner # mpc_local_planner 原始默认配置
```

fixed-path `mpc_local_planner` 推荐使用 tuned 配置，否则可能复现原始 N=3 `GOAL_NOT_REACHED`：

```bash
BASELINE=mpc_local_planner \
MPC_PLANNER_CONFIG=/home/a/scout_ws/src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_tuned_sim.yaml
```

### 3.4 LT-DWA official-wrapper fixed-path formal N=3

当前已验证的 corrected N=3 helper：

```bash
bash "/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/规控一体的实验记录/仿真实验/20260625_s_curve_common_limit_n3_20260625_182535/run_lt_dwa_official_corrected_n3.sh"
```

该 helper 的关键语义必须保持：

```text
planner_execution_mode:=in_process
raw_cmd_topic:=/baseline/lt_dwa/raw_cmd_vel
shadow_cmd_topic:=/baseline/lt_dwa/shadow_cmd_vel
diagnostics_topic:=/baseline/lt_dwa/diagnostics
publish_cmd_vel:=true
enable_actuated_output:=true
cmd_vel_topic:=/cmd_vel
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
record bag before bounded actuation
RECORD_SEC=60
fresh sim per run
```

如果将该 helper 迁移到 `/data/a/scout_sim_replacement/scripts/`，迁移后必须先做 shadow preflight：

```bash
# 已有隔离仿真 + fixed path 后，只 shadow，不驱动 /cmd_vel
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
export ROS_PACKAGE_PATH=/home/a/scout_ws/tools/lt_dwa/local_planner_runtime:${ROS_PACKAGE_PATH:-}

roslaunch spmpc_experiments run_lt_dwa_fixed_path_sim.launch \
  planner_execution_mode:=in_process \
  publish_cmd_vel:=false \
  enable_actuated_output:=false \
  cmd_vel_topic:=/cmd_vel \
  raw_cmd_topic:=/baseline/lt_dwa/raw_cmd_vel \
  max_v:=0.8 \
  max_w:=1.2 \
  max_acc:=0.6 \
  max_angular_acc:=1.2 \
  global_path_topic:=/scout/global_path_fixed \
  goal_topic:=/scout/goal \
  odom_topic:=/odom \
  map_topic:=/map \
  base_frame:=base_link \
  plan_target_frame:=map
```

shadow diagnostics 必须看到近似字段：

```text
status=OK
execution_mode=in_process
has_raw_command=true
has_final_command=true
```

## 3.5 下一轮内部消融：N=9 full + Top5 展示矩阵（2026-06-26 计划）

> 给后续 agent 的执行入口：本节用于跑 SPMPC 内部消融，不替换第 3.1 节已有 N=3 大矩阵。
> 目标是同时保留完整 N=9 统计可信度，并额外筛出 5 个代表性改善案例用于展示方法有效性。

### 3.5.1 变量轴与矩阵

硬约束已经接入，内部消融不要把 `B_ours_hard` 直接混成唯一最终方法；否则会分不清收益来自 slosh soft cost、smooth-priority，还是 hard cap。推荐拆成两层：

| 层级 | Variant | slosh soft | smooth priority | hard constraint | 主要问题 |
|---|---|---:|---:|---:|---|
| primary soft ablation | `spmpc_B0` | 否 | 否 | 否 | 无 slosh/无 smooth 的基线 |
| primary soft ablation | `spmpc_B_smooth` | 否 | 是 | 否 | 只看 smooth-priority |
| primary soft ablation | `spmpc_B_slosh` | 是 | 否 | 否 | 只看 slosh soft cost |
| primary soft ablation | `spmpc_B_ours` | 是 | 是 | 否 | soft slosh + smooth-priority 的主方法 |
| hard-cap increment | `spmpc_B_slosh_hard` | 是 | 否 | 是 | hard cap 对 slosh-only 的增量 |
| hard-cap increment | `spmpc_B_ours_hard` | 是 | 是 | 是 | hard cap 对最终方法的增量 |

正式 full N=9 至少跑这 6 个 variant，每个 variant 都保留 9 个 strict fresh cases。不要只保留好看的 5 个作为正式统计。

### 3.5.2 固定路径参数

与 `20260626_fixed_path_s_curve_matrix_n3/01_固定路径S曲线矩阵N3汇总.md` 保持一致：

```bash
GOAL_X=5.0
GOAL_Y=0.0
GOAL_YAW=0.0
PATH_TEMPLATE=s_curve
PATH_START_HEADING=current
PATH_AMPLITUDE_RATIO=0.18
PATH_MIN_AMPLITUDE=0.25
PATH_MAX_AMPLITUDE=1.20
PATH_SIDE=left
PATH_SMOOTH_ITERATIONS=3
RECORD_SEC=60
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
```

### 3.5.3 推荐运行方式：按 variant 分批，避免长命令挂死

不要一次把 6 个 variant 全塞进一个长矩阵命令。推荐一个 variant 一批，便于超时、失败重跑和审计：

```bash
cd /home/a/scout_ws
export ACADOS_SOURCE_DIR=/home/a/acados
export LD_LIBRARY_PATH=/home/a/acados/lib:${LD_LIBRARY_PATH:-}

BATCH_STAMP=$(date +%Y%m%d_%H%M%S)_internal_ablation_n9_spmpc_B_ours \
N=9 \
MATRIX="spmpc_B_ours" \
MODE=closed_loop \
RECORD_SEC=60 \
STRICT_PRE_CONTROL_SETTLE_SEC=30 \
STRICT_POST_SHUTDOWN_SEC=30 \
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream \
GOAL_X=5.0 \
GOAL_Y=0.0 \
GOAL_YAW=0.0 \
PATH_TEMPLATE=s_curve \
PATH_START_HEADING=current \
PATH_AMPLITUDE_RATIO=0.18 \
PATH_MIN_AMPLITUDE=0.25 \
PATH_MAX_AMPLITUDE=1.20 \
PATH_SIDE=left \
PATH_SMOOTH_ITERATIONS=3 \
/data/a/scout_sim_replacement/scripts/run_strict_fresh_fair_comparison_n3.sh
```

把 `BATCH_STAMP` 和 `MATRIX` 依次替换为：

```text
spmpc_B0
spmpc_B_smooth
spmpc_B_slosh
spmpc_B_ours
spmpc_B_slosh_hard
spmpc_B_ours_hard
```

说明：脚本名仍叫 `n3`，但由环境变量 `N=9` 控制实际次数。每个 run 仍必须是 fresh sim；如果 manifest 里任何 case 的 pre/post ROS/Gazebo reachability 不是 false，不能当 strict fresh 正式数据。

### 3.5.4 Top5 筛选规则：先全量统计，再展示代表性改善

全量 N=9 aggregate 必须完整保留。Top5 只用于展示，不替代 full table。

建议固定两个 Top5 榜：

1. **Top5 soft improvement**：比较 `spmpc_B_slosh -> spmpc_B_ours`。
   - 目的：突出 smooth-priority 加到 slosh-aware MPC 后的收益。
   - 候选：paired run 中两者都 `valid_strict_case=true`、都 `goal_reached=true`。
   - 保护条件：`tracking_rms_B_ours <= tracking_rms_B_slosh * 1.10`，或绝对恶化不超过 `+0.01 m`。
   - 排序：优先按 `slosh_p95` 降幅从大到小；并列时看 `slosh_peak` 降幅，再看 tracking RMS。

2. **Top5 hard-cap increment**：比较 `spmpc_B_ours -> spmpc_B_ours_hard`。
   - 目的：展示 hard constraint 在不显著损害 tracking 时对液面风险指标的增量。
   - 候选与保护条件同上。
   - 排序：优先按 `slosh_p95` 降幅；如果研究重点改成安全峰值，可在报告中明确改用 `slosh_peak` 降幅。

报告措辞必须写清楚：

```text
完整 N=9 strict-fresh 统计见 full table；Top5 是按预设规则从 valid/success paired cases 中筛出的代表性改善案例，用于展示，不替代全量统计。
```

### 3.5.5 输出建议

建议写入新的 Obsidian 子报告，不覆盖已有 N=3：

```text
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/规控一体的实验记录/仿真实验/20260626_fixed_path_s_curve_matrix_n3/04_SPMPC内部消融_N9全量与Top5.md
```

机器表格建议：

```text
tables/spmpc_internal_ablation_n9_per_case.csv
tables/spmpc_internal_ablation_n9_aggregate.csv
tables/spmpc_internal_ablation_n9_pairs_soft.csv
tables/spmpc_internal_ablation_n9_top5_soft.csv
tables/spmpc_internal_ablation_n9_pairs_hard.csv
tables/spmpc_internal_ablation_n9_top5_hard.csv
```

图像建议：

```text
figures/spmpc_internal_ablation_n9_full_*.png
figures/spmpc_internal_ablation_n9_top5_soft_*.png
figures/spmpc_internal_ablation_n9_top5_hard_*.png
```

### 3.5.6 硬约束注意事项

- 当前 hard cap 默认 `slosh_height_max=0.008 m`。
- 2026-06-26 的普通 S 曲线 fresh-sim 中，外部 slosh 大约是 1 mm 量级，明显低于 8 mm cap；因此 hard constraint 可能不 active。正常 S 曲线里不要强行宣称 hard cap 大幅改善，只能说它在正常跟踪下未破坏性能，且作为 safety-constrained 版本保留。
- 如果要证明 hard constraint 真正 active，应单独设计 stress/sensitivity 场景，例如更激进路径、更高速度或更紧 `slosh_height_max`。这类 stress 结果必须单独标注，不能混入 common-limit formal baseline。
- runner 当前 rosbag topic 列表未必包含 `/spmpc/debug/slosh_hard_constraint`。不要擅自修改 `/data/a/scout_sim_replacement` 脚本；如果必须记录 hard-margin topic，先向用户确认。

### 3.5.7 子代理 / 后台任务超时要求

用户明确要求“别挂太多 subagent，记得给 subagent 加超时”。后续 agent 必须遵守：

- 默认不要让 subagent 直接控制仿真；仿真运行由主 agent 顺序执行，最多只把结果聚合/审计交给子代理。
- 同时运行的 subagent 不超过 2 个；禁止一次 fan-out 很多仿真代理。
- 如果使用 subagent：
  - 代码/结果扫描代理：超时 5 分钟；
  - 单批结果聚合/画图代理：超时 10 分钟；
  - 单 variant N=9 仿真运行如果确实交给后台/子代理：必须在提示词里写明硬超时 45 分钟，并要求超时后停止该批、报告未完成 case，不得静默继续。
- 如果使用 shell 后台命令，外层要有明确超时；例如普通终端可用 `timeout --preserve-status 45m ...` 包住单 variant N=9 命令。若当前工具本身有更短硬限制，则按 variant/run 拆得更小，不要靠无限等待。
- N=9 全矩阵 6 个 variant 预期可能持续数小时；要按 variant 分批记录 batch stamp 和 manifest。任一批超时/失败后先汇报，不要继续把残缺数据写成 full N=9。
- 后处理脚本必须检查 `strict_fresh_manifest.csv`：`exit_status=0`、`valid_strict_case=true`、pre/post ROS/Gazebo 均 false。检查失败则标记 invalid，不得为了 Top5 筛选把 invalid case 混入候选。

## 4. 固定路径跟踪：current-sim diagnostic 命令

> 只用于人工观察/调试，不是 formal fresh-sim 证据。

### 4.1 启动隔离仿真 + 定位 + RViz

```bash
export ROS_MASTER_URI=http://localhost:11328
export GAZEBO_MASTER_URI=http://localhost:11362
export MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream

USE_RVIZ=true \
GAZEBO_GUI=true \
TRACKING_RVIZ=true \
/data/a/scout_sim_replacement/scripts/launch_proxy_sim_localization_env.sh
```

确认 RViz 中 `/map`、`/scan_front`、RobotModel、TF 对齐后，再另开终端 attach planner。

### 4.2 current-sim attach：SPMPC fixed path

```bash
export ROS_MASTER_URI=http://localhost:11328
export GAZEBO_MASTER_URI=http://localhost:11362

GOAL_X=5.0 \
GOAL_Y=0.0 \
GOAL_YAW=0.0 \
PATH_TEMPLATE=s_curve \
PATH_START_HEADING=current \
PATH_AMPLITUDE_RATIO=0.18 \
PATH_SIDE=left \
/data/a/scout_sim_replacement/scripts/launch_proxy_spmpc_localized_attach.sh
```

### 4.3 current-sim attach：TEB / DWA / mpc_local_planner fixed path

把 `BASELINE` 改成 `teb`、`dwa`、`mpc_local_planner`：

```bash
export ROS_MASTER_URI=http://localhost:11328
export GAZEBO_MASTER_URI=http://localhost:11362

BASELINE=teb \
GOAL_X=5.0 \
GOAL_Y=0.0 \
GOAL_YAW=0.0 \
GOAL_YAW_MODE=path_end \
PATH_TEMPLATE=s_curve \
PATH_START_HEADING=current \
PATH_AMPLITUDE_RATIO=0.18 \
PATH_SIDE=left \
SLOSH_MONITOR_ENABLE=true \
/data/a/scout_sim_replacement/scripts/launch_proxy_baseline_localized_attach.sh
```

`mpc_local_planner` current-sim fixed-path 诊断推荐同样显式指定 tuned config：

```bash
BASELINE=mpc_local_planner \
MPC_PLANNER_CONFIG=/home/a/scout_ws/src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_tuned_sim.yaml \
GOAL_X=5.0 \
GOAL_Y=0.0 \
GOAL_YAW=0.0 \
GOAL_YAW_MODE=path_end \
PATH_TEMPLATE=s_curve \
PATH_START_HEADING=current \
PATH_AMPLITUDE_RATIO=0.18 \
PATH_SIDE=left \
SLOSH_MONITOR_ENABLE=true \
/data/a/scout_sim_replacement/scripts/launch_proxy_baseline_localized_attach.sh
```

**不要用这个 attach 脚本跑 LT-DWA，除非已确认它内部不再走 `lt_dwa_adapter`。**

## 5. 点到点对比：current-sim smoke 命令

> 点到点 smoke 脚本假设仿真已经在运行；默认不是 formal fresh-sim 证据。
> 如果要 formal P2P，请按第 2 节 fresh-sim 策略：每个算法/run 单独启动仿真、跑一个 smoke、关闭并确认端口清空。

### 5.1 启动隔离仿真 + 定位

```bash
export ROS_MASTER_URI=http://localhost:11328
export GAZEBO_MASTER_URI=http://localhost:11362
export MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream

USE_RVIZ=true \
GAZEBO_GUI=true \
TRACKING_RVIZ=true \
/data/a/scout_sim_replacement/scripts/launch_proxy_sim_localization_env.sh
```

### 5.2 点到点：SPMPC

```bash
cd /home/a/scout_ws
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11328
export GAZEBO_MASTER_URI=http://localhost:11362

BASELINE=spmpc \
VARIANT=B_ours \
SPMPC_SOLVER_BACKEND=continuous_mpcc_acados \
RECORD_SEC=60 \
GOAL_X=-1.2 \
GOAL_Y=2.6 \
GOAL_YAW=1.0 \
OUT_DIR=/data/a/scout_sim_replacement/results/p2p_smoke_$(date +%Y%m%d_%H%M%S) \
bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

### 5.3 点到点：TEB / DWA

```bash
cd /home/a/scout_ws
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11328
export GAZEBO_MASTER_URI=http://localhost:11362

BASELINE=teb \
RECORD_SEC=60 \
GOAL_X=-1.2 \
GOAL_Y=2.6 \
GOAL_YAW=1.0 \
OUT_DIR=/data/a/scout_sim_replacement/results/p2p_smoke_$(date +%Y%m%d_%H%M%S) \
bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

替换：

```bash
BASELINE=dwa
```

### 5.4 点到点：mpc_local_planner

`mpc_local_planner` 需要 isolated MPC overlay：

```bash
cd /home/a/scout_ws
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
source /home/a/scout_ws/install_isolated_mpc/setup.bash
export ROS_MASTER_URI=http://localhost:11328
export GAZEBO_MASTER_URI=http://localhost:11362

BASELINE=mpc_local_planner \
RECORD_SEC=60 \
GOAL_X=-1.2 \
GOAL_Y=2.6 \
GOAL_YAW=1.0 \
OUT_DIR=/data/a/scout_sim_replacement/results/p2p_smoke_$(date +%Y%m%d_%H%M%S) \
bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

### 5.5 点到点：LT-DWA

截至 2026-06-25，LT-DWA official-wrapper 已验证 fixed-path bounded 仿真；**不要把 LT-DWA P2P 当作正式入口**，除非新增/验证了专门的 P2P wrapper。

如果 agent 要补 LT-DWA P2P：先 shadow，只看 `/baseline/lt_dwa/raw_cmd_vel`、`/baseline/lt_dwa/shadow_cmd_vel`、`/baseline/lt_dwa/diagnostics`；确认 `status=OK`、`has_raw_command=true`、`has_final_command=true` 后，才允许 bounded `/cmd_vel` 仿真测试。实物默认不要直接打开 `/cmd_vel`。

## 6. 结果后处理命令

### 6.1 固定路径 XY overlay

```bash
cd /home/a/scout_ws
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash

python3 src/scout_apps/control/spmpc_experiments/scripts/plot_fixed_path_xy_overlay.py \
  /path/to/run.bag \
  --output /path/to/xy_overlay.png \
  --csv /path/to/xy_overlay.csv \
  --path-topic /scout/global_path_fixed \
  --odom-topic /odom \
  --target-frame map
```

### 6.2 当前已更新的 S 曲线 N=3 记录

```text
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/规控一体的实验记录/仿真实验/20260625_s_curve_common_limit_n3_20260625_182535/01_矩阵结果汇总.md
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/规控一体的实验记录/仿真实验/20260625_s_curve_common_limit_n3_20260625_182535/02_LT-DWA官方wrapper复跑N3.md
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/规控一体的实验记录/仿真实验/20260625_s_curve_common_limit_n3_20260625_182535/03_MPC-local-planner修复复跑N3.md
```

## 7. 给 agent 的快速决策

- 用户说“正式对比 / N=3 / 论文表 / fresh sim”：用第 3 节 fixed-path formal 命令；每个 case fresh sim；60s 硬截止。
- 用户说“打开仿真让我看 / smoke / 当前观察”：用第 4 或第 5 节 current-sim diagnostic 命令；不要标成 strict fresh-sim 证据。
- 用户说“LT-DWA”：默认 official wrapper；先 shadow，看 raw/final/diagnostics；不要再调用旧 `lt_dwa_adapter`。
- 用户说“点到点”：优先说明当前 P2P smoke 不是自动 strict fresh 矩阵；若要 formal P2P，按 fresh-sim 策略一 case 一启停。
- 用户说“固定路径”：默认 `s_curve`，goal `(5.0, 0.0, 0.0)`，`PATH_START_HEADING=current`，`RECORD_SEC=60`；若跑 `mpc_local_planner`，默认显式传 tuned `MPC_PLANNER_CONFIG`。
