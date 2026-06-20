# spmpc_experiments scripts

本目录放 SPMPC 论文/实物实验前仿真用的实验调度、录包、指标提取和辅助验证脚本。脚本只负责启动不同 planner 的实验 run、发送目标/路径、录包、写 meta 和提取指标；不实现规划/控制算法。

## 使用原则

正式或半正式 fixed-path 对比必须遵守 fresh-sim 规则：

```text
1. 每个 case 单独启动仿真。
2. 启动后等待 30s，让定位恢复。
3. 只跑一个 planner / variant / weight case。
4. 每个 case 约 60s 内仍未有效完成则按失败处理，不继续无限等待。
5. 跑完关闭 planner、rosbag、path publisher、slosh monitor 和仿真。
6. 再等待 30s，确认仿真完全关闭后才启动下一次 fresh sim。
7. 不修改仿真环境；只调整 planner / 算法 / 配置。
```

推荐仿真启动命令示例：

```bash
source /home/a/scout_ws/devel/setup.bash
SIM_ENV=open USE_RVIZ=true \
  SPAWN_X=3.30 SPAWN_Y=0.15 SPAWN_Z=0.1 SPAWN_YAW=-3.08 \
  rosrun scout_local_planner launch_sim_nav_stack.sh
```

> 注意：`run_fixed_path_paper_matrix.sh`、`run_fixed_path_critical_sweep.sh`、`run_p2p_paper_supplement.sh`、`run_robustness_transfer_sweep.sh` 是编排入口，不负责启动/关闭 Gazebo。正式数据不要把它们当成“自动 fresh-sim runner”；需要由外层流程保证一个 case 一个 fresh sim。

## 按对比算法找入口

更完整的文件分类见 `../README_compare_algorithms.md`。常用入口如下：

| 类别 | 方法 | 配置 | 运行入口 | 核心实现 | 表格角色 |
|---|---|---|---|---|---|
| online SPMPC | `B0` / `B_slosh` / `B_smooth` / `B_ours` | SPMPC variant/config；benchmark policy 在 `config/benchmark/` | `run_fixed_path_spmpc_suite.sh` | `src/scout_apps/control/spmpc_local_planner/` | 主表 / 内部消融 |
| legacy local planner | TEB | `config/baselines/teb_local_planner_standalone_sim.yaml` | `run_fixed_path_baseline_suite.sh` | `teb_local_planner` + `baseline_local_planner_runner` | 同层 online baseline |
| legacy local planner | DWA | `config/baselines/dwa_local_planner_standalone_sim.yaml` | `run_fixed_path_baseline_suite.sh` | `navigation/dwa_local_planner` + `baseline_local_planner_runner` | 同层 online baseline |
| fallback MPC | `mpc_local_planner` | `config/baselines/mpc_local_planner_standalone_sim.yaml` | `run_fixed_path_baseline_suite.sh`，需 opt-in | `src/scout_apps/control/mpc_local_planner/` | fallback / supplement |
| profile baseline | Hamaguchi | `config/profile_baselines/hamaguchi_profile.yaml` | `run_fixed_path_profile_baseline_suite.sh` | wrapper `scout_profile_baselines/scripts/generate_hamaguchi_profile.py` -> impl `scripts/hamaguchi/generate_profile.py` | supplementary profile baseline |
| profile baseline | Lim | `config/profile_baselines/lim_profile.yaml` | `run_fixed_path_profile_baseline_suite.sh` | wrapper `scout_profile_baselines/scripts/generate_lim_style_profile.py` -> impl `scripts/lim/generate_profile.py` | supplementary profile baseline |
| advanced candidate | LT-DWA | `config/benchmark/capability_matrix.yaml` | readiness check only | `third_party/LT_DWA/` source-only，adapter pending | candidate，未通过前不进表 |
| advanced candidate | `src/mpc_planner` | `config/benchmark/capability_matrix.yaml` | readiness check only | `src/mpc_planner/` | candidate，未通过前不进表 |

Hamaguchi/Lim 的 runtime chain 固定为：`offline profile generator -> profile_csv -> common tracker -> cmd_vel -> metrics`。slosh monitor 只允许进入 rosbag/metrics/report，不得被 generator、command gate、OCP 或 `/cmd_vel` 链路消费。

## 脚本索引

| 脚本 | 类型 | 主要用途 | fresh-sim 注意事项 |
|---|---|---|---|
| `run_fixed_path_spmpc_suite.sh` | fixed-path run suite | 运行 SPMPC fixed-path case，支持 `planner_variant`、`w_slosh`、可选 `v_ref` 诊断覆盖、solver backend、shared command acceleration limit、slosh monitor meta/reset。 | 只负责单次/一组 planner run；正式实验建议外层每个 case 重启仿真。 |
| `run_fixed_path_baseline_suite.sh` | fixed-path baseline suite | 运行 TEB/DWA/可选 `mpc_local_planner` fixed-path baseline，并写入 speed tier、共同限幅目标、`slosh_eval_only`、`slosh_feedback_forbidden` 等公平性 meta。 | baseline 可以记录 slosh 作为评价信号，但不得作为控制输入；正式实验同样需要每 case fresh sim。 |
| `run_fixed_path_profile_baseline_suite.sh` | fixed-path profile baseline suite | 运行 Hamaguchi/Lim 离线 profile generator，生成标准 CSV 后交给 `scout_local_planner` common tracker；写入 `current_sim_only`、`profile_generated_before_case`、`monitor_feedback_used_for_profile=false` 等 meta。 | 不启动/关闭 Gazebo，本脚本不能单独作为 strict fresh 证据；正式运行必须由 `/data/a/scout_sim_replacement` 外层 strict wrapper 启停 fresh sim。 |
| `bench_check_profile_endpoint.py` | endpoint checker | 静态检查 profile baseline 的 endpoint/template/common limits 是否与旧仿真 canonical P2 设置一致。 | 不启动 ROS/Gazebo，不写 runtime 状态；失败时停止进入 strict fresh 正式流程。 |
| `bench_run_phase0_preflight.sh` | phase-0 gate | suite 启动前统一调用 `bench_preflight.py --dry-run`，检查方法 readiness/fairness/contract。 | 不启动 ROS/Gazebo；若 `SKIP_PHASE0_PREFLIGHT=true`，run 只能作为 diagnostics。 |
| `bench_validate_comparison_contracts.py` | contract validator | 检查 capability/profile YAML、CSV schema、runtime baseline common-limit/yaw tolerance、monitor topic contract、LT-DWA source-only gate 是否一致。 | 静态只读；用于防止对比算法配置和脚本漂移。 |
| `bench_validate_canonical_scenario.py` | scenario validator | 将 suite effective values 与 `canonical_fixed_path_p2.yaml` 对齐。 | 只读；canonical mismatch 时停止 formal/canonical profile-baseline 流程。 |
| `bench_write_freshness_evidence.py` | freshness evidence converter | 将 `/data/a/scout_sim_replacement` strict fresh manifest/batch meta 转成 `bench_check_freshness.py` 接受的新 evidence schema。 | 只转换证据文件；不推断 already-running sim 为 strict fresh。 |
| `run_fixed_path_paper_matrix.sh` | paper matrix 编排 | 编排 fixed-path internal ablation、external anchor、TEB/DWA 和可选 `B_accel`；默认提取窗口化 metrics。用于统一目录和 meta。 | 不启动/关闭 Gazebo；更适合 smoke 或手动 fresh-sim 单 case 调用，不适合直接连续跑 formal 数据。 |
| `run_fixed_path_critical_sweep.sh` | fixed-path critical scenario 编排 | 对一个或多个 fixed path 调用 paper matrix，默认包含 `P2_s_curve`。 | 继承 paper matrix 限制；路径/方法批量编排不等于 fresh-sim formal runner。 |
| `extract_fixed_path_paper_metrics.py` | 指标提取 | 从 fixed-path rosbag/meta 提取论文指标：success/stable、tracking、slosh height、cmd/odom speed、cmd acceleration、omega-rate、solver time、topic presence、evidence-chain meta 等。 | 离线分析脚本；不影响仿真环境。 |
| `run_p2p_baseline_smoke.sh` | P2P smoke | 用同一目标点和同一录包口径快速验证 `spmpc`、`teb`、`dwa`、`mpc`/`mpc_local_planner`。 | 前提是已经手动启动仿真与定位；主要用于 smoke，不作为 fixed-path 主线证据。 |
| `run_p2p_paper_supplement.sh` | P2P supplement 编排 | 运行点到点补充实验，默认比较 `B0/B_ours` 与 TEB/DWA，可选 `mpc_local_planner`。 | 不负责 fresh-sim；P2P 是论文补充，不替代 fixed-path critical scenario。 |
| `run_robustness_transfer_sweep.sh` | robustness/transfer 编排 | 统一鲁棒性/迁移实验目录和 meta，支持 nominal、yaw perturbation、`w_slosh_low/high` 等标签。 | yaw perturbation 需要用户先用对应 spawn yaw fresh 启动仿真；脚本只打印提示，不会自动改变仿真起点。 |
| `validate_slosh_monitor_against_visual.py` | slosh model validation | 将 `/slosh/height` 或 `/spmpc/slosh_height` 与视觉液面高度 CSV 对齐，计算 lag/correlation/error 并导出验证 CSV。 | 离线验证脚本；用于“模型/监控信号是否可信”的证据链前置环节。 |

## fixed-path 主矩阵与速度层级

paper-facing 主矩阵默认口径：

```text
internal ablation: B0 B_slosh B_ours
external anchors : B0 B_ours
external baselines: teb dwa
optional supplement: B_smooth / B_accel / mpc_local_planner
```

`mpc_local_planner` 只有在 `INCLUDE_MPC_LOCAL_PLANNER=true` 时纳入；其 standalone config 已按共同限幅对齐到 `max_vel_x=0.8`、`max_vel_theta=1.2`、`acc_lim_x=0.6`、`dec_lim_x=0.6`、`acc_lim_theta=1.2`。

速度层级分开记录：

```text
SPEED_TIER=fair_common      论文公平主表；共同硬限幅 0.8 / 1.2 / 0.6 / 1.2
SPEED_TIER=fast_diagnostic  提速诊断；不能直接混入 fair_common 主表
```

实物 B0/B_slosh bag 显示 runtime 硬上限已是 `v_max=0.8`，但 tracking 段实际均速约 `0.32–0.35m/s`。因此提速优先诊断 `v_ref` / 速度 profile / terminal window，而不是盲目上调 `v_max`。

## 常用 fixed-path 调用

### SPMPC 单 case：replay 固定 JSON

前提：已 fresh 启动仿真并等待定位恢复。`run_fixed_path_spmpc_suite.sh` 使用 `VARIANTS` / `OUT_ROOT`，run id 由脚本生成；不要用 `PLANNER_VARIANT` / `OUT_DIR` / `RUN_ID` 调这个 suite。

```bash
source /home/a/scout_ws/devel/setup.bash
cd /home/a/scout_ws

VARIANTS=B_slosh \
SPMPC_W_SLOSH=2.4 \
SPMPC_SOLVER_BACKEND=continuous_mpcc_acados \
PATH_SOURCE_MODE=replay \
PATH_ID=P2_s_curve \
PATH_FILE=/data/a/fixed_paths/sim/P2_s_curve.json \
OUT_ROOT=/data/a/spmpc_paper_compare/example_fixed_path \
RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_spmpc_suite.sh
```

高速诊断只用显式 opt-in，不改默认 `variants.yaml`：

```bash
SPEED_TIER=fast_diagnostic \
SPMPC_V_REF=0.60 \
VARIANTS=B_slosh \
SPMPC_SOLVER_BACKEND=continuous_mpcc_acados \
PATH_SOURCE_MODE=replay \
PATH_ID=P2_s_curve \
PATH_FILE=/data/a/fixed_paths/sim/P2_s_curve.json \
OUT_ROOT=/data/a/spmpc_paper_compare/fast_diag_vref060 \
RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_spmpc_suite.sh
```

该结果只能标成 `fast_diagnostic`；必须用 `extract_fixed_path_paper_metrics.py --phase windows` 的 `tracking/core` 速度字段确认行为，再决定是否进入 fresh-sim Gate。

### SPMPC 单 case：当前位置起点 + 固定终点

该模式先用 `template_fixed_path_generator.py` 从当前 `base_link` 位姿到固定 goal 生成 JSON，再用 `fixed_global_path_runner.py` replay 该 JSON 给 planner；适合排除“固定 JSON 起点和当前定位不一致”的影响。

```bash
source /home/a/scout_ws/devel/setup.bash
cd /home/a/scout_ws

VARIANTS=B0 \
SPMPC_SOLVER_BACKEND=continuous_mpcc_acados \
PATH_SOURCE_MODE=stable_goal \
PATH_ID=P2_s_curve_current_start \
GOAL_X=-3.6119120121002197 \
GOAL_Y=3.955589771270752 \
GOAL_YAW=2.584969730815858 \
PATH_TEMPLATE=s_curve \
START_HEADING=current \
FEASIBILITY_ANALYZE=true \
OUT_ROOT=/data/a/spmpc_omega_alpha_b0_p2_current_start_fixed \
RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_spmpc_suite.sh
```

### SPMPC 诊断：continuous direct-omega legacy B0

`continuous_mpcc_direct_omega_legacy` 只用于 B0 诊断，恢复 continuous OCP 中 `u[1]=omega` 直接下发角速度的旧式口径；不要把它作为 slosh formal 或 common-limit 论文结果。

```bash
source /home/a/scout_ws/devel/setup.bash
cd /home/a/scout_ws

VARIANTS=B0 \
SPMPC_SOLVER_BACKEND=continuous_mpcc_direct_omega_legacy \
PATH_SOURCE_MODE=stable_goal \
PATH_ID=P2_s_curve_current_start_direct_omega_legacy \
GOAL_X=-3.6119120121002197 \
GOAL_Y=3.955589771270752 \
GOAL_YAW=2.584969730815858 \
PATH_TEMPLATE=s_curve \
START_HEADING=current \
FEASIBILITY_ANALYZE=true \
OUT_ROOT=/data/a/spmpc_b0_p2_continuous_direct_omega_legacy_diag \
RECORD_SEC=60 \
RUN_TIMEOUT_SEC=60 \
SLOSH_MONITOR_ENABLE=false \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_spmpc_suite.sh
```

### 外部 baseline 单 case

```bash
source /home/a/scout_ws/devel/setup.bash
cd /home/a/scout_ws

BASELINE=teb \
PATH_ID=P2_s_curve \
PATH_FILE=/data/a/fixed_paths/sim/P2_s_curve.json \
OUT_ROOT=/data/a/spmpc_paper_compare/example_fixed_path \
RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_baseline_suite.sh
```

默认 `BASELINE=all` 只包含：

```text
teb dwa
```

如需把 `mpc_local_planner` 纳入，需要显式设置：

```bash
INCLUDE_MPC_LOCAL_PLANNER=true
```

### Profile baseline 单 case：Hamaguchi/Lim + common tracker

该入口只做 current-sim/smoke 编排：先用 canonical endpoint/template 生成 fixed path，再离线生成 profile CSV，最后用 `scout_local_planner` 的 `external_profile_mode=custom_csv` common tracker 跑同一路径。它不会启动/关闭 Gazebo，也不会宣称 strict fresh。

canonical P2 设置固定为：`GOAL_X=5.0`、`GOAL_Y=0.0`、`GOAL_YAW=0.0`、`PATH_TEMPLATE=s_curve`、`PATH_START_HEADING=current`、`PATH_AMPLITUDE_RATIO=0.18`、`PATH_MAX_AMPLITUDE=1.20`、`PATH_SMOOTH_ITERATIONS=3`，共同限幅为 `0.8/1.2/0.6/1.2`。

```bash
source /home/a/scout_ws/devel/setup.bash
cd /home/a/scout_ws

PROFILE_BASELINE=hamaguchi_profile \
PATH_SOURCE_MODE=stable_goal \
PATH_ID=P2_s_curve_current_start \
OUT_ROOT=/data/a/spmpc_paper_compare/profile_baseline_smoke \
RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_profile_baseline_suite.sh

PROFILE_BASELINE=lim_profile \
PATH_SOURCE_MODE=stable_goal \
PATH_ID=P2_s_curve_current_start \
OUT_ROOT=/data/a/spmpc_paper_compare/profile_baseline_smoke \
RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_profile_baseline_suite.sh
```

正式 strict fresh 结果必须由 `/data/a/scout_sim_replacement` 外层 wrapper 每 case 单独启停仿真并生成 freshness evidence 后，才能进入正式统计流程。

## P2P smoke 示例

前提：先启动仿真与定位。

```bash
source /home/a/scout_ws/devel/setup.bash
SIM_ENV=open USE_RVIZ=true \
  SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
  rosrun scout_local_planner launch_sim_nav_stack.sh
```

另开终端运行：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
cd /home/a/scout_ws

BASELINE=spmpc VARIANT=B_ours_anti OUT_DIR=/data/a/spmpc_baseline_smoke \
  bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

外部 baseline：

```bash
BASELINE=teb OUT_DIR=/data/a/spmpc_baseline_smoke \
  bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh

BASELINE=dwa OUT_DIR=/data/a/spmpc_baseline_smoke \
  bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

`mpc_local_planner` 需要先 source isolated install 空间：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/install_isolated_mpc/setup.bash
source /home/a/scout_ws/devel/setup.bash
cd /home/a/scout_ws

BASELINE=mpc OUT_DIR=/data/a/spmpc_baseline_smoke \
  bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

## 常用环境变量

### fixed-path suite

```text
PATH_ID / PATH_FILE       fixed path 标识与 JSON 路径（replay 模式需要 PATH_FILE）
PATH_SOURCE_MODE          replay（默认，回放既有 JSON）| stable_goal（当前位置起点 + 固定终点生成后回放）
OUT_ROOT                  输出根目录；run id 由 suite 自动生成
RECORD_SEC                录包时长，当前诊断通常用 60s
RUN_TIMEOUT_SEC           planner 启动后的 hard stop 秒数，默认 60；设为 0 可关闭
VARIANTS                  SPMPC variant 列表，例如 B0 / B_smooth / B_slosh / B_ours / B_accel
SPMPC_W_SLOSH             覆盖 variants/<variant>/w_slosh；负值表示使用 variants.yaml 默认值
SPMPC_V_REF               诊断用覆盖 variants/<variant>/v_ref；负值表示使用 variants.yaml 默认值
SPMPC_SOLVER_BACKEND      默认 continuous_mpcc_acados；可诊断用 continuous_mpcc_direct_omega_legacy（B0-only）
SPEED_TIER                fair_common（默认）| fast_diagnostic；写入 meta 和 metrics
LIMIT_PROFILE             默认 common_v0p8_w1p2_a0p6_alpha1p2
TARGET_V_MAX_MPS          共同线速度目标上限，默认 0.8
TARGET_OMEGA_MAX_RADPS    共同角速度目标上限，默认 1.2
TARGET_ACC_LIM_X_MPS2     共同线加速度目标上限，默认 0.6
TARGET_ACC_LIM_THETA_RADPS2 共同角加速度目标上限，默认 1.2
SLOSH_MONITOR_ENABLE      是否启动/记录 slosh monitor
SLOSH_RESET_BEFORE_RUN    run 前是否调用 /slosh/reset
EXPERIMENT_GROUP          证据链实验组
EVIDENCE_CHAIN_VERSION    证据链版本标签

stable_goal 额外变量：
GOAL_X / GOAL_Y / GOAL_YAW    固定终点（map frame）
PATH_TEMPLATE                 模板路径，当前常用 s_curve
START_HEADING                 current 表示路径初始切向使用当前车头朝向
FEASIBILITY_ANALYZE           是否对生成 JSON 进行曲率/omega_req 可行性检查
```

### P2P smoke

```text
BASELINE    spmpc | teb | dwa | mpc
VARIANT     SPMPC 内部 variant，例如 B0 / B_ours_anti
OUT_DIR     输出 bag、log、meta.yaml 的目录
RECORD_SEC  录包时长，默认 30
GOAL_X/Y/YAW 目标点
RUN_ID      可手动指定 run 名称
```

## 输出文件

常见输出：

```text
<OUT_DIR>/<RUN_ID>.bag
<OUT_DIR>/<RUN_ID>_meta.yaml
<OUT_DIR>/<RUN_ID>_planner.log
<OUT_DIR>/<RUN_ID>_rosbag.log
```

fixed-path 指标提取后常见输出（matrix 默认 `METRICS_PHASE=windows`）：

```text
fixed_path_metrics.csv
fixed_path_metrics_start.csv
fixed_path_metrics_tracking.csv
fixed_path_metrics_terminal.csv
fixed_path_metrics_reached.csv
fixed_path_metrics_motion.csv
fixed_path_metrics_core.csv
fixed_path_metrics_group_summary.csv
```

slosh cost 诊断/窄扫可能额外生成：

```text
slosh_cost_sweep_behavior_summary.csv
slosh_cost_sweep_behavior_summary.md
narrow_sweep_behavior_group_summary.csv
narrow_sweep_behavior_group_summary.md
```

## 当前使用提醒

- `variants.yaml` 里的 `B_slosh/B_ours` 默认 `w_slosh` 可能不等于最近 fresh-sim 窄扫使用的候选权重。正式实验不要无意中用 `SPMPC_W_SLOSH=-1.0` 继承默认值；应显式写明本轮权重。
- `B_accel` 是 non-slosh acceleration regularization baseline，用来区分“slosh-aware”与“只是加速度更小”。
- TEB/DWA/mpc_local_planner standalone common-limit 配置只对齐普通运动学限制；slosh 始终是 evaluation-only。
- `SPEED_TIER=fast_diagnostic` 和 `SPMPC_V_REF>=0` 是提速诊断入口，不等于 formal fair-common 结论。
- fixed-path critical scenario 是当前论文主线；P2P、robustness、transfer 是补充证据。
