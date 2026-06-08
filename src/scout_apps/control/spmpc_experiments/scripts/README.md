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

## 脚本索引

| 脚本 | 类型 | 主要用途 | fresh-sim 注意事项 |
|---|---|---|---|
| `run_fixed_path_spmpc_suite.sh` | fixed-path run suite | 运行 SPMPC fixed-path case，支持 `planner_variant`、`w_slosh`、solver backend、shared command acceleration limit、slosh monitor meta/reset。 | 只负责单次/一组 planner run；正式实验建议外层每个 case 重启仿真。 |
| `run_fixed_path_baseline_suite.sh` | fixed-path baseline suite | 运行 TEB/DWA/可选 `mpc_local_planner` fixed-path baseline，并写入 `slosh_eval_only`、`slosh_feedback_forbidden` 等公平性 meta。 | baseline 可以记录 slosh 作为评价信号，但不得作为控制输入；正式实验同样需要每 case fresh sim。 |
| `run_fixed_path_paper_matrix.sh` | paper matrix 编排 | 编排 fixed-path internal ablation、external anchor、TEB/DWA 和可选 `B_accel`。用于统一目录和 meta。 | 不启动/关闭 Gazebo；更适合 smoke 或手动 fresh-sim 单 case 调用，不适合直接连续跑 formal 数据。 |
| `run_fixed_path_critical_sweep.sh` | fixed-path critical scenario 编排 | 对一个或多个 fixed path 调用 paper matrix，默认包含 `P2_s_curve`。 | 继承 paper matrix 限制；路径/方法批量编排不等于 fresh-sim formal runner。 |
| `extract_fixed_path_paper_metrics.py` | 指标提取 | 从 fixed-path rosbag/meta 提取论文指标：success/stable、tracking、slosh height、cmd acceleration、omega-rate、solver time、topic presence、evidence-chain meta 等。 | 离线分析脚本；不影响仿真环境。 |
| `run_p2p_baseline_smoke.sh` | P2P smoke | 用同一目标点和同一录包口径快速验证 `spmpc`、`teb`、`dwa`、`mpc`/`mpc_local_planner`。 | 前提是已经手动启动仿真与定位；主要用于 smoke，不作为 fixed-path 主线证据。 |
| `run_p2p_paper_supplement.sh` | P2P supplement 编排 | 运行点到点补充实验，默认比较 `B0/B_ours` 与 TEB/DWA，可选 `mpc_local_planner`。 | 不负责 fresh-sim；P2P 是论文补充，不替代 fixed-path critical scenario。 |
| `run_robustness_transfer_sweep.sh` | robustness/transfer 编排 | 统一鲁棒性/迁移实验目录和 meta，支持 nominal、yaw perturbation、`w_slosh_low/high` 等标签。 | yaw perturbation 需要用户先用对应 spawn yaw fresh 启动仿真；脚本只打印提示，不会自动改变仿真起点。 |
| `validate_slosh_monitor_against_visual.py` | slosh model validation | 将 `/slosh/height` 或 `/spmpc/slosh_height` 与视觉液面高度 CSV 对齐，计算 lag/correlation/error 并导出验证 CSV。 | 离线验证脚本；用于“模型/监控信号是否可信”的证据链前置环节。 |

## 常用 fixed-path 调用

### SPMPC 单 case

前提：已 fresh 启动仿真并等待定位恢复。

```bash
source /home/a/scout_ws/devel/setup.bash
cd /home/a/scout_ws

PLANNER_VARIANT=B_slosh \
SPMPC_W_SLOSH=2.4 \
SPMPC_SOLVER_BACKEND=continuous_mpcc_acados \
PATH_ID=P2_s_curve \
PATH_FILE=/data/a/fixed_paths/sim/P2_s_curve.json \
OUT_DIR=/data/a/spmpc_paper_compare/example_fixed_path \
RUN_ID=P2_s_curve_spmpc_B_slosh_w2p4_run01 \
RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_spmpc_suite.sh
```

### 外部 baseline 单 case

```bash
source /home/a/scout_ws/devel/setup.bash
cd /home/a/scout_ws

BASELINE=teb \
PATH_ID=P2_s_curve \
PATH_FILE=/data/a/fixed_paths/sim/P2_s_curve.json \
OUT_DIR=/data/a/spmpc_paper_compare/example_fixed_path \
RUN_ID=P2_s_curve_teb_run01 \
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
PATH_ID / PATH_FILE       fixed path 标识与 JSON 路径
OUT_DIR / RUN_ID          输出目录与 run 名称
RECORD_SEC                录包时长，当前诊断通常用 60s
PLANNER_VARIANT           SPMPC variant，例如 B0 / B_smooth / B_slosh / B_ours / B_accel
SPMPC_W_SLOSH             覆盖 variants/<variant>/w_slosh；负值表示使用 variants.yaml 默认值
SPMPC_SOLVER_BACKEND      默认 continuous_mpcc_acados
SLOSH_MONITOR_ENABLE      是否启动/记录 slosh monitor
SLOSH_RESET_BEFORE_RUN    run 前是否调用 /slosh/reset
EXPERIMENT_GROUP          证据链实验组
EVIDENCE_CHAIN_VERSION    证据链版本标签
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

fixed-path 指标提取后常见输出：

```text
fixed_path_metrics.csv
fixed_path_metrics_pre_terminal.csv
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
- TEB/DWA common-limit 配置只对齐普通运动学限制；slosh 始终是 evaluation-only。
- fixed-path critical scenario 是当前论文主线；P2P、robustness、transfer 是补充证据。
