# spmpc_local_planner scripts

本目录放 SPMPC 自研 planner 的 smoke、录包和离线诊断脚本。
这些脚本服务于 `spmpc_local_planner` 本身，不用于启动外部 TEB/DWA/MPC baseline。

## record_spmpc_full_rgb_bag.sh

实物 SPMPC 黑匣子录包脚本。它只负责录制 topic 和保存 metadata，**不发送 `/cmd_vel`、不发送目标点、不启动/停止 planner**。

主要用途：正式实物 run 前先启动 recorder，把事后分析可能需要的证据一次录全，包括：

- `/cmd_vel`、`/odom`、`/tf`、`/map`、固定路径/goal；
- `/spmpc/status`、`/spmpc/solver_backend`、`/spmpc/controller_variant`；
- `/spmpc/debug/effective_config`、`/spmpc/cost_breakdown`、`/spmpc/slosh_horizon_summary`；
- `/spmpc/debug/raw_state`、`/spmpc/debug/predicted_state`、`/spmpc/debug/solver_input_state`、`/spmpc/debug/command_intervention`；
- `/spmpc/debug/predicted_horizon`、`/spmpc/debug/pre_solve_snapshot`，用于完整预测时域和 actual/zero replay；
- 相机、scan、standalone `/slosh/*`、在线 `/liquid/*` 等可选话题。

典型用法：

```bash
RUN_LABEL=Bslosh_delay_off_run01 \
VARIANT=B_slosh \
RECORD_SEC=60 \
RECORD_RGB=false \
OUT_DIR=/home/geist/slosh_bags/real/20260704_fixed_path_compare/B_slosh \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

输出包括 `.bag`、`*_info.txt`、`*_rosparam.yaml`、`*_recorded_topics.txt`、`*_selected_topics_not_recorded.txt`、`*_topic_info/` 等 sidecar。

### replay 话题 smoke 检查

启动 `continuous_mpcc_acados` 并进入正常求解后，正式录制前至少检查一次：

```bash
rostopic echo -n 1 /spmpc/debug/predicted_horizon
rostopic echo -n 1 /spmpc/debug/pre_solve_snapshot
```

必须满足：

- 两条消息的 `valid` 均为 `true`，`backend` 为 `continuous_mpcc_acados`；
- `horizon_steps: 60`；预测时域有 61 个状态样本和 60 个控制样本；
- pre-solve 快照的 `state_width: 10`、`control_width: 3`；
- B0 为 `parameter_width: 23`、`stage_parameters` 共 1403 个数；含 slosh 的 variant 为 `parameter_width: 32`、共 1952 个数；
- `initial_guess_states` 对应 `61 x 10`，`initial_guess_controls` 对应 `60 x 3`；
- 第二个及后续有效求解周期通常应有 `have_previous_solution: true`；
- `primal_guess_only: true` 是当前预期值，表示未录对偶变量和内部 SQP memory。

短录一包后再确认两个话题实际进入 bag：

```bash
rosbag info /path/to/run.bag | rg '/spmpc/debug/(predicted_horizon|pre_solve_snapshot)'
```

若现场机没有 `rg`，可把最后一段替换为 `grep -E`。上述检查只证明录制接口和显式输入完整；actual replay 还必须在冻结容差内复现在线 solver status、第一控制量和 raw command，才能用于正式反事实分析。

## run_spmpc_real_fixed_path_trial.sh

实物 SPMPC fixed-path 单次试验一键脚本。前提是实物传感器/定位/底盘栈已经启动。它支持两种路径来源：

```text
PATH_SOURCE_MODE=generate：按当前位姿生成路径，兼容旧用法
PATH_SOURCE_MODE=replay：回放冻结 JSON，通过起点位置/航向门控后启动
```

非 pilot 默认仍为 `generate`，目标点固定为 2026-07-02 实物 bag 中恢复出的终点：`GOAL_X=-5.424`、`GOAL_Y=-4.736`、`GOAL_YAW=0.0`。`PILOT_MODE=true` 时默认切换到 `replay`，默认读取 `${HOME}/fixed_paths/real/${DATE}_spmpc_parameter_pilot/H0_weight_pilot.json`；起点门控默认放宽为 `0.08 m / 0.15 rad`，也可用环境变量覆盖。

历史实物机工作空间为 `/home/geist/scout_ws`，旧路径文件位于 `/home/geist/fixed_paths/real/<DATE>_fixed_path_compare/fixed_s_curve_compare.json`。旧脚本会逐 run 覆盖这个文件，因此不能把某个历史日期目录下的最后一份 JSON 自动视为冻结路径。下一次现场先在地面起点标记处运行一次 `PATH_SOURCE_MODE=generate` 的 H0 path-freeze smoke，再让全部权重 pilot replay 这份新 JSON。

首次生成 H0：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=20260718 \
PILOT_MODE=true \
PILOT_METHOD=B0 \
PATH_SOURCE_MODE=generate \
RUN_LABEL=PF_PATH_FREEZE_H0_B0_smoke01 \
RUN_OUT_DIR=/home/geist/slosh_bags/real/20260718_spmpc_parameter_pilot/PF/PATH/H0_C1/B0 \
CMD_TOPIC=/cmd_vel \
PILOT_RECORD_RGB=false \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
V_REF=0.20 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

成功后路径应保存在：

```text
/home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json
```

pilot generate 默认拒绝覆盖已经存在的 H0。后续直接使用 replay；只有明确作废旧路径并准备重新冻结时，才可设置 `ALLOW_PILOT_PATH_OVERWRITE=true`。

pilot 默认强制 `RECORD_RGB=false`，不向 bag 写入原始彩色图像；如果确实要做单独的视觉确认，必须显式设置 `PILOT_RECORD_RGB=true`。无 RGB pilot 只能用于内部 slosh model、跟踪、完成时间、求解实时性和执行层指标的模型侧参数筛选，不能单独证明真实液面高度改善。

`PILOT_METHOD` 提供现场快速参数映射：

| `PILOT_METHOD` | `VARIANT` | `W_SLOSH` |
| --- | --- | ---: |
| `B0` | `B0` | `0` |
| `Bsmooth` | `B_smooth` | `0` |
| `W1` / `W2` / `W5` / `W10` | `B_slosh` | `1` / `2` / `5` / `10` |
| 任意 `W<number>`，如 `W3.5` | `B_slosh` | 对应数值 |

参数 pilot 推荐命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

PILOT_MODE=true \
PILOT_METHOD=W2 \
DATE=20260718 \
PATH_FILE=/home/geist/fixed_paths/real/20260718_spmpc_parameter_pilot/H0_weight_pilot.json \
RUN_LABEL=PF_WS_H0_C1_W2_b01_r01 \
START_POS_TOL=0.08 \
START_YAW_TOL=0.15 \
CMD_TOPIC=/cmd_vel \
PILOT_RECORD_RGB=false \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
V_REF=0.20 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

下一次只需修改 `PILOT_METHOD` 和 `RUN_LABEL`。例如临时检查 `w_slosh=3.5` 时使用 `PILOT_METHOD=W3.5`。脚本会把 pilot、路径模式、起点容差、`V_REF`、`W_SLOSH` 和实际 planner 命令同时写入 one-click 与 recorder sidecar。

`RECORD_SEC` 默认 60，且大于 `MAX_RECORD_SEC` 时会被截断；若希望录 90 秒，应同时设置 `RECORD_SEC=90 MAX_RECORD_SEC=90`。非 pilot 为保持旧用法仍默认 `DELAY_PHASE_MODE=off`；pilot 根据 0705/0706 实物稳定口径默认使用 `fixed_closed_loop 0.15/0.22`、`RECORDER_STARTUP_SEC=8` 和 `RECORD_TOPIC_INFO=false`，命令中仍建议显式写出以便现场复核。

旧的按当前位姿生成路径用法保持不变：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

ALG=B_slosh \
RUN_LABEL=Bslosh_delay_off_run01 \
CMD_TOPIC=/cmd_vel \
DELAY_PHASE_MODE=off \
DELAY_PHASE_LINEAR_DELAY_SEC=-1.0 \
DELAY_PHASE_ANGULAR_DELAY_SEC=-1.0 \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

该脚本只覆盖 SPMPC 内部消融，不用于外部 baseline。

## summarize_spmpc_real_trial.py

实物 SPMPC bag 离线 summary 脚本。支持传入单个 `.bag` 或 run 目录，读取 rosbag 与 recorder / one-click sidecar，输出：

```text
${bag_stem}_summary.json
${bag_stem}_summary.md
```

它按 `Float32MultiArray.layout.dim[0].label` 动态解析字段，避免依赖硬编码索引。旧 bag 缺少新 debug topic 时不会失败，而是在 summary 中列出 red flags。

典型用法：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

python3 src/scout_apps/control/spmpc_local_planner/scripts/summarize_spmpc_real_trial.py \
  /home/geist/slosh_bags/real/20260704_fixed_path_compare/B_slosh/Bslosh_delay_off_run01.bag
```

## compare_b0_bslosh_smoke.sh

Phase 2/3 内部消融 smoke 脚本。用于在仿真中跑一个 SPMPC variant，
并录制 `/spmpc/*`、`/cmd_vel`、`/odom` 等诊断话题。

典型用法：

```bash
source /home/a/scout_ws/devel/setup.bash
cd /home/a/scout_ws

VARIANT=B0 OUT_DIR=/data/a/spmpc_compare \
  bash src/scout_apps/control/spmpc_local_planner/scripts/compare_b0_bslosh_smoke.sh

VARIANT=B_slosh OUT_DIR=/data/a/spmpc_compare \
  bash src/scout_apps/control/spmpc_local_planner/scripts/compare_b0_bslosh_smoke.sh
```

## analyze_b0_bslosh_compare.py

离线分析 smoke bag，比较不同 variant 的控制输出、预测晃动、cost breakdown、
primitive 选择和 progress 单调性。

```bash
python3 src/scout_apps/control/spmpc_local_planner/scripts/analyze_b0_bslosh_compare.py \
  /data/a/spmpc_compare B0 B_slosh B_smooth B_ours
```

默认统计 active solver window，排除 `GOAL_REACHED` 后的全零段。

## phase3_smoke.sh

Phase 3 corridor / obstacle / guidance smoke 脚本。用于验证 Phase 3
点亮后 planner 是否能在 fixed-path 或 point-to-point 场景中闭环运行。

```bash
PHASE3_MODE=p2p VARIANT=B0 OUT_DIR=/data/a/spmpc_phase3_p2p_smoke \
  bash src/scout_apps/control/spmpc_local_planner/scripts/phase3_smoke.sh

PHASE3_MODE=fixed_path VARIANT=B0 OUT_DIR=/data/a/spmpc_phase3_fixed_smoke \
  bash src/scout_apps/control/spmpc_local_planner/scripts/phase3_smoke.sh
```

## phase4_fixed_path_run.sh

Phase 4 fixed-path 实物对比实验脚本。它只负责固定路径生成、目标发送、
启动 `spmpc_fixed_path.launch` 和录包；不替代实物传感器/定位/底盘启动脚本。

实物前置启动仍使用现有传感器栈，例如：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws
src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

另开终端运行 Phase 4：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

VARIANT=B_ours_anti \
OUT_DIR=/home/geist/slosh_bags/real/20260602_spmpc_phase4 \
GOAL_X=7.164488315582275 \
GOAL_Y=9.307367324829102 \
GOAL_YAW=1.0808 \
bash src/scout_apps/control/spmpc_local_planner/scripts/phase4_fixed_path_run.sh
```

常用环境变量：

```text
VARIANT      B0 / B_slosh / B_smooth / B_ours / B_slosh_anti / B_ours_anti
OUT_DIR      输出 bag、log、meta.yaml 的目录
RECORD_SEC   录包时长，默认 45
GOAL_X/Y/YAW 固定目标点
PATH_FILE    固定路径 JSON 输出位置
RECORD_CAMERA true/false
RUN_ID       可手动指定 run 名称
```

## record_spmpc_experiment.sh

手动录包备用入口。适合 planner 已经手动启动，只需要记录 SPMPC 诊断、
相机、odom、cmd_vel 和 slosh 话题的场景。

```bash
OUT_DIR=/home/geist/slosh_bags/real/20260602_spmpc_manual \
RUN_ID=B_ours_anti_run01 \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_experiment.sh
```
