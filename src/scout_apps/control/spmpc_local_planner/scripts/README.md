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
- 相机、scan、standalone `/slosh/*`、在线 `/liquid/*` 等可选话题。

典型用法：

```bash
RUN_LABEL=Bours_delay_080_050_run01 \
VARIANT=B_ours \
RECORD_SEC=60 \
RECORD_RGB=false \
OUT_DIR=/home/geist/slosh_bags/real/20260703_fixed_path_compare/B_ours \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

输出包括 `.bag`、`*_info.txt`、`*_rosparam.yaml`、`*_recorded_topics.txt`、`*_selected_topics_not_recorded.txt`、`*_topic_info/` 等 sidecar。

## run_spmpc_real_fixed_path_trial.sh

实物 SPMPC fixed-path 单次试验一键脚本。前提是实物传感器/定位/底盘栈已经启动；脚本负责把下面流程合并成一次命令：

```text
启动 fixed-path generator -> 启动黑匣子录包 -> 发送固定终点 -> 等待 fixed path -> 启动 SPMPC variant -> 60s 或 Ctrl-C 后清理
```

默认目标点固定为 2026-07-02 实物 bag 中恢复出的终点：`GOAL_X=-5.424`、`GOAL_Y=-4.736`、`GOAL_YAW=0.0`。`RECORD_SEC` 默认 60，且大于 60 或非法时会强制回到 60。

典型用法：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

ALG=B_ours \
RUN_LABEL=Bours_delay_080_050_run01 \
CMD_TOPIC=/cmd_vel \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.08 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.05 \
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
  /home/geist/slosh_bags/real/20260703_fixed_path_compare/B_ours/Bours_delay_080_050_run01.bag
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
