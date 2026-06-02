# spmpc_local_planner scripts

本目录放 SPMPC 自研 planner 的 smoke、录包和离线诊断脚本。
这些脚本服务于 `spmpc_local_planner` 本身，不用于启动外部 TEB/DWA/MPC baseline。

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
