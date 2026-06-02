# spmpc_experiments scripts

本目录放对比实验调度脚本。脚本只负责启动不同 planner 的 smoke run、
发送目标和录包，不实现任何规划/控制算法。

## run_p2p_baseline_smoke.sh

点到点仿真 smoke 脚本，用同一目标点和同一录包口径验证：

- `spmpc`
- `teb`
- `dwa`
- `mpc` / `mpc_local_planner`

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

常用环境变量：

```text
BASELINE    spmpc | teb | dwa | mpc
VARIANT     SPMPC 内部 variant，例如 B0 / B_ours_anti
OUT_DIR     输出 bag、log、meta.yaml 的目录
RECORD_SEC  录包时长，默认 30
GOAL_X/Y/YAW 目标点
RUN_ID      可手动指定 run 名称
```

输出：

```text
<OUT_DIR>/<RUN_ID>.bag
<OUT_DIR>/<RUN_ID>_meta.yaml
<OUT_DIR>/<RUN_ID>_planner.log
<OUT_DIR>/<RUN_ID>_rosbag.log
```
