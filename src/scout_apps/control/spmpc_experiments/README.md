# spmpc_experiments

`spmpc_experiments` 只放实验调度，不放算法实现。

```text
spmpc_local_planner           自研 SPMPC / B0 / B_slosh / B_smooth / B_ours
baseline_local_planner_runner 独立加载 nav_core plugin, 让外部 baseline 脱离 move_base
teb_local_planner             外部 baseline, ROS1 nav_core plugin
dwa_local_planner             外部 supplementary baseline, 已从 ROS navigation 本地源码化
mpc_local_planner             外部 MPC baseline, 已放入 src/scout_apps/control, 需补齐 corbo/control_box_rst 依赖后实跑

spmpc_experiments    统一 launch / config / recording / smoke scripts
```

## 对比算法文件分类入口

对比算法与 benchmark 文件先看：[`README_compare_algorithms.md`](README_compare_algorithms.md)。当前按四层分类，方便横向比较：

```text
config/benchmark/          公平性规则、common limits、freshness、capability matrix、主表准入
config/baselines/          TEB / DWA / mpc_local_planner / official LT-DWA wrapper runtime configs
config/profile_baselines/  Hamaguchi / Lim supplementary profile baseline configs
scripts/                   suites、preflight、freshness evidence、endpoint check、metrics
../scout_profile_baselines/ Hamaguchi / Lim offline profile generators 与独立 helper
../scout_local_planner/     common external-profile tracker 与旧 rosrun wrapper
../lt_dwa_official_wrapper/ official LT-DWA ROS Noetic core wrapper
../../../../third_party/LT_DWA/
                           LT-DWA upstream source-only vendor，保留给 official wrapper 编译使用
```

关键边界：`spmpc_experiments` 只做实验调度和 benchmark gate，不放 planner/OCP 算法实现；Hamaguchi/Lim 的 generator 实现在 `scout_profile_baselines`，runtime common tracker 仍在 `scout_local_planner`，它们是 supplementary profile baseline，不是 online local-planner 同层主表方法。

## 当前可用仿真入口

先启动仿真和定位：

```bash
source devel/setup.bash
SIM_ENV=open USE_RVIZ=true \
  SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
  rosrun scout_local_planner launch_sim_nav_stack.sh
```

然后在另一个终端运行 baseline smoke：

```bash
source devel/setup.bash
cd /home/a/scout_ws

BASELINE=spmpc VARIANT=B_ours_anti OUT_DIR=/data/a/spmpc_baseline_smoke \
  bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

```bash
BASELINE=teb OUT_DIR=/data/a/spmpc_baseline_smoke \
  bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

```bash
BASELINE=dwa OUT_DIR=/data/a/spmpc_baseline_smoke \
  bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

```bash
BASELINE=mpc OUT_DIR=/data/a/spmpc_baseline_smoke \
  bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

## 重要边界

- `TEB/DWA/mpc_local_planner` 本体仍是 `nav_core` 插件。
- `baseline_local_planner_runner` 把这些插件包装成独立 ROS node，直接订阅 path/goal 并发布 `/cmd_vel`。
- `SPMPC` 当前也是独立 ROS node。这样点到点 smoke 不再依赖 `move_base`。
- 当前 runner 的 goal-only 模式会生成一条直线路径；正式 fixed-path 对比时应输入同一条 `/scout/global_path_fixed`。

## mpc_local_planner 状态

`mpc_local_planner` 已 clone 到：

```text
src/scout_apps/control/mpc_local_planner
```

但该包依赖 `control_box_rst/corbo`。`control_box_rst` 已 clone 到：

```text
src/scout_apps/control/control_box_rst
```

它是 plain CMake 包，不能用普通全量 `catkin_make` 和 catkin 包混编。
当前已经通过 isolated 路线编译并安装到：

```text
install_isolated_mpc
```

构建命令：

```bash
source /opt/ros/noetic/setup.bash
catkin_make_isolated --install --force-cmake \
  --only-pkg-with-deps mpc_local_planner \
  --install-space install_isolated_mpc \
  --devel devel_isolated_mpc \
  --build build_isolated_mpc
```

运行 `BASELINE=mpc` 前需要 source isolated install 空间和主工作区：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/install_isolated_mpc/setup.bash
source /home/a/scout_ws/devel/setup.bash
```

当前 `control_box_rst` 和 `mpc_local_planner` 根目录不再放 `CATKIN_IGNORE`。
因此普通全量 `catkin_make` 不再作为推荐构建入口；主工作区日常编译请使用白名单，
`mpc_local_planner` 单独使用上述 isolated 路线。

## LT-DWA 状态

当前 `lt_dwa` baseline id 已切到官方 LT-DWA ROS Noetic core wrapper：

```text
src/scout_apps/control/lt_dwa_official_wrapper/
```

官方 source-only vendor 保留在：

```text
third_party/LT_DWA/
```

运行 official core 前需要确保 runtime `local_planner` overlay 在 `ROS_PACKAGE_PATH` 前面：

```bash
export SCOUT_WS_ROOT=/home/a/scout_ws
export ROS_PACKAGE_PATH=$SCOUT_WS_ROOT/tools/lt_dwa/local_planner_runtime:$ROS_PACKAGE_PATH
```

旧 `lt_dwa_adapter` / `lt_dwa_v2_adapter` 不再作为 active benchmark 路径使用；`third_party/LT_DWA` 不能删除，也不能 symlink 到 catkin `src/`。

## DWA 状态

`dwa_local_planner` 已从 `ros-planning/navigation` clone 到：

```text
src/scout_apps/control/navigation/dwa_local_planner
```

navigation 仓库中除 `dwa_local_planner` 外的其它包均放置 `CATKIN_IGNORE`，
避免覆盖当前系统 Noetic navigation 依赖。
