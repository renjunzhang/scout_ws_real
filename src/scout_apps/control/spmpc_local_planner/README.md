# spmpc_local_planner

`spmpc_local_planner` 是新的规控一体 SloshPriorityMPC 实验包，和现有
`scout_local_planner` 并行，不替代当前已验证的 tracking MPC 主线。

## 当前阶段

当前落地到 Phase 3 基础 corridor / costmap / guidance 候选：

```text
/odom + /scout/global_path_fixed
  -> ReferencePath / progress projection
  -> RolloutSamplingSolver 占位 solver
  -> SloshDynamics horizon rollout
  -> corridor excess penalty / diagnostics
  -> static OccupancyGrid obstacle penalty
  -> center/left/right lightweight guidance candidates
  -> /spmpc/local_trajectory + /cmd_vel + /spmpc/* diagnostics
```

这不是完整 SQP/OSQP 规控一体 MPC；当前只用于验证包结构、ROS 数据流、
slosh 状态传播、slosh-aware 候选评分、静态 costmap 代价和轻量 guidance 候选接口。

## 配置结构

```text
config/planner/common.yaml
config/planner/variants.yaml
config/platforms/scout_mini.yaml
config/containers/tube_default.yaml
config/experiments/fixed_path.yaml
config/experiments/point_to_point.yaml
```

## 实验组

```text
B0        普通 integrated MPC 占位组
B_slosh   B0 + slosh horizon cost
B_smooth  B0 + smooth-priority cost
B_ours    B0 + slosh horizon cost + smooth-priority cost

B_slosh_linear  B_slosh 的显式 linear primitive 消融组
B_slosh_anti    B_slosh + anti-slosh primitives
B_ours_anti     B_ours + anti-slosh primitives
```

默认 `B_slosh/B_ours` 仍使用 `primitive_mode=linear`，用于保持 Phase 3 回归口径。
`B_slosh_anti` 只用于验证 pre-turn-brake / mid-valley / jerk-limited recovery
这类候选模板是否比线性 start/end 候选更能让 slosh cost 改变行为。

## 启动

先单独编译新包：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner
source devel/setup.bash
```

固定路径 smoke：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch planner_variant:=B0
```

点到点 smoke：

```bash
roslaunch spmpc_local_planner spmpc_point_to_point.launch planner_variant:=B0
```

## 诊断话题

```text
/spmpc/status
/spmpc/controller_variant
/spmpc/experiment_mode
/spmpc/local_trajectory
/spmpc/debug/progress_s
/spmpc/debug/slosh_state
/spmpc/slosh_horizon_summary
/spmpc/corridor
/spmpc/guidance
/spmpc/primitive
/spmpc/solver_time_ms
/spmpc/cost_breakdown
```

不要复用 `/mpc/cost_breakdown`，避免污染 `scout_local_planner` 既有分析链路。

`J_contour` 表示贴近当前 guidance 线的软跟踪误差；`J_corridor` 只统计超过 reference corridor 半宽后的超限惩罚。`obstacle_enable=true` 时使用 `/map` 的 `nav_msgs/OccupancyGrid` 静态代价计算 `J_obstacle`。`homotopy_enable=true` 时求解器会在 center/left/right 三个 lateral guidance 候选中选总代价最低的一条；这是轻量 guidance 候选，不是 `mpc_planner` 的完整动态 homotopy/topology 搜索。

## 录包

```bash
OUT_DIR=/tmp/spmpc_bags NAME=spmpc_phase2_smoke \
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_experiment.sh
```

脚本会记录 `/spmpc/debug/slosh_state`、`/spmpc/slosh_horizon_summary`、`/spmpc/corridor`、`/spmpc/guidance` 和 `/spmpc/primitive`。

## Phase 3 Smoke

先手动启动仿真并等待约 30s：

```bash
source devel/setup.bash
SIM_ENV=open USE_RVIZ=true \
  SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
  rosrun scout_local_planner launch_sim_nav_stack.sh
```

固定路径回归：

```bash
PHASE3_MODE=fixed_path VARIANT=B0 OUT_DIR=/data/a/spmpc_phase3_fixed_smoke \
  bash src/scout_apps/control/spmpc_local_planner/scripts/phase3_smoke.sh
```

点到点 obstacle/guidance 链路：

```bash
PHASE3_MODE=point_to_point VARIANT=B0 OUT_DIR=/data/a/spmpc_phase3_p2p_smoke \
  bash src/scout_apps/control/spmpc_local_planner/scripts/phase3_smoke.sh
```

每次切换 variant 前仍建议重启仿真，保证起点一致。

anti-slosh primitive 消融：

```bash
# 第一次仿真
VARIANT=B_slosh_linear OUT_DIR=/data/a/spmpc_primitive_smoke \
  bash src/scout_apps/control/spmpc_local_planner/scripts/phase3_smoke.sh

# 重启仿真并等待 30s 后
VARIANT=B_slosh_anti OUT_DIR=/data/a/spmpc_primitive_smoke \
  bash src/scout_apps/control/spmpc_local_planner/scripts/phase3_smoke.sh

python3 src/scout_apps/control/spmpc_local_planner/scripts/analyze_b0_bslosh_compare.py \
  /data/a/spmpc_primitive_smoke B_slosh_linear B_slosh_anti
```
