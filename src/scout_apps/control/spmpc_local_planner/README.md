# spmpc_local_planner

`spmpc_local_planner` 是新的规控一体 SloshPriorityMPC 实验包，和现有
`scout_local_planner` 并行，不替代当前已验证的 tracking MPC 主线。

## 当前阶段

当前落地到 Phase 2 最小闭环：

```text
/odom + /scout/global_path_fixed
  -> ReferencePath / progress projection
  -> RolloutSamplingSolver 占位 solver
  -> SloshDynamics horizon rollout
  -> /spmpc/local_trajectory + /cmd_vel + /spmpc/* diagnostics
```

这不是完整 SQP/OSQP 规控一体 MPC；当前只用于验证包结构、ROS 数据流、
slosh 状态传播和 slosh-aware 候选评分接口。

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
```

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
/spmpc/solver_time_ms
/spmpc/cost_breakdown
```

不要复用 `/mpc/cost_breakdown`，避免污染 `scout_local_planner` 既有分析链路。

## 录包

```bash
OUT_DIR=/tmp/spmpc_bags NAME=spmpc_phase2_smoke \
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_experiment.sh
```

脚本会记录 `/spmpc/debug/slosh_state` 和 `/spmpc/slosh_horizon_summary`。
