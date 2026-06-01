# spmpc_local_planner

`spmpc_local_planner` 是新的规控一体 SloshPriorityMPC 实验包，和现有
`scout_local_planner` 并行，不替代当前已验证的 tracking MPC 主线。

## 当前阶段

当前只落地 Phase 1 最小骨架：

```text
/odom + /scout/global_path_fixed
  -> ReferencePath / progress projection
  -> RolloutSamplingSolver 占位 solver
  -> /spmpc/local_trajectory + /cmd_vel + /spmpc/* diagnostics
```

这不是完整 SQP/OSQP 规控一体 MPC，只用于验证包结构、ROS 数据流和诊断接口。

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
B_slosh   预留 slosh cost 组
B_smooth  预留 smooth-priority 组
B_ours    预留 slosh + smooth 主方法组
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
/spmpc/solver_time_ms
/spmpc/cost_breakdown
```

不要复用 `/mpc/cost_breakdown`，避免污染 `scout_local_planner` 既有分析链路。
