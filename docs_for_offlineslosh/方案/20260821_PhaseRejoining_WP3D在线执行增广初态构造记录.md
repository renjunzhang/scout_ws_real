# Phase-Rejoining WP3D 在线执行增广初态构造记录

- 日期：2026-08-21
- 分支：`offline-slosh-plan-online-tracking`
- 前置工作：WP3C 独立离散 acados capsule
- 状态：代码与回归通过；formal 在线 solver 尚未放行

> 后续状态说明：本文记录的是 WP3D 当时的切片边界。当前工作树随后已完成 WP4 在线 factory/backend/parameter/gate 接线；请以 `20260821_PhaseRejoining_WP4在线执行增广求解器接线记录.md` 和实际代码为准。本文第 4 节保留作为历史未完成项快照，不代表当前代码状态。

## 1. 本轮闭合的边界

WP3C 的候选 capsule 已能消费完整 `ExecutionHorizonContext`，但在线侧只有 history-only robot/slosh rollout，不能提供 expected-publish epoch 的 actuator state 和双通道 pending-command buffer。本轮新增严格、无 ROS 的完整初态构造链：

```text
source-stamped robot/liquid state
  + successful-receipt published-command history
  + frozen ExecutionModelContract
  + valid PublishEpochEstimate
                 │
                 ▼
ExecutionModel::alignPublishedHistory()
                 │
                 ▼
expected-publish-epoch ExecutionAugmentedState
  = robot + liquid + actuator outputs + v/omega pending buffers
                 │
                 ▼
ExecutionHorizonContextBuilder
                 │
                 ▼
SolverInput.execution_horizon
```

这条路径没有把旧 `rolloutPublishedHistory()` 的 9 维输出包装成 formal context。旧 predictor 继续保留 development/off/monitor/fixed 兼容语义；新增路径只在 `execution_horizon_requested=true` 时运行。

## 2. 关键实现

### 2.1 完整历史对齐

`ExecutionModel::alignPublishedHistory()` 从 source epoch 传播到预计发布时刻。传播在每个真实历史命令的分通道延迟生效时刻切段，并同时受数值积分步长限制；线、角通道始终分别采样各自的 `t-d_v` 和 `t-d_omega`。

输出的 `ExecutionAugmentedState` 显式包含：

- 预计发布时刻的 robot/slosh state；
- 与 `robot.v/omega` 一致的两个 actuator output；
- 由 `integer_delay_steps+1` 分别确定基数的线、角 pending-command buffer；
- 从最近真实成功发布序列提取的 buffer 内容，而不是零或 held-command 填充。

以下情况直接失败，不做零命令回退：

- source/target epoch 无效或倒退；
- history 为空、非单调、含非有限命令或出现 target epoch 及其后的样本；
- source epoch 的物理延迟查询缺样；
- 任一通道 pending buffer 基数不足；
- 传播、状态有限性或 robot/actuator 一致性失败。

### 2.2 frozen-context builder

新增 `ExecutionHorizonContextBuilder`。它在配置阶段冻结一份带非空 id/hash 的 resolved `ExecutionModelContract` 和积分/过期边界；每周期只接受：

- 可按完整 cycle image 重算一致的有效 `PublishEpochEstimate`；
- 未预计 deadline miss 的 expected publish epoch；
- 与控制周期相同的 execution `dt`；
- 与冻结合同完全一致的 expected contract hash；
- 完整且未过期的真实发布历史；
- 有效 progress 和正的 liquid horizon 基数。

成功后统一生成 initial、physical-front、grid-front 和 terminal epoch，以及 `N_e=n_f+N_l` 基数。生成结果已通过 `DelayAugmentedPhaseDynamics::validateHorizonContext()`；hash、horizon cardinality 或 terminal epoch 单点变异均会失败。

### 2.3 `SolverInput` 接线与默认兼容

`ControlCycleInputPreparer` 新增显式 formal 配置和 opt-in request。只有 builder 成功时才把 active context 写入 `SolverInput.execution_horizon`；任一失败会返回 `ControlInputFailure::ExecutionHorizonContext`，不会调用后续 solver。

本轮没有：

- 修改 YAML/launch 默认值；
- 在 ROS wrapper 中配置或请求该 formal context；
- 把候选 capsule 加入 `SolverFactory`；
- 升级 capability mask；
- 修改现有 development solver 的 robot/slosh origin。

因此现有在线默认行为保持不变。

## 3. 测试证据

新增 12 项 C++ 测试，覆盖：

- 双通道不同 delay 下的 pending buffer 内容和 fractional-delay 生效切段；
- source epoch 到 expected-publish epoch 的 actuator/robot/slosh 对齐；
- incomplete/stale/target-epoch history；
- estimate off、contract hash mutation；
- builder 输出与 solver validator 的正向交叉验证；
- hash、horizon cardinality 和 terminal epoch mutation；
- `ControlCycleInputPreparer` 默认 inactive、显式 active 和失败前阻断。

完整回归结果：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner -j2
  PASS

catkin_test_results build/test_results/spmpc_local_planner
  Summary: 590 tests, 0 errors, 0 failures, 0 skipped

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py'
  Ran 96 tests, OK

git diff --check
  PASS
```

## 4. 尚未关闭的边界

WP3D 只闭合“完整执行增广初态可从真实发布历史构造并进入 typed `SolverInput`”这一代码切片。以下仍未完成：

- ROS/formal session 对 builder 的冻结配置和 runtime request；
- 候选 capsule 的在线 factory/backend 接入；
- formal nominal-relative stage/terminal cost 和参数；
- terminal 9D empirical gate 与 $\mathcal B^{\mathrm{exec}}$；
- 30 Hz P50/P95/max deadline 统计；
- 独立 execution/liquid plant 和非零延迟故障注入；
- Scout 的 $\widehat d_c,d_v,d_\omega,\tau,K$ held-out 标定与 hash 冻结。

所以 WP3、IMP-02/03/04/05、B0 和 formal release 均未整体关闭，状态继续为 **G0 NO-GO**。

## 5. 下一切片

下一步应让候选 solver 消费 formal artifact 的逐 stage nominal augmented state/control，建立 nominal-relative stage/terminal cost 和严格参数 image；随后再加入 terminal 9D gate 与 $\mathcal B^{\mathrm{exec}}$，完成 capability、随机一致性和 deadline 验收。在线 factory 切换必须晚于这些边界，不能仅因 WP3D 已能构造初态就提前启用。
