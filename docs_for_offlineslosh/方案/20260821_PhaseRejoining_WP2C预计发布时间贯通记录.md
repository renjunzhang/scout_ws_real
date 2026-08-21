# Phase-Rejoining WP2C 预计发布时间贯通记录

- 日期：2026-08-21
- 分支：`offline-slosh-plan-online-tracking`
- 入口基线：`de2e7b4f`（WP2B 统一执行增广模型）
- 对应缺口：IMP-02 的在线 typed 接线切片
- 结论：WP2C 通过；IMP-02、整个 WP2 和 B0 尚未关闭；formal 仍为 G0 NO-GO

## 1. 本切片完成的时间贯通

同一个 `PublishEpochEstimate` 现在贯穿：

```text
cycle start
  -> PublishLatencyModel
  -> ControlCycleInputPreparer
  -> history prediction + PhaseClock
  -> SolverInput.publish_epoch_estimate / cycle_timing
  -> ControlCycleEngine
  -> publication audit
```

当 estimate 有效时，prediction evaluation epoch 由预计发布时间唯一决定：

$$
t_{\mathrm{eval}}=\widehat t_{\mathrm{pub}}=t_c+\widehat d_c.
$$

history predictor 从 source-stamped common epoch 对齐到该时刻，再按统一 `ExecutionModel` 传播到 physical execution front。ROS 的 PhaseClock 使用同一个 evaluation epoch，solver 同时收到 typed estimate 与展开后的 timing 字段。

当 `publish_timing.enabled=false` 时，estimate 保持 `ESTIMATE_OFF`，prediction 和 PhaseClock 继续使用显式求解前 evaluation epoch。因此默认配置和既有运行行为不变。

## 2. 一致性与 fail-closed

新增 `publishEpochEstimateMatchesCycle()`，要求 estimate 的完整 typed image 可由当前周期严格重算，包括：

- `cycle_id`、周期开始时刻和周期长度；
- nominal publish deadline；
- `estimated_dc_sec` 和 expected publish timestamp；
- expected deadline-miss 状态；
- `valid` 和稳定 typed status。

`ControlCycleInputPreparer` 在任何 robot-state lookup 前拒绝不一致 image，状态为 `PUBLISH_EPOCH_CONTRACT_MISMATCH`；`ControlCycleEngine` 在 solver 调用前使用自身冻结的 `PublishLatencyModel` 复核并重算不一致输入，避免 predictor、solver 和 publication audit 消费不同预计时刻。

## 3. 统一执行时间查询

`ExecutionStatePrediction` 现在显式返回：

```text
execution_lead_sec
grid_execution_lead_steps
```

`ControlCycleInputPreparer` 和 ROS delay diagnostics 都通过 `ExecutionStatePredictor -> ExecutionModel` 查询 `requiredHistorySec()`、`executionLeadSec()` 和 `gridExecutionLeadSteps()`，不再各自重复 `max(delay)` 或 `ceil(delay/dt)` 公式。

## 4. 自动化证据

新增和扩展测试覆盖：

- estimate 完整 image 的单字段篡改与关闭模式；
- expected publish epoch 驱动 prediction，并进入 solver typed input；
- estimate off 保持原显式 evaluation epoch；
- 不一致 estimate 在状态 lookup 前 fail closed；
- engine 在 solver 调用前复核并重算不一致 estimate；
- PhaseClock、physical front 和 grid front 复用统一时间语义。

完整结果：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner -j1
  PASS

catkin_test_results build/test_results/spmpc_local_planner
  Summary: 552 tests, 0 errors, 0 failures, 0 skipped

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py'
  Ran 92 tests, OK

git diff --check
  PASS
```

## 5. 尚未完成的边界

WP2C 闭合的是预计时刻的在线接线，不是 formal 执行因果闭环：

- 默认 `publish_timing.enabled=false`，运行行为尚未启用固定预计延迟；
- $\widehat d_c$ 尚未由 Scout held-out 标定 artifact、适用域和 hash 冻结；
- predictor 仍只回放旧的 published-command history；
- 本周期新 $u^{\mathrm{pub}}$ 尚未作为 solver 决策量压入双通道 buffer；
- execution augmented state 和 $N_e=n_f+N_\ell$ horizon 尚未进入 formal OCP；
- C++ 与 CasADi/generated 模型的随机逐步一致性及独立执行 plant 尚未建立；
- ROS receipt 仍不是 driver/CAN/底盘 ACK。

下一工作包必须新增独立 delay-augmented Phase-Rejoin solver，使当前新命令真实影响双通道 pending buffer、执行器、机器人和液体联合终端；不能用本切片的 history wrapper 宣称 WP2 或 B0 已关闭。
