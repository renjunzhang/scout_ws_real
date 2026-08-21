# Phase-Rejoining WP2A 预计发布时间合同记录

- 日期：2026-08-21
- 分支：`offline-slosh-plan-online-tracking`
- 入口基线：`55365264`（WP1 唯一最终命令事务）
- 对应缺口：IMP-02 的模型/API/审计切片
- 结论：WP2A 通过；IMP-02 和整个 WP2 尚未关闭；formal 仍为 G0 NO-GO

## 1. 本切片完成的时间合同

新增无 ROS 的 `runtime/timing/PublishLatencyModel`，核心类型为：

```text
CycleTimingContract
PublishLatencyModelConfig
PublishEpochEstimate
PublishLatencyObservation
```

每周期以控制回调开始时刻 $t_c$ 为唯一计算起点：

$$
\widehat t_{\mathrm{pub}}=t_c+\widehat d_c,
\qquad
t_{\mathrm{deadline}}=t_c+\Delta t.
$$

`PublishLatencyModel` 不在线自适应。`estimated_dc_sec` 将来必须来自冻结的执行标定 artifact；当前 development YAML 只提供显式接口，默认：

```text
publish_timing/enabled=false
publish_timing/estimated_dc_sec=0.0
```

关闭估计时不会产生可供预测消费的 `expected_publish_stamp`，因此不改变现有控制行为；成功 receipt 仍会测量：

```text
actual_dc_sec = actual_publish_stamp - cycle_start_stamp
publish_deadline_missed = actual_publish_stamp > cycle_start_stamp + dt
```

开启固定估计后还记录：

```text
dc_error_sec = actual_publish_stamp - expected_publish_stamp
expected_publish_deadline_missed
```

实际发布时间在 ROS `cmd_pub_.publish()` 返回后采样，表示 ROS publisher 已接受该命令后的本地时间，不表示 Scout CAN/底盘执行 ACK。

## 2. 在线接线

ROS control callback 一开始即生成 `CycleTimingContract` 和 `PublishEpochEstimate`，然后才进入状态选择、预测和求解。`ControlCycleEngine` 校验传入 estimate 的 `cycle_id`、`cycle_start_stamp` 和 nominal period；不匹配时从当前 request 重新计算，禁止跨周期复用预计时刻。

正常控制和早期 fail-closed 零命令都在唯一命令事务 receipt 后形成 `PublishLatencyObservation`。发布失败、禁用发布或无有效时间戳时 observation 保持无效，并给出稳定状态码；时钟倒退不会生成负 `d_c`。

## 3. Typed audit

`ControlCycleAudit` 从 schema v3 升至 v4，新增：

- `expected_publish_stamp`；
- `publish_deadline_stamp`；
- `estimated_dc_sec`；
- `actual_dc_sec`；
- `dc_error_sec`；
- `publish_epoch_estimate_valid`；
- `publish_latency_observation_valid`；
- `expected_publish_deadline_missed`；
- `publish_deadline_missed`；
- `publish_timing_status`。

v4 继承 WP1 的 proposed/finalized/published 三层命令语义。wire-image golden 已重新冻结。

## 4. 自动化证据

新增和扩展测试覆盖：

- 固定 $\widehat d_c$ 只从 $t_c$ 计算预计时刻；
- 预计时刻、nominal deadline 和非整数秒纳秒转换；
- 实际 $d_c$、误差、按时和超时状态；
- estimate off 时仍测量实际交付延迟；
- 非有限/负估计、无效周期、发布失败和时钟倒退；
- engine/receipt/telemetry/audit 的预计与实际时刻贯通；
- schema v4 字段映射和 frozen wire-image。

完整结果：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner -j1
  PASS

catkin_test_results build/test_results/spmpc_local_planner
  Summary: 528 tests, 0 errors, 0 failures, 0 skipped

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py'
  Ran 92 tests, OK

git diff --check
  PASS
```

## 5. 尚未完成的边界

WP2A 只建立时间合同和证据，不是完整执行因果闭环：

- $\widehat d_c$ 尚未由 Scout held-out 标定冻结；
- `expected_publish_stamp` 尚未驱动 source-state 到发布时间的对齐；
- history-only predictor 尚未改为统一 `ExecutionModel`；
- 当前新 $u^{\mathrm{pub}}$ 尚未进入线/角 pending buffer；
- fractional delay、$\tau/K$ 增广状态和执行有效域尚未建立；
- `publish_deadline_missed` 当前是 audit，不是阻止发布的安全 gate；
- actual stamp 仍不是 driver/CAN/底盘 ACK。

下一切片应建立 `ExecutionModelContract`、`ExecutionAugmentedState` 和双通道 fractional-delay buffer，并让 `requiredHistorySec()`、`executionLeadSec()` 与预计发布时间成为唯一公共时间语义。
