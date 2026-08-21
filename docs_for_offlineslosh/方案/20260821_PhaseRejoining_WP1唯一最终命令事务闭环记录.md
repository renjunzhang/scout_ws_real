# Phase-Rejoining WP1 唯一最终命令事务闭环记录

- 日期：2026-08-21
- 分支：`offline-slosh-plan-online-tracking`
- 入口基线：`0fd7dd95`（WP0 基线与架构决定）
- 对应缺口：IMP-01
- 结论：WP1 自动化退出条件通过；formal 实物闭环仍为 G0 NO-GO

## 1. 闭合后的命令真值

一次正常控制周期现在按以下顺序执行：

```text
solver / terminal / Phase-Rejoin / safety proposed decision
                         ↓
              CommandPipeline::finalize
       finite + configured limiter + execution contract
                         ↓
                    FinalCommand
                         ↓  每周期一次
                ICommandSink::publish
                         ↓
                PublicationReceipt
                  ├─ 更新 limiter 已发布状态
                  ├─ 写入真实 receipt command history
                  └─ receipt 一致且命令未改写时 phase commit
```

`ControlCycleEngine::step()` 已拥有 arbitration、finalization、sink 调用、receipt、history 和 Phase-Rejoin commit 时序。ROS adapter 只实现时间采样、`VelocityCommand` 到 `geometry_msgs::Twist` 的转换和 `/cmd_vel` 交付，不再二次限幅、替换命令或直接写 history。

等待 odom/reference/TF/observer、预测失败和其他早期 fail-closed 分支也通过同一个 `PublicationTransaction` 发布受控零命令。

## 2. Receipt 和失败语义

`PublicationReceipt.delivered=true` 的当前含义是命令已经交给 ROS publisher。它不是 Scout driver、CAN 或底盘执行 ACK；更强的运行时确认属于 WP4/WP5。

状态提交规则固定为：

| 情况 | sink 调用 | limiter/history | Phase-Rejoin commit |
| --- | ---: | ---: | ---: |
| receipt 成功且命令一致 | 1 | 提交 receipt command | 仅候选满足且未被 limiter/contract 改写时提交 |
| 发布失败 | 1 | 不提交 | 不提交 |
| receipt 命令不一致 | 1 | 以 receipt 声明命令作为当前最佳执行真值提交 | 不提交 |
| `publish_cmd_vel=false` | 1 | 不提交 | 不提交 |
| limiter 或 contract 改写 | 1 | 提交实际发布命令 | 不按原 residual 提案提交 |

发布失败后不会再调用 sink 补发第二个零命令，因为那会破坏“每周期一个最终出口”。后续停车保证必须由 driver watchdog 和 supervisor 合同提供。

## 3. 审计协议

`ControlCycleAudit` 升级为 schema v3，明确分开：

- proposed：`post_gate_cmd_v/post_gate_cmd_omega`；
- finalized：`finalized_cmd_v/finalized_cmd_omega`；
- published：`published_cmd_v/published_cmd_omega`；
- 提交状态：`publication_receipt_consistent`、`command_history_committed`、`phase_rejoin_committed`。

wire-image golden 已随 schema v3 重新冻结。分析端不得再把 proposed 或 finalized 字段当作实际执行输入；执行预测只消费成功交付 receipt 写入的 history。

## 4. 自动化证据

定向测试覆盖：

- fake sink 每周期恰好调用一次；
- `history == receipt == final` 的正常路径；
- 发布失败不推进 history 或 limiter state；
- receipt 不一致时记录其声称的实际命令，但阻止 phase commit；
- `publish_cmd_vel=false` 仍经过唯一 sink 边界；
- finalization 本身不修改已发布状态；
- 非有限命令以 `COMMAND_NONFINITE` fail closed；
- limiter 改写阻止 Phase-Rejoin commit；
- telemetry/audit 的 proposed、finalized 和 published 三层一致映射。

完整回归结果：

```text
catkin_test_results build/test_results/spmpc_local_planner
  Summary: 518 tests, 0 errors, 0 failures, 0 skipped

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py'
  Ran 92 tests, OK

git diff --check
  PASS
```

静态核查：

```text
cmd_pub_.publish(...)          生产代码仅 ROS sink 中 1 处
phase_rejoin_.commit(...)      生产代码仅 receipt 判断之后 1 处
CommandPipeline::finalize(...) 不更新 previous/history
```

## 5. WP1 退出核查

- [x] 每个正常或失败控制周期只有一次最终命令 sink 调用；
- [x] ROS wrapper 不再二次限幅、替换命令或写 history；
- [x] 早期失败路径通过同一事务发布受控零命令；
- [x] phase 不依据未发布或被改写的 residual 命令推进；
- [x] fake sink、limiter 改写、发布失败和 receipt 不一致测试通过；
- [x] proposed/finalized/published 三层进入 typed audit；
- [x] 完整 C++ 和 Python 回归通过。

因此 IMP-01 可以关闭，下一工作包进入 WP2 的预计发布时间和统一执行模型。

以下事项不属于 WP1 完成声明，仍保持未完成：`d_c` 预测、双通道 pending buffer、fractional delay、`tau/K` 正式执行模型、无条件 `|v|/|omega|` 硬包络、独立 odom/TF watchdog、solver deadline、driver watchdog/ACK、typed session 和实物 G0 标定。
