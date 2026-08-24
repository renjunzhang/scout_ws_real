# BT 参考全命令 S-MPCC 最低成本 Go/No-Go 实现与判定报告

- 日期：2026-08-25
- 工程基线：`b48039cb75ac0ec2f97a7d1fde4caf57c87520f3`
- 分支：`offline-slosh-plan-online-tracking`
- 证据等级：development only
- 最终决定：`NO_GO_ROUTE_B`

## 1. 范围与停止规则

本轮只验证老 continuous S-MPCC 跟踪完整 BT 时序参考这一核心假设：

1. 冻结 BT 名义基线及 D1/D2 两类扰动；
2. 使用预计发布时刻的 15D 执行对齐状态，提取老求解器需要的 10D 物理状态；
3. 先以 monitor 发布 BT、旁路计算 S-MPCC，再以 direct 发布 S-MPCC 完整命令；
4. 计划使用相同 seeds `9911`–`9915` 作 BT/S-MPCC-BT 配对比较；
5. monitor 或 direct canary 任一关键合同失败，立即写出唯一 `NO_GO_ROUTE_B`，不重试、不调权重、不继续补齐 grid。

本轮没有实现或启用尾段接纳器、Tail-Commit、经验 Gate、residual 权限收缩或新的复杂状态机。

## 2. 冻结合同

| 项目 | 冻结值 |
| --- | --- |
| seeds | `9911`–`9915` |
| D1 | 路径左法向 `+0.050 m`，yaw `+0.100 rad` |
| D2 | artifact indices `[750,760)`，线速度上限 `0.320 m/s`，角速度不变 |
| tracking 主指标 | `tracking_q95_m` |
| tracking 通过线 | 每类扰动至少 4/5 改善，配对中位改善不低于 10% |
| 完整液面非劣界 | `+0.010 mm` |
| fixed-tail 液面非劣界 | `+0.025 mm` |
| 完成时间最大比值 | `1.10` |
| 命令死区 | `|delta_v| > 0.01 m/s` 或 `|delta_omega| > 0.02 rad/s` |
| 有效修正占比 | 每类扰动均不低于 10% |
| BT 相位硬窗 | `|s-s_BT| <= 0.10 m` |

S-MPCC 权重冻结为：contour `1`、lag `0.2`、progress `0.2`、v `1`、vs `0.3`、control/alpha/du `0.1`、slosh `5`；heading、progress-coupling、yaw-rate 均为 `0`。

## 3. 最小实现

- 在旧 59 个 stage parameters 后追加 `BT_REFERENCE_ACTIVE=59`、`NOM_S=60`、`BT_PHASE_HALF_WIDTH=61`，合同变为 `NP=62`、`NH=3`，原索引前缀不变。
- 新增独立 `BtTimedReferenceContext`；它与 Phase-Rejoin enforce 互斥，不复用经验 gate、Tail-Commit 或 residual 状态机。
- N=60 老 10D continuous S-MPCC 接收完整 N+1 BT pose/progress/robot/slosh/control 参考，并使用 `|s-s_BT|<=0.10 m` 的硬窗。
- stage 0 warm start 使用 execution-aligned 实际物理状态，其余 stage 使用 BT 名义状态；artifact 末端以最后零命令样本 padding。
- 新增 `smpcc_bt_monitor` 和 `smpcc_bt_direct`：monitor 只发布 BT；direct 不允许候选失败后回退 BT。
- D2 只在唯一 `PublicationTransaction` 内、普通命令流水线之后和 sink 之前施加；receipt、history、limiter 和 Plant 看到同一个 cap 后命令事实。
- runner 记录完整时序参考误差、execution-aligned 状态、BT counterfactual、candidate 状态/耗时、命令差异、D2 审计、完整窗口和 fixed-tail 指标。

## 4. 构建与测试

本机重新生成并使用以下 gitignored capsule：

- `spmpc_slosh`: N=60、NP=62、NH=3；
- `spmpc_phase_rejoin`: N=3、NP=62、NH=3。

验证结果：

- package 构建成功；
- codegen `slosh`、`phase_rejoin` symbolic check 均通过；
- 定向 C++ 测试 58/58 通过；
- Python manifest/analyzer 测试 6/6 通过；
- fail-fast campaign 测试 6/6 通过。

运行时固定使用 private runner，不能使用 `devel/lib` 中的旧二进制：

```text
/home/a/scout_ws/devel/.private/spmpc_local_planner/lib/spmpc_local_planner/spmpc_phase_rejoin_closed_loop_trial
```

## 5. 冻结证据

证据目录：

```text
/data/a/spmpc_exec_identification/smpcc_bt_go_no_go_dev_20260825.K7mQ2s
```

关键 SHA256：

| 资产 | SHA256 |
| --- | --- |
| working diff（package scope，排除 `Testing/`） | `feb494d540e669eca9a81ee658763af95f67ac314ad330f8b2cb3181c67ee447` |
| private runner | `4b2a82b56c3acf85bb6bf6c400e77172f4c44beae505a20f42c831880d10fd07` |
| campaign | `b561aed6e3c15c8caa27f1bbe2cd3c1ea7eca6b7065bcadeb0a7e505531ec2b8` |
| analyzer | `22e8e938c7f75512511be633d8ca3404bcd547277a4f2e261a4322947fef4745` |
| artifact | `ff2b4d4b8858a1df2545cc32705dbe5c3542918ca51a761c7e3810166c2721cc` |
| independent Plant | `13bd56b2aa33919fd9d44fa4cab849896fc80e7743146f0123df3d2de05cf10d` |
| path | `8790c5ba5e7d167d2fcbfe2c366116ddffa5101ea3d9684821703461a4ee54fd` |

`campaign_manifest.json` 还冻结了所有条件 YAML 的逐文件 hash、planned trial 顺序和 private `LD_LIBRARY_PATH`。

## 6. 实验结果

### 6.1 BT 冻结

15 个 BT trial 全部完成：

| 条件 | 完成率 | tracking q95 中位 | 完整液面 q95 中位 | fixed-tail q95 中位 | settle time 中位 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 名义 | 5/5 | 22.394 mm | 0.133229 mm | 0.066184 mm | 39.000 s |
| D1 | 5/5 | 50.716 mm | 0.132484 mm | 0.066184 mm | 39.000 s |
| D2 | 5/5 | 23.340 mm | 0.150388 mm | 0.066184 mm | 39.000 s |

D2 每个 seed 都精确记录 10 个 active cycles 和 10 个 modified cycles，说明 cap 在 `[750,760)` 内实际生效。

### 6.2 D1 monitor canary

`seed=9911` 的 monitor 完成并通过前置合同：

- 发布 BT，完整 artifact clock 完成，task success；
- S-MPCC candidate `1310/1310` 成功；
- BT timed-reference cycles `1310`，terminal padding stages `1830`；
- 有效修正 `958/1166 = 82.16%`，没有退化成 BT；
- solve p95 `8.980 ms`、max `22.116 ms`，30 Hz deadline miss 为 `0`。

因此 direct canary 被允许启动。

### 6.3 D1 direct canary 与早停

`seed=9911` 的 direct canary 失败：

- task success 为 false，未进入 goal tolerance，也没有 settled time；
- candidate 出现 5 次 `ACADOS_SOLVE_FAILED_4`，acados 状态 4 为 `ACADOS_QP_FAILURE`；
- 第一次失败发生在 cycle 1187（39.533 s），失败周期发布零命令，状态为 `TRACKING_UNSAFE_PROJECTION`；
- 当时机器人已经落后完整 BT 时序约 7.72 m，随后仍未完成目标；
- 与同 seed BT 相比，`tracking_q95_m` 从 `0.050716 m` 变为 `0.063587 m`，恶化 25.38%；
- 完整液面 q95 增加 `0.065053 mm`，超过冻结的 `+0.010 mm` 非劣界；
- fixed-tail q95 差值约 `-0.000066 mm`，但不能抵消完成、tracking 和完整液面的失败；
- solve p95 `9.606 ms`、max `22.579 ms`，deadline miss 为 `0`，所以失败不是实时预算不足造成的。

campaign 在第 17 个 trial 后立即停止，没有运行 D1 direct seeds `9912`–`9915` 或任何 D2 direct trial，也没有补跑或调参。完整 20 对证据不存在，因此通用 analyzer 没有被错误地用于不完整 grid；fail-fast campaign 直接生成最终早停判定：

```text
NO_GO_ROUTE_B
```

机器可读判定为：

```text
/data/a/spmpc_exec_identification/smpcc_bt_go_no_go_dev_20260825.K7mQ2s/route_decision.json
```

## 7. 结论

老 S-MPCC 能在 BT 实际发布的 monitor 条件下稳定计算并产生非平凡修正，但这些候选一旦直接闭环发布，就不能可靠跟随完整 BT 时钟：首个 D1 direct canary 已经未完成、发生 QP failure、tracking 变差且完整液面越过非劣界。

这已经否定路线 A 的核心前提。按照预先冻结的“一次验证、失败即止”规则，停止老 S-MPCC 权重调整和状态机扩展，后续在线修正核心转为路线 B：低维在线相位重定时。
