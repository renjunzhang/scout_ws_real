# PR-RMPC 与 Tail-Commit 路线阶段性失败结论

- 结论日期：2026-08-25
- 正式 v1 基线：`e3b23105`
- 最后工程候选：`4f656f0300e901571934d557758d03cf3d62d79a`
- 证据范围：独立 Plant 仿真的正式 v1 与后续 development 试验

> **结论：PR-RMPC／Tail-Commit 作为论文的复杂主方法没有获得实验支持，本路线阶段性关闭。**
>
> 这不是“代码无法运行”的结论，也不能外推为相位重接或 Tail-Commit 在所有场景中普遍无效。失败的是当前实现在当前仿真合同下支撑论文效果与机制主张的能力。

## 1. 失败的是什么

| 层面 | 结论 |
| --- | --- |
| 总体效果 | 正式 v1 未通过，不允许声称 PR-RMPC 相对对比方法降低液面晃动 |
| gate 机制 | 正式 C4−C3 几乎为零；15D development 中差异仍极小，不足以支撑 gate 或尾段接纳的独立贡献 |
| 在线 residual 价值 | 15D C4 与 BT 在主运动过程中基本一样，且 C4 很早进入 Tail-Commit，没有证明复杂在线 MPC 带来足以支撑论文的增量价值 |
| 继续投入的合理性 | 在已查看 v1 结果后继续修改 cost、gate、阈值和 seeds，会扩大 post-hoc 风险，且目前效应大小与工程复杂度不匹配 |

## 2. 没有失败的是什么

- `4f656f03` 中的 15D 双通道 delay＋inertia 对齐、单调相位推进和持久 Tail-Commit 已形成可运行的工程闭环。
- seeds 9921--9923 上 BT、C3 和 C4 均为 `3/3` 完成，没有 solver failure、controlled stop、zero request、发布回执或 history 异常。
- 离线防晃轨迹、BT、15D execution aligner、命令事务和 TailCommitStateMachine 仍是可复用的工程资产。
- 本轮没有正式证明 BT 的论文效果，也没有否定相位重接在其他方法结构或实物条件下的可能性。

## 3. 正式 v1：必须保留的负结果

正式 v1 绑定 `e3b23105`，共 16 个 seeds、6 个条件和 96 个 trial；96/96 均已记录。冻结分析给出：

```text
status = FAIL
formal_pass = false
paper_claim_authorized = false
failed_trials_retained = 50
```

完整 motion-plus-fixed-tail 窗口的独立 Plant 液面 Q95 中位数为：

| 条件 | 液面 Q95 中位数 | 任务成功 | 本轮可作的解释 |
| --- | ---: | ---: | --- |
| C0 OrdinaryMPCC | `0.245 mm` | `0/16` | task／zero-request 合同失败，不能当作成功基线 |
| C1 SmoothMPCC | `0.190 mm` | `14/16` | 液面低于 C4，但保留 2 次任务失败 |
| C2 OfflineReplay | `0.583 mm` | `0/16` | task／zero-request 合同失败，不能当作成功离线证据 |
| C3 GateMonitorPR-RMPC | `0.375 mm` | `16/16` | 能完成任务，但效果不优于 C0/C1 |
| C4 完整 PR-RMPC | `0.375 mm` | `16/16` | 能完成任务，但总体效果方向为负 |
| IS ZVD | `0.201 mm` | `0/16` | task／zero-request 合同失败，只保留为失败记录 |

C3/C4 的 `16/16` 说明方法并非完全跑不起来；但 C4 液面高于 C0 和 C1，而 C4−C3 在科学量级上接近零。C0/C2/IS 的任务合同问题必须如实保留，它们既不能被宣称为成功对比，也不能把已经冻结的正式 `FAIL` 改写成正向结果。

## 4. 15D Tail-Commit development：工程闭环成功，论文效果未成立

seeds 9921--9923 的结果为：

| 条件 | 任务完成 | 完整窗口液面 Q95 中位数 |
| --- | ---: | ---: |
| BT | `3/3` | `0.12782 mm` |
| C3 Tail-Commit monitor | `3/3` | `0.12867 mm` |
| C4 15D Tail-Commit | `3/3` | `0.12680 mm` |

配对结果显示：

- C4−BT 的均值为 `-0.000303 mm`，中位数为 `+0.000296 mm`，三个 seed 符号混合，效应在实质上为零。
- 10%--90% progress 窗口的 C4−BT 均值约为 `+0.0000008 mm`，主运动过程几乎完全相同。
- C4−C3 的均值为 `-0.001247 mm`，三个 seed 同向，但绝对效应极小。
- C4 约在 `1.83--2.00 s` 就进入 Tail-Commit，之后基本执行 BT；C3 约在 `8.20--8.53 s` 进入 Tail-Commit。

因此，这组试验支持“延迟对齐和持久尾段锁定能够稳定运行”，却不支持“在线 residual MPC 相对 BT 带来有论文意义的改善”。其中 seed 9921 参与过排错，仅 9922--9923 是未见 seeds；样本不足以作为独立确认。新正式实验仍为 `HOLD`、0 trial，这一轮不是 replication，也不是正式正向证据。

## 5. 学术主张边界

后续写作中可以如实表述：

- 原 PR-RMPC 正式 v1 的预定总体效果检验失败。
- 15D Tail-Commit 候选解决了一部分执行闭环问题，但在当前 development 试验中与 BT 几乎不可区分。
- 当前证据不支持 gate、residual 或 Tail-Commit 的独立防晃增益主张。

不得表述为：

- PR-RMPC 或相位重接在所有仿真和实物条件下普遍无效；
- BT 已经被正式证明为最优或具有显著防晃优势；
- 15D development 是原方法的 replication；
- 已查看 v1 结果后得到的阈值、方法修改或 seeds 属于事前冻结。

## 6. 路线关闭决定

1. 冻结并保留正式 v1 的 96 个 trial、冻结 analyzer 输出和所有失败记录，不覆盖、不删除、不重命名为 replication。
2. 关闭 PR-RMPC 复杂主线；不再通过调 cost、gate、阈值或更换 seeds 来挽救当前论文主张。
3. `4f656f03` 保留为可复用工程底座；现有 C3/C4 候选保留原 implementation ID，不改名冒充新方法。
4. 下一方法进入 [BT 参考全命令 S-MPCC 与尾段重接方案](../方案/20260824_BT参考全命令SMPCC与尾段重接方案.md)：先对老全命令 S-MPCC 做最低成本 Go/No-Go，通过才加最小尾段检查；失败则立即转向低维在线相位重定时。
5. 新方法使用全新 implementation ID 和 development seeds；只有 development 多 seed 稳定正向后，才建立新的离线计划、正式 seeds/order/session。

## 7. 证据位置

正式 v1：

```text
/data/a/spmpc_exec_identification/
phase_rejoin_formal_campaign_v5_e3b23105_20260824/
```

15D Tail-Commit development：

```text
/data/a/spmpc_exec_identification/tail_commit_15d_c4_dev_3O9nrE/
/data/a/spmpc_exec_identification/tail_commit_15d_c3_dev_mmpyj0/
/data/a/spmpc_exec_identification/tail_commit_15d_bt_dev_Bhs34t/
```

对应 Git 节点：

- `e3b23105`：正式 v1 方法和实验口径基线；
- `4f656f03`：15D Tail-Commit 工程候选及本软件底座。
