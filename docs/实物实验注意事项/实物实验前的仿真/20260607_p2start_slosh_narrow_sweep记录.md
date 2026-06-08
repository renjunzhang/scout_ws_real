# 20260607 P2-start slosh 权重窄扫 N=3 记录

## 1. 目的

上一轮 `B_smooth / B_slosh / B_ours` 诊断性 `N=1` 已确认：

```text
slosh cost 确实进入 /spmpc/cost_breakdown，并且会改变 cmd_vel / 加速度 / omega-rate / 完成行为。
```

因此本轮不再验证“有没有生效”，而是检查：

```text
在 w_slosh = 2.0 / 2.2 / 2.4 的窄区间内，B_slosh / B_ours 是否存在既能完成、又能降晃的稳定可重复区间。
```

本轮仍然：

```text
1. 暂停外部 TEB/DWA。
2. 不直接 N=5。
3. 不修改仿真环境，只调整 planner/算法参数。
4. 每个 case 单独 fresh sim。
```

## 2. fresh sim 设置

P2 起点附近 spawn：

```text
SPAWN_X=3.30
SPAWN_Y=0.15
SPAWN_Z=0.1
SPAWN_YAW=-3.08
```

执行规则：

```text
1. 每个 case 单独启动仿真。
2. 启动后等待 30s，让定位恢复。
3. 只跑一个 planner/case。
4. 每个 case 最多记录 60s；超过约 1min 仍无有效完成则视为失败/不再继续记录。
5. 关闭 planner / rosbag / path publisher / slosh monitor / sim。
6. 等待 30s，确认仿真完全关闭后再启动下一次 fresh sim。
```

本轮结束后脚本检查为：

```text
ROS_MASTER_INACTIVE
```

## 3. 本轮对象

本轮为窄区间诊断 `N=3`：

```text
B_slosh: w_slosh = 2.0 / 2.2 / 2.4
B_ours:  w_slosh = 2.0 / 2.2 / 2.4
```

共：

```text
2 variants × 3 weights × 3 fresh-sim runs = 18 cases
```

外部 baseline：

```text
TEB/DWA 暂停，不参与本轮。
```

## 4. 输出目录

```text
/data/a/spmpc_paper_compare/p2start_slosh_narrow_sweep_n3_20260607_212552
```

脚本：

```text
/tmp/spmpc_p2start_narrow_sweep_n3.sh
/tmp/analyze_spmpc_slosh_cost_sweep.py
/tmp/aggregate_spmpc_slosh_narrow_sweep.py
```

主要输出：

```text
/data/a/spmpc_paper_compare/p2start_slosh_narrow_sweep_n3_20260607_212552/fixed_path_metrics.csv
/data/a/spmpc_paper_compare/p2start_slosh_narrow_sweep_n3_20260607_212552/fixed_path_metrics_pre_terminal.csv
/data/a/spmpc_paper_compare/p2start_slosh_narrow_sweep_n3_20260607_212552/fixed_path_metrics_group_summary.csv
/data/a/spmpc_paper_compare/p2start_slosh_narrow_sweep_n3_20260607_212552/slosh_cost_sweep_behavior_summary.csv
/data/a/spmpc_paper_compare/p2start_slosh_narrow_sweep_n3_20260607_212552/slosh_cost_sweep_behavior_summary.md
/data/a/spmpc_paper_compare/p2start_slosh_narrow_sweep_n3_20260607_212552/narrow_sweep_behavior_group_summary.csv
/data/a/spmpc_paper_compare/p2start_slosh_narrow_sweep_n3_20260607_212552/narrow_sweep_behavior_group_summary.md
```

`J_slosh` 仍按上一轮定义读取：

```text
J_slosh mean = /spmpc/cost_breakdown[10] + /spmpc/cost_breakdown[11]
slosh pct mean = /spmpc/cost_breakdown[21]
```

## 5. N=3 group summary

注意：失败/停滞 case 在近乎不运动时可能得到表面较低的 p95/peak，因此先看 `success/stable`，再看成功 run 内的 slosh peak/p95。

| variant | w_slosh | n | success | stable | duration mean s | final mean m | final success m | tracking success m | peak success mean±std mm | p95 success mean mm | cmd_v success mean m/s | cmd acc RMS | omega-rate RMS | J_slosh mean | slosh pct mean | solver ms | statuses |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| B_slosh | 2.0 | 3 | 2/3 | 2/3 | 31.09 | 2.073 | 0.259 | 0.105 | 26.84±1.67 | 9.02 | 0.420 | 0.368 | 8.764 | 0.0615 | 9.99 | 4.75 | B_slosh_ACADOS_OK:1; GOAL_REACHED:2 |
| B_slosh | 2.2 | 3 | 2/3 | 2/3 | 29.27 | 2.666 | 0.271 | 0.134 | 25.89±5.47 | 9.14 | 0.478 | 0.353 | 9.221 | 0.0607 | 8.52 | 4.81 | B_slosh_ACADOS_OK:1; GOAL_REACHED:2 |
| B_slosh | 2.4 | 3 | 3/3 | 3/3 | 15.76 | 0.266 | 0.266 | 0.149 | 31.49±4.10 | 11.86 | 0.458 | 0.461 | 12.878 | 0.1827 | 15.43 | 4.73 | GOAL_REACHED:3 |
| B_ours | 2.0 | 3 | 2/3 | 2/3 | 30.81 | 2.653 | 0.235 | 0.138 | 32.55±2.70 | 9.75 | 0.419 | 0.341 | 8.435 | 0.0642 | 7.70 | 4.82 | B_ours_ACADOS_OK:1; GOAL_REACHED:2 |
| B_ours | 2.2 | 3 | 2/3 | 2/3 | 29.66 | 2.640 | 0.264 | 0.115 | 37.60±3.58 | 10.16 | 0.465 | 0.330 | 7.746 | 0.0612 | 5.99 | 4.73 | B_ours_ACADOS_OK:1; GOAL_REACHED:2 |
| B_ours | 2.4 | 3 | 2/3 | 2/3 | 30.48 | 2.703 | 0.279 | 0.144 | 41.60±6.37 | 10.92 | 0.435 | 0.327 | 7.934 | 0.0954 | 11.83 | 4.76 | B_ours_ACADOS_OK:1; GOAL_REACHED:2 |

## 6. 单次 run 关键现象

### 6.1 B_slosh

```text
B_slosh w=2.0: 2/3 完成；失败 run final=5.700m，cmd_v_mean=0.070。
B_slosh w=2.2: 2/3 完成；失败 run final=7.454m，cmd_v_mean=0.019。
B_slosh w=2.4: 3/3 完成；没有停滞 run。
```

`B_slosh w=2.0/2.2` 的成功 run 内 peak 更低：

```text
B_slosh w=2.0: peak_success_mean = 26.84 mm, p95_success_mean = 9.02 mm
B_slosh w=2.2: peak_success_mean = 25.89 mm, p95_success_mean = 9.14 mm
```

但二者都有 1/3 停滞，不能作为正式稳定候选。

`B_slosh w=2.4` 是本轮唯一 `3/3 success/stable` 的组合：

```text
B_slosh w=2.4: peak_success_mean = 31.49 mm, p95_success_mean = 11.86 mm
```

它更稳，但降晃优势没有 `w=2.0/2.2` 的成功 run 明显。

### 6.2 B_ours

本轮 `B_ours` 在 2.0/2.2/2.4 均为：

```text
2/3 success, 2/3 stable
```

各权重都有一个停滞 run，且成功 run 内 peak 不低：

```text
B_ours w=2.0: peak_success_mean = 32.55 mm, p95_success_mean = 9.75 mm
B_ours w=2.2: peak_success_mean = 37.60 mm, p95_success_mean = 10.16 mm
B_ours w=2.4: peak_success_mean = 41.60 mm, p95_success_mean = 10.92 mm
```

这说明当前完整策略在该窄区间内没有给出明确的稳定降峰值候选；`smooth/control` 组合可能仍然压住了 slosh-aware 项，或者 slosh cost scale/归一化需要进一步检查。

## 7. 与前序结果的关系

上一轮 `N=1` 中 `B_slosh w=2.2` 单次表现很强：

```text
B_slosh w=2.2 N=1: peak = 21.50 mm, p95 = 8.06 mm, success=1
```

但本轮 `N=3` 显示：

```text
B_slosh w=2.2: success=2/3, stable=2/3
```

所以 `B_slosh w=2.2` 的强降峰值响应存在，但稳定性不足，不能直接扩大 N=5。

前序 robust common-limit P2-start N=3 中 `B_smooth` 参考值为：

```text
B_smooth: success=3/3, stable=3/3, peak mean≈30.83 mm, p95 mean≈10.72 mm
```

与这个参考相比，本轮可谨慎理解为：

```text
1. B_slosh w=2.0/2.2 的成功 run 有更低 peak/p95 潜力，但完成率不足。
2. B_slosh w=2.4 完成率达到 3/3，但 peak/p95 没有明显优于 B_smooth 参考。
3. B_ours 2.0/2.2/2.4 本轮未表现出“稳定完成 + 明显降峰值”的组合。
```

该比较不是同一批次同时包含 `B_smooth` 的 formal ranking，只作为决策参考。

## 8. 补充复核：去掉前 2s 的 post-start peak

对 P2_s_curve 的路径形状复查后发现：

```text
s=0.0–2.8 m  |kappa|≈0.8   急转/S 弯集中在最前面
s=2.8–9.2 m  |kappa|≈0.1   后面基本是长直线到终点
```

因此这条路径的“转弯段”与“从静止起步猛冲”高度重叠。仅按曲率框出 S 弯段不能有效隔离过弯抑晃，反而会把起步 lurch 一起算进去。更有效的诊断口径是：保留 full-window 指标，同时新增 `exclude-first-2s` / `post-start` 指标。

按去掉前 2s 的 peak 复核，本轮 narrow sweep 的方向为：

```text
B_slosh w=2.0 / 2.2 / 2.4: 18.7 / 20.4 / 19.1 mm
B_ours  w=2.0 / 2.2 / 2.4: 21.2 / 19.7 / 14.7 mm
B_smooth 参考:                 约 22–23 mm
```

这个补充结果修正了 full-window peak 的解释：

```text
1. 起步 lurch 是真实污染，而且幅度很大；尤其 B_ours 在 N=1 复核中可从 35.8 mm 降到 12.2 mm。
2. 去掉前 2s 后，slosh-aware 相对 B_smooth 的方向是对的，说明真效果一直存在，只是被起步峰值盖住。
3. N=3 中 slosh-aware 仍低于 B_smooth 参考，但差距比 N=1 小，结果更糊，不能直接作为 formal ranking。
4. 外部 TEB/DWA 的低晃动不是窗口假象；去掉前 2s 后它们仍然更低，后续只能用速度/完成率/跟踪/slosh 的 Pareto 口径解释。
```

注意：`exclude-first-2s` 不是“纯 S 弯段”指标。它只是去掉起步瞬态后的行进阶段指标。若要干净隔离过弯抑晃，需要统一起步速度 ramp，或者换一条“先直线引导段、再中段 S 弯”的 fixed path。

## 9. 当前结论

本轮可以得出的结论：

```text
1. 2.0–2.4 区间确实比 2.75 更接近可用区，但仍不是可以直接 N=5 的最终区间。
2. B_slosh w=2.4 是本轮唯一 3/3 success/stable 的 slosh-aware 组合。
3. full-window peak 受到起步 lurch 明显污染；在 P2_s_curve 上不能仅凭全程 peak 判断 B_ours 或 slosh-aware 无效。
4. 去掉前 2s 后，B_slosh/B_ours 相对 B_smooth 参考值呈现更低 peak，说明 slosh-aware 的行进阶段抑晃方向成立。
5. N=3 post-start 差距比 N=1 小，且 B_slosh 2.0/2.2、B_ours 2.0/2.2/2.4 仍有完成率问题，因此还不是 formal ranking。
6. 当前核心矛盾应更新为：slosh-aware 已能降低起步后晃动，但仍需同时解决起步 lurch、稳定完成率和外部 baseline 慢/保守带来的 Pareto 对比问题。
```

当前不能得出的结论：

```text
1. B_slosh w=2.2 是最终最优。
2. B_ours w=2.2 已经足够稳定且显著优于 B_smooth 的 formal 结论。
3. 可以直接把 B_slosh/B_ours 全部扩大到 N=5。
4. 2.4 是最终权重；它只是本轮唯一 3/3 完成的候选。
5. exclude-first-2s 可以替代 full-window；完整运输风险仍必须报告起步峰值。
6. P2_s_curve 的曲率窗口等于纯过弯窗口；这条路径的急转贴近起点，该口径会和起步段混在一起。
```

## 10. 下一步建议

不建议立刻跑 full N=5。更稳的下一步是先做指标修正和同批复核：

```text
方案 A：先修指标口径
  - 在 metrics 中同时报告 full-window 与 exclude-first-2s/post-start slosh peak/p95/RMS。
  - full-window 用于完整运输风险；post-start 用于排除起步 lurch 后观察行进阶段抑晃。
  - 不把 P2_s_curve 的曲率窗口称为“纯 S 弯段”。

方案 B：做干净同批 N≥3 internal 复核
  - 同批 fresh-sim 包含 B0 / B_smooth / B_slosh@2.2 / B_ours@2.2。
  - 可选加 B_slosh@2.4 / B_ours@2.4，用于比较完成率更稳的候选。
  - 同时报告 success/stable、duration、tracking、full-window slosh 与 post-start slosh。

方案 C：如果继续保留 P2_s_curve
  - 需要承认它的急转紧贴起点，不能干净隔离“起步”和“过弯”。
  - 若要干净隔离过弯抑晃，应考虑统一起步速度 ramp，或增加一条“先直线引导段、再中段 S 弯”的 fixed path。

方案 D：外部 baseline 仍走 Pareto 解释
  - TEB/DWA 去掉前 2s 后 absolute slosh 仍低，窗口修正不能解决这道坎。
  - 后续应同时比较 success / duration / tracking / slosh，而不是只比较 peak。
```

一句话：

> 本轮把问题从“slosh cost 是否生效”进一步推进到“full-window peak 被起步 lurch 污染，post-start 后 slosh-aware 效果能拉开，但仍需同批 N≥3 和 Pareto 口径复核”；因此仍不应直接做全量 N=5。
