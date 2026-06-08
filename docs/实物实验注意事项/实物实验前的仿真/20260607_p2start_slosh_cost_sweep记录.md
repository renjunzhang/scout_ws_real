# 20260607 P2-start slosh cost 小扫记录

## 1. 目的

上一轮 robust P2-start common-limit N=3 中，SPMPC 内部方法整体稳定，但 `B_ours w_slosh=2.2` 与 `B_smooth` 的晃动指标非常接近，不能直接说明完整 slosh-aware 方法已经明显优于非液体感知平滑基线。

因此本轮不直接扩大到 N=5，也暂停外部 TEB/DWA，而是加回 `B_slosh` 做小扫，先确认：

```text
1. slosh cost 是否真的进入并改变控制行为；
2. w_slosh 改变后 cmd_vel / 加速度 / omega-rate / slosh cost breakdown 是否随之变化；
3. B_slosh 与 B_ours 是否存在过保守或不稳定区间。
```

本轮只调整 planner/算法参数，不修改仿真环境。

## 2. fresh sim 设置

沿用 P2 起点附近 fresh sim：

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

ROS 状态检查：本轮结束后为：

```text
ROS_MASTER_INACTIVE
```

## 3. 本轮对象

本轮是诊断性 `N=1` 小扫，不是正式统计：

```text
B_smooth                w_slosh = default / -1.0 override sentinel
B_slosh                 w_slosh = 1.5 / 2.2 / 2.75
B_ours                  w_slosh = 1.5 / 2.2 / 2.75
```

外部 baseline：

```text
TEB/DWA 暂停，不参与本轮。
```

## 4. 输出目录

```text
/data/a/spmpc_paper_compare/p2start_slosh_cost_sweep_n1_20260607_205216
```

主要输出：

```text
/data/a/spmpc_paper_compare/p2start_slosh_cost_sweep_n1_20260607_205216/fixed_path_metrics.csv
/data/a/spmpc_paper_compare/p2start_slosh_cost_sweep_n1_20260607_205216/fixed_path_metrics_pre_terminal.csv
/data/a/spmpc_paper_compare/p2start_slosh_cost_sweep_n1_20260607_205216/fixed_path_metrics_group_summary.csv
/data/a/spmpc_paper_compare/p2start_slosh_cost_sweep_n1_20260607_205216/slosh_cost_sweep_behavior_summary.csv
/data/a/spmpc_paper_compare/p2start_slosh_cost_sweep_n1_20260607_205216/slosh_cost_sweep_behavior_summary.md
```

`slosh_cost_sweep_behavior_summary.*` 额外读取 `/spmpc/cost_breakdown`：

```text
J_slosh mean = cost_breakdown[10] + cost_breakdown[11]
slosh pct mean = cost_breakdown[21]
```

## 5. 结果表

pre-terminal / active solver window 主要结果：

| variant | w_slosh | success | stable | duration s | final m | tracking RMS m | peak mm | p95 mm | cmd_v mean | abs omega mean | cmd acc RMS | omega-rate RMS | J_slosh mean | slosh pct mean | pred h_peak mean | solver ms | last status |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| B_smooth | -1.0 | 1 | 1 | 15.44 | 0.287 | 0.172 | 36.18 | 11.02 | 0.474 | 0.573 | 0.474 | 12.141 | 0.0000 | 0.00 | 0.000 | 3.81 | GOAL_REACHED |
| B_slosh | 1.5 | 0 | 0 | 58.68 | 7.256 | 0.095 | 34.63 | 3.61 | 0.028 | 0.130 | 0.190 | 5.511 | 0.0337 | 7.09 | 0.974 | 4.73 | B_slosh_ACADOS_OK |
| B_slosh | 2.2 | 1 | 1 | 14.72 | 0.275 | 0.202 | 21.50 | 8.06 | 0.510 | 0.400 | 0.442 | 9.504 | 0.1112 | 11.97 | 3.110 | 4.72 | GOAL_REACHED |
| B_slosh | 2.75 | 0 | 0 | 58.72 | 7.454 | 0.081 | 40.05 | 2.24 | 0.019 | 0.073 | 0.150 | 4.232 | 0.0232 | 4.22 | 0.371 | 5.12 | B_slosh_ACADOS_OK |
| B_ours | 1.5 | 1 | 1 | 15.00 | 0.275 | 0.100 | 35.05 | 13.65 | 0.450 | 0.317 | 0.412 | 9.855 | 0.0648 | 8.99 | 2.954 | 4.51 | GOAL_REACHED |
| B_ours | 2.2 | 1 | 1 | 15.42 | 0.273 | 0.114 | 35.78 | 10.04 | 0.460 | 0.327 | 0.397 | 7.784 | 0.0827 | 8.37 | 2.477 | 4.47 | GOAL_REACHED |
| B_ours | 2.75 | 0 | 0 | 58.71 | 7.479 | 0.162 | 33.91 | 4.15 | 0.026 | 0.096 | 0.185 | 5.410 | 0.0784 | 5.58 | 0.874 | 5.14 | B_ours_ACADOS_OK |

补充：各 planner log 中仍可见不同数量的 `ACADOS_MINSTEP` 警告；本轮先作为行为诊断记录，不把这些 N=1 结果作为论文排序结论。

## 6. 关键观察

### 6.1 slosh cost 确实被激活

`B_smooth` 的 slosh cost 为 0：

```text
B_smooth: J_slosh mean = 0.0000, slosh pct mean = 0.00%
```

`B_slosh` / `B_ours` 的 slosh cost 非零：

```text
B_slosh w=1.5:  J_slosh mean = 0.0337, slosh pct mean = 7.09%
B_slosh w=2.2:  J_slosh mean = 0.1112, slosh pct mean = 11.97%
B_slosh w=2.75: J_slosh mean = 0.0232, slosh pct mean = 4.22%

B_ours w=1.5:   J_slosh mean = 0.0648, slosh pct mean = 8.99%
B_ours w=2.2:   J_slosh mean = 0.0827, slosh pct mean = 8.37%
B_ours w=2.75:  J_slosh mean = 0.0784, slosh pct mean = 5.58%
```

说明 slosh cost 不只是配置项，确实进入了 `/spmpc/cost_breakdown`。

### 6.2 w_slosh 会显著改变控制行为

`B_slosh` 的平均线速度和角速度随权重变化很大：

```text
B_slosh w=1.5:  cmd_v mean = 0.028, abs omega mean = 0.130, 未完成
B_slosh w=2.2:  cmd_v mean = 0.510, abs omega mean = 0.400, 完成
B_slosh w=2.75: cmd_v mean = 0.019, abs omega mean = 0.073, 未完成
```

`B_ours` 也出现类似现象：

```text
B_ours w=1.5:   cmd_v mean = 0.450, abs omega mean = 0.317, 完成
B_ours w=2.2:   cmd_v mean = 0.460, abs omega mean = 0.327, 完成
B_ours w=2.75:  cmd_v mean = 0.026, abs omega mean = 0.096, 未完成
```

因此可以确认：

```text
slosh cost / w_slosh 不是“没生效”，它确实会改变 cmd_vel 行为。
```

### 6.3 行为不是单调变好，而是存在失效/过保守区间

本轮最重要的现象不是“哪个数最小”，而是：

```text
B_slosh w=1.5 和 w=2.75 都没有完成，机器人平均速度非常低，final distance 约 7.3~7.5 m。
B_ours w=2.75 也没有完成，表现为强烈保守/停滞。
```

所以当前不能直接扩大 N=5。需要先避免把明显会导致停滞的权重放入正式统计。

### 6.4 `B_slosh w=2.2` 有强响应，但不等于最终候选

`B_slosh w=2.2` 本轮成功完成，并且 slosh peak 明显低于 `B_smooth`：

```text
B_smooth:       peak = 36.18 mm, p95 = 11.02 mm
B_slosh w=2.2: peak = 21.50 mm, p95 = 8.06 mm
```

但它的 tracking RMS 更高：

```text
B_smooth:       tracking RMS = 0.172 m
B_slosh w=2.2: tracking RMS = 0.202 m
```

且这是 N=1，不应直接作为最终 ranking。

### 6.5 `B_ours w=2.2` 的 full-window peak 被起步 lurch 污染

`B_ours w=2.2` 本轮完成且 p95 低于 `B_smooth`：

```text
B_ours w=2.2: full-window peak = 35.78 mm, p95 = 10.04 mm
```

如果只看全程 peak，`B_ours` 会显得和 `B_smooth` 接近。但对 P2_s_curve 的路径形状复查后发现：急转弯集中在起点附近，和“从静止起步猛冲”的瞬态几乎重叠。因此全程 peak 中有很大一部分是起步 lurch，而不一定代表行进/过弯阶段的 slosh-aware 效果。

按“去掉前 2s”的 post-start 口径复核 `w_slosh=2.2` 后：

```text
variant   full-window peak mm   exclude-first-2s peak mm
B_smooth  36.2                  22.5
B_slosh   21.5                  14.1
B_ours    35.8                  12.2
```

这个结果说明：

```text
1. 起步 lurch 对 full-window peak 的污染很大，尤其是 B_ours 从 35.8 mm 降到 12.2 mm。
2. 排除起步前 2s 后，slosh-aware 变体相对 B_smooth 的抑晃效果能拉开。
3. B_ours 的真实行进阶段效果不是没有，而是被起步瞬态峰值盖住了。
```

注意：这个 post-start 结果仍是诊断性 `N=1` 观察，不是 formal ranking；而且 P2_s_curve 的急转贴着起点，不能把该口径称为“纯 S 弯段”。更准确的名字是 `exclude-first-2s` 或 `post-start` 指标。

## 7. 当前结论

本轮可以得出的结论：

```text
1. B_slosh 存在且应作为核心 ablation；上一轮没有跑 B_slosh，因此不能只用 B_ours≈B_smooth 判断 slosh cost 无效。
2. slosh cost 已经进入 cost_breakdown，且 w_slosh 会显著改变 cmd_vel、加速度、omega-rate 与完成行为。
3. slosh-aware 项不是没生效；问题是权重/组合策略存在敏感区间，部分权重会让 planner 过于保守或停滞。
4. P2_s_curve 的 full-window peak 被起步 lurch 明显污染，尤其是 B_ours w=2.2 的 35.8 mm 主要由起步瞬态拉高。
5. 去掉前 2s 后，w=2.2 的 slosh-aware 变体能拉开 B_smooth：B_slosh 14.1 mm、B_ours 12.2 mm，对比 B_smooth 22.5 mm。
6. 因此 B_ours 不是“没有抑晃效果”，而是 full-window peak 把起步猛冲和后续行进阶段混在了一起。
```

当前不能得出的结论：

```text
1. B_slosh w=2.2 已经是最终最优。
2. B_ours 已经显著优于 B_smooth 的 formal 结论；post-start 结果仍是 N=1 诊断观察。
3. 可以直接扩大所有权重到 N=5。
4. w_slosh 越大越好。
5. exclude-first-2s 指标等同于“纯 S 弯段”指标；P2_s_curve 的转弯贴近起点，二者不能混用。
```

## 8. 下一步建议

不要直接 N=5。更稳的下一步是先把指标口径和复现实验分清楚：

```text
1. full-window peak 和 exclude-first-2s/post-start peak 必须同时保留；前者代表完整运输风险，后者用于排除起步 lurch 后观察行进阶段抑晃。
2. 不要把 P2_s_curve 的曲率窗口称为“纯 S 弯段”；这条路径急转在最前面，转弯段和起步段重叠。
3. 去掉明显停滞的 w=2.75 作为正式候选。
4. 对 B_slosh / B_ours 重点检查 w=2.0~2.4 附近，例如 2.0 / 2.2 / 2.4。
5. 做干净同批 fresh-sim N≥3，对比 B0 / B_smooth / B_slosh@2.2 / B_ours@2.2，并同时报告 full-window 与 exclude-first-2s 指标。
6. 如果外部 TEB/DWA 仍然 post-start peak 更低，不能靠窗口解释硬拉胜负；需要用 success / duration / tracking / slosh 的 Pareto 口径说明其慢和保守。
```

一句话：

> 这轮确认了 slosh cost 确实会改变行为；新增 post-start 复核进一步说明真实抑晃效果被起步 lurch 盖住了一部分。现在的问题不是“有没有生效”，而是“如何同时控制起步瞬态、完成率和行进阶段抑晃”，因此仍不应直接 N=5。
