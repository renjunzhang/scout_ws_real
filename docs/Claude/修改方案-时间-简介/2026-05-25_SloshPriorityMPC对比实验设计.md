# 2026-05-25 SloshPriorityMPC 对比实验设计

## 0. 当前决策

```text
P1 ω²·d / ω̇·d 偏置耦合补偿：本轮不做。
P2 terminal residual 治理：本轮不做。
P3 Ferrari ζ：已离线对比，不接在线默认。

后续重点：
  固定当前 MPC 结构；
  用严格对照实验说明 modal slosh cost 和 ax/jerk 平滑项的作用；
  terminal approach 只作为独立诊断，不进入主效果统计。
```

## 1. 研究问题

本轮实验只回答三个问题：

```text
Q1: 只加入 modal slosh cost，是否能在真实 RGB 液面上降低晃动？
Q2: 只提高 ax/jerk 平滑优先级，是否也能降低晃动？
Q3: modal slosh cost + ax/jerk 平滑合在一起，是否取得最好的液面抑制，同时带来可解释的时间代价？
```

不再试图回答：

```text
1. Ferrari 物理阻尼是否应该在线替换当前 ζ=0.05；
2. 容器偏置耦合是否提升模型保真度；
3. terminal residual 是否能进一步降低终点残振。
```

这三项都可以写成 limitation / future work。

## 2. 实验分组

采用 2×2 因子设计：

```text
因子 A: modal slosh cost
  off: Q_slosh=0
  on : Q_slosh=5

因子 B: ax/jerk 平滑优先级
  normal : R_a=0.5, R_da=1.5
  strong : R_a=1.0, R_da=2.0
```

| 组别 | 名称 | slosh cost | ax/jerk 平滑 | 论文含义 |
|---|---|---:|---:|---|
| C | Smooth baseline | off | normal | 无晃动项的平滑 MPC baseline |
| D | Slosh cost only | on | normal | 证明 modal slosh cost 单独作用 |
| E | Ax/jerk only | off | strong | 区分“平滑控制本身”的贡献 |
| F | Slosh + ax/jerk | on | strong | 主方法：晃动模态项 + 激励源头平滑 |

G 组 `slosh_preview_factor>0` 暂不进入主论文表。已有结果没有显示它稳定改善 peak，后续只作为 appendix / negative ablation。

## 3. 样本量与录制顺序

最低可用：

```text
C/D/E/F 各 n=3
```

更稳妥：

```text
C/D/E/F 各 n=5
```

建议按 block 交错录制，避免电量、地面状态、相机光照随时间漂移污染组间差异：

```text
Block 1: C1 -> D1 -> E1 -> F1
Block 2: D2 -> F2 -> C2 -> E2
Block 3: F3 -> E3 -> D3 -> C3
```

如果扩到 n=5，再补两个 block，但不要只给“看起来好”的组补样本。

## 4. 固定条件

所有组必须保持一致：

```text
路径：P2_s_curve 固定路径
MPC 输入：global_path_topic:=/scout/global_path_fixed
terminal：d200 当前稳定配置
RGB：同一天相机参数、同一天三标尺、同一 HSV
评价窗口：TRACKING start -> first terminal - 1.0s
主 RGB 指标：max(left, center, right)
模型参数：damping_ratio=0.05，不启用 Ferrari ζ 在线替换
OSCRS / GeoRef / post-processor：全部不参与
```

terminal approach 的窗口单独统计：

```text
first terminal - 1.0s -> terminal / reached
```

它只用于解释终点 jerk 和残振，不用于证明 slosh cost 主效果。

## 5. 指标

### 5.1 主论文窗口指标

```text
RGB:
  p95
  RMS
  peak
  AUC_τ = ∫ max(0, h_vis - τ) ds,  τ = 0.5 mm
    比 p95 对间歇超阈值更敏感，比 peak 更稳健；
    积分在路径进度 s 上而非时间 t 上（见 5.3 节）。

model:
  /slosh/height p95
  /slosh/height peak

motion:
  odom_v mean / peak
  ax_p95
  ay_p95
  jerk_p95
  duration

cost:
  pct_slosh
  pct_v
  pct_control
  pct_smooth
```

### 5.2 Ferrari-style 指标

```text
gamma_opt vs C
gamma_model
RMSE
corr
U_p95 / U_max
A_rank
```

主结论以 RGB 为准；`/slosh/height` 只作为模型侧解释。

### 5.3 路径进度对齐（s 轴 vs t 轴）

TOPPRA / Ruckig 本质是改 timing：同一条路径用不同速度走过。
如果只在时间轴上比较峰值位置，"更晚到达拐弯"会被误认成"晃动被抑制"。

因此主图和主表的**主轴必须是归一化路径进度 s ∈ [0, 1]**，时间轴只做辅轴。

实现：

```text
1. odom (x, y) 投影到 fixed path 得 s(t)
2. 视觉液面 h_vis(t)、模型 /slosh/height(t) 重采样到统一 progress grid（101 个点）
3. 主曲线图横轴用 s，纵轴用 h_vis
4. AUC_τ 的积分变量也用 ds 而非 dt
```

progress-aligned 曲线图画 `median + IQR`（四种方法叠在同一张图）。

## 6. 主要比较关系

```text
D - C:
  modal slosh cost 单独贡献。

E - C:
  ax/jerk 平滑项单独贡献。

F - E:
  在同等强平滑下，modal slosh cost 的额外贡献。

F - D:
  在同等 slosh cost 下，ax/jerk 平滑项的额外贡献。

F - C:
  完整 SloshPriorityMPC 目标相对 baseline 的总收益。
```

论文表述建议：

```text
D 组回答“晃动模态项是否有效”；
E 组回答“是否只是更平滑导致液面下降”；
F 组回答“提出的 slosh-priority objective rebalancing 是否最好”。
```

## 7. 公平性约束：±10% completion time 带

如果某方法的 completion time 与 baseline 相差超过 ±10%：

```text
结论不能写"纯 anti-slosh 优势"，
必须写成"在更保守的控制行为下降低了液面风险"（trade-off 语言）。
```

具体判定：

```text
若 (t_F - t_C) / t_C > 10%:
  不写"F strictly dominates C"
  写  "F reduces sloshing at a xx% time cost"

若所有方法 completion time 差异 <= 10%:
  可写  "comparable task efficiency with reduced sloshing"
```

该阈值适用于正文主表所有方法之间的两两比较。
如果 TOPPRA/Ruckig 因为约束保守导致比 Ours 更慢 >10%，也应标注。

## 8. 成功标准

最低成功标准：

```text
1. D 相对 C:
   RGB p95 下降，且 model p95 同向下降。

2. F 相对 C:
   RGB p95 / RGB peak 均下降。

3. F 相对 E:
   RGB p95 或 peak 继续下降，说明 slosh cost 不是摆设。

4. duration:
   F 允许变长，但必须报告 trade-off（见 §7 ±10% 规则）。
```

强成功标准：

```text
1. D-C、F-E 两个差分都支持 slosh cost 有增量作用；
2. F 的 RGB peak 最低；
3. ax/jerk 指标解释 F 的收益来源；
4. terminal 诊断没有污染主窗口。
```

如果只满足 F-C，不满足 D-C 或 F-E，则不能写“slosh cost 单独有效”，只能写“slosh-priority rebalancing 有效”。

## 9. 统计检验协议

4 方法 matched block → 非参数检验为主。

主流程：

```text
1. 对每个主指标（RGB p95, AUC_0.5mm, RGB peak, duration）做 Friedman 检验
2. 若 Friedman 显著（p < 0.05）:
   以 Ours 为参照，做 3 个配对 Wilcoxon signed-rank post hoc
   用 Holm 校正控制多重比较
3. 每个配对报告：
   p-value（Holm-corrected）
   效应量：rank-biserial correlation（Wilcoxon 对应）
```

两方法直接比较（如 E vs F 单独做一列）：

```text
直接配对 Wilcoxon signed-rank
若配对差值近似正态 → supplementary 补配对 t-test + Cohen's dz
```

progress curve 不逐 bin 做检验（多重比较问题太严重），只做描述性展示（median + IQR）。
真正进统计表的是 p95 / AUC / RMS / peak 这类**单 run 标量**。

样本量与检验力：

```text
n=5:  能检出 dz ≈ 1.3 的大效应（配对 Wilcoxon 双侧 α=0.05, power≈0.80）
n=10: 能检出 dz ≈ 1.0
n=5 对于"RGB p95 下降 30%"量级的效应足够；
若效应小于 15%，建议扩到 n=8-10。
```

## 10. 录包 protocol：static bag + block 交错

每个 block 前后各录一个 **10-15 s static bag**（液体静置、机器人不动）。

用途：

```text
1. 检查视觉证据链的 jitter floor（static jitter 应 < 0.08 mm）
2. 检查 bias drift（前后 static bag 的 mean 差应 < 0.05 mm）
3. 检查曝光 / 白平衡漂移（RGB 帧亮度一致性）
4. 作为 Ferrari γ_model 中 zero-correction 的参考
```

这是视觉证据链的唯一保险丝。如果 static bag 显示 jitter 或 drift 异常，
该 block 内的 motion bag 视觉数据降级为辅助证据，不能作为主结论。

录制命令（复用现有脚本）：

```bash
SLOSH_BAG_DIR=<bag_dir> SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 0 block_N_static_pre
# ... 跑 4 个 motion bag ...
SLOSH_BAG_DIR=<bag_dir> SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 0 block_N_static_post
```

run 间等待：每次回到起点后**静置 30-60 s**，让液体恢复近似同初态。

## 11. 分析输出

每批实验输出：

```text
1. group_metrics_main_window.csv
2. reduction_vs_C_main_window.csv
3. ferrari_indices_per_bag.csv
4. ferrari_indices_group_summary.csv
5. terminal_approach_1s.csv
6. cost_breakdown_group_summary.csv
7. paired_stats_summary.csv             （Friedman + Wilcoxon + effect size）
8. static_bag_jitter_drift.csv          （每 block 的 static bag 诊断）
```

图：

```text
Figure 1: progress_aligned_main_window   （s 轴，median + IQR，4 方法叠画）
Figure 2: group_metrics_main_window
Figure 3: reduction_vs_C_main_window
Figure 4: tradeoff_scatter               （横轴 duration，纵轴 RGB p95 或 AUC_0.5mm）
Figure 5: terminal_diagnostic
Figure 6: model_vs_rgb_scatter
Figure 7: representative_runs_overlay
```

## 12. 不做的事

```text
1. 不再加 P1 偏置耦合。
2. 不再加 P2 terminal residual。
3. 不把 Ferrari ζ 接成在线默认。
4. 不靠增大 Q_slosh 到极端值制造效果。
5. 不把 terminal approach 混入主效果窗口。
6. 不用 /slosh/height 替代 RGB 真值。
```

