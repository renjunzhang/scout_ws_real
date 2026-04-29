# 2026-04-28 Anti-Slosh 阶段结论与后续方案

更新时间：2026-04-28

本文件接续 `2026-04-27_Anti-Slosh_方向修正.md`。
04-27 文件保留完整排查链；从 2026-04-28 起，后续方向以本文件为准。

## 1. 阶段结论

当前结论：

```text
纯 MPC 内部 slosh soft-cost 路线没有形成稳定有效方案。
最终执行激励层是关键，但固定阈值 OUTPUT_GUARD 只在 P2_s_curve 有效，P3_mixed 不泛化。
```

更具体地说：

- `Q_slosh / Q_slosh_eta_dot / terminal_factor_slosh_*`：不能稳定降低 `h_rms / h_p95 / energy / eta_dot`。
- `R_da / R_domega` 全局加重：会改变过弯方式，可能让 `odom_ay / odom_kappa` 反向恶化。
- `GOV_AY`：能削当前 `odom |v·ω|`，但不能保证削在正确相位，也不能保证液体模态能量下降。
- `AY_COST / ENERGY_WIN`：预测域内 ay 或模态能量代价有局部正向信号，但复现不稳定，不能作为实物主线。
- `PROFILE_SAFE / PROFILE_SELECTIVE / PROFILE_WINDOW / PROFILE_RISK`：把抑制前移到速度剖面层后有正向信号，但存在时间代价、路径泛化或 ay_p95 不过线问题。
- `OUTPUT_GUARD`：P2_s_curve 首次稳定 PASS，但 P3_mixed 主段 FAIL，说明固定阈值输出 guard 不能作为最终主线。

## 2. 关键证据

### 2.1 P2_s_curve：OUTPUT_GUARD 有效

有效 bag：

```text
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_OUTPUT_GUARD_run04_210756.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_OUTPUT_GUARD_run05_211217.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_OUTPUT_GUARD_run06_211517.bag
```

对比基线：

```text
/data/a/slosh_bags/sim/20260427/20260427_P2_s_curve_NOM_run16_221711.bag
/data/a/slosh_bags/sim/20260427/20260427_P2_s_curve_NOM_run17_221911.bag
/data/a/slosh_bags/sim/20260427/20260427_P2_s_curve_NOM_run18_222058.bag
```

结果：

```text
OUTPUT_GUARD 3 PASS
tracking_s: 5.900 -> 5.483  (-7.1%)
h_rms:      3.092 -> 2.693 mm  (-12.9%)
h_p95:      6.105 -> 5.438 mm  (-10.9%)
energy:     0.054979 -> 0.048123  (-12.5%)
eta_dot:    14.333 -> 13.224 mm/s  (-7.7%)
ay_p95:     2.516 -> 1.659  (-34.1%)
```

解释：

- 输出层限制 `cmd_v * cmd_omega` 和 `cmd_domega` 可以直接压低最终执行激励。
- P2 结果说明前面多轮失败不是因为 `Q_slosh` 简单不够大，而是 MPC 内部代价没有稳定管住最终执行激励。

### 2.2 P3_mixed：OUTPUT_GUARD 不泛化

有效 bag：

```text
/data/a/slosh_bags/sim/20260428/20260428_P3_mixed_OUTPUT_GUARD_run01_213032.bag
/data/a/slosh_bags/sim/20260428/20260428_P3_mixed_OUTPUT_GUARD_run01_213321.bag
```

对比基线：

```text
/data/a/slosh_bags/sim/20260427/20260427_P3_mixed_NOM_run03_235358.bag
/data/a/slosh_bags/sim/20260427/20260427_P3_mixed_NOM_run04_235531.bag
/data/a/slosh_bags/sim/20260427/20260427_P3_mixed_NOM_run05_235702.bag
```

主段口径：

```text
/mpc_status == TRACKING && /terminal/mode == NONE
```

结果：

```text
OUTPUT_GUARD 2 FAIL(h_rms,h_p95,energy,eta_dot,ay_p95)
tracking_s: +3.3%
h_rms:      +2.8%
h_p95:      +0.7%
energy:     +3.1%
eta_dot:    +6.9%
ay_p95:     +12.8%
kappa_p95:  -5.8%
```

解释：

- guard 确实激活：`output_guard_ratio=0.24~0.28`。
- 但 P3 主段中液体指标和 `ay_p95` 没有下降，反而上升。
- 因为分析已排除 terminal/recovery，所以该失败不能归因于“没 reached”。
- 固定阈值输出 guard 对不同路径形状不稳定，不能直接进入实物。

## 3. 当前最可信的问题原因

### 事实

- `/slosh/height` 是模型估计量，不是真实液面测量。
- 当前 MPC 内部 slosh 代价只能影响预测域变量，不能保证最终发布的 `cmd_vel` 激励被约束。
- 实际液体激励更直接来自最终执行层：

```text
cmd_v * cmd_omega
cmd_domega
odom v * odom omega
实际 odom 曲率
```

- 多轮实验显示：降低单一指标，如 `eta`、`eta_dot`、`ay_pred` 或当前 `odom ay`，不一定降低 `h_rms / h_p95 / energy`。

### 推测

- 主要矛盾是相位/频率问题，而不是单纯幅值问题。
- 固定阈值 `OUTPUT_GUARD` 在 P2 有效，是因为 P2 的激励相位与路径结构刚好匹配；在 P3_mixed 中，限制同样的输出幅值可能改变通过方式和激励持续时间，导致能量不降反升。
- 参考路径曲率与实际 odom 曲率之间存在偏差，导致基于 reference κ 的前馈策略不可靠。

### 待验证

- 是否存在一种“相位/窗口感知”的输出 guard，可以保留 P2 的直接激励约束优点，同时避免 P3 的泛化失败。
- 是否需要先修 P3_mixed 的路径可达性和终点逻辑，再把它作为正式验证路径。
- 实物红色视觉液面指标是否与 `/slosh/height` 在趋势上足够一致。

## 4. 停止事项

暂时停止：

- 不继续扫 `Q_slosh / Q_slosh_eta_dot / terminal_factor_slosh_*`。
- 不继续扫 `Q_ay_pred / Q_modal_energy / modal_energy_window`。
- 不继续扫 `GOV_AY threshold`。
- 不继续扫 `OUTPUT_AY_GUARD_LIMIT / OUTPUT_AY_GUARD_DOMEGA_LIMIT`。
- 不继续录 P3_mixed `OUTPUT_GUARD` 完整 reached bag。
- 不把当前 diff 直接提交成最终主线。

保留：

- 现有参数入口、debug topic、分析脚本和录包脚本。
- 这些内容作为消融和诊断资产保留，但不宣称为最终 anti-slosh 控制方案。

## 5. 后续方案

### 5.1 目标

下一阶段不再问：

```text
哪个单一权重能让 /slosh/height 降？
```

改为问：

```text
怎样让最终执行激励在正确时机、正确窗口内被约束，同时不破坏路径跟踪？
```

### 5.2 推荐方向：相位/窗口感知的执行激励约束

在 `OUTPUT_GUARD` 基础上升级，但不直接扫阈值。

候选机制：

1. 只在预计会进入高响应窗口时启用 guard。
2. guard 触发依据不只看当前 `cmd_v * cmd_omega`，而是看未来短窗口内的预测 slosh 能量趋势。
3. 同时限制 `omega` 和 `domega`，但避免在所有路径段全程固定削顶。
4. 记录每次 guard 激活的原因：`ay_limit`、`domega_limit`、`energy_window`、`phase_window`。

最小实现原则：

- 默认关闭。
- 不改 NOM/FAS/GOV/PROFILE 旧入口行为。
- 不新增硬约束进 QP，先放在发布前输出层或参考速度层。
- 每个新机制必须有 debug topic 和 config summary。

### 5.3 先做离线诊断，不先写新控制器

下一步先用已有 bag 做离线诊断：

```text
P2 OUTPUT_GUARD PASS 组
P3 OUTPUT_GUARD FAIL 组
P3 NOM 基线组
```

诊断问题：

- P2 中 guard 激活发生在 height/energy 峰值前多久？
- P3 中 guard 激活是否发生在错误相位？
- P3 中 `cmd_ay` 被限制后，是否出现更长持续时间的中等激励？
- P3 中 `eta_dot` 上升是否来自 `domega`、`omega` 持续时间，还是路径跟踪偏差？

如果离线诊断不能解释 P2/P3 差异，不写新控制器。

### 5.4 推荐新增离线分析脚本

新增或扩展 `analyze_slosh_peak_precursors.py`：

输入：

```text
bag list
peak-signal: height / energy / eta_dot
window: 峰值前 0.5s / 1.0s / 1.5s / 2.0s
```

输出：

```text
peak_time
guard_active_before_peak
guard_first_to_peak_s
cmd_ay_integral_before_peak
odom_ay_integral_before_peak
cmd_domega_rms_before_peak
odom_wz_duration_before_peak
energy_growth_rate_before_peak
track_dist_p95_before_peak
```

验收：

- 能解释为什么 P2 PASS、P3 FAIL。
- 能指出下一版 guard 应该提前、推迟、缩短持续时间，还是改触发变量。

## 6. 是否还要录 bag

当前阶段不需要继续盲目录 bag。

只在以下情况录：

- 离线诊断形成了明确的新机制假设；
- 代码只做一个小改动；
- 新实验矩阵最多 2 组，每组 2~3 包。

在形成新机制前，不再录：

```text
P2_s_curve 新参数扫参
P3_mixed OUTPUT_GUARD 继续重复
GOV_AY / ENERGY_WIN / PROFILE_* 新阈值
```

## 7. 成功标准

下一阶段若提出新机制，必须同时满足：

```text
P2_s_curve:
  h_rms / h_p95 / energy / eta_dot 至少下降 8%
  ay_p95 不升
  tracking_time 不增加超过 15%

P3_mixed 主段:
  h_rms / h_p95 / energy / eta_dot 不升
  ay_p95 不升
  track_dist_p95 不恶化超过 15%

工程:
  solve_success_ratio >= 0.97
  默认 launch 行为不变
  bag 有同名 .txt 和 /experiment/config_summary
```

只有同时满足 P2 和 P3 主段，才考虑实物低风险验证。

## 8. 当前优先级

1. 整理当前 diff，不提交为最终主线。
2. 离线分析 P2 PASS 与 P3 FAIL 的激活时序差异。
3. 若差异清晰，再设计 `OUTPUT_GUARD_V2`。
4. 若差异不清晰，暂停控制器改动，转向真实液面视觉指标和路径可达性问题。

## 9. 2026-04-29 离线诊断补充

已扩展 `analyze_slosh_peak_precursors.py`，加入峰前窗口内：

```text
output_guard_active_ratio
output_guard_first_lead_s
cmd_ay_abs_integral
odom_ay_abs_integral
cmd_dwz_abs_integral
odom_wz_abs_integral
height / energy / eta_dot delta
```

诊断对象：

```text
P2 OUTPUT_GUARD PASS:
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_OUTPUT_GUARD_run04_210756.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_OUTPUT_GUARD_run05_211217.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_OUTPUT_GUARD_run06_211517.bag

P3 OUTPUT_GUARD FAIL:
/data/a/slosh_bags/sim/20260428/20260428_P3_mixed_OUTPUT_GUARD_run01_213032.bag
/data/a/slosh_bags/sim/20260428/20260428_P3_mixed_OUTPUT_GUARD_run01_213321.bag
```

峰前 1s、去掉每包第一个初始 peak 后的均值：

```text
P2_NOM:
cmd_ay_abs_integral  2.315
odom_ay_abs_integral 1.404
cmd_dwz_abs_integral 2.164
track_dist_max       1.283
energy_delta         0.047

P2_OUTPUT_GUARD:
cmd_ay_abs_integral  2.026
odom_ay_abs_integral 1.026
cmd_dwz_abs_integral 1.383
track_dist_max       1.326
energy_delta         0.088
guard_ratio          0.492
guard_first_lead_s   0.642

P3_NOM:
cmd_ay_abs_integral  1.561
odom_ay_abs_integral 0.832
cmd_dwz_abs_integral 2.542
track_dist_max       0.546
energy_delta         0.048

P3_OUTPUT_GUARD:
cmd_ay_abs_integral  1.433
odom_ay_abs_integral 0.759
cmd_dwz_abs_integral 2.625
track_dist_max       0.610
energy_delta         0.051
guard_ratio          0.489
guard_first_lead_s   0.843
```

新增判断：

- P3 失败不是因为 guard 没激活；`guard_ratio≈0.49`，与 P2 相近。
- P3 失败也不是因为峰前 `cmd_ay / odom_ay` 完全没被削；这两个积分都下降。
- 关键差异是：P3 中 `cmd_dwz_abs_integral` 没有下降，`track_dist_max` 上升，`energy_delta` 略升。
- 因此固定阈值 guard 的问题不是“削得不够”，而是“削顶改变了通过方式/角速度持续作用/相位”，没有稳定降低模态能量输入。

后续更新：

```text
不设计简单调阈值的 OUTPUT_GUARD_V2。
若继续改代码，V2 必须针对 cmd_domega 积分、激励持续时间、tracking 误差和相位窗口，而不是只降低 ay_limit。
```

## 10. 2026-04-29 PMG 离线到闭环结论

PMG 路线的证据链：

```text
lateral-only PMG:
  P2 有小幅信号
  P3 h_p95 +0.6%，离线否决

D1:
  P3 eta_x energy ratio ≈ 0.975
  说明 P3 主导通道是 longitudinal，而不是 lateral

D2:
  corr_ax_vref ≈ -0.11
  corr_ax_track ≈ -0.62
  status_fail_ratio = 0
  说明 P3 eta_x 激励不像路径速度剖面或 recovery 直接造成，更像 MPC 主动决策形成的 ax 脉冲

D3 longitudinal replay:
  P3 NOM h_p95 -18.14%
  P2 NOM h_p95 -6.75%

combined replay:
  P3 NOM h_p95 -19.00%
  P2 NOM h_p95 -12.98%
  omega_n / zeta ±20% 鲁棒性通过
```

闭环验证后，PMG controller 路线被否决为通用方案。

P2 闭环结果：

```text
P2 PMG combined:
  h_rms       -11.3%
  h_p95       -14.4%
  energy      -5.7%
  eta_dot     +51.2%
  ay_p95      -21.7%
  solve_success_ratio = 1.0

P2 PMG_LONG:
  h_p95       -13.4%
  eta_dot     +48.0%
  active_x    0.050

P2 PMG_LAT:
  h_p95       +5.0%
  eta_dot     -16.0%
  active_y    0.010
```

关键判断：

- `PMG_LONG` 是 P2 `eta_dot` 上升的主要来源；
- `PMG_LAT` 不抬高 `eta_dot`，但不能降低 `h_p95`；
- combined 在 P2 上由 x 通道主导，因此不能作为 P2/P3 统一默认 controller；
- PMG 的 signed cap 目标只约束 `eta` 峰值，没有约束 `eta_dot`，会把二阶模态能量从位移项推到速度项；
- 这不是简单阈值或触发时机问题，而是 cap 目标本身的结构性盲点。

离线 replay 可信度复核：

```text
P2 longitudinal replay:
  eta_dot: 0.0116 -> 0.0139 约 +19%

P2 combined replay:
  eta_dot: 0.0116 -> 0.0161 约 +39%

P2 closed-loop:
  PMG_LONG eta_dot +48%
  PMG combined eta_dot +51%
```

结论：

- 离线 replay 没有漏掉 `eta_dot` 风险，只是之前决策过度关注 `h_p95`；
- 闭环 MPC 会进一步放大 cap 后的 `eta_dot` 风险；
- 未来任何 output-cap 类机制，离线阶段必须把 `eta_dot` 和 modal energy 设为硬门槛，不能只看 `h_p95`。

最终决策：

```text
PMG 不进入实物主线。
不继续录 P3 PMG_LONG。
不写 PMG η_dot 增强版。
不引入路径分类器选择 PMG_LONG/PMG_LAT。
```

工程处理：

- PMG C++ 入口可保留为默认关闭的消融入口；
- 文档和提交信息必须明确：`offline positive, closed-loop P2 eta_dot FAIL, not validated for real deployment`；
- 后续主线应转向真实液面视觉测量、P3 路径可达性审查、或 `omega_n/zeta` 专项自由衰减辨识。

推荐下一步：

```text
优先做真液面视觉测量。
理由：当前所有控制策略都依赖 /slosh/height 模型估计；
若模型估计与真实液面在高动态下不一致，继续设计 controller 只是在优化内部 proxy。
```
