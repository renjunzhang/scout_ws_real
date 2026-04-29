# 2026-04-29 Anti-Slosh MPC 阶段性总结

## 1. 当前结论

在假设 `/slosh/height` 等模型估计量可代表真实液面晃动的前提下，当前工作已经得到清晰阶段结论：

```text
MPC 内部 slosh soft cost 路线没有形成稳定有效的 anti-slosh controller。
输出层/参考层方法能在 P2_s_curve 上取得局部正结果。
但所有方法都没有在 P3_mixed 上形成可泛化通过证据。
```

因此，当前论文不应写成“提出的 anti-slosh MPC controller 已稳定有效”。更稳妥的定位是：

```text
差速移动机器人液体晃动感知 MPC 的建模、消融验证与失效机理分析。
```

## 2. 已确认的正结果

满足“tracking 不显著减速（+15% 内）+ `h_rms/h_p95/energy/eta_dot` 下降 + `ay_p95` 不升”的结果只出现在 P2_s_curve。

### P2_s_curve：PROFILE_SAFE 中等强度

```text
tracking_time_s        +12.7%
h_rms                  -22.5%
h_p95                  -20.8%
modal_energy_norm      -21.7%
eta_dot                -11.9%
odom_ay_abs_p95        -24.2%
verdict: PASS
```

限制：

- 只在 P2 通过；
- P3_mixed 后续未泛化；
- 属于参考速度剖面层方法，不是 MPC 内部 slosh cost 的成功。

### P2_s_curve：PROFILE_SELECTIVE 强参数

```text
tracking_time_s        +13.0%
h_rms                  -13.8%
h_p95                  -10.3%
modal_energy_norm      -14.6%
eta_dot                -27.7%
odom_ay_abs_p95        -4.2%
```

说明：

- 按“ay_p95 不升”口径可视为 P2 正结果；
- 当时按更严格的 “ay_p95 至少下降 5%” 口径记为 `FAIL(ay_p95)`；
- P3_mixed 后续未泛化。

### P2_s_curve：OUTPUT_GUARD

```text
tracking_s             -7.1%
h_rms                  -12.9%
h_p95                  -10.9%
energy                 -12.5%
eta_dot                -7.7%
ay_p95                 -34.1%
verdict: PASS
```

说明：

- 这是当前最干净的 P2 正结果；
- 不靠大幅减速，tracking 反而更快；
- 但 P3_mixed 不通过，不能作为通用 controller。

## 3. 已否决路线

### MPC 内部 slosh cost

已测试或分析过：

```text
Q_slosh
Q_slosh_eta_dot
terminal_factor_slosh_eta
terminal_factor_slosh_eta_dot
Q_ay_pred
Q_modal_energy / ENERGY_WIN
R_da / R_domega 平滑加权
```

结论：

- 没有任何 MPC 内部 slosh cost 配置单独通过；
- 常见失败模式是 `h` 小幅下降但 `eta_dot` 上升，或激励不可比；
- soft cost 会与 tracking cost 竞争，容易改变激励相位，而不是稳定耗散液体能量。

### 反应式速度治理

`GOV_AY` 能降低部分 `odom_ay_abs_p95`，但不能稳定降低 `h/energy/eta_dot`。

结论：

```text
只按当前 |v·omega| 反应式削速度，容易削错相位或延长激励时间。
```

### 固定阈值输出层 guard

`OUTPUT_GUARD` 在 P2 有效，但 P3 失败。

P3 诊断说明：

- guard 不是没激活；
- 峰前 `cmd_ay/odom_ay` 也有下降；
- 但 `cmd_dwz_abs_integral` 没有下降，tracking 方式被改变；
- 固定阈值 guard 没有稳定降低 P3 模态能量输入。

### PMG: Predictive Modal Guard

离线阶段曾出现强正信号：

```text
lateral-only PMG:
  P3 h_p95 +0.6%，否决

longitudinal PMG:
  P3 NOM h_p95 -18.14%
  P2 NOM h_p95 -6.75%

combined PMG:
  P3 NOM h_p95 -19.00%
  P2 NOM h_p95 -12.98%
  omega_n / zeta ±20% 鲁棒
```

但闭环 P2 ablation 否决了 PMG 作为通用 controller：

```text
P2 PMG combined:
  h_p95       -14.4%
  eta_dot     +51.2%

P2 PMG_LONG:
  h_p95       -13.4%
  eta_dot     +48.0%

P2 PMG_LAT:
  h_p95       +5.0%
  eta_dot     -16.0%
```

关键机理：

```text
PMG signed cap 只约束 eta 峰值，不约束 eta_dot。
二阶振子中，限制位移峰值可能把能量推向模态速度。
闭环 MPC 还会为追踪路径重新产生 ax/omega 脉冲，进一步放大 eta_dot。
```

最终决策：

```text
PMG 不进入实物主线。
不继续录 P3 PMG_LONG。
不写 PMG eta_dot 增强版。
不引入路径分类器。
PMG C++ 只保留为默认关闭消融入口。
```

## 4. P3_mixed 的核心问题

P3_mixed 是当前跨路径泛化失败的关键。

诊断结果：

```text
eta_x energy ratio ≈ 0.975
corr_ax_vref ≈ -0.11
corr_ax_track ≈ -0.62
status_fail_ratio = 0
```

解释：

- P3 主导晃动通道是 longitudinal `eta_x`，不是 lateral `eta_y`；
- lateral-only guard 或只压 `v*omega` 不可能触到主导模态；
- P3 的 `eta_x` 激励不像参考速度剖面直接造成，也不像 recovery/fallback 问题；
- 更像 MPC 主动决策或跟踪过程中的纵向加速度脉冲。

但即便针对 `eta_x` 的 PMG 在离线有效，也在 P2 闭环中暴露 `eta_dot` 转移问题，因此不能作为通用结构。

## 5. 对论文的影响

当前不建议把论文主线写成：

```text
我们提出的 anti-slosh MPC controller 能稳定降低液体晃动。
```

更合理的论文主线：

```text
构建 slosh-aware MPC 框架，系统评估 MPC 内部代价、参考层限速、输出层 guard 等方案；
揭示 soft cost 和 output cap 在跨路径液体晃动抑制中的失效机理；
给出未来真实液面感知与路径/执行层协同设计的依据。
```

可保留贡献：

- 差速底盘 8 维 tracking MPC + 2D 单模态 slosh 状态扩展；
- ROS/Gazebo 固定终点模板路径实验平台；
- `/slosh/state / height / eta_dot / modal_energy / excitation` 诊断体系；
- 多轮消融数据证明“单纯调大 slosh cost”不足；
- P2 局部正结果证明执行层/参考层确实可能降低模型估计晃动；
- P3 失败诊断揭示跨路径泛化问题。

必须避免的论文表述：

```text
不能说：方法稳定抑制真实液体晃动。
不能说：PMG 是有效 controller。
不能说：MPC 内部 slosh cost 已验证有效。
不能把 /slosh/height 直接等同于真实液面，除非补视觉真值验证。
```

可以写：

```text
在模型估计指标上，若干执行层/参考层策略可在 P2_s_curve 降低液面高度峰值与能量；
但这些策略未能在 P3_mixed 泛化，说明 slosh-aware MPC 的实际有效性受路径几何、模态通道和执行激励相位强烈影响。
```

## 6. 下一阶段建议

优先级最高：

```text
真实液面视觉测量。
```

原因：

- 当前所有结论都依赖 `/slosh/height` 模型估计；
- 如果真实液面与模型估计在高动态下不一致，继续优化 controller 只是在优化 proxy；
- PMG 失败已经说明仅优化模型内部 `eta` 峰值可能误导，需要真实液面验证。

并行可做：

- `omega_n / zeta` 专项自由衰减辨识；
- P3_mixed 路径可达性与参考几何审查；
- 整理当前 diff，按默认关闭消融入口、离线诊断工具、文档结论、无关漂移分组提交。

## 7. 当前工程状态

可保留：

- 默认关闭的 slosh cost / speed cap / governor / output guard / PMG 消融入口；
- `offline_pmg_replay.py`；
- `diagnose_p3_failure_modes.py`；
- `extract_slosh_metrics.py` 扩展；
- 固定终点模板路径录包流程；
- 04-27、04-28、04-29 文档结论。

不应进入默认实物主线：

- `Q_slosh` 系列作为主 controller；
- `PMG` 作为主 controller；
- 路径特异 PMG_LONG/PMG_LAT 分类器；
- 未经真实液面验证的“晃动抑制有效”声明。

