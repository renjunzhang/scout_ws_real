# 2026-05-06 Online GeoRef 阶段性总结

## 1. 当前结论

在仿真 open 场景、假设 `/slosh/height` 可代表液面晃动的前提下，当前最有效的方案已经从“把晃动代价塞进 MPC”转为：

```text
MBF global path
  -> online anti_slosh_path_post_processor
  -> low-excitation geometry candidate selection
  -> ordinary MPC tracking
```

核心点是：防晃不放在 MPC 代价函数或 cmd_vel 后处理里，而是前移到几何参考生成层；MPC 只负责跟踪已经更低激励的参考路径。

当前主实验目标为：

```text
open_user_goal:
  x=-3.1570560932159424
  y=-2.897411346435547
  qz=-0.978164583074326
  qw=0.2078317791364693
```

## 2. 控制结构

实验中 MPC 内部防晃项全部关闭：

```text
Q_slosh=0
Q_slosh_eta_dot=0
risk_scheduler_enable=false
energy_profile_enable=false
input_shaping_enable=false
```

统一跟踪参数：

```text
mpc_R_a=1.0
mpc_R_da=2.0
mpc_cmd_vel_lead_time=0.05
vehicle_v_max=2.0
```

这意味着当前正结果不能写成“MPC 代价函数主动抑制晃动”。更准确的表述是：

```text
低激励几何参考生成降低了 MPC 需要执行的曲率/加速度/jerk 激励，
普通 MPC 在约束下跟踪该参考后，模型估计液面晃动下降。
```

## 3. 主结果

`RAW_TUNED` vs `GEOREF_TUNED`，每组 3 包：

```text
RAW:
  active_s=9.07
  h_rms=0.001043
  h_p95=0.002238
  h_max=0.002577
  eta_dot_rms=0.004813
  energy_rms=0.01856
  ay_p95=0.22778
  ax_p95=1.17269
  alpha_p95=1.15520

GEOREF:
  active_s=9.57
  h_rms=0.000847
  h_p95=0.001821
  h_max=0.002407
  eta_dot_rms=0.004261
  energy_rms=0.01518
  ay_p95=0.14011
  ax_p95=0.95575
  alpha_p95=0.99689
```

相对变化：

```text
active_s       +5.5%
h_rms          -18.7%
h_p95          -18.6%
h_max          -6.6%
eta_dot_rms    -11.5%
energy_rms     -18.2%
ay_p95         -38.5%
ax_p95         -18.5%
alpha_p95      -13.7%
ref_ay_p95     -42.0%
ref_ax_p95     -91.4%
ref_jerk_p95   -80.5%
```

阶段性判断：

```text
Online GeoRef 在 open_user_goal 上实现了可复现的仿真正结果。
它不是靠明显拖慢任务换晃动下降，active_s 增加约 5.5%，仍在 +15% 门槛内。
```

## 4. Baseline 结论

### 4.1 RAW_SLOW_MATCHED

`RAW_SLOW_MATCHED` 使用原始 MBF global path，不启用 GeoRef，只把 `vehicle_v_max` 降到 1.90 以接近 GeoRef 的运行时间。

三组均值：

```text
SLOW:
  active_s=9.37
  h_rms=0.000948
  h_p95=0.002201
  h_max=0.003144
  eta_dot_rms=0.006767
  energy_rms=0.01766
```

`SLOW` vs `RAW`：

```text
active_s       +3.3%
h_rms          -9.1%
h_p95          -1.7%
h_max          +22.0%
eta_dot_rms    +40.6%
energy_rms     -4.8%
```

`GEOREF` vs `SLOW`：

```text
active_s       +2.1%
h_rms          -10.7%
h_p95          -17.3%
h_max          -23.4%
eta_dot_rms    -37.0%
energy_rms     -14.1%
```

结论：

```text
简单降速不能解释 GeoRef 的收益。
慢速 baseline 只轻微降低 h_p95，并使 h_max/eta_dot 明显变差。
```

### 4.2 GEOREF_ORIGINAL

修正后的 `GEOREF_ORIGINAL` 仍走 post-processor 话题链路，但 selected=original 时直接转发原始 MBF path，不重采样。

语义检查：

```text
run01: /scout/global_path 297 pts, /scout/global_path_anti_slosh 297 pts
run02: /scout/global_path 295 pts, /scout/global_path_anti_slosh 295 pts
run03: /scout/global_path 298 pts, /scout/global_path_anti_slosh 298 pts
```

三组均值：

```text
ORIGINAL_FIXED:
  active_s=9.72
  h_rms=0.000929
  h_p95=0.002225
  h_max=0.002977
  eta_dot_rms=0.006693
  energy_rms=0.01738
```

`ORIGINAL_FIXED` vs `RAW`：

```text
active_s       +7.2%
h_rms          -10.9%
h_p95          -0.6%
h_max          +15.5%
eta_dot_rms    +39.1%
energy_rms     -6.3%
```

`GEOREF` vs `ORIGINAL_FIXED`：

```text
active_s       -1.5%
h_rms          -8.8%
h_p95          -18.1%
h_max          -19.1%
eta_dot_rms    -36.3%
energy_rms     -12.7%
```

结论：

```text
GeoRef 收益不是 post-processor topic chain 或 original fallback 带来的。
收益来自 geometry smoothing candidate selection。
```

## 5. 失败边界

### 5.1 Maze 不作为当前主验证场景

maze same-goal 中，raw tuned 本身也出现现场碰墙。也就是说：

```text
raw 都不安全
GeoRef 再优化也没有干净对照
```

该场景混入了避障、tracking safety、clearance margin 问题，不适合验证 anti-slosh 主效果。

maze 当前只能作为未来扩展边界：

```text
需要 collision-aware tracking-feasibility gate，
不只是 path point collision check。
```

### 5.2 不能宣称任意路径泛化

当前正结果主要来自 `open_user_goal`。另一个目标 `open_goal_b` 单包指标为正，但 selected=original，不能证明 smoothing candidate 泛化。

因此当前不能写：

```text
该方法已对任意 MBF 全局路径稳定有效。
```

只能写：

```text
在 open 场景代表性目标上，online GeoRef 相比 raw、matched slow 和 original-only baseline 显著降低模型估计晃动。
```

## 6. 论文表述建议

推荐贡献表述：

```text
提出一种面向液体搬运移动机器人的在线低激励几何参考后处理层。
该方法在 MBF 生成的全局路径上生成候选几何参考，
通过曲率、曲率变化、预测横向激励和碰撞 gate 选择低激励候选，
再由普通 MPC 跟踪该参考。
```

推荐方法名：

```text
Online Low-Excitation Geometric Reference Generation for Slosh-Aware MPC Tracking
```

或简称：

```text
Online GeoRef
```

需要避免的表述：

```text
不要写成“MPC 代价函数抑制晃动”。
不要写成“cmd_vel 后处理抑制晃动”。
不要写成“任意路径已稳定泛化”。
不要写成“保证每次峰值高度单调下降”。
```

## 7. 下一步

建议立即做：

```text
1. 整理一张主结果表：
   RAW / SLOW / ORIGINAL_FIXED / GEOREF

2. 更新论文提纲：
   把主线从“slosh-aware MPC cost”改为
   “reference-first GeoRef + MPC tracking”。

3. 提交当前代码与文档。
```

后续可做但不应阻塞当前收束：

```text
1. 修 maze tracking-feasibility/corridor safety gate。
2. 再找第二个 open 目标，要求 selected 非 original，再验证泛化。
3. 实物前补真实液面视觉与参数辨识。
```
