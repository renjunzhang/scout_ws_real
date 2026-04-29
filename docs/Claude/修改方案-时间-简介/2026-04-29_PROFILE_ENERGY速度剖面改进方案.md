# 2026-04-29 PROFILE_ENERGY 速度剖面改进方案

## 1. 背景

截至 2026-04-29，MPC 内部 slosh cost、输出层 guard 和 PMG 路线均未形成可泛化 controller。

已确认：

```text
MPC 内部 soft cost:
  Q_slosh / Q_eta_dot / terminal / modal_energy 均未稳定通过。

OUTPUT_GUARD:
  P2_s_curve 通过；
  P3_mixed 不泛化。

PMG:
  P2 能降低 h_p95，但 eta_dot 显著上升；
  不能进入实物主线。
```

同时，参考速度剖面层出现过最强的 P2 正结果：

```text
P2 PROFILE_SAFE 中等强度:
  tracking +12.7%
  h_rms -22.5%
  h_p95 -20.8%
  energy -21.7%
  eta_dot -11.9%
  ay_p95 -24.2%
```

这说明“提前降低可执行激励”方向有价值，但原 `PROFILE_SAFE / PROFILE_SELECTIVE / PROFILE_WINDOW / PROFILE_RISK` 的触发逻辑未能在 P3 泛化。

## 2. 当前核心判断

当前问题不是继续调 `Q_slosh`，而是速度/路径参考生成没有系统控制未来激励。

P3 失败诊断显示：

```text
eta_x energy ratio ≈ 0.975
```

说明只控制 lateral `ay = v^2*kappa` 不够，必须同时控制：

```text
longitudinal ax impulse
lateral ay
omega
domega / alpha_z
曲率符号切换与 dkappa 突变
```

因此下一版不应继续写 output cap，而应把抑制前移到 `PathHandler::updateSpeedProfile()`：

```text
PROFILE_ENERGY / EASP: Energy-Aware Speed Profile
```

## 3. 目标

目标是在路径参考生成层得到一个更平滑、低激励的 `v_ref(s)`，让 MPC 提前知道速度变化，而不是在 `publishCmdVel()` 最后硬裁剪。

成功目标：

```text
P2_s_curve 和 P3_mixed 均满足：
  tracking_time <= NOM * 1.15
  h_rms 下降
  h_p95 下降
  modal_energy_norm 下降
  eta_dot_rms 下降
  ay_p95 不升
  solve_success_ratio >= 0.97
```

如果只在 P2 通过、P3 不通过，则仍不能作为通用 anti-slosh controller。

## 4. 最小实现范围

不新建大系统，基于现有 `PathHandler::updateSpeedProfile()` 和 selective profile 继续扩展。

新增 condition：

```text
CONDITION=PROFILE_ENERGY
```

新增参数建议：

```yaml
path_handler:
  energy_profile_enable: false
  energy_profile_preview_distance: 1.0
  energy_profile_kappa_threshold: 0.65
  energy_profile_dkappa_threshold: 8.0
  energy_profile_ay_threshold: 1.6
  energy_profile_omega_threshold: 0.9
  energy_profile_lat_accel: 1.2
  energy_profile_omega_max: 1.1
  energy_profile_alpha_max: 3.0
  energy_profile_ax_max: 1.2
  energy_profile_decel_max: 1.2
  energy_profile_min_v: 0.35
```

默认全部关闭，不改变 NOM / FAS / OUTPUT_GUARD / PMG 行为。

## 5. 风险指标

对每个路径采样点 `i`，在未来 `preview_distance` 窗口内计算：

```text
kappa_max  = max |kappa|
dkappa_max = max |dkappa/ds|
ay_pred    = v^2 * kappa_max
omega_pred = v * kappa_max
alpha_pred ≈ v^2 * dkappa_max
```

触发条件：

```text
high_kappa  = kappa_max  > kappa_threshold
high_dkappa = dkappa_max > dkappa_threshold
high_ay     = ay_pred    > ay_threshold
high_omega  = omega_pred > omega_threshold
```

触发后使用更保守的局部速度上限：

```text
v <= sqrt(lat_accel / kappa_max)
v <= omega_max / kappa_max
v <= sqrt(alpha_max / dkappa_max)
```

然后在 forward/backward pass 中使用更保守的：

```text
accel <= energy_profile_ax_max
decel <= energy_profile_decel_max
```

关键区别：

- `PROFILE_SELECTIVE` 主要限制几何速度；
- `PROFILE_ENERGY` 还必须降低速度变化率，目标是抑制 `eta_dot` 和 longitudinal `eta_x` 激励。

## 6. 为什么不用 output cap / PMG

输出层方法的问题：

```text
output cap 在 MPC 求解之后修改 cmd_vel；
MPC 下一周期会为了追踪路径补偿；
容易形成新的 ax / omega 脉冲。
```

PMG 的问题：

```text
PMG 只约束 eta 峰值，不约束 eta_dot；
P2 PMG_LONG / combined 已经证明会把能量推到 eta_dot。
```

速度剖面层的优势：

```text
MPC 从一开始就跟踪低激励 v_ref；
速度变化连续，可通过 accel/decel pass 控制；
比 publish 层硬裁剪更不容易破坏 tracking。
```

## 7. 实验矩阵

先 P2，再 P3。不要同时扫很多参数。

### Step A：P2_s_curve

录 3 包：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=PROFILE_ENERGY RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

同参数录 `RUN_ID=02/03`。

对比基线：

```text
P2 NOM run16/run17/run18
```

P2 验收：

```text
tracking_time <= +15%
h_rms / h_p95 / energy / eta_dot 全下降
ay_p95 不升
solve_success_ratio >= 0.97
```

如果 P2 不通过，停止 `PROFILE_ENERGY` 当前实现，不录 P3。

### Step B：P3_mixed

只有 P2 通过后再录 3 包：

```bash
PATH_MODE=template_goal PATH_ID=P3_mixed CONDITION=PROFILE_ENERGY RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

同参数录 `RUN_ID=02/03`。

对比基线：

```text
P3 NOM reached run
```

P3 验收：

```text
tracking_time <= +15%
h_rms / h_p95 / energy / eta_dot 全下降
ay_p95 不升
track_dist_p95 不恶化超过 15%
solve_success_ratio >= 0.97
```

## 8. 停止条件

以下任一发生即停止当前路线，不继续扫参：

```text
P2 eta_dot 上升
P2 tracking_time > +15%
P2 h_p95 不降
P3 不能稳定 reached
P3 h_p95 / energy / eta_dot 任一不降
需要超过 2 轮参数扫才接近通过
```

如果 `PROFILE_ENERGY` 仍失败，说明当前路径/速度参考层方法也不足以解决跨路径晃动问题，应转向：

```text
真实液面视觉测量
omega_n / zeta 自由衰减辨识
P3 路径几何可达性审查
```

## 9. 成功率估计

基于历史数据：

```text
P2 通过概率：70%
P3 通过概率：35~45%
P2+P3 都通过：30~40%
```

原因：

- P2 已有 `PROFILE_SAFE` 正结果；
- P3 是主要瓶颈；
- 新方案补了 `ax / alpha / eta_dot` 风险意识，但仍受路径几何和 MPC tracking 能力限制。

## 10. 论文影响

如果 `PROFILE_ENERGY` 在 P2/P3 都通过，论文主线可转为：

```text
Energy-aware speed profiling for slosh-aware MPC tracking
```

如果只 P2 通过，仍只能写：

```text
模型、消融验证与失效机理分析；
部分路径上参考速度剖面能降低模型估计晃动，但跨路径泛化仍未解决。
```

