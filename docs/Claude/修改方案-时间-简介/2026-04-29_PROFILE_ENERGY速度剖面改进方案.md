# 2026-04-29 PROFILE_ENERGY 速度剖面改进方案

> 2026-04-30 v2 更新：吸收 `IEEE_FWMSV__ATiGB_2024` 的 flatness-based rest-to-rest 轨迹思想，将方案从“几何阈值限速”升级为“几何风险限速 + slosh ODE rollout 校正”。不照搬论文的 HSMC 力矩控制器。
>
> 2026-04-30 v3 更新：修正 `alpha = ax*kappa + v^2*dkappa`，补充 rollout 峰值时序验证、初始 slosh 状态口径、执行一致性检查、matched-time slow baseline、曲率抗噪、归一化能量阈值和终端残余指标。
>
> 2026-04-30 v4 更新：补齐实现前必须锁死的工程口径：candidate-profile 自归一化、rollout correction 接受/回滚、signed 序列 rollout、精确离散化/RK4、时间域 jerk 换算、在线 hysteresis/smoothing、`min_v` 小消融、debug topic 和 P3 纵向专属指标。

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

2026-04-30 进一步判断：

```text
几何阈值限速还不够。
真正应借鉴的是“先生成 anti-slosh reference，再由控制器跟踪”的结构。
```

相关论文 `IEEE_FWMSV__ATiGB_2024` 使用：

```text
flatness-based rest-to-rest anti-slosh trajectory
        +
hierarchical sliding mode tracking control
```

它的控制律不能直接搬到 Scout Mini，因为论文输出四轮麦克纳姆力矩 `tau`，而当前系统接口是 `/cmd_vel=[v, omega]`。但轨迹生成思想可以吸收：

```text
先让参考轨迹本身满足低晃动/终端静止倾向，
再让 tracking controller 跟踪参考。
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

不新建大系统，基于现有 `PathHandler::updateSpeedProfile()` 继续扩展。v2 分两层：

```text
Layer 1: geometry risk speed cap
  用 kappa / dkappa / ay / omega / alpha 生成初始低激励 v(s)

Layer 2: slosh rollout correction
  沿候选 v(s) 预测 ax / ay，并积分 slosh ODE
  根据 eta_x / eta_y / eta_dot / modal_energy 超阈值窗口局部修正 v(s)
```

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
  energy_profile_rollout_enable: true
  energy_profile_rollout_iters: 2
  energy_profile_eta_threshold: 0.0025
  energy_profile_eta_dot_threshold: 0.015
  energy_profile_energy_ratio_threshold: 0.8
  energy_profile_eta_dot_ratio_threshold: 0.8
  energy_profile_norm_source: candidate   # candidate | nominal_baseline | physical
  energy_profile_window_margin: 0.30
  energy_profile_window_scale: 0.85
  energy_profile_jerk_max: 3.0
  energy_profile_smooth_iters: 2
  energy_profile_smoothing_kernel: 5
  energy_profile_min_risk_window_length: 0.25
  energy_profile_terminal_window: 1.0
  energy_profile_dt_max: 0.05
  energy_profile_accept_require_h_drop: true
  energy_profile_accept_eta_dot_tolerance: 0.0
  energy_profile_enter_ratio: 1.0
  energy_profile_exit_ratio: 0.8
  energy_profile_temporal_beta: 0.6
```

默认全部关闭，不改变 NOM / FAS 行为。`OUTPUT_GUARD / PMG / PROFILE_*` 控制器入口已从主线撤回，仅作为历史实验结论保留在文档和离线分析中。

## 5. Layer 1：几何风险指标

对每个路径采样点 `i`，先对路径几何做抗噪处理：

```text
kappa_smooth = moving average 或 Savitzky-Golay smoothing
dkappa       = central difference on kappa_smooth
risk window 只有连续长度 >= min_risk_window_length 才有效
ignore isolated one-point spikes
```

原因：`dkappa/ds` 对离散路径噪声极敏感。如果不先抗噪，profile 会被单点曲率尖峰误触发。

然后在未来 `preview_distance` 窗口内计算：

```text
kappa_max  = max |kappa|
dkappa_max = max |dkappa/ds|
ay_pred    = v^2 * kappa_max
omega_pred = v * kappa_max
alpha_pred = ax * kappa + v^2 * dkappa
```

这里 `alpha_pred` 必须包含 `ax*kappa`。因为：

```text
omega = v * kappa
alpha = d omega / dt = ax * kappa + v^2 * dkappa/ds
```

P3 已经确认纵向 `eta_x` 主导，不能低估 `ax` 与曲率的耦合。

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

## 6. Layer 2：slosh ODE rollout 校正

几何风险只能间接约束激励，不能直接回答：

```text
当前 v(s) 是否真的会降低 eta_x / eta_y / eta_dot / modal_energy？
```

因此 v2 增加 slosh rollout。流程：

```text
输入:
  path samples: s_i, kappa_i, dkappa_i
  candidate speed profile: v_i
  slosh params: omega_n, zeta

计算:
  dt_i    = ds / max(v_i, v_min)
  ax_i    = (v_i^2 - v_{i-1}^2) / (2 ds)
  ay_i    = v_i^2 * kappa_i
  omega_i = v_i * kappa_i
  alpha_i = ax_i * kappa_i + v_i^2 * dkappa_i

rollout:
  eta_x_ddot + 2 zeta omega_n eta_x_dot + omega_n^2 eta_x = -ax_i
  eta_y_ddot + 2 zeta omega_n eta_y_dot + omega_n^2 eta_y = -ay_i

输出:
  eta_x_i, eta_x_dot_i
  eta_y_i, eta_y_dot_i
  modal_energy_i = omega_n^2(eta_x^2+eta_y^2) + eta_x_dot^2 + eta_y_dot^2
  E_norm_i = modal_energy_i / E_ref
```

rollout 初始状态必须明确分两种：

```text
offline/global profile:
  x_slosh(0) = 0

online/local update:
  x_slosh(0) = current /slosh/state
```

如果在线局部规划反复更新速度剖面，不能默认液体每次从静止开始，否则会低估已有残余晃动。

Layer 2 rollout 必须使用逐点有符号序列，不能把窗口最大绝对值直接喂给 ODE：

```text
signed input sequence:
  ax_i
  ay_i = v_i^2 * kappa_i
  omega_i = v_i * kappa_i
  alpha_i = ax_i * kappa_i + v_i^2 * dkappa_i
```

原因：S 弯和 P3 mixed 的曲率符号切换会影响液体相位。Layer 1 可以用窗口最大绝对值做保守限速，Layer 2 必须保留符号，否则容易削错相位。

### 6.1 归一化口径

`E_ref` 分离离线论文评估和在线运行两种口径：

```text
离线论文评估:
  E_ref = same-path NOM baseline 主段 modal_energy p95
  作用: 公平对比和图表归一化

在线/真实运行:
  E_ref = 当前未修正 candidate profile 的 rollout p95
  备选: 固定安全阈值或 eta_bar 映射出的物理能量阈值
```

推荐实现默认：

```text
E_norm = E_rollout / E_candidate_p95
eta_dot_norm = |eta_dot| / eta_dot_candidate_p95
```

这样 `Ours` 不依赖提前跑同路径 NOM bag，适合新路径在线运行。

如果某个窗口超阈值：

```text
|eta_x| or |eta_y| > eta_threshold
|eta_dot| > eta_dot_threshold
E_norm > energy_ratio_threshold
eta_dot_norm > eta_dot_ratio_threshold
```

使用归一化阈值比固定 `0.06` 更稳，后续换液位、容器或 `omega_n` 时不用重新解释绝对能量数值。

则对该窗口前后 `window_margin` 范围局部修正。不要对整段硬乘常数，应使用平滑 taper：

```text
v_cap(s) <- min(v_cap(s), v_old(s) * (1 - r * taper(s)))
ax_max   <- min(ax_max, energy_profile_ax_max)
decel_max<- min(decel_max, energy_profile_decel_max)
```

其中 `taper(s)` 建议使用 cosine taper：窗口边缘为 0，风险中心为 1。这样避免 `window_scale` 造成新的速度折角和 jerk 脉冲。

然后重新执行 forward/backward pass。最多迭代 `rollout_iters=2` 次，避免为了 proxy 指标过度优化速度剖面。

### 6.2 接受/回滚机制

每次 rollout correction 后必须重新 rollout，并比较本轮修正前后的预测指标。若出现以下任一情况，回滚本轮修正：

```text
h_p95_pred 没下降
eta_dot_pred 上升超过 tolerance
terminal_eta / terminal_eta_dot 上升
tracking_time_pred 超过 +15%
```

原因：液体是振荡系统，减速不一定单调降低峰值。某些降速会改变相位，反而把峰值移到更坏的位置。

因此实现上不能隐含：

```text
超阈值 → 一定降速 → 一定更安全
```

必须是：

```text
propose correction → rollout evaluate → accept or rollback
```

### 6.3 数值积分口径

不要使用最简单的显式 Euler 作为最终实现。优先级：

```text
1. 线性二阶系统精确离散化:
   x_{k+1} = A_d(dt_k) x_k + B_d(dt_k) u_k

2. 若实现成本过高，至少使用 RK4。
```

其中：

```text
x = [eta, eta_dot]^T
u = ax 或 ay
```

并限制：

```text
dt_i <= energy_profile_dt_max
```

如果 `dt_i = ds / max(v_i, v_min)` 超过 `dt_max`，rollout 内部对子段再插值。低速末端 `dt_i` 变大时，Euler 误差会直接污染 `eta_dot` 和 terminal residual 判断。

rollout 后还要做 jerk/smoothing pass：

```text
ax_i = (v_i^2 - v_{i-1}^2) / (2 ds)
jerk_i ≈ v_i * (ax_i - ax_{i-1}) / ds
|jerk_i| <= energy_profile_jerk_max
smooth_iters = 1~3
kernel_size = 5 or 7
```

注意：速度剖面是在路径坐标 `s` 上生成，jerk 是时间域量。不能直接对 `ax(s)` 普通差分后当成 jerk。目的不是追求速度曲线好看，而是避免局部降速窗口边缘反而制造新的 `ax` 脉冲。

### 6.4 在线重规划防抖

如果 `PathHandler::updateSpeedProfile()` 在线频繁重算，必须防止 profile 抖动：

```text
risk hysteresis:
  enter threshold = energy_profile_enter_ratio
  exit threshold  = energy_profile_exit_ratio

temporal smoothing:
  v_ref_new <- beta * v_ref_new + (1 - beta) * v_ref_prev
```

或更保守：

```text
只有当风险窗口位置/强度变化超过阈值时才更新 profile。
```

否则可能出现：

```text
本周期降速 → 下一周期放开 → 再下一周期降速
```

这种 profile 抖动本身会制造新的 `ax / jerk` 激励。

### 6.5 终端 residual 处理

吸收 rest-to-rest 思想时，不能只管路径中段峰值，还应检查末端残余：

```text
terminal_energy_pred
terminal_eta_norm
terminal_eta_dot_norm
```

在路径末端：

```text
s in [s_end - terminal_window, s_end]
```

额外要求：

```text
v_ref_end 平滑下降
ax_end 不突变
predicted terminal eta / eta_dot 不升
```

这不是完整 settling MPC，但能把 rest-to-rest 的“终点液体也要收敛”思想落到速度剖面层。

### 为什么不是完整 flatness

完整 flatness rest-to-rest 轨迹会直接设计 `x(t), y(t)`，并满足起终点：

```text
eta(tf)=0
eta_dot(tf)=0
```

当前 Scout 系统已有全局路径、局部 path handler 和 `/cmd_vel` 接口，直接替换为 flatness trajectory 会改动过大。因此 v2 只吸收其核心思想：

```text
让参考本身对 slosh 模型负责。
```

实现上先对既有路径的速度剖面做 model-rollout correction，而不是重写全局轨迹生成器。

## 7. 为什么不用 output cap / PMG / HSMC

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

HSMC 不直接采用的原因：

```text
论文 HSMC 输出四轮麦克纳姆力矩 tau；
Scout Mini 当前 local planner 输出 /cmd_vel；
没有直接力矩控制接口，也没有完整轮端动力学闭环。
```

可借鉴的是：

```text
anti-slosh reference first, tracking control second
```

不可直接照搬的是：

```text
torque-level HSMC control law
```

## 8. 实施顺序

### Step 0：离线验证 rollout 口径

在写控制器前，先扩展离线脚本，用历史 NOM bag 做 replay：

```text
输入: path + recorded odom/v_des_eff
输出: reconstructed eta_x/eta_y/eta_dot/modal_energy
对比: bag 内 /slosh/state 和 /slosh/height
```

通过标准：

```text
h_p95 reconstruction error <= 15%
eta_dot trend 不反向
peak_time_error <= 0.3~0.5 s
corr(eta_dot_replay, eta_dot_bag) > 0.6
high-risk window recall >= 0.7
```

这里 `high-risk window recall` 指 rollout 标记的风险窗口是否覆盖 bag 中实际 `height / eta_dot / modal_energy` 峰值前窗口。rollout 的目标不是逐帧重建，而是削对位置。

如果 rollout 自身不能复现历史 bag 的主峰，不能进入在线 `PathHandler` 实现。

### Step 1：Layer 1 几何 profile 作为 ablation baseline

先实现几何风险限速 + conservative accel/decel pass。它不是最终 proposed，而是 ablation baseline：

```text
B2: geometry-only profile
```

通过标准：

```text
P2 h_rms / h_p95 / energy / eta_dot 全下降
ay_p95 不升
tracking_time <= +15%
```

无论 Step 1 是否通过，最终方法仍应进入 Step 2。Layer 1 的作用是证明“仅几何风险限速是否足够”。

### Step 2：Layer 1 + Layer 2 作为最终 proposed

启用 rollout correction 后，才是最终方法：

```text
Ours: geometry profile + slosh rollout correction
```

先 P2，再 P3。

通过标准不降低：

```text
不能用大幅减速换取 h_p95 下降；
eta_dot 必须下降；
terminal residual 不能明显变差。
```

### Step 3：执行一致性检查

每个闭环 bag 必须检查 profile 是否真的被执行：

```text
v_ref_vs_odom_rmse
ax_ref_vs_odom_ax_corr
ay_ref_vs_odom_ay_corr
profile_violation_ratio
odom_v_exceeds_profile_ratio
```

如果 rollout 预测通过但闭环 `/slosh/height` 失败，先看 execution fidelity。若 MPC 或底盘没有执行低激励 profile，不能继续调 profile 阈值，应转向：

```text
MPC 速度跟踪权重
加速度/角加速度约束
cmd_vel 滤波
terminal recovery 是否破坏 profile
```

### Step 4：调试话题

实现时直接发布 debug topic，否则 P3 失败后无法定位失败层级：

```text
/profile_energy/v_nom
/profile_energy/v_geo
/profile_energy/v_rollout
/profile_energy/v_final
/profile_energy/ax_ref
/profile_energy/ay_ref
/profile_energy/omega_ref
/profile_energy/alpha_ref
/profile_energy/jerk_ref
/profile_energy/eta_x_pred
/profile_energy/eta_y_pred
/profile_energy/eta_dot_pred
/profile_energy/modal_energy_pred
/profile_energy/risk_flag
/profile_energy/risk_reason
/profile_energy/window_id
```

这些 topic 用来区分：

```text
rollout 没预测到
profile 预测到了但没降速
降速了但 MPC 没跟上
MPC 跟上了但 slosh 模型与实际 /slosh/height 不一致
```

## 9. 实验矩阵

先 P2，再 P3。不要同时扫很多参数。

### Step A：P2_s_curve

录 3 包：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=PROFILE_ENERGY RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

同参数录 `RUN_ID=02/03`。

对比基线至少包含：

```text
B0 NOM tracking MPC
B1 Uniform slow-down, matched tracking time
B2 Geometry-only profile
B3 MPC slosh-cost baseline
B4 historical output guard / PMG result if available
Ours PROFILE_ENERGY / EASP = Layer1 + Layer2 rollout correction
```

其中 `B1 matched-time slow baseline` 是硬要求。否则无法证明 `Ours` 不是简单慢下来。

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

对比基线同 P2，且必须只使用 reached 的 P3 NOM bag：

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

P3 额外报告纵向专属指标：

```text
eta_x_energy_ratio
eta_x_rms
eta_dot_x_rms
odom_ax_abs_p95
cmd_a_abs_p95
v_ref_jerk_p95
```

原因：P3 历史诊断显示 `eta_x` energy ratio ≈ 0.975。只报告总 `h_p95` 和 `ay_p95` 会掩盖纵向通道失败。

新增 rollout 内部验收：

```text
predicted eta_dot_peak 不升
predicted terminal eta / eta_dot 不升
rollout 标记的高风险窗口应与实际 height / eta_dot 峰前窗口重合
```

如果 rollout 预测通过但闭环失败，说明 MPC tracking 或底盘执行破坏了速度剖面，不能继续在 profile 层小修小补。

## 10. 停止条件

以下任一发生即停止当前路线，不继续扫参：

```text
离线 rollout 不能复现历史 bag 主峰
P2 eta_dot 上升
P2 tracking_time > +15%
P2 h_p95 不降
P3 不能稳定 reached
P3 h_p95 / energy / eta_dot 任一不降
rollout 预测通过但闭环 eta_dot 上升
rollout correction 被 accept/rollback 判据连续拒绝
execution fidelity 显示 MPC/底盘没有执行 profile
需要超过 2 轮参数扫才接近通过
```

如果 `PROFILE_ENERGY` 仍失败，说明当前路径/速度参考层方法也不足以解决跨路径晃动问题，应转向：

```text
真实液面视觉测量
omega_n / zeta 自由衰减辨识
P3 路径几何可达性审查
```

## 11. 成功率估计

基于历史数据，原 v1 几何阈值版估计：

```text
P2 通过概率：70%
P3 通过概率：35~45%
P2+P3 都通过：30~40%
```

v2 加入 slosh rollout 后，理论上 P3 成功率应略高，但实现风险也更高。更保守的工程估计：

```text
P2 通过概率：60~70%
P3 通过概率：35~50%
P2+P3 都通过：25~35%
```

v4 收紧工程口径后，预计“误判成功”的概率下降，但真实通过率不应上调：

```text
P2 通过概率：60~70%
P3 通过概率：35~50%
P2+P3 都通过：25~35%
```

如果 `energy_profile_min_v=0.35` 下风险窗口长期超阈值，允许一次小消融：

```text
min_v = 0.25 / 0.35
```

只做 P2、P3 各一次预检查，不进入大规模扫参。若必须靠 `min_v=0.25` 才通过，需要重点检查 tracking_time 是否超过 +15%。

原因：

- P2 已有 `PROFILE_SAFE` 正结果；
- P3 是主要瓶颈；
- v2 补了 `eta_x / eta_y / eta_dot / modal_energy` rollout，但仍受路径几何、MPC tracking 和底盘执行能力限制。

## 12. 论文影响

如果 `PROFILE_ENERGY` 在 P2/P3 都通过，论文主线可转为：

```text
Energy-aware speed profiling for slosh-aware MPC tracking.
```

更准确的贡献表述：

```text
受 flatness-based anti-slosh trajectory 启发，
本文在 /cmd_vel 接口和既有 MPC tracking 框架下，
提出一种 slosh-rollout-corrected speed profile generation 方法。
```

如果只 P2 通过，仍只能写：

```text
模型、消融验证与失效机理分析；
部分路径上参考速度剖面能降低模型估计晃动，但跨路径泛化仍未解决。
```

不能写：

```text
复现了 flatness + HSMC 方法。
MPC 代价函数主动抑制了液体晃动。
真实液面高度被证明降低。
```

除非补真实液面视觉验证，否则所有结果仍应写为：

```text
model-estimated slosh height / /slosh/height indicators
```
