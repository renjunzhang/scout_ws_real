# GeoRef 约束一致性 MPC 方案

日期：2026-05-06

## 1. 结论先说

最终 MPC 不应再设计成：

```text
slosh-cost-driven anti-slosh MPC
```

而应设计成：

```text
GeoRef-guided reference-constrained MPC
```

一句话定义：

```text
前端 GeoRef 负责生成低激励路径和执行预算；
MPC 负责在车辆约束下跟踪该参考，并通过时变约束保证执行不突破低激励预算；
slosh cost 只作为弱正则或消融项，不承担主要防晃职责。
```

## 2. 为什么需要这个方案

当前 Online GeoRef 已经证明：

```text
把防晃逻辑前移到几何参考生成层，比把 slosh cost 放进 MPC 或在 cmd_vel 后处理更有效。
```

但当前 GeoRef + normal MPC 仍有一个结构风险：

```text
GeoRef 生成了低激励路径；
MPC 只把 reference 当软目标；
为了 tracking，MPC 仍可能打出更大的 a / jerk / ay；
真实执行可能突破前端低激励假设。
```

所以新方案要解决的问题不是：

```text
能不能用 Q_slosh 让液体更小？
```

而是：

```text
能不能保证 MPC 执行不破坏 GeoRef 的低激励假设？
```

## 3. 最终结构

```text
raw/global path
    ↓
anti_slosh_path_post_processor
    ↓
低激励几何参考 + v/a/jerk/ay 预算
    ↓
reference-constrained MPC
    ↓
cmd_vel
    ↓
odom excitation
    ↓
slosh response
```

三层职责：

```text
Layer 1: GeoRef reference generation
  生成低曲率、低曲率变化、低执行激励风险的路径。

Layer 2: Reference-constrained MPC
  跟踪低激励参考，并把 v/a/jerk 等预算作为时变约束执行。

Layer 3: Slosh observer / evaluator
  monitoring-only in v1。
  /slosh/state、/slosh/height 用于离线指标、诊断、可选弱正则和未来终点 settling；
  第一版不进入 tracking 控制闭环。
```

## 4. 与旧方案的关系

保留：

```text
8 维 tracking MPC 框架
OSQP QP 求解器
Frenet tracking 状态
车辆 v/a/omega/da/domega 约束
slosh state 估计与 debug topic
extract_slosh_metrics.py 指标体系
```

降级：

```text
Q_slosh / Q_slosh_eta_dot
risk_scheduler
eta_box_constraint
speed governor
input shaping
```

这些不再作为主机制，只作为：

```text
弱正则
消融 baseline
failure analysis
安全监测
```

不再作为主线：

```text
OUTPUT_GUARD
PMG
PROFILE_ENERGY
PROFILE_REF_V2
```

## 5. 第一版最小实现范围（默认路径）

**默认情况**第一版只做：

```text
v/a/jerk vectorized upper bounds (Step 2a 硬上界 → Step 2b 仅必要时加 slack)
```

但**真正范围由 Step 0a 诊断结果决定**（见 §16 反馈环）：

```text
若 0a 无 channel 触发           → 不进入 Step 1，方案搁置
若 0a 主 gap 在 a / jerk         → 第一版按默认 v/a/jerk vectorized bounds
若 0a 主 gap 在 omega / alpha    → 第一版改为 ay 约束设计，跳过 v/a/jerk-only
若 0a 多通道触发                 → 按 §16 多通道分支处理
```

也就是说，"v/a/jerk-only" 不是绝对的最小实现，是默认假设；真正的最小实现由 0a 数据决定。

默认路径下暂时不做：

```text
ay_upper 线性化约束
alpha_upper 约束
eta_box_constraint
SLOSH_SETTLING
weak Q_slosh
```

理由：

```text
1. v/a/jerk 是最直接的执行一致性问题，工程改动最小。
2. ay = v * omega 需要线性化，调试面更大，仅当 0a 显示 omega/alpha 是主 gap 才提前进入。
3. SLOSH_SETTLING 是终点阶段问题，不应和路径中段 tracking 约束一起引入。
4. weak Q_slosh 的作用应在主机制通过后再看边际收益。
```

## 6. Step 0a：先做执行预算 gap 诊断

在写任何 OSQP 代码前，先用已采集的 GEOREF_TUNED bag 做零成本诊断。
初始三包只作为 quick check；若结果边缘，扩到五包后再决定是否进入代码实现。

### 6.1 reference 量的精确定义

诊断里的 `ref_*` 必须与 Step 1 将下发给 MPC 的 budget 同源，否则诊断结论不能预测 Step 1 的 binding 行为。

#### 6.1.1 ref_v 来源

```text
ref_v(s):
  对 bag 中 /scout/global_path_anti_slosh 选中 candidate 上做 forward velocity rollout，
  参数与 launch 中 prediction/{v_max, ay_max_budget, a_max, v_init} 一致。
  即 selection 阶段判 ay gate 用的是哪条 v(s)，诊断阶段就用同一条。
```

#### 6.1.2 时间域量的正确公式

`v` 在 path 坐标 `s` 上，但 `a / jerk / alpha` 是**时间域量**。直接对 `s` 做差分会少一个 `ds/dt = v` 的因子，量纲与物理意义都错。所有时间域量都要走 chain rule `d/dt = v · d/ds`：

```text
ref_a(s_i)     = (v(s_i)^2 - v(s_{i-1})^2) / (2 * ds)
                 # 推导: a = dv/dt = v · dv/ds = (1/2) · d(v^2)/ds

ref_jerk(s_i)  = v(s_i) * (a(s_i) - a(s_{i-1})) / ds
                 # 推导: da/dt = v · da/ds

ref_omega(s_i) = v(s_i) * κ(s_i)
                 # ω = v · κ

ref_alpha(s_i) = a(s_i) * κ(s_i) + v(s_i)^2 * (κ(s_i) - κ(s_{i-1})) / ds
                 # dω/dt = (dv/dt) · κ + v · (dκ/dt)
                 #       = a · κ + v · (dκ/ds) · v
                 #       = a · κ + v^2 · dκ/ds
```

各 channel `ref_*_p95` 取沿 path 长度的 p95。

不要使用 PathHandler 内部 v_ref 做诊断。它衡量的是 MPC 跟自己 v_ref 的差距，与 Step 1 budget 来源不同，会得到错误的 gap 估计。

不要把 `沿 s 一阶差分` 当成 `dv/dt`。这是上一版本的 bug，会让 ref_a / ref_jerk / ref_alpha 偏低（因为少乘 v），gap 假性偏大，Step 0a 误判。

### 6.2 输入

```text
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_GEOREF_TUNED_run01_190153.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_GEOREF_TUNED_run02_190940.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_GEOREF_TUNED_run03_191424.bag
```

### 6.3 测量量

四个 channel 一起测，不只 a/jerk：

```text
纵向通道:
  ref_a_p95, ref_jerk_p95
  cmd_a_p95, cmd_jerk_p95   (来自 /cmd_vel 数值差分)
  odom_a_p95, odom_jerk_p95 (来自 /odom 数值差分)

横向通道:
  ref_omega_p95, ref_alpha_p95
  cmd_omega_p95, cmd_alpha_p95
  odom_omega_p95, odom_alpha_p95
```

cmd/odom 对齐与滤波详见 §6.4。

对每个 channel `x ∈ {a, jerk, omega, alpha}`:

```text
abs_gap_x  = max(cmd_x_p95 - ref_x_p95, odom_x_p95 - ref_x_p95)
rel_gap_x  = abs_gap_x / vehicle_x_limit
gap_dist_x = 把 gap 沿 path s 分箱，记录集中区间是否能对应到具体路径段
```

`vehicle_x_limit` 使用车辆物理上限：

```text
vehicle_a_max, vehicle_j_max, vehicle_omega_max, vehicle_alpha_max
```

### 6.4 alignment 与滤波口径

ROS 时间抖动与数值差分容易污染 cmd_jerk / odom_jerk / alpha 的 p95，必须显式定义对齐与滤波，否则 Step 0a 不可复现。

时间窗：

```text
只取 TRACKING + terminal NONE 阶段的样本;
排除 GOAL_APPROACH / 末速度抖动段。
```

空间对齐：

```text
用 /odom pose 投影到 selected path，取最近 s_proj 作为该样本的路径进度;
cmd_vel 按时间戳线性插值到对应 s_proj 序列;
ref_*(s_proj) 由 §6.1 公式在该 s 上取值。
```

时序滤波：

```text
cmd_vel / odom 重采样到固定 dt = 0.05s (与 MPC 周期一致);
排除连续两点 dt < 0.5 * dt_nominal 或 > 1.5 * dt_nominal 的样本;
a / jerk / alpha 用 Savitzky-Golay (window=7, order=2) 或 5 点中值滤波;
omega 不需要滤波。
```

p95 计算：

```text
在过滤与对齐后的样本集上取 p95;
报告时同时给出 sample count 与每包过滤前后比例;
若过滤后 sample count < 100，标记该 bag 为 inconclusive。
```

### 6.5 判据

进入 Step 1 当且仅当存在某个 channel x 同时满足：

```text
(a) rel_gap_x >= 0.15
(b) gap_dist_x 能定位到具体 path 段，不是均匀分布在直线段噪声里
```

各种结果对应不同的 Step 1 范围：

```text
没有 channel 满足 (a)+(b):
  MPC 已经基本按 reference budget 执行,不写 Step 1。

仅 a / jerk 触发:
  Step 1 范围 = v_upper + a_upper + a_lower + jerk_upper。

仅 omega / alpha 触发:
  不做 v/a/jerk-only。
  单独设计 ay/omega 约束，或停止本方案。

多通道触发:
  按 §16 反馈环顺序处理。
```

不再使用 `(cmd - ref) / ref` 作为唯一阈值。原因：

```text
ref 在直线段会很小,相对 gap 容易被噪声放大成假阳性；
ref 在转弯段大,绝对差被压低,可能漏抓真问题。
绝对量与车辆物理量纲对齐,加上分布定位,才能区分真实问题与噪声。
```

### 6.6 为什么这一步不能省

```text
当前 open_user_goal 的 RAW/GEOREF active_s 已接近,v_upper 在多数 horizon 未必绑定；
真正需要确认的是曲率/加减速区 MPC 是否突破了 reference 的 a/jerk/omega/alpha budget。
如果没有明显突破,写约束代码只会增加复杂度,不会带来论文收益。
```

### 6.7 2026-05-07 五包 Step 0a 结果

输入：

```text
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_GEOREF_TUNED_run01_190153.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_GEOREF_TUNED_run02_190940.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_GEOREF_TUNED_run03_191424.bag
/data/a/slosh_bags/sim/20260507/20260507_open_user_goal_GEOREF_TUNED_run04_120200.bag
/data/a/slosh_bags/sim/20260507/20260507_open_user_goal_GEOREF_TUNED_run05_120405.bag
```

输出：

```text
csv:
  /data/a/slosh_bags/analysis/georef_budget_gap_step0a_5bags_20260507.csv

trigger_counts:
  a=0
  jerk=3
  omega=2
  alpha=0

overall:
  MULTI_CHANNEL_BRANCH
```

结论：

```text
Step 0a 已通过，normal MPC 存在一致突破 GeoRef budget 的证据。
主问题不是纯纵向 a，而是 jerk + omega 多通道。
触发位置集中在 path s≈6.75m，说明不是全路径均匀噪声。
```

对实施范围的影响：

```text
可以进入 Step 0b / Step 1。
第一波仍先做 v_upper + a_upper + a_lower + jerk_upper hard-bound smoke，
原因是这些约束能复用现有 box/rate 结构，工程风险最低。
但必须把 omega 作为 Step 3 的重点观察项：
若 v/a/jerk 约束后 omega_p95 仍不降或反升，
则不能宣称本方案完成，应进入 ay/omega 约束设计或停止扩展。
```

## 7. 第一版约束形式：已有 box/rate bound 向量化

GeoRef 给 MPC 提供随 horizon 变化的预算：

```text
v_upper,k
a_upper,k
a_lower,k
jerk_upper,k
```

第一版不新增新的约束类型，而是把已有车辆 box/rate bound 从标量改为时变向量：

```text
v_k <= min(vehicle_v_max, v_upper,k)

max(vehicle_a_min, a_lower,k) <= a_k <= min(vehicle_a_max, a_upper,k)

|a_k - a_{k-1}| <= min(vehicle_j_max, jerk_upper,k) * dt
```

`a_lower,k` 必须保留。原因：

```text
液体纵向模态输入是 signed ax；
急刹也是强激励，是否有益取决于液体相位，不是天然有益。
如果只限制正加速度、不限制减速度，会漏掉 terminal / braking 段的 longitudinal excitation。
```

slack 与硬上界分两阶段实现，避免一次性同时改 ub/lb 向量与 QP 结构：

#### Step 2a — vectorized hard bounds smoke

```text
不引入 slack;
只把已有 box / rate bound 从标量改为时变向量;
ConstraintManager / MPCSolver 矩阵结构基本不变,只改 lb/ub 向量生成。

约束形式:
  v_k <= min(vehicle_v_max, v_upper,k)
  max(vehicle_a_min, a_lower,k) <= a_k <= min(vehicle_a_max, a_upper,k)
  |a_k - a_{k-1}| <= min(vehicle_j_max, jerk_upper,k) * dt

目的:
  - 验证 budget 下发链路 (post-processor → topic → MPC) 工作;
  - 验证最小实现可解,统计 infeasible 频次;
  - 在 open_user_goal 上看 cmd_a / cmd_jerk 是否真的被压住。

风险:
  - 路径尖端可能 infeasible → fallback 到 vehicle_x_max,
    需要在 MPC 里加监测计数,记录 infeasible 频率;
  - 如果 infeasible 罕见,可以直接走 2a 论文跑;
  - 如果 infeasible 频次 > 5%,进入 Step 2b。
```

#### Step 2b — slack 版

```text
仅在 Step 2a infeasible 频次过高时引入。
增加 slack 决策变量与代价项,QP 结构修改 (列、约束、代价同时变更)。

约束形式:
  v_k <= min(vehicle_v_max, v_upper,k) + s_v,k
  max(vehicle_a_min, a_lower,k) - s_a,k <= a_k <= min(vehicle_a_max, a_upper,k) + s_a,k
  |a_k - a_{k-1}| <= min(vehicle_j_max, jerk_upper,k) * dt + s_j,k
  s_v,k, s_a,k, s_j,k >= 0

代价增加:
  W_v_slack * s_v,k^2 + W_a_slack * s_a,k^2 + W_j_slack * s_j,k^2

slack 权重 anchor:
  W_slack = N * Q_v_tracking, N ∈ [5, 10]
  第一版固定 N = 5,不扫参。
  语义: 违反 reference budget 的代价比少跟踪 v_ref 高 5 倍。
```

第一版默认走 `2a → 2b` 顺序，不直接上 2b。也就是说：写 ub 向量的代码量是 Step 2a 的全部内容；slack 决策变量与代价项只在 2a 不够用时才落地。

## 8. 第二版再考虑 ay 约束

如果 Step 1 通过，但仍出现：

```text
v/a/jerk 下降
但 ay_p95 或 omega 仍高
```

再加入：

```text
|v_k * omega_k| <= ay_upper,k + s_ay,k
```

QP 线性化口径：

```text
v_k * omega_k ≈ v_bar,k * omega_k
              + omega_bar,k * v_k
              - v_bar,k * omega_bar,k
```

这里使用现有 successive linearization 思路，保持 OSQP QP 框架，不改成 NMPC。

Step 2 成功标准：

```text
ay_p95 进一步下降；
tracking_time 仍在 +15% 内；
slack_ay 不长期激活；
solve_success_ratio >= 0.97。
```

如果 Step 1 后 ay_p95 已经同步下降，则不做 Step 2。不要为了“更完整”引入不必要的双线性化约束。

## 9. slosh cost 的最终定位

主方法：

```text
GeoRef + reference constraints
Q_slosh = 0
Q_slosh_eta_dot = 0
```

弱正则消融：

```text
GeoRef + reference constraints + weak slosh regularization
Q_slosh = 0.5 或 1.0
Q_slosh_eta_dot = 0.01 或 0.02
```

目的：

```text
验证在低激励参考与执行约束已经存在时，
slosh cost 是否还有稳定边际收益。
```

如果出现以下任一情况，弱正则不采用：

```text
tracking_time 超 +15%
eta_dot 上升
h_max 上升
slack 激活增加
solve_success 下降
跨目标不稳定
```

论文表述：

```text
slosh-state penalties are evaluated as weak regularization,
but the primary slosh reduction comes from low-excitation reference generation
and reference-consistent MPC execution.
```

不能写：

```text
MPC slosh cost is the core anti-slosh mechanism.
```

## 10. 终点 settling 的位置

如果要保留“主动压晃”的味道，最合理位置是终点阶段，而不是全路径 tracking 阶段。

可选未来状态机：

```text
TRACKING
  -> GOAL_APPROACH
  -> SLOSH_SETTLING
  -> REACHED
```

`SLOSH_SETTLING` 目标：

```text
保持在目标邻域内
v -> 0
omega -> 0
降低 eta_dot / modal_energy
等待液体残余衰减
```

但这不属于第一版。第一版只做路径中段执行一致性。

## 11. 实验设计

最小对比：

```text
B0: RAW path + normal MPC
B1: GeoRef + normal MPC
Ours: GeoRef + reference-constrained MPC
```

若 Ours 优于 B1，说明：

```text
光生成低激励参考不够；
MPC 必须约束一致地执行低激励预算。
```

旧方案消融：

```text
raw path + MPC slosh cost
GeoRef + weak slosh regularization
OUTPUT_GUARD / PMG / PROFILE_ENERGY / PROFILE_REF_V2 作为 failure analysis
```

主指标分四组，§12 成功标准按组对应：

```text
A. Excitation budget consistency (Step 1 直接目标):
   cmd_a_p95, cmd_jerk_p95, cmd_omega_p95, cmd_alpha_p95
   odom_a_p95, odom_jerk_p95, odom_omega_p95, odom_alpha_p95
   ref_a_violation_ratio   = 命令值超出 ref_a budget 的样本比例
   ref_jerk_violation_ratio
   ref_omega_violation_ratio (若启用 ay 约束才看)
   ref_alpha_violation_ratio (若启用 ay 约束才看)

B. Slosh outcome (Step 1 间接目标):
   h_rms, h_p95, h_max
   eta_dot_rms
   modal_energy_rms
   ay_p95

C. Tracking quality (Step 1 不能损失):
   tracking_time
   track_dist_p95

D. Solver health (Step 1 实现质量):
   solve_success_ratio
   slack activation ratio
   slack p95 / max
```

噪声地板分两档：

```text
工程推进门槛 (Step 3 是否进入扩约束阶段):
  Ours 对 B1 的 A、B 两组主指标改善超过 B1 三包样本 1 sigma。
  3 包样本估计 sigma 本身不稳,1 sigma 只能作为快速 go/no-go 信号。

论文入表门槛:
  3 包均值改善 + bootstrap CI 显著;
  必要时补到 5 包稳定 sigma 估计;
  p95 / peak 类指标必须给 CI,不只给均值。

若改善不超过工程门槛,Step 1 视为 inconclusive,不进入扩约束阶段;
论文入表门槛不达不应作为论文主结论支撑。
```

## 12. 第一版成功标准

相对 B1 `GeoRef + normal MPC`，Ours 需要：

```text
cmd_ax_p95 不升，最好下降
odom_ax_p95 不升，最好下降
cmd_jerk_p95 下降
ref/command/execution 一致性提高
h_p95 不劣于 B1，目标进一步下降
eta_dot_rms 不升
tracking_time <= B0 * 1.15
solve_success_ratio >= 0.97
slack 不长期激活
```

如果只看到：

```text
tracking_time 增加
h_p95 没改善
slack 大量激活
```

则说明约束太紧或预算口径错误，不应进入 Step 2。

## 13. 预期效果

Step 0a 之前**不预测 slosh outcome 改善方向**。这一节只声明方法学上的目标和已知风险，不给任何具体百分比。

方法学目标（不是承诺）：

```text
Step 1 直接目标:
  cmd / odom 更接近 reference budget (§11 A 组);
  ref budget violation ratio 下降;
  execution consistency 改善。

Step 1 间接目标 (可能伴随,需 Step 3 验证):
  h_rms / h_p95 / h_max 改善;
  eta_dot_rms 改善;
  modal_energy_rms 改善;
  ay_p95 改善 (主要由 v_upper 间接压制)。
```

已知风险（必须在 Step 3 显式监测，否则方法可能"换激励通道"而非"减激励"）：

```text
- 负向 ax 风险:
  第一版虽然包含 a_lower,k，但若 budget 生成过松或插值错位，急刹仍可能成为新的强纵向激励源。
  Step 3 必须把 cmd_ax_p95 拆 signed (cmd_ax_pos_p95 / cmd_ax_neg_p95) 分别报告。
  若 cmd_ax_neg_p95 显著上升，说明 a_lower budget 或 MPC 插值/执行链路有问题，不能进入扩约束。

- omega 反弹风险:
  v_upper 压住 v 后,MPC 可能用更急的 omega 完成 tracking。
  Step 3 必须对 cmd_omega_p95 / cmd_alpha_p95 与 B1 比较。
  若 omega 通道激励反而增,本方案在该数据集上失败,不应进 Step 2b 以外的扩约束。

- 终端 settling 漏控:
  第一版不引入 SLOSH_SETTLING,目标到达后液体残余衰减不受 reference budget 约束。
  Step 3 必须报告 t_arrive 之后 N 秒内的 h_rms / eta_dot,确认未被掩盖。
```

成立判据：

```text
不预设具体百分比改善。
Ours vs B1 的判据 = §11 A、B 两组主指标改善超过 B1 三包 1 sigma noise floor;
论文入表则按 §11 论文入表门槛 (bootstrap CI / 5 包扩样)。
关注重心:
  cmd 是否真正不突破 reference budget (§11 A 组);
  h_max / eta_dot 单包反弹是否减少 (§11 B 组);
  tracking_time / solver health 是否守住 (§11 C/D 组);
  signed ax 与 omega 通道是否被新方案变差 (上面"已知风险"三条)。
```

不应承诺：

```text
任意路径都防晃
maze 窄通道安全
/slosh/height 等于真实液面
Q_slosh 重新成为主贡献
slosh outcome 一定下降 (Step 0a 之前不预测方向)
任何具体百分比改善
```

## 14. 当前关键设计问题

后续一起拍板：

```text
1. v/a/jerk 预算由 post-processor 直接发布，作为 single source of truth。
2. 预算按 path 点级别发布，与 /scout/global_path_anti_slosh 对齐；MPC 端按路径 s 最近邻或线性插值到 horizon。
3. 先做 Step 2a vectorized hard bounds smoke，再决定是否做 Step 2b slack version。
   理由: hard bounds 可以最小化工程改动，先验证信号；
        slack version 会改 QP 结构，只有 2a 出现 infeasible 或约束过硬时才值得做。
4. jerk 约束复用现有 Δa bound，只把标量上界替换为时变上界。
   a_upper、a_lower、v_upper 同理。
5. 预算话题使用 sibling topic，与 path 同步发布;
   不把预算 pack 进 PoseStamped.position.z;
   不让 post-processor 关心 MPC 的 N、dt、horizon length。
```

建议第一版取舍：

```text
预算由 post-processor 在 selected candidate 上计算一次；
不要让 PathHandler 重新算，避免 selection budget 与 MPC execution budget 不一致；
预算与 path 点对齐发布，MPC 端插值成 horizon vector；
不让 post-processor 关心 MPC 的 N、dt、horizon length；
优先使用 trajectory_msgs/JointTrajectory；
  Header 与 /scout/global_path_anti_slosh 同步；
  每个 point.positions = [s, v_upper, a_upper, a_lower, jerk_upper]；
  time_from_start 可选，用于 future time parameterization。
不推荐 std_msgs/Float32MultiArray 做正式实现，因为没有 Header，同步语义弱。
不推荐把预算塞进 PoseStamped.pose.position.z，除非只做一次性 smoke。
slack 如工程量过大，先做 Step 2a hard-bound smoke；
正式版本只有在 2a 显示 infeasible 或约束过硬时进入 Step 2b。
```

## 14.1 Step 0b 代码结构结论

2026-05-07 阅读当前实现后，最小改动点如下。

当前链路：

```text
LocalPlannerROS::controlLoop
  -> PathHandler::getReferencePoints(...)
  -> std::vector<ReferencePoint> ref_points
  -> MPCSolver::solve(current_state, ref_points)
  -> MPCSolver::buildQP(...)
  -> ConstraintManager::buildQPConstraints(...)
  -> OSQP
```

关键事实：

```text
ReferencePoint 当前只有 x/y/theta/kappa/s/v_path/v_ref，没有 execution budget 字段。
v_ref 只进入 cost_function，是软目标，不是硬约束。
ConstraintManager 当前已有:
  - v state bound
  - a / omega control bound
  - Δa / Δomega rate bound
这些约束行已经存在。
MPCSolver::updateOSQP 每周期更新 P/A/q/l/u；
只要约束矩阵非零结构不变，OSQP 可直接 update_bounds。
```

因此 Step 2a 的最小实现应是：

```text
1. 在 ReferencePoint 增加可选 budget 字段:
   ref_v_upper
   ref_a_upper
   ref_a_lower
   ref_jerk_upper
   has_ref_budget

2. MPCSolver::buildQP(refs) 前将 horizon budget 传给 ConstraintManager。

3. ConstraintManager 在构建已有约束的 l/u 时按 k 取 min/max:
   v upper: min(vehicle_v_max, ref_v_upper[k])
   a upper: min(vehicle_a_max, ref_a_upper[k])
   a lower: max(-vehicle_a_max, ref_a_lower[k])
   Δa upper/lower: ±min(vehicle_j_max, ref_jerk_upper[k]) * dt

4. 不新增约束行，不新增 slack 变量，不改变 QP 决策变量维度。
```

这个实现保持 Step 2a 的定位：

```text
hard-bound smoke;
验证 budget 下发与 OSQP 可解性;
不解决 infeasible 的长期工程问题;
不处理 omega/ay 直接约束。
```

Step 1 初版实现：

```text
post-processor 发布 /scout/global_path_anti_slosh_budget；
消息类型 trajectory_msgs/JointTrajectory；
point.positions = [s, v_upper, a_upper, a_lower, jerk_upper]。
这里显式带 s，而不是只依赖点索引对齐。
原因：PathHandler 会 sanitize/resample/smooth 路径，索引对齐不稳定。
```

第一版不建议让 PathHandler 重新计算 post-processor 的 budget：

```text
否则 selection 用的 budget 与 MPC 执行约束 budget 可能不一致。
当前实现没有让 PathHandler 复算 budget；
PathHandler 只缓存 budget topic，并按 path s 插值到 horizon ReferencePoint。
```

Step 2a 初版实现：

```text
新增参数:
  mpc/reference_constraints_enable=false
  mpc/reference_budget_topic=/scout/global_path_anti_slosh_budget

默认关闭。
开启后，MPCSolver 只有在 horizon refs 中实际存在 has_ref_budget 时才启用 reference bounds；
若 budget topic 未到，不会仅因为开关打开就强制增加 Δa 约束。

约束改动:
  v upper: min(vehicle_v_max, ref_v_upper[k])
  a upper/lower: ref_a_upper/ref_a_lower 收紧既有 a box
  jerk upper: ref_jerk_upper 收紧既有 Δa rate bound

未做:
  slack
  ay/omega 线性化约束
  alpha 约束
  slosh cost 正则
```

仿真入口：

```text
run_sim_fixed_path_bag.sh 新增 CONDITION=GEOREF_CONSTRAINED。
该 condition 等价于 GEOREF_TUNED + reference_constraints_enable=true，
用于 Step 2a smoke 与后续 B1/Ours 对比。
```

## 15. 实物优先级

本方案不能阻塞实物 open 验证。

```text
实物 open RAW_REAL vs GEOREF_REAL 是论文成败主线；
reference-constrained MPC 是方法增强项。
```

执行原则：

```text
实物 open 录包与本方案并行；
若实物窗口可用，优先实物录包；
本方案只有 Step 0a 诊断通过后才进入 Step 1；
不要为了 MPC 改造推迟实物 open 验证。
```

判断：

```text
实物 open 通过：
  reference-first 论文叙事已经成立，本方案是加分增强。

实物 open 失败：
  本方案大概率也无法单独救回主结论，应先处理实物真液面/执行/定位差距。
```

## 16. 下一步

建议流程，0a 出数后再决定 Step 1 的具体范围：

```text
Step 0a: 用现有 GEOREF_TUNED bag 做四通道 (a/jerk/omega/alpha) gap 诊断。
         quick check 最少三包；若 EDGE_INCONCLUSIVE，扩到五包。
         按 §6.4 双判据 (rel_gap >= 0.15 且 gap_dist 可定位到具体路径段) 决定走向。
         不通过则停止，不写 OSQP 约束代码。

Step 0a → Step 1 设计反馈:

  没有 channel 触发:
    停止。Ours ≈ B1，无论文价值。

  仅 a / jerk 触发:
    Step 1 范围 = v_upper + a_upper + a_lower + jerk_upper。
    不做 ay 约束。

  仅 omega / alpha 触发:
    不做 v/a/jerk-only。
    单独设计 ay/omega 约束，或停止本方案。

  多通道触发 (a 与 omega 同时):
    第一波先下 v_upper + a_upper + a_lower + jerk_upper，看 ay_p95 是否同步下降；
    原因: ay = v² · κ，压住 v_upper 通常会间接压住 ay。
    若第一波未压住 ay，再加 ay 约束。
    避免一次性引入双线性化约束放大调试面。

Step 0b: 读 MPCSolver / ConstraintManager 当前约束矩阵结构，
         确认 §7 "已有 box 上界向量化" 的最小改动点。

Step 1: 设计 path 点级别 budget sibling topic 和 MPC 端插值数据结构。

Step 2:
  Step 2a: 按 Step 0a 决定的范围实现 vectorized hard bounds smoke，默认关闭。
  Step 2b: 若 2a 有信号且无系统性 infeasible，再实现 slack version。

Step 3: open_user_goal 跑 B0/B1/Ours x3，按 §11 分组指标 + §12 noise floor 判定。

Step 4: 只有 Step 3 通过且仍存在未约束 channel 的反弹，才扩约束范围。

Step 5: 只有 Step 3/4 稳定通过，才讨论 weak slosh regularization。
```

## 17. 2026-05-07 执行结论：实现撤回，负结果保留

本方案已完成最小实现与 open_user_goal x3 smoke 验证，但结果不支持继续作为主线推进。

关键结果：

```text
B1 = Online GeoRef + normal MPC tracking
Ours = Online GeoRef + v/a/jerk reference-constrained MPC

Ours tracking_time: +8.41%
Ours h_rms:         +13.73%
Ours h_p95:         +6.29%
Ours eta_dot_rms:   +28.85%
Ours energy_rms:    +14.90%
Ours odom_ay_p95:   -9.09%
```

解释：

```text
v/a/jerk hard-bound 确实改善了部分执行一致性与 ay 指标；
但 /slosh/height、eta_dot、modal energy 相对 B1 变差。
这说明当前 hard-bound 约束把激励从横向 ay/速度执行侧转移到了模态速度/能量侧，
不能作为晃动抑制主方法。
```

工程处理：

```text
已外科撤回 reference-constrained MPC 实现和运行入口；
保留 diagnose_georef_budget_gap.py 作为后续诊断工具；
保留本文档与日志作为 failure analysis。
```

撤回范围：

```text
撤回 /scout/global_path_anti_slosh_budget topic
撤回 trajectory_msgs 依赖
撤回 LocalPlannerROS budget subscriber
撤回 PathHandler reference budget cache
撤回 MPC ReferencePoint ref_* fields
撤回 ConstraintManager v/a/jerk vectorized hard bounds
撤回 run_sim_fixed_path_bag.sh 中的 GEOREF_CONSTRAINED condition
```

保留主线：

```text
Online GeoRef path post-processor
+ normal MPC tracking
+ open 场景 RAW / SLOW / ORIGINAL / GEOREF 对比
+ 后续实物 open 验证
```

论文使用方式：

```text
本方案不作为 proposed method。
可作为 failure analysis：
  "making the MPC follow reference budgets more tightly does not necessarily reduce slosh;
   constraining v/a/jerk can reduce lateral acceleration but increase modal velocity and energy."
```
