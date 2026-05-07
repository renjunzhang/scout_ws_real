# Slosh 模型引导 GeoRef 候选评分方案

日期：2026-05-07

## 1. 方案定位

当前主线是：

```text
MBF raw path
  -> Online GeoRef candidate generator
  -> /scout/global_path_anti_slosh
  -> 普通 MPC tracking
  -> /cmd_vel
```

现有 Online GeoRef 已经在 open_user_goal 上得到仿真正结果，但当前候选选择主要依赖几何代理指标：

```text
kappa
dkappa
predicted ay ratio
length_ratio
max_drift
collision gate
```

本方案的目标是把已有 slosh model 合理结合进 GeoRef：

```text
Geometry-only GeoRef
  -> Slosh-model-guided GeoRef
```

核心原则：

```text
slosh model 参与候选路径评分和诊断；
不重新放回 MPC cost 当主控制权威；
不做 slosh hard gate；
不复活 GEOREF_CONSTRAINED。
```

最终结构：

```text
MBF raw path
  ↓
Online GeoRef candidate generator
  ↓
slosh-model-guided candidate scoring
  ↓
/scout/global_path_anti_slosh
  ↓
普通 MPC tracking，Q_slosh=0
  ↓
/cmd_vel -> odom
  ↓
slosh observer / evaluator
```

## 2. 为什么这样结合

已否定的路线说明：

```text
Q_slosh / Q_eta_dot / terminal slosh cost
  -> 不能稳定跨路径压低 /slosh/height

OUTPUT_GUARD / PMG
  -> output-layer cap 容易触发闭环补偿或 eta_dot 上升

GEOREF_CONSTRAINED
  -> v/a/jerk hard-bound 降低部分 ay，但 h/eta_dot/energy 变差
```

因此，slosh model 不应再作为 MPC 主代价或硬约束，而应前移到 reference generation 层，回答：

```text
在多条候选参考路径中，哪一条预计液体晃动风险最低？
```

这样可以保留 slosh model 的价值：

```text
1. 用于候选路径风险排序；
2. 用于闭环 /slosh/height、eta_dot、modal_energy 诊断；
3. 用于与实物视觉液面做趋势一致性验证。
```

## 2.1 与 Q_slosh-as-cost 失败的物理区分

本方案使用的线性模态 slosh 模型与之前 MPC Q_slosh / Q_eta_dot 项使用的是同一套模型。
该模型作为 MPC cost 项时不能稳定压低 /slosh/height，必须解释为什么作为 candidate score
就能成立，否则 Step 0 即使通过也只是 in-sample 巧合。

区分的核心在于使用方式不同导致对模型保真度的要求不同：

```text
作为 MPC cost / 硬约束（已否定）:
  模型直接进入闭环优化目标。
  优化器会在 v、a、omega、Δa、Δomega 等多个维度同时 trade-off，
  把模型预测的 slosh 风险换成其它代价（例如 eta_dot 上升、能量转移、tracking 偏差）。
  这要求模型预测在所有维度上同时与真实闭环一致，才能避免被 trade-off 走偏。
  实测下这一保真度要求不成立。

作为 candidate score（本方案）:
  模型只在已通过 hard gate（collision / max_drift / length_ratio / endpoint）的候选间排序。
  优化器（MPC）始终在执行普通 tracking，不会按 slosh 模型重新分配代价。
  对模型只要求一个弱得多的性质：
    在 hard-gate 已筛过的候选集合内，模型预测的相对排序与闭环主指标方向一致。
  绝对值偏差不直接进入闭环。
```

因此本方案对 slosh 模型的可用门槛是：

```text
不要求模型能精确预测 /slosh/height 数值；
只要求模型能在候选集合上给出与闭环一致方向的排序。
```

Step 0 必须显式验证的就是这一弱性质（§6 Q2），而不是数值精度。

## 3. 第一版不改 MPC

第一版必须保持：

```text
Q_slosh=0
Q_slosh_eta_dot=0
enable_slosh_box_constraint=false
risk_scheduler_enable=false
energy_profile_enable=false
input_shaping_enable=false
slosh_speed_governor_enable=false
```

MPC 角色不变：

```text
输入: /scout/global_path_anti_slosh
输出: /cmd_vel
控制变量: a, omega
约束: v, a, omega, da, domega
求解器: OSQP
```

本方案只修改候选路径评分，不修改 MPC 优化问题。

## 4. 候选 rollout 模型

对每条候选路径 `C_i`，先生成 signed excitation 序列。

路径采样量：

```text
s_i
ds_i
kappa_i
dkappa_i
```

速度口径第一版采用 per-candidate rollout，不在 candidate 间共享同一条 v(s)，
也不引入在线速度优化。

口径选择说明：

```text
方案 A 共享 v(s):
  所有 candidate 用同一条 v(s)。score 差异只来自 kappa/dkappa 形状。
  优点：纯几何对比清晰。
  缺点：与闭环不符——MPC 实际会按 candidate 自身的曲率/约束跑出不同 v(s)，
        共享 v 会高估 strong 候选的 ay 风险，低估 mild 候选的可加速空间。

方案 B per-candidate v(s)（本方案选用）:
  每条 candidate 用相同的 prediction 参数集独立 rollout 自己的 v(s)。
  优点：与 post-processor 当前 predicted ay ratio 指标口径一致；
        直接反映 MPC 闭环将以何种速度执行该 candidate；
        smooth 候选自然获得更高 v 也更高 ax，避免"形状好但实际开不快"被高估。
  缺点：长且过度平滑的候选可能凭借低 v 拿到更低 slosh 预测，
        必须靠已有 length_ratio_penalty 与 collision/drift gate 抑制，
        且 §5 score 中 w_l 权重不能为 0。
```

prediction 参数必须与 `anti_slosh_path_post_processor.launch` 中现有
`prediction/{v_max, ay_max_budget, a_max, v_init}` 完全一致，禁止为 score 单独引入新口径。
这样保证 Step 1 在线接入时，rollout score 与现有 ay_ratio 指标基于同一速度模型，
不会因为口径分裂带来不可解释的排序变化。

rollout 输出（按 candidate 各自的 v(s)）：

```text
v_i
ax_i    ≈ (v_i² - v_{i-1}²) / (2 ds)
ay_i    = v_i² * kappa_i
omega_i = v_i * kappa_i
alpha_i = ax_i * kappa_i + v_i² * dkappa_i
```

其中 dkappa_i 表示 dκ/ds（路径弧长域导数）。

slosh rollout 使用线性二阶模态模型：

```text
eta_x_ddot + 2 ζ ω_n eta_x_dot + ω_n² eta_x = -ax_i
eta_y_ddot + 2 ζ ω_n eta_y_dot + ω_n² eta_y = -ay_i
```

modal energy 口径：

```text
E = ω_n²(eta_x² + eta_y²) + eta_dot_x² + eta_dot_y²
```

## 4.1 rollout 数值口径

模型是时域线性 ODE，path 是弧长域采样，必须显式定义二者衔接，否则 ds→0 或 v→0 处会发散。

积分域：

```text
方案 A ds 域积分:
  把 ODE 用 d/dt = v · d/ds 换到 ds 域。
  在 v 接近 0 时（候选起点 v_init 极小或终点减速段）放大数值噪声，不可用。

方案 B dt 域积分（本方案选用）:
  对每条 candidate 的 (s_i, v_i, ax_i, ay_i) 序列，按 dt_i = ds_i / max(v_i, v_floor)
  累计时间戳 t_i，然后在固定 dt = 0.05s 上重采样 ax(t), ay(t)，
  最后用半隐式或 RK2 在时域积分模态 ODE。
  v_floor 取 prediction/v_init 一致值，下限不低于 0.05 m/s。
```

rollout 时间窗口：

```text
rollout 起点 t=0 对应 candidate 第一个采样点（current pose 投影）。
rollout 终点 t_end 对应 candidate 最后一个采样点（goal 邻域）。
不延伸 rollout 到 settling 阶段；
candidate path 自身不覆盖 v→0 之后的残振，模型 rollout 也不覆盖。
```

输出风险指标（全部基于 dt 重采样后的时域序列）：

```text
h_p95_pred              dt 序列 |h(t)| 的 p95，h(t) 由 (eta_x, eta_y) 经现有 height_coeff 映射
h_max_pred              dt 序列 |h(t)| 的 max
eta_dot_rms_pred        sqrt(mean(eta_dot_x² + eta_dot_y²))
modal_energy_rms_pred   sqrt(mean(E))
path_terminal_E         t = t_end 处的 E（注意：是 path 末端瞬态，不代表 settling 残振）
path_terminal_eta_norm  t = t_end 处的 sqrt(eta_x² + eta_y²)
```

terminal 命名说明：

```text
不使用 terminal_energy / terminal_eta_norm 等无前缀名，
是为了避免与 settling 阶段（v=0 后的自由衰减）混淆。
本方案 rollout 不覆盖 settling，path_terminal_* 仅用于体现 candidate 在 path 末端
是否仍处于高激励状态，作为 score 的一个软分量，权重不应过大。
若后续要预测 settling 残振，需要在 candidate rollout 后追加固定时长的零输入自由响应段，
这是后续工作，不在第一版范围。
```

score 归一化：

```text
所有 _norm 指标采用 batch 内归一化:
  x_norm,i = x_i / max(epsilon, max_j x_j)
其中 max_j 在当前规划周期同一候选集内取，candidate 间相对比较，不跨规划周期累积。

理由:
  cross-batch 比较意义有限——不同规划周期的 path 长度、起点 pose、goal 距离都不同，
  全局归一化会让 score 受 batch 难度漂移影响；
  batch 内归一化只回答"在本周期候选里谁最低激励"，与 selector 的实际职责一致。

epsilon = 1e-6 防止全 0 batch 除零。
单一 candidate 的 batch（仅 original 通过 hard gate）直接退化为现有 geometry-only 选择，
不进 slosh score 路径。
```

## 5. 候选评分

当前 geometry-only score 可升级为：

```text
score =
  w_h    * h_p95_pred_norm
+ w_E    * modal_energy_rms_pred_norm
+ w_edot * eta_dot_rms_pred_norm
+ w_term * path_terminal_E_norm
+ w_k    * kappa_p95_norm
+ w_dk   * dkappa_p95_norm
+ w_l    * length_ratio_penalty
+ w_x    * max_drift_norm
```

第一版建议：

```text
slosh rollout 只做 score / tie-break；
不要作为 hard gate。
```

原因：

```text
模型 rollout 与闭环执行之间仍有 MPC tracking、底盘动力学和估计误差；
一开始做 hard gate 容易误杀有效 candidate。
```

必须继续保留 hard gate：

```text
collision check
max_drift
length_ratio
endpoint_error
path direction
min_segment_length
```

## 6. Step 0：先离线验证，不直接改在线节点

在改 `anti_slosh_path_post_processor.py` 前，先写离线诊断脚本。

输入：

```text
必选样本（最少跨 2 个 open 场景目标）:
  open_user_goal GEOREF bags x ≥ 5（已有，2026-05-06/07）
  open_user_goal RAW bags    x ≥ 3（已有）
  open_goal_b    GEOREF bags x ≥ 3（需补采）
  open_goal_b    RAW bags    x ≥ 3（需补采）

每包必须保存或可重建:
  /scout/global_path                MBF raw path
  /scout/global_path_anti_slosh     selected candidate
  /anti_slosh_path/candidate_report 全 candidate 的 kappa/ay/length/drift/collision 信息
  /slosh/height                     闭环主指标
  /slosh/state, /slosh/modal_energy 闭环辅助指标
  /odom                             tracking 与 ay/ax 真值
```

跨目标说明：

```text
2026-05-06 阶段性总结明确 “不能宣称任意路径泛化”。
Step 0 只用 open_user_goal 验证等同 in-sample 测试，结果不可外推。
必须在 open_goal_b 上同时复现 PASS 条件，结论才允许写成 “open 场景代表性目标”；
若 open_goal_b 无 GEOREF non-original 选择，说明 candidate generator 在该目标空间已退化，
本方案在该目标上没有可评分的 candidate 集合，应记录为 scope boundary，不算 FAIL。
```

要回答的问题：

```text
Q1. slosh rollout score 是否会选择当前成功的 mild/medium candidate？
Q2. slosh rollout predicted ranking 是否和闭环 /slosh/height 改善方向一致？
Q3. slosh rollout 是否会错误偏向过度平滑或过长路径？
Q4. eta_dot / modal_energy 指标是否能拦住”h 降但 eta_dot 升”的候选？
Q5. rollout 预测的最优 candidate 与当前 geometry-only selected 之间，
    h_p95_pred / modal_energy_rms_pred 还有多少剩余空间？
```

Step 0 验收（三分支判据）：

```text
PASS:
  跨 open_user_goal + open_goal_b 两个目标同时满足:
    (a) rollout score 选择 non-original candidate；
    (b) rollout 选择不劣于当前 geometry-only selected；
    (c) predicted h / energy / eta_dot 排序与闭环主指标方向一致（Spearman ρ ≥ 0.5）；
    (d) 不系统性偏向 strong 过度平滑或显著变长路径（length_ratio < 1.10）；
    (e) 改进空间充足（见下方 saturation floor）。
  允许进入 Step 1 在线实现。

SATURATED:
  (a)-(d) 满足，但 rollout 预测的最优 candidate 与当前 geometry-only selected 之间
  h_p95_pred 相对差距 < 5%，且 modal_energy_rms_pred 相对差距 < 5%。
  含义：geometry-only 已经接近本方法在 open 场景下的选择上界。
  处置：不进入 Step 1。
        slosh rollout 仅作为离线 ablation / 论文 analysis section 报告，
        不接入在线节点，不参与实物验证。
        主线维持 geometry-only Online GeoRef + normal MPC。

FAIL:
  以下任一成立即 FAIL:
    rollout score 经常选 original（≥ 50% 周期）；
    rollout 选出的 candidate 与闭环 /slosh/height 方向相反（Spearman ρ ≤ 0）；
    rollout score 主要靠变长/过度平滑降低风险（length_ratio ≥ 1.10 占主导）；
    eta_dot / energy 预测与闭环明显反向。
  处置：见 §10.1 FAIL 退出策略。
```

saturation floor 设定理由：

```text
B1 GEOREF vs B0 RAW 在 open_user_goal 上已实现 h_p95 -18.6% / eta_dot_rms -11.5%。
若 slosh rollout 在已选 candidate 之外只能再挖出 < 5% 的 h_p95_pred，
即使 Step 1/Step 2 仿真名义上”通过”，闭环噪声地板也会把信号吃掉，
最终落入 GEOREF_CONSTRAINED 同款的”勉强达标但论文不可用”陷阱。
预先设 SATURATED 分支是为了显式拦截这种结局，而不是事后追认。
```

Step 0 不通过（FAIL 或 SATURATED），均不进入在线实现。

## 7. Step 1：在线评分接入

只有 Step 0 通过后，才修改：

```text
scripts/anti_slosh_path_post_processor.py
launch/anti_slosh_path_post_processor.launch
```

新增参数建议：

```yaml
slosh_score_enable: false
slosh_score_weight_h: 1.0
slosh_score_weight_energy: 0.5
slosh_score_weight_eta_dot: 0.5
slosh_score_weight_terminal: 0.2
slosh_score_omega_n: 31.25
slosh_score_zeta: 0.05
slosh_score_height_coeff: <沿用当前 /slosh/height 映射>
```

默认必须关闭：

```text
slosh_score_enable=false
```

第一版在线实现只改变 candidate score，不改变 candidate gate，不改变 MPC。

## 8. Step 2：仿真验证矩阵

主对比：

```text
B0 RAW:
  raw MBF path + normal MPC

B1 Geometry-only GeoRef:
  当前 Online GeoRef + normal MPC

Ours Slosh-model-guided GeoRef:
  slosh rollout score + normal MPC
```

保留消融：

```text
A1 RAW_SLOW_MATCHED
A2 GEOREF_ORIGINAL
A3 GEOREF_CONSTRAINED 负结果
A4 MPC slosh-cost 负结果
```

Ours 相对 B1 的通过标准：

```text
h_p95 不低于 B1 noise floor 的前提下进一步下降，目标 >= 5%
eta_dot_rms 不上升
modal_energy_rms 不上升，最好下降
tracking_time 不超过 B1 +5%
ay_p95 不上升
selected candidate 不退化为 original
```

如果 Ours 与 B1 持平：

```text
保留 geometry-only GeoRef 作为主线；
slosh-model-guided score 放入 ablation / discussion。
```

如果 Ours 劣于 B1：

```text
不接入主线；
记录为模型 rollout 与闭环执行不一致的负结果。
```

## 9. 实物阶段使用方式

第一轮实物 open 验证仍先跑当前稳定主线：

```text
RAW_REAL x3
GEOREF_REAL x3
```

不要让 Slosh-model-guided GeoRef 阻塞实物 open 验证。

如果 Step 0/1/2 仿真通过，再做：

```text
GEOREF_SLOSH_SCORE_REAL x3
```

实物必须同步记录：

```text
/anti_slosh_path/candidate_report
/scout/global_path_anti_slosh
/slosh/height
/slosh/state
/slosh/modal_energy
视觉液面话题，如果 RealSense 可用
```

实物结论要区分：

```text
/slosh/height 估计值改善；
视觉真液面改善；
两者方向一致。
```

## 10. 不要做的事

本方案明确不做：

```text
不重新打开 Q_slosh 当主控制项；
不加 eta_box_constraint；
不复活 GEOREF_CONSTRAINED；
不把 slosh rollout 作为 hard gate；
不做 SLOSH_SETTLING；
不做 MPCC / SLSQP 大优化；
不在 Step 0 未通过前改在线节点。
```

## 10.1 FAIL 退出策略

Step 0 FAIL（见 §6 验收 FAIL 分支）的物理含义已经被多轮失败实验佐证：

```text
Q_slosh / Q_eta_dot / output guard / GEOREF_CONSTRAINED 全部失败，
且 Step 0 又显示 slosh rollout 排序与闭环不一致，
则可以判断:
  open 场景下 /slosh/height 的闭环主导项不是几何路径形状或参考速度上界，
  而是 MPC tracking 行为 + 底盘动力学 + 估计器一致性。
```

FAIL 之后的处置：

```text
1. slosh model 不再用作 path selection 评分，
   只保留为离线诊断与论文 ablation 工具。

2. 主线立即回退到 geometry-only Online GeoRef + normal MPC：
   不阻塞实物 open 验证（RAW_REAL / GEOREF_REAL）；
   不再开新的 MPC 内部增强分支；
   不再开新的参考层 slosh-aware 分支。

3. 如果未来仍要进一步压低 slosh，
   研究方向必须切换到当前主线之外的对象，例如：
     底盘悬挂 / 容器固定方式 / 容器内挡板（机械层）；
     更高保真的非线性 slosh 估计（估计层）；
     视觉真液面闭环（感知层）。
   这些都属于新工作，与本方案 / 本论文主线不再耦合。
```

SATURATED 分支（见 §6 验收 SATURATED）按 §6 处置即可，不重复列在退出策略里——
它与 FAIL 的关键区别是：模型方向**对**，但**没空间**；不需要否定模型，只需要不接入在线。

## 11. 论文定位

如果本方案通过，论文贡献可以从：

```text
Online geometry smoothing reference generation
```

增强为：

```text
Slosh-model-guided online geometric reference generation
```

推荐表述：

```text
Although the slosh states are not used as dominant MPC cost terms,
the same modal slosh model is used to evaluate candidate geometric references
and select a low-excitation path before MPC tracking.
```

中文：

```text
虽然晃动状态不再作为 MPC 主导代价项，
但同一模态晃动模型用于候选几何参考的风险评价，
从而在 MPC 跟踪前选择低晃动风险路径。
```

## 12. 2026-05-07 在线接入状态

虽然 Step 0 结果更接近 SATURATED，用户仍要求为了论文创新点接入在线节点。

当前工程处理：

```text
已接入 slosh_score 在线评分；
默认 slosh_score_enable=false；
不改 MPC；
不改 hard gate；
不影响 GEOREF_TUNED；
新增 GEOREF_SLOSH_SCORE 作为 ablation condition。
```

修改文件：

```text
scripts/anti_slosh_path_post_processor.py
launch/anti_slosh_path_post_processor.launch
scripts/run_sim_fixed_path_bag.sh
```

上线边界：

```text
GEOREF_SLOSH_SCORE 不是当前 proposed method 的替代主线；
它是 Ours ablation / candidate scoring 增强入口；
必须通过 B1 Geometry-only GeoRef vs Ours Slosh-score GeoRef x3 对比后，才允许写成主方法。
```

当前预期：

```text
由于 Step 0 中 slosh_winner == geometry_winner == medium，
第一轮 GEOREF_SLOSH_SCORE 大概率复现 GEOREF_TUNED，
未必产生额外收益。
```

验证入口：

```bash
PATH_MODE=global_goal CONDITION=GEOREF_SLOSH_SCORE PATH_ID=open_user_goal RUN_ID=01 \
GOAL_X=-3.1570560932159424 \
GOAL_Y=-2.897411346435547 \
GOAL_QZ=-0.978164583074326 \
GOAL_QW=0.2078317791364693 \
POST_PROCESSOR_COLLISION_CHECK=false \
RECORD_DURATION=25 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

检查：

```bash
rostopic echo -b <bag> -n1 /anti_slosh_path/candidate_report
```

有效 report 应包含：

```text
gscore
sscore
sE
sEdot
```

## 13. 2026-05-07 x3 验证后的收束

已完成 open_user_goal `GEOREF_SLOSH_SCORE` x3：

```text
/data/a/slosh_bags/sim/20260507/20260507_open_user_goal_GEOREF_SLOSH_SCORE_run01_160044.bag
/data/a/slosh_bags/sim/20260507/20260507_open_user_goal_GEOREF_SLOSH_SCORE_run02_160357.bag
/data/a/slosh_bags/sim/20260507/20260507_open_user_goal_GEOREF_SLOSH_SCORE_run03_160614.bag
```

关键现象：

```text
run01: selected=medium，有效 non-original slosh-score 样本
run02: no candidate passed gates，fallback original
run03: selected=original，mild/medium 被 ay gate reject
```

三包均值结论：

```text
GEOREF_SLOSH_SCORE 相对 GEOREF_TUNED:
  tracking     -6.70%
  h_rms        +12.59%
  h_p95        +10.34%
  h_max        +19.87%
  eta_dot_rms  -0.19%
  energy_rms   +11.66%
  odom_ay_p95  +72.30%
```

判定：

```text
GEOREF_SLOSH_SCORE 当前版本不通过主方法门槛。
它没有超过 geometry-only GEOREF_TUNED，
且 run02/run03 出现 original fallback / ay gate reject，
说明 score + gate 组合稳定性不足。
```

后续定位：

```text
1. 主线回到 GEOREF_TUNED geometry-only online GeoRef + normal MPC。
2. Slosh score 保留为默认关闭 ablation 和离线诊断工具。
3. 不建议继续通过放松 gate 或重调权重硬救该路线；
   否则会引入新的调参自由度，并削弱当前 geometry-only 正结果的干净性。
4. 论文中可写成：
   slosh model was further evaluated as an online candidate scoring signal,
   but did not outperform the simpler geometry-risk scoring under the current candidate set.
```

## 14. GEOREF_SLOSH_SCORE_TUNED 调优版

由于论文仍需要保留 “slosh-model-guided candidate scoring” 的实现入口，
新增 `GEOREF_SLOSH_SCORE_TUNED`，与旧 `GEOREF_SLOSH_SCORE` 分开。

调优目标不是直接追求 `/slosh/height` 单包下降，而是先解决旧版的工程失败模式：

```text
旧版 run02/run03:
  candidate 在进入 slosh score 前被 min_seg / ay gate 拦下；
  score 没有真正参与选择；
  最终 fallback original。
```

调优原则：

```text
1. 让合理 candidate 能进入 scoring。
2. 不大幅放松安全 gate。
3. 对 predicted_ay_ratio 加 score penalty，避免横向激励反弹。
4. 保持 Q_slosh=0，MPC 仍是 normal tracking MPC。
5. 不覆盖 GEOREF_TUNED，所有结果作为独立 condition 对比。
```

实现改动：

```text
candidate 平滑后再 sanitize；
min_segment_length: 0.02 -> 0.005，仅 tuned condition 生效；
ay_ratio_limit: 1.00 -> 1.05；
新增 slosh_score_w_ay=1.5；
降低 kappa/dkappa 几何权重，让 slosh energy/eta_dot/ay 参与主排序。
```

默认参数：

```text
condition: GEOREF_SLOSH_SCORE_TUNED

slosh score:
  w_h=0.0
  w_energy=1.0
  w_eta_dot=0.5
  w_terminal=0.2
  w_kappa=0.5
  w_dkappa=0.3
  w_ay=1.5
  w_length=0.3
  w_drift=0.5
```

第一步验收只看 candidate_report：

```text
accepted >= 2
selected != original
ayr <= 1.05
gscore/sscore/sE/sEdot 均有效
```

第二步才录 x3，对比 `GEOREF_TUNED`：

```text
h_p95 不升
energy 不升
eta_dot 不升
odom_ay_p95 不升
tracking_time 不超过 GEOREF_TUNED +5%
selected 至少 2/3 为 non-original
```

如果调优版仍不满足，则结论收束为：

```text
slosh rollout score 当前更适合离线解释与风险诊断，
不适合作为 online candidate selection 的主权威。
```
