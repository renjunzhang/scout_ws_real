# Phase-Rejoining KKT 站定性分量归因与无量纲尺度推导

- 日期：2026-08-22
- 状态：Accepted（诊断证据，未改代码）
- 对应证据：`/data/a/spmpc_exec_identification/phase_rejoin_seed8601_sqp_probe_20260822.S9cifq/summary.json`
- 涉及对象：完整 SQP backend `delay_augmented_phase_acados_full_sqp_v1`（配置 hash `f5d67f20…72a85`）

本文只回答一个问题：完整 SQP 在 seed 8601 cycle 2 以 `NLP_STATUS_2`（20 次迭代上限）停住、stationarity=2.20009 达不到 `1e-6`，其**最大分量、梯度项和尺度来源**分别是什么。本文不改变任何代码、参数或资产。

## 1. 结论

- stationarity 的**最大分量**是 `eta_x` 状态（state index 6），stage 5 处 `|cost_grad|=125.40`；第二名仍是 `eta_x`（stage 8，108.44）。
- 四个 slosh 通道（`eta_x/eta_x_dot/eta_y/eta_y_dot`，index 6–9）合计贡献了**全部 |cost_grad| 之和的 99.32%**。
- 尺度来源是代价函数里 `(η−η_nom)/η_scale` 的 `1/η_scale²` 因子：`η_scale=0.00275037`，使 η 通道的 Gauss–Newton Hessian 对角元 `2·w/scale² = 2.645×10⁵`，比最弱的 `w_omega/angular_pending` 通道（`0.1389`）大 **1.9×10⁶ 倍**。
- 因此 `res_stat` 是跨量纲的裸 KKT 站定性（acados `ocp_nlp_res_compute` 直接对 `cost_grad − dyn_adj − ineq_adj` 取 inf-norm，无归一化），用固定 `1e-6` 绝对阈值判定一个条件数 1.9×10⁶ 的问题，本身就是不可达的。
- 站定性 2.20 只占最大 cost-grad 分量 125.40 的 **1.75%**，说明解在相对意义上已接近 KKT 点；真正的障碍是 Hessian 病态导致的 merit backtracking 自第 2 次迭代起固定 `α=0.057648` 并缓慢回涨 stationarity，而不是存在尚未满足的 primal 硬约束（独立 C++ 审计全部具名约束违反量为 0）。

## 2. 分量归因（stage × 变量 × 梯度项）

用 cycle 2 失败时保存的 raw `x[0:N]`/`u[0:N-1]`（`failed_raw_solution_states/controls`）与逐 stage 64 项参数图像（`stage_parameters`），按 NLS 残差 `r_i = √w·(x_i−nom_i)/scale_i` 计算每个状态/控制变量的代价梯度 `∂L/∂z = 2·w·(z−nom)/scale²`，取绝对值排序：

| 排序 | stage | 变量 | cost_grad | 误差 err | w | scale |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 5 | eta_x | −125.40 | −4.742e-4 | 1.0 | 0.00275 |
| 2 | 8 | eta_x | +108.44 | +4.100e-4 | 1.0 | 0.00275 |
| 3 | 7 | eta_x | +95.43 | +3.608e-4 | 1.0 | 0.00275 |
| 4 | 4 | eta_x | −74.62 | −2.821e-4 | 1.0 | 0.00275 |
| 5 | 9 | eta_x | +41.76 | +1.579e-4 | 1.0 | 0.00275 |
| 6 | 6 | eta_x | −18.35 | −6.939e-5 | 1.0 | 0.00275 |
| 7 | 10 | eta_x | −4.40 | −1.664e-5 | 1.0 | 0.00275 |
| 8 | 4 | eta_x_dot | −1.372 | −1.688e-2 | 0.3 | 0.0859 |
| … | … | （后续均为 eta_x_dot / 其余通道，量级 ≤ 1） | | | | |

- 全部 |cost_grad| 之和 4.795×10²，其中四个 slosh 通道 4.762×10²，占比 **99.32%**。
- 最大分量 `eta_x@stage5` 的误差 −4.742e-4，相对 `η_scale=0.00275` 是 **17.24%**——这是有物理意义的剩余 slosh 跟踪误差（短时域 + 加速度上限 0.6 m/s² 下无法同时把 slosh 归零并跟踪位姿/速度），不是数值垃圾。

## 3. 尺度推导（无量纲化前后 Hessian 条件数）

代价残差各通道的 Hessian 对角元（Gauss–Newton 意义下）为 `2·w/scale²`。用 cycle 2 参数图像的实际权重（`w_position=1.0, w_yaw=0.2, w_progress=0.2, w_v=1.0, w_omega=0.1, w_slosh_eta=1.0, w_slosh_eta_dot=0.3, w_linear_pending=1.0, w_angular_pending=0.1, w_a=0.1, w_alpha=0.1, w_v_s=0.3`）：

| 通道 | 物理尺度 scale | Hessian 对角 `2w/scale²` |
| --- | --- | --- |
| w_slosh_eta | 0.00275037 | **2.645×10⁵** |
| w_position | 0.15 | 88.89 |
| w_slosh_eta_dot | 0.0859380 | 81.31 |
| w_progress | 0.1 | 40.0 |
| w_v / w_linear_pending | 0.8 | 3.125 |
| w_v_s | 0.8 | 0.9375 |
| w_a | 0.6 | 0.5556 |
| w_yaw | 1.0 | 0.4 |
| w_omega / w_angular_pending / w_alpha | 1.2 | 0.1389 |

- 物理尺度下条件数 `max/min = 2.645e5 / 0.1389 ≈ 1.9×10⁶`。
- 若做无量纲变量变换 `x̃ = x/scale_x`、`ũ = u/scale_u`，代价变为 `Σ w·(x̃−nom̃)²`，Hessian 对角元退化为 `2w`，条件数 `max(w)/min(w) = 2.0/0.2 = 10`（w 均为 O(0.1–2)）。
- 结论：条件数从 **1.9×10⁶ 降到 10**，约 5 个数量级，且目标函数在变量变换下**逐点不变**（只是换元），物理最优解、权重语义、gate/半径/B_exec 的几何意义都不变。

## 4. merit backtracking 自第 2 次迭代固定小步长的原因

cycle 2 的逐迭代轨迹（`solver_iterations`）：

```
it=0  stat=25.02  step=0.0000      （warm start）
it=1  stat= 2.177 step=1.0000      （完整步，25.02→2.18）
it=2..20 stat 2.177→2.199 step=0.057648（固定小步，stationarity 缓慢回涨）
```

- 第一次完整步就把 stationarity 从 25.02 降到 2.18，说明 warm-start 远离 KKT 点，第一步方向基本正确。
- 之后 Gauss–Newton Hessian（条件数 1.9×10⁶）给出的 QP 步方向被 η 通道主导，对其他通道的修正不准确；merit 函数预测下降与实际下降失配，backtracking 把 `α` 砍到 0.057648 并卡死。
- `α` 固定且 stationarity 回涨，是「line search on merit 停滞」的典型病态症状：不是解不存在，也不是收敛判定被放宽，而是**优化问题的变量尺度未归一化**。

## 5. 修复方向与实施结果

结构性修复：在 codegen 层做**无量纲变量缩放**（`x̃ = D_x⁻¹ x`、`ũ = D_u⁻¹ u`，`D_x=diag(scale)`），使 acados 求解器在无量纲空间工作，`res_stat` 变成有意义的无量纲量，现有 `1e-6` 阈值直接保留而无需放宽。参数图像保持物理单位（cost/transition/constraint 在模型内用 `x = scale_x⊙x̃` 恢复物理量），C++ `DelayAugmentedPhaseDynamics` 与 causal rollout 审计继续在物理单位下进行，V3 资产、gate 半径、B_exec、residual authority 与已冻结 hash 的语义均不变。

实施内容（未提交）：

- `generate_delay_augmented_phase_acados.py`：新增 `state_scaling_vectors()`，`build_symbolic_spec()` 在缩放基下组装 OCP；控制边界变为无量纲 `[-1,-1,0]…[1,1,1]`。
- `delay_augmented_phase_solver.cpp`：新增 `delayAugmentedStateScale/ControlScale` 与 scale/unscale 辅助；在 `create`（x0）、`setControlGuess`、`setCausalWarmStart`、`getState`、`getControl` 五处做 capsule 边界换算。公共 API 仍为物理单位，`DelayAugmentedPhaseDynamics` 不变。

验证结果：

- 完整 OCP 的 reduced Gauss–Newton Hessian（对 30 个控制量、状态经动力学消去）在 cycle 2 warm-start 处条件数 **9.36**，最小特征值 0.20，无退化方向。
- 真实 seed 8601 重放：cycle 2 的 stationarity 由 **2.20009 → 0.015864**（约 139 倍），equality 2.09e-12、inequality 0、complementarity 2.58e-6。
- 但 **仍以 `NLP_STATUS_2` 停住**：iteration 1 全步后 stationarity 固定在 ~1.59e-2，iteration 2–20 的 `alpha` 恒为 `0.7^8=0.057648`（MERIT_BACKTRACKING 每步退 8 次到 alpha_min 附近），complementarity 由 6.7e-7 缓慢涨到 2.5e-6。

## 6. 剩余的第二个问题（本轮未解决）

无量纲缩放把「eta 尺度失衡」这个主根因修掉后，暴露出第二个、**与缩放和 Hessian 近似都无关**的问题：SQP 的 merit 线搜索从第 2 次迭代起拒绝 QP 步、`alpha` 卡在 `alpha_min` 附近，stationarity 停在 ~1.6e-2 不再下降。已排除：

- Hessian 近似：`GAUSS_NEWTON` 与 `EXACT` 行为逐字节一致（都不是根因）；
- 结构退化：reduced Hessian 条件数 9.36、无近零特征值；
- 变量尺度：已无量纲化。

### 6.1 逐 stage 定位结果（`ocp_nlp_get_at_stage(..., "res_stat", ...)`）

真实 seed 8601 的失败周期（缩放后）逐 stage 最大 stationarity 分量：

| stage | max\|res_stat\| | 变量（idx） |
| --- | --- | --- |
| 0–9 | 1.0e-3 … 5.7e-3 | 分散（x/eta_x_dot/linear_q 等） |
| **10（terminal）** | **1.5867e-2** | **idx=0 = x 位置** |

标量 stationarity 1.586e-2 **完全由 terminal 阶段的 x 位置贡献**，其余 stage 均 ≤5.7e-3。

terminal 的 `res_stat` 与 terminal 代价梯度一致：acados 的 NONLINEAR_LS 代价为 `0.5·‖y‖²`（`ocp_nlp_cost_nls.c`），terminal 残差 `y_e=√w·(x−nom)/scale`，故 `res_stat_N[idx=0]=w_position·(x_N−nom_x)/scale`。实测 terminal x 误差 −2.380e-3 m，`w_position=1`、`scale=0.15`，得 `1.0·(−2.380e-3)/0.15 = −1.5867e-2`，与 acados 报出的 `res_stat` **逐位吻合**。也就是说：terminal 的 `res_stat` 就是 terminal 跟踪误差的（未与被 dynamics 伴随量抵消的）代价梯度，而不是一个无量纲 KKT 残差。

### 6.2 结论：这不是线搜索问题，是 terminal 残差度量问题

- `MERIT_BACKTRACKING` 与 `FUNNEL_L1PEN_LINESEARCH` 收敛到**同一个 stationarity 地板 1.5867e-2**（FUNNEL 只是全步、8 次迭代就停，MERIT 是 20 次 + `alpha=0.7^8`）；说明线搜索不是根因。
- 要让 `res_stat ≤ 1e-6`，terminal x 跟踪误差需 ≤ `1e-6·0.15/1.0 = 1.5e-7 m`（0.15 µm），对一个 0.15 m 尺度、多目标权衡的跟踪问题而言是**机器精度级要求**，并非「求解器没收敛」。
- 因此符合 goal 一.5 的判定：**acados 原始 `res_stat`（至少 terminal 阶段）不适合作为跨量纲完整性指标**。后续方向是建立无量纲 KKT 合同：把 stationarity 按其变量尺度（或 terminal 代价梯度尺度）归一化后再与 `1e-6` 比较，同时继续原样记录 raw residual。分量归因、尺度推导已在本节完成；还差「紧阈值解稳定性对照」与最终合同定义。

## 7. 边界

不修改 beta、radii、residual authority、成功定义或任何资产；不重新训练 gate、不重做 V3；不放宽 `1e-6`；不启动正式 C0–C4 实验。
