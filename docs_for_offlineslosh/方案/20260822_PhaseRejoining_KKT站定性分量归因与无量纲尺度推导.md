# Phase-Rejoining KKT 站定性分量归因与无量纲尺度推导

- 日期：2026-08-22
- 状态：Partially superseded（尺度分析保留；terminal/costate 结论已纠正）
- 对应证据：`/data/a/spmpc_exec_identification/phase_rejoin_seed8601_sqp_probe_20260822.S9cifq/summary.json`
- 涉及对象：完整 SQP backend `delay_augmented_phase_acados_full_sqp_v1`（配置 hash `f5d67f20…72a85`）

本文只回答一个问题：完整 SQP 在 seed 8601 cycle 2 以 `NLP_STATUS_2`（20 次迭代上限）停住、stationarity=2.20009 达不到 `1e-6`，其**最大分量、梯度项和尺度来源**分别是什么。本文不改变任何代码、参数或资产。

## 1. 结论

- stationarity 的**最大分量**是 `eta_x` 状态（state index 6），stage 5 处 `|cost_grad|=125.40`；第二名仍是 `eta_x`（stage 8，108.44）。
- 四个 slosh 通道（`eta_x/eta_x_dot/eta_y/eta_y_dot`，index 6–9）合计贡献了**全部 |cost_grad| 之和的 99.32%**。
- 尺度来源是代价函数里 `(η−η_nom)/η_scale` 的 `1/η_scale²` 因子：`η_scale=0.00275037`，使 η 通道的 Gauss–Newton Hessian 对角元 `2·w/scale² = 2.645×10⁵`，比最弱的 `w_omega/angular_pending` 通道（`0.1389`）大 **1.9×10⁶ 倍**。
- 原始量纲确实使 `res_stat` 条件很差，无量纲变量缩放是有效的结构性修复；但不能据 cost gradient 与 raw residual 的比例断言 `1e-6` 合同不可达，因为完整 KKT 还必须包含 dynamics costate 与 inequality adjoint。
- 后续复核发现 Full SQP 使用 `HPIPM SPEED_ABS`，该模式关闭 equality dual 计算，导致 `pi=0` 和 terminal `dyn_adj` 缺项。改为 `BALANCE` 后 stationarity 进一步由 0.015864 降到 4.275e-5，证明原“真实 terminal 跟踪残差”归因不成立；当前剩余问题是 20 次 line-search SQP 尚未达到冻结阈值。

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

## 6. 伴随量复核纠正与剩余 NO-GO

原分析把“terminal 没有 outgoing dynamics”误解成“terminal 没有 dynamics adjoint”，从而漏掉了上一段动力学约束对 `x_N` 的 incoming `pi[N-1]`。正确公式是：

```text
res_stat_N = cost_grad_N - incoming_pi[N-1] - ineq_adj_N
```

进一步追踪确认，Full SQP codegen 原来显式选择 `HPIPM SPEED_ABS`；上游该模式设置 `comp_dual_sol_eq=0`，所以 `out->pi` 全零不是最优性结论，而是求解器没有计算 equality dual。`qp_solver_cond_N=N` 在 partial-condensing 中表示不做 condensing；此前把它称为“满 condensing 表示层现象”同样错误。

Full SQP 改为 `HPIPM BALANCE`、RTI reference 保持 `SPEED_ABS` 后，同一 cycle 2 快照得到：

- stage 0..9 的 `pi` 全部非零；
- terminal x 的 cost gradient 约 `-1.583e-2` 被 incoming `pi[9]` 抵消，x residual 降至约 `-4.2e-6`；
- 新最大项是 terminal `eta_x_dot`（index 7）：`cost_grad=-4.5166448e-3`、`incoming_pi=-4.4738975e-3`、`ineq_adj=3.9132e-9`、`res_stat=-4.2751176e-5`；
- 独立 C++ 按 empirical 9D gate 与 14 项 execution bounds 的完整 Jacobian/lam 重算，与 acados terminal 22 个分量在 `1e-9` 内一致；
- 四项 residual 为 stationarity `4.2751e-5`、equality `5.13e-11`、inequality `0`、complementarity `6.85e-7`，仍为 20 次上限 `NLP_STATUS_2`。

迭代 1 将 stationarity 从 `6.882e-2` 降到 `1.316e-4`；iteration 2 起 `alpha=0.057648`，stationarity 单调下降，但 20 次内只到 `4.275e-5`。因此尺度修复和 equality-dual 修复均有效，当前剩余项是**冻结 20 次 Full SQP 未达到 1e-6**，不是 terminal 跟踪误差需要趋近零，也不能通过删除 terminal cost、放宽 residual 或改 gate 解决。

## 7. 边界

不修改 beta、radii、residual authority、成功定义或任何资产；不重新训练 gate、不重做 V3；不放宽 `1e-6`；不启动正式 C0–C4 实验。
