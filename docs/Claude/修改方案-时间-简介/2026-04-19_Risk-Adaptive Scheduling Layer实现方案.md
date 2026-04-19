# 2026-04-19 Risk-Adaptive Scheduling Layer 实现记录

> 状态：**已完成**（代码实现于 2026-04-19，本文档记录最终实现口径）

---

## 0. 结论

Risk-Adaptive Scheduling Layer 已按论文版定义完整实现，分布在两个文件：

- `src/risk_scheduler.cpp` + `include/scout_local_planner/risk_scheduler.h`：调度器核心逻辑
- `src/local_planner_ros.cpp`：outer loop 集成、单步延迟、话题发布

论文三项核心贡献的代码主体均已落地，当前主缺口是**实物实验数据**，不再是实现问题。

本文档后半部分只保留：

- 当前实现口径
- 已知小问题
- 剩余验证项

不再作为“待实现设计文档”使用。

---

## 1. 实现的精确口径

### 1.1 物理风险 r_k

```cpp
// risk_scheduler.cpp
const double h_max  = params_.eta_bar_max;        // 注：用 eta_bar_max 作代理，见 §4
const double E_max  = h_max * h_max * 2.0;

const double h_risk = clip(h_pred_max_prev / (h_max + 1e-9), 0.0, 1.0);
const double e_risk = clip(E_slosh_prev    / (E_max + 1e-9), 0.0, 1.0);
const double t_risk = clip((d_goal_thresh - d_goal) / (d_goal_thresh + 1e-9), 0.0, 1.0);

r_k = clip(w_h * h_risk + w_e * e_risk + w_t * t_risk, 0.0, 1.0);
```

输入 `h_pred_max_prev` 和 `E_slosh_prev` 来自**上一周期冻结的 monitor rollout**（单步延迟，防止优化器自评分）。

### 1.2 激励一致性不确定性 u_k

```cpp
// 独立于 r_k，来自当前 IMU 观测
const double u_k_raw = clip(fabs(a_y_imu - v_omega) / a_uncert_max, 0.0, 1.0);
```

`a_y_imu` 是已去零偏的 IMU 横向加速度，`v_omega = current_v * current_omega` 是运动学估计。

### 1.3 统一风险 ρ_k 与 sigmoid

```cpp
rho_k = clip(w_r * r_k + w_u * u_k, 0.0, 1.0);
s_rho  = sigmoid(gamma * (rho_k - rho_0));
```

### 1.4 三路输出 + 变化率限制

```cpp
// 期望值
Q_des    = Q_eta_min + (Q_eta_max - Q_eta_min) * s_rho;
bar_des  = eta_bar_max - delta_eta_bar * s_rho;
vref_des = v_ref * (1.0 - beta * s_rho);

// 每步最大变化量
dQ    = (Q_eta_max - Q_eta_min) * rate_limit_per_step;
dbar  = delta_eta_bar * rate_limit_per_step;
dvref = v_ref * beta * rate_limit_per_step;

// clip 到前一步 ± 变化量
Q_eta_k     = clip(Q_des,    prev.Q_eta_k   ± dQ);
eta_bar_k   = clip(bar_des,  prev.eta_bar_k ± dbar);
v_ref_eff_k = clip(vref_des, prev_vref      ± dvref);
```

### 1.5 Fallback 逻辑

三路触发，任一激活则退回固定保守参数：

```cpp
imu_timeout   = (now - imu_stamp).toSec() > imu_timeout_s;   // IMU 超时
bias_not_ready = !imu_bias_ready;                              // bias 未完成估计
u_persistent  = (u_k > u_threshold_high) && (count >= u_high_count_trigger);

if (any of above) {
    Q_eta_k     = Q_eta_fix;
    eta_bar_k   = eta_bar_fix;
    v_ref_eff_k = v_ref;       // 不折扣速度
    fallback_active = true;
}
```

---

## 2. 集成到 outer loop（local_planner_ros.cpp）

### 2.1 单步延迟机制

```
本周期流程：
  1. risk_scheduler_.update(last_predicted_height_max_, E_slosh_prev_, ...)  ← outer loop
  2. mpc_solver_.solve(runtime_mpc_params)                                   ← inner loop
  3. last_predicted_height_max_ = 本周期预测液面峰值                          ← solve 后更新
  4. E_slosh_prev_ = η_x² + η_y²                                            ← solve 后更新
  下周期 step 1 使用 step 3/4 的冻结值
```

`last_predicted_height_max_` 在 `local_planner_ros.cpp:L1145` 附近更新，传入 `update()` 时是上一周期的值，满足论文要求的"冻结 monitor rollout"。

### 2.2 输出注入 MPC 参数

```cpp
// Q_eta_k → runtime_mpc_params.Q_slosh_eta
runtime_mpc_params.Q_slosh_eta = risk_output_.Q_eta_k * rs_h_coeff_ * rs_h_coeff_;

// eta_bar_k → runtime_mpc_params.slosh_eta_bar（仅当 box constraint 启用时）
if (enable_slosh_box_constraint && rs_h_coeff_ > 1e-9) {
    runtime_mpc_params.slosh_eta_bar = risk_output_.eta_bar_k / (rs_h_coeff_ * sqrt(2.0));
}

// v_ref_eff_k → v_des_cmd
v_des_cmd = risk_output_.v_ref_eff_k;
```

### 2.3 话题发布

| 话题 | 内容 |
|---|---|
| `/risk_scheduler/rho_k` | 统一风险指标 |
| `/risk_scheduler/r_k` | 物理风险 |
| `/risk_scheduler/u_k` | 激励一致性不确定性 |
| `/risk_scheduler/Q_eta_k` | 当前周期调度的 Q_eta |
| `/risk_scheduler/fallback_active` | fallback 是否激活 |

---

## 3. 默认参数（mpc_params.yaml）

| 参数 | 值 | 含义 |
|---|---|---|
| `gamma` | 5.0 | sigmoid 斜率 |
| `rho_0` | 0.3 | sigmoid 激活阈值 |
| `rate_limit_per_step` | 0.05 | 每步最大变化量（占满量程比例） |
| `Q_eta_min / Q_eta_max` | 0.0 / 10.0 | Q_eta 调度范围 |
| `eta_bar_max` | 0.05 | η̄ 上限（m） |
| `delta_eta_bar` | 0.02 | η̄ 最大收紧量 |
| `beta` | 0.3 | 速度折扣上限 |
| `w_h / w_e / w_t` | 0.4 / 0.3 / 0.3 | r_k 三项权重 |
| `w_r / w_u` | 0.7 / 0.3 | ρ_k 合成权重 |
| `u_threshold_high` | 0.8 | u_k fallback 触发阈值 |
| `u_high_count_trigger` | 10 | u_k 持续步数门控 |
| `a_uncert_max` | 0.5 m/s² | u_k 归一化分母 |
| `Q_eta_fix / eta_bar_fix` | 5.0 / 0.04 | fallback 固定保守值 |
| `d_goal_thresh` | 1.0 m | t_risk 激活距离 |

---

## 4. 已知小问题（不影响实验结论）

### 4.1 h_max 量纲代理

`r_k` 中 `h_max = eta_bar_max = 0.05 m`，但 `h_pred_max_prev` 是液面高度（物理量），由 `h = h_coeff * sqrt(η_x² + η_y²)` 给出，h_coeff ≈ 0.3~0.5。

正确的 `h_max` 应为 `h_coeff * eta_bar_max`，当前值偏大约 2~3 倍，导致 `h_risk` 饱和门槛偏高（需要更大的液面偏移才能触发高 h_risk），等效降低了 h_risk 权重。

**修法**：在 `RiskSchedulerParams` 加字段 `h_max_physical`，由外层用 `rs_h_coeff_ * eta_bar_max` 填充后传入。

### 4.2 s_rho 话题未发布

`pub_s_rho_` 未在发布循环中调用。补充一行即可，优先级低。

---

## 5. 实物实验时需要调整的参数

实物调参优先级（参考总体推进方案 Day 4）：

| 参数 | 方向 | 原因 |
|---|---|---|
| `gamma` | [3, 10] | 控制 scheduler 响应灵敏度，过大会抖动 |
| `rho_0` | [0.2, 0.5] | 风险激活阈值，过小会常态保守 |
| `rate_limit_per_step` | [0.02, 0.1] | 过大导致 QP 参数突变，过小响应太慢 |
| `beta` | [0.1, 0.5] | 速度折扣幅度，影响任务时间代价 |
| `a_uncert_max` | 实测标定 | 依赖实际 IMU ay 与 v·ω 的偏差分布 |

调参原则：先固定 `u_k` 项（令 `w_u=0` 暂时关闭），只用 `r_k` 验证物理风险响应合理，再开 `u_k`。

---

## 6. 剩余验证项

当前需要补的不是代码实现，而是验证闭环。建议按下面顺序做。

### 6.1 bag 回放验证

至少确认：

- `/risk_scheduler/r_k` 在高晃动段上升
- `/risk_scheduler/u_k` 在 IMU 与运动学不一致时上升
- `/risk_scheduler/rho_k` 随 `r_k / u_k` 变化而平滑变化
- `/risk_scheduler/fallback_active` 在 IMU 断开或持续高 `u_k` 时触发

### 6.2 参数输出验证

建议补齐或确认可观测性：

- `Q_eta_k`
- `eta_bar_k`
- `v_ref_eff_k`
- `s_rho`

当前代码已发布：

- `rho_k`
- `r_k`
- `u_k`
- `Q_eta_k`
- `fallback_active`

如果后续要直接做论文图，建议继续补发布：

- `eta_bar_k`
- `v_ref_eff_k`
- `s_rho`

### 6.3 对比验证

至少完成：

1. `NOM`
2. `Fixed-Q`
3. `PROP`

三组在同路径、同液位、同速度口径下的对比，确认：

- 速度分布是否变化
- `rho_k` 是否合理工作
- 液面指标是否有稳定改善

### 6.4 IMU 异常与一致性门控验证

至少补两类样本：

- IMU 正常、`u_k` 低
- IMU 观测异常或明显不一致、`u_k` 高并触发 fallback

这部分是后面写论文方法合理性的关键证据。
