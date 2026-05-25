# 2026-05-24 SloshPriorityMPC 物理保真度与残振治理方案

> 本方案不动轨迹参考层、不动 launch 接口、不引入新决策变量、不破 QP 形式。
> 在不增加 online solve time 的前提下，把 Ferrari 2026 论文里"物理保真度"和"残振约束"两条
> 真正可借鉴的部分搬进当前 MPC，提升 F 组结果的 RGB peak 抑制与模型可信度。

---

## 0. 目标与边界

### 0.1 触发问题

20260524 SloshPriorityMPC 实验报告（基于 20260522 d200 + 固定 P2 包）给出当前 F 组（slosh cost + 平滑）winner 结果：

| 指标 | C 基线 | F | F vs C |
|---|---|---|---|
| RGB p95 [mm] | 2.89 | 1.99 | **-31.2%** |
| RGB peak [mm] | 5.12 | 3.57 | **-30.2%** |
| ax p95 | 0.70 | 0.40 | -43.2% |
| ay p95 | 0.44 | 0.29 | -34.3% |
| 运行时间 | 14.3 s | 18.1 s | +26.9% |

报告自己点出的 3 条局限：

```text
1. 样本数仍少（每组 n=1）；
2. terminal approach 1s 仍有 jerk 脉冲污染液体；
3. /slosh/height 与 RGB peak 对应关系不稳定 → 模型保真度不够。
```

本方案针对的是**第 2 条与第 3 条**——纯样本扩充（第 1 条）属于实物执行，不在本方案范围。

### 0.2 严禁的"伪改善"

- 不改 prediction_v_max / v_max
- 不动 Q_lag / Q_contour / Q_etheta / Q_v 等跟踪权重
- 不改 terminal_capture_stop_distance / terminal_slowdown_distance 等 gate
- 不引入新参数让数据"看起来好"
- 不改 record/analysis 实验口径（RGB 主指标，/slosh/height 调试量）
- 不在方案里"顺手"修 bug 或重构（重构走 2026-05-20 那份方案）

### 0.3 与外部接口的关系

本方案改动落在两块：

| 层 | 改什么 | 是否破坏外部接口 |
|---|---|---|
| **slosh observer**（`local_planner_ros.cpp` 里 `updateSloshEstimate` + `slosh_integration.update`） | ay/ax/alpha 公式补充物理项 | 否（/slosh/* topic 输出维度不变） |
| **MPC cost / horizon 末端处理** | 强化已有 `terminal_factor_slosh_*` 接口 | 否（topic / param 名不变） |

`/mpc/cost_breakdown` 21 字段 layout 不变；新增 slosh excitation 调试 topic 可选（如 `/slosh/excitation_ay_path` vs `/slosh/excitation_ay_yaw_offset` 两个分量）。

---

## 1. 对 Codex "constraint-aware slosh MPC" 4 条建议的评估

| codex 建议 | 评估 | 决定 |
|---|---|---|
| **C1** h ≤ h_lim + slack 软约束 | 没本质改变；h_lim 选取无依据；与现有 Q_slosh 数学几乎等价（只多一个 kink） | **不采纳** |
| **C2** terminal residual cost（η_N² + ḣ_N²） | **有价值**，对应 Ferrari 式 19f；但你已有 `terminal_factor_slosh_eta/_eta_dot` 半成品 → 升级而非新增 | **采纳，做成 P2 的形态升级** |
| **C3** peak-aware z ≥ h_k 引入新决策变量 | 加 N+1 不等式 + 1 新变量，OSQP solve time 风险；4 次方版本破 QP → 需 NLP | **不采纳** |
| **C4** modal-excitation jerk penalty | 当前 `R_da/R_domega` 已经在做；除非升级成 modal frequency band-pass jerk，否则重复 | **不采纳** |

**Codex 答案最大的缺口**：完全没提到 Ferrari 剖析 §10 的 **M-A 项**——`ω̇·d / ω²·d` 离心耦合补偿。这是当前 model peak 不准的物理根因，**比 codex 的任何一条更值得做**。

---

## 2. 改进路线（按 ROI + 风险升序）

| 顺序 | 内容 | 工作量 | 风险 | 论文价值 |
|---|---|---|---|---|
| **P1** | yaw-offset 激励耦合（ω̇·d / ω²·d）补偿到 slosh observer | 2 d | 中 | **高**（method contribution） |
| **P2** | terminal residual 治理：强化 terminal_factor_slosh + horizon 末端 free-decay 延伸 | 1 d | 中 | 高（对应 Ferrari 式 19f） |
| **P3** | ζ_n 物理标定（Ferrari 式 3 代替固定 0.05） | 0.5 d | 低 | 中（first-principles credibility） |
| **P4** | γ_model / γ_opt 分析口径（Ferrari 式 25/26） | 0.5 d（脚本） | 极低 | 高（RA-L reviewer 标准范式） |
| **P5** | 实物扩样本（C/D/F 各 n=3）+ P1/P2 验证 | 1 d 实物 | — | 必做 |

**建议执行顺序**：P3（最低风险，建立 ζ baseline）→ P1（核心物理改进）→ P4（分析脚本）→ P2（terminal 治理）→ P5（实物验证）

---

### P1（核心）：yaw-offset 激励耦合补偿到 slosh observer

#### 问题

当前 observer 在 `local_planner_ros.cpp` 大约 1862 行附近：

```cpp
// 简化示意，实际公式见代码
ax_filtered_  ← v 差分 + EMA
ay_filtered_  ← v_ * omega_         // ← 只有路径切向曲率离心，缺 yaw-offset 项
alpha_filtered_ ← omega_ 差分 + EMA
slosh_integration_.update(ax_filtered_, ay_filtered_, omega_, alpha_filtered_)
```

Ferrari 论文剖析 §3 EOM 显式给出三类激励源：

```text
1. 容器质心平动加速度 r̈ + ω̇ × d + ω × (ω × d)
2. tray 角加速度 θ̈ / 角速度平方 θ̇²
3. 重力恢复（已经包含在模态刚度里）
```

Scout 杯子放托盘上有水平偏置 `d = [d_x, d_y]^T`（典型十几 cm），ω̇·d 与 ω²·d **不可忽略**。

#### 改法

在 `cost_function.h` 同级的 `slosh_integration.h`（或新建 `slosh_excitation.h`）里加入 yaw-offset 修正公式：

```text
设容器中心在底盘坐标系下的偏置 d_x, d_y（参数化）

底盘 yaw 旋转产生的容器加速度（在容器本体坐标系下）：
  a_yaw_x = -omega² · d_x - alpha_z · d_y        # 径向离心 + 切向角加速度
  a_yaw_y = -omega² · d_y + alpha_z · d_x

总激励：
  ax_excit = ax_path + a_yaw_x
  ay_excit = ay_path + a_yaw_y      其中 ay_path = v * omega（路径切向曲率离心）
```

注意：a_yaw_x 与 a_yaw_y 是 `[d_x, d_y]` 与 `omega/alpha_z` 的双线性函数；ω² 项是径向，ω̇ 项是切向，符号按右手系。Scout 杯子如果**完全在 yaw 轴上方**（d_x = d_y = 0），新增项退化为 0，与当前公式数值一致 → 完全向后兼容。

#### 参数化

`mpc_params.yaml` / `mpc_params_sim.yaml` 加：

```yaml
slosh_estimator:
  container_offset_x: 0.0      # 容器距 yaw 轴的水平偏置 x [m]
  container_offset_y: 0.0      # 容器距 yaw 轴的水平偏置 y [m]
  yaw_offset_excitation_enable: false   # 默认关闭，保持向后兼容
```

`slosh_experiment.launch` / `_sim.launch` 加对应 arg。**默认 disabled**，启用前必须给具体偏置数值。

#### 实物标定

启用前需要量一次 Scout 杯子的实际偏置：

```text
1. 用卷尺量容器中心到底盘 base_link yaw 轴的水平距离（不是 base_footprint，
   是实物 base_link 的 yaw 旋转轴心）；
2. 区分 x（前后向）与 y（左右向）；
3. 数值写进 yaml + launch 默认值。
```

#### 输出

新增（可选）调试 topic：

- `/slosh/excitation_ay_path`（旧 v*omega 项）
- `/slosh/excitation_ay_yaw`（新 -ω²·d_y + α·d_x 项）
- `/slosh/excitation_ax_yaw`（新 -ω²·d_x - α·d_y 项）

便于分析阶段对照"哪种激励主导"。

#### 验证

```text
1. catkin_make scout_local_planner 通过
2. yaw_offset_excitation_enable=false 时:
   /slosh/* 输出与上一版 byte-equal（用 bag replay 比对）
3. yaw_offset_excitation_enable=true 时:
   静态: omega=0, alpha_z=0 → 新增项 = 0
   纯转: v=0, omega=0.5 → a_yaw_x = -0.25 * d_x（应该出现非零 ay/ax）
4. 用 F 组实物 bag replay (observer-only 模式):
   - 旧 observer model_peak vs 新 observer model_peak 应有显著差异
   - 新 observer model_peak 应更接近 RGB peak (gamma_model 下降)
```

#### 风险

- 偏置量错了 → model 估计反而更糟。**必须先量再开**。
- ω² 项在高速转弯时数值可能很大，需要看是否需要 saturation。

---

### P2：terminal residual 治理（强化已有 + horizon 末端延伸）

#### 问题

`terminal_factor_slosh_eta` 和 `terminal_factor_slosh_eta_dot` 已经存在（F 组配置：`terminal_factor_slosh_eta=5.0, terminal_factor_slosh_eta_dot=3.0`），但：

1. 只对 horizon 末端 k=N-1 加权
2. terminal phase 内 ax/jerk 脉冲仍激发液体（报告局限 6.2）
3. 没有"假设运动结束后液体自由衰减"的 residual 部分

Ferrari 式 19f：**η ≤ 0.2 · η_lim, t > t_end**——明确要求运动结束后的残振也要约束。

#### 改法

两段。

**改法 A**（轻量，先做）：terminal_factor 沿 horizon 末几步渐增

当前实现（推测）：只 k=N-1 加 factor。
改成：k ∈ [N-K_term, N-1] 都加权，权重 linear ramp 从 1.0 到 terminal_factor，平滑放大。

新参数：
```yaml
mpc:
  terminal_slosh_ramp_steps: 5    # horizon 末多少步开始 ramp
```

不破现有 terminal_factor 语义；ramp_steps=0 即等价旧行为。

**改法 B**（论文卖点）：horizon 末延伸 + free-decay 预测

在 MPC solve 之后，**post-process**：

```text
1. 取 horizon 末状态 (η_N, η̇_N)；
2. 假设此后 a_x = a_y = 0（free decay）
   用 slosh model 离散矩阵自由 rollout T_decay 秒（例如 1.5 秒）
3. 计算 free-decay 段的 peak η_decay_peak
4. 把 η_decay_peak 作为 soft penalty 加进下一周期 MPC 的 cost
   （通过对 horizon 末端的 terminal_factor 动态调节实现，
    不引入新决策变量）
```

具体说：

```text
auto_adjust_terminal_factor(eta_N_norm, eta_dot_N_norm) → terminal_factor_eta_runtime
  if (predicted free-decay peak > 0.2 * h_lim):
    terminal_factor_eta_runtime *= 2.0  # 临时放大
  else:
    保持配置值
```

这相当于把 Ferrari 的"η ≤ 0.2 · η_lim, t > t_end"软化成一个**自适应权重**，不需要 hard constraint。

新参数：
```yaml
mpc:
  terminal_decay_predict_enable: false   # 默认关闭
  terminal_decay_horizon_s: 1.5
  terminal_decay_h_lim_factor: 0.2       # Ferrari 0.2·η_lim 阈值
```

#### 验证

```text
1. terminal_slosh_ramp_steps=0 + terminal_decay_predict_enable=false:
   行为与上一版完全等价（bag replay byte-equal）
2. 启用后在 F 组 sim s_curve 重跑:
   /mpc/slosh_horizon_summary 末几步 eta 应明显更低
   terminal phase 后的 odom 残振应减弱
3. 实物 d200 同配置对照:
   - terminal_approach_1s 报告 ax_pulse / jerk_pulse 应下降
   - 不影响 reached / no overshoot 等 convergence pass 项
```

#### 风险

- ramp_steps 太大会侵蚀 horizon 早段的跟踪表现 → 实物 ramp_steps=3-5 先试
- free-decay rollout 加 ~10 microsec/cycle，对 30Hz 周期可忽略

---

### P3：ζ_n 物理标定（Ferrari 式 3）

#### 问题

当前 `slosh_integration` 用 ζ=0.05（拍脑袋经验值）。Ferrari 式 3 给出半经验公式：

```text
ζ_n = 0.92 · sqrt(ν/μ) / sqrt(g R³) 
      · [1 + 0.318/sinh(ξ_{1n} h/R) · (1 + (1−h/R)/cosh(ξ_{1n} h/R))]
```

水的 ν, μ 是已知物性，R, h 是容器参数。给定具体 Scout 杯子，ζ_1 应该有定值。

#### 改法

在 `slosh_integration.cpp` 加 `compute_zeta_from_physics(R, h, nu, mu, mode_n)` 静态函数；在 `mpc_params.yaml` 加：

```yaml
slosh_estimator:
  zeta_source: manual   # "manual"（默认，用 yaml zeta 值）/ "physics"（用 Ferrari 公式算）
  zeta: 0.05             # 仅 zeta_source=manual 生效
  liquid_kinematic_viscosity: 1.0e-6  # 水 @ 20°C, m²/s（仅 physics 生效）
  liquid_density: 1000.0              # kg/m³
```

启用 physics 模式后，启动时打印一次"computed zeta_1 = X.XXX"日志，便于复核。

#### 验证

- yaml 容器参数（R, h）对应你实物杯子，启动后打印的 zeta 应在 0.02-0.10 区间（水类常见范围）
- zeta_source=manual 时行为完全等价旧版
- physics 模式 + F 组 sim/实物对照，看 model_peak 变化（理论上 ζ 算对了，model_peak 与 RGB peak 一致性应提升）

---

### P4：γ_model / γ_opt 分析口径

#### 问题

报告 4.4 节用了 γ_model 概念（Ferrari 式 25），但没给出统一 csv 字段。RA-L reviewer 会期望看到"按 Ferrari 同款指标定义"的对比表。

#### 改法

新增 `scripts/analysis/analyze_ferrari_indices.py`（已落地，与 2026-05-08 旧脚本
`compute_ferrari_indices.py` 并存，新脚本服务 paper batch 用途）：

```text
输入: bag(s) + 配对的 RGB visual CSV (red_liquid_infer_from_bag.py 输出)
输出: 每个 bag 一行 csv，含窗口与口径列:
  bag, rgb_csv,
  tracking_start_s, first_terminal_s, window_end_s,
  window, rgb_height_mode, terminal_exclusion_s, tracking_extension_s, pair_max_gap_s,
  paired_samples,
  rgb_p95_mm, rgb_rms_mm, rgb_peak_mm,
  model_p95_mm, model_rms_mm, model_peak_mm,
  gamma_model_pct, rmse_mm, corr, U_p95_mm, U_max_mm,
  gamma_opt_pct
其中:
  gamma_model = 100 * ∫ |h_model - h_RGB| dt / ∫ |h_model| dt    (Eq.25)
  gamma_opt   = 100 * (RGB_peak_baseline - RGB_peak_opt) / RGB_peak_baseline   (Eq.26)
  U_*         = max(0, RGB_metric - model_metric)  # 模型低估幅度

RGB 高度口径 (--rgb-height-mode):
  max_lcr (默认): max(h_mm_left, h_mm_center, h_mm_right)
                  → 对应 maximum wall-rise，与 Ferrari η_lim 约束同语义，
                  也与 20260524 报告"RGB peak"实际评价口径一致
  smooth_corr   : abs(h_mm_smooth_corr)
                  → 对应 docs/重要文档/红色液体视觉验证固定流程.md §7.1.3 原始主指标
                  → 论文 limitation / supplementary 对照用

评价窗口 (--window):
  main (默认):     TRACKING start → first_terminal − terminal_exclusion_s (默认 1.0 s)
                   → 排除终点前 1 秒（对应 20260524 报告 §3）
  residual:        first_terminal → first_terminal + tracking_extension_s (默认 2.0 s)
                   → Ferrari Eq.25 的 1.25·t_end 残振区间

时间配对:
  以 RGB CSV 行为锚点 nearest-neighbor 到 /slosh/height；
  gap > --pair-max-gap-s (默认 0.15 s) 记为 NaN 丢弃 (流程文档 §8.6)
```

#### 验证

- 在 C/D/F 包上跑一次，输出表对照报告 4.4 节趋势一致
- 不动 RGB 真值定义，只多一份指标视图

---

### P5：实物扩样本（C/D/F 各 n=3，加 P1/P2 验证）

仅执行类，不在本方案展开。但需要：

```text
1. C/D/F 各跑 3 包，扩到 n=3（解决报告局限 6.1）
2. 在第 2/3 包启用 P1 (yaw_offset_excitation_enable=true) + P3 (zeta_source=physics)
   验证 P1/P3 是否进一步降低 RGB peak 或改善 gamma_model
3. F 组额外跑 1 包加 P2 改法 B（terminal_decay_predict_enable=true）
   验证 terminal residual 治理对 terminal_approach_1s 的影响
4. 全部走 d200 terminal、固定 P2_s_curve、record_slosh_experiment.sh -a
```

---

## 3. 文件级改动清单

| Phase | 新增 | 修改 | 默认行为是否变 |
|---|---|---|---|
| P1 | `include/slosh_excitation.h`（可选）/ `src/slosh_excitation.cpp` | `slosh_integration.h/.cpp`（update 签名扩展或重载） `local_planner_ros.cpp`（updateSloshEstimate 调用新接口） `mpc_params.yaml` / `mpc_params_sim.yaml`（加 3 新 param） `slosh_experiment.launch` / `_sim.launch`（加 3 arg） | 否（enable 默认 false） |
| P2 | — | `local_planner_ros.cpp`（terminal_factor ramp 计算 + 可选 free-decay 预测） `cost_function.cpp`（terminal_factor ramp 支持） `mpc_params.yaml` / `_sim.launch` / `_experiment.launch`（加 3 新 param/arg） | 否（默认 ramp_steps=0, decay_predict=false） |
| P3 | — | `slosh_integration.cpp`（compute_zeta_from_physics 静态函数） `slosh_integration.h`（zeta_source enum） `mpc_params.yaml` / launch（加 4 新 param/arg） | 否（默认 manual + 0.05） |
| P4 | `scripts/analysis/compute_ferrari_indices.py` | — | — |
| P5 | 录包目录 / sidecar | — | — |

**所有改动都是新增 param 默认关闭，向后兼容**。

---

## 4. 验证流程

每个 Phase 提交前必须做：

```text
1. catkin_make --pkg scout_local_planner 通过
2. 新 param 默认关闭时:
   sim S 曲线 smoke 替换不上 bag,/cmd_vel + /slosh/* + /mpc/cost_breakdown
   与上一 commit byte-equal（diff_two_bags.py 同 2026-05-20 重构方案 Phase 0）
3. 新 param 启用后:
   sim 跑一次,各 P 的"启用条件"必须真触发（看 /slosh/excitation_*_yaw 非零、
   terminal_factor ramp 在末几步生效等）
4. P1 / P3 后必须用 F 组实物 bag replay observer-only:
   gamma_model 应下降（模型与 RGB 一致性提升）
5. P2 后必须跑实物 d200 + F 配置对照,terminal_approach_1s 报告
   ax_pulse / jerk_pulse 应下降
任一项不通过 → 立即回退该 P,不进入下一阶段
```

---

## 5. 论文叙事（替代 codex 的 constraint-aware 提法）

codex 提议把方法叙事改成 "constraint-aware slosh MPC"，但你 online QP solver 做不到真正的 hard constraint，会被 reviewer 问 "为什么不直接 hard"。

我推荐的叙事更贴合**实际做了什么**：

```text
We propose an online slosh-priority MPC for non-prehensile transportation on a
mobile differential-drive platform. Different from Ferrari et al. [RA-L 2026]
who solve an offline NLP with hard η ≤ η_lim and post-motion η ≤ 0.2·η_lim
constraints on a prehensile SCARA manipulator, we operate in a real-time setting
where:

(1) modal excitation is corrected to include yaw-offset centripetal (ω²·d) and
    tangential (ω̇·d) coupling, which a pure path-curvature centripetal estimator
    misses on platforms where the container is offset from the yaw axis;

(2) damping ratio ζ_n is computed from first-principles using container geometry
    and liquid properties, removing one degree of empirical tuning;

(3) the terminal residual is treated by an adaptive horizon-end penalty that
    softens Ferrari's hard η ≤ 0.2·η_lim post-motion constraint, using
    free-decay prediction beyond the MPC horizon to anticipate residual peak.

We achieve 30% RGB peak reduction at 27% time cost on a Scout Mini platform,
with the model-vs-visual fidelity index γ_model = ... %, comparable to Ferrari
et al.'s prehensile setting.
```

**差异化贡献**:

| 维度 | Ferrari | 本工作 |
|---|---|---|
| 场景 | prehensile（夹持） | non-prehensile（杯子开口放托盘） |
| 求解 | offline NLP（85-471s） | online MPC（30Hz） |
| 平台 | 6-DOF SCARA 工业臂 | 移动差速底盘 |
| η_lim | hard constraint | 软代价（无法 hard 但用 free-decay 自适应权重） |
| 激励建模 | 直接给 EE 加速度 | **ω̇·d / ω²·d 偏置耦合显式补偿**（P1，own contribution） |
| 阻尼 | Ferrari 式 3（论文里只对内侧标定） | 同样用 Ferrari 式 3（P3，引用） |

---

## 6. 不在本方案的事

- 不动 MPC 控制律（state space / Q matrix 结构不变）
- 不引入新决策变量（不破 OSQP 形式）
- 不重写 controlLoop（重构走 2026-05-20 那份方案）
- 不动 RGB 视觉验证流程（`docs/重要文档/红色液体视觉验证固定流程.md` 接口保持）
- 不动 GeoRef / OSCRS / path post-processor
- 不动 terminal_capture_stop_distance / terminal_slowdown_distance 等 gate
- 不做 v_max 反推 from η_lim（GEOREF_CONSTRAINED 已 FAIL，论文剖析 §9.3 已警示）
- 不引入 hard slosh constraint（OSQP 不支持，且 prehensile 假设不成立）

---

## 7. 决策点（待用户确认）

1. **是否同意把 P1（yaw-offset 激励耦合）作为下一步核心**，而不是 codex 的 soft constraint slack 路线？
2. **Scout 实物杯子相对 yaw 轴的偏置 d_x, d_y 量过吗？没量需要先用卷尺量一次。**
3. **执行顺序**：建议 P3（最低风险，先建 ζ 基线）→ P1（核心改动）→ P4（脚本）→ P2（terminal 治理）→ P5（实物验证）。可否？
4. **P2 改法 A vs B**：先做 A（terminal ramp，简单）还是直接做 B（free-decay 自适应权重，论文卖点）？
5. **是否同期推进 2026-05-20 重构方案的 Phase 0-1-2**（cost breakdown 单源 + SloshTrackingCost 抽出）？
   - 推荐：先做。因为 P1 改激励 → cost breakdown 数值会变 → 如果没把 cost breakdown 单源做掉，
     双份实现会更难维护。
6. **样本扩充 n=3 的实物时间安排**（P5）—— RA-L 投稿截止前要留多少时间？

---

## 8. 关联文档

- `docs/重要文档/论文参考总结/Ferrari2026_方法剖析.md` —— §10 M-A 直接对应本方案 P1，§4.2 残振对应 P2，§2.2 ω_n/ζ_n 公式对应 P3，§7 γ_model/γ_opt 对应 P4
- `docs/Claude/修改方案-时间-简介/2026-05-20_scout_local_planner_重构方案.md` —— Phase 1-2 cost breakdown 单源 + SloshTrackingCost 抽出，与本方案 P1/P2 协同
- `docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md` —— 第二阶段 C/D/F 主线，本方案 P5 是它的扩展（n=3 + P1/P2 启用）
- `docs/重要文档/红色液体视觉验证固定流程.md` —— RGB 真值流程保持不动，P4 是其分析侧扩展
- `/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/20260524_SloshPriorityMPC实验结果说明.md` —— F 组 winner 报告，本方案是该报告局限 6.2/6.3 的治理

---

## 9. 一句话总结

```text
保持现有 MPC 结构（F 组已 -31% RGB p95、-30% peak），
不加新决策变量、不破 QP、不动跟踪权重。
把 Ferrari 论文里"物理保真度（ω²·d 偏置耦合 + ζ_n 物理标定）"
和"残振约束（终末自由衰减自适应权重）"两条真正可借鉴的部分搬进现有 observer 与 cost。
论文叙事差异化定位在 non-prehensile + online + 移动底盘 + 偏置耦合修正。
```
