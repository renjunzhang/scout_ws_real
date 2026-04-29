# 2026-04-27 Anti-Slosh 方向修正

更新时间：2026-04-27

本文件是 `2026-04-24_Anti-Slosh_MPC结构改进方案.md` 的方向修正。
04-24 文件保留为历史快照，方向冲突时以本文件为准。

## 0. 触发原因

到 2026-04-27 为止已闭环的实验：

| 阶段 | 机制 | 结果 |
|---|---|---|
| P1：`Q_slosh_eta_dot` | η-only 代价加 η̇ 项 | 失败：相位偏移，η̇_rms 升高 |
| P2：terminal slosh cost | 终端 η/η̇ 放大 | 不稳定：peak 升 / RMS 略降，激励不可比 |
| P3B：曲率前馈 `slosh_speed_cap` | preview κ 限速 | 失败：active_ratio≈0.95，h_rms 不降，task_time +10.7% |
| P3C：`dkappa_only` | preview Δκ 限速 | 失败：与 P3B 同批归档（commit cd1f768） |
| P4：SMOOTH_DOMEGA / SMOOTH_CTRL | 控制率 R_da/R_domega 加重 | 失败：tracking 改善，激励反向恶化 |
| C1：`GOV_AY` | odom `|v·ω|` 反应式削线速度 | 失败/不稳定：ay_p95 可降，但 height/energy/task_time 不满足判据 |

共同特征：**通过间接代价、全局平滑或反应式削峰去间接影响晃动**。
共同失败模式：无法同时压低 `h_rms / h_p95 / modal_energy_rms / eta_dot_rms`，并保持 `task_time` 可接受。

## 1. P4 控制率抑制失败的关键发现

仿真三包（同一固定路径，PATH_MODE=template_goal，同起点同 yaw，主段口径 `mpc_status == TRACKING && terminal/mode == NONE`）：

| 指标（vs NOM） | SMOOTH_DOMEGA | SMOOTH_CTRL |
|---|---:|---:|
| track_dist_p95 | **−21.2%** | **−23.8%** |
| heading_err_p95 | −1.2% | +0.4% |
| odom_ay_abs_p95 | **+37.1%** | **+20.7%** |
| odom_kappa_abs_p95 | **+52.3%** | **+63.4%** |
| odom_wz_rms | +18.9% | +19.6% |

机理：

- R_domega 加重后，planner 不能再用"快进-快出"的短弧过弯。
- 被迫选**更长持续时间、更高瞬时曲率**的弧通过相同弯道。
- v·ω 持续值与峰值同时升高 → ay 直接驱动 η̇ → 模态能量恶化。

衍生结论：

> **预览曲率前馈 cap（P3B/P3C 系列）的物理基础不成立。**
> odom 实测 κ 远大于 reference path κ，前馈永远偏乐观，这与 P3B/P3C 的失败模式完全自洽。

## 2. 修正后的方向：先诊断闭环，不继续堆机制

放弃"间接代价 + 全局平滑"仍然成立，但 04-27 的新增证据说明：

> **仅用当前 odom `|v·ω|` 反应式削线速度，也不能作为主线继续推进。**

`GOV_AY` 的关键结果：

| 条件 | 正向信号 | 失败信号 |
|---|---|---|
| `threshold=0.8` | `odom_ay_abs_p95` 下降约 12.8%，height/energy 小幅下降 | `task_time` 增加约 23.2%，超过 15% 上限 |
| `threshold=1.0` | `odom_ay_abs_p95` 下降约 24.3% | `h_rms / h_peak / eta_dot / modal_energy` 全部恶化，`task_time` 增加约 38.3% |

当前判断：

- `GOV_AY` 证明了实测 odom 激励可以被削掉，但没有证明液体模态能量会随之降低。
- 失败更像是**相位/频率问题**：削峰发生在错误时刻，或者降速改变了激励频率，使系统更接近晃动模态。
- 单独削 `v` 会抬高或延长曲率/角速度作用时间，`odom_kappa_abs_p95` 多次上升，这是当前路线的结构性风险。

因此，下一轮主线不是继续加代码，而是：

1. 用修复后的 `last_control_(OMEGA)` 代码重跑最小矩阵。
2. 用新增 `h_p95` 和 GOV_AY 时序指标确认失败机理。
3. 只有当诊断显示"只是触发时机问题"时，才继续改 governor；否则切换到相位/频率感知方案。

### 候选方向重新排序

| 优先级 | 方向 | 当前状态 |
|---|---|---|
| D0 | 诊断闭环：NOM / FAS_Q5 / GOV_AY 最小矩阵 | 当前唯一主线 |
| D1A | 现有 ZV input shaping（只整形 v_ref） | 已失败，停止作为主线 |
| D1B | 预测域内约束/惩罚未来 `ay=v·ω` 序列或模态能量下降 | 下一候选 |
| D2 | 改进 governor：预测下一周期激励、联合限制 `v` 与 `omega` | 仅当 D0 证明 GOV_AY 是"削晚了"才继续 |
| D3 | MPC 内 ay 软约束 | 后手，风险是双线性近似和 QP 不可行 |
| 停止 | 继续扫 `GOV_AY threshold` | 不再继续 |

## 3. 废弃方向（不再继续尝试）

| 方向 | 实验依据 |
|---|---|
| 单方向调大 `Q_slosh` / `Q_slosh_eta_dot` / `terminal_factor_slosh_*` | P1 / P2 失败 |
| 增大 `R_da` / `R_domega`（控制率全程平滑） | P4 SMOOTH 失败 |
| preview κ / dκ 前馈削顶（P3B/P3C 类） | odom κ ≫ reference κ，物理基础不成立 |
| 继续单独扫描 `GOV_AY threshold` | `threshold=0.8/1.0` 已显示时间代价或能量恶化 |
| 仅以 `h_peak` 作为成功判据 | 多次相位偏移误判 |

代码层面：

- 上述参数入口（launch arg / yaml 参数）**保留**，作为未来对照消融通道。
- 不再开新代码追加这些方向上的二阶变体。

## 4. 新增诊断基建（已落地，2026-04-27）

`scripts/extract_slosh_metrics.py` 扩展：

- **`track_dist` / `track_heading_err`**：用 `/tf` 的 `map→odom + odom→base_footprint` 链算 map 系车体位姿，再对参考路径求贴合误差。
  原因：仿真 bag 里 `/odom.pose` 在 odom 系，参考路径在 map 系，直接相减由 TF 漂移引入偏差。
- **odom 激励层**：`odom_ay_abs_p95 / odom_kappa_abs_p95 / odom_wz_rms`。
- **激励→响应滞后相关性**：`ay→eta_y / abs_ay→height / abs_ay→energy / alpha→eta_x` 的最佳 lag + Pearson corr。
- **命令域平滑**：`cmd_dvx_rms / cmd_dwz_rms`（保留 P4 时期口径，作为对照，不再当主指标）。

后续所有 ablation 必须同时报告：
`h_rms / h_p95 / modal_energy_rms`、`eta_dot_rms`、`odom_ay_abs_p95`、`track_dist_p95`、`task_time`、`solve_success_ratio`。

`odom_ay_abs_p95` 是判断"激励是否真的被削掉"的核心新增项，但不是成功充分条件。
必须同时看 `h_p95 / eta_dot_rms / modal_energy_rms / task_time`。

## 5. 当前计划：D0/D1A 已完成，准备 D1B

按 CLAUDE.md §4 的"目标 + 验证"格式：

### D0 结论

1. **D0.1：重跑最小矩阵** → `NOM / FAS_Q5 / GOV_AY(0.8)`，同路径同起点，每组 ≥3 包。验证：确认 `last_control_(OMEGA)` 修复后，旧失败结论是否仍成立。
2. **D0.2：时序诊断** → 用 `gov_ay_first_to_height_peak_s / gov_ay_first_to_eta_dot_peak_s / ay_to_height_lag_s` 判断 GOV_AY 是否削晚、削早或削在错误相位。
3. **D0.3：执行激励诊断** → 同时检查 `odom_ay_abs_p95 / odom_kappa_abs_p95 / odom_wz_rms / track_dist_p95`，确认削线速度是否造成更高曲率或更长激励时间。
4. **D0.4：Go/No-Go 决策** → 若 `GOV_AY` 仍不满足成功判据，停止路线 C，转向 D1 相位/频率感知方案。

D0 已完成，干净 baseline 口径：

```text
NOM: run03 / run04 / run06 / run07
FAS_Q5: run01 / run02 / run03
GOV_AY: run01 / run02 / run03
```

结论：

- `FAS_Q5`：`h_p95` 有改善，但 `h_rms / modal_energy_norm / ay_p95` 不过线。
- `GOV_AY`：`odom_ay_abs_p95` 明显下降，但 `h_rms / h_p95 / modal_energy_norm` 不过线，`eta_dot` 上升。
- 停止继续扫 `GOV_AY threshold`，停止继续单纯调 `Q_slosh`。

### D1A 现有 ZV input shaping 对照

代码状态：

- `local_planner_ros.cpp` 已有 `input_shaping_enable`。
- 当前实现是 ZV shaper，作用于 `ref_points[i].v_ref`。
- `run_sim_fixed_path_bag.sh` 已支持 `CONDITION=ISR`：
  - `Q_slosh=0`
  - `risk_scheduler_enable=false`
  - `input_shaping_enable=true`
  - `input_shaping_type=zv`
  - 默认使用 slosh 模型参数生成 `omega0 / zeta`

本轮不新增控制器、不改 `cost_function.cpp`。

验证矩阵：

```text
P2_s_curve:
NOM(clean baseline) / ISR
每组至少 3 包，优先 4 包
```

录包命令：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=ISR RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

分析命令：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  --group-summary \
  --csv /tmp/d1_isr_matrix.csv \
  /data/a/slosh_bags/sim/20260427/20260427_P2_s_curve_NOM_run03_145117.bag \
  /data/a/slosh_bags/sim/20260427/20260427_P2_s_curve_NOM_run04_150102.bag \
  /data/a/slosh_bags/sim/20260427/20260427_P2_s_curve_NOM_run06_150853.bag \
  /data/a/slosh_bags/sim/20260427/20260427_P2_s_curve_NOM_run07_151019.bag \
  /data/a/slosh_bags/sim/20260427/*P2_s_curve_ISR*.bag
```

D1A Go/No-Go：

- 若 `ISR` 同时满足 `h_rms / h_p95 / modal_energy_norm` 下降 ≥8%，`eta_dot` 不升，`task_time` 增加 ≤15%，进入实物小样本前的 P3_mixed 复验。
- 若 `ISR` 只改善平滑或 tracking，不改善能量，停止 input shaping 作为主线。
- 若 `ISR` 有部分改善但不足 8%，再考虑调 `input_shaping_omega0_override / zeta_override`，不超过 2 组。

D1A 结果：

```text
ISR 3 FAIL(h_rms,h_p95,energy,eta_dot,ay_p95)
tracking_time_s        +0.6%
h_rms                  +3.9%
h_p95                  +5.5%
modal_energy_norm      +4.9%
eta_dot               +18.4%
odom_ay_abs_p95        -4.7%
odom_kappa_abs_p95    +23.4%
```

判断：

- 现有 ISR 只整形 `v_ref`，没有同步约束 `omega` 或 `ay=v·ω`。
- 它不能作为 anti-slosh 主线继续推进。
- 不继续扫 `omega0/zeta`，除非先把整形对象改成 `ay` 或联合 `v/omega`。

### D1B 下一候选：预测域内处理未来激励/能量

候选不直接上实物，先做仿真小步验证：

1. **D1B-1：未来 ay 序列代价**
   在 MPC 预测域中增加对 `ay_pred = v_ref * omega` 或线性化 `v*omega` 的软代价/变化率代价，目标是压低未来横向激励序列，而不是压当前 odom ay。

2. **D1B-2：模态能量下降代理**
   对预测状态中的 `eta_dot` 与 `omega_n*eta` 构造 proxy energy，要求 horizon 末端或关键窗口能量不高于当前能量。优先软约束或代价，不做硬约束。

3. **D1B-3：联合 v/omega 限幅**
   不是单独削 `v`，而是在参考/控制层同时约束 `v` 与 `omega` 的组合，使 `v*omega` 降低时不把曲率/持续时间推高。

D1B-1 已落地为默认关闭的最小消融项：

- 新参数：`mpc/Q_ay_pred`，默认 `0.0`。
- 代价形式：`J_ay = Q_ay_pred * (v_ref * omega)^2`。
- 作用位置：MPC 预测域控制代价，只改变 `omega` 的二次权重，不新增硬约束。
- 目的：直接压低未来横向激励代理，而不是像 `GOV_AY` 一样等当前 odom ay 变大后再削线速度。
- 风险：它本质上仍是 speed-dependent omega penalty，不是完整模态能量控制；若只降低 ay 但延长激励时间，仍可能失败。
- 默认行为：实物和仿真 YAML / launch 默认均为 `0.0`，不影响 NOM/FAS/GOV/ISR 旧入口。

D1B-1 仿真命令：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=AY_COST RUN_ID=01 \
Q_AY_PRED=0.5 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

建议先录：

```text
AY_COST: Q_AY_PRED=0.5，3 包
若 tracking 正常但抑制不足，再试 Q_AY_PRED=1.0，3 包
若 task_time > +15% 或 odom_kappa_abs_p95 明显升高，停止该方向
```

D1B-1 结果：

```text
AY_COST Q_AY_PRED=0.5，3 包
tracking_time_s        +1.1%
h_rms                  -4.0%
h_p95                  +8.0%
modal_energy_norm      -2.7%
eta_dot               +14.5%
odom_ay_abs_p95        -9.5%
odom_kappa_abs_p95    -11.0%
verdict: FAIL(h_rms,h_p95,energy,eta_dot,ay_p95)
```

判断：

- 该项确实能降低预测/执行侧横向激励幅值，但不能稳定降低 slosh 模态响应。
- `h_p95` 与 `eta_dot` 反升，说明仍存在相位问题：降低某些时刻的 `v*omega` 幅值，不等价于降低能量输入。
- 不继续简单上调 `Q_AY_PRED`，避免重复 P4/GOV_AY 的“激励指标下降但晃动不降”问题。

D1B-1 后的方向修正：

- 继续使用 MPC 预测域，但目标从 `ay` 幅值转向模态能量/相位。
- 优先尝试 terminal/window energy cost：
  - `E_k = eta_dot_x^2 + eta_dot_y^2 + omega_n^2 * (eta_x^2 + eta_y^2)`
  - 对 horizon 后段或 terminal 加权，而不是对所有时刻均匀压 `ay`。
- 若要做更强版本，再考虑 `max(0, E_{k+1}-E_k)` 的软惩罚；这需要更谨慎处理凸性和 QP 结构。

### D1C：后段模态能量代价

已新增默认关闭参数：

```text
mpc/Q_modal_energy
mpc/modal_energy_window_start_ratio
mpc/modal_energy_terminal_factor
```

代价形式：

```text
J_E = Q_modal_energy * (eta_dot_x^2 + eta_dot_y^2
      + omega_n^2 * (eta_x^2 + eta_y^2))
```

作用范围：

- 默认只作用于 horizon 后 30%：`modal_energy_window_start_ratio=0.7`。
- 默认不做末端额外放大：`modal_energy_terminal_factor=1.0`。
- 只加状态二次软代价，不加硬约束，因此不会因为当前 slosh 已经较大而直接 QP infeasible。

第一组仿真命令：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=ENERGY_WIN RUN_ID=01 \
Q_MODAL_ENERGY=1.0 \
MODAL_ENERGY_WINDOW_START_RATIO=0.7 \
MODAL_ENERGY_TERMINAL_FACTOR=1.0 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

建议录 3 包：

```text
ENERGY_WIN Q_MODAL_ENERGY=1.0: run01/run02/run03
```

Go/No-Go：

- 若 `h_p95 / modal_energy_norm / eta_dot` 同时下降，且 `task_time <= +15%`，再试 `Q_MODAL_ENERGY=2.0` 或 `terminal_factor=2.0`。
- 若 `eta_dot` 仍上升，停止该方向，说明单纯状态能量窗口仍不能解决相位输入问题。

D1C Q=1.0 初测：

```text
ENERGY_WIN 3 FAIL(h_rms,h_p95,energy,eta_dot,ay_p95)
tracking_time_s        -1.4%
h_rms                  -0.6%
h_p95                  +0.3%
modal_energy_norm      -0.4%
eta_dot                +2.0%
odom_ay_abs_p95        -2.1%
odom_kappa_abs_p95     -0.7%
```

判断：

- `Q_MODAL_ENERGY=1.0` 太弱，几乎没有改变行为。
- 下一步不是重录同参数，而是做一组强参数响应测试。

强参数响应命令：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=ENERGY_WIN RUN_ID=04 \
Q_MODAL_ENERGY=10.0 \
MODAL_ENERGY_WINDOW_START_RATIO=0.5 \
MODAL_ENERGY_TERMINAL_FACTOR=2.0 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

D1C 强参数结果：

```text
ENERGY_WIN Q=10, window=0.5, terminal_factor=2.0
tracking_time_s       -10.2%
h_rms                 -12.5%
h_p95                  -8.1%
modal_energy_norm     -11.7%
eta_dot                -0.5%
odom_ay_abs_p95        +0.4%
odom_kappa_abs_p95    +61.2%
verdict: FAIL(ay_p95)
```

判断：

- 这是目前第一组同时降低 `h_rms / h_p95 / modal_energy_norm` 的 MPC 结构改动。
- 但曲率显著升高，且部分 run tracking 误差偏大，不能直接作为成功结论。
- D1C 不应放弃，应进入中间参数收敛。

中间参数命令：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=ENERGY_WIN RUN_ID=07 \
Q_MODAL_ENERGY=5.0 \
MODAL_ENERGY_WINDOW_START_RATIO=0.6 \
MODAL_ENERGY_TERMINAL_FACTOR=1.5 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

验收新增约束：

- `h_p95` 和 `modal_energy_norm` 下降 ≥8%。
- `eta_dot` 不升。
- `task_time` 不超过 NOM +15%。
- `odom_kappa_abs_p95` 不超过 NOM +15%，否则判定为通过路径几何/跟踪偏差换来的改善。

D1C 中间参数结果：

```text
ENERGY_WIN Q=5, window=0.6, terminal_factor=1.5
tracking_time_s        -5.3%
h_rms                  -3.6%
h_p95                  -3.8%
modal_energy_norm      -3.7%
eta_dot                -6.1%
odom_ay_abs_p95        +7.0%
odom_kappa_abs_p95    +18.4%
verdict: FAIL(h_rms,h_p95,energy,ay_p95)
```

判断：

- 降幅不足，且 `kappa_p95` 仍超过 +15% 门槛。
- D1C 已证明不是“完全无效”，但纯能量代价会把优化器推向几何/曲率污染。
- 不建议继续只扫 `Q_MODAL_ENERGY`。

下一步方向：

- 先修正分析口径，排除 terminal recovery 后再判断是否需要几何保护。

### D1C strict 口径修正结论

`extract_slosh_metrics.py` 默认口径已改为：

```text
/mpc_status == TRACKING && /terminal/mode == NONE
```

保留旧口径：

```bash
--tracking-with-terminal
```

strict 口径下，强参数组结论变化：

```text
ENERGY_WIN Q=10, window=0.5, terminal_factor=2.0
tracking_time_s        -3.3%
h_rms                 -12.5%
h_p95                  -8.1%
modal_energy_norm     -11.7%
eta_dot                -0.5%
odom_ay_abs_p95        -2.2%
odom_kappa_abs_p95    -19.4%
```

判断：

- 旧口径的 `kappa_p95 +61.2%` 是 terminal/recovery 混入造成的误判。
- D1C 强参数在主 tracking 段内没有曲率污染，反而降低了 odom kappa。
- 当前不应新增几何保护代码。
- 中间参数组太弱，不作为主线。

下一步：

- 保留强参数：

```text
Q_MODAL_ENERGY=10.0
MODAL_ENERGY_WINDOW_START_RATIO=0.5
MODAL_ENERGY_TERMINAL_FACTOR=2.0
```

- P2_s_curve 再补 3 包 strict 复验。
- 若复验仍同向，再录 P3_mixed 3 包。
- 只有 P2 与 P3_mixed 都通过 strict 口径，才进入实物小样本。

P2 复验 run10-run12：

```text
tracking_time_s        -1.9%
h_rms                  -1.4%
h_p95                  -0.9%
modal_energy_norm      -0.8%
eta_dot                +8.8%
odom_ay_abs_p95        -5.3%
odom_kappa_abs_p95     -8.9%
verdict: FAIL(h_rms,h_p95,energy,eta_dot,ay_p95)
```

判断：

- run10-run12 没有复现 run04-run06 强参数效果。
- 结果更像弱参数或参数未正确传入。
- bag 不记录 `Q_modal_energy` 参数，不能直接追溯。

下一步要求：

- 重新录 P2 强参数 3 包。
- 录制前必须确认脚本输出：

```text
Q_modal_energy       = 10.0
modal_energy_window  = start=0.5, terminal_factor=2.0
```

- 同时建议给录包脚本补参数记录能力，避免后续再出现“bag 名是 ENERGY_WIN，但无法确认实际权重”的问题。

run13-run15 复验：

```text
tracking_time_s        +0.4%
h_rms                  -2.5%
h_p95                  -0.6%
modal_energy_norm      -2.4%
eta_dot                -1.4%
odom_ay_abs_p95        -1.7%
odom_kappa_abs_p95     -2.1%
verdict: FAIL(h_rms,h_p95,energy,ay_p95)
```

run04-run06 + run13-run15 合并 6 包：

```text
tracking_time_s        -1.5%
h_rms                  -7.5%
h_p95                  -4.3%
modal_energy_norm      -7.0%
eta_dot                -1.0%
odom_ay_abs_p95        -1.9%
odom_kappa_abs_p95    -10.7%
verdict: FAIL(h_rms,h_p95,energy,ay_p95)
```

当前判断：

- D1C 强参数有正向趋势，但稳定性不足。
- 不能进入 P3_mixed 或实物。
- 继续录包前必须先解决参数可追溯问题。

下一步工程动作：

- 修改 `run_sim_fixed_path_bag.sh`，把关键参数写入 `${BAG_PATH}.txt`。
- 同时发布 `/experiment/config_summary`，让 rosbag 内也能追溯参数。
- 之后重录 P2 最小矩阵：`NOM 3 包 + ENERGY_WIN 强参数 3 包`。

已完成脚本补齐：

- `${BAG_PATH}.txt` 会记录关键实验参数和 git 状态。
- `/experiment/config_summary` 会进入 rosbag。

后续验收新增要求：

- 所有用于最终结论的仿真 bag 必须有同名 `.txt`。
- bag 内必须有 `/experiment/config_summary`。
- 复验时必须先确认 `.txt` 中：

```text
Q_MODAL_ENERGY=10.0
MODAL_ENERGY_WINDOW_START_RATIO=0.5
MODAL_ENERGY_TERMINAL_FACTOR=2.0
```

下一步重录：

```text
P2_s_curve:
NOM 3 包
ENERGY_WIN(Q=10, window=0.5, factor=2.0) 3 包
```

可追溯 P2 最小矩阵结果：

```text
NOM: run16/run17/run18
ENERGY_WIN: run16/run17/run18
ENERGY_WIN 参数确认：
Q_MODAL_ENERGY=10.0
MODAL_ENERGY_WINDOW_START_RATIO=0.5
MODAL_ENERGY_TERMINAL_FACTOR=2.0
```

strict 口径：

```text
ENERGY_WIN 3 FAIL(h_rms,h_p95,energy,eta_dot,ay_p95)
tracking_time_s        +2.5%
h_rms                  +2.3%
h_p95                  +6.7%
modal_energy_norm      +2.3%
eta_dot                +2.8%
odom_ay_abs_p95        +0.4%
odom_kappa_abs_p95     +2.8%
```

最终判断：

- D1C 强参数没有稳定复现，且在可追溯矩阵中变差。
- 停止继续扫 `Q_MODAL_ENERGY / window / terminal_factor`。
- D1C 不能进入 P3_mixed 或实物。

当前 04-27 方向结论：

- 纯 MPC soft-cost 路线没有形成稳定主线：
  - `Q_slosh / Q_slosh_eta_dot`：不稳定；
  - `GOV_AY`：可降 ay，但不稳定降 slosh；
  - `AY_COST`：降 ay/kappa，但 `h_p95/eta_dot` 变差；
  - `ENERGY_WIN`：有偶然正向，但可追溯复验失败。
- 下一步不再继续叠代价项，应转向结构性复盘：预测模型一致性、速度/路径规划层约束、真实液面观测链路。

分析命令：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  --group-summary \
  --csv /tmp/d1b_ay_cost_matrix.csv \
  /data/a/slosh_bags/sim/20260427/20260427_P2_s_curve_NOM_run03_145117.bag \
  /data/a/slosh_bags/sim/20260427/20260427_P2_s_curve_NOM_run04_150102.bag \
  /data/a/slosh_bags/sim/20260427/20260427_P2_s_curve_NOM_run06_150853.bag \
  /data/a/slosh_bags/sim/20260427/20260427_P2_s_curve_NOM_run07_151019.bag \
  /data/a/slosh_bags/sim/20260427/*P2_s_curve_AY_COST*.bag
```

不要做的事（在 D1B 方案明确前）：

- 不重启 P3B/P3C 预览前馈方向。
- 不调 `Q_slosh*` / `terminal_factor*` / `R_da*` 默认值。
- 不继续扫 `GOV_AY threshold`。
- 不新增第二套 governor。
- 不继续扫现有 ISR 的 `omega0/zeta`。

## 6. 成功判据（沿用 04-24 §10 + 新增）

必要前提：

```text
solve_success_ratio >= 0.97
task_time 增加 <= 15%
终点正常到达
```

主成功判据：

```text
h_rms 下降 >= 8%
h_p95 下降 >= 8%
modal_energy_rms 下降 >= 8%
eta_dot_rms 不升高，最好 >= 5%
odom_ay_abs_p95 下降 >= 10%   ← 新增，区别于 P4 失败模式
```

辅助判据：

- `track_dist_p95` 不显著恶化（容许 +20% 以内，本轮预期会用减速换激励削顶）。
- `odom_kappa_abs_p95` 不升高。

失败判据：

- 仅 `track_dist_p95` 改善但 `odom_ay_abs_p95` 上升 → 与 P4 SMOOTH 同型失败，必须停手。
- 仅 `task_time` 大幅增加换来的指标改善（节流而非削顶） → 不视为 anti-slosh 成功。
- `odom_ay_abs_p95` 下降但 `h_p95 / eta_dot_rms / modal_energy_rms` 不降 → 说明削激励峰值不是有效控制目标，必须切换方案。

## 7. 文件职责边界（沿用 04-24 §11，新增条目）

| 文件 | 允许改动 | 不应改动 |
|---|---|---|
| `local_planner_ros.cpp` 中 `slosh_speed_governor` 段 | 保留已落地 ay_trigger；仅在 D0 证明触发时机问题后再小改 | 改 η 触发主分支语义；继续堆新 governor |
| `mpc_params*.yaml` `slosh_speed_governor` 块 | 新增默认关闭子键 | 隐式打开 ay_trigger |
| `slosh_experiment*.launch` | 新增 ay_trigger args | 与 04-24 已有入口冲突命名 |
| `scripts/extract_slosh_metrics.py` | 增加诊断字段和 CSV 输出 | 修改历史指标定义导致前后不可比 |

## 8. 与 04-24 方案的关系

- 04-24 §0 / §1 / §2 / §3 / §11 / §12（已完成机制 + 设计原则 + 文件边界 + 立即停止）**沿用**。
- 04-24 §5 P3A 诊断结论 **已落地并扩展**（见本文件 §4）。
- 04-24 §6 整章（P3B / P3C / MPC 代价层激励项）**作废为主线**，仅保留代码作消融。
- 04-24 §9 推荐顺序 **作废**，以本文件 §5 为准。
- 04-24 §10 成功判据 **沿用并扩展**（见本文件 §6）。

## 9. 成功率判断与切换规则

分两类判断：

| 目标 | 预估成功率 | 判断 |
|---|---:|---|
| D0 诊断闭环：定位 GOV_AY / 代价路线为什么失败 | 85%~90% | 值得做，成本低，能减少盲目实物实验 |
| 沿当前 GOV_AY/调权重路线直接得到稳定 anti-slosh 效果 | 40%~60% | 低于 80%，不应继续作为主线 |
| 转向相位/频率感知方案后得到仿真稳定改善 | 60%~75% | 需要 D0 支撑后再设计 |
| 直接达到实物稳定改善 | <60% | 不能跳过仿真和视觉/模型一致性验证 |

Go/No-Go 规则：

- 如果一个方向的预估成功率低于 80%，不继续扩大代码实现，只允许做低成本诊断。
- 当前 `GOV_AY` 已低于 80%，因此只保留为 D0 诊断对象，不再作为主线继续调参。
- D0 后若证据指向"触发时机错误"，才考虑 D2；若证据指向"频率/相位耦合"，切换 D1。

## 10. 结构性复盘（2026-04-27 晚）

### 10.1 事实：当前失败不是单个权重没调好

截至可追溯矩阵 `NOM16-18` vs `ENERGY_WIN16-18`，以下方向均未形成稳定主线：

| 方向 | 控制入口 | 失败形态 |
|---|---|---|
| `FAS_Q5 / Q_slosh` | 惩罚 `eta_x / eta_y` | 可改变 peak/RMS，但相位和激励不可控 |
| `Q_slosh_eta_dot` | 惩罚 `eta_dot` | `eta_dot` 可能反升 |
| `terminal_factor_slosh_*` | horizon 末端放大 slosh 代价 | 改善不可归因，激励不可比 |
| `SMOOTH_DOMEGA / SMOOTH_CTRL` | 加大 `R_domega / R_da` | tracking 变好但 odom ay/kappa 反升 |
| `GOV_AY` | 当前 odom `|v*omega|` 反应式削速 | ay 可降，但 height/energy/task_time 不稳定 |
| `AY_COST` | 预测域 `Q_ay_pred*(v_ref*omega)^2` | ay/kappa 可降，但 `h_p95 / eta_dot` 变差 |
| `ENERGY_WIN` | horizon 后段模态能量代价 | 非可追溯批次偶然正向，可追溯复验变差 |

结论：

> 现在的问题不是“继续扫一个更合适的 `Q_slosh` / `Q_modal_energy`”，而是当前控制结构没有把液体激励的主导权放在正确层级。

### 10.2 结构原因 1：固定路径 + 高速参考先决定激励，MPC 软代价只能事后折中

当前仿真默认速度链路：

```text
vehicle/v_max = 3.0
v_nominal = 0.8 * v_max = 2.4 m/s
path_handler/max_lat_accel = 2.0 m/s^2
speed_profile_omega_max = 1.5 rad/s
Q_v = 8.0
Q_contour = 45.0
Q_omega_ff = 1.5
R_domega = 4.0
```

这意味着上游 `PathHandler::getReferencePoints()` 已经给 MPC 提供了一个强 tracking / 强速度参考。MPC 内部新增的 slosh 软代价只是在既定路径和速度剖面下重新分配 `v` 与 `omega`。

已观察到的失败模式与此一致：

- 降 `omega` 或平滑 `omega` 时，车辆可能用更长时间或更高局部曲率通过同一段路径；
- 削当前 `v` 时，可能延长激励作用时间，改变激励频率/相位；
- 惩罚 horizon 末端能量时，优化器可能只是把能量峰移动到窗口外或改变相位。

因此，单纯 MPC soft cost 不具备稳定降低液体能量的结构保证。

### 10.3 结构原因 2：预测模型和估计/高度模型不是同一个完整物理对象

预测域 `DiffDriveModel` 中 slosh 输入为：

```text
ax_pred = a
ay_pred = v * omega
```

而在线估计 `updateSloshEstimate()` 可使用：

```text
ay_est = odom v*omega 或 IMU ay
omega_est = odom omega 或 IMU yaw rate
alpha_est = odom 差分或 IMU alpha_z
```

同时 `/slosh/height` 还可能包含 `parabola_term`，但 MPC 预测状态代价只看 `eta / eta_dot`，不直接包含同一高度映射。也就是说：

- MPC 预测的是线性模态状态；
- debug 高度是模型估计高度，不是真实视觉液面；
- 若打开 IMU ay，估计侧激励和预测侧激励会进一步不一致；
- 当前 `Q_modal_energy` 惩罚的是预测状态，不保证实物/视觉高度同步下降。

这不是说模型无用，而是说明它适合作为**诊断和保守约束依据**，不宜继续被当成唯一真值来堆代价。

### 10.4 结构原因 3：P2_s_curve 当前试验太短且 terminal 影响强，容易制造偶然正向

严格口径已经过滤：

```text
mpc_status == TRACKING && terminal/mode == NONE
```

但 P2_s_curve 主段仍然只有约 6~10 s，且路径后段很快进入 terminal recovery / settling。这个条件下：

- 少量起点姿态、定位、速度反馈差异会明显改变相位；
- `h_peak / h_p95` 容易被峰值位置影响；
- whole-bag 口径会被 terminal 污染；
- 3 包以内的偶然正向不能作为实物依据。

因此，后续所有结论必须使用可追溯 bag，并至少保持同条件 3 包；若效果不足 8% 且不可复现，直接停。

### 10.5 当前最合理的下一步：转向参考层/速度剖面层验证

下一步不再新增 MPC 代价项。优先做一个低成本结构验证：

> 如果把 anti-slosh 权限前移到参考速度剖面层，而不是放在 MPC 软代价里，是否能稳定降低 `/slosh/height` 与模态能量？

建议新增一个默认关闭的仿真消融条件：

```text
PROFILE_SAFE
```

初始含义不是最终控制器，而是验证“上游参考层是否有足够控制权限”：

```text
vehicle/v_max: 下调
path_handler/max_lat_accel: 下调
path_handler/speed_profile_omega_max: 下调
path_handler/max_tan_accel / max_tan_decel: 下调或保持可控
```

验证矩阵：

```text
P2_s_curve:
NOM 3 包
PROFILE_SAFE 3 包
ENERGY_WIN 可选 3 包（仅作对照，不继续调参）
```

成功判据仍沿用 §6，但允许 `task_time` 增加接近 15% 上限。若 `PROFILE_SAFE` 仍不能稳定降低能量，说明问题不只在 MPC 层，还需要回到路径几何、模型参数或真实液面观测链。

### 10.6 Go/No-Go

继续做：

- 添加最小 `PROFILE_SAFE` 仿真入口；
- 保持所有新入口默认关闭；
- 只通过 launch/script 覆盖参数，不改实物默认参数；
- 录包必须带 `${BAG_PATH}.txt` 和 `/experiment/config_summary`。

暂时不做：

- 不继续扫 `Q_slosh / Q_ay_pred / Q_modal_energy`；
- 不继续扫 `GOV_AY threshold`；
- 不新增第二套 governor；
- 不把当前 D1C 结果推进实物；
- 不把 `/slosh/height` 当真实液面结论写入最终论文，只能写模型估计指标。

## 11. `PROFILE_SAFE` 验证方案

### 11.1 当前是否已有完整方案

已有方向级方案，见 §10.5：

```text
P2_s_curve:
NOM 3 包
PROFILE_SAFE 3 包
ENERGY_WIN 可选 3 包
```

当前已补齐可直接执行的 `PROFILE_SAFE` 入口：

- `slosh_experiment_sim.launch` 已暴露 `vehicle/v_max`、`path_handler/max_lat_accel`、`path_handler/speed_profile_omega_max`、`path_handler/speed_profile_alpha_max`、`path_handler/max_tan_accel/max_tan_decel`；
- `run_sim_fixed_path_bag.sh` 已支持 `CONDITION=PROFILE_SAFE`；
- `${BAG_PATH}.txt` 和 `/experiment/config_summary` 已记录这些参考层参数。

### 11.2 验证目标

本轮不是证明最终 anti-slosh 控制器，而是回答一个结构问题：

> 把 anti-slosh 权限前移到参考速度剖面层后，是否比继续在 MPC soft cost 层加权重更稳定？

如果 `PROFILE_SAFE` 仍不能稳定降低模型晃动指标，则说明问题不只是 MPC 代价层，需要回到路径几何、模型参数和视觉液面观测链。

### 11.3 需要先补的最小代码入口

已完成。实现原则：只做参数入口，不改实物默认参数，不改 MPC 结构。

```text
slosh_experiment_sim.launch:
  vehicle_v_max
  path_max_lat_accel
  speed_profile_omega_max
  speed_profile_alpha_max
  max_tan_accel
  max_tan_decel

run_sim_fixed_path_bag.sh:
  CONDITION=PROFILE_SAFE
  记录上述参数到 ${BAG_PATH}.txt
  写入 /experiment/config_summary
```

建议第一组 `PROFILE_SAFE` 参数：

```text
vehicle_v_max=2.0
path_max_lat_accel=1.0
speed_profile_omega_max=1.0
speed_profile_alpha_max=3.0
max_tan_accel=1.0
max_tan_decel=1.0
```

选择理由：

- 相比当前 `v_nominal=2.4 m/s`、`max_lat_accel=2.0`，这是明显前移到参考层的减激励；
- 不直接改 `Q_slosh`，避免继续混入 soft-cost 相位问题；
- 预期 task_time 会增加，但应控制在 `+15%` 附近。

### 11.4 录包矩阵

启动仿真后，使用固定终点模板路径口径：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=NOM RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh

PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=NOM RUN_ID=02 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh

PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=NOM RUN_ID=03 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

`PROFILE_SAFE` 录包命令：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=PROFILE_SAFE RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh

PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=PROFILE_SAFE RUN_ID=02 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh

PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=PROFILE_SAFE RUN_ID=03 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

如果 P2 有明显正向，再录 P3_mixed：

```bash
PATH_MODE=template_goal PATH_ID=P3_mixed CONDITION=PROFILE_SAFE RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

### 11.5 分析命令

strict 口径是默认口径：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  --group-summary \
  --csv /tmp/profile_safe_p2_matrix.csv \
  /data/a/slosh_bags/sim/20260427/*P2_s_curve_NOM_run*.bag \
  /data/a/slosh_bags/sim/20260427/*P2_s_curve_PROFILE_SAFE_run*.bag
```

若需要排除旧 NOM，只显式列出新录 3 包，避免混入不同 git 状态：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  --group-summary \
  --csv /tmp/profile_safe_p2_matrix.csv \
  <NOM_run01.bag> <NOM_run02.bag> <NOM_run03.bag> \
  <PROFILE_SAFE_run01.bag> <PROFILE_SAFE_run02.bag> <PROFILE_SAFE_run03.bag>
```

### 11.6 判定标准

必要前提：

```text
solve_success_ratio >= 0.97
终点正常 REACHED
bag 有同名 .txt
bag 内有 /experiment/config_summary
```

主判据：

```text
h_rms 下降 >= 8%
h_p95 下降 >= 8%
modal_energy_norm 下降 >= 8%
eta_dot_rms 不升
task_time 增加 <= 15%
```

辅助判据：

```text
odom_ay_abs_p95 下降
odom_kappa_abs_p95 不升
track_dist_p95 不恶化超过 20%
```

Go/No-Go：

- 若 P2 满足主判据，进入 P3_mixed 复验；
- 若 P2 只靠 `task_time` 大幅增加换来下降，判定为节流伪改善；
- 若 P2 不满足 `h_rms/h_p95/energy` 同时下降，停止 `PROFILE_SAFE` 当前参数，不进实物；
- 若 P2 正向但 task_time 超限，可只微调参考层参数一次，不回到 MPC soft-cost 扫参。

### 11.7 P2 首轮结果：强 `PROFILE_SAFE`

对比口径：

```text
NOM: run16 / run17 / run18
PROFILE_SAFE: run01 / run02 / run03
```

`PROFILE_SAFE` 参数：

```text
vehicle_v_max=2.0
path_max_lat_accel=1.0
speed_profile_omega_max=1.0
speed_profile_alpha_max=3.0
max_tan_accel=1.0
max_tan_decel=1.0
```

strict 口径结果：

```text
tracking_time_s        +38.7%
h_rms                  -40.0%
h_p95                  -42.4%
modal_energy_norm      -38.2%
eta_dot                -17.2%
odom_ay_abs_p95        -42.4%
odom_kappa_abs_p95      +8.0%
verdict: FAIL(time)
```

判断：

- 参考层前移方向成立：`h_rms / h_p95 / energy / eta_dot / ay` 同时大幅下降。
- 但第一组参数过保守，`tracking_time +38.7%`，超过 `+15%` 上限。
- 这不是 MPC soft-cost 失败模式，而是典型节流改善：有效但代价过大。
- 下一步只允许做一次中等强度 `PROFILE_SAFE` 参数复验，不回到 `Q_slosh / Q_modal_energy / GOV_AY` 扫参。

建议第二组中等强度参数：

```text
vehicle_v_max=2.5
path_max_lat_accel=1.4
speed_profile_omega_max=1.2
speed_profile_alpha_max=4.0
max_tan_accel=1.5
max_tan_decel=1.5
```

目标：

```text
task_time 增加 <= 15%
h_rms / h_p95 / energy 仍下降 >= 8%
eta_dot 不升
odom_ay_abs_p95 仍明显下降
```

### 11.8 P2 二轮结果：中等强度 `PROFILE_SAFE`

对比口径：

```text
NOM: run16 / run17 / run18
PROFILE_SAFE: run04 / run05 / run06
```

`PROFILE_SAFE` 参数：

```text
vehicle_v_max=2.5
path_max_lat_accel=1.4
speed_profile_omega_max=1.2
speed_profile_alpha_max=4.0
max_tan_accel=1.5
max_tan_decel=1.5
```

strict 口径结果：

```text
tracking_time_s        +12.7%
h_rms                  -22.5%
h_p95                  -20.8%
modal_energy_norm      -21.7%
eta_dot                -11.9%
odom_ay_abs_p95        -24.2%
odom_kappa_abs_p95     +13.9%
verdict: PASS
```

判断：

- 中等强度 `PROFILE_SAFE` 满足 P2 成功判据。
- 这是 04-27 以来第一组同时满足 `h_rms / h_p95 / energy / eta_dot / ay / task_time` 的结果。
- 辅助风险：`odom_kappa_abs_p95 +13.9%`，仍在当前可接受范围内，但 P3_mixed 必须继续检查。
- 下一步进入 P3_mixed 复验，不进入实物。

P3_mixed 复验命令：

```bash
VEHICLE_V_MAX=2.5 \
PATH_MAX_LAT_ACCEL=1.4 \
SPEED_PROFILE_OMEGA_MAX=1.2 \
SPEED_PROFILE_ALPHA_MAX=4.0 \
MAX_TAN_ACCEL=1.5 \
MAX_TAN_DECEL=1.5 \
PATH_MODE=template_goal PATH_ID=P3_mixed CONDITION=PROFILE_SAFE RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

至少录 `RUN_ID=01/02/03` 三包。若 P3_mixed 也通过，才考虑实物小样本；若 P3_mixed 失败，说明 P2_s_curve 结果不能外推。

### 11.9 P3_mixed 首轮状态：不能判定通过

已录：

```text
PROFILE_SAFE run01 / run02_234407 / run03
额外存在 PROFILE_SAFE run02_234231
```

参数确认：

```text
vehicle_v_max=2.5
path_max_lat_accel=1.4
speed_profile_omega_max=1.2
speed_profile_alpha_max=4.0
max_tan_accel=1.5
max_tan_decel=1.5
```

单组健康检查：

```text
run01: reached，tracking=6.75s，h_rms=2.84mm，h_p95=6.48mm，energy=0.0509，ay_p95=1.326
run02_234407: 未 reached，仅 IDLE/TRACKING，tracking=11.86s，h_max=24.6mm，eta_dot=31.3mm/s
run03: reached，tracking=6.10s，h_rms=2.81mm，h_p95=5.20mm，energy=0.0501，ay_p95=1.357
run02_234231: 未 reached，仅 IDLE/TRACKING，h_max=41.5mm，eta_dot=54.5mm/s
```

判断：

- 当前 P3_mixed 复验不能判定通过。
- 原因 1：缺少同路径、同日、同 git 状态的 `P3_mixed NOM` 基线。
- 原因 2：两个 `run02` 都没有进入 `SETTLING/REACHED`，不是有效重复。
- run01/run03 的单包指标看起来不坏，但不能替代组间对比。

下一步必须补录：

```text
P3_mixed NOM: 3 包
P3_mixed PROFILE_SAFE: 至少补 1 包有效 reached，若不稳定则补满 3 包重做
```

录包要求：

- 必须等到出现 `REACHED` 后再 Ctrl+C；
- 若 30s 仍未 `REACHED`，该包标记为未完成，不进入成功判据；
- 对比时只用有效 `REACHED` 包。

### 11.10 P3_mixed 二轮结果：未通过，P2 结果不能外推

补录基线：

```text
NOM run01 / run02 / run03 / run04 / run05
```

补录 `PROFILE_SAFE`：

```text
PROFILE_SAFE run04 / run05
```

有效对比 1：全量有效包

```text
NOM: run01-run05
PROFILE_SAFE: run01 / run03 / run04 / run05
```

strict 口径结果：

```text
tracking_time_s        +13.2%
h_rms                   -5.8%
h_p95                   +2.5%
modal_energy_norm       -5.6%
eta_dot                 -3.3%
odom_ay_abs_p95         -9.2%
odom_kappa_abs_p95     +43.2%
verdict: FAIL(h_rms,h_p95,energy,ay_p95)
```

有效对比 2：剔除早期 run01 后的稳定子集

```text
NOM: run02-run05
PROFILE_SAFE: run03-run05
```

strict 口径结果：

```text
tracking_time_s        +10.9%
h_rms                   -4.4%
h_p95                   +0.6%
modal_energy_norm       -4.0%
eta_dot                 +0.2%
odom_ay_abs_p95         -7.4%
odom_kappa_abs_p95     +19.8%
verdict: FAIL(h_rms,h_p95,energy,eta_dot,ay_p95)
```

最终判断：

- `PROFILE_SAFE` 在 P2_s_curve 通过，但在 P3_mixed 未通过。
- 当前中等强度参考层限速不能作为可外推方案进入实物。
- 失败主因不是时间超限，而是复杂路径下抑制不足：`h_p95` 不降、energy 降幅不足，且 `odom_kappa_abs_p95` 明显上升。
- 这说明简单全局降 `v_max / max_lat_accel / omega_max` 不是足够的 slosh-aware speed profile；下一步不能直接实物，应转向路径段/曲率段选择性速度剖面。

下一步方向：

```text
从 PROFILE_SAFE（全局保守速度剖面）
转向 PROFILE_SELECTIVE（按曲率段/换向段选择性减激励）
```

设计要求：

- 不能全程节流；
- 只在高曲率、曲率换向、长持续横向激励段降低速度；
- 保持 `task_time <= +15%`；
- P2 和 P3_mixed 必须同时通过后才允许考虑实物小样本。

### 11.11 `PROFILE_SELECTIVE` 实现与验证入口

已实现最小可验证入口：

```text
PathHandler::updateSpeedProfile()
```

新增逻辑：

- 保持默认全局参考层参数不变；
- 在速度剖面采样点满足以下任一条件时，局部叠加更保守速度上限：
  - `|kappa| >= selective_kappa_threshold`
  - `|dkappa/ds| >= selective_dkappa_threshold`
- 触发段可分别应用：
  - `selective_lat_accel`
  - `selective_omega_max`
  - `selective_alpha_max`
- 默认关闭，不影响 `NOM / PROFILE_SAFE / FAS / GOV / ENERGY_WIN`。

默认 `PROFILE_SELECTIVE` 参数：

```text
VEHICLE_V_MAX=3.0
PATH_MAX_LAT_ACCEL=2.0
SPEED_PROFILE_OMEGA_MAX=1.5
SPEED_PROFILE_ALPHA_MAX=5.0
MAX_TAN_ACCEL=2.0
MAX_TAN_DECEL=2.0

SELECTIVE_PROFILE_ENABLE=true
SELECTIVE_KAPPA_THRESHOLD=0.9
SELECTIVE_DKAPPA_THRESHOLD=12.0
SELECTIVE_LAT_ACCEL=1.2
SELECTIVE_OMEGA_MAX=1.2
SELECTIVE_ALPHA_MAX=3.5
```

验证顺序：

```text
1. 先 P2_s_curve：NOM16-18 vs PROFILE_SELECTIVE 3 包
2. 若 P2 通过，再 P3_mixed：NOM run02-05 vs PROFILE_SELECTIVE 3 包
3. P2/P3 都通过后，才讨论实物小样本
```

录包命令：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=PROFILE_SELECTIVE RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

P2 至少录 `RUN_ID=01/02/03`。如果要覆盖默认参数，可显式传：

```bash
SELECTIVE_KAPPA_THRESHOLD=0.9 \
SELECTIVE_DKAPPA_THRESHOLD=12.0 \
SELECTIVE_LAT_ACCEL=1.2 \
SELECTIVE_OMEGA_MAX=1.2 \
SELECTIVE_ALPHA_MAX=3.5 \
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=PROFILE_SELECTIVE RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

编译验证：

```bash
bash -n src/scout_apps/control/scout_local_planner/scripts/run_sim_fixed_path_bag.sh
python3 -m xml.etree.ElementTree src/scout_apps/control/scout_local_planner/launch/slosh_experiment_sim.launch
python3 -m py_compile src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py
git diff --check
catkin_make --pkg scout_local_planner
```

结果：全部通过。

### 11.12 `PROFILE_SELECTIVE` 默认参数 P2 结果

输入 bag：

```text
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_SELECTIVE_run01_185558.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_SELECTIVE_run02_185748.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_SELECTIVE_run03_185936.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_SELECTIVE_run04_190112.bag
```

对照基线：

```text
NOM run16/run17/run18
```

汇总结果：

```text
NOM 3 BASE
tracking_s=5.900
h_rms=3.092 mm
h_p95=6.105 mm
energy=0.054979
eta_dot=14.333 mm/s
ay_p95=2.516
kappa_p95=0.933

PROFILE_SELECTIVE 4 FAIL(h_rms,h_p95,energy,ay_p95)
tracking_s=6.575 (+11.4%)
h_rms=3.014 mm (-2.5%)
h_p95=6.306 mm (+3.3%)
energy=0.053596 (-2.5%)
eta_dot=14.014 mm/s (-2.2%)
ay_p95=2.658 (+5.6%)
kappa_p95=0.870 (-6.7%)
```

判断：

- 默认 `PROFILE_SELECTIVE` 未通过 P2。
- 时间增量 `+11.4%` 仍在可接受范围内，说明还有一定节流余量。
- `kappa_p95` 下降是正向信号，但 `h_p95` 和 `ay_p95` 反升，说明当前阈值/限幅太弱或触发位置没有覆盖主要激励段。
- 不应直接进入 P3_mixed，也不应进入实物。

下一步只做一轮更强选择性参数验证：

```text
SELECTIVE_KAPPA_THRESHOLD=0.65
SELECTIVE_DKAPPA_THRESHOLD=8.0
SELECTIVE_LAT_ACCEL=1.0
SELECTIVE_OMEGA_MAX=1.0
SELECTIVE_ALPHA_MAX=3.0
```

停止条件：

- 若更强参数仍不能在 P2 同时降低 `h_rms / h_p95 / energy / ay_p95`，停止 `PROFILE_SELECTIVE` 路线。
- 若 P2 通过，再录 P3_mixed；P3_mixed 通过后才考虑实物小样本。

### 11.13 `PROFILE_SELECTIVE` 强参数 P2 结果

强参数：

```text
SELECTIVE_KAPPA_THRESHOLD=0.65
SELECTIVE_DKAPPA_THRESHOLD=8.0
SELECTIVE_LAT_ACCEL=1.0
SELECTIVE_OMEGA_MAX=1.0
SELECTIVE_ALPHA_MAX=3.0
```

输入 bag：

```text
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_SELECTIVE_run05_191140.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_SELECTIVE_run06_191341.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_SELECTIVE_run07_191515.bag
```

对照基线：

```text
NOM run16/run17/run18
```

汇总结果：

```text
NOM 3 BASE
tracking_s=5.900
h_rms=3.092 mm
h_p95=6.105 mm
energy=0.054979
eta_dot=14.333 mm/s
ay_p95=2.516
kappa_p95=0.933

PROFILE_SELECTIVE 3 FAIL(ay_p95)
tracking_s=6.667 (+13.0%)
h_rms=2.665 mm (-13.8%)
h_p95=5.474 mm (-10.3%)
energy=0.046949 (-14.6%)
eta_dot=10.363 mm/s (-27.7%)
ay_p95=2.411 (-4.2%)
kappa_p95=0.775 (-16.9%)
```

判断：

- 严格判据仍为 `FAIL(ay_p95)`，因为 `ay_p95` 降幅不足 5%。
- 但与默认 `PROFILE_SELECTIVE` 相比，强参数已经明显压低 `h_rms / h_p95 / energy / eta_dot`，且任务时间 `+13.0%` 仍低于 `+15%` 上限。
- 该结果不是最终通过，但已经足够作为 P3_mixed 泛化候选；如果 P3_mixed 失败，则停止 `PROFILE_SELECTIVE`。

下一步：

```text
用同一强参数录 P3_mixed 3 包。
若 P3_mixed 不能同时压低 h_p95 / energy / eta_dot，停止 PROFILE_SELECTIVE。
```

### 11.14 `PROFILE_SELECTIVE` 强参数 P3_mixed 泛化结果

输入 bag：

```text
/data/a/slosh_bags/sim/20260428/20260428_P3_mixed_PROFILE_SELECTIVE_run01_191925.bag
/data/a/slosh_bags/sim/20260428/20260428_P3_mixed_PROFILE_SELECTIVE_run02_192048.bag
/data/a/slosh_bags/sim/20260428/20260428_P3_mixed_PROFILE_SELECTIVE_run03_192214.bag
```

对照基线：

```text
P3_mixed NOM run02/run03/run04/run05
```

三包配置一致：

```text
SELECTIVE_KAPPA_THRESHOLD=0.65
SELECTIVE_DKAPPA_THRESHOLD=8.0
SELECTIVE_LAT_ACCEL=1.0
SELECTIVE_OMEGA_MAX=1.0
SELECTIVE_ALPHA_MAX=3.0
```

汇总结果：

```text
NOM 4 BASE
tracking_s=6.312
h_rms=2.645 mm
h_p95=4.955 mm
energy=0.047446
eta_dot=13.694 mm/s
ay_p95=1.373
kappa_p95=0.897

PROFILE_SELECTIVE 3 FAIL(h_rms,h_p95,energy,ay_p95)
tracking_s=6.800 (+7.7%)
h_rms=2.511 mm (-5.1%)
h_p95=4.595 mm (-7.3%)
energy=0.044720 (-5.7%)
eta_dot=11.786 mm/s (-13.9%)
ay_p95=1.444 (+5.2%)
kappa_p95=0.919 (+2.4%)
```

判断：

- P3_mixed 未通过严格判据。
- 与 `PROFILE_SAFE` 相比，`PROFILE_SELECTIVE` 的 P3 结果更好：时间代价更小，`h_p95 / energy / eta_dot` 都有下降。
- 但下降幅度不足，且 `ay_p95` 与 `kappa_p95` 反升，说明它仍没有稳定地降低复杂路径下的主要激励源。
- 该路线不应直接进入实物。

结论：

```text
PROFILE_SELECTIVE 是当前最接近有效的参考层方法，但还不是可验收方案。
继续单纯调阈值/限幅的收益可能有限。
下一步应从“路径几何质量 + 速度剖面一致性”联动入手，而不是继续在 MPC soft-cost 或单点 governor 上叠补丁。
```

## 12. 下一阶段：峰值前因果诊断与窗口型速度剖面

### 12.1 为什么不继续小调参

已验证路线：

```text
MPC soft-cost / governor / PROFILE_SAFE / PROFILE_SELECTIVE
```

共同问题：

- P2_s_curve 可以看到局部正向信号；
- P3_mixed 泛化不足；
- 单点阈值限速容易降低部分统计量，但不能稳定压低复杂路径下的 `ay_p95 / kappa_p95`；
- 继续调 `SELECTIVE_KAPPA_THRESHOLD / SELECTIVE_LAT_ACCEL` 可能只是在改变速度相位，不一定命中真正的峰前激励。

因此下一阶段不再直接增加 MPC 代价项，也不继续对 `PROFILE_SELECTIVE` 做小幅扫参。

### 12.2 先做离线因果诊断

目标：

```text
找出 height / eta_dot / modal_energy_norm 峰值前 0.5-1.5s 内，真正升高的是 ay、kappa、omega、domega、v_des 还是局部路径曲率窗口。
```

已新增脚本：

```text
src/scout_apps/control/scout_local_planner/scripts/analyze_slosh_peak_precursors.py
```

脚本口径：

- 默认只统计 `TRACKING && terminal/mode==NONE`；
- 支持 `--peak-signal height|eta_dot|energy`；
- 输出峰值前 `0.5/1.0/1.5s` 窗口中的：
  - `odom_ay_abs`
  - `odom_kappa_abs`
  - `odom_wz_abs`
  - `odom_dwz_abs`
  - `cmd_ay_abs`
  - `cmd_kappa_abs`
  - `cmd_dwz_abs`
  - `v_des_eff`
  - `slosh_ay_abs`
  - `slosh_alpha_abs`

验证命令：

```bash
rosrun scout_local_planner analyze_slosh_peak_precursors.py \
  --peak-signal height \
  --top-k 3 \
  --csv /tmp/peak_precursors_p3.csv \
  /data/a/slosh_bags/sim/20260428/20260428_P3_mixed_PROFILE_SELECTIVE_run01_191925.bag
```

### 12.3 再决定是否实现窗口型速度剖面

只有当诊断显示“峰值前的激励来自连续曲率/曲率变化窗口，而不是单个点超阈值”时，才实现下一版：

```text
curvature-window / preview-energy speed profile
```

候选逻辑：

- 沿路径计算一个窗口风险：

```text
risk_i = max 或 RMS(|kappa|, |dkappa/ds|, expected_ay, expected_domega)
         over [s_i, s_i + preview_distance]
```

- 用窗口风险提前降低 `v_profile[i]`；
- 不再只在当前采样点超阈值时限速；
- 默认关闭，先只做仿真 P2/P3。

停止条件：

- 若峰前诊断不能证明触发错位，不写窗口型速度剖面；
- 若窗口型速度剖面 P2 通过但 P3 失败，不进入实物；
- 若 P2/P3 都通过，再讨论实物小样本。

### 12.4 `PROFILE_WINDOW` 最小实现

已实现最小可验证版本：

```text
PathHandler::updateSpeedProfile()
```

新增参数：

```text
path_handler/selective_preview_distance
```

语义：

- `selective_preview_distance <= 0`：保持旧 `PROFILE_SELECTIVE` 行为，只看当前采样点；
- `selective_preview_distance > 0`：对当前点前方窗口 `[s_i, s_i + preview_distance]` 内的最大 `|kappa| / |dkappa|` 做选择性限速；
- 默认关闭，不影响 NOM、FAS、PROFILE_SELECTIVE 旧结果。

新增实验条件：

```text
CONDITION=PROFILE_WINDOW
```

默认参数：

```text
SELECTIVE_KAPPA_THRESHOLD=0.65
SELECTIVE_DKAPPA_THRESHOLD=8.0
SELECTIVE_LAT_ACCEL=1.0
SELECTIVE_OMEGA_MAX=1.0
SELECTIVE_ALPHA_MAX=3.0
SELECTIVE_PREVIEW_DISTANCE=1.0
```

先录 P2：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=PROFILE_WINDOW RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

同参数录 `RUN_ID=02/03`。P2 通过后才录 P3_mixed。

### 12.5 `PROFILE_WINDOW` P2 结果

输入 bag：

```text
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_WINDOW_run01_200313.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_WINDOW_run02_200435.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_WINDOW_run03_200602.bag
```

配置：

```text
SELECTIVE_KAPPA_THRESHOLD=0.65
SELECTIVE_DKAPPA_THRESHOLD=8.0
SELECTIVE_LAT_ACCEL=1.0
SELECTIVE_OMEGA_MAX=1.0
SELECTIVE_ALPHA_MAX=3.0
SELECTIVE_PREVIEW_DISTANCE=1.0
```

对照基线：

```text
P2_s_curve NOM run16/run17/run18
```

汇总结果：

```text
NOM 3 BASE
tracking_s=5.900
h_rms=3.092 mm
h_p95=6.105 mm
energy=0.054979
eta_dot=14.333 mm/s
ay_p95=2.516
kappa_p95=0.933

PROFILE_WINDOW 3 FAIL(time,h_p95,ay_p95)
tracking_s=8.050 (+36.4%)
h_rms=2.433 mm (-21.3%)
h_p95=5.666 mm (-7.2%)
energy=0.043236 (-21.4%)
eta_dot=11.113 mm/s (-22.5%)
ay_p95=2.358 (-6.3%)
kappa_p95=1.149 (+23.2%)
```

判断：

- `PROFILE_WINDOW` 证明“前视窗口”确实能压低 `h_rms / energy / eta_dot`；
- 但任务时间 `+36.4%` 明显超出验收上限；
- `h_p95` 和 `ay_p95` 降幅不足，且 `kappa_p95` 反升；
- 当前 `preview_distance=1.0m` 版本不能进入 P3，也不能进入实物。

峰值前诊断：

```text
后半段主要 height 峰值前 1s 的 v_des_mean 仍为 2.4。
```

解释：

- 窗口限速主要影响了早期进入段；
- 后续大峰前仍出现高 `ay / wz / domega`，说明窗口风险没有覆盖真正的后半段激励；
- 继续直接增大 preview 或降低阈值会更慢，不能满足任务时间约束。

下一步：

```text
不录 P3_mixed。
先暂停 PROFILE_WINDOW 参数试验，复盘速度剖面的触发依据：
1. 当前只靠 kappa/dkappa 触发，无法稳定覆盖 v*omega 高峰；
2. 后续应考虑以 predicted ay = v_profile * omega_profile 或 expected ay = v^2*kappa 为窗口风险；
3. 若继续实现，必须直接输出/记录窗口风险与限速原因，否则难以解释实验结果。
```

### 12.6 `PROFILE_RISK`：按 expected ay/omega 触发的窗口速度剖面

目的：

```text
替代单纯 kappa/dkappa 触发，把触发依据改成 expected ay = v^2*kappa 与 expected omega = v*kappa。
```

已新增参数：

```text
path_handler/selective_ay_threshold
path_handler/selective_omega_threshold
```

触发逻辑：

- 对 `[s_i, s_i + selective_preview_distance]` 取最大 `|kappa|`；
- 用当前基础速度上限 `base_v` 估算：

```text
expected_ay = base_v^2 * max|kappa|
expected_omega = base_v * max|kappa|
```

- 当 `expected_ay` 或 `expected_omega` 超阈值时触发选择性限速；
- 限速预算仍使用 `selective_lat_accel / selective_omega_max`；
- SpeedProfile 日志输出触发原因计数：

```text
trigger(k/dk/ay/w)=...
```

新增实验条件：

```text
CONDITION=PROFILE_RISK
```

默认参数：

```text
SELECTIVE_KAPPA_THRESHOLD=0.0
SELECTIVE_DKAPPA_THRESHOLD=0.0
SELECTIVE_AY_THRESHOLD=1.6
SELECTIVE_OMEGA_THRESHOLD=0.9
SELECTIVE_LAT_ACCEL=1.4
SELECTIVE_OMEGA_MAX=1.2
SELECTIVE_ALPHA_MAX=0.0
SELECTIVE_PREVIEW_DISTANCE=1.0
```

第一轮只录 P2：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=PROFILE_RISK RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

同参数录 `RUN_ID=02/03`。若 P2 时间仍超过 `+15%` 或 `h_p95/energy` 不降，停止该路线。

### 12.7 `PROFILE_RISK` P2 结果

输入 bag：

```text
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_RISK_run01_202610.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_RISK_run02_202912.bag
/data/a/slosh_bags/sim/20260428/20260428_P2_s_curve_PROFILE_RISK_run03_203041.bag
```

配置：

```text
SELECTIVE_KAPPA_THRESHOLD=0.0
SELECTIVE_DKAPPA_THRESHOLD=0.0
SELECTIVE_AY_THRESHOLD=1.6
SELECTIVE_OMEGA_THRESHOLD=0.9
SELECTIVE_LAT_ACCEL=1.4
SELECTIVE_OMEGA_MAX=1.2
SELECTIVE_ALPHA_MAX=0.0
SELECTIVE_PREVIEW_DISTANCE=1.0
```

汇总结果：

```text
NOM 3 BASE
tracking_s=5.900
h_rms=3.092 mm
h_p95=6.105 mm
energy=0.054979
eta_dot=14.333 mm/s
ay_p95=2.516
kappa_p95=0.933

PROFILE_RISK 3 FAIL(time,ay_p95)
tracking_s=6.983 (+18.4%)
h_rms=2.504 mm (-19.0%)
h_p95=5.576 mm (-8.7%)
energy=0.044356 (-19.3%)
eta_dot=10.831 mm/s (-24.4%)
ay_p95=2.516 (+0.0%)
kappa_p95=1.116 (+19.6%)
```

判断：

- `PROFILE_RISK` 比 `PROFILE_WINDOW` 快，但仍超过 `+15%` 时间上限。
- `h_rms / energy / eta_dot` 有明显下降，说明 expected ay/omega 风险方向有信号。
- `ay_p95` 没有下降，`kappa_p95` 反升，说明实际高横向加速度没有被参考层风险窗口稳定压住。
- 当前版本不能进 P3，也不能进实物。

峰值前诊断：

```text
后半段主要 height 峰值前 1s:
v_des_mean 仍为 2.4
odom_ay_max 约 3.5
wz_max 约 1.2-1.3
```

解释：

- 高 `odom_ay` 很可能不是由参考路径曲率窗口直接预测出来的；
- 更像是跟踪/执行瞬态：实际 `omega` 建立、切弯、局部跟踪误差或控制饱和造成的高横向激励；
- 继续只改参考速度剖面，可能无法控制实际 `odom_ay_p95`。

下一步：

```text
暂停 PROFILE_RISK 参数试验。
转向峰值时刻的 reference-vs-odom 诊断：
1. 在 height 峰值前 1s 输出 reference_path 曲率/期望 v；
2. 同时输出 cmd_vel 与 odom 的 v/omega/ay；
3. 判断高 ay 来自参考几何、MPC 输出还是执行层/跟踪误差。
```

### 12.8 reference-vs-cmd-vs-odom 归因结果

已扩展脚本：

```text
src/scout_apps/control/scout_local_planner/scripts/analyze_slosh_peak_precursors.py
```

新增输出：

```text
ref_kappa_abs
ref_expected_ay_abs = v_des_eff^2 * ref_kappa_abs
ref_expected_omega_abs = v_des_eff * ref_kappa_abs
cmd_ay_abs = cmd_v * cmd_omega
odom_ay_abs = odom_v * odom_omega
track_dist
track_heading_err_abs
```

诊断对象：

```text
P2 NOM run16/run17/run18
P2 PROFILE_RISK run01/run02/run03
P3 NOM run02/run03/run04/run05
P3 PROFILE_SELECTIVE run01/run02/run03
```

关键观察：

- P2 后半段 height 峰值前 1s，经常出现：

```text
ref_expected_ay 不高或中等
cmd_ay 明显高于 odom_ay
track_dist 达到 1.1-1.6 m
v_des_mean 仍为 2.4
```

- P3 后半段 height 峰值前 1s，经常出现：

```text
ref_expected_ay 很高，约 5-10
ref_kappa_abs 很高，约 0.9-1.9
cmd_ay/odom_ay 低于 reference 风险，但液面峰仍出现
```

归因：

```text
P2: 主要问题偏 MPC 输出/跟踪层。高 ay 不是单纯 reference 几何导致，而是在高 v_des 下 MPC 输出较大 omega，且跟踪误差较大。
P3: 主要问题偏路径几何/参考层。reference_path 本身在高 v_des 下给出很高 expected ay，后续再叠加跟踪误差。
```

结论：

- 继续只调 `PROFILE_*` 速度剖面不足以解决 P2 的高 `cmd_ay` 与跟踪误差；
- 继续只调 MPC slosh cost 也不足以解决 P3 的 reference 几何风险；
- 下一步应拆成两条独立修复：
  - P2：先限制 MPC 输出层的实际激励，重点是 `cmd_ay = cmd_v * cmd_omega` 和 `cmd_domega`；
  - P3：先降低路径/参考层几何风险，重点是 reference_path 的 `v_des^2*kappa` 窗口。

优先级：

```text
先做 P2 输出层限激励，因为它不需要新路径，也能直接验证是否压住 cmd_ay/odom_ay。
P3 几何层修复放在 P2 输出层归因确认之后。
```

### 12.9 `OUTPUT_GUARD` 输出层限激励

目的：

```text
直接限制最终发布的 cmd_vel，验证 P2 高 ay 是否来自 MPC 输出/跟踪层。
```

实现位置：

```text
LocalPlannerROS::publishCmdVel()
```

新增参数：

```text
output_ay_guard/enable
output_ay_guard/ay_limit
output_ay_guard/domega_limit
output_ay_guard/omega_eps
```

行为：

- 默认关闭，不影响已有实验；
- 仅在 `TRACKING` 状态生效；
- 只改最终发布的 `omega`，不改 `v`；
- 先约束：

```text
|cmd_v * cmd_omega| <= ay_limit
```

- 再约束：

```text
|d cmd_omega / dt| <= domega_limit
```

- 发布后把 `last_control_.omega` 同步为实际发布的 `omega`，避免下一周期 MPC 的 `u_prev` 和真实输出不一致。

新增调试 topic：

```text
/slosh/output_guard_active
/slosh/output_guard_ay_limit
```

新增实验条件：

```text
CONDITION=OUTPUT_GUARD
```

默认参数：

```text
OUTPUT_AY_GUARD_LIMIT=2.4
OUTPUT_AY_GUARD_DOMEGA_LIMIT=6.0
OUTPUT_AY_GUARD_OMEGA_EPS=0.05
```

第一轮仍只跑 P2：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=OUTPUT_GUARD RUN_ID=01 \
START_DELAY=30 APPROACH_START_ENABLE=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

同参数录 `RUN_ID=02/03`。验收重点不是只看 `h_rms`，而是先确认：

```text
cmd_ay_p95 是否下降
odom_ay_p95 是否下降
tracking_time 是否不超过 +15%
track_dist 是否不明显恶化
```
