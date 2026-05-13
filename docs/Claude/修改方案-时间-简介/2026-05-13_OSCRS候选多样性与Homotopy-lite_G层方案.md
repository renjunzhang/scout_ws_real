# 2026-05-13 OSCRS 候选多样性与 Homotopy-lite G 层方案

## 0. 目标与结论

当前 OSCRS L2.5 的 G 层候选主要来自 GeoRef smoothing：

```text
original / mild / medium / mid / strong
```

这些候选在很多 S 弯或固定路径上属于同一条 reference tube 内的不同平滑强度。它们能改变曲率、曲率变化率和预测横向激励，但未必形成强烈对比。因此用户担心“候选路径差不多，导致 OSCRS 没有真正选择空间”是合理的。

本方案建议：

```text
L2.6:
  Step 0 先做模型保真度闭环，确认 R/S 的排序尺子不是随机的。
  Step 1 增加候选多样性诊断，不改变默认行为。
  Step 2 根据明确 gate 决定是否进入 homotopy-lite / excitation-class G 层候选族。

L3:
  只有在障碍物或走廊确实把自由空间分成不同通道时，
  再引入真正 PRM / homotopy class generator。
```

短结论：

```text
学 de Groot / T-MPC 的“多类候选覆盖 + fallback + decision”思想；
不直接搬完整 T-MPC solver、动态障碍预测和拓扑约束。
```

## 1. 当前问题

当前运行时代码路径：

```text
anti_slosh_path_post_processor.py
  -> oscrs/generators/georef.py
  -> reference_generation/candidate_generators.py
  -> smooth_path(base, iters, gain, drift_limit)
```

现有 G 层的差异来源：

```text
1. smoothing iteration 数；
2. smoothing gain；
3. max drift 限制；
4. 可选 tail_protect。
```

这会带来三个风险：

| 风险 | 表现 | 后果 |
|---|---|---|
| 候选塌缩 | 多条候选几何上几乎重合 | selected=strong 不代表有强选择空间 |
| 激励塌缩 | `ay_p95 / sH / sE / sEdot` 差异很小 | OSCRS score 排序意义弱 |
| 主激励错位 | 只覆盖 `ay/eta_y`，但路径瓶颈是 `ax/eta_x` | P3 这类纵向脉冲场景仍失败 |
| 模型排序不准 | `A_rank` 接近 0.5 | G 层越丰富，R/S 错选成本越高 |
| 论文表述风险 | 把 smoothing 强度说成拓扑多样性 | 容易被质疑不是 homotopy |

因此下一步不应先调大 smoothing 参数或直接实现新 generator，而应先确认两件事：

```text
1. R/S 的模型排序对 RGB 视觉真值是否足够可信；
2. 候选是否在主激励维度上真的不同。
```

## 2. de Groot / T-MPC 可参考点

参考文件：

```text
docs/重要文档/论文参考总结/deGroot2025_方法剖析.md
src/mpc_planner/mpc_planner/src/planner.cpp
src/mpc_planner/mpc_planner/include/mpc_planner/planner.h
src/mpc_planner/mpc_planner_modules/include/mpc_planner_modules/controller_module.h
src/mpc_planner/mpc_planner_types/include/mpc_planner_types/module_data.h
src/mpc_planner/mpc_planner_modules/include/mpc_planner_modules/guidance_constraints.h
src/mpc_planner/mpc_planner_modules/src/guidance_constraints.cpp
src/mpc_planner/mpc_planner_jackal/config/guidance_planner.yaml
```

### 2.1 Planner 调度模式

`src/mpc_planner/mpc_planner/src/planner.cpp` 中 `Planner::solveMPC()` 的关键顺序：

```text
reset ModuleData
check module data readiness
solver warmstart
module.update(...)
module.setParameters(..., k)
module.optimize(...) or solver.solve()
module.visualize(...)
module.saveData(...)
```

OSCRS 可借鉴为：

```text
OSCRSContext / CandidateSet
  -> G modules generate / annotate candidates
  -> F evaluates execution feasibility
  -> R rollout slosh metrics
  -> S hard gate + score + selection
  -> diagnostics report / debug paths / data export
```

当前 `oscrs/pipeline.py` 已经承担类似 Planner orchestrator 的角色。L2.6 不需要重写它，只需要让 G 层输出更丰富的 candidate meta。

### 2.2 ControllerModule 生命周期

`controller_module.h` 的接口边界：

```text
isDataReady()
update()
setParameters()
optimize()
visualize()
saveData()
reset()
```

OSCRS 不需要复制 C++ module 基类，但可以借鉴生命周期拆分：

| T-MPC module lifecycle | OSCRS L2.6 对应 |
|---|---|
| `isDataReady()` | costmap / global path / params 是否足够生成对应候选族 |
| `update()` | G 层生成候选和多样性 meta |
| `setParameters()` | 当前不需要 solver 参数，可对应 F/R/S 参数注入 |
| `optimize()` | 当前不做 MPC solve，对应 `run_pipeline()` 全候选评估 |
| `visualize()` | debug paths + candidate diversity markers |
| `saveData()` | candidate_report + metrics array + 离线 CSV |

### 2.3 ModuleData 共享数据思想

`module_data.h` 中的 `ModuleData` 保存：

```text
static_obstacles
path
path_width_left / path_width_right
path_velocity
current_path_segment
```

OSCRS L2.6 可引入轻量共享上下文，不一定要新建复杂类：

```text
OSCRSContext:
  base_path
  base_metrics
  costmap_snapshot
  corridor_width_left/right   # 可选
  generation_policy
  diversity_summary
```

核心是避免 G/F/R/S 用散乱字段互相猜测，让 diagnostics 能知道候选来自哪个 family。

### 2.4 GuidanceConstraints 的多 planner / fallback 思想

`guidance_constraints.h/cpp` 中可借鉴三点：

```text
1. LocalPlanner 保存每条 guidance 的 result/objective/success/guidance_ID。
2. mapGuidanceTrajectoriesToPlanners() 维持 homotopy class 到 planner 的映射。
3. T-MPC++ 额外加入 non-guided original planner 作为 fallback。
```

OSCRS 对应：

```text
CandidateEval:
  name
  family
  class_id
  accepted
  score
  oscrs_feasible
  selected/fallback reason

original:
  永远保留，不进入 S_full，作为 fallback anchor。
```

这与当前 OSCRS 的 original fallback 原则一致，不需要改。

### 2.5 guidance_planner.yaml 的参数思想

`guidance_planner.yaml` 中值得借鉴的参数结构：

```text
homotopy.n_paths
homotopy.comparison_function
sampling.n_samples
sampling.timeout
goals.longitudinal
goals.vertical
selection_weights.consistency
selection_weights.length
selection_weights.acceleration
```

OSCRS L2.6 对应参数可以是：

```text
diversity:
  enable: true
  min_path_sep_m: 0.08
  min_metric_spread: 0.10

generators:
  georef:
    enable: true
  lateral_offset:
    enable: false
    offsets_m: [-0.12, 0.12]
  corner_profile:
    enable: false
  homotopy_prm:
    enable: false
```

默认必须保持 `georef.enable=true`，其他 generator 默认关闭。

## 3. 不直接迁移的内容

以下内容当前不做：

```text
1. 不引入 P 个 MPC solver 并行优化。
2. 不把 downstream tracking MPC 替换为 T-MPC。
3. 不做动态障碍预测。
4. 不实现完整 Visibility-PRM + homotopy comparison + topology constraint。
5. 不声称 offset/smoothing 候选是严格 homotopy class。
```

原因：

```text
OSCRS 当前是 reference-layer post-processor。
真实 homotopy class 需要障碍物把自由空间切成不可连续变形的通道。
在空旷 S 弯里，左偏/右偏/smoothing 通常仍属同一同伦类。
```

因此论文表述应使用：

```text
diverse reference candidates
excitation-profile classes
corridor-offset candidates
homotopy-inspired candidate generation
```

避免使用：

```text
homotopy class candidates
homotopy globally optimal
topology-constrained optimization
```

除非后续真的实现 PRM / homotopy comparison / topology constraint。

## 4. L2.6 分阶段方案

### Step 0: 模型保真度闭环

目标：在扩展 G 层候选之前，先确认 R/S 的 slosh rollout 排序对 RGB 视觉真值有足够一致性。

理由：

```text
如果 A_rank ≈ 0.5，模型说候选 B 优于候选 A 时，视觉真值只有约一半概率同意。
这等价于用一把接近随机的尺子做 OSCRS 选择。
候选越多样，错选成本越高。
```

输入数据优先使用 phase4 已有视觉结果，不新增实物录包：

```text
phase4 run01:
  slosh_Q0_20260509_203210_RAW_REAL_run01.bag
  slosh_Q0_20260509_203408_GEOREF_FIXED_MILD_REAL_run01.bag
  slosh_Q0_20260509_203540_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_run01.bag

phase4 run02:
  slosh_Q0_20260509_205024_RAW_REAL_run02.bag
  slosh_Q0_20260509_204850_GEOREF_FIXED_MILD_REAL_run02.bag
  slosh_Q0_20260509_204629_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_run02.bag
```

不把 static bag 纳入排序，只可作为视觉/模型零运动 sanity。

视觉数据位置：

```text
docs/Claude/分析数据/phase4_visual_20260509/
/data/a/slosh_bags/real/20260508_phase4/phase4_red_visual_debug_20260510/
```

ablation 组：

| 组 | height coefficient | damping | parabola | 目的 |
|---|---|---|---|---|
| `M0_current_online` | 当前 `/slosh/height` 口径 | 当前 | 当前 | 基线 |
| `M1_ferrari_height_coeff` | Ferrari closed-form / `computeHeightCoeffNonlinear` | 当前 | 当前 | 检查高度系数分支 |
| `M2_parabola_ablation` | Ferrari closed-form | 当前 | on/off 对比 | 检查抛物面项贡献 |
| `M3_ferrari_full` | Ferrari closed-form | Ferrari physics damping | on | Ferrari-style 物理组 |

主指标：

```text
A_rank_p95
A_rank_peak
corr_model_visual
U_p95 / U_max
pair_dt_p95_ms
scale_fit_model_per_visual
```

进入后续 G 层扩展的门槛：

```text
A_rank >= 0.80:
  R/S 排序较可信，可以进入 corner_profile 设计和 smoke。

0.70 <= A_rank < 0.80:
  可进入 diversity diagnostics；
  新 generator 只做 smoke 和诊断，不进入主效果结论。

A_rank < 0.70:
  暂缓 G 层扩展；
  优先重审 R 层模型、阻尼、height_coeff、坐标轴、ax/ay 输入和时间配对。
```

注意：

```text
Step 0 不用 /slosh/height 取代 RGB 主指标；
它只判断在线/离线模型是否足以指导 OSCRS 选择。
```

### Phase A: 候选多样性诊断

目标：先证明候选是否塌缩，不改变路径生成、不改变选择结果。

新增 diagnostics 指标：

| 指标 | 含义 | 用途 |
|---|---|---|
| `div_path_sep_max` | 候选间最大横向/点集分离 | 几何差异 |
| `div_kappa_spread` | `kappa_p95` 的相对 spread | 曲率差异 |
| `div_dkappa_spread` | `dkappa_p95` 的相对 spread | 曲率变化差异 |
| `div_ay_spread` | `predicted_ay_p95` 的相对 spread | 执行激励差异 |
| `div_ax_spread` | `predicted_ax_p95` 的相对 spread | 纵向加减速差异 |
| `div_ax_pulse_spread` | `peak_abs_ax` 或短窗 ax impulse 的 spread | 纵向脉冲覆盖 |
| `div_sH_spread` | `slosh_h_p95` 的相对 spread | 模型侧液面预测差异 |
| `div_sE_spread` | `slosh_energy_rms` 的相对 spread | 模态能量差异 |
| `div_eta_x_spread` | rollout `eta_x` p95 spread | 纵向模态覆盖 |
| `div_eta_y_spread` | rollout `eta_y` p95 spread | 横向模态覆盖 |
| `dominant_excitation` | `eta_x / eta_y / balanced` | 当前路径主激励维度 |
| `diversity_aligned` | 主激励维度 spread 是否过阈值 | 判断候选是否覆盖关键维度 |
| `candidate_collapse` | 多数 spread 低于阈值 | 诊断候选太像 |

相对 spread 建议：

$$
\mathrm{spread}(x)=\frac{\max_i x_i-\min_i x_i}{\max(\epsilon,\max_i x_i)}
$$

candidate_report summary 增加：

```text
div_path=...
div_ay=...
div_ax=...
div_eta_x=...
div_eta_y=...
dominant_excitation=...
diversity_aligned=0/1
div_sH=...
collapse=0/1
```

注意：

```text
这些是诊断字段，不参与 hard gate。
不要用 /slosh/height 作为实物液面主指标；实物主指标仍是 RGB 视觉结果。
```

解释：

```text
collapse=0 只能说明候选之间有差异；
diversity_aligned=1 才说明差异覆盖了当前路径真正主导的激励维度。
例如 P3 若由 ax/eta_x 主导，则 div_ay 很大但 div_ax/div_eta_x 很小仍应视为关键维度塌缩。
```

### Phase B: Homotopy-lite / excitation-class G 层

目标：在不引入完整 PRM 的情况下，让初始候选形成更明显的几何和激励差异。

候选 family：

| 优先级 | family | 示例 name | 主要撬动 | 作用 | 默认 |
|---|---|---|---|---|---|
| 0 | `georef` | `mild/medium/strong` | 基线曲率平滑 | 保留当前 smoothing 基线 | 开 |
| 1 | `corner_profile` | `early_smooth/late_smooth` | `ax / eta_x` 时序 | 改变弯道进入/退出曲率分布，影响 MPC 减速/加速脉冲 | 关 |
| 2 | `offset` | `left_012/right_012` | `ay / eta_y` | 在 reference 法向做左右偏置，制造空间差异 | 关 |
| 3 | `speed_proxy` | `ay_limited_shape` | 跨场景激励预算 | 用预测 ay/ax 预算反推 shape penalty，不改 MPC 速度上限 | 关 |
| 4 | `tail_safe` | `tail_blend_*` | terminal 一致性 | 替代 hard tail replacement，减小尾段接缝 | 关 |

优先级理由：

```text
P3 历史失败更接近纵向 eta_x / ax 脉冲问题；
offset 主要改变横向 ay，对 P2 更友好，但不一定解决 P3；
因此 L2.6 若进入新 generator，先做 corner_profile，再做 offset。
```

其中 `offset` 是最接近 homotopy-lite 的候选，但只能称为 corridor-offset，不称为严格同伦类。

offset 生成规则：

```text
1. 对 base path 计算切向和法向。
2. 在中段施加 lateral offset d(s)，首尾用 taper 回到 0。
3. sanitize + resample。
4. 进入 F_collision / F_dynamic / F_terminal。
5. 如果碰撞或 endpoint/tail 不合格，则 reject，不强行发布。
```

建议 taper：

$$
d(s)=d_0 \sin^2\left(\pi \frac{s-s_0}{s_1-s_0}\right)
$$

这样首尾偏置为 0，减少 endpoint 和 terminal heading 问题。

corner_profile 生成规则：

```text
1. 找到曲率变化集中的弯道窗口。
2. 生成 early_smooth / late_smooth 两类候选：
   - early_smooth: 提前平滑进入弯道，减少入弯前急减速/急转向；
   - late_smooth: 延后释放曲率，改变出弯加速时序。
3. 首尾保持原 endpoint / heading，避免把 terminal 问题混入主效果。
4. F_dynamic 必须报告 predicted_ax / predicted_ay 和 ax pulse。
5. R 层必须输出 eta_x / eta_y 分量，便于判断是否覆盖 P3 主激励。
```

新增 row 字段：

```text
family=georef/offset/corner_profile/tail_safe
class_id=georef:strong / offset:left:0.12 / ...
div_group=...
```

### Phase C: G 层轻量 registry

当前不做 L3 full plugin，但 G 层可以先有轻量列表：

```text
generate_candidates(base, context, params):
  candidates = []
  candidates += georef.generate(...)
  if corner_profile.enable:
    candidates += corner_profile.generate(...)
  if offset.enable:
    candidates += offset.generate(...)
  return candidates, generation_meta
```

建议新增文件：

```text
scripts/oscrs/generators/diversity.py
scripts/oscrs/generators/corner_profile.py
scripts/oscrs/generators/lateral_offset.py
```

不建议现在新增复杂 ABC / plugin registry。用简单函数列表即可。

### Phase D: 真 Homotopy / PRM 预留

只有满足以下条件才进入：

```text
1. costmap 中确实存在障碍物或走廊边界，把自由空间分成不同通道；
2. offset/corner_profile 仍不能提供足够候选差异；
3. 需要论文主张 topology / homotopy coverage；
4. 有足够时间验证 collision、tracking 和视觉液面效果。
```

真 homotopy generator 需要：

```text
1. 从 costmap 提取自由空间；
2. 采样 PRM / visibility graph；
3. DFS / k-shortest 搜索多条 path；
4. 用 winding / obstacle side signature 去重；
5. spline smooth；
6. F/R/S 统一评估；
7. debug visualization 显示不同 class。
```

这属于 L3，不进入当前 L2.6 默认实现。

## 5. 数据结构建议

当前 row dict 已经足够承载 L2.6，不必立刻大重构。

建议增量字段：

```text
row["family"]
row["class_id"]
row["div_path_sep"]
row["div_group"]
row["generation_stage"]
```

generation_meta 增量：

```text
generation_meta = {
  "generation_policy": {...},
  "families": {
    "georef": {"enabled": True, "count": 5},
    "offset": {"enabled": False, "count": 0},
  },
  "diversity": {
    "path_sep_max": ...,
    "ay_spread": ...,
    "ax_spread": ...,
    "ax_pulse_spread": ...,
    "eta_x_spread": ...,
    "eta_y_spread": ...,
    "dominant_excitation": "eta_x",
    "diversity_aligned": False,
    "sH_spread": ...,
    "candidate_collapse": False,
  },
}
```

与 `mpc_planner` 的对应关系：

| `mpc_planner` | OSCRS |
|---|---|
| `ModuleData.path` | `base_path` |
| `ModuleData.path_width_left/right` | `corridor_width_left/right`，可选 |
| `GuidanceTrajectory.topology_class` | `class_id`，L2.6 只是 family class，不等价 homotopy |
| `SolverResult.objective` | `row["oscrs_score"]` |
| `FindBestPlanner()` | `select_oscrs_candidate()` |
| original planner | `original` fallback candidate |

## 6. 参数建议

新增 ROS/YAML 参数默认值：

```yaml
diversity:
  enable: true
  collapse_path_sep_m: 0.03
  collapse_metric_spread: 0.05
  report_only: true

generators:
  georef:
    enable: true
  corner_profile:
    enable: false
    variants: [early_smooth, late_smooth]
    window_mode: curvature_peak
    max_drift_m: 0.15
  offset:
    enable: false
    offsets_m: [-0.10, 0.10]
    taper_start_ratio: 0.15
    taper_end_ratio: 0.85
    max_offset_m: 0.15
  homotopy_prm:
    enable: false
```

默认行为必须保持：

```text
georef only
corner_profile disabled
offset disabled
homotopy_prm disabled
diversity report-only
```

## 7. 验证计划

### 单元测试

新增：

```text
test_model_fidelity_ablation_phase4.py
test_candidate_diversity_metrics.py
test_excitation_alignment_metrics.py
test_corner_profile_generator.py
test_lateral_offset_generator.py
test_generator_family_metadata.py
```

验证项：

```text
1. 默认 georef-only 候选数量和顺序不变。
2. corner_profile / offset disabled 时输出逐点等价旧实现。
3. corner_profile enabled 时 original 不变，首尾 endpoint / heading 不变。
4. offset enabled 时 original 不变，left/right 首尾 endpoint 不变。
5. diversity spread 对构造路径能正确识别 collapse / non-collapse。
6. excitation alignment 能区分 eta_x 主导和 eta_y 主导。
7. candidate_report 旧字段仍可被 validate 脚本解析。
```

### bag smoke

先跑现有通过包，要求行为不变：

```bash
python3 scripts/validate_georef_oscrs_bag.py \
  /data/a/slosh_bags/sim/20260513/20260513_P2_s_curve_GEOREF_OSCRS_ACTIVE_runsmoke_tail_gate03_005046.bag \
  --mode oscrs --require-non-original --require-takeover
```

再跑 corner_profile enabled smoke：

```text
POST_PROCESSOR_CORNER_PROFILE_ENABLE=true
POST_PROCESSOR_CORNER_PROFILE_VARIANTS="early_smooth,late_smooth"
```

再跑 offset enabled smoke：

```text
POST_PROCESSOR_OFFSET_ENABLE=true
POST_PROCESSOR_OFFSET_VALUES="-0.10,0.10"
```

验收不是“必须 takeover 更多”，而是：

```text
1. candidate_report 有 family/class_id/diversity 字段；
2. div_path / div_ax / div_eta_x / div_ay / div_eta_y / div_sH 有可解释差异；
3. fallback=0 或 fallback_reason 可解释；
4. safety_alarm=0 才能进入后续对比；
5. RGB 视觉仍是实物液面主指标。
```

### Phase A -> Phase B 决策 gate

Phase A 完成后，不自动进入新 generator。按以下四类判断：

| 情况 | 观察 | 下一步 |
|---|---|---|
| 1 | GeoRef-only 显著优于 RAW，OSCRS 也显著优于 GeoRef | 候选多样性不是瓶颈，Phase B 可延后；优先模型保真度和正式统计 |
| 2 | OSCRS 与 GeoRef 几乎一样，且关键维度 spread 显示候选塌缩 | Phase B 是关键路径；优先 `corner_profile` |
| 3 | OSCRS 与 GeoRef 几乎一样，但候选不塌缩且 `diversity_aligned=1` | 问题更可能在 R/S，不在 G；先查 rollout 模型保真度和 score |
| 4 | GeoRef 劣于 RAW | 候选生成本身有问题；回几何层调试，不新增 family |

强制条件：

```text
A_rank < 0.70 时，不进入 Phase B 主实现；
最多只允许 report-only 或 offline smoke。
```

## 8. 成功标准

L2.6 成功标准：

```text
1. Step 0 给出 phase4 模型保真度闭环结论，A_rank 口径清楚。
2. 默认配置下行为与当前 L2.5 等价。
3. candidate_report 能显示候选是否塌缩，以及是否覆盖主激励维度。
4. 打开 corner_profile/offset family 后，候选差异指标明显增加。
5. F/R/S 不因新增 G family 改语义。
6. 不改变 eta_lim_mm / prediction_v_max 来制造 takeover。
7. 论文表述不把 homotopy-lite 误称为严格 homotopy class。
```

## 9. 推荐实施顺序

### Step 0: phase4 模型保真度闭环

修改：

```text
优先复用现有视觉 CSV、phase4 bag 和分析脚本；
必要时新增 analysis 脚本做 M0/M1/M2/M3 ablation 汇总。
```

输出：

```text
model_fidelity_ablation_phase4.csv
model_selection_fidelity_phase4.csv
A_rank_by_model_phase4.md
```

验证：

```text
run01/run02 均有 RAW / FIXED_MILD / OSCRS_MEDIUM_ACTIVE 三条件；
pair_dt_p95_ms 合格；
A_rank_p95 明确给出。
```

### Step 1: diversity diagnostics

修改：

```text
oscrs/diagnostics.py
新增 oscrs/diversity.py
validate_georef_oscrs_bag.py 只增打印，不改旧解析
```

验证：

```text
py_compile
test_candidate_diversity_metrics.py
test_excitation_alignment_metrics.py
旧 bag validate PASS
```

### Step 2: corner_profile generator，默认关闭

修改：

```text
oscrs/generators/corner_profile.py
oscrs/generators/georef.py 或新增 generator orchestration helper
anti_slosh_path_post_processor.py 参数读取
launch 参数透传
```

验证：

```text
corner_profile disabled exact behavior
corner_profile enabled candidate_report family 可见
collision / endpoint / tail gate 正常 reject
div_ax / div_eta_x 有可解释变化
```

### Step 3: lateral_offset generator，默认关闭

只在 P2 / eta_y 主导场景确认需要横向空间差异后做。

修改：

```text
oscrs/generators/lateral_offset.py
anti_slosh_path_post_processor.py 参数读取
launch 参数透传
```

验证：

```text
offset disabled exact behavior
offset enabled candidate_report family 可见
div_ay / div_eta_y 有可解释变化
```

### Step 4: report / debug path 增强

修改：

```text
candidate_report row 增加 family/class_id/div/ax/eta_x/eta_y 字段
debug path topic 可按 family 发布
```

验证：

```text
validate 脚本旧字段解析不坏
rviz 能区分 georef/corner_profile/offset 候选
```

## 10. 当前不建议做的事

```text
1. 不为了制造强对比而放宽 max_drift 到很大。
2. 不为了提高 takeover 放宽 eta_lim_mm。
3. 不为了让 rollout 变好而降低 prediction_v_max。
4. 不把 tail_gate 打开后的结果纳入主效果统计。
5. 不把 /slosh/height 当作实物液面主指标。
6. 不在空旷 S 弯里声称实现了 homotopy class coverage。
7. 不在 A_rank 未达门槛时把新 generator 结果写成主结论。
```

## 11. 论文口径建议

可以写：

```text
OSCRS adopts the multi-stage candidate-evaluation-decision structure from
topology-driven MPC planners (de Groot et al., 2024), but replaces
homotopy-class space-topology candidates with reference-shape and
excitation-profile candidates suited to a static-environment,
slosh-suppression context.
```

中文口径：

```text
OSCRS 借鉴 T-MPC++ 的“多候选生成-并行评估-决策选择-fallback”结构。
当前 L2.6 增强的是参考路径族和激励形态族，不声称复现 T-MPC++ 的完整同伦约束优化。
```
