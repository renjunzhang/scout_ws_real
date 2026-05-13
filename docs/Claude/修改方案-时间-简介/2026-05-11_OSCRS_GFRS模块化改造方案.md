# 2026-05-11 OSCRS GFRS 模块化改造方案

## 0. 目标与边界

本文档只做实施方案，不修改运行代码。目标是参考 `src/mpc_planner` 的模块边界，把当前 OSCRS 从 `G + monolithic S` 改成：

```text
G -> F -> R -> S -> diagnostics / fallback
```

本轮是 L2.5 模块化，不做 L3 full registry / plugin 架构，不替换 MPC solver，不改 GeoRef / OSCRS 控制行为。默认实现必须保持现有 bag、launch、`candidate_report` 和离线分析脚本兼容。

参考文档：

- `docs/Claude/总结/2026-05-11_OSCRS当前阶段性总结.md`
- `docs/Claude/对话交接文档-时间-简介/2026-05-11_OSCRS改进方案初步研究.md`
- `src/mpc_planner/mpc_planner/src/planner.cpp`
- `src/mpc_planner/mpc_planner_modules/include/mpc_planner_modules/controller_module.h`
- `src/mpc_planner/mpc_planner_types/include/mpc_planner_types/module_data.h`

## 1. 从 src/mpc_planner 借鉴什么

`mpc_planner` 的关键边界不是某个具体 solver，而是调度方式：

```text
Planner
  reset shared ModuleData
  check module data readiness
  module.update(...)
  module.setParameters(...)
  solver/custom module optimize
  module.visualize(...)
```

对 OSCRS 的对应启发：

| mpc_planner 边界 | OSCRS 对应边界 |
|---|---|
| `Planner` orchestration | `oscrs/pipeline.py` 串接 G/F/R/S |
| `ControllerModule` lifecycle | G/F/R/S 各自只做一类决策 |
| `ModuleData` shared bus | `CandidateEval` / `OSCRSContext` 承载中间量 |
| `visualize/saveData` | `diagnostics.py` 统一生成 report/alarm/debug |
| solver 与模块分离 | ROS adapter 与 OSCRS 纯逻辑分离 |

不借鉴的内容：C++ solver 生成、并行局部优化器、full module registry、T-MPC++ 拓扑 guidance planner。这些超出当前 reference-layer post-processor 的改造边界。

## 2. 当前职责归类

当前 `anti_slosh_path_post_processor.py` 内部已经有 G/F/R/S 逻辑，但大多混在 `path_callback()` 和 `evaluate_candidate()` 中。

| 层 | 当前代码位置 | 职责 |
|---|---|---|
| G | `candidate_generators.py::generate_georef_candidates()` | 生成 `original/mild/medium/mid/strong` 候选 |
| F | `evaluate_candidate()`、`check_candidate_collision()` | 几何、碰撞、端点、方向、长度、`ay_ratio_limit` |
| R | `forward_profile()`、`rollout_slosh_metrics()` | 预测速度/加速度剖面与 slosh rollout |
| S | `apply_oscrs_score()`、`select_oscrs_candidate()` | hard gate、score、fixed candidate、fallback |
| diagnostics | `publish_candidate_report()`、`publish_safety_alarm()` | report、fb/takeover、alarm、debug path |
| ROS adapter | `__init__()`、`path_callback()`、`publish_outputs()` | 参数、topic、Path 消息、publisher |

## 3. 目标数据流

```text
/scout/global_path
  -> ROS adapter
  -> G: candidate generation + level cap + optional tail protection
  -> F: geometry / collision / dynamic feasibility
  -> R: slosh rollout + predicted execution summaries
  -> S: hard gate + score + select + fallback decision
  -> diagnostics: candidate_report / safety_alarm / debug paths
  -> /scout/global_path_anti_slosh
```

`anti_slosh_path_post_processor.py` 最终应变成 thin adapter：读取参数、接收 Path、调用 pipeline、发布结果。算法决策应搬到 `scripts/oscrs/` 下的纯 Python 模块，方便无 ROS 单元测试。

## 4. 建议目录

```text
src/scout_apps/control/scout_local_planner/scripts/
  anti_slosh_path_post_processor.py
  candidate_generators.py
  generate_anti_slosh_path_candidates.py
  oscrs/
    __init__.py
    types.py
    path_utils.py
    pipeline.py
    generators/
      __init__.py
      georef.py
      tail_protect.py
    feasibility.py
    slosh_rollout.py
    selector.py
    diagnostics.py
    tests/
      test_candidate_generators_equivalence.py
      test_tail_protect.py
      test_feasibility_filter.py
      test_slosh_rollout_smoke.py
      test_selector_fallback.py
```

`pipeline.py` 对应 `mpc_planner::Planner`，只负责串接，不承载具体算法。`types.py` 定义 `Candidate`、`FeasibilityResult`、`SloshMetrics`、`CandidateEval`、`SelectionResult` 等 dataclass。

## 5. 参数归属

| 参数/逻辑 | 新归属 | 说明 |
|---|---|---|
| `candidate_specs`、`max_candidate_level` | G | 保持候选顺序，cap 后再生成或过滤 |
| `tail_protect_enable`、tail 距离/偏差 | G | 默认关闭，第一版用 replace raw tail |
| `min_segment_length`、length/drift/endpoint/direction/kappa | F | 输出 `reject_stage=geometry` |
| collision costmap threshold | F | 保留当前 point-cost 逻辑，不换 footprint polygon |
| `ay_ratio_limit` | F | 属于执行可行性，不属于 slosh hard gate |
| `prediction_v_max`、`prediction_ay_max_budget`、`a_max` | R | 生成预测执行剖面 |
| `omega_n`、`zeta`、`height_coeff`、`use_parabola_term` | R | 只负责物理 rollout 和指标 |
| `eta_lim_mm`、`residual_ratio` | S | hard gate: `oh/or/os` |
| `fixed_candidate_name` | S | selector override，不放 ROS adapter |
| `fb/takeover/fallback/safety_alarm` | diagnostics / S | S 给选择结果，diagnostics 负责字符串协议 |

## 6. 兼容约束

1. `candidate_generators.py::generate_georef_candidates()` 旧接口不改，旧行为必须逐点等价。
2. `candidate_specs` 顺序不改：`original, mild, medium, mid, strong`。同分 tie-break 继续依赖顺序。
3. `original` 永远保留为 fallback，不因 F/S 失败被删除。
4. `path_to_msg()` 现有首尾 orientation 保留逻辑不改。
5. `candidate_report` 旧字段必须兼容：`summary:selected,geo,oscrs,active,fallback,fb,orig_safe,takeover`，以及 row 内 `sH/sHm/sHp/sHr/sE/sEdot/os/oh/or/ov/osc`。
6. `fb` 语义不改：`-1` 未 active，`0` takeover 成功，`1` only original safe，`2` geometry feasible 但 slosh gate 全失败，`3` 无可用几何候选。
7. 默认参数下首轮重构应是 behavior-preserving；tail protection 和新 diagnostics 字段只能增量启用。

## 7. 分阶段实施

### Commit 1: types + diagnostics contract

新增 `oscrs/types.py`、`oscrs/diagnostics.py`。先把现有 row/report/fb/takeover 结构化，但 `publish_candidate_report()` 输出格式保持兼容。验证重点是旧 bag 解析脚本不坏。

### Commit 2: G wrapper

新增 `oscrs/generators/georef.py`，调用旧 `generate_georef_candidates()`。把 `max_candidate_level` 归入 G wrapper。`tail_protect.py` 只加默认关闭能力。验证重点是 `tail_protect_enable=false` 时候选点列、顺序、数量完全一致。

### Commit 3: F extraction

把 geometry gate、collision gate、`ay_ratio_limit`、reject reason 从 `evaluate_candidate()` 抽到 `oscrs/feasibility.py`。输出 `accepted/reject_stage/reject_reason` 和 geometry metrics。验证重点是每条候选的 accept/reject 与旧实现一致。

### Commit 4: R extraction

把 `forward_profile()`、`rollout_slosh_metrics()` 抽到 `oscrs/slosh_rollout.py`。输出 `SloshMetrics`，同时保留 `sH/sHm/sHp/sHr/sE/sEdot` 字段映射。验证重点是固定 path 上 slosh 指标数值一致。

### Commit 5: S extraction

把 hard gate、batch-normalized OSCRS score、fixed candidate、`select_oscrs_candidate()`、fallback decision 抽到 `oscrs/selector.py`。验证重点是 `selected/geo/oscrs/fb/takeover` 与旧实现一致。

### Commit 6: pipeline + tail protection experiment

新增 `oscrs/pipeline.py` 作为调度器，让 ROS adapter 只负责输入输出。再以独立开关启用 terminal tail protection 实验。验证重点是不开 tail 时全链路等价；开 tail 时必须在 report 中显式记录 `tail_protect=1` 和 tail deviation 指标。

## 8. 验证计划

单元测试：

```bash
python3 -m pytest src/scout_apps/control/scout_local_planner/scripts/oscrs/tests
```

核心测试项：

- `test_candidate_generators_equivalence.py`: 旧 G 接口 exact equality。
- `test_feasibility_filter.py`: reject reason 与 F 层归因。
- `test_slosh_rollout_smoke.py`: `sH/sHr/sE` 数值稳定。
- `test_selector_fallback.py`: `fb=-1/0/1/2/3` 和 takeover 语义。

bag 回归：

```bash
rosbag play <phase4_or_phase5.bag>
python3 scripts/validate_georef_oscrs_bag.py <bag>
python3 scripts/check_oscrs_takeover.py <bag>
```

重构通过条件：

- 默认参数下候选数量、候选顺序、selected、fb、takeover 不变。
- `candidate_report` 旧字段可被现有脚本解析。
- `/scout/global_path_anti_slosh` 首尾 orientation 不回退。
- OSCRS active/shadow 行为不因模块化改变。

## 9. 主要风险与防护

| 风险 | 防护 |
|---|---|
| 候选顺序漂移导致 selector tie-break 改变 | G wrapper 加 exact order test |
| report 字段变化导致离线分析脚本失效 | diagnostics 第一阶段落地并保留旧字段 |
| `original` 被 F/S 误删，fallback 失效 | `original` 永远进入 selection context |
| `ay_ratio_limit` 与 slosh gate 混淆 | F 只管执行可行性，S 只管 slosh hard gate |
| tail protection 改变默认行为 | 默认关闭，单独 commit 和单独 report 字段 |
| 模型保真度不足被架构重构掩盖 | R 层输出预测剖面和 slosh 指标，继续用视觉真值验证 |

## 10. 当前不做

- 不引入 full registry / plugin / ABC 框架。
- 不把 T-MPC++ solver 或 SH-MPC solver 搬入当前工程。
- 不把碰撞检查从 point-cost threshold 改为 footprint polygon。
- 不把 Ferrari 液面公式直接替换到在线 OSCRS。
- 不改变 `eta_lim_mm`、`residual_ratio`、`prediction_v_max` 等实物参数默认值。

## 11. 结论

推荐路线是先做 L2.5：保留当前 OSCRS 行为，把单体 post_processor 中已经存在的 F/R/S 拆成可测试模块。`src/mpc_planner` 的价值在于证明这种边界应该由 orchestrator、module contract、shared data 和 diagnostics 共同维护；OSCRS 不需要复制它的 solver 体系，只需要复制它的职责边界。

## 12. 2026-05-12 修订：L2.5 后续补强计划

### 12.1 当前状态校正

截至 `7e67374` 之后，L2.5 主体已经完成，不再是“等待拆分”的状态：

```text
ROS adapter
  -> G: oscrs/generators/georef.py
  -> F: oscrs/feasibility.py
  -> R: oscrs/slosh_rollout.py
  -> S: oscrs/selector.py
  -> diagnostics: oscrs/diagnostics.py
  -> pipeline: oscrs/pipeline.py
```

因此本文档后续不再作为“大重构计划”，而作为 **L2.5 后续补强计划**。目标从“把 F/R/S 拆出来”调整为：

```text
1. 保持现有行为和 candidate_report 协议兼容；
2. 补齐 tail / terminal 一致性诊断；
3. 增强 reject 归因和单元测试；
4. 继续把 anti_slosh_path_post_processor.py 保持为 thin-ish ROS adapter；
5. 不进入 L3 registry / plugin / 多 generator 抽象。
```

### 12.2 对 `src/mpc_planner` 的准确借鉴口径

`src/mpc_planner` 是 T-MPC++ / mpc_planner 仓库，但当前 OSCRS 不复现它的 solver，也不接入它的 C++ 模块系统。只借鉴三个结构思想：

```text
Planner orchestration  ->  oscrs/pipeline.py
ControllerModule 边界  ->  G / F / R / S 只做单一职责
ModuleData shared bus  ->  Candidate row / CandidateEval 承载中间量
visualize/saveData     ->  diagnostics.py 统一输出 report / alarm / debug
```

论文表述应写成：

```text
OSCRS follows a generate-filter-rollout-select reference-layer architecture inspired by modular MPC planners such as T-MPC++, but does not instantiate multiple MPC solvers or replace the downstream tracking MPC.
```

中文口径：

```text
OSCRS 学 T-MPC++ 的模块边界和多候选选择思想，不学它的求解器实现。
```

### 12.3 后续补强项

#### A. Tail protection，默认关闭

目的：解决实物中“候选路径尾段改变导致终点过冲”的风险，但不改变默认行为。

设计：

```text
G 层：
  tail_protect_enable=false 默认关闭
  tail_protect_mode=replace_raw_tail
  对非 original candidate 的最后 tail_protect_distance 米替换为 raw/base tail

F 层：
  计算 tail_deviation_m
  计算 tail_heading_error_deg
  若 tail_gate_enable=true 且超过阈值，reject reason 使用 tail_dev / tail_heading
```

默认参数：

```yaml
tail_protect_enable: false
tail_gate_enable: false
tail_protect_distance: 0.6
tail_protect_mode: replace_raw_tail
tail_deviation_limit: 0.05
terminal_tail_heading_limit_deg: 10.0
```

验收：

```text
tail_protect_enable=false:
  candidate order / point list / selected / fb / takeover 不变

tail_protect_enable=true:
  candidate_report row 增加 tail / tail_dev / tail_yaw
  终点过冲 smoke 包必须单独验证，不能直接进入正式有效性统计

tail_gate_enable=true:
  tail_dev / tail_yaw 才参与 F_terminal reject
  默认关闭，避免 tail 几何改动和 terminal gate 同时引入导致失败原因耦合
```

#### B. Diagnostics 协议增强

已有旧字段必须保持兼容：

```text
summary:selected, geo, oscrs, active, fallback, fb, orig_safe, takeover
row:sH, sHm, sHp, sHr, sE, sEdot, os, oh, or, ov, osc
```

可以增量新增字段：

```text
tail
tail_dev
tail_yaw
pred_ay_p95
pred_vmax
```

暂不强行引入完整 `reject_stage` 枚举；原因是当前 `reject_reason` 已经被实物脚本消费，贸然重排会破坏现场排障链路。后续若加入 `reject_stage`，必须只作为新增字段，不替代 `reason`。

#### C. 单元测试补齐

优先补这些纯 Python 测试：

```text
test_tail_protect.py:
  replace_raw_tail 后末端点列来自 base
  tail_deviation / tail_heading_error 可计算

test_selector_score.py:
  batch_normalize=false 仍执行 RA-L weighted score
  use_legacy_score=true 才回退 legacy score

test_pipeline_defaults.py:
  tail_protect_enable=false 时 rows 字段存在且默认不 gate
```

#### D. 不做项

当前仍不做：

```text
不接入 src/mpc_planner C++ solver
不做 T-MPC++ 多 local MPC 并行优化
不做 full CandidateGenerator registry
不引入 B-spline / elastic band 第二 generator
不把 max_candidate_level 强行下沉到 G 层并删除 rejected diagnostic rows
不为了提高 takeover 放宽 eta_lim_mm 默认值
```

### 12.4 当前已执行的 L2.5 补强

本轮已开始执行 A/B：

```text
新增 oscrs/generators/tail_protect.py
GeoRef wrapper 支持 tail_protect_enable=false 默认关闭
F 层输出 tail_protect_applied / tail_deviation_m / tail_heading_error_deg
candidate_report row 增加 tail / tail_dev / tail_yaw
launch 和 run_sim_fixed_path_bag.sh 暴露 tail protection 参数
```

下一步建议先跑函数级验证，再跑一包 `tail_protect_enable=false` 的 OSCRS smoke，确认默认行为没有漂移；最后才打开 `tail_protect_enable=true` 做终点过冲 smoke。

### 12.5 P0/P0.5 已落实：fallback 可解释性与 G policy 诊断

按当前优先级，已继续落实两项低风险改动：

#### P0: fallback reason / dominant reason

`candidate_report summary` 在保留旧字段的基础上新增：

```text
fallback_reason
dominant_stage
dominant_reason
```

语义：

```text
fallback_reason:
  NOT_ACTIVE
  OSCRS_TAKEOVER
  ONLY_ORIGINAL_SLOSH_SAFE
  SLOSH_GATE_REJECT_ALL
  FEASIBILITY_REJECT_ALL

dominant_stage:
  NONE
  GENERATION_SKIPPED
  GEOMETRY_REJECT
  COLLISION_REJECT
  DYNAMIC_REJECT
  TERMINAL_REJECT
  SLOSH_REJECT

dominant_reason:
  level:medium>mild
  ay
  collision
  tail_dev
  tail_heading
  peak
  residual
```

目的：看到 `selected=original` 时，可以直接区分是未 active、OSCRS 正常 takeover、只有 original 安全、slosh hard gate 全失败，还是 F 层可行候选全失败。

#### P0.5: `max_candidate_level` 作为 G policy，但保留诊断 row

最终不采用“少生成候选”的实现，因为那会让现场报告里看不到被 cap 的候选。当前实现为：

```text
G 层:
  生成完整 GeoRef candidate rows
  同时输出 generation_policy[name].skipped / reason

F 层:
  如果 generation_policy[name].skipped=true:
    accepted=false
    reject_stage=GENERATION_SKIPPED
    reject_reason=level:<name>><max_candidate_level>
```

这保留了 “`max_candidate_level` 是 G 层 policy” 的语义，同时保留 `candidate_report` 对 `level:medium>mild` 的可见诊断。

新增测试：

```text
scripts/oscrs/tests/test_generation_policy.py
```

验证 `max_candidate_level=mild` 时，`medium` 不会静默消失，而是以 `GENERATION_SKIPPED` row 出现在评估链路中。

### 12.6 tail 几何与 tail gate 解耦

2026-05-13 open 场景 smoke 显示：

```text
tail_protect=true:
  reports=30
  takeover=1
  fallback=27
  fb_counts={'2': 20, '3': 7, '0': 3}
```

问题不是 OSCRS 主通路，而是 tail 支路同时做了两件事：

```text
1. 修改候选尾段几何；
2. 直接把 tail_dev / tail_yaw 作为 F_terminal gate。
```

因此当前实现已拆成两个开关：

```text
tail_protect_enable:
  是否改候选尾段几何，并输出 tail/tail_dev/tail_yaw

tail_gate_enable:
  是否让 tail_dev/tail_yaw 参与 reject
```

默认：

```text
tail_protect_enable=false
tail_gate_enable=false
```

推荐 smoke：

```text
主通路:
  tail_protect_enable=false
  tail_gate_enable=false

tail 诊断支路:
  tail_protect_enable=true
  tail_gate_enable=false

tail gate 实验:
  tail_protect_enable=true
  tail_gate_enable=true
```

新增测试：

```text
scripts/oscrs/tests/test_tail_gate.py
```

验证 `tail_gate_enable=false` 时 tail_yaw 超阈值不会 reject；`tail_gate_enable=true` 时才产生 `TERMINAL_REJECT`。
