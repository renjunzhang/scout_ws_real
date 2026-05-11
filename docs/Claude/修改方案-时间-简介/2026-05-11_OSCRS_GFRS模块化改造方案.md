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

