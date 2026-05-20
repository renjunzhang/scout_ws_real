# 2026-05-20 scout_local_planner / slosh_models 重构方案（仅审查 + 方案，不动代码）

> 本次任务：只产出重构方案。不改任何代码、不改任何外部行为/topic/launch/参数/实验口径。
> 后续执行需用户明确确认；执行时按分阶段、每步回归。

---

## 0. 目标与边界

### 0.1 目标
- 把 `local_planner_ros.cpp / .h`（132 KB / 2850 行 cpp + 407 行 h）和 `path_handler.cpp`（69 KB / 1730 行）拆成可读、可测、可单独演进的模块。
- 把 MPC tracking、terminal 处理、slosh state/feedback、diagnostics 发布、analysis 工具按职责分离。
- 整理 `scripts/` 顶层 23 个 Python 脚本的分目录归类。
- 全程不改任何外部观测行为。

### 0.2 必须严格保持不变的"外部契约"

| 类别 | 数量 / 范围 | 锁定理由 |
|---|---|---|
| ROS publisher topic 名/类型/语义/发布时机 | **60 个**（cmd_vel / local_path / mpc/* / slosh/* / reference/* / terminal_* / /risk_scheduler/*） | analysis 脚本与录包脚本依赖 |
| ROS subscriber topic | global_path / odom / imu（topic 名 + remap） | 上游路径源、底盘、IMU |
| ROS param 名 + 默认值 | ~70 个（loadParameters 271 行） | launch args 经 param 桥接，所有默认值锁定 |
| Launch args | `slosh_experiment.launch` 与 `slosh_experiment_sim.launch` 各 33+ arg | 实验命令一字不改 |
| MPC 数值行为 | 同输入 → 同 cmd_vel（允许浮点误差 < 1e-6） | 重构不得改变控制律 |
| 状态机迁移图 | `PlannerState`（IDLE/TRACKING/REACHED/SETTLING…）+ `TerminalMode` | terminal 收敛策略不变 |
| RGB visual_height 主指标地位 | 真值口径 | `/slosh/height` 仅作为模型调试量 |

### 0.3 严禁的"伪改善"（本次重构禁止做的）

- 不动 `prediction_v_max` / `v_max` / `Q_*` / `R_*` / 任何阈值
- 不动 `terminal_capture_stop_distance` / `terminal_slowdown_*` 等 gate
- 不引入新参数让"看起来更好"
- 不改 record / analysis 实验口径
- 不在重构里"顺手"修 bug 或加 feature（CLAUDE.md：外科手术式修改）

### 0.4 主线对齐：实物 cost / ax-jerk ablation

**当前主线**：在实物上跑 C / D / E / F 四组消融，验证 MPC cost 结构（Q_slosh / R_a / R_da）
对早段 ax 与 RGB peak 的影响。详见
`docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md`。

```text
C = SMOOTH_SPEED_RELAXED     Q_slosh=0, R_a=默认, R_da=默认
D = SLOSH_PRIORITY            Q_slosh=5, R_a=默认, R_da=默认
E = AX_JERK_PRIORITY          Q_slosh=0, R_a=1.0, R_da=2.0
F = SLOSH_PLUS_AX_JERK        Q_slosh=5, R_a=1.0, R_da=2.0
```

重构必须服务这条主线，**优先级判据**：

| 优先级 | 判据 | 典型工作 |
|---|---|---|
| **A 级（ablation 关键 enabler）** | 该改动直接消除 C/D/E/F 对比中的歧义、提高 cost breakdown 可信度 | cost breakdown 单一来源、slosh cost 抽独立 CostTerm |
| **B 级（ablation 辅助）** | 该改动让 ablation 数据 slicing / observer 不被控制路径 contaminate | terminal 三段独立化、slosh observer 解耦 |
| **C 级（一般代码健康）** | 与 ablation 无直接关系，纯代码组织 | controlLoop 完整切分、成员结构化、publisher 聚合、Python 重组 |

**主线节奏意味着**：

- A + B 级（即 Phase 1–4）即使单独做完，ablation 实验已经可以稳跑且数据可信。
- C 级（Phase 5–11）是长期代码健康投资，可在 RA-L 投稿（2026-08）后再分阶段做。
- 如果时间紧迫，做完 A + B 级就能切回实物 bag 主线。

### 0.4.1 视觉分析链兼容性硬约束（2026-05-20 新增）

参照 `docs/重要文档/红色液体视觉验证固定流程.md`，本次重构必须保证下列 bag 侧
"控制端真值"接口完全不变，否则会破坏 RGB 视觉 + Ferrari 保真度对比的整条分析链。

**bag topic 硬约束**：

| topic | 流程文档引用 | 必须保持 |
|---|---|---|
| `/mpc/cost_breakdown` | 7.1.1 / 7.1.4 | **21 字段 Float32MultiArray** layout 与每个字段数值（J_slosh_eta / J_slosh_eta_dot / pct_* 等） |
| `/terminal/mode` | 7.1.1 | TRACKING → first terminal 边界标记，发布时机 |
| `/terminal/goal_info` | 7.1.1 | 同上 |
| `/slosh/height` | 7.1 / 8.x | 模型侧高度，做 Ferrari 保真度对比的输入 |
| `/odom` (转发) | 7.1.4 | linear.x / angular.z 用于 ax/ay/jerk/v_p95 计算 |
| `/cmd_vel` | 7 | cmd_v_p95 来源 |
| `/slosh/*` 16 个 topic | 8.x | observer 各内部量 |
| `/reference/*` 16 个 topic | — | 参考曲线 |

**分析脚本路径硬约束**（流程文档与 record/run 脚本里硬编码）：

```text
src/scout_apps/control/scout_local_planner/scripts/analysis/analyze_fixed_path_cost_effect.py
src/scout_apps/control/scout_local_planner/scripts/analysis/extract_mpc_cost_breakdown.py
```

这两个脚本（以及 `scripts/analysis/` 子目录下的全部 11 个分析脚本）**绝不允许移动或改名**。
Phase 11 的 Python 重组 **不动 analysis/ 子目录**。

**逐 phase 风险与对策**：

| Phase | 是否触及视觉分析链 | 对策 |
|---|---|---|
| 0 | 否（基础设施） | 必须扩展 `diff_two_bags.py` 覆盖：`/mpc/cost_breakdown` 21 字段逐位 \|Δ\| < 1e-6；`/slosh/height` 时间序列；`/terminal/mode`/`/terminal/goal_info` 序列 |
| **1** | **是** | Phase 0 回归脚本必须用实物 C 组 + D 组 bag 各 replay 一次，前后对比 `/mpc/cost_breakdown` 21 字段 |
| **2** | **是** | 同上；额外验证 `J_slosh_eta` / `J_slosh_eta_dot` 在 terminal_factor 加权下数值一致 |
| **3** | **是** | grep 所有 `terminal_mode_pub_.publish` / `terminal_goal_info_pub_.publish` 调用，确保提取后仍按原时序发布；rostopic record 一段，diff 消息序列 |
| **4** | **是** | observer 抽出后 16 个 `/slosh/*` topic 仍必须从 LocalPlannerROS 经 `feedback_.getOutput()` 走，时序与数值不变；Phase 4 后必须做 rostopic list 全集对比 + 一段实物 bag replay 的 `/slosh/*` 全字段对比 |
| 5–8 / 10 | 否 | 无 |
| **9** | **是** | 60 publisher 聚合时必须保留 advertise queue size / latch 设置；`rostopic list` 在前后对比必须 0 diff |
| **11** | **是** | `scripts/analysis/` 保持原位；顶层 `analyze_*.py` 保留兼容入口；任何被 doc/launch/sh 硬编码引用的脚本绝不移动 |

**phase 通过标准（强化）**：

```text
任一 phase 通过 = catkin_make OK
                + smoke test 能停 goal
                + 同一 sim bag replay diff（cmd_vel + state + 21 字段 breakdown）|Δ| < 1e-6
                + 涉及视觉链 phase 还需用 实物 C + D bag 各 replay 一次，
                  /mpc/cost_breakdown 与 /slosh/height 全序列 |Δ| < 1e-6
                + Phase 3 / 4 / 9 后必须 rostopic list + record diff 验证 topic 完整性
任一项不通过 → 立即回退该 phase，不进入下一阶段
```

### 0.5 解耦目标的具体定义

"去耦合"在本次方案里有四个明确含义：

```text
1. cost 单一来源：solver 用的 cost 与 cost breakdown publish 的 cost
   是同一段代码计算，不允许两边手抄公式。

2. cost term 隔离：Q_slosh / R_a / R_da 应分别落在独立的 CostTerm 子类，
   使得 C/D/E/F 改动只改一处参数，不要触发 StateTrackingCost 内部分支。

3. observer / cost 隔离：slosh 状态估计（IMU+odom→η, η̇）
   与 cost 评估解耦。observer 单独可测、可禁。

4. terminal / tracking 隔离：terminal_capture_stop / recovery / slowdown
   三段必须有独立入口与可观测的 phase tag，
   使 analysis 脚本能干净 slicing 出 "TRACKING start → first terminal" 窗口。
```

---

## 1. 现状审查结论

### 1.1 文件 inventory

**`scout_local_planner/include/scout_local_planner/`（11 头文件）**

| 文件 | 大小 | 评估 |
|---|---|---|
| `local_planner_ros.h` | 16 KB / 407 行 | **god class header**，~50 成员、60 publisher、4 子状态机 |
| `types.h` | 12 KB | 共享 struct 集合，OK |
| `path_handler.h` | 7.5 KB | 接口尚清晰 |
| `slosh_integration.h` | 7.9 KB | OK |
| `constraint_manager.h` | 4.9 KB | OK |
| `cost_function.h` | 5.0 KB | 已有 CostTermBase 组合结构，OK |
| `mpc_solver.h` | 3.9 KB | OK |
| `risk_scheduler.h` | 3.4 KB | OK |
| `cubic_spline.h` | 3.4 KB | 工具，OK |
| `diff_drive_model.h` | 2.0 KB | OK |
| `dynamics_model.h` | 1.9 KB | OK |

**`scout_local_planner/src/`（10 cpp）**

| 文件 | 大小 | 状态 |
|---|---|---|
| **`local_planner_ros.cpp`** | **132 KB / 2850 行** | **重构核心**：controlLoop 810 行 + loadParameters 271 行 + 30+ 私有方法 |
| **`path_handler.cpp`** | **69 KB / 1730 行** | **次核心**：getReferencePoints 342 行 + updateGlobalPath 153 行 + 17 个匿名命名空间 helper |
| `cost_function.cpp` | 15 KB | 结构 OK |
| `mpc_solver.cpp` | 16 KB | 结构 OK |
| `constraint_manager.cpp` | 10 KB | OK |
| `diff_drive_model.cpp` | 7.3 KB | OK |
| `cubic_spline.cpp` | 5.5 KB | OK |
| `risk_scheduler.cpp` | 4.8 KB | OK |
| `slosh_integration.cpp` | 4.4 KB | OK |
| `local_planner_node.cpp` | 0.6 KB | main 入口 |

**`slosh_models/`**：2 cpp + 2 h，已是干净独立库，本次不动。

**`scripts/`**：23 顶层 + 11 analysis/ + 9 oscrs/(已规整) + 4 reference_generation/(已规整)

### 1.2 god class 内部分布

`LocalPlannerROS` 头里成员变量大致 9 个职责分组（位置已按职责分块，但变量未结构化）：

1. ROS 句柄（nh / subs / pubs / tf）
2. 核心组件持有（path_handler_, mpc_solver_, slosh_integration_, risk_scheduler_）
3. 参数（mpc_params_, vehicle_params_, path_params_）
4. tracking feasibility 与 reentry 状态
5. v_des rate limit 状态
6. heading align 状态
7. terminal_recovery + terminal_slowdown + terminal_capture_stop 状态
8. input_shaping 状态
9. settling 状态
10. slosh feedback（odom 差分、IMU 滤波、bias estimation）
11. slosh-aware speed governor 状态
12. risk scheduler 状态
13. 60 个 debug publisher
14. cmd_vel EMA 滤波状态

### 1.3 `controlLoop()` 810 行的内部 phase（从分支识别）

按代码流向大致 10 个 phase（**纯方法提取候选**）：

```text
Phase A  入口 reset：重置 terminal_recovery_latched_ / tracking_feasibility_recovery_active_ /
                     heading_align_active_ 等瞬态 flag
Phase B  非 TRACKING 状态早 return（IDLE / REACHED 路径，仅 publish 调试与 cmd_vel=0）
Phase C  terminal_capture_stop 早 return（distance < threshold → publish 0 + 切断 recovery）
Phase D  terminal_recovery latch + computeTerminalRecoveryCmd + limitTerminalRecoveryCmd
Phase E  risk_scheduler.update + Q_eta → Q_slosh_eta 换算
Phase F  settling 状态参数临时覆盖（Q_v / Q_eta / eta_bar）
Phase G  terminal_slowdown envelope 计算 + 临时 Q_v override
Phase H  v_des_cmd 选择 + v_des_rate_limit_ 平滑
Phase I  tracking_feasibility_guard 状态机 + reentry ramp
Phase J  path_handler.getReferencePoints + mpc_solver.solve + post-process + publishCmdVel
Phase K  publishSloshDebug / publishCostBreakdown / publishReferenceExecutionDebug / publishStatus / publishTerminalDebug 等
```

### 1.4 `path_handler.cpp` 顶部的 17 个匿名命名空间 helper

```text
normalizeAngle / hasUsableOrientation / resamplePath / bsplineSmooth /
sanitizePolyline / wrappedAngleDiff / estimateMaxCurvature(2 个签名) /
estimateMaxCurvatureRate(2 个签名) / estimateNominalSpacing /
removeSinglePointSpikes / repairPrefixWindowGeometry /
estimateDiscreteCurvatureSamples / estimateDiscreteCurvatureRateSamples /
interpolateByArcLength
```

这些是**纯函数 / 无状态**，是 Phase 1 最低风险的提取目标。

### 1.5 ablation 关键的解耦债（2026-05-20 新增审查）

**债 1：cost breakdown 双份实现，互不引用**

```text
cost_function.cpp                        local_planner_ros.cpp
  StateTrackingCost::evaluate()            computeCostBreakdown() [1915-2002]
  StateTrackingCost::getQuadraticCost()    手抄 J_lag/J_contour/J_etheta/J_v
  ControlCost::evaluate/getQuadraticCost   手抄 J_control = R_a * a² + R_omega * omega²
  ControlRateCost::evaluate/getQuadraticCost 手抄 J_smooth = R_da * da² + R_domega * domega²
                                           手抄 J_slosh_eta + terminal_factor 的修正
```

- 后者用于发布到 `mpc/cost_breakdown` topic，被所有 analysis 脚本读
- 前者用于 QP 求解
- **危险**：C/D/E/F ablation 时只要任一处的公式跟另一处不一致，cost breakdown 的解读就是错的
- analysis 脚本（如 `analyze_fixed_path_cost_effect.py`）依据 `pct_slosh_total ≈ 20%` 这种判据，
  如果 breakdown 与实际优化目标不同源，整个判据失效

**债 2：slosh cost 嵌在 `StateTrackingCost` 内**

```text
StateTrackingCost::evaluate()       cost_function.cpp:40-48
  if (params_.Q_slosh_eta > 0.0) { cost += Q_slosh_eta * (eta_x² + eta_y²); }
  if (params_.Q_slosh_eta_dot > 0.0) { cost += Q_slosh_eta_dot * (eta_x_dot² + eta_y_dot²); }

StateTrackingCost::getQuadraticCost  cost_function.cpp:84-93
  if (params_.Q_slosh_eta > 0.0) {
    Q_contrib(ETA_X, ETA_X) = params_.Q_slosh_eta;
    Q_contrib(ETA_Y, ETA_Y) = params_.Q_slosh_eta;
  }
```

- 没有独立的 `SloshTrackingCost : CostTermBase` 子类
- C/E 组（Q_slosh=0）走的是同一份代码、同一份 H 矩阵 sparsity pattern，只是数值为零
- 对 ablation 结论没致命问题（数学上 Q=0 等价于该项不存在），但：
  - cost breakdown 解读时易混淆"slosh cost 是被 zero 掉还是真的解出来很小"
  - 未来想做 "slosh-only 软约束" 类实验时无清晰挂载点
  - 违反 `CostFunction::addCostTerm / removeCostTerm` 的设计意图（这两个接口对 slosh 不起作用）

**债 3：slosh observer 与控制路径强耦合**

```text
LocalPlannerROS::updateSloshEstimate()   local_planner_ros.cpp:1833-1875
  在 controlLoop 内部被调用
  读 odom 差分 + IMU 滤波 → 写入 slosh_integration_ 内部状态
  涉及 LocalPlannerROS 的 ~20 个成员变量（IMU bias / EMA / has_imu 等）
```

- ablation 实验中 observer 的 bias 估计窗口若被 control flow 影响（如 SETTLING 状态延长 reached_debug_duration），
  η 的初值/收敛会被污染
- 当前无法单独单测 observer（无法在 ROS-free 环境下喂 IMU 序列得 η）

**债 4：terminal 三段 (capture_stop / recovery / slowdown) 内联在 controlLoop**

```text
controlLoop 内：
  Phase C  terminal_capture_stop 早 return (810 行内大约 95-115 行段)
  Phase D  terminal_recovery latch + cmd 计算（115-205 段）
  Phase G  terminal_slowdown envelope（260-300 段）
```

- 这三段都会在 ablation 实验中改变 cmd_vel 行为
- C/D/E/F 实验需要 "TRACKING start → first terminal/capture" 窗口 slicing
- 现有 publish 的 `terminal_phase_active` / `terminal_envelope_active` 已经能 slicing，但 control flow
  本身散在 controlLoop 内难以单独打开/关闭

### 1.6 外部依赖映射

```text
local_planner_node.cpp        → LocalPlannerROS::initialize / run
LocalPlannerROS               → PathHandler, MPCSolver, SloshIntegration, RiskScheduler
PathHandler                   → CubicSpline + 17 free helpers
MPCSolver                     → CostFunction, ConstraintManager, DiffDriveModel, DynamicsModel
SloshIntegration              → slosh_models::LiquidSloshModel (跨包依赖)
analysis/*.py (11 个脚本)     → 读 bag 的 /slosh/* /mpc/* /terminal_* /reference/* 等 60 topics
record_slosh_experiment.sh    → rosbag record 同样 60 topics
launch_sim_nav_stack.sh       → 启动 nav + slosh_experiment_sim.launch
run_s_curve_smoke_test.sh     → smoke 入口
```

---

## 2. 分阶段重构方案（mission-driven 重排）

按 0.4 节的 A/B/C 优先级重排。**A 级（Phase 1–2）+ B 级（Phase 3–4）做完即可完整支撑实物 C/D/E/F ablation**；
C 级（Phase 5–11）是长期代码健康，可在 RA-L 投稿后再做。

| Phase | 优先级 | 主题 | 估时 |
|---|---|---|---|
| 0 | 基础设施 | diff_two_bags.py 回归脚本 | 2 h |
| **1** | **A**（ablation 关键） | **cost breakdown 单一来源** | 3 h |
| **2** | **A**（ablation 关键） | **slosh cost 抽 SloshTrackingCost 子类** | 2 h |
| **3** | **B**（ablation 辅助） | **terminal 三段从 controlLoop 切独立方法** | 2 h |
| **4** | **B**（ablation 辅助） | **slosh observer 与 control 解耦** | 4 h |
| 5 | C | path_handler 顶部 17 free helper → path_utils | 1 h |
| 6 | C | local_planner_ros 顶部 5 free helper → local_planner_utils | 1 h |
| 7 | C | controlLoop 完整切分（剩余非-terminal 部分） | 3 h |
| 8 | C | ~50 成员变量装进嵌套 struct | 4 h |
| 9 | C | 60 publisher 聚合为 `LocalPlannerPublishers` | 3 h |
| 10 | C | `path_handler.getReferencePoints` 切分 | 3 h |
| 11 | C | Python `scripts/` 顶层 23 个按职责分子目录 | 2 h |
| 12 | 占位 | TerminalManager / FSM 引擎 / PIMPL（本轮不做） | — |
| **合计** | | | **30 h** |

**节奏建议**：
- 单次会话最多 2 phase
- Phase 2 / 4 / 7 / 9 后必须 review 暂停
- A+B 级（Phase 1–4，共 11 h）做完后强制暂停，先用回归 bag + 实物 dry-run 验证 ablation 流程没退化，再决定是否继续 C 级

---

### Phase 0（基础设施）：回归比对脚本

**改什么**：新增 `scripts/analysis/diff_two_bags.py`，输入两个 bag，对比：
- `/cmd_vel` 序列（浮点 |Δ| < 1e-6）
- `/scout_local_planner/state`（或等价状态机 topic）转换序列
- `/slosh/state` 时间序列
- `/mpc/cost_breakdown` 各项时间序列

输出：PASS/FAIL 与首个出现偏差的 t / topic。

**不改什么**：仓库内任何代码 / launch / param。

**为什么先做**：后续每个 phase 都靠它做"行为等价"判定。

**验证**：用今天的 sim bag `bags/sim_s_curve/20260520_133904_s_curve.bag` 自比，应 PASS。

---

### Phase 1（A 级 / ablation 关键）：cost breakdown 单一来源

**问题**：现 `local_planner_ros.cpp:1915-2002` 的 `computeCostBreakdown` 与 `cost_function.cpp` 的
公式各写一份，C/D/E/F 一旦哪边漏改，breakdown publish 出来的解读就是错的。

**改什么**：

```cpp
// 在 CostFunction 上新增一个公开方法：
struct CostBreakdownPerTerm {
    std::string name;          // "StateTrackingCost", "ControlCost", "ControlRateCost", ...
    double total = 0.0;
    double per_step_max = 0.0;
    // 可继续按需扩展
};

std::vector<CostBreakdownPerTerm>
CostFunction::computeBreakdown(
    const std::vector<StateVector>& x_traj,
    const std::vector<ControlVector>& u_traj,
    const std::vector<ReferencePoint>& refs) const;
```

实现内部：对每个已注册 CostTerm，逐步调用其现有的 `evaluate(x, u, ref, k)`，按 term name 聚合。

然后把 `local_planner_ros.cpp` 的 `computeCostBreakdown` 改为：
1. 调用 `cost_function_.computeBreakdown(...)` 拿到 per-term 字典
2. 把它映射成现有 `CostBreakdown` struct（J_lag / J_contour / J_etheta / J_v / J_control / J_smooth / J_slosh_eta / J_slosh_eta_dot / J_total）
3. **完全删除**当前重复手抄的公式（line 1915-1997 的核心）

**不改什么**：
- `mpc/cost_breakdown` topic 的 layout（21 字段 Float32MultiArray）一字不变
- 每个 J_* 的语义与数值不变
- terminal_factor_slosh_eta / terminal_factor_slosh_eta_dot 的"末端加权"语义不变（在 SloshTrackingCost 里实现，见 Phase 2）

**风险**：中（同一份代码可能对 omega_ff 项的处理细节差异，需要 phase-0 回归脚本严格对比 `J_*` 序列）。

**ablation 价值**：从此 cost breakdown 与 solver 内目标函数 100% 同源，C/D/E/F 任何差异都可信归因到参数。

---

### Phase 2（A 级 / ablation 关键）：抽 `SloshTrackingCost : CostTermBase`

**问题**：slosh 项（Q_slosh_eta, Q_slosh_eta_dot, terminal_factor_*）现在嵌在 `StateTrackingCost` 内
（cost_function.cpp:40-48, 84-93, 290-310），违反 CostTermBase 设计意图，也使 cost breakdown 难以独立追踪。

**改什么**：

```cpp
// cost_function.h 新增
class SloshTrackingCost : public CostTermBase {
public:
    SloshTrackingCost(const MPCParams& params);
    std::string name() const override { return "SloshTrackingCost"; }
    double evaluate(const StateVector& x, const ControlVector& u,
                    const ReferencePoint& ref, int k) const override;
    void getQuadraticCost(int k, int N,
                          Eigen::MatrixXd& Q_contrib,
                          Eigen::MatrixXd& R_contrib,
                          Eigen::VectorXd& q_contrib,
                          Eigen::VectorXd& r_contrib) const override;
    void setParams(const MPCParams& params) { params_ = params; }
private:
    MPCParams params_;
    double terminalFactor(int k, int N, double base) const;
};
```

实现：从 `StateTrackingCost` 把 slosh 相关分支（含 terminal_factor）原样搬过来。

`CostFunction::initialize(...)`：

```cpp
addCostTerm(std::make_shared<StateTrackingCost>(params));
addCostTerm(std::make_shared<ControlCost>(params));
addCostTerm(std::make_shared<ControlRateCost>(params));
addCostTerm(std::make_shared<SloshTrackingCost>(params));  // 新增，原 slosh 项从 StateTrackingCost 移走
```

**不改什么**：
- `mpc/cost_breakdown` topic 的 J_slosh_eta / J_slosh_eta_dot 字段语义、数值
- Q_slosh / Q_slosh_eta_dot / terminal_factor_slosh_eta / terminal_factor_slosh_eta_dot 参数名与数值生效逻辑
- StateTrackingCost 的 e_l / e_c / e_theta / v - v_ref 计算

**风险**：中。需要确认 Q matrix sparsity pattern 在新拆分后与旧 monolithic StateTrackingCost 在数值上等价
（包括 ETA_X / ETA_Y / ETA_X_DOT / ETA_Y_DOT 四个 diagonal 项的位置）。

**ablation 价值**：
- C/E 组（Q_slosh=0）走的 cost term 数学上变成"被 disable"，cost breakdown 显示更清晰
- 未来想做 "slosh 软约束 vs 软代价 vs 硬约束" 等结构对比时，有清晰挂载点
- 把"slosh 单独是不是 0"作为 C/E 是否 ablation 失败的早判据更直接

---

### Phase 3（B 级 / ablation 辅助）：terminal 三段从 controlLoop 切出

**问题**：terminal_capture_stop / terminal_recovery / terminal_slowdown 三段散在 controlLoop 内
（Phase C / D / G 段），analysis 脚本只能靠 `terminal_phase_active` 等 publish 间接判别。

**改什么**：在 `LocalPlannerROS` 新增三个 private 方法（**只搬，不改逻辑**）：

```cpp
// 返回 true 表示已处理 + publish，controlLoop 直接 return
bool runTerminalCaptureStopIfActive(const GoalInfo& goal);

// 返回 true 表示已处理 + publish + return（recovery latch 已生效）
bool runTerminalRecoveryIfActive(const GoalInfo& goal);

// 计算 terminal_slowdown envelope 并应用 v_max 与 Q_v override
void applyTerminalSlowdownIfActive(MPCParams& runtime_mpc_params,
                                    double& v_des_target);
```

controlLoop 内的对应段落改为函数调用，**逻辑零修改**。

**不改什么**：
- terminal_phase_active / terminal_envelope_active / terminal_v_envelope / terminal_recovery_latched
  等 publish 的语义和时机
- 三段的判定阈值与生效边界
- terminal_factor_slosh_eta 对 cost 的影响（这个在 Phase 2 已处理）

**风险**：低（纯方法提取）

**ablation 价值**：
- analysis 脚本 slicing "TRACKING start → first terminal" 窗口的逻辑可以直接从 controlLoop 反推
- 未来需要把 C/E vs D/F 的 terminal 行为做 A/B 对比时，可一键禁用某段（注释一行函数调用）
- 为 Phase 12 的 TerminalManager 抽取打基础

---

### Phase 4（B 级 / ablation 辅助）：slosh observer 与 control 解耦

**问题**：`updateSloshEstimate()` + IMU 滤波 + bias 估计 + odom 差分散布在 LocalPlannerROS，
涉及 ~20 个成员变量。ablation 实验里 observer 状态可能被 control 路径污染（SETTLING 拖长 / cmd_vel
归零导致 odom 差分异常）。

**改什么**：新增 `SloshFeedback` class（仍在 scout_local_planner 包内，不跨包）：

```cpp
// include/scout_local_planner/slosh_feedback.h
class SloshFeedback {
public:
    struct Config { /* IMU bias compensation 参数 + EMA alpha + topic */ };
    struct Output {
        double ax_est, ay_est, alpha_est, omega_est;
        bool   imu_bias_ready;
        double imu_ay_bias;
        // ... 现有 publish 的所有量
    };

    void configure(const Config& c);
    void onOdom(double v, double omega, const ros::Time& t);
    void onImu(double ay_raw, double omega_z, const ros::Time& t);
    Output getOutput() const;
    void reset();
};
```

LocalPlannerROS 持有一个 `SloshFeedback feedback_`；在 odomCallback / imuCallback 内只调
feedback_.onOdom / onImu。controlLoop 取 `feedback_.getOutput()` 喂给 `slosh_integration_.update(...)`。

**不改什么**：
- `slosh/ax_est` / `slosh/ay_est` / `slosh/imu_ay_bias` / `slosh/imu_ay_bias_ready` / `slosh/omega_est_used` 等
  topic 的语义和数值
- IMU bias 估计的窗口、tolerance、EMA 参数
- odom 差分 + ay = v*omega 离心估计的公式

**风险**：中。需要小心把 ~20 个成员的初始化、reset、bias estimator state 一字不漏地搬迁。

**ablation 价值**：
- observer 可以单独跑 GoogleTest（喂一条 IMU 序列得 η 输出），脱离 ROS
- ablation 实验里 control flow 改变（SETTLING / recovery 进入）不会再影响 observer 状态
- 后续若要切换 observer 算法（如加 Kalman），有清晰边界

---

### Phase 5（C 级）：path_handler 顶部 17 free helper → path_utils

同原方案，移动 + #include，纯结构。

### Phase 6（C 级）：local_planner_ros 顶部 5 free helper → local_planner_utils

同原方案。

### Phase 7（C 级）：controlLoop 完整切分（剩余非 terminal 部分）

Phase 3 已经把 terminal 三段切出。Phase 7 处理剩余 phase：

```cpp
void controlLoop_resetTransientFlags();
bool controlLoop_handleNonTracking();              // IDLE / REACHED 早返
void controlLoop_updateRiskScheduler(MPCParams& runtime);
void controlLoop_applySettlingOverrides(MPCParams& runtime);
double controlLoop_smoothVDes(double v_raw);
void controlLoop_evaluateTrackingFeasibility(...);
MPCSolution controlLoop_buildReferencesAndSolve(...);
void controlLoop_publishAllDebug(...);
```

注意：terminal 三段已在 Phase 3 提取，**不要重复**。

### Phase 8（C 级）：成员变量分组结构化

同原方案，把 ~50 个成员装进嵌套 struct。新增 struct：
- `TerminalRecoveryConfig / Runtime`
- `TerminalSlowdownConfig / Runtime`
- `TerminalCaptureStopConfig`
- `InputShapingState`
- `SettlingConfig / Runtime`
- `TrackingFeasibilityConfig / Runtime`
- ~~`SloshFeedbackConfig / Runtime`~~（已在 Phase 4 由 SloshFeedback 类替代）
- `CmdFilterState`
- `VDesRateLimitState`

### Phase 9（C 级）：60 publisher 聚合 `LocalPlannerPublishers`

同原方案。

### Phase 10（C 级）：`path_handler.getReferencePoints` 切分

同原方案。

### Phase 11（C 级 / 独立）：Python `scripts/` 按职责分子目录

同原方案。

### Phase 12（占位，本轮不做）

- `TerminalManager` 类合并 capture_stop / recovery / slowdown 三块业务
- `FSM` 引擎独立化
- LocalPlannerROS PIMPL 化
- GoogleTest 单元测试基础设施

---

## 3. 每阶段的回归验证流程

每个 phase commit 前必须做：

```text
1. 编译
   cd /home/a/scout_ws
   catkin_make --pkg scout_local_planner
   echo "exit code 必须 = 0"

2. Smoke 启动
   bash src/scout_apps/control/scout_local_planner/scripts/run_s_curve_smoke_test.sh
   # 确认能停 goal，cmd_vel 不为 NaN

3. 数值回归（关键）
   3.1 用 phase 前的 commit 跑一次 fixed-path smoke：
       git stash
       run_s_curve_smoke_test.sh   → bags/sim_s_curve/<ts>_baseline.bag
   3.2 应用 phase 改动，再跑一次：
       run_s_curve_smoke_test.sh   → bags/sim_s_curve/<ts>_after_phase_N.bag
   3.3 diff /cmd_vel 序列、/slosh/state、PlannerState 转换：
       python3 scripts/analysis/diff_two_bags.py <baseline> <after_phase_N>
       # 允许浮点 |Δ| < 1e-6
   3.4 不通过 → 立即回退 phase，不进入下一阶段

4. Topic 完整性（Phase 5 必做）
   rostopic list | sort > /tmp/topics_after.txt
   diff /tmp/topics_before.txt /tmp/topics_after.txt
   # 必须空 diff
```

**回归脚本待新增**：`scripts/analysis/diff_two_bags.py`（Phase 0 准备工作）。
该脚本本身不属于本次重构 scope，但是是回归保障的前置依赖，建议作为 Phase 0 单独 commit。

---

## 4. 文件级改动清单（按 phase 汇总，mission-driven 重排版）

| Phase | 优先级 | 新增 | 修改 | 删除 |
|---|---|---|---|---|
| 0 | 基础设施 | `scripts/analysis/diff_two_bags.py` | — | — |
| **1** | **A** | — | `include/.../cost_function.h`（加 `CostFunction::computeBreakdown`） `src/cost_function.cpp`（实现） `src/local_planner_ros.cpp`（`computeCostBreakdown` 改为调用 + 删手抄公式 ~80 行） | — |
| **2** | **A** | — | `include/.../cost_function.h`（加 `SloshTrackingCost` 类） `src/cost_function.cpp`（实现 + 从 StateTrackingCost 移除 slosh 分支 + initialize 注册新 term） | — |
| **3** | **B** | — | `include/.../local_planner_ros.h`（3 个 terminal 私有方法声明） `src/local_planner_ros.cpp`（controlLoop 内 terminal 三段抽出） | — |
| **4** | **B** | `include/.../slosh_feedback.h` `src/slosh_feedback.cpp` | `include/.../local_planner_ros.h`（移除 ~20 个 IMU/odom 滤波成员，改持 `SloshFeedback feedback_`） `src/local_planner_ros.cpp`（odomCallback/imuCallback/updateSloshEstimate 改为 feedback_ 委托） `CMakeLists.txt`（加 slosh_feedback.cpp） | — |
| 5 | C | `include/.../path_utils.h` `src/path_utils.cpp` | `src/path_handler.cpp`（去掉 17 个 free helper 定义，加 include） `CMakeLists.txt` | — |
| 6 | C | `include/.../local_planner_utils.h` `src/local_planner_utils.cpp` | `src/local_planner_ros.cpp`（去掉 5 个 free helper） `CMakeLists.txt` | — |
| 7 | C | — | `local_planner_ros.h`（剩余 ~8 个 private 方法声明） `local_planner_ros.cpp`（controlLoop 剩余 phase 切分） | — |
| 8 | C | — | `local_planner_ros.h`（成员变量装进嵌套 struct，注意 SloshFeedback 已在 Phase 4 处理掉） `local_planner_ros.cpp`（成员引用批量重命名） | — |
| 9 | C | — | `local_planner_ros.h`（用 LocalPlannerPublishers 替换 60 个 publisher 成员） `local_planner_ros.cpp`（advertise + publish 调用迁移） | — |
| 10 | C | — | `path_handler.cpp`（getReferencePoints 切分） | — |
| 11 | C | `scripts/experiment/` `scripts/path/` `scripts/diagnostics/` `scripts/tools/` 子目录（带 `__init__.py` 或 symlink） | 受影响 launch / sh 中的路径引用；新增旧路径 symlink（如有需要） | — |

---

## 5. 不在本方案内的事

- `slosh_models` 包暂不动（已是干净独立库）。
- `cost_function.cpp / mpc_solver.cpp / constraint_manager.cpp` 不动（已是合理大小）。
- `risk_scheduler / slosh_integration / diff_drive_model / cubic_spline` 不动。
- 算法层面的任何改动（MPC cost 公式、slosh 估计、terminal 收敛策略）一律不在重构里做。
- 性能优化（如 publisher 节流、bag 体积压缩）不在本次 scope。
- 单元测试新增（除 Phase 0 的 diff_two_bags.py 这条回归脚本外）不在本次 scope；GoogleTest 测试框架引入留给 Phase 8。

---

## 6. 决策与确认点（mission-driven 修订版）

执行前需用户明确：

1. **是否同意 A + B 级（Phase 1–4，11 h）优先执行**，以最小工作量解锁 C/D/E/F ablation 的可信度？
   - 若是：先做 0 → 1 → 2 → 3 → 4，强制 review 暂停，根据实物 dry-run 决定是否启动 C 级
   - 若否：直接跳到 C 级（与 ablation 无关），按风险升序做 Phase 5 起

2. **C 级（Phase 5–11）何时启动？**
   - A：A+B 级回归通过后立即接力（30 h 一次性做完）
   - B：RA-L 投稿（2026-08）后再做（推荐）
   - C：永远不做，A+B 级足够

3. **每 phase 后是否需要 Code review 暂停？**
   - 默认建议：
     - **Phase 2 后强制暂停**（slosh cost 抽出，可能影响数值）
     - **Phase 4 后强制暂停**（observer 解耦完成，A+B 级里程碑）
     - Phase 7 / 9 后建议暂停

4. **回归基线 bag 用哪个？**
   - sim 基线：`bags/sim_s_curve/20260520_133904_s_curve.bag`
   - **实物基线（ablation 关键）**：`/data/a/slosh_bags/real/20260518_fixed_path_cost/` 中
     C/D 各取 1 包做"重构前后 cost_breakdown / slosh/* 同序列 replay"，
     允许浮点 |Δ| < 1e-6
   - 如果允许重启 mpc 节点 replay bag，phase 1/2 后必须用 C 组和 D 组各跑一次回归

5. **重构期间是否暂停其它代码改动？**
   - 强烈推荐：是（否则 phase 间 diff 被无关改动污染）
   - 例外允许：仅文档 / 非控制路径的 Python 工具（如 analysis 脚本）改动

6. **Phase 1 的细节：cost breakdown 的 J_omega_ff 来源？**
   - 现 `computeCostBreakdown` 里有 J_omega_ff 项，但 cost_function.cpp 的 CostTerm 列表里没有显式的 omega_ff term
   - Phase 1 实施前需要确认：J_omega_ff 是 StateTrackingCost 内嵌的子项，还是已被合并到 J_v
   - 如果是嵌入子项，Phase 1 的 `computeBreakdown` 接口需要支持 per-term 内部 sub-bucket
   - 这一点需要在 Phase 1 启动前的 30 min 代码 walk-through 确认

---

## 7. 工期估算（mission-driven 修订版）

| Phase | 优先级 | 估时 | 累计 |
|---|---|---|---|
| 0 | 基础设施 | 2 h | 2 h |
| **1** | **A**（ablation 关键） | 3 h | 5 h |
| **2** | **A** | 2 h | 7 h |
| **3** | **B**（ablation 辅助） | 2 h | 9 h |
| **4** | **B** | 4 h | **13 h** ← **A+B 级里程碑** |
| 5 | C | 1 h | 14 h |
| 6 | C | 1 h | 15 h |
| 7 | C | 3 h | 18 h |
| 8 | C | 4 h | 22 h |
| 9 | C | 3 h | 25 h |
| 10 | C | 3 h | 28 h |
| 11 | C | 2 h | **30 h** |
| 12 | 占位 | — | — |

**两个里程碑**：

```text
里程碑 1：13 h 后（Phase 4 完成）
  cost breakdown 与 solver 同源
  slosh cost 独立可观测
  terminal 三段可独立切换
  observer 可独立单测
  → C/D/E/F ablation 的数据可信度问题已解决
  → 切回实物 bag 主线无障碍

里程碑 2：30 h 后（Phase 11 完成）
  controlLoop 全切分
  成员变量结构化
  publisher 聚合
  Python 脚本归类
  → 整体代码可维护性显著提升
  → 适合 RA-L 投稿后或团队协作前做
```

**节奏建议**：单次会话 ≤ 2 phase，每周 ≤ 5 h 投入。A+B 级建议 1–2 周内完成；C 级可在
2026-08 投稿后做。

---

## 8. 关联文档

- `docs/Claude/CLAUDE.md` —— 编码准则（已遵守：思考前提 / 简洁优先 / 外科手术式修改 / 目标驱动）
- `docs/重要文档/仿真笔记.md` —— 当前仿真状态（影响 Phase 3 的 smoke test 验证）
- `docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md` —— terminal 收敛策略（必须保持的"外部行为"）
- `docs/Claude/总体推进方案.md` —— OSCRS RA-L 投稿计划（重构不能影响 paper 节奏）
