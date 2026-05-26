# 2026-05-20 scout_local_planner 重构方案（2026-05-26 修订版）

> 目标已变化：旧方案要求“外部行为完全不变、OSCRS 保留”。现在目标是做减法：只保留 SloshPriorityMPC 主线和论文 baseline 对比实验，删除 OSCRS / GeoRef / 旧风险调度 / 旧低激励 profile 等不再服务主线的代码。

---

## 0. 当前结论

### 0.1 新目标

当前代码只需要支撑：

```text
固定 P2_s_curve 路径
  -> C / D / E / F 内部 MPC cost 消融
  -> TOPPRA-style / Ruckig-style 外部 v_ref(s) baseline
  -> RGB 真值 + /slosh/height 模型辅助分析
  -> terminal 单独诊断，不进入主效果窗口
```

对应执行文档：

```text
docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md
```

因此重构成功标准不是“保留所有历史功能”，而是：

```text
1. C/D/E/F 能实物启动、录包、分析；
2. TOPPRA/Ruckig-style 能通过 external_speed_profile_csv 注入同一固定路径；
3. record_slosh_experiment.sh 能录到 RGB、/slosh/*、/mpc/*、/reference/*、/terminal/*、/profile_cap/*；
4. 分析脚本能完成 RGB / model / cost breakdown / terminal / Ferrari-style 指标；
5. OSCRS / GeoRef / 旧风险调度等不再出现在主线 launch、README、record 白名单和 C++ 控制路径中。
```

### 0.2 必须保留的主线

#### C++ 核心

```text
local_planner_node.cpp
local_planner_ros.cpp / .h        当前先保留，后续拆
mpc_solver.cpp / .h
cost_function.cpp / .h
constraint_manager.cpp / .h
diff_drive_model.cpp / .h
dynamics_model.cpp / .h
path_handler.cpp / .h
cubic_spline.cpp / .h
slosh_integration.cpp / .h
types.h
```

#### slosh_models

```text
src/scout_apps/control/slosh_models
```

该包相对独立，当前不用重构。它仍是 MPC 中 slosh state propagation 的物理模型来源。

#### launch / config

```text
launch/slosh_experiment.launch
launch/slosh_experiment_sim.launch
config/mpc_params.yaml
config/mpc_params_sim.yaml
config/ferrari_oracle.yaml
config/visual_height.yaml
```

#### 实验脚本

```text
scripts/launch_real_sensors_stack.sh
scripts/launch_sim_nav_stack.sh
scripts/run_sim_fixed_path_bag.sh
scripts/record_slosh_experiment.sh
scripts/send_fixed_goal.py
scripts/template_fixed_path_generator.py
scripts/fixed_global_path_runner.py
scripts/launch_fixed_path_slosh_stack.sh   # 可选入口，不是实物默认入口
```

#### baseline / retiming 脚本

```text
scripts/analysis/retime_toppra_style.py
scripts/analysis/retime_ruckig_style.py
```

#### 当前分析脚本最小集

```text
scripts/analysis/analyze_fixed_path_cost_effect.py
scripts/analysis/analyze_ferrari_indices.py
scripts/analysis/analyze_slosh_peak_context.py
scripts/analysis/analyze_terminal_approach_1s.py
scripts/analysis/analyze_terminal_transition.py
scripts/analysis/diagnose_terminal_overshoot.py
scripts/analysis/extract_mpc_cost_breakdown.py
scripts/analysis/diagnose_speed_profile.py
scripts/analysis/trajectory_analysis.py
scripts/analysis/simulate_slosh_ode.py
scripts/extract_slosh_metrics.py
```

### 0.3 可以删除或迁出的历史分支

#### 0.3.1 commit 历史分界

不要按 `768003f628e6d2e0825affe2d174f3d2566df7eb..HEAD` 整段回退或整段删除。

该区间可以粗分为三类：

```text
OSCRS / GeoRef 主体引入与整理：
  b9f4cce Add slosh-guided GeoRef validation
  afb8f56 Add OSCRS active GeoRef validation pipeline
  64d7dab 收束 OSCRS 实物前参数与脚本说明
  cd7bbbf 拆分 OSCRS 候选生成层并加入 fixed strong 基线
  a335ef5 修正实物 GeoRef gate 并增加单包行为验收
  9d4122d 改进 GeoRef 单包验收脚本诊断输出
  1ff204b / e9ba64a 调参
  7e67374 重构OSCRS脚本目录与GFRS模块边界
  a7d30af oscrs: add fidelity and diversity diagnostics

模型保真度 / Ferrari / 视觉分析遗产：
  3120c9f 测试模型保真度实验
  3243755 测试模型保真度2
  ce4ca9b 整理SloshPriorityMPC论文执行方案与Ferrari指标脚本
  17afb99 完善SloshPriorityMPC论文对比实验设计与baseline选择

当前 MPC 主线：
  852882f 旧方案再测试
  700fe60 Smooth terminal recovery for slosh MPC tests
  65cb0f4 Add MPC cost breakdown monitoring
  615d32d Add terminal slowdown and cost contribution analysis
  7b3fb16 治理终点过渡与纵向速度参考脉冲
  b8fe6a2 / c260db2 / 25dcc8c terminal 治理
  19e93fd 加入MPC晃动预览代价与G组验证方案
  e886825 加入固定路径外部速度剖面 baseline
```

结论：

```text
可以删除 OSCRS / GeoRef 文件入口；
不能按提交区间整体回滚；
模型保真度脚本先不要删，因为论文 still 需要 Ferrari-style 绝对保真度；
terminal / cost_breakdown / fixed-path / TOPPRA/Ruckig 相关提交属于当前主线，必须保留。
```

### 0.4 实物验证兼容性硬约束

重构不能让 `docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md` 失效。

当前验证方案里仍有大量显式 launch arg：

```text
旧低激励速度剖面开关（已移除）
input_shaping_enable:=false（已移除）
risk_scheduler_enable:=false（已移除）
slosh_speed_governor_enable:=false（已移除）
旧 slosh box constraint 入口（已移除）
terminal_recovery_enable:=false
terminal_slowdown_*
terminal_capture_stop_*
external_speed_profile_csv
external_profile_execution_cap_*
```

因此每删一个 launch arg，都必须同步做三件事：

```text
1. 更新 slosh_experiment.launch / slosh_experiment_sim.launch；
2. 更新 20260518 验证方案里的所有启动命令；
3. 更新 run_sim_fixed_path_bag.sh / scripts/README.md 中对应示例。
```

否则实物验证会出现两类问题：

```text
启动命令仍传已删除 arg，roslaunch 直接失败；
或者旧 arg 已无效但文档还写着它，导致操作者误以为某机制已关闭。
```

录包话题硬约束：

```text
必须保留：
  /camera/color/image_raw
  /camera/color/camera_info
  /imu/data
  /odom
  /cmd_vel
  /scout/global_path_fixed
  /local_path
  /mpc/status_val
  /mpc/cost_breakdown
  /mpc/slosh_horizon_summary
  /reference/v_ref_horizon
  /reference/s_horizon
  /reference/implied_ax
  /reference/implied_ay
  /reference/implied_jerk
  /terminal/*
  /profile_cap/*
  /slosh/*
```

可以删除：

```text
  /scout/global_path_anti_slosh
  /anti_slosh_path/*
  /anti_slosh_path/candidate_report
  /anti_slosh_path/safety_alarm
```

脚本路径短期硬约束：

```text
在实物对比实验完成前，不移动：
  launch_real_sensors_stack.sh
  record_slosh_experiment.sh
  template_fixed_path_generator.py
  send_fixed_goal.py
  run_sim_fixed_path_bag.sh
  scripts/analysis/retime_toppra_style.py
  scripts/analysis/retime_ruckig_style.py
  scripts/analysis/analyze_fixed_path_cost_effect.py
  scripts/analysis/analyze_ferrari_indices.py
  scripts/analysis/extract_mpc_cost_breakdown.py
  scripts/analysis/analyze_terminal_approach_1s.py
```

这些路径已经写进验证方案和日常命令，移动脚本应放在最后，并提供兼容 wrapper。

#### OSCRS / GeoRef 在线路径后处理

这些不服务当前 MPC cost / fixed-path baseline 主线：

```text
launch/anti_slosh_path_post_processor.launch
config/oscrs_container.yaml
scripts/anti_slosh_path_post_processor.py
scripts/oscrs/**
scripts/reference_generation/**
scripts/candidate_generators.py
scripts/generate_anti_slosh_path_candidates.py
scripts/evaluate_anti_slosh_path_candidates.py
scripts/analyze_oscrs_candidates.py
scripts/check_oscrs_model_consistency.py
scripts/check_oscrs_takeover.py
scripts/validate_georef_oscrs_bag.py
scripts/diagnose_georef_budget_gap.py
scripts/diagnose_slosh_guided_georef_score.py
scripts/optimize_anti_slosh_reference.py
scripts/sweep_anti_slosh_timing_candidates.py
scripts/sweep_p3_geometry_candidates.py
scripts/test_candidate_generators_equivalence.py
```

建议处理方式：

```text
Phase A 先 git tag / 分支保留旧状态；
Phase B 用 git rm 删除；
Phase C 更新 README / record / launch 文档；
不要保留一堆 legacy wrapper，否则重构目标会失败。
```

#### C++ 旧实验机制

当前验证方案所有命令都显式关闭这些机制：

```text
旧低激励速度剖面（已移除）
input_shaping（已移除）
risk_scheduler（已移除）
slosh_speed_governor（已移除）
旧 slosh box constraint（已移除）
```

因此它们是主线外分支：

```text
risk_scheduler.cpp / .h
RiskScheduler 相关成员、参数、publisher
input_shaping 相关成员、参数、方法
slosh_speed_governor 相关成员、参数、分支
energy_profile_* 速度剖面分支
slosh box constraint 入口
heading_align 起点原地对齐分支
settling / terminal residual 分支
tracking_curvature_speed_cap 分支
```

注意：

```text
terminal envelope 的 capture 低速目标已迁移到 terminal_capture_v。
删除 terminal_recovery 前，仍需单独验证 recovery 几何分支是否还承担兜底职责。
```

---

## 1. 当前代码问题审查

### 1.1 `local_planner_ros.cpp` 已经不可维护

当前规模：

```text
local_planner_ros.cpp 约 3031 行
local_planner_ros.h   约 426 行
path_handler.cpp      约 1908 行
```

`LocalPlannerROS` 同时负责：

```text
ROS subscribe / publish
参数读取
状态机
terminal envelope
profile execution cap
slosh observer
MPC solve
cost breakdown
reference diagnostics
cmd_vel filter
risk scheduler / input shaping / governor 等旧分支
```

这不是“拆几个函数”能解决的。先删除主线外分支，再抽类，否则只是把复杂度搬家。

### 1.2 `cost_breakdown` 与 solver 仍是两套公式

当前 solver cost 在：

```text
src/cost_function.cpp
```

`/mpc/cost_breakdown` 在：

```text
LocalPlannerROS::computeCostBreakdown()
```

两者手写公式重复。对 C/D/E/F 实验而言，这是高风险点：

```text
如果 solver 真正优化的 J_slosh 与 cost_breakdown 发布的 J_slosh 不一致，
论文里 pct_slosh / J_slosh 占比就不可信。
```

这是重构里最该优先处理的“可信度问题”。

### 1.3 OSCRS 还污染脚本层和录包层

`scripts/README.md` 大量篇幅仍在描述 OSCRS / GeoRef。

`record_slosh_experiment.sh` 仍记录：

```text
/scout/global_path_anti_slosh
/anti_slosh_path/*
/anti_slosh_path/candidate_report
/anti_slosh_path/safety_alarm
```

这会让当前主线混乱：

```text
实验明明是 fixed-path MPC cost，
但脚本说明和 bag 白名单仍像 OSCRS 实验。
```

应该删除这些入口，而不是继续在命令里反复强调“不启动 OSCRS”。

### 1.4 PathHandler 职责混杂

当前 `PathHandler` 同时做：

```text
路径接收 / TF 转换 / 重采样 / 曲率估计 / B-spline 平滑
内部速度剖面 v(s)
旧 energy_profile
外部 CSV v_ref(s)
终点速度处理
reference horizon 生成
```

当前主线只需要：

```text
固定路径 -> 内部 v(s) 或外部 CSV v_ref(s) -> MPC refs
```

旧 `energy_profile_*` 应删掉；TOPPRA/Ruckig 已经是正式外部 timing baseline，不需要再保留一个内部 PROFILE_ENERGY 历史分支。

### 1.5 `scripts/analysis` 也混有历史研究脚本

很多分析脚本是阶段性产物：

```text
day3 / phase4 / 0424 red group / OSCRS step2 / PMG replay / P3 failure
```

它们不影响运行，但会让新实验入口难找。建议保留当前主线最小集，其余移到：

```text
docs/Claude/legacy_scripts_manifest.md
```

或直接删除。若担心复查旧数据，先打 tag，不要在主包里继续保留。

---

## 2. 重构原则

### 2.1 先删旧分支，再做解耦

错误顺序：

```text
先把 local_planner_ros.cpp 拆成很多类；
然后每个类里仍然保留 OSCRS、risk_scheduler、input_shaping、speed_governor、energy_profile。
```

正确顺序：

```text
1. 固化当前可运行 baseline；
2. 删除主线外入口；
3. 删除主线外 C++ 分支；
4. 再拆 Terminal / SloshObserver / Cost / Diagnostics。
```

### 2.2 每次只删一类东西

每个 phase 只做一个主题：

```text
删 OSCRS Python，不同时改 C++；
删 risk_scheduler，不同时改 terminal；
抽 SloshObserver，不同时改 cost；
```

否则出问题无法定位。

### 2.3 删除前必须有回退点

执行删除前先做：

```bash
git tag legacy-oscrs-l2p5-before-prune
```

或者新建分支：

```bash
git branch legacy/oscrs-l2p5
```

这样就可以大胆 `git rm`，不需要在主线里保留 “legacy” 垃圾。

### 2.4 主效果窗口不因重构改变

重构后主评价窗口仍是：

```text
TRACKING start -> first terminal/capture - 1s
```

terminal 仍只做诊断，不进入 SloshPriorityMPC 主效果统计。

---

## 3. 推荐新模块结构

目标目录：

```text
include/scout_local_planner/
  local_planner_ros.h          # ROS glue，只保留订阅、发布、控制循环编排
  mpc_solver.h
  cost_function.h
  constraint_manager.h
  path_handler.h
  slosh_integration.h
  slosh_feedback.h             # 新增：odom/imu -> ax/ay/omega/alpha
  terminal_controller.h        # 新增：terminal envelope / reached gate
  profile_execution_cap.h      # 新增：external v_ref(s) cmd_v cap
  diagnostics_publisher.h      # 新增：/slosh /mpc /reference /terminal /profile_cap 发布
  types.h
```

最终 `LocalPlannerROS` 应只做：

```text
1. load params
2. receive odom / imu / path
3. update slosh feedback
4. ask PathHandler for refs
5. ask MPCSolver solve
6. pass cmd through terminal/profile cap
7. publish diagnostics
```

---

## 4. 分阶段方案

### Phase 0：冻结当前可运行状态

目的：

```text
防止删完旧分支后不知道哪里坏了。
```

动作：

```text
1. 记录当前 commit；
2. 打 tag 或建 legacy 分支；
3. 保存当前验证命令；
4. 用仿真跑 3 个 smoke：
   - internal profile / C
   - TOPPRA-style
   - Ruckig-style
```

通过标准：

```text
catkin_make --pkg scout_local_planner 通过；
三包都能进入 REACHED；
TOPPRA/Ruckig 的 /profile_cap/v_profile 非 NaN；
/profile_cap/cmd_v_post_cap 不长期高于 /profile_cap/v_profile。
```

### Phase 1：删除 OSCRS / GeoRef 文件入口

删除：

```text
launch/anti_slosh_path_post_processor.launch
config/oscrs_container.yaml
scripts/anti_slosh_path_post_processor.py
scripts/oscrs/**
scripts/reference_generation/**
scripts/candidate_generators.py
scripts/generate_anti_slosh_path_candidates.py
scripts/evaluate_anti_slosh_path_candidates.py
scripts/analyze_oscrs_candidates.py
scripts/check_oscrs_model_consistency.py
scripts/check_oscrs_takeover.py
scripts/validate_georef_oscrs_bag.py
scripts/diagnose_georef_budget_gap.py
scripts/diagnose_slosh_guided_georef_score.py
scripts/optimize_anti_slosh_reference.py
scripts/sweep_anti_slosh_timing_candidates.py
scripts/sweep_p3_geometry_candidates.py
scripts/test_candidate_generators_equivalence.py
```

同步修改：

```text
scripts/README.md
record_slosh_experiment.sh
launch_sim_nav_stack.sh 里的 OSCRS 提示
```

验证：

```bash
rg -n "OSCRS|GeoRef|anti_slosh|global_path_anti_slosh|candidate_report" src/scout_apps/control/scout_local_planner
```

期望：

```text
只允许历史文档里出现；
主包代码、launch、record、README 不再出现。
```

### Phase 2：删除 Python 缓存与无关旧分析脚本

删除：

```text
scripts/**/__pycache__
```

删除或迁出 OSCRS 专属分析脚本：

```text
analyze_day3_abc_smoke.py
analysis/summarize_oscrs_step2.py
```

暂时保留模型保真度 / Ferrari / 旧实物数据脚本：

```text
analysis/phase4_model_fidelity_ablation.py
analysis/red_group_0424_*.py
analysis/model_truth_20260513_fidelity.py
analysis/analyze_zeta_fidelity_ablation.py
analysis/analyze_ferrari_indices.py
analysis/compute_ferrari_indices.py
analysis/ferrari_oracle.py
```

理由：

```text
这些脚本不属于运行主线，但论文仍需要 Ferrari-style 绝对保真度和历史模型对比证据。
第一轮不要删；等论文指标链完全迁到 analyze_ferrari_indices.py 后再决定。
```

### Phase 3：清理 launch / config 的旧开关

从 `slosh_experiment.launch` / `slosh_experiment_sim.launch` 删除：

```text
energy_profile_*
input_shaping_*
risk_scheduler_enable
slosh_speed_governor_*
heading_align_*（若确认实物固定路径不需要）
```

从 `mpc_params.yaml` / `mpc_params_sim.yaml` 删除对应段落。

保留：

```text
Q_slosh
slosh_height_ref
slosh_eta_dot_ratio
slosh_preview_factor
Q_slosh_eta_dot
terminal_factor_slosh_eta
terminal_factor_slosh_eta_dot
C/D/E/F 的 Q/R 参数覆盖
terminal_slowdown
terminal_capture_stop
v_des_rate_limit
external_speed_profile_csv
external_profile_execution_cap
slosh_estimator IMU 参数
```

注意：

```text
terminal_capture_v 已替代 terminal_recovery.v_max，成为 terminal envelope 的 capture 低速目标。
下一步可以在 terminal smoke 通过后，评估是否删除 terminal_recovery 其它逻辑。
```

硬要求：

```text
Phase 3 每删除一个 launch arg，都必须同步更新：
  docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md
  scripts/README.md
  scripts/run_sim_fixed_path_bag.sh

更新后再 grep：
  rg -n "energy_profile_enable|input_shaping_enable|risk_scheduler_enable|slosh_speed_governor_enable" \
    docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md \
    src/scout_apps/control/scout_local_planner/scripts/README.md \
    src/scout_apps/control/scout_local_planner/scripts/run_sim_fixed_path_bag.sh
```

如果还有残留，不能进入实物验证。

### Phase 4：删除 C++ 旧机制

不要一次性删除。拆成 6 个独立子阶段：

```text
Phase 4A:
  删除 risk_scheduler.h / .cpp
  删除 LocalPlannerROS 中 risk_scheduler_* 成员和 /risk_scheduler/* publisher

Phase 4B:
  删除 input_shaping 相关方法和成员

Phase 4C:
  删除 slosh_speed_governor 相关方法和成员

Phase 4D:
  删除 PathHandler energy_profile_* 分支

Phase 4E:
  删除 slosh box constraint 代理（已完成）

Phase 4F:
  评估并删除 heading_align / settling / tracking_curvature_speed_cap
```

同步：

```text
CMakeLists.txt 删除 risk_scheduler.cpp
types.h 删除不再使用的参数字段
record_slosh_experiment.sh 删除相关 topic
```

验证：

```bash
catkin_make --pkg scout_local_planner
rg -n "risk_scheduler|input_shaping|slosh_speed_governor|energy_profile" src/scout_apps/control/scout_local_planner
```

期望：

```text
主包代码不再出现这些字符串；
验证文档中可以保留“已删除旧机制”的说明。
```

每个子阶段都要单独：

```text
catkin_make --pkg scout_local_planner
仿真跑 internal profile / C smoke
必要时跑 TOPPRA 和 Ruckig smoke
```

如果一次删完，MPC 不动、terminal 不收敛或 `/slosh/height` 中断时，很难定位问题来源。

### Phase 5：cost 单一来源

目的：

```text
让 solver cost 和 /mpc/cost_breakdown 来自同一套计算逻辑。
```

动作：

```text
1. 把 StateTrackingCost 拆成：
   - TrackingCost
   - SloshStateCost
   - OmegaFeedforwardCost（可选，或内部 sub-bucket）
2. ControlCost 保留 a / omega；
3. ControlRateCost 保留 da / domega；
4. 新增 CostBreakdown 计算接口，由 CostFunction 统一输出；
5. LocalPlannerROS 只负责发布，不再手写公式。
```

通过标准：

```text
C/D/E/F 旧 bag replay 后 /mpc/cost_breakdown 数值一致或差异可解释；
J_slosh_eta / J_slosh_eta_dot / pct_slosh_total 可追溯到 solver 内同一项。
```

### Phase 6：抽 `SloshFeedback`

新类：

```cpp
class SloshFeedback {
public:
  void configure(...);
  void onOdom(double v, double omega, ros::Time t);
  void onImu(...);
  Output output() const;
  void reset();
};
```

迁出：

```text
odom 差分 ax
ay = v * omega
IMU yaw rate
IMU ay bias
EMA filter
```

保留 topic 语义：

```text
/slosh/ax_est
/slosh/ay_est
/slosh/alpha_est
/slosh/omega_est_used
/slosh/imu_*
```

### Phase 7：抽 `TerminalController`

当前 terminal 需要保留，但要干净：

```text
两段式 terminal envelope
goal_position_reached + speed_low -> REACHED
GoalInfo.dx <= 0 -> cmd_v 强制 0
terminal debug topics
```

删除：

```text
旧 terminal recovery ALIGN / APPROACH / FINAL_YAW 分支
```

前提：

```text
capture 低速目标已从 terminal_recovery.v_max 迁移到 terminal_capture_v。
```

硬前置步骤：

```text
1. 新增参数 terminal_capture_v 或 terminal_approach_v；（已完成）
2. terminal envelope 改为使用该新参数；
3. 保留一版兼容读取 terminal_recovery/v_max；
4. 跑 d200 terminal smoke；
5. 通过后再删除 terminal_recovery ALIGN / APPROACH / FINAL_YAW 分支。
```

不能直接删除 `terminal_recovery`，否则 capture 入口目标速度来源会消失，实物终点收敛风险很高。

新边界：

```cpp
TerminalOutput TerminalController::update(goal, current_v, current_omega, raw_cmd, dt);
```

### Phase 8：抽 `ProfileExecutionCap`

当前 TOPPRA/Ruckig baseline 需要保留：

```text
external_speed_profile_csv
external_profile_execution_cap_enable
external_profile_execution_accel_limit
external_profile_execution_decel_limit
external_profile_execution_jerk_limit 默认 0
/profile_cap/*
```

新类：

```cpp
class ProfileExecutionCap {
public:
  Output apply(double cmd_v, double filtered_v, double s_now, const PathHandler& path, double dt);
};
```

目的：

```text
把外部 baseline 的执行层 cap 从 LocalPlannerROS 主循环里移走。
```

### Phase 9：诊断发布器聚合

新增：

```cpp
class DiagnosticsPublisher
```

只做：

```text
advertise
publish /slosh/*
publish /mpc/*
publish /reference/*
publish /terminal/*
publish /profile_cap/*
```

不做任何控制逻辑。

### Phase 10：PathHandler 简化

保留：

```text
path ingest
resample
curvature
internal v(s)
external CSV v_ref(s)
getReferencePoints
```

删除：

```text
energy_profile
历史 candidate / geometry tuning 入口
不再使用的 smoothing 分支（若固定路径实验不用）
```

再拆 helper：

```text
path_utils.h / .cpp
speed_profile.h / .cpp
```

### Phase 11：脚本目录重排

建议最终结构：

```text
scripts/
  experiment/
    launch_real_sensors_stack.sh
    launch_sim_nav_stack.sh
    run_sim_fixed_path_bag.sh
    record_slosh_experiment.sh
  path/
    send_fixed_goal.py
    template_fixed_path_generator.py
    fixed_global_path_runner.py
  analysis/
    保留当前主线分析脚本
  retiming/
    retime_toppra_style.py
    retime_ruckig_style.py
```

但执行时要谨慎：

```text
短期可以先不移动脚本，只清理 README；
因为很多文档和命令已经引用旧路径。
真正移动脚本应放在最后，并保留一版兼容 wrapper。
```

---

## 5. 删除后保留的实验命令口径

重构后，实物主流程仍是：

```text
1. launch_real_sensors_stack.sh
2. roslaunch scout_global_planner mbf_global.launch
3. template_fixed_path_generator.py 生成 /scout/global_path_fixed
4. roslaunch scout_local_planner slosh_experiment.launch
5. record_slosh_experiment.sh
```

C/D/E/F 只改：

```text
Q_slosh
R_a
R_da
terminal_factor_slosh_eta
terminal_factor_slosh_eta_dot
```

TOPPRA/Ruckig 只额外改：

```text
external_speed_profile_csv
external_profile_execution_cap_enable=true
```

---

## 6. 验证矩阵

每个 phase 后至少跑：

```bash
catkin_make --pkg scout_local_planner
git diff --check
```

关键 phase 后跑仿真：

```text
C internal profile
TOPPRA-style external profile
Ruckig-style external profile
```

必须检查：

```text
/mpc_status 能 REACHED
/cmd_vel 非 NaN
/slosh/height 连续发布
/mpc/cost_breakdown 发布
/reference/v_ref_horizon 发布
/terminal/mode 发布
/profile_cap/* 在 TOPPRA/Ruckig 中发布
```

实物前检查：

```text
record_slosh_experiment.sh 白名单不再含 OSCRS 旧 topic；
README 不再推荐 OSCRS 命令；
20260518 验证方案中的命令仍可直接复制运行。
```

---

## 7. 推荐执行顺序

最现实版本：

```text
第 1 次提交：
  Phase 0 + Phase 1
  只删 OSCRS / GeoRef Python、launch、config、README、record 旧 topic。

第 2 次提交：
  Phase 2
  删除 __pycache__ 和 OSCRS 专属 analysis；模型保真度脚本先保留。

第 3 次提交：
  Phase 3
  清理 launch/config 旧开关，并同步更新 20260518 验证方案。

第 4 次提交：
  Phase 4A-4E
  分批删除 risk_scheduler / input_shaping / governor / energy_profile / box constraint。

第 5 次提交：
  Phase 5
  cost 单一来源，保证 cost breakdown 可信。

第 6 次提交：
  Phase 6 + Phase 7
  SloshFeedback + terminal_capture_v + TerminalController。

第 7 次提交：
  Phase 8 + Phase 9
  ProfileExecutionCap + DiagnosticsPublisher。

第 8 次提交：
  Phase 10 + Phase 11
  PathHandler 简化 + 脚本整理。
```

不要一次性做完。前三次提交会明显减少噪声；后四次才是结构解耦。

---

## 8. 现在不建议做的事

```text
1. 不要一上来拆 LocalPlannerROS 成 10 个类；
   先删掉不再服务主线的旧分支。

2. 不要保留 OSCRS legacy wrapper；
   要回看旧方案就切 legacy tag/branch。

3. 不要在重构里改 Q/R 参数、terminal 距离、slosh 模型参数；
   这些属于实验设计，不属于代码治理。

4. 不要把 TOPPRA/Ruckig 做成新 controller；
   它们只是 external v_ref(s) timing baseline。

5. 不要把 RGB 视觉指标和 /slosh/height 混为同级真值；
   RGB 仍是实物液面主指标。
```

---

## 9. 需要你确认的点

执行代码重构前需要确认：

```text
1. 是否允许直接 git rm OSCRS / GeoRef 文件，而不是迁到 legacy 目录？
2. 是否保留旧 analysis 脚本用于历史数据复查，还是全部删除？
3. terminal_recovery 是否可以彻底删除？
   terminal_capture_v 已替代 terminal_recovery.v_max；下一步只剩 recovery 几何分支是否保留的问题。
4. 是否保留 heading_align？
   固定路径实验通常不需要；如果实物偶尔起点 yaw 偏差大，可以先保留到 Phase 3 后再决定。
5. 是否保留 settling？
   当前计划明确不做 terminal residual；如果不做，应删除。
```

我的建议：

```text
先确认 1 和 3。
只要 OSCRS 可删、terminal_recovery 可替换，就可以开始 Phase 0/1。
```
