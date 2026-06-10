# 对话交接文档：2026-06-10 SPMPC 实物前 NO-GO、StartLockRecovery 与 alpha8 仿真门控

> 目的：给新开的 Claude 对话窗口快速接上当前工作，不依赖旧窗口上下文。  
> 工作目录：`/home/a/scout_ws`  
> 当前分支：`experiment/georef-mpc-hybrid`  
> 当前日期：2026-06-10

---

## 0. 给新对话窗口的建议开场提示词

可以在新窗口直接粘贴：

```text
请先阅读：
/home/a/scout_ws/docs/Claude/对话交接文档-2026-06-10-SPMPC实物前NoGo与StartLockRecovery.md

然后继续帮助我推进 src/scout_apps/control/spmpc_local_planner / continuous_mpcc_acados alpha-state 主线的实物前仿真 Gate。注意：当前正式实物结论仍是 NO-GO。你可以启动和关闭仿真，但必须遵守“不动仿真环境”的规则：不改 world/map/URDF/model/spawn 口径/Cartographer/TF/Gazebo 环境，不 reset/move/delete/spawn robot model，不用仿真环境变化掩盖 planner 问题。正式仿真一轮一 case，启动后等 30s，结束后关仿真并等 30s，70s 未到终点即失败。文件尽量归档到 /data/a。
```

---

## 1. 当前总判断

正式结论仍然是：

```text
正式实物试验：NO-GO。
continuous_mpcc_acados alpha-state 主线还没有通过稳定的实物前仿真 Gate。
```

当前状态比 2026-06-09 明显前进了一步：

- warm-start `v_s = Δs / dt` 问题已经修复。
- StartLockRecovery detector-only 观测层已经实现并接入。
- 在一次 **current-sim / current-pose generated path** 口径下，`continuous_mpcc_acados` / `B0` / `alpha_max=8.0` 能到终点。

但这些还不能推出“可以实物”：

- 最新 PASS 是 current-sim diagnostic，不是 strict fresh-sim formal Gate。
- 之前 strict fresh-sim alpha8 仍出现过 near-start low-progress lock。
- 还没有连续多轮 strict fresh-sim PASS。
- 实物安全总闸仍不足，尤其是 hard-zero abort、path-departure safety gate 等。

---

## 2. 用户明确过的仿真/安全规则

用户已经澄清：

```text
可以启动仿真和关闭仿真，但是要记得规则，不动仿真环境。
```

这里的“不动仿真环境”应解释为：

### 可以做

- 按既定命令启动 Gazebo/RViz 仿真。
- 按既定流程关闭 Gazebo/RViz 仿真。
- 启动/停止自己本轮创建的 planner、path generator、goal sender、rosbag。
- 每轮仿真启动后等待 30s。
- 每轮结束后关闭仿真，并等待 30s。
- 正式仿真一轮只跑一个 case。
- 文件归档到 `/data/a` 下，目录清晰。

### 不可以做

- 不改 world、map、URDF、robot model、Gazebo model、spawn 口径。
- 不 reset/move/delete/spawn robot model 来修正结果。
- 不修改 Cartographer/localization/TF 仿真环境。
- 不用环境变化掩盖 planner 问题。
- 不随意 `killall` / `pkill` 大范围杀进程。
- 不删除代码或实验数据。
- 不 `git reset` / `git clean` / `git checkout` / `git push`，除非用户明确要求。

### strict fresh-sim 与 current-sim 的区别

- `strict fresh-sim Gate`：可以用于正式门控结论。必须按既定命令启动 fresh Gazebo/RViz，等 30s，一轮一个 case，结束后关闭并等 30s。
- `current-sim diagnostic`：用户已经打开仿真并明确允许使用时，可以做诊断，但不能启动/停止/重置 Gazebo/RViz 或模型；结果不能标成 strict fresh-sim Gate。

---

## 3. 当前主线技术背景

主评估包：

```text
src/scout_apps/control/spmpc_local_planner
```

对照/成熟控制包：

```text
src/scout_apps/control/scout_local_planner
```

主线 backend：

```text
continuous_mpcc_acados
```

诊断/RouteB backend：

```text
continuous_mpcc_direct_omega_legacy
```

alpha-state OCP 模型口径：

```text
x = [px, py, theta, v, s, omega]
u = [a, alpha, v_s]

px_dot    = v cos(theta)
py_dot    = v sin(theta)
theta_dot = omega
v_dot     = a
s_dot     = v_s
omega_dot = alpha
```

直接 omega / RouteB 模型口径：

```text
x = [px, py, theta, v, s]
u = [a, omega, v_s]
```

alpha-state 命令重构的关键逻辑：

```cpp
const double cmd_v_pre = input.robot.v + u0[0] * input.dt;
const double cmd_omega_pre = input.robot.omega + u0[1] * input.dt;
output.cmd_v = clampValue(cmd_v_pre, 0.0, params_.v_max);
output.cmd_omega = clampValue(cmd_omega_pre, -params_.omega_max, params_.omega_max);
```

warm-start 语义：

```text
WarmStartState::omega = alpha-state yaw-rate state
WarmStartControl::alpha = OCP control d(omega)/dt
WarmStartControl::omega = legacy/debug mirror only
u_prev_[3] = [a, alpha, v_s]
```

---

## 4. 已完成的重要修复和新增能力

### 4.1 修复 warm-start `v_s`

早前问题：warm-start control 的 `v_s` 使用了 `output.states[k].v`，导致路径进度速度语义错误。

已改为使用相邻状态的路径进度差：

```cpp
const double ds = output.states[k + 1].s - output.states[k].s;
control.v_s = ds / input.dt;
```

相关文件：

```text
src/scout_apps/control/spmpc_local_planner/src/warm_start/diff_drive_flatness_warm_start.cpp
src/scout_apps/control/spmpc_local_planner/test/test_diff_drive_flatness_warm_start.cpp
```

### 4.2 新增 StartLockRecovery detector-only 观测层

目的：把 near-start low-progress lock 显式观测出来，暂时不做恢复控制。

新增文件：

```text
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/core/start_lock_recovery.h
src/scout_apps/control/spmpc_local_planner/src/core/start_lock_recovery.cpp
src/scout_apps/control/spmpc_local_planner/test/test_start_lock_recovery.cpp
```

核心原则：

```text
默认 disabled
只 detect_only
不覆盖 /cmd_vel
不伪造 progress
不修改 projector
不修改 objective
不 regenerate acados
不改变已有 topic layout
```

新增 YAML 默认配置：

```yaml
start_lock_recovery:
  enable: false
  detect_only: true
  start_window_s: 0.20
  min_stall_duration_sec: 1.50
  progress_epsilon_s: 0.005
  cmd_v_small_threshold: 0.03
  warm_start_v_s_min: 0.10
  u0_v_s_max: 0.02
  require_monotonic_clip: true
  max_projection_distance_m: 0.50
```

新增 topic：

```text
/spmpc/start_lock/active
/spmpc/start_lock/mode
/spmpc/start_lock/debug
```

`/spmpc/start_lock/debug` 固定 layout：

```text
enabled,detect_only,active,near_start,stall_progress,cmd_suppressed,warmstart_requests_motion,solver_rejects_progress,monotonic_clip_active,projection_distance_unsafe,stall_time_sec,active_count,progress_abs_s,progress_delta_s,projector_raw_s,projector_guarded_s,guard_minus_raw_s,projector_distance,cmd_v,robot_v,warm_start_v_s0,first_shot_u0_vs
```

### 4.3 StartLockRecovery 触发语义

仅当以下条件持续超过 `min_stall_duration_sec` 才 active：

```text
enable=true
detect_only=true
not terminal reached
progress_abs_s <= start_window_s
progress_delta_s <= progress_epsilon_s
abs(cmd_v) <= cmd_v_small_threshold
warm_start_v_s0 >= warm_start_v_s_min
abs(first_shot_u0_v_s) <= u0_v_s_max
如果 require_monotonic_clip=true，则 monotonic_clip_applied 必须为 true
projection distance 未超过 max_projection_distance_m
```

如果 projection distance 超过阈值，mode 会进入：

```text
UNSAFE_PROJECTION_DISTANCE
```

不会标成正常 `ACTIVE_START_LOCK`。

---

## 5. 已改动的关键源码文件

主要改动包括：

```text
src/scout_apps/control/spmpc_local_planner/CMakeLists.txt
src/scout_apps/control/spmpc_local_planner/config/planner/common.yaml
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/core/spmpc_problem.h
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/core/spmpc_solver.h
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/core/types.h
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/ros/diagnostics_publisher.h
src/scout_apps/control/spmpc_local_planner/src/core/spmpc_problem.cpp
src/scout_apps/control/spmpc_local_planner/src/ros/diagnostics_publisher.cpp
src/scout_apps/control/spmpc_local_planner/src/ros/spmpc_local_planner_ros.cpp
src/scout_apps/control/spmpc_local_planner/src/solvers/continuous_mpcc_solver_acados.cpp
src/scout_apps/control/spmpc_local_planner/src/warm_start/diff_drive_flatness_warm_start.cpp
src/scout_apps/control/spmpc_local_planner/test/test_diff_drive_flatness_warm_start.cpp
```

新增：

```text
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/core/start_lock_recovery.h
src/scout_apps/control/spmpc_local_planner/src/core/start_lock_recovery.cpp
src/scout_apps/control/spmpc_local_planner/test/test_start_lock_recovery.cpp
```

文档新增/更新：

```text
docs/Claude/修改日志-时间/2026-06-10.md
docs/Claude/遇到的问题与解决方案/2026-06-09_alpha-state主线实物前NoGo与catkin白名单问题.md
docs/实物实验注意事项/Subagent.md
```

---

## 6. 已跑过的非仿真验证

已经通过的验证包括：

```bash
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" --pkg spmpc_local_planner
```

```bash
catkin_make run_tests_spmpc_local_planner_gtest_test_start_lock_recovery --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner"
```

```bash
catkin_make run_tests_spmpc_local_planner_gtest_test_terminal_controller --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner"
```

```bash
catkin_make run_tests_spmpc_local_planner_gtest_test_diff_drive_flatness_warm_start --force-cmake -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner"
```

```bash
catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="scout_local_planner" --pkg scout_local_planner
```

```bash
git diff --check
```

单测数量：

```text
test_start_lock_recovery: 8/8 passed
test_terminal_controller: 6/6 passed
test_diff_drive_flatness_warm_start: 5/5 passed
```

---

## 7. 关键仿真结果

### 7.1 strict fresh-sim alpha8 after `v_s = Δs/dt`：失败，near-start lock

早前 strict fresh-sim alpha8 复现显示：

```text
warm_start_head control_vs = 0.56 / 0.56 / 0.56
first_shot u0_vs = 0.0
projector raw_s = 0.0 全程
projector guarded_s = 0.05474757 全程
monotonic_clip_applied = 2100/2100
local_traj_head solver s = 0.05474757 全程
local_traj_head proj_s = 0.0 全程
cmd_v ≈ 1e-10
cmd_omega small nonzero ≈ 0.0025~0.0048 rad/s
```

结论：warm-start `v_s` 修复不是最终解，alpha-state 主线仍可能掉入 near-start low-progress basin。

### 7.2 detector-only strict fresh-sim replay：INVALID

归档目录：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/11_gate1_alpha8_start_lock_detector/20260610_163254_gate1_b0_alpha8_detector_only
```

结果：

```text
verdict: INVALID
```

原因：

```text
fixed_global_path_runner.py 等待机器人回到 saved JSON path 起点，未发布 /scout/global_path_fixed。
planner 一直 WAITING_FOR_REFERENCE_PATH。
/spmpc/debug/runtime_bounds、/spmpc/debug/progress_s、/spmpc/terminal/mode 等 precheck topic 缺失。
因此 /spmpc/start_lock/* 没有得到有意义的 near-start lock 观测。
```

关键数据：

```text
odom_xy: [2.740338916275659, 0.1537987877711208]
dist_to_start: 0.7099770975946051
status_counts:
  INITIALIZED: 1
  WAITING_FOR_ODOM: 2
  WAITING_FOR_REFERENCE_PATH: 17
start_lock_active_count: 0
```

这不是 PASS/FAIL，不能用于评估主线是否能跑。

### 7.3 current-sim / current-pose generated path detector run：PASS，但不是 formal Gate

用户手动恢复仿真后，允许使用当前仿真，但要求不要更改仿真环境。随后执行了 current-sim diagnostic：

归档目录：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/12_gate1_alpha8_current_pose_start_lock_detector/20260610_165738_gate1_b0_alpha8_current_sim_detector_only
```

关键文件：

```text
run_summary.json
precheck.json
initial_check.json
20260610_165738_gate1_b0_alpha8_current_sim_detector_only.bag
```

结果：

```text
verdict: PASS
scope: current-sim/current-pose generated fixed path
fresh_sim_gate_valid: false
```

precheck：

```text
robot_model_present: true
odom_present: true
path_present: true
path_start_distance_m: 0.0939965
path_start_near_odom: true
backend: continuous_mpcc_acados
runtime_alpha_max: 8.0
generated_alpha_max: 1.2
progress_s: 0.0
terminal_mode: TRACKING
start_lock_enabled: true
start_lock_detect_only: true
```

run summary：

```text
progress_s:
  start: 0.0
  end:   0.9651558
  max:   0.9651558

status_counts:
  INITIALIZED: 1
  WAITING_FOR_TF_POSE: 1
  B0_ACADOS_OK: 721
  ACADOS_SOLVE_FAILED_4: 57
  GOAL_REACHED: 104

terminal_mode_counts:
  TRACKING: 570
  TERMINAL_SLOWDOWN: 104
  TERMINAL_CAPTURE_STOP: 102
  REACHED: 104

start_lock_active_count: 0
first_active_time: null
debug_max_stall_time_sec: 0.0

mode_counts:
  MONITORING: 584
  NO_VALID_OUTPUT: 57
  UNSAFE_PROJECTION_DISTANCE: 239

robot_displacement_m: 4.255
```

解释：

- 这是正向信号，说明 alpha-state 主线在 current-pose generated path 口径下不是完全跑不动。
- 但它不是 strict fresh-sim Gate，不解除 NO-GO。
- `UNSAFE_PROJECTION_DISTANCE: 239` 需要继续观察，不能解释成问题已彻底解决。

---

## 8. 路径口径的重要区别

### saved JSON same-path replay

代表：

```text
/data/a/fixed_paths/sim/P2_s_curve.json
fixed_global_path_runner.py
```

优点：路径完全固定，适合横向对比。

风险：如果机器人不在 saved path start 附近，`fixed_global_path_runner.py` 会等待，不发布 `/scout/global_path_fixed`，导致 planner 一直 `WAITING_FOR_REFERENCE_PATH`。

### current-pose generated fixed path

代表：

```bash
rosrun scout_local_planner template_fixed_path_generator.py \
  --template s_curve \
  --goal-topic /scout/goal \
  --output-topic /scout/global_path_fixed \
  --path-file <path.json> \
  --start-heading current \
  --spacing 0.05 \
  --amplitude-ratio 0.18 \
  --min-amplitude 0.25 \
  --max-amplitude 1.20 \
  --publish-count 0
```

固定目标常用：

```text
GOAL_X=-1.2
GOAL_Y=2.6
GOAL_YAW=1.0
```

这个口径符合用户之前说的：

```text
启动仿真是小车当前位置为起点，固定终点，形成固定路径
```

当前建议：下一轮 formal Gate 先跑：

```text
strict fresh-sim + current-pose generated fixed path + fixed goal
```

---

## 9. 下一步推荐计划

### Step 1：跑 3 轮 strict fresh-sim alpha8 current-pose generated path Gate 1

每轮规则：

```text
1. 启动 fresh sim。
2. 不改仿真环境。
3. 启动后等待 30s。
4. 当前 odom 为路径起点，固定终点生成 /scout/global_path_fixed。
5. 启动 continuous_mpcc_acados / B0 / alpha_max=8.0。
6. StartLockRecovery enable=true, detect_only=true。
7. precheck 必须通过：odom、path、runtime_bounds、progress_s、terminal、start_lock topics。
8. 70s 内到终点则 PASS，否则 FAIL。
9. 结束后关闭仿真，等待 30s。
10. 归档到 /data/a。
```

建议至少：

```text
3/3 PASS 才能讨论下一阶段。
5/5 PASS 更稳妥。
```

### Step 2：补实物安全总闸

即使 3/3 PASS，也还建议补：

```text
hard-zero safety abort
path-departure safety gate
odom/TF stale safety
solver invalid safety
projection distance unsafe safety
start-lock persisted safety
```

StartLockRecovery 目前只是 detector-only，不能代替安全保护。

### Step 3：跑 safety-abort smoke 和低速限幅仿真

正式实物前需要验证：

```text
异常时能停
偏航/偏离路径时能停
solver 连续失败时能停
cmd_vel 不尖峰
```

---

## 10. 实物试验还差多远

当前估计：

```text
约 60% ~ 70%。
```

理由：

- 主线已经不是完全不可运行。
- current-sim alpha8 已经到终点。
- detector-only 观测已具备。
- 但 formal fresh-sim 证据不足。
- 安全 abort 层还不足。
- 不能用单次 current-sim PASS 推出实物可行。

当前最短路径：

```text
3 轮 strict fresh-sim alpha8 current-pose generated path PASS
↓
补 hard-zero / path-departure / stale safety
↓
跑 safety smoke
↓
整理实物 checklist
↓
再讨论低速空场实物
```

---

## 11. 需要特别避免的误判

不要说：

```text
alpha-state OCP 已经解决。
主线已经完备。
可以开始实物试验。
current-sim PASS 等价于 fresh-sim Gate PASS。
```

可以说：

```text
alpha-state 主线在 current-pose generated path 口径下出现了正向结果。
StartLockRecovery detector-only 可以观测 near-start lock。
下一步需要 strict fresh-sim 多轮 Gate 证明稳定性。
正式实物仍 NO-GO。
```

---

## 12. 当前 git 状态提示

当前工作区有大量未提交改动和未跟踪文件。新对话应先用：

```bash
git status --short
```

确认状态，不要误删或 reset。

已知新增/改动大类：

```text
StartLockRecovery detector-only 源码与单测
warm-start v_s 修复与测试更新
continuous_mpcc_solver_acados 相关 debug/alpha-state 改动
诊断 publisher 新增 /spmpc/start_lock/*
common.yaml 新增 start_lock_recovery 默认配置
docs/Claude 修改日志和 No-Go 文档
docs/实物实验注意事项/Subagent.md
实验记录文档
```

新对话中若要提交/清理，必须先让用户确认提交范围。

---

## 13. 关键归档路径

invalid detector fresh-sim replay：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/11_gate1_alpha8_start_lock_detector/20260610_163254_gate1_b0_alpha8_detector_only
```

current-sim detector PASS：

```text
/data/a/scout_spmpc_experiments/raw/2026-06-10/12_gate1_alpha8_current_pose_start_lock_detector/20260610_165738_gate1_b0_alpha8_current_sim_detector_only
```

关键 No-Go 文档：

```text
docs/Claude/遇到的问题与解决方案/2026-06-09_alpha-state主线实物前NoGo与catkin白名单问题.md
```

修改日志：

```text
docs/Claude/修改日志-时间/2026-06-10.md
```

Subagent 安全 SOP：

```text
docs/实物实验注意事项/Subagent.md
```

---

## 14. 推荐的新窗口下一步

如果用户说“继续”，最合理的下一步不是改代码，而是跑一轮正式门控：

```text
strict fresh-sim alpha8 current-pose generated path Gate 1
```

要求：

```text
backend=continuous_mpcc_acados
variant=B0
alpha_max=8.0
start_lock_recovery.enable=true
start_lock_recovery.detect_only=true
path_semantics=current-pose generated fixed path to fixed goal
fresh sim yes
wait after launch 30s
one case per sim
70s timeout
shutdown sim after run
wait after shutdown 30s
archive under /data/a
```

这轮如果 PASS，再跑第 2/3 轮；如果 FAIL，立刻分析 bag，不要进入实物。
