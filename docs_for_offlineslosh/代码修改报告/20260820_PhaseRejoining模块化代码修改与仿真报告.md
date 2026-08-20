# Phase-Rejoining 模块化代码修改与仿真报告

- 日期：2026-08-20
- 目标包：`src/scout_apps/control/spmpc_local_planner`
- 对应方案：`docs_for_offlineslosh/方案/20260820_PhaseRejoining模块化代码修改与仿真方案.md`

## 1. 结论与当前放行状态

本轮已完成 Phase-Rejoining 的模块化开发实现、单元测试和指定 proxy 仿真 S0–S4。实现包括执行前沿对齐、有限相位候选、9 维经验恢复 gate、acados 名义相对代价与终端约束、保存恢复动作、`off/monitor/enforce` 三种模式以及 typed diagnostics。

当前结论只能写成：

> **execution-front-aligned phase-rejoining residual S-MPCC with a development empirical recovery gate**

当前状态如下：

| 项目 | 状态 | 结论边界 |
|---|---|---|
| 模块化代码 | 完成 | 核心算法与 ROS 适配已分离 |
| C++/Python 测试 | 通过 | 346 个 catkin 测试 + 10 个 Python 测试 |
| proxy 仿真 | S0–S4 通过 | 证明接口、时序和各控制分支可运行 |
| development artifact | 可用于仿真 smoke | 不是 OfflineSloshOCP，也没有 held-out 证据 |
| 实物 | 未运行 | 按用户要求只提供操作顺序和停止条件 |
| G0 总 lead 验证 | 未完成 | 实物 `enforce` 仍为 **NO-GO** |
| recovery funnel/certificate | 未完成 | 不能声称鲁棒性、递归可行性或安全保证 |

默认配置仍为 `phase_rejoin.mode=off`；development artifact 默认禁止进入 `enforce`。

## 2. 模块化结构

依赖方向为：

```text
NominalSequenceArtifact ─┐
PhaseCandidateSelector ──┼→ PhaseRejoinCoordinator → ROS adapter
EmpiricalRecoveryGate ───┘              │
                                        └→ SolverInput.phase_rejoin
                                                   │
                                                   └→ acados wrapper/model
```

核心 `phase_rejoin/` 不依赖 `roscpp`、ROS message 或 parameter server。ROS 层只负责参数、状态转换、生命周期、诊断发布和安全优先级。

| 模块 | 主要职责 |
|---|---|
| `NominalSequenceArtifact` | 严格加载 CSV、校验 metadata/schema、索引、时间、路径长度和 gate 半径 |
| `PhaseCandidateSelector` | 在上一接受索引附近生成有限、单调候选，计算 $j,j_f,j_e$ |
| `EmpiricalRecoveryGate` | 计算 9 维对角椭球 metric，包含 yaw wrap |
| `PhaseRejoinCoordinator` | 组织候选、nominal stages、当前/终端 gate、残差限幅、恢复与停车决策 |
| `ExecutionStatePredictor` | 分别传播线/角延迟和可选一阶惯性，输出共同执行前沿状态与 epoch |
| acados wrapper/model | 短窗口 nominal-relative 代价和阶段选择性终端经验 gate |
| `SpmpcLocalPlannerROS` | 模式门控、合同验证、安全优先级及最终命令接线 |
| `PhaseRejoinDebug` | 用 `cycle_id` 可连接的逐周期 typed 诊断 |

## 3. 实际代码改动

### 3.1 新增核心文件

```text
include/spmpc_local_planner/phase_rejoin/types.h
include/spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h
include/spmpc_local_planner/phase_rejoin/phase_candidate_selector.h
include/spmpc_local_planner/phase_rejoin/empirical_recovery_gate.h
include/spmpc_local_planner/phase_rejoin/phase_rejoin_coordinator.h

src/phase_rejoin/types.cpp
src/phase_rejoin/nominal_sequence_artifact.cpp
src/phase_rejoin/phase_candidate_selector.cpp
src/phase_rejoin/empirical_recovery_gate.cpp
src/phase_rejoin/phase_rejoin_coordinator.cpp
```

### 3.2 新增接口、工具和测试

```text
msg/PhaseRejoinDebug.msg
scripts/prepare_phase_rejoin_development_artifact.py
scripts/tests/test_phase_rejoin_development_artifact.py
test/test_nominal_sequence_artifact.cpp
test/test_phase_rejoin.cpp
```

### 3.3 修改的现有部分

- `CMakeLists.txt`：接入新消息、核心库和 GTest；
- `core/types.h`：在 `SolverInput` 中加入 `PhaseRejoinSolverContext`；
- `delay_phase_types.h`、`execution_state_predictor.*`：双通道延迟、惯性、state age 和未来 prediction epoch，并复用既有 `SloshDynamics::stepWithDt`，避免积分步内重复配置和日志爆炸；
- `continuous_mpcc_solver_acados.cpp`：严格校验 enforce context，逐 stage 注入 nominal 与 gate 参数；
- `scripts/acados/spmpc_acados_{model,cost,constraints}.py`：新增名义相对代价及第二个非线性 gate；
- `scripts/acados/generate_spmpc_acados.py`：生成前自检 inactive gate 与一半径边界；
- `spmpc_local_planner_ros.*`：参数、artifact、执行前沿、coordinator 和安全门接线；
- `diagnostics_publisher.*`：发布 `/spmpc/debug/phase_rejoin`；
- `common.yaml`：新增默认关闭的 phase-rejoin 参数；
- `spmpc_experiment.launch`、`spmpc_fixed_path.launch`：显式暴露 phase-rejoin 与完整历史参数；
- `scripts/README.md`：记录 development exporter 的证据限制；
- `test_execution_state_predictor.cpp`、`test_replay_diagnostics.cpp`：增加执行前沿和 solver 注入回归测试。

acados codegen 产物按仓库既有策略由 `generated/acados/.gitignore` 忽略，本轮已在本机重新生成 slosh solver；其接口为 `SPMPC_SLOSH_NP=55`、`SPMPC_SLOSH_NH=2`。新工作区需要执行：

```bash
python3 src/scout_apps/control/spmpc_local_planner/scripts/acados/generate_spmpc_acados.py \
  --model slosh
```

随后再构建 C++ wrapper；静态断言会拒绝旧的参数宽度或只有一个非线性约束的旧 solver。

## 4. 方案项到实现证据的映射

| 方案要求 | 实现位置 | 直接证据 |
|---|---|---|
| 双通道延迟与惯性 | `ExecutionStatePredictor` | 独立采样测试；S4 `front_steps=7`、`FIXED_CLOSED_LOOP_OK` |
| 未来执行前沿 epoch | predictor + ROS timing | epoch 单测；S4 `solver_origin_at_execution_front=true` |
| 冻结 artifact 合同 | `NominalSequenceArtifact` | 缺列、NaN、索引、半径、时钟漂移测试；S3b/S4 合同接受日志 |
| 有限相位候选 | `PhaseCandidateSelector` | bounded/monotonic 单测；S3b 候选数 1–5、索引单调 |
| 经验恢复 gate | `EmpiricalRecoveryGate` | yaw wrap/边界单测；S4b/S4c 分支证据 |
| 保存恢复动作 $\kappa$ | artifact + coordinator | coordinator 单测；S4b 实际发布 734 周期 |
| nominal-relative OCP | acados model/wrapper | 生成器自检、replay diagnostics；S4a residual clamp |
| 阶段终端 gate | acados constraint | `SPMPC_SLOSH_NH==2` 静态契约；S4b/c 强制不可行分支 |
| monitor 不干预 | coordinator + solver wrapper | 单测；S3b 638/638 周期命令与基线求解输出一致 |
| enforce fail closed | ROS + coordinator + wrapper | 单测；S4 首 4 周期停车、S4c 531 周期停车 |
| 安全门优先 | ROS 命令链 | S4b 后 550 周期由 `TRACKING_UNSAFE_PROJECTION` 覆盖 |
| typed diagnostics | `PhaseRejoinDebug.msg` | S3b/S4 每周期均有可按 `cycle_id` 连接的消息 |

## 5. 关键实现语义

### 5.1 执行前沿

共同索引前沿为：

$$
d_f=\max(d_v,d_\omega),\qquad
n_f=\left\lceil d_f/\Delta t\right\rceil.
$$

传播到物理时刻 $t+s$ 时，线、角通道分别读取：

$$
u_v^{\mathrm{pub}}(t+s-d_v),\qquad
u_\omega^{\mathrm{pub}}(t+s-d_\omega).
$$

预测从 source-stamped 状态传播至 `evaluation_time + d_f`。`prediction_epoch` 是未来执行前沿，而不是调用 predictor 的当前时刻。

### 5.2 相位索引

候选 $j$ 的语义固定为候选当前名义索引：

$$
j_f=j+n_f,\qquad j_e=j+n_f+N_\ell.
$$

候选只在上一接受索引附近搜索，不进行全局 nearest-neighbour，不允许自由时间伸缩，也不会重复计算 delay offset。

### 5.3 经验 gate

v1 gate 使用 9 维对角椭球：

$$
m_i(e)=\sum_q(e_q/r_{q,i})^2,\qquad m_i(e)\le1.
$$

状态包含 `x,y,yaw,v,omega,eta_x,eta_x_dot,eta_y,eta_y_dot`。它没有显式包含完整 actuator/delay-buffer 状态，所以诊断永久发布：

```text
state_complete_for_certificate=false
```

### 5.4 三种模式

| 模式 | OCP 是否改变 | 命令是否改变 |
|---|---:|---:|
| `off` | 否 | 否 |
| `monitor` | 否 | 否 |
| `enforce` | 是 | 可能改变 |

solver wrapper 只把 `active && enforce` 作为修改 OCP 的授权位。monitor context 即使被传入 solver，也被严格忽略。

### 5.5 enforce 前置条件

初始化时强制检查：

- 10 维 slosh acados mainline；
- `delay_phase=fixed_closed_loop`；
- `delay_phase/require_complete_history=true`；
- `state_timing/require_common_epoch=true`；
- artifact、frame、path length、dt 和 contract ID 一致；
- development artifact 必须显式开启仿真专用 override；
- $N_\ell$ 不超过求解器 horizon；
- post-solver limiter 若开启，必须纳入 fail-closed command contract。

### 5.6 求解失败和安全优先级

正常控制分支为：

```text
求解成功且终端 gate 通过 → residual-clamped 第一拍
求解失败且当前 execution-front 误差在 gate 内 → 保存的 κ
求解失败且当前 gate 外 → 受控停车
人员/碰撞/跟踪/命令合同安全门 → 覆盖上述所有结果
```

不能根据“上一周期预测未来会进入 gate”调用恢复动作；必须重新检查当前 execution-front 状态。

## 6. 构建和自动测试

为避免高并发，构建和测试均串行执行：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner -j1
catkin_make run_tests_spmpc_local_planner -j1
catkin_test_results build

python3 -m unittest \
  src/scout_apps/control/spmpc_local_planner/scripts/tests/test_phase_rejoin_development_artifact.py
```

结果：

```text
Summary: 346 tests, 0 errors, 0 failures, 0 skipped
Ran 10 tests
OK
```

主要新增覆盖包括：

- 不同 $d_v,d_\omega$ 的独立历史采样；
- 一阶惯性与 `tau=0`；
- state age 和未来 execution-front epoch；
- artifact metadata、非有限值、非连续索引、半径、endpoint；
- proxy 40/40/20 ms 时钟量化及累计漂移拒绝；
- bounded/monotonic candidates 和单次 delay offset；
- yaw wrap、gate 边界；
- monitor 命令不变；
- enforce residual、恢复和停车；
- development artifact 默认禁止 enforce；
- stage-local gate、monitor baseline 不变和非法 context fail closed；
- exporter 的一对一 `cycle_id`、时序、safety/limiter 和证据标签。

测试后 CMake whitelist 已恢复为：

```text
spmpc_sim_local_planner
```

## 7. 仿真环境与可复现入口

总证据目录：

```text
/data/a/scout_sim_replacement/logs/phase_rejoin_20260820_vYKNMs
```

ROS/Gazebo master：

```text
ROS_MASTER_URI=http://localhost:11328
GAZEBO_MASTER_URI=http://localhost:11362
```

### 7.1 实际环境启动方式

文档推荐入口是：

```text
/data/a/scout_sim_replacement/scripts/launch_proxy_sim_localization_env.sh
```

该脚本本身只是设置 environment-only 默认值，再 `exec` 到
`launch_proxy_sim_localization_spmpc.sh`。本轮实际直接调用了后者，并显式使用：

```bash
START_PROXY=true \
START_LOCALIZATION=true \
START_PATH_PUBLISHER=false \
START_SPMPC=false \
SCOUT_PROXY_FULL_ROS_MASTER_URI=http://localhost:11328 \
SCOUT_PROXY_FULL_GAZEBO_MASTER_URI=http://localhost:11362 \
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream \
USE_RVIZ=false \
GAZEBO_GUI=false \
TRACKING_RVIZ=false \
LOCALIZATION_RVIZ=false \
LOG_DIR=<本阶段环境日志目录> \
/data/a/scout_sim_replacement/scripts/launch_proxy_sim_localization_spmpc.sh
```

因此实际 `STACK_MODE=environment_only`，没有由环境脚本启动 path、SPMPC 或 SIM-R8。路径随后独立接入：

```bash
START_PATH_PUBLISHER=true \
START_SPMPC=false \
REQUIRE_TRACKING_RVIZ=false \
GOAL_X=4.0 GOAL_Y=0.0 GOAL_YAW=0.0 \
PATH_TEMPLATE=s_curve \
SCOUT_PROXY_FULL_ROS_MASTER_URI=http://localhost:11328 \
SCOUT_PROXY_FULL_GAZEBO_MASTER_URI=http://localhost:11362 \
LOG_DIR=<本阶段路径日志目录> \
/data/a/scout_sim_replacement/scripts/launch_proxy_spmpc_localized_attach.sh
```

控制器使用 `spmpc_fixed_path.launch` 单独启动。S4 的关键参数模式为：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_slosh \
  reference_target_frame:=map \
  shared_linear_accel_limit_enable:=false \
  shared_angular_limit_enable:=false \
  delay_phase_mode:=fixed_closed_loop \
  delay_phase_require_complete_history:=true \
  phase_rejoin_mode:=enforce \
  phase_rejoin_artifact_path:=<artifact.csv> \
  phase_rejoin_allow_development_artifact_in_enforce:=true \
  phase_rejoin_required_contract_id:=<contract_id> \
  phase_rejoin_required_frame_id:=map
```

`allow_development_artifact_in_enforce=true` 只用于本轮 proxy 分支测试，禁止照搬到实物。

各阶段 rosbag 至少记录了：

```text
/cmd_vel
/odom
/scout/global_path_fixed
/spmpc/status
/spmpc/debug/control_cycle_audit
/spmpc/debug/phase_rejoin
/spmpc/debug/predicted_horizon
/spmpc/debug/progress_s
```

S0 另以 readiness 文件保存 `/map`、`/scan_front`、`/imu/data` 和关键 TF；S4 还记录 execution-state、delay-phase 与 command-intervention 诊断。

每次只运行一套环境、一个路径发布器和一个控制器；所有阶段结束后仅停止各自被跟踪的 session，没有使用 `killall/pkill`。

## 8. S0–S4 仿真结果

### 8.1 S0：environment-only 边界

日志：

```text
S0_environment/
```

验证通过：

- `/map`、`/odom`、`/scan_front`、`/imu/data` 有数据；
- `map→base_link` 和 `odom→base_link` TF 可查询；
- 接入控制前 `/cmd_vel`、`/scout/global_path`、`/scout/global_path_fixed` 无发布者；
- 未启动 SPMPC、路径发布器或 SIM-R8。

### 8.2 S1：`phase_rejoin=off` 回归

最终有效包：

```text
S1b_off/S1b_phase_off.bag
```

4 m S 路径完整到达。`ControlCycleAudit` 共 1971 周期：

| 状态 | 周期数 |
|---|---:|
| `B_slosh_ACADOS_OK` | 540 |
| `TERMINAL_SPIN_FAIL` | 2 |
| `GOAL_REACHED` | 1429 |

命令合同违规为 0。最初的短路径也能到达，但很快进入终端控制，不适合生成 artifact，因此改用 4 m 路径。

### 8.3 S2：development artifact

#### 失败尝试

1. `S2_artifact_source/S2_terminal_disabled.bag`：目标容差仍为 0.20 m。有效周期 5–408 连续，但第 0 stage 最大进度仅 `4.451871898 m`，相对 `4.657341781 m` 路径短约 `0.20547 m`，被完整尾段合同拒绝。
2. S2b：收紧目标容差后才启动 recorder，起步段缺失；目录中只有 runtime 参数，没有完整 bag，不能补行或伪造起点。
3. S2c：在控制器启动前先录包，得到正式 development smoke 来源。

#### 最终来源

```text
S2c_artifact_source/S2c_terminal_disabled.bag
```

- 路径长度：`4.6592474841790832 m`；
- 严格区间：cycle 5–427；
- 连续有效周期：423；
- 最后一行进度：`4.610777563833098 m`；
- 距几何末端：`0.04847 m`。

导出的三个 artifact 均写死：

```text
evidence_level=development_only
source=development_proxy_replay
artifact_role=interface_smoke_only
nominal_sequence_kind=rolling_local_planner_first_stage_proxy
offline_slosh_ocp=false
hardware_formal_release=false
paper_main_result_eligible=false
```

| 文件 | 用途 | SHA-256 |
|---|---|---|
| `proxy_s_curve_4m_dev_wide_v1.csv` | 宽 gate，正常 enforce smoke | `5604d287a5f76775ccde5fd0f1c2549dccc680d57bc8e052a5a433a99d53e6be` |
| `proxy_s_curve_4m_dev_hybrid_v1.csv` | 前 10 行宽、其后窄，强制恢复分支 | `e024675f29bb1c9f3400d21629686e21931c14d9c54dcea9c7f4084d03f15e1a` |
| `proxy_s_curve_4m_dev_narrow_v1.csv` | 全窄 gate，强制停车分支 | `4d166b7da92e8d3c2ca4f25f716de1a2929632642c853b21b7353069f45dcad6` |

三个 artifact 的 exporter validator 和 C++ loader 均通过。保存恢复动作在 development 参数中显式设置为 $\kappa=(0.05,0)$；这些人工参数只用于分支 smoke。

### 8.4 S3：monitor

首轮 S3 的 bag 显示 548 条 phase 消息全部 `artifact_loaded=false`，其中 392 条为 `ARTIFACT_UNAVAILABLE`。原因是 exporter 已允许 proxy `/clock` 与 30 Hz 控制周期形成的 40/40/20 ms 有界量化，而当时的 C++ loader 仍按固定周期拒绝。修复后两端统一为：

- 单周期偏差允许到名义 `dt` 的 40%；
- 累计相位漂移不得超过一个 `dt`。

成功包：

```text
S3b_monitor/S3b_monitor_wide.bag
```

结果：

- 638 个 phase/audit/cmd 周期；
- 425 个 `MONITOR_TERMINAL_ACCEPTED`；
- 213 个 `BYPASSED_TERMINAL_PRIORITY`；
- 当前、前沿、终端索引全部单调；
- 候选数 1–5；
- `front_steps=7`、`liquid_steps=3`、`solver_terminal_step=10`；
- 425 次终端 gate 接受、0 次拒绝；
- 425 次当前 gate 接受、0 次拒绝；
- `command_intervened=0`；
- 638/638 phase 输出等于 solver 输出并等于实际发布命令；
- command-contract violation、safety intervention 均为 0；
- `state_complete_for_certificate` 始终为 false。

monitor 的 solver 起点仍是当前状态，所以终端检查步为 $n_f+N_\ell=10$；它没有把执行前沿状态或 phase context 注入 OCP。

### 8.5 S4a：宽 gate enforce

包：

```text
S4a_enforce_wide/S4a_enforce_wide.bag
```

结果：

- 完成整条路径，最终 progress 为 `0.99900347`；
- 最初 4 周期因 220 ms 命令历史不完整，发布受控零命令；
- 之后 557 周期全部 `prediction_status=FIXED_CLOSED_LOOP_OK`；
- `solver_origin_at_execution_front=true`；
- 380 个 `ENFORCE_TERMINAL_ACCEPTED`；
- 177 个终端优先 bypass；
- 195 个周期实际修改 solver 命令；
- 线速度残差 154 次触及 $\pm0.08$；
- 角速度残差 59 次触及 $\pm0.20$；
- `solver_terminal_step=3`；
- command-contract violation 和 safety intervention 均为 0。

该结果验证执行前沿、OCP 注入、终端 gate 和残差限幅接线，不证明液体性能优于基线。

### 8.6 S4b：保存策略恢复

包：

```text
S4b_enforce_recovery/S4b_enforce_recovery.bag
```

hybrid artifact 的前 10 行使用宽 gate，之后使用 $10^{-6}$ 量级窄半径。因此当前 execution-front 索引仍可接受，而 $j_e$ 对应的终端约束使 acados 求解失败。

准确的分支语义是：

```text
solver failure → current gate accepted → saved κ=(0.05,0)
```

不能写成“求解成功后的 terminal rejection”。统计为：

- 4 周期因历史不完整而停车；
- 1284 周期进入 `ENFORCE_SOLVER_FAILED_RECOVERY` 候选分支；
- 其中 734 周期的 $\kappa$ 真实发布到 `/cmd_vel`；
- 随后 550 周期被既有 `TRACKING_UNSAFE_PROJECTION` 安全门覆盖为零；
- command-contract violation 为 0。

`PhaseRejoinDebug.recovery_command_used=true` 表示 phase 模块提出了恢复命令；只有与同 `cycle_id` 的 `ControlCycleAudit.published_cmd_*` 连接后，才能判断是否真实发布。本报告的 734 是连接后的数字。

### 8.7 S4c：gate 外停车

包：

```text
S4c_enforce_stop/S4c_enforce_stop.bag
```

结果：

- 最初 4 周期历史不完整并停车；
- 后续 527 周期 execution-front 预测有效；
- 当前 gate 与终端 gate 均拒绝；
- 527 个 `ENFORCE_SOLVER_FAILED_STOP`；
- 共 531 个 controlled-stop 决策；
- 531/531 周期实际发布 `(0,0)`；
- recovery、command-contract violation 和其他 safety override 均为 0。

## 9. 仿真能够和不能证明什么

本轮仿真直接证明：

- 模块可以独立开关；
- monitor 不改变 OCP 或命令；
- enforce 只在完整执行前沿合同下工作；
- nominal-relative acados 参数和终端 gate 实际生效；
- residual、保存策略和停车分支均可触发；
- 既有安全门能覆盖 phase 恢复；
- typed diagnostics 足以区分“提出命令”和“实际发布命令”。

本轮仿真不能证明：

- 250–320 ms 总 lead 的实液预测可信；
- 真实 Scout 上第一拍命令能在终端产生可检测液体作用；
- development gate 能预测真实可恢复性；
- false-accept/false-reject 达到论文要求；
- 相比 OfflineSloshOCP 回放或 residual MPC 有防晃增益；
- 鲁棒不变性、递归可行性、安全证书或 recovery funnel；
- RAL 主实验有效。

特别地，S4 的 wide/hybrid/narrow gate 是人为构造的分支测试输入，不是数据学习或 held-out 验证所得的恢复集合。

## 10. 用户实物验证顺序

实物测试必须逐门放行。任何一步失败，都不能通过放宽 gate、扩大残差或启用 development override 绕过。

### H0：静态安全与命令合同

先不装液体或把轮子架空：

1. 确认急停、遥控接管、安全架和现场人员分工；
2. `phase_rejoin=off` 启动，核对 `/cmd_vel` 唯一发布者；
3. 记录 `u_opt`、最终 `u_pub` 和真实轮速/odom；
4. 检查 post-solver limiter、底盘固件限幅和 watchdog；
5. 人为触发一次 solver failure，确认最终发布零命令；
6. 不允许出现时间戳倒退、frame 错误或 command-contract violation。

停止条件：底盘非预期动作、急停无效、多个命令发布者、命令限幅未进入合同或任何状态时间异常。

### G0-A：执行延迟稳定性

在低速度、安全架或轮子离地条件下，分开激励线速度和角速度。跨多个 trial、电量区间和预定地面采集：

- 最终发布 `/cmd_vel`；
- 轮速/odom/IMU；
- 电量、地面、载荷和 trial ID；
- $d_v,d_\omega,\tau_v,\tau_\omega$ 的估计及置信区间。

必须在采集前冻结允许的漂移范围。若跨条件变化超出执行模型合同，应分合同建模或停止 Phase-Rejoining，不能只取平均延迟。

### G0-B：250–320 ms 总 lead 预测

使用 held-out trial，从源时间戳状态经真实命令历史预测至：

$$
t+d_f+T_\ell.
$$

至少报告：

- robot state 和液体状态的 lead-dependent RMSE/P95/max；
- 液体主模态幅值与相位误差；
- gate 各维误差相对半径的比例；
- prediction invalid、partial history 和 deadline miss 比例。

通过条件必须由预注册的 gate margin 和测量噪声决定。若 250–320 ms 误差已经超过终端 gate 的保守余量，停止该路线，转向降低延迟、增加真实液体校正或离线前馈＋低频纠偏。

### G0-C：第一拍控制灵敏度

从相近机器人/液体初态做配对试验，只改变第一拍 $\delta u_0$，后续执行同一安全尾段。检查 $t+d_f+T_\ell$ 的液体状态差异：

- 效应方向是否与模型一致；
- 置信区间是否排除零；
- 效应是否大于 observer、IMU 和视觉测量噪声；
- 是否存在 residual authority 内不可控的区域。

若第一拍作用不可检测，短时 residual MPC 没有足够控制权，不能启用 enforce。

### G0-D：processed IMU 主频幅相

当前容器第一模态约为 `4.97 Hz`。用独立传感器或视觉液面参考检查 processed IMU 在主频附近的：

- 增益误差；
- 相位延迟；
- bias、饱和和温漂；
- 时间戳与 observer epoch。

幅相容差必须先冻结；不通过时不能把 processed-IMU 状态用于 recovery 判定。

### H1：正式 OfflineSloshOCP artifact

实物不得使用本报告中的三个 `development_only` CSV。正式 artifact 必须：

1. 由完整 OfflineSloshOCP 序列生成；
2. 冻结路径、容器、液位、载荷、执行模型、约束和 config hash；
3. 保存名义 `u_opt/u_pub`、完整状态、gate 和 $\kappa_i$；
4. 使用独立 held-out 数据计算 coverage、false-accept、false-reject；
5. 对完整尾段做非线性仿真和实物回放验证；
6. 证据仍不足时标记 `empirical_held_out`，不得标记 certificate。

### H2：实物 monitor

满足 G0 和 H1 后，先运行 monitor。示例：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_slosh \
  phase_rejoin_mode:=monitor \
  phase_rejoin_artifact_path:=<正式_artifact.csv> \
  phase_rejoin_required_contract_id:=<冻结_contract_id> \
  phase_rejoin_required_frame_id:=map
```

monitor 阶段要求：

- `command_intervened` 永远为 false；
- 与 off 条件逐周期命令一致；
- $j,j_f,j_e$ 单调且在 artifact 内；
- prediction status、gate metric、coverage、误判均可解释；
- 每个接受/拒绝事件都能与独立真实液面结果连接；
- `state_complete_for_certificate=false` 是当前预期值。

### H3：用户审核后才允许 enforce

只有用户审阅 G0、正式 artifact 和 monitor 报告并明确确认后，才可低速启用：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_slosh \
  shared_linear_accel_limit_enable:=false \
  shared_angular_limit_enable:=false \
  delay_phase_mode:=fixed_closed_loop \
  delay_phase_require_complete_history:=true \
  phase_rejoin_mode:=enforce \
  phase_rejoin_artifact_path:=<正式_artifact.csv> \
  phase_rejoin_allow_development_artifact_in_enforce:=false \
  phase_rejoin_required_contract_id:=<冻结_contract_id> \
  phase_rejoin_required_frame_id:=map
```

第一轮 enforce 应在安全架/轮子离地完成 command contract，再使用低速度、小残差、充足防溢余量和固定路径落地。未经用户确认，不运行地面 enforce。

### 实物统一停止条件

出现任一项立即撤销方法并受控停车/急停：

- artifact、frame、dt、path length 或 contract ID 不一致；
- warm-up 后仍有 partial history、prediction invalid 或 epoch 不一致；
- command-contract violation 或未建模 post-solver limiter；
- 相位索引倒退、异常跳跃或越出 artifact；
- 当前真实状态在 gate 外却提出恢复命令；
- false-accept，即 gate 判断可恢复但真实液体/尾段失败；
- 液面接近预设 spill margin；
- 跟踪、碰撞、定位、传感器或人员安全门介入；
- 控制周期连续超时或 acados 连续失败；
- 任何与仿真不同且无法解释的底盘响应。

## 11. 证据和运维注意事项

- ROS launch 日志可能序列化完整 shell 环境。归档或上传本轮日志前必须检查并移除凭据；若某个访问令牌已经进入日志，应立即轮换，而不是把日志直接提交到 Git。
- 本轮 rosbag 与 artifact 位于 `/data/a`，没有加入仓库；报告保存了路径与 SHA-256。
- `docs_for_offlineslosh/防晃领域论文梳理/arXiv/` 未被本轮修改或清理。
- 本轮未运行实物、未提交 Git、未 push。

## 12. 最终判断

模块化实现和 proxy 机制验证已经完成，代码能够在同一框架内清楚地区分：

```text
正常残差修正 / 保存策略恢复 / gate 外停车 / 既有安全门覆盖
```

但实物研究仍卡在有意义的科学放行门，而不是代码接口：

1. 250–320 ms 总 lead 是否仍可预测；
2. 第一拍控制是否对终端液体状态有可检测作用；
3. 正式 OfflineSloshOCP artifact 和 held-out empirical gate 是否成立。

三项未通过前，当前成果应视为 **development simulation release**，实物保持 `off/monitor`，不启用 `enforce`。
