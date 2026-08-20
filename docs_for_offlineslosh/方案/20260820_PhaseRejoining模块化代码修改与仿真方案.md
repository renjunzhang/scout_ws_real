# Phase-Rejoining 模块化代码修改与仿真方案

- 日期：2026-08-20
- 目标包：`src/scout_apps/control/spmpc_local_planner`
- 方法依据：`docs_for_offlineslosh/Methods/Methods章节组织思路.md`

## 1. 本轮目标和证据边界

本轮把现有滚动 S-MPCC 扩展为一条可独立开关、可审计的 phase-rejoining development release，完成：

1. 线、角双通道执行前沿的正确传播；
2. 冻结名义序列 artifact 的严格加载和合同检查；
3. 有限离散相位候选选择，不允许全局跳转或自由时间伸缩；
4. 相位索引的经验恢复 gate 与可执行恢复动作；
5. nominal-relative 短时液体代价、残差控制代价和终端经验 gate；
6. `off / monitor / enforce` 三种运行模式和逐周期诊断；
7. 单元测试、指定 proxy 仿真环境中的回归与主动模式测试。

本轮只允许称为：

> **execution-front-aligned phase-rejoining residual S-MPCC with an empirical recovery gate**

不称 recovery funnel/certificate，不宣称递归可行性。原因是扰动集合、逐拍鲁棒前驱包含、终端正不变性和 tube-policy 移位证明尚未完成。

## 2. 当前代码基础和缺口

现有包已经具备：

- `CommandHistoryBuffer`：记录最终发布的 `/cmd_vel`，包含限幅标志；
- `ExecutionStatePredictor`：利用命令历史传播机器人和液体状态；
- `ControlCycleTimingDebug`：统一机器人、液体和求解器输入 epoch；
- 10 状态 acados S-MPCC：输出完整 `PredictedHorizon`；
- 发布链限幅、solver failure、tracking safety 和 terminal safety；
- processed-IMU/odom 双 observer 及 fail-closed 选择。

缺口是：

- 当前执行预测对不同的 $d_v,d_\omega$ 仍使用同一个命令采样时刻；
- 执行前沿状态的时间戳没有明确落在未来前沿；
- 没有冻结名义序列和恢复对象的运行时合同；
- 没有 bounded phase candidate、$j_f/j_e$ 或 nominal-relative OCP 参数；
- solver failure 只会归零，没有 artifact 绑定的恢复动作；
- 没有区分 empirical gate 与 robust certificate 的诊断字段。

## 3. 模块划分

所有方法逻辑放在 ROS 无关的 `phase_rejoin/` 目录；ROS 层不得自行复制候选选择或 gate 公式。

| 模块 | 单一职责 | 不负责 |
|---|---|---|
| `NominalSequenceArtifact` | 严格读取、校验、索引冻结名义序列 | ROS 参数、在线选择 |
| `PhaseCandidateSelector` | 构造有限候选并选择当前索引 $j$ | 求解器调用、命令发布 |
| `EmpiricalRecoveryGate` | 计算相位索引的归一化终端误差和 membership | 宣称鲁棒保证 |
| `PhaseRejoinCoordinator` | 组织 $j,j_f,j_e$、nominal horizon、gate 和恢复决策 | 读取文件、发布 ROS 消息 |
| `ExecutionStatePredictor` | 双通道延迟/惯性到共同执行前沿 | 相位选择和恢复判定 |
| `SpmpcLocalPlannerROS` | 参数、生命周期、数据适配和安全优先级 | 重复核心算法 |
| acados wrapper/model | nominal-relative stage cost 和 terminal empirical gate | artifact 文件解析 |

依赖方向固定为：

```text
artifact loader ─┐
candidate selector├→ coordinator → ROS adapter
empirical gate ───┘        ↓
                    SolverInput phase context
                              ↓
                        acados wrapper
```

核心模块不能依赖 `roscpp`、ROS message 或 parameter server，以便直接做 GTest 和离线 rollout。

## 4. 冻结 artifact 合同

### 4.1 文件格式

采用带元数据注释的 CSV，schema 固定为 `phase_rejoin_empirical_v1`。元数据至少包含：

```text
schema
evidence_level
source
contract_id
frame_id
dt
path_length
```

每个采样行至少包含：

```text
index,t,s,x,y,yaw,v,omega,
eta_x,eta_x_dot,eta_y,eta_y_dot,
a,alpha,v_s,u_pub_v,u_pub_omega,
kappa_v,kappa_omega,
r_x,r_y,r_yaw,r_v,r_omega,
r_eta_x,r_eta_x_dot,r_eta_y,r_eta_y_dot
```

其中：

- `u_pub_*` 是 nominal 最终发布命令；
- `kappa_*` 是 gate 内求解失败时可执行的经验恢复动作；
- `r_*` 是经验椭球的对角尺度；
- `evidence_level` 本轮只接受 `development_only` 或 `empirical_held_out`，不接受 `robust_certificate`；
- 所有行必须有限，`index/t/s` 单调，半径严格为正，采样周期与元数据一致。

### 4.2 合同失效

以下任一项不一致时，`enforce` 模式初始化失败；`monitor` 模式只发布不可用状态且不干预命令：

- schema、frame、dt 或 path length 不一致；
- artifact 样本数不足以覆盖 $j_e$；
- path/model/container/config contract ID 不一致；
- 文件含 NaN、重复索引、时间倒退或非正 gate 半径；
- artifact 只来自 development replay，却被配置成硬件 formal release。

末端只允许在 artifact 明确提供的 hold 区间内 clamp；不把最后一行经验 gate 无限延拓。

## 5. 执行前沿修正

共同叙事前沿为：

$$
d_f=\max(d_v,d_\omega),\qquad n_f=\lceil d_f/\Delta t\rceil.
$$

从当前时刻传播到 `now + d_f` 时，每个积分时刻 $t+s$ 分别读取：

$$
u_v^{\mathrm{pub}}(t+s-d_v),\qquad
u_\omega^{\mathrm{pub}}(t+s-d_\omega).
$$

不得再从统一的 `t-d_f` 同时读取线、角命令。可选一阶惯性为：

$$
v_{k+1}=v_k+\left(1-e^{-\Delta t/\tau_v}\right)(v_k^{\mathrm{target}}-v_k),
$$

角速度同理。`tau=0` 保留纯延迟兼容模式。

预测结果增加明确的 `prediction_epoch`；闭环应用时 solver input epoch 必须设置为该未来时刻，而不是调用 predictor 的当前时刻。

## 6. 有限相位重接

候选 $j$ 始终代表“候选当前索引”：

$$
j_f=j+n_f,\qquad j_e=j+n_f+N_\ell.
$$

候选集仅由上一接受索引附近的窗口构成：

```text
expected = min(last_accepted + 1, M)
candidates = [expected - r_back, expected + r_forward]
```

并满足：

- 不小于上一接受索引；
- 必须保留正常移位候选；
- 第一周期只搜索 artifact 起点附近的有限前缀；
- 不做全局 nearest-neighbour；
- 不优化连续 $\tau$ 或 $\dot\tau$。

候选代价在 $j_f$ 坐标下联合比较机器人、液体和几何误差。几何位置只用于代价的一部分，不能单独决定防晃相位。

## 7. 经验恢复 gate

本轮使用相位索引的对角内椭球：

$$
m_i(e)=
\sum_q\left(\frac{e_q}{r_{q,i}}\right)^2,
\qquad
e\in\widehat{\mathcal R}^{\mathrm{emp}}_i
\iff m_i(e)\le1.
$$

误差维度为：

```text
x, y, yaw, v, omega, eta_x, eta_x_dot, eta_y, eta_y_dot
```

该状态尚未显式包含完整 actuator/delay buffer，因此只能叫 empirical gate v1。诊断必须发布 `state_complete_for_certificate=false`。

在线 optimizer 从执行前沿状态开始，固定液体窗口 $N_\ell=3$。在 stage $N_\ell$ 施加：

$$
m_{j_e}(e_{N_\ell|t})\le1.
$$

同时对 stage $k$ 使用 $j_f+k$ 的 nominal：

- 液体代价改为 $z-\bar z$；
- $v/\omega$ 和 $a/\alpha/v_s$ 增加 nominal-relative 项；
- solver 输出速度命令相对 `u_pub_nominal(j)` 施加 residual authority clamp。

若生成的 acados solver 尚未包含上述参数或 gate 约束，`enforce` 必须 fail closed，不能退化成只在 ROS 层打标签。

## 8. 三种运行模式

| 模式 | artifact/候选/gate | 是否改 solver input | 是否改命令 |
|---|---|---|---|
| `off` | 不运行 | 否 | 否 |
| `monitor` | 运行并发布诊断 | 否 | 否 |
| `enforce` | 运行且所有合同必须通过 | 是 | 是 |

`monitor` 用于验证索引稳定性、gate coverage 和 false-accept 统计；它不能作为方法 efficacy 结果。

`enforce` 的前置条件为：

- slosh acados mainline backend；
- `delay_phase=fixed_closed_loop`；
- 完整命令历史和未来 execution-front epoch；
- artifact 合同通过；
- 固定 $N_\ell$ 不超过生成 solver horizon；
- post-solver limiter 要么关闭，要么纳入执行合同并 fail closed。

## 9. 求解失败和安全优先级

优先级保持：

```text
人员/碰撞硬安全
→ 平台和命令执行合同
→ empirical recovery gate
→ nominal adherence 和进度
```

`enforce` 模式下：

1. 正常求解且 terminal gate 通过：发布 residual-clamped 第一拍；
2. 求解失败，但当前实际误差已在当前相位 gate 内：发布该相位保存的 `kappa`；
3. terminal gate 拒绝、当前 gate 外、artifact/时间戳/命令合同失效：受控归零；
4. terminal、tracking safety 或紧急安全门触发：始终覆盖 phase recovery。

不能因为上一周期预测未来会进入 gate，就在当前 gate 外调用恢复动作。

## 10. 代码改动清单

### 10.1 新增核心文件

```text
include/spmpc_local_planner/phase_rejoin/types.h
include/spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h
include/spmpc_local_planner/phase_rejoin/phase_candidate_selector.h
include/spmpc_local_planner/phase_rejoin/empirical_recovery_gate.h
include/spmpc_local_planner/phase_rejoin/phase_rejoin_coordinator.h

src/phase_rejoin/nominal_sequence_artifact.cpp
src/phase_rejoin/phase_candidate_selector.cpp
src/phase_rejoin/empirical_recovery_gate.cpp
src/phase_rejoin/phase_rejoin_coordinator.cpp
```

### 10.2 修改现有文件

- `core/types.h`：增加 phase nominal horizon 和输出诊断对象；
- `ros/delay_phase_types.h`、`execution_state_predictor.*`：双通道采样、惯性参数和前沿 epoch；
- `continuous_mpcc_solver_acados.cpp`：注入逐 stage nominal/gate 参数；
- `scripts/acados/*`：增加 residual cost 和 empirical terminal gate，重新生成 slosh solver；
- `spmpc_local_planner_ros.*`：加载参数/artifact、调用 coordinator、执行安全优先级；
- `DiagnosticsPublisher`：发布 typed phase-rejoin 诊断；
- `common.yaml` 和 launch：增加显式开关，默认 `off`；
- `CMakeLists.txt`：新模块、消息和测试接线。

### 10.3 开发 artifact 工具

新增只用于仿真/开发的 recorder，将同一 `cycle_id` 的 `PredictedHorizon` 与 `ControlCycleAudit` 合并为 `development_only` artifact。工具名和文件元数据必须明确它不是 OfflineSloshOCP，也不能用于论文主结论或实物 formal release。

正式 OfflineSloshOCP 后续必须输出同一 schema，在线控制器无需改代码即可替换 artifact 来源。

## 11. 单元和静态测试

至少新增：

1. 不同 $d_v,d_\omega$ 时两通道读取不同历史命令；
2. 一阶惯性极限和 `tau=0` 兼容行为；
3. prediction epoch 等于 `now+d_f`；
4. artifact 缺列、NaN、索引/时间倒退和非正半径拒绝；
5. 候选集合有界、单调并保留正常移位候选；
6. $j_f/j_e$ 不重复计算 delay；
7. gate 边界内/边界外和 yaw wrap；
8. monitor 永不改变命令；
9. enforce terminal accept、terminal reject、solver failure recovery 和 gate 外归零；
10. phase mode 未开启时现有控制输出保持回归一致。

构建顺序保持串行：

```bash
catkin build spmpc_local_planner --no-deps
catkin run_tests spmpc_local_planner
catkin_test_results build/spmpc_local_planner
```

## 12. 指定 proxy 仿真验收

环境必须按 `单独仿真环境启动方法.md` 的 environment-only 边界运行。推荐入口为：

```text
/data/a/scout_sim_replacement/scripts/launch_proxy_sim_localization_env.sh
```

该入口只是设置 `START_PATH_PUBLISHER=false`、`START_SPMPC=false` 等默认值后，`exec` 到底层
`launch_proxy_sim_localization_spmpc.sh`。本轮实际验收直接调用了这个底层脚本，但显式设置：

```text
START_PROXY=true
START_LOCALIZATION=true
START_PATH_PUBLISHER=false
START_SPMPC=false
```

因此实际仍是 environment-only：环境进程没有启动路径发布器、`spmpc_local_planner` 或 SIM-R8；路径和控制器随后以独立进程接入。最终报告必须记录这一入口差异。不得使用 `simulation/spmpc_sim_local_planner`，也不得混用 SIM-R8。

### S0：环境边界

按 `单独仿真环境启动方法.md` 无界面启动，显式设置四个 `START_*`。验证 `/map`、`/odom`、`/scan_front`、`/imu/data`、`map→base_link`，并确认 `/cmd_vel`、两条 path topic 无发布者。

### S1：旧行为回归

接入 path 与原 `phase_rejoin=off` 控制器，验证：

- controller 初始化成功；
- `/cmd_vel` 只有预期发布者；
- 机器人沿 S 路径前进并达到终点或通过冻结运行窗口；
- 无 NaN、solver 连续失败、tracking safety 锁死和异常进程退出。

### S2：development artifact

在 S1 中运行 recorder，生成临时 `development_only` artifact；离线 validator 必须通过。该文件只用于接口/闭环机制 smoke。

### S3：monitor

重启同一环境和同一路径，以 artifact 运行 `phase_rejoin=monitor`：

- 命令与 off 模式不因 phase 模块被改写；
- $j,j_f,j_e$ 单调且均在 artifact 范围；
- 诊断明确 `empirical`、`command_intervened=false`；
- 统计 candidate count、gate accept/reject 和 metric。

### S4：enforce

只在 S0–S3 通过后运行：

- `delay_phase=fixed_closed_loop` 且完整 history 后才允许 active；
- residual clamp、terminal gate 和恢复策略各至少触发一次可解释分支；
- 不可用 artifact、history 不完整或 gate 外状态均 fail closed；
- 正常路径 smoke 中无异常急停、索引跳跃和 deadline 连续丢失。

每次只启动一套环境和一个控制器。停止时仅向本次脚本/session 发送 `Ctrl-C`，不用 `killall/pkill`。

## 13. 实物前交付边界

本轮不运行实物。报告给用户的实物待测表必须包含：

- G0 的 250–320 ms total-lead prediction 与 first-action sensitivity；
- 执行模型在跨 trial、电量、地面上的稳定性；
- artifact 的真实 OfflineSloshOCP 来源和 matched-time 证据；
- empirical gate held-out false-accept/false-reject；
- processed-IMU 在主频附近的幅值/相位误差；
- 小速度、轮子离地或安全架上的 command contract 检查；
- 实物 `monitor` 先行，未经用户确认不切 `enforce`。

## 14. 最终报告结构

报告写入 `docs_for_offlineslosh/代码修改报告`，逐项给出：

1. 实际新增/修改文件与模块依赖；
2. 方案项到代码/测试证据的映射；
3. 构建和 GTest 原始结果；
4. S0–S4 的命令、日志目录、topic 和运行结果；
5. 未通过或未实施项；
6. 不能宣称的理论/实验结论；
7. 用户实物验证的顺序和停止条件。

只有代码、指定仿真和报告三者都有直接证据后，本轮任务才算完成。
