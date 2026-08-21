# Phase-Rejoining 正式方法闭环实施方案

- 日期：2026-08-21
- 目标包：`src/scout_apps/control/spmpc_local_planner`
- 方法依据：`docs_for_offlineslosh/Methods/Methods章节组织思路.md`
- 当前开发阶段记录：`docs_for_offlineslosh/方案/20260820_PhaseRejoining模块化代码修改与仿真方案.md`

## 0. 先说结论

需要继续模块化解耦，但不再做无边界的大规模重构，也不以“拆出更多文件”为目标。

下一阶段应做一次**纵向贯通的正式闭环收尾重构**：围绕最终发布命令 $u^{\mathrm{pub}}$，依次闭合预计发布时间、双通道执行模型、delay-augmented OCP、正式离线 artifact、held-out gate、仿真 plant 和实物证据链。

一句话概括目标架构：

> 从同一个最终发布命令出发，让在线求解、离线名义序列、仿真执行 plant、实物标定和事后 replay 使用同一套执行模型与时间合同。

当前不是从零开始。相位候选、9 维经验 gate、保存动作、`off/monitor/enforce`、typed diagnostics 和 development 分支测试已经存在，应复用。尚未完成的是让这些模块具备正式方法所要求的因果性、可复现性和实物证据。

大致要做八件事：

1. 冻结当前可复现基线，先用测试钉住现有行为；
2. 把最终命令收敛为唯一发布出口，发布成功后再更新 history 和相位状态；
3. 增加 $\widehat d_c$、双通道 pending-command buffer、执行器状态和非整数延迟；
4. 新增正式 delay-augmented Phase-Rejoin solver，并用独立执行 plant 验证因果性；
5. 用 typed session、严格 preflight、运行时合同和 postflight 闭合证据链；
6. 在 Scout 上完成执行模型、时钟、IMU/RGB 和总 lead 的 G0 标定与放行；
7. G0 通过后生成正式 OfflineSloshOCP artifact、执行兼容集和 held-out empirical gate；
8. 完成正式 C0–C4 仿真、实物 monitor，再经人工审核进入低速 enforce。

旧方案记录的是 development release，不修改其历史结论。本文只描述从当前状态走到 formal-ready 的增量工作。

2026-08-21 实施进度：WP0 已冻结，WP1 已闭合唯一最终命令事务；WP2A 已建立预计发布时间模型和实际 $d_c$/deadline typed audit；WP2B 已建立统一双通道执行增广参考模型，并让 history predictor 复用同一合同和传播实现；WP2C 已让同一个 typed `PublishEpochEstimate` 驱动 history prediction、PhaseClock 和 `SolverInput`，并对完整 estimate image 做周期一致性校验；WP3A 已新增 solver 专用 `ExecutionHorizonContext` 和纯 C++ `DelayAugmentedPhaseDynamics`，让当前 $q=[a,\alpha,v_s]$ 从上一真实发布命令生成新 $u^{\mathrm{pub}}$、压入双通道 buffer，并冻结 $N_e=n_f+N_\ell$ 及 physical/grid/terminal epoch；WP3B 已生成确定性 CasADi C 离散转移核，并通过随机单步、第一拍 Jacobian 和 10 步 terminal Jacobian 与 C++ 参考模型的一致性验证；WP3C 已使用同一转移生成并实际编译独立 `DISCRETE nx=22,nu=3,N=10` acados capsule，加入 published-command、robot/pending speed 和 rate 硬约束，并以严格 hash/dimension/capability gate 拒绝 formal 提前放行；WP3D 又新增严格的完整 history alignment 和 frozen `ExecutionHorizonContextBuilder`，从 source-stamped robot/liquid state、真实发布历史及有效 expected-publish estimate 构造 actuator state、双通道 pending buffer 和统一 epoch，并以 opt-in 方式写入 `SolverInput.execution_horizon`。WP3D 的 590 项 C++ / 96 项 Python 回归见 `20260821_PhaseRejoining_WP3D在线执行增广初态构造记录.md`。新 capsule 仍未加入在线 factory/config，现有默认行为不变；它也尚无 formal nominal-relative cost/parameters、terminal 9D gate/$\mathcal B^{\mathrm{exec}}$，ROS/formal session 尚未激活新 builder。默认 `publish_timing.enabled=false`，$\widehat d_c$ 尚未由标定 artifact 冻结，独立 plant 也尚未完成，因此 WP2、WP3、B0 和 formal 放行均未关闭，状态仍为 G0 NO-GO。

## 1. 当前状态与剩余缺口

### 1.1 已经完成、应直接复用的部分

| 能力 | 当前状态 | 后续处理 |
| --- | --- | --- |
| `phase_rejoin/` 核心模块 | 已解耦于 ROS | 保留，不重写 selector、9 维 gate 和 coordinator |
| 有限、单调的相位候选 | development 测试已覆盖 | 补正式 epoch 与 artifact 合同，不改基本语义 |
| 双通道 history predictor | 能传播历史命令到共同前沿 | 保留给 `off/monitor` 和兼容诊断，不能作为 formal OCP 的固定前沿 |
| 9 维经验 gate | 可做分支 smoke | 保留 evaluator，重新构造正式半径和 held-out 证据 |
| 保存恢复动作 | 已能经过现有命令链发布 | 正式 artifact 中重新生成并验证 |
| `off/monitor/enforce` | 三种运行模式已接线 | 在 formal 合同完成前，实物 `enforce` 继续锁死 |
| typed diagnostics/audit | 可按 `cycle_id` 连接主要信息 | 增加预计/实际发布时间、执行增广状态和 session hash |
| S0–S4 proxy | 接口和分支已跑通 | 只作为 development 证据，不作为防晃性能证据 |

### 1.2 正式方法尚未闭合的部分

| ID | 缺口 | 当前风险 | 完成定义 |
| --- | --- | --- | --- |
| IMP-01 | 最终命令出口已归一（WP1 已闭合） | receipt 目前只证明 ROS publisher 接受交付，不是 Scout CAN/底盘 ACK | 每周期只有一次 finalization 和一次 sink 调用；history、audit、相位提交与 receipt 声明的 $u^{\mathrm{pub}}$ 一致；更强 ACK 由 WP4/WP5 闭合 |
| IMP-02 | 预计发布时间合同、统一执行参考模型和在线 typed 接线（WP2A–WP2C）已建立；有效 estimate 已统一驱动 history prediction、PhaseClock 和 `SolverInput` | 默认估计仍关闭；$\widehat d_c$ 尚未由 Scout held-out 标定 artifact/hash 冻结，当前接线仍是 history-only | 冻结 $\widehat d_c$、适用域和 hash，并由 formal session/preflight 强制绑定；当前已记录实际 $d_c$、误差和 deadline |
| IMP-03 | WP3C 已让 $q=[a,\alpha,v_s]$ 的新 $u^{\mathrm{pub}}$ 决策依赖进入独立 acados DISCRETE optimizer 的双通道 buffer，并对积分后 $u^{\mathrm{pub}}$ 建立硬边界；WP3D 已能把完整 history augmented context 写入 typed `SolverInput` | 该 capsule 尚未由在线 factory 调用，也没有 formal stage/terminal 参数 | 本周期新命令作为求解决策量进入线/角两路 delay buffer |
| IMP-04 | C++ 参考模型、`ExecutionHorizonContext`、CasADi 离散转移与独立 acados capsule 已统一整步/fractional delay、$n_f$ 及 physical/grid/terminal epoch；WP3D 已从真实发布序列构造两路 pending buffer 和 expected-publish 初态 | ROS/formal session 尚未激活 builder，artifact index 也尚未消费该合同 | 让同一 fractional-delay 合同同时驱动 physical epoch、solver stage 和 artifact index |
| IMP-05 | WP3C 已生成并编译 $N_e=n_f+N_\ell$、`nx=22`、`nu=3` 的独立 acados optimizer，并以 capability gate 明确标记已实现硬约束 | formal nominal-relative cost/parameters、terminal 9D gate 和 $\mathcal B^{\mathrm{exec}}$ 尚未进入；现有在线短窗 capsule 仍未替换 | 新增专用生成物；终端同时支持 9 维 gate 和执行兼容约束 |
| IMP-06 | 没有正式 OfflineSloshOCP artifact | development CSV 不是完整离线防晃序列 | 输出运动、减速、沉降、zero-hold 完整尾段，并冻结全部合同和 hash |
| IMP-07 | 没有 $\mathcal B_i^{\mathrm{exec}}$ | 9 维 gate 不知道 pending command 和执行器状态是否兼容 | 当前相位与 OCP 终端都检查逐相位执行状态硬边界 |
| IMP-08 | gate 没有 held-out 证据 | development 半径不能说明真实可恢复性 | trial 级数据隔离并报告 coverage、false-accept、false-reject 和真实重接率 |
| IMP-09 | formal runner/证据链不完整 | 关键参数散落在 launch、环境变量和大型 Shell 中 | typed session 是唯一真值；preflight、runtime ACK、recorder、postflight 全部闭合 |
| IMP-10 | 正式仿真 plant 不独立 | 零延迟或瞬时跟随 proxy 只能证明接线 | $u^{\mathrm{pub}}$ 必须经过非零 delay、$\tau/K$、死区、饱和和独立液体 plant |
| IMP-11 | G0 实物放行未完成 | 总 lead、第一拍灵敏度和 IMU/RGB 幅相尚未证明 | 完成 held-out 标定并给出明确 GO/NO-GO |

因此，当前状态应称为：

> **development simulation release；formal 实物闭环仍为 NO-GO。**

## 2. 重构边界

### 2.1 继续做什么

继续做面向正式闭环的定向解耦：

- 把执行时间、执行动力学和增广状态从 ROS wrapper 中抽成无 ROS 的 C++ 核心；
- 把最终命令、安全检查、发布回执、history 写入和相位提交组织为一个命令事务；
- 让 solver 只消费 typed augmented context，不自行解析 artifact 或 ROS 参数；
- 让 artifact loader 成为唯一 schema/hash 校验入口；
- 把正式运行参数收敛成不可变的 `ResolvedExperimentSession`；
- 把实物 runner 缩成薄 Shell，在线合同、preflight 和 supervisor 使用 C++；
- 保留 Python 做 CasADi/acados codegen、rosbag 提取、RGB 分析、统计和绘图。

### 2.2 不继续做什么

以下工作不应阻塞正式方法：

- 不为了降低单文件行数继续横向拆目录；
- 不拆成多个新的 catkin package；
- 不先重命名所有 `core/solver/solvers` 历史目录；
- 不重写已经通过测试的 selector、development gate 或 diagnostics；
- 不把所有 Python 分析工具迁移到 C++；
- 不增加通用 plugin/factory 框架；
- 不直接修改旧 development artifact 的 metadata 来冒充 formal artifact；
- 不同时维护两套正式延迟算法。

主路线选择**显式 delay-augmented OCP**。只有它无法满足冻结的控制 deadline 时，才允许改用凝聚 bridge；bridge 必须在随机状态/控制、第一拍 Jacobian、终端 epoch 和逐节点状态上与显式模型数值等价。

## 3. 目标架构与唯一控制流程

### 3.1 在线流程

```text
source-stamped odom / IMU / TF / command history
                         │
                         ▼
              CycleTimingContract
          预计发布时间 t̂_pub=t_c+d̂_c
                         │
                         ▼
           ExecutionModel.alignToPublishEpoch
       用既有 u_pub 历史对齐到预计发布时刻
                         │
                         ▼
             相位选择＋增广 SolverInput
                         │
                         ▼
        DelayAugmentedPhaseRejoinSolver
       当前新决策进入 v/ω 两路 delay buffer
                         │
                         ▼
       terminal 9维 gate ＋ B_exec ＋监督器
                         │
                         ▼
        CommandPipeline：唯一 finalization
                         │
                         ▼
              ICommandSink::publish
                         │
                         ▼
             PublicationReceipt
                         │
          ┌──────────────┴──────────────┐
          ▼                             ▼
  写入真实 u_pub history        phase commit＋cycle audit
```

ROS 层只负责消息转换和 `ICommandSink` 的实现，不再改写最终命令。

### 3.2 离线与证据流程

```text
冻结路径＋执行模型＋液体模型＋约束
                    │
                    ▼
           OfflineSloshOCP
                    │
                    ▼
        完整名义序列与完整尾段
                    │
                    ▼
  recovery rollout → B_exec → empirical gate
                    │
                    ▼
         held-out validation report
                    │
                    ▼
       PhaseRejoinReleaseManifest
                    │
                    ▼
 typed session → preflight → runtime ACK → bag → postflight
```

在线、离线、仿真和 actual-input replay 必须复用同一 `ExecutionModelContract`。禁止在 C++、CasADi 和 Python 中各自维护含义不同的 delay 公式。

## 4. 建议的模块边界

在现有包内扩展，不新建 catkin 包：

```text
include/spmpc_local_planner/
├── controller/
│   ├── control_loop.h
│   └── command/
│       ├── command_sink.h
│       └── publication_transaction.h
├── runtime/
│   ├── timing/
│   │   └── publish_latency_model.h
│   └── execution_prediction/
│       ├── execution_model_contract.h
│       ├── execution_augmented_state.h
│       └── execution_model.h
├── solver/api/
│   └── execution_horizon_context.h
├── solver/acados/
│   └── delay_augmented_phase_solver.h
├── phase_rejoin/
│   ├── execution_compatibility_set.h
│   └── formal_release_manifest.h
├── config/
│   ├── experiment_session_config.h
│   └── formal_method_policy.h
└── tools/
    ├── formal_preflight.h
    └── formal_postflight.h
```

名称可在实现时按现有命名习惯微调，但所有权必须固定：

| 对象 | 唯一所有者 | 禁止事项 |
| --- | --- | --- |
| $\widehat d_c$ 与预计发布时间 | `PublishLatencyModel` | ROS wrapper 私自增加固定时间偏移 |
| $d_v,d_\omega,\tau_v,\tau_\omega,K_v,K_\omega$ | `ExecutionModelContract` | 同时散落在 YAML、runner 和代码默认值中 |
| pending buffer 与 actuator state | `ExecutionAugmentedState` | 只保存 9 维状态却声称执行状态完整 |
| 最终 $u^{\mathrm{pub}}$ | `CommandPipeline + ICommandSink` | 发布后再次限幅或替换命令 |
| phase schema/hash | formal loader/manifest | solver、runner 各自解析 artifact |
| 正式运行配置 | `ResolvedExperimentSession` | 环境变量覆盖方法参数 |
| 正式 PASS/FAIL | C++ preflight/postflight | Shell 根据日志文本猜测放行状态 |

OfflineSloshOCP 生成器应是独立的 source-tree tool；在线 planner 只拥有 schema、validator 和 loader，不承担离线优化生成职责。

## 5. 工作包与实施顺序

工作包必须按依赖推进。每个工作包都形成独立、可构建、可回退的中文 commit，不做一个巨型重构提交。

```text
WP0 基线冻结
  ↓
WP1 唯一最终命令事务
  ↓
WP2 时间合同与统一执行模型
  ↓
WP3 delay-augmented solver＋独立 plant 仿真
  ↓
WP4 typed session 与正式证据链
  ↓
WP5 G0 实物模型放行
  ↓ GO
WP6 Offline artifact＋B_exec＋held-out gate
  ↓
WP7 正式仿真、monitor 与 release
  ↓
交给 C0–C4 正式实物实验
```

### WP0：冻结基线与架构决定

**目标：**防止在后续重构中丢失现有可运行行为。

实施内容：

1. 先将当前文档和分析脚本 diff 独立提交，不与执行链重构混在一起；
2. 固定当前 commit、acados 生成物、开发 artifact、配置和测试结果；
3. 保存现有 launch 展开参数、关键 bag replay 和 `off/monitor/enforce` characterization；
4. 写一页 ADR，冻结“显式增广 solver、唯一命令出口、同一执行模型、formal 严格配置”四项决定；
5. 建立缺口 ID 到模块、测试和报告的追踪表。

退出条件：

- 干净 revision 可重新生成 solver 并完成当前测试；
- `off` 和 `monitor` 的旧行为有 golden 证据；
- 所有正式关键参数只有一个计划中的权威来源；
- 当前版本仍明确标记为 formal NO-GO。

### WP1：唯一最终命令事务

**目标：**先确定真正进入 Scout 和 history 的命令真值。

实施内容：

1. 先增加 characterization test：每周期只允许一次 sink 调用，`history == audit == published command`；
2. 把所有正常、恢复、solver failure、等待和安全分支统一输出为 `CommandDecision`；
3. `CommandPipeline` 只执行一次 finite、速度、加速度、状态新鲜度、deadline 和安全检查；
4. `ICommandSink` 原样发布 `FinalCommand`，返回包含 `cycle_id` 和交付时刻的 `PublicationReceipt`；
5. 只有收到成功 receipt 后才写入 command history；
6. 把 phase `commit()` 移到 finalization 和 receipt 之后；若命令被 limiter 改写，不得按原 residual 提案推进相位；
7. diagnostics/audit 记录 proposed、finalized 和 published 三者，但只有 published 是执行模型输入。

退出条件：

- 每个控制周期只有一次最终命令出口；
- ROS wrapper 不再二次限幅或替换命令；
- 所有失败路径也通过同一出口发布受控零命令；
- phase 状态不会依据未发布命令推进；
- fake sink、limiter 改写和发布失败测试全部通过。

### WP2：预计发布时间与统一执行模型

**目标：**闭合 $d_c$、双通道延迟和当前新决策的因果传播。

核心类型：

```cpp
struct CycleTimingContract;
struct PublishEpochEstimate;
struct PublicationReceipt;
struct ExecutionModelContract;
struct ExecutionAugmentedState;
```

`ExecutionModelContract` 至少包含：

```text
dt
d_v, d_omega
tau_v, tau_omega
K_v, K_omega
每通道整步延迟和 fractional remainder
deadzone / saturation / 方向适用性
载荷、地面、电量、速度有效域
辨识误差边界、schema、contract hash
```

实施内容：

1. 以控制周期开始时刻 $t_c$ 为唯一计算起点，定义 $\widehat t_{\mathrm{pub}}=t_c+\widehat d_c$；
2. 用已发布历史和计算期间的保持命令，把 source-stamped 状态对齐到 $\widehat t_{\mathrm{pub}}$；
3. 从预计发布时刻开始，本周期新 $u^{\mathrm{pub}}$ 作为决策量压入线、角两路 pending buffer；
4. 用整步 buffer 加 fractional-delay kernel 表达非整数延迟；
5. 执行器状态显式传播 $\tau/K$，实际输出再驱动车体和液体；
6. 由执行模型提供 `requiredHistorySec()`、`executionLeadSec()`，禁止 predictor、preflight 和 warm-up 各自计算；
7. development 模式可对配置 normalize 并告警；formal 模式遇到缺失、非有限、未知字段或任何需要 clamp 的值直接失败。

必须增加的诊断：

```text
expected_publish_stamp
actual_publish_stamp
estimated_dc
actual_dc
dc_error
publish_deadline_missed
execution_contract_hash
history_span / history_complete
physical_front_stamp / grid_front_stamp / terminal_stamp
```

退出条件：

- impulse、step、partial-history、fractional-delay 和跨通道测试通过；
- 当前新线速度在共同前沿前产生正确影响，角速度仍按较慢通道传播；
- $13.3\,\mathrm{ms}$ 余量被显式建模或严格映射，不再被忽略；
- C++ 参考模型与 CasADi/generated 模型在随机状态和控制上逐步一致；
- history 不完整、epoch 倒退或错合同一律 fail closed。

### WP3：delay-augmented solver 与独立执行 plant

**目标：**解决 B0，并先在仿真中证明控制量真的经过延迟链影响联合终端。

2026-08-21 的 WP3A 已完成本节第 2–4、6 项所需的纯 C++ 参考转移和 typed horizon 骨架：当前决策按 published-command rate 语义进入双通道 buffer，horizon 固定为 $N_e=n_f+N_\ell$，并有第一拍线/角因果与联合终端灵敏度测试。WP3B 又建立了 `nx=22`、`nu=3`、`N_e=10` 的确定性 CasADi C 离散转移图像，完成 128 组随机单步、第一拍 Jacobian 和 terminal Jacobian 与 C++ 参考的一致性。WP3C 已以同一转移生成和编译独立 acados DISCRETE optimizer，加入 $q$、robot/pending state 和积分后 $u^{\mathrm{pub}}$ 硬约束，并可以消费通过严格合同校验的完整 augmented initial context。WP3D 已新增 `ExecutionModel::alignPublishedHistory()` 和 frozen `ExecutionHorizonContextBuilder`：用真实 receipt 后 history 把 source-stamped robot/liquid state 对齐到 expected-publish epoch，返回 actuator output、不同基数的双通道 pending buffer，并在 estimate、history、hash、epoch 和 cardinality 全部有效时 opt-in 写入 `SolverInput`。但它仍不是 formal 在线 solver：ROS/formal session 尚未激活 builder，候选 capsule 也未进入 factory，formal nominal-relative cost/parameters、terminal 9D gate/$\mathcal B^{\mathrm{exec}}$ 和独立 plant 均未完成；capability gate 会明确拒绝 formal mask，不能据此关闭 WP3/B0。

实施内容：

1. 保留现有 development solver，新建独立的 formal Phase-Rejoin 生成物；
2. 扩展 `SolverInput`，加入预计发布时刻、增广初态、执行模型合同和双通道 buffer；
3. horizon 使用 $N_e=n_f+N_\ell$，不能只求共同前沿后的 $N_\ell$；
4. 每一步由 $q=[a,\alpha,v_s]$ 形成候选 $u^{\mathrm{pub}}$，压入双通道 buffer 后再更新执行器、车体和液体；
5. terminal 同时暴露 9 维 gate 与 $\mathcal B^{\mathrm{exec}}$ 硬约束接口；
6. phase clock、物理时刻、solver stage、artifact index 使用同一个离散合同；
7. 建立与控制器预测模型独立的仿真 plant：

```text
u_pub
  → 非零 d_v / d_omega
  → tau / K / deadzone / saturation
  → v_real / omega_real
  → robot motion
  → independent liquid plant
  → internal slosh output + external slosh monitor
```

仿真 plant 必须允许参数偏差、传感器噪声、时间抖动和 state-estimation delay，不能直接把 solver 的预测状态当真值。

退出条件：

- 第一拍线/角控制对联合终端的 Jacobian 符合各自延迟，不能出现“假零”或提前作用；
- 显式增广模型与高精度连续事件参考仿真逐节点一致；
- nominal、延迟置信区间端点和故障注入均有可重复结果；
- solver 在冻结的 30 Hz command deadline 内运行，报告 P50/P95/max 和 miss；
- recovery 分支在仿真中真实触发，最终发布命令与 plant 输入一致。

若显式增广 solver 超时，先分析稀疏结构和 codegen；仍不能满足 deadline 时，才切换到经过上述等价测试的凝聚 bridge。不能用旧 history-only predictor 包装后声称 B0 已关闭。

### WP4：typed session 与正式证据链

**目标：**让一次正式运行可以被完整复现，并在启动前拒绝不一致配置。

建议引入：

```text
ExperimentSessionConfig
ResolvedExperimentSession
ArtifactBinding
RunInvocation
RuntimeContract
```

其中 `ResolvedExperimentSession` 是唯一运行真值，至少绑定：

- 完整 planner 配置；
- 执行标定 artifact；
- phase nominal/recovery artifact；
- 路径、地图、坐标系和 hash；
- 容器、液位、载荷和 IMU 标定；
- generated solver、binary、git commit；
- recorder、启动门、安全和停止策略。

实施内容：

1. C++ strict loader 解析、校验、规范化序列化并计算 session hash；
2. formal 模式不接受方法参数的 env/CLI override，也不接受隐式默认回退；
3. C++ preflight 校验 schema、全部文件 hash、相对路径、运行模式、完整 history、solver/artifact/执行合同一致性；
4. planner 初始保持 `DISARMED` 并发布零命令；supervisor 收到 runtime contract hash ACK、传感器 READY、recorder active 和 history complete 后才允许人工 arm；
5. recorder 强制记录 phase debug、cycle audit、最终命令、odom/TF/IMU、执行反馈、RGB 索引和所有 source/receive/effective stamp；
6. C++ postflight 给出 PASS/FAIL；Python 只生成 bag 统计、图和论文报告；
7. 大型实物 runner 收缩为薄 Shell，只传 session、trial 行号、输出目录和 arm 信息。

退出条件：

- 删除、重复、未知、NaN、`Inf`、越界字段和任一 byte 的 asset 篡改都无法 arm；
- runtime hash 与 freeze manifest 不同即 fail closed；
- bag 能按 `cycle_id` 连接状态、候选、最终发布命令、执行反馈和 phase 决策；
- 不再依靠录包结束后人工发现缺 topic、缺 $\tau$ 或错 artifact；
- 同一 session 可确定性重放并产生相同 resolved semantic hash。

### WP5：G0 实物模型与因果放行

**目标：**在投入正式 gate 构造前，先证明 Scout 的总 lead 仍可预测且第一拍确实有控制作用。

G0 前先完成 Layer 0：

1. `cycle_id` 贯通 source state、OCP、候选、最终 $u^{\mathrm{pub}}$、odom/IMU 和 RGB；
2. 校准 ROS、Nokov、IMU 和 RGB 的时间偏差；
3. 证明 actual-input replay 明显优于 planned-input replay；
4. 冻结命令发布、driver watchdog 和真实停车行为。

然后按以下顺序进行：

#### G0-A：Scout 执行模型辨识

使用相同容器和载荷，通过最终 $u^{\mathrm{pub}}$、Nokov、odom 和 IMU，分开激励线速度与角速度，辨识：

```text
d_c, d_v, d_omega
tau_v, tau_omega
K_v, K_omega
deadzone, saturation, 正反向差异和通道耦合
```

覆盖预定地面、电量、速度和载荷范围，给出 trial 级点估计、置信区间和 P95 漂移。变化超出单一合同范围时应分合同或停止，不取一个平均值掩盖差异。

#### G0-B：总 lead held-out 预测

用未参与辨识的 trial，从 source time 经预计发布时间和双通道执行链预测到联合终端。至少报告 robot/液体状态 RMSE、P95、最大误差、主频幅相误差和 gate margin 占用。

#### G0-C：第一拍 terminal sensitivity

以同一预激励构造近似液体初态，只改变第一拍 residual，后续执行同一安全尾段。检查联合终端液体效应是否方向正确、置信区间排除零并高于执行和测量噪声。

#### G0-D：IMU/RGB 观测资格

- processed IMU 在约 $4.97\,\mathrm{Hz}$ 主模态附近的增益与相位必须冻结；
- RGB 同时输出非负的 `max-LCR` 和观察平面的 signed height difference；
- signed RGB 用于过零、相位、主频和衰减，`max-LCR` 用于幅值、P95 和 freeboard；
- 论文只能声称 observed-plane fidelity，不能把单视角 RGB 写成完整二维液体状态真值。

数据必须隔离：

```text
D_id       执行/液体模型辨识
D_fid      held-out 保真度
D_dev      控制器与 gate 开发
D_pilot    方差和正式样本量
D_confirm  最终结论
```

若根据 `D_fid` 修改模型或阈值，原 `D_fid` 立即降级为 `D_dev`，重新采集 held-out 数据。

G0 的总出口只有两个：

- **GO：**总 lead 误差在预注册余量内，第一拍效应可检测，IMU/RGB 时间和幅相合格；
- **NO-GO：**停止 formal Phase-Rejoin，依次考虑降低控制链延迟、增加真实液体状态校正，或降级为执行感知离线前馈＋低频纠偏。

G0 未 GO，禁止把 development artifact、宽 gate 或更大 residual 当成绕过手段。

### WP6：正式 OfflineSloshOCP、执行兼容集和 held-out gate

**入口条件：**WP0–WP5 全部通过，尤其是 G0=GO。

正式资产分成三层：

```text
NominalPlanArtifact
  路径、X_aug、q、u_pub、运动—减速—沉降—zero-hold 完整尾段

RecoveryAdmissionArtifact
  每相位 9维 gate、B_exec、保存动作、fit/tune 数据合同

PhaseRejoinReleaseManifest
  绑定前两者、执行/液体模型、路径、容器、约束、solver 和 held-out 报告 hash
```

实施内容：

1. OfflineSloshOCP 与在线 solver、仿真 plant 和 replay 复用同一执行模型语义；
2. nominal 每相位保存完整执行增广状态，而不只保存 9 维状态；
3. artifact schema 升级，不覆盖 development v2 语义；
4. 构造逐相位 $\mathcal B_i^{\mathrm{exec}}$，分别约束线/角 buffer 和 actuator state；
5. 当前相位和 OCP 终端都必须通过 $\mathcal B^{\mathrm{exec}}$；
6. recovery rollout 按完整 trial 分为 fit、tune、test，test trial 不得参与半径或阈值调整；
7. held-out 报告以 trial 为统计单位，报告 coverage、false-accept、false-reject、实际重接率、phase-bin coverage 和最差相位；
8. 验证保存动作实际经过同一安全链并被发布，不能只验证“理论上提出”；
9. 对完整尾段做非线性仿真和 held-out 实物回放。

退出条件：

- artifact 可确定性重建，全部 hash 和 schema 可追溯；
- 路径、模型、容器、约束或 solver 任一变化都会使旧 manifest 失效；
- $\mathcal B^{\mathrm{exec}}$ 外样本被单独统计，不混入 9 维 gate confusion matrix；
- false-accept 上界、coverage 和最差相位达到预注册门限；
- fixed recovery action 仍只称 `stored recovery action`，不写成反馈 $\kappa(e)$；
- 方法仍称 **phase-indexed empirical recovery gate/set**，不称 certificate 或 funnel。

### WP7：正式仿真、monitor 和 release

#### WP7-A：正式 C0–C4 仿真

使用同一冻结 commit、solver、session、artifact、路径和独立 plant，完成：

- C0–C4 对照和受控偏离；
- $d/\tau/K$ 置信区间端点、死区、饱和和传感器延迟扰动；
- recovery 分支真实触发；
- internal slosh 与 external slosh monitor 两个通道均给出正向结果；
- 完成时间、速度、加速度、jerk、路径误差和安全约束的公平性检查；
- solver 30 Hz 实时性、失败率和所有资产 hash。

这里的“正向”不能只看 controller 内部模型：独立 plant/monitor 的 motion＋tail 液面指标也必须降低。正式统计和 baseline 定义引用实验章节，不在本方案重复定义。

#### WP7-B：实物 monitor

按以下阶梯升级：

```text
轮子离地/无液体命令合同
→ 地面 dry payload
→ 低速载液 off
→ 低速载液 monitor
→ 人工审核
→ 低速 enforce rehearsal
```

monitor 必须逐周期证明：

- `command_intervened=false`；
- 与 `off` 的最终命令一致；
- phase index、front index 和 terminal index 单调且不越界；
- gate 接受/拒绝可与对应物理时刻的 RGB 结果连接；
- session、calibration、artifact 和 binary hash 全程不变。

#### WP7-C：formal release

只有仿真、G0、artifact、held-out gate、preflight、monitor 和 postflight 全部通过，且用户明确审核后，才能生成唯一 `FREEZE_ID` 并启用地面低速 `enforce`。

以下任一情况立即退回 `off` 并停止升级：

- partial history、epoch/index 异常或 $d_c$ deadline 连续超限；
- session/artifact/model/hash 不一致；
- 多个命令发布者、未建模 limiter 或 command-contract violation；
- 当前状态或 terminal 不满足 gate/$\mathcal B^{\mathrm{exec}}$ 却提出恢复；
- false-accept、液面接近 spill margin 或路径/碰撞安全门介入；
- solver 连续超时、传感器失效、时钟跳变或底盘响应超出标定域。

## 6. 最低测试矩阵

| 层级 | 必须覆盖 |
| --- | --- |
| 单元测试 | timing、fractional delay、buffer、actuator model、command transaction、phase commit 时序、session schema/hash、artifact validator |
| 随机一致性 | C++、CasADi/generated solver、离线模型在随机状态/控制上的逐步一致性 |
| mutation | 缺字段、重复字段、unknown、NaN/Inf、越界值、错 schema、任一 asset 单字节篡改 |
| replay | empty/partial/complete/stale history、时钟倒退、不同 $d/\tau/K$、artifact/路径/solver 不匹配 |
| solver | 第一拍 Jacobian、fractional delay、terminal epoch、gate 与 $\mathcal B^{\mathrm{exec}}$、deadline |
| 独立仿真 | 非零执行延迟、参数偏差、死区/饱和、传感器抖动、故障注入、C0–C4 双通道 slosh 指标 |
| ROS/fake driver | 多 publisher、odom/TF/IMU dropout、clock jump、solver hang、node kill、cmd watchdog |
| 实物/HIL | 最终命令与反馈、driver watchdog、低/高电量、不同地面、线/角/组合激励、held-out 路径 |
| 证据链 | preflight、runtime ACK、cycle_id join、recorder completeness、postflight、freeze/hash 重算 |

所有数值门限应由标定或 pilot 数据预注册，不在代码中临时拍定，也不能看到 `D_confirm` 结果后再放宽。

## 7. 建议的提交拆分

为避免“重构很久但中间不可运行”，建议按以下顺序提交：

1. `测试：冻结现有命令链与Phase-Rejoin行为基线`
2. `重构：统一最终命令发布事务与相位提交时序`
3. `重构：引入预计发布时间和双通道执行增广模型`
4. `功能：新增delay-augmented Phase-Rejoin求解器`
5. `仿真：增加独立执行器与液体plant及故障注入`
6. `重构：引入正式实验session与严格preflight`
7. `工具：增加Scout执行模型标定与held-out验证`
8. `功能：生成正式OfflineSloshOCP与执行兼容集`
9. `验证：冻结held-out gate与正式仿真报告`
10. `实物：完成monitor证据链与formal release`

每个提交都必须至少通过对应单元测试和受影响的 replay；solver/schema 变化应与相应生成物、validator 和测试放在同一个提交中。

## 8. 最终完成定义

代码层“完成”不是文件拆完，也不是 development 仿真能跑。正式方法至少同时满足：

1. **命令真值闭合：**唯一最终出口，history、audit、phase 和 Scout 输入一致；
2. **执行因果闭合：**$\widehat d_c$、双通道 buffer、$\tau/K$、fractional delay 和当前新决策全部进入模型；
3. **求解器闭合：**$N_e$ 增广 OCP、9 维 gate 和 $\mathcal B^{\mathrm{exec}}$ 在 formal solver 中真实生效；
4. **仿真闭合：**独立 plant 下 C0–C4 可复现，两个 slosh 通道均正向且公平性成立；
5. **证据闭合：**typed session、hash、preflight、runtime ACK、bag 和 postflight 可复现；
6. **实物模型闭合：**G0 的执行稳定性、完整 lead、第一拍灵敏度和 IMU/RGB 幅相均为 GO；
7. **恢复资产闭合：**正式完整尾段、$\mathcal B^{\mathrm{exec}}$、held-out gate 和 false-accept 报告全部冻结；
8. **放行闭合：**monitor 不改命令，低速 enforce 只在人工审核后启用并可一键回退。

完成这些工程项，只说明方法具备正式实验资格；论文中的“防晃改善”仍必须由冻结的 C0–C4 仿真和独立 RGB 实物结果给出，不能用代码测试替代。
