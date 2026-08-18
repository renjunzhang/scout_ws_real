# RTX 5080 DualSPHysics GPU 液体仿真阶段 3–6 续接方案（Agent 执行版）

## Material Passport

- Origin Skill: `academic-research-suite / experiment-agent`
- Origin Mode: `plan`
- Origin Date: `2026-08-11 Asia/Shanghai`
- Verification Status: `STAGES_1_2_3_PASS / STAGE_4_LIQUID_ONLY_DEVELOPMENT_VALIDATION_PASS`
- Version Label: `gpu_liquid_stage_3_6_continuation_v8_one_primary_optional_pair`
- Last Status Sync: `2026-08-12 00:22:22+08:00`

> 本文是 create-new 的续接方案，不修改、不取代也不重新冻结
> [RTX 5080 GPU 8 小时快速构建方案](./20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md)。
>
> 父方案 SHA-256：
> `9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d`
>
> 本文只定义阶段 3–6 的执行合同和验收边界，不授权构建、加载 AppArmor、暴露 GPU、
> 执行 candidate、运行 solver、复制/改写原始 bag 或启动 ROS/Gazebo。

## 0. 结论与当前起点

完整 GPU 液体仿真按六阶段划分：

| 六阶段编号 | 内容             | 对应既有项目阶段                                        | 本文是否覆盖                                                             |
| ---------- | ---------------- | ------------------------------------------------------- | ------------------------------------------------------------------------ |
| 1          | GPU 程序构建     | GPU 构建方案 G0–G3/G5                                  | **PASS**；candidate 已构建并收紧为 `0400`                        |
| 2          | 静态审计         | GPU 构建方案 G4/G6/G7                                   | **PASS**；`sm_120`/PTX/对象/依赖已静态核验                       |
| 3          | GPU 运行冒烟测试 | 新 GPU runtime admission；U3 1 s C1M smoke              | **PASS**；V6 one-shot smoke 和 create-new postvalidation/QC 已通过 |
| 4          | 液体单独验证     | U3 settled state、CPU/GPU parity、U4 合成运动           | **PASS（development-only）；未验证实物保真**                       |
| 5          | 小车与液体耦合   | U5 R7 Gazebo 已执行运动离线回放                         | **本轮只执行 1 个主 bag，最多再执行 1 个可选配对 bag**             |
| 6          | 回放与对比       | 选定 R7 bag 信号/液体模型比较、动态可视化；实物参考待补 | **只闭合实际执行的 1–2 行 development-only 通道**                 |

当前已完成阶段 1–4。阶段 4 的最终结论为
`PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION`；该 PASS 只覆盖固定软件、固定数值合同和
冻结阈值下的液体单独开发验证，不是实物保真、formal 或 production PASS：

```text
GPU_BUILD_PASS=true
GPU_BINARY_EXISTS=true
GPU_STATIC_AUDIT_PASS=true
CANDIDATE=/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T170339Z_a.partial/output/artifacts/DualSPHysics5.4_linux64
CANDIDATE_MODE=0400
CANDIDATE_SIZE=105654136
CANDIDATE_SHA256=cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f
CANDIDATE_EXECUTED_IN_AUTHORIZED_STAGE3=true
GPU_RUNTIME_STATUS=PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY
GPU_SMOKE_SOLVER_RC=0
GPU_SMOKE_PHYSICAL_TIME_SECONDS=1.0
GPU_SMOKE_PARTICLES=9078
GPU_SMOKE_NOUT=0
U3_ACCEPTANCE=U3_SETTLED_STATE_FROZEN
CPU_GPU_PARITY_STATUS=PASS_CPU_GPU_PARITY_DEVELOPMENT_ONLY
STAGE4_STATUS=PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION
STAGE4_EXECUTION_AND_ADJUDICATION_COMPLETE=true
STAGE4_LIQUID_ONLY_VALIDATION_COMPLETE=true
DEVELOPMENT_ONLY=true
PHYSICAL_FIDELITY_VALIDATED=false
PHASE5_ADMITTED=false
U4=SYNTHETIC_ZERO_TRANSLATION_YAW_PASS
U5_SIM_R7_EXECUTED_MOTION=NOT_STARTED
```

旧 campaign `u3_source_gpu_build_sm120_20260810T102641Z` 及其 `TIMEBOX_EXHAUSTED`
证据仍原样保留。后续 fresh campaign
`u3_source_gpu_build_sm120_20260810T170339Z` 已完成一次 Make：`rc=0`，
`elapsed=744.931052 s`，产生 131 个对象和上述 candidate。静态审计 v2 已得到
`PASS_GPU_BUILD_SM120_STATIC_AUDIT`；审计前后 candidate 和对象身份不变，
profile lifecycle 为零残留。随后阶段 3 在独立 runtime admission 下完成一次 V6 C1M 1 s GPU
smoke：solver `rc=0`，输出 21 个 Part 帧、30 个文件、9,078 粒子且 `Nout=0`。合法的零字节
stderr 曾触发旧 reader 的 postflight false negative；未重跑 solver，而是以 create-new
postvalidator 和独立 QC 收口。该结论只证明 development-only GPU functional smoke，仍不代表
U3 settled-state、数值收敛或实物物理保真通过。

阶段 4 的早期分支完成了同一 RTX 5080、同一 `Part_0201`、同一 `10.05–20.05 s` 窗口的
`CFL=0.2` 与 `CFL=0.1` 配对 GPU 试验。两侧均为 201 帧、210 文件、9,078 粒子、
`Nout=0`，并通过逐文件 inventory、逐 Id 初态、Shifting、DDT、dp、GPU/backend 和唯一 CFL
delta 核验。降低 CFL 后四项 primary 指标均改善，primary normalized score ratio 为
`0.8794251121`；但候选仍有 9/17 个绝对稳态指标超限，速度 RMS 在达到最低点后回弹，且
`position_interframe_max_m_s`、`surface_abs_drift_m_s`、`surface_spread_m` 三项出现合同定义的
实质性退化。该分支按合同 FAIL 并保留，不再作为当前状态。

早期 create-new 技术裁决回执：

```text
/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_shift_none_cfl0p1_from_0201_v4.qc_phase4_final_v1.json
SHA-256=7e1a1dc537cc5ac2f558bd49d8de9c079d4557dc4da8a2578a9b9d9bbb5f2bd7
status=FAIL_U3_CFL0P1_SENSITIVITY_NUMERICAL_STABILITY
exact_blocker=CFL_0P1_FAILS_9_OF_17_ABSOLUTE_SETTLING_LIMITS_AND_REBOUND_PERSISTS
next=STOP_BEFORE_PHASE5_REQUIRES_NUMERICAL_REMEDIATION_AND_NEW_AUTHORIZATION
```

后续采用 DualSPHysics 原仓库已有的 artificial-viscosity 参数通路，保持 `Shifting=None`、
`CFL=0.1`，将主要数值修复冻结为 `ViscoArtificial=0.3`。冷启动 A/B、延长沉降、restart
等价性和重复性全部通过，并在 `45.05001991890928 s` 冻结 `Part_0901` settled state：

```text
settled_status=U3_SETTLED_STATE_FROZEN
settled_receipt=/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_repeatability_restart_20260811T124800Z_v15.qc_v1.json
settled_receipt_sha256=4703c08e09b33fb16ad68b368dd5e99c2b2c930a3091008fa399241fc58a7fd8
backend_parity_status=PASS_CPU_GPU_PARITY_DEVELOPMENT_ONLY
backend_parity_receipt=/home/zrj/scout_liquid_lab/audits/u3_c1m_cpu_gpu_backend_parity_20260811T133800Z_v19.qc_v1.json
backend_parity_receipt_sha256=41080d61f749e25db1d54da63d701c26b9e2badc2d89fdebd748997282eb18f1
```

从同一冻结稳态分别完成零回放、2 mm/1 Hz 平移和 2°/1 Hz 偏航。三次 solver 均 `rc=0`，
共 143 帧全部保持 9,078 粒子、`Nout=0`、精确 Id 集和有限数值。最终只读 QC 得到：

```text
status=PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION
qc_receipt=/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_synthetic_motion_20260811T143346Z_v23.qc_v1.json
qc_receipt_sha256=1f837ce7c52ce80971adb981e121eace3c92968095193bb6db3519b39776e383
metrics_sha256=10b8976e8fceff5dbcaa244693afd5ac016734cd10db5696613cb6371183ba25
figure_receipt_sha256=c0f669726973e98259e307b8c54afdd42c081a1bee993f84eaf34c040e80683d
stage5_admitted=false
next=STAGE5_REQUIRES_SEPARATE_USER_AUTHORIZATION_AND_PHYSICAL_INPUTS
```

上面的 `next` 是阶段 4 receipt 发布时的原始字段，作为历史证据保持字面不改；本方案在获得
R7 bag 候选输入加入后新增的当前操作入口以 `0.1` 和 `12` 的 S5A0 为准，不会反向改写该
receipt。本轮执行范围已经收缩为 1 个主 bag、最多 1 个可选配对 bag；88 行 corpus 只保留为
只读背景，不进入本轮封存、导出、solver 或阶段 6 验收分母。

### 0.1 R7 ROS1 bag 候选输入快照与本轮精确选择（只读观察，尚非正式 receipt）

阶段 5 的当前候选输入位于：

```text
/home/zrj/slosh_bags/matrix_bags/
source_domain=SIM_R7_EXECUTED_GAZEBO_MOTION
physical_robot_bag=false
r8_release=false
format=ROS1_BAG_V2
bags=88
total_bytes=1368771842
index_parse=88/88 PASS
active_files=0
```

Bag 路径合同如下；任何 Agent 都不得用 basename 全盘搜索或“选最新 bag”代替 exact path：

```text
bag_root:
  /home/zrj/slosh_bags/matrix_bags

per_attempt_template:
  /home/zrj/slosh_bags/matrix_bags/<attempt_id>/capture.bag

S1 template:
  /home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_<condition>_b<01..08>_r01/capture.bag

S2A template:
  /home/zrj/slosh_bags/matrix_bags/SIM-S2A_SELECTIVITY_L1_C1_<condition>_b<01..08>_r01/capture.bag

S2B template:
  /home/zrj/slosh_bags/matrix_bags/SIM-S2B_TRANSFER_H1_C2_<condition>_b<01..08>_r01/capture.bag

known failure exact path:
  /home/zrj/slosh_bags/matrix_bags/SIM-S2B_TRANSFER_H1_C2_FixedProfile_b08_r01/capture.bag
```

本轮唯一默认输入和唯一可选配对输入固定为：

```text
EXECUTION_SCOPE=ONE_PRIMARY_PLUS_ZERO_OR_ONE_OPTIONAL_PAIR

PRIMARY_ATTEMPT=SIM-S1_CORE_H1_C1_Bsmooth_b01_r01
PRIMARY_BAG=/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag
PRIMARY_OBSERVED_SIZE=13996902
PRIMARY_OBSERVED_MODE=0755
PRIMARY_OBSERVED_SHA256=c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07
PRIMARY_OBSERVED_DURATION_SECONDS=49.409

OPTIONAL_SECOND_ATTEMPT=SIM-S1_CORE_H1_C1_Bslosh_b01_r01
OPTIONAL_SECOND_BAG=/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bslosh_b01_r01/capture.bag
OPTIONAL_SECOND_OBSERVED_SIZE=14075948
OPTIONAL_SECOND_OBSERVED_MODE=0755
OPTIONAL_SECOND_OBSERVED_SHA256=620c05067e62bd998d9b8bb9350f1c0d56a843802beac42463165b5a450a2b54
OPTIONAL_SECOND_OBSERVED_DURATION_SECONDS=49.549
OPTIONAL_SECOND_DEFAULT=NOT_RUN

SELECTED_CONTAINER=C1
SELECTED_STAGE_PATH_BLOCK=SIM-S1_CORE/H1/C1/b01
FULL_CORPUS_INTAKE_OUT_OF_SCOPE=true
SIX_ROW_PILOT_OUT_OF_SCOPE=true
FULL_88_REPLAY_OUT_OF_SCOPE=true
C2_REPLAY_OUT_OF_SCOPE=true
```

选择 `Bsmooth_b01` 作为单 bag 基线，是因为它属于当前可复用 C1 geometry/mount/settled-state
链。若用户明确继续第二行，只增加同一 stage/path/container/block 的 `Bslosh_b01`，用于主要方法
与基线的配对比较；它不得因主 bag PASS 而自动运行。不得换成 C2，也不得在看到液体结果后更换
condition 或 block。上面 size/mode/hash/duration 只是方案编写时的 expected witness；S5A0 仍须
在只读 sandbox 中独立重算后才能发布正式 receipt。

当前 88/88 `capture.bag` 的观察 mode 均为 `0755`，包括上面两条精确路径。这不是执行授权，也
不得通过 `chmod` 改写原始输入；S5A0 必须把选定文件的 mode 作为 provenance 固定，在断网只读
sandbox 中只按数据打开，明确 `source_bag_executed=false`。

这是 R7 Gazebo 仿真矩阵的已执行运动，不是实物小车采集，也不是 R8 formal release。当前目录只有
每个 attempt 的 `capture.bag`，没有 attempt manifest、postflight 或 release sidecar，因此只能进入
“sealed imported bag-only development lane”；provenance 必须保持 `PARTIAL`，不得把它升级成
formal 或 physical evidence。

矩阵只读普查为：

| Stage                   | 路径/容器 | 条件                                           | Block |          Bag |
| ----------------------- | --------- | ---------------------------------------------- | ----: | -----------: |
| `SIM-S1_CORE`         | H1 / C1   | B0、Bsmooth、SmoothMatch、FixedProfile、Bslosh |     8 |           40 |
| `SIM-S2A_SELECTIVITY` | L1 / C1   | Bsmooth、FixedProfile、Bslosh                  |     8 |           24 |
| `SIM-S2B_TRANSFER`    | H1 / C2   | Bsmooth、FixedProfile、Bslosh                  |     8 |           24 |
| **合计**          |           |                                                |       | **88** |

88/88 均为 regular ROS1 Bag V2、索引可读；每个 attempt 目录恰有一个 `capture.bag`，未观察到
symlink、hardlink、special file 或 `.active`。单 bag 记录时长为 `46.528–75.315 s`。以
“相对路径 + 每 bag SHA-256”排序后得到的候选聚合摘要为：

```text
observed_sorted_relative_sha256_manifest_digest=
7db18f7a0771c31e84f6dbefab571304241ef99438f4eb6891b4cddebf01f567
```

该值只是方案编写时的只读观察，不是 S5A0 final receipt；执行 Agent 必须独立重算、逐项核验并
将新 manifest 原子发布到独立 audit root，绝不能在原 bag 目录写入清单、缓存或重建索引。

原 R7 证据链的唯一方法失败仍须原样保留，但它不属于本轮选定行集，也不得为“保留失败行”而
加入本轮 replay：

```text
attempt_id=SIM-S2B_TRANSFER_H1_C2_FixedProfile_b08_r01
source_outcome=METHOD_FAILURE
terminal=GOAL_TIMEOUT
retryable=false
```

不得从 `/spmpc/status` 重新推断或用液体 replay 成功覆盖该 source outcome。本轮阶段 5/6 的
planned denominator 只能是 `1`；仅当用户另行确认可选配对行后才是 `2`，绝不能写成 `6` 或
`88`。

因此当前实际顺序是：

```text
阶段 1 构建 PASS
  → 阶段 2 静态审计 PASS
  → 阶段 3 GPU functional smoke PASS
  → 阶段 4 液体单独验证 PASS（development-only，当前停止点）
  ↛ 阶段 5 R7 Gazebo 已执行运动单向回放（未启动；先做只读 bag 封存门禁）
  ↛ 阶段 6 R7 bag/液体模型回放与对比（未准入；实物 reference 仍缺失）
```

任何前一门禁失败都不得借用后一阶段的权限、数据或结果来“补证”。

## 1. 术语和系统边界

### 1.1 “小车与液体耦合”的准确含义

本文阶段 5 的“耦合”固定为 **two-pass、离线、单向运动耦合**。本轮只消费 1 个主 bag，最多
再消费 1 个由用户明确启用的同条件配对 bag；输入通道名固定为
`SIM_R7_EXECUTED_GAZEBO_MOTION`：

1. R7 Gazebo attempt 已独立完成并关闭 bag；
2. 只以不可变 bag 的 `/odom` executed pose 为 primary motion；`/tf`、`/tf_static` 只做
   frame/位姿交叉检查；
3. DualSPHysics 在另一阶段离线重放该容器运动；
4. 液体结果只进入 append-only secondary ledger 和事后比较报告。

明确禁止：

- 在 Gazebo case 运行时并发启动 DualSPHysics；
- 用 `/cmd_vel`、计划轨迹或控制器状态代替已执行运动；
- 把液面或液体力反馈给小车、控制器、planner 或底盘；
- 用液体 replay 的结果改变原 source attempt 的成功/失败结论；
- 把本方案表述为实时双向 FSI/co-simulation。

“已执行运动”只表示 Gazebo 中实际走出的 odometry，而不是命令轨迹；它不等于实物小车运动。
未来实物 bag 必须使用新的 `PHYSICAL_EXECUTED_MOTION` source domain、独立 ABI 身份和物理
reference 门禁，不能把本轮 R7 transfer package 改名复用。

若未来确需液体反作用力影响小车动力学，必须新成立 two-way co-simulation 项目，重新设计
plant ABI、时序、稳定性、安全防火墙和 formal 验收；不属于本文阶段 3–6。

### 1.2 结果身份上限

在数值收敛和实物验证完成前，所有阶段强制保留：

```text
development_only=true
formal=false
fidelity_validation_status=SIM_ONLY_UNVALIDATED
physical_primary_eligible=false
production_authorized=false
do_not_override_source_attempt=true
```

GPU functional PASS、U3 settled PASS、U4 synthetic PASS、
`U5_SIM_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY` 和实物 fidelity PASS 是五个
不同结论，禁止相互代替。当前 R7 bag 通道在阶段 5/6 的结果身份上限分别为：

```text
U5_SIM_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY
S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY
S6_PRIMARY_R7_BAG_REPLAY_AND_MODEL_COMPARISON_PASS_DEVELOPMENT_ONLY
optional_if_separately_authorized=S5B1/S6_PAIRED
PHYSICAL_REFERENCE_PENDING=true
```

## 2. 父输入、固定身份与待冻结输入

### 2.1 当前可引用的固定输入

```text
DualSPHysics source commit:
  ef3721a861fda961f0e2f9ec4cd317b19de99086

GPU build plan SHA-256:
  9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d

GPU build/static-audit evidence:
  campaign = u3_source_gpu_build_sm120_20260810T170339Z
  build_id = u3_source_gpu_build_sm120_20260810T170339Z_a
  candidate path = /home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T170339Z_a.partial/output/artifacts/DualSPHysics5.4_linux64
  candidate inode/mode/size = 15733148 / 0400 / 105654136
  candidate SHA-256 = cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f
  object count = 131
  object inventory SHA-256 = 19fcea3c6207ca00e33e177e9da6f35667dd168951bbd89c188765172fdefd9b
  build receipt = /home/zrj/scout_liquid_lab/audits/u3_source_gpu_build_sm120_20260810T170339Z_a_build_final.json
  build receipt SHA-256 = 2ff3da17da7658da35587a6b1233f3bb1c039488325345537c00d40ebf30e9da
  static-audit v2 receipt = /home/zrj/scout_liquid_lab/audits/u3_source_gpu_build_sm120_20260810T170339Z_a_static_audit_v2.json
  static-audit v2 receipt SHA-256 = 794ecf80d8665b65ba7e31d1a8bf32a67f8cca7c7cc24ea33718cb30c27c97fb
  static-audit v2 lifecycle = /home/zrj/scout_liquid_lab/audits/u3_source_gpu_build_sm120_20260810T170339Z_a_static_audit_v2_lifecycle.json
  static-audit v2 lifecycle SHA-256 = b29c4be44532f8eefa899d19427b25c4781cb535f12a6971ab7d4e958c3a3d83
  build status = PASS_GPU_BUILD_CANDIDATE_DISARMED
  static-audit status = PASS_GPU_BUILD_SM120_STATIC_AUDIT
  candidate_executed_during_build_or_static_audit = false

GPU runtime smoke evidence:
  run root = /home/zrj/scout_liquid_lab/gpu_runs/u3_c1m_gpu_smoke_v6_20260810T190500Z.partial
  access probe = /home/zrj/scout_liquid_lab/audits/u3_gpu_smoke_v6_stage_probe_20260810T190500Z.json
  access probe SHA-256 = b34a225f7ea5668b95501ce75eda6228ceca5f6194b63e334eca02a41ce901c2
  postvalidation = /home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_smoke_v6_20260810T190500Z.postvalidation_v1.json
  postvalidation SHA-256 = 9cbf9cdb48b240a73129eadbde36cf55ea3f0b1ad23e229f79aabe287085367e
  QC = /home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_smoke_v6_20260810T190500Z.postvalidated_qc_v1.json
  QC SHA-256 = e58f8c976ddaac22af2256af6fcdf3684ee897799d4c03df7c885f500498d824
  solver rc / physical time = 0 / 1.0 s
  output files / Part files = 30 / 21
  particles / moving / fluid / excluded = 9078 / 2669 / 6409 / 0
  output canonical SHA-256 = d33a9728f3a1f9bd6366f53a0c9f980c090950404f51b574616cc40f0592afff
  status = PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY

C1M GenCase v8:
  particles = 9,078
  moving boundary = 2,669
  fluid = 6,409
  BI4 SHA-256 = b463ddfe548b3db78b02f23b075dadbdd3c71ea766eb092701b56978ddb3a8e7
  XML SHA-256 = 28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb
  DsphConfig.xml size = 293 B
  DsphConfig.xml SHA-256 = 0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd

CPU development candidate SHA-256:
  5aa464a8f37b0185bac863987f0d1079a0f1a3d6daead6581562c832278ea202

U3 current state:
  cold-A/cold-B reproducibility = PASS
  restart equivalence = PASS
  baseline extension to 10.05 s = COMPLETE / SETTLE FAIL
  DDT-ramp raw run to 10.05 s = COMPLETE / QC FAIL
  Shifting=None short probe to 10.05 s = COMPLETE / ABSOLUTE SETTLE FAIL
  Shifting=None CFL=0.2 extension to 20.05 s = COMPLETE / SETTLE FAIL / REBOUND
  Shifting=None CFL=0.1 paired sensitivity to 20.05 s = COMPLETE / SETTLE FAIL / REBOUND
  above early branch adjudication = COMPLETE / FAIL_PHASE4_LIQUID_STANDALONE_NUMERICAL_STABILITY
  Shifting=None / CFL=0.1 / ViscoArtificial=0.3 cold-A/B and extension = PASS
  settled checkpoint = Part_0901 at 45.05001991890928 s
  settled/repeatability/restart status = U3_SETTLED_STATE_FROZEN
  CPU/GPU backend parity = PASS_CPU_GPU_PARITY_DEVELOPMENT_ONLY
  zero/translation/yaw synthetic motion = PASS
  phase-4 final status = PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION
  physical fidelity = NOT_VALIDATED
  phase-5 = NOT_STARTED / NEXT_IS_READ_ONLY_PRIMARY_R7_BAG_INTAKE
```

这些身份只能用于设计和后续 parent-hash 核验。CPU candidate 的存在不授权执行它；阶段 3 的
历史执行已绑定上述通过静态审计的 GPU candidate，并已在 runtime 准入时核验
path/inode/mode/size/SHA-256 与 parent receipts。任何后续 GPU run 仍需重新做自己的身份与资源准入。

### 2.2 进入正式物理验证前必须由用户提供或确认

以下信息缺失时可做阶段 3 软件 smoke，但不得完成阶段 4 的正式物理 case，也不得进行阶段 6
实物 fidelity 裁决：

1. C1/C2 容器总内高、内径/内表面尺寸、壁厚、底部形状/圆角、开口或盖体状态；
2. 容器相对 `base_link` 的安装位姿 `T_base_container` 及测量误差；
3. 液体类型、温度、密度、动力/运动黏度、表面张力、实际体积/液深误差；
4. 容器内壁材料、接触角或可用的润湿信息；
5. 实物 RGB/液位传感器的 ROI、采样率、空间标定、时间同步和 reference cases；
6. primary 晃动高度采用 `H_crest`、absolute elevation 还是 peak-to-peak；
7. 单 case 最长离线时间、可用 GPU 时段、磁盘保留预算和重复次数。

未知物理字段必须显式为 `UNKNOWN`，不得静默使用默认值后冒充实物参数。R7 development-only
replay 可以使用独立冻结的仿真 C1/C2 geometry、fluid 和 mount manifests，但必须明确
`source=SIM_R7`。C1 与 C2 必须是两套不同身份；不得用当前 C1M、C1 参数或 bag 内单条
`/tf_static` 冒充 C2 容器/安装合同。

## 3. 全局执行合同

### 3.1 Goal 和身份

每个实际执行阶段必须另立 goal。每个 goal 在写操作前冻结：

```text
campaign_id / run_id / replay_id
parent receipt paths and SHA-256
policy/schema/gate/tests SHA-256
candidate path/inode/mode/size/SHA-256
input manifest paths and SHA-256
exact output root
exact argv and sanitized environment
AppArmor profile name/path/mode/size/SHA-256
resource limits and timeout/kill-after
expected output inventory and receipt path
```

身份未冻结或 parent hash 不匹配时只能 `BLOCKED_PRECHECK`。不得在运行中更换 candidate、
case、GPU、参数、时间窗、分辨率或并发数。

### 3.2 文件和证据

- 所有 run root、receipt、manifest、ledger entry 和 final result 都 create-new；
- 中间输出只写 `<identity>.partial/`，通过完整 inventory 和 hash 后才原子发布 final；
- 输入目录只读，拒绝 symlink、hardlink、FIFO、socket、device file、额外文件和执行位漂移；
- 原始 BI4、bag、motion 和实物 reference 保持不可变，不覆盖、不清理、不“选最新文件”；
- 日志保存完整 stdout/stderr/return code，同时生成有固定大小上限的摘要；
- 每一结果绑定源码、binary、case、profile、GPU/driver、参数、输入和输出 hash；
- 失败、超时和 NO-GO 同样发布 create-new receipt，不删除后重用 identity。

### 3.3 进程、GPU 和系统动作

- solver 默认断网，禁止 ROS/Gazebo 环境注入；
- 通过 `locks/gpu0.lock` 实现单 GPU 独占；启动前记录 GPU、driver、显存和 compute processes；
- 仅精确暴露获准的 NVIDIA character devices，不挂整个 `/dev`；
- driver/runtime libraries 只读，禁止 CUDA stub `libcuda.so`；
- AppArmor 每个 profile 采用 load → verify → one fixed phase → unload → zero-residue 生命周期；
- profile load/unload、sudo、device exposure 和 candidate execution 均需针对精确 hash 的用户授权；
- runner 只能结束自己创建的 PID/进程组，禁止 `killall`/`pkill`；
- 外层固定 wall timeout 和 kill-after；硬截止、OOM、Xid/reset、NaN/Inf、粒子泄漏、
  `Nout>0`、Gauge 缺失、磁盘低水位均 fail-closed；
- 不因失败自动降分辨率、改时间步、换 CPU、放宽 profile 或延长时间后冒充同一 run；
- 每 10–30 秒记录 wall time、RSS、GPU memory/utilization/temperature、CPU/RAM、PSI、swap、
  磁盘余量和进程存活状态。

### 3.4 统一状态机

```text
STAGES_1_2_PASS
  → S3_RUNTIME_CONTRACT_FROZEN
  → S3_GPU_FUNCTIONAL_SMOKE_PASS
  → S4_DDT_QC_AND_NUMERICAL_CONTRACT_FROZEN
  → U3_SETTLED_STATE_FROZEN
  → U4_SYNTHETIC_MOTION_PASS
  → S4_LIQUID_STANDALONE_VALIDATION_PASS
  → S5A0_PRIMARY_R7_BAG_SEALED_DEVELOPMENT_ONLY
  → S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED
  → S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY
  → S6_PRIMARY_R7_BAG_REPLAY_AND_MODEL_COMPARISON_PASS_DEVELOPMENT_ONLY
  [optional new authorization]
  → S5B1_OPTIONAL_PAIRED_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY
  → S6_PAIRED_R7_BAG_REPLAY_AND_MODEL_COMPARISON_PASS_DEVELOPMENT_ONLY
```

任意 `FAIL`、`NO_GO`、`TIMEOUT` 或 parent drift 都进入 `STOP_AND_PRESERVE_EVIDENCE`，
不得自动跨阶段。

当前已到达并停在 `S4_LIQUID_STANDALONE_VALIDATION_PASS` 的 development-only 实例；
这不会自动授权 S5A0，也不会自动授权任何 GPU replay。

## 4. 阶段 3：GPU 运行冒烟测试

### 4.1 唯一目标

证明经阶段 2 静态审计通过的 RTX 5080 candidate 能在受限 GPU runtime sandbox 中完成
C1M 零运动 1 s 小 case，并产生结构完整、数值有限的输出。

成功状态只能是：

```text
PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY
```

它不是 U3 settle、CPU/GPU 等价、物理保真或 production PASS。

### 4.2 前置门禁

必须全部满足：

1. fresh 阶段 1 已产生 regular、non-symlink、`nlink=1`、mode `0400` 的 candidate；
2. 阶段 2 已对同一 inode/size/SHA-256 得到 `PASS_GPU_BUILD_SM120_STATIC_AUDIT`；
3. candidate、131-object inventory、`sm_120` cubin、配套 `compute_120` PTX 和依赖 hash 未漂移；
4. C1M 三个固定输入的 path/type/mode/size/SHA-256 匹配 §2.1；
5. 新 runtime policy、closed schema、gate、tests、bootstrap/supervisor 和 exact profile 已
   create-new、静态验证并取得精确用户授权；
6. GPU 设备无竞争 compute process，资源和温度通过动态 preflight；
7. `/dev/nvidiactl`、`/dev/nvidia0`、`/dev/nvidia-uvm` 逐项以 `stat` 冻结 character-device、
   owner、mode 和动态 major:minor；
8. `libcuda.so.1` 由宿主 `ldconfig -p` 的真实 64-bit driver 链解析，拒绝 Toolkit stub；
9. fresh output root、start/final/failure receipt 均不存在。

`/dev/nvidia-uvm-tools` 只允许在真实 access-denial 证据证明必要后，另做最小 profile revision；
禁止预先开放 `/dev/nvidia-modeset`、`/dev/dri/*` 或整个 `/dev`。

### 4.3 固定首跑

首跑必须冻结为父方案 §12 的字面 argv：

```text
<candidate> <case-prefix>/C1M_zero <fresh-output>
-gpu:0 -ompthreads:1 -stable:1 -vres:0 -cellmode:full
-tmax:1.0 -tout:0.05 -sv:binx,info -svres:1
-svtimers:0 -svdomainvtk:0 -saveposdouble:1
-nortimes:1 -createdirs:1 -csvsep:0
```

不得先运行 `-h`、`-info`、版本查询或第二个“探路”case。上述调用就是唯一 runtime attempt。

### 4.4 运行中监控和硬停

必须同时监控：

- candidate PID/进程组、wall time 和输出推进；
- CUDA error、kernel launch error、OOM；
- kernel log 中与本 run 时间窗关联的 NVIDIA Xid/reset；
- GPU memory、utilization、temperature 和 power；
- host RSS、MemAvailable、swap、PSI 和磁盘低水位；
- 输出文件数、最后完整 time slot、粒子数、`Nout`、NaN/Inf。

任何安全门禁失败立即结束自有进程组并保留 partial。不得为通过 smoke 修改 XML、DDT、
`dp`、`tmax`、`tout` 或 solver flags。

### 4.5 验收

全部满足才允许 PASS：

- solver return code 为 0，且没有 timeout、signal、CUDA error、Xid/reset 或 AppArmor deny；
- 21 个 `Part_####.bi4`，总计 30 个精确输出文件，无 extra/symlink/hardlink；
- 每帧 9,078 粒子，其中 moving boundary 2,669、fluid 6,409；
- 每帧 `Nout=0`，`PartMotionRef` 严格零运动；
- Posd/Vel/Rhop 等冻结字段均 finite，密度、domain、时间槽和粒子 ID 完整；
- 输入文件、candidate 和 host runtime libraries 在运行前后 inode/mode/size/hash 不变；
- profile、进程、mount、GPU lock 和 sudo timestamp 零残留；
- final receipt 和完整日志/inventory 的 SHA-256 已发布。

最小回执字段：

```text
run_id / parent_build_receipt / parent_static_audit_receipt
candidate path/inode/mode/size/SHA-256
GPU UUID/PCI/driver/device nodes/runtime-library hashes
policy/gate/profile hashes and lifecycle argv/rc
solver exact argv/env/uid/gid/groups
start/end/elapsed/return code/termination reason
resource samples and Xid audit window
input/output inventory and hashes
candidate_executed=true
network_used=false
ros_gazebo_used=false
status=PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY | exact failure
```

## 5. 阶段 4：液体单独验证

阶段 4 不接收 R7 或实物小车运动，分为 U3 静水闭合、CPU/GPU parity、数值收敛和 U4 合成运动。

### 5.1 S4-A：先完成现有 DDT-ramp 只读 QC

遵守权威交接 §11.25 的 DDT-first 顺序：

1. 只读解析已完成的 DDT-ramp `8.05..10.05 s` 输出；
2. 与原参数完全相同时间窗、粒子 ID 和冻结指标比较；
3. 输出速度 RMS/P95/max、specific KE、位置冻结、coverage、密度、粒子完整性和液面代理；
4. 报告 DDT 对数值平台的方向和效应量，不因单个指标改善就接受；
5. 只有候选参数通过预先冻结的全部阈值，才发布新的 numerical-parameter candidate；
6. 不覆盖原 DDT receipt，不把 raw run complete 写成 QC PASS。

若 DDT 候选不通过，先停止并提出新的、单变量、预注册数值实验；不得直接进入 fresh settle。

#### 5.1.1 早期失败分支与阶段 4 最终裁决（2026-08-11）

早期 CFL 配对分支已经执行并按预冻结判据失败闭合：

1. DDT ramp QC 为 `FAIL_U3_DDT_RAMP_NUMERICAL_CANDIDATE`；
2. `Shifting=None` 的 8.05–10.05 s 短探针改善了四项 primary 指标，但未通过绝对稳态门槛；
3. CFL=0.2 的 10.05–20.05 s 延长段在 12.60 s 达到速度 RMS 最低值
   `0.0026370551 m/s`，20.05 s 回升至 `0.0050049450 m/s`，回弹比 `1.89793`；
4. 唯一 CFL delta 的配对敏感性运行把 CFL 从 0.2 降到 0.1。运行时间从
   `367.56875 s` 增至 `728.604891 s`，四项 primary 指标全部有意义改善，归一化分数比为
   `0.8794251121`；
5. CFL=0.1 候选在 12.10 s 达到速度 RMS 最低值 `0.0026239659 m/s`，20.05 s 又回升到
   `0.0045888580 m/s`，回弹比 `1.74883`；
6. 候选仍有 9/17 个绝对指标超限，且三项指标达到“实质性退化”定义，故不得选择该候选。

两组延长段均通过 210 文件 inventory、201 个 Part 帧、逐 Id 相同初态、相同 RTX 5080/backend、
相同 Shifting/DDT/dp/窗口和 `CFL=0.2→0.1` 唯一数值 delta 核验。密度、液面绝对偏差、粒子
完整性、`Nout=0` 和输出结构保持干净；失败集中在速度、动能、位置冻结及持续回弹，而不是文件
损坏或 GPU runtime 错误。

```text
EARLY_BRANCH_STATUS=FAIL_PHASE4_LIQUID_STANDALONE_NUMERICAL_STABILITY
EARLY_BRANCH_ADJUDICATION_COMPLETE=true
EARLY_BRANCH_ACCEPTANCE_PASS=false
EARLY_BRANCH_EXACT_BLOCKER=CFL_0P1_FAILS_9_OF_17_ABSOLUTE_SETTLING_LIMITS_AND_REBOUND_PERSISTS
```

该 FAIL 只约束早期分支，不能覆盖后续独立合同的结果。后续使用原仓库支持的
`ViscoArtificial=0.3` 通路完成了 cold-A/B、延长沉降、repeatability、restart、CPU/GPU parity，
再从冻结 `Part_0901` 分别完成零回放、平移和偏航。最终裁决为：

```text
U3_ACCEPTANCE=U3_SETTLED_STATE_FROZEN
CPU_GPU_PARITY_STATUS=PASS_CPU_GPU_PARITY_DEVELOPMENT_ONLY
SYNTHETIC_RUNS=ZERO_PASS / TRANSLATION_PASS / YAW_PASS
STAGE4_STATUS=PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION
DEVELOPMENT_ONLY=true
PHYSICAL_FIDELITY_VALIDATED=false
PHASE5_ADMITTED=false
```

最终合成运动 QC 的平移/零回放流速响应比为 `18.100341`，偏航/零回放为 `3.952447`；
平移与偏航自由衰减比为 `0.00116321` 和 `0.0269582`，两种输入均重建为 `1.0 Hz`。
平移最大边界误差为 `4.0423e-8 m`，偏航最大位置/角度误差为 `1.4285e-8 m` /
`4.0423e-5 deg`；16-sector 最少粒子为零/平移/偏航 `233/230/229`，均高于门槛 128。

### 5.2 S4-B：冻结静水与数值合同

任何新 solver run 前必须冻结：

- geometry/fluid/numerical parameter manifest；
- `dp`、time step/CFL、kernel、boundary formulation、DDT、viscosity 和输出频率；
- Gauge probe 布局、`masslimit`、`pointdp`、无效值规则和坐标变换；
- settle 时长/尾窗、checkpoint 选择规则和停止条件；
- CPU/GPU 按 `Idp` 对齐的 Posd/Vel/Rhop/积分状态容差；
- repeat count、random/seed policy、资源/时间/磁盘上限；
- 每次 fresh clone、receipt、inventory 和 QC schema。

CPU/GPU 输出不得预设 BI4 字节完全相同；容差必须在看到 GPU 对比结果前冻结。

### 5.3 S4-C：U3 settled state 重新验收

参数候选冻结后，必须重新做：

1. fresh cold-A；
2. 独立 fresh cold-B；
3. A/B repeatability；
4. checkpoint restart equivalence；
5. 冻结 settled-end state，并验证动态 replay 从其只读 fresh clone 启动。

静水硬阈值沿用既有合同：

```text
speed RMS <= 0.001 m/s
speed P95 <= 0.001 m/s
speed max <= 0.005 m/s
specific KE <= 5e-7 J/kg
```

并必须同时通过：

- interframe/net-tail 位置冻结；
- 尾窗 coverage；
- 粒子数、ID 和质量完整性；
- `Nout=0`、无泄漏、无 NaN/Inf、无越界；
- 密度/压力和圆周液面稳定性；
- A/B 与 restart 在冻结容差内一致。

任何一项失败都保持 `U3_ACCEPTANCE=NO_GO_NOT_SETTLED`。只有全部通过才发布：

```text
U3_SETTLED_STATE_FROZEN
```

settled state 必须绑定 binary/backend/device/parameter/case hash。CPU settled state 不得直接冒充
GPU settled state；若跨后端复用，必须先有独立等价性门禁。

执行状态：**已完成**。`ViscoArtificial=0.3` 的独立 cold-A/B、延长沉降、restart 和重复性
均已按冻结阈值通过，GPU settled state 已冻结为 `Part_0901 @ 45.05001991890928 s`；裁决回执
SHA-256 为 `4703c08e09b33fb16ad68b368dd5e99c2b2c930a3091008fa399241fc58a7fd8`。

### 5.4 S4-D：CPU/GPU backend parity

使用同一 sealed case、参数、输出时间槽和 fresh initial state，分别生成有独立 identity 的 CPU/GPU
结果。至少比较：

- 粒子 ID 集合、粒子数、`Nout` 和输出时刻；
- 按 `Idp` 对齐的 Posd、Vel、Rhop 和必要积分状态；
- 静水偏置、速度/动能尾窗和 Gauge `zsurf`；
- wall time、峰值 RSS、峰值显存和资源效率；
- 重复运行离散度与 backend 差异的相对大小。

比较脚本、schema、容差和异常处理先静态验证。超过容差只能发布
`FAIL_CPU_GPU_PARITY`，不得挑选粒子、probe 或时间窗后重算。

执行状态：**已完成 / PASS（development-only）**。CPU 重复性、GPU 重复性及全部冻结的逐粒子
位置、速度、密度指标通过；裁决回执 SHA-256 为
`41080d61f749e25db1d54da63d701c26b9e2badc2d89fdebd748997282eb18f1`。

### 5.5 S4-E：U4 合成运动

只有 `U3_SETTLED_STATE_FROZEN` 后，才从同一 settled state 的 fresh clone 分别运行：

1. 零运动 replay；
2. 小幅单轴正弦平移；
3. 小幅正弦 yaw；
4. 激励停止后的自由衰减；
5. 若未来研究问题需要且另获授权，再执行组合运动；它不属于本轮 development-only 三分支硬门禁。

所有输入明确标记 `SYNTHETIC`，并在运行前冻结振幅、频率、相位、持续时间、tail、坐标系、
插值和采样周期。C1M 的 `mvnull` 零运动 transport 不能冒充非零 U4 PASS。

验收至少包括：

- motion timestamp 单调、坐标/方向/单位正确，输入与 boundary 实际运动一致；
- 16 个圆周 probes，另行评估 32 probes；中心/中半径 probes 用于 QC；
- `zsurf` 有效率、`H_crest(t)`、`H_abs(t)`、peak-to-peak、peak/p95/RMS；
- 主频、相位和自由衰减/阻尼与冻结理论或数值参考一致；
- 无 NaN、泄漏、质量损失、粒子越界和 GPU/runtime 异常；
- repeatability 和 CPU/GPU parity 在冻结容差内。

执行状态：**本轮授权范围已完成 / PASS（development-only）**：

- 零回放：21 帧，final receipt SHA-256
  `f304c64d4f8babb096efa94aa822346f304800e1011024f5347ab9ee9bd6e0ff`；
- 2 mm、1 Hz 平移：61 帧，final receipt SHA-256
  `3e3cc024090b8bdeb362aaddcc335b5903e7ef935aac6da480d010c01b8c2df5`；
- 2°、1 Hz 偏航：61 帧，final receipt SHA-256
  `9fb82f04a8e200dbe3b1cbc58bc0dfa64836f98adaa9ebe6e50aff257b00e082`；
- 三分支 closed-schema QC receipt SHA-256
  `1f837ce7c52ce80971adb981e121eace3c92968095193bb6db3519b39776e383`；
- 6-panel 彩色/灰度诊断图 receipt SHA-256
  `c0f669726973e98259e307b8c54afdd42c081a1bee993f84eaf34c040e80683d`。

### 5.6 S4-F：分辨率、时间步和物理参数验证

分辨率身份固定为：

- `dp=2.0 mm`：仅 smoke，不用于 1 mm 结论；
- `dp=1.0 mm`：development 主点；
- `dp=0.75 mm`、`dp=0.5 mm`：空间收敛点；
- C2/0.5 mm 若显存或时间预算 NO-GO，必须如实报告，不能删点后宣称收敛。

正式矩阵前预注册：

- 空间收敛方法和接受准则；
- time step/CFL 收敛方法；
- wall/contact/boundary、viscosity、DDT 的敏感性范围；
- C1/C2 参数转移规则；
- 每个点的重复次数和资源上限；
- primary 指标和置信/误差表达方式。

阶段 4 的 development-only 液体单独验证成功状态为：

```text
PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION
```

它要求 U3 settled、CPU/GPU parity、U4 本轮获授权 synthetic 分支及相应预注册检查全部通过；
该状态已经取得。

当前实际状态为：

```text
STAGE4_STATUS=PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION
STAGE4_LIQUID_ONLY_VALIDATION_COMPLETE=true
DEVELOPMENT_ONLY=true
FORMAL=false
PHYSICAL_FIDELITY_VALIDATED=false
STAGE5_STARTED=false
```

`dp=0.75/0.5 mm` 空间收敛、完整时间步/物性矩阵、C2 转移和实物参数验证仍未执行，因而不得把
本轮 PASS 写成 formal、production 或 physical-fidelity PASS。阶段 5 不会自动准入；开始前仍需
单独授权。本轮选定 R7 development replay 只需要独立冻结的 C1 geometry/fluid/mount 和不可变
bag transfer；未来 C2 或 physical replay 仍分别需要自己的几何/安装/实测参数，三者不得混用。

## 6. 阶段 5：R7 Gazebo 已执行运动的离线单向耦合

本阶段消费的是 R7 Gazebo odometry，不是实物小车轨迹。它回答的是：“固定的
DualSPHysics development case 能否完整重放 Gazebo 中实际走出的运动，并产生可审计液体结果？”
它不回答实物液面是否准确，也不改写原 R7 formal-simulation release。

### 6.1 Source domain、前置条件与身份上限

任何 GPU replay 前必须同时满足：

- 阶段 4 精确 parent 状态为 `PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION`，且
  QC/metrics/settled/parity receipts 的 path/hash 均未漂移；
- S5A0 已只读封存本次 exact source bag，topic/time/frame/inventory receipt 为 PASS；
- S5A1 已按 `R8-LIQUID-HANDOFF-ABI-v3` 为该 bag 生成并验收 exact transfer package；
- source domain 固定为 `SIM_R7_EXECUTED_GAZEBO_MOTION`；
- source attempt 的 success、`GOAL_TIMEOUT` 或 method-failure 标签来自冻结 R7 证据，不从
  topic 临时重判，也不被 replay 结果覆盖；
- 本轮只使用 C1，C1 geometry、mount、fluid、moving-boundary 和 settled-state 的兼容性逐 hash
  闭合；C2 明确不准入；
- 新 `transfer_id`、`replay_id`、profile、input/output receipt 和 output root 均 fresh；
- planned row 在读取任何新液体结果前冻结：默认只有 primary；optional pair 只有经新的明确授权
  后才能把分母从 1 改为 2。

阶段 4 当前冻结的是 C1M `Part_0901`，它只能证明既有 development chain。除非逐 hash 证明
C1 与 C1M 的 geometry/fluid/moving-boundary/particle-set 完全兼容，否则 C1 也必须另做 exact
settle。本轮不创建、替代或推断 C2 身份。

当前 bag 根没有 attempt manifest/postflight sidecar。允许先走 bag-only development lane，但必须：

```text
source_provenance=PARTIAL
formal_eligible=false
physical_primary_eligible=false
do_not_override_r7_release=true
```

若无法从冻结 R7 ledger/分析证据绑定某行的 source outcome，则写 `UNKNOWN`，不得用
`/spmpc/status`、goal distance 或 bag 长度自行改判。已知的 C2 failure row 保持在原 R7 证据链，
但不属于本轮 planned rows，不进入液体 replay 或阶段 6 分母。

### 6.2 S5A0：选定 bag 的只读封存和输入准入

S5A0 不运行 solver、不暴露 GPU、不启动 ROS/Gazebo，也不修改原目录。bag parser 必须断网并在
只读 bind/sandbox 中只看到本 goal 获准的 exact bag；只允许向独立 create-new audit root 写
receipt。默认只允许：

```text
/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag
```

可选 `Bslosh_b01` 不得随 primary 预读、预封存或自动加入；若用户之后启用第二行，它必须用自己的
exact path/hash 和 create-new receipt 重走同一 S5A0。执行顺序：

1. 冻结获准文件及所有父路径的 device/inode、挂载来源、只读策略和采集时间；
2. 核验 exact path，没有 basename 搜索、glob 或“最新文件”选择；
3. 核验 regular、`nlink=1`、无 symlink/special/`.active`、size、mode、mtime、SHA-256；即使源
   mode 为 `0755` 也只作为数据读取，sandbox/profile 明确禁止执行；
4. 只读解析该 ROS1 Bag V2 header、connection 和 chunk/index；冻结每个 connection 的 topic、
   type、MD5 和 message-definition SHA-256，拒绝同 topic 冲突定义；禁止 `rosbag reindex`、
   修复或缓存；
5. 核验 attempt ID 与 selected role（`PRIMARY` 或另行授权的 `OPTIONAL_PAIR`）精确一致；
6. 输出该 bag 的 topic/type/message-count/time-range、schema 和 parse error；
7. 检查 `/odom.header.stamp`、bag record timestamp、`/clock` 的范围/单调性，并枚举
   `/tf`/`/tf_static` frame graph；本步只报告，不生成 motion；
8. 重读原文件 identity/hash，原子发布 selected-bag manifest、topic census、time/frame report
   和 final receipt。

当前主机只有 ROS 2 Jazzy，不能假定 ROS1 `rosbag` CLI 或第三方 `rosbags` Python 包存在。
若实现纯 Python ROS1 reader，必须 create-new、离线、固定 hash，以最小/损坏/截断/冲突
connection/路径攻击 fixtures 通过后再读取选定 bag；解析时强制 timeout、RSS、单 chunk
解压尺寸和总输出上限。不得联网安装依赖，不得用解析失败作为重建 bag 索引的理由。

S5A0 final receipt 至少包含：

```text
source_root / mount identity / read_only_enforced
selected_role / attempt_id / exact absolute and relative path
selected_count=1 / bytes / path-size-mode-mtime-device-inode-nlink-SHA256
ROS1 format/index parse result
topic schema/types/MD5/message-definition hashes/exact counts
per-topic record/header/clock time ranges
frame inventory and anomaly list
source files before/after identity
files_written_under_source_root=0
source_bag_executed=false
status=S5A0_SELECTED_R7_BAG_SEALED_DEVELOPMENT_ONLY | exact failure
```

方案编写时记录的 selected size/mode/hash/duration 只能作为 expected witness；S5A0 必须独立
重算，匹配后才可 PASS。任何 bag hash 漂移、索引损坏、非预期 topic schema、路径越界或原 root
写入都 `STOP_AND_PRESERVE_EVIDENCE`。不要求也不允许以完成 S5A0 为理由解析其余 87 个 bag。

### 6.3 选定 bag 的真实 topic 合同

primary 与 optional pair 均来自 corpus 的 `SPMPC_NON_FIXED` schema。下表消息数是 88 行只读普查
得到的允许观察范围；S5A0 必须另行冻结选定 bag 的精确计数：

| Topic                        | ROS type                      | 每 bag 消息数 | 允许用途                                                            | 明确禁止                                 |
| ---------------------------- | ----------------------------- | ------------: | ------------------------------------------------------------------- | ---------------------------------------- |
| `/odom`                    | `nav_msgs/Odometry`         |    2326–3766 | **唯一 primary executed-motion 来源**；pose 为主，twist 作 QC | 不用别的 topic 替换                      |
| `/clock`                   | `rosgraph_msgs/Clock`       |  46532–75326 | 时间一致性 witness                                                  | 不直接驱动 solver motion                 |
| `/tf`                      | `tf2_msgs/TFMessage`        |   8652–13883 | 动态 frame/pose 交叉检查                                            | 不在与 odom 冲突时静默择优               |
| `/tf_static`               | `tf2_msgs/TFMessage`        |             1 | 静态 frame witness                                                  | 不得替代 container mount manifest        |
| `/cmd_vel`                 | `geometry_msgs/Twist`       |     939–1721 | 命令与执行差异诊断                                                  | **禁止作为液体运动输入**           |
| `/scout/global_path_fixed` | `nav_msgs/Path`             |      308–574 | 计划轨迹上下文                                                      | **禁止作为液体运动输入**           |
| `/scout/goal`              | `geometry_msgs/PoseStamped` |      308–574 | goal 上下文                                                         | 禁止补写/延长实际运动                    |
| `/slosh/height`            | `std_msgs/Float32`          |    2326–3766 | `H_proxy`；阶段 6 secondary comparator                            | 非实物 reference，禁止进 exporter/solver |
| `/spmpc/status`            | `std_msgs/String`           |       3–1302 | 事件 witness                                                        | 不得单独重判 source outcome              |

除共同 topic 外，选定 bag 必须存在 `/spmpc/debug/effective_config`、
`/spmpc/slosh_height`、`/spmpc/terminal/debug` 和 `/spmpc/terminal/mode`，因此本轮每个成功导出的
bag 都有 `H_proxy + H_modal`。任何 selected bag 出现 `FIXED_PROFILE` schema、required topic
缺失或未登记 extra，都由 S5A0 fail-closed；FixedProfile/C2 的 NA 规则只保留在 corpus 背景中，
不进入本轮 planned rows。

### 6.4 S5A1：`R8-LIQUID-HANDOFF-ABI-v3` 和 motion exporter

每个 attempt 对应一个独立 transfer package：

```text
incoming/<transfer_id>/
├── transfer_manifest.json
├── selected_bag_receipt_ref.json
├── source_bag_ref.json
├── source_outcome.json
├── topic_contract.json
├── time_window.json
├── frame_contract.json
├── odom_raw.csv
├── clock_alignment.csv
├── tf_alignment.json
├── motion.csv
├── solver_path.csv
├── motion_manifest.json
├── interpolation_qc.json
├── solver_path_qc.json
├── container_geometry_manifest.json
├── container_mount_manifest.json
├── fluid_spec.json
├── run_spec.json
└── checksums.sha256
```

`source_bag_ref.json` 必须绑定 selected-bag receipt、role、attempt ID、relative path、absolute path、
size 和 SHA-256。
默认不复制 bag；若未来要求可移植副本，必须另建 ABI variant，逐字节 copy 后核验 hash，原 bag
仍不得改写。

`odom_raw.csv` 保存原始证据，`motion.csv` 保存经过冻结坐标变换后的 canonical quaternion 轨迹，
`solver_path.csv` 才是 DualSPHysics `mvpathfile` 的可追溯、必须经过误差门禁的派生输入：

```text
odom_raw.csv:
bag_record_t_ns,odom_header_t_ns,frame_id,child_frame_id,
x_m,y_m,z_m,qx,qy,qz,qw,vx_m_s,vy_m_s,vz_m_s,wx_rad_s,wy_rad_s,wz_rad_s

motion.csv:
t_s,x_rel_m,y_rel_m,z_rel_m,qx_rel,qy_rel,qz_rel,qw_rel

solver_path.csv (首行可用 # 注释，数据恰为 7 个数值字段):
t_s,x_rel_m,y_rel_m,z_rel_m,ang1_deg,ang2_deg,ang3_deg
```

若要保留 covariance，使用 closed-schema companion 文件或明确扩展 v3 minor version，不得临时增加
列。绝对容器位姿和实际送入 solver 的相对位姿表达为：

```text
T_odom_container(t) = T_odom_base_from_odom(t) × T_base_container_from_frozen_manifest
T_rel(t) = inverse(T_odom_container(t0)) × T_odom_container(t)
```

`t0` 是冻结窗口的第一条 canonical pose；因此 `T_rel(t0)=I`。禁止把 odom 的绝对初始位置直接
送入 solver，否则会造成容器首次更新瞬移。完整 `T_odom_container` 仍保留在证据文件中，solver
只消费 `T_rel`。

时间和插值合同必须在导出前冻结：

- primary time 唯一为 `/odom.header.stamp`；
- bag record timestamp 与 `/clock` 仅作 alignment witness，不能在 primary 失败时静默回退；
- `t_s` 从冻结窗口起点归零并严格递增；重复、回退、reset、超限 gap 一律 fail；
- position 只允许预注册的线性插值，orientation 只允许归一化 quaternion SLERP；
- quaternion 必须 finite、单位化、符号连续；不得 Euler 逐轴插值；
- 不做平滑、去噪、路径投影、补终点或基于液体结果的重采样；
- interpolation grid、最大 gap、clock/header/record mismatch 容差先写入
  `time_window.json`，再读取液体结果。

当前 DualSPHysics 5.4 源码在 `JMotion.cpp` 的 `mvpathfile` 分支消费
`time,x,y,z,ang1,ang2,ang3`，并在相邻行间对 position 和 Euler angles 线性插值。因此 exporter
必须显式冻结以下 solver bridge，不能依赖默认值：

```text
motion_element=mvpathfile
fields=7
fieldtime/fieldx/fieldy/fieldz/fieldang1/fieldang2/fieldang3=0/1/2/3/4/5/6
anglesunits=degrees
axes=ZYX
intrinsic=true
movecenter=true
rotation_center=exact_case_container_reference_from_frozen_geometry_manifest
angle_order=continuous_yaw_Z,pitch_Y,roll_X
canonical_orientation=normalized_quaternion_SLERP
```

先在 canonical grid 上做 quaternion SLERP，再把每个 `T_rel` 转为固定 `intrinsic ZYX`，对 yaw
做连续 unwrap 后生成 `solver_path.csv`；不得直接对原始 Euler 角插值。生成后必须从
`solver_path.csv` 重建 rotation matrix/quaternion，与 canonical `motion.csv` 逐时刻 round-trip
比较，并在全部 solver query/output 时刻模拟 DualSPHysics 的 Euler 线性插值；最大位置误差、最大
角距离、gimbal-lock/非有限值和首行 identity 均须在读取液体输出前冻结阈值并 PASS。最终还要用
`executed_boundary_motion.csv` 复核真实 moving boundary，而不能只证明 CSV 自洽。

窗口合同固定为：“first-effective-motion 前固定 pre-roll + 完整已执行运动 + source terminal/timeout
后的固定 recorded tail”。first-effective-motion 阈值、pre-roll 和 tail 时长必须在导出前冻结。
若 bag-only provenance 无可信 terminal sidecar，保守消费到最后一个合法 `/odom` 样本并标记
`terminal_provenance=PARTIAL`；不得根据 goal 是否到达裁短。额外 solver relaxation tail 只能由
`run_spec.json` 预注册为 final-pose zero hold，并与 source recorded tail 分栏记录。

`/cmd_vel`、path、goal、`H_proxy`、`H_modal`、status 及 FixedProfile lifecycle topics 不得进入
motion exporter 或 solver argv。exporter 必须有负测证明：即使这些 topic 与 odom 冲突，也只会
FAIL/QC，而不会改变 `motion.csv`。

本轮 package 必须绑定 C1 的 `container_geometry_manifest.json` 和
`container_mount_manifest.json`，并证明它们与 chosen settled state 兼容。C2 package 不生成；
禁止以 C1M/C1 代跑 C2。H1 轨迹预计累计平移约 9 m，S5A1 还必须报告相对轨迹包围盒和最大位移；
S5B0 在完整运行前以它预估 moving-domain/cell-grid、峰值显存和边界越界风险。不得裁短、缩放轨迹
或把 9 m 运动偷偷投影为原地晃动来绕过资源门禁。

### 6.5 单主行 replay，最多增加一个同层配对行

本轮不做 pilot 或完整矩阵。执行层级固定为：

| 层级 | 内容                                                          | Solver/GPU                         | 通过后允许          |
| ---- | ------------------------------------------------------------- | ---------------------------------- | ------------------- |
| S5A0 | 只读封存 primary exact bag；optional 如获新授权则另发 receipt | 否                                 | 进入对应 exporter   |
| S5A1 | 为已封存的 selected bag 生成并验收 ABI-v3 transfer            | 否                                 | 进入对应 replay     |
| S5B0 | primary`Bsmooth_b01` 完整 motion + tail                     | 是，单独授权                       | 阶段 6 单行回放/QC  |
| S5B1 | optional`Bslosh_b01` 完整 motion + tail                     | 是，**默认不运行，另行授权** | 阶段 6 同层配对比较 |

精确行集为：

```text
required planned row:
  attempt_id=SIM-S1_CORE_H1_C1_Bsmooth_b01_r01
  role=PRIMARY_BASELINE
  bag=/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag

optional planned row:
  attempt_id=SIM-S1_CORE_H1_C1_Bslosh_b01_r01
  role=OPTIONAL_METHOD_PAIR
  bag=/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bslosh_b01_r01/capture.bag
  default=NOT_RUN
```

两行都属于 `SIM-S1_CORE / H1 / C1 / b01`，因此可以复用同一份经过兼容性核验的 C1
geometry、mount、fluid 和 settled-state 内容，但每行必须 fresh clone、fresh replay ID 和独立
receipt。primary 的 planned denominator 固定为 1；只有用户明确启用 optional 后，paired analysis
的 planned denominator 才固定为 2。不得自动增加 B0、SmoothMatch、FixedProfile、其他 block、
L1、C2 或 failure row。

primary 的结果足以交付“一条小车已执行运动驱动一箱液体”的动态回放和数值 QC。optional 的唯一
新增价值是 `Bsmooth` 与 `Bslosh` 的同层配对比较；它不是 primary PASS 的必要条件，也不得在看到
primary 曲线后改成别的 bag。6-row pilot、S5B2、full-88 scheduler/closure、88-run ETA 和全矩阵
磁盘预算均为 `OUT_OF_SCOPE`。

### 6.6 唯一 replay 生命周期

```text
只读验证 exact ABI-v3 transfer package
  → 核验 C1 exact geometry/mount/fluid/settled compatibility
  → create-new safety preflight receipt
  → 从同 stratum 冻结 settled state 创建 fresh clone
  → create-new <replay_id>.partial
  → 加载 exact runtime profile并只暴露最小 GPU devices
  → 运行完整 executed motion + recorded tail + 预注册 solver tail
  → finally 卸载 profile并核验 process/mount/device/lock/sudo 零残留
  → 输出 inventory/QC/hash
  → 原子发布 final result
  → append-only liquid secondary ledger
```

每个 source attempt 必须单独 replay。可以复用同一 geometry/parameter identity 的 settled-state
内容，但每个 replay 必须 fresh clone；不得复用另一行的动态液体末态。source bag、R7 release
ledger 和原 88 行 outcome 永远只读。primary 与 optional 之间也不得共享 `.partial` root、动态 Part
末态、日志、锁或 receipt。

### 6.7 输出、验收和状态

最小结果包：

```text
outgoing/<replay_id>/
├── result_manifest.json
├── executed_boundary_motion.csv
├── gauge_zsurf.csv
├── slosh_height.csv
├── summary.json
├── qc_report.json
├── resource_samples.jsonl
├── build_and_runtime.txt
├── solver.stdout.log
├── solver.stderr.log
└── checksums.sha256
```

必须证明：

- 完整 motion 和全部 tail 已消费，无倒退、重复、断点、非法 quaternion 或未登记插值；
- boundary 实际运动与 motion manifest 在冻结容差内；
- C1 的 geometry/mount/settled identities 正确，未引入 C2 或替代 case；
- 所有 Gauge 时间槽完整，invalid 比例低于预冻结上限；
- 粒子数/质量/ID 完整，`Nout=0`，无 NaN/Inf、泄漏和越界；
- GPU、CUDA、Xid、温度、显存、资源和 timeout 门禁通过；
- result manifest 绑定 transfer、settled state、source bag、binary、profile、config 和全部输出 hash；
- 原 source bag、ABI package 和 settled state 在前后未改变；
- replay receipt 单独保留 source outcome；replay PASS 不把 `GOAL_TIMEOUT` 改成 success。

单行成功状态只能是：

```text
U5_SIM_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY
```

primary PASS 只覆盖 `Bsmooth_b01` 这一行，可发布：

```text
S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY
planned_denominator=1
```

若 optional 经新授权并独立 PASS，才可另外发布：

```text
S5B1_OPTIONAL_PAIRED_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY
paired_planned_denominator=2
```

两个状态都只证明各自 R7 Gazebo executed-motion 被液体 solver 完整离线重放，不是 corpus closure，
也不证明实物液面准确或双向车液动力学。

## 7. 阶段 6：R7 bag 回放、液体模型比较与动态交付

### 7.1 输入冻结与证据链隔离

分析和可视化只能消费 finalized manifest，不扫描“最新目录”。开始前冻结：

- S5A0 selected-bag receipt、source bag、source outcome、transfer、replay 和 GPU run IDs/hashes；
- planned-row 集合及固定分母：primary-only 为 1，explicit paired 为 2；两者身份不得混写；
- `/odom.header.stamp` 时间零点、first-effective-motion、motion、recorded/solver tail 和网格；
- 坐标变换、probe 集、无效值、滤波、缺失和 NA 规则；
- primary/secondary metrics、比较容差、配对和排序规则；
- 若未来加入实物 reference：RGB ROI、标定、同步/延迟和不确定度；
- 重复次数、异常 run 排除条件和统计汇总方法。

原 R7 formal-simulation release 与本次 liquid replay 是两条证据链。阶段 6 只能写 create-new
comparison manifest 和 liquid secondary ledger；不得编辑 R7 的 88 行 outcome、append-only chain
或 formal 结论。

当前 bag 没有 RGB、IMU 或实物液面传感器 topic，且没有物理 reference sidecar。因此可以完成
R7 bag/model development comparison，但：

```text
PHYSICAL_REFERENCE_PENDING=true
PHYSICAL_FIDELITY_VALIDATED=false
```

### 7.2 统一 DualSPHysics 液面指标

圆周 probe 的界面高为 `h_i(t)`，冻结静态液深为 `h0`：

```text
eta_i(t)            = h_i(t) - h0
H_crest(t)          = max_i eta_i(t)
H_abs(t)            = max_i |eta_i(t)|
H_peak_to_peak(t)   = max_i h_i(t) - min_i h_i(t)
```

DualSPHysics development primary candidate 为 `H_crest(t)`；单个最高粒子 `maxz` 只能做
splash/QC。若未来做实物 fidelity，最终 physical primary 必须与 RGB/液位传感器测量语义对齐后
另行冻结，不能沿用本轮选择自动获得资格。

每个 replay 至少报告：

- first 15 s、完整 motion window、recorded tail 和 solver tail 的 peak/p95/RMS；
- 主频、峰值时刻、相位和衰减率/阻尼；
- 每个 probe 原始 `zsurf` 和 missing/invalid 比例；
- 粒子质量守恒、泄漏、NaN、越界和 splash QC；
- backend/device/resolution/time-step/geometry/config 和资源身份。

### 7.3 Bag 信号可用性与 NA 合同

| 信号                                         |                       覆盖 | 语义                            | 阶段 6 角色                     |
| -------------------------------------------- | -------------------------: | ------------------------------- | ------------------------------- |
| DualSPHysics`H_crest/H_abs/H_peak_to_peak` |         仅已成功 replay 行 | 新 GPU 液体模型结果             | 本轮 liquid development primary |
| `/slosh/height` = `H_proxy`              | primary 1/1；paired 时 2/2 | R7 仿真模型代理，原分析按 m→mm | secondary comparator            |
| `/spmpc/slosh_height` = `H_modal`        | primary 1/1；paired 时 2/2 | 控制器/模型族信号               | secondary comparator            |
| RGB/独立液位/实物 reference                  |        selected 0/1 或 0/2 | 选定 bag 中不存在               | `PHYSICAL_REFERENCE_PENDING`  |

`H_proxy` 和 `H_modal` 与 R7 controller/plant model 同属仿真证据，不是独立 liquid truth。
它们不能作为 motion exporter 输入、solver forcing、调参目标或 physical reference。

### 7.4 比较设计

#### A. GPU 与 CPU

阶段 4 的固定 synthetic case 已完成 CPU/GPU parity，它只作为 backend parent evidence。本轮不为
49 s executed-motion 追加 CPU replay，也不得把阶段 4 parity 冒充所选 bag 的逐轨迹 CPU/GPU
一致性；selected-bag CPU comparison 明确为 `OUT_OF_SCOPE`。若未来确需该比较，必须另立 goal、
按同一 geometry/motion/参数/初态/输出槽运行并预冻结容差。

#### B. DualSPHysics 与 R7 `H_proxy/H_modal`

先用共同的 `/odom.header.stamp` 运动窗口和预注册 alignment policy 对齐，再报告 amplitude、
frequency、damping、phase、相关/误差。比较必须：

- 以 planned row 为统计单位和固定分母，不以 topic message 为样本量；
- primary-only 只做 `Bsmooth_b01` 内 DualSPHysics 对 H_proxy/H_modal 的对齐，不声称
  cross-method ranking；
- 只有 optional 也成功时，才做同一 `SIM-S1_CORE/H1/C1/b01` 的
  `Bsmooth ↔ Bslosh` 配对并报告 cross-method ranking；
- 不跨 H1/L1、C1/C2/block 混配，不加入 FixedProfile 或 failure row；
- 缺 replay、无效 Gauge 或 source provenance partial 分别报告，不从分母静默删除；
- 禁止看结果后改 window、filter、probe、单位、行集或只展示正向 condition。

这类结果只能称为 `R7_BAG_REPLAY_AND_MODEL_COMPARISON`。DualSPHysics 与 H_proxy/H_modal 接近，
或 Bslosh 的排序一致，都不能证明真实液面改善。

#### C. 未来仿真与实物实验

实物 reference 只能来自另行冻结的 RGB liquid-height 或独立液位传感器 pipeline。必须按五维报告：

```text
amplitude
frequency
damping
phase
cross-case / cross-method ranking
```

每一维分别报告偏差、测量/重复不确定度、接受阈值和 PASS/FAIL。当前选定 bag 不具备该输入，
所以阶段 6 本轮不得生成 physical comparison PASS。

### 7.5 动态回放交付

每个选定 replay 生成：

- 基于 finalized solver 帧的 MP4；
- 用于快速审阅的 GIF 或等价低带宽预览；
- 带容器坐标、时间、粒子类别和液面 probe 的静态关键帧；
- `H_crest/H_abs/peak-to-peak` 与已登记 H_proxy/H_modal 的时间序列图；
- R7 model-comparison 图；如只有 primary，不画伪造的 paired/CPU 对比；实物列明确 `PENDING`；
- 可机读 CSV/JSON、绘图数据、工具/参数 hash 和 checksums；
- append-only comparison manifest、evidence index 和 liquid secondary-ledger entry。

可视化不得未登记地平滑、裁剪或重采样原始数据；GIF/MP4 不是数值证据事实源，事实源仍是
finalized BI4/Gauge/CSV/JSON、source bag 及其 hash。

### 7.6 阶段 6 结论

若预注册的 R7 行集、replay 和 model comparison 全部闭合，本轮最大状态为：

```text
S6_PRIMARY_R7_BAG_REPLAY_AND_MODEL_COMPARISON_PASS_DEVELOPMENT_ONLY
planned_denominator=1
PHYSICAL_REFERENCE_PENDING=true
```

若 optional 经独立授权、replay 和 QC 后也闭合，才可追加
`S6_PAIRED_R7_BAG_REPLAY_AND_MODEL_COMPARISON_PASS_DEVELOPMENT_ONLY / planned_denominator=2`。

这不是总体 physical-fidelity PASS。只有未来独立 physical source domain、reference、校准和五维
验收完成，才可另立 physical-fidelity goal；只有 P0–P3 全部通过并完成 P4 formal 设计复审后，
才可决定是否申请新的 formal release。本文不授权这些步骤。

## 8. 分阶段实现工作包

后续实现必须使用 versioned create-new 文件；不得修改冻结 GPU G1/A 文件、既有 CPU receipts、
历史 raw output、原 88 个 bag 或本文父方案。

| 工作包                         | 需实现的最小组件                                                                            | 静态门禁                                                                              | 系统/运行授权                               |
| ------------------------------ | ------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- | ------------------------------------------- |
| S3-A runtime contract          | policy、closed schema、gate、tests、bootstrap/supervisor、AppArmor template/exact instance  | AST、schema、mock/负测、self-check、non-loading AppArmor query                        | 不授权执行                                  |
| S3-B one-shot smoke            | start/final/failure receipts、resource/Xid monitor、output QC                               | parent/hash 与 fresh-root preflight                                                   | exact profile/device/candidate 单独授权     |
| S4-A DDT QC                    | read-only parser、closed result schema、baseline comparator、tests                          | fixture/negative tests、hash/inventory                                                | 只读，不运行 solver                         |
| S4-B/C settle                  | numerical manifest/schema、fresh-run gate、settle/restart QC                                | 参数/阈值/容差冻结                                                                    | 每个 run 单独授权                           |
| S4-D/E/F parity/synthetic      | Idp comparator、motion builder、Gauge/height extractor、matrix manifest                     | synthetic tests、no-cherry-pick negative tests                                        | 每个矩阵/资源预算单独授权                   |
| S5A0 selected-bag intake       | ROS1 Bag V2 只读 reader、selected topic/time/frame schema、receipt/tests                    | exact path/hash、损坏索引/topic-conflict/path/symlink/execute 负测；原 root 零写入    | 默认只读 primary，不运行 solver/GPU         |
| S5A1 ABI-v3 exporter           | package schema、odom-only exporter、relative transform、quaternion-to-`mvpathfile` bridge | cmd/path/H_proxy/status 冲突负测、C1 identity、SLERP/Euler round-trip、closed package | 只读 selected bag；只写 create-new transfer |
| S5B0 primary replay            | replay gate、profile、resource/Xid monitor、moving-domain、boundary/Gauge QC                | exact primary transfer/geometry/settled/profile/hash 与 fresh-root preflight          | `Bsmooth_b01` 一个 exact replay 单独授权  |
| S5B1 optional pair             | 复用组件但 fresh root/clone/receipt；paired manifest                                        | 同 stage/path/C1/block、planned denominator=2、no result-driven row swap              | `Bslosh_b01` 默认不运行，另行授权         |
| S6A selected-signal extraction | H_proxy/H_modal reader、provenance schema、alignment QC                                     | primary 1/1；paired 时 2/2；单位/时间负测                                             | 只读                                        |
| S6B visualization/comparison   | finalized-result reader、plot/animation、selected R7 comparator、report schema              | golden fixture、pairing/axis/unit/time/hash tests                                     | 只读分析                                    |
| S6C physical comparison        | future physical ABI、RGB/level reference、calibration/uncertainty                           | 独立 physical contract 和五维门禁                                                     | 当前不授权                                  |

每个工作包普通实现或测试失败应在自身范围内完成“定位证据 → 最小修复 → 聚焦测试 → 完整回归”；
不得通过放宽 closed schema、路径、权限、阈值或比较窗口让测试通过。

## 9. 推荐 goal 顺序与授权断点

下面是完整推荐目标顺序，不是当前授权。阶段 1–4 的 development-only 范围已完成，
不得仅为进入本文而重建或重做静态审计：

1. `S3A_FREEZE_GPU_RUNTIME_SMOKE_CONTRACT`：**已完成**；
2. `S3B_GPU_RUNTIME_SMOKE_ONE_SHOT`：**已完成**，V6 `PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY`；
3. `S4A_READ_ONLY_DDT_QC`：**已完成 / FAIL**，DDT ramp 未被选中；
4. `S4B_FREEZE_LIQUID_NUMERICAL_CONTRACT`：**已完成**；最终采用原仓库支持的
   `Shifting=None / CFL=0.1 / ViscoArtificial=0.3` development-only 合同；
5. `S4C_U3_SETTLE_COLD_AB_RESTART`：**已完成 / PASS**，状态为 `U3_SETTLED_STATE_FROZEN`；
6. `S4D_CPU_GPU_PARITY_AND_U4_SYNTHETIC`：**已完成本轮授权范围 / PASS（development-only）**；
7. `S5A0_SEAL_PRIMARY_R7_BAG_READ_ONLY`：**当前唯一下一 goal**；只读封存 exact
   `SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag`，不读取其余 bag、不生成 motion、不运行 solver；
8. `S5A1_BUILD_PRIMARY_R8_LIQUID_HANDOFF_ABI_V3_TRANSFER`：实现并验证 odom-only exporter、
   `T_rel`、`solver_path.csv` 和 `mvpathfile` round-trip；
9. `S5B0_RUN_PRIMARY_C1_COUPLED_REPLAY`：只运行 primary，单独 GPU/profile 授权；
10. `S6A_RENDER_AND_QC_PRIMARY_REPLAY_DEVELOPMENT_ONLY`：只读生成一条动态回放、数值 QC、
    H_proxy/H_modal 对齐和证据包；做到这里已经满足用户的单 bag 目标；
11. `S5B1_OPTIONAL_RUN_PAIRED_C1_REPLAY`：仅用户明确要求第二个 bag 时，先为 exact
    `SIM-S1_CORE_H1_C1_Bslosh_b01_r01/capture.bag` 重走 S5A0/S5A1，再单独授权 GPU replay；
12. `S6B_OPTIONAL_COMPARE_BSMOOTH_VS_BSLOSH_DEVELOPMENT_ONLY`：只有两行 finalized 后执行；
13. `S6C_PHYSICAL_REFERENCE_COMPARISON`：未来独立 physical 输入和授权；当前不可进入；
14. `P4_FORMAL_INTEGRATION_REVIEW`：仅在对应 formal 前置全部通过后由用户另行决定是否创建。

阶段成功不能自动授权下一 goal。涉及 profile lifecycle、sudo、GPU device、candidate/solver 执行、
跨机输入或大规模矩阵时，必须列出 exact path/name/hash/argv/资源上限并取得新的明确授权。
S5A0/S5A1 的只读或派生文件权限不得被解释成 S5B0 的 GPU runtime 授权；primary 的权限和 PASS
也不得被解释成 optional bag 的读取、导出或执行授权。6-row pilot、S5B2 和 full-88 在本版中没有
goal，也没有授权路径。

## 10. 时间和资源规划原则

阶段 3–6 不沿用旧 GPU 构建 campaign 的 T0，也不共享一个“8 小时”时间盒。每个实际 goal 创建时
根据当时资源和前一阶段 benchmark 冻结独立 T0、candidate deadline 和 final deadline。

- 阶段 3 只含 1 s C1M smoke，以固定 wall timeout 保护；
- 阶段 4 的 settle、收敛和参数矩阵可能远超 8 小时，必须先估算每点粒子数、显存、磁盘和 wall time；
- 阶段 5 每个 R7 executed-motion replay 独立预算，不静默缩短 motion 或 tail；
- 阶段 6 只读分析与可视化单独预算，不能为节省空间自动删除 raw BI4/reference。

primary 记录时长约 `49.409 s`，optional 约 `49.549 s`，但 bag 时长不等于 GPU wall time。
根据当前同类 C1M GPU 实测约 `78–80 × physical time` 的粗略倍率，在不含新增 relaxation tail、
首次工程调试和 exporter/QC 的情况下：primary solver 约 `64–66 min`，两行 solver 合计约
`129–132 min`（约 `2.2 h`）。每个 raw 输出按约 `0.4 GB` 预留，实际仍由 S5B0 报告：

```text
solver_wall_seconds / replay_physical_seconds
output_bytes / replay_physical_seconds
peak_vram / peak_rss / max_temperature
fixed_per-run_overhead
```

首次从 ROS1 reader、exporter、C1 compatibility、moving-domain 到动画的工程闭环仍可能需要半天至
一天；上面 65 min 只是 solver 量级，不是承诺的端到端完成时间。S5B0 取得实测后再更新 optional
ETA；不因赶时间缩短 motion/tail、降低验证或自动执行第二行。C2、6-row 和 88-row 的时间/磁盘
估算均不属于本轮。

## 11. 最终验收检查表

### 阶段 3

- [X] fresh GPU candidate 和阶段 2 static-audit receipt 匹配；
- [X] exact runtime contract/profile 已冻结并单独授权；
- [X] C1M 1 s、21 Part BI4/30 文件、9,078 粒子、`Nout=0` 全部通过；
- [X] CUDA/Xid/资源/profile/process/mount 零异常、零残留。

### 阶段 4

- [X] 既有 DDT-ramp QC 完成，并按预注册门槛拒绝候选；
- [X] CFL=0.2/0.1 唯一 delta、同初态、同 GPU 的配对运行和逐 Id QC 完成；
- [X] 早期 CFL 分支 FAIL 回执保留，未被后续结果覆盖或冒充 PASS；
- [X] `ViscoArtificial=0.3` cold-A/B、repeatability、restart 和 settled-state 全部通过；
- [X] speed/KE/位置/coverage/粒子/密度达到最终冻结的 development-only 阈值；
- [X] CPU/GPU parity 容差在看结果前冻结并通过；
- [X] 零回放及非零 synthetic translation/yaw/decay 与 16-sector height extractor 通过；
- [X] 最终 closed-schema QC 与彩色/灰度诊断图发布；
- [ ] 分辨率、完整时间步/物性矩阵和实物参数验证完成（不属于当前 development-only PASS）。

### 阶段 5

- [ ] primary exact ROS1 Bag V2 只读封存、文件 hash、索引和 `SPMPC_NON_FIXED` topic schema 通过；
- [ ] 原 bag root 零写入、源 bag 未执行；S5A0 selected-bag receipt 原子发布；
- [ ] primary `R8-LIQUID-HANDOFF-ABI-v3` package 逐文件/closed-schema 验收；
- [ ] primary time/motion 只来自 `/odom.header.stamp`/pose；bag time、`/clock`、`/tf` 仅作 witness；
- [ ] `/cmd_vel`、path、goal、H_proxy/H_modal/status 未进入 exporter/solver；
- [ ] `T_rel(t0)=I`，canonical quaternion/SLERP 到 `mvpathfile` Euler path 的 round-trip 通过；
- [ ] C1 geometry/mount/fluid/settled compatibility identity 通过，未引入或冒充 C2；
- [ ] pre-roll、完整 motion、timeout/terminal 和 tail 未被裁短；
- [ ] primary planned denominator=1，exact `Bsmooth_b01` 在看结果前冻结并独立裁决；
- [ ] optional 默认未读取/未运行；若获新授权，exact `Bslosh_b01` 单独封存、导出、运行和裁决；
- [ ] fresh settled-state clone、result package、profile/process/mount/device 零残留通过；
- [ ] liquid 结果未反馈控制器、未修改原 R7 release/source outcome。

### 阶段 6

- [ ] 动态回放由 finalized solver 帧生成并绑定 hash；
- [ ] primary 的 H_proxy/H_modal 各为 1/1 secondary；若 paired，则各为 2/2；
- [ ] planned-row 分母为 1，或在 optional 明确授权后为 2；没有把 6/88 当作本轮分母；
- [ ] 单 bag 报告不声称 cross-method ranking；只有 paired 才比较 Bsmooth/Bslosh；
- [ ] 阶段 4 CPU/GPU parity 只作为 parent，不伪称 selected trajectory CPU comparison；
- [ ] probe/filter/window/时间同步在看结果前冻结；
- [ ] amplitude/frequency/damping/phase 的单行 R7 model comparison 完成；paired 时再增加 ranking；
- [ ] comparison manifest、evidence index、secondary ledger 和 checksums 完整；
- [ ] `PHYSICAL_REFERENCE_PENDING=true`，未伪造 RGB/IMU/实物液面比较；
- [ ] 未把 development 结果表述为 formal、production 或 physical-primary。

## 12. 当前唯一允许的下一步

本文完成了阶段 3–6 的离线规划，并同步阶段 1–4 的不可变证据与 R7 corpus 的只读背景普查。
阶段 4 的 repeatability、parity 和本轮 U4 synthetic 已完成，无需重跑。阶段 5/6 尚未开始，也不会
因阶段 4 PASS 或“bag 已复制到本机”自动授权。

当前唯一下一步是 S5A0：只读封存/验证 primary exact bag
`/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag`。它不要求运行
GPU，不读取其余 87 个 bag，也不要求先伪装成实物输入。S5A0 PASS 后才允许 S5A1 为这一行生成
ABI-v3 transfer；C1 geometry/mount/settled compatibility 未冻结时，可以只读验收/导出 odom，
但 solver replay 必须保持 `NOT_ADMITTED`。optional `Bslosh_b01` 默认不读、不导出、不运行。

```text
PLAN_STAGE_3_6_STATUS=STATIC_CONTINUATION_PLAN_COMPLETE
GPU_BUILD_PASS=true
GPU_STATIC_AUDIT_PASS=true
GPU_RUNTIME_STATUS=PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY
STAGE4_EXECUTION_AND_ADJUDICATION_COMPLETE=true
STAGE4_LIQUID_ONLY_VALIDATION_COMPLETE=true
STAGE4_STATUS=PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION
U3_ACCEPTANCE=U3_SETTLED_STATE_FROZEN
CPU_GPU_PARITY_STATUS=PASS_CPU_GPU_PARITY_DEVELOPMENT_ONLY
U4_SYNTHETIC_STATUS=ZERO_TRANSLATION_YAW_PASS_DEVELOPMENT_ONLY
DEVELOPMENT_ONLY=true
PHYSICAL_FIDELITY_VALIDATED=false
R7_BAG_ROOT=/home/zrj/slosh_bags/matrix_bags
R7_BAG_CORPUS_OBSERVED=88_ROS1_BAG_V2
EXECUTION_SCOPE=ONE_PRIMARY_PLUS_ZERO_OR_ONE_OPTIONAL_PAIR
PRIMARY_ATTEMPT=SIM-S1_CORE_H1_C1_Bsmooth_b01_r01
PRIMARY_BAG=/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag
PRIMARY_BAG_SEALED=false
OPTIONAL_SECOND_ATTEMPT=SIM-S1_CORE_H1_C1_Bslosh_b01_r01
OPTIONAL_SECOND_BAG=/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bslosh_b01_r01/capture.bag
OPTIONAL_SECOND_DEFAULT=NOT_RUN
PLANNED_DENOMINATOR=1
MAX_PLANNED_DENOMINATOR=2
FULL_CORPUS_INTAKE_OUT_OF_SCOPE=true
SIX_ROW_PILOT_OUT_OF_SCOPE=true
FULL_88_REPLAY_OUT_OF_SCOPE=true
C2_REPLAY_OUT_OF_SCOPE=true
SOURCE_DOMAIN=SIM_R7_EXECUTED_GAZEBO_MOTION
SOURCE_PROVENANCE=PARTIAL_BAG_ONLY
HANDOFF_ABI=R8-LIQUID-HANDOFF-ABI-v3_NOT_MATERIALIZED
U5_STARTED=false
PHASE5_ADMITTED=false
CURRENT=STOP_AFTER_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION_PASS
NEXT=S5A0_SEAL_PRIMARY_R7_BAG_READ_ONLY
PHYSICAL_REFERENCE_PENDING=true
```

## 13. 权威参考

1. [RTX 5080 GPU 8 小时快速构建方案](./20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md)
2. [DualSPHysics 物理液体接入 SIM-R8 方案](./20260805_DualSPHysics物理液体接入SIM-R8方案.md)
3. [Ubuntu 24.04 液体仿真电脑任务与数据交接说明](./20260806_Ubuntu24.04液体仿真电脑任务与数据交接说明.md)
4. [RTX 5080 GPU 构建阶段进度与下一步（当前状态页）](./20260810_RTX5080_DualSPHysics_GPU构建阶段进度与下一步.md)
5. [SIM-MECHANISM-R7 40/64/88 结果分析与正式实物流程对照](../仿真对比试验分析/20260804_SIM-MECHANISM-R7_40_64_88结果分析与正式实物S-MPCC流程对照.md)
6. [SIM-R8 源码隔离、仿真实验矩阵与执行方案](../仿真对比试验/20260804_SIM-R8源码隔离_仿真实验矩阵与执行方案.md)
