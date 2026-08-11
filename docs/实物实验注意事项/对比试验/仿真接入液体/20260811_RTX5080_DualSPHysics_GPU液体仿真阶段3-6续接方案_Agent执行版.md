# RTX 5080 DualSPHysics GPU 液体仿真阶段 3–6 续接方案（Agent 执行版）

## Material Passport

- Origin Skill: `academic-research-suite / experiment-agent`
- Origin Mode: `plan`
- Origin Date: `2026-08-11 Asia/Shanghai`
- Verification Status: `STAGES_1_2_3_PASS / STAGE_4_LIQUID_ONLY_DEVELOPMENT_VALIDATION_PASS`
- Version Label: `gpu_liquid_stage_3_6_continuation_v5_status_sync`
- Last Status Sync: `2026-08-11 23:09:24+08:00`

> 本文是 create-new 的续接方案，不修改、不取代也不重新冻结
> [RTX 5080 GPU 8 小时快速构建方案](./20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md)。
>
> 父方案 SHA-256：
> `9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d`
>
> 本文只定义阶段 3–6 的执行合同和验收边界，不授权构建、加载 AppArmor、暴露 GPU、
> 执行 candidate、运行 solver、复制实际运动数据或启动 ROS/Gazebo。

## 0. 结论与当前起点

完整 GPU 液体仿真按六阶段划分：

| 六阶段编号 | 内容 | 对应既有项目阶段 | 本文是否覆盖 |
| --- | --- | --- | --- |
| 1 | GPU 程序构建 | GPU 构建方案 G0–G3/G5 | **PASS**；candidate 已构建并收紧为 `0400` |
| 2 | 静态审计 | GPU 构建方案 G4/G6/G7 | **PASS**；`sm_120`/PTX/对象/依赖已静态核验 |
| 3 | GPU 运行冒烟测试 | 新 GPU runtime admission；U3 1 s C1M smoke | **PASS**；V6 one-shot smoke 和 create-new postvalidation/QC 已通过 |
| 4 | 液体单独验证 | U3 settled state、CPU/GPU parity、U4 合成运动 | **PASS（development-only）；未验证实物保真** |
| 5 | 小车与液体耦合 | U5 实际运动离线回放 | **本文完整定义** |
| 6 | 回放与对比 | 动态可视化、CPU/GPU、SIM/实物五维比较 | **本文完整定义** |

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
U5=NOT_STARTED
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

因此当前实际顺序是：

```text
阶段 1 构建 PASS
  → 阶段 2 静态审计 PASS
  → 阶段 3 GPU functional smoke PASS
  → 阶段 4 液体单独验证 PASS（development-only，当前停止点）
  ↛ 阶段 5 实际运动单向回放（未启动；需实物输入和单独授权）
  ↛ 阶段 6 回放与对比（未准入）
```

任何前一门禁失败都不得借用后一阶段的权限、数据或结果来“补证”。

## 1. 术语和系统边界

### 1.1 “小车与液体耦合”的准确含义

本文阶段 5 的“耦合”固定为 **two-pass、离线、单向运动耦合**：

1. Gazebo/小车 attempt 先独立完成并关闭 bag；
2. 从不可变 bag 的 `/odom`、`/tf`、`/tf_static` 导出容器实际位姿；
3. DualSPHysics 在另一阶段离线重放该容器运动；
4. 液体结果只进入 append-only secondary ledger 和事后比较报告。

明确禁止：

- 在 Gazebo case 运行时并发启动 DualSPHysics；
- 用 `/cmd_vel`、计划轨迹或控制器状态代替已执行运动；
- 把液面或液体力反馈给小车、控制器、planner 或底盘；
- 用液体 replay 的结果改变原 source attempt 的成功/失败结论；
- 把本方案表述为实时双向 FSI/co-simulation。

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

GPU functional PASS、U3 settled PASS、U4 synthetic PASS、U5 real-motion replay PASS 和
实物 fidelity PASS 是五个不同结论，禁止相互代替。

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
  phase-5 = NOT_STARTED / REQUIRES_PHYSICAL_INPUTS_AND_SEPARATE_AUTHORIZATION
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

未知物理字段必须显式为 `UNKNOWN`，不得静默使用默认值后冒充实物参数。

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
  → U5_INPUT_PACKAGE_VERIFIED_ACCEPTED
  → U5_REAL_MOTION_REPLAY_PASS
  → S6_REPLAY_AND_COMPARISON_PASS
```

任意 `FAIL`、`NO_GO`、`TIMEOUT` 或 parent drift 都进入 `STOP_AND_PRESERVE_EVIDENCE`，
不得自动跨阶段。

当前已到达并停在 `S4_LIQUID_STANDALONE_VALIDATION_PASS` 的 development-only 实例；
这不会自动授权 `U5_INPUT_PACKAGE_VERIFIED_ACCEPTED`。

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

阶段 4 不接收小车实际运动，分为 U3 静水闭合、CPU/GPU parity、数值收敛和 U4 合成运动。

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
单独授权、实测容器/液体/安装参数以及不可变小车运动输入包。

## 6. 阶段 5：小车实际运动单向耦合

### 6.1 前置条件

必须同时满足：

- 阶段 4 精确 parent 状态为 `PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION`，且其
  QC/metrics/settled/parity receipts 的 path/hash 均未漂移；
- 一个 source Gazebo attempt 已关闭，postflight 完成且身份/hash 固定；
- `R8-LIQUID-HANDOFF-ABI-v2` 输入包完整；
- 容器安装位姿、物理参数、settled state、solver profile 和 Gauge policy 已冻结；
- source attempt 的成功、timeout 或 method-failure 标签原样保留；
- 新 `transfer_id`、`replay_id`、输入验收 receipt 和 output root 均 fresh。

不得只挑选“曲线好看”或 method success 的 attempt。若研究目标包含 failure-inclusive 分析，失败和
timeout attempt 必须按预注册规则保留。

当前状态为 `NOT_STARTED / NOT_ADMITTED`：尚未冻结实测参数、source attempt、transfer package
和 exact Stage-5 profile，也未取得执行授权。

### 6.2 输入包

输入目录固定为：

```text
incoming/<transfer_id>/
├── transfer_manifest.json
├── motion.csv
├── motion_manifest.json
├── container_spec.json
├── fluid_spec.json
├── run_spec.json
└── checksums.sha256
```

验收必须逐项检查普通文件类型、size、SHA-256、closed file set 和 closed schema，拒绝 extra、
symlink、hardlink、FIFO、socket、device、可执行文件、路径穿越和压缩炸弹。只有
`VERIFIED_ACCEPTED` 才允许 replay；输入完整但运行未授权时返回 `RUN_NOT_AUTHORIZED`，不能把它
解释为 source attempt 失败。

`motion.csv` 的首版合同为：

```text
t_s,x_m,y_m,z_m,qx,qy,qz,qw
```

其中 `t_s` 从 0 开始严格递增，平移使用 m，四元数为 `x,y,z,w` 且归一化/符号连续，表达
`T_odom_container(t)`：

```text
T_odom_container = T_odom_base × T_base_container
```

禁止用 `/cmd_vel`、计划路径、`H_proxy` 或 `H_modal` 作为运动输入。

### 6.3 唯一 replay 生命周期

```text
只读验证 transfer package
  → create-new safety preflight receipt
  → 从冻结 settled state 创建只读 fresh clone
  → create-new <replay_id>.partial
  → 加载 exact runtime profile
  → 运行完整实际运动窗口 + 冻结 tail
  → finally 卸载 profile并核验零残留
  → 输出 inventory/QC/hash
  → 原子发布 final result
  → append-only secondary ledger
```

每个 source attempt 必须单独 replay。可以复用相同配置的 settled-state 内容，但必须为每个 replay
创建 fresh clone；不得复用某次动态液体结果。

### 6.4 输出和验收

最小结果包：

```text
outgoing/<replay_id>/
├── result_manifest.json
├── gauge_zsurf.csv
├── slosh_height.csv
├── summary.json
├── qc_report.json
├── build_and_runtime.txt
├── solver.log
└── checksums.sha256
```

必须证明：

- 完整 motion 和 tail 已消费，无时间倒退、重复、断点、非法四元数或未登记插值；
- boundary 实际运动与 motion manifest 在冻结容差内；
- 所有 Gauge 时间槽完整，invalid 比例低于预冻结上限；
- 粒子数/质量/ID 完整，`Nout=0`，无 NaN/Inf、泄漏和越界；
- GPU、CUDA、Xid、温度、显存、资源和 timeout 门禁通过；
- result manifest 绑定 transfer、settled state、source/binary/profile/config 和全部输出 hash；
- 原 source bag/attempt、输入包和 settled state 在前后未改变；
- 结果只作 post-hoc，不改变 source attempt outcome。

成功状态：

```text
U5_REAL_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY
```

该状态只证明一个不可变实际运动已被液体 solver 完整离线回放，不证明实物液面准确，也不证明
双向车液动力学。

## 7. 阶段 6：动态回放与 CPU/实物对比

### 7.1 输入冻结

分析和可视化只能消费 finalized manifest，不扫描“最新目录”。开始前冻结：

- source attempt、transfer、replay、CPU/GPU run 和实物 reference IDs/hashes；
- 时间零点、first-effective-motion、运动窗口、tail 和重采样网格；
- 坐标变换、probe 集、无效值处理、滤波和缺失数据规则；
- primary metric、辅助 metric、比较容差和排序规则；
- RGB ROI、像素到长度标定、时间同步/延迟修正及其不确定度；
- 重复次数、异常 run 排除条件和统计汇总方法。

不得在看到算法标签或结果后修改 probe、滤波、窗口、ROI 或 primary metric。

### 7.2 统一液面指标

圆周 probe 的界面高为 `h_i(t)`，冻结静态液深为 `h0`：

```text
eta_i(t)            = h_i(t) - h0
H_crest(t)          = max_i eta_i(t)
H_abs(t)            = max_i |eta_i(t)|
H_peak_to_peak(t)   = max_i h_i(t) - min_i h_i(t)
```

primary candidate 为 `H_crest(t)`；单个最高粒子 `maxz` 只能做 splash/QC。最终 primary 必须与
实物 RGB/液位传感器的测量语义一致后再冻结。

每个 run 至少报告：

- first 15 s、完整运动窗口和 tail 的 peak/p95/RMS；
- 主频、峰值时刻、相位和衰减率/阻尼；
- 每个 probe 的原始 `zsurf` 和 missing/invalid 比例；
- 粒子质量守恒、泄漏、NaN、越界和 splash QC；
- backend/device/seed/resolution/time-step/配置和资源身份。

### 7.3 三类比较

#### A. GPU 与 CPU

同一 case、输入、数值参数和输出时间槽下，比较粒子状态、Gauge、汇总指标、重复性和资源消耗。
按冻结容差给出 `PASS_CPU_GPU_COMPARISON` 或精确差异，不以“曲线看起来接近”代替数值门禁。

#### B. DualSPHysics 与现有仿真信号

`H_sph` 与 `H_proxy/H_modal` 只作事后相关性、误差和 cross-method ranking；后两者不能作为
DualSPHysics 的实物 reference，也不能反向调参使排名吻合。

#### C. 仿真与实物实验

实物 reference 只能来自冻结的 RGB liquid-height 或独立液位传感器 pipeline。必须按五维报告：

```text
amplitude
frequency
damping
phase
cross-case / cross-method ranking
```

每一维分别报告偏差、测量/重复不确定度、接受阈值和 PASS/FAIL。DualSPHysics 与另一 CFD/SPH
后端相互接近不能替代实物证据。

### 7.4 动态回放交付

每个选定 replay 生成：

- 基于真实 solver 帧的 MP4；
- 用于快速审阅的 GIF 或等价低带宽预览；
- 带容器坐标、时间、粒子类别和液面 probe 的静态关键帧；
- `H_crest/H_abs/peak-to-peak` 时间序列图；
- CPU/GPU 和 SIM/实物对比图；
- 可机读 CSV/JSON、绘图数据、工具/参数 hash 和 checksums；
- append-only comparison manifest、evidence index 和 secondary-ledger entry。

可视化不得平滑、裁剪或重采样原始数据而不在 manifest 中记录；GIF/MP4 不是数值证据事实源，
事实源仍是 finalized BI4/Gauge/CSV/JSON 及其 hash。

### 7.5 阶段 6 结论

阶段 6 只有在预注册比较矩阵和所有必需 reference 完整后才能得到：

```text
S6_REPLAY_AND_COMPARISON_PASS_DEVELOPMENT_ONLY
```

若缺少实物输入，可以完成动态回放和 CPU/GPU 对比，但总体只能是：

```text
S6_PARTIAL_PASS_PHYSICAL_REFERENCE_PENDING
```

这不是 blocker 的掩饰，也不能写成 physical fidelity PASS。只有 P0–P3 全部通过并完成 P4
formal 设计复审后，才可决定是否申请新的 formal release；本文不授权该步骤。

## 8. 分阶段实现工作包

后续实现必须使用 versioned create-new 文件；不得修改冻结 GPU G1/A 文件、既有 CPU receipts、
历史 raw output 或本文父方案。

| 工作包 | 需实现的最小组件 | 静态门禁 | 系统/运行授权 |
| --- | --- | --- | --- |
| S3-A runtime contract | policy、closed schema、gate、tests、bootstrap/supervisor、AppArmor template/exact instance | AST、schema、mock/负测、self-check、non-loading AppArmor query | 不授权执行 |
| S3-B one-shot smoke | start/final/failure receipts、resource/Xid monitor、output QC | parent/hash 与 fresh-root preflight | exact profile/device/candidate 单独授权 |
| S4-A DDT QC | read-only parser、closed result schema、baseline comparator、tests | fixture/negative tests、hash/inventory | 只读，不运行 solver |
| S4-B/C settle | numerical manifest/schema、fresh-run gate、settle/restart QC | 参数/阈值/容差冻结 | 每个 run 单独授权 |
| S4-D/E/F parity/synthetic | Idp comparator、motion builder、Gauge/height extractor、matrix manifest | synthetic tests、no-cherry-pick negative tests | 每个矩阵/资源预算单独授权 |
| S5 transfer/replay | ABI schema、package validator、motion QC、replay gate、secondary ledger | malicious-file/path/time-series 负测 | exact transfer/replay/profile 单独授权 |
| S6 visualization/comparison | finalized-result reader、plot/animation、CPU/real comparator、report schema | golden fixture、axis/unit/time alignment、hash tests | 只读分析；实物包另验收 |

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
7. `S5A_VALIDATE_REAL_MOTION_PACKAGE`：**下一候选 goal，尚未授权**；只验收输入，不运行 solver；
8. `S5B_REAL_MOTION_REPLAY_ONE_SHOT`：针对 exact replay/profile 单独授权；
9. `S6_REPLAY_CPU_PHYSICAL_COMPARISON`：只读生成回放、统计和证据包；
10. `P4_FORMAL_INTEGRATION_REVIEW`：仅在 P0–P3 全通过后由用户另行决定是否创建。

阶段成功不能自动授权下一 goal。涉及 profile lifecycle、sudo、GPU device、candidate/solver 执行、
跨机输入或大规模矩阵时，必须列出 exact path/name/hash/argv/资源上限并取得新的明确授权。

## 10. 时间和资源规划原则

阶段 3–6 不沿用旧 GPU 构建 campaign 的 T0，也不共享一个“8 小时”时间盒。每个实际 goal 创建时
根据当时资源和前一阶段 benchmark 冻结独立 T0、candidate deadline 和 final deadline。

- 阶段 3 只含 1 s C1M smoke，以固定 wall timeout 保护；
- 阶段 4 的 settle、收敛和参数矩阵可能远超 8 小时，必须先估算每点粒子数、显存、磁盘和 wall time；
- 阶段 5 每个 actual-motion replay 独立预算，不静默缩短 motion 或 tail；
- 阶段 6 只读分析与可视化单独预算，不能为节省空间自动删除 raw BI4/reference。

阶段 3/4 已有 C1M 的真实 GPU benchmark，但在冻结 C2 geometry、0.5 mm 粒子规模和 actual-motion
窗口前，仍不承诺 C2/0.5 mm 或完整 60 s replay 的完成时间。

## 11. 最终验收检查表

### 阶段 3

- [x] fresh GPU candidate 和阶段 2 static-audit receipt 匹配；
- [x] exact runtime contract/profile 已冻结并单独授权；
- [x] C1M 1 s、21 Part BI4/30 文件、9,078 粒子、`Nout=0` 全部通过；
- [x] CUDA/Xid/资源/profile/process/mount 零异常、零残留。

### 阶段 4

- [x] 既有 DDT-ramp QC 完成，并按预注册门槛拒绝候选；
- [x] CFL=0.2/0.1 唯一 delta、同初态、同 GPU 的配对运行和逐 Id QC 完成；
- [x] 早期 CFL 分支 FAIL 回执保留，未被后续结果覆盖或冒充 PASS；
- [x] `ViscoArtificial=0.3` cold-A/B、repeatability、restart 和 settled-state 全部通过；
- [x] speed/KE/位置/coverage/粒子/密度达到最终冻结的 development-only 阈值；
- [x] CPU/GPU parity 容差在看结果前冻结并通过；
- [x] 零回放及非零 synthetic translation/yaw/decay 与 16-sector height extractor 通过；
- [x] 最终 closed-schema QC 与彩色/灰度诊断图发布；
- [ ] 分辨率、完整时间步/物性矩阵和实物参数验证完成（不属于当前 development-only PASS）。

### 阶段 5

- [ ] `R8-LIQUID-HANDOFF-ABI-v2` 包逐文件验收；
- [ ] motion 来自已执行容器位姿，不是 `/cmd_vel` 或计划轨迹；
- [ ] fresh settled-state clone、完整 motion+tail 和 result package 通过；
- [ ] liquid 结果未反馈控制器、未改变 source attempt outcome。

### 阶段 6

- [ ] 动态回放由 finalized solver 帧生成并绑定 hash；
- [ ] CPU/GPU 对比、重复性和资源报告完成；
- [ ] probe/filter/window/ROI/时间同步在看结果前冻结；
- [ ] amplitude/frequency/damping/phase/ranking 五维实物比较完成；
- [ ] comparison manifest、evidence index、secondary ledger 和 checksums 完整；
- [ ] 未把 development 结果表述为 formal、production 或 physical-primary。

## 12. 当前唯一允许的下一步

本文完成了阶段 3–6 的离线规划，并同步了阶段 1–4 的最新不可变证据。阶段 4 已取得
development-only 液体单独验证 PASS；repeatability、parity 和本轮 U4 synthetic 均已完成，
无需重跑。阶段 5/6 尚未开始，也不会因阶段 4 PASS 自动授权。

若用户决定进入阶段 5，下一 goal 只能先验收实测容器/液体/安装参数以及一个已关闭、hash 固定的
source Gazebo attempt，并冻结 `R8-LIQUID-HANDOFF-ABI-v2` transfer package；在该只读门禁通过前
不得运行新的 DualSPHysics replay。

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
U5_STARTED=false
PHASE5_ADMITTED=false
CURRENT=STOP_AFTER_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION_PASS
NEXT=STAGE5_REQUIRES_SEPARATE_USER_AUTHORIZATION_AND_PHYSICAL_INPUTS
```

## 13. 权威参考

1. [RTX 5080 GPU 8 小时快速构建方案](./20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md)
2. [DualSPHysics 物理液体接入 SIM-R8 方案](./20260805_DualSPHysics物理液体接入SIM-R8方案.md)
3. [Ubuntu 24.04 液体仿真电脑任务与数据交接说明](./20260806_Ubuntu24.04液体仿真电脑任务与数据交接说明.md)
4. [RTX 5080 GPU 构建阶段进度与下一步（当前状态页）](./20260810_RTX5080_DualSPHysics_GPU构建阶段进度与下一步.md)
