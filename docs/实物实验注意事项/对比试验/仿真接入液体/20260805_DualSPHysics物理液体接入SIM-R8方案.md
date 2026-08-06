# 20260805 DualSPHysics 物理液体接入 SIM-R8 方案

> 状态：`P0-A_SOURCE_ACQUIRED_STATIC_VERIFIED / DEVELOPMENT_NO-GO / NOT_BUILT / NOT_RUN`
>
> 适用源码域：`src/scout_apps/simulation/`（SIM-R8）
>
> 主液体后端候选：[DualSPHysics](https://github.com/DualSPHysics/DualSPHysics)
>
> 高保真复核候选：[OpenFOAM-dev](https://github.com/OpenFOAM/OpenFOAM-dev)
>
> 2026-08-05 已按用户授权完成 P0-A：安全根、固定源码获取和静态复验。该授权不扩展到
> 构建、执行上游二进制、修改 runner 或启动 ROS/Gazebo/液体仿真。

## 0. 结论先行

首版采用 **two-pass、离线、只读液体评价**，不把 DualSPHysics 直接塞入当前
Gazebo/ROS 生命周期：

1. 第一遍仍由 SIM-R8 唯一 runner 完成 fresh ROS/Gazebo、30 s settle、recorder
   before motion、60 s trajectory hard cap、frozen tail 和 postflight。
2. 只有 case 的 closed bag、manifest 和 postflight 已生成且 hash 固定后，第二遍才从
   closed bag 提取 executed `/odom`，生成 DualSPHysics 容器运动输入。
3. DualSPHysics 独立回放相同的容器运动，用内置 `swl`/`zsurf` Gauge 计算液面。
4. 液体结果写入新的 append-only secondary ledger；不得回写原 attempt，不改变
   `GOAL_REACHED`、method success/failure、planned row 或 40/64/88 固定分母。
5. 在通过数值收敛和实物 RGB 保真验证以前，全部结果固定为：

```text
development_only = true
formal = false
fidelity_validation_status = UNVALIDATED
physical_primary_eligible = false
```

该路径的直接目标是获得比 `H_proxy`/`H_modal` 更独立的粒子液体高度评价量；它仍然是
数值仿真，不等于真实水，也不自动具有 physical-primary 资格。

安全、依赖来源、存储根、空间余量、失败恢复和数据保留必须在安装前规划并通过
preflight；不能先运行大粒子 case，再根据现场情况补安全规则。

### 0.1 2026-08-05 P0-A 实施结果

已创建的仓库内实现仅位于：

```text
src/scout_apps/simulation/scout_dualsphysics_liquid/
```

外部窄根和证据：

```text
root:
  /data/a/scout_sim_replacement/r8_liquid
prepare receipt:
  /data/a/scout_sim_replacement/r8_liquid/dependency/manifests/safety_preflight_prepare_root_20260804T174945Z.json
acquire preflight receipt:
  /data/a/scout_sim_replacement/r8_liquid/dependency/manifests/dependency_acquire_preflight_20260804T175100Z.json
dependency manifest:
  /data/a/scout_sim_replacement/r8_liquid/dependency/manifests/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.json
source:
  /data/a/scout_sim_replacement/r8_liquid/dependency/source/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086
```

静态复验结果：

| 证据                            | 实际值                                                               |
| ------------------------------- | -------------------------------------------------------------------- |
| exact commit                    | `ef3721a861fda961f0e2f9ec4cd317b19de99086`                         |
| Git tree                        | `cef458cb358712f4694b9d2148f638440418e9dc`                         |
| commit signature                | `N`；未做密码学作者认证                                            |
| source inventory                | `3a59e83b1a93e3a7302ba37ab6a1b29a5335e8d468ad098dbb00f842848bbab5` |
| source entries / bytes          | `768` / `293416502`                                              |
| manifest hash（内部 canonical） | `7330a7c23e9a00a124f10718739d63e58c5138ed6f9dc1da208c1028c64ec4f2` |
| manifest file SHA-256           | `5ea1a431cfe992061c90617e1432017736a3c1be342b13982bf9cf10cd0ccad3` |
| license artifacts               | 3：主`LICENSE`、MoorDynPlus、KISS FFT                              |
| precompiled ELF                 | 11；全部`NOT_EXECUTED_NOT_ADMITTED`                                |
| working tree / submodule        | clean / 未初始化且清单为空                                           |
| `.partial` / symlink residue  | 无                                                                   |
| unit tests                      | 17 项无 ROS 测试 PASS                                                |
| read-only dependency verify     | `PASS`                                                             |

P0-A 全程未启动 ROS/Gazebo、未修改原 attempt/runner、未构建 solver，也未执行
GenCase、MeasureTool、预编译 `.so` 或任何 checkout 脚本。当前外部根占用约 `341 MiB`。
预编译 ELF 的存在是下一阶段的独立 NO-GO 风险，不是“已经安装可用”的证据。

### 0.2 2026-08-05 P0-B 沙箱准备状态

已新增 VM、最小 CPU build 和 GenCase 首次执行的冻结策略、schema、fail-closed gate 与
测试，并在批准根中创建专用 `vm_images/toolchains/audits/scratch/quarantine` 空目录。

当前实际门禁仍为：

```text
VM host:  NO_GO_QEMU_AND_QEMU_IMG_MISSING_NOT_ADMITTED
CPU build: NO_GO_VM_RUNTIME_MISSING_AND_BUILD_RECIPE_NOT_FROZEN
VM started: false
build started: false
GenCase started: false
upstream code executed: false
```

完整时间线、回执哈希和回退清单见
[`20260805_P0B独立构建沙箱与GenCase审计实施日志.md`](20260805_P0B独立构建沙箱与GenCase审计实施日志.md)。

## 1. 与现有 SIM-R8 流程的关系

现有流程以 [`src/scout_apps/simulation/README.md`](../../../../src/scout_apps/simulation/README.md)
为唯一上位入口。以下边界保持不变：

| 项目       | 继续使用的 R8 入口                                                   | 本方案禁止事项                                                   |
| ---------- | -------------------------------------------------------------------- | ---------------------------------------------------------------- |
| 控制器     | `spmpc_sim_local_planner`、节点 `/sim_spmpc_local_planner`       | 不链接或调用`src/scout_apps/control/`                          |
| 环境       | R8 自有 environment adapter，fresh loopback ROS/Gazebo               | 不调用历史 shared-source strict wrapper                          |
| 生命周期   | 30 s settle、recorder before motion、60 s hard cap、tail、postflight | 不让 CFD/SPH 卡住或改变第一遍 case 判定                          |
| 数据       | case-local closed bag、manifest、postflight、append-only ledger      | 不覆盖 bag，不重写原账本                                         |
| 液体信息   | 新 plant 只消费 executed motion                                      | 不消费`/cmd_vel`、controller state、`H_proxy` 或 `H_modal` |
| 控制防火墙 | `/sim_truth/*` 对 controller/planner/tracker/cmd-gate 禁止         | 不让物理液体结果进入控制器反馈                                   |

R7 结果保持历史不可变；本方案只面向新的 SIM-R8 development/release，不重标 R7。

### 1.1 现有三个高度信号的身份

| 名称        | 当前/计划来源                        | 允许用途                                   |
| ----------- | ------------------------------------ | ------------------------------------------ |
| `H_proxy` | `/slosh/height`                    | 机制 proxy，只记录                         |
| `H_modal` | `/sim_spmpc/slosh_height`          | 控制器内部 modal，只记录                   |
| `H_sph`   | DualSPHysics`zsurf`/`swl` 后处理 | 独立 development liquid plant 候选，只记录 |

禁止把 `H_sph` 偷换命名为 `H_truth`、`H_physical` 或 physical primary。推荐 ROS/报告语义
使用 `H_sph`；若未来 live bridge 成立，话题建议为
`/sim_truth/dualsphysics/liquid_height`，不复用现有 surrogate 的
`/sim_truth/liquid_height`。

## 2. 后端选择和版本冻结

### 2.1 主后端：DualSPHysics

选择原因：

- 工程型自由液面 SPH，提供 CUDA/CPU 实现；
- 支持固定、运动和浮体边界；
- `JMotion` 支持由文件给出平移、旋转和组合运动路径；
- 内置 Gauge 支持 `swl`、`maxz` 和网格 `zsurf`，能直接输出 CSV；
- 与“回放已执行容器运动并计算晃动高度”的目标直接匹配；
- 核心代码为 LGPL-2.1。

P0-A dependency gate 已冻结的 pin：

```text
repository: https://github.com/DualSPHysics/DualSPHysics
candidate_commit: ef3721a861fda961f0e2f9ec4cd317b19de99086
observed_branch: master
observed_gauge_format: DualSPHysics v5.4.350
```

该 pin 已完成 exact commit、Git tree、source/license/ELF hash 和 dependency manifest
复验，但提交本身无签名。正式构建前仍必须单独审查工具来源、编译器、CUDA、构建选项
和二进制 hash；不得使用浮动 `master`，也不得把源码获取 PASS 当作构建准入。

`MeasureTool` 的帮助文件声明了独立的二进制再分发/修改限制。首版主要依赖 core 内置
Gauge CSV 和自有只读解析器，不修改或再分发 `MeasureTool`；如以后需要发布工具链，
必须单独完成许可证审查。

### 2.2 复核后端：OpenFOAM

OpenFOAM 官方自带：

```text
tutorials/incompressibleVoF/sloshingCylinder
src/functionObjects/field/interfaceHeight
```

它使用水/空气 VOF 界面，适合抽取少量 case 做高分辨率 reference，不作为首版 R8
批量运行后端。DualSPHysics 与 OpenFOAM 都必须独立接受实物数据验证，二者相互接近
也不能替代实物 RGB 证据。

## 3. 总体架构

```text
第一遍：现有 SIM-R8 fresh case（唯一控制/运动事实源）

R8 environment + controller
        │
        ├── executed /odom, /tf, /tf_static
        ├── H_proxy / H_modal
        └── closed bag + manifest + postflight
                         │
                         │ 只读、SHA-256 绑定
                         ▼
第二遍：DualSPHysics liquid replay（不接 ROS 控制图）

motion exporter
        ├── time/pose continuity QC
        ├── T_odom_base × T_base_container
        └── motion.csv + motion_manifest.json
                         │
                         ▼
case builder ── container geometry + water params + Gauge freeze
                         │
                         ▼
DualSPHysics CUDA/CPU isolated process
                         │
                         ├── Gauge zsurf/swl CSV
                         ├── particle/mass/leakage QC
                         └── solver log + binary/config hashes
                         │
                         ▼
height extractor + fidelity/QC
                         │
                         ├── H_sph(t), peak, p95, frequency, damping
                         └── append-only secondary ledger
```

### 3.1 为什么首版不做 live bridge

DualSPHysics 的成熟入口是预先给定运动文件后批处理运行；当前 R8 development liquid
plant 则是 adapter-owned live Python child 和 ROS topic。首版不能把离线结果伪装成
live topic，也不能宣称满足现有 formal runtime plant ABI。

若以后必须 live，有两个独立选项：

1. 开发受控 DualSPHysics library/IPC bridge，按 `/clock` 步进并发布 record-only topic；
2. 正式扩展 R8 为 hash-bound post-run liquid plant ABI，允许 closed-bag deterministic
   replay 成为独立结果层。

两者都会改变 release/ABI，必须新增 source registry、tests、freeze、GO receipt 和 timing
admission，不能倒灌现有数据。

### 3.2 隔离生成和“合并”的准确语义

为保护原 Gazebo 环境，首版采用三级隔离：

```text
A. 独立 plant bootstrap（不启动 Gazebo）
   container geometry + water particles + hydrostatic settle
                              │
                              └── immutable settled_state + hash

B. 每个小车 case 的独立动态回放（Gazebo 已结束）
   fresh clone settled_state + closed-bag executed motion
                              │
                              └── H_sph time series + QC

C. 数据层合并
   source attempt hash + time alignment + liquid report hash
                              │
                              └── joined analysis/report
```

允许“一次生成”的只有完全相同配置下的容器几何、粒子排布和静水 settle state。动态
晃动不能只生成一次后复用，因为每个算法/run 的 executed motion 不同；每个 source
attempt 都必须从同一只读初态 fresh clone 后重新计算。

settled state 的身份必须绑定完整元组：

```text
dependency/binary hash
container geometry hash
mount-independent water volume/depth hash
fluid parameter hash
dp/kernel/boundary/dt hash
initial particle generator/seed hash
settle policy and completion report hash
```

元组任一项改变都生成新的 state ID。每个 replay 只能读 settled state，不能原地继续
写；case-local mutable state 必须位于自己的 `.partial` 目录。

“合并”仅指：

- 用 source attempt ID/hash 关联 Gazebo bag 与液体结果；
- 把 `H_sph(t)` 对齐到同一 first-effective-motion/terminal/tail 时间轴；
- 在报告中并排分析 tracking、command、`H_proxy`、`H_modal` 和 `H_sph`；
- 如需可视化，只做事后 RViz/ParaView/视频 overlay。

首版明确禁止：

- 把 DualSPHysics 粒子、world/plugin 或 shared library 加载到原 Gazebo world；
- 在 Gazebo case 运行时启动 DualSPHysics 争用 GPU/CPU；
- 将液体力、液面或 Gauge 输出反馈给 `/cmd_vel`、底盘或 controller；
- 用液体 replay 的成功/失败修改原 Gazebo method outcome。

若未来必须研究液体反作用力对小车动力学的影响，应新建独立 two-way co-simulation
environment/release，并验证时间同步、能量守恒、力/矩方向、失联 fail-safe 和实时性；
不能在当前 R8 runner 上悄悄打开反馈。

## 4. 建议的源码与数据布局

### 4.1 仓库内只存 wrapper、schema 和小配置

完整规划如下；P0-A 当前只创建 README、安全/依赖 schema、两个 gate 脚本和无 ROS 测试：

```text
src/scout_apps/simulation/scout_dualsphysics_liquid/
├── README.md
├── config/
│   ├── development/C1_D37_H58_unvalidated.yaml
│   ├── development/C2_D95_H58_unvalidated.yaml
│   └── gauges/crest_height_v1.yaml
├── schema/
│   ├── dependency_manifest_v1.json
│   ├── motion_manifest_v1.json
│   ├── replay_manifest_v1.json
│   └── height_report_v1.json
├── scripts/
│   ├── r8_liquid_dependency_gate.py
│   ├── r8_liquid_motion_export.py
│   ├── r8_liquid_case_builder.py
│   ├── r8_liquid_runner.py
│   ├── r8_liquid_height_extract.py
│   └── r8_liquid_fidelity_verify.py
└── tests/
```

不把 DualSPHysics 全仓库、编译产物、粒子输出或大 CSV/VTK 提交到 `scout_ws`。

### 4.2 外部构建和数据根

可以放在 `/data/a`，但只能使用一个预先解析并冻结的窄子根，不能把 `/data/a` 本身
交给脚本作为可写根。统一建议：

```text
APPROVED_ROOT=/data/a/scout_sim_replacement/r8_liquid

$APPROVED_ROOT/
├── dependency/
│   ├── source/DualSPHysics_<commit>/
│   ├── build/<dependency_hash>/
│   ├── install/<dependency_hash>/
│   ├── vm_images/
│   ├── toolchains/
│   └── manifests/<dependency_hash>.json
├── audits/
│   ├── sandbox/
│   └── tools/gencase/
├── releases/
│   └── SMPCC-SIM-R8-LIQUID-DEV-R1/
│       ├── geometry/
│       ├── initial_states/<settled_state_hash>/
│       ├── policies/
│       ├── reports/
│       └── secondary_ledger.jsonl
├── runs/
│   └── development/<release_id>/<source_attempt_id>/<replay_id>/
│       ├── input/
│       ├── raw/
│       ├── derived/
│       ├── logs/
│       └── manifests/
├── scratch/
│   ├── vm/<audit_id>.partial/
│   ├── build/<build_id>.partial/
│   └── <replay_id>.partial/
├── locks/
└── quarantine/
```

构建不得写 `/home/a/scout_ws/build`、`devel`，也不得进入 R8 controller workspace。
repo 内只保存小型源码、schema、development 配置和报告；大粒子数据只留在上述数据根。

2026-08-05 的只读检查结果：

```text
mount:       /data (/dev/nvme1n1, XFS, rw)
capacity:    932 GB
used/free:   437 GB / 495 GB
inode used:  1%
quota:       noquota
owner:       /data/a = a:a
existing /data/a/scout_sim_replacement: about 9.3 GB
```

因此当前空间足以做 P0/P1，但由于挂载没有 quota，application-level 空间门禁是必需项，
不是优化项。

### 4.3 路径和权限安全

所有工具启动时必须：

1. 将 `APPROVED_ROOT` 解析为 canonical absolute path；
2. 拒绝空路径、相对路径、`/data/a`、`/data`、`/`、`~` 和环境变量未展开结果；
3. 对每一级路径做 `realpath`/父目录检查，拒绝任意 symlink 逃逸；
4. 输出路径必须是 `APPROVED_ROOT` 的真子路径，且包含合法 `release_id/replay_id`；
5. 以 create-new/O_EXCL 语义创建结果，不覆盖同名文件或目录；
6. 建议 `umask 0027`、目录 `0750`、普通文件 `0640`，不修改 `/data/a` 的全局权限；
7. 输入 bag、manifest 和 freeze 以只读方式打开，禁止就地修改；
8. 记录 resolved path、device ID、owner、permission 和 SHA-256 到 preflight receipt。

不要依赖 shell glob、未校验环境变量或宽目录递归操作来选择清理/移动目标。

### 4.4 原子写入、崩溃恢复和账本

- replay 先写 `scratch/<replay_id>.partial/`，所有子进程成功、文件关闭并完成 hash 后，
  才在同一 XFS 文件系统内原子 rename 到最终 `runs/.../<replay_id>/`；
- 中途崩溃只留下 `.partial`，不得被分析器或 secondary ledger 接受；
- `.partial` 不自动删除，先生成 recovery/quarantine report，再由用户明确批准清理；
- secondary ledger 只在最终目录和 height report 全部复验后 append；
- ledger 每行带 `previous_entry_hash/entry_hash`，append 时持有独占 lock；
- lock 记录 PID、进程 start time、boot ID、用户和 replay ID；PID 不存在也不能静默删锁，
  必须先形成 stale-lock audit；
- append 或原子 rename 失败时不得产生“半成功”状态。

### 4.5 容量预留和保留策略

P0 尚未给出实际单 case 大小，所以以下是 **候选安全门槛**，必须在 benchmark 后冻结：

```text
reservation_bytes = max(estimated_case_bytes, previous_same_profile_peak_bytes) * 1.5
minimum_free_after_reservation = max(100 GiB, filesystem_capacity * 15%)
emergency_free_watermark = max(50 GiB, filesystem_capacity * 5%)
```

- 预检查若 `free - reservation` 低于 minimum，拒绝启动；
- 运行中触及 emergency 水位时，停止产生新 snapshot，受控结束自有 solver，并把 case
  标为 `INFRASTRUCTURE_DISK_LOW`；不得靠删除旧正式数据续跑；
- 预测空间必须包含 raw particle、Gauge、日志、临时 surface reconstruction 和 1.5 倍
  安全余量；
- 正式/候选证据不允许自动清理；development scratch 也只可由显式、目标清单化、hash
  校验后的 retention job 清理；
- 所有物理删除都必须由用户另行授权，不属于普通 runner 生命周期。

数据分层：

| 层级            | 内容                                                         | 默认保留策略                                    |
| --------------- | ------------------------------------------------------------ | ----------------------------------------------- |
| A：证据核心     | manifest、hash、配置、Gauge CSV、height report、日志、ledger | 长期保留，必要时做第二份小文件镜像              |
| B：复核原始数据 | 预冻结频率的 BI4/粒子 snapshot、surface 文件                 | 保留到数值/实物验证和报告签收                   |
| C：可再生中间量 | VTK、渲染缓存、临时重建输出                                  | 首版不自动删；以后按显式 retention receipt 处理 |

禁止先全频率保存全部粒子再临时决定删什么。P0 必须测量每秒输出量，并在 P1 前冻结
snapshot 频率；Gauge 可高频保存，raw particle snapshot 应使用满足复核需求的最低固定频率。

## 5. 输入合同：只接受 executed motion

### 5.1 允许输入

首版 exporter 只读取已关闭 R8 bag 的：

```text
/odom
/tf
/tf_static
```

主要时间基准是 `/odom.header.stamp`，并以 rosbag record timestamp 做一致性检查。未来
新 release 建议额外录 `/clock`；不得为历史 bag 伪造 `/clock`。

### 5.2 禁止输入

```text
/cmd_vel
/cmd_vel_drive
/sim_spmpc/* controller/observer/debug state
/slosh/height
/sim_spmpc/slosh_height
/sim_truth/*
```

只允许 manifest/metrics 工具读取 `H_proxy`/`H_modal` 做事后对比；运动 exporter、case
builder 和 SPH solver 不得读取它们。

### 5.3 容器位姿

冻结关系：

```text
T_odom_container(t) = T_odom_base(t) × T_base_container
```

`T_base_container` 必须来自独立 mount manifest，不能默认容器位于 `base_link` 原点。
当前 C1/C2 YAML 只有半径和液深，不能充当完整 physical geometry/mount freeze。

exporter 至少执行：

- bag closed/readable 和 source attempt hash 校验；
- 时间戳严格单调、gap、duplicate、reverse 检查；
- quaternion 归一化和 yaw unwrap；
- 全 SE(3) 位姿保留；若实际模型只采用平面运动，必须在 manifest 显式写明被丢弃的
  roll/pitch/z 及最大值；
- 统一采样仅使用预冻结插值规则：平移线性、旋转 SLERP；
- 第一有效运动、`GOAL_REACHED`/timeout、tail 的时间锚定；
- 输出 motion CSV 和 canonical JSON manifest，并记录 source bag/attempt/postflight hash。

不得直接对噪声 pose 做两次差分后未经规则地输入 SPH。优先使用 DualSPHysics 的
`mvpathfile` 驱动容器边界，避免加速度差分放大噪声。

### 5.4 移动域风险

P2 路径从 `(-4,0)` 到 `(5,0)`，绝对平移约 9 m；毫米级粒径下，直接建立覆盖整条路径
的粒子邻域可能带来大 cell-grid 内存开销。

DualSPHysics 存在 `AccInput`，但候选 commit 的实现会拒绝旧文档中的 `mkfluid`，不可
直接假设能够在固定容器坐标系给液体施加任意惯性加速度。P0 必须比较并门禁：

1. 绝对路径移动边界；
2. 平移域/局部原点策略；
3. 经源码审查的非惯性 frame 扩展。

在数值等价和能量审计通过以前，不允许用高通位置、截断路径或未经证明的伪力近似。

## 6. 容器和水参数合同

### 6.1 当前 development 条件

| 条件 |  内半径 | 静态液深 |    估算水量 | 当前身份                            |
| ---- | ------: | -------: | ----------: | ----------------------------------- |
| C1   | 18.5 mm |    58 mm |  约 62.4 mL | `SIM_ONLY_C1_D37_H58_UNVALIDATED` |
| C2   | 47.5 mm |    58 mm | 约 411.1 mL | `SIM_ONLY_C2_D95_H58_UNVALIDATED` |

正式几何仍缺：

- 容器总内高、freeboard、底部圆角；
- 壁厚和内壁 mesh；
- 开口/盖体状态；
- 容器轴相对 `base_link` 的位置与方向；
- 壁面材料、润湿/接触角；
- 液体温度、密度、动力/运动黏度、表面张力；
- 初始液深/体积测量误差和初始静置规则。

这些必须由用户/实物测量提供，不能从 modal 模型反推后冒充物理参数。

### 6.2 初始静置

先在完全隔离的 plant-bootstrap job 中生成静态水体并执行 hydrostatic settle。settle
完成判据需预冻结，例如：

- 总动能或粒子速度 p95 连续一段时间低于阈值；
- 圆周 `zsurf` 偏差稳定；
- 无粒子泄漏、质量损失或 NaN；
- 保存 settle-end state hash 作为 replay 初态。

P0 必须验证从冷启动重复 settle 的一致性，并验证“加载冻结 settled state 后第一步”与
冷启动 settle-end 在粒子数、质量、位置/速度统计和 Gauge 上等价。通过后，每个 replay
使用 settled state 的只读 fresh clone；这不等于复用同一 solver session。不能因为
某次曲线更好看而换初态、延长或缩短 settle。

## 7. 数值分辨率与本机预算

按粒子体积粗略估计：

| 条件 | `dp=2.0 mm` | `dp=1.0 mm` | `dp=0.75 mm` | `dp=0.5 mm` |
| ---- | ------------: | ------------: | -------------: | ------------: |
| C1   |      约 7,800 |     约 62,000 |     约 148,000 |    约 499,000 |
| C2   |     约 51,000 |    约 411,000 |     约 974,000 |  约 3,289,000 |

当前机器 RTX 3060 Laptop 6 GB、CUDA 12.1 可作为 P0/P1 原型机，但不能提前承诺
C2/0.5 mm 或 60 s replay 的实时率。DualSPHysics README 的当前构建示例面向 CUDA
12.3，候选 commit 在本机 CUDA 12.1 上必须单独 build gate；禁止直接复用未知二进制。

由于研究阈值是 1 mm，`dp=2 mm` 只能做 smoke，不得用于 1 mm 结论。建议：

1. P0 smoke：C1/`dp=2 mm`；
2. P1 主开发：C1 和 C2/`dp=1 mm`；
3. 收敛：至少 `1.0/0.75/0.5 mm`，若 C2/0.5 mm 超出显存，则先报告资源 NO-GO，
   不能删掉该点后宣称收敛；
4. 所有 `dt`、kernel、boundary formulation、density diffusion、viscosity 和 Gauge
   masslimit 随 release 冻结。

## 8. 晃动高度定义

### 8.1 不使用全局最高粒子作为 primary

单个飞溅/游离粒子的最大 z 会制造假峰。`maxz` 只作为 splash/QC；primary 使用
DualSPHysics `swl` 或 Gauge mesh 的 `zsurf`。

### 8.2 传感器布局

在容器坐标系冻结：

- 圆周至少 16 个等角度 vertical probes，建议再评估 32 个；
- probe 半径为 `R - k·dp`，`k` 随 boundary method 固定；
- 中心及中半径 probes 用于区分整体倾斜、局部 crest 和数值孔洞；
- 所有 probes 随容器运动，输出统一转换回 container frame；
- Gauge `masslimit`、`pointdp`、kernel correction 和无效值规则预先冻结。

### 8.3 指标

设第 `i` 个 probe 的界面高为 `h_i(t)`，静态液深为 `h0`：

```text
eta_i(t)      = h_i(t) - h0
H_crest(t)    = max_i eta_i(t)
H_abs(t)      = max_i |eta_i(t)|
H_peak        = max_t H_crest(t)
H_p95         = percentile_95(H_crest(t))
H_peak_to_peak(t) = max_i h_i(t) - min_i h_i(t)
```

建议将 `H_crest(t)` 作为“晃动高度”的 primary candidate，因为它与当前“高于静态液面
的 crest elevation”语义最接近；`H_abs` 和 peak-to-peak 作为补充。最终定义必须与实物
RGB ROI/标定输出一致后冻结。

输出还应包含：

- first-effective-motion 后完整 60 s 和 tail 时序；
- first 15 s、全窗口的 peak/p95/RMS；
- 主频、阻尼/衰减率、峰值时刻；
- 每个 probe 的原始 `zsurf`；
- missing/invalid probe 比例；
- 粒子质量守恒、泄漏、NaN、越界和 splash 数；
- 分辨率/时间步/重复 seed 标识。

不得在看到算法标签后选择 probe、滤波或时间窗口。

## 9. two-pass case 生命周期

### 9.1 第一遍：R8 case

完全使用现有入口。DualSPHysics 不启动、不占 ROS/Gazebo 端口，也不参与 method
success 判定。

源 case 至少满足：

- closed bag 存在并可读；
- attempt manifest 和 postflight hash 固定；
- pre/post ROS、Gazebo reachability 符合 R8 规则；
- motion start 和 terminal/timeout 事件可恢复；
- 原结果目录不可覆盖。

development 阶段可对 method failure 做液体回放，用于 failure-inclusive 分析；标签必须
保留，不得只选择成功 case。

### 9.2 第二遍：离线 liquid replay

计划 CLI（当前不存在，不可直接运行）：

```bash
python3 r8_liquid_motion_export.py \
  --source-attempt-manifest /abs/.../attempt_manifest.json \
  --source-bag /abs/.../h0_runtime.bag \
  --mount-manifest /abs/.../container_mount.json \
  --output /abs/new/motion_manifest.json

python3 r8_liquid_case_builder.py \
  --dependency-manifest /abs/.../dependency_manifest.json \
  --motion-manifest /abs/.../motion_manifest.json \
  --container-manifest /abs/.../C1.json \
  --gauge-policy /abs/.../crest_height_v1.json \
  --output-root /abs/new/replay_case

timeout --preserve-status --kill-after=60s 2h \
  python3 r8_liquid_runner.py --case-manifest /abs/.../replay_case_manifest.json

python3 r8_liquid_height_extract.py \
  --replay-manifest /abs/.../replay_manifest.json \
  --output /abs/new/height_report.json
```

timeout 值由 P0 benchmark 冻结；超时记录为 infrastructure/method failure，不静默加时。

### 9.3 secondary ledger

每行至少绑定：

```text
source_attempt_id
source_attempt_manifest_path/hash
source_bag_path/hash
source_postflight_path/hash
motion_manifest_path/hash
dependency_manifest_path/hash
container_manifest_path/hash
gauge_policy_path/hash
replay_manifest_path/hash
height_report_path/hash
replay_status
fidelity_validation_status
formal=false
physical_primary_eligible=false
previous_entry_hash
entry_hash
```

同一 source attempt + release + resolution 组合只允许一个 canonical entry；重跑必须有新
attempt/revision ID，不能覆盖。

## 10. 与现有 H0 liquid plant 的兼容策略

当前 `smpcc_sim_h0_runtime_adapter.py` 的
`--with-development-liquid-plant` 启动的是 live `scout_liquid_plant` Python surrogate，
并要求 `/sim_truth/liquid_*` ready 和三阶段 firewall snapshot。

首版 DualSPHysics 必须：

- 不复用该 CLI flag；
- 不冒充现有 capability ID 或三个 live topic；
- 不修改现有 H0 case manifest；
- 通过独立 postflight replay tool 和 secondary ledger 接入；
- 只有未来 live bridge 通过后，才设计新的 explicit flag，例如
  `--with-development-dualsphysics-live-plant`。

若未来扩展 formal toolchain，至少要新增/修改并重新验证：

- formal plant input/output schema；
- offline replay 或 live runtime ABI；
- recorder/plant artifact binding；
- ready/pre_motion/postflight firewall；
- source bag、motion conversion、solver 和 height extractor 的传递 hash；
- freeze/master/GO/timing/campaign tests。

## 11. 防火墙与进程安全

### 11.1 信息防火墙

DualSPHysics 工具不得成为任何控制节点的 publisher/subscriber 依赖。即使未来发布
ROS topic，也只有 recorder 和 metrics 节点可订阅：

```text
/sim_spmpc_local_planner              FORBIDDEN
/cmd_vel_guard                        FORBIDDEN
planner/tracker/reference publisher   FORBIDDEN
recorder/metrics                      ALLOWED
```

ready、pre_motion、postflight 三点继续检查 `/sim_truth/*` subscriber graph。

### 11.2 第三方依赖和构建安全

- 只获取 manifest 中的 HTTPS repository 和 exact commit，不执行浮动 branch；
- 下载完成先计算 source tree/archive hash，再进入构建；若上游提供签名或 release
  checksum，一并保存和验证；
- 不直接信任或执行来源不明的预编译 binary、安装脚本或 shell profile；
- 不使用 `sudo`，不写 `/usr`、`/usr/local`、系统 CUDA、ROS Noetic 或用户 Python
  site-packages；
- 以干净 `env -i` 和显式 compiler/CUDA 路径构建，记录完整 build command、compiler、
  CUDA driver/toolkit、flags、binary dependency 和 binary SHA-256；
- dependency build 与 R8 controller build prefix 完全分开；
- 网络只允许 dependency acquisition 阶段使用；case build、solver 和后处理默认断网；
- 构建后先运行上游最小测试和本项目 dependency gate，再允许生成 development case。

### 11.3 进程、GPU 和资源边界

- 不使用 `killall`/`pkill`；runner 只结束自己创建的 PID/进程组；
- DualSPHysics 不与 Gazebo case 并发是首版默认，避免 GPU/CPU 抢占改变 Gazebo 执行
  运动，也避免 solver 影响第一遍的 timing；
- 通过 `locks/gpu0.lock` 做单 GPU 独占；启动前记录 GPU 型号、driver、空闲显存和当前
  compute process inventory；
- 每个 replay 有独立 `.partial` 目录、外层 wall-time timeout、最大 snapshot 数和最大
  预估 bytes；
- 监测 wall time、RSS、GPU memory、温度和磁盘低水位；超限时只受控结束自有进程组；
- GPU OOM、driver reset、NaN/Inf、粒子泄漏、质量失守、Gauge 缺失、磁盘低水位均
  fail-closed，不自动降低分辨率、缩短窗口或换 CPU 后继续冒充同一 case；
- CPU/GPU fallback 是不同 execution profile，必须产生不同 manifest/hash。

### 11.4 文件系统和数据完整性边界

- 所有写入限于第 4 节 `APPROVED_ROOT`，禁止直接以 `/data/a` 为 output root；
- 启动前完成 path containment、权限、空间 reservation 和 source artifact hash 检查；
- 运行完成前不写最终目录、不 append ledger；
- analysis 只接受 finalized replay manifest，不扫描“最新目录”猜测输入；
- 原始 bag 和原 attempt 均只读，禁止通过 hardlink/symlink 让输出覆盖输入；
- 小型 A 类证据建议在签收后复制到第二存储位置并复验 hash；该镜像动作需要独立工具和
  目标授权，不在 solver 内隐式完成；
- 任何 cleanup 都要先列出 exact targets、类型、大小、hash/保留级别和恢复性，再由用户
  明确授权。

### 11.5 安全 preflight 收据

每次 replay 启动前生成 immutable `safety_preflight.json`，至少记录：

```text
approved_root/resolved_output_root
path_containment_status
source_attempt/bag hashes
dependency/source/binary hashes
user/uid/gid/umask
filesystem/device/fstype/mount_options
free_bytes/reservation_bytes/minimum_free_after_reservation
GPU/driver/toolkit/free_memory/compute_processes
CPU/RAM limits
timeout/snapshot/output limits
network_policy
owned_process_group_policy
status = PASS | NO-GO
```

只有 `PASS` 才创建 `.partial` 并启动 solver；收据缺失、字段漂移或检测异常均 NO-GO。

## 12. 验证与门禁

### P0：依赖和最小 smoke

目标：证明 exact commit 能在本机隔离构建并产生稳定 Gauge 输出。

检查：

- dependency manifest、license、compiler/CUDA/binary hash；
- C1 静水、`dp=2 mm`、短时运行；
- `swl`/`zsurf` CSV schema；
- 无 NaN、无泄漏、质量守恒、确定性重复；
- CPU/GPU 小 case 数值差异报告；
- 绝对路径运动的 domain/memory benchmark。

P0 分为两个独立门禁：

- `P0-A`：固定源码获取、provenance、license/ELF 静态清单；已完成并只获得
  `PINNED_SOURCE_VERIFIED_DEVELOPMENT_ONLY`。
- `P0-B`：隔离构建、静水和最小 Gauge smoke；当前仍 `NO-GO / NOT_BUILT / NOT_RUN`，
  必须先新增构建沙箱与预编译工具准入复审。

P0 即使全部通过也只允许 `DEVELOPMENT_SMOKE_NOT_FORMAL`。

### P1：数值物理验证

至少包含：

1. 静水：高度偏置、漂移、压力/密度稳定；
2. 小幅正弦平移：主频与线性理论/解析模态一致性；
3. 正弦 yaw/组合运动：相位和圆周 probe 方向正确；
4. 停止激励：自由衰减频率和阻尼；
5. `dp=1.0/0.75/0.5 mm` 空间收敛；
6. 时间步/CFL 收敛；
7. C1/C2 参数转移；
8. wall/contact/boundary method 敏感性；
9. OpenFOAM `sloshingCylinder` 抽样交叉验证。

阈值必须在跑正式对比前冻结。本文不擅自写死 PASS 数值。

### P2：实物保真验证

沿用当前独立 plant 的五维要求：

```text
amplitude
frequency
damping
phase
cross-case / cross-method ranking
```

参考信号只能来自冻结的 real RGB liquid-height 或独立液位传感器，必须绑定 source bag、
提取 pipeline、标定、freeze ID 和 hash。以下不能作为 reference：

```text
H_proxy
H_modal
旧 /sim_truth/*
另一个未验证 CFD/SPH signal
```

### P3：R8 H0 development replay

在不改变 H0 runner 的条件下，至少做：

- 静止/低激励/高激励预注册轨迹；
- 成功和 timeout/method-failure 均保留；
- `H_sph` 与 `H_proxy/H_modal` 只做事后相关性和排序比较；
- repeatability、GPU determinism、运行时间和资源报告；
- firewall 静态审计：motion exporter 无控制输入依赖。

### P4：是否申请 formal 接入

只有 P0–P3 全部通过后才选择：

- `P4A`：冻结 post-run liquid replay ABI；或
- `P4B`：开发并冻结 live solver bridge。

任一路径都必须创建新的 R8 liquid release。没有新的 freeze/master/GO/timing/plant
fidelity/firewall 前，formal adapter 继续 NO-GO。

## 13. 风险清单

| 风险                  | 影响                   | 门禁/缓解                                      |
| --------------------- | ---------------------- | ---------------------------------------------- |
| 1 mm 阈值接近粒径     | peak/p95 不可信        | 0.5–1.0 mm 收敛，不用 2 mm 结论               |
| 单粒子 splash 假峰    | height 虚高            | `zsurf` primary，`maxz` 仅 QC              |
| 小容器壁面/润湿       | 阻尼和 crest 偏差      | contact/boundary 敏感性 + 实物标定             |
| C2 粒子数/显存        | OOM、运行过慢          | P0 资源门禁，不能降分辨率后隐瞒                |
| 9 m 绝对路径域        | cell-grid 内存膨胀     | 比较 moving-domain/frame 策略并证明等价        |
| `/odom` 抖动/断点   | 非物理激励             | timestamp/gap/QC，冻结插值，不随结果调参       |
| SPH 数值阻尼          | 排序/衰减错误          | 频率、阻尼、相位和 ranking 实物验证            |
| GPU 非确定性          | paired comparison 漂移 | 固定 binary/device/seed，重复性报告            |
| 单相 SPH 忽略空气     | 封闭/强飞溅场景偏差    | 明确开口条件，OpenFOAM 水/空气复核             |
| 许可证混合            | 难以发布工具链         | core/MeasureTool 分开审查，不再分发未知 binary |
| offline 被误写成 live | 证据身份错误           | 独立 ABI/secondary ledger，禁止复用 live topic |

## 14. 实施工作包和交付物

### WP0：dependency gate

`WP0-A` 已交付：安全 preflight、固定目录、dependency manifest、source/license/ELF hash
和只读复验。`WP0-B` 尚未开始：构建日志、自编译 binary hash 和 C1 静水 smoke 均不存在。

### WP1：geometry/motion

交付：C1/C2 development geometry、mount schema、bag-to-motion exporter、连续性 QC、
绝对路径资源 benchmark。

### WP2：Gauge 和高度 ABI

交付：probe policy、`zsurf` extractor、height report schema、synthetic tests、particle/QC
报告。

### WP3：two-pass development runner

交付：case builder、owned process runner、source hash binding、secondary ledger、H0 replay
报告。此阶段仍 `formal=false`。

### WP4：数值/实物保真

交付：分辨率/时间步收敛、OpenFOAM 抽样、五维 real-reference fidelity report。

### WP5：formal 设计复审

交付：P4A/P4B 决策、更新后的 ABI threat model、release/freeze/GO/timing 计划。没有明确
用户授权不进入实现。

## 15. 需要用户补充/确认的物理输入

实现 WP1 前至少需要：

1. C1/C2 容器总内高、壁厚、底部形状和是否开口；
2. 容器在 Scout 上相对 `base_link` 的安装位姿；
3. 液体类型、温度、实际体积/液深误差；
4. 容器内壁材料及可用的接触角/润湿信息；
5. 实物 RGB 测高的 ROI、采样率、标定和可用 reference cases；
6. 最终“晃动高度”是否采用 crest elevation、absolute elevation 或 peak-to-peak；
7. 计算预算：单 case 可接受的最长离线时间和可用 GPU。

这些信息未齐时可以做 P0 软件 smoke，但不得冻结 C1/C2 物理条件或开始正式矩阵。

## 16. 首轮建议范围

首轮只做下面的最小闭环：

```text
C1 development geometry
+ 一个已有 R8 H0 closed bag
+ DualSPHysics exact-commit isolated build
+ dp=2 mm smoke / dp=1 mm development
+ 16/32 probe zsurf
+ H_crest(t), peak, p95
+ static/sinusoidal/real-odom 三类 QC
+ secondary ledger
```

首轮明确不做：C2/0.5 mm 全量、40/64/88、live ROS bridge、控制器反馈、formal claim、
覆盖现有 `scout_liquid_plant` 或修改实物源码。

## 17. 参考入口

- SIM-R8 总入口：`src/scout_apps/simulation/README.md`
- R8 controller 包：`src/scout_apps/simulation/spmpc_sim_local_planner/README.md`
- 现有 development plant：`src/scout_apps/simulation/scout_liquid_plant/README.md`
- R8 信息防火墙：
  `src/scout_apps/simulation/spmpc_sim_local_planner/config/information_access_policy.yaml`
- C1/C2 development 配置：
  `src/scout_apps/simulation/spmpc_sim_local_planner/config/containers/`
- R8/R7 边界：
  `docs/实物实验注意事项/对比试验/仿真对比试验分析/20260804_SIM-R8源码隔离迁移与R7历史边界.md`
- DualSPHysics Gauge：
  [https://github.com/DualSPHysics/DualSPHysics/blob/master/doc/xml_format/_FmtXML_Gauges.xml](https://github.com/DualSPHysics/DualSPHysics/blob/master/doc/xml_format/_FmtXML_Gauges.xml)
- DualSPHysics motion：
  [https://github.com/DualSPHysics/DualSPHysics/blob/master/src/source/JMotion.cpp](https://github.com/DualSPHysics/DualSPHysics/blob/master/src/source/JMotion.cpp)
- OpenFOAM sloshing cylinder：
  [https://github.com/OpenFOAM/OpenFOAM-dev/tree/master/tutorials/incompressibleVoF/sloshingCylinder](https://github.com/OpenFOAM/OpenFOAM-dev/tree/master/tutorials/incompressibleVoF/sloshingCylinder)
- OpenFOAM interface height：
  [https://github.com/OpenFOAM/OpenFOAM-dev/tree/master/src/functionObjects/field/interfaceHeight](https://github.com/OpenFOAM/OpenFOAM-dev/tree/master/src/functionObjects/field/interfaceHeight)
