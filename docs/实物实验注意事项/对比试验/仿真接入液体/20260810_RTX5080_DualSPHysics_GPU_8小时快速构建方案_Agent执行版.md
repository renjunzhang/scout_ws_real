# Ubuntu 24.04 / RTX 5080 DualSPHysics GPU 8 小时快速构建方案（Agent 执行版）

> 版本：`R8-LIQUID-GPU-BUILD-FAST-v2`
>
> 日期：2026-08-10（Asia/Shanghai）
>
> 当前状态：`PLAN_READY / GPU_BUILD_NOT_STARTED / GPU_RUNTIME_NOT_AUTHORIZED`
>
> v2 修订：闭合 g++-13 条件回退、A/B 精确 profile 映射、强制 static-audit sandbox、
> 低内存并发冻结以及 post-wrapper source-copy receipt 顺序
>
> 目标主机：`LIQUID_ZRJ_MSI_U2404`
>
> 目标产物：`DualSPHysics5.4_linux64`，仅包含 RTX 5080 所需的
> `compute_120 / sm_120`
>
> 硬时间盒：从用户明确下达“开始 GPU 构建”起，最多 8 小时；第 6 小时前必须得到完整候选，
> 最后 2 小时只用于静态审计、回执和一次受控收尾

本文是给后续执行 agent 的自包含操作方案。它追求快速得到一个来源明确、可复查、适配
RTX 5080 的 GPU candidate，同时继承已有 CPU 构建链的证据边界。

本文本身只是一份方案，**不授权现在执行 Make、NVCC、候选二进制、GPU solver、sudo、
AppArmor profile load 或任何系统修改**。真正开始前，执行 agent 必须收到用户明确的执行指令。

## 0. 执行 agent 必读合同

### 0.1 开始前必须完整阅读

执行 agent 在做任何写操作前，必须完整阅读：

1. 本文；
2. [Ubuntu 24.04 液体仿真交接说明的页首和 §11.25](./20260806_Ubuntu24.04液体仿真电脑任务与数据交接说明.md#1125-当前构建与-u3-执行进度总览截至-2026-08-10任务已暂停)；
3. 固定源码中的原始 Makefile：
   `/home/zrj/scout_liquid_lab/dependency/materialized/`
   `u3_source_materialization_v1_20260806T155752Z.partial/src/source/Makefile`；
4. 已通过的 CPU v15 构建策略、profile、gate 和测试，路径见本文 §7。

交接说明是 append-only 历史。旧小节中的“当前状态”可能已经过期，全局状态以页首及
§11.25 为准。

8 小时 T0 之前还必须冻结本文自身：优先将文件纳入 Git 并记录 commit/blob SHA；至少也要生成
只读快照并把精确 SHA-256 写入 campaign start record。文件仍未跟踪、审查中并发变化或 start
record 的方案 hash 不匹配时一律 `BLOCKED_PRECHECK`，不能边执行边改方案。

### 0.2 执行状态机

只允许按以下状态前进：

```text
PLAN_READY
  -> PREFLIGHT_PASS
  -> A_CONTRACT_AND_PROFILES_FROZEN
  -> A_GPU_BUILD_ADMISSION_READY
  -> ATTEMPT_A_RUNNING
  -> A_CANDIDATE_DISARMED
  -> A_STATIC_AUDIT_PASS
  -> PASS_GPU_BUILD_SM120_STATIC_AUDIT
```

只有 G4 证明一个 §11 允许根因时，才允许走条件分支：

```text
ATTEMPT_A_COMPILE_OR_STATIC_AUDIT_FAIL
  -> RETRY_B_ELIGIBLE
  -> B_PROFILES_FROZEN_AND_EXACT_SYSTEM_ACTION_AUTHORIZED
  -> B_GPU_BUILD_ADMISSION_READY
  -> ATTEMPT_B_RUNNING
  -> B_CANDIDATE_DISARMED
  -> B_STATIC_AUDIT_PASS
  -> PASS_GPU_BUILD_SM120_STATIC_AUDIT
```

纯 postflight 合同误报按 §11 修正 parser/contract 后只读重审原 A，不创建 B。缺少任一精确
profile hash 的用户授权时停在 `SYSTEM_ACTION_PENDING`；该状态不是授权，也不是 PASS。

任何阶段失败都必须进入明确的失败状态并保留现场：

```text
BLOCKED_PRECHECK
FAIL_GPU_BUILD_ADMISSION
FAIL_GPU_BUILD_COMPILE
FAIL_GPU_BUILD_STATIC_AUDIT
TIMEBOX_EXHAUSTED
```

禁止把“NVCC 开始运行”“链接成功”“文件存在”或“`nvidia-smi` 能看到显卡”写成 GPU
构建 PASS。唯一构建 PASS 是最终 ELF、依赖和 fatbin 静态审计全部通过。

### 0.3 进度沟通

执行期间至少每 30 分钟向用户报告一次；遇到首个明确错误、需要系统级权限、Attempt A
结束或进入静态审计时立即报告，不等待 30 分钟。报告模板见 §13。

## 1. 8 小时交付定义

### 1.1 必须交付

8 小时内的构建里程碑只包含：

1. 一个 fresh GPU build identity；
2. 一份从 sealed source 派生、仅在新 attempt 内可写的 build copy；
3. 一个最小 `U3GpuBuild.mk` wrapper；
4. 一次成功的 Make/NVCC 构建；
5. 一个已去执行位、模式为 `0400` 的 `DualSPHysics5.4_linux64`；
6. 最终 ELF 的 SHA-256、size、ELF header、program header、dynamic section 和
   RPATH/RUNPATH 结果；
7. `cuobjdump` 对 native `sm_120` cubin 以及配套 PTX 的列表和内容静态证明；
8. source-copy receipt、GPU-build receipt、完整日志 hash 和输出 inventory；
9. 最终状态 `PASS_GPU_BUILD_SM120_STATIC_AUDIT` 或一个精确 blocker。

### 1.2 本时间盒明确不包含

- 不执行新 candidate，包括 `-h`、`-info` 或版本查询；
- 不运行 C1M、cold-A/B、restart 或 DDT；
- 不暴露 `/dev/nvidia*`；
- 不证明 GPU runtime 正常；
- 不证明 CPU/GPU 数值等价；
- 不把 U3 从 `NO_GO_NOT_SETTLED` 改为 PASS；
- 不进入 U4/U5；
- 不安装、升级或替换驱动、CUDA、GCC、内核、AppArmor 或 sysctl；
- 不启动 ROS、Gazebo 或控制器；
- 不在本时间盒内切换到 CMake。

首次 GPU 运行是构建 PASS 后的独立门禁，见 §12。

## 2. 冻结输入与已知环境

### 2.1 不得漂移的身份

| 项目 | 固定值 |
| ---- | ------ |
| 上游仓库 | `https://github.com/DualSPHysics/DualSPHysics` |
| 上游 commit | `ef3721a861fda961f0e2f9ec4cd317b19de99086` |
| 上游 tree | `cef458cb358712f4694b9d2148f638440418e9dc` |
| sealed source | `/home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial` |
| source receipt | `/home/zrj/scout_liquid_lab/audits/u3_source_materialization_v1_20260806T155752Z.json` |
| source receipt SHA-256 | `90af263fb7ec8b7d6a46a53aa5354dd5b676cd4167d48fde93911e014a70b745` |
| source receipt canonical SHA-256 | `017a660bd38da43ae14fb97df9f19cc6cd2b90cf56f91ce047869dbf795d93e2` |
| sealed manifest SHA-256 | `a9a3debebff5ae30bb2d6226deb997f15a6eae0d07581a7c147ce95b9c3da9d2` |
| sealed source inventory | 352 files；5,473,917 B；0 symlink；0 hardlink |
| CPU baseline binary SHA-256 | `5aa464a8f37b0185bac863987f0d1079a0f1a3d6daead6581562c832278ea202` |
| CPU build receipt SHA-256 | `d407233107d4bc9eeea81b0c5a95cbfc98ad6529f1cb7494b68ae5ecc4b1d604` |

官方 `master` 在本文制定时与固定 commit 相同，未提供新的官方 `sm_120` 适配。不得在
8 小时时间盒中更新上游、换 commit 或重新 clone。

### 2.2 目标工具链

| 项目 | 首选值 | 说明 |
| ---- | ------ | ---- |
| GPU | NVIDIA GeForce RTX 5080 Laptop GPU | compute capability 12.0 |
| PCI | `00000000:01:00.0` | 每次 preflight 复核 |
| 驱动 | `580.173.02` | `nvidia-smi` 的 CUDA 13.0 是驱动支持上限 |
| CUDA Toolkit | `12.8.93` | 实际编译工具链，不是 CUDA 13.0 |
| CUDA 根 | `/usr/local/cuda-12.8` | 使用绝对路径，不依赖 alternatives |
| NVCC | `/usr/local/cuda-12.8/bin/nvcc` | 必须支持 `compute_120` 与 `sm_120` |
| host compiler A | `/usr/bin/x86_64-linux-gnu-g++-11` | 首选；11.5.0 |
| host compiler B | `/usr/bin/x86_64-linux-gnu-g++-13` | 仅按 §11 单变量回退；真实 helper 位于 `/usr/libexec/gcc/x86_64-linux-gnu/13/` |
| Make | `/usr/bin/make` | 首次构建唯一入口 |
| 并发 | G0 冻结为 `-j1` 或 `-j2` | `4–<8 GiB` 用 `-j1`；`>=8 GiB` 且无持续内存压力才用 `-j2` |

当前终端的 `PATH` 不含 NVCC，因此所有策略、日志和命令必须使用 CUDA 工具绝对路径。
不要通过全局 `export PATH=...` 隐藏实际工具身份。

截至本文 v2 修订时，本机 `MemAvailable` 约 5.6–6.0 GiB，因此当前条件对应 `-j1`。真正执行
仍须在 G0 和 G3 前重采，不能把这次快照当作未来准入证据。

## 3. 不可破坏边界

### 3.1 永远只读的既有证据

不得原地编辑、删除、移动、改名、chmod、hardlink 或 symlink 复用：

- sealed materialization；
- bare repository；
- 所有 CPU build tree、CPU candidate 和 build receipt；
- 所有历史失败 attempt；
- C1M GenCase v8 输出；
- cold-A、cold-B、restart、10.05 s extension 和 DDT-ramp；
- `/home/zrj/scout_liquid_lab` 下任何已有 `.partial`。

本项目的 `.partial` 不表示“可删除的临时文件”。成功 attempt 也故意保留该后缀。

### 3.2 新构建的边界

- 每次 attempt 使用新的 UTC identity 和此前不存在的目录；
- **build child** 的唯一可写 bind 是该 attempt 的 `output/` 和隔离 tmp；
- **外层 supervisor/review agent** 只可在用户授权范围内 create-new 写入本方案列出的
  `audits/` receipts、新增的 GPU policy/profile/schema/gate/test 文件，以及 source-copy 完成后
  唯一、字节冻结的 `U3GpuBuild.mk`；这不扩大 build child 的写权限，也不允许覆盖任何既有
  仓库文件；
- sealed source 只读输入，先复制到新 build tree，再编译；
- 构建 namespace 断网；
- static-audit namespace 同样断网、只读输入且零宿主可写 bind；
- 构建阶段不挂任何 `/dev/nvidia*`，并记录 `gpu_device_exposed=false`；
- 不运行 repo 预编译 ELF，不运行新 candidate；
- 不向 `scout_ws/build`、`install`、`log` 或 ROS 环境写入；
- 不使用 `sudo make`、root shell、Docker、ROS 环境或 Gazebo 环境；
- 如果 AppArmor profile 需要加载/卸载，必须单独获得用户对精确系统动作的授权；
- 失败后只终止本 attempt 的进程组，保留目录、对象和日志，不做隐式清理。

## 4. 为什么首次构建只走 Make

固定源码的原始 Makefile 已经定义完整 GPU 对象集合和 CUDA 链接流程；当前 CMake 仍使用
遗留 FindCUDA，缺少 `sm_120`，且在 source-only tree 下更容易触发 Chrono、WaveGen、
MoorDynPlus 和 install 布局问题。

8 小时路线只做以下最小适配：

1. `DIRTOOLKIT=/usr/local/cuda-12.8`；
2. `NCC=/usr/local/cuda-12.8/bin/nvcc`；
3. `CC=/usr/bin/x86_64-linux-gnu-g++-11`；
4. 只生成：

   ```text
   -gencode=arch=compute_120,code=\"sm_120,compute_120\"
   ```

5. `USE_FAST_MATH=NO`；
6. `USE_NATIVE_CPU_OPTIMIZATIONS=NO`；
7. `COMPILE_CHRONO=NO`；
8. `COMPILE_WAVEGEN=NO`；
9. `COMPILE_MOORDYNPLUS=NO`；
10. `LIBS_DIRECTORIES=`；
11. 继承 CPU 已验证的 `-include cstdint` 兼容修复。

不得修改 `FunctionsCuda.cpp` 中止于旧架构的“CUDA cores”显示表。它可能导致未来 RTX 5080
运行日志显示 `0 CUDA cores`，但只影响信息展示，不影响 kernel 选择或求解；首次构建不要扩大
补丁范围。

## 5. Build identity、目录与产物

### 5.1 命名

G1 一次性预声明 campaign 及 A/B 两个精确 attempt identity。copy/build 是两类 profile 模板，
但 A/B 的可写根不同；一旦 B 被消费，映射必须闭合为 A-copy、A-build、B-copy、B-build 四个
唯一命名、唯一路径和唯一 hash 的精确实例；另有一个只读 campaign static-audit profile。若 A
直接通过，B 两实例保持 `NOT_MATERIALIZED`。B-build 的最终字节
取决于 G4 证明的唯一回退根因，必须到 G5 再冻结，不能在 G1 预制一个囊括所有回退权限的超集。
B 目录在真正需要前不得创建：

```text
campaign_id = u3_source_gpu_build_sm120_<YYYYmmddTHHMMSSZ>
build_id A  = <campaign_id>_a
build_id B  = <campaign_id>_b
root        = /home/zrj/scout_liquid_lab/build/<build_id>.partial
```

Attempt A 失败后，Attempt B 使用已经预声明但从未消费的 B identity；其实际 start/end UTC 另记入
receipt。不得在 Attempt A 中重新执行 `make`，不得覆盖同名 receipt，也不得提前创建 B root。

### 5.2 最小目录布局

```text
<build_id>.partial/
└── output/
    ├── artifacts/
    │   └── DualSPHysics5.4_linux64
    └── buildtree/
        └── src/source/
            ├── Makefile
            ├── U3GpuBuild.mk
            ├── *.cpp / *.cu / *.h
            └── *.o
```

为保持 CPU v15 的 fail-closed output inventory，build child 不创建额外 `logs/` 或 `metadata/`
文件。完整 combined output、preflight、toolchain hash、readelf/cuobjdump 结果和 bounded prefixes
由外层 gate 直接写入 create-new receipt；不得靠放宽 build output allowlist 临时解决冲突。

外部 create-new receipts：

```text
/home/zrj/scout_liquid_lab/audits/<build_id>_source_copy.json.partial
/home/zrj/scout_liquid_lab/audits/<build_id>_source_copy.json
/home/zrj/scout_liquid_lab/audits/<build_id>_gpu_build.json.partial
/home/zrj/scout_liquid_lab/audits/<build_id>_gpu_build.json
# 仅在消费 B 时：
/home/zrj/scout_liquid_lab/audits/<build_id_B>_b_admission.json.partial
/home/zrj/scout_liquid_lab/audits/<build_id_B>_b_admission.json
# 仅在纯 postflight 合同修正时，<revision_id> 必须 create-new：
/home/zrj/scout_liquid_lab/audits/<build_id>_static_reaudit_<revision_id>.json.partial
/home/zrj/scout_liquid_lab/audits/<build_id>_static_reaudit_<revision_id>.json
```

每个 `.json.partial` 都是对应阶段在动作前 create-new 的不可变 start record；final receipt 只能
在该阶段完成后以另一个 create-new 文件发布。不得先写一个可变 JSON 再原地改成 PASS。

B admission 与 static re-audit 若被触发，也遵守同一 start/final create-new 规则。它们必须引用
G1 base policy/schema/gate/tests/profile 的路径与 SHA-256 作为 `parent_contract`，不得覆盖 G1
文件、A receipts 或既有 GPU-build receipt。

## 6. 8 小时时间表与硬停点

| 墙钟时间 | 阶段 | 必须得到的结果 |
| -------- | ---- | -------------- |
| T+0:00–0:20 | G0 动态 preflight | 环境身份、资源、进程、源码和工具链 PASS/FAIL |
| T+0:20–1:50 | G1/G2 冻结合同和 admission | 预声明 A/B、GPU delta、schema/tests/profile query/self-check；系统动作授权明确 |
| T+1:50–3:20 | G3 Attempt A | g++-11、G0 冻结的 `-j1/-j2`、90 分钟硬超时的 clean fresh build |
| T+3:20–3:50 | G4 静态审计/诊断 | ELF、NEEDED、RPATH、sm_120 cubin/PTX、对象 inventory 或单一根因 |
| T+3:50–4:20 | G5 B 准备 | 只实现一个根因对应的最小 delta set；不提前写 B 输出 |
| T+4:20–5:50 | G5 Attempt B | 使用预声明但未消费的 B identity；90 分钟硬超时 |
| T+5:50–6:20 | G6 最终审计 | T+6:00 前必须已出现完整 candidate，否则停止构建探索 |
| T+6:20–7:30 | G7 回执与交接 | final receipts、hash、状态、blocker/next stage |
| T+7:30–8:00 | 缓冲 | 只补审计和文档，不开启第三次构建或 CMake |

如果 T+6:00 仍没有完整 candidate，立即进入 `TIMEBOX_EXHAUSTED`；不要挤占最后两小时去
尝试 CMake、旧架构、未知 CUDA、放宽隔离或第三次编译。

### 6.1 阶段划分与各阶段 Agent goal

本方案按 `G0`–`G7` 分为 **8 个编号阶段**。`G5/G6` 是条件阶段：只有 Attempt A 未直接
通过、且 G4 从证据中确认恰好一个符合 §11 的可回退根因时才执行；若 A 在 G4 已通过完整
静态审计，则直接进入 G7。T+7:30–8:00 的缓冲只用于补审计、回执和清场，不算第 9 个阶段。

每次只给执行 agent 设置当前阶段的一个 goal。goal 应同时写明唯一目标、冻结输入、create-new
输出、机器可判的验收条件以及停止/非目标；完成某个阶段的 goal 只表示该阶段已经产生 PASS
或精确 blocker 及证据，不表示整个 GPU build 已通过。goal 本身也不是 sudo、AppArmor
加载/卸载或 candidate 执行授权；需要精确系统动作时，agent 只能核验用户授权是否已经存在，
不能自行取得、推定或绕过授权。只允许一个执行 agent 写当前 attempt，其他 agent 只能只读审查
或监控。

| 阶段 | 可直接交给 Agent 的 goal |
| ---- | ------------------------ |
| G0：动态 preflight | `只读执行 §7.1 的全部动态 preflight，核验主机、冻结身份、sealed source/receipts、CUDA/GCC/Make、资源、内存 PSI/swap、进程和 Git 工作树；按内存规则冻结本 campaign 的 parallel_jobs，输出 PREFLIGHT_PASS 或带完整证据的 BLOCKED_PRECHECK。不得创建 build tree、加载 profile 或启动编译。` |
| G1：冻结构建合同 | `在 G0=PREFLIGHT_PASS 后预声明本 campaign 唯一的 A/B identity；以 CPU v15 为只读参考，create-new 实现并静态验证最小 GPU v1 policy、schema、gate、tests、两类 copy/build profile 模板、A 精确实例、B identity/delta 合同、只读 static-audit profile 及 wrapper 字节/hash；输出冻结合同或 FAIL_GPU_BUILD_ADMISSION。不得创建 B-build 权限超集、修改 CPU v15/既有证据、复制源码、加载 profile、创建 B root 或运行 Make。` |
| G2：Attempt A 准入 | `只消费 G0 PASS 和 G1 冻结合同；核验精确系统动作授权已存在后，为 A 创建 fresh root/start record，按“复制→核验 352 项→O_EXCL/0600 写 wrapper→复核 353 项完整输入→发布 final source-copy receipt”的顺序完成准入，并通过 profile query/self-check 和固定 Make argv 审查；输出 A_GPU_BUILD_ADMISSION_READY。授权不存在时输出 SYSTEM_ACTION_PENDING 并停止；不得绕过隔离、创建 B root 或运行 Make。` |
| G3：Attempt A | `在 A_GPU_BUILD_ADMISSION_READY 后重采内存并确认不低于已冻结并发门槛，再通过冻结 gate 只运行一次 Attempt A，固定使用 CUDA 12.8、g++-11、G0/G1 冻结的 -j1 或 -j2 和唯一 sm_120/compute_120 GENCODE；完整封存日志、资源和输出 inventory。若产生 regular、非 symlink、nlink=1、size>0 的 candidate，立即收紧为 0400；否则输出真实编译失败。不得在 A 内重跑、运行中改并发、暴露 GPU、执行 candidate 或把 Make rc=0 宣布为构建 PASS。` |
| G4：A 静态裁决 | `只能通过 §9 强制的只读、断网 static-audit sandbox 审计 A 的 0400 candidate，或只读诊断 A 的失败日志；输出且只输出三类结论之一：A 完整静态审计通过并可进入 G7、恰好一个符合 §11 的根因并可进入 G5、或不可回退的精确 blocker。宿主不得进行任何内容感知的 ELF/fatbin/ET_REL 解析，不得修改/执行 candidate、改合同、启动 B 或重建。` |
| G5：条件受控回退 | `仅在 G4 已证明一个 §11 允许根因且未越过硬停点时应用唯一最小 delta set。若需要 B，先 create-new 发布引用 G1 parent hashes 的 B-specific admission manifest，确定性生成 B-copy/B-build 精确实例，冻结 name/path/hash、通过冻结的 schema/gate/tests/query/self-check 并核验该 hash 的用户授权，再创建 B root、fresh source copy 并执行唯一一次 Attempt B；不得修改 G1/A 文件。g++-13 权限只允许出现在已证明该编译器根因的 B-build。若只是纯 postflight 合同误报，则 create-new 发布 parser/contract revision 和独立 re-audit receipt 后重审原 A，不得覆盖旧 receipt 或创建 B。禁止权限超集、第三次构建、CMake、降级架构、多变量试错或执行 candidate。` |
| G6：条件最终静态验收 | `只能通过 §9 强制的只读、断网 static-audit sandbox 审计 G5 交付的最终唯一 0400 candidate：它只能是 B，或纯 postflight 合同修正后未被修改的原 A。完成 ELF、NEEDED、RPATH/RUNPATH、对象 inventory、sm_120 native cubin、配套 PTX、GENCODE 和隔离闭合检查；输出 STATIC_AUDIT_PASS 或 FAIL_GPU_BUILD_STATIC_AUDIT。不得在宿主运行任何 binary-content parser，也不得修 candidate、重建或执行；T+6:00 后不得再启动任何构建。` |
| G7：回执与交接 | `汇总本 campaign 的不可变证据，create-new 发布 final receipts、日志/产物 hash、inventory、profile 卸载和零残留结果；报告 PASS_GPU_BUILD_SM120_STATIC_AUDIT 或精确 blocker，并把 next_allowed_stage 限定为 SEPARATE_GPU_RUNTIME_EXECUTION_ADMISSION_REQUIRED。不得启动 GPU runtime、C1M、U3、U4 或 U5。` |

若使用不同 agent 依次执行，各阶段交接至少必须传递：阶段状态、campaign/build identity、证据路径
及 SHA-256、是否发生系统动作、candidate 是否存在且是否始终未执行，以及下一阶段唯一允许的
动作。后续 agent 必须重新核验这些字段，不能只接受上一 agent 的自然语言结论。

## 7. G0/G1：preflight 与最小实现

### 7.1 只读 preflight

执行 agent 应先在 G0 捕获下列命令的完整 stdout/stderr 和 return code；`PREFLIGHT_PASS` 后
创建 fresh attempt root，再把未改写的结果及 SHA-256 写入其 preflight 证据。若 preflight 失败，
只允许在 `audits/` 写 create-new no-go receipt，不创建可误认成已开始构建的 build tree。此处是
命令清单，不代表本文已执行：

```bash
date --iso-8601=seconds
lsb_release -ds
uname -r
nvidia-smi
nvidia-smi \
  --query-gpu=name,uuid,pci.bus_id,compute_cap,driver_version,memory.total,memory.used,memory.free,temperature.gpu,power.draw,utilization.gpu,utilization.memory \
  --format=csv,noheader
nvidia-smi \
  --query-compute-apps=pid,process_name,used_memory \
  --format=csv,noheader
readlink -f /usr/local/cuda
/usr/local/cuda-12.8/bin/nvcc --version
/usr/local/cuda-12.8/bin/nvcc --list-gpu-arch
/usr/local/cuda-12.8/bin/nvcc --list-gpu-code
/usr/local/cuda-12.8/bin/cuobjdump --version
/usr/local/cuda-12.8/bin/ptxas --version
/usr/local/cuda-12.8/bin/fatbinary --version
/usr/local/cuda-12.8/bin/nvlink --version
if test -x /usr/local/cuda-12.8/bin/nvdisasm; then
  /usr/local/cuda-12.8/bin/nvdisasm --version
else
  echo OPTIONAL_NVDISASM_MISSING
fi
/usr/bin/x86_64-linux-gnu-g++-11 --version
/usr/bin/x86_64-linux-gnu-g++-13 --version
/usr/bin/x86_64-linux-gnu-g++-11 -print-prog-name=cc1plus
/usr/bin/x86_64-linux-gnu-g++-11 -print-prog-name=collect2
/usr/bin/x86_64-linux-gnu-g++-11 -print-prog-name=as
/usr/bin/x86_64-linux-gnu-g++-11 -print-prog-name=ld
/usr/bin/x86_64-linux-gnu-g++-13 -print-prog-name=cc1plus
/usr/bin/x86_64-linux-gnu-g++-13 -print-prog-name=collect2
/usr/bin/x86_64-linux-gnu-g++-13 -print-prog-name=as
/usr/bin/x86_64-linux-gnu-g++-13 -print-prog-name=ld
/usr/bin/make --version
/usr/bin/file --version
/usr/bin/readelf --version
readlink -f /usr/share/misc/magic.mgc
ls -l \
  /usr/local/cuda-12.8/bin/nvcc \
  /usr/local/cuda-12.8/bin/cudafe++ \
  /usr/local/cuda-12.8/bin/ptxas \
  /usr/local/cuda-12.8/bin/fatbinary \
  /usr/local/cuda-12.8/bin/nvlink \
  /usr/local/cuda-12.8/bin/cuobjdump \
  /usr/local/cuda-12.8/nvvm/bin/cicc
ls -l \
  /usr/bin/x86_64-linux-gnu-g++-11 \
  /usr/lib/gcc/x86_64-linux-gnu/11/cc1plus \
  /usr/lib/gcc/x86_64-linux-gnu/11/collect2 \
  /usr/bin/x86_64-linux-gnu-g++-13 \
  /usr/libexec/gcc/x86_64-linux-gnu/13/cc1plus \
  /usr/libexec/gcc/x86_64-linux-gnu/13/collect2
ls -l /etc/magic /usr/share/misc/magic.mgc /usr/lib/file/magic.mgc
test -r /usr/local/cuda-12.8/lib64/libcudart_static.a
test -r /usr/local/cuda-12.8/include/cuda_runtime.h
sha256sum \
  /usr/local/cuda-12.8/bin/nvcc \
  /usr/local/cuda-12.8/bin/cudafe++ \
  /usr/local/cuda-12.8/bin/ptxas \
  /usr/local/cuda-12.8/bin/fatbinary \
  /usr/local/cuda-12.8/bin/nvlink \
  /usr/local/cuda-12.8/bin/cuobjdump \
  /usr/local/cuda-12.8/nvvm/bin/cicc \
  /usr/local/cuda-12.8/lib64/libcudart_static.a \
  /usr/local/cuda-12.8/bin/nvcc.profile \
  /usr/local/cuda-12.8/include/cuda_runtime.h \
  /usr/local/cuda-12.8/nvvm/libdevice/libdevice.10.bc \
  /usr/bin/x86_64-linux-gnu-g++-11 \
  /usr/lib/gcc/x86_64-linux-gnu/11/cc1plus \
  /usr/lib/gcc/x86_64-linux-gnu/11/collect2 \
  /usr/bin/x86_64-linux-gnu-g++-13 \
  /usr/libexec/gcc/x86_64-linux-gnu/13/cc1plus \
  /usr/libexec/gcc/x86_64-linux-gnu/13/collect2 \
  /usr/bin/timeout \
  /usr/bin/aa-exec \
  /usr/sbin/apparmor_parser \
  /usr/bin/bwrap \
  /usr/bin/env \
  /usr/bin/prlimit \
  /usr/bin/make \
  /usr/bin/dash \
  /usr/bin/rm \
  /usr/bin/as \
  /usr/bin/ld \
  /usr/bin/ld.bfd \
  /usr/bin/x86_64-linux-gnu-as \
  /usr/bin/x86_64-linux-gnu-ld \
  /usr/bin/x86_64-linux-gnu-ld.bfd \
  /usr/bin/file \
  /usr/bin/readelf \
  /usr/bin/sha256sum \
  /usr/bin/vmstat \
  /etc/magic \
  /usr/lib/file/magic.mgc
ls -l \
  /dev/nvidia0 \
  /dev/nvidiactl \
  /dev/nvidia-uvm \
  /dev/nvidia-uvm-tools \
  /dev/nvidia-modeset
df -h /home/zrj/scout_liquid_lab/build
df -i /home/zrj/scout_liquid_lab/build
free -h
awk '/MemAvailable|SwapTotal|SwapFree/ {print}' /proc/meminfo
cat /proc/pressure/memory
/usr/bin/vmstat 1 5
ps -eo pid=,comm=,args= | rg 'make|nvcc|ptxas|DualSPHysics' || true
sha256sum \
  /home/zrj/scout_liquid_lab/audits/u3_source_materialization_v1_20260806T155752Z.json \
  /home/zrj/scout_liquid_lab/audits/u3_source_cpu_build_20260807T023724Z_cpu_build.json
```

还必须进行以下定向检查：

```bash
/usr/local/cuda-12.8/bin/nvcc --list-gpu-arch | rg '^compute_120$'
/usr/local/cuda-12.8/bin/nvcc --list-gpu-code | rg '^sm_120$'
rg -n '__GNUC__ > 14|unsupported GNU version' \
  /usr/local/cuda-12.8/include/crt/host_config.h
rg -n '^(DIRTOOLKIT|CC)[[:space:]]*=|compute_120|sm_120|compute_86|sm_86' \
  /home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/src/source/Makefile
```

`PREFLIGHT_PASS` 的最低条件：

- GPU 名称、PCI、compute capability 和驱动符合 §2，并记录一次 compute/graphics 快照；
- 绝对路径 `/usr/local/cuda-12.8/bin/nvcc` 为 12.8.93；`/usr/local/cuda` symlink 只记录，
  因本方案不依赖它，单独漂移不构成技术 blocker；
- NVCC 同时列出 `compute_120`、`sm_120`；
- g++-11、g++-13 及各自真实 `cc1plus`/`collect2` helper、Make 和 CUDA 静态审计工具存在，
  realpath/mode/hash 全部已记录；
- `file` 的精确数据依赖 `/etc/magic` 与解析后的 `/usr/lib/file/magic.mgc` 存在且 hash 已记录；
- 可用内存至少 4 GiB；build 分区可用空间至少 20 GiB；
- `4 GiB <= MemAvailable < 8 GiB` 时冻结 `parallel_jobs=1`；只有 `MemAvailable >= 8 GiB`
  且 `vmstat 1 5` 的后四个样本均 `si=0, so=0`、memory PSI `full avg10=0.00` 且
  `some avg10<=0.10` 时才冻结 `parallel_jobs=2`；其他 `>=4 GiB` 情况均冻结为 `-j1`；
- sealed source、source receipt 和 CPU receipt hash 精确匹配；
- 没有会写同一 attempt、造成内存不足或违反用户明确排他要求的竞争任务；GPU compute task
  本身不是离线 NVCC 构建的技术 blocker，因为 build namespace 不暴露 GPU；
- Git 中已有用户修改已被识别且不会被覆盖。

`nvidia-smi --query-compute-apps` 在没有 compute task 时应是 return code 0 且 stdout 为空；
证据记录器不得把“成功的空结果”误判为命令失败。compute task 为空是首选的项目排他状态，
但不是 NVCC 的技术依赖；只有用户明确要求独占、资源不足或任务身份异常时才阻断离线构建。

本机当前没有 `/usr/local/cuda-12.8/bin/nvdisasm`。因此 `nvdisasm` 和
`cuobjdump --dump-sass` **不是本方案的必过项**，也不得为了构建里程碑擅自安装
`cuda-nvdisasm-12-8`。如果用户以后另行授权安装，才将其作为附加 SASS 证据，并把真实路径、
版本、SHA-256 和 AppArmor execute 权限加入新 revision；不能回写本次 campaign receipt。

CUDA 组件 patch 版本不保证完全相同：本机 NVCC/ptxas 等为 12.8.93，而 cuobjdump 为
12.8.90。receipt 必须分别记录每个组件的真实版本，不能把全部工具笼统写成 12.8.93。

任一固定身份不符都进入 `BLOCKED_PRECHECK`，不得“就近选择”另一个 CUDA、GPU、commit 或
source tree。

G0 选出的并发数必须作为字面整数进入 policy、schema、gate、tests、Make argv 和 receipt；G3
真正启动前再次采样，资源比 G0 变差时必须停下重做准入，不能在运行中的 Make 动态改 `-j`。
构建期间每 10–30 秒记录 `MemAvailable`、`SwapFree`、`vmstat` swap-in/out、memory PSI 以及
`cicc`/`ptxas`/`cc1plus` RSS。已有 swap 占用本身不等于正在抖动，但不得把剩余 swap 当作满足
`MemAvailable` 门槛的替代物。

### 7.2 复用 CPU v15，不重造框架

GPU 构建实现应以以下已通过 CPU v15 文件为只读参考：

```text
src/scout_apps/simulation/scout_dualsphysics_liquid/
├── config/apparmor_drafts/r8-liquid-u3-cpu-build-v15.profile
├── config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v15.json
├── schema/target_host_u3_cpu_build_execution_policy_v15.json
├── scripts/r8_liquid_target_u3_cpu_build_gate_v15.py
└── tests/test_target_u3_cpu_build_gate_v15.py
```

用 `apply_patch` 新增独立 GPU v1 文件，保留 CPU v15 byte-for-byte。GPU 版本只应增加或修改：

- GPU build identity、artifact 名称和 object contract；
- `/usr/local/cuda-12.8` 的只读路径和精确 CUDA tool allowlist；
- g++-11、按唯一根因启用的 g++-13、NVCC、GENCODE 和 GPU Make argv；
- `U3GpuBuild.mk` 固定内容；
- CUDA tool、fatbin、native cubin/PTX 的强制隔离 postflight；
- `network_used=false`、`gpu_device_exposed=false`、
  `compiled_artifact_executed=false` 的闭合检查。

不要在 8 小时内抽象一个通用 CPU/GPU gate 框架，也不要回改 CPU v15。优先复用其已经验证的
create-new、source-copy、AppArmor/bwrap、timeout/prlimit、candidate disarm、ELF 审计规则和 receipt
核心函数，只做最小差异；所有内容感知的 ELF/fatbin/ET_REL 解析逻辑必须移入 §9.2 sandbox，
不能复用 CPU v15 的宿主 in-process parser 执行位置。**不能机械复制 v15 的 byte-pin 断言**：
v15 会导入/固定 v14，并要求
profile 只允许 identity/output 差异，这与 GPU 所需的 CUDA execute/read、g++-11、资源和对象合同
必然冲突。GPU v1 必须改成预先冻结的 GPU delta contract，并为这些差异写正向断言和测试。

GPU v1 相对 CPU v15 的资源差异也必须进入 policy、schema、gate 和 tests：

```text
parallel_jobs = 1  # 4 GiB <= G0 MemAvailable < 8 GiB
# 或 parallel_jobs = 2，仅当 G0 MemAvailable >= 8 GiB 且无持续 swap/PSI 压力
parallel_jobs_memory_threshold_bytes = 8589934592
wall_timeout_seconds = 5400
cpu_limit_seconds = 5400
minimum_available_memory_bytes = 4294967296
address_space_limit_bytes = 8589934592
memory_monitor_interval_seconds = 10..30
```

实际 policy 只能写一个已经由 G0 选定的 `parallel_jobs` 字面整数，不能保留条件表达式或注释。
不得残留 CPU v15 的 `-j4`、3600 s，也不得把 4 GiB 最低准入误写成 `-j2` 准入线。

### 7.3 构建隔离差异

GPU build namespace 应继承 CPU v15 的：

- `clearenv`；
- empty `HOME` 语义；
- `--unshare-net`；
- `/usr` 只读 bind；
- 唯一 attempt output 可写 bind；
- 独立 tmp；
- wall/CPU/address-space/file-size/process/open-file/core 限制；
- `aa-exec -> bwrap -> env -> prlimit -> make` 顺序。

GPU build 的 guest 路径必须精确冻结为：

```text
--tmpfs /work
--dir /work/tmp
--bind <exact-attempt-output> /work/output
--chdir /work/output/buildtree/src/source
TMPDIR=/work/tmp
TMP=/work/tmp
TEMP=/work/tmp
```

不提供 guest `/tmp` 写权限；NVCC 的 `tmpxft_*` 必须全部落在 `/work/tmp`。如果实际 NVCC
无视三个临时目录变量并尝试写 `/tmp`，应 fail-closed 并记录证据，不得开放宿主 `/tmp` 写 bind。

source-copy 和 GPU-build 是两类隔离模板，不是两个可跨 attempt 复用的 profile。campaign
采用以下精确映射；B 未被消费时，第 3/4 项保持 `NOT_MATERIALIZED`：

1. **A-copy**：只允许 sealed source 向 A 的 fresh buildtree 做固定复制，保持 CUDA-blind；
2. **A-build**：只允许写 A output，固定 g++-11 与 CUDA 12.8 build toolchain；
3. **B-copy**：只允许 sealed source 向 B 的 fresh buildtree 做固定复制，保持 CUDA-blind；
4. **B-build**：只允许写 B output，且只含 G4 已批准的一个 delta set；
5. **campaign static-audit**：只读允许 A/B 两个精确 candidate 和需审计的精确对象输入，零宿主
   可写 bind；详细合同见 §9.2。

前四项是 A/B × copy/build 四个唯一 profile 实例，可由两类模板确定性生成，不要求手写四套
重复文本；但每个实例必须有唯一 name/path/SHA-256，只能包含自己的 exact attempt root，不能用
glob 或一个同时可写 A/B 的 profile。每个 phase 都按 `load -> run -> unload -> zero-residue`
闭合，禁止把 copy/build 合并为长期加载或过度放宽的 profile。

G1 冻结模板、A 的两个精确实例、B identity/允许 delta 合同和 static-audit profile。由于 B-build
权限取决于 A 的真实根因，G5 只能通过 G1 已冻结的 closed schema/gate/tests，create-new 生成
B-specific admission manifest 和 B-copy/B-build 最终字节。manifest 必须记录唯一 delta、精确
Make argv、两个 profile name/path/hash 及所有 G1 parent hashes；随后执行 parser query、
tests/self-check，并核验用户对这两个精确 hash 的系统动作授权。不得原地修改 G1 policy/schema/
gate/tests、A profile 或 A receipts；完成前不得创建 B root 或运行 B。
禁止提前制作一个同时允许 g++-11、g++-13 和所有潜在 deny 修复的 B-build 权限超集。

CUDA 12.8 位于 `/usr/local/cuda-12.8`，已经包含在 `/usr` 只读树中，但 AppArmor execute/read
规则仍必须覆盖 NVCC 实际调用的精确工具。至少审查：

```text
/usr/local/cuda-12.8/bin/nvcc
/usr/local/cuda-12.8/bin/cudafe++
/usr/local/cuda-12.8/bin/ptxas
/usr/local/cuda-12.8/bin/fatbinary
/usr/local/cuda-12.8/bin/nvlink
/usr/local/cuda-12.8/nvvm/bin/cicc
/usr/bin/x86_64-linux-gnu-g++-11
/usr/lib/gcc/x86_64-linux-gnu/11/cc1plus
/usr/lib/gcc/x86_64-linux-gnu/11/collect2
/usr/bin/as
/usr/bin/ld
/usr/bin/ld.bfd
/usr/bin/x86_64-linux-gnu-as
/usr/bin/x86_64-linux-gnu-ld
/usr/bin/x86_64-linux-gnu-ld.bfd
/usr/bin/rm
/usr/local/cuda-12.8/bin/nvcc.profile
/usr/local/cuda-12.8/nvvm/libdevice/**
/usr/local/cuda-12.8/targets/x86_64-linux/include/**
/usr/local/cuda-12.8/targets/x86_64-linux/lib/**
```

上表前半是 execute 候选；`nvcc.profile`、libdevice、headers 和 CUDA libraries 只获得精确
read/mmap 权限。实现时必须解析 symlink 后的真实目标并冻结 mode/hash，不能把 `/**` 直接翻译成
可写或可执行的宽泛 AppArmor 规则。

g++-13 回退不能沿用 A-build。只有 G4 已证明“g++-11 特有 host compile 错误”时，B-build
才加入以下本机已确认身份，并在 G5 重新计算、精确匹配后冻结到 policy/profile/tests；其他 B
根因的 profile 必须明确拒绝这三条 execute 权限：

| B tool | 精确路径 | 本文 v2 核验的 SHA-256 |
| ------ | -------- | ----------------------- |
| g++-13 | `/usr/bin/x86_64-linux-gnu-g++-13` | `1353e9bdd29a7295c7226bf6c63abccce056d8cac31f112e5cdbecc3f28c2769` |
| cc1plus-13 | `/usr/libexec/gcc/x86_64-linux-gnu/13/cc1plus` | `840b332fb62ec6f694ac77d91fe69ef7f80b0d69512ed89374af0ee7a506255d` |
| collect2-13 | `/usr/libexec/gcc/x86_64-linux-gnu/13/collect2` | `4d1f341ae5b763b513258ee2812422a45e063c30a2f1924a0cf63d3699f3a158` |

G5 还必须复核 g++-13 对 `as`/`ld` 的解析结果、所需 GCC 13 只读树和 NVCC `-ccbin`/host
调用身份；任一路径或 hash 漂移都使该回退 fail-closed，不能“就近”选择其他 helper。
tests 至少应证明：A-build 不含 g++-13；非编译器根因的 B-build 不含 g++-13；只有选中该
delta 的 B-build 同时包含上述三条精确 execute 规则；任一 helper 路径/hash 漂移、A profile
出现 B root、B profile 出现 A root或同一 profile 可写两个 output 都必须失败。

NVCC dry-run 已确认 CUDA 12.8 的 A 编译路径会调用 g++-11、`cudafe++`、`cicc`、`ptxas`、
`fatbinary` 和用于清理隔离临时文件的 `/usr/bin/rm`；最终 allowlist 仍须以 GPU gate 自己的
dry-run 和实际 AppArmor audit 再冻结。若出现 deny，只增加被证据证明必需的精确
CUDA/host-tool 路径。禁止开放整个 home、网络、GPU、`/dev` 或 unconfined。

如果需要加载新的 AppArmor profile，执行 agent 必须把 profile diff、parser query 结果、加载和
卸载命令精确展示给用户并取得授权。没有授权时状态为 `A_CONTRACT_AND_PROFILES_FROZEN / SYSTEM_ACTION_PENDING`，
不得绕过隔离直接构建。

## 8. G2/G3：wrapper 与唯一首选构建命令

### 8.1 最小 wrapper

只在 fresh build copy 中新增 `U3GpuBuild.mk`：

```make
.SUFFIXES:
.SUFFIXES: .cpp .o
include Makefile
override CCFLAGS += -include cstdint
```

wrapper 的字节内容和 SHA-256 必须进入 policy 与 receipt。不要原地编辑上游 Makefile。

source-copy 的发布顺序必须固定，不能把“源码复制通过”和“完整 build input 已冻结”混为一件事：

1. create-new 写不可变 `<build_id>_source_copy.json.partial` start record；
2. 在对应 copy profile 中复制 sealed source，退出并卸载该 profile；
3. 逐项核验 sealed manifest 声明的 352 个 entry；
4. 外层 gate 以 `O_CREAT|O_EXCL`、mode `0600` create-new 写入唯一 `U3GpuBuild.mk`；
5. **在 wrapper 写入后重新遍历并冻结完整 build input**：352 个 sealed entry 加 1 个 wrapper，
   合计精确 353 个 regular、non-symlink、`nlink=1` 文件；wrapper 字节/hash/mode 精确匹配，且
   不存在其他 extra、hardlink、ELF 或带执行位文件；
6. 只有上述复核通过后，才 create-new 发布 final source-copy receipt 并允许 build admission。

G1 tests 必须构造负例，拒绝没有 post-wrapper 353-entry inventory、wrapper 先后顺序不明、
wrapper 非 `0600` 或 final receipt 在完整复核前发布的实现。

首建不默认向 NVCC 添加 pre-include；只有出现来自 `.cu` 翻译单元的 `SIZE_MAX`/`uint*_t`
同类错误，才按 §11 在 Attempt B 中增加：

```make
override NCCFLAGS += --pre-include cstdint
```

### 8.2 Make argv 骨架

在已经完成 §8.1 的 final source-copy receipt 和隔离自检之后，只执行以下等价的固定 Make 目标。
代码中的 `<FROZEN_PARALLEL_JOBS>` 只是文档占位符；实际 policy、gate argv 和 receipt 必须直接写
字面 `1` 或 `2`，不得通过 shell/env 在运行时替换：

```bash
/usr/bin/make \
  --no-builtin-rules \
  --no-builtin-variables \
  -f U3GpuBuild.mk \
  -j<FROZEN_PARALLEL_JOBS> \
  SHELL=/usr/bin/dash \
  CC=/usr/bin/x86_64-linux-gnu-g++-11 \
  NCC=/usr/local/cuda-12.8/bin/nvcc \
  CUDA=12 \
  DIRTOOLKIT=/usr/local/cuda-12.8 \
  'GENCODE=-gencode=arch=compute_120,code=\"sm_120,compute_120\"' \
  USE_DEBUG=NO \
  USE_FAST_MATH=NO \
  USE_NATIVE_CPU_OPTIMIZATIONS=NO \
  COMPILE_CHRONO=NO \
  COMPILE_WAVEGEN=NO \
  COMPILE_MOORDYNPLUS=NO \
  LIBS_DIRECTORIES= \
  EXECS_DIRECTORY=/work/output/artifacts \
  /work/output/artifacts/DualSPHysics5.4_linux64
```

要求：

- wall timeout 为 5400 s，并设置短 kill-after；
- 使用 G0/G1 冻结的 `-j1` 或 `-j2`，不使用机器全部 24 个 logical CPU；按本文 v2 修订时
  的约 5.6–6.0 GiB `MemAvailable`，当前应冻结为 `-j1`；
- sanitized child environment 显式设置 `TMPDIR=/work/tmp`，不得继承 ROS、Conda、用户
  `LD_LIBRARY_PATH` 或宿主 shell 的 CUDA/PATH 状态；
- 显式构建 artifact target，不调用默认 `all`，从而保留 `.o` 供审计；
- fresh tree 不运行 `make clean`；
- stdout/stderr 由外层 gate 流式捕获，记录完整 byte count/SHA-256 和有界前缀；不在 build
  output 新建 `make.combined.log`；
- 构建命令中不得出现 `-use_fast_math`、`-ffast-math`、`-march=native`、
  `--allow-unsupported-compiler`、sm_61、sm_70、sm_86 或其他 GPU 架构；
- 构建阶段不读取 GPU 设备；宿主 `nvidia-smi` 监控由 supervisor 独立采样；
- supervisor 每 10–30 秒记录 `MemAvailable`、`SwapFree`、swap-in/out、memory PSI 以及
  `cicc`/`ptxas`/`cc1plus` RSS；监控只记录或触发 fail-closed 停止，禁止在线修改 Make 并发；
- Make 返回非零时，先封存日志和输出 inventory，再决定是否符合 §11 的唯一回退。

CPU v15 实测 source-only 构建约 325 s。GPU 首建因 NVCC/ptxas 更慢，但 90 分钟应是充足硬上限；
连续 10 分钟没有新日志、没有编译 CPU 活动且对象数不增长时，判定 stalled，停止本 attempt。

## 9. G4/G6：candidate disarm 与静态验收

### 9.1 先去执行位，再审计

Make 返回 0 后，gate 必须立即验证候选是单一 regular file、非 symlink、`nlink=1`、size > 0，
然后把新 candidate 模式收紧为 `0400`。在此之前和之后都不得运行它。

如果 disarm 失败，状态为 `FAIL_GPU_BUILD_STATIC_AUDIT`；不得为了检查版本临时恢复执行位。

### 9.2 必须执行的非运行审计

CPU v15 的 build bwrap 退出后，guest `/work` 已不存在。宿主 supervisor 必须先从已验证的
`build_id` 生成唯一 artifact 路径，使用 `O_NOFOLLOW`/`fstat` 复核 realpath、父目录、owner、
`st_dev`、`st_ino`、regular、`nlink=1`、size、mode=`0400` 和 SHA-256；但宿主进程**不得直接**
解析新 ELF/fatbin/ET_REL 的内容。该禁令同时覆盖 `file`、`readelf`、`cuobjdump` 等外部工具和
Python/其他语言的 in-process parser；宿主只能消费 sandbox 返回的有界文本、状态和 stream hash。

外部静态解析必须是独立、强制的 `static-audit` 隔离 phase，不是可选加固。固定调用链为：

```text
timeout --foreground --kill-after=5s <exact-wall-time>
  -> aa-exec -p <exact-campaign-static-audit-profile>
  -> bwrap
  -> env -i
  -> prlimit
  -> <one exact file/readelf/cuobjdump/sha256sum argv>
```

static-audit bwrap/profile 必须同时满足：

- bwrap 固定 `--die-with-parent`、`--new-session`、`--clearenv`、
  `--unshare-user/pid/net/ipc/uts`、`--disable-userns`、`--assert-userns-disabled`；断网、无
  `/proc`、无 `/dev`、无 GPU；
- `/usr` 只读绑定并只创建既定 `/lib -> usr/lib`、`/lib64 -> usr/lib64` 合成链接；
- 额外只读绑定 hash 已冻结的 `/etc/magic`；`/usr/share/misc/magic.mgc` 必须解析到已冻结的
  `/usr/lib/file/magic.mgc`，不得为 `file` 开放宽泛 `/etc/**` 或 `/usr/share/**`；
- 每次只把精确 candidate（或一个已冻结的精确对象输入）只读绑定到 `/audit/input/...`；
- `/audit/tmp` 只来自 guest tmpfs，**零宿主可写 bind**，stdout/stderr 只经 pipe 返回 supervisor；
- sanitized `HOME=/nonexistent`，所有工具使用绝对路径；
- audit profile 只允许 `bwrap`、`env`、`prlimit`、`file`、`readelf`、`cuobjdump`、`sha256sum`
  及其精确 loader/library/data read/mmap；禁止 shell、Make、g++、NVCC、linker、candidate execute、
  host home/workspace/receipt 写入；若 bwrap 建立 user/network namespace 确实需要 CPU v15 已证明的
  `userns create`、mount、capability、ptrace/signal 及 `network unix/inet/inet6 dgram`、
  `network netlink raw` bootstrap 规则，必须逐条冻结；禁止 stream/packet 规则，并由负测证明
  child 始终处于空 `--unshare-net` namespace、无外部连通性，不能据此给 parser 增加可用网络；
- 每条 `file/readelf/sha256sum` argv 最多 wall/CPU 60 s、AS 1 GiB；`cuobjdump --list-*`
  最多 wall/CPU 120 s、AS 2 GiB；`cuobjdump --dump-*` 最多 wall/CPU 300 s、AS 4 GiB；
  所有命令固定 `nproc<=64`、`nofile<=128`、core=0，单条输出超过 256 MiB 即 fail-closed；
- profile 必须以 G1 冻结的精确 hash 单独获得系统动作授权，每次审计均
  `load -> query enforce -> run -> unload -> zero-residue`，不得复用 build profile。

本文 v2 已核验 `/etc/magic` SHA-256 为
`58219ec4bfe06d84640b4e86341feb3099cb078146c9eee73ec55152819df247`，
`/usr/lib/file/magic.mgc` 为
`72a25195a2623fe160e926bf20952b6b74b29d6c91e0174a5fa062f02beee1aa`；G0/G1 必须重算并精确
匹配，不能只相信本文记录。

policy/schema/gate/tests 必须把上述 prefix、工具 allowlist、每类资源上限和输出上限冻结为闭合
合同，并以负例拒绝任何宿主 `--bind`、缺少 `--die-with-parent`/`--new-session`/`--clearenv`/
`--unshare-net`/`--kill-after`、profile 中出现 write/compiler/shell 权限、tool suffix 绕过 prefix、
候选路径不是只读 exact bind 或审计前后身份变化。

下面只列出 audit sandbox **内部**的 tool suffix。`GPU_ARTIFACT_PATH` 仅供 supervisor 做只读
bind 源；sandbox 内工具只能看到固定 guest 路径：

```bash
GPU_BUILD_ID='u3_source_gpu_build_sm120_REPLACE_WITH_EXACT_UTC_ID'
GPU_ARTIFACT_PATH="/home/zrj/scout_liquid_lab/build/${GPU_BUILD_ID}.partial/output/artifacts/DualSPHysics5.4_linux64"
/usr/bin/file /audit/input/DualSPHysics5.4_linux64
/usr/bin/readelf -hW /audit/input/DualSPHysics5.4_linux64
/usr/bin/readelf -lW /audit/input/DualSPHysics5.4_linux64
/usr/bin/readelf -dW /audit/input/DualSPHysics5.4_linux64
/usr/bin/readelf -nW /audit/input/DualSPHysics5.4_linux64
/usr/local/cuda-12.8/bin/cuobjdump \
  --list-elf /audit/input/DualSPHysics5.4_linux64
/usr/local/cuda-12.8/bin/cuobjdump \
  --dump-elf --gpu-architecture sm_120 /audit/input/DualSPHysics5.4_linux64
/usr/local/cuda-12.8/bin/cuobjdump \
  --dump-elf-symbols --gpu-architecture sm_120 /audit/input/DualSPHysics5.4_linux64
/usr/local/cuda-12.8/bin/cuobjdump \
  --list-ptx /audit/input/DualSPHysics5.4_linux64
/usr/local/cuda-12.8/bin/cuobjdump \
  --dump-ptx /audit/input/DualSPHysics5.4_linux64
/usr/bin/sha256sum /audit/input/DualSPHysics5.4_linux64
```

代码块中的 `REPLACE_WITH_EXACT_UTC_ID` 是文档占位符，执行 agent 必须在调用只读工具前替换
并验证完整 ID。实际 audit argv 还必须包含上面的固定 sandbox prefix；不得把代码块中的 tool
suffix 直接拿到宿主运行，也不能暗中复用已退出的 build namespace。

禁止使用 `ldd`，因为它不是本方案需要的纯静态依赖解析方式。

readelf/cuobjdump 输出也必须使用流式 byte count + SHA-256 + 有界前缀合同；不得把无上限 dump
全文塞进 JSON 或写入 build output。完整性由 stream hash 证明，人工诊断使用有界前缀和可重复的
只读 argv。每次 sandbox 调用前后还必须复核 candidate 的 `st_dev`、`st_ino`、size、mode 和
SHA-256 完全不变；任一 parser timeout、资源超限、输出超限、AppArmor deny 或身份变化都进入
`FAIL_GPU_BUILD_STATIC_AUDIT`。

### 9.3 PASS 条件

全部满足才允许 `PASS_GPU_BUILD_SM120_STATIC_AUDIT`：

1. Make return code 为 0，未超时、未 OOM、未被 signal 终止；
2. candidate 是 x86-64 `ET_DYN` PIE，interpreter 精确为
   `/lib64/ld-linux-x86-64.so.2`，带 `DF_1_PIE`，entry point 落在 executable `PT_LOAD`
   内，GNU stack 不可执行；
3. 没有 RPATH/RUNPATH；
4. NEEDED 正向 allowlist 精确为
   `libgomp.so.1`、`libstdc++.so.6`、`libm.so.6`、`libgcc_s.so.1`、`libc.so.6`；
   任何新增或缺失 SONAME 都进入人工审查，不以“看起来是系统库”自动放行；
5. Make 当前采用 `cudart_static`，因此出现意外动态 `libcudart.so` 必须解释并 fail-closed；
6. `cuobjdump --list-elf` 非空且只列出 `sm_120` native cubin，
   `--dump-elf --gpu-architecture sm_120` return code 为 0 且输出非空；绑定当前冻结的
   cuobjdump 12.8.90 输出语法后，聚合结果必须至少解析到一个名称严格以 `.text.` 开头
   （不得把 `.rela.text.*` 算入）、type=`PROGBITS`、带 executable flag、十六进制 size > 0
   的 section。`--dump-elf-symbols` 还应解析到至少一个定义在已验证非零 `.text.*` section 的
   function symbol 作为辅助证据；不得强制固定 kernel 名、GLOBAL binding 或每个 cubin 各有一个
   符号，以免把优化/可见性差异误判为失败；
7. `cuobjdump --list-ptx` 非空且只列出配套 `*.sm_120.ptx`，`--dump-ptx` 中的
   `arch`/`.target` 为 `sm_120`；CUDA 12.8 的命名不会保证出现字面 `compute_120`；
8. `code=compute_120` 的存在由冻结的 GENCODE、完整 NVCC argv/log 以及非空 PTX 三者共同证明，
   不通过在 `--list-ptx` 输出中搜索字面 `compute_120` 判定；
9. `cuobjdump --list-elf/--list-ptx` 的架构字段不得出现 sm_61、sm_70、sm_86 或其他非预期
   架构；禁止对整个源码、ELF strings 或 Makefile 做裸字符串搜索来判定，因为原 Makefile
   本来就保留旧分支文本；
10. 所有 NVCC 命令均来自 CUDA 12.8 绝对路径并含唯一 GENCODE；
11. 顶层对象全部是预期 regular ET_REL 文件；固定 Makefile 在本文变量覆盖下经 `make -pn`
    只读静态展开为 131 个无重名顶层对象，其中 11 个是 `.cu` 对应对象；GPU policy 必须再次
    计算并冻结这份 exact contract；
12. source copy 除固定 wrapper 和编译对象外没有不可解释变化；
13. build 与 static-audit 均为 `network_used=false`，并且
    `gpu_device_exposed=false`、`precompiled_binary_executed=false`、
    `compiled_artifact_executed=false`、`binary_content_parser_executed_on_host=false`；
14. candidate 已是 `0400`，SHA-256、size 和完整 inventory 已记录；
15. 本 campaign 实际使用的 A/B copy/build 精确 profile 实例及 static-audit profile 均已卸载，
    `aa-exec`、`bwrap`、`make`、`nvcc`、`cudafe++`、`cicc`、`ptxas`、`fatbinary`、外部 parser
    和本 attempt mount 均为零残留；
16. 所有受监控 sysctl 前后不变；若经单独授权使用过 sudo，必须清除 timestamp，并记录
    candidate 在完整生命周期内始终未执行；
17. static-audit 每条 argv 的 profile/hash、sandbox prefix、资源上限、return code、stream hash
    和输出字节数已记录，且 candidate 审计前后 `st_dev`、`st_ino`、size、mode、SHA-256 不变。

## 10. Receipt 最小合同

### 10.1 Source-copy receipt

`<build_id>_source_copy.json` 至少包含：

- `document_type`、`build_id`、UTC start/end；
- 上游 URL、commit、tree；
- sealed source path、receipt hash、manifest hash；
- source-copy 文件数、总字节、逐文件 hash manifest；
- symlink/hardlink/ELF/executable-bit 检查；
- 对 receipt 声明的 352 个 sealed entry 逐项复算并与 sealed manifest 对齐，不能只检查 receipt
  文件自身 SHA-256；
- copy profile 已卸载后，以 `O_CREAT|O_EXCL`、mode `0600` create-new 写 wrapper 的顺序证据；
- wrapper path、精确字节、内容 hash、mode、regular/non-symlink/`nlink=1`；
- wrapper 写入后的完整 build-input inventory：精确 353 个文件、无其他 extra，并带 canonical
  manifest SHA-256；
- final receipt 的发布时间晚于 post-wrapper 完整 inventory 完成时间；
- build root/device/mount identity；
- `network_used=false`；
- `status=PASS_U3_GPU_SOURCE_COPY`；
- `next_allowed_stage=SEPARATE_GPU_BUILD_EXECUTION_ADMISSION_REQUIRED`。

### 10.2 GPU-build receipt

`<build_id>_gpu_build.json` 至少包含：

- 完整有效 Make argv，不只记录模板；
- sanitized child environment；
- timeout、aa-exec、bwrap、env、prlimit、Make、实际 host g++ 及其 `cc1plus`/`collect2`、NVCC、cudafe++、ptxas、fatbinary、
  nvlink、cicc、file、readelf、cuobjdump、sha256sum 的路径、SHA-256，以及工具明确支持时的
  version 输出；`cicc --version` 会以 missing-input 返回非零，因此 cicc 只记录 path/stat/hash
  和 NVCC dry-run 中的调用身份，不制造假版本门禁；
- GPU 名称、UUID、PCI、compute capability、driver、显存和构建前后动态快照；
- CUDA root、NVCC version、`--list-gpu-arch/code` 证据；
- wall timeout、资源上限、G0/G3 内存/PSI/swap 快照、冻结并发数及构建期资源监控摘要；
- A/B copy/build 精确 profile 实例和 campaign static-audit profile 的 name/path/hash，以及
  policy/schema/gate/tests 的 hash；
- 若使用 B：B-specific admission manifest path/hash、G1 parent hashes、唯一 delta 和用户授权绑定
  的精确 profile hashes；若进行纯 postflight 重审：create-new revision/re-audit receipt path/hash；
- stdout/stderr bytes、完整性、SHA-256、return code、elapsed time；
- object exact inventory；
- candidate mode、size、SHA-256、ELF header、GNU_STACK、NEEDED、RPATH/RUNPATH；
- cuobjdump 的 list/dump cubin、dump ELF symbols 与 list/dump PTX 结果及各自 hash，以及至少
  一个非零 executable `.text.* PROGBITS` section 的结构化证据；
- static-audit 每条固定 sandbox argv、资源/输出上限、return code、stream hash、AppArmor deny
  计数，以及 candidate 审计前后身份不变证据；
- `static_audit_sandbox_pass=true`、`binary_content_parser_executed_on_host=false`；
- `network_used=false`；
- `gpu_device_exposed=false`；
- `upstream_code_executed=false`；
- `compiled_artifact_executed=false`；
- 每个实际使用的 copy/build/static-audit profile 各自 load/query/run/unload 身份和结果；
- `aa-exec`、`bwrap`、compiler/CUDA tool、mount 和本 attempt 进程零残留检查；
- 受监控 sysctl 前后值；如使用 sudo，记录 timestamp 已清除；
- 最终 `status` 和 `next_allowed_stage`。

成功时：

```text
status = PASS_GPU_BUILD_SM120_STATIC_AUDIT
gpu_runtime_status = NOT_RUN
u3_acceptance = NO_GO_NOT_SETTLED
u4_authorized = false
u5_authorized = false
next_allowed_stage = SEPARATE_GPU_RUNTIME_EXECUTION_ADMISSION_REQUIRED
```

失败 receipt 也必须 create-new 并保留真实状态，不得复制成功 receipt 或手工改写 PASS。

## 11. 唯一一次受控回退矩阵

Attempt A 未在 G4 直接通过后，只有错误能被下表精确分类时才允许受控回退。若需要 Attempt B，
它使用预声明但未消费的 build ID、fresh source copy 和新 receipts；一次只处理**一个已证明根因
对应的最小 delta set**。
一个 delta set 可以包含同一根因不可分割的数个精确字段，但不得顺带改变无关工具链、架构或物理参数。

G4 分类后、创建 B root 前，必须确定性生成 B-copy/B-build 两个精确 profile 实例。B-build 只能
包含该根因需要的最小权限变化；两个 profile 都必须重新冻结 name/path/hash。G5 以
`O_CREAT|O_EXCL` 发布 B-specific admission start/final manifest，引用 G1 base contract 的完整
parent hashes，并由 G1 冻结的 schema/gate/tests 验证唯一 delta、B argv 和 profile 映射，再通过
`apparmor_parser -Q -K -T` 和 self-check。不得原地编辑任何 G1/A 文件。只有用户授权这些
**精确 hash**后才能创建 B root；缺少授权时停止在 `SYSTEM_ACTION_PENDING`，不得沿用 A profile、
加载权限超集或直接运行 B。

若唯一根因是纯 postflight 合同误报，修复也必须 create-new 形成新的 `revision_id`、parent hashes、
parser/contract/tests hash 及独立 static re-audit start/final receipt。旧 GPU-build receipt、旧审计
输出和 candidate 保持不变；新 revision 只能只读重审同一 `0400` candidate，不能把旧 receipt
原地改成 PASS。

| 明确根因 | Attempt B 唯一允许变化 | 禁止做法 |
| -------- | ---------------------- | -------- |
| 主机内存 OOM/kill | 仅当 A 因 G0 `MemAvailable>=8 GiB` 而冻结为 `-j2` 时，允许 B 改为 `-j1` | A 已是 `-j1` 时同类重试；降架构、开 fast-math、复用旧对象 |
| g++-11 特有 host compile 错误 | `CC -> g++-13`，并只在 B-build 加入 §7.3 三个精确 tool/helper 路径、hash、GCC 13 只读树和对应测试 | 其他 B 根因顺带获得 g++-13 权限；加 `--allow-unsupported-compiler` |
| `.cu` 翻译单元缺 `SIZE_MAX`/`uint*_t` | wrapper 增加 `override NCCFLAGS += --pre-include cstdint` | 修改 sealed header |
| `unsupported gpu architecture` | 修正 NVCC 绝对路径，仍用 CUDA 12.8/sm_120 | 降到 sm_90/sm_86 或只留旧 PTX |
| 找不到 Chrono/Wave/MoorDyn | 作为一个依赖关闭 delta set，修复三项 `NO` 和空 `LIBS_DIRECTORIES` 的覆盖 | 物化或运行 repo 预编译库 |
| 找不到 `cudart_static` | 修复 sandbox 中 CUDA 12.8 lib64 的只读可见性 | 换未知系统 CUDA |
| AppArmor deny | 作为一个权限根因 delta set，只增加该 phase 的 audit evidence 证明必需的精确 tool/header/lib/tmp 规则，并重新冻结 profile hash/测试/授权 | 开放整个 `/usr/local` 写、home、网络、GPU、host output 写或 unconfined |
| GENCODE 引号/Make 覆盖未生效 | 保持工具链不变，只修正固定 argv/解析 | 降级架构或改物理源码 |
| 纯 postflight 合同误报 | 修正 GPU v1 自己的 sandbox static-audit parser/contract，并对现有 `0400` candidate 只读重审；不重建 Attempt B | 把 parser 移到宿主、改 candidate、删除失败 attempt |

以下错误不允许在本时间盒内重试：

- NVCC internal compiler error 且无已知最小修复；
- 完整 NVCC argv/GENCODE 已确认正确但最终 ELF 仍无 sm_120 cubin，且没有明确最小修复；
- candidate 损坏或 cuobjdump 已确认不支持/无法解析，且没有明确最小修复；
- source/receipt/hash 漂移；
- 需要驱动、CUDA、内核或系统包升级；
- 需要修改 DualSPHysics 数值/物理源码才能编译；
- A 已冻结为 `-j1` 仍发生 OOM/kill；
- 第 6 小时仍没有完整 candidate；
- 用户未授权必需的系统级 profile 动作。

这些情况应输出精确 blocker，不得为了“8 小时完成”制造假 PASS。

## 12. 构建 PASS 后的下一道 GPU runtime 门

构建 PASS 不授权执行 candidate。下一次用户明确要求 GPU smoke 时，应另立：

```text
run_id = u3_c1m_gpu_smoke_<UTC>
```

并遵守以下最小边界：

- 只读使用固定 C1M GenCase v8：9,078 粒子，其中 moving 2,669、fluid 6,409；
- 固定 BI4 SHA-256：`b463ddfe548b3db78b02f23b075dadbdd3c71ea766eb092701b56978ddb3a8e7`；
- 固定 XML SHA-256：`28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb`；
- 固定 `DsphConfig.xml`：293 B，SHA-256
  `0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd`；
- 保持原始 XML/BI4 和数值参数不变，不使用尚未验收的 DDT-ramp；
- fresh output root；
- 首次只运行到 1.0 s，`tout=0.05`；
- solver argv 冻结为：

  ```text
  <candidate> <case-prefix>/C1M_zero <fresh-output>
  -gpu:0 -ompthreads:1 -stable:1 -vres:0 -cellmode:full
  -tmax:1.0 -tout:0.05 -sv:binx,info -svres:1
  -svtimers:0 -svdomainvtk:0 -saveposdouble:1
  -nortimes:1 -createdirs:1 -csvsep:0
  ```

- 只精确开放 `/dev/nvidiactl`、`/dev/nvidia0`、`/dev/nvidia-uvm`；
- `/dev/nvidia-uvm-tools` 仅在实际 access evidence 证明需要时追加；
- 不开放 `/dev/nvidia-modeset`、`/dev/dri/*` 或整个 `/dev`；
- 每次 run preflight 都用 `stat` 冻结三个节点的 character-device 类型、owner、mode 和动态
  major:minor；尤其 UVM major 不得跨重启硬编码。bwrap exact bind、AppArmor 精确 rw/ioctl/mmap
  语义和 device-cgroup allow（若使用）必须同时满足；permission deny 必须 fail-closed；
- driver/runtime libraries 只读，网络关闭；`libcuda.so.1` 必须来自 `ldconfig -p` 解析出的宿主
  64-bit `/lib/x86_64-linux-gnu/libcuda.so.1` 真实链，明确禁止 Toolkit 的
  `/usr/local/cuda-12.8/targets/x86_64-linux/lib/stubs/libcuda.so`；
- NVML 监控留在宿主 supervisor，不为了 `nvidia-smi` 扩大 solver profile；
- 监控 CUDA error、Xid/reset、温度、显存、NaN/Inf、粒子数、`Nout` 和帧推进；
- 1 s smoke 的输出合同为 21 个 `Part_####.bi4`、总计 30 个精确输出文件、每帧 9,078
  粒子、moving 2,669、`Nout=0`、`PartMotionRef` 严格零运动，并通过 finite、density、domain
  和输出时间槽检查；
- 先通过 1 s functional smoke。若随后运行 GPU 8.05 s 只是 backend parity benchmark，必须使用
  独立 benchmark identity，并明确它不属于 settled-candidate 恢复；若要恢复 settle campaign，
  则必须遵守权威交接说明的 DDT-first 顺序：先只读完成现有 DDT QC，参数候选过冻结阈值后
  才重新做 fresh cold-A/B/restart；
- CPU/GPU 跨后端不得预设 BI4 字节完全相同，必须先冻结按 Idp 对齐的 Posd/Vel/Rhop
  和积分状态容差。

每个 GPU runtime receipt 还必须强制带有以下结论上限：

```text
development_only = true
formal = false
fidelity_validation_status = SIM_ONLY_UNVALIDATED
physical_primary_eligible = false
production_authorized = false
settled_state_authorized = false
u4_authorized = false
u5_authorized = false
```

GPU functional PASS 与 U3 settle PASS 是两件事。即使 GPU 运行更快，原冻结阈值仍必须独立满足：

```text
speed RMS <= 0.001 m/s
speed P95 <= 0.001 m/s
speed max <= 0.005 m/s
specific KE <= 5e-7 J/kg
并通过位置冻结、coverage、粒子完整性和密度检查
```

## 13. Agent 进度与最终报告模板

### 13.1 进行中更新

```text
GPU_BUILD_STATE: <state>
ELAPSED: <hh:mm>
BUILD_ID: <id or NOT_CREATED>
CURRENT_STAGE: <G0..G7>
LAST_COMMAND_CLASS: <read-only / source-copy / build / static-audit>
RESULT: <PASS / RUNNING / FAIL / BLOCKED>
EVIDENCE: <receipt/log path + SHA-256 if available>
NEXT: <one concrete next action>
RISK_OR_BLOCKER: <none or exact blocker>
```

### 13.2 成功交接

```text
status = PASS_GPU_BUILD_SM120_STATIC_AUDIT
gpu_runtime_status = NOT_RUN
u3_acceptance = NO_GO_NOT_SETTLED
u4_authorized = false
u5_authorized = false
build_id = ...
upstream commit/tree = ...
host compiler = ...
nvcc = ...
parallel_jobs + G0/G3 memory evidence = ...
make elapsed/rc = ...
artifact path = ...
artifact SHA-256/size/mode = ...
ELF NEEDED/RPATH/RUNPATH = ...
sm_120 native cubin list/dump = PASS
sm_120 nonzero executable .text.* + function-symbol evidence = PASS
sm_120 PTX list/dump = PASS
GENCODE includes compute_120 = PASS
static-audit sandbox/profile = PASS
binary_content_parser_executed_on_host = false
network_used = false
gpu_device_exposed = false
compiled_artifact_executed = false
source-copy receipt path/hash = ...
gpu-build receipt path/hash = ...
next_allowed_stage = SEPARATE_GPU_RUNTIME_EXECUTION_ADMISSION_REQUIRED
```

### 13.3 失败交接

```text
status = <exact failure state>
failed stage = ...
attempt id(s) = ...
first failing argv = ...
return code/signal/timeout = ...
first actionable error = ...
logs and hashes = ...
candidate exists = true/false
candidate executed = false
old evidence modified = false
retry used = true/false
root-cause delta set = ...
exact blocker = ...
recommended next action = ...
```

## 14. 最终检查表

执行 agent 在宣布完成前逐项确认：

- [ ] 用户已明确授权开始构建；涉及系统 profile 动作时另有精确授权；
- [ ] 完整阅读本文、交接说明页首与 §11.25；
- [ ] Git 现有修改已识别并保留；
- [ ] upstream commit/tree、source receipt、sealed manifest 精确匹配；
- [ ] CUDA 12.8.93、compute_120、sm_120、g++-11 全部通过 preflight；
- [ ] g++-11/g++-13 及各自 helper 的 realpath/mode/hash 已冻结；若未选择 g++-13 delta，实际
  build profile 不含其 execute 权限；
- [ ] G0/G3 已按 `<4 / 4–<8 / >=8 GiB` 规则冻结并复核 `-j1/-j2`，构建期间没有动态改并发；
- [ ] 使用新的 build ID 和 fresh `.partial` root；
- [ ] sealed source、CPU tree、历史 attempt 未被修改；
- [ ] 仅用 Make，未切 CMake；
- [ ] 关闭 fast-math、native、Chrono、WaveGen、MoorDynPlus；
- [ ] 使用唯一 sm_120 GENCODE；
- [ ] 构建断网且未暴露 GPU 设备；
- [ ] A/B copy/build 使用各自 exact-root profile 实例；若使用 B，其 profile 在唯一根因分类后
  才冻结、测试并按精确 hash 获得授权；
- [ ] 若使用 B，已 create-new 发布带 G1 parent hashes 的 B-specific admission manifest；若修正
  postflight 合同，已使用新 revision 和独立 re-audit receipt，未覆盖 G1/A/旧 receipt；
- [ ] source-copy final receipt 在 wrapper 写入和 353-entry 完整输入复核之后才发布；
- [ ] candidate 从未执行并已收紧为 `0400`；
- [ ] 所有 `file/readelf/cuobjdump/sha256sum` 外部解析均在强制的只读、断网 static-audit
  sandbox 内完成，宿主没有直接解析新 ELF/fatbin；
- [ ] ELF、NEEDED、RPATH/RUNPATH 静态审计通过；
- [ ] sm_120 native cubin、配套 PTX 及 GENCODE 中 compute_120 的三层证据均通过；
- [ ] sm_120 cubin 至少含一个非零 executable `.text.* PROGBITS` section，函数符号辅助证据已记录；
- [ ] source-copy/build receipts、日志和 inventory 均 create-new 且已 hash；
- [ ] 失败 attempt 原样保留；若使用 Attempt B，只处理一个有证据的根因及其最小 delta set；
- [ ] 实际使用的 A/B copy/build profile 和 campaign static-audit profile 已卸载，相关进程、
  mount 和 sudo timestamp 零残留；
- [ ] 没有把 GPU build PASS 写成 runtime、U3 settle、U4/U5 或正式安全认证 PASS；
- [ ] 下一阶段明确停在 `SEPARATE_GPU_RUNTIME_EXECUTION_ADMISSION_REQUIRED`。

---

本方案的速度来自复用已经成功的 CPU v15 source-copy、隔离、candidate disarm、ELF 审计和
receipt 结构，只增加 CUDA 12.8、`sm_120`、A/B 精确 profile 映射及强制 static-audit sandbox
所需的最小差异；不通过降低证据标准、污染旧产物或把“编译完成”冒充“GPU 仿真完成”来换取
表面进度。
