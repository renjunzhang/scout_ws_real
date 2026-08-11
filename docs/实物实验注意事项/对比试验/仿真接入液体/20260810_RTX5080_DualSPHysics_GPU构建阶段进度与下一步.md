# RTX 5080 DualSPHysics GPU 构建阶段进度与下一步

> 最后核验：2026-08-11 23:09:24+08:00
>
> 当前状态：`PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION`
>
> 构建状态：`PASS_GPU_BUILD_CANDIDATE_DISARMED`
>
> 下一门禁：`STAGE5_REQUIRES_SEPARATE_USER_AUTHORIZATION_AND_PHYSICAL_INPUTS`

本文是可更新的进度页，不修改、不取代冻结的
[RTX 5080 GPU 8 小时快速构建方案](./20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md)。
冻结方案 SHA-256 仍为：

```text
9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d
```

## 1. 当前结论

GPU 构建、静态审计、GPU 功能冒烟测试和阶段 4 液体单独数值验证均已完成。当前停止在
第 4 阶段完成点，尚未进入第 5 阶段。

```text
GPU_BUILD_PASS=true
GPU_STATIC_AUDIT_PASS=true
GPU_RUNTIME_SMOKE_PASS=true
STAGE4_LIQUID_ONLY_VALIDATION_COMPLETE=true
STAGE4_STATUS=PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION
DEVELOPMENT_ONLY=true
PHYSICAL_FIDELITY_VALIDATED=false
STAGE5_ENTERED=false
```

这里的 PASS 只说明固定软件、固定参数和固定数值门槛下的开发验证通过，不证明与实物液体一致，
也不是生产或正式验收结论。阶段 5 小车与液体耦合、阶段 6 回放与实物对比均未开始。

## 2. 六阶段进度

| 阶段 | 内容 | 当前状态 | 结论 |
| --- | --- | --- | --- |
| 1 | RTX 5080 GPU 程序构建 | `PASS_GPU_BUILD_CANDIDATE_DISARMED` | **PASS** |
| 2 | 静态审计 | `PASS_GPU_BUILD_SM120_STATIC_AUDIT` | **PASS** |
| 3 | GPU 运行冒烟测试 | `PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY` | **PASS** |
| 4 | 液体单独验证 | `PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION` | **PASS（development-only）** |
| 5 | 小车与液体耦合 | `NOT_STARTED / NOT_ADMITTED` | 未开始 |
| 6 | 回放与 CPU/实物对比 | `NOT_STARTED / NOT_ADMITTED` | 未开始 |

当前停止点：

```text
阶段 1 构建 PASS
  → 阶段 2 静态审计 PASS
  → 阶段 3 GPU functional smoke PASS
  → 阶段 4 液体单独数值验证 PASS（development-only）
  ↛ 阶段 5 未开始，需单独授权和实物参数
  ↛ 阶段 6 未准入
```

## 3. GPU 构建与静态审计

早期 campaign `u3_source_gpu_build_sm120_20260810T102641Z` 曾因 AppArmor 缺少
`/newroot/work/tmp/` 的精确 create 权限而在 Make 启动前失败，并按原 T+6 时间盒发布
`TIMEBOX_EXHAUSTED` 回执。该失败 root、日志和 receipts 仍原样保留。

后续 fresh campaign 已采用经过验证的最小权限修复并完成构建：

```text
campaign=u3_source_gpu_build_sm120_20260810T170339Z
build_id=u3_source_gpu_build_sm120_20260810T170339Z_a
Make_rc=0
Make_elapsed_seconds=744.931052
object_count=131
candidate_mode=0400
candidate_size_bytes=105654136
candidate_sha256=cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f
```

Candidate：

```text
/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T170339Z_a.partial/output/artifacts/DualSPHysics5.4_linux64
```

静态审计已证明：

- 131 个对象完整；
- 唯一 native 架构为 `sm_120`；
- 配套 `compute_120` PTX 存在；
- ELF header、动态依赖、RPATH/RUNPATH 和 kernel section 通过；
- candidate 与对象在审计前后身份不变；
- candidate 在构建和静态审计阶段均未执行。

关键回执：

| 证据 | SHA-256 |
| --- | --- |
| `/home/zrj/scout_liquid_lab/audits/u3_source_gpu_build_sm120_20260810T170339Z_a_build_final.json` | `2ff3da17da7658da35587a6b1233f3bb1c039488325345537c00d40ebf30e9da` |
| `/home/zrj/scout_liquid_lab/audits/u3_source_gpu_build_sm120_20260810T170339Z_a_static_audit_v2.json` | `794ecf80d8665b65ba7e31d1a8bf32a67f8cca7c7cc24ea33718cb30c27c97fb` |

## 4. 阶段 3：GPU 冒烟测试

RTX 5080 上的 C1M 1 s GPU one-shot smoke 已通过：

```text
solver_rc=0
physical_time_seconds=1.0
output_file_count=30
Part_file_count=21
particles=9078
moving_boundary=2669
fluid=6409
Nout=0
network_used=false
status=PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY
```

GPU 冒烟结果只证明程序可以安全、正常地使用 RTX 5080 运行小规模 DualSPHysics，不证明液体已经
稳定，也不证明实物物理保真。

## 5. 阶段 4：液体单独验证

### 5.1 数值稳定性修复与稳态冻结

早期 `Shifting=None + CFL=0.1` 分支未能消除速度回弹，该 FAIL 证据继续保留，但它不是阶段 4
最终状态。后续采用 DualSPHysics 原仓库已有的 artificial-viscosity 参数通路，保持
`Shifting=None`、`CFL=0.1`，将唯一主要数值修复冻结为 `ViscoArtificial=0.3`，从冷启动延长
沉降到 `45.05001991890928 s`。

完成的门禁包括：

1. 冷启动 A/B 与延长沉降的稳态 QC；
2. checkpoint restart 等价性和独立重复性；
3. CPU/GPU backend parity；
4. 从冻结 `Part_0901` 稳态出发的 1 s 零输入回放；
5. 同一稳态出发的 2 mm、1 Hz 平移和 2°、1 Hz 偏航；
6. 逐粒子/逐帧动态 QC、闭合 schema、负测和确定性可视化。

关键中间裁决：

| 裁决 | 状态 | SHA-256 |
| --- | --- | --- |
| 稳态与重复性 | `U3_SETTLED_STATE_FROZEN` | `4703c08e09b33fb16ad68b368dd5e99c2b2c930a3091008fa399241fc58a7fd8` |
| CPU/GPU 一致性 | `PASS_CPU_GPU_PARITY_DEVELOPMENT_ONLY` | `41080d61f749e25db1d54da63d701c26b9e2badc2d89fdebd748997282eb18f1` |

### 5.2 零回放、平移与偏航

三次均为独立、单次 raw run；没有复用或覆盖输出：

| Run | 帧数 | solver rc | elapsed | final receipt SHA-256 |
| --- | ---: | ---: | ---: | --- |
| 零回放 | 21 | 0 | 78.690525 s | `f304c64d4f8babb096efa94aa822346f304800e1011024f5347ab9ee9bd6e0ff` |
| 2 mm、1 Hz 平移 | 61 | 0 | 241.054545 s | `3e3cc024090b8bdeb362aaddcc335b5903e7ef935aac6da480d010c01b8c2df5` |
| 2°、1 Hz 偏航 | 61 | 0 | 238.954193 s | `9fb82f04a8e200dbe3b1cbc58bc0dfa64836f98adaa9ebe6e50aff257b00e082` |

全部 143 帧均保持 9,078 粒子、`Nout=0`、精确 Id 集合和有限数值。动态 QC 关键结果：

| 指标 | 结果 |
| --- | ---: |
| 平移流速响应/零回放 | 18.100341 |
| 偏航流速响应/零回放 | 3.952447 |
| 平移自由衰减末值/初峰 | 0.00116321 |
| 偏航自由衰减末值/初峰 | 0.0269582 |
| 平移/偏航重建频率 | 1.0 / 1.0 Hz |
| 平移最大边界位置误差 | 4.0423e-8 m |
| 偏航最大边界位置误差 | 1.4285e-8 m |
| 偏航最大角度误差 | 4.0423e-5 deg |
| 16-sector 最少粒子（零/平移/偏航） | 233 / 230 / 229（门槛 128） |

偏航对象是轴对称圆柱容器，因此偏航是否生效以刚体运动重建、切向流和动能响应裁决，不要求出现
显著液面倾斜。

最终 QC：

```text
status=PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION
stage4_liquid_only_validation_complete=true
development_only=true
physical_fidelity_validated=false
stage5_admitted=false
receipt=/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_synthetic_motion_20260811T143346Z_v23.qc_v1.json
receipt_sha256=1f837ce7c52ce80971adb981e121eace3c92968095193bb6db3519b39776e383
metrics_sha256=10b8976e8fceff5dbcaa244693afd5ac016734cd10db5696613cb6371183ba25
```

### 5.3 可视化

已输出 6-panel 诊断图，包含边界重建、流速、比动能自由衰减、液面一阶方位谐波和 16-sector
热图。彩色与灰度 PNG 均经过机器布局检查和实际读图复核，曲线同时使用颜色、线型和 marker
编码；图中明确标注 `development-only`。

| 产物 | SHA-256 |
| --- | --- |
| PNG | `ef85f0e799be4cd2298b4b117a2fb514d3cf1f1a8c4841be668df38bf2f428b3` |
| 灰度 PNG | `b936a63a9bb9c7b2e697a7f39f960de7afdf4137ae356143f6e4040c50aacc00` |
| PDF | `2d1f54b60aaab1b1b78f85dfc2937cbe27a9f0d9d6e199ba97bfbbfea3630750` |
| SVG | `d536ce55edbc3c31b30c8dc1039d0f0714b85e5db1e481285bd712f4c185577c` |
| figure receipt | `c0f669726973e98259e307b8c54afdd42c081a1bee993f84eaf34c040e80683d` |

阶段 3–6 的完整边界和后续合同见：
[RTX 5080 GPU 液体仿真阶段 3–6 续接方案](./20260811_RTX5080_DualSPHysics_GPU液体仿真阶段3-6续接方案_Agent执行版.md)。

## 6. 验证与安全边界

最近复核结果：

- synthetic-motion QC 的 AST、closed schema、13/13 单元/负测和 self-check 通过；
- figure 的 AST、closed schema、10/10 单元测试、self-check、严格资产检查和回执 schema 验证通过；
- 最终 QC 和绘图工具均只读消费 raw evidence，没有再次运行 solver、暴露 GPU 或使用网络；
- 冻结方案 SHA-256 未变；
- candidate 仍为 `0400`、105654136 B 和原 SHA-256；
- 最终安全审计未发现 DualSPHysics/NVCC 或 GPU compute 进程残留；
- 原始 roots、日志、receipts 和已有用户改动全部保留，未暂存、未提交、未 push。

当前边界：

```text
GPU_BUILD_PASS=true
GPU_STATIC_AUDIT_PASS=true
GPU_RUNTIME_STATUS=PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY
STAGE4_LIQUID_ONLY_VALIDATION_COMPLETE=true
U3_SETTLED_STATE_FROZEN=true
CPU_GPU_PARITY_PASS=true
SYNTHETIC_MOTION_QC_PASS=true
STAGE5_STARTED=false
STAGE6_STARTED=false
FORMAL=false
PHYSICAL_FIDELITY_VALIDATED=false
```

缺少实测容器几何、液体参数、安装位姿以及传感器标定时，现有结果始终只能作为软件/数值开发证据，
不得表述为实物保真或正式验收结果。

## 7. 下一步

阶段 4 已完成，不需要再重跑。进入阶段 5 前仍需单独 goal 和用户授权，并先补齐：

1. 实测容器内径、液位、液体密度与黏度；
2. 小车质量、质心、惯量、轮地接触和驱动/转向输入；
3. 容器与车体安装位姿、约束方式和运动坐标系；
4. 实验传感器、采样率、时间同步和标定误差；
5. 耦合工况、停止条件、验收指标和 CPU/实物对照方案；
6. 新的 exact run identity、输出根、资源上限和 GPU candidate 执行授权。

在这些输入冻结前，现有结果只能用于数值开发和可视化，不能用于实物一致性声明。

当前唯一正确的 NEXT：

```text
STAGE5_REQUIRES_SEPARATE_USER_AUTHORIZATION_AND_PHYSICAL_INPUTS
```
