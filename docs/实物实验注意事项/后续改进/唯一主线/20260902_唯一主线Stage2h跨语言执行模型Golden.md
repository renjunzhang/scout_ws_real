# 唯一主线 Stage 2h：Python/C++ 共用执行模型 Golden 合同

> 日期：2026-09-02
>
> 分支：`spmpc-mainline`
>
> 父提交：`bda7c5c 主线重构：拆分物理状态与进度投影合同`
>
> 状态：`FIRST_SHARED_EXECUTION_GOLDEN_IMPLEMENTED / SYNTHETIC_PARAMETERS_ONLY`

## 1. 目标与边界

本子阶段补齐 Stage 2 原合同中“Python/C++ 使用同一 golden vectors”的第一条端到端链路。首批只迁移 Stage 2e 的完整单拍离散 map 场景 `MatchesIndependentCompleteMapGolden`，因为它一次覆盖：

```text
issue map
-> 线/角分数延迟切换时刻并集
-> 三个固定 ZOH target slot
-> 解析 FOPDT actual
-> 中点 pose
-> actual 激励 RK4 liquid
-> progress/publisher/delay queue shift
```

它不生成 solver，不导入 CasADi/acados/ROS，不冻结实车 `L_max/delay/tau/gain`。fixture 中 `dt=1 s` 及执行器数值仍是合成单元测试参数，不能作为 Stage 1 或实车放行证据。

## 2. 单一数值事实源

`MatchesIndependentCompleteMapGolden` 这一完整端到端场景的唯一手写输入和期望输出位于：

```text
test/fixtures/stage2_execution_golden_v1.json
```

Python 测试和 C++ 测试不再分别手写这一场景的 complete-map 数值。其他专项边界/失败测试仍可保留各自最小 synthetic 输入，但不能复制本场景的 expected map。fixture 同时冻结：

- schema version 与 scenario ID；
- config、plant、pre-issue state 和 issue control；
- issued command、三槽 target 与完整 next state；
- 跨实现比较使用的显式绝对容差。

解析器严格拒绝 duplicate key、未知/缺失字段、`NaN/Inf`、非法范围和与 `L_max/dt` 推导维度不一致的 fixed array。selector width、older count 不在生成器中另写常数，而是由 fixture 的 `L_max/dt` 按主合同公式推导。

## 3. 独立参考计算与类型化 C++ 适配

`test/golden/stage2_execution_golden_reference.py` 不导入任何 C++ 或生产 Python 实现，独立重算整张离散 map，并逐叶与 JSON expected 比较。

`test/golden/generate_stage2_execution_golden_header.py` 复用上述严格解析与重算 gate，只负责把同一 JSON 转成测试专用 C++14 类型化头文件。这里的“独立”是指 Python 数值参考不依赖生产实现，而不是声称 header adapter 与 reference 互不依赖。Python reference、header adapter、JSON fixture 和生成头都位于测试边界：源文件不随生产 `scripts/` 安装，头文件只生成到 CMake build tree，包含 schema/scenario/canonical JSON SHA-256，不进入生产 include，也不让生产代码依赖 JSON parser。

CMake 先按 dependency 生成头文件，再无条件执行 `--check` 比较应生成字节与现有 build artifact；因此人工修改较新的生成头也不能绕过 freshness gate。CTest 另注册同一只读检查，避免跳过重新构建而直接测试时漏检 build-tree 漂移。只有 `test_mainline_actuator_discrete_model` 依赖该测试生成目标，其他 Stage 2 测试的模板宽度不受 fixture 控制。

## 4. 当前验证与剩余工作

当前新增 Python 合同覆盖：完整 map 重算、canonical hash/header identity、生成/检查模式、duplicate/unknown/non-finite、fixed array、derived queue width、expected drift 和显式 tolerance。

首批完成后仍有两个明确缺口：

1. Stage 2a/2b/2f 的双时钟、emitted history 和非均匀 prefix，以及 Stage 2g 的局部投影，尚未迁移为共用 JSON fixture；其 C++ 专项测试继续保留。
2. Python reference 与 Stage 3 CasADi 离散图的逐字段 parity 尚未建立；必须等唯一 solver 图实现后复用本 fixture，不得另建第四份数值真值。

因此 Stage 2 保持“进行中”，不能因首条跨语言 golden 通过就宣称全部 Stage 2/Stage 3 完成。
