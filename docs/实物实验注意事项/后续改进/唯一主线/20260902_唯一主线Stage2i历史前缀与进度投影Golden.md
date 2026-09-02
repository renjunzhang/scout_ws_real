# 唯一主线 Stage 2i：历史前缀与进度投影跨语言 Golden

> 日期：2026-09-02
>
> 分支：`spmpc-mainline`
>
> 父提交：`6ab9211 主线修复：稳定最小容量进度候选选择`
>
> 状态：`SHARED_HISTORY_PREFIX_PROJECTION_GOLDEN_IMPLEMENTED / SYNTHETIC_PARAMETERS_ONLY`

## 1. 目标与实施边界

本子阶段补齐 Stage 2h 留下的第二条 Python/C++ 共用 Golden 链路：

```text
真实 publish receipt
-> 固定容量 emitted history snapshot
-> 非均匀 known-prefix physical propagation
-> pose(T_k^-)
-> 上一拍 nominal progress authority
-> pose-only local progress projection
```

它验证 Stage 2f 与 Stage 2g 之间的数据边界，不把两套专项单元测试简单拼在一起。C++ consumer 必须先运行生产 `KnownPrefixPropagator`，再把本次实际输出的 `physical.pose` 送入生产 `ReferenceProgressProjector`；fixture 不保存第二份 projector pose，也不保存历史 `v_s`、actual velocity 的进度外推或 shifted warm start。

本阶段仍不接 ROS publisher、planning/release coordinator、solver、warm start、CasADi 或 acados，也不生成或冻结任何正式 solver artifact。

## 2. 单一 fixture 与显式 source link

唯一手写场景值位于：

```text
test/fixtures/stage2_fg_execution_projection_golden_v1.json
```

v1 只冻结两个固定 ID 的成功案例：一个非均匀 prefix 和一个 nominal live-release 投影。投影案例通过：

```text
source_known_prefix_case_id
```

显式引用 prefix，不再复制 `pose/clock/reset_epoch/target_cycle/history_generation`。Python reference 先独立重算被引用 prefix，再用计算结果中的 pose 和 generation 计算投影；生成器和 C++ consumer 同样保留并核对 source ID。由此，修改 prefix 物理结果不能靠同步修改一份重复 projector 输入来掩盖。

path identity 与 nominal authority identity 仍分别表示两个真实合同对象，但必须逐字段相等；authority kind、上一拍 cycle、release/history generation、path hash 和 reset epoch 任一漂移都在生成前失败。

## 3. emitted receipt 与 planned 物理时间线

fixture 中 cycle 0..7 均以 `nominal` reason 跨过真实 publish 边界，actual lateness 使用：

```text
0 / 100000 / 200000 / 300000 ns
```

reference 校验 receipt 不早于 release、双时钟严格递增、lateness 不越 publish gate，并让 snapshot 位于最后 receipt 与目标 release 之间。但 prefix 的延迟生效时刻只由 planned `cycle.release_model`、绝对 30 Hz 网格和 `m/beta` 决定。把所有 actual lateness 归零后，六段 target、完整 physical、publisher、older 和 coverage 结果必须完全相同；actual jitter 不成为第二份 delay。

传播区间仍是右连续、目标端开区间：

```text
[t0,T_k)
```

所以等于 `t0` 的 delayed event 立即生效，等于 `T_k` 的 event 不进入 prefix。coverage 从 `t0-max(L_max)` 的真实前驱开始，内部最大 gap 和末端 future hold 都必须不大于冻结门限。

## 4. 独立 Python oracle 与严格 schema

`stage2_fg_reference.py` 不导入生产实现，也不导入 Stage 2h complete-map reference。它独立实现：

- absolute release grid 与整数/分数 delay switch；
- 线角切换时刻并集和 ZOH target；
- 解析 FOPDT actual、中点 pose 与 actual 激励 RK4 liquid；
- publisher/older/coverage 重建；
- 两点路径上的 nominal forward window 与 pose-only projection。

v1 schema 明确只接受当前两个固定案例，避免这个小 oracle 对未实现的 frozen-start、自交/歧义或任意复杂路径作过度承诺；这些边界继续由 Stage 2g 专项 C++ 测试负责。

解析和重算门禁拒绝 duplicate key、未知/缺失字段、`NaN/Inf`、非法 C++ 整数范围、队列维度漂移、expected 漂移、非连续 cycle/generation、receipt 回退、coverage gap、非法 projector config、零 path hash、authority/path/reset/generation/cycle 不一致。

专门的 source-link 变异回归会污染 fixture 中的 prefix expected pose，同时保留独立重算结果不变；投影结果必须仍与基线一致。因此，未来若错误地从 `prefix.expected` 读取 pose，即使当前数值恰好相同也不能通过门禁。

## 5. build-tree 适配与无 ROS C++ consumer

生成器只向调用方指定路径写 C++14 头。CMake 固定把它放在：

```text
${CMAKE_CURRENT_BINARY_DIR}/stage2_fg_execution_projection_golden/
```

生成头包含 schema、canonical JSON SHA-256 和显式 tolerance；history capacity、selector width、older count、segment count 与 path capacity 全部由 fixture 推导为编译期常量，不把 `16/4/2/6` 重新手写进 consumer。该头不进入生产 `include`，不安装，也不允许生产 target 依赖。

独立 consumer 是普通 C++14 executable，不使用 gtest，不链接 `${PROJECT_NAME}`、Catkin 或 ROS。它只包含 ROS-free 的 history/prefix/projector headers 和 build-tree 生成头。CMake 构建依赖先生成头，再无条件执行 `--check`；CTest 另注册 Python oracle、header freshness 和 C++ consumer 三个门禁。

fixture 推导出的 path capacity 是公开合法的最小值 `2`。严格优化编译因此也覆盖该边界：projector 以值语义的最佳候选完成排序、歧义和等价顶点归并，不再用动态 best 下标重复访问最小固定数组。`ReferenceProgressProjector<2>` 专项回归和 Stage 2i consumer 都在 `-O2` 严格告警下编译。

## 6. 验证结果与剩余边界

当前验证覆盖：

```text
Python 3.12 / 3.10 Stage 2i contract       14/14 PASS
authority kind/generation/path/gap mutation  全部 fail closed
生成头 freshness 与严格 C++14 语法          PASS
独立 C++ prefix -> projector consumer       PASS
Stage 2 ROS-free C++ 回归            105/105 PASS
known-prefix / projector / consumer ASan+UBSan  9+12+1 PASS
Ruff、Python 3.8 grammar、git diff --check   PASS
```

当前容器没有 ROS 1 Catkin，因此无法运行 package-level Catkin configure/CTest；已用与 target 相同的 include、生成头和 C++14 源手工编译运行 consumer。进入 ROS 1 环境后仍须补跑完整 Catkin build/CTest。

Stage 2i 只覆盖 nominal live-release 的 prefix-to-projector 成功链。首周期 `FrozenStartProgressAnchor`、投影失败原子性、自交歧义和人工窗口不 clamp 仍由 Stage 2g 的专项测试覆盖，不能在文档中扩大为本 fixture 已有的跨语言覆盖。

Stage 1 仍为：

```text
BLOCKED_PENDING_DEDICATED_IDENTIFICATION_EVIDENCE
```

因此 fixture 中 delay、tau/gain、liquid 和 projector guards 全部是合成测试值，不得用于正式 codegen、实车 preset 或 Stage 3 放行。
