# 唯一主线 Stage 2c：固定延迟队列与 FOPDT 数值核

> 日期：2026-09-02
>
> 分支：`spmpc-mainline`
>
> 父提交：`c75fcd8 主线测试：补强权威发布历史安全边界`
>
> 状态：`SYNTHETIC_DELAY_AND_FOPDT_KERNEL_IMPLEMENTED / IDENTIFIED_PARAMS_NOT_FROZEN`

## 1. 实施边界

本子阶段实现两项 ROS-free、参数无关的数值基础：

- 固定宽度 `DiscreteDelayQueue` 与整数/分数纯延迟 schedule；
- 常值目标子段上的解析一阶执行器响应与 acceleration。

本阶段不接 ROS、history commit、release coordinator、known-prefix、位姿/液体积分或 generated solver。测试只使用明确标记的合成参数，不把旧候选值写入 production preset。

Stage 1 仍保持：

```text
BLOCKED_PENDING_DEDICATED_IDENTIFICATION_EVIDENCE
```

因此正式 `L_max,v/L_max,omega`、模板宽度、`delay/tau/gain` 和最终 `NP_exec` 仍未冻结，Stage 3 codegen 门禁没有打开。

## 2. 固定队列状态

模板宽度严格对应方案公式：

```text
R  = ceil(L_max / dt)
NQ = R + 1
D  = max(0, R - 1) = max(0, NQ - 2)

Q(0)    = 本边界 final emitted command
Q(1)    = q_prev
Q(r>=2) = older[r-2]
```

`DiscreteDelayQueue<NQ>::kOlderCount` 使用 `NQ-2`，不会把 `q_prev` 在 publisher state 与 older queue 中重复保存。队列只在权威 history 已成功接收 `kPublished` receipt 后调用 `advanceAfterPublished(final_command)`；solver proposal、mailbox ready 或 dry-run 不能调用该接口。这里刻意不复用 Stage 2b 的 `commitEmitted` 名称，避免把裸数值推进误解为 receipt 验证。

`restore(q_prev, older)` 只恢复已经存在的外部历史状态。`resetToConstant()` 只初始化数值对象，不创建 history event，也不能替代 Stage 2b 的真实零命令 warm-up。

唯一例外是 `NQ=1`（`L_max=0`）：该配置没有任何历史 tap，只有本拍 `Q(0)`，因此 `clear/reset` 后即可直接取 tap 和提交；`NQ>=2` 仍必须先恢复权威历史。

## 3. 单通道分数延迟

对 `delay/dt=m+beta`：

```text
slot 0: duration=beta*dt, target=Q(m+1)
slot 1: duration=dt-slot0, target=Q(m)
slot 2: duration=0, target=Q(0)
```

`beta=0` 时 slot 0 为空，slot 1 整拍选择 `Q(m)`。包括零时长 slot 在内，每个 selector 都必须是有限的合法 one-hot；最后有效 duration 由 `dt-前段` 构造。

构造时验证模板 `NQ` 必须等于 `ModelContract::commandSelectorWidth(L_max,dt)`。运行时 delay 只能位于 `[0,L_max]`；负数、NaN、越界或需要不存在 tap 的 schedule 立即失败。队列内部的 integer snap tolerance 是无量纲 ratio；启动工厂接受秒并只做一次 `tolerance_sec/dt` 转换，正式值后续进入 manifest/effective hash。

## 4. 线角切换时刻并集

线、角各自最多有一个拍内切换点。`mergeDelaySchedules()` 对 `{0,t_switch_v,t_switch_omega,dt}` 排序并取精确并集，生成最多三个公共子段：

- 每个子段分别保存线/角 one-hot selector；
- 两通道 delay、整数拍数和 beta 完全独立；
- 不做命令插值，也不把接近但不同的切换时刻静默合并；
- 未使用 slot 仍写合法 `Q(0)` selector 和零 duration；
- 三段时长必须在冻结容差内严格合计为 `dt`。

该固定三槽布局可以直接成为后续 Python/C++/CasADi 共用的 execution parameter schema，但当前尚未生成 artifact。

## 5. FOPDT 子段响应

每个常值 target 子段使用：

```text
rho       = exp(-delta/tau)
one_minus = -expm1(-delta/tau)
actual+   = rho*actual + one_minus*gain*target
accel     = (gain*target-actual)/tau
```

`expm1` 避免极短子段计算 `1-exp(-x)` 的消减误差。`delta=0` 严格返回原状态。delay 只决定 target schedule，不在一阶响应函数中再补第二份延迟。

参数校验要求：

```text
delay_sec >= 0
tau_sec   > 0
gain      > 0
全部有限
```

这只是通用数学合同；没有冻结任何实车数值。

## 6. 合成 golden fixture

单元测试统一使用：

```text
dt = 1/30 s
synthetic L_max = 3*dt
R=3, D=2, NQ=4
Q=[40,30,20,10]
```

覆盖 `delay=0/1/2/3 dt`、`0.5/1.5/2.5 dt`、integer snap、上界与越界、`NQ=1/2/4`、队列 shift、线角不同切换时刻，以及 FOPDT 零输入、阶跃、常值、换向、tau/gain 敏感性、分段组合和异常输入。线角切换点差异小于 duration tolerance、但未命中 integer snap 的用例仍必须保留独立短子段。

这些 golden vectors 只验证公式与离散顺序。硬件 onset、fidelity、正式 L_max 与参数选择仍必须由专用 development/validation/final-test bag 完成。
