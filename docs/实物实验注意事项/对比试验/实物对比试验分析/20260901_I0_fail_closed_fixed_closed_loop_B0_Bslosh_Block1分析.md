# 20260901 I0 fail-closed + fixed_closed_loop Bslosh Block 1 分析

> 日期：2026-09-01
>
> 性质：development 快速筛查，不作为正式效果声明
>
> 协议：历史 v1 `SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1`（全约 2 s horizon 计入液体代价）与独立 v2 `SMPCC_I0_FAILCLOSED_FIXED_SHORT100_ABBA_DEV_V2`（仅前 100 ms 液体代价）
>
> 对应方案：[20260901_I0_fail_closed_fixed_closed_loop_B0_Bslosh_ABBA验证方案.md](../实物对比实验/20260901_I0_fail_closed_fixed_closed_loop_B0_Bslosh_ABBA验证方案.md)

## 1. 结论

两轮 Block 1 都没有通过：

1. 历史 v1 冻结的 `processed-IMU I0 + fail_closed + legacy fixed_closed_loop/L22 + B_slosh(w=5)` 组合没有降低实物晃动，反而明显差于本轮 B0。
2. v2 将液体代价缩短到前 100 ms 后，`B_slosh_short100` 仍明显差于本轮 B0，并产生了肉眼可见的“一卡一卡”。数据表明该顿挫来自 solver 自身反复规划加速和减速，不是命令丢帧、发布链限幅或安全门停车。

两轮都已按各自预设规则判定 `STOP_BLOCK1_FUTILITY`，均不继续 Row03/Row04。

这只否定当前两个 legacy 组合，不否定 processed IMU、液体状态反馈或所有延迟补偿方案。v1 与 v2 是独立协议、独立配对，结果不能合并统计，也不能据此直接比较 full-horizon 和 short100 谁更好。

## 2. 历史 v1 有效数据

原 Block 1 配对：

```text
/home/geist/slosh_bags/real/20260901_spmpc_i0_failclosed_fixed_abba/H0/
  DEV_I0FC_FIXED_01_B0_b01_p01_a01.bag
  DEV_I0FC_FIXED_02_Bslosh_b01_p02_a01.bag
```

独立 B0 复测：

```text
/home/geist/slosh_bags/real/20260901_spmpc_i0fc_fixed_b0_recheck/H0/
  DEV_I0FC_FIXED_01_B0_b01_p01_a01.bag
```

独立复测只用于确认 B0 重复性，不替换原 ABBA Row01。

## 3. 历史 v1 预注册 RGB 主结果

统计窗口为运动开始至首次到点后 5 秒。

| 运行 | RGB P95 | RGB RMS | 到点时间 |
|---|---:|---:|---:|
| B0 Row01 | 1.440 mm | 0.741 mm | 28.462 s |
| Bslosh Row02 | 7.081 mm | 3.890 mm | 28.924 s |

以 `B0 - Bslosh` 定义改善量：

```text
P95 改善量 = -5.641 mm
RMS 改善量 = -3.149 mm
Bslosh/B0 到点时间比 = 1.016
```

Bslosh 的 RGB 晃动显著增大，而到点时间只增加约 1.6%，因此不能用明显减速或运行时间差解释结果。

## 4. 历史 v1 路径进度 10%～90% 对比

冻结路径长度为 `5.4284537 m`。去掉起步和终点段后，结果仍保持同一方向。

| 运行 | 区间时长 | RGB P95 / Peak / RMS | 内部模态 P95 / Peak / RMS |
|---|---:|---:|---:|
| B0 Row01 | 23.265 s | 1.363 / 2.508 / 0.760 mm | 1.492 / 3.256 / 0.744 mm |
| Bslosh Row02 | 23.805 s | 7.042 / 8.904 / 4.096 mm | 3.637 / 5.068 / 1.860 mm |
| B0 独立复测 | 23.230 s | 1.429 / 2.614 / 0.664 mm | 1.446 / 3.906 / 0.689 mm |

相对原 B0，Bslosh 的内部模态 P95、Peak、RMS 分别增大到约 `2.44`、`1.56`、`2.50` 倍。两次 B0 的 RGB 和内部模态统计接近，说明低晃动 B0 具有重复性。

同一区间内，B0/Bslosh 的横向误差 P95 为 `0.0216/0.0185 m`，航向误差 P95 为 `0.0527/0.0537 rad`。两者跟踪精度相近，Bslosh 的高晃动不是明显路径跟踪失控造成的。

## 5. 历史 v1 Solver 输入审计与模型偏差

Bslosh 使用 10D solver，最终液体输入为 I0 经 legacy 命令历史前推后的 L22。在 10%～90% 区间内，714 个样本逐项满足：

```text
predicted_state.h_modal_mm
= solver_input_state.h_modal_mm
= /spmpc/slosh_height
```

因此表中的 Bslosh 内部模态高度确实对应送入 solver 的液体状态。当前 `use_parabola_term=false`，所以这三个量可以直接对应。

B0 使用 6D solver，不消费液体状态；其内部模态高度只是同链路旁路诊断基线，不能表述为 B0 solver 的液体输入。

Bslosh 区间内内部模态 P95 为 `3.637 mm`，RGB P95 为 `7.042 mm`。模型判断出了晃动变大的方向，但只反映 RGB 幅值的约 `52%`，低估约 `48%`。

## 6. short100 v2 Block 1 结果

### 6.1 有效数据与冻结合同

```text
/home/geist/slosh_bags/real/20260901_spmpc_i0_failclosed_fixed_short100_abba_v2/H0/
  DEV_I0FC_FIXED_S100_V2_01_B0_b01_p01_a01.bag
  DEV_I0FC_FIXED_S100_V2_02_BsloshS100_b01_p02_a01.bag
```

两包 postflight 均为 `PASS`，话题齐全、到达终点、RGB 有效率和来源率均为 `1.0`。Row02 的液体代价合同为：前 `3` 个控制步、共 `100 ms`，尾部液体代价为 `0`；机器人预测仍保留完整 `60` 步时域。因此这是有效的 short100 对照，不是错误地把整个预测时域裁短。

### 6.2 预注册主结果

统计窗口为运动开始至首次到点后 5 秒。

| 指标 | B0 Row01 | Bslosh-short100 Row02 |
|---|---:|---:|
| RGB P95 | 1.357 mm | 7.663 mm |
| RGB RMS | 0.640 mm | 4.045 mm |
| 到点时间 | 28.432 s | 30.759 s |
| 横向误差 P95 | 0.01786 m | 0.01301 m |
| 航向误差 P95 | 0.04283 rad | 0.05471 rad |
| solver 计算时间 P95 | 2.638 ms | 13.283 ms |

以 `B0 - Bslosh-short100` 定义改善量：

```text
P95 改善量 = -6.305 mm
RMS 改善量 = -3.404 mm
Bslosh-short100/B0 到点时间比 = 1.082
```

short100 的 RGB 晃动明显增大，到点时间增加约 `8.2%`。横向误差没有恶化，航向误差仅小幅增加，不能把约 5～6 倍的 RGB 指标差异简单归因于路径跟踪失控。

### 6.3 “一卡一卡”的命令连续性证据

以下只统计冻结路径进度 `10%～90%`，排除起步等待和终点停车。`Δv` 表示相邻两个已发布线速度指令之差。

| 指标 | B0 Row01 | Bslosh-short100 Row02 |
|---|---:|---:|
| 命令发布频率 | 30.0 Hz | 30.0 Hz |
| 已发布速度标准差 | 0.00186 m/s | 0.03584 m/s |
| `|Δv|` P95 | 0.000218 m/s | 0.0200 m/s |
| 单周期最大 `|Δv|` | 0.000367 m/s | 0.03884 m/s |
| `|Δv| >= 0.015 m/s` 的周期数 | 0 | 211 |
| `v < 0.15 m/s` 的样本占比 | 0 | 16.4% |
| 低速段数量 / 最长持续 | 0 | 21 段 / 24 周期（约 0.8 s） |
| solver 首步 `|a|` P95 | 0.00653 m/s² | 0.600 m/s² |
| solver 首步加速度饱和占比 | 0 | 31.4% |
| 强正负加速度翻转次数 | 0 | 76 |
| 相邻重规划首步加速度变化 P95 | 0.0477 m/s² | 0.5968 m/s² |
| 相邻重规划首步加速度最大变化 | 0.0587 m/s² | 1.1146 m/s² |

short100 并非真正周期性停到零，而是在运动中持续给出明显的减速—加速脉冲。其首步加速度频繁触及约 `-0.6/+0.6 m/s²`，并在相邻重规划间快速反向，因而实车表现为速度忽快忽慢、走得不连续。相比之下，B0 在同一路段的速度和加速度指令基本连续。

### 6.4 卡顿位置判定与解释边界

可以由 bag 直接确认：

1. `published_cmd` 与 `solver_cmd` 逐样本一致，卡顿指令已经由 solver 直接给出。
2. 两包均约 30 Hz，无命令丢帧；Row02 的 solver P95 为 `13.283 ms`，低于 `33.3 ms` 控制周期，不是求解超时导致的断续。
3. 运动段没有 command limiter、线速度/角速度限幅、安全跟踪门或速度安全门介入，也没有 solver failure 触发零速。
4. 因此现场观察到的“一卡一卡”是真实且可复现的，主要属于优化器生成的原始速度/加速度规划振荡，而不是 ROS 发布中断或底盘外部保护对平滑命令的二次破坏。

目前最合理的机制嫌疑是：液体代价只看未来 100 ms，优化器反复追逐短时液体状态，在相邻求解周期频繁改变加减速选择。但这只是与数据一致的解释，当前两包还不能证明它是唯一根因；执行器延迟、液体状态共同时间基准和模型偏差仍需在下一阶段显式处理。

## 7. 阶段判断

1. v1 的失败方向同时出现在 RGB 实测和内部模态状态中，不是单一视觉异常。
2. v2 short100 仍未改善 RGB 晃动，并额外出现 solver 内生的速度/加速度振荡。
3. 两轮 Block 1 都已经给出有效负向结果，不运行各自 Row03/Row04，不选择性重录 Row02，也不再沿 legacy L22 继续做液体权重微调。
4. 下一步按冻结路线转入“共同状态时刻 + 显式执行器延迟模型”，先修正状态和动作的时间关系，再重新验证液体代价。

冻结自动决策文件：

```text
/home/geist/slosh_bags/real/20260901_spmpc_i0_failclosed_fixed_abba/H0/
I0_FAILCLOSED_FIXED_ABBA_RGB_ANALYSIS.json

/home/geist/slosh_bags/real/20260901_spmpc_i0_failclosed_fixed_short100_abba_v2/H0/
I0_FAILCLOSED_FIXED_SHORT100_ABBA_RGB_ANALYSIS.json
```
