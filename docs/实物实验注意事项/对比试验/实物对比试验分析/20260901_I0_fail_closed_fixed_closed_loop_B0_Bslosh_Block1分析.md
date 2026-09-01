# 20260901 I0 fail-closed + fixed_closed_loop 历史 Bslosh v1 Block 1 分析

> 日期：2026-09-01
>
> 性质：development 快速筛查，不作为正式效果声明
>
> 协议：`SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1`（全约 2 s horizon 计入液体代价）
>
> 对应方案：[20260901_I0_fail_closed_fixed_closed_loop_B0_Bslosh_ABBA验证方案.md](../实物对比实验/20260901_I0_fail_closed_fixed_closed_loop_B0_Bslosh_ABBA验证方案.md)

## 1. 结论

历史 v1 冻结的 `processed-IMU I0 + fail_closed + legacy fixed_closed_loop/L22 + B_slosh(w=5)` 组合没有降低实物晃动，反而明显差于 B0。Block 1 已按预设规则判定 `STOP_BLOCK1_FUTILITY`，不继续 v1 Row03/Row04。

这只否定当前 legacy 组合，不否定 processed IMU、液体状态反馈或所有延迟补偿方案。

## 2. 有效数据

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

## 3. 预注册 RGB 主结果

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

## 4. 路径进度 10%～90% 对比

冻结路径长度为 `5.4284537 m`。去掉起步和终点段后，结果仍保持同一方向。

| 运行 | 区间时长 | RGB P95 / Peak / RMS | 内部模态 P95 / Peak / RMS |
|---|---:|---:|---:|
| B0 Row01 | 23.265 s | 1.363 / 2.508 / 0.760 mm | 1.492 / 3.256 / 0.744 mm |
| Bslosh Row02 | 23.805 s | 7.042 / 8.904 / 4.096 mm | 3.637 / 5.068 / 1.860 mm |
| B0 独立复测 | 23.230 s | 1.429 / 2.614 / 0.664 mm | 1.446 / 3.906 / 0.689 mm |

相对原 B0，Bslosh 的内部模态 P95、Peak、RMS 分别增大到约 `2.44`、`1.56`、`2.50` 倍。两次 B0 的 RGB 和内部模态统计接近，说明低晃动 B0 具有重复性。

同一区间内，B0/Bslosh 的横向误差 P95 为 `0.0216/0.0185 m`，航向误差 P95 为 `0.0527/0.0537 rad`。两者跟踪精度相近，Bslosh 的高晃动不是明显路径跟踪失控造成的。

## 5. Solver 输入审计与模型偏差

Bslosh 使用 10D solver，最终液体输入为 I0 经 legacy 命令历史前推后的 L22。在 10%～90% 区间内，714 个样本逐项满足：

```text
predicted_state.h_modal_mm
= solver_input_state.h_modal_mm
= /spmpc/slosh_height
```

因此表中的 Bslosh 内部模态高度确实对应送入 solver 的液体状态。当前 `use_parabola_term=false`，所以这三个量可以直接对应。

B0 使用 6D solver，不消费液体状态；其内部模态高度只是同链路旁路诊断基线，不能表述为 B0 solver 的液体输入。

Bslosh 区间内内部模态 P95 为 `3.637 mm`，RGB P95 为 `7.042 mm`。模型判断出了晃动变大的方向，但只反映 RGB 幅值的约 `52%`，低估约 `48%`。

## 6. 阶段判断

1. 失败方向同时出现在 RGB 实测和内部模态状态中，不是单一视觉异常。
2. 结果不由终点尾段、明显减速或跟踪失控主导。
3. 当前 legacy L22 与全 horizon Bslosh 组合不值得继续完成 v1 ABBA，也不应选择性重录或只做权重微调。
4. 下一步先按独立 v2 协议筛查 `B_slosh_short100`；若仍为有效负向，再按冻结路线转入共同状态时刻和显式执行器延迟模型。

冻结自动决策文件：

```text
/home/geist/slosh_bags/real/20260901_spmpc_i0_failclosed_fixed_abba/H0/
I0_FAILCLOSED_FIXED_ABBA_RGB_ANALYSIS.json
```
