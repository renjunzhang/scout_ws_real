# 20260903 I0 + 显式执行器 OCP 的 B0/Bslosh Block 1 分析

> 日期：2026-09-03
>
> 性质：development 实物 Block 1 诊断，不作为正式降晃效果声明
>
> 协议：`SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_ABBA_DEV_V1`
>
> 文档写入基线：`diag/lt-dwa-collision-tracking @ a2eacb9f30e14b7439714a558b00696481054e85`，写入前工作区 clean
>
> 对应方案：[20260903_I0显式执行器OCP_B0_Bslosh_ABBA验证方案.md](../实物对比实验/20260903_I0显式执行器OCP_B0_Bslosh_ABBA验证方案.md)

## 1. 结论

本轮两个 bag 已足够定位 `B_slosh` 的实车卡顿，但不足以判断液体代价是否降低了实物晃动：

1. Row01 `B0` 完成路径且主合同、processed-IMU、NOKOV 和 RGB 后验均为 `PASS`，可保留为当前显式执行器主线的有效基线。
2. Row02 `B_slosh` 虽然到达终点，但运动窗内出现 `47` 拍共同状态时刻插值失败和 `10` 拍 acados `MINSTEP`。这 `57` 拍全部触发 `fail_closed` 零速，因此主后验为真实 `FAIL`。
3. 卡顿的首要原因是 `B_slosh` 控制回调接近或超过 30 Hz 的 `33.3 ms` 周期，阻塞了同一单线程回调队列中的 odom；`queue_size=1` 随后丢掉中间 odom，planner 内部形成 `55～80 ms` 的状态空洞，超过共同时间插值上限 `50 ms`。
4. 这不是 NOKOV 刚体或原始 odom 断流。Row02 原始 `/odom` 在运动窗内最大间隔约 `24.9 ms`、没有超过 `50 ms` 的空洞；空洞只出现在 planner 已处理的 odom 历史中。
5. 除硬停车外，`B_slosh` 正常求解拍本身也频繁给出正负饱和加速度。完整时域 `w_slosh=5`、较弱的控制变化代价与该现象一致，是剩余速度起伏的第二层原因，但当前证据不能把权重认定为唯一根因。

因此停止当前 Block 1，不运行 Row03/Row04。Row02 应保留为运行时故障证据，不能进入 RGB 改善量或正式 B0/Bslosh 效果统计。下一步先修复实时调度和求解稳定性，再录一个 `B_slosh` smoke；通过运行时门后才重新做配对实验。

## 2. 版本、配置与数据

显式执行器 runtime 主实现提交：

```text
5993ffaa01e3e3489405d8edb5277338d5ada5ce
SPMPC：接入显式执行器 OCP 并整理实验文档
```

Row01 采集于 `5993ffa`。Row01 后验误把“L22 应为 OFF”当成“L22 有效覆盖率必须为 99%”，后由以下提交修正：

```text
a74a62303bab022b6cb4f541f77571797ce26249
实验：修正显式执行器 ABBA 后验契约
```

Row02 采集于 `a74a623`。该提交只修改 runner、后验和测试，没有修改 planner 动力学或 acados runtime；两包实际控制模型保持一致。仍保留两次采集 revision 的区别，不把它们写成完全相同的仓库快照。

数据目录：

```text
/home/geist/slosh_bags/real/
20260903_spmpc_i0_failclosed_explicit_actuator_abba_v1/H0/
```

本轮使用：

```text
DEV_I0FC_EXPACT_V1_01_B0_b01_p01_a01.bag
DEV_I0FC_EXPACT_V1_02_Bslosh_b01_p02_a01.bag
```

两包冻结条件：

| 项目 | B0 | Bslosh |
|---|---:|---:|
| variant | `B0` | `B_slosh` |
| solver state | 23D | 27D |
| observer | processed-IMU 旁路健康证据 | processed-IMU I0，solver 消费 |
| fallback | `fail_closed` 合同 | `fail_closed` |
| common epoch | 配置开启，液体不消费 | 开启且必须逐拍满足 |
| execution model | `explicit_actuator` | `explicit_actuator` |
| legacy L22 | `off`、未应用 | `off`、未应用 |
| `v_ref / v_safe_max` | `0.20 / 0.25 m/s` | 同左 |
| horizon | `N=60, dt约33.3 ms` | 同左 |
| liquid cost | 关闭 | 完整时域，`w_slosh=5` |

执行器参数均为：

```text
linear:  L=0.1666666665 s, tau=0.112 s, K=1.018, FIFO=5
angular: L=0.3333333330 s, tau=0.119 s, K=1.096, FIFO=10
```

## 3. 有效性与可用范围

| 检查项 | B0 Row01 | Bslosh Row02 |
|---|---:|---:|
| 到达终点 | 是 | 是 |
| 主合同后验 | `PASS` | `FAIL` |
| processed-IMU 来源/READY | 通过 | 通过 |
| fallback 或 reset | 无 | 无 |
| actuator state 有效率 | `1.0` | `1.0` |
| legacy L22 application | `0` | `0` |
| common-epoch 失败 | `0` | `47` |
| solver failure | `0` | `10` |
| 安全/限速门改写 | 无 | 无 |

Row01 的 RGB 后验主指标为：

```text
H_vis P95 = 1.3681 mm
H_vis Peak = 2.0604 mm
H_vis RMS = 0.6708 mm
到点时间 = 29.619 s
```

Row02 主运行时合同未通过，因此不继续生成或解释预注册 RGB 改善量。它可以用于定位卡顿机制，不能作为有效的 `B_slosh` 效果样本。两包 observer 状态的直接高低也受到 Row02 多次强制停车污染，不能解释成液体代价本身的净效果。

## 4. 卡顿的直接证据

统计窗口均从首次有效求解到首次 `GOAL_REACHED`。控制回调耗时为 `cycle_start_stamp` 到 `command_publish_stamp`；planner odom 使用 `/spmpc/debug/slosh_observer_odom` 的已处理样本。

| 指标 | B0 | Bslosh |
|---|---:|---:|
| 运动窗 | 29.62 s | 37.60 s |
| 有效求解拍 | 890 | 1068 |
| solver 平均耗时 | 4.49 ms | 22.85 ms |
| solver P95 | 7.26 ms | 36.26 ms |
| 完整控制回调 P95 | 11.77 ms | 42.41 ms |
| 回调超过 33.3 ms | 0 | 187（17.5%） |
| 原始 odom 最大间隔 | 28.80 ms | 24.91 ms |
| planner odom 最大间隔 | 40.02 ms | 79.98 ms |
| planner odom 间隔 >50 ms | 0 | 25 处 |
| common-epoch 失败拍 | 0 | 47 |
| `ACADOS_SOLVE_FAILED_4` | 0 | 10 |

Row02 的 `47/47` 个共同时间失败都能落到 planner odom 的 `55.1～80.0 ms` 插值空洞中。一处空洞可被连续两个控制周期命中，所以 `25` 处 odom 空洞产生了 `47` 个失败拍。

当前回调结构为：

```text
默认单线程 ros::spin
  ├─ odom callback，subscriber queue_size=1
  └─ 30 Hz control timer + acados solve

独立 AsyncSpinner
  └─ processed-IMU callback
```

`B_slosh` 求解或前处理占住默认回调线程时，IMU 状态仍在独立线程继续更新，而 odom 的中间消息会因队列长度为 1 被覆盖。控制线程恢复后，共同状态时刻落在稀疏 odom 的两个样本之间，`max_interpolation_gap_sec=0.05` 正确拒绝插值，然后发布零速：

```text
Bslosh 回调超期
  -> planner odom 丢中间样本
  -> common epoch 的 odom 括号间隔 > 50 ms
  -> STATE_TIME_ALIGNMENT_FAILED_INTERPOLATION_GAP
  -> fail_closed 发布 (0, 0)
  -> 下一次求解从低 command 状态恢复
  -> 实车停走
```

诊断字段 `zero_due_to_waiting_for_tf` 是 `robotStateAtEpoch()` 失败分支的共用分类；本包的具体状态是 `INTERPOLATION_GAP`，不能据此写成 NOKOV 或 TF 数据源断开。

## 5. 零速对实车的影响

将相邻异常拍合并后，Row02 共形成 `30` 段故障性零速：

```text
累计零速保持时间                 2.263 s
单段中位保持时间                 0.074 s
单段最大保持时间                 0.155 s
恢复后首条有效 v_cmd <=0.023 m/s 27/30 段
故障前 v_cmd >0.10 m/s           16 段
其后 0.6 s 内 odom v <0.05 m/s   15/16 段
```

因此“卡顿”不是只损失 `57 × 1/30 s` 的抽象控制拍。零命令进入执行器和命令历史后，多数故障会把下一次有效命令拉回约 `0.02 m/s`，再经历重新加速，肉眼表现为明显停车再起步。

10 个 `ACADOS_MINSTEP` 全部出现在最近一次共同时间失败后的约 `0.04～0.35 s` 内，且集中在低速恢复段。这支持它们是故障恢复链上的第二层问题；但当前 bag 只能证明时间聚集，尚不能证明其唯一数值根因。

## 6. 正常求解拍仍存在主动速度起伏

为排除强制停车后的恢复影响，另统计“距离最近故障至少 1 s”的连续有效求解拍：

| 指标 | B0 | Bslosh |
|---|---:|---:|
| 样本数 | 890 | 624 |
| `|a_0| > 0.59 m/s²` | 11.5% | 67.9% |
| `a_0 < -0.3 m/s²` | 0 | 36.4% |
| 相邻强正负加速度翻转 | 0 | 127 |

Row02 的液体代价占比中位数约 `40.8%`，P95 约 `91.4%`。结合 `w_slosh=5`、`w_smooth/w_du_a=0.1` 和完整约 2 s 液体代价窗口，当前证据支持：即使消除 fail-closed 硬停车，优化器仍可能持续使用接近正负上限的 command 加速度抵消预测晃动，造成速度锯齿。

这是第二层问题，不能用调权重替代前面的实时调度修复。应先使每拍状态输入和求解有效，再判断需要多大的完整时域 `Delta a_cmd/Delta alpha_cmd` 或 jerk 平滑。

## 7. 已排除或尚未证明

已排除：

- 不是原始 `/odom` 或 NOKOV 断流；原始流的时间间隔正常。
- Row02 原始 NOKOV `Tracker0` pose 的 header 最大间隔约 `30.05 ms`，没有超过 `50 ms` 的空洞。
- 不是 legacy L22 双重补偿；两包均为 `off` 且 application 为 0。
- 不是 processed-IMU fallback/reset；I0 来源、READY 和 reset epoch 均正常。
- 不是发布后 limiter、安全跟踪门或速度安全门改写。
- planner 日志中的 `LiquidSloshModel dt too small` 不是本次特异原因；B0 有 `31` 条而 Bslosh 只有 `15` 条。该告警仍应清理，但不能解释 Row02 独有的卡顿。

尚未证明：

- 固定 `L/tau/K` 是否已足够准确地覆盖本次全部运动幅值。
- 修复回调调度后，10 个 `MINSTEP` 是否会自然消失。
- 去除故障拍后，当前完整时域液体代价能否降低 RGB 晃动。
- `w_slosh`、液体代价窗口或平滑项各自对主动速度翻转的独立贡献。

因此，本轮不否定 command/actual 分离和显式执行器 OCP 的结构方向；它否定的是当前 `B_slosh` runtime 已经可以直接进入效果比较这一判断。

## 8. 最小修复与验收顺序

1. 将 odom 接收从求解 timer 的单线程回调队列中解耦，使用独立 callback queue/spinner，并为 `last_odom`、odom history 和相关快照补齐同步保护。
2. 降低显式执行器 prefix rollout 的前处理开销。当前 10 ms 子步会反复重配液体离散模型；应缓存固定子步矩阵，并避免运行期重复 INFO 输出。
3. 保持 `max_interpolation_gap_sec=0.05`。不通过放宽门限掩盖 planner 内部丢样本。
4. 先只录一包低风险 `B_slosh` smoke，要求运动窗内：
   - common-epoch failure 为 0；
   - solver failure 为 0；
   - 无故障性零速；
   - planner odom 不再出现超过 50 ms 的内部空洞；
   - 控制回调 P95 低于 33.3 ms，超周期不形成连续积压。
5. 运行时门通过后，再检查无故障区间的加速度饱和和正负翻转。若仍明显，再加入完整时域 `Delta a_cmd/Delta alpha_cmd` 或 jerk 平滑，并重新选择至少覆盖执行器起效时间的液体代价窗口。
6. 只有新的 `B_slosh` smoke 同时通过运行时和命令连续性门，才重新开始 B0/Bslosh 配对；不复用本次 Row02 计算效果量。

## 9. 权威证据

主后验：

```text
/home/geist/slosh_bags/real/20260903_spmpc_i0_failclosed_explicit_actuator_abba_v1/H0/
DEV_I0FC_EXPACT_V1_01_B0_b01_p01_a01_i0_explicit_actuator_v1_postflight.json

/home/geist/slosh_bags/real/20260903_spmpc_i0_failclosed_explicit_actuator_abba_v1/H0/
DEV_I0FC_EXPACT_V1_02_Bslosh_b01_p02_a01_i0_explicit_actuator_v1_postflight.json
```

B0 的独立 observer、NOKOV 和 RGB 后验：

```text
DEV_I0FC_EXPACT_V1_01_B0_b01_p01_a01_explicit_actuator_v1_observer_postflight.json
DEV_I0FC_EXPACT_V1_01_B0_b01_p01_a01_explicit_actuator_v1_mocap_chain_postflight.json
DEV_I0FC_EXPACT_V1_01_B0_b01_p01_a01_i0_explicit_actuator_v1_rgb_postflight.json
```

相关实施边界见 [I0 与显式执行器 OCP 最小修复方案](../解决问题的思路/20260903_I0与显式执行器OCP最小修复方案.md)。
