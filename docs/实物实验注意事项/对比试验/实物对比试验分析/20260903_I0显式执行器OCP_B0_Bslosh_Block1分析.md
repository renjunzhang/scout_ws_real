# 20260903 I0 + 显式执行器 OCP 的 Block 1 及后续 smoke 分析

> 日期：2026-09-03
>
> 性质：development 实物诊断，不作为正式降晃效果声明
>
> 协议：`SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_ABBA_DEV_V1`、`SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_RUNTIME_SMOKE_DEV_V1`、`SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_WACCEL03_SMOKE_DEV_V1`
>
> 初始写入基线：`diag/lt-dwa-collision-tracking @ a2eacb9f30e14b7439714a558b00696481054e85`
>
> 本次续写基线：`diag/lt-dwa-collision-tracking @ fee6881eb1bfc689912c8ad841988f9b1de15e12`，续写前工作区 clean
>
> 对应方案：[20260903_I0显式执行器OCP_B0_Bslosh_ABBA验证方案.md](../实物对比实验/20260903_I0显式执行器OCP_B0_Bslosh_ABBA验证方案.md)

## 1. 结论

Block 1 和后续两包 smoke 已把“硬停车”和“MPC 主动锯齿”分成两层：

1. Row01 `B0` 完成路径且主合同、processed-IMU、NOKOV 和 RGB 后验均为 `PASS`，可保留为当前显式执行器主线的有效基线。
2. Row02 `B_slosh` 虽然到达终点，但运动窗内出现 `47` 拍共同状态时刻插值失败和 `10` 拍 acados `MINSTEP`。这 `57` 拍全部触发 `fail_closed` 零速，因此主后验为真实 `FAIL`。
3. Row02 的硬停车来自控制求解阻塞默认 callback queue 后造成的 planner 内部 odom 空洞，不是 NOKOV 刚体、原始 `/odom` 断流或 legacy L22 双重补偿。
4. 提交 `be8f1ef` 解耦 odom callback、缓存 prefix 离散模型并冻结 `qp_solver_cond_N=10` 后，`190310` runtime smoke 的 common-epoch failure、solver failure 和故障零速均为 `0`，完整控制回调 P95 为 `16.529 ms`。因此第一层运行时硬停车已由实物 smoke 验证消除。
5. 只把 `w_accel` 从 `0.0` 提高到 `0.3` 后，`192948` smoke 的运行时仍为 `PASS`，有效求解拍的线加速度饱和占比由约 `73.8%` 降至 `56.4%`；说明全时域加速度幅值代价有效，但没有解决主动翻转。
6. 转弯有效拍的饱和占比仍约 `59.3%`，强正负翻转只由 `60` 次降至 `53` 次，主频仍约 `5.13 Hz`，接近液体固有频率 `4.97 Hz`。转弯 slosh 代价占比中位数仍约 `87.0%`、P95 约 `90.9%`。
7. 当前证据支持：`w_slosh=5 + 2 s 完整液体时域` 相对其他代价仍过强，优化器主动规划了接近液体模态频率的加减速。下一包保持 `w_accel=0.3`，只把 `w_slosh` 降至 `1.0`；这是一项待验证的 development 调参，不是已证明的降晃修复。

Block 1 仍停在 Row02，不补跑 Row03/Row04。Row02 只保留为历史运行时故障证据；两包后续 smoke 均关闭 RGB，只能支持运行时和命令连续性判断，不能进入真实液面改善量或正式 B0/Bslosh 效果统计。

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
- Block 1 的 `10` 个 `MINSTEP` 是否全部由同一恢复链造成；`190310/192948` 两包均为 `0`，只能证明它们在这两次修复后运行中没有复现。
- 去除故障拍后，当前完整时域液体代价能否降低 RGB 晃动。
- `w_slosh`、液体代价窗口或平滑项各自对主动速度翻转的独立贡献。

因此，Block 1 不否定 command/actual 分离和显式执行器 OCP 的结构方向；它当时否定的是“该 revision 的 `B_slosh` runtime 已可直接进入效果比较”。后续 smoke 已解除运行时阻塞，但命令连续性仍未达到重新开展 RGB 效果比较的条件。

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
5. 运行时门通过后，先以 `w_accel=0.3` 检查无故障区间的加速度饱和和正负翻转；该步现已完成，结果为部分改善。
6. 因转弯 slosh 代价仍占约 `87%` 且约 `5 Hz` 翻转保留，下一包只把 `w_slosh` 降至 `1.0`。若仍明显翻转，再加入完整时域 `Delta a_cmd/Delta alpha_cmd` 或 jerk 平滑。
7. 只有新的 `B_slosh` smoke 同时通过运行时和命令连续性门，才重新开始带 RGB 的 B0/Bslosh 配对；不复用本次 Row02 计算效果量。

## 9. 运行时修复后的 Bslosh smoke

运行时修复由提交 `be8f1ef` 实现，单包验收入口由 `854e2cf` 固化。实物包采集 revision 为：

```text
854e2cf8a4b89becccb76358c0d307ca3627890e
```

数据：

```text
/home/geist/slosh_bags/real/
20260903_spmpc_i0_failclosed_explicit_actuator_runtime_smoke_v1/H0/
DEV_I0FC_EXPACT_RUNTIME_SMOKE_V1_190310_Bslosh.bag
```

冻结配置仍为 `processed-IMU I0 + fail_closed + common_epoch + explicit_actuator`，legacy L22 关闭；`w_slosh=5.0`、`w_accel=0.0`、`v_ref=0.20 m/s`。两层 postflight 均为 `PASS`：

| 指标 | 结果 |
|---|---:|
| common-epoch failure | 0 |
| solver failure | 0 |
| 故障性零速 | 0 |
| planner odom 最大间隔 | 20.167 ms |
| planner odom 间隔 >50 ms | 0 |
| 完整控制回调 P95 / max | 16.529 / 29.884 ms |
| 回调 >33.3 ms | 0 |

这验证了第一层运行时修复在该次低速 C02 实物运行中有效。它不证明所有负载和速度下都不会再次超期，但已经排除了本包抽搐来自 common-epoch/fail-closed 硬停车。

## 10. `w_accel=0.3` 单变量 smoke

参数入口由 `b457be8` 增加，本包采集 revision 为：

```text
b457be82ae18848fa9f9d348c29df1bfa4054585
```

数据：

```text
/home/geist/slosh_bags/real/
20260903_spmpc_i0_failclosed_explicit_actuator_waccel03_smoke_v1/H0/
DEV_I0FC_EXPACT_WACCEL03_SMOKE_V1_192948_Bslosh.bag
```

相对 `190310` 包只把 `w_accel` 从 `0.0` 改为 `0.3`；`w_slosh=5.0`、完整液体时域、执行器模型及其他权重不变。该包两层 postflight 也均为 `PASS`：common-epoch failure、solver failure、故障零速和回调超周期均为 `0`，planner odom 最大间隔 `21.193 ms`，完整控制回调 P95/max 为 `18.373/31.465 ms`。

命令统计只取 postflight 运动窗内 `solve_success + 非 terminal + ACADOS_OK` 的有效求解拍。转弯定义为 `|published_cmd_omega| >= 0.08 rad/s`；饱和定义为 `|a_0| >= 0.59 m/s²`；强翻转要求相邻两拍都在转弯、两拍 `|a_0| >= 0.5 m/s²` 且符号相反。

| 指标 | `w_accel=0.0` | `w_accel=0.3` |
|---|---:|---:|
| 有效求解拍 | 648 | 652 |
| 全部有效拍加速度饱和 | 478/648（73.8%） | 368/652（56.4%） |
| 转弯有效拍 | 384 | 351 |
| 转弯加速度饱和 | 297/384（77.3%） | 208/351（59.3%） |
| 转弯强正负翻转 | 60 | 53 |
| 转弯 `a_0` 主频 | 5.00 Hz | 5.13 Hz |
| 全部有效拍 slosh 占比中位/P95 | 83.3% / 92.9% | 79.8% / 90.6% |
| 转弯 slosh 占比中位/P95 | 89.4% / 93.2% | 87.0% / 90.9% |

`w_accel=0.3` 明显减少了饱和拍，但相邻 `a_0` 变化量的 P95 仍为 `1.2 m/s²`，转弯强翻转只下降约 `12%`。新包的主频仍贴近液体固有频率 $\omega_n/(2\pi)=4.97\,\mathrm{Hz}$，且液体项继续占据绝大多数有效代价。因此不能把本次结果写成“锯齿已解决”。

两包均关闭 RGB。processed-IMU 内部状态和 solver 预测量只能用于机制诊断，不能代替真实液面指标。

## 11. 下一包决策与参数化入口

下一包保持 `w_accel=0.3`、`w_du_a=0.1`、`w_alpha=0.1`，只把 `w_slosh` 从 `5.0` 降至 `1.0`。按 `192948` 已求解轨迹的现有 cost 分量做静态重标估算，`w_slosh=1.0` 时 slosh 占比中位数约为全部有效拍 `44%`、转弯 `57%`；这只是固定轨迹上的方向性估算，不能替代新权重下重新求解的实物结果。

提交 `fee6881` 已增加通用短入口，权重会进入 launch 展开检查、prereg、bag 元数据和 postflight 合同：

```bash
cd /home/geist/scout_ws

bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_weight_smoke.sh \
  --run \
  --w-slosh 1.0 \
  --w-accel 0.3
```

可选参数为 `--w-du-a` 和 `--w-alpha`；本轮不传时均保持 `0.1`。不加 `--run` 时只做 validate-only，不启动底盘。执行器 `L/tau/K`、速度安全上限、冻结路径、common epoch、fail-closed 和 legacy L22 关闭状态不对操作者开放，避免调权重时顺带改变结构与安全边界。

新包首先必须继续满足运行时零故障；随后比较饱和占比、强翻转和约 `5 Hz` 主频是否实质下降。若降低 `w_slosh` 后仍存在明显模态频率翻转，下一步应实现完整时域的 `Delta a_cmd`/jerk 代价，不继续无边界降低液体权重。只有命令连续性稳定后，才录带 RGB 的配对包判断真实液面是否改善。

## 12. 权威证据

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

后续 smoke 的机器报告：

```text
/home/geist/slosh_bags/real/20260903_spmpc_i0_failclosed_explicit_actuator_runtime_smoke_v1/H0/
DEV_I0FC_EXPACT_RUNTIME_SMOKE_V1_190310_Bslosh_i0_explicit_actuator_contract_postflight.json
DEV_I0FC_EXPACT_RUNTIME_SMOKE_V1_190310_Bslosh_runtime_postflight.json

/home/geist/slosh_bags/real/20260903_spmpc_i0_failclosed_explicit_actuator_waccel03_smoke_v1/H0/
DEV_I0FC_EXPACT_WACCEL03_SMOKE_V1_192948_Bslosh_i0_explicit_actuator_contract_postflight.json
DEV_I0FC_EXPACT_WACCEL03_SMOKE_V1_192948_Bslosh_runtime_postflight.json
```
