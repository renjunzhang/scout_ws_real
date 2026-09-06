# 20260903 I0 + 显式执行器 OCP：Block 1、权重 smoke、RGB 配对与 full-da smoke 分析

> 日期：2026-09-03；最近更新：2026-09-06
>
> 性质：development 实物诊断，不作为正式降晃效果声明
>
> 协议：`SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_ABBA_DEV_V1`、`SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_RUNTIME_SMOKE_DEV_V1`、`SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_WACCEL03_SMOKE_DEV_V1`、`SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_WEIGHT_TUNING_SMOKE_DEV_V1`、`SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_WS1_WA03_ABBA_DEV_V2`、`SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_FULL_DA_SMOKE_DEV_V1`
>
> 初始写入基线：`diag/lt-dwa-collision-tracking @ a2eacb9f30e14b7439714a558b00696481054e85`
>
> smoke 续写基线：`diag/lt-dwa-collision-tracking @ fee6881eb1bfc689912c8ad841988f9b1de15e12`，续写前工作区 clean
>
> RGB 配对采集与当次续写基线：`diag/lt-dwa-collision-tracking @ 4e4eaaecec444a8931f33170ca7cddc4e32fce76`，续写前工作区 clean
>
> full-da smoke 采集与 2026-09-06 续写基线：`diag/lt-dwa-collision-tracking @ 8228d1e4fd6efb74da84c5ea72e8e9333293d973`，续写前工作区 clean
>
> 对应方案：[20260903_I0显式执行器OCP_B0_Bslosh_ABBA验证方案.md](../实物对比实验/20260903_I0显式执行器OCP_B0_Bslosh_ABBA验证方案.md)

## 1. 结论

截至 2026-09-06，新增完整时域 `Delta a_cmd` 后的一包无 RGB smoke 运行正常，但两项连续性指标仅降低 `33.8% / 42.3%`，均未满足预注册的 `50%` 门；不能进入新 RGB 配对。现有 bag 足够继续离线排查，不需要为当前分析重录。新结果与口径边界见第 12 节。

2026-09-03 同权重 RGB 配对的有效负结果继续保留，不能用后续无 RGB smoke 推翻：

1. 最新 V2 配对的 `B0` 与 `Bslosh` 均完成路径；主合同、processed-IMU observer、NOKOV 链和 RGB 标量后验全部通过，common-epoch failure、solver failure 与故障零速均为 `0`。此前的运行时硬停车没有复现。
2. 在两包共同使用 `w_accel=0.3` 时，`Bslosh(w_slosh=1.0)` 的 RGB `H_vis` P95/RMS 分别为 `1.5423/0.7214 mm`，高于 `B0` 的 `0.9231/0.4410 mm`。按预注册的 `B0-Bslosh` 定义，差值为 `-0.6192/-0.2804 mm`，方向明确不利于 `Bslosh`。
3. `Bslosh/B0` 到点时间比为 `1.0323`，低于 `1.05` 的减速混杂门。因此不能用明显降速解释本次 RGB 负结果。
4. 机器判定为 `BLOCK1_RAPID_SCREEN / STOP / STOP_BLOCK1_FUTILITY`。这是协议停止门正常生效，不是分析程序故障；不得绕过门继续录 Row03/Row04。
5. 同口径控制诊断显示，`Bslosh` 的 `|a_0|` P95、`|Δa_0|` P95 和转弯约 `5 Hz` 分量均明显高于 `B0`；processed-IMU 内部模态高度 P95 也由 `1.6788 mm` 升至 `2.9871 mm`。RGB 与 I0 的方向一致，支持“当前液体代价仍在引入残余模态频率控制”这一机制判断。
6. `w_slosh=1.0` 相比先前 `w_slosh=5.0` smoke 已显著减轻锯齿，但与同为 `w_accel=0.3` 的公平 `B0` 相比仍没有形成降晃收益。当前最直接有效的运行配置是 `explicit actuator + B0 + w_accel=0.3`；现有 `Bslosh` 液体代价配置不再继续做无边界降权扫描。
7. 该结果只是一组 development Block1 的停止结论，不是正式普遍性效能声明，也不否定 command/actual 分离和显式执行器 OCP 的结构修复。它否定的是“当前液体代价形式与参数已能在实物上产生净收益”。

历史上，V1 Row02 的 `B_slosh(w_slosh=5)` 曾因 callback 阻塞产生 `47` 拍共同状态失败和 `10` 拍 acados `MINSTEP`，只能作为运行时故障证据。提交 `be8f1ef` 及后续 smoke 已消除这层硬停车；最新 V2 配对则是在运行时有效后，对液体代价本身给出的独立负结果。

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
- 最新 V2 的负结果能否跨日期、重复 block 和更高速度复现。
- 液体代价形式、完整时域窗口、权重和平滑项各自对残余约 `5 Hz` 控制的独立贡献。

因此，V1 Block 1 不否定 command/actual 分离和显式执行器 OCP 的结构方向；它当时否定的是“该 revision 的 `B_slosh` runtime 已可直接进入效果比较”。后续 smoke 解除了运行时阻塞，V2 又在有效配对中进一步否定了“当前 `w_slosh=1.0` 液体代价已有净降晃收益”。

## 8. 最小修复与验收执行记录

1. 已将 odom 接收从求解 timer 的单线程回调队列中解耦，使用独立 callback queue/spinner，并为 odom history 和相关快照补齐同步保护。
2. 已缓存显式执行器 prefix rollout 的固定离散模型，并冻结 `qp_solver_cond_N=10`，降低运行时前处理和求解开销。
3. `max_interpolation_gap_sec=0.05` 保持不变，没有通过放宽门限掩盖 planner 内部丢样本。
4. `190310` 低风险 `B_slosh` smoke 已满足：
   - common-epoch failure 为 0；
   - solver failure 为 0；
   - 无故障性零速；
   - planner odom 不再出现超过 50 ms 的内部空洞；
   - 控制回调 P95 低于 33.3 ms，超周期不形成连续积压。
5. `192948` 以 `w_accel=0.3` 检查无故障区间的加速度饱和和正负翻转，结果为“部分改善、未解决”。
6. 随后把 `w_slosh` 从 `5.0` 降至 `1.0`，锯齿显著收敛，具备开展低速配对的最低条件。
7. 最新 V2 以相同 `w_accel=0.3` 完成 `B0 -> Bslosh` RGB 标量配对，运行时有效但效果门失败，按预注册规则停止于 Row02，不录 Row03/Row04。

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

## 11. `w_slosh=1.0, w_accel=0.3` 的 smoke 与 RGB 标量配对

配对前先录制了不带 RGB 的单包 smoke：

```text
/home/geist/slosh_bags/real/
20260903_spmpc_i0_failclosed_explicit_actuator_weight_tuning_smoke_v1/H0/
DEV_I0FC_EXPACT_WEIGHT_TUNING_V1_204315_Bslosh.bag
```

该包采集于 `67babe7`，主合同与 runtime 后验均为 `PASS`：common-epoch failure、solver failure、故障零速和 planner odom 超 50 ms 间隔均为 `0`；控制回调 P95/max 为 `14.574/35.125 ms`，只有 `1` 个孤立超周期且未形成积压。它只用于确认 `w_slosh=1.0, w_accel=0.3` 已达到低速配对的运行条件，不提供 RGB 效能结论。

### 11.1 冻结条件与数据

采集 revision 为：

```text
4e4eaaecec444a8931f33170ca7cddc4e32fce76
```

协议和数据目录：

```text
SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_WS1_WA03_ABBA_DEV_V2

/home/geist/slosh_bags/real/
20260903_spmpc_i0_failclosed_explicit_actuator_ws1_wa03_abba_v2/H0/
```

本次有效 bag：

```text
DEV_I0FC_EXPACT_WS1_WA03_V2_01_B0_b01_p01_a01.bag
DEV_I0FC_EXPACT_WS1_WA03_V2_02_Bslosh_b01_p02_a01.bag
```

两包共同冻结 `processed-IMU I0 + fail_closed + common_epoch + explicit_actuator`、legacy L22 关闭、`v_ref=0.20 m/s`、`v_safe_max=0.25 m/s`、`w_accel=0.3`、`w_du_a=0.1`、`w_alpha=0.1`。唯一控制变量是：

| 条件 | 液体代价 | `w_slosh` |
|---|---|---:|
| `B0` | 不进入 solver | 0.0 |
| `Bslosh` | I0 进入 27D solver，完整 `0..60` 状态节点 | 1.0 |

bag 记录的是在线 RGB 液面标量 `/liquid/measurement` 和相机信息，没有录入原始图像流；因此这里的“RGB 配对”专指冻结 detector 生成的在线标量证据，不能从这两包重新处理原始 RGB 图像。

### 11.2 有效性

| 检查项 | B0 | Bslosh |
|---|---:|---:|
| 主合同后验 | `PASS` | `PASS` |
| observer 后验 | `PASS` | `PASS` |
| NOKOV 链 | `pass=true` | `pass=true` |
| RGB 标量后验 | `PASS` | `PASS` |
| common-epoch bad | 0 | 0 |
| solver failure | 0 | 0 |
| 故障/安全零速 | 0 | 0 |
| processed-IMU READY/source coverage | 1.0 / 1.0 | 1.0 / 1.0 |
| 到达终点 | 是 | 是 |

两包起步前 5 s 的 RGB 稳定性检查均完整且通过。由此，本节的负结果不是 fail-closed 停车、observer fallback、NOKOV 失效或分析输入不完整造成的。

### 11.3 RGB 结果与停止门

主指标窗口为运动开始至首次 `GOAL_REACHED`，再加 5 s tail；`H_vis` 使用因果 5 点中值滤波。

| 指标 | B0 | Bslosh | `B0-Bslosh` |
|---|---:|---:|---:|
| `H_vis` P95 | 0.9231 mm | 1.5423 mm | -0.6192 mm |
| `H_vis` RMS | 0.4410 mm | 0.7214 mm | -0.2804 mm |
| `H_vis` Peak | 1.6923 mm | 2.3789 mm | -0.6866 mm |
| 到点时间 | 29.0876 s | 30.0265 s | 比值 1.0323 |

协议定义差值为 `B0-Bslosh`，正值才有利于 `Bslosh`。P95 和 RMS 均为负；到点时间比 `1.0323` 又低于 `1.05` 的减速风险门，因此机器决策为：

```text
phase    = BLOCK1_RAPID_SCREEN
status   = STOP
decision = STOP_BLOCK1_FUTILITY
```

此前 Row03 启动时出现的 `expected=['PROMOTE_BLOCK2']` 正是该停止门在阻止无效续跑，不是 runner 故障。不得绕过，也不补录 Row03/Row04。

### 11.4 内部 slosh 高度与控制诊断

同一 RGB 后验窗口内，内部高度也不支持 `Bslosh`：

| 内部指标 P95 | B0 | Bslosh | `Bslosh-B0` |
|---|---:|---:|---:|
| processed-IMU 模态高度 | 1.6788 mm | 2.9871 mm | +1.3083 mm（+77.9%） |
| 预测/solver-input debug 模态高度 | 1.7507 mm | 2.9473 mm | +1.1966 mm（+68.4%） |

`B0` 的液体状态只作为 shadow/debug 证据，并未进入 23D solver；`Bslosh` 才实际消费 I0。所以上表可用于判断 observer 与 RGB 的方向是否一致，不能把内部模型量替代为真实液面真值。本次两者方向一致：`Bslosh` 都更高。

沿用有效求解运动窗的同口径诊断：

| 控制指标 | B0 | Bslosh |
|---|---:|---:|
| `|a_0|` P95 | 0.0387 m/s² | 0.1674 m/s² |
| `|Δa_0|` P95 | 0.0187 m/s² | 0.1570 m/s² |
| 转弯 `a_0` 约 5 Hz 幅值 | 0.00775 | 0.07821 |
| `|Δv_cmd|` P95 | 0.00122 m/s | 0.00553 m/s |
| `|Δω_cmd|` P95 | 0.00730 rad/s | 0.00987 rad/s |
| 强正负翻转 | 0 | 0 |

降低到 `w_slosh=1.0` 后，先前接近饱和的强翻转已经消失，但 `Bslosh` 仍保留明显高于 `B0` 的约 5 Hz 加速度分量和命令变化。这与 RGB/I0 同向，支持“当前液体代价仍引入残余模态频率控制”的解释；单个 block 尚不足以证明唯一因果机制。

### 11.5 当前决策

- 当前最直接有效的实物配置保留为 `explicit actuator + B0 + w_accel=0.3`。
- `w_slosh=1.0` 不进入下一 block，也不继续靠更低 `w_slosh` 做无边界扫描；趋近 0 只会退化回 B0。
- 若继续研究 `Bslosh`，应作为新 development 协议先修改液体目标或加入完整时域 `Δa_cmd/jerk` 连续性约束，再从 smoke 开始，不能把本次 Row03/04 当作补录任务。
- 上述决策只适用于当前 C02、低速、固定执行器参数和单个 Block1；正式效能结论仍需新的预注册重复实验。

## 12. 20260906 完整时域 Delta a_cmd smoke 与离线排查

### 12.1 版本、冻结条件与数据

这是独立 development smoke，不是旧 V2 配对的续行。实现提交为 `064e5e6`，六图诊断为 `a432b79`；采集提交 `8228d1e` 仅修复旧 Matplotlib 测试中 `imread(Path)` 的兼容性。采集前绘图测试 `6/6`、full-da `--validate-only` 均通过。

```text
protocol: SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_FULL_DA_SMOKE_DEV_V1
bag: /home/geist/slosh_bags/real/20260906_spmpc_i0_failclosed_explicit_actuator_full_da_smoke_v1/H0/DEV_I0FC_EXPACT_FULL_DA_SMOKE_V1_165501_Bslosh.bag
revision: 8228d1e4fd6efb74da84c5ea72e8e9333293d973

B_slosh + processed-IMU I0 + fail_closed + common_epoch
explicit_actuator；legacy delay off
w_slosh=1.0, w_accel=0.3, w_du_a=0.1, w_alpha=0.1
v_ref=0.20 m/s, v_safe_max=0.25 m/s, a_max=0.6 m/s²
N=60, dt≈1/30 s, cond_N=10；B0/slosh nx=24/28，np=28/37
PreSolveSnapshot / PredictedHorizon schema v4；RGB disabled
```

路径继续使用冻结 C02；map/path SHA-256 分别为 `34e45fd8205a766dbc6e3dcea667c5a0a618e26b331d48351c25645e31a19595`、`1464ef37857bcb899d8b0e4867ff63ea06f017e1b871bed80e077f450be14164`。定位恢复后未重建地图，也未另生成路径。

同目录、同 bag stem 下的权威产物：

```text
*_runtime_smoke_prereg.env
*_one_click_meta.env
*_i0_explicit_actuator_contract_postflight.json
*_runtime_postflight.json
*_diagnostic_plots/01_timing.png ... 06_cost_frequency.png
```

### 12.2 验收结果：运行正常，连续性未通过

主合同后验为 `PASS`，含连续性门的 runtime 后验为 `FAIL`，失败项恰为以下两项；不能把主合同通过解释成整个 smoke 通过。

| 指标 | 冻结 Bslosh 基线 | 本包 | 降幅 | 通过阈值 |
|---|---:|---:|---:|---:|
| `abs(Delta a0)` P95，m/s² | 0.1569965 | 0.1039313 | 33.8% | ≤0.0785 |
| 转弯约 5 Hz `a0` 幅值，m/s² | 0.0782094 | 0.0451378 | 42.3% | ≤0.0391 |

基线为第 11 节的 `DEV_I0FC_EXPACT_WS1_WA03_V2_02_Bslosh_b01_p02_a01.bag`，不是另选更差的历史包。新包频带峰为 `5.04425 Hz`；强正负翻转为 `0`。冻结 runtime gate 使用运动窗内有效、成功、非终端 solver 周期的相邻 `a0` 差分；转弯以 `abs(published_cmd_omega)≥0.08 rad/s` 筛选后拼接，沿用固定 `30 Hz`、`4.5–5.5 Hz` 的 DFT 口径。

其他运行门全部通过：到达终点；common-epoch failure、solver failure、故障零速、planner odom `>50 ms` 间隔均为 `0`；odom 最大间隔 `21.070 ms`；callback P95/max 为 `13.877/30.191 ms`，超 `33.3 ms` 周期数与连续超周期均为 `0`。运动窗为 `[1788684939.9300244, 1788684969.6343887]`，共 `892` 个 audit，连续性筛选后 `654` 个 solver 周期、`653` 个相邻差分及 `339` 个转弯样本。

### 12.3 已确认的离线事实

本节使用当前 bag、当前源码与本机 generated 编译库做只读重算，没有启动 ROS 运动节点、改控制参数或重新生成 solver。它是机制诊断，不替代上一节机器验收。

1. **发布链未引入这段振荡。** 654 个有效非终端周期中，solver 与最终发布的线/角命令最大差均为 `0`，terminal/safety/command-contract 介入计数为 `0`。此结论不外推到终端接管段。
2. **两种“Delta a0”不是同一量。** 同口径相邻 `a0` 差分 P95 为 `0.1039313 m/s²`；在上一轮计划可用的有效拍中，`abs(当前 a0 - 上轮预测 a1)` P95 为 `0.0235764 m/s²`。另由当前 `a0` 减 pre-solve 最终命令历史 memory 得到 P95 `0.1039102 m/s²`。前者说明当前输出随时间的起伏，第二个量说明重规划对旧计划的偏离；命令链图用第二种量，不能代替验收的第一种量。图中整个运动窗重采样/Hann 频谱也不能代替 gate 的转弯拼接 DFT。
3. **振荡已存在于预测序列内。** 运动后约 `7.3–13.5 s`、`15.7–20.0 s` 的两段中，完整 60 步 `a_cmd` 序列的 `4.5–5.5 Hz` 峰值幅值中位数约为 `0.0204/0.0203 m/s²`。这与命令链上当前 `a0` 和旧计划 `a1` 一起振荡的现象一致，支持优先检查预测内控制与模型/代价的关系，尚不足以证明唯一根因。
4. **当前实际缩放已核实。** [当前代价表达式](../../../../src/scout_apps/control/spmpc_local_planner/scripts/acados/spmpc_acados_cost.py) 对 running 项除以 `N`；本机 generated `cost_scaling[0..59]=0.0333333333`、`cost_scaling[60]=1`，所以实际 running 系数为 `dt/N≈0.00055556`，terminal 为 `1`。654 个周期的 stage 0、terminal 代价与已编译 C 函数交叉核对，最大绝对误差 `3.47e-18`。缩放事实来自当前实现，不来自其他分支的方案；不能据此直接认定“终端项过强”就是实车振荡根因。

真实目标的分项重算还显示，转弯段 `J_Delta_a`、液体位置 running、液体速度 running 的中位数约为 `4.09e-6 / 3.22e-4 / 9.79e-5`。这些是代价值，不是优化敏感度或因果贡献；不能单凭大小自动提高 `w_du_a` 或降低 `w_slosh`。发布的 cost 图只能辅助看趋势，不等于包含真实 stage/terminal 缩放的总目标。

### 12.4 特定去频带扰动：模型内机制证据

方法：按时间排序的有效非终端周期每 5 拍取样一次，再筛选转弯条件，得到 `69` 个快照。对每个快照保持初态、全部参数、`alpha_cmd` 和 `v_s` 不变，只把 60 步 `a_cmd` 的实数 DFT 中 `4.5–5.5 Hz` 分量置零；原序列与去频带序列都从同一初态按当前离散动力学重新 rollout，再用实际 `dt/N` 与 terminal 缩放计价。未重新优化，也未向 ROS 发布任何结果。

- `69/69` 对序列均通过本次检查的控制与非终端 actual/command 速度 box bounds；这不等于完整安全后验或实车可执行性证明。
- `69/69` 的全时域 `Delta a` 代价下降，但液体 running 代价和总目标均升高。总目标增量中位数为 `8.95e-5`。
- 仅在离线计价中将 running 缩放假设为 `1`，仍有 `67/69` 的去频带序列总目标更高。这个检查不等于按新缩放重求解，更不能预测修改缩放后的闭环结果。

当前证据支持：相对于这一特定的平滑扰动，原带频带序列在当前模型/目标中更划算，软连续性代价没有压过相关代价取舍。它不能证明真实液体从该振荡中获益，也不能把“去频带处理”当成拟上线修复。需要继续核对模型相位/幅值、液体状态反馈和连续性约束。

数值离散化也保留为待查线索：快照参数对应的液体固有频率约 `4.973 Hz`；单步 RK4 的自由模态等效衰减率约 `1.7453 s⁻¹`，连续模型为 `1.5623 s⁻¹`，约 333 ms 后的自由响应幅值比为 `0.9408`。这只是同一模型的数值传播差异，不是实车受迫响应或 RGB 误差测量，尚未据此改求解器。

### 12.5 当前决策与后续边界

- 保留本包和失败结果，不降低 50% 门、不进入新 RGB 配对；现有命令、IMU、odom、snapshot 和 horizon 足够下一步离线排查，当前不用再录 bag。
- 继续用实际已发布命令核对执行器/液体预测，区分未来命令被重规划改变、模型幅值/相位误差和数值传播差异。若做 solver 反事实比较，先完成原配置 replay 复现，再做单变量候选；本次代价重算和去频带 rollout 不等于完成 solver replay。
- 速度、加速度上限、`w_slosh`、执行器参数与地图/路径均保持冻结。下一项控制修复尚未选定；后续若实施，仍需软件验证和由操作者执行的新无 RGB smoke 达标后，才录新 RGB B0/Bslosh 配对。
- 用户已明确 `解决问题的思路/代码改造方案_唯一主线.md` 属于另一个分支，与当前分支无关。本节不采用它作为修改依据，不修改该文件，也不把该分支的重构设想记作已实现。
- 本次助手只做离线诊断及文档更新，没有改 runtime/solver 代码，也没有启动实车；文档按用户要求做阶段性提交，不推送。

对应当前分支的实施与验收入口见 [I0 与显式执行器 OCP 最小修复方案](../解决问题的思路/20260903_I0与显式执行器OCP最小修复方案.md)。

## 13. 权威证据

V1 主后验：

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

/home/geist/slosh_bags/real/20260903_spmpc_i0_failclosed_explicit_actuator_weight_tuning_smoke_v1/H0/
DEV_I0FC_EXPACT_WEIGHT_TUNING_V1_204315_Bslosh_i0_explicit_actuator_contract_postflight.json
DEV_I0FC_EXPACT_WEIGHT_TUNING_V1_204315_Bslosh_runtime_postflight.json
```

最新 V2 RGB 标量配对总报告：

```text
/home/geist/slosh_bags/real/20260903_spmpc_i0_failclosed_explicit_actuator_ws1_wa03_abba_v2/H0/
I0_FAILCLOSED_EXPLICIT_ACTUATOR_WS1_WA03_ABBA_RGB_ANALYSIS.json
```

V2 每包主合同与 RGB 后验：

```text
DEV_I0FC_EXPACT_WS1_WA03_V2_01_B0_b01_p01_a01_i0_explicit_actuator_ws1_wa03_v2_postflight.json
DEV_I0FC_EXPACT_WS1_WA03_V2_01_B0_b01_p01_a01_i0_explicit_actuator_ws1_wa03_v2_rgb_postflight.json

DEV_I0FC_EXPACT_WS1_WA03_V2_02_Bslosh_b01_p02_a01_i0_explicit_actuator_ws1_wa03_v2_postflight.json
DEV_I0FC_EXPACT_WS1_WA03_V2_02_Bslosh_b01_p02_a01_i0_explicit_actuator_ws1_wa03_v2_rgb_postflight.json
```

同目录下的 `*_observer_postflight.json` 与 `*_mocap_chain_postflight.json` 分别约束 processed-IMU observer 和 NOKOV 执行链；两包均无 failure。
