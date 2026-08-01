# 20260801 G3R2 W5_S10 四条 ABBA 配对确认与 Futility 停止分析

日期：2026-08-01

范围：G3R2 development paired confirmation，前 2 个完整 block / 4 个 bag

状态：`FUTILITY_STOP_NO_CONFIRMATION`，不计入正式样本量

> 结论：**停止采集 Row 05/06；W5_S10 的单次筛选正向结果没有在两个 ABBA 配对中复现，不能冻结为有效 anti-slosh 方法，也不能进入正式 Stage I。**
>
> 四条数据本身全部有效；停止原因不是采集失败，而是原冻结方向门槛已经数学上不可能通过。

## 1. 决策摘要

本批比较：

```text
Bsmooth:
  w_slosh = 0
  w_smooth = w_alpha = w_du_a = w_du_vs = 1.0

W5_S10:
  w_slosh = 5.0
  w_smooth = w_alpha = w_du_a = w_du_vs = 1.0

共同状态链：
  observer source = processed_imu
  fallback policy = fail_closed
  robot delay      = applied
  liquid rollout   = disabled
  delay mode       = fixed_robot_only
```

已经完成的顺序正好构成两个完整、顺序平衡的 ABBA block：

```text
Row 01 Bsmooth → Row 02 W5_S10
Row 03 W5_S10  → Row 04 Bsmooth
```

核心结果：

| 指标，正值表示 W5 更好 | Block 01 | Block 02 | 两 block 平均 | 冻结门槛 |
| --- | ---: | ---: | ---: | ---: |
| RGB P95 改善 `Bsmooth-W5` (mm) | +0.0000 | -0.0119 | **-0.0060** | ≥ +0.0500 |
| RGB RMS 改善 `Bsmooth-W5` (mm) | +0.0037 | -0.0051 | **-0.0007** | ≥ 0 |
| raw-IMU P95 改善 `Bsmooth-W5` (mm) | -0.1741 | +0.0461 | **-0.0640** | ≥ -0.0500 |
| RGB 正向 block | 否（持平） | 否 | **0/2** | 最终至少 2/3 |

因此：

```text
W5_S10 paired confirmation = NOT CONFIRMED
Rows 05/06                  = DO NOT ACQUIRE
formal Stage I              = NO-GO
runtime W5_S10 freeze       = NO
```

## 2. 协议与证据绑定

数据目录：

```text
/home/geist/slosh_bags/real/20260801_spmpc_g3r2_w5s10_paired_confirmation/H0/
```

release：

```text
git revision = 5f40d389e99f28dacedd19ba6cfa2bb0f745f964
protocol     = G3R2_robot_only_W5_S10_paired_confirmation_v1
```

上游单次筛选报告：

```text
G3R2_WEIGHT_SCREEN_REPORT.json
SHA-256 = e6c3b030cac925ce29887ea7d76da51d86fae8d01f6a486e5c2773f407eb7258
status  = PROMOTE_FOR_PAIRED_CONFIRMATION
selected= W5_S10
```

本批 preregistration：

```text
prereg SHA-256       = 5af9868dd94e9a3717886b24f738c6193dfba886eae6b0cd0c884276f0fce11c
order SHA-256        = d9a9d4c2ecc6da1e41017077b71180640482dce22803c6763c3e0c443606abeb
metric SHA-256       = 33b8810cd9c99b0e35d685dc54df0249c0d478b402b160e1ce41c1835fe68252
online config SHA-256= 2b35bd399e9fd88cd56c5c0fb2d797114cbf860086b3ec14a6eb234e89cb562d
path SHA-256         = 578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e
RGB calibration SHA  = 7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01
```

原协议计划 `3 blocks / 6 rows`，并写有 `no_early_stop=true`。操作者在完成 2 个完整 block 后因实验时间成本决定停止。因此本批不能冒充“按原计划完成的三配对报告”；本文将其透明记录为：

```text
protocol completion = INCOMPLETE_4_OF_6
stop type            = POST-HOC DEVELOPMENT FUTILITY STOP
formal claim         = FORBIDDEN
```

该偏离不用于放行 W5；相反，停止只是在已经无法满足冻结方向门槛后避免两条无效实车重复。

## 3. 四条数据完整性

| Row | Block/Pos | 条件 | postflight | bag SHA-256 | postflight SHA-256 |
| ---: | --- | --- | --- | --- | --- |
| 01 | 01/01 | Bsmooth | PASS | `a159691648802fdb3b9eb68c5b3f744edb5debcaa8f2e28b221e18a4bc68bae2` | `812e37949551ca5e2d87f18c98542e24e2c7cd6892a1e3f6414357e7a2c58ce2` |
| 02 | 01/02 | W5_S10 | PASS | `1b2ecc9ad81959b88d2206dd8023da16466f3c7c6527d23d56513457d440b0aa` | `dacbfba416a9ed4db57e86743c102bfafd79a7b360de65685ad09a2e0e88aa5e` |
| 03 | 02/01 | W5_S10 | PASS | `09ce81242152711aad358e871ff83344f57e90640d27948871756f0d45129fde` | `c02e23a90ca01bea670086bb5236d3374d7a40175f553a6ac7b32a670fb84873` |
| 04 | 02/02 | Bsmooth | PASS | `7a8455315c4b030b612b97d6a7557b3fd69e8397644c81e3b2b70433df4c6483` | `956235b2806145a68c466180e89165f9b14814aa582ea793d9b23e8690e0fa28` |

逐包复核结果：

- bag 文件存在，实际大小和 SHA-256 与 postflight 完全一致；
- 两个 RealSense gate（`pre_zero`、`pre_record`）四条均 PASS；
- RGB motion+tail 有效率四条均为 `100%`，在线频率约 `27 Hz`；
- processed-IMU READY/source coverage 四条均为 `100%`；
- fallback samples 均为 `0`，reset epoch 均为 `[0]`；
- robot delay applied fraction 均为 `100%`，liquid delay rollout 均为 `0%`；
- 无图像流、solver failure、tracking safety stop 或 unsafe zero intervention；
- 四条均到达目标，tracking 和 solver runtime 均通过单条门槛。

所以不能把当前负结果归因于相机时间戳、IMU reset/fallback、录包损坏或求解失败。

## 4. 单条结果

高度单位为 `mm`，tracking 单位为 `m/rad`：

| Row | 条件 | RGB P95 | RGB RMS | RGB Peak | raw-IMU P95 | contour P95 | yaw P95 | solver P95 (ms) | 到达时间 (s) |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 01 | Bsmooth | 0.6923 | 0.3680 | 2.1368 | 1.0777 | 0.0270 | 0.0700 | 4.27 | 34.855 |
| 02 | W5_S10 | 0.6923 | 0.3643 | 2.6585 | 1.2518 | 0.0422 | 0.0767 | 16.65 | 35.020 |
| 03 | W5_S10 | 0.6923 | 0.3185 | 1.5385 | 1.0551 | 0.0391 | 0.0843 | 18.31 | 34.881 |
| 04 | Bsmooth | 0.6804 | 0.3134 | 1.3846 | 1.1012 | 0.0255 | 0.0731 | 4.48 | 34.696 |

条件平均：

| 指标 | Bsmooth | W5_S10 | `Bsmooth-W5` |
| --- | ---: | ---: | ---: |
| RGB P95 (mm) | 0.6863 | 0.6923 | **-0.0060** |
| RGB RMS (mm) | 0.3407 | 0.3414 | **-0.0007** |
| raw-IMU P95 (mm) | 1.0895 | 1.1535 | **-0.0640** |
| contour P95 (m) | 0.02625 | 0.04070 | W5 高 `0.01445` |
| yaw P95 (rad) | 0.07152 | 0.08048 | W5 高 `0.00896` |
| solver P95 (ms) | 4.38 | 17.48 | W5 高 `13.10` |
| 到达时间 (s) | 34.775 | 34.950 | W5 慢 `0.175` |

W5 的 solver runtime 仍低于 `25 ms` 门槛，但大约是 Bsmooth 的 4 倍；这不是失败原因，却也没有换来可见的 RGB 改善。

## 5. 两个 ABBA 配对

冻结 paired difference 定义：

```text
Delta_b = Bsmooth - W5_S10
Delta_b > 0 表示 W5_S10 更低、更好
```

| Block | 顺序 | RGB P95 Delta | RGB RMS Delta | raw-IMU P95 Delta | contour：W5-Bsmooth |
| ---: | --- | ---: | ---: | ---: | ---: |
| 01 | Bsmooth → W5 | +0.0000 | +0.0037 | -0.1741 | +0.01530 m |
| 02 | W5 → Bsmooth | -0.0119 | -0.0051 | +0.0461 | +0.01360 m |
| 平均 | ABBA balanced | **-0.0060** | **-0.0007** | **-0.0640** | **+0.01445 m** |

顺序翻转后结论没有变成 W5 正向：Block 01 持平，Block 02 略微支持 Bsmooth。W5 在两个 block 中都产生更大的 contour error，说明当前代价组合至少稳定地牺牲了一部分路径跟踪，却没有得到独立 RGB 所支持的液面收益。

## 6. 为什么 Row 05/06 已无必要

原冻结门槛要求：

```text
minimum_positive_blocks = 2 of 3
```

已完成两个 block 的 RGB P95 Delta 为：

```text
Block 01 =  0.0000 mm  # 不算正向
Block 02 = -0.0119 mm  # 反向
```

即当前正向计数为 `0/2`。剩余最多只有一个 Block 03，因此即使它正向，最终也只能达到 `1/3`，不可能满足 `2/3`。这是离散方向门槛上的不可逆 futility，不依赖对第三个差值大小作任何预测。

其他门槛当前也不支持 W5：

- RGB P95 平均改善为 `-0.0060 mm`，低于要求的 `+0.0500 mm`；
- RGB RMS 平均改善为 `-0.0007 mm`，方向略微反向；
- raw-IMU P95 平均改善为 `-0.0640 mm`，超过最大允许退化 `0.0500 mm`。

因此继续 Row 05/06 只能补齐原计划样本数，不能改变冻结方向门槛的最终失败结论。在 development、时间受限且不允许把失败批次计入正式样本的前提下，停止比继续消耗实车时间更合理。

## 7. 与单次筛选结果的关系

单次筛选中，W5_S10 相对外部 Bsmooth baseline 的结果是：

```text
RGB P95: 0.6923 -> 0.6154 mm
improvement = +0.0769 mm（约 11.1%）
```

该结果满足单次候选筛选门槛，所以进入 paired confirmation 是正确流程。但新 ABBA 数据为：

```text
Block 01: W5 与 Bsmooth 持平
Block 02: W5 比 Bsmooth 高 0.0119 mm
```

这说明单次筛选正向没有重复性支持。不能选择性保留 screening Row 03 而忽略 confirmation；配对确认的目的正是过滤偶然初始液面、时间漂移、路径回位误差和低幅值 RGB 波动。

目前幅值已经接近在线 RGB 标量的低幅差区域，两个方法的 P95 多次落在 `0.68--0.69 mm` 附近。现有证据允许说“没有可重复改善”，但不足以唯一断言偶然正向来自 RGB 量化、初始液体状态或其他某一个单独原因。

## 8. 对 IMU、模型和控制器的含义

本批不否定 processed-IMU 的工程可用性：四条 READY/source coverage 都是 `100%`，无 reset/fallback，状态链稳定。被否定的是更窄的命题：

```text
在当前路径、当前二阶液体模型、fixed_robot_only 状态口径和
w_slosh=5 / smooth split=1.0 下，W5_S10 能可重复降低真实 RGB 液面。
```

与旧 G3 不同，本批已经关闭 liquid delay rollout，仅对机器人状态做 delay prediction。因此当前负结果不能继续简单归因于“液体状态被错误延迟传播”；更可能需要从以下几项离线区分：

1. 当前二阶模态状态与 RGB 低幅液面之间的可观测关系是否足以支撑在线代价；
2. `w_slosh=5` 是否主要改变路径/速度分配，却没有改变真实液面激励；
3. 模型相位、阻尼、旋转耦合或输入定义是否使 slosh cost 的梯度方向与实物不一致；
4. 当前 H0 路径和 `v_ref=0.20` 的激励是否太弱，RGB 主指标分辨率不足以区分方法；
5. 单次 screening 正向是否来自初始条件与低幅统计波动。

## 9. 冻结决策与声明边界

允许的结论：

- 四条 confirmation 数据质量 PASS；
- 两个完整 ABBA 配对均未显示 W5 的 RGB P95 优势；
- W5_S10 没有通过 development paired confirmation；
- 依据不可逆方向门槛停止 Row 05/06；
- W5_S10 不进入正式 Stage I。

不允许的结论：

- 不能把 4/6 样本称为“完成原三配对协议”；
- 不能宣称 Bsmooth 在统计意义上显著优于 W5；
- 不能宣称 processed-IMU 无效；
- 不能把 screening 的单次正向当成论文实物 efficacy；
- 不能通过选择性重录 W5 或修改阈值挽救当前 release。

最终冻结标签建议为：

```text
report_status       = FUTILITY_STOP_NO_CONFIRMATION
protocol_completion = INCOMPLETE_4_OF_6
selected_method     = NONE
W5_S10              = REJECT_FOR_FORMAL_STAGE
Bsmooth             = SAFE_COMPARATOR_ONLY
formal_stage        = NO_GO
```

## 10. 下一步

停止今天的 W5/Bsmooth 实车重复，优先使用现有四包做 G4 离线诊断：

1. 在统一 bag-time 窗口逐弯段对齐 RGB、raw processed-IMU、solver-input state、slosh cost 和首控制量；
2. 比较 screening 正向 Row 03 与 confirmation 四条的初始零点、命令频谱、曲率段和液体峰值位置；
3. 检查 W5 一致增大的 contour error 是来自速度、角速度还是 progress/lag trade-off；
4. 做不发布 `/cmd_vel` 的 replay ablation，测试关闭 slosh cost、修改模型参数或旋转一致传播后的预测排序；
5. 若模型或权重改变，建立新 development release；不得把当前四包重新解释为新方法结果；
6. 在新的 paired evidence 通过前，不开始正式 Stage I。

相关协议文档：

- [G3R2 robot-only 四候选单次筛选实验命令](../实物对比实验/正式论文实验/20260801_G3R2_robot-only四候选单次筛选实验命令.md)
- [G3R2 W5_S10 三配对确认实验命令](../实物对比实验/正式论文实验/20260801_G3R2_W5_S10三配对确认实验命令.md)
- [G3 延迟状态失配与 G3R 放行分析](./20260801_G3延迟状态失配与G3R放行分析.md)
