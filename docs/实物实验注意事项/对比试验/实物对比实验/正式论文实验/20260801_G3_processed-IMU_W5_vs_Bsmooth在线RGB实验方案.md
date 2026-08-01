# G3 processed-IMU W5 vs Bsmooth 在线 RGB efficacy 实验方案

> 日期：2026-08-01
>
> 范围：G3 development gate，永久不计入正式 `40/64/88` 的样本数。
>
> 当前状态：**G3 专用 wrapper/postflight/analyzer 和 RealSense 冻结入口已实现，开动前仍需提交/冻结 release**。不动车的 developer `VALIDATE_ONLY` 已通过。
>
> 阶段边界：**G3 PASS 只放行 G4/G5/G6，不直接放行正式 Stage I。**

## 1. 当前事实与 G2C 决策

G2C 已完成 `W2 → W5 → W5 → W2` 两个配对 block，4 条 postflight 全部 `PASS`：

- 运动段 processed-IMU effective-source 覆盖率 `100%`；
- IMU `READY` 覆盖率 `100%`；
- fallback/reset/solver failure/tracking-safety stop 均为 `0`；
- 4 条均 `GOAL_REACHED`；
- bag 中 image stream 计数均为 `0`。

统一使用非零 `/cmd_vel` 运动窗口比较，W5 相对 W2 的主要变化为：

| 指标 | W5 相对 W2 |
| --- | ---: |
| 运动时间 | `+2.79%` |
| contour P95 | 约 `-0.8%`，基本不变 |
| yaw P95 | 约 `+9.5%`，绝对值约 `0.071 → 0.078 rad` |
| 当前模型晃液 P95 | 约 `-16.5%` |
| 当前模型晃液最大值 | 约 `-18.8%` |
| horizon 峰值 P95 | 约 `-15.8%`，两个 block 均同向 |
| solver P95 | 约 `+1.8%`，仍为 `12–13 ms` |
| first action `|a| / |alpha|` P95 | 约 `-0.3% / +3.0%` |

因此本方案将以下内容作为 G3 的唯一候选：

```text
current-state source = processed_imu
Bslosh candidate     = W5
w_slosh              = 5
v_ref                 = 0.20 m/s
delay                 = fixed_closed_loop, linear 0.15 s / angular 0.22 s
```

G2C 的 `H_modal` 只能用于候选筛选，不能证明真实液面降低。G3 必须用新的在线 RGB 物理测量独立检验 `W5` 相对 `Bsmooth` 的效果。

G2C 历史执行命令见 [20260731_G2C_processed-IMU_W2W5实验命令.md](./20260731_G2C_processed-IMU_W2W5实验命令.md)。

## 2. G3 研究问题

在相同容器、液深、起点、路径、参考速度、定位、延迟补偿和执行限幅下，比较：

| 条件 | 控制器 | 液体状态进入 OCP | 主要作用 |
| --- | --- | --- | --- |
| `Bsmooth` | `B_smooth` | 否 | 平滑代价 comparator |
| `Bslosh-W5` | `B_slosh`, `w_slosh=5` | 是，必须为 processed-IMU | 最终 S-MPCC development candidate |

主问题是：

> `Bslosh-W5` 是否在不破坏成功率、tracking 和实时性的前提下，对真实 RGB 液面指标 `H_vis,p95(motion+tail)` 产生有实质意义且不由单个 block 独占的改善？

定义 block 内效应：

```text
Delta_b = H_vis,p95(Bsmooth, b) - H_vis,p95(W5, b)
```

`Delta_b > 0` 表示 W5 的真实液面晃动更小。

## 3. 第 1 条 G3 前的入口状态

本批 G3 使用“明确保留限制的 development bridge”：接受已有 G2S 工程决策选择 processed-IMU，但不将它改写为正式四包 G2S PASS。

| 入口项 | 当前状态 | 关闭要求 |
| --- | --- | --- |
| G2S source-selection | **ACCEPTED FOR G3 DEVELOPMENT WITH LIMITATION** | wrapper 核对旧 report 的 `3/3`、`34.56%`、coverage/dominance/hash，并在 prereg 中保留 `THREE_TRIAL_G2C_DEVELOPMENT_ONLY` |
| G2S 原始证据 | **FORMAL LIMITATION** | 旧 3 个原始 bag 已不存在，不可复算；该限制不被删除，G6/formal 前必须重新 source-validation 或明确冻结接受范围 |
| G2C candidate bridge | **PASS FOR G3 DEVELOPMENT** | wrapper 逐个打开 4 条 postflight，核对 bag 存在/大小、`PASS`、100% processed-IMU/READY 和无 fallback，并生成联合 evidence hash |
| IMU implementation bridge | **PASS FOR G3 DEVELOPMENT** | 绑定 no-command smoke 和 4 条 G2C effective-source 实车证据；正式独立 report 仍须在 G6 前生成 |
| G3 预注册 bundle | **IMPLEMENTED** | Row 01 前由 wrapper 一次生成并冻结 prereg、order、RGB metric/window 和 online-config hash |
| G3 专用实车工具 | **IMPLEMENTED / TESTED** | 8-row wrapper、RealSense freeze、fail-closed postflight 和四 block analyzer 已实现，语法/单元/developer validate-only 通过 |
| 冻结 release | **PENDING** | Row 01 前提交 G3 代码/文档，用干净的相关 runtime path 重跑 `VALIDATE_ONLY` |

若要完全关闭旧 G2S 的可复算缺口，最保守方法仍是新录 `4` 条 `Bsmooth + 同 trial odom/processed-IMU observer + 在线 RGB scalar` 的配对 source-validation。这不是当前 G3 development 的追加样本，也不计入正式 `n`。

## 4. 实验设计冻结建议

以下数值是本方案的预冻结建议。只有它们被写入 G3 prereg bundle、生成 hash 并提交，才是机器可验证的最终冻结值。

| 项目 | 冻结建议 |
| --- | --- |
| 范围 | development only |
| 容器 | 当前 `C1/tube`，液深、容器位姿和摄像机相对位姿整批不变 |
| 路径 | `/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json` |
| 路径 SHA-256 | `578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e` |
| formal 路径隔离 | 不使用 H1/L1，不占用正式样本 |
| 条件 | `Bsmooth` 与 `Bslosh-W5` |
| 输入源 | W5 必须有效消费 `processed_imu`；Bsmooth 可运行同一 observer 作诊断，但必须明确记录 `solver_consumes_selected_state=false` |
| paired blocks | `n_dev=4` |
| planned rows | `8` |
| 最小有效配对 | `4/4`；在当前没有冻结 retry verifier 时，任一行无法闭包则 G3 不得签 PASS |
| `v_ref` | `0.20 m/s` |
| delay | `fixed_closed_loop`, `0.15/0.22 s` |
| 回位后静稳 | `T_SETTLE >= 10 s`，同时必须通过 RGB zero-lock |
| 运动前 RGB | 至少 `2.0 s` 连续 `valid + zero_locked + STATUS_OK` |
| `T_HVIS_TAIL` | `5.0 s`，从 first arrival 后继续零速录制 |
| 录包到首个有效运动 | 不得超过 `23 s`；建议正常值约 `20 s`，超过即 postflight FAIL |
| `T_MOTION_MAX` | `42 s`，高于 G2C 实测约 `35.7–36.8 s` |
| 录制边界 | **最多 `70 s`**；wrapper 必须核对 `record-to-motion <= 23 s`、motion 和 `5 s` tail 均被完整覆盖 |
| RGB 入 bag | 只录 `/liquid/measurement` 和 camera/config sidecar；raw/compressed/depth/debug image 全部 `0` 条 |
| 调参预算 | Row 01 后两条件均为 `0`；不得根据已看到的 RGB 修改 W5、标尺、滤波、lag 或门槛 |
| 提前停止 | 禁止；除安全原因外，必须完成全部 8 条 planned rows |

### 4.1 固定平衡顺序

预冻结顺序为 `A B | B A | B A | A B`，其中 `A=Bsmooth`、`B=W5`。两种方法各有 2 次在 block 内先执行、2 次后执行。

| `G3_ROW` | block | position | condition | pilot method | 建议 run label |
| --- | --- | --- | --- | --- | --- |
| `01` | `01` | `01` | `Bsmooth` | `Bsmooth` | `DEV_G3_H0_C1_Bsmooth_b01_p01_a01` |
| `02` | `01` | `02` | `Bslosh` | `W5` | `DEV_G3_H0_C1_W5_b01_p02_a01` |
| `03` | `02` | `01` | `Bslosh` | `W5` | `DEV_G3_H0_C1_W5_b02_p01_a01` |
| `04` | `02` | `02` | `Bsmooth` | `Bsmooth` | `DEV_G3_H0_C1_Bsmooth_b02_p02_a01` |
| `05` | `03` | `01` | `Bslosh` | `W5` | `DEV_G3_H0_C1_W5_b03_p01_a01` |
| `06` | `03` | `02` | `Bsmooth` | `Bsmooth` | `DEV_G3_H0_C1_Bsmooth_b03_p02_a01` |
| `07` | `04` | `01` | `Bsmooth` | `Bsmooth` | `DEV_G3_H0_C1_Bsmooth_b04_p01_a01` |
| `08` | `04` | `02` | `Bslosh` | `W5` | `DEV_G3_H0_C1_W5_b04_p02_a01` |

该表必须作为 `efficacy_order.csv` 冻结并记录 hash。不用 `for` 循环连续执行；每条之间都必须人工确认回位、对齐、静稳和场地安全。

## 5. RGB 冻结与主指标

### 5.1 标尺与 RealSense

建议使用已完成离线三包验证和在线静态 smoke 的 v2 标尺，而不是早期的旧标尺：

```text
/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_relabel_frozen_v2.yaml
SHA-256 = 7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01
```

相机为 `1920x1080 @ 30 Hz`，预冻结手动参数：

```text
enable_auto_exposure      = false
exposure                  = 166
gain                      = 64
enable_auto_white_balance = false
white_balance             = 4600
```

整机或 RealSense 驱动重启后，必须重放同一参数 sidecar，不得重新选值：

```text
/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/camera_params/apply_realsense_rgb_fixed_params_20260731_203322.sh
```

Row 01 前必须重做几何静态检查：三把标尺顺序、ROI、红色掩码、液深和容器位姿与 v2 YAML 一致。这是几何/QC 检查，不允许根据 G3 方法结果重标尺。

### 5.2 窗口和 `H_vis`

对每条 trial：

```text
window_start = first effective nonzero motion
window_end   = first GOAL_REACHED/first arrival + T_HVIS_TAIL(5.0 s)
h_rgb(t)     = causal rolling_median_5(valid height_max_lcr_mm)
H_vis(t)     = max(0, h_rgb(t))
primary      = P95(H_vis(t)) over valid motion+tail samples
```

`height_max_lcr_mm` 来自 `/liquid/measurement`，已扣除运动前 zero-lock 基线。时序平滑冻结为“当前及之前最近 5 个有效标量的因果 rolling median”；不得使用未来帧，也不得跨越 `0.35 s` 以上的断流继承旧窗口。

timeout 或 method failure 时，零速后继续录制至：

```text
first effective motion + T_MOTION_MAX(42 s) + T_HVIS_TAIL(5 s)
```

不得用 control cycle 或 RGB frame 充当独立统计样本；统计单位始终是配对 block。

### 5.3 RGB 有效性门槛

每条参与 continuous `H_vis` 的 trial 必须同时满足：

- 运动前至少 `2.0 s` 的 `valid=true`、`zero_locked=true`、`status=OK`；
- motion+tail 有效比例至少 `90%`；
- 在线标量实际频率至少 `10 Hz`；
- 最大相邻标量间隔不超过 `0.35 s`；
- publish lag P95 不超过 `0.50 s`，source timestamp future skew 不超过 `0.05 s`；
- primary window 的 tail 完整；
- bag 内 `sensor_msgs/Image`/`CompressedImage` 计数为 `0`。

## 6. `delta_H,dev` 和 G3 PASS 规则

### 6.1 最小实质改善

本方案建议：

```text
delta_H_dev = 0.10 mm
```

`0.10 mm` 必须在 Row 01 前用不包含方法比较的静态/重复性 smoke 证明高于当前 detector 的重复性底噪。如果不成立，只能在任何 G3 数据产生前修改本方案、重建 prereg/hash；不得在看到 Row 01 后修改。

### 6.2 G3 `PASS`

只有同时满足以下条件，最终 `G3_EFFICACY` report 才可写 `PASS`：

1. 8 条 planned rows 全部有完整 attempt/postflight 归档，无提前停止；
2. `4/4` block 都形成 eligible paired outcome；
3. paired mean `mean(Delta_b) >= 0.10 mm`；
4. 至少 `3/4` block 的 `Delta_b > 0`；
5. 删除改善最大的那个 block 后，其余 `Delta_b` 之和仍大于 `0`，且 leave-one-block-out 方向不翻转；
6. 两种方法均 `4/4 GOAL_REACHED`，无 solver failure、tracking-safety stop 或人工轨迹干预；
7. 每条 stage-0 contour P95 `<= 0.05 m`，yaw P95 `<= 0.15 rad`；
8. solver-time P95 `< 25 ms`，无 30 Hz 控制周期的持续超期；
9. W5 运动段 effective source/IMU READY 覆盖率均至少 `98%`，fallback/reset 为 `0`；
10. 全部 RGB 和 image-free postflight 通过，且调参预算和 no-early-stop 规则未被破坏。

同时报告 paired median/均值、原始 4 个 `Delta_b`、描述性区间、relative effect、peak、RMS、tail RMS、completion time、tracking、runtime 和 command intervention。`n_dev=4` 不声称正式显著性，不把 RGB frame 数写成样本数。

### 6.3 `FAIL` / `INCONCLUSIVE`

- 真实 RGB 方向未达门槛：G3 `FAIL`，不得进入正式实验；
- 方法效果好，但 RGB/tail/配对覆盖不足：`INCONCLUSIVE`，不得用成功补录选择性替换原 row；
- 若需改 W5、RGB 参数、窗口或 gate：建立新 development release，整批 8 条从 Row 01 重做，旧批永久保留为 development failure/inconclusive 证据。

## 7. 单条 trial 标准流程

```text
回到同一起点并对齐
→ 检查无其他 /cmd_vel publisher，急停可用
→ 确认容器/液深/相机未变，静稳至少 10 s
→ wrapper 核对 release/source/G2C/G3/path/RGB/config 所有 hash
→ 启动在线 RGB，publish_debug=false
→ 等待至少 2 s clean zero-locked valid measurement
→ 启动 recorder，确认 .bag.active 存在且增长
→ 启动当前 row 的 Bsmooth 或 W5 planner
→ W5 等待 processed-IMU READY 并校验 effective source
→ 发布冻结 H0 路径并自动发送 /cmd_vel
→ first arrival/GOAL_REACHED 后发送零速
→ 继续录制 5 s RGB tail
→ 停止 trial 节点，再次发送零速
→ fail-closed G3 postflight + RGB window QC + summary
```

脚本应在正常退出、`Ctrl+C` 和异常退出时都尝试发送零速，但实车现场仍必须保持急停可用。除立即安全风险外，不应在 bag 完整闭包和 postflight 前手动 `Ctrl+C`。

## 8. 单条 postflight 必须验证

- bag 可读、无 `.bag.active`、时长和 motion/tail 完整；
- release revision、source report、IMU validation、G2C final-candidate、G3 prereg/order/row report 和 outcome-window hash 一致；
- 路径文件/hash、`v_ref`、delay、条件、W5 权重和 effective config 一致；
- `GOAL_REACHED`、有效运动窗口、预运动 RGB 和 5 s tail 完整；
- W5 为 `processed_imu` 真实消费且无 fallback；Bsmooth 明确为不消费液体状态；
- RGB source timestamp、valid/zero-lock/status、coverage/rate/gap/lag/clipping 通过冻结规则；
- solver/tracking/intervention 无安全失败；
- bag 中 raw/compressed/depth/debug image topic 为 `0`；
- 产生 bag、sidecar、postflight、RGB-window-QC 和所有 SHA-256。

只有本条 postflight 明确 `PASS` 才进入下一 planned row。失败产物不得删除；当前没有可签署的 retry verifier，因此不得手写 `a02` authorization 绕过失败。

## 9. 整批产物

建议输出目录：

```text
/home/geist/slosh_bags/real/20260801_spmpc_g3_processed_imu_w5_vs_bsmooth/H0/
```

全部 8 条结束后，按以下顺序归档：

```text
8 条 bag/postflight/RGB-window-QC/hash
→ immutable G3_DATASET_INDEX
→ G3_EFFICACY_REPORT.json
→ G3_EFFICACY_REPORT.json.sha256
```

`G3_EFFICACY_REPORT` 至少包含：

- release/source/IMU-validation/G2C-final-candidate/G3-prereg/dataset-index 的路径与 hash；
- `n_dev=4`、8 个 planned row、attempt 分类、eligible pair 和全部失败计数；
- `T_HVIS_TAIL=5.0`、`G3_OUTCOME_WINDOW_RULE_SHA256` 和 `delta_H_dev=0.10 mm`；
- 4 个 block 的原始 `Delta_b`、聚合效应、区间、方向、leave-one-block-out 和 single-block-dominance；
- success、tracking、runtime、fallback/reset、command intervention、RGB QC 和 no-early-stop；
- 唯一状态：`PASS` / `FAIL` / `INCONCLUSIVE`。

## 10. 做完 G3 后能否直接进入 Stage I

**不能。**

G3 `PASS` 只证明 W5 在独立 development RGB 数据上有值得继续冻结的物理方向。按 [0717_S-MPCC正式实物实验矩阵_先40后88.md](../0717_S-MPCC正式实物实验矩阵_先40后88.md) 的顺序，仍必须完成：

| 后续 gate | 必须关闭的内容 |
| --- | --- |
| G4 | `s_proj`、Z1–Z5、纵/横四相位、完整 horizon、first-action、online-input/zero-state replay reproduction |
| G5 | SmoothMatch 的唯一 `v_ref`、FixedProfile 实物 runner/profile/tracker，以及 comparator fairness report/hash |
| G6 | RGB measurement/analysis freeze、正式样本量、failure/retry 规则、五条件随机表、分析脚本和全部 hash |
| final freeze | 升级 manifest/validator，生成唯一只读 `freeze_manifest.yaml`，且 `status=GO` / `FREEZE_ID` 可验证 |

正确的放行链为：

```text
G3 八条在线 RGB，并且 G3_EFFICACY=PASS
→ G4 PASS
→ G5 PASS
→ 关闭或明确冻结 G2S 可复算证据限制
→ G6 PASS
→ freeze_manifest status=GO
→ 才能开始 Stage I 正式 40 条
```

现场命令见 [20260801_G3_processed-IMU_W5_vs_Bsmooth实验命令.md](./20260801_G3_processed-IMU_W5_vs_Bsmooth实验命令.md)。Row 01 前的剩余步骤是：提交/冻结 release，重跑干净 `VALIDATE_ONLY`，启动基础栈并应用 RealSense 固定参数。
