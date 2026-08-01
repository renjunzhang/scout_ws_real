# G3 processed-IMU W5 vs Bsmooth 实验命令

> 日期：2026-08-01
>
> 范围：G3 development，共 4 个配对 block / 8 条 trial，不计入正式 `40/64/88`。
>
> 单包录制上限：**70 s**。脚本冻结 `record-to-first-motion <= 23 s`，完整运动后必须保留 `5 s` RGB tail。
>
> 数据政策：RealSense 在线供 detector 使用，bag 只录 `/liquid/measurement` 派生标量，禁止 raw/compressed/depth/debug 图像流。

## 1. 当前证据边界

本 wrapper 前瞻冻结以下 development 决策：

```text
source    = processed_imu
candidate = W5 / w_slosh=5
comparator= Bsmooth
```

它会机器核对：

- G2S 三包 report 的 `3/3` 同向、平均 `34.56%` 改善和 hash；
- G2C 4 条 postflight `PASS`、100% processed-IMU/READY、无 fallback；
- G2C planner/RGB runtime 到 G3 release 没有改变。

边界仍保留：G2S report 是 `G2C_DEVELOPMENT_ONLY`，不会被 wrapper 伪装成“正式四包 G2S PASS”。这批 G3 先作 development 物理 efficacy gate，正式 Stage 仍为 `NO-GO`。

## 2. 终端 A：基础传感器栈

先检查：

```bash
pgrep -af launch_real_sensors_stack.sh
```

如果没有运行，启动一次：

```bash
bash /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

这个终端在全部 8 条结束前保持运行。

## 3. 终端 B：冻结 RealSense

### 3.1 只检查固定文件/hash，不改相机

```bash
VALIDATE_ONLY=true \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/prepare_spmpc_g3_realsense.sh
```

### 3.2 应用并核对冻结参数

```bash
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/prepare_spmpc_g3_realsense.sh
```

脚本固定并验证：

```text
1920x1080 @ 30 Hz
enable_auto_exposure=false
exposure=166
gain=64
enable_auto_white_balance=false
white_balance=4600
calibration=red_3ruler_g2s_20260731_relabel_frozen_v2.yaml
calibration SHA-256=7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01
```

基础栈、RealSense 驱动或整机重启后，重新执行同一条命令，不得重选曝光/增益/白平衡。

## 4. G3 dry-run（不动车）

代码已提交/冻结后，使用：

```bash
DATE=20260801 VALIDATE_ONLY=true G3_ROW=01 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh
```

dry-run 会核对 source/G2C/path/RGB/runtime 证据并打印：

```text
prereg SHA256
order SHA256
window SHA256
online RGB config SHA256
```

dry-run 不录 bag、不启动 planner、不发布 `/cmd_vel`。

## 5. 8 条实车命令

不要使用 `for` 循环。每条脚本退出且 postflight `PASS` 后，才能回位、对齐、等待液体静稳并执行下一条。

### Row 01 — Block 01 / Bsmooth

```bash
DATE=20260801 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES G3_ROW=01 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh
```

### Row 02 — Block 01 / W5

```bash
DATE=20260801 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES G3_ROW=02 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh
```

### Row 03 — Block 02 / W5

```bash
DATE=20260801 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES G3_ROW=03 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh
```

### Row 04 — Block 02 / Bsmooth

```bash
DATE=20260801 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES G3_ROW=04 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh
```

### Row 05 — Block 03 / W5

```bash
DATE=20260801 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES G3_ROW=05 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh
```

### Row 06 — Block 03 / Bsmooth

```bash
DATE=20260801 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES G3_ROW=06 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh
```

### Row 07 — Block 04 / Bsmooth

```bash
DATE=20260801 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES G3_ROW=07 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh
```

### Row 08 — Block 04 / W5

```bash
DATE=20260801 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES G3_ROW=08 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh
```

## 6. 单条脚本的自动行为

```text
检查上游证据、release、路径、顺序和 RGB hash
→ 应用/核对 RealSense 手动参数
→ 启动 v2 在线 RGB，publish_debug=false
→ 等待 20 个 clean zero-locked RGB 标量
→ 启动 70 s recorder
→ 启动 planner，小车保持静止建立 IMU bias
→ 最多等待 12 s IMU READY
→ 释放冻结 H0 路径并发送 /cmd_vel
→ GOAL_REACHED 后零速，继续记录 RGB tail
→ 70 s 边界自动停 planner/path，再次发送零速
→ G3 postflight + trial summary
```

实测 G2C 的录包到首次运动延迟约 `19.6–19.9 s`，运动约 `35.7–36.8 s`。因此 70 s 边界下预计仍有约 `13 s` 的 post-motion 空间；postflight 强制要求：

```text
record-to-first-motion <= 23 s
GOAL_REACHED
first-arrival 后在线 RGB tail >= 5 s
```

如果前置等待超时，脚本会停止，不允许用不完整 tail 的 bag 进入配对分析。

## 7. 每条之间

```text
确认脚本退出且小车零速
→ 保留 bag/postflight/log，不删失败产物
→ 遥控或人工将小车回到同一起点
→ 按起点姿态重新对齐
→ 等待液体完全静稳，至少 10 s
→ 检查相机/容器/液深未变
→ 执行下一 G3_ROW
```

## 8. Row 08 后的自动分析

Row 08 postflight `PASS` 后，wrapper 自动运行四 block analyzer，生成：

```text
/home/geist/slosh_bags/real/20260801_spmpc_g3_processed_imu_w5_vs_bsmooth/H0/G3_EFFICACY_REPORT.json
/home/geist/slosh_bags/real/20260801_spmpc_g3_processed_imu_w5_vs_bsmooth/H0/G3_EFFICACY_BLOCK_METRICS.csv
```

G3 只有在以下条件同时成立时才 `PASS`：

- `4/4` 配对完整；
- `mean(H_vis,p95_Bsmooth - H_vis,p95_W5) >= 0.10 mm`；
- 至少 `3/4` block 同向；
- 不由单个 block 独占，leave-one-block-out 不翻转；
- success/tracking/runtime/IMU/RGB/image-free 全部通过。

G3 `PASS` 后仍先进入 G4/G5/G6，不直接启动 Stage I。
