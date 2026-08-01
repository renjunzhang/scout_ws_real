# G2C processed-IMU W2/W5 实验命令

> 冻结日期：2026-08-01
>
> 范围：只用于 G2C development，不计入正式 40/64/88。
>
> 当前状态：G2C 可执行；G3 和正式 Stage 仍为 NO-GO。

## 1. 上游决策边界

G2S 原计划 4 条，操作员在完成 `u01..u03` 后决定停止，不采 `u04`。原 image-free 三包的在线 RGB center 列在运动段裁剪，正式 postflight 未通过；随后又采集了三条同路径 raw-RGB diagnostic bag，并用冻结的 v2 标尺和 HSV 对 raw RGB 重新离线提取。

可重复分析结果：

| Trial | RGB 对齐样本 | odom MAE | processed-IMU MAE | IMU 改善 | odom/IMU coverage |
| --- | ---: | ---: | ---: | ---: | ---: |
| u01 | 192 | 0.3934 mm | 0.2428 mm | 38.28% | 1.000 / 1.000 |
| u02 | 191 | 0.3355 mm | 0.2335 mm | 30.38% | 1.000 / 1.000 |
| u03 | 199 | 0.3679 mm | 0.2413 mm | 34.40% | 1.000 / 1.000 |
| 平均 | — | 0.3656 mm | 0.2392 mm | **34.56%** | 1.000 / 1.000 |

因此：

- 不宣称 G2S 正式 PASS；
- raw-RGB 重标尺结果中，processed-IMU 在 `3/3` trial 都优于 odom，平均改善 `34.56%`；
- 只放行 processed-IMU source 的 G2C development；
- G3 前仍需另行冻结完整正式 RGB gate，不能把三包 development 报告升格为正式四包 PASS。

决策证据：

```text
/home/geist/slosh_bags/real/20260731_spmpc_g2s_rgb_diagnostic/analysis/G2S_RAW_RGB_THREE_TRIAL_DEVELOPMENT_REPORT.json
SHA-256 = 36db4d12948f98e4ef3580bc52367cdef14d8440d80a8afbb32182b704ee5442

/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/analysis/PROCESSED_IMU_G2C_DEVELOPMENT_SMOKE_REPORT.json
SHA-256 = 5635d31e0221bdc54a00ee9eb11043515112a3326eb7945d10d0ffb5e13cc5d3
```

机器报告由下列只读命令从三包和三个离线 CSV 重算；正式 G2C wrapper 会逐项核对报告 hash、三包 hash、`3/3` 同向、覆盖率和 scope：

```bash
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/analyze_spmpc_g2s_raw_rgb_three_trial.sh
```

人类可读说明见 [20260731_G2S重标尺与IMU_odom输入源三包预分析.md](../../实物对比试验分析/20260731_G2S重标尺与IMU_odom输入源三包预分析.md)。

## 2. 实验结构

G2C 使用 2 个配对 block，共 4 条：

| `G2C_ROW` | 方法 | `w_slosh` | block |
| --- | --- | ---: | --- |
| 01 | W2 | 2 | 01 |
| 02 | W5 | 5 | 01 |
| 03 | W5 | 5 | 02 |
| 04 | W2 | 2 | 02 |

顺序固定为 `W2 → W5 → W5 → W2`（ABBA）。不要使用 `for` 循环。

冻结值：

```text
source=processed_imu
path=/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json
v_ref=0.20 m/s
delay=fixed_closed_loop 0.15/0.22 s
record_sec=90
RGB/online-liquid=false
```

G2C 不用 RGB，不需要重新冻结相机。

## 3. 基础栈

当前 `launch_real_sensors_stack.sh` 已运行时不要重复启动。只读检查：

```bash
pgrep -af launch_real_sensors_stack.sh
rostopic info /cmd_vel
```

trial 开始前 `/cmd_vel` 不应有其他 planner publisher。

## 4. Dry-run（不动车）

wrapper 会拒绝有 tracked diff/staged diff 的工作树，并确认 planner runtime 与 processed-IMU smoke 基线 `01700fd` 没有差异；Row 01 创建的 prereg 会记录实际冻结提交，后续三条必须保持同一提交。

```bash
DATE=20260801 VALIDATE_ONLY=true G2C_ROW=01 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2c_processed_imu_w2w5_trial.sh
```

## 5. 逐条执行

Row 01 / W2：

```bash
DATE=20260801 ARM_MOTION=YES G2C_ROW=01 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2c_processed_imu_w2w5_trial.sh
```

Row 02 / W5：

```bash
DATE=20260801 ARM_MOTION=YES G2C_ROW=02 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2c_processed_imu_w2w5_trial.sh
```

Row 03 / W5：

```bash
DATE=20260801 ARM_MOTION=YES G2C_ROW=03 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2c_processed_imu_w2w5_trial.sh
```

Row 04 / W2：

```bash
DATE=20260801 ARM_MOTION=YES G2C_ROW=04 \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g2c_processed_imu_w2w5_trial.sh
```

## 6. 单条自动行为

```text
核对三包 raw-RGB source report/smoke/path/release revision
→ 启动 recorder 和 processed-IMU planner
→ 小车静止，等待 IMU READY
→ 释放 H0 路径并发布 /cmd_vel
→ 90 s 边界结束
→ 发布零速并停止 planner/path
→ G2C postflight
→ 生成 trial summary
```

postflight 必须确认：

- 运动段 effective source 为 processed-IMU 且覆盖率至少 98%；
- IMU READY 覆盖率至少 98%；
- 无 fallback、无 reset epoch；
- effective `w_slosh` 与 W2/W5 匹配；
- `GOAL_REACHED`；
- bag 内无任何图像流。

只有上一条 postflight `PASS`，wrapper 才允许执行下一条。

## 7. 两条之间

```text
确认脚本退出且零速
→ 小车回到同一起点
→ 重新对齐
→ 等待液体完全静稳
→ 执行下一条
```

4 条全部 PASS 后停止，先比较 W2/W5 的安全、tracking、runtime、Z2/Z3 horizon 与 first action，不直接进入 G3。

## 8. 输出目录

```text
/home/geist/slosh_bags/real/20260801_spmpc_g2c_processed_imu_w2w5/H0/
```
