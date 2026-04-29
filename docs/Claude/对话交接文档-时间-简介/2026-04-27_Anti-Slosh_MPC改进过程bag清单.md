# 2026-04-27 Anti-Slosh MPC 改进过程 bag 清单

本文记录本轮 Anti-Slosh MPC 改进过程中明确使用、分析或作为结论依据的 bag。路径以当前工作机 `/home/a/scout_ws` 环境为准。

## 结论口径

- 事实：下列大部分 bag 已在 `/data/a/slosh_bags` 下存在。
- 事实：`/data/a/slosh_bags/sim/full_all.bag` 在对话中被提到为 NOM，但当前文件系统未找到。
- 待验证：部分旧 bag 只作为调试或中间尝试，不应直接进入最终论文/报告结论。
- 建议：后续 Claude 优先使用“核心对比集合”，不要把中间失败/调试 bag 混入最终均值。

## 核心对比集合

后续分析优先使用这些集合。

### P3B 速度参考裁剪

基线：

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_NOM_run01_182713.bag
```

P3B：

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run05_184720.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run07_205414.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run08_205559.bag
```

结论：P3B 不能稳定降低 `/slosh/height` 或 `modal_energy_norm`，不再作为主线。

### P3C 曲率变化段裁剪

基线：

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_NOM_run01_182713.bag
```

P3C 第一组：

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run01_210633.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run02_210807.bag
```

P3C 第二组：

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run03_212051.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run04_212217.bag
```

结论：P3C 未证明有效，停止沿 `dkappa_only` 方向继续调参。

### P4 控制变化率平滑

基线：

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_NOM_run01_182713.bag
```

`SMOOTH_DOMEGA`：

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_DOMEGA_run01_221549.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_DOMEGA_run02_221924.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_DOMEGA_run03_222227.bag
```

`SMOOTH_CTRL`：

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_CTRL_run01_221728.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_CTRL_run02_222107.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_CTRL_run03_222344.bag
```

结论：单纯提高 `R_domega/R_da` 不能稳定抑晃；平滑条件下实际 `odom_ay_abs_p95`、`odom_kappa_abs_p95`、`odom_wz_rms` 升高。

## 2026-04-22 实物首批 10 包

用途：早期实物基线、红色液体视觉分析、IMU 健康度离线复核、控制侧总结。

目录：

```text
/data/a/slosh_bags/real/0422
```

清单：

```text
/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145046_block1_NOM.bag
/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145304_block1_ISR.bag
/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145652_5.bag
/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145809_5.bag
/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145913_5.bag
/data/a/slosh_bags/real/0422/slosh_Q0_20260422_150105_10.bag
/data/a/slosh_bags/real/0422/slosh_Q0_20260422_150240_10block1_PROP2.bag
/data/a/slosh_bags/real/0422/slosh_Q0_20260422_150442_10block2_Q0.bag
/data/a/slosh_bags/real/0422/slosh_Q0_20260422_150601_5.bag
/data/a/slosh_bags/real/0422/slosh_Q0_20260422_150717_10.bag
```

## 2026-04-24 IMU 标定 bag

用途：IMU lateral acceleration scale / 健康度校验。

```text
/data/a/slosh_bags/real/imu_calib/imu_ay_calib_20260424_142356.bag
```

## 2026-04-24 实物 32 包主实验

用途：实物红色液体视觉高度、`/slosh/height`、轨迹表现对比。注意这批结论显示不同指标可能给出不同判断。

### block1: P3_mixed

```text
/data/a/slosh_bags/real/0424/block1/slosh_Q0_20260424_153110_block1_NOM_P3_mixed.bag
/data/a/slosh_bags/real/0424/block1/slosh_Q5_20260424_153518_block1_FAS_Q5_P3_mixed.bag
/data/a/slosh_bags/real/0424/block1/slosh_Q5_20260424_153906_10.bag
/data/a/slosh_bags/real/0424/block1/slosh_Q5_20260424_154335_5.bag
/data/a/slosh_bags/real/0424/block1/slosh_Q0_20260424_154805_block1_ISR_P3_mixed.bag
```

### block2: P0_straight

```text
/data/a/slosh_bags/real/0424/block2/slosh_Q0_20260424_164614_blockck2_NOM_P0_straight.bag
/data/a/slosh_bags/real/0424/block2/slosh_Q0_20260424_164943_5.bag
/data/a/slosh_bags/real/0424/block2/slosh_Q10_20260424_170003_block2_FAS.bag
/data/a/slosh_bags/real/0424/block2/slosh_Q105_20260424_170255_block2_PROP.bag
/data/a/slosh_bags/real/0424/block2/slosh_Q100_20260424_170537_block2_ISR.bag
```

### block3: P2_s_curve

```text
/data/a/slosh_bags/real/0424/block3/slosh_Q100_20260424_171233_block3_NOM.bag
/data/a/slosh_bags/real/0424/block3/slosh_Q105_20260424_171426_block3_FAS.bag
/data/a/slosh_bags/real/0424/block3/slosh_Q105_20260424_171609_block3_FAS.bag
/data/a/slosh_bags/real/0424/block3/slosh_Q105_20260424_171745_block3_PROP.bag
/data/a/slosh_bags/real/0424/block3/slosh_Q100_20260424_172025_block3_ISR.bag
```

### block4: P2_s_curve

```text
/data/a/slosh_bags/real/0424/block4/slosh_Q100_20260424_182102_block4_NOM.bag
/data/a/slosh_bags/real/0424/block4/slosh_Q105_20260424_182256_block4_FAS.bag
/data/a/slosh_bags/real/0424/block4/slosh_Q1010_20260424_182430_block4_FAS.bag
/data/a/slosh_bags/real/0424/block4/slosh_Q105_20260424_182628_block4_PROP.bag
```

### block5: P2_s_curve

```text
/data/a/slosh_bags/real/0424/block5/slosh_Q0_20260424_183029_block5_NOM.bag
/data/a/slosh_bags/real/0424/block5/slosh_Q0_20260424_183152_5.bag
/data/a/slosh_bags/real/0424/block5/slosh_Q0_20260424_183329_10.bag
/data/a/slosh_bags/real/0424/block5/slosh_Q0_20260424_183456_5.bag
```

### block6: P3_mixed

```text
/data/a/slosh_bags/real/0424/block6/slosh_Q0_20260424_184130_5.bag
/data/a/slosh_bags/real/0424/block6/slosh_Q0_20260424_184327_5.bag
/data/a/slosh_bags/real/0424/block6/slosh_Q0_20260424_184458_10.bag
/data/a/slosh_bags/real/0424/block6/slosh_Q0_20260424_184629_5.bag
```

### block7: P3_mixed + 静止姿态

```text
/data/a/slosh_bags/real/0424/block7/slosh_Q0_20260424_185103_5.bag
/data/a/slosh_bags/real/0424/block7/slosh_Q0_20260424_185225_5.bag
/data/a/slosh_bags/real/0424/block7/slosh_Q0_20260424_185352_10.bag
/data/a/slosh_bags/real/0424/block7/slosh_Q0_20260424_185521_5.bag
/data/a/slosh_bags/real/0424/block7/slosh_Q0_20260424_185829_5.bag
```

## 2026-04-25 仿真 P1/P2 初步实验

用途：旧地图/开阔地图切换后，验证 `Q_slosh_eta_dot`、terminal factor 是否有正向信号。

### test_P1 子目录

```text
/data/a/slosh_bags/sim/20260425/test_P1/20260425_P2_s_curve_NOM_run01_194813.bag
/data/a/slosh_bags/sim/20260425/test_P1/20260425_P2_s_curve_FAS_Q5_run01_195343.bag
/data/a/slosh_bags/sim/20260425/test_P1/20260425_P2_s_curve_FAS_Q5_DOT_run01_195552.bag
/data/a/slosh_bags/sim/20260425/test_P1/20260425_P2_s_curve_FAS_Q10_run01_195910.bag
/data/a/slosh_bags/sim/20260425/test_P1/20260425_P3_mixed_NOM_run01_200103.bag
/data/a/slosh_bags/sim/20260425/test_P1/20260425_P3_mixed_FAS_Q5_run01_200341.bag
/data/a/slosh_bags/sim/20260425/test_P1/20260425_P3_mixed_FAS_Q5_DOT_run01_200501.bag
/data/a/slosh_bags/sim/20260425/test_P1/20260425_P3_mixed_FAS_Q10_run01_200854.bag
/data/a/slosh_bags/sim/20260425/test_P1/20260425_P0_straight_CUSTOM_run01_205314.bag
/data/a/slosh_bags/sim/20260425/test_P1/20260425_P0_straight_FAS_Q5_run01_205458.bag
/data/a/slosh_bags/sim/20260425/test_P1/20260425_P2_s_curve_CUSTOM_run01_205047.bag
```

### P2 terminal 初步组

```text
/data/a/slosh_bags/sim/20260425/20260425_P2_s_curve_NOM_run01_225843.bag
/data/a/slosh_bags/sim/20260425/20260425_P2_s_curve_FAS_Q5_run01_230142.bag
/data/a/slosh_bags/sim/20260425/20260425_P2_s_curve_FAS_Q5_TERM_run01_230658.bag
/data/a/slosh_bags/sim/20260425/20260425_P2_s_curve_FAS_Q5_TERM_run02_235323.bag
```

## 2026-04-26 仿真 P2/P3/P4 改进实验

### 旧地图/非严格同一路径对比

用途：验证 NOM/FAS/FAS_TERM_f3 在近似相同路径下的趋势。注意路径不是严格 replay，同一结论需谨慎引用。

```text
/data/a/slosh_bags/sim/20260426/NOM_P2171958.bag
/data/a/slosh_bags/sim/20260426/NOFAS_Q5_172452.bag
/data/a/slosh_bags/sim/20260426/NOFAS_Q5_TERM_f3172847.bag
```

### 20260426/test1 子目录

用途：旧迷宫地图或单独 launch 录包阶段，主要用于 P2 terminal factor=3 对比。

```text
/data/a/slosh_bags/sim/20260426/test1/20260426_NOM.bag
/data/a/slosh_bags/sim/20260426/test1/20260426_P2_s_curve_FAS_Q5_TERM_run02_000700.bag
/data/a/slosh_bags/sim/20260426/test1/20260426_P2_s_curve_FAS_Q5_TERM_f3_run03.bag
/data/a/slosh_bags/sim/20260426/test1/20260426_FAS_Q5_TERM_f3_run04.bag
/data/a/slosh_bags/sim/20260426/test1/20260426_NOM_run04.bag
```

### 统一终点模板路径 P2_s_curve 基线与 FAS

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_NOM_run01_182713.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_FAS_Q5_run01_182932.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_FAS_Q5_SPEED_CAP_run01_183135.bag
```

### P3B SPEED_CAP 全部已录 bag

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run01_182440.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run02_183446.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run03_183732.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run04_184009.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run05_184720.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run06_185119.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run07_205414.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run08_205559.bag
```

### P3C DKAPPA_CAP 全部已录 bag

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run01_210633.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run02_210807.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run03_212051.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run04_212217.bag
```

### P4 SMOOTH_DOMEGA / SMOOTH_CTRL 全部已录 bag

```text
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_DOMEGA_run01_221549.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_DOMEGA_run02_221924.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_DOMEGA_run03_222227.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_CTRL_run01_221728.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_CTRL_run02_222107.bag
/data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SMOOTH_CTRL_run03_222344.bag
```

## 提到但当前未找到

```text
/data/a/slosh_bags/sim/full_all.bag
```

说明：对话中用户说明它是 NOM，但当前 `find /data/a/slosh_bags/sim -name full_all.bag` 未找到。不要把它作为后续自动脚本输入，除非重新确认路径。

## 相关分析输出

这些不是 bag，但后续复盘有用。

```text
/tmp/p2_p4_lag_20260426.csv
/tmp/p2_p4_tracking_20260427.csv
```

注意：`/tmp` 不是长期保存位置。如需长期保留，应复制到 `/data/a` 或 `docs/Claude` 下。

## 后续使用建议

- 若验证 P3B/P3C 失败结论：只用 20260426 P2_s_curve 的 NOM、SPEED_CAP run05/07/08、DKAPPA_CAP run01-04。
- 若验证 P4 失败结论：只用 20260426 P2_s_curve 的 NOM、SMOOTH_DOMEGA run01-03、SMOOTH_CTRL run01-03。
- 若验证实物视觉 vs `/slosh/height` 分歧：用 20260424 实物 block1-block7。
- 若验证 IMU 标定：用 `imu_ay_calib_20260424_142356.bag`，不要混入控制实验 bag。
- 若做最终报告：避免混用 20260425 `test_P1` 和 20260426 统一终点模板路径 bag，它们不是同一实验口径。
