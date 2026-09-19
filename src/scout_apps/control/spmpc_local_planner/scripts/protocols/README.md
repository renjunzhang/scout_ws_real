# 专项与历史实验协议

[返回总索引](../README.md) · [常用实物入口](../real/README.md) · [分析索引](../analysis/README.md)

这里按实验协议保存原入口，便于重现旧数据。场地、模型、标定和权重可能已冻结在脚本中；复用前核对所属实验，不直接套用到当前两层主线。`trial` 通常会在解锁后运动，具体以参数段为准。

## G 系列：来源、权重、配对

| 协议 | 脚本 |
| --- | --- |
| G2S 来源选择 | [run_spmpc_g2s_h0s_source_selection_trial.sh](g_series/run_spmpc_g2s_h0s_source_selection_trial.sh)、[来源分析](g_series/analyze_spmpc_g2s_source_selection.sh)、[三包原始 RGB 分析](g_series/analyze_spmpc_g2s_raw_rgb_three_trial.sh) |
| G2C processed-IMU | [run_spmpc_g2c_processed_imu_w2w5_trial.sh](g_series/run_spmpc_g2c_processed_imu_w2w5_trial.sh) |
| G3 W5/Smooth | [run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh](g_series/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh)、[失败行后续跑](g_series/continue_spmpc_g3_after_failed_row.sh) |
| G3R/G3R2 权重筛选 | [G3R](g_series/run_spmpc_g3r_weight_screen_trial.sh)、[G3R2](g_series/run_spmpc_g3r2_weight_screen_trial.sh) |
| G3R2 配对/车体 smoke | [配对确认](g_series/run_spmpc_g3r2_paired_confirmation_trial.sh)、[车体 smoke](g_series/run_spmpc_g3r2_robot_only_smoke_trial.sh) |
| G4 回放 | [run_spmpc_g4_from_g3.sh](g_series/run_spmpc_g4_from_g3.sh) |
| G5 对手准备/最小试验 | [prepare_spmpc_g5_comparators.sh](g_series/prepare_spmpc_g5_comparators.sh)、[run_spmpc_g5_minimal_trial.sh](g_series/run_spmpc_g5_minimal_trial.sh) |

G3 相机准备放在 [rgb/](../rgb/README.md)，不属于通用相机自动配置。

## I0/O0、显式执行器与 ABBA

| 协议 | 脚本 |
| --- | --- |
| I0 fixed / short100 | [fixed ABBA](comparisons/run_spmpc_i0_failclosed_fixed_abba_trial.sh)、[short100 ABBA](comparisons/run_spmpc_i0_failclosed_fixed_short100_abba_trial.sh) |
| I0 显式执行器 | [explicit ABBA](comparisons/run_spmpc_i0_failclosed_explicit_actuator_abba_trial.sh)、[WS1/WA03 ABBA](comparisons/run_spmpc_i0_failclosed_explicit_actuator_ws1_wa03_abba_trial.sh) |
| RGB ABBA | [run_spmpc_ws1_wa03_rgb_abba.sh](comparisons/run_spmpc_ws1_wa03_rgb_abba.sh) |
| O0/L22 | [B0/Bslosh](comparisons/run_spmpc_o0_l22_b0_bslosh_trial.sh)、[Smooth/Ours](comparisons/run_spmpc_o0_l22_bsmooth_bours_trial.sh) |
| 短时域/权重 | [short horizon](comparisons/run_spmpc_short_horizon_matched_trial.sh)、[weight smoke](comparisons/run_spmpc_weight_smoke.sh) |
| Full-DA smoke | [run_spmpc_full_da_smoke.sh](comparisons/run_spmpc_full_da_smoke.sh) |

当前常用 C03 wrapper 在 [real/](../real/run_spmpc_ablation_smoke.sh)，其共用 runtime 引擎在 [lib/](../lib/run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh)。

## 早期开发 smoke

| 脚本 | 用途 |
| --- | --- |
| [phase3_smoke.sh](smoke/phase3_smoke.sh)、[phase4_fixed_path_run.sh](smoke/phase4_fixed_path_run.sh) | 早期阶段验证/固定路径运行 |
| [compare_b0_bslosh_smoke.sh](smoke/compare_b0_bslosh_smoke.sh) | B0/Bslosh 仿真对照 |
| [verify_continuous_smoke.sh](smoke/verify_continuous_smoke.sh) | continuous MPCC 仿真闭环 |
| [sweep_w_slosh.sh](smoke/sweep_w_slosh.sh) | 单权重运行；[离线汇总](../analysis/sweep_w_slosh_summary.py) |

这些脚本会启动控制/发布路径，应在各自预期环境中使用；新主线模型验证入口见 [trajectory/](../trajectory/README.md)。

## 回放与冻结

- [run_spmpc_slosh_nowcast_replay.sh](run_spmpc_slosh_nowcast_replay.sh)：指定历史 nowcast ROS 回放及验收。
- [prepare_spmpc_g6_freeze.py](prepare_spmpc_g6_freeze.py)：从前置产物准备 G6 冻结材料。
- [validate_spmpc_formal_freeze.py](validate_spmpc_formal_freeze.py)：校验正式冻结清单、版本与哈希。

完整运行参数及当前无输出回放边界见 [USAGE.md](../docs/USAGE.md)。
