# 分析脚本索引

[返回总索引](../README.md) · [实物入口](../real/README.md) · [RGB 流程](../rgb/README.md)

本目录的脚本主要读取 bag、报告或冻结产物，按用途分组如下。命令中的路径、schema 和冻结协议版本必须与输入产物匹配。

## 常用实物分析

- [plot_spmpc_full_da_diagnostics.py](plot_spmpc_full_da_diagnostics.py)：整包时域六图，先看命令、实际运动、预测和液体链。
- [analyze_robot_state_prediction.py](analyze_robot_state_prediction.py)：核对车体初态及未来运动，需本场地 reference 配置。
- [analyze_internal_slosh_pair.py](analyze_internal_slosh_pair.py)、[freeze_internal_slosh_evaluation.py](freeze_internal_slosh_evaluation.py)：内部液体指标、Full/Smooth 配对与评价冻结；不是 RGB 真值。

## 执行器/动捕

- [analyze_mocap_execution_chain.py](analyze_mocap_execution_chain.py)、[validate_mocap_execution_chain_bag.py](validate_mocap_execution_chain_bag.py)、[validate_mocap_b0_delay_mode_summary.py](validate_mocap_b0_delay_mode_summary.py)：动捕执行链与 B0 延迟模式验收。
- [analyze_mocap_velocity_continuity.py](analyze_mocap_velocity_continuity.py)、[analyze_mocap_velocity_step.py](analyze_mocap_velocity_step.py)、[summarize_mocap_velocity_step_matrix.py](summarize_mocap_velocity_step_matrix.py)：动捕速度连续性、阶跃及矩阵汇总。
- [publish_mocap_velocity_continuity.py](publish_mocap_velocity_continuity.py)、[publish_mocap_velocity_step.py](publish_mocap_velocity_step.py)：会直接向 ROS 发布速度，属于在线/执行辅助工具，不是离线分析脚本。
- [analyze_same_bag_actuator_delay.py](analyze_same_bag_actuator_delay.py)、[estimate_cmd_odom_delay.py](estimate_cmd_odom_delay.py)、[validate_explicit_actuator_runtime_smoke.py](validate_explicit_actuator_runtime_smoke.py)：执行器延迟和运行时冒烟检查。
- [validate_mocap_field_localization.py](validate_mocap_field_localization.py)、[validate_mocap_field_map.py](validate_mocap_field_map.py)、[validate_mocap_s_path.py](validate_mocap_s_path.py)：动捕场地、地图和路径约束验证。

## 液体状态与预测

- [analyze_internal_slosh_pair.py](analyze_internal_slosh_pair.py)、[freeze_internal_slosh_evaluation.py](freeze_internal_slosh_evaluation.py)、[slosh_nowcast_analysis_core.py](slosh_nowcast_analysis_core.py)：内部液体状态评估与冻结。
- [analyze_slosh_nowcast_same_bag.py](analyze_slosh_nowcast_same_bag.py)、[validate_slosh_nowcast_replay.py](validate_slosh_nowcast_replay.py)、[validate_slosh_nowcast_shadow_bag.py](validate_slosh_nowcast_shadow_bag.py)：nowcast 同 bag、回放和 shadow 验证。
- [horizon_liquid_replay.py](horizon_liquid_replay.py)、[rotating_liquid_replay.py](rotating_liquid_replay.py)、[liquid_cost_window_contract.py](liquid_cost_window_contract.py)：液体预测回放与成本窗口协议。
- [analyze_robot_state_prediction.py](analyze_robot_state_prediction.py)、[robot_state_prediction_core.py](robot_state_prediction_core.py)：机器人状态预测及其核心模块。
- [analyze_velocity_continuity_spectrum.py](analyze_velocity_continuity_spectrum.py)、[continuity_spectrum_core.py](continuity_spectrum_core.py)、[velocity_continuity_core.py](velocity_continuity_core.py)、[velocity_step_response_core.py](velocity_step_response_core.py)：速度连续性频谱与阶跃响应核心。

内部 slosh 指标是模型/状态评估，不等同于 RGB 液面测量；输入 schema、报告格式和冻结协议版本必须匹配。

## RGB 及历史实验

- [analyze_i0_failclosed_fixed_abba_rgb.py](analyze_i0_failclosed_fixed_abba_rgb.py)、[validate_i0_failclosed_fixed_abba_bag.py](validate_i0_failclosed_fixed_abba_bag.py)：I0 fail-closed ABBA 的 RGB 与 bag 验证。
- [validate_g2c_processed_imu_trial.py](validate_g2c_processed_imu_trial.py)、[validate_g2s_paired_trial.py](validate_g2s_paired_trial.py)、[validate_g3_online_rgb_trial.py](validate_g3_online_rgb_trial.py)：G2C/G2S/G3 试验验收。
- [validate_g5_minimal_trial.py](validate_g5_minimal_trial.py)、[validate_short_horizon_matched_bag.py](validate_short_horizon_matched_bag.py)、[validate_spmpc_comparison_recording.py](validate_spmpc_comparison_recording.py)：历史比较、短时域和录制物验收。
- [g4_replay_from_g3.py](g4_replay_from_g3.py)、[g4_replay_config.yaml](g4_replay_config.yaml)、[i0_failclosed_fixed_abba_profile.py](i0_failclosed_fixed_abba_profile.py)：G4 回放和 I0 历史 profile。

## 验收

- [validate_spmpc_ablation_smoke.py](validate_spmpc_ablation_smoke.py)、[plot_spmpc_full_da_diagnostics.py](plot_spmpc_full_da_diagnostics.py)：消融冒烟与完整 DA 诊断图。
- [audit_ocp_cost_and_anticreep.py](audit_ocp_cost_and_anticreep.py)、[ocp_snapshot_contract.py](ocp_snapshot_contract.py)：OCP 成本、反爬行和 snapshot 合同。
- [validate_mocap_execution_chain_bag.py](validate_mocap_execution_chain_bag.py)、[validate_mocap_b0_delay_mode_summary.py](validate_mocap_b0_delay_mode_summary.py)：动捕执行链产物及延迟摘要验收。

## 内部模块

- [same_bag_delay_core.py](same_bag_delay_core.py)、[robot_state_prediction_core.py](robot_state_prediction_core.py)、[slosh_nowcast_analysis_core.py](slosh_nowcast_analysis_core.py)：供上层分析脚本复用的核心逻辑。
- [ocp_snapshot_contract.py](ocp_snapshot_contract.py)、[liquid_cost_window_contract.py](liquid_cost_window_contract.py)、[continuity_spectrum_core.py](continuity_spectrum_core.py)、[velocity_continuity_core.py](velocity_continuity_core.py)、[velocity_step_response_core.py](velocity_step_response_core.py)：协议、窗口和连续性内部模块。


## 历史批次分析

- [analyze_g2s_source_selection.py](analyze_g2s_source_selection.py)、[analyze_g2s_raw_rgb_three_trial.py](analyze_g2s_raw_rgb_three_trial.py)：G2S 来源选择与三次 RGB 试验汇总。
- [analyze_g3_w5_vs_bsmooth.py](analyze_g3_w5_vs_bsmooth.py)、[analyze_g3r_weight_screen.py](analyze_g3r_weight_screen.py)、[analyze_g3r2_weight_screen.py](analyze_g3r2_weight_screen.py)、[analyze_g3r2_paired_confirmation.py](analyze_g3r2_paired_confirmation.py)：G3/G3R 权重筛选和配对确认。
- [analyze_g5_minimal.py](analyze_g5_minimal.py)、[prepare_g5_comparators.py](prepare_g5_comparators.py)、[g5_comparator_config.yaml](g5_comparator_config.yaml)：G5 最小试验及比较器准备。
- [analyze_g3_delay_state_alignment.py](analyze_g3_delay_state_alignment.py)、[analyze_horizon_future_liquid_alignment.py](analyze_horizon_future_liquid_alignment.py)、[summarize_horizon_future_liquid_alignment.py](summarize_horizon_future_liquid_alignment.py)：延迟、状态与预测时域对齐。
- [analyze_b0_bslosh_compare.py](analyze_b0_bslosh_compare.py)、[analyze_spmpc_delay_phase.py](analyze_spmpc_delay_phase.py)、[check_omega_smoke.py](check_omega_smoke.py)、[sweep_w_slosh_summary.py](sweep_w_slosh_summary.py)：B0/B-slosh、延迟相位、omega 冒烟和权重扫描汇总。
