# 开发模块与回归索引

[返回总索引](../README.md) · [native 构建与验证](../../../../../../test/native/README.md)

## planning

上层规划实现，由 trajectory 入口调用。

| 用途 | 文件 |
| --- | --- |
| 任务、区域与策略 | [task.py](../planning/task.py)、[guide.py](../planning/guide.py)、[liquid_policy.py](../planning/liquid_policy.py)、[terminal_speed.py](../planning/terminal_speed.py) |
| 求解、初值与过程 | [optimizer.py](../planning/optimizer.py)、[warm_start.py](../planning/warm_start.py)、[process_runner.py](../planning/process_runner.py)、[diagnostics.py](../planning/diagnostics.py) |
| 传播、指标与停车 | [validation.py](../planning/validation.py)、[metrics.py](../planning/metrics.py)、[stopping.py](../planning/stopping.py) |
| 检查点与剩余任务 | [checkpoint.py](../planning/checkpoint.py)、[suffix.py](../planning/suffix.py)、[__init__.py](../planning/__init__.py) |

## acados

生成器会写生成物；模型变化后需按 native README 重建，B0/slosh 生成串行执行。

| 用途 | 文件 |
| --- | --- |
| 生成入口 | [generate_spmpc_acados.py](../acados/generate_spmpc_acados.py)、[generate_slosh_kernel.py](../acados/generate_slosh_kernel.py)、[generate_cost_kernel.py](../acados/generate_cost_kernel.py) |
| 模型、代价与约束 | [spmpc_acados_model.py](../acados/spmpc_acados_model.py)、[spmpc_acados_cost.py](../acados/spmpc_acados_cost.py)、[spmpc_acados_constraints.py](../acados/spmpc_acados_constraints.py) |
| 共用动力学与契约 | [actual_motion_kernel.py](../acados/actual_motion_kernel.py)、[slosh_kernel.py](../acados/slosh_kernel.py)、[planning_terms.py](../acados/planning_terms.py)、[model_contract.py](../acados/model_contract.py) |
| 依赖 | [requirements.txt](../acados/requirements.txt) |

## lib

共用引擎/配置，通常通过 real 或 protocols 的入口调用。runtime 引擎可以在解锁后启动实物运行；另外三个文件由 wrapper source。

| 用途 | 文件 |
| --- | --- |
| 显式执行器 runtime 引擎 | [run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh](../lib/run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh) |
| ABBA 引擎 | [run_spmpc_i0_failclosed_fixed_abba_engine.sh](../lib/run_spmpc_i0_failclosed_fixed_abba_engine.sh) |
| 消融参数与场景 | [spmpc_ablation_profile.sh](../lib/spmpc_ablation_profile.sh)、[spmpc_ablation_scene.sh](../lib/spmpc_ablation_scene.sh) |

## experiments

新主线 recorder 的配置合并、冻结和现场核对逻辑。

| 用途 | 文件 |
| --- | --- |
| 录制契约 | [recording_contract.py](../experiments/recording_contract.py) |
| 对应回归 | [test_recording_contract.py](../experiments/test_recording_contract.py) |

## tests

现有测试按下面主题查找。部分测试依赖 ROS1 及与源码一致的已生成消息，数值测试还依赖 CasADi；测试结果按实际依赖和执行范围记录。

### 任务与上层规划

- [test_liquid_policy.py](../tests/test_liquid_policy.py)、[test_planning_guide.py](../tests/test_planning_guide.py)、[test_planning_metrics.py](../tests/test_planning_metrics.py)
- [test_planning_stopping.py](../tests/test_planning_stopping.py)、[test_terminal_speed_policy.py](../tests/test_terminal_speed_policy.py)、[test_trajectory_checkpoint.py](../tests/test_trajectory_checkpoint.py)
- [test_trajectory_planning_validation.py](../tests/test_trajectory_planning_validation.py)、[test_trajectory_suffix.py](../tests/test_trajectory_suffix.py)、[test_transport_search.py](../tests/test_transport_search.py)
- [test_transport_tail_validation.py](../tests/test_transport_tail_validation.py)

### 模型与 OCP

- [test_ablation_switches.py](../tests/test_ablation_switches.py)、[test_explicit_actuator_model.py](../tests/test_explicit_actuator_model.py)、[test_ocp_cost_contract.py](../tests/test_ocp_cost_contract.py)
- [test_ocp_snapshot_contract.py](../tests/test_ocp_snapshot_contract.py)、[test_prediction_liquid_evaluation.py](../tests/test_prediction_liquid_evaluation.py)、[test_rotating_slosh_kernel.py](../tests/test_rotating_slosh_kernel.py)

### 动捕与执行链

- [test_continuity_spectrum_core.py](../tests/test_continuity_spectrum_core.py)、[test_mocap_execution_chain_tools.py](../tests/test_mocap_execution_chain_tools.py)、[test_mocap_field_map_tools.py](../tests/test_mocap_field_map_tools.py)
- [test_mocap_slosh_delay_diagnostic_contract.py](../tests/test_mocap_slosh_delay_diagnostic_contract.py)、[test_robot_state_prediction.py](../tests/test_robot_state_prediction.py)、[test_same_bag_delay_core.py](../tests/test_same_bag_delay_core.py)
- [test_velocity_continuity_core.py](../tests/test_velocity_continuity_core.py)、[test_velocity_step_response_core.py](../tests/test_velocity_step_response_core.py)

### RGB、录制与实物汇总

- [test_explicit_actuator_runtime_smoke.py](../tests/test_explicit_actuator_runtime_smoke.py)、[test_plot_spmpc_full_da_diagnostics.py](../tests/test_plot_spmpc_full_da_diagnostics.py)、[test_rgb_static_diagnostic.py](../tests/test_rgb_static_diagnostic.py)
- [test_spmpc_ablation_smoke.py](../tests/test_spmpc_ablation_smoke.py)、[test_spmpc_comparison_recording.py](../tests/test_spmpc_comparison_recording.py)、[test_summarize_spmpc_real_trial.py](../tests/test_summarize_spmpc_real_trial.py)

### 历史试验与分析协议

- [test_analyze_g2s_raw_rgb_three_trial.py](../tests/test_analyze_g2s_raw_rgb_three_trial.py)、[test_analyze_g2s_source_selection.py](../tests/test_analyze_g2s_source_selection.py)、[test_g3_online_rgb_gate.py](../tests/test_g3_online_rgb_gate.py)
- [test_g3r_release.py](../tests/test_g3r_release.py)、[test_g4_replay.py](../tests/test_g4_replay.py)、[test_g5_comparators.py](../tests/test_g5_comparators.py)
- [test_horizon_future_alignment.py](../tests/test_horizon_future_alignment.py)、[test_i0_failclosed_fixed_abba_contract.py](../tests/test_i0_failclosed_fixed_abba_contract.py)、[test_i0_failclosed_fixed_abba_rgb_analysis.py](../tests/test_i0_failclosed_fixed_abba_rgb_analysis.py)
- [test_i0_failclosed_fixed_short100_runtime_gate.py](../tests/test_i0_failclosed_fixed_short100_runtime_gate.py)、[test_internal_slosh_freeze.py](../tests/test_internal_slosh_freeze.py)、[test_internal_slosh_metrics.py](../tests/test_internal_slosh_metrics.py)
- [test_short_horizon_matched_release.py](../tests/test_short_horizon_matched_release.py)、[test_slosh_nowcast_analysis_core.py](../tests/test_slosh_nowcast_analysis_core.py)、[test_validate_spmpc_formal_freeze.py](../tests/test_validate_spmpc_formal_freeze.py)

### 辅助文件

- [planning_fixture.py](../tests/planning_fixture.py)、[rotating_slosh_bridge.cpp](../tests/rotating_slosh_bridge.cpp)

## 路径维护

入口文件放 real/mocap/rgb/trajectory/protocols，根目录只保留 README 和分类目录。Shell 入口从真实文件定位共享 scripts 根目录，跨目录调用写明分类路径；trajectory 的 Python 入口从该根目录导入 planning。analysis/acados/planning 内部模块保留相对布局。

CMake 整体安装 scripts 树，无需逐个列举入口。改动目录时检查调用路径、共享模块导入、录制契约及索引链接；源码冻结须重新记录版本与哈希。

本机 devel 的旧消息缺少 `zero_liquid_initial_state` 字段；依赖该字段的回归需使用与当前源码一致的生成消息。
