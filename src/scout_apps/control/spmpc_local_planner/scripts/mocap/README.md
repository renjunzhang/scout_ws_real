# 场地、动捕与执行器辨识

[返回总索引](../README.md) · [实物常用入口](../real/README.md)

这里集中检查实车运动与仿真模型的差异。Cartographer 提供控制定位，NOKOV 用作独立测量；现场顺序见[动捕场地 SOP](../../../../../../docs/实物实验注意事项/对比试验/实物对比实验/20260828_动捕场地建图定位与双场地切换SOP.md)。

| 目的 | 入口 | 配套/行为 |
| --- | --- | --- |
| 冻结场地地图 | [freeze_spmpc_mocap_field_map.sh](freeze_spmpc_mocap_field_map.sh) | [地图环境模板](mocap_field_mapping.env.example)；写地图与 SHA，按原解锁条件执行 |
| 准备固定 S 路径 | [prepare_spmpc_mocap_s_path.sh](prepare_spmpc_mocap_s_path.sh) | 生成/冻结路径；[路径校验](../analysis/validate_mocap_s_path.py) |
| 静止录包 | [record_spmpc_mocap_static_smoke.sh](record_spmpc_mocap_static_smoke.sh) | 只录制；用于定位与 topic 检查 |
| 选择现场路径 | [run_spmpc_mocap_path_selection_trial.sh](run_spmpc_mocap_path_selection_trial.sh) | 会运动的路径筛选试验 |
| 检查完整执行链 | [run_spmpc_mocap_execution_chain_trial.sh](run_spmpc_mocap_execution_chain_trial.sh) | [环境模板](mocap_execution_chain_field.env.example)；解锁后运动；[分析](../analysis/analyze_mocap_execution_chain.py) |
| 单次速度阶跃 | [run_spmpc_mocap_velocity_step_trial.sh](run_spmpc_mocap_velocity_step_trial.sh) | 解锁后直接发速度；[响应分析](../analysis/analyze_mocap_velocity_step.py) |
| 多幅值/方向阶跃 | [run_spmpc_mocap_velocity_step_matrix_trial.sh](run_spmpc_mocap_velocity_step_matrix_trial.sh) | 批量调用单次试验；[矩阵汇总](../analysis/summarize_mocap_velocity_step_matrix.py) |
| 速度连续性 | [run_spmpc_mocap_velocity_continuity_trial.sh](run_spmpc_mocap_velocity_continuity_trial.sh) | 发测试命令；[连续性](../analysis/analyze_mocap_velocity_continuity.py)、[频谱](../analysis/analyze_velocity_continuity_spectrum.py) |
| 硬件加速度上限 | [run_spmpc_mocap_hardware_accel_limit_trial.sh](run_spmpc_mocap_hardware_accel_limit_trial.sh) | 专项脉冲辨识；默认校验，实际运行需原有解锁项 |
| B0 延迟模式对照 | [run_spmpc_mocap_b0_delay_mode_trial.sh](run_spmpc_mocap_b0_delay_mode_trial.sh) | 指定历史协议；[同包延迟分析](../analysis/analyze_same_bag_actuator_delay.py) |
| 液体 nowcast shadow | [run_spmpc_mocap_slosh_nowcast_shadow_trial.sh](run_spmpc_mocap_slosh_nowcast_shadow_trial.sh) | 指定历史协议；[验收](../analysis/validate_slosh_nowcast_shadow_bag.py) |
| 液体延迟诊断 | [run_spmpc_mocap_slosh_delay_diagnostic_trial.sh](run_spmpc_mocap_slosh_delay_diagnostic_trial.sh) | 指定历史协议；[同包分析](../analysis/analyze_slosh_nowcast_same_bag.py) |

地图、定位、路径三项检查分别用 [field map](../analysis/validate_mocap_field_map.py)、[field localization](../analysis/validate_mocap_field_localization.py)、[S path](../analysis/validate_mocap_s_path.py)。辨识 runner 会在原有条件满足后发布运动命令。
