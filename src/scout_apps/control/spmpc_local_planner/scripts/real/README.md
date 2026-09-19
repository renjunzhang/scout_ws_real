# 实物常用脚本

[返回总索引](../README.md) · [动捕与执行器](../mocap/README.md) · [RGB 高度识别](../rgb/README.md)

先确定实验分支、地图/路径、观察器和评价口径，再选择入口。实物 tracking 分支的近期扩展与当前开发分支不同，不能直接沿用所有参数；详见 [运行说明](../docs/USAGE.md)。

## 运行与采集

| 脚本 | 用途 | 行为/复用边界 |
| --- | --- | --- |
| [run_spmpc_ablation_smoke.sh](run_spmpc_ablation_smoke.sh) | C03 Full/Smooth、三组 RGB、初态来源对照 | 优先入口；默认校验，`--run` 启动运动与采集；C03 须显式选场景 |
| [run_spmpc_real_fixed_path_trial.sh](run_spmpc_real_fixed_path_trial.sh) | 通用单次固定路径实验 | 环境变量接口；直接执行可能运动，**没有通用 `--help/--run` 门**；通常由实验 wrapper 调用 |
| [run_external_baseline_real_fixed_path_trial.sh](run_external_baseline_real_fixed_path_trial.sh) | DWA/MPC 等外部基线 | 启动对应控制与采集；按相同路线、条件复用 |
| [run_fixed_profile_real_trial.sh](run_fixed_profile_real_trial.sh) | 固定 profile 统一入口 | 分发 SPMPC 或外部基线；保留原协议约束 |
| [record_spmpc_full_rgb_bag.sh](record_spmpc_full_rgb_bag.sh) | 全链白名单录制 | 只录包及元数据，不启动控制器；默认录在线液面，原始 RGB 默认关闭 |
| [summarize_spmpc_real_trial.py](summarize_spmpc_real_trial.py) | 单包快速摘要 | bag → summary JSON/Markdown；先排查采集与运行问题 |
| [record_spmpc_mainline_ground_smoke.sh](record_spmpc_mainline_ground_smoke.sh) | 历史地面 smoke 录制 | 轻量录制入口 |
| [record_spmpc_experiment.sh](record_spmpc_experiment.sh) | 早期手动实验录包 | 轻量录制入口 |
| [run_continuous_real.sh](run_continuous_real.sh) | 早期 continuous MPCC 实物试验 | 会启动运动；不支持 processed-IMU shadow，近期实验优先用通用 runner |

两个轻量 recorder 不包含新两层主线的完整配置冻结；新主线使用 [trajectory recorder](../trajectory/record_trajectory_mpcc_comparison.sh)。

## 常用命令

从仓库根目录执行。C03 示例只校验；现场冻结文件缺失或版本不匹配时会报错。

```bash
SPMPC_SCRIPTS="$PWD/src/scout_apps/control/spmpc_local_planner/scripts"
bash "$SPMPC_SCRIPTS/real/run_spmpc_ablation_smoke.sh" \
  --scene 20260907_c03 --experiment internal-slosh \
  --condition full --trial-id s01_full --phase screening \
  --w-slosh 1 --validate-only
```

控制器已单独启动、相机与标定已准备好时，录原始 RGB：

```bash
RECORD_RGB=true RECORD_CAMERA_INFO=true RECORD_SEC=60 \
  OUT_DIR=/path/to/new_run RUN_LABEL=full_01 \
  bash "$SPMPC_SCRIPTS/real/record_spmpc_full_rgb_bag.sh"
```

录包后先摘要、再画图：

```bash
python3 "$SPMPC_SCRIPTS/real/summarize_spmpc_real_trial.py" /path/run.bag
python3 "$SPMPC_SCRIPTS/analysis/plot_spmpc_full_da_diagnostics.py" \
  /path/run.bag --output-dir /path/new_plots
```

固定路径复用要明确 `PATH_SOURCE_MODE=replay`、`PATH_FILE` 与路径哈希；录制时长受 `MAX_RECORD_SEC` 限制。保留 bag、参数、topic 清单和元数据 sidecar，便于核对实际运行条件。

## 仿真与实物对照优先看什么

| 需要核对的差异 | 工具 |
| --- | --- |
| 低速响应、延迟、加速度和连续性 | [动捕辨识链](../mocap/README.md) |
| 命令到实际运动、预测误差 | [六图](../analysis/plot_spmpc_full_da_diagnostics.py)、[车体预测](../analysis/analyze_robot_state_prediction.py) |
| IMU/odom 状态来源及内部液体响应 | [内部液体配对](../analysis/analyze_internal_slosh_pair.py)、[评价冻结](../analysis/freeze_internal_slosh_evaluation.py) |
| 真实液面是否改善 | [RGB 离线识别流程](../rgb/README.md) |

`/slosh/height` 和内部观察器输出是模型量，独立液面证据来自 RGB；采集通过与防晃效果成立是两项结论。
