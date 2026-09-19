# 两层主线：计划、诊断与录制

[返回总索引](../README.md) · [实验配置](../../config/experiments/trajectory_mpcc/README.md) · [native 验证](../../../../../../test/native/README.md)

| 入口 | 输入 → 输出 | 用途 |
| --- | --- | --- |
| [generate_trajectory_plan.py](generate_trajectory_plan.py) | task JSON → plan JSON | 全程几何、速度与液体联合规划；`--validate-plan` 重新传播已有计划 |
| [search_transport_duration.py](search_transport_duration.py) | 任务、基准计划、候选时长 → 搜索记录 | 默认固定时长并校验防晃；显式开启时间优化才搜索更短候选 |
| [extract_trajectory_checkpoint.py](extract_trajectory_checkpoint.py) | 单条 PredictedHorizon JSON/YAML → checkpoint | 提取完整液体状态，不能从 B0 零占位构造液体初态 |
| [diagnose_trajectory_suffix.py](diagnose_trajectory_suffix.py) | plan + checkpoint → 后缀报告 | 名义重放及重优化比较；`--nominal-only` 仅重放 |
| [record_trajectory_mpcc_comparison.sh](record_trajectory_mpcc_comparison.sh) | profile、区域、任务/计划 → bag、manifest、配置快照 | 只录已有节点；`VALIDATE_ONLY=true` 只做录制前校验 |

前四项离线运行，不发 ROS 运动命令。Python 使用 NumPy、CasADi/IPOPT、PyYAML；规划实现保留在 [planning/](../docs/DEVELOPMENT.md#planning)。

从仓库根目录调用：

```bash
SPMPC_SCRIPTS="$PWD/src/scout_apps/control/spmpc_local_planner/scripts"
python3 "$SPMPC_SCRIPTS/trajectory/generate_trajectory_plan.py" \
  test/native/scenarios/corner.json /path/new_plan.json
python3 "$SPMPC_SCRIPTS/trajectory/generate_trajectory_plan.py" \
  /path/new_plan.json /path/validation.json --validate-plan
python3 "$SPMPC_SCRIPTS/trajectory/extract_trajectory_checkpoint.py" \
  /path/horizon.yaml /path/checkpoint.json
python3 "$SPMPC_SCRIPTS/trajectory/diagnose_trajectory_suffix.py" \
  /path/new_plan.json /path/checkpoint.json /path/suffix.json --nominal-only
```

输出使用新目录/文件。录包需按配置 README 填 profile、region 和实际启动参数；上层任务 JSON 与 ROS 参数 overlay 是不同文件。原实物 wrapper 的冻结协议不能直接充当本主线验收。

## 防晃优先与时间优化

`search_transport_duration.py` 默认只校验任务固定时长，三项运输指标（峰值、P95、RMS）必须严格低于 `--slosh-limits limits.json` 中冻结的 B0 上限，单位为米：

```json
{"transport_peak_m": 0.00093, "transport_p95_m": 0.00046, "transport_rms_m": 0.00019}
```

以上仅为格式示例，实际数值须从同条件 B0 完整运输窗口冻结。保留 `--warm-plan`、`--cpp-validator` 和进程/总时限参数。完全相同且已真实收敛的基准计划通过重验后直接复用；任务或约束改变时重新求解。

只有显式传入 `--optimize-time --times 40 36` 才尝试更短时长，候选须从任务时长降序排列。更快但任一防晃指标不合格的候选不会被选中。这个开关用于离线规划；预测选中仍标为 `closed_loop_verified=false`，必须通过闭环完整运输、45 秒截止和停车观察检查才能报告通过。
