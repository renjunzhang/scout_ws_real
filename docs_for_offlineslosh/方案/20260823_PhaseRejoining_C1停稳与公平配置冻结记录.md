# Phase-Rejoining C1 停稳与公平配置冻结记录

- 日期：2026-08-23
- 分支：`offline-slosh-plan-online-tracking`
- 范围：正式 C0--C4/IS 主矩阵前的 task-only development 冻结
- 结论：**C1 停稳和多 seed 任务可靠性通过，配置冻结；正式液面结果未用于调参，正式 seeds 尚未运行**

## 1. 冻结约束

本轮保持 45 s 最大运动时间和 `|v| <= 0.03 m/s` 成功门不变，不修改 C4、gate、V3、B_exec、Full SQP、recovery 数据或 Plant。候选只按任务成功、tracking、完成时间、加速度和 jerk 选择；外部 Plant 液面、控制器内部液体 observer 和 C4 液面结果均不进入调参或择优。

development seeds 固定为 `8801--8805`，与 recovery fit/tune/held-out seeds `8301--8502`、C4 development seeds `8601/8701--8705` 和正式 seeds `3101--3116` 互斥。

## 2. 停稳根因与修复

旧逻辑从 terminal slowdown 进入 `capture_stop` 时，capture 速度上限会重新放宽已经收紧的 slowdown envelope，车辆因而在终点附近再次加速。现在 capture envelope 始终取 slowdown、capture cap 和制动可达 cap 的最小值；一旦减速就只能继续收紧。

新增单元测试精确覆盖“进入 capture 后不得放宽 slowdown envelope”。C1 condition 还显式绑定并在 summary 中记录 terminal 参数，避免依赖通用默认值：

```yaml
slowdown_distance_m: 0.60
slowdown_v_max_mps: 0.80
capture_distance_m: 0.50
capture_v_cap_mps: 0.80
```

## 3. 冻结配置

```yaml
implementation_id: continuous_mpcc_acados_smooth_match_v3
pilot_tuned_and_frozen: true
global_time_scale: 1.0
w_contour: 1.0
w_lag: 0.2
w_progress: 1.4
w_heading: 1.0
w_progress_coupling: 20.0
w_yaw_rate_tracking: 15.0
heading_feedback_gain: 4.0
w_v: 1.0
w_vs: 0.3
v_ref: 0.32
w_control: 0.1
w_smooth: 0.1
w_alpha: 0.1
w_du_a: 0.1
w_du_vs: 0.1
```

冻结 condition SHA-256（提交前内容）：`bdf91bc4067692090e9618cc5efd043aafa95706de039f333026c92b27f5d846`。

## 4. 多 seed development 结果

证据目录只读保留在：

`/data/a/spmpc_exec_identification/phase_rejoin_c1_final_freeze_dev_20260823.7pwUi3`

| seed | 完成时间 | tracking q95 | accel q95 | jerk q95 | 结果 |
| ---: | ---: | ---: | ---: | ---: | --- |
| 8801 | 43.77 s | 0.09460 m | 0.09444 m/s² | 2.5690 m/s³ | `GOAL_REACHED` |
| 8802 | 44.40 s | 0.09085 m | 0.09652 m/s² | 2.6114 m/s³ | `GOAL_REACHED` |
| 8803 | 44.27 s | 0.09065 m | 0.09444 m/s² | 2.4870 m/s³ | `GOAL_REACHED` |
| 8804 | 44.53 s | 0.09127 m | 0.08929 m/s² | 2.5779 m/s³ | `GOAL_REACHED` |
| 8805 | 44.23 s | 0.09090 m | 0.09919 m/s² | 2.6229 m/s³ | `GOAL_REACHED` |

五次均为 `task_success=true`、`solver_failures=0`、`controlled_stops=0`，最终位置误差为 `0.00884--0.01000 m`；控制器未读取外部液体真值。

冻结后的独立几何复核位于：

`/data/a/spmpc_exec_identification/phase_rejoin_c1_frozen_geometry_dev_20260823.mRHPE6`

直线、半径 3 m 的 90°圆弧和 S 弯均再次 `GOAL_REACHED`，无 solver failure 或 controlled stop。

## 5. 与 C4 的公平性解释

C4 development 参考 8701--8705 的平均完成时间约 `35.29 s`、tracking q95 约 `0.05842 m`、accel q95 约 `0.18761 m/s²`、jerk q95 约 `2.82497 m/s³`。C1 平均值依次为 `44.24 s`、`0.09165 m`、`0.09478 m/s²`、`2.57364 m/s³`。

C1 仍明显更慢且加速度/jerk 更小，不能声称两者精确等时等平滑；但这是一项保守 comparator：若后续冻结正式结果中 C4 的液面指标优于 C1，就不能用“C4 只是跑得更慢或更平滑”解释。C1 tracking 与 C4 development 参考的差约 `0.033 m`，处于预注册的 `0.05 m` task tracking 非劣界内。该解释在看到正式液面结果前冻结，后续不得根据液面结果修改 C1。

## 6. 当前边界

C1 控制侧冻结门已通过，但这不是正式性能证据。下一步只能在干净、已 push 的提交上重新构建并物化 hash-bound session；readiness 达到 `READY_NOT_EXECUTED` 后仍需人工检查，人工许可前不得运行 seeds `3101--3116`。
