# 20260606 fixed-path N=3 fresh 仿真记录

## 1. 实验目的

本轮仿真用于在实物实验前检查 fixed-path 对比流程，并尝试确认 `B_slosh` 在 `P2_s_curve` 上是否能相对 `B0 / B_smooth / B_accel` 体现液体晃动抑制优势。

本轮不是最终论文正式结果，原因：

```text
1. 仍是模型预测 /slosh/height，不是 RGB 真值。
2. 自动 fresh 启停 Gazebo 过程中出现过多次 sim 启动失败，需要补跑。
3. 每组 N=3，能看趋势，但还不足以下最终结论。
```

## 2. 输出目录

```text
/data/a/spmpc_paper_compare/fixed_path_n3_fresh_20260606_141900
```

最终成功录到：

```text
B0          3 bags
B_smooth    3 bags
B_accel     3 bags
B_slosh     3 bags
B_ours      3 bags
TEB         3 bags
DWA         3 bags
```

指标文件：

```text
/data/a/spmpc_paper_compare/fixed_path_n3_fresh_20260606_141900/fixed_path_metrics.csv
/data/a/spmpc_paper_compare/fixed_path_n3_fresh_20260606_141900/fixed_path_metrics_group_summary.csv
```

## 3. 运行设置

```text
path_id: P2_s_curve
path_file: /data/a/fixed_paths/sim/P2_s_curve.json
fresh sim: 是，每个 run 单独启动/关闭仿真
record_sec: 40
pre_path_wait_sec: 30
solver_backend: continuous_mpcc_acados
SPMPC_W_SLOSH: 3.0
slosh monitor: enabled
/slosh/height unit: m，metrics 中换算为 mm
```

方法：

```text
B0
B_smooth
B_accel
B_slosh
B_ours
TEB
DWA
```

注意：`B_accel` 是可选 supplemental baseline，不使用 slosh state，不读取 `/slosh/*`，只是加强加速度惩罚。

## 4. 仿真运行问题

第一次 N=3 自动跑时，部分 run 启动仿真失败，典型日志：

```text
timeout waiting for /odom
timeout waiting for /gazebo/model_states
SpawnModel: Failure - model name scout/ already exist
gzserver process has died
```

判断：

```text
不是 planner 方法本身失败，而是自动连续启停 Gazebo 不够干净。
```

后续使用 process group 方式补跑，最终补齐到每组 3 个 bag。

实物/正式仿真建议：

```text
手动 fresh 启动仿真 -> 跑一个方法 -> 关仿真 -> 确认 Gazebo 完全退出 -> 再跑下一个。
```

不要完全依赖自动脚本快速连续启停 Gazebo。

## 5. N=3 group summary：pre_terminal 主体段

| method | N | success_mean | tracking RMS m | slosh peak mm | slosh p95 mm | final dist m | stable stop |
|---|---:|---:|---:|---:|---:|---:|---:|
| DWA | 3 | 0.333 | 0.132 | 14.32 | 3.15 | 0.341 | 0.333 |
| B0 | 3 | 0.667 | 0.102 | 35.18 | 14.98 | 0.339 | 0.667 |
| B_accel | 3 | 0.667 | 0.119 | 32.62 | 13.81 | 0.381 | 0.667 |
| B_ours | 3 | 0.667 | 0.166 | 38.13 | 12.40 | 0.323 | 0.667 |
| B_slosh | 3 | 1.000 | 0.136 | 36.07 | 13.43 | 0.270 | 1.000 |
| B_smooth | 3 | 1.000 | 0.104 | 34.69 | 12.80 | 0.252 | 1.000 |
| TEB | 3 | 1.000 | 0.199 | 13.62 | 3.90 | 0.297 | 1.000 |

## 6. 当前结论

### 6.1 `B_slosh` 这轮没有稳定体现 peak 优势

这轮 N=3 中，`B_slosh` 的成功率最好：

```text
B_slosh success_mean = 1.0
stable_stop = 1.0
```

但液面模型指标没有明显优于 `B_smooth`：

```text
B_slosh peak = 36.07 mm
B_smooth peak = 34.69 mm

B_slosh p95 = 13.43 mm
B_smooth p95 = 12.80 mm
```

所以不能写成：

```text
B_slosh 在本轮 N=3 中明显优于 B_smooth。
```

更准确的说法：

```text
B_slosh 在本轮中成功率和终点稳定性较好，但 slosh peak/p95 相对 B_smooth 没有形成稳定优势。
```

### 6.2 `B_slosh` 相对 `B0` 有一点稳定性优势，但 peak 不占优

对比 `B0`：

```text
B0 success_mean = 0.667
B_slosh success_mean = 1.0

B0 final dist = 0.339 m
B_slosh final dist = 0.270 m

B0 slosh p95 = 14.98 mm
B_slosh slosh p95 = 13.43 mm
```

但 peak：

```text
B0 peak = 35.18 mm
B_slosh peak = 36.07 mm
```

因此当前只能说：

```text
B_slosh 相比 B0 有更好的完成率、终点距离和 p95 趋势；但 peak 没有明显降低。
```

### 6.3 TEB 的 peak 低，主要像是保守控制导致

TEB：

```text
slosh peak = 13.62 mm
slosh p95 = 3.90 mm
tracking RMS = 0.199 m
```

TEB 的 tracking error 明显大于 SPMPC 组，说明它更像是：

```text
路径跟得更松、控制更保守，所以模型预测晃液更低。
```

这不是 slosh-aware 能力，因为 TEB 没有使用 slosh state 或 slosh-aware cost。

### 6.4 `B_ours w_slosh=3.0` 当前不理想

`B_ours` 本轮：

```text
success_mean = 0.667
tracking RMS = 0.166 m
slosh peak = 38.13 mm
```

说明 `B_ours` 在当前参数下没有体现完整方法优势。可能原因：

```text
1. w_slosh=3.0 对 B_ours 未必最合适。
2. smooth + slosh 同时作用后，求解器可能更容易进入局部不稳定或恢复动作。
3. peak 指标可能受少数瞬时 spike 影响。
```

## 7. 不能得出的结论

当前数据不能支持：

```text
B_slosh 明显优于 B_smooth。
B_ours 明显优于所有 baseline。
TEB 比我们方法更强。
```

当前数据只能支持：

```text
1. fixed-path fresh-sim 流程已经能跑通并形成 N=3 bag/metrics。
2. /slosh monitor 记录正常。
3. TEB 低 peak 伴随更大的 tracking error，像是保守控制导致。
4. B_slosh 在成功率/终点稳定性上比 B0 更稳，但 slosh peak 优势还不稳定。
```

## 8. 下一步建议

### 8.1 不要急着用这组数据写论文结论

这组数据适合做：

```text
仿真流程验证
调参依据
实物实验前风险预判
```

不建议作为最终论文主表。

### 8.2 继续调 SPMPC 权重，而不是硬说当前结果好

建议下一轮小 sweep：

```text
B_slosh: w_slosh = 2.5 / 3.0 / 3.5
B_ours:  w_slosh = 2.0 / 2.5 / 3.0
```

重点看：

```text
success_mean
stable_stop_bool
tracking_error_rms
slosh_height_peak_mm
slosh_height_p95_mm
bad_status_count（当前 metrics 里叫 failure_count）
```

### 8.3 指标脚本建议改名

当前 `failure_count` 实际统计的是中间坏状态次数，例如：

```text
ACADOS_SOLVE_FAILED_4
NO_VALID_CMD
WAITING
```

它不等于整次 run 失败。建议后续改成：

```text
bad_status_count
fatal_failure_bool
```

否则看表容易误解。

## 9. 对实物实验的提醒

实物实验前仍应坚持：

```text
1. B0 / B_smooth / B_slosh / B_ours 使用同一 fixed path / goal / 起点标记。
2. 每组至少 N=3，更好 N=5。
3. 顺序交错，不要连续跑完同一方法。
4. 最终抑晃结论看离线 RGB 真值，不只看 /slosh/height。
5. TEB/DWA 只做非 slosh-aware baseline，不允许使用 /slosh/* 控制反馈。
```
