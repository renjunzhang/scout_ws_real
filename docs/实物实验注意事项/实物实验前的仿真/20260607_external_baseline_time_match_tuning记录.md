# 20260607 TEB / DWA time-matched 外部 baseline 调参记录

## 1. 目的

本轮用于给 fixed-path 外部 baseline 找一组公平参数：

```text
1. 能稳定完成 fixed path。
2. 时间尽量接近当前 B_ours w_slosh=2.2。
3. 不故意调差。
4. 不使用 slosh state / slosh-aware cost / /slosh/* 控制反馈。
5. /slosh/* 只用于评价。
```

## 2. 仿真启动和关闭规则

沿用本轮调参确认过的 fresh sim 规则：

```text
1. 每个 case 单独 fresh 启动仿真。
2. 启动后等待 /gazebo/model_states 和 /odom。
3. 再等待 30s 定位归位。
4. 只启动 baseline planner / fixed path publisher / slosh monitor / rosbag。
5. 跑完后只关闭本 case 启动的仿真进程组。
6. 关闭仿真后等待 30s。
7. 不修改 Gazebo / RViz / Cartographer / URDF。
```

起点采用 fixed path 起点附近的容错位姿，而不是完全重合：

```text
SPAWN_X=3.30
SPAWN_Y=0.15
SPAWN_YAW=-3.08
```

P2_s_curve 路径起点约为：

```text
x=3.45, y=0.21, yaw=-3.08
```

因此起点偏差约 0.16 m，属于合理容错范围。

## 3. 输出目录

粗扫：

```text
/data/a/spmpc_paper_compare/external_time_match_near_start_20260607_000714
/data/a/spmpc_paper_compare/dwa_time_match_extra_20260607_004227
/data/a/spmpc_paper_compare/dwa_finish_tune_20260607_005644
```

N=3 候选验证：

```text
/data/a/spmpc_paper_compare/external_time_match_candidates_n3_20260607_011256
```

## 4. metrics 口径修正

调参过程中确认：

```text
本 fixed-path/baseline 数据中，/odom 数值坐标和 fixed path 数值坐标是直接对齐的。
不应额外用 map->odom 再转换，否则会把 TEB/DWA 的 final_dist / success 算错。
```

因此 `extract_fixed_path_paper_metrics.py` 已恢复为 direct odom-vs-path 统计口径。

## 5. TEB 调参结果

TEB 粗扫结果：

| case | success | duration s | final m | tracking RMS | peak mm | p95 mm | cmd acc RMS |
|---|---:|---:|---:|---:|---:|---:|---:|
| teb_nominal | 1 | 14.85 | 0.317 | 0.240 | 11.88 | 3.04 | 0.24 |
| teb_time_mid | 1 | 18.52 | 0.324 | 0.144 | 8.37 | 2.52 | 0.23 |
| teb_time_slow | 1 | 22.73 | 0.303 | 0.178 | 6.19 | 2.26 | 0.16 |

结论：

```text
TEB_time_slow 最适合作为 time-matched baseline 候选。
它完成率好，时间最接近当前 B_ours w=2.2，但 slosh peak/p95 很低，说明它控制激励非常保守。
```

## 6. DWA 调参结果

第一轮 DWA 多数停在终点外 0.40 m 左右，没有进入 0.35 m 成功阈值。随后补了 finish-focused sweep。

DWA finish sweep 结果：

| case | success | duration s | final m | tracking RMS | peak mm | p95 mm | cmd acc RMS |
|---|---:|---:|---:|---:|---:|---:|---:|
| dwa_goal_bias60 | 1 | 16.69 | 0.198 | 0.368 | 8.02 | 2.45 | 0.15 |
| dwa_goal_bias80 | 1 | 16.58 | 0.216 | 0.341 | 9.72 | 2.84 | 0.16 |
| dwa_long_sim_goal | 1 | 17.94 | 0.188 | 0.357 | 19.45 | 3.72 | 0.54 |
| dwa_nominal_tight_goal | 1 | 17.43 | 0.265 | 0.276 | 25.36 | 4.70 | 0.74 |

结论：

```text
DWA_goal_bias60 和 DWA_long_sim_goal 都能稳定完成。
DWA_goal_bias60 更低晃动，但时间偏快。
DWA_long_sim_goal 时间略慢一点，但 tracking 更松，peak 更高。
```

## 7. N=3 外部 baseline 候选验证

输出目录：

```text
/data/a/spmpc_paper_compare/external_time_match_candidates_n3_20260607_011256
```

验证对象：

```text
TEB_time_slow
DWA_goal_bias60
DWA_long_sim_goal
```

N=3 group summary：

| label | method | N | success | stable stop | duration s | final m | tracking RMS | peak mm | p95 mm | cmd acc RMS | odom ax p95 | bad status |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| teb_time_slow | teb | 3 | 1.00 | 1.00 | 21.71 | 0.317 | 0.228 | 10.54 ± 1.13 | 2.60 ± 0.34 | 0.17 | 1.07 | 0.0 |
| dwa_goal_bias60 | dwa | 3 | 1.00 | 1.00 | 16.78 | 0.265 | 0.327 | 8.53 ± 0.42 | 3.15 ± 0.33 | 0.16 | 1.10 | 0.0 |
| dwa_long_sim_goal | dwa | 3 | 1.00 | 1.00 | 17.61 | 0.172 | 0.395 | 9.24 ± 0.41 | 2.79 ± 0.09 | 0.14 | 1.06 | 0.0 |

## 8. 当前推荐 baseline 参数

### 8.1 TEB 推荐

推荐进入下一轮正式对比：

```text
TEB_time_slow
```

理由：

```text
1. N=3 success = 1.0。
2. stable stop = 1.0。
3. duration ≈ 21.7 s，和 B_ours w=2.2 的时间最接近。
4. slosh peak/p95 很低，但控制加速度也很低，适合用来说明 TEB 是 conservative baseline。
```

### 8.2 DWA 推荐

推荐保留两个版本：

```text
DWA_goal_bias60
DWA_long_sim_goal
```

如果只能选一个主表 baseline，建议优先：

```text
DWA_goal_bias60
```

理由：

```text
1. N=3 success = 1.0。
2. stable stop = 1.0。
3. final distance 稳定小于 0.35 m。
4. peak/p95 较低。
```

但要在文中说明：

```text
DWA_time_matched 仍比 B_ours 快一些，且 tracking RMS 偏大。
```

如果更强调“时间接近”，可以展示 `DWA_long_sim_goal` 作为补充，因为它稍慢一点，但 tracking 更松。

## 9. 论文/报告口径

不要写成：

```text
我们故意把 TEB/DWA 调差。
```

应该写成：

```text
TEB/DWA were tuned for stable completion under comparable traversal conditions, without access to slosh states or slosh-aware costs.
```

中文意思：

```text
TEB/DWA 被调到能稳定完成，并尽量接近相同通过条件；它们不使用液体状态，也没有液体感知代价。
```

重要解释：

```text
TEB/DWA 的 slosh peak/p95 很低，主要伴随更低 cmd acceleration、较大的 tracking error 或更保守的运动；
这不是 slosh-aware 能力，而是普通外部 planner 的保守控制特性。
```

## 10. 下一步

建议下一轮正式 fixed-path 对比至少包括：

```text
B0
B_smooth
B_ours w_slosh=2.2
TEB_time_slow
DWA_goal_bias60
```

可选补充：

```text
B_slosh w_slosh=2.75  # peak-oriented，但完成率风险较高
DWA_long_sim_goal     # DWA slower variant
```
