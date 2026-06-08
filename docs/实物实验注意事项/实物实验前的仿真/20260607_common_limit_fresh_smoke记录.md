# 20260607 common-limit fresh sim smoke 记录

## 1. 目的

本轮按“先统一比赛规则，再调方法参数”的要求，先不继续调 SPMPC 权重，而是把 standalone TEB / DWA baseline 的普通运动学限制对齐到 SPMPC 当前限制后，做 N=3 fresh sim smoke。

本轮只调整 planner / baseline 配置，不修改仿真环境。

## 2. 配置改动

SPMPC 保持不动：

```text
v_max = 0.8 m/s
omega_max = 1.2 rad/s
a_max = 0.6 m/s^2
shared linear_accel_max = 0.6 m/s^2
```

TEB / DWA standalone baseline 对齐为：

```text
max_vel_x = 0.8 m/s
max_vel_theta = 1.2 rad/s
acc_lim_x = 0.6 m/s^2
acc_lim_theta = 1.2 rad/s^2
```

注意：`acc_lim_theta` 只作为 planner 参数对齐；论文/报告中不能声称角加速度已完全公平，因为 SPMPC 当前主要是 `omega` bound，没有等价的 `omega rate` OCP 硬约束。

推荐表述：

```text
We align the translational velocity, angular velocity, and longitudinal acceleration limits across planners. Angular acceleration is reported as an outcome metric because the planners implement it differently.
```

## 3. fresh sim 规则

每个 case 单独启动仿真：

```bash
source /home/a/scout_ws/devel/setup.bash
SIM_ENV=open USE_RVIZ=true \
SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
rosrun scout_local_planner launch_sim_nav_stack.sh
```

执行规则：

```text
1. 启动仿真。
2. 等待 30s，让定位恢复。
3. 跑一个 planner/case。
4. 关闭仿真环境。
5. 等待 30s，确认仿真完全关闭。
6. 再启动下一次 fresh sim。
```

## 4. 输出目录

```text
/data/a/spmpc_paper_compare/common_limit_fresh_n3_20260607_151454
```

指标文件：

```text
/data/a/spmpc_paper_compare/common_limit_fresh_n3_20260607_151454/fixed_path_metrics.csv
/data/a/spmpc_paper_compare/common_limit_fresh_n3_20260607_151454/fixed_path_metrics_group_summary.csv
```

## 5. 本轮运行对象

N=3，interleaved 顺序：

```text
B0
B_smooth
B_accel
B_ours w_slosh=2.2
TEB common-limit
DWA common-limit
```

## 6. 结果状态

本轮程序层面完成，18 个 bag 均生成，并成功抽取 metrics。

但本轮 **不能作为有效 common-limit 对比结果**，原因不是要求 fixed path 与 fresh sim 起点完全一样，而是本轮二者不在同一个起跑区域；偏差达到米级，所有方法 `success=0`、`stable_stop=0`。

group summary 中 pre-terminal 主要结果：

| method | N | success | duration s | tracking RMS m | slosh peak mm | slosh p95 mm | cmd acc RMS | cmd acc p95 | max cmd acc | cmd omega-rate RMS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| B0 | 3 | 0.00 | 17.23 | 2.14 | 32.55 | 11.82 | 0.450 | 0.600 | 0.600 | 11.38 |
| B_smooth | 3 | 0.00 | 33.62 | 2.30 | 20.07 | 5.12 | 0.335 | 0.432 | 0.600 | 7.05 |
| B_accel | 3 | 0.00 | 33.20 | 2.23 | 31.65 | 6.93 | 0.348 | 0.426 | 0.667 | 9.34 |
| B_ours w=2.2 | 3 | 0.00 | 50.81 | 2.63 | 19.58 | 2.19 | 0.189 | 0.207 | 0.600 | 4.17 |
| TEB common-limit | 3 | 0.00 | 71.68 | 3.34 | 24.06 | 1.75 | 0.225 | 0.077 | 4.44 | 0.370 |
| DWA common-limit | 3 | 0.00 | 71.75 | 3.07 | 10.81 | 1.92 | 0.244 | 0.377 | 4.09 | 0.346 |

这些数值只能用于诊断，不应用于论文或方法结论。

## 7. 关键诊断

`P2_s_curve.json` 的路径起点约为：

```text
path start: x=3.448, y=0.205, frame=map
path end:   x=-3.612, y=3.956, frame=map
```

本轮 fresh sim 启动后，bag 中估计的机器人 map-frame 起点约在 path 起点前方约 4 m，典型诊断：

```text
robot map start -> path start distance: about 3.9 ~ 4.0 m
robot final -> path end distance: about 4.0 m for most moving runs
```

也就是说，本轮多数方法实际先花了很长时间从 fresh sim 起点向 fixed path 方向移动，但在 `RECORD_SEC=40` 对应的 bag 时间内没有到达 P2 路径终点。

另外，本轮各 case 都出现：

```text
[WARN] 30s 内未观察到 /spmpc/status 或 /cmd_vel; 仍继续录包
[WARN] 30s 内未观察到 /baseline/.../status 或 /cmd_vel; 仍继续录包
```

bag 中实际有 `/cmd_vel`，说明 planner 运行与录包不是完全失败；但 wait 阶段没有及时看到 status/cmd，后续需要单独检查启动时序或 status topic 发布时机。

## 8. 当前结论

本轮只确认了：

```text
1. TEB / DWA common-limit 参数已经加载生效。
2. fresh sim per case 流程能够完整跑完并生成 bag/metrics。
3. 当前 fresh sim 起点与 P2_s_curve fixed path 起点不匹配，因此本轮不能用于比较方法优劣。
```

不能得出：

```text
1. B_ours 是否优于 B0/B_smooth。
2. TEB/DWA 是否在 common-limit 下更强或更保守。
3. 当前 slosh peak/p95 的论文结论。
```

## 9. 后续 robust P2-start common-limit N=3

按方案 B 改为 P2 起点附近 fresh sim：

```text
SPAWN_X=3.30
SPAWN_Y=0.15
SPAWN_Z=0.1
SPAWN_YAW=-3.08
```

同时采用 robust 执行规则：每个 case fresh sim，启动后等 30s，每个 case 最多记录 60s，到时立即停止 planner / rosbag / path / slosh monitor / sim，避免单 case 超时污染后续数据。

输出目录：

```text
/data/a/spmpc_paper_compare/common_limit_p2start_robust_n3_20260607_195418
```

pre-terminal group summary：

| method | N | success | stable stop | duration s | final m | tracking RMS m | peak mm | p95 mm | cmd acc RMS | cmd acc p95 | cmd acc max | cmd omega-rate RMS | odom ax p95 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| B0 | 3 | 1.00 | 1.00 | 14.49 | 0.259 | 0.147 | 38.77 | 11.92 | 0.456 | 0.600 | 0.600 | 11.22 | 3.96 |
| B_smooth | 3 | 1.00 | 1.00 | 14.26 | 0.267 | 0.114 | 30.83 | 10.72 | 0.435 | 0.600 | 0.600 | 10.00 | 3.60 |
| B_accel | 3 | 1.00 | 1.00 | 15.65 | 0.278 | 0.166 | 42.26 | 14.97 | 0.443 | 0.600 | 0.640 | 14.06 | 4.35 |
| B_ours w=2.2 | 3 | 1.00 | 1.00 | 14.83 | 0.292 | 0.116 | 34.00 | 10.70 | 0.416 | 0.600 | 0.637 | 10.25 | 3.53 |
| TEB common-limit | 3 | 0.67 | 0.67 | 13.91 | 0.336 | 0.147 | 15.66 | 5.02 | 0.401 | 0.857 | 2.079 | 1.01 | 1.73 |
| DWA common-limit | 3 | 0.67 | 0.67 | 21.39 | 0.341 | 0.168 | 13.74 | 3.64 | 0.558 | 0.655 | 5.959 | 0.78 | 1.68 |

结论：

```text
1. SPMPC 四个内部方法在本轮 P2-start common-limit N=3 中均 success=1.0、stable_stop=1.0。
2. B_ours w=2.2 相比 B0/B_accel 降低了 slosh peak/p95，但与 B_smooth 非常接近：p95 基本相同，peak 反而略高于 B_smooth。
3. TEB/DWA 的 slosh peak/p95 更低，但各有 1/3 未达到 success/stable stop；同时它们不是 slosh-aware，且实际运动机制不同，不能直接写成 TEB/DWA 更强。
4. common-limit 参数对齐后，实际 cmd/odom 激励仍不完全对齐：TEB/DWA 的 odom ax p95 明显低于 SPMPC，但 cmd acceleration max 有尖峰；角速度变化率也与 SPMPC 机制不同。
```

当前不应得出的结论：

```text
B_ours 已经显著优于 B_smooth。
TEB/DWA 在液体运输意义上更强。
这组 N=3 可以直接作为论文最终主表。
```

下一步建议：

```text
1. 保留 common-limit 结果作为公平约束 smoke 与调参依据。
2. 若要扩大到 N=5，优先聚焦 B0 / B_smooth / B_ours w=2.2 / TEB / DWA；B_accel 可作为 supplemental。
3. 在扩大 N 前，可先检查 B_ours 与 B_smooth 差异不明显的原因，但不要盲目大范围调权重。
```
