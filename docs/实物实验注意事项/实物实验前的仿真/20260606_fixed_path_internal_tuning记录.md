# 20260606 fixed-path 内部消融调参记录

## 1. 目的

本轮用于修正早前 fixed-path fresh 仿真中 `B_slosh / B_ours` 没有明显优于 `B_smooth` 的问题，重点寻找：

```text
B_slosh: 能稳定降低 slosh peak / p95 的 w_slosh
B_ours:  能稳定降低 p95，同时保持完成率的 w_slosh
```

本轮仍是仿真调参，不作为最终论文正式结果。

## 2. 仿真启动规则

按用户确认的 fresh sim 方式启动：

```bash
source /home/a/scout_ws/devel/setup.bash
SIM_ENV=open USE_RVIZ=true \
SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
rosrun scout_local_planner launch_sim_nav_stack.sh
```

执行规则：

```text
1. 每个 case 单独 fresh 启动仿真。
2. 启动后等待 /gazebo/model_states 和 /odom。
3. 再等待 30s 定位归位。
4. 只启动 planner / fixed path publisher / slosh monitor / rosbag。
5. 跑完后只关闭本 case 启动的仿真进程组。
6. 关闭仿真后等待 30s，再进入下一个 case。
7. 不修改 Gazebo / RViz / Cartographer / URDF 配置。
```

## 3. 输出目录

粗扫：

```text
/data/a/spmpc_paper_compare/manual_tune_20260606_173945_B0
/data/a/spmpc_paper_compare/internal_tune_fresh_safe_20260606_174312
/data/a/spmpc_paper_compare/internal_tune_fresh_safe2_20260606_212237
/data/a/spmpc_paper_compare/internal_tune_fresh_safe3_20260606_214856
/data/a/spmpc_paper_compare/internal_tune_fresh_safe4_20260606_222148
```

N=3 验证：

```text
/data/a/spmpc_paper_compare/internal_tune_candidates_n3_20260606_223907
/data/a/spmpc_paper_compare/internal_tune_bslosh25_n3_20260606_231317
```

## 4. 指标注意

本轮发现一个 metrics 口径问题：

```text
原脚本直接用 /odom 坐标和 map frame 的 fixed path 比较。
SPAWN_X=-4.0 时，final_dist / tracking / success 会误判。
```

处理：

```text
extract_fixed_path_paper_metrics.py 已改为读取 /tf 中的 map -> odom，
把 /odom pose 转到 path frame 后再计算 tracking / final_dist。
```

注意：这个修改只影响离线指标，不影响仿真环境。

## 5. 粗扫结论

粗扫范围：

```text
B_slosh: w_slosh = 1.0 / 1.5 / 2.0 / 2.25 / 2.5 / 2.6 / 2.7 / 2.75 / 2.85 / 3.0
B_ours:  w_slosh = 0.8 / 1.0 / 1.5 / 2.0 / 2.2
```

粗扫观察：

```text
1. B_slosh w=1.0 / 1.5：晃动很低，但没有到终点，不可用。
2. B_slosh w=2.5 / 2.75：单次看起来比较有希望。
3. B_ours w=2.2：单次 p95 最好，bad status 很少，值得 N=3 验证。
4. B_ours w=0.8 / 1.0 / 1.5：整体不如 w=2.2。
```

## 6. N=3 验证结果

### 6.1 主候选 N=3

输出目录：

```text
/data/a/spmpc_paper_compare/internal_tune_candidates_n3_20260606_223907
```

| variant | w_slosh | N | goal rate | bad status mean | duration mean s | peak mean mm | p95 mean mm | cmd acc RMS | odom ax p95 | tracking RMS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| B0 | -1.0 | 3 | 1.00 | 40.3 | 15.98 | 31.35 | 13.16 | 4.26 | 3.96 | 2.76 |
| B_smooth | -1.0 | 3 | 1.00 | 40.0 | 16.96 | 36.28 | 10.93 | 3.02 | 3.35 | 2.61 |
| B_slosh | 2.75 | 3 | 0.67 | 36.3 | 27.02 | 23.97 | 7.93 | 2.85 | 3.23 | 2.87 |
| B_ours | 2.2 | 3 | 1.00 | 25.7 | 21.22 | 26.19 | 7.34 | 1.74 | 2.51 | 2.84 |

结论：

```text
B_ours w=2.2 是目前最稳的完整方法候选。
它 goal rate=1.0，p95 明显低于 B0/B_smooth，peak 也低于 B0/B_smooth。

B_slosh w=2.75 peak 比较低，但 goal rate=0.67，不够稳。
```

### 6.2 B_slosh w=2.5 N=3 补验

输出目录：

```text
/data/a/spmpc_paper_compare/internal_tune_bslosh25_n3_20260606_231317
```

| variant | w_slosh | N | goal rate | bad status mean | duration mean s | peak mean mm | p95 mean mm | cmd acc RMS | odom ax p95 | tracking RMS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| B_slosh | 2.5 | 3 | 0.67 | 19.0 | 25.99 | 27.91 | 7.47 | 2.63 | 2.69 | 2.84 |

结论：

```text
B_slosh w=2.5 也不是稳定参数。
它 p95 不错，但 N=3 中 goal rate 仍只有 0.67。
```

## 7. 当前可用结论

本轮调参后，可以说：

```text
1. 早前 w_slosh=3.0 不是好参数。
2. B_slosh 单独项在 w=2.5 / 2.75 附近能降低 peak/p95，但完成率不稳定。
3. B_ours w=2.2 是目前最值得进入下一轮正式对比的参数。
4. B_ours w=2.2 相比 B0/B_smooth 有更低的 slosh peak 和 p95，同时 N=3 完成率保持 1.0。
```

不能说：

```text
1. B_slosh 已经稳定优于 B_smooth。
2. B_ours 已经最终打赢所有 baseline。
3. 这组仿真结果可以直接作为论文正式主表。
```

## 8. 下一步建议

建议下一步不要继续盲目大范围扫参，而是：

```text
1. 固定 B_ours w_slosh=2.2，做 N=5 或实物预实验。
2. B_slosh 如果还要保留，可在 w=2.5 / 2.75 附近再做更小步长或改终点策略，但不是当前最稳候选。
3. 正式对比表主指标优先看 slosh p95、peak、goal rate、bad status、duration、tracking。
4. TEB/DWA 对比时必须同时报告速度/加速度/跟踪误差，避免只看 peak。
```

当前推荐进入下一轮的参数：

```text
B_ours:  w_slosh = 2.2
B_slosh: 暂定 w_slosh = 2.75 作为 peak-oriented 候选，但需要标注完成率风险
```
