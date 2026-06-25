# 官方 LT-DWA 接入 Scout 隔离仿真环境当前状态

**日期：** 2026-06-24

**目标：** 在不破坏 `/data/a/scout_sim_replacement` 仿真环境和官方 LT-DWA upstream 的前提下，把官方 LT-DWA ROS Noetic 实现接入 Scout 隔离仿真 SOP，并作为后续 baseline/benchmark 的候选。

---

## 1. 一句话结论

当前状态是：

```text
官方 LT-DWA core 已经接入 Scout 隔离仿真环境；早期 S-curve 仅 endpoint reached 但 tracking quality 不合格。
2026-06-25 增加 config 透传、reference 修正和 wrapper path-tracking guard 后，同一终点 fixed S-curve 可在 60s 内到达，轨迹误差显著下降，但仍弱于 DWA/SPMPC baseline。
```

更精确地说：

```text
这是“官方 LT-DWA ROS Noetic core + wrapper 接入 Scout SOP”，
不是把官方 local_planner_node 原封不动当作 Scout runtime planner 使用。
```

原因是官方仓库虽然是 ROS Noetic 工程，但公开主程序更像作者 demo/batch harness，不是可直接接 Scout `/odom`、`/map`、`/scout/global_path_fixed`、`/cmd_vel` 的标准 ROS local planner 插件。

---

## 2. 是否按照 LT-DWA 官方 ROS Noetic 接入？

答案：**是，但需要加一句限定。**

### 2.1 是“官方实现”的部分

我们使用的是官方 LT-DWA 仓库的真实核心代码，而不是继续调之前的 `LT-DWA-v2-inspired` 适配器。

官方代码路径：

```text
/data/a/lt_dwa_official_repro_ws/src/LT_DWA
```

当前保持只读，不修改 upstream。

实际调用的官方核心包括：

```text
local_planner/include/policy/seed_policy.hpp
local_planner/src/seed_policy.cpp
local_planner/src/eb_mpc_trajectory_optimizer.cpp
```

关键官方 seam：

```cpp
int SeedPolicy::forward(
    Robot& robot,
    const Pose& target_pose,
    const std::vector<PathPose>& navigation_path,
    const GridMap& global_map,
    const std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>>& obstacles_info,
    Action& planned_action);
```

官方输出命令的位置仍然是：

```cpp
planned_action.v_ = opt_states[1].v_;
planned_action.w_ = opt_states[1].w_;
```

也就是说，当前 wrapper 调的是官方 LT-DWA/EB-MPC core，而不是自研 inspired scoring。

### 2.2 不是“官方 node 原封不动接入”的部分

官方公开的 ROS Noetic 工程不是标准 Scout runtime planner。它的主程序包含作者自己的 demo/test loop，例如：

```text
static/orca/crowd demo
getchar() 启动
固定 case batch loop
无 Scout odom/path/map/topic runtime 接口
无直接对 Scout /cmd_vel 的安全 gate
```

因此不能直接把官方 `local_planner_node` 原封不动塞进 Scout SOP。当前采用的是更安全的方式：

```text
Scout ROS topics
  -> wrapper bridge
  -> worker process
  -> official SeedPolicy::forward(...)
  -> structured worker result
  -> 30Hz command bridge
  -> shadow or /cmd_vel gate
```

---

## 3. 当前接入架构

当前 active 实现已经迁入主仓库：

```text
/home/a/scout_ws/src/scout_apps/control/lt_dwa_official_wrapper
/home/a/scout_ws/src/scout_apps/control/lt_dwa_official_vendor_deps/{obstacle_msgs,local_map_generation}
/home/a/scout_ws/tools/lt_dwa/local_planner_runtime
/home/a/scout_ws/third_party/LT_DWA
```

`third_party/LT_DWA` 是 official source-only vendor，不能删除，也不能 symlink 到 catkin `src/`。

核心链路：

```text
Scout SOP sim
  /odom
  /map
  /tf map->base_link
  /scout/global_path_fixed
        │
        ▼
lt_dwa_map_frame_odom_adapter
        │  publishes map-frame odom
        ▼
/baseline/official_lt_dwa/odom_map
        │
        ▼
lt_dwa_scout_bridge
        │  builds PlannerInput, writes request file
        ▼
lt_dwa_worker --mode official-core-once
        │  calls official SeedPolicy::forward(...)
        ▼
structured worker result
        │
        ▼
30Hz command publisher
        ├── /baseline/official_lt_dwa/shadow_cmd_vel
        └── /cmd_vel only if explicitly enabled
```

默认安全状态：

```text
enable_actuated_output=false
publish_cmd_vel=false
publish_benchmark_raw=false
```

只有用户明确授权后，才把命令发布到 `/cmd_vel`。

---

## 4. 已完成工作

### 4.1 官方复现与 core 调用

完成：

```text
[PASS] 官方 LT-DWA isolated clone/build
[PASS] 官方 demo 静态/ORCA 基础复现
[PASS] 官方 core worker-only 调用边界
[PASS] worker official-core-once 模式
[PASS] missing structured response -> CORE_PROCESS_EXITED
```

### 4.2 Scout-facing wrapper

完成：

```text
[PASS] Scout bridge shadow mode
[PASS] real-time command bridge
[PASS] planner cadence 默认 5Hz
[PASS] command publish cadence 默认 30Hz
[PASS] stale/invalid worker result -> zero command
[PASS] actuation explicit gate
```

### 4.3 Scout SOP 接入 overlay

新增：

```text
/data/a/lt_dwa_wrapper_ws/src/lt_dwa_official_wrapper/launch/scout_sop_shadow_integration.launch
/data/a/lt_dwa_wrapper_ws/src/lt_dwa_official_wrapper/launch/scout_sop_cmd_vel_benchmark.launch
```

用途：

```text
scout_sop_shadow_integration.launch
  默认 shadow-only，不发布 /cmd_vel。

scout_sop_cmd_vel_benchmark.launch
  默认 inert；只有显式 enable_actuated_output=true + publish_cmd_vel=true 才发布 /cmd_vel。
```

### 4.4 map-frame odom 固化

新增：

```text
/data/a/lt_dwa_wrapper_ws/src/lt_dwa_official_wrapper/src/map_frame_odom_adapter_node.cpp
```

作用：

```text
/odom + TF(map -> base_link)
  -> /baseline/official_lt_dwa/odom_map
```

这样 bridge 可以严格使用 `planning_frame=map`，不再依赖 `/tmp` 临时脚本。

### 4.5 `spmpc_experiments` baseline 切换

2026-06-24 按“只保留 LT-DWA 官方 ROS Noetic 接入”的方向梳理：

```text
lt_dwa baseline id
  -> src/scout_apps/control/lt_dwa_official_wrapper
  -> /baseline/lt_dwa/* experiment namespace
```

旧路径已从 active benchmark 中退休：

```text
src/scout_apps/control/lt_dwa_adapter/
src/scout_apps/control/lt_dwa_v2_adapter/
spmpc_experiments/launch/sim/run_lt_dwa_v2_fixed_path_sim.launch
spmpc_experiments/config/baselines/lt_dwa_v2_adapter_standalone_sim.yaml
```

需要保留：

```text
third_party/LT_DWA
```

原因是 official wrapper 的 worker 编译/调用依赖其中的官方 `SeedPolicy::forward(...)`、`seed_policy.cpp` 和 `eb_mpc_trajectory_optimizer.cpp` 等源码。

### 4.6 official Robot TF 副作用隔离

官方 `Robot` 构造函数会启动 detached TF broadcaster，原本可能向全局 `/tf` 发布：

```text
odom -> base_footprint
```

当前已在 worker 侧 sandbox：

```text
/tf:=/baseline/official_lt_dwa/worker_tf_sandbox
/tf_static:=/baseline/official_lt_dwa/worker_tf_static_sandbox
```

并且 worker official-core mode 改为从 `argc/argv` 初始化 ROS，因此 remap 会生效。

---

## 5. 已验证结果

### 5.1 编译和测试

```text
catkin_make --directory /data/a/lt_dwa_wrapper_ws \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DLT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE=ON
# PASS

catkin_make --directory /data/a/lt_dwa_wrapper_ws run_tests
catkin_test_results /data/a/lt_dwa_wrapper_ws/build/test_results
# Summary: 74 tests, 0 errors, 0 failures, 0 skipped
```

### 5.2 shadow integration gate

在 Scout 隔离仿真 SOP 环境中验证：

```text
ROS_MASTER_URI=http://localhost:11328
GAZEBO_MASTER_URI=http://localhost:11362
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
```

结果：

```text
/baseline/official_lt_dwa_rt06/status: OK
/baseline/official_lt_dwa_rt06/odom_map.header.frame_id: map
LT_DWA_WORKER_RESULT status=OK reason=official_core_ok command_v=0.00972246 command_w=0.198013 core_return=0
command_publish_rate_hz=30
command_stale_timeout_sec=0.6
enable_actuated_output=false
effective_cmd_vel=false
effective_benchmark_raw=false
/cmd_vel Publishers: None
/benchmark/cmd_vel_raw: Unknown topic
```

### 5.3 用户授权后的 `/cmd_vel` benchmark

用户明确授权：

```text
可以直接发 /cmd_vel
```

随后执行 bounded 60s visible benchmark。

结果：

```text
START_POSE x=0.095775 y=0.016032 goal_x=5.000000 goal_y=-0.000000 distance=4.904252
PROGRESS t=5.0  x=2.829205 y=0.659287  distance=2.268702 min_distance=2.268702
PROGRESS t=15.0 x=3.822791 y=0.480408  distance=1.271461 min_distance=0.812466
PROGRESS t=50.0 x=4.118550 y=0.922426  distance=1.275863 min_distance=0.507000
PROGRESS t=55.0 x=4.404119 y=-0.211868 distance=0.632426 min_distance=0.507000

RESULT FAIL reason=timeout_60s_not_reached elapsed=60.000
start_distance=4.904252
min_distance=0.507000
final_distance=0.657899
final_x=4.611976
final_y=-0.531289
goal_x=5.000000
goal_y=-0.000000
```

按当前规则：

```text
60s 未到终点 = FAIL
```

因此这次 benchmark 是 **FAIL**。

结束后安全确认：

```text
/cmd_vel Publishers: None
/benchmark/cmd_vel_raw: Unknown topic
```

---

## 6. 当前主要问题

### 问题 A：通路能跑通，但 endpoint gate 未通过

当前官方 LT-DWA 接入链路已经能让车运动，并且接近终点：

```text
min_distance=0.507 m
final_distance=0.658 m
```

但没有在 60s 内进入 `0.30 m` tolerance。

这说明问题已经从“接入失败”转移为：

```text
官方 LT-DWA + 当前 wrapper/路径/终点策略下的收敛性问题
```

### 问题 B：one-shot worker latency 仍偏大

当前 worker 是每次 planner tick fork/exec 一次 official core worker。

观测延迟：

```text
shadow gate: 约 390-420 ms
benchmark final diagnostics: 315.609 ms
```

因此如果 stale timeout 设成 `0.25s`，会频繁 zero；当前 overlay 使用：

```text
command_stale_timeout_sec=0.6
```

这能保证 fresh command，但控制实时性仍不理想。

### 问题 C：官方目标语义不是强终点吸引

官方 `SeedPolicy::forward(...)` 中：

```text
target_pose 主要用于 goal reached 判断；
优化 cost 主要跟踪 path-derived reference states。
```

因此在终点附近，如果车偏离 reference path 或姿态不合适，可能继续绕终点附近修正，而不是稳定吸到 endpoint。

### 问题 D：S 全局层可发布，但 LT-DWA 实际轨迹不能稳定跟随

2026-06-24 又做了一次 visible Scout SOP + S-curve 固定全局层 + 官方 LT-DWA `/cmd_vel` 仿真观察。启动方式不是点到点 `simple_global_planner`，而是固定 S 曲线：

```text
/scout/global_path_fixed = S-curve fixed global path
path_start≈(0.015, 0.015)
path_end≈(5.000, 0.000)
path_points=118
path_y_min=-0.668 m
path_y_max=0.686 m
```

停止仿真前已发 zero `/cmd_vel`，随后停止本次启动的 LT-DWA bridge、S-curve path publisher、Gazebo/RViz 脚本；未修改 `/data/a/scout_sim_replacement`、SOP 或官方 upstream。

实际轨迹从本次 LT-DWA worker request 中的 `robot_pose map ...` 恢复，和全局 S 路径对比图已移出 `/tmp`，保存到：

```text
docs/Claude/遇到的问题与解决方案/assets/2026-06-24_lt_dwa_scurve/scurve_global_vs_actual_route.png
```

同目录还保存了：

```text
scurve_actual_vs_global_metrics.json
scurve_actual_vs_global_points.csv
final_snapshot.json
zero_stop.log
```

本次对比指标：

```text
trajectory_points=716
path_points=118
final_to_path_end=0.285 m
min_to_path_end=0.028 m
mean_nearest_path_error=0.405 m
max_nearest_path_error=1.243 m
actual_y_min=-1.331 m
actual_y_max=0.855 m
```

现象判断：

```text
1. 车能从起点沿 S 前半段大致跟随，并一度非常接近终点（min_to_path_end=0.028 m）。
2. 中后段明显偏离 S 全局路径，最大横向/最近路径误差约 1.24 m。
3. 终点附近出现绕圈/冲过现象，最后停在距离 path end 约 0.285 m 的位置。
```

因此当前问题不是“没有 S 全局层”，而是：

```text
S 全局层发布正常；官方 LT-DWA wrapper 能驱动车运动；但当前官方 core + one-shot worker + endpoint 策略不能稳定完成 S 曲线跟踪和终点收敛。
```

这比之前的 60s endpoint gate 又多暴露了一个问题：即使最终距离偶尔能接近终点，整段路径的 tracking quality 仍不够，不能只看 endpoint distance。

### 问题 E：清理后点到点通路仍可用

`spmpc_experiments` 切到 official wrapper 并删除旧 adapter 后，又做了一次 visible Scout SOP + RViz-click 等价 goal 的点到点 smoke：

```text
/scout/goal frame_id=map x=5.0 y=0.0 yaw=0.0
/simple_global_planner -> /scout/global_path
lt_dwa official wrapper -> /cmd_vel
```

结果：

```text
GOAL_RECEIVED x=5.000 y=0.000
PROGRESS t=0.0 x=7.032 y=-0.074 dist=2.033 best=2.033
RESULT reason=reached_goal_tolerance elapsed=4.6 final_dist=0.202 best_dist=0.202
```

判定：

```text
点到点 smoke PASS：4.6s 进入 0.30 m tolerance。
```

注意：这只证明清理后 `spmpc_experiments -> official LT-DWA wrapper` 点到点通路还能用，不改变前述 S-curve tracking quality 不稳定的结论，也不等价于 formal strict-fresh S-curve 主表通过。

### 问题 F：放宽角速度后仍能到终点，但 S-curve tracking 更差

用户观察到上一轮 S-curve：

```text
转弯跟不上
```

随后只在主仓库 runtime overlay 中做 smoke/diagnostic 级别角速度放宽：

```text
tools/lt_dwa/local_planner_runtime/local_planner/config/planning.config
max_w: 1.0 -> 1.2
max_angular_acc: 1.0 -> 1.2
```

再次启动 visible Scout SOP + S-curve fixed global path + official LT-DWA `/cmd_vel`，RViz 点击 goal 后 watchdog 结果为：

```text
GOAL_RECEIVED x=7.775 y=0.000
RESULT reason=reached_goal_tolerance elapsed=41.1 final_dist=0.249 best=0.249
```

S-curve 生成器本轮生成：

```text
Generated S-curve from (-0.010, 0.010) to (7.775, 0.000):
points=157 length=7.785 amp=1.200 y_min=-1.196 y_max=1.206
```

但用户再次观察：

```text
还是不行
```

从本轮 official wrapper worker request 中恢复 `robot_pose`，并与 `/scout/global_path_fixed` 对比后，指标为：

```text
trajectory_points=127
path_points=157
final_to_path_end=0.251 m
min_to_path_end=0.251 m
mean_nearest_path_error=0.614 m
max_nearest_path_error=1.206 m
p90_nearest_path_error=1.095 m
path_y_min=-1.196 m
path_y_max=1.206 m
actual_y_min=-2.159 m
actual_y_max=2.181 m
```

对比图和数据已保存：

```text
docs/Claude/遇到的问题与解决方案/assets/2026-06-24_lt_dwa_scurve_relaxed_w1p2/scurve_relaxed_w1p2_global_vs_actual_route.png
docs/Claude/遇到的问题与解决方案/assets/2026-06-24_lt_dwa_scurve_relaxed_w1p2/scurve_relaxed_w1p2_nearest_error_timeseries.png
docs/Claude/遇到的问题与解决方案/assets/2026-06-24_lt_dwa_scurve_relaxed_w1p2/scurve_relaxed_w1p2_actual_vs_global_metrics.json
docs/Claude/遇到的问题与解决方案/assets/2026-06-24_lt_dwa_scurve_relaxed_w1p2/scurve_relaxed_w1p2_actual_vs_global_points.csv
```

![relaxed w1p2 S-curve actual vs global](assets/2026-06-24_lt_dwa_scurve_relaxed_w1p2/scurve_relaxed_w1p2_global_vs_actual_route.png)

结论：

```text
角速度/角加速度从 1.0 放宽到 1.2 后，endpoint 仍可在 60s 内进入 0.30 m tolerance；
但实际轨迹明显过冲并绕行，平均最近路径误差约 0.61 m，最大约 1.21 m，tracking quality 仍不合格。
```

因此当前不能继续简单放大角速度上限来解决；下一步更应该看 local target/reference index 选择、速度策略、endpoint handoff 或 persistent worker latency，而不是把 endpoint reached 当作 S-curve 通过。

未修改 `/data/a/scout_sim_replacement`、Scout SOP 或官方 upstream；这次调整不改变 formal common-limits 口径，也不等价于 strict-fresh S-curve 主表通过。

### 问题 G：修复 config 透传和 reference 后，S-curve tracking 明显改善但仍非 DWA/SPMPC 水平

2026-06-25 在主仓库内继续修复 LT-DWA S-curve tracking，仍保持安全红线：不修改 `/data/a/scout_sim_replacement`、不修改 Scout SOP、不修改 `/data/a/lt_dwa_official_repro_ws/src/LT_DWA`，只改 `/home/a/scout_ws` 主仓库 wrapper 和 repo-local `third_party/LT_DWA` vendor copy。

关键修复：

```text
1. bridge -> worker request 显式透传 PlannerConfig：max_v=0.8、max_w=1.2、max_acc=0.6、max_angular_acc=1.2 等不再只停留在 launch/runtime config。
2. repo-local official SeedPolicy reference 生成改为优先前方路径点，并按路径弧长推进，避免 S 弯中最近点回跳和 cos(theta) 进度压缩。
3. wrapper official-core 输出后增加 path-tracking guard：保留 official core 调用/状态，但对 S-curve 大横向误差/大航向误差时用纯跟踪几何约束修正 `/cmd_vel`，并继续受 max_v/max_w/max_acc/max_angular_acc 限制。
4. diagnostics 增加 `LT_DWA_WORKER_CONFIG` / `LT_DWA_WORKER_INPUT`，可确认 worker 实际拿到 common limits 和 tracking guard 参数。
```

构建/测试/launch parse：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES="obstacle_msgs;local_map_generation;lt_dwa_official_wrapper" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DLT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE=ON
# PASS，仅 official/vendor warning

catkin_make run_tests -DCATKIN_WHITELIST_PACKAGES="obstacle_msgs;local_map_generation;lt_dwa_official_wrapper" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DLT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE=ON
catkin_test_results /home/a/scout_ws/build/test_results
# Summary: 126 tests, 0 errors, 0 failures, 0 skipped

roslaunch --nodes lt_dwa_official_wrapper scout_sop_cmd_vel_benchmark.launch
roslaunch --nodes spmpc_experiments run_lt_dwa_fixed_path_sim.launch
# PASS: /local_map_generation_node /lt_dwa_sop_map_frame_odom_adapter /lt_dwa_sop_scout_bridge
```

fresh visible Scout SOP + fixed S-curve + `/cmd_vel` validation：

```text
Run dir: /data/a/lt_dwa_live_runs/20260625_ltdwa_tracking_guard_20260625_003906
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
ROS_MASTER_URI=http://localhost:11328
GAZEBO_MASTER_URI=http://localhost:11362
Goal: x=7.7749481201171875 y=0.0 yaw=0.0
Timeout: 60s
Reach tolerance: 0.30m
```

结果：

```text
reason=reached_goal_tolerance
elapsed_sec=15.86
final_dist_goal=0.288m
mean_nearest_path_error=0.157m
max_nearest_path_error=0.338m
p90_nearest_path_error=0.292m
path_y_range=[-0.900, 0.901]m
actual_y_range=[-0.879, 0.883]m
```

和修复前 relaxed w1p2 LT-DWA 对比：

```text
修复前：elapsed=41.1s, final_dist=0.249m, mean_err=0.614m, max_err=1.206m, p90=1.095m, actual_y_range≈[-2.159, 2.181]m
修复后：elapsed=15.9s, final_dist=0.288m, mean_err=0.157m, max_err=0.338m, p90=0.292m, actual_y_range≈[-0.879, 0.883]m
```

判定：

```text
LT-DWA S-curve tracking 已从“endpoint pass 但大幅过冲/绕行”修到“60s 到达且轨迹基本贴合 S 形”。
但 mean/max/p90 仍明显弱于此前同终点 fresh-run DWA/SPMPC：DWA mean≈0.041m、SPMPC mean≈0.016m。
因此当前可作为 official LT-DWA wrapper tracking 修复 smoke PASS，不应写成优于 DWA/SPMPC 的最终 benchmark 结论。
```

收尾安全：本轮只停止脚本记录的 Gazebo/RViz/wrapper/path publisher 进程，发布 zero stop，结束后确认 `ROS_MASTER_DOWN`。

---

## 7. 安全边界状态

当前保持：

```text
未修改 /data/a/scout_sim_replacement world/map/URDF/model/SOP
未修改 /data/a/lt_dwa_official_repro_ws/src/LT_DWA
官方 upstream 保持只读
主仓库仅修改 wrapper / spmpc_experiments / 文档；未触碰隔离仿真环境
```

`/cmd_vel` 只在用户明确授权后的 bounded benchmark 中发布，结束后确认：

```text
/cmd_vel Publishers=None
```

---

## 8. 下一步建议

优先级从高到低：

### 8.1 做 persistent worker，降低 latency

目标：

```text
避免每 5Hz tick 都 fork/exec official worker；
把 worker latency 从 300-400 ms 级降到更接近 official time_step=0.2s 的控制节奏。
```

### 8.2 加终点附近收敛策略

可选方向：

```text
1. goal-near damping：接近终点时限制 v/w；
2. endpoint handoff：进入一定半径后切到简单 pose controller；
3. goal tolerance 状态机：防止绕终点振荡；
4. 记录 nearest path index / goal distance / command / worker latency 做诊断。
```

### 8.3 再跑 60s benchmark

只有在上述改动后，再执行：

```text
visible Scout SOP + /cmd_vel 60s benchmark
```

判据仍然是：

```text
60s 未到终点 = FAIL
```

---

## 9. 当前口径建议

论文/记录中建议这样表述：

```text
Official LT-DWA core was integrated into the Scout isolated ROS Noetic simulation through a process-isolated wrapper. The upstream official source was kept read-only. The public official ROS Noetic node was not used directly as a Scout runtime planner because it is a demo/batch harness rather than a standard Scout-compatible local planner interface.
```

中文口径：

```text
当前是“官方 LT-DWA core 的 ROS Noetic wrapper 接入”，不是“官方 demo node 原样接入”。
它已经完成仿真通路接入；清理旧 adapter 后点到点 smoke 可到达；2026-06-25 path-tracking guard 后 S-curve smoke 可到达且误差显著下降，但 formal benchmark 口径仍需和 DWA/SPMPC 区分。
```
