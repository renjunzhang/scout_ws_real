# 官方 LT-DWA 接入 Scout 隔离仿真环境当前状态

**日期：** 2026-06-24

**目标：** 在不破坏 `/data/a/scout_sim_replacement` 仿真环境和官方 LT-DWA upstream 的前提下，把官方 LT-DWA ROS Noetic 实现接入 Scout 隔离仿真 SOP，并作为后续 baseline/benchmark 的候选。

---

## 1. 一句话结论

当前状态是：

```text
官方 LT-DWA core 已经接入 Scout 隔离仿真环境，仿真通路能跑通，/cmd_vel 授权 benchmark 已执行一次；
但 60s endpoint-reaching gate 仍未通过，结果为 FAIL。
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
它已经完成仿真通路接入；清理旧 adapter 后点到点 smoke 可到达，但 S-curve tracking quality / formal strict gate 仍未通过。
```
