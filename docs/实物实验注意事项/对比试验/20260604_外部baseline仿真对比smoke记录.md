# 20260604 外部 baseline 仿真对比 smoke 记录

本文记录在开发机上对 `SPMPC / TEB / DWA / mpc_local_planner` 做的第一轮仿真 smoke。目的不是产出正式论文指标，而是先确认外部 baseline 的 ROS 接口、TF、path、goal、`/cmd_vel` 和录包链路是否能跑通，避免直接上实物。

结论先写在前面：

```text
当前不能直接上实物。
TEB 在修正 path/topic、base_frame 和 plan timestamp 后已经能稳定加载并输出非零 /cmd_vel，但仍有 TRACKING/GOAL_REACHED 抖动，需要限速与容差校准。
DWA 已能加载 path，但仍持续 NO_VALID_CMD 或零速度，需要继续查 transformed plan / local costmap / DWA 参数。
mpc_local_planner runner 仍会退出，尚未跑通。
SPMPC B_ours 在 fixed-path continuous smoke 已跑通；P2P baseline wrapper 下仍启动即 GOAL_REACHED，P2P 入口不作为当前主结论。
```

---

## 1. 仿真条件

工作区：

```text
/home/a/scout_ws
```

分支：

```text
experiment/georef-mpc-hybrid
```

仿真环境：

```text
SIM_ENV=open
USE_RVIZ=false
ENABLE_ODOM_TF_BRIDGE=true
SPAWN_X=-4.0
SPAWN_Y=0.0
SPAWN_Z=0.1
SPAWN_YAW=0.0
```

目标点：

```text
GOAL_X=-1.2
GOAL_Y=2.6
GOAL_YAW=1.0
```

每组录包时间：

```text
RECORD_SEC=15s
```

输出目录：

```text
/data/a/spmpc_baseline_sim_test
```

统一 smoke 入口：

```bash
src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

---

## 2. 本轮 smoke 前修正的接口问题

第一轮启动 TEB/DWA/MPC 后，发现外部 baseline 不发 `/cmd_vel`。排查后主要是 P2P baseline 入口的接口没有和当前仿真栈对齐。

已做如下修正：

### 2.1 P2P global path topic 对齐

当前 P2P 仿真栈发布的是：

```text
/scout/global_path
```

原 TEB/DWA/MPC baseline launch 默认订阅的是：

```text
/scout/global_path_fixed
```

这会导致 runner 拿不到路径。已将以下 launch 的默认路径改为 `/scout/global_path`：

```text
src/scout_apps/control/spmpc_experiments/launch/sim/run_teb_p2p_sim.launch
src/scout_apps/control/spmpc_experiments/launch/sim/run_dwa_p2p_sim.launch
src/scout_apps/control/spmpc_experiments/launch/sim/run_mpc_local_planner_p2p_sim.launch
```

### 2.2 baseline local costmap base frame 对齐

仿真中可用 TF 是：

```text
odom -> base_footprint
```

原 baseline local costmap 使用：

```yaml
robot_base_frame: base_link
```

会出现 costmap 找不到 `odom -> base_link` 的问题。已改为：

```yaml
robot_base_frame: base_footprint
```

对应文件：

```text
src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_sim.yaml
```

### 2.3 baseline runner 暴露 base_frame 参数

`baseline_local_planner_runner` 内部 goal fallback 也需要 robot base frame。已在 launch 中加入：

```xml
<arg name="base_frame" default="base_footprint"/>
<param name="base_frame" value="$(arg base_frame)"/>
```

对应文件：

```text
src/scout_apps/control/baseline_local_planner_runner/launch/nav_core_runner.launch
```

### 2.4 SPMPC smoke 入口默认使用 continuous backend

为了避免 SPMPC baseline smoke 又回到 primitive，已给脚本增加：

```bash
SPMPC_SOLVER_BACKEND=continuous_mpcc_acados
SPMPC_W_SLOSH=-1.0
```

并把 metadata 写入：

```yaml
spmpc_solver_backend: continuous_mpcc_acados
spmpc_w_slosh: -1.0
```

对应文件：

```text
src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

---

## 3. 运行命令

每个 baseline 都重新启动一次仿真，保证从同一 spawn 起点开始。

仿真栈启动命令：

```bash
SIM_ENV=open USE_RVIZ=false ENABLE_ODOM_TF_BRIDGE=true \
SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
GAZEBO_WAIT_S=8 SENSOR_WAIT_S=2 LOCALIZATION_WAIT_S=3 \
rosrun scout_local_planner launch_sim_nav_stack.sh
```

SPMPC：

```bash
BASELINE=spmpc VARIANT=B_ours \
SPMPC_SOLVER_BACKEND=continuous_mpcc_acados \
RECORD_SEC=15 \
OUT_DIR=/data/a/spmpc_baseline_sim_test \
RUN_ID=20260604_sim_spmpc_B_ours_final \
bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

TEB：

```bash
BASELINE=teb \
RECORD_SEC=15 \
OUT_DIR=/data/a/spmpc_baseline_sim_test \
RUN_ID=20260604_sim_teb_final \
bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

DWA：

```bash
BASELINE=dwa \
RECORD_SEC=15 \
OUT_DIR=/data/a/spmpc_baseline_sim_test \
RUN_ID=20260604_sim_dwa_final \
bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

mpc_local_planner：

```bash
BASELINE=mpc \
RECORD_SEC=15 \
OUT_DIR=/data/a/spmpc_baseline_sim_test \
RUN_ID=20260604_sim_mpc_final \
bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

---

## 4. 结果汇总

| 方法 | 是否加载 | 是否有 global path | 是否发布 `/cmd_vel` | 运动情况 | 状态 | 初步结论 |
|---|---|---|---|---|---|---|
| `SPMPC B_ours` | 是 | 是，`/scout/global_path` 213 poses | 有 432 帧，但全 0 | 基本不动，net `0.008 m` | 一直 `GOAL_REACHED` | P2P wrapper 入口不可靠；fixed-path continuous smoke 已验证通过 |
| `TEB` | 是 | 是，212 poses | 有 143 帧，55 帧非零 | net `0.778 m`，path `2.124 m` | `TRACKING / GOAL_REACHED / NO_VALID_CMD` 交替 | 接口基本跑通，但状态震荡，速度偏激进，不能直接上实物 |
| `DWA` | 是 | 是，212 poses | 有 143 帧，25 帧非零 | net `0.074 m`，path `0.141 m` | `TRACKING / NO_VALID_CMD / GOAL_REACHED` 交替 | 能发命令但几乎不前进，需要调采样/速度/代价参数 |
| `mpc_local_planner` | 启动后退出 | 是，212 poses | 无 | 基本不动，net `0.006 m` | 无 status | runner/plugin/solver 配置尚未跑通 |

---

## 5. 关键数值

从 bag 中统计：

```text
SPMPC B_ours:
  cmd_count    = 432
  nonzero_cmd  = 0
  vmax         = 0.000 m/s
  wmax         = 0.000 rad/s
  net_motion   = 0.008 m
  status       = GOAL_REACHED

TEB:
  cmd_count    = 143
  nonzero_cmd  = 55
  vmax         = 1.462 m/s
  wmax         = 1.600 rad/s
  net_motion   = 0.778 m
  path_motion  = 2.124 m
  status       = GOAL_REACHED / TRACKING / NO_VALID_CMD 交替

DWA:
  cmd_count    = 143
  nonzero_cmd  = 25
  vmax         = 0.076 m/s
  wmax         = 1.200 rad/s
  net_motion   = 0.074 m
  path_motion  = 0.141 m
  status       = TRACKING / NO_VALID_CMD / GOAL_REACHED 交替

mpc_local_planner:
  cmd_count    = 0
  nonzero_cmd  = 0
  net_motion   = 0.006 m
  status       = 无
  planner log  = baseline_local_planner_runner exit code 1
```

---

## 6. 录包与日志

有效记录目录：

```text
/data/a/spmpc_baseline_sim_test
```

最终 smoke 文件：

```text
20260604_sim_spmpc_B_ours_final.bag
20260604_sim_spmpc_B_ours_final_planner.log
20260604_sim_spmpc_B_ours_final_meta.yaml

20260604_sim_teb_final.bag
20260604_sim_teb_final_planner.log
20260604_sim_teb_final_meta.yaml

20260604_sim_dwa_final.bag
20260604_sim_dwa_final_planner.log
20260604_sim_dwa_final_meta.yaml

20260604_sim_mpc_final.bag
20260604_sim_mpc_final_planner.log
20260604_sim_mpc_final_meta.yaml
```

---

## 7. 当前判断

### 7.1 SPMPC

`B_ours` 的 fixed-path continuous smoke 已在同一天通过，四组主变体 `B0 / B_smooth / B_slosh / B_ours` 都能跑到 `GOAL_REACHED`，且 `B_ours` 的 observer slosh 最低。

但本轮 `spmpc_experiments` 的 P2P baseline wrapper 下，`SPMPC B_ours` 启动后一直 `GOAL_REACHED`，没有输出非零 `/cmd_vel`。这说明 P2P baseline 入口和当前 continuous MPCC fixed-path 主线还没有完全对齐。正式比较中，SPMPC 主方法仍应优先使用 fixed-path continuous 入口。

### 7.2 TEB

TEB 已经能：

```text
加载 plugin
接收 global path
发布 /cmd_vel
驱动车体移动
```

但状态在 `TRACKING / GOAL_REACHED / NO_VALID_CMD` 之间跳，且峰值速度较大：

```text
vmax = 1.462 m/s
wmax = 1.600 rad/s
```

这对液体运输实物实验过于激进。下一步需要统一速度限制、目标容差、global plan prune 和 local costmap 参数。

### 7.3 DWA

DWA 已经能发 `/cmd_vel`，但实际前进很少：

```text
net_motion = 0.074 m
```

说明当前 DWA standalone 参数过保守或经常无有效轨迹。下一步需要调：

```text
sim_time
vx_samples
vtheta_samples
path_distance_bias
goal_distance_bias
occdist_scale
min/max velocity
xy_goal_tolerance
```

### 7.4 mpc_local_planner

`mpc_local_planner` 当前没有跑通，runner 进程退出：

```text
baseline_local_planner_runner exit code 1
```

需要继续查：

```text
plugin 初始化
IPOPT/solver 依赖
mpc_local_planner 参数命名空间
costmap / footprint / odom 配置
```

---

## 8. 下一步建议

不要直接进入实物外部 baseline 对比。建议先完成下面三步：

```text
1. 修 SPMPC P2P baseline wrapper，或者明确外部 baseline 都改成 fixed-path 输入，保证比较任务一致。
2. 调 TEB 和 DWA 参数，使它们在仿真中稳定 TRACKING，不频繁 NO_VALID_CMD/GOAL_REACHED 抖动。
3. 修 mpc_local_planner runner 退出问题，至少达到能发 /cmd_vel、能录状态、能完成一次 P2P smoke。
```

达到下面标准后再上实物：

```text
每个 baseline 至少 3 次仿真重复
每次都能正常发 /cmd_vel
速度/角速度不超过 Scout 实物安全限制
无持续 NO_VALID_CMD
能完成路径或明确到达目标
bag 中有 /cmd_vel /odom /global_path /baseline/status /spmpc 或 slosh observer 指标
```

---

## 9. 本轮最终结论

```text
这轮 smoke 的价值是把外部 baseline 接口问题暴露出来了。
TEB 和 DWA 已经不是“完全跑不起来”，但还没到可公平对比和可上实物的程度。
mpc_local_planner 还没跑通。
SPMPC continuous 主线 fixed-path 已验证，但 P2P baseline wrapper 需要补齐。
```


---

## 10. 继续修正与复跑记录

在第一轮 smoke 后，继续修正并复跑了外部 baseline，所有大体积 bag/log 均放在 `/data/a` 下。

### 10.1 继续修正的代码/配置

#### baseline runner 不再过早相信 plugin 的 `isGoalReached()`

部分 nav_core plugin 在 plan/goal 刚设置或局部规划失败时会短暂返回 `isGoalReached()`，导致 wrapper 误判到达。runner 现在只在下面条件下发布 `GOAL_REACHED`：

```text
1. wrapper 自己根据 goal 和 robot pose 判断确实到达；或
2. 没有显式 goal 时才退回使用 plugin->isGoalReached()
```

对应文件：

```text
src/scout_apps/control/baseline_local_planner_runner/src/baseline_local_planner_runner_node.cpp
```

#### plan timestamp 清零，避免 TF extrapolation

TEB/DWA 在 transform global plan 时遇到：

```text
Lookup would require extrapolation ... into the future
```

原因是 global path 中的 pose stamp 使用了当前仿真时间，局部 planner 计算时 TF 最新时间略滞后。runner 现在将传给 plugin 的 global plan pose stamp 统一置为：

```cpp
ros::Time(0)
```

这样让 TF 使用最近可用变换，避免 0.01s 级别的未来外推错误。

#### local costmap 改为 map frame，并扩大窗口

为了减少 map/odom 变换和 DWA transformed plan 为空的问题，baseline local costmap 改为：

```yaml
local_costmap:
  global_frame: map
  robot_base_frame: base_footprint
  width: 20.0
  height: 20.0
```

对应文件：

```text
src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_sim.yaml
```

#### DWA global frame 对齐为 map

DWA standalone 配置同步改为：

```yaml
DWAPlannerROS:
  global_frame_id: map
```

对应文件：

```text
src/scout_apps/control/spmpc_experiments/config/baselines/dwa_local_planner_standalone_sim.yaml
```

---

### 10.2 复跑输出目录

第二轮、第三轮、第四轮复跑输出分别在：

```text
/data/a/spmpc_baseline_sim_test_rerun
/data/a/spmpc_baseline_sim_test_rerun2
/data/a/spmpc_baseline_sim_test_rerun3
/data/a/spmpc_baseline_sim_test_rerun4
```

当前最有参考价值的 bag：

```text
TEB:
/data/a/spmpc_baseline_sim_test_rerun3/20260604_rerun3_teb_mapcostmap.bag

DWA:
/data/a/spmpc_baseline_sim_test_rerun4/20260604_rerun4_dwa_widecostmap.bag

mpc_local_planner:
/data/a/spmpc_baseline_sim_test_rerun2/20260604_rerun2_mpc.bag
```

---

### 10.3 修正后结果

| 方法 | 当前最好一轮 | `/cmd_vel` | 运动 | 状态 | 结论 |
|---|---|---|---|---|---|
| `TEB` | `rerun3_teb_mapcostmap` | 243 帧，28 帧非零 | net `0.509 m`，path `0.874 m` | `TRACKING / GOAL_REACHED` 交替 | 已能真实输出并运动，但仍需限速和目标容差校准 |
| `DWA` | `rerun4_dwa_widecostmap` | 244 帧，全 0 | net `0.014 m` | `GOAL_REACHED / NO_VALID_CMD` 交替 | 仍未跑通；global plan 有，但 DWA transformed plan 仍为空或无有效轨迹 |
| `mpc_local_planner` | `rerun2_mpc` | 0 帧 | net `0.014 m` | 无 status | runner/plugin 仍退出，需要继续查 plugin/solver 初始化 |

关键数值：

```text
TEB rerun3:
  cmd_count    = 243
  nonzero_cmd  = 28
  vmax         = 0.652 m/s
  wmax         = 1.436 rad/s
  net_motion   = 0.509 m
  path_motion  = 0.874 m
  status       = TRACKING / GOAL_REACHED 交替

DWA rerun4:
  cmd_count    = 244
  nonzero_cmd  = 0
  vmax         = 0.000 m/s
  wmax         = 0.000 rad/s
  net_motion   = 0.014 m
  status       = GOAL_REACHED / NO_VALID_CMD 交替

mpc_local_planner rerun2:
  cmd_count    = 0
  net_motion   = 0.014 m
  status       = 无
```

---

### 10.4 当前工程判断

```text
TEB：已经从“接口不通”推进到“能发命令、能让车动”，下一步是调稳定性和安全速度。
DWA：接口基本接上，但 DWA 自己仍拿不到有效 transformed plan/trajectory，不能作为实物候选。
mpc_local_planner：仍是未跑通状态，优先级低于 TEB/DWA。
SPMPC：主线仍使用 fixed-path continuous MPCC 入口，不用这套外部 baseline P2P wrapper 评价主方法。
```

当前仍不建议直接做外部 baseline 实物对比。下一步应优先做：

```text
1. TEB 限速：max_vel_x <= 0.8, max_vel_theta <= 1.2，并放宽/统一 goal tolerance，减少 GOAL_REACHED 抖动。
2. DWA 继续查 transformed plan 为空的问题；必要时给 DWA 单独走直线 plan fallback，不依赖 MBF 发布路径。
3. mpc_local_planner 单独启动并查看 node log，先解决 runner exit code 1。
4. 等 TEB/DWA 至少能连续 TRACKING 20~30s，再做正式多次仿真统计。
```

---

## 11. 20260605 继续调试记录

### 11.1 runner 侧新增的仿真 baseline 适配

为避免外部 baseline 被 `map -> odom` 定位跳变和 nav_core plugin 的早期 `isGoalReached()` 影响，继续对 `baseline_local_planner_runner` 做了以下仿真入口适配：

```text
1. path 接收后可按 plan_target_frame 转成 odom 帧并冻结，避免后续 map->odom 跳变影响局部控制。
2. path 末端可作为 fallback goal，避免 path 先于 /scout/goal 到达时 plugin 误报 GOAL_REACHED。
3. DWA/mpc_local_planner 可在收到 goal 后强制生成 odom 直线 plan，绕开外部 global path 起点和 TF 不一致的问题。
4. runner 增加 /cmd_vel 限幅：默认 max_cmd_vel_x=0.8, max_cmd_vel_theta=1.2。
5. runner 增加 stderr 失败输出，便于捕获 pluginlib 初始化异常。
```

涉及文件：

```text
src/scout_apps/control/baseline_local_planner_runner/src/baseline_local_planner_runner_node.cpp
src/scout_apps/control/baseline_local_planner_runner/launch/nav_core_runner.launch
src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_sim.yaml
src/scout_apps/control/spmpc_experiments/launch/sim/run_teb_p2p_sim.launch
src/scout_apps/control/spmpc_experiments/launch/sim/run_dwa_p2p_sim.launch
src/scout_apps/control/spmpc_experiments/launch/sim/run_mpc_local_planner_p2p_sim.launch
```

### 11.2 TEB 复跑结果

新增 TEB standalone 仿真配置：

```text
src/scout_apps/control/spmpc_experiments/config/baselines/teb_local_planner_standalone_sim.yaml
```

该配置把 TEB 限速到：

```text
max_vel_x = 0.8
max_vel_theta = 1.2
acc_lim_x = 0.6
acc_lim_theta = 1.2
```

复跑 bag：

```text
/data/a/spmpc_baseline_sim_test_rerun6/20260605_rerun6_teb_odomplan_limited.bag
```

统计结果：

```text
TEB rerun6:
  plan_frame   = odom
  cmd_count    = 206
  nonzero_cmd  = 206
  vmax         = 0.752 m/s
  wmax         = 1.200 rad/s
  net_motion   = 6.216 m
  path_motion  = 8.827 m
  status       = TRACKING
```

结论：

```text
TEB 已经从“能动但抖动”推进到“连续 TRACKING 且速度被限制”。
这是目前外部 baseline 里最接近可继续做正式仿真统计的一组。
但仍建议至少再做 3 次重复仿真，再考虑实物。
```

### 11.3 DWA 复跑结果

DWA 侧做了这些处理：

```text
1. plan_target_frame = odom
2. force_straight_plan_on_goal = true
3. use_wrapper_goal_check = false
4. prune_plan = false
5. global_frame_id = odom
```

关键原因：

```text
DWA 的 getLocalPlan() 会在 prune_plan=true 时按 1m 距离裁剪 plan。
当前仿真 TF 和 /odom topic 存在不一致，DWA 看到的 robot pose 与全局 path 起点可能相差超过 1m，导致 transformed_plan 被 prune 成空。
关闭 prune_plan 后，DWA 不再持续出现 empty transformed plan / zero length plan。
```

代表性复跑：

```text
/data/a/spmpc_baseline_sim_test_rerun6/20260605_rerun6_dwa_noprune_straight.bag
```

统计结果：

```text
DWA rerun6 noprune:
  cmd_count    = 244
  nonzero_cmd  = 92
  vmax         = 0.382 m/s
  wmax         = 2.873 rad/s  # clamp 前记录，过高
  net_motion   = 0.877 m
  path_motion  = 2.601 m
  status       = TRACKING / GOAL_REACHED / NO_VALID_CMD
```

加上 runner clamp 和关闭 wrapper goal check 后，DWA 不再持续 `NO_VALID_CMD`，但由于 TF/odom 不一致和 DWA 自身 stop-rotate 判断，实际位移仍很小：

```text
/data/a/spmpc_baseline_sim_test_rerun6/20260605_rerun6_dwa_final_goalcheck_off.bag

DWA final goalcheck_off:
  cmd_count    = 243
  nonzero_cmd  = 23
  vmax         = 0.009 m/s
  wmax         = 0.435 rad/s
  net_motion   = 0.041 m
  path_motion  = 0.068 m
  status       = TRACKING
```

结论：

```text
DWA 的 transformed plan 空问题已经定位到 prune/TF/plan 起点不一致，并通过 odom straight plan + prune_plan=false 缓解。
但 DWA 还没有达到可公平对比状态：它能进入 TRACKING，但有效前进仍不足。
下一步需要优先处理仿真中 /odom topic 与 TF odom->base_footprint 不一致的问题，否则 DWA 的 goal/stop-rotate 判断会失真。
```

### 11.4 mpc_local_planner 复跑结果

给 runner 增加 stderr 失败输出后，终于拿到了明确原因。

复跑：

```text
/data/a/spmpc_baseline_sim_test_rerun6/20260605_rerun6_mpc_stderr_planner.log
```

关键错误：

```text
[baseline_runner] failed: Could not find library corresponding to plugin mpc_local_planner/MpcLocalPlannerROS.
Make sure the plugin description XML file has the correct name of the library and that the library actually exists.
```

继续尝试单独编译 `mpc_local_planner`：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="mpc_local_planner;mpc_local_planner_msgs;baseline_local_planner_runner" --pkg mpc_local_planner
```

编译失败原因：

```text
The dependency target "corbo_core" of target "mpc_local_planner_utils" does not exist.
The dependency target "corbo_systems" of target "mpc_local_planner_utils" does not exist.
```

结论：

```text
mpc_local_planner 当前不是参数问题，而是 plugin 库没有成功构建/导出。
其依赖 control_box_rst / corbo_core / corbo_systems 未在当前 catkin 构建环境中就绪。
在补齐并成功编译这些依赖前，mpc_local_planner 不能参与仿真或实物对比。
```

### 11.5 当前更新后的工程判断

```text
TEB：目前已能作为外部 baseline 的下一步仿真统计候选。
DWA：接口和 transformed plan 问题已定位并部分缓解，但仍需处理 TF/odom 不一致与有效前进不足。
mpc_local_planner：缺 corbo/control_box_rst 构建链路，暂时不能跑。
SPMPC：主方法仍使用 fixed-path continuous MPCC 入口，不使用 P2P wrapper 作为主结论。
```

---

## 12. 20260605 SPMPC P2P wrapper 修正记录

继续修正 `SPMPC` 点到点 wrapper。之前 `BASELINE=spmpc VARIANT=B_ours` 在 P2P wrapper 下启动后容易直接进入 `GOAL_REACHED`，`/cmd_vel` 全 0，不能代表 continuous MPCC 主方法。

### 12.1 修正内容

主要修了两类问题：

```text
1. spmpc_experiments 的 run_spmpc_p2p_sim.launch 原来没有显式声明/转发 solver_backend 与 w_slosh。
   现在 P2P wrapper 会把 continuous_mpcc_acados 和 w_slosh 传到 spmpc_local_planner。

2. SPMPC P2P 输入的 /scout/global_path 是 map frame，但局部控制更适合在 odom frame 中冻结 reference。
   新增 reference_target_frame 参数，P2P 默认把收到的 reference path 转成 odom frame 再送入 SPMPC。
```

涉及文件：

```text
src/scout_apps/control/spmpc_experiments/launch/sim/run_spmpc_p2p_sim.launch
src/scout_apps/control/spmpc_local_planner/launch/spmpc_point_to_point.launch
src/scout_apps/control/spmpc_local_planner/launch/spmpc_experiment.launch
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/ros/spmpc_local_planner_ros.h
src/scout_apps/control/spmpc_local_planner/src/ros/spmpc_local_planner_ros.cpp
```

### 12.2 B_ours P2P 复跑

复跑命令等价于：

```bash
BASELINE=spmpc VARIANT=B_ours \
SPMPC_SOLVER_BACKEND=continuous_mpcc_acados \
SPMPC_W_SLOSH=-1.0 \
OUT_DIR=/data/a/spmpc_baseline_sim_test_rerun7 \
RUN_ID=20260605_rerun7_spmpc_p2p_B_ours_odomref \
bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
```

输出：

```text
/data/a/spmpc_baseline_sim_test_rerun7/20260605_rerun7_spmpc_p2p_B_ours_odomref.bag
```

统计：

```text
SPMPC B_ours P2P rerun7:
  global_path frame = map
  global_path poses = 211
  cmd_count         = 731
  nonzero_cmd       = 548
  vmax              = 0.787 m/s
  wmax              = 1.200 rad/s
  net_motion        = 2.839 m
  path_motion       = 4.366 m
  status            = B_ours_ACADOS_OK / ACADOS_SOLVE_FAILED_4 / GOAL_REACHED
```

结论：

```text
SPMPC B_ours 的 P2P wrapper 已经不再是“启动即 GOAL_REACHED、全 0 cmd”。
现在能输出非零 /cmd_vel，车体能运动，并最终进入 GOAL_REACHED。
```

但仍需注意：

```text
P2P 下 B_ours 仍有间歇 ACADOS_SOLVE_FAILED_4。
这说明 wrapper 已经接通，但 P2P continuous MPCC 的数值稳定性还需要进一步调参或 warm-start 处理。
正式论文主结果暂时仍优先使用 fixed-path continuous MPCC；P2P 可以作为后续扩展入口继续打磨。
```

## 13. 20260605 P2P 稳定化参数与 flatness warm-start 记录

本轮按 `docs/Claude/CLAUDE.md` 的要求做最小、解耦修改：P2P 专用参数写入 `point_to_point.yaml`，reference path 平滑重采样放到独立 `reference` 模块，acados 初值由差速底盘微分平坦性生成，不改 fixed-path 主线。

### 13.1 主要改动

```text
1. P2P 降低 v_max / omega_max：0.45 m/s / 0.75 rad/s。
2. P2P goal_tolerance 放宽到 0.25 m。
3. P2P 下 B_ours/B_ours_anti 的 w_smooth 降到 0.25，w_slosh 降到 1.0。
4. 新增 ReferencePathPreprocessor：近重复点删除、弧长重采样、轻量 moving average、yaw 重算。
5. continuous_mpcc_acados 增加 flatness-based primal warm-start：从平滑参考路径生成 x/u 初值。
6. /cmd_vel.angular.z 按运行时 omega_max 限幅。
```

### 13.2 复跑结果

较干净的一轮输出：

```text
/data/a/spmpc_p2p_stabilization/20260605_spmpc_p2p_B_ours_stabilized_clean.bag
```

统计：

```text
cmd_count    = 880
nonzero_cmd  = 108
vmax         = 0.45 m/s
wmax         = 0.75 rad/s
net_motion   = 1.174 m
path_motion  = 1.197 m
status       = B_ours_ACADOS_OK / ACADOS_SOLVE_FAILED_4 / GOAL_REACHED
ACADOS_SOLVE_FAILED_4 count = 2
progress_s   = 0.738 -> 0.961
```

对比上一轮 rerun7：

```text
旧结果：有多次 ACADOS_SOLVE_FAILED_4，vmax≈0.787，wmax≈1.200。
新结果：速度/角速度已被 P2P 参数限制到 0.45/0.75，ACADOS_SOLVE_FAILED_4 明显减少到 2 次。
```

### 13.3 当前判断

```text
P2P continuous MPCC 的数值稳定性有改善，flatness warm-start 是有效方向；
但这一轮 P2P 轨迹运动距离偏短，仍不建议直接作为正式对比实验结果。
正式对比仍优先 fixed-path；P2P 继续作为扩展入口调试。
```

## 14. 20260605 acados warm-start 模块化重构记录

本轮把 continuous MPCC 的 warm-start 从 solver 内部拆出来，形成 ROS-free `warm_start/` 模块：

```text
1. DiffDriveFlatnessWarmStart 独立生成 x/u horizon 初值。
2. omega 使用曲率关系 omega = kappa * v。
3. v/omega/a/v_s 先限幅再交给 acados。
4. slosh 初值从当前 SloshState 出发，经 SloshDynamics rollout，不再全置零。
5. 预留 MecanumWarmStart 骨架，当前返回 not implemented 并走 fallback。
6. ContinuousMpccSolverAcados 只依赖 WarmStartGenerator 接口，并支持 previous-solution / conservative fallback。
7. 新增 /spmpc/debug/warm_start 与 /spmpc/debug/warm_start_status 诊断。
```

验证：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" --pkg spmpc_local_planner                 PASS
catkin_make -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" run_tests_spmpc_local_planner              PASS, 3/3 gtest
catkin_make -DCATKIN_WHITELIST_PACKAGES="scout_local_planner" --pkg scout_local_planner                  PASS
BAG_DIR=/data/a/spmpc_p2p_stabilization/s_curve_smoke USE_RVIZ=false TRACK_SEC=20 run_s_curve_smoke_test  PASS
P2P B_ours smoke                                                                                          SKIP/FAIL: roscore/仿真栈未启动
```

S-curve smoke bag：

```text
/data/a/spmpc_p2p_stabilization/s_curve_smoke/20260605_123533_s_curve_smoke.bag
```

当前判断：

```text
warm-start 架构已经解耦；diff-drive 单元测试通过；slosh horizon 初值已由动力学 rollout 生成。
P2P 对比 smoke 这一轮没有复跑成功，原因是执行时仿真栈未启动，不是 warm-start 编译或单元测试失败。
```

## 15. 20260605 SPMPC soft terminal stopping 记录

本轮只给 `spmpc_local_planner` 增加 soft terminal stopping，DWA/TEB 暂时不加。设计原则是：terminal policy 作为所有 SPMPC variants 共用的实物安全外壳，不给 `B_ours` 单独加优势；正式液体晃动抑制对比优先统计 terminal 前的主体 tracking 段。

改动摘要：

```text
1. 新增 ROS-free TerminalController：slowdown envelope、capture-stop latch、低速/低角速度 reached gate、rate-limit clamp。
2. GOAL_REACHED 判定从 primitive/acados 后端上移到 SpmpcProblem，B0/B_smooth/B_slosh/B_ours 统一使用。
3. terminal 第一版只做 post-solve cmd_v envelope/rate-limit，不改 cost 权重，不接管 DWA/TEB。
4. 新增 /spmpc/terminal/debug 与 /spmpc/terminal/mode，用 pre_terminal_phase / terminal_phase 切分统计。
```

验证：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" --pkg spmpc_local_planner                 PASS
catkin_make -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" run_tests_spmpc_local_planner              PASS, terminal 6/6 + warm-start 3/3
catkin_make -DCATKIN_WHITELIST_PACKAGES="scout_local_planner" --pkg scout_local_planner                  PASS
BAG_DIR=/data/a/spmpc_terminal_smoke/s_curve_smoke USE_RVIZ=false TRACK_SEC=20 run_s_curve_smoke_test     PASS
P2P B_ours terminal smoke                                                                                 SKIP/FAIL: roscore/仿真栈未启动
```

S-curve smoke bag：

```text
/data/a/spmpc_terminal_smoke/s_curve_smoke/20260605_134632_s_curve_smoke.bag
```

后续正式分析建议：

```text
主体 SPMPC 抑晃能力：只统计 /spmpc/terminal/debug 中 pre_terminal_phase=1 的时间段。
终点实物安全停车：单独统计 terminal_phase=1 的时间段，包括停车距离、cmd_v 变化率、终端液面峰值。
```
