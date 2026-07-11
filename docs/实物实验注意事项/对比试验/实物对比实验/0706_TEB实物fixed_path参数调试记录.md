# 20260706 TEB 实物 fixed-path 参数调试记录

> 目的：记录 2026-07-06 晚上 TEB 外部 baseline 的 fixed-path 实物接入和参数调试过程，方便 2026-07-07 继续调参。本文只记录参数配置与现场判断，不做 bag 深度分析。
>
> **2026-07-10 状态说明：本文保留为历史调参日志，不再作为当前执行 SOP。** 文中的 `/tmp/*.yaml` 已不是可复现配置，`loose_fast` 也尚未经过量化 gate。当前 real/no-obstacle 固化配置、公共限制、录包口径和 formal N=3 流程统一见：
>
> ```text
> docs/实物实验注意事项/对比试验/实物对比实验/0710_TEB实物fixed_path正式化方案.md
> ```
>
> 当前 fixed-path SPMPC 主线同样关闭 obstacle/corridor，因此本文第 9 节“不把 no-obstacle TEB 作为最终公平 baseline”只代表 0706 当晚的保守判断；在当前无障碍主表中，应使用明确标注的 `TEB-noobs-fixed`。原文其余现场事实不回写，以免修改历史记录。

## 1. 总体结论

今天 TEB 已经能正常接入 fixed path 并输出 `/cmd_vel`，但还没有形成最终 formal N=3 参数。

现场判断：

```text
1. 默认 TEB 能跑，但跟踪不紧，有切弯/偏路径。
2. 关闭避障后，TEB 可以贴住 fixed S-curve。
3. tight / balanced 参数都能显著贴路径，但车速偏慢，并有左右摆头。
4. loose_fast 参数方向是对的：放松贴线、提高速度、降低角速度上限；但现场仍觉得太慢，明天继续。
```

因此今天不进入 TEB formal N=3。

## 2. 实验公共口径

固定路径与目标：

```text
PATH_TEMPLATE=s_curve
GOAL_X=-5.424
GOAL_Y=-4.736
GOAL_YAW=0.0
PATH_START_HEADING=current
PATH_AMPLITUDE_RATIO=0.18
PATH_MIN_AMPLITUDE=0.25
PATH_MAX_AMPLITUDE=1.20
PATH_SIDE=left
PATH_SMOOTH_ITERATIONS=3
PATH_SPACING=0.05
```

外部 baseline 脚本：

```bash
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

基础环境：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws
```

记录口径：

```text
RECORD_ALL_EXISTING_TOPICS=true
RECORD_RGB=true   # actuated smoke 使用
RECORD_RGB=false  # shadow 使用
RECORD_TOPIC_INFO=false
RECORDER_STARTUP_SEC=8
RECORD_SEC=60
MAX_RECORD_SEC=60
```

## 3. TEB 默认参数 smoke

### 3.1 shadow

运行标签：

```text
TEB_fixed_0706_shadow01
```

命令：

```bash
METHOD=teb \
STAGE=shadow \
RUN_LABEL=TEB_fixed_0706_shadow01 \
RECORD_ALL_EXISTING_TOPICS=true \
RECORD_RGB=false \
RECORD_SEC=30 \
MAX_RECORD_SEC=30 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

判定：shadow 正常，TEB 能接收 fixed path 并输出 shadow cmd，没有动真车。

### 3.2 actuated smoke

运行标签：

```text
TEB_fixed_0706_smoke01
```

命令：

```bash
METHOD=teb \
STAGE=actuated \
RUN_LABEL=TEB_fixed_0706_smoke01 \
MAX_V=0.20 \
MAX_W=1.2 \
RECORD_ALL_EXISTING_TOPICS=true \
RECORD_RGB=true \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

现场判断：

```text
能跑，但跟踪不紧。
```

当时发现默认 TEB 开了 costmap / obstacle：

```text
local_costmap obstacle_layer: enabled
local_costmap inflation_layer: enabled
TebLocalPlannerROS include_costmap_obstacles: true
scan topic: /scan_front
```

所以后续先关闭避障，只验证纯 fixed-path tracking。

## 4. no-obstacle tight 参数

### 4.1 no-obstacle costmap 临时配置

临时文件：

```text
/tmp/local_costmap_real_no_obstacles.yaml
```

内容：

```yaml
controller_frequency: 10.0

local_costmap:
  global_frame: map
  robot_base_frame: base_link
  footprint: [[0.31, 0.2925], [0.31, -0.2925], [-0.31, -0.2925], [-0.31, 0.2925]]
  transform_tolerance: 2.0
  update_frequency: 5.0
  publish_frequency: 0.0
  static_map: false
  rolling_window: true
  width: 20.0
  height: 20.0
  resolution: 0.05
  plugins:
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

  inflation_layer:
    enabled: false
    cost_scaling_factor: 3.0
    inflation_radius: 0.0
```

含义：

```text
不加载 obstacle_layer；
inflation_layer 存在但 disabled；
TEB 不使用 scan 障碍代价。
```

### 4.2 tight TEB 配置

临时文件：

```text
/tmp/teb_real_fixed_path_tight_no_obstacle.yaml
```

关键参数：

```yaml
TebLocalPlannerROS:
  odom_topic: /odom
  map_frame: map
  transform_tolerance: 1.0

  teb_autosize: true
  dt_ref: 0.20
  dt_hysteresis: 0.05
  max_samples: 500
  global_plan_overwrite_orientation: true
  allow_init_with_backwards_motion: false
  max_global_plan_lookahead_dist: 2.0
  global_plan_prune_distance: 0.15
  global_plan_viapoint_sep: 0.15

  max_vel_x: 0.20
  max_vel_x_backwards: 0.05
  max_vel_theta: 1.2
  acc_lim_x: 0.6
  acc_lim_theta: 1.2

  xy_goal_tolerance: 0.20
  yaw_goal_tolerance: 0.30
  free_goal_vel: false

  min_obstacle_dist: 0.0
  inflation_dist: 0.0
  include_costmap_obstacles: false
  costmap_obstacles_behind_robot_dist: 0.0
  obstacle_poses_affected: 0

  no_inner_iterations: 8
  no_outer_iterations: 5
  penalty_epsilon: 0.02
  weight_acc_lim_x: 5.0
  weight_acc_lim_theta: 5.0
  weight_kinematics_nh: 1000.0
  weight_kinematics_forward_drive: 80.0
  weight_optimaltime: 0.05
  weight_obstacle: 0.0
  weight_viapoint: 20.0

  enable_homotopy_class_planning: false
  max_number_classes: 1
```

### 4.3 shadow

运行标签：

```text
TEB_fixed_0706_noobs_tight_shadow01
```

命令：

```bash
METHOD=teb \
STAGE=shadow \
RUN_LABEL=TEB_fixed_0706_noobs_tight_shadow01 \
PLANNER_CONFIG=/tmp/teb_real_fixed_path_tight_no_obstacle.yaml \
COSTMAP_CONFIG=/tmp/local_costmap_real_no_obstacles.yaml \
MAX_V=0.20 \
MAX_W=1.2 \
RECORD_ALL_EXISTING_TOPICS=true \
RECORD_RGB=false \
RECORD_SEC=30 \
MAX_RECORD_SEC=30 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

判定：shadow 正常。

### 4.4 actuated smoke

运行标签：

```text
TEB_fixed_0706_noobs_tight_smoke01
```

命令：

```bash
METHOD=teb \
STAGE=actuated \
RUN_LABEL=TEB_fixed_0706_noobs_tight_smoke01 \
PLANNER_CONFIG=/tmp/teb_real_fixed_path_tight_no_obstacle.yaml \
COSTMAP_CONFIG=/tmp/local_costmap_real_no_obstacles.yaml \
MAX_V=0.20 \
MAX_W=1.2 \
RECORD_ALL_EXISTING_TOPICS=true \
RECORD_RGB=true \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

现场判断：

```text
能贴路径，但不顺滑，左右摆头明显。
```

当时判断问题参数：

```text
global_plan_viapoint_sep=0.15 太密；
weight_viapoint=20.0 太大；
global_plan_prune_distance=0.15 太近；
dt_ref=0.20 偏激进；
weight_acc_lim_theta=5.0 对角速度变化惩罚太弱。
```

## 5. no-obstacle balanced 参数

### 5.1 balanced TEB 配置

临时文件：

```text
/tmp/teb_real_fixed_path_balanced_no_obstacle.yaml
```

相比 tight 的改动：

```text
global_plan_viapoint_sep: 0.15 -> 0.35
weight_viapoint: 20.0 -> 6.0
global_plan_prune_distance: 0.15 -> 0.30
dt_ref: 0.20 -> 0.25
max_global_plan_lookahead_dist: 2.0 -> 3.0
weight_acc_lim_theta: 5.0 -> 30.0
max_vel_theta: 1.2 -> 1.0
```

关键参数：

```yaml
TebLocalPlannerROS:
  map_frame: map
  dt_ref: 0.25
  dt_hysteresis: 0.05
  max_global_plan_lookahead_dist: 3.0
  global_plan_prune_distance: 0.30
  global_plan_viapoint_sep: 0.35

  max_vel_x: 0.20
  max_vel_x_backwards: 0.05
  max_vel_theta: 1.0
  acc_lim_x: 0.6
  acc_lim_theta: 1.2

  include_costmap_obstacles: false
  weight_obstacle: 0.0

  no_inner_iterations: 5
  no_outer_iterations: 4
  penalty_epsilon: 0.05
  weight_acc_lim_x: 10.0
  weight_acc_lim_theta: 30.0
  weight_kinematics_forward_drive: 50.0
  weight_optimaltime: 0.10
  weight_viapoint: 6.0
```

### 5.2 actuated smoke

运行标签：

```text
TEB_fixed_0706_noobs_balanced_smoke01
```

用户实际运行命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

METHOD=teb \
STAGE=actuated \
RUN_LABEL=TEB_fixed_0706_noobs_balanced_smoke01 \
PLANNER_CONFIG=/tmp/teb_real_fixed_path_balanced_no_obstacle.yaml \
COSTMAP_CONFIG=/tmp/local_costmap_real_no_obstacles.yaml \
MAX_V=0.20 \
MAX_W=1.0 \
RECORD_ALL_EXISTING_TOPICS=true \
RECORD_RGB=true \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

现场判断：

```text
这个参数还是太紧了；
速度太慢；
左右摆头稍微好一点，但仍未满足正式 baseline 要求。
```

判断：

```text
balanced 比 tight 平顺一些，但仍然追路径太死，速度也偏保守。
```

## 6. no-obstacle loose_fast 参数

### 6.1 loose_fast TEB 配置

临时文件：

```text
/tmp/teb_real_fixed_path_loose_fast_no_obstacle.yaml
```

设计目的：

```text
不追求 2cm 贴线；
允许 10~25cm 误差；
提高速度，降低角速度上限，减少摆头。
```

相比 balanced 的改动方向：

```text
global_plan_viapoint_sep: 0.35 -> 0.70
weight_viapoint: 6.0 -> 2.0
global_plan_prune_distance: 0.30 -> 0.60
max_global_plan_lookahead_dist: 3.0 -> 4.0
dt_ref: 0.25 -> 0.30
max_vel_x: 0.20 -> 0.25
max_vel_theta: 1.0 -> 0.80
acc_lim_theta: 1.2 -> 1.0
weight_optimaltime: 0.10 -> 0.40
weight_acc_lim_theta: 30.0 -> 50.0
```

关键参数：

```yaml
TebLocalPlannerROS:
  map_frame: map
  dt_ref: 0.30
  dt_hysteresis: 0.08
  max_global_plan_lookahead_dist: 4.0
  global_plan_prune_distance: 0.60
  global_plan_viapoint_sep: 0.70

  max_vel_x: 0.25
  max_vel_x_backwards: 0.03
  max_vel_theta: 0.80
  acc_lim_x: 0.6
  acc_lim_theta: 1.0

  include_costmap_obstacles: false
  weight_obstacle: 0.0

  no_inner_iterations: 4
  no_outer_iterations: 3
  penalty_epsilon: 0.08
  weight_acc_lim_x: 10.0
  weight_acc_lim_theta: 50.0
  weight_kinematics_forward_drive: 40.0
  weight_optimaltime: 0.40
  weight_viapoint: 2.0
```

运行建议命令：

```bash
METHOD=teb \
STAGE=actuated \
RUN_LABEL=TEB_fixed_0706_noobs_loose_fast_smoke01 \
PLANNER_CONFIG=/tmp/teb_real_fixed_path_loose_fast_no_obstacle.yaml \
COSTMAP_CONFIG=/tmp/local_costmap_real_no_obstacles.yaml \
MAX_V=0.25 \
MAX_W=0.8 \
RECORD_ALL_EXISTING_TOPICS=true \
RECORD_RGB=true \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

现场反馈：

```text
还是太慢。
```

因此 loose_fast 仍不是最终参数，明天继续。

## 7. 今天形成的参数规律

### 7.1 影响“贴路径”的主要参数

```text
weight_viapoint 越大，越贴路径；
global_plan_viapoint_sep 越小，via-point 越密，越贴路径；
global_plan_prune_distance 越小，局部目标切换越敏感，也更容易左右修；
max_global_plan_lookahead_dist 越小，越偏局部追点。
```

今天现象：

```text
weight_viapoint=20.0, sep=0.15：贴得过死，左右摆头；
weight_viapoint=6.0, sep=0.35：稍好，但仍太紧太慢；
weight_viapoint=2.0, sep=0.70：方向是放松，但现场仍觉得速度不足。
```

### 7.2 影响“左右摆头”的主要参数

```text
weight_viapoint 过大；
global_plan_viapoint_sep 过小；
global_plan_prune_distance 过小；
max_vel_theta / MAX_W 过大；
weight_acc_lim_theta 太小；
dt_ref 太小。
```

今天较有效的改善方向：

```text
降低 weight_viapoint；
增大 global_plan_viapoint_sep；
增大 global_plan_prune_distance；
降低 max_vel_theta；
提高 weight_acc_lim_theta。
```

### 7.3 影响“速度太慢”的主要参数

```text
max_vel_x / MAX_V；
weight_optimaltime；
max_global_plan_lookahead_dist；
global_plan_viapoint_sep；
weight_viapoint；
max_vel_theta / MAX_W。
```

今天判断：

```text
MAX_V=0.20 对 TEB 来说偏保守；
MAX_V=0.25 仍被现场认为偏慢；
明天可以考虑 0.28~0.30，但要保持安全。
```

## 8. 明天建议起点

明天不要从 tight / balanced 重新开始，建议从 loose_fast 继续放松和提速。

建议下一版参数方向：

```text
MAX_V: 0.25 -> 0.28 或 0.30
max_vel_x: 0.25 -> 0.28 或 0.30
MAX_W: 0.8 -> 0.8 或 0.9
max_vel_theta: 0.80 -> 0.80 或 0.90
weight_viapoint: 2.0 -> 1.0~1.5
global_plan_viapoint_sep: 0.70 -> 0.80~1.00
global_plan_prune_distance: 0.60 -> 0.80
max_global_plan_lookahead_dist: 4.0 -> 4.0~5.0
weight_optimaltime: 0.40 -> 0.60
weight_acc_lim_theta: 保持 50.0 或更高
```

建议明天第一个 smoke 名称：

```text
TEB_fixed_0707_noobs_looser_faster_smoke01
```

建议明天第一个参数目标：

```text
不要追求厘米级贴线；
path p95 允许 0.10~0.30 m；
优先让车速上来、摆头减少、整体轨迹可作为外部 baseline。
```

## 9. 今天不做的事

```text
1. 不做 TEB formal N=3。
2. 不恢复避障。
3. 不把 no-obstacle TEB 作为最终公平 baseline。
4. 不分析 RGB。
5. 不对今天最后一把 bag 做深度分析，明天继续。
```

## 10. 注意事项

当前调试用的 TEB 配置都在 `/tmp`，重启后可能丢失。本文已经记录关键参数，明天若 `/tmp/*.yaml` 不存在，需要按本文重新生成。

如果明天要固化参数，再考虑把最终版配置放入 repo；今天暂时只记录过程，不改正式配置。
