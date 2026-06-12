# 20260611 TEB/DWA 隔离 Scout 仿真对比矩阵 SOP

## 1. 目的

本轮在新的隔离 Scout 仿真环境中补齐 TEB / DWA baseline 入口，让它们可以和 SPMPC 使用同一类 fixed path、同一套速度/加速度口径、同一套 bag/summary 证据做对比。

本 SOP 只规定隔离仿真脚本与对比矩阵，不修改 world、map、URDF、robot model、Gazebo model、spawn、Cartographer 或 TF 语义。

## 2. 环境边界

- 新隔离仿真根目录：`/data/a/scout_sim_replacement`。
- 当前 Scout workspace：`/home/a/scout_ws`，本轮只读复用 baseline planner、planner config、slosh monitor 与文档记录。
- 新增运行脚本和运行产物均放在 `/data/a/scout_sim_replacement`。
- 不使用 broad `killall` / `pkill`；脚本只清理自己记录的子进程 PID。
- current-sim smoke 只能说明链路跑通，不能自动标记为 strict fresh-sim Gate。

## 3. 新增入口

### 3.1 launch wrapper

```text
/data/a/scout_sim_replacement/classic_ws/src/scout_mini_proxy_nav_adapter/launch/proxy_teb_localized.launch
/data/a/scout_sim_replacement/classic_ws/src/scout_mini_proxy_nav_adapter/launch/proxy_dwa_localized.launch
/data/a/scout_sim_replacement/classic_ws/src/scout_mini_proxy_nav_adapter/config/local_costmap_proxy_baseline.yaml
```

两个 wrapper 都复用：

- fixed path publisher：`proxy_fixed_path_publisher.py`
- baseline runner：`baseline_local_planner_runner/launch/nav_core_runner.launch`
- proxy-local costmap：`local_costmap_proxy_baseline.yaml`

### 3.2 attach 脚本

```text
/data/a/scout_sim_replacement/scripts/launch_proxy_baseline_localized_attach.sh
/data/a/scout_sim_replacement/scripts/launch_proxy_teb_localized_attach.sh
/data/a/scout_sim_replacement/scripts/launch_proxy_dwa_localized_attach.sh
```

attach 脚本只连接已经运行的隔离 ROS master，不启动 roscore/Gazebo/Cartographer/RViz。

### 3.3 smoke runner 与 observer

```text
/data/a/scout_sim_replacement/scripts/run_proxy_baseline_mainline_smoke.sh
/data/a/scout_sim_replacement/scripts/baseline_mainline_summary_observer.py
```

observer 只读 `/baseline/<planner>/status`、`/cmd_vel`、`/cmd_vel_drive`、`/odom` 和可选 `/slosh/height`，只写 `summary.json`，不发布控制命令。

## 4. 对比矩阵

| planner | smoke entrypoint | path topic | path frame | plan target frame | base frame | costmap frame | cmd topic | status topic | limits | slosh in control | slosh evaluation |
|---|---|---|---|---|---|---|---|---|---|---|---|
| SPMPC | `run_proxy_spmpc_mainline_smoke.sh` | `/scout/global_path_fixed` | `map` | `map` | `base_link` | `/map` input | `/cmd_vel` | `/spmpc/status` | `v=0.8`, `omega=1.2`, `a=0.6`, `alpha`按 run 记录 | variant 决定 | `/spmpc/slosh_height` 与 `/spmpc/slosh_horizon_summary` |
| TEB | `run_proxy_baseline_mainline_smoke.sh BASELINE=teb` | `/scout/global_path_fixed` | `map` | `odom` | `base_footprint` | `odom` | `/cmd_vel` | `/baseline/teb/status` | `0.8 / 1.2 / 0.6 / 1.2` | 否 | 可选 `/slosh/*`，evaluation-only |
| DWA | `run_proxy_baseline_mainline_smoke.sh BASELINE=dwa` | `/scout/global_path_fixed` | `map` | `odom` | `base_footprint` | `odom` | `/cmd_vel` | `/baseline/dwa/status` | `0.8 / 1.2 / 0.6 / 1.2` | 否 | 可选 `/slosh/*`，evaluation-only |

说明：

- TEB/DWA 不订阅、不使用 `/slosh/*` 做控制；`/slosh/*` 只作为外部评估。
- TEB/DWA 的 path 先由 proxy fixed-path publisher 在 `map` 下生成，再由 baseline runner 转到 `odom` 口径供 local planner/costmap 使用。
- TEB/DWA 默认 `PATH_PUBLISH_COUNT=1` 且 `PATH_KEEP_ALIVE_AFTER_PUBLISH_COUNT=true`：路径/目标只发布一次，随后节点保持存活以维持 latched path/goal，避免 baseline runner 被 2Hz 重复 `setPlan()` 重置。
- `base_footprint` 是 baseline runner 与 costmap 的默认口径；如后续 TF 实测发现隔离仿真只有 `base_link` 可用，必须在 run 记录中说明 frame 口径变化。

## 5. 推荐 smoke 命令

TEB：

```bash
BASELINE=teb MODE=closed_loop RECORD_SEC=70 SLOSH_MONITOR_ENABLE=true \
GOAL_X=5.0 GOAL_Y=0.0 GOAL_YAW=0.0 \
PATH_TEMPLATE=s_curve PATH_START_HEADING=current \
/data/a/scout_sim_replacement/scripts/run_proxy_baseline_mainline_smoke.sh
```

DWA：

```bash
BASELINE=dwa MODE=closed_loop RECORD_SEC=70 SLOSH_MONITOR_ENABLE=true \
GOAL_X=5.0 GOAL_Y=0.0 GOAL_YAW=0.0 \
PATH_TEMPLATE=s_curve PATH_START_HEADING=current \
/data/a/scout_sim_replacement/scripts/run_proxy_baseline_mainline_smoke.sh
```

如果需要后处理完整轨迹指标，显式加：

```bash
RECORD_BAG=true
```

## 6. 输出证据

默认输出目录在：

```text
/data/a/scout_sim_replacement/results/proxy_baseline_mainline_<timestamp>_<mode>/<planner>/
/data/a/scout_sim_replacement/bags/proxy_baseline_mainline_<timestamp>_<mode>/
```

关键文件：

```text
_meta.yaml
summary.json
observer_stdout.json
observer_stderr.log
run_summary.txt
```

`summary.json` 至少记录：

- `status_first`、`status_final`、`statuses`
- `goal_reached`
- `no_go` 与 `no_go_flags`
- `/cmd_vel` 和 `/cmd_vel_drive` 最大线速度/角速度、非零样本数
- `/odom` 起终点与位移
- 可选 `/slosh/height` peak/p95，单位同时给出 m 与 mm

## 7. pass/fail 判据

单次 smoke 可标记为通过的最低条件：

1. `/baseline/<planner>/status` 出现。
2. `/scout/global_path_fixed` 出现。
3. `/cmd_vel` 与 `/cmd_vel_drive` 有非零输出。
4. observer 未标记 `SET_PLAN_FAILED` 或持续 `NO_VALID_CMD`。
5. 若目标可达，最终或过程中出现 `GOAL_REACHED`。
6. 所有日志、bag、summary 都在 `/data/a/scout_sim_replacement` 下。

如果失败，保留所有 bag/log/summary；不能通过改 world/map/URDF/spawn/Cartographer/TF 来掩盖 planner 问题。

## 8. strict fresh-sim 规则

正式对比仍执行旧规则：

```text
1. fresh 启动 Gazebo/RViz/定位环境。
2. 等待 30s。
3. 一次只跑一个 planner/case。
4. 70s timeout。
5. 结束后关闭仿真。
6. 关闭后等待 30s。
7. 证据归档到 /data/a。
```

未按这个流程执行的 current-sim 或开发 smoke，不写成 strict fresh-sim Gate。

## 9. 20260611 开发 smoke 记录

本轮先按新脚本各跑了一次 TEB/DWA closed-loop 开发 smoke，用来验证脚本链路和记录口径；这不是 strict fresh-sim Gate。

共同设置：

```text
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
MODE=closed_loop
RECORD_SEC=70
SLOSH_MONITOR_ENABLE=true
GOAL_X=5.0 GOAL_Y=0.0 GOAL_YAW=0.0
PATH_TEMPLATE=s_curve
PATH_START_HEADING=current
ROS_MASTER_URI=http://localhost:11331
GAZEBO_MASTER_URI=http://localhost:11365
```

结果摘要：

| planner | result dir | goal_reached | no_go_flags | odom displacement m | max cmd v m/s | max cmd omega rad/s | `/slosh/height` peak mm | p95 mm |
|---|---|---:|---|---:|---:|---:|---:|---:|
| TEB | `/data/a/scout_sim_replacement/results/proxy_baseline_mainline_20260611_170959_closed_loop/teb` | false | `GOAL_NOT_REACHED` | 4.754 | 0.754 | 0.779 | 3.169 | 0.794 |
| DWA | `/data/a/scout_sim_replacement/results/proxy_baseline_mainline_20260611_170810_closed_loop/dwa` | false | `GOAL_NOT_REACHED` | 4.751 | 0.545 | 0.549 | 2.307 | 1.114 |

解释：

- 两个 baseline 的脚本链路均已跑通：`/baseline/<planner>/status`、`/scout/global_path_fixed`、`/cmd_vel`、`/cmd_vel_drive` 与 `/slosh/height` 都有记录。
- 两个 baseline 在 70s 内都有明显位移，但 wrapper 未发布 `GOAL_REACHED`，因此按新 observer 口径标记 `GOAL_NOT_REACHED`，不能写成成功对比结果。
- smoke 后检查：`ROS_MASTER_URI=http://localhost:11331` 不可通信；Gazebo 输出 `An instance of Gazebo is not running.`，未使用 broad kill。

## 10. 20260611 one-shot 发布修正与复测

针对 2Hz 重复发布 path/goal 可能反复触发 baseline runner `setPlan()` 与 goal latch reset 的问题，TEB/DWA wrapper 已改为默认 one-shot 发布：

```text
PATH_PUBLISH_COUNT=1
PATH_KEEP_ALIVE_AFTER_PUBLISH_COUNT=true
```

修正点：

- `proxy_fixed_path_publisher.py` 支持 `~keep_alive_after_publish_count`；达到 `publish_count` 后可 `rospy.spin()` 保持 latched path/goal。
- `proxy_teb_localized.launch` 与 `proxy_dwa_localized.launch` 默认传入 `path_publish_count=1`、`path_keep_alive_after_publish_count=true`。
- `launch_proxy_baseline_localized_attach.sh` 与 `run_proxy_baseline_mainline_smoke.sh` 记录并透传上述参数。

静态检查结果：`ONESHOT_STATIC_OK`。

one-shot 复测仍为开发 smoke，不是 strict fresh-sim Gate。

共同设置同第 9 节，复测结果如下：

| planner | result dir | goal_reached | no_go_flags | status_counts | odom displacement m | odom final x/y | max cmd v m/s | max cmd omega rad/s | `/slosh/height` peak mm | p95 mm |
|---|---|---:|---|---|---:|---|---:|---:|---:|---:|
| TEB | `/data/a/scout_sim_replacement/results/proxy_baseline_mainline_20260611_173140_closed_loop/teb` | false | `GOAL_NOT_REACHED` | `TRACKING: 1` | 4.707 | `(1.017, -0.010)` | 0.800 | 0.915 | 3.065 | 0.736 |
| DWA | `/data/a/scout_sim_replacement/results/proxy_baseline_mainline_20260611_173333_closed_loop/dwa` | false | `GOAL_NOT_REACHED` | `TRACKING: 1` | 4.750 | `(0.864, -0.090)` | 0.547 | 0.498 | 2.629 | 0.856 |

结论：

- one-shot 已生效，日志出现 `published 1 time(s); keeping node alive to hold latched path/goal`，DWA 也只出现一次 `Got new plan`。
- 原先“重复发布导致反复 reset/反复 setPlan”的症状消失；`status_counts` 只剩 `TRACKING: 1`。
- 但 TEB/DWA 仍未到点，observer 继续标记 `GOAL_NOT_REACHED`。
- 新的主要线索是 frame/goal 口径：复测的 `/baseline/<planner>/global_plan` 终点在 `odom` 下约为 `x=5.0, y=0.0`，但 70s 结束时 `/odom` 位置仅到 TEB `(1.017, -0.010)`、DWA `(0.864, -0.090)`；下一步应在不修改仿真环境的前提下诊断 `path_frame=map -> plan_target_frame=odom` 的目标变换与 wrapper goal check 口径。

复测后检查：`ROS_MASTER_URI=http://localhost:11331` 不可通信；Gazebo 输出 `An instance of Gazebo is not running.`，未使用 broad kill。

## 11. 20260611 frame/goal 口径诊断

继续诊断 one-shot 后仍 `GOAL_NOT_REACHED` 的原因时，额外跑了带 `/tf` bag 的开发诊断 smoke；仍然不是 strict fresh-sim Gate。

诊断结果修正了第 10 节的初步判断：`/baseline/<planner>/global_plan` 的 `odom` 终点约 `x=5.0` 本身不是主要问题。关键区别是：

- baseline runner/local planner 使用 TF tree 中的 `odom -> base_footprint`。
- observer 的 `summary.json` 记录的是 `/odom` topic pose；在当前 proxy 仿真中它带有 spawn/world 偏移，不能直接拿来和 `/baseline/<planner>/global_plan` 的 TF-`odom` 坐标比较。
- bag 中 TF 证据显示 `map -> odom` 基本接近 identity，`/baseline/<planner>/global_plan` 的 `odom` 口径与 TF 口径一致。

带 bag 的 `GOAL_YAW=0.0` 诊断：

| planner | result dir | bag | `/scout/goal` yaw | global plan 末端 yaw | final TF `odom->base_footprint` | 结论 |
|---|---|---|---:|---:|---|---|
| TEB | `/data/a/scout_sim_replacement/results/proxy_baseline_mainline_20260611_175248_closed_loop/teb` | `/data/a/scout_sim_replacement/bags/proxy_baseline_mainline_20260611_175248_closed_loop/teb_closed_loop.bag` | 0.000 | 0.624 | `(x=5.006, y=0.002, yaw=0.629)` | 位置已到，但 yaw 与 `/scout/goal` 相差约 0.63 rad |
| DWA | `/data/a/scout_sim_replacement/results/proxy_baseline_mainline_20260611_180117_closed_loop/dwa` | `/data/a/scout_sim_replacement/bags/proxy_baseline_mainline_20260611_180117_closed_loop/dwa_closed_loop.bag` | 0.000 | 0.625 | `(x=4.852, y=-0.105, yaw=0.652)` | 位置基本到，但 yaw 与 `/scout/goal` 相差约 0.65 rad |

根因判断：

- `proxy_fixed_path_publisher.py` 对 fixed path 的最后一个 pose 使用路径切线 yaw，S-curve 末端约 `0.624~0.625 rad`。
- 但同一个节点额外发布的 `/scout/goal` 使用命令行参数 `GOAL_YAW=0.0`。
- baseline runner 的 `goalCallback()` 会用 `/scout/goal` 覆盖 path 末点派生的 goal；wrapper `goalCloseEnough()` 同时检查距离和 yaw，默认 `yaw_goal_tolerance=0.2 rad`。
- 因此 TEB/DWA 实际已经跟到了路径末端方向附近，但 goal check 要求车头回到 `0.0 rad`，导致一直不发布 `GOAL_REACHED`。

验证性复测：仅把诊断输入改为 `GOAL_YAW=0.624`，不修改仿真环境，TEB/DWA 均到点：

| planner | result dir | GOAL_YAW | goal_reached | duration sec | status_counts |
|---|---|---:|---:|---:|---|
| TEB | `/data/a/scout_sim_replacement/results/proxy_baseline_mainline_20260611_180352_closed_loop/teb` | 0.624 | true | 6.590 | `TRACKING: 1, GOAL_REACHED: 1` |
| DWA | `/data/a/scout_sim_replacement/results/proxy_baseline_mainline_20260611_180431_closed_loop/dwa` | 0.624 | true | 17.342 | `TRACKING: 1, GOAL_REACHED: 1` |

已落实的修正方向：不要修改 world/map/URDF/spawn/Cartographer/TF；只在 fixed-path 输入口径上保证 TEB/DWA 的 `/scout/goal` yaw 与 path 末端 yaw 一致。

实现方式：

- `proxy_fixed_path_publisher.py` 新增 `goal_yaw_mode`，默认保持旧行为 `explicit`，避免影响 SPMPC proxy 链路。
- `proxy_teb_localized.launch` 与 `proxy_dwa_localized.launch` 默认设置 `goal_yaw_mode=path_end`。
- baseline attach/smoke 脚本记录并透传 `GOAL_YAW_MODE`，默认 `path_end`。

修正后用默认命令复测，即仍传入 `GOAL_YAW=0.0`，由 `goal_yaw_mode=path_end` 自动发布 path 末端 yaw：

| planner | result dir | GOAL_YAW | GOAL_YAW_MODE | goal_reached | duration sec | max cmd v m/s | `/slosh/height` peak mm | p95 mm | status_counts |
|---|---|---:|---|---:|---:|---:|---:|---:|---|
| TEB | `/data/a/scout_sim_replacement/results/proxy_baseline_mainline_20260611_183834_closed_loop/teb` | 0.0 | `path_end` | true | 6.640 | 0.781 | 3.534 | 2.601 | `TRACKING: 1, GOAL_REACHED: 1` |
| DWA | `/data/a/scout_sim_replacement/results/proxy_baseline_mainline_20260611_183913_closed_loop/dwa` | 0.0 | `path_end` | true | 18.191 | 0.549 | 2.815 | 1.357 | `TRACKING: 1, GOAL_REACHED: 1` |

复测日志确认：

```text
proxy_fixed_path_publisher: ... goal=(5.000, 0.000, 0.624) goal_yaw_mode=path_end ...
```

诊断后检查：`ROS_MASTER_URI=http://localhost:11331` 不可通信；Gazebo 输出 `An instance of Gazebo is not running.`，未使用 broad kill。

## 12. 20260611 公平 peak 对比：ALL 与去起终点段

按“保证实验公平”的要求，前面旧数据不再混入本表：旧 SPMPC verify 曾用 `ALPHA_MAX=8.0`，而 TEB/DWA 是 `alpha=1.2`；且部分 TEB/DWA 成功 run 没有 bag，不能重新按统一窗口裁剪。因此补跑了一组 bag-backed 单次开发对比，统一使用外部 `/slosh/height` 作为 slosh 评估源。

本节仍是“公平单次开发对比”，不是 strict fresh-sim Gate 的最终统计结论；正式结论还要按第 8 节做 fresh-sim、多次重复、每 case 单独归档。

### 12.1 本轮公平 run 证据

共同设置：

```text
RUN_STAMP=fair_peak_20260611_190337
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
MODE=closed_loop
RECORD_SEC=70
RECORD_BAG=true
SLOSH_MONITOR_ENABLE=true
GOAL_X=5.0 GOAL_Y=0.0 GOAL_YAW=0.0
PATH_TEMPLATE=s_curve
PATH_START_HEADING=current
/slosh/height = 外部 evaluation-only 监控；TEB/DWA 不使用 slosh 做控制
```

| planner | result dir | bag | goal_reached | first `GOAL_REACHED` from bag status | observed max v m/s | observed max omega rad/s |
|---|---|---|---:|---:|---:|---:|
| SPMPC `B_ours` | `/data/a/scout_sim_replacement/results/fair_peak_20260611_190337/spmpc/B_ours` | `/data/a/scout_sim_replacement/bags/fair_peak_20260611_190337/spmpc/B_ours_closed_loop.bag` | true | 16.505s | 0.494 | 0.604 |
| TEB | `/data/a/scout_sim_replacement/results/fair_peak_20260611_190337/teb/teb` | `/data/a/scout_sim_replacement/bags/fair_peak_20260611_190337/teb/teb_closed_loop.bag` | true | 6.366s | 0.754 | 1.020 |
| DWA | `/data/a/scout_sim_replacement/results/fair_peak_20260611_190337/dwa/dwa` | `/data/a/scout_sim_replacement/bags/fair_peak_20260611_190337/dwa/dwa_closed_loop.bag` | true | 18.687s | 0.550 | 0.524 |

说明：SPMPC `summary.json` 的 `duration_sec=70.0` 是 observer 参数值，不用于和 TEB/DWA 的 early-stop `duration_sec` 直接比较；上表的 `first GOAL_REACHED` 是从 bag 中 status topic 首次出现 `GOAL_REACHED` 的时间差计算。

### 12.2 参数公平对齐表

| planner | variant/plugin | path / goal | planner frame 口径 | target `v_max` | target `omega_max` | target `a_max` | target `alpha/acc_lim_theta` | goal yaw 口径 | slosh 口径 |
|---|---|---|---|---:|---:|---:|---:|---|---|
| SPMPC | `B_ours`, `continuous_mpcc_acados` | `s_curve`, current heading, `(5.0, 0.0, 0.0)` | `map`, `base_link`, costmap `/map` | 0.8 | 1.2 | 0.6 | 1.2 (`ALPHA_MAX=1.2`, `MAX_ANGULAR_ACCEL=1.2`) | 保持 SPMPC fixed-path 默认 explicit `GOAL_YAW=0.0` | 控制内部可用自身 variant；本表评估统一取外部 `/slosh/height` |
| TEB | `teb_local_planner/TebLocalPlannerROS` | `s_curve`, current heading, `(5.0, 0.0, 0.0)` | path `map` -> planner `odom`, `base_footprint` | 0.8 | 1.2 | 0.6 | 1.2 (`acc_lim_theta`) | `GOAL_YAW_MODE=path_end`，使 `/scout/goal` yaw 等于 path 末端 yaw | `/slosh/*` 只读 evaluation-only，`external_baseline_uses_slosh=false` |
| DWA | `dwa_local_planner/DWAPlannerROS` | `s_curve`, current heading, `(5.0, 0.0, 0.0)` | path `map` -> planner `odom`, `base_footprint` | 0.8 | 1.2 | 0.6 | 1.2 (`acc_lim_theta`) | `GOAL_YAW_MODE=path_end`，使 `/scout/goal` yaw 等于 path 末端 yaw | `/slosh/*` 只读 evaluation-only，`external_baseline_uses_slosh=false` |

公平性说明：

- 三者统一使用 `v_max=0.8 m/s`、`omega_max=1.2 rad/s`、`a_max=0.6 m/s²`、角加速度上限 `1.2 rad/s²` 的 common-limit 口径。
- 本表的 slosh peak/p95 统一来自 bag 中的外部 `/slosh/height`，避免把 SPMPC 内部 `/spmpc/slosh_height` 与 TEB/DWA 外部 monitor 直接混比。
- TEB/DWA 的 `goal_yaw_mode=path_end` 是 fixed-path goal 口径修正，不是仿真环境修改；它避免 `/scout/goal` yaw 覆盖 path 末端 yaw 导致 wrapper goal check 假失败。
- SPMPC 这次显式设置 `ALPHA_MAX=1.2`；旧 `ALPHA_MAX=8.0` SPMPC 结果只能作为历史诊断，不能放入本公平表。

### 12.3 `/slosh/height` peak 对比：ALL 全段

`ALL` 表示 bag 中全部 `/slosh/height` 样本，包含启动段、跟踪段、接近终点段。

| planner | goal_reached | `/slosh/height` peak mm | p95 mm | slosh samples | observed max v m/s |
|---|---:|---:|---:|---:|---:|
| SPMPC `B_ours` | true | 1.266 | 0.431 | 859 | 0.494 |
| TEB | true | 2.350 | 1.639 | 351 | 0.754 |
| DWA | true | 2.938 | 1.521 | 968 | 0.550 |

按当前单次 ALL peak 判断：SPMPC `B_ours` 最低；DWA peak 最高；TEB 的 peak 介于两者之间但 p95 明显高于 SPMPC。

### 12.4 `/slosh/height` peak 对比：去起终点段

“去起终点段”采用统一后处理窗口：把 TF pose 投影到 `/scout/global_path_fixed`，只统计路径 progress 10%--90% 内的 `/slosh/height` 样本。这样不是删除数据，而是把起步和临近终点的 transient 单独隔离，便于观察中段跟踪激励。

| planner | goal_reached | progress 10%--90% peak mm | p95 mm | core samples | observed max v m/s |
|---|---:|---:|---:|---:|---:|
| SPMPC `B_ours` | true | 0.939 | 0.376 | 705 | 0.494 |
| TEB | true | 2.346 | 1.580 | 267 | 0.754 |
| DWA | true | 2.361 | 1.563 | 623 | 0.550 |

按当前单次去起终点段 peak 判断：SPMPC `B_ours` 仍最低；TEB 与 DWA 的中段 peak 接近，均约为 SPMPC 的 2.5 倍。由于 TEB 的速度更接近 0.8 m/s 上限，后续正式对比应同时报告速度分布、用时、路径误差和 slosh，不只看 peak 一个指标。

### 12.5 后处理产物

统一后处理结果保存为：

```text
/data/a/scout_sim_replacement/results/fair_peak_20260611_190337/fair_peak_metrics_progress10_90.json
```

后续若扩展到多次重复，建议沿用同一后处理口径输出：

- `ALL`：完整 bag 样本。
- `CORE`：progress 10%--90%。
- 同时保留 `0%--10%` 与 `90%--100%` 的起终点窗口，避免把起终点问题“删掉”。

## 13. 20260611 strict fresh-sim N=3：SPMPC / MPC / TEB / DWA

本节用于记录更正式的 strict fresh-sim N=3 对比。它替代第 12 节的“单次开发公平表”作为实物对比前的仿真 gate 证据；但不能为了进入实物试验而改 world、map 文件内容、URDF、robot model、Gazebo model、spawn、Cartographer 参数或 TF 语义，也不能删除失败 case。

共同规则：

```text
root=/data/a/scout_sim_replacement
N=3
MODE=closed_loop
RECORD_SEC=70
STRICT_PRE_CONTROL_SETTLE_SEC=30
STRICT_POST_SHUTDOWN_SEC=30
GOAL_X=5.0 GOAL_Y=0.0 GOAL_YAW=0.0
PATH_TEMPLATE=s_curve
PATH_START_HEADING=current
SLOSH_MONITOR_ENABLE=true
slosh_source=/slosh/height external evaluation-only
common_limits=v0.8 / omega1.2 / a0.6 / alpha_or_acc_lim_theta1.2
established_map=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
```

实现注意：

- strict runner 在每个 case 前后检查 ROS master 与 Gazebo master；已有进程会使该 case 标为 invalid，不贴 strict 标签。
- `mpc_local_planner` 通过 `/home/a/scout_ws/install_isolated_mpc/setup.bash` 提供 plugin library；这是 runtime overlay，不把库复制进 `/home/a/scout_ws/devel`。
- Cartographer isolated workspace 只在 runtime source，未改地图、定位参数、TF 语义、world/model/spawn。
- SPMPC 使用 `continuous_mpcc_acados`，并显式设置 `ALPHA_MAX=1.2`、`MAX_ANGULAR_ACCEL=1.2`。
- TEB/DWA/MPC baseline 仍使用 `GOAL_YAW_MODE=path_end`，这是 fixed-path goal 口径修正，不是环境修改。
- 20260611 晚间复核后，SPMPC strict runner 已显式使用并记录上述 `established_map`，避免 strict 两阶段环境启动时静默落回 localization launch 默认 map。

### 13.1 初始四 planner strict batch：保留为诊断证据

初始批次：

```text
result_root=/data/a/scout_sim_replacement/results/strict_fresh_fair_n3_20260611_202103
bag_root=/data/a/scout_sim_replacement/bags/strict_fresh_fair_n3_20260611_202103
manifest=/data/a/scout_sim_replacement/results/strict_fresh_fair_n3_20260611_202103/strict_fresh_manifest.csv
aggregate=/data/a/scout_sim_replacement/results/strict_fresh_fair_n3_20260611_202103/strict_fair_metrics_aggregate.json
plot=/data/a/scout_sim_replacement/results/strict_fresh_fair_n3_20260611_202103/strict_fair_metric_comparison.png
```

该批次完成 12 个 case；`failed_cases=3`、`invalid_cases=0`。MPC/TEB/DWA 均 3/3 到点，但 SPMPC `B_ours` 3/3 触发 `TRACKING_UNSAFE_PROJECTION`：

| planner | valid strict cases | passes | goal reached | failed flags | 初始批次结论 |
|---|---:|---:|---:|---|---|
| SPMPC `B_ours` | 3/3 | 0/3 | 0/3 | 三次均 `TRACKING_UNSAFE_PROJECTION` | 保留为 map 口径不一致诊断证据 |
| `mpc_local_planner` | 3/3 | 3/3 | 3/3 | none | PASS |
| TEB | 3/3 | 3/3 | 3/3 | none | PASS |
| DWA | 3/3 | 3/3 | 3/3 | none | PASS |

初始 aggregate 指标如下；SPMPC 行不能解读为“通过且 slosh 更好”，因为三次均未 `GOAL_REACHED`。

| planner | ALL peak mean±std mm | ALL p95 mean mm | CORE 10%--90% peak mean±std mm | CORE p95 mean mm | first GOAL mean s | max v mean m/s | max omega mean rad/s |
|---|---:|---:|---:|---:|---:|---:|---:|
| SPMPC `B_ours` | 1.656 ± 0.117 | 0.280 | 0.423 ± 0.052 | 0.377 | N/A | 0.500 | 0.615 |
| `mpc_local_planner` | 1.553 ± 0.106 | 1.260 | 0.859 ± 0.045 | 0.691 | 5.916 | 0.800 | 0.566 |
| TEB | 3.350 ± 0.686 | 1.892 | 3.350 ± 0.686 | 2.108 | 6.272 | 0.771 | 0.804 |
| DWA | 3.232 ± 0.412 | 1.776 | 3.024 ± 0.630 | 1.833 | 16.773 | 0.548 | 0.506 |

### 13.2 为什么“突然不行”：strict 两阶段 SPMPC 用错了 map 口径

上一次成功的 `fair_peak_20260611_190337` 中，SPMPC `B_ours` 明确使用：

```text
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
goal_reached=true
status: B_ours_ACADOS_OK -> GOAL_REACHED
progress_s max/final≈0.9693
map->odom max jump≈0.0193m
```

而初始 strict SPMPC 两阶段环境启动日志显示：

```text
MAP_FILE=<localization launch default>
```

`proxy_cartographer_localization.launch` 的默认 map 是：

```text
/home/a/scout_ws/src/scout_apps/scout_maps/maps/map_sim_empty.pbstream
```

它和新的隔离仿真已建立 map 不是同一个文件：

```text
map_sim_empty.pbstream sha256=9a409f7c6a7556fdb930bfd4dd163cdbd6eb460349ecf23e6f7cb9355cd62b98
proxy_world_manual_saved_20260611_154348.pbstream sha256=fd065fcc95b1ed2c25dd355b8c312b0d2f84d96e379d137b67997bd225f8ead0
```

bag/TF 复核显示，初始 strict 失败 run 的 `map->odom` 有约 `4.47~4.50m` 级跳变；这会把 SPMPC map-frame projection distance 推过 `tracking_safety/projection/max_distance_m=0.5`，从而触发 `TRACKING_UNSAFE_PROJECTION`。因此“突然不行”不是 `B_ours` 控制逻辑突然退化，而是 strict runtime 的 localization map 口径和上一轮成功 run 不一致。

### 13.3 `B_slosh` 同口径复测：也失败，说明不是 `B_ours` 特有问题

按用户要求，在初始 strict 口径下补跑 `B_slosh` N=3：

```text
result_root=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_B_slosh_n3_20260611_205842
bag_root=/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_B_slosh_n3_20260611_205842
manifest=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_B_slosh_n3_20260611_205842/strict_fresh_B_slosh_manifest.csv
aggregate=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_B_slosh_n3_20260611_205842/strict_B_slosh_metrics_aggregate.json
```

结果：`valid_cases=3/3`、`passes=0/3`、`goal_reached=0/3`、三次均 `TRACKING_UNSAFE_PROJECTION`。这说明失败不是 `B_ours` 特有，而是 SPMPC strict 环境口径问题。

| variant | valid strict cases | passes | goal reached | ALL peak mean mm | CORE 10%--90% peak mean mm | max v mean m/s | max omega mean rad/s | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| SPMPC `B_slosh`（default-map strict） | 3/3 | 0/3 | 0/3 | 1.749 | 0.437 | 0.497 | 0.705 | 三次均 `TRACKING_UNSAFE_PROJECTION`；作为 map 口径诊断证据保留 |

### 13.4 显式 established map 复测：SPMPC `B_ours` 与 `B_slosh` 均 PASS

修正 strict SPMPC 运行口径：不改地图文件内容、不改 world/URDF/spawn/Cartographer 参数/TF，只把 strict 两阶段环境启动显式指向与上一轮成功 run 相同的 established map。

```text
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
result_root=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_explicit_map_n3_20260611_211023
bag_root=/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_explicit_map_n3_20260611_211023
manifest=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_explicit_map_n3_20260611_211023/strict_fresh_spmpc_explicit_map_manifest.csv
aggregate=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_explicit_map_n3_20260611_211023/strict_spmpc_explicit_map_metrics_aggregate.json
```

批次结果：`completed_cases=6`、`failed_cases=0`、`invalid_cases=0`、`exit_status=0`；manifest 中 6 个 case 的 pre/post ROS master 与 Gazebo master 均为 false/false，strict freshness 条件有效。

| variant | valid strict cases | passes | goal reached | first GOAL mean s | ALL peak mean±std mm | ALL p95 mean mm | CORE 10%--90% peak mean±std mm | CORE p95 mean mm | max v mean m/s | max omega mean rad/s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| SPMPC `B_ours`（explicit map） | 3/3 | 3/3 | 3/3 | 16.052 | 1.098 ± 0.228 | 0.501 | 1.075 ± 0.237 | 0.379 | 0.497 | 0.595 |
| SPMPC `B_slosh`（explicit map） | 3/3 | 3/3 | 3/3 | 15.922 | 1.359 ± 0.195 | 0.520 | 1.260 ± 0.216 | 0.425 | 0.498 | 0.701 |

### 13.5 当前 corrected strict 证据口径

截至本记录，仿真 gate 的正确读法是：

1. 初始 `strict_fresh_fair_n3_20260611_202103` 的 SPMPC `B_ours` 失败必须保留，不能删除；但它暴露的是 strict SPMPC 两阶段环境启动未显式使用 established map 的口径问题。
2. `B_slosh` 在同一错误口径下也 0/3，进一步说明不是 `B_ours` 特有退化。
3. 使用 established map 显式复测后，SPMPC `B_ours` 与 `B_slosh` 均 strict N=3 通过。
4. MPC/TEB/DWA 的 strict N=3 PASS 仍采用初始批次证据；若后续要做论文级最终图表，建议在脚本修正后重新跑完整四 planner 同批次 N=3，以避免跨批次组合口径争议。

因此：仿真层面已经解释并复核了“SPMPC 为什么突然不行”；它不是 planner 回归，而是 map-file runtime 口径不一致。进入实物时仍不能直接做正式四 planner 对比，必须先按实物 SOP 做 P0 straight SPMPC B0、P2 SPMPC B0/B_slosh/B_ours smoke；这些都安全后，才允许逐个 planner 做 shadow-first 的实物对比。

## 14. SPMPC `B0` / `B_slosh` / `B_ours` explicit-map strict fresh-sim N=3 对比

本节按同一口径补齐 SPMPC 三个 ablation variant 的 explicit-map strict fresh-sim N=3。该批次只改变 rosbag 记录 topic，使 `/spmpc/slosh_height` 与 `/spmpc/slosh_horizon_summary` 能进入离线指标；不修改 world、map、URDF、robot model、Gazebo model、spawn、Cartographer、localization 或 TF 语义。

共同设置：

```text
root=/data/a/scout_sim_replacement
N=3
MODE=closed_loop
RECORD_SEC=70
STRICT_PRE_CONTROL_SETTLE_SEC=30
STRICT_POST_SHUTDOWN_SEC=30
GOAL_X=5.0 GOAL_Y=0.0 GOAL_YAW=0.0
PATH_TEMPLATE=s_curve
PATH_START_HEADING=current
SOLVER_BACKEND=continuous_mpcc_acados
ALPHA_MAX=1.2
MAX_ANGULAR_ACCEL=1.2
SLOSH_MONITOR_ENABLE=true
established_map=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
```

输出目录：

```text
result_root=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305
bag_root=/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305
manifest=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/strict_fresh_spmpc_variants_explicit_map_manifest.csv
batch_meta=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/batch_meta.yaml
per_case_metrics=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/spmpc_variant_strict_metrics_per_case.json
aggregate_metrics=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/spmpc_variant_strict_metrics_aggregate.json
per_case_csv=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/spmpc_variant_strict_metrics_per_case.csv
aggregate_csv=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/spmpc_variant_strict_metrics_aggregate.csv
analysis_script=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/analyze_spmpc_variant_strict_metrics.py
```

strict 有效性：9 个 case 均为 `pre_ros=false`、`pre_gazebo=false`、`post_ros=false`、`post_gazebo=false`；`completed_cases=9`、`failed_cases=0`、`invalid_cases=0`、`goal_reached_cases=9`、`pass_strict_goal_cases=9`。每个 case 都按 fresh 启动、30s pre-control settle、单 variant、70s timeout/early goal、关闭仿真、30s post-shutdown wait 归档。

### 14.1 汇总指标

下表为每个 variant 的 N=3 per-case 指标均值；`/slosh/height` 是外部 evaluation-only slosh monitor，单位由 bag 中 m 转为 mm；`/spmpc/slosh_height` 是 SPMPC 内部模型 proxy，发布单位 mm；`10%--90%` 窗口按 TF pose 投影到 `/scout/global_path_fixed` 的路径 progress 后裁剪。

| variant | valid/pass/goal | first `GOAL_REACHED` mean s | progress final mean | `/slosh/height` ALL peak/p95/rms mm | `/slosh/height` 10%--90% peak/p95/rms mm | `/spmpc/slosh_height` ALL peak/p95/rms mm | `/spmpc/slosh_height` 10%--90% peak/p95/rms mm |
|---|---:|---:|---:|---:|---:|---:|---:|
| `B0` | 3/3/3 | 15.204 | 0.968 | 1.405 / 0.622 / 0.339 | 1.405 / 0.542 / 0.334 | 2.003 / 0.754 / 0.399 | 1.986 / 0.631 / 0.382 |
| `B_slosh` | 3/3/3 | 15.617 | 0.968 | 1.376 / 0.522 / 0.303 | 1.338 / 0.434 / 0.290 | 1.973 / 0.659 / 0.358 | 1.895 / 0.478 / 0.326 |
| `B_ours` | 3/3/3 | 16.300 | 0.970 | 1.132 / 0.460 / 0.261 | 1.113 / 0.369 / 0.256 | 1.629 / 0.494 / 0.303 | 1.629 / 0.428 / 0.291 |

`/spmpc/slosh_horizon_summary` 与速度/平滑性指标：

| variant | h_peak_pred mean/p95/max mm | h_p95_pred mean/p95/max mm | `/cmd_vel` v max/mean/p95 m/s | `/cmd_vel` omega max/mean/p95 rad/s | `|dv/dt|` p95/max m/s² | `|domega/dt|` p95/max rad/s² |
|---|---:|---:|---:|---:|---:|---:|
| `B0` | 0.000 / 0.000 / 0.000 | 0.000 / 0.000 / 0.000 | 0.531 / 0.286 / 0.498 | 0.684 / 0.174 / 0.629 | 0.451 / 0.620 | 0.620 / 5.103 |
| `B_slosh` | 0.463 / 1.018 / 1.973 | 0.335 / 0.667 / 1.150 | 0.505 / 0.278 / 0.484 | 0.683 / 0.165 / 0.607 | 0.276 / 0.613 | 0.594 / 4.251 |
| `B_ours` | 0.478 / 1.018 / 1.934 | 0.322 / 0.576 / 1.261 | 0.497 / 0.272 / 0.489 | 0.594 / 0.156 / 0.542 | 0.223 / 0.622 | 0.473 / 3.881 |

说明：`B0` 的 `/spmpc/slosh_horizon_summary` topic 存在但 h 字段为 0，符合 slosh cost/prediction 未启用时的诊断口径；不能把这一行解读为“B0 真实预测晃动为零”，只能说明内部 horizon slosh 预测摘要在 `B0` 下未输出有效非零值。跨 variant 的主要 slosh 对比仍以外部 `/slosh/height` 和内部 `/spmpc/slosh_height` 两条已记录曲线为准。

### 14.2 每个 case 的证据路径

每个 case 的 manifest 记录行均在：

```text
/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/strict_fresh_spmpc_variants_explicit_map_manifest.csv
```

| case | goal_reached | first `GOAL_REACHED` s | status_final | progress final/max | bag | summary |
|---|---:|---:|---|---:|---|---|
| `B0_run_1` | true | 15.510 | `GOAL_REACHED` | 0.969 / 0.969 | `/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B0/run_1/B0_closed_loop.bag` | `/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B0/run_1/B0/summary.json` |
| `B0_run_2` | true | 15.081 | `GOAL_REACHED` | 0.968 / 0.968 | `/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B0/run_2/B0_closed_loop.bag` | `/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B0/run_2/B0/summary.json` |
| `B0_run_3` | true | 15.020 | `GOAL_REACHED` | 0.967 / 0.967 | `/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B0/run_3/B0_closed_loop.bag` | `/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B0/run_3/B0/summary.json` |
| `B_slosh_run_1` | true | 16.080 | `GOAL_REACHED` | 0.970 / 0.970 | `/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_slosh/run_1/B_slosh_closed_loop.bag` | `/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_slosh/run_1/B_slosh/summary.json` |
| `B_slosh_run_2` | true | 15.383 | `GOAL_REACHED` | 0.967 / 0.967 | `/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_slosh/run_2/B_slosh_closed_loop.bag` | `/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_slosh/run_2/B_slosh/summary.json` |
| `B_slosh_run_3` | true | 15.389 | `GOAL_REACHED` | 0.967 / 0.967 | `/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_slosh/run_3/B_slosh_closed_loop.bag` | `/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_slosh/run_3/B_slosh/summary.json` |
| `B_ours_run_1` | true | 16.833 | `GOAL_REACHED` | 0.971 / 0.971 | `/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_ours/run_1/B_ours_closed_loop.bag` | `/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_ours/run_1/B_ours/summary.json` |
| `B_ours_run_2` | true | 16.013 | `GOAL_REACHED` | 0.969 / 0.969 | `/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_ours/run_2/B_ours_closed_loop.bag` | `/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_ours/run_2/B_ours/summary.json` |
| `B_ours_run_3` | true | 16.054 | `GOAL_REACHED` | 0.969 / 0.969 | `/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_ours/run_3/B_ours_closed_loop.bag` | `/data/a/scout_sim_replacement/results/strict_fresh_spmpc_variants_explicit_map_n3_20260612_173305/B_ours/run_3/B_ours/summary.json` |

### 14.3 结论与和实物的一致/不一致

1. 仿真中 `B_slosh` 是否真的优于 `B0`：本批 explicit-map strict N=3 中，`B_slosh` 在 `/slosh/height` 的 p95/rms 与 10%--90% 窗口上低于 `B0`，例如 ALL p95 `0.522 < 0.622 mm`、CORE p95 `0.434 < 0.542 mm`；但 ALL peak 均值只从 `1.405` 降到 `1.376 mm`，差值约 `0.029 mm`，且 per-case peak 有重叠。因此只能谨慎写成“在该仿真 proxy 口径下有小幅改善趋势”，不宜夸大为强结论或普遍结论。
2. 仿真中 `B_ours` 是否优于 `B_slosh`：本批结果支持。`B_ours` 在外部 `/slosh/height` 上低于 `B_slosh`（ALL peak/p95/rms：`1.132/0.460/0.261` vs `1.376/0.522/0.303 mm`；CORE p95：`0.369` vs `0.434 mm`），内部 `/spmpc/slosh_height` 也低于 `B_slosh`（ALL p95：`0.494` vs `0.659 mm`）。同时 `B_ours` 的 `omega` p95、`|dv/dt|` p95 和 `|domega/dt|` p95 均更低，说明 smooth priority 在这个仿真口径下不仅降低 slosh proxy，也降低速度/角速度调制强度。
3. 仿真和实物是否一致：只部分一致。二者一致点是 `B_ours < B_slosh`，即 smooth priority 能显著压低 `B_slosh` 的晃动；不一致点是 `B_slosh` 相对 `B0` 的排序。实物 RGB 与 `/spmpc/slosh_height` 显示 `B0` 最低、`B_slosh` 最高、`B_ours` 明显低于 `B_slosh`；而本批仿真 proxy 显示 `B_slosh` 相对 `B0` 略低或接近，并不复现实物中 `B_slosh` 最高的现象。
4. 不一致的可能来源：优先怀疑度量与激励口径差异，而不是直接推翻实物结论。仿真的 `/slosh/height` 与 `/spmpc/slosh_height` 都是模型 proxy，不能等价于实物 RGB 液面；当前 `s_curve`、目标点、速度水平和终端段激励也未必等同 P2 实物路径；本批仿真里 `B_slosh` 的 `|dv/dt|` p95 低于 `B0`，而实物 cost 分析中 `B_slosh` 的 p95 `|dv/dt|` 最高，说明 slosh-cost-only 在实物中诱发的速度调制没有被当前仿真完全复现。差异还可能来自液体模型保真度、轮地/容器扰动、RGB 与 proxy 的标定误差，以及 slosh cost 与 smooth cost 的权重耦合。

论文/实验章节建议表述：本批仿真可作为“`B_ours` 相对 `B_slosh` 更稳、更低晃”的补强证据；对于 `B_slosh` 相对 `B0`，只能报告仿真 proxy 下的小幅改善趋势，不能写成 slosh-cost-only ablation 必然优于 baseline。结合实物结果，应强调单独 slosh cost 的局限，以及 smooth priority 对抑制速度调制和降低液面晃动的必要性。

### 14.4 后续排查：高激励、cost sweep 与实物 odom offline replay

为解释“仿真差异小且未复现实物 `B_slosh` 最高”的问题，后续按“高激励 fixed-path → `w_slosh` cost sweep → 实物 odom offline replay”的顺序做了补充排查。完整记录与图见 Obsidian：

```text
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/规控一体的实验记录/仿真实验/20260612_SPMPC_B0_Bslosh_Bours_explicit_map_strict_N3/20260612_SPMPC_B0_Bslosh_Bours_explicit-map_strict-fresh_N3.md
```

补充结果摘要：

1. 高激励 `S1_long_x8` N=3（`GOAL_X=8.0`，长 S 曲线）仍未复现实物 `B_slosh` 最高排序。`/slosh/height` peak/p95/rms 均值为：`B0 1.303/0.507/0.271 mm`，`B_slosh 1.257/0.354/0.214 mm`，`B_ours 1.205/0.313/0.205 mm`。
2. `S1_long_x8` 下 `w_slosh=10/20` N=1 sweep 显示 override 有效，`pct_slosh_total` 随权重增大而上升；但它没有诱发仿真里的 `B_slosh` 高液面/高 `|dv/dt|`，反而使 `B_slosh_w20` 的外部 `/slosh/height` p95/rms 低于默认 `B_slosh`。因此继续盲目调大 slosh cost 不是优先方向。
3. 20260611 实物 P2 bag 的 offline odom→standalone-slosh replay 复现了实物主排序：offline slosh p95 为 `B0 0.554 < B_ours 0.763 < B_slosh 1.342 mm`；内部 `/spmpc/slosh_height` p95 为 `B0 1.093 < B_ours 1.487 < B_slosh 2.675 mm`。同时实物 `/cmd_vel |dv/dt|` p95 为 `B_slosh 0.602`，高于 `B0 0.351` 和 `B_ours 0.335`，支持“slosh-cost-only 诱发更强速度调制”的证据链。

因此，最新判断是：当前 isolated fixed-path 仿真和实物不一致，主要问题更可能来自仿真路径/速度/里程计激励没有复现实物 P2，或仿真动力学扰动不足，而不是 slosh proxy 或 `w_slosh` 透传失效。若继续补仿真，应优先做实物轨迹/速度 profile 复刻，或把实物 `/odom`/`cmd_vel` 转成仿真可复现 reference，而不是继续盲扫 cost 权重。

### 14.5 `B_smooth_only` vs `B_ours`：隔离 slosh cost 贡献

为避免结论变成“只有 smooth 有用”，补做同 smooth-priority 下的 ablation。代码中合法 smooth-only variant 为 `B_smooth`，本节记作 `B_smooth_only`：`B_smooth` 是 smooth priority only，`w_slosh=0`；`B_ours` 是同类 smooth priority + slosh cost。

```text
result_root=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_Bsmooth_vs_Bours_explicit_map_n3_20260612_201305
bag_root=/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_Bsmooth_vs_Bours_explicit_map_n3_20260612_201305
manifest=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_Bsmooth_vs_Bours_explicit_map_n3_20260612_201305/Bsmooth_vs_Bours_manifest.csv
```

口径保持原 explicit-map strict fresh-sim：`GOAL_X=5.0`、`PATH_TEMPLATE=s_curve`、`ALPHA_MAX=1.2`、`MAX_ANGULAR_ACCEL=1.2`、每 case 30s settle + 30s post-shutdown；6/6 case valid、6/6 到点。

| variant | pass/goal | first GOAL mean s | `/slosh/height` ALL peak/p95/rms mm | `/slosh/height` 10%-90% peak/p95/rms mm | `/spmpc/slosh_height` ALL peak/p95/rms mm | `/spmpc/slosh_height` 10%-90% peak/p95/rms mm |
|---|---:|---:|---:|---:|---:|---:|
| `B_smooth_only` (`B_smooth`) | 3/3 | 17.124 | 0.967 / 0.552 / 0.270 | 0.852 / 0.521 / 0.268 | 1.272 / 0.776 / 0.376 | 1.125 / 0.746 / 0.376 |
| `B_ours` | 3/3 | 15.791 | 1.267 / 0.499 / 0.284 | 1.267 / 0.383 / 0.271 | 1.668 / 0.593 / 0.334 | 1.664 / 0.444 / 0.308 |

速度/调制与 cost：

| variant | `/cmd_vel` v max/mean/p95 | `/cmd_vel` omega max/mean/p95 | `|dv/dt|` p95/max | `|domega/dt|` p95/max | `pct_slosh_total` mean/p95/max |
|---|---:|---:|---:|---:|---:|
| `B_smooth_only` (`B_smooth`) | 0.495 / 0.258 / 0.478 | 0.503 / 0.153 / 0.474 | 0.332 / 0.621 | 0.377 / 3.917 | 0.000 / 0.000 / 0.000 |
| `B_ours` | 0.498 / 0.275 / 0.491 | 0.602 / 0.157 / 0.549 | 0.238 / 0.620 | 0.456 / 4.124 | 5.483 / 12.689 / 31.278 |

读法：在相同 smooth priority 下，加入 slosh cost 的 `B_ours` 对 p95 型液面指标有额外抑制：外部 `/slosh/height` ALL p95 下降约 `9.7%`，10%-90% p95 下降约 `26.4%`；内部 `/spmpc/slosh_height` ALL p95 下降约 `23.5%`，10%-90% p95 下降约 `40.5%`。同时 `|dv/dt|` p95 下降约 `28.4%`。这可以作为“slosh 项本身在相同 smooth 条件下有效”的直接仿真证据。

谨慎点：`B_ours` 的 peak 并未下降（外部 peak `1.267 > 0.967 mm`，内部 peak `1.668 > 1.272 mm`），外部 rms 也只是接近，且角速度调制略增。因此论文中应写成“slosh cost 主要降低持续性/主体段 p95 液面 proxy；对瞬时 peak 的抑制仍不稳定，需要 smooth/jerk 约束配合”，不要写成所有指标全面优于 smooth-only。

### 14.6 `B_ours + W_SLOSH=10`：尝试拉开 peak 差距

为进一步测试 slosh cost 对 peak 的作用，只使用当前 isolated runner 已经接通的安全 override `W_SLOSH`，不修改 world/map/URDF/spawn/localization/TF 语义。N=1 screening 显示 `B_ours + W_SLOSH=10` 的外部 10%-90% raw peak 最低，随后做 N=3 strict fresh-sim 验证。

```text
screen=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_peak_param_screen_n1_20260612_211151
result_root=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_Bsmooth_vs_Bours_w10_explicit_map_n3_20260612_211843
bag_root=/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_Bsmooth_vs_Bours_w10_explicit_map_n3_20260612_211843
manifest=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_Bsmooth_vs_Bours_w10_explicit_map_n3_20260612_211843/Bsmooth_vs_Bours_w10_manifest.csv
```

口径保持原 explicit-map strict fresh-sim：`GOAL_X=5.0`、`PATH_TEMPLATE=s_curve`、`ALPHA_MAX=1.2`、`MAX_ANGULAR_ACCEL=1.2`、每 case 30s settle + 30s post-shutdown；6/6 case valid、6/6 到点。

| variant | pass/goal | first GOAL mean s | `/slosh/height` ALL raw peak / sustained peak / p95 / rms mm | `/slosh/height` 10%-90% raw peak / sustained peak / p95 / rms mm | `/spmpc/slosh_height` ALL raw peak / p95 / rms mm | `/spmpc/slosh_height` 10%-90% raw peak / p95 / rms mm |
|---|---:|---:|---:|---:|---:|---:|
| `B_smooth_only` (`B_smooth`) | 3/3 | 17.096 | 1.119 / 1.046 / 0.528 / 0.274 | 1.100 / 1.066 / 0.477 / 0.267 | 1.440 / 0.740 / 0.369 | 1.386 / 0.581 / 0.350 |
| `B_ours_w10` (`B_ours`, `W_SLOSH=10`) | 3/3 | 16.040 | 1.068 / 1.038 / 0.489 / 0.261 | 1.058 / 1.036 / 0.365 / 0.253 | 1.261 / 0.457 / 0.280 | 1.160 / 0.401 / 0.266 |

速度/调制：`B_ours_w10` 的 `|dv/dt|` p95 从 `0.326` 降到 `0.207 m/s²`（约 `-36.3%`），但 `|domega/dt|` p95 从 `0.386` 升到 `0.483 rad/s²`（约 `+25.1%`），`cmd_omega` p95 从 `0.473` 升到 `0.550 rad/s`（约 `+16.4%`）。

判断：`W_SLOSH=10` 比默认 `B_ours` 更接近“peak 也下降”的目标；N=3 均值上外部 `/slosh/height` raw peak / sustained peak 略低于 smooth-only，内部 `/spmpc/slosh_height` peak 与 p95 降幅更明显。但外部 peak 降幅仍小（ALL sustained peak 仅约 `0.7%`，10%-90% sustained peak 约 `2.8%`），per-case 不是每一对 run 都下降，且角速度调制增加。因此目前可以写“peak 有趋势性改善”，不宜写“peak 已稳定显著降低”。

### 14.7 `V_REF=0.70 + W_SLOSH=10`：速度激励下的 peak 验证

在默认速度下 `W_SLOSH=10` 的外部 peak 差距仍偏小，因此继续只使用 isolated runner 已接通的 planner override `V_REF` 做速度激励筛选。该步骤仍保持安全边界：不修改 world/map/URDF/robot model/Gazebo model/spawn/localization/TF 语义，不 reset/move/delete/spawn robot model，不启动实物控制，不使用 broad kill/pkill。N=1 screen 中 `V_REF=0.60` 不推荐；`V_REF=0.70` 显示 `/slosh/height` peak/p95/rms 改善，因此推进 strict fresh-sim N=3。

```text
screen=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_speed_screen_n1_20260612_233035
result_root=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_speed_v070_Bsmooth_vs_Bours_w10_n3_20260612_233925
bag_root=/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_speed_v070_Bsmooth_vs_Bours_w10_n3_20260612_233925
manifest=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_speed_v070_Bsmooth_vs_Bours_w10_n3_20260612_233925/speed_v070_n3_manifest.csv
aggregate=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_speed_v070_Bsmooth_vs_Bours_w10_n3_20260612_233925/peak_sweep_metrics_aggregate.json
per_case=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_speed_v070_Bsmooth_vs_Bours_w10_n3_20260612_233925/peak_sweep_metrics_per_case.csv
```

口径保持 explicit-map strict fresh-sim：`GOAL_X=5.0`、`PATH_TEMPLATE=s_curve`、`PATH_AMPLITUDE_RATIO=0.18`、`PATH_MAX_AMPLITUDE=1.20`、`ALPHA_MAX=1.2`、`MAX_ANGULAR_ACCEL=1.2`、`V_REF=0.70`、每 case 30s settle + 30s post-shutdown；6/6 case valid、6/6 到点。

| variant | pass/goal | first GOAL mean s | `/slosh/height` ALL raw peak / p95 / rms mm | `/slosh/height` 10%-90% raw peak / p95 / rms mm | `/spmpc/slosh_height` ALL raw peak / p95 / rms mm | `/spmpc/slosh_height` 10%-90% raw peak / p95 / rms mm |
|---|---:|---:|---:|---:|---:|---:|
| `B_smooth_v070` (`B_smooth`, `V_REF=0.70`) | 3/3 | 10.903 | 1.233 / 0.961 / 0.502 | 1.233 / 0.967 / 0.568 | 1.488 / 1.024 / 0.535 | 1.488 / 1.041 / 0.599 |
| `B_ours_w10_v070` (`B_ours`, `W_SLOSH=10`, `V_REF=0.70`) | 3/3 | 11.042 | 1.018 / 0.853 / 0.460 | 1.008 / 0.856 / 0.517 | 1.317 / 0.926 / 0.489 | 1.317 / 0.921 / 0.546 |

速度/调制指标：

| variant | `/cmd_vel` v max/mean/p95 | `/cmd_vel` omega max/mean/p95 | `|dv/dt|` p95/max | `|domega/dt|` p95/max |
|---|---:|---:|---:|---:|
| `B_smooth_v070` | 0.694 / 0.383 / 0.682 | 0.780 / 0.259 / 0.731 | 0.553 / 0.621 | 1.438 / 6.646 |
| `B_ours_w10_v070` | 0.680 / 0.383 / 0.672 | 0.783 / 0.244 / 0.724 | 0.559 / 0.613 | 1.255 / 8.532 |

判断：`V_REF=0.70 + W_SLOSH=10` 是目前最接近“slosh 项对 peak 也有效”的仿真组合。相对 `B_smooth_v070`，`B_ours_w10_v070` 的外部 `/slosh/height` 10%-90% raw peak 从 `1.233` 降到 `1.008 mm`（约 `-18.3%`），10%-90% p95 从 `0.967` 降到 `0.856 mm`（约 `-11.5%`），10%-90% rms 从 `0.568` 降到 `0.517 mm`（约 `-9.0%`）；内部 `/spmpc/slosh_height` 10%-90% raw peak/p95/rms 也同步下降。p95 每个 run 都下降，外部 10%-90% p95 的 std 从 `0.0736` 降到 `0.0017 mm`，说明 p95 证据比默认速度更稳定；raw peak 是 2/3 个 run 下降，run 1 略高，因此仍应避免写成“每次 peak 都下降”。

谨慎点：本批 sustained peak 字段均为 `null`，所以结论基于 raw peak/p95/rms/spike 口径；同时 `|domega/dt|` max 从 `6.646` 升到 `8.532 rad/s²`，单 run 最高 `9.536 rad/s²`。因此该组合适合作为“仿真中证明 slosh cost 对 peak/p95 有可观贡献”的推荐证据和候选方向，但不应直接当成实物参数；若要实物迁移，应先复核角加速度峰值/jerk 限制和硬件侧限幅。

### 14.8 TEB/DWA common-limit 对比：补传统 local planner baseline

为补齐传统 local planner baseline，对 TEB/DWA 做 strict fresh-sim common-limit N=3。这里的“同参数”按跨 planner 可公平对齐的运动学约束定义：`v_max=0.8 m/s`、`omega_max=1.2 rad/s`、`acc_lim_x=0.6 m/s²`、`acc_lim_theta=1.2 rad/s²`，而不是 cost 权重相同。TEB/DWA 的 `/slosh/height` 只作为 evaluation-only 外部评估，不参与控制。

参数对齐与安全处理：

- 使用 established map：`/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream`。
- 不修改 world/map/URDF/robot model/Gazebo model/spawn/localization/TF 语义。
- 不 reset/move/delete/spawn robot model，不启动实物控制，不使用 broad kill/pkill。
- 使用 `/data/a` 下隔离 planner config 副本，避免误用 workspace 中更激进配置：
  - TEB：`/data/a/scout_sim_replacement/results/fair_baseline_configs/teb_local_planner_common_limit_forward_only.yaml`，其中 `max_vel_x_backwards=0.0`、`yaw_goal_tolerance=0.20`。
  - DWA：`/data/a/scout_sim_replacement/results/fair_baseline_configs/dwa_local_planner_common_limit.yaml`。

```text
smoke=/data/a/scout_sim_replacement/results/strict_fresh_teb_dwa_common_limit_smoke_n1_20260613_001649
result_root=/data/a/scout_sim_replacement/results/strict_fresh_teb_dwa_common_limit_n3_20260613_002101
bag_root=/data/a/scout_sim_replacement/bags/strict_fresh_teb_dwa_common_limit_n3_20260613_002101
manifest=/data/a/scout_sim_replacement/results/strict_fresh_teb_dwa_common_limit_n3_20260613_002101/strict_fresh_manifest.csv
aggregate=/data/a/scout_sim_replacement/results/strict_fresh_teb_dwa_common_limit_n3_20260613_002101/baseline_common_limit_metrics_aggregate.json
per_case=/data/a/scout_sim_replacement/results/strict_fresh_teb_dwa_common_limit_n3_20260613_002101/baseline_common_limit_metrics_per_case.csv
analyzer=/data/a/scout_sim_replacement/scripts/analyze_baseline_common_limit_metrics.py
```

N=1 smoke：2/2 valid 且到点；N=3：6/6 case valid、6/6 到点，且每个 case 的 pre/post ROS/Gazebo 检查均为 unreachable。

与同路径 `SPMPC B_smooth_v070` / `B_ours_w10_v070` 对比：

| planner / variant | pass/goal | first GOAL mean s | `/slosh/height` ALL peak / p95 / rms mm | `/slosh/height` 10%-90% peak / p95 / rms mm | `/cmd_vel` v max/mean/p95 | `/cmd_vel` omega max/mean/p95 | `|dv/dt|` p95/max | `|domega/dt|` p95/max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `DWA common-limit` | 3/3 | 18.745 | 3.191 / 1.616 / 0.776 | 2.936 / 1.735 / 0.847 | 0.549 / 0.268 / 0.547 | 0.535 / 0.164 / 0.439 | 0.569 / 1.421 | 0.838 / 1.261 |
| `TEB common-limit` | 3/3 | 6.213 | 2.030 / 1.372 / 0.784 | 2.030 / 1.285 / 0.673 | 0.759 / 0.641 / 0.749 | 0.861 / 0.249 / 0.761 | 0.561 / 3.308 | 2.173 / 6.212 |
| `SPMPC B_smooth_v070` | 3/3 | 10.903 | 1.233 / 0.961 / 0.502 | 1.233 / 0.967 / 0.568 | 0.694 / 0.383 / 0.682 | 0.780 / 0.259 / 0.731 | 0.553 / 0.621 | 1.438 / 6.646 |
| `SPMPC B_ours_w10_v070` | 3/3 | 11.042 | 1.018 / 0.853 / 0.460 | 1.008 / 0.856 / 0.517 | 0.680 / 0.383 / 0.672 | 0.783 / 0.244 / 0.724 | 0.559 / 0.613 | 1.255 / 8.532 |

判断：该批次支持写成“在相同固定 S 曲线路径、相同地图/目标和相同运动学 envelope 下，`SPMPC B_ours_w10_v070` 的外部 `/slosh/height` peak/p95/rms 均低于 TEB 和 DWA”。其中相对 DWA，10%-90% peak/p95/rms 约下降 `65.7% / 50.7% / 38.9%`；相对 TEB，约下降 `50.4% / 33.4% / 23.2%`。谨慎点是：TEB 到点最快但 slosh 更高；`B_ours_w10_v070` 的 `|domega/dt|` max 高于 TEB/DWA，实物迁移前仍需复核角加速度峰值/jerk 限制。
