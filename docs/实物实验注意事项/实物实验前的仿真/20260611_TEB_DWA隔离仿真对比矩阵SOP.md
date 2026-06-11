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
