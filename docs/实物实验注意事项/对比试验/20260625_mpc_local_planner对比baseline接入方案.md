# 20260625 mpc_local_planner 对比 baseline 接入方案

## 0. 结论

`src/scout_apps/control/mpc_local_planner` 可以作为本项目的外部对比方法，但它必须被定义为：

> **external baseline：传统 ROS navigation / nav_core 非线性 MPC local planner**

不能把它写成 `spmpc_local_planner` 的内部消融，也不能写成 “SPMPC 去掉 slosh”。

本项目接入顺序必须是：

```text
先接入隔离仿真环境 /data/a/scout_sim_replacement
  -> 通过 map/TF/RViz/fixed-path smoke
  -> 通过 fresh-sim baseline Gate
  -> 冻结参数与日志口径
  -> 再进入实物 shadow / 低速 / 带液体分级流程
```

当前推荐路线不是重新搭一个 `~/mpc_baseline_ws`，也不是直接启动旧 `/home/a/scout_ws` 仿真；而是复用当前已经存在的工程资产：

```text
/data/a/scout_sim_replacement                               # 隔离仿真环境，唯一默认仿真入口
/home/a/scout_ws/install_isolated_mpc/setup.bash            # mpc_local_planner isolated overlay
/home/a/scout_ws/src/scout_apps/control/baseline_local_planner_runner
/home/a/scout_ws/src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_standalone_sim.yaml
/data/a/scout_sim_replacement/scripts/launch_proxy_baseline_localized_attach.sh
```

一句话定位：

| 方法 | 实验定位 | 不能声称 |
| --- | --- | --- |
| `spmpc_local_planner` 的 `B0/B_smooth/B_slosh/B_ours` | internal ablation，同一 continuous MPCC/acados 框架内消融 | 外部传统 planner |
| `mpc_local_planner` | external baseline，传统 SE2 NMPC local planner | SPMPC 去掉 slosh 的同构消融 |

---

## 1. 环境安全红线

本方案必须遵守隔离仿真 SOP：

```text
docs/实物实验注意事项/仿真环境/20260611_隔离Scout仿真环境使用SOP.md
```

### 1.1 工作区边界

- 新仿真、地图、日志、结果默认只放在：

  ```text
  /data/a/scout_sim_replacement
  ```

- `/home/a/scout_ws` 只作为只读代码/接口/已编译 overlay 来源使用。
- 不修改旧仿真 world/map/URDF/model/spawn/Cartographer/TF。
- 不使用旧 `launch_sim_nav_stack.sh` 仿真入口。

- 不使用：

  ```text
  git reset / git clean / git checkout / git push
  broad killall / pkill
  ```

- 停止仿真或 baseline 时，只停止对应启动脚本追踪到的子进程 PID；如果由 Claude 后台启动，则只停止对应 task id。

### 1.2 地图硬规则

所有固定路径、可视化诊断、baseline smoke、TEB/DWA/MPC/LT-DWA 对比观察，都必须显式使用已确认地图：

```text
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
```

不要依赖 launch 默认地图。启动日志中如果没有看到这个 `MAP_FILE`，或者 RViz 中 `/map`、`/scan_front`、RobotModel、TF 没有对齐，则本次测试无效，应先停止脚本，再按 SOP 重启。

---

## 2. 审查依据

### 2.1 `mpc_local_planner` 关键事实

| 项 | 结论 | 依据 |
| --- | --- | --- |
| 插件形态 | 同时导出 `nav_core::BaseLocalPlanner` 与 `mbf_costmap_core::CostmapController` | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/mpc_local_planner_plugin.xml:1-13`；`src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:38-40` |
| 主控制入口 | `MpcLocalPlannerROS::computeVelocityCommands()` | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:254-460` |
| global plan 来源 | 原生插件不直接订阅 `/scout/global_path_fixed`，而是通过 `setPlan()` 接收路径 | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:232-251` |
| odom 来源 | `base_local_planner::OdometryHelperRos`，参数键 `odom_topic` | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:108-110,186-188,287-292` |
| pose 来源 | `costmap_2d::Costmap2DROS::getRobotPose()` + TF/costmap frame | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:282-285` |
| 求解器 | `corbo/control_box_rst` OCP，主要是 `ipopt`/`lsq_lm`，不是 acados | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/controller.cpp:380-480` |
| 机器人模型 | 传统 SE2 模型；差速模型控制量为 `[v, omega]` | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/include/mpc_local_planner/systems/base_robot_se2.h:57`；`src/scout_apps/control/mpc_local_planner/mpc_local_planner/include/mpc_local_planner/systems/unicycle_robot.h:56-79` |
| 障碍来源 | local costmap、costmap_converter、自定义 `obstacles` topic | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:474-615` |
| via-points | 可从 global plan 自动抽取，也可订阅 `via_points` | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:619-635,869-888` |
| 输出方式 | 取第一拍控制 `u0`，经 robot dynamics 转成 `Twist` 返回给上层 | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:430-441` |

### 2.2 `spmpc_local_planner` 关键事实

| 项 | 结论 | 依据 |
| --- | --- | --- |
| 节点形态 | 独立 ROS node：`spmpc_local_planner_node` | `src/scout_apps/control/spmpc_local_planner/src/spmpc_local_planner_node.cpp:3-13` |
| fixed-path launch | 默认 reference path `/scout/global_path_fixed`，cmd `/cmd_vel`，后端 `continuous_mpcc_acados` | `src/scout_apps/control/spmpc_local_planner/launch/spmpc_fixed_path.launch:1-46` |
| 主控制循环 | timer callback 内构造 `SolverInput`，调用 `problem_.solve()`，发布/限幅 `cmd_vel` | `src/scout_apps/control/spmpc_local_planner/src/ros/spmpc_local_planner_ros.cpp:805-878` |
| 默认时域 | `control_frequency=30 Hz`，`dt=0.0333333333`，`horizon_steps=60` | `src/scout_apps/control/spmpc_local_planner/config/planner/common.yaml:1-3` |
| Scout 限制 | `v_max=0.8`，`omega_max=1.2`，`a_max=0.6`，`alpha_max=1.2` | `src/scout_apps/control/spmpc_local_planner/config/platforms/scout_mini.yaml:1-15` |
| fixed-path 默认障碍 | `corridor_enable=false`，`obstacle_enable=false`，`homotopy_enable=false` | `src/scout_apps/control/spmpc_local_planner/config/experiments/fixed_path.yaml:1-15` |
| terminal gate | 到达不只看距离，还看速度和角速度门 | `src/scout_apps/control/spmpc_local_planner/src/core/terminal_controller.cpp:34-112` |
| safety gate | terminal spin fail、tracking unsafe projection、tracking spin fail | `src/scout_apps/control/spmpc_local_planner/src/ros/spmpc_local_planner_ros.cpp:654-749` |
| 诊断 | `/spmpc/*` status、solver time、cost breakdown、bounds、terminal、cmd output、slosh 等 | `src/scout_apps/control/spmpc_local_planner/src/ros/diagnostics_publisher.cpp:7-40` |

---

## 3. 当前工程实际链路

### 3.1 不再以 `move_base` 作为首选落地路径

`mpc_local_planner` 原生是 `nav_core` 插件，可以被 `move_base`/MBF 调用。但当前仓库已经有更适合 fixed-path 对比的执行器：

```text
src/scout_apps/control/baseline_local_planner_runner
```

该 runner 不实现控制算法，只负责把 `nav_core::BaseLocalPlanner` 插件从 `move_base` 中拿出来单独运行。当前固定路径 baseline 推荐链路为：

```text
/scout/global_path_fixed
/scout/goal
/odom, /tf, /map, /scan_front
        ↓
baseline_local_planner_runner
        ↓
mpc_local_planner/MpcLocalPlannerROS
        ↓
/cmd_vel
```

`move_base`/MBF 路线保留为备选，不作为当前隔离仿真首版接入主路线。

### 3.2 当前已有文件

| 角色 | 路径 |
| --- | --- |
| mpc isolated overlay | `/home/a/scout_ws/install_isolated_mpc/setup.bash` |
| baseline runner launch | `src/scout_apps/control/baseline_local_planner_runner/launch/nav_core_runner.launch` |
| project-side MPC fixed-path launch | `src/scout_apps/control/spmpc_experiments/launch/sim/run_mpc_local_planner_fixed_path_sim.launch` |
| proxy mpc baseline launch | `/data/a/scout_sim_replacement/classic_ws/src/scout_mini_proxy_nav_adapter/launch/proxy_mpc_local_planner_localized.launch` |
| attach 脚本 | `/data/a/scout_sim_replacement/scripts/launch_proxy_baseline_localized_attach.sh` |
| 当前 mpc 参数 | `src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_standalone_sim.yaml` |
| common limits | `src/scout_apps/control/spmpc_experiments/config/benchmark/common_limits.yaml` |
| project-side costmap | `src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_sim.yaml` |
| proxy costmap | `/data/a/scout_sim_replacement/classic_ws/src/scout_mini_proxy_nav_adapter/config/local_costmap_proxy_baseline.yaml` |

`run_mpc_local_planner_fixed_path_sim.launch` 作为项目侧固定路径入口，已将 runner 关键参数显式暴露并传给 `nav_core_runner.launch`：

```text
controller_frequency
base_frame
plan_target_frame
max_cmd_vel_x
max_cmd_vel_theta
planner_config
costmap_config
```

这使它能在不修改 `/data/a/scout_sim_replacement` 仿真资产的前提下，对齐当前 proxy baseline 链路，并支持后续用外层脚本覆盖频率、frame、限幅和 costmap 配置。

### 3.3 当前 baseline 默认值，不等于最终正式配置

当前 `proxy_mpc_local_planner_localized.launch` / `run_mpc_local_planner_fixed_path_sim.launch` / `mpc_local_planner_standalone_sim.yaml` 的重要默认值：

| 项 | 当前值 | 说明 |
| --- | --- | --- |
| `BASELINE` | `mpc_local_planner` | attach 脚本支持 `teb/dwa/mpc_local_planner/lt_dwa` |
| path frame | `map` | baseline attach 默认非 LT-DWA 使用 `map` |
| plan target frame | `odom` | runner 将 plan 转到该 frame |
| base frame | `base_footprint` | 注意不是纯 `base_link` |
| controller frequency | `10.0 Hz` | smoke 默认，不等于 SPMPC 30 Hz |
| solver | `lsq_lm` | 当前 YAML 中 `solver/type: lsq_lm` |
| objective | `quadratic_form` | 当前 YAML 默认 |
| grid | `grid_size_ref=20, dt_ref=0.2` | horizon 约 3.8 s |
| costmap obstacles | `include_costmap_obstacles=true` | fixed-path 公平性需单独审查 |
| orientation policy | `global_plan_overwrite_orientation=true` | 会改写局部目标朝向，正式 fixed-path 需单独决策 |

因此：

- 当前配置适合 **diagnostic smoke**；
- 不应直接宣称为 30 Hz formal baseline；
- 不应不加说明地和 SPMPC 主表对比。

---

## 4. baseline 定义和论文口径

推荐在实验表中使用：

- `SPMPC-B0`
- `SPMPC-B_smooth`
- `SPMPC-B_slosh`
- `SPMPC-B_ours`
- `MPC-local-planner` 或 `Generic NMPC local planner`

不要把 `mpc_local_planner` 命名为：

- `SPMPC-no-slosh`
- `B0`
- `MPCC-no-slosh`

原因：`mpc_local_planner` 与 SPMPC 不只差 slosh，还差 solver、状态量、控制量、路径参数化、终端 gate、ROS 接口和 warm-start 机制。

它可以回答：

> 与传统 ROS navigation NMPC local planner 相比，SPMPC 在 fixed-path 跟踪、终端稳定、控制平滑和晃液抑制上是否更好？

它不能单独回答：

> slosh penalty 本身贡献了多少？

slosh 贡献应由 `spmpc_local_planner` 内部消融组回答，例如 `B0/B_smooth/B_slosh/B_ours` 或 `w_slosh=0` sweep。

---

## 5. 仿真接入标准流程

### 5.1 Step 1：按 SOP 启动隔离仿真 + Cartographer + RViz

正式可视化和 baseline smoke 必须先打开隔离仿真环境：

```bash
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream \
USE_RVIZ=true \
/data/a/scout_sim_replacement/scripts/launch_proxy_sim_localization_env.sh
```

该命令只启动：

```text
roscore
proxy Gazebo sim
Cartographer localization
proxy tracking RViz
```

不会启动 SPMPC，也不会发布 fixed path。

启动后先确认 RViz 中能看到：

```text
/map
/scan_front
RobotModel
TF
```

常用检查：

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11328

rostopic echo -n 1 /map
rostopic echo -n 1 /scan_front
rosrun tf tf_echo map base_link
```

若地图、雷达、RobotModel、TF 没对齐，直接停止该环境，不要继续 attach baseline。

### 5.2 Step 2A：attach `mpc_local_planner` baseline

另开终端或新 shell，接入 MPC baseline：

```bash
export ROS_MASTER_URI=http://localhost:11328
export GAZEBO_MASTER_URI=http://localhost:11362

BASELINE=mpc_local_planner \
GOAL_X=5.0 GOAL_Y=0.0 GOAL_YAW=0.0 \
PATH_TEMPLATE=s_curve \
PATH_START_HEADING=current \
/data/a/scout_sim_replacement/scripts/launch_proxy_baseline_localized_attach.sh
```

如果本轮要记录外部 slosh proxy，必须显式开启：

```bash
SLOSH_MONITOR_ENABLE=true \
BASELINE=mpc_local_planner \
GOAL_X=5.0 GOAL_Y=0.0 GOAL_YAW=0.0 \
PATH_TEMPLATE=s_curve \
PATH_START_HEADING=current \
/data/a/scout_sim_replacement/scripts/launch_proxy_baseline_localized_attach.sh
```

默认 `SLOSH_MONITOR_ENABLE=false`，不显式打开就没有 `/slosh/height`。

### 5.3 Step 2B：attach SPMPC 对照

同一环境中如果要测 SPMPC，应停止 MPC baseline 后再启动 SPMPC：

```bash
export ROS_MASTER_URI=http://localhost:11328
export GAZEBO_MASTER_URI=http://localhost:11362

GOAL_X=5.0 GOAL_Y=0.0 GOAL_YAW=0.0 \
PATH_TEMPLATE=s_curve \
/data/a/scout_sim_replacement/scripts/launch_proxy_spmpc_localized_attach.sh
```

同一时间只允许一个控制器发布 `/cmd_vel`。

### 5.4 停止方式

- 启动终端中使用 `Ctrl-C`。
- Claude 后台任务中只停止对应 task id。
- 不使用 `killall` / `pkill`。
- 每次停止后检查是否还有残留 `/cmd_vel` publisher、baseline node 或 SPMPC node。

---

## 6. 公平性对齐清单

### 6.1 运行条件

| 项 | SPMPC 当前口径 | `mpc_local_planner` 接入口径 |
| --- | --- | --- |
| 仿真入口 | `/data/a/scout_sim_replacement` | 必须相同 |
| 地图 | `proxy_world_manual_saved_20260611_154348.pbstream` | 必须相同且显式 `MAP_FILE` |
| 正式对比启动 | fresh sim / fresh run | 必须相同；已运行环境只能标记 current-sim diagnostic |
| 起点 | 同一 spawn / 同一人工复位点 | 必须相同 |
| 路径 | `/scout/global_path_fixed` | runner 收到的 global plan 必须同源 |
| goal | `/scout/goal` | 必须同源 |
| timeout | 60 s 内未到终点记 FAIL | 相同 |
| 控制权 | 只允许一个 `/cmd_vel` 发布者 | 运行 mpc baseline 时必须停掉 SPMPC 节点 |
| slosh 信息 | 只评价，不反馈 planner | 相同 |

### 6.2 约束参数映射

正式对比前必须生成一份“实际参数记录表”，至少包含下列映射。

| 约束 | SPMPC 参数 | `mpc_local_planner` 参数 | 建议值/规则 |
| --- | --- | --- | --- |
| 最大线速度 | `robot/v_max` | `MpcLocalPlannerROS/robot/unicycle/max_vel_x` | `0.8` |
| 倒车速度 | SPMPC 若无明确倒车能力则禁用 | `MpcLocalPlannerROS/robot/unicycle/max_vel_x_backwards` | 首版设 `0.0` |
| 最大角速度 | `robot/omega_max` | `MpcLocalPlannerROS/robot/unicycle/max_vel_theta` | `1.2` |
| 线加速度 | `robot/a_max`、shared linear accel | `MpcLocalPlannerROS/robot/unicycle/acc_lim_x` | `0.6` |
| 线减速度 | `robot/a_max` 或制动限制 | `MpcLocalPlannerROS/robot/unicycle/dec_lim_x` | 建议 `0.6` |
| 角加速度 | `robot/alpha_max` | `MpcLocalPlannerROS/robot/unicycle/acc_lim_theta` | 默认 `1.2`；若 SPMPC launch 改了 `alpha_max`，必须同步 |
| 控制频率 | `control_frequency` | runner `controller_frequency` | smoke 可 10 Hz；formal 需冻结并说明是否同频 |
| OCP 步长 | `dt` | `MpcLocalPlannerROS/grid/dt_ref` | 不强求等于 SPMPC，但必须记录 |
| 预测时域 | `horizon_steps * dt ≈ 2.0 s` | `(grid_size_ref - 1) * dt_ref` | 候选配置单独验证 |
| 到达距离 | `goal_tolerance` | `MpcLocalPlannerROS/controller/xy_goal_tolerance` | `0.20 m` 或本轮统一值 |
| 到达速度门 | `goal_reached_max_speed` | 插件内部无同构门 | 用离线 evaluator 统一判定 `<=0.03 m/s` |
| 到达角速度门 | `goal_reached_max_omega` | 插件内部无同构门 | 用离线 evaluator 统一判定 `<=0.05 rad/s` |

### 6.3 frame 对齐

当前隔离 baseline 链路的实际 frame 口径：

```yaml
frames:
  global_path_frame: map
  local_plan_target_frame: odom
  baseline_base_frame: base_footprint
  robot_visual_frame: base_link
```

不要在文档或 evaluator 中简单写成“全部 base_link”。正式评估时应通过 TF 统一投影到同一 fixed path frame。

### 6.4 需要单独决策的参数

下列参数决定 baseline 语义，不能默认混用：

| 参数 | 当前值 | formal fixed-path 建议 |
| --- | --- | --- |
| `global_plan_overwrite_orientation` | `true` | 如果要求 strict fixed-path 姿态语义，设 `false`；如果保留 `true`，表格中标注 navigation-style baseline |
| `include_costmap_obstacles` | `true` | fixed-path 无障碍主表中需确认是否与 DWA/TEB/SPMPC 口径一致 |
| `controller_frequency` | `10 Hz` | 先 smoke；formal 同频需验证 15/20/30 Hz 实时性 |
| `solver/type` | `lsq_lm` | 可作为 smoke；正式是否改 `ipopt` 需单独 validation |
| `objective` | `quadratic_form` | 可与 `minimum_time_via_points` 在 validation set 上比较，冻结一套 |

---

## 7. 仿真阶段 Gate

### Gate S0：环境安全与地图审计

- 使用 `/data/a/scout_sim_replacement`；
- 启动日志出现正确 `MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream`；
- RViz 中 `/map`、`/scan_front`、RobotModel、TF 对齐；
- `/home/a/scout_ws` 只被 source/read-only，不启动旧仿真；
- 无 broad `killall/pkill`。

### Gate S1：baseline attach 接线通过

- `baseline_local_planner_runner` 成功加载 `mpc_local_planner/MpcLocalPlannerROS`；
- `/baseline/mpc_local_planner/status` 正常发布；
- `/baseline/mpc_local_planner/global_plan` 与 `/scout/global_path_fixed` 同源；
- 只有一个 `/cmd_vel` 发布者；
- 无明显 TF/costmap/odom 报错。

### Gate S2：current-sim diagnostic smoke

目标：快速证明接线可跑，但不作为正式对比。

允许：

- 使用当前已经启动的 sim；
- 不重启/重置 Gazebo/RViz；
- 结果只能写为 `current-sim diagnostic`。

检查：

1. 能输出非零 `/cmd_vel`；
2. `/baseline/mpc_local_planner/global_plan` 正常；
3. `/move_base/MpcLocalPlannerROS/ocp_result` 或 runner 可用诊断正常；
4. command 不明显超 common limits；
5. 若失败，能分类是 TF、global plan、solver、costmap 还是控制性能。

### Gate S3：fresh-sim fixed-path smoke

目标：正式准入 baseline。

流程：

1. 按 SOP fresh 启动仿真 + Cartographer + RViz；
2. 同一 spawn 起点；
3. 等待 `/map`、`/scan_front`、TF 稳定；
4. attach `mpc_local_planner`；
5. 记录 rosbag；
6. 用统一 evaluator 判断 60 s 内是否达到终点。

准入标准：

- 至少连续 2 次 fresh-sim smoke 不因接线/TF/costmap 崩溃；
- 不出现双 `/cmd_vel` publisher；
- 不出现明显超限 command；
- 能输出完整 bag 和 baseline debug topics。

### Gate S4：formal sim validation

在正式纳入对比表前，至少完成：

```text
直线
单左弯
单右弯
低曲率 S 弯
正式 S 弯
```

每类先 N=2 pilot，再 N=3/5 正式。若成本高，必须先写明 N 和统计口径。

formal validation 中才允许比较：

```text
quadratic_form vs minimum_time_via_points
10 Hz vs 15/20/30 Hz
orientation overwrite true vs false
costmap obstacles on vs off
```

最终论文/报告只冻结一套 `mpc_local_planner` 配置，不能看 test 结果后再挑最优配置进主表。

---

## 8. command gate 与实物前置要求

### 8.1 当前仿真 smoke 阶段

当前 proxy baseline 默认是：

```text
mpc_local_planner -> /cmd_vel
```

这适合仿真 smoke，但不等于实物安全链。

### 8.2 正式仿真/实物阶段

正式实物前必须形成 raw/post-gate 分离：

```text
mpc_local_planner
  -> /benchmark/mpc_local_planner/cmd_vel_raw
  -> common command gate
  -> /cmd_vel
```

common command gate 只能做所有 planner 共享的：

```text
v / omega hard bound
a / alpha rate limit
stale command timeout
emergency stop
path-departure stop
goal stop
```

禁止给 `mpc_local_planner` 单独加入会提高跟踪性能的二次控制器。若 common gate 频繁介入，该参数配置不合格，而不是把 gate 后结果当成原算法性能。

---

## 9. 统一记录与指标

### 9.1 必录 topic

公共 topic：

```text
/cmd_vel
/odom
/tf
/tf_static
/scout/global_path_fixed
/scout/goal
/map
/scan_front
```

SPMPC 专用：

```text
/spmpc/status
/spmpc/solver_time_ms
/spmpc/cost_breakdown
/spmpc/debug/cmd_vel_output
/spmpc/debug/runtime_bounds
/spmpc/terminal/debug
/spmpc/slosh_height
/spmpc/debug/slosh_state
```

mpc baseline 专用：

```text
/baseline/mpc_local_planner/status
/baseline/mpc_local_planner/global_plan
/move_base/MpcLocalPlannerROS/global_plan
/move_base/MpcLocalPlannerROS/local_plan
/move_base/MpcLocalPlannerROS/mpc_markers
/move_base/MpcLocalPlannerROS/ocp_result
```

如果做晃液主指标，必须记录独立真值源：

```text
camera RGB / liquid surface video topics
```

不要用 `/spmpc/slosh_height` 直接和外部 baseline 比主结果；它是 SPMPC 模型 proxy，不是跨方法公平真值。

### 9.2 统一指标

| 指标 | 计算口径 |
| --- | --- |
| success | 60 s 内满足统一终点判据 |
| time-to-goal | 从首个有效 cmd 或统一 start stamp 到 goal reached |
| path tracking error | 对同一 fixed path 做离线投影，统计 mean/p95/max |
| terminal error | 最终位置误差、速度、角速度 |
| command smoothness | `cmd_v/cmd_omega` 的 jerk、omega-rate p95/max |
| limit violation | 是否超过 common limits；若被限幅，统计次数 |
| spin fail | tracking/terminal 高角速度持续时间离线统计 |
| slosh | 离线 RGB max-LCR 或同一外部测量口径 |
| compute cost | SPMPC 用 `/spmpc/solver_time_ms`；mpc baseline 优先用 `ocp_result`，若字段不足则补日志/外部计时 |

---

## 10. 从仿真迁移到实物

只有通过 Gate S0-S4，才能进入实物。实物不是“把仿真 launch 换成实车 topic”这么简单，必须分级。

### R0：工控机离线 / bag replay shadow

目的：验证插件、TF、CPU、solver、日志，不输出到底盘。

要求：

- 播放仿真或历史实物 `/odom`、`/tf`、path、goal；
- 运行 `baseline_local_planner_runner + mpc_local_planner`；
- 输出只记录到 raw topic，不发 `/cmd_vel`；
- 统计 solve time、deadline miss、command spike。

### R1：实车 command shadow

目的：接真实传感器与定位，但不控制底盘。

要求：

- raw command 只记录；
- common gate 在线运行但输出不接底盘；
- 急停链路人工确认；
- 检查方向、frame、goal、path 投影是否正确。

### R2：架空轮 / 低速空载

目的：确认方向和速度符号。

初始限制建议低于正式 common limits：

```text
v_max = 0.20 ~ 0.30 m/s
omega_max = 0.30 ~ 0.50 rad/s
```

通过条件：

- 无突然倒车；
- 无大角速度尖峰；
- stale timeout 工作；
- hard zero 工作。

### R3：无液体地面直线/单弯/S 弯

顺序：

```text
直线
左弯
右弯
低曲率 S
正式 S
```

每级通过再进入下一级。若任一级出现 solver timeout、tracking diverged、command gate excessive，停止调参记录，不继续带液体。

### R4：空容器

目的：验证容器安装、相机视角、振动、底盘负载变化。

要求：

- 不装液体；
- 记录 RGB、odom、raw/executed cmd、local plan；
- 检查容器固定和相机不遮挡。

### R5：带液体低速

正式带液体前先低速：

- 同一水位；
- 同一起点；
- 同一固定路径；
- 方法随机/交错运行，减少水位、电池、时间漂移偏差；
- 每方法至少 N=3 pilot，正式建议 N=5。

必须记录：

```text
RGB
/slosh/height 或外部 slosh monitor proxy
/odom
raw/executed cmd
local plan
solver time
/tf
/scout/global_path_fixed
```

---

## 11. 主要风险与规避

### 风险 1：它不是严格 fixed-path tracker

`mpc_local_planner` 会 prune global plan，并按 local costmap 和 lookahead 截断局部路径。它不是把整条 fixed path 作为 MPCC reference spline 优化。

规避：

- 论文/报告中明确称为 external NMPC local planner；
- 强制 global plan 与 fixed path 同源；
- 记录 `/baseline/mpc_local_planner/global_plan`、`/move_base/MpcLocalPlannerROS/global_plan` 和 local trajectory。

### 风险 2：global plan 姿态被重写

当前配置 `global_plan_overwrite_orientation=true`，可能改变 fixed-path 姿态语义。

规避：

- strict fixed-path formal 配置中优先测试 `false`；
- 如果保留 `true`，实验表中标注为 navigation-style baseline。

### 风险 3：频率和 dt 语义不同

`mpc_local_planner` 的控制变化约束使用 `dt = 1/controller_frequency` 的名义值，不一定等于真实 loop 时间。当前 baseline 默认 10 Hz，不等于 SPMPC 30 Hz。

规避：

- 记录实际 callback interval；
- 若只能 10 Hz 实时运行，报告中明确写 validated frequency；
- 不把 10 Hz smoke 写成 30 Hz 同频主表。

### 风险 4：没有最终 command 饱和保险

源码中返回 `Twist` 前的 velocity saturate 逻辑被注释。

规避：

- 仿真阶段先用 OCP bounds + bag 审计；
- 实物前必须使用 common command gate；
- gate 介入率过高则配置不合格。

### 风险 5：障碍能力不对等

`mpc_local_planner` 有 costmap/costmap_converter obstacle 体系，而 SPMPC fixed-path 默认 `obstacle_enable=false`。

规避：

- fixed-path slosh 主表不要混入“谁避障更强”的结论；
- 如果单独做导航避障 baseline，另开实验矩阵；
- costmap on/off 必须和 DWA/TEB/SPMPC 口径同步说明。

### 风险 6：实物安全被 planner 输出绑架

外部 planner 可能偶发 solver fail、cmd spike、stale command。

规避：

- raw/post-gate 分离；
- emergency stop；
- stale timeout；
- path departure stop；
- hard zero；
- 实物每一级只通过后进入下一级。

---

## 12. 立即执行清单

```text
[ ] 按 SOP 用正确 MAP_FILE 启动隔离仿真 + RViz
[ ] 确认 /map /scan_front RobotModel TF 对齐
[ ] attach BASELINE=mpc_local_planner current-sim diagnostic smoke
[ ] 记录当前默认参数：10Hz / lsq_lm / quadratic / grid 20x0.2 / costmap on / orientation overwrite true
[ ] 检查 /baseline/mpc_local_planner/status 和 global_plan
[ ] 检查只有一个 /cmd_vel 发布者
[ ] fresh-sim smoke N=2
[ ] 决策 formal 参数：frequency / horizon / objective / orientation / costmap
[ ] 形成 raw cmd -> common gate -> /cmd_vel 链路
[ ] formal sim validation N>=3
[ ] bag replay shadow
[ ] 实车 shadow
[ ] 架空轮或低速空载
[ ] 无液体直线/单弯/S 弯
[ ] 空容器
[ ] 带液体低速 pilot
```

---

## 13. 最终纳入主表条件

只有同时满足以下条件，`mpc_local_planner` 才能进入主文/主表 external baseline：

```text
1. 隔离仿真正确 MAP_FILE 与 RViz/TF/map/scan 对齐；
2. fixed-path global plan 与 SPMPC 同源；
3. common limits 已映射到 planner 参数；
4. 60 s timeout 与统一 evaluator 生效；
5. fresh-sim formal validation 通过；
6. solver fail / deadline miss 可解释且未破坏控制；
7. raw/post-gate command 记录完整；
8. slosh/RGB 只评价，不反馈 planner；
9. 参数、commit、地图、launch、bag 均可追溯；
10. 实物前完成 shadow 和低速无液体 Gate。
```

主表表述建议：

> `mpc_local_planner` was configured as an external differential-drive nonlinear MPC local-planner baseline under the same fixed global path, kinematic envelope, timeout, and evaluation pipeline as SPMPC. It did not receive liquid-state or RGB information; liquid motion was evaluated only by common external observers and offline RGB metrics.
