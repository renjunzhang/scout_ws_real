# 20260625 mpc_local_planner 对比 baseline 接入方案

## 0. 结论

`src/scout_apps/control/mpc_local_planner` 可以接入本项目的 SPMPC 对比试验，但它应被定义为 **external baseline：传统 ROS navigation NMPC local planner**，不能定义为 `spmpc_local_planner` 的“无晃液消融版本”。

建议第一版接入目标：

- 只做 fixed-path、无障碍、无 homotopy 的公平 smoke；
- 统一路径、起点、终点、frame、速度/角速度/加速度限制、60 s timeout；
- 用外部离线 evaluator 判定 success/time/path-error/slosh，不依赖各 planner 内部 `isGoalReached()` 口径；
- 先以 `move_base`/MBF 插件方式运行 `mpc_local_planner/MpcLocalPlannerROS`，待 smoke 稳定后再考虑为它写薄 wrapper。

一句话定位：

| 方法 | 实验定位 | 不能声称 |
| --- | --- | --- |
| `spmpc_local_planner` 的 `B0/B_smooth/B_slosh/B_ours` | internal ablation，同一 continuous MPCC/acados 框架内消融 | 外部传统 planner |
| `mpc_local_planner` | external baseline，传统 SE2 NMPC local planner | SPMPC 去掉 slosh 的同构消融 |

---

## 1. 审查依据

### 1.1 `mpc_local_planner` 关键事实

| 项 | 结论 | 依据 |
| --- | --- | --- |
| 插件形态 | 同时导出 `nav_core::BaseLocalPlanner` 与 `mbf_costmap_core::CostmapController` | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/mpc_local_planner_plugin.xml:1-13`；`src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:38-40` |
| 主控制入口 | `MpcLocalPlannerROS::computeVelocityCommands()` | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:254-460` |
| global plan 来源 | 不直接订阅 `/scout/global_path_fixed`，而是通过 `setPlan()` 接收 move_base/MBF 传入路径 | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:232-251` |
| odom 来源 | `base_local_planner::OdometryHelperRos`，参数键 `odom_topic` | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:108-110,186-188,287-292` |
| pose 来源 | `costmap_2d::Costmap2DROS::getRobotPose()` + TF/costmap frame | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:282-285` |
| 求解器 | `corbo/control_box_rst` OCP，主要是 `ipopt`/`lsq_lm`，不是 acados | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/controller.cpp:380-480` |
| 机器人模型 | 传统 SE2 模型；差速模型控制量为 `[v, omega]` | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/include/mpc_local_planner/systems/base_robot_se2.h:57`；`src/scout_apps/control/mpc_local_planner/mpc_local_planner/include/mpc_local_planner/systems/unicycle_robot.h:56-79` |
| 障碍来源 | local costmap、costmap_converter、自定义 `obstacles` topic | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:474-615` |
| via-points | 可从 global plan 自动抽取，也可订阅 `via_points` | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:619-635,869-888` |
| 输出方式 | 取第一拍控制 `u0`，经 robot dynamics 转成 `Twist` 返回给 move_base/MBF | `src/scout_apps/control/mpc_local_planner/mpc_local_planner/src/mpc_local_planner_ros.cpp:430-441` |

### 1.2 `spmpc_local_planner` 关键事实

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

## 2. baseline 定义和论文口径

### 2.1 推荐命名

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

### 2.2 它能回答的问题

可以回答：

> 与传统 ROS navigation NMPC local planner 相比，SPMPC 在 fixed-path 跟踪、终端稳定、控制平滑和晃液抑制上是否更好？

不能单独回答：

> slosh penalty 本身贡献了多少？

slosh 贡献应由 `spmpc_local_planner` 内部消融组回答，例如 `B0/B_smooth/B_slosh/B_ours` 或 `w_slosh=0` sweep。

---

## 3. 首版接入方案：move_base/MBF 插件方式

### 3.1 推荐架构

首版不要改 `mpc_local_planner` 源码，按它原生形态接入：

```text
fixed path generator / goal source
        |
        v
move_base 或 MBF
        |
        | setPlan(global_plan)
        v
mpc_local_planner/MpcLocalPlannerROS
        |
        | computeVelocityCommands()
        v
/cmd_vel
```

同时保留公共记录链路：

```text
/odom
/tf, /tf_static
/scout/global_path_fixed
/scout/goal
/cmd_vel
RGB / camera topics
/move_base/MpcLocalPlannerROS/local_plan
/move_base/MpcLocalPlannerROS/global_plan
/move_base/MpcLocalPlannerROS/ocp_result
```

### 3.2 必须准备的配置资产

建议新增专门的 baseline 配置，不直接改上游 demo 配置：

```text
src/scout_apps/control/mpc_local_planner/mpc_local_planner_examples/cfg/scout_fixed_path/
  mpc_local_planner_params_spmpc_common.yaml
  costmap_common_params.yaml
  local_costmap_params.yaml
  global_costmap_params.yaml

src/scout_apps/control/mpc_local_planner/mpc_local_planner_examples/launch/
  scout_fixed_path_mpc_local_baseline.launch
```

如果后续希望和 SPMPC 的脚本放在同一调度层，也可以另建：

```text
src/scout_apps/control/spmpc_local_planner/scripts/run_mpc_local_baseline_fixed_path.sh
```

但第一版建议只做 launch/config，不写 wrapper。

### 3.3 move_base 必要参数

move_base 层至少应设置：

```yaml
base_local_planner: mpc_local_planner/MpcLocalPlannerROS
controller_frequency: 30.0
planner_frequency: 0.0        # fixed-path 对比中不建议在线重规划；实际是否可用需按现有 move_base 配置验证
planner_patience: 5.0
controller_patience: 5.0
```

注意：`planner_frequency: 0.0` 的含义依赖 move_base/global planner 配置；如果当前全局路径必须由 global planner 周期更新，则不能机械照抄。正式方案应保证 **传给 `setPlan()` 的 path 与 `/scout/global_path_fixed` 同源**。

---

## 4. 公平性对齐清单

### 4.1 运行条件

| 项 | SPMPC 当前口径 | `mpc_local_planner` 接入口径 |
| --- | --- | --- |
| 仿真/实物起点 | 同一 spawn / 人工复位到同一起点 | 必须相同 |
| 正式对比启动 | fresh sim / fresh run | 必须相同；若只用当前 sim，只能标记为 current-sim diagnostic |
| 路径 | `/scout/global_path_fixed` | `setPlan()` 收到的 global plan 必须与该 path 同源 |
| goal | 同一终点 | move_base goal 与 `/scout/goal` 同源 |
| timeout | 60 s 内未到终点记 FAIL | 相同 |
| 障碍 | fixed-path 首版默认不开 obstacle/corridor/homotopy | 首版不启用额外 obstacle 优势；costmap 仅作为导航栈/footprint 支撑 |
| 控制权 | 只允许一个 `/cmd_vel` 发布者 | 运行 mpc baseline 时必须停掉 SPMPC 节点 |

### 4.2 约束参数映射

正式对比前必须生成一份“实际参数记录表”，至少包含下列映射。

| 约束 | SPMPC 参数 | `mpc_local_planner` 参数 | 建议值/规则 |
| --- | --- | --- | --- |
| 最大线速度 | `robot/v_max` | `MpcLocalPlannerROS/robot/unicycle/max_vel_x` | `0.8` |
| 倒车速度 | SPMPC 若无明确倒车能力则禁用 | `MpcLocalPlannerROS/robot/unicycle/max_vel_x_backwards` | 建议首版设 `0.0`；若 SPMPC 允许倒车，再同幅值对齐 |
| 最大角速度 | `robot/omega_max` | `MpcLocalPlannerROS/robot/unicycle/max_vel_theta` | `1.2` |
| 线加速度 | `robot/a_max`、shared linear accel | `MpcLocalPlannerROS/robot/unicycle/acc_lim_x` | `0.6` |
| 线减速度 | `robot/a_max` 或制动限制 | `MpcLocalPlannerROS/robot/unicycle/dec_lim_x` | 建议 `0.6`，若实车制动另有限制则同步记录 |
| 角加速度 | `robot/alpha_max` | `MpcLocalPlannerROS/robot/unicycle/acc_lim_theta` | 默认 `1.2`；若 SPMPC launch 改了 `alpha_max`，必须同步 |
| 控制频率 | `control_frequency` | move_base `controller_frequency` | `30.0 Hz` |
| OCP 步长 | `dt` | `MpcLocalPlannerROS/grid/dt_ref` | `0.0333333333` |
| 预测时域 | `horizon_steps * dt ≈ 2.0 s` | `(grid_size_ref - 1) * dt_ref` | 精确对齐建议 `grid_size_ref=61`；若算力不够，需单独标注 |
| 到达距离 | `goal_tolerance` | `MpcLocalPlannerROS/controller/xy_goal_tolerance` | `0.20 m` 或本轮统一值 |
| 到达速度门 | `goal_reached_max_speed` | 插件内部无同构门 | 用离线 evaluator 统一判定 `<=0.03 m/s` |
| 到达角速度门 | `goal_reached_max_omega` | 插件内部无同构门 | 用离线 evaluator 统一判定 `<=0.05 rad/s` |

### 4.3 path/frame 对齐

SPMPC fixed-path 当前是：

- odom：`/odom`
- path：`/scout/global_path_fixed`
- cmd：`/cmd_vel`
- base frame：`base_link`
- pose：默认优先 TF pose，回退受 frame 约束

`mpc_local_planner` 接入时必须检查：

1. move_base global frame、local costmap global frame、path frame 是否与 SPMPC 记录口径一致；
2. `odom_topic` 是否指向同一个 `/odom`；
3. base frame 是否同为 `base_link`；
4. `/scout/global_path_fixed` 是否被桥接/转换成 move_base 的 global plan；
5. 不能让 global planner 生成另一条路径来替代 fixed path。

### 4.4 关闭会改变 fixed-path 语义的选项

首版 baseline 建议：

```yaml
MpcLocalPlannerROS:
  controller:
    global_plan_overwrite_orientation: false
    allow_init_with_backward_motion: false
    global_plan_viapoint_sep: -1.0   # 若要自定义 via_points，则避免自动 via-points 与自定义互斥
```

理由：

- `global_plan_overwrite_orientation=true` 会根据局部路径方向重写目标朝向，改变 fixed-path 姿态语义；
- `allow_init_with_backward_motion=true` 会让初始化猜测倒车，若 SPMPC 不允许倒车则不公平；
- 自定义 `via_points` 与自动 via-points 有互斥逻辑，需明确只选一种。

---

## 5. 分阶段实施计划

### Phase A：只读/配置准备

目标：不动 planner 源码，只增加 baseline launch/config。

任务：

1. 基于 `mpc_local_planner_examples/cfg/diff_drive/mpc_local_planner_params_quadratic_form.yaml` 新建 Scout fixed-path 参数文件；
2. 设置 unicycle 模型与 common limits：`0.8 / 1.2 / 0.6 / 1.2`；
3. 设置 `controller_frequency=30 Hz`、`dt_ref=1/30`；
4. 根据算力决定 `grid_size_ref=61` 或保守较小值，并在文档/记录中写明；
5. 新建 launch，使 `base_local_planner=mpc_local_planner/MpcLocalPlannerROS`；
6. 明确 global plan 来源：必须与 `/scout/global_path_fixed` 同源。

验收：

- `rosparam get /move_base/base_local_planner` 为 `mpc_local_planner/MpcLocalPlannerROS`；
- `/move_base/MpcLocalPlannerROS/global_plan` 与 `/scout/global_path_fixed` 几何一致；
- 只有一个 `/cmd_vel` 发布者。

### Phase B：current-sim diagnostic smoke

目标：快速证明接线可跑，但不作为正式 Gate 1。

约束：

- 可以使用当前已经启动的 sim；
- 不重启/重置 Gazebo/RViz；
- 结果只能写为 `current-sim diagnostic`，不能写为 formal fresh-sim 对比。

检查：

1. `move_base` 能加载 `MpcLocalPlannerROS`；
2. `computeVelocityCommands()` 能输出非零 `/cmd_vel`；
3. `/move_base/MpcLocalPlannerROS/local_plan` 正常发布；
4. 无明显 TF/costmap/odom 报错；
5. 输出速度不超过 common limits。

### Phase C：fresh-sim fixed-path smoke

目标：正式准入 baseline。

流程：

1. fresh Gazebo/RViz；
2. 同一 spawn 起点；
3. 等待定位/TF/仿真稳定；
4. 发布同一 `/scout/global_path_fixed` 与同一 goal；
5. 启动 mpc baseline；
6. 记录 rosbag；
7. 用统一 evaluator 判断 60 s 内是否达到终点。

准入标准：

- 至少连续 2 次 fresh-sim smoke 不因接线/TF/costmap 崩溃；
- 不出现双 `/cmd_vel` publisher；
- 不出现明显超限 command；
- 能输出完整 bag 和 baseline debug topics。

### Phase D：N 次正式对比

建议顺序：

1. `SPMPC-B0`
2. `SPMPC-B_ours`
3. `MPC-local-planner`
4. 如做更多外部 baseline，再接 TEB/DWA

每个方法至少 N=3，若实物实验成本高，先 N=2 pilot，再 N=3/5 正式。

每次切换方法必须：

- 复位到同一起点；
- 等待液体状态稳定；
- 清楚记录方法名、参数文件、commit、bag 名；
- 检查 `/cmd_vel` 发布者唯一。

### Phase E：可选薄 wrapper

只有当 Phase C/D 证明 `move_base` 方案可用但公平性仍受限时，才考虑薄 wrapper。

wrapper 目标：

- 订阅 `/odom` 和 `/scout/global_path_fixed`；
- 内部复用 `MpcLocalPlannerROS` 或直接调用 Controller；
- 发布 `/cmd_vel`；
- 额外发布 baseline 诊断，如 solver time、limit trigger、goal gate 状态。

注意：这是第二阶段工程，不建议作为首版接入前置条件。

---

## 6. 统一记录与指标

### 6.1 必录 topic

公共 topic：

```text
/cmd_vel
/odom
/tf
/tf_static
/scout/global_path_fixed
/scout/goal
/map                 # 如果本轮 navigation stack/costmap 使用 map
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
/move_base/MpcLocalPlannerROS/global_plan
/move_base/MpcLocalPlannerROS/local_plan
/move_base/MpcLocalPlannerROS/mpc_markers
/move_base/MpcLocalPlannerROS/ocp_result
/move_base/status
```

如果做晃液主指标，必须记录独立真值源：

```text
camera RGB / liquid surface video topics
```

不要用 `/spmpc/slosh_height` 直接和外部 baseline 比主结果；它是 SPMPC 模型 proxy，不是跨方法公平真值。

### 6.2 统一指标

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

## 7. 主要风险与规避

### 风险 1：它不是严格 fixed-path tracker

`mpc_local_planner` 会 prune global plan，并按 local costmap 和 lookahead 截断局部路径。它不是把整条 fixed path 作为 MPCC reference spline 优化。

规避：

- 论文/报告中明确称为 external NMPC local planner；
- 强制 global plan 与 fixed path 同源；
- 记录 `/move_base/MpcLocalPlannerROS/global_plan` 和 `/local_plan`，证明实际跟踪路径。

### 风险 2：global plan 姿态被重写

规避：

- 首版设 `global_plan_overwrite_orientation=false`；
- 若必须开启，实验表中单独标注。

### 风险 3：dt 语义不同

`mpc_local_planner` 的控制变化约束使用 `dt = 1/controller_frequency` 的名义值，不一定等于真实 loop 时间。

规避：

- controller frequency 固定 30 Hz；
- 记录实际 `/cmd_vel` 时间戳间隔；
- 若 loop 抖动大，结果只作为工程 baseline，不能做精确动态约束对比。

### 风险 4：没有最终 command 饱和保险

源码中返回 `Twist` 前的 velocity saturate 逻辑被注释。

规避：

- 首版先用 OCP bounds + bag 审计；
- 若发现超限，再考虑在外层加通用 command guard，但必须对所有 baseline 一致启用。

### 风险 5：障碍能力不对等

`mpc_local_planner` 有 costmap/costmap_converter obstacle 体系，而 SPMPC fixed-path 默认 `obstacle_enable=false`。

规避：

- 首版 fixed-path baseline 不启用额外 obstacle 优势；
- 如果单独做“导航避障外部 baseline”实验，则另开表格，不与 fixed-path slosh 主表混合。

### 风险 6：move_base 管线引入额外变量

global planner、costmap、recovery behavior、goal tolerance 都可能影响结果。

规避：

- 关闭 recovery behaviors 或至少固定；
- 固定 global planner 输出；
- 不让 move_base 在线重规划改变 fixed path；
- 所有配置随 bag 归档。

---

## 8. 最小参数草案

下面不是最终配置，只是首版参数方向。实际落地时必须以 YAML 文件和 `rosparam dump` 为准。

```yaml
MpcLocalPlannerROS:
  odom_topic: /odom

  robot:
    type: unicycle
    unicycle:
      max_vel_x: 0.8
      max_vel_x_backwards: 0.0
      max_vel_theta: 1.2
      acc_lim_x: 0.6
      dec_lim_x: 0.6
      acc_lim_theta: 1.2

  grid:
    type: fd_grid
    grid_size_ref: 61        # (61 - 1) * 1/30 = 2.0 s
    dt_ref: 0.0333333333
    warm_start: true
    variable_grid:
      enable: false

  controller:
    xy_goal_tolerance: 0.20
    yaw_goal_tolerance: 0.30       # 不作为最终成功判据，最终用离线 evaluator
    global_plan_overwrite_orientation: false
    global_plan_prune_distance: 0.2
    max_global_plan_lookahead_dist: 2.0
    global_plan_viapoint_sep: -1.0
    allow_init_with_backward_motion: false
    publish_ocp_results: true
    print_cpu_time: true

  planning:
    objective:
      type: quadratic_form          # 首版更稳定；minimum_time 可作为后续对照

  collision_avoidance:
    include_costmap_obstacles: false # 首版 fixed-path 公平 smoke；导航避障实验另开
    min_obstacle_dist: 0.0

  solver:
    type: ipopt
    ipopt:
      iterations: 50
      max_cpu_time: 0.025           # 需按 30 Hz 实测调整；超时率必须记录
```

move_base 层草案：

```yaml
base_local_planner: mpc_local_planner/MpcLocalPlannerROS
controller_frequency: 30.0
planner_frequency: 0.0
recovery_behavior_enabled: false
clearing_rotation_allowed: false
```

注意：`max_cpu_time=0.025` 只是面向 30 Hz 的初值。若 Ipopt 在该预算下频繁失败，不能悄悄放宽；必须记录为参数变更，并同步解释 compute-cost 公平性。

---

## 9. 正式纳入对比前的 Gate

### Gate 0：配置审计通过

- 参数 dump 中 common limits 正确；
- path/goal/frame/odom/cmd topic 正确；
- `global_plan_overwrite_orientation=false`；
- `allow_init_with_backward_motion=false`；
- 无第二个 `/cmd_vel` 发布者。

### Gate 1：fresh-sim smoke 通过

- fresh sim 启动；
- 同一 spawn；
- 60 s 内至少一轮到达，或若失败，失败原因是 planner 性能而非接线/TF/costmap；
- bag 完整；
- command 未超限。

### Gate 2：重复性通过

- 至少 N=2 pilot；
- 成功/失败模式一致；
- 无随机 TF/costmap 初始化问题；
- solver fail rate 可解释。

### Gate 3：加入论文/报告表格

只有 Gate 0-2 通过后，才把它放入正式表格，并标注为：

> External baseline: generic NMPC local planner under ROS navigation stack.

---

## 10. 后续代码任务建议

1. 新增 Scout fixed-path baseline launch/config；
2. 新增一个 baseline run 脚本，只负责启动/检查/录包，不修改 planner；
3. 新增离线 evaluator，把 SPMPC 和 mpc baseline 都转成同一 CSV；
4. 若 `ocp_result` 不足以提取求解时间，补一个外层 timing logger；
5. 若 move_base path 注入不稳定，再评估薄 wrapper，而不是一开始改 planner 核心。
