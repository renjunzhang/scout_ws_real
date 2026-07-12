# 20260712 mpc-local-planner 实物接入问题记录

## 0. 当前结论

当前实物端 `/home/geist/scout_ws` 里，`mpc_local_planner` **还不能直接跑实物 fixed-path 对比**。

主要原因不是参数本身，而是：

1. 实物端当前没有可用的 `mpc_local_planner` ROS 包；
2. `baseline` runner 里 MPC 默认配置仍偏仿真/诊断口径；
3. 默认 costmap 开了 obstacle layer，不符合当前 no-obstacle fixed-path 主表口径；
4. 用户开发端已有 `mpc-local-planner`，后续应从开发端同步/复用，而不是在实物端临时从 GitHub clone。

因此当前建议：

```text
先暂停实物端 clone/编译；
在开发端确认 mpc_local_planner 可用版本；
再同步到实物端或提供 isolated overlay；
然后补 no-obstacle 公平参数；
最后按 shadow -> actuated smoke -> N=3 顺序执行。
```

---

## 1. 实物端当前发现的问题

### 1.1 ROS 包不可见

实物端检查：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
rospack find mpc_local_planner
```

结果：

```text
[rospack] Error: package 'mpc_local_planner' not found
```

说明当前环境里 `mpc_local_planner` 没有被 `rospack` 发现，`nav_core` 插件也无法加载。

---

### 1.2 源码目录为空/不可用

当前实物端这些目录存在，但内容为空或不可用：

```text
/home/geist/scout_ws/src/mpc_planner
/home/geist/scout_ws/src/scout_apps/control/control_box_rst
/home/geist/scout_ws/src/scout_apps/control/mpc_local_planner
```

这些目录对应 `.gitmodules` 里的 submodule：

```text
src/mpc_planner                         -> https://github.com/tud-amr/mpc_planner.git
src/scout_apps/control/control_box_rst  -> https://github.com/rst-tu-dortmund/control_box_rst.git
src/scout_apps/control/mpc_local_planner -> https://github.com/renjunzhang/scout_ws_real.git
```

但当前实物端没有完整工作树，因此不能直接编译。

---

### 1.3 临时 clone 失败，不建议继续在实物端硬拉

曾尝试初始化 submodule：

```bash
git submodule update --init --recursive \
  src/mpc_planner \
  src/scout_apps/control/control_box_rst \
  src/scout_apps/control/mpc_local_planner
```

失败原因是 GitHub 网络不稳定：

```text
GnuTLS recv error (-110): The TLS connection was non-properly terminated
Failed to connect to github.com port 443: 连接超时
```

本次尝试没有完成 clone，也没有完成编译。

后续不建议在实物端继续临时反复 clone。用户已说明：

```text
我的开发端有 mpc-local-planner
```

因此后续应以开发端已有版本为准。

---

## 2. 当前 runner 对 mpc_local_planner 的配置问题

当前实物一键脚本支持：

```text
METHOD=mpc_local_planner
```

相关脚本：

```text
src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

当前逻辑中 MPC 分支使用：

```text
PLUGIN_TYPE="mpc_local_planner/MpcLocalPlannerROS"
PLUGIN_NAME="MpcLocalPlannerROS"
STATUS_TOPIC="/baseline/mpc_local_planner/status"
TRACKING_DIAGNOSTICS_TOPIC="/baseline/mpc_local_planner/tracking_error"
PLANNER_CONFIG=".../mpc_local_planner_fixed_path_tuned_sim.yaml"
COSTMAP_CONFIG=".../local_costmap_real.yaml"
```

### 2.1 默认 costmap 不公平

当前 MPC 分支默认使用：

```text
src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_real.yaml
```

该 costmap 开启了：

```yaml
obstacle_layer:
  enabled: true

inflation_layer:
  enabled: true
  inflation_radius: 0.5
```

但当前 SPMPC / TEB 主实验口径是 no-obstacle fixed-path：

```text
SPMPC: obstacle_enable=false, corridor_enable=false
TEB: include_costmap_obstacles=false, no-obstacle local costmap
```

因此 MPC 实物主表也应使用：

```text
src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_real_no_obstacles.yaml
```

否则 MPC 会额外受到障碍物 costmap / inflation 影响，不公平。

---

### 2.2 当前 planner config 偏仿真/诊断口径

当前 MPC 默认配置：

```text
src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_tuned_sim.yaml
```

里面仍有：

```yaml
robot:
  unicycle:
    max_vel_x: 0.8
    max_vel_theta: 1.2
    acc_lim_x: 0.6
    dec_lim_x: 0.6
    acc_lim_theta: 1.2

collision_avoidance:
  include_costmap_obstacles: true
```

问题：

1. `max_vel_x=0.8` 是内部规划上限，不适合作为当前实物 fixed-path baseline 主表参数；
2. 外层 runner 即使 clamp `MAX_V`，内部优化仍可能按更高速度规划，形成“内部规划上限”和“外部限幅”不一致；
3. `include_costmap_obstacles=true` 与当前 no-obstacle 对比口径不一致。

---

## 3. 后续公平参数建议

等开发端 `mpc-local-planner` 同步到实物端后，建议新增一个实物 no-obstacle 配置，而不是直接复用 sim tuned 配置。

建议文件名：

```text
src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_real_noobs.yaml
```

建议核心口径：

```yaml
MpcLocalPlannerROS:
  odom_topic: /odom

  robot:
    type: "unicycle"
    unicycle:
      max_vel_x: 0.30
      max_vel_x_backwards: 0.0
      max_vel_theta: 1.20
      acc_lim_x: 0.60
      dec_lim_x: 0.60
      acc_lim_theta: 1.20

  collision_avoidance:
    min_obstacle_dist: 0.0
    enable_dynamic_obstacles: false
    include_costmap_obstacles: false
    costmap_obstacles_behind_robot_dist: 0.0

  controller:
    xy_goal_tolerance: 0.20
    yaw_goal_tolerance: 0.30
    global_plan_overwrite_orientation: true
    allow_init_with_backward_motion: false
```

外层运行建议：

```text
MAX_V=0.30
MAX_W=1.20
MAX_ACC=0.60
MAX_ANGULAR_ACC=1.20
COSTMAP_CONFIG=local_costmap_real_no_obstacles.yaml
PLANNER_CONFIG=mpc_local_planner_fixed_path_real_noobs.yaml
```

---

## 4. 推荐接入顺序

后续不要直接上 actuated。推荐顺序：

```text
1. 在开发端确认 mpc_local_planner 可运行版本；
2. 将源码或 isolated overlay 同步到实物端；
3. 实物端验证：
   rospack find mpc_local_planner
   rospack plugins --attrib=plugin nav_core | grep mpc_local_planner
4. 新增/确认 no-obstacle real config；
5. shadow：只看插件能否加载、路径能否接收、cmd 是否合理；
6. actuated smoke N=1：短跑，确认无异常摆头/急停/NO_VALID_CMD；
7. 通过 gate 后再做正式 N=3。
```

---

## 5. 论文/报告口径

`mpc_local_planner` 应描述为：

```text
external baseline：传统 ROS navigation / nav_core NMPC local planner
```

不能写成：

```text
SPMPC-no-slosh
B0
MPCC-no-slosh
```

原因是它和 SPMPC 的差异不仅是 slosh：

```text
solver、路径参数化、状态/控制量、terminal gate、warm-start、ROS 接口、诊断输出都不同。
```

它适合回答：

```text
与传统 ROS NMPC local planner 相比，SPMPC 在 fixed-path 跟踪、终端稳定性、控制平滑性和晃液抑制上是否更好。
```

不适合单独回答：

```text
slosh cost 本身贡献了多少。
```

slosh cost 贡献仍应由内部消融：

```text
B0 / B_smooth / B_slosh / B_ours
```

来回答。
