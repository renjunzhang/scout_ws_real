# baseline_local_planner_runner

`baseline_local_planner_runner` 是外部 `nav_core::BaseLocalPlanner`
baseline 的独立执行器。它不实现控制算法，只负责把插件式 local planner
从 `move_base` 中拿出来单独运行。

## 输入输出

输入：

```text
/scout/global_path_fixed  nav_msgs/Path，可选
/scout/goal               geometry_msgs/PoseStamped
/odom                     nav_msgs/Odometry，由插件内部读取
/map, /scan_front, /tf    costmap / TF 输入
```

输出：

```text
/cmd_vel
/baseline/<name>/status
/baseline/<name>/global_plan
```

如果只收到 goal、没有 path，runner 会按当前位姿到目标点生成一条直线路径。
正式 fixed-path 对比应显式发布同一条 `/scout/global_path_fixed`。

## 支持的插件

```text
teb_local_planner/TebLocalPlannerROS
dwa_local_planner/DWAPlannerROS
mpc_local_planner/MpcLocalPlannerROS
```

其中 `mpc_local_planner` 已 clone 到 `src/scout_apps/control`，其依赖
`control_box_rst/corbo` 需要通过 `catkin_make_isolated` 构建。运行 MPC
baseline 前需要先 source：

```bash
source /home/a/scout_ws/install_isolated_mpc/setup.bash
source /home/a/scout_ws/devel/setup.bash
```

`TEB/DWA/MPC` 三者在 runner 层使用相同输入输出接口；差别只在加载的
`nav_core::BaseLocalPlanner` 插件和对应参数文件。
