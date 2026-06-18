# 20260616 SPMPC 点到点 + current global planner 可视化启动 SOP

目的：启动隔离仿真环境，使用当前 `simple_global_planner_node` 生成 `/scout/global_path`，由 `spmpc_local_planner` 执行 point-to-point，并在 RViz/Gazebo 中可视化检查跟踪效果。

> 已人工验证：按本文命令启动后，SPMPC P2P forward goal 跟踪效果良好。
> 记录口径：这是隔离仿真 smoke/可视化流程，不是实车流程。

---

## 0. 安全边界

必须遵守：

- 只使用隔离仿真 `/data/a/scout_sim_replacement`。
- 不启动实车控制。
- 不发布真实机器人 `/cmd_vel`。
- 不修改 map/world/URDF/robot model/Gazebo model/spawn/localization/TF 语义。
- 不删除已有实验数据。
- 不使用 broad `killall` / `pkill`。
- 停止时使用各 terminal 的 `Ctrl-C`，或只停止自己明确启动的进程。

本流程使用独立 master：

```text
ROS_MASTER_URI=http://localhost:11332
GAZEBO_MASTER_URI=http://localhost:11366
```

如果端口被占用，可以换成另一组空闲端口，但同一次测试中所有 terminal 必须保持一致。

---

## 1. Terminal 1：启动隔离仿真 + Cartographer localization

### 1.1 默认 headless Gazebo，稍后手动打开 gzclient

推荐先用这个方式，仿真 server 更稳定：

```bash
export ROS_MASTER_URI=http://localhost:11332
export GAZEBO_MASTER_URI=http://localhost:11366
export SCOUT_PROXY_FULL_ROS_MASTER_URI=http://localhost:11332
export SCOUT_PROXY_FULL_GAZEBO_MASTER_URI=http://localhost:11366

export USE_RVIZ=false
export GAZEBO_GUI=false
export TRACKING_RVIZ=false
export LOCALIZATION_RVIZ=false

export MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
export SPAWN_X=-4.0
export SPAWN_Y=0.0
export SPAWN_Z=0.0
export SPAWN_YAW=0.0

/data/a/scout_sim_replacement/scripts/launch_proxy_sim_localization_env.sh
```

说明：`GAZEBO_GUI=false` 只是不弹 Gazebo 窗口，Gazebo server 仍在运行。

### 1.2 如果希望启动时直接弹 Gazebo GUI

把上面的：

```bash
export GAZEBO_GUI=false
```

改成：

```bash
export GAZEBO_GUI=true
```

其余保持不变。

---

## 2. Terminal 2：等待 map / odom / TF ready

```bash
source /home/a/scout_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11332

rostopic echo -n 1 /map
rostopic echo -n 1 /odom
timeout 3s rosrun tf tf_echo map base_link
```

看到 `/map`、`/odom` 有消息，且 `tf_echo map base_link` 输出 `Translation` 后，再启动 SPMPC P2P。

---

## 3. Terminal 3：启动 current global planner + SPMPC P2P

```bash
source /home/a/scout_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11332
export GAZEBO_MASTER_URI=http://localhost:11366

roslaunch spmpc_experiments run_spmpc_p2p_live_global_sim.launch \
  planner_variant:=B0 \
  solver_backend:=continuous_mpcc_acados \
  reference_target_frame:=map
```

该 launch 会启动：

```text
/scout/simple_global_planner
/spmpc_local_planner
```

它使用当前 `simple_global_planner_node`，不是旧的 `mbf_global_sim.launch`。

关键 topic：

```text
/scout/goal          输入 goal
/scout/global_path   current global planner 输出路径
/spmpc/local_trajectory  SPMPC 局部预测轨迹
/cmd_vel             SPMPC 仿真命令输出
/spmpc/status        SPMPC 状态
```

---

## 4. Terminal 4：打开 RViz

```bash
source /home/a/scout_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11332

rviz -d /data/a/scout_sim_replacement/classic_ws/src/scout_mini_proxy_nav_adapter/rviz/proxy_tracking.rviz
```

如果 RViz 没自动显示 P2P 相关内容，手动添加：

```text
Fixed Frame: map

Add -> By topic:
  /map                      Map
  /tf                       TF
  /scan_front               LaserScan
  /scout/global_path        Path
  /spmpc/local_trajectory   Path
  /scout/goal               Pose
```

建议颜色：

```text
/scout/global_path        绿色或蓝色
/spmpc/local_trajectory   红色或黄色
```

---

## 5. Terminal 5：打开 Gazebo 可视化窗口

如果 Terminal 1 使用了 `GAZEBO_GUI=false`，可以另开 terminal 只启动 Gazebo client：

```bash
export GAZEBO_MASTER_URI=http://localhost:11366
gzclient --verbose
```

关闭 Gazebo 窗口或退出 `gzclient` 不会等价于关闭整个仿真；Gazebo server / ROS 节点仍可能在其他 terminal 中运行。

---

## 6. Terminal 6：发送 P2P goal

推荐先发 forward goal：

```bash
source /home/a/scout_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11332

rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic /scout/goal \
  --frame map \
  --x 3.0 --y 0.0 --yaw 0.0 \
  --repeat-count 3 --repeat-rate 2
```

预期现象：

1. RViz 中 `/scout/global_path` 从当前机器人附近指向 `x≈3.0`；
2. RViz 中 `/spmpc/local_trajectory` 在机器人前方滚动更新；
3. Gazebo 中机器人沿路径前进；
4. `/spmpc/status` 从 `B0_ACADOS_OK` 进入 terminal 阶段，最后 `GOAL_REACHED`。

---

## 7. 常用观察命令

### 7.1 看 SPMPC 状态

```bash
source /home/a/scout_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11332

rostopic echo /spmpc/status
```

### 7.2 看速度命令

```bash
source /home/a/scout_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11332

rostopic echo /cmd_vel
```

### 7.3 检查 current global planner topic 链路

```bash
source /home/a/scout_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11332

rostopic info /scout/global_path
```

应看到：

```text
Publishers:
 * /scout/simple_global_planner

Subscribers:
 * /spmpc_local_planner
```

### 7.4 看当前 global path

```bash
rostopic echo -n 1 /scout/global_path
```

### 7.5 看 SPMPC local trajectory

```bash
rostopic echo -n 1 /spmpc/local_trajectory
```

---

## 8. 停止顺序

不要用 `killall` / `pkill`。

推荐停止顺序：

1. 停 `/spmpc/status`、`/cmd_vel` 等 echo terminal：`Ctrl-C`；
2. 关闭 RViz：关窗口或 `Ctrl-C`；
3. 关闭 Gazebo client：关窗口或 `Ctrl-C`；
4. 停 Terminal 3 的 SPMPC + current global planner：`Ctrl-C`；
5. 最后停 Terminal 1 的 isolated sim + localization：`Ctrl-C`。

注意：只退出 Gazebo GUI / `gzclient`，通常不会停止 ROS master、Gazebo server、Cartographer、SPMPC 或 global planner。

---

## 9. 已知注意事项

1. 如果看不见 Gazebo，是因为 `GAZEBO_GUI=false`，可以另开 terminal 运行：

   ```bash
   export GAZEBO_MASTER_URI=http://localhost:11366
   gzclient --verbose
   ```

2. 如果 RViz 里 `/spmpc/local_trajectory` 和 `/scout/global_path` 显示方向不一致，先检查两者 `header.frame_id`。本 SOP 使用：

   ```text
   reference_target_frame:=map
   ```

   正常情况下两者应在 `map` frame 下显示。

3. 不建议用反向/behind-goal 作为第一条 P2P smoke。当前 terminal controller 对 behind-goal capture-stop 有硬零命令语义；forward goal 已确认 clean pass。

4. 若要做 formal 对比，每个 case 应 fresh sim，不要在同一个仿真中连续复用多个 goal 后直接写正式结论。
