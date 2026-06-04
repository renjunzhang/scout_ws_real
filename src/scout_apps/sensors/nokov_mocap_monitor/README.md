# nokov_mocap_monitor

Nokov/XINGYING 动捕监控包，用于 Scout 实物实验中的**外部轨迹监控、rosbag 真值记录、RViz 显示和离线分析**。

> **隔离边界：** 本包只做监控，不参与规划控制闭环。它不替换 `/odom`，不作为任何 planner 的输入，不发布 `/cmd_vel`，默认不发布 TF，也不会发布 `map -> base_link` 或 `odom -> base_link` 这类控制链路 TF。

---

## 1. 数据链路

本包底层接入方式按照 Nokov PDF 中 ROS1 + VRPN 的流程：

```text
XINGYING/Nokov
  -> 开启 VRPN
  -> vrpn_client_ros
  -> /vrpn_client_node/<Tracker>/pose
  -> mocap_pose_monitor.py
  -> /mocap/scout_pose
  -> /mocap/scout_odom
  -> /mocap/scout_path
  -> /mocap/status
```

其中 PDF 原始命令是：

```bash
roslaunch vrpn_client_ros sample.launch server:=10.1.1.198
```

本包的 `nokov_monitor.launch` 对这个流程做了封装，并额外发布隔离的 `/mocap/*` 监控话题，便于 RViz 和 rosbag 记录。

---

## 2. XINGYING/Nokov 端设置

### 2.1 网络

推荐配置示例：

```text
XINGYING/Nokov 主机 IP: 10.1.1.198
工控机有线网口 IP:       10.1.1.196/24
VRPN 端口:                3883（常用默认值，以 XINGYING 实际配置为准）
```

工控机检查：

```bash
ping 10.1.1.198
```

如果 ping 不通，优先检查：

- 两台机器是否在同一网段；
- Windows 防火墙是否放行；
- XINGYING 中 VRPN 绑定的网卡/IP 是否正确；
- 工控机是否使用了正确的有线网口。

### 2.2 刚体名称

建议在 XINGYING 中将 Scout 底盘刚体命名为：

```text
Scout
```

这样 ROS 中原始动捕话题会是：

```text
/vrpn_client_node/Scout/pose
```

如果现场仍用 PDF 示例名，例如 `Tracker2`，启动时把 `tracker` 参数改成 `Tracker2` 即可。

### 2.3 类型和单位

建议：

```text
类型：刚体
单位：米
```

刚体类型会提供位置和姿态；单个 marker 点通常没有完整姿态，不建议作为主真值轨迹源。

---

## 3. 工控机一次性依赖

首次使用前，在工控机安装 VRPN ROS 包：

```bash
sudo apt-get update
sudo apt-get install -y ros-noetic-vrpn-client-ros
```

检查是否安装成功：

```bash
source /opt/ros/noetic/setup.bash
rospack find vrpn_client_ros
```

如果能输出类似下面的路径，说明安装成功：

```text
/opt/ros/noetic/share/vrpn_client_ros
```

---

## 4. 编译本包

在工控机或开发机：

```bash
source /opt/ros/noetic/setup.bash
cd ~/scout_ws
catkin_make --pkg nokov_mocap_monitor
source devel/setup.bash
```

检查本包是否可被 ROS 找到：

```bash
rospack find nokov_mocap_monitor
```

---

## 5. 使用方式

### 5.1 先按 PDF 原始方式测试 VRPN

建议第一次现场调试时，先不启动本包，只验证 PDF 链路。

终端 1：

```bash
source /opt/ros/noetic/setup.bash
roslaunch vrpn_client_ros sample.launch server:=10.1.1.198
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
rostopic list | grep vrpn
rostopic echo -n 1 /vrpn_client_node/Scout/pose
rostopic hz /vrpn_client_node/Scout/pose
```

如果刚体名是 `Tracker2`：

```bash
rostopic echo -n 1 /vrpn_client_node/Tracker2/pose
rostopic hz /vrpn_client_node/Tracker2/pose
```

通过标准：

- 能看到 `/vrpn_client_node/<Tracker>/pose`；
- `position` 数值单位合理，通常应为米；
- 移动 Scout 后位置和姿态连续变化；
- `rostopic hz` 频率稳定。

### 5.2 启动仓库封装监控端

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source ~/scout_ws/devel/setup.bash

roslaunch nokov_mocap_monitor nokov_monitor.launch \
  server:=10.1.1.198 \
  vrpn_port:=3883 \
  tracker:=Scout
```

`3883` 是 VRPN 常用默认端口，也是 Nokov PDF 示例中 `vrpn_client_ros` 的默认端口；它不是协议上“必须固定”的端口。如果 XINGYING/VRPN 广播配置里改过端口，ROS 端必须把 `vrpn_port` 改成同一个值。

如果 XINGYING 刚体名是 `Tracker2`：

```bash
roslaunch nokov_mocap_monitor nokov_monitor.launch \
  server:=10.1.1.198 \
  vrpn_port:=3883 \
  tracker:=Tracker2
```

终端 2 检查：

```bash
source /opt/ros/noetic/setup.bash
source ~/scout_ws/devel/setup.bash

rostopic list | grep -E 'vrpn|mocap'
rostopic echo -n 1 /mocap/scout_pose
rostopic echo -n 1 /mocap/scout_odom
rostopic echo -n 1 /mocap/status
rostopic hz /mocap/scout_pose
```

### 5.3 启动 RViz 监控

可以直接在 launch 中打开 RViz：

```bash
roslaunch nokov_mocap_monitor nokov_monitor.launch \
  server:=10.1.1.198 \
  tracker:=Scout \
  use_rviz:=true
```

默认 RViz fixed frame 是：

```text
mocap_world
```

会显示：

- `/mocap/scout_pose`
- `/mocap/scout_path`
- `/mocap/scout_odom`

### 5.4 只启动监控桥，不启动 VRPN

如果你已经在其他终端手动启动了 PDF 原始命令：

```bash
roslaunch vrpn_client_ros sample.launch server:=10.1.1.198
```

则可以只启动本包的桥接节点：

```bash
roslaunch nokov_mocap_monitor nokov_monitor.launch \
  start_vrpn:=false \
  tracker:=Scout
```

---

## 6. 话题说明

### 6.1 原始 VRPN 话题

```text
/vrpn_client_node/<Tracker>/pose    geometry_msgs/PoseStamped
```

例如：

```text
/vrpn_client_node/Scout/pose
```

### 6.2 本包发布的监控话题

```text
/mocap/scout_pose                  geometry_msgs/PoseStamped
/mocap/scout_odom                  nav_msgs/Odometry，仅用于记录/分析
/mocap/scout_path                  nav_msgs/Path，用于 RViz 轨迹显示
/mocap/status                      std_msgs/String，监控状态
```

默认 frame：

```text
mocap_world
```

默认不发布 TF：

```text
publish_tf=false
```

---

## 7. TF 隔离说明

默认情况下，本包**不发布 TF**。

如果为了 RViz 或调试确实要发布监控 TF，可以显式启用：

```bash
roslaunch nokov_mocap_monitor nokov_monitor.launch \
  server:=10.1.1.198 \
  tracker:=Scout \
  publish_tf:=true
```

此时只允许发布隔离的监控 TF：

```text
mocap_world -> mocap_scout
```

本包会拒绝使用以下控制链路 frame：

```text
map
odom
base_link
base_footprint
```

因此不会发布：

```text
map -> base_link
odom -> base_link
```

---

## 8. 与 SPMPC 的隔离检查

本包只做监控，SPMPC 仍使用原来的 `/odom` 和控制 TF。

SPMPC 启动时不要这样做：

```bash
odom_topic:=/mocap/scout_odom
```

也不要把配置改成：

```yaml
topics:
  odom: /mocap/scout_odom
```

现场可以用下面命令确认隔离：

```bash
rostopic info /odom
rostopic info /cmd_vel
rosnode info /mocap_pose_monitor
rosnode info /spmpc_local_planner
```

期望结果：

- `/mocap_pose_monitor` 只订阅 `/vrpn_client_node/<Tracker>/pose`；
- `/mocap_pose_monitor` 只发布 `/mocap/*`；
- `spmpc_local_planner` 不订阅 `/mocap/scout_odom`；
- Nokov 相关节点不发布 `/odom`；
- Nokov 相关节点不发布 `/cmd_vel`；
- Nokov 相关节点不发布控制 TF。

---

## 9. 录包使用方式

### 9.1 单独录 SPMPC 实验时追加动捕 topic

默认不录动捕。需要录动捕时显式加：

```bash
RECORD_MOCAP=true MOCAP_TRACKER=Scout \
OUT_DIR=/tmp/spmpc_bags \
NAME=spmpc_mocap_smoke \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_experiment.sh
```

会追加记录：

```text
/vrpn_client_node/Scout/pose
/mocap/scout_pose
/mocap/scout_odom
/mocap/scout_path
/mocap/status
```

如果刚体名是 `Tracker2`：

```bash
RECORD_MOCAP=true MOCAP_TRACKER=Tracker2 \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_experiment.sh
```

### 9.2 Phase4 固定路径实验追加动捕 topic

```bash
RECORD_MOCAP=true MOCAP_TRACKER=Scout \
VARIANT=B_ours \
OUT_DIR=/home/geist/slosh_bags/real/20260604_spmpc_mocap \
bash src/scout_apps/control/spmpc_local_planner/scripts/phase4_fixed_path_run.sh
```

metadata 中会写入：

```yaml
record_mocap: true
mocap_tracker: Scout
mocap_raw_pose_topic: /vrpn_client_node/Scout/pose
mocap_monitor_topics:
  pose: /mocap/scout_pose
  odom: /mocap/scout_odom
  path: /mocap/scout_path
  status: /mocap/status
```

检查 bag：

```bash
rosbag info <your_bag>.bag | grep -E 'mocap|vrpn'
```

---

## 10. 诊断脚本

可以用只读诊断脚本快速检查环境：

```bash
rosrun nokov_mocap_monitor check_nokov_env.sh 10.1.1.198 Scout
```

等价环境变量方式：

```bash
NOKOV_SERVER=10.1.1.198 MOCAP_TRACKER=Scout \
rosrun nokov_mocap_monitor check_nokov_env.sh
```

该脚本会检查：

- `vrpn_client_ros` 是否安装；
- 是否能 ping 通 Nokov 主机；
- `/vrpn_client_node/<Tracker>/pose` 是否存在；
- `/mocap/scout_pose` 等监控 topic 是否存在；
- 打印隔离检查提醒。

脚本只读，不会修改网络配置，不会启动控制节点。

---

## 11. 工控机 git pull 后推荐流程

```bash
source /opt/ros/noetic/setup.bash
cd /home/geist/scout_ws

git fetch origin
git checkout experiment/georef-mpc-hybrid
git pull --ff-only
git submodule sync --recursive
git submodule update --init --recursive

catkin_make --pkg nokov_mocap_monitor
source devel/setup.bash
```

启动监控：

```bash
roslaunch nokov_mocap_monitor nokov_monitor.launch \
  server:=10.1.1.198 \
  vrpn_port:=3883 \
  tracker:=Scout
```

启动 SPMPC 时保持原有方式，不接动捕 odom。下面仅为 continuous MPCC 主线示例；正式实物实验以 `run_continuous_real.sh` 和 SOP 为准：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_ours \
  solver_backend:=continuous_mpcc_acados
```

---

## 12. 常见问题

### 12.1 `rospack find vrpn_client_ros` 找不到

说明工控机没有安装 VRPN ROS 包：

```bash
sudo apt-get install -y ros-noetic-vrpn-client-ros
```

### 12.2 看不到 `/vrpn_client_node/Scout/pose`

检查：

- XINGYING 是否开启 VRPN；
- 刚体名是否真的是 `Scout`；
- 如果刚体名是 `Tracker2`，启动时使用 `tracker:=Tracker2`；
- 工控机是否能 ping 通 `10.1.1.198`；
- Windows 防火墙是否放行。

### 12.3 `/mocap/status` 显示 `STALE`

说明最近一段时间没有收到新的 VRPN pose。检查：

```bash
rostopic hz /vrpn_client_node/Scout/pose
```

### 12.4 RViz 看不到轨迹

检查 fixed frame 是否是：

```text
mocap_world
```

检查是否有 path：

```bash
rostopic echo -n 1 /mocap/scout_path
```

---

## 13. 论文/报告表述建议

中文：

```text
Nokov 动捕仅用于外部轨迹监控和离线评估，不参与定位反馈和规划控制输入。
```

英文：

```text
Nokov motion capture was used only for external trajectory monitoring and offline evaluation, not for feedback localization or planner input.
```
