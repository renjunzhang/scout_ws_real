# 20260604 Nokov 动捕监控端接入与实物端 git pull 方案

本文记录将 Nokov/XINGYING 动捕系统接入当前 `experiment/georef-mpc-hybrid` 分支实物实验的推荐方案。**重要边界：动捕只作为监控端/真值记录端，不参与规划器输入，不参与控制闭环，不替换 `/odom`，不发布 `map -> base_link` 控制用 TF。**

目标是：开发机完成代码、launch、记录脚本和 SOP；推送到远端；工控机端通过 `git pull` 拉取后，用统一命令启动动捕监控和 rosbag 记录。SPMPC/底盘控制链路与动捕链路保持隔离，避免动捕配置、丢帧或坐标误差影响实物控制安全。

参考资料：

- `docs/实物实验注意事项/动捕设备使用/ROS与Nokov 动作捕捉系统的通信-V2.pdf`
- `docs/实物实验注意事项/动捕设备使用/Ros2与Nokov的通信.pdf`

当前项目实物实验主线仍是 ROS1 Noetic，因此以下以 **ROS1 + `vrpn_client_ros` + 监控/记录** 为主。

---

## 1. 总原则：动捕与规划控制隔离

本项目中 Nokov 动捕定位不接入 SPMPC 规划器闭环。动捕只用于：

```text
1. 现场监控 Scout 真实轨迹；
2. 录包后离线对比规划轨迹、轮速 odom、实物真值轨迹；
3. 辅助判断路径跟踪误差、漂移、超调、重复性；
4. 与 RGB 液面真值、/spmpc/* 诊断做时间对齐分析。
```

动捕明确不做：

```text
1. 不替换 /odom；
2. 不给 spmpc_local_planner 提供 odom_topic；
3. 不发布 map -> base_link 控制用 TF；
4. 不参与 move_base/MBF/global planner/local planner 的 TF 树；
5. 不影响 /cmd_vel；
6. 不因为动捕断流导致规划器状态变化。
```

推荐隔离数据流：

```text
控制链路：
Scout wheel odom / 原有定位
  -> /odom
  -> odom -> base_link TF
  -> spmpc_local_planner
  -> /cmd_vel
  -> Scout 底盘

监控链路：
Nokov/XINGYING
  -> VRPN
  -> vrpn_client_ros
  -> /vrpn_client_node/Scout/pose
  -> 可选监控桥 /mocap/scout_pose, /mocap/scout_odom, /mocap/scout_path
  -> rosbag / RViz / 离线分析
```

两条链路只在 rosbag 和离线分析阶段汇合，不在实时控制阶段汇合。

---

## 2. PDF 教程与本方案的对应关系

PDF 教程解决的是：**如何让 Nokov/XINGYING 通过 VRPN 把动捕数据发到 ROS。**

本方案完全沿用 PDF 的基础接入方式：

```text
XINGYING/Nokov 开启 VRPN
  -> 工控机安装 ros-noetic-vrpn-client-ros
  -> roslaunch vrpn_client_ros sample.launch server:=10.1.1.198
  -> ROS 中出现 /vrpn_client_node/<TrackerName>/pose
```

但本方案不会继续把这个 pose 接入控制闭环，而是只做监控与记录。

| PDF 步骤 | 本项目使用方式 |
|---|---|
| XINGYING 安装与启动 | 按 PDF 执行 |
| 开启 VRPN | 按 PDF 执行 |
| Windows/Nokov 主机设置 IP，例如 `10.1.1.198` | 按 PDF 执行，IP 作为 launch 参数 |
| 工控机同网段，例如 `10.1.1.196/24` | 按 PDF 执行 |
| `ping 10.1.1.198` | 作为现场 smoke 必做项 |
| 安装 `ros-noetic-vrpn-client-ros` | 工控机一次性系统依赖 |
| `roslaunch vrpn_client_ros sample.launch server:=10.1.1.198` | 由仓库内 `nokov_monitor.launch` 封装 |
| `/vrpn_client_node/Tracker2/pose` | 作为监控/录包 topic，不喂给 planner |
| 单位/反转/偏移 | 优先只影响监控命名空间，不影响控制 TF |

---

## 3. 工控机一次性依赖

工控机首次使用 Nokov 监控前需要安装：

```bash
sudo apt-get update
sudo apt-get install -y ros-noetic-vrpn-client-ros
```

之后每次更新实验逻辑只需要：

```bash
git pull --ff-only
catkin_make
source devel/setup.bash
```

暂不建议把 `vrpn_client_ros` 作为 submodule/vendor 放进主仓库，原因：

- apt 包足够完成监控数据接收；
- 避免和 `/opt/ros/noetic` 中同名包冲突；
- 减少第三方依赖维护成本。

---

## 4. XINGYING/Nokov 侧设置建议

### 4.1 网络

示例配置：

```text
XINGYING/Nokov 主机 IP: 10.1.1.198
工控机有线网口 IP:       10.1.1.196/24
```

工控机检查：

```bash
ping 10.1.1.198
```

若 ping 不通，检查：

```text
1. 两台机器是否同一网段；
2. Windows 防火墙是否放行；
3. XINGYING 中 VRPN 绑定的网卡/IP 是否正确；
4. 工控机是否使用了正确的有线网卡。
```

### 4.2 刚体命名

建议 XINGYING 中将 Scout 底盘刚体命名为：

```text
Scout
```

这样 ROS topic 期望为：

```text
/vrpn_client_node/Scout/pose
```

若现场仍用 PDF 示例里的 `Tracker2`，则 launch 参数中设置：

```bash
tracker:=Tracker2
```

### 4.3 类型与单位

建议：

```text
类型：刚体
单位：米
```

刚体类型可以提供 `position` 和 `orientation`。普通 marker 点通常不具备完整车体姿态，不适合作为轨迹真值主源。

### 4.4 反转与偏移

由于动捕只做监控，反转/偏移不会影响控制安全。但为了离线分析可复现，仍建议：

```text
XINGYING 端尽量少做临时 GUI 修正；
坐标轴变换、offset、yaw offset 尽量记录到仓库 YAML 或 bag metadata。
```

---

## 5. 仓库内建议新增内容

实际落地为独立 sensor package，而不是放入 SPMPC planner/control 包。这样从目录结构上强化“监控-only、真值记录-only”的边界。

```text
src/scout_apps/sensors/nokov_mocap_monitor/
  package.xml
  CMakeLists.txt
  config/
    nokov_monitor.yaml
  launch/
    nokov_monitor.launch
  scripts/
    mocap_pose_monitor.py        # 生成监控用 /mocap/*，不发布控制 TF
    check_nokov_env.sh           # 只读检查网络与 topic
  rviz/
    nokov_monitor.rviz           # 可选 RViz 监控配置
```

核心原则：

```text
1. 所有 Nokov topic 放在 /mocap 或 /vrpn_client_node 命名空间；
2. 不 remap 到 /odom；
3. 不覆盖 /tf 中的 odom -> base_link 或 map -> base_link；
4. 不修改 spmpc_local_planner 的 odom_topic；
5. 只扩展 rosbag topic 列表和 RViz 监控配置。
```

---

## 6. 推荐 topic 设计

VRPN 原始输出：

```text
/vrpn_client_node/Scout/pose            geometry_msgs/PoseStamped
```

可选监控桥输出：

```text
/mocap/scout_pose                       geometry_msgs/PoseStamped
/mocap/scout_odom                       nav_msgs/Odometry，仅监控/分析，不给 planner
/mocap/scout_path                       nav_msgs/Path，RViz 轨迹显示
/mocap/status                           std_msgs/String 或 diagnostic_msgs，可选
```

注意：

```text
/mocap/scout_odom 不能 remap 成 /odom；
/mocap/scout_odom 不能作为 spmpc_local_planner 的 odom_topic；
监控桥默认不发布 TF，或只发布独立命名的监控 TF。
```

如果为了 RViz 方便确实需要 TF，必须使用隔离 frame：

```text
mocap_world -> mocap_scout
```

不要发布：

```text
map -> base_link
odom -> base_link
```

因为这些 frame 属于控制链路。

---

## 7. `nokov_monitor.launch` 设计

推荐 launch 参数：

```xml
<arg name="server" default="10.1.1.198"/>
<arg name="tracker" default="Scout"/>
<arg name="vrpn_port" default="3883"/>
<arg name="start_monitor_bridge" default="true"/>
<arg name="publish_monitor_tf" default="false"/>
<arg name="mocap_world_frame" default="mocap_world"/>
<arg name="mocap_body_frame" default="mocap_scout"/>
```

`3883` 是 VRPN 常用默认端口，也是 PDF 示例中 `vrpn_client_ros sample.launch` 的默认端口；它不是协议上不可改变的固定值。若 XINGYING/VRPN 广播配置里改过端口，启动时用 `vrpn_port:=<端口>` 与 XINGYING 保持一致。

启动内容：

```text
1. 启动 vrpn_client_ros；
2. 接收 /vrpn_client_node/$(arg tracker)/pose；
3. 可选启动 mocap_pose_monitor.py；
4. 发布 /mocap/scout_pose、/mocap/scout_odom、/mocap/scout_path；
5. 默认不发布任何控制链路 TF。
```

示例使用：

```bash
roslaunch nokov_mocap_monitor nokov_monitor.launch \
  server:=10.1.1.198 \
  vrpn_port:=3883 \
  tracker:=Scout
```

若现场刚体名是 `Tracker2`：

```bash
roslaunch nokov_mocap_monitor nokov_monitor.launch \
  server:=10.1.1.198 \
  tracker:=Tracker2
```

---

## 8. SPMPC 启动保持原样

SPMPC 实物实验仍按原有链路启动，不因为动捕监控改变 planner 输入。

例如固定路径实验仍使用原有方式：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_slosh_anti \
  solver_backend:=primitive
```

不要添加：

```bash
odom_topic:=/mocap/scout_odom
```

也不要把：

```yaml
topics:
  odom: /mocap/scout_odom
```

写入 SPMPC 默认配置。

动捕启动失败、丢帧、坐标反向，都只影响监控数据和后处理，不应影响：

```text
/odom
/tf 控制树
spmpc_local_planner
/cmd_vel
```

---

## 9. rosbag 记录建议

正式实物实验录包时，在原有 SPMPC topic 基础上追加动捕监控 topic。

建议至少录：

```text
# 控制链路
/odom
/cmd_vel
/tf
/tf_static
/scout/global_path_fixed
/scout/global_path
/spmpc/status
/spmpc/selected_trajectory
/spmpc/*
/slosh/*

# 动捕监控链路
/vrpn_client_node/Scout/pose
/mocap/scout_pose
/mocap/scout_odom
/mocap/scout_path
/mocap/status

# 可选真值/视频
/camera/color/image_raw
/liquid/*
```

如果现场刚体名不是 `Scout`，应将 bag topic 中的 `Scout` 替换为实际 tracker 名：

```text
/vrpn_client_node/Tracker2/pose
```

录包脚本应允许通过环境变量配置：

```bash
MOCAP_TRACKER=Scout
RECORD_MOCAP=1
```

然后自动追加：

```text
/vrpn_client_node/${MOCAP_TRACKER}/pose
/mocap/scout_pose
/mocap/scout_odom
/mocap/scout_path
```

---

## 10. RViz 监控建议

RViz 中建议显示：

```text
1. 原有控制 TF 和机器人模型；
2. /scout/global_path_fixed；
3. /spmpc/selected_trajectory；
4. /odom 轨迹；
5. /mocap/scout_path 动捕轨迹；
6. /vrpn_client_node/Scout/pose 或 /mocap/scout_pose 的 Pose marker。
```

若不发布动捕 TF，可直接用 Pose/Path 显示，不需要加入控制 TF 树。

如果 RViz fixed frame 是 `map`，而 VRPN 原始 frame 是 `world` 或其他名称，有两种选择：

```text
推荐：监控桥把 pose header.frame_id 统一改成监控用 frame 或 map，仅用于显示和记录；
谨慎：发布静态 TF map -> mocap_world，仅用于 RViz 可视化，不发布到 base_link。
```

无论哪种方式，都不要发布 `map -> base_link` 来覆盖控制链路。

---

## 11. 现场 smoke 流程

### 11.1 只测 PDF 原始链路

```bash
source /opt/ros/noetic/setup.bash
ping 10.1.1.198
roslaunch vrpn_client_ros sample.launch server:=10.1.1.198
```

另开终端：

```bash
source /opt/ros/noetic/setup.bash
rostopic list | grep vrpn
rostopic echo -n 1 /vrpn_client_node/Scout/pose
rostopic hz /vrpn_client_node/Scout/pose
```

通过标准：

```text
能看到 pose；
position 数值单位合理；
移动 Scout 时 position/orientation 有连续变化；
频率稳定。
```

### 11.2 测仓库封装监控 launch

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash

roslaunch nokov_mocap_monitor nokov_monitor.launch \
  server:=10.1.1.198 \
  vrpn_port:=3883 \
  tracker:=Scout
```

另开终端：

```bash
rostopic list | grep -E 'vrpn|mocap'
rostopic hz /mocap/scout_pose
rostopic echo -n 1 /mocap/scout_pose
```

检查 TF 隔离：

```bash
rosrun tf view_frames
```

确认动捕监控节点没有发布控制用：

```text
map -> base_link
odom -> base_link
```

### 11.3 与 SPMPC 同时运行

先启动原有控制链路和 SPMPC，再启动 Nokov 监控。

检查：

```bash
rostopic info /odom
rostopic info /cmd_vel
rostopic info /vrpn_client_node/Scout/pose
rostopic info /mocap/scout_pose
```

通过标准：

```text
/odom publisher 仍是原有定位/底盘链路；
spmpc_local_planner 没有订阅 /mocap/scout_odom；
Nokov monitor 没有发布 /cmd_vel；
关闭 Nokov monitor 后，SPMPC 控制链路不受影响。
```

---

## 12. 工控机 git pull 后推荐流程

### 12.1 首次安装系统依赖

```bash
sudo apt-get update
sudo apt-get install -y ros-noetic-vrpn-client-ros
```

### 12.2 每次同步开发机代码

```bash
source /opt/ros/noetic/setup.bash
cd /home/geist/scout_ws

git fetch origin
git checkout experiment/georef-mpc-hybrid
git pull --ff-only
git submodule sync --recursive
git submodule update --init --recursive

# 只验证动捕监控包时：
catkin_make --pkg nokov_mocap_monitor

# 若同时更新 SPMPC 实验脚本/规划器，可按需构建相关包：
catkin_make --pkg nokov_mocap_monitor spmpc_local_planner spmpc_experiments
source devel/setup.bash
```

### 12.3 启动 Nokov 监控

```bash
roslaunch nokov_mocap_monitor nokov_monitor.launch \
  server:=10.1.1.198 \
  vrpn_port:=3883 \
  tracker:=Scout
```

### 12.4 启动 SPMPC

按原有 SOP 启动，不接动捕 odom：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_slosh_anti \
  solver_backend:=primitive
```

或使用现有 Phase4/record 脚本，只追加动捕 topic 录制，不改变 planner 参数。

---

## 13. 后处理分析建议

动捕作为真值监控后，离线分析可以做：

```text
1. /mocap/scout_path 与 /scout/global_path_fixed 的横向误差；
2. /mocap/scout_odom 与 /odom 的漂移/延迟比较；
3. /cmd_vel 与动捕速度估计的响应关系；
4. /spmpc/selected_trajectory 与动捕实际轨迹的偏差；
5. 动捕轨迹与 RGB 液面真值的时间对齐；
6. 各 planner_variant 的实物轨迹重复性比较。
```

注意：若动捕只作为真值，论文/报告中应表述为：

```text
Nokov motion capture was used only for external trajectory monitoring and offline evaluation, not for feedback localization or planner input.
```

中文表述：

```text
Nokov 动捕仅用于外部轨迹监控和离线评估，不参与定位反馈和规划控制输入。
```

---

## 14. 验收标准

实现后应满足：

1. 工控机 `git pull` 后可以启动：

   ```bash
   roslaunch nokov_mocap_monitor nokov_monitor.launch server:=10.1.1.198 tracker:=Scout
   ```

2. ROS 中有：

   ```text
   /vrpn_client_node/Scout/pose
   /mocap/scout_pose
   /mocap/scout_path
   ```

3. ROS 中不应因为 Nokov 监控新增控制输入：

   ```text
   /odom 不被动捕替换；
   spmpc_local_planner 不订阅 /mocap/scout_odom；
   /cmd_vel 不由 Nokov 相关节点发布；
   控制 TF 树不被 Nokov 节点覆盖。
   ```

4. 关闭 Nokov 监控节点后，SPMPC 与底盘控制仍能按原链路运行。

5. rosbag 中可以同时看到控制链路和动捕监控链路，便于离线对比。

---

## 15. 推荐落地顺序

1. 保持 `spmpc_local_planner` 输入不变，不引入动捕 odom；
2. 在 `src/scout_apps/sensors/nokov_mocap_monitor/` 增加独立监控包；
3. 通过 `nokov_monitor.launch` 按 PDF 封装 `vrpn_client_ros sample.launch`；
4. `mocap_pose_monitor.py` 只发布 `/mocap/*` 监控 topic；
5. 修改 record 脚本，显式 `RECORD_MOCAP=true` 时追加动捕 topic，但不改 planner 参数；
6. 增加 RViz 监控配置和 README；
7. 工控机按 PDF 先验证 VRPN 原始 pose；
8. 再验证仓库封装 launch；
9. 最后与 SPMPC 同时运行，确认隔离。

---

## 16. 当前结论

本方案按 PDF 完成 Nokov/VRPN 到 ROS 的基础接入，但边界明确调整为：

```text
Nokov 动捕 = 监控端 / 真值记录端 / 离线评估端
SPMPC 规划器 = 继续使用原有 /odom 与 TF，不接动捕闭环
```

最终数据关系：

```text
Nokov/XINGYING VRPN
  -> vrpn_client_ros
  -> /vrpn_client_node/Scout/pose
  -> /mocap/* 监控 topic
  -> rosbag / RViz / 离线分析

原有定位/底盘链路
  -> /odom + 控制 TF
  -> spmpc_local_planner
  -> /cmd_vel
```

两条链路实时隔离，只在记录和分析阶段汇合。
