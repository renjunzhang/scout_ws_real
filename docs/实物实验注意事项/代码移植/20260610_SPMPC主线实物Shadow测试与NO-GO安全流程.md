# 20260610 SPMPC 主线实物测试流程（Shadow / Operator Override）

这份文档给现场开测时照着做。核心原则：**先 B0，再 B_slosh，最后 B_ours**。

当前技术结论仍然是：

```text
SPMPC continuous_mpcc_acados 主线正式实物放行：NO-GO
```

但如果现场安全措施已经到位，可以按本文做两种测试：

```text
1. Shadow 测试：SPMPC 算法运行，但速度发到 /spmpc_shadow_cmd_vel，不驱动车。
2. Operator override 闭环 smoke：现场负责人确认安全后，速度发到 /cmd_vel，短窗口地面闭环验证。
```

## 0. 今天测试什么

测试对象是 `spmpc_local_planner` 的 continuous MPCC/acados 主线：

```text
solver_backend:=continuous_mpcc_acados
alpha_max:=8.0
reference_path_topic:=/scout/global_path_fixed
costmap_topic:=/map
```

测试顺序必须是：

```text
B0 -> B_slosh -> B_ours
```

含义：

```text
B0       : 基础 continuous_mpcc_acados 路径跟踪，不带 slosh。
B_slosh  : slosh 融入 state 和 cost，但不加 smooth priority。
B_ours   : slosh 融入 state 和 cost，并加 smooth priority；这是最终主线方案。
```

如果 `B0` 都出现原地转圈、明显离轨、`TRACKING_UNSAFE_PROJECTION`，就不要继续测 `B_slosh/B_ours`。

## 1. 终端 A：启动实物传感器栈

你已经确认传感器准备好了的话，这一步就是现场已有状态。需要重启时用：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

这个脚本负责底盘、雷达、Cartographer 纯定位、IMU、RealSense。它不是 SPMPC controller。

启动后等定位/TF/map 稳定，建议至少 30s。

## 2. 终端 B：开测前检查

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

rostopic echo -n 1 /odom
rostopic echo -n 1 /map
rostopic hz /odom
rostopic info /cmd_vel
```

如果要录相机真值，再检查：

```bash
rostopic echo -n 1 /camera/color/image_raw
rostopic echo -n 1 /camera/color/camera_info
```

正式启动 planner 前，确认 `/cmd_vel` 没有旧的 `spmpc_local_planner` publisher。

## 3. 终端 C：生成 current-pose fixed path

固定目标点使用今天这组：

```text
x   = 7.164488315582275
y   = 9.307367324829102
yaw = 1.0808
```

启动路径生成器。这个终端保持运行：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=$(date +%Y%m%d)
OUT_DIR=/home/geist/slosh_bags/real/${DATE}_spmpc_mainline_ground
mkdir -p "$OUT_DIR" /home/geist/fixed_paths/real/${DATE}

GOAL_TOPIC=/scout/goal
REF_TOPIC=/scout/global_path_fixed
PATH_FILE=/home/geist/fixed_paths/real/${DATE}/P2_s_curve_spmpc_mainline.json

rosrun scout_local_planner template_fixed_path_generator.py \
  --template s_curve \
  --goal-topic "${GOAL_TOPIC}" \
  --output-topic "${REF_TOPIC}" \
  --path-file "${PATH_FILE}" \
  --start-heading current \
  --spacing 0.05 \
  --amplitude-ratio 0.18 \
  --min-amplitude 0.25 \
  --max-amplitude 1.20 \
  --publish-count 0 \
  >"${OUT_DIR}/path_generator.log" 2>&1
```

另开一个终端发送目标点：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic /scout/goal \
  --x 7.164488315582275 \
  --y 9.307367324829102 \
  --yaw 1.0808 \
  --repeat-count 1 \
  --repeat-rate 1
```

确认路径已经发布：

```bash
rostopic echo -n 1 /scout/global_path_fixed/header
rostopic echo -n 1 /scout/global_path_fixed/poses
```

如果路径起点明显不在车附近，停止，不要启动 SPMPC。

## 4. 终端 D：先开轻量录包

本次不录全量，使用轻量脚本：

```text
src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_mainline_ground_smoke.sh
```

它默认只录关键控制和 `/spmpc/*` 诊断，不录全量相机图像。

### 4.1 第一次建议手动停止录包

这样不用担心还没启动 planner，录包就超时了：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

VARIANT=B0 \
SOLVER_BACKEND=continuous_mpcc_acados \
RECORD_SEC=0 \
OUT_DIR=/home/geist/slosh_bags/real/$(date +%Y%m%d)_spmpc_mainline_ground \
NAME=spmpc_B0_acados_ground_smoke \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_mainline_ground_smoke.sh
```

`RECORD_SEC=0` 表示一直录到你按 Ctrl-C。第一轮 B0 跑 10~20s 就可以停。

### 4.2 后续固定窗口录包

B0 稳定后，后续可以用 20s 自动停止：

```bash
VARIANT=B_slosh \
SOLVER_BACKEND=continuous_mpcc_acados \
RECORD_SEC=20 \
OUT_DIR=/home/geist/slosh_bags/real/$(date +%Y%m%d)_spmpc_mainline_ground \
NAME=spmpc_B_slosh_acados_ground_smoke \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_mainline_ground_smoke.sh
```

```bash
VARIANT=B_ours \
SOLVER_BACKEND=continuous_mpcc_acados \
RECORD_SEC=20 \
OUT_DIR=/home/geist/slosh_bags/real/$(date +%Y%m%d)_spmpc_mainline_ground \
NAME=spmpc_B_ours_acados_ground_smoke \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_mainline_ground_smoke.sh
```

如果确实要录 RGB 原始图像做离线液面真值，才加：

```bash
RECORD_CAMERA=true
```

## 5. 终端 E：启动 SPMPC

### 5.1 Shadow 模式（不驱动车）

如果要先 shadow，看算法输出但不驱动车：

```bash
source /opt/ros/noetic/setup.bash
source ~/.bashrc
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B0 \
  solver_backend:=continuous_mpcc_acados \
  alpha_max:=8.0 \
  reference_path_topic:=/scout/global_path_fixed \
  costmap_topic:=/map \
  cmd_vel_topic:=/spmpc_shadow_cmd_vel \
  publish_cmd_vel:=true
```

检查：

```bash
rostopic info /cmd_vel
rostopic info /spmpc_shadow_cmd_vel
```

期望 `/cmd_vel` 没有 SPMPC publisher。

### 5.2 Operator override 闭环模式（驱动车）

如果现场负责人确认安全措施到位，直接闭环 smoke：

第一轮：`B0`

```bash
source /opt/ros/noetic/setup.bash
source ~/.bashrc
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B0 \
  solver_backend:=continuous_mpcc_acados \
  alpha_max:=8.0 \
  reference_path_topic:=/scout/global_path_fixed \
  costmap_topic:=/map \
  cmd_vel_topic:=/cmd_vel \
  publish_cmd_vel:=true
```

B0 没有明显危险后，再测 `B_slosh`：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_slosh \
  solver_backend:=continuous_mpcc_acados \
  alpha_max:=8.0 \
  reference_path_topic:=/scout/global_path_fixed \
  costmap_topic:=/map \
  cmd_vel_topic:=/cmd_vel \
  publish_cmd_vel:=true
```

最后测 `B_ours`：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_ours \
  solver_backend:=continuous_mpcc_acados \
  alpha_max:=8.0 \
  reference_path_topic:=/scout/global_path_fixed \
  costmap_topic:=/map \
  cmd_vel_topic:=/cmd_vel \
  publish_cmd_vel:=true
```

## 6. 启动后立刻检查

另开终端看：

```bash
rostopic echo -n 1 /spmpc/solver_backend
rostopic echo -n 1 /spmpc/status
rostopic echo -n 1 /spmpc/debug/runtime_bounds
rostopic echo -n 1 /spmpc/start_lock/mode
rostopic echo -n 1 /spmpc/terminal/mode
```

期望：

```text
/spmpc/solver_backend = continuous_mpcc_acados
/spmpc/debug/runtime_bounds 里 alpha_max 约为 8.0
/spmpc/status 不是 ACADOS_NOT_IMPLEMENTED / ACADOS_NOT_CREATED
```

## 7. 立即停机条件

出现任意一个，立即停：

```text
1. 启动后原地明显转圈。
2. 明显没跟上 /scout/global_path_fixed。
3. /spmpc/status = TRACKING_UNSAFE_PROJECTION。
4. /spmpc/status = TRACKING_SPIN_FAIL。
5. /spmpc/status = TERMINAL_SPIN_FAIL。
6. /spmpc/status = ACADOS_NOT_IMPLEMENTED 或 ACADOS_NOT_CREATED。
7. /spmpc/start_lock/mode = UNSAFE_PROJECTION_DISTANCE 持续出现。
8. /cmd_vel 中 |angular.z| > 0.5rad/s 持续约 2s，且 linear.x 很小。
9. 任何人主观认为不安全。
```

停机顺序：

```text
1. 优先硬件 E-stop / 遥控急停。
2. Ctrl-C 停 SPMPC roslaunch。
3. 连续发几次 /cmd_vel zero。
4. Ctrl-C 停 rosbag。
5. 保存 bag、planner log、path JSON，不要删除失败数据。
```

zero 命令：

```bash
for i in 1 2 3 4 5; do
  rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  sleep 0.1
done
```

## 8. 测完后怎么记结论

不要只写“跑了/没撞”。按下面写：

```text
variant: B0 / B_slosh / B_ours
backend: continuous_mpcc_acados
alpha_max: 8.0
bag: <bag path>
是否明显转圈: yes/no
是否明显离轨: yes/no
是否触发 TRACKING_UNSAFE_PROJECTION: yes/no
是否触发 TRACKING_SPIN_FAIL/TERMINAL_SPIN_FAIL: yes/no
progress_s 是否增长: yes/no
是否人工急停: yes/no
```

判断顺序：

```text
B0 不安全 -> 不测 B_slosh/B_ours，先回头修基础 tracking。
B0 安全但 B_slosh 不安全 -> 问题在 slosh state/cost 接入或权重。
B0、B_slosh 安全但 B_ours 不安全 -> 问题可能在 smooth priority 和 slosh 叠加。
三者都安全 -> 再考虑更长时间、更正式的实物实验。
```

本文最后结论仍然是：

```text
当前测试是现场 smoke / operator override，不等于正式实物放行。
正式放行还需要后续仿真 Gate 和实物 bag 复核共同通过。
```
