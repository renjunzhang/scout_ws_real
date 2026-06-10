# 20260610 SPMPC 主线实物 Shadow 测试与 NO-GO 安全流程

本文回答一个现场问题：**会不会是仿真不准，所以应该直接在实物上跑 SPMPC？**

结论先写清楚：

```text
当前 continuous_mpcc_acados + B0 + alpha_max=8.0 主线仍是实物前 NO-GO。
不建议、也不把本文作为授权，在地面上直接让 SPMPC 发布 /cmd_vel 闭环跑车。
```

可以做的是：在实物传感器栈上启动 SPMPC **shadow / dry-run**，让它读取真实 `/odom`、`/map`、TF 和实物当前位姿生成路径，但把控制命令发到不接底盘的 shadow topic，或完全不发布控制命令。这样可以判断“实物定位/路径/solver 启动口径是否正常”，但不让车动。

原因：最近 strict fresh-sim/current-pose Gate 已经暴露了两个不是“仿真精度”能解释掉的问题：

1. 旧 terminal 判定曾把 projected `remaining_s` 当成真实 `distance_to_goal`，会 false PASS；已修。
2. 修完 terminal 后，机器人仍出现没有跟上轨迹、启动阶段转圈、unsafe projection；已加 `tracking_safety` hard-zero 门控，但底层 tracking/path-feasibility 仍未修好。

所以实物可以用于 **只读验证真实传感器与启动链路**，不能用来绕过仿真 Gate。

## 1. 现场红线

### 1.1 禁止项

当前阶段不要做：

```text
1. 不要直接运行会把 SPMPC 接到 /cmd_vel 的地面闭环实验。
2. 不要用 run_continuous_real.sh 作为第一步实物 smoke；它默认会启动 spmpc_fixed_path.launch 并发布到 /cmd_vel。
3. 不要通过移动/重置/重生成仿真或实物环境来掩盖 tracking 问题。
4. 不要临场修改 world/map/URDF/TF/Cartographer 口径后把结果当作主线通过。
5. 不要在没有 E-stop、遥控接管、空旷区域和旁站人员时试任何闭环运动。
```

### 1.2 本文允许的测试

本文只允许三类：

```text
A. real sensors stack preflight：只确认 /odom、/map、TF、相机、雷达、底盘节点在。
B. SPMPC shadow：SPMPC 正常求解，但命令发到 /spmpc_shadow_cmd_vel，不接底盘。
C. SPMPC no-publish：SPMPC 正常求解，但 publish_cmd_vel=false，不发布 Twist。
```

A/B/C 都不能作为“实物闭环通过”。它们最多证明：实物端代码、acados、topic、frame、路径生成和诊断链没有明显启动错误。

## 2. 终端 A：启动实物传感器栈

按你给的方式启动：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

这个脚本的职责是底盘、雷达、Cartographer 纯定位、IMU、RealSense；它不是 SPMPC 控制器启动脚本。

启动后不要马上跑 SPMPC，先等定位和 TF 稳定，建议至少 30s。

## 3. 终端 B：传感器与控制口 preflight

另开终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws
```

检查 ROS master 和关键话题：

```bash
rostopic list
rostopic echo -n 1 /odom
rostopic echo -n 1 /map
rostopic echo -n 1 /camera/color/image_raw
rostopic echo -n 1 /camera/color/camera_info
rostopic hz /odom
```

检查 `/cmd_vel` 当前发布者。shadow 测试前，**不应该有 SPMPC 发布到 `/cmd_vel`**：

```bash
rostopic info /cmd_vel
```

如果这里已经看到 `/spmpc_local_planner` 是 `/cmd_vel` publisher，说明你不是 shadow 模式，立即停掉对应 roslaunch。

可选：发一次零速度，确认现场应急命令可用：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 4. 终端 C：生成 current-pose fixed path

本步骤只发布参考路径，不发控制命令。

先设置当天目录和目标点。`GOAL_X/GOAL_Y/GOAL_YAW` 必须是当天固定目标点，不要临时为了让曲线容易而换目标。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=$(date +%Y%m%d)
OUT_DIR=/home/geist/slosh_bags/real/${DATE}_spmpc_shadow
mkdir -p "$OUT_DIR" /home/geist/fixed_paths/real/${DATE}

GOAL_X=7.164488315582275
GOAL_Y=9.307367324829102
GOAL_YAW=1.0808
REF_TOPIC=/scout/global_path_fixed
GOAL_TOPIC=/scout/goal
PATH_FILE=/home/geist/fixed_paths/real/${DATE}/P2_s_curve_shadow.json
```

启动模板路径生成器：

```bash
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

另开同一终端或新终端发送本次固定目标点：

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
ls -lh "${PATH_FILE}"
```

如果路径起点明显不在车附近，停止，不要启动 SPMPC。

## 5. 终端 D：启动 SPMPC shadow，不接底盘

推荐使用 **shadow command topic**，这样能录到 SPMPC 想发的命令，但不会发到 `/cmd_vel`：

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

安全核对：

```bash
rostopic info /cmd_vel
rostopic info /spmpc_shadow_cmd_vel
rostopic echo -n 1 /spmpc/solver_backend
rostopic echo -n 1 /spmpc/status
rostopic echo -n 1 /spmpc/solver_time_ms
rostopic echo -n 1 /spmpc/debug/runtime_bounds
rostopic echo -n 1 /spmpc/debug/generated_bounds
```

期望：

```text
/spmpc/solver_backend = continuous_mpcc_acados
/spmpc/debug/runtime_bounds 中 alpha_max 约为 8.0
/spmpc_shadow_cmd_vel 有 SPMPC publisher
/cmd_vel 没有 SPMPC publisher
```

如果想更保守，完全不发布 Twist，用 no-publish 模式：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B0 \
  solver_backend:=continuous_mpcc_acados \
  alpha_max:=8.0 \
  reference_path_topic:=/scout/global_path_fixed \
  costmap_topic:=/map \
  publish_cmd_vel:=false
```

no-publish 模式下不会有 `/spmpc_shadow_cmd_vel`，主要看 `/spmpc/status`、debug topics 和 solver time。

## 6. 终端 E：录 shadow bag

建议 shadow 先录 20~30s，不需要车动。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=$(date +%Y%m%d)
OUT_DIR=/home/geist/slosh_bags/real/${DATE}_spmpc_shadow
mkdir -p "$OUT_DIR"

rosbag record -O "${OUT_DIR}/spmpc_B0_alpha8_shadow.bag" \
  /cmd_vel \
  /spmpc_shadow_cmd_vel \
  /odom \
  /map \
  /tf \
  /tf_static \
  /imu/data \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /scout/global_path_fixed \
  /spmpc/status \
  /spmpc/solver_backend \
  /spmpc/solver_time_ms \
  /spmpc/cost_breakdown \
  /spmpc/debug/progress_s \
  /spmpc/debug/runtime_bounds \
  /spmpc/debug/generated_bounds \
  /spmpc/debug/first_shot_summary \
  /spmpc/debug/projector \
  /spmpc/debug/stage0_reference \
  /spmpc/debug/local_traj_head \
  /spmpc/debug/warm_start_head \
  /spmpc/start_lock/active \
  /spmpc/start_lock/mode \
  /spmpc/start_lock/debug \
  /spmpc/terminal/mode \
  /spmpc/terminal/debug
```

停止时按 Ctrl-C 停 rosbag，再 Ctrl-C 停 SPMPC launch，再按需要停 path generator。不要用宽泛 `killall` / `pkill` 清场。

## 7. Shadow bag 的判据

shadow 不动车，所以它不能证明“闭环能到点”。它只回答：真实传感器条件下，SPMPC 是否启动正常、是否一上来就给出危险形态。

### 7.1 shadow 通过的最低条件

至少满足：

```text
1. /spmpc/solver_backend 是 continuous_mpcc_acados。
2. runtime_bounds.alpha_max 约为 8.0。
3. 没有 ACADOS_NOT_IMPLEMENTED、ACADOS_NOT_CREATED。
4. /cmd_vel 没有 SPMPC publisher；如果用了 shadow topic，命令只在 /spmpc_shadow_cmd_vel。
5. 没有 TERMINAL_SPIN_FAIL、TRACKING_SPIN_FAIL、TRACKING_UNSAFE_PROJECTION 连续出现。
6. /spmpc/start_lock/mode、/spmpc/debug/projector、/spmpc/terminal/debug 都被录到。
7. shadow command 没有一启动就长时间 |omega| 接近 1.2rad/s，同时 v 很低。
```

### 7.2 shadow 失败就必须停止

出现下面任意一种，不要进地面闭环：

```text
/spmpc/status = ACADOS_NOT_IMPLEMENTED
/spmpc/status = ACADOS_NOT_CREATED
/spmpc/status = TRACKING_UNSAFE_PROJECTION
/spmpc/status = TRACKING_SPIN_FAIL
/spmpc/status = TERMINAL_SPIN_FAIL
/spmpc/start_lock/mode = UNSAFE_PROJECTION_DISTANCE 持续出现
/spmpc_shadow_cmd_vel 中 |angular.z| > 0.5rad/s 持续 2s 以上且 linear.x 很小
runtime_bounds.alpha_max 不是 8.0 口径
/scout/global_path_fixed 起点明显不在当前车附近
```

这些现象和仿真中已经看到的问题一致，说明不是“仿真不准就可以实物跑”的情况。

## 8. 不建议现在用 run_continuous_real.sh 直接跑

`src/scout_apps/control/spmpc_local_planner/scripts/run_continuous_real.sh` 的定位是实物连续 MPCC 单组运行 + 录包。它会：

```text
1. 生成 current-pose fixed path。
2. 启动 spmpc_fixed_path.launch。
3. 默认让 SPMPC 发布到 /cmd_vel。
4. 录一段 real bag。
```

在当前 NO-GO 阶段，不要把它作为第一步实物测试。等满足以下条件后，才考虑把它作为正式实物脚本：

```text
1. fresh-sim/current-pose strict Gate 不再触发 TRACKING_UNSAFE_PROJECTION。
2. bag 复核确认不再启动阶段转圈、不再跑离路径。
3. shadow bag 确认实物端 solver/topic/frame/path 正常。
4. 现场有 E-stop、遥控接管、空旷区域和旁站人员。
5. 先从 B0 空载/无液体/极短窗口开始，而不是直接装液体跑完整组。
```

## 9. 如果现场坚持继续推进，推荐顺序

推荐顺序是：

```text
Step 0: 只启动 real sensors stack，确认 /odom /map /tf /camera 正常。
Step 1: SPMPC no-publish，确认 acados 后端、runtime bounds、status、solver time。
Step 2: SPMPC shadow topic，录 /spmpc_shadow_cmd_vel 和所有 debug topics。
Step 3: 离线复核 shadow bag。
Step 4: 回到仿真修 tracking/path-feasibility 根因。
Step 5: 仿真 strict Gate 通过后，再写单独的地面闭环实物 SOP。
```

当前不要跳到 Step 5。

## 10. 现场最小命令清单

### 10.1 传感器栈

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws
src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

### 10.2 Shadow SPMPC

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

### 10.3 确认没有接到底盘

```bash
rostopic info /cmd_vel
rostopic info /spmpc_shadow_cmd_vel
```

只有确认 `/cmd_vel` 没有 SPMPC publisher，才继续录 bag。

## 11. 本文结论

实物测试不是不能做，但现在只能做 shadow/dry-run 证据收集，不能把当前 SPMPC 主线直接放到地面闭环。

如果 shadow 也复现高转向、unsafe projection、start-lock 或 acados failure，那就进一步证明问题在 planner/path/projection/OCP 链路，不是 Gazebo 精度导致。

如果 shadow 完全干净，也只能说明实物启动链路正常；还需要回到仿真把 current-pose strict Gate 跑通，再进入真实闭环。正式结论仍是：

```text
SPMPC continuous_mpcc_acados 主线实物闭环：NO-GO
SPMPC 实物 shadow/dry-run：允许，用于诊断，不用于放行
```
