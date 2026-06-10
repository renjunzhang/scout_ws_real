# 20260611 SPMPC 实物 Smoke：先点到点，再固定路径

这份文档用于下一轮实物车在身边时直接照做。当前不再把现有仿真作为放行依据；现有仿真只保留为开发参考，实物 smoke 以真实 `/odom`、TF、`/map`、`/scan_front`、bag 和人工安全观察为准。

核心顺序：

```text
0. 实物传感器栈 + TF/odom/map 检查
1. 简单点到点 smoke：straight current-pose path，只测 B0
2. 固定路径 smoke：s_curve current-pose path，B0 -> B_slosh -> B_ours
3. 提速诊断：只在默认速度 smoke 安全后，才 opt-in 做 fast_diagnostic
```

结论先写清楚：**需要做一个简单点到点测试**。原因是它比 S 曲线更容易解释：如果 straight 点到点 B0 都不安全，就不要继续测 slosh 或复杂 fixed path；如果 straight 能稳定到点，再进入 S 曲线 smoke。

---

## 0. 安全边界

本流程是实物 smoke / pilot，不是论文正式对比，也不是高速实物放行。

必须满足：

```text
1. 现场有人盯车，随时能硬件 E-stop / 遥控急停。
2. 开测前确认 /cmd_vel 没有旧 planner publisher。
3. 先录包，再启动 planner，保留失败数据。
4. 任何明显转圈、离轨、障碍风险、TF/odom 异常，立即停。
5. 不因为仿真结果好/坏而覆盖实物观察。
6. 不改地图、TF、定位、URDF、雷达、底盘模型来掩盖 planner 问题。
```

---

## 1. 终端 A：启动实物传感器栈

仿照 20260610 流程，实物端使用：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

这个脚本负责底盘、雷达、Cartographer 纯定位、IMU、RealSense。它不是 SPMPC controller。

启动后至少等 30s，让定位、TF、map 稳定。

---

## 2. 终端 B：开测前检查

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

rostopic echo -n 1 /odom
rostopic echo -n 1 /map
rostopic echo -n 1 /scan_front
rostopic hz /odom
rostopic info /cmd_vel
```

期望：

```text
/odom 有速度和 pose；频率稳定。
/map 能收到。
/scan_front 能收到。
/cmd_vel 没有旧的 spmpc_local_planner publisher。
```

如果要录 RGB 真值，再检查：

```bash
rostopic echo -n 1 /camera/color/image_raw
rostopic echo -n 1 /camera/color/camera_info
```

---

## 3. 固定目标点

沿用 20260610 的实物目标点：

```text
x   = 7.164488315582275
y   = 9.307367324829102
yaw = 1.0808
```

统一变量：

```bash
DATE=$(date +%Y%m%d)
OUT_DIR=/home/geist/slosh_bags/real/${DATE}_spmpc_mainline_smoke
PATH_DIR=/home/geist/fixed_paths/real/${DATE}
GOAL_TOPIC=/scout/goal
REF_TOPIC=/scout/global_path_fixed

mkdir -p "$OUT_DIR" "$PATH_DIR"
```

---

## 4. 第一阶段：简单点到点 smoke（必须先做）

这里的“点到点”不要先用复杂 P2P pipeline，而是使用同一套 `template_fixed_path_generator.py` 生成 **straight current-pose path**，再用已验证的 `spmpc_fixed_path.launch` 跑。这样变量最少，和后续 S 曲线 smoke 共用同一 topic、同一录包脚本、同一终点。

### 4.1 终端 C：生成 straight path

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=$(date +%Y%m%d)
OUT_DIR=/home/geist/slosh_bags/real/${DATE}_spmpc_mainline_smoke
PATH_DIR=/home/geist/fixed_paths/real/${DATE}
GOAL_TOPIC=/scout/goal
REF_TOPIC=/scout/global_path_fixed
PATH_FILE=${PATH_DIR}/P0_straight_spmpc_smoke.json

mkdir -p "$OUT_DIR" "$PATH_DIR"

rosrun scout_local_planner template_fixed_path_generator.py \
  --template straight \
  --goal-topic "${GOAL_TOPIC}" \
  --output-topic "${REF_TOPIC}" \
  --path-file "${PATH_FILE}" \
  --start-heading current \
  --spacing 0.05 \
  --publish-count 0 \
  >"${OUT_DIR}/p0_straight_path_generator.log" 2>&1
```

### 4.2 终端 C2：发送同一个固定目标点

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

确认 path 已发布：

```bash
rostopic echo -n 1 /scout/global_path_fixed/header
rostopic echo -n 1 /scout/global_path_fixed/poses
```

人工看 RViz：如果 straight path 穿过障碍、太贴边、起点明显不在车附近，停止，不要启动 planner。

### 4.3 终端 D：先录包，包含 `/scan_front`

`record_spmpc_mainline_ground_smoke.sh` 默认不录 `/scan_front`，这次建议打开，因为实物 smoke 需要保留前雷达证据。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=$(date +%Y%m%d)
OUT_DIR=/home/geist/slosh_bags/real/${DATE}_spmpc_mainline_smoke

VARIANT=B0 \
SOLVER_BACKEND=continuous_mpcc_acados \
RECORD_SEC=0 \
RECORD_SCAN=true \
OUT_DIR=${OUT_DIR} \
NAME=spmpc_P0_straight_B0_default_smoke \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_mainline_ground_smoke.sh
```

`RECORD_SEC=0` 表示手动 Ctrl-C 停录。第一轮建议 10~20s 即可，或到点后马上停。

### 4.4 终端 E：先做 Shadow，不驱动车

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
rostopic echo -n 1 /spmpc_shadow_cmd_vel
rostopic echo -n 1 /spmpc/status
rostopic echo -n 1 /spmpc/debug/runtime_bounds
```

期望：

```text
/cmd_vel 没有 SPMPC publisher。
/spmpc_shadow_cmd_vel 有合理输出。
runtime_bounds 里 alpha_max 约为 8.0。
```

如果 shadow 已经明显输出异常：例如高角速度、几乎零线速度、status 异常，不进入闭环。

### 4.5 终端 E：Operator override 闭环 B0 straight

现场负责人确认安全后才执行：

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

P0 straight B0 通过标准：

```text
1. 没有明显原地转圈。
2. 没有明显离开 straight path。
3. progress_s 持续增长。
4. 没有 TRACKING_UNSAFE_PROJECTION / TRACKING_SPIN_FAIL / TERMINAL_SPIN_FAIL。
5. 可以接近固定目标点并停止，或至少短窗口内行为可解释、可控。
```

如果 P0 straight B0 不通过：停止本轮，不测 B_slosh/B_ours，不测 S 曲线。

---

## 5. 第二阶段：S 曲线固定路径 smoke

只有 P0 straight B0 安全后才进入这一阶段。

### 5.1 终端 C：生成 s_curve path

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=$(date +%Y%m%d)
OUT_DIR=/home/geist/slosh_bags/real/${DATE}_spmpc_mainline_smoke
PATH_DIR=/home/geist/fixed_paths/real/${DATE}
GOAL_TOPIC=/scout/goal
REF_TOPIC=/scout/global_path_fixed
PATH_FILE=${PATH_DIR}/P2_s_curve_spmpc_smoke.json

mkdir -p "$OUT_DIR" "$PATH_DIR"

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
  >"${OUT_DIR}/p2_s_curve_path_generator.log" 2>&1
```

发送同一终点：

```bash
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic /scout/goal \
  --x 7.164488315582275 \
  --y 9.307367324829102 \
  --yaw 1.0808 \
  --repeat-count 1 \
  --repeat-rate 1
```

人工确认 S 曲线路径在场地内且起点贴近当前车位姿。

### 5.2 B0 -> B_slosh -> B_ours

每个 variant 都先开录包，再开 planner。每个 run 建议单独 bag。

#### B0 录包

```bash
VARIANT=B0 \
SOLVER_BACKEND=continuous_mpcc_acados \
RECORD_SEC=0 \
RECORD_SCAN=true \
OUT_DIR=/home/geist/slosh_bags/real/$(date +%Y%m%d)_spmpc_mainline_smoke \
NAME=spmpc_P2_s_curve_B0_default_smoke \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_mainline_ground_smoke.sh
```

#### B0 闭环

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B0 \
  solver_backend:=continuous_mpcc_acados \
  alpha_max:=8.0 \
  reference_path_topic:=/scout/global_path_fixed \
  costmap_topic:=/map \
  cmd_vel_topic:=/cmd_vel \
  publish_cmd_vel:=true
```

B0 安全后再测 B_slosh：

```bash
VARIANT=B_slosh \
SOLVER_BACKEND=continuous_mpcc_acados \
RECORD_SEC=0 \
RECORD_SCAN=true \
OUT_DIR=/home/geist/slosh_bags/real/$(date +%Y%m%d)_spmpc_mainline_smoke \
NAME=spmpc_P2_s_curve_B_slosh_default_smoke \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_mainline_ground_smoke.sh
```

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

B_slosh 安全后再测 B_ours：

```bash
VARIANT=B_ours \
SOLVER_BACKEND=continuous_mpcc_acados \
RECORD_SEC=0 \
RECORD_SCAN=true \
OUT_DIR=/home/geist/slosh_bags/real/$(date +%Y%m%d)_spmpc_mainline_smoke \
NAME=spmpc_P2_s_curve_B_ours_default_smoke \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_mainline_ground_smoke.sh
```

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

---

## 6. 第三阶段：提速 fast_diagnostic（只在默认速度安全后）

不要一上来就把速度翻倍上实物。正确顺序：

```text
默认速度 P0 straight B0 安全
默认速度 P2 s_curve B0/B_slosh/B_ours 安全
离线 bag 指标确认 tracking/core 没有异常
再进入 fast_diagnostic
```

如果实物端已经同步了支持 `v_ref` override 的代码，可以用：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_ours \
  solver_backend:=continuous_mpcc_acados \
  alpha_max:=8.0 \
  v_ref:=0.50 \
  reference_path_topic:=/scout/global_path_fixed \
  costmap_topic:=/map \
  cmd_vel_topic:=/cmd_vel \
  publish_cmd_vel:=true
```

录包命名必须写清楚：

```bash
VARIANT=B_ours \
SOLVER_BACKEND=continuous_mpcc_acados \
RECORD_SEC=0 \
RECORD_SCAN=true \
OUT_DIR=/home/geist/slosh_bags/real/$(date +%Y%m%d)_spmpc_mainline_smoke \
NAME=spmpc_P2_s_curve_B_ours_fastdiag_vref050_smoke \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_mainline_ground_smoke.sh
```

第一档建议 `v_ref:=0.50`，不要直接 `0.65`；如果 `0.50` 的 tracking/core 速度、误差、slosh、安全状态都正常，再考虑 `0.60`。

fast_diagnostic 不能混入 fair-common 主表，只能作为“提速诊断”。

---

## 7. 启动后必须盯的 topic

```bash
rostopic echo -n 1 /spmpc/solver_backend
rostopic echo -n 1 /spmpc/status
rostopic echo -n 1 /spmpc/debug/runtime_bounds
rostopic echo -n 1 /spmpc/debug/progress_s
rostopic echo -n 1 /spmpc/start_lock/mode
rostopic echo -n 1 /spmpc/terminal/mode
rostopic echo -n 1 /cmd_vel
```

期望：

```text
/spmpc/solver_backend = continuous_mpcc_acados
/spmpc/debug/runtime_bounds 里 alpha_max 约为 8.0
/spmpc/status 不是 ACADOS_NOT_IMPLEMENTED / ACADOS_NOT_CREATED
progress_s 随运动增长
/cmd_vel 没有长时间小 linear.x + 大 angular.z 的原地转圈模式
```

---

## 8. 立即停机条件

出现任意一个，立即停：

```text
1. 启动后明显原地转圈。
2. 明显没跟上 /scout/global_path_fixed。
3. /spmpc/status = TRACKING_UNSAFE_PROJECTION。
4. /spmpc/status = TRACKING_SPIN_FAIL。
5. /spmpc/status = TERMINAL_SPIN_FAIL。
6. /spmpc/status = ACADOS_NOT_IMPLEMENTED 或 ACADOS_NOT_CREATED。
7. /spmpc/start_lock/mode = UNSAFE_PROJECTION_DISTANCE 持续出现。
8. /cmd_vel 中 |angular.z| > 0.5rad/s 持续约 2s，且 linear.x 很小。
9. `/scan_front` 或现场观察显示前方有人/障碍/空间不足。
10. 任何人主观认为不安全。
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

---

## 9. 每个 run 的现场记录模板

```text
run_name:
path_type: P0_straight / P2_s_curve
variant: B0 / B_slosh / B_ours
backend: continuous_mpcc_acados
alpha_max: 8.0
v_ref_override: none / 0.50 / 0.60
bag:
path_file:
record_scan: true

是否先 shadow: yes/no
是否明显转圈: yes/no
是否明显离轨: yes/no
是否触发 TRACKING_UNSAFE_PROJECTION: yes/no
是否触发 TRACKING_SPIN_FAIL/TERMINAL_SPIN_FAIL: yes/no
progress_s 是否增长: yes/no
是否到达 terminal REACHED: yes/no
是否人工急停: yes/no
现场主观评价:
```

判断：

```text
P0 straight B0 不安全 -> 不测 S 曲线，不测 slosh，不提速。
P0 straight B0 安全但 P2 B0 不安全 -> 先解决路径复杂度/跟踪问题。
P2 B0 安全但 B_slosh 不安全 -> 问题在 slosh state/cost 或权重。
P2 B0/B_slosh 安全但 B_ours 不安全 -> 问题在 smooth priority 与 slosh 叠加。
P2 三者默认速度都安全 -> 才允许进入 fast_diagnostic。
```

---

## 10. 测后离线分析

把 bag 同步回开发机后，优先用窗口化指标：

```bash
cd /home/a/scout_ws
python3 src/scout_apps/control/spmpc_experiments/scripts/extract_fixed_path_paper_metrics.py \
  /data/a/slosh_bags/real/<date_or_batch_dir> \
  --csv /data/a/slosh_bags/real/<date_or_batch_dir>/spmpc_real_smoke_metrics.csv \
  --phase windows
```

重点看 tracking/core：

```text
cmd_v_mean_mps
odom_v_abs_mean_mps
tracking_error_rms_m
tracking_error_p95_m
heading_error_p95_deg
slosh_height_p95_mm
safety_abort / solver_fail 是否为 0
reached_tail_duration_s 是否被单独剔除
```

实物 smoke 的结论必须基于 bag + 现场观察共同判断，不能只看 `GOAL_REACHED`。
