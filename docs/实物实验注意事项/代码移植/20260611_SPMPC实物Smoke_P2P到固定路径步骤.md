# 20260611 SPMPC/MPC/TEB/DWA 实物 Smoke：先点到点，再固定路径，再做对比

这份文档用于下一轮实物车在身边时直接照做。旧仿真不作为放行依据；新的隔离 Scout strict fresh-sim N=3 只作为“进入实物对比前的仿真证据”。如果 strict N=3 不是全部有效且无 NO-GO，正式实物对比仍保持 NO-GO；实物 smoke 仍以真实 `/odom`、TF、`/map`、`/scan_front`、bag 和人工安全观察为准。

核心顺序：

```text
0. 复核隔离仿真 strict fresh-sim N=3 gate：SPMPC B_ours / mpc_local_planner / TEB / DWA
1. 实物传感器栈 + TF/odom/map/scan 检查
2. 简单点到点 smoke：straight current-pose path，只测 SPMPC B0
3. 固定路径 smoke：s_curve current-pose path，SPMPC B0 -> B_slosh -> B_ours
4. 对比试验：只在 SPMPC smoke 和仿真 gate 都安全后，才逐个 planner 做 SPMPC / mpc_local_planner / TEB / DWA
5. 提速诊断：只在默认速度 smoke 和 common-limit 对比安全后，才 opt-in 做 fast_diagnostic
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
/cmd_vel 没有旧的 planner publisher；做对比时只能有当前这个 planner 发布 `/cmd_vel`。
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

如果本轮要做更完整的 RGB + SPMPC 离线分析，可以改用全量白名单录包脚本；它只录 topic，不启动 planner、不发 goal、不发布 `/cmd_vel`：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=$(date +%Y%m%d)
OUT_DIR=/home/geist/slosh_bags/real/${DATE}_spmpc_mainline_smoke

VARIANT=B0 \
RUN_LABEL=P0_straight \
RECORD_SEC=0 \
RECORD_CAMERA=true \
RECORD_SCAN=true \
RECORD_DEPTH=false \
RECORD_ONLINE_LIQUID=false \
OUT_DIR=${OUT_DIR} \
NAME=spmpc_full_P0_straight_B0_rgb \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

说明：`RECORD_CAMERA=true` 会录 `/camera/color/image_raw` 与 `/camera/color/camera_info`，用于离线 RGB 液面分析；`RECORD_SCAN=true` 会录 `/scan_front`，用于现场障碍/安全证据；`RECORD_DEPTH`、`RECORD_ONLINE_LIQUID`、`RECORD_MOCAP` 默认关闭，需要时再显式打开，避免 bag 过大。

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
planner_family: spmpc / mpc_local_planner / teb / dwa
comparison_mode: smoke / common_limit / tuned_limit / fast_diagnostic
path_type: P0_straight / P2_s_curve
variant_or_plugin: B0 / B_slosh / B_ours / MpcLocalPlannerROS / TebLocalPlannerROS / DWAPlannerROS
backend: continuous_mpcc_acados / nav_core
alpha_or_acc_lim_theta: 8.0 / 1.2
v_ref_override: none / 0.50 / 0.60
limit_profile: default_smoke / common_v0p8_w1p2_a0p6_alpha1p2 / fastdiag
cmd_vel_topic: /spmpc_shadow_cmd_vel / /baseline_<planner>_shadow_cmd_vel / /cmd_vel
status_topic: /spmpc/status / /baseline/mpc_local_planner/status / /baseline/teb/status / /baseline/dwa/status
goal_yaw_mode: explicit / path_end
slosh_eval_only: true/false
bag:
path_file:
record_scan: true

是否先 shadow: yes/no
/cmd_vel 是否只有当前 planner 一个 publisher: yes/no
是否明显转圈: yes/no
是否明显离轨: yes/no
是否触发 TRACKING_UNSAFE_PROJECTION: yes/no/not_applicable
是否触发 TRACKING_SPIN_FAIL/TERMINAL_SPIN_FAIL: yes/no/not_applicable
baseline status 是否 GOAL_REACHED: yes/no/not_applicable
progress_s 是否增长: yes/no/not_applicable
是否到达 terminal REACHED: yes/no/not_applicable
是否人工急停: yes/no
no_go_flags:
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

---

## 11. SPMPC / MPC / TEB / DWA 对比试验口径

### 11.1 仿真 gate 先决条件

最新对比试验以新的隔离 Scout 仿真 strict fresh-sim N=3 为前置证据，矩阵固定为：

```text
SPMPC B_ours / mpc_local_planner / TEB / DWA
fresh sim per case
30s pre-control settle
70s timeout
shutdown + 30s post-shutdown wait
archive under /data/a/scout_sim_replacement
common limit: v=0.8, omega=1.2, a=0.6, alpha/acc_lim_theta=1.2
external slosh source: /slosh/height
established map: /data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
```

本轮必须区分三组证据：

```text
1) 初始四 planner strict batch：
   result_root=/data/a/scout_sim_replacement/results/strict_fresh_fair_n3_20260611_202103
   bag_root=/data/a/scout_sim_replacement/bags/strict_fresh_fair_n3_20260611_202103
   manifest=/data/a/scout_sim_replacement/results/strict_fresh_fair_n3_20260611_202103/strict_fresh_manifest.csv
   aggregate=/data/a/scout_sim_replacement/results/strict_fresh_fair_n3_20260611_202103/strict_fair_metrics_aggregate.json
   plot=/data/a/scout_sim_replacement/results/strict_fresh_fair_n3_20260611_202103/strict_fair_metric_comparison.png

2) 初始 default-map 口径下的 B_slosh 复测：
   result_root=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_B_slosh_n3_20260611_205842
   aggregate=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_B_slosh_n3_20260611_205842/strict_B_slosh_metrics_aggregate.json

3) 显式 established map 口径下的 SPMPC 复测：
   MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
   result_root=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_explicit_map_n3_20260611_211023
   bag_root=/data/a/scout_sim_replacement/bags/strict_fresh_spmpc_explicit_map_n3_20260611_211023
   manifest=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_explicit_map_n3_20260611_211023/strict_fresh_spmpc_explicit_map_manifest.csv
   aggregate=/data/a/scout_sim_replacement/results/strict_fresh_spmpc_explicit_map_n3_20260611_211023/strict_spmpc_explicit_map_metrics_aggregate.json
```

初始四 planner strict batch 的表面结论曾是：MPC/TEB/DWA 3/3 到点，SPMPC `B_ours` 0/3，三次均 `TRACKING_UNSAFE_PROJECTION`。复核后确认关键差异不是 `B_ours` 突然退化，而是 strict 两阶段 SPMPC 环境启动时没有显式传入 established map，日志为：

```text
MAP_FILE=<localization launch default>
```

该默认 map 是 `/home/a/scout_ws/src/scout_apps/scout_maps/maps/map_sim_empty.pbstream`，不是上一轮成功 run 使用的 `/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream`。失败 bags 中 `map->odom` 出现约 `4.47~4.50m` 跳变，而上一轮成功 `fair_peak_20260611_190337` 的 `map->odom` 最大跳变约 `0.0193m`；因此 SPMPC 被安全门 `TRACKING_UNSAFE_PROJECTION` 拦下。

按同一 default-map strict 口径补跑 `B_slosh` 后，`B_slosh` 也 0/3：

| variant | valid strict cases | passes | goal reached | first goal mean | ALL peak mean mm | CORE 10%--90% peak mean mm | observed max v mean m/s | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| SPMPC `B_slosh`（default-map strict） | 3/3 | 0/3 | 0/3 | N/A | 1.749 | 0.437 | 0.497 | 三次均 `TRACKING_UNSAFE_PROJECTION`；说明不是 `B_ours` 特有退化 |

显式使用 established map 后，SPMPC strict 复测通过：

| variant | valid strict cases | passes | goal reached | first goal mean | ALL peak mean mm | CORE 10%--90% peak mean mm | observed max v mean m/s | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| SPMPC `B_ours`（explicit map） | 3/3 | 3/3 | 3/3 | 16.052s | 1.098 | 1.075 | 0.497 | 通过 corrected SPMPC strict 复测 |
| SPMPC `B_slosh`（explicit map） | 3/3 | 3/3 | 3/3 | 15.922s | 1.359 | 1.260 | 0.498 | 通过 corrected SPMPC strict 复测 |

当前 corrected strict 证据的读法：

| planner / variant | evidence batch | valid strict cases | passes | goal reached | first goal mean | ALL peak mean mm | CORE 10%--90% peak mean mm | max v mean m/s | 结论 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| SPMPC `B_ours` | explicit-map SPMPC N=3 | 3/3 | 3/3 | 3/3 | 16.052s | 1.098 | 1.075 | 0.497 | SPMPC strict 口径已复核通过 |
| `mpc_local_planner` | initial four-planner N=3 | 3/3 | 3/3 | 3/3 | 5.916s | 1.553 | 0.859 | 0.800 | 通过 strict batch |
| TEB | initial four-planner N=3 | 3/3 | 3/3 | 3/3 | 6.272s | 3.350 | 3.350 | 0.771 | 通过 strict batch |
| DWA | initial four-planner N=3 | 3/3 | 3/3 | 3/3 | 16.773s | 3.232 | 3.024 | 0.548 | 通过 strict batch |

注意：如果后续要做论文级最终图表，建议在脚本修正后重新跑完整四 planner 同批次 N=3；当前表用于实物 smoke 前复核“SPMPC 突然失败”的原因，并给出 corrected strict evidence。即使 corrected strict evidence 通过，实物也必须从 P0 straight SPMPC B0 smoke 开始，不能直接上四 planner 正式对比。

判定规则：

```text
任一 planner 的 valid strict case 少于 3/3 -> 实物对比 NO-GO。
任一 strict case 触发 NO-GO 或未到点 -> 先分析并复核，不进入正式实物对比。
即使 corrected strict evidence 全部通过，实物也必须从 P0 straight SPMPC B0 smoke 开始，不能直接上四 planner 对比。
```

### 11.2 planner 对比 topic 与入口

| planner | 实物入口 | status / plan topic | `/cmd_vel` 原则 | slosh 原则 |
|---|---|---|---|---|
| SPMPC | `spmpc_local_planner spmpc_fixed_path.launch` | `/spmpc/status`, `/spmpc/debug/*` | smoke 先 shadow，再 operator override 到 `/cmd_vel` | SPMPC variant 可内部使用 slosh；对比表统一另算外部指标 |
| `mpc_local_planner` | `baseline_local_planner_runner nav_core_runner.launch` + `plugin_type:=mpc_local_planner/MpcLocalPlannerROS` | `/baseline/mpc_local_planner/status`, `/baseline/mpc_local_planner/global_plan` | 先发到 `/baseline_mpc_local_planner_shadow_cmd_vel`，闭环时才改 `/cmd_vel` | `/slosh/*` 只读 evaluation-only，不能进控制 |
| TEB | `baseline_local_planner_runner nav_core_runner.launch` + `plugin_type:=teb_local_planner/TebLocalPlannerROS` | `/baseline/teb/status`, `/baseline/teb/global_plan` | 先发到 `/baseline_teb_shadow_cmd_vel`，闭环时才改 `/cmd_vel` | `/slosh/*` 只读 evaluation-only，不能进控制 |
| DWA | `baseline_local_planner_runner nav_core_runner.launch` + `plugin_type:=dwa_local_planner/DWAPlannerROS` | `/baseline/dwa/status`, `/baseline/dwa/global_plan` | 先发到 `/baseline_dwa_shadow_cmd_vel`，闭环时才改 `/cmd_vel` | `/slosh/*` 只读 evaluation-only，不能进控制 |

注意：当前仓库里 TEB/DWA/`mpc_local_planner` 的可复用入口是 generic `baseline_local_planner_runner`；没有已经实车复核过的专用 baseline real launch。实物 baseline 对比前必须准备并现场复核 real costmap yaml（`/scan_front`、`odom`、`base_footprint`、footprint、inflation、obstacle range），不要直接把 sim costmap 当作已验证实物配置。

`mpc_local_planner` 依赖 isolated build；实物端如果沿用同一结构，启动前需要确保类似下面的 overlay 顺序存在：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
# 若实物端也采用 isolated MPC install，必须最后 source，使 pluginlib 找到 libmpc_local_planner
source /home/geist/scout_ws/install_isolated_mpc/setup.bash
```

### 11.3 实物对比执行顺序

```text
A. 先完成第 1~5 节：传感器栈、P0 straight SPMPC B0、P2 s_curve SPMPC B0/B_slosh/B_ours。
B. 如果 A 中任何一步不安全，停止；不测 MPC/TEB/DWA。
C. 准备 baseline real costmap，并对每个 baseline 先做 shadow：cmd_vel_topic 指到 shadow topic，不碰真实 /cmd_vel。
D. shadow 输出正常、/cmd_vel 无旧 publisher、现场确认安全后，才一次只闭环一个 planner。
E. 每个 planner 单独录包、单独记录，不复用上一次 planner 进程；失败数据保留。
F. common-limit 对比表与 fast_diagnostic 提速表分开，不混表。
```

实物对比不是为了证明某个 planner “一定更好”，而是为了在同一路径、同一速度/加速度上限、同一传感器/定位条件、同一外部 slosh 指标口径下观察：是否到点、是否明显离轨、速度是否达到设定、`/slosh/height` ALL/core peak 与 p95、是否触发 NO-GO。
