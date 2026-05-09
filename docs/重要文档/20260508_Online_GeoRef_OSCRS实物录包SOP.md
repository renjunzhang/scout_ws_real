# Online GeoRef / OSCRS 实物录包 SOP

日期：2026-05-08

本 SOP 是 `docs/重要文档/20260506有效性验证方案.md` 的执行清单。目标是把 `RAW_REAL / GEOREF_FIXED_STRONG_REAL / GEOREF_TUNED_STRONG_REAL / GEOREF_OSCRS_ACTIVE_REAL` 的录包流程固定下来，避免正式实验时混入起点、目标、液位、相机、MPC 参数或 OSCRS 配置差异。

## 1. 实验条件

同一目标点依次录四条线：

```text
RAW_REAL
GEOREF_FIXED_STRONG_REAL
GEOREF_TUNED_STRONG_REAL
GEOREF_OSCRS_ACTIVE_REAL
```

每条线最低 3 包，推荐 5 包。第一轮只做一个 open goal；第一轮通过后再扩展其他 goal。若时间不足，`GEOREF_FIXED_STRONG_REAL` 可先录 3 包作为 ablation。

## 2. 每包前 Checklist

场地与车辆：

```text
[ ] 起点位置一致，车头 yaw 一致
[ ] 终点附近有足够制动距离
[ ] 路径上无动态障碍
[ ] 定位正常，map -> base_link TF 稳定
[ ] /odom 连续，无明显跳变
[ ] 发目标前车辆静止
```

容器与液位：

```text
[ ] 容器固定方式一致，无松动
[ ] 液体种类一致
[ ] 液位高度记录
[ ] 静止液面基准记录
[ ] 每包之间等待液面基本静止
```

RealSense RGB：

```text
[ ] /camera/color/image_raw 有频率
[ ] 图像能看到容器和液面 ROI
[ ] 标尺 / ArUco / 刻度在画面内
[ ] 相机时间戳正常递增
```

GeoRef / OSCRS：

```text
[ ] GEOREF/ACTIVE 包有 /scout/global_path_anti_slosh
[ ] GEOREF/ACTIVE 包有 /anti_slosh_path/candidate_report
[ ] TUNED 包 summary 中 active=0
[ ] ACTIVE 包 summary 中 active=1
[ ] ACTIVE 包 candidate_report 有 fb / takeover / os / oh / or / ov / osc 字段
[ ] 无 no_costmap / frame_mismatch / collision 异常
```

## 3. 调参规则

原则：

```text
一次只改一类参数；
RAW / GEOREF / OSCRS 三条线的 MPC 参数保持一致；
不得通过重新打开 Q_slosh / speed governor / PROFILE_ENERGY 来救主表；
每次调参必须记录：参数名、旧值、新值、对应 bag、修改原因。
```

主要调参位置：

```text
src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml
  slosh:
    container_radius / liquid_height / damping_ratio / offset_x / offset_y
  oscrs:
    eta_lim_mm
    residual_ratio
    settle_duration
    score/w_h_p95
    score/w_energy_rms
    score/w_eta_dot_rms
    score/w_terminal_E
    score/w_geom

anti_slosh_path_post_processor.launch 启动参数
  ds
  max_candidate_level
  enable_collision_check
  collision_threshold
  ay_ratio_limit
  prediction_v_max
  prediction_ay_max_budget
  prediction_a_max
  mild/mid/medium/strong 的 iters / gain / max_drift
```

推荐顺序：

```text
1. 先调通路，不调效果：
   candidate_report 有消息，ACTIVE 包 active=1，fb/takeover/os/oh/or/ov/osc 字段存在。

2. 先调安全，不调 slosh：
   如果出现 col=no_costmap / frame_mismatch / collision，优先修 costmap_topic、frame、collision_threshold。
   不允许为了通过碰撞检查而关闭 enable_collision_check 录正式实物包。

3. 再调 GeoRef 候选强度：
   路径改得太弱，且 h 不降：提高 max_candidate_level，或增大 strong_gain / strong_iters / strong_max_drift。
   路径贴墙、绕路过多、tracking 变差：降低 max_candidate_level，或减小 gain / max_drift。
   这一步先看 fixed strong 是否可行，再看 geometry-only GEOREF_TUNED_STRONG_REAL，不混入 OSCRS。

   若 candidate_report 显示所有非 original 都因为 `ay:...>1.000` 被拒绝：
   先把 `ay_ratio_limit` 临时放宽到 `3.0` 跑 1 包 takeover smoke。
   这一步只验证 GeoRef/OSCRS 是否能真正发布非 original，不作为正式有效性结果。
   smoke 通过后再根据闭环 `odom_ay_p95` 回收阈值，建议尝试 `2.0 -> 1.5`。

4. 再调 OSCRS hard gate：
   fb=0：OSCRS 选中非 original 且通过 hard gate，通路正常。
   fb=1：只有 original slosh-safe，候选集饱和或非原候选没优势。
   fb=2：有几何候选但 slosh hard gate 失败，检查 oh/or/ov，先确认模型和单位，不直接放宽 eta_lim。
   fb=3：没有可用几何候选，回到 GeoRef 候选生成和碰撞检查。

5. 最后调 OSCRS score：
   想更重视液面峰值：提高 score/w_h_p95。
   想抑制模态能量：提高 score/w_energy_rms。
   想避免 eta_dot 反弹：提高 score/w_eta_dot_rms。
   想压终端残余：提高 score/w_terminal_E。
   想避免过度牺牲几何质量：提高 score/w_geom。
```

调参停止条件：

```text
连续 2 包出现碰撞、人工接管或定位失效：停止调参，先排查安全链路。
OSCRS takeover=1 但 h/eta_dot/energy 明显劣于 GEOREF：回退上一个参数集。
OSCRS takeover 长期为 0：记录为 candidate set saturated，不继续盲调 score。
```

## 4. 通用启动

每个终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
```

基础系统：

```bash
sudo modprobe gs_usb
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
roslaunch scout_bringup scout_mini_robot_base.launch
```

激光雷达：

```bash
roslaunch nanoscan3_bringup nanoscan3_front.launch use_rviz:=false
```

定位：

```bash
roslaunch nanoscan3_localization scout_nanoscan3_cartographer_localization.launch
```

IMU：

```bash
roslaunch scout_bringup scout_imu_with_tf.launch
```

RealSense RGB：

```bash
source /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/realsense_ros_env_local.sh
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

MBF 全局规划：

```bash
roslaunch scout_global_planner mbf_global.launch
```

## 5. RAW_REAL

只启动 MPC，不启动 post-processor：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path \
  Q_slosh:=0 \
  Q_slosh_eta_dot:=0 \
  enable_slosh_box_constraint:=false \
  risk_scheduler_enable:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  slosh_speed_governor_enable:=false \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

## 6. GEOREF_TUNED_STRONG_REAL

终端 1：post-processor。

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  oscrs_config:=/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  ds:=0.03 \
  max_candidate_level:=strong \
  publish_debug:=true \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  ay_ratio_limit:=1.0 \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  prediction_a_max:=1.0 \
  mild_iters:=8 \
  mild_gain:=0.20 \
  mild_max_drift:=0.04 \
  oscrs_shadow_enable:=false \
  oscrs_active_enable:=false
```

终端 2：MPC。

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_anti_slosh \
  Q_slosh:=0 \
  Q_slosh_eta_dot:=0 \
  enable_slosh_box_constraint:=false \
  risk_scheduler_enable:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  slosh_speed_governor_enable:=false \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

## 6A. GEOREF_FIXED_STRONG_REAL

终端 1：fixed strong post-processor。该 baseline 固定发布 `strong` 候选；如果 strong 被 gate 拒绝，则回退 original。它不启用 geometry-only selector，也不启用 OSCRS selector。

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  oscrs_config:=/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  ds:=0.03 \
  max_candidate_level:=strong \
  fixed_candidate_name:=strong \
  publish_debug:=true \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  ay_ratio_limit:=1.0 \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  prediction_a_max:=1.0 \
  mild_iters:=8 \
  mild_gain:=0.20 \
  mild_max_drift:=0.04 \
  oscrs_shadow_enable:=false \
  oscrs_active_enable:=false
```

终端 2：同一 MPC。

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_anti_slosh \
  Q_slosh:=0 \
  Q_slosh_eta_dot:=0 \
  enable_slosh_box_constraint:=false \
  risk_scheduler_enable:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  slosh_speed_governor_enable:=false \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

## 7. GEOREF_OSCRS_ACTIVE_REAL

终端 1：OSCRS active post-processor。

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  oscrs_config:=/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  ds:=0.03 \
  max_candidate_level:=strong \
  publish_debug:=true \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  ay_ratio_limit:=1.0 \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  prediction_a_max:=1.0 \
  mild_iters:=8 \
  mild_gain:=0.20 \
  mild_max_drift:=0.04 \
  oscrs_shadow_enable:=true \
  oscrs_active_enable:=true
```

终端 2：同一 MPC。

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_anti_slosh \
  Q_slosh:=0 \
  Q_slosh_eta_dot:=0 \
  enable_slosh_box_constraint:=false \
  risk_scheduler_enable:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  slosh_speed_governor_enable:=false \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

## 8. 录包

主 bag：

```bash
cd $(rospack find scout_local_planner)
SLOSH_BAG_DIR=/data/$USER/slosh_bags/real/20260508_phase1 \
./scripts/record_slosh_experiment.sh 0 <CONDITION>_<GOAL>_run01
```

如果主 bag 没有 GeoRef/OSCRS 诊断话题，另开诊断 bag：

```bash
mkdir -p /data/$USER/slosh_bags/real/20260508_phase1
rosbag record -O /data/$USER/slosh_bags/real/20260508_phase1/georef_diag_<CONDITION>_<GOAL>_run01 \
  /scout/goal \
  /scout/global_path \
  /scout/global_path_anti_slosh \
  /anti_slosh_path/candidate_report \
  /anti_slosh_path/metrics \
  /anti_slosh_path/safety_alarm \
  /anti_slosh_path/debug/original \
  /anti_slosh_path/debug/mild \
  /anti_slosh_path/debug/mid \
  /anti_slosh_path/debug/medium \
  /anti_slosh_path/debug/strong
```

## 9. 发目标

所有条件使用同一个目标：

```bash
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic /scout/goal \
  --frame map \
  --x <GOAL_X> \
  --y <GOAL_Y> \
  --yaw <GOAL_YAW> \
  --repeat-count 30 \
  --repeat-rate 5 \
  --wait-subscriber-timeout 20
```

发目标前先录 3-5 秒静止段；到达后继续录 2-3 秒 residual。

## 10. ACTIVE smoke

正式三线录包前先跑一包：

```text
GEOREF_OSCRS_ACTIVE_REAL smoke x1
```

通过条件：

```text
/anti_slosh_path/candidate_report 有消息
/scout/global_path_anti_slosh 有消息
summary active=1
fb 字段为 0/1/2/3
候选行有 os / oh / or / ov / osc
takeover 字段为 0 或 1
车辆安全到达或安全停止
```

如果 smoke 不通过，不进入正式录包。

## 11. 每包后检查

基础检查：

```bash
rosbag info /data/$USER/slosh_bags/real/20260508_phase1/<bag>.bag
```

主指标：

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  /data/$USER/slosh_bags/real/20260508_phase1/<bag>.bag
```

OSCRS takeover：

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/check_oscrs_takeover.py \
  /data/$USER/slosh_bags/real/20260508_phase1/<bag>.bag
```

RGB 离线视觉：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/RGB_infer_from_bag.py \
  --bag /data/$USER/slosh_bags/real/20260508_phase1/<bag>.bag \
  --calibration <rgb_calib.yaml> \
  --out-dir /data/$USER/slosh_bags/real/20260508_phase1/rgb_<condition>_<run> \
  --debug-every 30
```

Ferrari 指标：

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/compute_ferrari_indices.py \
  --baseline-bag <RAW_BAG> \
  --optimised-bag <GEOREF_OR_OSCRS_BAG> \
  --baseline-visual-csv <RAW_rgb_heights.csv> \
  --optimised-visual-csv <OPT_rgb_heights.csv> \
  --out-summary <summary.csv>
```

## 12. 无效包判据

```text
未到达目标
碰撞或人工接管
定位明显跳变
solve_success_ratio < 0.97
GEOREF/ACTIVE 缺 /scout/global_path_anti_slosh
GEOREF/ACTIVE 缺 candidate_report
TUNED 包 active != 0
ACTIVE 包 active != 1
RGB 图像看不到液面 ROI，且该包要用于视觉结论
```

## 13. 录包顺序建议

每个 run 按这个顺序录，减少慢变量影响：

```text
RAW_REAL_run01
GEOREF_FIXED_STRONG_REAL_run01
GEOREF_TUNED_STRONG_REAL_run01
GEOREF_OSCRS_ACTIVE_REAL_run01
RAW_REAL_run02
GEOREF_FIXED_STRONG_REAL_run02
GEOREF_TUNED_STRONG_REAL_run02
GEOREF_OSCRS_ACTIVE_REAL_run02
...
```

每包之间等待液面基本静止。
