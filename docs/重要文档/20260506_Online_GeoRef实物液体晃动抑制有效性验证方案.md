# Online GeoRef 实物液体晃动抑制有效性验证方案

日期：2026-05-06

## 1. 验证目标

本方案只验证一件事：

```text
在 /slosh/height 作为液体晃动观测量、并尽量用视觉液面作真值交叉检查的前提下，
Online GeoRef 是否比原始 MBF 全局路径降低液体晃动。
```

当前要验证的是新的 reference-first 路线：

```text
MBF 全局路径
  -> anti_slosh_path_post_processor 几何后处理
  -> /scout/global_path_anti_slosh
  -> 普通 MPC 跟踪
  -> /cmd_vel
```

不是旧路线：

```text
MPC 内部 Q_slosh 软代价
OUTPUT_GUARD / PMG / cmd_vel 后处理
PathHandler 内 PROFILE_ENERGY 速度剖面
```

所以实物验证时，MPC 内部防晃项必须关闭：

```text
Q_slosh=0
Q_slosh_eta_dot=0
enable_slosh_box_constraint=false
risk_scheduler_enable=false
energy_profile_enable=false
input_shaping_enable=false
slosh_speed_governor_enable=false
```

## 2. 成功标准

每个 condition 至少录 3 包。正式结论看同起点、同终点、同地图、同定位、同 MPC 参数下的 3 包均值。

GEOREF 相对 RAW 需要同时满足：

```text
h_rms             下降
h_p95             下降，目标 >= 10%
modal_energy_rms  下降，目标 >= 10%
eta_dot_rms       不上升，最好下降
tracking_time     不超过 RAW +15%
ay_p95            不上升
solve_success     >= 0.97
无碰撞、无人工接管、无明显定位丢失
```

如果启动了 RealSense 液面视觉，则额外要求：

```text
视觉液面峰值 / p95 与 /slosh/height 改善方向一致。
如果 /slosh/height 下降但视觉真液面不降，不能声明实物液体晃动被抑制。
```

## 3. 对比组

第一阶段只做最小闭环，不要一次混入太多变量。

```text
RAW_REAL
  /scout/global_path -> MPC
  不启动 post-processor
  Q_slosh=0

GEOREF_REAL
  /scout/global_path -> post-processor -> /scout/global_path_anti_slosh -> MPC
  Q_slosh=0

ORIGINAL_REAL，可选
  启动 post-processor，但 max_candidate_level=original
  用来确认 topic chain / original fallback 本身不产生收益

SLOW_REAL，可选
  原始路径 + 匹配 GEOREF 时间的低速 baseline
  只在 RAW/GEOREF 初步有效后再做
```

论文主表至少需要 RAW_REAL 和 GEOREF_REAL。ORIGINAL_REAL / SLOW_REAL 用于排除“只是换 topic”或“只是慢了”。

## 4. 场地要求

第一轮实物不要在窄走廊或迷宫环境做。

推荐条件：

```text
开阔平整场地
无行人和动态障碍
有足够转弯余量
起点和终点之间至少包含一个转弯或 S 型段
路径长度足够让液体被激励，建议 >= 4 m
终点附近留足制动距离
```

停止条件：

```text
RAW 本身跟踪不住或接近碰撞 -> 停止，这不是 anti-slosh 验证
GEOREF 选中 original -> 该包不能算作 GeoRef 有效样本
GEOREF 明显贴墙、绕路过大或定位漂移 -> 停止，先处理导航可行性
连续 2 包 h_p95 或 eta_dot 明显变差 -> 停止，不继续盲目录包
```

## 5. 实物启动顺序

每个终端都先加载工作区：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
```

如果工控机用户名不是 `a`，把 `/home/a/scout_ws` 替换成实物机器实际路径。

### 5.1 CAN 和底盘

```bash
sudo modprobe gs_usb
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
roslaunch scout_bringup scout_mini_robot_base.launch
```

可选状态监听：

```bash
roslaunch scout_bringup bms_status_monitor.launch topic:=/BMS_status period:=60.0
```

### 5.2 激光雷达

```bash
roslaunch nanoscan3_bringup nanoscan3_front.launch use_rviz:=false
```

检查：

```bash
rostopic hz /scan_front
```

### 5.3 定位

二选一，优先使用当前实物地图对应的定位方案。

```bash
roslaunch nanoscan3_localization scout_nanoscan3_amcl.launch use_rviz:=true
```

或：

```bash
roslaunch nanoscan3_localization scout_nanoscan3_cartographer_localization.launch
```

检查 TF：

```bash
rosrun tf tf_echo map base_link
```

### 5.4 IMU

做 anti-slosh 实物验证时建议启动 IMU，但第一轮局部规划仍不把 IMU 横向加速度作为唯一真源。

```bash
roslaunch scout_bringup scout_imu_with_tf.launch
```

检查：

```bash
rostopic echo -n1 /imu/data
rostopic hz /imu/data
```

### 5.5 RealSense 液面视觉，可选但强烈建议

如果要声明“真实液面晃动被抑制”，建议同步启动 RealSense 和液面检测链路。

```bash
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash
source /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/realsense_ros_env_local.sh
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

检查：

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/depth/image_rect_raw
```

如果实物机器路径是 `/home/geist/scout_ws`，按 `docs/重要文档/change_log.md` 中 RealSense 流程替换路径。

### 5.6 MBF 全局规划

```bash
roslaunch scout_global_planner mbf_global.launch
```

检查：

```bash
rostopic list | grep global_path
rostopic echo -n1 /scout/global_path
```

如果还没有发目标，`/scout/global_path` 没消息是正常的。

## 6. RAW_REAL 启动

RAW_REAL 不启动 `anti_slosh_path_post_processor.launch`。

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

## 7. GEOREF_REAL 启动

先启动在线 path post-processor：

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  ds:=0.03 \
  max_candidate_level:=medium \
  publish_debug:=true \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  ay_ratio_limit:=1.0 \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  prediction_a_max:=1.0 \
  mild_iters:=18 \
  mild_gain:=0.35 \
  mild_max_drift:=0.08 \
  medium_iters:=40 \
  medium_gain:=0.45 \
  medium_max_drift:=0.12
```

再启动 MPC，订阅后处理路径：

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

启动后必须检查：

```bash
rostopic echo -n1 /anti_slosh_path/candidate_report
rostopic echo -n1 /scout/global_path_anti_slosh
```

如果 `/anti_slosh_path/candidate_report` 没消息，通常说明还没有新的 `/scout/global_path`，需要先发目标。

如果 report 显示 selected=original，该包不能作为 GeoRef 有效样本，只能作为 fallback 样本记录。

## 8. 可选 ORIGINAL_REAL 启动

用于确认 post-processor topic chain 本身没有制造收益。

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  ds:=0.03 \
  max_candidate_level:=original \
  publish_debug:=true \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap
```

MPC 命令与 GEOREF_REAL 相同，仍订阅 `/scout/global_path_anti_slosh`。

## 9. 发同一目标

推荐使用固定脚本，避免 RViz 手点目标带来误差。

示例：

```bash
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic /scout/goal \
  --frame map \
  --x 2.5 \
  --y -1.5 \
  --yaw 0.0 \
  --repeat-count 5 \
  --repeat-rate 5
```

实物实验时需要先选一个安全目标点，然后 RAW / GEOREF / ORIGINAL 全部使用同一组 `x/y/yaw`。

## 10. 录包

现有脚本可以录大部分 MPC、晃动、IMU、视觉、MBF 话题：

```bash
cd $(rospack find scout_local_planner)
SLOSH_BAG_DIR=/data/$USER/slosh_bags/real \
./scripts/record_slosh_experiment.sh 0 RAW_REAL_run01
```

GEOREF_REAL 正式实验还必须记录以下话题：

```text
/scout/global_path_anti_slosh
/anti_slosh_path/candidate_report
/anti_slosh_path/debug/original
/anti_slosh_path/debug/mild
/anti_slosh_path/debug/medium
/anti_slosh_path/debug/strong
```

当前 `record_slosh_experiment.sh` 尚未包含这些 GeoRef 诊断话题。正式实物实验前有两个选择：

```text
推荐：先把上述话题补进 record_slosh_experiment.sh，保证所有数据在一个 bag 内。
临时：另开一个 rosbag 只录 GeoRef 诊断话题，但后处理会更麻烦。
```

临时补录命令：

```bash
mkdir -p /data/$USER/slosh_bags/real
rosbag record -O /data/$USER/slosh_bags/real/georef_diag_$(date +%Y%m%d_%H%M%S) \
  /scout/global_path_anti_slosh \
  /anti_slosh_path/candidate_report \
  /anti_slosh_path/debug/original \
  /anti_slosh_path/debug/mild \
  /anti_slosh_path/debug/medium \
  /anti_slosh_path/debug/strong
```

## 11. 单包检查

每包录完后先做快速检查，不要等 3 包录完才发现无效。

必查：

```bash
rosbag info /data/$USER/slosh_bags/real/<bag_name>.bag
```

GEOREF 包必查：

```bash
rostopic echo -b /data/$USER/slosh_bags/real/<bag_name>.bag -n1 /anti_slosh_path/candidate_report
```

有效 GEOREF 包应满足：

```text
有 /anti_slosh_path/candidate_report
有 /scout/global_path_anti_slosh
selected 不是 original，或至少 report 中能解释为何 fallback
无碰撞、无人工接管、无定位丢失
```

## 12. 分析口径

主段切片沿用仿真标准：

```text
/mpc_status == TRACKING
AND /terminal/mode == NONE
```

主指标：

```text
h_rms
h_p95
h_max
eta_dot_rms
modal_energy_rms
ay_p95
ax_p95
alpha_p95
active_time
solve_success_ratio
track_dist_p95
```

GeoRef 诊断指标：

```text
selected candidate
kappa ratio
predicted ay ratio
path length ratio
max drift
collision gate pass/fail
```

视觉真值指标，如果 RealSense 可用：

```text
height_peak_rel_mm p95
height_peak_rel_mm max
meniscus_valid ratio
meniscus_confidence mean
```

## 13. 结论写法

如果 GEOREF 通过，只能写：

```text
在相同 MBF 全局路径目标、相同 MPC 跟踪器且 MPC 内部防晃项关闭的条件下，
几何参考后处理降低了实物液体晃动观测指标。
```

不要写：

```text
MPC 代价函数主动抑制了液体晃动。
```

如果 GEOREF 不通过，优先按下面顺序定位：

```text
1. post-processor 是否实际选中 non-original candidate
2. candidate 是否通过 collision / ay gate
3. MPC 是否跟踪了 /scout/global_path_anti_slosh
4. /slosh/height 与视觉液面是否方向一致
5. 实物定位和底盘执行是否比仿真差太多
```

不要在失败后直接打开 `Q_slosh=5` 或旧 speed governor，因为那会混淆路线归因。

