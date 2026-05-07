# MPC 代价函数旧方案实物 Toy Example 验证方案

日期：2026-05-07

## 1. 目的

本方案用于补充说明：

```text
把 slosh soft cost 直接放进 MPC 内部，
不如把防晃逻辑前移到几何参考生成层。
```

它对应文档：

```text
docs/Claude/总结/2026-04-30_MPC代价函数旧方案失效总结.md
```

注意：

```text
这不是当前主方法的第一优先级。
主实验仍是 RAW_REAL vs GEOREF_REAL。
本 toy example 只在主实验完成或至少主链路确认可跑后执行。
```

## 2. 核心对比

使用同一条固定路径 replay，排除 MBF 每次重新规划带来的路径差异。

对比三组：

```text
RAW_REAL_FIXED
  fixed path -> normal MPC
  Q_slosh=0

QSLOSH_REAL_FIXED
  fixed path -> slosh-cost MPC
  Q_slosh=5
  Q_slosh_eta_dot=0 或历史 best

GEOREF_REAL_FIXED
  fixed path -> GeoRef post-processor -> normal MPC
  Q_slosh=0
```

目标不是证明 `Q_slosh` 在所有实物场景都失败，而是给出一个可控反例：

```text
在相同固定路径输入下，
MPC soft slosh cost 不能稳定优于 reference-first GeoRef。
```

## 3. 成功/失败判据

每组至少 1 包 smoke；若结果清晰，再补到每组 3 包。

主要指标：

```text
h_rms
h_p95
h_max
eta_dot_rms
modal_energy_rms
tracking_time
track_dist_p95
odom_ay_p95
solve_success_ratio
```

支持旧方案失效总结的典型结果：

```text
QSLOSH_REAL_FIXED 相对 RAW_REAL_FIXED:
  /slosh/height 不稳定下降，或 eta_dot / energy / tracking 变差

GEOREF_REAL_FIXED 相对 RAW_REAL_FIXED:
  /slosh/height、eta_dot、energy 至少部分稳定改善
  tracking_time 不超过 +15%
```

如果 QSLOSH_REAL_FIXED 在某一条 toy path 上也有改善，不要强行解释为旧总结错误。更严谨写法是：

```text
slosh soft cost can alter behavior and may help in selected cases,
but prior simulation and controlled comparisons show it is not the most robust primary mechanism.
```

## 4. 固定路径准备

先按实物流程启动：

```text
底盘 / 雷达 / 定位 / mbf_global
```

发布一个安全目标，等待 `/scout/global_path` 出现。

保存固定路径：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash

mkdir -p /data/$USER/fixed_paths/real

rosrun scout_local_planner fixed_global_path_runner.py \
  --mode capture \
  --input-topic /scout/global_path \
  --path-file /data/$USER/fixed_paths/real/toy_qslosh_path_A.json \
  --capture-timeout 30
```

检查：

```bash
python3 -m json.tool /data/$USER/fixed_paths/real/toy_qslosh_path_A.json | head
```

路径要求：

```text
开阔场地
有明显转弯或 S 型段
不贴墙
路径长度建议 >= 4 m
RAW_REAL_FIXED 能安全完成
```

## 5. RAW_REAL_FIXED

终端 1：启动 MPC：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_fixed \
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

终端 2：回到固定路径起点后 replay：

```bash
rosrun scout_local_planner fixed_global_path_runner.py \
  --mode replay \
  --path-file /data/$USER/fixed_paths/real/toy_qslosh_path_A.json \
  --output-topic /scout/global_path_fixed \
  --manual-start \
  --start-pos-tol 0.08 \
  --start-yaw-tol 0.15 \
  --publish-once-keepalive
```

## 6. QSLOSH_REAL_FIXED

终端 1：启动 MPC soft cost baseline：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_fixed \
  Q_slosh:=5 \
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

说明：

```text
第一版只开 Q_slosh=5。
不要同时打开 risk_scheduler / speed_governor / box constraint，
否则无法归因到 soft cost。
```

终端 2：使用同一个 replay 命令：

```bash
rosrun scout_local_planner fixed_global_path_runner.py \
  --mode replay \
  --path-file /data/$USER/fixed_paths/real/toy_qslosh_path_A.json \
  --output-topic /scout/global_path_fixed \
  --manual-start \
  --start-pos-tol 0.08 \
  --start-yaw-tol 0.15 \
  --publish-once-keepalive
```

可选第二版：

```text
如果历史 best 是 Q_slosh_eta_dot>0，
可另加一组 QSLOSH_DOT_REAL_FIXED。
但不要让 toy example 扩成大扫参。
```

## 7. GEOREF_REAL_FIXED

终端 1：启动 post-processor：

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path_fixed \
  output_topic:=/scout/global_path_anti_slosh \
  ds:=0.03 \
  max_candidate_level:=medium \
  publish_debug:=true \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  ay_ratio_limit:=1.0 \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  prediction_a_max:=1.0
```

终端 2：启动 MPC：

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

终端 3：replay 同一个 path：

```bash
rosrun scout_local_planner fixed_global_path_runner.py \
  --mode replay \
  --path-file /data/$USER/fixed_paths/real/toy_qslosh_path_A.json \
  --output-topic /scout/global_path_fixed \
  --manual-start \
  --start-pos-tol 0.08 \
  --start-yaw-tol 0.15 \
  --publish-once-keepalive
```

GEOREF 包必须检查：

```bash
rostopic echo -b /data/$USER/slosh_bags/real/<bag>.bag -n1 /anti_slosh_path/candidate_report
```

若 `selected=original`，该包不能说明 GeoRef candidate selection 有效。

## 8. 录包话题

使用主实物方案中的录包脚本，并确保包含：

```text
/scout/global_path_fixed
/scout/global_path_anti_slosh
/anti_slosh_path/candidate_report
/anti_slosh_path/debug/original
/anti_slosh_path/debug/mild
/anti_slosh_path/debug/mid
/anti_slosh_path/debug/medium
/anti_slosh_path/debug/strong
/slosh/height
/slosh/state
/slosh/modal_energy_norm
/odom
/cmd_vel
/mpc_status
/terminal/mode
/mpc/solve_success
```

临时录包示例：

```bash
mkdir -p /data/$USER/slosh_bags/real

rosbag record -O /data/$USER/slosh_bags/real/toy_qslosh_RAW_run01_$(date +%Y%m%d_%H%M%S) \
  /tf /tf_static /cmd_vel /odom /imu/data \
  /scout/global_path_fixed /scout/global_path_anti_slosh \
  /anti_slosh_path/candidate_report \
  /anti_slosh_path/debug/original \
  /anti_slosh_path/debug/mild \
  /anti_slosh_path/debug/mid \
  /anti_slosh_path/debug/medium \
  /anti_slosh_path/debug/strong \
  /slosh/height /slosh/state /slosh/modal_energy_norm \
  /mpc_status /terminal/mode /mpc/solve_success
```

## 9. 分析命令

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  /data/$USER/slosh_bags/real/<RAW_REAL_FIXED.bag> \
  /data/$USER/slosh_bags/real/<QSLOSH_REAL_FIXED.bag> \
  /data/$USER/slosh_bags/real/<GEOREF_REAL_FIXED.bag>
```

若需要出表，导出 CSV：

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  --csv /data/$USER/slosh_bags/real/toy_qslosh_summary.csv \
  /data/$USER/slosh_bags/real/<RAW_REAL_FIXED.bag> \
  /data/$USER/slosh_bags/real/<QSLOSH_REAL_FIXED.bag> \
  /data/$USER/slosh_bags/real/<GEOREF_REAL_FIXED.bag>
```

## 10. 论文写法

如果结果支持旧方案失效总结，可写：

```text
In addition to simulation failures of slosh soft costs,
a controlled fixed-path real-robot toy example shows that directly adding slosh cost
does not outperform reference-first GeoRef under the same path input.
```

中文：

```text
除仿真中多轮晃动软代价失败外，
固定路径实物 toy example 也显示，
在相同原始路径输入下，直接加入 MPC 晃动软代价不优于前端几何参考生成。
```

如果 QSLOSH_REAL_FIXED 意外表现不错，写法应改为：

```text
Soft slosh cost can help in selected fixed-path cases,
but it was not robust in simulation and is not used as the primary mechanism.
```

不要写：

```text
实物已经全面证明 Q_slosh 无效。
```
