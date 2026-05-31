# 20260527 SloshPriorityMPC 正式对比实验验证方案

> 状态：2026-05-31 按当前代码重写为实物现场 SOP。
> 目标：用同一固定模板路径与同一录包链路，对比 SloshPriorityMPC 与非晃液 baseline。
> 主评价真值：RGB max(left, center, right) 液面高度；`/slosh/height` 只作模型辅助。
> 代码方案：`docs/Claude/修改方案-时间-简介/2026-05-30_SloshPriorityMPC对比实验设计与代码解耦方案.md`

---

## 目录

- [0. 当前实验结构](#0-当前实验结构)
- [1. 实物启动总顺序](#1-实物启动总顺序)
- [2. 固定 P2 S 弯路径生成](#2-固定-p2-s-弯路径生成)
- [3. 外部 profile CSV 生成](#3-外部-profile-csv-生成)
- [4. MPC 启动命令](#4-mpc-启动命令)
- [5. 录包命名与录包命令](#5-录包命名与录包命令)
- [6. 启动后检查](#6-启动后检查)
- [7. 三层 smoke 与正式录制计划](#7-三层-smoke-与正式录制计划)
- [8. 分析口径](#8-分析口径)
- [9. 常见错误](#9-常见错误)

---

## 0. 当前实验结构

### 0.1 `experiment_group` 是唯一正式切换字段

正式实验必须显式传：

```text
experiment_group:=C / D / E / F / RPP_STYLE / BIAGIOTTI / TOPPRA / RUCKIG
```

代码内 `LocalPlannerROS::configureExperimentVariant()` 会派生：

```text
experiment_group -> controller_variant -> external_profile_mode
```

规则：

| group | controller_variant | external_profile_mode | Q_slosh | 作用 |
|---|---|---|---:|---|
| C | `mpc` | `none` | `0` | ordinary MPC |
| D | `mpc` | `none` | `>0` | slosh-only 消融 |
| E | `mpc` | `none` | `0` | smooth-only MPC |
| F | `mpc` | `none` | `>0` | SloshPriorityMPC / ours |
| RPP_STYLE | `rpp_speed_reg` | `none` | `0` | RPP 启发的非晃液 `v_ref` 调速 baseline |
| BIAGIOTTI | `mpc` | `biagiotti` | `0` | 开环 slosh-aware shaping baseline |
| TOPPRA | `mpc` | `toppra` | `0` | 限加速度 retiming baseline |
| RUCKIG | `mpc` | `ruckig` | `0` | 限 jerk retiming baseline |

`experiment_group` 会覆盖误传的 `controller_variant` / `external_profile_mode` 并 `ROS_WARN`。
但以下问题会直接 `ROS_FATAL`：

```text
非晃液组 Q_slosh > 0
D/F 组 Q_slosh <= 0
外部 profile 组没有 external_speed_profile_csv
RPP_STYLE 同时接外部 profile
```

### 0.2 论文表格结构

```text
正文主表：
  C / E / RPP_STYLE / BIAGIOTTI / F*

内部消融：
  C / D / E / F*

supplementary:
  TOPPRA / RUCKIG / BIAGIOTTI 开环 profile 阶梯
```

`F*` 表示先通过 Q sweep 选出的最终 F 工作点。

---

## 1. 实物启动总顺序

每次实物实验按这个顺序开终端。不要先用旧 JSON 生成 CSV，再重新生成路径；外部 CSV 必须基于本次最终 JSON 生成。

### 1.1 终端 A：启动底盘、雷达、定位、IMU、RealSense

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

脚本会检查：

```text
/camera/color/image_raw
/camera/color/camera_info
/imu/data
```

当前默认 RealSense color：

```text
1920x1080 @ 30 Hz
depth=false
infra=false
```

### 1.2 终端 B：固定 RealSense RGB 参数

推荐正式实验使用“冻结当前 RGB 参数”的方式，不依赖交互调参窗口。先启动 `launch_real_sensors_stack.sh`，让自动曝光/自动白平衡稳定几秒，确认 `/camera/color/image_raw` 有数据后执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

OUT_DIR=/home/geist/slosh_bags/real/<DATE>_visual_tuning/realsense_rgb_fixed_params \
src/scout_apps/control/scout_local_planner/scripts/set_realsense_rgb_manual_params.sh
```

脚本默认 `MODE=freeze_current`，会：

```text
1. 读取当前 exposure / gain / white_balance；
2. 关闭 enable_auto_exposure / enable_auto_white_balance；
3. 把刚刚读取到的当前值写回相机；
4. 保存当天固定参数 YAML 和可复用 apply 脚本。
```

如果不想冻结当前值，而是明确使用手动指定值：

```bash
MODE=manual EXPOSURE=6500 GAIN=24 WHITE_BALANCE=4200 \
OUT_DIR=/home/geist/slosh_bags/real/<DATE>_visual_tuning/realsense_rgb_fixed_params \
src/scout_apps/control/scout_local_planner/scripts/set_realsense_rgb_manual_params.sh
```

脚本会检查 `/camera/color/image_raw` 频率。正式录包前至少确认：

```bash
rosrun dynamic_reconfigure dynparam get /camera/rgb_camera enable_auto_exposure
rosrun dynamic_reconfigure dynparam get /camera/rgb_camera exposure
rosrun dynamic_reconfigure dynparam get /camera/rgb_camera gain
rosrun dynamic_reconfigure dynparam get /camera/rgb_camera enable_auto_white_balance
rosrun dynamic_reconfigure dynparam get /camera/rgb_camera white_balance
rostopic hz /camera/color/image_raw
```

当天所有 pilot / 正式组必须使用同一份固定参数。不要在组间重新打开自动曝光或改白平衡。

### 1.2.1 可选：交互视觉调参

只在光照/相机位置变化明显后做。调好曝光、增益、白平衡后，可以把保存报告中的参数写回 `set_realsense_rgb_manual_params.sh` 的环境变量命令。

注意：实物机上退出 OpenCV 调参窗口可能导致 RealSense 图像话题异常。若必须使用交互窗口，建议只按 `s` 保存参数，不在实验前频繁退出窗口。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

mkdir -p /home/geist/slosh_bags/real/<DATE>_visual_tuning/realsense_rgb_interactive

python3 src/scout_apps/sensors/realsense_liquid_measurement/scripts/interactive_realsense_red_tuner.py \
  --image-topic /camera/color/image_raw \
  --dynparam-ns /camera/rgb_camera \
  --select-roi \
  --out-dir /home/geist/slosh_bags/real/<DATE>_visual_tuning/realsense_rgb_interactive \
  --init-exposure 7000 \
  --init-gain 32 \
  --init-white-balance 4200 \
  --hue1-low 0 --hue1-high 12 \
  --hue2-low 168 --hue2-high 179 \
  --sat-min 90 --val-min 60
```

注意：

```text
<DATE> 必须替换成真实目录名，例如 20260531。
交互窗口只用于找参数；正式录包前优先使用 set_realsense_rgb_manual_params.sh 固定参数。
```

### 1.3 终端 C：生成本次 P2 固定模板路径

这个节点先启动并等待 `/scout/goal`。它会用**当前小车位姿**作为起点，收到 goal 后生成 `/scout/global_path_fixed` 和 JSON。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

mkdir -p /home/geist/fixed_paths/real/<DATE>

rosrun scout_local_planner template_fixed_path_generator.py \
  --template s_curve \
  --goal-topic /scout/goal \
  --output-topic /scout/global_path_fixed \
  --path-file /home/geist/fixed_paths/real/<DATE>/P2_s_curve_d200.json \
  --start-heading current \
  --spacing 0.05 \
  --amplitude-ratio 0.18 \
  --min-amplitude 0.25 \
  --max-amplitude 1.20 \
  --publish-count 0
```

### 1.4 终端 D：发送固定 goal

2026-05-31 实物 pilot smoke 使用以下固定 P2 终点。后续同一天所有组必须使用同一个 goal，除非重新生成整套路径和外部 profile：

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

如果换场地或换地面标记，需要同步更新这里的 goal，并重新生成：

```text
/home/geist/fixed_paths/real/<DATE>/P2_s_curve_d200.json
/home/geist/fixed_paths/real/<DATE>/baseline_profiles/*.csv
```

确认 JSON 已生成：

```bash
ls -lh /home/geist/fixed_paths/real/<DATE>/P2_s_curve_d200.json
rostopic echo -n 1 /scout/global_path_fixed/header
```

---

## 2. 固定 P2 S 弯路径生成

本实验使用“固定模板路径规则”，不是 MBF 每次自由规划：

```text
template_fixed_path_generator.py 负责生成 /scout/global_path_fixed；
send_fixed_goal.py 只负责提供终点；
MPC 订阅 /scout/global_path_fixed；
MBF 可保持运行，但固定路径主实验不依赖 MBF 重新规划。
```

公平性要求：

```text
1. 每包实验前小车回到同一地面标记附近；
2. 使用同一终点 goal；
3. 使用同一 template 参数；
4. 每次实际使用的 JSON 都保存；
5. 对 BIAGIOTTI/TOPPRA/RUCKIG，必须用本次 JSON 重新生成 CSV。
```

如果起点偏差太大，废弃该 run，重新摆车。

---

## 3. 外部 profile CSV 生成

只在 `BIAGIOTTI / TOPPRA / RUCKIG` 组执行。`C / D / E / F / RPP_STYLE` 不需要 CSV。

### 3.1 目录

```bash
mkdir -p /home/geist/fixed_paths/real/<DATE>/baseline_profiles
```

### 3.2 TOPPRA-style

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

python3 src/scout_apps/control/scout_local_planner/scripts/analysis/retime_toppra_style.py \
  --path-file /home/geist/fixed_paths/real/<DATE>/P2_s_curve_d200.json \
  --out-csv /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_toppra_style.csv \
  --plot /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_toppra_style.png \
  --v-max 0.80 \
  --a-max 0.60 \
  --decel-max 0.80 \
  --ds 0.02
```

### 3.3 Ruckig-style

实物机需要 Python `ruckig`。Python 3.8 下不要装新版源码包；优先用二进制旧版：

```bash
python3 -m pip install --user --only-binary=:all: 'ruckig==0.9.2'
```

生成 CSV：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

python3 src/scout_apps/control/scout_local_planner/scripts/analysis/retime_ruckig_style.py \
  --path-file /home/geist/fixed_paths/real/<DATE>/P2_s_curve_d200.json \
  --out-csv /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_ruckig_style.csv \
  --plot /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_ruckig_style.png \
  --v-max 0.80 \
  --a-max 0.60 \
  --j-max 1.50 \
  --delta-time 0.02
```

如果安装失败，先跳过 RUCKIG，不要用手写曲线冒充。

### 3.4 Biagiotti-style

`omega_n` 和 `damping_ratio` 用当前容器识别值。当前脚本参数名是 `--damping-ratio`，不是 `--zeta`。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

python3 src/scout_apps/control/scout_local_planner/scripts/analysis/shape_biagiotti.py \
  --path-file /home/geist/fixed_paths/real/<DATE>/P2_s_curve_d200.json \
  --out-csv /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_biagiotti_style.csv \
  --plot /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_biagiotti_style.png \
  --debug-prefix /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_biagiotti_style \
  --omega-n 5.0 \
  --damping-ratio 0.05 \
  --v-max 0.80 \
  --a-max 0.60 \
  --decel-max 0.80 \
  --ds 0.02 \
  --delta-time 0.02
```

检查：

```bash
head /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_biagiotti_style.csv
tail /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_biagiotti_style.csv
ls /home/geist/fixed_paths/real/<DATE>/baseline_profiles/*biagiotti_style*_time_law.csv
```

---

## 4. MPC 启动命令

以下命令都订阅：

```text
global_path_topic:=/scout/global_path_fixed
```

### 4.1 参数组定义

基线跟踪参数 `C_DEFAULT`：

```text
mpc_Q_lag=0.5
mpc_Q_contour=32.0
mpc_Q_etheta=15.0
mpc_Q_v=9.0
mpc_R_a=0.4
mpc_R_omega=2.0
mpc_R_da=0.5
mpc_R_domega=4.0
```

平滑参数 `SMOOTH`：

```text
mpc_Q_lag=0.5
mpc_Q_contour=28.0
mpc_Q_etheta=12.0
mpc_Q_v=3.0
mpc_R_a=1.0
mpc_R_omega=2.0
mpc_R_da=2.0
mpc_R_domega=6.0
```

共同 terminal / execution 参数：

```text
terminal_slowdown_distance=2.00
terminal_slowdown_v_max=0.80
terminal_slowdown_Q_v=40.0
terminal_slowdown_terminal_factor_v=5.0
terminal_capture_stop_distance=0.70
terminal_capture_v=0.18
v_des_accel_limit=0.60
v_des_decel_limit=0.80
max_lat_accel_safety=2.5
filter_alpha_v=1.0
filter_alpha_omega=1.0
filter_kappa_boost=0.0
```

### 4.2 C：ordinary MPC

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  experiment_group:=C \
  global_path_topic:=/scout/global_path_fixed \
  mpc_Q_lag:=0.5 \
  mpc_Q_contour:=32.0 \
  mpc_Q_etheta:=15.0 \
  mpc_Q_v:=9.0 \
  mpc_R_a:=0.4 \
  mpc_R_omega:=2.0 \
  mpc_R_da:=0.5 \
  mpc_R_domega:=4.0 \
  Q_slosh:=0.0 \
  terminal_factor_slosh_eta:=0.0 \
  terminal_factor_slosh_eta_dot:=0.0 \
  terminal_slowdown_enable:=true \
  terminal_slowdown_distance:=2.00 \
  terminal_slowdown_v_max:=0.80 \
  terminal_slowdown_Q_v:=40.0 \
  terminal_slowdown_terminal_factor_v:=5.0 \
  terminal_capture_stop_enable:=true \
  terminal_capture_stop_distance:=0.70 \
  terminal_capture_v:=0.18 \
  max_lat_accel_safety:=2.5 \
  v_des_rate_limit_enable:=true \
  v_des_accel_limit:=0.60 \
  v_des_decel_limit:=0.80 \
  filter_alpha_v:=1.0 \
  filter_alpha_omega:=1.0 \
  filter_kappa_boost:=0.0 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

### 4.3 D：slosh-only

复制 C 命令，只改：

```bash
experiment_group:=D
Q_slosh:=<F候选Q，例如 5.0>
terminal_factor_slosh_eta:=5.0
terminal_factor_slosh_eta_dot:=3.0
```

D 必须保持 C_DEFAULT 的 `mpc_R_a=0.4 / mpc_R_da=0.5`，否则它会变成 F。

### 4.4 E：smooth-only

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  experiment_group:=E \
  global_path_topic:=/scout/global_path_fixed \
  mpc_Q_lag:=0.5 \
  mpc_Q_contour:=28.0 \
  mpc_Q_etheta:=12.0 \
  mpc_Q_v:=3.0 \
  mpc_R_a:=1.0 \
  mpc_R_omega:=2.0 \
  mpc_R_da:=2.0 \
  mpc_R_domega:=6.0 \
  Q_slosh:=0.0 \
  terminal_factor_slosh_eta:=0.0 \
  terminal_factor_slosh_eta_dot:=0.0 \
  terminal_slowdown_enable:=true \
  terminal_slowdown_distance:=2.00 \
  terminal_slowdown_v_max:=0.80 \
  terminal_slowdown_Q_v:=40.0 \
  terminal_slowdown_terminal_factor_v:=5.0 \
  terminal_capture_stop_enable:=true \
  terminal_capture_stop_distance:=0.70 \
  terminal_capture_v:=0.18 \
  max_lat_accel_safety:=2.5 \
  v_des_rate_limit_enable:=true \
  v_des_accel_limit:=0.60 \
  v_des_decel_limit:=0.80 \
  filter_alpha_v:=1.0 \
  filter_alpha_omega:=1.0 \
  filter_kappa_boost:=0.0 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

### 4.5 F：SloshPriorityMPC / ours

复制 E 命令，只改：

```bash
experiment_group:=F
Q_slosh:=<F_best，例如 5.0 或 7.0>
terminal_factor_slosh_eta:=5.0
terminal_factor_slosh_eta_dot:=3.0
```

### 4.6 RPP_STYLE

RPP-style 使用 C_DEFAULT R，不使用 slosh cost，不接外部 CSV。

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  experiment_group:=RPP_STYLE \
  global_path_topic:=/scout/global_path_fixed \
  rpp_regulated_min_radius:=0.50 \
  rpp_approach_dist:=0.70 \
  rpp_min_approach_v:=0.05 \
  rpp_replace_base_curvature_cap:=true \
  mpc_Q_lag:=0.5 \
  mpc_Q_contour:=32.0 \
  mpc_Q_etheta:=15.0 \
  mpc_Q_v:=9.0 \
  mpc_R_a:=0.4 \
  mpc_R_omega:=2.0 \
  mpc_R_da:=0.5 \
  mpc_R_domega:=4.0 \
  Q_slosh:=0.0 \
  terminal_factor_slosh_eta:=0.0 \
  terminal_factor_slosh_eta_dot:=0.0 \
  terminal_slowdown_enable:=true \
  terminal_slowdown_distance:=2.00 \
  terminal_slowdown_v_max:=0.80 \
  terminal_slowdown_Q_v:=40.0 \
  terminal_slowdown_terminal_factor_v:=5.0 \
  terminal_capture_stop_enable:=true \
  terminal_capture_stop_distance:=0.70 \
  terminal_capture_v:=0.18 \
  max_lat_accel_safety:=2.5 \
  v_des_rate_limit_enable:=true \
  v_des_accel_limit:=0.60 \
  v_des_decel_limit:=0.80 \
  filter_alpha_v:=1.0 \
  filter_alpha_omega:=1.0 \
  filter_kappa_boost:=0.0 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

### 4.7 BIAGIOTTI / TOPPRA / RUCKIG

外部 profile 组使用 C_DEFAULT R，不使用 slosh cost。

BIAGIOTTI：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  experiment_group:=BIAGIOTTI \
  global_path_topic:=/scout/global_path_fixed \
  external_speed_profile_csv:=/home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_biagiotti_style.csv \
  external_profile_execution_cap_enable:=true \
  external_profile_execution_accel_limit:=0.60 \
  external_profile_execution_decel_limit:=0.80 \
  external_profile_execution_jerk_limit:=0.0 \
  mpc_Q_lag:=0.5 \
  mpc_Q_contour:=32.0 \
  mpc_Q_etheta:=15.0 \
  mpc_Q_v:=9.0 \
  mpc_R_a:=0.4 \
  mpc_R_omega:=2.0 \
  mpc_R_da:=0.5 \
  mpc_R_domega:=4.0 \
  Q_slosh:=0.0 \
  terminal_factor_slosh_eta:=0.0 \
  terminal_factor_slosh_eta_dot:=0.0 \
  terminal_slowdown_enable:=true \
  terminal_slowdown_distance:=2.00 \
  terminal_slowdown_v_max:=0.80 \
  terminal_slowdown_Q_v:=40.0 \
  terminal_slowdown_terminal_factor_v:=5.0 \
  terminal_capture_stop_enable:=true \
  terminal_capture_stop_distance:=0.70 \
  terminal_capture_v:=0.18 \
  max_lat_accel_safety:=2.5 \
  v_des_rate_limit_enable:=true \
  v_des_accel_limit:=0.60 \
  v_des_decel_limit:=0.80 \
  filter_alpha_v:=1.0 \
  filter_alpha_omega:=1.0 \
  filter_kappa_boost:=0.0 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

TOPPRA 只替换：

```bash
experiment_group:=TOPPRA
external_speed_profile_csv:=/home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_toppra_style.csv
```

RUCKIG 只替换：

```bash
experiment_group:=RUCKIG
external_speed_profile_csv:=/home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_ruckig_style.csv
external_profile_execution_jerk_limit:=1.50
```

---

## 5. 录包命名与录包命令

`record_slosh_experiment.sh` 会读取当前 ROS param：

```text
/scout_local_planner/experiment_group
/scout_local_planner/controller_variant
/scout_local_planner/external_profile_mode
/scout_local_planner/mpc/R_a
/scout_local_planner/mpc/R_da
```

正式 group 的 bag 名自动变成：

```text
slosh_<GROUP>_qs<Q>_ra<R_a>_rda<R_da>_<timestamp>_<suffix>.bag
```

录包命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

export SLOSH_BAG_DIR=/home/geist/slosh_bags/real/<DATE>_formal_compare
export SLOSH_BAG_MODE=real
export SLOSH_RECORD_ALL=true

# 例：C 组
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 0 P2_s_curve_C_block01_run01

# 例：F 组
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh <F_best_Q> P2_s_curve_F_block01_run01
```

建议 suffix：

```text
P2_s_curve_C_block01_run01
P2_s_curve_E_block01_run01
P2_s_curve_RPP_STYLE_block01_run01
P2_s_curve_BIAGIOTTI_block01_run01
P2_s_curve_F_block01_run01
P2_s_curve_D_block01_run01
P2_s_curve_TOPPRA_supp_run01
P2_s_curve_RUCKIG_supp_run01
```

---

## 6. 启动后检查

### 6.1 所有组都检查

```bash
rosparam get /scout_local_planner/experiment_group
rosparam get /scout_local_planner/controller_variant
rosparam get /scout_local_planner/external_profile_mode

rostopic echo -n 1 /diagnostics/experiment_group
rostopic echo -n 1 /diagnostics/controller_variant
rostopic echo -n 1 /diagnostics/external_profile_mode
rostopic echo -n 1 /diagnostics/mpc_cost_variant

rostopic echo -n 1 /reference/v_ref_horizon
rostopic echo -n 1 /mpc/cost_breakdown
rostopic echo -n 1 /slosh/state
```

### 6.2 RPP_STYLE 额外检查

```bash
rostopic echo -n 1 /rpp_speed_reg/active
rostopic echo -n 1 /rpp_speed_reg/v_raw
rostopic echo -n 1 /rpp_speed_reg/v_out
rostopic echo -n 1 /rpp_speed_reg/curvature_active
rostopic echo -n 1 /rpp_speed_reg/approach_active
```

期望：

```text
/diagnostics/experiment_group = RPP_STYLE
/diagnostics/controller_variant = rpp_speed_reg
/diagnostics/external_profile_mode = none
/rpp_speed_reg/active = 1
/rpp_speed_reg/v_out <= /rpp_speed_reg/v_raw
```

### 6.3 外部 profile 组额外检查

```bash
rosparam get /scout_local_planner/path_handler/external_speed_profile_csv
rosparam get /scout_local_planner/external_profile_execution_cap/enable

rostopic echo -n 1 /profile_cap/active
rostopic echo -n 1 /profile_cap/v_profile
rostopic echo -n 1 /profile_cap/cmd_v_pre_cap
rostopic echo -n 1 /profile_cap/cmd_v_post_cap
```

期望：

```text
external_speed_profile_csv 路径正确；
external_profile_execution_cap/enable = true；
/profile_cap/* 有数据；
/reference/v_ref_horizon 的 max 不应长期超过 CSV v_max。
```

---

## 7. 三层 smoke 与正式录制计划

不要直接进入正式 n=5 实物对比。任何代码/launch/文档调整后，必须先过三层 smoke。

### 7.1 第一层：本机静态 smoke

在开发机或实物机都可以执行。目标是确认代码、launch 参数和脚本入口没有明显错误。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

catkin_make --pkg scout_local_planner
roslaunch --nodes scout_local_planner slosh_experiment.launch
roslaunch --nodes scout_local_planner slosh_experiment_sim.launch

bash -n src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh
python3 -m py_compile \
  src/scout_apps/control/scout_local_planner/scripts/analysis/path_profile_utils.py \
  src/scout_apps/control/scout_local_planner/scripts/analysis/retime_toppra_style.py \
  src/scout_apps/control/scout_local_planner/scripts/analysis/retime_ruckig_style.py \
  src/scout_apps/control/scout_local_planner/scripts/analysis/shape_biagiotti.py
```

通过条件：

```text
catkin_make 通过；
两个 roslaunch --nodes 都能解析出 /scout_local_planner；
record 脚本 bash -n 通过；
profile 脚本 py_compile 通过。
```

### 7.2 第二层：仿真 fixed-path smoke

目标是验证新代码流，而不是验证真实液面效果。至少跑：

```text
C
RPP_STYLE
BIAGIOTTI 或 TOPPRA
F
```

仿真 smoke 检查：

```text
/diagnostics/experiment_group 与启动组一致；
能进入 TRACKING；
能到终点或至少没有 ERROR；
/slosh/* 持续发布；
/mpc/cost_breakdown 有数据；
RPP_STYLE: /rpp_speed_reg/active = 1；
外部 profile: /profile_cap/* 有数据。
```

如果仿真 smoke 中：

```text
RPP_STYLE 的 /rpp_speed_reg/active 不是 1；
BIAGIOTTI/TOPPRA/RUCKIG 的 /profile_cap/* 没数据；
CSV 路径不正确；
MPC 订阅的不是 /scout/global_path_fixed；
```

不要进入实物，先修代码或命令。

### 7.3 第三层：实物 pilot smoke

每组先 1 包，按正式流程启动，但 **不进入论文正式统计**。2026-05-31 pilot 建议顺序：

```text
C -> E -> RPP_STYLE -> BIAGIOTTI -> F -> D
```

建议 bag 后缀：

```text
P2_s_curve_C_pilot_run01
P2_s_curve_E_pilot_run01
P2_s_curve_RPP_STYLE_pilot_run01
P2_s_curve_BIAGIOTTI_pilot_run01
P2_s_curve_F_pilot_run01
P2_s_curve_D_pilot_run01
```

可选补充：

```text
P2_s_curve_TOPPRA_pilot_run01
P2_s_curve_RUCKIG_pilot_run01
```

`BIAGIOTTI / TOPPRA / RUCKIG` 必须先用本次最终 JSON 生成对应 CSV，再启动 MPC。

smoke 只验证：

```text
能开始 TRACKING；
能到终点；
无 ERROR；
/slosh/* 不断；
/mpc/cost_breakdown 有数据；
RGB 话题正常；
RPP/profile 诊断和组别一致。
```

pilot smoke 不进入论文正式统计。pilot 通过后再做 Q sweep 和正式 block。

### 7.4 F 的 Q sweep

建议：

```text
F_Q3
F_Q5
F_Q7
F_Q10
```

选择 `F*`：

```text
首看 RGB max-LCR p95/RMS/peak；
再看 ax_p95 / ay_p95；
再看 model p95/peak 是否方向一致；
duration 或 tracking error 明显变差的 Q 不作为主方法。
```

### 7.5 正式 block

推荐每个 block 随机顺序：

```text
C / E / RPP_STYLE / BIAGIOTTI / F*
```

录 5 个 block。再补：

```text
D n=5
TOPPRA n=2-3
RUCKIG n=2-3
```

每个 block 前后录 static bag 10-15 s，用于 RGB jitter / 光照漂移检查。

---

## 8. 分析口径

### 8.1 主窗口

主论文窗口：

```text
TRACKING start -> first terminal/capture - 1.0 s
```

terminal approach 不进入主效果统计，因为终点停车仍是独立 jerk/ax 诊断问题，会污染 slosh cost 的主效果归因。

### 8.2 主指标

RGB 主指标必须使用：

```text
max(left, center, right)
```

而不是 median。指标：

```text
RGB p95
RGB RMS
RGB peak
AUC_0.5mm
model p95 / peak
ax_p95 / ay_p95
duration
tracking error
```

### 8.3 公平性

若某方法 completion time 相对 F* 或 C 差异超过 ±10%：

```text
不能写“纯 anti-slosh 优势”；
只能写“晃动-时间 trade-off”。
```

### 8.4 Ferrari-style 保真度

保真度只用于解释模型可信度，不用于替代 RGB 真值。

```text
model and RGB are directionally compared, not assumed identical.
```

---

## 9. 常见错误

### 9.1 roslaunch 参数名写错

正确参数名是：

```text
mpc_R_a
mpc_R_da
mpc_R_omega
mpc_R_domega
```

不是：

```text
R_a
R_da
```

### 9.2 Biagiotti 参数名写错

正确：

```text
--damping-ratio 0.05
--debug-prefix <prefix>
```

当前脚本没有：

```text
--zeta
--provenance-json
```

### 9.3 CSV 与路径不一致

错误流程：

```text
用旧 JSON 生成 CSV；
随后 template_fixed_path_generator.py 又覆盖生成新 JSON。
```

正确流程：

```text
先生成本次最终 JSON；
再用这个 JSON 生成 CSV；
再启动外部 profile 组 MPC。
```

### 9.4 忘记 `global_path_topic`

固定路径实验必须传：

```text
global_path_topic:=/scout/global_path_fixed
```

否则 MPC 会订阅默认 `/scout/global_path`，不一定是模板固定路径。

### 9.5 把 `/slosh/height` 当真值

不允许。主结论看 RGB max-LCR。

`/slosh/height` 用于：

```text
模型趋势；
cost 是否生效；
Ferrari-style 保真度辅助解释。
```
