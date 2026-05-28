# 20260527 SloshPriorityMPC 正式对比实验验证方案

## 目录

- [20260527 SloshPriorityMPC 正式对比实验验证方案](#20260527-sloshprioritympc-正式对比实验验证方案)
  - [目录](#目录)
  - [0. 本方案和 20260518 方案的关系](#0-本方案和-20260518-方案的关系)
  - [1. 实验目标](#1-实验目标)
  - [2. 方法分组](#2-方法分组)
  - [3. 公平性规则](#3-公平性规则)
    - [3.1 固定路径](#31-固定路径)
    - [3.2 主窗口](#32-主窗口)
    - [3.3 RGB 主指标](#33-rgb-主指标)
    - [3.4 TOPPRA/RUCKIG 公平性](#34-toppraruckig-公平性)
    - [3.5 当前实验优先级：先调 F，再调外部 baseline](#35-当前实验优先级先调-f再调外部-baseline)
  - [4. 实物启动](#4-实物启动)
    - [4.1 传感器 / 底盘 / 定位](#41-传感器--底盘--定位)
    - [4.2 MBF 全局规划器](#42-mbf-全局规划器)
  - [5. 录包前 RGB 视觉准备](#5-录包前-rgb-视觉准备)
  - [6. 固定 P2 S 弯路径准备](#6-固定-p2-s-弯路径准备)
    - [6.1 生成并持续发布固定路径](#61-生成并持续发布固定路径)
    - [6.2 发送同一个 P2 goal](#62-发送同一个-p2-goal)
  - [7. TOPPRA / Ruckig 外部速度剖面准备](#7-toppra--ruckig-外部速度剖面准备)
    - [7.1 TOPPRA-style CSV](#71-toppra-style-csv)
    - [7.2 Ruckig-style CSV](#72-ruckig-style-csv)
  - [8. MPC 启动命令](#8-mpc-启动命令)
    - [8.1 C 组：BASE / ordinary MPC](#81-c-组base--ordinary-mpc)
    - [8.2 D 组：SLOSH\_ONLY / 论文创新核心](#82-d-组slosh_only--论文创新核心)
    - [8.3 E 组：SMOOTH\_ONLY](#83-e-组smooth_only)
    - [8.4 F 组：OURS\_FULL](#84-f-组ours_full)
    - [8.4A F 组 Q\_slosh sweep](#84a-f-组-q_slosh-sweep)
    - [8.5 TOPPRA-style baseline](#85-toppra-style-baseline)
    - [8.6 Ruckig-style baseline](#86-ruckig-style-baseline)
    - [8.7 外部 profile 启动后检查](#87-外部-profile-启动后检查)
    - [8.8 TOPPRA/RUCKIG speed-matched 调参](#88-toppraruckig-speed-matched-调参)
  - [9. 录包命名和顺序](#9-录包命名和顺序)
    - [9.1 第一轮：F 组 Q\_slosh sweep](#91-第一轮f-组-q_slosh-sweep)
    - [9.2 第二轮 pilot：baseline 接线和 duration fairness](#92-第二轮-pilotbaseline-接线和-duration-fairness)
    - [9.3 正式随机 block：每组 n=3](#93-正式随机-block每组-n3)
  - [10. 录后验收](#10-录后验收)
  - [11. 分析口径](#11-分析口径)
  - [12. 进入正式统计的条件](#12-进入正式统计的条件)

## 0. 本方案和 20260518 方案的关系

本方案直接继承：

```text
docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md
```

能复制的命令和流程尽量复制。区别是：

```text
20260518 方案：
  重点是 terminal 收敛、固定路径链路、重构 smoke、外部 profile 接线。

本方案：
  重点是下一轮正式论文对比实验：
    先做 F 组 Q_slosh sweep，确定 SloshPriorityMPC 的合理工作点 F*；
    再把 TOPPRA/RUCKIG 调到与 F* 近似同速或同 duration；
    最后做 C / D / E / F* / TOPPRA / RUCKIG 正式对比。

  原则：
    不为了让 ours 赢而削弱 TOPPRA/RUCKIG；
    也不在 F 尚未调到合理工作点时，直接和强 smooth baseline 硬拼。
```

本方案只讨论当前 MPC cost / fixed-path speed-profile baseline，不涉及 OSCRS。

## 1. 实验目标

核心问题：

```text
在同一条固定 P2 S 弯几何路径上，
显式液体模态 slosh cost 是否比普通 tracking MPC 和 non-slosh smooth baseline 更有价值？
```

需要分开回答三件事：

```text
1. C vs D:
   modal slosh cost 单独是否有效？

2. E vs F:
   在同样 smooth-control MPC 框架下，
   加入 slosh modal cost 是否提供额外收益？

3. F vs TOPPRA/RUCKIG:
   与外部 acceleration-limited / jerk-limited retiming baseline 相比，
   F 是否有更好的 RGB / model / duration / tracking trade-off？
```

注意：

```text
如果 TOPPRA/RUCKIG 明显更慢，只能写成 trade-off；
不能直接写“TOPPRA/RUCKIG 更优”或“ours 更弱”。
```

## 2. 方法分组

| 组别 | 名称 | 含义 | 论文角色 |
|---|---|---|---|
| C | BASE / ordinary MPC | 普通 tracking MPC | 基线 |
| D | SLOSH_ONLY | C + modal slosh cost | 论文创新核心消融 |
| E | SMOOTH_ONLY | C + 更强控制平滑，`Q_slosh=0` | non-slosh smooth MPC baseline |
| F | OURS_FULL | D + 更强控制平滑 | 工程完整主线 / full proposed controller |
| TOPPRA | TOPPRA-style retiming | 同一路径外部限加速度 `v_ref(s)`，无 slosh | 外部 smooth baseline |
| RUCKIG | Ruckig-style retiming | 同一路径外部限 jerk `v_ref(s)`，无 slosh | 外部 smooth baseline |

论文口径：

```text
创新核心 = D 组 modal slosh objective
工程主线 = F 组 modal slosh objective + smooth-control regularization
E/TOPPRA/RUCKIG = 不使用液体模型的 smooth baseline
```

本轮调参后，正式主方法记为：

```text
F*: 从 F_Q3 / F_Q5 / F_Q7 / F_Q10 中选出的 SloshPriorityMPC 工作点。
```

如果 `Q_slosh=10` 的液面最低但 completion time 明显变长，论文主方法优先选更平衡的工作点；
`Q_slosh=10` 可作为 aggressive ablation，不强行作为主线。

## 3. 公平性规则

### 3.1 固定路径

所有组必须使用同一条：

```text
/home/geist/fixed_paths/real/<DATE>/P2_s_curve_d200.json
```

规则：

```text
1. 只生成一次固定 P2_s_curve JSON；
2. C/D/E/F/TOPPRA/RUCKIG 全部 replay 同一 JSON；
3. TOPPRA/RUCKIG 只替换 v_ref(s)，不改变路径几何；
4. 如果起点偏差导致初始 tracking error 明显变大，该 run 作废重来。
```

### 3.2 主窗口

主效果窗口固定为：

```text
TRACKING start -> 第一次 terminal/capture 相关状态之前
```

terminal approach 单独诊断，不进入主效果统计。

### 3.3 RGB 主指标

按 `docs/重要文档/红色液体视觉验证固定流程.md` 当前口径：

```text
H_vis(t) = abs(rolling_median(max(h_left, h_center, h_right), window=5))
主指标 = p95(H_vis) over TRACKING_PRE_TERMINAL
```

同时报告：

```text
RGB max-LCR RMS
RGB max-LCR peak
model p95 / peak
ax_p95 / ay_p95
duration
tracking error
```

`/slosh/height` 只作为模型侧 surrogate，不当真实液面主指标。

### 3.4 TOPPRA/RUCKIG 公平性

先用 F 组 duration 作为参考：

```text
T_F = F 组主窗口平均 duration
```

TOPPRA/RUCKIG 必须尽量满足：

```text
T_profile ∈ [0.9 T_F, 1.1 T_F]
```

如果 TOPPRA/RUCKIG 超出 ±10%：

```text
1. 先不要扩展到 n=3；
2. 调整外部 profile 的 v_max / a_max / decel_max / j_max；
3. 重新各跑 1 包；
4. 直到 duration 接近 F 后再进入正式统计。
```

若实物时间有限，允许保留不匹配包，但报告必须写成：

```text
该结果是 time/speed trade-off，不是公平单点性能优势。
```

### 3.5 当前实验优先级：先调 F，再调外部 baseline

20260524 和 20260526 的现有报告已经说明：

```text
1. TOPPRA/RUCKIG 是很强的 non-slosh smooth baseline；
2. 当前 TOPPRA/RUCKIG 在 duration 上大致落在 F 的 ±10% 内；
3. 但 TOPPRA/RUCKIG 的速度/加速度行为更保守，RGB 端可能明显占优；
4. 直接用未充分调参的 F_Q5 和它们比较，不能代表 SloshPriorityMPC 的最佳工作点。
```

因此下一轮实验采用以下顺序：

```text
Step 1:
  固定路径、terminal、E/F smooth 参数不变；
  只扫 F 组的 Q_slosh，得到 F_Q3 / F_Q5 / F_Q7 / F_Q10。

Step 2:
  选择综合最优 F*：
    RGB max-LCR p95/RMS/peak 优先；
    ax_p95 / ay_p95 辅助解释；
    model p95/peak 作为模型侧一致性；
    duration 不得明显恶化；
    tracking error 不得明显恶化。

Step 3:
  以 F* 的主窗口 duration、平均速度和速度峰值作为参考；
  再调 TOPPRA/RUCKIG 的 v_max / a_max / decel_max / j_max，
  得到 speed-matched 或 duration-matched external baseline。

Step 4:
  用 C / D / E / F* / TOPPRA-matched / RUCKIG-matched 做正式随机 block。
```

调参边界：

```text
不要通过降低 R_a/R_da 来“放大 slosh”；
不要启用 slosh_preview_factor，除非 peak debug 证明主峰来自 horizon future k>0；
不要把 terminal phase 纳入主效果统计；
不要把 /slosh/height 当 RGB 真值。
```

## 4. 实物启动

### 4.1 传感器 / 底盘 / 定位

终端 A：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

检查：

```bash
rostopic hz /camera/color/image_raw
rostopic hz /imu/data
rostopic echo -n 1 /camera/color/camera_info | grep -E "width|height"
```

期望：

```text
/camera/color/image_raw 正常；
/imu/data 正常；
RGB 分辨率符合当天设置；
Cartographer localization 稳定。
```

### 4.2 MBF 全局规划器

终端 B：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

roslaunch scout_global_planner mbf_global.launch
```

固定路径实验不依赖 MBF 重新规划，但保持该栈启动有助于保持系统环境一致。

## 5. 录包前 RGB 视觉准备

正式录包前先完成：

```text
1. 固定 RealSense RGB exposure / gain / white_balance；
2. 关闭 auto exposure / auto white balance；
3. 重新做当天三标尺 YAML；
4. 重新采 HSV；
5. 记录当天 HSV 参数；
6. 保留 static bag。
```

交互调参命令复制自 20260518 方案：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

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

固定相机参数示例：

```bash
rosrun dynamic_reconfigure dynparam set /camera/rgb_camera enable_auto_exposure false
rosrun dynamic_reconfigure dynparam set /camera/rgb_camera exposure <BEST_EXPOSURE>
rosrun dynamic_reconfigure dynparam set /camera/rgb_camera gain <BEST_GAIN>
rosrun dynamic_reconfigure dynparam set /camera/rgb_camera enable_auto_white_balance false
rosrun dynamic_reconfigure dynparam set /camera/rgb_camera white_balance <BEST_WHITE_BALANCE>
```

如果 namespace 不对：

```bash
rosrun dynamic_reconfigure dynparam list | grep camera
```

## 6. 固定 P2 S 弯路径准备

### 6.1 生成并持续发布固定路径

终端 F：

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

该终端不要关闭。它会持续发布 `/scout/global_path_fixed`。

### 6.2 发送同一个 P2 goal

终端 E：

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

确认 JSON 已生成：

```bash
ls -lh /home/geist/fixed_paths/real/<DATE>/P2_s_curve_d200.json
rostopic hz /scout/global_path_fixed
```

重要：

```text
先生成最终 JSON，再生成 TOPPRA/RUCKIG CSV。
不要先用旧 JSON 生成 CSV，再让 template_fixed_path_generator.py 覆盖 JSON。
```

## 7. TOPPRA / Ruckig 外部速度剖面准备

只在 TOPPRA/RUCKIG 组执行。C/D/E/F 不需要。

### 7.1 TOPPRA-style CSV

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws
mkdir -p /home/geist/fixed_paths/real/<DATE>/baseline_profiles

python3 src/scout_apps/control/scout_local_planner/scripts/analysis/retime_toppra_style.py \
  --path-file /home/geist/fixed_paths/real/<DATE>/P2_s_curve_d200.json \
  --out-csv /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_toppra_style.csv \
  --plot /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_toppra_style.png \
  --v-max 0.80 \
  --a-max 0.60 \
  --decel-max 0.80 \
  --ds 0.02
```

### 7.2 Ruckig-style CSV

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws
mkdir -p /home/geist/fixed_paths/real/<DATE>/baseline_profiles

python3 src/scout_apps/control/scout_local_planner/scripts/analysis/retime_ruckig_style.py \
  --path-file /home/geist/fixed_paths/real/<DATE>/P2_s_curve_d200.json \
  --out-csv /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_ruckig_style.csv \
  --plot /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_ruckig_style.png \
  --v-max 0.80 \
  --a-max 0.60 \
  --j-max 1.50 \
  --delta-time 0.02
```

检查：

```bash
head /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_toppra_style.csv
tail /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_toppra_style.csv

head /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_ruckig_style.csv
tail /home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_ruckig_style.csv
```

若实物机没有 Ruckig 依赖：

```bash
python3 -m pip install --user --only-binary=:all: 'ruckig==0.9.2'
```

如果安装失败，先跳过 RUCKIG，只跑 TOPPRA / C / D / E / F。

## 8. MPC 启动命令

每次只启动一个 MPC 组。切组时停止上一组 `roslaunch scout_local_planner slosh_experiment.launch`，重新启动下一组。

### 8.1 C 组：BASE / ordinary MPC

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_fixed \
  mpc_Q_lag:=0.5 \
  mpc_Q_contour:=28.0 \
  mpc_Q_etheta:=12.0 \
  mpc_Q_v:=3.0 \
  mpc_R_a:=0.5 \
  mpc_R_omega:=2.0 \
  mpc_R_da:=1.5 \
  mpc_R_domega:=6.0 \
  Q_slosh:=0.0 \
  slosh_height_ref:=0.005 \
  slosh_eta_dot_ratio:=0.3 \
  Q_slosh_eta_dot:=0.0 \
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
  v_des_rate_limit_enable:=true \
  v_des_accel_limit:=0.60 \
  v_des_decel_limit:=0.80 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

### 8.2 D 组：SLOSH_ONLY / 论文创新核心

C 组基础上只打开 modal slosh cost：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_fixed \
  mpc_Q_lag:=0.5 \
  mpc_Q_contour:=28.0 \
  mpc_Q_etheta:=12.0 \
  mpc_Q_v:=3.0 \
  mpc_R_a:=0.5 \
  mpc_R_omega:=2.0 \
  mpc_R_da:=1.5 \
  mpc_R_domega:=6.0 \
  Q_slosh:=5.0 \
  slosh_height_ref:=0.005 \
  slosh_eta_dot_ratio:=0.3 \
  Q_slosh_eta_dot:=0.0 \
  terminal_factor_slosh_eta:=5.0 \
  terminal_factor_slosh_eta_dot:=3.0 \
  terminal_slowdown_enable:=true \
  terminal_slowdown_distance:=2.00 \
  terminal_slowdown_v_max:=0.80 \
  terminal_slowdown_Q_v:=40.0 \
  terminal_slowdown_terminal_factor_v:=5.0 \
  terminal_capture_stop_enable:=true \
  terminal_capture_stop_distance:=0.70 \
  terminal_capture_v:=0.18 \
  v_des_rate_limit_enable:=true \
  v_des_accel_limit:=0.60 \
  v_des_decel_limit:=0.80 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

### 8.3 E 组：SMOOTH_ONLY

不启用 slosh，只提高 smooth-control 权重：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

roslaunch scout_local_planner slosh_experiment.launch \
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
  slosh_height_ref:=0.005 \
  slosh_eta_dot_ratio:=0.3 \
  Q_slosh_eta_dot:=0.0 \
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
  v_des_rate_limit_enable:=true \
  v_des_accel_limit:=0.60 \
  v_des_decel_limit:=0.80 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

### 8.4 F 组：OURS_FULL

工程完整主线：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_fixed \
  mpc_Q_lag:=0.5 \
  mpc_Q_contour:=28.0 \
  mpc_Q_etheta:=12.0 \
  mpc_Q_v:=3.0 \
  mpc_R_a:=1.0 \
  mpc_R_omega:=2.0 \
  mpc_R_da:=2.0 \
  mpc_R_domega:=6.0 \
  Q_slosh:=5.0 \
  slosh_height_ref:=0.005 \
  slosh_eta_dot_ratio:=0.3 \
  Q_slosh_eta_dot:=0.0 \
  terminal_factor_slosh_eta:=5.0 \
  terminal_factor_slosh_eta_dot:=3.0 \
  terminal_slowdown_enable:=true \
  terminal_slowdown_distance:=2.00 \
  terminal_slowdown_v_max:=0.80 \
  terminal_slowdown_Q_v:=40.0 \
  terminal_slowdown_terminal_factor_v:=5.0 \
  terminal_capture_stop_enable:=true \
  terminal_capture_stop_distance:=0.70 \
  terminal_capture_v:=0.18 \
  v_des_rate_limit_enable:=true \
  v_des_accel_limit:=0.60 \
  v_des_decel_limit:=0.80 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

### 8.4A F 组 Q_slosh sweep

本节用于确定正式主方法 `F*`。除 `Q_slosh` 之外，保持 8.4 F 组参数完全一致。

建议先每组 1 包 smoke：

```text
F_Q3:  Q_slosh:=3.0
F_Q5:  Q_slosh:=5.0
F_Q7:  Q_slosh:=7.0
F_Q10: Q_slosh:=10.0
```

启动方式：

```text
复制 8.4 F 组命令，只替换 Q_slosh。
```

选择规则：

```text
1. 首看 RGB max-LCR p95/RMS/peak；
2. 再看 ax_p95 / ay_p95 是否同步下降；
3. 再看 model p95/peak 是否方向一致；
4. duration 相对 E 或 F_Q5 明显变长时，按 trade-off 处理；
5. tracking error 明显变差时，该 Q 不作为主方法。
```

推荐保守决策：

```text
如果 Q=7 比 Q=5 略好且 duration 没明显增加，F*=F_Q7；
如果 Q=10 液面最低但明显变慢，F_Q10 只作为 aggressive ablation；
如果 Q=3/5/7 差异很小，保持 F_Q5，避免过度调参。
```

### 8.5 TOPPRA-style baseline

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_fixed \
  external_speed_profile_csv:=/home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_toppra_style.csv \
  external_profile_execution_cap_enable:=true \
  external_profile_execution_accel_limit:=0.60 \
  external_profile_execution_decel_limit:=0.80 \
  external_profile_execution_jerk_limit:=0.0 \
  mpc_Q_lag:=0.5 \
  mpc_Q_contour:=28.0 \
  mpc_Q_etheta:=12.0 \
  mpc_Q_v:=3.0 \
  mpc_R_a:=0.5 \
  mpc_R_omega:=2.0 \
  mpc_R_da:=1.5 \
  mpc_R_domega:=6.0 \
  Q_slosh:=0.0 \
  slosh_height_ref:=0.005 \
  slosh_eta_dot_ratio:=0.3 \
  Q_slosh_eta_dot:=0.0 \
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
  v_des_rate_limit_enable:=true \
  v_des_accel_limit:=0.60 \
  v_des_decel_limit:=0.80 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

### 8.6 Ruckig-style baseline

只替换 TOPPRA 命令中的 CSV 路径：

```bash
external_speed_profile_csv:=/home/geist/fixed_paths/real/<DATE>/baseline_profiles/P2_s_curve_d200_ruckig_style.csv
```

其它参数保持和 TOPPRA-style 一致。

### 8.7 外部 profile 启动后检查

TOPPRA/RUCKIG 启动后必须检查：

```bash
rosparam get /scout_local_planner/path_handler/external_speed_profile_csv
rosparam get /scout_local_planner/external_profile_execution_cap/enable
rostopic echo -n 1 /reference/v_ref_horizon
rostopic echo -n 1 /profile_cap/active
rostopic echo -n 1 /profile_cap/v_profile
rostopic echo -n 1 /profile_cap/cmd_v_post_cap
```

期望：

```text
CSV 路径正确；
external_profile_execution_cap/enable = true；
/reference/v_ref_horizon 的 max 不超过 CSV v-max；
/profile_cap/active 有数据；
/profile_cap/cmd_v_post_cap 不长期高于 /profile_cap/v_profile。
```

### 8.8 TOPPRA/RUCKIG speed-matched 调参

完成 F 组 Q_slosh sweep 后，再做外部 baseline 调参。目标不是削弱 baseline，
而是让外部方法和 F* 在任务效率上可比。

参考目标：

```text
T_profile ∈ [0.9 T_F*, 1.1 T_F*]
优先接近 T_F*；
同时记录 v_p95、ax_p95、ay_p95，防止只靠极低速度赢。
```

TOPPRA 调参顺序：

```text
默认：
  v_max=0.80, a_max=0.60, decel_max=0.80

若比 F* 慢：
  v_max: 0.80 -> 0.90 -> 1.00
  a_max: 0.60 -> 0.80
  decel_max: 0.80 -> 1.00

若比 F* 快且液面明显变差：
  先回到默认，不为了强行匹配而破坏 smooth baseline 语义。
```

Ruckig 调参顺序：

```text
默认：
  v_max=0.80, a_max=0.60, j_max=1.50

若比 F* 慢：
  v_max: 0.80 -> 0.90 -> 1.00
  a_max: 0.60 -> 0.80
  j_max: 1.50 -> 2.00 -> 2.50

若比 F* 快且 jerk/液面明显变差：
  先回到默认，不用过激 profile 冒充 Ruckig-style smooth baseline。
```

正式命名建议：

```text
TOPPRA_default: 默认参数；
TOPPRA_matched: 调到接近 F* duration 的参数；
RUCKIG_default: 默认参数；
RUCKIG_matched: 调到接近 F* duration 的参数。
```

如果实物时间不足：

```text
正文优先使用 matched；
default 放 appendix 或 trade-off 图；
但必须保留 default 参数和 duration，避免被质疑只调了有利 baseline。
```

## 9. 录包命名和顺序

### 9.1 第一轮：F 组 Q_slosh sweep

先跑 F_Q3 / F_Q5 / F_Q7 / F_Q10 各 1 包，用于确定 `F*`：

```text
P2_s_curve_F_Q3_d200_qsweep_run01
P2_s_curve_F_Q5_d200_qsweep_run01
P2_s_curve_F_Q7_d200_qsweep_run01
P2_s_curve_F_Q10_d200_qsweep_run01
```

录包命令模板：

```bash
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh <Q_LABEL> <RUN_SUFFIX>
```

示例：

```bash
# F_Q3
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 3 P2_s_curve_F_Q3_d200_qsweep_run01

# F_Q5
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 5 P2_s_curve_F_Q5_d200_qsweep_run01

# F_Q7
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 7 P2_s_curve_F_Q7_d200_qsweep_run01

# F_Q10
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 10 P2_s_curve_F_Q10_d200_qsweep_run01
```

### 9.2 第二轮 pilot：baseline 接线和 duration fairness

选出 `F*` 后，再跑 C/D/E/F*/TOPPRA/RUCKIG 各 1 包，用于确认 duration 是否公平、terminal 是否干净：

```text
P2_s_curve_C_d200_pilot_run01
P2_s_curve_D_d200_pilot_run01
P2_s_curve_E_d200_pilot_run01
P2_s_curve_Fstar_d200_pilot_run01
P2_s_curve_TOPPRA_default_d200_pilot_run01
P2_s_curve_RUCKIG_default_d200_pilot_run01
```

如果 TOPPRA/RUCKIG 默认参数与 F* 差异超过 ±10%，再补 matched pilot：

```text
P2_s_curve_TOPPRA_matched_d200_pilot_run01
P2_s_curve_RUCKIG_matched_d200_pilot_run01
```

录包示例：

```bash
# C
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 0 P2_s_curve_C_d200_pilot_run01

# F*
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh <FSTAR_Q_LABEL> P2_s_curve_Fstar_d200_pilot_run01

# TOPPRA matched/default
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 0 P2_s_curve_TOPPRA_matched_d200_pilot_run01
```

### 9.3 正式随机 block：每组 n=3

pilot 通过后，正式录包采用 randomized block：

```text
Block 1: C / D / E / F* / TOPPRA_matched / RUCKIG_matched 随机顺序各 1 包
Block 2: C / D / E / F* / TOPPRA_matched / RUCKIG_matched 随机顺序各 1 包
Block 3: C / D / E / F* / TOPPRA_matched / RUCKIG_matched 随机顺序各 1 包
```

命名：

```text
P2_s_curve_C_d200_block01_run01
P2_s_curve_D_d200_block01_run01
P2_s_curve_E_d200_block01_run01
P2_s_curve_Fstar_d200_block01_run01
P2_s_curve_TOPPRA_matched_d200_block01_run01
P2_s_curve_RUCKIG_matched_d200_block01_run01
...
```

每个 block 前后各录一包 static：

```bash
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 0 P2_s_curve_static_block01_before
```

```bash
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 0 P2_s_curve_static_block01_after
```

static 包只用于 RGB jitter / exposure drift 检查，不进入运动效果统计。

## 10. 录后验收

每包必须检查：

```text
1. 能进入 TRACKING；
2. 最终 REACHED；
3. 没有 GOAL_PASSED / turnback；
4. /slosh/state、/slosh/height、/slosh/height_pred_max 持续发布；
5. /mpc/cost_breakdown 有数据；
6. /terminal/mode、/terminal/goal_info 有数据；
7. /reference/v_ref_horizon 有数据；
8. TOPPRA/RUCKIG 的 /profile_cap/* 有数据；
9. RGB topic 和 camera_info 正常；
10. sidecar 文件齐全：topics / rosparam / nodes / info。
```

行为分析：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/analysis/analyze_terminal_approach_1s.py \
  --bag-dir /home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
  --glob "*.bag" \
  --out-dir /home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare/terminal_approach_1s \
  --plot
```

固定路径行为分析：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/analysis/analyze_fixed_path_cost_effect.py \
  --bag-dir /home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare \
  --glob "*.bag" \
  --exclude-static \
  --out-dir /home/geist/slosh_bags/real/<DATE>_sloshpriority_baseline_compare/fixed_path_behavior
```

## 11. 分析口径

正式分析按 `docs/重要文档/红色液体视觉验证固定流程.md` 执行。

主表指标：

```text
RGB max-LCR p95
RGB max-LCR RMS
RGB max-LCR peak
model p95
model peak
ax_p95
ay_p95 = p95(|v * omega|)
jerk_p95
tracking duration
tracking error
completion time
```

模型保真度：

```text
gamma_model = 100 * integral |h_model - h_RGB| dt / integral |h_model| dt
Pearson / Spearman
RMSE
model-vs-RGB p95/peak scatter
```

核心对比：

```text
C vs D:
  证明 modal slosh cost 单独有效。

E vs F:
  证明同等 smooth-control 框架下 slosh cost 有额外收益。

F* vs TOPPRA/RUCKIG:
  做 duration / tracking-error trade-off 对比。
```

## 12. 进入正式统计的条件

Q_slosh sweep 通过条件：

```text
1. F_Q3 / F_Q5 / F_Q7 / F_Q10 都能完成固定路径并 REACHED；
2. terminal 不污染主窗口；
3. RGB max-LCR 检测链可用；
4. 至少一个 Q 在 RGB p95/RMS/peak 上不差于 F_Q5；
5. 选出的 F* 不以明显 duration 变长或 tracking error 变差为代价。
```

baseline pilot 通过条件：

```text
1. 六组都能完成固定路径并 REACHED；
2. terminal 不污染主窗口；
3. RGB 检测链可用，max-LCR 不是全 0；
4. F* / TOPPRA / RUCKIG 的 duration 不差太多；
5. TOPPRA/RUCKIG 的外部 profile 确实生效。
```

如果 TOPPRA/RUCKIG 比 F 慢超过 10%：

```text
先调 profile 参数，不扩展 n=3。

TOPPRA 可调：
  v_max
  a_max
  decel_max

RUCKIG 可调：
  v_max
  a_max
  j_max
```

如果 F 仍明显输给 time-matched TOPPRA/RUCKIG：

```text
不要硬写 ours 更强；
改写论文主张：
  D 证明 modal slosh objective 有效；
  F* 是同框架 slosh-aware full controller；
  TOPPRA/RUCKIG 是强 smooth-motion baseline；
  我们的方法提供液体状态可解释性和反馈式 MPC 集成，
  但在该 P2 路径上还需进一步优化效率-防晃 Pareto trade-off。
```
