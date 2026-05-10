# 2026-05-10 phase5 长路径同 session 录包计划

## 0. 录前参数 sanity（独立 session，必做）

phase5 主 session 全程 ~2.5 h 同液位同标定；若 OSCRS smoke `fb=2` 现场调参，期间 30-60 min 液位漂会污染后续 RAW/FIXED 的同液位前提。**先在另一次 session 锁定 OSCRS 能 takeover 的参数**，phase5 主 session 直接用锁定值，不再现场决策。

**前置事实**：phase4 短路径通过的参数组 (`ay=3.0, residual=0.2`) 不保证长路径够：

```text
phase3 长路径 medium ayr ∈ {3.08, 3.45}    → ay_ratio_limit=3.0 几率被拦
长路径残振时间更长                           → residual_ratio=0.2 几率被拦
```

### 0.1 工作流

全部命令复用 phase5 后续章节，**不重写**：

```text
1. 起链路 + 定位 smoke + 静止包 + 视觉标定          §4.2 → §4.5
2. OSCRS_MEDIUM smoke × 1                            §5.3 命令, RUN_ID=sanity01
3. validate                                          §5.3 末段 oscrs --require-non-original
4. 决策按 §11.3:
   A 档 (selected ∈ {mid, medium}, fb=0, takeover=1)  → 锁定, 跳 §0.2
   B 档 (selected=mild, fb=0)                          → 锁定 (论文标注), 跳 §0.2
   C 档 (fb=2/3 或 safety_alarm)                       → 改一参重 smoke
        最多 3 轮:  ay 3.0 → 3.5  →  residual 0.2 → 0.5  →  停
        3 轮全失败 → §0.3
```

每次重 smoke 用新 `RUN_ID=sanity02 / sanity03 / ...`，不要覆盖。

### 0.2 锁定参数输出

通过后落地：

```text
docs/Claude/分析数据/phase5_params_sanity_<DATE>.md
```

格式：

```yaml
locked_params:
  max_candidate_level:   medium
  ay_ratio_limit:        <最终锁定值>     # default 3.0 或调到 3.5
  oscrs_residual_ratio:  <最终锁定值>     # default 0.2 或调到 0.5
  oscrs_eta_lim_mm:      25               # 永远不动
  prediction_v_max:      2.0              # 永远不动
verified_by:
  bag:         slosh_Q0_<TS>_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_sanity0X.bag
  fb:          0
  takeover:    1
  selected:    <mid|medium|mild>
  archetype:   <A|B>
session_date:  <YYYY-MM-DD>
tuning_path:   <例: "ay 3.0→3.5 (sanity01 fb=2 ay reject) → 通过 (sanity02)">
```

**phase5 §5.3 GEOREF_OSCRS_MEDIUM_ACTIVE_REAL smoke 的 launch 命令中 `ay_ratio_limit` 和 `oscrs_residual_ratio` 必须用此 yaml 锁定值填入**，否则 sanity 工作白做。

### 0.3 三轮全失败的处置

```text
- OSCRS 在该长路径无可行参数 → 停 OSCRS 主线
  phase5 缩为 RAW + FIXED × 3 两组对比 (仍有论文价值)
- 不要把 oscrs_eta_lim_mm 调高 — 破坏论文证据链
- 不要降低 prediction_v_max — rollout 与 MPC 口径必须一致
```

输出仍写 sanity 报告，标记 `locked_params: null` + `oscrs_disabled_reason: ...`。

### 0.4 sanity 与 phase5 的边界

```text
sanity session   ≥ 1 天前
                 标定 / 液位 / 静止包 不复用到 phase5
                 输出 locked_params.yaml + 一行调参轨迹
                 sanity bag 不进 phase5 主表数据

phase5 主 session  另一天
                  重新标定 / 重新加注液位
                  §5.3 launch 用锁定参数
                  正常情况不应再触发 §11.3 fb=2/3 决策
```

---

## 1. 目标

在 phase3 同样的 14 m 长路径（含弯）上，**同一 session 内**录得 `RAW_REAL` / `GEOREF_FIXED_MILD_REAL` / `GEOREF_OSCRS_MEDIUM_ACTIVE_REAL` 三组对比数据，达到：

```text
- SOP §11.1 单包准入全部通过；
- SOP §11.2 数据量 N=3 起步，CV<0.3 即足够；
- SOP §11.3 主效果判据（h_p95 ≥ 10% 下降 + ay 不上升 + tracking ≤ ×1.15）；
- 视觉真值与 /slosh/height 方向一致（A_rank ≥ 0.80 推荐）。
```

执行口径与判据继承 `20260509_phase3实物GeoRef修复与复录SOP.md`（**下文中所有 "SOP §X.Y" 均指此文件对应章节**），本文件只补 phase5 特有的"长路径 + 同 session"约束。

---

## 2. 决策：同 session 重录而非复用 phase3

phase3 的 RAW + FIXED 长路径数据已通过 validate，但**不进入 phase5 主表**。原因：

```text
液位漂      phase3 录于 5/9 18:51,  phase5 间隔 ≥ 1 天,  液面 mm 级可能漂；
            sub-mm 信号 ablation 跨 session 时 RAW vs OSCRS 差被液位差污染。
标定漂      RealSense 视角 / 三标尺 ROI / HSV 阈值 重新搭场地后必须重标。
定位漂      AMCL 在不同 session 同一坐标可能落在物理上不同位置。
方法学      论文审稿不接受跨 session 拼数据。
```

phase3 数据保留作历史诊断；本次 phase5 录制的三组共用同一标定、同一定位、同一液位记录。

---

## 3. 路径与 session 参数

```text
起点 / 终点：    与 phase3 相同
                start ≈ (7.65, 4.80)
                goal  = (2.03, 17.13), yaw = 175.1°
路径长度：       global_path ≈ 14.65 m, 直线 ≈ 13.55 m, 曲率指数 ≈ 1.08
N：              3 起步 (run01/run02/run03)；CV ≥ 0.3 升 N=5
顺序：           A/B 反向序对冲液位单调漂
                run01:  RAW → FIXED → OSCRS_MEDIUM
                run02:  OSCRS_MEDIUM → FIXED → RAW
                run03:  RAW → FIXED → OSCRS_MEDIUM
```

---

## 4. 录前 checklist

所有终端先执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
```

### 4.1 容器与液位（T-30 min）

```text
1. 容器加注到目标液位（与 phase4 一致的 Q0 mark）；
2. 用 RealSense 拍标定帧 P0_calib.png，记录初始液位 h0_visual；
3. 容器 / 相机锁紧；后续不能松动直到 session 结束。
```

### 4.2 起链路（T-20 min）

每个步骤独立终端，**起完一个再起下一个**。

#### CAN 与底盘

```bash
sudo modprobe gs_usb
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
roslaunch scout_bringup scout_mini_robot_base.launch
```

#### 激光雷达

```bash
roslaunch nanoscan3_bringup nanoscan3_front.launch use_rviz:=false
```

检查：

```bash
rostopic hz /scan_front
```

#### 定位（按当前地图二选一）

```bash
roslaunch nanoscan3_localization scout_nanoscan3_amcl.launch use_rviz:=true
```

或：

```bash
roslaunch nanoscan3_localization scout_nanoscan3_cartographer_localization.launch
```

#### IMU

```bash
roslaunch scout_bringup scout_imu_with_tf.launch
```

检查：

```bash
rostopic hz /imu/data
rostopic echo -n1 /imu/data
```

#### RealSense RGB

```bash
source /home/geist/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/realsense_ros_env_local.sh
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

检查：

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/depth/image_rect_raw
```

#### MBF 全局规划

```bash
roslaunch scout_global_planner mbf_global.launch
```

未发目标前 `/scout/global_path` 没消息正常。

### 4.3 定位 smoke（T-15 min）

按 `20260509_phase3实物GeoRef修复与复录SOP.md §4` 量化判据：

```bash
# 静止 10 s 漂移
rosrun tf tf_echo map base_link

# AMCL 协方差
rostopic echo -n 1 /amcl_pose | grep -A6 covariance
```

判据：

```text
静止 10 s   xy 漂 < 0.05 m, yaw 漂 < 1°
amcl_pose   cov[0,0] < 0.05, cov[5,5] < 0.005
```

不通过则停 phase5，先修定位。

### 4.4 静止包（T-10 min）

车不动 30 s，启动 OSCRS 完整 stack 但不发 goal。

终端 A：post-processor。

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  oscrs_config:=/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  fixed_candidate_name:= \
  max_candidate_level:=medium \
  ay_ratio_limit:=3.0 \
  collision_threshold:=90 \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  oscrs_shadow_enable:=true \
  oscrs_active_enable:=true \
  oscrs_eta_lim_mm:=25 \
  oscrs_residual_ratio:=0.2
```

终端 B：MPC。

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
  slosh_use_imu_lateral_accel:=false
```

终端 C：录包 30 s 不发 goal。

```bash
cd /home/geist/scout_ws/src/scout_apps/control/scout_local_planner
SLOSH_BAG_DIR=/data/$USER/slosh_bags/real/20260510_phase5 \
CONDITION=GEOREF_OSCRS_MEDIUM_ACTIVE_REAL RUN_ID=static \
  ./scripts/record_slosh_experiment.sh 0
```

作用：

```text
- IMU bias 评估窗口；
- 视觉零点 h0 估计窗口；
- HSV 静态采样备用帧。
```

录完后关闭终端 A、B（goal 发完后才需要再启）。

### 4.5 视觉三标尺标定 + HSV 采样（T-5 min）

按 `红色液体视觉验证固定流程.md §3 / §4`。

#### 三标尺标定

```bash
mkdir -p /home/geist/scout_ws/docs/Claude/分析数据/phase5_visual_20260510

python3 /home/geist/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_calibrate.py \
  --image /home/geist/scout_ws/docs/Claude/分析数据/phase5_visual_20260510/P0_calib.png \
  --ruler-heights-mm 0,2,4,6,8 \
  --output-yaml /home/geist/scout_ws/docs/Claude/分析数据/phase5_visual_20260510/calib.yaml \
  --output-image /home/geist/scout_ws/docs/Claude/分析数据/phase5_visual_20260510/calib_preview.png
```

点击顺序见 `红色液体视觉验证固定流程.md §3`。验收：preview 中三列刻度贴合真实标尺。

#### HSV 阈值采样

先从静止包提取一帧动态峰值帧（若无动态可只用静止帧）：

```bash
python3 /home/geist/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py \
  --bag /data/$USER/slosh_bags/real/20260510_phase5/slosh_Q0_*_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_static.bag \
  --image-topic /camera/color/image_raw \
  --out-dir /home/geist/scout_ws/docs/Claude/分析数据/phase5_visual_20260510/hsv_frames \
  --frame-index 0
```

采样：

```bash
python3 /home/geist/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_sample_hsv.py \
  --images /home/geist/scout_ws/docs/Claude/分析数据/phase5_visual_20260510/P0_calib.png \
           /home/geist/scout_ws/docs/Claude/分析数据/phase5_visual_20260510/hsv_frames/frame_*.png \
  --calibration /home/geist/scout_ws/docs/Claude/分析数据/phase5_visual_20260510/calib.yaml
```

把输出的 `--hue1-low ... --val-min ...` 这串参数写入：

```text
/home/geist/scout_ws/docs/Claude/分析数据/phase5_visual_20260510/hsv_params.txt
```

---

## 5. smoke 录制命令（每个 condition 一包）

正式 N=3 之前先录一轮 smoke。**录制顺序 `RAW → FIXED → OSCRS_MEDIUM`**。每包录完立即 validate，通过再录下一包。

每个条件都用 4 个终端：

```text
终端 A   post-processor (RAW 不起)
终端 B   slosh_experiment.launch (MPC)
终端 C   send_fixed_goal.py (发同一终点)
终端 D   record_slosh_experiment.sh (录包)
```

### 5.1 RAW_REAL smoke

终端 A：不起 post-processor。

终端 B：MPC，订阅原始全局路径。

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
  slosh_use_imu_lateral_accel:=false
```

终端 C：发同一终点（与 phase3 相同）。

```bash
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic /scout/goal \
  --frame map \
  --x 2.026 \
  --y 17.125 \
  --yaw 175.1 \
  --repeat-count 30 \
  --repeat-rate 5 \
  --wait-subscriber-timeout 20
```

终端 D：录包。

```bash
cd /home/geist/scout_ws/src/scout_apps/control/scout_local_planner
SLOSH_BAG_DIR=/data/$USER/slosh_bags/real/20260510_phase5 \
CONDITION=RAW_REAL RUN_ID=smoke \
  ./scripts/record_slosh_experiment.sh 0
```

录完立即验收：

```bash
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/validate_georef_oscrs_bag.py \
  /data/$USER/slosh_bags/real/20260510_phase5/slosh_Q0_*_RAW_REAL_smoke.bag \
  --mode raw
```

通过条件：

```text
VERDICT=PASS
reports=0
global_path_anti_slosh=0
safety_alarm=0
```

### 5.2 GEOREF_FIXED_MILD_REAL smoke

终端 A：post-processor (fixed mild)。

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  oscrs_config:=/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  fixed_candidate_name:=mild \
  max_candidate_level:=mild \
  ay_ratio_limit:=3.0 \
  collision_threshold:=90 \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  oscrs_shadow_enable:=false \
  oscrs_active_enable:=false
```

终端 B：MPC，订阅 anti_slosh path。

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
  slosh_use_imu_lateral_accel:=false
```

终端 C：同 §5.1 send_fixed_goal 命令。

终端 D：

```bash
cd /home/geist/scout_ws/src/scout_apps/control/scout_local_planner
SLOSH_BAG_DIR=/data/$USER/slosh_bags/real/20260510_phase5 \
CONDITION=GEOREF_FIXED_MILD_REAL RUN_ID=smoke \
  ./scripts/record_slosh_experiment.sh 0
```

录完立即验收：

```bash
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/validate_georef_oscrs_bag.py \
  /data/$USER/slosh_bags/real/20260510_phase5/slosh_Q0_*_GEOREF_FIXED_MILD_REAL_smoke.bag \
  --mode fixed --require-non-original
```

通过条件：

```text
VERDICT=PASS
selected=mild
safety_alarm=0
global_path_anti_slosh=1
```

**FIXED smoke 录完必须跑 yaw 回归 smoke**：

```bash
python3 - <<'PY'
import rosbag, sys, glob
bag = glob.glob("/data/$USER/slosh_bags/real/20260510_phase5/slosh_Q0_*_GEOREF_FIXED_MILD_REAL_smoke.bag")[0]
o, a = None, None
with rosbag.Bag(bag) as b:
    for t,m,_ in b.read_messages(["/scout/global_path","/scout/global_path_anti_slosh"]):
        if t == "/scout/global_path" and o is None: o = m.poses[-1].pose.orientation
        if t == "/scout/global_path_anti_slosh" and a is None: a = m.poses[-1].pose.orientation
def eq(p,q): return (p.x,p.y,p.z,p.w) == (q.x,q.y,q.z,q.w)
print("EQUAL =", eq(o,a))
sys.exit(0 if eq(o,a) else 1)
PY
```

`EQUAL = True` 才能进 §5.3；否则停 phase5 排查 yaw 修复。

### 5.3 GEOREF_OSCRS_MEDIUM_ACTIVE_REAL smoke

**重要**：下面命令中的 `ay_ratio_limit` 和 `oscrs_residual_ratio` 是 phase4 同值默认。phase5 主 session 必须**使用 §0 sanity session 输出的 `locked_params.yaml` 中锁定值替换**；若 sanity 没有调过参，保持下面默认。

终端 A：post-processor (OSCRS active, max_candidate_level=medium)。

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  oscrs_config:=/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  fixed_candidate_name:= \
  max_candidate_level:=medium \
  ay_ratio_limit:=3.0 \
  collision_threshold:=90 \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  oscrs_shadow_enable:=true \
  oscrs_active_enable:=true \
  oscrs_eta_lim_mm:=25 \
  oscrs_residual_ratio:=0.2
```

终端 B：MPC（同 §5.2 命令）。

终端 C：同 §5.1 send_fixed_goal。

终端 D：

```bash
cd /home/geist/scout_ws/src/scout_apps/control/scout_local_planner
SLOSH_BAG_DIR=/data/$USER/slosh_bags/real/20260510_phase5 \
CONDITION=GEOREF_OSCRS_MEDIUM_ACTIVE_REAL RUN_ID=smoke \
  ./scripts/record_slosh_experiment.sh 0
```

录完立即验收：

```bash
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/validate_georef_oscrs_bag.py \
  /data/$USER/slosh_bags/real/20260510_phase5/slosh_Q0_*_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_smoke.bag \
  --mode oscrs --require-non-original
```

OSCRS smoke 结果分级（继承 SOP §5.1 / §7.3）：

```text
A 档（理想）  selected ∈ {mid, medium}, fb=0, takeover=1, safety_alarm=0
              → 进 §7 正式批量

B 档（接受）  selected=mild, fb=0, safety_alarm=0
              → 进批量但论文里标注"OSCRS 选择倾向 mild"

C 档（停录）  fb=2 / safety_alarm=1 / 现场终点过冲
              → 看 candidate_report 里 medium/mid 的 reject reason，
                按 §6 fallback 顺序改一参重 smoke
```

---

## 6. 长路径专属风险与 fallback

phase4 短路径下 OSCRS_MEDIUM 配置已通；长路径含弯时**两个 gate 可能新拦截**。预备应对（按 SOP §9 fallback 顺序）：

```text
风险 1  ay_ratio_limit=3.0 不够
   依据 phase3 实测 medium ayr ∈ {3.078, 3.454}，长路径含弯 ay 更高；
   smoke 出现 medium reject reason=ay 时:
   ay_ratio_limit:=3.0 → 3.5

风险 2  residual_ratio=0.2 不够
   长路径残振更长, medium sHr 可能超 0.2 × eta_lim = 5 mm；
   smoke 出现 medium reject reason=residual 时:
   oscrs_residual_ratio:=0.2 → 0.5

不要先动 eta_lim_mm
   eta_lim 的物理基础不让步给"让 OSCRS takeover"。
   只有 SOP §9.5 RGB 经验标定明确支持时才能换默认 25。

不要降低 prediction_v_max
   prediction_v_max 必须等于 MPC cmd_vel.linear.x_max。
```

OSCRS smoke `fb=2` 时按上述顺序**一次只动一参**，重 smoke 验证。

---

## 7. 正式批量

smoke 全部 PASS（OSCRS 至少 B 档）后录 run01/run02/run03，每个 run 三包按 §3 节 A/B 反向序。命令复用 §5 各小节，**只需改 `RUN_ID`**。

### 7.1 录制循环

```text
run01:  RUN_ID=run01
  §5.1 RAW_REAL          → validate raw                  → PASS 才继续
  §5.2 GEOREF_FIXED_MILD → validate fixed --require-non-original → PASS
  §5.3 GEOREF_OSCRS_MEDIUM → validate oscrs --require-non-original → A/B 档

run02:  RUN_ID=run02       (反向序)
  §5.3 GEOREF_OSCRS_MEDIUM
  §5.2 GEOREF_FIXED_MILD
  §5.1 RAW_REAL

run03:  RUN_ID=run03       (正向序，同 run01)
  §5.1 → §5.2 → §5.3
```

每包录完立刻跑 validate + extract_slosh_metrics：

```bash
# 行为验收（按 condition 选 mode）
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/validate_georef_oscrs_bag.py \
  /data/$USER/slosh_bags/real/20260510_phase5/<BAG>.bag \
  --mode <raw|fixed|oscrs> --require-non-original

# 模型侧效果指标
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  /data/$USER/slosh_bags/real/20260510_phase5/<BAG>.bag
```

任一包 SOP §11.1 失败立刻判定根因 → 重录该 condition 一包**不顶替**。不要"先录满再分析"。

### 7.2 录后静止包（T+90 min）

```bash
# 终端 A、B 起 §4.4 同样的 OSCRS 配置
# 终端 C 不发 goal
cd /home/geist/scout_ws/src/scout_apps/control/scout_local_planner
SLOSH_BAG_DIR=/data/$USER/slosh_bags/real/20260510_phase5 \
CONDITION=GEOREF_OSCRS_MEDIUM_ACTIVE_REAL RUN_ID=static_end \
  ./scripts/record_slosh_experiment.sh 0
```

对比初始静止包检查液位 / IMU bias 是否漂：

```bash
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  /data/$USER/slosh_bags/real/20260510_phase5/slosh_Q0_*_static.bag
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  /data/$USER/slosh_bags/real/20260510_phase5/slosh_Q0_*_static_end.bag
```

若初末液位漂 > 1 mm，标记本 session 数据可能不可信。

---

## 8. 录后处理流程

下面命令在分析机（`/home/a/scout_ws`）上执行；bag 路径替换为分析机能访问的绝对路径。

### 8.1 行为验收 + 模型侧指标（每包）

```bash
DIR=/data/a/slosh_bags/real/20260510_phase5

# 全部 RAW
for f in $DIR/*RAW_REAL*.bag; do
  python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/validate_georef_oscrs_bag.py \
    "$f" --mode raw
  python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py "$f"
done

# 全部 FIXED
for f in $DIR/*GEOREF_FIXED*.bag; do
  python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/validate_georef_oscrs_bag.py \
    "$f" --mode fixed --require-non-original
  python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py "$f"
done

# 全部 OSCRS_MEDIUM
for f in $DIR/*OSCRS_MEDIUM*.bag; do
  [[ "$f" == *static* ]] && continue
  python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/validate_georef_oscrs_bag.py \
    "$f" --mode oscrs --require-non-original
  python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py "$f"
done
```

### 8.2 视觉离线提取（每包）

把 §4.5 输出的 HSV 参数读出来：

```bash
HSV_ARGS=$(cat /home/a/scout_ws/docs/Claude/分析数据/phase5_visual_20260510/hsv_params.txt)
CALIB=/home/a/scout_ws/docs/Claude/分析数据/phase5_visual_20260510/calib.yaml
DEBUG_ROOT=/data/a/slosh_bags/real/20260510_phase5/phase5_red_visual_debug_20260510
DIR=/data/a/slosh_bags/real/20260510_phase5

for f in $DIR/*RAW_REAL*.bag $DIR/*GEOREF_FIXED*.bag $DIR/*OSCRS_MEDIUM*.bag; do
  [[ "$f" == *static* ]] && continue
  name=$(basename "$f" .bag)
  python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py \
    --bag "$f" \
    --calibration "$CALIB" \
    --out-dir "$DEBUG_ROOT/$name" \
    --topic /camera/color/image_raw \
    --zero-correction-frames 30 \
    --smooth-frames 5 \
    --debug-every 30 \
    $HSV_ARGS
done
```

每包输出 `<name>_red_top.csv`（含 `h_smooth_corr` 等列）。

### 8.3 Ferrari-style 保真度 + A_rank

按 `OSCRS模型保真度指标整理.md §7.4 / §7.5` 算：

```bash
OUT=/home/a/scout_ws/docs/Claude/分析数据/phase5_visual_20260510

# 单包 Ferrari fidelity → model_fidelity_summary.csv
# OSCRS pairwise selection fidelity → model_selection_fidelity.csv

# 若 phase4 有现成脚本，沿用：
ls /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/*fidelity*.py 2>/dev/null
# 没有则按 §7.4 公式（trapz + p95 + corr）写一段 ad hoc 脚本，
# 必含 sign_eps = 0.05 mm 死区（§4 OSCRS模型保真度指标整理.md）。
```

输出文件：

```text
$OUT/model_fidelity_summary.csv             每包一行: gamma_model_pct/RMSE/corr/U_p95/U_max
$OUT/model_selection_fidelity.csv           每 pair 一行: visual_diff/model_diff/sign_match
$OUT/A_rank.txt                              单数: matched/total
```

### 8.4 视觉指标汇总 + 报告

```bash
OUT=/home/a/scout_ws/docs/Claude/分析数据/phase5_visual_20260510

# 0424 风格视觉指标汇总（每 run 每 condition 一行）
# 输出：smooth_peak / smooth_p95 / smooth_rms / max_lcr / valid_ratio / clipped_ratio / slosh_p95
$OUT/phase5_visual_metric_summary_0424style.csv

# Run 内差值（每 run 三 pair）
# 输出：d_peak / d_p95 / d_rms / d_max_lcr / d_slosh_p95
$OUT/phase5_runwise_effect_summary_0424style.csv

# 总报告
$OUT/ANALYSIS_REPORT_2026-05-10.md
```

报告结构按 `红色液体视觉验证固定流程.md §9`：

```text
1. 数据分组与标定说明
2. HSV 参数
3. 视觉指标汇总表
4. Run 内差值表
5. tracking_start 三图索引
6. trajectory_analysis 辅证
7. 视觉结论
8. 模型保真度与 /slosh/height 可信度判断（按 §8.7 三层）
```

---

## 9. 成功标准

phase5 视为成功当且仅当：

```text
工程链  9 包正式 + 3 包 smoke 全部 SOP §11.1 单包准入通过；
        OSCRS 三包至少 N=3 全部 A 档（理想）或 ≥ 2 包 A 档（最低）；
        yaw 回归 smoke EQUAL = True；
        录前后静止包液位漂 ≤ 1 mm。

模型侧  OSCRS_MEDIUM 相对 RAW：
        h_p95 下降 ≥ 10%，modal_energy_rms 下降 ≥ 10%，
        eta_dot 不上升，ay_p95 不上升，
        tracking_time ≤ RAW × 1.15，solve_success ≥ 0.97。

视觉    smooth_p95 / smooth_peak / smooth_rms 在 ≥ 2/3 个 run 上方向 = OSCRS < RAW；
        A_rank ≥ 0.80（推荐）或 ≥ 0.50（最低底线）。

可宣称  按 SOP §11.4 主结论：
        "OSCRS 在 Scout Mini 实物长路径上能从 GeoRef 候选集合中选择低晃参考路径，
         相对 RAW 显著降低液面峰值高度和模态能量。"
```

视觉部分项不达 ≥ 0.80：按 SOP §11.4 inconclusive 模板写，给出未达项与可能成因，不掩盖。

---

## 10. 不要做的事

继承 `20260509_phase3实物GeoRef修复与复录SOP.md §10`：

```text
1. 不要为让 selector 接管而改"禁止选 original"；
2. 不要把放宽 eta_lim_mm 当作长路径 OSCRS 第一攻击向量；
3. 不要降低 prediction_v_max；
4. 不要打开 slosh_speed_governor 修终点过冲；
5. 不要在定位漂时录正式包；
6. 不要复用 phase4 标定 yaml；
7. 不要中途补加液体（让液位自然漂 1-2 mm 是 phase5 的预期工况）。
```

---

## 11. 现场快速决策卡（validate FAIL → 改什么）

每包录完跑 validate，**只看下面 4 行**就够了：

```text
VERDICT=                       PASS / FAIL
last=selected=...,active=...,fb=...,takeover=... ,safety_alarm=...
reject_reason_counts={...}     候选被哪些 gate 拒绝
last_candidate_reasons=...     每条候选 accepted=? reason=?
```

后面表格的"处理"列里 `[现场可改]` 表示在 launch 命令里改一行参数重 smoke；`[停录返回]` 表示当前条件不再录，等回来后处理。**一次只动一参**，重 smoke 验证再决定下一步。

### 11.1 RAW_REAL FAIL

| validate 关键行 | 判定 | 处理 |
|---|---|---|
| `FAIL: missing /scout/goal` | goal 没发出 | [现场可改] 重启 §5.1 终端 C 的 send_fixed_goal.py，查 `--wait-subscriber-timeout` 报错 |
| `FAIL: missing /scout/global_path` | MBF 没规划 | [现场可改] 检查 §4.3 定位 smoke 通不通；通则手动 RViz 看起点是否在地图内 |
| `WARN: RAW bag contains candidate_report` | post-processor 误启 | [现场可改] 关掉 §4.4 残留的 `anti_slosh_path_post_processor.launch` 终端，重录 |

### 11.2 GEOREF_FIXED_MILD_REAL FAIL

看 `last_candidate_reasons` 里 `mild` 那条的 `accepted` 与 `reason`。

| 症状 | 处理 |
|---|---|
| `FAIL: missing /scout/global_path_anti_slosh` | [现场可改] post-processor 没启：重起 §5.2 终端 A 的 launch |
| `mild:accepted=0,reason=collision:idx=...` | [现场可改] launch A 改 `collision_threshold:=100` |
| `mild:accepted=0,reason=ay:...` | [现场可改] launch A 改 `ay_ratio_limit:=3.5` |
| `mild:accepted=0,reason=drift` 或 `endpoint` 或 `direction` | [现场可改] launch A 改 `mild_max_drift:=0.05`（从 0.08 调小） |
| `mild:accepted=1` 但 `selected=original` | [停录返回] 不该在 FIXED 模式发生（fixed_candidate_name=mild 强制选中），可能是 launch 误传；检查 `fixed_candidate_name` 参数 |
| `WARN: safety_alarm messages=1` | [停录返回] FIXED 不应有 OSCRS active；查 launch 是否误传 `oscrs_active_enable:=true` |

### 11.3 GEOREF_OSCRS_MEDIUM_ACTIVE_REAL FAIL

按 `fb` 值分支：

#### fb=0, selected=mild (B 档可接受)

```text
不是 FAIL。validate 在 --require-non-original 下也会 PASS。
进入正式批量，论文里标注"OSCRS 选择倾向 mild"。
不要现场改参数。
```

#### fb=2（几何可行，slosh hard gate 全失败）

看 `last_candidate_reasons` 里 `medium` 和 `mid` 的 `accepted` 与 `reason`：

| medium/mid 状态 | 处理 |
|---|---|
| `accepted=0,reason=ay:...` | [现场可改] launch A 改 `ay_ratio_limit:=3.5` |
| `accepted=0,reason=collision:idx=...` | [现场可改] 先做 §4.3 定位 smoke；通过后 launch A 改 `collision_threshold:=100` |
| `accepted=1` 但 `os=0`（OSCRS hard gate reject） | 看 reason：含 `residual` 或 `sHr` → [现场可改] launch A 改 `oscrs_residual_ratio:=0.5` |
| `accepted=1,os=0` 且 reason 不含 residual：sH 真的超 eta_lim | [停录返回] 这是物理超限，需走 SOP §9.5 RGB 标定，现场不能做。OSCRS 主线停，RAW+FIXED 继续 |

#### fb=3（几何全失败）

| 主导 reject reason | 处理 |
|---|---|
| 全 `ay` | [现场可改] launch A 改 `ay_ratio_limit:=3.5` |
| 全 `collision` | [现场可改] 先 §4.3 定位 smoke；通过则 `collision_threshold:=100` |
| `frame_mismatch` 或 `no_costmap` | [停录返回] costmap topic 错：查 `costmap_topic` 与 MBF 实发 topic 是否一致 |

#### `WARN: safety_alarm messages=1`

```text
看 fb 走对应分支。safety_alarm 本身不指示具体 gate，
只表明 OSCRS 进入 fallback。
```

### 11.4 通用 FAIL（不在 condition 内）

| 症状 | 处理 |
|---|---|
| §3.1 yaw 回归 smoke `EQUAL = False` | [停录返回] 停 phase5 全部录制，回来跑 `git diff anti_slosh_path_post_processor.py` |
| 录制中途 RViz scan-map 明显不重合 | [现场可改] 立刻停当前包，重做 §4.3 定位 smoke；该包标注废 |
| 录后静止包液位漂 > 1 mm | [可继续录] 不立刻重录，分析时标记本 session 数据可疑 |
| smoke 三档全失败（A/B/C 档都没进） | [停录返回] 不是常见症状，回来一起诊断 |

### 11.5 何时停 phase5 整体

下表任一触发就停录所有后续条件，等回来：

```text
§3.1 yaw 回归 smoke 失败                              代码回归
§4.3 定位 smoke 三次都不通过                          硬件/地图问题
OSCRS fb=2 试过 ay_ratio_limit:=3.5 + residual_ratio:=0.5 仍失败  物理超限
现场明显终点过冲（车撞墙 / 急刹 / 回头 > 30 cm）       candidate 平滑过强
两次连续重 smoke 都出现 safety_alarm                  系统性问题
```

**RAW + FIXED 即使 OSCRS 主线停了仍可继续录满 N=3**——长路径 RAW vs FIXED 的对比本身就有论文价值。

### 11.6 不要做的事（现场版）

```text
1. 不要"再录一包看运气"。validate FAIL 必须先改一参数再录。
2. 不要同时改两个参数（无法判断哪个起的作用）。
3. 不要把 oscrs_eta_lim_mm 从 25 调高来"救" OSCRS。
   eta_lim 是物理硬门，调高就破坏论文证据链。
4. 不要把 prediction_v_max 调低于 MPC 真实 v_max（=2.0）。
   rollout 与执行口径必须一致。
5. 不要为通过 require-non-original 强行换 fixed_candidate_name 到更激进档位。
   FIXED 主线就是 mild。
6. 不要对一个 condition 录超过 5 包试图凑均值。
   SOP §11.2 N=5 是上限；超过说明本次 session 不稳定，下次重做。
```

---

## 12. 文件清单

phase5 结束时下列文件应齐：

```text
/data/a/slosh_bags/real/20260510_phase5/
  slosh_Q0_<TS>_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_static.bag
  slosh_Q0_<TS>_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_static_end.bag
  slosh_Q0_<TS>_RAW_REAL_smoke.bag
  slosh_Q0_<TS>_RAW_REAL_run01.bag .. run03.bag
  slosh_Q0_<TS>_GEOREF_FIXED_MILD_REAL_smoke.bag
  slosh_Q0_<TS>_GEOREF_FIXED_MILD_REAL_run01.bag .. run03.bag
  slosh_Q0_<TS>_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_smoke.bag
  slosh_Q0_<TS>_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_run01.bag .. run03.bag

  phase5_red_visual_debug_20260510/
    <每包一目录>/
      *_red_top.csv
      visual_compare.png
      visual_max_compare.png
      slosh_height_compare.png

docs/Claude/分析数据/phase5_visual_20260510/
  P0_calib.png
  calib.yaml
  calib_preview.png
  hsv_frames/
  hsv_params.txt
  phase5_visual_metric_summary_0424style.csv
  phase5_runwise_effect_summary_0424style.csv
  model_fidelity_summary.csv
  model_selection_fidelity.csv
  A_rank.txt
  ANALYSIS_REPORT_2026-05-10.md
```
