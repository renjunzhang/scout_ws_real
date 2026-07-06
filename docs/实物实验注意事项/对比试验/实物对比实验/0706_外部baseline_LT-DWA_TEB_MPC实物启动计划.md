# 0706 外部 baseline：LT-DWA / TEB / mpc_local_planner 实物启动计划

> 目的：补充论文需要的外部对比算法实物证据。本文只计划 `LT-DWA official`、`TEB`、`mpc_local_planner tuned` 三类 baseline 的实物 fixed-path 启动流程。  
> 原则：先 shadow / N=1 smoke，再决定是否进入 formal N=3；任一时刻只允许一个 planner 发布 `/cmd_vel`。

相关已有指南：

```text
docs/实物实验注意事项/对比试验/实物对比试验启动指南.md
docs/实物实验注意事项/对比试验/实物对比实验/0706_SPMPC实物补充实验矩阵计划.md
```

---

## 1. 0706 baseline 实验定位

0706 如果必须补外部对比算法，优先顺序建议为：

```text
1. LT-DWA official wrapper：最接近移动底盘防晃局部规划 baseline。
2. TEB：普通 online local planner baseline。
3. mpc_local_planner tuned：普通 MPC local planner baseline，但依赖 isolated MPC overlay。
```

不建议 0706 同时引入 Hamaguchi / Lim 实物主线。Ham/Lim 更像离线 profile baseline，需要 profile 生成、common tracker 与额外输出文件审计；建议单独安排，或先作为仿真/相关工作层级参照。

---

## 2. 总体启动顺序

### 2.1 通用前置模块

先按总启动指南完成：

```text
1. 实物传感器 / 定位 / 底盘栈；
2. 固定 RealSense RGB 参数；
3. 在线 RGB 液面观察；
4. standalone slosh monitor；
5. fixed S-curve path / goal 发布；
6. recorder 先于 planner 启动。
```

通用环境：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=$(date +%Y%m%d)
OUT_DIR=/home/geist/slosh_bags/real/${DATE}_fixed_path_compare
PATH_DIR=/home/geist/fixed_paths/real/${DATE}_fixed_path_compare
PATH_FILE=${PATH_DIR}/fixed_s_curve_compare.json
REF_TOPIC=/scout/global_path_fixed
GOAL_TOPIC=/scout/goal
mkdir -p "$OUT_DIR" "$PATH_DIR"
```

固定终点与 S-curve 参数沿用 0705/0706 SPMPC 口径：

```text
GOAL_X=-5.424
GOAL_Y=-4.736
GOAL_YAW=0.0
PATH_TEMPLATE=s_curve
PATH_START_HEADING=current
PATH_AMPLITUDE_RATIO=0.18
PATH_MIN_AMPLITUDE=0.25
PATH_MAX_AMPLITUDE=1.20
PATH_SIDE=left
PATH_SMOOTH_ITERATIONS=3
PATH_SPACING=0.05
```

### 2.2 每个 baseline 的两阶段流程

每个算法都按两阶段：

```text
Stage A: shadow / dry-run，不发布 /cmd_vel，只看 path、goal、status、raw/final command 是否合理。
Stage B: actuated N=1 smoke，发布 /cmd_vel，60s 内到点才允许进入正式 N=3。
```

进入 actuated 前必须检查：

```bash
rostopic info /cmd_vel
rostopic echo -n 1 /odom
rostopic echo -n 1 /map
rostopic echo -n 1 /scan_front
rostopic echo -n 1 /camera/color/image_raw
rostopic echo -n 1 ${REF_TOPIC}
rostopic echo -n 1 ${GOAL_TOPIC}
```

人工确认：

```text
/cmd_vel 无旧 publisher；
E-stop / 遥控急停就位；
RViz 中 path/goal/定位正常；
RGB 画面和液面 ROI 正常；
bag recorder 已经开始；
液体静稳 60~90s；
机器人周围安全。
```

---

## 3. 统一限制与记录口径

为避免外部 baseline 靠更慢速度或更松约束获得不公平优势，先使用保守 common-limit：

```text
max_v = 0.50 m/s
max_w = 1.2 rad/s
max_acc = 0.6 m/s^2
max_angular_acc = 1.2 rad/s^2
goal_xy_tolerance = 0.20 m
goal_yaw_tolerance = 0.30 rad
timeout = 60 s
```

说明：0705/0706 SPMPC 实物主线 `v_ref=0.20`，外部 baseline 的 `max_v=0.50` 只是上限，不等于一定以 0.50 全程运动。后处理必须同时报告到点时间、轨迹误差、`cmd_v/cmd_omega` 和 RGB 液面指标，不能只按液面峰值排序。

正式 N=3 每个 run 都必须录：

```text
/camera/color/image_raw
/camera/color/camera_info
/odom
/tf
/tf_static
/map
/scan_front
/scout/global_path_fixed
/scout/goal
/cmd_vel
/cmd_vel_drive
/liquid/height
/liquid/height_lcr
/slosh/height
/baseline/<method>/status
/baseline/<method>/global_plan
```

LT-DWA 额外录：

```text
/baseline/official_lt_dwa/raw_cmd_vel
/baseline/official_lt_dwa/shadow_cmd_vel
/baseline/official_lt_dwa/diagnostics
/baseline/official_lt_dwa/worker_result
/baseline/official_lt_dwa/local_plan
```

---

## 4. 固定路径与 recorder 启动

### 4.1 固定路径 generator

终端 A：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=$(date +%Y%m%d)
PATH_DIR=/home/geist/fixed_paths/real/${DATE}_fixed_path_compare
PATH_FILE=${PATH_DIR}/fixed_s_curve_compare.json
REF_TOPIC=/scout/global_path_fixed
GOAL_TOPIC=/scout/goal
mkdir -p "$PATH_DIR"

rosrun scout_local_planner template_fixed_path_generator.py \
  --template s_curve \
  --goal-topic ${GOAL_TOPIC} \
  --output-topic ${REF_TOPIC} \
  --path-file "${PATH_FILE}" \
  --start-heading current \
  --spacing 0.05 \
  --amplitude-ratio 0.18 \
  --min-amplitude 0.25 \
  --max-amplitude 1.20 \
  --side left \
  --smooth-iterations 3 \
  --publish-count 0
```

终端 B：发送固定终点：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic /scout/goal \
  --frame map \
  --x -5.424 \
  --y -4.736 \
  --yaw 0.0 \
  --repeat-count 5 \
  --repeat-rate 5
```

RViz 确认路径安全后再启动 baseline。

### 4.2 recorder 模板

每个 run 先开 recorder，再开 planner。示例：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

DATE=$(date +%Y%m%d)
ALG=teb
RUN_LABEL=TEB_fixed_150_220_0706_smoke01
OUT_DIR=/home/geist/slosh_bags/real/${DATE}_fixed_path_compare/${ALG}
mkdir -p "$OUT_DIR"

VARIANT=${ALG} \
RUN_LABEL=${RUN_LABEL} \
RECORD_SEC=60 \
OUT_DIR=${OUT_DIR} \
NAME=${RUN_LABEL} \
RECORD_RGB=true \
RECORD_SCAN=true \
RECORD_DEPTH=false \
RECORD_STANDALONE_SLOSH=true \
RECORD_ONLINE_LIQUID=true \
RECORD_ALL_EXISTING_TOPICS=true \
RECORD_TOPIC_INFO=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

说明：外部 baseline 第一轮建议 `RECORD_ALL_EXISTING_TOPICS=true`，避免漏录 `/baseline/*` 诊断。等 topic whitelist 确认后，再切回 whitelist recorder。

---

## 5. LT-DWA official wrapper

### 5.1 LT-DWA shadow

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

export SCOUT_WS_ROOT=/home/geist/scout_ws
export ROS_PACKAGE_PATH=$SCOUT_WS_ROOT/tools/lt_dwa/local_planner_runtime:${ROS_PACKAGE_PATH:-}

roslaunch lt_dwa_official_wrapper scout_sop_shadow_integration.launch \
  start_local_map_service:=true \
  enable_actuated_output:=false \
  publish_cmd_vel:=false \
  planner_execution_mode:=in_process \
  planner_rate_hz:=5.0 \
  command_publish_rate_hz:=30.0 \
  max_v:=0.50 \
  max_w:=1.2 \
  max_acc:=0.6 \
  max_angular_acc:=1.2 \
  robot_radius:=0.426 \
  goal_xy_tolerance:=0.20 \
  goal_yaw_tolerance:=0.30 \
  input_odom_topic:=/odom \
  map_topic:=/map \
  path_topic:=/scout/global_path_fixed \
  goal_topic:=/scout/goal \
  raw_cmd_topic:=/baseline/official_lt_dwa/raw_cmd_vel \
  shadow_cmd_topic:=/baseline/official_lt_dwa/shadow_cmd_vel \
  status_topic:=/baseline/official_lt_dwa/status \
  diagnostics_topic:=/baseline/official_lt_dwa/diagnostics \
  worker_result_topic:=/baseline/official_lt_dwa/worker_result \
  cmd_vel_topic:=/cmd_vel
```

shadow 必看：

```bash
rostopic echo -n 1 /baseline/official_lt_dwa/status
rostopic echo -n 1 /baseline/official_lt_dwa/raw_cmd_vel
rostopic echo -n 1 /baseline/official_lt_dwa/shadow_cmd_vel
rostopic echo -n 1 /baseline/official_lt_dwa/diagnostics
rostopic echo -n 1 /baseline/official_lt_dwa/worker_result
```

只有看到 raw/final command 合理、status 正常，才进入 actuated。

### 5.2 LT-DWA actuated N=1 smoke

先开 recorder：

```text
RUN_LABEL=LTDWA_fixed_0706_smoke01
ALG=lt_dwa_official
RECORD_ALL_EXISTING_TOPICS=true
```

再启动 actuated：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

export SCOUT_WS_ROOT=/home/geist/scout_ws
export ROS_PACKAGE_PATH=$SCOUT_WS_ROOT/tools/lt_dwa/local_planner_runtime:${ROS_PACKAGE_PATH:-}

roslaunch lt_dwa_official_wrapper scout_sop_cmd_vel_benchmark.launch \
  start_local_map_service:=true \
  enable_actuated_output:=true \
  publish_cmd_vel:=true \
  planner_execution_mode:=in_process \
  planner_rate_hz:=5.0 \
  command_publish_rate_hz:=30.0 \
  max_v:=0.50 \
  max_w:=1.2 \
  max_acc:=0.6 \
  max_angular_acc:=1.2 \
  robot_radius:=0.426 \
  goal_xy_tolerance:=0.20 \
  goal_yaw_tolerance:=0.30 \
  input_odom_topic:=/odom \
  map_topic:=/map \
  path_topic:=/scout/global_path_fixed \
  goal_topic:=/scout/goal \
  raw_cmd_topic:=/baseline/official_lt_dwa/raw_cmd_vel \
  shadow_cmd_topic:=/baseline/official_lt_dwa/shadow_cmd_vel \
  status_topic:=/baseline/official_lt_dwa/status \
  diagnostics_topic:=/baseline/official_lt_dwa/diagnostics \
  worker_result_topic:=/baseline/official_lt_dwa/worker_result \
  cmd_vel_topic:=/cmd_vel
```

停止条件：raw/final command 异常、原地转圈、明显离轨、60s 未到点、diagnostics 长时间异常。

---

## 6. TEB baseline

### 6.1 TEB shadow

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

BASELINE_RUNNER=$(rospack find baseline_local_planner_runner)
SPMPC_EXP=$(rospack find spmpc_experiments)

roslaunch baseline_local_planner_runner nav_core_runner.launch \
  plugin_type:=teb_local_planner/TebLocalPlannerROS \
  plugin_name:=TebLocalPlannerROS \
  global_path_topic:=/scout/global_path_fixed \
  goal_topic:=/scout/goal \
  cmd_vel_topic:=/spmpc_shadow_cmd_vel \
  status_topic:=/baseline/teb/status \
  global_plan_topic:=/baseline/teb/global_plan \
  controller_frequency:=10.0 \
  base_frame:=base_link \
  plan_target_frame:=map \
  max_cmd_vel_x:=0.50 \
  max_cmd_vel_theta:=1.2 \
  xy_goal_tolerance:=0.20 \
  yaw_goal_tolerance:=0.30 \
  costmap_config:=${BASELINE_RUNNER}/config/local_costmap_real.yaml \
  planner_config:=${SPMPC_EXP}/config/baselines/teb_local_planner_standalone_sim.yaml
```

shadow 检查：

```bash
rostopic echo -n 1 /baseline/teb/status
rostopic echo -n 1 /baseline/teb/global_plan
rostopic echo -n 1 /spmpc_shadow_cmd_vel
```

### 6.2 TEB actuated N=1 smoke

只把 `cmd_vel_topic` 改为 `/cmd_vel`。先开 recorder：

```text
RUN_LABEL=TEB_fixed_0706_smoke01
ALG=teb
RECORD_ALL_EXISTING_TOPICS=true
```

然后：

```bash
roslaunch baseline_local_planner_runner nav_core_runner.launch \
  plugin_type:=teb_local_planner/TebLocalPlannerROS \
  plugin_name:=TebLocalPlannerROS \
  global_path_topic:=/scout/global_path_fixed \
  goal_topic:=/scout/goal \
  cmd_vel_topic:=/cmd_vel \
  status_topic:=/baseline/teb/status \
  global_plan_topic:=/baseline/teb/global_plan \
  controller_frequency:=10.0 \
  base_frame:=base_link \
  plan_target_frame:=map \
  max_cmd_vel_x:=0.50 \
  max_cmd_vel_theta:=1.2 \
  xy_goal_tolerance:=0.20 \
  yaw_goal_tolerance:=0.30 \
  costmap_config:=${BASELINE_RUNNER}/config/local_costmap_real.yaml \
  planner_config:=${SPMPC_EXP}/config/baselines/teb_local_planner_standalone_sim.yaml
```

---

## 7. mpc_local_planner tuned baseline

### 7.1 MPC overlay 要求

`mpc_local_planner` 需要 isolated MPC overlay。启动前先确认机器人端存在：

```bash
ls /home/geist/scout_ws/install_isolated_mpc/setup.bash
```

若不存在，不跑该实物 baseline，先回开发机/仿真补环境。

推荐 source 顺序：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
source /home/geist/scout_ws/install_isolated_mpc/setup.bash
cd /home/geist/scout_ws
```

### 7.2 MPC shadow

```bash
BASELINE_RUNNER=$(rospack find baseline_local_planner_runner)
SPMPC_EXP=$(rospack find spmpc_experiments)

roslaunch baseline_local_planner_runner nav_core_runner.launch \
  plugin_type:=mpc_local_planner/MpcLocalPlannerROS \
  plugin_name:=MpcLocalPlannerROS \
  global_path_topic:=/scout/global_path_fixed \
  goal_topic:=/scout/goal \
  cmd_vel_topic:=/spmpc_shadow_cmd_vel \
  status_topic:=/baseline/mpc_local_planner/status \
  global_plan_topic:=/baseline/mpc_local_planner/global_plan \
  controller_frequency:=10.0 \
  base_frame:=base_link \
  plan_target_frame:=map \
  max_cmd_vel_x:=0.50 \
  max_cmd_vel_theta:=1.2 \
  xy_goal_tolerance:=0.20 \
  yaw_goal_tolerance:=0.30 \
  costmap_config:=${BASELINE_RUNNER}/config/local_costmap_real.yaml \
  planner_config:=${SPMPC_EXP}/config/baselines/mpc_local_planner_fixed_path_tuned_sim.yaml
```

shadow 检查：

```bash
rostopic echo -n 1 /baseline/mpc_local_planner/status
rostopic echo -n 1 /baseline/mpc_local_planner/global_plan
rostopic echo -n 1 /spmpc_shadow_cmd_vel
```

### 7.3 MPC actuated N=1 smoke

先开 recorder：

```text
RUN_LABEL=MPC_local_planner_fixed_0706_smoke01
ALG=mpc_local_planner
RECORD_ALL_EXISTING_TOPICS=true
```

然后只把 `cmd_vel_topic` 改成 `/cmd_vel`：

```bash
roslaunch baseline_local_planner_runner nav_core_runner.launch \
  plugin_type:=mpc_local_planner/MpcLocalPlannerROS \
  plugin_name:=MpcLocalPlannerROS \
  global_path_topic:=/scout/global_path_fixed \
  goal_topic:=/scout/goal \
  cmd_vel_topic:=/cmd_vel \
  status_topic:=/baseline/mpc_local_planner/status \
  global_plan_topic:=/baseline/mpc_local_planner/global_plan \
  controller_frequency:=10.0 \
  base_frame:=base_link \
  plan_target_frame:=map \
  max_cmd_vel_x:=0.50 \
  max_cmd_vel_theta:=1.2 \
  xy_goal_tolerance:=0.20 \
  yaw_goal_tolerance:=0.30 \
  costmap_config:=${BASELINE_RUNNER}/config/local_costmap_real.yaml \
  planner_config:=${SPMPC_EXP}/config/baselines/mpc_local_planner_fixed_path_tuned_sim.yaml
```

---

## 8. 一键脚本用法

新增外部 baseline 单 run 一键脚本：

```bash
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

脚本一次只跑一个 `METHOD + STAGE`，不自动跑完整 N=3：

```text
METHOD=lt_dwa_official | teb | mpc_local_planner
STAGE=shadow | actuated
```

推荐先跑 shadow，再跑 actuated N=1 smoke：

```bash
# LT-DWA shadow
METHOD=lt_dwa_official \
STAGE=shadow \
RUN_LABEL=LTDWA_fixed_0706_shadow01 \
RECORD_ALL_EXISTING_TOPICS=true \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh

# LT-DWA actuated N=1 smoke
METHOD=lt_dwa_official \
STAGE=actuated \
RUN_LABEL=LTDWA_fixed_0706_smoke01 \
RECORD_ALL_EXISTING_TOPICS=true \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh

# TEB shadow / actuated
METHOD=teb STAGE=shadow RUN_LABEL=TEB_fixed_0706_shadow01 RECORD_ALL_EXISTING_TOPICS=true \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh

METHOD=teb STAGE=actuated RUN_LABEL=TEB_fixed_0706_smoke01 RECORD_ALL_EXISTING_TOPICS=true \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh

# mpc_local_planner shadow / actuated
METHOD=mpc_local_planner STAGE=shadow RUN_LABEL=MPC_fixed_0706_shadow01 RECORD_ALL_EXISTING_TOPICS=true \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh

METHOD=mpc_local_planner STAGE=actuated RUN_LABEL=MPC_fixed_0706_smoke01 RECORD_ALL_EXISTING_TOPICS=true \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

脚本默认口径：

```text
RECORD_SEC=60
GOAL=(-5.424, -4.736, 0.0) in map
PATH_TEMPLATE=s_curve
MAX_V=0.50
MAX_W=1.2
MAX_ACC=0.6
MAX_ANGULAR_ACC=1.2
XY_GOAL_TOL=0.20
YAW_GOAL_TOL=0.30
```

安全边界：

```text
1. STAGE=shadow 默认不发布 /cmd_vel；
2. STAGE=actuated 启动前脚本会检查 /cmd_vel 是否已有 publisher，发现旧 publisher 直接失败；
3. recorder 一定先于 planner 启动；
4. 退出/中断时 actuated run 会发布 zero /cmd_vel；
5. 第一版只做单 baseline 单 run，不自动执行 formal N=3。
```

`mpc_local_planner` 需要机器人端已有 isolated MPC overlay：

```text
/home/geist/scout_ws/install_isolated_mpc/setup.bash
```

若该文件不存在，脚本会直接失败，不进入 planner launch。

---

## 9. 建议执行顺序

### 9.1 当天最低 smoke 顺序

```text
1. B0 fixed 0.15/0.22 gate，确认当天 SPMPC baseline 正常；
2. LT-DWA shadow；
3. LT-DWA actuated N=1；
4. TEB shadow；
5. TEB actuated N=1；
6. mpc_local_planner shadow；
7. mpc_local_planner actuated N=1。
```

任一算法 N=1 smoke 失败，不进入该算法 formal N=3。

### 9.2 如果三者 N=1 都成功，再做 formal N=3

推荐交错，不要一个算法连续跑完：

```text
Round 1: LT-DWA -> TEB -> mpc_local_planner
Round 2: mpc_local_planner -> TEB -> LT-DWA
Round 3: TEB -> LT-DWA -> mpc_local_planner
```

每个 run 之间：

```text
停 planner -> 发 /cmd_vel zero -> 停 bag -> 回起点 -> 液体静稳 60~90s -> 查 /cmd_vel publisher -> 开下一轮 recorder -> 开 planner
```

---

## 10. 结果判读

每个 baseline 至少输出：

```text
1. success / timeout / invalid；
2. first GOAL_REACHED time；
3. projection / tracking p95/max；
4. RGB H_vis p95/max/RMS；
5. /slosh/height p95/max/RMS 作为统一 proxy；
6. cmd_v / cmd_omega p95/max；
7. 是否有 command zero / limiter / guard；
8. 失败原因。
```

写论文时注意：

```text
1. 外部 baseline 不使用液面反馈，因此不要说它主动防晃，除非算法本身确实有防晃 profile；
2. 如果某 baseline 很慢导致 RGB 更低，必须同时报告到点时间和平均速度；
3. 如果某 baseline tracking 差但液面低，不能直接认为更好；
4. 实物外部 baseline 结论建议作为代表性补充，主统计仍以仿真大矩阵和 SPMPC 内部消融为主。
```
