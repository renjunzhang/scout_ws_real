# Slosh-Priority MPC 实物有效性验证方案

日期：2026-05-13

## 1. 目的

本方案验证一条独立于 OSCRS / GeoRef 的路线：

```text
通过重新分配 MPC 代价函数中 tracking、速度跟踪、控制平滑和液体模态响应的优先级，
观察 MPC 是否能在路径跟踪仍可接受的前提下降低真实 RGB 液面晃动。
```

这不是 OSCRS 主线验证。OSCRS 主线保持：

```text
Q_slosh=0
slosh_height_ref=0.005
slosh_eta_dot_ratio=0.3
Q_slosh_eta_dot=0
MPC 作为 normal tracker
防晃逻辑位于 reference / candidate selection 层
```

本方案只回答：

```text
如果把 MPC 本体重新调成 slosh-priority objective，
它是否能产生可被 RGB 视觉真值支持的抑晃效果？
```

## 2. 核心假设

普通 tracking MPC 当前默认更偏向：

```text
强贴线
强速度跟踪
控制平滑适中
晃动 soft cost 关闭
```

如果只把 `Q_slosh` 拉大，而不降低 `Q_v`、不提高 `R_da/R_domega`，优化器可能只是改变液体相位或产生控制突变，未必降低真实液面。

本方案的调参原则是：

```text
不要靠降低 R_a / R_omega 来凸显 slosh；
要降低速度跟踪刚性，保留路径跟踪刚性，提高 slosh 状态权重；
同时用控制变化率代价和硬约束防止 bang-bang 控制。
```

真正证明 slosh 项有效，必须和同等速度/平滑设置下的 `SMOOTH_SPEED_RELAXED` 对照：

```text
SMOOTH_SPEED_RELAXED:
  降 Q_v + 提高 R_da/R_domega，但 Q_slosh=0

SLOSH_PRIORITY_MPC:
  同样 Q_v/R_da/R_domega，只额外打开 normalized Q_slosh / slosh_eta_dot_ratio / terminal slosh
```

因此本方案验证的是：

```text
slosh-aware objective rebalancing
```

而不是单独验证：

```text
Q_slosh 越大越好
```

### 2.1 当前 MPC cost 组成

当前 MPC 的 stage cost 可按工程项理解为：

```text
J = J_tracking + J_control + J_smooth + J_slosh + J_optional
```

其中基础项为：

```text
J_tracking:
  Q_lag      * e_lag^2
  Q_contour  * e_contour^2
  Q_etheta   * e_theta^2
  Q_v        * (v - v_ref)^2

J_control:
  R_a        * a^2
  R_omega    * omega^2

J_smooth:
  R_da       * (a_k - a_{k-1})^2
  R_domega   * (omega_k - omega_{k-1})^2
```

当前已实现的晃动项是 normalized modal cost：

```text
J_slosh:
  Q_slosh_eta     * (eta_x^2 + eta_y^2)
  Q_slosh_eta_dot * (eta_dot_x^2 + eta_dot_y^2)
```

其中：

```text
Q_slosh_eta     = Q_slosh * h_coeff^2 / slosh_height_ref^2
Q_slosh_eta_dot = slosh_eta_dot_ratio * Q_slosh_eta / omega_n^2
```

可选项：

```text
J_optional:
  Q_omega_ff * (omega - v_ref * kappa)^2
  terminal ramp 对 e_contour/e_theta/v/slosh 权重做末端放大
```

注意：

```text
v_des_rate_limit 不是 cost 项；
terminal_slowdown 主要改变进入 PathHandler 的速度上限和 terminal 速度权重；
TERMINAL_MPC_STOP 是终点监督状态，仍继续由 MPC 做低速收敛。
```

所以当前验证的核心还是：

```text
在相同 v_des_rate_limit / terminal 策略下，
比较 Q_slosh=0 与 Q_slosh>0 的 normalized modal cost 是否带来额外收益。
```

## 3. 工程前置检查

当前实物入口：

```bash
roslaunch scout_local_planner slosh_experiment.launch
```

已暴露：

```text
Q_slosh
slosh_height_ref
slosh_eta_dot_ratio
Q_slosh_eta_dot
mpc_Q_lag
mpc_Q_contour
mpc_Q_etheta
mpc_Q_v
mpc_R_a
mpc_R_omega
mpc_R_da
mpc_R_domega
terminal_factor_slosh_eta
terminal_factor_slosh_eta_dot
enable_slosh_box_constraint
terminal_slowdown_enable
terminal_slowdown_distance
terminal_slowdown_v_max
terminal_slowdown_Q_v
terminal_slowdown_terminal_factor_v
terminal_capture_stop_enable
terminal_capture_stop_distance
v_des_rate_limit_enable
v_des_accel_limit
v_des_decel_limit
energy_profile_enable
risk_scheduler_enable
input_shaping_enable
slosh_speed_governor_enable
```

这些参数会在加载 `mpc_params.yaml` 后、节点初始化前覆盖对应 ROS 参数。A/B/C/D/E 五组通过 launch arg 覆盖代价权重；终点默认采用 `mpc_planner` 风格的外层 capture stop，但当前语义是进入 `TERMINAL_MPC_STOP` 后继续由 MPC 平滑收敛，不再由外层直接接管制动。不把 near-goal 停车段作为 slosh 主效果窗口。

当前控制频率配置：

```text
control_rate = 30 Hz
mpc.dt = 0.0333333333 s
mpc.N = 60
prediction horizon ≈ 2.0 s
```

注意：

```text
30 Hz 下单周期约 33 ms。N=60 会增加 QP 规模，实物 smoke 必须检查 /mpc/solve_ms。
同时检查 /mpc/cost_breakdown，确认晃动项在优化器内部有可见占比。
```

启动时可覆盖：

```text
mpc/Q_lag
mpc/Q_v
mpc/Q_contour
mpc/Q_etheta
mpc/R_a
mpc/R_omega
mpc/R_da
mpc/R_domega
mpc/slosh_height_ref
mpc/slosh_eta_dot_ratio
mpc/terminal_factor_slosh_eta
mpc/terminal_factor_slosh_eta_dot
```

检查结果：

```text
src/scout_apps/control/scout_local_planner/launch/slosh_experiment.launch
  已支持 A/B/C/D/E 组所需的 Q_slosh / slosh_height_ref / slosh_eta_dot_ratio / Q_v / Q_contour / Q_etheta /
  R_a / R_da / R_domega / terminal_factor_slosh_* / global_path_topic 等参数。

src/scout_apps/control/scout_local_planner/launch/slosh_experiment_sim.launch
  仿真入口已支持部分覆盖：mpc_Q_v / mpc_R_a / mpc_R_da / terminal_factor_slosh_*；
  但仍未覆盖 Q_contour / Q_etheta / R_domega。
```

执行前先确认：

```bash
rosparam get /scout_local_planner/mpc/Q_lag
rosparam get /scout_local_planner/mpc/Q_contour
rosparam get /scout_local_planner/mpc/Q_etheta
rosparam get /scout_local_planner/mpc/Q_v
rosparam get /scout_local_planner/mpc/R_a
rosparam get /scout_local_planner/mpc/R_omega
rosparam get /scout_local_planner/mpc/R_da
rosparam get /scout_local_planner/mpc/R_domega
rosparam get /scout_local_planner/mpc/Q_slosh
rosparam get /scout_local_planner/mpc/slosh_height_ref
rosparam get /scout_local_planner/mpc/slosh_eta_dot_ratio
rosparam get /scout_local_planner/mpc/Q_slosh_eta_dot
rosparam get /scout_local_planner/mpc/terminal_factor_slosh_eta
rosparam get /scout_local_planner/mpc/terminal_factor_slosh_eta_dot
rosparam get /scout_local_planner/terminal_capture_stop/enable
rosparam get /scout_local_planner/terminal_capture_stop/distance
rosparam get /scout_local_planner/v_des_rate_limit/enable
rosparam get /scout_local_planner/v_des_rate_limit/accel_limit
rosparam get /scout_local_planner/v_des_rate_limit/decel_limit
```

除控制频率、`N`、`dt` 这类全局时基设置外，不要为某一组实验直接改 `mpc_params.yaml` 默认代价权重。A/B/C/D/E 的代价权重应通过 `slosh_experiment.launch` 参数覆盖。

## 4. 固定边界

所有组必须使用同一条路径、同一套视觉识别参数、同一相机姿态、同一液位、同一容器、同一初始静止流程。

本方案不允许同时打开这些机制：

```text
OSCRS post-processor
GeoRef post-processor
PROFILE_ENERGY / energy_profile_enable
input_shaping
risk_scheduler
slosh_speed_governor
enable_slosh_box_constraint
```

原因：本实验只验证 MPC cost/objective rebalancing。若混入外层参考生成或输出治理，无法归因。

固定关闭：

```text
energy_profile_enable=false
input_shaping_enable=false
risk_scheduler_enable=false
slosh_speed_governor_enable=false
enable_slosh_box_constraint=false
```

固定保持一致：

```text
terminal_slowdown_enable=true
terminal_capture_stop_enable=true
v_des_rate_limit_enable=true
```

说明：`v_des_rate_limit` 是当前为治理纵向 `ax` 脉冲加入的结构层，不属于 slosh cost，也不是某一实验组的可变因素。A/B/C/D/E 必须使用同一套 `v_des_rate_limit`，否则无法把差异归因到 MPC cost/objective rebalancing。

`/slosh/height` 只作为模型内部参考和调试量，不能作为真实液面主指标。正式结论以 RGB 视觉液面为准。

## 5. 实验分组

每组至少 3 包；如果组间差异接近，补到 5 包。

### A. BASE

目的：当前普通 tracking MPC 基线。

```yaml
Q_lag: 0.5
Q_contour: 32.0
Q_etheta: 15.0
Q_v: 9.0
R_a: 0.4
R_omega: 2.0
R_da: 0.5
R_domega: 4.0
Q_slosh: 0.0
slosh_height_ref: 0.005
slosh_eta_dot_ratio: 0.3
Q_slosh_eta_dot: 0.0
terminal_factor_slosh_eta: 0.0
terminal_factor_slosh_eta_dot: 0.0
```

### B. SOFT_SLOSH_ONLY

目的：只验证“在原 tracking MPC 上加入晃动 soft cost”是否有效。

```yaml
Q_lag: 0.5
Q_contour: 32.0
Q_etheta: 15.0
Q_v: 9.0
R_a: 0.4
R_omega: 2.0
R_da: 0.5
R_domega: 4.0
Q_slosh: 5.0
slosh_height_ref: 0.005
slosh_eta_dot_ratio: 0.3
Q_slosh_eta_dot: 0.0
terminal_factor_slosh_eta: 0.0
terminal_factor_slosh_eta_dot: 0.0
```

解释边界：

```text
B 组只能说明单独 slosh soft cost 的效果。
如果 B 不优于 A，不代表 slosh-priority MPC 失败。
```

### C. SMOOTH_SPEED_RELAXED

目的：区分“降速/平滑控制本身”与“晃动模态项”的贡献。

```yaml
Q_lag: 0.5
Q_contour: 28.0
Q_etheta: 12.0
Q_v: 3.0
R_a: 0.5
R_omega: 2.0
R_da: 1.5
R_domega: 6.0
Q_slosh: 0.0
slosh_height_ref: 0.005
slosh_eta_dot_ratio: 0.3
Q_slosh_eta_dot: 0.0
terminal_factor_slosh_eta: 0.0
terminal_factor_slosh_eta_dot: 0.0
```

解释边界：

```text
如果 C 已显著降低 RGB 液面，说明主要收益可能来自速度松弛和控制平滑。
此时 D 组必须进一步优于 C，才能说明 slosh 模态项有增量价值。
```

### D. SLOSH_PRIORITY_MPC

目的：主实验组。验证重新分配 MPC objective 后，晃动项是否能产生额外效果。

```yaml
Q_lag: 0.5
Q_contour: 28.0
Q_etheta: 12.0
Q_v: 3.0
R_a: 0.5
R_omega: 2.0
R_da: 1.5
R_domega: 6.0
Q_slosh: 5.0
slosh_height_ref: 0.005
slosh_eta_dot_ratio: 0.3
Q_slosh_eta_dot: 0.0
terminal_factor_slosh_eta: 5.0
terminal_factor_slosh_eta_dot: 3.0
```

允许的第二轮小步升级：

```yaml
Q_slosh: 10.0
slosh_height_ref: 0.005
slosh_eta_dot_ratio: 0.3
```

只有在 D 组 `pct_slosh_total` 仍低于 1%，且 horizon summary 显示 eta 已正常预测出来时，才进入该升级。

### E. SLOSH_DOMINANT

目的：展示更强晃动优先级的 trade-off 上限，不作为默认主方法。

```yaml
Q_lag: 0.5
Q_contour: 24.0
Q_etheta: 10.0
Q_v: 2.0
R_a: 0.6
R_omega: 2.0
R_da: 2.0
R_domega: 8.0
Q_slosh: 10.0
slosh_height_ref: 0.005
slosh_eta_dot_ratio: 0.3
Q_slosh_eta_dot: 0.0
terminal_factor_slosh_eta: 8.0
terminal_factor_slosh_eta_dot: 8.0
```

解释边界：

```text
E 组若液面更低但 completion time 明显变长，只能作为 trade-off 曲线，不应作为主结论。
```

## 6. 路径与动作设计

优先使用固定路径 replay，避免每次 MBF 规划差异污染结果。

路径要求：

```text
开阔场地
路径长度 >= 4 m
包含至少一个 90 度左右转弯或 S 型段
不贴墙
BASE 能稳定完成
不需要人工接管
```

推荐两类路径：

```text
P2_REAL_S:
  主要考察 lateral ay / omega / domega 激励。

P3_REAL_MIXED:
  包含直线加减速 + 弯道，考察 longitudinal ax 与弯道组合激励。
```

每包开始前必须：

```text
小车静止 >= 5 s
液体可视液面基本静止
相机画面无遮挡
红液 ROI 和三标尺可见
```

如果上一包终点残余明显，等待液面停止后再录下一包。

## 7. 推荐录包流程

### 7.1 启动基础系统

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
```

底盘、雷达、定位、相机按实物固定流程启动。RGB 视觉按：

```text
docs/重要文档/红色液体视觉验证固定流程.md
```

建议相机保持 30 Hz。当前 MPC 控制频率也按 30 Hz 配置，与 RealSense RGB 时间基准对齐。

如果 USB/CPU 压力导致相机丢帧，可先把相机降到 20 Hz；此时不要为了单包实验临时改 MPC 频率，分析时按时间戳重采样对齐。

当前方案已将 MPC 控制频率调整到 30 Hz，与 RealSense RGB 对齐：

```text
control_rate=30.0
mpc.dt=0.0333333333
mpc.N=60
```

启动后建议确认：

```bash
rostopic hz /cmd_vel
rostopic hz /slosh/height
rostopic echo /mpc/solve_ms
rostopic echo /mpc/cost_breakdown
rostopic echo /mpc/slosh_horizon_summary
```

正式录包统一使用：

```bash
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh
```

实物机推荐命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

SLOSH_BAG_DIR=/home/geist/slosh_bags/real/20260517_slosh_priority \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 0 C_run01
```

参数含义：

```text
第 1 个参数：Q_slosh，用于写入 bag 文件名，例如 0 / 3 / 7 / 10。
第 2 个参数：自定义后缀，用于写入 bag 文件名，例如 C_run01 / D_run02 / Q7_run03。
SLOSH_BAG_DIR：输出目录。
SLOSH_RECORD_ALL=true：默认全量录制所有当前存在的 ROS topic。
```

常用命名示例：

```bash
# C 组：SMOOTH_SPEED_RELAXED，Q_slosh=0
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/20260517_slosh_priority \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 0 C_run01

# D 组：SLOSH_PRIORITY_MPC，按 launch 中实际 Q_slosh 填第 1 个参数
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/20260517_slosh_priority \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 5 D_run01

# Q7 组：D3 基础上 Q_slosh=7
SLOSH_BAG_DIR=/home/geist/slosh_bags/real/20260517_slosh_priority \
SLOSH_RECORD_ALL=true \
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 7 Q7_run01
```

该脚本现在默认按“完整实验包”录制：

```text
SLOSH_RECORD_ALL=true
rosbag record -a
```

也就是把录制开始时 ROS master 上存在的 topic 全部录进 bag，避免因为漏列某个诊断 topic 导致重新录包。除 RGB 原始视频外，至少应覆盖以下主链路：

```text
slosh observer/model:
  /slosh/state, /slosh/height, /slosh/height_pred_max,
  /slosh/eta_norm, /slosh/eta_dot_norm, /slosh/modal_energy*,
  /slosh/ax_est, /slosh/ay_est, /slosh/alpha_est,
  /slosh/imu_*, /slosh/settling_time

RGB / visual truth:
  /camera/color/image_raw, /camera/color/camera_info,
  /camera/depth/image_rect_raw, /camera/depth/camera_info,
  /camera/aligned_depth_to_color/*,
  /slosh/h_visual, /slosh/h_visual_quality,
  /liquid_measurement/*

MPC / reference / cost:
  /mpc/cost_breakdown, /mpc/slosh_horizon_summary,
  /reference/v_des_*, /reference/v_ref*, /reference/s_horizon,
  /reference/implied_ax, /reference/implied_ay, /reference/implied_jerk

terminal / execution:
  /terminal/mode, /terminal/goal_info, /terminal/recovery_latched,
  /cmd_vel, /odom, /scout/cmd_vel, /scout/odom

path / OSCRS:
  /scout/global_path*, /local_path, /mpc/reference_path,
  /anti_slosh_path/*

localization / safety:
  /scan_front, /map*, /tf, /tf_static, /diagnostics, /rosout,
  cartographer/amcl 常用定位输出。
```

脚本还会在 bag 同目录保存 sidecar 元数据：

```text
*_info.txt       当前 git branch/commit/dirty 状态、实验名、话题数
*_rosparam.yaml 录包开始前的完整 ROS 参数快照
*_topics.txt    录包开始前 rostopic list
*_nodes.txt     录包开始前 rosnode list
```

后续若怀疑参数、节点、topic 是否正确，优先查这些 sidecar 文件，避免因为缺少 launch 参数快照重新录包。

如果实物机磁盘或写入速度压力过大，可以临时切回白名单录制：

```bash
SLOSH_RECORD_ALL=false src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh 0 C_run01
```

白名单包含上面列出的核心分析话题，但正式论文实验优先使用默认全量录制。

当前实物入口采用“速度参考平滑 + 终点 MPC stop 监督”的终点策略，借鉴 `src/mpc_planner` 的外层 goal capture 思路：MPC 负责主路径跟踪和终点低速收敛，终点附近由外层状态机验收是否已低速到达，不再由外层直接抢占刹车。

第一层：进入 capture stop 半径后进入 `TERMINAL_MPC_STOP`。

```text
terminal_capture_stop_enable=true
terminal_capture_stop_distance=0.70
```

语义：

```text
goal_dist < terminal_capture_stop_distance:
  goal_stop_pending = true；
  /terminal/mode = TERMINAL_MPC_STOP；
  继续执行 MPC solve；
  v_des 目标降为 0，并提高 terminal Q_v / terminal_factor_v；
  若 odom 速度降到 goal_reached_max_speed / goal_reached_max_omega 以下，则进入 REACHED。
```

这对应 `mpc_planner` 的处理方式：

```text
GoalModule / Contouring 均使用约 1.0m 的宽到达判定；
ROS navigation 外层 isGoalReached/reset 负责结束任务；
MPC 不承担最后厘米级停车。
```

本项目使用 0.70m 而不是 1.0m，是为了减少固定路径实验中“停得过早”的风险。若仍过冲，可改到 0.90 或 1.00；若明显过早，可降到 0.50。

第二层：进入 terminal MPC stop 之前仍保留平滑 slowdown 参考。

```text
terminal_slowdown_enable=true
terminal_slowdown_distance=1.20
terminal_slowdown_v_max=0.18
terminal_slowdown_Q_v=40.0
terminal_slowdown_terminal_factor_v=5.0
```

slowdown 的作用是在进入 terminal MPC stop 前降低接近速度。MPC 会在 `terminal_slowdown_distance` 内按剩余距离施加制动速度包络：

```text
v_brake = sqrt(goal_speed^2 + 2 * decel * max(0, goal_dist - goal_tolerance))
v_terminal = min(terminal_slowdown_v_max, v_brake)
v_cap = smoothstep(goal_dist) * v_nominal + (1 - smoothstep(goal_dist)) * v_terminal
v_des_cmd <= v_cap
```

其中 `decel` 优先使用 `terminal_cmd_v_rate_limit`。`v_brake` 是按“剩余距离内能制动到 `goal_speed`”反推的速度上限；`smoothstep(goal_dist)` 用于从正常巡航速度平滑过渡到该制动包络。

注意：`terminal_slowdown_v_max` 只是终点段速度上界，不再作为 0.18 m/s 平台速度。越接近 `goal_tolerance`，`v_brake` 会继续降到 `goal_speed=0`，避免最后只在几厘米内才刹停。

当该制动速度上限实际低于当前 `v_des_cmd` 时，控制器会临时提高速度跟踪权重：

```text
Q_v >= terminal_slowdown_Q_v
terminal_factor_v >= terminal_slowdown_terminal_factor_v
```

这样终点停车段的速度参考不再只是软提示，避免 slosh-priority 组因为降低 `Q_v` 而忽略停车参考。该临时权重只作用于 `terminal_slowdown` 内，不参与进入 `terminal_slowdown` 前的 slosh cost 主有效性判断。

第三层：执行层 `v_des_rate_limit` 治理纵向 `ax` 脉冲。

```text
v_des_rate_limit_enable=true
v_des_accel_limit=0.60
v_des_decel_limit=0.80
```

链路：

```text
v_des_raw
  -> terminal_slowdown / feasibility / curvature / governor 后得到 v_des_target
  -> v_des_rate_limit 得到 v_des_eff
  -> PathHandler 生成 horizon 内 v_ref
```

这一步不是 cost，也不是硬约束；它是参考生成前的速度上限平滑器。目的在于避免 `v_des_cmd` 一帧突降或突升进入 horizon，减少纵向 `ax` 脉冲对 RGB 液面结论的污染。

下一轮 bag 需要同时看：

```text
/reference/v_des_raw
/reference/v_des_target
/reference/v_des_eff
/reference/v_des_rate_limited
/reference/v_ref_horizon
/reference/implied_ax
/cmd_vel
/scout/odom
```

判断：

```text
raw -> target 突降：上游限速/terminal/curvature 在压速度；
target -> eff 被削：v_des_rate_limit 正在平滑突变；
eff 已平滑但 odom_ax 仍大：问题更可能在 MPC 输出、cmd_vel 或底盘执行侧。
```

第四层：terminal recovery 只作为手动兜底，不进入主实验默认路径。

```text
terminal_recovery_enable=false
```

如果实物出现明显冲过终点后无法停稳、必须调头回点，再显式开启：

```bash
terminal_recovery_enable:=true
```

不要通过增大 `Q_slosh` 解决终点过冲或纵向 `ax` 脉冲。终点策略不参与 slosh 主归因，主结论只看进入 `terminal_slowdown` 前的 `TRACKING_PRE_TERMINAL`。

### 7.2 保存固定路径

先用普通导航生成一次目标路径，然后保存：

```bash
mkdir -p /home/geist/fixed_paths/real

rosrun scout_local_planner fixed_global_path_runner.py \
  --mode capture \
  --input-topic /scout/global_path \
  --path-file /home/geist/fixed_paths/real/slosh_priority_p2_s.json \
  --capture-timeout 30
```

检查：

```bash
python3 -m json.tool /home/geist/fixed_paths/real/slosh_priority_p2_s.json | head
```

### 7.3 启动 MPC 实验入口

每组都订阅固定路径：

```text
global_path_topic:=/scout/global_path_fixed
```

当前实物入口是：

```bash
roslaunch scout_local_planner slosh_experiment.launch
```

它现在能直接运行 A/B/C/D/E 五组。

### 7.3.1 当前实物 launch 可直接运行的命令

A 组 BASE：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
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
  slosh_height_ref:=0.005 \
  slosh_eta_dot_ratio:=0.3 \
  Q_slosh_eta_dot:=0.0 \
  terminal_factor_slosh_eta:=0.0 \
  terminal_factor_slosh_eta_dot:=0.0 \
  enable_slosh_box_constraint:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  risk_scheduler_enable:=false \
  slosh_speed_governor_enable:=false \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

B 组 SOFT_SLOSH_ONLY：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_fixed \
  mpc_Q_lag:=0.5 \
  mpc_Q_contour:=32.0 \
  mpc_Q_etheta:=15.0 \
  mpc_Q_v:=9.0 \
  mpc_R_a:=0.4 \
  mpc_R_omega:=2.0 \
  mpc_R_da:=0.5 \
  mpc_R_domega:=4.0 \
  Q_slosh:=5.0 \
  slosh_height_ref:=0.005 \
  slosh_eta_dot_ratio:=0.3 \
  Q_slosh_eta_dot:=0.0 \
  terminal_factor_slosh_eta:=0.0 \
  terminal_factor_slosh_eta_dot:=0.0 \
  enable_slosh_box_constraint:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  risk_scheduler_enable:=false \
  slosh_speed_governor_enable:=false \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

C 组 SMOOTH_SPEED_RELAXED：

```bash
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
  enable_slosh_box_constraint:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  risk_scheduler_enable:=false \
  slosh_speed_governor_enable:=false
```

D 组 SLOSH_PRIORITY_MPC：

```bash
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
  enable_slosh_box_constraint:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  risk_scheduler_enable:=false \
  slosh_speed_governor_enable:=false
```

E 组 SLOSH_DOMINANT：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_fixed \
  mpc_Q_lag:=0.5 \
  mpc_Q_contour:=24.0 \
  mpc_Q_etheta:=10.0 \
  mpc_Q_v:=2.0 \
  mpc_R_a:=0.6 \
  mpc_R_omega:=2.0 \
  mpc_R_da:=2.0 \
  mpc_R_domega:=8.0 \
  Q_slosh:=10.0 \
  slosh_height_ref:=0.005 \
  slosh_eta_dot_ratio:=0.3 \
  Q_slosh_eta_dot:=0.0 \
  terminal_factor_slosh_eta:=8.0 \
  terminal_factor_slosh_eta_dot:=8.0 \
  enable_slosh_box_constraint:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  risk_scheduler_enable:=false \
  slosh_speed_governor_enable:=false
```

如果不使用固定路径 replay，而是订阅普通导航路径，则去掉 `global_path_topic:=/scout/global_path_fixed`，保持默认 `/scout/global_path`。

启动后必须核对实际参数：

```bash
rosparam get /scout_local_planner/mpc/Q_lag
rosparam get /scout_local_planner/mpc/Q_contour
rosparam get /scout_local_planner/mpc/Q_etheta
rosparam get /scout_local_planner/mpc/Q_v
rosparam get /scout_local_planner/mpc/R_a
rosparam get /scout_local_planner/mpc/R_omega
rosparam get /scout_local_planner/mpc/R_da
rosparam get /scout_local_planner/mpc/R_domega
rosparam get /scout_local_planner/mpc/Q_slosh
rosparam get /scout_local_planner/mpc/slosh_height_ref
rosparam get /scout_local_planner/mpc/slosh_eta_dot_ratio
rosparam get /scout_local_planner/mpc/Q_slosh_eta_dot
rosparam get /scout_local_planner/mpc/terminal_factor_slosh_eta
rosparam get /scout_local_planner/mpc/terminal_factor_slosh_eta_dot
rosparam get /scout_local_planner/mpc/N
rosparam get /scout_local_planner/mpc/dt
rosparam get /scout_local_planner/control_rate
rosparam get /scout_local_planner/terminal_capture_stop/enable
rosparam get /scout_local_planner/terminal_capture_stop/distance
```

### 7.3.2 可选：回退到当前默认 BASE

如果只想确认裸启动默认参数，可运行：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_fixed \
  Q_slosh:=0.0 \
  slosh_height_ref:=0.005 \
  slosh_eta_dot_ratio:=0.3 \
  Q_slosh_eta_dot:=0.0 \
  enable_slosh_box_constraint:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  risk_scheduler_enable:=false \
  slosh_speed_governor_enable:=false
```

该命令依赖 `mpc_params.yaml` 默认：

```text
  mpc_Q_contour:=32.0 \
  mpc_Q_etheta:=15.0 \
  mpc_Q_v:=9.0 \
  mpc_R_a:=0.4 \
  mpc_R_da:=0.5 \
  mpc_R_domega:=4.0 \
```

### 7.4 replay 固定路径

```bash
rosrun scout_local_planner fixed_global_path_runner.py \
  --mode replay \
  --path-file /home/geist/fixed_paths/real/slosh_priority_p2_s.json \
  --output-topic /scout/global_path_fixed \
  --manual-start \
  --start-pos-tol 0.08 \
  --start-yaw-tol 0.15 \
  --publish-once-keepalive
```

### 7.5 录包命名

建议命名：

```text
slosh_mpc_A_BASE_p2_run01.bag
slosh_mpc_B_SOFT_SLOSH_ONLY_p2_run01.bag
slosh_mpc_C_SMOOTH_SPEED_RELAXED_p2_run01.bag
slosh_mpc_D_SLOSH_PRIORITY_MPC_p2_run01.bag
slosh_mpc_E_SLOSH_DOMINANT_p2_run01.bag
```

### 7.6 终点过冲诊断脚本

如果出现“超过终点后调头”，先不要直接改 `Q_slosh`。该现象通常来自终点逻辑，例如：

```text
1. 车已经越过 goal，terminal recovery 进入 ALIGN_TO_POINT；
2. 终点前 reference/v_ref 或 cmd_vel 仍偏高；
3. 位置已经到达，但终点 yaw 不满足，导致姿态修正；
4. near-goal solver failure 或代价占比异常。
```

实时监测：

```bash
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/analysis/diagnose_terminal_overshoot.py
```

离线分析 bag：

```bash
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/analysis/diagnose_terminal_overshoot.py \
  --bag /home/geist/slosh_bags/real/xxx.bag
```

如果是在 `/home/a/scout_ws` 机器上运行，把路径改成：

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/analysis/diagnose_terminal_overshoot.py \
  --bag /data/a/slosh_bags/real/xxx.bag
```

脚本读取：

```text
/terminal/goal_info
/terminal/mode
/terminal/recovery_latched
/cmd_vel
/odom
/reference/v_ref
/mpc/status_val
/mpc/cost_breakdown
/mpc/slosh_horizon_summary
```

输出原因标签：

```text
GOAL_PASSED_THEN_TERMINAL_TURNBACK
REFERENCE_SPEED_TOO_HIGH_NEAR_GOAL
EXECUTION_SPEED_TOO_HIGH_NEAR_GOAL
ENDPOINT_YAW_MISMATCH
LARGE_TERMINAL_BEARING
SOLVER_FAILURE_NEAR_GOAL
TERMINAL_RECOVERY_ACTIVE
```

关键判断：

```text
dx < -0.05 且 terminal/mode = ALIGN_TO_POINT
```

这表示小车已经越过终点，terminal recovery 正在调头对准 goal。此时应优先检查终点速度剖面、固定路径终点姿态和 terminal recovery 参数，而不是把问题归因到晃动项。

### 7.7 终点 terminal MPC stop 调参

默认终点策略不再让外层 capture stop 直接发布制动命令。进入 `terminal_capture_stop_distance` 后，系统只进入 `TERMINAL_MPC_STOP`，由 MPC 继续低速收敛。如果仍出现过冲，优先检查 terminal 参考速度和 odom 速度是否连续下降，而不是继续增大 `Q_slosh`。

更早捕获：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  terminal_capture_stop_enable:=true \
  terminal_capture_stop_distance:=0.90
```

最保守捕获，接近 `mpc_planner` 的 1m 到达口径：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  terminal_capture_stop_enable:=true \
  terminal_capture_stop_distance:=1.00
```

如果停得明显过早：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  terminal_capture_stop_enable:=true \
  terminal_capture_stop_distance:=0.50
```

实验归因口径：

```text
terminal MPC stop 用于任务完成和低速收敛验收。
NEAR_GOAL / TERMINAL_MPC_STOP 段不参与 slosh cost 主归因。
slosh 主结论只看 TRACKING_PRE_TERMINAL。
```

如果 `terminal_capture_stop_distance:=1.00` 仍过冲，优先检查：

```text
1. 当前运行节点是否确实使用了最新 devel 编译产物；
2. /reference/v_des_eff 和 /reference/v_ref_horizon 是否在 terminal 前连续下降；
3. /cmd_vel 和 /odom 是否跟随参考速度连续下降；
4. 底盘执行 cmd_vel 是否有明显滞后；
5. /odom 或 map->base_link 是否有延迟/漂移；
6. 固定路径终点是否在真实目标后方。
```

## 8. 必录 topic

基础控制：

```text
/cmd_vel
/odom
/tf
/tf_static
/scout/global_path_fixed
/scout/global_path_smooth
/mpc/status_val
/mpc/solve_ms
/mpc/cost_breakdown
```

MPC / slosh 调试：

```text
/slosh/state
/slosh/height
/slosh/eta_norm
/slosh/eta_dot_norm
/slosh/modal_energy
/slosh/modal_energy_norm
/slosh/ax_est
/slosh/ay_est
/slosh/alpha_est
/slosh/q_slosh_eta
```

视觉真值：

```text
/camera/color/image_raw
/camera/color/camera_info
```

如使用压缩图像，必须记录对应 image transport 话题，并在视觉脚本中明确输入源。

## 9. 离线分析指标

### 9.1 tracking

```text
lateral RMSE
lateral p95
heading RMSE
final position error
completion time
manual takeover count
```

最低接受条件：

```text
completion_time <= BASE * 1.30
lateral_p95 <= BASE * 1.30
无碰撞
无人工接管
```

说明：

```text
本方案允许 slosh-priority MPC 适度变慢；变慢本身不是失败。
但如果 completion_time 明显超过 BASE * 1.30，主结论应写成 trade-off，而不是稳定优于 baseline。
```

### 9.2 control / excitation

```text
max |v|
mean |v|
max |omega|
RMS |omega|
max |a_x|
RMS |a_x|
max |a_y|
RMS |a_y|
max |delta a_x|
max |delta omega|
```

关键指标：

```text
max |a_y|
RMS |a_y|
max |a_x|
max |delta omega|
```

### 9.3 RGB external slosh truth

使用红液固定流程输出：

```text
RGB height peak
RGB height p95
RGB height RMS
high-slosh frame ratio
near-goal last 3 s RGB peak / p95
```

注意：

```text
RGB 是实物液面主指标；
/slosh/height 是模型内部指标，不能作为主结论。
```

### 9.4 model-side diagnostics

```text
/slosh/height peak / p95 / RMS
eta_dot RMS / p95
modal_energy RMS / p95
```

这组只用于解释模型是否和 RGB 趋势一致。

### 9.5 solver

```text
solve_ms median / p95
status success ratio
failure count
fallback / recovery count
```

最低接受条件：

```text
solve_success_ratio >= 0.97
无连续 solver failure
solve_ms p95 明显低于 33 ms 控制周期预算
```

## 10. Cost Contribution Check

为避免归一化晃动项仍然不可见或过度主导，正式实验前应做一次 cost contribution 诊断。

当前控制器已发布 MPC 内部预测 horizon 上的代价分项：

```text
/mpc/cost_breakdown
```

消息类型：

```text
std_msgs/Float32MultiArray
```

字段顺序：

```text
0  J_total
1  J_lag
2  J_contour
3  J_etheta
4  J_v
5  J_omega_ff
6  J_control
7  J_smooth
8  J_slosh_eta
9  J_slosh_eta_dot
10 pct_lag
11 pct_contour
12 pct_etheta
13 pct_v
14 pct_omega_ff
15 pct_control
16 pct_smooth
17 pct_slosh_eta
18 pct_slosh_eta_dot
19 pct_slosh_total
```

检查命令：

```bash
rostopic echo /mpc/cost_breakdown
```

录包脚本已包含该 topic：

```text
src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh
```

离线提取和画图脚本：

```text
src/scout_apps/control/scout_local_planner/scripts/analysis/extract_mpc_cost_breakdown.py
```

实物机常用命令：

```bash
source /home/geist/scout_ws/devel/setup.bash
python3 /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/analysis/extract_mpc_cost_breakdown.py \
  /home/geist/slosh_bags/real/xxx.bag \
  --phase TRACKING \
  --out-dir /home/geist/slosh_bags/real/xxx_cost_breakdown
```

开发机常用命令：

```bash
source /home/a/scout_ws/devel/setup.bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/analysis/extract_mpc_cost_breakdown.py \
  /data/a/slosh_bags/real/xxx.bag \
  --phase TRACKING \
  --out-dir /data/a/slosh_bags/real/xxx_cost_breakdown
```

多包合并对比：

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/analysis/extract_mpc_cost_breakdown.py \
  /data/a/slosh_bags/real/run_A.bag \
  /data/a/slosh_bags/real/run_C.bag \
  /data/a/slosh_bags/real/run_D.bag \
  --phase TRACKING \
  --out-dir /data/a/slosh_bags/real/cost_breakdown_compare \
  --prefix A_C_D_cost_contribution
```

输出文件：

```text
cost_contribution.csv
cost_contribution_summary.md
cost_contribution_timeseries.png
cost_contribution_summary_bar.png
```

注意：

```text
旧 bag 如果没有录到 /mpc/cost_breakdown，不能严格还原 MPC 内部每项 cost 占比。
这种 bag 只能用速度、加速度、轨迹误差和 RGB 视觉液面做外部行为分析。
```

### 10.7 Horizon 内 eta 预测诊断

`/mpc/slosh_horizon_summary` 用于判断 MPC horizon 内是否真的预测出了未来晃动，而不是只看当前 `/slosh/state`。

字段顺序：

```text
0  eta_norm_0_m
1  eta_norm_max_m
2  eta_dot_norm_0_mps
3  eta_dot_norm_max_mps
4  h_modal_max_mm
5  h_total_max_mm
6  k_h_total_max
7  v_abs_p95_mps
8  omega_abs_p95_radps
9  ax_abs_p95_mps2
10 ay_abs_p95_mps2
11 eta_growth_ratio
12 h_total_0_mm
```

检查命令：

```bash
rostopic echo /mpc/slosh_horizon_summary
```

判断：

```text
eta_growth_ratio < 1.1 且 h_total_max_mm 很小:
  horizon 没有预测出明显未来晃动，先查 slosh 状态注入、动力学矩阵和参考序列激励。

eta_growth_ratio 明显 > 1，但 pct_slosh_total 仍 < 1%:
  horizon 内 eta 已经预测出来，主要问题是 slosh cost 尺度太小。

eta_growth_ratio 明显 > 1，pct_slosh_total 到 3%-20%，但 RGB 未下降:
  晃动模型方向、模型保真度或控制权衡仍有问题，不能用 /slosh/height 自证。
```

D 组期望：

```text
J_slosh_eta + J_slosh_eta_dot 可见
J_v 相对 A/B 下降
J_smooth 占比上升
J_contour 仍保持主要路径约束作用
```

更具体的第一轮数值判断：

```text
pct_slosh_total 长期 < 1%:
  晃动项在优化器里基本不可见，即使 Q_slosh 已打开，也很难期待行为变化。

pct_slosh_total 大约 3%-20%:
  晃动项有可见影响，适合作为第一轮有效区间。

pct_slosh_total 长期 > 40%:
  晃动项可能过度主导，需检查 tracking、completion time 和奇怪慢行。
```

异常判断：

```text
J_slosh 长期 < 1%:
  晃动项可能没有实际影响。

J_slosh 长期 > 40%:
  晃动项可能过度主导，需检查 tracking 和奇怪绕行/慢行。

J_smooth 明显上升但 D 不优于 C:
  主要收益来自控制平滑，不应声称 slosh 模态项有增量价值。
```

## 11. 判定逻辑

### 主判定窗口

为避免终点平滑减速、goal stop、terminal recovery 污染“slosh 项是否生效”的判断，主有效性结论只使用进入 `terminal_slowdown` 前的数据：

```text
TRACKING_PRE_TERMINAL:
  /mpc_status == TRACKING
  goal_dist > terminal_slowdown_distance
```

该窗口用于证明：

```text
slosh-priority MPC 在正常路径跟踪阶段已经改变了速度/加速度激励和 RGB 液面响应。
```

终点附近数据单独作为补充窗口：

```text
NEAR_GOAL:
  goal_dist <= terminal_slowdown_distance
```

`NEAR_GOAL` 只能用于说明终点策略是否平稳，不能作为 slosh cost 主证明。所有 A/B/C/D/E 分组仍必须使用同一套 `terminal_capture_stop`、`terminal_slowdown`、position/yaw 解耦和 terminal recovery 逻辑。

### 支持 slosh-priority MPC 的结果

```text
D 相对 A，在 TRACKING_PRE_TERMINAL 内:
  RGB peak / p95 下降
  max |a_y| 或 RMS |a_y| 下降
  max |a_x| 或 RMS |a_x| 不上升
  tracking_time <= A * 1.30
  lateral_p95 <= A * 1.30

D 相对 C，在 TRACKING_PRE_TERMINAL 内:
  RGB peak / p95 仍有额外下降
  或 max/RMS ax/ay 进一步下降

NEAR_GOAL:
  只作为补充结果报告，不参与 slosh cost 主归因。
```

此时可以写：

```text
slosh-aware objective rebalancing provides additional reduction beyond speed relaxation and control smoothing.
```

### 只支持平滑/降速的结果

```text
C 明显优于 A
D 与 C 持平
```

结论应写成：

```text
主要收益来自速度跟踪松弛和控制平滑；
当前 slosh surrogate 在 MPC cost 中没有体现稳定增量。
```

### 不支持 MPC cost 路线的结果

```text
B/D/E 的 RGB 液面不降，或 tracking 明显恶化；
或 /slosh/height 下降但 RGB 不降。
```

结论应写成：

```text
模型内部 slosh cost 可以改变控制行为，
但未转化为真实 RGB 液面收益。
```

### E 组解释

```text
E 有效但时间代价大:
  只能作为 slosh-priority trade-off 上限。

E 仍无效:
  不继续盲目加大 Q_slosh。
  优先回到模型保真度和 reference-first 路线。
```

## 12. 论文表述边界

可以写：

```text
We evaluate a slosh-priority MPC objective that relaxes speed tracking and increases control smoothness while penalizing predicted modal displacement and velocity.
```

谨慎写：

```text
The slosh-priority tuning reduces excitation and RGB-observed liquid oscillation under the tested fixed-path conditions.
```

不要写：

```text
Q_slosh alone guarantees anti-sloshing.
The internal /slosh/height proves real liquid suppression.
Acceleration constraints always reduce sloshing.
```

如果 D 组只在部分路径有效，应写：

```text
The approach is effective in selected fixed-path maneuvers but remains sensitive to model fidelity and execution mismatch.
```

## 13. 第一轮最小执行清单

第一轮不要直接跑全矩阵。建议：

```text
1. 确认 slosh_experiment.launch 参数覆盖和 30 Hz 时基生效。
2. 固定一条 P2_REAL_S 路径。
3. 每组先录 1 包 A/C/D smoke。
4. 若 A/C/D 均安全完成，再补 A/B/C/D 各 3 包。
5. 只有 D 明显优于 C，才跑 E。
6. 若 P2 成立，再换 P3_REAL_MIXED。
```

停止条件：

```text
连续 2 包出现明显 tracking 不可接受、人工接管或 solver failure，
停止该参数组，不继续加大 Q_slosh。

30 Hz 下若 solve_ms p95 接近或超过 33 ms，先停止正式录包，回查 N=60 的求解负载。
```
