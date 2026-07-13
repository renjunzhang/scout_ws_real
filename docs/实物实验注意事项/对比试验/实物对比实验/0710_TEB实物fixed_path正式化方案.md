# 0710 TEB 实物 fixed-path 正式化方案

> 目的：在不等待 Slosh-risk Governor 实物消融的前提下，先把一个普通外部 local planner baseline 做到可复现、可量化、可进入 formal N=3。本文以 `TEB-noobs-fixed` 为当前首选，并把 0706 的 loose_fast 调参方向固化为受控的候选起点。
>
> 2026-07-13 更新：补充 0712 TEB/B_ours 实物结果的公平性解释、tracking-matched
> T0/T1 流程，以及 standalone slosh monitor 的自动启动、reset、运行期存活检查和
> NO-GO 口径。
>
> 0706 原始现场记录仍保留在：
>
> ```text
> docs/实物实验注意事项/对比试验/实物对比实验/0706_TEB实物fixed_path参数调试记录.md
> ```

---

## 1. 当前实验定位

当前顺序保持为：

```text
已完成内部消融
  -> 完成一个 governor-off 外部 baseline formal N=3
  -> 后续单独做 B_ours governor off/on 消融
```

本组外部对照使用：

```text
SPMPC side: B_ours, governor disabled
External side: TEB-noobs-fixed
Scene: 同一无障碍 fixed S-curve
RGB: 只离线评价，不进入控制回路
```

本组只回答：普通 local planner 不传播液体状态时，在相近任务完成质量下能否获得同样的真实液面表现。Governor 不属于本组自变量。

---

## 2. 为什么正式主表使用 no-obstacle TEB

当前 SPMPC continuous fixed-path 主线配置为：

```text
obstacle_enable=false
corridor_enable=false
homotopy_enable=false
```

因此让 TEB 使用 scan obstacle/inflation，而让 SPMPC 忽略障碍，会把不同环境代价混入比较。当前正文主表应使用：

```text
TEB-noobs-fixed
```

并明确其含义是“普通在线局部轨迹优化器的无障碍固定路径对照”。若后续需要 collision-aware TEB，只能另开实验；在 SPMPC 未加入同口径 collision handling 前，不与本组混表。

---

## 3. 当前 source-of-truth 配置

不要再从 `/tmp` 重建配置。当前版本化文件为：

```text
TEB planner:
src/scout_apps/control/spmpc_experiments/config/baselines/teb_local_planner_fixed_path_real_noobs.yaml

No-obstacle costmap:
src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_real_no_obstacles.yaml

One-click runner:
src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh

Metric extractor:
src/scout_apps/control/spmpc_experiments/scripts/extract_fixed_path_paper_metrics.py
```

一键脚本在 `METHOD=teb` 时已默认选择上述 real/no-obstacle 配置。不要省略 run metadata；每个 bag 必须能追溯 git commit、planner config、costmap config 和限制参数。

---

## 4. 固定路径与现场口径

```text
PATH_TEMPLATE=s_curve
GOAL_X=-5.424
GOAL_Y=-4.736
GOAL_YAW=0.0
PATH_START_HEADING=current
PATH_AMPLITUDE_RATIO=0.18
PATH_MIN_AMPLITUDE=0.25
PATH_MAX_AMPLITUDE=1.20
PATH_SIDE=left
PATH_SMOOTH_ITERATIONS=3
PATH_SPACING=0.05
```

现场固定：

```text
同一容器、液位和固定方式；
同一相机位置、曝光、ROI、标定和光照；
同一定位栈、地图、TF 与底盘模式；
液体每次静稳 60-90s；
任一时刻只允许一个 /cmd_vel publisher；
E-stop / 遥控急停就位。
```

---

## 5. 公平性与限制口径

当前候选 smoke 使用：

```text
MAX_V=0.30
MAX_W=1.20
MAX_ACC=0.60
MAX_ANGULAR_ACC=1.20
XY_GOAL_TOL=0.20
YAW_GOAL_TOL=0.30
CONTROLLER_FREQUENCY=10.0
```

其中：

```text
MAX_V:
  同时覆盖 TEB 内部 max_vel_x/max_vel_trans 和 runner 最终 clamp；

MAX_W:
  同时覆盖 TEB 内部 max_vel_theta 和 runner 最终 clamp；

MAX_ACC / MAX_ANGULAR_ACC:
  覆盖 TEB 内部 acc_lim_x/acc_lim_theta；

/baseline/teb/raw_cmd_vel:
  TEB 插件原始命令；

/baseline/teb/command_intervention:
  runner 前后命令和速度 clamp 标记；

/baseline/teb/tracking_error:
  使用 TF 在 map/path frame 中计算的距离、航向误差和路径进度。
```

`MAX_V=0.30` 是安全的第一版 smoke 起点，不是未经试验就冻结的论文参数。当前历史 `B_ours` 数据使用 `v_ref=0.20`、机器人硬上限高于 0.30，因此若最终保持 TEB 的 0.30 上限，论文必须同时报告实际 `cmd_v mean/p95` 和 goal time，表述为任务表现/实际速度匹配，而不能声称两者内部速度参数完全同构。

如果需要“严格相同线速度硬上限”的额外表，应先增加两方法共用的输出速度 cap，再同日重跑 `B_ours` bridge；不要把该问题留到结果出来后再调整。

0712 实物数据进一步说明，`tracking p95 <=0.30m` 只能作为“车辆安全完成任务”的宽松
gate，不能自动代表两种方法具有相近的路径跟踪质量：

```text
TEB tracking p95:           约 0.154m
B_ours governor-off p95:    约 0.0384m
TEB / B_ours:               约 4.0x

TEB |cmd_omega| p95:        约 0.159rad/s
B_ours |cmd_omega| p95:     约 0.212rad/s

TEB cmd_v p95:              0.300m/s
B_ours cmd_v p95:           约 0.428m/s
```

因此当前 TEB 较低的 model slosh peak 很可能同时来自：

```text
1. via-point 跟踪较弱、弯道允许切弯，实际路径曲率更小；
2. 转向角速度 p95 更低，横向激励 |v*omega| 更小；
3. 线速度基本固定在 0.30m/s，没有 B_ours 的速度上冲；
4. TEB 优化目标并不要求像 B_ours 一样同时紧跟 fixed path。
```

当前配置没有一个直接名为“允许 tracking error”的控制参数。`XY_GOAL_TOL` 只作用于
终点，不决定全程跟踪。更相关的是 `weight_viapoint=2.0` 偏弱、
`global_plan_viapoint_sep=0.70m` 较稀，以及较长的 lookahead。正式结论必须同时报告
tracking、实际命令、到点时间和 slosh，不能只按 peak 排序。

---

## 6. TEB 候选 v1

当前候选只把 0706 loose_fast 方向固化，不继续一次改九个参数：

| 参数 | v1 |
|---|---:|
| `dt_ref` | `0.30` |
| `dt_hysteresis` | `0.08` |
| `max_global_plan_lookahead_dist` | `4.0` |
| `global_plan_prune_distance` | `0.60` |
| `global_plan_viapoint_sep` | `0.70` |
| `weight_viapoint` | `2.0` |
| `weight_optimaltime` | `0.60` |
| `weight_acc_lim_theta` | `50.0` |
| `penalty_epsilon` | `0.02` |
| `max_vel_x` | 由 `MAX_V` 覆盖，首轮 `0.30` |
| `max_vel_trans` | 由 `MAX_V` 覆盖，首轮 `0.30` |
| `max_vel_theta` | 由 `MAX_W` 覆盖，首轮 `1.20` |
| `acc_lim_x` | `0.60` |
| `acc_lim_theta` | `1.20` |

注意：`weight_acc_lim_theta` 主要提高对角加速度约束违反的惩罚，不等于独立的角速度平滑/jerk 代价。减少摆头首先检查 via-point 密度、路径剪枝、lookahead 和原始命令，不要只靠降低角速度硬上限把问题遮住。

### 6.1 0712 结果定位和 tracking-matched 候选

0712 v1 的三次 TEB 均成功到点且无人工接管，但 tracking p95 约 `0.154m`，明显松于
同日 B_ours governor-off 的 `0.0384m`。因此这组三次运行可以证明：

```text
TEB 能在当前 fixed-path 实物链路稳定到点；
在宽松 tracking gate 下，它的模型高度较低。
```

但不能证明：

```text
TEB 在相同 fixed-path 跟踪质量下仍具有同样的低 slosh peak。
```

下一候选按单变量方式定义为：

```text
T0:
  penalty_epsilon=0.02
  weight_viapoint=2.0
  MAX_V=0.30

T1:
  penalty_epsilon=0.02
  weight_viapoint=3.0
  MAX_V=0.30
```

对应 T1 文件：

```text
src/scout_apps/control/spmpc_experiments/config/baselines/
teb_local_planner_fixed_path_real_noobs_track_t1.yaml
```

注意：Git diff 中基础配置曾把 `penalty_epsilon: 0.08 -> 0.02`。它会将速度软惩罚开始
作用的位置从约 `0.22m/s` 移到约 `0.28m/s`，不是无关修改。运行 T1 前必须检查
0712 bag 配套的 `*_rosparam.yaml`：

```text
若 0712 实际已经是 0.02：可将对应 v1 作为 T0；
若 0712 实际是 0.08：先补跑 T0，再运行 T1；
不得把 epsilon 不同的 run 宣称为只改变 weight_viapoint 的单变量对照。
```

T1 选择标准不以 slosh peak 为依据，先看任务质量：

```text
GOAL_REACHED；
tracking p95 <=0.08m；
无持续摆头、NO_VALID_CMD 或人工接管；
runner limiter fraction <1%；
MAX_V 仍保持 0.30m/s。
```

收紧 tracking 后，TEB 的角速度、到点时间和 slosh peak 可能上升，这是公平性校正的
预期结果，不应因为 peak 变差就回退到切弯更多的参数。

---

## 7. 执行阶段

### 7.1 环境与 preflight

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

rostopic info /cmd_vel
rostopic echo -n 1 /odom
rostopic echo -n 1 /camera/color/image_raw
rostopic echo -n 1 /tf
```

actuated 前 `/cmd_vel` 不得存在旧 planner publisher。

### 7.2 Standalone slosh monitor

一键脚本现在明确区分两个开关：

```text
START_STANDALONE_SLOSH=true（默认）：
  启动 slosh_models/slosh_monitor.launch；
  等待 /slosh/height；
  在路径、录包和 planner 启动前调用 /slosh/reset；
  保存独立 monitor log，并在退出时清理进程。

RECORD_STANDALONE_SLOSH=true：
  只让 record_spmpc_full_rgb_bag.sh 把 /slosh/* 加入录包 topic；
  它控制“是否录制”，不代替 START_STANDALONE_SLOSH 的生命周期管理。
```

正式命令显式写出：

```bash
START_STANDALONE_SLOSH=true \
RECORD_STANDALONE_SLOSH=true \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

默认 monitor 固定使用：

```text
odom_topic=/odom；
cmd_vel_topic=/cmd_vel（仅 debug，模型由 odom 驱动）；
output_namespace=/slosh；
R=0.0185m；h=0.058m；zeta=0.05；
use_parabola_term=false；
model_dt=0.02；
accel_filter_alpha=0.3；min_dt=0.001；max_dt=0.1。
```

脚本的强制 gate：

```text
检测到旧 /slosh/slosh_monitor 时拒绝启动，避免重复 publisher；
启动后规定时间内收不到 /slosh/height，停止实验；
/slosh/reset 调用失败，停止实验；
脚本拥有的 monitor 在录包结束前退出，本次 run 判失败；
monitor 参数、topic 和 eval-only 口径写入 external_baseline_meta.env；
外部 baseline 不订阅 /slosh/*，monitor 只用于统一外部评价。
```

如确需复用人工启动的 monitor，可显式设置 `START_STANDALONE_SLOSH=false`；此时脚本仍会
要求 `/slosh/height` 和 `/slosh/reset` 可用，但不会负责外部进程的退出清理。正式 formal
优先使用默认的自动管理模式。

### 7.3 Shadow

```bash
DATE=20260710 \
METHOD=teb \
STAGE=shadow \
RUN_LABEL=TEB_noobs_v1_0710_shadow01 \
MAX_V=0.30 \
MAX_W=1.20 \
MAX_ACC=0.60 \
MAX_ANGULAR_ACC=1.20 \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_RGB=false \
START_STANDALONE_SLOSH=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_TOPIC_INFO=true \
RECORD_SEC=30 \
MAX_RECORD_SEC=30 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

Shadow 通过条件：

```text
/baseline/teb/status 能进入 TRACKING；
/baseline/teb/global_plan 是同一 fixed S-curve；
/baseline/teb/raw_cmd_vel 连续发布；
/baseline/teb/tracking_error 的 frame 口径有效；
/slosh/height 在 planner 启动前已经存在且连续发布；
无 SET_PLAN_FAILED / NO_VALID_CMD 长时间持续；
/cmd_vel 没有被 shadow planner 发布。
```

### 7.4 Actuated N=1 smoke

```bash
DATE=20260710 \
METHOD=teb \
STAGE=actuated \
RUN_LABEL=TEB_noobs_v1_0710_smoke01 \
MAX_V=0.30 \
MAX_W=1.20 \
MAX_ACC=0.60 \
MAX_ANGULAR_ACC=1.20 \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_RGB=true \
START_STANDALONE_SLOSH=true \
RECORD_STANDALONE_SLOSH=true \
RECORD_TOPIC_INFO=true \
RECORDER_STARTUP_SEC=8 \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

手在急停上。首轮不同时修改 speed、via-point 和 angular limit。

---

## 8. N=1 量化 gate

现有 `B_ours` 参考完成时间约 `38.1s`。TEB N=1 首先通过安全/完整性门槛：

```text
GOAL_REACHED；
无人工接管 / E-stop / 定位跳变；
goal time <= 57s（约 B_ours 的 1.5 倍）；
tracking p95 <= 0.30m；
无持续左右摆头、急停或长时间 NO_VALID_CMD；
linear/angular limited fraction 不长期非零，建议 < 1%；
RGB 覆盖运动开始 -> first GOAL_REACHED，并保留 post-goal 残余段；
odom / tf / fixed path / cmd / baseline diagnostics 完整；
/slosh/height、/slosh/state、/slosh/debug 从运动前开始连续记录；
rosbag 在 path progress 10%~90% 核心段无 buffer exceeded。
```

但进入“tracking-matched 正式比较”的门槛进一步收紧为：

```text
tracking p95 <=0.08m；
同时报告与 B_ours 当日 tracking p95 的差值；
参数选择先依据 tracking、成功和稳定性，不依据 slosh peak；
若只能满足 0.08m~0.30m，则保留为 tracking 不匹配的描述性 external baseline。
```

`0.08m` 不是声称两种控制器已经完全同构，而是避免继续用约 `0.154m` 的明显切弯结果
支撑“同路径质量下 peak 更低”的强结论。正式表仍必须把连续 tracking 指标并列报告。

提取控制、tracking 和 limiter 指标：

```bash
python3 src/scout_apps/control/spmpc_experiments/scripts/extract_fixed_path_paper_metrics.py \
  /home/geist/slosh_bags/real/20260710_fixed_path_compare/teb \
  --phase windows \
  --csv /home/geist/slosh_bags/real/20260710_fixed_path_compare/teb/teb_noobs_v1_metrics.csv
```

RGB 仍使用统一离线 max-LCR 流程，不使用在线 `/liquid/*` 作为控制或主真值。

录包脚本虽然已把 rosbag buffer 默认提高到 `4096MB`，但大 buffer 只能吸收瞬时拥塞。
正式补跑前先使用相同 RGB/topic 集合原地录制 60 秒，确认 recorder log 无
`buffer exceeded`。出现核心段 drop 的 run 不能通过增加 buffer 后“修复”，只能重跑。

---

## 9. 失败后的单变量调整阶梯

### 9.1 稳定但慢

保持路径参数不变，只改速度轴：

```text
v1: MAX_V=0.30
v2: MAX_V=0.35
v3: MAX_V=0.40（仅在现场安全和前两档稳定时）
```

每档先 N=1，不能直接把更快参数带入 formal。

每次速度扫描在 planner 启动后都必须确认：

```text
/baseline_local_planner_runner/TebLocalPlannerROS/max_vel_x=MAX_V；
/baseline_local_planner_runner/TebLocalPlannerROS/max_vel_trans=MAX_V；
/baseline_local_planner_runner/max_cmd_vel_x=MAX_V；
实际 cmd_v p95 确实突破上一档，而不是被隐藏上限继续截住。
```

### 9.2 左右摆头

先看：

```text
raw_cmd_vel 是否已经摆头；
command_intervention 是否长期 clamp；
tracking_error 是否在路径两侧周期性交替；
```

然后一次只改一个：

```text
weight_viapoint: 2.0 -> 1.5
或
global_plan_viapoint_sep: 0.70 -> 0.80
或
global_plan_prune_distance: 0.60 -> 0.80
```

不要同时降低 `MAX_W`、降低 `acc_lim_theta`、增大 viapoint sep 和提高速度，否则无法判断改善来自哪里。

### 9.3 切弯/偏路径过大

0712 已确认 v1 tracking p95 约 `0.154m`，因此下一步优先执行预注册 T0/T1，不先做速度
扫描，也不根据 slosh peak 选择参数：

```text
T0: penalty_epsilon=0.02, weight_viapoint=2.0, MAX_V=0.30
T1: penalty_epsilon=0.02, weight_viapoint=3.0, MAX_V=0.30
```

运行时显式指定版本化配置，不能临时修改同一个 YAML：

```text
T0 PLANNER_CONFIG:
src/scout_apps/control/spmpc_experiments/config/baselines/
teb_local_planner_fixed_path_real_noobs.yaml

T1 PLANNER_CONFIG:
src/scout_apps/control/spmpc_experiments/config/baselines/
teb_local_planner_fixed_path_real_noobs_track_t1.yaml
```

若 T1 仍不能把 tracking p95 压到 `0.08m`，下一次只向相反方向恢复一个参数：

```text
weight_viapoint: 3.0 -> 4.0（T2，仅在 T1 稳定但 tracking 仍偏松时）
或
固定 weight_viapoint 后，仅改 global_plan_viapoint_sep: 0.70 -> 0.60
```

不能在同一个 candidate 中同时提高 via-point 权重、缩短 via-point 间距、提高速度和降低
lookahead。tracking p95 超过 `0.30m` 的 run 不能进入安全主表；超过 `0.08m` 的 run
不能宣称已经完成 tracking-matched 比较。

### 9.4 Timebox

最多尝试 2-3 个有明确单变量差异的 candidate。如果仍不能同时满足到点、tracking、时间和摆头门槛，停止继续围绕 TEB 大规模调参，转 `mpc_local_planner`。外部 baseline 是补充证据，不能吞掉 governor 和主方法实验时间。

---

## 10. 参数冻结与 formal N=3

N=1 通过后：

```text
1. 将最终 YAML 另存为带 final/frozen 名称的版本化文件；
2. 记录 git commit；
3. 固定 path、goal、limits、goal tolerance、controller frequency；
4. 固定 standalone monitor 的 R/h/zeta/model_dt/filter/parabola 口径；
5. 每个 run 在运动前 reset monitor，并确认 /slosh/height 已发布；
6. formal 过程中不得继续调参数；
7. 调参 bag 全部保留，但不进入主统计。
```

同日必须运行 governor-off `B_ours` bridge。推荐交错顺序：

```text
Round 1: B_ours_off -> TEB
Round 2: TEB -> B_ours_off
Round 3: B_ours_off -> TEB
```

`B_ours` bridge 显式沿用已完成内部消融口径：

```text
VARIANT=B_ours
V_REF=0.20
ALPHA_MAX=1.2
DELAY_PHASE_MODE=fixed_closed_loop
DELAY_PHASE_LINEAR_DELAY_SEC=0.15
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22
hard constraint disabled
governor disabled
```

正式表至少报告：

```text
success rate；
goal time；
actual cmd_v mean/p95；
actual |cmd_omega| p95/max；
tracking RMS/p95；
runner limiter fraction；
standalone model height peak/p95/RMS；
RGB H_vis peak/p95/RMS。
```

---

## 11. 无效 run

以下情况保留 bag，但不进入 clean 主表：

```text
未 GOAL_REACHED；
人工接管或 E-stop；
旧 /cmd_vel publisher 污染；
TF / map / odom 跳变；
RGB 丢失或 ROI 不可用；
容器液位、相机姿态或光照条件改变；
调参过程中临时修改 YAML；
runner clamp 长时间介入；
TEB 连续 NO_VALID_CMD；
用于 model-height formal 时 monitor 未启动、未 reset 或参数口径不一致；
path progress 10%~90% 核心段出现 rosbag buffer exceeded / odom 丢样。
```

若控制、tracking 和 RGB 完整，但 standalone monitor topic 缺失，该 run 可以保留为
轨迹/RGB 描述性证据；不得把离线补算结果伪装成实录 `/slosh/height`。如果论文的 model
height 表要求 clean online monitor N=3，则必须补跑。

外部 baseline 不要求 `delay_compensation_applied_frac`，因为它没有 SPMPC delay-phase 模块；其有效性由 baseline status、map-frame tracking diagnostics、cmd 完整性和现场安全记录判定。

---

## 12. TEB 完成后的 mpc_local_planner 实物接入

### 12.1 当前状态和方法定位

`mpc_local_planner` 是标准 `nav_core::BaseLocalPlanner` 插件，runner 直接执行
`MpcLocalPlannerROS::computeVelocityCommands()` 的输出，不需要像 LT-DWA 那样用自定义 path-tracking guard 长期覆盖命令。因此，如果实物 smoke 能通过，它是比 LT-DWA 更干净的第二外部 baseline 候选。

当前状态必须区分：

```text
isolated MPC overlay / plugin：已构建、可解析；
固定 S-curve 仿真：已验证；
实物 shadow：尚未正式验证；
实物 actuated N=1：尚未验证；
实物 formal N=3：未开始。
```

0706 输出目录中已有的 `sim_MPC_*` bag 虽然放在 `slosh_bags/real` 下，但包含
`/clock` 和 `/gazebo/*`，属于仿真证据，不能当成实物 smoke。

### 12.2 20260711 strict fresh-sim N=1 复测

按仿真统一指南，用当前代码、isolated overlay 和 tuned config 重新运行：

```text
matrix: mpc_local_planner
strict fresh-sim: true
N=1
goal: (5.0, 0.0, 0.0)
path: s_curve / start_heading=current
limits: v=0.8, omega=1.2, a=0.6, alpha=1.2
delay phase: off
planner config: mpc_local_planner_fixed_path_tuned_sim.yaml
```

结果：

```text
valid_strict_case=true；
GOAL_REACHED；
duration=6.537s；
tracking RMS≈0.100m；
tracking p95≈0.198m；
tracking max≈0.206m；
最终 path progress≈0.968，无明显回退；
cmd_v mean/p95/max≈0.773/0.800/0.800m/s；
|cmd_w| p95/max≈1.163/1.200rad/s；
/slosh/height p95/peak≈1.708/1.804mm。
```

结果路径：

```text
/data/a/scout_sim_replacement/results/strict_fresh_fair_n3_20260711_193812_codex_mpc_local_planner_n1
/data/a/scout_sim_replacement/bags/strict_fresh_fair_n3_20260711_193812_codex_mpc_local_planner_n1
```

判定：当前 MPC plugin、固定路径和闭环接口正常，但 tuned-sim 参数会同时触及线速度和角速度上限，不能未经降速和实物配置固化就直接上车。

### 12.3 实物前必须固化 real/no-obstacle 配置

不能把现有 simulation 配置直接改名后使用。开始实物 shadow 前新增版本化文件：

```text
src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_real_noobs.yaml
```

第一版从 tuned-sim 复制结构，但至少修改：

```text
MpcLocalPlannerROS/robot/unicycle/max_vel_x=0.30
MpcLocalPlannerROS/robot/unicycle/max_vel_x_backwards=0.0
MpcLocalPlannerROS/robot/unicycle/max_vel_theta=1.20
MpcLocalPlannerROS/robot/unicycle/acc_lim_x=0.60
MpcLocalPlannerROS/robot/unicycle/dec_lim_x=0.60
MpcLocalPlannerROS/robot/unicycle/acc_lim_theta=1.20
MpcLocalPlannerROS/collision_avoidance/include_costmap_obstacles=false
MpcLocalPlannerROS/collision_avoidance/enable_dynamic_obstacles=false
```

costmap 使用与 `TEB-noobs-fixed` 相同的：

```text
src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_real_no_obstacles.yaml
```

原因：当前 fixed-path SPMPC 和 TEB 正文主表均关闭 obstacle/corridor。若 MPC 单独开启 scan obstacle/inflation，就不能与该主表混合解释。

MPC 内部限制必须和 runner 参数相同。不能让内部 `max_vel_x=0.8`，再依靠 runner 长期截到 `0.30`。`/baseline/mpc_local_planner/command_intervention` 应作为隐藏限幅检查，建议 formal 候选的 linear/angular limited fraction `<1%`。

### 12.4 Overlay 与 preflight

实物机器先确认 isolated overlay：

```bash
ls /home/geist/scout_ws/install_isolated_mpc/setup.bash

source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
source /home/geist/scout_ws/install_isolated_mpc/setup.bash
cd /home/geist/scout_ws

rospack find mpc_local_planner
rospack plugins --attrib=plugin nav_core | grep mpc_local_planner
```

如果报：

```text
Could not find library corresponding to plugin mpc_local_planner/MpcLocalPlannerROS
```

说明 isolated overlay 没有正确 source；不能继续实车。还要检查：

```bash
rostopic echo -n 1 /odom
rostopic echo -n 1 /map
rostopic echo -n 1 /scan_front
rosrun tf tf_echo map base_link
rostopic info /cmd_vel
```

### 12.5 Shadow

以下命令只有在 real/no-obstacle YAML 已创建并通过 launch parse 后才能执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
source /home/geist/scout_ws/install_isolated_mpc/setup.bash
cd /home/geist/scout_ws

METHOD=mpc_local_planner \
STAGE=shadow \
RUN_LABEL=MPC_local_planner_real_noobs_v1_shadow01 \
PLANNER_CONFIG=/home/geist/scout_ws/src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_real_noobs.yaml \
COSTMAP_CONFIG=/home/geist/scout_ws/src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_real_no_obstacles.yaml \
MAX_V=0.30 MAX_W=1.20 MAX_ACC=0.60 MAX_ANGULAR_ACC=1.20 \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_RGB=false RECORD_TOPIC_INFO=true \
RECORD_SEC=30 MAX_RECORD_SEC=30 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

shadow 通过条件：

```text
/baseline/mpc_local_planner/status 进入 TRACKING；
/baseline/mpc_local_planner/global_plan 是同一 fixed S-curve；
/baseline/mpc_local_planner/raw_cmd_vel 连续且数值有限；
/baseline/mpc_local_planner/tracking_error 连续发布；
/spmpc_shadow_cmd_vel 有合理命令；
无 SET_PLAN_FAILED / NO_VALID_CMD 持续出现；
/cmd_vel 不由 shadow MPC 发布。
```

### 12.6 20s actuated 短程 smoke

shadow 通过后，第一轮只跑 `20s`，手在急停上：

```bash
METHOD=mpc_local_planner \
STAGE=actuated \
RUN_LABEL=MPC_local_planner_real_noobs_v1_short01 \
PLANNER_CONFIG=/home/geist/scout_ws/src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_fixed_path_real_noobs.yaml \
COSTMAP_CONFIG=/home/geist/scout_ws/src/scout_apps/control/baseline_local_planner_runner/config/local_costmap_real_no_obstacles.yaml \
MAX_V=0.30 MAX_W=1.20 MAX_ACC=0.60 MAX_ANGULAR_ACC=1.20 \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_RGB=true RECORD_TOPIC_INFO=true \
RECORDER_STARTUP_SEC=8 \
RECORD_SEC=20 MAX_RECORD_SEC=25 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

短程 gate：

```text
机器人持续沿 fixed path 正向推进；
tracking p95 < 0.30m；
无明显左右摆头、原地高速旋转或切弯失控；
20s 内 path progress 至少增加 0.15；
raw command 不长期贴 MAX_V/MAX_W；
runner limiter fraction 建议 <1%；
无持续 NO_VALID_CMD；
人工未接管。
```

### 12.7 单变量调参和 60s gate

第一轮只改速度轴。若 `MAX_V=0.30` 稳定但过慢：

```text
v1: MAX_V=0.30
v2: MAX_V=0.35
v3: MAX_V=0.40（仅在前两档稳定且现场安全时）
```

每次同时更新 real YAML 内部 `max_vel_x` 和 runner `MAX_V`。如果角速度频繁饱和或摆头，优先一次只改一个：

```text
quadratic_form control_weights angular 项：0.04 -> 0.08
或
max_global_plan_lookahead_dist：1.0 -> 1.5
```

如果 tracking 太松，一次只提高 position/state tracking 权重；不能同时提高 tracking 权重、降低 lookahead、提高速度和降低角速度上限。

短程通过后才跑 60s N=1。进入 formal 的最低门槛：

```text
GOAL_REACHED；
goal time <=57s；
tracking p95 <=0.30m；
无人工接管 / 定位跳变 / 持续 NO_VALID_CMD；
无长期线速度或角速度后级 clamp；
RGB、odom、TF、fixed path、raw/final cmd 和 tracking diagnostics 完整。
```

最多尝试 2~3 个单变量 candidate。仍不能同时满足到点、tracking、平顺性和 limiter gate 时停止 MPC 实物调参，不让第二外部 baseline 吞掉 governor 消融时间。

### 12.8 参数冻结和 formal

N=1 通过后，将最终 real YAML 另存为 frozen/final 文件，记录 git commit，之后不得继续调参。若 MPC 进入补充 formal，仍需与同日 governor-off `B_ours` bridge 交错运行，并明确：

```text
mpc_local_planner 不使用 slosh feedback；
液面仅作为外部评价；
必须同时报告 goal time、actual cmd_v、tracking 和 RGB；
不能因为其速度快或慢只比较液面 peak。
```

---

## 13. TEB 完成后的 LT-DWA rescue

### 13.1 顺序和实验定位

LT-DWA 不打断当前 TEB 主线。执行顺序固定为：

```text
TEB shadow / N=1 gate
  -> TEB 参数冻结
  -> TEB 与同日 B_ours governor-off 交错 formal N=3
  -> TEB 数据确认完整
  -> 优先决定是否补 mpc_local_planner
  -> 最后再单独处理 LT-DWA
```

LT-DWA 是可选补充 baseline，不得反过来阻塞 TEB、governor 消融或主方法实物实验。0706 原始参数不能直接重跑 formal：当时 official core 持续返回 OK，但 wrapper 的 `path_tracking_guard` 全程接管后仍出现 map-frame tracking p95 约 `1.23m`、进度最大约 `39%` 而最终退回约 `13%`、角速度达到 `1.2rad/s` 上限。

当前仿真成功的准确口径是：

```text
LT-DWA official core + wrapper path-tracking guard
```

不能把 guarded/final command 当成未经修改的官方 LT-DWA 原始输出。后续表格必须同时保留 raw command、final command 和 guard applied fraction。

### 13.2 实物 rescue 前先补齐启动和记录接口

当前 one-click 脚本只透传公共速度/加速度限制。开始 LT-DWA rescue 前，先让
`run_external_baseline_real_fixed_path_trial.sh` 显式支持并写入 metadata：

```text
MIN_V
TIME_STEP
PATH_RESAMPLE_SPACING
ENABLE_PATH_TRACKING_GUARD
PATH_TRACKING_LOOKAHEAD_M
PATH_TRACKING_MIN_V
```

这些变量应继续透传到 `lt_dwa_official_wrapper` launch，不能依赖 launch 内部隐藏默认值。同时补齐固定录包 topic：

```text
/baseline/official_lt_dwa/odom_map
/baseline/official_lt_dwa/status
/baseline/official_lt_dwa/diagnostics
/baseline/official_lt_dwa/worker_result
/baseline/official_lt_dwa/raw_cmd_vel
/baseline/official_lt_dwa/shadow_cmd_vel
/baseline/official_lt_dwa/global_plan
/baseline/official_lt_dwa/local_plan
```

tracking 必须使用 `/baseline/official_lt_dwa/odom_map` 与
`/scout/global_path_fixed`，不能直接用不同 frame 的 `/odom`。指标脚本还应同时识别 real topic 前缀 `/baseline/official_lt_dwa/*` 和 sim topic 前缀 `/baseline/lt_dwa/*`。

### 13.3 第一阶段：只做低速参数 rescue

第一候选 R0 的目的只是验证“能否稳定沿路径推进”，不追求速度：

```text
MAX_V=0.20
MIN_V=0.00
MAX_W=1.20
MAX_ACC=0.30
MAX_ANGULAR_ACC=1.20
PLANNER_RATE_HZ=5.0
COMMAND_PUBLISH_RATE_HZ=30.0
TIME_STEP=0.20
PATH_RESAMPLE_SPACING=0.05
ENABLE_PATH_TRACKING_GUARD=true
PATH_TRACKING_LOOKAHEAD_M=1.20
PATH_TRACKING_MIN_V=0.08
```

先跑 shadow：

```bash
METHOD=lt_dwa_official \
STAGE=shadow \
RUN_LABEL=LTDWA_fixed_rescue_R0_shadow01 \
MAX_V=0.20 MIN_V=0.00 \
MAX_W=1.20 MAX_ACC=0.30 MAX_ANGULAR_ACC=1.20 \
PLANNER_RATE_HZ=5.0 COMMAND_PUBLISH_RATE_HZ=30.0 TIME_STEP=0.20 \
PATH_RESAMPLE_SPACING=0.05 \
ENABLE_PATH_TRACKING_GUARD=true \
PATH_TRACKING_LOOKAHEAD_M=1.20 \
PATH_TRACKING_MIN_V=0.08 \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_RGB=false RECORD_SEC=30 MAX_RECORD_SEC=30 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

shadow 通过后，只跑 `15~25s` actuated：

```bash
METHOD=lt_dwa_official \
STAGE=actuated \
RUN_LABEL=LTDWA_fixed_rescue_R0_short01 \
MAX_V=0.20 MIN_V=0.00 \
MAX_W=1.20 MAX_ACC=0.30 MAX_ANGULAR_ACC=1.20 \
PLANNER_RATE_HZ=5.0 COMMAND_PUBLISH_RATE_HZ=30.0 TIME_STEP=0.20 \
PATH_RESAMPLE_SPACING=0.05 \
ENABLE_PATH_TRACKING_GUARD=true \
PATH_TRACKING_LOOKAHEAD_M=1.20 \
PATH_TRACKING_MIN_V=0.08 \
RECORD_ALL_EXISTING_TOPICS=false \
RECORD_RGB=true RECORD_SEC=20 MAX_RECORD_SEC=25 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

短程通过条件：

```text
odom_map -> fixed path p95 < 0.50m；
path progress 持续增加，20s 内至少增加 0.10；
不得出现大于 0.03 的明显进度回退；
角速度不能长期贴 1.2rad/s；
无连续 STALE_INPUT / local_map_service_wait_failed；
人工未接管，肉眼未明显离轨。
```

R0 太慢但稳定时，一次只把 `MAX_V` 调到 `0.22`。R0 仍外切时，一次只把
`MAX_V` 降到 `0.15` 或把 lookahead 增到 `1.50m`。不允许同时修改速度、lookahead、最低速度和角速度上限。

### 13.4 第二阶段：formal 前必须修改 wrapper guard

参数能完成短程 smoke，不代表当前 guard 已具备实物鲁棒性。进入 60s smoke 或 formal 前，wrapper 至少要补：

```text
1. 保存单调路径进度或局部最近点搜索窗口，禁止最近点跳回路径前段；
2. 大横向/航向误差时允许 v=0，先对准路径再恢复前进；
3. 超出最大捕获距离时发布 zero 并进入 LOST_TRACK，而不是继续强制前进；
4. 使用上一条实际发布命令和真实控制周期做加速度限制，不围绕滞后 odom 速度反复修正；
5. map-frame pose、odom twist 和时间戳必须对应同一采样时刻；latest-TF fallback 必须显式诊断；
6. diagnostics 发布 tracking distance、heading error、path progress、nearest index、recovery state、pose/twist age；
7. GOAL_REACHED 同时检查 XY 和 yaw tolerance，不能只检查终点距离。
```

推荐 recovery gate 起点：

```text
tracking distance > 0.50m 或 |heading error| > 1.0rad：v=0，低速转向重捕获；
tracking distance > 0.80m：发布 zero，状态 LOST_TRACK，等待人工检查；
恢复到 distance < 0.25m 且 |heading error| < 0.45rad 后再允许前进。
```

阈值需先在 shadow 和短程实物中验证，不能未经验证直接冻结为论文参数。

### 13.5 60s smoke、formal 和停止线

只有“R0/R1/R2 短程通过 + guard 代码 gate 通过”后，才允许 LT-DWA 60s smoke。最低门槛：

```text
GOAL_REACHED；
最终 path progress > 0.90；
tracking p95 < 0.50m，max 最好 < 0.80m；
无明显进度回退；
STALE_INPUT / zero-command 只允许短暂启动瞬态；
角速度饱和不是长期状态；
raw/final command、guard fraction、odom_map 和 RGB 数据完整。
```

60s smoke 通过后才能冻结参数并跑 formal N=3。若 guard 在绝大多数周期都接管，论文中必须使用 `LT-DWA official core + path-tracking guard` 标签，不能写成纯官方 LT-DWA。

Timebox：最多尝试 2~3 个单变量参数候选和 1 个明确的 guard 修正版。仍无法稳定到点时，保留失败 bag 和原因，停止 LT-DWA；TEB 继续作为正文外部 baseline。

---

## 14. 当前一句话原则

```text
先用 T0/T1 把 TEB 的 tracking 质量收紧并量化，确认 standalone monitor 已启动、reset 且录包无核心段丢样后再冻结；随后与同日 governor-off B_ours 交错跑 N=3。TEB 数据完成后优先尝试标准插件 mpc_local_planner，最后才处理 LT-DWA。
```
