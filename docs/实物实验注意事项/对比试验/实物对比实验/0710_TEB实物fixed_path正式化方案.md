# 0710 TEB 实物 fixed-path 正式化方案

> 目的：在不等待 Slosh-risk Governor 实物消融的前提下，先把一个普通外部 local planner baseline 做到可复现、可量化、可进入 formal N=3。本文以 `TEB-noobs-fixed` 为当前首选，并把 0706 的 loose_fast 调参方向固化为受控的候选起点。
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
MAX_V / MAX_W:
  同时覆盖 TEB 内部 max_vel_x/max_vel_theta 和 runner 最终 clamp；

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
| `max_vel_x` | 由 `MAX_V` 覆盖，首轮 `0.30` |
| `max_vel_theta` | 由 `MAX_W` 覆盖，首轮 `1.20` |
| `acc_lim_x` | `0.60` |
| `acc_lim_theta` | `1.20` |

注意：`weight_acc_lim_theta` 主要提高对角加速度约束违反的惩罚，不等于独立的角速度平滑/jerk 代价。减少摆头首先检查 via-point 密度、路径剪枝、lookahead 和原始命令，不要只靠降低角速度硬上限把问题遮住。

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

### 7.2 Shadow

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
无 SET_PLAN_FAILED / NO_VALID_CMD 长时间持续；
/cmd_vel 没有被 shadow planner 发布。
```

### 7.3 Actuated N=1 smoke

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
RECORD_TOPIC_INFO=true \
RECORDER_STARTUP_SEC=8 \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

手在急停上。首轮不同时修改 speed、via-point 和 angular limit。

---

## 8. N=1 量化 gate

现有 `B_ours` 参考完成时间约 `38.1s`。TEB 进入 formal 的最低门槛：

```text
GOAL_REACHED；
无人工接管 / E-stop / 定位跳变；
goal time <= 57s（约 B_ours 的 1.5 倍）；
tracking p95 <= 0.30m；
无持续左右摆头、急停或长时间 NO_VALID_CMD；
linear/angular limited fraction 不长期非零，建议 < 1%；
RGB 覆盖运动开始 -> first GOAL_REACHED，并保留 post-goal 残余段；
odom / tf / fixed path / cmd / baseline diagnostics 完整。
```

提取控制、tracking 和 limiter 指标：

```bash
python3 src/scout_apps/control/spmpc_experiments/scripts/extract_fixed_path_paper_metrics.py \
  /home/geist/slosh_bags/real/20260710_fixed_path_compare/teb \
  --phase windows \
  --csv /home/geist/slosh_bags/real/20260710_fixed_path_compare/teb/teb_noobs_v1_metrics.csv
```

RGB 仍使用统一离线 max-LCR 流程，不使用在线 `/liquid/*` 作为控制或主真值。

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

一次只向相反方向恢复一个参数：

```text
weight_viapoint: 2.0 -> 3.0
或
global_plan_viapoint_sep: 0.70 -> 0.60
```

tracking p95 超过 `0.30m` 的 run 不能进入正式主表。

### 9.4 Timebox

最多尝试 2-3 个有明确单变量差异的 candidate。如果仍不能同时满足到点、tracking、时间和摆头门槛，停止继续围绕 TEB 大规模调参，转 `mpc_local_planner`。外部 baseline 是补充证据，不能吞掉 governor 和主方法实验时间。

---

## 10. 参数冻结与 formal N=3

N=1 通过后：

```text
1. 将最终 YAML 另存为带 final/frozen 名称的版本化文件；
2. 记录 git commit；
3. 固定 path、goal、limits、goal tolerance、controller frequency；
4. formal 过程中不得继续调参数；
5. 调参 bag 全部保留，但不进入主统计。
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
TEB 连续 NO_VALID_CMD。
```

外部 baseline 不要求 `delay_compensation_applied_frac`，因为它没有 SPMPC delay-phase 模块；其有效性由 baseline status、map-frame tracking diagnostics、cmd 完整性和现场安全记录判定。

---

## 12. TEB 完成后的 LT-DWA rescue

### 12.1 顺序和实验定位

LT-DWA 不打断当前 TEB 主线。执行顺序固定为：

```text
TEB shadow / N=1 gate
  -> TEB 参数冻结
  -> TEB 与同日 B_ours governor-off 交错 formal N=3
  -> TEB 数据确认完整
  -> 再单独处理 LT-DWA
```

LT-DWA 是可选补充 baseline，不得反过来阻塞 TEB、governor 消融或主方法实物实验。0706 原始参数不能直接重跑 formal：当时 official core 持续返回 OK，但 wrapper 的 `path_tracking_guard` 全程接管后仍出现 map-frame tracking p95 约 `1.23m`、进度最大约 `39%` 而最终退回约 `13%`、角速度达到 `1.2rad/s` 上限。

当前仿真成功的准确口径是：

```text
LT-DWA official core + wrapper path-tracking guard
```

不能把 guarded/final command 当成未经修改的官方 LT-DWA 原始输出。后续表格必须同时保留 raw command、final command 和 guard applied fraction。

### 12.2 实物 rescue 前先补齐启动和记录接口

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

### 12.3 第一阶段：只做低速参数 rescue

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

### 12.4 第二阶段：formal 前必须修改 wrapper guard

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

### 12.5 60s smoke、formal 和停止线

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

## 13. 当前一句话原则

```text
先用版本化 real/no-obstacle 配置把 TEB 做到稳定、量化、冻结，再与同日 governor-off B_ours 交错跑 N=3；TEB 数据完成后才处理 LT-DWA，并坚持“短程参数 rescue -> guard 实物鲁棒性修改 -> 60s smoke -> formal”的顺序。
```
