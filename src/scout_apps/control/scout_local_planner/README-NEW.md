# Online GeoRef + MPC 跟踪方案

更新时间：2026-05-06

本文档记录当前 anti-slosh 主线。旧 `README.md` 描述的是增广 slosh MPC / risk scheduler / 输出裁剪等历史方案，不代表当前最有效的实现。

## 1. 当前结论

当前有效路线是：

```text
MBF 全局路径
  -> anti_slosh_path_post_processor 在线几何后处理
  -> /scout/global_path_anti_slosh
  -> 普通 MPC 跟踪
  -> /cmd_vel
```

核心判断：

```text
防晃收益来自 MPC 前端的低激励几何参考生成；
不是来自 MPC 内部 slosh soft cost；
不是来自 cmd_vel 后处理；
不是来自简单降速。
```

当前成功配置中，MPC 内部防晃项全部关闭：

```text
Q_slosh=0
Q_slosh_eta_dot=0
enable_slosh_box_constraint=false
risk_scheduler_enable=false
energy_profile_enable=false
input_shaping_enable=false
slosh_speed_governor_enable=false
```

## 2. 与旧方案的区别

| 项目 | 旧 README 风险自适应 MPC | 当前 Online GeoRef |
|---|---|---|
| 防晃位置 | MPC cost / constraint / scheduler / output guard | MPC 前端路径几何参考 |
| MPC 角色 | tracking + anti-slosh 同时承担 | 只做 constrained tracking |
| 主要控制对象 | `Q_slosh`、`eta_dot`、`rho_k`、输出 cap | `kappa`、`dkappa`、路径激励、执行一致性 |
| 已验证正信号 | 不稳定，跨路径失败 | open 场景同目标三包正结果 |
| 实物验证口径 | 曾建议 `Q_slosh=5` | 当前必须 `Q_slosh=0`，只验证 GeoRef |
| 论文定位 | 风险自适应 MPC 主动压晃 | 低激励几何参考生成 + MPC 跟踪 |

论文中不能写成：

```text
MPC 代价函数主动抑制液体晃动。
```

应写成：

```text
将防晃逻辑前移到几何参考生成层，并由 MPC 在车辆约束下跟踪该低激励参考。
```

## 3. 系统结构

```text
/scout/goal
  ↓
scout_global_planner mbf_global.launch
  ↓
/scout/global_path
  ↓
anti_slosh_path_post_processor.py
  ↓
/scout/global_path_anti_slosh
  ↓
scout_local_planner slosh_experiment(.launch/.sim.launch)
  ↓
/cmd_vel
```

`anti_slosh_path_post_processor.py` 不发布速度命令，也不是新的控制器。它只修改 `nav_msgs/Path` 的几何形状。

MPC 仍是原 tracking MPC：

```text
输入路径: /scout/global_path 或 /scout/global_path_anti_slosh
输出: /cmd_vel
控制变量: a, omega
约束: v, a, omega, da, domega
求解器: OSQP
```

## 4. Post-Processor 当前实现

文件：

```text
scripts/anti_slosh_path_post_processor.py
launch/anti_slosh_path_post_processor.launch
```

订阅：

```text
/scout/global_path
/scout/mbf_costmap_nav/global_costmap/costmap   # enable_collision_check=true 时
```

发布：

```text
/scout/global_path_anti_slosh
/anti_slosh_path/candidate_report
/anti_slosh_path/debug/original
/anti_slosh_path/debug/mild
/anti_slosh_path/debug/medium
/anti_slosh_path/debug/strong
```

候选生成：

```text
original
mild smoothing
medium smoothing
strong smoothing
```

gate：

```text
max_drift
length_ratio upper/lower
endpoint_error
min_segment_length
path direction
max_candidate_level
collision check，启用时使用 global costmap inflation
predicted ay ratio 诊断/门控
```

score 目标不是最小曲率，而是中等程度降低曲率和曲率变化，避免过度拉直路径后引入更大的速度/jerk 激励。

当前 open 验证中主要有效状态：

```text
selected candidate = medium 或 mild
selected original 只能说明 fallback，不算 GeoRef 有效样本
```

重要语义修正：

```text
selected=original 时，/scout/global_path_anti_slosh 直接转发原始 MBF path；
不再发布重采样后的 original。
```

## 5. 仿真主结果

主验证场景：

```text
SIM_ENV=open
目标 open_user_goal:
  x=-3.1570560932159424
  y=-2.897411346435547
  qz=-0.978164583074326
  qw=0.2078317791364693
```

RAW_TUNED 三包：

```text
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_RAW_TUNED_run01_190008.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_RAW_TUNED_run02_190809.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_RAW_TUNED_run03_191300.bag
```

GEOREF_TUNED 三包：

```text
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_GEOREF_TUNED_run01_190153.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_GEOREF_TUNED_run02_190940.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_GEOREF_TUNED_run03_191424.bag
```

GEOREF_TUNED 相对 RAW_TUNED 三包均值：

```text
active_s       +5.5%
h_rms          -18.7%
h_p95          -18.6%
h_max          -6.6%
eta_dot_rms    -11.5%
energy_rms     -18.2%
ay_p95         -38.5%
ax_p95         -18.5%
alpha_p95      -13.7%
ref_ay_p95     -42.0%
ref_ax_p95     -91.4%
ref_jerk_p95   -80.5%
```

阶段性结论：

```text
在 open_user_goal 上，Online GeoRef 在 tracking time +15% 门槛内降低了 /slosh/height 的 rms/p95/max、
eta_dot、modal energy 以及主要执行激励。
```

边界：

```text
这不是任意目标泛化证明。
这不是实物真液面证明。
这不保证每一包每一个瞬时峰值都单调下降。
```

## 6. Baseline 结果

### 6.1 RAW_SLOW_MATCHED

目的：排除“只是慢了”。

配置：

```text
原始 MBF path
不启用 GeoRef
vehicle_v_max=1.90
其余 tracking 参数与 RAW/GEOREF 保持一致
```

SLOW 三包：

```text
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_slow_RAW_TUNED_run01_192143.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_slow_RAW_TUNED_run02_192317.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_slow_RAW_TUNED_run03_192452.bag
```

SLOW vs RAW：

```text
active_s       +3.3%
h_rms          -9.1%
h_p95          -1.7%
h_max          +22.0%
eta_dot_rms    +40.6%
energy_rms     -4.8%
ay_p95         -19.7%
alpha_p95      +15.2%
```

GEOREF vs SLOW：

```text
active_s       +2.1%
h_rms          -10.7%
h_p95          -17.3%
h_max          -23.4%
eta_dot_rms    -37.0%
energy_rms     -14.1%
ay_p95         -23.3%
alpha_p95      -25.1%
```

结论：

```text
简单降速不能解释 GeoRef 收益；慢速 baseline 甚至让 h_max 和 eta_dot 变差。
```

### 6.2 修正版 GEOREF_ORIGINAL

目的：排除“只是换 topic / post-processor chain”。

修正版语义：

```text
max_candidate_level=original
selected=original 时直接转发原始 MBF path，不重采样
```

ORIGINAL_FIXED 三包：

```text
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_original_fixed_GEOREF_ORIGINAL_run01_195728.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_original_fixed_GEOREF_ORIGINAL_run02_195902.bag
/data/a/slosh_bags/sim/20260506/20260506_open_user_goal_original_fixed_GEOREF_ORIGINAL_run03_200200.bag
```

ORIGINAL_FIXED vs RAW：

```text
active_s       +7.2%
h_rms          -10.9%
h_p95          -0.6%
h_max          +15.5%
eta_dot_rms    +39.1%
energy_rms     -6.3%
ay_p95         -18.3%
alpha_p95      +12.3%
```

GEOREF vs ORIGINAL_FIXED：

```text
active_s       -1.5%
h_rms          -8.8%
h_p95          -18.1%
h_max          -19.1%
eta_dot_rms    -36.3%
energy_rms     -12.7%
ay_p95         -24.7%
alpha_p95      -23.1%
```

结论：

```text
GeoRef 收益来自 geometry smoothing candidate selection，
不是来自 topic chain、latching 或 original fallback。
```

## 7. 当前推荐启动方式

### 7.1 仿真环境

```bash
source /home/a/scout_ws/devel/setup.bash
SIM_ENV=open USE_RVIZ=true \
SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
rosrun scout_local_planner launch_sim_nav_stack.sh
```

### 7.2 RAW_TUNED 录包

```bash
PATH_MODE=global_goal CONDITION=RAW_TUNED PATH_ID=open_user_goal RUN_ID=01 \
GOAL_X=-3.1570560932159424 \
GOAL_Y=-2.897411346435547 \
GOAL_QZ=-0.978164583074326 \
GOAL_QW=0.2078317791364693 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

### 7.3 GEOREF_TUNED 录包

```bash
PATH_MODE=global_goal CONDITION=GEOREF_TUNED PATH_ID=open_user_goal RUN_ID=01 \
GOAL_X=-3.1570560932159424 \
GOAL_Y=-2.897411346435547 \
GOAL_QZ=-0.978164583074326 \
GOAL_QW=0.2078317791364693 \
POST_PROCESSOR_COLLISION_CHECK=false \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

说明：

```text
open 场地可关闭 collision_check。
maze/实物必须打开 collision_check，并确认 global costmap 正常。
```

### 7.4 手动启动 Online GeoRef

启动 post-processor：

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  ds:=0.03 \
  max_candidate_level:=medium \
  publish_debug:=true \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  ay_ratio_limit:=1.0
```

启动 MPC tracker：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  global_path_topic:=/scout/global_path_anti_slosh \
  Q_slosh:=0 \
  Q_slosh_eta_dot:=0 \
  enable_slosh_box_constraint:=false \
  risk_scheduler_enable:=false \
  energy_profile_enable:=false \
  input_shaping_enable:=false \
  slosh_speed_governor_enable:=false
```

如果是仿真并需要调 sim 专用参数，使用 `slosh_experiment_sim.launch`。

## 8. 实物验证口径

实物验证详见：

```text
docs/重要文档/20260506_Online_GeoRef实物液体晃动抑制有效性验证方案.md
```

第一轮实物验证必须保持：

```text
RAW_REAL:
  /scout/global_path -> MPC

GEOREF_REAL:
  /scout/global_path -> anti_slosh_path_post_processor -> /scout/global_path_anti_slosh -> MPC

两者都:
  Q_slosh=0
  Q_slosh_eta_dot=0
  risk_scheduler=false
  energy_profile=false
  input_shaping=false
  slosh_speed_governor=false
```

实物第一轮建议在开阔场地做，不在 maze/窄走廊做。若 `/slosh/height` 降低但视觉真液面不降低，不能声明真实液体晃动被抑制。

## 9. 不再作为主线的方案

已否定或降级为 failure analysis：

```text
Q_slosh / Q_slosh_eta_dot / terminal slosh cost
Q_modal_energy / Q_ay_pred
risk scheduler
OUTPUT_GUARD
PMG lateral / longitudinal / combined
PROFILE_ENERGY speed-only profile
PROFILE_REF_V2 fixed-geometry speed/reference correction
```

这些可以作为论文消融和负结果，但不应继续作为当前 proposed controller。

## 10. 当前边界与下一步

已成立：

```text
open_user_goal 三包均值显示 Online GeoRef 降低 /slosh/height、eta_dot、energy、ay/ax/alpha，
tracking time 增加在 +15% 内。
RAW_SLOW_MATCHED 和 GEOREF_ORIGINAL 已排除简单降速和 topic chain 解释。
```

未成立：

```text
任意目标泛化
maze / 窄通道安全验证
实物真实液面验证
MPC 内部 slosh cost 有效性
```

下一步优先级：

```text
1. 按实物验证方案在开阔场地做 RAW_REAL vs GEOREF_REAL。
2. 正式实物实验前，把 GeoRef 诊断话题补进 record_slosh_experiment.sh。
3. 如果实物 open 通过，再考虑第二目标和视觉液面真值验证。
4. maze 只在 raw tracking/collision 本身安全后再进入，不用于当前主结论。
```
