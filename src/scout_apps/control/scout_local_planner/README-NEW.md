# Online GeoRef / OSCRS Reference Generation + MPC Tracking

更新时间：2026-05-08

本文档记录当前推荐的 anti-slosh 工程结构。旧 `README.md` 仍可作为历史 MPC/slosh-cost 方案记录，但不代表当前主线。

## 1. 当前结论

当前系统采用 reference-first 结构：

```text
MBF global path
  -> anti_slosh_path_post_processor
  -> /scout/global_path_anti_slosh
  -> normal MPC tracking
  -> /cmd_vel
```

核心原则：

```text
防晃逻辑放在 MPC 前端的参考路径生成/选择层；
MPC 只作为 constrained tracking layer；
不再依赖 Q_slosh、eta_dot soft cost、输出裁剪或 cmd_vel 后处理作为主防晃机制。
```

主线实验中 MPC 内部防晃项必须关闭：

```text
Q_slosh=0
Q_slosh_eta_dot=0
enable_slosh_box_constraint=false
risk_scheduler_enable=false
energy_profile_enable=false
input_shaping_enable=false
slosh_speed_governor_enable=false
```

## 2. 三条实物对比线

正式实物有效性验证只比较三条线。

### RAW_REAL

```text
/scout/global_path
  -> normal MPC tracking
```

含义：MBF 原始全局路径 baseline，不启动 post-processor。

### GEOREF_TUNED_STRONG_REAL

```text
/scout/global_path
  -> geometry-only GeoRef candidate selection
  -> /scout/global_path_anti_slosh
  -> normal MPC tracking
```

含义：只用几何指标选择候选路径。它是 Online GeoRef 主线，也是 OSCRS 的公平 baseline。

实物主表使用：

```text
max_candidate_level=strong
oscrs_shadow_enable=false
oscrs_active_enable=false
```

注意：历史 open_user_goal 仿真正结果主要来自 medium/调参版本。和 OSCRS_ACTIVE 做公平对比时，GeoRef baseline 必须使用同一 candidate set，即 `strong`。

### GEOREF_OSCRS_ACTIVE_REAL

```text
/scout/global_path
  -> same GeoRef candidate set
  -> OSCRS hard gate + normalized slosh/geometry score
  -> /scout/global_path_anti_slosh
  -> normal MPC tracking
```

含义：RA-L/Ferrari 整合方案的在线落地线。OSCRS 不使用实时液面高度闭环；它在路径执行前对候选参考做 slosh rollout，按 `eta_lim / residual / score` 选择路径。

实物主表使用：

```text
max_candidate_level=strong
oscrs_shadow_enable=true
oscrs_active_enable=true
```

有效 OSCRS 样本必须通过 `candidate_report` 解释：

```text
summary:selected=strong,geo=medium,oscrs=strong,active=1,fallback=0,fb=0,orig_safe=1,takeover=1
```

字段含义：

```text
active=1     OSCRS active selector 已开启
fb=0         OSCRS 选中非 original 且通过 hard gate
takeover=1   OSCRS 选择不同于 geometry_best 的候选，实际改变参考路径
takeover=0   OSCRS 运行了，但与 geometry-only 选择相同，不能计为 OSCRS 额外物理贡献
```

## 3. 与旧 MPC 方案的区别

| 项目 | 旧方案 | 当前方案 |
|---|---|---|
| 防晃位置 | MPC cost / risk scheduler / output guard / speed cap | MPC 前端 reference generation / selection |
| MPC 角色 | 同时承担 tracking 和 anti-slosh 决策 | 只做 constrained tracking |
| 主变量 | `Q_slosh`、`Q_eta_dot`、`rho_k`、cmd_vel cap | candidate path、`kappa/dkappa`、slosh rollout gate、OSCRS score |
| 当前主表 | 不再使用 | RAW / GeoRef / OSCRS 三线 |
| 论文表述 | 不应写“MPC 代价函数主动消晃” | 写“低激励参考选择 + MPC 跟踪” |

不进入当前主表的历史方案：

```text
Q_slosh / Q_slosh_eta_dot / terminal slosh cost
Q_modal_energy / Q_ay_pred
risk scheduler
OUTPUT_GUARD
PMG lateral / longitudinal / combined
PROFILE_ENERGY speed-only profile
PROFILE_REF_V2 fixed-geometry speed/reference correction
GEOREF_CONSTRAINED reference-budget MPC
GEOREF_SLOSH_SCORE_TUNED
```

这些可以作为 failure analysis / ablation，不作为当前推荐控制器。

## 4. Post-Processor

主要文件：

```text
scripts/anti_slosh_path_post_processor.py
launch/anti_slosh_path_post_processor.launch
config/oscrs_container.yaml
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
/anti_slosh_path/metrics
/anti_slosh_path/safety_alarm
/anti_slosh_path/debug/original
/anti_slosh_path/debug/mild
/anti_slosh_path/debug/mid
/anti_slosh_path/debug/medium
/anti_slosh_path/debug/strong
```

候选集合：

```text
original
mild
mid
medium
strong
```

通用 hard gates：

```text
max_drift
length_ratio upper/lower
endpoint_error
min_segment_length
path direction
max_candidate_level
collision check
predicted ay ratio
```

## 5. GeoRef Selection

`GEOREF_TUNED_STRONG_REAL` 使用 geometry score：

```text
score = target_kappa_penalty
      + dkappa_ratio
      + length/drift/shortening/over-smooth penalty
```

推荐实物启动参数：

```text
ds=0.03
max_candidate_level=strong
enable_collision_check=true
ay_ratio_limit=1.0
prediction_v_max=2.0
prediction_ay_max_budget=2.0
prediction_a_max=1.0
```

如果路径改得太弱且 `/slosh/height` 不降，优先提高候选强度；如果路径贴墙、绕路过多或 tracking 变差，降低 gain/max_drift 或候选等级。不要通过重新打开 MPC 防晃项补救。

## 6. OSCRS Selection

`GEOREF_OSCRS_ACTIVE_REAL` 使用同一候选集合，但选择规则变为：

```text
1. 对 accepted candidate 做 signed linear modal slosh rollout；
2. 计算 h_p95、modal_energy_rms、eta_dot_rms、terminal_E、residual height；
3. 用 eta_lim 与 residual_ratio 做 hard gate；
4. 在 feasible non-original candidates 中用 batch-normalized score 选最小者；
5. 若无 feasible non-original candidate，则 fallback 到 geometry_best，并在 candidate_report 中记录 fb。
```

核心模型口径：

```text
eta_x_ddot + 2*zeta*omega_n*eta_x_dot + omega_n^2*eta_x = -ax_eff
eta_y_ddot + 2*zeta*omega_n*eta_y_dot + omega_n^2*eta_y = -ay_eff

ax_eff = ax - alpha * offset_y - omega^2 * offset_x
ay_eff = ay + alpha * offset_x - omega^2 * offset_y

height = linear modal height + optional parabola term
```

默认参数入口：

```text
config/oscrs_container.yaml
```

关键字段：

```text
slosh.container_radius
slosh.liquid_height
slosh.damping_ratio
slosh.offset_x
slosh.offset_y
oscrs.eta_lim_mm
oscrs.residual_ratio
oscrs.settle_duration
oscrs.score/w_h_p95
oscrs.score/w_energy_rms
oscrs.score/w_eta_dot_rms
oscrs.score/w_terminal_E
oscrs.score/w_geom
```

`fb` 语义：

```text
fb=0  OSCRS 选中非 original 且通过 hard gate
fb=1  original slosh-safe，但无非 original feasible candidate
fb=2  有 geometry candidate，但 slosh hard gate 失败
fb=3  无可用 geometry candidate
```

`/anti_slosh_path/safety_alarm` 只在所有候选 hard gate 全失败时发布，用于实物安全复盘。

## 7. Normal MPC Tracking

MPC 启动文件：

```text
launch/slosh_experiment.launch
```

RAW 输入：

```text
global_path_topic:=/scout/global_path
```

GeoRef / OSCRS 输入：

```text
global_path_topic:=/scout/global_path_anti_slosh
```

主表固定关闭：

```bash
Q_slosh:=0 \
Q_slosh_eta_dot:=0 \
enable_slosh_box_constraint:=false \
risk_scheduler_enable:=false \
energy_profile_enable:=false \
input_shaping_enable:=false \
slosh_speed_governor_enable:=false
```

这保证三条线只差参考路径来源，不差 MPC、不差速度治理、不差旧防晃模块。

## 8. 实物运行命令

完整 SOP 见：

```text
docs/重要文档/20260508_Online_GeoRef_OSCRS实物录包SOP.md
```

### RAW_REAL

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

### GEOREF_TUNED_STRONG_REAL

Post-processor：

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  oscrs_config:=/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  ds:=0.03 \
  max_candidate_level:=strong \
  publish_debug:=true \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  ay_ratio_limit:=1.0 \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  prediction_a_max:=1.0 \
  mild_iters:=8 \
  mild_gain:=0.20 \
  mild_max_drift:=0.04 \
  oscrs_shadow_enable:=false \
  oscrs_active_enable:=false
```

MPC：

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

### GEOREF_OSCRS_ACTIVE_REAL

Post-processor：

```bash
roslaunch scout_local_planner anti_slosh_path_post_processor.launch \
  input_topic:=/scout/global_path \
  output_topic:=/scout/global_path_anti_slosh \
  oscrs_config:=/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
  ds:=0.03 \
  max_candidate_level:=strong \
  publish_debug:=true \
  enable_collision_check:=true \
  costmap_topic:=/scout/mbf_costmap_nav/global_costmap/costmap \
  ay_ratio_limit:=1.0 \
  prediction_v_max:=2.0 \
  prediction_ay_max_budget:=2.0 \
  prediction_a_max:=1.0 \
  mild_iters:=8 \
  mild_gain:=0.20 \
  mild_max_drift:=0.04 \
  oscrs_shadow_enable:=true \
  oscrs_active_enable:=true
```

MPC 同 `GEOREF_TUNED_STRONG_REAL`。

## 9. 仿真通路验证

启动 open 仿真：

```bash
source /home/a/scout_ws/devel/setup.bash
SIM_ENV=open USE_RVIZ=true \
SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
rosrun scout_local_planner launch_sim_nav_stack.sh
```

OSCRS active 通路验证命令示例：

```bash
source /home/a/scout_ws/devel/setup.bash

PATH_MODE=global_goal \
PATH_ID=open_custom_goal \
CONDITION=GEOREF_OSCRS_ACTIVE \
RUN_ID=yamlcheck01 \
START_DELAY=10 \
RECORD_DURATION=0 \
TEMPLATE_GOAL_X=-3.014343023300171 \
TEMPLATE_GOAL_Y=2.987114429473877 \
TEMPLATE_GOAL_QZ=0.9999403278718936 \
TEMPLATE_GOAL_QW=0.010924316704027428 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

检查 takeover：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/check_oscrs_takeover.py \
  <bag> --require-takeover
```

最近通路验证 bag：

```text
/data/a/slosh_bags/sim/20260508/20260508_open_custom_goal_GEOREF_OSCRS_ACTIVE_runyamlcheck01_173329.bag
```

结果：

```text
reports=29
active_reports=29
takeover_reports=5
fallback_reports=11
fb_counts={'2': 9, '0': 18, '3': 2}
selected_counts={'strong': 17, 'medium': 5, 'mid': 5, 'original': 2}
```

结论：代码通路顺畅，但这不是有效性证明。

## 10. 录包

实物录包脚本：

```text
scripts/record_slosh_experiment.sh
```

已覆盖关键话题：

```text
/scout/goal
/scout/global_path
/scout/global_path_anti_slosh
/anti_slosh_path/candidate_report
/anti_slosh_path/metrics
/anti_slosh_path/safety_alarm
/anti_slosh_path/debug/*
/slosh/state
/slosh/height
/slosh/height_pred_max
/slosh/modal_energy
/slosh/modal_energy_norm
/slosh/h_visual
/slosh/h_visual_quality
/camera/color/image_raw
/camera/depth/image_rect_raw
/imu/data
/odom
/cmd_vel
/mpc_status
/tf
/tf_static
```

录包示例：

```bash
cd $(rospack find scout_local_planner)
SLOSH_BAG_DIR=/data/$USER/slosh_bags/real/20260508_phase1 \
./scripts/record_slosh_experiment.sh 0 GEOREF_OSCRS_ACTIVE_REAL_open_goal_run01
```

## 11. 调参纪律

调参原则见：

```text
docs/重要文档/20260508_Online_GeoRef_OSCRS实物录包SOP.md
```

最重要的规则：

```text
一次只改一类参数；
RAW / GEOREF / OSCRS 三条线的 MPC 参数保持一致；
不得通过重新打开 Q_slosh / speed governor / PROFILE_ENERGY 来救主表；
每次调参记录参数名、旧值、新值、对应 bag、修改原因。
```

推荐顺序：

```text
1. 先确认 candidate_report、active、fb、takeover 字段正常；
2. 先修 collision / costmap / frame 安全问题；
3. 再调 GeoRef 候选强度；
4. 再调 OSCRS hard gate；
5. 最后调 OSCRS score 权重。
```

停止条件：

```text
连续 2 包碰撞、人工接管或定位失效；
OSCRS takeover=1 但 h/eta_dot/energy 明显劣于 GeoRef；
OSCRS takeover 长期为 0，此时记录为 candidate set saturated，不继续盲调 score。
```

## 12. 实物判定

第一阶段只在 open 场地做，不在 maze/窄走廊做。

每个 condition 至少 3 包，正式结果优先 5 包均值。

GeoRef 成功标准：

```text
GEOREF_TUNED_STRONG_REAL 相对 RAW_REAL:
  /slosh/height h_rms 下降
  /slosh/height h_p95 下降，目标 >= 10%
  modal_energy_rms 下降，目标 >= 10%
  eta_dot_rms 不上升
  tracking_time <= RAW * 1.15
  ay_p95 不上升
  solve_success_ratio >= 0.97
  无碰撞、无人工接管、无明显定位丢失
```

OSCRS 成功标准：

```text
GEOREF_OSCRS_ACTIVE_REAL 相对 GEOREF_TUNED_STRONG_REAL:
  takeover=1 的包中，h_p95 / h_rms / h_max 至少一项进一步下降；
  eta_dot 不显著上升；
  candidate_report 能解释 selected path。
```

如果使用 RGB 离线视觉真值：

```text
视觉 h_p95 / h_max 的改善方向必须与 /slosh/height 一致。
如果 /slosh/height 下降但 RGB 真液面不降，不能声明真实液体晃动被抑制。
```

## 13. 当前未完成项

```text
1. 实物 RAW_REAL / GEOREF_TUNED_STRONG_REAL / GEOREF_OSCRS_ACTIVE_REAL 有效性对比尚未完成。
2. OSCRS 已完成在线 active 通路，但尚未证明实物收益。
3. Ferrari oracle 当前只作为离线上界/参考工具，曲线路径 NLP 仍未稳定收敛。
4. RGB 视觉真值走离线处理链，最终论文结论需要与 /slosh/height 交叉验证。
```

## 14. 推荐下一步

```text
1. 做 GEOREF_OSCRS_ACTIVE_REAL smoke x1；
2. 按 RAW -> GEOREF -> OSCRS 交错顺序录同一 open goal；
3. 每条线先录 3 包，若安全和数据质量正常扩到 5 包；
4. 用 /slosh/height 和 RGB 离线真值共同判断；
5. 实物结果出来前，不再用单包仿真宣称 OSCRS 有效。
```
