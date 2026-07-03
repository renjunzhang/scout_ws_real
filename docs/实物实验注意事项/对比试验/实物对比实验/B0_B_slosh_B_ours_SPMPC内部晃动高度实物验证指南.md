# B0 / B_slosh / B_ours：SPMPC 内部晃动高度实物验证指南（当前版）

> 目的：在实物 Scout 跟踪同一条 fixed S-curve 路径时，对比 `B0`、`B_slosh` 与 `B_ours` 的 **SPMPC 内部液体晃动模型输出**，先证明 slosh-aware 项与 ours 组合策略在模型层面是否能降低运行过程中的晃动风险。
>
> 当前排障阶段先采用“模型内部预测/observer 值就是真值”的调试假设，**RGB 真值先不作为本轮定量结论**。RGB 仍可录制，作为后续物理真值补充证据。

---

## 1. 当前核心问题

本轮实验先回答下面的问题：

```text
在同一实物平台、同一容器/液位、同一 fixed S-curve 路径、同一 v_ref、同一共享约束和同一 delay compensation 设置下：

1. B_slosh 是否比 B0 产生更低的 SPMPC 内部模型晃动？
2. B_ours 是否在 B_slosh 基础上兼顾晃动、控制平滑性和 tracking？
3. 如果没有降低，问题发生在 MPC 初始状态、optimizer slosh pressure，还是命令执行链路？
```

这句话是本轮调试的核心：

```text
既然模型预测是真值，为什么优化器没有把这个真值压下来？
```

本轮实验原则：

```text
每个 bag 都录完整诊断；每个 run 只改一个实验变量。
```

也就是说，同一包里可以同时录 cost、delay、command、slosh observer、solver status 这些被动诊断量；但不要同一包里又改 delay、又改权重、又开 IMU、又开 hard，否则事后仍然分不清因果。

---

## 2. 本实验能证明什么 / 不能证明什么

### 2.1 能证明

如果 `B_slosh`、`B_ours` 的内部模型指标低于 `B0`，可以证明：

```text
在 SPMPC 自己的液体动力学模型和实物 odom 反馈口径下，
slosh-aware soft cost / ours 组合策略确实降低了模型层面的晃动风险。
```

这对论文/报告的价值是：即使 RGB 液面识别后续有噪声，也能先建立一条内部机制证据链。

### 2.2 不能单独证明

本实验不能单独证明真实液体液面一定更低。真实液面最终仍建议用：

```text
/camera/color/image_raw -> 离线 RGB max-LCR 复算
```

作为物理真值证据。

推荐论文/报告分开写：

```text
证据 A：SPMPC internal model evidence：B0 vs B_slosh vs B_ours。
证据 B：RGB physical liquid evidence：B0 vs B_slosh vs B_ours。
```

本文件只覆盖证据 A 和当前实物调试流程。

---

## 3. 三个方法定义

### 3.1 B0

variant 名称：

```text
B0
```

含义：

```text
slosh_enable: false
smooth_priority_enable: false
slosh_constraint_enable: false
w_slosh: 0.0
```

解释：`B0` 不把 slosh-aware 项放进优化目标/约束，但 wrapper 仍发布统一内部 slosh observer 和 debug topic，用于评估 B0 的实际运动激发出的模型晃动。

### 3.2 B_slosh

variant 名称：

```text
B_slosh
```

含义：

```text
slosh_enable: true
smooth_priority_enable: false
slosh_constraint_enable: false
```

解释：只打开 slosh-aware soft cost，用于验证“液体晃动项本身”是否能降低模型晃动。

### 3.3 B_ours

variant 名称：

```text
B_ours
```

含义：

```text
slosh_enable: true
smooth_priority_enable: true
slosh_constraint_enable: false
```

解释：在 slosh-aware soft cost 基础上叠加 smooth priority，目标是在保持 tracking / 到点的同时降低模型晃动和控制尖峰。

### 3.4 当前不作为主线的 hard variant

`B_slosh_hard`、`B_ours_hard` 暂时不作为本轮主线。原因：

```text
1. hard constraint 当前是 modal-only 口径，不等于 RGB 真值 hard cap；
2. delay compensation 尚需先稳定；
3. 本轮先判断 soft cost / ours 策略在模型层面是否恢复仿真规律。
```

hard-cap 相关实验等 delay 和 soft-cost 规律跑通后再单独设计。

---

## 4. 当前标准路径与终点

本实验采用 fixed-path tracking，不是自由点到点导航。

当前一键脚本默认终点来自 2026-07-02 实物 bag 中 `/scout/global_path_fixed` 的终点：

```text
GOAL_X=-5.424
GOAL_Y=-4.736
GOAL_YAW=0.0
GOAL_FRAME=map
```

说明：旧 bag 中未保留原始 `/scout/goal` yaw；`template_fixed_path_generator.py` 当前主要使用 goal x/y。

S-curve 默认参数：

```text
PATH_TEMPLATE=s_curve
PATH_START_HEADING=current
PATH_AMPLITUDE_RATIO=0.18
PATH_MIN_AMPLITUDE=0.25
PATH_MAX_AMPLITUDE=1.20
PATH_SIDE=left
PATH_SMOOTH_ITERATIONS=3
PATH_SPACING=0.05
```

当前推荐使用方案：

```text
固定终点 + 每 run 从当前起点重新生成 S-curve。
```

原因：实物回到起点时很难做到完全一致；从当前起点重新生成路径可以避免 path start 与车体当前位置明显不匹配。后处理时必须使用每个 run 实际 bag 里的 `/scout/global_path_fixed` 计算 tracking。

---

## 5. 当前统一控制口径

本轮不使用经验地图，也不使用 Map-vref：

```bash
rosparam set /spmpc_local_planner/map_vref/runtime_v_ref_enable false
rosparam set /spmpc_local_planner/map_vref/profile_enable false
```

期望：

```text
/spmpc/debug/map_vref_status = VARIANT_FALLBACK
```

当前标准低速 smoke / formal debug 统一使用：

```text
v_ref=0.20 m/s
shared_linear_accel_max=0.6 m/s^2
shared_angular_rate_max=1.2 rad/s
shared_angular_accel_max=1.2 rad/s^2
alpha_max=1.2 rad/s^2
timeout=60 s
```

当前 delay compensation 口径改为：

```text
delay_phase_mode=fixed_closed_loop
delay_phase_linear_delay_sec=0.08
delay_phase_angular_delay_sec=0.05
```

旧配置：

```text
linear_delay_sec=0.15
angular_delay_sec=0.22
```

只作为问题复现实验或 old-delay 对照，不作为当前正式推荐口径。尤其 `angular_delay_sec=0.22` 目前怀疑过补偿。

---

## 6. 当前推荐启动流程

### 6.1 前置：实物栈与观察模块

先按总启动指南完成前四步：

```text
docs/实物实验注意事项/对比试验/实物对比试验启动指南.md
```

至少确认：

```text
1. 实物传感器/定位/底盘栈已启动；
2. /odom、/map、/scan_front、/tf 正常；
3. RealSense RGB 已启动，若本轮要保留 RGB 证据则固定 exposure/gain/white balance；
4. standalone slosh monitor 可选启动，用于统一评价 proxy；
5. 现场遥控/急停就位。
```

最小检查：

```bash
rostopic echo -n 1 /odom
rostopic echo -n 1 /map
rostopic echo -n 1 /scan_front
rosrun tf tf_echo map base_link
rostopic info /cmd_vel
```

注意：任一时刻只允许一个 planner 有效发布 `/cmd_vel`。

### 6.2 一键执行固定路径 + 录包 + SPMPC

当前推荐使用一键脚本执行后续流程：

```text
启动 fixed-path generator -> 启动黑匣子 recorder -> 发送固定终点 -> 等待 /scout/global_path_fixed -> 启动 SPMPC variant -> 60s 或 Ctrl-C 后清理
```

脚本：

```text
src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

最小命令模板：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

ALG=B0 \
RUN_LABEL=B0_delay_080_050_run01 \
CMD_TOPIC=/cmd_vel \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

脚本默认值已经是当前推荐口径：

```text
V_REF=0.20
DELAY_PHASE_MODE=fixed_closed_loop
DELAY_PHASE_LINEAR_DELAY_SEC=0.08
DELAY_PHASE_ANGULAR_DELAY_SEC=0.05
RECORD_SEC=60
MAX_RECORD_SEC=60
GOAL_X=-5.424
GOAL_Y=-4.736
GOAL_YAW=0.0
```

`RECORD_SEC` 必须在 `1..60`；如果设成 0 或超过 60，脚本会强制回到 60。现场想提前停，按 `Ctrl-C`。

---

## 7. 当前推荐：delay 归因最小测试矩阵

当前还不是正式统计阶段，先不要一开始就按 `B0 -> B_slosh -> B_ours -> N=3` 盲跑；本轮优先回答：

```text
在同一版新代码、同一路径/容器/液位/v_ref/shared limits 下，只改变 delay，
B_slosh 的内部模型晃动是否从异常恢复？
```

每个 run 之间：

```text
脚本自动停 planner/recorder/generator -> 确认 /cmd_vel 为 0 -> 遥控回起点标记 -> 液体静稳 60~90s -> 开下一次 run
```

推荐每个 bag 保留一点静止基线：

```text
0–5 s：静止，等待 topic 全部发布；
5–60 s：执行 fixed S-curve；
60–65 s：若已到点，继续观察残余 slosh / terminal 行为。
```

当前一键脚本默认 recorder 先启动、再发送 goal、再启动 planner；运动开始可在后处理中用 `/cmd_vel` 第一次非零时刻或 `/spmpc/status` 状态变化自动识别。

如果本轮需要保留 RGB 原始证据，把 `RECORD_RGB=false` 改为 `RECORD_RGB=true`；但当前结论仍限定为 internal model evidence。

### 7.0 Run 0：静态配置检查（约 10s）

目的：正式动起来前确认新 topic、effective config、run meta 和 delay 参数真的落地。若希望不驱动车体，可把命令发布到 shadow topic：

```bash
ALG=B_slosh \
RUN_LABEL=static_config_check_10s \
CMD_TOPIC=/spmpc_shadow_cmd_vel \
RECORD_SEC=10 \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

必须检查：

```text
/spmpc/debug/effective_config
/spmpc/debug/raw_state
/spmpc/debug/predicted_state
/spmpc/debug/solver_input_state
/spmpc/debug/slosh_cost_monitor
/spmpc/debug/command_intervention
/spmpc/debug/cmd_odom_alignment
```

若 topic 缺失、effective config 与 run meta 不一致、`fixed_closed_loop` 未生效，先停下修配置再跑正式 run。静态检查阶段的 history completeness 可能受启动瞬间 command history 影响，正式判断仍以后续运动 run 的 summary 为准。

### 7.1 Run 1：B_slosh 新 delay

```bash
ALG=B_slosh \
RUN_LABEL=Bslosh_delay_080_050_run01 \
CMD_TOPIC=/cmd_vel \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.08 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.05 \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

目的：看 slosh-aware soft cost 在当前推荐 delay 下是否恢复模型层优势。

### 7.2 Run 2：B_slosh 旧 delay，同一版新代码复现

```bash
ALG=B_slosh \
RUN_LABEL=Bslosh_delay_150_220_run01 \
CMD_TOPIC=/cmd_vel \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

目的：只改 delay，观察旧 delay 是否重新导致 B_slosh 变差。旧 bag 只能作为背景证据，不能作为唯一 old-delay 对照，因为代码版本、诊断链路和参数落地方式已经变化。

### 7.3 Run 3：B0 新 delay baseline

```bash
ALG=B0 \
RUN_LABEL=B0_delay_080_050_run01 \
CMD_TOPIC=/cmd_vel \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.08 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.05 \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

注意：`B0` 不启用 slosh prediction，不要把 horizon peak 与 `B_slosh` 直接等价比较；比较 `B0` 时主要看 `/spmpc/slosh_height`、tracking、cmd smoothness、completion time 和 command intervention。

### 7.4 Run 4：B_ours 新 delay

```bash
ALG=B_ours \
RUN_LABEL=Bours_delay_080_050_run01 \
CMD_TOPIC=/cmd_vel \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.08 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.05 \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

目的：若 `B_slosh` 已经恢复规律，再看完整策略是否兼顾 slosh、smooth 和 tracking。

### 7.5 可选：B_slosh delay off / shadow

时间允许时补一个：

```bash
ALG=B_slosh \
RUN_LABEL=Bslosh_delay_off_run01 \
CMD_TOPIC=/cmd_vel \
DELAY_PHASE_MODE=off \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

判读：

```text
delay_off > old_delay 但 < new_delay：新 delay 有帮助；
delay_off 最好：当前 compensation 模型可能仍有问题，不只是 delay 数值问题。
```

---

## 8. 正式 N=3 交错顺序

只有当上面的 delay 归因矩阵说明新 delay 下链路干净、`B_slosh` 规律可信后，再进入正式 N=3。不要连续跑完一个方法再跑另一个方法，推荐交错：

```text
Round 1: B0 -> B_slosh -> B_ours
Round 2: B_ours -> B_slosh -> B0
Round 3: B_slosh -> B0 -> B_ours
```

命名建议：

```text
B0_delay_080_050_r1
Bslosh_delay_080_050_r1
Bours_delay_080_050_r1
...
```

如果 delay 归因矩阵或 N=1 中出现下面任一情况，先停止，不进入 N=3：

```text
明显离轨；
原地转圈；
需要人工急停；
/odom、/tf、/map 异常；
新 debug topic 缺失；
summary red flags 指向 solver input 相位错误或命令链路大量改写。
```

本轮不建议跑 hard：hard 约束会混入可行性/求解失败问题，不适合作为 delay 诊断主线。

---

## 9. 必录 topic（当前版）

黑匣子 recorder 已在 whitelist 中包含以下关键 topic。

### 9.1 主模型指标

```text
/spmpc/slosh_height
/spmpc/debug/slosh_state
/spmpc/slosh_horizon_summary
/spmpc/debug/slosh_cost_monitor
/spmpc/cost_breakdown
```

说明：

```text
/spmpc/slosh_height：运行中统一内部 slosh observer 的模型液面高度 proxy，单位 mm。
/spmpc/slosh_horizon_summary：solver horizon 内预测峰值/p95，单位 mm。
/spmpc/debug/slosh_cost_monitor：slosh cost 是否真的 active、占比多少。
```

### 9.2 当前新增四个关键链路 topic

```text
/spmpc/debug/raw_state
/spmpc/debug/predicted_state
/spmpc/debug/solver_input_state
/spmpc/debug/command_intervention
```

判读目的：

```text
raw_state：controller 入口看到的原始 robot/slosh 状态；
predicted_state：delay phase 根据 cmd history 前向积分得到的预测当前状态；
solver_input_state：problem_.solve() 实际收到的最终输入状态；
command_intervention：solver 命令经过 ROS gate、shared limiter、zero path 后是否被改写。
```

### 9.3 delay / timing / command 证据

```text
/spmpc/debug/effective_config
/spmpc/debug/delay_phase
/spmpc/debug/odom_timing
/spmpc/debug/execution_state
/spmpc/debug/execution_alignment_status
/spmpc/debug/delay_compensation
/spmpc/debug/cmd_odom_alignment
/spmpc/debug/cmd_vel_output
/spmpc/debug/cmd_vel_output_status
/cmd_vel
/cmd_vel_drive
/odom
/tf
/tf_static
```

### 9.4 路径 / goal / 环境证据

```text
/scout/global_path_fixed
/scout/goal
/map
/scan_front
/camera/color/image_raw
/camera/color/camera_info
/slosh/height
/slosh/state
/liquid/height
/liquid/height_lcr
```

录包后必须检查：

```text
${RUN_LABEL}_recorded_topics.txt
${RUN_LABEL}_selected_topics_not_recorded.txt
```

注意：一键脚本先启动 recorder 再启动 planner，所以新 topic 在 recorder 开始瞬间可能出现在 `missing_selected_topics_at_start.txt`，这是正常的；只要 planner 启动后发布，bag 中仍会录到，最终以 `recorded_topics.txt` 和 summary 为准。

---

## 10. 每个 run 结束后的立即检查

每个 run 结束后，先跑 summary：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

python3 src/scout_apps/control/spmpc_local_planner/scripts/summarize_spmpc_real_trial.py \
  /home/geist/slosh_bags/real/${DATE}_fixed_path_compare/${ALG}/${RUN_LABEL}.bag
```

脚本会输出：

```text
${RUN_LABEL}_summary.json
${RUN_LABEL}_summary.md
```

如果不确定 bag 在哪里，看一键脚本最后打印的：

```text
bag/meta dir = ...
```

### 10.1 必须先看 red flags

如果 summary 出现下面 red flags，先不要把该 run 当正式有效样本：

```text
critical_topic_missing
fixed_closed_loop_not_applied
delay_history_incomplete
solver_input_phase_shift_large
command_limited_often
published_zero_often
solver_fail_or_gate_fail
sidecar_effective_config_mismatch
```

### 10.2 当前最重要的三段链路

#### A. MPC 初始状态是否正确

看：

```text
/spmpc/debug/raw_state
/spmpc/debug/predicted_state
/spmpc/debug/solver_input_state
/spmpc/debug/cmd_odom_alignment
```

注意：`predicted_state - raw_state` 有差异不一定是错，因为 fixed closed-loop compensation 本来就是把 raw state 往前推。更可靠的判断是：

```text
predicted_state(t) 是否接近 raw_state(t + delay)
```

因此本轮先用 summary 的 raw→predicted shift 做快速筛查，离线细查时应对齐未来 raw state：

```text
predicted_vs_future_raw_yaw_error
predicted_vs_future_raw_omega_error
predicted_vs_future_raw_eta_error
predicted_vs_future_raw_eta_dot_error
```

判读：

```text
predicted 比 future raw 还推得更远：过补偿；
predicted 落后 future raw：补偿不足；
predicted 接近 future raw：补偿合理。
```

快速期望：

```text
fixed_closed_loop_applied_frac 高；
history_complete_frac 高；
solver_input_state 与 raw/future raw 的 yaw / omega / eta 偏移合理；
不要出现明显过补偿或欠补偿。
```

#### B. optimizer 是否真的在压 slosh

看：

```text
/spmpc/debug/slosh_cost_monitor
/spmpc/slosh_horizon_summary
/spmpc/cost_breakdown
```

期望：

```text
B_slosh / B_ours 的 J_slosh_total 非零；
pct_slosh_total_abs_sum 有可见占比；
h_modal_peak_pred_mm / h_modal_p95_pred_mm 低于 B0 或至少不恶化；
eta_dot_norm_peak 低于 B0 或至少不恶化。
```

#### C. solver 命令是否被执行链路改写

看：

```text
/spmpc/debug/command_intervention
/spmpc/debug/cmd_vel_output
/cmd_vel
/odom
```

期望：

```text
published_cmd 基本等于 post_gate_cmd；
linear_limited_frac 不高；
angular_rate_limited_frac 不高；
angular_accel_limited_frac 不高；
published_zero_frac 不高；
zero_due_to_solver_failure / terminal_spin_fail / tracking_safety 基本为 0。
```

如果 `command_limited_often`、`published_zero_often` 或 `solver_fail_or_gate_fail` 出现，该 run 只能用于 debugging，不能直接作为 clean delay attribution 或 clean method comparison。

### 10.3 建议自动判读阈值

第一版阈值只用于筛查，不作为最终物理结论：

```text
fixed_closed_loop_applied_frac < 0.95 -> red
history_complete_frac < 0.95 -> red
command_limiter_frac > 0.20 -> yellow
published_zero_frac > 0.05 -> red
solver_fail_count > 0 for soft variants -> red

|delta_yaw| > 0.10 rad -> yellow
|delta_yaw| > 0.20 rad -> red
|delta_omega| > 0.15 rad/s -> yellow
|delta_omega| > 0.30 rad/s -> red
delta_height_proxy > 0.5 mm -> yellow
delta_height_proxy > 1.0 mm -> red
|delta_eta| / eta_ref > 0.25 -> yellow
|delta_eta| / eta_ref > 0.50 -> red
|delta_eta_dot| / eta_dot_ref > 0.25 -> yellow
|delta_eta_dot| / eta_dot_ref > 0.50 -> red
```

---

## 11. 成功 / 失败 / 无效判定

### 11.1 成功 run

```text
60s 内到点；
无急停；
无明显离轨/转圈；
关键 topic 完整；
summary 无关键 red flag；
/spmpc/slosh_height、/spmpc/slosh_horizon_summary、/spmpc/debug/slosh_cost_monitor 有足够样本；
/scout/global_path_fixed 和 /odom 可用于 tracking 后处理。
```

### 11.2 方法失败，进入统计

```text
60s 未到点；
明显离轨；
原地转圈或小 linear.x + 大 angular.z 持续约 2s；
SPMPC status 出现 TRACKING_UNSAFE_PROJECTION / TRACKING_SPIN_FAIL / ACADOS_SOLVE_FAILED；
需要人工急停。
```

失败 run 不能删除。即使失败，也要保留 bag/log，因为失败可能伴随高模型晃动风险或揭示 command intervention 问题。

### 11.3 无效 run，保留但不进正式统计

```text
起点明显偏离地面标记；
路径生成错误或穿墙/贴障碍物；
定位/TF/odom/map 异常；
bag 缺关键 topic；
容器松动、液位变化、现场人员干扰；
summary 显示 solver input 相位明显错误或命令链路大量改写；
RGB/相机异常只影响 RGB 物理真值，不必然使内部模型实验无效，除非 bag/时间同步也受影响。
```

---

## 12. 主指标与辅助指标

### 12.1 内部模型主指标

主指标来自：

```text
/spmpc/slosh_height
/spmpc/slosh_horizon_summary
```

每个 run 至少报告：

| 指标                       | 含义                                                    |
| -------------------------- | ------------------------------------------------------- |
| `internal_slosh_peak_mm` | `/spmpc/slosh_height` 全程最大值                      |
| `internal_slosh_p95_mm`  | `/spmpc/slosh_height` 全程 p95                        |
| `internal_slosh_rms_mm`  | `/spmpc/slosh_height` RMS                             |
| `horizon_h_peak_p95_mm`  | `/spmpc/slosh_horizon_summary` 中 horizon peak 的 p95 |
| `horizon_h_p95_p95_mm`   | `/spmpc/slosh_horizon_summary` 中 horizon p95 的 p95  |

### 12.2 必须同时报告的控制/执行指标

避免“慢所以晃得小”的误判，必须同时报告：

| 指标                                | 来源                                            |
| ----------------------------------- | ----------------------------------------------- |
| `first_goal_time_sec`             | `/spmpc/status`                               |
| `tracking_rms_m / p95 / max`      | `/odom` vs `/scout/global_path_fixed`       |
| `cmd_v_mean / p95 / max`          | `/cmd_vel` 或 `/spmpc/debug/cmd_vel_output` |
| `cmd_omega_p95 / max`             | `/cmd_vel` 或 `/spmpc/debug/cmd_vel_output` |
| `solver_time_p95 / max`           | `/spmpc/solver_time_ms`                       |
| `delay_compensation_applied_frac` | `/spmpc/debug/solver_input_state`             |
| `history_complete_frac`           | `/spmpc/debug/cmd_odom_alignment`             |
| `command_limiter_frac`            | `/spmpc/debug/command_intervention`           |
| `published_zero_frac`             | `/spmpc/debug/command_intervention`           |

---

## 13. 最小后处理表格

每个 run 一行：

| run | method  | success | time_s | tracking_rms_m | slosh_peak_mm | slosh_p95_mm | slosh_rms_mm | horizon_peak_p95_mm | cmd_v_mean | cmd_w_p95 | delay_applied_frac | limiter_frac | note |
| --: | ------- | ------- | -----: | -------------: | ------------: | -----------: | -----------: | ------------------: | ---------: | --------: | -----------------: | -----------: | ---- |
|   1 | B0      |         |        |                |               |              |              |                     |            |           |                    |              |      |
|   1 | B_slosh |         |        |                |               |              |              |                     |            |           |                    |              |      |
|   1 | B_ours  |         |        |                |               |              |              |                     |            |           |                    |              |      |

聚合表：

| method  | N | success | time mean±std | tracking RMS mean±std | slosh peak mean±std | slosh p95 mean±std | slosh RMS mean±std | horizon peak p95 mean±std |
| ------- | -: | ------: | -------------: | ---------------------: | -------------------: | ------------------: | ------------------: | -------------------------: |
| B0      |   |         |                |                        |                      |                     |                     |                            |
| B_slosh |   |         |                |                        |                      |                     |                     |                            |
| B_ours  |   |         |                |                        |                      |                     |                     |                            |

核心比较句：

```text
B_slosh vs B0: internal_slosh_p95 降低 xx%，internal_slosh_peak 降低 xx%，说明 slosh-aware soft cost 在模型层面有效。
B_ours vs B_slosh: internal_slosh 指标维持/进一步降低 xx%，同时 cmd_w_p95、tracking 或终端停车表现更优/不变。
B_ours vs B0: 汇总说明 ours 相对无 slosh baseline 的总体收益。
```

---

## 14. 结论表达口径

如果结果成立，推荐写法：

```text
在同一 fixed S-curve 实物路径、同一速度/约束和相同 delay compensation 设置下，
B_slosh 相比 B0 降低了 SPMPC internal slosh observer 的 /spmpc/slosh_height peak/p95/RMS，
说明 slosh-aware soft cost 本身在模型层面具有抑制晃动风险的作用。
B_ours 在 B_slosh 基础上叠加 smooth priority 后，进一步观察其是否在保持相近到点时间、tracking 误差的同时，改善控制平滑性并维持或进一步降低内部晃动指标。
```

如果 RGB 后续可靠，可补一句：

```text
离线 RGB max-LCR 结果进一步验证了这种模型层面的降低趋势也反映到真实液面响应中。
```

如果 RGB 不可靠，不要强行写真实液面结论，只写：

```text
RGB 链路本轮仅作记录/可视化，不作为定量真值；
本节结论限定为 SPMPC internal model evidence。
```

如果下一轮仍然出现 `B0` 更好，先按 debug topic 分层解释：

```text
1. solver_input_state 相对 raw_state 偏移大：优先怀疑 delay compensation；
2. solver input 正常但 horizon slosh 不下降：优先查权重/代价/约束设计；
3. horizon slosh 下降但 published_cmd 被大量改写：优先查 shared limiter / gate / 底盘执行链路；
4. 模型指标下降但 RGB 不下降：再进入模型-真实液体一致性问题。
```

---

## 15. 每个 run 现场记录模板

```text
run_name:
date/time:
operator:
method: B0 / B_slosh / B_ours
run_index:
run_type: N1_smoke / formal_N3

script: run_spmpc_real_fixed_path_trial.sh
cmd_topic: /cmd_vel / /spmpc_shadow_cmd_vel
record_sec: 60
run_out_dir:
bag_file:
summary_json:
summary_md:

path_mode: fixed S-curve
path_generation: regenerate_from_current_start
path_file:
start_marker:
start_pose_map:
goal_x/y/yaw: -5.424 / -4.736 / 0.0
S_curve_params: ratio=0.18,min=0.25,max=1.20,side=left,smooth=3,spacing=0.05

container:
liquid_level:
payload_fixed: yes/no
RGB_recorded: yes/no
RGB_calibration:

map_vref_runtime_v_ref_enable: false
map_vref_profile_enable: false
map_vref_status:

delay_phase_mode: fixed_closed_loop
linear_delay_sec: 0.08
angular_delay_sec: 0.05
delay_compensation_applied_frac:
history_complete_frac:
solver_input_phase_shift_large: yes/no

v_ref: 0.20
shared_linear_accel_max: 0.6
shared_angular_rate_max: 1.2
shared_angular_accel_max: 1.2
timeout_sec: 60

/spmpc/slosh_height samples:
/slosh_horizon_summary samples:
/slosh_cost_monitor samples:
command_intervention samples:
red_flags:

result: success / timeout / e-stop / planner_fail / invalid
first_goal_time_sec:
tracking notes:
obvious off-track: yes/no
obvious spin: yes/no
manual intervention: yes/no
notes:
```
