# B0 / B_slosh / B_ours：SPMPC 内部晃动高度实物验证指南

> 目的：在实物 Scout 运行同一条 fixed S-curve 路径时，对比 `B0`、`B_slosh` 与 `B_ours` 的 **SPMPC 内部液体晃动模型输出**，证明 slosh-aware 项与 ours 组合策略在模型层面确实降低了运行过程中的晃动风险。  
> 这组实验不依赖 RGB 液面识别准确性；RGB 仍建议录制，但作为后续物理真值补充证据。

---

## 1. 核心问题

本实验回答的问题是：

```text
在同一实物平台、同一容器/液位、同一 fixed S-curve 路径、同一速度/约束口径下，
B_slosh 是否比 B0 产生更低的 SPMPC 内部模型晃动高度？
B_ours 是否在 B_slosh 基础上进一步兼顾更低晃动、平滑控制和 tracking？
```

这里的“内部模型晃动高度”主指标是：

```text
/spmpc/slosh_height
```

它是 SPMPC wrapper 内部统一 slosh observer 根据实物运行中的 odom/速度响应计算并发布的模型液面高度 proxy，单位是 **mm**。

---

## 2. 这组实验能证明什么 / 不能证明什么

### 2.1 能证明

如果结果显示 `B_slosh`、`B_ours` 的 `/spmpc/slosh_height` 明显低于 `B0`，可以证明：

```text
在 SPMPC 自己的液体动力学模型和实物运动反馈口径下，
slosh-aware 优化确实降低了模型预测/估计的晃动风险；
B_ours 再用于观察 smooth priority 与 slosh-aware cost 组合后是否进一步改善控制平滑性、tracking 或晃动风险。
```

这对论文很有用，因为即使后续 RGB 液面识别有噪声或局部失败，也能先建立：

```text
算法机制在模型层面是有效的，不是只靠 RGB 后处理偶然得出。
```

### 2.2 不能单独证明

这组实验不能单独证明真实液体液面一定更低。真实液体最终仍建议用：

```text
/camera/color/image_raw -> 离线 RGB max-LCR 复算
```

作为物理真值证据。

论文/报告口径建议分开写：

```text
证据 A：SPMPC internal slosh model evidence：B0 vs B_slosh vs B_ours。
证据 B：RGB physical liquid evidence：B0 vs B_slosh vs B_ours。
```

本文件只覆盖证据 A。

---

## 3. 方法定义

### 3.1 B0

repo 中实际 variant 名称：

```text
B0
```

配置含义：

```text
slosh_enable: false
smooth_priority_enable: false
slosh_constraint_enable: false
w_slosh: 0.0
```

实验解释：

```text
B0 不把 slosh-aware 项放进优化目标/约束；
但 wrapper 仍可发布统一内部 slosh observer 的 /spmpc/slosh_height，
用于评估 B0 这套运动实际激发出的模型晃动风险。
```

### 3.2 B_slosh

repo 中实际 variant 名称：

```text
B_slosh
```

配置含义：

```text
slosh_enable: true
smooth_priority_enable: false
slosh_constraint_enable: false
w_slosh: 5.0
```

实验解释：

```text
B_slosh 只打开 slosh-aware soft cost，不打开 smooth priority。
它用于验证“液体晃动项本身”是否能降低 /spmpc/slosh_height。
```

### 3.3 B_ours

repo 中实际 variant 名称：

```text
B_ours
```

配置含义：

```text
slosh_enable: true
smooth_priority_enable: true
slosh_constraint_enable: false
w_slosh: 5.0
```

实验解释：

```text
B_ours 在 SPMPC 优化中显式使用 slosh-aware soft cost，
并叠加 smooth priority，目标是在保持 tracking/到点的同时降低模型晃动和控制尖峰。
```

---

## 4. 实验路径口径

本实验采用 fixed-path tracking，不是自由点到点导航。

正式口径：

```text
固定起点标记 + 固定终点 + 固定 S-curve 生成规则。
每个 run 从当前实物起点 pose 到固定终点生成 /scout/global_path_fixed，
planner 跟踪 /scout/global_path_fixed。
/scout/goal 只用于路径生成和终端成功判定。
```

推荐 S-curve 参数与仿真 fixed-path 对齐：

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

### 4.1 更强公平性的路径选择

二选一：

#### A. 固定终点 + 每 run 重新生成路径（现场推荐）

优点：起点 pose 小偏差不会导致路径起点和当前车体不匹配。  
缺点：不同 run 的路径会有轻微差异。

要求：每个 run 必须保存：

```text
path_file
/scout/global_path_fixed bag topic
start pose
goal_x/y/yaw
S-curve 参数
```

后处理 tracking 误差按每个 run 实际发布的 `/scout/global_path_fixed` 计算。

#### B. 首次生成一次 S-curve，后续 replay 同一个 path_file（严格同路径）

优点：B0/B_slosh/B_ours 跟踪完全同一条 map-frame path。  
缺点：要求每次回到起点非常准；如果起点偏差大，起步 tracking 会不公平。

建议 gate：

```text
start position error <= 0.05 m
start yaw error <= 5 deg
```

如果达不到，退回方案 A。

---

## 5. 必须关闭经验地图 / Map-vref

本实验不使用经验地图，也不使用 Map-vref：

```bash
rosparam set /spmpc_local_planner/map_vref/runtime_v_ref_enable false
rosparam set /spmpc_local_planner/map_vref/profile_enable false
```

期望：

```text
/spmpc/debug/map_vref_status = VARIANT_FALLBACK
```

不能出现：

```text
PROFILE_LOOKUP
RUNTIME_OVERRIDE
```

---

## 6. 推荐速度与相位延迟口径

为了让 `B0`、`B_slosh` 与 `B_ours` 的差异来自算法本身，下面设置必须一致：

```text
same path rule
same v_ref
same acceleration/angular limits
same delay_phase_mode
same container/liquid level
same start marker
same terminal goal
same bag topic whitelist
```

### 6.1 第一轮低速 smoke

```text
v_ref=0.20 m/s
shared_linear_accel_max=0.6 m/s^2
shared_angular_rate_max=1.2 rad/s
shared_angular_accel_max=1.2 rad/s^2
timeout=60 s
```

### 6.2 正式实物对比

低速 smoke 通过后可提高到：

```text
v_ref=0.50 m/s
```

若现场安全、tracking 和停稳都稳定，再考虑：

```text
v_ref=0.65 m/s
```

### 6.3 相位延迟补偿

`B0`、`B_slosh`、`B_ours` 都有同一套相位延迟显示/补偿开关。它是 SPMPC wrapper-level 的 `delay_phase_mode`，不是某个 variant 独有的参数。

如果本轮实物要求相位延迟开启，三组必须同时使用完全相同的设置：

```text
delay_phase_mode=fixed_closed_loop
delay_phase_linear_delay_sec=0.15
delay_phase_angular_delay_sec=0.22
```

如果只是先看诊断、不让补偿参与控制，则三组都用：

```text
delay_phase_mode=shadow
```

不要一组开、一组不开；否则 slosh 差异会混入延迟补偿差异。

---

## 7. 必录 topic

主证据 topic：

```text
/spmpc/slosh_height
/spmpc/debug/slosh_state
/spmpc/status
/spmpc/debug/progress_s
/spmpc/solver_time_ms
/spmpc/controller_variant
/spmpc/solver_backend
/spmpc/debug/map_vref_status
/spmpc/debug/cmd_vel_output
/cmd_vel
/cmd_vel_drive
/odom
/tf
/tf_static
/scout/global_path_fixed
/scout/goal
```

相位延迟证据 topic：

```text
/spmpc/debug/delay_phase
/spmpc/debug/odom_timing
/spmpc/debug/execution_state
/spmpc/debug/execution_alignment_status
/spmpc/debug/delay_compensation
```

辅助 topic：

```text
/spmpc/slosh_horizon_summary
/spmpc/cost_breakdown
/spmpc/local_trajectory
/spmpc/debug/runtime_bounds
/spmpc/debug/generated_bounds
/spmpc/debug/slosh_hard_constraint
/slosh/height
/slosh/state
/camera/color/image_raw
/camera/color/camera_info
/liquid/height
/liquid/height_lcr
```

注意：

```text
/spmpc/slosh_height：主比较指标，单位 mm。
/spmpc/slosh_horizon_summary[0]：h_peak_pred_mm，优化 horizon 内预测峰值。
/spmpc/slosh_horizon_summary[1]：h_p95_pred_mm，优化 horizon 内预测 p95。
```

对 `B0` 来说，优化器中的 slosh horizon 可能因为 `slosh_enable=false` 而不作为主要可比值；因此本实验主指标固定使用 `/spmpc/slosh_height` 与 `/spmpc/debug/slosh_state`。

---

## 8. 启动前置

先按总启动指南完成：

```text
docs/实物实验注意事项/对比试验/实物对比试验启动指南.md
```

至少已经启动并确认：

```text
1. 实物传感器/定位/底盘栈；
2. RealSense RGB 固定参数；
3. 在线 RGB 观察；
4. standalone slosh monitor；
5. fixed S-curve path 生成/发布；
6. full RGB recorder 已准备。
```

起步前 gate：

```bash
rostopic info /cmd_vel
rostopic echo -n 1 /odom
rostopic echo -n 1 /map
rostopic echo -n 1 /scan_front
rostopic echo -n 1 /camera/color/image_raw
rostopic echo -n 1 /scout/global_path_fixed
rostopic echo -n 1 /scout/goal
```

现场人工确认：

```text
E-stop / 遥控急停就位: yes/no
/cmd_vel 无旧 planner publisher: yes/no
RGB online zero locked: yes/no
bag recorder 已启动: yes/no
RViz path/goal/定位正常: yes/no
机器人周围安全: yes/no
液体已静稳 60~90s: yes/no
```

---

## 9. 单 run recorder 模板

每个 run 先开 recorder，再开 planner。

```bash
ALG=B0   # B0 / B_slosh / B_ours
RUN_LABEL=internal_slosh_${ALG}_run01

VARIANT=${ALG} \
RUN_LABEL=${RUN_LABEL} \
RECORD_SEC=75 \
RECORD_CAMERA=true \
RECORD_SCAN=true \
RECORD_DEPTH=false \
RECORD_STANDALONE_SLOSH=true \
RECORD_ONLINE_LIQUID=true \
RECORD_ALL_EXISTING_TOPICS=false \
OUT_DIR=${OUT_DIR}/internal_slosh_${ALG} \
NAME=${RUN_LABEL}_rgb \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

如果第一次怀疑 topic 漏录，可以做一次短诊断：

```bash
RECORD_ALL_EXISTING_TOPICS=true RECORD_SEC=20 ... \
bash src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
```

但正式表建议使用 whitelist recorder，避免 bag 过大。

---

## 10. 三种 variant 通用启动命令

本实验跑三个 SPMPC variant：

```bash
ALG=B0       # 无 slosh cost / 无 smooth priority
ALG=B_slosh  # slosh-aware soft cost only
ALG=B_ours   # slosh-aware soft cost + smooth priority
```

### 10.1 shadow：只显示诊断，不驱动车体

```bash
CMD_TOPIC=/spmpc_shadow_cmd_vel
ALG=B0   # 改成 B_slosh 或 B_ours

rosparam set /spmpc_local_planner/map_vref/runtime_v_ref_enable false
rosparam set /spmpc_local_planner/map_vref/profile_enable false

roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=${ALG} \
  solver_backend:=continuous_mpcc_acados \
  reference_path_topic:=/scout/global_path_fixed \
  cmd_vel_topic:=${CMD_TOPIC} \
  costmap_topic:=/map \
  reference_target_frame:=map \
  delay_phase_mode:=shadow \
  delay_phase_linear_delay_sec:=0.15 \
  delay_phase_angular_delay_sec:=0.22 \
  v_ref:=0.20 \
  alpha_max:=1.2 \
  shared_linear_accel_limit_enable:=true \
  shared_linear_accel_max:=0.6 \
  shared_angular_limit_enable:=true \
  shared_angular_rate_max:=1.2 \
  shared_angular_accel_max:=1.2
```

### 10.2 actuated：实物闭环运行

```bash
CMD_TOPIC=/cmd_vel
ALG=B0   # 改成 B_slosh 或 B_ours

rosparam set /spmpc_local_planner/map_vref/runtime_v_ref_enable false
rosparam set /spmpc_local_planner/map_vref/profile_enable false

roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=${ALG} \
  solver_backend:=continuous_mpcc_acados \
  reference_path_topic:=/scout/global_path_fixed \
  cmd_vel_topic:=${CMD_TOPIC} \
  costmap_topic:=/map \
  reference_target_frame:=map \
  delay_phase_mode:=fixed_closed_loop \
  delay_phase_linear_delay_sec:=0.15 \
  delay_phase_angular_delay_sec:=0.22 \
  v_ref:=0.20 \
  alpha_max:=1.2 \
  shared_linear_accel_limit_enable:=true \
  shared_linear_accel_max:=0.6 \
  shared_angular_limit_enable:=true \
  shared_angular_rate_max:=1.2 \
  shared_angular_accel_max:=1.2
```

正式速度提高时只改：

```text
v_ref:=0.50
```

---

## 11. 推荐实验顺序

### 11.1 N=1 smoke

```text
1. B0 shadow：只看 topic、status、/spmpc/slosh_height 是否正常。
2. B0 actuated low-speed：v_ref=0.20，60s 内到点。
3. B_slosh shadow：只看 topic、status、/spmpc/slosh_height 是否正常。
4. B_slosh actuated low-speed：v_ref=0.20，60s 内到点。
5. B_ours shadow：只看 topic、status、/spmpc/slosh_height 是否正常。
6. B_ours actuated low-speed：v_ref=0.20，60s 内到点。
```

如果任一组出现明显离轨、转圈、急停、定位异常，停止本实验，不进入正式 N=3。

### 11.2 正式 N=3 交错

不要连续跑完一个方法再跑另一个方法。推荐：

```text
Round 1: B0 -> B_slosh -> B_ours
Round 2: B_ours -> B_slosh -> B0
Round 3: B_slosh -> B0 -> B_ours
```

若做 N=5：

```text
Round 1: B0 -> B_slosh -> B_ours
Round 2: B_ours -> B_slosh -> B0
Round 3: B_slosh -> B0 -> B_ours
Round 4: B_ours -> B0 -> B_slosh
Round 5: B0 -> B_ours -> B_slosh
```

每个 run 之间：

```text
停 planner -> 发 /cmd_vel zero -> 停 bag -> 回到同一起点标记 -> 等液体静稳 60~90s -> 查 /cmd_vel publisher -> 开下一轮 recorder -> 开 planner
```

---

## 12. 成功 / 失败 / 无效判定

### 12.1 成功

```text
60s 内到点；
无急停；
无明显离轨/转圈；
关键 topic 完整；
/spmpc/slosh_height 有足够样本；
/scout/global_path_fixed 和 /odom 可用于 tracking 后处理。
```

### 12.2 方法失败，进入统计

```text
60s 未到点；
明显离轨；
原地转圈或小 linear.x + 大 angular.z 持续约 2s；
SPMPC status 出现 TRACKING_UNSAFE_PROJECTION / SPIN_FAIL / ACADOS_SOLVE_FAILED；
需要人工急停。
```

失败 run 不能删除。即使失败，也要保留 `/spmpc/slosh_height`，因为失败可能伴随高模型晃动风险。

### 12.3 无效 run，保留但不进正式统计

```text
起点明显偏离地面标记；
路径生成错误或穿墙/贴障碍物；
定位/TF/odom/map 异常；
bag 缺关键 topic；
容器松动、液位变化、现场人员干扰；
RGB/相机异常只影响 RGB 物理真值，不必然使内部模型实验无效，除非 bag/时间同步也受影响。
```

---

## 13. 主指标

主指标只从 `/spmpc/slosh_height` 计算，单位 mm：

| 指标 | 含义 |
|---|---|
| `internal_slosh_peak_mm` | 全程最大模型晃动高度 |
| `internal_slosh_p95_mm` | 全程 p95，降低偶发尖峰影响 |
| `internal_slosh_rms_mm` | 全程 RMS |
| `internal_slosh_auc_mm_s` | 时间积分，反映持续晃动 |
| `core_internal_slosh_p95_mm` | progress 10%~90% 主体段 p95 |
| `core_internal_slosh_rms_mm` | progress 10%~90% 主体段 RMS |
| `terminal_internal_slosh_peak_mm` | 终端停车附近模型晃动峰值 |

必须同时报告控制/速度/跟踪，避免“慢所以晃得小”的误判：

| 指标 | topic |
|---|---|
| first goal time | `/spmpc/status` |
| tracking RMS / p95 / max | `/odom` vs `/scout/global_path_fixed` |
| cmd_v mean/p95/max | `/cmd_vel` 或 `/spmpc/debug/cmd_vel_output` |
| cmd_omega p95/max | `/cmd_vel` 或 `/spmpc/debug/cmd_vel_output` |
| solver p95/max | `/spmpc/solver_time_ms` |
| delay compensation active fraction | `/spmpc/debug/delay_compensation[1]` |

---

## 14. 推荐结论表达

如果结果成立，推荐写法：

```text
在同一 fixed S-curve 实物路径、同一速度/约束和相同延迟补偿设置下，
B_slosh 相比 B0 降低了 SPMPC 内部 slosh observer 的 /spmpc/slosh_height peak/p95/RMS，
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

---

## 15. 每个 run 现场记录模板

```text
run_name:
date/time:
operator:
method: B0 / B_slosh / B_ours
run_index:
run_type: shadow / low_speed_smoke / formal_N3

path_mode: fixed S-curve
path_generation: regenerate_from_current_start / replay_saved_path
path_file:
path_id:
start_marker:
start_pose_map:
goal_x/y/yaw:
S_curve_params: ratio=0.18,min=0.25,max=1.20,side=left,smooth=3,spacing=0.05

container:
liquid_level:
payload_fixed: yes/no
RGB_recorded: yes/no
RGB_calibration:

map_vref_runtime_v_ref_enable: false
map_vref_profile_enable: false
map_vref_status:

delay_phase_mode:
linear_delay_sec:
angular_delay_sec:
closed_loop_enabled_frac:

v_ref:
shared_linear_accel_max:
shared_angular_rate_max:
shared_angular_accel_max:
timeout_sec: 60

bag_file:
/spmpc/slosh_height samples:
result: success / timeout / e-stop / planner_fail / invalid
first_goal_time_sec:
tracking notes:
obvious off-track: yes/no
obvious spin: yes/no
manual intervention: yes/no
notes:
```

---

## 16. 最小后处理表格

每个 run 一行：

| run | method | success | time_s | tracking_rms_m | slosh_peak_mm | slosh_p95_mm | slosh_rms_mm | slosh_auc_mm_s | cmd_v_mean | cmd_w_p95 | closed_frac | note |
|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | B0 |  |  |  |  |  |  |  |  |  |  |  |
| 1 | B_slosh |  |  |  |  |  |  |  |  |  |  |  |
| 1 | B_ours |  |  |  |  |  |  |  |  |  |  |  |

聚合表：

| method | N | success | time mean±std | tracking RMS mean±std | slosh peak mean±std | slosh p95 mean±std | slosh RMS mean±std | slosh AUC mean±std |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| B0 |  |  |  |  |  |  |  |  |
| B_slosh |  |  |  |  |  |  |  |  |
| B_ours |  |  |  |  |  |  |  |  |

核心比较句：

```text
B_slosh vs B0: internal_slosh_p95 降低 xx%，internal_slosh_peak 降低 xx%，说明 slosh-aware soft cost 有效。
B_ours vs B_slosh: internal_slosh 指标维持/进一步降低 xx%，同时 cmd_w_p95、tracking 或终端停车表现更优/不变。
B_ours vs B0: 汇总说明 ours 相对无 slosh baseline 的总体收益。
```
