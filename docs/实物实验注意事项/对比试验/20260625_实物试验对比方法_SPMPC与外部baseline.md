# 20260625 实物试验对比方法：SPMPC 与外部 baseline

> 本文记录实物试验阶段的对比方法设计。上位设计参考：`docs/实物实验注意事项/对比试验/20260605_SPMPC论文对比实验设计建议.md`。
> 受控实物 SOP 与安全细节参考：`docs/实物实验注意事项/对比试验/20260603_SPMPC连续MPCC实物对比实验SOP.md`、`docs/实物实验注意事项/对比试验/20260613_SPMPC实物受控对比试验方案.md`。
>
> 核心原则：**一组实验只回答一个问题**；外部 baseline 不接入液面信息，液面只作为统一评价指标。

---

## 0. 当前结论口径

实物对比不是把所有方法混成一张大表，而是分成证据链：

```text
1. 先证明液面观测/monitor 与真实 RGB 液面响应有对应关系。
2. 再用 SPMPC 内部消融证明 slosh-aware 项本身有贡献。
3. 最后用 LT-DWA / DWA / TEB / mpc_local_planner 等外部 baseline 证明不是自研框架内部自嗨。
```

当前 official LT-DWA 的定位：

```text
LT-DWA official wrapper 是外部传统局部规划 baseline。
它不使用液面信息，不根据 /liquid/* 或 /slosh/* 改轨迹、改速度、改代价。
液面只作为实验后评价指标，用来和 SPMPC 的 slosh-aware 变体比较。
```

因此，若 LT-DWA 能完成路径但液面高度或角速度变化更大，这正是有效对比结果，而不是要求 LT-DWA 也做液面闭环。

### 0.1 与 Map-vref / 地形经验地图的阶段关系

`Map-vref` 是另一条实物证据链，不能和普通 SPMPC / external baseline 对比混成同一个阶段：

```text
R0：实物探索采集 / 数据链路验证
  - 使用 B_ours 或保守 B_ours_hard_1mm
  - map_vref/profile_enable=false
  - map_vref/runtime_v_ref_enable=false
  - 只采集 RGB/IMU/odom/cmd/path/progress，不验证 Map-vref 效果

R1：离线构图、校验、冻结 profile
  - 输出 frozen profile CSV、profile_hash、path_hash、map_id、path_id、payload_id、freeze note

R2：frozen profile blind test
  - B_ours
  - B_ours_uniform_slow
  - B_ours_map_vref（只读 frozen profile，禁止用 test RGB 在线调参）
```

R0 数据不能同时作为构图集和盲测集；正式液面真值仍以 `RGB bag/video -> offline max-LCR` 为准。

### 0.2 当前仿真前置状态（2026-06-27）

隔离仿真 fixed-path S 曲线外部对比已经完成 `N=9` formal fresh-sim：

```text
spmpc_B_ours_hard_1mm / dwa / teb / mpc_local_planner_tuned / lt_dwa_official
45/45 strict valid，45/45 到点，未触发 60s FAIL 或 freshness violation。
```

当前 SPMPC hard-cap 推荐值为：

```text
slosh_height_max = 0.001 m  # 1.0 mm
```

`0.85 mm` 可作为更激进补充，但 acados transient failure 更多；实物阶段若使用 hard-cap 方法，metadata 必须记录 cap 值，且不要把原始内部消融的 `8 mm` 口径与当前推荐 `1 mm` 口径混用。

---

## 1. 方法矩阵

### 1.1 主线：SPMPC 内部消融

| 方法 | 类型 | 是否使用 slosh state/cost | 是否使用 smooth/control regularization | 实物作用 |
|---|---|---:|---:|---|
| `B0` | SPMPC internal baseline | 否 | 否 | 基础 tracking 锚点；证明连续 MPCC 本身能完成任务 |
| `B_smooth` | SPMPC internal ablation | 否 | 是 | 回答“只靠平滑控制能否降晃”；当前实物中不稳定时只作诊断 |
| `B_slosh` | SPMPC internal ablation | 是 | 否/弱 | 回答 slosh-aware 模型/代价本体是否有效 |
| `B_ours` | SPMPC final soft | 是 | 是 | R0 首批推荐主控；Map-vref 关闭时作为无地图 baseline |
| `B_slosh_hard` | SPMPC hard-cap increment | 是 + hard cap | 否/弱 | hard cap 对 slosh-only 的增量；需记录 cap 值 |
| `B_ours_hard_1mm` | SPMPC final hard-cap | 是 + `slosh_height_max=0.001 m` | 是 | 当前仿真推荐 hard-cap 行；实物采用前先 N=1 smoke |
| `B_ours_map_vref` | SPMPC + frozen profile | 是；profile 只调 `v_ref` | 是 | 只用于 R2 blind test；R0 不启用，不用 test RGB 在线调参 |

### 1.2 外部 baseline

| 方法 | 类型 | 是否利用液面 | 实物对比定位 | 备注 |
|---|---|---:|---|---|
| official `LT-DWA wrapper` | 外部速度采样/局部规划 baseline | 否 | 传统局部规划器对比项 | 使用 wrapper final/guarded 输出驱动；raw official 输出只作观察 |
| `DWA` | 外部传统 baseline | 否 | 速度采样 baseline | 必须先调到能稳定完成任务，再冻结参数 |
| `TEB` | 外部传统 baseline | 否 | 图优化/轨迹优化 baseline | 必须先调到能稳定完成任务，再冻结参数 |
| `mpc_local_planner` | 外部优化式 baseline | 否 | 权威 optimization baseline | 跑稳后进正式表；否则只作为 diagnostic |

外部 baseline 的禁止项：

```text
不订阅 /liquid/* 或 /slosh/* 作为控制反馈。
不加入 slosh dynamics、eta/eta_dot cost、液面高度 cost。
不根据某一次 run 的液面结果单独改参数。
不把魔改后的 slosh-aware TEB/DWA/LT-DWA 当作标准外部 baseline。
```

---

## 2. 公平性控制

每个有效实物 run 必须尽量满足：

```text
同一 Scout 实物平台
同一容器、液位、安装姿态和载荷固定方式
同一地面起点标记、同一目标点、同一 fixed path 生成规则
同一地图、定位、TF、costmap/障碍口径
同一 RGB 相机位置、曝光/增益/白平衡、ROI/HSV/calibration
同一 bag 录制 topic 口径
组间回到同一起点，等待液体静稳后再开始下一轮
失败 run 保留并进入统计，不删除失败样本
```

### 2.1 参数对齐口径

正式实物 external baseline 对比前，必须先冻结一套 common-limit 或明确标注 tuned-limit。

推荐实物首轮 common-limit 以已验证的实物安全锚点为上限：

| 参数 | 推荐实物首轮口径 | 说明 |
|---|---:|---|
| `v_ref / max_v` | `0.65 m/s` | 来自当前实物 P2 S-curve 的 B0 成功锚点；若现场风险高，先降到 `0.50 m/s` smoke |
| `max_w / shared_angular_rate_max` | `1.2 rad/s` | 与仿真 common-limit 和实物安全层一致 |
| `max_acc / acc_lim_x` | `0.6 m/s²` | 外部 planner 与 SPMPC 均按同口径约束或记录实际命令变化率 |
| `alpha / angular accel` | 必须统一或显式标注 | 若 SPMPC 使用输出安全层 `shared_angular_accel_max=3.0`，外部方法也应经过同一安全 envelope；否则只能写 tuned-limit，不写 pure common-limit |
| goal tolerance | `0.20 m` | SPMPC terminal/goal tolerance、外部 planner xy tolerance、离线 success 阈值统一 |
| timeout | `60 s` | 60s 未到点或未达到 fixed-path terminal 判 FAIL，并安全停止 |

注意：仿真 common-limit 常用 `0.8 / 1.2 / 0.6 / 1.2`，但实物不能机械照搬仿真速度。实物正式表应以现场安全和已验证闭环能力冻结一套参数；一旦冻结，不允许按 run 单独调参。

### 2.2 SPMPC 输出安全层与外部 baseline

如果 SPMPC 使用：

```text
shared_angular_limit_enable=true
shared_angular_rate_max=1.2
shared_angular_accel_max=3.0
shared_angular_accel_max_dt=0.2
```

则外部 baseline 有两种合法写法：

```text
A. common safety envelope：LT-DWA/DWA/TEB/mpc_local_planner 的最终 /cmd_vel 也经过同一输出安全 envelope。
B. tuned/native output：外部方法使用各自冻结后的原生输出；报告中明确标注不是 pure common-limit，并报告实际 cmd |dω/dt| p95/max。
```

不要把 A 和 B 混在同一张“公平 common-limit”表里。

---

## 3. 实验顺序

### 3.1 每个方法先 N=1 smoke，再进入 N=3

最低流程：

```text
1. SPMPC B0 N=1 smoke：确认路径、定位、tracking、安全层和 RGB 链路正常。
2. SPMPC B_slosh / B_ours N=1 smoke：确认 slosh-aware 方法实物闭环安全。
3. LT-DWA official wrapper N=1 shadow：publish_cmd_vel=false，只看 raw/final/diagnostics 是否正常。
4. LT-DWA official wrapper N=1 bounded closed-loop：现场急停就位后才允许 publish_cmd_vel=true。
5. DWA/TEB/mpc_local_planner 同样先 N=1 smoke，确认能完成且不危险。
6. 所有候选方法 N=1 安全且数据完整后，才做 N=3 或 N=5 正式交错顺序。
```

#### 2026-06-25 `mpc_local_planner` 隔离仿真 smoke 记录

在迁移到实物前，`mpc_local_planner` 已按隔离仿真 SOP 做过一次 current-sim diagnostic smoke。该结果只证明“接入链路能跑、能到终点”，**不能直接作为 formal fixed-path 对比结果**。

运行条件：

```text
仿真入口: /data/a/scout_sim_replacement
MAP_FILE: /data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
baseline attach: /data/a/scout_sim_replacement/scripts/launch_proxy_baseline_localized_attach.sh
BASELINE: mpc_local_planner
PATH_TEMPLATE: s_curve
PATH_START_HEADING: current
GOAL: x=5.0, y=0.0, yaw=0.0
cmd_vel_topic: /cmd_vel
runner: baseline_local_planner_runner -> mpc_local_planner/MpcLocalPlannerROS
```

观测结果：

```text
/map、/scan_front、/odom、TF map->base_link 正常。
/baseline/mpc_local_planner/status = TRACKING。
/baseline/mpc_local_planner/global_plan 正常发布。
/cmd_vel publisher = /baseline_local_planner_runner。
终点附近 map pose ≈ (5.00, 0.095)，final error ≈ 0.095 m。
```

运行轨迹与全局参考层对比：

![mpc_local_planner executed trajectory vs global reference](assets/20260625_mpc_local_planner_executed_vs_reference.png)

本次图中黑色虚线是 `/scout/global_path_fixed`，紫色实线是 Cartographer `/trajectory_node_list` 中对应本轮的实际运行轨迹。近邻参考误差约为：

| 指标 | 值 |
|---|---:|
| mean tracking error | `0.140 m` |
| RMS tracking error | `0.203 m` |
| max tracking error | `0.543 m` |
| final error | `0.095 m` |

解释：

```text
mpc_local_planner 当前配置能到终点，但更像 navigation-style endpoint reaching。
它接收的 global_plan 与 /scout/global_path_fixed 基本一致，问题不是参考路径没有送进去；
实际闭环会明显切弯，尚未达到 strict fixed-path tracking baseline 要求。
```

因此进入实物前的口径更新为：

```text
mpc_local_planner 原始配置：曾通过隔离仿真 diagnostic smoke 的“能跑/能到”门槛，
但 fixed-path tracking 不足，只能作为历史诊断记录。

mpc_local_planner_tuned：已在 2026-06-27 外部 baseline fixed-path N=9 formal fresh-sim 中
9/9 strict valid、9/9 到点；若迁移到实物，必须显式使用 tuned fixed-path 配置，
并仍需先做 N=1 smoke，确认现场 tracking、安全和 RGB/odom/cmd/path 数据完整后再进正式表。
```

LT-DWA 进入实物 closed-loop 的前置门槛：

```text
1. 仿真 fresh visible sim 已通过同类 fixed-path 60s gate。
2. 实物端先 shadow，不发布 /cmd_vel。
3. /baseline/lt_dwa/raw_cmd_vel、/baseline/lt_dwa/shadow_cmd_vel、/baseline/lt_dwa/diagnostics 正常。
4. tracking guard 介入率、raw/final 差异和 planner latency 已记录。
5. 现场确认 /cmd_vel 没有旧 publisher，急停人员就位。
```

### 3.2 正式 N=3 交错顺序

不要按一个方法连续跑完三次。推荐交错，降低电量、光照、地面、液体初始状态和操作者因素影响：

```text
Round 1: B0 -> B_ours -> LT-DWA -> DWA -> TEB
Round 2: TEB -> DWA -> LT-DWA -> B_ours -> B0
Round 3: LT-DWA -> B0 -> TEB -> B_ours -> DWA
```

若外部 baseline 只先做 LT-DWA，则可用：

```text
Round 1: B0 -> B_ours -> LT-DWA
Round 2: LT-DWA -> B_ours -> B0
Round 3: B_ours -> B0 -> LT-DWA
```

每个 run 之间必须：

```text
停 planner -> 发 /cmd_vel zero -> 停 bag -> 回到同一起点 -> 等液体静稳 60~90s -> 确认 /cmd_vel 无旧 publisher -> 下一轮
```

### 3.3 R0 / Map-vref 实物采集入口（半手动安全流程）

R0 不是 full automation，建议保持多终端半手动流程：先录包，再启动 planner；现场安全人员随时可停。当前 repo 中可用入口如下：

| 环节 | 当前入口 | 状态 / 注意 |
|---|---|---|
| 传感器/定位栈 | `src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh` | 启动 base、LiDAR、Cartographer localization、IMU、RealSense；只停止自己追踪 PID |
| 在线 RGB 观察 | `roslaunch realsense_liquid_measurement online_liquid_monitor_combined.launch ...` | 源码位于 `src/scout_apps/sensors/realsense_liquid_measurement`；`/liquid/height` 只作现场观察 |
| fixed path 生成 | `rosrun scout_local_planner template_fixed_path_generator.py ...` | 生成 current-pose 起点固定路径；必须 RViz 人工确认不穿墙/不贴墙 |
| goal 发送 | `rosrun scout_local_planner send_fixed_goal.py ...` | 只发送目标；不替代人工安全确认 |
| 全量 RGB recorder | `src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh` | R0 推荐 recorder；只录包，不发控制；白名单需包含 Map-vref debug topics |
| 短安全 smoke recorder | `src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_mainline_ground_smoke.sh` | 只用于短 smoke；不能替代 RGB R0 数据 |
| SPMPC fixed path | `roslaunch spmpc_local_planner spmpc_fixed_path.launch ...` | R0-A 首批关闭 Map-vref：`profile_enable=false`、`runtime_v_ref_enable=false` |
| offline RGB | `red_liquid_infer_from_bag.py` / `export_liquid_variation_from_bags.py` | 正式液面真值来自离线 max-LCR，不来自在线 `/liquid/height` |

R0-A 首批推荐：

```bash
rosparam set /spmpc_local_planner/map_vref/runtime_v_ref_enable false
rosparam set /spmpc_local_planner/map_vref/profile_enable false

roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_ours \
  solver_backend:=continuous_mpcc_acados \
  reference_path_topic:=/scout/global_path_fixed \
  cmd_vel_topic:=/cmd_vel \
  costmap_topic:=/map \
  reference_target_frame:=map \
  v_ref:=0.25 \
  alpha_max:=1.2
```

每个 R0 trial 的 metadata 至少记录 `map_vref_profile_enable`、`map_vref_runtime_override_enable`、`planner_variant`、`v_ref`、`path_id/path_file`、`payload_id`、`camera calibration` 和 bag 路径。

---

## 4. 实物安全红线

启动 closed-loop 前必须满足：

```text
现场至少一人专门盯车，手边有硬件 E-stop / 遥控急停。
/cmd_vel 没有旧 planner publisher。
RGB 在线液面监控已启动并 zero locked。
bag recorder 已经开始。
RViz/path/goal/定位都正常。
机器人周围无人员、障碍物和不可控风险。
```

立即停机条件：

```text
明显离开固定路径或朝人/障碍物运动。
原地转圈或小 linear.x + 大 angular.z 持续约 2s。
60s 未到点或 fixed-path progress 长时间不增长。
定位、TF、/odom、/scan_front、/map 明显异常。
容器固定异常、液体溅出风险、现场人员主观认为不安全。
LT-DWA bridge/planner diagnostics 长时间异常或 final command 不合理。
SPMPC 出现 TRACKING_UNSAFE_PROJECTION / SPIN_FAIL / ACADOS_SOLVE_FAILED_* 等关键错误。
```

停机顺序：

```text
1. 优先硬件 E-stop / 遥控急停。
2. Ctrl-C 停当前 planner roslaunch。
3. 连续发 /cmd_vel zero。
4. Ctrl-C 停 rosbag。
5. 保存 bag、log、path JSON、现场记录。
```

zero 命令：

```bash
for i in 1 2 3 4 5; do
  rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  sleep 0.1
done
```

---

## 5. 录包 topic 口径

每个 run 单独一个 bag。通用 topic：

```text
/tf /tf_static
/odom /map /scan_front
/imu/data /container_imu
/camera/color/image_raw /camera/color/camera_info
/scout/goal /scout/global_path_fixed
/cmd_vel
/liquid/height /liquid/height_lcr /liquid/height_median /liquid/debug_image
/slosh/height /slosh/state /slosh/debug
```

SPMPC 额外 topic：

```text
/spmpc/status /spmpc/solver_backend
/spmpc/debug/runtime_bounds
/spmpc/debug/generated_bounds
/spmpc/debug/progress_s
/spmpc/debug/v_ref_current
/spmpc/debug/map_vref_status
/spmpc/debug/projector
/spmpc/debug/stage0_reference
/spmpc/debug/local_traj_head
/spmpc/debug/first_shot_summary
/spmpc/debug/warm_start_head
/spmpc/debug/cmd_vel_output
/spmpc/debug/cmd_vel_output_status
/spmpc/start_lock/mode
/spmpc/terminal/mode
/spmpc/terminal/debug
/spmpc/slosh_height
/spmpc/slosh_horizon_summary
```

Map-vref 状态判读：

```text
R0 / 普通 B_ours：/spmpc/debug/map_vref_status = VARIANT_FALLBACK
R2 B_ours_map_vref：/spmpc/debug/map_vref_status = PROFILE_LOOKUP
runtime smoke：/spmpc/debug/map_vref_status = RUNTIME_OVERRIDE
```

LT-DWA 额外 topic：

```text
/baseline/lt_dwa/raw_cmd_vel        # official raw 输出，只作观察
/baseline/lt_dwa/shadow_cmd_vel     # wrapper final/guarded 输出
/baseline/lt_dwa/diagnostics        # planner latency、guard、raw/final、状态
/baseline/lt_dwa/worker_result      # in_process/worker fallback 结构化状态
```

LT-DWA 记录时必须在现场记录里写清：

```text
planner_execution_mode: in_process / worker_once
raw_cmd_topic:
shadow_cmd_topic:
publish_cmd_vel: true/false
guard_applied_rate:
raw/final 是否明显不同:
是否使用同一输出 safety envelope:
```

---

## 6. 主评价指标

### 6.1 成功与跟踪

| 指标 | 说明 |
|---|---|
| success rate | 60s 内到点或 fixed-path terminal 成功 |
| failure / no-go count | 超时、急停、离轨、planner fail 都记录 |
| duration / first-goal time | 完成时间，不能只看液面低不看速度 |
| tracking RMS / p95 / max | 实际轨迹 vs fixed global path |
| final error / stop distance | 终点误差、停车距离 |
| stable stop bool | 终端是否稳定停住 |

### 6.2 液面指标

主真值：**离线 RGB max-LCR 液面高度**。

| 指标 | 说明 |
|---|---|
| RGB max-LCR peak / p95 / RMS | 论文主评价液面真值 |
| RGB core-window p95 / RMS | 去掉起步和终端后的主体段，更能反映行进抑晃 |
| RGB AUC / settling time | 可选，用于分析持续晃动 |
| `/slosh/height` peak / p95 | standalone monitor/evaluation proxy，不能替代 RGB 真值 |
| `/spmpc/slosh_height` | SPMPC 内部模型 debug，只作解释，不当真实液面 |

窗口建议：

```text
full-window：全程风险，包含起步和终端。
post-start/core：t > motion_start + 2s，且去掉 terminal 停车段。
terminal：终端停车段单独报告 stop distance、final speed、terminal slosh。
```

不要只用 raw peak 下结论。慢的方法天然晃得少，因此主结论应同时报告：

```text
success / duration / tracking / liquid height / command smoothness
```

并优先写成：

```text
同等完成率、同等 tracking 或同等速度附近，B_ours 的 RGB 液面风险更低。
```

### 6.3 控制平滑与底盘响应

| 指标 | 说明 |
|---|---|
| cmd_v mean/p95/max | 指令线速度 |
| cmd_omega p95/max | 指令角速度 |
| cmd abs(dv/dt) p95/max | 指令线加速度变化 |
| cmd abs(dω/dt) p95/max | 指令角加速度/打舵尖峰 |
| odom ax/ay p95 | 实际底盘响应 |
| LT-DWA guard_applied rate | wrapper 对 official raw 的改写比例 |
| SPMPC desired vs limited cmd | 输出安全层是否长期替 planner 修正 |

---

## 7. 无效 run、失败 run 与重跑规则

### 7.1 作废并重跑，但保留记录

以下属于数据链路或现场条件无效，不进入正式统计，但 bag/log 仍保留：

```text
起步前液体未静稳。
起点明显偏离地面标记。
RGB calibration/ROI/HSV 错误，离线液面识别失败。
bag 缺少 /camera/color/image_raw 或 /camera/color/camera_info。
/odom、TF、/map、/scan_front 明显异常。
录包损坏或关键 topic 缺失。
现场发生非算法因素干扰，如人进入路线、容器松动。
```

### 7.2 进入失败统计

以下属于方法失败，必须进入 success/failure 统计：

```text
60s 未到点。
planner 无可用输出或长期 stuck。
明显离轨、转圈、朝障碍物运动，需要急停。
SPMPC solver/status 失败。
LT-DWA bridge/planner 输出异常导致不能安全执行。
DWA/TEB/mpc_local_planner 无法稳定完成同一任务。
```

不能因为失败 run 液面数据不好看而删除样本。

---

## 8. 现场记录模板

每个 run 手动记录：

```text
run_name:
date/time:
operator:
method: B0 / B_slosh / B_ours / LT-DWA / DWA / TEB / mpc_local_planner
run_type: shadow / N=1 smoke / formal N=3
path_id:
path_file:
goal_x/y/yaw:
start_marker:
container/liquid_level:
RGB calibration:
bag_file:

common-limit or tuned-limit:
max_v / v_ref:
max_w:
max_acc:
angular_accel / safety envelope:
goal_tolerance:
timeout_sec: 60

publish_cmd_vel: true/false
/cmd_vel old publisher cleared: yes/no
E-stop operator ready: yes/no
online RGB zero locked: yes/no

result: success / timeout / e-stop / planner_fail / invalid
first_goal_time_sec:
obvious off-track: yes/no
obvious spin: yes/no
manual intervention: yes/no
notes:
```

LT-DWA 额外记录：

```text
planner_execution_mode:
raw_cmd observed: yes/no
final/guarded cmd observed: yes/no
guard_applied_rate:
planner_latency_ms mean/p95:
raw/final difference notes:
```

---

## 9. 推荐表格组织

实物论文/报告不要一张表塞完。推荐：

```text
Table R1: slosh monitor validation
  RGB max-LCR vs /slosh/height or /spmpc/slosh_height correlation / RMSE / lag

Table R2: SPMPC internal real-robot ablation
  B0 / B_slosh / B_ours；B_smooth 若实物不稳定则仅诊断

Table R3: external real-robot baseline comparison
  B_ours vs LT-DWA official wrapper vs DWA vs TEB vs mpc_local_planner(if stable)

Table R4: safety/command smoothness diagnostics
  cmd |dω/dt|, limiter ratio, guard_applied rate, failure modes

Table R5: Map-vref frozen-profile blind test（仅 R2）
  B_ours / B_ours_uniform_slow / B_ours_map_vref；报告 profile_hash、path_hash、map_vref_status、v_ref_current/cmd/odom 响应
```

其中 LT-DWA 行的标准描述：

```text
Official LT-DWA wrapper baseline. It does not use liquid-height or slosh-state feedback. The raw official command is recorded for diagnosis; the guarded final command is used for bounded real-robot actuation when authorized. Liquid metrics are computed only offline for evaluation.
```

---

## 10. 最小可执行版本

若现场时间有限，只做最小实物对比：

```text
1. B0 N=1 smoke：确认实物路径、定位、RGB、录包和安全层正常。
2. B_ours 或 B_slosh N=1 smoke：确认 slosh-aware 方法能安全闭环。
3. LT-DWA official wrapper shadow N=1：不发 /cmd_vel，只确认 raw/final/diagnostics。
4. LT-DWA official wrapper bounded closed-loop N=1：现场确认安全后才发 /cmd_vel。
5. 若 1~4 都安全且数据完整，再做 B0 / B_ours / LT-DWA 的 N=3 交错顺序。
6. DWA/TEB/mpc_local_planner 另开批次加入，不和第一轮 LT-DWA 验证混在一起。
```

最小结论句模板：

```text
在同一实物 Scout、同一 P2 S-curve fixed path、同一 RGB 液面真值和冻结后的运动约束下，LT-DWA/DWA/TEB 等外部 baseline 不使用液面反馈，只作为传统局部规划对照；SPMPC 的 slosh-aware 变体若在相近完成率与 tracking 下获得更低 RGB 液面 p95/RMS，才能形成实物降晃证据。
```
