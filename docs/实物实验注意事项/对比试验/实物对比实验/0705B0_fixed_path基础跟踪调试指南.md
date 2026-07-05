# B0 fixed-path 基础跟踪调试指南

> 目的：在进入 `B0 / B_slosh / B_ours` 内部晃动高度对比前，先把 **B0 fixed-path 基础跟踪**调到稳定可复现。
>
> 前置结论：如果 B0 自己都不能稳定跟踪同一条 fixed S-curve，那么后续 `B_slosh` / `B_ours` 的 slosh 指标没有可解释性；必须先证明 baseline tracking 链路干净。

相关后续文件：

```text
docs/实物实验注意事项/对比试验/实物对比实验/B0_B_slosh_B_ours_SPMPC内部晃动高度实物验证指南.md
```

本文件是上面三方法对比指南的前置 gate：**先通过本文件，再进入三方法对比。**

---

## 1. 为什么要先调 B0

2026-07-05 对比中观察到：

```text
旧成功 B0：2026-07-02 / commit 4eef4e5
- GOAL_REACHED
- projection p95 ≈ 0.025~0.028 m
- delay_phase = fixed_closed_loop
- linear_delay = 0.15 s
- angular_delay = 0.22 s

当前失败 B0：2026-07-05 / commit 671fd74
- delay_phase_mode = off
- 约 15 s 触发 TRACKING_UNSAFE_PROJECTION
- projection distance 到约 0.57 m
- zero_due_to_tracking_safety = 1
```

关键判断：

```text
1. B0 delay off 已经失败，所以首要问题不是 slosh cost / hard constraint。
2. 当前失败更像 delay off + 速度/路径推进相位差导致切弯，而不是 TF/map/odom/path frame 错误。
3. 当前 bag 第一帧 cmd_v≈0.30 还混入 recorder 晚于 planner 启动的问题，不能直接当作 planner 真正第一拍。
4. 旧 B0 fixed_closed_loop 0.15/0.22 能成功，所以当前要先复现这个 clean B0 baseline。
```

因此当前顺序应改为：

```text
Step 1: B0 fixed-path 基础跟踪调通。
Step 2: 确认 B0 baseline 的 delay / speed / recorder 口径。
Step 3: 再做 B0 / B_slosh / B_ours 内部晃动高度对比。
```

不要在 B0 仍然 `TRACKING_UNSAFE_PROJECTION` 时开始比较 `B_slosh` / `B_ours`。

---

## 2. B0 基线通过标准

一个 B0 run 只有满足下面条件，才算可以作为后续对比的基础口径。

### 2.1 必须通过

```text
1. /spmpc/status 出现 GOAL_REACHED。
2. 不出现 TRACKING_UNSAFE_PROJECTION。
3. 不出现持续 zero_due_to_tracking_safety=1。
4. 无人工急停、无明显离轨、无原地转圈。
5. /scout/global_path_fixed、/odom、/tf、/cmd_vel、/spmpc/debug/projector、/spmpc/debug/stage0_reference 可用于后处理。
```

### 2.2 推荐量化阈值

以 2026-07-02 旧成功 B0 为参考，建议先用下面阈值做 gate：

| 指标 | 通过建议 |
|---|---:|
| projection distance p95 | `<= 0.05 m` |
| projection distance max | `<= 0.15 m` |
| stage0 contour error p95 | `<= 0.05 m` |
| stage0 yaw error p95 | `<= 0.15 rad` |
| zero_due_to_tracking_safety fraction | `0` |
| first goal time | 可稳定到点即可，先不强行卡时间 |

说明：旧成功包 p95 约 `0.025~0.028 m`，max 最高约 `0.10 m`。当前先把 B0 拉回同一量级，不要追求一步到论文最终口径。

---

## 3. 现场前置检查

### 3.1 只允许一个 `/cmd_vel` 发布者

```bash
rostopic info /cmd_vel
```

期望：只有当前要测的 planner 或安全链路在发布，不要同时有旧 planner、move_base、teleop、shadow bridge 等抢 `/cmd_vel`。

### 3.2 TF / map / odom 基础检查

```bash
rostopic echo -n 1 /odom
rostopic echo -n 1 /map
rosrun tf tf_echo map base_link
rostopic echo -n 1 /scan_front
```

注意：`/odom` 的 pose 是 `odom` frame，`/scout/global_path_fixed` 是 `map` frame。不要直接把 `/odom.pose` 和 map 路径相减判断路径是否错位；优先看 planner 发布的：

```text
/spmpc/debug/stage0_reference
/spmpc/debug/projector
```

### 3.3 起点状态

每个 B0 调试 run 前确认：

```text
1. 车体回到实物起点标记附近；
2. heading 尽量对齐路径起点方向；
3. 车完全静止；
4. 液体静稳 60~90 s；
5. 急停/遥控接管就位。
```

如果要判断“真实起步 cmd_v”，必须保证 bag 在 planner 启动前已经真正开始 recording。

---

## 4. recorder 时序注意事项

当前 one-click 脚本逻辑是：

```text
启动 fixed-path generator -> 启动 recorder -> 发送 fixed goal -> 等待 /scout/global_path_fixed -> 启动 planner
```

但 recorder 脚本在正式 `rosbag record` 前会做 topic / rosparam / node snapshot。2026-07-05 包里观察到 planner initialized 到 bag 真正 `Recording to ...` 之间存在约 6 s 差异，导致 bag 第一帧已经不是 planner 第一拍。

为了 B0 基础调试，建议使用下面的 passive recorder 口径，减少“漏掉起步段”的干扰：

```text
RECORD_TOPIC_INFO=false
RECORDER_STARTUP_SEC=8
```

这两个变量不改变控制行为，只影响录包前等待/sidecar 完整度。调 B0 起步时优先保证 bag 捕获真实启动瞬态；正式统计时再按需要恢复更完整 topic-info sidecar。

---

## 5. 最小 B0 调试矩阵

本阶段只跑 B0，不跑 `B_slosh` / `B_ours`。

结合实物说明书，Scout Mini 的底盘能力明显高于当前 one-click 的保守角向限制：说明书给出整车最高速度 `1.5 m/s`，指令控制帧线速度值域 `[-3.0, 3.0] m/s`、角速度值域 `[-2.523, 2.523] rad/s`，控制帧周期 `20 ms`、超时 `500 ms`。因此当前 `omega_max=1.2` 仍是保守值；真正可能过紧的是角加速度/转向修正权限。

调 B0 时建议放开的不是 tracking safety，也不是直接把速度拉高，而是**先恢复已在 20260613 实物 B0 中验证过的角向修正权限**：

```text
alpha_max: 1.2 -> 8.0
shared_angular_accel_max: 1.2 -> 3.0
shared_angular_rate_max: 保持 1.2
v_ref: 先保持 0.20
```

每次只改一个主动控制口径。推荐主线三步：

```text
Run A: B0 + 旧成功 delay 口径 fixed_closed_loop 0.15 / 0.22
Run B: B0 + delay off + 当前保守角向限制
Run C: B0 + delay off + 放开角向限制到 20260613 B0 实物成功口径
```

如果 Run C 仍失败，再把 `V_REF=0.10` 作为额外低速保底实验；低速不是主线第一选择。

---

## 6. Run A：复现旧成功 B0 口径

目的：先证明当前代码、当前 one-click 路径和现场状态下，B0 能回到旧成功包的 tracking 水平。

主动变量：

```text
delay_phase_mode = fixed_closed_loop
linear_delay_sec = 0.15
angular_delay_sec = 0.22
```

命令模板：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

ALG=B0 \
RUN_LABEL=B0_fixed_150_220_baseline_run01 \
CMD_TOPIC=/cmd_vel \
V_REF=0.20 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

期望：

```text
1. GOAL_REACHED。
2. projection p95 回到 0.03~0.05 m 量级。
3. stage0 contour p95 回到 0.03~0.05 m 量级。
4. zero_due_to_tracking_safety = 0。
5. /spmpc/debug/solver_input_state 显示 delay compensation applied。
```

判读：

```text
Run A 成功：说明当前代码/现场/路径可实现 clean B0 tracking，继续 Run B。
Run A 失败：先不要做任何 slosh 对比；转查起点、TF、/cmd_vel 发布者、路径生成、底盘状态和 recorder/planner 时序。
```

---

## 7. Run B：只改 delay off

目的：验证 2026-07-05 当前失败是否主要由 `delay_phase_mode=off` 导致。

相对 Run A 只改一件事：

```text
fixed_closed_loop 0.15/0.22 -> off
```

命令模板：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

ALG=B0 \
RUN_LABEL=B0_delay_off_clean_start_run01 \
CMD_TOPIC=/cmd_vel \
V_REF=0.20 \
DELAY_PHASE_MODE=off \
DELAY_PHASE_LINEAR_DELAY_SEC=-1.0 \
DELAY_PHASE_ANGULAR_DELAY_SEC=-1.0 \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

期望与判读：

```text
Run B 也成功：
  当前 2026-07-05 失败更可能来自起步/路径/记录时序/现场状态，不是 delay off 本身。
  后续三方法对比可以考虑 delay off，但仍要用 clean-start 口径。

Run A 成功、Run B 失败：
  基本确认当前速度/路径下需要 delay compensation；delay off 不是 clean baseline。
  后续三方法对比不要直接使用 delay off，除非先降低速度或另行证明可稳定跟踪。
```

---

## 8. Run C：只放开角向限制（条件触发）

触发条件：

```text
Run A 成功，但 Run B delay off 失败。
```

目的：判断 Run B 失败是否主要因为当前角加速度/转向修正权限过保守，而不是 delay off 绝对不可用。

相对 Run B 只改一类主动变量：

```text
角向限制口径：当前保守值 -> 20260613 B0 实物成功口径
alpha_max: 1.2 -> 8.0
shared_angular_accel_max: 1.2 -> 3.0
shared_angular_rate_max: 保持 1.2
V_REF: 保持 0.20
```

命令模板：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

ALG=B0 \
RUN_LABEL=B0_delay_off_ang_open_run01 \
CMD_TOPIC=/cmd_vel \
V_REF=0.20 \
ALPHA_MAX=8.0 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=3.0 \
DELAY_PHASE_MODE=off \
DELAY_PHASE_LINEAR_DELAY_SEC=-1.0 \
DELAY_PHASE_ANGULAR_DELAY_SEC=-1.0 \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

判读：

```text
Run C 成功：
  delay off 不是绝对不可用；Run B 的失败主要来自当前角向限制过紧，弯道纠偏能力不足。
  后续三方法对比可以考虑沿用这个角向限制口径，但必须让 B0/B_slosh/B_ours 全部一致。

Run C 仍失败：
  当前 fixed-path 链路下，delay off 即使放开角向限制也不够稳。
  后续三方法对比应先采用 Run A 成功的 fixed_closed_loop 口径。
```

可选低速保底：若 Run C 仍失败但想确认是否纯速度问题，再额外跑 `B0 delay off + V_REF=0.10`；这个低速实验不作为主线三步之一。

---

## 9. 每个 B0 run 后立即检查

### 9.1 先看 status / safety

```bash
python3 src/scout_apps/control/spmpc_local_planner/scripts/summarize_spmpc_real_trial.py \
  /home/geist/slosh_bags/real/${DATE}_fixed_path_compare/B0/${RUN_LABEL}.bag
```

必须记录：

```text
first GOAL_REACHED / TRACKING_UNSAFE_PROJECTION 时间
zero_due_to_tracking_safety 是否出现
projection p50/p95/max/final
stage0 contour p50/p95/max/final
stage0 yaw p50/p95/max/final
cmd_v max / p95
odom v max / p95
/scout/global_path_fixed first pose / last pose / pose count / frame_id
```

### 9.2 检查 effective config

bag 内应有：

```text
/spmpc/controller_variant = B0
/spmpc/solver_backend = continuous_mpcc_acados
/spmpc/experiment_mode = fixed_path
/spmpc/debug/effective_config
```

注意：如果 `${RUN_LABEL}_rosparam.yaml` 与 planner log 或 bag 内 effective config 冲突，优先相信：

```text
1. planner.log 中 roslaunch PARAMETERS；
2. /spmpc/debug/effective_config；
3. /spmpc/controller_variant / solver_backend / experiment_mode；
4. 最后才是 recorder 启动前 dump 的 rosparam.yaml。
```

原因：recorder 可能在 planner 启动前 dump 到 stale rosparam。

### 9.3 检查路径起点

用 `/spmpc/debug/stage0_reference` 判断 robot 与 path 的 map-frame 对齐，不要直接用 `/odom.pose` 对 map 路径。

期望：

```text
clean-start run 的第一段：
- s0 接近 0；
- robot 到 path first / nearest pose 是厘米级或可解释的小偏差；
- robot_v 接近 0；
- cmd_v 从低值逐步爬升。
```

如果第一条记录已经 `s0≈0.8~1.0m`、`robot_v≈0.3m/s`，说明 bag 仍未捕获真正起步段；该 run 可以用于 tracking 成败判断，但不能用于“起步命令是否过激”的结论。

---

## 10. B0 baseline 决策表

| Run A fixed 0.15/0.22 | Run B off 当前角向限制 | Run C off 放开角向限制 | 后续策略 |
|---|---|---|---|
| 成功 | 成功 | 不需要 | 三方法对比可用 delay off，但必须沿用 clean-start / recorder 口径 |
| 成功 | 失败 | 成功 | delay off 可用，但当前角向限制过紧；三方法对比必须统一使用 Run C 的角向限制口径 |
| 成功 | 失败 | 失败 | delay off 暂不作为实物 baseline；三方法对比先用 fixed 0.15/0.22 |
| 失败 | 不跑 | 不跑 | 不进入三方法对比；先排查 TF/path/start/cmd publisher/底盘 |

### 10.1 2026-07-05 实测执行结果

本轮已经按上表完成三组 B0 gate run，bag 目录：

```text
/home/geist/slosh_bags/real/20260705_fixed_path_compare/B0/
```

| run | 主动口径 | 结果 | projection p95 / max | stage0 contour p95 / max | yaw p95 / max | 结论 |
|---|---|---|---:|---:|---:|---|
| `B0_fixed_150_220_baseline_run01` | `fixed_closed_loop 0.15 / 0.22`，当前保守角向限制 | `GOAL_REACHED @ 44.5s` | `0.0276 / 0.081 m` | `0.0257 / 0.0538 m` | `0.065 / 0.155 rad` | 最干净 baseline，复现旧成功口径 |
| `B0_delay_off_clean_start_run01` | `delay off`，当前保守角向限制 | `TRACKING_UNSAFE_PROJECTION @ 31.0s` | `0.595 / 0.607 m` | `0.596 / 0.607 m` | `0.947 / 0.949 rad` | clean-start 仍失败；不是录包没抓到起步 |
| `B0_delay_off_ang_open_run01` | `delay off`，角向限制放开到 20260613 成功口径 | `GOAL_REACHED @ 47.1s` | `0.089 / 0.127 m` | `0.090 / 0.127 m` | `0.217 / 0.263 rad` | delay off 可用，但需要更强角向纠偏权限 |

本轮关键结论：

```text
1. 当前代码、路径和实物平台本身没有坏：B0 + fixed_closed_loop 0.15/0.22 能稳定到点。
2. B0 + delay off + 当前默认角向限制会失败，并且 clean-start 下仍失败。
3. 放开角向限制后，B0 + delay off 可以到点，说明 Run B 失败主要来自角向纠偏权限过紧，而不是 delay off 绝对不可用。
4. 但 delay off + 放开角向限制的 tracking 误差仍明显大于 fixed_closed_loop 0.15/0.22；若目标是最干净 baseline，优先选 fixed_closed_loop 0.15/0.22。
```

因此本轮推荐 baseline 写为：

```text
B0_baseline_date: 2026-07-05
B0_baseline_bag: /home/geist/slosh_bags/real/20260705_fixed_path_compare/B0/B0_fixed_150_220_baseline_run01.bag
B0_baseline_delay_mode: fixed_closed_loop
B0_baseline_linear_delay_sec: 0.15
B0_baseline_angular_delay_sec: 0.22
B0_baseline_v_ref: 0.20
B0_projection_p95/max: 0.0276 / 0.081 m
B0_stage0_contour_p95/max: 0.0257 / 0.0538 m
B0_goal_reached_time: 44.5 s
B0_notes: 最干净 B0 fixed-path baseline；后续 B0/B_slosh/B_ours 对比优先使用这一 delay 口径。
```

备用口径：若后续坚持不使用 delay compensation，则必须统一使用 `B0_delay_off_ang_open_run01` 的角向限制口径：

```text
DELAY_PHASE_MODE=off
ALPHA_MAX=8.0
SHARED_ANGULAR_RATE_MAX=1.2
SHARED_ANGULAR_ACCEL_MAX=3.0
```

但该口径 tracking p95 约 `0.089 m`，不如 fixed_closed_loop baseline 干净。

---

## 11. 进入三方法对比的规则

只有当 B0 baseline 满足第 2 节 gate 后，才进入：

```text
docs/实物实验注意事项/对比试验/实物对比实验/B0_B_slosh_B_ours_SPMPC内部晃动高度实物验证指南.md
```

进入后要遵守：

```text
1. B0 / B_slosh / B_ours 必须使用同一个经 B0 gate 验证的 delay 口径。
2. 必须使用同一个 v_ref / shared constraints / fixed-path 生成口径。
3. 每个 run 都从静止起点开始，液体静稳后再跑。
4. 若任一方法出现 TRACKING_UNSAFE_PROJECTION，该 run 保留为失败样本，但不能拿 slosh 指标当 clean 方法收益。
5. 如果后续指南里的默认 delay off 与本文件 B0 gate 结果冲突，以本文件 gate 结果为准。
```

一句话原则：

```text
先让 B0 在实物 fixed path 上稳定、可解释、可复现；再讨论 slosh-aware 是否降低内部晃动。
```
