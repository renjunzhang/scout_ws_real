# 20260706 LT-DWA 实物 fixed-path 偏离诊断与救援方案

> 结论先行：LT-DWA 不是“启动失败”，而是“能出命令但 fixed-path 跟踪不成立”。必须救，但不能直接进 formal N=3。救援顺序是：先校验 frame/路径输入，再用低速、低最小前进速度、较大 lookahead 的 path-tracking guard 做短程 actuated rescue，短程通过后才做 60s smoke。

## 1. 已有记录

### 1.1 shadow 已经能启动

`LTDWA_fixed_0706_shadow01` 说明 LT-DWA wrapper、worker、local map generation 已经编译并能被 roslaunch 拉起。

shadow 日志里有两个需要注意的点：

```text
map-frame odom adapter TF lookup failed: "map" passed to lookupTransform argument target_frame does not exist.
call local map service failed
```

这两个不是最终判死刑，但说明启动早期 TF / local map 服务有过短暂不可用。后续救援时要把它们作为 gate 检查项。

### 1.2 actuated smoke 失败

失败 bag：

```text
/home/geist/slosh_bags/real/20260706_fixed_path_compare/lt_dwa_official/LTDWA_fixed_0706_smoke01.bag
```

关键现象：

```text
active cmd window: 约 47.53 s
/baseline/official_lt_dwa/status:
  OK: 1348
  STALE_INPUT: 79
  WAITING_FOR_INPUT: 22

/cmd_vel:
  v p50 / p95 / max = 0.180 / 0.314 / 0.352 m/s
  w p50 / p95 / max = 0.280 / 0.948 / 1.200 rad/s

/baseline/official_lt_dwa/worker_result:
  reason = official_core_ok_path_tracking_guard
  guard_applied = 1
  guard_reason = path_tracking_guard
```

注意：LT-DWA 的 tracking 误差不能直接用 `/odom` 去和 `/scout/global_path_fixed` 比，因为 `/odom` 和 fixed path 不是同一 frame。应使用 `/baseline/official_lt_dwa/odom_map` 与 `/scout/global_path_fixed` 比。

修正到 map frame 后，结果仍然失败：

```text
odom_map -> fixed path distance:
  p50 ≈ 0.41 m
  p95 ≈ 1.23 m
  max ≈ 1.33 m

path progress:
  first ≈ 0.000
  last  ≈ 0.128
  min   ≈ 0.000
  max   ≈ 0.390
```

判读：

```text
1. 不是没出命令；命令持续发布。
2. 不是 worker 崩溃；official core 返回 OK。
3. 但 path_tracking_guard 全程接管，并且角速度多次接近 1.2 rad/s 上限。
4. 约 47s 后最终只停在路径进度约 13%，中途最大到过约 39%，说明跟踪有回退/绕偏，不是稳定沿 S 曲线推进。
5. 因此不能进入 LT-DWA formal N=3。
```

## 2. 最可能的问题

按优先级排序：

```text
P0. frame / path / start pose 对齐虽然大体可用，但仍需在每次 rescue 前复核。
P1. 当前 path_tracking_guard 参数不适合实物 fixed-path：
    默认 path_tracking_min_v=0.18 太硬，姿态误差大时仍强制往前顶；
    默认 lookahead=0.75 可能导致 S 曲线局部跟踪时角速度饱和。
P2. LT-DWA official core 本身更像 goal/local obstacle planner，不是严格 path tracker；
    当前 wrapper 依赖 path_tracking_guard 把它硬改成 path follower。
P3. local map service 偶发失败会污染 worker 初期输出，需要排除连续失败。
P4. `/baseline/official_lt_dwa/local_plan` 当前更像 current->goal 的短显示，不足以证明它真的在沿完整 S 曲线走；要看 `worker_result`、`global_plan`、`odom_map` 与 path 的关系。
```

所以救援重点不是再跑一遍原参数，而是先把 guard 调成“保守纯跟踪器”：低速、允许低速转向、减少强制前进。

## 3. 救援原则

```text
1. 不删除任何 bag。
2. 不直接跑 formal N=3。
3. 每次 actuated rescue 先短时 15~25s，手在急停上。
4. 短时通过后，才允许 60s smoke。
5. smoke 通过后，才讨论 LT-DWA formal N=3。
6. 若 rescue 后仍不能稳定贴路径，就把 LT-DWA 记录为“外部 baseline 接入失败/不适合该 fixed-path 实物任务”，转 TEB。
```

## 4. Rescue 参数阶梯

### R0：首选救援参数

目的：先救“能沿路径走”，不追求最快。

```text
max_v                  = 0.20
min_v                  = 0.00
max_w                  = 1.20
max_acc                = 0.30
max_angular_acc        = 1.20
planner_rate_hz         = 10.0
path_resample_spacing  = 0.05
path_tracking_lookahead_m = 1.20
path_tracking_min_v       = 0.08
```

为什么：

```text
1. max_v 降到和 SPMPC V_REF=0.20 同级，公平且更安全。
2. max_acc 降到 0.30，减少底盘前冲。
3. path_tracking_min_v 从 0.18 降到 0.08，允许车在大角度修正时慢下来。
4. lookahead 从 0.75 增到 1.20，减少局部 S 曲线追点导致的角速度饱和。
5. path_resample_spacing=0.05，与 fixed path spacing 一致。
```

### R1：如果 R0 太慢或原地抖

```text
max_v                  = 0.22
max_acc                = 0.35
path_tracking_lookahead_m = 0.90
path_tracking_min_v       = 0.08
```

### R2：如果 R0 仍外切/偏离

```text
max_v                  = 0.15
max_acc                = 0.25
path_tracking_lookahead_m = 1.50
path_tracking_min_v       = 0.05
```

### R3：只做 shadow，不做实车

若怀疑 guard 本身把 official core 带坏，可以 shadow 比较：

```text
enable_path_tracking_guard = false
```

但不建议直接 actuated 关闭 guard。因为 official LT-DWA core 未必会严格跟 fixed S-curve。

## 5. 推荐执行流程

### Step A：先做静态 gate

实车放在起点，真实定位/TF 已启动后检查：

```bash
rostopic echo -n 1 /scout/global_path_fixed/header
rostopic echo -n 1 /baseline/official_lt_dwa/odom_map
rostopic hz /baseline/official_lt_dwa/odom_map
rostopic echo -n 1 /baseline/official_lt_dwa/status
rostopic echo -n 1 /baseline/official_lt_dwa/worker_result
```

通过条件：

```text
1. `/baseline/official_lt_dwa/odom_map` 连续发布。
2. fixed path header.frame_id 是 map 或能稳定转到 map。
3. 起点处 odom_map 到 fixed path 第一个点距离最好 < 0.15 m。
4. status 不长期 WAITING_FOR_INPUT / STALE_INPUT。
5. planner log 里不能连续出现 local map service failed。
```

### Step B：R0 shadow sanity

如果继续用当前 one-click 脚本，注意它目前只透传 `MAX_V/MAX_W/MAX_ACC/MAX_ANGULAR_ACC`，不透传 `path_tracking_lookahead_m/path_tracking_min_v/path_resample_spacing/min_v`。因此真正救援阶段建议临时用 roslaunch 直接跑 LT-DWA，不改代码。

启动参数示例：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
export SCOUT_WS_ROOT=/home/geist/scout_ws
export ROS_PACKAGE_PATH=/home/geist/scout_ws/tools/lt_dwa/local_planner_runtime:${ROS_PACKAGE_PATH}

roslaunch lt_dwa_official_wrapper scout_sop_shadow_integration.launch \
  start_local_map_service:=true \
  enable_actuated_output:=false \
  publish_cmd_vel:=false \
  input_odom_topic:=/odom \
  map_topic:=/map \
  path_topic:=/scout/global_path_fixed \
  goal_topic:=/scout/goal \
  shadow_cmd_topic:=/baseline/official_lt_dwa/shadow_cmd_vel \
  raw_cmd_topic:=/baseline/official_lt_dwa/raw_cmd_vel \
  status_topic:=/baseline/official_lt_dwa/status \
  global_plan_topic:=/baseline/official_lt_dwa/global_plan \
  local_plan_topic:=/baseline/official_lt_dwa/local_plan \
  worker_result_topic:=/baseline/official_lt_dwa/worker_result \
  max_v:=0.20 \
  min_v:=0.0 \
  max_w:=1.2 \
  max_acc:=0.30 \
  max_angular_acc:=1.2 \
  planner_rate_hz:=10.0 \
  path_resample_spacing:=0.05 \
  path_tracking_lookahead_m:=1.20 \
  path_tracking_min_v:=0.08
```

shadow 只看命令是否合理，不看 path progress，因为 shadow 不推动车体。

通过条件：

```text
1. `/baseline/official_lt_dwa/status` 多数为 OK。
2. `/baseline/official_lt_dwa/shadow_cmd_vel` 持续输出。
3. `worker_result` 不连续 WAITING / STALE / local_map_service_wait_failed。
4. `final_command_w` 不应长期贴 1.2 rad/s 上限。
5. v 应该在 0.05~0.20 m/s 内变化，而不是一直硬顶 0.18。
```

### Step C：R0 短程 actuated rescue

只跑 15~25 秒。不要一上来 60 秒。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
export SCOUT_WS_ROOT=/home/geist/scout_ws
export ROS_PACKAGE_PATH=/home/geist/scout_ws/tools/lt_dwa/local_planner_runtime:${ROS_PACKAGE_PATH}

roslaunch lt_dwa_official_wrapper scout_sop_cmd_vel_benchmark.launch \
  start_local_map_service:=true \
  enable_actuated_output:=true \
  publish_cmd_vel:=true \
  cmd_vel_topic:=/cmd_vel \
  input_odom_topic:=/odom \
  map_topic:=/map \
  path_topic:=/scout/global_path_fixed \
  goal_topic:=/scout/goal \
  shadow_cmd_topic:=/baseline/official_lt_dwa/shadow_cmd_vel \
  raw_cmd_topic:=/baseline/official_lt_dwa/raw_cmd_vel \
  status_topic:=/baseline/official_lt_dwa/status \
  global_plan_topic:=/baseline/official_lt_dwa/global_plan \
  local_plan_topic:=/baseline/official_lt_dwa/local_plan \
  worker_result_topic:=/baseline/official_lt_dwa/worker_result \
  max_v:=0.20 \
  min_v:=0.0 \
  max_w:=1.2 \
  max_acc:=0.30 \
  max_angular_acc:=1.2 \
  planner_rate_hz:=10.0 \
  path_resample_spacing:=0.05 \
  path_tracking_lookahead_m:=1.20 \
  path_tracking_min_v:=0.08
```

短程通过条件：

```text
1. 肉眼不能明显离开 fixed S-curve。
2. `/baseline/official_lt_dwa/odom_map` 对 `/scout/global_path_fixed` 的 p95 path distance < 0.50 m。
3. path progress 应持续增加，不能出现明显回退。
4. 15~25s 内 progress 至少应推进到 0.10~0.20。
5. `/cmd_vel.angular.z` 不能长时间贴 1.2 rad/s。
```

如果短程不通过，按 R1/R2 调参，不要跑 60s。

### Step D：60s smoke

只有短程通过后，才跑 60s smoke。建议命名：

```text
LTDWA_fixed_0706_rescue_R0_smoke01
```

通过条件：

```text
1. 能接近或到达 goal。
2. map-frame path distance p95 < 0.50 m，max 最好 < 0.80 m。
3. path progress 最终 > 0.85；若未到终点，至少不能停在 < 0.70。
4. status OK 占多数，无连续 STALE_INPUT。
5. worker_result 不长期失败。
```

### Step E：formal N=3

只有 60s smoke 通过后，才允许 formal N=3。formal N=3 时必须固定同一组 rescue 参数，不能每轮再调。

## 6. 如果想继续用 one-click 脚本

当前脚本能直接调：

```text
MAX_V
MAX_W
MAX_ACC
MAX_ANGULAR_ACC
PLANNER_RATE_HZ
COMMAND_PUBLISH_RATE_HZ
```

但不能直接调：

```text
min_v
path_resample_spacing
path_tracking_lookahead_m
path_tracking_min_v
enable_path_tracking_guard
```

所以只靠 one-click 的最低风险尝试是：

```bash
METHOD=lt_dwa_official \
STAGE=actuated \
RUN_LABEL=LTDWA_fixed_0706_rescue_lowv01 \
MAX_V=0.20 \
MAX_ACC=0.30 \
MAX_W=1.2 \
MAX_ANGULAR_ACC=1.2 \
PLANNER_RATE_HZ=10.0 \
RECORD_SEC=25 \
MAX_RECORD_SEC=25 \
RECORD_ALL_EXISTING_TOPICS=true \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_external_baseline_real_fixed_path_trial.sh
```

但这不能改变 `path_tracking_min_v=0.18` 和 `lookahead=0.75`，所以不作为首选救援方案。

## 7. 记录口径

LT-DWA 的 tracking 统计统一用：

```text
pose: /baseline/official_lt_dwa/odom_map
path: /scout/global_path_fixed
active window: first nonzero /cmd_vel -> last nonzero /cmd_vel
```

不要用 `/odom` 直接和 fixed path 算距离。

每次 rescue 至少记录：

```text
1. bag path
2. 参数组 R0/R1/R2
3. active duration
4. status counts
5. worker reason / guard reason
6. cmd_v p50/p95/max
7. cmd_w p50/p95/max
8. odom_map -> path distance p50/p95/max
9. path progress first/last/min/max
10. 是否肉眼离轨
```

## 8. 最终判定线

```text
可救成功：
  R0/R1/R2 任一参数组短程通过，并且 60s smoke path p95 < 0.50 m、progress > 0.85。

只能作为失败 baseline 记录：
  三组 rescue 仍出现明显离轨，或 map-frame path p95 > 0.70 m，或 progress 卡在 < 0.70。

绝不进入 formal N=3：
  短程 rescue 未通过，或 60s smoke 未通过。
```

## 9. 目前建议

下一步不要直接重跑原始 LT-DWA actuated。建议按这个顺序：

```text
1. R0 shadow sanity。
2. R0 15~25s actuated rescue。
3. 若 R0 仍偏，试 R1；若角速度饱和/外切，试 R2。
4. 任一短程通过后，跑 60s smoke。
5. 60s smoke 通过后，才考虑 formal N=3。
```
