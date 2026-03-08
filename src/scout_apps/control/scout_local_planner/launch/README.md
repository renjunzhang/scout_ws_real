# scout_local_planner launch 说明

本文档说明 `test_mpc.launch`、`test_mpc_sim.launch`、`slosh_experiment.launch` 三个启动文件的区别，以及推荐使用场景。

## 实物调参指导

### 先按这个顺序调，不要一开始同时改很多项

1. 先做纯跟踪基线  
   - 先用 `test_mpc.launch`，或者用 `slosh_experiment.launch` 但保持：
     - `Q_slosh:=0`
     - `enable_slosh_box_constraint:=false`
     - `slosh_speed_governor_enable:=false`
   - 目的：先把“路径跟踪”调顺，再加 anti-slosh。

2. 先调约束和路径层，再调权重  
   - 先看：
     - `vehicle/v_max`
     - `vehicle/omega_max`
     - `vehicle/a_max`
     - `vehicle/alpha_max`
     - `path_handler/lookahead_distance`
     - `path_handler/max_lat_accel`
     - `path_handler/max_tan_accel`
     - `path_handler/max_tan_decel`
   - 再看：
     - `Q_ec / Q_contour / Q_etheta / Q_v`
     - `R_omega / R_domega / R_a / R_da`

3. 每次只改 1~2 个参数，单次改动控制在 10%~30%  
   - 不要一边改 `lookahead_distance`，一边改 `Q_ec`，再一边开 `Q_slosh`。

4. 先看基础跟踪，再看终点，再看 anti-slosh  
   - 基础跟踪没稳之前，不要急着开：
     - `Q_slosh`
     - `enable_slosh_box_constraint`
     - `slosh_speed_governor_enable`

### 实物调参时建议同时观察的话题

```bash
rostopic echo /mpc_status
rostopic echo /mpc/status_val
rostopic echo /cmd_vel
rostopic echo /local_path
rostopic echo /scout/global_path
```

如果在做 anti-slosh 实验，再额外看：

```bash
rostopic echo /slosh/height_pred_max
rostopic echo /slosh/v_des_eff
rostopic echo /slosh/speed_governor_active
```

### 常见问题 -> 优先调整哪些参数

| 现象 | 优先看哪些参数 | 调整方向 |
|---|---|---|
| 跟踪不顺滑、角速度发抖、`cmd_vel.angular.z` 锯齿 | `R_domega`、`R_omega`、`lookahead_distance`、`vehicle/alpha_max`、`vehicle/omega_max` | 先增大 `R_domega`；再适当增大 `R_omega`；必要时增大 `lookahead_distance`（如 `0.5 -> 0.6/0.7`）；若还太激进，降低 `alpha_max` 或 `omega_max` |
| 车总是扭来扭去、左右摆头 | `Q_ec/Q_contour`、`Q_etheta`、`R_domega`、`lookahead_distance` | 一般是横向纠偏太猛或航向收敛不稳；可适当降低 `Q_ec/Q_contour`，提高 `Q_etheta`，再提高 `R_domega`；`lookahead_distance` 太小也会更容易来回摆 |
| 速度不快、明显偏保守 | `slosh_speed_governor_enable`、`/slosh/speed_governor_active`、`/slosh/v_des_eff`、`vehicle/v_max`、`path_handler/max_lat_accel`、`path_handler/max_tan_accel`、`Q_v`、`R_a` | 先确认是不是 governor 在限速；如果是基础跟踪调参，先关闭 anti-slosh；若 governor 没介入仍然慢，再逐步增大 `v_max`、`max_lat_accel`、`max_tan_accel`，必要时提高 `Q_v` 或适当减小 `R_a` |
| 直线还行，但弯道跟不上、切弯、贴墙 | `Q_ec/Q_contour`、`Q_etheta`、`lookahead_distance`、`path_handler/max_lat_accel`、`vehicle/omega_max` | 如果切弯太厉害，先提高 `Q_ec/Q_contour`；如果弯中姿态跟不住，再提高 `Q_etheta`；如果前视太大导致抄近路，可适当减小 `lookahead_distance`；如果响应能力不够，再小步提高 `omega_max` |
| 终点附近提前停住、很难进 `REACHED` | `goal_capture_distance`、`goal_capture_min_speed`、`goal_tolerance`、`yaw_tolerance` | 先小幅增大 `goal_capture_distance` 或 `goal_capture_min_speed`，避免最后一段速度掉死；仍不进 `REACHED` 时，再适当放宽 `goal_tolerance / yaw_tolerance` |
| 求解失败、`/mpc/status_val` 频繁为 0 | `vehicle/*` 约束、`path_handler/max_*`、`enable_slosh_box_constraint`、`slosh_speed_governor_enable` | 先降低激进程度：减小 `v_max/omega_max/max_lat_accel/max_tan_accel`；调基础跟踪时先关闭盒约束和 governor；先保证可行性，再追求性能 |

### 当前实物参数的调参建议

当前 `mpc_params.yaml` 是偏保守的实物版本，特点是：

- `vehicle/v_max = 1.0`
- `vehicle/omega_max = 1.0`
- `vehicle/a_max = 1.5`
- `vehicle/alpha_max = 2.5`
- `path_handler/lookahead_distance = 0.5`
- `path_handler/max_lat_accel = 1.5`

这套配置的目标是先稳，不是先快。  
如果你现在的主要问题是“太慢”，建议按这个顺序试：

1. `vehicle/v_max: 1.0 -> 1.2`
2. `path_handler/max_lat_accel: 1.5 -> 1.8`
3. `vehicle/omega_max: 1.0 -> 1.2`
4. `path_handler/max_tan_accel: 1.5 -> 1.8`

如果你现在的主要问题是“扭来扭去”，建议按这个顺序试：

1. `R_domega: 3.0 -> 4.0`
2. `lookahead_distance: 0.5 -> 0.6`
3. `Q_etheta: 12.5 -> 14.0`
4. 若仍过猛，再把 `alpha_max: 2.5 -> 2.0`

### anti-slosh 参数怎么调

只有在基础跟踪已经稳定后，再调这组：

- `Q_slosh`
- `enable_slosh_box_constraint`
- `slosh_speed_governor_enable`
- `slosh_speed_governor_k_eta`
- `slosh_speed_governor_ay_max_base`

建议顺序：

1. `Q_slosh:=0` 跑通基础跟踪
2. `Q_slosh:=5` 打开 soft cost
3. 再打开 `enable_slosh_box_constraint:=true`
4. 最后再打开 `slosh_speed_governor_enable:=true`

当前工程建议仍然是：

- **实物默认实验点优先用 `Q_slosh:=5`**
- 不建议一开始就上 `Q_slosh:=10` 并同时打开所有 anti-slosh 机制

## 三者区别总表

| launch 文件 | 配置文件 | 默认场景 | 默认 `Q_slosh` | 额外实验参数覆盖 | 适用用途 | 推荐程度 |
|---|---|---|---:|---|---|---|
| `test_mpc.launch` | `config/mpc_params.yaml` | 实物 | `0.0` | 无 | 日常实物 MPC 跟踪、普通导航 | 实物日常使用推荐 |
| `test_mpc_sim.launch` | `config/mpc_params_sim.yaml` | 仿真 | `5.0` | 无 | 日常仿真 MPC 跟踪、快速验证 | 仿真日常使用推荐 |
| `slosh_experiment.launch` | `config/mpc_params.yaml` 或 `config/mpc_params_sim.yaml` | 实物/仿真（由 `sim` 决定） | `0.0` | 有，集中覆盖 anti-slosh 实验参数 | 液体晃动抑制实验、消融对比、参数扫描 | anti-slosh 实验推荐 |

## 关键差异

| 对比项 | `test_mpc.launch` | `test_mpc_sim.launch` | `slosh_experiment.launch` |
|---|---|---|---|
| 是否区分实物/仿真 | 只用于实物 | 只用于仿真 | 通过 `sim:=true/false` 切换 |
| 是否只暴露少量参数 | 是 | 是 | 否，集中暴露实验参数 |
| 是否默认关闭执行端额外 EMA | 否 | 否 | 是，默认 `filter/alpha_v=1.0`、`filter/alpha_omega=1.0`、`filter/kappa_boost=0.0` |
| 是否适合做 `Q_slosh` 消融 | 一般 | 一般 | 是 |
| 是否支持盒约束开关 | 需手动额外传参 | 需手动额外传参 | 直接支持 |
| 是否支持 speed governor 参数集中传入 | 需手动额外传参 | 需手动额外传参 | 直接支持 |
| 是否适合论文实验复现实验口径 | 不推荐 | 不推荐 | 推荐 |

## 为什么做液体晃动实验时优先用 `slosh_experiment.launch`

原因不是它“更高级”，而是它的职责更明确：

1. 它把 anti-slosh 相关参数集中暴露出来，避免每次手工拼很多 launch arg。
2. 它默认关闭实验中不希望引入的执行端额外 EMA，减少“模型外隐藏动态”。
3. 它统一了实验入口，便于 rosbag 录制、结果复现和消融分析。

因此：

- **普通 MPC 跟踪 / 日常导航**：优先用 `test_mpc.launch` 或 `test_mpc_sim.launch`
- **液体晃动抑制实验 / 消融分析**：优先用 `slosh_experiment.launch`

## 推荐启动方式

### 1. 实物日常跟踪

```bash
roslaunch scout_local_planner test_mpc.launch
```

### 2. 仿真日常跟踪

```bash
roslaunch scout_local_planner test_mpc_sim.launch
```

### 3. 实物液体晃动实验

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  Q_slosh:=5 \
  enable_slosh_box_constraint:=true \
  slosh_speed_governor_enable:=true
```

### 4. 仿真液体晃动实验

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  sim:=true \
  Q_slosh:=5 \
  enable_slosh_box_constraint:=true \
  slosh_speed_governor_enable:=true
```

### 5. 实物 IMU 预留实验

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  Q_slosh:=5 \
  enable_slosh_box_constraint:=true \
  slosh_speed_governor_enable:=true \
  slosh_use_imu_lateral_accel:=true \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_alpha_z:=true \
  slosh_imu_topic:=/imu/data
```

## `slosh_experiment.launch` 常用参数

| 参数 | 作用 | 常用值 |
|---|---|---|
| `sim` | 是否加载仿真参数文件 | `true / false` |
| `Q_slosh` | 晃动软代价权重 | `0 / 5 / 10 / 20` |
| `enable_slosh_box_constraint` | 是否启用第一版液面盒约束代理 | `true / false` |
| `slosh_speed_governor_enable` | 是否启用残余晃动感知速度治理 | `true / false` |
| `slosh_speed_governor_k_eta` | 液面高度比例缩放系数 | `2.5` |
| `slosh_speed_governor_eta_deadband` | 介入死区 | `0.3` |
| `slosh_speed_governor_eta_exit_ratio` | governor 退出阈值（滞回） | `0.2` |
| `slosh_speed_governor_min_active_steps` | 最少保持周期数 | `10` |
| `slosh_speed_governor_ay_max_base` | 横向加速度预算 | `0.6`（实验常用起点） |
| `slosh_speed_governor_v_des_min` | 调速后的最低参考速度 | `0.2` |
| `slosh_speed_governor_preview_distance` | 前方曲率预览长度 | `1.0` |
| `slosh_use_imu_lateral_accel` | 是否用 IMU `linear_acceleration.y` 替代 `v*omega` | `true / false` |
| `slosh_use_imu_yaw_rate` | 是否用 IMU `angular_velocity.z` 替代 odom `omega` | `true / false` |
| `slosh_use_imu_alpha_z` | 是否用 IMU 角速度差分替代 `alpha_z` | `true / false` |
| `slosh_imu_topic` | IMU 输入话题 | `/imu/data` |
| `slosh_imu_filter_alpha` | IMU 数据 EMA 滤波系数 | `0.3` |

## 实际使用建议

- 如果你只是想确认 MPC 能不能跟踪路径，不要先上 `slosh_experiment.launch`。
- 如果你要录 bag、做 `Q=0/5/10` 对比，直接用 `slosh_experiment.launch`，不要混用 `test_mpc*.launch`。
- `test_mpc_sim.launch` 当前默认 `Q_slosh=5.0`，这更像“带一定 anti-slosh 倾向的仿真默认入口”，不是严格的消融基线。
- 如果要验证阶段 7，优先只切换 `slosh_use_imu_*` 和 `slosh_imu_topic`，不要同时再改一组 governor 参数。
