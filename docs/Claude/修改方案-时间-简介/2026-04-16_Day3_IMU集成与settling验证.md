# Day 3 执行方案：仿真优先的 IMU 集成与 T2 settling 验证

> 日期：2026-04-16
> 当前策略：先把仿真链路和 T2 settling 功能闭环做扎实；实物 A/B/C 与 settling 验证后置。
> 原主方案目标保持不变，但今日不再强行完成 9 条实物 bag。

---

## 0. 与主方案的关系

主方案 Day 3 原目标是：

1. IMU lateral acceleration 正式集成，并通过 A/B/C 实物实验验证
2. T2 settling MPC 状态机实机验证
3. `/slosh/settling_time` 正确发布

当前实际前置条件发生变化：

- 侧视相机/刻度尺外部液面锚定暂未接入
- 今日主要调通的是仿真环境
- 官方底盘 + bridge + Gazebo 真 IMU 已经可以作为仿真主链

因此 Day 3 今天改为：

- **必做**：仿真 IMU + settling 代码闭环
- **必做**：`/slosh/settling_time` 实现
- **必做**：仿真验证 `SETTLING` 能进入、退出或 timeout
- **顺延**：实物 IMU A/B/C 9 条 bag
- **顺延**：实物 settling 验证
- **顺延**：外部液面锚定结论

注意：
仿真 SL 模型评估只能作为工程 smoke test，不能替代论文里的外部液面测量结论。

---

## Step 0：整理当前状态，不立即提交 settling 代码

昨日已写好 T2 settling 初版，但还缺：

- `/slosh/settling_time`
- 仿真验证
- 实物默认 enable 策略确认

因此今天不先提交半成品，而是：

1. 保留当前未提交 settling 改动
2. 补齐 `/slosh/settling_time`
3. 编译
4. 仿真验证
5. 再提交

通过标准：

- `git status` 能清楚区分 Day3 settling 相关改动
- 不把未验证的实物默认行为提前提交

---

## Step 1：补 `/slosh/settling_time` 话题发布

总体推进方案要求 Day 3 完成标准之一：`/slosh/settling_time` 正确发布。

### 1.1 头文件改动

文件：

- `src/scout_apps/control/scout_local_planner/include/scout_local_planner/local_planner_ros.h`

新增：

```cpp
ros::Publisher slosh_settling_time_pub_;  // /slosh/settling_time
ros::Time settling_enter_time_;           // 进入 SETTLING 的时间戳
```

### 1.2 初始化发布器

文件：

- `src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp`

在已有 slosh 话题发布器附近新增：

```cpp
slosh_settling_time_pub_ = nh_.advertise<std_msgs::Float32>("slosh/settling_time", 1, true);
```

该话题使用 latched publisher，便于 settling 结束后再 echo 最近一次结果。

### 1.3 进入 SETTLING 时记录时间

在 `transitionTo()` 中：

```cpp
if (new_state == PlannerState::SETTLING) {
    settling_enter_time_ = ros::Time::now();
}
```

### 1.4 退出 SETTLING 时发布时长

在 SETTLING 进入 `REACHED` 前发布：

```cpp
publishSettlingTime(timeout);
transitionTo(PlannerState::REACHED);
```

超时进入 `REACHED` 时也必须发布，避免只在正常收敛路径有输出。

当前实现使用 `publishSettlingTime(bool timeout)` 统一处理正常收敛和 timeout 两条路径，日志中会区分 `convergence` / `timeout`。

### 1.5 编译

```bash
catkin_make --only-pkg-with-deps scout_local_planner
```

通过标准：

- 编译无 error
- `/slosh/settling_time` 在 `SETTLING -> REACHED` 时发布 `std_msgs/Float32`

---

## Step 2：仿真 IMU 状态确认

当前仿真 IMU 已切换为 Gazebo 真 IMU：

```text
Gazebo IMU plugin -> /imu/data_raw
imu_frame_relay.py -> /imu/data
```

验证命令：

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/validate_sim_imu.py --exercise
```

通过标准：

- `/imu/data` 频率约 `50 Hz`
- `header.frame_id = imu_link`
- `base_link -> imu_link = (0.10, -0.045, 0.0)`
- 小运动下 angular velocity 和 linear acceleration 有响应

当前该项已通过，可作为 Day3 仿真前置。

---

## Step 3：仿真启动链路确认

启动仿真导航骨架：

```bash
rosrun scout_local_planner launch_sim_nav_stack.sh
```

如果需要 Gazebo GUI：

```bash
USE_RVIZ=true rosrun scout_local_planner launch_sim_nav_stack.sh
```

脚本当前负责：

- 官方 Scout Mini bridge
- `maze_course.world`
- Gazebo 真 IMU
- `/scan_front`
- Cartographer sim
- MBF global planner sim
- 定位刷新：后退 4 秒，不自转

不要再单独启动：

```bash
roslaunch nanoscan3_bringup nanoscan3_front_sim.launch
```

通过标准：

```bash
rostopic hz /scan_front
rostopic hz /imu/data
rostopic echo -n 1 /scout/global_path
```

- `/scan_front` 约 `11 Hz`
- `/imu/data` 约 `50 Hz`
- MBF 能发布全局路径

---

## Step 4：仿真验证 T2 settling 状态

启动局部规划：

```bash
roslaunch scout_local_planner slosh_experiment_sim.launch \
  risk_scheduler_enable:=true \
  Q_slosh:=5.0
```

建议先只发一个普通目标点，观察：

```bash
rostopic echo /terminal/mode
rostopic echo /mpc_status
rostopic echo -n 1 /slosh/settling_time
```

注意：不再使用自写运行时 observer 脚本参与验证，避免给定位链路引入额外变量。`/slosh/settling_time` 是 latched topic，settling 结束后再 echo 也能拿到最近一次结果。

bag 录制建议：

```bash
mkdir -p /data/a/slosh_bags/sim/day3_settling
rosbag record -O /data/a/slosh_bags/sim/day3_settling/settling_sim_$(date +%Y%m%d_%H%M%S).bag \
  /cmd_vel /odom /imu/data \
  /slosh/state /slosh/height /slosh/settling_time \
  /terminal/mode /mpc_status \
  /scout/global_path /local_path \
  /risk_scheduler/rho_k /risk_scheduler/u_k /risk_scheduler/Q_eta_k \
  /risk_scheduler/fallback_active
```

bag 录完后做离线检查：

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/analyze_settling_day3.py \
  /data/a/slosh_bags/sim/day3_settling/settling_sim_YYYYMMDD_HHMMSS.bag
```

更严格的检查：

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/analyze_settling_day3.py \
  --strict \
  /data/a/slosh_bags/sim/day3_settling/settling_sim_YYYYMMDD_HHMMSS.bag
```

通过标准：

- 状态能出现 `SETTLING`
- 最终能进入 `REACHED`，或者在 `timeout_s` 后退出
- `/slosh/settling_time` 有数值
- 不破坏正常 TRACKING
- `analyze_settling_day3.py` 输出整体 `PASS`；若只因 bag 停得太早没有采到 `REACHED`，保留终端日志作为补充证据

首条 bag 复盘：

- `/data/a/slosh_bags/sim/day3_settling/settling_sim_20260416_213523.bag`
- 未进入 `SETTLING`
- `/scout/global_path` 仅 2 条，`/local_path` 在约 10.56s 后停止
- `mpc_status` 进入 `ERROR`，判断与仿真 `path_timeout=5s` 过短有关
- 已将 `mpc_params_sim.yaml` 的 `path_handler/path_timeout` 放宽到 `30.0s`，下一条 bag 需重启 local planner 后复测

复测 bag：

- `/data/a/slosh_bags/sim/day3_settling/settling_sim_20260416_214508.bag`
- `mpc_status`: `TRACKING -> SETTLING -> REACHED`
- `SETTLING` 观测区间：`12.77s -> 13.17s`
- `/slosh/settling_time`: `0.45s`
- `analyze_settling_day3.py` verdict: `PASS`

---

## Step 5：实物默认参数策略

当前 T2 settling 改动涉及：

- `config/mpc_params.yaml`
- `config/mpc_params_sim.yaml`

在实物尚未验证前，建议策略是：

- `mpc_params_sim.yaml` 可以保持 `settling.enable: true`
- `mpc_params.yaml` 建议先设为 `settling.enable: false`
- C++ 内部默认值也设为 `false`，避免某个 launch 未加载 YAML 时误启用 settling

这样可以保证：

- 仿真继续验证 settling
- 实物主链默认行为不被未验证功能改变

实物阶段需要显式打开时，再通过参数或 launch 覆盖。

---

## Step 6：提交 T2 settling 仿真闭环

只有满足以下条件后再提交：

- `/slosh/settling_time` 已实现
- `scout_local_planner` 编译通过
- 仿真中 `SETTLING` 至少一次进入并退出或 timeout
- 实物默认 enable 策略已确认

提交内容建议包括：

```bash
git add \
  src/scout_apps/control/scout_local_planner/include/scout_local_planner/types.h \
  src/scout_apps/control/scout_local_planner/include/scout_local_planner/local_planner_ros.h \
  src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp \
  src/scout_apps/control/scout_local_planner/config/mpc_params.yaml \
  src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml \
  src/scout_apps/control/scout_local_planner/CMakeLists.txt \
  src/scout_apps/control/scout_local_planner/scripts/analyze_settling_day3.py \
  docs/Claude/修改方案-时间-简介/2026-04-16_Day3_IMU集成与settling验证.md \
  docs/Claude/修改日志-时间/2026-04-16.md
git commit -m "Day 3: add T2 settling time publication and sim validation"
```

---

## Step 7：IMU lateral_accel A/B/C 仿真 smoke test

今天如果 settling 仿真闭环完成，再做 A/B/C 的**仿真 smoke test**。

配置：

| 配置 | 参数 |
|---|---|
| A | `slosh_use_imu_yaw_rate=true`, `slosh_use_imu_lateral_accel=false` |
| B | `slosh_use_imu_yaw_rate=true`, `slosh_use_imu_lateral_accel=true` |
| C | `slosh_use_imu_yaw_rate=true`, `slosh_use_imu_lateral_accel=true`, `risk_scheduler_enable=true` |

每组先跑 1 条 bag，确认：

- IMU ay 进入链路
- `/slosh/imu_ay_bias_ready = true`
- C 组 `/risk_scheduler/u_k` 有非零值
- MPC 不崩、不明显发散

这一步只作为工程 smoke test，不替代实物 A/B/C 实验。

推荐 bag 目录：

```bash
mkdir -p /data/a/slosh_bags/sim/day3_abc
```

每组启动局部规划前，先确保仿真导航骨架已启动：

```bash
rosrun scout_local_planner launch_sim_nav_stack.sh
```

### A 组：不用 IMU ay

```bash
roslaunch scout_local_planner slosh_experiment_sim.launch \
  Q_slosh:=5.0 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  risk_scheduler_enable:=false
```

录包建议：

```bash
SLOSH_BAG_DIR=/data/a/slosh_bags/sim/day3_abc \
SLOSH_BAG_MODE=sim \
rosrun scout_local_planner record_slosh_experiment.sh 5 day3_A_no_imu_ay
```

### B 组：启用 IMU ay

```bash
roslaunch scout_local_planner slosh_experiment_sim.launch \
  Q_slosh:=5.0 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=true \
  risk_scheduler_enable:=false
```

录包建议：

```bash
SLOSH_BAG_DIR=/data/a/slosh_bags/sim/day3_abc \
SLOSH_BAG_MODE=sim \
rosrun scout_local_planner record_slosh_experiment.sh 5 day3_B_imu_ay
```

### C 组：启用 IMU ay + risk scheduler

```bash
roslaunch scout_local_planner slosh_experiment_sim.launch \
  Q_slosh:=5.0 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=true \
  risk_scheduler_enable:=true
```

录包建议：

```bash
SLOSH_BAG_DIR=/data/a/slosh_bags/sim/day3_abc \
SLOSH_BAG_MODE=sim \
rosrun scout_local_planner record_slosh_experiment.sh 5 day3_C_imu_ay_risk
```

三条 bag 完成后离线分析：

```bash
rosrun scout_local_planner analyze_day3_abc_smoke.py \
  A=/data/a/slosh_bags/sim/day3_abc/<A组bag>.bag \
  B=/data/a/slosh_bags/sim/day3_abc/<B组bag>.bag \
  C=/data/a/slosh_bags/sim/day3_abc/<C组bag>.bag
```

通过标准：

- A/B/C 三组均有 `/imu/data`
- B/C 组 `/slosh/imu_ay_bias_ready` 大部分时间为 `true`
- C 组 `/risk_scheduler/u_k` 有非零响应
- C 组 `/risk_scheduler/fallback_active` 不应长期占主导
- 三组均能进入 `TRACKING`，且 MPC 不出现全程失败

当前未录 bag 的人工观察：

- A/B 组可能无法稳定 `goal reached`
- C 组更容易 `goal reached`
- 这说明 `risk_scheduler_enable=true` 可能通过降低有效参考速度或提高晃动权重，改善终点段可达性
- 因此第一轮 A/B/C smoke 不把 A/B 终点到达作为硬验收；A/B 只要求链路正常、能进入 `TRACKING`、IMU ay 相关话题正常、不明显发散
- C 组则重点确认 `/risk_scheduler/u_k` 非零、fallback 不长期占主导，并记录是否能到达 `REACHED`

复测 bag 结果：

- A：`/data/a/slosh_bags/sim/day3_abc/slosh_Q5_20260416_225005_day3_A_no_imu_ay.bag`
  - `overall: PASS`
  - `mpc_status`: 包含 `TRACKING/SETTLING/REACHED`
  - `/mpc/status_val success_ratio=0.601`
- B：`/data/a/slosh_bags/sim/day3_abc/slosh_Q5_20260416_225145_daday3_B_imu_ay.bag`
  - `overall: PASS`
  - `/slosh/imu_ay_bias_ready true_ratio=0.500`
  - `/slosh/ay_est` 使用 IMU ay filtered，p95 约 `2.056 m/s^2`
- C：`/data/a/slosh_bags/sim/day3_abc/slosh_Q5_20260416_225318_daday3_C_imu_ay_risk.bag`
  - `overall: PASS`
  - `/risk_scheduler/u_k` 非零，p95=`1.0`
  - `/risk_scheduler/fallback_active true_ratio=0.361`，未长期占主导
  - `mpc_status`: 包含 `TRACKING/SETTLING/REACHED`

---

## 实物阶段顺延项

以下内容今天不强行完成，后续实物环境稳定后再做：

1. IMU 零偏标定实物确认
2. A/B/C 各 3 条实物 bag，共 9 条
3. 外部液面测量或侧视相机锚定
4. settling 实物验证
5. A/B/C peak height / p95 统计

---

## Day 3 今日完成标准

### 必做

- [x] `/slosh/settling_time` 实现并编译通过
- [x] 仿真真 IMU 验证通过，`frame_id=imu_link`
- [x] 仿真链 `bridge + localization + MBF + local planner` 可启动
- [x] 仿真中验证 `SETTLING` 能进入、退出或 timeout
- [x] 实物默认 `settling.enable` 策略确认
- [x] T2 settling 仿真闭环提交

### 可选

- [x] A/B/C 仿真 smoke test 各 1 条 bag
- [ ] ISR ZV shaper 初版实现（暂不混入 Day3 settling/IMU 仿真收尾）

### 顺延到实物阶段

- [ ] IMU A/B/C 各 3 条实物 bag
- [ ] settling 实机验证
- [ ] 外部液面锚定与统计结论
