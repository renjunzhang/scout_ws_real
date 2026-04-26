# `scripts` 目录说明

本目录存放 `scout_local_planner` 相关的实验辅助脚本，主要用于固定目标/固定路径实验、录包、离线指标提取，以及 IMU 标定验证。

## 脚本列表

### `send_fixed_goal.py`

向指定 ROS 话题发布一个固定的 `geometry_msgs/PoseStamped` 目标点，默认发到 `/scout/goal`。

适用场景：
- 手动触发一次固定终点跟踪
- 做对照实验时保证 goal pose 一致

### `sim_fixed_goal_tool.py`

仿真专用固定 goal 工具，支持：
- `capture`：抓取当前 `/scout/goal` 并保存为 JSON
- `show`：显示已保存 goal 的 `frame/x/y/yaw`
- `replay`：从 JSON 重发同一个 goal 到 `/scout/goal`

适用场景：
- Day4/Day5 仿真实验要求终点完全一致
- 不再依赖 RViz 手点 goal，避免人为偏差

### `fixed_global_path_runner.py`

固定 `/scout/global_path` 的采集与回放工具，支持以下模式：
- `capture`：抓取第一次收到的全局路径并保存到 JSON
- `replay`：从 JSON 读取固定路径并持续发布
- `capture_and_replay`：先抓取再回放
- `goal_only`：只发布已保存路径的起点或终点 goal

适用场景：
- 先记录第一次规划出的路径，再让 Q0/Q5 都复用同一条 `/scout/global_path`
- 正式 tracking 前，先将机器人移回固定路径起点并对齐，再开始局部跟踪

补充能力：
- `--manual-start`：实物场景下先人工摆位，按 Enter 后再进入起点门控 / replay
- `--skip-start-wait`：直接跳过起点门控，用于快速 debug

### `template_fixed_path_generator.py`

从当前机器人位姿到点击终点，自动生成标准化固定路径模板并发布到 `/scout/global_path_fixed`。默认等待 RViz `2D Nav Goal` 发布到 `/scout/goal`。

仿真 open 场景采集固定路径时，推荐加 `--start-heading current`，让路径起点朝向等于当前车头方向：
```bash
rosrun scout_local_planner template_fixed_path_generator.py \
  --template s_curve \
  --start-heading current \
  --goal-topic /scout/goal \
  --output-topic /scout/global_path_fixed \
  --path-file /data/a/fixed_paths/sim/P2_s_curve.json
```
使用该模式时，RViz 终点需要点在当前车头前方。

当前支持模板：
- `straight`
- `single_turn`
- `s_curve`
- `mixed`
- `multi_s`
- `sharp_turn`

适用场景：
- 空旷场地快速生成可重复的直线 / 单弯 / S 弯 / 连续 S 弯实验路径
- 生成后直接给 `slosh_experiment.launch global_path_topic:=/scout/global_path_fixed`
- 可选同时保存为 JSON，供后续 fixed-path replay 复用

### `launch_fixed_path_slosh_stack.sh`

固定路径 slosh 实验的一键启动脚本，按顺序启动：
- `nanoscan3_localization scout_nanoscan3_cartographer_localization.launch`
- `scout_global_planner mbf_global.launch`
- `scout_local_planner slosh_experiment.launch`

默认让 local planner 订阅 `/scout/global_path_fixed`，并保持第一轮有效性验证的固定口径：
- `enable_slosh_box_constraint=false`
- `slosh_speed_governor_enable=false`
- `filter_alpha_v=1.0`
- `filter_alpha_omega=1.0`
- `slosh_use_imu_lateral_accel=false`
- `slosh_use_imu_yaw_rate=true`

用法：
```bash
rosrun scout_local_planner launch_fixed_path_slosh_stack.sh 0
rosrun scout_local_planner launch_fixed_path_slosh_stack.sh 5
rosrun scout_local_planner launch_fixed_path_slosh_stack.sh 10
```

脚本会在启动 `nanoscan3_localization scout_nanoscan3_cartographer_localization.launch` 后暂停，等待定位准确度达到 `70%` 后再继续启动 global planner 和 local planner。当前 Cartographer launch 中没有明确的“定位准确度百分比”ROS topic，因此默认是人工确认门：看到定位准确度达到 `70%` 后按 Enter 继续。

如果后续已有可读的准确度话题，可以用环境变量启用自动等待：
```bash
LOCALIZATION_ACCURACY_TOPIC=/your/localization_accuracy_topic \
LOCALIZATION_ACCURACY_THRESHOLD=70 \
rosrun scout_local_planner launch_fixed_path_slosh_stack.sh 5
```

如果需要临时换固定路径话题：
```bash
GLOBAL_PATH_TOPIC=/scout/global_path_fixed rosrun scout_local_planner launch_fixed_path_slosh_stack.sh 5
```

### `record_slosh_experiment.sh`

`rosbag` 录包脚本，用于记录 anti-slosh MPC 实验相关话题。

主要覆盖：
- 液面/晃动估计话题，如 `/slosh/height`、`/slosh/height_pred_max`
- 控制与状态话题，如 `/cmd_vel`、`/odom`、IMU、MPC 状态
- 路径与任务上下文，如 `/scout/goal`、`/scout/global_path`
- RealSense 液面测量和终点恢复诊断相关话题

适用场景：
- 录制 Q0/Q5 对照 bag
- 录制 IMU 标定或终点恢复行为分析 bag

### `record_slosh_debug.sh`

轻量级调试录包脚本，面向实物参数整定与根因定位。

与 `record_slosh_experiment.sh` 的区别：
- 不录 RealSense 原始图像
- 不录地图、costmap、MBF 大量接口话题
- 只保留 `cmd_vel / odom / mpc_status / v_des_eff / terminal / 路径几何 / IMU / TF` 等核心诊断信号

适用场景：
- 实物 `Q0/Q5` 调参
- 配合 `diagnose_real_tuning.py` 快速判断下一轮该调哪些参数
- 需要更小 bag 体积、方便从工控机拷回开发机

### `extract_slosh_metrics.py`

从 bag 文件中提取 slosh/MPC 实验指标的离线分析脚本。

可提取内容包括：
- `/slosh/height`、`/slosh/height_pred_max` 的峰值和统计量
- `/mpc/solve_ms`、`/mpc/status_val` 的求解性能
- governor、约束触发、速度命令等运行指标
- 按 `mpc_status` 分段统计成功率（`TRACKING / SETTLING / REACHED / IDLE` 各段单独输出），避免整包 success_ratio 被 near-goal 段污染

适用场景：
- 比较不同参数组下的晃动指标和控制开销
- 判断 success_ratio 低的根因来自哪个阶段
- 批量导出 CSV 做进一步统计

### `analyze_settling_day3.py`

Day 3 T2 settling 验证专用分析脚本，从 bag 中检查 settling 状态机的进出行为。

关注内容包括：
- `mpc_status` 是否经过 `TRACKING → SETTLING → REACHED` 完整链路
- `/slosh/settling_time` 是否正常发布及其数值
- `SETTLING` 阶段的 `status_val` 成功率
- `/risk_scheduler/u_k`、`fallback_active`、`rho_k` 在 settling 阶段的行为

适用场景：
- 验证 T2 settling 状态机能否在实物/仿真中正常进出
- 排查"SETTLING 一直没进入"或"settling_time 没发布"的问题

### `analyze_day3_abc_smoke.py`

Day 3 A/B/C 配置对照的仿真 smoke 检查脚本，对比三种 IMU 使用配置下的关键指标。

三种配置：
- A：`yaw_rate=true, lateral_accel=false`（默认）
- B：`yaw_rate=true, lateral_accel=true`
- C：`yaw_rate=true, lateral_accel=true + risk_scheduler`

关注内容：
- 各配置的 `TRACKING success rate`、`solve_ms p95`
- `/slosh/height p95`、`fallback_active true_ratio`
- IMU ay bias ready 比例、`imu_ay_filtered` 与 `ay_est` 的一致性

用法：
```bash
rosrun scout_local_planner analyze_day3_abc_smoke.py \
  A=<bag_A> B=<bag_B> C=<bag_C>
```

### `analyze_sim_speed_issue.py`

仿真 MPC 速度偏慢问题诊断脚本，从 bag 中分析机器人在哪些阶段速度低于预期。

关注内容：
- 低速（默认阈值 0.25 m/s）帧数占比及分布
- `/slosh/v_des_eff` 与实际 `/cmd_vel` 的差异
- `risk_scheduler/rho_k`、`Q_eta_k`、`fallback_active` 在低速段的状态
- `terminal/mode` 是否在 near-goal 段拉低整体速度

适用场景：
- 区分"全程慢"还是"near-goal 段慢"
- 判断速度低是 risk scheduler 激进还是 terminal recovery 保守

### `analyze_global_path_duplicates.py`

定位 `/scout/global_path` 中重复点/极短段的脚本，输出每个可疑位置的索引和坐标。

关注内容：
- 段长低于阈值（默认 1e-3 m）的相邻点对
- 每对可疑短段的 `idx → idx+1`、段长、前后点坐标

适用场景：
- 确认全局路径发布链是否存在重复 waypoint
- 为 `PathHandler::sanitizePolyline` 的阈值设置提供依据

### `analyze_tracking_infeasible.py`

TRACKING 阶段 OSQP -3 根因分析脚本，将 solver 失败时刻与路径几何异常对齐。

关注内容：
- `/mpc/status_val` 失败时刻最近的 `/mpc/reference_path`、`/scout/global_path_smooth` 几何量（max_kappa、max_dkappa、min_seg）
- 失败事件的几何特征分布（按路径来源分层）
- 判断失败主因：全局路径几何过激 / 局部参考几何病态 / 非几何主导

适用场景：
- 定位 TRACKING infeasible 的真正根因
- 区分"geometric failure"和"constraint structure failure"（如 u_prev 冻结）

### `analyze_global_path_prefix_window.py`

分析 `/scout/global_path` 前段窗口几何的专项脚本，面向"路径起步阶段 fitLocalSpline 频繁失败"场景。

关注内容：
- 前 K 个路径点（默认 89 个，对应 `closest_idx=0, end=88` 的典型失败窗口）的段长、航向跳变和曲率变化率
- 最短段、最大航向跳变的具体索引和坐标

用法：
```bash
python3 analyze_global_path_prefix_window.py <bag> --window-size 89
```

适用场景：
- 定位前段局部窗口几何过激的具体点位
- 判断是单点折返尖刺还是正常大曲率弯道

### `diagnose_speed_profile.py`

速度剖面限速来源诊断脚本，对单条 bag 分析当前速度慢的根因属于哪一候选。

三类候选根因：
- **A**：`global_spline kappa` 远大于实际路径几何（cubic spline 二阶导数放大）
- **B**：`dkappa` 有限差分噪声放大（`dkappa >> 100 1/m²`）
- **C**：局部 reactive cap 仍在主导全局速度剖面（速度剖面计算正确，但被执行层重写）

输出内容：
- 全局平滑路径的 kappa/dkappa 统计及各项 `v_geom_min`（a_lat / omega / alpha 三约束）
- 实际 cmd_vel 速度分布（均值、中位数、低速段占比）

用法：
```bash
python3 diagnose_speed_profile.py <bag> --omega-max 2.0 --alpha-max 4.0 --a-lat-max 1.0
```

适用场景：
- 判断速度慢的主因在规划层还是执行层
- 验证 P1 v_plan/v_exec 解耦是否真正生效

### `observe_terminal_recovery.py`

终点恢复行为观察脚本，用于在线或回放时判断机器人在终点附近是否进入终点恢复逻辑，并输出阶段性状态。

关注内容包括：
- 当前位置与 goal 的相对关系
- 是否处于终点对点接近、终点朝向对齐等恢复阶段
- `/cmd_vel` 与 `/odom` 是否符合预期恢复行为

适用场景：
- 分析 near-goal/terminal recovery 行为是否合理
- 排查“到点后转不正”“终点附近抖动”之类问题

### `imu_ay_tool.py`

IMU 横向加速度一体化工具，合并 Stage-2 标准动作、离线健康检查和 `imu_ay_scale` 标定。

子命令：
- `sequence`：向 `/cmd_vel` 发布固定低速动作序列：静止、左弧线、静止、右弧线、静止。
- `analyze`：读取 bag 中的 `/imu/data`、`/odom` 和 `/slosh/imu_ay_*`，检查静止残差、左右转符号、与 `v*omega` 的相关性。
- `calibrate`：从单个标定 bag 估计 `slosh_estimator/imu_ay_scale`，输出 YAML，可选保存验证图。

典型用法：
```bash
rosrun scout_local_planner imu_ay_tool.py sequence --linear 0.30 --omega 0.30
python3 scripts/imu_ay_tool.py analyze /path/to/imu_calib.bag
python3 scripts/imu_ay_tool.py calibrate /path/to/imu_calib.bag --output /data/a/imu_calib/imu_ay_calibration.yaml --plot
```

适用场景：
- 采集 IMU 横向加速度 `a_y` 标定 bag
- 判断 IMU `a_y` 是否已经达到可接入控制器的质量
- 生成可写入 launch / yaml 的 `imu_ay_scale`

### `run_imu_stage4_sequence.py`

Stage-4 IMU `alpha_z` 验证动作脚本，向 `/cmd_vel` 发布一组固定的原地旋转序列。

典型序列：
- 静止
- 左自旋
- 静止
- 右自旋
- 静止

适用场景：
- 采集 IMU 角加速度/角速度变化验证 bag
- 为 Stage-4 相关分析提供标准化激励

### `validate_sim_imu.py`

在线验证仿真 IMU 话题质量的脚本（需要 ROS 节点在线，非离线 bag 分析）。

关注内容：
- `/imu/data` 频率、时间戳一致性、`frame_id`
- IMU `linear_acceleration.y` 与运动学估计 `v * omega` 的偏差
- TF `imu_link` 是否可查询

适用场景：
- 仿真首次接入真 IMU 时的链路验收
- 确认 Gazebo IMU 插件输出与 `/odom` 时间戳对齐

### `launch_sim_nav_stack.sh`

仿真导航栈一键启动脚本，顺序拉起 Gazebo、Cartographer 定位、全局规划器。

支持环境变量配置：
- `USE_RVIZ`：是否启动 RViz（默认 false）
- `SPAWN_X/Y/Z`：机器人初始位置
- `GAZEBO_WAIT_S`：等待 Gazebo 就绪时间
- `LOCALIZATION_BACKUP_V`：定位初始化倒退速度

用法：
```bash
rosrun scout_local_planner launch_sim_nav_stack.sh
USE_RVIZ=true rosrun scout_local_planner launch_sim_nav_stack.sh
```

注意：本脚本只启动到全局规划器，不启动 local planner。local planner 需要单独用 `slosh_experiment_sim.launch` 启动，以便灵活传入实验参数。

### `run_day4_profile.sh`

Day 4 仿真参数组合包装脚本，统一管理 baseline / conservative / no_imu_ay / relaxed_settling 四组预定义参数，避免手工切换时漏参。

支持的 profile：

| profile | 说明 |
|---|---|
| `baseline` | Day4 C baseline，默认参数 |
| `conservative` | 保守风险调度（gamma=3.0, rho_0=0.4, rate_limit=0.02, beta=0.2） |
| `no_imu_ay` | 关闭 IMU 横向加速度，保留风险调度 |
| `relaxed_settling` | 放宽 settling 终止阈值（eta_tol=0.002, eta_dot_tol=0.05） |

用法：
```bash
rosrun scout_local_planner run_day4_profile.sh conservative
rosrun scout_local_planner run_day4_profile.sh conservative Q_slosh:=10
```

适用场景：
- Day 4 仿真参数 sweep，快速切换对照组
- 复现指定参数组的仿真 bag

## 推荐用法

如果目标是做“固定路径下的 Q0/Q5 anti-slosh 对照实验”，推荐顺序如下：

1. 用实时规划先跑一次，并用 `fixed_global_path_runner.py` 的 `capture` 模式保存第一次 `/scout/global_path`
2. 用 `send_fixed_goal.py` 或已有导航链将机器人移回固定起点
3. 用 `fixed_global_path_runner.py` 的 `replay` 模式发布固定路径
4. 用 `record_slosh_experiment.sh` 录制 Q0 和 Q5 两组 bag
5. 用 `extract_slosh_metrics.py` 做离线对比分析
