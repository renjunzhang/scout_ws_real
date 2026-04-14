# `scripts` 目录说明

本目录存放 `scout_local_planner` 相关的实验辅助脚本，主要用于固定目标/固定路径实验、录包、离线指标提取，以及 IMU 标定验证。

## 脚本列表

### `send_fixed_goal.py`

向指定 ROS 话题发布一个固定的 `geometry_msgs/PoseStamped` 目标点，默认发到 `/scout/goal`。

适用场景：
- 手动触发一次固定终点跟踪
- 做对照实验时保证 goal pose 一致

### `fixed_global_path_runner.py`

固定 `/scout/global_path` 的采集与回放工具，支持以下模式：
- `capture`：抓取第一次收到的全局路径并保存到 JSON
- `replay`：从 JSON 读取固定路径并持续发布
- `capture_and_replay`：先抓取再回放
- `goal_only`：只发布已保存路径的起点或终点 goal

适用场景：
- 先记录第一次规划出的路径，再让 Q0/Q5 都复用同一条 `/scout/global_path`
- 正式 tracking 前，先将机器人移回固定路径起点并对齐，再开始局部跟踪

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

### `extract_slosh_metrics.py`

从 bag 文件中提取 slosh/MPC 实验指标的离线分析脚本，默认聚焦 `TRACKING` 阶段。

可提取内容包括：
- `/slosh/height`、`/slosh/height_pred_max` 的峰值和统计量
- `/mpc/solve_ms`、`/mpc/status_val` 的求解性能
- governor、约束触发、速度命令等运行指标

适用场景：
- 比较不同参数组下的晃动指标和控制开销
- 批量导出 CSV 做进一步统计

### `observe_terminal_recovery.py`

终点恢复行为观察脚本，用于在线或回放时判断机器人在终点附近是否进入终点恢复逻辑，并输出阶段性状态。

关注内容包括：
- 当前位置与 goal 的相对关系
- 是否处于终点对点接近、终点朝向对齐等恢复阶段
- `/cmd_vel` 与 `/odom` 是否符合预期恢复行为

适用场景：
- 分析 near-goal/terminal recovery 行为是否合理
- 排查“到点后转不正”“终点附近抖动”之类问题

### `run_imu_stage2_sequence.py`

Stage-2 IMU `a_y` 验证动作脚本，向 `/cmd_vel` 发布一组固定的低速动作序列。

典型序列：
- 静止
- 左转弧线
- 静止
- 右转弧线
- 静止

适用场景：
- 采集 IMU 横向加速度 `a_y` 标定 bag
- 为 `analyze_imu_ay_stage2.py` 提供标准化输入

### `analyze_imu_ay_stage2.py`

Stage-2 IMU 横向加速度质量离线分析脚本。

主要功能：
- 读取 bag 中的 IMU、里程计和 `/slosh/imu_ay_*` 话题
- 自动识别静止、直行、左转、右转等区段
- 评估 bias、滤波输出和可用性结论

适用场景：
- 判断 IMU `a_y` 是否已经达到可接入控制器的质量
- 给 Stage-2 横向加速度链路做验收

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

## 推荐用法

如果目标是做“固定路径下的 Q0/Q5 anti-slosh 对照实验”，推荐顺序如下：

1. 用实时规划先跑一次，并用 `fixed_global_path_runner.py` 的 `capture` 模式保存第一次 `/scout/global_path`
2. 用 `send_fixed_goal.py` 或已有导航链将机器人移回固定起点
3. 用 `fixed_global_path_runner.py` 的 `replay` 模式发布固定路径
4. 用 `record_slosh_experiment.sh` 录制 Q0 和 Q5 两组 bag
5. 用 `extract_slosh_metrics.py` 做离线对比分析
