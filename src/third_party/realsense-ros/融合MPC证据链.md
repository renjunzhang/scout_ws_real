# RealSense 融合 MPC 证据链测试步骤

## 目录

- 1. 当前目标
- 2. 当前链路定位
- 3. 当前相关文件
- 4. 现场录包前检查
- 5. 当前推荐测试顺序
- 6. 当前推荐录制内容
- 7. 录包后的离线分析
- 8. 当前测试步骤
- 9. 结果存放建议
- 10. 注意事项
- 11. 当前建议的一句话

## 1. 当前目标

当前目标不是把 RealSense 直接闭环接入 `scout_local_planner`，而是先建立一条**实车外部证据链**：

- 模型侧：
  - `/slosh/height`
  - `/slosh/height_pred_max`
- 视觉侧：
  - RealSense 原始 RGB 图像
  - 后处理得到的 `height_peak_rel_px`

因此当前测试的本质是：

- **同一路线、同一液体状态、同一相机位置**
- 比较：
  - `Q_slosh=0`
  - `Q_slosh=5`
- 然后离线分析：
  - 模型估计峰值
  - 视觉液面峰值

## 2. 当前链路定位

当前证据链分两部分：

### 2.1 现场录包阶段

现场只负责录：

- `/slosh/*` 模型估计与实验上下文
- `/camera/color/image_raw`
- `/camera/color/camera_info`
- 以及底盘/IMU/路径相关话题

### 2.2 离线分析阶段

录包结束后再离线做：

- 从 bag 提取 RealSense 图像
- 用 `realsense_liquid_measurement` 跑出视觉液面曲线
- 用 `extract_slosh_metrics.py` 提取模型侧指标
- 再做视觉与模型对比

所以当前主线是：

- **录原始 bag**
- **离线重算视觉液面**
- **离线对比模型估计**

而不是：

- 现场实时把 `/liquid_measurement/*` 接进控制闭环

## 3. 当前相关文件

### 3.1 录包脚本

- [record_slosh_experiment.sh](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh)

当前脚本已经包含：

- `/slosh/height`
- `/slosh/height_pred_max`
- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/imu/data`
- `/odom`
- `/cmd_vel`
- `/local_path`
- `/scout/global_path`
- 以及预留的 `/liquid_measurement/*`

### 3.2 目标发送脚本

- [send_fixed_goal.py](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/send_fixed_goal.py)

作用：

- 用固定目标点复现实验路线
- 保证 `Q_slosh=0` 和 `Q_slosh=5` 尽量一致

### 3.3 模型侧指标提取脚本

- [extract_slosh_metrics.py](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py)

作用：

- 从 bag 提取 `/slosh/height`、`/slosh/height_pred_max` 等统计量

### 3.4 视觉侧离线脚本

- [extract_liquid_height_from_bag.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py)

当前主标定：

- [frame_000000_calibration_line_auto_zero_peak.yaml](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration_line_auto_zero_peak.yaml)

## 4. 现场录包前检查

录制前至少确认下面这些：

### 4.1 RealSense 话题是否存在

```bash
rostopic list | grep /camera/color
```

至少应看到：

- `/camera/color/image_raw`
- `/camera/color/camera_info`

如果你现场实际 namespace 不是这个，而是例如：

- `/camera/camera/color/image_raw`

那就必须先改录包脚本，否则图像不会录进去。

### 4.2 MPC anti-slosh 链是否存在

```bash
rostopic list | grep /slosh/
```

至少应看到：

- `/slosh/height`
- `/slosh/height_pred_max`

### 4.3 路径和控制链是否正常

至少确认：

- `/scout/global_path`
- `/local_path`
- `/odom`
- `/cmd_vel`

### 4.4 RealSense 画面检查

至少人工确认：

- 试管完整入镜
- 液面区域清晰
- 画面没有严重反光遮住液面
- 相机位置固定，不会在 A/B 实验中被碰动

## 5. 当前推荐测试顺序

### 5.1 启动局部规划实验

基线：

```bash
roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=0
```

实验组：

```bash
roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=5
```

当前默认安全配置已经在 launch 内处理为：

- `slosh_use_imu_yaw_rate=true`
- `slosh_use_imu_lateral_accel=false`
- `slosh_use_imu_alpha_z=false`

### 5.2 启动录包

```bash
cd /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts
./record_slosh_experiment.sh 0 run1
```

或：

```bash
./record_slosh_experiment.sh 5 run1
```

默认输出目录优先是：

- `/data/$USER/slosh_bags`

### 5.3 发送固定目标

用固定目标保证 A/B 路线尽量一致，例如：

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/send_fixed_goal.py \
  --goal-topic /scout/goal \
  --frame map \
  --x 1.0 \
  --y 0.0 \
  --yaw 0.0
```

这里的 `x/y/yaw` 要换成你当前实际测试点。

### 5.4 每组建议重复

建议每组至少：

- `3 ~ 5` 次

当前建议最先做的 A/B：

- `Q_slosh=0`
- `Q_slosh=5`

先不要一上来叠太多变量。

## 6. 当前推荐录制内容

当前 bag 里建议至少要有：

### 6.1 视觉原始证据

- `/camera/color/image_raw`
- `/camera/color/camera_info`

### 6.2 模型估计证据

- `/slosh/height`
- `/slosh/height_pred_max`
- `/slosh/q_slosh_eta`

### 6.3 运动上下文

- `/odom`
- `/cmd_vel`
- `/imu/data`
- `/local_path`
- `/scout/global_path`
- `/tf`
- `/tf_static`

## 7. 录包后的离线分析

### 7.1 先提模型侧指标

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  /data/a/slosh_bags/your_bag.bag
```

### 7.2 再提视觉侧液面曲线

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py \
  --bag /data/a/slosh_bags/your_bag.bag \
  --calibration /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration_line_auto_zero_peak.yaml \
  --out-dir /data/a/realsense_validation/verify/motion/your_bag
```

输出重点看：

- `liquid_height.csv`
- `liquid_height_peak_curve.png`

### 7.3 当前最关心的对比量

视觉侧：

- `height_peak_rel_px`

模型侧：

- `/slosh/height`
- `/slosh/height_pred_max`

当前建议先比较：

- 最大峰值
- `p90 / p95`
- 曲线形状是否同步
- 有无 anti-slosh 时峰值是否整体下降

## 8. 当前测试步骤

### 8.1 第 0 步：确认当前主标定和输出目录

当前主标定：

- [frame_000000_calibration_line_auto_zero_peak.yaml](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration_line_auto_zero_peak.yaml)

当前推荐离线输出目录：

- 候选标定：
  - `/data/a/realsense_validation/candidates/`
- 静止验证：
  - `/data/a/realsense_validation/verify/static/`
- 运动验证：
  - `/data/a/realsense_validation/verify/motion/`

### 8.2 第 1 步：启动实验与相机

先保证：

- `slosh_experiment.launch` 正常启动
- RealSense 正常发 `/camera/color/image_raw`
- 试管、背景板、相机位置已经固定

### 8.3 第 2 步：开始录包

在录包终端运行：

```bash
cd /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts
./record_slosh_experiment.sh 0 run1
```

或：

```bash
./record_slosh_experiment.sh 5 run1
```

这里：

- 第一个参数对应 `Q_slosh`
- 第二个参数是本次运行标签

### 8.4 第 3 步：发送固定目标

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/send_fixed_goal.py \
  --goal-topic /scout/goal \
  --frame map \
  --x 1.0 \
  --y 0.0 \
  --yaw 0.0
```

注意：

- A/B 两组必须尽量使用同一个目标
- 路线、起点和液位状态尽量保持一致

### 8.5 第 4 步：结束录包并记录实验条件

每次录完后，至少额外记录：

- `Q_slosh`
- 运行编号
- 是否成功到达目标
- 是否出现明显碰撞/急停/人工干预
- 录制时的液位是否明显变化
- 是否有相机被碰动、试管被碰动、背景改变

### 8.6 第 5 步：先做模型侧离线分析

```bash
python3 /home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  /data/a/slosh_bags/your_bag.bag
```

### 8.7 第 6 步：再做视觉侧离线分析

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py \
  --bag /data/a/slosh_bags/your_bag.bag \
  --calibration /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration_line_auto_zero_peak.yaml \
  --out-dir /data/a/realsense_validation/verify/motion/your_bag \
  --skip-debug-video
```

如果需要检查叠加效果，再去掉 `--skip-debug-video`。

### 8.8 第 7 步：查看视觉曲线

重点看：

- `liquid_height.csv`
- `liquid_height_peak_curve.png`

当前曲线图里：

- 浅灰色：`valid peak_rel_px`
- 彩色：`reported peak_rel_px`
- 灰色 `x`：检测到了，但被 `accept_for_peak_report` 拒绝

### 8.9 第 8 步：做 A/B 对比

当前建议最先对比：

- `Q_slosh=0`
- `Q_slosh=5`

优先看：

- 视觉侧 `height_peak_rel_px`
- 模型侧 `/slosh/height`
- 模型侧 `/slosh/height_pred_max`

当前最有价值的比较量：

- 最大峰值
- `p90 / p95`
- 峰值出现时刻
- 曲线整体是否同步

## 9. 结果存放建议

### 9.1 不要把正式验证结果放在 `/tmp`

当前推荐：

- bag 原始文件：
  - `/data/a/slosh_bags/`
- 候选标定：
  - `/data/a/realsense_validation/candidates/`
- 静止验证结果：
  - `/data/a/realsense_validation/verify/static/`
- 运动验证结果：
  - `/data/a/realsense_validation/verify/motion/`

### 9.2 候选标定不要直接覆盖主标定

正确流程是：

1. 先把候选 auto-zero 输出写到候选目录
2. 先跑静止 bag
3. 再跑运动 bag
4. 只有两边都不过度退化，才升级成主标定

原因：

- 已经出现过“静止包更好，但运动包明显变差”的失败案例

## 10. 注意事项

### 10.1 当前不要只录 `/liquid_measurement/*`

因为当前视觉算法还在继续改进，所以：

- **原始图像一定要录**

这样后面算法变了，你还能重跑同一包。

### 10.2 当前不要把视觉链当闭环真值

当前视觉链的定位仍然是：

- 外部证据链

而不是：

- 已经可以直接闭环替代模型状态

### 10.3 当前不要把“px 还没变成 mm”误当成主问题

现在最主要的问题是：

- 检测质量
- 假峰抑制
- `accept_for_peak_report`

而不是：

- 还没换成毫米单位

### 10.4 A/B 实验必须保持下面这些不变

- 试管位置
- RealSense 位置
- 背景
- 光照
- 黑墨水浓度
- 液位高度
- 目标点和路线

否则视觉峰值变化就不一定来自控制策略。

### 10.5 当前 `/liquid_measurement/*` 可能为空

录包脚本已经加了 `/liquid_measurement/*`，但如果现场没有启动对应节点：

- bag 里不会有这些话题

这不影响当前主线，因为：

- 你仍然有 `/camera/color/image_raw`
- 可以回放后离线重算视觉液面

### 10.6 运动包里“掉帧”不等于没测到

当前 `liquid_height_peak_curve.png` 里如果看到曲线断开，常见原因不是图像没读到，而是：

- `valid=0`
- 或 `valid=1` 但 `accept_for_peak_report=0`

也就是说：

- 一部分帧是被质量门槛主动拒绝
- 不是简单的“测不到”

### 10.7 静止包仍有小幅波动是检测噪声，不是单位问题

当前静止包里的小幅波动主要来自：

- 透明圆柱试管的反光和折射
- 表观液面边界不够锐利
- 单目侧视检测噪声

这类波动不会因为把 `px` 换成 `mm` 自动消失。

## 11. 当前建议的一句话

**现在就可以在实车上录制“RealSense 原始图像 + `/slosh/height` / `/slosh/height_pred_max` + 运动上下文”的对比 bag；现场录原始证据，回去离线重算视觉液面，再与 MPC 模型估计做对比。**
