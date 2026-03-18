# IMU 融入步骤

## 0. 目标与边界

本文的目标不是“把所有 `slosh_use_imu_*` 一次性全打开”，而是按风险从低到高，逐步把 IMU 融入 `scout_local_planner`。

当前项目里，IMU 接口的使用方式是：

- `linear_acceleration.y`
- `angular_velocity.z`
- `angular_velocity.z` 差分得到的 `alpha_z`

当前代码只做了 EMA 滤波，没有做下面这些处理：

- TF 旋转
- 重力补偿
- 静止零偏扣除

所以融入顺序必须是：

1. 先融入 `yaw_rate`
2. 再验证并处理 `lateral_accel`
3. 最后再决定是否启用 `alpha_z`

当前结论：

- IMU 厂家层 bring-up 已完成
- `/imu/data` 已稳定发布
- 当前实机频率约 `50 Hz`
- 当前波特率 `115200`
- 当前安装方向已知：`x` 朝车头
- `slosh_use_imu_yaw_rate:=true` 已经验证过“真的进了 slosh 链路”
- `slosh_use_imu_lateral_accel:=true` 目前还不建议正式启用
- `slosh_use_imu_alpha_z:=true` 目前还不建议正式启用

---

## 1. 总原则

### 1.1 一次只改一件事

每一轮实验只允许改下面其中一类：

- IMU 开关
- near-goal / tracking 参数
- 液体模型参数

不要在同一轮里同时：

- 改终点停靠逻辑
- 改 `Q_slosh`
- 改 `slosh_use_imu_*`

否则 bag 很难解释。

### 1.2 优先用可复现控制

建议优先使用键盘或 ROS 发布 `/cmd_vel` 来做 IMU 融入验证。  
如果使用遥控器，务必同时录：

- `/scout_status`
- `/rs_status`

原因是遥控器模式下，动作不一定经过 `/cmd_vel`。

### 1.3 当前终点停靠缺陷不要混进 IMU 结论

当前终点停靠修正是外层状态机在目标容差区内直接发 `cmd_vel=0`，不是 MPC 在最后一段连续优化减速。  
这件事和 IMU 融入是两条线，不要把终点硬停带来的影响误判成 IMU 问题。

---

## 2. 阶段 0：上位机 IMU 基线确认

### 2.1 启动顺序

终端 1：

```bash
roslaunch scout_bringup scout_mini_robot_base.launch
```

终端 2：

```bash
roslaunch scout_bringup scout_imu.launch
```

如果 `udev` 还没修好，临时用：

```bash
roslaunch scout_bringup scout_imu.launch port:=/dev/ttyUSB0 baud:=115200
```

### 2.2 通过标准

执行：

```bash
rostopic list | grep imu
rostopic echo -n1 /imu/data
rostopic hz /imu/data
```

应满足：

- `/imu/data` 存在
- `frame_id` 正常
- 频率约 `50 Hz`
- 静止时 `angular_velocity.z` 接近 `0`
- 静止时 `linear_acceleration.z` 接近 `+9.8`

### 2.3 坐标约定

后续所有判断统一按车体坐标系：

- `x` 正方向：车头朝前
- `y` 正方向：车体左侧
- `z` 正方向：竖直向上

对应符号：

- 左转：`angular_velocity.z > 0`
- 右转：`angular_velocity.z < 0`
- 左转弧线：`linear_acceleration.y > 0`
- 右转弧线：`linear_acceleration.y < 0`

---

## 3. 阶段 1：最小融入，只开 IMU yaw rate

这是当前推荐首先落地的配置。

### 3.1 启动命令

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  Q_slosh:=5 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

### 3.2 参数确认

执行：

```bash
rosparam get /scout_local_planner/slosh_estimator/use_imu_yaw_rate
rosparam get /scout_local_planner/slosh_estimator/use_imu_lateral_accel
rosparam get /scout_local_planner/slosh_estimator/use_imu_alpha_z
```

应看到：

- `use_imu_yaw_rate = true`
- `use_imu_lateral_accel = false`
- `use_imu_alpha_z = false`

### 3.3 在线确认 IMU yaw 是否真正融入

执行：

```bash
rostopic echo -n5 /slosh/omega_est_used
rostopic echo -n5 /slosh/imu_omega_z_filtered
```

判断标准：

- 两者应基本一致
- 且转弯时它不应长期完全等于 `/odom.twist.twist.angular.z`

这一步的意义是：确认 slosh 估计实际使用的是 IMU yaw，而不是“参数看起来开了，但内部仍在吃 odom”。

### 3.4 录包

建议在 `scout_local_planner` 包目录下执行：

```bash
cd $(rospack find scout_local_planner)
./scripts/record_slosh_experiment.sh 5 imu_yaw_only
```

建议动作：

1. 静止 5 秒
2. 缓慢直行 5 秒
3. 顺时针转弯或原地顺时针旋转
4. 静止 3 秒
5. 逆时针转弯或原地逆时针旋转
6. 静止 5 秒

### 3.5 阶段 1 通过标准

满足下面几条就算通过：

- `/slosh/omega_est_used` 与 `/slosh/imu_omega_z_filtered` 基本一致
- 其数值与 `/odom.twist.twist.angular.z` 存在可观测差异
- planner 无明显回归
- 不因为启用 IMU yaw 出现持续性 solver failure

如果通过，说明：

- IMU yaw 已经可以正式作为第一路实机输入保留

---

## 4. 阶段 2：验证 IMU lateral acceleration，不急着启用

这一阶段的目标不是立刻开 `slosh_use_imu_lateral_accel:=true`，而是先判断 `ay` 是否值得进入系统。

### 4.1 启动方式

仍然保持：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  Q_slosh:=5 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false
```

也就是：

- 系统先继续只用 IMU yaw
- `ay` 只观察，不正式喂给 slosh 估计器

### 4.2 专用动作

录一包专门的 `ay` 验证数据：

1. 静止 5 秒
2. 低速左弧线 5 秒
3. 静止 3 秒
4. 低速右弧线 5 秒
5. 静止 5 秒

### 4.3 重点观察

至少看这些量：

- `/imu/data/angular_velocity.z`
- `/imu/data/linear_acceleration.y`
- `/odom/twist/twist/angular.z`
- `/odom/twist/twist/linear.x`

### 4.4 判断标准

理想情况应满足：

- 左弧线时 `ay > 0`
- 右弧线时 `ay < 0`
- 静止时 `ay` 接近 `0`
- 静止均值不要明显漂

当前你前面的 bag 结论是：

- `ay` 左右符号基本对
- 但静止时仍有小偏置
- 并且这路加速度保留了重力投影

所以当前阶段的结论大概率仍然会是：

- `ay` 数据有价值
- 但不能直接裸用

---

## 5. 阶段 3：给 IMU ay 加最小预处理

这一步开始才考虑正式融入 `lateral_accel`。

### 5.1 最小处理目标

先不上完整重力补偿，只做最小可用版：

1. 实验开始前静止 3 到 5 秒
2. 估计 `ay` 静止零偏
3. 在线减去该零偏
4. 再做 EMA 低通

### 5.2 通过标准

加了预处理后，再重复阶段 2 的动作。  
如果满足下面条件，才进入下一步：

- 静止时 `ay` 更接近 `0`
- 左右弧线符号仍稳定正确
- 直线段 `ay` 不再长期偏在某一侧

### 5.3 通过后再启用

通过后才开始试：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  Q_slosh:=5 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=true \
  slosh_use_imu_alpha_z:=false
```

### 5.4 阶段 3 失败时的处理

如果 `ay` 仍然明显受重力投影和姿态变化影响，就不要硬开 `slosh_use_imu_lateral_accel`。  
此时保留：

- `yaw_rate = true`
- `lateral_accel = false`

这仍然是一个有效的工程版本。

---

## 6. 阶段 4：验证 IMU alpha_z，不急着正式启用

`alpha_z` 是由 `angular_velocity.z` 差分得到的，比 `yaw_rate` 更敏感、更容易抖。

### 6.1 为什么放在最后

虽然现在 IMU 已经是 `50 Hz`，比之前 `10 Hz` 好很多，但 `alpha_z` 本质上还是差分量。  
当前代码路径是：

1. 读取 `omega_z_raw`
2. 用相邻帧做差分得到 `alpha_raw`
3. 再做 EMA

所以它天然比 `yaw_rate` 更脆弱。

### 6.2 验证动作

建议录一包转向起停数据：

1. 静止 5 秒
2. 原地左转起转
3. 左转停下
4. 静止 3 秒
5. 原地右转起转
6. 右转停下
7. 静止 5 秒

### 6.3 判断标准

主要看：

- 静止时 `alpha_z` 是否接近 `0`
- 起转/停转时是否有合理尖峰
- 是否出现过大的噪声尖峰
- 启用后是否引入明显 solver failure 或控制抖动

### 6.4 启用命令

只有通过后才试：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  Q_slosh:=5 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=true \
  slosh_use_imu_alpha_z:=true
```

如果 `alpha_z` 仍然偏噪，就维持：

- `yaw_rate = true`
- `lateral_accel = true`
- `alpha_z = false`

这也是可接受的正式版本。

---

## 7. 什么时候才算“全面融入”

当前项目里，“全面融入 IMU”不等于“三个开关必须同时为 true”。  
更合理的定义是：

1. IMU yaw 已稳定替代 odom yaw 参与 slosh 估计
2. IMU lateral accel 在经过最小预处理后可稳定使用
3. IMU alpha_z 只有在确实干净时才启用

也就是说，最终可能出现两种正式形态：

### 7.1 稳妥正式版

- `slosh_use_imu_yaw_rate:=true`
- `slosh_use_imu_lateral_accel:=true`
- `slosh_use_imu_alpha_z:=false`

### 7.2 完整实验版

- `slosh_use_imu_yaw_rate:=true`
- `slosh_use_imu_lateral_accel:=true`
- `slosh_use_imu_alpha_z:=true`

如果 `alpha_z` 一直噪声大，不要为了形式上的“全开”硬上。

---

## 8. 每一阶段都要录的关键话题

建议每次都录：

- `/imu/data`
- `/wit/mag`
- `/odom`
- `/cmd_vel`
- `/scout_status`
- `/rs_status`
- `/slosh/state`
- `/slosh/height`
- `/slosh/height_pred_max`
- `/slosh/ay_est`
- `/slosh/alpha_est`
- `/slosh/omega_est_used`
- `/slosh/imu_omega_z_filtered`
- `/mpc/solve_ms`
- `/mpc/status_val`
- `/mpc_status`
- `/tf`
- `/tf_static`

推荐命令：

```bash
cd $(rospack find scout_local_planner)
./scripts/record_slosh_experiment.sh 5 step_test
```

---

## 9. 当前推荐执行顺序

按下面顺序做，不要跳步：

1. 先固定阶段 1：只开 `yaw_rate`
2. 做一轮 A/B 对照 bag：`yaw_rate=false` vs `yaw_rate=true`
3. 做阶段 2：专门录左弧线 / 右弧线数据，判断 `ay`
4. 如果 `ay` 值得用，再实现最小零偏扣除
5. 实现后再做阶段 3：尝试 `lateral_accel=true`
6. 最后单独验证 `alpha_z`

---

## 10. 当前不该做的事

- 不要现在就把三个 `slosh_use_imu_*` 一次性全打开
- 不要在做 IMU 融入验证时同时改终点停靠参数
- 不要在同一轮里同时改 `Q_slosh`、`v_max`、tracking 参数和 IMU 开关
- 不要因为 `yaw_rate` 已经可用，就默认 `ay` 和 `alpha_z` 也一定可用

---

## 11. 当前一句话结论

当前最稳的推进路线是：

- 先把 IMU yaw 正式保留进系统
- 再把 IMU ay 做成“带零偏扣除的可用输入”
- 最后再决定 IMU alpha_z 是否值得进入正式版
