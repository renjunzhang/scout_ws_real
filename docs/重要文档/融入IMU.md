# IMU 融入步骤

## 目录

- 0. 目标与边界
  - 0.0 当前 IMU 外参状态必须先说明白
  - 0.1 当前零偏心配置下，IMU 三路输入的真实作用边界
  - 0.2 后续若试管不在机体中心，修改方案应这样做
- 1. 总原则
  - 1.1 一次只改一件事
  - 1.2 优先用可复现控制
  - 1.3 当前终点停靠缺陷不要混进 IMU 结论
- 2. 阶段 0：上位机 IMU 基线确认
  - 2.1 启动顺序
  - 2.2 通过标准
  - 2.3 坐标约定
- 3. 阶段 1：最小融入，只开 IMU yaw rate
  - 3.1 启动命令
  - 3.2 参数确认
  - 3.3 在线确认 IMU yaw 是否真正融入
  - 3.4 录包
  - 3.5 阶段 1 通过标准
  - 3.6 当前阶段 1 实测结论（2026-03-18）
- 4. 阶段 2：验证 IMU lateral acceleration，不急着启用
  - 4.1 启动方式
  - 4.1.1 推荐测试方式
  - 4.1.2 具体建议
  - 4.1.3 为什么不推荐“相同起终点”
  - 4.2 专用动作
  - 4.3 重点观察
  - 4.3.1 离线快检
  - 4.4 判断标准
- 5. 阶段 3：给 IMU ay 加最小预处理
  - 5.1 最小处理目标
  - 5.1.1 当前实现状态
  - 5.1.2 推荐验证方式
  - 5.2 通过标准
  - 5.2.1 当前阶段 3 实测结论（2026-03-18）
  - 5.2.2 当前阶段 3 第二轮修正方向（2026-03-18）
  - 5.2.3 当前阶段 3 复测结论（2026-03-18）
  - 5.3 通过后再启用
  - 5.4 阶段 3 失败时的处理
- 6. 阶段 4：验证 IMU alpha_z，不急着正式启用
  - 6.1 为什么放在最后
  - 6.2 验证动作
  - 6.3 判断标准
  - 6.4 启用命令
- 7. 什么时候才算“全面融入”
  - 7.1 稳妥正式版
  - 7.2 完整实验版
- 8. 每一阶段都要录的关键话题
- 9. 当前推荐执行顺序
- 10. 当前不该做的事
- 11. 当前一句话结论

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
- 阶段 1（只融入 `yaw_rate`）已完成
- `slosh_use_imu_yaw_rate:=true` 已经验证过“真的进了 slosh 链路”
- `slosh_use_imu_lateral_accel:=true` 目前还不建议正式启用
- `slosh_use_imu_alpha_z:=true` 目前还不建议正式启用

### 0.0 当前 IMU 外参状态必须先说明白

当前 IMU 外参**不是已经完整标定并被 planner 使用了**，而是处于下面这个状态：

- 如果直接启动 `scout_imu.launch`，厂家驱动只是在消息头里写 `frame_id`
- 这不等于 IMU 外参已经标定完成
- 真正用于表达 `base_link -> imu_link` 关系的入口，是：
  - `scout_bringup/launch/scout_imu_with_tf.launch`
- 当前这组外参参数在该 launch 中配置：
  - `imu_x`
  - `imu_y`
  - `imu_z`
  - `imu_roll`
  - `imu_pitch`
  - `imu_yaw`

当前默认值仍应理解为：

- **粗测值**
- 用于先把 TF 语义、RViz 可视化和 frame 关系理顺
- **不代表已经完成高精度 IMU 外参标定**

更关键的是：

- 当前 `scout_local_planner` 还没有把这组 IMU 平移外参真正用于 `ay` 杠杆臂补偿
- 所以“IMU 话题接入成功”不等于“IMU 外参已在控制里生效”

因此在看后续所有 IMU 融入结论时，必须记住：

1. 当前已确认的是 IMU 话题、频率、符号和部分 frame 语义
2. 当前尚未完成的是 IMU 外参的精细标定与控制级补偿生效
3. 如果后续要把 `IMU ay` 作为正式输入长期保留，IMU 外参标定优先级高于 `alpha_z`

### 0.1 当前零偏心配置下，IMU 三路输入的真实作用边界

当前运行配置里，试管偏心参数还是：

- `slosh/offset_x = 0.0`
- `slosh/offset_y = 0.0`

这会带来一个很关键的工程事实：

- `LiquidSloshModel::update()` 里的旋转修正项会退化
- 因此 `alpha_z` 对当前 **modal state 更新**几乎没有实质作用
- `yaw_rate` 仍然会进入当前高度监测中的抛物面项
- 如果启用了 speed governor，`yaw_rate` 还会通过当前高度风险链路间接影响速度治理
- 真正直接进入当前 slosh 状态传播、会改变主模态演化的 IMU 通道，其实是 `linear_acceleration.y`

所以在当前 `offset_x = offset_y = 0` 的版本里，IMU 融入的优先级应理解为：

1. `yaw_rate` 值得保留，但不要高估它对主模态传播的贡献
2. `ay` 才是最值得投入预处理和标定精力的通道
3. `alpha_z` 可以继续保留验证入口，但短期不是最高优先级

### 0.2 后续若试管不在机体中心，修改方案应这样做

如果后续把试管安装到机体旋转中心之外，不能只改 `slosh/offset_x`、`slosh/offset_y` 就结束。  
要保证 **估计器和优化器使用同一套物理假设**，至少要同步做下面几件事：

1. 先量清楚试管中心相对机体旋转中心的几何关系，并写入：
   - `slosh/offset_x`
   - `slosh/offset_y`
2. 保持估计侧继续使用旋转修正：
   - `a_cx = a_x - alpha_z * r_y - omega_z^2 * r_x`
   - `a_cy = a_y + alpha_z * r_x - omega_z^2 * r_y`
3. 把同样的偏心旋转修正并入 `DiffDriveModel::predict()` 和 `DiffDriveModel::linearize()`，不能再只用 `a_y = v * omega`
4. 如果希望 `alpha_z` 在 MPC 预测域内也真正发挥作用，就要给预测侧提供一致的 `alpha_z` 近似，而不是只在在线估计侧使用它
5. 完成以上同步后，再重新评估：
   - `slosh_use_imu_alpha_z`
   - `slosh_use_imu_lateral_accel`
   - `Q_slosh` 的物理解释

否则会出现“估计器按偏心容器在算，优化器仍按中心容器在算”的不一致，论文和实验解释都会被污染。

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

如果你已经准备开始规范 IMU frame，推荐改用：

```bash
roslaunch scout_bringup scout_imu_with_tf.launch \
  imu_frame:=imu_link \
  imu_x:=0.13 \
  imu_y:=-0.13 \
  imu_z:=0.0 \
  imu_roll:=0.0 \
  imu_pitch:=0.0 \
  imu_yaw:=0.0
```

说明：

- 这一步的目标是把 `base_link -> imu_link` 的静态 TF 建起来
- 当前可以先接受“粗测值”
- 不要因为先有了静态 TF，就立刻把这些值拿去做 `ay` 杠杆臂补偿

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

### 3.6 当前阶段 1 实测结论（2026-03-18）

阶段 1 已完成。

本次用于判断的两包为：

- `/home/geist/slosh_bags/slosh_Q5_20260318_192838_imu_yaw_only_true.bag`
- `/home/geist/slosh_bags/slosh_Q5_20260318_193455_imu_yaw_only_flase.bag`

对应结论如下：

- `true` 包中：
  - `/slosh/omega_est_used` 与 `/slosh/imu_omega_z_filtered` 全程一致
  - 平均绝对误差约为 `0`
  - 最大绝对误差约为 `0`
  - 与 `/odom.twist.twist.angular.z` 存在可观测差异，平均绝对误差约 `0.037`，最大约 `0.276`
- `false` 包中：
  - `/slosh/omega_est_used` 更接近 odom yaw，而不是 IMU yaw
  - 与 `/odom.twist.twist.angular.z` 的平均绝对误差约 `0.010`
  - 与 `/slosh/imu_omega_z_filtered` 的平均绝对误差约 `0.230`

因此可以确认：

- `slosh_use_imu_yaw_rate:=true` 时，slosh 估计实际已经在使用 IMU yaw
- `slosh_use_imu_yaw_rate:=false` 时，slosh 估计仍然回退到 odom yaw
- 当前代码中的 yaw source 切换逻辑已经闭环打通

稳定性方面：

- `true` 包 `solver fail = 0`
- `false` 包 `solver fail = 3`

所以“启用 IMU yaw 后出现持续性 solver failure”这一条不成立，反而 `true` 包更稳定。

补充说明：

- `false` 包中确实出现了终点不收敛现象
- 但这不构成“阶段 1 失败”，原因是：
  - 阶段 1 的核心验收标准是“IMU yaw 是否真正融入 slosh 链路，以及是否引入明显回归”
  - 这两包并不是严格相同 goal / 相同路径的纯净 A/B
  - 当前终点收敛问题本身也属于 tracking / near-goal 逻辑问题，不应直接归因到 IMU yaw 是否融入

所以当前工程结论可以明确写为：

- 阶段 1 已通过
- `slosh_use_imu_yaw_rate:=true` 可以正式保留
- 下一步进入阶段 2：评估 `lateral_accel` 是否值得融入

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

### 4.1.1 推荐测试方式

这一阶段建议不要继续用 MPC 去“规划一条特殊路线”。

原因是：

- 阶段 2 的目标是验证原始 `imu ay` 的符号、偏置和可用性
- 如果继续走 tracking MPC，会把路径跟踪误差、前视距离、终点收敛、障碍物和定位误差一起混进来
- 这样即使 `ay` 看起来异常，也很难判断到底是 IMU 本身问题，还是轨迹执行问题

因此阶段 2 更推荐换成：

- 底盘直控测试
- 不要求“相同起终点”
- 只要求动作模式清晰、左右弧线持续时间足够、速度尽量稳定

### 4.1.2 具体建议

推荐启动：

1. 底盘：

```bash
roslaunch scout_bringup scout_mini_robot_base.launch
```

2. IMU：

```bash
roslaunch scout_bringup scout_imu.launch
```

3. 录包：

```bash
cd $(rospack find scout_local_planner)
./scripts/record_slosh_experiment.sh 5 stage2_ay
```

4. 动作序列：

```bash
cd $(rospack find scout_local_planner)
python3 scripts/run_imu_stage2_sequence.py
```

该脚本默认执行：

- 静止 `5 s`
- 左弧线 `5 s`
- 静止 `3 s`
- 右弧线 `5 s`
- 静止 `5 s`

默认速度为：

- `linear.x = 0.30 m/s`
- `angular.z = ±0.30 rad/s`

如需更保守，可改为：

```bash
python3 scripts/run_imu_stage2_sequence.py --linear 0.20 --omega 0.20
```

### 4.1.3 为什么不推荐“相同起终点”

这个阶段不需要用“相同起终点”来约束动作。

原因是：

- 阶段 2 不是在比路径跟踪性能
- 而是在看：
  - 左转时 `ay` 是否主要为正
  - 右转时 `ay` 是否主要为负
  - 静止时是否接近 `0`
  - 直线时是否长期偏在某一侧

所以这一步的关键是“动作标签清楚”，不是“轨迹几何闭环”。

如果你一定想让轨迹大致回到原地，也建议用“左弧线 + 右弧线”自然回摆，而不是专门用 MPC 去做相同起终点导航。

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

### 4.3.1 离线快检

录完 bag 后，可直接跑：

```bash
cd $(rospack find scout_local_planner)
python3 scripts/analyze_imu_ay_stage2.py /path/to/your_stage2_ay.bag
```

脚本会直接给出：

- 静止段 `imu_ay` 均值/方差
- 左转段 `imu_ay` 符号是否主要为正
- 右转段 `imu_ay` 符号是否主要为负
- `imu_ay` 与 `v * omega` 的相关性
- 一个简短结论：是“可用但需预处理”，还是“当前还不适合直接融入”

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

### 5.1.1 当前实现状态

当前代码已经补上阶段 3 的最小预处理，并进一步改成“第一段静止窗口 + 稳健估计”：

- 只使用启动后的第一段静止窗口估计 `imu_ay_bias`
- 在该静止窗口内，先对 `ay_raw` 做一层专用于 bias 估计的 EMA
- 再对这批 EMA 后样本做 trimmed mean，避免直接用 `ay_raw` 普通均值
- 第一段静止结束前只锁定一次 bias，后续不再重开 bias 估计
- 零偏锁定后，对 `linear_acceleration.y` 先减去 `imu_ay_bias`
- 再进入现有的在线 EMA 低通

当前默认参数为：

- `slosh_estimator/imu_ay_bias_compensation_enable: true`
- `slosh_estimator/imu_ay_bias_init_duration: 3.0`
- `slosh_estimator/imu_ay_bias_static_v_max: 0.03`
- `slosh_estimator/imu_ay_bias_static_omega_max: 0.03`
- `slosh_estimator/imu_ay_bias_min_samples: 100`
- `slosh_estimator/imu_ay_bias_estimator_alpha: 0.15`
- `slosh_estimator/imu_ay_bias_trim_ratio: 0.10`

新增调试话题：

- `/slosh/imu_ay_bias`
- `/slosh/imu_ay_filtered`
- `/slosh/imu_ay_bias_ready`

说明：

- `/slosh/imu_ay_bias` 表示当前锁定的静止零偏
- `/slosh/imu_ay_filtered` 表示“扣零偏后再 EMA”的 IMU ay
- `/slosh/imu_ay_bias_ready` 为 `1` 时表示零偏已经锁定完成

说明：

- 如果机器人在第一段静止窗口满足 `init_duration + min_samples` 后才开始运动，则会在“第一段静止结束到开始运动”的切换点锁定 bias
- 如果一上来就运动，或者第一段静止时间不足，则本次运行不再继续估计 bias

### 5.1.2 推荐验证方式

阶段 3 的验证不要直接让 planner 接管底盘去跑路径。  
更推荐让 planner 只做“观测和发布调试话题”，底盘仍由固定动作脚本驱动。

推荐方式：

1. 启动底盘和 IMU：

```bash
roslaunch scout_bringup scout_mini_robot_base.launch
roslaunch scout_bringup scout_imu.launch
```

2. 启动 planner，但把输出重定向到调试话题，避免抢 `/cmd_vel`：

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  Q_slosh:=5 \
  slosh_use_imu_yaw_rate:=true \
  slosh_use_imu_lateral_accel:=false \
  slosh_use_imu_alpha_z:=false \
  cmd_vel_topic:=/scout_local_planner/cmd_vel_debug
```

3. 录包：

```bash
cd $(rospack find scout_local_planner)
./scripts/record_slosh_experiment.sh 5 stage3_ay_bias
```

4. 继续用阶段 2 的固定动作脚本驱动底盘：

```bash
cd $(rospack find scout_local_planner)
python3 scripts/run_imu_stage2_sequence.py
```

5. 离线分析：

```bash
cd $(rospack find scout_local_planner)
python3 scripts/analyze_imu_ay_stage2.py /path/to/your_stage3_ay_bias.bag
```

如果 bag 中存在 `/slosh/imu_ay_filtered`，脚本会额外输出“扣零偏后”的统计结果。

### 5.2 通过标准

加了预处理后，再重复阶段 2 的动作。  
如果满足下面条件，才进入下一步：

- 静止时 `ay` 更接近 `0`
- 左右弧线符号仍稳定正确
- 直线段 `ay` 不再长期偏在某一侧

### 5.2.1 当前阶段 3 实测结论（2026-03-18）

用于判断的 bag：

- `/home/geist/slosh_bags/slosh_Q5_20260318_202134_stage3_ay_bias.bag`

当前结论：

- 零偏扣除方案是有效的
- 但还不建议立刻把 `IMU ay` 带进复杂导航场景
- 更合适的下一步是：
  - 先做一轮“小范围开启 `slosh_use_imu_lateral_accel:=true`”的专用动作验证
  - 不要直接上带目标点和避障风险的 tracking 场景

本次最关键的对比如下：

- 原始 `imu_ay`：
  - 静止均值约 `-0.0528`
  - 左转均值约 `+0.0182`
  - 右转均值约 `-0.1564`
  - 与 `v * omega` 的相关性约 `0.366`
- 扣零偏后的 `imu_ay_filtered`：
  - 静止均值约 `+0.0264`
  - 左转均值约 `+0.1062`
  - 右转均值约 `-0.0809`
  - 与 `v * omega` 的相关性约 `0.701`

这说明：

- 预处理后，左右转符号一致性明显变好
- 与运动学近似 `v * omega` 的一致性也明显增强
- 但静止段仍有小幅残余偏差，因此当前更适合先做受控试验，而不是直接在正式导航中启用

补充观察：

- `/slosh/imu_ay_bias_ready` 已稳定为 `1`
- `/slosh/imu_ay_bias` 锁定值约为 `-0.0795`
- 说明阶段 3 的“静止窗口估计 bias + 在线扣除”链路已经打通

因此当前工程判断可以写为：

- 阶段 3 预处理链路已打通
- 预处理效果明显优于原始 `imu_ay`
- 下一步进入：
  - `slosh_use_imu_lateral_accel:=true` 的小范围专用动作验证
  - 暂不直接进入复杂 MPC tracking 导航

### 5.2.2 当前阶段 3 第二轮修正方向（2026-03-18）

用于判断的 bag：

- `/home/geist/slosh_bags/slosh_Q5_20260318_202834_stage3_ay_enabled.bag`

这包说明两件事：

- `slosh_use_imu_lateral_accel:=true` 的链路已经切成功
  - `/slosh/ay_est` 已等于 `/slosh/imu_ay_filtered`
- 但旧版 bias 估计仍有“扣过头”现象
  - 原始静止 `imu_ay` 均值约 `-0.0571`
  - 当时锁定的 `/slosh/imu_ay_bias` 却约为 `-0.0994`
  - 结果是静止段被抬成正残差，右转幅值也被明显吃掉

因此当前阶段 3 已进一步修正为：

- bias 只允许从第一段静止窗口锁定一次
- 不再使用原始 `ay_raw` 的普通均值
- 改为“静止窗口内 EMA 后 ay + trimmed mean”的稳健估计

这一步之后，再重新录一包 `stage3_ay_enabled`，再决定 `slosh_use_imu_lateral_accel:=true` 是否可以进入正式场景。

### 5.2.3 当前阶段 3 复测结论（2026-03-18）

用于判断的 bag：

- `/home/geist/slosh_bags/slosh_Q5_20260318_205505_stage3_ay_enabled_v2.bag`

当前结论：

- 稳健 bias 估计修正是有效的
- 阶段 3 可以认为基本通过
- 但仍不建议立刻把 `slosh_use_imu_lateral_accel:=true` 作为默认正式配置
- 更合适的下一步是：
  - 进入开阔环境下的小范围导航 A/B 对照
  - 对比 `slosh_use_imu_lateral_accel:=false/true`

本次最关键的对比如下：

- 原始 `imu_ay`：
  - 静止均值约 `-0.0369`
  - 左转均值约 `+0.0166`
  - 右转均值约 `-0.1644`
  - 与 `v * omega` 的相关性约 `0.414`
- 稳健 bias 后的 `imu_ay_filtered`：
  - 静止均值约 `-0.0204`
  - 左转均值约 `+0.0515`
  - 右转均值约 `-0.1441`
  - 与 `v * omega` 的相关性约 `0.725`

这说明：

- 静止段残余偏差进一步收敛，更接近 `0`
- 左转信号仍保持为正，右转信号明显恢复为负
- 右转负值占比已经明显改善，说明上一包“扣过头”问题已基本修正

补充观察：

- `imu_ay_bias_ready_ratio` 约为 `0.639`
- 这不是异常，而是因为当前逻辑只在第一段静止结束时锁定一次 bias
- 从话题统计范围看，锁定后的 bias 约为 `-0.0317`

因此当前工程判断可以写为：

- `slosh_use_imu_lateral_accel` 链路可用
- 稳健 bias 估计优于旧版普通均值
- 阶段 3 已经具备进入低风险导航 A/B 测试的条件
- 在拿到导航 A/B 结果前，仍建议：
  - `slosh_use_imu_yaw_rate:=true`
  - `slosh_use_imu_lateral_accel:=false` 作为默认安全配置

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

另外还要明确一点：在当前 `slosh/offset_x = 0`、`slosh/offset_y = 0` 的默认配置下，`alpha_z` 对主模态状态传播的直接作用本来就很弱。  
所以阶段 4 的意义主要是“为后续偏心容器版本预留验证入口”，而不是说明它已经是当前版本里最关键的 IMU 通道。

### 6.2 验证动作

建议录一包转向起停数据：

1. 静止 5 秒
2. 原地左转起转
3. 左转停下
4. 静止 3 秒
5. 原地右转起转
6. 右转停下
7. 静止 5 秒

已经补了一个专用动作脚本：

- `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/run_imu_stage4_sequence.py`

直接运行即可：

```bash
cd $(rospack find scout_local_planner)
python3 scripts/run_imu_stage4_sequence.py
```

默认动作是：

- 静止 `5s`
- 原地左转 `2s`
- 左转停下 `2s`
- 静止 `3s`
- 原地右转 `2s`
- 右转停下 `2s`
- 静止 `5s`

默认角速度：

- `omega = 0.60 rad/s`

如果想保守一点，可以这样跑：

```bash
cd $(rospack find scout_local_planner)
python3 scripts/run_imu_stage4_sequence.py --omega 0.40
```

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
