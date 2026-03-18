# 变更记录 2

## 当前已知缺陷

- 终点停靠修正目前是通过 `LocalPlannerROS` 的 `goal_stop_pending_` 在外层状态机里直接发布 `cmd_vel = 0` 实现的，而不是由 MPC 在终点最后一段连续优化减速完成。
- 这不会破坏当前 MPC 的 8 维增广状态结构，也不会移除 `Q_slosh` 的液体晃动抑制代价项。
- 但它会让进入目标容差区后的最后一小段控制暂时绕开 MPC，因此终点最后一段的 anti-slosh 最优性会弱于“全程由 MPC 连续减速”的版本。
- 当前版本更偏向“先保证终点能停住、避免冲过头”的工程折中；如果后续要进一步兼顾终点收敛和液体残余晃动，应优先改 near-goal `v_ref` 连续衰减逻辑，而不是继续叠加硬停分支。

## 2026-03-14

### MBF 局部代价地图基础设施补齐

- 修改文件：[global_planner.yaml](/home/a/scout_ws/src/scout_apps/navigation/scout_global_planner/config/global_planner.yaml)、[global_planner_sim.yaml](/home/a/scout_ws/src/scout_apps/navigation/scout_global_planner/config/global_planner_sim.yaml)
- 在 `local_costmap` 中新增 `costmap_2d::ObstacleLayer`
- 障碍观测源统一使用激光话题 `/scan_front`
- 保留原有 `StaticLayer + InflationLayer` 结构，形成“静态地图 + 实时障碍 + 膨胀”的局部代价地图
- 当前该改动仅用于建立局部避障基础设施，`scout_local_planner` / `MPC` 仍未接入 costmap，控制逻辑继续只做 tracking

### 说明

- 这一步的目标是先把局部障碍感知链路、TF 和 costmap 更新机制跑通
- 后续如果要研究避障，可在此基础上继续接入：
  - 直接使用 `local_costmap`
  - 或从 `local_costmap` 再生成二维距离场/ESDF-like 场给 MPC 使用

### 28 mm 水试管的 slosh 参数初值

- 修改文件：[mpc_params.yaml](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params.yaml)、[slosh_params.yaml](/home/a/scout_ws/src/scout_apps/control/slosh_models/config/slosh_params.yaml)
- 目标容器：直径 `28 mm`、总高 `104 mm`、当前装液高度约 `55 mm`、液体为水
- 更新后的初值：
  - `container_radius = 0.014 m`
  - `liquid_height = 0.055 m`
  - `liquid_density = 1000.0 kg/m^3`
  - `damping_ratio = 0.12`
  - `mode_index = 1`
  - `offset_x = 0.0`
  - `offset_y = 0.0`
- 依据当前模型公式计算，这组几何参数对应的理论首模频率约为：
  - `omega_n ≈ 35.9 rad/s`
  - `f_n ≈ 5.72 Hz`
- 说明：
  - 当前代码可直接读取并使用 `container_radius / liquid_height / damping_ratio`
  - 当前代码尚无显式 `omega_eff` 覆写参数，首模频率仍由几何参数自动计算
  - 因此这次改动先把模型对齐到真实试管尺寸，再通过辨识修正 `damping_ratio`，必要时后续再新增 `omega_eff` 覆写接口

### 短辨识实验流程：拟合 `omega_eff` 和 `zeta_eff`

1. 实验准备
   - 使用当前这根 28 mm 水试管，液面高度保持在约 `55 mm`
   - 关闭 anti-slosh，确保实验时控制器不主动压晃动：`Q_slosh = 0`
   - 固定手机或相机侧拍试管，建议 `120 fps` 以上；在试管旁贴一把直尺或标定纸
   - 相机尽量正侧视，保证能看到液面最高点或明显对比标记

2. 激励方式
   - 先做纵向单脉冲实验：
     - 小车静止 `2 s`
     - 给一个短前进脉冲：目标可先用 `v = 0.20 ~ 0.25 m/s`，持续 `0.25 ~ 0.35 s`
     - 然后回到 `0`，记录至少 `5 s` 自由衰减
   - 每组重复 `5` 次，选波形最干净的 `3` 组做拟合
   - 如果后续要验证转弯激励，再补一组：
     - `v = 0.20 m/s`
     - 短时间施加 `omega = 0.6 ~ 0.8 rad/s`，持续 `0.3 ~ 0.5 s`
     - 然后回正，记录自由衰减

3. 数据提取
   - 从侧拍视频中提取液面代理量 `y(t)`：
     - 可直接取液面最高点像素高度
     - 或取液面边缘相对静止位置的位移
   - 对 `y(t)` 做零均值处理，得到振荡信号
   - 找到连续峰值 `A_1, A_2, A_3, ...` 及其时刻 `t_1, t_2, t_3, ...`

4. 参数拟合
   - 阻尼振荡周期：
     - `T_d = mean(t_{i+1} - t_i)`
     - `omega_d = 2*pi / T_d`
   - 对数衰减率：
     - `delta = mean(ln(A_i / A_{i+1}))`
   - 有效阻尼比：
     - `zeta_eff = delta / sqrt(4*pi^2 + delta^2)`
   - 有效固有频率：
     - `omega_eff = omega_d / sqrt(1 - zeta_eff^2)`

5. 更新策略
   - 先把拟合得到的 `zeta_eff` 直接写回 `damping_ratio`
   - 将 `omega_eff` 与理论值 `35.9 rad/s` 比较
   - 如果偏差在 `10%` 以内，可继续沿用当前几何公式
   - 如果偏差明显大于 `10%`，建议后续在代码中增加 `omega_eff` 覆写参数，而不是继续强行依赖几何公式

### 当前阶段性决策

- 当前暂无相机/视频链路，暂不执行液面视频辨识实验
- 现阶段先假设这根 `28 mm` 水试管在当前参数下可作为可用的低阶风险代理模型
- 近期工作重点不放在更换 slosh 模型结构，而是优先测试“融合液体晃动抑制后的 MPC”在实物/联调中的可用性
- 当前默认路线：
  - 保持现有 `4` 状态线性二维 MSD slosh 模型
  - 使用已更新的试管参数初值
  - 先验证 `Q_slosh`、box constraint、speed governor 接入后系统是否稳定、是否有可观测收益
- 后续待具备相机或其他可信液面观测后，再回到 `omega_eff / zeta_eff` 辨识与模型校正

### 增加 soft cost 调试话题

- 修改文件：[local_planner_ros.h](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/include/scout_local_planner/local_planner_ros.h)、[local_planner_ros.cpp](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp)、[record_slosh_experiment.sh](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh)、[topic_list.md](/home/a/scout_ws/docs/topic_list.md)
- 新增发布话题：`/slosh/q_slosh_eta`
- 语义：发布当前实际进入 QP 的液体软代价权重 `Q_slosh_eta`
- 目的：下一轮实验可直接在 bag 中确认 soft cost 是否真的生效，而不必只靠 launch 参数推断

### 当前 anti-slosh MPC 实验执行顺序

- 前置条件：
  - 已启动底盘、雷达、定位、MBF 全局规划
  - 确认 `/scout/global_path`、`/odom`、`/cmd_vel` 存在
  - 使用同一条测试路线，尽量保持每组实验路径一致

1. 组 A：基线（无晃动抑制）
   - 启动命令：
     - `roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=0`
   - 建议重复次数：
     - `3 ~ 5` 次
   - 重点观察话题：
     - `/mpc/status_val`
     - `/mpc/solve_ms`
     - `/cmd_vel`
     - `/local_path`
     - `/slosh/height_pred_max`
     - `/slosh/height`

2. 组 B：只开 slosh soft cost
   - 启动命令：
     - `roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=5`
   - 建议重复次数：
     - `3 ~ 5` 次
   - 重点观察话题：
     - `/mpc/status_val`
     - `/mpc/solve_ms`
     - `/cmd_vel`
     - `/slosh/height_pred_max`
     - `/slosh/height`
     - `/slosh/ax_est`
     - `/slosh/ay_est`

3. 组 C：soft cost + box constraint
   - 启动命令：
     - `roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=5 enable_slosh_box_constraint:=true`
   - 建议重复次数：
     - `3 ~ 5` 次
   - 重点观察话题：
     - `/mpc/status_val`
     - `/mpc/solve_ms`
     - `/cmd_vel`
     - `/slosh/height_pred_max`
     - `/slosh/constraint_active`
     - `/slosh/height`

4. 组 D：soft cost + box constraint + speed governor
   - 启动命令：
     - `roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=5 enable_slosh_box_constraint:=true slosh_speed_governor_enable:=true`
   - 建议重复次数：
     - `3 ~ 5` 次
   - 重点观察话题：
     - `/mpc/status_val`
     - `/mpc/solve_ms`
     - `/cmd_vel`
     - `/slosh/height_pred_max`
     - `/slosh/constraint_active`
     - `/slosh/v_des_eff`
     - `/slosh/speed_governor_active`
     - `/slosh/height`

- 每组实验结束后，优先比较以下现象：
  - 是否出现求解失败或控制发散
  - `/slosh/height_pred_max` 是否明显下降
  - `/cmd_vel` 是否变得过于保守或抖动
  - box constraint / governor 是否频繁触发
  - 到点成功率和轨迹跟踪是否明显退化

## 2026-03-16

### IMU 厂家层 bring-up 打通

- 新增文档：[配置IMU.md](/home/a/scout_ws/docs/配置IMU.md)
- 今日完成了 IMU 从“电脑端独立 bring-up”到“工控机端独立 bring-up”的整理与实测记录
- 电脑端已确认可直接通过厂家包 `wit_ros_imu` 跑出：
  - `/imu/data`
  - `/wit/mag`
- 初始问题定位：
  - 厂家默认 `imu_usb.rules` 只匹配 `10c4:ea60`
  - 当前实物 IMU 实际枚举为 `1a86:7523`（CH341 USB-Serial）
  - 因此 `/dev/imu_usb` 未自动生成，当前先以 `/dev/ttyUSB0` 跑通
- 电脑端最终验证结果：
  - `/imu/data` 正常输出
  - `/wit/mag` 正常输出
  - 最初频率约 `10 Hz`
- 后续在实物端已将 IMU 输出提升到：
  - `50 Hz`
  - `115200 baud`

### IMU 启动链路补齐

- 新增文件：[scout_imu.launch](/home/a/scout_ws/src/scout_ros/scout_bringup/launch/scout_imu.launch)
- 新增文件：[imu_only.launch](/home/a/scout_ws/src/wit_ros_imu/launch/imu_only.launch)
- 修改文件：[wit_normal_ros.py](/home/a/scout_ws/src/wit_ros_imu/scripts/wit_normal_ros.py)
- 修改文件：[package.xml](/home/a/scout_ws/src/scout_ros/scout_bringup/package.xml)
- 改动内容：
  - 为 `wit_ros_imu` 增加独立 `imu_only.launch`
  - 在 `scout_bringup` 中增加统一入口 `scout_imu.launch`
  - 驱动脚本支持通过参数配置 `frame_id`
  - `scout_bringup` 显式声明对 `wit_ros_imu` 的运行依赖
- 当前约定：
  - `port` 默认可传 `/dev/ttyUSB0`
  - `baud` 默认改为 `115200`
  - `frame_id` 可配置，当前默认仍为 `base_link`

### 实物流程与话题清单同步更新

- 修改文件：[change_log.md](/home/a/scout_ws/docs/change_log.md)
- 修改文件：[topic_list.md](/home/a/scout_ws/docs/topic_list.md)
- 改动内容：
  - 在实物流程中新增 `5.5 启动 IMU`
  - 明确 IMU 属于“实物 anti-slosh 实验建议单独启动”的链路
  - 记录当前实车完整话题清单
  - 补充 `/imu/data`、`/wit/mag` 的实车接入现状
  - 标注当前 IMU 频率已提升至 `50 Hz`

### 实验录包脚本补充 IMU 和底盘状态

- 修改文件：[record_slosh_experiment.sh](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh)
- 今日补录的话题：
  - `/imu/data`
  - `/wit/mag`
  - `/scout_status`
  - `/rs_status`
- 目的：
  - 实物端可同时回看 IMU 原始输入、底盘速度状态、遥控器状态
  - 避免遥控器控制时 `/cmd_vel` 全零导致后处理误判

### IMU 安装方向与数据质量阶段性结论

- 当前已确认 IMU 物理安装满足：
  - `x` 轴朝车头
- 当前项目内部使用的车体坐标约定继续保持：
  - `x` 前
  - `y` 左
  - `z` 上
- 两个实车 bag 的阶段性分析结论如下：

#### 1. `/home/a/下载/slosh_bags/imu_check_20260316_163517.bag`

- 动作包含：
  - 静止
  - 缓慢前进
  - 顺时针旋转约 `180°`
  - 静止
  - 加速前进
  - 逆时针旋转约 `180°`
- 结论：
  - `angular_velocity.z` 与 `odom.twist.twist.angular.z` 符号一致
  - 顺时针时 `omega_z < 0`
  - 逆时针时 `omega_z > 0`
  - `omega_z` 可视为当前最可信的一路 IMU 输入
  - 该包没有充分覆盖左/右弧线，因此尚不足以单独判定 `linear_acceleration.y` 可直接接入 planner

#### 2. `/home/a/下载/slosh_bags/imu_check_20260316_165703.bag`

- 动作包含：
  - 静止
  - 前进 + 顺时针持续转
  - 原地顺时针
  - 直行前进
  - 原地逆时针
  - 前进 + 逆时针持续转
- 结论：
  - 纯转向段中，`imu wz` 与 `odom angz` 相关性约 `0.999`
  - IMU 角速度幅值整体比 odom 稍大，约高 `8% ~ 10%`
  - 顺时针弧线段 `ay < 0`
  - 逆时针弧线段 `ay > 0`
  - 说明 `y` 轴方向与车体约定基本一致
- 但同时确认：
  - `linear_acceleration` 当前保留重力分量
  - 静止时 `ax` 长期存在约 `+0.2 m/s^2` 量级偏置
  - 静止时 `ay` 偏置会随姿态/地面条件变化
- 因此当前工程建议为：
  - `slosh_use_imu_yaw_rate` 可优先尝试
  - `slosh_use_imu_lateral_accel` 不建议直接裸开
  - `slosh_use_imu_alpha_z` 可在 `50 Hz` 条件下进一步评估，但不应先于 `yaw_rate` 验证

### 当前静止样本的解释

- 当前静止 `rostopic echo /imu/data` 样本呈现：
  - `angular_velocity.z ≈ 0`
  - `linear_acceleration.z ≈ 9.83`
  - `linear_acceleration.x ≈ 0.19`
  - `linear_acceleration.y ≈ 0`
- 解释：
  - `z` 轴朝上基本成立
  - IMU 工作正常
  - 加速度数据中包含重力
  - `x/y` 方向静止时仍存在小偏置，不能把当前 `linear_acceleration` 直接当作已补偿的纯线加速度

### 明天的 Todos

1. 先做 `yaw_rate` 最小接入验证。
   - 在 `scout_local_planner` 中仅启用 `slosh_use_imu_yaw_rate:=true`
   - 暂不启用 `slosh_use_imu_lateral_accel`
   - 暂不启用 `slosh_use_imu_alpha_z`

2. 评估 `linear_acceleration.y` 的最小预处理方案。
   - 方案优先级：
     - 先做静止零偏扣除
     - 再加 EMA 低通
     - 若仍不稳定，再考虑重力补偿

3. 明确 planner 接 IMU 前后的对比验证项。
   - 记录启用前后：
     - `/cmd_vel`
     - `/odom`
     - `/imu/data`
     - `/slosh/ay_est`
     - `/slosh/alpha_est`
     - `/mpc/status_val`
     - `/mpc/solve_ms`

4. 复核 `frame_id` 使用策略。
   - 评估是否继续使用 `base_link`
   - 或单独切成 `imu_link` 再在后端显式处理

5. 视情况修正 `udev` 规则。
   - 将当前设备 `1a86:7523` 写入 `imu_usb.rules`
   - 使 `/dev/imu_usb` 能稳定映射到实物 IMU

6. 如时间允许，补一轮“专门针对 lateral accel 的验证 bag”。
   - 静止
   - 低速左弧线
   - 静止
   - 低速右弧线
   - 静止

### 2026-03-17 代码对齐结论补充（MPC 融合位置）

- 当前 anti-slosh 不是只加在某一个位置，而是“**模型 + 代价 + 约束 + 外环治理**”协同：
  - 模型层：MPC 增广状态含 `[ETA_X, ETA_X_DOT, ETA_Y, ETA_Y_DOT]`，并在 `DiffDriveModel::predict/linearize` 中引入 `A_slosh/B_slosh` 与 `ay=v*omega` 耦合。
  - 代价层：`Q_slosh_eta` 进入状态二次型，惩罚 `ETA_X^2 + ETA_Y^2`（软约束）。
  - 约束层：可选 `enable_slosh_box_constraint`，对 `ETA_X/ETA_Y` 施加盒约束。
  - 控制外环：可选 speed governor，按液面风险压低 `v_des`，不直接改动 QP 结构。
- 因此回答“是在代价函数还是运动学模型中加入”：**两者都有**，且还叠加了可选约束和外环速度治理。

### 2026-03-17 数学化架构文档补充

- 新增文档：`docs/mpc_数学化架构总结.md`
- 内容包括：
  - 当前项目 MPC 的数学化问题定义（决策变量、动力学、代价、约束）
  - 增广状态 `ETA_X/ETA_X_DOT/ETA_Y/ETA_Y_DOT` 的物理含义
  - 关闭液面约束/软代价后的结构退化分析
  - 通用 MPC 与本项目 MPC 的数学化对比

### 2026-03-18 IMU 最小闭环验证计划

- 当前判断：
  - IMU 话题 `/imu/data` 已经 bring-up 成功
  - 但若按默认参数启动 `scout_local_planner`，IMU 仍未真正接入 `slosh estimator`
  - 原因是 `slosh_estimator/use_imu_lateral_accel`、`use_imu_yaw_rate`、`use_imu_alpha_z` 默认全为 `false`
- 因此今天的目标不是“一次把 IMU 全部接进来”，而是先完成：
  - `yaw_rate` 最小闭环验证
  - 确认 planner 在启用 IMU 后仍稳定
  - 确认估计量趋势合理、无明显发散

#### 今日验证命令

1. 对照组（不用 IMU）：

   ```bash
   roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=5
   ```

2. 实验组（只开 IMU yaw rate）：

   ```bash
   roslaunch scout_local_planner slosh_experiment.launch Q_slosh:=5 slosh_use_imu_yaw_rate:=true
   ```

3. 今日明确不打开：
   - `slosh_use_imu_lateral_accel:=true`
   - `slosh_use_imu_alpha_z:=true`

#### 今日动作结构

- 每组先跑同一套简单动作，不先上复杂导航：
  - 静止 `10 s`
  - 原地顺时针转
  - 原地逆时针转
  - 低速直行
  - 低速左弧线
  - 低速右弧线
- 若以上都稳定，再补一组带全局路径的短路线

#### 今日重点观测话题

- `/imu/data`
- `/odom`
- `/cmd_vel`
- `/slosh/ay_est`
- `/slosh/alpha_est`
- `/slosh/height`
- `/slosh/height_pred_max`
- `/mpc/status_val`
- `/mpc/solve_ms`

#### 今日判据

- 启用 `slosh_use_imu_yaw_rate:=true` 后：
  - `scout_local_planner` 不出现持续求解失败
  - `/mpc/status_val` 不明显劣化
  - `/mpc/solve_ms` 不明显升高
  - `/slosh/height_pred_max` 不出现异常尖峰
  - 转弯时估计量符号和变化趋势保持合理
- 如果这些都满足，则认为：
  - IMU `yaw_rate` 已具备接入 anti-slosh 链路的基础条件

#### 今日暂不下结论的部分

- 不根据今天结果直接认定 `IMU` 一定优于 `odom`
- 不根据今天结果直接启用 `linear_acceleration.y`
- 不根据今天结果直接启用 `alpha_z`

### 2026-03-18 终点停靠门槛与 IMU yaw 调试补充

- 修改文件：
  - `src/scout_apps/control/scout_local_planner/include/scout_local_planner/types.h`
  - `src/scout_apps/control/scout_local_planner/src/path_handler.cpp`
  - `src/scout_apps/control/scout_local_planner/include/scout_local_planner/local_planner_ros.h`
  - `src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp`
  - `src/scout_apps/control/scout_local_planner/config/mpc_params.yaml`
  - `src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml`
  - `src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh`
- 改动内容：
  - 在 `path_handler` 中新增 `goal_reached_max_speed` 与 `goal_reached_max_omega`
  - `isGoalReached()` 现在不仅检查位置/航向容差，也要求当前线速度和角速度足够低
  - 目的：避免机器人以较高速度刚进入容差区就切到 `REACHED`，随后因实物惯性继续冲过终点
  - 新增调试话题：
    - `/slosh/omega_est_used`
    - `/slosh/imu_omega_z_filtered`
  - 语义：
    - `/slosh/omega_est_used` 表示当前真正送入 `slosh_integration_.update()` 的角速度
    - `/slosh/imu_omega_z_filtered` 表示 IMU `angular_velocity.z` 的 EMA 滤波值
  - 目的：后续对照 bag 时可直接判断 `slosh_use_imu_yaw_rate:=true` 后，slosh 链路是否真的切到了 IMU yaw
  - 录包脚本已补录以上两个新话题，避免再次出现“bag 里能看到 IMU 原始值，但无法直接确认 planner 实际采用值”的歧义

### 2026-03-18 终点停靠回归修正

- 问题：
  - 上一版把 `REACHED` 判定直接改成“位置/航向容差 + 当前 odom 速度门槛”后，实车出现“始终不进入 REACHED、每个点都不停”的回归
- 原因：
  - 控制器在 `TRACKING` 状态下并不会因为这个判定门槛自动触发制动
  - 结果变成“只有已经很慢时才允许到达”，但系统本身又未先进入主动刹停阶段，因此会长期卡在 `TRACKING`
- 修正：
  - `PathHandler::isGoalReached()` 恢复为只检查位置/航向容差
  - `LocalPlannerROS` 中新增“goal stop pending”两阶段停靠逻辑：
    - 先在进入目标容差区后持续发布 `0` 速制动
    - 再等待 `odom` 线速度/角速度降到 `goal_reached_max_speed / goal_reached_max_omega` 以下
    - 只有此时才真正切换到 `REACHED`
- 结果：
  - 避免了“高速直接判到达后惯性冲过头”
  - 也避免了“永远达不到低速门槛、导致永不 REACHED”的回归

### 2026-03-18 `slosh_Q5IMU_test2_20260318_163215.bag` 阶段性结论

- 分析对象：
  - `/home/geist/slosh_bags/slosh_Q5IMU_test2_20260318_163215.bag`
- 当前状态：
  - IMU yaw rate 已确认真正融入 slosh 链路
  - 证据是：
    - `/slosh/omega_est_used` 与 `/slosh/imu_omega_z_filtered` 全程一致
    - 二者平均绝对误差为 `0`
    - `/slosh/omega_est_used` 与 `/odom.twist.twist.angular.z` 存在明显差异，平均绝对误差约 `0.0336`，最大约 `0.3677`
  - 因此当前 bag 中，`slosh_use_imu_yaw_rate:=true` 不是“参数看起来打开了”，而是 slosh 估计实际已经在使用 IMU yaw
- 终点停靠状态：
  - 前两个目标点的终点停靠逻辑已基本正常
  - bag 中两次进入 `REACHED` 时，odom 速度约为：
    - 第 1 次：`odom_v ≈ 0.029 m/s`，`odom_w ≈ -0.040 rad/s`
    - 第 2 次：`odom_v ≈ 0.024 m/s`，`odom_w ≈ -0.010 rad/s`
  - 说明“两阶段停靠”修正已经生效，不再是旧版那种高速切 `REACHED` 后惯性冲过终点
- 第三个目标点的问题性质：
  - 第三个目标点没有收敛，不是“已经到终点但不切 REACHED”
  - 更准确地说，是第三段 `TRACKING` 本身已经明显偏离参考路径，随后实车接近障碍物，用户在风险增大后切到了遥控模式
  - bag 中 `control_mode` 在 `46.465 s` 由 `1` 切到 `3`，对应人为接管
  - 该段在切遥控前的路径跟踪指标约为：
    - 平均路径偏差 `0.226 m`
    - 最大路径偏差 `0.937 m`
    - 离目标点最近距离仍有 `0.648 m`
  - 因此第三段的主矛盾不是 near-goal creeping，而是 tracking 精度不足
- 第三个目标点的控制健康状态：
  - 第三段 `TRACKING` 早期即出现 `10` 次 MPC 求解失败
  - 失败时刻集中在：
    - `37.029 s`
    - `37.094 s`
    - `37.158 s`
    - `37.196 s`
    - `37.242 s`
    - `37.301 s`
    - `37.349 s`
    - `37.412 s`
    - `37.720 s`
    - `37.775 s`
  - 说明第三段在用户切遥控前很早就进入了不健康的跟踪状态
- 对当前系统行为的判断：
  - 当前 `scout_local_planner` 仍然是 tracking MPC，不是 obstacle-aware MPC
  - 因此参数调优的目标是“尽量更贴参考路径”，而不是“保证绕开贴边障碍物”
  - 如果全局路径本身就贴障碍物很近，或跟踪误差累积到 `0.3 ~ 1.0 m` 量级，撞障碍物在机制上是可能发生的

#### 当前优先测试与修正项

- 第一优先级：先让第三段更贴参考路径，不先继续改终点停靠逻辑
  - 原因：
    - 前两个目标点已证明当前 `REACHED` 低速切换逻辑基本正常
    - 第三个目标点的问题发生在“到终点之前很久”，先修 tracking 才有意义
- 第二优先级：先降第三段的跟踪激进度，再决定是否继续提高贴路径权重
  - 建议第一轮只做最小调参，不同时大改多项
  - 建议先测：
    - `vehicle/v_max: 3.0 -> 2.0`
    - `path_handler/max_lat_accel: 2.0 -> 1.2`
    - `path_handler/lookahead_distance: 0.60 -> 0.45`
  - 目的：
    - 降低默认巡航目标速度
    - 降低弯道允许速度
    - 减少几何前视导致的切弯和贴障碍物风险
- 第三优先级：若第一轮后仍偏离较大，再提高 tracking 误差权重
  - 注意：
    - 当前配置 `use_contour_lag: true`
    - 因此实际生效的横向权重是 `Q_contour`，不是 `Q_ec`
  - 第二轮候选参数：
    - `mpc/Q_contour: 32 -> 40`
    - `mpc/Q_etheta: 10 -> 12`
    - `mpc/terminal_factor_ec: 5 -> 7`
    - `mpc/terminal_factor_etheta: 3 -> 5`
- 当前不建议优先做的调整：
  - 暂不优先降低 `R_omega` 或 `R_domega`
  - 原因：
    - 第三段已经出现较明显的角速度饱和和求解失败
    - 这时若进一步放松角速度惩罚，容易让控制更激进，而不是更稳定
- near-goal 参数暂列为次要问题：
  - 如果后续第三段 tracking 修稳后，仍出现“最后几厘米收不住”或“终点前 creeping 过快”，再单独测试：
    - `goal_capture_min_speed`
    - `goal_capture_distance`
    - `max_tan_decel`
  - 当前这三个参数不是本 bag 的主矛盾

#### 下一轮测试口径

- 固定实验前提：
  - 继续录制以下调试话题：
    - `/slosh/omega_est_used`
    - `/slosh/imu_omega_z_filtered`
  - 全程尽量保持底盘 `control_mode=1`
  - 同一路线只改一组参数，不混改 IMU 开关和 tracking 参数
- 建议测试顺序：
  1. 保持 `slosh_use_imu_yaw_rate:=true` 不变，只做第一轮最小调参
  2. 若第三段贴路径明显改善，再考虑第二轮权重调参
  3. 若第三段贴路径改善后，末端仍有收敛问题，再回头单独测 near-goal 参数
