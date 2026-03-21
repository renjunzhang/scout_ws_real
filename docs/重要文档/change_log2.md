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

- 修改文件：[local_planner_ros.h](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/include/scout_local_planner/local_planner_ros.h)、[local_planner_ros.cpp](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp)、[record_slosh_experiment.sh](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh)、[topic_list.md](/home/a/scout_ws/docs/重要文档/topic_list.md)
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

- 新增文档：[配置IMU.md](/home/a/scout_ws/docs/重要文档/配置IMU.md)
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

- 修改文件：[change_log.md](/home/a/scout_ws/docs/重要文档/change_log.md)
- 修改文件：[topic_list.md](/home/a/scout_ws/docs/重要文档/topic_list.md)
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


### 2026-03-18 阶段 3：IMU ay 零偏扣除启动

- 背景：
  - 阶段 2 专用 bag `/home/geist/slosh_bags/slosh_Q5_20260318_200204_stage2_ay.bag` 表明：
    - `imu ay` 左右转方向信息有价值
    - 但静止段存在稳定负偏置，约 `-0.04 m/s²`
    - 左弧线时均值虽然为正，但符号占比不够干净，不能直接裸用
  - 因此结论是：
    - `ay` 值得继续做
    - 但不能直接开启 `slosh_use_imu_lateral_accel:=true`
- 本轮实现：
  - 在 `LocalPlannerROS::imuCallback()` 中加入阶段 3 的最小预处理：
    - 连续静止窗口估计 `imu_ay_bias`
    - 在线执行 `ay_raw - imu_ay_bias`
    - 再进入 IMU ay 的 EMA 低通
  - 新增参数：
    - `slosh_estimator/imu_ay_bias_compensation_enable`
    - `slosh_estimator/imu_ay_bias_init_duration`
    - `slosh_estimator/imu_ay_bias_static_v_max`
    - `slosh_estimator/imu_ay_bias_static_omega_max`
    - `slosh_estimator/imu_ay_bias_min_samples`
  - 新增调试话题：
    - `/slosh/imu_ay_bias`
    - `/slosh/imu_ay_filtered`
    - `/slosh/imu_ay_bias_ready`
  - 录包脚本已补录以上三个话题
- 为了便于阶段 3 验证，还补了两项工程支持：
  - `slosh_experiment.launch` 新增 `cmd_vel_topic` 参数
    - 可把 planner 输出重定向到调试话题，避免与底盘直控脚本抢 `/cmd_vel`
  - `controlLoop()` 在 `IDLE/ERROR` 下也持续发布 slosh/IMU 调试话题
    - 这样即使 planner 不负责开车，也能在阶段 3 bag 中观测 bias 扣除效果
- 新增工具：
  - `scripts/analyze_imu_ay_stage2.py` 已升级：
    - 若 bag 中存在 `/slosh/imu_ay_filtered`
    - 会自动输出“原始 imu ay”和“扣零偏后 imu ay”两套统计结果

### 2026-03-18 阶段 3 首包结果

- 分析 bag：
  - `/home/geist/slosh_bags/slosh_Q5_20260318_202134_stage3_ay_bias.bag`
- 结果：
  - bias 扣除链路工作正常：
    - `/slosh/imu_ay_bias_ready = 1`
    - `/slosh/imu_ay_bias ≈ -0.0795`
  - 扣偏置后，`imu ay` 质量明显提升：
    - 静止均值：`-0.0528 -> +0.0264`
    - 左转均值：`+0.0182 -> +0.1062`
    - 右转均值：`-0.1564 -> -0.0809`
    - 与 `v*omega` 相关性：`0.366 -> 0.701`
- 当前判断：
  - 阶段 3 的最小预处理是有效的
  - 但仍存在小幅残余静止偏差，不建议直接把 `slosh_use_imu_lateral_accel:=true` 带入复杂导航
  - 下一步应先做：
    - `slosh_use_imu_lateral_accel:=true`
    - 固定动作、无复杂路径的专用验证

### 2026-03-18 阶段 3 稳健 bias 估计调整

- 背景：
  - `slosh_Q5_20260318_202834_stage3_ay_enabled.bag` 证明：
    - `slosh_use_imu_lateral_accel:=true` 时，`/slosh/ay_est` 已经真正切到 `/slosh/imu_ay_filtered`
    - 但旧版 bias 估计存在“扣过头”问题
      - 原始静止 `imu_ay` 均值约 `-0.0571`
      - 已锁定 bias 却达到 `-0.0994`
      - 导致静止段出现正残差，右转有效幅值被明显抵消
- 本轮修正：
  - `imu ay` bias 只允许在启动后的第一段静止窗口内锁定一次
  - 不再用原始 `ay_raw` 的普通均值直接估计 bias
  - 改为：
    - 静止窗口内先做一层专用 EMA
    - 再对 EMA 后样本做 trimmed mean
  - 若机器人在第一段静止窗口结束前未满足 `init_duration + min_samples`
    - 本次运行不再继续补做 bias 估计
- 新增参数：
  - `slosh_estimator/imu_ay_bias_estimator_alpha`
  - `slosh_estimator/imu_ay_bias_trim_ratio`
- 当前待验证：
  - 重新录制 `stage3_ay_enabled` 专用 bag
  - 重点看：
    - `/slosh/imu_ay_bias`
    - `/slosh/imu_ay_filtered`
    - 左右转时 `imu_ay_filtered` 的符号占比
    - 静止段残余偏差是否明显收敛到接近 `0`

### 2026-03-18 阶段 3 稳健 bias 估计复测结果

- 分析 bag：
  - `/home/geist/slosh_bags/slosh_Q5_20260318_205505_stage3_ay_enabled_v2.bag`
- 结果：
  - 稳健 bias 估计较上一包明显改善：
    - `static imu_ay_filtered mean: +0.0426 -> -0.0204`
    - `left-turn imu_ay_filtered mean: +0.1352 -> +0.0515`
    - `right-turn imu_ay_filtered mean: -0.0233 -> -0.1441`
    - `corr(imu_ay_filtered, v*omega): 0.6429 -> 0.7252`
  - 左右转符号一致性已明显改善：
    - 左转正值占比约 `0.795`
    - 右转负值占比约 `0.967`
  - `imu_ay_bias_ready_ratio = 0.639`
    - 该现象符合“只在第一段静止结束时锁定一次 bias”的新逻辑
  - 从 bag 话题范围看，锁定后的 bias 约为 `-0.0317`
- 当前判断：
  - 阶段 3 基本通过
  - `slosh_use_imu_lateral_accel` 链路可进入低风险导航 A/B 测试
  - 在拿到导航 A/B 结果前，仍不建议把 `slosh_use_imu_lateral_accel:=true` 直接作为默认正式配置
- 本轮修改文件：
  - `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/include/scout_local_planner/local_planner_ros.h`
  - `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp`
  - `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params.yaml`
  - `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml`
  - `/home/geist/scout_ws/docs/融入IMU.md`
  - `/home/geist/scout_ws/docs/change_log2.md`

### 2026-03-18 阶段 4 测试脚本补充

- 新增脚本：
  - `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/run_imu_stage4_sequence.py`
- 用途：
  - 自动执行阶段 4 的 `alpha_z` 专用动作
  - 顺序为：
    - 静止
    - 原地左转起转
    - 左转停下
    - 静止
    - 原地右转起转
    - 右转停下
    - 静止
- 默认参数：
  - `stop-1 = 5s`
  - `left-duration = 2s`
  - `left-stop = 2s`
  - `stop-2 = 3s`
  - `right-duration = 2s`
  - `right-stop = 2s`
  - `stop-3 = 5s`
  - `omega = 0.60 rad/s`
- 同步更新：
  - `CMakeLists.txt` 已加入该脚本安装列表
  - `融入IMU.md` 已补充阶段 4 的脚本入口和推荐运行方式

### 2026-03-20 按 20260319 方案冻结默认安全配置

- 依据：
  - `20260319进一步修改方案.md` 已明确当前默认安全配置应为：
    - `slosh_use_imu_yaw_rate:=true`
    - `slosh_use_imu_lateral_accel:=false`
    - `slosh_use_imu_alpha_z:=false`
  - 同时明确：
    - `alpha_z` 暂不作为当前主线
    - 当前最高优先级是先完成 `lateral_accel` 的低风险导航 A/B
- 本轮修改：
  - 将 `use_imu_yaw_rate` 的默认值从 `false` 冻结为 `true`
    - 覆盖代码默认成员
    - 覆盖 YAML 默认参数
    - 覆盖 `slosh_experiment.launch` 的默认 launch 参数
  - 保持：
    - `slosh_use_imu_lateral_accel = false`
    - `slosh_use_imu_alpha_z = false`
  - 将 `launch/README.md` 的“实物 IMU 预留实验”示例改成当前主线推荐配置
    - 不再默认展示“三个 IMU 开关全开”的入口
    - 额外补充 `lateral_accel` 低风险导航 A/B 的推荐命令
- 本轮修改文件：
  - `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/include/scout_local_planner/local_planner_ros.h`
  - `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp`
  - `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params.yaml`
  - `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml`
  - `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/launch/slosh_experiment.launch`
  - `/home/geist/scout_ws/src/scout_apps/control/scout_local_planner/launch/README.md`
  - `/home/geist/scout_ws/docs/20260320测试流程命令.md`
  - `/home/geist/scout_ws/docs/重要文档/change_log2.md`



### 2026-03-19 slosh 参数真源收口

- 问题：
  - 文档里已经把 28 mm 试管参数更新成 `container_radius = 0.014 m`、`liquid_height = 0.055 m`、`damping_ratio = 0.12`
  - 但 `slosh_experiment.launch` 实际只加载 `scout_local_planner/config/mpc_params*.yaml`
  - 而运行时代码 `LocalPlannerROS::loadParameters()` 也是直接从当前节点的 `slosh/*` 读取参数
  - 因此如果 `mpc_params*.yaml` 仍保留旧值，实物实验实际用到的就仍是旧的大容器参数
- 修正：
  - 将 `scout_local_planner/config/mpc_params.yaml` 中 `slosh/*` 更新为 28 mm 试管参数
  - 将 `scout_local_planner/config/mpc_params_sim.yaml` 中 `slosh/*` 也同步到同一组参数，避免 `sim` / `real` 两套实验入口语义分裂
  - 将 `slosh_models/config/slosh_params.yaml` 明确标注为“参考示例文件，不是 `scout_local_planner` 的运行时真源”
- 当前约定：
  - `scout_local_planner` 的 slosh 参数真源为：
    - `scout_local_planner/config/mpc_params.yaml`
    - `scout_local_planner/config/mpc_params_sim.yaml`
  - 后续如果修改 `(R, h, zeta)`，应优先修改上述 planner YAML，而不是只改 `slosh_models/config/slosh_params.yaml`

### 2026-03-19 `imu_link` 静态 TF 方案整理

- 背景：
  - 当前 IMU 安装平移量只有粗测值，例如：
    - `imu_rx = +0.13 m`
    - `imu_ry = -0.13 m`
  - 这类粗测值暂时不适合直接拿去做 `ay` 杠杆臂补偿
  - 但可以先用于把 `imu_link` 语义和 `TF` 树理顺
- 新增文件：
  - [scout_imu_with_tf.launch](/home/a/scout_ws/src/scout_ros/scout_bringup/launch/scout_imu_with_tf.launch)
- 方案：
  - 继续复用 `scout_imu.launch` 启动厂家 IMU 驱动
  - 同时额外发布 `base_link -> imu_link` 静态 TF
  - 默认把 IMU 消息头 `frame_id` 改为 `imu_link`
- 当前约定：
  - 这套静态 TF 方案当前用于：
    - 理顺 frame 语义
    - 便于 RViz / tf 工具核对安装位姿
    - 为后续如需精细外参补偿预留统一入口
  - 当前不据此直接修改 `scout_local_planner` 里的 `ay` 补偿逻辑
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



## 2026-03-20

### 文档总览更新：`总结1.md`

- 修改文件：[总结1.md](/home/a/scout_ws/docs/重要文档/总结1.md)
- 本轮将项目总览文档按当前真实实现状态重新对齐，重点更新了：
  - 实机 IMU 接入现状
  - `slosh_models` 当前运行口径
  - `LocalPlannerROS` 中 `imuCallback()` 的实际处理链
  - 阶段 7 / anti-slosh 当前状态与已知局限
- 关键口径调整：
  - 明确当前实机 `/imu/data` 已稳定接入，约 `50 Hz / 115200`
  - 明确当前默认安全配置是：
    - `yaw_rate=true`
    - `lateral_accel=false`
    - `alpha_z=false`
  - 明确当前状态传播仍是 L 动力学，`use_linear_model` 主要切高度系数
  - 明确当前运行语义是：
    - L dynamics
    - L height coefficient
    - `L + parabola term` 监测高度
  - 明确当前 `offset_x = offset_y = 0` 时：
    - `alpha_z / yaw_rate` 对主模态传播直接作用较弱
    - 真正直接改变模态传播的 IMU 通道主要是 `ay`

### 文档目录补齐：`总结1.md`

- 修改文件：[总结1.md](/home/a/scout_ws/docs/重要文档/总结1.md)
- 将目录补齐到当前 `## / ###` 结构，新增了各章小节入口
- 这样后续再维护 IMU、MPC、slosh 相关段落时，目录和正文结构能保持一致

### MPC 数学文档补强：`mpc_数学化架构总结.md`

- 修改文件：[mpc_数学化架构总结.md](/home/a/scout_ws/docs/重要文档/mpc_数学化架构总结.md)
- 开头新增：
  - 目录
  - 参数入口速查
- 参数入口速查中明确了 3 个真实入口：
  - 实物默认参数真源：`mpc_params.yaml`
  - 仿真默认参数真源：`mpc_params_sim.yaml`
  - 实验时常用覆盖入口：`slosh_experiment.launch`
- 同时把常用参数改成“参数名 + 同行解释”的格式，涵盖：
  - cost 权重
  - near-goal / 约束参数
  - `slosh` 模型参数
  - IMU / 实验覆盖参数

### cost 项数与语义说明补充

- 修改文件：[mpc_数学化架构总结.md](/home/a/scout_ws/docs/重要文档/mpc_数学化架构总结.md)
- 补充了“当前 cost 到底有几类、几项”的更精确口径：
  - 基础 tracking：4 项
  - 基础 control：2 项
  - 基础 control-rate：2 项
  - 因此基础骨架可理解为 `8` 项
  - 可选 slosh：`2` 项
  - 可选 `omega_ff`：`1` 项
- 同时明确：
  - `terminal ramp` 不是新增 cost 项，只是末段权重放大
  - 当前 `Q_slosh > 0` 时才会真正把 slosh 软代价加入 QP
  - 当前 `enable_omega_ff = true` 时会加入 `(\omega-\omega_ref)^2`

### IMU 文档可读性整理：`配置IMU.md` 与 `融入IMU.md`

- 修改文件：
  - [配置IMU.md](/home/a/scout_ws/docs/重要文档/配置IMU.md)
  - [融入IMU.md](/home/a/scout_ws/docs/重要文档/融入IMU.md)
- 处理内容：
  - 两份文档开头都补上了目录
  - `配置IMU.md` 的目录与正文编号重新对齐
  - 修正了 `10.4 / 10.5` 这两个尾部小节的编号残留
- 目的：
  - 便于后续按“bring-up / 工控机核验 / IMU 融入阶段”快速跳读

### 论文路线状态定位：`20260314进一步修改方案.md`

- 修改文件：[20260314进一步修改方案.md](/home/a/scout_ws/docs/重要文档/20260314进一步修改方案.md)
- 将“投稿前必须完成的里程碑”改成带状态的版本，统一使用：
  - `已完成`
  - `部分完成`
  - `未开始`
- 当前定位更新为：
  - `P0`：基本完成
  - `P1`：只完成了一部分
  - `P2`：大多未开始
- 这样后续看论文主线时，可以直接区分：
  - 哪些只是工程上已经跑通
  - 哪些还没有形成投稿级证据链

### 论文建模总结补目录：`Slosh Dynamics论文中建模总结.md`

- 修改文件：[Slosh Dynamics论文中建模总结.md](/home/a/scout_ws/docs/重要文档/Slosh%20Dynamics论文中建模总结.md)
- 在文档开头新增目录
- 目录按当前结构展开到：
  - `L / NL`
  - 高度映射
  - 工程对应关系
- 便于后续快速对照论文与当前 `slosh_models` 实现

### 今日修改范围说明

- 本轮主要是**文档与当前实现状态的对齐**，没有新增控制逻辑代码改动
- 核心目标是把以下几件事写清楚：
  - 当前 IMU 融入真实进度
  - 当前 slosh 模型真实口径
  - 当前 cost / 参数入口 / 文档目录结构
  - 当前论文路线完成到哪一步

### 重要文档目录重组：迁移到 `docs/重要文档/`

- 修改范围：
  - [总结1.md](/home/a/scout_ws/docs/重要文档/总结1.md)
  - [mpc_数学化架构总结.md](/home/a/scout_ws/docs/重要文档/mpc_数学化架构总结.md)
  - [Slosh Dynamics论文中建模总结.md](/home/a/scout_ws/docs/重要文档/Slosh%20Dynamics论文中建模总结.md)
  - [配置IMU.md](/home/a/scout_ws/docs/重要文档/配置IMU.md)
  - [融入IMU.md](/home/a/scout_ws/docs/重要文档/融入IMU.md)
  - [20260319进一步修改方案.md](/home/a/scout_ws/docs/重要文档/20260319进一步修改方案.md)
  - [20260314进一步修改方案.md](/home/a/scout_ws/docs/重要文档/20260314进一步修改方案.md)
  - [change_log2.md](/home/a/scout_ws/docs/重要文档/change_log2.md)
  - [change_log.md](/home/a/scout_ws/docs/重要文档/change_log.md)
  - [topic_list.md](/home/a/scout_ws/docs/重要文档/topic_list.md)
  - [重要文档列表.md](/home/a/scout_ws/docs/重要文档列表.md)
- 本轮处理内容：
  - 以上文件的**真实位置**统一迁移到 `/home/a/scout_ws/docs/重要文档/`
  - 工作区内这些重要文档之间的**内部绝对链接**统一改到新路径
  - `docs` 根目录下原先保留的兼容符号链接已删除
- 本轮确认结果：
  - 旧路径 `/home/a/scout_ws/docs/*.md` 的硬编码引用已清理
  - 当前这些重要文档只保留 `docs/重要文档/` 这一套正式路径
- 边界说明：
  - 这一步的核心是**目录重组和链接修正**
  - **没有新增技术结论，也没有修改控制逻辑**
  - 除了内部路径更新外，没有因为“迁移文件”而重写正文内容

## 2026-03-21

### 终点问题重新定性：主矛盾不在 slosh，而在 terminal behavior

- 基于多条 `Q=0` bag 的复盘，明确当前终点不收敛的主问题不是 `Q_slosh`、不是 IMU，也不是全局路径更新频率。
- 当前更准确的定位是：
  - base tracking / terminal logic 仍不完整
  - 冲过终点后 goal 落到车后，系统仍长期停留在 forward-only tracking 语义里
  - 终点阶段缺少稳定的 recovery / align 结构

### `test_mpc.launch` 与 `slosh_experiment.launch` 默认滤波口径对齐

- 修改文件：[test_mpc.launch](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/launch/test_mpc.launch)
- 处理内容：
  - 将默认滤波覆盖改成与 `slosh_experiment.launch` 一致：
    - `filter_alpha_v = 1.0`
    - `filter_alpha_omega = 1.0`
    - `filter_kappa_boost = 0.0`
- 目的：
  - 消除“实验 launch 已关闭输出 EMA、而 `test_mpc.launch` 又重新打开默认滤波”带来的对照混乱

### 新增终点专项方案文档：`20260320代码修改方案.md`

- 新增文件：[20260320代码修改方案.md](/home/a/scout_ws/docs/重要文档/20260320代码修改方案.md)
- 文档内容：
  - 将终点问题拆成“已修复项 / 当前主因 / 下一刀 / 当前不建议继续改的项”
  - 明确当前最短路径是：
    1. 修 terminal stop 链
    2. 补 terminal recovery
    3. 再看是否需要做执行滞后 A/B

### 终点 stop 链修正：不再硬切零速，且 `v_des=0` 真正压住 `v_ref`

- 修改文件：
  - [path_handler.cpp](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/src/path_handler.cpp)
  - [local_planner_ros.cpp](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp)
  - [local_planner_ros.h](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/include/scout_local_planner/local_planner_ros.h)
- 本轮落地内容：
  - `goal_stop_pending_` 不再一进容差区就直接 `publishCmdVel(0,0)`
  - 改成由 MPC 继续收最后一段，只把 `v_des_cmd` 压到 `0`
  - `PathHandler::getReferencePoints()` 中，外部传入的 `v_des` 现在作为 `v_ref` 的硬上界
  - 修复了“上层要求停，但 time-parameterized speed profile 仍给正速度参考”的结构错误
  - `goal_stop_pending_` 增加最小锁存 / 滞回释放，避免一帧滑出容差就立刻恢复巡航

### 终点几何出口统一：新增 `GoalInfo`

- 修改文件：
  - [types.h](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/include/scout_local_planner/types.h)
  - [path_handler.h](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/include/scout_local_planner/path_handler.h)
  - [path_handler.cpp](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/src/path_handler.cpp)
- 本轮落地内容：
  - 新增统一的终点几何结构 `GoalInfo`
  - `PathHandler` 新增 `getGoalInfo()`
  - `isGoalReached()`、`getGoalDistance()` 改成复用统一终点几何出口
  - 终点 yaw 判定优先使用 goal pose 自身 orientation；不可用时才回退到路径尾部非退化切线

### 最小 terminal recovery 已接入

- 修改文件：
  - [local_planner_ros.cpp](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp)
  - [local_planner_ros.h](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/include/scout_local_planner/local_planner_ros.h)
  - [mpc_params.yaml](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params.yaml)
  - [mpc_params_sim.yaml](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml)
- 本轮落地内容：
  - 在 `TRACKING` 分支最前面接入最小 terminal recovery
  - 第一版 mode：
    - `ALIGN_TO_POINT`
    - `APPROACH_POINT`
    - `ALIGN_FINAL_YAW`
  - 后续又做了两次小修：
    - 调整 mode 优先级，先对准 goal 点，再补 final yaw
    - 补最小锁存，填平 `position_reached && !pose_reached` 时掉回 `NONE` 的空档

### 新增离线观察脚本：`observe_terminal_recovery.py`

- 新增文件：[observe_terminal_recovery.py](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/observe_terminal_recovery.py)
- 作用：
  - 在 bag 回放时，用 `/scout/goal`、`/odom`、`/cmd_vel_replay`、`/tf` 粗看 terminal recovery 是否触发
- 当前结论：
  - 这个脚本只能做启发式初筛
  - 在代码连续修改后，单靠它推断内部真实 mode 已不够可靠

### 新增真实 terminal debug 话题

- 修改文件：
  - [local_planner_ros.cpp](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp)
  - [record_slosh_experiment.sh](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh)
- 新增话题：
  - `/terminal/mode`
  - `/terminal/recovery_latched`
  - `/terminal/goal_info`
- `goal_info` 当前数组顺序：
  - `[dx, dy, dist, bearing, goal_yaw_err, has_goal_yaw, position_reached, pose_reached]`
- 目的：
  - 不再靠外部脚本猜测 terminal mode
  - 直接观测 planner 内部当前到底处于：
    - `NONE`
    - `TERMINAL_LATCHED`
    - `GOAL_STOP_PENDING`
    - `ALIGN_TO_POINT`
    - `APPROACH_POINT`
    - `ALIGN_FINAL_YAW`
    - `REACHED`

### 编译与验证

- 已多次执行：
  - `source /opt/ros/noetic/setup.bash && catkin_make --pkg scout_local_planner -j1`
- 结果：
  - `scout_local_planner` 与 `local_planner_node` 均编译通过
- 当前验证结论：
  - terminal recovery 已经不是“完全没进”
  - 但截至今天结束，**还不能宣布终点已经稳定收敛**
  - 下一步应优先使用新的 `/terminal/*` 真实调试话题做离线回放判因，而不是继续盲目扫参数

### 基于 `slosh_Q0_20260321_153105_terminal_debug2.bag` 的终点新结论

- 这条 bag 已经录到了新的真实 debug 话题：
  - `/terminal/mode`
  - `/terminal/recovery_latched`
  - `/terminal/goal_info`
- 直接结论：
  - 当前不是“terminal recovery 写错了却没收敛”
  - 而是 **terminal recovery 根本没有被触发**
- bag 里的直接证据：
  - `/terminal/mode` 全程只有：
    - `IDLE`
    - `NONE`
  - `/terminal/recovery_latched` 全程为 `0`
  - `/terminal/goal_info` 的最小 `dist` 约为 `0.583 m`
  - 而当时 `terminal_recovery/enter_distance` 仅为 `0.35 m`
- 因此当前最准确的判断是：
  - terminal 接管太晚
  - 车还没有进入 terminal band，就已经把 goal 推到车后
  - 于是整个终点阶段仍然是 normal tracking 在控

### 参数收口：放大 terminal 触发半径

- 修改文件：
  - [mpc_params.yaml](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params.yaml)
  - [mpc_params_sim.yaml](/home/a/scout_ws/src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml)
- 修改内容：
  - `terminal_recovery/enter_distance: 0.35 -> 0.70`
  - `terminal_recovery/release_distance: 0.55 -> 1.00`
- 修改原因：
  - 先让 terminal recovery 进入真实失效区
  - 在新的 bag 证明 terminal mode 真的开始触发之前，不再继续改 recovery 逻辑

### 基于 `slosh_Q0_20260321_154102_terminal_debug3.bag` 与 `slosh_Q0_20260321_155446_terminal_debug4.bag` 的终点结论修正

- 后续新 bag 表明：
  - terminal recovery 已经真正进入：
    - `APPROACH_POINT`
    - `ALIGN_TO_POINT`
    - `ALIGN_FINAL_YAW`
    - `GOAL_STOP_PENDING`
    - `REACHED`
  - 第一个 goal 已可稳定进入 `REACHED`
  - “第二个 goal 不能规划”并非结构性问题，后续复测证明是一次发送失败/流程问题，不是 terminal 逻辑本体问题
- 因此截至今天收尾，终点问题应重新定性为：
  - **terminal 收敛链已从“结构性不收敛”修到“可用版本”**
  - 当前主线不再是继续深挖 terminal，而是冻结这条链，转回 IMU/真实液面测量

### IMU `lateral_accel` A/B：`Q_slosh=0`

- 分析 bag：
  - [slosh_Q0_20260321_161120_imu_lateral_A_run1.bag](/home/a/下载/slosh_bags/debug0320/slosh_Q0_20260321_161120_imu_lateral_A_run1.bag)
  - [slosh_Q0_20260321_161426_imu_lateral_B_run1.bag](/home/a/下载/slosh_bags/debug0320/slosh_Q0_20260321_161426_imu_lateral_B_run1.bag)
- 关键结论：
  - A、B 两组都可到达 `REACHED`
  - `yaw_rate` 两组都稳定来自 IMU
  - B 组 `ay_est` 已真实切到 IMU：
    - A 组：`/slosh/ay_est` 与 `/slosh/imu_ay_filtered` RMS 约 `0.1086`
    - B 组：RMS `0.0`
  - 这轮说明：
    - `slosh_use_imu_lateral_accel:=true` 已真实接入
    - 但还不能单靠这轮就宣布“默认应打开”

### IMU `lateral_accel` A/B：`Q_slosh=5`

- 首轮分析 bag：
  - [slosh_Q5_20260321_163303_imu_lateral_A_run1.bag](/home/a/下载/slosh_bags/debug0320/slosh_Q5_20260321_163303_imu_lateral_A_run1.bag)
  - [slosh_Q5_20260321_163519_imu_lateral_B_run1.bag](/home/a/下载/slosh_bags/debug0320/slosh_Q5_20260321_163519_imu_lateral_B_run1.bag)
- 发现问题：
  - 首轮 A 组 `imu_ay_bias_ready` 在开始 `TRACKING` 后才置 `1`
  - 因此首轮 `Q=5` A/B 不完全公平
- 补录 A 组：
  - [slosh_Q5_20260321_164417_imu_lateral_A_run1.bag](/home/a/下载/slosh_bags/debug0320/slosh_Q5_20260321_164417_imu_lateral_A_run1.bag)
  - 该包中 `imu_ay_bias_ready` 在开始 `TRACKING` 前已完成
- 当前准确结论：
  - `lateral_accel=true` 已经真实接入
  - 但从目前 `Q=5` 的可比样本看，**尚未显示出明确收益**
  - 也没有显示出明显灾难性回归
  - 因此当前默认安全配置继续保持：
    - `slosh_use_imu_yaw_rate:=true`
    - `slosh_use_imu_lateral_accel:=false`
    - `slosh_use_imu_alpha_z:=false`

### RealSense 方案收束：当前下一步主线切到真实液面测量链

- 更新/完善文档：
  - [Realsense方案.md](/home/a/scout_ws/docs/重要文档/Realsense方案.md)
- 当前收口结论：
  - 不再继续围绕 `alpha_z` 或 IMU 外参深挖
  - 下一步主线是建立 **RealSense 真实液面测量链**
  - 第一版只做：
    - `1` 个相机
    - `RGB` 侧视
    - 黑墨水
    - 固定 ROI
    - 输出 `height_peak_mm / height_peak_rel_mm / meniscus_valid`
  - 第一版不做：
    - depth 主测量
    - 双相机
    - 视觉结果直接进控制器
- 结合你补充的安装信息，文档也明确记录：
  - 镜头最边缘距离试管中心约 `12 cm`
  - 这个距离从分辨率角度是可行的，当前优先检查对焦、入镜和反光，而不是继续拉远距离

### 文档收口与主线更新

- 更新文档：
  - [总结1.md](/home/a/scout_ws/docs/重要文档/总结1.md)
  - [融入IMU.md](/home/a/scout_ws/docs/重要文档/融入IMU.md)
  - [change_log.md](/home/a/scout_ws/docs/重要文档/change_log.md)
  - [重要文档列表.md](/home/a/scout_ws/docs/重要文档列表.md)
- 本轮文档更新的核心：
  - 在 `总结1.md` 与 `融入IMU.md` 开头增加“2026-03-21 关键结论”
  - 明确：
    - terminal 链已经进入可用状态
    - IMU 当前保留 `yaw_rate`，`lateral_accel` 暂不默认开启，`alpha_z` 暂缓
    - 当前下一步主线是 RealSense 真实液面测量
  - `change_log.md` 的实物流程中，`MPC` 局部规划主入口统一改成：
    - `roslaunch scout_local_planner slosh_experiment.launch ...`
    - `test_mpc.launch` 降级为最小局部规划器检查入口
  - `重要文档列表.md` 补入：
    - [20260320代码修改方案.md](/home/a/scout_ws/docs/重要文档/20260320代码修改方案.md)
    - [Realsense方案.md](/home/a/scout_ws/docs/重要文档/Realsense方案.md)

## 2026-03-22

### RealSense 液面测量包骨架落地

- 新增包：[realsense_liquid_measurement](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement)
- 当前已建立的核心文件：
  - [CMakeLists.txt](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/CMakeLists.txt)
  - [package.xml](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/package.xml)
  - [README.md](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/README.md)
  - [liquid_measurement.yaml](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/liquid_measurement.yaml)
  - [calibrate_liquid_roi.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py)
  - [annotate_liquid_roi.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/annotate_liquid_roi.py)
  - [extract_liquid_height_from_bag.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py)
- 包定位已明确为：
  - 先服务于 **RealSense RGB 离线证据链**
  - 当前不接入 `scout_local_planner` 闭环
  - 当前主口径为 `height_peak_rel_px`

### 离线标定链打通

- 当前静止 bag：
  - [/data/a/bags/realsense_session_2026-03-21_17-48-52.bag](/data/a/bags/realsense_session_2026-03-21_17-48-52.bag)
- 当前运动 bag：
  - [/data/a/bags/realsense_session_2026-03-21_17-47-55.bag](/data/a/bags/realsense_session_2026-03-21_17-47-55.bag)
- 已实现流程：
  1. 用 [calibrate_liquid_roi.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py) 从 bag 导出 RGB 参考图
  2. 用 [annotate_liquid_roi.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/annotate_liquid_roi.py) 做人工标定
  3. 当前先用 `px_only` 模式，不依赖背景标尺
- 当前已保存的参考标定文件：
  - [frame_000000_calibration.yaml](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration.yaml)
  - [frame_000000_annotated.png](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_annotated.png)
- 当前标定文件中已经固定：
  - `roi`
  - `tube_inner.x_left / x_right`
  - `calibration.still_level_px`
  - `mm_per_pixel = null`

### 提取脚本从 PoC 推到可用离线测量链

- 重点修改文件：
  - [extract_liquid_height_from_bag.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py)
- 当前主流程已不是最早的简单列扫描，而是：
  - 灰度预处理 + CLAHE
  - Otsu + bias
  - 开闭运算
  - 保留底部连通液体区域
  - 中央可信带候选点
  - 液面线拟合
  - 固定内部评估点读数
  - 时间门控
- 当前正式输出已收紧为：
  - `height_left_px`
  - `height_right_px`
  - `height_peak_px`
  - `height_left_rel_px`
  - `height_right_rel_px`
  - `height_peak_rel_px`
  - `meniscus_confidence`
  - `fit_rms_px`
  - `fit_slope`
  - `temporal_jump_px`
  - `temporal_gate_passed`
- 今日已明确不再把绝对 `height_left_mm / height_right_mm / height_peak_mm` 当作正式输出语义
- 当前如果 `mm_per_pixel = null`，脚本仍然正常运行，但 `*_rel_mm` 留空

### 静止包自动归零基线完成

- 在 [extract_liquid_height_from_bag.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py) 中新增：
  - `--auto-zero-baseline`
  - `--baseline-frame-count`
  - `--baseline-stat`
  - `--write-adjusted-calibration`
- 当前静止包已实际完成自动归零：
  - 原静止基线相对偏移约 `5 px`
  - 自动归零后建议：
    - `still_level_px ≈ 175.949570`
- 自动生成修正后的标定文件：
  - [frame_000000_calibration_auto_zero.yaml](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration_auto_zero.yaml)
- 归零后，静止包的 `height_peak_rel_px` 已基本回到 `0 px` 附近，可作为运动包分析基线

### 运动包验证结果

- 使用修正后的标定文件运行运动 bag：
  - [frame_000000_calibration_auto_zero.yaml](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration_auto_zero.yaml)
  - [/data/a/bags/realsense_session_2026-03-21_17-47-55.bag](/data/a/bags/realsense_session_2026-03-21_17-47-55.bag)
- 当前终端摘要结果：
  - `processed frames = 751`
  - `valid frames = 552`
  - `valid ratio = 73.5%`
  - `max height_peak_rel_px = 18.236`
- 当前输出目录：
  - [/data/a/bags/realsense_session_2026-03-21_17-47-55_liquid_measurement](/data/a/bags/realsense_session_2026-03-21_17-47-55_liquid_measurement)
- 当前主要输出文件：
  - [liquid_height.csv](/data/a/bags/realsense_session_2026-03-21_17-47-55_liquid_measurement/liquid_height.csv)
  - [liquid_debug.mp4](/data/a/bags/realsense_session_2026-03-21_17-47-55_liquid_measurement/liquid_debug.mp4)
  - [liquid_height_peak_curve.png](/data/a/bags/realsense_session_2026-03-21_17-47-55_liquid_measurement/liquid_height_peak_curve.png)
- 当前阶段性判断：
  - 这条链已经可以作为 **相对抬升量证据链**
  - 但还不能当成高精度绝对液位真值链

### 曲线图与 README 说明补齐

- 更新文件：
  - [README.md](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/README.md)
  - [package.xml](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/package.xml)
  - [Realsense具体修改过程.md](/home/a/scout_ws/docs/Realsense具体修改过程.md)
- 当前曲线图行为已调整为：
  - 默认主图只画 `height_peak_rel_px`
  - 有效点按 `meniscus_confidence` 着色
  - 右侧颜色条显示 `meniscus_confidence`
  - 自动标注当前最高峰对应的 `peak_rel_px` 与 `confidence`
  - 无效帧以灰色 `x` 标出
- README 当前已补充：
  - 各脚本用法
  - 自动归零流程
  - 曲线图怎么读
  - `CSV` 各字段语义
- `package.xml` 已新增：
  - `python3-matplotlib`

### 今日关于“检测质量本身”的结论

- 当前确认：
  - 标尺只能把 `px` 转成 `mm`
  - 不能自动提升有效帧比例，也不能自动消除假峰
- 当前更重要的问题是：
  - 运动阶段仍有假峰
  - 峰值报告规则还不够硬
  - 标定仍然是“单点/双 x”简化版本
- 今日已形成的下一步路线：
  1. 先固定采集条件
  2. 再做最小几何正确性
  3. 再强化主检测器
  4. 再强化峰值接受规则
- 其中“最小几何正确性”当前已确定应前移，包括：
  - `still_level_line`
  - `left_wall_line / right_wall_line`
  - `tube_axis_line`

### 今日收尾状态

- 今日已经完成：
  - 包骨架建立
  - 标定链打通
  - 自动归零
  - 静止/运动 bag 跑通
  - 曲线图输出
  - README 与过程文档补齐
- 明日继续重点：
  - 线标定替代单点/双 x 标定
  - ROI 旋正
  - `accept_for_peak_report` 硬门槛
  - 更强的假峰抑制
