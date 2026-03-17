# 变更记录 2

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
