# 两层轨迹规划与液体感知 MPCC

本文档描述 `feat/spmpc-liquid-control` 主线（2026-09-16）。代码和测试是最终事实来源；实验依据见[两层协同实现与软件验证记录](../../../../docs/实物实验注意事项/对比试验/解决问题的思路/20260916_两层协同实现与软件验证记录.md)，配置和入口见[`config/experiments/trajectory_mpcc/README.md`](config/experiments/trajectory_mpcc/README.md)。

## 方法定位

系统由两层组成。离线层在给定原路线和显式凸运动区域内，使用生产车辆、执行器延迟/FIFO 和四状态液体模型求解全任务轨迹。在线层使用同一 alpha-state MPCC，在原路线附近自主选择局部几何、速度和进度，并受显式区域约束。离线后缀工具从真实完整预测状态重放名义剩余命令，也可重新优化，用于诊断，不反馈在线控制。

原路线提供几何引导和进度坐标；区域 cell 提供实际空间权限，并按路线进度有序选择。新几何组的 contour 是弱偏好，lag 保留进度关联，显式区域负责空间约束，因此当前方法不要求强跟踪原路线，也不把优化后的切角曲线替换为原路线身份。

当前实现支持差速等效底盘、固定 30 Hz 网格、有限 deadline 和固定 stop window。没有实现在线剩余段反馈、连续自由终止时间优化、全局区域拓扑搜索或递归可行性保证。

## 离线任务层

[`scripts/planning/task.py`](scripts/planning/task.py) 校验任务身份、原路线、完整 28 维初态、目标、期限、执行器参数、液体参数、运动限制、目标权重和 region。CasADi/IPOPT multiple shooting（[`scripts/planning/optimizer.py`](scripts/planning/optimizer.py)）完整传播：

- 车辆状态和 alpha-state：`x,y,yaw,v,s,omega`；
- 四个液体模态状态；
- 线/角执行器 command FIFO 和加速度记忆；
- `v_s` 进度速度、速度/加速度/角加速度/jerk 限制；
- 目标姿态、deadline、TAIL 和液体 modal 代价或可选硬约束。

计划使用固定采样网格，`transport_duration <= deadline`，TAIL 从 transport 时刻开始并延续到 `deadline + stop_window`。移动段 progress 与实际速度通过任务 `progress_scale` 联系，计划终点仍是原路线长度。 [`scripts/planning/validation.py`](scripts/planning/validation.py) 从同一初态用生产动力学重新传播完整状态，并核对 FIFO、加速度记忆、区域收缩空间、目标、停车、jerk 和细化液体峰值；solver 成功状态本身不是可行性证明。

计划 schema 为 1，液体模型版本为 1，代价版本为 3。当前参数与诊断布局为 B0 `NP=92,NX=24,NU=3`、slosh `NP=104,NX=28,NU=3`，预测/快照 schema 为 8。旧 schema 7 只可按带来源身份的显式转换处理，不能用新代价重构冒充旧结果。

## 在线 MPCC

在线入口由 `SpmpcProblem`、`OcpPlanningAdapter` 和 continuous MPCC solver 组成。每周期保留持续任务时钟；重复发布同一路线不会续期，时钟倒退、换源或计时任务更换路线会失败。adapter 核对计划与运行时的 route、frame、region、物理参数、dt、目标和期限，然后装配每个 horizon stage 的计划参考和 region halfspaces。

MPCC 控制为

\[
u=[a_{cmd},\alpha_{cmd},v_s]^T,\qquad
\dot v_{cmd}=a_{cmd},\quad\dot\omega_{cmd}=\alpha_{cmd},\quad\dot s=v_s.
\]

实际线/角速度由延迟 FIFO 和一阶执行器传播，不直接积分上述命令加速度。车辆实际速度允许配置的微小负漂移（当前候选 `actual_v_min=-0.002`），命令速度和进度速度保持非负。车辆、命令、FIFO 和加速度记忆在预测域内均受约束；RTI 成功后还检查预测动力学 defect、区域/参考域、运动边界、jerk、目标和液体限制。

局部几何使用归一化三次拟合；每阶段速度/进度速度参考保存四个 progress knot；代价、诊断和采样使用同一坐标域。contour/lag、速度/进度速度、控制平滑、曲率和曲率变化项共同构成弱引导目标。低速曲率使用

\[
\sqrt{v_{actual}^2+speed\_floor^2}
\]

正则化，代表有限优化惩罚，不定义零速处真实曲率。

显式 region cell 的凸半空间收缩车体 footprint、margin，并在预测节点额外考虑 `max_speed * dt` 的保守扫掠余量。相邻 cell 要有车体可通行的凸交集；仅 progress 重叠不构成空间连续性。终点阶段按实际位置、朝向、实际速度、角速度、命令历史和队列共同判定停车。四组共享 `mpc_stop_handoff_enable=true`、`complete_stop.enable=false`；默认不启用完整液体稳定等待。

## 液体模型

液体使用四状态低阶阻尼模态，状态顺序为 `eta_x, eta_x_dot, eta_y, eta_y_dot`，基础系数包括 `two_zeta_omega_n, omega_n_sq, kappa_x, kappa_y`。预测用实际运动驱动旋转液体核；当前液体初态可选 odom 或 processed-IMU 观察器，必须依据实际 selection 判断。当前没有 RGB 液面观测闭环估计。在容器随车旋转的坐标系中，`q=[eta_x,eta_y]` 满足：

```text
q_ddot = -2*zeta*omega_n*q_dot - omega_n²*q - a_C
         - 2*omega_actual*J*q_dot - alpha_actual*J*q + omega_actual²*q
```

其中J为平面90度旋转算子，a_C为容器作用点加速度；预测用实际运动，测量端按传感外参换算，不能再重复叠加向心项。内层主要使用

\[
H_{modal}=c_h\sqrt{\eta_x^2+\eta_y^2}
\]

作为 modal cost、可选 hard cap 和预测诊断量。准静态转弯抛物面修正只在明确的 governor/可视化口径中使用，不能与 modal 高度混称真实液面。真实液面效果必须由独立 RGB 或其他外部测量评价。

离线层的液体项改变全任务速度/几何选择；在线 `planned_slosh` 使用同一计划并在 MPCC 内启用液体项，`planned_mpcc` 是匹配的下层液体关闭对照。四个 profile 不是完整的“上层液体开关 × 下层液体开关”析因设计；若要隔离上层液体规划，需另生成 liquid objective 关闭的计划。

## Profile 与证据边界

- `raw_mpcc`：原路线引导的普通 MPCC 对照；新几何目标、计划和液体机制关闭，但使用相同显式 region 匹配空间权限；
- `geometry_mpcc`：原路线弱引导、显式区域和在线几何目标，无离线计划；
- `planned_mpcc`：离线全任务计划加在线自主几何，在线液体项关闭；
- `planned_slosh`：与 `planned_mpcc` 使用同一计划和非液体设置，在线液体项开启。

`raw_mpcc` 保留普通 contour=1，其他三组为0.02；它不做新增路径优化。旧 `Full/Smooth`、旧 direct-omega 或历史 governor 结果属于历史路线，不能与当前两层主线混为同一实现。

2026-09-16 软件验证显示：15个C++测试程序和62项Python定向检查通过，24组模型闭环完成22组；当前候选尚未取得相对原始 MPCC 的稳定降晃收益，也未通过 30 Hz 实时性验收。当前结果不是实物验证，ROS1/catkin 整节点也未在本轮 ROS2 Jazzy 主机完成编译运行。`diag/lt-dwa-collision-tracking` 是历史实物分支，不由本主线结果替代。

离线后缀诊断保留真实 progress、FIFO 和液体状态；它不清零状态、不投影回名义位姿、不改变在线命令，也不宣称递归可行性。后缀失败会保留候选失败原因；剩余段反馈尚未接入在线控制。

输入核对、录制边界和复现入口见[`config/experiments/trajectory_mpcc/README.md`](config/experiments/trajectory_mpcc/README.md)及其中引用的脚本说明。录制脚本只核对 live 参数并记录已有节点，不启动 ROS、不发布速度、不代表实物安全验收。
