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

车体 pose/twist 始终对齐同一观测 epoch；消费液体的显式执行器组还要求共同液体 epoch。进度投影首次可全局定位，之后复用上拍分支，在 `planning/projection/lookahead` 窗口中按路径顺序选择首个局部距离极小；相等距离平台统一处理。外层与 solver 共用已接受的进度，避免自交处重新选支。窗口使用路线进度单位，允许切角和横向偏离；显式新任务重置定位，重复同路线不重置。

MPCC 控制为

\[
u=[a_{cmd},\alpha_{cmd},v_s]^T,\qquad
\dot v_{cmd}=a_{cmd},\quad\dot\omega_{cmd}=\alpha_{cmd},\quad\dot s=v_s.
\]

实际线/角速度由延迟 FIFO 和一阶执行器传播，不直接积分上述命令加速度。车辆实际速度允许配置的微小负漂移（当前候选 `actual_v_min=-0.002`），命令速度和进度速度保持非负。车辆、命令、FIFO 和加速度记忆在预测域内均受约束；RTI 成功后还检查预测动力学 defect、区域/参考域、运动边界、jerk、目标和液体限制。

planned阶段几何使用归一化三次拟合，每阶段速度/进度速度参考保存四个progress knot。raw/geometry的cruise几何仍按绝对progress坐标拟合；路线末端重复采样会使导数与参考朝向退化，ReferenceSpline端点差分还可能产生假曲率，见[本轮复盘](../../../../docs/实物实验注意事项/后续改进/20260916_当前主线复盘与剩余缺口.md#32-普通参考的末端几何退化影响自主几何与对照可靠性)。contour/lag、速度/进度速度、控制平滑、曲率和曲率变化项共同构成弱引导目标。低速曲率使用

\[
\sqrt{v_{actual}^2+speed\_floor^2}
\]

正则化，代表有限优化惩罚，不定义零速处真实曲率。

显式 region cell 的凸半空间收缩车体 footprint、margin，并在预测节点额外考虑 `max_speed * dt` 的保守扫掠余量。相邻 cell 要有车体可通行的凸交集；仅 progress 重叠不构成空间连续性。终点阶段按实际位置、朝向、实际速度、角速度、命令历史和队列共同判定停车。四组共享 `mpc_stop_handoff_enable=true`、`complete_stop.enable=false`；默认不启用完整液体稳定等待。

区域启用时，`TaskStopManager` 共享检查当前停车尾段和执行候选首命令后的停车尾段，覆盖真实FIFO、jerk制动、实际运动及quiet后的执行器残余位移。尾段固定当前cell，因而可能保守拒绝本可跨cell完成的制动。候选命令会失去停车余量时，停车模块接管并持续至任务重置；到达状态仍逐拍重查，位姿漂移后撤销成功也不会自动开动。`STOP_FIFO_REGION_VIOLATION` 表示模型中不可改变的FIFO前缀节点已越界，`STOP_REGION_UNSAFE` 只表示本制动策略未通过保守核验。它们都不构成模型误差、求解后延迟或故障发布路径下的实物安全保证。

当前尾段尚未逐节点核对 `actual_v_min/v_max/omega_max`，外层只检查首条停车命令上界；`STOP_TAIL_QUIET`不能解释为完整运动边界均已通过。在线OCP预测的运动边界核验与停车尾段核验需分别看待，具体反例见[本轮复盘](../../../../docs/实物实验注意事项/后续改进/20260916_当前主线复盘与剩余缺口.md)。

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

2026-09-16五项修复后，stub 17个、真实acados 18个C++测试程序均已通过；本轮49项Python检查通过，另2项依赖roslaunch的用例受环境阻断。24组模型闭环完成22组，两个geometry单弯仍失败，最长周期82.2 ms。当前候选尚未取得相对原始 MPCC 的稳定降晃收益，也未通过30 Hz实时性验收，见[本轮回归记录](../../../../docs/实物实验注意事项/后续改进/20260916_局部规划器五项修复与回归.md)。ROS1/catkin整节点未在本轮ROS2 Jazzy主机完成编译运行；`diag/lt-dwa-collision-tracking`是历史实物分支，不由模型结果替代。

离线后缀诊断保留真实 progress、FIFO 和液体状态；它不清零状态、不投影回名义位姿、不改变在线命令，也不宣称递归可行性。后缀失败会保留候选失败原因；剩余段反馈尚未接入在线控制。

输入核对、录制边界和复现入口见[`config/experiments/trajectory_mpcc/README.md`](config/experiments/trajectory_mpcc/README.md)及其中引用的脚本说明。录制脚本只核对 live 参数并记录已有节点，不启动 ROS、不发布速度、不代表实物安全验收。
