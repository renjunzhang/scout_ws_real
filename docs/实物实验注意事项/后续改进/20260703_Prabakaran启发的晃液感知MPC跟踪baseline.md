# 20260703 Prabakaran 启发的晃液感知 MPC 跟踪 baseline

## 1. 当前决策

论文 `Slosh-Aware Trajectory Control in a Reconfigurable Staircase Service Robot` 是 SPMPC 的重要近邻工作，需要在论文写作中正面回应；但当前阶段不优先实现该对比 baseline。

当前优先级仍是：

1. 先把 SPMPC 自身实物效果跑稳定；
2. 先修正/确认 delay compensation、diagnostics、hard cap、eta-dot 尺度等当前已知问题；
3. 等 B0 / B_slosh / B_ours 的实物内部消融结果稳定后，再考虑新增 Prabakaran-inspired baseline。

因此，本方案暂放入“后续改进”，不进入当前实物排障主线。

## 2. 为什么不能原样复刻 Prabakaran

Prabakaran 等人的方法面向 sTetro-SR reconfigurable staircase service robot，平台是 Mecanum/全向底盘，并在楼梯清洁任务中假设姿态变化较小，将运动近似为两个方向解耦的加速度控制：

```text
xddot = v_x
yddot = v_y
```

Scout Mini 是四轮差速 / skid-steer 底盘，不能直接横移，横向晃液激励主要来自：

```text
a_y ≈ v * omega
```

因此在 Scout 上不能声称“复刻 Prabakaran 方法”，更合适的命名是：

```text
Prabakaran-inspired slosh-aware MPC tracking baseline
```

即：受 Prabakaran 启发的晃液感知 MPC 轨迹跟踪 baseline。

## 3. 与 SPMPC 的核心边界

| 维度 | Prabakaran-inspired baseline | SPMPC |
|---|---|---|
| 层级 | tracking control | local planning / MPCC |
| 输入 | 带时间参数的参考轨迹 | reference path / local path |
| 是否优化路径进度 `s` | 否 | 是 |
| 优化目标 | tracking error + input effort + slosh penalty | contour + lag + progress + smooth + slosh |
| 输出 | `/cmd_vel` | `/cmd_vel` |
| 主要作用 | 证明 slosh-aware tracking control 的已有路线 | 证明 slosh-aware local planning 的方法优势 |

因此，后续如果实现 baseline，应保持 tracking-control 口径，不要加入路径进度状态 `s`。一旦加入 `s` 并优化 path progress，就会接近 SPMPC/MPCC，失去对比边界。

## 4. 后续建议实现的 baseline

建议最终形成以下对比组：

| 方法 | 作用 |
|---|---|
| Pure tracking / PID / DWA / TEB | 常规移动机器人 baseline |
| MPC tracking | 不考虑晃液的 MPC tracking |
| Slosh-aware MPC tracking | Prabakaran-inspired baseline |
| SPMPC | 我们的方法 |

第一版不建议直接实现完整 KF。更稳妥路线是：

```text
阶段 1：普通 MPC tracking
阶段 2：加入 slosh state propagation
阶段 3：加入 slosh cost / slosh constraint
阶段 4：必要时再加入 KF
```

## 5. 推荐系统结构

```text
Time-parameterized reference trajectory
x_d(t), y_d(t), theta_d(t), v_d(t), omega_d(t)
        ↓
Scout tracking model
        ↓
MPC tracking controller
tracking error + input effort + slosh penalty
        ↓
cmd_vel = [v, omega]
        ↓
Scout Mini + liquid container
        ↓
odom / localization / IMU
        ↓
slosh state propagation or Kalman filter
        ↓
estimated augmented state
        └── back to MPC
```

RGB / LCR / max-LCR 液面检测只作为外部评价，不进入反馈控制。

## 6. Scout 上建议的模型口径

机器人 tracking model：

```text
xdot = v cos(theta)
ydot = v sin(theta)
thetadot = omega
vdot = a
omegadot = alpha
```

控制量可用：

```text
u = [a, alpha]^T
```

晃液模型采用简单摆思想：

```text
alpha_x_ddot = -(g/l) alpha_x - (c/M_p) alpha_x_dot + (1/l) a_x
alpha_y_ddot = -(g/l) alpha_y - (c/M_p) alpha_y_dot + (1/l) a_y
```

Scout 上可近似：

```text
a_x ≈ vdot
a_y ≈ v * omega
```

若后续 IMU 横向加速度经过零偏扣除和 A/B 验证稳定，也可考虑：

```text
a_y = imu_ay_filtered
```

## 7. 评价指标

后续正式对比时建议记录：

| 指标 | 含义 |
|---|---|
| peak LCR / max liquid height | 最大晃动风险 |
| RMS LCR | 全程平均晃动 |
| settling time | 晃动衰减时间 |
| path tracking RMSE | 路径跟踪精度 |
| completion time | 通行效率 |
| control effort | 控制激烈程度 |
| solver time | 实时性 |

## 8. 进入该后续任务的前置条件

在启动该 baseline 实现前，应先满足：

1. SPMPC 当前实物内部消融重新跑通；
2. `angular_delay_sec` 过补偿问题已处理或完成方波 bag 标定；
3. `/spmpc/debug/slosh_state`、`/spmpc/slosh_height`、`/spmpc/debug/effective_config` 等诊断口径已稳定；
4. hard cap 的 `slosh/slosh_height_max` 路径和 modal-only 解释已固定；
5. RGB / LCR 外部液面评价流程可用于正式实验。

## 9. 论文写作用法

后续论文中可这样定位 Prabakaran：

> Prabakaran et al. proposed a Kalman-filter-based slosh-aware MPC controller for a reconfigurable staircase service robot. Their method addresses trajectory tracking under liquid sloshing on a specialized platform. In contrast, SPMPC formulates slosh-aware local planning for a standard wheeled mobile robot, jointly optimizing path progress, contouring error, control smoothness, and slosh states in an MPCC framework.

中文概括：

> Prabakaran 已经证明 slosh-aware MPC tracking/control 是有效方向；SPMPC 的区别是将 slosh-aware 思想推进到普通轮式移动底盘的在线局部规划层。
