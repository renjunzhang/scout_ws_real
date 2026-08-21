# Phase-Rejoining 未来实物执行精确辨识流程

- 日期：2026-08-21
- 当前状态：**只准备流程，未运行 Scout，实物 enforce 保持关闭**
- 当前 planar_r03 数值：只允许作为仿真开发候选，禁止升级为实车 release

本文是未来人工审核后的执行顺序，不是自动实车脚本。本轮没有启动 ROS 实车节点、没有发布 `/cmd_vel`，也没有生成任何实物 `enforce` 配置。

## 1. 数据隔离

在采集前按完整 trial 固定归属：

```text
D_id       模型结构选择和参数辨识
D_fid      完全 held-out 的执行/总 lead 保真度
D_dev      控制器、gate 和门槛开发
D_pilot    方差、样本量和现场流程试跑
D_confirm  最终正式结论
```

不能按同一个 bag 的采样点随机拆分。任何 D_fid 数据一旦用于改结构、参数或门槛，就立即降级为 D_dev，并重新采集 D_fid。

## 2. 采集前人工冻结

- Scout 身份、底盘固件、CAN、轮胎和 watchdog；
- 容器、装液量、安装位置、总载荷和质量分布；
- 目标地面、电量分层、速度/角速度安全范围和急停职责；
- `/cmd_vel` 唯一 publisher 与最终 `u_pub` audit/topic；
- mocap、odom/driver、IMU 和 ROS host 的 source/receive 时间语义；
- 输出目录、trial 表、随机顺序和不可变 sidecar schema。

缺任一项时不进入运动采集。

## 3. 激励矩阵

先低速、小幅、充分净空，逐级人工放行：

1. 线速度正/反向多幅值阶跃、ramp、制动和短 PRBS；
2. 左/右角速度多幅值阶跃、直接反转和短 PRBS；
3. 线角 S 形组合激励，只用于耦合与外推验证；
4. 每种激励跨 trial 重复，并覆盖预注册的低/中电量、正式载荷和正式地面；
5. 不靠一次长 bag 中的片段数量冒充独立重复。

必须记录最终 `u_pub`、control-cycle audit、mocap pose/status、odom/driver feedback、IMU、battery 以及每一路 source/receive timestamp。辨识输入不是 limiter 前的 solver command。

## 4. 拟合与模型选择

`/cmd_vel` 必须按发布时间做右连续零阶保持（ZOH），不能用线性插值把命令阶跃提前。延迟后的
一阶执行器也必须在真实 ZOH 事件之间传播，保证 `delay` 和 `tau` 的因果含义分开。命令 bag time、
mocap header source time、odom/driver 时间戳必须先闭合；时间语义不统一时不允许输出正式参数。

每通道至少比较：

```text
M0  pure delay / zero-order baseline
M1  delay + first-order tau
M2  M1 + directional gain/deadzone/saturation when supported by data
M3  only if held-out evidence requires cross-channel coupling or scheduling
```

联合估计并报告：

- `d_v,d_omega,tau_v,tau_omega`；
- `K+/K-`、deadzone、quantization 和 saturation；
- 正反转/左右转差异、制动、watchdog 和通信中断行为；
- 线角耦合；
- 电量、载荷、地面、日期和温度分层的参数漂移。

导数只从 source-time 对齐且平滑后的 mocap/feedback 速度计算，不能对量化 odom 逐点生硬差分。delay/deadzone/gain/tau 共线时应选择更简单结构并报告不可辨识性，不能用过参数化模型制造更小训练 RMSE。

## 5. Held-out 放行报告

D_fid 按完整 trial 报告：

- `v/omega` RMSE、P95、max、R² 和 peak timing；
- 完整 source state → expected publish → 双通道生效 → execution front 的状态误差；
- 各参数点估计、置信区间、跨 trial P95 漂移；
- 正反向、左右转、电量、载荷和地面分层；
- 模型相对 M0/M1 baseline 的 held-out 改善；
- 适用域、越界检测和 `W_exec` 误差集合。

参数漂移无法被一个冻结合同时，应缩窄工况或建立明确的分工况合同；不能取一个平均值掩盖差异。

## 6. 产物与人工放行

未来只有在报告审核通过后才生成新的实物 freeze：

```text
execution_calibration.json/yaml
held_out_execution_report.json/md
trial_assignment.csv
timeline_audit.json/md
immutable manifest with every SHA-256
```

新实物参数必须使当前仿真 v1 失效并触发 OfflineSloshOCP、online solver contract、recovery artifact 和带实物参数仿真的重新生成/验收。随后仍按 `off → monitor → 人工审核 → 安全架低速 enforce → 地面低速` 升级，任何阶段都不自动运行实车。

当前完成条件只是“未来流程已准备”；实物辨识、held-out GO 和 enforce 放行均未发生。
