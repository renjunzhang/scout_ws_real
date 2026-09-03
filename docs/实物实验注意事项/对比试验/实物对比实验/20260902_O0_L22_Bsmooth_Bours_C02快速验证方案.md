# O0 + L22 下 B_smooth / B_ours 的 C02 快速验证方案

日期：2026-09-02
状态：待执行，development 快速筛查

## 1. 目的

2026-07-05 的实物结果中，`B_ours` 在 RGB 液面指标上有正向结果。本次不追求完整解释延迟机理，只把当时有效配置的核心状态放到当前 C02 路径上复测：

```text
odom 液体 observer（O0）
+ legacy fixed_closed_loop 0.15 / 0.22 s（最终为 L22 solver 初态）
+ B_ours 现有 smooth/slosh 权重
```

用同日 `B_smooth` 作对照，回答一个简单问题：在当前系统上加入 slosh cost 后，是否能稳定降低实际 RGB 液面晃动。

本实验只用于判断系统级效果，不用于证明 `0.15/0.22 s` 是真实物理延迟，也不替代未来的执行器模型。

## 2. 固定配置

| 项目 | 配置 |
|---|---|
| 路径 | 冻结 C02_v2 |
| 路径 SHA-256 | `1464ef37857bcb899d8b0e4867ff63ea06f017e1b871bed80e077f450be14164` |
| 对照/候选 | `B_smooth` / `B_ours` |
| 液体 observer | `odom`，即 O0 |
| 延迟模式 | `fixed_closed_loop` |
| 延迟参数 | 线向 `0.15 s`，角向 `0.22 s` |
| 共同状态时刻 | `require_common_epoch=true` |
| 速度 | `v_ref=0.20 m/s`，硬上限 `0.25 m/s` |
| 平滑权重 | `w_control=0.3`，`w_smooth/w_alpha/w_du_a/w_du_vs=1.0` |
| slosh 权重 | `B_smooth=0`，`B_ours=5` |
| 物理液面 | 在线 RGB 标量 `/liquid/measurement` |
| 其他记录 | NOKOV、odom、IMU、planner、delay/state diagnostics |
| 图像策略 | 不向 bag 写入原始或调试图像 |

这里的 O0 是 observer 起点。对 `B_ours`，`fixed_closed_loop` 会继续用命令历史滚动机器人和液体状态，实际进入 solver stage 0 的是 L22；`B_smooth` 不消费液体状态，但使用同一机器人延迟补偿配置。

本次不是对 0705 的逐字节复现：使用的是当前 C02 路径、当前代码以及新增的共同状态时刻和速度安全门。

## 3. 实验顺序

采用最小 ABBA，降低液体初始状态和现场漂移的影响：

```text
Row 01  B_smooth
Row 02  B_ours
Row 03  B_ours
Row 04  B_smooth
```

每包单独启动。两包之间将机器人送回 C02 起点、对齐航向，并让液体静置 `60–90 s`，不要用循环连续运行。

## 4. 运行命令

入口脚本：

```text
src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_o0_l22_bsmooth_bours_trial.sh
```

传感器、Cartographer 定位和 NOKOV monitor 保持运行。先执行 Row 01：

```bash
cd /home/geist/scout_ws
VALIDATE_ONLY=false ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES \
ROW=01 bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_o0_l22_bsmooth_bours_trial.sh
```

之后按相同命令依次把 `ROW` 改为 `02`、`03`、`04`。如果某次采集失败且已有同名产物，保留原产物并使用 `ATTEMPT=02` 重试。

默认输出目录：

```text
/home/geist/slosh_bags/real/20260902_spmpc_o0_l22_bsmooth_bours_c02
```

## 5. 简单判定

先满足最低有效性：四包均正常闭包、到达终点、NOKOV/RGB 有效、`fixed_closed_loop` 实际应用，且没有安全停车或明显离轨。

效果判断以在线 RGB 的 `H_vis` 为主：

- 两个 block 中 `B_ours` 的 `H_vis` P95/RMS 都低于相邻的 `B_smooth`，可记为当前系统级正向；
- 只有一组改善或差异很小，记为不确定，可补一组而不立即改模型；
- 两组都更差，则停止沿用该 O0+L22 组合；
- 如果改善主要来自明显变慢，应把减晃与时间代价同时报告，不宣称延迟模型已被验证。

目标是先确认“这套系统是否有效”，不要求本轮一次解决全部状态估计和执行器建模问题。
