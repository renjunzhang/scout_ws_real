# I0 + 显式执行器 OCP 的 B0/Bslosh ABBA 实物验证方案

日期：2026-09-03

协议 ID：`SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_ABBA_DEV_V1`

预注册时状态：`CODE-IMPLEMENTED / SOFTWARE-VERIFIED / HARDWARE-UNVERIFIED / SOURCE-UNCOMMITTED`

执行状态（2026-09-03）：`ROW01 B0 PASS / ROW02 Bslosh RUNTIME FAIL / BLOCK1 STOPPED`

执行分析见 [20260903 I0 + 显式执行器 OCP 的 B0/Bslosh Block 1 分析](../实物对比试验分析/20260903_I0显式执行器OCP_B0_Bslosh_Block1分析.md)。Row02 因共同时间插值失败和 acados `MINSTEP` 触发故障性零速，不进入效果统计；Row03/Row04 不再执行。

## 目的

只回答一个系统问题：

> 替换 legacy L22 后，`processed-IMU I0 + explicit actuator OCP` 下的 literal `Bslosh`，能否比同配置 `B0` 降低实物 RGB 液面晃动，同时不明显变慢？

这是 development 验证，不把单个正向 bag 当作正式结论。

## 冻结配置

| 项目 | B0 | Bslosh |
|---|---:|---:|
| variant | `B0` | `B_slosh` |
| solver state | 23D | 27D |
| liquid state/cost | 关闭，`w_slosh=0` | I0，`w_slosh=5`，完整时域 |
| observer | processed-IMU（录制/健康门） | processed-IMU I0（solver 输入） |
| fallback | I0 异常则该包后验无效 | `fail_closed` 实时停车 |
| common epoch | 液体 N/A；actual 前推到求解时刻 | odom/I0 对齐后再前推到求解时刻 |
| legacy delay phase | `off`，不得应用 | `off`，不得应用 |
| execution model | `explicit_actuator` | `explicit_actuator` |
| `v_ref / v_safe_max` | `0.20 / 0.25 m/s` | `0.20 / 0.25 m/s` |
| horizon | `N=60, dt约33.3 ms` | 同左 |

固定执行器参数：

```text
linear:  L=0.1666666665 s, tau=0.112 s, K=1.018, FIFO=5
angular: L=0.3333333330 s, tau=0.119 s, K=1.096, FIFO=10
```

路径、地图、RGB 标定和 70 s 录制窗沿用上一版 ABBA 的冻结资产。B0 不消费液体代价，但同样录制 processed-IMU、RGB 和执行器诊断，包不通过健康门时不得参与比较。

## 顺序与录制

```text
Row01  B0
Row02  Bslosh
Row03  Bslosh
Row04  B0
```

新入口只是薄 profile wrapper：

```text
src/scout_apps/control/spmpc_local_planner/scripts/
run_spmpc_i0_failclosed_explicit_actuator_abba_trial.sh
```

它继续复用公共 ABBA engine 和已有 `record_spmpc_full_rgb_bag.sh`，不另建录包流程。默认输出：

```text
/home/geist/slosh_bags/real/
20260903_spmpc_i0_failclosed_explicit_actuator_abba_v1/H0
```

## 开始门与有效性门

开始前必须满足：

- 定位、RealSense 和 NOKOV 已运行，场地清空且急停可用；
- 当前 runtime 源码已提交/冻结，Git-clean 门通过；
- `/cmd_vel` 没有其他 publisher；
- processed-IMU READY，路径、地图与 RGB 标定哈希通过；
- `VALIDATE_ONLY` 通过后才允许设置三个运动确认变量。

每包 postflight 必须确认：

- execution model code 为 `1`，legacy L22 的机器人/液体 application 均为 false；
- B0/Bslosh state width 分别为 `23/27`，FIFO 为 `5/10`；
- `L/tau/K` 与冻结值一致，actual/command 诊断完整；
- solver command 等于 horizon `x1.command`，无 post-solver limiter 改写；
- observer、common epoch、NOKOV、RGB、求解、跟踪和安全门全部有效。

## 执行命令

先做无运动检查：

```bash
cd /home/geist/scout_ws
PAIR_ROW=01 VALIDATE_ONLY=true \
  bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_i0_failclosed_explicit_actuator_abba_trial.sh
```

实车按 Row01 到 Row04 逐条运行，只修改 `PAIR_ROW`：

```bash
PAIR_ROW=01 VALIDATE_ONLY=false ARM_MOTION=YES \
CONFIRM_RGB_GEOMETRY=YES CONFIRM_NEW_SPEED_PROFILE=YES \
  bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_i0_failclosed_explicit_actuator_abba_trial.sh
```

Row02 完成后，脚本自动分析 Block 1。只有输出 `PROMOTE_BLOCK2` 才继续 Row03、Row04；若输出 `STOP_BLOCK1_FUTILITY`，立即停止。每行之间回到同一起点并等待液体稳定。

完整 ABBA 的正向门仍要求两个 block 的 RGB `DeltaP95` 同向、平均改善至少 `0.05 mm`、平均 `DeltaRMS >= 0`，且到点时间比不超过 `1.05`。这些阈值只用于 development 决策；最终仍以实际报告为准。

## 预注册边界与执行更新

方案冻结时，四行 `VALIDATE_ONLY`、232 项 catkin 测试和无运动启动检查已通过，实车 bag 尚未录制。若 actual 预测不匹配 NOKOV/odom，先停止并修正执行器参数，不用增大 `w_slosh` 掩盖结构误差。

随后完成 Row01/Row02。Row01 全部后验通过；Row02 到达终点但主运行时后验失败，已按有效性门停止 Block 1。当前两个 bag 足够定位卡顿机制，但不能据此声明 `B_slosh` 降晃或计算预注册改善量。

实施说明见 [I0 与显式执行器 OCP 最小修复方案](../解决问题的思路/20260903_I0与显式执行器OCP最小修复方案.md)。
