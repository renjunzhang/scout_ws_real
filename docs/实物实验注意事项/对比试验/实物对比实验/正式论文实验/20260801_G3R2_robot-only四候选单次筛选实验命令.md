# G3R2 robot-only 四候选单次筛选实验命令

## 1. 已冻结基线

不再重复采集 Bsmooth。筛选基线固定为已通过的 G3R2 smoke：

```text
bag SHA256        = 63399f5f6e80c9afe438e5dae65942c43545c4b2c54bffa866579d6a105820ba
postflight SHA256 = 9f645ae70b385d15935207d8b980bdd5a6ec5d1bcab2109056c5b3513a99e784
tracking P95      = 0.024 m / 0.075 rad
arrival           = 34.68 s
robot/liquid      = 100% / 0%
```

四个候选各做一次；只有出现正向候选，才另建 release 做配对重复。

| Row | 条件 | `w_slosh` | `w_smooth=w_alpha=w_du_a=w_du_vs` |
| --- | --- | ---: | ---: |
| 02 | W2_S03 | 2 | 0.3 |
| 03 | W5_S10 | 5 | 1.0 |
| 04 | W2_S10 | 2 | 1.0 |
| 05 | W5_S03 | 5 | 0.3 |

共同配置为 `fixed_robot_only`：机器人状态使用延迟预测，液体状态使用当前 processed-IMU，禁止 liquid delay rollout。

## 2. 当前进度与 Row 03 受控重录

Row 02 `W2_S03` 已 PASS。Row 03 attempt 01 的控制、processed-IMU、跟踪和求解均通过，只有 RealSense 源时间戳在启动阶段未收敛：

```text
online publish-lag P95       = 0.587 s  > 0.500 s
online source future skew    = 1.944 s  > 0.050 s
```

因此 attempt 01 固定归类为 `METHOD_INDEPENDENT_ACQUISITION`，保留 bag 和 postflight，但其 RGB 指标永久不能参与候选筛选。失败证据和唯一一次重录授权分别冻结在：

```text
20260801_G3R2_Row03_a01_相机时间戳采集失败证据.env
20260801_G3R2_Row03_a02_相机时间戳采集重试授权.env
```

Row 03 当时唯一合法的重录是保持 `W5_S10` 配置不变的 attempt 02，执行命令为：

```bash
DATE=20260801 G3R2_ROW=03 G3R2_ATTEMPT=02 \
ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r2_weight_screen_trial.sh
```

wrapper 会在运动前两次检查 `/camera/color/camera_info`，分别位于在线 RGB 零点建立之前和录包之前。每次会在不动车的状态下等待连续 90 帧健康时间戳，最长 50 秒；到期仍不通过才会 fail-closed 停止。只有该 gate 最终返回 `PASS` 后流程才会继续。

必须等到末尾出现：

```text
[G3R2-SCREEN] acquisition PASS
```

## 3. Row 04 方法失败与 Row 05 继续授权

Row 03 attempt 02 已 PASS。Row 04 `W2_S10` 的采集、RGB、processed-IMU 和时间戳均正常，但 postflight 因跟踪门槛失败：

```text
stage-0 contour P95 = 0.053296 m > 0.050000 m
```

该结果固定归类为 `METHOD_PERFORMANCE_FAILURE`：保留为不合格候选，不允许重录，也不能参与晋级。证据和继续筛选授权冻结在：

```text
20260801_G3R2_Row04_a01_跟踪门槛方法失败证据.env
20260801_G3R2_Row05_方法失败后继续筛选授权.env
```

车辆回位、液体重新静稳后，唯一下一条为 Row 05 `W5_S03`：

```bash
DATE=20260801 G3R2_ROW=05 G3R2_ATTEMPT=01 \
ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r2_weight_screen_trial.sh
```

不要使用循环，不要跳行。每条 bag 上限 `70 s`，不录图像，只录在线 RGB 标量。wrapper 自动核对 RealSense、路径、基线证据、二进制、权重、IMU source 和 robot/liquid 分流。

Row 05 wrapper 会同时核对 Row 03 retry 和 Row 04 方法失败授权。最终分析器会排除 Row 03 attempt 01 的采集故障，并把 Row 04 保留在数据集中但标记为 `eligible_for_selection=false`。

Row 05 后自动生成：

```text
/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_weight_screen/H0/G3R2_WEIGHT_SCREEN_REPORT.json
/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_weight_screen/H0/G3R2_WEIGHT_SCREEN_METRICS.csv
```

完成 Row 05 后停止，先检查 `PROMOTE_FOR_PAIRED_CONFIRMATION`、`NO_PROMOTION` 或 `SCREEN_INVALID`，不能直接进入重复采集。

## 4. 筛选完成结论

Row 05 已 PASS，完整报告为 `PROMOTE_FOR_PAIRED_CONFIRMATION`，唯一晋级候选是 `W5_S10`。后续三配对确认已单独冻结在：

```text
20260801_G3R2_W5_S10三配对确认实验命令.md
```
