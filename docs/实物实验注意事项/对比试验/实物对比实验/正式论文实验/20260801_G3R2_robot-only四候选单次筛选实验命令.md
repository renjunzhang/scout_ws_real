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

## 2. 下一条：Row 02

回到同一起点、液体静稳并确认急停可用后：

```bash
DATE=20260801 G3R2_ROW=02 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r2_weight_screen_trial.sh
```

必须等到末尾出现：

```text
[G3R2-SCREEN] acquisition PASS
```

## 3. 后续顺序

上一条 PASS、车辆回位且液体重新静稳后，逐条执行：

```bash
DATE=20260801 G3R2_ROW=03 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r2_weight_screen_trial.sh
```

```bash
DATE=20260801 G3R2_ROW=04 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r2_weight_screen_trial.sh
```

```bash
DATE=20260801 G3R2_ROW=05 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r2_weight_screen_trial.sh
```

不要使用循环，不要跳行。每条 bag 上限 `70 s`，不录图像，只录在线 RGB 标量。wrapper 自动核对 RealSense、路径、基线证据、二进制、权重、IMU source 和 robot/liquid 分流。

Row 05 后自动生成：

```text
/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_weight_screen/H0/G3R2_WEIGHT_SCREEN_REPORT.json
/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_weight_screen/H0/G3R2_WEIGHT_SCREEN_METRICS.csv
```

完成 Row 05 后停止，先检查 `PROMOTE_FOR_PAIRED_CONFIRMATION`、`NO_PROMOTION` 或 `SCREEN_INVALID`，不能直接进入重复采集。
