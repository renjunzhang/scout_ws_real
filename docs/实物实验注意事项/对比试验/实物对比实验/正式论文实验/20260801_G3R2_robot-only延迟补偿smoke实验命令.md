# G3R2 robot-only 延迟补偿 smoke 实验命令

## 1. 为什么先做这一条

G3R Row 01 已证明：把机器人状态和液体状态的延迟传播同时关闭，会造成明显路径偏离。该失败包起点误差约 `6 mm`，但运动段横向误差 P95 为 `0.292 m`、航向误差 P95 为 `0.638 rad`，首次到达约 `48.0 s`；因此不能继续原 G3R Row 02–05。

G3R2 只改一个状态契约：

- `SolverInput.robot`：使用 `0.15 s / 0.22 s` 命令历史预测后的机器人状态；
- `SolverInput.slosh`：使用当前 processed-IMU observer 状态；
- 禁止把 command-based liquid rollout 写入 solver；
- processed-IMU 失效时 fail-closed，不回退 odom；
- 模式名 `fixed_robot_only`，模式码 `4`。

这一版先只开放 **Bsmooth Row 01 smoke 一条**。它只验证 tracking 恢复和状态分流，不是液面抑制效果证据。

## 2. 前置条件

基础传感器栈保持运行：

```bash
bash /home/geist/scout_ws/src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh
```

小车回到冻结路径起点并对齐，液体静稳，相机、容器和三把标尺不能移动。wrapper 会再次应用并核对已冻结的 RealSense 参数，不需要另开相机录制；bag 禁止包含图像，只记录在线 RGB 标量。

## 3. 不动车检查

```bash
DATE=20260801 G3R2_ROW=01 VALIDATE_ONLY=true \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r2_robot_only_smoke_trial.sh
```

必须看到：

```text
row/condition = 01/Bsmooth
mode/code     = fixed_robot_only / 4
robot state   = fixed delay-predicted odom/TF
liquid state  = current processed-IMU (no delay rollout)
```

## 4. 唯一一条实车命令

确认周围安全、急停可用、起点正确后执行：

```bash
DATE=20260801 G3R2_ROW=01 ARM_MOTION=YES CONFIRM_RGB_GEOMETRY=YES \
bash /home/geist/scout_ws/src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r2_robot_only_smoke_trial.sh
```

输出固定为：

```text
/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_smoke/H0/
DEV_G3R2_H0_C1_Bsmooth_robot_only_r01_a01.bag
```

录包上限为 `70 s`。wrapper 自动等待 IMU READY、自动发布冻结路径、自动运动、退出时自动发零速，并执行 postflight。

## 5. PASS 门槛

- 首次 `GOAL_REACHED <= 42 s`；
- 横向误差 P95 `<= 0.05 m`；
- 航向误差 P95 `<= 0.15 rad`；
- 到达后有完整 `5 s` 在线 RGB 标量尾窗；
- processed-IMU source 与 READY 覆盖率均 `>= 98%`；
- robot delay application `>= 98%`；
- liquid delay application `<= 2%`；
- 无 fallback、无 reset、无安全停车异常；
- bag 内无图像话题。

脚本末尾只有出现以下信息才算通过：

```text
[G3R2] smoke PASS
```

完成后停止，不要手动运行旧 G3R Row 02–05。先冻结这条 smoke 的 postflight；只有 PASS 后，才生成新的候选权重 release。
