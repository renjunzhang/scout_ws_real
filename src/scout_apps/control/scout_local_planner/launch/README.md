# scout_local_planner launch 说明

本文档说明 `test_mpc.launch`、`test_mpc_sim.launch`、`slosh_experiment.launch` 三个启动文件的区别，以及推荐使用场景。

## 三者区别总表

| launch 文件 | 配置文件 | 默认场景 | 默认 `Q_slosh` | 额外实验参数覆盖 | 适用用途 | 推荐程度 |
|---|---|---|---:|---|---|---|
| `test_mpc.launch` | `config/mpc_params.yaml` | 实物 | `0.0` | 无 | 日常实物 MPC 跟踪、普通导航 | 实物日常使用推荐 |
| `test_mpc_sim.launch` | `config/mpc_params_sim.yaml` | 仿真 | `5.0` | 无 | 日常仿真 MPC 跟踪、快速验证 | 仿真日常使用推荐 |
| `slosh_experiment.launch` | `config/mpc_params.yaml` 或 `config/mpc_params_sim.yaml` | 实物/仿真（由 `sim` 决定） | `0.0` | 有，集中覆盖 anti-slosh 实验参数 | 液体晃动抑制实验、消融对比、参数扫描 | anti-slosh 实验推荐 |

## 关键差异

| 对比项 | `test_mpc.launch` | `test_mpc_sim.launch` | `slosh_experiment.launch` |
|---|---|---|---|
| 是否区分实物/仿真 | 只用于实物 | 只用于仿真 | 通过 `sim:=true/false` 切换 |
| 是否只暴露少量参数 | 是 | 是 | 否，集中暴露实验参数 |
| 是否默认关闭执行端额外 EMA | 否 | 否 | 是，默认 `filter/alpha_v=1.0`、`filter/alpha_omega=1.0`、`filter/kappa_boost=0.0` |
| 是否适合做 `Q_slosh` 消融 | 一般 | 一般 | 是 |
| 是否支持盒约束开关 | 需手动额外传参 | 需手动额外传参 | 直接支持 |
| 是否支持 speed governor 参数集中传入 | 需手动额外传参 | 需手动额外传参 | 直接支持 |
| 是否适合论文实验复现实验口径 | 不推荐 | 不推荐 | 推荐 |

## 为什么做液体晃动实验时优先用 `slosh_experiment.launch`

原因不是它“更高级”，而是它的职责更明确：

1. 它把 anti-slosh 相关参数集中暴露出来，避免每次手工拼很多 launch arg。
2. 它默认关闭实验中不希望引入的执行端额外 EMA，减少“模型外隐藏动态”。
3. 它统一了实验入口，便于 rosbag 录制、结果复现和消融分析。

因此：

- **普通 MPC 跟踪 / 日常导航**：优先用 `test_mpc.launch` 或 `test_mpc_sim.launch`
- **液体晃动抑制实验 / 消融分析**：优先用 `slosh_experiment.launch`

## 推荐启动方式

### 1. 实物日常跟踪

```bash
roslaunch scout_local_planner test_mpc.launch
```

### 2. 仿真日常跟踪

```bash
roslaunch scout_local_planner test_mpc_sim.launch
```

### 3. 实物液体晃动实验

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  Q_slosh:=5 \
  enable_slosh_box_constraint:=true \
  slosh_speed_governor_enable:=true
```

### 4. 仿真液体晃动实验

```bash
roslaunch scout_local_planner slosh_experiment.launch \
  sim:=true \
  Q_slosh:=5 \
  enable_slosh_box_constraint:=true \
  slosh_speed_governor_enable:=true
```

## `slosh_experiment.launch` 常用参数

| 参数 | 作用 | 常用值 |
|---|---|---|
| `sim` | 是否加载仿真参数文件 | `true / false` |
| `Q_slosh` | 晃动软代价权重 | `0 / 5 / 10 / 20` |
| `enable_slosh_box_constraint` | 是否启用第一版液面盒约束代理 | `true / false` |
| `slosh_speed_governor_enable` | 是否启用残余晃动感知速度治理 | `true / false` |
| `slosh_speed_governor_k_eta` | 液面高度比例缩放系数 | `2.5` |
| `slosh_speed_governor_eta_deadband` | 介入死区 | `0.3` |
| `slosh_speed_governor_eta_exit_ratio` | governor 退出阈值（滞回） | `0.2` |
| `slosh_speed_governor_min_active_steps` | 最少保持周期数 | `10` |
| `slosh_speed_governor_ay_max_base` | 横向加速度预算 | `0.6`（实验常用起点） |
| `slosh_speed_governor_v_des_min` | 调速后的最低参考速度 | `0.2` |
| `slosh_speed_governor_preview_distance` | 前方曲率预览长度 | `1.0` |

## 实际使用建议

- 如果你只是想确认 MPC 能不能跟踪路径，不要先上 `slosh_experiment.launch`。
- 如果你要录 bag、做 `Q=0/5/10` 对比，直接用 `slosh_experiment.launch`，不要混用 `test_mpc*.launch`。
- `test_mpc_sim.launch` 当前默认 `Q_slosh=5.0`，这更像“带一定 anti-slosh 倾向的仿真默认入口”，不是严格的消融基线。

