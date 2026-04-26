# 2026-04-24 Anti-Slosh MPC 结构改进方案

更新时间：2026-04-26

## 0. 当前结论

把液体晃动状态融入 MPC 的方向仍然成立，但当前实现还不是有效的 anti-slosh MPC。

当前版本更准确的定位是：

```text
slosh-aware tracking MPC
```

它能感知并预测 `/slosh/height`，但没有稳定降低液体模态能量。

最新仿真三包结果（P2_s_curve 近似同一路径，主段 `TRACKING && terminal/mode==NONE`）：

| 指标 | NOM | FAS_Q5 | FAS_Q5_TERM_f3 |
|---|---:|---:|---:|
| h_peak | 7.239 mm | 7.488 mm | 7.189 mm |
| h_p95 | 6.273 mm | 6.162 mm | 6.233 mm |
| h_rms | 3.130 mm | 3.039 mm | 3.165 mm |
| eta_rms | 1.719 mm | 1.669 mm | 1.738 mm |
| eta_dot_rms | 11.633 mm/s | 13.725 mm/s | 13.930 mm/s |
| ay_rms | 1.227 m/s² | 1.110 m/s² | 1.215 m/s² |
| omega_rms | 0.633 rad/s | 0.563 rad/s | 0.639 rad/s |

判断：

- `FAS_Q5` 的 h_rms 只下降约 2.9%，但 eta_dot_rms 升高约 18%。
- `FAS_Q5_TERM_f3` 的 h_peak 略低，但 h_rms 高于 NOM，eta_dot_rms 最高。
- 两个 anti-slosh 条件都没有同时压低 `h_rms / h_peak / eta_dot_rms`。

因此不能继续把主线放在单纯增大 `Q_slosh` 或 terminal factor 上。

## 1. 失败机制

### 1.1 `Q_slosh` 只惩罚 eta，容易变成调相位

当前 `Q_slosh` 本质上主要惩罚：

```text
eta_x^2 + eta_y^2
```

优化器可以让液体状态在某个预测时刻穿过零点，从而降低 eta 代价，但代价是 eta_dot 增大。

这不是阻尼，而是相位移动。

本次数据中：

```text
NOM eta_dot_rms    = 11.633 mm/s
FAS_Q5 eta_dot_rms = 13.725 mm/s
```

该数值来自 §9 近似同路径三包的严格主段口径：

```text
mpc_status == TRACKING 且 terminal/mode == NONE
```

FAS_Q5 相比 NOM 的 eta_dot_rms 升高约 18%。这是 P1 机制在更严格口径下的再现：η-only 代价没有抑制模态速度，反而让液体以更高模态速度振荡。

说明 `FAS_Q5` 没有把液体“冷静下来”。

### 1.2 terminal slosh cost 不能单独负责全程抑制

`FAS_Q5_TERM_f3` 增加了末端 eta 和 eta_dot 权重，但它仍然是 receding horizon 的移动终端。

它可能降低某个局部峰值，但无法保证整段路径的晃动能量下降。

本次数据中：

```text
NOM h_rms            = 3.130 mm
FAS_Q5_TERM_f3 h_rms = 3.165 mm
```

terminal cost 可以保留，但不能作为主机制。

### 1.3 当前结构没有直接约束晃动激励源

主要激励来自：

```text
ay ≈ v * omega
alpha = d(omega)/dt
jerk / d(ay)/dt
曲率突变处的速度
```

当前 slosh 代价主要惩罚液体状态，不直接约束产生晃动的运动激励。

结果是 MPC 仍然优先完成 tracking，只在相位上微调液体状态。

## 2. 设计原则

后续代码改动按整体结构推进，不做零散补丁。

原则：

- `Q_slosh=0` 行为必须保持不变。
- 新机制全部默认关闭，通过参数显式启用。
- 仿真和实物入口同步新增参数，避免两套实验口径分叉。
- 每一轮只引入一个主机制，便于消融。
- 不重写 MPC 求解器，不改变状态维度含义。
- 不把非线性 `/slosh/height = sqrt(eta_x^2 + eta_y^2) + parabola` 直接塞进 QP。
- 每次代码修改记录到 `docs/Claude/修改日志-时间/2026-04-24.md`。

## 3. 已完成但不再作为主线的机制

### P1：`Q_slosh_eta_dot`

状态：已实现，保留为消融项。

结论：

- 单独加 eta_dot 代价没有形成稳定阻尼。
- 部分情况下 h 指标下降，但 eta_dot_rms 上升，说明仍然是相位移动。

处理：

- 不删除代码。
- 不继续通过增大 `Q_slosh_eta_dot` 寻找效果。
- 后续只作为对照条件。

### P2：terminal slosh cost

状态：已实现，保留为消融项。

结论：

- factor=10 有 eta_dot 下降信号，但激励不可比。
- factor=3 在近似同路径测试中没有稳定降低 h_rms 和 eta_dot_rms。
- terminal cost 可以防止终点后残余晃动，但不能单独负责全程 anti-slosh。

处理：

- 不删除代码。
- 不继续盲目调大 terminal factor。
- 后续只作为辅助项，而不是主机制。

## 4. 下一版主线：激励源约束 + 模态能量判据

下一版目标：

```text
从“惩罚液体状态”转向“减少产生晃动的运动激励”
```

核心判断标准从只看 `h_peak` 改为：

```text
h_rms / h_p95 / h_peak
eta_dot_rms
modal_energy_rms = omega0^2 * eta^2 + eta_dot^2
ay_rms / omega_rms / alpha_rms
task_time / solve_success
```

只有 `h` 和 `modal_energy` 同时下降，才认为是真正 anti-slosh。

## 5. P3A：先补齐诊断，不改变控制行为

状态：已接入代码（2026-04-26），下一步用新 bag 验证 topic 与统计字段。

目的：

- 明确到底是液体状态下降，还是相位移动。
- 明确晃动来自 `ay`、`alpha` 还是曲率速度组合。

新增诊断 topic：

```text
/slosh/eta_norm
/slosh/eta_dot_norm
/slosh/modal_energy
/slosh/modal_energy_norm
/slosh/excitation_ay_abs
/slosh/excitation_alpha_abs
```

计算口径：

```text
eta_norm     = sqrt(eta_x^2 + eta_y^2)
eta_dot_norm = sqrt(eta_x_dot^2 + eta_y_dot^2)
modal_energy = omega0^2 * eta_norm^2 + eta_dot_norm^2
```

改动范围：

| 文件 | 修改内容 |
|---|---|
| `local_planner_ros.h` | 新增诊断 publisher 成员 |
| `local_planner_ros.cpp` | 在 `publishSloshDebug()` 中发布诊断 |
| `scripts/extract_slosh_metrics.py` 或新增脚本 | 离线统计 energy、eta_dot、激励指标 |

录包脚本同步加入上述 topic：

- `scripts/run_sim_fixed_path_bag.sh`
- `scripts/record_slosh_experiment.sh`
- `scripts/record_slosh_debug.sh`

默认行为：

- 只新增 topic。
- 不改变 MPC 输出。
- 不影响实物或仿真控制。

## 6. P3B/P3C：加入激励源抑制，而不是继续调 eta 权重

状态：

- P3B 纯曲率前馈 `slosh_speed_cap` 已验证不稳定，不作为主线继续调参。
- P3C `dkappa_only` 已接入代码（2026-04-26），默认关闭，下一步验证。

目的：

降低液体晃动的输入源，尤其是 S 弯中的横向加速度和角加速度激励。

优先顺序：

1. 先做速度规划层的 anti-slosh 限速。
2. 再考虑 MPC 代价中的激励项。
3. 最后才考虑 hard constraint。

### 6.1 P3B 结论：纯曲率限速不稳定

P2_s_curve 仿真重复验证结果：

```text
NOM:
tracking=8.60s, h_rms=2.552mm, eta_dot_rms=14.381mm/s, energy_norm=0.0461

SPEED_CAP run05/07/08 同参数均值:
tracking=9.52s (+10.7%)
h_rms=2.736mm (+7.2%)
h_peak=7.238mm (-0.2%)
eta_dot_rms=11.932mm/s (-17.0%)
energy_norm=0.0484 (+5.0%)
```

判断：

- 纯曲率限速可以降低 `eta_dot_rms`。
- 但 `h_rms` 和 `modal_energy` 没有稳定下降。
- `speed_cap_active_ratio` 仍接近 0.95，说明它仍然近似全程参与，只是降速幅度变小。
- 因此不能宣称 P3B 是有效 anti-slosh 结构。

### 6.2 P3C：只抑制换向/曲率变化激励

核心思想：

```text
不要限制“弯道中持续曲率 kappa”
改为限制“进入弯道/换向时的曲率变化 dkappa”
```

预期影响：

- 会让底盘在 S 弯方向切换、曲率快速变化段主动变慢。
- 不应让底盘在稳定弯道和直线段全程变慢。
- 如果改善主要来自全程速度下降，而不是高风险段选择性降速，则不能宣称 anti-slosh 结构成功。
- 因此 `task_time` 是硬约束，默认要求增加不超过 15%。

候选公式：

```text
v_limit_slosh_dkappa = sqrt(weight * alpha_max / max(abs(dkappa), eps))
v_ref = min(v_ref, gated(v_limit_slosh_dkappa))
```

建议参数：

```yaml
slosh_speed_cap:
  enable: false
  mode: dkappa_only
  dkappa_limit_weight: 0.15
  dkappa_threshold: 8.0
  min_v: 0.4
  preview_distance: 0.30
  activation_ratio: 0.9
  max_slowdown_ratio: 0.8
```

已接入的 debug topic：

```text
/slosh/speed_cap_active
/slosh/speed_cap_v_limit
```

实现位置：

| 文件 | 修改内容 |
|---|---|
| `local_planner_ros.cpp` | `slosh_speed_cap/mode=dkappa_only` 时跳过纯 `kappa` 限速 |
| `mpc_params.yaml` / `mpc_params_sim.yaml` | 新增默认关闭参数 |
| `slosh_experiment.launch` / `slosh_experiment_sim.launch` | 暴露参数 |

成功标准：

- `speed_cap_active_ratio` 不应接近 1.0。
- `eta_dot_rms` 下降。
- `modal_energy_norm_rms` 下降。
- `h_rms` 或 `h_p95` 不恶化，最好下降。
- `task_time` 增加不超过 15%。
- `solve_success_ratio >= 0.97`。

### 6.3 MPC 代价层：激励项作为第二步

如果速度 cap 有效，再考虑在 MPC 代价中加入激励项。

候选项：

```text
Q_slosh_ay      * ay_pred^2
Q_slosh_alpha   * alpha_pred^2
Q_slosh_day     * d(ay_pred)^2
```

注意：

- 不要直接引入难以维护的非线性 QP 项。
- 第一版可以用近似项或已有控制变量代理。
- 先保证解释性和可消融性。

建议参数：

```yaml
mpc:
  Q_slosh_ay: 0.0
  Q_slosh_alpha: 0.0
  Q_slosh_day: 0.0
```

默认全部关闭。

## 7. P3D：peak-aware 机制暂缓

`/slosh/height_pred_max` 仍然有价值，但暂不作为下一步主线。

原因：

- peak 下降可能只是相位移动。
- 当前更需要先确认 modal energy 是否下降。
- 直接优化 peak 容易引入非线性和求解不稳定。

保留方向：

- 继续记录 `/slosh/height_pred_max`。
- 后续如有必要，再考虑 soft box constraint 或 predicted peak penalty。

## 8. P4：IMU ay 作为实物激励源

IMU ay 是实物 slosh 估计的关键，但不作为下一轮代码主线。

前置条件：

- IMU 外参和符号稳定。
- `imu_ay_tool.py analyze` 多包通过。
- 静止 bias ready ratio 正常。
- IMU ay 与 `v * omega` 在左右转符号上稳定一致。

启用方式仍应显式指定：

```bash
slosh_use_imu_lateral_accel:=true
slosh_imu_ay_scale:=<calibrated_value>
```

如果启用后效果变差，优先检查标定、外参和滤波，不先改 MPC 主结构。

## 9. 当前推荐代码顺序

下一轮不要继续只调 `Q_slosh`、`Q_slosh_eta_dot` 或 terminal factor。

推荐顺序：

1. P3A：新增 modal energy / eta_dot / excitation 诊断 topic 和离线统计。
2. 用当前三包和后续 bag 复算 `modal_energy_rms`，确认失败机制。
3. P3B：实现默认关闭的 `slosh_speed_cap_enable`。
4. 仿真测试 `NOM / FAS_Q5 / SPEED_CAP / FAS_Q5_SPEED_CAP`。
5. 如果 speed cap 能同时降低 `ay_rms / eta_dot_rms / h_rms`，再考虑进入实物小样本。
6. 若 speed cap 只靠明显减速取得改善，记录 task_time 代价，不直接宣称 anti-slosh 成功。

## 10. 成功判据

必要前提：

```text
solve_success_ratio >= 0.97
task_time 增加 <= 15%
终点正常到达
```

主成功判据：

```text
h_rms 下降 >= 8%
h_p95 下降 >= 8%
eta_dot_rms 不升高，最好下降 >= 5%
modal_energy_rms 下降 >= 8%
```

辅助判据：

```text
ay_rms 下降或不升高
omega_rms 下降或不升高
cmd/odom 跟踪不明显恶化
```

失败判据：

- 只降低 h_peak，但 h_rms 或 modal_energy 不降。
- h 指标下降，但 eta_dot_rms 明显升高。
- 通过明显减速获得改善，task_time 超过 15%。
- MPC 求解失败率升高。

## 11. 文件职责边界

| 文件 | 允许改动 | 不应改动 |
|---|---|---|
| `types.h` | 新增默认关闭参数 | 改状态维度含义 |
| `cost_function.cpp` | 新增可消融代价项 | 改 tracking cost 语义 |
| `local_planner_ros.cpp` | 读取参数、发布诊断、速度 cap 接入 | 重写主循环 |
| `mpc_params.yaml` | 新增默认关闭参数 | 改实物默认行为 |
| `mpc_params_sim.yaml` | 新增仿真默认关闭参数 | 隐式打开新机制 |
| `slosh_experiment.launch` | 暴露实验参数 | 改普通启动链路 |
| `slosh_experiment_sim.launch` | 暴露实验参数 | 和实物入口分叉 |
| `scripts/*analysis*.py` | 增加 energy/激励统计 | 改历史指标定义 |

## 12. 立即停止的调参方向

暂不继续：

- 盲目增大 `Q_slosh`。
- 盲目增大 `Q_slosh_eta_dot`。
- 盲目增大 `terminal_factor_slosh_eta/_dot`。
- 只根据 h_peak 判断成功。
- 在 IMU 标定未稳定前默认启用 IMU ay。

当前主线改为：

```text
先证明能降低激励源，再证明能降低模态能量，最后才看液面 peak。
```
