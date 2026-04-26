# 2026-04-26 Anti-Slosh MPC 仿真验证与代码修改交接

## 项目背景

本仓库是 Scout Mini 机器人 MPC 局部规划与液体晃动抑制实验工程。核心任务是在不破坏路径跟踪、求解稳定性和实物可执行性的前提下，让 MPC 或其参考速度生成链路降低红色液体的晃动。

液面晃动目前有两类指标：

- `/slosh/height`：控制侧 slosh 模型估计的液面高度，单位 m。当前实验主要按该指标判断控制结构是否有效。
- 红色液体视觉 RGB 方法：实物侧用于离线估计液面变化，但仿真中无法直接观测真实视觉液面。

本轮重点在仿真环境中验证 Anti-Slosh 结构，而不是直接进行昂贵实物实验。仿真启动流程已支持 open 场景、固定终点模板路径、录 bag 和离线指标分析。

## 当前目标

本轮对话最终目标是验证并改进 MPC anti-slosh 结构，尤其是：

1. 证明旧的 `Q_slosh` / η-only 代价为什么不能稳定压低 `/slosh/height`。
2. 尝试从“激励源抑制”角度改进：先做速度参考层限速，再尝试换向/曲率变化限速。
3. 建立可重复的仿真录包流程，减少固定路径起点不对齐问题。
4. 将结论和代码变更记录到 `docs/Claude/修改日志-时间/2026-04-24.md` 和 Anti-Slosh 方案文档，方便后续继续。

## 已完成工作

### 事实：诊断 topic 已接入

新增并录制以下 slosh 诊断 topic，用于判断控制是否真的降低模态能量，而不是只移动峰值相位：

- `/slosh/eta_norm`
- `/slosh/eta_dot_norm`
- `/slosh/modal_energy`
- `/slosh/modal_energy_norm`
- `/slosh/excitation_ay_abs`
- `/slosh/excitation_alpha_abs`
- `/slosh/speed_cap_active`
- `/slosh/speed_cap_v_limit`

这些 topic 已进入录包脚本和 `extract_slosh_metrics.py` 分析输出。

### 事实：仿真录包脚本已改为支持固定终点生成路径

新增脚本：

```text
src/scout_apps/control/scout_local_planner/scripts/run_sim_fixed_path_bag.sh
```

该脚本当前是未跟踪文件，但已由 CMake 安装列表包含。它支持：

- `PATH_MODE=replay`：旧固定 JSON 路径 replay 行为。
- `PATH_MODE=template_goal`：启动模板路径生成器，从当前车体位姿到固定终点生成路径，再启动 MPC 和 rosbag。

默认固定终点：

```text
x=-3.6119120121002197
y=3.955589771270752
qz=0.9992705515413127
qw=0.03818854307669733
topic=/scout/goal
frame=map
```

### 事实：P3B 纯曲率限速已验证失败

P3B 第一版 `SPEED_CAP` 基于前方曲率 `kappa` 限制 `v_des_cmd`：

```text
v_cap_kappa = sqrt(ay_limit / max(abs(kappa), eps))
```

实验结果说明：

- 纯曲率限速可以降低 `eta_dot_rms`。
- 但无法稳定降低 `/slosh/height` 或 `modal_energy_norm`。
- `speed_cap_active_ratio` 长期接近 0.95，说明它近似全程参与，不是选择性抑制高风险段。
- 因此不能作为有效 anti-slosh 结构。

关键数据，P2_s_curve：

```text
NOM:
tracking=8.60s, h_rms=2.552mm, h_peak=7.253mm,
eta_dot_rms=14.381mm/s, energy_norm=0.0461

SPEED_CAP run05/07/08 同参数均值:
tracking=9.52s (+10.7%)
h_rms=2.736mm (+7.2%)
h_peak=7.238mm (-0.2%)
eta_dot_rms=11.932mm/s (-17.0%)
energy_norm=0.0484 (+5.0%)
```

结论：`eta_dot_rms` 改善但能量和液面不稳定，不能验收。

### 事实：P3B.2 门控裁剪已实现但仍失败

为避免“慢速通过伪改善”，给 `slosh_speed_cap` 增加：

```yaml
activation_ratio: 0.9
max_slowdown_ratio: 0.75
```

含义：

- 只有几何 cap 明显低于当前 `v_des_cmd` 才触发。
- 单次裁剪不低于当前 `v_des_cmd` 的指定比例。

该修改把 tracking 时间拉回可接受区间，但重复验证仍不稳定。

### 事实：P3C `dkappa_only` 已实现并验证失败

新增：

```yaml
slosh_speed_cap:
  mode: curvature
  mode: dkappa_only
```

`dkappa_only` 跳过纯曲率限速，只按前方曲率变化率 `dkappa` 限速。

首轮无阈值结果：

```text
DKAPPA run01:
tracking=9.95s, h_rms=2.852mm, h_peak=6.285mm,
eta_dot_rms=10.443mm/s, energy_norm=0.0500, speed_cap_ratio=0.955

DKAPPA run02:
tracking=8.55s, h_rms=2.778mm, h_peak=7.744mm,
eta_dot_rms=15.932mm/s, energy_norm=0.0503, speed_cap_ratio=0.948
```

结论：仍几乎全程触发，且能量更差。

### 事实：P3C.2 `dkappa_threshold` 和短 preview 已实现并验证失败

新增：

```yaml
slosh_speed_cap:
  dkappa_threshold: 0.0
```

默认 `0.0` 保持旧行为。`DKAPPA_CAP / FAS_Q5_DKAPPA_CAP` 脚本默认改为：

```text
mode=dkappa_only
dkappa_limit_weight=0.15
dkappa_threshold=8.0
preview_distance=0.30
activation_ratio=0.9
max_slowdown_ratio=0.80
```

P3C.2 结果：

```text
DKAPPA run03:
tracking=9.55s, h_rms=3.111mm, h_peak=9.578mm,
eta_dot_rms=15.763mm/s, energy_norm=0.0557, speed_cap_ratio=0.787

DKAPPA run04:
tracking=9.50s, h_rms=3.096mm, h_peak=8.279mm,
eta_dot_rms=11.771mm/s, energy_norm=0.0544, speed_cap_ratio=0.778
```

结论：阈值确实降低触发比例，但 `/slosh/height` 和 `modal_energy_norm` 均变差。不要继续在 `dkappa_threshold / dkappa_limit_weight` 上盲调。

### 决策

当前决策是：停止继续调 `SPEED_CAP / DKAPPA_CAP` 的速度参考裁剪参数。下一步应转向更直接的控制变化率或激励平滑结构：

- 增强 `R_domega / R_da` 或相关代价，抑制控制突变。
- 对 `omega_ref / cmd_vel.angular.z` 做 jerk/变化率限制。
- 在 MPC cost 中引入 `alpha_z`、`delta_omega` 或激励变化项，而不是继续裁剪速度参考。

## 涉及文件

### `src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp`

修改内容：

- 读取 `mpc/Q_slosh_eta_dot`、`terminal_factor_slosh_eta`、`terminal_factor_slosh_eta_dot`。
- 发布新增 slosh 诊断 topic。
- 增加 `slosh_speed_cap` 参数读取：
  - `enable`
  - `mode`
  - `ay_limit`
  - `dkappa_limit_weight`
  - `dkappa_threshold`
  - `min_v`
  - `preview_distance`
  - `activation_ratio`
  - `max_slowdown_ratio`
- 在 reference point 生成前对 `v_des_cmd` 做可选 speed cap。
- `mode=dkappa_only` 时跳过纯 `kappa` 分支。
- `dkappa_preview > dkappa_threshold` 才启用 dkappa cap。
- 发布 `/slosh/speed_cap_active` 和 `/slosh/speed_cap_v_limit`。

修改目的：

- 支持 P3B/P3C 速度参考层激励源抑制实验。
- 支持离线判断 modal energy、激励和限速触发行为。

当前判断：

- 代码可编译。
- 功能可运行。
- 但 P3B/P3C 实验结果不支持继续调参。

### `src/scout_apps/control/scout_local_planner/include/scout_local_planner/local_planner_ros.h`

修改内容：

- 新增 `slosh_speed_cap_*` 成员变量。
- 新增 slosh 诊断 publisher 成员。

修改目的：

- 保存 P3B/P3C 参数和 debug topic publisher。

### `src/scout_apps/control/scout_local_planner/include/scout_local_planner/types.h`

修改内容：

- 增加 `Q_slosh_eta_dot`、terminal factor 等 MPC 参数字段。

修改目的：

- 支持 P1/P2 中 ηdot 代价和 terminal slosh 代价。

### `src/scout_apps/control/scout_local_planner/src/cost_function.cpp`

修改内容：

- 接入 `Q_slosh_eta_dot` 和 terminal slosh 相关代价。

修改目的：

- 支持早前 P1/P2 实验。

当前判断：

- P1/P2 结果未能稳定证明 η-only 或 terminal slosh 代价有效。
- 后续不建议继续单纯加大 slosh 权重。

### `src/scout_apps/control/scout_local_planner/config/mpc_params.yaml`

修改内容：

- 新增 slosh 代价参数。
- 新增 `slosh_speed_cap` 参数块：
  - `enable`
  - `mode`
  - `ay_limit`
  - `dkappa_limit_weight`
  - `dkappa_threshold`
  - `min_v`
  - `preview_distance`
  - `activation_ratio`
  - `max_slowdown_ratio`

修改目的：

- 实物配置默认保持关闭，保证旧行为不变。

### `src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml`

修改内容：

- 同步新增 slosh 代价参数和 `slosh_speed_cap` 参数。
- 该文件还有较多仿真参数改动，例如速度、角速度、路径容差、安全保护等。

修改目的：

- 支持仿真调参和 P2/P3 实验。

注意：

- 该文件的 diff 包含早前仿真调参，不全是最后一轮 P3C 修改。
- Claude 接手前应谨慎 review，不要无意回退。

### `src/scout_apps/control/scout_local_planner/launch/slosh_experiment.launch`

修改内容：

- 暴露 slosh 代价和 speed cap 相关 launch arg。
- 增加 `slosh_speed_cap_mode`、`slosh_speed_cap_dkappa_threshold`。

修改目的：

- 实物实验可通过 launch 参数显式开启/关闭新功能。

默认行为：

- `slosh_speed_cap_enable=false`，不开启时旧行为不变。

### `src/scout_apps/control/scout_local_planner/launch/slosh_experiment_sim.launch`

修改内容：

- 同步暴露仿真 launch 参数：
  - `Q_slosh_eta_dot`
  - terminal slosh factors
  - speed cap mode / threshold / gate 参数

修改目的：

- 支持一条命令切换 NOM、FAS_Q5、SPEED_CAP、DKAPPA_CAP 等条件。

### `src/scout_apps/control/scout_local_planner/scripts/run_sim_fixed_path_bag.sh`

状态：

- 未跟踪文件。
- 已通过 `bash -n`。
- 已加入 CMake 安装列表。

修改内容：

- 一键录制单包仿真实验。
- 支持 `PATH_MODE=replay` 和 `PATH_MODE=template_goal`。
- 支持条件：
  - `NOM`
  - `FAS_Q5`
  - `FAS_Q5_DOT`
  - `FAS_Q10`
  - `FAS_Q5_TERM`
  - `SPEED_CAP`
  - `FAS_Q5_SPEED_CAP`
  - `DKAPPA_CAP`
  - `FAS_Q5_DKAPPA_CAP`
  - `PROP_Q5`
  - `ISR`
  - `CUSTOM`
- 录制关键 topic，包括 slosh、MPC、路径、odom、cmd_vel、tf。

修改目的：

- 避免固定路径 replay 起点不对齐。
- 每次只改环境变量即可录制一包。

### `src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py`

修改内容：

- 输出 `eta_norm_rms`、`eta_dot_norm_rms_mps`、`modal_energy_norm_rms`。
- 输出 `speed_cap_active_ratio`、`speed_cap_v_limit_mean/min`。
- 默认按 TRACKING 段统计。

修改目的：

- 离线比较 anti-slosh 结构是否真的降低模态能量。

### `src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh`

修改内容：

- 录制新增 slosh debug topic。

### `src/scout_apps/control/scout_local_planner/scripts/record_slosh_debug.sh`

修改内容：

- 录制新增 slosh debug topic。

### `src/scout_apps/control/scout_local_planner/scripts/template_fixed_path_generator.py`

修改内容：

- 支持按当前车体 heading 生成模板路径。
- `--publish-count 0` 表示持续发布直到 Ctrl-C。

修改目的：

- 配合 `PATH_MODE=template_goal`，从当前位姿生成路径，减少起点对齐误差。

### `src/scout_apps/control/scout_local_planner/scripts/fixed_global_path_runner.py`

修改内容：

- 早前用于固定路径 replay / goal_only / 起点等待等流程。

当前判断：

- 由于用户后续改用固定终点生成模板路径，起点对齐相关逻辑不是当前主线。

### `src/scout_apps/control/scout_local_planner/scripts/launch_sim_nav_stack.sh`

修改内容：

- 支持仿真 open 场景切换相关流程。

### `src/scout_apps/control/scout_local_planner/CMakeLists.txt`

修改内容：

- 安装 `run_sim_fixed_path_bag.sh`。

### `docs/Claude/修改日志-时间/2026-04-24.md`

修改内容：

- 记录 P3A/P3B/P3C 代码修改、验证命令、bag 分析结果和失败结论。

### `docs/Claude/修改方案-时间-简介/2026-04-24_Anti-Slosh_MPC结构改进方案.md`

状态：

- 未跟踪文件。

修改内容：

- 记录 Anti-Slosh MPC 结构改进方案。
- 已更新 P3B 失败结论和 P3C 失败结论。
- 当前建议转向控制变化率/激励平滑，而不是继续速度参考裁剪。

### `docs/Claude/CLAUDE.md`

状态：

- 未跟踪文件。

内容要点：

- 修改前先明确假设和成功标准。
- 简洁优先，不做推测性实现。
- 外科手术式修改，不顺手重构。
- 每次改动都要验证并记录。

### 其他修改文件

当前 `git status` 还显示：

- `docs/重要文档/change_log.md`
- `src/scout_apps/control/scout_local_planner/scripts/README.md`
- `src/scout_apps/sensors/nanoscan3_localization/rviz/cartographer_localization.rviz`
- `docs/Claude/遇到的问题与解决方案/2026-04-25_仿真固定路径测试方案.md`

这些主要来自本轮或前序仿真流程整理，不是 P3B/P3C 核心逻辑。提交前需统一 review。

## 当前代码状态

### 已可用

- `catkin_make --pkg scout_local_planner` 通过。
- `run_sim_fixed_path_bag.sh` 语法检查通过。
- `git diff --check` 通过。
- `PATH_MODE=template_goal` 可用于从当前位姿到固定终点生成模板路径并录包。
- 新增 slosh debug topic 可进入 bag，并被 `extract_slosh_metrics.py` 读取。

### 不确定

- `slosh_speed_cap` 结构虽然能运行，但实验结果不支持继续作为主线。
- P3C.2 的 `dkappa_threshold=8.0` 和 `preview_distance=0.30` 只是验证点，不是推荐最终参数。
- P3B/P3C 都没有达到“稳定降低 `/slosh/height` 和 `modal_energy_norm`”的验收标准。

### 不应继续投入的方向

- 不建议继续盲调 `ay_limit`、`dkappa_limit_weight`、`dkappa_threshold`。
- 不建议继续单纯加大 `Q_slosh` 或 terminal factor。
- 不建议用“任务时间大幅变慢”作为 anti-slosh 成功依据。

## 运行过的命令

### 构建与静态检查

多次运行并通过：

```bash
bash -n src/scout_apps/control/scout_local_planner/scripts/run_sim_fixed_path_bag.sh
catkin_make --pkg scout_local_planner
git diff --check
```

结果：

- `bash -n` 通过。
- `catkin_make --pkg scout_local_planner` 通过，生成 `local_planner_node`。
- `git diff --check` 通过。

### 离线 bag 指标提取

使用：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  --csv /tmp/p2_speedcap_candidate_20260426.csv \
  /data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_NOM_run01_182713.bag \
  /data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run05_184720.bag \
  /data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run07_205414.bag \
  /data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_SPEED_CAP_run08_205559.bag
```

使用：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  --csv /tmp/p2_dkappa_cap_20260426.csv \
  /data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_NOM_run01_182713.bag \
  /data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run01_210633.bag \
  /data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run02_210807.bag
```

使用：

```bash
python3 src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  --csv /tmp/p2_dkappa_cap_p3c2_20260426.csv \
  /data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_NOM_run01_182713.bag \
  /data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run03_212051.bag \
  /data/a/slosh_bags/sim/20260426/20260426_P2_s_curve_DKAPPA_CAP_run04_212217.bag
```

结果：

- 见 `docs/Claude/修改日志-时间/2026-04-24.md` 第 13、14、15 节。

### 推荐但未继续执行的下一步命令

当前不建议继续执行 `DKAPPA_CAP` 调参命令，除非 Claude 明确要复现实验。

如需复现 P3C.2：

```bash
PATH_MODE=template_goal PATH_ID=P2_s_curve CONDITION=DKAPPA_CAP RUN_ID=03 \
  SLOSH_SPEED_CAP_DKAPPA_LIMIT_WEIGHT=0.15 \
  SLOSH_SPEED_CAP_DKAPPA_THRESHOLD=8.0 \
  SLOSH_SPEED_CAP_PREVIEW_DISTANCE=0.30 \
  SLOSH_SPEED_CAP_ACTIVATION_RATIO=0.9 \
  SLOSH_SPEED_CAP_MAX_SLOWDOWN_RATIO=0.80 \
  START_DELAY=30 APPROACH_START_ENABLE=false \
  rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

## 重要约束

### 代码风格约束

来自 `docs/Claude/CLAUDE.md`：

- 修改前明确假设、成功标准和计划。
- 只改必须改的地方。
- 不做推测性抽象。
- 不顺手重构无关代码。
- 每个改动都应有可追溯原因。
- 修改后必须验证并记录日志。

### 接口兼容性

- `slosh_speed_cap/enable=false` 时必须保持旧行为。
- `mode=curvature` 应保持旧 P3B 行为。
- 新功能必须通过 launch 参数显式开启。
- 不应破坏实物实验默认启动参数。
- 不应让默认 `NOM` 行为混入 anti-slosh 限速。

### 实验约束

- 实物实验成本高，仿真先筛掉明显无效结构。
- 成功不能只看 `eta_dot_rms`，必须同时看 `/slosh/height` 和 `modal_energy_norm`。
- 任务时间增加应尽量不超过 15%。
- 若改善来自明显慢速通过，不能宣称 anti-slosh 控制有效。

### 平台依赖

- ROS Noetic。
- catkin workspace: `/home/a/scout_ws`。
- bag 路径主要在 `/data/a/slosh_bags/sim/20260426`。
- 固定路径路径主要在 `/data/a/fixed_paths/sim`。

## 已知问题

### 事实

- 当前 `SPEED_CAP` 和 `DKAPPA_CAP` 均未达到验收标准。
- `speed_cap_active_ratio` 在很多实验中仍偏高。
- 即使 `dkappa_threshold` 降低触发比例，液面和模态能量仍变差。
- 当前速度参考裁剪会改变通过相位，可能把液面峰值推高。

### 推测

- `/slosh/height` 对激励相位非常敏感，简单速度裁剪可能改变相位而不是降低能量。
- 速度层限速不是足够直接的 anti-slosh 手段，应该转向控制变化率/激励变化率约束。
- `Q_slosh` 直接惩罚 η 可能导致优化器选择更大转速来改变相位，因此不是稳定主线。

### 待验证事项

- 控制变化率代价或约束是否比速度参考裁剪更稳定。
- `R_domega / R_da` 增强是否能降低 `modal_energy_norm`，同时保持路径跟踪和任务时间。
- 是否应加入 `alpha_z`、`delta_omega`、`d(ay)/dt` 代理项。
- 是否需要更严格地截断终点后 `SETTLING/REACHED` 反复切换段，目前主要按 TRACKING 段统计。

## 下一步建议

### 优先级 1：停止 P3B/P3C 调参，转向控制变化率抑制

建议 Claude 先 review 当前 `cost_function.cpp`、`types.h`、`mpc_params*.yaml` 中已有的 `R_domega`、`R_da`、控制变化率相关项。

目标是形成 P4 或 P3D 方案：

```text
抑制控制突变 / 角速度变化 / 横向激励变化
而不是继续裁剪参考速度
```

可选方向：

- 增大 `mpc/R_domega`。
- 增大 `mpc/R_da`。
- 新增 `Q_alpha_z` 或 `Q_delta_omega` 代理项。
- 对 `cmd_vel.angular.z` 或参考 `omega` 做 jerk 限制，但要谨慎避免跟踪变差。

### 优先级 2：整理并提交当前可保留代码

提交前必须 review：

- 未跟踪文件是否全部应加入 git：
  - `docs/Claude/CLAUDE.md`
  - `docs/Claude/修改方案-时间-简介/2026-04-24_Anti-Slosh_MPC结构改进方案.md`
  - `docs/Claude/遇到的问题与解决方案/2026-04-25_仿真固定路径测试方案.md`
  - `src/scout_apps/control/scout_local_planner/scripts/run_sim_fixed_path_bag.sh`
- `mpc_params_sim.yaml` 中仿真参数是否确实要保留。
- RViz 文件改动是否属于本次提交。

建议拆 commit：

1. 仿真环境/路径生成/录包脚本。
2. slosh 诊断 topic 与指标提取。
3. P3B/P3C speed cap 实验代码与失败结论文档。

### 优先级 3：更新分析报告

如需对外汇报，应明确写：

- P3B/P3C 是失败验证，不是最终成功方案。
- 其价值是排除了“速度参考裁剪即可稳定抑制晃动”的假设。
- 下一步转向控制平滑/激励变化率抑制。

## 验收标准

一个后续 anti-slosh 方案只有满足以下条件，才算完成：

- 同一路径至少 3 次重复。
- `tracking_time` 相对 NOM 增加不超过 15%。
- `solve_success_ratio >= 0.97`。
- `h_rms` 不高于 NOM，最好下降。
- `h_peak` 或 `h_p95` 不显著恶化。
- `eta_dot_rms` 下降。
- `modal_energy_norm_rms` 下降。
- `speed_cap_active_ratio` 或其他抑制器 active ratio 不能接近全程，除非能证明不是慢速通过。
- 结论必须基于均值和离散度，而不是单个 bag。

## 未提交 diff 总结

### `git diff --stat`

```text
 .../2026-04-24.md"                                 | 975 +++++++++++++++++++++
 .../change_log.md"                                 |   2 +-
 .../control/scout_local_planner/CMakeLists.txt     |   1 +
 .../scout_local_planner/config/mpc_params.yaml     |  17 +
 .../scout_local_planner/config/mpc_params_sim.yaml |  63 +-
 .../scout_local_planner/local_planner_ros.h        |  21 +
 .../include/scout_local_planner/types.h            |   5 +
 .../launch/slosh_experiment.launch                 |  20 +
 .../launch/slosh_experiment_sim.launch             |  24 +
 .../control/scout_local_planner/scripts/README.md  |  38 +
 .../scripts/extract_slosh_metrics.py               |  40 +
 .../scripts/fixed_global_path_runner.py            |  18 +
 .../scripts/launch_sim_nav_stack.sh                |  16 +
 .../scripts/record_slosh_debug.sh                  |   9 +
 .../scripts/record_slosh_experiment.sh             |   8 +
 .../scripts/template_fixed_path_generator.py       |  77 +-
 .../scout_local_planner/src/cost_function.cpp      |  23 +
 .../scout_local_planner/src/local_planner_ros.cpp  | 152 +++-
 .../rviz/cartographer_localization.rviz            |  97 +-
 19 files changed, 1546 insertions(+), 60 deletions(-)
```

注意：以上 `git diff --stat` 不包含未跟踪文件。

### 未跟踪文件

```text
docs/Claude/CLAUDE.md
docs/Claude/修改方案-时间-简介/2026-04-24_Anti-Slosh_MPC结构改进方案.md
docs/Claude/遇到的问题与解决方案/2026-04-25_仿真固定路径测试方案.md
src/scout_apps/control/scout_local_planner/scripts/run_sim_fixed_path_bag.sh
docs/Claude/对话交接文档-时间-简介/2026-04-26_Anti-Slosh_P3B_P3C仿真验证交接.md
```

### 关键 diff 摘要

- `local_planner_ros.cpp/h`：新增 slosh 诊断 topic、speed cap 参数、P3B/P3C 限速逻辑。
- `mpc_params*.yaml`：新增 slosh 代价和 `slosh_speed_cap` 参数。
- `slosh_experiment*.launch`：透传新参数。
- `run_sim_fixed_path_bag.sh`：新增一键录包、固定终点模板路径、条件切换。
- `extract_slosh_metrics.py`：新增 modal energy、eta dot、speed cap 统计。
- 文档：记录大量 bag 结果和失败结论。

## 最后状态

事实：

- 代码构建通过。
- P3B/P3C 速度参考裁剪方向已被仿真结果基本否定。
- 当前最重要的工程资产是：诊断 topic、录包脚本、分析脚本和失败结论。

推测：

- 更有效方向可能是控制变化率/激励变化率抑制，而不是参考速度裁剪。

待验证：

- 新的控制平滑方案是否能同时降低 `/slosh/height`、`eta_dot_rms` 和 `modal_energy_norm_rms`。

## 交接补充：对 Claude 疑问的明确答复

### 1. Claude 接手后应先做什么

事实：

- 当前 P3B/P3C 已经有可保留代码、可复现实验脚本和失败结论。
- 当前工作区改动很多，且混有仿真流程、RViz、参数调试、文档和控制代码。

决策建议：

- 先不要实现 P4。
- 先做 review 和提交整理，把可保留资产稳定下来。
- 提交前至少拆成 2 到 3 个逻辑 commit，避免把无关仿真调参、RViz 布局和 anti-slosh 代码混成一个提交。

建议优先级：

```text
1. Review 未提交 diff，决定哪些进入本批 commit。
2. 提交录包/路径/诊断/分析脚本和 P3B/P3C 失败结论文档。
3. 之后再开新任务设计 P4 控制变化率/激励平滑方案。
```

### 2. 关于 R_domega / R_da 的事实修正

事实：

- `src/scout_apps/control/scout_local_planner/config/mpc_params.yaml` 当前：

```yaml
R_da: 0.5
R_domega: 4.0
```

- `src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml` 当前：

```yaml
R_da: 0.5
R_domega: 1.0
```

- `src/scout_apps/control/scout_local_planner/include/scout_local_planner/types.h` 编译默认值仍是：

```cpp
R_da = 0.1
R_domega = 0.1
```

结论：

- 交接文档中“增大 `R_domega / R_da`”只能作为方向，不应理解为直接继续加大实物 `R_domega=4.0`。
- 若做 P4，应先在仿真中基于 `mpc_params_sim.yaml` 的 `R_domega=1.0` 做小范围消融。
- 实物 `R_domega=4.0` 已经较高，继续加大有吃掉转向能力的风险。

### 3. 关于 P3C / P3C.2 只有两个 run

事实：

- P3B 的主要候选有 3 次重复，结论更稳。
- P3C 和 P3C.2 每个配置只有 2 次重复。
- P3C.2 两个 run 中 `h_rms`、`h_peak`、`modal_energy_norm` 均明显差于 NOM。

决策建议：

- 不必为了“证明失败”继续补第三个 run，除非要写正式报告或论文式统计。
- 当前两个 run 足够支持工程决策：停止沿 `dkappa_threshold / dkappa_limit_weight` 继续盲调。
- 若未来要严谨写报告，可以补 1 个 P3C.2 run，但这不是当前优先事项。

### 4. 关于 `mpc_params_sim.yaml` 的 63 行 diff

事实：

- 该文件的 diff 不只包含 P3B/P3C 必需参数。
- 它还包含早前仿真跟踪相关调参，例如 `Q_el/Q_ec/Q_etheta`、`v_max/omega_max/alpha_max/j_max`、`lookahead_distance`、goal tolerance、safety tracking guard 等。

建议提交策略：

- 不要在 anti-slosh 代码 commit 中混入整份 `mpc_params_sim.yaml` diff，除非用户明确接受。
- 推荐拆分：

```text
commit A: 仿真 MPC 参数调参与 open 场景运行稳定性
commit B: slosh 诊断 topic / 分析脚本 / 录包脚本
commit C: P3B/P3C speed cap 实验代码与失败结论文档
```

如果必须只做一个提交，则 commit message 必须明确包含“仿真参数同步/路径录制/anti-slosh 实验工具”多个范围，避免误导。

### 5. 关于 RViz 文件 diff

事实：

- `src/scout_apps/sensors/nanoscan3_localization/rviz/cartographer_localization.rviz` 的 diff 主要是 UI 展示状态、TF 展开、`Path` topic 从 `/scout/global_path` 改为 `/scout/global_path_fixed` 和窗口位置。
- 它与 anti-slosh 控制逻辑无关。

建议：

- 不要放入 anti-slosh 代码 commit。
- 可单独作为“仿真 RViz 显示固定路径”提交，或提交前排除。
- 不要未经确认直接 reset，因为这可能是用户调 RViz 时故意保存的布局。

### 6. 关于未跟踪文件归属

建议：

- `src/scout_apps/control/scout_local_planner/scripts/run_sim_fixed_path_bag.sh` 必须加入 git。它已被 `CMakeLists.txt` install 引用，不加入会导致别人拿不到脚本。
- `docs/Claude/CLAUDE.md` 建议加入 git，因为它是本工程后续修改准则。
- `docs/Claude/修改方案-时间-简介/2026-04-24_Anti-Slosh_MPC结构改进方案.md` 建议加入 git，因为它记录当前 anti-slosh 方案和失败结论。
- `docs/Claude/遇到的问题与解决方案/2026-04-25_仿真固定路径测试方案.md` 建议加入 git，因为它记录仿真固定路径测试流程。
- 本交接文档可加入 git，也可以作为本地交接资料；若团队习惯保留 Claude 交接文档，则加入。

### 7. 关于“速度裁剪只是改相位不降能量”

事实：

- 这是推测，不是已被频域分析证明的机理。
- 直接证据是：`eta_dot_rms` 有时下降，但 `/slosh/height` 和 `modal_energy_norm` 没有稳定下降。
- P3C.2 中触发比例下降后能量反而变差，说明“触发更选择性”本身也不是充分条件。

建议：

- 若要巩固机理，可以用已有 bag 做频域或相位分析，但这不是提交当前工具链和失败结论的前置条件。
- 进入 P4 时，不要把“active ratio 下降”作为成功标准；必须继续用 `h_rms/h_peak/eta_dot_rms/modal_energy_norm` 共同验收。

### 8. 关于固定终点是否适用于所有路径模板

事实：

- 当前固定终点 `(-3.6119, 3.9556)` 是 open 场地 P2_s_curve 验证时 RViz 选择的终点。
- 它不是所有路径模板的通用终点。
- `template_fixed_path_generator.py` 会根据 `--template` 和当前车体位姿到该终点的 chord 生成路径。

建议：

- 对 P2_s_curve，当前终点可继续复现实验。
- 若换 P0/P1/P3 或换场地，应重新在 RViz 中选终点，或显式覆盖：

```bash
TEMPLATE_GOAL_X=...
TEMPLATE_GOAL_Y=...
TEMPLATE_GOAL_QZ=...
TEMPLATE_GOAL_QW=...
```

### 9. 关于 `PATH_MODE=template_goal` 的路径形状

事实：

- `PATH_MODE=template_goal` 本身不是路径形状。
- 具体形状由 `PATH_ID` 映射到 `TEMPLATE_NAME` 决定。
- 当前映射：

```text
P0_straight    -> straight
P1_single_turn -> single_turn
P2_s_curve     -> s_curve
P3_mixed       -> mixed
```

P2_s_curve 的 `s_curve` 控制点逻辑来自 `template_fixed_path_generator.py`：

```text
(0,0) -> (0.25L,A) -> (0.50L,0) -> (0.75L,-A) -> (L,0)
```

结论：

- 当前 P3B/P3C 失败结论严格适用于 P2_s_curve。
- 不应直接外推到 P0/P1/P3，除非补对应路径实验。

### 10. 如何避免 P4 落入同样陷阱

事实：

- P3B/P3C 失败说明“看起来选择性抑制风险段”不足以证明 anti-slosh 有效。
- 后续 P4 必须继续按输出指标验收，而不是按触发比例或控制变平滑程度验收。

建议的 P4 验收逻辑：

```text
NOM vs P4 candidate，至少 3 次重复。
tracking_time <= NOM * 1.15。
solve_success_ratio >= 0.97。
h_rms 不升，h_peak/h_p95 不显著恶化。
eta_dot_rms 下降。
modal_energy_norm_rms 下降。
路径跟踪误差不显著恶化。
```

建议的 P4 工程策略：

- 第一轮只做参数消融，不先写新 cost：

```text
SMOOTH_CTRL_A: 仿真中小幅提高 R_domega
SMOOTH_CTRL_B: 小幅提高 R_da + R_domega
```

- 若参数消融有效，再考虑新代价项。
- 若参数消融无效，不要直接新增复杂项，应先分析控制序列、`ay_est`、`alpha_est` 与 slosh state 的相位关系。
